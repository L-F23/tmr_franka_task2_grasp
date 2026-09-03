#!/usr/bin/env python3
"""Compute a fail-closed left-arm thermal-pad grasp plan; never moves hardware."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from franka_msgs.msg import FrankaRobotState
from franka_spine_msgs.srv import GetPosition
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from realsense2_camera_msgs.msg import Extrinsics
from sensor_msgs.msg import CameraInfo, Image, JointState

from thermal_pad_geometry import (
    Intrinsics,
    detect_pad_end,
    pose_matrix,
    register_depth_point,
    transform_point,
)
from thermal_pad_sequence import (
    build_sequence,
    horizontal_gripper_orientation,
    quaternion_matrix,
    slerp,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "thermal_pad_pick.json"
DEFAULT_RECORD = ROOT / "config" / "latest_thermal_pad_ik.json"
JOINT_NAMES = [f"left_fr3v2_joint{i}" for i in range(1, 8)]
SPINE_JOINT_NAME = "franka_spine_vertical_joint"


class PlanningError(RuntimeError):
    pass


def stamp_s(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def quaternion_angle_deg(a, b) -> float:
    qa = np.asarray(a, dtype=float)
    qb = np.asarray(b, dtype=float)
    qa /= np.linalg.norm(qa)
    qb /= np.linalg.norm(qb)
    return math.degrees(2.0 * math.acos(float(np.clip(abs(np.dot(qa, qb)), -1.0, 1.0))))


def pose_values(pose) -> tuple[list[float], list[float]]:
    return (
        [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
        [float(pose.orientation.x), float(pose.orientation.y),
         float(pose.orientation.z), float(pose.orientation.w)],
    )


def interpolate(a: np.ndarray, b: np.ndarray, samples: int):
    for index in range(1, samples + 1):
        yield a + (b - a) * (index / samples)


class ThermalPadPlanner(Node):
    def __init__(self, config: dict):
        super().__init__("thermal_pad_fk_ik_planner")
        self.cfg = config
        self.bridge = CvBridge()
        self.color = self.depth = None
        self.color_stamp = self.depth_stamp = None
        self.color_info = self.depth_info = self.extrinsics = None
        self.joints = self.robot_state = self.measured_pose = None
        self.spine_position = None
        self.odom = deque(maxlen=400)
        camera = config["camera"]
        stationary = config["base_stationary_gate"]
        self.create_subscription(Image, camera["color_topic"], self._color, qos_profile_sensor_data)
        self.create_subscription(Image, camera["depth_topic"], self._depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera["color_info_topic"], self._color_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera["depth_info_topic"], self._depth_info, qos_profile_sensor_data)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Extrinsics, camera["extrinsics_topic"], self._extrinsics, latched_qos)
        self.create_subscription(Odometry, stationary["odom_topic"], self._odom, qos_profile_sensor_data)
        self.create_subscription(
            JointState,
            "/left/franka_robot_state_broadcaster/measured_joint_states",
            self._joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FrankaRobotState,
            "/left/franka_robot_state_broadcaster/robot_state",
            self._robot_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/left/franka_robot_state_broadcaster/current_pose",
            self._measured_pose,
            qos_profile_sensor_data,
        )
        kin = config["kinematics"]
        self.fk_client = self.create_client(GetPositionFK, kin["fk_service"])
        self.ik_client = self.create_client(GetPositionIK, kin["ik_service"])
        self.validity_client = self.create_client(GetStateValidity, kin["validity_service"])
        self.motion_plan_client = self.create_client(
            GetMotionPlan, kin.get("motion_plan_service", "/left_ik/plan_kinematic_path")
        )
        self.spine_client = self.create_client(GetPosition, kin["spine_position_service"])

    def _color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_stamp = stamp_s(msg)

    def _depth(self, msg):
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        scale = self.cfg["camera"]["depth_scale_m"] if raw.dtype == np.uint16 else 1.0
        self.depth = raw.astype(np.float32) * float(scale)
        self.depth_stamp = stamp_s(msg)

    def _color_info(self, msg):
        self.color_info = msg

    def _depth_info(self, msg):
        self.depth_info = msg

    def _extrinsics(self, msg):
        self.extrinsics = msg

    def _joints(self, msg):
        mapped = dict(zip(msg.name, msg.position))
        if all(name in mapped for name in JOINT_NAMES):
            self.joints = [float(mapped[name]) for name in JOINT_NAMES]

    def _robot_state(self, msg):
        self.robot_state = msg

    def _measured_pose(self, msg):
        self.measured_pose = msg

    def _odom(self, msg):
        t = msg.twist.twist
        linear = math.sqrt(t.linear.x**2 + t.linear.y**2 + t.linear.z**2)
        angular = math.sqrt(t.angular.x**2 + t.angular.y**2 + t.angular.z**2)
        self.odom.append((time.monotonic(), float(linear), float(angular)))

    def call(self, client, request, timeout=3.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise PlanningError("ROS service timeout")
        return future.result()

    def wait_inputs(self, timeout=12.0):
        services = (
            self.fk_client,
            self.ik_client,
            self.validity_client,
            self.motion_plan_client,
            self.spine_client,
        )
        for service in services:
            if not service.wait_for_service(timeout_sec=3.0):
                raise PlanningError(f"service unavailable: {service.srv_name}")
        spine = self.call(self.spine_client, GetPosition.Request())
        if not spine.success:
            raise PlanningError("spine position query failed")
        self.spine_position = float(spine.position)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            ready = all(value is not None for value in (
                self.color, self.depth, self.color_info, self.depth_info,
                self.extrinsics, self.joints, self.robot_state, self.measured_pose,
            ))
            if ready and self.base_stationary():
                return
        missing = [name for name in (
            "color", "depth", "color_info", "depth_info", "extrinsics",
            "joints", "robot_state", "measured_pose",
        ) if getattr(self, name) is None]
        if missing:
            raise PlanningError("missing fresh inputs: " + ",".join(missing))
        raise PlanningError("base did not remain stationary for the required interval")

    def base_stationary(self) -> bool:
        gate = self.cfg["base_stationary_gate"]
        now = time.monotonic()
        samples = [row for row in self.odom if now - row[0] <= gate["required_duration_s"] + 0.15]
        if len(samples) < 4 or now - samples[-1][0] > 0.25:
            return False
        if samples[-1][0] - samples[0][0] < gate["required_duration_s"]:
            return False
        return all(
            linear <= gate["maximum_linear_speed_m_s"]
            and angular <= gate["maximum_angular_speed_rad_s"]
            for _, linear, angular in samples
        )

    def active_errors(self) -> list[str]:
        errors = self.robot_state.current_errors
        return [
            name for name in errors.get_fields_and_field_types()
            if bool(getattr(errors, name))
        ]

    def model_joint_state(self, joints: list[float]) -> tuple[list[str], list[float]]:
        if self.spine_position is None:
            raise PlanningError("spine position is unavailable")
        return JOINT_NAMES + [SPINE_JOINT_NAME], list(map(float, joints)) + [self.spine_position]

    def fk(self, joints: list[float], link: str | None = None):
        kin = self.cfg["kinematics"]
        request = GetPositionFK.Request()
        request.header.frame_id = kin["frame"]
        request.fk_link_names = [link or kin["link"]]
        names, positions = self.model_joint_state(joints)
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = positions
        request.robot_state.is_diff = True
        response = self.call(self.fk_client, request)
        if response.error_code.val != 1 or not response.pose_stamped:
            raise PlanningError(f"FK failed, MoveIt code {response.error_code.val}")
        return response.pose_stamped[0].pose

    def ik(self, position: np.ndarray, quaternion: list[float], seed: list[float]):
        kin = self.cfg["kinematics"]
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = kin["group"]
        ik.ik_link_name = kin["link"]
        ik.pose_stamped.header.frame_id = kin["frame"]
        ik.pose_stamped.pose.position.x = float(position[0])
        ik.pose_stamped.pose.position.y = float(position[1])
        ik.pose_stamped.pose.position.z = float(position[2])
        ik.pose_stamped.pose.orientation.x = quaternion[0]
        ik.pose_stamped.pose.orientation.y = quaternion[1]
        ik.pose_stamped.pose.orientation.z = quaternion[2]
        ik.pose_stamped.pose.orientation.w = quaternion[3]
        names, positions = self.model_joint_state(seed)
        ik.robot_state.joint_state.name = names
        ik.robot_state.joint_state.position = positions
        ik.robot_state.is_diff = True
        ik.avoid_collisions = bool(kin["avoid_collisions"])
        timeout_ns = int(float(kin["ik_timeout_s"]) * 1e9)
        ik.timeout.sec, ik.timeout.nanosec = divmod(timeout_ns, 1_000_000_000)
        response = self.call(self.ik_client, request, timeout=float(kin["ik_timeout_s"]) + 1.5)
        if response.error_code.val != 1:
            raise PlanningError(f"IK failed, MoveIt code {response.error_code.val}")
        mapped = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        if not all(name in mapped for name in JOINT_NAMES):
            raise PlanningError("IK response is missing left-arm joints")
        return [float(mapped[name]) for name in JOINT_NAMES]

    def state_valid(self, joints: list[float]) -> tuple[bool, list[list[str]]]:
        request = GetStateValidity.Request()
        request.group_name = self.cfg["kinematics"]["group"]
        names, positions = self.model_joint_state(joints)
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = positions
        request.robot_state.is_diff = True
        response = self.call(self.validity_client, request)
        contacts = [[c.contact_body_1, c.contact_body_2] for c in response.contacts]
        return bool(response.valid), contacts

    def solve_pose_segment(
        self, label, start_position, target_position,
        start_quaternion, target_quaternion, seed,
    ):
        distance = float(np.linalg.norm(target_position - start_position))
        orientation_distance = quaternion_angle_deg(start_quaternion, target_quaternion)
        steps = max(
            2,
            int(math.ceil(distance / 0.025)),
            int(math.ceil(
                orientation_distance
                / float(self.cfg["kinematics"]["maximum_orientation_step_deg"])
            )),
        )
        result, previous = [], list(seed)
        positions = interpolate(start_position, target_position, steps)
        for index, position in enumerate(positions, 1):
            quaternion = slerp(start_quaternion, target_quaternion, index / steps).tolist()
            solution = self.ik(position, quaternion, previous)
            joint_step = max(abs(a - b) for a, b in zip(solution, previous))
            if joint_step > self.cfg["kinematics"]["maximum_joint_step_rad"]:
                if label == "staging_above_pick":
                    return self.plan_pose_transition(
                        label, target_position, target_quaternion, seed
                    )
                raise PlanningError(f"{label} IK discontinuity {joint_step:.6f} rad")
            interpolation_samples = self.cfg["kinematics"]["path_samples_between_waypoints"]
            for subindex, interpolated in enumerate(
                interpolate(np.asarray(previous), np.asarray(solution), interpolation_samples), 1
            ):
                valid, contacts = self.state_valid(interpolated.tolist())
                if not valid:
                    raise PlanningError(
                        f"{label} waypoint {index}.{subindex} collides: {contacts}"
                    )
            result.append({
                "index": index,
                "position_m": position.tolist(),
                "orientation_xyzw": quaternion,
                "joint_positions_rad": solution,
                "maximum_joint_step_rad": joint_step,
                "collision_checked_interpolation_samples": interpolation_samples,
            })
            previous = solution
        return result, previous

    def plan_pose_transition(self, label, target_position, target_quaternion, seed):
        """Use OMPL for the large initial wrist reorientation without IK branch jumps."""
        kin = self.cfg["kinematics"]
        candidates = []
        for _ in range(int(kin.get("ik_candidate_attempts", 64))):
            try:
                candidate = self.ik(target_position, target_quaternion, seed)
            except PlanningError:
                continue
            valid, _ = self.state_valid(candidate)
            if valid:
                candidates.append(candidate)
                if len(candidates) >= int(kin.get("minimum_ik_candidates", 12)):
                    break
        if not candidates:
            raise PlanningError(f"{label} has no valid IK candidate")
        target_joints = min(
            candidates,
            key=lambda values: max(abs(a - b) for a, b in zip(values, seed)),
        )
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = kin["group"]
        motion.pipeline_id = "ompl"
        motion.num_planning_attempts = 6
        motion.allowed_planning_time = 6.0
        motion.max_velocity_scaling_factor = 0.08
        motion.max_acceleration_scaling_factor = 0.08
        names, positions = self.model_joint_state(seed)
        motion.start_state.joint_state.name = names
        motion.start_state.joint_state.position = positions
        motion.start_state.is_diff = True

        goal = Constraints()
        for name, value in zip(JOINT_NAMES, target_joints):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(value)
            constraint.tolerance_above = 0.002
            constraint.tolerance_below = 0.002
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        motion.goal_constraints = [goal]

        response = self.call(self.motion_plan_client, request, timeout=9.0)
        plan = response.motion_plan_response
        if plan.error_code.val != 1:
            raise PlanningError(f"{label} OMPL failed, MoveIt code {plan.error_code.val}")
        trajectory = plan.trajectory.joint_trajectory
        mapped_indices = {name: index for index, name in enumerate(trajectory.joint_names)}
        if not all(name in mapped_indices for name in JOINT_NAMES):
            raise PlanningError(f"{label} OMPL trajectory is missing left-arm joints")

        result = []
        previous = list(seed)
        maximum_step = float(kin["maximum_joint_step_rad"])
        for trajectory_point in trajectory.points:
            target = [
                float(trajectory_point.positions[mapped_indices[name]]) for name in JOINT_NAMES
            ]
            delta = max(abs(a - b) for a, b in zip(target, previous))
            subdivisions = max(1, int(math.ceil(delta / maximum_step)))
            for joints in interpolate(np.asarray(previous), np.asarray(target), subdivisions):
                valid, contacts = self.state_valid(joints.tolist())
                if not valid:
                    raise PlanningError(f"{label} OMPL path collides: {contacts}")
                step = max(abs(a - b) for a, b in zip(joints, previous))
                result.append({
                    "index": len(result) + 1,
                    "position_m": None,
                    "orientation_xyzw": None,
                    "joint_positions_rad": joints.tolist(),
                    "maximum_joint_step_rad": step,
                    "collision_checked_interpolation_samples": 1,
                    "planner": "ompl_initial_pose_transition",
                })
                previous = joints.tolist()
        if not result:
            raise PlanningError(f"{label} OMPL returned an empty trajectory")
        return result, previous

    def solve_cartesian_segment(self, label, start_position, target_position, quaternion, seed):
        return self.solve_pose_segment(
            label, start_position, target_position, quaternion, quaternion, seed
        )

    def validate_joint_segment(self, label, start, target):
        invalid = []
        samples = int(self.cfg["kinematics"]["path_samples_per_segment"])
        for index, joints in enumerate(interpolate(np.asarray(start), np.asarray(target), samples), 1):
            valid, contacts = self.state_valid(joints.tolist())
            if not valid:
                invalid.append({"index": index, "contacts": contacts})
        if invalid:
            raise PlanningError(f"{label} joint path has {len(invalid)} invalid samples")
        return samples

    def plan(
        self,
        annotated_output: Path | None = None,
        *,
        max_segment: int = 2,
        require_center: bool = True,
    ) -> dict:
        self.wait_inputs()
        if not self.base_stationary():
            raise PlanningError("base stationary gate changed after capture")
        errors = self.active_errors()
        if errors:
            raise PlanningError("persistent Franka errors: " + ",".join(errors))
        initial = np.asarray(self.cfg["initial_joints_rad"], dtype=float)
        measured = np.asarray(self.joints, dtype=float)
        initial_error = float(np.max(np.abs(measured - initial)))
        if initial_error > self.cfg["initial_joint_tolerance_rad"]:
            raise PlanningError(f"left arm is not at initial pose ({initial_error:.6f} rad)")
        visual = self.cfg["visual_gate"]
        if visual["center_axis"] != "y":
            raise PlanningError("only the calibrated wrist Y-axis gate is supported")
        sequence_cfg = self.cfg["motion_sequence"]
        if sequence_cfg["ground_aligned_frame"] != self.cfg["kinematics"]["frame"]:
            raise PlanningError("motion sequence frame differs from the MoveIt planning frame")
        if sequence_cfg["shoulder_frame"] != self.cfg["kinematics"]["arm_mount_link"]:
            raise PlanningError("configured shoulder frame differs from the live arm mount frame")

        def intrinsics(info):
            projection = info.p if info.p[0] else info.k
            return Intrinsics(
                int(info.width), int(info.height), float(projection[0]),
                float(projection[5]), float(projection[2]), float(projection[6]),
            )

        # The thin hanging endpoint is an RGB/depth occlusion edge.  Use a
        # temporal median and require a majority of samples to agree in 3-D.
        observations, points, per_frame_depth, sync_errors = [], [], [], []
        last_pair = None
        camera_cfg = self.cfg["camera"]
        deadline = time.monotonic() + camera_cfg["sample_timeout_s"]
        while len(points) < camera_cfg["temporal_samples"] and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.06)
            pair = (self.color_stamp, self.depth_stamp)
            if pair == last_pair:
                continue
            last_pair = pair
            frame_sync_error = abs(float(self.color_stamp) - float(self.depth_stamp))
            if frame_sync_error > camera_cfg["maximum_sync_error_s"]:
                continue
            observation = detect_pad_end(
                self.color,
                tuple(visual["near_robot_end_image_direction"]),
                visual["endpoint_inset_fraction"],
            )
            if observation is None:
                continue
            frame_center_error = observation.center_uv[1] - self.color.shape[0] / 2.0
            if require_center and abs(frame_center_error) > visual["center_deadband_px"]:
                raise PlanningError(
                    f"thermal pad left wrist Y gate changed ({frame_center_error:+.2f} px)"
                )
            try:
                point, stats = register_depth_point(
                    self.depth,
                    intrinsics(self.depth_info),
                    intrinsics(self.color_info),
                    np.asarray(self.extrinsics.rotation).reshape(3, 3),
                    np.asarray(self.extrinsics.translation),
                    observation.grasp_uv,
                    observation.mask,
                    radius_px=visual["depth_search_radius_px"],
                    min_depth_m=camera_cfg["minimum_depth_m"],
                    max_depth_m=camera_cfg["maximum_depth_m"],
                )
            except ValueError:
                continue
            observations.append(observation)
            points.append(point)
            per_frame_depth.append(stats)
            sync_errors.append(frame_sync_error)
        if len(points) < camera_cfg["minimum_temporal_inliers"]:
            raise PlanningError(f"only {len(points)} valid synchronized RGB/depth samples")
        raw_points = np.asarray(points)
        provisional = np.median(raw_points, axis=0)
        residuals = np.linalg.norm(raw_points - provisional, axis=1)
        inlier_mask = residuals <= camera_cfg["maximum_temporal_point_residual_m"]
        inlier_count = int(np.count_nonzero(inlier_mask))
        if inlier_count < camera_cfg["minimum_temporal_inliers"]:
            raise PlanningError(
                f"unstable temporal depth: {inlier_count}/{len(points)} agreeing samples"
            )
        point_camera = np.median(raw_points[inlier_mask], axis=0)
        point_spread = float(np.max(np.linalg.norm(
            raw_points[inlier_mask] - point_camera, axis=1
        )))
        observation = observations[-1]
        center_error = float(np.median([
            item.center_uv[1] - self.color.shape[0] / 2.0 for item in observations
        ]))
        sync_error = max(sync_errors)
        depth_stats = {
            "temporal_samples": len(points),
            "temporal_inliers": inlier_count,
            "maximum_inlier_residual_m": point_spread,
            "per_frame": per_frame_depth,
        }

        fk_pose = self.fk(self.joints)
        mount_pose = self.fk(self.joints, self.cfg["kinematics"]["arm_mount_link"])
        fk_position, fk_quaternion = pose_values(fk_pose)
        measured_position, measured_quaternion = pose_values(self.measured_pose.pose)
        base_from_mount = pose_matrix(*pose_values(mount_pose))
        mount_from_measured_ee = pose_matrix(measured_position, measured_quaternion)
        link8_from_measured_ee = np.eye(4)
        link8_from_measured_ee[:3, 3] = np.asarray(
            self.cfg["hand_eye"]["link8_to_measured_ee_local_m"], dtype=float
        )
        base_from_measured_link8 = (
            base_from_mount @ mount_from_measured_ee @ np.linalg.inv(link8_from_measured_ee)
        )
        base_from_fk_link8 = pose_matrix(fk_position, fk_quaternion)
        pose_delta = float(np.linalg.norm(
            base_from_fk_link8[:3, 3] - base_from_measured_link8[:3, 3]
        ))
        orientation_delta = math.degrees(math.acos(float(np.clip(
            (np.trace(base_from_fk_link8[:3, :3].T @ base_from_measured_link8[:3, :3]) - 1.0) / 2.0,
            -1.0,
            1.0,
        ))))
        if pose_delta > 0.03 or orientation_delta > 5.0:
            raise PlanningError(
                f"MoveIt/live link8 mismatch ({pose_delta:.4f} m, {orientation_delta:.2f} deg)"
            )
        base_from_parent = base_from_mount @ mount_from_measured_ee
        parent_from_camera = np.asarray(
            self.cfg["hand_eye"]["transform_parent_from_color_camera_row_major"], dtype=float
        )
        point_base = transform_point(base_from_parent @ parent_from_camera, point_camera)
        grasp_cfg = self.cfg["grasp"]
        lower = np.asarray(grasp_cfg["workspace_contact_min_m"])
        upper = np.asarray(grasp_cfg["workspace_contact_max_m"])
        if not np.all((point_base >= lower) & (point_base <= upper)):
            raise PlanningError(f"3-D contact outside workspace: {point_base.tolist()}")

        pick_orientation = horizontal_gripper_orientation(sequence_cfg).tolist()
        rotation = quaternion_matrix(pick_orientation)
        link8_to_contact = rotation @ np.asarray(grasp_cfg["link8_to_finger_contact_local_m"])
        grasp_position = point_base + np.array([0.0, 0.0, grasp_cfg["contact_clearance_base_z_m"]]) - link8_to_contact
        current_position = np.asarray(fk_position)
        # grasp_position and fk_quaternion are in the ground-aligned whole-robot
        # planning frame. Never apply the following offsets to the FCI pose,
        # whose origin and axes are local to the left shoulder.
        sequence = build_sequence(grasp_position, sequence_cfg)
        sequence_plans = {}
        previous_position = current_position
        previous_orientation = fk_quaternion
        previous_joints = list(self.joints)
        selected_targets = [
            target for target in sequence["targets"]
            if int(target["segment"]) <= int(max_segment)
        ]
        for target in selected_targets:
            target_position = np.asarray(target["position_m"], dtype=float)
            target_orientation = target["orientation_xyzw"]
            try:
                segment, previous_joints = self.solve_pose_segment(
                    target["name"], previous_position, target_position,
                    previous_orientation, target_orientation, previous_joints,
                )
            except PlanningError as exc:
                raise PlanningError(
                    f"{exc}; target_position_m={target_position.tolist()}; "
                    f"target_orientation_xyzw={target_orientation}"
                ) from exc
            sequence_plans[target["name"]] = segment
            previous_position = target_position
            previous_orientation = target_orientation
        return_checks = self.validate_joint_segment(
            "return_to_initial", previous_joints, initial.tolist()
        )

        if annotated_output is not None:
            annotated_output.parent.mkdir(parents=True, exist_ok=True)
            annotated = self.color.copy()
            cv2.drawMarker(annotated, tuple(map(int, observation.center_uv)), (0, 255, 255),
                           cv2.MARKER_CROSS, 22, 2)
            cv2.drawMarker(annotated, tuple(map(int, observation.grasp_uv)), (0, 0, 255),
                           cv2.MARKER_TILTED_CROSS, 22, 2)
            cv2.imwrite(str(annotated_output), annotated)

        return {
            "schema_version": 1,
            "status": "valid",
            "semantics": "FK/IK and collision validation only; no motion or gripper command sent",
            "base_stationary": True,
            "right_arm_commanded": False,
            "planning_scope_max_segment": int(max_segment),
            "wrist_center_gate_required": bool(require_center),
            "spine_position_m": self.spine_position,
            "left_arm_initial_error_rad": initial_error,
            "rgb_depth_sync_error_s": sync_error,
            "pad_center_uv": list(observation.center_uv),
            "pad_center_y_error_px": center_error,
            "grasp_pixel_uv": list(observation.grasp_uv),
            "point_color_camera_m": point_camera.tolist(),
            "contact_point_base_m": point_base.tolist(),
            "depth": depth_stats,
            "fk_measured_consistency": {
                "position_error_m": pose_delta,
                "orientation_error_deg": orientation_delta,
                "comparison": "MoveIt base->link8 versus live mount->measured_EE with configured flange offset",
            },
            "poses": {
                "grasp_link8_position_m": grasp_position.tolist(),
                "pick_orientation_xyzw": pick_orientation,
            },
            "motion_sequence": sequence,
            "plans": sequence_plans,
            "return_to_initial": {
                "target_joint_positions_rad": initial.tolist(),
                "collision_checked_samples": return_checks,
            },
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument("--annotated-output", type=Path, default=ROOT / "outputs" / "thermal_pad_ik.jpg")
    parser.add_argument("--max-segment", type=int, choices=(1, 2), default=2)
    parser.add_argument("--allow-off-center", action="store_true")
    parser.add_argument("--single-depth-frame", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if args.single_depth_frame:
        config["camera"]["temporal_samples"] = 1
        config["camera"]["minimum_temporal_inliers"] = 1
        config["camera"]["maximum_sync_error_s"] = 10.0
        config["camera"]["maximum_temporal_point_residual_m"] = 10.0
    rclpy.init()
    node = ThermalPadPlanner(config)
    try:
        result = node.plan(
            args.annotated_output,
            max_segment=args.max_segment,
            require_center=not args.allow_off_center,
        )
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except Exception as exc:
        result = {
            "status": "blocked",
            "error": str(exc),
            "semantics": "zero arm/gripper command sent by this planner",
        }
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
