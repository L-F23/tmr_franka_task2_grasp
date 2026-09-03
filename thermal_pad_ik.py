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
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
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


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "config" / "thermal_pad_pick.json"
DEFAULT_RECORD = ROOT / "config" / "latest_thermal_pad_ik.json"
JOINT_NAMES = [f"left_fr3v2_joint{i}" for i in range(1, 8)]


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
        services = (self.fk_client, self.ik_client, self.validity_client)
        for service in services:
            if not service.wait_for_service(timeout_sec=3.0):
                raise PlanningError(f"service unavailable: {service.srv_name}")
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

    def fk(self, joints: list[float]):
        kin = self.cfg["kinematics"]
        request = GetPositionFK.Request()
        request.header.frame_id = kin["frame"]
        request.fk_link_names = [kin["link"]]
        request.robot_state.joint_state.name = JOINT_NAMES
        request.robot_state.joint_state.position = list(map(float, joints))
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
        ik.robot_state.joint_state.name = JOINT_NAMES
        ik.robot_state.joint_state.position = list(map(float, seed))
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
        request.robot_state.joint_state.name = JOINT_NAMES
        request.robot_state.joint_state.position = list(map(float, joints))
        request.robot_state.is_diff = True
        response = self.call(self.validity_client, request)
        contacts = [[c.contact_body_1, c.contact_body_2] for c in response.contacts]
        return bool(response.valid), contacts

    def solve_cartesian_segment(self, label, start_position, target_position, quaternion, seed):
        distance = float(np.linalg.norm(target_position - start_position))
        steps = max(2, int(math.ceil(distance / 0.025)))
        result, previous = [], list(seed)
        for index, position in enumerate(interpolate(start_position, target_position, steps), 1):
            solution = self.ik(position, quaternion, previous)
            joint_step = max(abs(a - b) for a, b in zip(solution, previous))
            if joint_step > self.cfg["kinematics"]["maximum_joint_step_rad"]:
                raise PlanningError(f"{label} IK discontinuity {joint_step:.6f} rad")
            valid, contacts = self.state_valid(solution)
            if not valid:
                raise PlanningError(f"{label} waypoint {index} collides: {contacts}")
            result.append({
                "index": index,
                "position_m": position.tolist(),
                "joint_positions_rad": solution,
                "maximum_joint_step_rad": joint_step,
            })
            previous = solution
        return result, previous

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

    def plan(self, annotated_output: Path | None = None) -> dict:
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
        sync_error = abs(float(self.color_stamp) - float(self.depth_stamp))
        if sync_error > self.cfg["camera"]["maximum_sync_error_s"]:
            raise PlanningError(f"RGB/depth sync error {sync_error:.6f} s")

        visual = self.cfg["visual_gate"]
        observation = detect_pad_end(
            self.color,
            tuple(visual["near_robot_end_image_direction"]),
            visual["endpoint_inset_fraction"],
        )
        if observation is None:
            raise PlanningError("grey thermal pad not detected in left wrist image")
        if visual["center_axis"] != "y":
            raise PlanningError("only the calibrated wrist Y-axis gate is supported")
        center_error = observation.center_uv[1] - self.color.shape[0] / 2.0
        if abs(center_error) > visual["center_deadband_px"]:
            raise PlanningError(f"thermal pad not centered in wrist Y ({center_error:+.2f} px)")

        def intrinsics(info):
            projection = info.p if info.p[0] else info.k
            return Intrinsics(
                int(info.width), int(info.height), float(projection[0]),
                float(projection[5]), float(projection[2]), float(projection[6]),
            )

        point_camera, depth_stats = register_depth_point(
            self.depth,
            intrinsics(self.depth_info),
            intrinsics(self.color_info),
            np.asarray(self.extrinsics.rotation).reshape(3, 3),
            np.asarray(self.extrinsics.translation),
            observation.grasp_uv,
            observation.mask,
            radius_px=visual["depth_search_radius_px"],
            min_depth_m=self.cfg["camera"]["minimum_depth_m"],
            max_depth_m=self.cfg["camera"]["maximum_depth_m"],
        )

        fk_pose = self.fk(self.joints)
        fk_position, fk_quaternion = pose_values(fk_pose)
        measured_position, measured_quaternion = pose_values(self.measured_pose.pose)
        pose_delta = float(np.linalg.norm(np.asarray(fk_position) - np.asarray(measured_position)))
        orientation_delta = quaternion_angle_deg(fk_quaternion, measured_quaternion)
        if pose_delta > 0.03 or orientation_delta > 5.0:
            raise PlanningError(
                f"hand-eye parent/link8 mismatch ({pose_delta:.4f} m, {orientation_delta:.2f} deg)"
            )
        base_from_parent = pose_matrix(measured_position, measured_quaternion)
        parent_from_camera = np.asarray(
            self.cfg["hand_eye"]["transform_parent_from_color_camera_row_major"], dtype=float
        )
        point_base = transform_point(base_from_parent @ parent_from_camera, point_camera)
        grasp_cfg = self.cfg["grasp"]
        lower = np.asarray(grasp_cfg["workspace_contact_min_m"])
        upper = np.asarray(grasp_cfg["workspace_contact_max_m"])
        if not np.all((point_base >= lower) & (point_base <= upper)):
            raise PlanningError(f"3-D contact outside workspace: {point_base.tolist()}")

        rotation = base_from_parent[:3, :3]
        link8_to_contact = rotation @ np.asarray(grasp_cfg["link8_to_finger_contact_local_m"])
        grasp_position = point_base + np.array([0.0, 0.0, grasp_cfg["contact_clearance_base_z_m"]]) - link8_to_contact
        pregrasp_position = grasp_position + np.array([0.0, 0.0, grasp_cfg["pregrasp_clearance_base_z_m"]])
        lift_position = grasp_position + np.array([0.0, 0.0, grasp_cfg["lift_clearance_base_z_m"]])
        current_position = np.asarray(fk_position)

        approach, q_pre = self.solve_cartesian_segment(
            "approach", current_position, pregrasp_position, fk_quaternion, self.joints
        )
        descend, q_grasp = self.solve_cartesian_segment(
            "descend", pregrasp_position, grasp_position, fk_quaternion, q_pre
        )
        lift, q_lift = self.solve_cartesian_segment(
            "lift", grasp_position, lift_position, fk_quaternion, q_grasp
        )
        return_checks = self.validate_joint_segment("return_to_initial", q_lift, initial.tolist())

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
            },
            "poses": {
                "pregrasp_link8_position_m": pregrasp_position.tolist(),
                "grasp_link8_position_m": grasp_position.tolist(),
                "lift_link8_position_m": lift_position.tolist(),
                "orientation_xyzw": fk_quaternion,
            },
            "plans": {"approach": approach, "descend": descend, "lift": lift},
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
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = ThermalPadPlanner(config)
    try:
        result = node.plan(args.annotated_output)
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
