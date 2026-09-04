#!/usr/bin/env python3
"""Replan, then execute only segment 1 of the thermal-pad sequence."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from franka_msgs.action import PTPMotion
from franka_spine_msgs.srv import GetPosition
from rclpy.action import ActionClient

from thermal_pad_ik import DEFAULT_CONFIG, ROOT, ThermalPadPlanner
from thermal_pad_sequence import horizontal_gripper_orientation


GRIPPER_ACTION = "/left/gripper/robotiq_gripper_controller/gripper_cmd"
PTP_ACTION = "/left/action_server/ptp_motion"
DEFAULT_RECORD = ROOT / "config" / "latest_thermal_pad_grasp.json"


class GuardedContactStop(RuntimeError):
    """Raised after cancelling an approach because external contact was detected."""


def _vector3(value) -> np.ndarray:
    return np.asarray([value.x, value.y, value.z], dtype=float)


def _collision_indicator_values(indicators) -> list[float]:
    values = []
    for name in (
        "is_cartesian_linear_contact",
        "is_cartesian_angular_contact",
        "is_cartesian_linear_collision",
        "is_cartesian_angular_collision",
    ):
        values.extend(_vector3(getattr(indicators, name)).tolist())
    values.extend(map(float, indicators.is_joint_contact))
    values.extend(map(float, indicators.is_joint_collision))
    return values


class ExternalContactGuard:
    """Debounced, baseline-relative Franka external-wrench guard.

    The native contact/collision flags always win.  Wrench thresholds are
    relative to a settled baseline so the tool/load gravity bias is not
    mistaken for contact during the slow final approach.
    """

    def __init__(
        self,
        baseline_force_n,
        baseline_torque_nm,
        baseline_joint_torque_nm,
        *,
        approach_axis: int = 0,
        axis_force_delta_n: float = 4.0,
        force_delta_norm_n: float = 7.0,
        torque_delta_norm_nm: float = 1.5,
        joint_torque_delta_nm: float = 2.0,
        consecutive_samples: int = 3,
    ):
        self.baseline_force_n = np.asarray(baseline_force_n, dtype=float)
        self.baseline_torque_nm = np.asarray(baseline_torque_nm, dtype=float)
        self.baseline_joint_torque_nm = np.asarray(
            baseline_joint_torque_nm, dtype=float
        )
        self.approach_axis = int(approach_axis)
        self.axis_force_delta_n = float(axis_force_delta_n)
        self.force_delta_norm_n = float(force_delta_norm_n)
        self.torque_delta_norm_nm = float(torque_delta_norm_nm)
        self.joint_torque_delta_nm = float(joint_torque_delta_nm)
        self.consecutive_samples = max(1, int(consecutive_samples))
        self.trigger_count = 0
        self.last_measurement = None

    def observe(self, state) -> dict | None:
        force = _vector3(state.o_f_ext_hat_k.wrench.force)
        torque = _vector3(state.o_f_ext_hat_k.wrench.torque)
        joint = np.asarray(state.tau_ext_hat_filtered.effort, dtype=float)
        force_delta = force - self.baseline_force_n
        torque_delta = torque - self.baseline_torque_nm
        joint_delta = joint - self.baseline_joint_torque_nm
        native_flag = any(
            abs(value) > 0.0
            for value in _collision_indicator_values(state.collision_indicators)
        )
        reasons = []
        if native_flag:
            reasons.append("franka_native_contact_or_collision_flag")
        if abs(force_delta[self.approach_axis]) >= self.axis_force_delta_n:
            reasons.append("approach_axis_force_delta")
        if float(np.linalg.norm(force_delta)) >= self.force_delta_norm_n:
            reasons.append("force_delta_norm")
        if float(np.linalg.norm(torque_delta)) >= self.torque_delta_norm_nm:
            reasons.append("torque_delta_norm")
        if joint_delta.size and float(np.max(np.abs(joint_delta))) >= self.joint_torque_delta_nm:
            reasons.append("joint_torque_delta")
        # During a straight +X approach, orientation/load transients can move
        # torque estimates without any object contact.  Torque and joint
        # torque remain useful diagnostics, but may not stop the arm by
        # themselves.  A stop requires either Franka's native indication or
        # a sustained force change along the commanded approach axis.
        trigger_eligible = native_flag or "approach_axis_force_delta" in reasons
        self.trigger_count = self.trigger_count + 1 if trigger_eligible else 0
        self.last_measurement = {
            "force_base_n": force.tolist(),
            "force_delta_n": force_delta.tolist(),
            "torque_base_nm": torque.tolist(),
            "torque_delta_nm": torque_delta.tolist(),
            "maximum_joint_torque_delta_nm": (
                float(np.max(np.abs(joint_delta))) if joint_delta.size else 0.0
            ),
            "native_contact_or_collision": native_flag,
            "candidate_reasons": reasons,
            "trigger_eligible": trigger_eligible,
            "consecutive_trigger_samples": self.trigger_count,
        }
        if self.trigger_count >= self.consecutive_samples:
            return dict(self.last_measurement)
        return None


class ThermalPadExecutor(ThermalPadPlanner):
    def __init__(self, config: dict):
        super().__init__(config)
        self.ptp_client = ActionClient(self, PTPMotion, PTP_ACTION)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.controller_list_client = self.create_client(
            ListControllers, "/left/controller_manager/list_controllers"
        )
        self.controller_switch_client = self.create_client(
            SwitchController, "/left/controller_manager/switch_controller"
        )
        self.fast_execution = False
        self.isolated_base_zero_locked = False
        self.contact_guard: ExternalContactGuard | None = None
        self.contact_stop = None

    def prepare_ptp_control(self) -> None:
        """Release the impedance command interface before native PTP starts."""
        if not self.controller_list_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("left controller-list service unavailable")
        listed = self.call(
            self.controller_list_client, ListControllers.Request(), timeout=5.0
        )
        states = {item.name: item.state for item in listed.controller}
        if states.get("joint_impedance_controller") != "active":
            return
        if not self.controller_switch_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("left controller-switch service unavailable")
        request = SwitchController.Request()
        request.deactivate_controllers = ["joint_impedance_controller"]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout.sec = 5
        response = self.call(self.controller_switch_client, request, timeout=8.0)
        if not response.ok:
            raise RuntimeError("failed to release left impedance controller for PTP")

    def base_stationary(self) -> bool:
        """Accept a separately verified zero lock on the isolated base host.

        The supported deployment keeps base ROS 2 traffic on host .50 in a
        localhost-only domain, so the arm host cannot subscribe to odometry.
        This explicit execution-only assertion is set after the operator has
        latched zero through the base host's exclusive command adapter.
        """
        return self.isolated_base_zero_locked or super().base_stationary()

    def cancel(self, handle) -> None:
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=4.0)

    def motion_gate(self, handle=None) -> None:
        errors = self.active_errors() if self.robot_state is not None else ["robot_state_lost"]
        if errors or not self.base_stationary():
            if handle is not None:
                self.cancel(handle)
            reason = "Franka errors: " + ",".join(errors) if errors else "base moved during grasp"
            raise RuntimeError(reason)
        if self.contact_guard is not None:
            triggered = self.contact_guard.observe(self.robot_state)
            if triggered is not None:
                self.contact_stop = triggered
                if handle is not None:
                    self.cancel(handle)
                raise GuardedContactStop(
                    "guarded approach stopped on external contact: "
                    + ",".join(triggered["candidate_reasons"])
                )

    def capture_external_wrench_baseline(
        self, *, duration_s: float = 0.6, minimum_samples: int = 80, **guard_kwargs
    ) -> dict:
        """Capture a settled, read-only baseline and arm the contact guard."""
        forces, torques, joints = [], [], []
        deadline = time.monotonic() + float(duration_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.robot_state is None:
                continue
            forces.append(_vector3(self.robot_state.o_f_ext_hat_k.wrench.force))
            torques.append(_vector3(self.robot_state.o_f_ext_hat_k.wrench.torque))
            joints.append(np.asarray(
                self.robot_state.tau_ext_hat_filtered.effort, dtype=float
            ))
        if len(forces) < int(minimum_samples):
            raise RuntimeError(
                f"insufficient Franka wrench samples: {len(forces)} < {minimum_samples}"
            )
        force = np.median(np.asarray(forces), axis=0)
        torque = np.median(np.asarray(torques), axis=0)
        joint = np.median(np.asarray(joints), axis=0)
        self.contact_stop = None
        self.contact_guard = ExternalContactGuard(
            force, torque, joint, **guard_kwargs
        )
        return {
            "sample_count": len(forces),
            "force_base_n": force.tolist(),
            "torque_base_nm": torque.tolist(),
            "joint_torque_nm": joint.tolist(),
            "thresholds": {
                "axis_force_delta_n": self.contact_guard.axis_force_delta_n,
                "force_delta_norm_n": self.contact_guard.force_delta_norm_n,
                "torque_delta_norm_nm": self.contact_guard.torque_delta_norm_nm,
                "joint_torque_delta_nm": self.contact_guard.joint_torque_delta_nm,
                "consecutive_samples": self.contact_guard.consecutive_samples,
            },
        }

    def disable_contact_guard(self) -> None:
        self.contact_guard = None

    def wait_arm_stationary(
        self, *, timeout_s: float = 4.0, maximum_velocity_rad_s: float = 0.01
    ) -> dict:
        """Wait for a cancelled native PTP goal to stop before retracting."""
        deadline = time.monotonic() + float(timeout_s)
        stable_samples = 0
        maximum_seen = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.robot_state is None:
                continue
            velocities = np.asarray(
                self.robot_state.measured_joint_state.velocity, dtype=float
            )
            maximum = float(np.max(np.abs(velocities))) if velocities.size else 0.0
            maximum_seen = max(maximum_seen, maximum)
            stable_samples = stable_samples + 1 if maximum <= maximum_velocity_rad_s else 0
            if stable_samples >= 20:
                return {
                    "maximum_velocity_limit_rad_s": float(maximum_velocity_rad_s),
                    "stable_samples": stable_samples,
                    "maximum_velocity_at_stop_rad_s": maximum,
                    "maximum_velocity_seen_rad_s": maximum_seen,
                }
        raise RuntimeError("left arm did not settle after guarded-contact cancellation")

    def move_ptp(self, joints: list[float], label: str, speed: float) -> dict:
        self.prepare_ptp_control()
        self.motion_gate()
        goal = PTPMotion.Goal()
        goal.goal_joint_configuration = list(map(float, joints))
        goal.maximum_joint_velocities = [float(speed)] * 7
        goal.goal_tolerance = 0.006
        future = self.ptp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} PTP goal rejected")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 30.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.04)
            self.motion_gate(handle)
        if not result_future.done():
            self.cancel(handle)
            raise RuntimeError(f"{label} PTP timeout")
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{label} PTP failed with action status {getattr(wrapped, 'status', None)}")
        target_status = int(wrapped.result.target_status.status)
        if target_status != wrapped.result.target_status.TARGET_REACHED:
            raise RuntimeError(
                f"{label} PTP target status {target_status}: {wrapped.result.error_message}"
            )
        settle_s = 0.08 if self.fast_execution else 2.0
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.04)
        endpoint_error = float(np.max(np.abs(np.asarray(self.joints) - np.asarray(joints))))
        if endpoint_error > 0.012:
            raise RuntimeError(f"{label} endpoint error {endpoint_error:.6f} rad")
        return {"label": label, "maximum_joint_error_rad": endpoint_error}

    def command_gripper(self, position: float, label: str) -> dict:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 1.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} gripper goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=35.0)
        wrapped = result_future.result() if result_future.done() else None
        if wrapped is None:
            self.cancel(handle)
            raise RuntimeError(f"{label} gripper timeout")
        result = wrapped.result
        return {
            "label": label,
            "action_status": int(wrapped.status),
            "position": float(result.position),
            "effort": float(result.effort),
            "stalled": bool(result.stalled),
            "reached_goal": bool(result.reached_goal),
        }

    def compress_waypoints(self, waypoints: list[dict], start: list[float]) -> list[dict]:
        """Safely coalesce dense OMPL output for the explicitly requested fast run."""
        if not waypoints:
            return []
        maximum_jump = float(self.cfg["empty_cycle"].get("fast_max_joint_jump_rad", 0.20))
        samples = (
            int(self.cfg["empty_cycle"].get("fast_collision_samples", 10))
            if self.collision_guard_enabled else 0
        )
        compressed = []
        previous = np.asarray(start, dtype=float)
        cursor = 0
        while cursor < len(waypoints):
            furthest = cursor
            while furthest + 1 < len(waypoints):
                candidate = np.asarray(
                    waypoints[furthest + 1]["joint_positions_rad"], dtype=float
                )
                if float(np.max(np.abs(candidate - previous))) > maximum_jump:
                    break
                furthest += 1
            accepted = None
            for candidate_index in range(furthest, cursor - 1, -1):
                candidate = np.asarray(
                    waypoints[candidate_index]["joint_positions_rad"], dtype=float
                )
                collision_free = True
                if self.collision_guard_enabled:
                    for sample in np.linspace(previous, candidate, samples + 1)[1:]:
                        valid, _ = self.state_valid(sample.tolist())
                        if not valid:
                            collision_free = False
                            break
                if collision_free:
                    accepted = candidate_index
                    break
            if accepted is None:
                raise RuntimeError(f"cannot safely compress waypoint {cursor + 1}")
            item = dict(waypoints[accepted])
            item["index"] = len(compressed) + 1
            item["maximum_joint_step_rad"] = float(np.max(np.abs(
                np.asarray(item["joint_positions_rad"], dtype=float) - previous
            )))
            item["collision_checked_interpolation_samples"] = samples
            item["fast_path_compressed"] = True
            compressed.append(item)
            previous = np.asarray(item["joint_positions_rad"], dtype=float)
            cursor = accepted + 1
        return compressed

    def plan_empty_cycle(self, *, resume: bool = False, fast: bool = False) -> dict:
        """Plan the requested fixed empty-gripper demonstration without perception."""
        for service in self.planning_services():
            if not service.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(f"service unavailable: {service.srv_name}")
        spine = self.call(self.spine_client, GetPosition.Request())
        if not spine.success:
            raise RuntimeError("spine position query failed")
        self.spine_position = float(spine.position)
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.joints is not None
                and self.robot_state is not None
                and self.measured_pose is not None
                and self.base_stationary()
            ):
                break
        else:
            raise RuntimeError("fresh arm state or stationary-base evidence unavailable")
        errors = self.active_errors()
        if errors:
            raise RuntimeError("persistent Franka errors: " + ",".join(errors))
        initial = np.asarray(self.cfg["initial_joints_rad"], dtype=float)
        measured_start = np.asarray(self.joints, dtype=float)
        initial_error = float(np.max(np.abs(measured_start - initial)))
        if not resume and initial_error > self.cfg["initial_joint_tolerance_rad"]:
            raise RuntimeError(f"left arm is not at initial pose ({initial_error:.6f} rad)")

        empty = self.cfg["empty_cycle"]
        retracted = np.asarray(empty["retracted_link8_position_m"], dtype=float)
        forward = np.asarray(self.cfg["motion_sequence"]["ground_forward_axis_xyz"], dtype=float)
        up = np.asarray(self.cfg["motion_sequence"]["ground_up_axis_xyz"], dtype=float)
        advanced = retracted + forward * float(empty["advance_m"])
        lifted = advanced + up * float(empty["lift_m"])
        carried = lifted + forward * float(empty["carry_far_m"])
        orientation = horizontal_gripper_orientation(self.cfg["motion_sequence"]).tolist()
        current_pose = self.fk(self.joints)
        current_position = np.array([
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
        ])
        current_orientation = [current_pose.orientation.x, current_pose.orientation.y,
                               current_pose.orientation.z, current_pose.orientation.w]
        if resume:
            # After an operator interruption, keep the tool motion intuitive:
            # interpolate position and orientation together instead of allowing
            # another unconstrained joint-space detour.
            first, seed = self.solve_pose_segment(
                "controlled_reorientation",
                current_position,
                retracted,
                current_orientation,
                orientation,
                self.joints,
            )
        else:
            first, seed = self.plan_pose_transition(
                "staging_above_pick", retracted, orientation, self.joints
            )
        if fast:
            first = self.compress_waypoints(first, measured_start.tolist())
        advance, seed = self.solve_pose_segment(
            "advance_open_to_pad", retracted, advanced, orientation, orientation, seed
        )
        lift, seed = self.solve_pose_segment(
            "lift_vertical_12cm", advanced, lifted, orientation, orientation, seed
        )
        carry, seed = self.solve_pose_segment(
            "carry_far_12cm", lifted, carried, orientation, orientation, seed
        )
        return {
            "status": "valid",
            "semantics": "fixed empty-gripper segment 1; no camera localization used",
            "base_stationary": True,
            "right_arm_commanded": False,
            "left_arm_initial_error_rad": initial_error,
            "resumed_from_interrupted_staging": bool(resume),
            "fast_execution": bool(fast),
            "measured_start_joints_rad": measured_start.tolist(),
            "poses": {
                "retracted_m": retracted.tolist(),
                "advanced_m": advanced.tolist(),
                "lifted_m": lifted.tolist(),
                "carried_far_m": carried.tolist(),
                "orientation_xyzw": orientation,
                "initial_orientation_xyzw": current_orientation,
            },
            "plans": {
                "staging_above_pick": first,
                "pick_height_retracted": [],
                "advance_open_to_pad": advance,
                "lift_vertical_12cm": lift,
                "carry_far_12cm": carry,
            },
        }

    def execute(self, annotated_output: Path, *, allow_off_center: bool, empty_cycle: bool,
                resume_empty_cycle: bool = False, fast: bool = False,
                isolated_base_zero_locked: bool = False,
                stage_start_only: bool = False) -> dict:
        if not self.ptp_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left PTP action unavailable")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("left gripper action unavailable")
        self.fast_execution = bool(fast)
        self.isolated_base_zero_locked = bool(isolated_base_zero_locked)
        plan = self.plan_empty_cycle(resume=resume_empty_cycle, fast=fast) if empty_cycle else self.plan(
            annotated_output, max_segment=1, require_center=not allow_off_center
        )
        report = {
            "status": "executing",
            "right_arm_commanded": False,
            "base_commanded": False,
            "isolated_base_zero_locked": self.isolated_base_zero_locked,
            "executed_segment": 1,
            "wrist_center_gate_overridden": bool(allow_off_center),
            "empty_cycle": bool(empty_cycle),
            "stage_start_only": bool(stage_start_only),
            "plan_snapshot": plan,
            "motions": [],
        }
        speed_scale = 4.0 if fast else 1.0
        for target_name, speed in (
            ("staging_above_pick", 0.025 * speed_scale),
            ("pick_height_retracted", 0.020 * speed_scale),
        ):
            for waypoint in plan["plans"][target_name]:
                report["motions"].append(self.move_ptp(
                    waypoint["joint_positions_rad"],
                    f"{target_name}_{waypoint['index']}",
                    speed,
                ))
        if stage_start_only:
            report["status"] = "stage_1_start_reached"
            report["gripper_commanded"] = False
            report["segment_1_executed"] = False
            return report
        report["gripper_open"] = self.command_gripper(
            float(self.cfg["empty_cycle"].get("open_position", 0.0)),
            "open_before_advance",
        )
        if not report["gripper_open"]["reached_goal"]:
            raise RuntimeError("left gripper did not reach the open position")
        for waypoint in plan["plans"]["advance_open_to_pad"]:
            report["motions"].append(self.move_ptp(
                waypoint["joint_positions_rad"],
                f"advance_open_to_pad_{waypoint['index']}",
                0.015 * speed_scale,
            ))
        self.motion_gate()
        report["gripper_close"] = self.command_gripper(
            float(self.cfg["empty_cycle"].get("closed_position", 0.7929)),
            "close_once_at_pad_end",
        )
        if not (
            report["gripper_close"]["reached_goal"]
            or report["gripper_close"]["stalled"]
        ):
            raise RuntimeError("left gripper neither closed nor stalled on contact")
        report["grasp_verification_skipped_by_user"] = True
        for target_name in ("lift_vertical_12cm", "carry_far_12cm"):
            for waypoint in plan["plans"][target_name]:
                report["motions"].append(self.move_ptp(
                    waypoint["joint_positions_rad"],
                    f"{target_name}_{waypoint['index']}",
                    0.02 * speed_scale,
                ))
        report["status"] = "segment_1_complete"
        report["held_at_segment_1_terminal_pose"] = True
        report["segment_2_executed"] = False
        return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="required physical-motion authorization")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--record", type=Path, default=DEFAULT_RECORD)
    parser.add_argument(
        "--allow-off-center",
        action="store_true",
        help="explicitly bypass only the wrist-image Y centering gate",
    )
    parser.add_argument(
        "--single-depth-frame",
        action="store_true",
        help="use one valid RGB/depth observation without temporal consistency checks",
    )
    parser.add_argument(
        "--empty-cycle",
        action="store_true",
        help="execute fixed segment 1 without camera localization or object verification",
    )
    parser.add_argument(
        "--resume-empty-cycle",
        action="store_true",
        help="resume the fixed empty cycle from the measured arm pose after a safe interruption",
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="use bounded waypoint compression and shorter settling waits",
    )
    parser.add_argument(
        "--isolated-base-zero-locked",
        action="store_true",
        help=(
            "assert that the localhost-only base host has been independently "
            "verified stationary and latched at zero velocity"
        ),
    )
    parser.add_argument(
        "--stage-start-only",
        action="store_true",
        help="move only to the configured segment-1 start pose; do not command the gripper",
    )
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required")
    if args.stage_start_only and not args.empty_cycle:
        parser.error("--stage-start-only requires --empty-cycle")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if args.single_depth_frame:
        config["camera"]["temporal_samples"] = 1
        config["camera"]["minimum_temporal_inliers"] = 1
        config["camera"]["maximum_sync_error_s"] = 10.0
        config["camera"]["maximum_temporal_point_residual_m"] = 10.0
    rclpy.init()
    node = ThermalPadExecutor(config)
    try:
        result = node.execute(
            ROOT / "outputs" / "thermal_pad_grasp_attempt.jpg",
            allow_off_center=args.allow_off_center,
            empty_cycle=args.empty_cycle,
            resume_empty_cycle=args.resume_empty_cycle,
            fast=args.fast,
            isolated_base_zero_locked=args.isolated_base_zero_locked,
            stage_start_only=args.stage_start_only,
        )
        code = 0
    except Exception as exc:
        result = {
            "status": "blocked",
            "error": str(exc),
            "right_arm_commanded": False,
            "base_commanded": False,
        }
        code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    args.record.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
