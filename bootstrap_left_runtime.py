#!/usr/bin/env python3
"""Safely bootstrap the left Franka hold controller from measured joints."""

from __future__ import annotations

import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from lifecycle_msgs.msg import State
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"left_fr3v2_joint{i}" for i in range(1, 8)]


class Bootstrap(Node):
    def __init__(self):
        super().__init__("left_runtime_measured_hold_bootstrap")
        self.q = None
        self.state = None
        self.create_subscription(
            JointState,
            "/left/franka_robot_state_broadcaster/measured_joint_states",
            self._joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FrankaRobotState,
            "/left/franka_robot_state_broadcaster/robot_state",
            self._state,
            qos_profile_sensor_data,
        )
        self.hold = self.create_publisher(JointState, "/left/gello/joint_states", 1)
        self.hardware = self.create_client(
            SetHardwareComponentState, "/left/controller_manager/set_hardware_component_state"
        )
        self.switch = self.create_client(
            SwitchController, "/left/controller_manager/switch_controller"
        )
        self.recovery = ActionClient(self, ErrorRecovery, "/left/action_server/error_recovery")

    def _joints(self, message):
        mapped = dict(zip(message.name, message.position))
        if all(name in mapped for name in JOINT_NAMES):
            self.q = [float(mapped[name]) for name in JOINT_NAMES]

    def _state(self, message):
        self.state = message

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("ROS service timeout")
        return future.result()

    def set_hardware_active(self):
        if not self.hardware.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("left hardware lifecycle service unavailable")
        for _ in range(2):
            request = SetHardwareComponentState.Request()
            request.name = "left_FrankaHardwareInterface"
            request.target_state.id = State.PRIMARY_STATE_ACTIVE
            request.target_state.label = "active"
            response = self.call(self.hardware, request, 12.0)
            if response.state.id == State.PRIMARY_STATE_ACTIVE:
                return
        raise RuntimeError(f"left hardware activation failed: {response.state.label}")

    def recover(self):
        if not self.recovery.wait_for_server(timeout_sec=4.0):
            raise RuntimeError("left error-recovery action unavailable")
        future = self.recovery.send_goal_async(ErrorRecovery.Goal())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("left error-recovery goal rejected")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=12.0)
        wrapped = result.result() if result.done() else None
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("left error recovery failed")

    def switch_controllers(self, activate, deactivate=(), strictness=2):
        if not self.switch.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("left controller switch service unavailable")
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = int(strictness)
        request.activate_asap = False
        request.timeout.sec = 5
        response = self.call(self.switch, request, 8.0)
        if not response.ok:
            raise RuntimeError(f"controller activation failed: {list(activate)}")

    def wait_measured(self, timeout=5.0):
        self.q = self.state = None
        deadline = time.monotonic() + timeout
        while (self.q is None or self.state is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.q is None or self.state is None:
            raise RuntimeError("fresh measured left-arm state unavailable")

    def publish_hold(self, count=1):
        if self.q is None:
            raise RuntimeError("cannot hold without measured joints")
        message = JointState()
        message.name = JOINT_NAMES
        message.position = list(self.q)
        for _ in range(count):
            message.header.stamp = self.get_clock().now().to_msg()
            self.hold.publish(message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / 30.0)

    def active_errors(self):
        errors = self.state.current_errors
        return [name for name in errors.get_fields_and_field_types() if getattr(errors, name)]

    def run(self, state_only=False):
        self.set_hardware_active()
        self.recover()
        self.switch_controllers(
            ["joint_state_broadcaster", "franka_robot_state_broadcaster"],
            strictness=1,
        )
        self.wait_measured()
        if self.active_errors():
            self.recover()
            self.wait_measured()
        measured = list(self.q)
        if state_only:
            return {
                "status": "state_only_ready",
                "measured_joint_positions_rad": measured,
                "impedance_controller_activated": False,
            }
        deadline = time.monotonic() + 2.0
        while self.hold.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.hold.get_subscription_count() < 1:
            raise RuntimeError("left hold controller subscription unavailable")
        # Messages received while the controller is inactive are intentionally
        # ignored by its implementation. Keep publishing at a real 30 Hz after
        # activation so its first accepted goal is the measured posture.
        self.publish_hold(10)
        self.switch_controllers(["joint_impedance_controller"], strictness=2)
        self.publish_hold(120)
        self.wait_measured()
        errors = self.active_errors()
        if errors:
            raise RuntimeError("left errors after hold activation: " + ",".join(errors))
        return {
            "status": "ready",
            "measured_hold_joint_positions_rad": measured,
            "final_joint_positions_rad": list(self.q),
            "maximum_hold_delta_rad": max(abs(a - b) for a, b in zip(self.q, measured)),
            "timestamped_hold_samples": 130,
        }


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-only", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = Bootstrap()
    try:
        print(json.dumps(node.run(state_only=args.state_only), indent=2), flush=True)
        return 0
    except Exception as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2), flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
