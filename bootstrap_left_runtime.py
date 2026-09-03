#!/usr/bin/env python3
"""Safely bootstrap the left Franka hold controller from measured joints."""

from __future__ import annotations

import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from lifecycle_msgs.msg import State
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"left_fr3v2_joint{i}" for i in range(1, 8)]
HARDWARE_NAME = "left_FrankaHardwareInterface"
STATE_BROADCASTERS = ("joint_state_broadcaster", "franka_robot_state_broadcaster")


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
        self.hardware_list = self.create_client(
            ListHardwareComponents, "/left/controller_manager/list_hardware_components"
        )
        self.controller_list = self.create_client(
            ListControllers, "/left/controller_manager/list_controllers"
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

    def hardware_state(self) -> str:
        if not self.hardware_list.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("left hardware-list service unavailable")
        response = self.call(self.hardware_list, ListHardwareComponents.Request(), 8.0)
        for component in response.component:
            if component.name == HARDWARE_NAME:
                return component.state.label
        raise RuntimeError(f"hardware component not found: {HARDWARE_NAME}")

    def controller_states(self) -> dict[str, str]:
        if not self.controller_list.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("left controller-list service unavailable")
        response = self.call(self.controller_list, ListControllers.Request(), 8.0)
        return {controller.name: controller.state for controller in response.controller}

    def set_hardware_active(self):
        if not self.hardware.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("left hardware lifecycle service unavailable")
        request = SetHardwareComponentState.Request()
        request.name = HARDWARE_NAME
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        request.target_state.label = "active"
        response = self.call(self.hardware, request, 18.0)
        if not response.ok or response.state.id != State.PRIMARY_STATE_ACTIVE:
            raise RuntimeError(
                f"left hardware activation failed: ok={response.ok}, state={response.state.label}"
            )

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
        if not activate and not deactivate:
            return
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
            raise RuntimeError(
                f"controller switch failed: activate={list(activate)}, "
                f"deactivate={list(deactivate)}"
            )

    def ensure_controllers(self, required) -> None:
        states = self.controller_states()
        missing = [name for name in required if name not in states]
        if missing:
            raise RuntimeError(f"required left controllers are not loaded: {missing}")
        inactive = [name for name in required if states[name] != "active"]
        self.switch_controllers(inactive, strictness=2)

    def recover_to_active(self) -> None:
        # Franka's documented recovery sequence is recovery first, then
        # hardware activation, then controller activation.  Deactivating any
        # still-active controller also makes this safe on ROS 2 Humble while
        # remaining harmless on Jazzy (where controllers normally deactivate
        # automatically after a hardware error).
        states = self.controller_states()
        active = [name for name, state in states.items() if state == "active"]
        self.switch_controllers([], active, strictness=1)
        self.recover()
        self.set_hardware_active()
        self.ensure_controllers(STATE_BROADCASTERS)

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
        initial_hardware_state = self.hardware_state()
        recovery_performed = False
        if initial_hardware_state != "active":
            self.recover_to_active()
            recovery_performed = True
        else:
            self.ensure_controllers(STATE_BROADCASTERS)
        try:
            self.wait_measured()
        except RuntimeError:
            if recovery_performed:
                raise
            self.recover_to_active()
            recovery_performed = True
            self.wait_measured()
        if self.active_errors():
            self.recover_to_active()
            recovery_performed = True
            self.wait_measured()
            errors = self.active_errors()
            if errors:
                raise RuntimeError("left errors after recovery: " + ",".join(errors))
        measured = list(self.q)
        if state_only:
            return {
                "status": "state_only_ready",
                "initial_hardware_state": initial_hardware_state,
                "final_hardware_state": self.hardware_state(),
                "recovery_performed": recovery_performed,
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
