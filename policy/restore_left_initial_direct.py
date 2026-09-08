#!/usr/bin/env python3
"""Move the left FR3 to the recorded initial joints without impedance startup."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from franka_msgs.action import PTPMotion
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"left_fr3v2_joint{i}" for i in range(1, 8)]
TARGET = [
    -1.71976900100708, -1.6329213380813599, 1.8240526914596558,
    -2.447446823120117, 2.177191972732544, 0.8496646285057068,
    -3.05077862739563,
]
RESTORE_SPEED_RAD_S = 0.08


class DirectRestore(Node):
    def __init__(self, side: str = "left"):
        if side not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")
        super().__init__(f"{side}_initial_direct_ptp")
        self.side = side
        self.joint_names = [f"{side}_fr3v2_joint{i}" for i in range(1, 8)]
        self.q = None
        self.create_subscription(
            JointState,
            f"/{side}/franka_robot_state_broadcaster/measured_joint_states",
            self._joints,
            qos_profile_sensor_data,
        )
        self.action = ActionClient(self, PTPMotion, f"/{side}/action_server/ptp_motion")

    def _joints(self, message):
        mapped = dict(zip(message.name, message.position))
        if all(name in mapped for name in self.joint_names):
            self.q = [float(mapped[name]) for name in self.joint_names]

    def wait_state(self, timeout=5.0):
        self.q = None
        deadline = time.monotonic() + timeout
        while self.q is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.q is None:
            raise RuntimeError(f"{self.side} measured joints unavailable")

    def run(self, target=None, speed_rad_s=RESTORE_SPEED_RAD_S):
        target = list(TARGET if target is None else map(float, target))
        self.wait_state()
        start = list(self.q)
        if not self.action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{self.side} PTP action unavailable")
        goal = PTPMotion.Goal()
        goal.goal_joint_configuration = target
        goal.maximum_joint_velocities = [float(speed_rad_s)] * 7
        goal.goal_tolerance = 0.004
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("left initial PTP goal rejected")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 100.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        wrapped = result_future.result() if result_future.done() else None
        if wrapped is None:
            handle.cancel_goal_async()
            raise RuntimeError("left initial PTP timeout")
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{self.side} initial PTP did not succeed (action_status={wrapped.status})"
            )
        target_status = int(wrapped.result.target_status.status)
        if target_status != wrapped.result.target_status.TARGET_REACHED:
            raise RuntimeError(
                "left initial PTP target was not reached: "
                f"target_status={target_status}, error={wrapped.result.error_message}"
            )
        self.wait_state(timeout=5.0)
        error = max(abs(a - b) for a, b in zip(self.q, target))
        if error > 0.012:
            raise RuntimeError(
                f"{self.side} initial endpoint error {error:.6f} rad (status={wrapped.status})"
            )
        return {
            "status": "success",
            "action_status": int(wrapped.status),
            "action_succeeded": wrapped.status == GoalStatus.STATUS_SUCCEEDED,
            "start_joint_positions_rad": start,
            "target_joint_positions_rad": target,
            "measured_joint_positions_rad": list(self.q),
            "maximum_joint_error_rad": error,
            "impedance_controller_commanded": False,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-joints", type=float, nargs=7)
    parser.add_argument("--speed-rad-s", type=float, default=RESTORE_SPEED_RAD_S)
    args = parser.parse_args()
    rclpy.init()
    node = DirectRestore()
    try:
        print(json.dumps(node.run(args.target_joints, args.speed_rad_s), indent=2), flush=True)
        return 0
    except Exception as exc:
        print(json.dumps({"status": "blocked", "error": str(exc)}, indent=2), flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
