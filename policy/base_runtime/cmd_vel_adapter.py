#!/usr/bin/env python3
"""Fail-closed adapter from Task 2 mission commands to the base controller."""

from __future__ import annotations

import copy
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class Task2VelocityAdapter(Node):
    def __init__(self) -> None:
        super().__init__("tmr_task2_cmd_vel_adapter")
        self._lease_active = False
        self._last_command_at = 0.0
        self._publisher = self.create_publisher(
            TwistStamped, "/swerve_drive_controller/cmd_vel", 10
        )
        self.create_subscription(
            TwistStamped, "/tmr_cycle/mission_cmd_vel", self._on_command, 10
        )
        self.create_subscription(Bool, "/tmr_cycle/mission_active", self._on_lease, 10)
        self.create_timer(0.05, self._watchdog)
        self._publish_zero()

    @staticmethod
    def _finite(twist: Twist) -> bool:
        return all(
            math.isfinite(float(value))
            for value in (
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            )
        )

    def _publish(self, twist: Twist) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist = copy.deepcopy(twist)
        self._publisher.publish(message)

    def _publish_zero(self) -> None:
        self._publish(Twist())

    def _on_lease(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self._lease_active:
            self._lease_active = True
            self._last_command_at = 0.0
            self._publish_zero()
        elif not requested:
            self._lease_active = False
            self._last_command_at = 0.0
            self._publish_zero()

    def _on_command(self, message: TwistStamped) -> None:
        if not self._lease_active or not self._finite(message.twist):
            self._publish_zero()
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if (
            stamp_ns <= 0
            or message.header.frame_id.lstrip("/") != "base_link"
            or age_s > 0.35
            or age_s < -0.10
        ):
            self._publish_zero()
            return
        self._last_command_at = time.monotonic()
        self._publish(message.twist)

    def _watchdog(self) -> None:
        if self._lease_active and time.monotonic() - self._last_command_at > 0.50:
            self._publish_zero()

    def stop(self) -> None:
        for _ in range(20):
            self._publish_zero()
            rclpy.spin_once(self, timeout_sec=0.01)


def main() -> None:
    rclpy.init()
    node = Task2VelocityAdapter()

    def interrupt(_signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
