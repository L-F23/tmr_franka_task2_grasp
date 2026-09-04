#!/usr/bin/env python3
"""Base-local CCW rotation and rear-wall docking controller for Task 2.

This file is streamed to the base computer and runs inside its isolated ROS 2
graph.  Rear-wall range is measured from the robot body rear face, not from the
odometry origin.  LiDAR is used for wall pose estimation only; it is not used
as a general collision guard in this stage.
"""

from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


ODOM_TOPIC = "/swerve_drive_controller/odom"
COMMAND_TOPIC = "/tmr_cycle/mission_cmd_vel"
LEASE_TOPIC = "/tmr_cycle/mission_active"
SCAN_TOPICS = ("/lidar_front/scan", "/lidar_rear/scan")
LIDAR_EXTRINSICS = {
    "lidar_front": (0.3275, 0.2175, 0.7846018366025517),
    "lidar_rear": (-0.3275, -0.2175, -2.3569908169872414),
}
REAR_BODY_EXTENT_M = 0.40


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(quaternion) -> float:
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        ),
    )


def fit_rear_wall(points: list[tuple[float, float]]) -> dict:
    """Robustly fit rear wall ``x = slope*y + intercept`` in base_link."""
    values = np.asarray([
        (x, y) for x, y in points
        if -1.60 <= x <= -0.42 and abs(y) <= 1.35
    ], dtype=float)
    if values.shape[0] < 30:
        raise RuntimeError("too few rear-sector LiDAR points for wall fitting")

    x_values, y_values = values[:, 0], values[:, 1]
    generator = np.random.default_rng(0)
    best: tuple[float, np.ndarray, float, float] | None = None
    for _ in range(min(240, values.shape[0] * 4)):
        first, second = generator.choice(values.shape[0], size=2, replace=False)
        dy = y_values[second] - y_values[first]
        if abs(dy) < 0.18:
            continue
        slope = (x_values[second] - x_values[first]) / dy
        if abs(slope) > math.tan(math.radians(25.0)):
            continue
        intercept = x_values[first] - slope * y_values[first]
        residuals = np.abs(x_values - (slope * y_values + intercept))
        inliers = residuals <= 0.035
        count = int(np.count_nonzero(inliers))
        if count < 24:
            continue
        span = float(np.ptp(y_values[inliers]))
        if span < 0.55:
            continue
        score = count + 30.0 * span
        if best is None or score > best[0]:
            best = (score, inliers, slope, intercept)
    if best is None:
        raise RuntimeError("rear wall has no stable linear support")

    inliers = best[1]
    slope, intercept = np.polyfit(y_values[inliers], x_values[inliers], 1)
    residuals = np.abs(x_values - (slope * y_values + intercept))
    inliers = residuals <= 0.030
    if int(np.count_nonzero(inliers)) < 24:
        raise RuntimeError("rear wall refit lost support")
    lateral_span = float(np.ptp(y_values[inliers]))
    if lateral_span < 0.55:
        raise RuntimeError("rear wall support is too narrow")
    median_residual = float(np.median(residuals[inliers]))
    return {
        "slope_x_per_y": float(slope),
        "intercept_x_m": float(intercept),
        "wall_angle_error_deg": math.degrees(math.atan(float(slope))),
        "wall_distance_from_base_origin_m": -float(intercept),
        "rear_body_clearance_m": -float(intercept) - REAR_BODY_EXTENT_M,
        "inlier_count": int(np.count_nonzero(inliers)),
        "lateral_support_m": lateral_span,
        "median_residual_m": median_residual,
    }


def wall_control_targets(
    rear_clearance_m: float,
    wall_angle_error_deg: float,
    target_clearance_m: float,
    maximum_linear_speed_m_s: float,
    maximum_yaw_speed_rad_s: float,
) -> tuple[float, float]:
    """Return body-forward and yaw commands with explicitly tested signs."""
    gap_error = float(rear_clearance_m) - float(target_clearance_m)
    angle_error = math.radians(float(wall_angle_error_deg))
    return (
        clamp(-0.9 * gap_error, -maximum_linear_speed_m_s, maximum_linear_speed_m_s),
        clamp(-1.2 * angle_error, -maximum_yaw_speed_rad_s, maximum_yaw_speed_rad_s),
    )


def wall_distance_intent(
    rear_clearance_m: float,
    target_clearance_m: float,
    tolerance_m: float = 0.018,
) -> dict:
    """Describe the required fore/aft correction without commanding it."""
    signed_error = float(rear_clearance_m) - float(target_clearance_m)
    if abs(signed_error) <= float(tolerance_m):
        direction = "hold"
        distance_m = 0.0
    elif signed_error > 0.0:
        direction = "backward"
        distance_m = signed_error
    else:
        direction = "forward"
        distance_m = -signed_error
    return {
        "direction": direction,
        "distance_m": distance_m,
        "current_rear_body_clearance_m": float(rear_clearance_m),
        "target_rear_body_clearance_m": float(target_clearance_m),
        "signed_clearance_error_m": signed_error,
        "translation_commanded": False,
    }


class WallDockingController(Node):
    def __init__(self) -> None:
        super().__init__("tmr_task2_wall_docking")
        self.pose = None
        self.velocity = None
        self.odom_at = 0.0
        self.last_yaw = None
        self.unwrapped_yaw = None
        self.scan_points: dict[str, tuple[float, list[tuple[float, float]]]] = {}
        self.command = [0.0, 0.0, 0.0]
        self.command_pub = self.create_publisher(TwistStamped, COMMAND_TOPIC, 10)
        self.lease_pub = self.create_publisher(Bool, LEASE_TOPIC, 10)
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data
        )
        for topic in SCAN_TOPICS:
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, source=topic: self._on_scan(source, message),
                qos_profile_sensor_data,
            )

    def _on_odom(self, message: Odometry) -> None:
        yaw = yaw_of(message.pose.pose.orientation)
        if self.last_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            self.unwrapped_yaw += wrap(yaw - self.last_yaw)
        self.last_yaw = yaw
        position = message.pose.pose.position
        self.pose = (float(position.x), float(position.y), yaw)
        twist = message.twist.twist
        self.velocity = (
            float(twist.linear.x), float(twist.linear.y),
            float(twist.angular.z),
        )
        self.odom_at = time.monotonic()

    def _on_scan(self, _topic: str, message: LaserScan) -> None:
        frame = message.header.frame_id.lstrip("/")
        if frame not in LIDAR_EXTRINSICS:
            return
        tx, ty, yaw = LIDAR_EXTRINSICS[frame]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        points = []
        angle = float(message.angle_min)
        for index, distance in enumerate(message.ranges):
            if (
                index % 2 == 0
                and math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                scan_x = distance * math.cos(angle)
                scan_y = distance * math.sin(angle)
                points.append((
                    tx + cosine * scan_x + sine * scan_y,
                    ty + sine * scan_x - cosine * scan_y,
                ))
            angle += float(message.angle_increment)
        self.scan_points[frame] = (time.monotonic(), points)

    def publish(self, vx: float, vy: float, wz: float) -> None:
        lease = Bool()
        lease.data = True
        self.lease_pub.publish(lease)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self.command_pub.publish(message)

    def stop(self) -> None:
        self.command[:] = (0.0, 0.0, 0.0)
        for _ in range(30):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.025)

    def wait_ready(self, *, require_wall: bool) -> None:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            stopped = self.velocity is not None and (
                math.hypot(self.velocity[0], self.velocity[1]) <= 0.015
                and abs(self.velocity[2]) <= 0.03
            )
            scans_ready = not require_wall or all(
                frame in self.scan_points and now - self.scan_points[frame][0] <= 0.35
                for frame in LIDAR_EXTRINSICS
            )
            if (
                self.pose is not None
                and now - self.odom_at < 0.35
                and stopped
                and scans_ready
                and self.command_pub.get_subscription_count() >= 1
            ):
                return
        raise RuntimeError(
            "fresh odometry, stationary base, wall LiDAR, or command adapter unavailable"
        )

    def fresh_scan_points(self, now: float | None = None) -> list[tuple[float, float]]:
        current = time.monotonic() if now is None else now
        points = []
        for captured_at, values in self.scan_points.values():
            if current - captured_at <= 0.35:
                points.extend(values)
        return points

    def require_fresh_scan_points(self) -> list[tuple[float, float]]:
        now = time.monotonic()
        missing = [
            frame for frame in LIDAR_EXTRINSICS
            if frame not in self.scan_points or now - self.scan_points[frame][0] > 0.35
        ]
        if missing:
            raise RuntimeError(
                "dual LiDAR wall evidence became stale: " + ",".join(missing)
            )
        return self.fresh_scan_points(now)

    def observe_rear_wall(self, sample_count: int = 15) -> dict:
        """Measure the rear wall repeatedly without publishing a motion command."""
        self.wait_ready(require_wall=True)
        observations = []
        for _ in range(sample_count):
            rclpy.spin_once(self, timeout_sec=0.06)
            observations.append(fit_rear_wall(self.require_fresh_scan_points()))
        numeric_keys = (
            "slope_x_per_y",
            "intercept_x_m",
            "wall_angle_error_deg",
            "wall_distance_from_base_origin_m",
            "rear_body_clearance_m",
            "lateral_support_m",
            "median_residual_m",
        )
        wall = {
            key: float(np.median([item[key] for item in observations]))
            for key in numeric_keys
        }
        wall["inlier_count"] = int(round(float(np.median([
            item["inlier_count"] for item in observations
        ]))))
        slope = wall["slope_x_per_y"]
        intercept = wall["intercept_x_m"]
        left_y_m, right_y_m = 0.50, -0.50
        left_clearance = -(slope * left_y_m + intercept) - REAR_BODY_EXTENT_M
        right_clearance = -(slope * right_y_m + intercept) - REAR_BODY_EXTENT_M
        angle = wall["wall_angle_error_deg"]
        return {
            "status": "success",
            "phase": "rear_wall_observed",
            "physical_motion_commanded": False,
            "sample_count": sample_count,
            "final_wall": wall,
            "parallel_assessment": {
                "tolerance_deg": 1.8,
                "is_parallel": abs(angle) <= 1.8,
                "wall_angle_error_deg": angle,
                "left_sample_y_m": left_y_m,
                "right_sample_y_m": right_y_m,
                "left_rear_body_clearance_m": left_clearance,
                "right_rear_body_clearance_m": right_clearance,
                "left_minus_right_clearance_m": left_clearance - right_clearance,
                "yaw_correction_intent": (
                    "hold" if abs(angle) <= 1.8
                    else ("clockwise" if angle > 0.0 else "counterclockwise")
                ),
            },
            "distance_intent_for_0_22m": wall_distance_intent(
                wall["rear_body_clearance_m"], 0.22
            ),
        }

    def slew(self, desired: tuple[float, float, float], last_tick: float) -> float:
        now = time.monotonic()
        delta = clamp(now - last_tick, 0.02, 0.10)
        for index, (target, acceleration) in enumerate(
            zip(desired, (0.12, 0.12, 0.28))
        ):
            step = acceleration * delta
            self.command[index] += clamp(
                target - self.command[index], -step, step
            )
        self.publish(*self.command)
        return now

    def rotate_ccw(self, degrees: float, speed_rps: float, timeout_s: float) -> dict:
        self.wait_ready(require_wall=False)
        start_pose = tuple(self.pose)
        start_unwrapped = float(self.unwrapped_yaw)
        target = start_unwrapped + math.radians(degrees)
        deadline = time.monotonic() + timeout_s
        stable_since = None
        last_tick = time.monotonic()
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                if self.pose is None or time.monotonic() - self.odom_at > 0.35:
                    raise RuntimeError("odometry became stale during rotation")
                error = target - float(self.unwrapped_yaw)
                if abs(error) <= math.radians(0.7):
                    self.publish(0.0, 0.0, 0.0)
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.35:
                        break
                    continue
                stable_since = None
                yaw = self.pose[2]
                world_x = start_pose[0] - self.pose[0]
                world_y = start_pose[1] - self.pose[1]
                desired = (
                    clamp(math.cos(yaw) * world_x + math.sin(yaw) * world_y, -0.015, 0.015),
                    clamp(-math.sin(yaw) * world_x + math.cos(yaw) * world_y, -0.015, 0.015),
                    clamp(1.15 * error, -speed_rps, speed_rps),
                )
                last_tick = self.slew(desired, last_tick)
            else:
                raise TimeoutError("CCW rotation timed out")
        finally:
            self.stop()
        actual = float(self.unwrapped_yaw) - start_unwrapped
        report = {
            "status": "success",
            "phase": "rotate_ccw",
            "requested_ccw_deg": degrees,
            "actual_ccw_deg": math.degrees(actual),
            "error_deg": math.degrees(actual) - degrees,
            "position_drift_m": math.hypot(
                self.pose[0] - start_pose[0], self.pose[1] - start_pose[1]
            ),
        }
        if abs(report["error_deg"]) > 1.5 or report["position_drift_m"] > 0.05:
            raise RuntimeError("rotation endpoint outside tolerance: " + json.dumps(report))
        return report

    def align_rear_wall(
        self,
        clearance_m: float,
        speed_mps: float,
        yaw_speed_rps: float,
        timeout_s: float,
        *,
        orientation_only: bool = False,
        target_wall_angle_deg: float = 0.0,
        clearance_tolerance_m: float = 0.012,
        angle_tolerance_deg: float = 1.2,
    ) -> dict:
        self.wait_ready(require_wall=True)
        deadline = time.monotonic() + timeout_s
        stable_since = None
        last_tick = time.monotonic()
        observations = []
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.04)
                if self.pose is None or time.monotonic() - self.odom_at > 0.35:
                    raise RuntimeError("odometry became stale during wall alignment")
                observation = fit_rear_wall(self.require_fresh_scan_points())
                observations.append(observation)
                gap_error = observation["rear_body_clearance_m"] - clearance_m
                angle_error_deg = (
                    observation["wall_angle_error_deg"] - target_wall_angle_deg
                )
                angle_error = math.radians(angle_error_deg)
                distance_ready = (
                    orientation_only or abs(gap_error) <= clearance_tolerance_m
                )
                if (
                    distance_ready
                    and abs(angle_error_deg) <= angle_tolerance_deg
                ):
                    self.publish(0.0, 0.0, 0.0)
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.50:
                        break
                    continue
                stable_since = None
                # Positive gap error means the wall is too far behind, so move
                # backward. Positive wall angle means the base needs CW yaw.
                desired_vx, desired_wz = wall_control_targets(
                    observation["rear_body_clearance_m"],
                    angle_error_deg,
                    clearance_m,
                    speed_mps,
                    yaw_speed_rps,
                )
                if orientation_only:
                    desired_vx = 0.0
                desired = (desired_vx, 0.0, desired_wz)
                last_tick = self.slew(desired, last_tick)
            else:
                raise TimeoutError("rear-wall alignment timed out")
        finally:
            self.stop()
        final = fit_rear_wall(self.require_fresh_scan_points())
        if (
            abs(final["wall_angle_error_deg"] - target_wall_angle_deg)
            > max(0.25, angle_tolerance_deg + 0.1)
        ):
            raise RuntimeError("rear-wall endpoint outside tolerance: " + json.dumps(final))
        if (
            not orientation_only
            and abs(final["rear_body_clearance_m"] - clearance_m)
            > max(0.006, clearance_tolerance_m + 0.002)
        ):
            raise RuntimeError("rear-wall endpoint outside tolerance: " + json.dumps(final))
        intent = wall_distance_intent(final["rear_body_clearance_m"], clearance_m)
        return {
            "status": (
                "awaiting_distance_confirmation" if orientation_only else "success"
            ),
            "phase": (
                "rear_wall_heading_aligned" if orientation_only else "rear_wall_aligned"
            ),
            "target_rear_body_clearance_m": clearance_m,
            "target_wall_angle_error_deg": target_wall_angle_deg,
            "final_wall": final,
            "distance_intent": intent,
            "distance_translation_commanded": not orientation_only,
            "observation_count": len(observations),
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("rotate", "wall-align", "observe"), required=True
    )
    parser.add_argument("--ccw-deg", type=float, default=90.0)
    parser.add_argument("--wall-clearance-m", type=float, default=0.22)
    parser.add_argument("--speed-mps", type=float, default=0.025)
    parser.add_argument("--yaw-speed-rps", type=float, default=0.14)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--orientation-only", action="store_true")
    parser.add_argument("--wall-angle-deg", type=float, default=0.0)
    parser.add_argument("--clearance-tolerance-m", type=float, default=0.012)
    parser.add_argument("--angle-tolerance-deg", type=float, default=1.2)
    parser.add_argument("--observe-samples", type=int, default=15)
    args = parser.parse_args()
    if not -180.0 <= args.ccw_deg <= 180.0 or abs(args.ccw_deg) < 1.0:
        parser.error("--ccw-deg must be in [-180, -1] or [1, 180]")
    if not 0.10 <= args.wall_clearance_m <= 0.60:
        parser.error("--wall-clearance-m must be in [0.10, 0.60]")
    if not 0.008 <= args.speed_mps <= 0.04:
        parser.error("--speed-mps must be in [0.008, 0.04]")
    if not 0.04 <= args.yaw_speed_rps <= 0.20:
        parser.error("--yaw-speed-rps must be in [0.04, 0.20]")
    if not -10.0 <= args.wall_angle_deg <= 10.0:
        parser.error("--wall-angle-deg must be in [-10, 10]")
    if not 0.002 <= args.clearance_tolerance_m <= 0.018:
        parser.error("--clearance-tolerance-m must be in [0.002, 0.018]")
    if not 0.1 <= args.angle_tolerance_deg <= 1.8:
        parser.error("--angle-tolerance-deg must be in [0.1, 1.8]")

    rclpy.init()
    node = WallDockingController()
    try:
        if args.mode == "observe":
            if not 3 <= args.observe_samples <= 100:
                parser.error("--observe-samples must be in [3, 100]")
            result = node.observe_rear_wall(args.observe_samples)
        elif args.mode == "rotate":
            result = node.rotate_ccw(
                args.ccw_deg, args.yaw_speed_rps, args.timeout_s
            )
        else:
            result = node.align_rear_wall(
                args.wall_clearance_m,
                args.speed_mps,
                args.yaw_speed_rps,
                args.timeout_s,
                orientation_only=args.orientation_only,
                target_wall_angle_deg=args.wall_angle_deg,
                clearance_tolerance_m=args.clearance_tolerance_m,
                angle_tolerance_deg=args.angle_tolerance_deg,
            )
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        node.stop()
        print(json.dumps({"status": "failed", "error": repr(exc)}, indent=2), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
