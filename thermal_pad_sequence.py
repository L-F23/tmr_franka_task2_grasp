#!/usr/bin/env python3
"""Pure geometry for the configured thermal-pad pick/lift/place/release sequence."""

from __future__ import annotations

import math

import numpy as np


class SequenceDesignError(ValueError):
    pass


def _unit(values, label: str) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    norm = float(np.linalg.norm(vector))
    if vector.shape != (3,) or not math.isfinite(norm) or norm < 1e-9:
        raise SequenceDesignError(f"{label} must be a finite non-zero XYZ vector")
    return vector / norm


def normalize_quaternion(values) -> np.ndarray:
    quaternion = np.asarray(values, dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if quaternion.shape != (4,) or not math.isfinite(norm) or norm < 1e-9:
        raise SequenceDesignError("orientation must be a finite non-zero XYZW quaternion")
    return quaternion / norm


def quaternion_multiply(left, right) -> np.ndarray:
    """Hamilton product for XYZW quaternions."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ])


def axis_angle_quaternion(axis, angle_rad: float) -> np.ndarray:
    axis = _unit(axis, "rotation axis")
    half = float(angle_rad) / 2.0
    return np.r_[axis * math.sin(half), math.cos(half)]


def quaternion_matrix(quaternion) -> np.ndarray:
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def slerp(start, target, fraction: float) -> np.ndarray:
    start = normalize_quaternion(start)
    target = normalize_quaternion(target)
    dot = float(np.dot(start, target))
    if dot < 0.0:
        target = -target
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(start + float(fraction) * (target - start))
    angle = math.acos(dot)
    scale = math.sin(angle)
    return normalize_quaternion(
        math.sin((1.0 - fraction) * angle) / scale * start
        + math.sin(fraction * angle) / scale * target
    )


def tool_down_axis(quaternion) -> np.ndarray:
    """Return link8 local +Z expressed in the robot base frame."""
    return quaternion_matrix(quaternion)[:, 2]


def tilt_tool_toward(quaternion, direction, angle_deg: float) -> np.ndarray:
    """Tilt the tool-down axis toward a base-frame horizontal direction."""
    direction = _unit(direction, "tilt direction")
    down = tool_down_axis(quaternion)
    horizontal = direction - np.dot(direction, down) * down
    horizontal = _unit(horizontal, "tilt direction projected normal to tool axis")
    target_down = math.cos(math.radians(angle_deg)) * down + math.sin(
        math.radians(angle_deg)
    ) * horizontal
    axis = _unit(np.cross(down, target_down), "tilt rotation axis")
    rotation = axis_angle_quaternion(axis, math.radians(angle_deg))
    return quaternion_multiply(rotation, quaternion)


def build_sequence(grasp_position, vertical_orientation, config: dict) -> dict:
    """Build named pose targets and gripper events without solving IK or moving hardware."""
    grasp = np.asarray(grasp_position, dtype=float)
    if grasp.shape != (3,) or not np.all(np.isfinite(grasp)):
        raise SequenceDesignError("grasp position must be finite XYZ")
    orientation = normalize_quaternion(vertical_orientation)
    reference_frame = str(config["ground_aligned_frame"])
    shoulder_frame = str(config["shoulder_frame"])
    if not reference_frame or not shoulder_frame or reference_frame == shoulder_frame:
        raise SequenceDesignError(
            "ground-aligned reference frame must be explicit and distinct from the arm shoulder frame"
        )
    if int(config.get("segment_boundary_after_step", -1)) != 6:
        raise SequenceDesignError("the configured segment boundary must remain after step 6")
    if config.get("segment_1_terminal_target") != "carry_far_12cm":
        raise SequenceDesignError("segment 1 must terminate at carry_far_12cm")
    forward = _unit(config["ground_forward_axis_xyz"], "ground-frame forward axis")
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, up))) > 1e-6:
        raise SequenceDesignError("ground-frame forward axis must be horizontal")

    vertical_error = math.degrees(math.acos(float(np.clip(np.dot(tool_down_axis(orientation), -up), -1, 1))))
    if vertical_error > float(config["maximum_tool_vertical_error_deg"]):
        raise SequenceDesignError(
            f"gripper is not vertical to the table ({vertical_error:.2f} deg error)"
        )

    positive = (
        "open_advance_m", "staging_clearance_z_m", "lift_vertical_m",
        "carry_far_m", "pre_place_lower_m", "diagonal_down_m",
        "diagonal_inward_m", "release_inward_m", "release_forward_tilt_deg",
    )
    if any(float(config[name]) <= 0.0 for name in positive):
        raise SequenceDesignError("all configured sequence distances and angles must be positive")

    pick_level = grasp - forward * float(config["open_advance_m"])
    staging = pick_level + up * float(config["staging_clearance_z_m"])
    lifted = grasp + up * float(config["lift_vertical_m"])
    carried_far = lifted + forward * float(config["carry_far_m"])
    lowered_for_place = carried_far - up * float(config["pre_place_lower_m"])
    diagonal = (
        lowered_for_place
        - up * float(config["diagonal_down_m"])
        - forward * float(config["diagonal_inward_m"])
    )
    release = diagonal - forward * float(config["release_inward_m"])
    release_orientation = tilt_tool_toward(
        orientation, forward, float(config["release_forward_tilt_deg"])
    )

    def pose(name, position, segment, step, quaternion=orientation, semantics=""):
        return {
            "name": name,
            "segment": int(segment),
            "step": int(step),
            "position_m": np.asarray(position, dtype=float).tolist(),
            "orientation_xyzw": normalize_quaternion(quaternion).tolist(),
            "semantics": semantics,
        }

    targets = [
        pose("staging_above_pick", staging, 1, 1,
             semantics="wrist flange horizontal; gripper vertical"),
        pose("pick_height_retracted", pick_level, 1, 2,
             semantics="same ground-frame Z as the previously detected grasp endpoint"),
        pose("advance_open_to_pad", grasp, 1, 3,
             semantics="advance along ground-frame +X while gripper remains open"),
        pose("lift_vertical_12cm", lifted, 1, 5),
        pose("carry_far_12cm", carried_far, 1, 6,
             semantics="segment 1 ends here and holds this pose"),
        pose("lower_vertical_22cm", lowered_for_place, 2, 7),
        pose("lower_and_retract", diagonal, 2, 8,
             semantics="simultaneous ground-frame -Z and -X translation"),
        pose(
            "tilt_forward_and_retract_release",
            release,
            2,
            10,
            release_orientation,
            semantics="simultaneous forward tool tilt and ground-frame -X retraction",
        ),
    ]
    segment_1_names = [target["name"] for target in targets if target["segment"] == 1]
    segment_2_names = [target["name"] for target in targets if target["segment"] == 2]
    return {
        "reference_frame": reference_frame,
        "shoulder_frame_forbidden_for_offsets": shoulder_frame,
        "coordinate_semantics": (
            "ground-aligned robot frame: +X forward/far, -X inward, +Z up; "
            "FCI shoulder-local axes are never used for these offsets"
        ),
        "gripper_vertical_error_deg": vertical_error,
        "parameters_calibrated": bool(config.get("parameters_calibrated", False)),
        "execution_ready": bool(config.get("parameters_calibrated", False)),
        "execution_blockers": [] if config.get("parameters_calibrated", False) else [
            "diagonal placement distances and release tilt require physical calibration",
            "post-lift object-held verification is not yet implemented",
        ],
        "targets": targets,
        "segments": [
            {
                "id": 1,
                "name": "pick_lift_and_far_transfer",
                "steps": [1, 2, 3, 4, 5, 6],
                "target_names": segment_1_names,
                "terminal_target": "carry_far_12cm",
                "terminal_behavior": "hold_pose_and_wait_for_explicit_segment_2_authorization",
            },
            {
                "id": 2,
                "name": "lower_place_and_release",
                "steps": [7, 8, 9, 10],
                "target_names": segment_2_names,
                "start_gate": "segment_1_terminal_pose_and_object_hold_reconfirmed",
            },
        ],
        "events": [
            {"step": 3, "after_target": "pick_height_retracted", "command": "open_gripper"},
            {"step": 4, "after_target": "advance_open_to_pad", "command": "close_gripper"},
            {"step": 5, "after_target": "lift_vertical_12cm", "command": "verify_object_held"},
            {"step": 6, "after_target": "carry_far_12cm", "command": "hold_and_end_segment_1"},
            {"step": 7, "before_target": "lower_vertical_22cm", "command": "authorize_segment_2"},
            {"step": 9, "after_target": "lower_and_retract", "command": "open_gripper"},
            {"step": 10, "after_target": "tilt_forward_and_retract_release", "command": "verify_release"},
        ],
    }
