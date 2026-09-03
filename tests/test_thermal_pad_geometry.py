import cv2
import numpy as np

from thermal_pad_geometry import (
    Intrinsics,
    detect_pad_end,
    pose_matrix,
    register_depth_point,
    transform_point,
)


def test_detects_robot_near_pad_end_at_positive_image_x():
    image = np.full((480, 640, 3), 240, dtype=np.uint8)
    cv2.rectangle(image, (250, 205), (390, 275), (15, 15, 15), -1)
    cv2.rectangle(image, (320, 226), (535, 256), (115, 115, 115), -1)
    found = detect_pad_end(image, (1.0, 0.0), 0.14)
    assert found is not None
    assert abs(found.center_uv[1] - 241) < 3
    assert found.grasp_uv[0] > found.center_uv[0]
    assert found.mask[int(found.center_uv[1]), int(found.center_uv[0])] > 0


def test_raw_depth_is_registered_into_color_frame():
    intr = Intrinsics(64, 48, 50.0, 50.0, 31.5, 23.5)
    depth = np.full((48, 64), 0.4, dtype=np.float32)
    mask = np.full((48, 64), 255, dtype=np.uint8)
    point, stats = register_depth_point(
        depth, intr, intr, np.eye(3), np.zeros(3), (31.5, 23.5), mask, radius_px=3
    )
    assert np.allclose(point, [0.0, 0.0, 0.4], atol=0.006)
    assert stats["support"] >= 6


def test_pose_transform_composition():
    transform = pose_matrix([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(transform_point(transform, np.array([0.1, 0.2, 0.3])), [1.1, 2.2, 3.3])
