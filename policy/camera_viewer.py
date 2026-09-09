#!/usr/bin/env python3
"""Bridge the three deployed ROS RGB topics to local MJPEG endpoints."""

from __future__ import annotations

import argparse
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


DEFAULT_TOPICS = {
    "main": "/head_camera/zed/rgb/color/rect/image/compressed",
    "left": "/wrist_camera_left/color/image_raw",
    "right": "/wrist_camera_right/color/image_raw",
}


class FrameStore:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frames: dict[str, bytes] = {}
        self._sequence = {name: 0 for name in DEFAULT_TOPICS}
        self._updated = {name: 0.0 for name in DEFAULT_TOPICS}

    def update(self, name: str, image: np.ndarray) -> None:
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return
        with self._condition:
            self._frames[name] = encoded.tobytes()
            self._sequence[name] += 1
            self._updated[name] = time.monotonic()
            self._condition.notify_all()

    def status(self) -> dict:
        now = time.monotonic()
        with self._condition:
            age = {
                name: None if not stamp else round(now - stamp, 3)
                for name, stamp in self._updated.items()
            }
            return {
                "healthy": {
                    name: value is not None and value <= 2.0
                    for name, value in age.items()
                },
                "sequence": dict(self._sequence),
                "frame_age_s": age,
            }

    def wait_for_frame(self, name: str, last_sequence: int) -> tuple[int, bytes | None]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence[name] > last_sequence,
                timeout=2.0,
            )
            return self._sequence[name], self._frames.get(name)


class CameraBridge(Node):
    def __init__(self, store: FrameStore, topics: dict[str, str]) -> None:
        super().__init__("tmr_task2_camera_viewer")
        self._store = store
        self._bridge = CvBridge()
        self.create_subscription(
            CompressedImage, topics["main"], self._main_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, topics["left"], lambda msg: self._raw_callback("left", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, topics["right"], lambda msg: self._raw_callback("right", msg),
            qos_profile_sensor_data,
        )

    def _main_callback(self, message: CompressedImage) -> None:
        image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is not None:
            self._store.update("main", image)

    def _raw_callback(self, name: str, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:  # ROS encoding errors must not kill the bridge.
            self.get_logger().error(f"failed to convert {name} frame: {exc}")
            return
        self._store.update(name, image)


def handler_class(store: FrameStore):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
            path = self.path.split("?", 1)[0]
            if path == "/status.json":
                payload = json.dumps(store.status()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            if path in ("/main.mjpg", "/left.mjpg", "/right.mjpg"):
                self._stream(path[1:-5])
                return
            if path == "/":
                payload = (
                    b"<html><body><img src='/main.mjpg'>"
                    b"<img src='/left.mjpg'><img src='/right.mjpg'></body></html>"
                )
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            self.send_error(404)

        def _stream(self, name: str) -> None:
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            sequence = -1
            try:
                while True:
                    sequence, frame = store.wait_for_frame(name, sequence)
                    if frame is None:
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode("ascii")
                        + b"\r\n\r\n"
                        + frame
                        + b"\r\n"
                    )
            except (BrokenPipeError, ConnectionResetError):
                return

        def log_message(self, _format: str, *_args) -> None:
            return

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18081)
    args = parser.parse_args()
    topics = {
        name: os.environ.get(f"TMR_{name.upper()}_CAMERA_TOPIC", topic)
        for name, topic in DEFAULT_TOPICS.items()
    }
    store = FrameStore()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_class(store))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    rclpy.init()
    node = CameraBridge(store, topics)
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
