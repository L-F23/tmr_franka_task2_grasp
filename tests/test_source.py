from email.message import Message
import io

import cv2
import numpy as np

from red_strip_detector.source import HttpJpegSource


class Response(io.BytesIO):
    def __init__(self, payload: bytes, marker: str):
        super().__init__(payload)
        self.headers = Message()
        self.headers["Last-Modified"] = marker
    def __enter__(self): return self
    def __exit__(self, *_args): self.close()


class Opener:
    def __init__(self, response): self.response = response
    def open(self, *_args, **_kwargs):
        return Response(self.response.getvalue(), self.response.headers["Last-Modified"])


def test_repeated_export_file_is_not_reported_as_new() -> None:
    ok, encoded = cv2.imencode(".jpg", np.zeros((20, 30, 3), np.uint8))
    assert ok
    source = HttpJpegSource("http://camera/frame.jpg")
    source._opener = Opener(Response(encoded.tobytes(), "Thu, 03 Sep 2026 00:00:00 GMT"))
    assert source.read_new() is not None
    assert source.read_new() is None

