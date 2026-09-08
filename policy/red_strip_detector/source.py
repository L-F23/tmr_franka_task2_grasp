from __future__ import annotations

from dataclasses import dataclass
import hashlib
import time
import urllib.request

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray
    source_marker: str
    received_unix_s: float


class HttpJpegSource:
    def __init__(self, url: str, timeout_s: float = 2.0) -> None:
        self.url = url
        self.timeout_s = timeout_s
        self._last_marker: str | None = None
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def read_new(self) -> CameraFrame | None:
        request = urllib.request.Request(self.url, headers={"Cache-Control": "no-cache"})
        with self._opener.open(request, timeout=self.timeout_s) as response:
            payload = response.read(8_000_001)
            if len(payload) > 8_000_000:
                raise RuntimeError("camera JPEG exceeds 8 MB limit")
            marker = response.headers.get("Last-Modified") or hashlib.sha256(payload).hexdigest()
        if marker == self._last_marker:
            return None
        encoded = np.frombuffer(payload, np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("camera response is not a valid JPEG")
        self._last_marker = marker
        return CameraFrame(image=image, source_marker=marker, received_unix_s=time.time())

