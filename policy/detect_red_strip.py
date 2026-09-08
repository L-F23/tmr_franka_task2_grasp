#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import cv2

from red_strip_detector.detector import DetectorConfig, annotate, detect_red_strips
from red_strip_detector.source import HttpJpegSource


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect red strip labels in the TMR head-camera image")
    parser.add_argument("--camera-url", default="http://172.16.0.50:18082/tmr_zed_latest.jpg")
    parser.add_argument("--image", type=Path, help="use an image file instead of the live camera")
    parser.add_argument("--roi-top", type=float, default=0.42)
    parser.add_argument("--roi-bottom", type=float, default=1.0)
    parser.add_argument("--minimum-area-px", type=float, default=180.0)
    parser.add_argument("--minimum-confidence", type=float, default=0.55)
    parser.add_argument("--wait-s", type=float, default=5.0)
    parser.add_argument("--annotated-output", type=Path)
    parser.add_argument("--all", action="store_true", help="return all valid strips instead of only the best")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    marker = None
    if args.image:
        image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit(f"cannot read image: {args.image}")
        marker = str(args.image.resolve())
    else:
        source = HttpJpegSource(args.camera_url)
        deadline = time.monotonic() + args.wait_s
        frame = None
        while frame is None and time.monotonic() < deadline:
            frame = source.read_new()
            if frame is None:
                time.sleep(0.05)
        if frame is None:
            print(json.dumps({"status": "stale_camera", "camera_url": args.camera_url}), file=sys.stderr)
            return 3
        image, marker = frame.image, frame.source_marker

    config = DetectorConfig(
        roi_top=args.roi_top,
        roi_bottom=args.roi_bottom,
        minimum_area_px=args.minimum_area_px,
    )
    detections = [item for item in detect_red_strips(image, config) if item.confidence >= args.minimum_confidence]
    selected = detections if args.all else detections[:1]
    result = {
        "status": "detected" if selected else "not_found",
        "source": marker,
        "image_size": {"width": image.shape[1], "height": image.shape[0]},
        "count": len(selected),
        "detections": [item.to_dict() for item in selected],
    }
    if args.annotated_output:
        args.annotated_output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.annotated_output), annotate(image, selected)):
            raise RuntimeError(f"failed to write {args.annotated_output}")
        result["annotated_output"] = str(args.annotated_output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if selected else 2


if __name__ == "__main__":
    raise SystemExit(main())

