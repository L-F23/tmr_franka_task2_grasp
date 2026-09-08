#!/usr/bin/env python3
"""Canonical entry point: initialize the robot, then run live detection."""

import subprocess
import sys

from robot_initializer import InitializationError, initialize


def main() -> int:
    try:
        initialize()
    except InitializationError as exc:
        print(f"startup initialization failed: {exc}", file=sys.stderr)
        return 10
    return subprocess.call([sys.executable, "detect_red_strip.py", *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
