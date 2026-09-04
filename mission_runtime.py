"""Shared Task 2 coordinator contracts adapted from the Task 3 runner."""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import tempfile
import time


LOCK_FILE = Path("/tmp/tmr_task2_motion.lock")


class MissionAlreadyRunning(RuntimeError):
    pass


def acquire_motion_lock(path: Path = LOCK_FILE):
    """Acquire the one lock shared by every Task 2 motion entrypoint."""
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+b")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        handle.close()
        raise MissionAlreadyRunning(
            f"another Task 2 motion coordinator is active ({path})"
        ) from exc
    handle.seek(0)
    handle.truncate()
    handle.write(json.dumps({
        "pid": os.getpid(),
        "started_unix_s": time.time(),
    }).encode("utf-8"))
    handle.flush()
    os.fsync(handle.fileno())
    return handle


def release_motion_lock(handle) -> None:
    if handle is None:
        return
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
    finally:
        handle.close()


def atomic_write_json(path: Path, payload: dict) -> None:
    """Commit a complete checkpoint even if the coordinator is interrupted."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    os.replace(temporary, path)
