"""Make the runtime source directory importable during repository-level tests."""

from pathlib import Path
import sys


POLICY_ROOT = Path(__file__).resolve().parents[1] / "policy"
sys.path.insert(0, str(POLICY_ROOT))
