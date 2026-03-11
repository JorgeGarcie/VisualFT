"""Test package bootstrap for pure-Python driver modules."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]

for rel_path in ("ros2_ws/src/common", "ros2_ws/src/flexiv_driver"):
    path = str(REPO_ROOT / rel_path)
    if path not in sys.path:
        sys.path.insert(0, path)
