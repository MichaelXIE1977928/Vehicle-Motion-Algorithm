"""
Path anchors for the VMA project tree.

All paths are derived from this file's location under `src/vma/` — no references to
directories outside the repository copy that contains this package.
"""
from pathlib import Path

_PKG_DIR = Path(__file__).resolve().parent
SRC_ROOT = _PKG_DIR.parent
REPO_ROOT = SRC_ROOT.parent
DATA_DIR = REPO_ROOT / "data"
VEHICLE_DATA_DIR = DATA_DIR / "vehicles"
MANEUVER_DATA_DIR = DATA_DIR / "maneuvers"
TESTS_DIR = REPO_ROOT / "tests"
SCRIPTS_DIR = REPO_ROOT / "scripts"
