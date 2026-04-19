"""Ensure path anchors stay inside the repository tree."""
from pathlib import Path

from vma.paths import DATA_DIR, MANEUVER_DATA_DIR, REPO_ROOT, SRC_ROOT, VEHICLE_DATA_DIR


def test_paths_resolve_inside_repo():
    tests_file = Path(__file__).resolve()
    assert REPO_ROOT == tests_file.parents[1]
    assert SRC_ROOT == REPO_ROOT / "src"
    assert DATA_DIR.is_dir()
    assert VEHICLE_DATA_DIR.is_dir()
    assert MANEUVER_DATA_DIR.is_dir()
