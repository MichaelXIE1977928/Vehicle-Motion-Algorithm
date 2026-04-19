"""Analytical benchmarks attached for all maneuver types."""
from pathlib import Path

from vma.api import run_simulation_from_files
from vma.paths import REPO_ROOT


def _maneuver(name: str) -> Path:
    return REPO_ROOT / "data" / "maneuvers" / name


def _vehicle() -> Path:
    return REPO_ROOT / "data" / "vehicles" / "vehicle_case_sedan.json"


def test_analytical_present_for_iso7401():
    r = run_simulation_from_files(
        _vehicle(),
        _maneuver("iso7401_step_steer.json"),
        include_analytical=True,
    )
    assert "analytical" in r
    a = r["analytical"]
    assert "commands" in a and "steady" in a and "kinematic_same_input" in a
    assert "steer_rad" in a["commands"]
    kin = a["kinematic_same_input"]["states_on_main_grid"]
    assert "omega" in kin
    assert len(kin["omega"]) == len(r["time"])


def test_analytical_for_constant_radius():
    r = run_simulation_from_files(
        _vehicle(),
        _maneuver("iso4138_constant_radius.json"),
        include_analytical=True,
    )
    assert r["analytical"]["steady"].get("kinematic_yaw_rate_rad_s") is not None


def test_analytical_disabled():
    r = run_simulation_from_files(
        _vehicle(),
        _maneuver("iso3888_1_double_lane_change.json"),
        include_analytical=False,
    )
    assert "analytical" not in r


def test_analytical_ackermann_slalom_and_throttle_in_turn():
    for name in ("ackermann_slow_slalom_linrange.json", "throttle_on_in_turn_understeer.json"):
        r = run_simulation_from_files(_vehicle(), _maneuver(name), include_analytical=True)
        assert "analytical" in r
        st = r["analytical"]["steady"]
        assert st.get("geometry_steer_mean_rad") is not None or st.get("geometry_steer_rad") is not None
