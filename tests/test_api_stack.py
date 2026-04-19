"""Smoke tests for factories, tires, and API file-based runs."""
from pathlib import Path

import numpy as np

from vma.api import run_simulation_from_files
from vma.core.factories import make_solver, make_tire_pair, make_vehicle
from vma.core.tire.fiala import FialaTire
from vma.core.tire.magic_formula import MagicFormulaTire
from vma.core.tire.brush_combined import BrushCombinedTire
from vma.paths import REPO_ROOT


def _root() -> Path:
    return REPO_ROOT


def test_make_vehicle_and_tires():
    p = {
        "mass": 1500,
        "yaw_inertia": 2500,
        "cg_to_front": 1.2,
        "cg_to_rear": 1.6,
        "front_cornering_stiffness": 80000,
        "rear_cornering_stiffness": 90000,
        "friction_coeff": 0.9,
    }
    v = make_vehicle("bicycle", p)
    assert v.state_names[0] == "x"
    k = make_vehicle("kinematic", p)
    assert len(k.state_names) == 6
    d = make_vehicle("dof14", p)
    assert len(d.state_names) == 16
    dt = make_vehicle("double_track", p)
    assert len(dt.state_names) == 8
    tf, tr = make_tire_pair("fiala", p)
    assert isinstance(tf, FialaTire)
    tf2, _ = make_tire_pair("magic_formula", {**p, "magic_formula": {"B": 8.0, "C": 1.8, "E": -0.4, "mu": 0.8}})
    assert isinstance(tf2, MagicFormulaTire)
    tf3, _ = make_tire_pair("brush_combined", {**p, "brush_combined": {"B": 8.0, "C": 1.8, "E": -0.4, "mu": 0.8}})
    assert isinstance(tf3, BrushCombinedTire)


def test_run_iso7401_rk4():
    root = _root()
    vp = root / "data" / "vehicles" / "vehicle_case_sedan.json"
    mp = root / "data" / "maneuvers" / "iso7401_step_steer.json"
    r = run_simulation_from_files(
        vp,
        mp,
        vehicle_model="bicycle",
        tire_model="linear",
        solver_backend="custom_rk4",
        rk4_dt=0.005,
    )
    assert r["states"].shape[1] == 8
    assert "roll_gradient_deg_per_g" in r.get("derived_series", {})
    assert len(r["time"]) > 10


def test_scipy_lsoda():
    root = _root()
    vp = root / "data" / "vehicles" / "vehicle_case_sedan.json"
    mp = root / "data" / "maneuvers" / "iso4138_constant_radius.json"
    r = run_simulation_from_files(
        vp,
        mp,
        vehicle_model="bicycle",
        tire_model="dugoff",
        solver_backend="scipy_ode",
        scipy_method="LSODA",
    )
    assert np.all(np.isfinite(r["states"]))


def test_brush_combined_smoke():
    root = _root()
    vp = root / "data" / "vehicles" / "vehicle_case_sedan.json"
    mp = root / "data" / "maneuvers" / "iso7975_braking_in_turn.json"
    r = run_simulation_from_files(
        vp,
        mp,
        vehicle_model="double_track",
        tire_model="brush_combined",
        solver_backend="scipy_ode",
        scipy_method="RK45",
    )
    assert np.all(np.isfinite(r["states"]))
    ds = r.get("derived_series", {})
    assert "Fy_fl_tire_N" in ds


def test_double_track_smoke():
    root = _root()
    vp = root / "data" / "vehicles" / "vehicle_case_sedan.json"
    mp = root / "data" / "maneuvers" / "iso7401_step_steer.json"
    r = run_simulation_from_files(
        vp,
        mp,
        vehicle_model="double_track",
        tire_model="linear",
        solver_backend="scipy_ode",
        scipy_method="RK45",
    )
    assert r["states"].shape[1] == 8
    ds = r.get("derived_series") or {}
    assert "alpha_fl_rad" in ds
    assert "alpha_fr_rad" in ds
    assert np.all(np.isfinite(r["states"]))


def test_dof14_states():
    root = _root()
    vp = root / "data" / "vehicles" / "vehicle_case_sedan.json"
    mp = root / "data" / "maneuvers" / "iso7401_step_steer.json"
    r = run_simulation_from_files(vp, mp, vehicle_model="dof14", tire_model="linear")
    assert r["states"].shape[1] == 16
    ds = r.get("derived_series") or {}
    assert "alpha_fl_rad" in ds
    assert "alpha_fr_rad" in ds


def test_make_solver_rk45_string():
    s = make_solver("scipy_ode", scipy_method="RK23")
    assert s.method == "RK23"
