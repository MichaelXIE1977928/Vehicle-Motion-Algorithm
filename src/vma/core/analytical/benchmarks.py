"""
Analytical / reference signals for every working case (not DLC-only).

Provides:
- Open-loop command time series (exact from maneuver definition).
- Same-input kinematic bicycle trajectory (benchmark vs dynamic tire models).
- Simple steady-state scalars where closed forms exist (e.g. constant-radius turn).
"""
from __future__ import annotations

from typing import Any, Dict

import numpy as np

from vma.core.factories import make_solver, make_tire_pair, make_vehicle
from vma.core.maneuver.standard import Maneuver
from vma.core.simulation import run_simulation


def command_history(t: np.ndarray, maneuver: Maneuver) -> Dict[str, np.ndarray]:
    """Sample [steer, Fx, mu_scale] from the maneuver definition at each time (analytical commands)."""
    n = len(t)
    steer = np.zeros(n)
    fx = np.zeros(n)
    mu = np.ones(n)
    for i, ti in enumerate(t):
        u = maneuver.inputs_at_time(float(ti))
        steer[i], fx[i], mu[i] = float(u[0]), float(u[1]), float(u[2])
    return {"steer_rad": steer, "Fx_N": fx, "mu_scale": mu}


def steady_state_hints(maneuver_params: dict, vehicle_params: dict) -> Dict[str, float]:
    """Closed-form targets where they are unambiguous (all maneuver families covered where applicable)."""
    mtype = maneuver_params["type"]
    lf = float(vehicle_params["cg_to_front"])
    lr = float(vehicle_params["cg_to_rear"])
    L = lf + lr
    out: Dict[str, float] = {}

    def _v0() -> float:
        return float(
            maneuver_params.get(
                "initial_forward_speed",
                maneuver_params.get("speed", 0.0),
            )
        )

    if mtype == "constant_radius":
        R = float(maneuver_params["radius"])
        v = _v0()
        if R > 1e-6:
            out["geometry_steer_rad"] = float(np.arctan(L / R))
            out["kinematic_yaw_rate_rad_s"] = v / R

    elif mtype == "iso7975_braking_in_turn":
        R = float(maneuver_params["radius"])
        v = _v0()
        if R > 1e-6:
            out["kinematic_yaw_rate_preflight_rad_s"] = v / R
        out["brake_decel_m_s2"] = float(maneuver_params.get("brake_decel_m_s2", 0.0))

    elif mtype == "ackermann_slow_slalom_linrange":
        R = float(maneuver_params["radius"])
        v = _v0()
        if R > 1e-6:
            out["geometry_steer_mean_rad"] = float(np.arctan(L / R))
            out["kinematic_yaw_rate_preflight_rad_s"] = v / R
        out["sine_amplitude_rad"] = float(maneuver_params.get("sine_amplitude_rad", 0.0))
        out["sine_freq_hz"] = float(maneuver_params.get("sine_freq_hz", 0.0))

    elif mtype == "throttle_on_in_turn":
        R = float(maneuver_params["radius"])
        v = _v0()
        if R > 1e-6:
            out["geometry_steer_rad"] = float(np.arctan(L / R))
            out["kinematic_yaw_rate_preflight_rad_s"] = v / R
        out["longitudinal_accel_m_s2"] = float(maneuver_params.get("longitudinal_accel_m_s2", 0.0))
        out["throttle_start_time_s"] = float(maneuver_params.get("throttle_start_time", 0.0))

    elif mtype == "step_steer":
        out["step_steer_amplitude_rad"] = float(maneuver_params.get("steer_amplitude", 0.0))
        out["step_time_s"] = float(maneuver_params.get("step_time", 0.0))

    elif mtype in ("double_lane_change", "iso3888_2_obstacle", "lane_change_with_brake"):
        out["steer_amplitude_peak_rad"] = float(maneuver_params.get("steer_amplitude", 0.05))
        if mtype == "lane_change_with_brake":
            out["brake_decel_m_s2"] = float(maneuver_params.get("brake_decel_m_s2", 0.0))
            out["brake_start_time_s"] = float(maneuver_params.get("brake_start_time", 0.0))

    elif mtype == "ramp_step_steer":
        out["ramp_time_s"] = float(maneuver_params.get("ramp_time", 0.0))
        out["hold_time_s"] = float(maneuver_params.get("hold_time", 0.0))
        out["ramp_steer_peak_rad"] = float(maneuver_params.get("steer_amplitude", 0.0))

    elif mtype == "straight_line_brake":
        out["brake_decel_m_s2"] = float(maneuver_params.get("brake_decel_m_s2", 0.0))
        out["brake_start_time_s"] = float(maneuver_params.get("brake_start_time", 0.0))

    elif mtype == "sine_sweep":
        out["sine_amplitude_rad"] = float(maneuver_params.get("amplitude", 0.0))

    elif mtype == "iso7401_impulse":
        out["impulse_steer_rad"] = float(maneuver_params.get("steer_amplitude", 0.0))
        out["impulse_width_s"] = float(maneuver_params.get("impulse_width", 0.0))

    elif mtype == "split_mu":
        out["mu_ratio_after_split"] = float(maneuver_params.get("mu_ratio", 1.0))

    elif mtype == "iso17288_uturn":
        out["max_steer_rad"] = float(maneuver_params.get("max_steer", 0.0))

    elif mtype == "fish_hook":
        out["steer_positive_rad"] = float(maneuver_params.get("steer_positive", 0.0))
        out["steer_negative_rad"] = float(maneuver_params.get("steer_negative", 0.0))

    elif mtype == "sine_with_dwell":
        out["sine_amplitude_rad"] = float(maneuver_params.get("amplitude", 0.0))
        out["dwell_s"] = float(maneuver_params.get("dwell_s", 0.0))
        out["frequency_hz"] = float(maneuver_params.get("frequency_hz", 0.0))

    return out


def _interp_states(
    t_target: np.ndarray,
    t_ref: np.ndarray,
    states: np.ndarray,
    names: list,
) -> Dict[str, np.ndarray]:
    out: Dict[str, np.ndarray] = {}
    for j, nm in enumerate(names):
        out[nm] = np.interp(t_target, t_ref, states[:, j])
    return out


def add_analytical_to_results(
    results: Dict[str, Any],
    vehicle_params: dict,
    maneuver_params: dict,
    solver_backend: str,
    scipy_method: str,
    rk4_dt: float,
) -> Dict[str, Any]:
    """
    Mutates a shallow copy of results: adds results['analytical'] for every maneuver type.

    - commands: steer / Fx / mu_scale sampled on the simulation time grid
    - steady: scalar hints from steady_state_hints
    - kinematic_same_input: states of kinematic bicycle with identical open-loop inputs
    """
    out = dict(results)
    t = np.asarray(results["time"], dtype=float)
    maneuver = Maneuver(maneuver_params)

    analytical: Dict[str, Any] = {
        "commands": command_history(t, maneuver),
        "steady": steady_state_hints(maneuver_params, vehicle_params),
        "description": (
            "Commands are exact from the maneuver JSON. "
            "Kinematic reference uses the same steer/Fx profile with the kinematic bicycle model "
            "(no slip dynamics), comparable for all ISO-style cases."
        ),
    }

    veh_k = make_vehicle("kinematic", vehicle_params)
    tire_f, tire_r = make_tire_pair("linear", vehicle_params)
    solver = make_solver(solver_backend, scipy_method=scipy_method, rk4_dt=rk4_dt)
    kin = run_simulation(veh_k, tire_f, tire_r, maneuver, solver=solver)
    analytical["kinematic_same_input"] = {
        "state_names": kin["state_names"],
        "states_on_main_grid": _interp_states(
            t, kin["time"], kin["states"], list(kin["state_names"])
        ),
    }

    out["analytical"] = analytical
    return out
