"""Streamlit home: Dash-Board — simulation configuration matrix (calls internal API)."""
import json
import re
import sys
from datetime import datetime
from pathlib import Path

_SRC_ROOT = Path(__file__).resolve().parents[2]
if str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

import streamlit as st

from vma.gui.theme import apply_vma_button_theme
from vma.iso_catalog import WORKING_CASES
from vma.paths import MANEUVER_DATA_DIR, VEHICLE_DATA_DIR

st.set_page_config(page_title="VMA — Dash-Board", layout="wide")
apply_vma_button_theme()

_DEFAULT_ISO3888_1_LABEL = next(c["gui_label"] for c in WORKING_CASES if c["id"] == "iso3888_1")
_GUI100_DEFAULTS = {
    "vma_gui100_vehicle_model": "bicycle",
    "vma_gui100_tire_model": "linear",
    "vma_gui100_solver_backend": "custom_rk4",
    "vma_gui100_scipy_method": "DOP853",
    "vma_gui100_rk4_dt": 0.01,
    "vma_gui100_working_case_label": _DEFAULT_ISO3888_1_LABEL,
    "vma_gui100_selected_samples": [],
}
_GUI100_PERSIST_KEY = "vma_gui100_persist_snapshot"
for _k, _v in _GUI100_DEFAULTS.items():
    if _k not in st.session_state:
        st.session_state[_k] = _v
if _GUI100_PERSIST_KEY not in st.session_state:
    st.session_state[_GUI100_PERSIST_KEY] = {}


def _restore_gui100_widget_state() -> None:
    persisted = st.session_state.get(_GUI100_PERSIST_KEY, {}) or {}
    # Defaults for first visit only.
    for k, v in _GUI100_DEFAULTS.items():
        if k not in st.session_state:
            st.session_state[k] = v
    # Always restore persisted Dash-Board widget values before widgets are instantiated.
    for k, v in persisted.items():
        if k.startswith("cv_") or k.startswith("vma_gui100_"):
            st.session_state[k] = v


def _snapshot_gui100_widget_state() -> None:
    keep: dict = {}
    for k, v in st.session_state.items():
        if k.startswith("vma_gui100_") or k.startswith("cv_"):
            keep[k] = v
    st.session_state[_GUI100_PERSIST_KEY] = keep


_restore_gui100_widget_state()

st.title("VMA — Dash-Board")
st.caption(
    "Select up to three **sample** vehicles and/or **custom** definitions. "
    "Total simulation cases must be between **1** and **6** (samples + customs). "
    "**Analytical-Information** opens for every selected case after you run analysis."
)

vehicle_dir = VEHICLE_DATA_DIR
maneuver_dir = MANEUVER_DATA_DIR

if not vehicle_dir.exists():
    st.error(f"Vehicle directory not found: {vehicle_dir}")
    st.stop()

vehicle_files = sorted(vehicle_dir.glob("*.json"))
if not vehicle_files:
    st.error(f"No vehicle JSON files in {vehicle_dir}")
    st.stop()

SAMPLE_PREFIX = "vehicle_case_"
sample_stems = sorted(
    f.stem for f in vehicle_files if f.name.startswith(SAMPLE_PREFIX) and f.suffix.lower() == ".json"
)
saved_samples = list(st.session_state.get("vma_gui100_selected_samples", []))
valid_samples = [s for s in saved_samples if s in sample_stems]
if valid_samples != saved_samples:
    st.session_state["vma_gui100_selected_samples"] = valid_samples

st.subheader("ISO working cases — brief introductions")
for c in WORKING_CASES:
    with st.expander(c["gui_label"], expanded=False):
        st.write(c["intro"])

RECOMMENDED_PRESETS = {
    "kinematic": {"solver": "scipy_ode", "scipy_method": "RK45"},
    "bicycle": {"solver": "scipy_ode", "scipy_method": "DOP853"},
    "dof14": {"solver": "scipy_ode", "scipy_method": "BDF"},
    "double_track": {"solver": "scipy_ode", "scipy_method": "DOP853"},
}
VEHICLE_MODEL_LABELS = {
    "bicycle": "bicycle (2 equivalent wheels)",
    "kinematic": "kinematic (2 equivalent wheels)",
    "dof14": "dof14 — true 4 wheels (planar yaw/roll + vertical corner scaffold, 16 states)",
    "double_track": "double_track — 4 wheels, planar yaw/roll (2 tire types per axle)",
}

st.subheader("Simulation matrix")
col_vm, col_t, col_s = st.columns(3)

with col_vm:
    vehicle_model = st.selectbox(
        "Vehicle model",
        ["bicycle", "kinematic", "dof14", "double_track"],
        format_func=lambda vm: VEHICLE_MODEL_LABELS.get(vm, vm),
        key="vma_gui100_vehicle_model",
    )

with col_t:
    tire_model = st.selectbox(
        "Tire model",
        ["linear", "fiala", "dugoff", "magic_formula", "brush_combined"],
        key="vma_gui100_tire_model",
    )

with col_s:
    solver_backend = st.selectbox(
        "Solver",
        ["scipy_ode", "custom_rk4"],
        key="vma_gui100_solver_backend",
    )

rec = RECOMMENDED_PRESETS.get(vehicle_model, {"solver": "scipy_ode", "scipy_method": "RK45"})
col_m, col_dt, col_case = st.columns(3)
with col_m:
    scipy_method = "RK45"
    if solver_backend == "scipy_ode":
        recommended_method = rec["scipy_method"]
        if "vma_gui100_scipy_method" not in st.session_state:
            st.session_state["vma_gui100_scipy_method"] = recommended_method
        if st.session_state.get("vma_gui100_last_vehicle_model") != vehicle_model:
            st.session_state["vma_gui100_scipy_method"] = recommended_method
        st.session_state["vma_gui100_last_vehicle_model"] = vehicle_model
        scipy_method = st.selectbox(
            "SciPy solve_ivp method",
            ["RK45", "RK23", "DOP853", "Radau", "BDF", "LSODA"],
            key="vma_gui100_scipy_method",
        )
with col_dt:
    rk4_dt = 0.01
    if solver_backend == "custom_rk4":
        rk4_dt = st.number_input(
            "RK4 fixed step dt (s)",
            min_value=1e-4,
            format="%.4f",
            key="vma_gui100_rk4_dt",
        )
    else:
        rk4_dt = float(st.session_state.get("vma_gui100_rk4_dt", 0.01))

with col_case:
    case_labels = [c["gui_label"] for c in WORKING_CASES]
    label_to_id = {c["gui_label"]: c["id"] for c in WORKING_CASES}
    if st.session_state.get("vma_gui100_working_case_label") not in case_labels:
        st.session_state["vma_gui100_working_case_label"] = (
            _DEFAULT_ISO3888_1_LABEL if _DEFAULT_ISO3888_1_LABEL in case_labels else case_labels[0]
        )
    chosen_label = st.selectbox(
        "Working case (ISO-oriented)",
        case_labels,
        key="vma_gui100_working_case_label",
    )
    working_case_id = label_to_id[chosen_label]

st.markdown("---")
with st.expander("Vehicle model files: purpose", expanded=False):
    st.markdown(
        """**base.py**

Defines the interface contract for any vehicle model.  
Requires two things: `state_derivative(...)` and `state_names`.  
Purpose: all simulators/GUI/API can treat models uniformly.

**bicycle.py**

Implements a dynamic bicycle model with roll (`phi`, `phi_dot`).  
Uses tire slip angles and lateral forces to compute `vx_dot`, `vy_dot`, `omega_dot`, etc.  
Uses **2 equivalent wheels** (single-track front/rear representation).  
Purpose: mid-fidelity handling dynamics with relatively low complexity.

**kinematic.py**

Implements a kinematic bicycle model (no tire slip dynamics).  
Motion is geometric (path-following style), simpler and faster.  
Uses **2 equivalent wheels** (single-track front/rear representation).  
Purpose: low-speed/path planning style analysis and baseline comparisons.

**double_track_planar.py**

Double-track rigid body in the horizontal plane with the same roll pair (`phi`, `phi_dot`) as the bicycle plant.  
**Four contact patches** (FL, FR, RL, RR): slip and lateral force are evaluated per corner using rigid-body patch velocities and **half the axle static normal load** on each side (lateral load transfer across the track is a natural follow-up).  
Front axle uses the same steer command for both wheels; rear steer / toe / camber follow the same JSON K&C fields as the bicycle path.  
Purpose: stronger yaw–moment realism from left/right slip differences, toe/camber asymmetry groundwork, and split-μ studies that need a left/right force path (still one friction scale from the maneuver unless extended later).

**dof14.py**

Defines a **16-state** vehicle: **8 planar+roll** states integrated with the same **four-wheel (double-track)** tire slip/force path as `double_track_planar.py`, plus **4 unsprung vertical displacements** and **4 vertical velocities** per corner.  
Important: vertical modes are still a **linear uncoupled scaffold** (they do not yet modify per-corner tire normal loads in the planar force calculation).  
Purpose: combine true **4-wheel planar** yaw realism with early vertical-dynamics structure toward a full CarSim-style plant.
"""
    )

with st.expander("Tire model files: purpose", expanded=False):
    st.markdown(
        """**linear.py**

Implements a linear tire approximation where lateral force scales with slip angle.  
Purpose: quick baseline analysis and easy parameter sensitivity checks.

**fiala.py**

Implements a brush-style nonlinear tire model with saturation behavior.  
Purpose: more realistic lateral force buildup and breakaway than linear models.

**dugoff.py**

Implements the Dugoff tire formulation with combined friction utilization behavior.  
Purpose: practical nonlinear tire behavior for transient handling studies.

**magic_formula.py**

Implements a Pacejka-style Magic Formula tire model using `B`, `C`, `E`, and `mu` style parameters.  
Purpose: higher-fidelity empirical tire force shaping for closer real-vehicle matching.

**brush_combined.py**

Uses a Magic-Formula lateral backbone and a Pacejka-style combined-slip reduction factor using longitudinal-utilization proxy (`kappa`).  
Purpose: practical first-step coupled Fx/Fy behavior (e.g., throttle/brake while cornering) without introducing full wheel-rotational states.
"""
    )

with st.expander("Solver files: purpose", expanded=False):
    st.markdown(
        """**scipy_ode.py**

Wraps `scipy.integrate.solve_ivp` with selectable methods (`RK45`, `DOP853`, `BDF`, etc.).  
Purpose: robust general integration for development, validation, and stiff/non-stiff comparisons.

**custom_rk4.py**

Implements a fixed-step classical Runge-Kutta 4 integrator.  
Purpose: deterministic step timing and real-time/HiL-oriented simulation workflows.
"""
    )

with st.expander("SciPy solve_ivp method: explanation", expanded=False):
    st.markdown(
        """**RK45**

Explicit Runge-Kutta method of order 5(4).  
Good general-purpose non-stiff solver; usually the default first try.

**RK23**

Explicit Runge-Kutta method of order 3(2).  
Lower-order alternative for non-stiff problems; can be useful when tolerances are loose.

**DOP853**

High-order explicit Runge-Kutta method (order 8).  
Often efficient for smooth non-stiff dynamics when higher accuracy is desired.

**Radau**

Implicit Runge-Kutta method (Radau IIA, order 5).  
Designed for stiff systems; more robust when explicit methods struggle.

**BDF**

Implicit multi-step backward differentiation formula.  
Common choice for stiff dynamics, especially when transients include fast/slow scales.

**LSODA**

ODEPACK-based method that auto-switches between non-stiff Adams and stiff BDF modes.  
Convenient mixed-behavior solver when stiffness may appear only in parts of the run.
"""
    )
st.subheader("Vehicle cases (1–6 total)")
st.caption(
    "Choose **sample** presets (max 3) and/or **custom** definitions. "
    "Custom slots available = **6 − (number of samples selected)**. "
    "Example: all 3 samples → 3 custom slots; no samples → 6 custom slots."
)

if sample_stems:
    max_sel = min(3, len(sample_stems))
    selected_samples = st.multiselect(
        "Sample vehicles (data/vehicles, max 3)",
        options=sample_stems,
        max_selections=max_sel,
        key="vma_gui100_selected_samples",
        help="ISO-style presets. Combined with enabled custom rows below: 1 ≤ samples + customs ≤ 6.",
    )
else:
    st.info("No `vehicle_case_*.json` files found in the vehicle folder; use custom definitions only (up to 6).")
    selected_samples = []

n_sample = len(selected_samples)
max_custom_slots = max(0, 6 - n_sample)

st.metric("Custom definition slots available", max_custom_slots)

tire_is_mf = tire_model == "magic_formula"
tire_is_brush_combined = tire_model == "brush_combined"
tire_needs_mf_inputs = tire_is_mf or tire_is_brush_combined

for i in range(max_custom_slots):
    slot_defined = bool(st.session_state.get(f"cv_{i}_include", False))
    with st.expander(f"Custom vehicle definition #{i + 1} (optional)", expanded=slot_defined):
        st.checkbox(
            "Include this vehicle in the batch run",
            key=f"cv_{i}_include",
            help="When checked, this set of parameters is saved as JSON and simulated together with any selected samples.",
        )
        if st.session_state.get(f"cv_{i}_include", False):
            st.caption("Parameters are written to `data/vehicles` when you press **Run analysis**.")
            col_name, col_mass, col_iz = st.columns(3)
            with col_name:
                st.text_input(
                    "Vehicle name suffix",
                    value="concept",
                    key=f"cv_{i}_name",
                    help="Used in the saved file name.",
                )
            with col_mass:
                st.number_input("Mass (kg)", min_value=300.0, value=1500.0, step=10.0, key=f"cv_{i}_mass")
            with col_iz:
                st.number_input(
                    "Yaw inertia Iz (kg·m^2)", min_value=500.0, value=2500.0, step=50.0, key=f"cv_{i}_yaw_inertia"
                )

            col_a, col_b, col_cf, col_cr = st.columns(4)
            with col_a:
                st.number_input("CG to front axle a (m)", min_value=0.2, value=1.2, step=0.01, key=f"cv_{i}_cg_to_front")
            with col_b:
                st.number_input("CG to rear axle b (m)", min_value=0.2, value=1.6, step=0.01, key=f"cv_{i}_cg_to_rear")
            with col_cf:
                st.number_input(
                    "Front cornering stiffness Cf (N/rad)",
                    min_value=1000.0,
                    value=80000.0,
                    step=500.0,
                    key=f"cv_{i}_cf",
                )
            with col_cr:
                st.number_input(
                    "Rear cornering stiffness Cr (N/rad)",
                    min_value=1000.0,
                    value=90000.0,
                    step=500.0,
                    key=f"cv_{i}_cr",
                )

            col_mu, col_vf, col_vz = st.columns(3)
            with col_mu:
                st.number_input(
                    "Base friction coeff mu (-)", min_value=0.1, max_value=2.0, value=0.9, step=0.01, key=f"cv_{i}_mu"
                )
            with col_vf:
                st.number_input(
                    "Vertical natural frequency (Hz)", min_value=0.5, value=2.0, step=0.05, key=f"cv_{i}_vert_freq"
                )
            with col_vz:
                st.number_input(
                    "Vertical damping ratio (-)", min_value=0.01, max_value=2.0, value=0.25, step=0.01, key=f"cv_{i}_vert_zeta"
                )

            st.markdown("**Roll dynamics parameters**")
            col_sm, col_ir, col_hcg, col_tw = st.columns(4)
            with col_sm:
                st.number_input("Sprung mass (kg)", min_value=100.0, value=1320.0, step=10.0, key=f"cv_{i}_sprung_mass")
            with col_ir:
                st.number_input(
                    "Roll inertia Ixx (kg·m^2)", min_value=50.0, value=640.0, step=10.0, key=f"cv_{i}_roll_inertia"
                )
            with col_hcg:
                st.number_input("CG height h (m)", min_value=0.1, value=0.54, step=0.01, key=f"cv_{i}_cg_height")
            with col_tw:
                st.number_input("Track width (m)", min_value=0.8, value=1.58, step=0.01, key=f"cv_{i}_track_width")

            col_kf, col_kr, col_cf_roll, col_cr_roll = st.columns(4)
            with col_kf:
                st.number_input(
                    "Front roll stiffness (N·m/rad)",
                    min_value=1000.0,
                    value=29000.0,
                    step=500.0,
                    key=f"cv_{i}_front_roll_k",
                )
            with col_kr:
                st.number_input(
                    "Rear roll stiffness (N·m/rad)",
                    min_value=1000.0,
                    value=25000.0,
                    step=500.0,
                    key=f"cv_{i}_rear_roll_k",
                )
            with col_cf_roll:
                st.number_input(
                    "Front roll damping (N·m·s/rad)",
                    min_value=50.0,
                    value=2200.0,
                    step=50.0,
                    key=f"cv_{i}_front_roll_c",
                )
            with col_cr_roll:
                st.number_input(
                    "Rear roll damping (N·m·s/rad)",
                    min_value=50.0,
                    value=2000.0,
                    step=50.0,
                    key=f"cv_{i}_rear_roll_c",
                )

            st.number_input(
                "Load transfer coupling (-)",
                min_value=0.0,
                max_value=1.0,
                value=0.08,
                step=0.01,
                key=f"cv_{i}_load_transfer_coupling",
            )

            st.markdown("**Steering geometry and K&C**")
            col_toe_f, col_toe_r, col_cam_f, col_cam_r = st.columns(4)
            with col_toe_f:
                st.number_input(
                    "Front toe (deg)",
                    min_value=-10.0,
                    max_value=10.0,
                    value=0.0,
                    step=0.05,
                    key=f"cv_{i}_front_toe_deg",
                )
            with col_toe_r:
                st.number_input(
                    "Rear toe (deg)",
                    min_value=-10.0,
                    max_value=10.0,
                    value=0.0,
                    step=0.05,
                    key=f"cv_{i}_rear_toe_deg",
                )
            with col_cam_f:
                st.number_input(
                    "Front camber (deg)",
                    min_value=-15.0,
                    max_value=15.0,
                    value=0.0,
                    step=0.05,
                    key=f"cv_{i}_front_camber_deg",
                )
            with col_cam_r:
                st.number_input(
                    "Rear camber (deg)",
                    min_value=-15.0,
                    max_value=15.0,
                    value=0.0,
                    step=0.05,
                    key=f"cv_{i}_rear_camber_deg",
                )

            col_sgain, col_rgain, col_cgain, col_camber_gain = st.columns(4)
            with col_sgain:
                st.number_input(
                    "Front steering gain (-)",
                    min_value=0.2,
                    max_value=3.0,
                    value=1.0,
                    step=0.05,
                    key=f"cv_{i}_front_steer_gain",
                )
            with col_rgain:
                st.number_input(
                    "Rear steer gain (rad/rad)",
                    min_value=-1.0,
                    max_value=1.0,
                    value=0.0,
                    step=0.01,
                    key=f"cv_{i}_rear_steer_gain",
                )
            with col_cgain:
                st.number_input(
                    "Compliance steer gain (rad/(m/s^2))",
                    min_value=-0.05,
                    max_value=0.05,
                    value=0.0,
                    step=0.001,
                    format="%.3f",
                    key=f"cv_{i}_compliance_steer_gain",
                )
            with col_camber_gain:
                st.number_input(
                    "Camber thrust gain (rad/rad)",
                    min_value=-2.0,
                    max_value=2.0,
                    value=0.0,
                    step=0.05,
                    key=f"cv_{i}_camber_thrust_gain",
                )

            if tire_needs_mf_inputs:
                st.markdown("**Magic Formula (tire model) parameters**")
                col_bf, col_cf_mf, col_ef, col_muf = st.columns(4)
                with col_bf:
                    st.number_input("MF B", value=10.0, step=0.1, key=f"cv_{i}_mf_b")
                with col_cf_mf:
                    st.number_input("MF C", value=1.9, step=0.05, key=f"cv_{i}_mf_c")
                with col_ef:
                    st.number_input("MF E", value=-0.5, step=0.05, key=f"cv_{i}_mf_e")
                with col_muf:
                    st.number_input(
                        "MF mu", min_value=0.1, max_value=2.0, value=0.85, step=0.01, key=f"cv_{i}_mf_mu"
                    )
            if tire_is_brush_combined:
                st.markdown("**Combined-slip scaling parameters (brush_combined)**")
                col_bx, col_cx, col_gmin = st.columns(3)
                with col_bx:
                    st.number_input("Combined Bx", value=8.0, step=0.1, key=f"cv_{i}_combined_bx")
                with col_cx:
                    st.number_input("Combined Cx", value=1.1, step=0.05, key=f"cv_{i}_combined_cx")
                with col_gmin:
                    st.number_input(
                        "Combined min scale",
                        min_value=0.0,
                        max_value=1.0,
                        value=0.2,
                        step=0.01,
                        key=f"cv_{i}_combined_min_scale",
                    )


def _safe_suffix(name: str) -> str:
    s = re.sub(r"[^a-zA-Z0-9_-]+", "_", name.strip()).strip("_")
    return s or "concept"


def _build_custom_payload(i: int, *, include_magic_formula: bool, include_brush_combined: bool) -> dict:
    payload = {
        "mass": float(st.session_state[f"cv_{i}_mass"]),
        "yaw_inertia": float(st.session_state[f"cv_{i}_yaw_inertia"]),
        "cg_to_front": float(st.session_state[f"cv_{i}_cg_to_front"]),
        "cg_to_rear": float(st.session_state[f"cv_{i}_cg_to_rear"]),
        "front_cornering_stiffness": float(st.session_state[f"cv_{i}_cf"]),
        "rear_cornering_stiffness": float(st.session_state[f"cv_{i}_cr"]),
        "sprung_mass": float(st.session_state[f"cv_{i}_sprung_mass"]),
        "roll_inertia": float(st.session_state[f"cv_{i}_roll_inertia"]),
        "cg_height": float(st.session_state[f"cv_{i}_cg_height"]),
        "front_roll_stiffness": float(st.session_state[f"cv_{i}_front_roll_k"]),
        "rear_roll_stiffness": float(st.session_state[f"cv_{i}_rear_roll_k"]),
        "front_roll_damping": float(st.session_state[f"cv_{i}_front_roll_c"]),
        "rear_roll_damping": float(st.session_state[f"cv_{i}_rear_roll_c"]),
        "track_width": float(st.session_state[f"cv_{i}_track_width"]),
        "load_transfer_coupling": float(st.session_state[f"cv_{i}_load_transfer_coupling"]),
        "front_toe_deg": float(st.session_state[f"cv_{i}_front_toe_deg"]),
        "rear_toe_deg": float(st.session_state[f"cv_{i}_rear_toe_deg"]),
        "front_camber_deg": float(st.session_state[f"cv_{i}_front_camber_deg"]),
        "rear_camber_deg": float(st.session_state[f"cv_{i}_rear_camber_deg"]),
        "front_steer_gain": float(st.session_state[f"cv_{i}_front_steer_gain"]),
        "rear_steer_gain": float(st.session_state[f"cv_{i}_rear_steer_gain"]),
        "compliance_steer_gain": float(st.session_state[f"cv_{i}_compliance_steer_gain"]),
        "camber_thrust_gain": float(st.session_state[f"cv_{i}_camber_thrust_gain"]),
        "friction_coeff": float(st.session_state[f"cv_{i}_mu"]),
        "vert_natural_freq_hz": float(st.session_state[f"cv_{i}_vert_freq"]),
        "vert_damping_ratio": float(st.session_state[f"cv_{i}_vert_zeta"]),
    }
    if include_magic_formula:
        payload["magic_formula"] = {
            "B": float(st.session_state[f"cv_{i}_mf_b"]),
            "C": float(st.session_state[f"cv_{i}_mf_c"]),
            "E": float(st.session_state[f"cv_{i}_mf_e"]),
            "mu": float(st.session_state[f"cv_{i}_mf_mu"]),
        }
    if include_brush_combined:
        payload["brush_combined"] = {
            "B": float(st.session_state[f"cv_{i}_mf_b"]),
            "C": float(st.session_state[f"cv_{i}_mf_c"]),
            "E": float(st.session_state[f"cv_{i}_mf_e"]),
            "mu": float(st.session_state[f"cv_{i}_mf_mu"]),
            "combined_Bx": float(st.session_state[f"cv_{i}_combined_bx"]),
            "combined_Cx": float(st.session_state[f"cv_{i}_combined_cx"]),
            "combined_min_scale": float(st.session_state[f"cv_{i}_combined_min_scale"]),
        }
    return payload


st.markdown("---")
if st.button("Run analysis (open Analytical-Information)"):
    n_custom = sum(1 for i in range(max_custom_slots) if st.session_state.get(f"cv_{i}_include", False))
    total_runs = n_sample + n_custom

    if total_runs < 1:
        st.error("Select at least one sample vehicle or enable at least one custom vehicle definition.")
        st.stop()
    if total_runs > 6:
        st.error("Total cases cannot exceed 6.")
        st.stop()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    paths: list[str] = []
    labels: list[str] = []

    for stem in selected_samples:
        p = vehicle_dir / f"{stem}.json"
        paths.append(str(p.resolve()))
        labels.append(f"Sample · {stem}")

    for i in range(max_custom_slots):
        if not st.session_state.get(f"cv_{i}_include", False):
            continue
        safe = _safe_suffix(str(st.session_state.get(f"cv_{i}_name", "concept")))
        out_name = f"custom_vehicle_{safe}_slot{i + 1}_{ts}.json"
        custom_vehicle_path = vehicle_dir / out_name
        payload = _build_custom_payload(
            i,
            include_magic_formula=tire_is_mf,
            include_brush_combined=tire_is_brush_combined,
        )
        custom_vehicle_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        st.success(f"Saved custom vehicle JSON: {custom_vehicle_path.name}")
        paths.append(str(custom_vehicle_path.resolve()))
        labels.append(f"Custom · {safe} (#{i + 1})")

    st.session_state["vma_vehicle_paths"] = paths
    st.session_state["vma_vehicle_labels"] = labels
    st.session_state["vma_vehicle_model"] = vehicle_model
    st.session_state["vma_tire_model"] = tire_model
    st.session_state["vma_solver_backend"] = solver_backend
    st.session_state["vma_scipy_method"] = scipy_method
    st.session_state["vma_rk4_dt"] = float(rk4_dt)
    st.session_state["vma_working_case_id"] = working_case_id
    st.session_state["vma_working_case_label"] = chosen_label
    st.session_state["vma_maneuver_dir"] = str(maneuver_dir)
    _snapshot_gui100_widget_state()
    # Invalidate cached batch results when launching a new run
    for k in ("vma_batch_results", "vma_batch_sig"):
        st.session_state.pop(k, None)
    st.switch_page("pages/Analytical_Information.py")

st.markdown(
    "Use **Run analysis** to open **Analytical-Information**. Use the sidebar or the back button on that page to return to **Dash-Board**."
)
