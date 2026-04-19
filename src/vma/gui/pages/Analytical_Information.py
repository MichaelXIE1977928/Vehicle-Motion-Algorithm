"""Analytical-Information — unified analysis for ISO-oriented working cases (routes Dash-Board selections)."""
import csv
import io
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# `vma` is under `src/` — insert `src` (pages → gui → vma → src)
_SRC_ROOT = Path(__file__).resolve().parents[3]
if str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

from vma.api import run_simulation_from_files
from vma.gui.theme import apply_vma_button_theme
from vma.iso_catalog import maneuver_path_for_id

# Uniform matplotlib figure size for every chart (matches Custom chart builder).
_CHART_FIGSIZE_INCHES: Tuple[float, float] = (9.0, 5.0)

st.set_page_config(page_title="VMA — Analytical-Information", layout="wide")
apply_vma_button_theme()

st.title("VMA — Analytical-Information")
if st.button("← Go back to Dash-Board"):
    st.switch_page("app.py")

required = [
    "vma_vehicle_paths",
    "vma_vehicle_model",
    "vma_tire_model",
    "vma_solver_backend",
    "vma_scipy_method",
    "vma_rk4_dt",
    "vma_working_case_id",
    "vma_maneuver_dir",
]
if not all(k in st.session_state for k in required):
    st.warning("No configuration in session. Open **Dash-Board** and press **Run analysis**.")
    st.stop()

case_label = st.session_state.get("vma_working_case_label", "")
st.subheader(case_label)

vehicle_paths: List[str] = list(st.session_state["vma_vehicle_paths"])
raw_labels: List[str] = list(st.session_state.get("vma_vehicle_labels") or [])
maneuver_dir = Path(st.session_state["vma_maneuver_dir"])
maneuver_path = maneuver_path_for_id(st.session_state["vma_working_case_id"], maneuver_dir)

if not maneuver_path.exists():
    st.error(f"Maneuver file missing: {maneuver_path}")
    st.stop()

while len(raw_labels) < len(vehicle_paths):
    raw_labels.append(Path(vehicle_paths[len(raw_labels)]).stem)
legend_labels: List[str] = []
seen: Dict[str, int] = {}
for lab in raw_labels:
    if lab in seen:
        seen[lab] += 1
        legend_labels.append(f"{lab} ({seen[lab]})")
    else:
        seen[lab] = 1
        legend_labels.append(lab)

sig = (
    tuple(vehicle_paths),
    st.session_state["vma_vehicle_model"],
    st.session_state["vma_tire_model"],
    st.session_state["vma_solver_backend"],
    st.session_state["vma_scipy_method"],
    float(st.session_state["vma_rk4_dt"]),
    st.session_state["vma_working_case_id"],
)

need_run = (
    "vma_batch_sig" not in st.session_state
    or st.session_state["vma_batch_sig"] != sig
    or "vma_batch_results" not in st.session_state
    or len(st.session_state["vma_batch_results"]) != len(vehicle_paths)
)

if need_run:
    batch_results: List[Dict[str, Any]] = []
    progress = st.progress(0.0)
    status = st.empty()
    for idx, vp in enumerate(vehicle_paths):
        status.text(f"Running case {idx + 1}/{len(vehicle_paths)} …")
        progress.progress((idx + 1) / max(len(vehicle_paths), 1))
        try:
            batch_results.append(
                run_simulation_from_files(
                    Path(vp),
                    maneuver_path,
                    vehicle_model=st.session_state["vma_vehicle_model"],
                    tire_model=st.session_state["vma_tire_model"],
                    solver_backend=st.session_state["vma_solver_backend"],
                    scipy_method=st.session_state["vma_scipy_method"],
                    rk4_dt=st.session_state["vma_rk4_dt"],
                    show_plot=False,
                    include_analytical=True,
                )
            )
        except Exception as e:
            progress.empty()
            status.empty()
            st.error(f"Simulation failed for `{Path(vp).name}`: {e}")
            st.stop()
    progress.empty()
    status.empty()
    st.session_state["vma_batch_results"] = batch_results
    st.session_state["vma_batch_sig"] = sig
else:
    batch_results = st.session_state["vma_batch_results"]

st.success(f"Simulation complete — {len(vehicle_paths)} case(s). Charts overlay all vehicles.")

solver_note = (
    st.session_state["vma_scipy_method"]
    if st.session_state["vma_solver_backend"] == "scipy_ode"
    else f"dt={st.session_state['vma_rk4_dt']}"
)
st.caption(
    f"Plant `{st.session_state['vma_vehicle_model']}` · tire `{st.session_state['vma_tire_model']}` · "
    f"solver `{st.session_state['vma_solver_backend']}` ({solver_note})"
)


def _interp_to_grid(t_src: np.ndarray, y: np.ndarray, t_ref: np.ndarray) -> np.ndarray:
    """Linearly interpolate `y` defined on `t_src` onto `t_ref`."""
    t_src = np.asarray(t_src, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    t_ref = np.asarray(t_ref, dtype=float).ravel()
    if len(t_src) < 2:
        return np.full_like(t_ref, np.nan, dtype=float)
    return np.interp(t_ref, t_src, y)


def _series_map(results: Dict[str, Any]) -> Tuple[np.ndarray, List[str], Dict[str, np.ndarray]]:
    t = np.asarray(results["time"], dtype=float)
    states = results["states"]
    names = list(results["state_names"])
    analytical = results.get("analytical", {})
    derived = results.get("derived_series", {}) or {}
    kin_grid = (analytical.get("kinematic_same_input") or {}).get("states_on_main_grid") or {}
    cmds = analytical.get("commands") or {}

    series: Dict[str, np.ndarray] = {"time": t}
    for i, name in enumerate(names):
        series[f"state:{name}"] = np.asarray(states[:, i], dtype=float)
    for key, values in cmds.items():
        series[f"command:{key}"] = np.asarray(values, dtype=float)
    for key, values in kin_grid.items():
        series[f"kinematic:{key}"] = np.asarray(values, dtype=float)
    for key, values in derived.items():
        series[f"derived:{key}"] = np.asarray(values, dtype=float)
    return t, names, series


def _common_series_keys(batch: List[Dict[str, Any]]) -> List[str]:
    _, _, s0 = _series_map(batch[0])
    keys = set(s0.keys())
    for res in batch[1:]:
        _, _, s = _series_map(res)
        keys &= set(s.keys())
    return sorted(keys)


def _vehicle_color(idx: int) -> str:
    return f"C{idx % 10}"


def _state_intro(name: str) -> Tuple[str, str]:
    """Return (category, brief introduction) for a state name."""
    key = str(name).strip().lower()
    intro_map: Dict[str, Tuple[str, str]] = {
        "x": ("Position", "Global X position of the vehicle on the ground plane."),
        "y": ("Position", "Global Y position of the vehicle on the ground plane."),
        "psi": ("Attitude", "Yaw angle (heading) showing where the vehicle points."),
        "vx": ("Velocity", "Longitudinal speed along vehicle forward direction."),
        "vy": ("Velocity", "Lateral speed indicating side-slip motion."),
        "omega": ("Yaw Dynamics", "Yaw rate (vehicle rotation speed around the vertical axis)."),
        "r": ("Yaw Dynamics", "Yaw rate (vehicle rotation speed around the vertical axis)."),
        "phi": ("Roll Dynamics", "Body roll angle (lean) caused mainly by lateral acceleration."),
        "phi_dot": ("Roll Dynamics", "Roll rate (time derivative of roll angle)."),
        "zu_fl": ("Vertical Corner Dynamics", "Front-left unsprung vertical displacement (wheel-corner heave)."),
        "zu_fr": ("Vertical Corner Dynamics", "Front-right unsprung vertical displacement (wheel-corner heave)."),
        "zu_rl": ("Vertical Corner Dynamics", "Rear-left unsprung vertical displacement (wheel-corner heave)."),
        "zu_rr": ("Vertical Corner Dynamics", "Rear-right unsprung vertical displacement (wheel-corner heave)."),
        "wz_fl": (
            "Vertical Corner Dynamics",
            "Front-left unsprung vertical velocity (time derivative of zu_fl in the current 14-DOF scaffold).",
        ),
        "wz_fr": (
            "Vertical Corner Dynamics",
            "Front-right unsprung vertical velocity (time derivative of zu_fr in the current 14-DOF scaffold).",
        ),
        "wz_rl": (
            "Vertical Corner Dynamics",
            "Rear-left unsprung vertical velocity (time derivative of zu_rl in the current 14-DOF scaffold).",
        ),
        "wz_rr": (
            "Vertical Corner Dynamics",
            "Rear-right unsprung vertical velocity (time derivative of zu_rr in the current 14-DOF scaffold).",
        ),
    }
    if key in intro_map:
        return intro_map[key]
    return (
        "State",
        "Model state variable integrated during simulation; meaning depends on the selected vehicle model definition.",
    )


def _render_batch_overlays(
    batch: List[Dict[str, Any]],
    paths: List[str],
    labels: List[str],
) -> None:
    t_ref, ref_names, _ = _series_map(batch[0])
    t_ref = np.asarray(t_ref, dtype=float)

    for res in batch[1:]:
        _, names, _ = _series_map(res)
        if names != ref_names:
            st.warning(
                "State names differ between batch cases; overlays use the first case's state list. "
                f"First: {ref_names[:4]}… vs other: {names[:4]}…"
            )
            break

    series_keys = _common_series_keys(batch)
    if not series_keys:
        st.error("No common series keys across batch results.")
        return

    st.subheader("State quick-intro summary")
    st.caption("Use this matrix as a quick guide before selecting variables in custom charts.")
    summary_rows = []
    for idx, state_name in enumerate(ref_names, start=1):
        category, intro = _state_intro(state_name)
        summary_rows.append(
            {
                "#": idx,
                "State": state_name,
                "Category": category,
                "Brief introduction": intro,
            }
        )
    st.dataframe(summary_rows, hide_index=True, width="stretch")

    with st.expander("Derived channels for steering geometry / K&C tracking", expanded=True):
        st.markdown(
            """Use these `derived:*` channels in the custom chart builder:

- `derived:alpha_f_eff_rad`, `derived:alpha_r_eff_rad`: effective front/rear slip angles after toe + camber-thrust equivalent + compliance steer.
- `derived:alpha_f_base_rad`, `derived:alpha_r_base_rad`: base slip angles before toe/camber additions.
- `derived:Fy_front_N`, `derived:Fy_rear_N`: estimated front/rear lateral tire forces.
- `derived:delta_cmd_rad`, `derived:delta_eff_rad`, `derived:rear_steer_rad`: commanded steering, effective front steering, and rear steer contribution.
- `derived:delta_front_eff_rad`, `derived:delta_rear_eff_rad`: aliases for effective front steer angle and rear steer angle (rad).
- `derived:alpha_front_eff_rad`, `derived:alpha_rear_eff_rad`: aliases for effective front/rear slip angles (rad).
- `derived:front_toe_deg`, `derived:rear_toe_deg`, `derived:front_camber_deg`, `derived:rear_camber_deg`: geometry values applied in the run.
"""
        )
    st.subheader("Custom chart builder (all vehicles overlaid)")
    col_x, col_y = st.columns(2)
    default_x = "state:x" if "state:x" in series_keys else ("time" if "time" in series_keys else series_keys[0])
    default_y = "state:y" if "state:y" in series_keys else (f"state:{ref_names[0]}" if ref_names else series_keys[0])
    with col_x:
        x_key = st.selectbox(
            "X axis parameter",
            options=series_keys,
            index=series_keys.index(default_x) if default_x in series_keys else 0,
            key="overlay_chart_x",
        )
    with col_y:
        y_key = st.selectbox(
            "Y axis parameter",
            options=series_keys,
            index=series_keys.index(default_y) if default_y in series_keys else 0,
            key="overlay_chart_y",
        )

    fig_custom, ax_custom = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
    any_line = False
    for idx, res in enumerate(batch):
        t_i, _, series = _series_map(res)
        x_raw = np.asarray(series[x_key], dtype=float).ravel()
        y_raw = np.asarray(series[y_key], dtype=float).ravel()
        t_i = np.asarray(t_i, dtype=float).ravel()
        if x_key == "time":
            x_plot = t_ref
            y_plot = _interp_to_grid(t_i, y_raw, t_ref)
        elif y_key == "time":
            x_plot = _interp_to_grid(t_i, x_raw, t_ref)
            y_plot = t_ref
        else:
            x_plot = _interp_to_grid(t_i, x_raw, t_ref)
            y_plot = _interp_to_grid(t_i, y_raw, t_ref)
        m = np.isfinite(x_plot) & np.isfinite(y_plot)
        if np.count_nonzero(m) >= 2:
            c = _vehicle_color(idx)
            ax_custom.plot(x_plot[m], y_plot[m], color=c, linewidth=1.6, label=labels[idx])
            any_line = True
    if any_line:
        ax_custom.set_xlabel(x_key)
        ax_custom.set_ylabel(y_key)
        ax_custom.set_title(f"{y_key} vs {x_key}")
        ax_custom.grid(True)
        ax_custom.legend(loc="best", fontsize=8)
        st.caption("Custom overlay: compare all selected vehicles on the chosen X/Y parameters.")
        st.pyplot(fig_custom)
    else:
        st.warning("Selected parameters do not have enough valid points for plotting.")
    plt.close(fig_custom)

    def _derived_series(res: Dict[str, Any], key: str) -> Optional[np.ndarray]:
        _, _, series = _series_map(res)
        sk = f"derived:{key}"
        if sk not in series:
            return None
        return np.asarray(series[sk], dtype=float).ravel()

    def _all_batch_have_derived(keys: tuple[str, ...]) -> bool:
        for res in batch:
            _, _, s = _series_map(res)
            for k in keys:
                if f"derived:{k}" not in s:
                    return False
        return True

    st.subheader("Fy–α loops (lateral tire force vs effective slip angle)")
    st.caption(
        "Parametric plots in slip-angle space (time advances along the trace). "
        "Effective slip angles include toe, camber-thrust equivalent, compliance steer, and rear steer where configured."
    )
    fy_alpha_keys_f = ("alpha_front_eff_rad", "Fy_front_N")
    fy_alpha_keys_r = ("alpha_rear_eff_rad", "Fy_rear_N")
    if _all_batch_have_derived(fy_alpha_keys_f) and _all_batch_have_derived(fy_alpha_keys_r):
        st.markdown("**1.1 Front tire — Fy vs effective slip angle**")
        fig_af, ax_af = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
        for idx, res in enumerate(batch):
            af = _derived_series(res, "alpha_front_eff_rad")
            fy = _derived_series(res, "Fy_front_N")
            if af is None or fy is None:
                continue
            m = np.isfinite(af) & np.isfinite(fy)
            if np.count_nonzero(m) >= 2:
                ax_af.plot(
                    np.degrees(af[m]),
                    fy[m],
                    color=_vehicle_color(idx),
                    linewidth=1.4,
                    label=labels[idx],
                )
        ax_af.set_xlabel("α_front,eff (deg)")
        ax_af.set_ylabel("Fy,front (N)")
        ax_af.set_title("Front tire lateral force vs effective slip angle")
        ax_af.grid(True)
        ax_af.legend(loc="best", fontsize=8)
        st.pyplot(fig_af)
        plt.close(fig_af)

        st.markdown("**1.2 Rear tire — Fy vs effective slip angle**")
        fig_ar, ax_ar = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
        for idx, res in enumerate(batch):
            ar = _derived_series(res, "alpha_rear_eff_rad")
            fy = _derived_series(res, "Fy_rear_N")
            if ar is None or fy is None:
                continue
            m = np.isfinite(ar) & np.isfinite(fy)
            if np.count_nonzero(m) >= 2:
                ax_ar.plot(
                    np.degrees(ar[m]),
                    fy[m],
                    color=_vehicle_color(idx),
                    linewidth=1.4,
                    label=labels[idx],
                )
        ax_ar.set_xlabel("α_rear,eff (deg)")
        ax_ar.set_ylabel("Fy,rear (N)")
        ax_ar.set_title("Rear tire lateral force vs effective slip angle")
        ax_ar.grid(True)
        ax_ar.legend(loc="best", fontsize=8)
        st.pyplot(fig_ar)
        plt.close(fig_ar)
    else:
        st.info(
            "Fy–α loops need tire-derived channels from the planar model (not available for this result set). "
            "Use **bicycle**, **double_track**, or **dof14** (four-wheel planar path) with tire-based lateral forces."
        )

    st.subheader("K&C tracking (effective steering, slip angles, lateral forces vs time)")
    st.caption(
        "Time histories of effective steer angles, effective slip angles, and estimated lateral tire forces "
        "(includes compliance / K&C terms when set in vehicle JSON or custom slots)."
    )
    kc_specs = [
        ("delta_front_eff_rad", "2.1 derived:delta_front_eff_rad vs time", "δ_front,eff (rad)"),
        ("delta_rear_eff_rad", "2.2 derived:delta_rear_eff_rad vs time", "δ_rear,eff (rad)"),
        ("alpha_front_eff_rad", "2.3 derived:alpha_front_eff_rad vs time", "α_front,eff (rad)"),
        ("alpha_rear_eff_rad", "2.4 derived:alpha_rear_eff_rad vs time", "α_rear,eff (rad)"),
        ("Fy_front_N", "2.5 derived:Fy_front_N vs time", "Fy,front (N)"),
        ("Fy_rear_N", "2.6 derived:Fy_rear_N vs time", "Fy,rear (N)"),
    ]
    kc_keys = tuple(s[0] for s in kc_specs)
    if _all_batch_have_derived(kc_keys):

        def _plot_kc_single(dkey: str, title: str, ylab: str) -> None:
            fig_k, ax_k = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
            for idx, res in enumerate(batch):
                t_i = np.asarray(res["time"], dtype=float).ravel()
                yv = _derived_series(res, dkey)
                if yv is None:
                    continue
                m = np.isfinite(t_i) & np.isfinite(yv)
                if np.count_nonzero(m) < 2:
                    continue
                ax_k.plot(t_i[m], yv[m], color=_vehicle_color(idx), linewidth=1.3, label=labels[idx])
            ax_k.set_title(title, fontsize=10)
            ax_k.set_xlabel("Time (s)")
            ax_k.set_ylabel(ylab)
            ax_k.grid(True)
            ax_k.legend(loc="best", fontsize=8)
            st.pyplot(fig_k)
            plt.close(fig_k)

        for spec in kc_specs:
            _plot_kc_single(spec[0], spec[1], spec[2])
    else:
        st.info("K&C tracking charts require the same derived tire channels as Fy–α loops.")

    r0 = batch[0]
    analytical0 = r0.get("analytical", {})
    steady0 = analytical0.get("steady") or {}
    cmds0 = analytical0.get("commands") or {}

    with st.expander("Analytical benchmark — description & steady-state hints (first case)", expanded=False):
        st.write(analytical0.get("description", ""))
        if steady0:
            st.markdown("**Scalar hints (maneuver-specific):**")
            for k, v in sorted(steady0.items()):
                st.text(f"{k}: {v}")

    if cmds0:
        st.subheader("Open-loop commands (maneuver definition — single trace)")
        st.caption("Same maneuver file for all vehicles; one curve shown.")
        t_cmd = np.asarray(r0["time"], dtype=float)
        fig_steer, ax_steer = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
        ax_steer.plot(t_cmd, cmds0.get("steer_rad", 0 * t_cmd), color="#2a6f2a", linewidth=1.8)
        ax_steer.set_xlabel("Time (s)")
        ax_steer.set_ylabel("delta (rad)")
        ax_steer.set_title("Steering command")
        ax_steer.grid(True)
        st.caption("This chart shows the steering input profile applied over time.")
        st.pyplot(fig_steer)
        plt.close(fig_steer)

        fig_fx, ax_fx = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
        ax_fx.plot(t_cmd, cmds0.get("Fx_N", 0 * t_cmd), color="#2a6f2a", linewidth=1.8)
        ax_fx.set_xlabel("Time (s)")
        ax_fx.set_ylabel("Fx (N)")
        ax_fx.set_title("Longitudinal force command")
        ax_fx.grid(True)
        st.caption("This chart shows commanded longitudinal force throughout the maneuver.")
        st.pyplot(fig_fx)
        plt.close(fig_fx)

        fig_mu, ax_mu = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
        ax_mu.plot(t_cmd, cmds0.get("mu_scale", np.ones_like(t_cmd)), color="#2a6f2a", linewidth=1.8)
        ax_mu.set_xlabel("Time (s)")
        ax_mu.set_ylabel("mu scale")
        ax_mu.set_title("Friction scaling command")
        ax_mu.grid(True)
        st.caption("This chart shows how tire-road friction scaling changes over time.")
        st.pyplot(fig_mu)
        plt.close(fig_mu)

    st.subheader("Time histories (simulation per vehicle)")
    for si, name in enumerate(ref_names):
        fig_state, ax_state = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
        for idx, res in enumerate(batch):
            t_i, names_i, series = _series_map(res)
            if name not in names_i:
                continue
            j = names_i.index(name)
            states = res["states"]
            y_sim = np.asarray(states[:, j], dtype=float)
            y_s = _interp_to_grid(t_i, y_sim, t_ref)
            c = _vehicle_color(idx)
            ax_state.plot(t_ref, y_s, color=c, linewidth=1.5, label=f"{labels[idx]} sim")
        ax_state.set_xlabel("Time (s)")
        ax_state.set_ylabel(name)
        ax_state.set_title(f"State history: {name}")
        ax_state.grid(True)
        ax_state.legend(loc="best", fontsize=8)
        st.caption(f"This chart shows how `{name}` evolves over time for each selected vehicle.")
        st.pyplot(fig_state)
        plt.close(fig_state)

    st.subheader("Planar trajectory (x–y, all vehicles)")
    ix = ref_names.index("x") if "x" in ref_names else 0
    iy = ref_names.index("y") if "y" in ref_names else 1
    fig2, ax = plt.subplots(figsize=_CHART_FIGSIZE_INCHES)
    for idx, res in enumerate(batch):
        t_i, _, _ = _series_map(res)
        states = res["states"]
        xs = np.asarray(states[:, ix], dtype=float)
        ys = np.asarray(states[:, iy], dtype=float)
        x_p = _interp_to_grid(t_i, xs, t_ref)
        y_p = _interp_to_grid(t_i, ys, t_ref)
        m = np.isfinite(x_p) & np.isfinite(y_p)
        c = _vehicle_color(idx)
        ax.plot(x_p[m], y_p[m], color=c, linewidth=1.6, label=f"{labels[idx]} sim")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Path")
    ax.grid(True)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=8)
    st.caption("This chart shows the vehicle path on the ground plane for each selected case.")
    st.pyplot(fig2)
    plt.close(fig2)

    with st.expander("Download results CSV (per vehicle)", expanded=False):
        for idx, (res, vp) in enumerate(zip(batch, paths)):
            t = res["time"]
            names = list(res["state_names"])
            states = res["states"]
            csv_buffer = io.StringIO()
            writer = csv.writer(csv_buffer)
            writer.writerow(["time"] + names)
            for ti, time in enumerate(t):
                writer.writerow([time] + list(np.asarray(states[ti, :]).ravel()))
            st.download_button(
                label=f"Download CSV — {labels[idx]}",
                data=csv_buffer.getvalue(),
                file_name=f"vma_simulation_{Path(vp).stem}.csv",
                mime="text/csv",
                key=f"dl_overlay_{idx}",
            )


_render_batch_overlays(batch_results, vehicle_paths, legend_labels)
