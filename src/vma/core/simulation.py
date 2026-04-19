"""Orchestrates a simulation (core + tires + maneuver + solver)."""
import numpy as np
from vma.core.vehicle.kinematic import KinematicModel
from vma.core.vehicle.dof14 import DOF14Model
from vma.core.vehicle.double_track_planar import DoubleTrackPlanarModel
from vma.core.tire.linear import LinearTire
from vma.core.tire.fiala import FialaTire
from vma.core.tire.dugoff import DugoffTire
from vma.core.tire.magic_formula import MagicFormulaTire
from vma.core.tire.brush_combined import BrushCombinedTire
from vma.core.solver.scipy_ode import ScipyOdeSolver


def _geometry_kc_terms(vehicle):
    p = getattr(vehicle, "params", {}) or {}
    d2r = np.pi / 180.0
    return {
        "toe_f": float(p.get("front_toe_deg", 0.0)) * d2r,
        "toe_r": float(p.get("rear_toe_deg", 0.0)) * d2r,
        "camber_f": float(p.get("front_camber_deg", 0.0)) * d2r,
        "camber_r": float(p.get("rear_camber_deg", 0.0)) * d2r,
        "front_steer_gain": float(p.get("front_steer_gain", 1.0)),
        "rear_steer_gain": float(p.get("rear_steer_gain", 0.0)),
        "compliance_steer_gain": float(p.get("compliance_steer_gain", 0.0)),
        "camber_thrust_gain": float(p.get("camber_thrust_gain", 0.0)),
    }


def _scale_tires(tire_front, tire_rear, mu_scale):
    if mu_scale >= 0.999:
        return tire_front, tire_rear
    if isinstance(tire_front, LinearTire):
        return (
            LinearTire(tire_front.cornering_stiffness * mu_scale),
            LinearTire(tire_rear.cornering_stiffness * mu_scale),
        )
    if isinstance(tire_front, FialaTire):
        return (
            FialaTire(tire_front.cornering_stiffness, tire_front.mu * mu_scale),
            FialaTire(tire_rear.cornering_stiffness, tire_rear.mu * mu_scale),
        )
    if isinstance(tire_front, DugoffTire):
        return (
            DugoffTire(tire_front.cornering_stiffness, tire_front.mu * mu_scale),
            DugoffTire(tire_rear.cornering_stiffness, tire_rear.mu * mu_scale),
        )
    if isinstance(tire_front, MagicFormulaTire):
        p = dict(tire_front.params)
        p["mu"] = float(p.get("mu", 0.85)) * mu_scale
        return MagicFormulaTire(p), MagicFormulaTire(p)
    if isinstance(tire_front, BrushCombinedTire):
        p = dict(tire_front.params)
        p["mu"] = float(p.get("mu", 0.85)) * mu_scale
        return BrushCombinedTire(p), BrushCombinedTire(p)
    return tire_front, tire_rear


def _tire_lateral_force(tire, alpha: float, fz: float, kappa_proxy: float = 0.0) -> float:
    if hasattr(tire, "lateral_force_combined"):
        return float(tire.lateral_force_combined(alpha, fz, kappa_proxy))
    return float(tire.lateral_force(alpha, fz))


def _tire_mu_value(tire) -> float:
    if hasattr(tire, "mu"):
        return float(getattr(tire, "mu"))
    if hasattr(tire, "params"):
        return float(getattr(tire, "params", {}).get("mu", 0.9))
    return 0.9


def _scale_tire_quartet(tire_front, tire_rear, mu_ratio):
    """Front axle tires share `tire_front`; rear axle shares `tire_rear` (FL, FR, RL, RR)."""
    tf, tr = _scale_tires(tire_front, tire_rear, mu_ratio)
    return tf, tf, tr, tr


def _double_track_corner_velocities(vx: float, vy: float, omega: float, lf: float, lr: float, half_tw: float):
    """Rigid-body patch velocities in body frame (x forward, y left)."""
    vx_fl = vx - omega * half_tw
    vy_fl = vy + lf * omega
    vx_fr = vx + omega * half_tw
    vy_fr = vy + lf * omega
    vx_rl = vx - omega * half_tw
    vy_rl = vy - lr * omega
    vx_rr = vx + omega * half_tw
    vy_rr = vy - lr * omega
    return vx_fl, vy_fl, vx_fr, vy_fr, vx_rl, vy_rl, vx_rr, vy_rr


def run_simulation(vehicle, tire_front, tire_rear, maneuver, solver=None, t_span=None, y0=None):
    if t_span is None:
        t_span = (0.0, float(maneuver.params.get("duration", 5.0)))

    if solver is None:
        solver = ScipyOdeSolver()

    names = list(vehicle.state_names)
    n = len(names)
    if y0 is None:
        y0 = np.zeros(n)
        v0 = float(
            maneuver.params.get(
                "initial_forward_speed",
                maneuver.params.get("speed", 15.0 if isinstance(vehicle, KinematicModel) else 0.0),
            )
        )
        if "vx" in names:
            y0[names.index("vx")] = v0
    y0 = np.asarray(y0, dtype=float)

    is_kin = isinstance(vehicle, KinematicModel)
    is_dof14 = isinstance(vehicle, DOF14Model)
    # True four-wheel planar slip/force resolution (double-track) + optional vertical scaffold (dof14).
    is_four_wheel_planar = isinstance(vehicle, (DoubleTrackPlanarModel, DOF14Model))
    geo = _geometry_kc_terms(vehicle)

    def rhs(t, y):
        delta, Fx, mu_ratio = maneuver.inputs_at_time(t)
        if is_kin:
            return vehicle.state_derivative(t, y, np.array([delta, Fx]))

        if is_dof14:
            y8 = y[:8]
            zu = y[8:12]
            wz = y[12:16]
        else:
            y8 = y

        m = vehicle.mass
        Iz = vehicle.Iz
        lf = vehicle.lf
        lr = vehicle.lr
        x, _y, psi, vx, vy, omega, phi, phi_dot = y8
        vx_safe = vx if vx >= 0.01 else 0.01

        ay_est = vx * omega
        delta_eff = geo["front_steer_gain"] * delta + geo["compliance_steer_gain"] * ay_est
        rear_steer = geo["rear_steer_gain"] * delta_eff

        if is_four_wheel_planar:
            half_tw = max(0.5 * vehicle.track_width, 0.15)
            t_fl, t_fr, t_rl, t_rr = _scale_tire_quartet(tire_front, tire_rear, mu_ratio)
            vx_fl, vy_fl, vx_fr, vy_fr, vx_rl, vy_rl, vx_rr, vy_rr = _double_track_corner_velocities(
                vx, vy, omega, lf, lr, half_tw
            )

            def _vx_s(vxc: float) -> float:
                if abs(vxc) >= 0.01:
                    return float(vxc)
                return float(np.copysign(0.01, vxc if vxc != 0.0 else 1.0))

            alpha_fl = (
                -np.arctan(vy_fl / _vx_s(vx_fl))
                + delta_eff
                + geo["toe_f"]
                + geo["camber_thrust_gain"] * geo["camber_f"]
            )
            alpha_fr = (
                -np.arctan(vy_fr / _vx_s(vx_fr))
                + delta_eff
                + geo["toe_f"]
                + geo["camber_thrust_gain"] * geo["camber_f"]
            )
            alpha_rl = (
                -np.arctan(vy_rl / _vx_s(vx_rl))
                - rear_steer
                + geo["toe_r"]
                + geo["camber_thrust_gain"] * geo["camber_r"]
            )
            alpha_rr = (
                -np.arctan(vy_rr / _vx_s(vx_rr))
                - rear_steer
                + geo["toe_r"]
                + geo["camber_thrust_gain"] * geo["camber_r"]
            )

            Fzf = m * 9.81 * lr / (lf + lr)
            Fzr = m * 9.81 * lf / (lf + lr)
            Fz_fl = 0.5 * Fzf
            Fz_fr = 0.5 * Fzf
            Fz_rl = 0.5 * Fzr
            Fz_rr = 0.5 * Fzr

            p = getattr(vehicle, "params", {}) or {}
            drive_bias_front = float(p.get("drive_bias_front", 0.4))
            brake_bias_front = float(p.get("brake_bias_front", 0.6))
            front_share = drive_bias_front if Fx >= 0.0 else brake_bias_front
            front_share = float(np.clip(front_share, 0.0, 1.0))
            fx_front = front_share * Fx
            fx_rear = Fx - fx_front
            fx_fl = 0.5 * fx_front
            fx_fr = 0.5 * fx_front
            fx_rl = 0.5 * fx_rear
            fx_rr = 0.5 * fx_rear
            k_fl = fx_fl / max(_tire_mu_value(t_fl) * Fz_fl, 1.0)
            k_fr = fx_fr / max(_tire_mu_value(t_fr) * Fz_fr, 1.0)
            k_rl = fx_rl / max(_tire_mu_value(t_rl) * Fz_rl, 1.0)
            k_rr = fx_rr / max(_tire_mu_value(t_rr) * Fz_rr, 1.0)

            Fy_fl_m = _tire_lateral_force(t_fl, float(alpha_fl), float(Fz_fl), float(k_fl))
            Fy_fr_m = _tire_lateral_force(t_fr, float(alpha_fr), float(Fz_fr), float(k_fr))
            Fy_rl_m = _tire_lateral_force(t_rl, float(alpha_rl), float(Fz_rl), float(k_rl))
            Fy_rr_m = _tire_lateral_force(t_rr, float(alpha_rr), float(Fz_rr), float(k_rr))

            Fx_fl_b = Fy_fl_m * np.sin(delta_eff)
            Fy_fl_b = Fy_fl_m * np.cos(delta_eff)
            Fx_fr_b = Fy_fr_m * np.sin(delta_eff)
            Fy_fr_b = Fy_fr_m * np.cos(delta_eff)

            sig_r = float(-rear_steer)
            Fx_rl_b = Fy_rl_m * np.sin(sig_r)
            Fy_rl_b = Fy_rl_m * np.cos(sig_r)
            Fx_rr_b = Fy_rr_m * np.sin(sig_r)
            Fy_rr_b = Fy_rr_m * np.cos(sig_r)

            sum_fx_body = Fx_fl_b + Fx_fr_b + Fx_rl_b + Fx_rr_b
            sum_fy_body = Fy_fl_b + Fy_fr_b + Fy_rl_b + Fy_rr_b

            vx_dot = (Fx - sum_fx_body + m * vy * omega) / m
            vy_dot = (sum_fy_body - m * vx * omega) / m
            mz = (
                lf * (Fy_fl_b + Fy_fr_b)
                - lr * (Fy_rl_b + Fy_rr_b)
                + half_tw * (-Fx_fl_b + Fx_fr_b - Fx_rl_b + Fx_rr_b)
            )
            omega_dot = mz / Iz
        else:
            tf, tr = _scale_tires(tire_front, tire_rear, mu_ratio)
            alpha_f_base = -np.arctan((vy + lf * omega) / vx_safe) + delta_eff
            alpha_r_base = -np.arctan((vy - lr * omega) / vx_safe) - rear_steer
            alpha_f = alpha_f_base + geo["toe_f"] + geo["camber_thrust_gain"] * geo["camber_f"]
            alpha_r = alpha_r_base + geo["toe_r"] + geo["camber_thrust_gain"] * geo["camber_r"]

            Fzf = m * 9.81 * lr / (lf + lr)
            Fzr = m * 9.81 * lf / (lf + lr)

            p = getattr(vehicle, "params", {}) or {}
            drive_bias_front = float(p.get("drive_bias_front", 0.4))
            brake_bias_front = float(p.get("brake_bias_front", 0.6))
            front_share = drive_bias_front if Fx >= 0.0 else brake_bias_front
            front_share = float(np.clip(front_share, 0.0, 1.0))
            fx_front = front_share * Fx
            fx_rear = Fx - fx_front
            kappa_front = fx_front / max(_tire_mu_value(tf) * Fzf, 1.0)
            kappa_rear = fx_rear / max(_tire_mu_value(tr) * Fzr, 1.0)
            Fyf = _tire_lateral_force(tf, alpha_f, Fzf, kappa_front)
            Fyr = _tire_lateral_force(tr, alpha_r, Fzr, kappa_rear)

            vx_dot = (Fx - Fyf * np.sin(delta) + m * vy * omega) / m
            vy_dot = (Fyf * np.cos(delta) + Fyr - m * vx * omega) / m
            omega_dot = (lf * Fyf * np.cos(delta) - lr * Fyr) / Iz

        ay = vy_dot + vx * omega
        track_eff = vehicle.track_width if vehicle.track_width > 0.3 else 0.3
        load_transfer_term = (
            vehicle.load_transfer_coupling * vehicle.sprung_mass * vehicle.cg_height * ay
        ) / track_eff
        roll_num = vehicle.sprung_mass * vehicle.cg_height * ay + load_transfer_term
        phi_ddot = (roll_num - vehicle.roll_c * phi_dot - vehicle.roll_k * phi) / vehicle.roll_inertia
        x_dot = vx * np.cos(psi) - vy * np.sin(psi)
        y_dot = vx * np.sin(psi) + vy * np.cos(psi)
        psi_dot = omega
        d8 = np.array([x_dot, y_dot, psi_dot, vx_dot, vy_dot, omega_dot, phi_dot, phi_ddot])

        if is_dof14:
            dz = wz
            dw = -vehicle._wn2 * zu - vehicle._2zwn * wz
            dvert = np.concatenate([dz, dw])
            return np.concatenate([d8, dvert])
        return d8

    t, states = solver.integrate(rhs, t_span, y0)
    derived = {}
    if "phi" in names and "vx" in names and "omega" in names:
        i_phi = names.index("phi")
        i_vx = names.index("vx")
        i_omega = names.index("omega")
        phi_deg = np.degrees(states[:, i_phi])
        ay = states[:, i_vx] * states[:, i_omega]
        ay_g = ay / 9.81
        ay_g_safe = np.where(np.abs(ay_g) < 1e-4, np.nan, ay_g)
        roll_gradient = phi_deg / ay_g_safe
        derived["phi_deg"] = phi_deg
        derived["ay_mps2"] = ay
        derived["ay_g"] = ay_g
        derived["roll_gradient_deg_per_g"] = roll_gradient
    if all(s in names for s in ("vx", "vy", "omega")):
        ix_vx = names.index("vx")
        ix_vy = names.index("vy")
        ix_om = names.index("omega")
        vx_arr = np.asarray(states[:, ix_vx], dtype=float)
        vy_arr = np.asarray(states[:, ix_vy], dtype=float)
        om_arr = np.asarray(states[:, ix_om], dtype=float)
        delta_arr = np.zeros_like(vx_arr)
        fx_arr = np.zeros_like(vx_arr)
        mu_arr = np.ones_like(vx_arr)
        for i, ti in enumerate(t):
            d_i, fx_i, mu_i = maneuver.inputs_at_time(float(ti))
            delta_arr[i] = float(d_i)
            fx_arr[i] = float(fx_i)
            mu_arr[i] = float(mu_i)

        vx_safe_arr = np.where(vx_arr >= 0.01, vx_arr, 0.01)
        ay_est_arr = vx_arr * om_arr
        delta_eff_arr = (
            geo["front_steer_gain"] * delta_arr
            + geo["compliance_steer_gain"] * ay_est_arr
        )
        rear_steer_arr = geo["rear_steer_gain"] * delta_eff_arr

        Fzf = vehicle.mass * 9.81 * vehicle.lr / (vehicle.lf + vehicle.lr)
        Fzr = vehicle.mass * 9.81 * vehicle.lf / (vehicle.lf + vehicle.lr)
        p = getattr(vehicle, "params", {}) or {}
        drive_bias_front = float(p.get("drive_bias_front", 0.4))
        brake_bias_front = float(p.get("brake_bias_front", 0.6))

        if is_four_wheel_planar:
            half_tw = max(0.5 * vehicle.track_width, 0.15)
            lf_v = vehicle.lf
            lr_v = vehicle.lr
            n_t = len(t)
            alpha_fl = np.zeros(n_t)
            alpha_fr = np.zeros(n_t)
            alpha_rl = np.zeros(n_t)
            alpha_rr = np.zeros(n_t)
            fy_fl = np.zeros(n_t)
            fy_fr = np.zeros(n_t)
            fy_rl = np.zeros(n_t)
            fy_rr = np.zeros(n_t)
            for i in range(n_t):
                vx_i = float(vx_arr[i])
                vy_i = float(vy_arr[i])
                om_i = float(om_arr[i])
                vx_fl, vy_fl, vx_fr, vy_fr, vx_rl, vy_rl, vx_rr, vy_rr = _double_track_corner_velocities(
                    vx_i, vy_i, om_i, lf_v, lr_v, half_tw
                )

                def _vx_s(vxc: float) -> float:
                    if abs(vxc) >= 0.01:
                        return float(vxc)
                    return float(np.copysign(0.01, vxc if vxc != 0.0 else 1.0))

                de = float(delta_eff_arr[i])
                rs = float(rear_steer_arr[i])
                alpha_fl[i] = (
                    -np.arctan(vy_fl / _vx_s(vx_fl))
                    + de
                    + geo["toe_f"]
                    + geo["camber_thrust_gain"] * geo["camber_f"]
                )
                alpha_fr[i] = (
                    -np.arctan(vy_fr / _vx_s(vx_fr))
                    + de
                    + geo["toe_f"]
                    + geo["camber_thrust_gain"] * geo["camber_f"]
                )
                alpha_rl[i] = (
                    -np.arctan(vy_rl / _vx_s(vx_rl))
                    - rs
                    + geo["toe_r"]
                    + geo["camber_thrust_gain"] * geo["camber_r"]
                )
                alpha_rr[i] = (
                    -np.arctan(vy_rr / _vx_s(vx_rr))
                    - rs
                    + geo["toe_r"]
                    + geo["camber_thrust_gain"] * geo["camber_r"]
                )
                t_fl_i, t_fr_i, t_rl_i, t_rr_i = _scale_tire_quartet(tire_front, tire_rear, float(mu_arr[i]))
                fz_fl = 0.5 * Fzf
                fz_fr = 0.5 * Fzf
                fz_rl = 0.5 * Fzr
                fz_rr = 0.5 * Fzr
                fx_i = float(fx_arr[i])
                front_share = drive_bias_front if fx_i >= 0.0 else brake_bias_front
                front_share = float(np.clip(front_share, 0.0, 1.0))
                fx_front = front_share * fx_i
                fx_rear = fx_i - fx_front
                k_fl = (0.5 * fx_front) / max(_tire_mu_value(t_fl_i) * fz_fl, 1.0)
                k_fr = (0.5 * fx_front) / max(_tire_mu_value(t_fr_i) * fz_fr, 1.0)
                k_rl = (0.5 * fx_rear) / max(_tire_mu_value(t_rl_i) * fz_rl, 1.0)
                k_rr = (0.5 * fx_rear) / max(_tire_mu_value(t_rr_i) * fz_rr, 1.0)
                fy_fl[i] = _tire_lateral_force(t_fl_i, float(alpha_fl[i]), float(fz_fl), float(k_fl))
                fy_fr[i] = _tire_lateral_force(t_fr_i, float(alpha_fr[i]), float(fz_fr), float(k_fr))
                fy_rl[i] = _tire_lateral_force(t_rl_i, float(alpha_rl[i]), float(fz_rl), float(k_rl))
                fy_rr[i] = _tire_lateral_force(t_rr_i, float(alpha_rr[i]), float(fz_rr), float(k_rr))

            sig_r = -rear_steer_arr
            fy_fl_b = fy_fl * np.cos(delta_eff_arr)
            fy_fr_b = fy_fr * np.cos(delta_eff_arr)
            fy_rl_b = fy_rl * np.cos(sig_r)
            fy_rr_b = fy_rr * np.cos(sig_r)

            alpha_f_base = 0.5 * (alpha_fl + alpha_fr)
            alpha_r_base = 0.5 * (alpha_rl + alpha_rr)
            fyf_arr = fy_fl_b + fy_fr_b
            fyr_arr = fy_rl_b + fy_rr_b

            derived["alpha_fl_rad"] = alpha_fl
            derived["alpha_fr_rad"] = alpha_fr
            derived["alpha_rl_rad"] = alpha_rl
            derived["alpha_rr_rad"] = alpha_rr
            derived["Fy_fl_tire_N"] = fy_fl
            derived["Fy_fr_tire_N"] = fy_fr
            derived["Fy_rl_tire_N"] = fy_rl
            derived["Fy_rr_tire_N"] = fy_rr
        else:
            alpha_f_base = -np.arctan((vy_arr + vehicle.lf * om_arr) / vx_safe_arr) + delta_eff_arr
            alpha_r_base = -np.arctan((vy_arr - vehicle.lr * om_arr) / vx_safe_arr) - rear_steer_arr
            alpha_f_eff = alpha_f_base + geo["toe_f"] + geo["camber_thrust_gain"] * geo["camber_f"]
            alpha_r_eff = alpha_r_base + geo["toe_r"] + geo["camber_thrust_gain"] * geo["camber_r"]

            fyf_arr = np.zeros_like(vx_arr)
            fyr_arr = np.zeros_like(vx_arr)
            for i in range(len(t)):
                tf_i, tr_i = _scale_tires(tire_front, tire_rear, mu_arr[i])
                fx_i = float(fx_arr[i])
                front_share = drive_bias_front if fx_i >= 0.0 else brake_bias_front
                front_share = float(np.clip(front_share, 0.0, 1.0))
                fx_front = front_share * fx_i
                fx_rear = fx_i - fx_front
                kappa_front = fx_front / max(_tire_mu_value(tf_i) * Fzf, 1.0)
                kappa_rear = fx_rear / max(_tire_mu_value(tr_i) * Fzr, 1.0)
                fyf_arr[i] = _tire_lateral_force(tf_i, float(alpha_f_eff[i]), float(Fzf), float(kappa_front))
                fyr_arr[i] = _tire_lateral_force(tr_i, float(alpha_r_eff[i]), float(Fzr), float(kappa_rear))

        derived["delta_cmd_rad"] = delta_arr
        derived["delta_eff_rad"] = delta_eff_arr
        derived["rear_steer_rad"] = rear_steer_arr
        derived["delta_front_eff_rad"] = delta_eff_arr
        derived["delta_rear_eff_rad"] = rear_steer_arr
        derived["alpha_f_base_rad"] = alpha_f_base
        derived["alpha_r_base_rad"] = alpha_r_base
        if is_four_wheel_planar:
            derived["alpha_f_eff_rad"] = 0.5 * (alpha_fl + alpha_fr)
            derived["alpha_r_eff_rad"] = 0.5 * (alpha_rl + alpha_rr)
        else:
            derived["alpha_f_eff_rad"] = alpha_f_eff
            derived["alpha_r_eff_rad"] = alpha_r_eff
        derived["alpha_front_eff_rad"] = derived["alpha_f_eff_rad"]
        derived["alpha_rear_eff_rad"] = derived["alpha_r_eff_rad"]
        derived["Fy_front_N"] = fyf_arr
        derived["Fy_rear_N"] = fyr_arr
        derived["Fx_cmd_N"] = fx_arr
        derived["mu_scale_cmd"] = mu_arr
        derived["front_toe_deg"] = np.full_like(vx_arr, np.degrees(geo["toe_f"]))
        derived["rear_toe_deg"] = np.full_like(vx_arr, np.degrees(geo["toe_r"]))
        derived["front_camber_deg"] = np.full_like(vx_arr, np.degrees(geo["camber_f"]))
        derived["rear_camber_deg"] = np.full_like(vx_arr, np.degrees(geo["camber_r"]))
    return {"time": t, "states": states, "state_names": names, "derived_series": derived}
