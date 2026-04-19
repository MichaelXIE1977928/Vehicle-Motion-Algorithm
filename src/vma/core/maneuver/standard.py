"""Standard maneuvers: ISO-style open-loop inputs [steer, Fx, mu_scale]."""
import numpy as np


class Maneuver:
    """Maneuver: inputs_at_time returns [steering, longitudinal_force_Fx, mu_scale]."""

    def __init__(self, params: dict):
        self.params = params
        self.maneuver_type = params["type"]

    def inputs_at_time(self, t: float) -> np.ndarray:
        throttle_fx = 0.0
        mu_scale = 1.0

        if self.maneuver_type == "step_steer":
            step_time = self.params["step_time"]
            steer_amp = self.params["steer_amplitude"]
            steering = steer_amp if t >= step_time else 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "constant_radius":
            radius = self.params["radius"]
            wheelbase = float(self.params.get("wheelbase", 2.8))
            steering = np.arctan(wheelbase / radius)
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "double_lane_change":
            duration = self.params["duration"]
            amp = float(self.params.get("steer_amplitude", 0.05))
            t1 = duration * 0.2
            t2 = duration * 0.4
            t3 = duration * 0.6
            t4 = duration * 0.8
            if t < t1:
                steering = 0.0
            elif t < t2:
                steering = amp * np.sin(np.pi * (t - t1) / (t2 - t1))
            elif t < t3:
                steering = amp * np.sin(np.pi * (t - t2) / (t3 - t2)) * -1.0
            elif t < t4:
                steering = amp * np.sin(np.pi * (t - t3) / (t4 - t3))
            else:
                steering = 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "iso3888_2_obstacle":
            duration = self.params["duration"]
            amp = float(self.params.get("steer_amplitude", 0.06))
            t1 = duration * 0.15
            t2 = duration * 0.35
            t3 = duration * 0.55
            t4 = duration * 0.75
            if t < t1:
                steering = 0.0
            elif t < t2:
                steering = amp * np.sin(0.5 * np.pi * (t - t1) / (t2 - t1))
            elif t < t3:
                steering = amp * np.cos(0.5 * np.pi * (t - t2) / (t3 - t2))
            elif t < t4:
                steering = -0.6 * amp * np.sin(0.5 * np.pi * (t - t3) / (t4 - t3))
            else:
                steering = 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "lane_change_with_brake":
            # ISO 3888-3-class proxy: same steer envelope as double lane change, plus commanded longitudinal decel.
            duration = float(self.params["duration"])
            amp = float(self.params.get("steer_amplitude", 0.05))
            t1 = duration * 0.2
            t2 = duration * 0.4
            t3 = duration * 0.6
            t4 = duration * 0.8
            if t < t1:
                steering = 0.0
            elif t < t2:
                steering = amp * np.sin(np.pi * (t - t1) / (t2 - t1))
            elif t < t3:
                steering = amp * np.sin(np.pi * (t - t2) / (t3 - t2)) * -1.0
            elif t < t4:
                steering = amp * np.sin(np.pi * (t - t3) / (t4 - t3))
            else:
                steering = 0.0
            brake_t = float(self.params["brake_start_time"])
            decel = float(self.params["brake_decel_m_s2"])
            ref_mass = float(self.params.get("reference_mass", 1500.0))
            fx = -ref_mass * decel if t >= brake_t else 0.0
            return np.array([steering, fx, mu_scale])

        if self.maneuver_type == "ramp_step_steer":
            # ISO 7401-class proxy: trapezoidal steer angle (ramp–hold–return) for repeatable transients.
            T_ramp = float(self.params["ramp_time"])
            T_hold = float(self.params["hold_time"])
            amp = float(self.params["steer_amplitude"])
            t_end = 2.0 * T_ramp + T_hold
            if t < T_ramp:
                steering = amp * t / max(T_ramp, 1e-9)
            elif t < T_ramp + T_hold:
                steering = amp
            elif t < t_end:
                steering = amp * (1.0 - (t - (T_ramp + T_hold)) / max(T_ramp, 1e-9))
            else:
                steering = 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "straight_line_brake":
            brake_t = float(self.params["brake_start_time"])
            decel = float(self.params["brake_decel_m_s2"])
            ref_mass = float(self.params.get("reference_mass", 1500.0))
            fx = -ref_mass * decel if t >= brake_t else 0.0
            return np.array([0.0, fx, mu_scale])

        if self.maneuver_type == "sine_sweep":
            f_start = self.params["freq_start"]
            f_end = self.params["freq_end"]
            amplitude = self.params["amplitude"]
            duration = self.params["duration"]
            if t <= duration:
                f = f_start + (f_end - f_start) * t / duration
                steering = amplitude * np.sin(2 * np.pi * f * t)
            else:
                steering = 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "iso7401_impulse":
            t0 = float(self.params["impulse_time"])
            width = float(self.params["impulse_width"])
            amp = float(self.params["steer_amplitude"])
            steering = amp if (t0 <= t < t0 + width) else 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "iso7975_braking_in_turn":
            radius = self.params["radius"]
            wheelbase = float(self.params.get("wheelbase", 2.8))
            steering = np.arctan(wheelbase / radius)
            brake_t = float(self.params["brake_start_time"])
            decel = float(self.params["brake_decel_m_s2"])
            ref_mass = float(self.params.get("reference_mass", 1500.0))
            if t >= brake_t:
                fx = -ref_mass * decel
            else:
                fx = 0.0
            return np.array([steering, fx, mu_scale])

        if self.maneuver_type == "ackermann_slow_slalom_linrange":
            # Quasi-steady Ackermann steer for radius R, plus low-amplitude slow sinusoid for
            # understeer-gradient / linear tire-range checks (open-loop proxy, not a track layout).
            radius = float(self.params["radius"])
            wheelbase = float(self.params.get("wheelbase", 2.8))
            amp = float(self.params.get("sine_amplitude_rad", 0.005))
            freq = float(self.params.get("sine_freq_hz", 0.12))
            phase = float(self.params.get("phase_rad", 0.0))
            steer_mean = np.arctan(wheelbase / max(radius, 1e-6))
            steering = steer_mean + amp * np.sin(2.0 * np.pi * freq * t + phase)
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "throttle_on_in_turn":
            # Same constant-radius steer machinery as ISO 7975, with positive Fx (drive) schedule
            # after throttle_start_time — open-loop proxy for throttle-on understeer push studies.
            radius = float(self.params["radius"])
            wheelbase = float(self.params.get("wheelbase", 2.8))
            steering = np.arctan(wheelbase / max(radius, 1e-6))
            throttle_t = float(self.params["throttle_start_time"])
            accel = float(self.params["longitudinal_accel_m_s2"])
            ref_mass = float(self.params.get("reference_mass", 1500.0))
            fx = ref_mass * accel if t >= throttle_t else 0.0
            return np.array([steering, fx, mu_scale])

        if self.maneuver_type == "iso17288_uturn":
            duration = float(self.params["duration"])
            max_steer = float(self.params["max_steer"])
            t_ramp = duration * 0.25
            if t < t_ramp:
                steering = max_steer * (t / t_ramp)
            elif t < duration - 1.0:
                steering = max_steer
            else:
                steering = max_steer * max(0.0, (duration - t))
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "fish_hook":
            # Open-loop proxy: ramp + hold positive steer, fast ramp to larger negative steer, hold, return to zero.
            # Not a certified NHTSA / OEM track replay; for limit-handling / roll-yaw excitation studies.
            D = float(self.params["duration"])
            if D <= 1e-6:
                return np.array([0.0, throttle_fx, mu_scale])
            sp = float(self.params["steer_positive"])
            sn = float(self.params["steer_negative"])
            f1 = float(self.params.get("frac_ramp_pos_end", 0.07))
            f2 = float(self.params.get("frac_hold_pos_end", 0.30))
            f3 = float(self.params.get("frac_ramp_neg_end", 0.36))
            f4 = float(self.params.get("frac_hold_neg_end", 0.85))
            t1, t2, t3, t4 = f1 * D, f2 * D, f3 * D, f4 * D
            if t < t1:
                steering = sp * (t / max(t1, 1e-9))
            elif t < t2:
                steering = sp
            elif t < t3:
                steering = sp + (sn - sp) * ((t - t2) / max(t3 - t2, 1e-9))
            elif t < t4:
                steering = sn
            elif t < D:
                steering = sn * (1.0 - (t - t4) / max(D - t4, 1e-9))
            else:
                steering = 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "sine_with_dwell":
            # First quarter-sine to peak, dwell at peak, quarter-cosine back toward neutral (open-loop ESC-class proxy).
            A = float(self.params["amplitude"])
            freq = float(self.params["frequency_hz"])
            dwell = float(self.params["dwell_s"])
            if freq < 1e-6:
                return np.array([0.0, throttle_fx, mu_scale])
            t_quarter = 0.25 / freq
            t_dwell_end = t_quarter + dwell
            if t < t_quarter:
                steering = A * np.sin(2.0 * np.pi * freq * t)
            elif t < t_dwell_end:
                steering = A
            elif t < t_dwell_end + t_quarter:
                tau = t - t_dwell_end
                steering = A * np.cos(0.5 * np.pi * tau / max(t_quarter, 1e-9))
            else:
                steering = 0.0
            return np.array([steering, throttle_fx, mu_scale])

        if self.maneuver_type == "split_mu":
            split_time = self.params["split_time"]
            mu_ratio = float(self.params["mu_ratio"])
            m = mu_ratio if t >= split_time else 1.0
            return np.array([0.0, 0.0, m])

        return np.array([0.0, throttle_fx, mu_scale])
