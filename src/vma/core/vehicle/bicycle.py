"""Bicycle model with explicit roll state (lateral, yaw, longitudinal, roll)."""
import numpy as np
from .base import VehicleModel


class BicycleModel(VehicleModel):
    """
    Simple bicycle model with linear tire forces.

    State vector: [x, y, psi, vx, vy, omega, phi, phi_dot]
    Inputs: [steering_angle, throttle_brake]
    """
    def __init__(self, params: dict):
        super().__init__(params)
        self.mass = float(params["mass"])
        self.Iz = float(params["yaw_inertia"])
        self.lf = float(params["cg_to_front"])
        self.lr = float(params["cg_to_rear"])
        self.Cf = float(params["front_cornering_stiffness"])
        self.Cr = float(params["rear_cornering_stiffness"])
        self.sprung_mass = float(params.get("sprung_mass", 0.9 * self.mass))
        self.roll_inertia = float(params.get("roll_inertia", 650.0))
        self.cg_height = float(params.get("cg_height", 0.55))
        self.front_roll_stiffness = float(params.get("front_roll_stiffness", 28000.0))
        self.rear_roll_stiffness = float(params.get("rear_roll_stiffness", 25000.0))
        self.front_roll_damping = float(params.get("front_roll_damping", 2200.0))
        self.rear_roll_damping = float(params.get("rear_roll_damping", 2000.0))
        self.track_width = float(params.get("track_width", 1.58))
        self.load_transfer_coupling = float(params.get("load_transfer_coupling", 0.08))
        self.roll_k = self.front_roll_stiffness + self.rear_roll_stiffness
        self.roll_c = self.front_roll_damping + self.rear_roll_damping

    def state_derivative(self, t: float, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        x, y, psi, vx, vy, omega, phi, phi_dot = state
        delta, Fx = inputs

        # Tire slip angles
        alpha_f = -np.arctan((vy + self.lf * omega) / vx) + delta
        alpha_r = -np.arctan((vy - self.lr * omega) / vx)

        # Lateral forces (linear)
        Fyf = self.Cf * alpha_f
        Fyr = self.Cr * alpha_r

        # Differential equations
        vx_dot = (Fx - Fyf * np.sin(delta) + self.mass * vy * omega) / self.mass
        vy_dot = (Fyf * np.cos(delta) + Fyr - self.mass * vx * omega) / self.mass
        omega_dot = (self.lf * Fyf * np.cos(delta) - self.lr * Fyr) / self.Iz
        ay = vy_dot + vx * omega
        track_eff = self.track_width if self.track_width > 0.3 else 0.3
        load_transfer_term = (self.load_transfer_coupling * self.sprung_mass * self.cg_height * ay) / track_eff
        roll_num = self.sprung_mass * self.cg_height * ay + load_transfer_term
        phi_ddot = (roll_num - self.roll_c * phi_dot - self.roll_k * phi) / self.roll_inertia
        x_dot = vx * np.cos(psi) - vy * np.sin(psi)
        y_dot = vx * np.sin(psi) + vy * np.cos(psi)
        psi_dot = omega

        return np.array([x_dot, y_dot, psi_dot, vx_dot, vy_dot, omega_dot, phi_dot, phi_ddot])

    @property
    def state_names(self) -> list:
        return ["x", "y", "psi", "vx", "vy", "omega", "phi", "phi_dot"]
