"""Kinematic model (for low speed / path following without slip dynamics)."""
import numpy as np
from .base import VehicleModel


class KinematicModel(VehicleModel):
    """
    Kinematic bicycle model (no lateral slip dynamics).
    State: [x, y, psi, vx, vy, omega] — omega kept for interface compatibility (unused).
    Inputs: [steering_angle, longitudinal_force_Fx]
    """
    def __init__(self, params: dict):
        super().__init__(params)
        self.lf = float(params["cg_to_front"])
        self.lr = float(params["cg_to_rear"])
        self.mass = float(params.get("mass", 1500.0))
        self.L = self.lf + self.lr

    def state_derivative(self, t: float, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        x, y, psi, vx, vy, _omega = state
        delta, Fx = float(inputs[0]), float(inputs[1])

        v = float(np.hypot(vx, vy))
        if v < 0.05:
            v = 0.05

        beta = np.arctan((self.lr / self.L) * np.tan(delta))
        x_dot = v * np.cos(psi + beta)
        y_dot = v * np.sin(psi + beta)
        psi_dot = (v / self.L) * np.tan(delta)

        # Align longitudinal force with body x-axis velocity component
        vx_dot = Fx / self.mass
        vy_dot = 0.0
        omega_dot = 0.0

        return np.array([x_dot, y_dot, psi_dot, vx_dot, vy_dot, omega_dot])

    @property
    def state_names(self) -> list:
        return ["x", "y", "psi", "vx", "vy", "omega"]
