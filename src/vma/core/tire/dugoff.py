"""Dugoff tire model (lateral slip, zero longitudinal slip)."""
import numpy as np
from .base import TireModel


class DugoffTire(TireModel):
    """Simplified Dugoff lateral force with kappa = 0."""

    def __init__(self, cornering_stiffness: float, friction_coeff: float):
        self.cornering_stiffness = float(cornering_stiffness)
        self.mu = float(friction_coeff)

    def lateral_force(self, slip_angle: float, normal_load: float) -> float:
        Fz = max(float(normal_load), 1.0)
        C = self.cornering_stiffness
        mu = self.mu
        alpha = float(slip_angle)
        tan_a = np.tan(alpha) if abs(alpha) < 1.4 else np.copysign(5.5, alpha)
        Fy_sl = C * tan_a
        theta = (mu * Fz * (1.0)) / (2.0 * max(abs(Fy_sl), 1e-6))
        if theta < 1.0:
            lam = theta * (2.0 - theta)
        else:
            lam = 1.0
        return float(np.clip(lam * Fy_sl, -mu * Fz, mu * Fz))
