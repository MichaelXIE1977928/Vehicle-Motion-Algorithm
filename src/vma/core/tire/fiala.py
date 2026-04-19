"""Fiala brush-based lateral tire model (single-contact, lateral slip)."""
import numpy as np
from .base import TireModel


class FialaTire(TireModel):
    """Fiala non-linear lateral force (small brush model, saturation with friction limit)."""

    def __init__(self, cornering_stiffness: float, friction_coeff: float):
        self.cornering_stiffness = float(cornering_stiffness)
        self.mu = float(friction_coeff)

    def lateral_force(self, slip_angle: float, normal_load: float) -> float:
        Fz = max(float(normal_load), 1.0)
        C = self.cornering_stiffness
        mu = self.mu
        alpha = float(slip_angle)
        F_max = mu * Fz
        denom_c = 3.0 * mu * Fz + 1e-6
        denom_d = 27.0 * (mu * Fz) ** 2 + 1e-6
        fy = C * alpha - (C**2 * abs(alpha) * alpha) / denom_c + (C**3 * alpha**3) / denom_d
        return float(np.clip(fy, -F_max, F_max))
