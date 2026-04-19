"""Pacejka Magic Formula lateral force (simplified parameter set)."""
import numpy as np
from .base import TireModel


class MagicFormulaTire(TireModel):
    """Lateral Magic Formula: Fy = D sin(C atan(B*alpha - E (B*alpha - atan(B*alpha))))."""

    def __init__(self, params: dict):
        self.params = dict(params)

    def lateral_force(self, slip_angle: float, normal_load: float) -> float:
        Fz = max(float(normal_load), 1.0)
        p = self.params
        B = float(p.get("B", 10.0))
        C = float(p.get("C", 1.9))
        E = float(p.get("E", -0.5))
        mu = float(p.get("mu", 0.85))
        D = mu * Fz
        alpha = float(slip_angle)
        phi = B * alpha
        return float(D * np.sin(C * np.arctan(phi - E * (phi - np.arctan(phi)))))
