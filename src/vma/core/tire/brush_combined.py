"""Brush/Pacejka-combined style lateral tire with longitudinal utilization scaling."""
import numpy as np

from .base import TireModel


class BrushCombinedTire(TireModel):
    """
    Lateral Magic-Formula core with a combined-slip reduction factor.

    The combined term uses a common proxy form:
        G(kappa) = max(min_scale, cos(Cx * atan(Bx * kappa)))
    and scales the pure lateral response:
        Fy = G(kappa) * Fy_pure(alpha)
    """

    def __init__(self, params: dict):
        self.params = dict(params)

    def _fy_pure(self, slip_angle: float, normal_load: float) -> float:
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

    def combined_scale(self, kappa_proxy: float) -> float:
        p = self.params
        Bx = float(p.get("combined_Bx", 8.0))
        Cx = float(p.get("combined_Cx", 1.1))
        min_scale = float(p.get("combined_min_scale", 0.2))
        kappa = abs(float(kappa_proxy))
        g = np.cos(Cx * np.arctan(Bx * kappa))
        return float(np.clip(g, min_scale, 1.0))

    def lateral_force_combined(self, slip_angle: float, normal_load: float, kappa_proxy: float) -> float:
        return self.combined_scale(kappa_proxy) * self._fy_pure(slip_angle, normal_load)

    def lateral_force(self, slip_angle: float, normal_load: float) -> float:
        # Backward-compatible pure-slip path.
        return self._fy_pure(slip_angle, normal_load)
