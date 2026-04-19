"""Fixed-step Runge–Kutta 4th order (for HIL / deterministic stepping)."""
import numpy as np
from .base import Solver


class CustomRK4Solver(Solver):
    def __init__(self, dt: float = 0.01):
        self.dt = float(dt)

    def integrate(self, func, t_span, y0, args=(), **kwargs):
        t0, tf = float(t_span[0]), float(t_span[1])
        y0 = np.asarray(y0, dtype=float)
        dt = self.dt
        t_list = [t0]
        y_list = [y0.copy()]
        t = t0
        y = y0.copy()
        while t < tf - 1e-15:
            h = min(dt, tf - t)
            k1 = np.asarray(func(t, y), dtype=float)
            k2 = np.asarray(func(t + 0.5 * h, y + 0.5 * h * k1), dtype=float)
            k3 = np.asarray(func(t + 0.5 * h, y + 0.5 * h * k2), dtype=float)
            k4 = np.asarray(func(t + h, y + h * k3), dtype=float)
            y = y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            t = t + h
            t_list.append(t)
            y_list.append(y.copy())
        return np.array(t_list), np.vstack(y_list)
