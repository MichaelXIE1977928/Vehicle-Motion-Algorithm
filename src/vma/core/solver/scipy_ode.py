"""Wrapper for scipy.integrate.solve_ivp."""
import numpy as np
from scipy.integrate import solve_ivp
from .base import Solver


class ScipyOdeSolver(Solver):
    def __init__(self, method='RK45'):
        self.method = method

    def integrate(self, func, t_span, y0, args=(), **kwargs):
        sol = solve_ivp(func, t_span, y0, args=args, method=self.method, **kwargs)
        return sol.t, sol.y.T
