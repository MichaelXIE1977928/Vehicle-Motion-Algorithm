"""DAE solver for future use."""
# Placeholder
from .base import Solver


class DaeSolver(Solver):
    def integrate(self, func, t_span, y0, args=(), **kwargs):
        raise NotImplementedError
