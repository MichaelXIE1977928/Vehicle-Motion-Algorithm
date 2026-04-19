"""Abstract solver interface."""
from abc import ABC, abstractmethod


class Solver(ABC):
    @abstractmethod
    def integrate(self, func, t_span, y0, args=(), **kwargs):
        """Integrate ODE."""
        pass
