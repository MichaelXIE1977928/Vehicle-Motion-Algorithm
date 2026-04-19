"""Abstract base class for tire models."""
from abc import ABC, abstractmethod


class TireModel(ABC):
    """Base class for tire force computation."""

    @abstractmethod
    def lateral_force(self, slip_angle: float, normal_load: float) -> float:
        """Compute lateral force given slip angle and normal load."""
        pass
