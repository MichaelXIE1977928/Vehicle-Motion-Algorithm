"""Abstract base class for vehicle models."""
from abc import ABC, abstractmethod
import numpy as np


class VehicleModel(ABC):
    """Base class for all vehicle models."""

    def __init__(self, params: dict):
        self.params = params

    @abstractmethod
    def state_derivative(self, t: float, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        """Compute state derivative given current state and inputs."""
        pass

    @property
    @abstractmethod
    def state_names(self) -> list:
        """List of state variable names."""
        pass
