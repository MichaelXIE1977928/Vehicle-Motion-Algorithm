"""Linear tire model."""
from .base import TireModel


class LinearTire(TireModel):
    def __init__(self, cornering_stiffness: float):
        self.cornering_stiffness = cornering_stiffness

    def lateral_force(self, slip_angle: float, normal_load: float) -> float:
        return self.cornering_stiffness * slip_angle
