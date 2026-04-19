"""Double-track (four-wheel) planar vehicle with roll — rigid body, slip at each corner."""
import numpy as np

from .base import VehicleModel


class DoubleTrackPlanarModel(VehicleModel):
    """
    Planar dynamics at CG with four independent tire slips (front/rear track).

    Same roll state as the bicycle plant (`phi`, `phi_dot`). Lateral/longitudinal
    equations are assembled in `simulation.run_simulation` using one front and
    one rear tire *type* (stiffness / MF params) applied per corner with corner
    normal loads from static split plus lateral load transfer.

    Inputs (via simulation): [steer at front axle, Fx body, mu_scale].
    """

    def __init__(self, params: dict):
        super().__init__(params)
        self.mass = float(params["mass"])
        self.Iz = float(params["yaw_inertia"])
        self.lf = float(params["cg_to_front"])
        self.lr = float(params["cg_to_rear"])
        self.Cf = float(params["front_cornering_stiffness"])
        self.Cr = float(params["rear_cornering_stiffness"])
        self.sprung_mass = float(params.get("sprung_mass", 0.9 * self.mass))
        self.roll_inertia = float(params.get("roll_inertia", 650.0))
        self.cg_height = float(params.get("cg_height", 0.55))
        self.front_roll_stiffness = float(params.get("front_roll_stiffness", 28000.0))
        self.rear_roll_stiffness = float(params.get("rear_roll_stiffness", 25000.0))
        self.front_roll_damping = float(params.get("front_roll_damping", 2200.0))
        self.rear_roll_damping = float(params.get("rear_roll_damping", 2000.0))
        self.track_width = float(params.get("track_width", 1.58))
        self.load_transfer_coupling = float(params.get("load_transfer_coupling", 0.08))
        self.roll_k = self.front_roll_stiffness + self.rear_roll_stiffness
        self.roll_c = self.front_roll_damping + self.rear_roll_damping

    def state_derivative(self, t: float, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        raise RuntimeError("DoubleTrackPlanarModel dynamics are integrated in simulation.run_simulation.")

    @property
    def state_names(self) -> list:
        return ["x", "y", "psi", "vx", "vy", "omega", "phi", "phi_dot"]
