"""16-state vehicle: 8-state planar+roll (four-wheel / double-track planar forces) + 4 unsprung vertical + 4 vertical rates."""
import numpy as np
from .base import VehicleModel


class DOF14Model(VehicleModel):
    """
    Planar+roll dynamics use the same **four-wheel (double-track)** slip and lateral-force resolution as
    `DoubleTrackPlanarModel` in `simulation.run_simulation`, plus **decoupled** linear vertical modes per corner.

    State vector (16): [x, y, psi, vx, vy, omega, phi, phi_dot, zu_fl, zu_fr, zu_rl, zu_rr, wz_fl, wz_fr, wz_rl, wz_rr]

    Naming note: this is still a **16-state** integration vector in code. The vertical channels are a scaffold
    toward a full multi-body / CarSim-style stack; they do **not** yet feed back into per-corner tire normal
    loads used for the planar tire forces (those still use the static axle split + 50/50 left-right model).
    """

    def __init__(self, params: dict):
        super().__init__(params)
        self.mass = float(params["mass"])
        self.Iz = float(params["yaw_inertia"])
        self.lf = float(params["cg_to_front"])
        self.lr = float(params["cg_to_rear"])
        self.sprung_mass = float(params.get("sprung_mass", 0.9 * self.mass))
        self.roll_inertia = float(params.get("roll_inertia", 700.0))
        self.cg_height = float(params.get("cg_height", 0.58))
        self.front_roll_stiffness = float(params.get("front_roll_stiffness", 30000.0))
        self.rear_roll_stiffness = float(params.get("rear_roll_stiffness", 27000.0))
        self.front_roll_damping = float(params.get("front_roll_damping", 2400.0))
        self.rear_roll_damping = float(params.get("rear_roll_damping", 2200.0))
        self.track_width = float(params.get("track_width", 1.6))
        self.load_transfer_coupling = float(params.get("load_transfer_coupling", 0.1))
        self.roll_k = self.front_roll_stiffness + self.rear_roll_stiffness
        self.roll_c = self.front_roll_damping + self.rear_roll_damping
        wn = float(params.get("vert_natural_freq_hz", 2.0)) * (2.0 * np.pi)
        zeta = float(params.get("vert_damping_ratio", 0.25))
        self._wn2 = wn**2
        self._2zwn = 2.0 * zeta * wn

    def state_derivative(self, t: float, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        """Not used directly — planar part is integrated in simulation with tire forces."""
        raise RuntimeError("DOF14Model uses simulation.run_simulation planar+tire coupling.")

    @property
    def state_names(self) -> list:
        return [
            "x", "y", "psi", "vx", "vy", "omega", "phi", "phi_dot",
            "zu_fl", "zu_fr", "zu_rl", "zu_rr",
            "wz_fl", "wz_fr", "wz_rl", "wz_rr",
        ]
