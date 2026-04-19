"""Factories for pluggable vehicle, tire, and solver components (registry pattern)."""
from typing import Tuple

from vma.core.vehicle.bicycle import BicycleModel
from vma.core.vehicle.kinematic import KinematicModel
from vma.core.vehicle.dof14 import DOF14Model
from vma.core.vehicle.double_track_planar import DoubleTrackPlanarModel
from vma.core.vehicle.base import VehicleModel
from vma.core.tire.linear import LinearTire
from vma.core.tire.fiala import FialaTire
from vma.core.tire.dugoff import DugoffTire
from vma.core.tire.magic_formula import MagicFormulaTire
from vma.core.tire.brush_combined import BrushCombinedTire
from vma.core.tire.base import TireModel
from vma.core.solver.scipy_ode import ScipyOdeSolver
from vma.core.solver.custom_rk4 import CustomRK4Solver
from vma.core.solver.base import Solver


VEHICLE_REGISTRY = {
    "bicycle": BicycleModel,
    "kinematic": KinematicModel,
    "dof14": DOF14Model,
    "double_track": DoubleTrackPlanarModel,
    "double_track_planar": DoubleTrackPlanarModel,
}

SCIPY_METHODS = ("RK45", "RK23", "DOP853", "Radau", "BDF", "LSODA")


def make_vehicle(kind: str, params: dict) -> VehicleModel:
    key = kind.lower().replace(" ", "_")
    if key not in VEHICLE_REGISTRY:
        raise ValueError(f"Unknown vehicle model: {kind}. Options: {list(VEHICLE_REGISTRY)}")
    return VEHICLE_REGISTRY[key](params)


def _magic_params_from_vehicle(params: dict) -> dict:
    if "magic_formula" in params and isinstance(params["magic_formula"], dict):
        return params["magic_formula"]
    return {
        "B": float(params.get("mf_B", 10.0)),
        "C": float(params.get("mf_C", 1.9)),
        "E": float(params.get("mf_E", -0.5)),
        "mu": float(params.get("friction_coeff", 0.9)),
    }


def _brush_combined_params_from_vehicle(params: dict) -> dict:
    if "brush_combined" in params and isinstance(params["brush_combined"], dict):
        return params["brush_combined"]
    if "magic_formula" in params and isinstance(params["magic_formula"], dict):
        base = dict(params["magic_formula"])
    else:
        base = {
            "B": float(params.get("mf_B", 10.0)),
            "C": float(params.get("mf_C", 1.9)),
            "E": float(params.get("mf_E", -0.5)),
            "mu": float(params.get("friction_coeff", 0.9)),
        }
    base.setdefault("combined_Bx", float(params.get("combined_Bx", 8.0)))
    base.setdefault("combined_Cx", float(params.get("combined_Cx", 1.1)))
    base.setdefault("combined_min_scale", float(params.get("combined_min_scale", 0.2)))
    return base


def make_tire_pair(kind: str, params: dict) -> Tuple[TireModel, TireModel]:
    """Build front/rear tires from vehicle parameter dict."""
    k = kind.lower().replace(" ", "_")
    Cf = float(params["front_cornering_stiffness"])
    Cr = float(params["rear_cornering_stiffness"])
    mu = float(params.get("friction_coeff", 0.9))

    if k == "linear":
        return LinearTire(Cf), LinearTire(Cr)
    if k == "fiala":
        return FialaTire(Cf, mu), FialaTire(Cr, mu)
    if k == "dugoff":
        return DugoffTire(Cf, mu), DugoffTire(Cr, mu)
    if k in ("magic_formula", "magic", "pacejka"):
        mp = _magic_params_from_vehicle(params)
        return MagicFormulaTire(mp), MagicFormulaTire(mp)
    if k in ("brush_combined", "combined_slip", "pacejka_combined", "brush"):
        bp = _brush_combined_params_from_vehicle(params)
        return BrushCombinedTire(bp), BrushCombinedTire(bp)
    raise ValueError(
        f"Unknown tire model: {kind}. Options: linear, fiala, dugoff, magic_formula, brush_combined"
    )


def make_solver(
    backend: str,
    scipy_method: str = "RK45",
    rk4_dt: float = 0.01,
) -> Solver:
    b = backend.lower().replace(" ", "_")
    if b in ("scipy", "scipy_ode", "solve_ivp"):
        m = scipy_method.upper()
        if m not in SCIPY_METHODS:
            raise ValueError(f"Unknown SciPy method: {scipy_method}. Options: {SCIPY_METHODS}")
        return ScipyOdeSolver(method=m)
    if b in ("rk4", "custom_rk4", "fixed_rk4"):
        return CustomRK4Solver(dt=float(rk4_dt))
    raise ValueError(f"Unknown solver backend: {backend}. Options: scipy_ode, custom_rk4")
