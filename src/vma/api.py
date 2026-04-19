"""Internal API used by all frontends."""
from pathlib import Path
from typing import Any, Dict, Optional

from vma.core.maneuver.standard import Maneuver
from vma.core.simulation import run_simulation
from vma.core.factories import make_vehicle, make_tire_pair, make_solver
from vma.core.analytical.benchmarks import add_analytical_to_results
from vma.io.parameters import load_vehicle, load_maneuver
from vma.io.results import save_csv, plot_results


def run_simulation_from_files(
    vehicle_path: Path,
    maneuver_path: Path,
    vehicle_model: str = "bicycle",
    tire_model: str = "linear",
    solver_backend: str = "scipy_ode",
    scipy_method: str = "RK45",
    rk4_dt: float = 0.01,
    output_csv: Optional[Path] = None,
    show_plot: bool = False,
    include_analytical: bool = True,
) -> Dict[str, Any]:
    """
    Run a simulation using JSON files and selected model/solver stack.

    Args:
        vehicle_path: Path to vehicle JSON file.
        maneuver_path: Path to maneuver JSON file.
        vehicle_model: bicycle | kinematic | double_track | dof14
        tire_model: linear | fiala | dugoff | magic_formula | brush_combined
        solver_backend: scipy_ode | custom_rk4
        scipy_method: RK45 | RK23 | DOP853 | Radau | BDF | LSODA (scipy_ode only)
        rk4_dt: Fixed step for custom_rk4.
        output_csv: Optional path to save CSV results.
        show_plot: Whether to display a matplotlib plot (non-Streamlit).
        include_analytical: Attach command history, steady-state hints, and same-input kinematic reference.

    Returns:
        Dictionary containing time, states, state_names, and optionally analytical.
    """
    vehicle_params = load_vehicle(vehicle_path)
    maneuver_params = load_maneuver(maneuver_path)

    vehicle = make_vehicle(vehicle_model, vehicle_params)
    tire_front, tire_rear = make_tire_pair(tire_model, vehicle_params)
    maneuver = Maneuver(maneuver_params)
    solver = make_solver(solver_backend, scipy_method=scipy_method, rk4_dt=rk4_dt)

    results = run_simulation(vehicle, tire_front, tire_rear, maneuver, solver=solver)

    if include_analytical:
        results = add_analytical_to_results(
            results,
            vehicle_params,
            maneuver_params,
            solver_backend=solver_backend,
            scipy_method=scipy_method,
            rk4_dt=rk4_dt,
        )

    if output_csv:
        save_csv(results, output_csv)
    if show_plot:
        plot_results(results)

    return results
