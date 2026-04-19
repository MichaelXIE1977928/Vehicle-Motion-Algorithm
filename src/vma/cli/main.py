#!/usr/bin/env python3
"""Command-line interface for VMA."""
import argparse
import sys
from pathlib import Path

# Repo-local `src/` bootstrap (same tree as pyproject / setup package_dir).
_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from vma.api import run_simulation_from_files


def main():
    parser = argparse.ArgumentParser(description="VMA Vehicle Dynamics Simulator")
    parser.add_argument("--vehicle", "-v", required=True, help="Vehicle JSON file")
    parser.add_argument("--maneuver", "-m", required=True, help="Maneuver JSON file")
    parser.add_argument(
        "--vehicle-model",
        default="bicycle",
        help="bicycle | kinematic | double_track | dof14",
    )
    parser.add_argument(
        "--tire-model",
        default="linear",
        help="linear | fiala | dugoff | magic_formula | brush_combined",
    )
    parser.add_argument("--solver", default="scipy_ode", help="scipy_ode | custom_rk4")
    parser.add_argument(
        "--scipy-method",
        default="RK45",
        help="RK45 | RK23 | DOP853 | Radau | BDF | LSODA (scipy_ode only)",
    )
    parser.add_argument("--rk4-dt", type=float, default=0.01, help="Fixed step for custom_rk4")
    parser.add_argument("--output", "-o", help="Output CSV file")
    parser.add_argument("--plot", "-p", action="store_true", help="Show plot")

    args = parser.parse_args()

    vehicle_path = Path(args.vehicle)
    maneuver_path = Path(args.maneuver)

    if not vehicle_path.exists():
        print(f"Error: Vehicle file not found: {vehicle_path}")
        sys.exit(1)
    if not maneuver_path.exists():
        print(f"Error: Maneuver file not found: {maneuver_path}")
        sys.exit(1)

    try:
        results = run_simulation_from_files(
            vehicle_path,
            maneuver_path,
            vehicle_model=args.vehicle_model,
            tire_model=args.tire_model,
            solver_backend=args.solver,
            scipy_method=args.scipy_method,
            rk4_dt=args.rk4_dt,
            output_csv=Path(args.output) if args.output else None,
            show_plot=args.plot,
        )
        print(f"Simulation complete. Final time: {results['time'][-1]:.2f} s")
        if args.output:
            print(f"Results saved to: {args.output}")
    except Exception as e:
        print(f"Error during simulation: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
