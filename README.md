# VMA — Vehicle Motion Architecture

Vehicle dynamics simulation in Python: pluggable **vehicle** and **tire** models, **maneuvers** (JSON), **ODE solvers**, optional **analytical** benchmarks, a **Streamlit** GUI, and a **`vma`** CLI.

**Author / copyright:** Michael XZC (see [`Introduction.md`](Introduction.md) for ownership and attribution notes.)

**License:** [MIT](LICENSE)

## Requirements

- Python **3.11+**
- Dependencies: NumPy, SciPy, Matplotlib, Streamlit (see [`pyproject.toml`](pyproject.toml))

## Install

From the repository root:

```bash
python -m pip install --upgrade pip
pip install -e .
```

Development install (includes pytest):

```bash
pip install -e ".[dev]"
```

## Run

**Streamlit GUI** (**Dash-Board** home → **Analytical-Information** page):

```bash
streamlit run src/vma/gui/app.py
```

Or:

```bash
python scripts/run_gui.py
```

**CLI** (after install):

```bash
vma --vehicle data/vehicles/vehicle_case_sedan.json --maneuver data/maneuvers/step_steer.json --plot
```

Adjust paths to your JSON files. Use `vma --help` for model and solver options.

## Tests

```bash
pytest
```

## Repository layout

| Path | Purpose |
|------|---------|
| `src/vma/` | Package source (`api`, `core`, `gui`, `io`, …) |
| `data/vehicles/`, `data/maneuvers/` | JSON parameters |
| `tests/` | Pytest suite |
| `Introduction.md` | Extended intro, ownership, and legal summary |

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Security

See [`SECURITY.md`](SECURITY.md).
