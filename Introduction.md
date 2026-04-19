# VMA — Introduction

**VMA** (Vehicle Motion Architecture; also described in related materials as Vehicle Motion Algorithm) is a **vehicle dynamics simulation** project: a Python package for open-loop maneuvers, multiple vehicle and tire models, time integration, optional analytical benchmarks, a **Streamlit** configuration and analysis GUI, and a **`vma`** command-line entry point.

---

## Ownership, attribution, and protection

**Copyright © 2026 Michael XZC.** Michael XZC is the **named author and owner** of this VMA distribution for copyright and package metadata purposes (see `pyproject.toml`, `[project].authors`).

To **protect the project’s integrity and Michael XZC’s rights** in normal open-source practice:

- **Preserve notices**: Anyone who copies or redistributes this work should keep **copyright and license notices** visible in source or documentation, as required by the **MIT License** under which the project is declared.
- **No implied transfer**: Use of this repository does **not** transfer ownership of the **VMA** name, documentation, or code beyond what the license explicitly grants.
- **No extra warranty**: The software is provided **as is**; liability limitations follow the MIT License.

This introduction is part of the project documentation and should be **retained or reasonably summarized** in derivative distributions if you ship a substantial copy of the tree.

**License (summary):** The project is under the **MIT License**. The full text is in the repository root [`LICENSE`](LICENSE) file; `pyproject.toml` also declares `license = { text = "MIT" }`.

---

## Technical snapshot

| Item | Detail |
|------|--------|
| Package name | `vma` |
| Layout | `src/vma/` (installable package) |
| Python | ≥ 3.11 (`pyproject.toml`) |
| Main dependencies | NumPy, SciPy, Matplotlib, Streamlit |
| Internal API | `vma.api.run_simulation_from_files` (used by CLI and GUI flows) |
| Data | JSON vehicles under `data/vehicles/`, maneuvers under `data/maneuvers/` |

---

## Quick start (from this repository root)

**Streamlit GUI** (**Dash-Board** home and **Analytical-Information** analysis page):

```bash
streamlit run src/vma/gui/app.py
```

**CLI** (after installing the package, e.g. editable install from repo root):

```bash
pip install -e .
vma --vehicle path/to/vehicle.json --maneuver path/to/maneuver.json
```

**Helper script** (runs the same Streamlit app):

```bash
python scripts/run_gui.py
```

---

## Relationship to other docs

- **`pyproject.toml`** — build system, project name, version, dependencies, `vma` console script, authors, and license declaration.
- **`README.md`** — if present at repo root, may duplicate short run instructions; **this file** is the **introductory and ownership-oriented** reference for VMA in this tree.
