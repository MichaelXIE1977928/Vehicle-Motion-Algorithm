#!/usr/bin/env python3
"""Launch the Streamlit GUI (paths are relative to this repo root only)."""
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
app_path = _REPO_ROOT / "src" / "vma" / "gui" / "app.py"
if not app_path.is_file():
    raise FileNotFoundError(f"Expected GUI at {app_path} — run from the VMA_generated_full tree.")
subprocess.run([sys.executable, "-m", "streamlit", "run", str(app_path)], check=False)
