"""Load vehicle and maneuver JSON files."""
import json
from pathlib import Path


def load_vehicle(path: Path) -> dict:
    with open(path, 'r') as f:
        return json.load(f)


def load_maneuver(path: Path) -> dict:
    with open(path, 'r') as f:
        return json.load(f)
