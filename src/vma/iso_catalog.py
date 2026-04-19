"""
ISO-oriented working cases for Dash-Board and Analytical-Information routing.

This catalog maps Streamlit / API selections to maneuver JSON files and user-facing text.
Commonly cited ISO vehicle-dynamics families (non-exhaustive of all ISO road-vehicle standards):
3888 (severe lane change / obstacle avoidance / lane-change-with-brake class tracks), 4138 (steady-state circular),
7401 (lateral transient open-loop), 7975 (braking in a turn), Ackermann slow slalom (linear-range),
throttle-on in turn (understeer push), split-friction (μ-split class),
17288 (low-speed maneuverability / U-turn class),
fish-hook / sine-with-dwell (stability / consumer-ESC-class open-loop proxies),
straight-line braking and ramp-step steer (longitudinal / transient extensions),
15037 (general test conditions — informational).
"""
from pathlib import Path
from typing import Any, Dict, List

# id -> stable key for session_state and docs; maneuver_stem matches data/maneuvers/*.json (no extension)
# GUI list order: ascending ISO family (3888 → 4138 → 7401 block → 7975 → 17288), then non-numbered proxies.
WORKING_CASES: List[Dict[str, Any]] = [
    {
        "id": "iso3888_1",
        "gui_label": "ISO 3888-1 — Double lane change",
        "maneuver_stem": "iso3888_1_double_lane_change",
        "intro": (
            "Severe double lane-change style maneuver (track layout defined in ISO 3888-1). "
            "Used to probe limit handling and stability during rapid lateral transitions."
        ),
    },
    {
        "id": "iso3888_2",
        "gui_label": "ISO 3888-2 — Obstacle avoidance (severe lane change)",
        "maneuver_stem": "iso3888_2_obstacle_avoidance",
        "intro": (
            "Obstacle-avoidance / severe lane-change family per ISO 3888-2 (track geometry in standard). "
            "Open-loop profile here is a representative asymmetric steer sequence, not a full cone layout."
        ),
    },
    {
        "id": "iso3888_3",
        "gui_label": "ISO 3888-3-class — Lane change with braking (open-loop proxy)",
        "maneuver_stem": "iso3888_3_lane_change_brake",
        "intro": (
            "Double lane-change style steering with a commanded longitudinal deceleration after a set time. "
            "Open-loop proxy for combined lateral–longitudinal regimes in the ISO 3888-3 family; not a full track layout."
        ),
    },
    {
        "id": "iso4138",
        "gui_label": "ISO 4138 — Constant-radius (steady-state circular)",
        "maneuver_stem": "iso4138_constant_radius",
        "intro": (
            "Open-loop approximately steady circular path: constant steer for a fixed radius. "
            "Aligns with steady-state circular driving behaviour tests in ISO 4138."
        ),
    },
    {
        "id": "iso7401_step",
        "gui_label": "ISO 7401 — Step steer (lateral transient)",
        "maneuver_stem": "iso7401_step_steer",
        "intro": (
            "Step change in steer angle; core open-loop transient from ISO 7401-style lateral response testing."
        ),
    },
    {
        "id": "iso7401_sine",
        "gui_label": "ISO 7401 — Sinusoidal / swept steer",
        "maneuver_stem": "iso7401_sine_sweep",
        "intro": (
            "Sinusoidal or frequency-swept steering input for gain/phase style studies under ISO 7401 open-loop methods."
        ),
    },
    {
        "id": "iso7401_impulse",
        "gui_label": "ISO 7401 — Impulse steer",
        "maneuver_stem": "iso7401_impulse_steer",
        "intro": (
            "Short-duration steer pulse to excite yaw/sideslip modes (ISO 7401 transient family)."
        ),
    },
    {
        "id": "iso7401_ramp_step",
        "gui_label": "ISO 7401 — Ramp–hold–return steer (open-loop proxy)",
        "maneuver_stem": "iso7401_ramp_step_steer",
        "intro": (
            "Linear steer ramp, hold at peak, then ramp back to neutral (trapezoidal steer envelope). "
            "Useful for repeatable transient gain/phase studies in the ISO 7401 family."
        ),
    },
    {
        "id": "fish_hook",
        "gui_label": "ISO 7401 — Fish hook (J-turn / limit stability, open-loop proxy)",
        "maneuver_stem": "fish_hook_consumer_class",
        "intro": (
            "Rapid steer-in, hold, then larger opposite steer (fish-hook / J-turn style excitation). "
            "Open-loop timing proxy for limit handling and stability studies — not a certified regulatory replay."
        ),
    },
    {
        "id": "sine_with_dwell",
        "gui_label": "ISO 7401 — Sine-with-dwell (ESC-type excitation, open-loop proxy)",
        "maneuver_stem": "sine_with_dwell_esc_class",
        "intro": (
            "Quarter-sine to peak steering, dwell at peak, then return toward neutral. "
            "Qualitatively similar to sine-with-dwell / consumer ESC-type excitations; timing is JSON-tunable, not FMVSS-identical."
        ),
    },
    {
        "id": "iso7975",
        "gui_label": "ISO 7975 — Braking in a turn",
        "maneuver_stem": "iso7975_braking_in_turn",
        "intro": (
            "Combines near steady-turn steering with a commanded longitudinal decel to study combined braking/cornering."
        ),
    },
    {
        "id": "iso17288",
        "gui_label": "ISO 17288-class — U-turn / large steer",
        "maneuver_stem": "iso17288_uturn",
        "intro": (
            "Large, sustained steer ramp representative of low-speed U-turn / maneuverability regimes (ISO 17288 family)."
        ),
    },
    {
        "id": "ackermann_slalom_linrange",
        "gui_label": "ISO-class — Ackermann slow slalom — low-amplitude sinusoid (linear-range / understeer gradient)",
        "maneuver_stem": "ackermann_slow_slalom_linrange",
        "intro": (
            "Constant-radius Ackermann-equivalent mean steer with a slow, small-amplitude steering sinusoid. "
            "Open-loop proxy for understeer-gradient and linear tire-range checks without large slip angles."
        ),
    },
    {
        "id": "throttle_on_in_turn",
        "gui_label": "ISO-class — Throttle-on in turn — constant steer + Fx drive schedule (understeer push)",
        "maneuver_stem": "throttle_on_in_turn_understeer",
        "intro": (
            "Holds ISO-7975-style constant-radius steering while applying a step-onset positive longitudinal "
            "force (drive torque proxy) to excite combined lateral–longitudinal understeer push; Fx schedule mirrors the braking case."
        ),
    },
    {
        "id": "straight_line_brake",
        "gui_label": "ISO-class — Straight-line braking (constant Fx decel, zero steer, open-loop proxy)",
        "maneuver_stem": "iso_straight_line_brake",
        "intro": (
            "Zero steering with a step-onset constant commanded longitudinal deceleration (body-fixed Fx). "
            "Baseline longitudinal / brake validation proxy; not tied to a single numbered ISO track file."
        ),
    },
    {
        "id": "split_friction",
        "gui_label": "ISO-class — μ-split — Split friction (chassis / ABS validation class)",
        "maneuver_stem": "iso_split_friction",
        "intro": (
            "Open-loop proxy for unequal left/right friction: after a set time, tire μ / cornering stiffness is scaled down. "
            "Used in many OEM and regulatory brake-stability scenarios; not a one-to-one replacement for a specific ISO track file."
        ),
    },
]


def maneuver_path_for_id(case_id: str, maneuver_dir: Path) -> Path:
    for c in WORKING_CASES:
        if c["id"] == case_id:
            return maneuver_dir / f"{c['maneuver_stem']}.json"
    raise KeyError(case_id)


def case_by_id(case_id: str) -> Dict[str, Any]:
    for c in WORKING_CASES:
        if c["id"] == case_id:
            return c
    raise KeyError(case_id)
