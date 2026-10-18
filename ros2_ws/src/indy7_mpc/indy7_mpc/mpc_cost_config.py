"""Load and apply the MPC cost settings that can change during a run."""

from __future__ import annotations

import math
from pathlib import Path

import yaml


WEIGHT_PARAMETERS = {
    "se3_position_cost_weight": "se3_position",
    "se3_orientation_cost_weight": "se3_orientation",
    "posture_cost_weight": "posture",
    "velocity_cost_weight": "velocity",
    "torque_cost_weight": "torque",
    "joint_limit_cost_weight": "joint_limit",
}


def load_runtime_cost_settings(path: str | Path) -> dict:
    """Read and validate cost weights and the preferred joint posture."""
    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError("YAML root must be a mapping")
    node_config = document.get("crocoddyl_mpc")
    if not isinstance(node_config, dict):
        raise ValueError("missing crocoddyl_mpc mapping")
    parameters = node_config.get("ros__parameters")
    if not isinstance(parameters, dict):
        raise ValueError("missing crocoddyl_mpc.ros__parameters mapping")

    weights = {}
    for parameter_name, action_name in WEIGHT_PARAMETERS.items():
        try:
            value = float(parameters[parameter_name])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{parameter_name} must be a positive number") from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{parameter_name} must be a positive finite number")
        weights[action_name] = value

    posture = parameters.get("nominal_posture")
    if not isinstance(posture, (tuple, list)) or len(posture) != 6:
        raise ValueError("nominal_posture must contain six finite joint angles in radians")
    nominal_posture = []
    for value in posture:
        try:
            parsed = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("nominal_posture must contain six finite joint angles in radians") from exc
        if not math.isfinite(parsed):
            raise ValueError("nominal_posture must contain six finite joint angles in radians")
        nominal_posture.append(parsed)

    scales = {}
    for name in ("running_cost_weight", "terminal_cost_weight"):
        try:
            value = float(parameters[name])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a positive number") from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a positive finite number")
        scales[name] = value

    return {
        "weights": weights,
        "nominal_posture": nominal_posture,
        **scales,
    }


def apply_runtime_cost_settings(running_actions, terminal_action, settings: dict) -> None:
    """Update action-model costs in place so the existing solver sees them."""
    for action in running_actions:
        action.weights.update(settings["weights"])
        action.nominal_posture = list(settings["nominal_posture"])
        action.terminal_scale = float(settings["running_cost_weight"])
    terminal_action.weights.update(settings["weights"])
    terminal_action.nominal_posture = list(settings["nominal_posture"])
    terminal_action.terminal_scale = float(settings["terminal_cost_weight"])
