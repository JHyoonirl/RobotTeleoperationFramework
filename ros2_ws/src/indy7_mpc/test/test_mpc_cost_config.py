"""Tests for live-reloaded MPC objective settings."""

from types import SimpleNamespace

import pytest
import yaml

from indy7_mpc.mpc_cost_config import (
    apply_runtime_cost_settings,
    load_runtime_cost_settings,
)


def _write_config(path, **updates):
    parameters = {
        "se3_position_cost_weight": 12000.0,
        "se3_orientation_cost_weight": 450.0,
        "posture_cost_weight": 0.05,
        "velocity_cost_weight": 0.5,
        "torque_cost_weight": 0.0001,
        "joint_limit_cost_weight": 10.0,
        "running_cost_weight": 1.0,
        "terminal_cost_weight": 10.0,
        "nominal_posture": [0.0, 0.0, -1.57, 0.0, -1.57, 0.0],
    }
    parameters.update(updates)
    path.write_text(
        yaml.safe_dump({"crocoddyl_mpc": {"ros__parameters": parameters}}),
        encoding="utf-8",
    )


def test_load_runtime_cost_settings_reads_weights_and_posture(tmp_path):
    config = tmp_path / "mpc.yaml"
    _write_config(config, posture_cost_weight=0.2, nominal_posture=[0, 1, 2, 3, 4, 5])

    settings = load_runtime_cost_settings(config)

    assert settings["weights"]["posture"] == 0.2
    assert settings["nominal_posture"] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert settings["running_cost_weight"] == 1.0
    assert settings["terminal_cost_weight"] == 10.0


def test_invalid_weight_is_rejected_without_settings(tmp_path):
    config = tmp_path / "mpc.yaml"
    _write_config(config, posture_cost_weight=float("nan"))

    with pytest.raises(ValueError, match="posture_cost_weight"):
        load_runtime_cost_settings(config)


def test_runtime_settings_update_existing_action_models():
    running = [
        SimpleNamespace(weights={"posture": 0.05}, nominal_posture=[], terminal_scale=1.0)
        for _ in range(2)
    ]
    terminal = SimpleNamespace(
        weights={"posture": 0.05}, nominal_posture=[], terminal_scale=10.0
    )
    settings = {
        "weights": {"posture": 0.25},
        "nominal_posture": [0, 0, -1, 0, -1, 0],
        "running_cost_weight": 2.0,
        "terminal_cost_weight": 20.0,
    }

    apply_runtime_cost_settings(running, terminal, settings)

    assert all(action.weights["posture"] == 0.25 for action in running)
    assert all(action.terminal_scale == 2.0 for action in running)
    assert terminal.weights["posture"] == 0.25
    assert terminal.terminal_scale == 20.0
    assert terminal.nominal_posture == settings["nominal_posture"]
