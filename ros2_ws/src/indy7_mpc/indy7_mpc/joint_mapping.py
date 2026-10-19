"""Joint ordering, direction and zero-offset mapping for sim-to-real control."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml


def identity_mapping(joint_names: list[str]) -> dict[str, dict[str, Any]]:
    return {
        name: {"simulation_joint": name, "direction": 1, "offset_deg": 0.0}
        for name in joint_names
    }


def validate_mapping(
    mapping: dict[str, dict[str, Any]],
    simulation_joints: list[str],
    robot_joints: list[str],
) -> dict[str, dict[str, Any]]:
    if set(mapping) != set(robot_joints):
        raise ValueError("The mapping must contain all six robot joints exactly once.")
    if len(simulation_joints) != len(robot_joints):
        raise ValueError("Simulation and robot joint counts must match.")
    result: dict[str, dict[str, Any]] = {}
    used: list[str] = []
    for robot_joint in robot_joints:
        row = mapping[robot_joint]
        simulation_joint = str(row["simulation_joint"])
        direction_value = float(row["direction"])
        direction = int(direction_value) if direction_value in {-1.0, 1.0} else 0
        offset_deg = float(row["offset_deg"])
        if simulation_joint not in simulation_joints:
            raise ValueError(f"Unknown simulation joint: {simulation_joint}")
        if direction not in {-1, 1}:
            raise ValueError(f"{robot_joint}: direction must be +1 or -1.")
        if not math.isfinite(offset_deg) or abs(offset_deg) > 720.0:
            raise ValueError(f"{robot_joint}: offset must be a finite value between -720 and 720 degrees.")
        used.append(simulation_joint)
        result[robot_joint] = {
            "simulation_joint": simulation_joint,
            "direction": direction,
            "offset_deg": offset_deg,
        }
    if len(set(used)) != len(simulation_joints):
        raise ValueError("Each simulation joint must be mapped exactly once.")
    return result


def map_sim_to_robot(
    sim_positions: list[float],
    simulation_joints: list[str],
    robot_joints: list[str],
    mapping: dict[str, dict[str, Any]],
) -> list[float]:
    mapping = validate_mapping(mapping, simulation_joints, robot_joints)
    if len(sim_positions) != len(simulation_joints) or not all(
        math.isfinite(float(value)) for value in sim_positions
    ):
        raise ValueError("All six simulation joint values must be finite.")
    sim_by_name = dict(zip(simulation_joints, sim_positions))
    return [
        mapping[name]["direction"] * float(sim_by_name[mapping[name]["simulation_joint"]])
        + math.radians(mapping[name]["offset_deg"])
        for name in robot_joints
    ]


def map_robot_to_sim(
    robot_positions: list[float],
    simulation_joints: list[str],
    robot_joints: list[str],
    mapping: dict[str, dict[str, Any]],
) -> list[float]:
    mapping = validate_mapping(mapping, simulation_joints, robot_joints)
    if len(robot_positions) != len(robot_joints) or not all(
        math.isfinite(float(value)) for value in robot_positions
    ):
        raise ValueError("All six robot joint values must be finite.")
    robot_by_name = dict(zip(robot_joints, robot_positions))
    sim_by_name = {
        mapping[robot_name]["simulation_joint"]: mapping[robot_name]["direction"]
        * (float(robot_by_name[robot_name]) - math.radians(mapping[robot_name]["offset_deg"]))
        for robot_name in robot_joints
    }
    return [sim_by_name[name] for name in simulation_joints]


def load_mapping(path: str | Path, robot_joints: list[str], simulation_joints: list[str]):
    path = Path(path).expanduser()
    if not path.is_file():
        return identity_mapping(robot_joints)
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return validate_mapping(data.get("joint_mapping", {}), simulation_joints, robot_joints)


def save_mapping(path: str | Path, mapping: dict[str, dict[str, Any]]) -> None:
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump({"joint_mapping": mapping}, stream, sort_keys=False, allow_unicode=True)
    temporary.replace(path)
