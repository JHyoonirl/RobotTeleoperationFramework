"""Pure validation helpers for the Indy7 DCP3 command boundary."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np


DOF = 6


def six(values: Any, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (DOF,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values")
    return result


def load_joint_limits(urdf_path: str | Path, joint_names: list[str]):
    root = ET.parse(urdf_path).getroot()
    limits = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if joint.get("name") in joint_names and limit is not None:
            limits[joint.get("name")] = limit
    missing = [name for name in joint_names if name not in limits]
    if missing:
        raise ValueError(f"URDF is missing limits for: {', '.join(missing)}")
    lower = np.array([float(limits[name].get("lower")) for name in joint_names])
    upper = np.array([float(limits[name].get("upper")) for name in joint_names])
    velocity = np.array([float(limits[name].get("velocity")) for name in joint_names])
    if not np.all(np.isfinite(np.r_[lower, upper, velocity])):
        raise ValueError("URDF joint limits must be finite")
    return lower, upper, velocity


def limit_joint_target(
    current: Any,
    requested: Any,
    lower: Any,
    upper: Any,
    max_velocity: Any,
    period: float,
    margin: float,
) -> np.ndarray:
    """Reject out-of-range targets and cap each update by a joint speed limit."""
    current = six(current, "current")
    requested = six(requested, "requested")
    lower = six(lower, "lower")
    upper = six(upper, "upper")
    max_velocity = six(max_velocity, "max_velocity")
    if not math.isfinite(period) or period <= 0.0:
        raise ValueError("period must be positive and finite")
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("margin must be non-negative and finite")
    if np.any(max_velocity <= 0.0) or np.any(lower + margin >= upper - margin):
        raise ValueError("invalid velocity limit or joint-limit margin")
    if np.any(current < lower) or np.any(current > upper):
        raise ValueError("measured joint position is outside URDF limits")
    if np.any(requested < lower + margin) or np.any(requested > upper - margin):
        raise ValueError("requested position crosses the soft joint limits")
    delta = np.clip(requested - current, -max_velocity * period, max_velocity * period)
    return current + delta
