"""Pure validation helpers for the Indy7 DCP3 command boundary."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np


DOF = 6


def percent_ratio(value: Any, name: str) -> float:
    """Validate a DCP3 motion ratio expressed on the 0-100 scale."""
    try:
        ratio = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number from 0 to 100") from exc
    if not math.isfinite(ratio) or not 0.0 <= ratio <= 100.0:
        raise ValueError(f"{name} must be a finite number from 0 to 100")
    return ratio


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
    if not np.all(np.isfinite(np.r_[lower, upper])):
        raise ValueError("URDF joint limits must be finite")
    return lower, upper


def validate_joint_target(
    current: Any,
    requested: Any,
    lower: Any,
    upper: Any,
    margin: float,
) -> np.ndarray:
    """Validate joint positions and forward the requested target unchanged."""
    current = six(current, "current")
    requested = six(requested, "requested")
    lower = six(lower, "lower")
    upper = six(upper, "upper")
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("margin must be non-negative and finite")
    if np.any(lower + margin >= upper - margin):
        raise ValueError("invalid joint-limit margin")
    if np.any(current < lower) or np.any(current > upper):
        raise ValueError("measured joint position is outside URDF limits")
    if np.any(requested < lower + margin) or np.any(requested > upper - margin):
        raise ValueError("requested position crosses the soft joint limits")
    return requested.copy()
