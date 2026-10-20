"""SE(3) helpers for mapping a Vive hand pose into the Indy7 world.

T_A_B maps coordinates in B into A. Poses use metres and xyzw quaternions.
"""

import math

import numpy as np


def transform_from_pose(position, quaternion_xyzw):
    position = np.asarray(position, dtype=float)
    q = np.asarray(quaternion_xyzw, dtype=float)
    if position.shape != (3,) or q.shape != (4,):
        raise ValueError("Expected a 3D position and an xyzw quaternion")
    if not np.isfinite(position).all() or not np.isfinite(q).all():
        raise ValueError("Pose must contain only finite values")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("Pose quaternion must not be zero")
    x, y, z, w = q / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    result[:3, 3] = position
    return result


def transform_from_rpy(position, rpy_degrees):
    rpy = np.asarray(rpy_degrees, dtype=float)
    if rpy.shape != (3,) or not np.isfinite(rpy).all():
        raise ValueError("Expected three finite roll/pitch/yaw angles in degrees")
    roll, pitch, yaw = np.deg2rad(rpy) / 2
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return transform_from_pose(position, [
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    ])


def pose_from_transform(transform):
    rotation = transform[:3, :3]
    trace = np.trace(rotation)
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        q = [(rotation[2, 1] - rotation[1, 2]) / s,
             (rotation[0, 2] - rotation[2, 0]) / s,
             (rotation[1, 0] - rotation[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + rotation[i, i] - rotation[j, j] - rotation[k, k]) * 2
        q = np.zeros(4)
        q[i] = s / 4
        q[j] = (rotation[i, j] + rotation[j, i]) / s
        q[k] = (rotation[i, k] + rotation[k, i]) / s
        q[3] = (rotation[k, j] - rotation[j, k]) / s
    q = np.asarray(q) / np.linalg.norm(q)
    return np.asarray(transform[:3, 3]).copy(), q


def inverse_transform(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def align_worlds(world_from_hand, vive_from_hand):
    """Align the current Vive hand pose with the current robot hand TCP."""
    return world_from_hand @ inverse_transform(vive_from_hand)
