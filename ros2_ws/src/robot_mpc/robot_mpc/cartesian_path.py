"""Cartesian segment geometry and the revolute-arm command boundary (metres)."""

import numpy as np


def segment_error(position, start, goal):
    """Return distance residual and its position Jacobian for a closed segment."""
    direction = np.asarray(goal) - start
    length_squared = float(direction @ direction)
    offset = np.asarray(position) - start
    if length_squared <= 1.0e-24:
        return offset, np.eye(3)
    progress = float(offset @ direction) / length_squared
    residual = offset - np.clip(progress, 0.0, 1.0) * direction
    projection = np.eye(3)
    if 0.0 < progress < 1.0:
        projection -= np.outer(direction, direction) / length_squared
    return residual, projection


def project_cartesian_command(command, lower, upper, tcp_position, tcp_jacobian,
                              start, goal, tolerance):
    """Move a clipped command toward the line without relaxing joint bounds."""
    command = command.copy()
    for _ in range(12):
        error, projection = segment_error(tcp_position(command), start, goal)
        error_norm = float(np.linalg.norm(error))
        if error_norm <= 0.5 * tolerance:
            break
        jacobian = projection @ tcp_jacobian(command)
        if not np.all(np.isfinite(error)) or not np.all(np.isfinite(jacobian)):
            break
        # Solve again on free joints when a position/rate bound blocks correction.
        free = np.ones(command.size, dtype=bool)
        for _ in range(command.size):
            active_jacobian = jacobian[:, free]
            correction = np.zeros(command.size)
            correction[free] = -active_jacobian.T @ np.linalg.solve(
                active_jacobian @ active_jacobian.T + 1e-8 * np.eye(3), error
            )
            blocked = ((command <= lower + 1e-12) & (correction < 0)) | (
                (command >= upper - 1e-12) & (correction > 0)
            )
            if not np.any(blocked & free):
                break
            free[blocked] = False
        for scale in (1.0, 0.5, 0.25, 0.125):
            candidate = np.clip(command + scale * correction, lower, upper)
            candidate_error = np.linalg.norm(segment_error(tcp_position(candidate), start, goal)[0])
            if candidate_error < error_norm:
                command = candidate
                break
        else:
            break
    return command


def limit_cartesian_step(measured_q, command, tcp_position, start, goal, tolerance, reach):
    """Backtrack until the entire joint-linear step fits inside the path tube.

    For a revolute chain, ||d²p/ds²|| <= reach * sum(abs(delta_q))².
    Between samples the TCP differs from their chord by at most M*h²/8.
    The tube around a segment is convex, so this also bounds unsampled poses.
    """
    def deviation(q):
        position = np.asarray(tcp_position(q), dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return float("inf")
        return float(np.linalg.norm(segment_error(position, start, goal)[0]))

    initial_deviation = deviation(measured_q)
    # An external tracking error cannot be undone by certifying a new step.
    if initial_deviation > tolerance:
        return measured_q.copy(), 0.0
    delta = command - measured_q
    for attempt in range(13):
        scale = 0.5 ** attempt
        step = scale * delta
        curvature_margin = reach * float(np.sum(np.abs(step))) ** 2 / (8.0 * 4**2)
        previous = initial_deviation
        for fraction in (0.25, 0.5, 0.75, 1.0):
            current = deviation(measured_q + fraction * step)
            if max(previous, current) + curvature_margin > tolerance:
                break
            previous = current
        else:
            return measured_q + step, scale
    return measured_q.copy(), 0.0
