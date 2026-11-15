"""Cartesian segment geometry and the revolute-arm command boundary (metres)."""

import numpy as np
from scipy.optimize import lsq_linear


def smooth_path_progress(alpha, ramp_fraction):
    """Integrate cubic speed ramps around a constant-speed middle section."""
    alpha = float(np.clip(alpha, 0.0, 1.0))
    ramp = float(np.clip(ramp_fraction, 1.0e-12, 0.5))
    if alpha < ramp:
        u = alpha / ramp
        return ramp * (u**3 - 0.5 * u**4) / (1.0 - ramp)
    if alpha > 1.0 - ramp:
        u = (1.0 - alpha) / ramp
        return 1.0 - ramp * (u**3 - 0.5 * u**4) / (1.0 - ramp)
    return (alpha - 0.5 * ramp) / (1.0 - ramp)


def segment_error(position, start, goal):
    """Return distance residual and its position Jacobian for a closed segment."""
    direction = np.asarray(goal) - start
    length_squared = float(direction @ direction)
    offset = np.asarray(position) - start
    if length_squared <= 1.0e-24:
        return offset, np.eye(3)
    progress = float(offset @ direction) / length_squared
    residual = offset - min(1.0, max(0.0, progress)) * direction
    projection = np.eye(3)
    if 0.0 < progress < 1.0:
        projection -= np.outer(direction, direction) / length_squared
    return residual, projection


def project_cartesian_command(command, lower, upper, tcp_position, tcp_jacobian,
                              start, goal, tolerance):
    """Move a clipped command toward the line without relaxing joint bounds."""
    return project_cartesian_tasks(
        command, lower, upper, [(tcp_position, tcp_jacobian, start, goal)], tolerance
    )


def project_cartesian_tasks(command, lower, upper, tasks, tolerance, *, measured_q=None):
    """Project jointly onto hand paths using only free joints inside q/rate bounds.

    Each task is (position(q), jacobian(q), segment_start, segment_goal).
    Infeasible paths can leave a residual; callers must certify the final step.
    """
    command = np.clip(np.asarray(command, dtype=float), lower, upper)
    if not tasks:
        return command

    def residuals(q):
        return [segment_error(position(q), start, goal)
                for position, _jacobian, start, goal in tasks]

    inward_constraints = []
    if measured_q is not None:
        measured_q = np.asarray(measured_q, dtype=float)
        for task, (error, projection) in zip(tasks, residuals(measured_q)):
            distance = float(np.linalg.norm(error))
            if distance > 0.75 * tolerance:
                normal = error @ projection @ task[1](measured_q) / distance
                inward = 0.25 * (distance - 0.5 * tolerance)
                inward_constraints.append((normal, inward))

    def errors(q, paths):
        # An endpoint can be inside the tube while the initial direction goes
        # outward. Give the projection room to turn inward before backtracking.
        inward = [max(float(normal @ (q - measured_q)) + margin, 0.0)
                  for normal, margin in inward_constraints]
        return np.r_[np.concatenate([path[0] for path in paths]), inward]

    for _ in range(12):
        paths = residuals(command)
        error = errors(command, paths)
        if (max(np.linalg.norm(path[0]) for path in paths) <= 0.5 * tolerance
                and np.max(error[3 * len(tasks):], initial=0.0) <= 1e-8):
            break
        jacobian = np.vstack([path[1] @ task[1](command)
                              for task, path in zip(tasks, paths)])
        if inward_constraints:
            jacobian = np.vstack([jacobian, *[
                normal if value > 0.0 else np.zeros_like(normal)
                for (normal, _margin), value in zip(inward_constraints, error[3 * len(tasks):])
            ]])
        if not np.all(np.isfinite(error)) or not np.all(np.isfinite(jacobian)):
            break
        error_norm = float(np.linalg.norm(error))
        # Solve with the actual box bounds. Permanently deleting joints whose
        # first unconstrained correction points outward can discard a feasible
        # combination once other joints reach their bounds.
        free = upper - lower > 1e-12
        if not np.any(free):
            break
        correction = np.zeros(command.size)
        count = int(np.sum(free))
        correction[free] = lsq_linear(
            np.vstack([jacobian[:, free], 1e-4 * np.eye(count)]),
            np.r_[-error, np.zeros(count)],
            bounds=((lower - command)[free], (upper - command)[free]),
            method="bvls", tol=1e-8, max_iter=2 * count,
        ).x
        for scale in (1.0, 0.5, 0.25, 0.125):
            candidate = np.clip(command + scale * correction, lower, upper)
            candidate_error = np.linalg.norm(errors(candidate, residuals(candidate)))
            if candidate_error < error_norm:
                command = candidate
                break
        else:
            break
    return command


def limit_cartesian_step(measured_q, command, tcp_position, start, goal, tolerance, reach,
                         *, allow_recovery=False):
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
    # An external tracking error cannot be certified inside the tube. A caller
    # may instead accept only steps that shrink the existing error.
    recovering = initial_deviation > tolerance
    if recovering and not allow_recovery:
        return measured_q.copy(), 0.0
    delta = command - measured_q
    for attempt in range(13):
        scale = 0.5 ** attempt
        step = scale * delta
        curvature = reach * float(np.sum(np.abs(step))) ** 2 / (2.0 * 4**2)
        previous = initial_deviation
        for fraction in (0.25, 0.5, 0.75, 1.0):
            current = deviation(measured_q + fraction * step)
            # The chord-error envelope vanishes at each known endpoint, so a
            # point exactly on the tube boundary can still move inward.
            if curvature > 0.0:
                peak = float(np.clip(0.5 + (current - previous) / (2.0 * curvature), 0.0, 1.0))
                bound = previous + peak * (current - previous) + curvature * peak * (1.0 - peak)
            else:
                bound = max(previous, current)
            if not np.isfinite(bound) or bound > (initial_deviation + 1e-9 if recovering else tolerance):
                break
            previous = current
        else:
            if recovering and previous >= initial_deviation - 1e-9:
                continue
            return measured_q + step, scale
    return measured_q.copy(), 0.0
