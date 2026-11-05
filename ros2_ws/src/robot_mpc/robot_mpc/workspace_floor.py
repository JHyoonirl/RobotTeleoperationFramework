"""Continuous joint-command checks for a horizontal Cartesian floor."""

import numpy as np


def limit_height_step(measured_q, command, position, jacobian, minimum_z, reach):
    """Certify a joint-linear step, or return a shorter safe prefix.

    For this bounded revolute chain, |z''(s)| <= reach * sum(|delta_q|)^2.
    The chord remainder bounds unsampled heights; the derivative bound also
    permits an upward departure when the initial point is exactly on the floor.
    """
    measured_q, command = np.asarray(measured_q), np.asarray(command)

    def height(q):
        point = np.asarray(position(q), dtype=float)
        return float(point[2]) if point.shape == (3,) and np.all(np.isfinite(point)) else -np.inf

    initial = height(measured_q)
    if initial < minimum_z:
        return measured_q.copy(), 0.0
    delta = command - measured_q
    if not np.any(delta):
        return measured_q.copy(), 1.0
    for attempt in range(17):
        scale = 0.5 ** attempt
        step = scale * delta
        curvature = reach * float(np.sum(np.abs(step))) ** 2
        previous = initial
        for index in range(1, 5):
            fraction = index / 4.0
            current = height(measured_q + fraction * step)
            if current < minimum_z:
                break
            if min(previous, current) - curvature / 128.0 < minimum_z:
                previous_q = measured_q + (fraction - 0.25) * step
                gradient = np.asarray(jacobian(previous_q), dtype=float)[2]
                slope = float(gradient @ step)
                if not np.isfinite(slope) or slope < curvature * 0.25:
                    break
            previous = current
        else:
            return measured_q + step, scale
    return measured_q.copy(), 0.0
