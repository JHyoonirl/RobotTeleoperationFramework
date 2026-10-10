"""Reserved extension point for experiment-specific cost terms.

The current direct SE(3) action model keeps its five baseline terms in
``se3_mpc_model.py``.  Keep additional experiment code here rather than
editing the ROS node; wire a custom evaluator into that action model when a
new term needs to be part of the optimization.
"""

from __future__ import annotations

def add_custom_costs(
    *,
    c,
    pin,
    model,
    state,
    nu,
    xref,
    running_costs,
    terminal_costs,
) -> None:
    """Compatibility hook for older experiments; no baseline terms live here."""
    return None
