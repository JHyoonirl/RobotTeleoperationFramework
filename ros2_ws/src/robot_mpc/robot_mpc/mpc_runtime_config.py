"""Validate and apply tunable YAML values while preserving the running robot graph."""
from copy import copy
from pathlib import Path
import time

import numpy as np
from rclpy.parameter import Parameter
import yaml

from .mpc_cost_config import WEIGHT_PARAMETERS, apply_runtime_cost_settings


# These describe startup, robot identity, ROS wiring, or the prediction time grid.
RESTART_PARAMETERS = {
    'urdf_path', 'config_file_path', 'joint_names', 'task_frame', 'vive_hand_frame',
    'pose_frame', 'control_mode', 'prediction_model', 'dt', 'horizon', 'simulate',
    'internal_simulation', 'enable_vive_follow', 'teleop_input_mode',
    'hand_target_frame', 'start_paused', 'initialize_target_from_current_state',
}
MODEL_PARAMETERS = {
    'torque_limits', 'acceleration_limits', 'joint_limit_margin',
    'workspace_floor_enabled', 'workspace_min_z', 'workspace_floor_margin',
    'workspace_floor_cost_weight',
}


def load_runtime_parameters(path):
    try:
        document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except yaml.YAMLError as exc:
        raise ValueError(f'invalid YAML: {exc}') from exc
    if not isinstance(document, dict):
        raise ValueError('YAML root must be a mapping')
    section = document.get('crocoddyl_mpc')
    parameters = section.get('ros__parameters') if isinstance(section, dict) else None
    if not isinstance(parameters, dict) or not all(isinstance(k, str) for k in parameters):
        raise ValueError('crocoddyl_mpc.ros__parameters must be a parameter mapping')
    return parameters


def _typed_value(name, value, default):
    if isinstance(default, bool):
        valid = isinstance(value, bool)
    elif isinstance(default, float):
        valid = type(value) in (int, float) and np.isfinite(value)
        if valid:
            value = float(value)
    elif isinstance(default, int):
        valid = type(value) is int
    elif isinstance(default, str):
        valid = isinstance(value, str)
    elif isinstance(default, list):
        valid = isinstance(value, list) and bool(value)
        if valid and isinstance(default[0], str):
            valid = all(isinstance(v, str) and v for v in value)
        elif valid:
            valid = all(type(v) in (int, float) and np.isfinite(v) for v in value)
            if valid:
                value = [float(v) for v in value]
    else:
        valid = False
    if not valid:
        raise ValueError(f'{name}: invalid type or non-finite value')
    return value


def reload_runtime_configuration(node):
    path = node.cost_config_path
    if path is None:
        return
    modified = path.stat().st_mtime_ns
    if modified == node.cost_config_mtime_ns:
        return
    raw = load_runtime_parameters(path)
    unknown = set(raw) - node._configuration_defaults.keys()
    if unknown:
        raise ValueError(f'unknown MPC parameters: {", ".join(sorted(unknown))}')
    parsed = {name: _typed_value(name, value, node._configuration_defaults[name]) for name, value in raw.items()}
    previous = node._yaml_parameters
    if previous is None:
        previous = {name: node.get_parameter(name).value for name in raw}
    edited = {name for name in raw.keys() | previous.keys() if raw.get(name) != previous.get(name)}
    restart = RESTART_PARAMETERS | {name for name in node._configuration_defaults if name.endswith(('_topic', '_service'))}
    deferred = {name for name in parsed if name in restart and parsed[name] != node.get_parameter(name).value}
    changed = edited - restart
    if deferred:
        node.get_logger().warning(f'MPC YAML restart required (not applied live): {", ".join(sorted(deferred))}')
    if not changed:
        node._yaml_parameters, node.cost_config_mtime_ns = raw, modified
        return
    values = {name: node.get_parameter(name).value for name in node._configuration_defaults}
    for name in changed:
        values[name] = parsed[name] if name in parsed else node._configuration_defaults[name]
    if 'cartesian_path_cost_weight' not in raw and {'cartesian_path_tolerance', 'cartesian_path_cost_weight'} & changed:
        tolerance = values['cartesian_path_tolerance']
        if tolerance <= 0:
            raise ValueError('cartesian_path_tolerance must be positive')
        values['cartesian_path_cost_weight'] = 1. / tolerance**2
        changed.add('cartesian_path_cost_weight')
    for name in ('vive_input_timeout', 'vive_hold_timeout'):
        if values[name] <= 0:
            raise ValueError(f'{name} must be positive and finite')

    # Reuse startup validation on a shallow copy, without creating interfaces or
    # changing ROS parameters, measured state, targets, or the current solver.
    staged = copy(node)
    staged.get_parameter = lambda name: Parameter(name, value=values[name])
    before = vars(staged).copy()
    staged._read_configuration()
    fields = {name: value for name, value in vars(staged).items()
              if name != 'get_parameter' and before.get(name) is not value}
    fields.pop('cost_config_path', None)
    for name in ('acceleration_limits', 'diagnostic_acceleration_clip'):
        if np.any(staged._parse_vector(values[name], staged.model.nv) <= 0):
            raise ValueError(f'{name} must contain positive values')
    staged.state = staged.crocoddyl.StateMultibody(staged.model)
    # Validate every action, including barrier margins and control bounds.
    problem, solver = staged._make_problem()
    rebuild = bool(changed & MODEL_PARAMETERS)
    settings = {
        'weights': {action: values[name] for name, action in WEIGHT_PARAMETERS.items()},
        'nominal_posture': values['nominal_posture'],
        **{name: values[name] for name in ('running_cost_weight', 'terminal_cost_weight',
           'cartesian_path_cost_weight', 'torso_cost_weight', 'torso_pitch_cost_weight') if name in values},
    }
    if staged.control_mode == 'joint':
        settings['nominal_posture'] = values['goal']
        settings.pop('cartesian_path_cost_weight', None)
    log_changed = bool(changed & {'diagnostic_logging', 'diagnostic_log_directory'})
    old_log = node._diagnostic_file
    if log_changed:
        staged._diagnostic_file = None
        staged._open_diagnostic_log(Path(values['urdf_path']))
        if values['diagnostic_logging'] and staged._diagnostic_file is None:
            raise ValueError('could not open the proposed diagnostic log directory')
    new_log = staged._diagnostic_file if log_changed else None
    try:
        if path.stat().st_mtime_ns != modified:
            return  # The editor saved another revision during validation.
        result = node.set_parameters_atomically([Parameter(name, value=values[name]) for name in changed])
        if not result.successful:
            raise ValueError(f'ROS parameter update rejected: {result.reason}')
        if not rebuild:
            for name in ('model', 'frame_id', 'left_frame_id', 'cartesian_reach_bound'):
                fields.pop(name, None)
        node.__dict__.update(fields)
        if rebuild:
            for name in ('state', 'running_actions', 'running_action', 'terminal_action'):
                setattr(node, name, getattr(staged, name))
            node.pose_data, node.ik_data = node.model.createData(), node.model.createData()
            node.problem, node.solver = problem, solver
            state = (node.latest_state.copy() if node.latest_state is not None else
                     np.r_[node.sim_q, node.sim_v])
            node.xs = [state.copy() for _ in range(node.horizon + 1)]
            node.us = [np.zeros(node.model.nv) for _ in range(node.horizon)]
        else:
            apply_runtime_cost_settings(node.running_actions, node.terminal_action, settings)
        if node.vive_control is not None:
            node.vive_control.timeout = values['vive_input_timeout']
            node.vive_control.hold_timeout = values['vive_hold_timeout']
        if 'goal' in changed and node.control_mode == 'joint':
            node.xref = np.r_[node.goal, np.zeros(node.model.nv)]
        if changed & {'goal_position', 'goal_orientation_xyzw'}:
            target = node.pin.SE3(node.pin.Quaternion(node.goal_orientation).matrix(), node.goal_position)
            node.target_pose = target
            q = node.latest_state[:node.model.nq] if node.latest_state is not None else node.sim_q
            node.cartesian_path_start_pose = node._tcp_pose(q).copy()
            node.cartesian_path_start_time = time.monotonic()
            node._reset_pose_velocity_estimate(target)
            node.target_update_count += 1
        node.stationary_hold_active = False
        node._stationary_hold_since = None
        node._stationary_hold_samples.clear()
        if hasattr(node, '_precise_hold_q'):
            node._precise_hold_q = None
        if log_changed:
            node._diagnostic_file = new_log
            node._diagnostic_path = staged._diagnostic_path
            if old_log is not None:
                old_log.close()
        node._yaml_parameters, node.cost_config_mtime_ns = raw, modified
        node.get_logger().info(f'Applied live MPC YAML update: {", ".join(sorted(changed))}')
        return settings
    finally:
        if new_log is not None and node._diagnostic_file is not new_log:
            new_log.close()
