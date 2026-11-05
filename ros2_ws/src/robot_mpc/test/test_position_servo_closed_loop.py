"""Run real MPC callbacks against a delayed position servo, without hold."""

from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pinocchio as pin
import pytest
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from robot_mpc.crocoddyl_mpc_node import CrocoddylMPCNode


@pytest.fixture
def controller():
    package = Path(__file__).parents[1]
    params = yaml.safe_load((package / "config/indy7_mpc.yaml").read_text())["crocoddyl_mpc"]["ros__parameters"]
    params.update(
        urdf_path=str(package.parent / "robot_description/urdf/indy7.urdf"),
        task_frame="hand_tcp", simulate=True, internal_simulation=False,
        enable_vive_follow=False, diagnostic_logging=False,
        initialize_target_from_current_state=False, start_paused=False,
        stationary_hold_duration=30.0,
    )
    context = Context()
    rclpy.init(args=[], context=context, domain_id=95)
    node = None
    try:
        node = CrocoddylMPCNode(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in params.items()
        ])
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        node._diagnostics_due = lambda: False
        node.status_pub = Mock()
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def run_servo(node, monkeypatch, delay, *, captured_start=False, moving=False, rotate=False):
    clock = [100.0]
    monkeypatch.setattr("robot_mpc.crocoddyl_mpc_node.time.monotonic", lambda: clock[0])
    q = np.array([0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
    if captured_start:
        # Start of fixed target 54 in the operator's 20261105T110028 trace.
        q = np.array([-.0075919571, .3557264994, -1.838661032,
                      .0010929032, -1.698345104, -.0096630305])
    node.latest_state = np.r_[q, np.zeros(6)]
    node.last_state_update_monotonic = clock[0]
    node.last_state_stamp_s = clock[0]
    node.cartesian_path_start_pose = node._tcp_pose(q).copy()
    node.cartesian_path_start_time = clock[0]
    node.cartesian_path_initialized_from_state = True
    node.target_pose = pin.SE3(
        pin.Quaternion(np.array([0.0, -.00079632671, 0.0, .99999968293])).matrix(),
        np.array([.25049029938, -.18650000009, .44193837023]),
    )
    origin = node.cartesian_path_start_pose.copy()
    if rotate:
        node.target_pose = origin.copy()
        node.target_pose.rotation = origin.rotation @ pin.exp3(np.array([0.15, 0.10, 0.0]))
    node.xs = [node.latest_state.copy() for _ in range(node.horizon + 1)]
    node.us = [np.zeros(6) for _ in range(node.horizon)]
    pending = [q.copy()]
    node._publish_joint_command = lambda command: pending.__setitem__(0, command.copy())
    history = [q.copy() for _ in range(delay + 1)]
    positions, errors = [], []
    for tick in range(400):
        clock[0] = 100.0 + (tick + 1) * node.dt
        measured = history[-1 - delay]
        message = JointState()
        message.name = node.joint_names
        message.position = measured.tolist()
        message.velocity = [0.0] * 6
        stamp = clock[0] - delay * node.dt
        message.header.stamp.sec = int(stamp)
        message.header.stamp.nanosec = round((stamp % 1.0) * 1e9)
        node._on_joint_state(message)
        if moving and tick < 200 and tick % 2 == 0:
            pose = origin.copy()
            t = tick * node.dt
            pose.translation += np.array([.03 * np.sin(np.pi * t), .02 * (1 - np.cos(np.pi * t)), 0.0])
            target = PoseStamped()
            target.header.frame_id = node.pose_frame
            target.pose.position.x, target.pose.position.y, target.pose.position.z = pose.translation
            quaternion = pin.Quaternion(pose.rotation).coeffs()
            (target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w) = quaternion
            node._apply_target_pose(target)
        node._solve_once()
        assert node.status_pub.publish.call_args.args[0].data != "stationary_hold"
        actual = pending[0].copy()
        assert np.all(np.isfinite(actual))
        assert np.all(actual >= node.model.lowerPositionLimit)
        assert np.all(actual <= node.model.upperPositionLimit)
        pose = node._tcp_pose(actual).copy()
        assert pose.translation[2] >= node.workspace_min_z
        history.append(actual)
        positions.append(actual)
        errors.append(np.linalg.norm(pose.translation - node.target_pose.translation))
    return np.asarray(positions), np.asarray(errors)


@pytest.mark.parametrize("delay,captured_start", [(0, False), (1, False), (2, False), (1, True)])
def test_fixed_goal_converges_with_delayed_feedback_and_hold_disabled(
    controller, monkeypatch, delay, captured_start
):
    q, errors = run_servo(controller, monkeypatch, delay, captured_start=captured_start)
    assert errors[-1] < 0.002
    assert np.max(np.ptp(q[-50:], axis=0)) < 1e-4


def test_moving_goal_tracks_then_settles_with_twenty_ms_feedback_delay(controller, monkeypatch):
    q, errors = run_servo(controller, monkeypatch, 2, moving=True)
    assert np.sqrt(np.mean(errors[:200] ** 2)) < 0.01
    assert errors[-1] < 0.002
    assert np.max(np.ptp(q[-50:], axis=0)) < 1e-4


def test_rotation_settles_with_twenty_ms_feedback_delay(controller, monkeypatch):
    q, errors = run_servo(controller, monkeypatch, 2, rotate=True)
    pose = controller._tcp_pose(q[-1])
    assert errors[-1] < 0.002
    assert np.linalg.norm(pin.log3(pose.rotation.T @ controller.target_pose.rotation)) < 0.01
    assert np.max(np.ptp(q[-50:], axis=0)) < 1e-4
