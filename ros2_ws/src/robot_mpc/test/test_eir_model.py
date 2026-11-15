"""Check official EIR assets, reduced arm models and actual MPC callbacks."""

from pathlib import Path
from unittest.mock import Mock
import xml.etree.ElementTree as ET

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
from robot_mpc.mpc_cost_config import load_runtime_cost_settings
from robot_mpc.robot_model import load_joint_model


PACKAGE = Path(__file__).parents[1]
DESCRIPTION = PACKAGE.parent / "robot_description"
URDF = DESCRIPTION / "urdf/eir.urdf"


def arm_joints(arm):
    return [f"Joint_L{i}_{arm[0].upper()}" for i in range(2, 9)]


def initial_joint_positions():
    return {joint.get("name"): float(joint.findtext(
        "./state_interface[@name='position']/param[@name='initial_value']"))
        for joint in ET.parse(URDF).getroot().findall("./ros2_control/joint")}


def test_eir_mesh_paths_and_mock_joint_interfaces_are_complete():
    root = ET.parse(URDF).getroot()
    joints = root.findall("./joint[@type='revolute']")
    assert len(joints) == 18
    for mesh in root.findall(".//mesh"):
        relative = mesh.get("filename").removeprefix("package://robot_description/")
        assert (DESCRIPTION / relative).is_file()
    control = root.find("ros2_control")
    assert control.findtext("hardware/plugin") == "mock_components/GenericSystem"
    assert len(control.findall("joint")) == 18
    for joint in control.findall("joint"):
        assert [i.get("name") for i in joint.findall("command_interface")] == ["position"]
    for arm in ("left", "right"):
        settings = yaml.safe_load((PACKAGE / f"config/eir_{arm}_controllers.yaml").read_text())
        assert settings["forward_position_controller"]["ros__parameters"]["joints"] == arm_joints(arm)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_simulation_elbow_limits_match_urdf_control_interface_and_reduced_model(arm):
    root = ET.parse(URDF).getroot()
    name = f"Joint_L5_{arm[0].upper()}"
    limit = root.find(f"./joint[@name='{name}']/limit")
    bounds = [float(limit.get(key)) for key in ("lower", "upper")]
    np.testing.assert_allclose(bounds, [-2.5307, 2.5307], rtol=0, atol=1e-12)
    command = root.find(f"./ros2_control/joint[@name='{name}']/command_interface[@name='position']")
    np.testing.assert_allclose([float(command.findtext(f"./param[@name='{key}']"))
                               for key in ("min", "max")], bounds, rtol=0, atol=1e-12)
    model, _ = load_joint_model(pin, URDF, arm_joints(arm))
    index = model.joints[model.getJointId(name)].idx_q
    np.testing.assert_allclose([model.lowerPositionLimit[index], model.upperPositionLimit[index]],
                               bounds, rtol=0, atol=1e-12)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_reduced_eir_arm_matches_full_model_kinematics(arm):
    full = pin.buildModelFromUrdf(str(URDF))
    q_full = pin.neutral(full)
    for name, value in initial_joint_positions().items():
        q_full[full.joints[full.getJointId(name)].idx_q] = value
    model, q = load_joint_model(pin, URDF, arm_joints(arm))
    assert model.nq == model.nv == 7
    assert list(model.names[1:]) == arm_joints(arm)
    q += np.array([0.1, -0.1, 0.2, 0.1, -0.1, 0.1, 0.1])
    for name, value in zip(arm_joints(arm), q):
        q_full[full.joints[full.getJointId(name)].idx_q] = value
    full_data, data = full.createData(), model.createData()
    pin.framesForwardKinematics(full, full_data, q_full)
    pin.framesForwardKinematics(model, data, q)
    frame = f"{arm}_hand_tcp"
    np.testing.assert_allclose(data.oMf[model.getFrameId(frame)].homogeneous,
                               full_data.oMf[full.getFrameId(frame)].homogeneous, atol=1e-12)


@pytest.mark.parametrize("config_name", ["eir_mpc", "eir_left_mpc", "eir_dual_mpc"])
def test_ready_posture_matches_initial_goal_and_nominal_for_every_control_mode(config_name):
    params = yaml.safe_load((PACKAGE / f"config/{config_name}.yaml").read_text())[
        "crocoddyl_mpc"]["ros__parameters"]
    initial = initial_joint_positions()
    expected = [initial[name] for name in params["joint_names"]]
    np.testing.assert_allclose(params["goal"], expected, atol=1e-12)
    np.testing.assert_allclose(params["nominal_posture"], expected, atol=1e-12)
    if config_name != "eir_dual_mpc":
        side = "left" if config_name == "eir_left_mpc" else "right"
        assert params["joint_names"] == arm_joints(side)
        assert params["task_frame"] == params["vive_hand_frame"] == f"{side}_hand_tcp"


def test_ready_pose_has_symmetric_hand_positions_and_full_rank_arm_jacobians():
    model = pin.buildModelFromUrdf(str(URDF))
    data, q = model.createData(), pin.neutral(model)
    initial = initial_joint_positions()
    for name, value in initial.items():
        q[model.joints[model.getJointId(name)].idx_q] = value
    pin.framesForwardKinematics(model, data, q)
    positions = {}
    for side in ("left", "right"):
        frame = model.getFrameId(f"{side}_hand_tcp")
        positions[side] = data.oMf[frame].translation.copy()
        jacobian = pin.computeFrameJacobian(model, data, q, frame, pin.LOCAL_WORLD_ALIGNED)
        columns = [model.joints[model.getJointId(name)].idx_v for name in arm_joints(side)]
        assert np.linalg.matrix_rank(jacobian[:, columns], tol=1e-6) == 6
    np.testing.assert_allclose(positions["left"], positions["right"] * [1, -1, 1], atol=1e-4)
    assert all(initial[name] == 0.0 for name in ("Joint_L0", "Joint_L1", "Joint_L2_U", "Joint_L3_U"))


@pytest.mark.parametrize("names", [[], ["missing"], ["Joint_L2_R"] * 2, arm_joints("right")[::-1]])
def test_invalid_control_joint_selection_is_rejected(names):
    with pytest.raises(ValueError):
        load_joint_model(pin, URDF, names)


@pytest.mark.parametrize("arm", ["left", "right"])
@pytest.mark.parametrize("rotate", [False, True])
def test_seven_axis_mpc_reaches_a_pose_with_delayed_full_robot_feedback(arm, rotate, monkeypatch):
    config_path = PACKAGE / ("config/eir_left_mpc.yaml" if arm == "left" else "config/eir_mpc.yaml")
    params = yaml.safe_load(config_path.read_text())["crocoddyl_mpc"]["ros__parameters"]
    assert len(load_runtime_cost_settings(config_path, joint_count=7)["nominal_posture"]) == 7
    params.update(urdf_path=str(URDF), joint_names=arm_joints(arm),
                  task_frame=f"{arm}_hand_tcp", enable_vive_follow=False,
                  start_paused=False, diagnostic_logging=False, stationary_hold_duration=30.0)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=98)
    node = None
    try:
        node = CrocoddylMPCNode(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in params.items()])
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        clock = [100.0]
        monkeypatch.setattr("robot_mpc.crocoddyl_mpc_node.time.monotonic", lambda: clock[0])
        node._diagnostics_due = lambda: False
        node.status_pub = Mock()
        q = node.sim_q.copy()
        pending = [q.copy()]
        node._publish_joint_command = lambda command: pending.__setitem__(0, command.copy())
        history = [q.copy(), q.copy(), q.copy()]
        root = ET.parse(URDF).getroot()
        names = [j.get("name") for j in root.findall("./joint[@type='revolute']")]
        initial = initial_joint_positions()
        target = None
        # Allow the redundant posture to settle after the TCP arrives; hold stays disabled.
        for tick in range(400):
            clock[0] = 100.0 + tick * node.dt
            measured = dict(zip(node.joint_names, history[-3]))
            feedback = JointState()
            feedback.name = names
            feedback.position = [measured.get(name, initial[name]) for name in names]
            stamp = clock[0] - 2 * node.dt
            feedback.header.stamp.sec = int(stamp)
            feedback.header.stamp.nanosec = int((stamp % 1) * 1e9)
            node._on_joint_state(feedback)
            if tick == 0:
                target = node._tcp_pose(q).copy()
                if rotate:
                    target.rotation = target.rotation @ pin.exp3(np.array([0.04, 0.03, 0.0]))
                else:
                    target.translation += np.array([0.03, 0.0, 0.01])
                message = PoseStamped()
                message.header.frame_id = "world"
                message.pose.position.x, message.pose.position.y, message.pose.position.z = target.translation
                quaternion = pin.Quaternion(target.rotation).coeffs()
                (message.pose.orientation.x, message.pose.orientation.y,
                 message.pose.orientation.z, message.pose.orientation.w) = quaternion
                node._apply_target_pose(message)
            node._solve_once()
            command = pending[0]
            assert command.shape == (7,)
            assert np.all(np.isfinite(command))
            assert np.all(command >= node.model.lowerPositionLimit)
            assert np.all(command <= node.model.upperPositionLimit)
            history.append(command.copy())
        error = node._pose_error(history[-1], target, node.pose_data)
        assert np.linalg.norm(error[:3]) < 0.002
        assert np.linalg.norm(error[3:]) < 0.01
        # A redundant arm can still make slow posture corrections at a fixed TCP.
        assert np.max(np.abs(np.diff(history[-50:], axis=0))) < 1e-5
        assert node.status_pub.publish.call_args.args[0].data != "stationary_hold"
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
