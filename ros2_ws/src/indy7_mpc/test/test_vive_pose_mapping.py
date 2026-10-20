"""Check local TCP offsets and full-pose reference alignment."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from indy7_mpc.pose_mapping import (
    align_worlds, pose_from_transform, transform_from_pose, transform_from_rpy,
)


def test_hand_tcp_offset_is_in_original_tcp_axes():
    urdf = Path(__file__).resolve().parents[2] / "indy7_description/urdf/indy7.urdf"
    robot = ET.parse(urdf).getroot()
    joint = robot.find("joint[@name='hand_tcp_fixed']")
    assert joint.attrib["type"] == "fixed"
    assert joint.find("parent").attrib["link"] == "tcp"
    assert joint.find("child").attrib["link"] == "hand_tcp"
    origin = joint.find("origin")
    translation = np.fromstring(origin.attrib["xyz"], sep=" ")
    rpy = np.fromstring(origin.attrib["rpy"], sep=" ")
    offset = transform_from_rpy(translation, np.rad2deg(rpy))
    tcp = transform_from_rpy([0.5, 0.2, 0.4], [0, 0, 90])
    hand = tcp @ offset
    # Rotate the configured offset by world yaw=90 degrees. Keep this check
    # valid when the hand mount translation is tuned in the local URDF.
    tx, ty, tz = translation
    np.testing.assert_allclose(hand[:3, 3], [0.5 - ty, 0.2 + tx, 0.4 + tz])
    np.testing.assert_allclose(hand[:3, 0], [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(hand[:3, 2], [0, 0, -1], atol=1e-12)


def test_pinocchio_hand_frame_follows_tcp_at_different_joint_poses():
    pin = pytest.importorskip("pinocchio")
    urdf = Path(__file__).resolve().parents[2] / "indy7_description/urdf/indy7.urdf"
    model = pin.buildModelFromUrdf(str(urdf))
    assert model.nq == model.nv == 6
    data = model.createData()
    origin = ET.parse(urdf).getroot().find("joint[@name='hand_tcp_fixed']/origin")
    translation = np.fromstring(origin.attrib["xyz"], sep=" ")
    for q in (np.zeros(6), np.array([0.2, -0.3, -1.2, 0.2, -1.4, 0.15])):
        pin.framesForwardKinematics(model, data, q)
        relative = data.oMf[model.getFrameId("tcp")].inverse() * data.oMf[model.getFrameId("hand_tcp")]
        np.testing.assert_allclose(relative.translation, translation, atol=1e-12)
        np.testing.assert_allclose(relative.rotation, np.diag([-1, 1, -1]), atol=1e-12)


def test_alignment_rotates_vive_translation_into_indy_world():
    hand_start = transform_from_rpy([0.5, 0, 0.6], [0, 0, 90])
    vive_start = transform_from_rpy([2, 3, 4], [0, 0, 0])
    mapping = align_worlds(hand_start, vive_start)
    np.testing.assert_allclose(mapping @ vive_start, hand_start, atol=1e-12)
    moved_vive = transform_from_rpy([2.1, 3, 4], [0, 0, 90])
    expected = transform_from_rpy([0.5, 0.1, 0.6], [0, 0, 180])
    np.testing.assert_allclose(mapping @ moved_vive, expected, atol=1e-12)


@pytest.mark.parametrize("rpy", [[0, -180, 0], [180, 0, 0], [0, 0, 180], [35, -42, 76]])
def test_quaternion_round_trip_at_large_rotations(rpy):
    matrix = transform_from_rpy([0.3, -0.2, 0.5], rpy)
    position, quaternion = pose_from_transform(matrix)
    np.testing.assert_allclose(transform_from_pose(position, quaternion), matrix, atol=1e-12)


@pytest.mark.parametrize("quaternion", [[0, 0, 0, 0], [0, float("nan"), 0, 1]])
def test_invalid_quaternion_is_rejected(quaternion):
    with pytest.raises(ValueError):
        transform_from_pose([0, 0, 0], quaternion)
