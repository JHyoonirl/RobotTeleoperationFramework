"""Keep the hand frames on the CAD flange, rather than the wrist joint axis."""

from math import cos, sin
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest


DESCRIPTION = Path(__file__).parents[1]


@pytest.mark.parametrize("arm", ["left", "right"])
def test_hand_tcp_origin_and_forward_axis_match_terminal_cad_flange(arm):
    root = ET.parse(DESCRIPTION / "urdf/eir.urdf").getroot()
    joint = root.find(f"./joint[@name='{arm}_hand_tcp_joint']")
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == f"{arm}_arm_link_7"
    assert joint.find("child").get("link") == f"{arm}_hand_tcp"
    assert len(root.find(f"./link[@name='{arm}_hand_tcp']")) == 0
    origin = joint.find("origin")
    x, y, z = map(float, origin.get("xyz").split())
    roll, pitch, yaw = map(float, origin.get("rpy").split())
    visual = root.find(f"./link[@name='{arm}_arm_link_7']/visual")
    assert visual.find("origin").get("xyz") == "0 0 0"
    assert visual.find("origin").get("rpy") == "0 0 0"
    mesh = visual.find("geometry/mesh")
    sx, sy, sz = map(float, mesh.get("scale").split())
    data = (DESCRIPTION / mesh.get("filename").removeprefix(
        "package://robot_description/")).read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + count * 50
    vertices = [tuple(v * scale for v, scale in zip(
        struct.unpack_from("<3f", data, 84 + triangle * 50 + 12 + vertex * 12),
        (sx, sy, sz))) for triangle in range(count) for vertex in range(3)]
    # STL tessellation has sub-micrometre roundoff at the nominal flange plane.
    assert abs(z - max(v[2] for v in vertices)) < 1e-6
    flange = [v for v in vertices if abs(v[2] - z) < 1e-6]
    assert len(flange) > 500
    assert abs(x - (min(v[0] for v in flange) + max(v[0] for v in flange)) / 2) < 1e-5
    assert abs(y - (min(v[1] for v in flange) + max(v[1] for v in flange)) / 2) < 1e-5
    assert roll == yaw == 0
    assert abs(cos(pitch)) < 1e-4
    assert -sin(pitch) > 0.99999  # TCP +X follows the outward flange normal +Z.
