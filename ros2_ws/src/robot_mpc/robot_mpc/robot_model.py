"""Load the commanded revolute joints and lock the rest at their initial pose."""

import xml.etree.ElementTree as ET

import numpy as np


def load_joint_model(pin, urdf_path, joint_names):
    model = pin.buildModelFromUrdf(str(urdf_path))
    if not joint_names or len(set(joint_names)) != len(joint_names):
        raise ValueError("joint_names must contain unique model joint names")
    unknown = set(joint_names) - set(model.names[1:])
    if unknown:
        raise ValueError(f"Unknown commanded joints: {sorted(unknown)}")
    if any(joint.nq != 1 or joint.nv != 1 for joint in list(model.joints)[1:]):
        raise ValueError("MPC requires scalar, bounded revolute joints")

    initial = pin.neutral(model)
    for joint in ET.parse(urdf_path).getroot().findall("./ros2_control/joint"):
        value = joint.find("./state_interface[@name='position']/param[@name='initial_value']")
        name = joint.get("name")
        if value is not None and name in model.names:
            initial[model.joints[model.getJointId(name)].idx_q] = float(value.text)
    if (not np.all(np.isfinite(initial))
            or np.any(initial < model.lowerPositionLimit)
            or np.any(initial > model.upperPositionLimit)):
        raise ValueError("URDF initial joint positions must be finite and within limits")

    selected = [name for name in model.names[1:] if name in joint_names]
    if selected != joint_names:
        raise ValueError(f"joint_names must follow model order: {selected}")
    initial_selected = np.array([initial[model.joints[model.getJointId(name)].idx_q]
                                 for name in joint_names])
    locked = [model.getJointId(name) for name in model.names[1:] if name not in joint_names]
    if locked:
        model = pin.buildReducedModel(model, locked, initial)
    return model, initial_selected
