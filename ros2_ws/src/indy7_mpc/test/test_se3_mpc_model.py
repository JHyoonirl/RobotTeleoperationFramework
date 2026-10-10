"""Smoke check for the URDF/ABA-backed SE(3) Crocoddyl action model."""

from pathlib import Path

import crocoddyl
import numpy as np
import pinocchio as pin

from indy7_mpc.se3_dynamic_model import SE3DynamicActionModel


def test_zero_pose_error_has_zero_tracking_cost():
    urdf = Path(__file__).parents[1] / "urdf" / "indy7.urdf"
    model = pin.buildModelFromUrdf(str(urdf))
    state = crocoddyl.StateMultibody(model)
    q = np.array([0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
    data = model.createData()
    frame_id = model.getFrameId("tcp")
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    weights = {
        "se3_position": 20.0,
        "se3_orientation": 5.0,
        "posture": 0.1,
        "velocity": 1.0,
        "torque": 0.0001,
        "joint_limit": 10.0,
    }
    action = SE3DynamicActionModel(
        crocoddyl=crocoddyl,
        pin=pin,
        model=model,
        state=state,
        frame_id=frame_id,
        target=data.oMf[frame_id],
        nominal_posture=q,
        weights=weights,
        torque_limits=model.effortLimit,
    )
    x = np.r_[q, np.zeros(model.nv)]
    u = np.zeros(model.nv)
    action_data = action.createData()
    action.calc(action_data, x, u)
    action.calcDiff(action_data, x, u)
    assert action_data.cost == 0.0
    assert np.all(np.isfinite(action_data.Lx))
    assert np.all(np.isfinite(action_data.Lxx))


if __name__ == "__main__":
    test_zero_pose_error_has_zero_tracking_cost()
    print("SE(3) action model smoke check passed")
