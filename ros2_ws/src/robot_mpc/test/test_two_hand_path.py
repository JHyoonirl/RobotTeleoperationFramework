"""Bounded simultaneous path correction, including the recorded EIR elbow stall."""

from pathlib import Path

import numpy as np
import pinocchio as pin

from robot_mpc.cartesian_path import (
    limit_cartesian_step, project_cartesian_command, project_cartesian_tasks, segment_error,
)
from robot_mpc.robot_model import load_joint_model


def test_two_paths_are_corrected_together_without_changing_input():
    first_jacobian = np.c_[np.eye(3), np.zeros((3, 3))]
    second_jacobian = np.c_[np.zeros((3, 3)), np.eye(3)]
    start, goal = np.zeros(3), np.array([0.1, 0.0, 0.0])
    tasks = [(lambda q: q[:3], lambda q: first_jacobian, start, goal),
             (lambda q: q[3:], lambda q: second_jacobian, start, goal)]
    command = np.array([0.04, 0.01, -0.01, 0.03, -0.02, 0.01])
    original = command.copy()
    corrected = project_cartesian_tasks(command, np.full(6, -0.1), np.full(6, 0.1), tasks, 0.005)
    np.testing.assert_array_equal(command, original)
    np.testing.assert_allclose(corrected[[0, 3]], command[[0, 3]])
    for position, _, begin, end in tasks:
        assert np.linalg.norm(segment_error(position(corrected), begin, end)[0]) <= 0.0025
    np.testing.assert_allclose(project_cartesian_tasks(
        command, np.full(6, -0.1), np.full(6, 0.1), tasks[::-1], 0.005), corrected, atol=1e-12)


def test_infeasible_shared_paths_preserve_joint_bounds_without_claiming_success():
    # Both hands depend on one joint but demand disjoint points outside its bounds.
    position = lambda q: np.array([q[0], 0.0, 0.0])
    jacobian = lambda q: np.array([[1.0], [0.0], [0.0]])
    tasks = [(position, jacobian, np.array([x, 0.0, 0.0]), np.array([x, 0.0, 0.0]))
             for x in (0.03, 0.04)]
    corrected = project_cartesian_tasks(np.array([0.02]), np.array([0.0]),
                                        np.array([0.01]), tasks, 0.005)
    assert 0.0 <= corrected[0] <= 0.01
    assert all(np.linalg.norm(segment_error(position(corrected), start, goal)[0]) > 0.005
               for position, _, start, goal in tasks)


def test_box_projection_does_not_discard_a_joint_needed_for_a_feasible_correction():
    jacobian = .01 * np.array([[-2., 3., 2.], [0., 2., 3.], [0., 0., 0.]])
    goal = np.array([-.01, -.015, 0.0])
    command = np.array([1., 1., -1.])
    tasks = [(lambda q: jacobian @ q, lambda q: jacobian, goal, goal)]
    corrected = project_cartesian_tasks(command, -np.ones(3), np.ones(3), tasks, .001)
    assert np.all(corrected >= -1.) and np.all(corrected <= 1.)
    assert np.linalg.norm(jacobian @ corrected - goal) < .0005


def test_single_task_wrapper_keeps_the_existing_api():
    position, jacobian = lambda q: q.copy(), lambda q: np.eye(3)
    task = (position, jacobian, np.zeros(3), np.array([0.1, 0.0, 0.0]))
    command, lower, upper = np.array([0.05, 0.02, 0.01]), np.full(3, -0.1), np.full(3, 0.1)
    np.testing.assert_array_equal(
        project_cartesian_command(command, lower, upper, *task, 0.005),
        project_cartesian_tasks(command, lower, upper, [task], 0.005))


def test_projection_can_turn_inward_when_a_safe_endpoint_initially_points_outward():
    tolerance = 0.005
    position = lambda q: np.array([q[0], tolerance + .01 * q[0] - .04 * q[0]**2 - q[1], 0.0])
    jacobian = lambda q: np.array([[1.0, 0.0], [.01 - .08 * q[0], -1.0], [0.0, 0.0]])
    measured, proposed = np.zeros(2), np.array([.5, 0.0])
    start, goal = np.zeros(3), np.array([1.0, 0.0, 0.0])
    tasks = [(position, jacobian, start, goal)]
    # Endpoint-only projection retains a step that cannot leave the boundary.
    _, scale = limit_cartesian_step(measured, proposed, position, start, goal, tolerance, 1.0)
    assert scale == 0.0
    corrected = project_cartesian_tasks(proposed, np.array([0.0, -.1]),
                                        np.array([1.0, .1]), tasks, tolerance, measured_q=measured)
    command, scale = limit_cartesian_step(measured, corrected, position, start, goal, tolerance, 1.0)
    assert scale > 0.0
    assert command[0] > 0.0
    for fraction in np.linspace(0, 1, 101):
        assert np.linalg.norm(segment_error(position(fraction * command), start, goal)[0]) <= tolerance


def test_recorded_elbow_limit_stall_can_be_corrected_using_other_joints():
    # Captured trace row62, elapsed16.481s. Reanchor both path segments at this
    # measured pose to isolate the joint-clipping failure from earlier drift.
    measured = np.array([
        0.01634953046267533, -0.024694994364621227,
        -0.37036103982812146, -0.9089954450959996, 0.03658848419653759,
        -1.0472, 0.026233515999941306, 0.14720015310151635, 0.14568019741313715,
        0.5003235088314779, 1.0541193988397801, 0.004713486687755569,
        0.7546585737330606, 0.010759406139518783, 0.015735224633891775,
        -0.0011537962941479715,
    ])
    predicted = np.array([
        0.01814241994143334, -0.028936796645192185,
        -0.36076398881698946, -0.9035289458005394, 0.04660085974046399,
        -1.0824016841885953, 0.024448311878166002, 0.1483473857428904,
        0.15562115499076476, 0.496198568532598, 1.0549162823799,
        0.006603307918864746, 0.7510811088655643, 0.011183604494909174,
        0.01749515637494837, -0.0016096222924835495,
    ])
    names = ["Joint_L0", "Joint_L1"] + [
        f"Joint_L{i}_{side}" for side in ("L", "R") for i in range(2, 9)]
    urdf = Path(__file__).parents[2] / "robot_description/urdf/eir.urdf"
    model, _ = load_joint_model(pin, urdf, names)
    data = model.createData()
    step = np.array([0.008, 0.006] + [0.02] * 14)
    lower = np.maximum(model.lowerPositionLimit, measured - step)
    # This captured regression keeps the old -60 degree elbow bound even when
    # the project's current simulation model allows a wider range.
    recorded_elbow_lower = -1.0472
    lower[5] = max(lower[5], recorded_elbow_lower)
    upper = np.minimum(model.upperPositionLimit, measured + step)
    clipped = np.clip(predicted, lower, upper)

    def position(q, frame):
        pin.framesForwardKinematics(model, data, q)
        return data.oMf[frame].translation.copy()

    def jacobian(q, frame):
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        return pin.getFrameJacobian(model, data, frame, pin.LOCAL_WORLD_ALIGNED)[:3].copy()

    tasks = []
    goals = ([0.3179847345572307, -0.3194906190811588, 0.8761013871546948],
             [0.20797771889981687, 0.3194989340445848, 0.8761017339525021])
    for side, goal in zip(("right", "left"), goals):
        frame = model.getFrameId(side + "_hand_tcp")
        tasks.append((lambda q, frame=frame: position(q, frame),
                      lambda q, frame=frame: jacobian(q, frame),
                      position(measured, frame), np.asarray(goal)))

    def errors(command):
        return np.array([np.linalg.norm(segment_error(position(command), start, goal)[0])
                         for position, _, start, goal in tasks])

    right_only = project_cartesian_command(clipped, lower, upper, *tasks[0], 0.005)
    assert errors(right_only)[0] < 0.0003
    assert errors(right_only)[1] > 0.0075
    corrected = project_cartesian_tasks(clipped, lower, upper, tasks, 0.005)
    assert np.all(corrected >= lower) and np.all(corrected <= upper)
    assert corrected[5] == recorded_elbow_lower
    assert np.max(errors(corrected)) < 0.00005
    assert np.max(np.abs(corrected - measured)) > 0.001
    # The projected endpoint also permits a certified continuous step for both
    # hands; the root controller independently certifies each hand and floor.
    reach = sum(np.linalg.norm(placement.translation) for placement in model.jointPlacements) + 0.2
    command = corrected
    for task_position, _, start, goal in tasks:
        command, scale = limit_cartesian_step(measured, command, task_position, start, goal, 0.005, reach)
        assert scale > 0.0
    for fraction in np.linspace(0.0, 1.0, 101):
        assert np.max(errors(measured + fraction * (command - measured))) < 0.005
