"""Tests for forward kinematics."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import numpy.testing as npt

from robot_arm_sim.models.robot import URDFJoint, URDFLink, URDFRobot
from robot_arm_sim.simulate.kinematics import (
    forward_kinematics,
    matrix_to_position_euler,
    rotation_matrix,
    rpy_to_matrix,
    translation_matrix,
)


def test_rotation_matrix_identity():
    m = rotation_matrix([0, 0, 1], 0.0)
    npt.assert_array_almost_equal(m, np.eye(4))


def test_rotation_matrix_90_z():
    m = rotation_matrix([0, 0, 1], math.pi / 2)
    # X axis should map to Y axis
    npt.assert_array_almost_equal(m[:3, 0], [0, 1, 0], decimal=5)
    npt.assert_array_almost_equal(m[:3, 1], [-1, 0, 0], decimal=5)
    npt.assert_array_almost_equal(m[:3, 2], [0, 0, 1], decimal=5)


def test_rpy_to_matrix_identity():
    m = rpy_to_matrix([0, 0, 0])
    npt.assert_array_almost_equal(m, np.eye(4))


def test_translation_matrix():
    m = translation_matrix([1.0, 2.0, 3.0])
    assert m[0, 3] == 1.0
    assert m[1, 3] == 2.0
    assert m[2, 3] == 3.0
    npt.assert_array_almost_equal(m[:3, :3], np.eye(3))


def test_forward_kinematics_zero_config(robot_dir: Path):
    from robot_arm_sim.simulate.urdf_loader import load_urdf

    robot = load_urdf(robot_dir / "robot.urdf")
    chain = robot.get_kinematic_chain()
    angles = {j.name: 0.0 for j in chain}
    transforms = forward_kinematics(robot, angles)

    # Root link should be at identity
    root_name = chain[0].parent
    npt.assert_array_almost_equal(transforms[root_name], np.eye(4))


def test_forward_kinematics_nonzero(robot_dir: Path):
    from robot_arm_sim.simulate.urdf_loader import load_urdf

    robot = load_urdf(robot_dir / "robot.urdf")
    chain = robot.get_kinematic_chain()
    angles = {j.name: 0.0 for j in chain}

    transforms_zero = forward_kinematics(robot, angles)

    # Rotate first joint
    first_joint = chain[0]
    angles[first_joint.name] = 0.5
    transforms_rotated = forward_kinematics(robot, angles)

    # Child link should have moved
    child = first_joint.child
    diff = np.linalg.norm(
        transforms_rotated[child][:3, 3] - transforms_zero[child][:3, 3]
    )
    # If origin is at the rotation axis, position might not change,
    # but the rotation part of the matrix should differ
    rot_diff = np.linalg.norm(
        transforms_rotated[child][:3, :3] - transforms_zero[child][:3, :3]
    )
    assert diff > 1e-6 or rot_diff > 1e-6


def test_matrix_to_position_euler_roundtrip():
    # Create a known transform
    t = translation_matrix([1.0, 2.0, 3.0])
    r = rpy_to_matrix([0.1, 0.2, 0.3])
    m = t @ r

    pos, euler = matrix_to_position_euler(m)
    npt.assert_array_almost_equal(pos, [1.0, 2.0, 3.0], decimal=5)
    npt.assert_array_almost_equal(euler, [0.1, 0.2, 0.3], decimal=5)


def _simple_robot() -> URDFRobot:
    """Build a minimal 2-link robot for testing."""
    links = [
        URDFLink(name="base"),
        URDFLink(name="link1"),
        URDFLink(name="link2"),
    ]
    joints = [
        URDFJoint(
            name="j1",
            joint_type="revolute",
            parent="base",
            child="link1",
            origin_xyz=[0, 0, 0.1],
        ),
        URDFJoint(
            name="j2",
            joint_type="revolute",
            parent="link1",
            child="link2",
            origin_xyz=[0, 0, 0.2],
        ),
    ]
    return URDFRobot(name="test", links=links, joints=joints)


def test_forward_kinematics_simple_robot():
    robot = _simple_robot()
    angles = {"j1": 0.0, "j2": 0.0}
    transforms = forward_kinematics(robot, angles)

    # link2 should be at z = 0.1 + 0.2 = 0.3
    npt.assert_array_almost_equal(transforms["link2"][:3, 3], [0, 0, 0.3], decimal=5)
