"""Tests for forward kinematics."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import numpy.testing as npt

from robot_arm_sim.models.robot import URDFJoint, URDFLink, URDFRobot
from robot_arm_sim.simulate.kinematics import (
    axis_to_quaternion,
    forward_kinematics,
    matrix_to_position_euler,
    matrix_to_quaternion,
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


# ---------------------------------------------------------------------------
# matrix_to_quaternion tests
# ---------------------------------------------------------------------------


def _assert_unit_quaternion(q: list[float]) -> None:
    """Assert quaternion has unit norm."""
    norm = math.sqrt(sum(c * c for c in q))
    assert abs(norm - 1.0) < 1e-6, f"Quaternion norm {norm} != 1"


def test_matrix_to_quaternion_identity():
    """Identity matrix -> quaternion [0,0,0,1]."""
    q = matrix_to_quaternion(np.eye(4))
    _assert_unit_quaternion(q)
    npt.assert_array_almost_equal(q, [0, 0, 0, 1], decimal=5)


def test_matrix_to_quaternion_90_z():
    """90-degree rotation around Z axis (trace > 0 branch)."""
    m = rotation_matrix([0, 0, 1], math.pi / 2)
    q = matrix_to_quaternion(m)
    _assert_unit_quaternion(q)
    # Expected: [0, 0, sin(45), cos(45)]
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    npt.assert_array_almost_equal(q, [0, 0, s, c], decimal=5)


def test_matrix_to_quaternion_180_x():
    """180-degree rotation around X -> m[0,0] dominant branch."""
    m = rotation_matrix([1, 0, 0], math.pi)
    q = matrix_to_quaternion(m)
    _assert_unit_quaternion(q)
    # Should represent 180 deg around X: [1, 0, 0, 0] or [-1, 0, 0, 0]
    assert abs(abs(q[0]) - 1.0) < 1e-5
    assert abs(q[3]) < 1e-5


def test_matrix_to_quaternion_180_y():
    """180-degree rotation around Y -> m[1,1] dominant branch."""
    m = rotation_matrix([0, 1, 0], math.pi)
    q = matrix_to_quaternion(m)
    _assert_unit_quaternion(q)
    assert abs(abs(q[1]) - 1.0) < 1e-5
    assert abs(q[3]) < 1e-5


def test_matrix_to_quaternion_180_z():
    """180-degree rotation around Z -> m[2,2] dominant branch."""
    m = rotation_matrix([0, 0, 1], math.pi)
    q = matrix_to_quaternion(m)
    _assert_unit_quaternion(q)
    assert abs(abs(q[2]) - 1.0) < 1e-5
    assert abs(q[3]) < 1e-5


def test_matrix_to_quaternion_90_x():
    """90-degree rotation around X."""
    m = rotation_matrix([1, 0, 0], math.pi / 2)
    q = matrix_to_quaternion(m)
    _assert_unit_quaternion(q)
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    npt.assert_array_almost_equal(q, [s, 0, 0, c], decimal=5)


def test_matrix_to_quaternion_90_y():
    """90-degree rotation around Y."""
    m = rotation_matrix([0, 1, 0], math.pi / 2)
    q = matrix_to_quaternion(m)
    _assert_unit_quaternion(q)
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    npt.assert_array_almost_equal(q, [0, s, 0, c], decimal=5)


# ---------------------------------------------------------------------------
# axis_to_quaternion tests
# ---------------------------------------------------------------------------


def test_axis_to_quaternion_zero_vector():
    """Zero vector -> identity quaternion."""
    q = axis_to_quaternion([0, 0, 0])
    npt.assert_array_almost_equal(q, [0, 0, 0, 1], decimal=5)


def test_axis_to_quaternion_y_aligned():
    """Y-axis aligned -> identity quaternion (already along local Y)."""
    q = axis_to_quaternion([0, 1, 0])
    npt.assert_array_almost_equal(q, [0, 0, 0, 1], decimal=5)


def test_axis_to_quaternion_negative_y():
    """Opposite to Y -> 180 deg around Z."""
    q = axis_to_quaternion([0, -1, 0])
    npt.assert_array_almost_equal(q, [0, 0, 1, 0], decimal=5)


def test_axis_to_quaternion_x_direction():
    """X direction -> general case."""
    q = axis_to_quaternion([1, 0, 0])
    _assert_unit_quaternion(q)
    # Rotating Y onto X is a -90 deg rotation around Z
    # q should rotate [0,1,0] to [1,0,0]


def test_axis_to_quaternion_z_direction():
    """Z direction -> general case."""
    q = axis_to_quaternion([0, 0, 1])
    _assert_unit_quaternion(q)


def test_axis_to_quaternion_arbitrary():
    """Arbitrary direction vector."""
    q = axis_to_quaternion([0.5, 0.5, 0.5])
    _assert_unit_quaternion(q)


# ---------------------------------------------------------------------------
# rpy_to_matrix additional tests
# ---------------------------------------------------------------------------


def test_rpy_to_matrix_roll_only():
    """Pure roll rotation (around X)."""
    angle = math.pi / 4
    m = rpy_to_matrix([angle, 0, 0])
    # Should rotate Y toward Z
    c = math.cos(angle)
    s = math.sin(angle)
    # First column unchanged (X axis)
    npt.assert_array_almost_equal(m[:3, 0], [1, 0, 0], decimal=5)
    # Check rotation in YZ plane
    npt.assert_array_almost_equal(m[:3, 1], [0, c, s], decimal=5)


def test_rpy_to_matrix_pitch_only():
    """Pure pitch rotation (around Y)."""
    angle = math.pi / 6
    m = rpy_to_matrix([0, angle, 0])
    c = math.cos(angle)
    s = math.sin(angle)
    # Y column unchanged
    npt.assert_array_almost_equal(m[:3, 1], [0, 1, 0], decimal=5)
    # Z axis rotates toward X
    npt.assert_array_almost_equal(m[:3, 2], [s, 0, c], decimal=5)


def test_rpy_to_matrix_yaw_only():
    """Pure yaw rotation (around Z)."""
    angle = math.pi / 3
    m = rpy_to_matrix([0, 0, angle])
    c = math.cos(angle)
    s = math.sin(angle)
    # Z column unchanged
    npt.assert_array_almost_equal(m[:3, 2], [0, 0, 1], decimal=5)
    # X axis rotates toward Y
    npt.assert_array_almost_equal(m[:3, 0], [c, s, 0], decimal=5)


def test_rpy_to_matrix_combined():
    """Combined roll-pitch-yaw, verify orthogonality."""
    m = rpy_to_matrix([0.3, 0.5, 0.7])
    # Rotation part should be orthogonal: R^T R = I
    r = m[:3, :3]
    npt.assert_array_almost_equal(r.T @ r, np.eye(3), decimal=5)
    # Determinant should be 1
    assert abs(np.linalg.det(r) - 1.0) < 1e-6


# ---------------------------------------------------------------------------
# matrix_to_position_euler edge cases
# ---------------------------------------------------------------------------


def test_matrix_to_position_euler_gimbal_lock():
    """Gimbal lock (pitch = +90 degrees) triggers singular branch."""
    m = rpy_to_matrix([0, math.pi / 2, 0])
    t = translation_matrix([5.0, 6.0, 7.0]) @ m
    pos, euler = matrix_to_position_euler(t)
    npt.assert_array_almost_equal(pos, [5.0, 6.0, 7.0], decimal=5)
    # Reconstruct and verify the rotation matrices match
    m_reconstructed = rpy_to_matrix(euler)
    npt.assert_array_almost_equal(m[:3, :3], m_reconstructed[:3, :3], decimal=4)


def test_matrix_to_position_euler_negative_pitch():
    """Negative pitch = -90 degrees (singular)."""
    m = rpy_to_matrix([0, -math.pi / 2, 0])
    t = translation_matrix([0, 0, 0]) @ m
    pos, euler = matrix_to_position_euler(t)
    m_reconstructed = rpy_to_matrix(euler)
    npt.assert_array_almost_equal(m[:3, :3], m_reconstructed[:3, :3], decimal=4)
