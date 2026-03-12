"""Tests for the custom Jacobian-based IK solver."""

from __future__ import annotations

import math

import numpy as np
import numpy.testing as npt

from robot_arm_sim.models.robot import URDFJoint, URDFLink, URDFRobot
from robot_arm_sim.simulate.ik_solver import (
    _compute_jacobian,
    _rotation_error,
    solve_ik,
)
from robot_arm_sim.simulate.kinematics import (
    forward_kinematics,
    translation_matrix,
)


def _simple_robot() -> URDFRobot:
    """Build a minimal 2-link planar robot for testing."""
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
            axis=[0, 0, 1],
            origin_xyz=[0, 0, 0.1],
        ),
        URDFJoint(
            name="j2",
            joint_type="revolute",
            parent="link1",
            child="link2",
            axis=[0, 0, 1],
            origin_xyz=[0.2, 0, 0],
        ),
    ]
    return URDFRobot(name="test_2link", links=links, joints=joints)


def _6dof_robot() -> URDFRobot:
    """Build a simple 6-DOF robot for full IK testing."""
    links = [URDFLink(name=f"link{i}") for i in range(7)]
    joints = [
        URDFJoint(
            name="j1",
            joint_type="revolute",
            parent="link0",
            child="link1",
            axis=[0, 0, 1],
            origin_xyz=[0, 0, 0.1],
        ),
        URDFJoint(
            name="j2",
            joint_type="revolute",
            parent="link1",
            child="link2",
            axis=[0, 1, 0],
            origin_xyz=[0, 0, 0.1],
        ),
        URDFJoint(
            name="j3",
            joint_type="revolute",
            parent="link2",
            child="link3",
            axis=[0, 1, 0],
            origin_xyz=[0, 0, 0.15],
        ),
        URDFJoint(
            name="j4",
            joint_type="revolute",
            parent="link3",
            child="link4",
            axis=[0, 0, 1],
            origin_xyz=[0, 0, 0.1],
        ),
        URDFJoint(
            name="j5",
            joint_type="revolute",
            parent="link4",
            child="link5",
            axis=[0, 1, 0],
            origin_xyz=[0, 0, 0.05],
        ),
        URDFJoint(
            name="j6",
            joint_type="revolute",
            parent="link5",
            child="link6",
            axis=[0, 0, 1],
            origin_xyz=[0, 0, 0.05],
        ),
    ]
    return URDFRobot(name="test_6dof", links=links, joints=joints)


class TestRotationError:
    def test_identity_gives_zero(self):
        err = _rotation_error(np.eye(3), np.eye(3))
        npt.assert_array_almost_equal(err, [0, 0, 0])

    def test_small_rotation(self):
        angle = 0.01
        r = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )
        err = _rotation_error(np.eye(3), r)
        # Should be approximately [0, 0, angle]
        npt.assert_array_almost_equal(err, [0, 0, angle], decimal=4)

    def test_90_degree_rotation(self):
        angle = math.pi / 2
        r = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )
        err = _rotation_error(np.eye(3), r)
        npt.assert_almost_equal(np.linalg.norm(err), angle, decimal=5)
        # Axis should be Z
        axis = err / np.linalg.norm(err)
        npt.assert_array_almost_equal(axis, [0, 0, 1], decimal=5)


class TestComputeJacobian:
    def test_jacobian_shape(self):
        robot = _simple_robot()
        angles = {"j1": 0.0, "j2": 0.0}
        jac, t_ee = _compute_jacobian(robot, angles)
        assert jac.shape == (6, 2)
        assert t_ee.shape == (4, 4)

    def test_jacobian_6dof_shape(self):
        robot = _6dof_robot()
        angles = {f"j{i}": 0.0 for i in range(1, 7)}
        jac, t_ee = _compute_jacobian(robot, angles)
        assert jac.shape == (6, 6)


class TestSolveIK:
    def test_solve_to_current_pose(self):
        """IK should converge instantly when target = current FK."""
        robot = _simple_robot()
        angles = {"j1": 0.3, "j2": -0.5}
        tfs = forward_kinematics(robot, angles)
        target = tfs["link2"]

        result = solve_ik(robot, target, angles)
        assert result is not None
        npt.assert_almost_equal(result["j1"], 0.3, decimal=3)
        npt.assert_almost_equal(result["j2"], -0.5, decimal=3)

    def test_solve_position_change(self):
        """IK should find angles that reach a different position."""
        robot = _simple_robot()
        zero_angles = {"j1": 0.0, "j2": 0.0}
        # Target: rotate j1 by 0.5 rad
        target_angles = {"j1": 0.5, "j2": 0.0}
        tfs = forward_kinematics(robot, target_angles)
        target = tfs["link2"]

        result = solve_ik(robot, target, zero_angles)
        assert result is not None

        # Verify the resulting FK matches the target
        result_tfs = forward_kinematics(robot, result)
        pos_err = np.linalg.norm(result_tfs["link2"][:3, 3] - target[:3, 3])
        assert pos_err < 1e-3

    def test_6dof_full_pose(self):
        """6-DOF robot should converge on both position and orientation."""
        robot = _6dof_robot()
        seed = {f"j{i}": 0.0 for i in range(1, 7)}
        goal_angles = {
            "j1": 0.3,
            "j2": 0.2,
            "j3": -0.4,
            "j4": 0.1,
            "j5": 0.3,
            "j6": -0.2,
        }
        tfs = forward_kinematics(robot, goal_angles)
        target = tfs["link6"]

        result = solve_ik(robot, target, seed)
        assert result is not None

        result_tfs = forward_kinematics(robot, result)
        pos_err = np.linalg.norm(result_tfs["link6"][:3, 3] - target[:3, 3])
        rot_err = np.linalg.norm(
            _rotation_error(result_tfs["link6"][:3, :3], target[:3, :3])
        )
        assert pos_err < 1e-3
        assert rot_err < 1e-2

    def test_unreachable_returns_none(self):
        """IK should return None for an unreachable target."""
        robot = _simple_robot()
        angles = {"j1": 0.0, "j2": 0.0}
        # Target far outside workspace
        target = translation_matrix([10.0, 10.0, 10.0])

        result = solve_ik(robot, target, angles)
        assert result is None

    def test_respects_joint_limits(self):
        """Joint angles in result should be within limits."""
        robot = _6dof_robot()
        seed = {f"j{i}": 0.0 for i in range(1, 7)}
        goal = {
            "j1": 1.0,
            "j2": 0.5,
            "j3": -0.3,
            "j4": 0.0,
            "j5": 0.5,
            "j6": 0.0,
        }
        tfs = forward_kinematics(robot, goal)
        target = tfs["link6"]

        result = solve_ik(robot, target, seed)
        if result is not None:
            chain = robot.get_kinematic_chain()
            for j in chain:
                if j.name in result:
                    assert j.limit_lower <= result[j.name] <= j.limit_upper
