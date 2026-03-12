"""Inverse kinematics solver using damped least-squares on the geometric Jacobian."""

from __future__ import annotations

import logging

import numpy as np

from robot_arm_sim.models.robot import URDFRobot

from .kinematics import forward_kinematics, origin_transform

logger = logging.getLogger(__name__)


def _rotation_error(r_current: np.ndarray, r_target: np.ndarray) -> np.ndarray:
    """Compute orientation error as an axis-angle vector.

    Args:
        r_current: 3x3 current rotation matrix.
        r_target: 3x3 target rotation matrix.

    Returns:
        3-element error vector (axis * angle).
    """
    r_err = r_target @ r_current.T
    trace = float(r_err[0, 0] + r_err[1, 1] + r_err[2, 2])

    # Small-angle approximation
    if trace > 3.0 - 1e-6:
        return (
            np.array(
                [
                    r_err[2, 1] - r_err[1, 2],
                    r_err[0, 2] - r_err[2, 0],
                    r_err[1, 0] - r_err[0, 1],
                ]
            )
            / 2.0
        )

    # Normal case: extract axis-angle
    angle = np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
    # Axis from skew-symmetric part
    denom = 2.0 * np.sin(angle)
    if abs(denom) < 1e-10:
        return np.zeros(3)
    axis = (
        np.array(
            [
                r_err[2, 1] - r_err[1, 2],
                r_err[0, 2] - r_err[2, 0],
                r_err[1, 0] - r_err[0, 1],
            ]
        )
        / denom
    )
    return axis * angle


def _compute_jacobian(
    robot: URDFRobot, joint_angles: dict[str, float]
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the 6xN geometric Jacobian and the end-effector transform.

    Args:
        robot: URDFRobot model.
        joint_angles: Current joint angles by name.

    Returns:
        (J, T_ee) where J is (6, N) and T_ee is (4, 4).
    """
    chain = robot.get_kinematic_chain()
    transforms = forward_kinematics(robot, joint_angles)

    # End-effector = last child link
    t_ee = transforms[chain[-1].child]
    p_ee = t_ee[:3, 3]

    # Only revolute/continuous joints
    active = [j for j in chain if j.joint_type in ("revolute", "continuous")]
    n = len(active)
    jac = np.zeros((6, n))

    for col, joint in enumerate(active):
        # Pivot frame: parent transform * joint origin (before rotation)
        t_parent = transforms.get(joint.parent, np.eye(4))
        pivot = t_parent @ origin_transform(joint)

        z_i = pivot[:3, :3] @ np.array(joint.axis)
        z_i = z_i / (np.linalg.norm(z_i) + 1e-15)
        p_i = pivot[:3, 3]

        # Linear velocity: z_i x (p_ee - p_i)
        jac[:3, col] = np.cross(z_i, p_ee - p_i)
        # Angular velocity: z_i
        jac[3:, col] = z_i

    return jac, t_ee


def solve_ik(
    robot: URDFRobot,
    target: np.ndarray,
    current_angles: dict[str, float],
    max_iter: int = 100,
    pos_tol: float = 1e-4,
    rot_tol: float = 1e-3,
    damping: float = 0.05,
) -> dict[str, float] | None:
    """Solve IK for a 4x4 target pose using damped least-squares.

    Args:
        robot: URDFRobot model.
        target: 4x4 target pose matrix.
        current_angles: Current joint angles dict.
        max_iter: Maximum iterations.
        pos_tol: Position convergence tolerance (metres).
        rot_tol: Rotation convergence tolerance (radians).
        damping: Damping factor (lambda) for DLS.

    Returns:
        Updated joint angles dict, or None if not converged.
    """
    chain = robot.get_kinematic_chain()
    active = [j for j in chain if j.joint_type in ("revolute", "continuous")]
    if not active:
        return None

    angles = dict(current_angles)
    p_target = target[:3, 3]
    r_target = target[:3, :3]
    lam2 = damping * damping

    for _ in range(max_iter):
        jac, t_ee = _compute_jacobian(robot, angles)
        p_ee = t_ee[:3, 3]
        r_ee = t_ee[:3, :3]

        pos_err = p_target - p_ee
        rot_err = _rotation_error(r_ee, r_target)

        if np.linalg.norm(pos_err) < pos_tol and np.linalg.norm(rot_err) < rot_tol:
            return angles

        error = np.concatenate([pos_err, rot_err])

        # Damped least-squares: dq = J^T (J J^T + λ²I)^{-1} e
        jjt = jac @ jac.T + lam2 * np.eye(6)
        dq = jac.T @ np.linalg.solve(jjt, error)

        # Update and clamp
        for i, joint in enumerate(active):
            angles[joint.name] = float(
                np.clip(
                    angles.get(joint.name, 0.0) + dq[i],
                    joint.limit_lower,
                    joint.limit_upper,
                )
            )

    logger.debug("IK did not converge within %d iterations", max_iter)
    return None
