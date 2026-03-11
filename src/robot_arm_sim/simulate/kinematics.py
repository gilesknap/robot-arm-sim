"""Forward kinematics using numpy matrix math."""

from __future__ import annotations

import numpy as np

from robot_arm_sim.models.robot import URDFJoint, URDFRobot


def rotation_matrix(axis: list[float], angle: float) -> np.ndarray:
    """Create a 4x4 rotation matrix from axis and angle (radians)."""
    ax = np.array(axis, dtype=float)
    norm = np.linalg.norm(ax)
    if norm < 1e-10:
        return np.eye(4)
    ax = ax / norm

    c = np.cos(angle)
    s = np.sin(angle)
    t = 1 - c
    x, y, z = ax

    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
            [0, 0, 0, 1],
        ]
    )


def rpy_to_matrix(rpy: list[float]) -> np.ndarray:
    """Convert roll-pitch-yaw (XYZ extrinsic) to a 4x4 rotation matrix."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0],
            [-sp, cp * sr, cp * cr, 0],
            [0, 0, 0, 1],
        ]
    )


def translation_matrix(xyz: list[float]) -> np.ndarray:
    """Create a 4x4 translation matrix."""
    m = np.eye(4)
    m[0, 3] = xyz[0]
    m[1, 3] = xyz[1]
    m[2, 3] = xyz[2]
    return m


def origin_transform(joint: URDFJoint) -> np.ndarray:
    """Get the fixed origin transform for a joint."""
    return translation_matrix(joint.origin_xyz) @ rpy_to_matrix(joint.origin_rpy)


def forward_kinematics(
    robot: URDFRobot,
    joint_angles: dict[str, float],
) -> dict[str, np.ndarray]:
    """Compute forward kinematics for all links.

    Returns a dict mapping link_name -> 4x4 transform matrix (world frame).
    """
    chain = robot.get_kinematic_chain()
    transforms: dict[str, np.ndarray] = {}

    # Root link at identity
    if chain:
        root_link = chain[0].parent
        transforms[root_link] = np.eye(4)
    elif robot.links:
        transforms[robot.links[0].name] = np.eye(4)

    for joint in chain:
        parent_tf = transforms.get(joint.parent, np.eye(4))
        angle = joint_angles.get(joint.name, 0.0)

        # T_child = T_parent * T_origin * R_joint(angle)
        joint_tf = origin_transform(joint)
        rot = rotation_matrix(joint.axis, angle)
        child_tf = parent_tf @ joint_tf @ rot

        transforms[joint.child] = child_tf

    return transforms


def matrix_to_quaternion(m: np.ndarray) -> list[float]:
    """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def axis_to_quaternion(axis: list | np.ndarray) -> list[float]:
    """Return quaternion [qx,qy,qz,qw] rotating local-Y onto *axis*."""
    a = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    a = a / norm
    # local Y = [0, 1, 0]
    dot = float(a[1])  # dot([0,1,0], a)
    if dot > 0.99999:
        return [0.0, 0.0, 0.0, 1.0]
    if dot < -0.99999:
        # 180° around Z
        return [0.0, 0.0, 1.0, 0.0]
    # cross([0,1,0], a) = [a[2], 0, -a[0]]  (only Y=0 component)
    cx, cy, cz = float(a[2]), 0.0, float(-a[0])
    s = np.sqrt((1.0 + dot) * 2.0)
    inv_s = 1.0 / s
    return [cx * inv_s, cy * inv_s, cz * inv_s, s * 0.5]


def matrix_to_position_euler(m: np.ndarray) -> tuple[list[float], list[float]]:
    """Decompose a 4x4 matrix into position [x,y,z] and euler angles [rx,ry,rz]."""
    pos = [float(m[0, 3]), float(m[1, 3]), float(m[2, 3])]

    # Extract euler angles (XYZ convention)
    sy = np.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        rx = float(np.arctan2(m[2, 1], m[2, 2]))
        ry = float(np.arctan2(-m[2, 0], sy))
        rz = float(np.arctan2(m[1, 0], m[0, 0]))
    else:
        rx = float(np.arctan2(-m[1, 2], m[1, 1]))
        ry = float(np.arctan2(-m[2, 0], sy))
        rz = 0.0

    return pos, [rx, ry, rz]
