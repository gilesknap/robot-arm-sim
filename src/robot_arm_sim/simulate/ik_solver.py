"""Inverse kinematics solver using ikpy."""

from __future__ import annotations

import logging
import warnings
from pathlib import Path

import numpy as np
from ikpy.chain import Chain

logger = logging.getLogger(__name__)


def build_ik_chain(urdf_path: Path) -> Chain:
    """Build an ikpy Chain from a URDF file, marking fixed links inactive."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        chain = Chain.from_urdf_file(str(urdf_path))
    mask = [link.joint_type != "fixed" for link in chain.links]
    return Chain.from_urdf_file(str(urdf_path), active_links_mask=mask)


def solve_ik(
    chain: Chain,
    target_xyz: np.ndarray,
    current_angles: list[float],
) -> list[float] | None:
    """Solve IK for a target position.

    Args:
        chain: ikpy Chain object.
        target_xyz: Target end-effector position [x, y, z] in meters.
        current_angles: Current joint angles including base link at index 0.

    Returns:
        Joint angles (including base at index 0), or None on failure.
    """
    try:
        result = chain.inverse_kinematics(
            target_position=target_xyz,
            initial_position=current_angles,
        )
        return list(result)
    except Exception:
        logger.exception("IK solve failed")
        return None
