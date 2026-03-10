"""Shared test fixtures."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

ROBOTS_DIR = Path(__file__).resolve().parent.parent / "robots"
MECA500_DIR = ROBOTS_DIR / "Meca500-R3"


@pytest.fixture
def robot_dir(tmp_path: Path) -> Path:
    """Copy the Meca500-R3 robot folder to a temp directory and return it."""
    dest = tmp_path / "Meca500-R3"
    shutil.copytree(MECA500_DIR, dest)
    return dest


@pytest.fixture
def robot_dir_no_urdf(robot_dir: Path) -> Path:
    """Robot directory without a URDF file (pre-assembly state)."""
    urdf = robot_dir / "robot.urdf"
    if urdf.exists():
        urdf.unlink()
    return robot_dir
