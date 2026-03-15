"""Shared test fixtures."""

from __future__ import annotations

import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest
import trimesh

from robot_arm_sim.models.models import ConnectionPoint, GeometricFeature

ROBOTS_DIR = Path(__file__).resolve().parent.parent / "robots"
SAMPLE_ROBOT_DIR = ROBOTS_DIR / "Meca500-R3"


@pytest.fixture
def robot_dir(tmp_path: Path) -> Path:
    """Copy the sample robot folder to a temp directory and return it."""
    dest = tmp_path / SAMPLE_ROBOT_DIR.name
    shutil.copytree(SAMPLE_ROBOT_DIR, dest)
    return dest


@pytest.fixture
def robot_dir_no_urdf(robot_dir: Path) -> Path:
    """Robot directory without a URDF file (pre-assembly state)."""
    urdf = robot_dir / "robot.urdf"
    if urdf.exists():
        urdf.unlink()
    return robot_dir


# ---------------------------------------------------------------------------
# Synthetic mesh fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def cylinder_mesh() -> trimesh.Trimesh:
    """A simple cylinder mesh (radius=10mm, height=50mm) along Z axis."""
    return trimesh.creation.cylinder(radius=10.0, height=50.0, sections=32)


@pytest.fixture
def box_mesh() -> trimesh.Trimesh:
    """A simple box mesh (20x20x40mm)."""
    return trimesh.creation.box(extents=[20.0, 20.0, 40.0])


@pytest.fixture
def l_shaped_mesh() -> trimesh.Trimesh:
    """An L-shaped mesh from two merged cylinders on perpendicular axes."""
    cyl_z = trimesh.creation.cylinder(radius=8.0, height=40.0, sections=32)
    cyl_x = trimesh.creation.cylinder(radius=8.0, height=30.0, sections=32)
    # Rotate second cylinder 90 degrees around Y and translate
    rot = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
    rot[:3, 3] = [15.0, 0.0, 20.0]
    cyl_x.apply_transform(rot)
    return trimesh.util.concatenate([cyl_z, cyl_x])


# ---------------------------------------------------------------------------
# GeometricFeature factory helpers
# ---------------------------------------------------------------------------


def make_cylinder_feature(
    axis: list[float] | None = None,
    radius_mm: float = 10.0,
    length_mm: float = 50.0,
    center: list[float] | None = None,
) -> GeometricFeature:
    """Create a cylindrical surface GeometricFeature."""
    return GeometricFeature(
        kind="cylindrical_surface",
        description="test cylinder",
        axis=axis or [0.0, 0.0, 1.0],
        radius_mm=radius_mm,
        length_mm=length_mm,
        center=center or [0.0, 0.0, 0.0],
    )


def make_flat_face_feature(
    normal: list[float] | None = None,
    area_mm2: float = 100.0,
    centroid: list[float] | None = None,
) -> GeometricFeature:
    """Create a flat face GeometricFeature."""
    return GeometricFeature(
        kind="flat_face",
        description="test flat face",
        normal=normal or [0.0, 0.0, 1.0],
        area_mm2=area_mm2,
        centroid=centroid or [0.0, 0.0, 25.0],
    )


def make_connection_point(
    end: str = "proximal",
    position: list[float] | None = None,
    axis: list[float] | None = None,
    radius_mm: float = 10.0,
    method: str = "cross_section",
) -> ConnectionPoint:
    """Create a ConnectionPoint for testing."""
    return ConnectionPoint(
        end=end,  # type: ignore[arg-type]
        position=position or [0.0, 0.0, 0.0],
        axis=axis or [0.0, 0.0, 1.0],
        radius_mm=radius_mm,
        method=method,
    )


# ---------------------------------------------------------------------------
# Mock NiceGUI fixture
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_ui(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Patch nicegui.ui to prevent server startup during tests."""
    mock = MagicMock()
    mock.run_javascript = AsyncMock(return_value=None)
    mock.navigate = MagicMock()
    mock.navigate.to = MagicMock()

    # Make context managers work (ui.row(), ui.column(), etc.)
    for widget in (
        "row",
        "column",
        "card",
        "label",
        "button",
        "slider",
        "radio",
        "separator",
        "element",
        "html",
        "dialog",
        "expansion",
        "tooltip",
        "icon",
        "select",
        "number",
        "switch",
    ):
        ctx = MagicMock()
        ctx.__enter__ = MagicMock(return_value=ctx)
        ctx.__exit__ = MagicMock(return_value=False)
        setattr(mock, widget, MagicMock(return_value=ctx))

    mock.timer = MagicMock()
    monkeypatch.setattr("nicegui.ui", mock)
    return mock
