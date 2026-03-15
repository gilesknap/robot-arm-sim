"""Tests for simulate/app/loaders.py — pure data-loading functions."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from robot_arm_sim.simulate.app.loaders import (
    load_connection_points,
    load_flat_faces,
    load_mesh_centers,
    quantize_axis,
)
from robot_arm_sim.simulate.urdf_loader import load_urdf

# ---------------------------------------------------------------------------
# quantize_axis
# ---------------------------------------------------------------------------


class TestQuantizeAxis:
    """Tests for quantize_axis — snaps a normal to the nearest cardinal axis."""

    def test_positive_x_dominant(self):
        assert quantize_axis([0.9, 0.1, 0.0]) == [1.0, 0.0, 0.0]

    def test_negative_y_dominant(self):
        assert quantize_axis([0.0, -0.8, 0.2]) == [0.0, -1.0, 0.0]

    def test_positive_z_dominant(self):
        assert quantize_axis([0.1, -0.2, 0.95]) == [0.0, 0.0, 1.0]

    def test_negative_x_dominant(self):
        assert quantize_axis([-0.7, 0.3, 0.1]) == [-1.0, 0.0, 0.0]

    def test_equal_components_picks_largest_abs(self):
        # When two are equal, max picks the first one (index 0)
        result = quantize_axis([0.5, 0.5, 0.0])
        # Both x and y are equal; max returns first match → idx 0
        assert result == [1.0, 0.0, 0.0]

    def test_all_equal_picks_first(self):
        result = quantize_axis([0.5, 0.5, 0.5])
        assert result == [1.0, 0.0, 0.0]

    def test_negative_z_dominant(self):
        assert quantize_axis([0.0, 0.0, -1.0]) == [0.0, 0.0, -1.0]

    def test_unit_vector_passthrough(self):
        assert quantize_axis([0.0, 1.0, 0.0]) == [0.0, 1.0, 0.0]


# ---------------------------------------------------------------------------
# load_mesh_centers
# ---------------------------------------------------------------------------


class TestLoadMeshCenters:
    """Tests for load_mesh_centers with real robot data."""

    def test_returns_centers_for_links_with_analysis(self, robot_dir: Path):
        robot = load_urdf(robot_dir / "robot.urdf")
        centers = load_mesh_centers(robot, robot_dir)

        # Should have entries for links that have analysis YAML files
        assert len(centers) > 0

        for _link_name, center in centers.items():
            assert isinstance(center, np.ndarray)
            assert center.shape == (3,)

    def test_center_values_are_finite(self, robot_dir: Path):
        robot = load_urdf(robot_dir / "robot.urdf")
        centers = load_mesh_centers(robot, robot_dir)

        for center in centers.values():
            assert np.all(np.isfinite(center))

    def test_returns_empty_when_no_analysis_dir(self, tmp_path: Path):
        """No analysis directory → empty dict."""
        robot = load_urdf(
            Path(__file__).parent.parent / "robots" / "Meca500-R3" / "robot.urdf"
        )
        # tmp_path has no analysis/ subdir
        centers = load_mesh_centers(robot, tmp_path)
        assert centers == {}


# ---------------------------------------------------------------------------
# load_flat_faces
# ---------------------------------------------------------------------------


class TestLoadFlatFaces:
    """Tests for load_flat_faces with real robot data."""

    def test_returns_faces_for_links_with_features(self, robot_dir: Path):
        robot = load_urdf(robot_dir / "robot.urdf")
        faces = load_flat_faces(robot, robot_dir)

        # At least some links should have flat faces
        assert isinstance(faces, dict)

        for _link_name, face_list in faces.items():
            assert isinstance(face_list, list)
            for ff in face_list:
                assert "normal" in ff
                assert "area_mm2" in ff
                assert "centroid" in ff

    def test_returns_empty_when_no_analysis_dir(self, tmp_path: Path):
        robot = load_urdf(
            Path(__file__).parent.parent / "robots" / "Meca500-R3" / "robot.urdf"
        )
        faces = load_flat_faces(robot, tmp_path)
        assert faces == {}


# ---------------------------------------------------------------------------
# load_connection_points
# ---------------------------------------------------------------------------


class TestLoadConnectionPoints:
    """Tests for load_connection_points with real robot data."""

    def test_returns_connection_points(self, robot_dir: Path):
        robot = load_urdf(robot_dir / "robot.urdf")
        cps = load_connection_points(robot, robot_dir)

        assert len(cps) > 0

        for _link_name, points in cps.items():
            assert isinstance(points, list)
            for cp in points:
                assert "end" in cp
                assert cp["end"] in ("proximal", "distal")
                assert "position" in cp
                assert isinstance(cp["position"], np.ndarray)
                assert cp["position"].shape == (3,)
                assert "axis" in cp
                assert "radius_mm" in cp
                assert isinstance(cp["radius_mm"], (int, float))
                assert "centering" in cp

    def test_returns_empty_when_no_analysis_dir(self, tmp_path: Path):
        robot = load_urdf(
            Path(__file__).parent.parent / "robots" / "Meca500-R3" / "robot.urdf"
        )
        cps = load_connection_points(robot, tmp_path)
        assert cps == {}
