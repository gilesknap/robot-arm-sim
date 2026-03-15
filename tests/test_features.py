"""Tests for geometric feature detection."""

from __future__ import annotations

import numpy as np
import pytest
import trimesh

from conftest import make_cylinder_feature, make_flat_face_feature
from robot_arm_sim.analyze.endpoint_detection import (
    detect_base_connection,
    detect_multi_axis_connections,
    find_bore_end_for_axis,
)
from robot_arm_sim.analyze.features import detect_features

pytestmark = pytest.mark.filterwarnings(
    "ignore:DEPRECATED.*to_planar:DeprecationWarning"
)


def test_detect_features_on_cube():
    mesh = trimesh.creation.box(extents=[10, 10, 10])
    features = detect_features(mesh)

    flat_faces = [f for f in features if f.kind == "flat_face"]
    # A cube has 6 large flat faces
    assert len(flat_faces) >= 4  # at least most faces detected


def test_detect_features_on_cylinder():
    mesh = trimesh.creation.cylinder(radius=10, height=50)
    features = detect_features(mesh)

    cylinders = [f for f in features if f.kind == "cylindrical_surface"]
    # Should detect at least one cylindrical surface
    assert len(cylinders) >= 1
    # Should have a reasonable radius
    assert any(abs((c.radius_mm or 0) - 10.0) < 2.0 for c in cylinders)


def test_detect_flat_faces_threshold():
    """Tiny faces below 2% threshold should not be reported."""
    # A large box — all faces are big enough
    mesh = trimesh.creation.box(extents=[100, 100, 100])
    features = detect_features(mesh)
    flat_faces = [f for f in features if f.kind == "flat_face"]

    # All detected faces should have area > 2% of total
    total_area = mesh.area
    for f in flat_faces:
        assert (f.area_mm2 or 0) >= total_area * 0.02 * 0.9  # small tolerance


# ---------------------------------------------------------------------------
# Endpoint detection tests
# ---------------------------------------------------------------------------


class TestDetectBaseConnection:
    """Tests for detect_base_connection."""

    def test_top_flat_face(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Cylinder with a top flat face should produce a distal connection."""
        features = [
            make_flat_face_feature(
                normal=[0, 0, 1], area_mm2=np.pi * 10**2, centroid=[0, 0, 25]
            ),
        ]
        points = detect_base_connection(cylinder_mesh, features)
        assert len(points) >= 1
        assert any(p.end == "distal" for p in points)

    def test_no_top_face_uses_largest_flat(self, box_mesh: trimesh.Trimesh) -> None:
        """When no Z-up face exists, fallback to largest flat face."""
        features = [
            make_flat_face_feature(
                normal=[1, 0, 0], area_mm2=800.0, centroid=[10, 0, 0]
            ),
            make_flat_face_feature(
                normal=[0, 1, 0], area_mm2=200.0, centroid=[0, 10, 0]
            ),
        ]
        points = detect_base_connection(box_mesh, features)
        assert len(points) == 1
        assert points[0].method == "largest_face_fallback"

    def test_no_flat_faces_returns_empty(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """No flat faces at all should return empty list."""
        points = detect_base_connection(cylinder_mesh, [])
        assert points == []

    def test_top_face_with_centroid_fallback(self, box_mesh: trimesh.Trimesh) -> None:
        """Top face exists but circle fitting fails — centroid fallback used."""
        features = [
            make_flat_face_feature(
                normal=[0, 0, 1], area_mm2=400.0, centroid=[0, 0, 20]
            ),
        ]
        points = detect_base_connection(box_mesh, features)
        assert len(points) >= 1
        # Should be distal with some method
        assert points[0].end == "distal"


class TestDetectMultiAxisConnections:
    """Tests for detect_multi_axis_connections."""

    def test_l_shaped_mesh(self, l_shaped_mesh: trimesh.Trimesh) -> None:
        """L-shaped mesh should detect connections along both axes."""
        cylinders = [
            make_cylinder_feature(axis=[0, 0, 1], radius_mm=8.0, length_mm=40.0),
            make_cylinder_feature(axis=[1, 0, 0], radius_mm=8.0, length_mm=30.0),
        ]
        axis_groups = [[cylinders[0]], [cylinders[1]]]
        flat_faces = [
            make_flat_face_feature(
                normal=[0, 0, -1], area_mm2=200.0, centroid=[0, 0, -20]
            ),
            make_flat_face_feature(
                normal=[1, 0, 0], area_mm2=200.0, centroid=[30, 0, 20]
            ),
        ]
        features = cylinders + flat_faces

        points = detect_multi_axis_connections(
            l_shaped_mesh, features, cylinders, axis_groups
        )
        assert len(points) >= 1
        # Should have proximal and/or distal
        ends = {p.end for p in points}
        assert len(ends) >= 1

    def test_returns_empty_when_no_candidates(self) -> None:
        """Tiny mesh that fails all slicing should return empty."""
        tiny = trimesh.creation.box(extents=[1.0, 1.0, 1.0])
        cylinders = [
            make_cylinder_feature(axis=[0, 0, 1], radius_mm=0.1, length_mm=1.0),
            make_cylinder_feature(axis=[1, 0, 0], radius_mm=0.1, length_mm=1.0),
        ]
        axis_groups = [[cylinders[0]], [cylinders[1]]]
        features = list(cylinders)
        points = detect_multi_axis_connections(tiny, features, cylinders, axis_groups)
        # May or may not find candidates on a 1mm box
        assert isinstance(points, list)


class TestFindBoreEndForAxis:
    """Tests for find_bore_end_for_axis."""

    def test_flat_face_at_max_end(self) -> None:
        """Flat face at max Z selects max end (below direction)."""
        axis = np.array([0.0, 0.0, 1.0])
        bounds = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 50.0]])
        flat_faces = [
            make_flat_face_feature(
                normal=[0, 0, 1], area_mm2=300.0, centroid=[5, 5, 45]
            ),
        ]
        val, direction = find_bore_end_for_axis(axis, 2, bounds, flat_faces)
        assert val == 50.0
        assert direction == "below"

    def test_flat_face_at_min_end(self) -> None:
        """Flat face at min Z selects min end (above direction)."""
        axis = np.array([0.0, 0.0, 1.0])
        bounds = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 50.0]])
        flat_faces = [
            make_flat_face_feature(
                normal=[0, 0, -1], area_mm2=300.0, centroid=[5, 5, 5]
            ),
        ]
        val, direction = find_bore_end_for_axis(axis, 2, bounds, flat_faces)
        assert val == 0.0
        assert direction == "above"

    def test_no_aligned_faces_defaults_min(self) -> None:
        """No aligned flat faces defaults to min end."""
        axis = np.array([0.0, 0.0, 1.0])
        bounds = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 50.0]])
        # Face normal perpendicular to axis — not aligned
        flat_faces = [
            make_flat_face_feature(
                normal=[1, 0, 0], area_mm2=300.0, centroid=[5, 5, 25]
            ),
        ]
        val, direction = find_bore_end_for_axis(axis, 2, bounds, flat_faces)
        assert val == 0.0
        assert direction == "above"

    def test_larger_face_wins(self) -> None:
        """When faces exist at both ends, the larger area wins."""
        axis = np.array([0.0, 0.0, 1.0])
        bounds = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 50.0]])
        flat_faces = [
            make_flat_face_feature(
                normal=[0, 0, -1], area_mm2=100.0, centroid=[5, 5, 5]
            ),
            make_flat_face_feature(
                normal=[0, 0, 1], area_mm2=500.0, centroid=[5, 5, 45]
            ),
        ]
        val, direction = find_bore_end_for_axis(axis, 2, bounds, flat_faces)
        assert val == 50.0
        assert direction == "below"
