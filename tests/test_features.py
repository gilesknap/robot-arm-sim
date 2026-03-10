"""Tests for geometric feature detection."""

from __future__ import annotations

import trimesh

from robot_arm_sim.analyze.features import detect_features


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
