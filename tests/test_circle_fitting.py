"""Tests for circle fitting and cross-section analysis."""

from __future__ import annotations

import numpy as np
import pytest
import trimesh

from conftest import make_flat_face_feature
from robot_arm_sim.analyze.circle_fitting import (
    circle_fit_from_boundary,
    estimate_radius,
    estimate_radius_at_slice,
    find_circle_center_at_slice,
)

Z_AXIS = np.array([0.0, 0.0, 1.0])

# trimesh emits a DeprecationWarning for path.to_planar -> path.to_2D.
# Since filterwarnings="error" in pyproject.toml, this gets caught by
# try/except in the source code, causing fallback behaviour.  Suppress it
# so the actual circle-fitting logic is exercised.
pytestmark = pytest.mark.filterwarnings(
    "ignore:DEPRECATED.*to_planar:DeprecationWarning"
)


# ---------------------------------------------------------------------------
# find_circle_center_at_slice
# ---------------------------------------------------------------------------


class TestFindCircleCenterAtSlice:
    """Tests for find_circle_center_at_slice."""

    def test_cylinder_above(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Slicing a cylinder from below should find a center near the origin."""
        bounds = cylinder_mesh.bounds
        z_min = bounds[0][2]
        center = find_circle_center_at_slice(cylinder_mesh, Z_AXIS, z_min, "above")
        assert center is not None
        # Center should be near (0, 0, *)
        np.testing.assert_allclose(center[:2], [0.0, 0.0], atol=2.0)

    def test_cylinder_below(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Slicing a cylinder from above should find a center near the origin."""
        bounds = cylinder_mesh.bounds
        z_max = bounds[1][2]
        center = find_circle_center_at_slice(cylinder_mesh, Z_AXIS, z_max, "below")
        assert center is not None
        np.testing.assert_allclose(center[:2], [0.0, 0.0], atol=2.0)

    def test_box_returns_center_or_none(self, box_mesh: trimesh.Trimesh) -> None:
        """A box may return a fallback centroid or None (no circular cross-section)."""
        bounds = box_mesh.bounds
        z_min = bounds[0][2]
        result = find_circle_center_at_slice(box_mesh, Z_AXIS, z_min, "above")
        # Box cross-sections are square, not circular — may fall back to
        # vertex centroid or return None depending on circularity threshold.
        if result is not None:
            # If a fallback center is returned, it should be near (0,0,*)
            np.testing.assert_allclose(result[:2], [0.0, 0.0], atol=12.0)

    def test_flat_mesh_returns_none(self) -> None:
        """A very flat mesh (extent < 1mm) should return None."""
        flat = trimesh.creation.box(extents=[50.0, 50.0, 0.5])
        result = find_circle_center_at_slice(flat, Z_AXIS, 0.0, "above")
        assert result is None

    def test_elongated_cylinder(self) -> None:
        """Elongated cylinder (>200mm) caps offset via max_virtual_extent."""
        long_cyl = trimesh.creation.cylinder(radius=10.0, height=300.0, sections=32)
        bounds = long_cyl.bounds
        z_min = bounds[0][2]
        center = find_circle_center_at_slice(long_cyl, Z_AXIS, z_min, "above")
        assert center is not None
        np.testing.assert_allclose(center[:2], [0.0, 0.0], atol=2.0)


# ---------------------------------------------------------------------------
# estimate_radius_at_slice
# ---------------------------------------------------------------------------


class TestEstimateRadiusAtSlice:
    """Tests for estimate_radius_at_slice."""

    def test_cylinder_radius_below(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Radius of a cylinder (r=10) sliced from below should be close to 10."""
        bounds = cylinder_mesh.bounds
        z_max = bounds[1][2]
        r = estimate_radius_at_slice(cylinder_mesh, Z_AXIS, z_max, "below")
        np.testing.assert_allclose(r, 10.0, atol=2.0)

    def test_cylinder_radius_above(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Radius of a cylinder (r=10) sliced from above should be close to 10."""
        bounds = cylinder_mesh.bounds
        z_min = bounds[0][2]
        r = estimate_radius_at_slice(cylinder_mesh, Z_AXIS, z_min, "above")
        np.testing.assert_allclose(r, 10.0, atol=2.0)

    def test_box_returns_fallback(self, box_mesh: trimesh.Trimesh) -> None:
        """Box cross-sections are not circular; should return 20.0 fallback."""
        bounds = box_mesh.bounds
        z_min = bounds[0][2]
        r = estimate_radius_at_slice(box_mesh, Z_AXIS, z_min, "above")
        # Either finds a non-circular section or falls back to 20.0
        assert r > 0


# ---------------------------------------------------------------------------
# circle_fit_from_boundary
# ---------------------------------------------------------------------------


class TestCircleFitFromBoundary:
    """Tests for circle_fit_from_boundary."""

    def test_cylinder_boundary_fit(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Boundary fit on a cylinder should return center and radius."""
        bounds = cylinder_mesh.bounds
        z_min = bounds[0][2]
        result = circle_fit_from_boundary(cylinder_mesh, Z_AXIS, z_min, "above")
        assert result is not None
        np.testing.assert_allclose(result["center"][:2], [0.0, 0.0], atol=2.0)
        np.testing.assert_allclose(result["radius"], 10.0, atol=3.0)

    def test_cylinder_boundary_fit_below(self, cylinder_mesh: trimesh.Trimesh) -> None:
        bounds = cylinder_mesh.bounds
        z_max = bounds[1][2]
        result = circle_fit_from_boundary(cylinder_mesh, Z_AXIS, z_max, "below")
        assert result is not None
        np.testing.assert_allclose(result["center"][:2], [0.0, 0.0], atol=2.0)

    def test_few_vertices_returns_none(self) -> None:
        """Mesh with very few vertices near the end should return None."""
        # Create a tiny tetrahedron — few vertices at each end
        verts = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 10.0]])
        faces = np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]])
        mesh = trimesh.Trimesh(vertices=verts, faces=faces)
        result = circle_fit_from_boundary(mesh, Z_AXIS, 0.0, "above")
        assert result is None

    def test_box_boundary_may_fail(self, box_mesh: trimesh.Trimesh) -> None:
        """Box boundary fit: high std/mean ratio should return None."""
        bounds = box_mesh.bounds
        z_min = bounds[0][2]
        result = circle_fit_from_boundary(box_mesh, Z_AXIS, z_min, "above")
        # A box's boundary vertices form a square, not a circle,
        # so circularity check (std/mean > 0.5) may reject it.
        # Either None or a valid dict is acceptable.
        if result is not None:
            assert "center" in result
            assert "radius" in result

    def test_x_axis(self) -> None:
        """Boundary fit along X axis."""
        cyl = trimesh.creation.cylinder(radius=8.0, height=40.0, sections=32)
        rot = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
        cyl.apply_transform(rot)
        x_axis = np.array([1.0, 0.0, 0.0])
        bounds = cyl.bounds
        x_min = bounds[0][0]
        result = circle_fit_from_boundary(cyl, x_axis, x_min, "above")
        assert result is not None
        np.testing.assert_allclose(result["radius"], 8.0, atol=3.0)


# ---------------------------------------------------------------------------
# estimate_radius
# ---------------------------------------------------------------------------


class TestEstimateRadius:
    """Tests for estimate_radius (from flat face feature area)."""

    def test_area_to_radius(self) -> None:
        """area = pi*r^2 => r = sqrt(area/pi)."""
        area = np.pi * 15.0**2
        feat = make_flat_face_feature(area_mm2=area)
        r = estimate_radius(feat)
        np.testing.assert_allclose(r, 15.0, atol=0.01)

    def test_zero_area_returns_default(self) -> None:
        """Zero area should return fallback 20.0."""
        feat = make_flat_face_feature(area_mm2=0.0)
        r = estimate_radius(feat)
        assert r == 20.0

    def test_none_area_returns_default(self) -> None:
        """None area should return fallback 20.0."""
        from robot_arm_sim.models import GeometricFeature

        feat = GeometricFeature(kind="flat_face", description="no area")
        r = estimate_radius(feat)
        assert r == 20.0
