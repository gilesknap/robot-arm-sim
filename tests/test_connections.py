"""Tests for connection point detection."""

from __future__ import annotations

import pytest
import trimesh

from conftest import make_cylinder_feature, make_flat_face_feature
from robot_arm_sim.analyze.connections import (
    _filter_barrel_groups,
    _group_cylinder_axes,
    detect_connection_points,
)

pytestmark = pytest.mark.filterwarnings(
    "ignore:DEPRECATED.*to_planar:DeprecationWarning"
)


# ---------------------------------------------------------------------------
# detect_connection_points
# ---------------------------------------------------------------------------


class TestDetectConnectionPoints:
    """Tests for detect_connection_points."""

    def test_cylinder_mesh_finds_two_points(
        self, cylinder_mesh: trimesh.Trimesh
    ) -> None:
        """Cylinder mesh with cylinder features should find proximal + distal."""
        features = [
            make_cylinder_feature(axis=[0, 0, 1], radius_mm=10.0, length_mm=50.0)
        ]
        points = detect_connection_points(cylinder_mesh, features, "test_cyl")
        assert len(points) == 2
        ends = {p.end for p in points}
        assert "proximal" in ends
        assert "distal" in ends

    def test_no_cylinders_falls_back_to_base(
        self, cylinder_mesh: trimesh.Trimesh
    ) -> None:
        """No cylindrical features -> base detection (distal only or fallback)."""
        # Use flat face features only
        features = [
            make_flat_face_feature(
                normal=[0, 0, 1], area_mm2=300.0, centroid=[0, 0, 25]
            ),
        ]
        points = detect_connection_points(cylinder_mesh, features, "base_part")
        # Should get at least one connection point from base detection
        assert len(points) >= 1

    def test_box_mesh_no_cylinders(self, box_mesh: trimesh.Trimesh) -> None:
        """Box with no cylinder features uses base fallback."""
        features = [
            make_flat_face_feature(
                normal=[0, 0, 1], area_mm2=400.0, centroid=[0, 0, 20]
            ),
        ]
        points = detect_connection_points(box_mesh, features, "box_part")
        assert len(points) >= 1

    def test_multi_axis_cylinders(self, l_shaped_mesh: trimesh.Trimesh) -> None:
        """L-shaped mesh with perpendicular cylinders triggers multi-axis path."""
        features = [
            make_cylinder_feature(axis=[0, 0, 1], radius_mm=8.0, length_mm=40.0),
            make_cylinder_feature(axis=[1, 0, 0], radius_mm=8.0, length_mm=30.0),
            make_flat_face_feature(
                normal=[0, 0, -1], area_mm2=200.0, centroid=[0, 0, -20]
            ),
            make_flat_face_feature(
                normal=[1, 0, 0], area_mm2=200.0, centroid=[30, 0, 20]
            ),
        ]
        points = detect_connection_points(l_shaped_mesh, features, "l_part")
        # Multi-axis should produce connection points
        assert len(points) >= 1

    def test_single_axis_group(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Multiple parallel cylinders reduce to single axis group."""
        features = [
            make_cylinder_feature(axis=[0, 0, 1], radius_mm=10.0, length_mm=50.0),
            make_cylinder_feature(axis=[0, 0, 1], radius_mm=5.0, length_mm=30.0),
        ]
        points = detect_connection_points(cylinder_mesh, features, "parallel_cyl")
        assert len(points) == 2

    def test_barrel_filter_path(self, cylinder_mesh: trimesh.Trimesh) -> None:
        """Two axis groups where one has 2x+ radius triggers barrel filter."""
        features = [
            make_cylinder_feature(axis=[0, 0, 1], radius_mm=10.0, length_mm=50.0),
            make_cylinder_feature(axis=[1, 0, 0], radius_mm=25.0, length_mm=30.0),
            make_flat_face_feature(
                normal=[1, 0, 0], area_mm2=500.0, centroid=[10, 0, 0]
            ),
        ]
        points = detect_connection_points(cylinder_mesh, features, "barrel_part")
        assert len(points) >= 1


# ---------------------------------------------------------------------------
# _group_cylinder_axes
# ---------------------------------------------------------------------------


class TestGroupCylinderAxes:
    """Tests for _group_cylinder_axes."""

    def test_parallel_cylinders_same_group(self) -> None:
        """Parallel cylinders (angle < threshold) should be in one group."""
        cyls = [
            make_cylinder_feature(axis=[0, 0, 1], length_mm=50.0),
            make_cylinder_feature(axis=[0, 0.05, 1], length_mm=30.0),  # nearly Z
        ]
        groups = _group_cylinder_axes(cyls, angle_threshold_deg=15.0)
        assert len(groups) == 1
        assert len(groups[0]) == 2

    def test_perpendicular_cylinders_split(self) -> None:
        """Perpendicular cylinders should be in separate groups."""
        cyls = [
            make_cylinder_feature(axis=[0, 0, 1], length_mm=50.0),
            make_cylinder_feature(axis=[1, 0, 0], length_mm=30.0),
        ]
        groups = _group_cylinder_axes(cyls, angle_threshold_deg=15.0)
        assert len(groups) == 2

    def test_sorted_by_total_length(self) -> None:
        """Groups should be sorted by total cylinder length (largest first)."""
        cyls = [
            make_cylinder_feature(axis=[1, 0, 0], length_mm=10.0),
            make_cylinder_feature(axis=[0, 0, 1], length_mm=50.0),
            make_cylinder_feature(axis=[0, 0, 1], length_mm=30.0),
        ]
        groups = _group_cylinder_axes(cyls, angle_threshold_deg=15.0)
        assert len(groups) == 2
        # Z-axis group (50+30=80) should be first
        assert len(groups[0]) == 2
        assert len(groups[1]) == 1

    def test_zero_axis_skipped(self) -> None:
        """Cylinder with zero axis should be skipped."""
        cyls = [
            make_cylinder_feature(axis=[0, 0, 0], length_mm=50.0),
            make_cylinder_feature(axis=[0, 0, 1], length_mm=30.0),
        ]
        groups = _group_cylinder_axes(cyls, angle_threshold_deg=15.0)
        assert len(groups) == 1

    def test_antiparallel_same_group(self) -> None:
        """Antiparallel axes ([0,0,1] and [0,0,-1]) should be in same group."""
        cyls = [
            make_cylinder_feature(axis=[0, 0, 1], length_mm=50.0),
            make_cylinder_feature(axis=[0, 0, -1], length_mm=30.0),
        ]
        groups = _group_cylinder_axes(cyls, angle_threshold_deg=15.0)
        assert len(groups) == 1

    def test_three_distinct_axes(self) -> None:
        """Three mutually perpendicular axes -> three groups."""
        cyls = [
            make_cylinder_feature(axis=[1, 0, 0], length_mm=20.0),
            make_cylinder_feature(axis=[0, 1, 0], length_mm=30.0),
            make_cylinder_feature(axis=[0, 0, 1], length_mm=40.0),
        ]
        groups = _group_cylinder_axes(cyls, angle_threshold_deg=15.0)
        assert len(groups) == 3


# ---------------------------------------------------------------------------
# _filter_barrel_groups
# ---------------------------------------------------------------------------


class TestFilterBarrelGroups:
    """Tests for _filter_barrel_groups."""

    def test_no_barrel_all_kept(self) -> None:
        """Groups with similar radii should all be kept."""
        groups = [
            [make_cylinder_feature(axis=[0, 0, 1], radius_mm=10.0, length_mm=50.0)],
            [make_cylinder_feature(axis=[1, 0, 0], radius_mm=12.0, length_mm=30.0)],
        ]
        kept, removed = _filter_barrel_groups(groups)
        assert len(kept) == 2
        assert len(removed) == 0

    def test_barrel_removed(self) -> None:
        """Group with 2x+ radius relative to smallest should be removed."""
        groups = [
            [make_cylinder_feature(axis=[0, 0, 1], radius_mm=10.0, length_mm=50.0)],
            [make_cylinder_feature(axis=[1, 0, 0], radius_mm=25.0, length_mm=30.0)],
        ]
        kept, removed = _filter_barrel_groups(groups)
        assert len(kept) == 1
        assert len(removed) == 1
        # The kept group should be the smaller-radius one
        assert kept[0][0].radius_mm == 10.0

    def test_all_barrel_returns_original(self) -> None:
        """If all groups would be removed, return all as kept."""
        # Single group — can't have all removed since min_radius matches
        groups = [
            [make_cylinder_feature(axis=[0, 0, 1], radius_mm=50.0, length_mm=50.0)],
        ]
        kept, removed = _filter_barrel_groups(groups)
        assert len(kept) == 1
        assert len(removed) == 0

    def test_zero_radius_all_kept(self) -> None:
        """Groups with None/0 radius should be kept (no barrel filter)."""
        groups = [
            [make_cylinder_feature(axis=[0, 0, 1], radius_mm=0.0, length_mm=50.0)],
            [make_cylinder_feature(axis=[1, 0, 0], radius_mm=0.0, length_mm=30.0)],
        ]
        kept, removed = _filter_barrel_groups(groups)
        assert len(kept) == 2
        assert len(removed) == 0

    def test_borderline_ratio_kept(self) -> None:
        """Radius exactly at 2x threshold should be kept (<=)."""
        groups = [
            [make_cylinder_feature(axis=[0, 0, 1], radius_mm=10.0, length_mm=50.0)],
            [make_cylinder_feature(axis=[1, 0, 0], radius_mm=20.0, length_mm=30.0)],
        ]
        kept, removed = _filter_barrel_groups(groups)
        # 20.0 <= 10.0 * 2.0, so both kept
        assert len(kept) == 2
        assert len(removed) == 0
