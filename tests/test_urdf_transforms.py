"""Tests for urdf_transforms module — pure logic functions."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from conftest import make_connection_point, make_flat_face_feature
from robot_arm_sim.analyze.urdf_transforms import (
    _find_opposite_face,
    auto_detect_visual_flips,
    close_surface_gaps_along_axis,
    compute_joint_origin,
    compute_visual_origin,
    rotation_matrix_to_rpy,
    rpy_to_rotation,
    update_derived_joint_origins,
    validate_fk,
)

# ---------------------------------------------------------------------------
# rotation_matrix_to_rpy
# ---------------------------------------------------------------------------


class TestRotationMatrixToRpy:
    def test_identity(self):
        rpy = rotation_matrix_to_rpy(np.eye(3))
        np.testing.assert_allclose(rpy, [0, 0, 0], atol=1e-10)

    def test_90deg_roll(self):
        """Rotation of pi/2 around X axis."""
        rpy_in = [np.pi / 2, 0, 0]
        rot = rpy_to_rotation(rpy_in)
        rpy_out = rotation_matrix_to_rpy(rot)
        np.testing.assert_allclose(rpy_out, rpy_in, atol=1e-10)

    def test_90deg_pitch(self):
        """Rotation of pi/2 around Y axis."""
        rpy_in = [0, np.pi / 4, 0]
        rot = rpy_to_rotation(rpy_in)
        rpy_out = rotation_matrix_to_rpy(rot)
        np.testing.assert_allclose(rpy_out, rpy_in, atol=1e-10)

    def test_90deg_yaw(self):
        """Rotation of pi/2 around Z axis."""
        rpy_in = [0, 0, np.pi / 2]
        rot = rpy_to_rotation(rpy_in)
        rpy_out = rotation_matrix_to_rpy(rot)
        np.testing.assert_allclose(rpy_out, rpy_in, atol=1e-10)

    def test_combined_rotation(self):
        """Combined roll-pitch-yaw roundtrips correctly."""
        rpy_in = [0.3, 0.5, 0.7]
        rot = rpy_to_rotation(rpy_in)
        rpy_out = rotation_matrix_to_rpy(rot)
        np.testing.assert_allclose(rpy_out, rpy_in, atol=1e-10)

    def test_gimbal_lock_positive(self):
        """Gimbal lock at pitch = +pi/2."""
        rpy_in = [0.4, np.pi / 2, 0.0]
        rot = rpy_to_rotation(rpy_in)
        rpy_out = rotation_matrix_to_rpy(rot)
        # At gimbal lock, pitch should be pi/2 and yaw is 0
        assert abs(rpy_out[1] - np.pi / 2) < 1e-6
        assert abs(rpy_out[2]) < 1e-6
        # The rotation matrices should match even if RPY decomposition differs
        rot_roundtrip = rpy_to_rotation(rpy_out)
        np.testing.assert_allclose(rot_roundtrip, rot, atol=1e-6)

    def test_gimbal_lock_negative(self):
        """Gimbal lock at pitch = -pi/2."""
        rpy_in = [0.0, -np.pi / 2, 0.3]
        rot = rpy_to_rotation(rpy_in)
        rpy_out = rotation_matrix_to_rpy(rot)
        assert abs(rpy_out[1] + np.pi / 2) < 1e-6
        rot_roundtrip = rpy_to_rotation(rpy_out)
        np.testing.assert_allclose(rot_roundtrip, rot, atol=1e-6)

    def test_roundtrip_random(self):
        """Several random RPY values roundtrip via matrix."""
        rng = np.random.default_rng(42)
        for _ in range(20):
            rpy_in = (rng.random(3) - 0.5) * np.array([np.pi, np.pi / 2, np.pi])
            rot = rpy_to_rotation(rpy_in.tolist())
            rpy_out = rotation_matrix_to_rpy(rot)
            rot2 = rpy_to_rotation(rpy_out)
            np.testing.assert_allclose(rot2, rot, atol=1e-10)


# ---------------------------------------------------------------------------
# rpy_to_rotation
# ---------------------------------------------------------------------------


class TestRpyToRotation:
    def test_identity(self):
        rot = rpy_to_rotation([0, 0, 0])
        np.testing.assert_allclose(rot, np.eye(3), atol=1e-10)

    def test_pure_roll(self):
        """Pure roll (rotation about X) — Y→Z, Z→-Y for pi/2."""
        rot = rpy_to_rotation([np.pi / 2, 0, 0])
        expected = np.array(
            [
                [1, 0, 0],
                [0, 0, -1],
                [0, 1, 0],
            ],
            dtype=float,
        )
        np.testing.assert_allclose(rot, expected, atol=1e-10)

    def test_pure_pitch(self):
        """Pure pitch (rotation about Y) — X→Z, Z→-X for pi/2."""
        rot = rpy_to_rotation([0, np.pi / 2, 0])
        expected = np.array(
            [
                [0, 0, 1],
                [0, 1, 0],
                [-1, 0, 0],
            ],
            dtype=float,
        )
        np.testing.assert_allclose(rot, expected, atol=1e-10)

    def test_pure_yaw(self):
        """Pure yaw (rotation about Z) — X→Y, Y→-X for pi/2."""
        rot = rpy_to_rotation([0, 0, np.pi / 2])
        expected = np.array(
            [
                [0, 0, 0],  # placeholder
                [0, 0, 0],
                [0, 0, 1],
            ],
            dtype=float,
        )
        # More carefully:
        expected = np.array(
            [
                [0, -1, 0],
                [1, 0, 0],
                [0, 0, 1],
            ],
            dtype=float,
        )
        np.testing.assert_allclose(rot, expected, atol=1e-10)

    def test_orthogonal(self):
        """Rotation matrix is always orthogonal (R^T R = I)."""
        rot = rpy_to_rotation([0.5, 0.3, 0.7])
        np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-10)

    def test_determinant_one(self):
        """Rotation matrix has determinant 1."""
        rot = rpy_to_rotation([1.2, -0.4, 0.8])
        np.testing.assert_allclose(np.linalg.det(rot), 1.0, atol=1e-10)


# ---------------------------------------------------------------------------
# compute_visual_origin
# ---------------------------------------------------------------------------


class TestComputeVisualOrigin:
    def _make_analysis(
        self,
        prox_pos=None,
        dist_pos=None,
        prox_axis=None,
        centering="surface",
        flat_faces=None,
    ):
        """Build a minimal analysis dict for testing."""
        cps = []
        if prox_pos is not None:
            cp = make_connection_point(
                end="proximal",
                position=prox_pos,
                axis=prox_axis or [0, 0, 1],
            )
            d = cp.model_dump(exclude_none=True)
            d["centering"] = centering
            cps.append(d)
        if dist_pos is not None:
            cp = make_connection_point(end="distal", position=dist_pos)
            cps.append(cp.model_dump(exclude_none=True))
        features = {}
        if flat_faces is not None:
            features["flat_faces"] = [
                f.model_dump(exclude_none=True) for f in flat_faces
            ]
        return {"connection_points": cps, "features": features}

    def test_basic_proximal_snap(self):
        """Visual origin negates the proximal position (mm -> m)."""
        analysis = self._make_analysis(prox_pos=[10.0, 20.0, 30.0])
        link_spec: dict = {}
        msgs: list[str] = []
        xyz, rpy = compute_visual_origin(analysis, link_spec, "link_1", messages=msgs)
        np.testing.assert_allclose(xyz, [-0.01, -0.02, -0.03], atol=1e-6)
        assert rpy == [0, 0, 0]
        assert len(msgs) == 1

    def test_no_connection_points(self):
        """Returns zeros when no connection points exist."""
        analysis: dict = {"connection_points": [], "features": {}}
        msgs: list[str] = []
        xyz, rpy = compute_visual_origin(analysis, {}, "link_1", messages=msgs)
        assert xyz == [0, 0, 0]
        assert rpy == [0, 0, 0]
        assert any("no proximal" in m for m in msgs)

    def test_visual_rpy_from_link_spec(self):
        """When link_spec has visual_rpy, that RPY is returned and affects xyz."""
        analysis = self._make_analysis(prox_pos=[100.0, 0.0, 0.0])
        link_spec = {"visual_rpy": [0, 0, np.pi / 2]}
        msgs: list[str] = []
        xyz, rpy = compute_visual_origin(analysis, link_spec, "link_1", messages=msgs)
        assert rpy == [0, 0, np.pi / 2]
        # xyz should be rotated: -(R @ pos/1000)
        rot = rpy_to_rotation([0, 0, np.pi / 2])
        expected = (-rot @ np.array([0.1, 0.0, 0.0])).tolist()
        np.testing.assert_allclose(xyz, expected, atol=1e-6)

    def test_center_mode_with_opposite_face(self):
        """Center mode averages proximal with opposite face along axis."""
        # Proximal at z=0, opposite face at z=50
        flat = make_flat_face_feature(
            normal=[0, 0, -1],
            centroid=[0.0, 0.0, 50.0],
        )
        analysis = self._make_analysis(
            prox_pos=[0.0, 0.0, 0.0],
            centering="center",
            flat_faces=[flat],
        )
        msgs: list[str] = []
        xyz, _ = compute_visual_origin(analysis, {}, "link_1", messages=msgs)
        # Averaged z = (0 + 50) / 2 = 25mm => visual origin z = -0.025
        np.testing.assert_allclose(xyz[2], -0.025, atol=1e-6)

    def test_distal_anchor(self):
        """When _visual_anchor is 'distal', uses distal connection point."""
        analysis = self._make_analysis(
            prox_pos=[0.0, 0.0, 0.0],
            dist_pos=[0.0, 0.0, 100.0],
        )
        link_spec = {"_visual_anchor": "distal"}
        msgs: list[str] = []
        xyz, _ = compute_visual_origin(analysis, link_spec, "link_1", messages=msgs)
        np.testing.assert_allclose(xyz, [0.0, 0.0, -0.1], atol=1e-6)


# ---------------------------------------------------------------------------
# compute_joint_origin
# ---------------------------------------------------------------------------


class TestComputeJointOrigin:
    def test_explicit_origin(self):
        """When origin is explicit in joint_spec, returns it directly."""
        joint_spec = {"name": "j1", "parent": "base", "origin": [0.1, 0.2, 0.3]}
        msgs: list[str] = []
        result = compute_joint_origin(joint_spec, {}, {}, {}, msgs)
        assert result == [0.1, 0.2, 0.3]

    def test_from_distal_cp_with_proximal(self):
        """Computes origin from distal - proximal when both exist."""
        prox = make_connection_point("proximal", [10, 0, 0]).model_dump(
            exclude_none=True
        )
        dist = make_connection_point("distal", [10, 0, 80]).model_dump(
            exclude_none=True
        )
        analysis = {"connection_points": [prox, dist]}
        link_specs = {"base": {"mesh": "m1"}}
        joint_spec = {"name": "j1", "parent": "base"}
        msgs: list[str] = []
        result = compute_joint_origin(
            joint_spec, link_specs, {"m1": analysis}, {}, msgs
        )
        # (distal - proximal) / 1000 = (0, 0, 80) / 1000 = (0, 0, 0.08)
        np.testing.assert_allclose(result, [0.0, 0.0, 0.08], atol=1e-6)

    def test_from_distal_cp_without_proximal(self):
        """Uses absolute distal position when no proximal exists."""
        dist = make_connection_point("distal", [0, 0, 50]).model_dump(exclude_none=True)
        analysis = {"connection_points": [dist]}
        link_specs = {"base": {"mesh": "m1"}}
        joint_spec = {"name": "j1", "parent": "base"}
        msgs: list[str] = []
        result = compute_joint_origin(
            joint_spec, link_specs, {"m1": analysis}, {}, msgs
        )
        np.testing.assert_allclose(result, [0.0, 0.0, 0.05], atol=1e-6)

    def test_fallback_no_data(self):
        """Falls back to joint_spec origin or [0,0,0] when no analysis data."""
        joint_spec = {"name": "j1", "parent": "base"}
        msgs: list[str] = []
        result = compute_joint_origin(joint_spec, {}, {}, {}, msgs)
        assert result == [0, 0, 0]
        assert any("fallback" in m for m in msgs)

    def test_parent_visual_rpy_rotation(self):
        """When parent has visual_rpy, joint origin is rotated."""
        prox = make_connection_point("proximal", [0, 0, 0]).model_dump(
            exclude_none=True
        )
        dist = make_connection_point("distal", [100, 0, 0]).model_dump(
            exclude_none=True
        )
        analysis = {"connection_points": [prox, dist]}
        link_specs = {
            "base": {"mesh": "m1", "visual_rpy": [0, 0, np.pi / 2]},
        }
        joint_spec = {"name": "j1", "parent": "base"}
        msgs: list[str] = []
        result = compute_joint_origin(
            joint_spec, link_specs, {"m1": analysis}, {}, msgs
        )
        # xyz = (100, 0, 0)/1000 = (0.1, 0, 0), rotated by yaw=pi/2
        rot = rpy_to_rotation([0, 0, np.pi / 2])
        expected = (rot @ np.array([0.1, 0.0, 0.0])).tolist()
        np.testing.assert_allclose(result, expected, atol=1e-6)


# ---------------------------------------------------------------------------
# auto_detect_visual_flips
# ---------------------------------------------------------------------------


class TestAutoDetectVisualFlips:
    def _make_chain_and_analyses(self, prox_pos, dist_pos, next_origin):
        """Build a minimal 3-link chain (base -> child -> tip) for flip testing."""
        chain = {
            "links": [
                {"name": "base", "mesh": "m_base"},
                {"name": "child", "mesh": "m_child"},
                {"name": "tip", "mesh": "m_tip"},
            ],
            "joints": [
                {
                    "name": "j1",
                    "parent": "base",
                    "child": "child",
                    "origin_rpy": [np.pi, 0, 0],  # flip
                    "origin": [0, 0, 0.1],
                },
                {
                    "name": "j2",
                    "parent": "child",
                    "child": "tip",
                    "origin_rpy": [0, 0, 0],
                    "origin": next_origin,
                },
            ],
        }
        prox = make_connection_point("proximal", prox_pos).model_dump(exclude_none=True)
        dist = make_connection_point("distal", dist_pos).model_dump(exclude_none=True)
        analyses = {"m_child": {"connection_points": [prox, dist]}}
        return chain, analyses

    def test_no_flip_when_aligned(self):
        """No flip when mesh direction matches next joint origin direction."""
        chain, analyses = self._make_chain_and_analyses(
            prox_pos=[0, 0, 0],
            dist_pos=[0, 0, 100],  # extends +Z
            next_origin=[0, 0, 0.1],  # also +Z
        )
        msgs: list[str] = []
        auto_detect_visual_flips(chain, analyses, msgs)
        child_spec = chain["links"][1]
        assert "_visual_anchor" not in child_spec

    def test_flip_when_opposed(self):
        """Sets _visual_anchor to distal when mesh opposes next joint direction."""
        chain, analyses = self._make_chain_and_analyses(
            prox_pos=[0, 0, 100],
            dist_pos=[0, 0, 0],  # extends -Z (proximal > distal)
            next_origin=[0, 0, 0.1],  # +Z
        )
        msgs: list[str] = []
        auto_detect_visual_flips(chain, analyses, msgs)
        child_spec = chain["links"][1]
        assert child_spec.get("_visual_anchor") == "distal"
        assert any("frame-flip" in m for m in msgs)

    def test_skip_explicit_visual_rpy(self):
        """Does not modify links that already have visual_rpy set."""
        chain, analyses = self._make_chain_and_analyses(
            prox_pos=[0, 0, 100],
            dist_pos=[0, 0, 0],
            next_origin=[0, 0, 0.1],
        )
        chain["links"][1]["visual_rpy"] = [0, 0, 0]
        msgs: list[str] = []
        auto_detect_visual_flips(chain, analyses, msgs)
        child_spec = chain["links"][1]
        assert "_visual_anchor" not in child_spec

    def test_skip_small_mesh(self):
        """Skips detection when proximal-distal distance < 10mm."""
        chain, analyses = self._make_chain_and_analyses(
            prox_pos=[0, 0, 5],
            dist_pos=[0, 0, 0],
            next_origin=[0, 0, 0.1],
        )
        msgs: list[str] = []
        auto_detect_visual_flips(chain, analyses, msgs)
        child_spec = chain["links"][1]
        assert "_visual_anchor" not in child_spec


# ---------------------------------------------------------------------------
# close_surface_gaps_along_axis
# ---------------------------------------------------------------------------


class TestCloseSurfaceGapsAlongAxis:
    def test_shifts_child_visual_along_axis(self):
        """Surface-mode child gets shifted along joint axis to close gap."""
        chain = {
            "links": [
                {"name": "base", "mesh": "m_base"},
                {"name": "child", "mesh": "m_child"},
            ],
            "joints": [
                {
                    "name": "j1",
                    "parent": "base",
                    "child": "child",
                    "axis": [0, 0, 1],
                },
            ],
        }
        # Parent: distal at z=50mm, visual origin at (0, 0, 0)
        prox_p = make_connection_point("proximal", [0, 0, 0]).model_dump(
            exclude_none=True
        )
        dist_p = make_connection_point("distal", [0, 0, 50]).model_dump(
            exclude_none=True
        )
        # Child: proximal surface mode
        prox_c = make_connection_point("proximal", [0, 0, 0]).model_dump(
            exclude_none=True
        )
        prox_c["centering"] = "surface"

        analyses = {
            "m_base": {"connection_points": [prox_p, dist_p]},
            "m_child": {"connection_points": [prox_c]},
        }
        visual_origins = {
            "base": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            "child": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        }
        joint_origins = {"j1": [0.0, 0.0, 0.05]}
        joint_rpys: dict[str, list[float]] = {"j1": [0.0, 0.0, 0.0]}
        msgs: list[str] = []

        close_surface_gaps_along_axis(
            chain, analyses, visual_origins, joint_origins, joint_rpys, msgs
        )

        # distal_in_parent = visual_xyz + distal*0.001 = (0,0,0) + (0,0,0.05)
        # gap = distal_in_parent - jnt_xyz = (0,0,0.05) - (0,0,0.05) = 0
        # No shift expected when gap is zero
        child_xyz, _ = visual_origins["child"]
        np.testing.assert_allclose(child_xyz, [0.0, 0.0, 0.0], atol=1e-6)

    def test_gap_shift_with_offset(self):
        """Child visual shifts when gap exists between parent distal and joint."""
        chain = {
            "links": [
                {"name": "base", "mesh": "m_base"},
                {"name": "child", "mesh": "m_child"},
            ],
            "joints": [
                {
                    "name": "j1",
                    "parent": "base",
                    "child": "child",
                    "axis": [0, 0, 1],
                },
            ],
        }
        prox_p = make_connection_point("proximal", [0, 0, 0]).model_dump(
            exclude_none=True
        )
        dist_p = make_connection_point("distal", [0, 0, 50]).model_dump(
            exclude_none=True
        )
        prox_c = make_connection_point("proximal", [0, 0, 0]).model_dump(
            exclude_none=True
        )
        prox_c["centering"] = "surface"

        analyses = {
            "m_base": {"connection_points": [prox_p, dist_p]},
            "m_child": {"connection_points": [prox_c]},
        }
        visual_origins = {
            "base": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            "child": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        }
        # Joint origin offset from parent distal creates a 10mm gap
        joint_origins = {"j1": [0.0, 0.0, 0.04]}
        joint_rpys: dict[str, list[float]] = {"j1": [0.0, 0.0, 0.0]}
        msgs: list[str] = []

        close_surface_gaps_along_axis(
            chain, analyses, visual_origins, joint_origins, joint_rpys, msgs
        )

        child_xyz, _ = visual_origins["child"]
        # gap = distal_in_parent - jnt_xyz = (0,0,0.05) - (0,0,0.04) = (0,0,0.01)
        # along = 0.01, shift = [0, 0, 0.01]
        np.testing.assert_allclose(child_xyz[2], 0.01, atol=1e-6)

    def test_skip_center_mode(self):
        """Center-mode children are not shifted."""
        chain = {
            "links": [
                {"name": "base", "mesh": "m_base"},
                {"name": "child", "mesh": "m_child"},
            ],
            "joints": [
                {
                    "name": "j1",
                    "parent": "base",
                    "child": "child",
                    "axis": [0, 0, 1],
                },
            ],
        }
        prox_p = make_connection_point("proximal", [0, 0, 0]).model_dump(
            exclude_none=True
        )
        dist_p = make_connection_point("distal", [0, 0, 50]).model_dump(
            exclude_none=True
        )
        prox_c = make_connection_point("proximal", [0, 0, 0]).model_dump(
            exclude_none=True
        )
        prox_c["centering"] = "center"

        analyses = {
            "m_base": {"connection_points": [prox_p, dist_p]},
            "m_child": {"connection_points": [prox_c]},
        }
        visual_origins = {
            "base": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            "child": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        }
        joint_origins = {"j1": [0.0, 0.0, 0.04]}
        joint_rpys: dict[str, list[float]] = {"j1": [0.0, 0.0, 0.0]}
        msgs: list[str] = []

        close_surface_gaps_along_axis(
            chain, analyses, visual_origins, joint_origins, joint_rpys, msgs
        )

        child_xyz, _ = visual_origins["child"]
        assert child_xyz == [0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# update_derived_joint_origins
# ---------------------------------------------------------------------------


class TestUpdateDerivedJointOrigins:
    def test_updates_non_explicit_origin(self):
        """Joint origin is recomputed from shifted visual origin + distal CP."""
        chain = {
            "links": [
                {"name": "base", "mesh": "m_base"},
                {"name": "child", "mesh": "m_child"},
            ],
            "joints": [
                {
                    "name": "j1",
                    "parent": "base",
                    "child": "child",
                    # No "origin" key — this is a derived joint
                },
            ],
        }
        dist = make_connection_point("distal", [0, 0, 80]).model_dump(exclude_none=True)
        analyses = {"m_base": {"connection_points": [dist]}}
        # Parent visual was shifted to (0, 0, 0.01) by gap closing
        visual_origins = {
            "base": ([0.0, 0.0, 0.01], [0.0, 0.0, 0.0]),
            "child": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        }
        joint_origins = {"j1": [0.0, 0.0, 0.07]}
        msgs: list[str] = []

        update_derived_joint_origins(
            chain, analyses, visual_origins, joint_origins, msgs
        )

        # new_origin = parent_viz_xyz + distal * 0.001
        # = (0, 0, 0.01) + (0, 0, 0.08) = (0, 0, 0.09)
        np.testing.assert_allclose(joint_origins["j1"], [0, 0, 0.09], atol=1e-6)

    def test_skips_explicit_origin(self):
        """Joints with explicit origin are never modified."""
        chain = {
            "links": [
                {"name": "base", "mesh": "m_base"},
                {"name": "child"},
            ],
            "joints": [
                {
                    "name": "j1",
                    "parent": "base",
                    "child": "child",
                    "origin": [0, 0, 0.1],  # explicit
                },
            ],
        }
        dist = make_connection_point("distal", [0, 0, 80]).model_dump(exclude_none=True)
        analyses = {"m_base": {"connection_points": [dist]}}
        visual_origins = {"base": ([0.0, 0.0, 0.01], [0.0, 0.0, 0.0])}
        joint_origins = {"j1": [0.0, 0.0, 0.1]}
        msgs: list[str] = []

        update_derived_joint_origins(
            chain, analyses, visual_origins, joint_origins, msgs
        )

        # Should remain unchanged
        assert joint_origins["j1"] == [0.0, 0.0, 0.1]


# ---------------------------------------------------------------------------
# _find_opposite_face
# ---------------------------------------------------------------------------


class TestFindOppositeFace:
    def test_basic_opposite_face(self):
        """Finds the z-coordinate of the face opposing the bore axis."""
        analysis = {
            "features": {
                "flat_faces": [
                    {"normal": [0, 0, -1], "centroid": [0, 0, 50]},
                    {"normal": [0, 0, 1], "centroid": [0, 0, 0]},  # same dir, ignored
                ],
            },
        }
        result = _find_opposite_face(
            analysis,
            bore_pos=[0, 0, 0],
            bore_axis=[0, 0, 1],
            axis_idx=2,
        )
        assert result == 50

    def test_max_depth_rejects_distant(self):
        """Face beyond max_depth is rejected."""
        analysis = {
            "features": {
                "flat_faces": [
                    {"normal": [0, 0, -1], "centroid": [0, 0, 200]},
                ],
            },
        }
        result = _find_opposite_face(
            analysis,
            bore_pos=[0, 0, 0],
            bore_axis=[0, 0, 1],
            axis_idx=2,
            max_depth=100,
        )
        assert result is None

    def test_max_depth_accepts_close(self):
        """Face within max_depth is accepted."""
        analysis = {
            "features": {
                "flat_faces": [
                    {"normal": [0, 0, -1], "centroid": [0, 0, 50]},
                ],
            },
        }
        result = _find_opposite_face(
            analysis,
            bore_pos=[0, 0, 0],
            bore_axis=[0, 0, 1],
            axis_idx=2,
            max_depth=100,
        )
        assert result == 50

    def test_no_faces(self):
        """Returns None when no flat faces exist."""
        analysis: dict = {"features": {"flat_faces": []}}
        result = _find_opposite_face(
            analysis, bore_pos=[0, 0, 0], bore_axis=[0, 0, 1], axis_idx=2
        )
        assert result is None

    def test_selects_closest_in_cross_plane(self):
        """When two opposing faces exist, picks the one closest cross-axis."""
        analysis = {
            "features": {
                "flat_faces": [
                    {"normal": [0, 0, -1], "centroid": [100, 0, 50]},  # far
                    {"normal": [0, 0, -1], "centroid": [5, 0, 40]},  # close
                ],
            },
        }
        result = _find_opposite_face(
            analysis,
            bore_pos=[0, 0, 0],
            bore_axis=[0, 0, 1],
            axis_idx=2,
        )
        assert result == 40

    def test_require_far_side(self):
        """With require_far_side=True, only faces on far side are considered."""
        analysis = {
            "features": {
                "flat_faces": [
                    {"normal": [0, 0, -1], "centroid": [0, 0, -10]},  # near side
                    {"normal": [0, 0, -1], "centroid": [0, 0, 50]},  # far side
                ],
            },
        }
        result = _find_opposite_face(
            analysis,
            bore_pos=[0, 0, 0],
            bore_axis=[0, 0, 1],
            axis_idx=2,
            require_far_side=True,
        )
        assert result == 50


# ---------------------------------------------------------------------------
# validate_fk
# ---------------------------------------------------------------------------


class TestValidateFk:
    def test_with_real_urdf(self, robot_dir: Path):
        """FK validation runs on real Meca500 URDF without errors."""
        import yaml

        chain = yaml.safe_load((robot_dir / "chain.yaml").read_text())
        urdf_path = robot_dir / "robot.urdf"
        assert urdf_path.exists()

        messages = validate_fk(chain, urdf_path)
        assert any("FK validation" in m for m in messages)
        # Should report positions and distances
        assert any("distance=" in m for m in messages)

    def test_no_dh_params(self):
        """Returns empty messages when chain has no dh_params."""
        chain: dict = {"dh_params": {}}
        msgs = validate_fk(chain, Path("/nonexistent.urdf"))
        assert msgs == []

    def test_simple_urdf(self, tmp_path: Path):
        """FK with a hand-crafted simple URDF."""
        urdf_content = """\
<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="link1"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""
        urdf_path = tmp_path / "test.urdf"
        urdf_path.write_text(urdf_content)
        chain = {"dh_params": {"d1": 100}}

        msgs = validate_fk(chain, urdf_path)
        assert any("j1" in m for m in msgs)
        # Position should be (0, 0, 100) mm
        assert any("100.0" in m for m in msgs)
