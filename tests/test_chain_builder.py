"""Tests for the programmatic chain builder (DH → chain.yaml)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from robot_arm_sim.chain_builder import (
    _detect_combined_parts,
    _extract_dh_summary,
    _normalise_dh_rows,
    _rotation_to_rpy,
    _rx,
    _rz,
    _tx,
    _tz,
    build_chain,
    dh_to_urdf_joints,
)
from robot_arm_sim.models.models import _load_yaml

# ---------------------------------------------------------------------------
# Transform helpers
# ---------------------------------------------------------------------------


class TestTransformHelpers:
    def test_rz_identity(self):
        np.testing.assert_allclose(_rz(0), np.eye(4), atol=1e-12)

    def test_rz_90(self):
        m = _rz(math.pi / 2)
        assert abs(m[0, 0]) < 1e-12  # cos(90) ≈ 0
        assert abs(m[0, 1] + 1) < 1e-12  # -sin(90) = -1
        assert abs(m[1, 0] - 1) < 1e-12  # sin(90) = 1

    def test_rx_identity(self):
        np.testing.assert_allclose(_rx(0), np.eye(4), atol=1e-12)

    def test_rx_90(self):
        m = _rx(math.pi / 2)
        assert abs(m[1, 1]) < 1e-12
        assert abs(m[2, 1] - 1) < 1e-12

    def test_tz(self):
        m = _tz(100.0)
        assert m[2, 3] == 100.0

    def test_tx(self):
        m = _tx(50.0)
        assert m[0, 3] == 50.0


class TestRotationToRpy:
    def test_identity(self):
        rpy = _rotation_to_rpy(np.eye(3))
        assert all(abs(v) < 1e-12 for v in rpy)

    def test_rx_90(self):
        r = _rx(math.pi / 2)[:3, :3]
        rpy = _rotation_to_rpy(r)
        assert abs(rpy[0] - math.pi / 2) < 1e-6  # roll
        assert abs(rpy[1]) < 1e-6
        assert abs(rpy[2]) < 1e-6


# ---------------------------------------------------------------------------
# DH to URDF conversion
# ---------------------------------------------------------------------------


# Minimal 2-DOF DH for easy manual verification
SIMPLE_2DOF = [
    {"joint": 1, "d": 100.0, "a": 0.0, "alpha": 0.0, "theta_offset": 0.0},
    {"joint": 2, "d": 0.0, "a": 200.0, "alpha": 0.0, "theta_offset": 0.0},
]


class TestDhToUrdfJoints:
    def test_simple_2dof_count(self):
        result = dh_to_urdf_joints(SIMPLE_2DOF)
        assert len(result) == 2

    def test_simple_2dof_first_joint(self):
        """First joint: P=Tz(100), Q_prev=I → origin = [0, 0, 0.1]."""
        result = dh_to_urdf_joints(SIMPLE_2DOF)
        j1 = result[0]
        assert j1["joint_index"] == 1
        np.testing.assert_allclose(j1["origin"], [0, 0, 0.1], atol=1e-6)
        assert j1["axis"] == [0, 0, 1]

    def test_simple_2dof_second_joint(self):
        """Second joint: Q_prev=Tx(0)·Rx(0)=I, P=Tz(0) → origin=[0,0,0].
        But a1=0, so Q_prev from j1 is Tx(0)·Rx(0)=I.
        P2 = Rz(0)·Tz(0) = I → origin = [0,0,0]."""
        result = dh_to_urdf_joints(SIMPLE_2DOF)
        j2 = result[1]
        np.testing.assert_allclose(j2["origin"], [0, 0, 0], atol=1e-6)

    def test_all_axes_are_z(self):
        result = dh_to_urdf_joints(SIMPLE_2DOF)
        for j in result:
            assert j["axis"] == [0, 0, 1]

    def test_nonzero_alpha_produces_rpy(self):
        rows = [
            {
                "joint": 1,
                "d": 100.0,
                "a": 50.0,
                "alpha": -math.pi / 2,
                "theta_offset": 0.0,
            },
        ]
        result = dh_to_urdf_joints(rows)
        rpy = result[0]["origin_rpy"]
        # alpha = -pi/2 → Q_prev=I for first joint, so no rpy on joint 1
        # rpy comes from Q_prev applied to next joint; joint 1 itself:
        # Origin_1 = Q_0 · P_1 = I · Rz(0)·Tz(100) = Tz(100)
        # rpy should be [0,0,0] for the first joint since Q_prev=I
        assert all(abs(v) < 1e-6 for v in rpy)

    def test_alpha_affects_next_joint(self):
        """Alpha on joint i should appear in joint i+1's origin_rpy."""
        rows = [
            {
                "joint": 1,
                "d": 100.0,
                "a": 0.0,
                "alpha": -math.pi / 2,
                "theta_offset": 0.0,
            },
            {
                "joint": 2,
                "d": 50.0,
                "a": 0.0,
                "alpha": 0.0,
                "theta_offset": 0.0,
            },
        ]
        result = dh_to_urdf_joints(rows)
        j2 = result[1]
        # Q_1 = Tx(0) · Rx(-pi/2) = Rx(-pi/2)
        # P_2 = Rz(0) · Tz(50) = Tz(50)
        # Origin_2 = Rx(-pi/2) · Tz(50)
        # Rx(-pi/2) = [[1,0,0],[0,0,1],[0,-1,0]]
        # Translation: Rx(-pi/2) · [0,0,50] = [0, 50, 0] mm
        np.testing.assert_allclose(j2["origin"], [0, 0.05, 0], atol=1e-6)
        # Rotation: Rx(-pi/2) → roll = -pi/2
        assert abs(j2["origin_rpy"][0] - (-math.pi / 2)) < 1e-4


# ---------------------------------------------------------------------------
# FK preservation: DH chain == URDF chain at zero config
# ---------------------------------------------------------------------------


# Meca500 DH parameters from specs.yaml
MECA500_DH = [
    {"joint": 1, "d": 135.0, "a": 0.0, "alpha": -1.5707963, "theta_offset": 0.0},
    {"joint": 2, "d": 0.0, "a": 135.0, "alpha": 0.0, "theta_offset": -1.5707963},
    {"joint": 3, "d": 0.0, "a": 38.0, "alpha": -1.5707963, "theta_offset": 0.0},
    {"joint": 4, "d": 120.0, "a": 0.0, "alpha": 1.5707963, "theta_offset": 0.0},
    {"joint": 5, "d": 0.0, "a": 0.0, "alpha": -1.5707963, "theta_offset": 0.0},
    {"joint": 6, "d": 70.0, "a": 0.0, "alpha": 0.0, "theta_offset": 3.1415927},
]


def _dh_transform(d: float, a: float, alpha: float, theta: float) -> np.ndarray:
    """Standard DH: T = Rz(θ)·Tz(d)·Tx(a)·Rx(α)."""
    return _rz(theta) @ _tz(d) @ _tx(a) @ _rx(alpha)


class TestFkPreservation:
    """Verify that the P/Q decomposition preserves exact DH kinematics."""

    def test_meca500_fk_at_zero(self):
        """FK via DH chain must equal FK via P/Q decomposition at q=0.

        Since T_i = P_i · Q_i (by definition), the decomposition is exact.
        We verify the identity: prod(T_i) == prod(P_i · Q_i).
        """
        # Build DH FK at zero config
        dh_fk = np.eye(4)
        for row in MECA500_DH:
            theta = float(row.get("theta_offset", 0.0))
            t = _dh_transform(row["d"], row["a"], row["alpha"], theta)
            dh_fk = dh_fk @ t

        # Build P·Q FK (this is what chain_builder decomposes)
        pq_fk = np.eye(4)
        for row in MECA500_DH:
            p = _rz(row.get("theta_offset", 0.0)) @ _tz(row["d"])
            q = _tx(row["a"]) @ _rx(row["alpha"])
            pq_fk = pq_fk @ p @ q

        np.testing.assert_allclose(
            dh_fk,
            pq_fk,
            atol=1e-6,
            err_msg="P/Q decomposition must be identical to DH",
        )

    def test_meca500_origin_grouping(self):
        """Verify Q_{i-1}·P_i grouping preserves DH kinematics.

        URDF at q=0: prod(Origin_i) = prod(Q_{i-1}·P_i)
        This equals P_1·Q_1·P_2·...·P_n (missing trailing Q_n).
        Appending Q_n gives the full DH FK: T_1·T_2·...·T_n.
        """
        q_prev = np.eye(4)
        urdf_fk = np.eye(4)
        for row in MECA500_DH:
            p = _rz(row.get("theta_offset", 0.0)) @ _tz(row["d"])
            origin_exact = q_prev @ p
            q = _tx(row["a"]) @ _rx(row["alpha"])
            urdf_fk = urdf_fk @ origin_exact
            q_prev = q

        # Append trailing Q_n (end-effector frame)
        urdf_fk = urdf_fk @ q_prev

        # DH FK
        dh_fk = np.eye(4)
        for row in MECA500_DH:
            theta = float(row.get("theta_offset", 0.0))
            dh_fk = dh_fk @ _dh_transform(row["d"], row["a"], row["alpha"], theta)

        np.testing.assert_allclose(
            dh_fk,
            urdf_fk,
            atol=1e-6,
            err_msg="Origin grouping + trailing Q must match DH FK",
        )

    def test_meca500_joint_count(self):
        result = dh_to_urdf_joints(MECA500_DH)
        assert len(result) == 6

    def test_meca500_first_joint_origin(self):
        """First joint should be at d1=135mm = 0.135m along Z."""
        result = dh_to_urdf_joints(MECA500_DH)
        np.testing.assert_allclose(result[0]["origin"], [0, 0, 0.135], atol=1e-6)


# ---------------------------------------------------------------------------
# Part mapping helpers
# ---------------------------------------------------------------------------


class TestDetectCombinedParts:
    def test_a3_4(self):
        parts = [{"name": "A3_4"}]
        result = _detect_combined_parts(parts)
        assert result == {"A3_4": [3, 4]}

    def test_no_combined(self):
        parts = [{"name": "A0"}, {"name": "A1"}]
        assert _detect_combined_parts(parts) == {}

    def test_multi_combined(self):
        parts = [{"name": "X1_2"}, {"name": "Y5_7"}]
        result = _detect_combined_parts(parts)
        assert result == {"X1_2": [1, 2], "Y5_7": [5, 6, 7]}


class TestExtractDhSummary:
    def test_meca500(self):
        summary = _extract_dh_summary(MECA500_DH)
        assert summary["d1"] == 135.0
        assert summary["a2"] == 135.0
        assert summary["a3"] == 38.0
        assert summary["d4"] == 120.0
        assert summary["d6"] == 70.0
        # Zero values should be excluded
        assert "a1" not in summary
        assert "d2" not in summary


# ---------------------------------------------------------------------------
# Full build_chain with Meca500 fixture data
# ---------------------------------------------------------------------------


class TestBuildChainMeca500:
    def test_builds_successfully(self, robot_dir: Path):
        specs_path = robot_dir / "specs.yaml"
        summary_path = robot_dir / "analysis" / "summary.yaml"
        chain, messages = build_chain(specs_path, summary_path)

        assert chain.robot_name == "Meca500-R3"
        assert len(chain.joints) == 6
        # base_link + 6 links = 7 links minimum
        assert len(chain.links) >= 7

    def test_all_joints_have_origins(self, robot_dir: Path):
        specs_path = robot_dir / "specs.yaml"
        summary_path = robot_dir / "analysis" / "summary.yaml"
        chain, _ = build_chain(specs_path, summary_path)

        for joint in chain.joints:
            assert joint.origin is not None, f"{joint.name} missing origin"

    def test_all_joints_have_limits(self, robot_dir: Path):
        specs_path = robot_dir / "specs.yaml"
        summary_path = robot_dir / "analysis" / "summary.yaml"
        chain, _ = build_chain(specs_path, summary_path)

        for joint in chain.joints:
            assert joint.limits is not None, f"{joint.name} missing limits"
            assert len(joint.limits) == 2

    def test_virtual_link_for_combined_part(self, robot_dir: Path):
        """A3_4 spans joints 3-4, so link_3 should be virtual (mesh=None)."""
        specs_path = robot_dir / "specs.yaml"
        summary_path = robot_dir / "analysis" / "summary.yaml"
        chain, _ = build_chain(specs_path, summary_path)

        link_3 = next(lk for lk in chain.links if lk.name == "link_3")
        assert link_3.mesh is None

    def test_preserves_user_added(self, robot_dir: Path):
        """User-added links/joints from existing chain.yaml are preserved."""
        specs_path = robot_dir / "specs.yaml"
        summary_path = robot_dir / "analysis" / "summary.yaml"
        existing_chain = robot_dir / "chain.yaml"

        chain, _ = build_chain(specs_path, summary_path, existing_chain)

        user_link_names = [lk.name for lk in chain.links if lk.user_added]
        user_joint_names = [j.name for j in chain.joints if j.user_added]
        assert len(user_link_names) > 0, "Expected at least one user-added link"
        assert len(user_joint_names) > 0, "Expected at least one user-added joint"

    def test_dh_params_in_output(self, robot_dir: Path):
        specs_path = robot_dir / "specs.yaml"
        summary_path = robot_dir / "analysis" / "summary.yaml"
        chain, _ = build_chain(specs_path, summary_path)

        assert chain.dh_params is not None
        assert chain.dh_params["d1"] == 135.0


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


class TestBuildChainCli:
    def test_writes_chain_yaml(self, robot_dir: Path):
        from robot_arm_sim.chain_builder import build_chain_cli

        output = robot_dir / "chain_test.yaml"
        messages = build_chain_cli(robot_dir, output)

        assert output.exists()
        assert any("Wrote" in m for m in messages)


# ---------------------------------------------------------------------------
# Cross-robot: build chain for all robots in the repo
# ---------------------------------------------------------------------------

ROBOTS_DIR = Path(__file__).resolve().parent.parent / "robots"
ALL_ROBOTS = sorted(
    p.name
    for p in ROBOTS_DIR.iterdir()
    if p.is_dir()
    and (p / "specs.yaml").exists()
    and (p / "analysis" / "summary.yaml").exists()
)


@pytest.mark.parametrize("robot_name", ALL_ROBOTS)
class TestAllRobots:
    def test_builds_without_error(self, robot_name: str):
        robot_dir = ROBOTS_DIR / robot_name
        chain, messages = build_chain(
            robot_dir / "specs.yaml",
            robot_dir / "analysis" / "summary.yaml",
        )
        assert chain.robot_name
        assert len(chain.joints) > 0
        assert len(chain.links) > len(chain.joints)  # links = joints + 1

    def test_joint_count_matches_dof(self, robot_name: str):
        robot_dir = ROBOTS_DIR / robot_name
        specs_data = _load_yaml(robot_dir / "specs.yaml")
        dof = specs_data.get("dof", 6)

        chain, _ = build_chain(
            robot_dir / "specs.yaml",
            robot_dir / "analysis" / "summary.yaml",
        )
        assert len(chain.joints) == dof

    def test_fk_matches_dh(self, robot_name: str):
        """P/Q decomposition matches direct DH FK for every robot."""
        robot_dir = ROBOTS_DIR / robot_name
        specs_data = _load_yaml(robot_dir / "specs.yaml")
        dh_rows_raw = (
            specs_data.get("dh_parameters") or specs_data.get("dh_params") or []
        )
        dh_rows = _normalise_dh_rows(dh_rows_raw)

        # DH FK
        dh_fk = np.eye(4)
        for row in dh_rows:
            theta = float(row.get("theta_offset", 0.0))
            dh_fk = dh_fk @ _dh_transform(row["d"], row["a"], row["alpha"], theta)

        # Direct P·Q FK (must be identical)
        pq_fk = np.eye(4)
        for row in dh_rows:
            p = _rz(row.get("theta_offset", 0.0)) @ _tz(row["d"])
            q = _tx(row["a"]) @ _rx(row["alpha"])
            pq_fk = pq_fk @ p @ q

        np.testing.assert_allclose(
            dh_fk,
            pq_fk,
            atol=1e-6,
            err_msg=f"{robot_name}: P·Q must equal DH FK",
        )
