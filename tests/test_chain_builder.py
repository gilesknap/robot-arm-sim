"""Tests for the chain_builder module."""

from __future__ import annotations

from pathlib import Path

import pytest

from robot_arm_sim.analyze.chain_builder import (
    _compute_axis_from_connections,
    _compute_origin_from_connections,
    _deg_to_rad,
    _extract_link_number,
    _is_numbered_naming,
    _mm_to_m,
    _order_parts_numbered,
    build_chain,
)
from robot_arm_sim.models.models import ConnectionPoint, PartAnalysis

# --- Unit tests for helpers ---


class TestHelpers:
    def test_deg_to_rad(self):
        assert _deg_to_rad(180) == pytest.approx(3.1416, abs=1e-4)
        assert _deg_to_rad(-90) == pytest.approx(-1.5708, abs=1e-4)
        assert _deg_to_rad(0) == 0.0

    def test_mm_to_m(self):
        assert _mm_to_m(1000) == 1.0
        assert _mm_to_m(89.159) == pytest.approx(0.089159)

    def test_extract_link_number(self):
        assert _extract_link_number("base") == 0
        assert _extract_link_number("base_link") == 0
        assert _extract_link_number("link_1") == 1
        assert _extract_link_number("link_5") == 5
        assert _extract_link_number("A3") == 3
        assert _extract_link_number("shoulder") is None

    def test_is_numbered_naming(self):
        assert _is_numbered_naming(["base_link", "link_1", "link_2"])
        assert not _is_numbered_naming(["base", "shoulder", "upperarm"])
        assert _is_numbered_naming(["A0", "A1", "A2"])

    def test_order_parts_numbered(self):
        assert _order_parts_numbered(["link_2", "base_link", "link_1"]) == [
            "base_link",
            "link_1",
            "link_2",
        ]


# --- Connection point tests ---


def _make_analysis(
    proximal_pos: list[float] | None = None,
    distal_pos: list[float] | None = None,
    distal_axis: list[float] | None = None,
) -> PartAnalysis:
    """Create a minimal PartAnalysis with optional connection points."""
    cps = []
    if proximal_pos:
        cps.append(
            ConnectionPoint(
                end="proximal",
                position=proximal_pos,
                axis=[0, 0, 1],
                radius_mm=50,
                method="test",
            )
        )
    if distal_pos:
        cps.append(
            ConnectionPoint(
                end="distal",
                position=distal_pos,
                axis=distal_axis or [0, 0, 1],
                radius_mm=50,
                method="test",
            )
        )
    return PartAnalysis(
        part_name="test",
        source_file="test.stl",
        format="binary_stl",
        connection_points=cps,
    )


class TestConnectionPoints:
    def test_origin_from_proximal_distal(self):
        analysis = _make_analysis(
            proximal_pos=[0, 0, 0],
            distal_pos=[0, 0, 100],  # 100mm apart
        )
        msgs: list[str] = []
        origin = _compute_origin_from_connections(analysis, msgs, "j1")
        assert origin is not None
        assert origin == [0.0, 0.0, 0.1]  # 100mm = 0.1m

    def test_origin_no_proximal(self):
        analysis = _make_analysis(distal_pos=[50, 0, 200])
        msgs: list[str] = []
        origin = _compute_origin_from_connections(analysis, msgs, "j1")
        assert origin is not None
        assert origin == [0.05, 0.0, 0.2]

    def test_origin_no_connection_points(self):
        analysis = _make_analysis()
        msgs: list[str] = []
        origin = _compute_origin_from_connections(analysis, msgs, "j1")
        assert origin is None

    def test_axis_from_distal(self):
        analysis = _make_analysis(
            distal_pos=[0, 0, 100],
            distal_axis=[0, 1, 0],
        )
        axis = _compute_axis_from_connections(analysis)
        assert axis == [0, 1, 0]

    def test_axis_no_distal(self):
        analysis = _make_analysis(proximal_pos=[0, 0, 0])
        axis = _compute_axis_from_connections(analysis)
        assert axis is None


# --- Integration tests with real robot data ---


class TestBuildChainUR5:
    """Test build_chain against UR5 robot data."""

    @pytest.fixture
    def ur5_dir(self) -> Path:
        return Path("robots/UR5")

    def test_builds_chain(self, ur5_dir: Path):
        if not ur5_dir.exists():
            pytest.skip("UR5 robot data not available")
        chain, messages = build_chain(ur5_dir)
        assert chain.robot_name == "UR5"
        assert len(chain.links) == 7  # 7 parts
        assert len(chain.joints) == 6  # 6 DOF
        assert chain.dh_params is not None

    def test_joint_limits_converted(self, ur5_dir: Path):
        if not ur5_dir.exists():
            pytest.skip("UR5 robot data not available")
        chain, _ = build_chain(ur5_dir)
        for joint in chain.joints:
            assert joint.limits is not None
            # UR5 has ±360° limits
            assert joint.limits[0] == pytest.approx(-6.2832, abs=0.01)

    def test_flags_semantic_ordering(self, ur5_dir: Path):
        if not ur5_dir.exists():
            pytest.skip("UR5 robot data not available")
        _, messages = build_chain(ur5_dir)
        assert any("REVIEW" in m and "semantic" in m for m in messages)

    def test_flags_alpha_rotations(self, ur5_dir: Path):
        if not ur5_dir.exists():
            pytest.skip("UR5 robot data not available")
        _, messages = build_chain(ur5_dir)
        assert any("alpha=90" in m for m in messages)


class TestBuildChainFANUC:
    """Test build_chain against FANUC robot data."""

    @pytest.fixture
    def fanuc_dir(self) -> Path:
        return Path("robots/FANUC-LRMate200iD")

    def test_builds_chain(self, fanuc_dir: Path):
        if not fanuc_dir.exists():
            pytest.skip("FANUC robot data not available")
        chain, messages = build_chain(fanuc_dir)
        assert chain.robot_name == "FANUC-LRMate200iD"
        assert len(chain.links) == 7
        assert len(chain.joints) == 6

    def test_numbered_ordering(self, fanuc_dir: Path):
        if not fanuc_dir.exists():
            pytest.skip("FANUC robot data not available")
        chain, messages = build_chain(fanuc_dir)
        names = [lk.name for lk in chain.links]
        assert names == [
            "base_link",
            "link_1",
            "link_2",
            "link_3",
            "link_4",
            "link_5",
            "link_6",
        ]

    def test_dh_fallback_origins(self, fanuc_dir: Path):
        if not fanuc_dir.exists():
            pytest.skip("FANUC robot data not available")
        chain, _ = build_chain(fanuc_dir)
        # Joint 1: d=330, a=50 → origin=[0.05, 0, 0.33]
        j1 = chain.joints[0]
        assert j1.origin is not None
        assert j1.origin[0] == pytest.approx(0.05)
        assert j1.origin[2] == pytest.approx(0.33)
