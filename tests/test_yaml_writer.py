"""Tests for YAML output generation."""

from __future__ import annotations

from pathlib import Path

import yaml

from robot_arm_sim.analyze.yaml_writer import (
    _detect_sequential_pattern,
    _generate_assembly_hints,
    _infer_role_hint,
    write_part_yaml,
    write_summary_yaml,
)
from robot_arm_sim.models import (
    BoundingBox,
    Features,
    FlatFace,
    Geometry,
    PartAnalysis,
)


def _make_analysis(name: str, features: Features | None = None) -> PartAnalysis:
    return PartAnalysis(
        part_name=name,
        source_file=f"stl_files/{name}.stl",
        format="binary_stl",
        geometry=Geometry(
            vertex_count=100,
            face_count=50,
            bounding_box=BoundingBox(
                min=[0, 0, 0],
                max=[50, 50, 50],
                extents=[50.0, 50.0, 50.0],
            ),
        ),
        features=features or Features(),
    )


def test_write_part_yaml_creates_file(tmp_path: Path):
    analysis = _make_analysis("TestPart")
    out = tmp_path / "TestPart.yaml"
    write_part_yaml(analysis, out)

    assert out.exists()
    data = yaml.safe_load(out.read_text())
    assert data["part_name"] == "TestPart"
    assert "geometry" in data
    assert "features" in data


def test_write_summary_yaml_creates_file(tmp_path: Path):
    analyses = [_make_analysis("P0"), _make_analysis("P1")]
    out = tmp_path / "summary.yaml"
    write_summary_yaml("TestRobot", analyses, out)

    assert out.exists()
    data = yaml.safe_load(out.read_text())
    assert data["robot_name"] == "TestRobot"
    assert data["part_count"] == 2
    assert len(data["parts"]) == 2


def test_infer_role_hint_base_detection():
    """Part with a large flat bottom face should get 'possible base' hint."""
    features = Features(
        flat_faces=[
            FlatFace(
                description="flat bottom",
                normal=[0, 0, -1],
                area_mm2=500.0,
            )
        ],
    )
    analysis = _make_analysis("base", features=features)
    hint = _infer_role_hint(analysis)
    assert "base" in hint.lower()


def test_infer_role_hint_combined_part():
    """Part with underscore in name should get combined hint."""
    analysis = _make_analysis("A3_4")
    hint = _infer_role_hint(analysis)
    assert "combined" in hint.lower()


def test_infer_role_hint_first_in_sequence():
    """Part with 0 suffix should get 'first part in sequence' hint."""
    analysis = _make_analysis("L0")
    hint = _infer_role_hint(analysis)
    assert "first part" in hint.lower() or "base" in hint.lower()


def test_generate_assembly_hints_sequential():
    """Sequential A0,A1,A2 parts should get a sequential naming hint."""
    analyses = [_make_analysis("A0"), _make_analysis("A1"), _make_analysis("A2")]
    hints = _generate_assembly_hints(analyses)
    hint_text = " ".join(hints).lower()
    assert "sequential" in hint_text


def test_generate_assembly_hints_sequential_other_prefix():
    """Sequential L0,L1,L2 parts should also get sequential hint."""
    analyses = [_make_analysis("L0"), _make_analysis("L1"), _make_analysis("L2")]
    hints = _generate_assembly_hints(analyses)
    hint_text = " ".join(hints).lower()
    assert "sequential" in hint_text


def test_generate_assembly_hints_non_sequential():
    """Non-sequential names should not get a sequential naming hint."""
    analyses = [
        _make_analysis("base"),
        _make_analysis("arm"),
        _make_analysis("gripper"),
    ]
    hints = _generate_assembly_hints(analyses)
    hint_text = " ".join(hints).lower()
    # Should list parts but not claim sequential naming
    assert "base" in hint_text
    assert "sequential" not in hint_text


def test_detect_sequential_pattern_various():
    """Test the sequential pattern detector with various inputs."""
    assert _detect_sequential_pattern(["A0", "A1", "A2"]) is not None
    assert _detect_sequential_pattern(["link0", "link1", "link2"]) is not None
    assert _detect_sequential_pattern(["base", "arm", "grip"]) is None
    assert _detect_sequential_pattern(["P1"]) is None  # too few
