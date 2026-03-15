"""Tests for urdf_generator module — URDF generation pipeline."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from robot_arm_sim.analyze.urdf_generator import _fmt_xyz, generate_urdf

# ---------------------------------------------------------------------------
# _fmt_xyz
# ---------------------------------------------------------------------------


class TestFmtXyz:
    def test_zeros(self):
        assert _fmt_xyz([0.0, 0.0, 0.0]) == "0.000000 0.000000 0.000000"

    def test_positive(self):
        result = _fmt_xyz([1.5, 2.25, 3.0])
        assert result == "1.500000 2.250000 3.000000"

    def test_negative(self):
        result = _fmt_xyz([-0.1, 0.0, 0.05])
        assert result == "-0.100000 0.000000 0.050000"

    def test_negative_zero(self):
        """Negative zero should format as positive zero (v + 0.0 trick)."""
        result = _fmt_xyz([-0.0, 0.0, 0.0])
        assert result == "0.000000 0.000000 0.000000"

    def test_integer_values(self):
        """Integer values pass through str() directly."""
        result = _fmt_xyz([0, 1, 0])
        assert result == "0 1 0"


# ---------------------------------------------------------------------------
# generate_urdf — full pipeline
# ---------------------------------------------------------------------------


class TestGenerateUrdf:
    def test_full_pipeline(self, robot_dir: Path):
        """Full URDF generation for Meca500 produces valid XML."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        messages = generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        assert output_path.exists()
        assert any("Wrote URDF" in m for m in messages)

        # Parse and validate XML structure
        tree = ET.parse(output_path)
        root = tree.getroot()
        assert root.tag == "robot"
        assert root.get("name") == "Meca500-R3"

    def test_link_count(self, robot_dir: Path):
        """Generated URDF has correct number of links."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        tree = ET.parse(output_path)
        root = tree.getroot()
        links = root.findall("link")
        # chain.yaml has 7 links (base_link + link_1..6)
        assert len(links) == 7

    def test_joint_count(self, robot_dir: Path):
        """Generated URDF has correct number of joints."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        tree = ET.parse(output_path)
        root = tree.getroot()
        joints = root.findall("joint")
        # chain.yaml has 6 joints
        assert len(joints) == 6

    def test_mesh_paths(self, robot_dir: Path):
        """All mesh filenames reference stl_files/ directory."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        tree = ET.parse(output_path)
        root = tree.getroot()
        meshes = root.findall(".//mesh")
        assert len(meshes) > 0
        for mesh_el in meshes:
            filename = mesh_el.get("filename", "")
            assert filename.startswith("stl_files/")
            assert filename.endswith(".stl")

    def test_joint_structure(self, robot_dir: Path):
        """Each joint has parent, child, origin, and axis elements."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        tree = ET.parse(output_path)
        root = tree.getroot()
        for joint_el in root.findall("joint"):
            assert joint_el.find("parent") is not None
            assert joint_el.find("child") is not None
            assert joint_el.find("origin") is not None
            assert joint_el.find("axis") is not None

    def test_revolute_joints_have_limits(self, robot_dir: Path):
        """Revolute joints with limits in chain.yaml get limit elements."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        tree = ET.parse(output_path)
        root = tree.getroot()
        revolute_joints = [
            j for j in root.findall("joint") if j.get("type") == "revolute"
        ]
        assert len(revolute_joints) > 0
        for joint_el in revolute_joints:
            limit = joint_el.find("limit")
            if limit is not None:
                assert "lower" in limit.attrib
                assert "upper" in limit.attrib

    def test_fk_validation_runs(self, robot_dir: Path):
        """FK validation messages are included in output."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        messages = generate_urdf(chain_path, analysis_dir, stl_dir, output_path)
        assert any("FK validation" in m for m in messages)

    def test_missing_analysis_warns(self, robot_dir_no_urdf: Path):
        """Missing analysis YAML for a mesh produces a warning."""

        chain_path = robot_dir_no_urdf / "chain.yaml"
        analysis_dir = robot_dir_no_urdf / "analysis"
        stl_dir = robot_dir_no_urdf / "stl_files"
        output_path = robot_dir_no_urdf / "test_output.urdf"

        # Remove one analysis file to trigger warning
        a1_yaml = analysis_dir / "A1.yaml"
        if a1_yaml.exists():
            a1_yaml.unlink()

        messages = generate_urdf(chain_path, analysis_dir, stl_dir, output_path)
        assert any("WARNING" in m and "A1" in m for m in messages)

    def test_output_is_pretty_printed(self, robot_dir: Path):
        """Output URDF is indented XML, not single-line."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        content = output_path.read_text()
        assert content.startswith('<?xml version="1.0"?>')
        lines = content.strip().split("\n")
        assert len(lines) > 10  # not a single-line blob
        # Should have indentation
        assert any(line.startswith("  ") for line in lines)

    def test_mesh_scale(self, robot_dir: Path):
        """All meshes have mm-to-m scale factor."""
        chain_path = robot_dir / "chain.yaml"
        analysis_dir = robot_dir / "analysis"
        stl_dir = robot_dir / "stl_files"
        output_path = robot_dir / "test_output.urdf"

        generate_urdf(chain_path, analysis_dir, stl_dir, output_path)

        tree = ET.parse(output_path)
        root = tree.getroot()
        for mesh_el in root.findall(".//mesh"):
            assert mesh_el.get("scale") == "0.001 0.001 0.001"
