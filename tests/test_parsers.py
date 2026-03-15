"""Tests for analyze/parsers/ module."""

from __future__ import annotations

from pathlib import Path

import pytest
import trimesh

from robot_arm_sim.analyze.parsers import get_parser
from robot_arm_sim.analyze.parsers.stl_parser import STLParser


class TestGetParser:
    """Tests for the get_parser registry function."""

    def test_stl_extension_returns_stl_parser(self) -> None:
        parser = get_parser(Path("foo/bar.stl"))
        assert isinstance(parser, STLParser)

    def test_stl_extension_case_insensitive(self) -> None:
        parser = get_parser(Path("foo/bar.STL"))
        assert isinstance(parser, STLParser)

    def test_unsupported_extension_raises_value_error(self) -> None:
        with pytest.raises(ValueError, match="Unsupported format '.obj'"):
            get_parser(Path("model.obj"))

    def test_unsupported_extension_lists_supported(self) -> None:
        with pytest.raises(ValueError, match=r"\.stl"):
            get_parser(Path("model.step"))

    def test_no_extension_raises(self) -> None:
        with pytest.raises(ValueError, match="Unsupported format"):
            get_parser(Path("no_extension"))


class TestSTLParser:
    """Tests for STLParser."""

    def test_supported_extensions(self) -> None:
        parser = STLParser()
        assert parser.supported_extensions() == [".stl"]

    def test_parse_real_stl(self, robot_dir: Path) -> None:
        stl_files = sorted((robot_dir / "stl_files").glob("*.stl"))
        assert len(stl_files) > 0, "No STL files found in fixture"

        parser = STLParser()
        result = parser.parse(stl_files[0])

        # Check top-level fields
        assert result.part_name == stl_files[0].stem
        assert result.source_file == str(stl_files[0])
        assert result.format in ("ascii_stl", "binary_stl")

        # Check geometry
        geo = result.geometry
        assert geo.vertex_count > 0
        assert geo.face_count > 0
        assert len(geo.bounding_box.min) == 3
        assert len(geo.bounding_box.max) == 3
        assert len(geo.bounding_box.extents) == 3
        assert all(e > 0 for e in geo.bounding_box.extents)
        assert geo.surface_area_mm2 > 0
        assert len(geo.center_of_mass) == 3

        # Check inertia
        assert len(result.inertia.principal_moments) == 3

    def test_parse_multiple_stl_files(self, robot_dir: Path) -> None:
        """All STL files in fixture should parse without error."""
        stl_files = sorted((robot_dir / "stl_files").glob("*.stl"))
        parser = STLParser()

        for stl_file in stl_files:
            result = parser.parse(stl_file)
            assert result.part_name == stl_file.stem
            assert result.geometry.vertex_count > 0

    def test_parse_nonexistent_file_raises(self) -> None:
        parser = STLParser()
        with pytest.raises((FileNotFoundError, ValueError, OSError)):
            parser.parse(Path("/nonexistent/file.stl"))

    def test_watertight_volume(self, robot_dir: Path) -> None:
        """Watertight meshes should have volume > 0, non-watertight should be 0."""
        stl_files = sorted((robot_dir / "stl_files").glob("*.stl"))
        parser = STLParser()

        for stl_file in stl_files:
            result = parser.parse(stl_file)
            if result.geometry.is_watertight:
                assert result.geometry.volume_mm3 > 0
            else:
                assert result.geometry.volume_mm3 == 0.0

    def test_inertia_fallback_on_error(self, robot_dir: Path) -> None:
        """When principal_inertia raises, fallback to zeros (lines 38-40)."""
        from unittest.mock import PropertyMock, patch

        stl_files = sorted((robot_dir / "stl_files").glob("*.stl"))
        assert len(stl_files) > 0

        parser = STLParser()

        # Patch trimesh so principal_inertia_components raises
        with patch.object(
            trimesh.Trimesh,
            "principal_inertia_components",
            new_callable=PropertyMock,
            side_effect=Exception("non-watertight"),
        ):
            result = parser.parse(stl_files[0])
            assert result.inertia.principal_moments == [0.0, 0.0, 0.0]
            assert result.inertia.principal_axes == []

    def test_center_mass_fallback_on_error(self, robot_dir: Path) -> None:
        """When center_mass raises, fallback to bounds mean (lines 45-46)."""
        from unittest.mock import PropertyMock, patch

        stl_files = sorted((robot_dir / "stl_files").glob("*.stl"))
        assert len(stl_files) > 0

        parser = STLParser()

        with patch.object(
            trimesh.Trimesh,
            "center_mass",
            new_callable=PropertyMock,
            side_effect=Exception("degenerate"),
        ):
            result = parser.parse(stl_files[0])
            # Should still have a valid 3-element center_of_mass
            assert len(result.geometry.center_of_mass) == 3

    def test_ascii_stl_format_detection(self, tmp_path: Path) -> None:
        """STL files starting with 'solid' header should be detected as ASCII."""
        # Create a minimal ASCII STL file
        ascii_stl = tmp_path / "test_ascii.stl"
        ascii_stl.write_text(
            "solid test\n"
            "  facet normal 0 0 1\n"
            "    outer loop\n"
            "      vertex 0 0 0\n"
            "      vertex 1 0 0\n"
            "      vertex 0 1 0\n"
            "    endloop\n"
            "  endfacet\n"
            "endsolid test\n"
        )

        parser = STLParser()
        result = parser.parse(ascii_stl)
        assert result.format == "ascii_stl"
