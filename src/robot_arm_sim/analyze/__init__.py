"""Analysis module: parse meshes, detect features, render, write YAML."""

from __future__ import annotations

import logging
from pathlib import Path

import trimesh

from robot_arm_sim.models.part import PartAnalysis

from .connections import detect_connection_points
from .features import detect_features
from .parsers import get_parser
from .renderer import render_views
from .yaml_writer import write_part_yaml, write_summary_yaml

logger = logging.getLogger(__name__)


def run_analysis(robot_dir: Path) -> None:
    """Run full analysis pipeline on a robot directory.

    Expects STL files in <robot_dir>/stl_files/.
    Outputs analysis YAML to <robot_dir>/analysis/.
    """
    stl_dir = robot_dir / "stl_files"
    if not stl_dir.exists():
        raise FileNotFoundError(f"No stl_files/ directory found in {robot_dir}")

    stl_files = sorted(stl_dir.glob("*.stl")) + sorted(stl_dir.glob("*.STL"))
    if not stl_files:
        raise FileNotFoundError(f"No STL files found in {stl_dir}")

    analysis_dir = robot_dir / "analysis"
    renders_dir = analysis_dir / "renders"
    analysis_dir.mkdir(parents=True, exist_ok=True)

    robot_name = robot_dir.name
    analyses: list[PartAnalysis] = []

    for stl_file in stl_files:
        logger.info(f"Analyzing {stl_file.name}...")
        print(f"  Parsing {stl_file.name}...")

        # Parse mesh
        parser = get_parser(stl_file)
        analysis = parser.parse(stl_file)

        # Store relative source path
        analysis.source_file = f"stl_files/{stl_file.name}"

        # Detect features
        print(f"  Detecting features for {stl_file.name}...")
        mesh = trimesh.load(stl_file, force="mesh")
        assert isinstance(mesh, trimesh.Trimesh)
        features = detect_features(mesh)
        analysis.features = features

        # Detect connection points
        print(f"  Detecting connection points for {stl_file.name}...")
        conn_points = detect_connection_points(mesh, features, analysis.part_name)
        analysis.connection_points = conn_points
        if conn_points:
            for cp in conn_points:
                print(
                    f"    {cp.end}: pos={cp.position} "
                    f"axis={cp.axis} r={cp.radius_mm}mm ({cp.method})"
                )

        # Generate text description
        analysis.text_description = _generate_text_description(analysis)

        # Render views
        print(f"  Rendering views for {stl_file.name}...")
        render_paths = render_views(mesh, renders_dir, analysis.part_name)
        analysis.render_paths = render_paths

        # Write part YAML
        yaml_path = analysis_dir / f"{analysis.part_name}.yaml"
        write_part_yaml(analysis, yaml_path)
        print(f"  Wrote {yaml_path}")

        analyses.append(analysis)

    # Write summary
    summary_path = analysis_dir / "summary.yaml"
    write_summary_yaml(robot_name, analyses, summary_path)
    print(f"\nWrote summary to {summary_path}")
    print(f"Analyzed {len(analyses)} parts in {robot_name}")


def _generate_text_description(analysis: PartAnalysis) -> str:
    """Generate a human-readable description of the part."""
    lines = [f"Part '{analysis.part_name}' from {analysis.source_file}."]

    ext = analysis.bounding_box_extents
    lines.append(f"Bounding box: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm.")

    if analysis.is_watertight:
        lines.append(f"Watertight solid, volume {analysis.volume_mm3:.1f} mm³.")
    else:
        lines.append("Non-watertight mesh (volume not computed).")

    lines.append(f"Surface area: {analysis.surface_area_mm2:.1f} mm².")
    lines.append(
        f"Mesh complexity: {analysis.vertex_count} vertices, "
        f"{analysis.face_count} faces."
    )

    flat_faces = [f for f in analysis.features if f.kind == "flat_face"]
    cylinders = [f for f in analysis.features if f.kind == "cylindrical_surface"]
    holes = [f for f in analysis.features if f.kind == "hole"]

    if flat_faces:
        lines.append(f"Has {len(flat_faces)} significant flat face(s).")
    if cylinders:
        lines.append(f"Has {len(cylinders)} cylindrical surface(s).")
    if holes:
        lines.append(f"Has {len(holes)} hole(s).")

    return "\n".join(lines)
