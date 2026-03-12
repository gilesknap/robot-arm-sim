"""Analysis module: parse meshes, detect features, render, write YAML."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import trimesh

from robot_arm_sim.models import Features, PartAnalysis, load_part_yaml

from .connections import detect_connection_points
from .features import detect_features
from .parsers import get_parser
from .yaml_writer import write_part_yaml, write_summary_yaml

logger = logging.getLogger(__name__)


def _load_manual_connection_points(
    yaml_path: Path,
) -> list[dict[str, Any]]:
    """Load manually-placed connection points from an existing analysis YAML."""
    if not yaml_path.exists():
        return []
    try:
        model = load_part_yaml(yaml_path)
    except Exception:
        return []
    return [cp.model_dump() for cp in model.connection_points if cp.method == "manual"]


def run_analysis(robot_dir: Path, *, override_manual: bool = False) -> None:
    """Run full analysis pipeline on a robot directory.

    Expects STL files in <robot_dir>/stl_files/.
    Outputs analysis YAML to <robot_dir>/analysis/.

    By default, manually-placed connection points from existing analysis
    files are preserved.  Pass ``override_manual=True`` to re-detect all
    connection points from scratch.
    """
    stl_dir = robot_dir / "stl_files"
    if not stl_dir.exists():
        raise FileNotFoundError(f"No stl_files/ directory found in {robot_dir}")

    stl_files = sorted(stl_dir.glob("*.stl")) + sorted(stl_dir.glob("*.STL"))
    if not stl_files:
        raise FileNotFoundError(f"No STL files found in {stl_dir}")

    analysis_dir = robot_dir / "analysis"
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
        feature_list = detect_features(mesh)
        analysis.features = Features.from_feature_list(feature_list)

        # Check for existing manual connection points
        yaml_path = analysis_dir / f"{analysis.part_name}.yaml"
        manual_cps = (
            [] if override_manual else _load_manual_connection_points(yaml_path)
        )
        manual_ends = {cp["end"] for cp in manual_cps}

        # Detect connection points
        print(f"  Detecting connection points for {stl_file.name}...")
        conn_points = detect_connection_points(mesh, feature_list, analysis.part_name)
        analysis.connection_points = conn_points
        if conn_points:
            for cp in conn_points:
                tag = " (overridden by manual)" if cp.end in manual_ends else ""
                print(
                    f"    {cp.end}: pos={cp.position} "
                    f"axis={cp.axis} r={cp.radius_mm}mm ({cp.method}){tag}"
                )
        if manual_cps:
            for cp in manual_cps:
                print(
                    f"    {cp['end']}: pos={cp['position']} "
                    f"axis={cp['axis']} r={cp['radius_mm']}mm "
                    f"(manual — preserved)"
                )

        # Generate text description
        analysis.text_description = _generate_text_description(analysis)

        # Write part YAML, passing manual overrides to splice in
        write_part_yaml(analysis, yaml_path, manual_connection_points=manual_cps)
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

    ext = analysis.geometry.bounding_box.extents
    lines.append(f"Bounding box: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm.")

    if analysis.geometry.is_watertight:
        lines.append(
            f"Watertight solid, volume {analysis.geometry.volume_mm3:.1f} mm³."
        )
    else:
        lines.append("Non-watertight mesh (volume not computed).")

    lines.append(f"Surface area: {analysis.geometry.surface_area_mm2:.1f} mm².")
    lines.append(
        f"Mesh complexity: {analysis.geometry.vertex_count} vertices, "
        f"{analysis.geometry.face_count} faces."
    )

    n_flat = len(analysis.features.flat_faces)
    n_cyl = len(analysis.features.cylindrical_surfaces)
    n_holes = len(analysis.features.holes)

    if n_flat:
        lines.append(f"Has {n_flat} significant flat face(s).")
    if n_cyl:
        lines.append(f"Has {n_cyl} cylindrical surface(s).")
    if n_holes:
        lines.append(f"Has {n_holes} hole(s).")

    return "\n".join(lines)
