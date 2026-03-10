"""YAML output for part analysis results."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml

from robot_arm_sim.models.part import ConnectionPoint, GeometricFeature, PartAnalysis


def write_part_yaml(analysis: PartAnalysis, output_path: Path) -> None:
    """Write a single part analysis to YAML."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    data = _part_to_dict(analysis)
    with open(output_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, width=120)


def write_summary_yaml(
    robot_name: str,
    analyses: list[PartAnalysis],
    output_path: Path,
) -> None:
    """Write assembly summary YAML."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    parts = []
    for a in analyses:
        role_hint = _infer_role_hint(a)
        parts.append(
            {
                "name": a.part_name,
                "file": f"{a.part_name}.yaml",
                "role_hint": role_hint,
            }
        )

    assembly_hints = _generate_assembly_hints(analyses)

    summary = {
        "robot_name": robot_name,
        "part_count": len(analyses),
        "parts": parts,
        "assembly_hints": assembly_hints,
    }

    with open(output_path, "w") as f:
        yaml.dump(summary, f, default_flow_style=False, sort_keys=False, width=120)


def _part_to_dict(analysis: PartAnalysis) -> dict[str, Any]:
    """Convert a PartAnalysis to a YAML-friendly dict."""
    features_dict: dict[str, list[dict]] = {
        "flat_faces": [],
        "cylindrical_surfaces": [],
        "holes": [],
    }

    for f in analysis.features:
        fd = _feature_to_dict(f)
        if f.kind == "flat_face":
            features_dict["flat_faces"].append(fd)
        elif f.kind == "cylindrical_surface":
            features_dict["cylindrical_surfaces"].append(fd)
        elif f.kind == "hole":
            features_dict["holes"].append(fd)

    conn_points = [_connection_point_to_dict(cp) for cp in analysis.connection_points]

    return {
        "part_name": analysis.part_name,
        "source_file": analysis.source_file,
        "format": analysis.format,
        "connection_points": conn_points,
        "geometry": {
            "vertex_count": analysis.vertex_count,
            "face_count": analysis.face_count,
            "bounding_box": {
                "min": analysis.bounding_box_min,
                "max": analysis.bounding_box_max,
                "extents": analysis.bounding_box_extents,
            },
            "volume_mm3": analysis.volume_mm3,
            "surface_area_mm2": analysis.surface_area_mm2,
            "center_of_mass": analysis.center_of_mass,
            "is_watertight": analysis.is_watertight,
        },
        "inertia": {
            "principal_moments": analysis.principal_moments,
            "principal_axes": analysis.principal_axes,
        },
        "features": features_dict,
        "text_description": analysis.text_description,
        "renders": analysis.render_paths,
    }


def _connection_point_to_dict(cp: ConnectionPoint) -> dict[str, Any]:
    """Convert a ConnectionPoint to a dict."""
    return {
        "end": cp.end,
        "position": [float(v) for v in cp.position],
        "axis": [float(v) for v in cp.axis],
        "radius_mm": float(cp.radius_mm),
        "method": cp.method,
    }


def _feature_to_dict(f: GeometricFeature) -> dict[str, Any]:
    """Convert a GeometricFeature to a dict, omitting None fields."""
    d: dict[str, Any] = {"description": f.description}
    if f.normal is not None:
        d["normal"] = f.normal
    if f.axis is not None:
        d["axis"] = f.axis
    if f.area_mm2 is not None:
        d["area_mm2"] = f.area_mm2
    if f.radius_mm is not None:
        d["radius_mm"] = f.radius_mm
    if f.length_mm is not None:
        d["length_mm"] = f.length_mm
    if f.center is not None:
        d["center"] = f.center
    if f.centroid is not None:
        d["centroid"] = f.centroid
    if f.concave is not None:
        d["concave"] = f.concave
    return d


def _infer_role_hint(analysis: PartAnalysis) -> str:
    """Generate a brief role hint for the summary."""
    hints = []

    # Size-based hints
    extents = analysis.bounding_box_extents
    if extents:
        max_extent = max(extents)
        if max_extent > 100:
            hints.append("large part")

    # Feature-based hints
    flat_faces = [f for f in analysis.features if f.kind == "flat_face"]
    cylinders = [f for f in analysis.features if f.kind == "cylindrical_surface"]

    if flat_faces:
        biggest = max(flat_faces, key=lambda f: f.area_mm2 or 0)
        if biggest.normal and abs(biggest.normal[2]) > 0.9 and biggest.normal[2] < 0:
            hints.append("flat bottom face (possible base)")

    if cylinders:
        hints.append(f"{len(cylinders)} cylindrical surface(s)")

    name = analysis.part_name
    match = re.search(r"(\d+)", name)
    if match and int(match.group(1)) == 0:
        hints.append("first part in sequence (likely base)")
    if "_" in name:
        hints.append("combined part (may span multiple joints)")

    return "; ".join(hints) if hints else "standard link"


def _generate_assembly_hints(analyses: list[PartAnalysis]) -> list[str]:
    """Generate assembly-level hints from all parts."""
    hints = []
    names = sorted(a.part_name for a in analyses)
    hints.append(f"Parts: {', '.join(names)}")

    # Check for sequential naming pattern (any prefix)
    seq_hint = _detect_sequential_pattern(names)
    if seq_hint:
        hints.append(seq_hint)

    # Check for combined parts
    combined = [n for n in names if "_" in n]
    if combined:
        hints.append(
            f"{', '.join(combined)} are combined parts, may span multiple joints"
        )

    return hints


def _detect_sequential_pattern(names: list[str]) -> str | None:
    """Detect if part names share a prefix with sequential numbers."""
    pattern = re.compile(r"^(.*?)(\d+)$")
    parsed = []
    for n in names:
        base = n.split("_")[0]  # handle combined parts like A3_4
        m = pattern.match(base)
        if m:
            parsed.append((m.group(1), int(m.group(2))))

    if len(parsed) < 2:
        return None

    prefixes = {p for p, _ in parsed}
    if len(prefixes) == 1:
        nums = sorted(n for _, n in parsed)
        if nums[-1] - nums[0] <= len(names) + 1:
            prefix = prefixes.pop()
            return (
                f"Parts follow sequential naming ({prefix}*), "
                "suggesting serial kinematic chain"
            )
    return None
