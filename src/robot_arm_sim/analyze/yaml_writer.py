"""YAML output for part analysis results."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from robot_arm_sim.models import (
    ConnectionPoint,
    PartAnalysis,
    Summary,
    SummaryPart,
    save_part_yaml,
    save_summary_yaml,
)


def write_part_yaml(
    analysis: PartAnalysis,
    output_path: Path,
    *,
    manual_connection_points: list[dict[str, Any]] | None = None,
) -> None:
    """Write a single part analysis to YAML.

    If *manual_connection_points* is provided, those raw dicts replace any
    auto-detected connection points for the same end (proximal/distal).
    This preserves hand-tuned connection point placements across re-analysis.
    """
    if manual_connection_points:
        manual_ends = {cp["end"] for cp in manual_connection_points}
        # Keep auto-detected CPs whose end wasn't manually placed
        kept = [cp for cp in analysis.connection_points if cp.end not in manual_ends]
        # Append manual CPs (raw dicts, may include extra keys like 'center')
        for cp_dict in manual_connection_points:
            kept.append(ConnectionPoint.model_validate(cp_dict))
        analysis.connection_points = kept

    save_part_yaml(analysis, output_path)


def write_summary_yaml(
    robot_name: str,
    analyses: list[PartAnalysis],
    output_path: Path,
) -> None:
    """Write assembly summary YAML."""
    parts = []
    for a in analyses:
        role_hint = _infer_role_hint(a)
        parts.append(
            SummaryPart(
                name=a.part_name,
                file=f"{a.part_name}.yaml",
                role_hint=role_hint,
            )
        )

    assembly_hints = _generate_assembly_hints(analyses)

    model = Summary(
        robot_name=robot_name,
        part_count=len(analyses),
        parts=parts,
        assembly_hints=assembly_hints,
    )

    save_summary_yaml(model, output_path)


def _infer_role_hint(analysis: PartAnalysis) -> str:
    """Generate a brief role hint for the summary."""
    hints = []

    # Size-based hints
    extents = analysis.geometry.bounding_box.extents
    if extents:
        max_extent = max(extents)
        if max_extent > 100:
            hints.append("large part")

    # Feature-based hints
    flat_faces = analysis.features.flat_faces
    cylinders = analysis.features.cylindrical_surfaces

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
