"""Build a draft chain.yaml from specs.yaml and analysis data.

Automates the deterministic parts of chain construction:
- Part ordering from summary.yaml
- Joint limits from specs.yaml (deg → rad)
- Joint axes from bore detection (parent distal axis)
- Joint origins from connection points (proximal → distal)
- DH params dict for reference metadata
- Cross-validation of connection-point distances against DH params
- Preservation of user-added links from existing chain.yaml

Parts that still require LLM review are flagged with warnings:
- Joint origins when no connection points are available
- Part ordering for semantic (non-numbered) names
- origin_rpy for joints with non-zero DH alpha
"""

from __future__ import annotations

import math
import re
from pathlib import Path

from ..models.models import (
    Chain,
    ChainJoint,
    ChainLink,
    ConnectionPoint,
    PartAnalysis,
    load_chain_yaml,
    load_part_yaml,
    load_specs_yaml,
    load_summary_yaml,
)


def _deg_to_rad(deg: float) -> float:
    return round(deg * math.pi / 180, 4)


def _mm_to_m(mm: float) -> float:
    return round(mm / 1000, 6)


def _extract_link_number(name: str) -> int | None:
    """Extract numeric index from link names like 'link_2', 'A3', 'base_link'."""
    if name in ("base", "base_link"):
        return 0
    m = re.search(r"(\d+)", name)
    return int(m.group(1)) if m else None


def _is_numbered_naming(part_names: list[str]) -> bool:
    """Check if parts use numbered naming (link_0, A1, etc.)."""
    nums = [_extract_link_number(n) for n in part_names]
    return all(n is not None for n in nums)


def _order_parts_numbered(part_names: list[str]) -> list[str]:
    """Order parts by their numeric index."""
    return sorted(part_names, key=lambda n: _extract_link_number(n) or 0)


def _get_connection_point(analysis: PartAnalysis, end: str) -> ConnectionPoint | None:
    """Get proximal or distal connection point from analysis."""
    for cp in analysis.connection_points:
        if cp.end == end:
            return cp
    return None


def _build_dh_params_dict(specs_data: dict) -> dict[str, float]:
    """Build the flat dh_params dict for chain.yaml from specs.yaml DH table."""
    dh_list = specs_data.get("dh_params", [])
    result: dict[str, float] = {}
    for entry in dh_list:
        joint_num = entry["joint"]
        d = entry.get("d_mm", 0)
        a = entry.get("a_mm", 0)
        if abs(d) > 1e-6:
            result[f"d{joint_num}"] = d
        if abs(a) > 1e-6:
            result[f"a{joint_num}"] = a
    # Also include physical_offsets if present
    offsets = specs_data.get("physical_offsets", {})
    for key, val in offsets.items():
        if key not in result and abs(val) > 1e-6:
            result[key] = val
    return result


def _compute_origin_from_connections(
    parent_analysis: PartAnalysis,
    messages: list[str],
    joint_name: str,
) -> list[float] | None:
    """Compute joint origin from parent's proximal→distal connection points.

    Returns origin in metres, or None if connection points are missing.
    """
    proximal = _get_connection_point(parent_analysis, "proximal")
    distal = _get_connection_point(parent_analysis, "distal")

    if distal is None:
        return None

    if proximal is not None:
        # Origin = distal - proximal (in parent mesh coords), converted to metres
        origin = [_mm_to_m(distal.position[i] - proximal.position[i]) for i in range(3)]
    else:
        # No proximal — use distal position directly
        origin = [_mm_to_m(distal.position[i]) for i in range(3)]

    messages.append(f"  {joint_name}: origin from connection points = {origin}")
    return origin


def _compute_axis_from_connections(
    parent_analysis: PartAnalysis,
) -> list[float] | None:
    """Get joint axis from parent's distal bore axis."""
    distal = _get_connection_point(parent_analysis, "distal")
    if distal is None:
        return None
    # Round to clean unit vectors
    return [round(v, 4) for v in distal.axis]


def _cross_validate_origins(
    joints: list[ChainJoint],
    dh_list: list[dict],
    messages: list[str],
) -> None:
    """Compare connection-point-derived origins against DH parameter distances."""
    if not dh_list:
        return

    messages.append("\nCross-validation (connection points vs DH params):")

    for joint, dh in zip(joints, dh_list, strict=False):
        if joint.origin is None:
            continue

        origin = joint.origin
        dist_mm = math.sqrt(sum(v**2 for v in origin)) * 1000

        # Expected distance from DH: sqrt(a² + d²)
        d = abs(dh.get("d_mm", 0))
        a = abs(dh.get("a_mm", 0))
        expected_mm = math.sqrt(a**2 + d**2)

        if expected_mm < 1e-6:
            continue

        diff = abs(dist_mm - expected_mm)
        status = "OK" if diff < 10 else "REVIEW"
        messages.append(
            f"  {joint.name}: distance={dist_mm:.1f}mm,"
            f" DH expected={expected_mm:.1f}mm,"
            f" diff={diff:.1f}mm [{status}]"
        )
        if diff >= 10:
            messages.append(
                f"    WARNING: >10mm discrepancy — review origin for {joint.name}"
            )


def _flag_alpha_joints(
    joints: list[ChainJoint],
    dh_list: list[dict],
    messages: list[str],
) -> None:
    """Flag joints that need origin_rpy due to non-zero DH alpha."""
    for i, dh in enumerate(dh_list):
        alpha = dh.get("alpha_deg", 0)
        if abs(alpha) < 1e-6:
            continue
        if i < len(joints):
            joint = joints[i]
            alpha_rad = round(math.radians(alpha), 6)
            messages.append(
                f"\n  REVIEW: {joint.name} has DH alpha={alpha}deg"
                f" ({alpha_rad} rad)."
                f"\n    May need origin_rpy. Common pattern:"
                f" origin_rpy=[{alpha_rad}, 0, 0]"
                f"\n    The joint axis and origin placement may also"
                f" need adjustment for this frame rotation."
            )


def build_chain(robot_dir: Path) -> tuple[Chain, list[str]]:
    """Build a draft chain.yaml from specs.yaml and analysis data.

    Returns the Chain model and diagnostic messages.
    """
    messages: list[str] = []
    specs_path = robot_dir / "specs.yaml"
    summary_path = robot_dir / "analysis" / "summary.yaml"
    analysis_dir = robot_dir / "analysis"
    existing_chain_path = robot_dir / "chain.yaml"

    # --- Load inputs ---
    if not specs_path.exists():
        raise FileNotFoundError(f"specs.yaml not found at {specs_path}")
    if not summary_path.exists():
        raise FileNotFoundError(f"summary.yaml not found at {summary_path}")

    load_specs_yaml(specs_path)  # validate schema
    # Re-load raw for extra fields (Specs uses extra="allow")
    import yaml

    with open(specs_path) as f:
        specs_raw = yaml.safe_load(f)

    summary = load_summary_yaml(summary_path)
    dof = specs_raw.get("dof", 6)
    dh_list: list[dict] = specs_raw.get("dh_params", [])
    joint_limits_list: list[dict] = specs_raw.get("joint_limits", [])

    messages.append(f"Building chain for {summary.robot_name} (DOF={dof})")

    # --- Load per-part analyses ---
    analyses: dict[str, PartAnalysis] = {}
    for part in summary.parts:
        yaml_path = analysis_dir / part.file
        if yaml_path.exists():
            analyses[part.name] = load_part_yaml(yaml_path)

    # --- Determine part ordering ---
    part_names = [p.name for p in summary.parts]
    if _is_numbered_naming(part_names):
        ordered = _order_parts_numbered(part_names)
        messages.append(f"Part ordering (numbered): {ordered}")
    else:
        # Semantic names — can't reliably order automatically
        ordered = part_names
        messages.append(
            f"\n  REVIEW: Parts use semantic names: {part_names}"
            f"\n    Cannot auto-order — please verify/reorder."
            f"\n    Hint: look at bounding box sizes and role_hints in summary.yaml"
        )

    # --- Build DH params dict ---
    dh_params_dict = _build_dh_params_dict(specs_raw)

    # --- Build links ---
    links: list[ChainLink] = []
    for name in ordered:
        links.append(ChainLink(name=name, mesh=name))

    # --- Build joints ---
    joints: list[ChainJoint] = []
    n_joints = min(len(links) - 1, dof)

    for i in range(n_joints):
        parent = links[i]
        child = links[i + 1]
        joint_name = f"joint_{i + 1}"

        # Joint limits from specs
        limits: list[float] | None = None
        if i < len(joint_limits_list):
            jl = joint_limits_list[i]
            limits = [
                _deg_to_rad(jl.get("min_deg", -360)),
                _deg_to_rad(jl.get("max_deg", 360)),
            ]

        # Joint axis from parent's distal bore
        axis: list[float] = [0, 0, 1]  # default
        parent_analysis = analyses.get(parent.name)
        if parent_analysis:
            detected_axis = _compute_axis_from_connections(parent_analysis)
            if detected_axis:
                axis = detected_axis
                messages.append(
                    f"  {joint_name}: axis from {parent.name} distal bore = {axis}"
                )
            else:
                messages.append(
                    f"  {joint_name}: no distal bore on {parent.name},"
                    f" using default axis [0,0,1]"
                )

        # Joint origin from parent's connection points
        origin: list[float] | None = None
        if parent_analysis:
            origin = _compute_origin_from_connections(
                parent_analysis, messages, joint_name
            )

        if origin is None:
            # Fall back: try to derive from DH params
            if i < len(dh_list):
                dh = dh_list[i]
                d = dh.get("d_mm", 0)
                a = dh.get("a_mm", 0)
                # Simple vertical stacking — works for alpha=0 robots
                origin = [_mm_to_m(a), 0, _mm_to_m(d)]
                messages.append(
                    f"  {joint_name}: origin from DH fallback [a={a}, d={d}] = {origin}"
                )
                messages.append(
                    "    REVIEW: This assumes simple vertical stacking."
                    " Check if DH alpha requires different axis placement."
                )
            else:
                messages.append(f"  {joint_name}: WARNING — no origin data available!")

        joints.append(
            ChainJoint(
                name=joint_name,
                type="revolute",
                parent=parent.name,
                child=child.name,
                axis=axis,
                limits=limits,
                origin=origin,
            )
        )

    # --- Cross-validate ---
    _cross_validate_origins(joints, dh_list, messages)

    # --- Flag alpha rotations needing review ---
    _flag_alpha_joints(joints, dh_list, messages)

    # --- Preserve user-added links/joints from existing chain ---
    user_links: list[ChainLink] = []
    user_joints: list[ChainJoint] = []
    if existing_chain_path.exists():
        existing = load_chain_yaml(existing_chain_path)
        user_links = [lk for lk in existing.links if lk.user_added]
        user_joints = [jt for jt in existing.joints if jt.user_added]
        if user_links or user_joints:
            messages.append(
                f"\nPreserved {len(user_links)} user-added link(s)"
                f" and {len(user_joints)} user-added joint(s)"
            )

    links.extend(user_links)
    joints.extend(user_joints)

    chain = Chain(
        robot_name=summary.robot_name,
        dh_params=dh_params_dict if dh_params_dict else None,
        links=links,
        joints=joints,
    )

    # --- Final validation ---
    if len(joints) != dof:
        messages.append(
            f"\n  WARNING: Generated {len(joints)} joints but specs say DOF={dof}"
        )

    return chain, messages
