"""Programmatic chain.yaml builder from specs.yaml + analysis data.

Converts DH parameters to URDF-compatible joint origins using the
P/Q decomposition:

    T_i = Rz(θ_off) · Tz(d) · Tx(a) · Rx(α)  =  P_i · Rz(q) · Q_i

where P_i = Rz(θ_off) · Tz(d) and Q_i = Tx(a) · Rx(α).

URDF joint i origin = Q_{i-1} · P_i, with axis always [0, 0, 1].
This preserves exact DH kinematics in a valid URDF parameterisation.
"""

from __future__ import annotations

import logging
import math
import re
from pathlib import Path

import numpy as np

from .models.models import (
    Chain,
    ChainJoint,
    ChainLink,
    _load_yaml,
    load_specs_yaml,
    load_summary_yaml,
    save_chain_yaml,
)

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Low-level homogeneous-transform helpers
# ---------------------------------------------------------------------------


def _rz(theta: float) -> np.ndarray:
    """Rotation about Z."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        dtype=float,
    )


def _rx(alpha: float) -> np.ndarray:
    """Rotation about X."""
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array(
        [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]],
        dtype=float,
    )


def _tz(d: float) -> np.ndarray:
    """Translation along Z."""
    m = np.eye(4)
    m[2, 3] = d
    return m


def _tx(a: float) -> np.ndarray:
    """Translation along X."""
    m = np.eye(4)
    m[0, 3] = a
    return m


def _rotation_to_rpy(r: np.ndarray) -> list[float]:
    """Convert 3×3 rotation matrix to [roll, pitch, yaw]."""
    sy = float(np.clip(-r[2, 0], -1.0, 1.0))
    pitch = float(np.arcsin(sy))
    if abs(sy) < 1.0 - 1e-6:
        roll = float(np.arctan2(r[2, 1], r[2, 2]))
        yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    else:
        roll = float(np.arctan2(-r[1, 2], r[1, 1]))
        yaw = 0.0
    return [roll, pitch, yaw]


# ---------------------------------------------------------------------------
# DH → URDF joint parameter computation
# ---------------------------------------------------------------------------


def dh_to_urdf_joints(
    dh_rows: list[dict],
) -> list[dict]:
    """Convert DH parameter rows to URDF joint origin/rpy/axis dicts.

    Each returned dict has keys:
        joint_index (1-based), origin (metres), origin_rpy, axis.

    Uses the P/Q decomposition so that all URDF axes are [0, 0, 1]:
        P_i = Rz(θ_offset) · Tz(d)
        Q_i = Tx(a) · Rx(α)
        URDF_origin_i = Q_{i-1} · P_i
    """
    results: list[dict] = []
    q_prev = np.eye(4)

    for row in dh_rows:
        d_mm = float(row["d"])
        a_mm = float(row["a"])
        alpha = float(row["alpha"])
        theta_off = float(row.get("theta_offset", 0.0))

        p = _rz(theta_off) @ _tz(d_mm)
        q = _tx(a_mm) @ _rx(alpha)

        origin_mat = q_prev @ p
        xyz_mm = origin_mat[:3, 3].tolist()
        xyz_m = [round(v / 1000.0, 6) for v in xyz_mm]
        rpy = _rotation_to_rpy(origin_mat[:3, :3])
        rpy_clean = [round(v, 6) if abs(v) > 1e-6 else 0.0 for v in rpy]

        results.append(
            {
                "joint_index": int(row["joint"]),
                "origin": xyz_m,
                "origin_rpy": rpy_clean,
                "axis": [0, 0, 1],
            }
        )
        q_prev = q

    return results


# ---------------------------------------------------------------------------
# Part-to-link mapping
# ---------------------------------------------------------------------------

_COMBINED_RE = re.compile(r"^[A-Za-z]*(\d+)_(\d+)$")


def _detect_combined_parts(parts: list[dict]) -> dict[str, list[int]]:
    """Detect combined parts like A3_4 that span multiple joints.

    Returns {part_name: [joint_indices]} for combined parts.
    """
    combined: dict[str, list[int]] = {}
    for part in parts:
        m = _COMBINED_RE.match(part["name"])
        if m:
            indices = list(range(int(m.group(1)), int(m.group(2)) + 1))
            combined[part["name"]] = indices
    return combined


def _build_links_and_mesh_map(
    parts: list[dict],
    dof: int,
    combined: dict[str, list[int]],
) -> list[ChainLink]:
    """Build link list from parts and DOF.

    Returns list of ChainLink. Combined parts get a virtual link (mesh=None)
    for all but the last joint index in their span.
    """
    # Base link is always the first part
    links: list[ChainLink] = [ChainLink(name="base_link", mesh=parts[0]["name"])]

    # Figure out which joint indices need virtual links
    virtual_joints: set[int] = set()
    for joint_indices in combined.values():
        # All but the last index in the span get virtual links
        for idx in joint_indices[:-1]:
            virtual_joints.add(idx)

    non_base_parts = parts[1:]
    part_iter = iter(non_base_parts)
    for joint_idx in range(1, dof + 1):
        link_name = f"link_{joint_idx}"
        if joint_idx in virtual_joints:
            links.append(ChainLink(name=link_name, mesh=None))
        else:
            try:
                part = next(part_iter)
            except StopIteration:
                links.append(ChainLink(name=link_name, mesh=None))
                continue

            # Skip EndEffector-like parts (handled separately)
            if part["name"].lower() in ("endeffector", "end_effector"):
                links.append(ChainLink(name=link_name, mesh=None))
                continue

            links.append(ChainLink(name=link_name, mesh=part["name"]))

    return links


# ---------------------------------------------------------------------------
# DH params summary for chain.yaml header
# ---------------------------------------------------------------------------


def _extract_dh_summary(dh_rows: list[dict]) -> dict[str, float]:
    """Extract named DH distance parameters for the chain.yaml header.

    Only includes non-zero d and a values, named as d<i> or a<i>.
    """
    summary: dict[str, float] = {}
    for row in dh_rows:
        j = int(row["joint"])
        d = float(row["d"])
        a = float(row["a"])
        if abs(d) > 0.01:
            summary[f"d{j}"] = d
        if abs(a) > 0.01:
            summary[f"a{j}"] = a
    return summary


# ---------------------------------------------------------------------------
# Cross-validation
# ---------------------------------------------------------------------------


def _cross_validate(
    urdf_joints: list[dict],
    total_reach_mm: float | None,
    dof: int,
    messages: list[str],
) -> None:
    """Validate chain against specs."""
    # Joint count
    if len(urdf_joints) != dof:
        messages.append(f"WARNING: joint count {len(urdf_joints)} != DOF {dof}")

    # Sum of joint-to-joint distances
    total_dist = 0.0
    for uj in urdf_joints:
        dist = sum(v**2 for v in uj["origin"]) ** 0.5
        total_dist += dist
    total_dist_mm = total_dist * 1000

    if total_reach_mm is not None:
        ratio = total_dist_mm / total_reach_mm if total_reach_mm > 0 else 0
        if abs(ratio - 1.0) > 0.3:
            messages.append(
                f"WARNING: sum of joint distances {total_dist_mm:.1f}mm"
                f" differs significantly from total reach {total_reach_mm}mm"
                f" (ratio={ratio:.2f})"
            )
        else:
            messages.append(
                f"  Joint distance sum: {total_dist_mm:.1f}mm"
                f" vs reach: {total_reach_mm}mm (ratio={ratio:.2f})"
            )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def _normalise_dh_rows(rows: list[dict]) -> list[dict]:
    """Normalise DH rows to canonical keys: joint, d, a, alpha, theta_offset.

    Handles two variants:
      - d/a/alpha/theta_offset (mm, radians) — Meca500 style
      - d_mm/a_mm/alpha_deg/theta_offset_deg — ABB/FANUC/UR5/Kinova style
    """
    out: list[dict] = []
    for row in rows:
        if "d" in row and "alpha" in row:
            # Already in canonical form (mm, radians)
            out.append(row)
        elif "d_mm" in row:
            out.append(
                {
                    "joint": row["joint"],
                    "d": float(row["d_mm"]),
                    "a": float(row["a_mm"]),
                    "alpha": math.radians(float(row["alpha_deg"])),
                    "theta_offset": math.radians(
                        float(row.get("theta_offset_deg", 0.0))
                    ),
                }
            )
        else:
            out.append(row)
    return out


def _normalise_limits(lim: dict) -> tuple[float | None, float | None]:
    """Extract joint limits in radians from various key conventions."""
    # lower/upper (radians)
    lower = lim.get("lower")
    upper = lim.get("upper")
    if lower is not None and upper is not None:
        return float(lower), float(upper)

    # lower_rad/upper_rad
    lower = lim.get("lower_rad")
    upper = lim.get("upper_rad")
    if lower is not None and upper is not None:
        return float(lower), float(upper)

    # min_deg/max_deg
    lower_deg = lim.get("min_deg")
    upper_deg = lim.get("max_deg")
    if lower_deg is not None and upper_deg is not None:
        return math.radians(float(lower_deg)), math.radians(float(upper_deg))

    return None, None


def build_chain(
    specs_path: Path,
    summary_path: Path,
    existing_chain_path: Path | None = None,
) -> tuple[Chain, list[str]]:
    """Build a Chain model from specs.yaml and summary.yaml.

    Args:
        specs_path: Path to the robot's specs.yaml.
        summary_path: Path to the robot's analysis/summary.yaml.
        existing_chain_path: If set, user-added links/joints are preserved.

    Returns:
        (chain, messages) tuple.
    """
    messages: list[str] = []

    # Load inputs
    specs_data = _load_yaml(specs_path)
    load_specs_yaml(specs_path)  # validate against schema
    summary = load_summary_yaml(summary_path)

    robot_name = str(specs_data.get("robot_name", "robot"))
    dof: int = specs_data.get("dof", 6)
    dh_rows_raw: list[dict] = (
        specs_data.get("dh_parameters") or specs_data.get("dh_params") or []
    )
    joint_limits: list[dict] = specs_data.get("joint_limits", [])
    total_reach: float | None = specs_data.get("total_reach_mm")

    # Normalise DH rows to canonical keys: d, a, alpha, theta_offset (all mm/rad)
    dh_rows = _normalise_dh_rows(dh_rows_raw)

    if not dh_rows:
        raise ValueError(f"No DH parameters in {specs_path}")

    messages.append(f"Building chain for {robot_name} ({dof} DOF)")

    # Compute URDF joint parameters from DH
    urdf_joints = dh_to_urdf_joints(dh_rows)
    messages.append(f"  Computed {len(urdf_joints)} joint origins from DH")

    # Build limit lookup
    limit_map: dict[int, list[float]] = {}
    for lim in joint_limits:
        lower, upper = _normalise_limits(lim)
        if lower is not None and upper is not None:
            limit_map[int(lim["joint"])] = [lower, upper]

    # Detect combined parts and build links
    parts = [{"name": p.name, "role_hint": p.role_hint} for p in summary.parts]
    combined = _detect_combined_parts(parts)
    if combined:
        messages.append(f"  Combined parts: {combined}")

    links = _build_links_and_mesh_map(parts, dof, combined)

    # Build joints
    joints: list[ChainJoint] = []
    for uj in urdf_joints:
        j_idx = uj["joint_index"]
        parent = links[j_idx - 1].name  # link_{j-1} or base_link
        child = links[j_idx].name

        rpy = uj["origin_rpy"]
        has_rpy = any(abs(v) > 1e-6 for v in rpy)

        joint = ChainJoint(
            name=f"joint_{j_idx}",
            type="revolute",
            parent=parent,
            child=child,
            origin=uj["origin"],
            origin_rpy=rpy if has_rpy else None,
            axis=uj["axis"],
            limits=limit_map.get(j_idx),
        )
        joints.append(joint)

    # DH summary for header
    dh_summary = _extract_dh_summary(dh_rows)

    # Cross-validate
    _cross_validate(urdf_joints, total_reach, dof, messages)

    # Preserve user-added links/joints from existing chain
    if existing_chain_path and existing_chain_path.exists():
        from .models.models import load_chain_yaml

        existing = load_chain_yaml(existing_chain_path)
        for link in existing.links:
            if link.user_added:
                links.append(link)
                messages.append(f"  Preserved user-added link: {link.name}")
        for joint in existing.joints:
            if joint.user_added:
                joints.append(joint)
                messages.append(f"  Preserved user-added joint: {joint.name}")

    chain = Chain(
        robot_name=robot_name,
        dh_params=dh_summary,
        links=links,
        joints=joints,
    )

    return chain, messages


def build_chain_cli(
    robot_dir: Path,
    output: Path | None = None,
) -> list[str]:
    """CLI entry point: build chain.yaml for a robot directory.

    Expects robot_dir to contain specs.yaml and analysis/summary.yaml.
    Writes chain.yaml to output (default: robot_dir/chain.yaml).
    """
    specs_path = robot_dir / "specs.yaml"
    summary_path = robot_dir / "analysis" / "summary.yaml"
    chain_path = output or (robot_dir / "chain.yaml")

    existing = chain_path if chain_path.exists() else None

    chain, messages = build_chain(specs_path, summary_path, existing)
    save_chain_yaml(chain, chain_path)
    messages.append(f"Wrote {chain_path}")

    return messages
