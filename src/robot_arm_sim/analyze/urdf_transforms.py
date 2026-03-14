"""Transform computation helpers for URDF generation.

Computes visual origins, joint origins, and FK validation for the
URDF generation pipeline.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _find_opposite_face(
    analysis: dict,
    bore_pos: list[float],
    bore_axis: list[float],
    axis_idx: int,
    max_depth: float | None = None,
    require_far_side: bool = False,
) -> float | None:
    """Find the opposite face position along the bore axis.

    Returns the bore-axis coordinate of the centroid of the flat face
    whose normal opposes the bore axis and whose non-bore-axis centroid
    is closest to the bore position.  This selects the face directly
    "across" from the bore rather than a distant parallel face.

    If *max_depth* is given, candidates further than that distance along
    the bore axis are rejected (prevents matching faces deep inside
    irregular parts where the bore is a shallow surface feature).

    If *require_far_side* is True, only faces whose centroid is on the
    far side of the marker along the bore axis are considered (prevents
    matching the face the marker sits on).
    """
    faces = analysis.get("features", {}).get("flat_faces", [])
    if not faces:
        return None
    bore_sign = 1.0 if bore_axis[axis_idx] > 0 else -1.0
    cross = [i for i in range(3) if i != axis_idx]
    best: float | None = None
    best_dist = float("inf")
    for face in faces:
        normal = face.get("normal", [0, 0, 0])
        if abs(normal[axis_idx]) < 0.5:
            continue
        face_sign = 1.0 if normal[axis_idx] > 0 else -1.0
        if face_sign == bore_sign:
            continue
        centroid = face.get("centroid")
        if centroid is None:
            continue
        along = centroid[axis_idx] - bore_pos[axis_idx]
        # Reject faces on the marker's side of the bore axis
        if require_far_side and bore_sign * along <= 0:
            continue
        # Reject faces too far along the bore axis
        if max_depth is not None and abs(along) > max_depth:
            continue
        # Distance in the non-bore-axis plane
        dist = sum((centroid[i] - bore_pos[i]) ** 2 for i in cross) ** 0.5
        if dist < best_dist:
            best_dist = dist
            best = centroid[axis_idx]
    return best


def compute_visual_origin(
    analysis: dict,
    link_spec: dict,
    link_name: str,
    *,
    messages: list[str],
) -> tuple[list[float], list[float]]:
    """Compute visual origin xyz/rpy: place proximal connection at frame origin.

    Translates the mesh so the proximal bore sits on the joint axis.
    For center mode, averages with the opposite face along the bore axis.
    """
    conn_points = analysis.get("connection_points", [])
    proximal = next((cp for cp in conn_points if cp["end"] == "proximal"), None)
    viz_rpy = link_spec.get("visual_rpy", [0, 0, 0])

    if proximal is None:
        viz_xyz = link_spec.get("visual_xyz", [0, 0, 0])
        messages.append(f"  {link_name}: no proximal connection point")
        return viz_xyz, viz_rpy

    pos = np.array(proximal["position"], dtype=float)  # mm, mesh coords

    # For center mode, average proximal with opposite face along bore axis
    bore_axis = proximal.get("axis", [0, 0, 0])
    axis_idx = max(range(3), key=lambda i: abs(bore_axis[i]))
    centering = proximal.get("centering", "surface")

    if centering == "center" and abs(bore_axis[axis_idx]) > 0.5:
        opp = _find_opposite_face(analysis, pos.tolist(), bore_axis, axis_idx)
        if opp is not None:
            pos = pos.copy()
            pos[axis_idx] = (pos[axis_idx] + opp) / 2

    # Visual origin = -proximal/1000: places proximal at frame origin (on axis)
    messages.append(
        f"  {link_name}: proximal @ origin,"
        f" proximal=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})mm"
    )
    if viz_rpy == [0, 0, 0]:
        viz_xyz = (-pos / 1000).tolist()
    else:
        viz_xyz = (-rpy_to_rotation(viz_rpy) @ (pos / 1000)).tolist()

    extra = link_spec.get("visual_xyz")
    if extra is not None:
        viz_xyz = [v + e for v, e in zip(viz_xyz, extra, strict=True)]
    return [round(v, 6) for v in viz_xyz], viz_rpy


def compute_joint_origin(
    joint_spec: dict,
    link_specs: dict,
    analyses: dict,
    dh_params: dict,
    messages: list[str],
) -> list[float]:
    """Compute joint origin xyz in parent frame.

    Uses parent mesh's distal connection point if available,
    falling back to explicit origin in chain spec.
    """
    parent_name = joint_spec["parent"]
    parent_spec = link_specs.get(parent_name, {})
    parent_mesh = parent_spec.get("mesh")

    if "origin" in joint_spec:
        return joint_spec["origin"]

    if parent_mesh and parent_mesh in analyses:
        analysis = analyses[parent_mesh]
        conn_points = analysis.get("connection_points", [])
        distal = next(
            (cp for cp in conn_points if cp["end"] == "distal"),
            None,
        )

        if distal is not None:
            pos = distal["position"]
            messages.append(
                f"  {joint_spec['name']}: distal="
                f"({pos[0]:.1f}, {pos[1]:.1f},"
                f" {pos[2]:.1f})mm"
            )
            proximal = next(
                (cp for cp in conn_points if cp["end"] == "proximal"),
                None,
            )

            if proximal is not None:
                pp = proximal["position"]
                xyz = [
                    (pos[0] - pp[0]) / 1000,
                    (pos[1] - pp[1]) / 1000,
                    (pos[2] - pp[2]) / 1000,
                ]
            else:
                xyz = [
                    pos[0] / 1000,
                    pos[1] / 1000,
                    pos[2] / 1000,
                ]

            parent_rpy = parent_spec.get("visual_rpy", [0, 0, 0])
            if parent_rpy != [0, 0, 0]:
                rot = rpy_to_rotation(parent_rpy)
                xyz = (rot @ np.array(xyz)).tolist()

            rounded = [round(v, 4) for v in xyz]
            messages.append(
                f"  {joint_spec['name']}: origin from connection points = {rounded}"
            )
            return [round(v, 6) for v in xyz]

    messages.append(
        f"  {joint_spec['name']}: no connection point data, using chain spec fallback"
    )
    return joint_spec.get("origin", [0, 0, 0])


def close_surface_gaps_along_axis(
    chain: dict,
    analyses: dict,
    visual_origins: dict[str, tuple[list[float], list[float]]],
    joint_origins: dict[str, list[float]],
    joint_rpys: dict[str, list[float]],
    messages: list[str],
) -> None:
    """Shift surface-mode children along the joint axis to meet parent distal.

    Only adjusts along the joint axis — cross-axis is already handled by
    the per-mesh snap in compute_visual_origin.
    """
    link_specs = {lk["name"]: lk for lk in chain["links"]}

    for joint_spec in chain["joints"]:
        child_name = joint_spec["child"]
        parent_name = joint_spec["parent"]
        joint_name = joint_spec["name"]
        axis = np.array(joint_spec["axis"], dtype=float)
        axis = axis / (np.linalg.norm(axis) + 1e-12)

        # Only surface-mode children
        child_mesh = link_specs.get(child_name, {}).get("mesh")
        if not child_mesh or child_mesh not in analyses:
            continue
        child_cps = analyses[child_mesh].get("connection_points", [])
        child_prox = next((cp for cp in child_cps if cp["end"] == "proximal"), None)
        if child_prox is None:
            continue
        if child_prox.get("centering", "surface") != "surface":
            continue

        # Parent distal in parent frame
        parent_mesh = link_specs.get(parent_name, {}).get("mesh")
        if not parent_mesh or parent_mesh not in analyses:
            continue
        parent_cps = analyses[parent_mesh].get("connection_points", [])
        parent_dist = next((cp for cp in parent_cps if cp["end"] == "distal"), None)
        if parent_dist is None:
            continue

        parent_viz_xyz, parent_viz_rpy = visual_origins[parent_name]
        dp = np.array(parent_dist["position"]) * 0.001
        if parent_viz_rpy != [0, 0, 0]:
            dp = rpy_to_rotation(parent_viz_rpy) @ dp
        distal_in_parent = np.array(parent_viz_xyz) + dp

        # Transform to child frame
        jnt_xyz = np.array(joint_origins[joint_name])
        jnt_rpy = joint_rpys.get(joint_name, [0, 0, 0])
        gap = distal_in_parent - jnt_xyz
        if jnt_rpy != [0, 0, 0]:
            gap = rpy_to_rotation(jnt_rpy).T @ gap

        # Project gap onto joint axis — shift only along axis
        along = float(np.dot(gap, axis))
        shift = along * axis

        child_viz_xyz, child_viz_rpy = visual_origins[child_name]
        adjusted = [round(child_viz_xyz[i] + shift[i], 6) for i in range(3)]
        messages.append(
            f"  {child_name}: surface gap closed along axis, shift={along * 1000:.1f}mm"
        )
        visual_origins[child_name] = (adjusted, child_viz_rpy)


def rpy_to_rotation(rpy: list[float]) -> np.ndarray:
    """Convert roll-pitch-yaw to 3x3 rotation matrix."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def validate_fk(chain: dict, urdf_path: Path) -> list[str]:
    """Run FK at zero config and report joint positions.

    Also validates inter-joint distances against DH params.
    """
    messages: list[str] = []
    dh = chain.get("dh_params", {})
    if not dh:
        return messages

    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        pos = np.zeros(3)
        rot = np.eye(3)
        messages.append("FK validation (zero config):")

        positions: dict[str, np.ndarray] = {}
        for joint_el in root.findall("joint"):
            name = joint_el.get("name")
            origin = joint_el.find("origin")
            xyz_str = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy_str = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            xyz = np.array([float(v) for v in xyz_str.split()])
            rpy = [float(v) for v in rpy_str.split()]

            pos = pos + rot @ xyz

            if any(abs(v) > 1e-6 for v in rpy):
                rot = rot @ rpy_to_rotation(rpy)

            pos_mm = pos * 1000
            assert name is not None
            positions[name] = pos_mm.copy()
            messages.append(
                f"  {name}: ({pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}) mm"
            )

        # Report consecutive joint distances
        joint_names = list(positions.keys())
        for i in range(len(joint_names) - 1):
            j_from = joint_names[i]
            j_to = joint_names[i + 1]
            dist = float(np.linalg.norm(positions[j_to] - positions[j_from]))
            messages.append(f"  {j_from}->{j_to} distance={dist:.1f}mm")

    except Exception as e:
        messages.append(f"FK validation failed: {e}")

    return messages
