"""Connection point detection for mesh parts.

Finds bore/shaft centers at each end of a mesh by slicing cross-sections
along detected cylindrical axes and fitting circles to the resulting
profiles.
"""

from __future__ import annotations

import logging

import numpy as np
import trimesh

from robot_arm_sim.models import ConnectionPoint, GeometricFeature

from .endpoint_detection import (
    detect_base_connection,
    detect_endpoints_along_axis,
    detect_multi_axis_connections,
)

logger = logging.getLogger(__name__)


def detect_connection_points(
    mesh: trimesh.Trimesh,
    features: list[GeometricFeature],
    part_name: str,
) -> list[ConnectionPoint]:
    """Detect connection points (bore/shaft centers) at mesh ends.

    Uses cylindrical surface axes from feature detection, then slices
    cross-sections near both ends to find circular features and their
    centers.

    Classification is geometry-based:
    - No significant cylindrical surfaces -> base part (distal only)
    - Cylinders along 2+ distinct axes (>30 deg apart) -> multi-axis part
    - Single dominant cylinder axis -> standard part with proximal/distal
    """
    cylinders = [f for f in features if f.kind == "cylindrical_surface"]

    if not cylinders:
        return detect_base_connection(mesh, features)

    axis_groups = _group_cylinder_axes(cylinders, angle_threshold_deg=30.0)

    if len(axis_groups) >= 2:
        kept, removed = _filter_barrel_groups(axis_groups)
        if removed and len(kept) == 1:
            barrel_points = _detect_barrel_part_bores(
                mesh, features, kept[0], removed, part_name
            )
            if barrel_points:
                return barrel_points
            axis_groups = kept
        else:
            axis_groups = kept

    if len(axis_groups) >= 2:
        return detect_multi_axis_connections(mesh, features, cylinders, axis_groups)

    group_cyls = axis_groups[0] if axis_groups else cylinders
    primary_cyl = max(group_cyls, key=lambda c: c.length_mm or 0)
    axis = np.array(primary_cyl.axis)

    return detect_endpoints_along_axis(mesh, axis, part_name)


def _group_cylinder_axes(
    cylinders: list[GeometricFeature],
    angle_threshold_deg: float = 30.0,
) -> list[list[GeometricFeature]]:
    """Group cylinders by axis direction.

    Cylinders whose axes are within angle_threshold_deg are in the same group.
    Returns groups sorted by total cylinder length (largest first).
    """
    threshold_rad = np.radians(angle_threshold_deg)
    groups: list[list[GeometricFeature]] = []

    for cyl in cylinders:
        axis = np.array(cyl.axis)
        norm = np.linalg.norm(axis)
        if norm < 1e-6:
            continue
        axis = axis / norm

        placed = False
        for group in groups:
            ref_axis = np.array(group[0].axis)
            ref_norm = np.linalg.norm(ref_axis)
            if ref_norm < 1e-6:
                continue
            ref_axis = ref_axis / ref_norm
            angle = np.arccos(np.clip(abs(np.dot(axis, ref_axis)), -1.0, 1.0))
            if angle < threshold_rad:
                group.append(cyl)
                placed = True
                break

        if not placed:
            groups.append([cyl])

    groups.sort(key=lambda g: sum(c.length_mm or 0 for c in g), reverse=True)
    return groups


_BARREL_RADIUS_RATIO = 2.0


def _filter_barrel_groups(
    axis_groups: list[list[GeometricFeature]],
) -> tuple[list[list[GeometricFeature]], list[list[GeometricFeature]]]:
    """Filter out barrel/bulge cylinder groups with outsized radii.

    Returns ``(kept, removed)`` so the caller can use removed-group axes
    as evidence for bore direction on barrel-shaped parts.
    """

    def _rep_radius(group: list[GeometricFeature]) -> float:
        longest = max(group, key=lambda c: c.length_mm or 0)
        return longest.radius_mm if longest.radius_mm is not None else 0.0

    radii = [_rep_radius(g) for g in axis_groups]
    min_radius = min(r for r in radii if r > 0) if any(r > 0 for r in radii) else 0.0

    if min_radius <= 0:
        return axis_groups, []

    kept = []
    removed = []
    for g, r in zip(axis_groups, radii, strict=True):
        if r <= min_radius * _BARREL_RADIUS_RATIO:
            kept.append(g)
        else:
            removed.append(g)

    if not kept:
        return axis_groups, []

    logger.debug(
        "Barrel filter: %d/%d axis groups kept (min_r=%.1f, threshold=%.1f)",
        len(kept),
        len(axis_groups),
        min_radius,
        min_radius * _BARREL_RADIUS_RATIO,
    )
    return kept, removed


def _detect_barrel_part_bores(
    mesh: trimesh.Trimesh,
    features: list[GeometricFeature],
    tube_group: list[GeometricFeature],
    removed_groups: list[list[GeometricFeature]],
    part_name: str,
) -> list[ConnectionPoint] | None:
    """Detect bore connections on barrel-shaped parts using flat face evidence.

    When the barrel filter leaves a single tube axis and removes barrel groups,
    the bore axis matches the removed group direction. Flat faces aligned with
    that bore axis indicate bore openings; their centroids give bore centers.
    """
    flat_faces = [f for f in features if f.kind == "flat_face" and f.normal]

    best_axis = None
    best_score = 0.0
    for group in removed_groups:
        ref = np.array(group[0].axis)
        ref_norm = np.linalg.norm(ref)
        if ref_norm < 1e-6:
            continue
        ref = ref / ref_norm
        score = sum(
            f.area_mm2 or 0
            for f in flat_faces
            if abs(float(np.dot(f.normal, ref))) > 0.95  # type: ignore[arg-type]
        )
        if score > best_score:
            best_score = score
            best_axis = ref

    if best_axis is None or best_score < 1.0:
        return None

    bore_axis = best_axis

    tube_cyl = max(tube_group, key=lambda c: c.length_mm or 0)
    tube_axis = np.array(tube_cyl.axis)
    tube_norm = np.linalg.norm(tube_axis)
    if tube_norm < 1e-6:
        return None
    tube_axis = tube_axis / tube_norm
    tube_idx = int(np.argmax(np.abs(tube_axis)))

    bore_faces = [
        f
        for f in flat_faces
        if abs(float(np.dot(f.normal, bore_axis))) > 0.95  # type: ignore[arg-type]
        and f.centroid
    ]

    if not bore_faces:
        return None

    bounds = mesh.bounds
    tube_mid = (bounds[0][tube_idx] + bounds[1][tube_idx]) / 2.0

    proximal_faces = [
        f
        for f in bore_faces
        if f.centroid[tube_idx] < tube_mid  # type: ignore[index]
    ]
    distal_faces = [
        f
        for f in bore_faces
        if f.centroid[tube_idx] >= tube_mid  # type: ignore[index]
    ]

    points: list[ConnectionPoint] = []
    for end, face_list in [("proximal", proximal_faces), ("distal", distal_faces)]:
        if not face_list:
            continue
        face = max(face_list, key=lambda f: f.area_mm2 or 0)
        centroid = np.array(face.centroid)  # type: ignore[arg-type]
        pos = centroid.copy()

        radius = float(np.sqrt((face.area_mm2 or 0) / np.pi))
        points.append(
            ConnectionPoint(
                end=end,  # type: ignore[arg-type]
                position=[round(float(v), 3) for v in pos],
                axis=[round(float(a), 4) for a in bore_axis.tolist()],
                radius_mm=round(radius, 1),
                method="barrel_bore_face",
            )
        )

    if not points:
        return None

    logger.debug(
        "Barrel bore detection on %s: %d points along axis %s",
        part_name,
        len(points),
        [round(float(a), 2) for a in bore_axis],
    )
    return points
