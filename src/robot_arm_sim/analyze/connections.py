"""Connection point detection for mesh parts.

Finds bore/shaft centers at each end of a mesh by slicing cross-sections
along detected cylindrical axes and fitting circles to the resulting
profiles.
"""

from __future__ import annotations

import logging

import numpy as np
import trimesh

from robot_arm_sim.models.part import ConnectionPoint, GeometricFeature

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
    - No significant cylindrical surfaces → base part (distal only)
    - Cylinders along 2+ distinct axes (>30° apart) → multi-axis part
    - Single dominant cylinder axis → standard part with proximal/distal
    """
    cylinders = [f for f in features if f.kind == "cylindrical_surface"]

    if not cylinders:
        # No cylindrical surfaces → treat as base (distal connection only)
        return _detect_base_connection(mesh, features)

    # Group cylinder axes: two axes are "distinct" if angle between them > 30°
    axis_groups = _group_cylinder_axes(cylinders, angle_threshold_deg=30.0)

    if len(axis_groups) >= 2:
        # Multi-axis part (e.g., L-shaped with bores along different axes)
        return _detect_multi_axis_connections(mesh, features, cylinders, axis_groups)

    # Standard parts: find primary axis from largest cylinder
    primary_cyl = max(cylinders, key=lambda c: c.length_mm or 0)
    axis = np.array(primary_cyl.axis)

    return _detect_endpoints_along_axis(mesh, axis, part_name)


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

    # Sort groups by total cylinder length (dominant axis first)
    groups.sort(key=lambda g: sum(c.length_mm or 0 for c in g), reverse=True)
    return groups


def _detect_base_connection(
    mesh: trimesh.Trimesh,
    features: list[GeometricFeature],
) -> list[ConnectionPoint]:
    """Base part: only has a distal connection at the top."""
    top_faces = [
        f for f in features if f.kind == "flat_face" and f.normal and f.normal[2] > 0.9
    ]
    if not top_faces:
        return []

    top_face = max(top_faces, key=lambda f: f.area_mm2 or 0)

    bounds = mesh.bounds
    z_top = bounds[1][2]
    center = _find_circle_center_at_slice(mesh, np.array([0, 0, 1]), z_top, "below")

    if center is not None:
        r = top_face.radius_mm or _estimate_radius(top_face)
        return [
            ConnectionPoint(
                end="distal",
                position=[
                    round(center[0], 3),
                    round(center[1], 3),
                    round(z_top, 3),
                ],
                axis=[0, 0, 1],
                radius_mm=round(r, 1),
                method="cross_section",
            )
        ]

    # Fallback: use centroid of top face
    if top_face.centroid:
        c = top_face.centroid
        r = np.sqrt((top_face.area_mm2 or 0) / np.pi)
        return [
            ConnectionPoint(
                end="distal",
                position=[round(c[0], 3), round(c[1], 3), round(z_top, 3)],
                axis=[0, 0, 1],
                radius_mm=round(float(r), 1),
                method="centroid_fallback",
            )
        ]

    return []


def _detect_multi_axis_connections(
    mesh: trimesh.Trimesh,
    features: list[GeometricFeature],
    cylinders: list[GeometricFeature],
    axis_groups: list[list[GeometricFeature]],
) -> list[ConnectionPoint]:
    """Multi-axis part (e.g., L-shaped): proximal along first dominant axis,
    distal along second dominant axis."""
    points = []
    bounds = mesh.bounds

    # First dominant axis group → proximal end
    primary_axis = np.array(axis_groups[0][0].axis)
    primary_norm = np.linalg.norm(primary_axis)
    if primary_norm > 1e-6:
        primary_axis = primary_axis / primary_norm
    axis_idx_p = int(np.argmax(np.abs(primary_axis)))
    proj_min_p = bounds[0][axis_idx_p]

    center_p = _find_circle_center_at_slice(
        mesh, np.abs(primary_axis) * np.sign(primary_axis), proj_min_p, "above"
    )
    if center_p is not None:
        p_cyl = next((c for c in axis_groups[0] if c.radius_mm is not None), None)
        radius = p_cyl.radius_mm if p_cyl and p_cyl.radius_mm is not None else 20.0
        pos = center_p.copy()
        pos[axis_idx_p] = proj_min_p
        points.append(
            ConnectionPoint(
                end="proximal",
                position=[round(v, 3) for v in pos],
                axis=[round(a, 4) for a in primary_axis.tolist()],
                radius_mm=round(radius, 1),
                method="cross_section",
            )
        )
    else:
        # Fallback: use flat face normals opposite to primary axis
        neg_dir = -primary_axis
        matching_faces = [
            f
            for f in features
            if f.kind == "flat_face" and f.normal and np.dot(f.normal, neg_dir) > 0.9
        ]
        if matching_faces:
            bf = max(matching_faces, key=lambda f: f.area_mm2 or 0)
            if bf.centroid:
                c = bf.centroid
                pos = list(c)
                pos[axis_idx_p] = proj_min_p
                points.append(
                    ConnectionPoint(
                        end="proximal",
                        position=[round(v, 3) for v in pos],
                        axis=[round(a, 4) for a in primary_axis.tolist()],
                        radius_mm=20.0,
                        method="centroid_fallback",
                    )
                )

    # Second dominant axis group → distal end
    secondary_axis = np.array(axis_groups[1][0].axis)
    secondary_norm = np.linalg.norm(secondary_axis)
    if secondary_norm > 1e-6:
        secondary_axis = secondary_axis / secondary_norm
    axis_idx_s = int(np.argmax(np.abs(secondary_axis)))
    proj_max_s = bounds[1][axis_idx_s]

    center_s = _find_circle_center_at_slice(
        mesh, np.abs(secondary_axis) * np.sign(secondary_axis), proj_max_s, "below"
    )
    if center_s is not None:
        s_cyl = next((c for c in axis_groups[1] if c.radius_mm is not None), None)
        radius = s_cyl.radius_mm if s_cyl and s_cyl.radius_mm is not None else 20.0
        pos = center_s.copy()
        pos[axis_idx_s] = proj_max_s
        points.append(
            ConnectionPoint(
                end="distal",
                position=[round(v, 3) for v in pos],
                axis=[round(a, 4) for a in secondary_axis.tolist()],
                radius_mm=round(radius, 1),
                method="cross_section",
            )
        )
    else:
        # Fallback: use flat faces along secondary axis direction
        matching_faces = [
            f
            for f in features
            if f.kind == "flat_face"
            and f.normal
            and np.dot(f.normal, secondary_axis) > 0.9
        ]
        if matching_faces:
            rf = max(matching_faces, key=lambda f: f.area_mm2 or 0)
            if rf.centroid:
                c = rf.centroid
                pos = list(c)
                pos[axis_idx_s] = proj_max_s
                points.append(
                    ConnectionPoint(
                        end="distal",
                        position=[round(v, 3) for v in pos],
                        axis=[round(a, 4) for a in secondary_axis.tolist()],
                        radius_mm=20.0,
                        method="centroid_fallback",
                    )
                )

    return points


def _detect_endpoints_along_axis(
    mesh: trimesh.Trimesh,
    axis: np.ndarray,
    part_name: str,
) -> list[ConnectionPoint]:
    """Detect proximal/distal connection points along an axis."""
    points = []

    # Project vertices onto axis to find extent
    projections = mesh.vertices @ axis
    proj_min = float(np.min(projections))
    proj_max = float(np.max(projections))

    axis_idx = int(np.argmax(np.abs(axis)))

    for end, proj_val, direction in [
        ("proximal", proj_min, "above"),
        ("distal", proj_max, "below"),
    ]:
        center = _find_circle_center_at_slice(mesh, axis, proj_val, direction)
        if center is not None:
            pos = center.copy()
            pos[axis_idx] = proj_val
            radius = _estimate_radius_at_slice(mesh, axis, proj_val, direction)
            points.append(
                ConnectionPoint(
                    end=end,
                    position=[round(p, 3) for p in pos],
                    axis=[round(a, 4) for a in axis.tolist()],
                    radius_mm=round(radius, 1),
                    method="cross_section",
                )
            )
        else:
            center_fb = _circle_fit_from_boundary(mesh, axis, proj_val, direction)
            if center_fb is not None:
                pos = center_fb["center"].copy()
                pos[axis_idx] = proj_val
                points.append(
                    ConnectionPoint(
                        end=end,
                        position=[round(p, 3) for p in pos],
                        axis=[round(a, 4) for a in axis.tolist()],
                        radius_mm=round(center_fb["radius"], 1),
                        method="cylinder_fit",
                    )
                )

    return points


def _find_circle_center_at_slice(
    mesh: trimesh.Trimesh,
    axis: np.ndarray,
    bound_val: float,
    direction: str,
) -> np.ndarray | None:
    """Slice mesh near a bound, find center of circular cross-section.

    direction: "above" = slightly above bound_val,
               "below" = slightly below.
    """
    projections = mesh.vertices @ axis
    proj_min = float(np.min(projections))
    proj_max = float(np.max(projections))
    extent = proj_max - proj_min

    if extent < 1.0:
        return None

    # Try slicing at 5%, 10%, 15% from the end
    for frac in [0.05, 0.10, 0.15]:
        if direction == "above":
            offset = proj_min + frac * extent
        else:
            offset = proj_max - frac * extent

        plane_origin = axis * offset
        try:
            section = mesh.section(plane_origin=plane_origin, plane_normal=axis)
        except Exception:
            continue

        if section is None:
            continue

        center = _extract_circle_center_from_section(section, axis)
        if center is not None:
            return center

    # Fallback: centroid of vertices near the end
    threshold = 0.08 * extent
    if direction == "above":
        mask = projections < (proj_min + threshold)
    else:
        mask = projections > (proj_max - threshold)

    end_verts = mesh.vertices[mask]
    if len(end_verts) >= 4:
        return np.mean(end_verts, axis=0)

    return None


def _extract_circle_center_from_section(
    section: trimesh.path.Path3D,  # type: ignore[name-defined]
    axis: np.ndarray,
) -> np.ndarray | None:
    """Extract center of the most circular polygon in a section."""
    try:
        path_2d, transform = section.to_planar()
    except Exception:
        return None

    if not hasattr(path_2d, "polygons_full"):
        return None
    if not path_2d.polygons_full:
        return None

    candidates = []

    for poly in path_2d.polygons_full:
        if poly.area < 10:
            continue

        coords = np.array(poly.exterior.coords[:-1])
        centroid_2d = np.mean(coords, axis=0)
        radii = np.linalg.norm(coords - centroid_2d, axis=1)
        mean_r = float(np.mean(radii))
        if mean_r < 1.0:
            continue

        circularity = float(np.std(radii)) / mean_r

        pts_2d_h = np.array([[centroid_2d[0], centroid_2d[1], 0, 1]])
        pts_3d = (transform @ pts_2d_h.T).T
        center_3d = pts_3d[0, :3]

        candidates.append(
            {
                "center": center_3d,
                "circularity": circularity,
                "radius": mean_r,
            }
        )

    if not candidates:
        return None

    # Prefer highly circular (< 0.15), then relaxed (< 0.35)
    for threshold in [0.15, 0.35]:
        good = [c for c in candidates if c["circularity"] < threshold]
        if good:
            best = min(good, key=lambda c: c["circularity"])
            return best["center"]

    return None


def _estimate_radius_at_slice(
    mesh: trimesh.Trimesh,
    axis: np.ndarray,
    bound_val: float,
    direction: str,
) -> float:
    """Estimate radius of the circular feature at a slice."""
    projections = mesh.vertices @ axis
    proj_min = float(np.min(projections))
    proj_max = float(np.max(projections))
    extent = proj_max - proj_min

    frac = 0.10
    if direction == "above":
        offset = proj_min + frac * extent
    else:
        offset = proj_max - frac * extent

    plane_origin = axis * offset
    try:
        section = mesh.section(plane_origin=plane_origin, plane_normal=axis)
        if section is not None:
            path_2d, _ = section.to_planar()
            if path_2d.polygons_full:
                for poly in path_2d.polygons_full:
                    coords = np.array(poly.exterior.coords[:-1])
                    centroid_2d = np.mean(coords, axis=0)
                    radii = np.linalg.norm(coords - centroid_2d, axis=1)
                    mean_r = float(np.mean(radii))
                    if mean_r <= 0:
                        continue
                    circ = float(np.std(radii)) / mean_r
                    if circ < 0.15:
                        return mean_r
    except Exception:
        pass

    return 20.0  # default fallback


def _circle_fit_from_boundary(
    mesh: trimesh.Trimesh,
    axis: np.ndarray,
    bound_val: float,
    direction: str,
) -> dict | None:
    """Fallback: fit a circle to boundary vertices near a mesh end.

    Used for non-watertight meshes where cross-section fails.
    """
    projections = mesh.vertices @ axis
    proj_min = float(np.min(projections))
    proj_max = float(np.max(projections))
    extent = proj_max - proj_min

    threshold = 0.10 * extent
    if direction == "above":
        mask = projections < (proj_min + threshold)
    else:
        mask = projections > (proj_max - threshold)

    end_vertices = mesh.vertices[mask]
    if len(end_vertices) < 8:
        return None

    axis_idx = int(np.argmax(np.abs(axis)))
    other_axes = [i for i in range(3) if i != axis_idx]

    pts_2d = end_vertices[:, other_axes]
    center_2d = np.mean(pts_2d, axis=0)
    radii = np.linalg.norm(pts_2d - center_2d, axis=1)
    mean_r = float(np.mean(radii))
    std_r = float(np.std(radii))

    if mean_r < 1.0 or (std_r / mean_r) > 0.5:
        return None

    center_3d = np.zeros(3)
    center_3d[other_axes[0]] = center_2d[0]
    center_3d[other_axes[1]] = center_2d[1]
    center_3d[axis_idx] = bound_val

    return {"center": center_3d, "radius": mean_r}


def _estimate_radius(feature: GeometricFeature) -> float:
    """Estimate radius from a flat face's area (assuming circular)."""
    area = feature.area_mm2 or 0
    if area > 0:
        return float(np.sqrt(area / np.pi))
    return 20.0
