"""Circle fitting and cross-section analysis for bore detection.

Provides functions that slice meshes at cross-sections and fit circles
to detect bore/shaft centers and radii.
"""

from __future__ import annotations

import numpy as np
import trimesh

from robot_arm_sim.models import GeometricFeature


def find_circle_center_at_slice(
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


def estimate_radius_at_slice(
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


def circle_fit_from_boundary(
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


def estimate_radius(feature: GeometricFeature) -> float:
    """Estimate radius from a flat face's area (assuming circular)."""
    area = feature.area_mm2 or 0
    if area > 0:
        return float(np.sqrt(area / np.pi))
    return 20.0
