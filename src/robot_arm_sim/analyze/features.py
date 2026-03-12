"""Geometric feature detection for mesh analysis."""

from __future__ import annotations

import numpy as np
import trimesh

from robot_arm_sim.models import GeometricFeature


def detect_features(mesh: trimesh.Trimesh) -> list[GeometricFeature]:
    """Detect geometric features (flat faces, cylinders, holes) on a mesh."""
    features: list[GeometricFeature] = []
    features.extend(_detect_flat_faces(mesh))
    features.extend(_detect_cylindrical_surfaces(mesh))
    return features


def _detect_flat_faces(mesh: trimesh.Trimesh) -> list[GeometricFeature]:
    """Find large coplanar face groups (candidate mating surfaces)."""
    features = []
    if not hasattr(mesh, "facets") or mesh.facets is None:
        return features

    total_area = mesh.area
    for i, facet in enumerate(mesh.facets):
        facet_area = float(np.sum(mesh.area_faces[facet]))
        # Only report facets that are at least 2% of total area
        if facet_area < total_area * 0.02:
            continue

        normal = mesh.facets_normal[i].tolist()
        centroid = np.mean(mesh.vertices[mesh.faces[facet].flatten()], axis=0).tolist()

        # Describe direction
        direction = _describe_direction(normal)
        description = (
            f"Large flat face on {direction}, "
            f"area {facet_area:.1f} mm², likely mating surface"
        )

        features.append(
            GeometricFeature(
                kind="flat_face",
                description=description,
                normal=_round_list(normal),
                area_mm2=round(facet_area, 2),
                centroid=_round_list(centroid),
            )
        )

    # Sort by area descending
    features.sort(key=lambda f: f.area_mm2 or 0, reverse=True)
    return features


def _detect_cylindrical_surfaces(mesh: trimesh.Trimesh) -> list[GeometricFeature]:
    """Detect cylindrical surfaces by clustering face normals in ring patterns."""
    features = []
    if len(mesh.faces) < 20:
        return features

    normals = mesh.face_normals
    centroids = mesh.triangles_center

    # Find faces whose normals form a ring (perpendicular to a common axis)
    for candidate_axis in [
        np.array([1, 0, 0]),
        np.array([0, 1, 0]),
        np.array([0, 0, 1]),
    ]:
        # Find faces whose normals are roughly perpendicular to this axis
        dots = np.abs(normals @ candidate_axis)
        perp_mask = dots < 0.3  # within ~73 degrees of perpendicular

        if np.sum(perp_mask) < 12:
            continue

        perp_centroids = centroids[perp_mask]
        perp_normals = normals[perp_mask]

        # Project centroids onto plane perpendicular to axis
        proj = perp_centroids - np.outer(
            perp_centroids @ candidate_axis, candidate_axis
        )

        # Check if they form a circle by looking at distance from centroid
        center_2d = np.mean(proj, axis=0)
        radii = np.linalg.norm(proj - center_2d, axis=1)
        mean_radius = float(np.mean(radii))
        std_radius = float(np.std(radii))

        if mean_radius < 1.0:
            continue

        # A cylinder has consistent radius (low std relative to mean)
        if std_radius / mean_radius < 0.3:
            # Estimate length along axis
            axis_proj = perp_centroids @ candidate_axis
            length = float(np.max(axis_proj) - np.min(axis_proj))

            center = np.mean(perp_centroids, axis=0)

            # Check concavity: if normals point inward, it's a hole
            to_center = center_2d - proj
            to_center_norm = to_center / (
                np.linalg.norm(to_center, axis=1, keepdims=True) + 1e-8
            )
            dot_with_normal = np.sum(perp_normals * to_center_norm, axis=1)
            concave = bool(np.mean(dot_with_normal) > 0)

            if concave and mean_radius < 5.0:
                desc = (
                    f"Hole along {_describe_axis(candidate_axis)}, "
                    f"radius {mean_radius:.1f} mm"
                )
                features.append(
                    GeometricFeature(
                        kind="hole",
                        description=desc,
                        axis=candidate_axis.tolist(),
                        radius_mm=round(mean_radius, 2),
                        center=_round_list(center.tolist()),
                    )
                )
            else:
                desc = (
                    f"{'Concave c' if concave else 'C'}ylindrical surface along "
                    f"{_describe_axis(candidate_axis)}, "
                    f"radius {mean_radius:.1f} mm, length {length:.1f} mm"
                )
                features.append(
                    GeometricFeature(
                        kind="cylindrical_surface",
                        description=desc,
                        axis=candidate_axis.tolist(),
                        radius_mm=round(mean_radius, 2),
                        length_mm=round(length, 2),
                        center=_round_list(center.tolist()),
                        concave=concave,
                    )
                )

    return features


def _describe_direction(normal: list[float]) -> str:
    """Describe a normal direction in human-readable terms."""
    abs_n = [abs(x) for x in normal]
    idx = abs_n.index(max(abs_n))
    axis_names = ["X", "Y", "Z"]
    sign = "+" if normal[idx] > 0 else "-"
    directions = {
        ("Z", "+"): "top",
        ("Z", "-"): "bottom",
        ("Y", "+"): "front",
        ("Y", "-"): "back",
        ("X", "+"): "right",
        ("X", "-"): "left",
    }
    return directions.get((axis_names[idx], sign), f"{sign}{axis_names[idx]} side")


def _describe_axis(axis: np.ndarray) -> str:
    """Describe an axis direction."""
    abs_a = np.abs(axis)
    idx = int(np.argmax(abs_a))
    return ["X", "Y", "Z"][idx] + " axis"


def _round_list(values: list[float], decimals: int = 4) -> list[float]:
    return [round(v, decimals) for v in values]
