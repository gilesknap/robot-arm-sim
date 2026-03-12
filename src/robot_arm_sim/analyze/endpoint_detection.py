"""Endpoint detection along axes for bore/shaft connections.

Finds proximal/distal connection points by analyzing mesh endpoints
along cylindrical axes, using cross-section and flat-face evidence.
"""

from __future__ import annotations

import logging

import numpy as np
import trimesh

from robot_arm_sim.models import ConnectionPoint, GeometricFeature

from .circle_fitting import (
    circle_fit_from_boundary,
    estimate_radius,
    estimate_radius_at_slice,
    find_circle_center_at_slice,
)

logger = logging.getLogger(__name__)


def detect_base_connection(
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
    center = find_circle_center_at_slice(mesh, np.array([0, 0, 1]), z_top, "below")

    if center is not None:
        r = top_face.radius_mm or estimate_radius(top_face)
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


def detect_multi_axis_connections(
    mesh: trimesh.Trimesh,
    features: list[GeometricFeature],
    cylinders: list[GeometricFeature],
    axis_groups: list[list[GeometricFeature]],
) -> list[ConnectionPoint]:
    """Multi-axis part (e.g., L-shaped): proximal along first dominant axis,
    distal along second dominant axis.

    Uses flat face evidence to determine which end of each axis has the bore
    opening, rather than assuming min/max — critical for L-shaped parts where
    bore openings face outward from the junction.
    """
    points = []
    bounds = mesh.bounds
    flat_faces = [f for f in features if f.kind == "flat_face" and f.normal]

    for end, group in [("proximal", axis_groups[0]), ("distal", axis_groups[1])]:
        axis = np.array(group[0].axis)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-6:
            continue
        axis = axis / axis_norm
        axis_idx = int(np.argmax(np.abs(axis)))

        # Use flat face evidence to find which end has the bore opening
        bore_val, direction = find_bore_end_for_axis(axis, axis_idx, bounds, flat_faces)

        center = find_circle_center_at_slice(
            mesh, np.abs(axis) * np.sign(axis), bore_val, direction
        )
        if center is not None:
            cyl = next((c for c in group if c.radius_mm is not None), None)
            radius = cyl.radius_mm if cyl and cyl.radius_mm is not None else 20.0
            pos = center.copy()
            pos[axis_idx] = bore_val
            points.append(
                ConnectionPoint(
                    end=end,  # type: ignore[arg-type]
                    position=[round(v, 3) for v in pos],
                    axis=[round(a, 4) for a in axis.tolist()],
                    radius_mm=round(radius, 1),
                    method="cross_section",
                )
            )
        else:
            # Fallback: use the largest aligned flat face centroid
            matching_faces = [
                f
                for f in flat_faces
                if f.normal and abs(float(np.dot(f.normal, axis))) > 0.9 and f.centroid
            ]
            # Filter to faces near the bore end
            mid = (bounds[0][axis_idx] + bounds[1][axis_idx]) / 2.0
            if direction == "above":
                matching_faces = [
                    f
                    for f in matching_faces
                    if f.centroid[axis_idx] < mid  # type: ignore[index]
                ]
            else:
                matching_faces = [
                    f
                    for f in matching_faces
                    if f.centroid[axis_idx] >= mid  # type: ignore[index]
                ]
            if matching_faces:
                bf = max(matching_faces, key=lambda f: f.area_mm2 or 0)
                if bf.centroid:
                    pos = list(bf.centroid)
                    pos[axis_idx] = bore_val
                    points.append(
                        ConnectionPoint(
                            end=end,  # type: ignore[arg-type]
                            position=[round(v, 3) for v in pos],
                            axis=[round(a, 4) for a in axis.tolist()],
                            radius_mm=20.0,
                            method="centroid_fallback",
                        )
                    )

    return points


def find_bore_end_for_axis(
    axis: np.ndarray,
    axis_idx: int,
    bounds: np.ndarray,
    flat_faces: list[GeometricFeature],
) -> tuple[float, str]:
    """Find which end of an axis has the bore opening using flat face evidence.

    For L-shaped parts, bore openings face outward from the junction.  The
    largest flat face aligned with the axis indicates the bore opening end.

    Returns ``(bound_val, slice_direction)`` where *bound_val* is the bounding
    box value at the bore opening end and *slice_direction* is ``"above"`` when
    the bore is at the min end or ``"below"`` when at the max end.
    """
    aligned: list[GeometricFeature] = []
    for f in flat_faces:
        if f.normal and f.centroid:
            dot = abs(float(np.dot(f.normal, axis)))
            if dot > 0.95:
                aligned.append(f)

    if not aligned:
        # No flat-face evidence; default to min end
        return float(bounds[0][axis_idx]), "above"

    # Split into faces at min end vs max end
    mid = (bounds[0][axis_idx] + bounds[1][axis_idx]) / 2.0
    min_faces = [f for f in aligned if f.centroid[axis_idx] < mid]  # type: ignore[index]
    max_faces = [f for f in aligned if f.centroid[axis_idx] >= mid]  # type: ignore[index]

    min_area = max((f.area_mm2 or 0 for f in min_faces), default=0)
    max_area = max((f.area_mm2 or 0 for f in max_faces), default=0)

    if max_area >= min_area:
        return float(bounds[1][axis_idx]), "below"
    else:
        return float(bounds[0][axis_idx]), "above"


def detect_endpoints_along_axis(
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
        center = find_circle_center_at_slice(mesh, axis, proj_val, direction)
        if center is not None:
            pos = center.copy()
            pos[axis_idx] = proj_val
            radius = estimate_radius_at_slice(mesh, axis, proj_val, direction)
            points.append(
                ConnectionPoint(
                    end=end,  # type: ignore[arg-type]
                    position=[round(p, 3) for p in pos],
                    axis=[round(a, 4) for a in axis.tolist()],
                    radius_mm=round(radius, 1),
                    method="cross_section",
                )
            )
        else:
            center_fb = circle_fit_from_boundary(mesh, axis, proj_val, direction)
            if center_fb is not None:
                pos = center_fb["center"].copy()
                pos[axis_idx] = proj_val
                points.append(
                    ConnectionPoint(
                        end=end,  # type: ignore[arg-type]
                        position=[round(p, 3) for p in pos],
                        axis=[round(a, 4) for a in axis.tolist()],
                        radius_mm=round(center_fb["radius"], 1),
                        method="cylinder_fit",
                    )
                )

    return points
