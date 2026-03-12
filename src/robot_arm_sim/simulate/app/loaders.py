"""Data loaders for analysis YAML files."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from robot_arm_sim.models import load_part_yaml
from robot_arm_sim.models.robot import URDFRobot


def load_mesh_centers(robot: URDFRobot, robot_dir: Path) -> dict[str, np.ndarray]:
    """Load bounding box centers from analysis YAML files.

    Returns a dict mapping link_name -> bbox center in STL coordinates (mm).
    """
    centers: dict[str, np.ndarray] = {}
    analysis_dir = robot_dir / "analysis"
    if not analysis_dir.exists():
        return centers

    for link in robot.links:
        if not link.mesh_path:
            continue
        part_name = Path(link.mesh_path).stem
        yaml_path = analysis_dir / f"{part_name}.yaml"
        if not yaml_path.exists():
            continue
        model = load_part_yaml(yaml_path)
        bbox = model.geometry.bounding_box
        center = np.array(
            [
                (bbox.min[0] + bbox.max[0]) / 2.0,
                (bbox.min[1] + bbox.max[1]) / 2.0,
                (bbox.min[2] + bbox.max[2]) / 2.0,
            ]
        )
        centers[link.name] = center
    return centers


def load_flat_faces(robot: URDFRobot, robot_dir: Path) -> dict[str, list[dict]]:
    """Load flat face features from analysis YAML files.

    Returns dict mapping link_name -> list of {normal, area_mm2, centroid}.
    """
    result: dict[str, list[dict]] = {}
    analysis_dir = robot_dir / "analysis"
    if not analysis_dir.exists():
        return result

    for link in robot.links:
        if not link.mesh_path:
            continue
        part_name = Path(link.mesh_path).stem
        yaml_path = analysis_dir / f"{part_name}.yaml"
        if not yaml_path.exists():
            continue
        model = load_part_yaml(yaml_path)
        faces = model.features.flat_faces
        if faces:
            result[link.name] = [
                {
                    "normal": ff.normal,
                    "area_mm2": ff.area_mm2,
                    "centroid": ff.centroid,
                }
                for ff in faces
            ]
    return result


def quantize_axis(normal: list[float]) -> list[float]:
    """Snap a normal vector to the nearest cardinal axis."""
    idx = max(range(3), key=lambda i: abs(normal[i]))
    result = [0.0, 0.0, 0.0]
    result[idx] = 1.0 if normal[idx] > 0 else -1.0
    return result


def load_connection_points(robot: URDFRobot, robot_dir: Path) -> dict[str, list[dict]]:
    """Load bore connection points from analysis YAML files.

    Returns dict mapping link_name -> list of connection point dicts,
    each with keys: end, position (mm ndarray), axis, radius_mm.
    """
    result: dict[str, list[dict]] = {}
    analysis_dir = robot_dir / "analysis"
    if not analysis_dir.exists():
        return result

    for link in robot.links:
        if not link.mesh_path:
            continue
        part_name = Path(link.mesh_path).stem
        yaml_path = analysis_dir / f"{part_name}.yaml"
        if not yaml_path.exists():
            continue
        model = load_part_yaml(yaml_path)
        cps = model.connection_points
        if cps:
            points = []
            for cp in cps:
                points.append(
                    {
                        "end": cp.end,
                        "position": np.array(cp.position),
                        "axis": cp.axis,
                        "radius_mm": cp.radius_mm,
                        "center": cp.center if cp.center is not None else False,
                    }
                )
            result[link.name] = points
    return result
