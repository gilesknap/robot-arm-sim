"""STL file parser using trimesh."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh

from robot_arm_sim.models.part import PartAnalysis

from .base import AbstractMeshParser


class STLParser(AbstractMeshParser):
    """Parse STL files using trimesh."""

    def supported_extensions(self) -> list[str]:
        return [".stl"]

    def parse(self, file_path: Path) -> PartAnalysis:
        mesh = trimesh.load(file_path, force="mesh")
        assert isinstance(mesh, trimesh.Trimesh)

        # Determine format
        with open(file_path, "rb") as f:
            header = f.read(5)
        fmt = "ascii_stl" if header == b"solid" else "binary_stl"

        bb_min = mesh.bounds[0].tolist()
        bb_max = mesh.bounds[1].tolist()
        extents = mesh.extents.tolist()

        # Inertia
        try:
            moments = mesh.principal_inertia_components.tolist()
            axes = mesh.principal_inertia_vectors.tolist()
        except Exception:
            moments = [0.0, 0.0, 0.0]
            axes = []

        # Center of mass
        try:
            com = mesh.center_mass.tolist()
        except Exception:
            com = np.mean(mesh.bounds, axis=0).tolist()

        return PartAnalysis(
            part_name=file_path.stem,
            source_file=str(file_path),
            format=fmt,
            vertex_count=len(mesh.vertices),
            face_count=len(mesh.faces),
            bounding_box_min=bb_min,
            bounding_box_max=bb_max,
            bounding_box_extents=extents,
            volume_mm3=float(mesh.volume) if mesh.is_watertight else 0.0,
            surface_area_mm2=float(mesh.area),
            center_of_mass=com,
            is_watertight=bool(mesh.is_watertight),
            principal_moments=moments,
            principal_axes=axes,
        )
