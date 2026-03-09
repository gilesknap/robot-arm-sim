"""Dataclasses for mesh part analysis results."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class GeometricFeature:
    """A detected geometric feature on a mesh part."""

    kind: str  # "flat_face", "cylindrical_surface", "hole"
    description: str
    normal: list[float] | None = None
    axis: list[float] | None = None
    area_mm2: float | None = None
    radius_mm: float | None = None
    length_mm: float | None = None
    center: list[float] | None = None
    centroid: list[float] | None = None
    concave: bool | None = None


@dataclass
class ConnectionPoint:
    """A detected connection point (bore/shaft center) at a mesh end."""

    end: str  # "proximal" or "distal"
    position: list[float]  # [x,y,z] in STL coords (mm)
    axis: list[float]  # joint axis unit vector
    radius_mm: float
    method: str  # "cross_section" or "cylinder_fit"


@dataclass
class PartAnalysis:
    """Complete analysis result for a single mesh part."""

    part_name: str
    source_file: str
    format: str
    vertex_count: int = 0
    face_count: int = 0
    bounding_box_min: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    bounding_box_max: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    bounding_box_extents: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    volume_mm3: float = 0.0
    surface_area_mm2: float = 0.0
    center_of_mass: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    is_watertight: bool = False
    principal_moments: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    principal_axes: list[list[float]] = field(default_factory=list)
    features: list[GeometricFeature] = field(default_factory=list)
    connection_points: list[ConnectionPoint] = field(default_factory=list)
    text_description: str = ""
    render_paths: list[str] = field(default_factory=list)
