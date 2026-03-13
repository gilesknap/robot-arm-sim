"""Pydantic models for robot arm analysis and YAML I/O."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict

# ---------------------------------------------------------------------------
# Part Analysis models
# ---------------------------------------------------------------------------


class BoundingBox(BaseModel):
    min: list[float]
    max: list[float]
    extents: list[float]


class Geometry(BaseModel):
    vertex_count: int = 0
    face_count: int = 0
    bounding_box: BoundingBox = BoundingBox(
        min=[0, 0, 0], max=[0, 0, 0], extents=[0, 0, 0]
    )
    volume_mm3: float = 0.0
    surface_area_mm2: float = 0.0
    center_of_mass: list[float] = [0.0, 0.0, 0.0]
    is_watertight: bool = False


class Inertia(BaseModel):
    principal_moments: list[float] = [0.0, 0.0, 0.0]
    principal_axes: list[list[float]] = []


class ConnectionPoint(BaseModel):
    end: Literal["proximal", "distal"]
    position: list[float]
    axis: list[float]
    radius_mm: float
    method: str  # "cross_section", "cylinder_fit", "manual", etc.
    center: bool | None = None  # deprecated — use centering instead
    centering: Literal["center", "surface"] | None = None


class GeometricFeature(BaseModel):
    """A detected geometric feature on a mesh part.

    Used throughout the detection pipeline. The ``kind`` field determines
    which bucket (flat_faces, cylindrical_surfaces, holes) the feature
    lands in when serialized to YAML via ``Features``.
    """

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


class FlatFace(BaseModel):
    description: str
    normal: list[float] | None = None
    area_mm2: float | None = None
    centroid: list[float] | None = None


class CylindricalSurface(BaseModel):
    description: str
    axis: list[float] | None = None
    radius_mm: float | None = None
    length_mm: float | None = None
    center: list[float] | None = None
    concave: bool | None = None


class Hole(BaseModel):
    description: str
    axis: list[float] | None = None
    radius_mm: float | None = None
    length_mm: float | None = None
    center: list[float] | None = None


class Features(BaseModel):
    flat_faces: list[FlatFace] = []
    cylindrical_surfaces: list[CylindricalSurface] = []
    holes: list[Hole] = []

    @classmethod
    def from_feature_list(cls, features: list[GeometricFeature]) -> Features:
        """Split a flat list of GeometricFeature by kind into typed buckets."""
        flat_faces: list[FlatFace] = []
        cylindrical_surfaces: list[CylindricalSurface] = []
        holes: list[Hole] = []

        for f in features:
            d: dict[str, Any] = {"description": f.description}
            if f.kind == "flat_face":
                if f.normal is not None:
                    d["normal"] = f.normal
                if f.area_mm2 is not None:
                    d["area_mm2"] = f.area_mm2
                if f.centroid is not None:
                    d["centroid"] = f.centroid
                flat_faces.append(FlatFace(**d))
            elif f.kind == "cylindrical_surface":
                if f.axis is not None:
                    d["axis"] = f.axis
                if f.radius_mm is not None:
                    d["radius_mm"] = f.radius_mm
                if f.length_mm is not None:
                    d["length_mm"] = f.length_mm
                if f.center is not None:
                    d["center"] = f.center
                if f.concave is not None:
                    d["concave"] = f.concave
                cylindrical_surfaces.append(CylindricalSurface(**d))
            elif f.kind == "hole":
                if f.axis is not None:
                    d["axis"] = f.axis
                if f.radius_mm is not None:
                    d["radius_mm"] = f.radius_mm
                if f.length_mm is not None:
                    d["length_mm"] = f.length_mm
                if f.center is not None:
                    d["center"] = f.center
                holes.append(Hole(**d))

        return cls(
            flat_faces=flat_faces,
            cylindrical_surfaces=cylindrical_surfaces,
            holes=holes,
        )


class PartAnalysis(BaseModel):
    """Complete analysis result for a single mesh part."""

    part_name: str
    source_file: str
    format: str
    connection_points: list[ConnectionPoint] = []
    geometry: Geometry = Geometry()
    inertia: Inertia = Inertia()
    features: Features = Features()
    text_description: str = ""


# ---------------------------------------------------------------------------
# Chain models
# ---------------------------------------------------------------------------


class ChainLink(BaseModel):
    name: str
    mesh: str | None = None
    visual_rpy: list[float] | None = None
    visual_xyz: list[float] | None = None


class ChainJoint(BaseModel):
    name: str
    type: str  # "revolute", "prismatic", "fixed", "continuous"
    parent: str
    child: str
    axis: list[float] = [0.0, 0.0, 1.0]
    limits: list[float] | None = None
    origin: list[float] | None = None
    origin_rpy: list[float] | None = None
    effort: float | None = None
    velocity: float | None = None


class Chain(BaseModel):
    robot_name: str
    dh_params: dict[str, float] | None = None
    links: list[ChainLink] = []
    joints: list[ChainJoint] = []


# ---------------------------------------------------------------------------
# Summary models
# ---------------------------------------------------------------------------


class SummaryPart(BaseModel):
    name: str
    file: str
    role_hint: str


class Summary(BaseModel):
    robot_name: str
    part_count: int
    parts: list[SummaryPart] = []
    assembly_hints: list[str] = []


# ---------------------------------------------------------------------------
# Specs model (extra="allow" for robot-specific fields)
# ---------------------------------------------------------------------------


class Specs(BaseModel):
    model_config = ConfigDict(extra="allow")

    robot_name: str
    manufacturer: str | None = None
    dof: int | None = None


# ---------------------------------------------------------------------------
# View Mapping models
# ---------------------------------------------------------------------------


class View(BaseModel):
    manufacturer_label: str
    viewcube_face: str | None = None
    reference_image: str | None = None
    dir: list[float] | None = None
    up: list[float] | None = None
    notes: str | None = None


class ViewMapping(BaseModel):
    views: list[View] = []


# ---------------------------------------------------------------------------
# Schema tag helpers
# ---------------------------------------------------------------------------

_SCHEMA_TAG = "# yaml-language-server: $schema="


def _schema_relative_path(yaml_path: Path, schema_name: str) -> str:
    """Compute relative path from YAML file to schemas/<name>.json at repo root."""
    # Walk up to find the repo root (contains schemas/ dir or pyproject.toml)
    p = yaml_path.resolve().parent
    depth = 0
    while p != p.parent:
        if (p / "schemas").is_dir() or (p / "pyproject.toml").is_file():
            return "/".join([".."] * depth) + f"/schemas/{schema_name}.json"
        p = p.parent
        depth += 1
    # Fallback: just use relative prefix based on common layout
    return f"schemas/{schema_name}.json"


def _dump_yaml_with_tag(data: dict[str, Any], path: Path, schema_name: str) -> None:
    """Write YAML with a yaml-language-server schema tag at the top."""
    path.parent.mkdir(parents=True, exist_ok=True)
    rel = _schema_relative_path(path, schema_name)
    tag_line = f"{_SCHEMA_TAG}{rel}\n"
    body = yaml.dump(data, default_flow_style=False, sort_keys=False, width=120)
    path.write_text(tag_line + body)


def _load_yaml(path: Path) -> dict[str, Any]:
    """Load YAML, stripping any yaml-language-server tag."""
    with open(path) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Load / save helpers
# ---------------------------------------------------------------------------


def load_part_yaml(path: Path) -> PartAnalysis:
    data = _load_yaml(path)
    return PartAnalysis.model_validate(data)


def save_part_yaml(model: PartAnalysis, path: Path) -> None:
    data = model.model_dump(exclude_none=True)
    _dump_yaml_with_tag(data, path, "part_analysis")


def load_chain_yaml(path: Path) -> Chain:
    data = _load_yaml(path)
    return Chain.model_validate(data)


def save_chain_yaml(model: Chain, path: Path) -> None:
    data = model.model_dump(exclude_none=True)
    _dump_yaml_with_tag(data, path, "chain")


def load_summary_yaml(path: Path) -> Summary:
    data = _load_yaml(path)
    return Summary.model_validate(data)


def save_summary_yaml(model: Summary, path: Path) -> None:
    data = model.model_dump(exclude_none=True)
    _dump_yaml_with_tag(data, path, "summary")


def load_specs_yaml(path: Path) -> Specs:
    data = _load_yaml(path)
    return Specs.model_validate(data)


def save_specs_yaml(model: Specs, path: Path) -> None:
    data = model.model_dump(exclude_none=True)
    _dump_yaml_with_tag(data, path, "specs")


def load_view_mapping_yaml(path: Path) -> ViewMapping:
    data = _load_yaml(path)
    return ViewMapping.model_validate(data)


def save_view_mapping_yaml(model: ViewMapping, path: Path) -> None:
    data = model.model_dump(exclude_none=True)
    _dump_yaml_with_tag(data, path, "view_mapping")
