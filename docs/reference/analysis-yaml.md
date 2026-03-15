# Analysis YAML format

The `robot-arm-sim analyze` command produces one YAML file per STL part plus
a `summary.yaml` in the `analysis/` directory. These files capture the
geometric analysis that the `generate` command uses to compute joint origins
and visual offsets.

## Per-part YAML

Each file (e.g. `analysis/A0.yaml`) has the following sections.

### Top-level fields

| Field | Type | Description |
|-------|------|-------------|
| `part_name` | string | Part identifier (STL file stem) |
| `source_file` | string | Relative path to the source STL |
| `format` | string | Mesh format (e.g. `binary_stl`) |

### `connection_points`

A list of detected connection points. Each entry:

| Field | Type | Description |
|-------|------|-------------|
| `end` | string | `proximal` (parent-side) or `distal` (child-side) |
| `position` | list of 3 floats | Connection centre position in mm (STL coordinates) |
| `axis` | list of 3 floats | Normalised connection axis vector |
| `radius_mm` | float | Detected connection radius in mm |
| `method` | string | Detection method (e.g. `cross_section`) |
| `centering` | string or null | `"surface"` (default) or `"center"` — controls how the visual origin is computed. See {doc}`/explanations/urdf-generation-pipeline` |

#### Valid `method` values

| Value | Description |
|-------|-------------|
| `cross_section` | Detected by slicing mesh perpendicular to bore axis and fitting circles |
| `cylinder_fit` | Detected by fitting a circle to boundary vertices near a mesh end |
| `barrel_bore_face` | Detected on barrel-shaped parts using flat-face evidence along the bore axis |
| `centroid_fallback` | Fallback: mesh centroid used when no geometric features found |
| `largest_face_fallback` | Fallback: largest flat face used when no cylindrical features found |
| `manual` | Manually placed via the Edit Connections UI |

The `generate` command uses these positions to compute joint origins
(distal-to-proximal distance between connected parts) and visual offsets
(shift mesh so proximal connection centre is at the link frame origin).

### `geometry`

| Field | Type | Description |
|-------|------|-------------|
| `vertex_count` | int | Number of mesh vertices |
| `face_count` | int | Number of mesh faces |
| `bounding_box.min` | list of 3 floats | Bounding box minimum corner (mm) |
| `bounding_box.max` | list of 3 floats | Bounding box maximum corner (mm) |
| `bounding_box.extents` | list of 3 floats | Bounding box dimensions (mm) |
| `volume_mm3` | float | Mesh volume in mm³ |
| `surface_area_mm2` | float | Surface area in mm² |
| `center_of_mass` | list of 3 floats | Centre of mass (mm) |
| `is_watertight` | bool | Whether the mesh is a closed solid |

### `inertia`

| Field | Type | Description |
|-------|------|-------------|
| `principal_moments` | list of 3 floats | Principal moments of inertia |
| `principal_axes` | list of 3 lists | 3×3 principal axis matrix |

### `features`

Detected geometric features used for connection and mating surface
identification.

#### `flat_faces`

Each entry:

| Field | Type | Description |
|-------|------|-------------|
| `description` | string | Human-readable summary |
| `normal` | list of 3 floats | Face normal vector |
| `area_mm2` | float | Face area in mm² |
| `centroid` | list of 3 floats | Face centroid (mm) |

#### `cylindrical_surfaces`

Each entry:

| Field | Type | Description |
|-------|------|-------------|
| `description` | string | Human-readable summary |
| `axis` | list of 3 floats | Cylinder axis |
| `radius_mm` | float | Cylinder radius in mm |
| `length_mm` | float | Cylinder length in mm |
| `center` | list of 3 floats | Cylinder centre (mm) |
| `concave` | bool | Whether the surface is concave (a hole) |

#### `holes`

Holes are concave cylindrical surfaces with a radius less than 5 mm,
detected by `_detect_cylindrical_surfaces` in `features.py`. Face normals
that point inward (toward the cylinder centre) indicate concavity; the
radius threshold separates small holes from larger bore features reported
as `cylindrical_surfaces`.

Each entry:

| Field | Type | Description |
|-------|------|-------------|
| `description` | string | Human-readable summary |
| `axis` | list of 3 floats | Hole axis |
| `radius_mm` | float | Hole radius in mm |
| `length_mm` | float | Hole length in mm |
| `center` | list of 3 floats | Hole centre (mm) |

### `text_description`

A human-readable text summary of the part geometry.

---

## summary.yaml

One file per robot, summarising all analysed parts.

| Field | Type | Description |
|-------|------|-------------|
| `robot_name` | string | Robot name (from directory) |
| `part_count` | int | Number of analysed parts |
| `parts` | list | Per-part entries (see below) |
| `assembly_hints` | list of strings | Auto-generated hints about assembly topology |

Each entry in `parts`:

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | Part name |
| `file` | string | Relative path to the part YAML |
| `role_hint` | string | Auto-generated description of likely role (e.g. "large part; flat bottom face (possible base)") |

Role hints are generated by `_infer_role_hint` using several heuristics:
parts with a bounding-box extent over 100 mm are tagged "large part";
the largest flat face pointing downward (-Z) adds "flat bottom face
(possible base)"; all detected cylindrical surfaces are counted; a
leading `0` in the part name adds "first part in sequence (likely base)";
and an underscore in the name flags "combined part (may span multiple
joints)". Parts matching none of these receive the default hint
"standard link".

## Example

See the Meca500-R3 analysis directory (`robots/Meca500-R3/analysis/`) for a
complete set of analysis files.
