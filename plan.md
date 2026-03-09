# Robot Arm Sim — Implementation Plan

## Context

This project aims to build an end-to-end pipeline for analyzing robot arm CAD models and simulating them interactively in a browser. The pipeline:

```
robot-arm-sim analyze → [Claude skill] → URDF → robot-arm-sim simulate
```

Each stage reads from and writes to a robot folder (e.g. `robots/one/`), and each is independently re-runnable. The project serves as a demo of building a project collaboratively with Claude.

## Phase 1: Foundation (CLI + Dependencies)

### 1a. Add dependencies to `pyproject.toml`

```toml
dependencies = [
    "typer>=0.9",
    "trimesh[easy]>=4.0",
    "pyyaml>=6.0",
    "numpy>=1.24",
    "nicegui>=1.4",
    "ikpy>=3.3",
]
```

Add second entry point:
```toml
[project.scripts]
robot-arm-sim = "robot_arm_sim.__main__:main"
simulate = "robot_arm_sim.simulate:main"
```

Install with `uv sync`.

### 1b. Replace argparse with Typer in `src/robot_arm_sim/__main__.py`

- Typer app with `--version` via eager callback
- `analyze` subcommand (takes robot directory path)
- `simulate` subcommand (takes robot directory path)
- Deferred imports for heavy deps (trimesh, nicegui) so `--version` stays fast

### 1c. Update `tests/test_cli.py` for Typer

### 1d. Module structure

```
src/robot_arm_sim/
    __init__.py              # existing
    __main__.py              # rewrite: Typer CLI
    _version.py              # existing, auto-generated
    analyze/
        __init__.py          # run_analysis() orchestrator
        parsers/
            __init__.py      # parser registry
            base.py          # AbstractMeshParser ABC
            stl_parser.py    # STL via trimesh
        features.py          # geometric feature detection
        renderer.py          # off-screen PNG rendering
        yaml_writer.py       # YAML output
    simulate/
        __init__.py          # run_simulator() + standalone main()
        app.py               # NiceGUI application
        urdf_loader.py       # URDF XML parsing
        kinematics.py        # FK/IK
        scene.py             # Three.js scene helpers
        widgets.py           # sliders, controls, camera presets
    models/
        __init__.py
        part.py              # PartAnalysis + GeometricFeature dataclasses
        robot.py             # URDFRobot/URDFLink/URDFJoint dataclasses
```

## Phase 2: `analyze` Subcommand

### 2a. Abstract parser (`analyze/parsers/base.py`)
- `AbstractMeshParser` ABC with `supported_extensions()` and `parse()` methods
- Registry in `parsers/__init__.py` maps file extensions to parser classes
- Extensible to STEP, OBJ etc. later

### 2b. STL parser (`analyze/parsers/stl_parser.py`)
- Load binary STL via `trimesh.load()`
- Extract: bounding box, volume, surface area, center of mass, vertex/face counts, watertightness
- Principal axes and moments of inertia

### 2c. Geometric feature detection (`analyze/features.py`)
- **Flat faces**: Use `mesh.facets` to find large coplanar face groups → candidate mating surfaces
- **Cylindrical surfaces**: Cluster face normals forming ring patterns, PCA on centroids to find axis → candidate joints/shafts
- **Holes**: Concave cylindrical surfaces with small radius
- **Symmetry**: Check rotational symmetry about candidate axes
- Each feature gets a structured record + human-readable description string

### 2d. Off-screen rendering (`analyze/renderer.py`)
- `trimesh.Scene` + `save_image()` for multi-angle PNGs (front, side, top, iso)
- 800x600 per image, mesh centered and scaled
- Graceful fallback if headless/no GPU — rendering is supplementary, YAML is primary

### 2e. YAML output format (`analyze/yaml_writer.py`)

Per-part YAML (e.g. `robots/one/analysis/A0.yaml`):
```yaml
part_name: A0
source_file: stl_files/A0.stl
format: binary_stl
geometry:
  vertex_count: 12345
  face_count: 4115
  bounding_box: { min: [x,y,z], max: [x,y,z], extents: [dx,dy,dz] }
  volume_mm3: 123456.78
  surface_area_mm2: 98765.43
  center_of_mass: [cx, cy, cz]
  is_watertight: true
inertia:
  principal_moments: [Ixx, Iyy, Izz]
  principal_axes: [[ax,ay,az], [bx,by,bz], [cx,cy,cz]]
features:
  flat_faces:
    - description: "Large flat face on bottom, likely base mounting surface"
      normal: [0, 0, -1]
      area_mm2: 2345.6
      centroid: [x, y, z]
  cylindrical_surfaces:
    - description: "Vertical cylinder at top center, likely joint rotation shaft"
      axis: [0, 0, 1]
      radius_mm: 15.2
      length_mm: 30.5
      center: [x, y, z]
      concave: false
  holes:
    - description: "Mounting bolt hole, M4 diameter"
      axis: [0, 0, 1]
      radius_mm: 2.0
      center: [x, y, z]
text_description: |
  Auto-generated prose summary of the part for Claude to reason about.
renders:
  - renders/A0_front.png
  - renders/A0_iso.png
```

Assembly summary (`robots/one/analysis/summary.yaml`):
```yaml
robot_name: one
part_count: 6
parts:
  - name: A0
    file: A0.yaml
    role_hint: "Base - largest flat face at bottom, cylindrical shaft at top"
  # ...
assembly_hints:
  - "Parts named A0-A6, suggesting sequential kinematic chain"
  - "A3_4 is combined, likely forearm connecting joints 3-4"
```

### 2f. Orchestrator (`analyze/__init__.py`)
- Discover STL files in `<robot_dir>/stl_files/`
- Parse each, detect features, render, write YAML
- Write summary.yaml

### 2g. Robot folder output structure
```
robots/one/
├── stl_files/           # source (unchanged)
├── analysis/
│   ├── A0.yaml ... A6.yaml
│   ├── summary.yaml
│   └── renders/
│       ├── A0_front.png, A0_iso.png, ...
```

### 2h. Verification
- `robot-arm-sim analyze robots/one/` creates expected files
- YAML bounding boxes match reasonable dimensions
- Each YAML has features and text_description

## Phase 3: Claude Skill for Assembly Reasoning

### 3a. Skill file: `.claude/skills/assembly-reasoning.md`

Instructions for Claude to:
1. Read `summary.yaml` for overview + assembly hints
2. Read each part YAML
3. For adjacent part pairs, identify mating features (concentric cylinders, coplanar flat faces)
4. Infer joint axes from cylindrical features at mating interfaces
5. Determine transforms (position + orientation) between links
6. **If the analysis data is insufficient** (e.g. joint limits unclear, link dimensions ambiguous, part roles uncertain), search online for the manufacturer's specifications, datasheets, or user manuals to fill in the gaps
7. Output valid URDF referencing original STL files

### 3b. URDF output
- Links reference STL meshes with `scale="0.001 0.001 0.001"` (mm → meters)
- Joints specify type (revolute), axis, origin, limits
- Written to `robots/one/robot.urdf`

### 3c. Online research fallback
- The skill should use WebSearch to find manufacturer datasheets, user manuals, and specs when STL analysis alone doesn't provide enough info (e.g. joint limits, link lengths, motor specs, weight)
- Search terms derived from robot folder name, part naming conventions, or any text found in the analysis
- Cross-validate geometric analysis against manufacturer data

### 3d. URDF validation utility
- `validate_urdf(robot_dir)` — parse XML, verify all links/joints connected, STL refs resolve

### 3d. Verification
- Claude generates valid URDF with 7 links (base + 6) and 6 revolute joints
- All STL file references are correct relative paths
- XML is well-formed

## Phase 4: `simulate` Subcommand (Core)

### 4a. URDF loader (`simulate/urdf_loader.py`)
- Parse URDF XML with `xml.etree.ElementTree`
- Extract links (mesh paths), joints (type, axis, origin, limits, parent/child)
- Build `URDFRobot` dataclass with kinematic tree

### 4b. Forward kinematics (`simulate/kinematics.py`)
- Walk kinematic chain base→tip
- Each joint: `child_transform = parent_transform @ origin_transform @ rotation(axis, angle)`
- Pure numpy matrix math, returns `dict[link_name, 4x4_matrix]`

### 4c. NiceGUI app (`simulate/app.py`)
- **Left panel**: Joint sliders (one per revolute joint, range from URDF limits, in degrees)
- **Right panel**: `ui.scene()` with STL meshes loaded via `scene.stl()`
- Serve STL files via `app.add_static_files()`
- On slider change: recompute FK, update mesh transforms
- Transform application: decompose 4x4 into position + euler, or use `ui.run_javascript()` for direct matrix set

### 4d. Verification
- Browser opens with 3D robot view at zero angles
- All 6 STL meshes visible and correctly positioned
- Sliders rotate downstream links correctly
- Joint limits enforced

## Phase 5: Advanced Features

Each is independent, layered on Phase 4:

### 5a. Coordinate frame visualization
- Toggle-able RGB axis lines at each joint origin
- Checkbox in control panel

### 5b. Joint limit feedback
- Slider color: green (safe) → yellow (near limit) → red (at limit)
- Visual feedback on mesh near limits

### 5c. Camera presets
- Buttons: Front, Side, Top, Isometric
- Set Three.js camera via `ui.run_javascript()`

### 5d. Inverse kinematics
- `ikpy.chain.Chain.from_urdf_file()` to build chain
- Click in 3D scene → raycaster → IK solve → animate joints to solution
- Draggable target sphere

### 5e. Collision detection
- `trimesh.collision.CollisionManager` checks mesh intersections
- Colliding parts highlight red
- Debounced on joint angle updates

### 5f. Trajectory recording/playback
- Record: save joint angles + timestamps
- Playback: interpolate keyframes, animate
- Store as JSON in robot folder
- Record/Stop/Play buttons in UI

## Technical Risks

1. **Off-screen rendering**: trimesh needs display/EGL. Mitigate with graceful fallback + warning.
2. **NiceGUI 4x4 transforms**: `ui.scene` may not expose `set_matrix()`. Mitigate with `ui.run_javascript()` or decompose to position+euler.
3. **STL coordinate origins**: Parts may have inconsistent origins. The analysis detects this; URDF must account for mesh offsets.
4. **Cylindrical detection accuracy**: Tessellated meshes need tolerance (~5 degrees) for normal clustering. Tune on actual data.
5. **ikpy URDF compatibility**: Generated URDF must specify active/passive joints correctly.

## Implementation Order

1. Phase 1: Foundation (~30 min) — CLI, deps, module structure
2. Phase 2: Analyze (~2 hrs) — parser, features, renderer, YAML output
3. Phase 3: Skill (~30 min) — assembly reasoning skill file
4. Phase 4: Simulate core (~2 hrs) — URDF loader, FK, NiceGUI app
5. Phase 5: Advanced features — one at a time as desired
