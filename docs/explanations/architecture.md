# Architecture

This document describes the overall architecture of robot-arm-sim, how the
main components fit together, and the design decisions behind them.

## Pipeline overview

Robot-arm-sim is built around a three-stage pipeline that takes raw STL mesh
files and produces an interactive 3D simulation in the browser:

```
STL files → analyze → chain.yaml → generate → URDF → simulate
```

Each stage is exposed as a CLI subcommand (`analyze`, `generate`, `simulate`)
so they can be run independently or composed into an end-to-end workflow.

## Main components

### Analysis (`robot_arm_sim.analyze`)

The analysis stage inspects STL mesh files and extracts geometric information
needed to build a kinematic model:

- **Feature detection** (`features.py`) — identifies flat faces (potential
  mating surfaces) via facet clustering and cylindrical surfaces via normal
  ring patterns. Reports area, centroid, and normal vectors.
- **Connection point detection** (`connections.py`) — locates bore and shaft
  centres by analysing cylindrical surface axes, slicing cross-sections near
  mesh ends, and fitting circles to the profiles. Each connection point is
  classified as *proximal* (parent-side) or *distal* (child-side).
- **URDF generation** (`urdf_generator.py`) — combines a hand-authored
  `chain.yaml` kinematic specification with the per-part analysis results to
  compute exact joint origins and produce a valid URDF XML file.
Analysis results are written as YAML files into an `analysis/` directory
alongside the source meshes.

### Data models (`robot_arm_sim.models`)

Typed data models shared across the pipeline:

- **`PartAnalysis`** (Pydantic `BaseModel`) — mesh geometry (vertices, faces,
  bounding box, volume, centre of mass), detected geometric features, and
  connection points.
- **`URDFRobot`**, **`URDFLink`**, **`URDFJoint`** (dataclasses) — a structured
  representation of the URDF model including mesh paths, joint types, axes,
  limits, and kinematic chain ordering.

### Simulation (`robot_arm_sim.simulate`)

The simulation stage launches a web-based 3D viewer:

- **URDF loader** (`urdf_loader.py`) — parses the generated URDF XML into
  the `URDFRobot` dataclass, validates that the kinematic tree is connected,
  and resolves mesh file paths.
- **Forward kinematics** (`kinematics.py`) — computes 4×4 homogeneous
  transforms for every link using axis-angle rotation (Rodrigues' formula)
  and RPY conventions. The chain is traversed from root to tip on each
  joint-angle update.
- **Inverse kinematics** (`ik_solver.py`) — a custom damped least-squares
  solver that computes the 6×N geometric Jacobian analytically from the URDF
  joint chain and iteratively drives both position and orientation error to
  zero. Used by the simulator's IK mode to let users control the
  end-effector pose directly. See
  {doc}`/explanations/inverse-kinematics` for a detailed explanation.
- **Web application** (`app/`) — a NiceGUI application split across several
  modules (scene, controls, toolbar, state, edit-connections) that builds the
  interactive 3D scene, joint-angle sliders, and toolbar controls. STL
  meshes are served as static files and loaded into a Three.js scene via
  NiceGUI's `ui.scene()` component.

### Mesh parsers (`robot_arm_sim.analyze.parsers`)

A small abstraction layer for mesh file formats. An `AbstractMeshParser`
interface defines `supported_extensions()` and `parse()` methods;
`STLParser` is the current concrete implementation using trimesh.

## Coordinate systems and units

- **STL files** are in millimetres, Z-up.
- **URDF** uses metres — a scale factor of 0.001 is applied when loading
  meshes into the simulation scene.
- **Joint axes** are unit vectors (e.g. `[0, 0, 1]` for a yaw rotation).
- **Transform chain**: `T_child = T_parent × T_joint_origin × R_joint(θ)`.

## Rendering stack

The browser-based simulator uses several layers:

1. **NiceGUI** — Python web framework providing the UI layout, sliders, and
   a `ui.scene()` 3D viewport backed by Three.js.
2. **Three.js** — handles WebGL rendering. Post-initialisation JavaScript
   upgrades mesh materials to PBR (`MeshStandardMaterial` with
   metalness/roughness), generates a procedural studio HDRI environment map,
   and configures camera controls.
3. **Forward kinematics on the server** — joint-angle changes trigger a
   server-side FK computation; the resulting transforms are broadcast to
   the browser over WebSocket.

## Key dependencies

| Dependency | Purpose |
|---|---|
| trimesh | Mesh I/O, geometry queries, inertia |
| numpy | Matrix maths, transforms, and the damped least-squares IK solver |
| nicegui | Web UI framework (Vue.js + Quasar frontend) |
| typer | CLI argument parsing |
| pyyaml | YAML configuration parsing |
| pydantic | Data validation and serialisation for analysis models |

## Robot directory layout

Each robot lives in its own directory under `robots/`. The core files are:

```
robots/<RobotName>/
├── stl_files/          # Source STL meshes (one per link)
├── analysis/           # Generated analysis YAML per part
├── chain.yaml          # Kinematic chain specification
└── robot.urdf          # Generated URDF model
```

Individual robots may also contain specs files, verification scripts,
reference images, or view mappings — the exact set varies by robot.

The `chain.yaml` file defines joint types, rotation axes, angle limits, and
which mesh files map to which links. This is the only file that requires
manual authoring — the rest is generated by the pipeline.
