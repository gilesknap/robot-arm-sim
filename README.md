[![CI](https://github.com/gilesknap/robot-arm-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/gilesknap/robot-arm-sim/actions/workflows/ci.yml)
[![Coverage](https://codecov.io/gh/gilesknap/robot-arm-sim/branch/main/graph/badge.svg)](https://codecov.io/gh/gilesknap/robot-arm-sim)
[![PyPI](https://img.shields.io/pypi/v/robot-arm-sim.svg)](https://pypi.org/project/robot-arm-sim)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

# robot-arm-sim

Analyze robot arm STL meshes, generate URDF models, and simulate them interactively in the browser.

The pipeline takes a set of STL files for a robot arm, detects geometric features and connection points, generates a URDF from a kinematic chain specification, and launches a browser-based 3D simulator with joint sliders.

Source          | <https://github.com/gilesknap/robot-arm-sim>
:---:           | :---:
PyPI            | `pip install robot-arm-sim`
Releases        | <https://github.com/gilesknap/robot-arm-sim/releases>

## Pipeline overview

```
STL files → analyze → chain.yaml → generate → URDF → simulate
```

1. **Analyze** -- parse STL meshes, detect flat faces, cylindrical surfaces, and connection points (bore/shaft centers at each end of a part)
2. **Generate** -- combine a `chain.yaml` kinematic chain spec with the analysis data to compute exact joint and visual transforms, producing a URDF
3. **Simulate** -- launch a NiceGUI web app with a 3D scene and joint sliders for interactive control

## Requirements

- Python 3.11+
- [uv](https://docs.astral.sh/uv/) package manager
- System dependency: `libglu1-mesa` (for pyrender/EGL headless rendering)

## Installation

```bash
git clone https://github.com/gilesknap/robot-arm-sim.git
cd robot-arm-sim
uv sync
```

## Quick start with the included Meca500-R3

The repository ships with a pre-analyzed Meca500-R3 robot. To launch the simulator:

```bash
uv run robot-arm-sim simulate robots/Meca500-R3/
```

This opens a browser at `http://localhost:8080` with the 3D model and joint sliders.

To regenerate the URDF from the chain spec:

```bash
uv run robot-arm-sim generate robots/Meca500-R3/ robots/Meca500-R3/chain.yaml
```

To re-run the full analysis (feature detection, connection points, renders):

```bash
uv run robot-arm-sim analyze robots/Meca500-R3/
```

## Adding a new robot

### 1. Prepare STL files

Create a directory under `robots/` with your robot name and place the STL mesh files (one per link) inside `stl_files/`:

```
robots/MyRobot/
  stl_files/
    A0.stl    # base
    A1.stl    # link 1
    A2.stl    # link 2
    ...
```

STL files should be in millimetres. Name them so they sort in kinematic chain order.

### 2. Run analysis

```bash
uv run robot-arm-sim analyze robots/MyRobot/
```

This produces `robots/MyRobot/analysis/` containing:
- Per-part YAML files with geometry, features, and detected connection points
- Rendered PNG views of each part
- A `summary.yaml` with assembly hints

### 3. Write chain.yaml

Create `robots/MyRobot/chain.yaml` describing the kinematic chain. This is where you specify joint types, axes, limits, and link-to-mesh mappings. Use manufacturer specs for DH parameters and joint limits.

```yaml
robot_name: MyRobot
dh_params:
  d1: 135     # link lengths in mm (for validation)
  a2: 200

links:
  - name: base_link
    mesh: A0
  - name: link_1
    mesh: A1
  - name: link_2
    mesh: A2
    visual_rpy: [0, -1.5708, 0]   # only if STL needs rotating to align with link frame

joints:
  - name: joint_1
    type: revolute
    parent: base_link
    child: link_1
    axis: [0, 0, 1]
    limits: [-3.14, 3.14]
    # origin is computed from connection points automatically,
    # or override explicitly:
    # origin: [0, 0, 0.135]
```

Key fields:

| Field | Purpose |
|---|---|
| `links[].mesh` | Which STL to use (by stem name), or `null` for virtual links |
| `links[].visual_rpy` | Rotation to align STL coords with the link frame (Z-up) |
| `joints[].axis` | Joint rotation axis: `[0,0,1]` for yaw/roll, `[0,1,0]` for pitch |
| `joints[].limits` | Joint limits in radians |
| `joints[].origin` | Explicit joint origin in metres (overrides auto-detection) |

The assembly-reasoning Claude skill (`.claude/skills/assembly-reasoning.md`) can generate this file from the analysis data and manufacturer specs.

### 4. Generate URDF

```bash
uv run robot-arm-sim generate robots/MyRobot/ robots/MyRobot/chain.yaml
```

This reads the connection points from the analysis YAMLs, computes visual and joint transforms, and writes `robots/MyRobot/robot.urdf`. It also prints FK validation showing the zero-config position of each joint.

### 5. Verify kinematics (optional)

If you have a `verify_kinematics.py` in the robot directory, run it to compare against expected joint positions:

```bash
uv run python robots/MyRobot/verify_kinematics.py
uv run python robots/MyRobot/verify_kinematics.py --json   # structured output
```

### 6. Launch simulator

```bash
uv run robot-arm-sim simulate robots/MyRobot/
uv run robot-arm-sim simulate robots/MyRobot/ --port 9000  # custom port
```

## CLI reference

```
robot-arm-sim analyze  <robot_dir>                    # Analyze STL meshes
robot-arm-sim generate <robot_dir> <chain.yaml>       # Generate URDF
robot-arm-sim simulate <robot_dir> [--port 8080]      # Launch simulator
```

## How connection point detection works

The `analyze` command detects connection points (bore/shaft centers) at each end of every mesh:

1. For each detected cylindrical surface, use its axis as the candidate joint axis
2. Slice cross-sections near both ends of the mesh along that axis (at 5%, 10%, 15% of extent)
3. Find circular features in each cross-section by checking polygon circularity
4. Compute the bore center as the centroid of the most circular feature, transformed back to 3D STL coordinates
5. Classify as proximal (parent-side) or distal (child-side) based on position along the axis

Special handling exists for base links (distal only), L-shaped multi-joint parts (multiple axes), and non-watertight meshes (vertex-based circle fitting fallback).
