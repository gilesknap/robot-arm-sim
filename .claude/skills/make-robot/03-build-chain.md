# 03 — Build Kinematic Chain

Combine analysis data and manufacturer specs to write `chain.yaml`.

## Input

- `robots/<name>/analysis/summary.yaml` and per-part YAMLs
- `robots/<name>/specs.yaml` (from step 02)

## Steps

### 1. Read Analysis Data

1. Read `summary.yaml` for part ordering and assembly hints
2. Read each `<part>.yaml` for connection points, bore positions, axes

### 2. Determine Chain Topology

- Sequential parts (A0, A1, ...) or semantic names (base, shoulder, ...) form a serial chain
- For semantic names: determine order from geometry (base is flat/short), bounding box sizes, manufacturer docs
- Combined parts (e.g., A3_4) span multiple joints — create a virtual link with `mesh: null`
- Connection point axes suggest joint rotation axis at each interface

### 3. Compute Joint Origins from DH Parameters

**CRITICAL**: ALWAYS use explicit `origin` values from DH params. Bore-detected connection points are for validation only.

#### Simple robots (all DH alpha = 0)
All joints stack vertically at zero config:
- joint_1 origin: `[0, 0, d1_base]` (base height in metres)
- joint_2 origin: `[0, 0, remaining_d1]`
- Subsequent joints: DH `a` and `d` as Z distances

#### Robots with DH alpha rotations (e.g., UR5)
Need `origin_rpy` and may have lateral offsets:
1. Identify which joints have α ≠ 0
2. For each α = ±π/2: add `origin_rpy` to that joint
3. Origin values follow URDF convention — DH `a` and `d` go into different components depending on accumulated frame rotation

**UR-family example:**
```yaml
# J1: origin [0, 0, d1/1000]
# J2: origin [0, shoulder_offset/1000, 0], origin_rpy: [0, pi/2, 0]
# J3: origin [0, elbow_offset/1000, a2/1000]
# J4: origin [0, 0, a3/1000], origin_rpy: [0, pi/2, 0]
# J5: origin [0, d4/1000, 0]
# J6: origin [0, 0, d5/1000]
```

### 4. Write chain.yaml

```yaml
robot_name: <Robot-Name>
dh_params:  # from manufacturer specs (mm)
  d1: ...
  a2: ...
  # shoulder_offset: ...  # if applicable
  # elbow_offset: ...     # if applicable

links:
  - name: base_link
    mesh: <stl_stem>        # matches stl_files/<stem>.stl
  - name: link_1
    mesh: <stl_stem>
  # ... one entry per link
  # Virtual links: mesh: null

joints:
  - name: joint_1
    type: revolute
    parent: base_link
    child: link_1
    axis: [0, 0, 1]
    limits: [-3.054, 3.054]   # from specs.yaml, in radians
    origin: [0, 0, 0.093]    # ALWAYS explicit from DH params
    # origin_rpy: [0, 1.5707963, 0]  # if DH alpha ≠ 0
  # ... one entry per joint
```

### 5. Cross-Validate

Compare chain.yaml origins against bore-detected connection points:
- If a joint origin differs from the detected bore by >10mm, investigate
- The DH params should win, but large discrepancies suggest a problem

## Field Reference

| Field | Purpose | When to use |
|-------|---------|-------------|
| `links[].mesh` | STL file stem, or `null` for virtual links | Always |
| `links[].visual_xyz` | Additive visual offset (metres) | When auto-detection places mesh wrong |
| `links[].visual_rpy` | Mesh rotation (radians) | When STL orientation doesn't match link frame |
| `joints[].axis` | Rotation axis: `[0,0,1]` for yaw, `[0,1,0]` for pitch | Always |
| `joints[].limits` | Joint angle limits in radians | Always |
| `joints[].origin` | Joint position (metres) | **Always** — use DH params |
| `joints[].origin_rpy` | Frame rotation (radians) | When DH alpha ≠ 0 |

## Output

- `robots/<name>/chain.yaml`

## Gate

- Valid YAML
- All links have mesh references (or explicit `null`)
- All joints have explicit `origin` values
- All joints have `limits` from manufacturer specs
- Joint count matches DOF from specs.yaml
