# Assembly Reasoning Skill

Infer robot arm kinematic chain from analyzed STL parts and generate a `chain.yaml` for automated URDF generation.

## When to Use

Use this skill when the user asks to generate a URDF for a robot arm from analyzed STL parts. The `robot-arm-sim analyze` command should have been run first, producing YAML files (including connection_points data) in `<robot_dir>/analysis/`.

## Architecture

```
analyze → [connection points] → skill writes chain.yaml → generate → URDF → simulate
               (exact math)        (topology only)         (exact math)
```

The skill writes a `chain.yaml` describing the kinematic chain topology (part order, joint types, axes, limits). The `robot-arm-sim generate` command computes exact transforms from mesh connection points and produces the URDF.

**The skill does NOT write raw URDF XML or guess xyz/rpy offsets.**

## Inputs

- `<robot_dir>/analysis/summary.yaml` — overview of all parts with assembly hints
- `<robot_dir>/analysis/<part>.yaml` — detailed analysis per part (geometry, features, connection_points)
- `<robot_dir>/stl_files/` — original STL mesh files

## Steps

1. **Read summary.yaml** to get the part list, role hints, and assembly hints.

2. **Read each part YAML**, paying special attention to `connection_points` data which gives precise bore/shaft center positions and axes for each part end.

3. **Infer the kinematic chain:**
   - Parts named sequentially (A0, A1, A2, ...) form a serial chain from base to end-effector.
   - Combined parts (e.g., A3_4) span multiple joints.
   - Connection point axes tell you the joint axis direction at each interface.

4. **Search for manufacturer specifications:**
   - Search for the robot name + "specifications" or "datasheet"
   - Get joint limits, link lengths (DH parameters), weight, payload specs
   - Cross-validate connection point distances against manufacturer data

5. **Write `chain.yaml`** with this format:

```yaml
robot_name: Robot-Name
dh_params:  # from manufacturer specs (mm)
  d1: 135
  a2: 135
  # etc.
links:
  - name: base_link
    mesh: A0
    # Optional overrides (usually not needed):
    # visual_xyz: [0, 0, 0]    # explicit visual origin offset (meters)
    # visual_rpy: [0, 0, 0]    # mesh rotation to align with link frame
  - name: link_1
    mesh: A1
  - name: link_2
    mesh: A2
    visual_rpy: [0, -1.5708, 0]  # if STL extends along X but should point up (Z)
  # For virtual links (no mesh):
  - name: link_4
    mesh: null
joints:
  - name: joint_1
    type: revolute
    parent: base_link
    child: link_1
    axis: [0, 0, 1]
    limits: [-3.054, 3.054]
    # Optional: explicit origin override (meters) if auto-detection is wrong
    # origin: [0, 0, 0.093]
  # ... etc for all joints
```

### Key decisions for chain.yaml:

- **Joint axes:** Determined from connection_points axis data. J1/J4/J6 (roll) → `[0,0,1]`; J2/J3/J5 (pitch) → `[0,1,0]`.
- **visual_rpy:** Only needed when STL coordinate frame doesn't match the link frame convention (Z-up). Parts extending along STL +X that should point up: `[0, -1.5708, 0]`.
- **Joint limits:** From manufacturer specs, in radians.
- **origin overrides:** Only use when connection point auto-detection gives wrong results. Prefer fixing the detection instead.

6. **Run the generate command:**

```bash
uv run robot-arm-sim generate <robot_dir> <robot_dir>/chain.yaml
```

7. **Run verification:**

```bash
uv run python <robot_dir>/verify_kinematics.py
```

Or with JSON output for programmatic checking:

```bash
uv run python <robot_dir>/verify_kinematics.py --json
```

8. **If validation fails**, adjust `chain.yaml` (add origin overrides, fix visual_rpy, etc.) and regenerate. Do NOT edit the URDF directly.

## Output

- `<robot_dir>/chain.yaml` — kinematic chain specification
- `<robot_dir>/robot.urdf` — generated automatically by `robot-arm-sim generate`

## URDF Frame Conventions

**CRITICAL:** Keep all link frames with Z pointing up at zero config. Do NOT add DH-style frame rotations.

- **Joint axes:** J1/J4/J6 (roll) → axis Z `(0,0,1)`; J2/J3/J5 (pitch) → axis Y `(0,1,0)`
- **Joint origins:** Computed automatically from connection points — pure translations, no rpy
- **Mesh rotations:** Only in visual_rpy in chain.yaml, to align STL coords with link frame

## Validation

After generating the URDF, validate it:
- All links referenced by joints must exist
- All STL file paths must resolve relative to the robot directory
- The kinematic tree must have exactly one root
- Zero-config joint positions match manufacturer's link lengths within 2mm
