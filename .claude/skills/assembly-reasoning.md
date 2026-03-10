# Assembly Reasoning Skill

Infer robot arm assembly from analyzed STL parts and generate a valid URDF file.

## When to Use

Use this skill when the user asks to generate a URDF for a robot arm from analyzed STL parts. The `robot-arm-sim analyze` command should have been run first, producing YAML files in `<robot_dir>/analysis/`.

## Inputs

- `<robot_dir>/analysis/summary.yaml` — overview of all parts with assembly hints
- `<robot_dir>/analysis/<part>.yaml` — detailed analysis per part (geometry, features, text description)
- `<robot_dir>/stl_files/` — original STL mesh files

## Steps

1. **Read summary.yaml** to get the part list, role hints, and assembly hints.

2. **Read each part YAML** to understand geometry, features (flat faces, cylindrical surfaces, holes), and text descriptions.

3. **Infer the kinematic chain:**
   - Check summary.yaml's `assembly_hints` for sequential naming patterns. If present, parts form a serial chain from base to end-effector.
   - If no sequential pattern, infer ordering from geometric mating features (matching cylindrical surfaces and flat faces between adjacent parts).
   - Combined parts (noted by underscore in name, e.g., `part3_4`) span multiple joints — create intermediate virtual links with no mesh.
   - The part with a "possible base" role hint or the largest flat bottom face is likely the base.

4. **Identify mating features between adjacent parts:**
   - Concentric cylindrical surfaces at part interfaces → revolute joint axes
   - Coplanar flat faces → mating surfaces that define joint origins
   - Matching hole patterns → bolt connections (fixed joints)

5. **Determine joint parameters:**
   - Joint type: `revolute` for rotary joints, `fixed` for rigid connections
   - Joint axis: from the cylindrical feature direction at the interface
   - Joint origin: position where parts connect, in the parent link's frame
   - Joint limits: search for manufacturer specifications first (see step 6). As a last resort, use conservative defaults (±180° for first/last joints, ±135° for intermediate joints).

6. **Search for manufacturer specifications** using the `robot_name` from summary.yaml:
   - Use WebSearch for the robot name + "specifications", "datasheet", or "user manual"
   - Look for joint limits, link lengths (DH parameters), weight, payload specs
   - Cross-validate geometric analysis against manufacturer data
   - Check if any reference documents (PDF, manual) exist in the robot directory

7. **Generate URDF** with:
   - One `<link>` per part, referencing STL mesh with `scale="0.001 0.001 0.001"` (mm→m)
   - One `<joint>` between each adjacent pair
   - Proper `<origin>` transforms placing each link correctly
   - `<limit>` elements on revolute joints
   - The number of joints and links depends on the robot — do NOT assume a specific count. Use the part list from summary.yaml.

8. **Create or run `verify_kinematics.py`:**
   - If the script already exists in the robot directory, run it after generating the URDF.
   - If it doesn't exist, create one that: (a) parses the generated URDF, (b) computes zero-config forward kinematics, (c) prints each joint's world position in mm, (d) if manufacturer specs were found, includes expected positions as assertions.

## Output

Write the URDF to `<robot_dir>/robot.urdf`.

## URDF Frame Conventions

**CRITICAL:** Keep all link frames with Z pointing up at zero config. Do NOT add DH-style
frame rotations (like α rotations) at joint origins — these cause confusing coordinate
mismatches between joint xyz offsets and the expected robot geometry.

- **Joint axes:** Determine each joint's rotation axis from the cylindrical features at its interface. Common pattern for 6-DOF arms is Z-Y-Y-Z-Y-Z (alternating roll/pitch), but always verify against the part geometry. For robots with different DOF counts, derive axes purely from analysis.
- **Joint origins:** Pure translations — no rpy needed for serial arm joints
- **Mesh rotations:** Only in `<visual><origin rpy="..."/>` to align STL coords with link frame
  - Parts modeled with Z up: `rpy="0 0 0"` (no rotation)
  - Parts extending along STL +X that should point up: `rpy="0 -1.5708 0"` (maps X→Z)
  - Parts extending along STL +X that should point down: `rpy="0 1.5708 0"` (maps X→−Z)

Always verify the kinematic chain by computing zero-config world positions and checking
they match the manufacturer's link lengths.

## URDF Template

The template below shows the general structure. Adapt the number of links/joints, filenames, axes, and limits based on the analysis output and manufacturer specs.

```xml
<?xml version="1.0"?>
<robot name="{robot_name from summary.yaml}">
  <!-- Base link: use the first part (or part with base role hint) -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="stl_files/{base_part_filename}" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <!-- One joint per connection, parameters from analysis + specs -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="{from mating feature positions}" rpy="0 0 0"/>
    <axis xyz="{from cylindrical feature analysis}"/>
    <limit lower="{from specs}" upper="{from specs}" effort="100" velocity="1"/>
  </joint>

  <link name="link_1">
    <visual>
      <geometry>
        <mesh filename="stl_files/{next_part_filename}" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <!-- Continue for ALL parts in the analysis. The number of joints/links
       varies per robot — always match the part count from summary.yaml. -->
</robot>
```

## Validation

After generating the URDF, validate it:
- All links referenced by joints must exist
- All STL file paths must resolve relative to the robot directory
- The kinematic tree must have exactly one root
- XML must be well-formed

Use the validation utility:
```python
from robot_arm_sim.simulate.urdf_loader import validate_urdf
errors = validate_urdf(robot_dir)
```

If there are errors, fix them and regenerate.
