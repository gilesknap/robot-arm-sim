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
   - Parts named sequentially (A0, A1, A2, ...) form a serial chain from base to end-effector.
   - Combined parts (e.g., A3_4) span multiple joints.
   - The largest part with a flat bottom face is likely the base.

4. **Identify mating features between adjacent parts:**
   - Concentric cylindrical surfaces at part interfaces → revolute joint axes
   - Coplanar flat faces → mating surfaces that define joint origins
   - Matching hole patterns → bolt connections (fixed joints)

5. **Determine joint parameters:**
   - Joint type: `revolute` for rotary joints, `fixed` for rigid connections
   - Joint axis: from the cylindrical feature direction at the interface
   - Joint origin: position where parts connect, in the parent link's frame
   - Joint limits: use typical robot arm ranges (±175° for base, ±120° for elbows, etc.)

6. **If analysis data is insufficient**, use WebSearch to find manufacturer specifications:
   - Search for the robot name (from the folder name) + "specifications" or "datasheet"
   - Look for joint limits, link lengths, weight, payload specs
   - Cross-validate geometric analysis against manufacturer data

7. **Generate URDF** with:
   - One `<link>` per part, referencing STL mesh with `scale="0.001 0.001 0.001"` (mm→m)
   - One `<joint>` between each adjacent pair
   - Proper `<origin>` transforms placing each link correctly
   - `<limit>` elements on revolute joints

8. **If a `verify_kinematics.py` script exists** in the robot directory, run it after
   generating the URDF to verify zero-config joint positions match expected values.
   If the script doesn't exist, create one (see the Meca500-R3 example for a template).

## Output

Write the URDF to `<robot_dir>/robot.urdf`.

## URDF Frame Conventions

**CRITICAL:** Keep all link frames with Z pointing up at zero config. Do NOT add DH-style
frame rotations (like α rotations) at joint origins — these cause confusing coordinate
mismatches between joint xyz offsets and the expected robot geometry.

- **Joint axes:** J1/J4/J6 (roll) → axis Z `(0,0,1)`; J2/J3/J5 (pitch) → axis Y `(0,1,0)`
- **Joint origins:** Pure translations — no rpy needed for serial arm joints
- **Mesh rotations:** Only in `<visual><origin rpy="..."/>` to align STL coords with link frame
  - Parts modeled with Z up: `rpy="0 0 0"` (no rotation)
  - Parts extending along STL +X that should point up: `rpy="0 -1.5708 0"` (maps X→Z)
  - Parts extending along STL +X that should point down: `rpy="0 1.5708 0"` (maps X→−Z)

Always verify the kinematic chain by computing zero-config world positions and checking
they match the manufacturer's link lengths (d1, a2, d4, d6, etc.).

## URDF Template

```xml
<?xml version="1.0"?>
<robot name="ROBOT_NAME">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="stl_files/A0.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.054" upper="3.054" effort="100" velocity="1"/>
  </joint>

  <link name="link_1">
    <visual>
      <geometry>
        <mesh filename="stl_files/A1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <!-- Continue for all joints and links... -->
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
