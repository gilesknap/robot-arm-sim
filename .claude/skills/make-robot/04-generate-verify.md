# 04 — Generate and Verify URDF

Generate the URDF from chain.yaml and verify kinematics match manufacturer specs.

## Input

- `robots/<name>/chain.yaml` (from step 03)
- `robots/<name>/specs.yaml` (from step 02)

## Steps

### 1. Generate URDF

```bash
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
```

This reads connection points from analysis data, computes transforms, and writes `robots/<name>/robot.urdf`.

### 2. Run Kinematics Verification

```bash
uv run python robots/<name>/verify_kinematics.py --json
```

**If verify_kinematics.py doesn't exist**, create one that:
1. Parses the generated URDF (handles both xyz and rpy in joint origins)
2. Computes zero-config forward kinematics with proper frame accumulation
3. Prints each joint's world position in mm
4. Compares against expected positions from DH parameters
5. Reports PASS/FAIL per joint with a 2mm tolerance
6. For UR-family: include an "arm up" test pose (J2=-π/2)
7. See `robots/Meca500-R3/verify_kinematics.py` or `robots/UR5/verify_kinematics.py` as templates

### 3. Interpret Results

- **All joints within 2mm**: PASS → proceed to step 05
- **Some joints off by 2-10mm**: likely `origin` or `visual_xyz` adjustment needed → fix chain.yaml and loop
- **Joints off by >10mm**: wrong chain topology, axis, or connection point → revisit step 03

### 4. Fix and Loop

For each failing joint:

| Problem | Fix in chain.yaml |
|---------|-------------------|
| Wrong link length | Check `origin` value against DH params |
| Mesh floating/disconnected | Add `visual_xyz` on the affected link |
| Wrong rotation direction | Flip axis sign or change axis |
| Combined part spanning wrong joints | Adjust virtual link placement |

After each fix:
```bash
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
uv run python robots/<name>/verify_kinematics.py --json
```

**Do not proceed to step 05 until kinematics pass.**

## Output

- `robots/<name>/robot.urdf`
- Passing kinematics verification

## Gate

All joints within 2mm of manufacturer specs in the kinematics verification.
