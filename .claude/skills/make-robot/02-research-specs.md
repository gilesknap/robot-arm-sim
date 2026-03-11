# 02 — Research Manufacturer Specifications

Search for the robot's official specifications to establish ground truth for the kinematic chain.

## Input

- Robot name/model (from the directory name)

## Steps

### 1. Web Search

Use `WebSearch` with these queries:
1. `"<robot name>" specifications DH parameters`
2. `"<robot name>" datasheet joint limits`
3. `"<robot name>" user manual PDF`
4. `"<robot name>" technical drawing dimensions mm`

### 2. Check Local Resources

```bash
# Look for PDFs or manuals already in the robot directory
find robots/<name>/ -type f \( -name "*.pdf" -o -name "*.png" -o -name "*.jpg" \) | head -10
```

Read any PDFs found — they often contain dimensioned drawings and DH tables.

### 3. Extract Key Parameters

| Parameter | Where to record |
|-----------|-----------------|
| DH parameters (d, a, alpha, theta_offset) | `dh_parameters` in specs.yaml |
| Joint limits (radians) | `joint_limits` in specs.yaml |
| Zero-config pose description | `zero_config` in specs.yaml |
| Physical offsets (shoulder, elbow) | `physical_offsets` in specs.yaml |
| Total reach | `total_reach_mm` in specs.yaml |
| DOF | `dof` in specs.yaml |

### 4. Save Reference Images

```bash
mkdir -p robots/<name>/images/
```

Download dimensioned technical drawings and product photos:
- `reference_side_dimensions.png` — dimensioned technical drawing (side view)
- `reference_front_dimensions.png` — dimensioned technical drawing (front view)
- `reference_photo_side.jpg` — real photo, side view
- `reference_photo_3quarter.jpg` — real photo, three-quarter view

### 5. Write specs.yaml

Write `robots/<name>/specs.yaml`:

```yaml
robot_name: <name>
manufacturer: <manufacturer>
dof: <number>
zero_config: "<description of zero-config pose>"
dh_parameters:
  - joint: 1
    d: <mm>
    a: <mm>
    alpha: <degrees>
    theta_offset: <degrees>
  # ... one entry per joint
physical_offsets:
  shoulder_offset: <mm>   # lateral offset J1→J2 (if applicable)
  elbow_offset: <mm>      # lateral offset J2→J3 (if applicable)
joint_limits:  # radians
  - joint: 1
    lower: <rad>
    upper: <rad>
  # ... one entry per joint
total_reach_mm: <mm>
references:
  - file: "images/<filename>"
    type: <dimensioned_drawing|product_photo|cad_render>
    view: <side|front|top|3quarter>
  # ... one entry per reference image
```

## Output

- `robots/<name>/specs.yaml` — structured manufacturer specifications
- `robots/<name>/images/` — reference images for visual comparison

## Gate

`specs.yaml` has:
- All DH parameters populated (d, a, alpha for each joint)
- Joint limits for every joint
- At least one reference image saved locally

## Notes on DH Alpha

Check the DH alpha (twist) values carefully:
- α = π/2 or -π/2 → the robot needs `origin_rpy` on some joints (common for UR-family)
- α = 0 for all joints → simpler case (e.g., Meca500)
- Record this distinction — it affects how chain.yaml origins are computed in step 03
