# Auto Build Robot Skill

Orchestrate the complete pipeline from raw STL files to a verified, visually accurate robot simulation. This is the single entry point for adding any new robot.

## When to Use

Use this skill when:
- The user wants to add a new robot from STL files
- The user says "build a robot", "add a robot", or "auto-build"
- A `robots/<name>/stl_files/` directory exists but no `chain.yaml` or `robot.urdf`

## Prerequisites

- STL mesh files in `robots/<name>/stl_files/`, in millimetres with Z up
- Files can use any naming convention: sequential (A0, A1, ...) or semantic (base, shoulder, upperarm, ...)
- A working devcontainer with `uv` available
- Optionally: claude-in-chrome MCP server for visual comparison (Stage 5)

## Pipeline Overview

```
Stage 1: Analyze STLs → connection points, geometry
Stage 2: Research manufacturer specs → DH params, joint limits
Stage 3: Infer kinematic chain → chain.yaml
Stage 4: Generate & verify URDF → robot.urdf + kinematics check
Stage 5: Visual comparison → iterate until simulation matches reality
```

Each stage produces artifacts that feed the next. If a stage fails, fix it before proceeding.

---

## Stage 1: Analyze STL Files

Run the analysis command to extract geometry, connection points, and assembly hints:

```bash
uv run robot-arm-sim analyze robots/<name>/
```

**Verify output:**
- `robots/<name>/analysis/summary.yaml` exists with part list and assembly hints
- Each `robots/<name>/analysis/<part>.yaml` has `connection_points` entries
- Connection points show bore/shaft centres with axis directions

**If analysis fails or connection points are missing:**
- Check STL files are valid (not empty, in mm, Z-up)
- Files can use any naming: sequential (A0, A1, ...) or semantic (base, shoulder, ...)
- The `mesh` field in chain.yaml must match the STL file stem (e.g., `mesh: shoulder` → `stl_files/shoulder.stl`)

---

## Stage 2: Research Manufacturer Specifications

Search for the robot's official specifications before writing any chain.yaml.

### What to search for

Use `WebSearch` with these queries:
1. `"<robot name>" specifications DH parameters`
2. `"<robot name>" datasheet joint limits`
3. `"<robot name>" user manual PDF`
4. `"<robot name>" technical drawing dimensions mm`

### What to extract

| Parameter | Example | Where to record |
|-----------|---------|-----------------|
| DH parameters (d, a, alpha) | d1=135mm, a2=135mm | `dh_params` in chain.yaml |
| DH alpha values | α₁=π/2, α₂=0 | Determines if `origin_rpy` needed |
| Joint limits (radians) | J1: +/-175 deg | `limits` in chain.yaml joints |
| Zero-config pose description | "straight up" or "arm horizontal" | Informs assembly reasoning |
| Link lengths / total reach | 460mm height at zero | Cross-validation target |
| Number of axes / DOF | 6-axis | Must match STL part count |
| Physical offsets (shoulder, elbow) | shoulder_offset=135.85mm | `dh_params` + `origin` values |

### DH alpha and origin_rpy

**CRITICAL**: Check the DH alpha (twist) values. If any alpha ≠ 0, the robot needs `origin_rpy` on some joints to align link frames correctly. This is common for UR-family robots and most industrial 6-DOF arms.

- α = π/2 or -π/2 → add `origin_rpy: [0, 1.5707963, 0]` (or appropriate rotation) to the corresponding joint
- α = 0 → no origin_rpy needed
- The generator and verify_kinematics both handle origin_rpy correctly

**Also check for physical offsets** not in the standard 4-parameter DH table:
- `shoulder_offset`: lateral distance between J1 and J2 axes (common in UR robots)
- `elbow_offset`: lateral distance between J2 and J3 axes
- These appear as Y components in joint origins

### Check for local resources

```bash
# Look for PDFs or manuals already in the robot directory
find robots/<name>/ -type f \( -name "*.pdf" -o -name "*.png" -o -name "*.jpg" \) | head -10
```

Read any PDFs found — they often contain dimensioned drawings and DH tables.

### Record findings

Note the key specs in comments within chain.yaml's `dh_params` section. These serve as ground truth for verification.

---

## Stage 3: Infer Kinematic Chain

Using the analysis data (Stage 1) and manufacturer specs (Stage 2), write `chain.yaml`.

### Read analysis data

1. Read `robots/<name>/analysis/summary.yaml` for part ordering and assembly hints
2. Read each `robots/<name>/analysis/<part>.yaml` for connection points, bore positions, and axes

### Determine the chain topology

- Parts named sequentially (A0, A1, A2, ...) or semantically (base, shoulder, ...) form a serial chain base-to-tip
- For semantic names, determine order from: part geometry (base is flat/short), bounding box sizes (arm links are long), and manufacturer documentation
- Combined parts (e.g., A3_4) span multiple joints — create a virtual link with `mesh: null`
- Connection point axes tell you the joint rotation axis at each interface
- **Caution with collision meshes**: bore detection may find lateral motor housings instead of the main kinematic bore. Always cross-validate bore positions against DH parameters

### Joint axis conventions

Determine each joint's axis from the connection point geometry:
- Bore axis along Z at the interface → joint axis `[0, 0, 1]` (roll/yaw)
- Bore axis along Y at the interface → joint axis `[0, 1, 0]` (pitch)
- Bore axis along X at the interface → joint axis `[1, 0, 0]` (roll)

Cross-validate against manufacturer specs (e.g., typical 6-DOF pattern: Z-Y-Y-X-Y-X or Z-Y-Y-Z-Y-Z or Z-Y-Y-Y-Z-Y for UR-family).

### Write chain.yaml

```yaml
robot_name: <Robot-Name>
dh_params:  # from manufacturer specs (mm)
  d1: ...
  a2: ...
  # Also record physical offsets if they exist:
  # shoulder_offset: 135.85  # Y offset J1→J2
  # elbow_offset: -119.7     # Y offset J2→J3

links:
  - name: base_link
    mesh: base        # matches stl_files/base.stl
  - name: link_1
    mesh: shoulder    # or A1 — must match actual filename stem
  # ... one entry per link
  # For virtual links (no mesh):
  # - name: link_5
  #   mesh: null

joints:
  - name: joint_1
    type: revolute
    parent: base_link
    child: link_1
    axis: [0, 0, 1]
    limits: [-3.054, 3.054]  # from manufacturer specs, in radians
    origin: [0, 0, 0.093]    # ALWAYS use explicit origin from DH params
    # For joints with DH alpha ≠ 0:
    # origin_rpy: [0, 1.5707963, 0]  # aligns link frame with arm direction
  # ... one entry per joint
```

### chain.yaml field reference

| Field | Purpose | When to use |
|-------|---------|-------------|
| `links[].mesh` | STL file stem (e.g., `shoulder`), or `null` for virtual links | Always |
| `links[].visual_xyz` | Additive offset on top of auto-computed position (metres) | When auto-detection places mesh wrong |
| `links[].visual_rpy` | Mesh rotation to align STL coords with link frame (radians) | When STL orientation doesn't match link frame |
| `joints[].axis` | Rotation axis: `[0,0,1]` for yaw/roll, `[0,1,0]` for pitch | Always |
| `joints[].limits` | Joint angle limits in radians | Always (from manufacturer specs) |
| `joints[].origin` | Override auto-computed joint position (metres) | **Always** — auto-detection is unreliable for collision meshes |
| `joints[].origin_rpy` | Frame rotation at joint (radians, [roll, pitch, yaw]) | When DH alpha ≠ 0 (see below) |
| `dh_params` | Manufacturer DH parameters for validation | When available |

### Key rules

- **Never write raw URDF XML** — always write chain.yaml and let the generator handle it
- **visual_xyz is ADDITIVE** — `[0, 0, 0.005]` means "shift 5mm up from auto-detected position"
- **ALWAYS use explicit origin values** — auto-detected connection points are unreliable, especially for collision meshes where bore detection finds lateral motor housings instead of kinematic bores. Use manufacturer DH parameters to compute explicit `origin` values for every joint.
- **visual_xyz is usually needed** for parts whose proximal bore is offset from the STL origin. Read each part's `proximal` connection point Z from analysis YAML — if Z is significantly non-zero, add `visual_xyz: [0, 0, -Z_mm/1000]` to compensate.

### How to compute origin values

There are two robot families with different approaches:

#### Simple robots (all DH alpha = 0, e.g., Meca500)
All joints stack vertically at zero config:
- joint_1 origin: `[0, 0, d1_base]` (base height)
- joint_2 origin: `[0, 0, d1 - d1_base]` (remaining height)
- Subsequent joints: DH `a` and `d` values as Z distances
- No `origin_rpy` needed

#### Robots with DH alpha rotations (e.g., UR5, most industrial 6-DOF)
These need `origin_rpy` and may have lateral offsets:
1. **Identify which joints have α ≠ 0** in the DH table
2. **For each α = ±π/2**: add `origin_rpy: [0, 1.5707963, 0]` (or appropriate rotation) to that joint
3. **Origin values follow the official URDF convention** — DH `a` and `d` values go into different origin components depending on the accumulated frame rotation:

**UR-family example (Z-Y-Y-Y-Z-Y axes):**
```yaml
# J1: origin [0, 0, d1]                              # height from base
# J2: origin [0, shoulder_offset, 0]                  # lateral offset
#     origin_rpy: [0, pi/2, 0]                        # frame rotation
# J3: origin [0, elbow_offset, a2]                    # in rotated frame
# J4: origin [0, 0, a3]                               # in rotated frame
#     origin_rpy: [0, pi/2, 0]                        # second frame rotation
# J5: origin [0, d4, 0]                               # lateral in double-rotated frame
# J6: origin [0, 0, d5]                               # axial in double-rotated frame
```

4. **Verify with FK trace**: compute world-frame positions by applying accumulated rotations to each origin. The inter-joint distances should match DH `a` and `d` values (use Euclidean distance for joints with lateral offsets).
5. **Record shoulder_offset and elbow_offset** in `dh_params` section — these are NOT in the standard DH table but are essential for correct URDF origins.

---

## Stage 4: Generate and Verify URDF

### Generate

```bash
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
```

This reads connection points from analysis data, computes transforms, and writes `robots/<name>/robot.urdf`.

### Verify kinematics

```bash
uv run python robots/<name>/verify_kinematics.py --json
```

**If verify_kinematics.py doesn't exist**, create one that:
1. Parses the generated URDF (handles both xyz and rpy in joint origins)
2. Computes zero-config forward kinematics with proper frame accumulation
3. Prints each joint's world position in mm
4. Compares against expected positions computed from DH parameters
5. Reports PASS/FAIL per joint with a 2mm tolerance
6. Include an "arm up" test pose for UR-family robots (J2=-π/2) to verify vertical configuration
7. See `robots/Meca500-R3/verify_kinematics.py` or `robots/UR5/verify_kinematics.py` as templates

### Interpret results

- All joints within 2mm: **PASS** — proceed to Stage 5
- Some joints off by 2-10mm: likely `visual_xyz` or `origin` adjustment needed in chain.yaml → fix and regenerate
- Joints off by >10mm: likely wrong chain topology, axis, or connection point → revisit Stage 3

### Common fixes at this stage

| Problem | Fix |
|---------|-----|
| Wrong link length | Check `origin` override or DH params |
| Mesh floating/disconnected | Add `visual_xyz` on the affected link |
| Wrong rotation direction | Flip axis sign or change axis |
| Combined part spanning wrong joints | Adjust virtual link placement |

After fixing, regenerate and re-verify. Do not proceed to Stage 5 until kinematics pass.

---

## Stage 5: Visual Comparison (Optional but Recommended)

Compare the simulation against manufacturer reference images to catch visual issues that kinematics alone won't reveal (mesh alignment, gaps, overlaps, rotation centres).

### 5a: Find reference images

Search for and save reference images locally:

```bash
mkdir -p robots/<name>/images/
```

Use `WebSearch` for:
1. `"<robot name>" technical drawing dimensions mm` — dimensioned orthographic views
2. `"<robot name>" zero position product photo` — actual photos
3. `"<robot name>" datasheet PDF` — official datasheets with drawings

Save images to `robots/<name>/images/` with descriptive names:
- `reference_side_dimensions.png` — dimensioned technical drawing
- `reference_photo_side.jpg` — real photo

**If reference images already exist** in `robots/<name>/images/`, use them directly.

### 5b: Launch simulator and compare

```bash
nohup uv run robot-arm-sim simulate robots/<name>/ > /tmp/sim.log 2>&1 &
```

Wait for startup, then use claude-in-chrome to:
1. Open `http://localhost:8080` in a browser tab
2. Switch to orthographic projection (click "Ortho" button)
3. Snap to a named view matching the reference image (use ViewCube or JS camera control)
4. Screenshot and compare part-by-part from base to tip

### 5c: Diagnose and fix

| Symptom | Likely cause | Fix in chain.yaml |
|---------|-------------|-------------------|
| Mesh floating/disconnected | Wrong visual origin | Add `visual_xyz` on the link |
| Part too high after parent lowered | Parent visual_xyz moved mesh but child joint didn't follow | Adjust child joint `origin` Z |
| COR offset from mesh centre | Used visual_xyz but COR stayed at joint origin | Move joint `origin` instead |
| Part rotated wrong at zero config | Wrong `visual_rpy` | Adjust `visual_rpy` |
| Gap between parts | Joint origin too long | Reduce `origin` distance |
| Parts overlapping | Joint origin too short | Increase `origin` distance |

After each fix:
```bash
# Regenerate URDF
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml

# Click "Reload URDF" button in the simulator toolbar — this re-reads the
# URDF and STL meshes from disk without restarting the server.
# Ctrl+Shift+R may still be needed to bust browser STL cache after
# regenerating mesh files.
```

**Tip:** Use the "Show up to" slider in the right panel to progressively reveal
parts from base to tip. This makes it easy to inspect each part's placement
individually during the fix-verify cycle.

### 5d: Convergence criteria

A link is "done" when ALL of:
- Link length matches manufacturer dimension within **2mm**
- Silhouette matches reference contour
- No spurious gaps or overlaps at joints
- `verify_kinematics.py --json` passes
- Consistent across at least two orthogonal views

### 5e: Escalation

If a joint doesn't converge after 5 rounds:
1. Re-examine analysis YAML for incorrect connection point detection
2. Check if `visual_xyz` exceeds 40mm — likely wrong connection point
3. Ask the user for guidance

---

## Camera Control Reference

### Orthographic named views (via JavaScript)

Use the `zoom-rotate-camera` skill for the complete JS template. Key views:

| ViewCube Face | Direction (camera from) | Camera Up |
|--------------|------------------------|-----------|
| FRONT | `(1, 0, 0)` — from +X | `(0,0,1)` |
| RIGHT | `(0, -1, 0)` — from -Y | `(0,0,1)` |
| TOP | `(0, 0, 1)` — from +Z | `(0,-1,0)` |

### Slider control (NiceGUI Quasar sliders)

```javascript
const tracks = document.querySelectorAll('.q-slider__track-container');
function clickSlider(idx, degrees, minDeg, maxDeg) {
    const t = tracks[idx];
    const r = t.getBoundingClientRect();
    const frac = (degrees - minDeg) / (maxDeg - minDeg);
    const x = r.left + frac * r.width;
    const y = r.top + r.height / 2;
    t.dispatchEvent(new MouseEvent('mousedown', {clientX: x, clientY: y, bubbles: true}));
    t.dispatchEvent(new MouseEvent('mouseup', {clientX: x, clientY: y, bubbles: true}));
}
```

---

## URDF Frame Conventions

- **World Z points up** — gravity is -Z
- **Joint origins** are translations, optionally with `origin_rpy` for DH frame alignment
- **Simple robots** (Meca500-like): pure translations, no rpy, arm stacks vertically at zero config
- **Complex robots** (UR-like): `origin_rpy` needed at joints with DH α ≠ 0; zero config may be arm-horizontal
- **Mesh rotations** via `visual_rpy` in chain.yaml when STL orientation doesn't match link frame
- **Never edit robot.urdf directly** — always edit chain.yaml and regenerate

## DH Parameters vs Bore Detection

DH parameters define link lengths in joint-local frames, NOT world XYZ. For joints with non-Z axes, DH `d` and `a` don't map directly to world Z/X.

**Correct approach:**
1. Use manufacturer DH parameters + physical offsets for explicit `origin` values
2. Bore-detected connection points are useful for **validation** but unreliable as primary data, especially for collision meshes
3. Validate using **inter-joint distances** (frame-invariant) compared against DH `a` and `d` values
4. Collision meshes often have lateral motor bores that get detected instead of the main kinematic bore — always cross-check

---

## Complete Example Workflow

```bash
# Stage 1
uv run robot-arm-sim analyze robots/MyRobot/

# Stage 2 — research specs (done interactively with WebSearch)

# Stage 3 — write chain.yaml (done interactively)

# Stage 4
uv run robot-arm-sim generate robots/MyRobot/ robots/MyRobot/chain.yaml
uv run python robots/MyRobot/verify_kinematics.py --json

# Stage 5 (if visual comparison needed)
uv run robot-arm-sim simulate robots/MyRobot/
# ... visual comparison and iteration ...

# Final check
uv run python robots/MyRobot/verify_kinematics.py --json
```
