# Auto Build Robot Skill

Orchestrate the complete pipeline from raw STL files to a verified, visually accurate robot simulation. This is the single entry point for adding any new robot.

## When to Use

Use this skill when:
- The user wants to add a new robot from STL files
- The user says "build a robot", "add a robot", or "auto-build"
- A `robots/<name>/stl_files/` directory exists but no `chain.yaml` or `robot.urdf`

## Prerequisites

- STL mesh files in `robots/<name>/stl_files/`, named in kinematic order (A0, A1, ...), in millimetres with Z up
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
- Ensure files are named in order (A0, A1, A2, ... or similar sequential naming)

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
| Joint limits (radians) | J1: +/-175 deg | `limits` in chain.yaml joints |
| Zero-config pose description | "straight up" or "folded" | Informs assembly reasoning |
| Link lengths / total reach | 460mm height at zero | Cross-validation target |
| Number of axes / DOF | 6-axis | Must match STL part count |

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

- Parts named sequentially (A0, A1, A2, ...) form a serial chain base-to-tip
- Combined parts (e.g., A3_4) span multiple joints — create a virtual link with `mesh: null`
- Connection point axes tell you the joint rotation axis at each interface

### Joint axis conventions

Determine each joint's axis from the connection point geometry:
- Bore axis along Z at the interface → joint axis `[0, 0, 1]` (roll/yaw)
- Bore axis along Y at the interface → joint axis `[0, 1, 0]` (pitch)
- Bore axis along X at the interface → joint axis `[1, 0, 0]` (roll)

Cross-validate against manufacturer specs (e.g., typical 6-DOF pattern: Z-Y-Y-X-Y-X or Z-Y-Y-Z-Y-Z).

### Write chain.yaml

```yaml
robot_name: <Robot-Name>
dh_params:  # from manufacturer specs (mm)
  d1: ...
  a2: ...
  # etc.

links:
  - name: base_link
    mesh: A0
  - name: link_1
    mesh: A1
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
    # Optional: origin override if auto-detection is wrong
    # origin: [0, 0, 0.093]
  # ... one entry per joint
```

### chain.yaml field reference

| Field | Purpose | When to use |
|-------|---------|-------------|
| `links[].mesh` | STL file stem, or `null` for virtual links | Always |
| `links[].visual_xyz` | Additive offset on top of auto-computed position (metres) | When auto-detection places mesh wrong |
| `links[].visual_rpy` | Mesh rotation to align STL coords with link frame (radians) | When STL orientation doesn't match link frame |
| `joints[].axis` | Rotation axis: `[0,0,1]` for yaw/roll, `[0,1,0]` for pitch | Always |
| `joints[].limits` | Joint angle limits in radians | Always (from manufacturer specs) |
| `joints[].origin` | Override auto-computed joint position (metres) | Only when auto-detection is wrong |
| `dh_params` | Manufacturer DH parameters for validation | When available |

### Key rules

- **Never write raw URDF XML** — always write chain.yaml and let the generator handle it
- **visual_xyz is ADDITIVE** — `[0, 0, 0.005]` means "shift 5mm up from auto-detected position"
- **Start with origin overrides from DH params** — auto-detected connection points frequently have large Y offsets or wrong Z distances. Use manufacturer DH parameters to compute explicit `origin` values for every joint. This is usually necessary — don't wait for Stage 4 to fail.
- **How to compute origin values from DH params:**
  - Identify the base height (d1_base) from the A0 distal connection point Z
  - joint_1 origin: `[0, 0, d1_base]` (in metres)
  - joint_2 origin: `[0, 0, d1 - d1_base]` (remaining height to J2)
  - For subsequent joints, use DH `a` and `d` values as inter-joint distances along the appropriate axis
  - Cross-reference analysis YAML connection points to validate, but prefer DH-derived values
- **visual_xyz is usually needed** for parts whose proximal bore is offset from the STL origin. Read each part's `proximal` connection point Z from analysis YAML — if Z is significantly non-zero, add `visual_xyz: [0, 0, -Z_mm/1000]` to compensate.

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
1. Parses the generated URDF
2. Computes zero-config forward kinematics
3. Prints each joint's world position in mm
4. Compares against manufacturer DH parameters (if available)
5. Reports PASS/FAIL per joint with a 2mm tolerance

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

# Restart simulator (required — browser refresh alone won't pick up URDF changes)
pkill -9 -f "robot-arm-sim simulate" 2>/dev/null
for i in 1 2 3 4 5; do lsof -i :8080 >/dev/null 2>&1 || break; sleep 1; done
nohup uv run robot-arm-sim simulate robots/<name>/ > /tmp/sim.log 2>&1 &
# Wait for HTTP 200 before navigating browser
for i in 1 2 3 4 5 6 7 8; do sleep 1; curl -s -o /dev/null -w "%{http_code}" http://localhost:8080/ | grep -q 200 && break; done
```

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

- **Z always points up** at zero config — no DH-style frame rotations
- **Joint origins** are pure translations, no rpy
- **Mesh rotations** only via `visual_rpy` in chain.yaml
- **Never edit robot.urdf directly** — always edit chain.yaml and regenerate

## DH Parameters vs Bore Detection

DH parameters define link lengths in joint-local frames, NOT world XYZ. For joints with non-Z axes, DH `d` and `a` don't map directly to world Z/X.

**Correct approach:** Use bore-detected connection points as joint origins. Validate using **inter-joint distances** (frame-invariant) compared against DH `a` and `d` values.

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
