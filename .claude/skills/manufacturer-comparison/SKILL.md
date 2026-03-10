# Manufacturer Comparison Skill

Automate the workflow of finding manufacturer orthogonal drawings, setting up matching ortho views in the simulator, and iteratively tuning the URDF until the model matches the manufacturer's technical drawings.

## When to Use

Use this skill when:
- Starting URDF visual refinement for a new robot (before any manual tuning)
- The user wants to compare the simulator against official manufacturer dimensions
- You need dimensioned technical drawings to establish ground truth for link lengths and joint positions
- The user says "compare against datasheet", "match the manufacturer drawing", or similar

## Prerequisites

- `chain.yaml` and `robot.urdf` exist in the robot directory
- Browser automation tools (claude-in-chrome) available
- The `zoom-rotate-camera` skill (for camera JS templates and named view table)
- The `visual-urdf-tuning` skill (for diagnosis table and chain.yaml editing patterns)
- The `refine-with-image` skill (for simulator restart and slider control patterns)

---

## Phase 1: Find Manufacturer Reference Images

### Check local files first

```bash
# Look for PDFs, images, and datasheets already in the robot directory
find robots/<name>/ -type f \( -name "*.pdf" -o -name "*.png" -o -name "*.jpg" -o -name "*.jpeg" -o -name "*.svg" \) | head -20
```

Read any PDFs found — manufacturer manuals often contain dimensioned orthographic drawings.

### Web search for technical drawings

Use `WebSearch` with these queries (in priority order):

1. `"<robot model>" technical drawing dimensions mm` — dimensioned orthographic views
2. `"<robot model>" datasheet PDF filetype:pdf` — official datasheets
3. `"<robot model>" user manual PDF` — full manuals with mechanical drawings
4. `"<robot model>" CAD model STEP` — CAD files may have orthographic renders
5. `"<robot model>" dimensions specifications` — product pages with dimension diagrams

### Evaluate and prioritize references

| Priority | Source Type | Why |
|----------|-----------|-----|
| 1 (best) | Dimensioned technical drawing (orthographic) | Exact measurements, no perspective distortion |
| 2 | CAD orthogonal render | Correct geometry but may lack dimensions |
| 3 | Manufacturer product photo (straight-on) | Real appearance but has perspective |
| 4 | Third-party photo | Useful for validation only |

**Must-have**: At least one dimensioned drawing showing link lengths. Without dimensions, you can only check silhouettes, not absolute accuracy.

### Save reference data

Download or note URLs for:
- Front view drawing with dimensions
- Side view drawing with dimensions
- Top view if available
- Any DH parameter tables from the manual

Extract key dimensions into a reference table:

```
Link lengths (from manufacturer):
  d1 (base to J2):     ___mm
  a2 (J2 to J3):       ___mm
  a3 (J3 to J4):       ___mm
  d4 (J4 to J5):       ___mm
  d6 (J5 to flange):   ___mm
  Total height (zero config): ___mm
```

---

## Phase 2: Map Manufacturer Views to ViewCube

**Critical step** — manufacturer "front" often differs from our ViewCube FRONT (+X direction).

### Identify the manufacturer's coordinate convention

Look for:
- Axis arrows printed on the drawing (X/Y/Z labels)
- "Front view" / "Side view" labels
- Base mounting orientation (cable exits, logo placement, teach pendant connector)

### Create an explicit mapping table

For each manufacturer-labeled view, determine which ViewCube face it corresponds to:

```
Manufacturer View → ViewCube Face Mapping:
  Manufacturer "Front"  → ViewCube _______ (direction: ___)
  Manufacturer "Side"   → ViewCube _______ (direction: ___)
  Manufacturer "Top"    → ViewCube _______ (direction: ___)
```

### How to determine the mapping

1. **If manufacturer shows axis arrows**: Map directly. E.g., if manufacturer +X points toward viewer in their "front view", and our ViewCube FRONT is +X, they match.

2. **If no axis arrows**: Use physical landmarks:
   - Cable exit location (usually rear/back of base)
   - Teach pendant port location
   - Mecademic logo position (for Meca500)
   - Mounting bolt pattern orientation

3. **Verify**: The mapping must be consistent across all views. If manufacturer front maps to our RIGHT, then manufacturer left must map to our FRONT (90° rotation).

### ViewCube named views reference

See the `zoom-rotate-camera` skill for the complete named view table. Key views:

| ViewCube Face | Direction (camera from) | Camera Up |
|--------------|------------------------|-----------|
| FRONT | `(1, 0, 0)` — from +X | `(0,0,1)` |
| BACK | `(-1, 0, 0)` — from -X | `(0,0,1)` |
| RIGHT | `(0, -1, 0)` — from -Y | `(0,0,1)` |
| LEFT | `(0, 1, 0)` — from +Y | `(0,0,1)` |
| TOP | `(0, 0, 1)` — from +Z | `(0,-1,0)` |

---

## Phase 3: Set Up Orthographic Comparison Views

**Rule: Always use orthographic projection for comparison with technical drawings.** Technical drawings are orthographic — comparing them against a perspective view introduces parallax errors that look like misalignment.

### Switch to ortho and snap to named view

Use the **Ortho Named View One-liner Template** from the `zoom-rotate-camera` skill. Set the `dir` and `up` arrays to match the ViewCube face determined in Phase 2.

Example: If manufacturer "front" maps to our ViewCube RIGHT:
```javascript
const dir = [0, -1, 0];     // RIGHT face direction
const up = [0, 0, 1];       // camera up for side views
```

See `zoom-rotate-camera` skill for the complete JS template — do not duplicate it here.

### Match the joint pose

Set joints to match the manufacturer drawing's pose. Most technical drawings show zero config (all joints at 0°). Use the slider JavaScript from the `refine-with-image` skill.

If the manufacturer drawing shows a non-zero pose, set each joint to match before comparing.

### Fit to frame

After setting the view, click the "Fit" button in the ViewCube control bar (or use the Fit JS from `view_controls.py`) to ensure the robot fills the viewport, matching the scale of the manufacturer drawing as closely as possible.

---

## Phase 4: Part-by-Part Comparison (Base to Tip)

### Systematic comparison procedure

For each joint, working from base to tip:

1. **Screenshot** the ortho view of the simulator at zero config
2. **Overlay mentally** against the manufacturer drawing
3. **Check these aspects in order:**

| Check | What to look for | How to measure |
|-------|-----------------|----------------|
| Link length | Distance between joint centers | Compare against manufacturer dimensions |
| Silhouette shape | Overall outline of the link | Should match drawing contour |
| Joint position | Where the rotation center is | Compare bore circle position to drawing |
| Part width/depth | Cross-section proportions | Compare against drawing dimensions |
| Gaps/overlaps | Space or interference at joints | Should be flush, matching drawing |

4. **Record each discrepancy** with:
   - Which link/joint is affected
   - Direction of error (too long, shifted left, rotated, etc.)
   - Estimated magnitude (in mm, by comparing proportionally against known dimensions)

### Use multiple views

A single view can hide errors along the viewing axis. Always check at least two orthogonal views (e.g., front + side) before concluding a joint is correct.

### Enable labels for identification

Click "Show Labels" in the simulator to see part names (blue, left) and joint names (red, right). This helps match simulator parts to manufacturer drawing callouts.

---

## Phase 5: Adjust and Iterate

### Edit chain.yaml

For each discrepancy found in Phase 4, apply the appropriate fix. See the diagnosis table in the `visual-urdf-tuning` skill for the mapping from symptoms to chain.yaml changes.

Key reminders:
- **`visual_xyz` is ADDITIVE** to the auto-computed offset — `[0, 0, 0.005]` means "shift 5mm up from auto-detected position"
- **Work base to tip** — fix base_link/link_1 first, since errors compound along the chain
- **One change at a time** — adjust one value, regenerate, restart, re-compare

### Regenerate and restart

```bash
# Regenerate URDF
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml

# Restart simulator (see refine-with-image skill for full restart procedure)
pkill -9 -f "robot-arm-sim simulate" 2>/dev/null
# Wait for port to free, then relaunch — see refine-with-image skill
```

### Verify kinematics after each change

```bash
uv run python robots/<name>/verify_kinematics.py --json
```

All joints must pass within tolerance.

### Convergence criteria

A joint/link is "done" when ALL of the following are met:

- [ ] Link length matches manufacturer dimension within **2mm** (0.002m)
- [ ] Silhouette matches manufacturer drawing contour (no visible shape mismatch)
- [ ] No spurious gaps or overlaps at joints (flush connection)
- [ ] `verify_kinematics.py` passes for all joints
- [ ] Consistent across at least two orthogonal views

### Escalation

If a joint does not converge after **5 adjustment rounds**:

1. Re-examine the analysis YAML (`robots/<name>/analysis/<part>.yaml`) for incorrect connection point detection
2. Check if `visual_xyz` exceeds 40mm — this usually means the wrong connection point was selected
3. Consider whether the mesh itself has geometry issues (e.g., missing features, wrong scale)
4. Ask the user for guidance, showing the current state and what you've tried

---

## Phase 6: Multi-View Validation

Once all joints pass convergence criteria from a single view:

### Screenshot all orthographic views

Take screenshots from each available ortho view that has a corresponding manufacturer drawing:

1. Front (or whichever maps to manufacturer front)
2. Side (or whichever maps to manufacturer side)
3. Top (if manufacturer top view is available)

Compare each against the corresponding manufacturer drawing.

### Test at non-zero joint angles

Move joints to non-zero positions to verify rotation centers are correct:

| Test | What it validates |
|------|------------------|
| J1 = 90° | Base rotation axis centering |
| J2 = 45° | Shoulder rotation center, upper arm length |
| J3 = -45° | Elbow rotation center, forearm alignment |
| J2 = 45°, J3 = -45° combined | Compound arm pose, overall chain accuracy |
| J5 = 45° | Wrist rotation center |

At each pose:
- Verify the joint rotates around the correct point (no orbit artifact)
- Check that parts on both sides of the joint remain flush
- Compare overall pose silhouette against manufacturer range-of-motion diagrams if available

### Final sign-off checklist

- [ ] All link lengths within 2mm of manufacturer specs
- [ ] All ortho views match manufacturer drawings
- [ ] Non-zero joint angles produce correct rotation (no orbiting)
- [ ] `verify_kinematics.py --json` passes all joints
- [ ] No visible gaps, overlaps, or misaligned parts
- [ ] `chain.yaml` changes are documented (which visual_xyz/rpy values were added and why)

---

## Key Design Principles

- **Ortho-only comparison**: Never compare a perspective simulator view against an orthographic technical drawing. Always switch to ortho mode first.
- **References not duplicates**: This skill cross-references `zoom-rotate-camera` for JS camera templates, `visual-urdf-tuning` for diagnosis tables and chain.yaml patterns, and `refine-with-image` for simulator restart and slider control procedures. Do not duplicate those instructions here.
- **Explicit view mapping**: Always establish the manufacturer-to-ViewCube mapping before comparing. Skipping this step leads to comparing the wrong views and chasing phantom errors.
- **Measurable convergence**: Use the manufacturer's own dimensions (DH parameters, link lengths) as ground truth, with a 2mm tolerance. Don't rely solely on "looks right."
- **Base to tip**: Always work from the base upward. A 2mm error at the base becomes a 5mm error at the wrist due to compounding.
