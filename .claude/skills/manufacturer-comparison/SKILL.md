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
- The `auto-build-robot` skill (for diagnosis table, chain.yaml editing patterns, and simulator restart procedures)

---

## Phase 1: Find Manufacturer Reference Images

### Check local files first

```bash
# Check for previously saved reference images
ls robots/<name>/images/reference_*.png 2>/dev/null

# Look for PDFs, images, and datasheets already in the robot directory
find robots/<name>/ -type f \( -name "*.pdf" -o -name "*.png" -o -name "*.jpg" -o -name "*.jpeg" -o -name "*.svg" \) | head -20
```

**If `robots/<name>/images/reference_*.png` files exist**, these were saved from a previous session — use them directly as ground truth. Skip web search and PDF extraction.

Read any PDFs found — manufacturer manuals often contain dimensioned orthographic drawings.

### Web search for reference material

Use `WebSearch` with these queries (in priority order):

1. `"<robot model>" technical drawing dimensions mm` — dimensioned orthographic views
2. `"<robot model>" zero position product photo` — actual photos for shape comparison
3. `"<robot model>" datasheet PDF filetype:pdf` — official datasheets
4. `"<robot model>" user manual PDF` — full manuals with mechanical drawings
5. `"<robot model>" CAD model STEP` — CAD files may have orthographic renders
6. `"<robot model>" dimensions specifications` — product pages with dimension diagrams

**Real photos are as valuable as technical drawings** — they show actual part shapes, gaps, proportions, and surface details that line drawings omit. Collect both.

### Evaluate and prioritize references

| Priority | Source Type | Why |
|----------|-----------|-----|
| 1 (best) | Dimensioned technical drawing (orthographic) | Exact measurements, no perspective distortion |
| 2 | CAD orthogonal render | Correct geometry but may lack dimensions |
| 3 | Manufacturer product photo (straight-on) | Real appearance but has perspective |
| 4 | Third-party photo | Useful for validation only |

**Must-have**: At least one dimensioned drawing showing link lengths. Without dimensions, you can only check silhouettes, not absolute accuracy.

### Save reference images to the robot directory

**Always save reference images locally** so they persist across sessions and can be viewed directly without re-extracting from PDFs or re-downloading.

```bash
# Create images directory
mkdir -p robots/<name>/images/

# Extract dimensioned drawings from manufacturer PDF
pdftoppm -png -r 150 -f <page> -l <page> \
  robots/<name>/<manual>.pdf /tmp/ref 2>/dev/null
cp /tmp/ref-<page>.png robots/<name>/images/reference_side_dimensions.png

# Download product photos from web (use curl)
curl -sL -o robots/<name>/images/reference_photo_side.jpg "<url>"
```

Use descriptive filenames that encode the source type and view:
- `reference_side_dimensions.png` — dimensioned technical drawing (side view)
- `reference_front_dimensions.png` — dimensioned technical drawing (front view)
- `reference_side_cad.png` — CAD render at zero config
- `reference_photo_side.jpg` — real photo, side view
- `reference_photo_3quarter.jpg` — real photo, three-quarter view
- `reference_photo_front.jpg` — real photo, front view

**Collect both technical drawings AND photos.** Drawings give exact measurements; photos show actual part shapes, gaps, and proportions that drawings may simplify or omit. Both are needed for thorough comparison.

These saved images serve as ground truth for all future comparison sessions.

### Extract key dimensions

From the saved reference images, note the key link lengths:

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
   - Manufacturer logo/branding position (if identifiable)
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

Set joints to match the manufacturer drawing's pose. Most technical drawings show zero config (all joints at 0°). Use the slider JavaScript from the `auto-build-robot` skill (Camera Control Reference section).

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

For each discrepancy found in Phase 4, apply the appropriate fix. See the diagnosis table in the `auto-build-robot` skill (Stage 5c) for the mapping from symptoms to chain.yaml changes.

Key reminders:
- **`visual_xyz` is ADDITIVE** to the auto-computed offset — `[0, 0, 0.005]` means "shift 5mm up from auto-detected position"
- **Work base to tip** — fix base_link/link_1 first, since errors compound along the chain
- **One change at a time** — adjust one value, regenerate, restart, re-compare

### Regenerate and restart

```bash
# Regenerate URDF
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml

# Restart simulator (see auto-build-robot skill for full restart procedure)
pkill -9 -f "robot-arm-sim simulate" 2>/dev/null
# Wait for port to free, then relaunch — see auto-build-robot skill
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

## Common Pitfall: DH Parameters vs Bore Detection

**DH parameters do NOT map directly to world XYZ positions.** DH convention defines link lengths and offsets in each joint's local frame. For joints whose axis is not world-Z (e.g., J4 with axis along X), the DH `d` and `a` parameters produce offsets in the joint-local frame, not world Z/X.

**Wrong approach:** Convert DH params to expected world positions by assuming d→Z and a→X. This fails for any robot where joint axes change orientation (which is most robots).

**Correct approach:**
1. Use bore-detected connection points as joint origins — these are actual geometric positions measured from the STL meshes
2. Validate using **inter-joint distances** (which are frame-invariant) rather than absolute world positions
3. Compare distances against DH `a` and `d` values, which represent link lengths regardless of frame orientation

The `urdf_generator.py` pipeline now warns if inter-joint distances deviate from DH params by >5mm.

---

## Key Design Principles

- **Ortho-only comparison**: Never compare a perspective simulator view against an orthographic technical drawing. Always switch to ortho mode first.
- **References not duplicates**: This skill cross-references `zoom-rotate-camera` for JS camera templates, and `auto-build-robot` for diagnosis tables, chain.yaml patterns, simulator restart, and slider control procedures. Do not duplicate those instructions here.
- **Explicit view mapping**: Always establish the manufacturer-to-ViewCube mapping before comparing. Skipping this step leads to comparing the wrong views and chasing phantom errors.
- **Measurable convergence**: Use the manufacturer's own dimensions (DH parameters, link lengths) as ground truth, with a 2mm tolerance. Don't rely solely on "looks right."
- **Base to tip**: Always work from the base upward. A 2mm error at the base becomes a 5mm error at the wrist due to compounding.
