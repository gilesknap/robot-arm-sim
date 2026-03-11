# 06 — Refine Link (Per-Link Visual Alignment)

The core iteration loop. Called once per link, working base-to-tip. Operates on exactly ONE link at a time.

## Input

- Link index N (0-based, from orchestrator)
- `robots/<name>/view_mapping.yaml` (from step 05)
- Simulator running on `localhost:8080`
- claude-in-chrome MCP for browser screenshots
- `zoom-rotate-camera` skill for JS camera templates

## Steps

### 0. Pre-Visual Gap Check (BEFORE opening the simulator)

Before looking at any screenshots, **compute expected gaps analytically** using the analysis YAMLs and chain.yaml. For each pair of adjacent links (N-1, N):

1. Read the analysis YAML for both links. Note the **bounding box** Z extents and the **proximal connection point** Z position.
2. Compute where link N-1's mesh top is in world coords:
   - The URDF generator aligns the proximal bore with the joint origin, so the mesh extends from `joint_origin_Z + (mesh_Z_min - proximal_Z)` to `joint_origin_Z + (mesh_Z_max - proximal_Z)` (converted mm→m).
3. Compute where link N's mesh bottom is in world coords (same formula for the next joint).
4. If there is a **gap > 5mm** between the top of link N-1 and the bottom of link N, the shoulder/child mesh needs a negative `visual_xyz` Z to shift it down to close the gap.

**Common pattern**: When the joint origin height (e.g. DH d1) is significantly taller than the parent mesh, a gap will appear. The child mesh must be shifted down with `visual_xyz` to bridge it. This is expected when the bore connection is at the extreme bottom of a mesh — the auto-detected offset pushes the entire mesh above the joint, leaving empty space below.

**Fix formula**: `visual_xyz Z ≈ -(joint_origin_Z - parent_mesh_top_Z)` (in metres, negative to shift down).

Apply any computed `visual_xyz` fixes to chain.yaml and regenerate BEFORE starting visual comparison. This avoids wasting iteration cycles on obvious geometric gaps.

### 1. Set "Show up to" Slider

Use the slider JS to show links 0..N only:
```javascript
const tracks = document.querySelectorAll('.q-slider__track-container');
function clickSlider(idx, degrees, minDeg, maxDeg) {
    const t = tracks[idx];
    const r = t.getBoundingClientRect();
    const frac = (degrees - minDeg) / (maxDeg - minDeg);
    const x = r.left + frac * r.width;
    const y = r.top + r.height / 2;
    t.dispatchEvent(new MouseEvent('mousedown',
        {clientX: x, clientY: y, bubbles: true}));
    t.dispatchEvent(new MouseEvent('mouseup',
        {clientX: x, clientY: y, bubbles: true}));
}
```

The "Show up to" slider is the last slider in the control panel (highest index). Set it to show N+1 links (value = N).

### 2. Compare Each Mapped View

For each view in `view_mapping.yaml`:

1. **Snap to ortho view** using the one-liner template from `zoom-rotate-camera` skill, with the `dir` and `up` from the view mapping
2. **Screenshot** the simulator via browser MCP
3. **Compare** against the reference image:
   - Does link N's shape match?
   - Is it positioned correctly relative to link N-1?
   - Are there gaps or overlaps at the joint?
   - Does the overall silhouette match up to this link?

### 3. Diagnose and Fix

If there's a discrepancy, use this symptom-to-fix table:

| Symptom | Fix in chain.yaml |
|---------|-------------------|
| Mesh floating above joint | Add negative `visual_xyz` Z on this link |
| Gap between links N and N-1 (bore at mesh extreme) | Add negative `visual_xyz` Z on link N to bridge the gap (see step 0) |
| Gap between links N and N-1 (joint origin too large) | Reduce joint `origin` Z/distance |
| Overlap between links | Increase joint `origin` Z/distance |
| Part rotated wrong | Adjust `visual_rpy` on this link |
| COR offset from mesh center | Move `origin` on the joint, NOT `visual_xyz` |
| Part shifted laterally | Check `origin` Y component or `visual_xyz` X/Y |
| Overlap visible from FRONT/BACK but not SIDE | Lateral (Y) misalignment — adjust `visual_xyz` Y |

After each fix:
```bash
# Regenerate URDF
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
# Click "Reload URDF" in the simulator toolbar
```

Then re-screenshot and compare again.

### 4. Verify Kinematics

After visual alignment:
```bash
uv run python robots/<name>/verify_kinematics.py --json
```

Ensure this link's joint still passes within 2mm.

### 5. Multi-View Consistency (CRITICAL)

Check **every** fix from at least 2 orthogonal views (FRONT + RIGHT minimum). A fix that looks right from one view may be wrong from another.

**Common trap**: The SIDE view only shows Z alignment. Lateral (Y) misalignment is invisible from the side — you MUST check FRONT or BACK views to catch it. Always check FRONT/BACK for overlap/clipping between adjacent links, especially at the base-shoulder and shoulder-upperarm junctions.

## Gate

ALL of:
- Link length within 2mm of manufacturer spec
- Silhouette matches reference in 2+ views
- No gaps or overlaps at joints
- `verify_kinematics.py --json` passes
- `visual_xyz` magnitude < 40mm

## Escalation

If ANY of these occur, stop and invoke `review` for human discussion:
- `visual_xyz` exceeds 40mm on any axis — likely wrong connection point
- 5 iterations without convergence — likely a deeper issue
- Kinematics and visual alignment contradict each other

## Key Rules

- **One change at a time** — adjust one value, regenerate, re-compare
- **visual_xyz is ADDITIVE** — `[0, 0, 0.005]` means 5mm up from auto-detected position
- **Prefer origin over visual_xyz** — if the COR is wrong, fix the joint origin, not the visual offset
- **Never edit robot.urdf directly** — always edit chain.yaml and regenerate
