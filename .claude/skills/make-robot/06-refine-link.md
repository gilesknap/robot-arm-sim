# 06 — Refine Link (Per-Link Visual Alignment)

The core iteration loop. Called once per link, working base-to-tip. Operates on exactly ONE link at a time.

## Input

- Link index N (0-based, from orchestrator)
- `robots/<name>/view_mapping.yaml` (from step 05)
- Simulator running on `localhost:8080`
- claude-in-chrome MCP for browser screenshots
- `zoom-rotate-camera` skill for JS camera templates

## Steps

### 0. Pre-Visual Gap/Overlap Check (BEFORE opening the simulator)

Before looking at any screenshots, **compute expected gaps analytically in all three axes** using the analysis YAMLs and chain.yaml. For each pair of adjacent links (N-1, N):

1. Read the analysis YAML for both links. Note the **bounding box** extents in X, Y, and Z, and the **proximal connection point** position.
2. For each axis, compute where link N-1's mesh boundary is in world coords:
   - The URDF generator aligns the proximal bore with the joint origin, so the mesh extends from `joint_origin + (mesh_min - proximal) / 1000` to `joint_origin + (mesh_max - proximal) / 1000` along each axis.
3. Compute where link N's mesh boundary is in world coords (same formula for the next joint).
4. Check for **gaps > 5mm** or **overlaps** along each axis.

**Common pattern**: When the joint origin height (e.g. DH d1) is significantly taller than the parent mesh, a Z gap will appear. But also check Y and X — lateral misalignment between meshes is common and only visible from FRONT/BACK (Y) or LEFT/RIGHT (X) views.

**Fix approach — visual_xyz ONLY**:
- Use `visual_xyz` to shift the visual mesh into position. This is purely cosmetic — it does NOT affect kinematics.
- **NEVER adjust joint `origin` to fix visual alignment.** Joint origins are defined by DH parameters and verified by `verify_kinematics.py`. Changing them would break the kinematic chain.
- Large `visual_xyz` values (e.g. 60mm) will cause minor COR drift — when the joint rotates, the mesh may shift slightly because the visual center doesn't coincide with the joint origin. **This is an acceptable cosmetic trade-off.** The alternative (moving the COR) would give wrong end-effector positions.

**Why COR drift is OK**: A 60mm `visual_xyz` with ±45° rotation produces ~2mm of visible misalignment at the joint. This is cosmetic only — the kinematic model remains correct, which matters more for simulation accuracy.

Apply any computed `visual_xyz` fixes to chain.yaml and regenerate BEFORE starting visual comparison. This avoids wasting iteration cycles on obvious geometric misalignment.

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

### 2. Two-View Alignment Check

For each link, check from **two orthogonal views**. Use this view-pair strategy:

1. **Primary view**: The view where the joint axis is visible (e.g. RIGHT/SIDE for Z-axis joints).
2. **Secondary view**: The orthogonal view (e.g. FRONT for Z-axis joints). This catches lateral misalignment invisible in the primary view.

**If the part is obscured** in one view (e.g. hidden behind a parent link from the front), switch to the **opposite** view (BACK instead of FRONT, or LEFT instead of RIGHT). The opposite view shows the same alignment axis but from the other side, which often reveals the part.

For each view:
1. **Snap to ortho view** using the one-liner template from `zoom-rotate-camera` skill
2. **Screenshot** the simulator via browser MCP
3. **Compare** against the reference image:
   - Does link N's shape match?
   - Is it positioned correctly relative to link N-1?
   - Are there gaps or overlaps at the joint?
   - Does the overall silhouette match up to this link?

### 3. Diagnose and Fix

If there's a discrepancy, use this symptom-to-fix table:

| Symptom | Visible in | Fix in chain.yaml |
|---------|-----------|-------------------|
| Mesh floating above joint | SIDE | Add negative `visual_xyz` Z |
| Gap between links (bore at mesh extreme) | SIDE | Negative `visual_xyz` Z on child link |
| Overlap between links vertically | SIDE | Positive `visual_xyz` Z on child link |
| Overlap visible from FRONT/BACK only | FRONT/BACK | Lateral (Y) misalignment — adjust `visual_xyz` Y |
| Overlap visible from LEFT/RIGHT only | LEFT/RIGHT | Depth (X) misalignment — adjust `visual_xyz` X |
| Part rotated wrong | any | Adjust `visual_rpy` on this link |
| Part shifted laterally | FRONT/BACK | Adjust `visual_xyz` Y |
| Minor COR drift on rotation | rotate joint | Expected with large `visual_xyz` — cosmetic only, do NOT adjust joint origin |

After each fix:
```bash
# Regenerate URDF
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
# Click "Reload URDF" in the simulator toolbar (preserves camera + slider state)
```

**IMPORTANT**: The "Reload URDF" button preserves camera position, "Show up to" slider, and joint angles via sessionStorage. Use it instead of full page reload — this saves significant time during iterative refinement. After clicking, wait ~5 seconds for the state to restore.

Then re-screenshot and compare again.

### 4. Rotation Test

After visual alignment, **test the joint rotation** to check for COR drift:
1. Set joint N to ±45° using the slider
2. Screenshot — the parts on both sides of the joint should remain approximately flush
3. Minor drift (1-3mm visible gap at ±45°) is **expected and acceptable** when `visual_xyz` is large — the mesh visual center doesn't coincide with the kinematic joint origin
4. **Do NOT adjust joint origins to fix COR drift** — this would break verified kinematics. The cosmetic drift is the correct trade-off.

### 5. Verify Kinematics

After visual alignment:
```bash
uv run python robots/<name>/verify_kinematics.py --json
```

Ensure this link's joint still passes within 2mm.

### 6. Multi-View Consistency (CRITICAL)

Check **every** fix from at least 2 orthogonal views. A fix that looks right from one view may be wrong from another.

**View selection rules**:
- Z alignment (gaps/overlap): check from SIDE (RIGHT or LEFT)
- Y alignment (lateral offset): check from FRONT or BACK
- X alignment (depth offset): check from RIGHT or LEFT
- **If a part is obscured** in one view, switch to the opposite face (FRONT↔BACK, LEFT↔RIGHT). The opposite view shows the same alignment axis from the other side.

**Common trap**: The SIDE view only shows Z alignment. Lateral (Y) misalignment is invisible from the side — you MUST check FRONT or BACK views to catch it. Always check FRONT/BACK for overlap/clipping between adjacent links, especially at the base-shoulder and shoulder-upperarm junctions.

## Gate

ALL of:
- Link length within 2mm of manufacturer spec
- Silhouette matches reference in 2+ orthogonal views
- No gaps or overlaps at joints from any view
- Joint rotation test shows no excessive drift (< 3mm at ±45°)
- `verify_kinematics.py --json` passes
- Joint origins unchanged from DH-derived values

## Escalation

If ANY of these occur, stop and invoke `review` for human discussion:
- COR drift exceeds 5mm at ±45° — may indicate the mesh bore detection was wrong
- 5 iterations without convergence — likely a deeper issue
- Kinematics verification fails after visual changes (should never happen with visual_xyz only)

## Key Rules

- **One change at a time** — adjust one value, regenerate, re-compare
- **visual_xyz is ADDITIVE** — `[0, 0, 0.005]` means 5mm up from auto-detected position
- **NEVER adjust joint origins for visual alignment** — joint origins come from DH params and are kinematically verified. Accept minor COR drift as a cosmetic trade-off.
- **Check ALL three axes** — Z from the side, Y from front/back, X from left/right
- **Use opposite views for obscured parts** — if FRONT is blocked, try BACK
- **Use Reload URDF button** — it preserves camera, sliders, and joint angles
- **Never edit robot.urdf directly** — always edit chain.yaml and regenerate
