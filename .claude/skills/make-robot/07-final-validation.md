# 07 — Final Validation (DEPRECATED — better done by humans)

Multi-view, multi-pose acceptance test to confirm the robot model is complete and accurate.

## Prerequisites

- All links refined via step 06
- Simulator running on `localhost:8080`
- claude-in-chrome MCP for browser screenshots
- `robots/<name>/view_mapping.yaml` from step 05

## Steps

### 1. Screenshot All Orthographic Views

For each view in `view_mapping.yaml`:
1. Set "Show up to" slider to show ALL links
2. Snap to the ortho view using `zoom-rotate-camera` JS templates
3. Screenshot via browser MCP
4. Compare against the corresponding reference image

All views should match — no visible misalignment, gaps, or overlaps.

### 2. Test Non-Zero Joint Poses

Set joints to non-zero positions to verify rotation centres:

| Test | What it validates |
|------|------------------|
| J1 = 90° | Base rotation axis centering |
| J2 = 45° | Shoulder rotation center, upper arm length |
| J3 = -45° | Elbow rotation center, forearm alignment |
| J2 = 45°, J3 = -45° combined | Compound arm pose, overall chain accuracy |
| J5 = 45° | Wrist rotation center |

At each pose:
- Verify the joint rotates around the correct point (no orbit artifact)
- Parts on both sides of the joint should remain flush
- Compare overall pose silhouette against manufacturer range-of-motion diagrams if available

### 3. Run Final Kinematics Verification

```bash
uv run python robots/<name>/verify_kinematics.py --json
```

All joints must pass within 2mm tolerance.

### 4. Sign-Off Checklist

- [ ] All link lengths within 2mm of manufacturer specs
- [ ] All ortho views match manufacturer drawings
- [ ] Non-zero joint angles produce correct rotation (no orbiting)
- [ ] `verify_kinematics.py --json` passes all joints
- [ ] No visible gaps, overlaps, or misaligned parts
- [ ] `chain.yaml` values are reasonable (no `visual_xyz` > 40mm)
- [ ] At least 2 orthogonal views verified

## Gate

ALL checklist items pass. If any fail, identify which link(s) need rework and invoke step 06 for those specific links.

## Output

Report the final state:
- Which views were checked and their match quality
- Which poses were tested
- Kinematics verification results
- Any remaining minor issues or known limitations
