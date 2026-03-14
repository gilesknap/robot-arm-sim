# Center on Axis Skill

Center robot meshes on their rotation axes by zeroing out cross-axis visual origin offsets while preserving on-axis positioning.

## When to Use

Use this skill when:
- The user says "center on axis", "align meshes", "center the parts", or similar
- Meshes appear visually offset from their rotation axis in the simulator
- After building a new robot or editing connection points, to clean up cross-axis wobble

## Concept

Each link rotates around a joint axis (X, Y, or Z). The URDF visual origin has three components — the on-axis component positions the mesh correctly along the kinematic chain, while the cross-axis components come from asymmetric connection point positions. Zeroing the cross-axis components centers the mesh on its rotation axis without affecting kinematics.

Example: if joint_3 has axis `[0, 1, 0]` (Y), and the URDF visual origin is `xyz="0.002 -0.006 -0.001"`, then X=0.002 and Z=-0.001 are cross-axis offsets. The nudge `[-0.002, 0, 0.001]` cancels them.

## Procedure

### Step 1: Identify links, axes, and current visual origins

Read `chain.yaml` to get the joint axes for each link. Read `robot.urdf` to get the current computed visual origins (these include connection-point snapping + gap closing).

For each joint, note:
- **child link name**
- **joint axis** — `[0,0,1]` = Z, `[0,1,0]` = Y, `[1,0,0]` = X
- **current URDF visual origin** `xyz` for that link

### Step 2: Calculate cross-axis nudges

For each link, the nudge cancels the cross-axis components of the visual origin:

```
axis = Z → nudge_x = -visual_x, nudge_y = -visual_y, nudge_z = 0
axis = Y → nudge_x = -visual_x, nudge_y = 0,          nudge_z = -visual_z
axis = X → nudge_x = 0,          nudge_y = -visual_y, nudge_z = -visual_z
```

**Important**: `visual_xyz` in `chain.yaml` is a *nudge added on top of* `compute_visual_origin`, not an absolute value. If a link already has a `visual_xyz`, subtract the existing nudge from the computed origin first, then recalculate.

### Step 3: Apply nudges to chain.yaml

Add or update `visual_xyz` entries for each link in `chain.yaml`. Format: `visual_xyz: [x, y, z]`.

Skip links that already have zero cross-axis offsets (nudge would be `[0, 0, 0]`).

### Step 4: Regenerate and verify

```bash
uv run python -m robot_arm_sim generate robots/<name>
```

Verify:
1. **FK distances unchanged** — compare joint distances before and after
2. **Joint origins unchanged** — diff URDF, confirm only `<origin>` inside `<visual>` tags changed
3. **Cross-axis components are zero** — each visual origin should have 0.000000 for the cross-axis dimensions

### Step 5: Diff against previous commit

Per CLAUDE.md, diff the URDF:
```bash
diff <(git show HEAD:robots/<name>/robot.urdf) robots/<name>/robot.urdf
```

Every changed line should be a visual origin with cross-axis components going to zero. Joint origins must NOT change.

## Scope Options

The user may request:
- **Single link**: "center the end effector" → apply to one link only
- **All links**: "center all parts" → apply to every link in the chain
- **Shoulder and arm parts**: typically means all links except base_link

Always confirm which links to include if ambiguous.

## Accuracy Warning

This skill trades geometric accuracy for visual consistency. It does **not** know where the true bore hole is in the mesh — it simply places the mesh's own coordinate origin (0,0) on the joint axis in the cross-axis plane.

This works well when:
- The STL was modeled with the bore center at (0,0) in the cross-axis plane (e.g., Meca500 meshes — offsets are <0.2mm)
- All parts in the robot share a consistent origin convention, so errors cancel between parent and child

This can be misleading when:
- The STL origin is offset from the bore (e.g., UR5 meshes have cross-axis offsets up to 6mm from the detected bore center)
- In such cases the skill eliminates wobble during rotation but shifts the bore off-axis by the same amount

**Why it still looks good even with inaccurate meshes:** Connection point detection finds slightly different bore centers at each end of each part (because cross-sections vary). This creates *inconsistent* small errors between parent and child — visible as wobble. Center-on-axis replaces these with a *consistent* error (the mesh origin convention), which looks smooth because adjacent parts share the same bias. Consistent error looks clean; inconsistent error looks wobbly.

**Bottom line:** The result looks visually smooth but is not geometrically precise. For robots where the STL origins are not on the bore axis, the connection-point-based positioning is actually more accurate — it just wobbles slightly because each end's bore center is detected independently.

## Key Principles

- **Never modify joint origins** — kinematics is sacred (per CLAUDE.md)
- **Only cancel cross-axis** — on-axis offsets maintain correct chain positioning
- **visual_xyz is a nudge** — it's added to the computed visual origin, not a replacement
- **Regenerate, don't hand-edit URDF** — always go through chain.yaml → generator
- **Check all robots** — per CLAUDE.md, diff URDFs for all robots to verify no unintended changes
