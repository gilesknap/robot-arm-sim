# Debugging with the Chrome Connector

This page walks through a real Claude Code session that diagnosed and fixed
a missing end-effector on the Meca500-R3 simulator. It demonstrates the
browser automation tools (Chrome connector), the `zoom-rotate-camera` skill,
and iterative URDF tuning — all driven from a single conversation.

```{contents}
:local:
:depth: 2
```

---

## The problem

The user noticed three issues in the simulator's **Visible Parts** panel:

The user's opening prompt:

> look into what happened to the meca500 end-effector. There is a part for
> it but we see a link\_5 before a6 in the list of parts in the simulator
> and no EE. link\_5 seems invisible as well. Can you look at the simulator
> in Chrome and investigate?

---

## Phase 1: Investigation

Claude launched two tasks in parallel: reading the browser state via the
Chrome connector, and exploring the codebase with a sub-agent.

### Reading the browser

Claude called `tabs_context_mcp` to discover the open simulator tab, then
`read_page` to get the accessibility tree. This revealed the Visible Parts
list: `All`, `A0`, `A1`, `A2`, `A3_4`, `A5`, **`link_5`**, `A6` —
confirming the phantom entry and missing EndEffector.

### Exploring the codebase

In parallel, an `Explore` sub-agent searched the codebase for all
meca500-related configuration. Key findings:

- **`robot.urdf`**: `link_5` was defined as `<link name="link_5"/>` — an
  empty link with no mesh, used as an intermediate frame between joint\_5
  (wrist pitch) and joint\_6 (wrist roll).
- **`EndEffector.stl`** (26 MB, 263K vertices) existed with a full
  analysis YAML, but was not referenced anywhere in the URDF.
- **`toolbar.py`**: `build_visibility_section` iterated all
  `chain_link_names` and created a chip for every link, even meshless ones.

### Root cause

The visibility panel code at `toolbar.py:256` used the link name as a
fallback label for meshless links instead of skipping them. And the
EndEffector was never added to the URDF kinematic chain.

Claude presented the findings and proposed two fixes. The user's response
set the tone for the rest of the session:

> I would like you to autonomously iterate until there is an end effector
> attached to the end of a6. Use bore edits of the EE if needed. OK?

---

## Phase 2: Fixing the visibility panel

A one-line change in `toolbar.py` — replace the `else: display = lname`
fallback with `else: continue` to skip links with no `mesh_path`:

```python
lnk = state.robot.get_link(lname)
if lnk and lnk.mesh_path:
    display = Path(lnk.mesh_path).stem
else:
    # Skip links with no mesh (e.g. link_5 intermediate frame)
    continue
```

This removed the phantom `link_5` entry from the Visible Parts panel.

---

## Phase 3: Adding the EndEffector to the URDF

### Initial placement

The EndEffector needed to attach to A6's distal bore, approximately 60 mm
along the X-axis from `link_6`'s origin. The mating surface of the
EndEffector faces −Z in mesh coordinates, so a rotation was needed to align
it with the +X axis of the link frame.

Claude added the EndEffector as a new link with a fixed joint, then used
the Chrome connector to reload the simulator and verify visually.

### Using the camera skill

At this scale, the EndEffector (~20 mm) was nearly invisible in the default
overview. Claude used the `zoom-rotate-camera` skill to position the camera
directly at the arm tip:

```javascript
sc.camera.position.set(0.22, -0.10, 0.35);
sc.controls.target.set(0.15, 0, 0.308);
sc.controls.update();
```

This revealed the EndEffector as a small pointed probe extending from
A6's bore centre — correctly positioned but needing rotation refinement.

### Restructuring: which joint rotates what

After verifying the EE was correctly placed, the user tested the joint
sliders and spotted a problem:

> it works. the EE is a little wobbly as it rotates - is the mesh not
> aligned with our axis?

But before the wobble, there was a more fundamental issue. The user had
noticed earlier:

> re the EE - looks good. But the A6 rotation is still rotating a6 itself
> whereas that should do the end effector.

In the real Meca500, joint\_6 is the tool roll — only the end-effector
should spin, not A6. The fix was to reassign meshes in the kinematic chain:

| Link | Before | After |
|------|--------|-------|
| `link_5` | *(empty)* | A6 mesh |
| `link_6` | A6 mesh | EndEffector mesh |

This eliminated the need for a separate `end_effector` link and `joint_ee`
fixed joint. A6 now rotates with joint\_5 (wrist pitch) and the
EndEffector rotates with joint\_6 (tool roll).

---

## Phase 4: Fixing the wobble

### Symptom

With the mesh reassignment done, the user tested again:

> still wobbly - the pin looks to be around 10 degrees downward at zero

The probe was tracing a cone instead of spinning cleanly around its axis.

### Diagnosis

The initial rotation assumed the EndEffector's axis aligned perfectly with
the mesh Z-axis (i.e. `rpy="0 1.5708 0"` — a 90° pitch). But the
EndEffector analysis YAML revealed the actual mating surface normal:

```yaml
normal: [-0.2392, 0.0198, -0.9708]
```

This is **14° off from pure −Z** — the mesh geometry is slightly tilted.
Using an exact 90° rotation left a 14° misalignment, which manifested as
wobble during rotation.

### Fix

The correct pitch angle is `arctan(0.9708 / 0.2392) = 76.15°` (1.3295
radians) instead of 90°, with a small yaw correction of 0.0198 radians:

```xml
<origin xyz="0.065795 0.000027 -0.000051" rpy="0 1.3295 0.0198"/>
```

After reloading, the user confirmed:

> nailed it. Please record this conversation as an example use of our skills and chrome connector.

---

## Tools and skills used

| Tool / Skill | Purpose |
|---|---|
| `mcp__claude-in-chrome__tabs_context_mcp` | Discover open browser tabs |
| `mcp__claude-in-chrome__read_page` | Read the accessibility tree to inspect UI state |
| `mcp__claude-in-chrome__computer` (screenshot) | Capture the 3D viewport for visual inspection |
| `mcp__claude-in-chrome__computer` (zoom) | Pixel-level inspection of small features |
| `mcp__claude-in-chrome__navigate` | Reload the simulator after URDF changes |
| `mcp__claude-in-chrome__javascript_tool` | Execute JS for camera control |
| `/zoom-rotate-camera` skill | Position the Three.js camera programmatically |
| `/robot-data` skill | Understand robot analysis YAML structure |
| `Explore` sub-agent | Parallel codebase investigation |

### The iteration loop

The workflow followed a tight loop:

1. **Edit** the URDF or Python code
2. **Reload** the simulator via the Chrome connector
3. **Position camera** using the `zoom-rotate-camera` skill
4. **Screenshot and zoom** to inspect the result
5. **Diagnose** any remaining issues from the visual
6. Repeat

Each iteration took about 30 seconds. The full session — from
investigation through final wobble fix — completed in a single
conversation.

---

## Key takeaways

- **Use the Chrome connector to close the feedback loop.** Rather than
  asking the user to describe what they see, Claude can screenshot the
  simulator, zoom into specific areas, and verify fixes directly.

- **Camera control via JavaScript is essential.** Mouse-dragging the 3D
  viewport risks accidentally moving joint sliders. The
  `zoom-rotate-camera` skill provides reliable, repeatable camera
  positioning.

- **Mesh geometry is not always axis-aligned.** The EndEffector's bore axis
  was reported as `[0, 0, 1]` in the analysis, but the actual surface
  normal was 14° off. Always verify rotations visually and use the measured
  surface normals rather than assumed axes.

- **Assign meshes to the right kinematic link.** A6 (wrist housing) should
  rotate with wrist pitch (joint\_5), not tool roll (joint\_6). Getting
  this wrong makes the simulator look broken even when the kinematics are
  correct.
