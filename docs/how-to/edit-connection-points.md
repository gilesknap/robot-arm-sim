# Edit connection points

This guide explains what connection points are, how the two centering modes
work, and how to edit them to fix mesh placement in the URDF.

## What connection points are

Every robot part has one or two **connection points** — the positions where it
attaches to its neighbours in the kinematic chain:

- **proximal** — where this part connects to its parent (closer to the base)
- **distal** — where this part connects to its child (closer to the
  end-effector)

The base link has only a distal connection; the end-effector has only a
proximal connection; all other links have both.

Each connection point has these fields:

```{literalinclude} ../../src/robot_arm_sim/models/models.py
:language: python
:start-at: class ConnectionPoint
:end-at: centering
:caption: ConnectionPoint model
```

| Field | Meaning |
|---|---|
| `end` | `"proximal"` or `"distal"` |
| `position` | XYZ coordinates of the marker in STL mesh space (mm) |
| `axis` | Unit vector defining the joint rotation axis direction |
| `radius_mm` | Estimated bore/shaft radius |
| `method` | How the point was detected: `"cross_section"`, `"cylinder_fit"`, `"manual"`, etc. |
| `centering` | `"surface"` or `"center"` — controls how the visual origin is computed |

Connection points are stored in each part's analysis YAML file (e.g.
`robots/MyRobot/analysis/A1.yaml`). Here is a typical example:

```{literalinclude} ../../robots/Meca500-R3/analysis/A1.yaml
:language: yaml
:lines: 4-28
:caption: Connection points from A1.yaml
```

## The two centering modes

The `centering` field controls how `compute_visual_origin` positions the mesh
within its link frame. The goal is always the same: place the proximal
connection point at the link frame origin so the joint rotates around the
correct axis.

### Axis snapping (both modes)

Regardless of centering mode, `compute_visual_origin` always **snaps the
proximal bore center onto the joint axis line**. It decomposes the proximal
position into along-axis and cross-axis components relative to the joint
axis, then zeros the cross-axis error:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:lines: 93-122
:caption: Axis snapping and centering logic in compute_visual_origin
```

This means each mesh independently positions itself — no cross-link
dependencies, no error accumulation up the chain.

### `surface` mode (default)

The marker sits on the **bore surface** — a flat mating face. The along-axis
component of the proximal position is used as-is (only cross-axis is zeroed).

After all visual origins are computed, a second pass
(`close_surface_gaps_along_axis`) shifts each surface-mode child **along the
joint axis only** so its proximal surface meets the parent's distal surface:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:lines: 210-221
:caption: close_surface_gaps_along_axis docstring
```

This is the right choice when the connection is a flat face mating against
another flat face (the most common case).

### `center` mode

The marker sits on a **surface face that represents where the rotation axis
should pass through** — typically the end of a through-bore or shaft hole.
The pipeline finds the opposite flat face along the bore axis and averages
the two positions to compute the bore midpoint. The result is then projected
onto the joint axis.

This is the right choice when the connection is a through-bore where the
rotation axis must pass through the geometric center of the bore, not a
surface face. Center-mode connections are **not** adjusted by the gap-closing
pass — their bore centering already defines the rotation axis precisely.

## How connection points and `visual_rpy` work together

Connection points tell the pipeline **where** each joint is. But when a
part's proximal and distal faces are on different planes — for example, an
L-shaped link where the proximal bore faces along Z but the distal bore
faces along Y — the mesh also needs to be **rotated** so the proximal axis
aligns with the joint frame. This is what `visual_rpy` in `chain.yaml` does.

The URDF generation pipeline works in two passes:

1. **Pass 1 — per-mesh snap** (`compute_visual_origin`) — for each link,
   rotate the mesh (`visual_rpy`) and snap its proximal bore center onto the
   joint axis line, zeroing cross-axis error. Along-axis placement depends
   on centering mode.
2. **Pass 2 — surface gap closure** (`close_surface_gaps_along_axis`) — for
   surface-mode connections only, shift the child mesh along the joint axis
   so its proximal face meets the parent's distal face.

When `visual_rpy` is `[0, 0, 0]` (the default), the translation is a simple
negation of the proximal position. When a rotation is applied, the proximal
position is rotated first, then negated:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:lines: 129-137
:caption: visual_rpy applied before translation in compute_visual_origin
```

### Why this matters for fixing jumbled parts

When auto-detection picks the wrong connection points, parts end up in the
wrong position *and* orientation. Fixing this requires both steps:

1. **Place the markers correctly** — use Edit Connections to assign proximal
   and distal markers to the right faces. This tells the pipeline where the
   joint axes are, and gives it the axis directions from the face normals.

2. **Set `visual_rpy` if the axes aren't aligned** — if a part's proximal
   face is not perpendicular to the joint axis (i.e. the STL mesh
   coordinates don't naturally align with the link frame), you need a
   `visual_rpy` rotation in `chain.yaml` to bring them into alignment.

For a typical straight part where proximal and distal are on parallel faces
along the same axis, `visual_rpy` can stay at `[0, 0, 0]`. For L-shaped or
angled parts where the two connections face different directions, you need a
rotation to align the proximal axis with the joint frame.

**Work from base to tip.** Each link's visual origin is computed
independently (snapped to its own joint axis), so errors no longer compound
along the chain. However, surface gap closure still references the parent's
distal position, so fixing a parent link's connections first makes it easier
to verify child links visually.

## How to edit connection points

### Using the simulator UI

1. Launch the simulator:

   ```bash
   uv run robot-arm-sim simulate robots/MyRobot/
   ```

2. Click **Edit Connections** in the toolbar — meshes go semi-transparent and
   coloured sphere markers appear at every flat face centroid.

3. Use the **Proximal / Distal** toggle to choose which end to assign.

4. Select a **centering mode** from the dropdown (`surface` or `center`).

5. Click a yellow marker to assign it. Markers turn **green** (proximal) or
   **red** (distal).

6. Click **Save & Rebuild** — updated connection points are written to the
   analysis YAML (with `method: manual`) and the URDF is regenerated.

### Editing analysis YAML directly

You can also edit the `connection_points` section of any analysis YAML file
directly. After editing:

```bash
uv run robot-arm-sim generate robots/MyRobot/
```

This regenerates the URDF from the updated analysis data.

## When to use each mode

| Geometry | Mode | Why |
|---|---|---|
| Flat mating faces (most joints) | `surface` | Marker on face surface; gap-closing shifts along axis to meet parent |
| Through-bores / shaft holes | `center` | Marker at bore center; pipeline averages with opposite face, snaps to axis |
| Barrel bores (large concentric cylinders) | `center` | Bore center defines the rotation axis; snapped onto axis line |
| Shallow surface features | `surface` | No opposite face to average with; surface position is sufficient |

**Rule of thumb:** if the joint rotation axis passes *through* a bore, use
`center`. If the joint is at a flat face where two parts mate, use `surface`.
