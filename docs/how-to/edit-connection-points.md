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

### `surface` mode (default)

The marker sits on the **bore surface** — a flat mating face. The raw marker
position is used directly as the visual origin reference point. No axial
adjustment is applied.

This is the right choice when the connection is a flat face mating against
another flat face (the most common case).

### `center` mode

The marker sits at the **bore center** — inside a through-bore or shaft hole.
The pipeline finds the opposite flat face along the bore axis and averages
the two positions to compute the bore midpoint:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:lines: 89-103
:caption: Centering logic in compute_visual_origin
```

This is the right choice when the connection is a through-bore where the
rotation axis must pass through the geometric center of the bore, not a
surface face.

## Why `close_surface_gaps` skips center-mode connections

After computing visual origins, a second pass called `close_surface_gaps`
walks the kinematic chain and shifts each child mesh so its proximal surface
meets the parent's distal surface exactly — closing any small gaps caused by
mesh tolerances.

**This pass only applies to `surface`-mode connections.** Center-mode
connections are skipped:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:lines: 198-206
:caption: close_surface_gaps docstring explaining the skip
```

The reason: gap-closing computes a 3D shift vector between parent distal and
child proximal positions. For surface-mode connections this shift is
predominantly along the bore axis (closing an axial gap), which is harmless.
But for center-mode connections the bore centering already defines the
rotation axis precisely. Applying a cross-axis shift (perpendicular to the
bore) would accumulate errors up the kinematic chain, progressively skewing
each joint's axial rotation. This was discovered during debugging when
center-mode parts drifted sideways through the chain, causing visible
rotation misalignment.

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
| Flat mating faces (most joints) | `surface` | Marker on face surface; gap-closing applies normally |
| Through-bores / shaft holes | `center` | Marker at bore center; pipeline averages with opposite face for precise axis |
| Barrel bores (large concentric cylinders) | `center` | Bore center defines the rotation axis |
| Shallow surface features | `surface` | No opposite face to average with; surface position is sufficient |

**Rule of thumb:** if the joint rotation axis passes *through* a bore, use
`center`. If the joint is at a flat face where two parts mate, use `surface`.
