# URDF generation pipeline

How connection points, centering modes, and chain.yaml settings combine
to produce visual origins in the generated URDF.

For step-by-step editing instructions, see
{doc}`/how-to/edit-connection-points`.

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
:lines: 92-113
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
:lines: 186-197
:caption: close_surface_gaps_along_axis docstring
```

This is the right choice when the connection is a flat face mating against
another flat face (the most common case). The parent's distal marker indicates
which surface face the child should be shifted to meet.

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

## The three-pass pipeline

Connection points tell the pipeline **where** each joint is. But when a
part's proximal and distal faces are on different planes — for example, an
L-shaped link where the proximal bore faces along Z but the distal bore
faces along Y — the mesh also needs to be **rotated** so the proximal axis
aligns with the joint frame. This is what `visual_rpy` in `chain.yaml` does.

The URDF generation pipeline works in three passes:

1. **Pass 1 — per-mesh snap** (`compute_visual_origin`) — for each link,
   rotate the mesh (`visual_rpy`) and snap its proximal bore center onto the
   joint axis line, zeroing cross-axis error. Along-axis placement depends
   on centering mode.
2. **Pass 2 — surface gap closure** (`close_surface_gaps_along_axis`) — for
   surface-mode connections only, shift the child mesh along the joint axis
   so its proximal face meets the parent's distal face.
3. **Pass 3 — visual_xyz nudge** — apply any `visual_xyz` offset from
   `chain.yaml` as a local-only adjustment. This is added *after*
   gap-closing, so nudging one part never affects any other part.

When `visual_rpy` is `[0, 0, 0]` (the default), the translation is a simple
negation of the proximal position. When a rotation is applied, the proximal
position is rotated first, then negated:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:lines: 108-113
:caption: visual_rpy applied before translation in compute_visual_origin
```

### Why this matters for fixing jumbled parts

When auto-detection picks the wrong connection points, parts end up in the
wrong position *and* orientation. Fixing this requires both steps:

1. **Place the markers correctly** — use Edit Connections and click directly on
   the correct mesh surfaces to assign proximal and distal markers. This tells
   the pipeline where the joint axes are, and gives it the axis directions from
   the face normals.

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

## What affects what

Understanding which inputs affect which outputs prevents surprises when
editing connection points or chain.yaml values.

### Data flow summary

```{mermaid}
flowchart TB
    subgraph inputs ["Input files"]
        analysis["analysis/*.yaml<br/>proximal position<br/>distal position<br/>centering mode"]
        chain["chain.yaml<br/>DH params / origins<br/>visual_rpy<br/>visual_xyz"]
    end

    subgraph pipeline ["URDF generation pipeline"]
        direction TB
        pass1["Pass 1: per-mesh snap<br/>proximal + visual_rpy → visual origin"]
        pass2["Pass 2: gap-closing<br/>parent distal → child shift along axis"]
        pass3["Pass 3: visual_xyz nudge<br/>local-only, never propagated"]
        joints["Joint origins ← DH params only"]
        pass1 --> pass2 --> pass3
    end

    subgraph output ["Output"]
        urdf["robot.urdf"]
    end

    analysis --> pass1
    analysis --> pass2
    chain --> pass1
    chain --> joints
    chain --> pass3
    pass3 --> urdf
    joints --> urdf
```

### What each input controls

| Input | Stored in | Affects | Propagates? |
|---|---|---|---|
| **DH params / joint origins** | `chain.yaml` | Joint frame positions (the kinematic chain) | N/A — defines the chain |
| **Proximal position** | `analysis/*.yaml` | That link's visual origin (mesh placement) | No — local to the link |
| **Distal position** | `analysis/*.yaml` | Gap-closing shift on the **child** link (surface mode only) | One level down only |
| **Centering mode** | `analysis/*.yaml` | Whether proximal is averaged with opposite face | No — local to the link |
| **`visual_rpy`** | `chain.yaml` | Mesh rotation before proximal snap | No — local to the link |
| **`visual_xyz`** | `chain.yaml` | Final nudge after all other computation | No — local to the link |

(key-rules)=
### Key rules

- **Nothing propagates upstream.** Changing a child never affects its parent.
- **Proximal markers are local.** Moving a proximal only shifts that link's
  mesh — no other link is affected.
- **Distal markers affect one level down.** A parent's distal position feeds
  into gap-closing for its immediate child (surface mode only). It does not
  affect the kinematic chain.
- **`visual_xyz` is always local.** It is applied after gap-closing and
  never feeds into any other calculation. Use it to nudge a single part
  without side effects.
- **Joint origins are fixed.** They come from DH parameters in `chain.yaml`
  and are never derived from connection point markers (when DH params are
  present). The kinematic chain does not change when you edit markers.
- **Re-analysis can change visual origins.** Running `analyze` regenerates
  `analysis/*.yaml` which may shift proximal positions. Running `generate`
  afterwards will produce different visual origins even though `chain.yaml`
  hasn't changed.
