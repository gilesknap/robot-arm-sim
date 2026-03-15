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

On multi-axis (L-shaped) parts where bores face along two perpendicular
axes, the auto-detector assigns proximal/distal by **Z-position** of the
detected bore centers: the bore at lower Z is proximal (toward the base)
and the bore at higher Z is distal (toward the tip). This relies on the
STL convention of Z-up. For parts with no cylindrical surfaces (e.g. an
end-effector flange), the detector falls back to the largest flat face
along any axis.

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

### Visual origin computation (both modes)

For each link, `compute_visual_origin` negates the proximal connection
point position (converting from mm to metres) so the connection sits at
the link frame origin. Each mesh is positioned independently — no
cross-link dependencies, no error accumulation up the chain.

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:start-at: def compute_visual_origin(
:end-at: return [round(v, 6)
:caption: compute_visual_origin — places proximal connection at frame origin
```

### `surface` mode (default)

The marker sits on the **bore surface** — a flat mating face. The along-axis
component of the proximal position is used as-is (only cross-axis is zeroed).

After all visual origins are computed, a second pass
(`close_surface_gaps_along_axis`) shifts each surface-mode child **along the
joint axis only** so its proximal surface meets the parent's distal surface:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:start-at: def close_surface_gaps_along_axis(
:end-before: link_specs = {
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

The URDF generation pipeline works in five passes:

0. **Pass 0 — visual flip detection** (`auto_detect_visual_flips`) — detects
   links where the proximal connection is frame-flipped relative to the joint
   axis. For these links, the anchor switches to the distal connection point
   so `compute_visual_origin` positions the mesh correctly.
1. **Pass 1 — per-mesh snap** (`compute_visual_origin`) — for each link,
   rotate the mesh (`visual_rpy`) and negate the anchor position to place the
   connection point at the link frame origin. For `center` mode, the position
   is averaged with the opposite face along the bore axis before negation.
2. **Pass 2 — surface gap closure** (`close_surface_gaps_along_axis`) — for
   surface-mode connections only, shift the child mesh along the joint axis
   so its proximal face meets the parent's distal face.
3. **Pass 2b — derived joint update** (`update_derived_joint_origins`) —
   recompute connection-point-derived joint origins to reflect any visual
   shifts from gap closing.
4. **Pass 3 — visual_xyz nudge** — apply any `visual_xyz` offset from
   `chain.yaml` as a local-only adjustment. This is added *after*
   gap-closing, so nudging one part never affects any other part.

When `visual_rpy` is `[0, 0, 0]` (the default), the translation is a simple
negation of the proximal position. When a rotation is applied, the proximal
position is rotated first, then negated:

```{literalinclude} ../../src/robot_arm_sim/analyze/urdf_transforms.py
:language: python
:start-at: if viz_rpy == [0, 0, 0]:
:end-at: viz_xyz = (-rpy_to_rotation
:caption: visual_rpy applied before translation in compute_visual_origin
```

### Fixing jumbled parts

When auto-detection picks the wrong connection points, see
{doc}`/how-to/edit-connection-points` for step-by-step instructions on
placing markers and setting `visual_rpy`.

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
        pass0["Pass 0: auto-detect visual flips<br/>flip anchor for frame-flipped links"]
        pass1["Pass 1: per-mesh snap<br/>proximal + visual_rpy → visual origin"]
        pass2["Pass 2: gap-closing<br/>parent distal → child shift along axis"]
        pass2b["Pass 2b: derived joint update<br/>recompute connection-point-derived joint origins"]
        pass3["Pass 3: visual_xyz nudge<br/>local-only, never propagated"]
        joints["Joint origins ← DH params only"]
        fk["FK validation<br/>verify joint + visual origins"]
        pass0 --> pass1 --> pass2 --> pass2b --> pass3
        pass3 --> fk
    end

    subgraph output ["Output"]
        urdf["robot.urdf"]
    end

    analysis --> pass0
    analysis --> pass1
    analysis --> pass2
    chain --> pass1
    chain --> joints
    chain --> pass3
    fk --> urdf
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
