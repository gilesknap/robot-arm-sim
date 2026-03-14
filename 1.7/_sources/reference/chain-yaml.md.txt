# chain.yaml specification

The `chain.yaml` file is a project-specific format that defines the kinematic
chain of a robot — which parts connect, what joints they form, and how the
STL meshes align with the kinematic frames. It is not an external standard;
it was designed to capture only the information that requires human or AI
judgement (topology, joint types, axes, limits) while leaving precise
geometric computation (connection point positions, visual offsets) to the
`robot-arm-sim generate` command, which reads this file alongside the
analysis data to produce a standard URDF.

## Top-level fields

### `robot_name`

**Type:** string
**Required:** no (defaults to `"robot"`)

The name written into the URDF `<robot>` element.

```yaml
robot_name: Meca500-R3
```

### `dh_params`

**Type:** mapping of string to number
**Required:** no

Denavit-Hartenberg parameters from the manufacturer, in millimetres. Used
only for FK validation after URDF generation — they do not affect the
generated transforms.

```yaml
dh_params:
  d1: 135    # base to J2
  a2: 135    # upper arm length
  a3: 38     # elbow offset (X)
  d4: 120    # forearm length
  d6: 70     # wrist to flange
```

### `links`

**Type:** list of link specifications
**Required:** yes

### `joints`

**Type:** list of joint specifications
**Required:** yes

---

## Link specification

Each entry in the `links` list defines one link in the kinematic chain.

### `name`

**Type:** string
**Required:** yes

The link name used in URDF and referenced by joints.

### `mesh`

**Type:** string or `null`
**Required:** yes

The STL file stem name (without `.stl` extension). The generator looks for
`<robot_dir>/stl_files/<mesh>.stl`. Set to `null` for virtual links that
have no geometry (e.g. when two joints are co-located).

```yaml
- name: link_4
  mesh: null
```

### `visual_rpy`

**Type:** list of 3 floats `[roll, pitch, yaw]` in radians
**Default:** `[0, 0, 0]`

Rotation applied to the STL mesh to align its coordinate frame with the link
frame convention (Z-up). Use this when the STL's natural orientation doesn't
match.

Common example — a part whose long axis is STL +X but should point up (+Z):

```yaml
visual_rpy: [0, -1.5708, 0]   # -90° pitch rotates X to Z
```

### `visual_xyz`

**Type:** list of 3 floats `[x, y, z]` in metres
**Default:** `[0, 0, 0]`

**This is additive.** The generator automatically computes a visual offset
from the mesh's proximal connection point (so the connection centre sits at the
link frame origin). The `visual_xyz` value is added on top of that
auto-computed offset.

- `[0, 0, 0]` means "use auto-computed position only"
- Small adjustments (1-5 mm / 0.001-0.005 m) are typical for imprecise
  connection point detection
- Values larger than 40 mm usually indicate the wrong connection point was
  detected — check the analysis YAML

```yaml
visual_xyz: [0, 0, -0.039]   # shift mesh 39mm down
```

---

## Joint specification

Each entry in the `joints` list defines one joint.

### `name`

**Type:** string
**Required:** yes

The joint name used in the URDF.

### `type`

**Type:** string
**Required:** yes

URDF joint type. Supported values: `revolute`, `continuous`, `fixed`,
`prismatic`.

### `parent`

**Type:** string
**Required:** yes

Name of the parent link (must match a `links[].name`).

### `child`

**Type:** string
**Required:** yes

Name of the child link (must match a `links[].name`).

### `axis`

**Type:** list of 3 floats `[x, y, z]`
**Required:** yes

Unit vector defining the joint rotation axis in the child link frame.

| Rotation | Axis |
|---|---|
| Yaw (vertical) | `[0, 0, 1]` |
| Pitch (horizontal) | `[0, 1, 0]` |
| Roll (along forearm) | `[1, 0, 0]` |

### `limits`

**Type:** list of 2 floats `[lower, upper]` in radians
**Required:** for `revolute` joints

Joint angle limits. Used for URDF `<limit>` element and simulator slider
range.

```yaml
limits: [-3.054, 3.054]   # approximately ±175°
```

### `effort`

**Type:** number
**Default:** `100`

Maximum joint effort (N·m). Written to the URDF `<limit>` element.

### `velocity`

**Type:** number
**Default:** `1`

Maximum joint velocity (rad/s). Written to the URDF `<limit>` element.

### `origin`

**Type:** list of 3 floats `[x, y, z]` in metres
**Default:** auto-computed from connection points

Explicit joint origin position in the parent link frame. **Overrides**
auto-computation from the parent mesh's distal connection point.

Use this when:

- The parent mesh has no analysis data
- The detected connection point is wrong
- The parent is a virtual link (`mesh: null`)

```yaml
origin: [0, 0, 0.093]   # 93mm above parent frame origin
```

If omitted, the generator computes the origin as:

```
origin = (parent_distal_bore - parent_proximal_bore) / 1000
```

converting from STL millimetres to URDF metres.

### `origin_rpy`

**Type:** list of 3 floats `[roll, pitch, yaw]` in radians
**Default:** `[0, 0, 0]`

Rotation of the child link frame relative to the parent. Prefer changing
`axis` instead of using `origin_rpy` — it is simpler and more intuitive.

---

## Complete example

The Meca500-R3 chain.yaml demonstrates most features — `visual_xyz` offsets,
virtual links (`mesh: null`), varied joint axes, and explicit origins:

```{literalinclude} ../../robots/Meca500-R3/chain.yaml
:language: yaml
```
