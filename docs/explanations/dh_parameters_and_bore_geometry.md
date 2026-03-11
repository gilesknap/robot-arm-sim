# DH Parameters and Bore Geometry

How Denavit-Hartenberg parameters and bore detection interact to produce
correct URDF joint origins and visual origins.

## DH Parameters

The **Denavit-Hartenberg (DH) convention** describes the kinematic chain of a
serial robot as a sequence of four parameters per joint:

| Parameter | Symbol | Meaning |
|-----------|--------|---------|
| Joint angle | θ | Rotation about the Z axis (variable for revolute joints) |
| Link offset | d | Translation along the Z axis |
| Link length | a | Translation along the X axis |
| Link twist | α | Rotation about the X axis |

Each (θ, d, a, α) tuple defines the homogeneous transform from frame *i−1*
to frame *i*.  Manufacturers publish these tables in datasheets — for example,
the UR5 datasheet gives:

| Joint | θ | d (mm) | a (mm) | α (rad) |
|-------|---|--------|--------|---------|
| 1 | 0 | 89.159 | 0 | π/2 |
| 2 | 0 | 135.85 | −425.0 | 0 |
| 3 | 0 | −119.7 | −392.25 | 0 |
| 4 | 0 | 109.15 | 0 | π/2 |
| 5 | 0 | 94.65 | 0 | −π/2 |
| 6 | 0 | 82.3 | 0 | 0 |

### Mapping DH to URDF

In URDF, each `<joint>` has an `<origin xyz="..." rpy="...">` that positions
the child frame relative to the parent frame.  The DH parameters map to these
as follows:

- **d** → translation along the parent Z axis
- **a** → translation along the (rotated) X axis
- **α** → rotation about the X axis (maps to `rpy`)

The exact mapping depends on axis conventions, but the key point is: **DH
parameters define the distance between consecutive joint axes**, measured
center-to-center through the link.

## Bore Geometry

A **bore** is a cylindrical opening in a robot link where a joint shaft or
bearing is inserted.  Each link typically has two bores:

- **Proximal bore**: where the link connects to the parent joint
- **Distal bore**: where the link connects to the child joint

### Face vs Center

Bore detection can report two positions along the bore axis:

- **Bore face**: the flat surface at the opening of the cylindrical bore.
  This corresponds to the bounding-box extreme along the bore axis — where the
  link material actually starts/ends.

- **Bore center** (barrel center): the geometric midpoint of the barrel
  cross-section along the bore axis.  This is where the actual joint rotation
  axis passes through.

```
                  bore axis
                     │
     ┌───────────────┼───────────────┐
     │               │               │
     │   ┌───────────┼───────────────┤ ← bore face (bbox extreme)
     │   │           │    barrel     │
     │   │           ●───────────────┤ ← bore center (bbox midpoint)
     │   │           │               │
     │   └───────────┼───────────────┤
     │               │               │
     └───────────────┼───────────────┘
                     │
              ← r →  ← r →
              (half barrel diameter)
```

The face and center differ by approximately half the barrel diameter,
typically 50–70 mm on UR-sized robots.

## How They Connect

**DH distances measure center-to-center**, not face-to-face.

When bore detection reports *face* positions, the face-to-face distance
between two connected bores is shorter than the DH distance by the sum of the
two half-barrel offsets at each end.

```
  Parent link                           Child link
  ┌──────────────┐                     ┌──────────────┐
  │         ╔════╗│   DH distance d    │╔════╗        │
  │         ║ B1 ║├───────────────────►│║ B2 ║        │
  │         ╚════╝│                     │╚════╝        │
  └──────────────┘                     └──────────────┘
            │  │                        │  │
         center face                 face center
            │  │                        │  │
            ├──┤                        ├──┤
             r1                          r2

  DH distance = face-to-face + r1 + r2
  gap = DH distance − face-to-face distance = r1 + r2
```

## Effect on URDF Generation

### Joint origins (kinematics)

Joint origins come directly from DH parameters (or the manufacturer chain
spec).  They are **not affected** by bore position definition — they always
measure the correct center-to-center distance.

### Visual origins (rendering)

Visual origins shift each mesh so that its reference point aligns with its
frame origin.  We use the **proximal bore** as the reference point: negate its
position to shift the mesh so the proximal bore lands at (0, 0, 0).

- **Using face positions**: the mesh is shifted so the proximal bore *face* is
  at the origin.  But DH distances measure to the bore *center*, so the mesh
  sits offset by half a barrel diameter, creating a visible gap.

- **Using center positions**: the mesh is shifted so the proximal bore
  *center* (barrel midpoint) is at the origin.  This matches the DH distance
  convention, closing the gap.

We compute the bore center by replacing the bore-axis component of the face
position with the bounding-box midpoint along that axis:

```python
axis_idx = max(range(3), key=lambda i: abs(bore_axis[i]))
pos[axis_idx] = (bbox_min[axis_idx] + bbox_max[axis_idx]) / 2
```

## UR5 Worked Example: Shoulder → Upperarm

Consider the connection between shoulder (link_1) and upperarm (link_2)
at joint_2, which has DH parameter d₂ = 135.85 mm along Y.

**Shoulder distal bore** (on link_1):
- Bore axis: Y
- Face position: Y = 70.5 mm (bbox extreme)
- Center position: Y ≈ 5.4 mm (bbox midpoint)

**Upperarm proximal bore** (on link_2):
- Bore axis: Y
- Face position: Y = −65.2 mm (bbox extreme)
- Center position: Y ≈ 1.7 mm (bbox midpoint)

**With face positioning**:
- Shoulder mesh ends at Y = 70.5 mm (face)
- Upperarm mesh starts at Y = −65.2 mm in its own frame
- After joint_2 translation of 135.85 mm, the upperarm face is at
  135.85 − 65.2 = 70.65 mm → flush with shoulder face (0.15 mm gap)
- But the mesh is only shifted to its face, not its center, leaving the
  visual center of the barrel offset from the joint axis

**With center positioning**:
- Shoulder mesh is centered at Y ≈ 5.4 mm
- Upperarm mesh center at Y ≈ 1.7 mm → shifted to origin
- After joint_2 translation, meshes overlap by the barrel thickness,
  producing the correct visual appearance with the joint axis passing
  through the barrel center on both sides
