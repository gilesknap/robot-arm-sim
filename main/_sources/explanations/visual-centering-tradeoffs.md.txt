# Visual centering tradeoffs

Why connection-point-based positioning can look wobbly, why zeroing
cross-axis offsets looks smooth, and why neither approach is universally
correct.

This page explains the tradeoffs discovered during the development of the
`/center-on-axis` skill. For the URDF generation pipeline that produces
these visual origins, see {doc}`urdf-generation-pipeline`.

## Two sources of positioning information

When placing a mesh in its link frame, the pipeline has two independent
sources of cross-axis positioning:

1. **Connection point detection** — slices the mesh near each end, fits
   circles to the cross-sections, and uses the circle centre as the bore
   position. This is a geometric measurement of the actual mesh.

2. **Mesh coordinate origin** — the (0, 0) point in the STL file's own
   coordinate system. This is wherever the CAD designer happened to place
   the origin when exporting the file.

The standard pipeline uses source 1. The `/center-on-axis` skill
effectively switches to source 2 by zeroing the cross-axis components of
the visual origin, which places mesh (0, 0) on the joint axis.

## How connection point detection introduces wobble

The connection point detector calls `find_circle_center_at_slice` to
locate bore centres:

```{literalinclude} ../../src/robot_arm_sim/analyze/circle_fitting.py
:language: python
:pyobject: _extract_circle_center_from_section
:caption: Circle centre extraction from cross-section polygons
```

This finds the most circular polygon in a cross-section slice and uses its
centroid as the bore centre. Two things can shift that centroid away from
the true rotation axis:

- **Asymmetric mesh geometry.** The most circular polygon at a slice
  position may not be the bore — it could be the outer housing profile.
  Flanges, cable channels, or mounting bosses shift the centroid of that
  profile away from the true axis.

- **Slice position sensitivity.** The detector slices at 5%, 10%, 15%
  from the mesh end. At those positions, transition features (fillets,
  chamfers, flanges) distort the cross-section shape and bias the
  centroid.

Crucially, these errors are **independently computed** for each connection
point. The parent's distal bore centre and the child's proximal bore
centre are found from different cross-sections on different meshes.
When these small errors don't match, adjacent parts appear offset from
each other — visible as wobble during rotation.

## How zeroing cross-axis offsets eliminates wobble

The `/center-on-axis` skill takes a completely different approach: instead
of using the detected bore centre, it places each mesh so that its own
coordinate origin (0, 0) sits on the joint axis. In code terms, it zeroes
the cross-axis components of the URDF visual origin.

This eliminates wobble because **all meshes in a robot share the same
origin convention**. If the CAD designer placed the origin consistently
(e.g., on the main cylindrical axis), then every parent–child junction
shares the same systematic bias. Consistent bias looks smooth; inconsistent
bias looks wobbly.

## The accuracy tradeoff

Neither approach is geometrically perfect:

| Approach | Cross-axis accuracy | Visual smoothness |
|---|---|---|
| **Connection points** | Better — measures actual bore geometry | Worse — independent errors create parent–child mismatch |
| **Center-on-axis** | Worse — assumes mesh origin is on bore axis | Better — consistent origin convention across all parts |

### When mesh origins are on the bore axis

Some STL files are modeled with the coordinate origin on the rotation
axis. The Meca500-R3 is an example — its proximal connection points are
within 0.2 mm of (0, 0) in the cross-axis plane:

| Part | Proximal cross-axis offset from (0, 0) |
|------|---------------------------------------|
| A1   | x=0.0 mm, y=0.0 mm |
| A2   | x=0.0 mm, z=0.0 mm |
| A3_4 | x=0.0 mm, z=0.0 mm |
| A5   | y=0.0 mm, z=0.0 mm |
| A6   | x=0.0 mm, z=0.0 mm |

For these meshes, center-on-axis is both accurate and smooth — the mesh
origin genuinely is the bore centre.

### When mesh origins are not on the bore axis

Other STL files have origins offset from the bore by several millimetres.
The UR5 is an example:

| Part | Proximal cross-axis offset from (0, 0) |
|------|---------------------------------------|
| shoulder  | x=−1.8 mm, y=6.4 mm |
| upperarm  | x=5.3 mm, z=0.6 mm |
| forearm   | x=−2.0 mm, z=−2.4 mm |
| wrist1    | x=0.9 mm, z=0.5 mm |
| wrist2    | x=0.2 mm, y=−0.8 mm |
| wrist3    | x=−1.1 mm, z=3.5 mm |

For these meshes, center-on-axis trades accuracy for smoothness. The bore
holes in adjacent parts no longer align — they can be millimetres apart —
but because the misalignment is consistent, the robot still looks correct
at normal viewing scales. For the UR5 (850 mm reach), a 5 mm cross-axis
error is hard to see.

## Why STL origins vary

STL files do not encode any semantic information about rotation axes or
joint centres. The coordinate origin is simply wherever the CAD tool
placed it during export. Common conventions include:

- **Bore-centred** — origin on the main cylindrical axis (e.g.,
  Meca500-R3). This is common when meshes are exported specifically for
  simulation.
- **Assembly-origin** — origin at a corner, mounting face, or assembly
  reference point (e.g., UR5). This is common when meshes are exported
  from a full-assembly CAD model.

There is no way to determine the convention from the STL file alone. The
only way to know is to inspect the mesh geometry or consult the CAD
source.

## Implications

- **Connection points remain valuable.** They provide the on-axis (depth)
  positioning that center-on-axis preserves, and they compute joint-to-joint
  distances from geometry when DH parameters are not available. See
  {doc}`decisions/0003-keep-connection-point-auto-detection` for the
  rationale behind keeping auto-detection.

- **Center-on-axis is a visual polish step, not a correction.** It should
  be applied after connection points are set, and users should understand
  that it may shift bores off-axis for meshes whose origins are not on the
  bore.

- **Neither approach can be eliminated.** Connection points give depth;
  mesh origins give consistent cross-axis alignment. The pipeline needs
  both.
