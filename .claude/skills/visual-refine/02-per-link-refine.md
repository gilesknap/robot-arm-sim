# Per-Link Visual Refinement

## Goal

For each link from base to tip, visually assess mesh placement and
apply `visual_xyz` / `visual_rpy` corrections to achieve <2mm gaps
between parts at joints.

## Setup

Enable these simulator controls before starting:
- **Frames** — shows RGB coordinate axes at each joint frame
  (Red=X, Green=Y, Blue=Z)
- **Transparent** — lets you see frame axes through the mesh

Reset all joints to zero configuration.

## Per-Link Workflow

Work from the **base link** upward toward the tip. For each link:

### 1. Isolate the view

Use **Visible Parts** checkboxes to hide all links above the
current one. Show the current link and all links below it.

### 2. Assess proximal alignment

The link's **proximal end** should sit at the creating joint's
frame origin (where the RGB axes are). Check:

- Is the mesh centered on the frame origin?
- Is the frame origin inside the mesh (good) or floating in space
  (bad — needs xyz correction)?
- How far off is it? Estimate in millimetres.

### 3. Assess distal alignment

The link's **distal end** should point toward the next joint's
frame origin. Check:

- Does the mesh extend in the right direction?
- Is the next joint's frame at the correct end of the mesh?
- If the mesh is rotated (distal pointing wrong way), a
  `visual_rpy` correction is needed.

### 4. Compare to reference

Compare the simulator view against the manufacturer reference
image:

- Does the part shape match?
- Is the part oriented correctly?
- Are proportions right (length vs width)?

### 5. Estimate corrections

Based on the visual assessment:

**For position offsets (`visual_xyz`):**
- Estimate the offset in metres (e.g. 5mm = 0.005)
- Remember visual_xyz is in the link's local frame
- The coordinate system depends on joint rotations above

**For rotation corrections (`visual_rpy`):**
- Only needed when the mesh is fundamentally rotated
  (e.g. 90 degrees around an axis)
- Express as roll, pitch, yaw in radians
- Common values: pi/2 = 1.5708, pi = 3.1416

### 6. Apply and verify

1. Add `visual_xyz` and/or `visual_rpy` to chain.yaml for this link
2. Regenerate: `uv run python -m robot_arm_sim generate robots/<name>/`
3. Reload URDF in the simulator (click **Reload URDF**)
4. Re-check the alignment
5. Iterate until the gap is <2mm

### 7. Move to next link

Once the current link looks correct, show the next link and repeat.

## Estimation Tips

### Gauging distance from screenshots

- Use the frame axis arrows as a scale reference — they're typically
  50-100mm long in the viewport
- Compare the gap size to known dimensions from the DH parameters
- A gap that's barely visible is likely <2mm (acceptable)
- A gap showing clear daylight between parts needs correction

### Common patterns

- **Consistent Y offset on bore-axis links**: The gap-closing
  algorithm may over/under-shift. Apply the same Y correction to
  all links along that bore axis.
- **Base link floating above ground**: Usually needs a small Z
  correction to place the base on the ground plane.
- **Wrist parts overlapping**: The auto-detection may place CPs at
  bore centers rather than bore surfaces. A correction along the
  bore axis fixes this.
- **User-added links (EndEffector)**: These have no CPs at all and
  always need manual `visual_xyz` + `visual_rpy`.

### Using multiple camera angles

- **Side view (RIGHT)**: Best for checking vertical alignment and
  arm extension
- **Front view (FRONT)**: Best for checking lateral offsets
- **Top view (TOP)**: Best for checking rotation around Z axis
- **Perspective view**: Good for overall shape assessment but bad
  for precise measurement

## Gate

For each link:
- Frame origin is inside or at the surface of the mesh proximal end
- Mesh extends toward the next joint
- Gap between this mesh and adjacent meshes is <2mm
- Visual appearance matches manufacturer reference images
