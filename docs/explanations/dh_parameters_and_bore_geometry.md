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

## Forward Kinematics (FK)

**Forward kinematics** is the process of computing the position and orientation
of every link in the robot given a set of joint angles.  It is the direct
consumer of DH parameters: FK walks the kinematic chain from base to tip,
multiplying 4×4 homogeneous transforms at each joint to accumulate the
world-frame pose of every link.

For each joint in the chain the transform is:

```
T_child = T_parent × T_origin × R_joint(θ)
```

where:

- **T_origin** is the fixed offset from the parent frame to the joint frame,
  derived from the DH parameters (d, a, α) and stored in the URDF as
  `<joint><origin xyz="..." rpy="...">`.
- **R_joint(θ)** is the rotation produced by the current joint angle θ around
  the joint axis.

Starting from the base link at identity, FK multiplies these transforms
sequentially to produce a dictionary of `link_name → 4×4 matrix` in world
coordinates.  The translation column of the final matrix is the end-effector
position; the rotation sub-matrix is its orientation.

### Relationship to the other concepts on this page

| Concept | Role in FK |
|---------|-----------|
| **DH parameters** | Source of the fixed joint-origin transforms (T_origin) |
| **Bore geometry** | Determines the *visual* mesh offset so that rendered links align with the FK-computed frames — FK itself only uses DH-derived origins, not bore positions |
| **URDF joint origins** | The concrete encoding of DH parameters that FK reads at runtime |
| **Visual origins** | Shift meshes so the proximal bore center coincides with the frame origin that FK computes for that link |

In short: DH parameters define *where* each joint frame sits, bore geometry
defines *how* the mesh is shifted to match, and FK multiplies the chain of
joint-frame transforms to get world-space poses for simulation and rendering.

## Inverse Kinematics (IK)

**Inverse kinematics** is the reverse of FK: given a desired end-effector
position (and optionally orientation), find the set of joint angles that
achieves it.  While FK has a unique closed-form solution — multiply transforms
down the chain — IK is generally harder:

- There may be **multiple solutions** (a 6-DOF arm can often reach the same
  point with different elbow/wrist configurations).
- There may be **no solution** (the target is outside the workspace).
- For general robots the problem is solved **numerically** rather than
  analytically.

This project uses the [ikpy](https://github.com/Phmusic/ikpy) library, which
builds an IK chain directly from the URDF file and solves numerically via
iterative optimization.  The solver takes:

1. A **target position** `[x, y, z]` in world coordinates (meters).
2. The **current joint angles** as an initial guess to seed the optimizer and
   bias toward nearby solutions.

It returns a new set of joint angles whose FK result places the end-effector
at (or near) the target.

### How IK relates to the other concepts on this page

| Concept | Role in IK |
|---------|-----------|
| **DH parameters** | Define the kinematic model that IK must invert — link lengths and twists determine the reachable workspace |
| **FK** | IK solvers evaluate FK internally on every iteration to measure how close the current guess is to the target |
| **URDF** | ikpy reads the URDF to build its internal chain, so correct joint origins (from DH) are essential for accurate IK |
| **Bore geometry** | Not directly involved — IK operates on joint-space kinematics, not visual mesh positioning |

The IK solver lives in `src/robot_arm_sim/simulate/ik_solver.py`.

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

## Where to Find These in a Robot Folder

Each robot lives under `robots/<RobotName>/`.  Here is how the concepts on this
page map to files you can inspect:

```
robots/<RobotName>/
├── specs.yaml              # Manufacturer DH parameters (d, a, α tables)
├── chain.yaml              # Project kinematic chain: DH params, joint
│                           #   definitions (axes, origins, limits), link
│                           #   list with visual offsets
├── robot.urdf              # Generated URDF — the concrete encoding of
│                           #   joint origins (from DH/chain) and visual
│                           #   origins (from bore analysis) that FK reads
├── verify_kinematics.py    # FK verification script — runs forward
│                           #   kinematics at known poses and checks
│                           #   results against expected DH positions
├── analysis/
│   ├── <part>.yaml         # Per-link bore geometry: proximal/distal
│   │                       #   connection points (position, axis, radius)
│   └── summary.yaml        # Aggregate analysis across all links
├── stl_files/
│   └── <part>.stl          # Visual meshes positioned by bore offsets
└── images/
    └── *.svg / *.png       # Reference diagrams (DH joint layout, etc.)
```

The FK implementation is in `src/robot_arm_sim/simulate/kinematics.py`
(`forward_kinematics()`) and the IK solver is in
`src/robot_arm_sim/simulate/ik_solver.py` (`solve_ik()`).
