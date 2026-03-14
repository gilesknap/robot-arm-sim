# Add a new robot

This guide walks through the complete process of taking a set of STL mesh
files for a robot arm and producing a working interactive simulation. It
covers using Claude Code with the project's built-in skills to automate much
of the work.

## Prerequisites

- A working devcontainer (see {doc}`/how-to/run-container`)
- Claude Code installed and authenticated
- STL mesh files for your robot (one per link, in millimetres)

## Step 1: Prepare STL files

Create a directory under `robots/` for your robot and place the STL files
inside `stl_files/`:

```
robots/MyRobot/
  stl_files/
    A0.stl    # base
    A1.stl    # link 1
    A2.stl    # link 2
    ...
```

Name the files so they sort in kinematic chain order (base to end-effector).
STL files should be in millimetres with Z pointing up.

## Step 2: Run auto-build-robot

The `make-robot` skill orchestrates the entire pipeline — from STL
analysis through chain.yaml generation to URDF verification and visual
comparison.

In Claude Code, invoke the skill:

```
/make-robot
```

Then tell Claude which robot to work on:

```
Build robots/MyRobot/
```

Claude will run through 5 stages:

1. **Analyze STLs** — detect connection points, geometry, and assembly hints
2. **Research manufacturer specs** — find DH parameters, joint limits online
3. **Infer kinematic chain** — write `chain.yaml` with topology, axes, limits
4. **Generate & verify URDF** — produce `robot.urdf` and check FK positions
5. **Visual comparison** — compare simulation against reference images

### chain.yaml key fields

| Field | Purpose |
|---|---|
| `links[].mesh` | Which STL file to use (stem name), or `null` for virtual links |
| `links[].visual_rpy` | Rotation to align STL coordinates with the link frame |
| `links[].visual_xyz` | Additional offset on top of auto-computed position (metres) |
| `joints[].axis` | Rotation axis: `[0,0,1]` for yaw/roll, `[0,1,0]` for pitch |
| `joints[].limits` | Joint angle limits in radians |
| `joints[].origin` | Override auto-computed joint position (metres) |

The resulting `chain.yaml` defines links, joints, axes, and limits. See the
Meca500-R3 example at `robots/Meca500-R3/chain.yaml` for reference.

## Step 3: Launch the simulator

```bash
uv run robot-arm-sim simulate robots
```

Open `http://localhost:8080` in a browser. You should see the 3D model with
joint sliders on the right.

Select your robot from the dropdown if not already selected.

Check that:

- All meshes are visible and roughly in the right place
- Sliders rotate the correct parts
- The overall shape resembles the real robot

The simulator includes several controls for inspecting the model:

- **Joint sliders** — drag to rotate each joint in real time
- **Show Labels** — display part names (blue) and joint names (red) as callout
  labels
- **Reset Joints** — return all sliders to zero degrees
- **ViewCube** — a 3D orientation widget in the corner of the viewport. Click a
  face (Front, Right, Top, etc.) to snap the camera to that named view.
- **Ortho / Persp** — toggle between orthographic and perspective projection.
  Orthographic mode removes perspective distortion, which is essential for
  accurate comparison against technical drawings.
- **Fit** — auto-zoom to fit the entire robot in the viewport.
- **Camera** — left-click drag to orbit, scroll to zoom, middle-click drag to
  pan.
- **Edit Connections** — enter an interactive mode to manually assign connection
  points by clicking on flat face markers. See below for details.

If the robot looks correct, you are done. If parts are misaligned, continue
to the visual tuning step below.

## Step 4: Fix connection points

Auto-detection of connection points works well for most parts, but can
fail on ambiguous geometries (e.g. wrist links with multiple cylindrical
surfaces). When detection picks the wrong face, the URDF generator
places joints incorrectly.

The **Edit Connections** mode lets you fix this interactively:

1. Click **Edit Connections** in the toolbar — the meshes go semi-transparent and
   coloured sphere markers appear at every flat face centroid on each part.
2. Use the **Proximal / Distal** toggle to choose which connection end to assign.
3. Click a yellow marker to assign it. Markers turn **green** (proximal) or
   **red** (distal). If the opposite end was already at the same face, they
   swap automatically.
4. Repeat for any parts that need correction.
5. Click **Save & Rebuild** — the updated connection points are written to the
   analysis YAML files (with `method: manual`) and the URDF is regenerated
   and reloaded.

This is particularly useful for robots like the UR5, where wrist parts have
lateral motor bores that confuse the auto-detector. Manual assignment lets
you pick the correct kinematic connection faces directly.

For a detailed explanation of connection points, centering modes, and how
they interact with `visual_rpy`, see {doc}`/how-to/edit-connection-points`.

## Step 5: Verify and commit

Once the simulation looks correct:

1. Run kinematics verification if you have a verification script:

   ```bash
   uv run python robots/MyRobot/verify_kinematics.py --json
   ```

2. Test a few joint poses to make sure rotations work as expected

3. Commit the robot directory:

   ```bash
   git add robots/MyRobot/
   git commit -m "Add MyRobot: STL analysis, chain.yaml, and URDF"
   ```

## Summary of skills used

| Skill | Purpose | Invocation |
|---|---|---|
| `make-robot` | Full pipeline: analyze, infer chain, generate, verify | `/make-robot` |
| `control-simulator` | Camera control, part visibility, connection editing | `/control-simulator` |

All skills edit `chain.yaml` and regenerate — they never touch `robot.urdf`
directly.
