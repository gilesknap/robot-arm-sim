# Add a new robot

This guide walks through the complete process of taking a set of STL mesh
files for a robot arm and producing a working interactive simulation. It
covers using Claude Code with the project's built-in skills to automate much
of the work, and using the claude-in-chrome MCP server for visual refinement.

## Prerequisites

- A working devcontainer (see {doc}`/how-to/run-container`)
- Claude Code installed and authenticated
- STL mesh files for your robot (one per link, in millimetres)
- Optionally: the [claude-in-chrome](https://github.com/anthropics/claude-in-chrome)
  MCP server for visual tuning (see Setting up claude-in-chrome below)

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

The `auto-build-robot` skill orchestrates the entire pipeline — from STL
analysis through chain.yaml generation to URDF verification and visual
comparison.

In Claude Code, invoke the skill:

```
/auto-build-robot
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
uv run robot-arm-sim simulate robots/MyRobot/
```

Open `http://localhost:8080` in a browser. You should see the 3D model with
joint sliders on the right. Check that:

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
- **Edit Bores** — enter an interactive mode to manually assign bore connection
  points by clicking on flat face markers. See below for details.

If the robot looks correct, you are done. If parts are misaligned, continue
to the visual tuning step below.

### Fixing bore connection points with Edit Bores

Auto-detection of bore connection points works well for most parts, but can
fail on ambiguous geometries (e.g. wrist links with multiple cylindrical
surfaces). When bore detection picks the wrong face, the URDF generator
places joints incorrectly.

The **Edit Bores** mode lets you fix this interactively:

1. Click **Edit Bores** in the toolbar — the meshes go semi-transparent and
   coloured sphere markers appear at every flat face centroid on each part.
2. Use the **Proximal / Distal** toggle to choose which bore end to assign.
3. Click a yellow marker to assign it. Markers turn **green** (proximal) or
   **red** (distal). If the opposite end was already at the same face, they
   swap automatically.
4. Repeat for any parts that need correction.
5. Click **Save & Rebuild** — the updated bore points are written to the
   analysis YAML files (with `method: manual`) and the URDF is regenerated
   and reloaded.

This is particularly useful for robots like the UR5, where wrist parts have
lateral motor bores that confuse the auto-detector. Manual assignment lets
you pick the correct kinematic bore faces directly.

## Step 4: Visual tuning with reference images

This is where the project's visual skills and claude-in-chrome come together
to iteratively refine the URDF until the simulation matches reality.

### Setting up claude-in-chrome

The `claude-in-chrome` MCP server lets Claude Code control a Chrome browser —
navigating pages, clicking, taking screenshots, and running JavaScript. This
project uses it to interact with the NiceGUI simulator.

1. Install the claude-in-chrome extension in Chrome following the
   instructions at its repository
2. The project's `.claude/settings.json` already includes the permission
   `mcp__claude-in-chrome__*`, so Claude Code can use it without prompting

Once configured, Claude can:

- Open the simulator in a Chrome tab
- Set camera angles via JavaScript (using the `zoom-rotate-camera` skill)
- Set joint poses via slider automation
- Take screenshots for comparison against reference photos
- Snap to orthographic named views via the ViewCube for precise comparison
- Click the Fit button to auto-zoom the robot into the viewport

### Using the manufacturer-comparison skill

The `manufacturer-comparison` skill is the most systematic approach to visual
tuning. It automates finding dimensioned technical drawings from the
manufacturer, setting up orthographic comparison views, and iterating with
measurable convergence criteria.

```
/manufacturer-comparison
```

This skill follows a six-phase workflow:

1. **Find reference images** — searches for dimensioned technical drawings,
   CAD renders, and product photos. Saves them to `robots/<name>/images/` for
   reuse across sessions.
2. **Map manufacturer views** — determines how the manufacturer's labelled
   views (front, side, top) correspond to the simulator's ViewCube faces, since
   conventions often differ.
3. **Set up orthographic comparison** — snaps the camera to the matching
   ViewCube face in orthographic projection, ensuring no parallax errors when
   comparing against technical drawings.
4. **Part-by-part comparison** — works base-to-tip, checking link lengths,
   silhouettes, joint positions, and gaps against the manufacturer's dimensions.
5. **Adjust and iterate** — edits `chain.yaml`, regenerates the URDF, and
   re-compares until each link is within **2mm** of manufacturer specs.
6. **Multi-view validation** — verifies accuracy across multiple orthographic
   views and at non-zero joint angles to confirm rotation centres are correct.

This is the recommended skill when manufacturer datasheets or technical drawings
are available, since it uses actual dimensions as ground truth rather than
relying solely on visual appearance.

### Common adjustments

| Symptom | Fix in chain.yaml |
|---|---|
| Part too high or low | Adjust Z in `visual_xyz` on the link |
| Part shifted sideways | Adjust X or Y in `visual_xyz` |
| Part rotated wrong | Adjust `visual_rpy` on the link |
| Gap between parts | Reduce origin distance or adjust `visual_xyz` |
| Parts overlapping | Increase origin distance or adjust `visual_xyz` |
| Joint rotates wrong point | Adjust `visual_xyz` to centre bore on joint |

### Important rules

- **Never edit `robot.urdf` directly** — always edit `chain.yaml` and
  regenerate with `robot-arm-sim generate`
- **Restart the simulator process** after regenerating the URDF — refreshing
  the browser is not enough:

  ```bash
  pkill -9 -f "robot-arm-sim simulate"
  sleep 3
  uv run robot-arm-sim simulate robots/MyRobot/
  ```

- **Work from base to tip** — fix the base link first, then link 1, then
  link 2, and so on. Errors compound along the kinematic chain.
- **One change at a time** — adjust one parameter, regenerate, and compare
  before making the next change.

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
| `auto-build-robot` | Full pipeline: analyze, infer chain, generate, verify | `/auto-build-robot` |
| `assembly-reasoning` | Generate `chain.yaml` from analysis data (standalone) | `/assembly-reasoning` |
| `manufacturer-comparison` | Compare against manufacturer technical drawings | `/manufacturer-comparison` |
| `zoom-rotate-camera` | Position the 3D camera programmatically | `/zoom-rotate-camera` |

All skills edit `chain.yaml` and regenerate — they never touch `robot.urdf`
directly.
