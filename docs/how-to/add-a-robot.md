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

## Step 2: Run the analysis

```bash
uv run robot-arm-sim analyze robots/MyRobot/
```

This produces `robots/MyRobot/analysis/` containing:

- Per-part YAML files with geometry, features, and detected connection points
- Rendered PNG views of each part (front, side, top, isometric)
- A `summary.yaml` with assembly hints

Check the output makes sense. Each part YAML should have `connection_points`
entries showing detected bore/shaft centres.

## Step 3: Generate chain.yaml with the assembly-reasoning skill

This is where Claude does the semantic reasoning — figuring out which parts
connect, what type of joint they form, and what the rotation axes are.

In Claude Code, invoke the skill:

```
/assembly-reasoning
```

Then tell Claude which robot to work on:

```
Generate a chain.yaml for robots/MyRobot/
```

Claude will:

1. Read the analysis YAMLs and connection point data
2. Search online for manufacturer specifications (DH parameters, joint limits)
3. Write `robots/MyRobot/chain.yaml` with the kinematic chain topology
4. Run `robot-arm-sim generate` to produce the URDF
5. Run verification to check FK positions

The resulting `chain.yaml` defines links, joints, axes, and limits. See the
Meca500-R3 example at `robots/Meca500-R3/chain.yaml` for reference.

### chain.yaml key fields

| Field | Purpose |
|---|---|
| `links[].mesh` | Which STL file to use (stem name), or `null` for virtual links |
| `links[].visual_rpy` | Rotation to align STL coordinates with the link frame |
| `links[].visual_xyz` | Additional offset on top of auto-computed position (metres) |
| `joints[].axis` | Rotation axis: `[0,0,1]` for yaw/roll, `[0,1,0]` for pitch |
| `joints[].limits` | Joint angle limits in radians |
| `joints[].origin` | Override auto-computed joint position (metres) |

## Step 4: Generate the URDF

If the skill didn't already do this:

```bash
uv run robot-arm-sim generate robots/MyRobot/ robots/MyRobot/chain.yaml
```

This reads the connection points from the analysis data, computes visual and
joint transforms, and writes `robots/MyRobot/robot.urdf`. It prints the
zero-config FK position of each joint for validation.

## Step 5: Launch the simulator

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

If the robot looks correct, you are done. If parts are misaligned, continue
to the visual tuning steps below.

## Step 6: Visual tuning with reference images

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

### Using the refine-with-image skill

The `refine-with-image` skill is a 10-step workflow for visual URDF
refinement. It is the most effective way to get a pixel-perfect simulation.

1. Find or take a reference photo of the real robot at a known pose (ideally
   zero config — all joints at zero degrees)

2. In Claude Code, invoke the skill:

   ```
   /refine-with-image
   ```

3. Claude will ask for:
   - **Image**: provide the file path or paste the image
   - **Pose**: what joint angles the robot is at in the photo
   - **View angle**: what direction the camera is looking from

4. Claude then:
   - Starts the simulator if needed
   - Opens it in Chrome via claude-in-chrome
   - Matches the camera angle and joint pose to the reference photo
   - Takes a screenshot and compares part-by-part from base to tip
   - Edits `chain.yaml` to fix misalignments
   - Regenerates the URDF and restarts the simulator
   - Repeats until the simulation matches the photo

A typical tuning session takes 2-4 iterations.

### Using the visual-urdf-tuning skill

The `visual-urdf-tuning` skill is an alternative when you don't have a
reference photo handy. It searches online for images of the robot and uses
those as reference:

```
/visual-urdf-tuning
```

This skill follows the same compare-diagnose-fix cycle but starts by finding
reference images via web search.

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

## Step 7: Verify and commit

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
| `assembly-reasoning` | Generate `chain.yaml` from analysis data | `/assembly-reasoning` |
| `visual-urdf-tuning` | Find reference images and tune URDF | `/visual-urdf-tuning` |
| `refine-with-image` | Tune URDF against a provided reference photo | `/refine-with-image` |
| `manufacturer-comparison` | Compare against manufacturer technical drawings | `/manufacturer-comparison` |
| `zoom-rotate-camera` | Position the 3D camera programmatically | `/zoom-rotate-camera` |

All skills edit `chain.yaml` and regenerate — they never touch `robot.urdf`
directly.
