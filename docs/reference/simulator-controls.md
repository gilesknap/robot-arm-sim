# Simulator controls

Complete reference for all controls in the robot-arm-sim web simulator,
launched via `robot-arm-sim simulate`.

## Robot selector

When the robots directory contains multiple robots (each with a `robot.urdf`),
a dropdown at the top of the page lets you switch between them. You can also
navigate directly to a robot via its URL path (e.g. `/Meca500-R3`).

To choose which robot is selected by default, create a `.default` file in
the robots directory containing the robot folder name.

## Toolbar buttons

The toolbar runs along the bottom of the 3D viewport.

| Button | Description |
|--------|-------------|
| **Labels** | Toggle part name labels (blue, left-aligned) and joint name labels (red, right-aligned) as callout lines in the viewport |
| **Frames** | Show XYZ coordinate-frame axes at each joint origin |
| **Transparent** | Set all meshes to 25 % opacity so internal structure is visible |
| **Connections** | Show connection point markers as spheres on each part |
| **Edit Connections** | Enter interactive connection assignment mode (see below) |
| **Reload URDF** | Re-parse the URDF file and refresh the scene. Camera position, joint angles, and visibility state are preserved via session storage |
| **Screenshot** | Download a PNG image of the current 3D viewport |
| **Stop Simulator** | Gracefully shut down the server (hidden in cluster mode). Shows a confirmation dialog before stopping |
| **Info** | Open the About dialog showing version, project description, and a toolbar button summary |

## Control panel (right sidebar)

An **FK / IK** radio toggle at the top of the panel switches between forward
and inverse kinematics modes.

### FK mode (default)

One slider per revolute or continuous joint. Drag to set the joint angle;
the value is shown in degrees. The 3D model updates in real time as the
server computes forward kinematics and broadcasts transforms over WebSocket.

### IK mode

Six sliders control the desired end-effector pose:

| Slider | Unit | Range |
|--------|------|-------|
| X | mm | -300 to +300 |
| Y | mm | -300 to +300 |
| Z | mm | 0 to +500 |
| Rx, Ry, Rz | degrees | -180 to +180 |

Moving any IK slider triggers the damped least-squares IK solver. If a
solution is found, the FK sliders update automatically to show the
resulting joint angles.

### Reset buttons

| Button | Description |
|--------|-------------|
| **Reset Joints** | Return all joint angles to zero (home pose) |
| **Reset View** | Restore all part visibility, switch to orthographic front view, and fit the robot in the viewport |

### End-effector readout

Below the sliders, the current end-effector position (X, Y, Z in mm) and
orientation (Rx, Ry, Rz in degrees) are displayed and update in real time.

### Visible Parts

A set of toggle chips — one per link plus an **All** chip — controls which
meshes are rendered. Visibility state is preserved across URDF reloads.

## ViewCube

A 3D orientation widget in the top-right corner of the viewport.

- **Six labelled faces** — click FRONT, BACK, LEFT, RIGHT, TOP, or BOTTOM
  to snap the camera to that orthographic view.
- **Axis indicators** — X (red), Y (green), Z (blue). Clicking an axis
  indicator snaps to the corresponding axis-aligned view.
- The cube rotates in sync with the main camera.

Below the ViewCube:

| Button | Description |
|--------|-------------|
| **Persp / Ortho** | Toggle between perspective and orthographic projection |
| **Fit** | Auto-zoom so the entire robot fills the viewport |

## Camera controls

| Action | Input |
|--------|-------|
| Orbit | Left-click drag (or two-finger drag) |
| Zoom | Scroll wheel |
| Pan | Right-click drag (or middle-click drag) |

## Edit Connections mode

Activated by the **Edit Connections** toolbar button. Used to manually assign
connection points and adjust mesh placement.

When entering Edit Connections mode:

- All meshes become semi-transparent (25 % opacity)
- Coordinate frames are automatically enabled
- The camera locks to orthographic views (drag on empty space to swipe
  between adjacent views)
- The toolbar changes: **Edit Connections** becomes **Exit Edit**

### Mode buttons

| Mode | Colour | Description |
|------|--------|-------------|
| **Move Part** | amber | Drag a part to reposition, or use arrow keys (0.1 mm per press, Shift for 2 mm steps) |
| **Proximal Centred** | blue | Click a mesh surface to place a `center`-mode proximal marker |
| **Proximal Surface** | green | Click a mesh surface to place a `surface`-mode proximal marker |
| **Distal** | red | Click a mesh surface to place a distal marker |

### Action buttons

| Button | Description |
|--------|-------------|
| **Undo** / **Redo** | Step back and forward through edits |
| **Rot X** / **Rot Y** / **Rot Z** | Rotate selected part 90° around that axis |
| **Remove Part Conns** | Remove connection points for the selected part only, baking current placement into `visual_xyz` |
| **Remove Connections** | Remove all connection points for all parts, baking current placement into `visual_xyz` |
| **Save & Rebuild** | Write changes to analysis YAML and chain.yaml, regenerate URDF, and reload |

**Exit Edit** discards all unsaved changes — connection assignments, part
moves, and rotations are restored to their state when edit mode was entered.

## Session state persistence

The simulator uses browser `sessionStorage` to preserve state across URDF
reloads and page refreshes (via `beforeunload`):

- Per-part visibility toggles
- Joint angles (radians)
- Camera position, target, up vector, and projection type

State is restored approximately 3 seconds after page load to allow the
Three.js scene to initialise.
