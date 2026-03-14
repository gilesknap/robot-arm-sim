# Simulator controls

Complete reference for all controls in the robot-arm-sim web simulator,
launched via `robot-arm-sim simulate`.

## Robot selector

When the robots directory contains multiple robots (each with a `robot.urdf`),
a dropdown at the top of the page lets you switch between them.

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
| **Reset** | Return all joint angles to zero (home pose) |
| **Stop Simulator** | Gracefully shut down the server (hidden in cluster mode) |

## Control panel (right sidebar)

### FK mode (default)

One slider per revolute or continuous joint. Drag to set the joint angle;
the value is shown in degrees. The 3D model updates in real time as the
server computes forward kinematics and broadcasts transforms over WebSocket.

### IK mode

Six sliders control the desired end-effector pose:

| Slider | Unit | Range |
|--------|------|-------|
| X, Y, Z | mm | -300 to +300 |
| Rx, Ry, Rz | degrees | -180 to +180 |

Moving any IK slider triggers the damped least-squares IK solver. If a
solution is found, the FK sliders update automatically to show the
resulting joint angles.

### End-effector readout

Below the sliders, the current end-effector position (X, Y, Z in mm) and
orientation (Rx, Ry, Rz in degrees) are displayed and update in real time.

### Visible Parts

A set of toggle chips — one per link plus an **All** chip — controls which
meshes are rendered. Visibility state is preserved across URDF reloads.

## ViewCube

A 3D orientation widget in the top-right corner of the viewport.

- **Six labelled faces** — click Front, Back, Left, Right, Top, or Bottom
  to snap the camera to that orthographic view.
- **Axis indicators** — X (red), Y (green), Z (blue).
- The cube rotates in sync with the main camera.

Below the ViewCube:

| Button | Description |
|--------|-------------|
| **Persp / Ortho** | Toggle between perspective and orthographic projection |
| **Fit** | Auto-zoom so the entire robot fills the viewport |

## Camera controls

| Action | Input |
|--------|-------|
| Orbit | Right-click drag (or two-finger drag) |
| Zoom | Scroll wheel |
| Pan | Middle-click drag |

## Edit Connections mode

Activated by the **Edit Connections** toolbar button. Used to manually assign
flat-face connection points as proximal or distal connections.

When active:

1. All meshes become semi-transparent (25 % opacity).
2. Detected flat faces are shown as coloured sphere markers:
   - **Yellow** — unassigned
   - **Green** — proximal (parent-side)
   - **Red** — distal (child-side)
3. A **Proximal / Distal** toggle at the top selects the assignment mode.
4. Click a sphere marker to assign it.
5. Click **Save & Rebuild** to write connection assignments to the analysis
   YAML files, propagate connection axes to `chain.yaml`, regenerate the
   URDF, and reload the simulator.

## Session state persistence

The simulator uses browser `sessionStorage` to preserve state across URDF
reloads (triggered by the Reload URDF button):

- Per-part visibility toggles
- Joint angles (radians)
- Camera position, target, up vector, and projection type

State is restored approximately 3 seconds after page load to allow the
Three.js scene to initialise.
