# Quick start

This tutorial gets the included Meca500-R3 robot running in your browser in
under a minute.

## Prerequisites

- Python 3.11+
- [uv](https://docs.astral.sh/uv/) package manager

## Clone and install

```bash
git clone https://github.com/gilesknap/robot-arm-sim.git
cd robot-arm-sim
uv sync
```

## Launch the simulator

```bash
uv run robot-arm-sim simulate robots/Meca500-R3/
```

The simulator starts on port 8080. Open <http://localhost:8080> in your
browser to see a 3D model of the Meca500-R3 collaborative robot.

## Explore the controls

- **Joint sliders** (right panel) — drag any slider to rotate that joint.
  The model updates in real time.
- **Labels** — click the toolbar button to display part names (blue, left)
  and joint names (red, right) as callout labels.
- **Reset Joints** — returns all sliders to zero degrees.
- **ViewCube** — a 3D orientation widget in the corner of the viewport. Click
  a face (Front, Right, Top, etc.) to snap to that view.
- **Persp** — toggle between perspective and orthographic camera projection.
- **Fit** — auto-zoom so the robot fills the viewport.
- **Camera** — left-click drag to orbit, scroll to zoom, right-click drag
  to pan.

There are more controls not covered here — **IK mode** for controlling the
end-effector pose directly, **Edit Connections** for adjusting connection
points, **Frames**, **Transparent**, and **Screenshot** buttons. If multiple
robots are available, a dropdown selector appears at the top of the page. See
{doc}`/reference/simulator-controls` for the full reference.

## Try some poses

Set joint 2 to 45° and joint 3 to -45° to see the arm reach forward.
Set joint 1 to 90° to rotate the whole arm on its base.

## What's in the robot directory

```
robots/Meca500-R3/
├── stl_files/      # STL meshes (one per link)
├── analysis/       # Generated geometry analysis (YAML + renders)
├── chain.yaml      # Kinematic chain specification
└── robot.urdf      # Generated URDF model
```

## Next steps

- {doc}`/how-to/add-a-robot` — add your own robot
- {doc}`/explanations/architecture` — understand how the pipeline works
- {doc}`/explanations/building-with-claude` — how this project was built
