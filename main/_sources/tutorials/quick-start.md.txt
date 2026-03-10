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

Your browser opens at `http://localhost:8080` with a 3D model of the
Meca500-R3 collaborative robot.

## Explore the controls

- **Joint sliders** (right panel) — drag any slider to rotate that joint.
  The model updates in real time.
- **Show Labels** — click the button below the viewport to display part names
  (blue, left) and joint names (red, right) as callout labels.
- **Reset Joints** — returns all sliders to zero degrees.
- **Camera** — left-click drag to orbit, scroll to zoom, middle-click drag
  to pan.

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
