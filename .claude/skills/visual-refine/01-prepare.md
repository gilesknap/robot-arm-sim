# Prepare for Visual Refinement

## Goal

Ensure the simulator is showing the current URDF and gather reference
material for visual comparison. Does NOT modify chain.yaml — works
with whatever state it is currently in.

## Steps

### 1. Regenerate URDF from current chain.yaml

```bash
uv run python -m robot_arm_sim generate robots/<name>/
```

Check the output for:
- FK validation — joint distances should match DH parameters
- "DH-fallback CPs" messages — these links had no auto-detected
  connection points and will likely need more correction
- "auto visual_rpy" messages — these links had mesh orientation
  auto-corrected

### 2. Gather reference images

If not already done, web search for manufacturer reference images:
- `"<robot name> dimensions drawing"`
- `"<robot name> technical specification"`
- `"<robot name> zero configuration"`
- `"<robot name> DH parameters diagram"`

Open reference images in a browser tab for side-by-side comparison.
Look for images showing the robot at zero/home configuration with
dimensions labelled.

### 3. Start simulator and set up view

Ensure the simulator is running:

```bash
uv run python -m robot_arm_sim simulate robots/ --port 8080
```

Navigate to `http://localhost:8080/<robot-name>` in the browser.

Set up the view:
- Click **Reset Joints** to set zero configuration
- Click **Frames** to show coordinate axes at each joint
- Click **Transparent** to see axes through mesh surfaces
- Click **Fit** to frame the robot in the viewport

### Gate

- URDF generates without errors
- FK validation shows correct inter-joint distances
- Simulator is running and showing the robot with frames visible
- Reference images available for comparison
