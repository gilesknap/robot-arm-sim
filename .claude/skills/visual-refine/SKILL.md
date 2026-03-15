# Refine Robot Visual Alignment

Refine mesh placement in the simulator using visual reasoning. Works
base-upward through each link, comparing the simulator against
manufacturer reference images to determine `visual_xyz` and
`visual_rpy` corrections.

This is step 05 of the make-robot pipeline. It assumes chain.yaml
and robot.urdf already exist with correct FK. It can be invoked on a
fresh post-generate robot or on one that is already partially refined.

## When to Use

- After `/make-robot` produces a URDF that has correct FK but
  visually misaligned meshes
- When a robot has gaps between parts at joints
- When parts are rotated incorrectly in the simulation
- To continue refining a partially-corrected robot
- The user says "refine", "fix visual", "align meshes", etc.

## Prerequisites

- A working `chain.yaml` and `robot.urdf` with correct FK
- The simulator running (`uv run robot-arm-sim simulate robots/`)
- claude-in-chrome MCP server for browser automation
- Manufacturer reference images (fetched during make-robot step 02)

## Pipeline

```
prepare ──> per-link-refine (base to tip) ──> validate
```

### 1. Prepare (read `01-prepare.md`)

- Regenerate URDF from current chain.yaml
- Fetch manufacturer reference images if not already available
- Start the simulator and navigate to the robot
- Enable Frames + Transparent, reset joints to zero

### 2. Per-Link Refinement (read `02-per-link-refine.md`)

For each link from base to tip:
- Hide links above the current one
- Compare frame axis alignment with the mesh
- Compare against reference images
- Estimate and apply `visual_xyz` / `visual_rpy` corrections
- Regenerate and re-check

### 3. Validate

- Show all links, check overall appearance
- Compare end effector position against manufacturer specs
- Run FK validation
- Diff against main (if available) to sanity-check magnitudes
- Run `uv run tox -p`

## Key Principles

- **Frames are ground truth** — the RGB axes show where each link's
  proximal end should sit
- **Work base-upward** — errors compound along the chain
- **One link at a time** — isolate with part visibility controls
- **Never modify joint origins** — only `visual_xyz`/`visual_rpy`
- **Small corrections first** — most auto-placed meshes need <10mm nudges;
  if a correction is >50mm, investigate whether the connection point
  detection failed
- **Iterate** — apply correction, regenerate, re-check, adjust
- **Reference images** — always compare against manufacturer drawings,
  not just "looks right"
- **Resumable** — can be invoked multiple times on the same robot to
  continue refining; existing `visual_xyz`/`visual_rpy` values are
  adjusted, not discarded
