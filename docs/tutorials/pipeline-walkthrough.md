# Pipeline walkthrough

In the {doc}`quick-start` you launched a pre-built robot in the simulator.
Now rebuild the Meca500-R3 from scratch to understand the three pipeline
stages: **analyze**, **generate**, and **simulate**.

## Prerequisites

- You have completed the {doc}`quick-start` tutorial (clone, install, run).

## Stage 1 -- Analyze the STL files

The `analyze` command inspects every STL mesh in `stl_files/` and writes
per-part geometry data to an `analysis/` directory.

```bash
uv run robot-arm-sim analyze robots/Meca500-R3/
```

Once finished, open one of the generated YAML files to see what the analyzer
found:

```{literalinclude} ../../robots/Meca500-R3/analysis/A0.yaml
:language: yaml
:lines: 1-16
:caption: robots/Meca500-R3/analysis/A0.yaml (first 16 lines)
```

Each file records the part's bounding box, volume, connection points (where
links attach), and notable geometric features such as large flat faces.
The analyzer also writes multi-angle renders of each part for visual
inspection.

## Stage 2 -- Generate the URDF

The `generate` command combines the analysis data with the kinematic chain
specification (`chain.yaml`) to produce a URDF file.

```bash
uv run robot-arm-sim generate robots/Meca500-R3/
```

This writes `robots/Meca500-R3/robot.urdf`. The URDF describes every link's
visual mesh, every joint's type and axis, and the transforms that connect them.
You can open the file to verify link names and joint limits match the real
robot's datasheet.

## Stage 3 -- Simulate

Launch the simulator to verify the result:

```bash
uv run robot-arm-sim simulate robots/Meca500-R3/
```

Open <http://localhost:8080> and drag the joint sliders. If every joint rotates
around the correct axis and the meshes line up without gaps, the pipeline
succeeded.

## What you learned

The pipeline has three discrete stages. **Analyze** extracts geometry and
connection points from raw STL meshes. **Generate** merges that geometry data
with a kinematic chain spec to produce a standards-compliant URDF. **Simulate**
loads the URDF into an interactive 3D viewer for verification. Each stage's
output feeds the next, so you can inspect or override intermediate files before
moving on.

## Next steps

- {doc}`/how-to/add-a-robot` -- add your own robot using the same pipeline.
- {doc}`/explanations/architecture` -- deeper look at how the stages connect.
