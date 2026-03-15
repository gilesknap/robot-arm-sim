# Edit chain.yaml and analysis YAML directly

How to adjust connection points, visual alignment, and joint parameters by
editing YAML files and regenerating the URDF — without the simulator UI.

For the interactive approach, see {doc}`edit-connection-points`.
For the full `chain.yaml` field reference, see {doc}`/reference/chain-yaml`.

## Edit connection points in analysis YAML

Each part's analysis file (e.g. `robots/MyRobot/analysis/A1.yaml`) contains a
`connection_points` section. You can edit the `position`, `axis`, `end`, or
`centering` fields directly. Set `method: manual` to prevent the analyzer
from overwriting your changes on the next run.

After editing, regenerate the URDF:

```bash
uv run robot-arm-sim generate robots/MyRobot/
```

## Fine-tune with `visual_xyz`

After connection points place the mesh approximately, apply a small local
nudge via `visual_xyz` in `chain.yaml`. This offset is applied after all
other pipeline computation and never affects other links.

```yaml
- name: link_2
  mesh: A2
  visual_xyz: [0, 0, -0.003]   # shift 3mm down
```

Then regenerate:

```bash
uv run robot-arm-sim generate robots/MyRobot/
```

See {ref}`the pipeline explanation <key-rules>` for details on what
propagates and what doesn't.

## Adjust mesh orientation with `visual_rpy`

When a part's STL coordinate frame doesn't align with the link frame
convention (Z-up along joint axis), add a `visual_rpy` rotation in
`chain.yaml`:

```yaml
- name: link_3
  mesh: A3
  visual_rpy: [0, -1.5708, 0]   # -90° pitch rotates X to Z
```

## Adjust joint parameters

Joint limits, axes, and types are set in the `joints` section of
`chain.yaml`. Common edits:

```yaml
- name: joint_2
  type: revolute
  parent: link_1
  child: link_2
  axis: [0, 1, 0]              # pitch axis
  limits: [-2.356, 2.356]      # ±135°
```

See {doc}`/reference/chain-yaml` for the complete field reference.

## Verify changes

After any edit, regenerate and check the result:

```bash
uv run robot-arm-sim generate robots/MyRobot/
uv run robot-arm-sim simulate robots/MyRobot/
```

To diff against the previous URDF:

```bash
diff <(git show HEAD:robots/MyRobot/robot.urdf) robots/MyRobot/robot.urdf
```
