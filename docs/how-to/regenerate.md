# Regenerate analysis and URDF

After editing STL files or modifying `chain.yaml`, you need to re-run the
analysis and regenerate the URDF outside the full `/make-robot` pipeline.
This page covers the standalone commands for that workflow.

## When to re-analyze

Re-run analysis when you have:

- Replaced or edited STL mesh files
- Deleted the `analysis/` directory to start fresh
- Changed STL orientation or scale

If you only edited `chain.yaml` (axes, limits, visual offsets), skip ahead to
{ref}`regenerate-urdf` — analysis is not needed.

## Re-run analysis

```bash
uv run robot-arm-sim analyze robots/MyRobot/
```

This regenerates the per-part YAML files and renders under `analysis/`.
Existing manual connection points are preserved by default.

(override-manual)=
### Resetting manual connection points

To discard all manual placements and re-detect from scratch:

```bash
uv run robot-arm-sim analyze robots/MyRobot/ --override-manual
```

Use this when the underlying STL geometry has changed enough that previous
manual assignments no longer apply.

(regenerate-urdf)=
## Regenerate the URDF

```bash
uv run robot-arm-sim generate robots/MyRobot/
```

This reads `chain.yaml` and the `analysis/` data to produce `robot.urdf`.
See {doc}`/reference/cli` for additional options such as `--output` and
specifying an alternate chain file.

## Diff against main

Always diff the generated URDF to confirm changes are intentional:

```bash
diff <(git show main:robots/MyRobot/robot.urdf) robots/MyRobot/robot.urdf
```

Every changed line should have a clear justification. If you maintain multiple
robots, repeat this for each one that may have been affected.

## Verify in the simulator

Launch the simulator to visually confirm the result:

```bash
uv run robot-arm-sim simulate robots/
```

Open `http://localhost:8080` and check that meshes are correctly placed, joints
rotate the expected parts, and the overall shape matches the real robot. See
{doc}`/how-to/add-a-robot` for a full checklist of what to look for.
