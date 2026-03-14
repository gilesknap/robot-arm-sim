# Edit connection points

How to assign and adjust connection point markers to fix mesh placement
in the URDF.

For background on how connection points, centering modes, and the pipeline
work, see {doc}`/explanations/urdf-generation-pipeline`.

## Using the simulator UI

1. Launch the simulator:

   ```bash
   uv run robot-arm-sim simulate robots/MyRobot/
   ```

2. Click **Edit Connections** in the toolbar — meshes go semi-transparent and
   coloured sphere markers appear at every flat face centroid.

3. Use the **Proximal / Distal** toggle to choose which end to assign.

4. Select a **centering mode** from the dropdown (`surface` or `center`).
   See {ref}`when-to-use-each-mode` below for guidance.

5. Click a yellow marker to assign it. Markers turn **green** (proximal) or
   **red** (distal).

6. Click **Save & Rebuild** — this writes changes to three places:

   - **`analysis/*.yaml`** — updated connection points (with `method: manual`)
   - **`chain.yaml`** — if you used Move Parts, the accumulated offset is
     saved as `visual_xyz`; if you only edited connections without moving,
     any stale `visual_xyz` is cleared
   - **`robot.urdf`** — regenerated from the updated analysis and chain data

## Editing analysis YAML directly

You can also edit the `connection_points` section of any analysis YAML file
directly. After editing:

```bash
uv run robot-arm-sim generate robots/MyRobot/
```

This regenerates the URDF from the updated analysis data.

## Fixing jumbled parts

When auto-detection picks the wrong connection points, parts end up in the
wrong position *and* orientation. Fix this in two steps:

1. **Place the markers correctly** — use Edit Connections to assign proximal
   and distal markers to the right faces. This tells the pipeline where the
   joint axes are, and gives it the axis directions from the face normals.

2. **Set `visual_rpy` if the axes aren't aligned** — if a part's proximal
   face is not perpendicular to the joint axis (i.e. the STL mesh
   coordinates don't naturally align with the link frame), you need a
   `visual_rpy` rotation in `chain.yaml` to bring them into alignment.

For a typical straight part where proximal and distal are on parallel faces
along the same axis, `visual_rpy` can stay at `[0, 0, 0]`. For L-shaped or
angled parts, you need a rotation.

**Work from base to tip.** Surface gap closure references the parent's
distal position, so fixing a parent link's connections first makes it easier
to verify child links visually.

(when-to-use-each-mode)=
## When to use each mode

| Geometry | Mode | Why |
|---|---|---|
| Flat mating faces (most joints) | `surface` | Marker on face surface; gap-closing shifts along axis to meet parent |
| Through-bores / shaft holes | `center` | Marker at bore center; pipeline averages with opposite face, snaps to axis |
| Barrel bores (large concentric cylinders) | `center` | Bore center defines the rotation axis; snapped onto axis line |
| Shallow surface features | `surface` | No opposite face to average with; surface position is sufficient |

**Rule of thumb:** if the joint rotation axis passes *through* a bore, use
`center`. If the joint is at a flat face where two parts mate, use `surface`.

## Fine-tuning with `visual_xyz`

After connection points place the mesh approximately, you can apply a
small local nudge via `visual_xyz` in `chain.yaml`. This offset is applied
after all other pipeline computation and never affects other links.

You can set it by:

- Using **Move Parts** mode in the simulator, then clicking **Save & Rebuild**
- Editing `chain.yaml` directly and running `generate`

See {ref}`the pipeline explanation <key-rules>` for details on what
propagates and what doesn't.
