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
   existing connection markers appear (green = proximal surface,
   blue = proximal centred, red = distal).

3. Select a mode from the toolbar:

   - **Proximal Centred** (blue) — places a `center`-mode proximal marker.
   - **Proximal Surface** (green) — places a `surface`-mode proximal marker.
   - **Distal** (red) — places a distal marker (tells gap-closing which parent
     surface the child should meet).
   - **Move Parts** (amber) — drag a part or use arrow keys (0.1 mm per press).

   See {ref}`when-to-use-each-mode` below for guidance on `surface` vs `center`.

4. Click directly on any mesh surface to place the marker at the click point.
   The face normal at that point becomes the marker's axis.

5. Use the **Show All** checkbox to see markers for all parts at once.

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

1. **Place the markers correctly** — use Edit Connections and click directly on
   the correct mesh surfaces to assign proximal and distal markers. This tells
   the pipeline where the joint axes are, and gives it the axis directions from
   the face normals.

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

**When to place a distal marker:** the distal marker tells the gap-closing pass
which parent surface the child link should be shifted to meet. Auto-detected
distals are usually correct; you only need to place one manually when
auto-detection picked the wrong face.

## Fine-tuning with `visual_xyz`

After connection points place the mesh approximately, you can apply a
small local nudge via `visual_xyz` in `chain.yaml`. This offset is applied
after all other pipeline computation and never affects other links.

You can set it by:

- Using **Move Parts** mode in the simulator, then clicking **Save & Rebuild**
- Editing `chain.yaml` directly and running `generate`

See {ref}`the pipeline explanation <key-rules>` for details on what
propagates and what doesn't.

## Removing connection points

If auto-detected connections are consistently wrong and you prefer full manual
control over mesh placement, you can remove all connection points and work
entirely with `visual_xyz` offsets.

**When to use it:** auto-detection places parts incorrectly for most joints, and
fixing each one individually is slower than positioning parts manually.

**How:** in the simulator, enter **Edit Connections** mode and click
**Remove Connections** (red button). This:

1. **Bakes current placement into `visual_xyz`** — each link's current URDF
   visual origin is saved as `visual_xyz` in `chain.yaml`, preserving exact
   part positions.
2. **Clears all `connection_points`** from every `analysis/*.yaml` file.
3. **Regenerates the URDF** — with no connection points the pipeline returns
   `[0,0,0]` for the visual origin, so the `visual_xyz` values produce
   identical output.

Parts do not move — the visual result is the same before and after.

After removal, fine-tune placement with **Move Parts** mode or by editing
`visual_xyz` values directly in `chain.yaml`. Rotation editing
(`visual_rpy`) is planned for a future release.
