# 06 — Refine Link

Fix connection points for one link at a time, working base-to-tip. Use reference images + the simulator's Edit Connections mode to visually identify and click the correct mating surfaces.

## Input

- Link index N (0-based), robot name
- Simulator on `localhost:8080`, claude-in-chrome MCP for screenshots/clicks
- Reference images already collected (skill 05)

## Steps

### 1. Study the part against reference images

1. Set **Show up to** slider to link N
2. Use `zoom-rotate-camera` to get clear views of the part from multiple angles
3. Screenshot via browser MCP
4. Compare against manufacturer reference images — identify:
   - Where does this part connect to its **parent** (proximal)?
   - Where does this part connect to its **child** (distal)?
   - What kind of surface is it? (flat flange → `surface` mode, through-bore → `center` mode)

### 2. Assign connection points via Edit Connections

1. Click **Edit Connections** in the toolbar — meshes go semi-transparent
2. Select **Proximal** or **Distal** toggle
3. Set **centering mode** (`surface` or `center`)
4. Use `zoom-rotate-camera` to snap to an **orthogonal view** where the target surface faces the camera — this ensures accurate click placement (perspective views distort positions)
5. **Click directly on the mesh surface** at the mating face — the click handler captures the exact position and face normal
6. Repeat for the other end (proximal/distal)
7. Click **Save & Rebuild** — writes the analysis YAML (with `method: manual`) and regenerates the URDF automatically

### 3. Verify the result

1. Exit Edit Connections mode
2. Screenshot from orthogonal views (FRONT, RIGHT minimum), compare against reference
3. Check: no gaps, no overlaps, silhouette matches reference

### 4. Verify kinematics

```bash
uv run python robots/<name>/verify_kinematics.py --json
```

All joints must pass within 2mm.

### 5. Diff the URDF

```bash
diff <(git show HEAD:robots/<name>/robot.urdf) robots/<name>/robot.urdf
```

Confirm only visual origins changed — joint origins must be unchanged.

### 6. If wrong, iterate

Return to step 1. Re-examine the reference images — did you pick the wrong surface? Wrong centering mode? Re-enter Edit Connections and try again.

## Centering mode guide

| Geometry | Mode | Why |
|---|---|---|
| Flat mating flange | `surface` | Marker on the face; gap-closing applies normally |
| Through-bore / shaft hole | `center` | Pipeline averages with opposite face for precise axis centering |
| Large concentric cylinder | `center` | Bore center defines the rotation axis |

**Rule of thumb:** if the joint rotation axis passes *through* a bore, use `center`. If the joint is at a flat face where two parts mate, use `surface`.

## Key Rules

- **NEVER modify joint origins** — they come from DH params and are sacrosanct
- **One link at a time**, base-to-tip — errors compound along the chain
- **Save & Rebuild** writes the analysis YAML and regenerates — no need to run generate manually
- If `visual_rpy` is needed (L-shaped parts where proximal/distal axes differ), set it in `chain.yaml` and regenerate
- Never edit `robot.urdf` directly

## Gate

- [ ] Connection points match visually identifiable mating surfaces (confirmed against reference images)
- [ ] `verify_kinematics.py --json` passes (all joints within 2mm)
- [ ] Joint origins unchanged from DH-derived values (confirmed via URDF diff)
- [ ] Silhouette matches reference in 2+ orthogonal views
- [ ] No gaps or overlaps at joints

## Escalation

Stop and invoke `review` if:
- 3 iterations without convergence on a single link
- Kinematics verification fails after changes
- Cannot identify mating surfaces from reference images
