# 06 — Refine Link (DEPRECATED — better done by humans)

Fix connection points for one link at a time, working base-to-tip. Use reference images + the simulator's Edit Connections mode to visually identify and click the correct mating surfaces.

## Input

- Link index N (0-based), robot name
- Simulator on `localhost:8080`, claude-in-chrome MCP for screenshots/clicks

## Steps

### 1. Gather reference images from the web

1. Use `WebSearch` to find manufacturer photos and diagrams of the robot (e.g. "FANUC LR Mate 200iD robot arm")
2. Use `WebFetch` to download 2–3 clear reference images showing the robot from different angles (front, side, isometric)
3. Save them to `robots/<name>/reference/` — these are used throughout the remaining steps to judge correct placement
4. Note the key features: overall silhouette, joint locations, relative segment lengths, mating surfaces

### 2. Inspect BEFORE editing — only fix what's broken

**CRITICAL: Do NOT blindly re-assign connection points for every link.** The auto-detected points may already be correct. Editing a well-positioned link will make it worse.

1. Show ALL parts, set RIGHT ortho view, compare full robot silhouette against reference
2. Look for specific problems: gaps between parts, overlapping meshes, parts in wrong positions
3. Identify WHICH links need fixing — typically the ones with visible gaps or misalignment
4. For links that look correct, **skip them entirely**
5. Only enter Edit Connections for links that clearly need new connection points

When you do need to inspect a single part:
1. Use `set-visible-parts` to isolate the link
2. Use `zoom-rotate-camera` to get clear views from multiple angles
3. Compare against reference images — identify:
   - Where does this part connect to its **parent** (proximal)?
   - Where does this part connect to its **child** (distal)?
   - What kind of surface is it? (flat flange → `surface` mode, through-bore → `center` mode)

### 3. Assign connection points via Edit Connections

1. Click **Edit Connections** in the toolbar — meshes go semi-transparent
2. Use `set-visible-parts` skill to isolate just the link being edited (use `find` to get chip refs, click "All" to deselect, then click the target link)
3. Select **Proximal** or **Distal** toggle
   - **Proximal** = toward the base (parent joint side)
   - **Distal** = toward the tip (child joint side)
   - For base_link: proximal = bottom (ground), distal = top (shoulder)
4. Set **centering mode** (`surface` or `center`)
5. **CRITICAL: Use an orthogonal view where the camera looks straight down at the target surface** — the camera viewing direction must be perpendicular to the surface you're clicking. This ensures the raycast hits the correct face with accurate position and normal:
   - Bottom face → use **BOTTOM** view (camera from -Z)
   - Top face → use **TOP** view (camera from +Z)
   - Front-facing flange → use **FRONT** view (camera from +X)
   - Side-facing flange → use **RIGHT** or **LEFT** view
   - Never click surfaces from an angled or perspective view — it distorts positions
6. **Click directly on the mesh surface** at the mating face — the click handler captures the exact position and face normal
7. Repeat for the other end (proximal/distal), switching to the appropriate orthogonal view
8. Click **Save & Rebuild** — writes the analysis YAML (with `method: manual`) and regenerates the URDF automatically

### 4. Verify the result

1. Exit Edit Connections mode
2. Screenshot from orthogonal views (FRONT, RIGHT minimum), compare against reference
3. Check: no gaps, no overlaps, silhouette matches reference

### 5. Verify kinematics

```bash
uv run python robots/<name>/verify_kinematics.py --json
```

All joints must pass within 2mm.

### 6. Diff the URDF

```bash
diff <(git show HEAD:robots/<name>/robot.urdf) robots/<name>/robot.urdf
```

Confirm only visual origins changed — joint origins must be unchanged.

### 7. If wrong, iterate

Return to step 2. Re-examine the reference images — did you pick the wrong surface? Wrong centering mode? Re-enter Edit Connections and try again.

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
