Base directory for this skill: /workspaces/robot-arm-sim/.claude/skills/refine-with-image

# Refine URDF with Reference Image

Iteratively refine robot URDF part alignment by comparing the simulator against reference photos of the real robot.

## When to Use

Use this skill when the user provides a reference image (photo, CAD screenshot, datasheet illustration) of the robot and wants the simulator to match it. The robot must already have a working `chain.yaml` and `robot.urdf`.

## Prerequisites

- A reference image of the robot at a known pose (user provides path or pastes it)
- `chain.yaml` and `robot.urdf` exist in the robot directory
- Browser automation tools (claude-in-chrome) available
- The `zoom-rotate-camera` and `visual-urdf-tuning` skills for camera control and URDF editing

## Workflow

### Step 1: Gather the Reference

Ask the user:
1. **Image**: "Please provide a reference image of the robot (file path, screenshot, or URL)"
2. **Pose**: "What joint angles is the robot at in this image? (e.g., zero config, J2=45°, or 'unknown')"
3. **View angle**: "What direction is the camera looking from? (e.g., front, side, 3/4)"

If the user just pastes an image, infer what you can from context and ask about unknowns.

### Step 2: Read the Reference Image

Use the `Read` tool to view the reference image file. Study it carefully:
- Which parts are visible?
- Where are the joints? Look for bore circles, seams, gaps between parts.
- What's the overall silhouette shape?
- Are there visible misalignments (gaps, overlaps, wrong angles)?

### Step 3: Set Up the Simulator

1. **Ensure simulator is running:**
   ```bash
   pgrep -f "robot-arm-sim simulate" || (nohup uv run robot-arm-sim simulate robots/<name>/ > /tmp/sim.log 2>&1 & sleep 5)
   ```

2. **Load browser tools** (if not loaded):
   ```
   ToolSearch("select:mcp__claude-in-chrome__tabs_context_mcp,mcp__claude-in-chrome__read_page,mcp__claude-in-chrome__computer,mcp__claude-in-chrome__javascript_tool,mcp__claude-in-chrome__navigate")
   ```

3. **Open/refresh the simulator tab:**
   - `tabs_context_mcp` with `createIfEmpty: true`
   - Navigate to `http://localhost:8080` if needed
   - Wait 4-5 seconds for 3D scene to load

### Step 4: Match the View

Use the camera control skill to match the reference image's viewing angle.

**Camera control JavaScript** (always use this, never mouse drag):
```javascript
function findSceneComp(vnode) {
    if (!vnode) return null;
    if (vnode.component && vnode.component.proxy) {
        const p = vnode.component.proxy;
        if (p.renderer && p.scene && p.controls) return p;
    }
    if (vnode.children && Array.isArray(vnode.children)) {
        for (const c of vnode.children) { const r = findSceneComp(c); if (r) return r; }
    }
    if (vnode.component && vnode.component.subTree) return findSceneComp(vnode.component.subTree);
    return null;
}
const sc = findSceneComp(document.getElementById('app').__vue_app__._container._vnode);
sc.camera.position.set(X, Y, Z);      // camera position in metres
sc.controls.target.set(TX, TY, TZ);   // look-at target in metres
sc.controls.update();
'camera set'
```

**Coordinate system:**
- X: left(-) / right(+)
- Y: front(-) / back(+) — negative Y faces the default viewer
- Z: ground(0) / up(+)

**Common starting positions:**
| View | Camera | Target |
|------|--------|--------|
| Front | `(0, -0.6, 0.25)` | `(0, 0, 0.25)` |
| Side | `(0, -0.25, 0.25)` or `(-0.25, 0, 0.25)` | `(0, 0, 0.25)` |
| 3/4 | `(-0.4, -0.4, 0.3)` | `(0, 0, 0.25)` |

Adjust camera distance to match the reference image's zoom level.

### Step 5: Match the Joint Pose

Set joints to match the reference using JavaScript sliders:
```javascript
const tracks = document.querySelectorAll('.q-slider__track-container');
function clickSlider(idx, degrees, minDeg, maxDeg) {
    const t = tracks[idx];
    const r = t.getBoundingClientRect();
    const frac = (degrees - minDeg) / (maxDeg - minDeg);
    const x = r.left + frac * r.width;
    const y = r.top + r.height / 2;
    t.dispatchEvent(new MouseEvent('mousedown', {clientX: x, clientY: y, bubbles: true}));
    t.dispatchEvent(new MouseEvent('mouseup', {clientX: x, clientY: y, bubbles: true}));
}
// Meca500-R3 slider mapping:
// idx 0: joint_1 [-175, 175]
// idx 1: joint_2 [-70, 90]
// idx 2: joint_3 [-135, 70]
// idx 3: joint_4 [-170, 170]
// idx 4: joint_5 [-115, 115]
// idx 5: joint_6 [-180, 180]
```

### Step 6: Compare and Diagnose

Take a screenshot and compare against the reference image. Work from base to tip:

1. **Screenshot the simulator** at the matching pose and angle
2. **Compare part by part**, starting from base_link upward:
   - Is each part's position correct relative to its parent?
   - Are bore circles aligned with joint rotation centers?
   - Are there gaps or overlaps at joints?
   - Does the overall silhouette match?
3. **Zoom in** on problem areas using `computer action=zoom region=[x0,y0,x1,y1]`
4. **Turn on labels** (click "Show Labels") to identify which part/joint is misaligned

**Diagnosis table:**

| Symptom | Likely cause | Fix in chain.yaml |
|---------|-------------|-------------------|
| Part too high/low | `visual_xyz` Z offset wrong | Adjust Z in `visual_xyz` on the link |
| Part shifted sideways | `visual_xyz` X/Y offset wrong | Adjust X/Y in `visual_xyz` |
| Part rotated wrong | `visual_rpy` wrong | Adjust `visual_rpy` on the link |
| Gap between parts | Joint origin too long or visual_xyz wrong | Reduce origin Z or adjust visual_xyz |
| Parts overlapping | Joint origin too short or visual_xyz wrong | Increase origin Z or adjust visual_xyz |
| Joint rotates around wrong point | Bore not aligned with joint origin | Adjust `visual_xyz` to center bore on joint |

### Step 7: Fix and Iterate

For each issue found:

1. **Edit `chain.yaml`** — never edit robot.urdf directly
   - `visual_xyz: [x, y, z]` is ADDITIVE to auto-computed offset (metres)
   - `visual_rpy: [r, p, y]` sets mesh rotation (radians)
   - `origin: [x, y, z]` on a joint overrides auto-computed joint position

2. **Regenerate URDF:**
   ```bash
   uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
   ```

3. **Restart simulator** (URDF changes require process restart, not just F5):
   ```bash
   pkill -9 -f "robot-arm-sim simulate" 2>/dev/null
   sleep 2
   # Verify port is free before restarting — stale NiceGUI can hold it
   for i in 1 2 3 4 5; do lsof -i :8080 >/dev/null 2>&1 || break; sleep 1; done
   nohup uv run robot-arm-sim simulate robots/<name>/ > /tmp/sim.log 2>&1 &
   ```
   Then wait and **verify HTTP is responding** before navigating the browser:
   ```bash
   for i in 1 2 3 4 5 6 7 8; do
     sleep 1
     curl -s -o /dev/null -w "%{http_code}" http://localhost:8080/ | grep -q 200 && break
   done
   ```
   If `curl` still returns 000 or a non-200 code, check `tail -30 /tmp/sim.log` for errors.
   Common failure: NiceGUI "parent slot deleted" crash from stale browser tabs — use `kill -9` and wait for port to free.
   Only navigate the browser tab AFTER the curl health-check passes.

4. **Verify kinematics** haven't regressed:
   ```bash
   uv run python robots/<name>/verify_kinematics.py --json
   ```

5. **Re-screenshot** at the same pose/angle and compare again

### Step 8: Symmetry Check

For each joint that was adjusted, verify with a symmetry test:
- Set the joint to +45° and screenshot
- Set the joint to -45° and screenshot
- Compare: the gap/overlap at the joint should be symmetric in both directions
- If asymmetric, the bore isn't centered on the rotation — adjust `visual_xyz`

### Step 9: Request Additional Images

If you can't diagnose an issue from the current angle, ask the user:
- "Can you provide a side view photo? I need to check the joint_3 alignment from a different angle."
- "Do you have a photo with J2 bent at ~45°? That would help me verify the rotation center."
- "Can you provide a top-down view? I'm seeing an X-axis offset I can't diagnose from the front."

### Step 10: Final Validation

Once all visible parts match the reference:
1. Reset all joints to zero config
2. Screenshot from front, side, and 3/4 views
3. Compare all three against reference images
4. Run `verify_kinematics.py --json` to confirm FK positions
5. Show the user the before/after comparison

## Key Principles

- **Work base to tip** — fix base_link first, errors compound along the chain
- **One change at a time** — adjust one `visual_xyz` or `visual_rpy`, regenerate, compare
- **Small adjustments** — typical corrections are 1-5mm (0.001-0.005m). Larger than 40mm suggests a wrong connection point
- **Use analysis data** — read `<robot_dir>/analysis/<part>.yaml` for connection point positions, bore radii, and cylindrical surface data to inform adjustments
- **visual_xyz is additive** — `[0, 0, 0]` means "use auto-computed position". Non-zero values shift relative to that
- **Never edit robot.urdf** — always edit chain.yaml and regenerate
- **Always restart the simulator process** after regenerating the URDF — hot reload does NOT pick up URDF file changes
- **Check connection point detection** — the analyzer may pick a flat face (e.g. bottom plate) as proximal instead of the actual bore. Cross-check with flat_faces centroids in analysis YAML. If `visual_xyz` exceeds ~40mm, the proximal is likely wrong.
- **Don't assume visual_rpy** — check reference photos for mesh orientation at zero config. Wrist parts (A5/A6) may extend along +X naturally and should NOT be rotated to Z-up if the wrist points horizontally at zero config.
- **Bore direction matters** — if a bore is on the FRONT face of a part (X-axis), the child part connects from the front, not the top. Check distal axis in analysis YAML.
