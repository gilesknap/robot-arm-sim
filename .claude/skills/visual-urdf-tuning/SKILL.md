# Visual URDF Tuning Skill

Compare the simulator against reference images of the real robot and refine the URDF by adjusting `chain.yaml`.

## When to Use

Use this skill when the user says the URDF "doesn't look right", wants to fix mesh placement, or asks to compare the simulator against real photos. The robot must already have a working `robot.urdf` and `chain.yaml`.

## Prerequisites

- `robot-arm-sim simulate` must be running (or you will start it)
- Browser automation tools (claude-in-chrome) must be available
- The robot directory must contain `chain.yaml` and `robot.urdf`

## Workflow

### Phase 1: Gather Reference Images

1. **Search for reference images** of the robot at known poses using WebSearch:
   - `"<robot name>" zero position photo`
   - `"<robot name>" home position`
   - `"<robot name>" CAD model`
   - Also check for manufacturer product pages and datasheets

2. **Save reference URLs** — note which pose each image shows (zero config, specific joint angles, etc.)

3. **Check for reference images already in the robot directory** — the user may have placed photos there.

### Phase 2: Launch and Capture Simulator

4. **Start the simulator** if not already running:
   ```bash
   nohup uv run robot-arm-sim simulate robots/<name>/ > /tmp/sim.log 2>&1 &
   ```
   Wait ~5 seconds, then verify with `curl -s -o /dev/null -w "%{http_code}" http://localhost:8080` (expect 200).
   - The simulator uses `show=False` so it won't auto-open browser tabs.
   - To stop: click "Stop Simulator" in the UI, or `pgrep -f "robot-arm-sim" | xargs kill`.
   - Check `/tmp/sim.log` if the process dies unexpectedly.

5. **Set up browser tab:**
   - Load browser tools first: `ToolSearch("select:mcp__claude-in-chrome__tabs_context_mcp,mcp__claude-in-chrome__read_page,mcp__claude-in-chrome__computer,mcp__claude-in-chrome__javascript_tool,mcp__claude-in-chrome__navigate")`
   - Call `mcp__claude-in-chrome__tabs_context_mcp` with `createIfEmpty: true` to get a tab
   - Navigate with `mcp__claude-in-chrome__navigate` to `http://localhost:8080`
   - After navigation, wait 4 seconds for the 3D scene to load before interacting

6. **Handle WebGL/connection issues:**
   - If `read_page` shows "WebGL context lost" or "Connection lost", press F5 to reload:
     ```
     mcp__claude-in-chrome__computer action=key text=F5
     ```
   - Wait 3 seconds after reload before interacting
   - If "Connection lost" persists, the simulator process may have died — check with `pgrep -f "robot-arm-sim simulate"` and restart if needed

7. **Use the label toggle** to identify parts and joints:
   - Click "Show Labels" button to display callout labels on the 3D model
   - Blue labels (left side) = part/mesh names (A0, A1, A2, etc.)
   - Red labels (right side) = joint names (joint_1, joint_2, etc.)
   - Part labels point to bounding box centers; joint labels point to joint origins

8. **Capture the zero-config pose:**
   - Use `mcp__claude-in-chrome__computer action=screenshot` to capture the full page
   - Use `action=zoom` with a region to inspect specific joints closely
   - This is the baseline to compare against the reference zero-config image

8. **Set specific test poses** using JavaScript slider control:

   **CRITICAL: NiceGUI uses Quasar `.q-slider` components, NOT native `input[type="range"]`.** Standard form input tools will not work. Use this JavaScript pattern:

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
   // Example: Set J2 (slider index 1) to 45° (range -70 to 90)
   clickSlider(1, 45, -70, 90);
   ```

   **Slider index mapping for Meca500-R3:**
   | Index | Joint | Label | Min° | Max° |
   |-------|-------|-------|------|------|
   | 0 | joint_1 | A1 | -175 | 175 |
   | 1 | joint_2 | A2 | -70 | 90 |
   | 2 | joint_3 | A3_4 | -135 | 70 |
   | 3 | joint_4 | A3_4 | -170 | 170 |
   | 4 | joint_5 | A5 | -115 | 115 |
   | 5 | joint_6 | A6 | -180 | 180 |

   **To reset all joints:** Click the "Reset Joints" button via `read_page` to find its ref, then `computer action=left_click ref=<ref_id>`.

   **Recommended test poses for a 6-axis arm:**
   - Zero config (all joints 0) — reveals overall assembly accuracy
   - J2 = 45° — reveals shoulder/upper arm placement
   - J3 = -45° — reveals elbow placement
   - J2 = 45°, J3 = -45° — reveals forearm alignment
   - J5 = 45° — reveals wrist placement
   - J1 = 90° — reveals base rotation axis centering

### Phase 3: Compare and Diagnose

8. **Compare simulator screenshots with reference images.** Look for:

   | Symptom | Likely cause | Fix in chain.yaml |
   |---------|-------------|-------------------|
   | Mesh floating/disconnected from parent | Wrong visual origin (proximal connection point off) | Add `visual_xyz` override on the link |
   | Mesh rotated wrong at zero config | Wrong `visual_rpy` | Adjust `visual_rpy` on the link |
   | Joint rotates around wrong point | Wrong joint origin | Add `origin` override on the joint |
   | Joint rotates in wrong direction | Wrong axis sign | Flip axis sign in joint spec |
   | Forearm at wrong angle | Axis wrong (Y vs Z) | Change `axis` on the joint |
   | Parts overlapping | Joint origin too short | Increase origin Z (or X) component |
   | Gap between parts | Joint origin too long | Decrease origin Z (or X) component |
   | Mesh offset sideways | Non-zero XY in connection point | Add `visual_xyz` to correct the offset |

9. **Cross-reference with analysis data.** Read the analysis YAMLs to check:
   - Connection point positions — are they sensible for each part?
   - Bounding box extents — do joint origins match expected link lengths?
   - Cylindrical surface axes — do they match the joint axes in chain.yaml?

### Phase 4: Fix and Regenerate

10. **Edit `chain.yaml`** to fix diagnosed issues. Common adjustments:

    - **Visual origin offset** — when a mesh isn't centered on its joint:
      ```yaml
      - name: link_5
        mesh: A5
        visual_rpy: [0, -1.5708, 0]
        visual_xyz: [0, 0.005, 0]  # nudge 5mm in Y
      ```

    - **Joint origin override** — when auto-computed origin is wrong:
      ```yaml
      - name: joint_4
        origin: [0.038, 0, 0.120]  # explicit metres
      ```

    - **Visual rotation** — when mesh coordinate frame is wrong:
      ```yaml
      - name: link_3
        mesh: A3_4
        visual_rpy: [0, 0, 1.5708]  # rotate 90° about Z
      ```

11. **Regenerate the URDF:**
    ```bash
    uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
    ```

12. **Verify kinematics:**
    ```bash
    uv run python robots/<name>/verify_kinematics.py --json
    ```
    Check all joints still pass within tolerance.

13. **Refresh the simulator** — the NiceGUI app needs restarting to pick up the new URDF:
    ```bash
    pkill -f "robot-arm-sim simulate" 2>/dev/null
    sleep 2
    uv run robot-arm-sim simulate robots/<name>/ &
    disown
    ```
    Wait ~5 seconds, verify with `curl`, then ask the user to reload (or press F5 via `computer action=key text=F5`).
    After reload, wait 3 seconds before interacting with sliders.

14. **Re-capture screenshots** at the same poses and compare again.

### Phase 5: Iterate

15. Repeat phases 3-4 until the simulator matches the reference images. Typical iteration count is 2-4 rounds.

16. **Document what was changed** — note which visual origins or joint origins needed manual overrides and why (e.g., "A5 proximal connection point was 5mm off in Y because the cross-section at that end is not circular").

## Key Principles

- **Never edit robot.urdf directly** — always edit chain.yaml and regenerate.
- **Make one change at a time** and check the result before making another.
- **Start from the base** and work outward — fix base_link first, then link_1, etc. Errors compound along the chain.
- **Use the verify_kinematics.py --json** check after each change to ensure FK positions haven't regressed.
- **`visual_xyz` in chain.yaml is ADDITIVE** — it's added on top of the auto-computed offset from the proximal connection point. So `visual_xyz: [0, 0, 0.019]` shifts the mesh 19mm up relative to where the auto-detection placed it.
- **Small visual_xyz adjustments** (< 5mm / 0.005m) are normal for imprecise connection point detection. Larger adjustments (10-20mm) can occur when the mesh doesn't fully span the kinematic link length (e.g., A3_4 is 101mm tall but d4=120mm).

## Meca500-R3 Specific Notes

- The Meca500 home/zero position is straight up (all joints 0°)
- A3_4 is L-shaped: Z bore at bottom (J3), X bore at +X end (J4/J5)
- A5 and A6 extend along STL +X, need `visual_rpy: [0, -1.5708, 0]` to map X→Z
- J4 and J5 are co-located (origin [0,0,0] between them)
- Expected flange height at zero config: 460mm from base
- DH params: d1=135, a2=135, a3=38, d4=120, d6=70 (all mm)
