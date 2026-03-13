Base directory for this skill: /workspaces/robot-arm-sim/.claude/skills/edit-connections-ui

# Edit Connections UI Skill

How to drive the Edit Connections mode in the simulator to assign proximal/distal connection points on robot meshes.

## Entering Edit Connections Mode

1. Find and click the "Edit Connections" button in the toolbar (use `find` to get ref)
2. Meshes go semi-transparent, green/red markers appear for existing connections
3. A bottom bar appears with: Proximal/Distal toggle, Centering dropdown, Show All checkbox, status label, Save & Rebuild button

## The Proximal/Distal Toggle

This is the most important control. It determines WHICH end you are assigning when you click a mesh.

- **Proximal** = toward the base/parent (green when active, green markers)
- **Distal** = toward the tip/child (orange/red when active, red markers)

**CRITICAL: Always verify the correct toggle is active BEFORE clicking on a mesh.** The toggle state determines what gets assigned. If you click with the wrong toggle, you'll overwrite the wrong connection point.

### How to use:
1. Use `find` to locate the toggle buttons: `"Proximal toggle button"` and `"Distal toggle button"`
2. Click the desired button using its ref
3. **Verify** by zooming into the bottom bar — the active button text is colored (green for Proximal, orange for Distal)
4. Only then click on the mesh surface

## The Centering Dropdown

Controls how the pipeline interprets the clicked point:

- **Surface** — places the marker exactly where you click. Use for flat mating flanges (base bottom, flat faces between parts)
- **Center** — the pipeline averages with the opposite face to find the bore center. Use for cylindrical bores/shafts where the joint axis passes through

### How to use:
1. Use `find` to locate: `"Centering dropdown select"`
2. Click the dropdown ref to open it
3. A popup appears with "Surface" and "Center" options
4. Click the desired option text
5. **Verify** by zooming into the bottom bar to confirm the value changed

## Click Placement — Orthogonal Views

**CRITICAL: Always use an orthogonal view where the camera looks perpendicular to the surface you are clicking.**

The raycast from a click needs to hit the target face head-on for accurate position and normal capture. Angled views distort the hit point.

| Target surface | Required view | Camera direction |
|---|---|---|
| Bottom face (Z=0) | BOTTOM | from -Z, looking up |
| Top face (Z=max) | TOP | from +Z, looking down |
| Front-facing flange | FRONT | from +X |
| Back-facing flange | BACK | from -X |
| Right-facing flange | RIGHT | from -Y |
| Left-facing flange | LEFT | from +Y |

Use the `zoom-rotate-camera` skill's Ortho Named View One-liner to set the view before clicking.

## Workflow for One Link

1. **Isolate the link** — use `set-visible-parts` skill to show only the target link
2. **Set Proximal toggle** (verify it's green)
3. **Set Centering** (Surface for flat face, Center for bore)
4. **Switch to orthogonal view** perpendicular to the proximal surface
5. **Click the proximal surface** on the mesh
6. **Verify** status bar says "Set <link> proximal"
7. **Set Distal toggle** (verify it's orange)
8. **Set Centering** as appropriate for the distal surface
9. **Switch to orthogonal view** perpendicular to the distal surface
10. **Click the distal surface** on the mesh
11. **Verify** status bar says "Set <link> distal"
12. **Click Save & Rebuild** — writes YAML, regenerates URDF, page reloads

## After Save & Rebuild

- The page reloads with new URDF
- Edit Connections mode exits
- Camera may reset — you'll need to re-enter Edit Connections and re-navigate for the next link
- **IMPORTANT: The page reload preserves visibility state from before Save, which was likely isolating one link. You MUST re-show all parts** using `set-visible-parts` skill (expand Visible Parts, click "All" chip to enable all) before evaluating the result
- Check the result by switching to RIGHT/FRONT views and comparing against reference

## Finding UI Elements

Always use `mcp__claude-in-chrome__find` with refs rather than coordinate clicks for toolbar elements:

```
"Edit Connections button in toolbar"     → Edit Connections / Exit Edit button
"Proximal toggle button"                  → Proximal toggle
"Distal toggle button"                    → Distal toggle
"Centering dropdown select"               → Centering dropdown
"Save & Rebuild button"                   → Save & Rebuild
"selectable chip buttons for visible parts" → Visibility chips
```

**Ref IDs change after page reload** — always re-find after Save & Rebuild.

## Common Mistakes

- Clicking a mesh with the wrong Proximal/Distal toggle active
- Not switching to the correct orthogonal view before clicking (angled views give wrong normals)
- Forgetting to change Centering mode between Surface and Center
- Not verifying the status bar message after each click
