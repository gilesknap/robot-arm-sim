Base directory for this skill: /workspaces/robot-arm-sim/.claude/skills/nicegui-patterns

# NiceGUI 3D Simulator Patterns

Gotchas and patterns for working with NiceGUI's 3D scene in this project. Consult this when editing the simulator UI code under `src/robot_arm_sim/simulate/app/`.

## Visibility

- `Object3D.visible(False)` doesn't reliably hide objects in the renderer. Instead, move hidden objects off-screen: `obj.move(0, 0, -100)`.

## App Lifecycle

- `app.shutdown()` crashes if stale browser tabs are still connected. Use `os.kill(os.getpid(), signal.SIGTERM)` instead.
- To reload after URDF regeneration, navigate to `/` — this re-reads the URDF without restarting the process. Save UI state to `sessionStorage` before navigating so it can be restored.
- After regenerating STL files, users need Ctrl+Shift+R to bust the browser cache.

## JavaScript Interop

- The Three.js scene component is found by walking the Vue vnode tree looking for a proxy with `renderer && scene && controls`. This is wrapped in `findSceneComp()` in several JS snippets.
- `ui.run_javascript()` called from timers can throw `TimeoutError` or `RuntimeError` when a client disconnects during page reload. Always wrap in try/except.
- NiceGUI's `scene.on_click` only reports hits on NiceGUI-managed objects. For custom JS-created meshes (bore markers, face markers), use a JS-side raycaster + `window.__lastFaceClick` + a Python polling timer.

## Deferred Handler Pattern

When a button must be created before its handler is defined (e.g., toolbar button whose handler depends on UI built later), store a callable on the shared state object:

```python
# In toolbar (created first):
state.edit_bores_btn = ui.button(
    "Edit Bores",
    on_click=lambda: state.toggle_edit_bores(),
)

# In edit_bores builder (created later):
def _toggle_edit_bores():
    ...
state.toggle_edit_bores = _toggle_edit_bores
```

The lambda captures `state` by reference, so the call resolves correctly at click time.

## Architecture

The simulator UI is built with a `SimulatorState` class (in `state.py`) that holds all shared state. Separate builder functions each take the state object and add NiceGUI elements to the current container context:

- `build_scene(state)` — 3D viewport, meshes, callouts, JS init
- `build_toolbar(state)` — toolbar buttons
- `build_visibility_panel(state)` — per-part checkboxes
- `build_edit_bores(state)` — interactive bore assignment mode
- `build_controls_panel(state)` — joint/IK sliders, EE readout, state restore
