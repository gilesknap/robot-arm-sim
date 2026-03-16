# Toolbar Buttons

Click any simulator button via JavaScript — no ref lookups or coordinate clicks needed.

## Helper Function

```javascript
function clickBtn(name) {
  const b = Array.from(document.querySelectorAll('button, .q-btn'))
    .find(b => b.textContent.trim() === name);
  if (b) { b.click(); return 'clicked ' + name; }
  return 'not found: ' + name;
}
```

## Available Buttons

| Button | Effect |
|--------|--------|
| `Reset Joints` | Set all joint sliders to 0° (zero configuration) |
| `Reset View` | Ortho RIGHT side view, fitted to robot |
| `Frames` | Toggle coordinate frame axes at each joint |
| `Transparent` | Toggle mesh transparency (see axes through mesh) |
| `Labels` | Toggle link name labels |
| `Connections` | Toggle connection point markers |
| `Reload URDF` | Reload the URDF from disk (after regenerating) |
| `Edit Connections` | Enter/exit connection point editing mode |
| `Screenshot` | Save a screenshot |
| `Stop Simulator` | Stop the simulator process |

## Special Buttons (by ID)

The Fit and Persp/Ortho buttons live in the ViewCube overlay and must be clicked by element ID, not by `clickBtn`:

```javascript
document.getElementById('viewcube-fit-btn').click();   // Fit to visible parts
document.getElementById('viewcube-ortho-btn').click();  // Toggle ortho/perspective
```

**Do NOT use `clickBtn('Fit')` or `clickBtn('Persp')`** — these match NiceGUI wrapper elements that lack the JS event listeners.

## Standard Init Sequence

After navigating to a robot, run this to get a clean starting view:

```javascript
function clickBtn(name) {
  const b = Array.from(document.querySelectorAll('button, .q-btn'))
    .find(b => b.textContent.trim() === name);
  if (b) { b.click(); return 'clicked ' + name; }
  return 'not found: ' + name;
}
clickBtn('Reset Joints');
clickBtn('Reset Joints');  // twice — workaround for occasional bug
clickBtn('Reset View');    // ortho RIGHT side view
'init done'
```

## Notes

- Button text is exact and case-sensitive
- `Reset Joints` may need to be clicked twice due to a known bug
- `Reset View` sets ortho RIGHT side view with the robot fitted
- After `Reload URDF`, the scene re-renders but the page does NOT reload — refs remain valid
- `Persp`/`Ortho` button text changes depending on current projection mode
