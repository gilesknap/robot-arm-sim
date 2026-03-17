# Toolbar Toggle Buttons

Toggle buttons in the bottom toolbar (Frames, Transparent, Labels, Connections) are stateful — they can be on or off.

## Clicking Toggles

Use the same `clickBtn` helper from [buttons.md](buttons.md):

```javascript
function clickBtn(name) {
  const b = Array.from(document.querySelectorAll('button, .q-btn'))
    .find(b => b.textContent.trim() === name);
  if (b) { b.click(); return 'clicked ' + name; }
  return 'not found: ' + name;
}
clickBtn('Transparent');
```

## Checking Toggle State

Active toggles have the `text-primary` CSS class (blue text). Inactive toggles do not.

```javascript
function isToggleOn(name) {
  const b = Array.from(document.querySelectorAll('button, .q-btn'))
    .find(b => b.textContent.trim() === name);
  return b ? b.classList.contains('text-primary') : null;
}
isToggleOn('Transparent');  // true = on, false = off, null = not found
```

## IMPORTANT: Always verify after toggling

Toggle clicks can occasionally be swallowed, especially right after entering/exiting edit mode or after page transitions. **Always check the toggle state after clicking** and retry if it didn't change:

```javascript
function setToggle(name, wantOn) {
  const b = Array.from(document.querySelectorAll('button, .q-btn'))
    .find(b => b.textContent.trim() === name);
  if (!b) return 'not found: ' + name;
  const isOn = b.classList.contains('text-primary');
  if (isOn !== wantOn) {
    b.click();
    return 'toggled ' + name + ' to ' + (wantOn ? 'ON' : 'OFF');
  }
  return name + ' already ' + (wantOn ? 'ON' : 'OFF');
}
setToggle('Transparent', false);  // ensure transparency is OFF
setToggle('Frames', true);        // ensure frames are ON
```

After calling `setToggle`, take a screenshot or zoom into the toolbar to verify the state actually changed. If it didn't, call `setToggle` again.

## Toggle Buttons

| Button | Effect when ON |
|--------|---------------|
| `Frames` | Coordinate frame axes visible at each joint (RGB = XYZ) |
| `Transparent` | Meshes semi-transparent, can see axes through them |
| `Labels` | Link name labels shown in 3D view |
| `Connections` | Connection point markers shown (green=proximal, red=distal) |

## Notes

- Edit Connections mode auto-enables Frames and Transparent on entry
- The `Persp`/`Ortho` button is NOT a toggle in the same sense — its text changes to reflect current state
