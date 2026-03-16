# Visible Parts

Control which robot parts are visible in the simulator.

## Via JavaScript

```javascript
function setCB(name, checked) {
  const cbs = document.querySelectorAll('.q-checkbox');
  const cb = Array.from(cbs).find(c => c.textContent.trim() === name);
  if (!cb) return 'not found: ' + name;
  const isChecked = cb.getAttribute('aria-checked') === 'true';
  if (isChecked !== checked) { cb.click(); return 'toggled ' + name; }
  return name + ' already ' + (checked ? 'checked' : 'unchecked');
}
```

Use `setCB(name, true)` to show a part and `setCB(name, false)` to hide it. This is idempotent — it checks current state via `aria-checked` and only clicks if a change is needed.

The widgets are NiceGUI `ui.checkbox` elements rendered as Quasar `.q-checkbox`.

## All / None Buttons

- **All** button: checks every individual checkbox (shows all parts)
- **None** button: unchecks every individual checkbox (hides all parts)

These are `ui.button` elements, not checkboxes. Click via JS:
```javascript
clickBtn('All');   // show all parts
clickBtn('None');  // hide all parts
```

## Common Patterns

**Show only a pair of links** (e.g. base + shoulder):
```javascript
clickBtn('None');          // hide all first
setCB('base', true);
setCB('shoulder', true);
```

**Show all links**:
```javascript
clickBtn('All');
```

## Checkbox Names

Names match the mesh file stems in chain.yaml. They vary per robot, e.g.:
- UR5: `base`, `shoulder`, `upperarm`, `forearm`, `wrist1`, `wrist2`, `wrist3`
- Meca500: `A0`, `A1`, `A2`, `A3_4`, `A5`, `A6`, `EndEffector`

The `All` checkbox is always present.

## Notes

- Checkbox names are case-sensitive
- After toggling, the 3D scene updates automatically
- After Fit, only visible parts are used to compute the bounding box
