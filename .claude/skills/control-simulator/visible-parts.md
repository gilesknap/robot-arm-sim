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

## "All" Checkbox Behaviour

- **All checked**: every part is visible regardless of individual checkboxes
- **All unchecked**: individual checkboxes control visibility

To show only specific parts, **uncheck All first**, then uncheck the parts you want hidden.

## Common Patterns

**Show only a pair of links** (e.g. base + shoulder):
```javascript
setCB('All', false);
setCB('base', true);
setCB('shoulder', true);
setCB('upperarm', false);
setCB('forearm', false);
setCB('wrist1', false);
setCB('wrist2', false);
setCB('wrist3', false);
```

**Show all links**:
```javascript
setCB('All', true);
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
