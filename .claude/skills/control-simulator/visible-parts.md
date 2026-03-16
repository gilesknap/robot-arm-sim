# Visible Parts

Control which robot parts are visible in the simulator using the Visible Parts chip toggles.

## Via JavaScript (preferred)

```javascript
function clickChip(name) {
  const chips = document.querySelectorAll('.q-chip');
  const chip = Array.from(chips).find(c => c.textContent.trim().replace('check', '') === name);
  if (chip) { chip.click(); return 'clicked ' + name; }
  return 'not found: ' + name;
}
```

Chip text content includes a "check" prefix from the checkmark icon — the helper strips it.

## Common Patterns

**Show only a pair of links** (e.g. base + shoulder):
```javascript
clickChip('All');        // deselect everything
clickChip('base');       // show base
clickChip('shoulder');   // show shoulder
```

**Show links base through link N** (cumulative base-to-tip):
```javascript
clickChip('All');        // deselect everything
clickChip('base');
clickChip('shoulder');
clickChip('upperarm');   // etc.
```

**Show all links**:
```javascript
clickChip('All');        // if currently deselected, this selects all
```

## Chip Names

Chip names match the link names in chain.yaml. They vary per robot, e.g.:
- UR5: `base`, `shoulder`, `upperarm`, `forearm`, `wrist1`, `wrist2`, `wrist3`
- Meca500: `A0`, `A1`, `A2`, `A3_4`, `A5`, `A6`, `EndEffector`

The `All` chip is always present and toggles every part on/off.

## Notes

- The "All" chip has special behaviour: toggling it off deselects every link, toggling it on selects every link
- After toggling, the 3D scene updates automatically (no need to call reload)
- Chip names are case-sensitive
