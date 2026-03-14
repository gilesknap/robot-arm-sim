Base directory for this skill: /workspaces/robot-arm-sim/.claude/skills/set-visible-parts

# Set Visible Parts Skill

Control which robot parts are visible in the simulator using the Visible Parts chip toggles.

## When to Use

Use this skill when you need to show/hide specific robot links in the simulator, such as:
- Isolating a single link for inspection during visual refinement
- Showing a subset of links (e.g. base_link + link_1 to check a joint)
- Restoring all parts to visible

## How the Widgets Work

The Visible Parts section is always visible in the right sidebar (no expansion panel) containing NiceGUI `ui.chip` widgets (`q-chip` in Quasar). Each chip is `selectable=True`:

- **Selected** (checked): chip has a checkmark icon, part is visible
- **Deselected** (unchecked): no checkmark, part is hidden

There is an "All" chip that toggles every part on/off at once.

## How to Interact

### Step 1: Find the chip refs

Use `mcp__claude-in-chrome__find` to get element references:
```
find: "selectable chip buttons for visible parts"
```

This returns refs like:
- `ref_124`: "All"
- `ref_127`: "base_link"
- `ref_130`: "link_1"
- etc.

**Important**: ref IDs change between page loads. Always use `find` to get fresh refs.

### Step 2: Click chips using refs

Use `mcp__claude-in-chrome__computer` with `left_click` and `ref` parameter:
```
action: left_click
ref: ref_124  (the "All" chip)
```

### Common Patterns

**Show only one link** (e.g. base_link):
1. Click "All" chip to deselect everything (if currently all selected)
2. Click the specific link chip to select it

**Show links 0 through N** (for base-to-tip inspection):
1. Click "All" to deselect everything
2. Click each link chip from base_link through link_N

**Show all links**:
1. Click "All" chip (if currently deselected, this selects all)

### Checking Current State

- A chip with a checkmark (tick icon) is **selected/visible**
- A chip without a checkmark is **deselected/hidden**
- Take a screenshot or zoom into the chip area to verify state
- Zoom region for chips: approximately `[1010, 550, 1200, 680]` (may vary)

## Key Details

- The chips are Quasar `q-chip` elements, NOT standard checkboxes
- Clicking the chip body toggles selection — coordinate clicks may miss, so **always use `ref` parameter**
- The "All" chip has special behavior: toggling it off deselects every link, toggling it on selects every link
- After toggling, the 3D scene updates automatically (no need to call reload)
