# Review — Human Review Skill

Interactive discussion mode for reviewing the current state, diagnosing issues, and guiding adjustments.

## When to Use

- User invokes it at any time during or after the pipeline
- Orchestrator invokes it when step 06 escalates (visual_xyz > 40mm, 5 failed iterations)
- Post-pipeline fine-tuning or cosmetic adjustments

## Steps

### 1. Gather Current State

Read all current artifacts:
- `robots/<name>/chain.yaml` — current kinematic chain
- `robots/<name>/specs.yaml` — manufacturer specifications
- `robots/<name>/view_mapping.yaml` — view mappings (if exists)
- `robots/<name>/analysis/summary.yaml` — analysis overview
- Relevant `robots/<name>/analysis/<part>.yaml` files

### 2. Take Screenshots

If simulator is running on `localhost:8080` and claude-in-chrome is available:
- Screenshot the current view
- Use "Show up to" slider to isolate specific links if needed
- Snap to ortho views matching `view_mapping.yaml`

### 3. Present State to User

Summarize:
- Which links have been refined and pass gates
- Which links have issues and what the issues are
- Current `visual_xyz` and `origin` values for each link
- Any escalation triggers (large visual_xyz, iteration count)

### 4. Interactive Discussion

The user is the expert. They may:
- Point out a specific problem ("the shoulder is too high")
- Ask to compare specific views
- Request a specific joint pose for inspection
- Ask to see the "Show up to" slider at a specific level

For each user observation:
1. **Diagnose**: Map the symptom to a likely cause using the table from 06-refine-link
2. **Propose**: Suggest a specific chain.yaml edit (one value change)
3. **Apply** (if user agrees): Edit chain.yaml, regenerate URDF, reload simulator
4. **Verify**: Re-screenshot and re-run kinematics

### 5. Apply Fixes

When the user agrees to a fix:
```bash
# Edit chain.yaml (the specific value)
# Regenerate
uv run robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml
# Click "Reload URDF" in simulator
# Re-verify
uv run python robots/<name>/verify_kinematics.py --json
```

## Key Principles

- **User is the expert** — present information, propose fixes, but let the user decide
- **Show, don't tell** — take screenshots to illustrate issues
- **One fix at a time** — propose single-value changes, verify before moving on
- **Track iterations** — if going in circles, step back and reassess the approach
- **Can invoke sub-skills** — use 06-refine-link or 04-generate-verify as needed
