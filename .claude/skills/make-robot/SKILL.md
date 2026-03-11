# Make Robot Skill

Orchestrate the complete pipeline from raw STL files to a verified, visually accurate robot simulation. Launches focused sub-agents for each stage with gates between them.

## When to Use

Use this skill when:
- The user wants to add a new robot from STL files
- The user says "build a robot", "add a robot", "make robot", or "auto-build"
- A `robots/<name>/stl_files/` directory exists but no `chain.yaml` or `robot.urdf`

## Prerequisites

- STL mesh files in `robots/<name>/stl_files/`, in millimetres with Z up
- A working devcontainer with `uv` available
- For visual refinement (steps 05-07): claude-in-chrome MCP server for browser screenshots

## Pipeline

```
01-analyze-stls ──┐
                  ├──> 03-build-chain ──> 04-generate-verify ──> 05-setup-comparison
02-research-specs ┘                                               ──> 06-refine-link (×N) ──> 07-final-validation

At any point: user can invoke `review` to discuss issues interactively
```

## Orchestration Steps

### 1. Parallel: Analyze STLs + Research Specs

Launch two sub-agents **in parallel**:

- **01-analyze-stls**: Run `uv run robot-arm-sim analyze robots/<name>/`
  - Read instructions from `.claude/skills/make-robot/01-analyze-stls.md`
  - **Gate**: Every STL has a corresponding YAML with `connection_points` in `analysis/`

- **02-research-specs**: Web search for DH params, datasheets, reference images
  - Read instructions from `.claude/skills/make-robot/02-research-specs.md`
  - **Gate**: `robots/<name>/specs.yaml` has all DH parameters and joint limits populated

### 2. Build Kinematic Chain

- **03-build-chain**: Combine analysis YAMLs + specs.yaml into chain.yaml
  - Read instructions from `.claude/skills/make-robot/03-build-chain.md`
  - **Gate**: Valid `chain.yaml` with all links, joints, origins, and limits

### 3. Generate and Verify URDF

- **04-generate-verify**: Generate URDF and run kinematics verification
  - Read instructions from `.claude/skills/make-robot/04-generate-verify.md`
  - **Gate**: All joints within 2mm of manufacturer specs
  - **Loop**: Fix chain.yaml → regenerate → re-verify until passing

### 4. Setup Visual Comparison

Launch the simulator:
```bash
nohup uv run robot-arm-sim simulate robots/<name>/ > /tmp/sim.log 2>&1 &
```

- **05-setup-comparison**: Map reference images to simulator views
  - Read instructions from `.claude/skills/make-robot/05-setup-comparison.md`
  - **Gate**: At least 2 orthogonal views mapped, baseline screenshots taken

### 5. Per-Link Refinement (Base to Tip)

Read `robots/<name>/analysis/summary.yaml` to get the link count N.

For each link index 0 through N-1, **sequentially**:
- **06-refine-link**: Refine visual alignment for one link
  - Read instructions from `.claude/skills/make-robot/06-refine-link.md`
  - Pass the link index as context
  - **Gate**: Link length within 2mm, silhouette matches, no gaps, kinematics pass
  - **Escalation**: If `visual_xyz` > 40mm or 5 iterations without convergence → invoke `review`

### 6. Final Validation

- **07-final-validation**: Multi-view + multi-pose acceptance test
  - Read instructions from `.claude/skills/make-robot/07-final-validation.md`
  - **Gate**: Full sign-off checklist passes

### 7. Error Handling

If any gate fails:
1. Retry the step once
2. If still failing, invoke `review` for human discussion

## Human Review

At any point, the user can invoke the `review` skill for interactive discussion:
- Read instructions from `.claude/skills/make-robot/review.md`
- Takes screenshots, reads current artifacts, enables back-and-forth debugging
- Can invoke sub-skills (06-refine-link, 04-generate-verify) to apply fixes

## Key Principles

- **DH params are ground truth** — bore-detected connection points are validation only
- **Work base-to-tip** — errors compound along the chain
- **One change at a time** — adjust, regenerate, verify, repeat
- **Visual comparison is core** — not optional, not an afterthought
- **Never edit robot.urdf directly** — always edit chain.yaml and regenerate
