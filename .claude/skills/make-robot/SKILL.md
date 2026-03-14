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
                  ├──> 03-build-chain ──> 04-generate-verify ──> human visual refinement
02-research-specs ┘

Steps 05-07 (visual refinement) are DEPRECATED — hand off to the human.
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

### 4. Visual Refinement (HUMAN STEP)

Steps visual connection point refinement requires human judgement and is better done manually using the simulator's Edit Connections UI. review.md provides a structured process for this discussion and refinement.

Launch the simulator and hand off to the human:
```bash
nohup uv run robot-arm-sim simulate robots/<name>/ > /tmp/sim.log 2>&1 &
```

Tell the user: "The simulator is running at localhost:8080. Use **Edit Connections** to click on mating surfaces for each link (proximal = toward base, distal = toward tip). Click **Save & Rebuild** after each link. The auto-detected connection points are a starting point but will need manual adjustment."

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
