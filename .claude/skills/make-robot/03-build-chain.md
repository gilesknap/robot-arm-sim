# 03 — Build Kinematic Chain

Combine analysis data and manufacturer specs to write `chain.yaml`.

## Input

- `robots/<name>/analysis/summary.yaml` and per-part YAMLs
- `robots/<name>/specs.yaml` (from step 02)

## Steps

### 1. Generate Draft Chain (automated)

Run the `build-chain` command to generate a scaffold `chain.yaml`:

```bash
uv run python -m robot_arm_sim build-chain robots/<name> --dry-run
```

This automates:
- Part ordering (for numbered names like `link_0`, `link_1`, ...)
- Joint axes from parent distal bore detection
- Joint origins from connection points (proximal → distal distances)
- Joint limits from specs.yaml (deg → rad conversion)
- DH params dict for reference metadata
- Cross-validation of distances against DH parameters
- Preservation of existing user-added links

When satisfied with the dry-run output, run without `--dry-run` to write `chain.yaml`.

### 2. Review and Fix (LLM judgment required)

The `build-chain` output flags items marked **REVIEW** that need manual attention:

#### Part ordering for semantic names
If parts use semantic names (base, shoulder, forearm, ...) instead of numbered
names, the script cannot auto-order them. Determine order from:
- Geometry: base is flat/short, bounding box sizes increase along the chain
- Role hints in `summary.yaml`
- Manufacturer documentation

#### Joint origins when connection points are missing
When parts have no detected connection points, the script falls back to
DH parameters with simple `[a/1000, 0, d/1000]` placement. This assumes
vertical stacking and will be wrong for robots with DH alpha rotations.
Fix the origin placement based on the robot's physical zero configuration.

#### origin_rpy for non-zero DH alpha
Joints with DH α ≠ 0 need frame rotations. The script flags these but
does not set `origin_rpy` because the correct mapping depends on the URDF
frame convention chosen for this robot. Two valid approaches:

1. **Use origin_rpy** (UR5 approach): Set `origin_rpy` on joints with α ≠ 0
   and keep axes simple. Common pattern: `origin_rpy: [α_rad, 0, 0]`.
2. **Fold into axes** (FANUC approach): No origin_rpy, but vary axis
   directions (e.g. `[-1,0,0]`, `[0,-1,0]`) to match accumulated frame
   rotations.

Pick one approach consistently for the robot.

#### Combined parts spanning multiple joints
Parts like `A3_4` that span multiple joints need a virtual link with
`mesh: null` inserted between them.

#### Cross-validation warnings
The script reports distance discrepancies between connection-point-derived
origins and DH parameters. Differences >10mm should be investigated —
the DH params should win, but large discrepancies may indicate wrong
part ordering or incorrect connection point detection.

### 3. Preserve User-Added Links

The `build-chain` command automatically preserves links/joints with
`user_added: true` from an existing `chain.yaml`. No manual action needed.

## Field Reference

| Field | Purpose | When to use |
|-------|---------|-------------|
| `links[].mesh` | STL file stem, or `null` for virtual links | Always |
| `links[].visual_xyz` | Additive visual offset (metres) | When auto-detection places mesh wrong |
| `links[].visual_rpy` | Mesh rotation (radians) | When STL orientation doesn't match link frame |
| `joints[].axis` | Rotation axis from parent's distal bore axis (e.g. `[0,0,1]` yaw, `[0,1,0]` pitch) | Always |
| `joints[].limits` | Joint angle limits in radians | Always |
| `joints[].origin` | Joint position (metres) | **Always** — use DH params |
| `joints[].origin_rpy` | Frame rotation (radians) | When DH alpha ≠ 0 |
| `links[].user_added` | Marks link as user-added (not from manufacturer) | End-effectors, custom tools — preserved across regeneration |
| `joints[].user_added` | Marks joint as user-added | Joint connecting a user-added link — preserved across regeneration |

## Output

- `robots/<name>/chain.yaml`

## Gate

- Valid YAML
- All links have mesh references (or explicit `null`)
- All joints have explicit `origin` values
- All joints have `limits` from manufacturer specs
- Joint count matches DOF from specs.yaml
- Cross-validation shows no unexplained >10mm discrepancies
