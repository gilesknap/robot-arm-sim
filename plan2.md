# Plan 2: Fix URDF Assembly via Programmatic Connection Point Detection

## Context

Comparing the simulator against a real photo of the Meca500-R3, parts are disconnected/floating — especially the wrist (A5) and flange (A6). The root cause is that the assembly-reasoning skill asks Claude to **manually guess** 3D mesh origin offsets from text-based feature descriptions. This is inherently imprecise, and small errors compound along the 6-joint chain.

**Core insight:** Connection point detection (finding bore/shaft centers at each end of a mesh) is a well-defined geometric computation that Python can do precisely. Kinematic chain reasoning (which parts connect, joint types/axes) requires semantic understanding that Claude does well. Split the problem accordingly.

## Architecture

```
analyze → [connection points] → skill writes chain.yaml → generate → URDF → simulate
                                     (topology only)        (exact math)
```

The skill stops writing raw URDF XML. Instead it writes a simple `chain.yaml` describing the kinematic chain (part order, joint types, axes, limits). A new `generate` command computes exact transforms from mesh geometry and produces the URDF.

---

## Step 1: Add `ConnectionPoint` model

**File:** `src/robot_arm_sim/models/part.py`

Add dataclass:
```python
@dataclass
class ConnectionPoint:
    end: str            # "proximal" or "distal"
    position: list[float]  # [x,y,z] in STL coords (mm)
    axis: list[float]      # joint axis unit vector
    radius_mm: float
    method: str            # "cross_section" or "cylinder_fit"
```

Add `connection_points: list[ConnectionPoint]` field to `PartAnalysis`.

## Step 2: Create `analyze/connections.py` — Connection Point Detection

**New file:** `src/robot_arm_sim/analyze/connections.py`

Algorithm for each mesh:

1. **For each detected cylindrical surface** (from features.py), use its axis as a candidate joint axis
2. **Slice cross-sections** near both ends of the mesh along that axis (at 5%, 10%, 15% of extent) using `mesh.section(plane_origin, plane_normal)`
3. **Find circular features** in each cross-section: convert to 2D path, extract polygons, check circularity (`std(radii)/mean(radii) < 0.15`)
4. **Compute bore center** — the centroid of the circular feature, transformed back to 3D STL coordinates
5. **Classify as proximal/distal** based on position along the axis (min = proximal/parent-side, max = distal/child-side)

Special cases:
- **A3_4 (L-shaped, multi-joint):** Slice along both Z and X axes to find J3 bore (bottom, Z-axis) and J4 bore (+X end, X-axis)
- **A5, A6 (X-extending):** Primary axis is X, slice perpendicular to X at both ends
- **Non-watertight meshes (A5, A6):** Fall back to least-squares circle fit on boundary vertices if `to_planar()` fails to produce clean polygons

Function: `detect_connection_points(mesh, features, part_name) -> list[ConnectionPoint]`

## Step 3: Integrate into analysis pipeline

**File:** `src/robot_arm_sim/analyze/__init__.py`

After `detect_features(mesh)`, call `detect_connection_points(mesh, features, part_name)` and store results in `analysis.connection_points`.

**File:** `src/robot_arm_sim/analyze/yaml_writer.py`

Serialize connection points into each part YAML:
```yaml
connection_points:
  - end: proximal
    position: [0.0, 0.0, 1.03]
    axis: [0, 0, 1]
    radius_mm: 22.5
    method: cross_section
```

## Step 4: Create `analyze/urdf_generator.py` — Automated URDF Generation

**New file:** `src/robot_arm_sim/analyze/urdf_generator.py`

Input: a `chain.yaml` specifying kinematic topology + analysis YAMLs with connection points.

**chain.yaml format** (written by the skill):
```yaml
robot_name: Meca500-R3
dh_params:  # from manufacturer specs
  d1: 135
  a2: 135
  a3: 38
  d4: 120
  d6: 70
links:
  - name: base_link
    mesh: A0
  - name: link_1
    mesh: A1
  - name: link_2
    mesh: A2
  - name: link_3
    mesh: A3_4
  - name: link_4
    mesh: null  # virtual link
  - name: link_5
    mesh: A5
  - name: link_6
    mesh: A6
joints:
  - name: joint_1
    type: revolute
    parent: base_link
    child: link_1
    axis: [0, 0, 1]
    limits: [-3.054, 3.054]
  # ... etc
```

**Algorithm for computing transforms:**

For each (joint, child_link) in chain:
1. Load child mesh's proximal connection point from analysis YAML
2. **Visual origin xyz** = negation of proximal connection point (mm→m) — places the mesh so its bore center sits at the link frame origin
3. **Visual origin rpy** = rotation to align mesh primary axis with link Z-up convention
4. **Joint origin xyz** = parent mesh's distal connection point position in parent frame (mm→m)

**Cross-validation:** After computing all transforms, run FK at zero config (reusing `simulate/kinematics.py` logic) and compare against expected positions from DH params. Warn if any joint deviates > 2mm.

Function: `generate_urdf(chain_spec, analysis_dir, stl_dir, output_path) -> list[str]`

## Step 5: Add `generate` CLI subcommand

**File:** `src/robot_arm_sim/__main__.py`

```python
@app.command()
def generate(robot_dir: Path, chain_file: Path):
    """Generate URDF from kinematic chain spec + analysis data."""
```

Calls `urdf_generator.generate_urdf(...)` and prints validation results.

## Step 6: Update assembly-reasoning skill

**File:** `.claude/skills/assembly-reasoning.md`

Rewrite to:
1. Read analysis YAMLs (including connection_points data)
2. Write `chain.yaml` (topology, joint types, axes, limits, DH params from web search)
3. Run `robot-arm-sim generate robots/<name>/ robots/<name>/chain.yaml`
4. Run `verify_kinematics.py` to validate
5. If validation fails, adjust chain.yaml and regenerate

The skill no longer writes raw URDF XML or guesses xyz/rpy offsets.

## Step 7: Enhance `verify_kinematics.py`

**File:** `robots/Meca500-R3/verify_kinematics.py`

Add structured JSON output mode (`--json`) so the skill can programmatically check pass/fail per joint and iterate if needed.

---

## Key files to modify
- `src/robot_arm_sim/models/part.py` — add ConnectionPoint dataclass
- `src/robot_arm_sim/analyze/connections.py` — **new**, core detection logic
- `src/robot_arm_sim/analyze/urdf_generator.py` — **new**, URDF generation from chain spec
- `src/robot_arm_sim/analyze/__init__.py` — integrate connection detection
- `src/robot_arm_sim/analyze/yaml_writer.py` — serialize connection points
- `src/robot_arm_sim/__main__.py` — add `generate` subcommand
- `.claude/skills/assembly-reasoning.md` — rewrite to use chain.yaml workflow
- `robots/Meca500-R3/verify_kinematics.py` — add JSON output

## Verification
1. Run `robot-arm-sim analyze robots/Meca500-R3/` — check connection_points appear in YAMLs
2. Write `chain.yaml` manually or via skill, run `robot-arm-sim generate`
3. Run `verify_kinematics.py` — all joints within 2mm of expected positions
4. Run `robot-arm-sim simulate robots/Meca500-R3/` — visual comparison against real photo
