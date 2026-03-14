# 01 — Analyze STL Files

Run the STL analysis pipeline to extract geometry, connection points, and assembly hints.

## Input

- `robots/<name>/stl_files/` containing STL mesh files (mm, Z-up)
- override. If the user specifies override then use --override-manual on the `robot-arm-sim analyze` command. (removes manual mesh translations)

## Steps

1. Run analysis:
   ```bash
   uv run robot-arm-sim analyze robots/<name>/
   ```

2. Verify output:
   - `robots/<name>/analysis/summary.yaml` exists with part list and assembly hints
   - Each `robots/<name>/analysis/<part>.yaml` has `connection_points` entries
   - Connection points show bore/shaft centres with axis directions

3. If analysis fails or connection points are missing:
   - Check STL files are valid (not empty, in mm, Z-up)
   - Files can use any naming: sequential (A0, A1, ...) or semantic (base, shoulder, ...)
   - The `mesh` field in chain.yaml must match the STL file stem

## Output

- `robots/<name>/analysis/summary.yaml` — part list, role hints, assembly hints
- `robots/<name>/analysis/<part>.yaml` — per-part geometry, features, connection_points

## Gate

Every STL file has a corresponding YAML in `analysis/` with at least one `connection_points` entry.

## Caution

Bore detection on low-poly collision meshes can be unreliable — it may find motor housings instead of kinematic bores. Connection points are useful for **validation** but should not be the primary source for joint origins. The DH parameters from `02-research-specs` take precedence.

### Barrel-shaped parts (upperarm, forearm, etc.)

Barrel-shaped parts require special bore detection logic:
- The bore axis comes from the barrel filter's **removed** face groups (faces that didn't pass the barrel curvature test)
- Flat face normals aligned with the bore axis reliably indicate bore openings — look for `barrel_bore_face` in the analysis YAML
- Method: `barrel_bore_face` means the bore was derived from flat-face evidence rather than cross-section analysis
- Bore positions are placed on the **flat face surface** (face centroid), not at the bbox center — this is consistent with `cross_section` method and ensures bore markers appear at the physical bore opening
- If barrel bore detection fails, it falls through to standard cross-section detection
