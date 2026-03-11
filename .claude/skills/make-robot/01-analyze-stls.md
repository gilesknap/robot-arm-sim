# 01 — Analyze STL Files

Run the STL analysis pipeline to extract geometry, connection points, and assembly hints.

## Input

- `robots/<name>/stl_files/` containing STL mesh files (mm, Z-up)

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
