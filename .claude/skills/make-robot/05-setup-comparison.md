# 05 — Setup Visual Comparison

Map manufacturer reference images to simulator views and take baseline screenshots.

## Prerequisites

- Simulator running on `localhost:8080`
- claude-in-chrome MCP server available for browser screenshots
- Reference images saved in `robots/<name>/images/` (from step 02)
- `zoom-rotate-camera` skill for JS camera templates

## Steps

### 1. Identify Manufacturer's Coordinate Convention

From the reference images, determine:
- Axis arrows on drawings (X/Y/Z labels)
- "Front view" / "Side view" labels
- Physical landmarks (cable exits, logo, teach pendant port)

### 2. Map Views

For each reference image, determine which ViewCube face it corresponds to:

| ViewCube Face | Direction (camera from) | Camera Up |
|--------------|------------------------|-----------|
| FRONT | `(1, 0, 0)` — from +X | `(0,0,1)` |
| BACK | `(-1, 0, 0)` — from -X | `(0,0,1)` |
| RIGHT | `(0, -1, 0)` — from -Y | `(0,0,1)` |
| LEFT | `(0, 1, 0)` — from +Y | `(0,0,1)` |
| TOP | `(0, 0, 1)` — from +Z | `(0,-1,0)` |

### 3. Write view_mapping.yaml

```yaml
views:
  - manufacturer_label: "Front"
    viewcube_face: RIGHT
    reference_image: "images/reference_side_dimensions.png"
    dir: [0, -1, 0]
    up: [0, 0, 1]
  - manufacturer_label: "Side"
    viewcube_face: FRONT
    reference_image: "images/reference_photo_3quarter.png"
    dir: [1, 0, 0]
    up: [0, 0, 1]
```

### 4. Take Baseline Screenshots

For each mapped view:
1. Use the **Ortho Named View One-liner Template** from the `zoom-rotate-camera` skill
2. Set the `dir` and `up` arrays to match the ViewCube face
3. Click "Fit" to ensure the robot fills the viewport
4. Screenshot via browser MCP
5. Save as `robots/<name>/images/baseline_<face>.png`

### 5. Verify Mapping

Compare each baseline screenshot against its reference image:
- The robot should face the same direction
- Proportions should roughly match
- If mapping is wrong, try a different ViewCube face

## Output

- `robots/<name>/view_mapping.yaml`
- Baseline screenshots in `robots/<name>/images/`

## Gate

- At least 2 orthogonal views mapped
- Baseline screenshots taken and roughly match reference image orientation
