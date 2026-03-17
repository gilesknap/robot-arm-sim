# Nudge Parts

Move selected parts with arrow keys in Edit Connections mode.

## Selecting a Part

Click on the mesh in the 3D viewport. The status bar (bottom-right) shows the selected part name.

## Arrow Key Movement

| Key | Step Size |
|-----|-----------|
| Arrow key | 0.1mm per press |
| Shift + Arrow key | 2mm per press |

The arrow keys move the **part**, not the canvas:
- **ArrowUp / ArrowDown** = move part vertically on screen (Z axis in side views)
- **ArrowLeft / ArrowRight** = move part horizontally on screen (X axis in side views)

The axis mapping depends on the current ortho view:

| View | ArrowLeft/Right | ArrowUp/Down |
|------|----------------|--------------|
| LEFT / RIGHT | X axis | Z axis |
| FRONT / BACK | Y axis | Z axis |
| TOP / BOTTOM | X axis | Y axis |

## Alignment Strategy

Two complementary visual cues for correct placement:

### 1. Bore centering
The frame crosspoint (where RGB axes meet) should be in the **center of the part's rotational bore/bearing**. Zoom in to the bore and nudge until the crosspoint is centered in the circle.

### 2. Face mating
Adjacent parts should have their mating surfaces flush:
- Cylindrical walls of adjacent parts should be co-linear
- Flat mating faces should touch with no visible gap
- The gap between parts should be <1mm

### Workflow
1. Zoom into the bore area using the `zoom` tool to assess offset
2. Make coarse corrections with Shift+Arrow (2mm steps)
3. Re-zoom to check progress
4. Fine-tune with plain Arrow (0.1mm steps)
5. Check from a second ortho view (e.g. FRONT after LEFT) to verify the depth axis
6. Use Fit to see overall appearance after each adjustment

## Saving

Click **Save & Rebuild** to persist the offsets to chain.yaml and regenerate the URDF.

## Notes

- Always work in Edit Connections mode (enforces ortho)
- Arrow keys only work when a part is selected (check status bar)
- After Save & Rebuild, the page re-renders — re-enter edit mode to continue
- Use `zoom` tool on the bore area for precise assessment rather than judging from the full view
