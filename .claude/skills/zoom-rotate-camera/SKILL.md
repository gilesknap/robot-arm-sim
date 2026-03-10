Base directory for this skill: /workspaces/robot-arm-sim/.claude/skills/zoom-rotate-camera

# Camera Control Skill

Control the NiceGUI Three.js 3D camera programmatically to get the right view of the robot for inspection.

## When to Use

Use this skill when you need to position the camera to inspect a specific joint, part, or area of the robot in the simulator. Avoids accidental slider interaction from mouse dragging.

## How It Works

The simulator uses Three.js with a PerspectiveCamera and OrbitControls. You can set camera position and look-at target via JavaScript.

### Finding the Scene Component

```javascript
function findSceneComp(vnode) {
    if (!vnode) return null;
    if (vnode.component && vnode.component.proxy) {
        const p = vnode.component.proxy;
        if (p.renderer && p.scene && p.controls) return p;
    }
    if (vnode.children && Array.isArray(vnode.children)) {
        for (const c of vnode.children) {
            const r = findSceneComp(c);
            if (r) return r;
        }
    }
    if (vnode.component && vnode.component.subTree) {
        const r = findSceneComp(vnode.component.subTree);
        if (r) return r;
    }
    return null;
}
const sc = findSceneComp(document.getElementById('app').__vue_app__._container._vnode);
const cam = sc.camera;
const ctrl = sc.controls;
```

### Setting the Camera

```javascript
// cam.position.set(x, y, z)  — where the camera sits (metres)
// ctrl.target.set(x, y, z)   — what the camera looks at (metres)
// ctrl.update()               — MUST call after changing position/target

cam.position.set(0, -0.5, 0.2);
ctrl.target.set(0, 0, 0.2);
ctrl.update();
```

### Coordinate System

- **X**: left(-) / right(+) when facing the robot front
- **Y**: front(-) / back(+) — negative Y is toward the viewer in default view
- **Z**: down(0) / up(+) — Z=0 is ground plane

### Preset Views for Meca500-R3

All presets assume the robot base is at origin. The robot stands ~0.46m tall at zero config.

| View | Camera Position | Target | Use Case |
|------|----------------|--------|----------|
| **Front** | `(0, -0.6, 0.25)` | `(0, 0, 0.25)` | Overall zero-config check |
| **Side** | `(-0.6, 0, 0.25)` | `(0, 0, 0.25)` | Side profile, bore circle alignment |
| **Side (close, base)** | `(-0.35, 0, 0.13)` | `(0, 0, 0.13)` | Inspect joint_1 / joint_2 area |
| **Side (close, elbow)** | `(-0.35, 0, 0.27)` | `(0, 0, 0.27)` | Inspect joint_3 area |
| **Side (close, wrist)** | `(-0.25, 0, 0.40)` | `(0, 0, 0.40)` | Inspect joint_4/5/6 area |
| **Top-down** | `(0, 0, 0.8)` | `(0, 0, 0.25)` | Check base rotation centering |
| **3/4 view** | `(-0.4, -0.4, 0.3)` | `(0, 0, 0.25)` | General 3D perspective |
| **Close-up any joint** | Compute from joint FK position | Same Z as joint | See formula below |

### Close-up Formula for Any Joint

To get a close-up side view of a specific joint at known FK position `(jx, jy, jz)` mm:

```javascript
const jz_m = jz / 1000;  // convert mm to metres
cam.position.set(-0.3, 0, jz_m);
ctrl.target.set(jx/1000, jy/1000, jz_m);
ctrl.update();
```

Adjust the -0.3 (distance) to zoom in/out: -0.2 = very close, -0.5 = wider.

### Reading Current Camera State

```javascript
JSON.stringify({
    pos: {x: cam.position.x, y: cam.position.y, z: cam.position.z},
    target: {x: ctrl.target.x, y: ctrl.target.y, z: ctrl.target.z}
});
```

### Tips

- **Always use JavaScript** to move the camera. Mouse dragging in the 3D viewport risks accidentally moving joint sliders.
- **Combine the findSceneComp + camera set** in a single `javascript_tool` call to avoid redundant round trips.
- **After setting camera**, take a screenshot to verify the view before doing detailed inspection with `zoom`.
- **ctrl.update() is required** — without it the view won't refresh.
- **Zoom tool is still useful** for pixel-level inspection after positioning the camera to the right area.

### One-liner Template

Copy-paste friendly single JS block. NOTE: `v.children` may not always be an array (can be a string or object in Vue vnodes), so always check `Array.isArray`:

```javascript
function findSceneComp(vnode) {
    if (!vnode) return null;
    if (vnode.component && vnode.component.proxy) {
        const p = vnode.component.proxy;
        if (p.renderer && p.scene && p.controls) return p;
    }
    if (vnode.children && Array.isArray(vnode.children)) {
        for (const c of vnode.children) { const r = findSceneComp(c); if (r) return r; }
    }
    if (vnode.component && vnode.component.subTree) return findSceneComp(vnode.component.subTree);
    return null;
}
const sc = findSceneComp(document.getElementById('app').__vue_app__._container._vnode);
sc.camera.position.set(0, -0.5, 0.25);   // <-- edit position
sc.controls.target.set(0, 0, 0.25);       // <-- edit target
sc.controls.update();
'camera set'
```
