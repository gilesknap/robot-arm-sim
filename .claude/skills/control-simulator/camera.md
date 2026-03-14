# Camera Control

Control the NiceGUI Three.js 3D camera programmatically to get the right view of the robot for inspection. **Always use JavaScript** — mouse dragging risks accidentally moving joint sliders.

## Finding the Scene Component

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
const cam = sc.camera;
const ctrl = sc.controls;
```

## Setting the Camera

```javascript
// cam.position.set(x, y, z)  — where the camera sits (metres)
// ctrl.target.set(x, y, z)   — what the camera looks at (metres)
// ctrl.update()               — MUST call after changing position/target

cam.position.set(0, -0.5, 0.2);
ctrl.target.set(0, 0, 0.2);
ctrl.update();
```

## Coordinate System

- **X**: left(-) / right(+) when facing the robot front
- **Y**: front(-) / back(+) — negative Y is toward the viewer in default view
- **Z**: down(0) / up(+) — Z=0 is ground plane

## Orthogonal Named Views (ViewCube convention)

Camera is placed at `target + direction * dist` where `target = (0, 0, 0.22)` and `dist = 0.55`.

| View | Direction (dir) | Camera Up | Camera Position | Use Case |
|------|----------------|-----------|-----------------|----------|
| **FRONT** | `(1, 0, 0)` | `(0,0,1)` | `(0.55, 0, 0.22)` | Compare with manufacturer front view |
| **BACK** | `(-1, 0, 0)` | `(0,0,1)` | `(-0.55, 0, 0.22)` | Rear inspection |
| **RIGHT** | `(0, -1, 0)` | `(0,0,1)` | `(0, -0.55, 0.22)` | Right side profile |
| **LEFT** | `(0, 1, 0)` | `(0,0,1)` | `(0, 0.55, 0.22)` | Left side profile |
| **TOP** | `(0, 0, 1)` | `(0,-1,0)` | `(0, 0, 0.77)` | Plan view from above |
| **BOTTOM** | `(0, 0, -1)` | `(0,1,0)` | `(0, 0, -0.33)` | Underside |

**Important**: When using named views, also set `cam.up` to match the view's "Camera Up" vector, otherwise the orientation will be wrong.

## Switching to Orthographic Mode

```javascript
const THREE = window.__THREE_REF;
const canvas = sc.renderer.domElement;
const aspect = canvas.width / canvas.height;

if (cam.isPerspectiveCamera) {
    const d = cam.position.distanceTo(ctrl.target);
    const fovRad = (cam.fov * Math.PI) / 180;
    const frustumH = 2 * d * Math.tan(fovRad / 2);
    const frustumW = frustumH * aspect;
    const ortho = new THREE.OrthographicCamera(
        -frustumW/2, frustumW/2, frustumH/2, -frustumH/2, 0.001, 100
    );
    ortho.position.copy(cam.position);
    ortho.quaternion.copy(cam.quaternion);
    ortho.up.copy(cam.up);
    ortho.zoom = 1;
    ortho.updateProjectionMatrix();
    sc.camera = ortho;
    ctrl.object = ortho;
    ctrl.update();
}
```

To switch back to perspective:
```javascript
if (cam.isOrthographicCamera) {
    const canvas = sc.renderer.domElement;
    const aspect = canvas.width / canvas.height;
    const persp = new THREE.PerspectiveCamera(50, aspect, 0.01, 100);
    persp.position.copy(cam.position);
    persp.quaternion.copy(cam.quaternion);
    persp.up.copy(cam.up);
    persp.updateProjectionMatrix();
    sc.camera = persp;
    ctrl.object = persp;
    ctrl.update();
}
```

## Ortho Named View One-liner Template

Complete JS block to set an orthographic named view for manufacturer image comparison:

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
const THREE = window.__THREE_REF;
const cam = sc.camera;
const ctrl = sc.controls;
const canvas = sc.renderer.domElement;
const aspect = canvas.width / canvas.height;

// --- Edit these for your desired view ---
const dir = [1, 0, 0];       // FRONT (see table above)
const up = [0, 0, 1];        // camera up for this view
const target = [0, 0, 0.22]; // robot center
const dist = 0.55;
// -----------------------------------------

const camPos = [target[0]+dir[0]*dist, target[1]+dir[1]*dist, target[2]+dir[2]*dist];

// Switch to ortho if needed
let activeCam = cam;
if (cam.isPerspectiveCamera) {
    const d = cam.position.distanceTo(ctrl.target);
    const fovRad = (cam.fov * Math.PI) / 180;
    const frustumH = 2 * d * Math.tan(fovRad / 2);
    const frustumW = frustumH * aspect;
    activeCam = new THREE.OrthographicCamera(
        -frustumW/2, frustumW/2, frustumH/2, -frustumH/2, 0.001, 100
    );
    sc.camera = activeCam;
    ctrl.object = activeCam;
}

activeCam.position.set(...camPos);
activeCam.up.set(...up);
ctrl.target.set(...target);

// Adjust ortho frustum for this distance
const fovRad = (50 * Math.PI) / 180;
const frustumH = 2 * dist * Math.tan(fovRad / 2);
const frustumW = frustumH * aspect;
activeCam.left = -frustumW / 2;
activeCam.right = frustumW / 2;
activeCam.top = frustumH / 2;
activeCam.bottom = -frustumH / 2;
activeCam.updateProjectionMatrix();
ctrl.update();
'ortho view set'
```

## Perspective One-liner Template

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

## Preset Views (legacy perspective)

Robot base at origin, ~0.46m tall at zero config.

| View | Camera Position | Target | Use Case |
|------|----------------|--------|----------|
| **Front** | `(0, -0.6, 0.25)` | `(0, 0, 0.25)` | Overall zero-config check |
| **Side** | `(-0.6, 0, 0.25)` | `(0, 0, 0.25)` | Side profile, connection circle alignment |
| **Side (close, base)** | `(-0.35, 0, 0.13)` | `(0, 0, 0.13)` | Inspect joint_1 / joint_2 area |
| **Side (close, elbow)** | `(-0.35, 0, 0.27)` | `(0, 0, 0.27)` | Inspect joint_3 area |
| **Side (close, wrist)** | `(-0.25, 0, 0.40)` | `(0, 0, 0.40)` | Inspect joint_4/5/6 area |
| **Top-down** | `(0, 0, 0.8)` | `(0, 0, 0.25)` | Check base rotation centering |
| **3/4 view** | `(-0.4, -0.4, 0.3)` | `(0, 0, 0.25)` | General 3D perspective |

## Close-up Formula for Any Joint

For a joint at FK position `(jx, jy, jz)` mm:

```javascript
const jz_m = jz / 1000;
cam.position.set(-0.3, 0, jz_m);
ctrl.target.set(jx/1000, jy/1000, jz_m);
ctrl.update();
```

Adjust -0.3 to zoom: -0.2 = very close, -0.5 = wider.

## Reading Current Camera State

```javascript
JSON.stringify({
    pos: {x: cam.position.x, y: cam.position.y, z: cam.position.z},
    target: {x: ctrl.target.x, y: ctrl.target.y, z: ctrl.target.z}
});
```

## Tips

- **Always use JavaScript** to move the camera. Mouse dragging risks accidentally moving joint sliders.
- **Combine findSceneComp + camera set** in a single `javascript_tool` call.
- **After setting camera**, take a screenshot to verify the view.
- **ctrl.update() is required** — without it the view won't refresh.
