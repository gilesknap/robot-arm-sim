"""JavaScript snippets for the 3D simulator.

These are injected into the browser to set up PBR materials, environment maps,
coordinate frame helpers, connection markers, face markers, and transparency controls.
"""

# Upgrades materials to PBR metal, generates a studio HDRI env map,
# tweaks lighting, and sets middle-click to pan.
POST_INIT_JS = """
import('nicegui-scene').then(SceneLib => {
    const THREE = SceneLib.default
        ? SceneLib.default.THREE : SceneLib.THREE;
    if (!THREE) return;

    // Find scene component via Vue tree
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene) { sc = p; return; }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;

    const renderer = sc.renderer;
    const scene = sc.scene;

    // Middle-click = pan
    if (sc.controls) sc.controls.mouseButtons.MIDDLE = 2;

    // Ortho view lock for Edit Connections mode
    window.__lockToOrthoViews = function(lock) {
        const ctrl = sc.controls;
        if (!ctrl) return;
        if (lock) {
            // Snap to nearest orthogonal view
            const cam = sc.camera;
            const dir = new THREE.Vector3();
            cam.getWorldDirection(dir);
            const candidates = [
                { d: [1,0,0], up: [0,0,1] },
                { d: [-1,0,0], up: [0,0,1] },
                { d: [0,1,0], up: [0,0,1] },
                { d: [0,-1,0], up: [0,0,1] },
                { d: [0,0,1], up: [0,-1,0] },
                { d: [0,0,-1], up: [0,1,0] },
            ];
            let best = candidates[0], bestDot = -Infinity;
            for (const c of candidates) {
                const dot = dir.x*c.d[0] + dir.y*c.d[1] + dir.z*c.d[2];
                if (dot > bestDot) { bestDot = dot; best = c; }
            }
            // Switch to ortho if perspective
            const tb = document.getElementById('viewcube-proj-toggle');
            if (cam.isPerspectiveCamera && tb) tb.click();
            // Snap camera
            setTimeout(() => {
                const c2 = sc.camera;
                const t = ctrl.target;
                const dist = c2.position.distanceTo(t);
                c2.position.set(
                    t.x - best.d[0]*dist,
                    t.y - best.d[1]*dist,
                    t.z - best.d[2]*dist
                );
                c2.up.set(best.up[0], best.up[1], best.up[2]);
                ctrl.update();
            }, 50);
            ctrl.enableRotate = false;
        } else {
            ctrl.enableRotate = true;
        }
    };

    // Snap to a specific ortho view by direction
    function snapToOrthoView(viewDir, viewUp) {
        const ctrl = sc.controls;
        if (!ctrl) return;
        const cam = sc.camera;
        const t = ctrl.target;
        const dist = cam.position.distanceTo(t);
        cam.position.set(
            t.x - viewDir[0]*dist,
            t.y - viewDir[1]*dist,
            t.z - viewDir[2]*dist
        );
        cam.up.set(viewUp[0], viewUp[1], viewUp[2]);
        if (cam.isOrthographicCamera) {
            const fovRad = (50 * Math.PI) / 180;
            const frustumH = 2 * dist * Math.tan(fovRad / 2);
            const el = sc.renderer.domElement;
            const aspect = el.width / el.height;
            const frustumW = frustumH * aspect;
            cam.left = -frustumW / 2;
            cam.right = frustumW / 2;
            cam.top = frustumH / 2;
            cam.bottom = -frustumH / 2;
            cam.updateProjectionMatrix();
        }
        ctrl.update();
    }
    window.__snapToOrthoView = snapToOrthoView;

    // Drag-to-rotate in edit mode: swipe over empty space to
    // snap to adjacent ortho view
    (function() {
        // Neighbor map: for each view direction key, what view
        // to go to when dragging left/right/up/down in screen space
        // Key format: "x,y,z" of the camera look-at direction
        // (negated from position offset, i.e. the direction the
        //  camera is looking)
        const neighbors = {
            // Looking along +X (FRONT)
            '1,0,0':  {L:[0,1,0],  R:[0,-1,0], U:[0,0,1],  D:[0,0,-1]},
            // Looking along -X (BACK)
            '-1,0,0': {L:[0,-1,0], R:[0,1,0],  U:[0,0,1],  D:[0,0,-1]},
            // Looking along +Y (LEFT)
            '0,1,0':  {L:[-1,0,0], R:[1,0,0],  U:[0,0,1],  D:[0,0,-1]},
            // Looking along -Y (RIGHT)
            '0,-1,0': {L:[1,0,0],  R:[-1,0,0], U:[0,0,1],  D:[0,0,-1]},
            // Looking along +Z (TOP)
            '0,0,1':  {L:[0,1,0],  R:[0,-1,0], U:[-1,0,0], D:[1,0,0]},
            // Looking along -Z (BOTTOM)
            '0,0,-1': {L:[0,1,0],  R:[0,-1,0], U:[1,0,0],  D:[-1,0,0]},
        };
        const upMap = {
            '1,0,0': [0,0,1],  '-1,0,0': [0,0,1],
            '0,1,0': [0,0,1],  '0,-1,0': [0,0,1],
            '0,0,1': [0,-1,0], '0,0,-1': [0,1,0],
        };

        function getCurrentViewKey() {
            const cam = sc.camera;
            const dir = new THREE.Vector3();
            cam.getWorldDirection(dir);
            // Snap to nearest cardinal axis
            const abs = [Math.abs(dir.x), Math.abs(dir.y), Math.abs(dir.z)];
            const maxI = abs.indexOf(Math.max(...abs));
            const arr = [0,0,0];
            arr[maxI] = dir.getComponent(maxI) > 0 ? 1 : -1;
            return arr.join(',');
        }

        const THRESHOLD = 60; // pixels
        let dragStart = null;
        let swipeFired = false;

        const el = sc.renderer.domElement;
        el.addEventListener('mousedown', function(e) {
            if (!window.__faceEditMode) return;
            if (e.button !== 0) return;
            // Only trigger on empty space — check if a mesh was hit
            const rect = el.getBoundingClientRect();
            const mouse = new THREE.Vector2(
                ((e.clientX - rect.left) / rect.width) * 2 - 1,
                -((e.clientY - rect.top) / rect.height) * 2 + 1
            );
            const ray = new THREE.Raycaster();
            ray.setFromCamera(mouse, sc.camera);
            const stl = window.__stlMeshes || [];
            if (ray.intersectObjects(stl).length > 0) return;
            dragStart = {x: e.clientX, y: e.clientY};
            swipeFired = false;
        });

        el.addEventListener('mousemove', function(e) {
            if (!dragStart || swipeFired) return;
            const dx = e.clientX - dragStart.x;
            const dy = e.clientY - dragStart.y;
            const dist = Math.sqrt(dx*dx + dy*dy);
            if (dist < THRESHOLD) return;

            swipeFired = true;
            const key = getCurrentViewKey();
            const nb = neighbors[key];
            if (!nb) return;

            // Determine dominant direction
            let nextDir;
            if (Math.abs(dx) > Math.abs(dy)) {
                nextDir = dx < 0 ? nb.L : nb.R;
            } else {
                nextDir = dy < 0 ? nb.U : nb.D;
            }
            const nk = nextDir.join(',');
            const nextUp = upMap[nk] || [0,0,1];
            snapToOrthoView(nextDir, nextUp);
        });

        el.addEventListener('mouseup', function() {
            dragStart = null;
            swipeFired = false;
        });
    })();

    // --- Studio HDRI environment map ---
    const w = 256, h = 128;
    const data = new Float32Array(w * h * 4);
    for (let y = 0; y < h; y++) {
        const v = y / (h - 1);
        for (let x = 0; x < w; x++) {
            const u = x / (w - 1);
            const i = (y * w + x) * 4;
            // Neutral sky gradient (warm top, slightly cool bottom)
            let r = 0.45 + 0.15 * (1 - v);
            let g = 0.45 + 0.12 * (1 - v);
            let b = 0.48 + 0.12 * v;
            // Subtle horizon rim band
            const hz = Math.exp(-200 * (v-0.5)*(v-0.5));
            r += hz*0.8; g += hz*0.8; b += hz*0.7;
            // Warm key light (soft)
            const kk = Math.exp(
                -60*((u-0.25)*(u-0.25)+(v-0.35)*(v-0.35)));
            r += kk*1.2; g += kk*1.1; b += kk*0.9;
            // Cool fill light (subtle)
            const fl = Math.exp(
                -60*((u-0.75)*(u-0.75)+(v-0.45)*(v-0.45)));
            r += fl*0.3; g += fl*0.35; b += fl*0.5;
            data[i]=r; data[i+1]=g; data[i+2]=b; data[i+3]=1;
        }
    }
    const envTex = new THREE.DataTexture(
        data, w, h, THREE.RGBAFormat, THREE.FloatType);
    envTex.mapping = THREE.EquirectangularReflectionMapping;
    envTex.needsUpdate = true;

    const pmrem = new THREE.PMREMGenerator(renderer);
    pmrem.compileEquirectangularShader();
    const envMap =
        pmrem.fromEquirectangular(envTex).texture;
    pmrem.dispose(); envTex.dispose();
    scene.environment = envMap;

    // --- Lighting ---
    scene.traverse(obj => {
        if (obj.isAmbientLight) obj.intensity = 0.5;
        if (obj.isDirectionalLight) {
            obj.intensity = 3.0;
            obj.position.set(3, -2, 5);
        }
    });

    // --- PBR metallic materials (skip ground plane) ---
    scene.traverse(obj => {
        if (!obj.isMesh || !obj.material) return;
        const c = obj.material.color
            ? obj.material.color.clone()
            : new THREE.Color(0xb0b0b0);
        if (obj.position.z < 0) {
            // Ground plane — matte, no reflections
            obj.material = new THREE.MeshStandardMaterial({
                color: new THREE.Color(0x909090),
                metalness: 0.0,
                roughness: 1.0,
                envMapIntensity: 0.0,
            });
        } else {
            obj.material = new THREE.MeshStandardMaterial({
                color: c,
                metalness: 0.88,
                roughness: 0.15,
                envMap: envMap,
                envMapIntensity: 0.72,
            });
        }
    });
});
"""


SCREENSHOT_JS = """
(function() {
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene && p.camera) {
                sc = p; return;
            }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;
    sc.renderer.render(sc.scene, sc.camera);
    const url = sc.renderer.domElement.toDataURL('image/png');
    const a = document.createElement('a');
    a.href = url;
    a.download = 'robot-screenshot.png';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
})();
"""


AXES_INIT_JS = """
(async function() {
    const SceneLib = await import('nicegui-scene');
    const THREE = SceneLib.default
        ? SceneLib.default.THREE : SceneLib.THREE;
    if (!THREE) return;
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene) {
                sc = p; return;
            }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;
    const helpers = {};
    const joints = JOINT_NAMES;
    const L = 0.15;
    const colors = [0xff0000, 0x00ff00, 0x0000ff];
    const dirs = [
        [1,0,0], [0,1,0], [0,0,1]];
    joints.forEach(name => {
        const grp = new THREE.Group();
        grp.visible = false;
        for (let i = 0; i < 3; i++) {
            const d = dirs[i];
            const solidPts = [
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(
                    d[0]*L, d[1]*L, d[2]*L)];
            const sg = new THREE.BufferGeometry()
                .setFromPoints(solidPts);
            const sl = new THREE.Line(sg,
                new THREE.LineBasicMaterial({
                    color: colors[i]}));
            grp.add(sl);
            const dashPts = [
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(
                    -d[0]*L, -d[1]*L, -d[2]*L)];
            const dg = new THREE.BufferGeometry()
                .setFromPoints(dashPts);
            const dl = new THREE.Line(dg,
                new THREE.LineDashedMaterial({
                    color: colors[i],
                    dashSize: 0.008,
                    gapSize: 0.006}));
            dl.computeLineDistances();
            grp.add(dl);
        }
        sc.scene.add(grp);
        helpers[name] = grp;
    });
    window.__axesHelpers = helpers;
    window.__setAxesVisible = function(show) {
        Object.values(helpers).forEach(
            h => { h.visible = show; });
    };
    window.__updateAxesPoses = function(data) {
        for (const [name, info] of Object.entries(data)) {
            const h = helpers[name];
            if (!h) continue;
            h.position.set(
                info.p[0], info.p[1], info.p[2]);
            h.quaternion.set(
                info.q[0], info.q[1],
                info.q[2], info.q[3]);
        }
    };
})();
"""


CONNECTION_INIT_JS = """
(async function() {
    const SceneLib = await import('nicegui-scene');
    const THREE = SceneLib.default
        ? SceneLib.default.THREE : SceneLib.THREE;
    if (!THREE) return;
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene) {
                sc = p; return;
            }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;
    const spheres = {};
    const connections = CONNECTION_DATA;
    for (const b of connections) {
        const r = 0.005;
        const color = b.centering === 'center'
            ? 0x4488ff
            : (b.end === 'proximal' ? 0x00cc00 : 0xcc0000);
        const geo = new THREE.SphereGeometry(r, 16, 12);
        const mat = new THREE.MeshStandardMaterial({
            color: color, metalness: 0.3,
            roughness: 0.6, transparent: true,
            opacity: 0.85
        });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.visible = false;
        sc.scene.add(mesh);
        spheres[b.id] = mesh;
    }
    window.__connSpheres = spheres;
    window.__connLines = {};
    window.__setConnectionsVisible = function(show) {
        Object.values(spheres).forEach(
            s => { s.visible = show; });
    };
    window.__updateConnectionPoses = function(data) {
        for (const [id, pos] of Object.entries(data)) {
            const s = spheres[id];
            if (!s) continue;
            const vis = pos[3] > 0;
            s.position.set(pos[0], pos[1], pos[2]);
            s.visible = vis;
        }
    };
    window.__moveConnectionSphere = function(
        id, x, y, z, colorHex
    ) {
        const s = spheres[id];
        if (!s) return false;
        s.position.set(x, y, z);
        s.visible = true;
        if (colorHex !== undefined) {
            s.material.color.setHex(colorHex);
        }
        return true;
    };
})();
"""


FACE_MARKER_INIT_JS = """
(async function() {
    const SceneLib = await import('nicegui-scene');
    const THREE = SceneLib.default
        ? SceneLib.default.THREE : SceneLib.THREE;
    if (!THREE) return;
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene) {
                sc = p; return;
            }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;

    // Tag STL meshes with link names via object_id
    const meshMap = MESH_LINK_MAP;
    const stlMeshes = [];
    for (const [objId, linkName] of Object.entries(meshMap)) {
        const obj = sc.objects.get(objId);
        if (!obj) continue;
        obj.traverse(child => {
            if (child.isMesh) {
                child.userData.linkName = linkName;
                stlMeshes.push(child);
            }
        });
    }
    window.__stlMeshes = stlMeshes;

    const markers = {};
    const faces = FACE_DATA;
    for (const f of faces) {
        const geo = new THREE.SphereGeometry(0.008, 12, 8);
        const mat = new THREE.MeshStandardMaterial({
            color: 0xffcc00,
            metalness: 0.3, roughness: 0.6,
            transparent: true, opacity: 0.85
        });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.name = f.id;
        mesh.visible = false;
        sc.scene.add(mesh);
        markers[f.id] = mesh;
    }
    window.__faceMarkers = markers;
    window.__faceEditMode = false;
    window.__lastFaceClick = null;

    window.__setFaceMarkersVisible = function(show) {
        window.__faceEditMode = show;
        Object.values(markers).forEach(
            m => { m.visible = show; });
    };

    window.__updateFaceMarkerPoses = function(data) {
        for (const [id, pos] of Object.entries(data)) {
            const m = markers[id];
            if (!m) continue;
            const vis = pos[3] > 0;
            m.position.set(pos[0], pos[1], pos[2]);
            m.visible = vis && window.__faceEditMode;
        }
    };

    window.__setFaceMarkerColor = function(id, colorHex) {
        const m = markers[id];
        if (m) m.material.color.setHex(colorHex);
    };

    // --- Move Parts drag mode ---
    let dragEnabled = true;
    let dragging = false;
    let dragMesh = null;
    let dragLinkName = null;
    let dragStartMouse = null;
    let dragStartPos = null;
    let selectedMesh = null;
    let selectedLinkName = null;
    window.__lastPartMove = null;

    window.__enablePartDrag = function(enable) {
        dragEnabled = enable;
        if (!enable) {
            dragging = false;
            dragMesh = null;
            selectedMesh = null;
            selectedLinkName = null;
        }
    };

    function getViewPlaneAxes() {
        // Determine which two axes are in the view plane
        // based on the camera direction (ortho lock ensures
        // camera faces along one cardinal axis)
        const cam = sc.camera;
        const dir = new THREE.Vector3();
        cam.getWorldDirection(dir);
        const ax = Math.abs(dir.x);
        const ay = Math.abs(dir.y);
        const az = Math.abs(dir.z);
        // The dominant axis is the view axis; the other two
        // are the drag plane axes
        if (ax > ay && ax > az) return {u: 'y', v: 'z'};
        if (ay > az) return {u: 'x', v: 'z'};
        return {u: 'x', v: 'y'};
    }

    function screenToWorld(e, refPos) {
        const rect = sc.renderer.domElement.getBoundingClientRect();
        const mouse = new THREE.Vector2(
            ((e.clientX - rect.left) / rect.width) * 2 - 1,
            -((e.clientY - rect.top) / rect.height) * 2 + 1
        );
        const ray = new THREE.Raycaster();
        ray.setFromCamera(mouse, sc.camera);
        // Intersect with plane at refPos perpendicular to camera
        const cam = sc.camera;
        const normal = new THREE.Vector3();
        cam.getWorldDirection(normal);
        const plane = new THREE.Plane();
        plane.setFromNormalAndCoplanarPoint(normal, refPos);
        const pt = new THREE.Vector3();
        ray.ray.intersectPlane(plane, pt);
        return pt;
    }

    sc.renderer.domElement.addEventListener(
        'mousedown', function(e) {
        if (!dragEnabled || !window.__faceEditMode) return;
        if (e.button !== 0) return;
        const rect = sc.renderer.domElement.getBoundingClientRect();
        const mouse = new THREE.Vector2(
            ((e.clientX - rect.left) / rect.width) * 2 - 1,
            -((e.clientY - rect.top) / rect.height) * 2 + 1
        );
        const ray = new THREE.Raycaster();
        ray.setFromCamera(mouse, sc.camera);
        const hits = ray.intersectObjects(window.__stlMeshes);
        if (hits.length === 0) return;
        const hit = hits[0];
        const ln = hit.object.userData.linkName;
        if (!ln) return;
        dragging = true;
        dragMesh = hit.object;
        dragLinkName = ln;
        selectedMesh = hit.object;
        selectedLinkName = ln;
        window.__lastPartSelect = {linkName: ln};
        dragStartPos = dragMesh.position.clone();
        dragStartMouse = screenToWorld(e, dragStartPos);
        e.stopPropagation();
        e.preventDefault();
    }, true);

    sc.renderer.domElement.addEventListener(
        'mousemove', function(e) {
        if (!dragging || !dragMesh || !dragStartMouse) return;
        const current = screenToWorld(e, dragStartPos);
        if (!current) return;
        const delta = current.clone().sub(dragStartMouse);
        // Constrain to view plane axes only
        const axes = getViewPlaneAxes();
        const newPos = dragStartPos.clone();
        newPos[axes.u] += delta[axes.u];
        newPos[axes.v] += delta[axes.v];
        dragMesh.position.copy(newPos);
    });

    sc.renderer.domElement.addEventListener(
        'mouseup', function(e) {
        if (!dragging || !dragMesh) return;
        const delta = dragMesh.position.clone().sub(dragStartPos);
        window.__lastPartMove = {
            linkName: dragLinkName,
            delta: [delta.x, delta.y, delta.z]
        };
        dragging = false;
        dragMesh = null;
    });

    // Arrow-key nudge: move selected part 0.1 mm per keypress
    document.addEventListener('keydown', function(e) {
        if (!dragEnabled || !window.__faceEditMode) return;
        if (!selectedMesh || !selectedLinkName) return;
        const key = e.key;
        if (!['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(key))
            return;
        e.preventDefault();
        e.stopPropagation();
        const step = 0.0001;  // 0.1 mm in metres
        const axes = getViewPlaneAxes();
        const startPos = selectedMesh.position.clone();
        if (key === 'ArrowRight') selectedMesh.position[axes.u] += step;
        if (key === 'ArrowLeft')  selectedMesh.position[axes.u] -= step;
        if (key === 'ArrowUp')    selectedMesh.position[axes.v] += step;
        if (key === 'ArrowDown')  selectedMesh.position[axes.v] -= step;
        const delta = selectedMesh.position.clone().sub(startPos);
        window.__lastPartMove = {
            linkName: selectedLinkName,
            delta: [delta.x, delta.y, delta.z]
        };
    });

    // Track mouse movement to distinguish clicks from drags/rotates
    let _mdX = 0, _mdY = 0, _wasDrag = false;
    sc.renderer.domElement.addEventListener(
        'mousedown', function(e) {
        _mdX = e.clientX; _mdY = e.clientY; _wasDrag = false;
    });
    sc.renderer.domElement.addEventListener(
        'mouseup', function(e) {
        const dx = e.clientX - _mdX, dy = e.clientY - _mdY;
        if (dx*dx + dy*dy > 9) _wasDrag = true;
    });

    // Click: raycast markers first, then STL meshes
    sc.renderer.domElement.addEventListener(
        'click', function(e) {
        if (!window.__faceEditMode) return;
        if (_wasDrag) return;
        const rect = sc.renderer.domElement
            .getBoundingClientRect();
        const mouse = new THREE.Vector2(
            ((e.clientX - rect.left) / rect.width)
                * 2 - 1,
            -((e.clientY - rect.top) / rect.height)
                * 2 + 1);
        const ray = new THREE.Raycaster();
        ray.setFromCamera(mouse, sc.camera);

        // Try STL meshes
        const sHits = ray.intersectObjects(
            window.__stlMeshes);
        if (sHits.length > 0) {
            const hit = sHits[0];
            const ln = hit.object.userData.linkName;
            if (ln) {
                const n = hit.face
                    ? hit.face.normal : {x:0,y:0,z:1};
                window.__lastFaceClick = {
                    type: 'mesh',
                    linkName: ln,
                    faceIndex: hit.faceIndex,
                    point: [
                        hit.point.x,
                        hit.point.y,
                        hit.point.z
                    ],
                    normal: [n.x, n.y, n.z]
                };
            }
        }
    });
})();
"""


BEFOREUNLOAD_JS = """
(function() {
    window.__simJointAngles = {};
    window.addEventListener('beforeunload', function() {
        var appEl = document.getElementById('app');
        if (!appEl || !appEl.__vue_app__) return;
        var sc = null;
        function walk(n, d) {
            if (d > 30 || sc) return;
            if (n.component) {
                var p = n.component.proxy;
                if (p && p.renderer && p.scene && p.controls) {
                    sc = p; return;
                }
                if (n.component.subTree)
                    walk(n.component.subTree, d+1);
            }
            if (Array.isArray(n.children))
                n.children.forEach(function(c) {
                    if (c && typeof c === 'object')
                        walk(c, d+1);
                });
        }
        walk(
            appEl.__vue_app__._container._vnode,
            0
        );
        var cam = null;
        if (sc) {
            var c = sc.camera;
            var t = sc.controls;
            cam = {
                px: c.position.x,
                py: c.position.y,
                pz: c.position.z,
                tx: t.target.x,
                ty: t.target.y,
                tz: t.target.z,
                ux: c.up.x,
                uy: c.up.y,
                uz: c.up.z,
                isOrtho: !!c.isOrthographicCamera,
                left: c.left,
                right: c.right,
                top: c.top,
                bottom: c.bottom
            };
        }
        sessionStorage.setItem(
            'sim_state',
            JSON.stringify({
                joints: window.__simJointAngles,
                camera: cam
            })
        );
    });
})();
"""


SCENE_RESIZE_JS = """
(function() {
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene && p.camera) {
                sc = p; return;
            }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;

    const wrapper = document.querySelector('.sim-scene-wrapper');
    if (!wrapper) return;
    const canvas = sc.renderer.domElement;
    const sceneDiv = canvas.parentElement;

    function resize() {
        const w = wrapper.clientWidth;
        const h = wrapper.clientHeight;
        if (w <= 0 || h <= 0) return;
        sceneDiv.style.width = w + 'px';
        sceneDiv.style.height = h + 'px';
        sc.renderer.setSize(w, h);
        if (sc.text_renderer)
            sc.text_renderer.setSize(w, h);
        const cam = sc.camera;
        if (cam.isPerspectiveCamera) {
            cam.aspect = w / h;
            cam.updateProjectionMatrix();
        } else if (cam.isOrthographicCamera) {
            const fH = cam.top - cam.bottom;
            const fW = fH * (w / h);
            cam.left = -fW / 2;
            cam.right = fW / 2;
            cam.updateProjectionMatrix();
        }
    }

    const obs = new ResizeObserver(resize);
    obs.observe(wrapper);
    resize();
})();
"""


LABEL_CALLOUT_JS = """
(async function() {
    const SceneLib = await import('nicegui-scene');
    const THREE = SceneLib.default
        ? SceneLib.default.THREE : SceneLib.THREE;
    if (!THREE) return;
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene) {
                sc = p; return;
            }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;

    const canvas = sc.renderer.domElement;
    const wrapper = canvas.closest('.sim-scene-wrapper');
    if (!wrapper) return;
    wrapper.style.position = 'relative';

    // Remove existing overlay (hot reload)
    const old = document.getElementById('label-overlay');
    if (old) old.remove();

    // Overlay container
    const overlay = document.createElement('div');
    overlay.id = 'label-overlay';
    overlay.style.cssText =
        'position:absolute;top:0;left:0;width:100%;'
        + 'height:100%;pointer-events:none;overflow:hidden;'
        + 'display:none;z-index:5;';

    // SVG for leader lines
    const svgNS = 'http://www.w3.org/2000/svg';
    const svg = document.createElementNS(svgNS, 'svg');
    svg.style.cssText =
        'position:absolute;top:0;left:0;'
        + 'width:100%;height:100%;';
    overlay.appendChild(svg);

    // Left panel — part labels
    const leftPanel = document.createElement('div');
    leftPanel.style.cssText =
        'position:absolute;left:8px;top:8px;bottom:40px;'
        + 'display:flex;flex-direction:column;'
        + 'justify-content:space-around;gap:2px;';
    overlay.appendChild(leftPanel);

    // Bottom panel — joint labels
    const bottomPanel = document.createElement('div');
    bottomPanel.style.cssText =
        'position:absolute;bottom:8px;left:100px;right:8px;'
        + 'display:flex;flex-direction:row;'
        + 'justify-content:space-around;gap:4px;';
    overlay.appendChild(bottomPanel);

    // Build labels from metadata
    const labels = LABEL_DATA;
    const labelEls = {};
    const lineEls = {};

    function makeLabel(lb, parent) {
        const el = document.createElement('div');
        el.textContent = lb.name;
        const isJ = lb.is_joint;
        const color = isJ ? '#C62828' : '#1565C0';
        el.style.cssText =
            'font-size:' + (isJ ? '11' : '13') + 'px;'
            + 'font-weight:bold;color:' + color + ';'
            + 'background:rgba(255,255,255,0.92);'
            + 'padding:2px 6px;border-radius:3px;'
            + 'border:1px solid ' + color + ';'
            + 'white-space:nowrap;';
        parent.appendChild(el);
        labelEls[lb.id] = el;

        const line = document.createElementNS(svgNS, 'line');
        line.setAttribute('stroke', color);
        line.setAttribute('stroke-width', '1.5');
        line.setAttribute('stroke-dasharray', '4,3');
        line.setAttribute('opacity', '0');
        svg.appendChild(line);
        lineEls[lb.id] = line;
    }

    const partLabels = labels.filter(l => !l.is_joint).reverse();
    const jointLabels = labels.filter(l => l.is_joint);
    for (const lb of partLabels) makeLabel(lb, leftPanel);
    for (const lb of jointLabels) makeLabel(lb, bottomPanel);

    wrapper.appendChild(overlay);

    let anchors = {};
    const v3 = new THREE.Vector3();

    window.__updateLabelAnchors = function(data) {
        anchors = data;
    };
    window.__setLabelsVisible = function(show) {
        overlay.style.display = show ? 'block' : 'none';
    };

    function tick() {
        requestAnimationFrame(tick);
        if (overlay.style.display === 'none') return;

        const oRect = overlay.getBoundingClientRect();
        const cRect = canvas.getBoundingClientRect();
        const ox = cRect.left - oRect.left;
        const oy = cRect.top - oRect.top;
        const W = cRect.width;
        const H = cRect.height;
        if (W <= 0 || H <= 0) return;

        const cam = sc.camera;

        for (const lb of labels) {
            const a = anchors[lb.id];
            const line = lineEls[lb.id];
            if (!a) {
                line.setAttribute('opacity', '0');
                continue;
            }

            v3.set(a[0], a[1], a[2]);
            v3.project(cam);

            // Behind camera
            if (v3.z > 1) {
                line.setAttribute('opacity', '0');
                continue;
            }
            line.setAttribute('opacity', '0.5');

            // NDC to screen (relative to overlay)
            const sx = ox + (v3.x * 0.5 + 0.5) * W;
            const sy = oy + (-v3.y * 0.5 + 0.5) * H;

            // Label position (relative to overlay)
            const el = labelEls[lb.id];
            const elRect = el.getBoundingClientRect();
            let lx, ly;
            if (lb.is_joint) {
                lx = elRect.left + elRect.width / 2
                    - oRect.left;
                ly = elRect.top - oRect.top;
            } else {
                lx = elRect.right - oRect.left;
                ly = elRect.top + elRect.height / 2
                    - oRect.top;
            }

            line.setAttribute('x1', lx);
            line.setAttribute('y1', ly);
            line.setAttribute('x2', sx);
            line.setAttribute('y2', sy);
        }
    }
    tick();
})();
"""


TRANSPARENCY_INIT_JS = """
(async function() {
    const SceneLib = await import('nicegui-scene');
    const THREE = SceneLib.default
        ? SceneLib.default.THREE : SceneLib.THREE;
    if (!THREE) return;
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let sc = null;
    function walk(n, d) {
        if (d > 30 || sc) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene) {
                sc = p; return;
            }
            if (n.component.subTree)
                walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(
                c => c && typeof c === 'object'
                    && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!sc) return;
    window.__setMeshTransparency = function(opacity) {
        sc.scene.traverse(function(obj) {
            if (!obj.isMesh || !obj.material) return;
            if (obj.position.z < 0) return;
            obj.material.transparent = true;
            obj.material.opacity = opacity;
            obj.material.depthWrite = opacity > 0.9;
            obj.material.needsUpdate = true;
        });
    };
})();
"""
