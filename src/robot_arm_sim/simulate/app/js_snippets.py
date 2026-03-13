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
    joints.forEach(name => {
        const ax = new THREE.AxesHelper(0.05);
        ax.visible = false;
        sc.scene.add(ax);
        helpers[name] = ax;
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
    const lines = {};
    const connections = CONNECTION_DATA;
    for (const b of connections) {
        const r = 0.005;
        const color = b.end === 'proximal'
            ? 0x00cc00 : 0xcc0000;
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
        const L = r * 4;
        const pts = [
            new THREE.Vector3(0, -L, 0),
            new THREE.Vector3(0, L, 0)];
        const lg = new THREE.BufferGeometry()
            .setFromPoints(pts);
        const lm = new THREE.LineBasicMaterial({
            color: color, linewidth: 2});
        const ln = new THREE.Line(lg, lm);
        ln.visible = false;
        sc.scene.add(ln);
        lines[b.id] = ln;
    }
    window.__connSpheres = spheres;
    window.__connLines = lines;
    window.__setConnectionsVisible = function(show) {
        Object.values(spheres).forEach(
            s => { s.visible = show; });
        Object.values(lines).forEach(
            l => { l.visible = show; });
    };
    window.__updateConnectionPoses = function(data) {
        for (const [id, pos] of Object.entries(data)) {
            const s = spheres[id];
            const ln = lines[id];
            if (!s) continue;
            const vis = pos[3] > 0;
            s.position.set(pos[0], pos[1], pos[2]);
            s.visible = vis;
            if (ln) {
                ln.position.set(
                    pos[0], pos[1], pos[2]);
                if (pos.length >= 8) {
                    ln.quaternion.set(
                        pos[4], pos[5],
                        pos[6], pos[7]);
                }
                ln.visible = vis;
            }
        }
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

    // Connection edit markers (dynamic green/red spheres)
    const connEditMarkers = {};
    window.__connEditMarkers = connEditMarkers;

    window.__placeConnEditMarker = function(
        id, x, y, z, colorHex
    ) {
        let m = connEditMarkers[id];
        if (!m) {
            const g = new THREE.SphereGeometry(
                1.0, 16, 12);
            const mt = new THREE.MeshStandardMaterial({
                color: colorHex, metalness: 0.3,
                roughness: 0.6, transparent: true,
                opacity: 0.9
            });
            m = new THREE.Mesh(g, mt);
            sc.scene.add(m);
            connEditMarkers[id] = m;
        }
        m.scale.setScalar(0.005);
        m.material.color.setHex(colorHex);
        m.position.set(x, y, z);
        m.visible = true;
    };

    window.__removeConnEditMarker = function(id) {
        const m = connEditMarkers[id];
        if (m) {
            sc.scene.remove(m);
            m.geometry.dispose();
            m.material.dispose();
            delete connEditMarkers[id];
        }
    };

    window.__setFaceMarkersVisible = function(show) {
        window.__faceEditMode = show;
        Object.values(markers).forEach(
            m => { m.visible = show; });
        Object.values(connEditMarkers).forEach(
            m => { m.visible = show; });
    };

    window.__setConnEditMarkersVisible = function(show) {
        Object.values(connEditMarkers).forEach(
            m => { m.visible = show; });
    };

    window.__setConnEditMarkerVisibility = function(visMap) {
        for (const [id, vis] of Object.entries(connEditMarkers)) {
            vis.visible = !!visMap[id];
        }
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

    // Click: raycast markers first, then STL meshes
    sc.renderer.domElement.addEventListener(
        'click', function(e) {
        if (!window.__faceEditMode) return;
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
