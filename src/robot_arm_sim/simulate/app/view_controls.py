"""View control widget — 3D ViewCube gizmo for the simulator viewport.

Renders a small interactive orientation cube (like Blender/Fusion 360)
that syncs with the main camera. Click cube faces to snap to preset views.
Includes an orthographic/perspective toggle.
"""

from __future__ import annotations

from nicegui import ui

# Robot center height (approximate midpoint for look-at target)
_TARGET = (0, 0, 0.22)
# Camera distance from target for preset views
_DIST = 0.55

# The entire ViewCube is implemented in a single JS blob that:
# 1. Finds the main Three.js scene component
# 2. Creates a mini Three.js scene with a labeled cube
# 3. Syncs the cube rotation with the main camera on each frame
# 4. Handles raycasting clicks to snap the main camera to views
_VIEWCUBE_JS = (
    """
(async function() {
    const SceneLib = await import('nicegui-scene');
    const THREE = SceneLib.default ? SceneLib.default.THREE : SceneLib.THREE;
    if (!THREE) return;
    window.__THREE_REF = THREE;

    // Find main scene component
    const appEl = document.getElementById('app');
    if (!appEl || !appEl.__vue_app__) return;
    let mainSC = null;
    function walk(n, d) {
        if (d > 30 || mainSC) return;
        if (n.component) {
            const p = n.component.proxy;
            if (p && p.renderer && p.scene && p.camera) { mainSC = p; return; }
            if (n.component.subTree) walk(n.component.subTree, d+1);
        }
        if (Array.isArray(n.children))
            n.children.forEach(c => c && typeof c === 'object' && walk(c, d+1));
    }
    walk(appEl.__vue_app__._container._vnode, 0);
    if (!mainSC) return;

    // --- Create the ViewCube mini-scene ---
    const container = document.getElementById('viewcube-container');
    if (!container) return;
    const SIZE = 192;

    const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    renderer.setSize(SIZE, SIZE);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setClearColor(0x000000, 0);
    container.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.OrthographicCamera(-1.8, 1.8, 1.8, -1.8, 0.1, 100);
    camera.position.set(0, 0, 5);
    camera.lookAt(0, 0, 0);

    // Lighting
    scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(2, 3, 5);
    scene.add(dirLight);

    // --- Build the cube with labeled faces ---
    const cubeGroup = new THREE.Group();
    scene.add(cubeGroup);

    // Face definitions: label, normal direction, color, rotation to face camera
    const faces = [
        {label:'RIGHT', dir:[0,-1,0], color:0x50B86C,
         up:[0,0,1], textRot:0},
        {label:'LEFT', dir:[0,1,0], color:0x50B86C,
         up:[0,0,1], textRot:Math.PI},
        {label:'FRONT', dir:[1,0,0], color:0x4A90D9,
         up:[0,0,1], textRot:3*Math.PI/2},
        {label:'BACK', dir:[-1,0,0], color:0x4A90D9,
         up:[0,0,1], textRot:Math.PI/2},
        {label:'TOP', dir:[0,0,1], color:0xE8734A,
         up:[0,-1,0], textRot:Math.PI},
        {label:'BOTTOM', dir:[0,0,-1], color:0xE8734A,
         up:[0,1,0], textRot:0},
    ];

    // Create a rounded box geometry
    const boxSize = 1.4;
    const boxGeo = new THREE.BoxGeometry(boxSize, boxSize, boxSize, 4, 4, 4);

    // Round the corners by normalizing and blending
    const pos = boxGeo.attributes.position;
    const v = new THREE.Vector3();
    for (let i = 0; i < pos.count; i++) {
        v.set(pos.getX(i), pos.getY(i), pos.getZ(i));
        const half = boxSize / 2;
        // Blend toward sphere for rounding
        const sphere = v.clone().normalize().multiplyScalar(half * 1.05);
        const t = 0.15;  // rounding amount
        v.lerp(sphere, t);
        pos.setXYZ(i, v.x, v.y, v.z);
    }
    boxGeo.computeVertexNormals();

    // Main cube body — semi-transparent dark
    const cubeMat = new THREE.MeshStandardMaterial({
        color: 0x2a2a2a,
        metalness: 0.3,
        roughness: 0.7,
        transparent: true,
        opacity: 0.85,
    });
    const cubeMesh = new THREE.Mesh(boxGeo, cubeMat);
    cubeGroup.add(cubeMesh);

    // Face label sprites
    const faceHitTargets = [];  // for raycasting
    faces.forEach(f => {
        // Create canvas texture for label
        const canvas = document.createElement('canvas');
        canvas.width = 128;
        canvas.height = 128;
        const ctx = canvas.getContext('2d');

        // Rounded rect background
        const r = 12;
        ctx.beginPath();
        ctx.roundRect(4, 4, 120, 120, r);
        ctx.fillStyle = `#${f.color.toString(16).padStart(6, '0')}`;
        ctx.fill();
        ctx.strokeStyle = 'rgba(255,255,255,0.3)';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Label text
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 26px -apple-system, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        if (f.textRot) {
            ctx.save();
            ctx.translate(64, 64);
            ctx.rotate(f.textRot);
            ctx.translate(-64, -64);
        }
        ctx.fillText(f.label, 64, 64);
        if (f.textRot) ctx.restore();

        const texture = new THREE.CanvasTexture(canvas);
        texture.needsUpdate = true;

        // Place a small plane on each face
        const planeSize = boxSize * 0.85;
        const planeGeo = new THREE.PlaneGeometry(planeSize, planeSize);
        const planeMat = new THREE.MeshBasicMaterial({
            map: texture,
            transparent: true,
            depthWrite: false,
            side: THREE.FrontSide,
        });
        const plane = new THREE.Mesh(planeGeo, planeMat);

        // Position slightly outside cube face
        const offset = boxSize / 2 + 0.01;
        plane.position.set(
            f.dir[0] * offset,
            f.dir[1] * offset,
            f.dir[2] * offset
        );

        // Orient plane to face outward
        const normal = new THREE.Vector3(...f.dir);
        const target = plane.position.clone().add(normal);
        plane.lookAt(target);

        plane.userData = { viewDir: f.dir, viewUp: f.up, viewLabel: f.label };
        cubeGroup.add(plane);
        faceHitTargets.push(plane);
    });

    // Axis indicators (short colored lines from cube corners)
    const axisLen = 0.45;
    const axisOffset = boxSize / 2 + 0.05;
    const axes = [
        { dir: [1,0,0], color: 0xff4444, label: 'X', up: [0,0,1] },
        { dir: [0,1,0], color: 0x44ff44, label: 'Y', up: [0,0,1] },
        { dir: [0,0,1], color: 0x4488ff, label: 'Z', up: [0,-1,0] },
    ];
    const axisHitTargets = [];
    axes.forEach(a => {
        const d = a.dir, o = axisOffset, e = o + axisLen;
        const start = new THREE.Vector3(d[0]*o, d[1]*o, d[2]*o);
        const end = new THREE.Vector3(d[0]*e, d[1]*e, d[2]*e);
        // Cylinder for thick axis line (WebGL linewidth capped at 1px)
        const axDir = end.clone().sub(start);
        const len = axDir.length();
        const cylGeo = new THREE.CylinderGeometry(0.03, 0.03, len, 6);
        const cylMat = new THREE.MeshBasicMaterial({ color: a.color });
        const cyl = new THREE.Mesh(cylGeo, cylMat);
        const mid = start.clone().add(end).multiplyScalar(0.5);
        cyl.position.copy(mid);
        const upVec = new THREE.Vector3(0, 1, 0);
        const quat = new THREE.Quaternion();
        quat.setFromUnitVectors(upVec, axDir.normalize());
        cyl.quaternion.copy(quat);
        cyl.userData = { viewDir: a.dir, viewUp: a.up, axisLabel: a.label };
        cubeGroup.add(cyl);
        axisHitTargets.push(cyl);

        // Axis letter sprite — use a small sphere as click target behind it
        const hitGeo = new THREE.SphereGeometry(0.2, 8, 8);
        const hitMat = new THREE.MeshBasicMaterial({ visible: false });
        const hitSphere = new THREE.Mesh(hitGeo, hitMat);
        hitSphere.position.set(
            a.dir[0]*(axisOffset+axisLen+0.2),
            a.dir[1]*(axisOffset+axisLen+0.2),
            a.dir[2]*(axisOffset+axisLen+0.2)
        );
        hitSphere.userData = { viewDir: a.dir, viewUp: a.up, axisLabel: a.label };
        cubeGroup.add(hitSphere);
        axisHitTargets.push(hitSphere);

        // Axis letter sprite
        const spCanvas = document.createElement('canvas');
        spCanvas.width = 96; spCanvas.height = 96;
        const spCtx = spCanvas.getContext('2d');
        spCtx.strokeStyle = '#000';
        spCtx.lineWidth = 6;
        spCtx.font = 'bold 64px -apple-system, sans-serif';
        spCtx.textAlign = 'center';
        spCtx.textBaseline = 'middle';
        spCtx.strokeText(a.label, 48, 48);
        spCtx.fillStyle = `#${a.color.toString(16).padStart(6, '0')}`;
        spCtx.fillText(a.label, 48, 48);
        const spTex = new THREE.CanvasTexture(spCanvas);
        const spMat = new THREE.SpriteMaterial({ map: spTex });
        const sprite = new THREE.Sprite(spMat);
        sprite.position.copy(hitSphere.position);
        sprite.scale.set(0.45, 0.45, 1);
        cubeGroup.add(sprite);
    });

    // --- Raycasting for click-to-snap ---
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();

    renderer.domElement.addEventListener('click', (e) => {
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        raycaster.setFromCamera(mouse, camera);
        // Check both face labels and axis indicators
        const allTargets = faceHitTargets.concat(axisHitTargets);
        const hits = raycaster.intersectObjects(allTargets);
        if (hits.length === 0) return;

        const hit = hits[0].object;
        const dir = hit.userData.viewDir;
        const up = hit.userData.viewUp;
        const dist = CAMERA_DIST;
        const tx = TARGET[0], ty = TARGET[1], tz = TARGET[2];

        const mainCam = mainSC.camera;
        const ctrl = mainSC.controls;

        mainCam.position.set(
            tx + dir[0] * dist,
            ty + dir[1] * dist,
            tz + dir[2] * dist
        );
        ctrl.target.set(tx, ty, tz);
        mainCam.up.set(up[0], up[1], up[2]);

        // Adjust ortho frustum if needed
        if (mainCam.isOrthographicCamera) {
            const fovRad = (50 * Math.PI) / 180;
            const frustumH = 2 * dist * Math.tan(fovRad / 2);
            const el = mainSC.renderer.domElement;
            const aspect = el.width / el.height;
            const frustumW = frustumH * aspect;
            mainCam.left = -frustumW / 2;
            mainCam.right = frustumW / 2;
            mainCam.top = frustumH / 2;
            mainCam.bottom = -frustumH / 2;
            mainCam.updateProjectionMatrix();
        }

        ctrl.update();
    });

    // Highlight on hover
    let hoveredFace = null;
    renderer.domElement.addEventListener('mousemove', (e) => {
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        raycaster.setFromCamera(mouse, camera);
        const allTargets = faceHitTargets.concat(axisHitTargets);
        const hits = raycaster.intersectObjects(allTargets);

        if (hoveredFace) {
            hoveredFace.material.opacity = 1.0;
            hoveredFace = null;
        }
        renderer.domElement.style.cursor = hits.length > 0 ? 'pointer' : 'default';
        if (hits.length > 0) {
            const h = hits[0].object;
            // Only dim face label planes, not axis meshes
            if (faceHitTargets.includes(h)) {
                hoveredFace = h;
                hoveredFace.material.opacity = 0.7;
            }
        }
    });

    // --- Sync loop: match cube rotation to main camera ---
    const TARGET = ["""
    + f"{_TARGET[0]}, {_TARGET[1]}, {_TARGET[2]}"
    + """];
    const CAMERA_DIST = """
    + str(_DIST)
    + """;
    const targetVec = new THREE.Vector3(...TARGET);

    function animate() {
        requestAnimationFrame(animate);

        // Get main camera's view direction
        const mainCam = mainSC.camera;
        const camPos = mainCam.position.clone();
        const lookDir = new THREE.Vector3();
        mainCam.getWorldDirection(lookDir);

        // Build a rotation matrix from the main camera's orientation
        // The cube should rotate inversely to the camera
        const q = mainCam.quaternion.clone().invert();
        cubeGroup.quaternion.copy(q);

        renderer.render(scene, camera);
    }
    animate();

    // --- Ortho/Persp toggle ---
    const toggleBtn = document.getElementById('viewcube-proj-toggle');
    if (toggleBtn) {
        toggleBtn.addEventListener('click', () => {
            const mainCam = mainSC.camera;
            const ctrl = mainSC.controls;
            const canvas = mainSC.renderer.domElement;
            const aspect = canvas.width / canvas.height;

            if (mainCam.isPerspectiveCamera) {
                // Switch to ortho
                const d = mainCam.position.distanceTo(ctrl.target);
                const fovRad = (mainCam.fov * Math.PI) / 180;
                const frustumH = 2 * d * Math.tan(fovRad / 2);
                const frustumW = frustumH * aspect;

                const ortho = new THREE.OrthographicCamera(
                    -frustumW/2, frustumW/2, frustumH/2, -frustumH/2, 0.001, 100
                );
                ortho.position.copy(mainCam.position);
                ortho.quaternion.copy(mainCam.quaternion);
                ortho.up.copy(mainCam.up);
                ortho.zoom = 1;
                ortho.updateProjectionMatrix();

                mainSC.camera = ortho;
                ctrl.object = ortho;
                ctrl.update();
                toggleBtn.textContent = 'Ortho';
                toggleBtn.classList.add('viewcube-ortho-active');
            } else {
                // Switch to perspective
                const persp = new THREE.PerspectiveCamera(50, aspect, 0.01, 100);
                persp.position.copy(mainCam.position);
                persp.quaternion.copy(mainCam.quaternion);
                persp.up.copy(mainCam.up);
                persp.updateProjectionMatrix();

                mainSC.camera = persp;
                ctrl.object = persp;
                ctrl.update();
                toggleBtn.textContent = 'Persp';
                toggleBtn.classList.remove('viewcube-ortho-active');
            }
        });
    }

    // --- Fit to view button ---
    const fitBtn = document.getElementById('viewcube-fit-btn');
    if (fitBtn) {
        fitBtn.addEventListener('click', () => {
            const mainCam = mainSC.camera;
            const ctrl = mainSC.controls;
            const canvas = mainSC.renderer.domElement;
            const aspect = canvas.width / canvas.height;

            // Compute bounding box of robot meshes only
            // (STL meshes have high vertex counts vs primitives)
            const box = new THREE.Box3();
            mainSC.scene.traverse((obj) => {
                if (!obj.isMesh || !obj.visible) return;
                const geo = obj.geometry;
                if (!geo || !geo.attributes ||
                    !geo.attributes.position) return;
                const vc = geo.attributes.position.count;
                if (vc < 100) return;  // skip primitives
                box.expandByObject(obj);
            });
            if (box.isEmpty()) return;

            const center = new THREE.Vector3();
            box.getCenter(center);
            const size = new THREE.Vector3();
            box.getSize(size);
            const maxDim = Math.max(size.x, size.y, size.z);

            // Point camera at the robot center
            ctrl.target.copy(center);

            if (mainCam.isPerspectiveCamera) {
                const fovRad = (mainCam.fov * Math.PI) / 180;
                let dist = (maxDim / 2) / Math.tan(fovRad / 2);
                dist *= 1.2;  // margin
                const dir = new THREE.Vector3();
                mainCam.getWorldDirection(dir);
                mainCam.position.copy(
                    center.clone().sub(dir.multiplyScalar(dist))
                );
            } else {
                // Ortho: adjust frustum to fit robot
                const pad = 1.2;
                const frustumH = maxDim * pad;
                const frustumW = frustumH * aspect;
                mainCam.left = -frustumW / 2;
                mainCam.right = frustumW / 2;
                mainCam.top = frustumH / 2;
                mainCam.bottom = -frustumH / 2;
                const dir = new THREE.Vector3();
                mainCam.getWorldDirection(dir);
                mainCam.position.copy(
                    center.clone().sub(dir.multiplyScalar(CAMERA_DIST))
                );
                mainCam.updateProjectionMatrix();
            }
            ctrl.update();
        });
    }
})();
"""
)


def add_view_controls(parent_container=None) -> None:
    """Add a 3D ViewCube gizmo overlay to the viewport.

    *parent_container* is the wrapper ``div`` around the scene (must have
    ``position: relative``).  The overlay is positioned absolutely in its
    top-right corner.
    """
    # Overlay container positioned in top-right corner of viewport
    with ui.element("div").style(
        "position: absolute; "
        "top: 8px; "
        "right: 8px; "
        "width: 210px; "
        "z-index: 10; "
        "pointer-events: none; "
    ):
        with ui.element("div").style(
            "pointer-events: auto; "
            "display: flex; flex-direction: column; align-items: center; "
            "gap: 4px; "
        ):
            # The cube canvas
            ui.html(
                '<div id="viewcube-container" style="width:192px;height:192px;"></div>'
            )

            # Button row: Ortho/Persp toggle + Fit to view
            ui.html("""
                <div style="
                    display: flex; gap: 4px;
                    margin-top: -14px;
                    justify-content: center;
                ">
                <button id="viewcube-proj-toggle"
                    class="viewcube-btn"
                    style="
                        font-size: 11px;
                        font-weight: 600;
                        padding: 2px 12px;
                        border: 1px solid rgba(255,255,255,0.4);
                        border-radius: 10px;
                        background: rgba(40,40,40,0.7);
                        color: #ccc;
                        cursor: pointer;
                        backdrop-filter: blur(4px);
                        transition: all 0.2s;
                        letter-spacing: 0.5px;
                    "
                    onmouseover="this.style.background='rgba(74,144,217,0.8)';this.style.color='#fff'"
                    onmouseout="this.style.background=this.classList.contains('viewcube-ortho-active')?'rgba(74,144,217,0.6)':'rgba(40,40,40,0.7)';this.style.color=this.classList.contains('viewcube-ortho-active')?'#fff':'#ccc'"
                >Persp</button>
                <button id="viewcube-fit-btn"
                    class="viewcube-btn"
                    style="
                        font-size: 11px;
                        font-weight: 600;
                        padding: 2px 12px;
                        border: 1px solid rgba(255,255,255,0.4);
                        border-radius: 10px;
                        background: rgba(40,40,40,0.7);
                        color: #ccc;
                        cursor: pointer;
                        backdrop-filter: blur(4px);
                        transition: all 0.2s;
                        letter-spacing: 0.5px;
                    "
                    onmouseover="this.style.background='rgba(74,144,217,0.8)';this.style.color='#fff'"
                    onmouseout="this.style.background='rgba(40,40,40,0.7)';this.style.color='#ccc'"
                >Fit</button>
                </div>
                <style>
                    .viewcube-ortho-active {
                        background: rgba(74,144,217,0.6) !important;
                        color: #fff !important;
                    }
                </style>
            """)

    # Initialize the ViewCube after the scene loads
    ui.timer(2.0, lambda: ui.run_javascript(_VIEWCUBE_JS), once=True)
