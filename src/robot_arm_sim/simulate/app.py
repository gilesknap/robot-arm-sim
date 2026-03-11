"""NiceGUI application for robot arm simulation."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml
from nicegui import app, ui

from robot_arm_sim.models.robot import URDFRobot

from .kinematics import (
    forward_kinematics,
    matrix_to_position_euler,
    rpy_to_matrix,
    translation_matrix,
)
from .urdf_loader import load_urdf
from .view_controls import add_view_controls


def _load_mesh_centers(robot: URDFRobot, robot_dir: Path) -> dict[str, np.ndarray]:
    """Load bounding box centers from analysis YAML files.

    Returns a dict mapping link_name -> bbox center in STL coordinates (mm).
    """
    centers: dict[str, np.ndarray] = {}
    analysis_dir = robot_dir / "analysis"
    if not analysis_dir.exists():
        return centers

    for link in robot.links:
        if not link.mesh_path:
            continue
        part_name = Path(link.mesh_path).stem
        yaml_path = analysis_dir / f"{part_name}.yaml"
        if not yaml_path.exists():
            continue
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        bbox = data.get("geometry", {}).get("bounding_box", {})
        bb_min = bbox.get("min")
        bb_max = bbox.get("max")
        if bb_min and bb_max:
            center = np.array(
                [
                    (bb_min[0] + bb_max[0]) / 2.0,
                    (bb_min[1] + bb_max[1]) / 2.0,
                    (bb_min[2] + bb_max[2]) / 2.0,
                ]
            )
            centers[link.name] = center
    return centers


def _load_connection_points(robot: URDFRobot, robot_dir: Path) -> dict[str, list[dict]]:
    """Load bore connection points from analysis YAML files.

    Returns dict mapping link_name -> list of connection point dicts,
    each with keys: end, position (mm ndarray), axis, radius_mm.
    """
    result: dict[str, list[dict]] = {}
    analysis_dir = robot_dir / "analysis"
    if not analysis_dir.exists():
        return result

    for link in robot.links:
        if not link.mesh_path:
            continue
        part_name = Path(link.mesh_path).stem
        yaml_path = analysis_dir / f"{part_name}.yaml"
        if not yaml_path.exists():
            continue
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        cps = data.get("connection_points", [])
        if cps:
            points = []
            for cp in cps:
                points.append(
                    {
                        "end": cp["end"],
                        "position": np.array(cp["position"]),
                        "axis": cp.get("axis", [0, 0, 1]),
                        "radius_mm": cp.get("radius_mm", 10.0),
                    }
                )
            result[link.name] = points
    return result


def create_app(robot_dir: Path, port: int = 8080) -> None:
    """Create and run the NiceGUI simulator app."""
    urdf_path = robot_dir / "robot.urdf"
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"No robot.urdf found in {robot_dir}. "
            "Run the assembly-reasoning skill first."
        )

    # Serve STL files
    stl_dir = robot_dir / "stl_files"
    if stl_dir.exists():
        app.add_static_files("/stl", str(stl_dir))

    @app.get("/healthz")
    async def healthz():
        return {"status": "ok"}

    @ui.page("/")
    def index():
        robot = load_urdf(urdf_path)
        _build_ui(robot, robot_dir)

    ui.run(
        port=port,
        title=f"Robot Sim: {robot_dir.name}",
        reload=False,
        show=False,  # Don't auto-open a browser tab on startup
        reconnect_timeout=30,  # Keep tabs alive longer when inactive
    )


def _make_offsets(
    count: int, x: float, origins: list[list[float]] | None = None
) -> list[tuple[float, float, float]]:
    """Generate callout offsets for labels.

    For co-located joints (origin distance < 5mm between consecutive joints),
    apply alternating z-nudges of ±0.02 to prevent overlap.
    """
    offsets: list[tuple[float, float, float]] = []
    for i in range(count):
        z_nudge = 0.0
        if origins and i > 0:
            dist = (
                sum((origins[i][k] - origins[i - 1][k]) ** 2 for k in range(3)) ** 0.5
            )
            if dist < 0.005:  # co-located (< 5mm)
                z_nudge = 0.02 if i % 2 else -0.02
        offsets.append((x, 0.0, z_nudge))
    return offsets


# Position to hide labels (far off screen)
_HIDDEN_POS = (0, 0, -100)

# JavaScript executed once after the 3D scene initialises.
# Upgrades materials to PBR metal, generates a studio HDRI env map,
# tweaks lighting, and sets middle-click to pan.
_POST_INIT_JS = """
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


_SCREENSHOT_JS = """
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


_AXES_INIT_JS = """
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


_BORE_INIT_JS = """
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
    const bores = BORE_DATA;
    for (const b of bores) {
        const r = Math.max(0.002, Math.min(0.015,
            b.radius_mm * 0.001 * 0.3));
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
    window.__boreSpheres = spheres;
    window.__boreLines = lines;
    window.__setBoresVisible = function(show) {
        Object.values(spheres).forEach(
            s => { s.visible = show; });
        Object.values(lines).forEach(
            l => { l.visible = show; });
    };
    window.__updateBorePoses = function(data) {
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


_TRANSPARENCY_INIT_JS = """
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


def _build_ui(robot: URDFRobot, robot_dir: Path) -> None:
    """Build the simulator UI."""
    # State
    joint_angles: dict[str, float] = {}
    mesh_objects: dict[str, object] = {}
    labels_visible = {"value": False}
    frames_visible = {"value": False}
    bores_visible = {"value": False}
    transparent_mode = {"value": False}
    callout_items: list[dict] = []
    chain = robot.get_kinematic_chain()
    mesh_centers = _load_mesh_centers(robot, robot_dir)
    connection_points = _load_connection_points(robot, robot_dir)

    # Build ordered chain link names (parent then child, deduped)
    chain_link_names: list[str] = []
    seen: set[str] = set()
    for joint in chain:
        for lname in (joint.parent, joint.child):
            if lname not in seen:
                chain_link_names.append(lname)
                seen.add(lname)
    visible_links: dict[str, bool] = dict.fromkeys(chain_link_names, True)

    # Generate callout offsets dynamically based on link/joint counts
    part_count = sum(1 for link in robot.links if link.mesh_path)
    joint_origins = [j.origin_xyz for j in chain]
    part_offsets = _make_offsets(part_count, -0.18)
    joint_offsets = _make_offsets(len(chain), 0.18, joint_origins)

    for joint in chain:
        joint_angles[joint.name] = 0.0

    ui.label(f"Robot: {robot.name}").classes("text-h4")

    # Map joint names to descriptive labels using child link / part names
    joint_labels: dict[str, str] = {}
    for joint in chain:
        child_link = robot.get_link(joint.child)
        if child_link and child_link.mesh_path:
            part = Path(child_link.mesh_path).stem
            joint_labels[joint.name] = f"{part} ({joint.name})"
        else:
            parent_link = robot.get_link(joint.parent)
            if parent_link and parent_link.mesh_path:
                part = Path(parent_link.mesh_path).stem
                joint_labels[joint.name] = f"{part} ({joint.name})"
            else:
                joint_labels[joint.name] = joint.name

    with ui.row().style("flex-wrap: nowrap; gap: 0"):
        # Left panel: 3D scene + toolbar
        with ui.column().style("gap: 0"):
            with ui.scene(
                width=900,
                height=700,
                grid=(2, 100),
                background_color="#e0e0e0",
            ) as scene:
                scene.spot_light(intensity=1.0).move(2, 2, 3)
                scene.spot_light(intensity=0.6).move(-2, -1, 2)

                # Add meshes for each link
                for link in robot.links:
                    if link.mesh_path:
                        stl_name = Path(link.mesh_path).name
                        stl_url = f"/stl/{stl_name}"
                        obj = (
                            scene.stl(stl_url)
                            .scale(
                                link.mesh_scale[0],
                                link.mesh_scale[1],
                                link.mesh_scale[2],
                            )
                            .material(color="#b0b0b0")
                        )
                        mesh_objects[link.name] = obj

                # Create callout labels for links (part names — blue, LEFT side)
                pidx = 0
                for link in robot.links:
                    if link.mesh_path:
                        part_name = Path(link.mesh_path).stem
                        off = part_offsets[pidx % len(part_offsets)]
                        text_obj = scene.text(
                            part_name,
                            style=(
                                "font-size: 14px; font-weight: bold; "
                                "color: #1565C0; background: rgba(255,255,255,0.85); "
                                "padding: 2px 6px; border-radius: 3px; "
                                "border: 1px solid #1565C0; "
                                "pointer-events: none;"
                            ),
                        ).move(*_HIDDEN_POS)
                        line_obj = (
                            scene.line(
                                [0, 0, 0],
                                [off[0], off[1], off[2]],
                            )
                            .material(color="#1565C0")
                            .move(*_HIDDEN_POS)
                        )
                        callout_items.append(
                            {
                                "text": text_obj,
                                "line": line_obj,
                                "name": link.name,
                                "is_joint": False,
                                "offset": off,
                            }
                        )
                        pidx += 1

                # Create callout labels for joints (red, RIGHT side)
                jidx = 0
                for joint in chain:
                    off = joint_offsets[jidx % len(joint_offsets)]
                    text_obj = scene.text(
                        joint.name,
                        style=(
                            "font-size: 11px; "
                            "color: #C62828; background: rgba(255,255,255,0.8); "
                            "padding: 1px 4px; border-radius: 2px; "
                            "border: 1px solid #C62828; "
                            "pointer-events: none;"
                        ),
                    ).move(*_HIDDEN_POS)
                    line_obj = (
                        scene.line(
                            [0, 0, 0],
                            [off[0], off[1], off[2]],
                        )
                        .material(color="#C62828")
                        .move(*_HIDDEN_POS)
                    )
                    callout_items.append(
                        {
                            "text": text_obj,
                            "line": line_obj,
                            "name": joint.name,
                            "is_joint": True,
                            "offset": off,
                        }
                    )
                    jidx += 1

                # Initial positioning (meshes only, labels hidden)
                _update_scene(
                    robot,
                    joint_angles,
                    mesh_objects,
                    callout_items,
                    labels_visible,
                    mesh_centers,
                    visible_links=visible_links,
                )

            # Set initial camera to a good viewpoint
            scene.move_camera(
                x=0.5,
                y=-0.5,
                z=0.4,
                look_at_x=0,
                look_at_y=0,
                look_at_z=0.25,
                duration=0,
            )

            # View controls overlay (top-right corner of viewport)
            add_view_controls(scene_width=900)

            # Post-init: lighting, materials, env map, middle-click pan
            ui.timer(
                1.0,
                lambda: ui.run_javascript(_POST_INIT_JS),
                once=True,
            )

            # Initialize coordinate frame axes helpers
            joint_names_js = "[" + ",".join(f"'{j.name}'" for j in chain) + "]"
            axes_js = _AXES_INIT_JS.replace("JOINT_NAMES", joint_names_js)
            ui.timer(
                2.5,
                lambda: ui.run_javascript(axes_js),
                once=True,
            )

            # Initialize bore center spheres
            import json as _json

            bore_data_list = []
            for link_name, cps in connection_points.items():
                for i, cp in enumerate(cps):
                    bore_data_list.append(
                        {
                            "id": f"{link_name}_{cp['end']}_{i}",
                            "end": cp["end"],
                            "radius_mm": cp["radius_mm"],
                        }
                    )
            bore_js = _BORE_INIT_JS.replace("BORE_DATA", _json.dumps(bore_data_list))
            ui.timer(
                2.5,
                lambda: ui.run_javascript(bore_js),
                once=True,
            )

            # Initialize mesh transparency function
            ui.timer(
                2.5,
                lambda: ui.run_javascript(_TRANSPARENCY_INIT_JS),
                once=True,
            )

            # ee_readout placeholder — set after right panel is built
            ee_readout_ref: list[ui.label | None] = [None]

            # Toolbar below 3D viewport
            def toggle_labels():
                labels_visible["value"] = not labels_visible["value"]
                _update_scene_now()
                label_btn.text = (
                    "Hide Labels" if labels_visible["value"] else "Show Labels"
                )

            def shutdown():
                import os
                import signal

                os.kill(os.getpid(), signal.SIGTERM)

            with ui.row().classes("q-pa-sm").style("width: 900px; gap: 8px;"):
                ui.button(
                    "About",
                    on_click=lambda: ui.navigate.to(
                        "https://gilesknap.github.io/"
                        "robot-arm-sim/main/"
                        "explanations/building-with-claude.html",
                        new_tab=True,
                    ),
                ).props("color=green-7 flat dense")

                ui.space()

                label_btn = ui.button("Show Labels", on_click=toggle_labels).props(
                    "flat dense"
                )

                def toggle_frames():
                    frames_visible["value"] = not frames_visible["value"]
                    show = frames_visible["value"]
                    frames_btn.text = "Hide Frames" if show else "Show Frames"
                    ui.run_javascript(f"window.__setAxesVisible({str(show).lower()})")
                    if show:
                        _update_scene_now()

                frames_btn = ui.button("Show Frames", on_click=toggle_frames).props(
                    "flat dense"
                )

                def toggle_bores():
                    bores_visible["value"] = not bores_visible["value"]
                    show = bores_visible["value"]
                    bores_btn.text = "Hide Bores" if show else "Show Bores"
                    ui.run_javascript(f"window.__setBoresVisible({str(show).lower()})")
                    if show:
                        _update_scene_now()

                bores_btn = ui.button("Show Bores", on_click=toggle_bores).props(
                    "flat dense"
                )

                def toggle_transparent():
                    transparent_mode["value"] = not transparent_mode["value"]
                    on = transparent_mode["value"]
                    trans_btn.text = "Opaque" if on else "Transparent"
                    op = "0.25" if on else "1.0"
                    ui.run_javascript(f"window.__setMeshTransparency({op})")

                trans_btn = ui.button("Transparent", on_click=toggle_transparent).props(
                    "flat dense"
                )

                def _take_screenshot():
                    ui.run_javascript(_SCREENSHOT_JS)

                ui.button("Screenshot", on_click=_take_screenshot).props("flat dense")

                async def _reload_urdf():
                    # Save current state to sessionStorage before reload
                    vl_json = _json.dumps(visible_links)
                    save_js = (
                        "sessionStorage.setItem("
                        "'reload_state', JSON.stringify({"
                        f"visible_links: {vl_json},"
                        "joints: {"
                        + ",".join(f"'{k}': {v}" for k, v in joint_angles.items())
                        + "},"
                        "camera: (function() {"
                        "  function f(v){"
                        "    if(!v) return null;"
                        "    if(v.component&&v.component.proxy){"
                        "      var p=v.component.proxy;"
                        "      if(p.renderer&&p.scene&&p.controls)"
                        "        return p;}"
                        "    if(v.children&&Array.isArray(v.children))"
                        "      for(var c of v.children){"
                        "        var r=f(c); if(r) return r;}"
                        "    if(v.component&&v.component.subTree)"
                        "      return f(v.component.subTree);"
                        "    return null;}"
                        "  var sc=f(document.getElementById("
                        "    'app').__vue_app__"
                        "    ._container._vnode);"
                        "  if(!sc) return null;"
                        "  var c=sc.camera, t=sc.controls;"
                        "  return {"
                        "    px:c.position.x,py:c.position.y,"
                        "    pz:c.position.z,"
                        "    tx:t.target.x,ty:t.target.y,"
                        "    tz:t.target.z,"
                        "    ux:c.up.x,uy:c.up.y,uz:c.up.z,"
                        "    isOrtho:!!c.isOrthographicCamera,"
                        "    left:c.left,right:c.right,"
                        "    top:c.top,bottom:c.bottom"
                        "  };"
                        "})()}))"
                    )
                    await ui.run_javascript(save_js)
                    ui.navigate.to("/")

                ui.button(
                    "Reload URDF",
                    on_click=_reload_urdf,
                ).props("color=blue-7 flat dense")

                ui.button("Stop Simulator", on_click=shutdown).props(
                    "color=red-7 flat dense"
                )

            # --- Per-part visibility checkboxes (collapsible, below toolbar) ---
            # _update_scene_now defined early so checkbox handlers can use it
            ee_readout_ref_local = ee_readout_ref

            def _update_scene_now():
                _update_scene(
                    robot,
                    joint_angles,
                    mesh_objects,
                    callout_items,
                    labels_visible,
                    mesh_centers,
                    ee_readout_ref_local[0],
                    frames_visible,
                    visible_links=visible_links,
                    connection_points=connection_points,
                    bores_visible=bores_visible,
                )

            link_checkboxes: dict[str, ui.checkbox] = {}
            with (
                ui.expansion("Visible Parts", value=False)
                .props("dense")
                .style("width: 900px")
            ):
                with ui.row().classes("q-pa-xs").style("gap: 12px"):
                    for lname in chain_link_names:
                        lnk = robot.get_link(lname)
                        if lnk and lnk.mesh_path:
                            display = Path(lnk.mesh_path).stem
                        else:
                            display = lname

                        def make_vis_handler(ln):
                            def on_change(e):
                                visible_links[ln] = e.value
                                _update_scene_now()

                            return on_change

                        cb = ui.checkbox(
                            display,
                            value=True,
                            on_change=make_vis_handler(lname),
                        ).props("dense")
                        link_checkboxes[lname] = cb

        # Right panel: controls (snug against scene)
        with ui.column().classes("p-4").style("width: 280px; overflow-y: auto"):
            # Cache ikpy chain once for IK solving
            from .ik_solver import build_ik_chain, solve_ik

            ik_chain = build_ik_chain(robot_dir / "robot.urdf")

            def _populate_ik_from_fk():
                """Set IK slider values from current FK pose."""
                ee_chain = robot.get_kinematic_chain()
                if not ee_chain:
                    return
                tfs = forward_kinematics(robot, joint_angles)
                last = ee_chain[-1].child
                ee_tf = tfs.get(last, np.eye(4))
                p, eu = matrix_to_position_euler(ee_tf)
                ik_sliders["x"].value = round(p[0] * 1000)
                ik_sliders["y"].value = round(p[1] * 1000)
                ik_sliders["z"].value = round(p[2] * 1000)
                ik_sliders["rx"].value = round(math.degrees(eu[0]))
                ik_sliders["ry"].value = round(math.degrees(eu[1]))
                ik_sliders["rz"].value = round(math.degrees(eu[2]))

            def reset_all():
                for jname in joint_angles:
                    joint_angles[jname] = 0.0
                for s in sliders.values():
                    s.value = 0
                _populate_ik_from_fk()
                _update_scene_now()

            # Compact header row: title + mode radio + reset button
            with ui.row().classes("w-full items-center"):
                ui.label("Controls").classes("text-h6")
                ui.space()
                mode_radio = ui.radio(["Joint", "IK"], value="Joint").props(
                    "dense inline"
                )
                ui.button("Reset", on_click=reset_all).props("flat dense")

            # --- Joint control panel ---
            joint_panel = ui.column().classes("w-full")
            with joint_panel:
                sliders: dict[str, ui.slider] = {}

                for joint in chain:
                    if joint.joint_type not in ("revolute", "continuous"):
                        continue

                    lower_deg = math.degrees(joint.limit_lower)
                    upper_deg = math.degrees(joint.limit_upper)
                    display = joint_labels[joint.name]

                    with ui.column().classes("w-full"):
                        label = ui.label(f"{display}: 0.0\u00b0")

                        def make_handler(jname, lbl, disp):
                            def on_change():
                                angle_deg = sliders[jname].value
                                joint_angles[jname] = math.radians(angle_deg)
                                lbl.text = f"{disp}: {angle_deg:.1f}\u00b0"
                                _update_scene_now()

                            return on_change

                        slider = ui.slider(
                            min=lower_deg,
                            max=upper_deg,
                            value=0,
                            step=0.5,
                            on_change=make_handler(joint.name, label, display),
                        )
                        sliders[joint.name] = slider

            # --- IK control panel ---
            ik_panel = ui.column().classes("w-full")
            ik_panel.set_visibility(False)
            ik_sliders: dict[str, ui.slider] = {}
            ik_labels: dict[str, ui.label] = {}

            def _solve_ik_from_sliders():
                target = np.array(
                    [
                        (ik_sliders["x"].value or 0) / 1000,
                        (ik_sliders["y"].value or 0) / 1000,
                        (ik_sliders["z"].value or 0) / 1000,
                    ]
                )
                current = [0.0] + [joint_angles.get(j.name, 0.0) for j in chain]
                result = solve_ik(ik_chain, target, current)
                if result is None:
                    return
                for i, jt in enumerate(chain):
                    angle = result[i + 1]
                    joint_angles[jt.name] = angle
                    if jt.name in sliders:
                        sliders[jt.name].value = math.degrees(angle)
                _update_scene_now()

            ik_defs = [
                ("x", "X (mm)", -300, 300),
                ("y", "Y (mm)", -300, 300),
                ("z", "Z (mm)", 0, 500),
                ("rx", "Rx (\u00b0)", -180, 180),
                ("ry", "Ry (\u00b0)", -180, 180),
                ("rz", "Rz (\u00b0)", -180, 180),
            ]

            with ik_panel:
                for key, name, lo, hi in ik_defs:

                    def make_ik_handler(k, nm):
                        def on_change():
                            v = ik_sliders[k].value
                            ik_labels[k].text = f"{nm}: {v}"
                            _solve_ik_from_sliders()

                        return on_change

                    with ui.column().classes("w-full"):
                        ik_labels[key] = ui.label(f"{name}: 0")
                        ik_sliders[key] = ui.slider(
                            min=lo,
                            max=hi,
                            value=0,
                            step=1,
                            on_change=make_ik_handler(key, name),
                        )

            # Mode toggle handler
            def on_mode_change(e):
                is_joint = e.value == "Joint"
                joint_panel.set_visibility(is_joint)
                ik_panel.set_visibility(not is_joint)
                if not is_joint:
                    _populate_ik_from_fk()

            mode_radio.on_value_change(on_mode_change)

            # --- End-effector readout ---
            ui.separator()
            ui.label("End Effector").classes("text-caption")
            ee_readout = ui.label(
                "X: 0.0  Y: 0.0  Z: 0.0 mm\nRx: 0.0\u00b0  Ry: 0.0\u00b0  Rz: 0.0\u00b0"
            ).style("white-space: pre; font-family: monospace; font-size: 0.75rem;")

            # Wire up the ee_readout reference
            ee_readout_ref[0] = ee_readout

            # Initial EE readout update
            _update_scene(
                robot,
                joint_angles,
                mesh_objects,
                callout_items,
                labels_visible,
                mesh_centers,
                ee_readout,
                visible_links=visible_links,
            )

            # --- Restore state from sessionStorage (after URDF reload) ---
            async def _restore_state():
                raw = await ui.run_javascript(
                    "var s = sessionStorage.getItem('reload_state');"
                    "sessionStorage.removeItem('reload_state');"
                    "s"
                )
                if not raw:
                    return
                import json

                try:
                    state = json.loads(raw)
                except (json.JSONDecodeError, TypeError):
                    return

                # Restore per-part visibility
                vl_state = state.get("visible_links", {})
                for lname, vis in vl_state.items():
                    if lname in visible_links:
                        visible_links[lname] = vis
                        if lname in link_checkboxes:
                            link_checkboxes[lname].value = vis

                # Restore joint angles
                joints_state = state.get("joints", {})
                for jname, angle_rad in joints_state.items():
                    if jname in joint_angles and jname in sliders:
                        joint_angles[jname] = angle_rad
                        sliders[jname].value = math.degrees(angle_rad)
                _update_scene_now()

                # Restore camera (after scene is ready)
                cam = state.get("camera")
                if cam:
                    restore_cam_js = (
                        "function f(v){"
                        "if(!v)return null;"
                        "if(v.component&&v.component.proxy){"
                        "var p=v.component.proxy;"
                        "if(p.renderer&&p.scene&&p.controls)"
                        "return p;}"
                        "if(v.children&&Array.isArray(v.children))"
                        "for(var c of v.children){"
                        "var r=f(c);if(r)return r;}"
                        "if(v.component&&v.component.subTree)"
                        "return f(v.component.subTree);"
                        "return null;}"
                        "var sc=f(document.getElementById("
                        "'app').__vue_app__._container._vnode);"
                        "if(sc){"
                        "var THREE=window.__THREE_REF;"
                    )
                    if cam.get("isOrtho") and cam.get("left") is not None:
                        restore_cam_js += (
                            "var canvas=sc.renderer.domElement;"
                            "var aspect=canvas.width/canvas.height;"
                            "var oc=new THREE.OrthographicCamera("
                            f"{cam['left']},{cam['right']},"
                            f"{cam['top']},{cam['bottom']},"
                            "0.001,100);"
                            f"oc.position.set("
                            f"{cam['px']},{cam['py']},{cam['pz']});"
                            f"oc.up.set("
                            f"{cam['ux']},{cam['uy']},{cam['uz']});"
                            "oc.updateProjectionMatrix();"
                            "sc.camera=oc;"
                            "sc.controls.object=oc;"
                        )
                    else:
                        restore_cam_js += (
                            f"sc.camera.position.set("
                            f"{cam['px']},{cam['py']},{cam['pz']});"
                            f"sc.camera.up.set("
                            f"{cam['ux']},{cam['uy']},{cam['uz']});"
                        )
                    restore_cam_js += (
                        f"sc.controls.target.set("
                        f"{cam['tx']},{cam['ty']},{cam['tz']});"
                        "sc.controls.update();"
                        "}"
                    )
                    await ui.run_javascript(restore_cam_js)

            ui.timer(3.0, _restore_state, once=True)


def _matrix_to_quaternion(m: np.ndarray) -> list[float]:
    """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def _axis_to_quaternion(axis: list | np.ndarray) -> list[float]:
    """Return quaternion [qx,qy,qz,qw] rotating local-Y onto *axis*."""
    a = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    a = a / norm
    # local Y = [0, 1, 0]
    dot = float(a[1])  # dot([0,1,0], a)
    if dot > 0.99999:
        return [0.0, 0.0, 0.0, 1.0]
    if dot < -0.99999:
        # 180° around Z
        return [0.0, 0.0, 1.0, 0.0]
    # cross([0,1,0], a) = [a[2], 0, -a[0]]  (only Y=0 component)
    cx, cy, cz = float(a[2]), 0.0, float(-a[0])
    s = np.sqrt((1.0 + dot) * 2.0)
    inv_s = 1.0 / s
    return [cx * inv_s, cy * inv_s, cz * inv_s, s * 0.5]


def _update_scene(
    robot: URDFRobot,
    joint_angles: dict[str, float],
    mesh_objects: dict[str, object],
    callout_items: list[dict] | None = None,
    labels_visible: dict | None = None,
    mesh_centers: dict[str, np.ndarray] | None = None,
    ee_label: ui.label | None = None,
    frames_visible: dict | None = None,
    visible_links: dict[str, bool] | None = None,
    connection_points: dict[str, list[dict]] | None = None,
    bores_visible: dict | None = None,
) -> None:
    """Recompute FK and update mesh transforms."""
    # Compute which links are visible from per-part checkboxes
    vis_set: set[str] | None = None
    if visible_links is not None:
        vis_set = {n for n, v in visible_links.items() if v}

    transforms = forward_kinematics(robot, joint_angles)

    # Build lookup: joint_name -> world position, link_name -> mesh center world pos
    joint_positions: dict[str, tuple[float, ...]] = {}
    link_center_positions: dict[str, tuple[float, ...]] = {}
    # Store visual transforms for bore marker computation
    visual_transforms: dict[str, np.ndarray] = {}

    for link_name, tf in transforms.items():
        link_pos, _ = matrix_to_position_euler(tf)

        for joint in robot.joints:
            if joint.child == link_name:
                joint_positions[joint.name] = tuple(link_pos)
                break

        obj = mesh_objects.get(link_name)
        if obj is None:
            continue

        if vis_set is not None and link_name not in vis_set:
            obj.move(*_HIDDEN_POS)  # type: ignore[attr-defined]
            continue

        link = robot.get_link(link_name)
        if link:
            visual_tf = translation_matrix(link.origin_xyz) @ rpy_to_matrix(
                link.origin_rpy
            )
            tf_visual = tf @ visual_tf
        else:
            tf_visual = tf

        visual_transforms[link_name] = tf_visual
        pos, euler = matrix_to_position_euler(tf_visual)
        obj.move(pos[0], pos[1], pos[2])  # type: ignore[attr-defined]
        obj.rotate(euler[0], euler[1], euler[2])  # type: ignore[attr-defined]

        # Compute mesh bbox center in world coords for part labels
        if mesh_centers and link_name in mesh_centers:
            center_stl = mesh_centers[link_name] * 0.001
            center_homo = np.array([center_stl[0], center_stl[1], center_stl[2], 1.0])
            center_world = tf_visual @ center_homo
            link_center_positions[link_name] = (
                center_world[0],
                center_world[1],
                center_world[2],
            )
        else:
            link_center_positions[link_name] = tuple(link_pos)

    # Update callout positions
    if callout_items:
        show = labels_visible and labels_visible.get("value", False)
        for item in callout_items:
            if not show:
                item["text"].move(*_HIDDEN_POS)
                item["line"].move(*_HIDDEN_POS)
                continue

            name = item["name"]
            ox, oy, oz = item["offset"]

            if item["is_joint"]:
                anchor = joint_positions.get(name)
            else:
                anchor = link_center_positions.get(name)

            if anchor is None:
                continue

            ax, ay, az = anchor
            item["text"].move(ax + ox, ay + oy, az + oz)
            item["line"].move(ax, ay, az)

    # End-effector readout
    if ee_label is not None:
        chain = robot.get_kinematic_chain()
        if chain:
            last_link = chain[-1].child
            ee_tf = transforms.get(last_link, np.eye(4))
            ee_pos, ee_euler = matrix_to_position_euler(ee_tf)
            x_mm = ee_pos[0] * 1000
            y_mm = ee_pos[1] * 1000
            z_mm = ee_pos[2] * 1000
            rx_deg = math.degrees(ee_euler[0])
            ry_deg = math.degrees(ee_euler[1])
            rz_deg = math.degrees(ee_euler[2])
            ee_label.text = (
                f"X: {x_mm:.1f}  Y: {y_mm:.1f}"
                f"  Z: {z_mm:.1f} mm\n"
                f"Rx: {rx_deg:.1f}\u00b0  Ry: {ry_deg:.1f}\u00b0"
                f"  Rz: {rz_deg:.1f}\u00b0"
            )

    # Coordinate frames update
    if frames_visible and frames_visible.get("value", False):
        frame_data = {}
        for jt in robot.get_kinematic_chain():
            tf = transforms.get(jt.child)
            if tf is None:
                continue
            pos_j = [
                float(tf[0, 3]),
                float(tf[1, 3]),
                float(tf[2, 3]),
            ]
            q = _matrix_to_quaternion(tf[:3, :3])
            frame_data[jt.name] = {"p": pos_j, "q": q}
        import json

        js_data = json.dumps(frame_data)
        ui.run_javascript(f"window.__updateAxesPoses({js_data})")

    # Bore center markers update
    show_bores = bores_visible and bores_visible.get("value", False)
    if connection_points and show_bores:
        import json as _json2

        bore_poses: dict[str, list[float]] = {}
        for link_name, cps in connection_points.items():
            tf_vis = visual_transforms.get(link_name)
            is_vis = vis_set is None or link_name in vis_set
            for i, cp in enumerate(cps):
                bid = f"{link_name}_{cp['end']}_{i}"
                if tf_vis is not None and is_vis:
                    cp_m = cp["position"] * 0.001
                    cp_homo = np.array([cp_m[0], cp_m[1], cp_m[2], 1.0])
                    wp = tf_vis @ cp_homo
                    world_axis = tf_vis[:3, :3] @ np.asarray(cp["axis"], dtype=float)
                    q = _axis_to_quaternion(world_axis)
                    bore_poses[bid] = [
                        float(wp[0]),
                        float(wp[1]),
                        float(wp[2]),
                        1.0,
                        q[0],
                        q[1],
                        q[2],
                        q[3],
                    ]
                else:
                    bore_poses[bid] = [
                        0.0,
                        0.0,
                        -100.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    ]
        ui.run_javascript(f"window.__updateBorePoses({_json2.dumps(bore_poses)})")
