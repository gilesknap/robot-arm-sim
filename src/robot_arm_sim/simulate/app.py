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


def create_app(robot_dir: Path, port: int = 8080) -> None:
    """Create and run the NiceGUI simulator app."""
    urdf_path = robot_dir / "robot.urdf"
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"No robot.urdf found in {robot_dir}. "
            "Run the assembly-reasoning skill first."
        )

    robot = load_urdf(urdf_path)

    # Serve STL files
    stl_dir = robot_dir / "stl_files"
    if stl_dir.exists():
        app.add_static_files("/stl", str(stl_dir))

    @ui.page("/")
    def index():
        _build_ui(robot, robot_dir)

    ui.run(
        port=port,
        title=f"Robot Sim: {robot.name}",
        reload=False,
        show=False,  # Don't auto-open a browser tab on startup
        reconnect_timeout=30,  # Keep tabs alive longer when inactive
    )


# Callout offsets: (x_offset, y_offset, z_offset) from the anchor point.
# Part labels go LEFT, joint labels go RIGHT to avoid overlap.
# z_offset staggers vertically within each side.
_PART_OFFSETS = [
    (-0.18, 0.0, 0.0),  # A0 (base)
    (-0.18, 0.0, 0.0),  # A1
    (-0.18, 0.0, 0.0),  # A2
    (-0.18, 0.0, 0.0),  # A3_4
    (-0.18, 0.0, 0.0),  # A5
    (-0.18, 0.0, 0.0),  # A6
    (-0.18, 0.0, 0.0),
    (-0.18, 0.0, 0.0),
]
_JOINT_OFFSETS = [
    (0.18, 0.0, -0.02),  # joint_1 — nudge down to separate from joint_2
    (0.18, 0.0, 0.02),  # joint_2 — nudge up
    (0.18, 0.0, 0.0),  # joint_3
    (0.18, 0.0, 0.0),  # joint_4
    (0.18, 0.0, 0.0),  # joint_5
    (0.18, 0.0, 0.0),  # joint_6
    (0.18, 0.0, 0.0),
    (0.18, 0.0, 0.0),
]

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


def _build_ui(robot: URDFRobot, robot_dir: Path) -> None:
    """Build the simulator UI."""
    # State
    joint_angles: dict[str, float] = {}
    mesh_objects: dict[str, object] = {}
    labels_visible = {"value": False}
    callout_items: list[dict] = []
    chain = robot.get_kinematic_chain()
    mesh_centers = _load_mesh_centers(robot, robot_dir)

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
                        off = _PART_OFFSETS[pidx % len(_PART_OFFSETS)]
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
                    off = _JOINT_OFFSETS[jidx % len(_JOINT_OFFSETS)]
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

            # Post-init: lighting, materials, env map, middle-click pan
            ui.timer(
                1.0,
                lambda: ui.run_javascript(_POST_INIT_JS),
                once=True,
            )

            # Toolbar below 3D viewport
            def toggle_labels():
                labels_visible["value"] = not labels_visible["value"]
                _update_scene(
                    robot,
                    joint_angles,
                    mesh_objects,
                    callout_items,
                    labels_visible,
                    mesh_centers,
                )
                label_btn.text = (
                    "Hide Labels" if labels_visible["value"] else "Show Labels"
                )

            def shutdown():
                import os
                import signal

                os.kill(os.getpid(), signal.SIGTERM)

            with (
                ui.row()
                .classes("q-pa-sm")
                .style("width: 900px; justify-content: flex-end; gap: 8px;")
            ):
                label_btn = ui.button("Show Labels", on_click=toggle_labels).props(
                    "flat dense"
                )
                ui.button("Stop Simulator", on_click=shutdown).props(
                    "color=red-7 flat dense"
                )

        # Right panel: controls (snug against scene)
        with ui.column().classes("p-4").style("width: 250px; overflow-y: auto"):
            ui.label("Joint Controls").classes("text-h6")
            sliders: dict[str, ui.slider] = {}

            for joint in chain:
                if joint.joint_type not in ("revolute", "continuous"):
                    continue

                lower_deg = math.degrees(joint.limit_lower)
                upper_deg = math.degrees(joint.limit_upper)
                display = joint_labels[joint.name]

                with ui.column().classes("w-full"):
                    label = ui.label(f"{display}: 0.0°")

                    def make_handler(jname, lbl, disp):
                        def on_change():
                            angle_deg = sliders[jname].value
                            joint_angles[jname] = math.radians(angle_deg)
                            lbl.text = f"{disp}: {angle_deg:.1f}°"
                            _update_scene(
                                robot,
                                joint_angles,
                                mesh_objects,
                                callout_items,
                                labels_visible,
                            )

                        return on_change

                    slider = ui.slider(
                        min=lower_deg,
                        max=upper_deg,
                        value=0,
                        step=0.5,
                        on_change=make_handler(joint.name, label, display),
                    )
                    sliders[joint.name] = slider

            # Reset button
            def reset_joints():
                for jname in joint_angles:
                    joint_angles[jname] = 0.0
                for s in sliders.values():
                    s.value = 0
                _update_scene(
                    robot,
                    joint_angles,
                    mesh_objects,
                    callout_items,
                    labels_visible,
                    mesh_centers,
                )

            ui.button("Reset Joints", on_click=reset_joints)


def _update_scene(
    robot: URDFRobot,
    joint_angles: dict[str, float],
    mesh_objects: dict[str, object],
    callout_items: list[dict] | None = None,
    labels_visible: dict | None = None,
    mesh_centers: dict[str, np.ndarray] | None = None,
) -> None:
    """Recompute FK and update mesh transforms."""
    transforms = forward_kinematics(robot, joint_angles)

    # Build lookup: joint_name -> world position, link_name -> mesh center world pos
    joint_positions: dict[str, tuple[float, ...]] = {}
    link_center_positions: dict[str, tuple[float, ...]] = {}

    for link_name, tf in transforms.items():
        link_pos, _ = matrix_to_position_euler(tf)

        for joint in robot.joints:
            if joint.child == link_name:
                joint_positions[joint.name] = tuple(link_pos)
                break

        obj = mesh_objects.get(link_name)
        if obj is None:
            continue

        link = robot.get_link(link_name)
        if link:
            visual_tf = translation_matrix(link.origin_xyz) @ rpy_to_matrix(
                link.origin_rpy
            )
            tf_visual = tf @ visual_tf
        else:
            tf_visual = tf

        pos, euler = matrix_to_position_euler(tf_visual)
        obj.move(pos[0], pos[1], pos[2])  # type: ignore[attr-defined]
        obj.rotate(euler[0], euler[1], euler[2])  # type: ignore[attr-defined]

        # Compute mesh bbox center in world coords for part labels
        if mesh_centers and link_name in mesh_centers:
            # bbox center in STL coords (mm), scale to meters
            center_stl = mesh_centers[link_name] * 0.001
            # Transform: world = tf @ visual_origin @ center_stl
            center_homo = np.array([center_stl[0], center_stl[1], center_stl[2], 1.0])
            center_world = tf_visual @ center_homo
            link_center_positions[link_name] = (
                center_world[0],
                center_world[1],
                center_world[2],
            )
        else:
            # Fallback: use joint origin
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
