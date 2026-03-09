"""NiceGUI application for robot arm simulation."""

from __future__ import annotations

import math
from pathlib import Path

from nicegui import app, ui

from robot_arm_sim.models.robot import URDFRobot

from .kinematics import (
    forward_kinematics,
    matrix_to_position_euler,
    rpy_to_matrix,
    translation_matrix,
)
from .urdf_loader import load_urdf


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

    ui.run(port=port, title=f"Robot Sim: {robot.name}", reload=False)


def _build_ui(robot: URDFRobot, robot_dir: Path) -> None:
    """Build the simulator UI."""
    # State
    joint_angles: dict[str, float] = {}
    mesh_objects: dict[str, object] = {}
    chain = robot.get_kinematic_chain()

    for joint in chain:
        joint_angles[joint.name] = 0.0

    ui.label(f"Robot: {robot.name}").classes("text-h4")

    # Map joint names to descriptive labels using child link / part names
    joint_labels: dict[str, str] = {}
    for joint in chain:
        child_link = robot.get_link(joint.child)
        if child_link and child_link.mesh_path:
            part = Path(child_link.mesh_path).stem  # e.g. "A1", "A3_4"
            joint_labels[joint.name] = f"{part} ({joint.name})"
        else:
            # Virtual link — use parent part name
            parent_link = robot.get_link(joint.parent)
            if parent_link and parent_link.mesh_path:
                part = Path(parent_link.mesh_path).stem
                joint_labels[joint.name] = f"{part} ({joint.name})"
            else:
                joint_labels[joint.name] = joint.name

    with ui.row().style("flex-wrap: nowrap; gap: 0"):
        # Left panel: 3D scene
        with ui.column():
            with ui.scene(
                width=900,
                height=700,
                grid=(2, 100),
                background_color="#a0a0a0",
            ) as scene:
                scene.spot_light(intensity=1.0).move(2, 2, 3)
                scene.spot_light(intensity=0.6).move(-2, -1, 2)

                # Add meshes for each link
                for link in robot.links:
                    if link.mesh_path:
                        stl_name = Path(link.mesh_path).name
                        stl_url = f"/stl/{stl_name}"
                        obj = scene.stl(stl_url).scale(
                            link.mesh_scale[0],
                            link.mesh_scale[1],
                            link.mesh_scale[2],
                        ).material(color="#b0b0b0")
                        mesh_objects[link.name] = obj

                # Initial positioning
                _update_scene(robot, joint_angles, mesh_objects)

            # Set initial camera to a good viewpoint
            scene.move_camera(
                x=0.5, y=-0.5, z=0.4,
                look_at_x=0, look_at_y=0, look_at_z=0.25,
                duration=0,
            )

            # Tweak built-in lighting for better surface contrast.
            # NiceGUI scene has AmbientLight(0.7π) + DirectionalLight(0.3π)
            # by default — ambient is too strong, washing out surface detail.
            scene_el_id = scene._props.get("id", scene.id)
            ui.timer(
                1.0,
                lambda: ui.run_javascript(f"""
                    // Access the three.js scene via NiceGUI's test hook
                    for (const key of Object.keys(window)) {{
                        if (key.startsWith('scene_')) {{
                            const s = window[key];
                            if (s && s.isScene) {{
                                s.traverse(obj => {{
                                    // Lower ambient light for more contrast
                                    if (obj.isAmbientLight) obj.intensity = 0.8;
                                    // Boost directional light
                                    if (obj.isDirectionalLight) {{
                                        obj.intensity = 3.0;
                                        obj.position.set(3, -2, 5);
                                    }}
                                    // Increase specular shininess on meshes
                                    if (obj.isMesh && obj.material && obj.material.isMeshPhongMaterial) {{
                                        obj.material.shininess = 60;
                                        obj.material.needsUpdate = true;
                                    }}
                                }});
                                break;
                            }}
                        }}
                    }}
                """),
                once=True,
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
                            _update_scene(robot, joint_angles, mesh_objects)

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
                _update_scene(robot, joint_angles, mesh_objects)

            ui.button("Reset Joints", on_click=reset_joints)


def _update_scene(
    robot: URDFRobot,
    joint_angles: dict[str, float],
    mesh_objects: dict[str, object],
) -> None:
    """Recompute FK and update mesh transforms."""
    transforms = forward_kinematics(robot, joint_angles)

    for link_name, tf in transforms.items():
        obj = mesh_objects.get(link_name)
        if obj is None:
            continue

        # Apply link visual origin offset (xyz + rpy)
        link = robot.get_link(link_name)
        if link:
            visual_tf = (
                translation_matrix(link.origin_xyz)
                @ rpy_to_matrix(link.origin_rpy)
            )
            tf = tf @ visual_tf

        pos, euler = matrix_to_position_euler(tf)
        obj.move(pos[0], pos[1], pos[2])
        obj.rotate(euler[0], euler[1], euler[2])
