"""NiceGUI application for robot arm simulation."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from nicegui import app, ui

from robot_arm_sim.models.robot import URDFRobot

from .kinematics import forward_kinematics, matrix_to_position_euler
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
    mesh_objects: dict[str, ui.scene.object3d] = {}
    chain = robot.get_kinematic_chain()

    for joint in chain:
        joint_angles[joint.name] = 0.0

    ui.label(f"Robot: {robot.name}").classes("text-h4")

    with ui.row().classes("w-full h-screen"):
        # Left panel: controls
        with ui.column().classes("w-1/4 p-4"):
            ui.label("Joint Controls").classes("text-h6")
            sliders: dict[str, ui.slider] = {}

            for joint in chain:
                if joint.joint_type not in ("revolute", "continuous"):
                    continue

                lower_deg = math.degrees(joint.limit_lower)
                upper_deg = math.degrees(joint.limit_upper)

                with ui.column().classes("w-full"):
                    label = ui.label(f"{joint.name}: 0.0°")

                    def make_handler(jname, lbl):
                        def on_change(e):
                            angle_deg = e.value
                            joint_angles[jname] = math.radians(angle_deg)
                            lbl.text = f"{jname}: {angle_deg:.1f}°"
                            _update_scene(robot, joint_angles, mesh_objects)

                        return on_change

                    slider = ui.slider(
                        min=lower_deg,
                        max=upper_deg,
                        value=0,
                        step=0.5,
                    ).on("update:model-value", make_handler(joint.name, label))
                    sliders[joint.name] = slider

            # Reset button
            def reset_joints():
                for jname in joint_angles:
                    joint_angles[jname] = 0.0
                for s in sliders.values():
                    s.value = 0
                _update_scene(robot, joint_angles, mesh_objects)

            ui.button("Reset Joints", on_click=reset_joints)

        # Right panel: 3D scene
        with ui.column().classes("w-3/4"):
            with ui.scene(
                width=800,
                height=600,
                grid=True,
            ) as scene:
                # Add meshes for each link
                for link in robot.links:
                    if link.mesh_path:
                        stl_name = Path(link.mesh_path).name
                        stl_url = f"/stl/{stl_name}"
                        obj = scene.stl(stl_url).scale(
                            link.mesh_scale[0],
                            link.mesh_scale[1],
                            link.mesh_scale[2],
                        )
                        mesh_objects[link.name] = obj

                # Initial positioning
                _update_scene(robot, joint_angles, mesh_objects)


def _update_scene(
    robot: URDFRobot,
    joint_angles: dict[str, float],
    mesh_objects: dict[str, ui.scene.object3d],
) -> None:
    """Recompute FK and update mesh transforms."""
    transforms = forward_kinematics(robot, joint_angles)

    for link_name, tf in transforms.items():
        obj = mesh_objects.get(link_name)
        if obj is None:
            continue

        # Also apply link visual origin offset
        link = robot.get_link(link_name)
        if link:
            visual_tf = np.eye(4)
            visual_tf[0, 3] = link.origin_xyz[0]
            visual_tf[1, 3] = link.origin_xyz[1]
            visual_tf[2, 3] = link.origin_xyz[2]
            tf = tf @ visual_tf

        pos, euler = matrix_to_position_euler(tf)
        obj.move(pos[0], pos[1], pos[2])
        obj.rotate(euler[0], euler[1], euler[2])
