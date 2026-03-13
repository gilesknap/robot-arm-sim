"""3D scene construction — meshes, callouts, camera, and JS initialization."""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING

from nicegui import ui

from .js_snippets import (
    AXES_INIT_JS,
    CONNECTION_INIT_JS,
    FACE_MARKER_INIT_JS,
    LABEL_CALLOUT_JS,
    POST_INIT_JS,
    SCENE_RESIZE_JS,
    TRANSPARENCY_INIT_JS,
)
from .scene_update import update_scene
from .view_controls import add_view_controls

if TYPE_CHECKING:
    from .state import SimulatorState


def build_scene(state: SimulatorState) -> None:
    """Build the 3D scene with meshes, callouts, camera, and JS init."""
    chain = state.chain

    # Wrapper: sized by CSS class .sim-scene-wrapper (see main.py)
    with ui.element("div").classes("sim-scene-wrapper"):
        with ui.scene(
            width=900,
            height=700,
            grid=(2, 100),
            background_color="#e0e0e0",
        ) as scene:
            state.scene = scene
            scene.spot_light(intensity=1.0).move(2, 2, 3)
            scene.spot_light(intensity=0.6).move(-2, -1, 2)

            # Add meshes for each link
            for link in state.robot.links:
                if link.mesh_path:
                    stl_name = Path(link.mesh_path).name
                    stl_url = f"/stl/{state.robot_dir.name}/{stl_name}"
                    obj = (
                        scene.stl(stl_url)
                        .scale(
                            link.mesh_scale[0],
                            link.mesh_scale[1],
                            link.mesh_scale[2],
                        )
                        .material(color="#b0b0b0")
                    )
                    state.mesh_objects[link.name] = obj

            # Initial positioning (meshes only, labels hidden)
            update_scene(state)

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
        add_view_controls()

    # Post-init: lighting, materials, env map, middle-click pan
    ui.timer(1.0, lambda: ui.run_javascript(POST_INIT_JS), once=True)

    # Initialize coordinate frame axes helpers
    joint_names_js = "[" + ",".join(f"'{j.name}'" for j in chain) + "]"
    axes_js = AXES_INIT_JS.replace("JOINT_NAMES", joint_names_js)
    ui.timer(2.5, lambda: ui.run_javascript(axes_js), once=True)

    # Initialize connection point spheres
    connection_data_list = []
    for link_name, cps in state.connection_points.items():
        for i, cp in enumerate(cps):
            connection_data_list.append(
                {
                    "id": f"{link_name}_{cp['end']}_{i}",
                    "end": cp["end"],
                    "radius_mm": cp["radius_mm"],
                    "centering": cp.get("centering", "surface"),
                }
            )
    connection_js = CONNECTION_INIT_JS.replace(
        "CONNECTION_DATA", json.dumps(connection_data_list)
    )
    ui.timer(2.5, lambda: ui.run_javascript(connection_js), once=True)

    # Initialize face marker spheres for Edit Connections mode
    face_data_list = []
    for link_name, faces in state.flat_faces.items():
        for i, _ff in enumerate(faces):
            face_data_list.append({"id": f"face_{link_name}_{i}"})
    mesh_link_map = {obj.id: link_name for link_name, obj in state.mesh_objects.items()}
    face_js = FACE_MARKER_INIT_JS.replace("FACE_DATA", json.dumps(face_data_list))
    face_js = face_js.replace("MESH_LINK_MAP", json.dumps(mesh_link_map))
    ui.timer(2.5, lambda: ui.run_javascript(face_js), once=True)

    # Initialize mesh transparency function
    ui.timer(2.5, lambda: ui.run_javascript(TRANSPARENCY_INIT_JS), once=True)

    # Initialize label callout overlay (parts on left, joints on bottom)
    label_js = LABEL_CALLOUT_JS.replace("LABEL_DATA", json.dumps(state.label_metadata))
    ui.timer(2.5, lambda: ui.run_javascript(label_js), once=True)

    # Resize scene to fill its wrapper when the window is resized
    ui.timer(0.1, lambda: ui.run_javascript(SCENE_RESIZE_JS), once=True)
