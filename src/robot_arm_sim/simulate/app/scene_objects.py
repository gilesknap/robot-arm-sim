"""3D scene construction — meshes, callouts, camera, and JS initialization."""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING

from nicegui import ui

from .js_snippets import (
    AXES_INIT_JS,
    BORE_INIT_JS,
    FACE_MARKER_INIT_JS,
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

            # Create callout labels for links (blue, LEFT side)
            pidx = 0
            for link in state.robot.links:
                if link.mesh_path:
                    part_name = Path(link.mesh_path).stem
                    off = state.part_offsets[pidx % len(state.part_offsets)]
                    text_obj = scene.text(
                        part_name,
                        style=(
                            "font-size: 14px; font-weight: bold; "
                            "color: #1565C0; "
                            "background: rgba(255,255,255,0.85); "
                            "padding: 2px 6px; border-radius: 3px; "
                            "border: 1px solid #1565C0; "
                            "pointer-events: none;"
                        ),
                    )
                    text_obj.visible(False)
                    line_obj = scene.line(
                        [0, 0, 0],
                        [off[0], off[1], off[2]],
                    ).material(color="#1565C0")
                    line_obj.visible(False)
                    state.callout_items.append(
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
                off = state.joint_offsets[jidx % len(state.joint_offsets)]
                text_obj = scene.text(
                    joint.name,
                    style=(
                        "font-size: 11px; "
                        "color: #C62828; "
                        "background: rgba(255,255,255,0.8); "
                        "padding: 1px 4px; border-radius: 2px; "
                        "border: 1px solid #C62828; "
                        "pointer-events: none;"
                    ),
                )
                text_obj.visible(False)
                line_obj = scene.line(
                    [0, 0, 0],
                    [off[0], off[1], off[2]],
                ).material(color="#C62828")
                line_obj.visible(False)
                state.callout_items.append(
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

    # Initialize bore center spheres
    bore_data_list = []
    for link_name, cps in state.connection_points.items():
        for i, cp in enumerate(cps):
            bore_data_list.append(
                {
                    "id": f"{link_name}_{cp['end']}_{i}",
                    "end": cp["end"],
                    "radius_mm": cp["radius_mm"],
                }
            )
    bore_js = BORE_INIT_JS.replace("BORE_DATA", json.dumps(bore_data_list))
    ui.timer(2.5, lambda: ui.run_javascript(bore_js), once=True)

    # Initialize face marker spheres for Edit Bores mode
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

    # Resize scene to fill its wrapper when the window is resized
    ui.timer(1.5, lambda: ui.run_javascript(SCENE_RESIZE_JS), once=True)
