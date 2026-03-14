"""Edit Connections mode — interactive connection point assignment."""

from __future__ import annotations

import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import yaml
from nicegui import ui

from robot_arm_sim.models import (
    ConnectionPoint,
    load_part_yaml,
    save_part_yaml,
)

from .loaders import quantize_axis

if TYPE_CHECKING:
    from .state import SimulatorState


def _force_reset_mesh_positions(state: SimulatorState) -> None:
    """Invalidate NiceGUI's cached mesh positions so the next move() always pushes."""
    for obj in state.mesh_objects.values():
        obj.x = float("nan")
        obj.y = float("nan")
        obj.z = float("nan")


def _get_part_name(state: SimulatorState, link_name: str) -> str:
    """Get the mesh stem name for a link."""
    link = state.robot.get_link(link_name)
    if link and link.mesh_path:
        return Path(link.mesh_path).stem
    return link_name


def _update_selected_part(state: SimulatorState, link_name: str) -> None:
    """Update the selected part name display."""
    state.selected_part_name = link_name
    part_name = _get_part_name(state, link_name)
    if state.selected_part_label is not None:
        state.selected_part_label.text = part_name
        state.selected_part_label.update()


def build_edit_connections(state: SimulatorState) -> None:
    """Build the Edit Connections mode UI and handlers."""
    state.edit_connections_row = (
        ui.column().classes("q-pa-sm").style("width: 100%; gap: 4px; display: none;")
    )
    with state.edit_connections_row:
        # === Row 1: Mode buttons | Show All | Selected part name ===
        with ui.row().classes("items-center").style("width: 100%; gap: 8px;"):
            _mode_defs = [
                ("proximal_centred", "Proximal Centred", "blue-7"),
                ("proximal_surface", "Proximal Surface", "green-7"),
                ("distal", "Distal", "red-7"),
                ("move_parts", "Move Parts", "amber-9"),
            ]
            mode_buttons: dict[str, Any] = {}

            def _apply_mode_styles() -> None:
                cur = state.connection_edit_mode["value"]
                for key, _, color in _mode_defs:
                    btn = mode_buttons[key]
                    if key == cur:
                        btn.props(f"dense no-caps color={color}")
                        btn.classes(add="bg-blue-1")
                    else:
                        btn.props(f"flat dense no-caps color={color}")
                        btn.classes(remove="bg-blue-1")
                    btn.update()

            def _make_mode_handler(mode_key: str):
                def handler() -> None:
                    state.connection_edit_mode["value"] = mode_key
                    if mode_key.startswith("proximal"):
                        state.connection_end_toggle["value"] = "Proximal"
                        state.connection_centering["value"] = (
                            "center" if mode_key == "proximal_centred" else "surface"
                        )
                    elif mode_key == "distal":
                        state.connection_end_toggle["value"] = "Distal"
                        state.connection_centering["value"] = "surface"
                    ui.run_javascript(
                        f"if(window.__enablePartDrag)"
                        f" window.__enablePartDrag("
                        f"{str(mode_key == 'move_parts').lower()})"
                    )
                    _apply_mode_styles()

                return handler

            for i, (key, lbl, color) in enumerate(_mode_defs):
                if i > 0:
                    ui.separator().props("vertical")
                btn = ui.button(lbl, on_click=_make_mode_handler(key)).props(
                    f"dense no-caps color={color}"
                )
                mode_buttons[key] = btn

            _apply_mode_styles()

            # Dummy centering select for compatibility
            state.connection_centering_select = type(
                "_Dummy", (), {"value": "surface"}
            )()

            ui.separator().props("vertical")

            def _on_show_all(e: Any) -> None:
                state.show_all_connections["value"] = e.value
                _sync_connection_visibility(state)

            ui.checkbox("Show All", value=False).props("dense").on_value_change(
                _on_show_all
            )

            ui.separator().props("vertical")

            state.selected_part_label = ui.label("(no part)").style(
                "font-size: 0.85rem; color: #333; font-weight: bold;"
            )

        # === Row 2: Undo/Redo | Rot | Remove Part | status | Save ===
        with ui.row().classes("items-center").style("width: 100%; gap: 8px;"):

            def _undo() -> None:
                if not state.undo_stack:
                    return
                action = state.undo_stack.pop()
                state.redo_stack.append(action)
                _reverse_action(state, action)

            def _redo() -> None:
                if not state.redo_stack:
                    return
                action = state.redo_stack.pop()
                state.undo_stack.append(action)
                _apply_action(state, action)

            ui.button("Undo", on_click=_undo).props("flat dense no-caps")
            ui.button("Redo", on_click=_redo).props("flat dense no-caps")

            ui.separator().props("vertical")

            def _make_rotate_handler(axis_idx: int):
                axis_names = ["X", "Y", "Z"]

                def handler() -> None:
                    if state.selected_part_name is None:
                        state.connection_status_label.text = "Select a part first"
                        return
                    link_name = state.selected_part_name
                    before_rpy = list(
                        state.part_visual_rpys.get(link_name, [0.0, 0.0, 0.0])
                    )
                    after_rpy = list(before_rpy)
                    after_rpy[axis_idx] += math.pi / 2
                    state.part_visual_rpys[link_name] = after_rpy
                    state.connection_dirty_links.add(link_name)
                    # Record undo
                    state.undo_stack.append(
                        {
                            "type": "rotate",
                            "link_name": link_name,
                            "axis": axis_idx,
                            "angle": math.pi / 2,
                            "before_rpy": before_rpy,
                            "after_rpy": after_rpy,
                        }
                    )
                    state.redo_stack.clear()
                    state.update_scene_now()
                    state.connection_status_label.text = (
                        f"Rotated {_get_part_name(state, link_name)}"
                        f" {axis_names[axis_idx]}+90\u00b0"
                    )

                return handler

            ui.button("Rot X", on_click=_make_rotate_handler(0)).props(
                "flat dense no-caps"
            )
            ui.button("Rot Y", on_click=_make_rotate_handler(1)).props(
                "flat dense no-caps"
            )
            ui.button("Rot Z", on_click=_make_rotate_handler(2)).props(
                "flat dense no-caps"
            )

            ui.separator().props("vertical")

            async def _remove_part_connections() -> None:
                """Remove connections for the selected part only."""
                if state.selected_part_name is None:
                    state.connection_status_label.text = "Select a part first"
                    return
                link_name = state.selected_part_name
                await _remove_connections_for_link(state, link_name)

            ui.button("Remove Part Conns", on_click=_remove_part_connections).props(
                "color=red-7 flat dense no-caps"
            )

            ui.separator().props("vertical")

            state.connection_status_label = ui.label(
                "Click mesh or marker to assign"
            ).style("font-size: 0.85rem; color: #666;")

            ui.space()

            async def _save_and_rebuild() -> None:
                """Write connection assignments to YAML and regenerate URDF."""
                analysis_dir = state.robot_dir / "analysis"
                for link_name, assignments in state.connection_assignments.items():
                    if link_name not in state.connection_dirty_links:
                        continue
                    link = state.robot.get_link(link_name)
                    if not link or not link.mesh_path:
                        continue
                    part_name = Path(link.mesh_path).stem
                    yaml_path = analysis_dir / f"{part_name}.yaml"
                    if not yaml_path.exists():
                        continue
                    model = load_part_yaml(yaml_path)
                    new_cps: list[ConnectionPoint] = []
                    for end, connection_data in assignments.items():
                        centering_val = connection_data.get("centering", "surface")
                        cp = ConnectionPoint(
                            end=end,  # type: ignore[arg-type]
                            position=[
                                round(connection_data["centroid"][i], 3)
                                for i in range(3)
                            ],
                            axis=quantize_axis(connection_data["normal"]),
                            radius_mm=round(connection_data["radius_mm"], 1),
                            method="manual",
                            centering=centering_val,
                        )
                        new_cps.append(cp)
                    if new_cps:
                        model.connection_points = new_cps
                        save_part_yaml(model, yaml_path)

                # Update chain.yaml: visual_xyz and visual_rpy for edited links
                chain_file = state.robot_dir / "chain.yaml"
                if chain_file.exists():
                    with open(chain_file) as f:
                        chain_data = yaml.safe_load(f)
                    chain_modified = False

                    link_specs = {
                        lk.get("mesh"): lk
                        for lk in chain_data.get("links", [])
                        if lk.get("mesh")
                    }

                    for link_name in state.connection_dirty_links:
                        link = state.robot.get_link(link_name)
                        if not link or not link.mesh_path:
                            continue
                        mesh_name = Path(link.mesh_path).stem
                        spec = link_specs.get(mesh_name)
                        if not spec:
                            continue
                        # Apply accumulated visual offset from Move Parts
                        offset = state.part_visual_offsets.get(link_name)
                        if offset and any(abs(d) > 1e-6 for d in offset):
                            cur = spec.get("visual_xyz", [0, 0, 0])
                            spec["visual_xyz"] = [
                                round(cur[i] + offset[i], 6) for i in range(3)
                            ]
                            chain_modified = True
                        elif "visual_xyz" in spec:
                            del spec["visual_xyz"]
                            chain_modified = True

                        # Apply accumulated visual rotation
                        extra_rpy = state.part_visual_rpys.get(link_name)
                        if extra_rpy and any(abs(v) > 1e-6 for v in extra_rpy):
                            _compose_and_save_rpy(spec, extra_rpy)
                            chain_modified = True

                    if chain_modified:
                        from robot_arm_sim.models.models import _FixedFloatDumper

                        with open(chain_file, "w") as f:
                            yaml.dump(
                                chain_data,
                                f,
                                Dumper=_FixedFloatDumper,
                                default_flow_style=False,
                            )

                # Regenerate URDF
                from robot_arm_sim.analyze.urdf_generator import generate_urdf

                if chain_file.exists():
                    generate_urdf(
                        chain_file,
                        analysis_dir,
                        state.robot_dir / "stl_files",
                        state.robot_dir / "robot.urdf",
                    )

                state.connection_status_label.text = "Saved! Reloading..."
                await state.reload_urdf()

            ui.button("Save & Rebuild", on_click=_save_and_rebuild).props(
                "color=orange-7 flat dense"
            )

            async def _remove_connections() -> None:
                """Bake current visual placement into visual_xyz, clear connections."""
                from robot_arm_sim.analyze.urdf_generator import generate_urdf
                from robot_arm_sim.models.models import _FixedFloatDumper

                urdf_path = state.robot_dir / "robot.urdf"
                chain_file = state.robot_dir / "chain.yaml"
                analysis_dir = state.robot_dir / "analysis"

                if not urdf_path.exists() or not chain_file.exists():
                    state.connection_status_label.text = "Missing URDF or chain.yaml"
                    return

                tree = ET.parse(urdf_path)  # noqa: S314
                urdf_origins: dict[str, list[float]] = {}
                for link_el in tree.findall("link"):
                    name = link_el.get("name", "")
                    vis = link_el.find("visual")
                    if vis is None:
                        continue
                    origin = vis.find("origin")
                    if origin is not None and origin.get("xyz"):
                        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
                    else:
                        xyz = [0.0, 0.0, 0.0]
                    urdf_origins[name] = xyz

                with open(chain_file) as f:
                    chain_data = yaml.safe_load(f)

                for link_spec in chain_data.get("links", []):
                    link_name = link_spec["name"]
                    xyz = urdf_origins.get(link_name, [0.0, 0.0, 0.0])
                    if any(abs(v) > 1e-9 for v in xyz):
                        link_spec["visual_xyz"] = [round(v, 6) for v in xyz]
                    elif "visual_xyz" in link_spec:
                        del link_spec["visual_xyz"]

                with open(chain_file, "w") as f:
                    yaml.dump(
                        chain_data,
                        f,
                        Dumper=_FixedFloatDumper,
                        default_flow_style=False,
                    )

                for yaml_file in sorted(analysis_dir.glob("*.yaml")):
                    if yaml_file.name == "summary.yaml":
                        continue
                    model = load_part_yaml(yaml_file)
                    if model.connection_points:
                        model.connection_points = []
                        save_part_yaml(model, yaml_file)

                generate_urdf(
                    chain_file,
                    analysis_dir,
                    state.robot_dir / "stl_files",
                    urdf_path,
                )

                state.connection_status_label.text = "Connections removed. Reloading..."
                await state.reload_urdf()

            ui.button("Remove Connections", on_click=_remove_connections).props(
                "color=red-7 flat dense"
            )

    def _toggle_edit_connections() -> None:
        state.edit_connections_active["value"] = not state.edit_connections_active[
            "value"
        ]
        active = state.edit_connections_active["value"]
        state.edit_connections_btn.text = "Exit Edit" if active else "Edit Connections"
        state.edit_connections_row.style(
            "width: 100%; gap: 4px; display: flex;"
            if active
            else "width: 100%; gap: 4px; display: none;"
        )
        ui.run_javascript(f"window.__faceEditMode = {str(active).lower()}")
        ui.run_javascript(
            f"if(window.__lockToOrthoViews)"
            f" window.__lockToOrthoViews({str(active).lower()})"
        )
        if active:
            ui.run_javascript("window.__setMeshTransparency(0.25)")
            state.connection_status_label.text = "Click mesh to assign"
            _seed_connection_assignments(state)
            state.conn_snapshot = copy.deepcopy(state.connection_assignments)
            state.connections_visible["value"] = True
            state.update_scene_now()
        else:
            # Restore assignments from snapshot, discarding unsaved edits
            state.connection_assignments.clear()
            if state.conn_snapshot:
                state.connection_assignments.update(copy.deepcopy(state.conn_snapshot))
            state.connection_dirty_links.clear()
            state.part_visual_offsets.clear()
            state.part_visual_rpys.clear()
            state.selected_part_name = None
            if state.selected_part_label is not None:
                state.selected_part_label.text = "(no part)"
            state.undo_stack.clear()
            state.redo_stack.clear()
            state.connection_center_target = None
            # Force-reset mesh positions so exit restores originals
            _force_reset_mesh_positions(state)
            if not state.transparent_mode["value"]:
                ui.run_javascript("window.__setMeshTransparency(1.0)")
            state.connections_visible["value"] = True
            state.update_scene_now()
            if state.connections_btn is not None:
                state.connections_btn.props("dense color=primary")
                state.connections_btn.classes(add="bg-blue-1")
                state.connections_btn.update()

    state.toggle_edit_connections = _toggle_edit_connections

    def _on_visibility_changed() -> None:
        if state.edit_connections_active["value"]:
            _sync_connection_visibility(state)

    state.on_visibility_changed = _on_visibility_changed

    # Polling timer for face clicks, part moves, and part selections
    async def _poll_face_click() -> None:
        if not state.edit_connections_active["value"]:
            return
        try:
            result = await ui.run_javascript(
                "(() => { const v = window.__lastFaceClick;"
                " window.__lastFaceClick = null; return v; })()",
                timeout=0.5,
            )
        except (TimeoutError, RuntimeError):
            return
        if result and isinstance(result, dict):
            if result.get("type") == "mesh":
                _handle_mesh_click(state, result)

        # Poll for part move completions
        try:
            move = await ui.run_javascript(
                "(() => { const v = window.__lastPartMove;"
                " window.__lastPartMove = null; return v; })()",
                timeout=0.5,
            )
        except (TimeoutError, RuntimeError):
            return
        if move and isinstance(move, dict):
            _handle_part_move(state, move)

        # Poll for part selection (from drag mousedown)
        try:
            sel = await ui.run_javascript(
                "(() => { const v = window.__lastPartSelect;"
                " window.__lastPartSelect = null; return v; })()",
                timeout=0.5,
            )
        except (TimeoutError, RuntimeError):
            return
        if sel and isinstance(sel, dict):
            ln = sel.get("linkName")
            if ln:
                _update_selected_part(state, ln)

    poll_timer = ui.timer(0.2, _poll_face_click)

    # Deactivate timer when the client disconnects (robot switch / page close)
    ui.context.client.on_disconnect(lambda: setattr(poll_timer, "active", False))


def _compose_and_save_rpy(spec: dict, extra_rpy: list[float]) -> None:
    """Compose extra_rpy with existing visual_rpy via rotation matrix multiplication."""
    from robot_arm_sim.analyze.urdf_transforms import (
        rotation_matrix_to_rpy,
        rpy_to_rotation,
    )

    existing = spec.get("visual_rpy", [0, 0, 0])
    r_existing = rpy_to_rotation(existing)
    r_extra = rpy_to_rotation(extra_rpy)
    r_composed = r_existing @ r_extra
    composed_rpy = rotation_matrix_to_rpy(r_composed)
    spec["visual_rpy"] = [round(v, 6) for v in composed_rpy]


async def _remove_connections_for_link(state: SimulatorState, link_name: str) -> None:
    """Remove connections for a single link — bake placement, clear CPs."""
    from robot_arm_sim.analyze.urdf_generator import generate_urdf
    from robot_arm_sim.models.models import _FixedFloatDumper

    urdf_path = state.robot_dir / "robot.urdf"
    chain_file = state.robot_dir / "chain.yaml"
    analysis_dir = state.robot_dir / "analysis"

    if not urdf_path.exists() or not chain_file.exists():
        state.connection_status_label.text = "Missing URDF or chain.yaml"
        return

    link = state.robot.get_link(link_name)
    if not link or not link.mesh_path:
        state.connection_status_label.text = f"No mesh for {link_name}"
        return

    part_name = Path(link.mesh_path).stem

    # Parse URDF to get visual origin for this link
    tree = ET.parse(urdf_path)  # noqa: S314
    xyz = [0.0, 0.0, 0.0]
    for link_el in tree.findall("link"):
        if link_el.get("name") == link_name:
            vis = link_el.find("visual")
            if vis is not None:
                origin = vis.find("origin")
                if origin is not None and origin.get("xyz"):
                    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            break

    # Update chain.yaml for this link only
    with open(chain_file) as f:
        chain_data = yaml.safe_load(f)

    for link_spec in chain_data.get("links", []):
        if link_spec["name"] == link_name:
            if any(abs(v) > 1e-9 for v in xyz):
                link_spec["visual_xyz"] = [round(v, 6) for v in xyz]
            elif "visual_xyz" in link_spec:
                del link_spec["visual_xyz"]
            break

    with open(chain_file, "w") as f:
        yaml.dump(chain_data, f, Dumper=_FixedFloatDumper, default_flow_style=False)

    # Clear connection_points from this part's analysis YAML only
    yaml_path = analysis_dir / f"{part_name}.yaml"
    if yaml_path.exists():
        model = load_part_yaml(yaml_path)
        if model.connection_points:
            model.connection_points = []
            save_part_yaml(model, yaml_path)

    # Clear this part's connection_assignments
    if link_name in state.connection_assignments:
        del state.connection_assignments[link_name]

    # Regenerate URDF and reload
    generate_urdf(chain_file, analysis_dir, state.robot_dir / "stl_files", urdf_path)

    state.connection_status_label.text = (
        f"Removed {part_name} connections. Reloading..."
    )
    await state.reload_urdf()


def _reverse_action(state: SimulatorState, action: dict) -> None:
    """Reverse an undo action."""
    atype = action["type"]
    link_name = action["link_name"]

    if atype == "assign":
        before = action["before"]
        end = action["end"]
        if before is None:
            # Was a new assignment — remove it
            assigns = state.connection_assignments.get(link_name, {})
            assigns.pop(end, None)
        else:
            if link_name not in state.connection_assignments:
                state.connection_assignments[link_name] = {}
            state.connection_assignments[link_name][end] = before
            _move_connection_sphere(
                state,
                link_name,
                end,
                before["centroid"],
                centering=before.get("centering", "surface"),
            )
        state.connection_status_label.text = "Undone assign"

    elif atype == "move":
        delta = action["delta"]
        prev = state.part_visual_offsets.get(link_name, [0, 0, 0])
        state.part_visual_offsets[link_name] = [prev[i] - delta[i] for i in range(3)]
        _force_reset_mesh_positions(state)
        state.update_scene_now()
        state.connection_status_label.text = "Undone move"

    elif atype == "rotate":
        state.part_visual_rpys[link_name] = list(action["before_rpy"])
        state.update_scene_now()
        state.connection_status_label.text = "Undone rotate"


def _apply_action(state: SimulatorState, action: dict) -> None:
    """Re-apply a redo action."""
    atype = action["type"]
    link_name = action["link_name"]

    if atype == "assign":
        after = action["after"]
        end = action["end"]
        if link_name not in state.connection_assignments:
            state.connection_assignments[link_name] = {}
        state.connection_assignments[link_name][end] = after
        _move_connection_sphere(
            state,
            link_name,
            end,
            after["centroid"],
            centering=after.get("centering", "surface"),
        )
        state.connection_status_label.text = "Redone assign"

    elif atype == "move":
        delta = action["delta"]
        prev = state.part_visual_offsets.get(link_name, [0, 0, 0])
        state.part_visual_offsets[link_name] = [prev[i] + delta[i] for i in range(3)]
        _force_reset_mesh_positions(state)
        state.update_scene_now()
        state.connection_status_label.text = "Redone move"

    elif atype == "rotate":
        state.part_visual_rpys[link_name] = list(action["after_rpy"])
        state.update_scene_now()
        state.connection_status_label.text = "Redone rotate"


def _sync_connection_visibility(state: SimulatorState) -> None:
    """Show/hide connection markers based on part visibility and Show All."""
    show_all = state.show_all_connections.get("value", False)
    vis_map: dict[str, bool] = {}
    for link_name, cps in state.connection_points.items():
        visible = show_all or state.visible_links.get(link_name, True)
        for i, cp in enumerate(cps):
            sphere_id = f"{link_name}_{cp['end']}_{i}"
            vis_map[sphere_id] = visible
    js = (
        "if(window.__connSpheres){"
        + "".join(
            f"var s=window.__connSpheres['{sid}'];"
            f"if(s)s.visible={str(vis).lower()};"
            f"var l=window.__connLines&&window.__connLines['{sid}'];"
            f"if(l)l.visible={str(vis).lower()};"
            for sid, vis in vis_map.items()
        )
        + "}"
    )
    ui.run_javascript(js)


def _get_sphere_id(state: SimulatorState, link_name: str, end: str) -> str | None:
    """Find the view-mode sphere ID for a given (link_name, end) pair."""
    cps = state.connection_points.get(link_name, [])
    for i, cp in enumerate(cps):
        if cp["end"] == end:
            return f"{link_name}_{end}_{i}"
    return None


def _move_connection_sphere(
    state: SimulatorState,
    link_name: str,
    end: str,
    centroid_mm: list[float],
    centering: str = "surface",
) -> None:
    """Move a view-mode connection sphere to a new position."""
    transforms = _get_visual_transforms(state)
    tf_vis = transforms.get(link_name)
    if tf_vis is None:
        return

    c_m = [centroid_mm[i] * 0.001 for i in range(3)]
    c_homo = np.array([c_m[0], c_m[1], c_m[2], 1.0])
    wp = tf_vis @ c_homo

    sphere_id = _get_sphere_id(state, link_name, end)
    if not sphere_id:
        return

    if centering == "center":
        color = "0x4488ff"
    elif end == "proximal":
        color = "0x00cc00"
    else:
        color = "0xcc0000"
    ui.run_javascript(
        f"window.__moveConnectionSphere("
        f"'{sphere_id}', {wp[0]:.6f}, {wp[1]:.6f}, {wp[2]:.6f},"
        f" {color})"
    )


def _get_visual_transforms(
    state: SimulatorState,
) -> dict[str, np.ndarray]:
    """Compute current visual transforms for all links."""
    from ..kinematics import (
        forward_kinematics,
        rpy_to_matrix,
        translation_matrix,
    )

    transforms = forward_kinematics(state.robot, state.joint_angles)
    visual_transforms: dict[str, np.ndarray] = {}
    for link_name, tf in transforms.items():
        link = state.robot.get_link(link_name)
        if link:
            visual_tf = translation_matrix(link.origin_xyz) @ rpy_to_matrix(
                link.origin_rpy
            )
            visual_transforms[link_name] = tf @ visual_tf
        else:
            visual_transforms[link_name] = tf
    return visual_transforms


def _assign_connection(
    state: SimulatorState,
    link_name: str,
    end: str,
    connection_data: dict,
) -> None:
    """Assign a connection point and update the view-mode sphere."""
    if link_name not in state.connection_assignments:
        state.connection_assignments[link_name] = {}

    assigns = state.connection_assignments[link_name]

    # Record undo before overwriting
    before = copy.deepcopy(assigns.get(end))
    assigns[end] = connection_data
    state.undo_stack.append(
        {
            "type": "assign",
            "link_name": link_name,
            "end": end,
            "before": before,
            "after": copy.deepcopy(connection_data),
        }
    )
    state.redo_stack.clear()

    _move_connection_sphere(
        state,
        link_name,
        end,
        connection_data["centroid"],
        centering=connection_data.get("centering", "surface"),
    )

    state.connection_dirty_links.add(link_name)

    centering_val = connection_data.get("centering", "surface")
    state.connection_centering_select.value = centering_val
    state.connection_centering["value"] = centering_val
    state.connection_center_target = (link_name, end)

    state.connection_status_label.text = f"Set {_get_part_name(state, link_name)} {end}"


def _seed_connection_assignments(state: SimulatorState) -> None:
    """Pre-populate connection_assignments from analysis connection_points."""
    for link_name, cps in state.connection_points.items():
        if link_name in state.connection_assignments:
            continue
        assigns: dict[str, dict] = {}
        for cp in cps:
            end = cp["end"]
            pos = cp["position"]
            cp_centering = cp.get("centering", "surface")
            assigns[end] = {
                "centroid": [float(pos[i]) for i in range(3)],
                "normal": [float(cp["axis"][i]) for i in range(3)],
                "area_mm2": math.pi * cp["radius_mm"] ** 2,
                "radius_mm": cp["radius_mm"],
                "centering": cp_centering,
            }
        if assigns:
            state.connection_assignments[link_name] = assigns


def _handle_part_move(state: SimulatorState, move: dict) -> None:
    """Process a completed part drag — store visual offset delta."""
    link_name = move.get("linkName")
    delta = move.get("delta", [0, 0, 0])
    if not link_name or all(abs(d) < 1e-6 for d in delta):
        return

    prev = state.part_visual_offsets.get(link_name, [0, 0, 0])
    state.part_visual_offsets[link_name] = [prev[i] + delta[i] for i in range(3)]
    state.connection_dirty_links.add(link_name)

    # Record undo
    state.undo_stack.append(
        {"type": "move", "link_name": link_name, "delta": list(delta)}
    )
    state.redo_stack.clear()

    state.connection_status_label.text = f"Moved {_get_part_name(state, link_name)}"


def _handle_mesh_click(state: SimulatorState, result: dict) -> None:
    """Process a direct mesh click — place connection at click point."""
    mode = state.connection_edit_mode["value"]
    link_name = result["linkName"]

    # Always update selected part on click
    _update_selected_part(state, link_name)

    if mode == "move_parts":
        return  # dragging handled in JS

    if mode == "proximal_centred":
        end = "proximal"
        centering = "center"
    elif mode == "proximal_surface":
        end = "proximal"
        centering = "surface"
    else:  # distal
        end = "distal"
        centering = "surface"

    wp = result.get("point", [0, 0, 0])
    wn = result.get("normal", [0, 0, 1])

    transforms = _get_visual_transforms(state)
    tf_vis = transforms.get(link_name)
    if tf_vis is None:
        return

    tf_inv = np.linalg.inv(tf_vis)
    stl_pt = (tf_inv @ np.array([wp[0], wp[1], wp[2], 1.0]))[:3] * 1000.0
    stl_n = tf_inv[:3, :3] @ np.array([wn[0], wn[1], wn[2]])
    norm = float(np.linalg.norm(stl_n))
    if norm > 1e-8:
        stl_n = stl_n / norm

    connection_data = {
        "centroid": [round(float(v), 4) for v in stl_pt],
        "normal": [round(float(v), 4) for v in stl_n],
        "area_mm2": 78.54,
        "radius_mm": 5.0,
        "centering": centering,
    }
    _assign_connection(state, link_name, end, connection_data)
