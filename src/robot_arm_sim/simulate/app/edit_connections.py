"""Edit Connections mode — interactive connection point assignment."""

from __future__ import annotations

import copy
import math
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


def build_edit_connections(state: SimulatorState) -> None:
    """Build the Edit Connections mode UI and handlers."""
    state.edit_connections_row = (
        ui.row()
        .classes("q-pa-sm items-center")
        .style("width: 100%; gap: 8px; display: none;")
    )
    with state.edit_connections_row:
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
                # Sync legacy state for compatibility
                if mode_key.startswith("proximal"):
                    state.connection_end_toggle["value"] = "Proximal"
                    state.connection_centering["value"] = (
                        "center" if mode_key == "proximal_centred" else "surface"
                    )
                elif mode_key == "distal":
                    state.connection_end_toggle["value"] = "Distal"
                    state.connection_centering["value"] = "surface"
                # Enable/disable part dragging
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

        # Dummy centering select for compatibility with _assign_connection
        state.connection_centering_select = type("_Dummy", (), {"value": "surface"})()

        ui.separator().props("vertical")

        def _on_show_all(e: Any) -> None:
            state.show_all_connections["value"] = e.value
            _sync_connection_visibility(state)

        ui.checkbox("Show All", value=False).props("dense").on_value_change(
            _on_show_all
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
                            round(connection_data["centroid"][i], 3) for i in range(3)
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

            # Update chain.yaml: clear stale visual_xyz for edited links.
            chain_file = state.robot_dir / "chain.yaml"
            if chain_file.exists():
                with open(chain_file) as f:
                    chain_data = yaml.safe_load(f)
                chain_modified = False

                # Build mesh→link-spec lookup
                link_specs = {
                    lk.get("mesh"): lk
                    for lk in chain_data.get("links", [])
                    if lk.get("mesh")
                }

                # Clear visual_xyz for edited links; apply part move offsets
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

                if chain_modified:
                    with open(chain_file, "w") as f:
                        yaml.dump(chain_data, f, default_flow_style=False)

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

    def _toggle_edit_connections() -> None:
        state.edit_connections_active["value"] = not state.edit_connections_active[
            "value"
        ]
        active = state.edit_connections_active["value"]
        state.edit_connections_btn.text = "Exit Edit" if active else "Edit Connections"
        state.edit_connections_row.style(
            "width: 100%; gap: 8px; display: flex;"
            if active
            else "width: 100%; gap: 8px; display: none;"
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
            # Show connection markers (reuse the view-mode spheres)
            state.connections_visible["value"] = True
            state.update_scene_now()
        else:
            # Restore assignments from snapshot, discarding unsaved edits
            state.connection_assignments.clear()
            if state.conn_snapshot:
                state.connection_assignments.update(copy.deepcopy(state.conn_snapshot))
            state.connection_dirty_links.clear()
            state.part_visual_offsets.clear()
            state.connection_center_target = None
            if not state.transparent_mode["value"]:
                ui.run_javascript("window.__setMeshTransparency(1.0)")
            # Re-show view-mode connection markers at original positions
            state.connections_visible["value"] = True
            state.update_scene_now()
            # Sync the toolbar Connections button style
            if state.connections_btn is not None:
                state.connections_btn.props("dense color=primary")
                state.connections_btn.classes(add="bg-blue-1")
                state.connections_btn.update()

    state.toggle_edit_connections = _toggle_edit_connections

    def _on_visibility_changed() -> None:
        if state.edit_connections_active["value"]:
            _sync_connection_visibility(state)

    state.on_visibility_changed = _on_visibility_changed

    # Polling timer for face clicks and part moves
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

    poll_timer = ui.timer(0.2, _poll_face_click)

    # Deactivate timer when the client disconnects (robot switch / page close)
    ui.context.client.on_disconnect(lambda: setattr(poll_timer, "active", False))


def _sync_connection_visibility(state: SimulatorState) -> None:
    """Show/hide connection markers based on part visibility and Show All."""
    show_all = state.show_all_connections.get("value", False)
    vis_map: dict[str, bool] = {}
    for link_name, cps in state.connection_points.items():
        visible = show_all or state.visible_links.get(link_name, True)
        for i, cp in enumerate(cps):
            sphere_id = f"{link_name}_{cp['end']}_{i}"
            vis_map[sphere_id] = visible
    # Use __updateConnectionPoses visibility flag (pos[3])
    # to selectively show/hide — just toggle the sphere directly
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
    assigns[end] = connection_data
    _move_connection_sphere(
        state,
        link_name,
        end,
        connection_data["centroid"],
        centering=connection_data.get("centering", "surface"),
    )

    # Mark this link as user-modified
    state.connection_dirty_links.add(link_name)

    # Sync centering state
    centering_val = connection_data.get("centering", "surface")
    state.connection_centering_select.value = centering_val
    state.connection_centering["value"] = centering_val
    state.connection_center_target = (link_name, end)

    part_link = state.robot.get_link(link_name)
    part_name = (
        Path(part_link.mesh_path).stem
        if part_link and part_link.mesh_path
        else link_name
    )
    state.connection_status_label.text = f"Set {part_name} {end}"


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

    # Accumulate visual offset (world-space meters)
    prev = state.part_visual_offsets.get(link_name, [0, 0, 0])
    state.part_visual_offsets[link_name] = [prev[i] + delta[i] for i in range(3)]
    state.connection_dirty_links.add(link_name)

    part_link = state.robot.get_link(link_name)
    part_name = (
        Path(part_link.mesh_path).stem
        if part_link and part_link.mesh_path
        else link_name
    )
    state.connection_status_label.text = f"Moved {part_name}"


def _handle_mesh_click(state: SimulatorState, result: dict) -> None:
    """Process a direct mesh click — place connection at click point."""
    mode = state.connection_edit_mode["value"]
    if mode == "move_parts":
        return  # dragging handled in JS

    link_name = result["linkName"]

    if mode == "proximal_centred":
        end = "proximal"
        centering = "center"
    elif mode == "proximal_surface":
        end = "proximal"
        centering = "surface"
    else:  # distal
        end = "distal"
        centering = "surface"

    # World-space hit point and face normal from JS (meters)
    wp = result.get("point", [0, 0, 0])
    wn = result.get("normal", [0, 0, 1])

    # Convert world-space → STL-space (mm) via inverse visual transform
    transforms = _get_visual_transforms(state)
    tf_vis = transforms.get(link_name)
    if tf_vis is None:
        return

    tf_inv = np.linalg.inv(tf_vis)
    # Point: full affine transform, then m → mm
    stl_pt = (tf_inv @ np.array([wp[0], wp[1], wp[2], 1.0]))[:3] * 1000.0
    # Normal: rotation only (no translation)
    stl_n = tf_inv[:3, :3] @ np.array([wn[0], wn[1], wn[2]])
    norm = float(np.linalg.norm(stl_n))
    if norm > 1e-8:
        stl_n = stl_n / norm

    connection_data = {
        "centroid": [round(float(v), 4) for v in stl_pt],
        "normal": [round(float(v), 4) for v in stl_n],
        "area_mm2": 78.54,  # cosmetic default ~5mm radius circle
        "radius_mm": 5.0,
        "centering": centering,
    }
    _assign_connection(state, link_name, end, connection_data)
