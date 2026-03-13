"""Edit Bores mode — interactive bore point assignment."""

from __future__ import annotations

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


def build_edit_bores(state: SimulatorState) -> None:
    """Build the Edit Bores mode UI and handlers."""
    state.edit_bores_row = (
        ui.row()
        .classes("q-pa-sm items-center")
        .style("width: 100%; gap: 8px; display: none;")
    )
    with state.edit_bores_row:
        bore_toggle = ui.toggle(
            {"Proximal": "Proximal", "Distal": "Distal"},
            value="Proximal",
        ).props("dense flat no-caps toggle-color='' text-color=''")

        def _update_toggle_style(val: str) -> None:
            color = "green" if val == "Proximal" else "red"
            bore_toggle.props(f"toggle-color={color}")
            bore_toggle.update()

        def _on_bore_toggle(e: Any) -> None:
            state.bore_end_toggle["value"] = e.value
            _update_toggle_style(e.value)

        bore_toggle.on_value_change(_on_bore_toggle)
        _update_toggle_style("Proximal")

        ui.separator().props("vertical")

        centering_options = {
            "surface_bbox": "Surface BBox",
            "surface": "Surface",
            "center": "Center",
        }
        ui.label("Centering").style("font-size: 0.85rem;")
        centering_select = ui.select(
            centering_options,
            value="surface_bbox",
        ).props("dense options-dense borderless style='min-width: 110px;'")

        def _on_centering(e: Any) -> None:
            state.bore_centering["value"] = e.value
            target = getattr(state, "bore_center_target", None)
            if target:
                link_name, end = target
                assigns = state.bore_assignments.get(link_name, {})
                if end in assigns:
                    assigns[end]["centering"] = e.value
                    state.bore_dirty_links.add(link_name)

        centering_select.on_value_change(_on_centering)

        state.bore_centering_select = centering_select

        ui.separator().props("vertical")

        def _on_show_all(e: Any) -> None:
            state.show_all_bores["value"] = e.value
            _sync_bore_edit_visibility(state)

        ui.checkbox("Show All", value=False).props("dense").on_value_change(
            _on_show_all
        )

        ui.separator().props("vertical")

        state.bore_status_label = ui.label("Click mesh or marker to assign").style(
            "font-size: 0.85rem; color: #666;"
        )

        ui.space()

        async def _save_and_rebuild() -> None:
            """Write bore assignments to YAML and regenerate URDF."""
            analysis_dir = state.robot_dir / "analysis"
            for link_name, assignments in state.bore_assignments.items():
                if link_name not in state.bore_dirty_links:
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
                for end, bore_data in assignments.items():
                    centering_val = bore_data.get("centering", "surface_bbox")
                    cp = ConnectionPoint(
                        end=end,  # type: ignore[arg-type]
                        position=[round(bore_data["centroid"][i], 3) for i in range(3)],
                        axis=quantize_axis(bore_data["normal"]),
                        radius_mm=round(bore_data["radius_mm"], 1),
                        method="manual",
                        centering=centering_val
                        if centering_val != "surface_bbox"
                        else None,
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

                # Clear visual_xyz for any link whose bores were edited
                for link_name in state.bore_dirty_links:
                    link = state.robot.get_link(link_name)
                    if not link or not link.mesh_path:
                        continue
                    mesh_name = Path(link.mesh_path).stem
                    spec = link_specs.get(mesh_name)
                    if spec and "visual_xyz" in spec:
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

            state.bore_status_label.text = "Saved! Reloading..."
            await state.reload_urdf()

        ui.button("Save & Rebuild", on_click=_save_and_rebuild).props(
            "color=orange-7 flat dense"
        )

    def _toggle_edit_bores() -> None:
        state.edit_bores_active["value"] = not state.edit_bores_active["value"]
        active = state.edit_bores_active["value"]
        state.edit_bores_btn.text = "Exit Edit" if active else "Edit Bores"
        state.edit_bores_row.style(
            "width: 100%; gap: 8px; display: flex;"
            if active
            else "width: 100%; gap: 8px; display: none;"
        )
        ui.run_javascript(
            f"window.__faceEditMode = {str(active).lower()};"
            f" window.__setBoreEditMarkersVisible({str(active).lower()})"
        )
        if active:
            ui.run_javascript("window.__setMeshTransparency(0.25)")
            state.bore_status_label.text = "Click mesh to assign"
            _seed_bore_assignments(state)
            _show_existing_bore_markers(state)
            state.update_scene_now()
        else:
            if not state.transparent_mode["value"]:
                ui.run_javascript("window.__setMeshTransparency(1.0)")

    state.toggle_edit_bores = _toggle_edit_bores

    def _on_visibility_changed() -> None:
        if state.edit_bores_active["value"]:
            _sync_bore_edit_visibility(state)

    state.on_visibility_changed = _on_visibility_changed

    # Polling timer for face clicks
    async def _poll_face_click() -> None:
        if not state.edit_bores_active["value"]:
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

    ui.timer(0.2, _poll_face_click)


def _show_existing_bore_markers(state: SimulatorState) -> None:
    """Place all existing bore assignments as markers, then sync visibility."""
    for link_name, assigns in state.bore_assignments.items():
        for end, bore_data in assigns.items():
            _place_bore_marker(
                state, link_name, end, bore_data["centroid"], bore_data["radius_mm"]
            )
    _sync_bore_edit_visibility(state)


def _sync_bore_edit_visibility(state: SimulatorState) -> None:
    """Show/hide bore edit markers based on part visibility and Show All."""
    import json

    show_all = state.show_all_bores.get("value", False)
    vis_map: dict[str, bool] = {}
    for link_name, assigns in state.bore_assignments.items():
        visible = show_all or state.visible_links.get(link_name, True)
        for end in assigns:
            marker_id = f"bore_edit_{link_name}_{end}"
            vis_map[marker_id] = visible
    ui.run_javascript(f"window.__setBoreEditMarkerVisibility({json.dumps(vis_map)})")


def _place_bore_marker(
    state: SimulatorState,
    link_name: str,
    end: str,
    centroid_mm: list[float],
    radius_mm: float = 8.0,
) -> None:
    """Place/update a bore edit marker sphere at the given position."""
    # Transform centroid from STL-space (mm) to world-space (m)
    transforms = _get_visual_transforms(state)
    tf_vis = transforms.get(link_name)
    if tf_vis is None:
        return

    c_m = [centroid_mm[i] * 0.001 for i in range(3)]
    c_homo = np.array([c_m[0], c_m[1], c_m[2], 1.0])
    wp = tf_vis @ c_homo

    marker_id = f"bore_edit_{link_name}_{end}"
    color = "0x00cc00" if end == "proximal" else "0xcc0000"
    ui.run_javascript(
        f"window.__placeBoreEditMarker("
        f"'{marker_id}', {wp[0]:.6f}, {wp[1]:.6f}, {wp[2]:.6f},"
        f" {color}, {radius_mm:.1f})"
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


def _assign_bore(
    state: SimulatorState,
    link_name: str,
    end: str,
    bore_data: dict,
) -> None:
    """Assign a bore point and update markers."""
    if link_name not in state.bore_assignments:
        state.bore_assignments[link_name] = {}

    assigns = state.bore_assignments[link_name]

    # Remove old marker for this end if any
    if end in assigns:
        old_id = f"bore_edit_{link_name}_{end}"
        ui.run_javascript(f"window.__removeBoreEditMarker('{old_id}')")

    assigns[end] = bore_data
    _place_bore_marker(
        state, link_name, end, bore_data["centroid"], bore_data["radius_mm"]
    )

    # Mark this link as user-modified
    state.bore_dirty_links.add(link_name)

    # Sync centering dropdown to this bore's state
    centering_val = bore_data.get("centering", "surface_bbox")
    state.bore_centering_select.value = centering_val
    state.bore_centering["value"] = centering_val
    # Track which bore is currently selected for checkbox toggling
    state.bore_center_target = (link_name, end)

    part_link = state.robot.get_link(link_name)
    part_name = (
        Path(part_link.mesh_path).stem
        if part_link and part_link.mesh_path
        else link_name
    )
    state.bore_status_label.text = f"Set {part_name} {end}"


def _seed_bore_assignments(state: SimulatorState) -> None:
    """Pre-populate bore_assignments from analysis connection_points."""
    for link_name, cps in state.connection_points.items():
        if link_name in state.bore_assignments:
            continue
        assigns: dict[str, dict] = {}
        for cp in cps:
            end = cp["end"]
            pos = cp["position"]
            # Resolve centering: new key takes precedence, fall back to
            # legacy 'center' boolean, default to surface_bbox.
            cp_centering = cp.get("centering")
            if cp_centering is None:
                cp_centering = "center" if cp.get("center", False) else "surface_bbox"
            assigns[end] = {
                "centroid": [float(pos[i]) for i in range(3)],
                "normal": [float(cp["axis"][i]) for i in range(3)],
                "area_mm2": math.pi * cp["radius_mm"] ** 2,
                "radius_mm": cp["radius_mm"],
                "centering": cp_centering,
            }
        if assigns:
            state.bore_assignments[link_name] = assigns


def _handle_mesh_click(state: SimulatorState, result: dict) -> None:
    """Process a direct mesh click — place bore at click point."""
    link_name = result["linkName"]
    end = state.bore_end_toggle["value"].lower()

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

    bore_data = {
        "centroid": [round(float(v), 4) for v in stl_pt],
        "normal": [round(float(v), 4) for v in stl_n],
        "area_mm2": 78.54,  # cosmetic default ~5mm radius circle
        "radius_mm": 5.0,
        "centering": state.bore_centering["value"],
    }
    _assign_bore(state, link_name, end, bore_data)
