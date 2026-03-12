"""Edit Bores mode — interactive bore point assignment."""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import yaml
from nicegui import ui

from .loaders import quantize_axis

if TYPE_CHECKING:
    from .state import SimulatorState


def build_edit_bores(state: SimulatorState) -> None:
    """Build the Edit Bores mode UI and handlers."""
    state.edit_bores_row = (
        ui.row().classes("q-pa-sm").style("width: 100%; gap: 8px; display: none;")
    )
    with state.edit_bores_row:
        bore_toggle = ui.toggle(["Proximal", "Distal"], value="Proximal").props("dense")

        def _on_bore_toggle(e: Any) -> None:
            state.bore_end_toggle["value"] = e.value

        bore_toggle.on_value_change(_on_bore_toggle)

        kk_cb = ui.checkbox("Keep Kinematics", value=True).props("dense")

        def _on_keep_kin(e: Any) -> None:
            state.keep_kinematics["value"] = e.value

        kk_cb.on_value_change(_on_keep_kin)

        state.bore_status_label = ui.label("Click mesh or marker to assign").style(
            "font-size: 0.85rem; color: #666;"
        )

        ui.space()

        async def _save_and_rebuild() -> None:
            """Write bore assignments to YAML and regenerate URDF."""
            analysis_dir = state.robot_dir / "analysis"
            for link_name, assignments in state.bore_assignments.items():
                link = state.robot.get_link(link_name)
                if not link or not link.mesh_path:
                    continue
                part_name = Path(link.mesh_path).stem
                yaml_path = analysis_dir / f"{part_name}.yaml"
                if not yaml_path.exists():
                    continue
                with open(yaml_path) as f:
                    data = yaml.safe_load(f)
                new_cps = []
                for end, bore_data in assignments.items():
                    new_cps.append(
                        {
                            "end": end,
                            "position": [
                                round(bore_data["centroid"][i], 3) for i in range(3)
                            ],
                            "axis": quantize_axis(bore_data["normal"]),
                            "radius_mm": round(bore_data["radius_mm"], 1),
                            "method": "manual",
                        }
                    )
                if new_cps:
                    data["connection_points"] = new_cps
                    with open(yaml_path, "w") as f:
                        yaml.dump(
                            data,
                            f,
                            default_flow_style=False,
                            sort_keys=False,
                        )

            # Propagate bore axes to chain.yaml (only if Keep Kinematics off)
            chain_file = state.robot_dir / "chain.yaml"
            if not state.keep_kinematics["value"] and chain_file.exists():
                with open(chain_file) as f:
                    chain_data = yaml.safe_load(f)
                link_meshes = {
                    lk["name"]: lk.get("mesh") for lk in chain_data.get("links", [])
                }
                chain_modified = False
                for jnt in chain_data.get("joints", []):
                    parent_mesh = link_meshes.get(jnt["parent"])
                    if not parent_mesh:
                        continue
                    ay = analysis_dir / f"{parent_mesh}.yaml"
                    if not ay.exists():
                        continue
                    with open(ay) as f:
                        adata = yaml.safe_load(f)
                    cps = adata.get("connection_points", [])
                    distal = next(
                        (c for c in cps if c["end"] == "distal"),
                        None,
                    )
                    if distal is None:
                        continue
                    bore_axis = [int(v) if v == int(v) else v for v in distal["axis"]]
                    if jnt.get("axis") != bore_axis:
                        jnt["axis"] = bore_axis
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
        ui.run_javascript(f"window.__setFaceMarkersVisible({str(active).lower()})")
        if active:
            ui.run_javascript("window.__setMeshTransparency(0.25)")
            state.bore_status_label.text = "Click mesh or marker to assign"
            _show_existing_bore_markers(state)
            state.update_scene_now()
        else:
            if not state.transparent_mode["value"]:
                ui.run_javascript("window.__setMeshTransparency(1.0)")

    state.toggle_edit_bores = _toggle_edit_bores

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
        if result:
            if isinstance(result, dict):
                if result.get("type") == "marker":
                    _handle_marker_click(state, result["name"])
                elif result.get("type") == "mesh":
                    _handle_mesh_click(state, result)
            elif isinstance(result, str):
                _handle_marker_click(state, result)

    ui.timer(0.2, _poll_face_click)


def _show_existing_bore_markers(state: SimulatorState) -> None:
    """Show existing bore assignments as colored markers on enter."""
    for link_name, assigns in state.bore_assignments.items():
        for end, bore_data in assigns.items():
            _place_bore_marker(state, link_name, end, bore_data["centroid"])


def _place_bore_marker(
    state: SimulatorState,
    link_name: str,
    end: str,
    centroid_mm: list[float],
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
        f"'{marker_id}', {wp[0]:.6f}, {wp[1]:.6f}, {wp[2]:.6f}, {color})"
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
    _place_bore_marker(state, link_name, end, bore_data["centroid"])

    part_link = state.robot.get_link(link_name)
    part_name = (
        Path(part_link.mesh_path).stem
        if part_link and part_link.mesh_path
        else link_name
    )
    state.bore_status_label.text = f"Set {part_name} {end}"


def _handle_marker_click(state: SimulatorState, name: str) -> None:
    """Process a face marker click: face_{link}_{idx}."""
    parts = name.split("_", 1)
    if len(parts) < 2 or parts[0] != "face":
        return
    remainder = parts[1]
    matched_link = None
    matched_idx = None
    for lname in state.flat_faces:
        prefix = lname + "_"
        if remainder.startswith(prefix):
            try:
                idx = int(remainder[len(prefix) :])
                matched_link = lname
                matched_idx = idx
                break
            except ValueError:
                continue
    if matched_link is None or matched_idx is None:
        return

    link_name = matched_link
    face_idx = matched_idx
    end = state.bore_end_toggle["value"].lower()

    faces_list = state.flat_faces.get(link_name, [])
    if face_idx >= len(faces_list):
        return
    ff = faces_list[face_idx]
    radius = math.sqrt(ff["area_mm2"] / math.pi)

    bore_data = {
        "centroid": ff["centroid"],
        "normal": ff["normal"],
        "area_mm2": ff["area_mm2"],
        "radius_mm": radius,
    }
    _assign_bore(state, link_name, end, bore_data)

    # Recolour face markers for this link
    assigns = state.bore_assignments.get(link_name, {})
    assigned_centroids = {e: d["centroid"] for e, d in assigns.items()}
    for i in range(len(faces_list)):
        fid = f"face_{link_name}_{i}"
        fc = faces_list[i]["centroid"]
        matched_end = None
        for e, c in assigned_centroids.items():
            if c == fc:
                matched_end = e
                break
        if matched_end:
            color = "0x00cc00" if matched_end == "proximal" else "0xcc0000"
        else:
            color = "0xffcc00"
        ui.run_javascript(f"window.__setFaceMarkerColor('{fid}', {color})")


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
    }
    _assign_bore(state, link_name, end, bore_data)
