"""Edit Bores mode — interactive bore point assignment."""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING

import yaml
from nicegui import ui

from .loaders import quantize_axis

if TYPE_CHECKING:
    from .state import SimulatorState


def build_edit_bores(state: SimulatorState) -> None:
    """Build the Edit Bores mode UI and handlers."""
    state.edit_bores_row = (
        ui.row().classes("q-pa-sm").style("width: 900px; gap: 8px; display: none;")
    )
    with state.edit_bores_row:
        bore_toggle = ui.toggle(["Proximal", "Distal"], value="Proximal").props("dense")

        def _on_bore_toggle(e):
            state.bore_end_toggle["value"] = e.value

        bore_toggle.on_value_change(_on_bore_toggle)

        state.bore_status_label = ui.label("Click a face marker to assign").style(
            "font-size: 0.85rem; color: #666;"
        )

        ui.space()

        async def _save_and_rebuild():
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
                faces_list = state.flat_faces.get(link_name, [])
                new_cps = []
                for face_idx, end in assignments.items():
                    ff = faces_list[face_idx]
                    radius = math.sqrt(ff["area_mm2"] / math.pi)
                    new_cps.append(
                        {
                            "end": end,
                            "position": [
                                round(ff["centroid"][0], 3),
                                round(ff["centroid"][1], 3),
                                round(ff["centroid"][2], 3),
                            ],
                            "axis": quantize_axis(ff["normal"]),
                            "radius_mm": round(radius, 1),
                            "method": "manual",
                        }
                    )
                if new_cps:
                    data["connection_points"] = new_cps
                    with open(yaml_path, "w") as f:
                        yaml.dump(data, f, default_flow_style=False)

            # Propagate bore axes to chain.yaml
            chain_file = state.robot_dir / "chain.yaml"
            if chain_file.exists():
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

    def _toggle_edit_bores():
        state.edit_bores_active["value"] = not state.edit_bores_active["value"]
        active = state.edit_bores_active["value"]
        state.edit_bores_btn.text = "Exit Edit" if active else "Edit Bores"
        state.edit_bores_row.style(
            "width: 900px; gap: 8px; display: flex;"
            if active
            else "width: 900px; gap: 8px; display: none;"
        )
        ui.run_javascript(f"window.__setFaceMarkersVisible({str(active).lower()})")
        if active:
            ui.run_javascript("window.__setMeshTransparency(0.25)")
            state.bore_status_label.text = "Click a face marker to assign"
            state.update_scene_now()
        else:
            if not state.transparent_mode["value"]:
                ui.run_javascript("window.__setMeshTransparency(1.0)")

    state.toggle_edit_bores = _toggle_edit_bores

    # Polling timer for face marker clicks
    async def _poll_face_click():
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
            _handle_face_click(state, result)

    ui.timer(0.2, _poll_face_click)


def _handle_face_click(state: SimulatorState, name: str) -> None:
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

    if link_name not in state.bore_assignments:
        state.bore_assignments[link_name] = {}

    assigns = state.bore_assignments[link_name]

    # Check if opposite end is at this face — swap
    opposite = "distal" if end == "proximal" else "proximal"
    if assigns.get(face_idx) == opposite:
        old_face = None
        for fi, e in assigns.items():
            if e == end:
                old_face = fi
                break
        if old_face is not None:
            assigns[old_face] = opposite
            old_id = f"face_{link_name}_{old_face}"
            color = "0xcc0000" if opposite == "distal" else "0x00cc00"
            ui.run_javascript(f"window.__setFaceMarkerColor('{old_id}', {color})")

    assigns[face_idx] = end

    # Recolour all markers for this link
    faces_list = state.flat_faces.get(link_name, [])
    for i in range(len(faces_list)):
        fid = f"face_{link_name}_{i}"
        if i in assigns:
            color = "0x00cc00" if assigns[i] == "proximal" else "0xcc0000"
        else:
            color = "0xffcc00"
        ui.run_javascript(f"window.__setFaceMarkerColor('{fid}', {color})")

    part_link = state.robot.get_link(link_name)
    part_name = (
        Path(part_link.mesh_path).stem
        if part_link and part_link.mesh_path
        else link_name
    )
    state.bore_status_label.text = f"Set {part_name} face {face_idx} as {end}"
