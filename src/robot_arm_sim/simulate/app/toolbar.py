"""Toolbar and visibility controls for the simulator."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from nicegui import ui

from .js_snippets import SCREENSHOT_JS

if TYPE_CHECKING:
    from .state import SimulatorState


def _apply_toggle_style(btn, active: bool) -> None:
    """Apply active/inactive styling to a toggle button."""
    if active:
        btn.props("dense color=primary")
    else:
        btn.props("flat dense")
    btn.update()


def build_toolbar(state: SimulatorState) -> None:
    """Build the toolbar below the 3D viewport."""

    def shutdown():
        import os
        import signal

        os.kill(os.getpid(), signal.SIGTERM)

    with ui.dialog() as stop_dialog, ui.card():
        ui.label("Are you sure you want to stop the simulator?")
        with ui.row().classes("w-full justify-end"):
            ui.button("Cancel", on_click=stop_dialog.close).props("flat")
            ui.button("Stop", on_click=shutdown).props("color=red")

    with ui.row().classes("q-pa-sm items-center").style("width: 900px; gap: 8px;"):
        # --- View group ---
        def toggle_labels():
            state.labels_visible["value"] = not state.labels_visible["value"]
            state.update_scene_now()
            _apply_toggle_style(label_btn, state.labels_visible["value"])

        label_btn = ui.button("Labels", on_click=toggle_labels).props("flat dense")

        def toggle_frames():
            state.frames_visible["value"] = not state.frames_visible["value"]
            show = state.frames_visible["value"]
            ui.run_javascript(f"window.__setAxesVisible({str(show).lower()})")
            if show:
                state.update_scene_now()
            _apply_toggle_style(frames_btn, show)

        frames_btn = ui.button("Frames", on_click=toggle_frames).props("flat dense")

        def toggle_transparent():
            state.transparent_mode["value"] = not state.transparent_mode["value"]
            on = state.transparent_mode["value"]
            op = "0.25" if on else "1.0"
            ui.run_javascript(f"window.__setMeshTransparency({op})")
            _apply_toggle_style(trans_btn, on)

        trans_btn = ui.button("Transparent", on_click=toggle_transparent).props(
            "flat dense"
        )

        # --- Separator ---
        ui.separator().props("vertical")

        # --- Edit group ---
        def toggle_bores():
            state.bores_visible["value"] = not state.bores_visible["value"]
            show = state.bores_visible["value"]
            ui.run_javascript(f"window.__setBoresVisible({str(show).lower()})")
            if show:
                state.update_scene_now()
            _apply_toggle_style(bores_btn, show)

        bores_btn = ui.button("Bores", on_click=toggle_bores).props("flat dense")

        state.edit_bores_btn = ui.button(
            "Edit Bores",
            on_click=lambda: state.toggle_edit_bores(),
        ).props("color=orange-7 flat dense")

        ui.button(
            "Reload URDF",
            on_click=state.reload_urdf,
        ).props("color=blue-7 flat dense")

        ui.button(
            "Screenshot",
            on_click=lambda: ui.run_javascript(SCREENSHOT_JS),
        ).props("flat dense")

        # --- Separator ---
        ui.separator().props("vertical")

        # --- Session group ---
        ui.button(
            "Reset",
            on_click=lambda: state.reset_all(),
        ).props("dense color=orange-7")

        ui.button("Stop Simulator", on_click=stop_dialog.open).props(
            "color=red-7 flat dense"
        )

        ui.space()

        # About icon button
        ui.button(
            icon="info",
            on_click=lambda: ui.navigate.to(
                "https://gilesknap.github.io/"
                "robot-arm-sim/main/"
                "explanations/building-with-claude.html",
                new_tab=True,
            ),
        ).props("flat dense round color=green-7").tooltip("About")


def build_visibility_section(state: SimulatorState) -> None:
    """Build per-part visibility chips (collapsible, for right sidebar)."""

    def toggle_all(e):
        """Toggle all parts on/off."""
        val = e.value
        for lname in state.chain_link_names:
            state.visible_links[lname] = val
            if lname in state.link_checkboxes:
                state.link_checkboxes[lname].selected = val
        state.update_scene_now()

    with ui.expansion("Visible Parts", value=False).props("dense").classes("w-full"):
        with ui.row().classes("q-pa-xs").style("gap: 8px; flex-wrap: wrap"):
            all_chip = ui.chip(  # noqa: F841
                "All",
                selectable=True,
                selected=True,
                on_selection_change=toggle_all,
            ).props("dense")

            for lname in state.chain_link_names:
                lnk = state.robot.get_link(lname)
                if lnk and lnk.mesh_path:
                    display = Path(lnk.mesh_path).stem
                else:
                    display = lname

                def make_vis_handler(ln):
                    def on_change(e):
                        state.visible_links[ln] = e.value
                        state.update_scene_now()

                    return on_change

                chip = ui.chip(
                    display,
                    selectable=True,
                    selected=True,
                    on_selection_change=make_vis_handler(lname),
                ).props("dense")
                state.link_checkboxes[lname] = chip
