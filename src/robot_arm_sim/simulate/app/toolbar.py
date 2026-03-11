"""Toolbar and visibility controls for the simulator."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from nicegui import ui

from .js_snippets import SCREENSHOT_JS

if TYPE_CHECKING:
    from .state import SimulatorState


def build_toolbar(state: SimulatorState) -> None:
    """Build the toolbar below the 3D viewport."""

    def toggle_labels():
        state.labels_visible["value"] = not state.labels_visible["value"]
        state.update_scene_now()
        label_btn.text = (
            "Hide Labels" if state.labels_visible["value"] else "Show Labels"
        )

    def shutdown():
        import os
        import signal

        os.kill(os.getpid(), signal.SIGTERM)

    with ui.row().classes("q-pa-sm").style("width: 900px; gap: 8px;"):
        ui.button(
            "About",
            on_click=lambda: ui.navigate.to(
                "https://gilesknap.github.io/"
                "robot-arm-sim/main/"
                "explanations/building-with-claude.html",
                new_tab=True,
            ),
        ).props("color=green-7 flat dense")

        ui.space()

        label_btn = ui.button("Show Labels", on_click=toggle_labels).props("flat dense")

        def toggle_frames():
            state.frames_visible["value"] = not state.frames_visible["value"]
            show = state.frames_visible["value"]
            frames_btn.text = "Hide Frames" if show else "Show Frames"
            ui.run_javascript(f"window.__setAxesVisible({str(show).lower()})")
            if show:
                state.update_scene_now()

        frames_btn = ui.button("Show Frames", on_click=toggle_frames).props(
            "flat dense"
        )

        def toggle_bores():
            state.bores_visible["value"] = not state.bores_visible["value"]
            show = state.bores_visible["value"]
            bores_btn.text = "Hide Bores" if show else "Show Bores"
            ui.run_javascript(f"window.__setBoresVisible({str(show).lower()})")
            if show:
                state.update_scene_now()

        bores_btn = ui.button("Show Bores", on_click=toggle_bores).props("flat dense")

        def toggle_transparent():
            state.transparent_mode["value"] = not state.transparent_mode["value"]
            on = state.transparent_mode["value"]
            trans_btn.text = "Opaque" if on else "Transparent"
            op = "0.25" if on else "1.0"
            ui.run_javascript(f"window.__setMeshTransparency({op})")

        trans_btn = ui.button("Transparent", on_click=toggle_transparent).props(
            "flat dense"
        )

        ui.button(
            "Screenshot",
            on_click=lambda: ui.run_javascript(SCREENSHOT_JS),
        ).props("flat dense")

        # Edit Bores button — handler set by build_edit_bores
        state.edit_bores_btn = ui.button(
            "Edit Bores",
            on_click=lambda: state.toggle_edit_bores(),
        ).props("color=orange-7 flat dense")

        ui.button(
            "Reload URDF",
            on_click=state.reload_urdf,
        ).props("color=blue-7 flat dense")

        ui.button("Stop Simulator", on_click=shutdown).props("color=red-7 flat dense")


def build_visibility_panel(state: SimulatorState) -> None:
    """Build per-part visibility checkboxes (collapsible)."""
    with (
        ui.expansion("Visible Parts", value=False).props("dense").style("width: 900px")
    ):
        with ui.row().classes("q-pa-xs").style("gap: 12px"):
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

                cb = ui.checkbox(
                    display,
                    value=True,
                    on_change=make_vis_handler(lname),
                ).props("dense")
                state.link_checkboxes[lname] = cb
