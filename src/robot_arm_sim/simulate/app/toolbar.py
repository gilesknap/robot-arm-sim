"""Toolbar and visibility controls for the simulator."""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING

from nicegui import ui

from .js_snippets import SCREENSHOT_JS

if TYPE_CHECKING:
    from .state import SimulatorState

_CLUSTER_MODE = bool(
    os.environ.get("CLUSTER_MODE") or os.environ.get("KUBERNETES_SERVICE_HOST")
)


def _apply_toggle_style(btn, active: bool) -> None:
    """Apply active/inactive styling to a toggle button."""
    if active:
        btn.props("dense color=primary")
    else:
        btn.props("flat dense")
    btn.update()


def build_toolbar(state: SimulatorState) -> None:
    """Build the toolbar below the 3D viewport."""

    stop_dialog = None
    if not _CLUSTER_MODE:

        def shutdown():
            import asyncio
            import os
            import signal

            async def _delayed_kill():
                await asyncio.sleep(0.5)
                os.kill(os.getpid(), signal.SIGTERM)

            ui.run_javascript(
                "document.querySelector('#app').__vue_app__.unmount();"
                "document.body.innerHTML = '<div style=\""
                "display:flex;flex-direction:column;"
                "align-items:center;"
                "justify-content:center;height:100vh;"
                "gap:16px;font-family:sans-serif"
                '">'
                '<span style="font-size:64px">'
                "&#x1f44b;</span>"
                "<h1>Simulator stopped</h1>"
                '<p style="color:#888">'
                "Run the simulate command again"
                " to restart."
                "</p></div>';"
            )
            asyncio.get_event_loop().create_task(_delayed_kill())

        with ui.dialog() as stop_dialog, ui.card():
            ui.label("Are you sure you want to stop the simulator?")
            with ui.row().classes("w-full justify-end"):
                ui.button("Cancel", on_click=stop_dialog.close).props("flat")
                ui.button("Stop", on_click=shutdown).props("color=red")

    docs_url = "https://gilesknap.github.io/robot-arm-sim"
    story_url = f"{docs_url}/main/explanations/building-with-claude.html"

    with (
        ui.dialog() as about_dialog,
        ui.card().style("max-width: 480px; text-align: left"),
    ):
        ui.label("About Robot Arm Simulator").classes("text-h6")
        ui.label("Toolbar buttons").classes("text-subtitle2")
        _bdr = "border: 1px solid rgba(0,0,0,0.12);"
        _th = f"text-align: left; padding: 6px 8px; {_bdr}"
        _td = f"padding: 4px 8px; {_bdr}"
        _hdr_bg = "background: rgba(0,0,0,0.06);"
        rows = [
            ("Labels", "Toggle part/joint name callouts in the 3D view"),
            ("Frames", "Show coordinate-frame axes at each joint"),
            ("Transparent", "Make meshes semi-transparent"),
            ("Bores", "Highlight detected bore holes on each part"),
            ("Edit Bores", "Assign bore holes to proximal/distal ends"),
            ("Reload URDF", "Re-parse the URDF and refresh the scene"),
            ("Screenshot", "Download a PNG of the current 3D view"),
            ("Reset", "Zero all joint angles"),
        ]
        body = "".join(
            f"<tr><td style='{_td}'><b>{b}</b></td><td style='{_td}'>{d}</td></tr>"
            for b, d in rows
        )
        ui.html(
            f'<table style="border-collapse: collapse; width: 100%; {_bdr}">'
            f'<tr style="{_hdr_bg}">'
            f'<th style="{_th}">Button</th>'
            f'<th style="{_th}">Description</th>'
            f"</tr>{body}</table>"
        )
        with ui.row().style("gap: 16px"):
            ui.link("Documentation", docs_url, new_tab=True)
            ui.link("How it was built", story_url, new_tab=True)
        with ui.row().classes("w-full justify-end q-mt-sm"):
            ui.button("Close", on_click=about_dialog.close).props("flat")

    with ui.row().classes("q-pa-sm items-center").style("width: 100%; gap: 8px;"):
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

        if stop_dialog:
            sd = stop_dialog
            ui.button("Stop Simulator", on_click=sd.open).props(
                "color=red-7 flat dense"
            )

        ui.space()

        # About icon button
        ui.button(
            icon="info",
            on_click=about_dialog.open,
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
