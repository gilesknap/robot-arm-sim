"""Toolbar and visibility controls for the simulator."""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING

from nicegui import ui

from robot_arm_sim import __version__

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
        ui.label(f"Version {__version__}").style("color: #888; font-size: 0.8rem;")
        ui.label(
            "Automatically generates interactive robot simulations from"
            " STL mesh files. Powered by Claude Code and Python, it infers"
            " part connections, derives the kinematics, and builds a"
            " fully articulated 3D simulator."
        ).style("color: #555; font-size: 0.9rem; line-height: 1.4;")
        ui.label("Toolbar buttons").classes("text-subtitle2")
        _bdr = "border: 1px solid rgba(0,0,0,0.12);"
        _th = f"text-align: left; padding: 6px 8px; {_bdr}"
        _td = f"padding: 4px 8px; {_bdr}"
        _hdr_bg = "background: rgba(0,0,0,0.06);"
        rows = [
            ("Labels", "Toggle part/joint name callouts in the 3D view"),
            ("Frames", "Show coordinate-frame axes at each joint"),
            ("Transparent", "Make meshes semi-transparent"),
            ("Connections", "Highlight detected connection points on each part"),
            (
                "Edit Connections",
                "Assign connection points to proximal/distal ends with centering",
            ),
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
        def toggle_connections():
            state.connections_visible["value"] = not state.connections_visible["value"]
            show = state.connections_visible["value"]
            ui.run_javascript(f"window.__setConnectionsVisible({str(show).lower()})")
            if show:
                state.update_scene_now()
            _apply_toggle_style(connections_btn, show)

        connections_btn = ui.button("Connections", on_click=toggle_connections).props(
            "flat dense"
        )

        state.edit_connections_btn = ui.button(
            "Edit Connections",
            on_click=lambda: state.toggle_edit_connections(),
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
            "Reset Joints",
            on_click=lambda: state.reset_all(),
        ).props("flat dense color=orange-7")

        async def _reset_view() -> None:
            # Show all parts
            for lname in state.visible_links:
                state.visible_links[lname] = True
                if lname in state.link_checkboxes:
                    state.link_checkboxes[lname].selected = True
            state.update_scene_now()
            # Switch to ortho RIGHT view + fit
            await ui.run_javascript(
                "(() => {"
                "  const appEl = document.getElementById('app');"
                "  if (!appEl || !appEl.__vue_app__) return;"
                "  let sc = null;"
                "  function walk(n, d) {"
                "    if (d > 30 || sc) return;"
                "    if (n.component) {"
                "      const p = n.component.proxy;"
                "      if (p && p.renderer && p.scene && p.camera)"
                "        { sc = p; return; }"
                "      if (n.component.subTree)"
                "        walk(n.component.subTree, d+1);"
                "    }"
                "    if (Array.isArray(n.children))"
                "      n.children.forEach("
                "        c => c && typeof c === 'object'"
                "          && walk(c, d+1));"
                "  }"
                "  walk(appEl.__vue_app__._container._vnode, 0);"
                "  if (!sc) return;"
                "  const tb = document.getElementById("
                "    'viewcube-proj-toggle');"
                "  if (sc.camera.isPerspectiveCamera && tb)"
                "    tb.click();"
                "  setTimeout(() => {"
                "    const cam = sc.camera;"
                "    const ctrl = sc.controls;"
                "    const tx=ctrl.target.x,"
                "      ty=ctrl.target.y, tz=ctrl.target.z;"
                "    const d = cam.position.distanceTo(ctrl.target);"
                "    cam.position.set(tx, ty - d, tz);"
                "    cam.up.set(0, 0, 1);"
                "    ctrl.update();"
                "    const fb = document.getElementById("
                "      'viewcube-fit-btn');"
                "    if (fb) fb.click();"
                "  }, 50);"
                "})();",
                timeout=2.0,
            )

        ui.button(
            "Reset View",
            on_click=_reset_view,
        ).props("flat dense color=orange-7")

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
        state.on_visibility_changed()

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
                    # Skip links with no mesh (e.g. link_5 intermediate frame)
                    continue

                def make_vis_handler(ln):
                    def on_change(e):
                        state.visible_links[ln] = e.value
                        state.update_scene_now()
                        state.on_visibility_changed()

                    return on_change

                chip = ui.chip(
                    display,
                    selectable=True,
                    selected=True,
                    on_selection_change=make_vis_handler(lname),
                ).props("dense")
                state.link_checkboxes[lname] = chip
