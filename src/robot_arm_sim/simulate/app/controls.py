"""Right-side control panel — joint sliders, IK, EE readout, state restore."""

from __future__ import annotations

import json
import math
from typing import TYPE_CHECKING

import numpy as np
from nicegui import ui

from ..ik_solver import solve_ik
from ..kinematics import forward_kinematics, matrix_to_position_euler, rpy_to_matrix

if TYPE_CHECKING:
    from .state import SimulatorState


def build_controls_panel(state: SimulatorState) -> None:
    """Build the right-side controls panel (joints, IK, EE readout)."""
    chain = state.chain

    def _populate_ik_from_fk():
        """Set IK slider values from current FK pose."""
        ee_chain = state.robot.get_kinematic_chain()
        if not ee_chain:
            return
        tfs = forward_kinematics(state.robot, state.joint_angles)
        last = ee_chain[-1].child
        ee_tf = tfs.get(last, np.eye(4))
        p, eu = matrix_to_position_euler(ee_tf)
        state.ik_sliders["x"].value = round(p[0] * 1000)
        state.ik_sliders["y"].value = round(p[1] * 1000)
        state.ik_sliders["z"].value = round(p[2] * 1000)
        state.ik_sliders["rx"].value = round(math.degrees(eu[0]))
        state.ik_sliders["ry"].value = round(math.degrees(eu[1]))
        state.ik_sliders["rz"].value = round(math.degrees(eu[2]))

    state.reset_all = lambda: None  # placeholder, set below

    def reset_all():
        for jname in state.joint_angles:
            state.joint_angles[jname] = 0.0
        for s in state.sliders.values():
            s.value = 0
        _populate_ik_from_fk()
        state.update_scene_now()

    state.reset_all = reset_all

    # Compact header row: title + mode radio
    with ui.row().classes("w-full items-center"):
        ui.label("Controls").classes("text-h6")
        ui.space()
        mode_radio = ui.radio(["FK", "IK"], value="FK").props("dense inline")

    # --- Joint control panel ---
    state.joint_panel = ui.column().classes("w-full").style("gap: 4px")
    with state.joint_panel:
        for joint in chain:
            if joint.joint_type not in ("revolute", "continuous"):
                continue

            lower_deg = math.degrees(joint.limit_lower)
            upper_deg = math.degrees(joint.limit_upper)
            display = state.joint_labels[joint.name]

            with ui.column().classes("w-full").style("gap: 0"):
                with ui.row().classes("w-full items-center no-wrap"):
                    ui.label(display).style("font-size: 0.8rem;")
                    ui.space()
                    angle_label = ui.label("0.0°").style("font-size: 0.8rem;")

                def make_handler(jname, albl):
                    def on_change():
                        angle_deg = state.sliders[jname].value
                        state.joint_angles[jname] = math.radians(angle_deg)
                        albl.text = f"{angle_deg:.1f}°"
                        state.update_scene_now()

                    return on_change

                slider = ui.slider(
                    min=lower_deg,
                    max=upper_deg,
                    value=0,
                    step=0.5,
                    on_change=make_handler(joint.name, angle_label),
                )
                state.sliders[joint.name] = slider

    # --- Reset buttons ---
    with ui.row().classes("w-full q-mt-sm").style("gap: 8px"):

        async def _reset_view() -> None:
            for lname in state.visible_links:
                state.visible_links[lname] = True
                if lname in state.link_checkboxes:
                    state.link_checkboxes[lname].selected = True
            state.update_scene_now()
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
            "Reset Joints",
            on_click=lambda: state.reset_all(),
        ).props("flat dense color=orange-7")

        ui.button(
            "Reset View",
            on_click=_reset_view,
        ).props("flat dense color=orange-7")

    # --- IK control panel ---
    state.ik_panel = ui.column().classes("w-full")
    state.ik_panel.set_visibility(False)

    def _solve_ik_from_sliders():
        pos = np.array(
            [
                (state.ik_sliders["x"].value or 0) / 1000,
                (state.ik_sliders["y"].value or 0) / 1000,
                (state.ik_sliders["z"].value or 0) / 1000,
            ]
        )
        rx = math.radians(state.ik_sliders["rx"].value or 0)
        ry = math.radians(state.ik_sliders["ry"].value or 0)
        rz = math.radians(state.ik_sliders["rz"].value or 0)
        target_matrix = rpy_to_matrix([rx, ry, rz])
        target_matrix[:3, 3] = pos
        result = solve_ik(state.robot, target_matrix, state.joint_angles)
        if result is None:
            return
        state.joint_angles.update(result)
        for jname, angle in result.items():
            if jname in state.sliders:
                state.sliders[jname].value = math.degrees(angle)
        state.update_scene_now()

    trans_defs = [
        ("x", "X (mm)", -300, 300),
        ("y", "Y (mm)", -300, 300),
        ("z", "Z (mm)", 0, 500),
    ]
    rot_defs = [
        ("rx", "Rx (°)", -180, 180),
        ("ry", "Ry (°)", -180, 180),
        ("rz", "Rz (°)", -180, 180),
    ]

    with state.ik_panel:
        with ui.row().classes("w-full").style("gap: 16px"):
            # Translation column
            with ui.column().style("flex: 1"):
                ui.label("Translation (mm)").classes("text-caption")
                for key, name, lo, hi in trans_defs:

                    def make_ik_handler(k, nm):
                        def on_change():
                            v = state.ik_sliders[k].value
                            state.ik_labels[k].text = f"{nm}: {v}"
                            _solve_ik_from_sliders()

                        return on_change

                    with ui.column().classes("w-full"):
                        state.ik_labels[key] = ui.label(f"{name}: 0")
                        state.ik_sliders[key] = ui.slider(
                            min=lo,
                            max=hi,
                            value=0,
                            step=1,
                            on_change=make_ik_handler(key, name),
                        )

            # Rotation column
            with ui.column().style("flex: 1"):
                ui.label("Rotation (°)").classes("text-caption")
                for key, name, lo, hi in rot_defs:

                    def make_ik_handler(k, nm):
                        def on_change():
                            v = state.ik_sliders[k].value
                            state.ik_labels[k].text = f"{nm}: {v}"
                            _solve_ik_from_sliders()

                        return on_change

                    with ui.column().classes("w-full"):
                        state.ik_labels[key] = ui.label(f"{name}: 0")
                        state.ik_sliders[key] = ui.slider(
                            min=lo,
                            max=hi,
                            value=0,
                            step=1,
                            on_change=make_ik_handler(key, name),
                        )

    # Mode toggle handler
    def on_mode_change(e):
        is_fk = e.value == "FK"
        state.joint_panel.set_visibility(is_fk)
        state.ik_panel.set_visibility(not is_fk)
        if not is_fk:
            _populate_ik_from_fk()

    mode_radio.on_value_change(on_mode_change)

    # --- End-effector readout ---
    mono = "font-family: monospace; font-size: 0.75rem;"
    with ui.card().props("flat bordered").classes("w-full q-pa-sm"):
        ui.label("End Effector").classes("text-caption")
        with ui.row().classes("w-full").style("gap: 16px"):
            ee_trans_label = ui.label("X: 0.0\nY: 0.0\nZ: 0.0 mm").style(
                f"white-space: pre; {mono}"
            )
            ee_rot_label = ui.label("Rx: 0.0°\nRy: 0.0°\nRz: 0.0°").style(
                f"white-space: pre; {mono}"
            )

    state.ee_readout_ref = [ee_trans_label, ee_rot_label]

    # Initial EE readout update
    from .scene_update import update_scene

    update_scene(state)

    # --- Restore state from sessionStorage (after URDF reload) ---
    _setup_state_restore(state)


def _setup_state_restore(state: SimulatorState) -> None:
    """Set up timer to restore UI state after page reload."""

    async def _restore_state():
        raw = await ui.run_javascript(
            "var s = sessionStorage.getItem('reload_state');"
            "if(s) sessionStorage.removeItem('reload_state');"
            "else s = sessionStorage.getItem('sim_state');"
            "s"
        )
        if not raw:
            return

        try:
            saved = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return

        # Restore per-part visibility
        vl_state = saved.get("visible_links", {})
        for lname, vis in vl_state.items():
            if lname in state.visible_links:
                state.visible_links[lname] = vis
                if lname in state.link_checkboxes:
                    state.link_checkboxes[lname].selected = vis

        # Restore joint angles
        joints_state = saved.get("joints", {})
        for jname, angle_rad in joints_state.items():
            if jname in state.joint_angles and jname in state.sliders:
                state.joint_angles[jname] = angle_rad
                state.sliders[jname].value = math.degrees(angle_rad)
        state.update_scene_now()

        # Restore camera (after scene is ready)
        cam = saved.get("camera")
        if cam:
            restore_cam_js = (
                "function f(v){"
                "if(!v)return null;"
                "if(v.component&&v.component.proxy){"
                "var p=v.component.proxy;"
                "if(p.renderer&&p.scene&&p.controls)"
                "return p;}"
                "if(v.children&&Array.isArray(v.children))"
                "for(var c of v.children){"
                "var r=f(c);if(r)return r;}"
                "if(v.component&&v.component.subTree)"
                "return f(v.component.subTree);"
                "return null;}"
                "var sc=f(document.getElementById("
                "'app').__vue_app__._container._vnode);"
                "if(sc){"
                "var THREE=window.__THREE_REF;"
            )
            if cam.get("isOrtho") and cam.get("left") is not None:
                restore_cam_js += (
                    "var canvas=sc.renderer.domElement;"
                    "var aspect=canvas.width/canvas.height;"
                    "var oc=new THREE.OrthographicCamera("
                    f"{cam['left']},{cam['right']},"
                    f"{cam['top']},{cam['bottom']},"
                    "0.001,100);"
                    f"oc.position.set("
                    f"{cam['px']},{cam['py']},{cam['pz']});"
                    f"oc.up.set("
                    f"{cam['ux']},{cam['uy']},{cam['uz']});"
                    "oc.updateProjectionMatrix();"
                    "sc.camera=oc;"
                    "sc.controls.object=oc;"
                )
            else:
                restore_cam_js += (
                    f"sc.camera.position.set("
                    f"{cam['px']},{cam['py']},{cam['pz']});"
                    f"sc.camera.up.set("
                    f"{cam['ux']},{cam['uy']},{cam['uz']});"
                )
            restore_cam_js += (
                f"sc.controls.target.set("
                f"{cam['tx']},{cam['ty']},{cam['tz']});"
                "sc.controls.update();"
                "}"
            )
            await ui.run_javascript(restore_cam_js)

    ui.timer(3.0, _restore_state, once=True)
