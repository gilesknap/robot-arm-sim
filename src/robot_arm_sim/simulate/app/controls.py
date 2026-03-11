"""Right-side control panel — joint sliders, IK, EE readout, state restore."""

from __future__ import annotations

import json
import math
from typing import TYPE_CHECKING

import numpy as np
from nicegui import ui

from ..kinematics import forward_kinematics, matrix_to_position_euler

if TYPE_CHECKING:
    from .state import SimulatorState


def build_controls_panel(state: SimulatorState) -> None:
    """Build the right-side controls panel (joints, IK, EE readout)."""
    from ..ik_solver import build_ik_chain, solve_ik

    ik_chain = build_ik_chain(state.robot_dir / "robot.urdf")
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

    def reset_all():
        for jname in state.joint_angles:
            state.joint_angles[jname] = 0.0
        for s in state.sliders.values():
            s.value = 0
        _populate_ik_from_fk()
        state.update_scene_now()

    # Compact header row: title + mode radio + reset button
    with ui.row().classes("w-full items-center"):
        ui.label("Controls").classes("text-h6")
        ui.space()
        mode_radio = ui.radio(["Joint", "IK"], value="Joint").props("dense inline")
        ui.button("Reset", on_click=reset_all).props("flat dense")

    # --- Joint control panel ---
    state.joint_panel = ui.column().classes("w-full")
    with state.joint_panel:
        for joint in chain:
            if joint.joint_type not in ("revolute", "continuous"):
                continue

            lower_deg = math.degrees(joint.limit_lower)
            upper_deg = math.degrees(joint.limit_upper)
            display = state.joint_labels[joint.name]

            with ui.column().classes("w-full"):
                label = ui.label(f"{display}: 0.0\u00b0")

                def make_handler(jname, lbl, disp):
                    def on_change():
                        angle_deg = state.sliders[jname].value
                        state.joint_angles[jname] = math.radians(angle_deg)
                        lbl.text = f"{disp}: {angle_deg:.1f}\u00b0"
                        state.update_scene_now()

                    return on_change

                slider = ui.slider(
                    min=lower_deg,
                    max=upper_deg,
                    value=0,
                    step=0.5,
                    on_change=make_handler(joint.name, label, display),
                )
                state.sliders[joint.name] = slider

    # --- IK control panel ---
    state.ik_panel = ui.column().classes("w-full")
    state.ik_panel.set_visibility(False)

    def _solve_ik_from_sliders():
        target = np.array(
            [
                (state.ik_sliders["x"].value or 0) / 1000,
                (state.ik_sliders["y"].value or 0) / 1000,
                (state.ik_sliders["z"].value or 0) / 1000,
            ]
        )
        current = [0.0] + [state.joint_angles.get(j.name, 0.0) for j in chain]
        result = solve_ik(ik_chain, target, current)
        if result is None:
            return
        for i, jt in enumerate(chain):
            angle = result[i + 1]
            state.joint_angles[jt.name] = angle
            if jt.name in state.sliders:
                state.sliders[jt.name].value = math.degrees(angle)
        state.update_scene_now()

    ik_defs = [
        ("x", "X (mm)", -300, 300),
        ("y", "Y (mm)", -300, 300),
        ("z", "Z (mm)", 0, 500),
        ("rx", "Rx (\u00b0)", -180, 180),
        ("ry", "Ry (\u00b0)", -180, 180),
        ("rz", "Rz (\u00b0)", -180, 180),
    ]

    with state.ik_panel:
        for key, name, lo, hi in ik_defs:

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
        is_joint = e.value == "Joint"
        state.joint_panel.set_visibility(is_joint)
        state.ik_panel.set_visibility(not is_joint)
        if not is_joint:
            _populate_ik_from_fk()

    mode_radio.on_value_change(on_mode_change)

    # --- End-effector readout ---
    ui.separator()
    ui.label("End Effector").classes("text-caption")
    ee_readout = ui.label(
        "X: 0.0  Y: 0.0  Z: 0.0 mm\nRx: 0.0\u00b0  Ry: 0.0\u00b0  Rz: 0.0\u00b0"
    ).style("white-space: pre; font-family: monospace; font-size: 0.75rem;")

    state.ee_readout_ref[0] = ee_readout

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
            "sessionStorage.removeItem('reload_state');"
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
                    state.link_checkboxes[lname].value = vis

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
