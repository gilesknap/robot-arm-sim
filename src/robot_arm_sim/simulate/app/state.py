"""Shared state for the simulator UI."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from nicegui import ui

from robot_arm_sim.models.robot import URDFRobot

from .loaders import load_connection_points, load_flat_faces, load_mesh_centers


def _make_offsets(
    count: int, x: float, origins: list[list[float]] | None = None
) -> list[tuple[float, float, float]]:
    """Generate callout offsets for labels.

    For co-located joints (origin distance < 5mm between consecutive joints),
    apply alternating z-nudges of ±0.02 to prevent overlap.
    """
    offsets: list[tuple[float, float, float]] = []
    for i in range(count):
        z_nudge = 0.0
        if origins and i > 0:
            dist = (
                sum((origins[i][k] - origins[i - 1][k]) ** 2 for k in range(3)) ** 0.5
            )
            if dist < 0.005:  # co-located (< 5mm)
                z_nudge = 0.02 if i % 2 else -0.02
        offsets.append((x, 0.0, z_nudge))
    return offsets


class SimulatorState:
    """Holds all shared state for the simulator UI."""

    def __init__(self, robot: URDFRobot, robot_dir: Path) -> None:
        self.robot = robot
        self.robot_dir = robot_dir

        # Kinematic data
        self.chain = robot.get_kinematic_chain()
        self.joint_angles: dict[str, float] = {j.name: 0.0 for j in self.chain}
        self.mesh_objects: dict[str, Any] = {}
        self.callout_items: list[dict] = []

        # Toggle states
        self.labels_visible = {"value": False}
        self.frames_visible = {"value": False}
        self.bores_visible = {"value": False}
        self.transparent_mode = {"value": False}
        self.edit_bores_active = {"value": False}

        # Edit bores tracking: {link_name: {end: {centroid, normal, ...}}}
        self.bore_assignments: dict[str, dict[str, Any]] = {}
        self.bore_end_toggle = {"value": "Proximal"}
        self.keep_kinematics = {"value": True}
        self.bore_centering = {"value": "surface_bbox"}
        self.bore_centering_select: Any = None  # set by edit_bores UI
        self.bore_center_target: tuple[str, str] | None = None
        self.bore_dirty_links: set[str] = set()

        # Loaded analysis data
        self.mesh_centers = load_mesh_centers(robot, robot_dir)
        self.connection_points = load_connection_points(robot, robot_dir)
        self.flat_faces = load_flat_faces(robot, robot_dir)

        # Chain link ordering
        chain_link_names: list[str] = []
        seen: set[str] = set()
        for joint in self.chain:
            for lname in (joint.parent, joint.child):
                if lname not in seen:
                    chain_link_names.append(lname)
                    seen.add(lname)
        self.chain_link_names = chain_link_names
        self.visible_links: dict[str, bool] = dict.fromkeys(chain_link_names, True)

        # Callout offsets
        part_count = sum(1 for link in robot.links if link.mesh_path)
        joint_origins = [j.origin_xyz for j in self.chain]
        self.part_offsets = _make_offsets(part_count, -0.35)
        self.joint_offsets = _make_offsets(len(self.chain), 0.35, joint_origins)

        # Joint labels
        self.joint_labels = self._build_joint_labels()

        # UI widgets (populated during build)
        self.scene: Any = None
        self.sliders: dict[str, Any] = {}
        self.ik_sliders: dict[str, Any] = {}
        self.ik_labels: dict[str, Any] = {}
        self.link_checkboxes: dict[str, Any] = {}
        self.ee_readout_ref: list[ui.label | None] = [None, None]
        self.bore_status_label: Any = None
        self.edit_bores_btn: Any = None
        self.edit_bores_row: Any = None
        self.joint_panel: Any = None
        self.ik_panel: Any = None

        # Deferred handlers (set by builders)
        self.toggle_edit_bores = lambda: None
        self.reset_all = lambda: None

    def _build_joint_labels(self) -> dict[str, str]:
        joint_labels: dict[str, str] = {}
        for joint in self.chain:
            child_link = self.robot.get_link(joint.child)
            if child_link and child_link.mesh_path:
                part = Path(child_link.mesh_path).stem
                joint_labels[joint.name] = f"{part}"
            else:
                parent_link = self.robot.get_link(joint.parent)
                if parent_link and parent_link.mesh_path:
                    part = Path(parent_link.mesh_path).stem
                    joint_labels[joint.name] = f"{part}"
                else:
                    joint_labels[joint.name] = joint.name
        return joint_labels

    def update_scene_now(self) -> None:
        """Recompute FK and update all scene elements."""
        from .scene_update import update_scene

        update_scene(self)

    async def reload_urdf(self) -> None:
        """Save current state to sessionStorage and reload the page."""
        vl_json = json.dumps(self.visible_links)
        save_js = (
            "sessionStorage.setItem("
            "'reload_state', JSON.stringify({"
            f"visible_links: {vl_json},"
            "joints: {"
            + ",".join(f"'{k}': {v}" for k, v in self.joint_angles.items())
            + "},"
            "camera: (function() {"
            "  function f(v){"
            "    if(!v) return null;"
            "    if(v.component&&v.component.proxy){"
            "      var p=v.component.proxy;"
            "      if(p.renderer&&p.scene&&p.controls)"
            "        return p;}"
            "    if(v.children&&Array.isArray(v.children))"
            "      for(var c of v.children){"
            "        var r=f(c); if(r) return r;}"
            "    if(v.component&&v.component.subTree)"
            "      return f(v.component.subTree);"
            "    return null;}"
            "  var sc=f(document.getElementById("
            "    'app').__vue_app__"
            "    ._container._vnode);"
            "  if(!sc) return null;"
            "  var c=sc.camera, t=sc.controls;"
            "  return {"
            "    px:c.position.x,py:c.position.y,"
            "    pz:c.position.z,"
            "    tx:t.target.x,ty:t.target.y,"
            "    tz:t.target.z,"
            "    ux:c.up.x,uy:c.up.y,uz:c.up.z,"
            "    isOrtho:!!c.isOrthographicCamera,"
            "    left:c.left,right:c.right,"
            "    top:c.top,bottom:c.bottom"
            "  };"
            "})()}))"
        )
        await ui.run_javascript(save_js)
        ui.navigate.to(f"/{self.robot_dir.name}")
