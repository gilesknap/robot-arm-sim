"""Scene update logic — recomputes FK and updates all 3D elements."""

from __future__ import annotations

import json
import math
from typing import TYPE_CHECKING

import numpy as np
from nicegui import ui

from ..kinematics import (
    axis_to_quaternion,
    forward_kinematics,
    matrix_to_position_euler,
    matrix_to_quaternion,
    rpy_to_matrix,
    translation_matrix,
)

if TYPE_CHECKING:
    from .state import SimulatorState

# Position to hide labels (far off screen)
HIDDEN_POS = (0, 0, -100)


def update_scene(state: SimulatorState) -> None:
    """Recompute FK and update mesh transforms."""
    vis_set: set[str] | None = None
    if state.visible_links is not None:
        vis_set = {n for n, v in state.visible_links.items() if v}

    transforms = forward_kinematics(state.robot, state.joint_angles)

    # Sync joint angles to JS for beforeunload persistence
    angles_json = json.dumps(state.joint_angles)
    ui.run_javascript(f"window.__simJointAngles={angles_json}")

    joint_positions: dict[str, tuple[float, ...]] = {}
    link_center_positions: dict[str, tuple[float, ...]] = {}
    visual_transforms: dict[str, np.ndarray] = {}

    for link_name, tf in transforms.items():
        link_pos, _ = matrix_to_position_euler(tf)

        for joint in state.robot.joints:
            if joint.child == link_name:
                joint_positions[joint.name] = tuple(link_pos)
                break

        obj = state.mesh_objects.get(link_name)
        if obj is None:
            continue

        if vis_set is not None and link_name not in vis_set:
            obj.move(*HIDDEN_POS)
            continue

        link = state.robot.get_link(link_name)
        if link:
            visual_tf = translation_matrix(link.origin_xyz) @ rpy_to_matrix(
                link.origin_rpy
            )
            tf_visual = tf @ visual_tf
        else:
            tf_visual = tf

        # Apply extra rotation from edit mode
        if (
            state.edit_connections_active["value"]
            and link_name in state.part_visual_rpys
        ):
            extra_rpy = state.part_visual_rpys[link_name]
            if any(abs(v) > 1e-6 for v in extra_rpy):
                tf_visual = tf_visual @ rpy_to_matrix(extra_rpy)

        visual_transforms[link_name] = tf_visual
        pos, euler = matrix_to_position_euler(tf_visual)
        obj.move(pos[0], pos[1], pos[2])
        obj.rotate(euler[0], euler[1], euler[2])

        if state.mesh_centers and link_name in state.mesh_centers:
            center_stl = state.mesh_centers[link_name] * 0.001
            center_homo = np.array([center_stl[0], center_stl[1], center_stl[2], 1.0])
            center_world = tf_visual @ center_homo
            link_center_positions[link_name] = (
                center_world[0],
                center_world[1],
                center_world[2],
            )
        else:
            link_center_positions[link_name] = tuple(link_pos)

    # Update callout positions
    _update_callouts(state, joint_positions, link_center_positions)

    # End-effector readout
    _update_ee_readout(state, transforms)

    # Coordinate frames
    _update_frames(state, transforms)

    # Connection markers
    _update_connections(state, visual_transforms, vis_set)


def _update_callouts(
    state: SimulatorState,
    joint_positions: dict[str, tuple[float, ...]],
    link_center_positions: dict[str, tuple[float, ...]],
) -> None:
    show = state.labels_visible and state.labels_visible.get("value", False)
    ui.run_javascript(
        f"window.__setLabelsVisible && window.__setLabelsVisible({str(show).lower()})"
    )
    if not show:
        return

    anchors: dict[str, list[float]] = {}
    for item in state.label_metadata:
        if item["is_joint"]:
            pos = joint_positions.get(item["id"])
        else:
            pos = link_center_positions.get(item["id"])
        if pos:
            anchors[item["id"]] = list(pos)

    data = json.dumps(anchors)
    ui.run_javascript(
        f"window.__updateLabelAnchors && window.__updateLabelAnchors({data})"
    )


def _update_ee_readout(
    state: SimulatorState, transforms: dict[str, np.ndarray]
) -> None:
    trans_label = state.ee_readout_ref[0]
    rot_label = state.ee_readout_ref[1]
    if trans_label is None or rot_label is None:
        return
    chain = state.robot.get_kinematic_chain()
    if not chain:
        return
    last_link = chain[-1].child
    ee_tf = transforms.get(last_link, np.eye(4))
    ee_pos, ee_euler = matrix_to_position_euler(ee_tf)
    x_mm = ee_pos[0] * 1000
    y_mm = ee_pos[1] * 1000
    z_mm = ee_pos[2] * 1000
    rx_deg = math.degrees(ee_euler[0])
    ry_deg = math.degrees(ee_euler[1])
    rz_deg = math.degrees(ee_euler[2])
    trans_label.text = f"X: {x_mm:.1f}\nY: {y_mm:.1f}\nZ: {z_mm:.1f} mm"
    rot_label.text = (
        f"Rx: {rx_deg:.1f}\u00b0\nRy: {ry_deg:.1f}\u00b0\nRz: {rz_deg:.1f}\u00b0"
    )


def _update_frames(state: SimulatorState, transforms: dict[str, np.ndarray]) -> None:
    if not (state.frames_visible and state.frames_visible.get("value", False)):
        return
    frame_data = {}
    for jt in state.robot.get_kinematic_chain():
        tf = transforms.get(jt.child)
        if tf is None:
            continue
        pos_j = [float(tf[0, 3]), float(tf[1, 3]), float(tf[2, 3])]
        q = matrix_to_quaternion(tf[:3, :3])
        frame_data[jt.name] = {"p": pos_j, "q": q}
    js_data = json.dumps(frame_data)
    ui.run_javascript(f"window.__updateAxesPoses({js_data})")


def _update_connections(
    state: SimulatorState,
    visual_transforms: dict[str, np.ndarray],
    vis_set: set[str] | None,
) -> None:
    show_connections = state.connections_visible and state.connections_visible.get(
        "value", False
    )
    if not (state.connection_points and show_connections):
        return
    connection_poses: dict[str, list[float]] = {}
    for link_name, cps in state.connection_points.items():
        tf_vis = visual_transforms.get(link_name)
        is_vis = vis_set is None or link_name in vis_set
        for i, cp in enumerate(cps):
            bid = f"{link_name}_{cp['end']}_{i}"
            if tf_vis is not None and is_vis:
                cp_m = cp["position"] * 0.001
                cp_homo = np.array([cp_m[0], cp_m[1], cp_m[2], 1.0])
                wp = tf_vis @ cp_homo
                world_axis = tf_vis[:3, :3] @ np.asarray(cp["axis"], dtype=float)
                q = axis_to_quaternion(world_axis)
                connection_poses[bid] = [
                    float(wp[0]),
                    float(wp[1]),
                    float(wp[2]),
                    1.0,
                    q[0],
                    q[1],
                    q[2],
                    q[3],
                ]
            else:
                connection_poses[bid] = [0.0, 0.0, -100.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    ui.run_javascript(f"window.__updateConnectionPoses({json.dumps(connection_poses)})")
