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

    # Bore markers
    _update_bores(state, visual_transforms, vis_set)

    # Face markers (Edit Bores mode)
    _update_face_markers(state, visual_transforms, vis_set)


def _update_callouts(
    state: SimulatorState,
    joint_positions: dict[str, tuple[float, ...]],
    link_center_positions: dict[str, tuple[float, ...]],
) -> None:
    if not state.callout_items:
        return
    show = state.labels_visible and state.labels_visible.get("value", False)
    for item in state.callout_items:
        if not show:
            item["text"].visible(False)
            item["line"].visible(False)
            item["text"].move(*HIDDEN_POS)
            item["line"].move(*HIDDEN_POS)
            continue

        name = item["name"]
        ox, oy, oz = item["offset"]

        if item["is_joint"]:
            anchor = joint_positions.get(name)
        else:
            anchor = link_center_positions.get(name)

        if anchor is None:
            continue

        ax, ay, az = anchor
        item["text"].visible(True)
        item["line"].visible(True)
        item["text"].move(ax + ox, ay + oy, az + oz)
        item["line"].move(ax, ay, az)


def _update_ee_readout(
    state: SimulatorState, transforms: dict[str, np.ndarray]
) -> None:
    ee_label = state.ee_readout_ref[0]
    if ee_label is None:
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
    ee_label.text = (
        f"X: {x_mm:.1f}  Y: {y_mm:.1f}"
        f"  Z: {z_mm:.1f} mm\n"
        f"Rx: {rx_deg:.1f}\u00b0  Ry: {ry_deg:.1f}\u00b0"
        f"  Rz: {rz_deg:.1f}\u00b0"
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


def _update_bores(
    state: SimulatorState,
    visual_transforms: dict[str, np.ndarray],
    vis_set: set[str] | None,
) -> None:
    show_bores = state.bores_visible and state.bores_visible.get("value", False)
    if not (state.connection_points and show_bores):
        return
    bore_poses: dict[str, list[float]] = {}
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
                bore_poses[bid] = [
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
                bore_poses[bid] = [0.0, 0.0, -100.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    ui.run_javascript(f"window.__updateBorePoses({json.dumps(bore_poses)})")


def _update_face_markers(
    state: SimulatorState,
    visual_transforms: dict[str, np.ndarray],
    vis_set: set[str] | None,
) -> None:
    show_faces = state.edit_bores_active and state.edit_bores_active.get("value", False)
    if not (state.flat_faces and show_faces):
        return
    face_poses: dict[str, list[float]] = {}
    for link_name, faces in state.flat_faces.items():
        tf_vis = visual_transforms.get(link_name)
        is_vis = vis_set is None or link_name in vis_set
        for i, ff in enumerate(faces):
            fid = f"face_{link_name}_{i}"
            if tf_vis is not None and is_vis:
                c = ff["centroid"]
                c_m = [c[0] * 0.001, c[1] * 0.001, c[2] * 0.001]
                c_homo = np.array([c_m[0], c_m[1], c_m[2], 1.0])
                wp = tf_vis @ c_homo
                face_poses[fid] = [
                    float(wp[0]),
                    float(wp[1]),
                    float(wp[2]),
                    1.0,
                ]
            else:
                face_poses[fid] = [0.0, 0.0, -100.0, 0.0]
    ui.run_javascript(f"window.__updateFaceMarkerPoses({json.dumps(face_poses)})")
