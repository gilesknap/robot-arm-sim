"""Tests for simulate/app/scene_update.py — FK scene update logic."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from robot_arm_sim.simulate.urdf_loader import load_urdf

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _patch_scene_ui(mock_ui, monkeypatch):
    """Patch scene_update's module-level ui reference for all tests.

    Use a plain MagicMock for run_javascript since scene_update calls it
    synchronously (not awaited). AsyncMock would cause unawaited coroutine
    warnings that are treated as errors.
    """
    from robot_arm_sim.simulate.app import scene_update as su_mod

    mock_ui.run_javascript = MagicMock(return_value=None)
    monkeypatch.setattr(su_mod, "ui", mock_ui)


def _make_state_with_mocks(mock_ui: MagicMock, robot_dir: Path):
    """Build a real SimulatorState with mock mesh objects."""
    from robot_arm_sim.simulate.app.state import SimulatorState

    robot = load_urdf(robot_dir / "robot.urdf")
    state = SimulatorState(robot, robot_dir)

    for link in robot.links:
        if link.mesh_path:
            obj = MagicMock()
            state.mesh_objects[link.name] = obj

    return state


# ---------------------------------------------------------------------------
# update_scene
# ---------------------------------------------------------------------------


class TestUpdateScene:
    """Tests for the main update_scene function."""

    def test_moves_all_mesh_objects(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import update_scene

        state = _make_state_with_mocks(mock_ui, robot_dir)
        update_scene(state)

        for obj in state.mesh_objects.values():
            obj.move.assert_called_once()
            obj.rotate.assert_called_once()

    def test_hidden_links_moved_offscreen(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import HIDDEN_POS, update_scene

        state = _make_state_with_mocks(mock_ui, robot_dir)

        # Hide the first link
        first_link = state.chain_link_names[0]
        state.visible_links[first_link] = False

        update_scene(state)

        if first_link in state.mesh_objects:
            obj = state.mesh_objects[first_link]
            obj.move.assert_called_once_with(*HIDDEN_POS)
            obj.rotate.assert_not_called()

    def test_syncs_joint_angles_to_js(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import update_scene

        state = _make_state_with_mocks(mock_ui, robot_dir)
        update_scene(state)

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__simJointAngles" in js_calls

    def test_nonzero_joint_angles(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import update_scene

        state = _make_state_with_mocks(mock_ui, robot_dir)

        for i, jname in enumerate(state.joint_angles):
            state.joint_angles[jname] = 0.1 * (i + 1)

        update_scene(state)

        for obj in state.mesh_objects.values():
            obj.move.assert_called_once()

    def test_edit_mode_applies_extra_rpy(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import update_scene

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.edit_connections_active["value"] = True

        first_link = next(
            ln for ln in state.chain_link_names if ln in state.mesh_objects
        )
        state.part_visual_rpys[first_link] = [0.5, 0.0, 0.0]

        update_scene(state)

        obj = state.mesh_objects[first_link]
        obj.move.assert_called_once()
        obj.rotate.assert_called_once()


# ---------------------------------------------------------------------------
# _update_ee_readout
# ---------------------------------------------------------------------------


class TestUpdateEeReadout:
    """Tests for end-effector readout updates."""

    def test_updates_label_text(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_ee_readout

        state = _make_state_with_mocks(mock_ui, robot_dir)

        trans_label = MagicMock()
        rot_label = MagicMock()
        state.ee_readout_ref = [trans_label, rot_label]

        from robot_arm_sim.simulate.kinematics import forward_kinematics

        transforms = forward_kinematics(state.robot, state.joint_angles)
        _update_ee_readout(state, transforms)

        assert "X:" in trans_label.text
        assert "mm" in trans_label.text
        assert "Rx:" in rot_label.text

    def test_no_update_when_labels_none(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_ee_readout

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.ee_readout_ref = [None, None]

        from robot_arm_sim.simulate.kinematics import forward_kinematics

        transforms = forward_kinematics(state.robot, state.joint_angles)
        _update_ee_readout(state, transforms)


# ---------------------------------------------------------------------------
# _update_callouts
# ---------------------------------------------------------------------------


class TestUpdateCallouts:
    """Tests for label callout position updates."""

    def test_labels_hidden_when_not_visible(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_callouts

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.labels_visible["value"] = False

        _update_callouts(state, {}, {})

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__setLabelsVisible" in js_calls

    def test_labels_shown_with_anchors(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_callouts

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.labels_visible["value"] = True

        joint_positions = {
            j.name: (0.0, 0.0, float(i) * 0.1) for i, j in enumerate(state.chain)
        }
        link_positions = dict.fromkeys(state.chain_link_names, (0.0, 0.0, 0.0))

        _update_callouts(state, joint_positions, link_positions)

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__updateLabelAnchors" in js_calls


# ---------------------------------------------------------------------------
# _update_frames
# ---------------------------------------------------------------------------


class TestUpdateFrames:
    """Tests for coordinate frame axis updates."""

    def test_no_update_when_not_visible(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_frames

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.frames_visible["value"] = False

        from robot_arm_sim.simulate.kinematics import forward_kinematics

        transforms = forward_kinematics(state.robot, state.joint_angles)
        _update_frames(state, transforms)

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__updateAxesPoses" not in js_calls

    def test_update_when_visible(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_frames

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.frames_visible["value"] = True

        from robot_arm_sim.simulate.kinematics import forward_kinematics

        transforms = forward_kinematics(state.robot, state.joint_angles)
        _update_frames(state, transforms)

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__updateAxesPoses" in js_calls


# ---------------------------------------------------------------------------
# _update_connections
# ---------------------------------------------------------------------------


class TestUpdateConnections:
    """Tests for connection marker updates."""

    def test_no_update_when_not_visible(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_connections

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.connections_visible["value"] = False

        _update_connections(state, {}, None)

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__updateConnectionPoses" not in js_calls

    def test_update_when_visible(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_update import _update_connections

        state = _make_state_with_mocks(mock_ui, robot_dir)
        state.connections_visible["value"] = True

        from robot_arm_sim.simulate.kinematics import forward_kinematics

        transforms = forward_kinematics(state.robot, state.joint_angles)
        visual_transforms = {}
        for link_name, tf in transforms.items():
            visual_transforms[link_name] = tf.copy()

        vis_set = set(state.visible_links.keys())

        _update_connections(state, visual_transforms, vis_set)

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__updateConnectionPoses" in js_calls
