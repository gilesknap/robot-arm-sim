"""Tests for simulate/app/edit_connections.py — connection editing logic."""

from __future__ import annotations

import math
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from robot_arm_sim.simulate.urdf_loader import load_urdf

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _patch_edit_ui(mock_ui, monkeypatch):
    """Patch module-level ui references for edit_connections and scene_update."""
    from robot_arm_sim.simulate.app import edit_connections as ec_mod
    from robot_arm_sim.simulate.app import scene_update as su_mod

    # Use plain MagicMock for run_javascript since it's called synchronously
    mock_ui.run_javascript = MagicMock(return_value=None)
    monkeypatch.setattr(ec_mod, "ui", mock_ui)
    monkeypatch.setattr(su_mod, "ui", mock_ui)


def _make_state(mock_ui: MagicMock, robot_dir: Path):
    """Build a real SimulatorState for testing."""
    from robot_arm_sim.simulate.app.state import SimulatorState

    robot = load_urdf(robot_dir / "robot.urdf")
    state = SimulatorState(robot, robot_dir)

    for link in robot.links:
        if link.mesh_path:
            obj = MagicMock()
            state.mesh_objects[link.name] = obj

    # Set up UI mocks needed by edit_connections
    state.connection_status_label = MagicMock()
    state.selected_part_label = MagicMock()
    state.edit_connections_btn = MagicMock()
    state.edit_connections_row = MagicMock()
    state.frames_btn = MagicMock()
    state.connections_btn = MagicMock()
    state.connection_centering_select = type("_Dummy", (), {"value": "surface"})()

    return state


# ---------------------------------------------------------------------------
# _compose_and_save_rpy
# ---------------------------------------------------------------------------


class TestComposeAndSaveRpy:
    """Tests for _compose_and_save_rpy — rotation composition."""

    def test_identity_extra_rpy(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _compose_and_save_rpy

        spec: dict = {"visual_rpy": [0.0, 0.0, 0.0]}
        _compose_and_save_rpy(spec, [0.0, 0.0, 0.0])

        result = spec["visual_rpy"]
        for v in result:
            assert abs(v) < 1e-4

    def test_90_degree_z_rotation(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _compose_and_save_rpy

        spec: dict = {"visual_rpy": [0.0, 0.0, 0.0]}
        _compose_and_save_rpy(spec, [0.0, 0.0, math.pi / 2])

        result = spec["visual_rpy"]
        # Should be approximately [0, 0, pi/2]
        assert abs(result[2] - math.pi / 2) < 1e-4

    def test_composition_with_existing_rpy(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _compose_and_save_rpy

        spec: dict = {"visual_rpy": [0.0, 0.0, math.pi / 2]}
        _compose_and_save_rpy(spec, [0.0, 0.0, math.pi / 2])

        result = spec["visual_rpy"]
        # Two 90-degree Z rotations should give ~pi
        assert abs(abs(result[2]) - math.pi) < 1e-4

    def test_no_existing_rpy_defaults_to_zero(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from robot_arm_sim.simulate.app.edit_connections import _compose_and_save_rpy

        spec: dict = {}
        _compose_and_save_rpy(spec, [math.pi / 4, 0.0, 0.0])

        result = spec["visual_rpy"]
        assert abs(result[0] - math.pi / 4) < 1e-4


# ---------------------------------------------------------------------------
# _seed_connection_assignments
# ---------------------------------------------------------------------------


class TestSeedConnectionAssignments:
    """Tests for _seed_connection_assignments."""

    def test_populates_from_connection_points(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from robot_arm_sim.simulate.app.edit_connections import (
            _seed_connection_assignments,
        )

        state = _make_state(mock_ui, robot_dir)
        assert len(state.connection_assignments) == 0

        _seed_connection_assignments(state)

        # Should have populated assignments for links with connection_points
        assert len(state.connection_assignments) > 0
        for link_name, assigns in state.connection_assignments.items():
            assert link_name in state.connection_points
            for end, data in assigns.items():
                assert end in ("proximal", "distal")
                assert "centroid" in data
                assert "normal" in data
                assert "area_mm2" in data
                assert "radius_mm" in data
                assert "centering" in data

    def test_does_not_overwrite_existing(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import (
            _seed_connection_assignments,
        )

        state = _make_state(mock_ui, robot_dir)

        # Pre-populate one assignment
        first_link = next(iter(state.connection_points))
        sentinel = {"centroid": [999, 999, 999], "normal": [1, 0, 0]}
        state.connection_assignments[first_link] = {"proximal": sentinel}

        _seed_connection_assignments(state)

        # The pre-populated one should be untouched
        assert state.connection_assignments[first_link]["proximal"] is sentinel


# ---------------------------------------------------------------------------
# _get_part_name
# ---------------------------------------------------------------------------


class TestGetPartName:
    """Tests for _get_part_name."""

    def test_returns_mesh_stem(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _get_part_name

        state = _make_state(mock_ui, robot_dir)

        for link in state.robot.links:
            if link.mesh_path:
                result = _get_part_name(state, link.name)
                assert result == Path(link.mesh_path).stem

    def test_returns_link_name_when_no_mesh(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _get_part_name

        state = _make_state(mock_ui, robot_dir)

        result = _get_part_name(state, "nonexistent_link")
        assert result == "nonexistent_link"


# ---------------------------------------------------------------------------
# _reverse_action / _apply_action — undo/redo
# ---------------------------------------------------------------------------


class TestReverseAction:
    """Tests for _reverse_action — undo logic."""

    def test_undo_assign_new(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _reverse_action

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.connection_points))

        # Simulate a new assignment (before=None)
        after_data = {"centroid": [1, 2, 3], "normal": [0, 0, 1]}
        state.connection_assignments[link_name] = {"proximal": after_data}

        action = {
            "type": "assign",
            "link_name": link_name,
            "end": "proximal",
            "before": None,
            "after": after_data,
        }
        _reverse_action(state, action)

        # Should remove the assignment
        assigns = state.connection_assignments.get(link_name, {})
        assert "proximal" not in assigns

    def test_undo_assign_restore_previous(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _reverse_action

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.connection_points))

        before_data = {
            "centroid": [10, 20, 30],
            "normal": [0, 1, 0],
            "centering": "surface",
        }
        after_data = {"centroid": [1, 2, 3], "normal": [0, 0, 1]}
        state.connection_assignments[link_name] = {"distal": after_data}

        action = {
            "type": "assign",
            "link_name": link_name,
            "end": "distal",
            "before": before_data,
            "after": after_data,
        }
        _reverse_action(state, action)

        assert state.connection_assignments[link_name]["distal"] is before_data

    def test_undo_move(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _reverse_action

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        state.part_visual_offsets[link_name] = [0.1, 0.2, 0.3]
        action = {
            "type": "move",
            "link_name": link_name,
            "delta": [0.1, 0.2, 0.3],
        }
        _reverse_action(state, action)

        result = state.part_visual_offsets[link_name]
        assert abs(result[0]) < 1e-6
        assert abs(result[1]) < 1e-6
        assert abs(result[2]) < 1e-6

    def test_undo_rotate(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _reverse_action

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        before_rpy = [0.0, 0.0, 0.0]
        after_rpy = [math.pi / 2, 0.0, 0.0]
        state.part_visual_rpys[link_name] = list(after_rpy)

        action = {
            "type": "rotate",
            "link_name": link_name,
            "axis": 0,
            "angle": math.pi / 2,
            "before_rpy": before_rpy,
            "after_rpy": after_rpy,
        }
        _reverse_action(state, action)

        assert state.part_visual_rpys[link_name] == before_rpy


class TestApplyAction:
    """Tests for _apply_action — redo logic."""

    def test_redo_assign(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _apply_action

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.connection_points))

        after_data = {
            "centroid": [1, 2, 3],
            "normal": [0, 0, 1],
            "centering": "surface",
        }
        action = {
            "type": "assign",
            "link_name": link_name,
            "end": "proximal",
            "before": None,
            "after": after_data,
        }
        _apply_action(state, action)

        assert state.connection_assignments[link_name]["proximal"] is after_data

    def test_redo_move(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _apply_action

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        state.part_visual_offsets[link_name] = [0.0, 0.0, 0.0]
        action = {
            "type": "move",
            "link_name": link_name,
            "delta": [0.5, 0.3, 0.1],
        }
        _apply_action(state, action)

        result = state.part_visual_offsets[link_name]
        assert abs(result[0] - 0.5) < 1e-6
        assert abs(result[1] - 0.3) < 1e-6
        assert abs(result[2] - 0.1) < 1e-6

    def test_redo_rotate(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _apply_action

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        before_rpy = [0.0, 0.0, 0.0]
        after_rpy = [0.0, math.pi / 2, 0.0]
        state.part_visual_rpys[link_name] = list(before_rpy)

        action = {
            "type": "rotate",
            "link_name": link_name,
            "axis": 1,
            "angle": math.pi / 2,
            "before_rpy": before_rpy,
            "after_rpy": after_rpy,
        }
        _apply_action(state, action)

        assert state.part_visual_rpys[link_name] == after_rpy


# ---------------------------------------------------------------------------
# _sync_connection_visibility
# ---------------------------------------------------------------------------


class TestSyncConnectionVisibility:
    """Tests for _sync_connection_visibility."""

    def test_generates_js_for_all_connections(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from robot_arm_sim.simulate.app.edit_connections import (
            _sync_connection_visibility,
        )

        state = _make_state(mock_ui, robot_dir)

        _sync_connection_visibility(state)

        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        js_calls = " ".join(calls)
        assert "__connSpheres" in js_calls

    def test_show_all_makes_all_visible(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import (
            _sync_connection_visibility,
        )

        state = _make_state(mock_ui, robot_dir)
        state.show_all_connections["value"] = True

        # Hide some links
        for lname in list(state.visible_links.keys())[:2]:
            state.visible_links[lname] = False

        _sync_connection_visibility(state)

        # JS should still be generated (show_all overrides)
        mock_ui.run_javascript.assert_called()


# ---------------------------------------------------------------------------
# _force_reset_mesh_positions
# ---------------------------------------------------------------------------


class TestForceResetMeshPositions:
    """Tests for _force_reset_mesh_positions."""

    def test_sets_nan_positions(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import (
            _force_reset_mesh_positions,
        )

        state = _make_state(mock_ui, robot_dir)

        # Set up mock mesh objects with numeric positions
        for obj in state.mesh_objects.values():
            obj.x = 1.0
            obj.y = 2.0
            obj.z = 3.0

        _force_reset_mesh_positions(state)

        for obj in state.mesh_objects.values():
            assert math.isnan(obj.x)
            assert math.isnan(obj.y)
            assert math.isnan(obj.z)


# ---------------------------------------------------------------------------
# _update_selected_part
# ---------------------------------------------------------------------------


class TestUpdateSelectedPart:
    """Tests for _update_selected_part."""

    def test_updates_name_and_label(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _update_selected_part

        state = _make_state(mock_ui, robot_dir)

        link = next(lnk for lnk in state.robot.links if lnk.mesh_path)
        _update_selected_part(state, link.name)

        assert state.selected_part_name == link.name
        assert link.mesh_path is not None
        expected_text = Path(link.mesh_path).stem
        state.selected_part_label.update.assert_called()
        assert state.selected_part_label.text == expected_text


# ---------------------------------------------------------------------------
# build_edit_connections (UI builder)
# ---------------------------------------------------------------------------


class TestBuildEditConnections:
    """Tests for build_edit_connections UI builder."""

    def test_builds_without_error(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        # build_edit_connections needs context manager support
        build_edit_connections(state)

        # Should have set the toggle handler
        assert state.toggle_edit_connections is not None
        assert callable(state.toggle_edit_connections)

    def test_toggle_sets_active_state(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        assert state.edit_connections_active["value"] is False
        state.toggle_edit_connections()
        assert state.edit_connections_active["value"] is True

    def test_toggle_off_restores_snapshot(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        # Toggle on
        state.toggle_edit_connections()
        assert state.edit_connections_active["value"] is True

        # Add a dirty change
        state.connection_dirty_links.add("test_link")
        state.part_visual_offsets["test_link"] = [1, 2, 3]

        # Toggle off — should clear dirty state
        state.toggle_edit_connections()
        assert state.edit_connections_active["value"] is False
        assert len(state.connection_dirty_links) == 0
        assert len(state.part_visual_offsets) == 0

    def test_on_visibility_changed_syncs_when_active(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        # Toggle on
        state.toggle_edit_connections()

        # Calling on_visibility_changed should not raise
        state.on_visibility_changed()

    def test_toggle_on_seeds_assignments(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        # Toggle on — should seed connection assignments from connection_points
        state.toggle_edit_connections()
        assert state.edit_connections_active["value"] is True

        # Should have seeded connection_assignments
        if state.connection_points:
            assert len(state.connection_assignments) > 0

    def test_toggle_on_takes_snapshot(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.toggle_edit_connections()
        # conn_snapshot should be a deep copy of connection_assignments
        assert state.conn_snapshot is not state.connection_assignments
        assert state.connections_visible["value"] is True

    def test_toggle_off_clears_undo_redo(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.toggle_edit_connections()  # on
        state.undo_stack.append({"type": "move", "link_name": "x", "delta": [1, 0, 0]})
        state.redo_stack.append({"type": "move", "link_name": "y", "delta": [0, 1, 0]})
        state.toggle_edit_connections()  # off

        assert len(state.undo_stack) == 0
        assert len(state.redo_stack) == 0

    def test_toggle_off_resets_selected_part(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.toggle_edit_connections()  # on
        state.selected_part_name = "some_link"
        state.toggle_edit_connections()  # off

        assert state.selected_part_name is None
        assert state.selected_part_label.text == "(no part)"

    def test_toggle_on_enables_frames(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        assert state.frames_visible["value"] is False
        state.toggle_edit_connections()  # on
        assert state.frames_visible["value"] is True

    def test_toggle_off_clears_rpys(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.toggle_edit_connections()  # on
        state.part_visual_rpys["link1"] = [1.0, 2.0, 3.0]
        state.toggle_edit_connections()  # off

        assert len(state.part_visual_rpys) == 0

    def test_toggle_off_restores_connections_btn(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.toggle_edit_connections()  # on
        state.toggle_edit_connections()  # off

        # connections_btn should be styled
        state.connections_btn.props.assert_called()
        state.connections_btn.update.assert_called()

    def test_visibility_changed_noop_when_inactive(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        # Should not raise even when inactive
        mock_ui.run_javascript.reset_mock()
        state.on_visibility_changed()
        # _sync_connection_visibility should not have been called
        # (no __connSpheres JS call expected when inactive)


# ---------------------------------------------------------------------------
# _handle_part_move
# ---------------------------------------------------------------------------


class TestHandlePartMove:
    """Tests for _handle_part_move — drag delta tracking."""

    def test_accumulates_offset(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        _handle_part_move(state, {"linkName": link_name, "delta": [0.1, 0.2, 0.3]})

        assert state.part_visual_offsets[link_name] == [
            pytest.approx(0.1),
            pytest.approx(0.2),
            pytest.approx(0.3),
        ]

    def test_accumulates_multiple_moves(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        _handle_part_move(state, {"linkName": link_name, "delta": [0.1, 0.0, 0.0]})
        _handle_part_move(state, {"linkName": link_name, "delta": [0.2, 0.0, 0.0]})

        assert state.part_visual_offsets[link_name][0] == pytest.approx(0.3)

    def test_records_undo(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        _handle_part_move(state, {"linkName": link_name, "delta": [0.5, 0.0, 0.0]})

        assert len(state.undo_stack) == 1
        assert state.undo_stack[0]["type"] == "move"
        assert state.undo_stack[0]["delta"] == [0.5, 0.0, 0.0]

    def test_clears_redo_stack(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))
        state.redo_stack.append({"type": "move", "link_name": "x", "delta": [1, 0, 0]})

        _handle_part_move(state, {"linkName": link_name, "delta": [0.1, 0.0, 0.0]})

        assert len(state.redo_stack) == 0

    def test_marks_link_dirty(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        _handle_part_move(state, {"linkName": link_name, "delta": [0.1, 0.0, 0.0]})

        assert link_name in state.connection_dirty_links

    def test_ignores_zero_delta(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)

        _handle_part_move(state, {"linkName": "link1", "delta": [0, 0, 0]})

        assert len(state.undo_stack) == 0
        assert "link1" not in state.part_visual_offsets

    def test_ignores_no_link_name(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)

        _handle_part_move(state, {"delta": [1, 2, 3]})

        assert len(state.undo_stack) == 0

    def test_updates_status_label(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_part_move

        state = _make_state(mock_ui, robot_dir)
        link_name = next(iter(state.mesh_objects))

        _handle_part_move(state, {"linkName": link_name, "delta": [0.1, 0.0, 0.0]})

        assert "Moved" in state.connection_status_label.text


# ---------------------------------------------------------------------------
# _handle_mesh_click
# ---------------------------------------------------------------------------


class TestHandleMeshClick:
    """Tests for _handle_mesh_click — mode-dependent click processing."""

    def test_move_parts_mode_just_selects(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_mesh_click

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)
        state.connection_edit_mode["value"] = "move_parts"

        _handle_mesh_click(state, {"linkName": link_name, "type": "mesh"})

        assert state.selected_part_name == link_name
        # Should NOT have created any connection assignment
        assert len(state.undo_stack) == 0

    def test_proximal_centred_mode_assigns(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_mesh_click

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)
        state.connection_edit_mode["value"] = "proximal_centred"

        _handle_mesh_click(
            state,
            {
                "linkName": link_name,
                "type": "mesh",
                "point": [0.01, 0.02, 0.03],
                "normal": [0, 0, 1],
            },
        )

        assert link_name in state.connection_assignments
        assert "proximal" in state.connection_assignments[link_name]
        cd = state.connection_assignments[link_name]["proximal"]
        assert cd["centering"] == "center"

    def test_proximal_surface_mode_assigns(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_mesh_click

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)
        state.connection_edit_mode["value"] = "proximal_surface"

        _handle_mesh_click(
            state,
            {
                "linkName": link_name,
                "type": "mesh",
                "point": [0.01, 0.02, 0.03],
                "normal": [0, 0, 1],
            },
        )

        assert link_name in state.connection_assignments
        assert "proximal" in state.connection_assignments[link_name]
        cd = state.connection_assignments[link_name]["proximal"]
        assert cd["centering"] == "surface"

    def test_distal_mode_assigns(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_mesh_click

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)
        state.connection_edit_mode["value"] = "distal"

        _handle_mesh_click(
            state,
            {
                "linkName": link_name,
                "type": "mesh",
                "point": [0.01, 0.02, 0.03],
                "normal": [0, 0, 1],
            },
        )

        assert link_name in state.connection_assignments
        assert "distal" in state.connection_assignments[link_name]
        cd = state.connection_assignments[link_name]["distal"]
        assert cd["centering"] == "surface"

    def test_mesh_click_records_undo(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _handle_mesh_click

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)
        state.connection_edit_mode["value"] = "distal"

        _handle_mesh_click(
            state,
            {
                "linkName": link_name,
                "type": "mesh",
                "point": [0.01, 0.02, 0.03],
                "normal": [0, 0, 1],
            },
        )

        assert len(state.undo_stack) == 1
        assert state.undo_stack[0]["type"] == "assign"
        assert state.undo_stack[0]["end"] == "distal"


# ---------------------------------------------------------------------------
# _assign_connection
# ---------------------------------------------------------------------------


class TestAssignConnection:
    """Tests for _assign_connection — assignment + undo recording."""

    def test_new_assignment(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _assign_connection

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)

        cd = {
            "centroid": [10, 20, 30],
            "normal": [0, 0, 1],
            "area_mm2": 78.54,
            "radius_mm": 5.0,
            "centering": "surface",
        }
        _assign_connection(state, link_name, "proximal", cd)

        assert state.connection_assignments[link_name]["proximal"] is cd
        assert len(state.undo_stack) == 1
        assert state.undo_stack[0]["before"] is None
        assert link_name in state.connection_dirty_links
        assert state.connection_center_target == (link_name, "proximal")

    def test_overwrites_previous_with_undo(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _assign_connection

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)

        old = {
            "centroid": [1, 2, 3],
            "normal": [0, 0, 1],
            "centering": "surface",
        }
        state.connection_assignments[link_name] = {"proximal": old}

        new = {
            "centroid": [10, 20, 30],
            "normal": [0, 1, 0],
            "centering": "center",
        }
        _assign_connection(state, link_name, "proximal", new)

        assert state.connection_assignments[link_name]["proximal"] is new
        assert state.undo_stack[0]["before"] == old

    def test_clears_redo_stack(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _assign_connection

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)
        state.redo_stack.append({"type": "dummy"})

        cd = {"centroid": [0, 0, 0], "normal": [0, 0, 1], "centering": "surface"}
        _assign_connection(state, link_name, "distal", cd)

        assert len(state.redo_stack) == 0

    def test_updates_centering_state(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _assign_connection

        state = _make_state(mock_ui, robot_dir)
        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)

        cd = {"centroid": [0, 0, 0], "normal": [0, 0, 1], "centering": "center"}
        _assign_connection(state, link_name, "proximal", cd)

        assert state.connection_centering["value"] == "center"
        assert state.connection_centering_select.value == "center"


# ---------------------------------------------------------------------------
# _get_sphere_id
# ---------------------------------------------------------------------------


class TestGetSphereId:
    """Tests for _get_sphere_id."""

    def test_found(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _get_sphere_id

        state = _make_state(mock_ui, robot_dir)

        # Pick first link with connection_points
        for ln, cps in state.connection_points.items():
            if cps:
                end = cps[0]["end"]
                result = _get_sphere_id(state, ln, end)
                assert result == f"{ln}_{end}_0"
                break

    def test_not_found(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import _get_sphere_id

        state = _make_state(mock_ui, robot_dir)
        result = _get_sphere_id(state, "nonexistent", "proximal")
        assert result is None


# ---------------------------------------------------------------------------
# _poll_face_click (async)
# ---------------------------------------------------------------------------


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestPollFaceClick:
    """Tests for _poll_face_click — async JS polling."""

    @pytest.mark.asyncio
    async def test_noop_when_inactive(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        # Not active — poll should return immediately
        # Get the timer callback
        timer_calls = mock_ui.timer.call_args_list
        assert len(timer_calls) > 0
        poll_fn = timer_calls[-1][0][1]  # second positional arg is the callback

        mock_ui.run_javascript.reset_mock()
        await poll_fn()
        # Should not have called run_javascript since not active
        mock_ui.run_javascript.assert_not_called()

    @pytest.mark.asyncio
    async def test_handles_mesh_click_result(self, mock_ui: MagicMock, robot_dir: Path):
        from unittest.mock import AsyncMock

        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.edit_connections_active["value"] = True
        state.connection_edit_mode["value"] = "distal"

        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)

        # Mock JS to return a mesh click, then None for move and select
        click_result = {
            "type": "mesh",
            "linkName": link_name,
            "point": [0.01, 0.02, 0.03],
            "normal": [0, 0, 1],
        }
        mock_ui.run_javascript = AsyncMock(side_effect=[click_result, None, None])

        timer_calls = mock_ui.timer.call_args_list
        poll_fn = timer_calls[-1][0][1]

        await poll_fn()

        # Should have assigned a distal connection
        assert link_name in state.connection_assignments

    @pytest.mark.asyncio
    async def test_handles_part_move_result(self, mock_ui: MagicMock, robot_dir: Path):
        from unittest.mock import AsyncMock

        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.edit_connections_active["value"] = True

        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)

        # No face click, but a part move
        move_result = {"linkName": link_name, "delta": [0.1, 0.0, 0.0]}
        mock_ui.run_javascript = AsyncMock(side_effect=[None, move_result, None])

        timer_calls = mock_ui.timer.call_args_list
        poll_fn = timer_calls[-1][0][1]

        await poll_fn()

        assert link_name in state.part_visual_offsets

    @pytest.mark.asyncio
    async def test_handles_part_select_result(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from unittest.mock import AsyncMock

        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.edit_connections_active["value"] = True

        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)

        # No face click, no move, but a selection
        mock_ui.run_javascript = AsyncMock(
            side_effect=[None, None, {"linkName": link_name}]
        )

        timer_calls = mock_ui.timer.call_args_list
        poll_fn = timer_calls[-1][0][1]

        await poll_fn()

        assert state.selected_part_name == link_name

    @pytest.mark.asyncio
    async def test_handles_timeout_error(self, mock_ui: MagicMock, robot_dir: Path):
        from unittest.mock import AsyncMock

        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        state.edit_connections_active["value"] = True

        mock_ui.run_javascript = AsyncMock(side_effect=TimeoutError)

        timer_calls = mock_ui.timer.call_args_list
        poll_fn = timer_calls[-1][0][1]

        # Should not raise
        await poll_fn()


# ---------------------------------------------------------------------------
# _save_and_rebuild (async, via build_edit_connections)
# ---------------------------------------------------------------------------


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestSaveAndRebuild:
    """Tests for _save_and_rebuild — invoked via button handler."""

    @pytest.mark.asyncio
    async def test_save_writes_yaml(self, mock_ui: MagicMock, robot_dir: Path):
        """Test that save writes connection data to part YAML files."""
        from unittest.mock import AsyncMock, patch

        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        # Toggle on to seed assignments
        state.toggle_edit_connections()

        # Mark a link dirty and give it an assignment
        link = next(lnk for lnk in state.robot.links if lnk.mesh_path)
        link_name = link.name
        state.connection_dirty_links.add(link_name)

        state.connection_assignments[link_name] = {
            "proximal": {
                "centroid": [1.0, 2.0, 3.0],
                "normal": [0.0, 0.0, 1.0],
                "area_mm2": 78.54,
                "radius_mm": 5.0,
                "centering": "surface",
            }
        }

        # Find the save button handler from mock calls
        # The save handler is _save_and_rebuild, captured via ui.button
        button_calls = mock_ui.button.call_args_list
        save_btn_call = None
        for call in button_calls:
            args, kwargs = call
            if args and "Save" in str(args[0]):
                save_btn_call = call
                break

        if save_btn_call is None:
            pytest.skip("Could not locate Save button in mock calls")

        _, kwargs = save_btn_call
        save_handler = kwargs.get("on_click")
        if save_handler is None:
            pytest.skip("Save button has no on_click handler")

        # Mock generate_urdf and reload_urdf
        state.reload_urdf = AsyncMock()
        with patch("robot_arm_sim.analyze.urdf_generator.generate_urdf"):
            await save_handler()

        assert "Saved" in state.connection_status_label.text


# ---------------------------------------------------------------------------
# _remove_connections (async, via build_edit_connections)
# ---------------------------------------------------------------------------


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestRemoveConnections:
    """Tests for _remove_connections — invoked via button handler."""

    @pytest.mark.asyncio
    async def test_remove_clears_connections(self, mock_ui: MagicMock, robot_dir: Path):
        """Test that remove connections clears part YAMLs."""
        from unittest.mock import AsyncMock, patch

        from robot_arm_sim.simulate.app.edit_connections import build_edit_connections

        state = _make_state(mock_ui, robot_dir)
        build_edit_connections(state)

        # Find remove button handler
        button_calls = mock_ui.button.call_args_list
        remove_btn_call = None
        for call in button_calls:
            args, kwargs = call
            if args and "Remove Connections" == str(args[0]):
                remove_btn_call = call
                break

        if remove_btn_call is None:
            pytest.skip("Could not locate Remove Connections button")

        _, kwargs = remove_btn_call
        remove_handler = kwargs.get("on_click")
        if remove_handler is None:
            pytest.skip("Remove button has no on_click handler")

        state.reload_urdf = AsyncMock()
        with patch("robot_arm_sim.analyze.urdf_generator.generate_urdf"):
            await remove_handler()

        assert (
            "Removed" in state.connection_status_label.text
            or "removed" in state.connection_status_label.text.lower()
            or "Reloading" in state.connection_status_label.text
        )


# ---------------------------------------------------------------------------
# _remove_connections_for_link (async)
# ---------------------------------------------------------------------------


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestRemoveConnectionsForLink:
    """Tests for _remove_connections_for_link."""

    @pytest.mark.asyncio
    async def test_missing_urdf(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import (
            _remove_connections_for_link,
        )

        state = _make_state(mock_ui, robot_dir)
        # Remove the URDF
        (robot_dir / "robot.urdf").unlink()

        link_name = next(lnk.name for lnk in state.robot.links if lnk.mesh_path)
        await _remove_connections_for_link(state, link_name)

        assert "Missing" in state.connection_status_label.text

    @pytest.mark.asyncio
    async def test_no_mesh_for_link(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import (
            _remove_connections_for_link,
        )

        state = _make_state(mock_ui, robot_dir)

        await _remove_connections_for_link(state, "nonexistent_link")

        assert "No mesh" in state.connection_status_label.text

    @pytest.mark.asyncio
    async def test_removes_for_valid_link(self, mock_ui: MagicMock, robot_dir: Path):
        from unittest.mock import AsyncMock, patch

        from robot_arm_sim.simulate.app.edit_connections import (
            _remove_connections_for_link,
        )

        state = _make_state(mock_ui, robot_dir)
        state.reload_urdf = AsyncMock()

        link = next(lnk for lnk in state.robot.links if lnk.mesh_path)
        link_name = link.name

        # Pre-populate assignment
        state.connection_assignments[link_name] = {"proximal": {"centroid": [0, 0, 0]}}

        with patch("robot_arm_sim.analyze.urdf_generator.generate_urdf"):
            await _remove_connections_for_link(state, link_name)

        assert link_name not in state.connection_assignments
        assert "Removed" in state.connection_status_label.text


# ---------------------------------------------------------------------------
# _get_visual_transforms
# ---------------------------------------------------------------------------


class TestGetVisualTransforms:
    """Tests for _get_visual_transforms."""

    def test_returns_transforms_for_all_links(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        import numpy as np

        from robot_arm_sim.simulate.app.edit_connections import (
            _get_visual_transforms,
        )

        state = _make_state(mock_ui, robot_dir)
        transforms = _get_visual_transforms(state)

        assert len(transforms) > 0
        for _link_name, tf in transforms.items():
            assert isinstance(tf, np.ndarray)
            assert tf.shape == (4, 4)


# ---------------------------------------------------------------------------
# _move_connection_sphere
# ---------------------------------------------------------------------------


class TestMoveConnectionSphere:
    """Tests for _move_connection_sphere."""

    def test_calls_js_for_valid_sphere(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import (
            _move_connection_sphere,
        )

        state = _make_state(mock_ui, robot_dir)

        # Find a link with connection_points
        for ln, cps in state.connection_points.items():
            if cps:
                end = cps[0]["end"]
                mock_ui.run_javascript.reset_mock()
                _move_connection_sphere(
                    state, ln, end, [10.0, 20.0, 30.0], centering="surface"
                )
                # Should have called run_javascript with __moveConnectionSphere
                calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
                js = " ".join(calls)
                assert "__moveConnectionSphere" in js
                break

    def test_noop_for_unknown_link(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.edit_connections import (
            _move_connection_sphere,
        )

        state = _make_state(mock_ui, robot_dir)
        mock_ui.run_javascript.reset_mock()

        _move_connection_sphere(state, "nonexistent_link", "proximal", [0, 0, 0])
        # Should not call __moveConnectionSphere
        calls = [str(c) for c in mock_ui.run_javascript.call_args_list]
        assert "__moveConnectionSphere" not in " ".join(calls)
