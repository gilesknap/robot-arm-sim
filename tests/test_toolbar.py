"""Tests for simulate/app/toolbar.py — toolbar and visibility UI."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from robot_arm_sim.simulate.urdf_loader import load_urdf

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _patch_toolbar_ui(mock_ui, monkeypatch):
    """Patch module-level ui references for toolbar and scene_update."""
    from robot_arm_sim.simulate.app import scene_update as su_mod
    from robot_arm_sim.simulate.app import toolbar as tb_mod

    mock_ui.run_javascript = MagicMock(return_value=None)
    monkeypatch.setattr(tb_mod, "ui", mock_ui)
    monkeypatch.setattr(su_mod, "ui", mock_ui)


def _make_state(mock_ui: MagicMock, robot_dir: Path):
    """Build a SimulatorState for toolbar testing."""
    from robot_arm_sim.simulate.app.state import SimulatorState

    robot = load_urdf(robot_dir / "robot.urdf")
    state = SimulatorState(robot, robot_dir)

    for link in robot.links:
        if link.mesh_path:
            state.mesh_objects[link.name] = MagicMock()

    return state


# ---------------------------------------------------------------------------
# _apply_toggle_style
# ---------------------------------------------------------------------------


class TestApplyToggleStyle:
    """Tests for _apply_toggle_style button styling."""

    def test_active_style(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.toolbar import _apply_toggle_style

        btn = MagicMock()
        _apply_toggle_style(btn, active=True)

        btn.props.assert_called_once_with("dense color=primary")
        btn.classes.assert_called_once()
        btn.update.assert_called_once()

    def test_inactive_style(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.toolbar import _apply_toggle_style

        btn = MagicMock()
        _apply_toggle_style(btn, active=False)

        btn.props.assert_called_once_with("flat dense")
        btn.classes.assert_called_once()
        btn.update.assert_called_once()


# ---------------------------------------------------------------------------
# _CLUSTER_MODE detection
# ---------------------------------------------------------------------------


class TestClusterMode:
    """Tests for _CLUSTER_MODE environment variable detection."""

    def test_cluster_mode_from_env(self, monkeypatch):
        monkeypatch.setenv("CLUSTER_MODE", "1")
        # Force reimport to pick up env change
        import importlib

        import robot_arm_sim.simulate.app.toolbar as toolbar_mod

        importlib.reload(toolbar_mod)
        assert toolbar_mod._CLUSTER_MODE is True
        # Cleanup: reload without the var
        monkeypatch.delenv("CLUSTER_MODE", raising=False)
        monkeypatch.delenv("KUBERNETES_SERVICE_HOST", raising=False)
        importlib.reload(toolbar_mod)

    def test_cluster_mode_from_k8s(self, monkeypatch):
        monkeypatch.delenv("CLUSTER_MODE", raising=False)
        monkeypatch.setenv("KUBERNETES_SERVICE_HOST", "10.0.0.1")
        import importlib

        import robot_arm_sim.simulate.app.toolbar as toolbar_mod

        importlib.reload(toolbar_mod)
        assert toolbar_mod._CLUSTER_MODE is True
        monkeypatch.delenv("KUBERNETES_SERVICE_HOST", raising=False)
        importlib.reload(toolbar_mod)

    def test_no_cluster_mode(self, monkeypatch):
        monkeypatch.delenv("CLUSTER_MODE", raising=False)
        monkeypatch.delenv("KUBERNETES_SERVICE_HOST", raising=False)
        import importlib

        import robot_arm_sim.simulate.app.toolbar as toolbar_mod

        importlib.reload(toolbar_mod)
        assert toolbar_mod._CLUSTER_MODE is False


# ---------------------------------------------------------------------------
# build_toolbar
# ---------------------------------------------------------------------------


class TestBuildToolbar:
    """Tests for build_toolbar UI builder."""

    def test_builds_without_error(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.toolbar import build_toolbar

        state = _make_state(mock_ui, robot_dir)
        build_toolbar(state)

        # Should have set frames_btn and connections_btn on state
        assert state.frames_btn is not None
        assert state.connections_btn is not None
        assert state.edit_connections_btn is not None

    def test_toggle_labels_handler(self, mock_ui: MagicMock, robot_dir: Path):
        """Exercise the toggle_labels inner function (lines 126-128)."""
        from robot_arm_sim.simulate.app.toolbar import build_toolbar

        state = _make_state(mock_ui, robot_dir)
        state.update_scene_now = MagicMock()
        build_toolbar(state)

        # Find the on_click handler for "Labels" button
        label_call = _find_button_call(mock_ui, "Labels")
        assert label_call is not None, "Labels button not found"
        handler = label_call[1]["on_click"]

        # Initially labels_visible is False, toggling should set True
        assert state.labels_visible["value"] is False
        handler()
        assert state.labels_visible["value"] is True
        state.update_scene_now.assert_called()

    def test_toggle_frames_handler(self, mock_ui: MagicMock, robot_dir: Path):
        """Exercise the toggle_frames inner function (lines 133-138)."""
        from robot_arm_sim.simulate.app.toolbar import build_toolbar

        state = _make_state(mock_ui, robot_dir)
        state.update_scene_now = MagicMock()
        build_toolbar(state)

        result = _find_button_call(mock_ui, "Frames")
        assert result is not None
        handler = result[1]["on_click"]

        # Toggle on
        handler()
        assert state.frames_visible["value"] is True
        mock_ui.run_javascript.assert_called()
        state.update_scene_now.assert_called()

        # Toggle off
        state.update_scene_now.reset_mock()
        handler()
        assert state.frames_visible["value"] is False

    def test_toggle_transparent_handler(self, mock_ui: MagicMock, robot_dir: Path):
        """Exercise the toggle_transparent inner function (lines 144-148)."""
        from robot_arm_sim.simulate.app.toolbar import build_toolbar

        state = _make_state(mock_ui, robot_dir)
        build_toolbar(state)

        result = _find_button_call(mock_ui, "Transparent")
        assert result is not None
        handler = result[1]["on_click"]

        handler()
        assert state.transparent_mode["value"] is True
        mock_ui.run_javascript.assert_called()

    def test_toggle_connections_handler(self, mock_ui: MagicMock, robot_dir: Path):
        """Exercise the toggle_connections inner function (lines 159-164)."""
        from robot_arm_sim.simulate.app.toolbar import build_toolbar

        state = _make_state(mock_ui, robot_dir)
        state.update_scene_now = MagicMock()
        build_toolbar(state)

        result = _find_button_call(mock_ui, "Connections")
        assert result is not None
        handler = result[1]["on_click"]

        # Toggle on
        handler()
        assert state.connections_visible["value"] is True
        state.update_scene_now.assert_called()

        # Toggle off
        state.update_scene_now.reset_mock()
        handler()
        assert state.connections_visible["value"] is False

    def test_shutdown_dialog_not_cluster(
        self, mock_ui: MagicMock, robot_dir: Path, monkeypatch
    ):
        """In non-cluster mode, shutdown dialog is created (lines 41-65)."""
        import robot_arm_sim.simulate.app.toolbar as tb_mod

        monkeypatch.setattr(tb_mod, "_CLUSTER_MODE", False)

        state = _make_state(mock_ui, robot_dir)
        build_toolbar = tb_mod.build_toolbar
        build_toolbar(state)

        # The dialog context manager should have been called
        assert mock_ui.dialog.call_count >= 1

    def test_shutdown_dialog_cluster_mode(
        self, mock_ui: MagicMock, robot_dir: Path, monkeypatch
    ):
        """In cluster mode, no stop dialog is created."""
        import robot_arm_sim.simulate.app.toolbar as tb_mod

        monkeypatch.setattr(tb_mod, "_CLUSTER_MODE", True)

        state = _make_state(mock_ui, robot_dir)
        build_toolbar = tb_mod.build_toolbar
        build_toolbar(state)

        # The "Stop Simulator" button should not appear
        stop_calls = [
            c
            for c in mock_ui.button.call_args_list
            if c[0] and c[0][0] == "Stop Simulator"
        ]
        assert len(stop_calls) == 0


def _find_button_call(mock_ui, label):
    """Find a mock_ui.button() call by its first positional arg (label)."""
    for call in mock_ui.button.call_args_list:
        args, kwargs = call
        if args and args[0] == label:
            return (args, kwargs)
        if "text" in kwargs and kwargs["text"] == label:
            return (args, kwargs)
    return None


# ---------------------------------------------------------------------------
# build_visibility_section
# ---------------------------------------------------------------------------


class TestBuildVisibilitySection:
    """Tests for build_visibility_section."""

    def test_builds_without_error(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.toolbar import build_visibility_section

        state = _make_state(mock_ui, robot_dir)

        build_visibility_section(state)

        # Should have called ui.checkbox at least once (per-link checkboxes)
        assert mock_ui.checkbox.call_count > 0

    def test_toggle_all_handler(self, mock_ui: MagicMock, robot_dir: Path):
        """Exercise the set_all(True) handler via the 'All' button."""
        from robot_arm_sim.simulate.app.toolbar import build_visibility_section

        state = _make_state(mock_ui, robot_dir)
        state.update_scene_now = MagicMock()
        state.on_visibility_changed = MagicMock()

        build_visibility_section(state)

        # Find the "None" button to set all invisible
        none_call = _find_button_call(mock_ui, "None")
        assert none_call is not None, "'None' button not found"
        none_handler = none_call[1]["on_click"]

        # Call the "None" handler (a lambda wrapping set_all(False))
        none_handler()

        # All links should be set to False
        for lname in state.chain_link_names:
            assert state.visible_links[lname] is False
        state.update_scene_now.assert_called()
        state.on_visibility_changed.assert_called()

    def test_per_link_vis_handler(self, mock_ui: MagicMock, robot_dir: Path):
        """Exercise the per-link visibility handler."""
        from robot_arm_sim.simulate.app.toolbar import build_visibility_section

        state = _make_state(mock_ui, robot_dir)
        state.update_scene_now = MagicMock()
        state.on_visibility_changed = MagicMock()

        build_visibility_section(state)

        # Find a per-link checkbox call
        per_link_handler = None
        link_name = None
        for call in mock_ui.checkbox.call_args_list:
            args, kwargs = call
            if args and "on_change" in kwargs:
                per_link_handler = kwargs["on_change"]
                display = args[0]
                for lname in state.chain_link_names:
                    lnk = state.robot.get_link(lname)
                    if lnk and lnk.mesh_path and Path(lnk.mesh_path).stem == display:
                        link_name = lname
                        break
                break

        if per_link_handler and link_name:
            event = MagicMock()
            event.value = False
            per_link_handler(event)

            assert state.visible_links[link_name] is False
            state.update_scene_now.assert_called()
            state.on_visibility_changed.assert_called()

    def test_skip_links_without_mesh(self, mock_ui: MagicMock, robot_dir: Path):
        """Links without mesh_path should be skipped."""
        from robot_arm_sim.simulate.app.toolbar import build_visibility_section

        state = _make_state(mock_ui, robot_dir)

        # Add a link with no mesh to chain_link_names
        state.chain_link_names.append("fake_no_mesh_link")
        state.visible_links["fake_no_mesh_link"] = True

        # The robot has no link named "fake_no_mesh_link" so get_link returns None
        build_visibility_section(state)

        # The fake link should NOT have a checkbox entry
        assert "fake_no_mesh_link" not in state.link_checkboxes
