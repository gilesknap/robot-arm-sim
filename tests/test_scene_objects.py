"""Tests for simulate/app/scene_objects.py — 3D scene construction."""

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
    """Patch module-level ui references for scene_objects and its deps."""
    from robot_arm_sim.simulate.app import scene_objects as so_mod
    from robot_arm_sim.simulate.app import scene_update as su_mod
    from robot_arm_sim.simulate.app import view_controls as vc_mod

    mock_ui.run_javascript = MagicMock(return_value=None)
    monkeypatch.setattr(so_mod, "ui", mock_ui)
    monkeypatch.setattr(su_mod, "ui", mock_ui)
    monkeypatch.setattr(vc_mod, "ui", mock_ui)

    # Patch update_scene to be a no-op (tested elsewhere)
    monkeypatch.setattr(so_mod, "update_scene", MagicMock())
    # Patch add_view_controls to be a no-op
    monkeypatch.setattr(so_mod, "add_view_controls", MagicMock())


def _make_state(mock_ui: MagicMock, robot_dir: Path):
    """Build a SimulatorState for scene testing."""
    from robot_arm_sim.simulate.app.state import SimulatorState

    robot = load_urdf(robot_dir / "robot.urdf")
    state = SimulatorState(robot, robot_dir)

    # Set up scene mock with stl method
    scene_mock = MagicMock()
    stl_obj = MagicMock()
    stl_obj.scale = MagicMock(return_value=stl_obj)
    stl_obj.material = MagicMock(return_value=stl_obj)
    stl_obj.id = "mock_id"
    scene_mock.stl = MagicMock(return_value=stl_obj)
    scene_mock.spot_light = MagicMock(return_value=MagicMock())

    # Make ui.scene return a context manager that yields scene_mock
    scene_ctx = MagicMock()
    scene_ctx.__enter__ = MagicMock(return_value=scene_mock)
    scene_ctx.__exit__ = MagicMock(return_value=False)
    mock_ui.scene = MagicMock(return_value=scene_ctx)

    return state


# ---------------------------------------------------------------------------
# build_scene
# ---------------------------------------------------------------------------


class TestBuildScene:
    """Tests for build_scene — the main 3D scene builder."""

    def test_builds_without_error(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_objects import build_scene

        state = _make_state(mock_ui, robot_dir)
        build_scene(state)

        # Scene should be set on state
        assert state.scene is not None

    def test_creates_mesh_objects(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_objects import build_scene

        state = _make_state(mock_ui, robot_dir)
        build_scene(state)

        # Should have mesh objects for links with mesh_path
        links_with_mesh = [lnk for lnk in state.robot.links if lnk.mesh_path]
        assert len(state.mesh_objects) == len(links_with_mesh)

    def test_stl_urls_use_robot_dir_name(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_objects import build_scene

        state = _make_state(mock_ui, robot_dir)

        # Get the scene mock from the ui.scene context manager
        scene_mock = mock_ui.scene.return_value.__enter__.return_value
        build_scene(state)

        # Each stl() call should use the correct URL pattern
        for call in scene_mock.stl.call_args_list:
            url = call[0][0]
            assert url.startswith(f"/stl/{robot_dir.name}/")
            assert url.endswith(".stl")

    def test_camera_positioned(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_objects import build_scene

        state = _make_state(mock_ui, robot_dir)

        scene_mock = mock_ui.scene.return_value.__enter__.return_value
        build_scene(state)

        scene_mock.move_camera.assert_called_once()

    def test_js_initializations_scheduled(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_objects import build_scene

        state = _make_state(mock_ui, robot_dir)
        build_scene(state)

        # Should schedule timers for JS init (POST_INIT, AXES, CONNECTIONS, etc.)
        assert mock_ui.timer.call_count >= 5

    def test_spot_lights_created(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.scene_objects import build_scene

        state = _make_state(mock_ui, robot_dir)

        scene_mock = mock_ui.scene.return_value.__enter__.return_value
        build_scene(state)

        assert scene_mock.spot_light.call_count == 2

    def test_connection_data_built(self, mock_ui: MagicMock, robot_dir: Path):
        """Connection point data should be serialized into JS init."""
        from robot_arm_sim.simulate.app.scene_objects import build_scene

        state = _make_state(mock_ui, robot_dir)
        # Add some connection points
        state.connection_points = {
            "link_0": [{"end": "proximal", "radius_mm": 5.0, "centering": "surface"}]
        }
        build_scene(state)

        # Timer calls should include connection init
        assert mock_ui.timer.call_count >= 5
