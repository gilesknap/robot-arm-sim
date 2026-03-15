"""Tests for simulate/app/state.py — SimulatorState initialization and methods."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import pytest

from robot_arm_sim.simulate.urdf_loader import load_urdf

# ---------------------------------------------------------------------------
# SimulatorState construction
# ---------------------------------------------------------------------------


class TestSimulatorStateInit:
    """Test SimulatorState constructor with real robot data."""

    def test_basic_attributes(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        assert state.robot is robot
        assert state.robot_dir == robot_dir

    def test_joint_angles_initialized_to_zero(
        self, mock_ui: MagicMock, robot_dir: Path
    ):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        assert len(state.joint_angles) > 0
        for jname, angle in state.joint_angles.items():
            assert angle == 0.0, f"Joint {jname} should start at 0"

    def test_chain_matches_robot_chain(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        expected_chain = robot.get_kinematic_chain()
        assert len(state.chain) == len(expected_chain)
        for s, e in zip(state.chain, expected_chain, strict=True):
            assert s.name == e.name

    def test_chain_link_names_ordered(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        # Should contain unique link names from the chain
        assert len(state.chain_link_names) == len(set(state.chain_link_names))
        # Every link in the chain should appear
        chain = robot.get_kinematic_chain()
        for joint in chain:
            assert joint.parent in state.chain_link_names
            assert joint.child in state.chain_link_names

    def test_visible_links_all_true(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        for lname, vis in state.visible_links.items():
            assert vis is True, f"Link {lname} should be visible initially"

    def test_toggle_states_default_false(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        assert state.labels_visible["value"] is False
        assert state.frames_visible["value"] is False
        assert state.connections_visible["value"] is False
        assert state.show_all_connections["value"] is False
        assert state.transparent_mode["value"] is False
        assert state.edit_connections_active["value"] is False

    def test_mesh_centers_loaded(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        # Should have loaded mesh centers from analysis files
        assert isinstance(state.mesh_centers, dict)
        assert len(state.mesh_centers) > 0

    def test_connection_points_loaded(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        assert isinstance(state.connection_points, dict)
        assert len(state.connection_points) > 0

    def test_label_metadata_built(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        assert len(state.label_metadata) > 0
        # Should have both link and joint labels
        has_link = any(not m["is_joint"] for m in state.label_metadata)
        has_joint = any(m["is_joint"] for m in state.label_metadata)
        assert has_link
        assert has_joint

        for meta in state.label_metadata:
            assert "id" in meta
            assert "name" in meta
            assert "is_joint" in meta


# ---------------------------------------------------------------------------
# _build_joint_labels
# ---------------------------------------------------------------------------


class TestBuildJointLabels:
    """Test _build_joint_labels produces correct labels."""

    def test_every_joint_has_label(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        chain = robot.get_kinematic_chain()
        assert len(state.joint_labels) == len(chain)
        for joint in chain:
            assert joint.name in state.joint_labels
            assert len(state.joint_labels[joint.name]) > 0

    def test_labels_are_part_names(self, mock_ui: MagicMock, robot_dir: Path):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        # Labels should be mesh stem names (not raw joint names)
        for joint in state.chain:
            label = state.joint_labels[joint.name]
            child_link = robot.get_link(joint.child)
            if child_link and child_link.mesh_path:
                expected = Path(child_link.mesh_path).stem
                assert label == expected


# ---------------------------------------------------------------------------
# reload_urdf
# ---------------------------------------------------------------------------


class TestReloadUrdf:
    """Test reload_urdf async method."""

    @pytest.mark.asyncio
    async def test_reload_calls_javascript_and_navigate(
        self, mock_ui: MagicMock, robot_dir: Path, monkeypatch: pytest.MonkeyPatch
    ):
        from robot_arm_sim.simulate.app import state as state_mod
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        st = SimulatorState(robot, robot_dir)

        # Patch the module-level ui reference used in reload_urdf
        mock_run_js = AsyncMock(return_value=None)
        mock_navigate = MagicMock()
        monkeypatch.setattr(
            state_mod,
            "ui",
            MagicMock(
                run_javascript=mock_run_js,
                navigate=mock_navigate,
            ),
        )

        await st.reload_urdf()

        mock_run_js.assert_awaited()
        mock_navigate.to.assert_called_once_with(f"/{robot_dir.name}")


# ---------------------------------------------------------------------------
# update_scene_now
# ---------------------------------------------------------------------------


class TestUpdateSceneNow:
    """Test update_scene_now delegates to scene_update."""

    def test_delegates_to_update_scene(
        self, mock_ui: MagicMock, robot_dir: Path, monkeypatch: pytest.MonkeyPatch
    ):
        from robot_arm_sim.simulate.app import scene_update as su_mod
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        state = SimulatorState(robot, robot_dir)

        # Patch scene_update's module-level ui with a plain MagicMock
        # (run_javascript is called synchronously, so AsyncMock would
        # produce unawaited coroutine warnings)
        scene_ui = MagicMock()
        scene_ui.run_javascript = MagicMock(return_value=None)
        monkeypatch.setattr(su_mod, "ui", scene_ui)

        # Add mock mesh objects so update_scene can operate
        for link in robot.links:
            if link.mesh_path:
                obj = MagicMock()
                state.mesh_objects[link.name] = obj

        # Should not raise
        state.update_scene_now()

        # Mesh objects should have had move() called
        for obj in state.mesh_objects.values():
            obj.move.assert_called()
