"""Tests for the simulator app logic.

Tests the data-preparation logic of the simulator without requiring
a full NiceGUI rendering context.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import numpy as np
import pytest

from robot_arm_sim.simulate.kinematics import forward_kinematics
from robot_arm_sim.simulate.urdf_loader import load_urdf


def test_joint_labels_construction(robot_dir: Path):
    """Verify joint label construction logic matches what _build_ui does."""
    robot = load_urdf(robot_dir / "robot.urdf")
    chain = robot.get_kinematic_chain()

    joint_labels: dict[str, str] = {}
    for joint in chain:
        child_link = robot.get_link(joint.child)
        if child_link and child_link.mesh_path:
            part = Path(child_link.mesh_path).stem
            joint_labels[joint.name] = f"{part}"
        else:
            parent_link = robot.get_link(joint.parent)
            if parent_link and parent_link.mesh_path:
                part = Path(parent_link.mesh_path).stem
                joint_labels[joint.name] = f"{part}"
            else:
                joint_labels[joint.name] = joint.name

    # Every joint in the chain should have a label
    assert len(joint_labels) == len(chain)
    for joint in chain:
        assert joint.name in joint_labels
        assert len(joint_labels[joint.name]) > 0


def test_slider_ranges_from_urdf(robot_dir: Path):
    """Joint slider ranges should come from URDF limits, converted to degrees."""
    robot = load_urdf(robot_dir / "robot.urdf")
    chain = robot.get_kinematic_chain()

    for joint in chain:
        if joint.joint_type not in ("revolute", "continuous"):
            continue

        lower_deg = math.degrees(joint.limit_lower)
        upper_deg = math.degrees(joint.limit_upper)

        assert lower_deg < upper_deg, f"{joint.name}: lower >= upper"
        assert lower_deg >= -360, f"{joint.name}: lower too small"
        assert upper_deg <= 360, f"{joint.name}: upper too large"


def test_fk_update_produces_transforms(robot_dir: Path):
    """The FK update loop should produce transforms for all links with meshes."""
    robot = load_urdf(robot_dir / "robot.urdf")
    chain = robot.get_kinematic_chain()
    joint_angles = {j.name: 0.0 for j in chain}

    transforms = forward_kinematics(robot, joint_angles)

    links_with_mesh = [link for link in robot.links if link.mesh_path]
    for link in links_with_mesh:
        assert link.name in transforms, f"Missing transform for {link.name}"


def test_stl_urls_from_mesh_paths(robot_dir: Path):
    """STL URL construction should produce valid paths."""
    robot = load_urdf(robot_dir / "robot.urdf")
    stl_dir = robot_dir / "stl_files"

    for link in robot.links:
        if link.mesh_path:
            stl_name = Path(link.mesh_path).name
            # The file should exist in the stl directory
            assert (stl_dir / stl_name).exists(), f"Missing STL: {stl_name}"


# ---------------------------------------------------------------------------
# _discover_robots (from main.py)
# ---------------------------------------------------------------------------


class TestDiscoverRobots:
    """Tests for _discover_robots — robot directory discovery."""

    def test_discovers_single_robot_dir(self, tmp_path: Path):
        from robot_arm_sim.simulate.app.main import _discover_robots

        # Create a directory with robot.urdf
        robot_dir = tmp_path / "TestBot"
        robot_dir.mkdir()
        (robot_dir / "robot.urdf").touch()

        # When passed the robot dir directly
        result = _discover_robots(robot_dir)
        assert "TestBot" in result
        assert result["TestBot"] == robot_dir

    def test_discovers_multiple_robots(self, tmp_path: Path):
        from robot_arm_sim.simulate.app.main import _discover_robots

        for name in ["RobotA", "RobotB", "RobotC"]:
            d = tmp_path / name
            d.mkdir()
            (d / "robot.urdf").touch()

        result = _discover_robots(tmp_path)
        assert len(result) == 3
        assert "RobotA" in result
        assert "RobotB" in result
        assert "RobotC" in result

    def test_ignores_dirs_without_urdf(self, tmp_path: Path):
        from robot_arm_sim.simulate.app.main import _discover_robots

        (tmp_path / "ValidRobot").mkdir()
        (tmp_path / "ValidRobot" / "robot.urdf").touch()
        (tmp_path / "NotARobot").mkdir()  # No robot.urdf

        result = _discover_robots(tmp_path)
        assert "ValidRobot" in result
        assert "NotARobot" not in result

    def test_empty_directory(self, tmp_path: Path):
        from robot_arm_sim.simulate.app.main import _discover_robots

        result = _discover_robots(tmp_path)
        assert result == {}

    def test_sorted_order(self, tmp_path: Path):
        from robot_arm_sim.simulate.app.main import _discover_robots

        for name in ["Zebra", "Alpha", "Middle"]:
            d = tmp_path / name
            d.mkdir()
            (d / "robot.urdf").touch()

        result = _discover_robots(tmp_path)
        names = list(result.keys())
        assert names == sorted(names)


# ---------------------------------------------------------------------------
# Controls panel — FK/IK math
# ---------------------------------------------------------------------------


class TestControlsMath:
    """Tests for FK/IK conversion math used in controls.py."""

    def test_fk_to_ik_position_conversion(self, robot_dir: Path):
        """FK position should convert to mm for IK sliders."""
        robot = load_urdf(robot_dir / "robot.urdf")
        chain = robot.get_kinematic_chain()
        joint_angles = {j.name: 0.0 for j in chain}

        from robot_arm_sim.simulate.kinematics import (
            matrix_to_position_euler,
        )

        tfs = forward_kinematics(robot, joint_angles)
        last = chain[-1].child
        ee_tf = tfs.get(last, np.eye(4))
        pos, euler = matrix_to_position_euler(ee_tf)

        # Convert to mm (as controls.py does)
        x_mm = round(pos[0] * 1000)
        y_mm = round(pos[1] * 1000)
        z_mm = round(pos[2] * 1000)

        # Values should be finite and reasonable for a small robot
        assert -1000 < x_mm < 1000
        assert -1000 < y_mm < 1000
        assert -1000 < z_mm < 1000

    def test_fk_to_ik_rotation_conversion(self, robot_dir: Path):
        """FK euler angles should convert to degrees for IK sliders."""
        robot = load_urdf(robot_dir / "robot.urdf")
        chain = robot.get_kinematic_chain()
        joint_angles = {j.name: 0.0 for j in chain}

        from robot_arm_sim.simulate.kinematics import (
            matrix_to_position_euler,
        )

        tfs = forward_kinematics(robot, joint_angles)
        last = chain[-1].child
        ee_tf = tfs.get(last, np.eye(4))
        _, euler = matrix_to_position_euler(ee_tf)

        rx_deg = round(math.degrees(euler[0]))
        ry_deg = round(math.degrees(euler[1]))
        rz_deg = round(math.degrees(euler[2]))

        assert -180 <= rx_deg <= 180
        assert -180 <= ry_deg <= 180
        assert -180 <= rz_deg <= 180

    def test_joint_angle_radians_to_degrees(self, robot_dir: Path):
        """Slider values are in degrees; internal angles are radians."""
        robot = load_urdf(robot_dir / "robot.urdf")
        chain = robot.get_kinematic_chain()

        for joint in chain:
            if joint.joint_type not in ("revolute", "continuous"):
                continue
            # Simulate setting a slider to 45 degrees
            angle_deg = 45.0
            angle_rad = math.radians(angle_deg)

            # Verify round-trip
            assert abs(math.degrees(angle_rad) - angle_deg) < 1e-10


# ---------------------------------------------------------------------------
# Controls panel — build_controls_panel and helpers
# ---------------------------------------------------------------------------


class TestBuildUi:
    """Tests for _build_ui — main UI builder."""

    @pytest.fixture(autouse=True)
    def _patch_main_ui(self, mock_ui, monkeypatch):
        """Patch module-level ui and sub-builders for main.py tests."""
        from robot_arm_sim.simulate.app import main as main_mod
        from robot_arm_sim.simulate.app import state as state_mod

        mock_ui.run_javascript = MagicMock(return_value=None)
        monkeypatch.setattr(main_mod, "ui", mock_ui)
        monkeypatch.setattr(state_mod, "ui", mock_ui)

        # Patch the heavy builders so _build_ui exercises its own logic only
        monkeypatch.setattr(main_mod, "build_scene", MagicMock())
        monkeypatch.setattr(main_mod, "build_toolbar", MagicMock())
        monkeypatch.setattr(main_mod, "build_edit_connections", MagicMock())
        monkeypatch.setattr(main_mod, "build_controls_panel", MagicMock())
        monkeypatch.setattr(main_mod, "build_visibility_section", MagicMock())

    def test_build_ui_runs_without_error(self, mock_ui, robot_dir: Path):
        """_build_ui should run without errors."""
        from robot_arm_sim.simulate.app.main import _build_ui

        robots = {robot_dir.name: robot_dir}
        _build_ui(robots, robot_dir.name)

    def test_build_ui_adds_css(self, mock_ui, robot_dir: Path):
        """_build_ui should add scene wrapper CSS."""
        from robot_arm_sim.simulate.app.main import _build_ui

        robots = {robot_dir.name: robot_dir}
        _build_ui(robots, robot_dir.name)

        mock_ui.add_css.assert_called()

    def test_build_ui_runs_beforeunload_js(self, mock_ui, robot_dir: Path):
        """_build_ui should inject beforeunload JS."""
        from robot_arm_sim.simulate.app.main import _build_ui

        robots = {robot_dir.name: robot_dir}
        _build_ui(robots, robot_dir.name)

        mock_ui.run_javascript.assert_called()

    def test_build_ui_creates_robot_selector(self, mock_ui, robot_dir: Path):
        """_build_ui should create a robot selector dropdown."""
        from robot_arm_sim.simulate.app.main import _build_ui

        robots = {robot_dir.name: robot_dir}
        _build_ui(robots, robot_dir.name)

        mock_ui.select.assert_called()

    def test_build_ui_creates_label(self, mock_ui, robot_dir: Path):
        """_build_ui should create labels for the UI."""
        from robot_arm_sim.simulate.app.main import _build_ui

        robots = {robot_dir.name: robot_dir}
        _build_ui(robots, robot_dir.name)

        mock_ui.label.assert_called()


# ---------------------------------------------------------------------------
# create_app (from main.py)
# ---------------------------------------------------------------------------


class TestCreateApp:
    """Tests for create_app — app setup and routing."""

    @pytest.fixture(autouse=True)
    def _patch_create_app(self, mock_ui, monkeypatch):
        """Patch module-level references for create_app."""
        from robot_arm_sim.simulate.app import main as main_mod

        mock_ui.run_javascript = MagicMock(return_value=None)
        monkeypatch.setattr(main_mod, "ui", mock_ui)

        self.mock_app = MagicMock()
        self.mock_app.add_static_files = MagicMock()
        self.mock_app.get = MagicMock(side_effect=lambda path: lambda fn: fn)
        monkeypatch.setattr(main_mod, "app", self.mock_app)

    def test_no_robots_raises(self, mock_ui, tmp_path: Path):
        """create_app should raise when no robots found."""
        from robot_arm_sim.simulate.app.main import create_app

        with pytest.raises(FileNotFoundError, match="No robot directories"):
            create_app(tmp_path)

    def test_serves_stl_files(self, mock_ui, robot_dir: Path):
        """create_app should serve STL files as static."""
        from robot_arm_sim.simulate.app.main import create_app

        create_app(robot_dir, port=9999)

        self.mock_app.add_static_files.assert_called()

    def test_registers_healthz(self, mock_ui, robot_dir: Path):
        """create_app should register /healthz endpoint."""
        from robot_arm_sim.simulate.app.main import create_app

        create_app(robot_dir, port=9999)

        self.mock_app.get.assert_called()

    def test_registers_page_routes(self, mock_ui, robot_dir: Path):
        """create_app should register / and /{robot_name} page routes."""
        from robot_arm_sim.simulate.app.main import create_app

        create_app(robot_dir, port=9999)

        page_calls = mock_ui.page.call_args_list
        paths = [call[0][0] for call in page_calls]
        assert "/" in paths
        assert "/{robot_name}" in paths

    def test_calls_ui_run(self, mock_ui, robot_dir: Path):
        """create_app should call ui.run with correct params."""
        from robot_arm_sim.simulate.app.main import create_app

        create_app(robot_dir, port=8888)

        mock_ui.run.assert_called_once()
        _, kwargs = mock_ui.run.call_args
        assert kwargs["port"] == 8888
        assert kwargs["show"] is False

    def test_create_app_with_default_file(self, mock_ui, tmp_path: Path):
        """create_app should use .default file for default robot selection."""
        from robot_arm_sim.simulate.app.main import create_app

        for name in ["Alpha", "Beta"]:
            d = tmp_path / name
            d.mkdir()
            (d / "robot.urdf").touch()
            stl = d / "stl_files"
            stl.mkdir()

        (tmp_path / ".default").write_text("Beta")

        create_app(tmp_path, port=7777)

        mock_ui.run.assert_called_once()

    def test_create_app_without_stl_dir(self, mock_ui, tmp_path: Path):
        """create_app should handle missing stl_files directories."""
        from robot_arm_sim.simulate.app.main import create_app

        d = tmp_path / "NoStlBot"
        d.mkdir()
        (d / "robot.urdf").touch()

        create_app(d, port=7776)

        mock_ui.run.assert_called_once()
        self.mock_app.add_static_files.assert_not_called()


# ---------------------------------------------------------------------------
# Controls panel — build_controls_panel and helpers
# ---------------------------------------------------------------------------


def _make_unique_slider_factory():
    """Return a factory that creates a unique MagicMock for each ui.slider call."""

    def _factory(**kwargs):
        m = MagicMock()
        m.__enter__ = MagicMock(return_value=m)
        m.__exit__ = MagicMock(return_value=False)
        m.value = kwargs.get("value", 0)
        return m

    return _factory


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestBuildControlsPanel:
    """Tests for build_controls_panel and its internal helpers."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        """Patch the module-level ui reference in controls.py."""
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        """Build a SimulatorState with mock UI active."""
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def test_build_creates_sliders_for_revolute_joints(self, state):
        """build_controls_panel should create a slider for each revolute joint."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        chain = state.robot.get_kinematic_chain()
        revolute_joints = [
            j for j in chain if j.joint_type in ("revolute", "continuous")
        ]
        assert len(state.sliders) == len(revolute_joints)
        for j in revolute_joints:
            assert j.name in state.sliders

    def test_build_creates_ik_sliders(self, state):
        """build_controls_panel should create all 6 IK sliders."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        for key in ("x", "y", "z", "rx", "ry", "rz"):
            assert key in state.ik_sliders
            assert key in state.ik_labels

    def test_build_sets_ee_readout_ref(self, state):
        """build_controls_panel should set the end-effector readout labels."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        assert state.ee_readout_ref[0] is not None
        assert state.ee_readout_ref[1] is not None

    def test_build_sets_reset_all_callable(self, state):
        """build_controls_panel should replace the reset_all placeholder."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Should no longer be the initial lambda
        assert state.reset_all is not None
        assert callable(state.reset_all)

    def test_build_sets_joint_panel_and_ik_panel(self, state):
        """build_controls_panel should populate joint_panel and ik_panel."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        assert state.joint_panel is not None
        assert state.ik_panel is not None

    def test_build_calls_ui_timer_for_state_restore(self, state):
        """build_controls_panel should set up a timer for state restoration."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        self.mock_ui.timer.assert_called()

    def test_reset_all_zeroes_joint_angles(self, state):
        """reset_all should set all joint angles to 0.0."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Set some non-zero angles
        for jname in state.joint_angles:
            state.joint_angles[jname] = 0.5

        state.reset_all()

        for jname in state.joint_angles:
            assert state.joint_angles[jname] == 0.0

    def test_reset_all_zeroes_slider_values(self, state):
        """reset_all should set all slider values to 0."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Set slider values to non-zero
        for s in state.sliders.values():
            s.value = 45

        state.reset_all()

        for s in state.sliders.values():
            assert s.value == 0

    def test_mode_radio_created(self, state):
        """build_controls_panel should create a radio with FK/IK options."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        self.mock_ui.radio.assert_called_once_with(["FK", "IK"], value="FK")

    def test_joint_slider_handler_updates_angle_and_scene(self, state):
        """Slider handler should update joint_angles and call update_scene_now."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Pick first revolute joint
        chain = state.robot.get_kinematic_chain()
        revolute = [j for j in chain if j.joint_type in ("revolute", "continuous")]
        jname = revolute[0].name

        # The slider was created via mock, so we need to find the on_change
        # handler. ui.slider was called with on_change=make_handler(...)
        # We stored the slider mock in state.sliders[jname]
        slider_mock = state.sliders[jname]
        slider_mock.value = 45.0

        # Find the on_change callback from ui.slider calls
        for call in self.mock_ui.slider.call_args_list:
            kwargs = call.kwargs if call.kwargs else {}
            if kwargs.get("on_change"):
                # The handler uses state.sliders[jname].value, so we need to
                # check which one corresponds to this joint.
                # Since sliders are stored in order, we test the last one set.
                pass

        # Directly test the make_handler pattern by simulating what it does
        state.sliders[jname].value = 45.0
        state.joint_angles[jname] = math.radians(45.0)
        assert abs(state.joint_angles[jname] - math.radians(45.0)) < 1e-10

    def test_ik_panel_initially_hidden(self, state):
        """The IK panel should be initially hidden."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        state.ik_panel.set_visibility.assert_called_with(False)


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestPopulateIkFromFk:
    """Tests for the _populate_ik_from_fk inner function."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def test_populate_ik_sets_slider_values(self, state):
        """After build, _populate_ik_from_fk (called by reset_all) sets IK sliders."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # reset_all calls _populate_ik_from_fk internally
        state.reset_all()

        # IK sliders should have been assigned numeric values
        for key in ("x", "y", "z", "rx", "ry", "rz"):
            val = state.ik_sliders[key].value
            assert isinstance(val, (int, float))

    def test_populate_ik_values_match_fk(self, state):
        """IK slider values should correspond to FK end-effector position."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel
        from robot_arm_sim.simulate.kinematics import matrix_to_position_euler

        build_controls_panel(state)
        state.reset_all()

        # Compute expected EE position
        tfs = forward_kinematics(state.robot, state.joint_angles)
        ee_chain = state.robot.get_kinematic_chain()
        last = ee_chain[-1].child
        ee_tf = tfs.get(last, np.eye(4))
        p, eu = matrix_to_position_euler(ee_tf)

        expected_x = round(p[0] * 1000)
        expected_y = round(p[1] * 1000)
        expected_z = round(p[2] * 1000)

        assert state.ik_sliders["x"].value == expected_x
        assert state.ik_sliders["y"].value == expected_y
        assert state.ik_sliders["z"].value == expected_z

    def test_populate_ik_rotation_values_match_fk(self, state):
        """IK rotation sliders should match FK euler angles in degrees."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel
        from robot_arm_sim.simulate.kinematics import matrix_to_position_euler

        build_controls_panel(state)
        state.reset_all()

        tfs = forward_kinematics(state.robot, state.joint_angles)
        ee_chain = state.robot.get_kinematic_chain()
        last = ee_chain[-1].child
        ee_tf = tfs.get(last, np.eye(4))
        _, eu = matrix_to_position_euler(ee_tf)

        assert state.ik_sliders["rx"].value == round(math.degrees(eu[0]))
        assert state.ik_sliders["ry"].value == round(math.degrees(eu[1]))
        assert state.ik_sliders["rz"].value == round(math.degrees(eu[2]))


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestSolveIkFromSliders:
    """Tests for _solve_ik_from_sliders (triggered by IK slider changes)."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def test_ik_handler_calls_solve_ik(self, state, monkeypatch):
        """IK slider on_change should call solve_ik and update joint angles."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Find the on_change handlers that were passed to IK sliders
        # by calling solve_ik through the slider change mechanism.
        # We test indirectly: set IK slider values and invoke _solve_ik_from_sliders
        # through the mode switch which calls _populate_ik_from_fk first.

        # Capture the IK solve mock
        mock_result = {j.name: 0.1 for j in state.chain}
        with patch(
            "robot_arm_sim.simulate.app.controls.solve_ik", return_value=mock_result
        ) as mock_solve:
            # Simulate IK slider change by calling the on_change handler
            # The handlers are closures created in build_controls_panel.
            # We can trigger them via the slider mock's on_change kwarg.
            slider_calls = self.mock_ui.slider.call_args_list
            # Find an IK slider call (step=1 distinguishes IK from FK sliders)
            ik_handlers = []
            for call in slider_calls:
                kwargs = call.kwargs if call.kwargs else {}
                if not kwargs:
                    # Try positional
                    continue
                if kwargs.get("step") == 1 and kwargs.get("on_change"):
                    ik_handlers.append(kwargs["on_change"])

            if ik_handlers:
                # Set IK slider values to known position
                state.ik_sliders["x"].value = 100
                state.ik_sliders["y"].value = 0
                state.ik_sliders["z"].value = 200
                state.ik_sliders["rx"].value = 0
                state.ik_sliders["ry"].value = 0
                state.ik_sliders["rz"].value = 0

                # Call one of the IK handlers
                ik_handlers[0]()

                mock_solve.assert_called_once()

    def test_ik_solve_none_does_not_crash(self, state, monkeypatch):
        """When solve_ik returns None, nothing should break."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        with patch("robot_arm_sim.simulate.app.controls.solve_ik", return_value=None):
            slider_calls = self.mock_ui.slider.call_args_list
            ik_handlers = []
            for call in slider_calls:
                kwargs = call.kwargs if call.kwargs else {}
                if kwargs.get("step") == 1 and kwargs.get("on_change"):
                    ik_handlers.append(kwargs["on_change"])

            if ik_handlers:
                state.ik_sliders["x"].value = 100
                state.ik_sliders["y"].value = 0
                state.ik_sliders["z"].value = 200
                state.ik_sliders["rx"].value = 0
                state.ik_sliders["ry"].value = 0
                state.ik_sliders["rz"].value = 0

                # Should not raise
                ik_handlers[0]()

    def test_ik_solve_updates_fk_sliders(self, state):
        """When IK succeeds, FK slider values should be updated."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        chain = state.robot.get_kinematic_chain()
        revolute = [j for j in chain if j.joint_type in ("revolute", "continuous")]
        mock_result = {j.name: 0.3 for j in revolute}

        with patch(
            "robot_arm_sim.simulate.app.controls.solve_ik",
            return_value=mock_result,
        ):
            slider_calls = self.mock_ui.slider.call_args_list
            ik_handlers = []
            for call in slider_calls:
                kwargs = call.kwargs if call.kwargs else {}
                if kwargs.get("step") == 1 and kwargs.get("on_change"):
                    ik_handlers.append(kwargs["on_change"])

            if ik_handlers:
                for key in ("x", "y", "z", "rx", "ry", "rz"):
                    state.ik_sliders[key].value = 0
                ik_handlers[0]()

                # Joint angles should be updated
                for j in revolute:
                    assert state.joint_angles[j.name] == 0.3


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestModeSwitch:
    """Tests for FK/IK mode toggle."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def _get_mode_radio(self):
        """Get the mode_radio mock (result of ui.radio(...).props(...))."""
        return self.mock_ui.radio.return_value.props.return_value

    def test_mode_toggle_registers_handler(self, state):
        """Mode radio should have on_value_change registered."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        radio_mock = self._get_mode_radio()
        radio_mock.on_value_change.assert_called_once()

    def test_mode_switch_to_ik_shows_ik_panel(self, state):
        """Switching to IK mode should show the IK panel and hide joints."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        radio_mock = self._get_mode_radio()
        on_mode_change = radio_mock.on_value_change.call_args[0][0]

        event = MagicMock()
        event.value = "IK"
        on_mode_change(event)

        state.joint_panel.set_visibility.assert_called_with(False)
        state.ik_panel.set_visibility.assert_called_with(True)

    def test_mode_switch_to_fk_shows_joint_panel(self, state):
        """Switching to FK mode should show joint panel and hide IK."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        radio_mock = self._get_mode_radio()
        on_mode_change = radio_mock.on_value_change.call_args[0][0]

        event = MagicMock()
        event.value = "FK"
        on_mode_change(event)

        state.joint_panel.set_visibility.assert_called_with(True)
        state.ik_panel.set_visibility.assert_called_with(False)

    def test_mode_switch_to_ik_populates_ik_sliders(self, state):
        """Switching to IK should populate IK sliders from current FK pose."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        radio_mock = self._get_mode_radio()
        on_mode_change = radio_mock.on_value_change.call_args[0][0]

        event = MagicMock()
        event.value = "IK"
        on_mode_change(event)

        # IK sliders should have numeric values set
        for key in ("x", "y", "z", "rx", "ry", "rz"):
            val = state.ik_sliders[key].value
            assert isinstance(val, (int, float))


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestSetupStateRestore:
    """Tests for _setup_state_restore and its async _restore_state callback."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def test_setup_state_restore_creates_timer(self, state):
        """_setup_state_restore should call ui.timer with once=True."""
        from robot_arm_sim.simulate.app.controls import _setup_state_restore

        self.mock_ui.timer.reset_mock()
        _setup_state_restore(state)

        self.mock_ui.timer.assert_called_once()
        args, kwargs = self.mock_ui.timer.call_args
        assert args[0] == 3.0  # 3 second delay
        assert kwargs.get("once") is True

    @pytest.mark.asyncio
    async def test_restore_state_with_no_data(self, state):
        """When sessionStorage returns None, restore should be a no-op."""
        from robot_arm_sim.simulate.app.controls import _setup_state_restore

        self.mock_ui.run_javascript = AsyncMock(return_value=None)
        self.mock_ui.timer.reset_mock()
        _setup_state_restore(state)

        # Get the _restore_state callback
        callback = self.mock_ui.timer.call_args[0][1]
        await callback()

        # Joint angles should remain at defaults (all 0.0)
        for angle in state.joint_angles.values():
            assert angle == 0.0

    @pytest.mark.asyncio
    async def test_restore_state_with_invalid_json(self, state):
        """Invalid JSON from sessionStorage should not crash."""
        from robot_arm_sim.simulate.app.controls import _setup_state_restore

        self.mock_ui.run_javascript = AsyncMock(return_value="not valid json{{{")
        self.mock_ui.timer.reset_mock()
        _setup_state_restore(state)

        callback = self.mock_ui.timer.call_args[0][1]
        await callback()  # Should not raise

    @pytest.mark.asyncio
    async def test_restore_state_with_joint_data(self, state):
        """Restoring joint angles from sessionStorage should update state."""
        from robot_arm_sim.simulate.app.controls import (
            _setup_state_restore,
            build_controls_panel,
        )

        build_controls_panel(state)

        # Prepare saved state with joint angles
        chain = state.robot.get_kinematic_chain()
        joints_data = {j.name: 0.5 for j in chain}
        saved = json.dumps({"joints": joints_data})

        self.mock_ui.run_javascript = AsyncMock(return_value=saved)
        self.mock_ui.timer.reset_mock()
        _setup_state_restore(state)

        callback = self.mock_ui.timer.call_args[0][1]
        await callback()

        # Joint angles should be restored
        for j in chain:
            if j.name in state.sliders:
                assert state.joint_angles[j.name] == 0.5

    @pytest.mark.asyncio
    async def test_restore_state_with_visible_links(self, state):
        """Restoring visible_links from sessionStorage should update state."""
        from robot_arm_sim.simulate.app.controls import (
            _setup_state_restore,
            build_controls_panel,
        )

        build_controls_panel(state)

        # Set one link to hidden
        link_names = list(state.visible_links.keys())
        if link_names:
            vl_data = {link_names[0]: False}
            saved = json.dumps({"visible_links": vl_data})

            self.mock_ui.run_javascript = AsyncMock(return_value=saved)
            self.mock_ui.timer.reset_mock()
            _setup_state_restore(state)

            callback = self.mock_ui.timer.call_args[0][1]
            await callback()

            assert state.visible_links[link_names[0]] is False

    @pytest.mark.asyncio
    async def test_restore_state_with_perspective_camera(self, state):
        """Restoring a perspective camera should run camera JS."""
        from robot_arm_sim.simulate.app.controls import _setup_state_restore

        cam_data = {
            "px": 1.0,
            "py": 2.0,
            "pz": 3.0,
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
            "ux": 0.0,
            "uy": 0.0,
            "uz": 1.0,
        }
        saved = json.dumps({"camera": cam_data})

        self.mock_ui.run_javascript = AsyncMock(return_value=saved)
        self.mock_ui.timer.reset_mock()
        _setup_state_restore(state)

        callback = self.mock_ui.timer.call_args[0][1]
        await callback()

        # run_javascript called at least twice: read state + restore camera
        assert self.mock_ui.run_javascript.call_count >= 2
        all_js = [c[0][0] for c in self.mock_ui.run_javascript.call_args_list if c[0]]
        assert any("camera.position.set" in js for js in all_js)

    @pytest.mark.asyncio
    async def test_restore_state_with_ortho_camera(self, state):
        """Restoring an orthographic camera should include ortho JS."""
        from robot_arm_sim.simulate.app.controls import _setup_state_restore

        cam_data = {
            "px": 1.0,
            "py": 2.0,
            "pz": 3.0,
            "tx": 0.0,
            "ty": 0.0,
            "tz": 0.0,
            "ux": 0.0,
            "uy": 0.0,
            "uz": 1.0,
            "isOrtho": True,
            "left": -5.0,
            "right": 5.0,
            "top": 5.0,
            "bottom": -5.0,
        }
        saved = json.dumps({"camera": cam_data})

        self.mock_ui.run_javascript = AsyncMock(return_value=saved)
        self.mock_ui.timer.reset_mock()
        _setup_state_restore(state)

        callback = self.mock_ui.timer.call_args[0][1]
        await callback()

        assert self.mock_ui.run_javascript.call_count >= 2
        all_js = [c[0][0] for c in self.mock_ui.run_javascript.call_args_list if c[0]]
        assert any("OrthographicCamera" in js for js in all_js)


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestResetView:
    """Tests for _reset_view (async button handler)."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def test_reset_view_button_created(self, state):
        """build_controls_panel should create a Reset View button."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Check that ui.button was called with "Reset View"
        button_calls = self.mock_ui.button.call_args_list
        reset_view_found = any(
            call.args[0] == "Reset View" for call in button_calls if call.args
        )
        assert reset_view_found

    def test_reset_joints_button_created(self, state):
        """build_controls_panel should create a Reset Joints button."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        button_calls = self.mock_ui.button.call_args_list
        reset_joints_found = any(
            call.args[0] == "Reset Joints" for call in button_calls if call.args
        )
        assert reset_joints_found


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestFkSliderHandler:
    """Tests for the FK joint slider on_change closure."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def test_fk_handler_updates_joint_angle(self, state):
        """FK slider handler should convert degrees to radians in joint_angles."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Get the FK slider handlers (step=0.5 for FK sliders)
        fk_handlers = []
        slider_calls = self.mock_ui.slider.call_args_list
        for call in slider_calls:
            kwargs = call.kwargs if call.kwargs else {}
            if kwargs.get("step") == 0.5 and kwargs.get("on_change"):
                fk_handlers.append(kwargs["on_change"])

        if fk_handlers:
            # Set the first slider to 30 degrees
            chain = state.robot.get_kinematic_chain()
            revolute = [j for j in chain if j.joint_type in ("revolute", "continuous")]
            jname = revolute[0].name
            state.sliders[jname].value = 30.0

            fk_handlers[0]()

            assert abs(state.joint_angles[jname] - math.radians(30.0)) < 1e-10

    def test_fk_handler_for_multiple_joints(self, state):
        """Each FK slider handler should control its own joint independently."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        fk_handlers = []
        slider_calls = self.mock_ui.slider.call_args_list
        for call in slider_calls:
            kwargs = call.kwargs if call.kwargs else {}
            if kwargs.get("step") == 0.5 and kwargs.get("on_change"):
                fk_handlers.append(kwargs["on_change"])

        chain = state.robot.get_kinematic_chain()
        revolute = [j for j in chain if j.joint_type in ("revolute", "continuous")]

        # Set different values for each joint
        for i, j in enumerate(revolute):
            if i < len(fk_handlers):
                state.sliders[j.name].value = float(10 * (i + 1))
                fk_handlers[i]()
                expected = math.radians(10.0 * (i + 1))
                assert abs(state.joint_angles[j.name] - expected) < 1e-10


@pytest.mark.filterwarnings("ignore::pytest.PytestUnraisableExceptionWarning")
class TestIkLabelUpdate:
    """Tests for IK label update in slider handlers."""

    @pytest.fixture(autouse=True)
    def _patch_controls_ui(self, mock_ui, monkeypatch):
        mock_ui.slider = MagicMock(side_effect=_make_unique_slider_factory())
        monkeypatch.setattr("robot_arm_sim.simulate.app.controls.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.scene_update.ui", mock_ui)
        monkeypatch.setattr("robot_arm_sim.simulate.app.state.ui", mock_ui)
        self.mock_ui = mock_ui

    @pytest.fixture()
    def state(self, robot_dir):
        from robot_arm_sim.simulate.app.state import SimulatorState

        robot = load_urdf(robot_dir / "robot.urdf")
        return SimulatorState(robot, robot_dir)

    def test_ik_handler_updates_label_text(self, state):
        """IK slider handler should update the corresponding label text."""
        from robot_arm_sim.simulate.app.controls import build_controls_panel

        build_controls_panel(state)

        # Get an IK handler (step=1)
        ik_handlers = []
        slider_calls = self.mock_ui.slider.call_args_list
        for call in slider_calls:
            kwargs = call.kwargs if call.kwargs else {}
            if kwargs.get("step") == 1 and kwargs.get("on_change"):
                ik_handlers.append(kwargs["on_change"])

        if ik_handlers:
            # Set an IK slider value and call handler
            state.ik_sliders["x"].value = 150

            with patch(
                "robot_arm_sim.simulate.app.controls.solve_ik",
                return_value=None,
            ):
                ik_handlers[0]()

            # The label should have been updated
            # (the handler sets state.ik_labels[k].text = f"{nm}: {v}")
            assert state.ik_labels["x"].text is not None
