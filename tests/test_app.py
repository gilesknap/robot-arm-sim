"""Tests for the simulator app logic.

Tests the data-preparation logic of the simulator without requiring
a full NiceGUI rendering context.
"""

from __future__ import annotations

import math
from pathlib import Path

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
