"""Tests for URDF loading and validation."""

from __future__ import annotations

from pathlib import Path

from robot_arm_sim.simulate.urdf_loader import load_urdf, validate_urdf


def test_load_urdf_parses_all_links(robot_dir: Path):
    robot = load_urdf(robot_dir / "robot.urdf")
    assert len(robot.links) >= 2
    assert all(link.name for link in robot.links)


def test_load_urdf_parses_all_joints(robot_dir: Path):
    robot = load_urdf(robot_dir / "robot.urdf")
    assert len(robot.joints) >= 1
    for joint in robot.joints:
        assert joint.name
        assert joint.parent
        assert joint.child
        assert joint.joint_type in ("revolute", "prismatic", "fixed", "continuous")


def test_load_urdf_joint_limits(robot_dir: Path):
    robot = load_urdf(robot_dir / "robot.urdf")
    revolute_joints = [j for j in robot.joints if j.joint_type == "revolute"]
    assert len(revolute_joints) >= 1
    # At least one joint should have non-default limits (not exactly ±pi)
    import math

    has_custom_limits = any(
        abs(j.limit_lower - (-math.pi)) > 0.01 or abs(j.limit_upper - math.pi) > 0.01
        for j in revolute_joints
    )
    assert has_custom_limits, "Expected at least one joint with non-default limits"


def test_load_urdf_mesh_paths(robot_dir: Path):
    robot = load_urdf(robot_dir / "robot.urdf")
    links_with_mesh = [link for link in robot.links if link.mesh_path]
    assert len(links_with_mesh) >= 1
    for link in links_with_mesh:
        mesh_file = robot_dir / link.mesh_path
        assert mesh_file.exists(), f"Mesh file missing: {link.mesh_path}"


def test_validate_urdf_no_errors(robot_dir: Path):
    errors = validate_urdf(robot_dir)
    assert errors == []


def test_validate_urdf_missing_file(tmp_path: Path):
    errors = validate_urdf(tmp_path)
    assert len(errors) == 1
    assert "not found" in errors[0].lower()


def test_validate_urdf_broken_mesh_ref(robot_dir: Path):
    # Break a mesh reference in the URDF
    urdf_path = robot_dir / "robot.urdf"
    content = urdf_path.read_text()
    content = content.replace("A0.stl", "NONEXISTENT.stl", 1)
    urdf_path.write_text(content)

    errors = validate_urdf(robot_dir)
    assert any("NONEXISTENT" in e or "missing" in e.lower() for e in errors)
