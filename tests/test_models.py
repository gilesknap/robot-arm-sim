"""Tests for robot model dataclasses."""

from __future__ import annotations

from robot_arm_sim.models.robot import URDFJoint, URDFLink, URDFRobot


def _make_robot() -> URDFRobot:
    links = [
        URDFLink(name="base"),
        URDFLink(name="link1", mesh_path="stl_files/L1.stl"),
        URDFLink(name="link2", mesh_path="stl_files/L2.stl"),
    ]
    joints = [
        URDFJoint(name="j1", joint_type="revolute", parent="base", child="link1"),
        URDFJoint(name="j2", joint_type="revolute", parent="link1", child="link2"),
    ]
    return URDFRobot(name="test_robot", links=links, joints=joints)


def test_get_kinematic_chain():
    robot = _make_robot()
    chain = robot.get_kinematic_chain()
    assert len(chain) == 2
    assert chain[0].name == "j1"
    assert chain[1].name == "j2"


def test_get_kinematic_chain_order():
    """Chain should go base→tip regardless of joint insertion order."""
    links = [URDFLink(name=n) for n in ["base", "l1", "l2", "l3"]]
    # Insert joints in reverse order
    joints = [
        URDFJoint(name="j3", joint_type="revolute", parent="l2", child="l3"),
        URDFJoint(name="j1", joint_type="revolute", parent="base", child="l1"),
        URDFJoint(name="j2", joint_type="revolute", parent="l1", child="l2"),
    ]
    robot = URDFRobot(name="test", links=links, joints=joints)
    chain = robot.get_kinematic_chain()
    assert [j.name for j in chain] == ["j1", "j2", "j3"]


def test_get_link():
    robot = _make_robot()
    link = robot.get_link("base")
    assert link is not None
    assert link.name == "base"
    assert robot.get_link("nonexistent") is None


def test_get_joint():
    robot = _make_robot()
    joint = robot.get_joint("j1")
    assert joint is not None
    assert joint.joint_type == "revolute"
    assert robot.get_joint("nonexistent") is None


def test_link_defaults():
    link = URDFLink(name="test")
    assert link.mesh_path is None
    assert link.mesh_scale == [0.001, 0.001, 0.001]
    assert link.origin_xyz == [0.0, 0.0, 0.0]
    assert link.origin_rpy == [0.0, 0.0, 0.0]
