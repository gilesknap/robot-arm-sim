"""URDF XML parser — loads robot model into dataclasses."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from robot_arm_sim.models.robot import URDFJoint, URDFLink, URDFRobot


def load_urdf(urdf_path: Path) -> URDFRobot:
    """Parse a URDF file and return a URDFRobot."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    robot_name = root.attrib.get("name", "robot")
    links = []
    joints = []

    for link_elem in root.findall("link"):
        link = _parse_link(link_elem)
        links.append(link)

    for joint_elem in root.findall("joint"):
        joint = _parse_joint(joint_elem)
        joints.append(joint)

    return URDFRobot(name=robot_name, links=links, joints=joints)


def validate_urdf(robot_dir: Path) -> list[str]:
    """Validate a URDF file in a robot directory. Returns list of errors."""
    urdf_path = robot_dir / "robot.urdf"
    errors = []

    if not urdf_path.exists():
        return [f"URDF file not found: {urdf_path}"]

    try:
        robot = load_urdf(urdf_path)
    except ET.ParseError as e:
        return [f"XML parse error: {e}"]

    # Check all joint parent/child links exist
    link_names = {link.name for link in robot.links}
    for joint in robot.joints:
        if joint.parent not in link_names:
            errors.append(
                f"Joint '{joint.name}' references unknown parent link '{joint.parent}'"
            )
        if joint.child not in link_names:
            errors.append(
                f"Joint '{joint.name}' references unknown child link '{joint.child}'"
            )

    # Check mesh file references
    stl_dir = robot_dir
    for link in robot.links:
        if link.mesh_path:
            mesh_file = stl_dir / link.mesh_path
            if not mesh_file.exists():
                errors.append(
                    f"Link '{link.name}' references missing mesh: {link.mesh_path}"
                )

    # Check kinematic tree connectivity
    if robot.joints:
        child_names = {j.child for j in robot.joints}
        parent_names = {j.parent for j in robot.joints}
        roots = parent_names - child_names
        if len(roots) != 1:
            errors.append(f"Expected 1 root link, found {len(roots)}: {roots}")

    return errors


def _parse_link(elem: ET.Element) -> URDFLink:
    """Parse a <link> element."""
    name = elem.attrib["name"]
    mesh_path = None
    mesh_scale = [0.001, 0.001, 0.001]
    origin_xyz = [0.0, 0.0, 0.0]
    origin_rpy = [0.0, 0.0, 0.0]

    visual = elem.find("visual")
    if visual is not None:
        origin = visual.find("origin")
        if origin is not None:
            origin_xyz = _parse_floats(origin.attrib.get("xyz", "0 0 0"))
            origin_rpy = _parse_floats(origin.attrib.get("rpy", "0 0 0"))

        geometry = visual.find("geometry")
        if geometry is not None:
            mesh = geometry.find("mesh")
            if mesh is not None:
                mesh_path = mesh.attrib.get("filename")
                scale_str = mesh.attrib.get("scale")
                if scale_str:
                    mesh_scale = _parse_floats(scale_str)

    return URDFLink(
        name=name,
        mesh_path=mesh_path,
        mesh_scale=mesh_scale,
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
    )


def _parse_joint(elem: ET.Element) -> URDFJoint:
    """Parse a <joint> element."""
    name = elem.attrib["name"]
    joint_type = elem.attrib.get("type", "fixed")

    parent = elem.find("parent")
    child = elem.find("child")
    parent_name = parent.attrib["link"] if parent is not None else ""
    child_name = child.attrib["link"] if child is not None else ""

    origin_xyz = [0.0, 0.0, 0.0]
    origin_rpy = [0.0, 0.0, 0.0]
    origin = elem.find("origin")
    if origin is not None:
        origin_xyz = _parse_floats(origin.attrib.get("xyz", "0 0 0"))
        origin_rpy = _parse_floats(origin.attrib.get("rpy", "0 0 0"))

    axis = [0.0, 0.0, 1.0]
    axis_elem = elem.find("axis")
    if axis_elem is not None:
        axis = _parse_floats(axis_elem.attrib.get("xyz", "0 0 1"))

    limit_lower = -3.14159
    limit_upper = 3.14159
    limit_effort = 100.0
    limit_velocity = 1.0
    limit_elem = elem.find("limit")
    if limit_elem is not None:
        limit_lower = float(limit_elem.attrib.get("lower", limit_lower))
        limit_upper = float(limit_elem.attrib.get("upper", limit_upper))
        limit_effort = float(limit_elem.attrib.get("effort", limit_effort))
        limit_velocity = float(limit_elem.attrib.get("velocity", limit_velocity))

    return URDFJoint(
        name=name,
        joint_type=joint_type,
        parent=parent_name,
        child=child_name,
        axis=axis,
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
        limit_lower=limit_lower,
        limit_upper=limit_upper,
        limit_effort=limit_effort,
        limit_velocity=limit_velocity,
    )


def _parse_floats(s: str) -> list[float]:
    """Parse a space-separated string of floats."""
    return [float(x) for x in s.split()]
