"""URDF generation from kinematic chain spec + analysis data.

Reads a chain.yaml (kinematic topology) and per-part analysis YAMLs
(with connection_points), then computes exact joint/visual transforms
and writes a URDF file.
"""

from __future__ import annotations

import logging
import xml.dom.minidom
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

logger = logging.getLogger(__name__)


def generate_urdf(
    chain_path: Path,
    analysis_dir: Path,
    stl_dir: Path,
    output_path: Path,
) -> list[str]:
    """Generate URDF from chain spec + analysis data.

    Returns a list of warning/info messages.
    """
    messages: list[str] = []

    with open(chain_path) as f:
        chain = yaml.safe_load(f)

    # Load analysis data for each mesh
    analyses = {}
    for link in chain["links"]:
        mesh_name = link.get("mesh")
        if mesh_name:
            yaml_path = analysis_dir / f"{mesh_name}.yaml"
            if yaml_path.exists():
                with open(yaml_path) as f:
                    analyses[mesh_name] = yaml.safe_load(f)
            else:
                messages.append(f"WARNING: No analysis YAML for {mesh_name}")

    robot_name = chain.get("robot_name", "robot")
    robot = ET.Element("robot", name=robot_name)

    link_specs = {lk["name"]: lk for lk in chain["links"]}

    # Generate links
    for link_spec in chain["links"]:
        link_name = link_spec["name"]
        mesh_name = link_spec.get("mesh")

        link_el = ET.SubElement(robot, "link", name=link_name)

        if mesh_name and mesh_name in analyses:
            analysis = analyses[mesh_name]
            visual = ET.SubElement(link_el, "visual")

            viz_xyz, viz_rpy = _compute_visual_origin(
                analysis,
                link_spec,
                link_name,
                messages=messages,
            )

            if viz_xyz != [0, 0, 0] or viz_rpy != [0, 0, 0]:
                ET.SubElement(
                    visual,
                    "origin",
                    attrib={
                        "xyz": _fmt_xyz(viz_xyz),
                        "rpy": _fmt_xyz(viz_rpy),
                    },
                )

            geom = ET.SubElement(visual, "geometry")
            ET.SubElement(
                geom,
                "mesh",
                attrib={
                    "filename": f"stl_files/{mesh_name}.stl",
                    "scale": "0.001 0.001 0.001",
                },
            )

    # Generate joints
    for joint_spec in chain["joints"]:
        joint_el = ET.SubElement(
            robot,
            "joint",
            **{
                "name": joint_spec["name"],
                "type": joint_spec["type"],
            },
        )

        ET.SubElement(joint_el, "parent", link=joint_spec["parent"])
        ET.SubElement(joint_el, "child", link=joint_spec["child"])

        jnt_xyz = _compute_joint_origin(
            joint_spec,
            link_specs,
            analyses,
            chain.get("dh_params", {}),
            messages,
        )

        jnt_rpy = joint_spec.get("origin_rpy", [0, 0, 0])
        ET.SubElement(
            joint_el,
            "origin",
            attrib={
                "xyz": _fmt_xyz(jnt_xyz),
                "rpy": _fmt_xyz(jnt_rpy),
            },
        )

        ET.SubElement(
            joint_el,
            "axis",
            xyz=_fmt_xyz(joint_spec["axis"]),
        )

        if joint_spec["type"] == "revolute" and "limits" in joint_spec:
            limits = joint_spec["limits"]
            ET.SubElement(
                joint_el,
                "limit",
                attrib={
                    "lower": str(limits[0]),
                    "upper": str(limits[1]),
                    "effort": str(joint_spec.get("effort", 100)),
                    "velocity": str(joint_spec.get("velocity", 1)),
                },
            )

    # Write pretty-printed XML
    rough_xml = ET.tostring(robot, encoding="unicode")
    dom = xml.dom.minidom.parseString(rough_xml)
    doc_el = dom.documentElement
    assert doc_el is not None
    pretty_xml = '<?xml version="1.0"?>\n' + doc_el.toprettyxml(indent="  ")
    lines = [line for line in pretty_xml.split("\n") if line.strip()]
    final_xml = "\n".join(lines) + "\n"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(final_xml)
    messages.append(f"Wrote URDF to {output_path}")

    fk_messages = _validate_fk(chain, output_path)
    messages.extend(fk_messages)

    return messages


def _compute_visual_origin(
    analysis: dict,
    link_spec: dict,
    link_name: str,
    *,
    messages: list[str],
) -> tuple[list[float], list[float]]:
    """Compute visual origin xyz/rpy: proximal bore at frame origin."""
    conn_points = analysis.get("connection_points", [])
    proximal = next((cp for cp in conn_points if cp["end"] == "proximal"), None)
    viz_rpy = link_spec.get("visual_rpy", [0, 0, 0])

    if proximal is None:
        viz_xyz = link_spec.get("visual_xyz", [0, 0, 0])
        messages.append(f"  {link_name}: no proximal connection point")
        return viz_xyz, viz_rpy

    pos = list(proximal["position"])  # copy — don't mutate original

    # Adjust bore-axis component from face to barrel center.
    # Bore detection reports the face surface (bbox extreme); DH distances
    # measure center-to-center, so we use the bbox midpoint along the bore
    # axis to place each mesh centered on its joint axis.
    bore_axis = proximal.get("axis", [0, 0, 0])
    axis_idx = max(range(3), key=lambda i: abs(bore_axis[i]))
    bbox = analysis.get("geometry", {}).get("bounding_box", {})
    method = proximal.get("method", "")
    if bbox and abs(bore_axis[axis_idx]) > 0.5 and method != "manual":
        bmin = bbox["min"][axis_idx]
        bmax = bbox["max"][axis_idx]
        pos[axis_idx] = (bmin + bmax) / 2

    messages.append(
        f"  {link_name}: proximal @ origin,"
        f" proximal=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})mm"
    )
    if viz_rpy == [0, 0, 0]:
        viz_xyz = [-pos[i] / 1000 for i in range(3)]
    else:
        viz_xyz = (-_rpy_to_rotation(viz_rpy) @ (np.array(pos) / 1000)).tolist()

    extra = link_spec.get("visual_xyz")
    if extra is not None:
        viz_xyz = [v + e for v, e in zip(viz_xyz, extra, strict=True)]
    return [round(v, 6) for v in viz_xyz], viz_rpy


def _compute_joint_origin(
    joint_spec: dict,
    link_specs: dict,
    analyses: dict,
    dh_params: dict,
    messages: list[str],
) -> list[float]:
    """Compute joint origin xyz in parent frame.

    Uses parent mesh's distal connection point if available,
    falling back to explicit origin in chain spec.
    """
    parent_name = joint_spec["parent"]
    parent_spec = link_specs.get(parent_name, {})
    parent_mesh = parent_spec.get("mesh")

    if "origin" in joint_spec:
        return joint_spec["origin"]

    if parent_mesh and parent_mesh in analyses:
        analysis = analyses[parent_mesh]
        conn_points = analysis.get("connection_points", [])
        distal = next(
            (cp for cp in conn_points if cp["end"] == "distal"),
            None,
        )

        if distal is not None:
            pos = distal["position"]
            messages.append(
                f"  {joint_spec['name']}: distal="
                f"({pos[0]:.1f}, {pos[1]:.1f},"
                f" {pos[2]:.1f})mm"
            )
            proximal = next(
                (cp for cp in conn_points if cp["end"] == "proximal"),
                None,
            )

            if proximal is not None:
                pp = proximal["position"]
                xyz = [
                    (pos[0] - pp[0]) / 1000,
                    (pos[1] - pp[1]) / 1000,
                    (pos[2] - pp[2]) / 1000,
                ]
            else:
                xyz = [
                    pos[0] / 1000,
                    pos[1] / 1000,
                    pos[2] / 1000,
                ]

            parent_rpy = parent_spec.get("visual_rpy", [0, 0, 0])
            if parent_rpy != [0, 0, 0]:
                rot = _rpy_to_rotation(parent_rpy)
                xyz = (rot @ np.array(xyz)).tolist()

            rounded = [round(v, 4) for v in xyz]
            messages.append(
                f"  {joint_spec['name']}: origin from connection points = {rounded}"
            )
            return [round(v, 6) for v in xyz]

    messages.append(
        f"  {joint_spec['name']}: no connection point data, using chain spec fallback"
    )
    return joint_spec.get("origin", [0, 0, 0])


def _rpy_to_rotation(rpy: list[float]) -> np.ndarray:
    """Convert roll-pitch-yaw to 3x3 rotation matrix."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _validate_fk(chain: dict, urdf_path: Path) -> list[str]:
    """Run FK at zero config and report joint positions.

    Also validates inter-joint distances against DH params
    (comparing distances, not absolute positions, to avoid the
    DH-to-world mapping pitfall).
    """
    messages: list[str] = []
    dh = chain.get("dh_params", {})
    if not dh:
        return messages

    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        pos = np.zeros(3)
        rot = np.eye(3)
        messages.append("FK validation (zero config):")

        positions: dict[str, np.ndarray] = {}
        for joint_el in root.findall("joint"):
            name = joint_el.get("name")
            origin = joint_el.find("origin")
            xyz_str = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy_str = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            xyz = np.array([float(v) for v in xyz_str.split()])
            rpy = [float(v) for v in rpy_str.split()]

            pos = pos + rot @ xyz

            if any(abs(v) > 1e-6 for v in rpy):
                rot = rot @ _rpy_to_rotation(rpy)

            pos_mm = pos * 1000
            assert name is not None
            positions[name] = pos_mm.copy()
            messages.append(
                f"  {name}: ({pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}) mm"
            )

        # Report consecutive joint distances
        joint_names = list(positions.keys())
        for i in range(len(joint_names) - 1):
            j_from = joint_names[i]
            j_to = joint_names[i + 1]
            dist = float(np.linalg.norm(positions[j_to] - positions[j_from]))
            messages.append(f"  {j_from}→{j_to} distance={dist:.1f}mm")

    except Exception as e:
        messages.append(f"FK validation failed: {e}")

    return messages


def _fmt_xyz(values: list[float]) -> str:
    """Format a list of 3 floats as space-separated string."""
    return " ".join(f"{v:.6f}" if isinstance(v, float) else str(v) for v in values)
