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

from robot_arm_sim.models import load_chain_yaml, load_part_yaml

from .urdf_transforms import (
    close_surface_gaps,
    compute_joint_origin,
    compute_visual_origin,
    validate_fk,
)

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

    chain_model = load_chain_yaml(chain_path)
    chain = chain_model.model_dump(exclude_none=True)

    # Load analysis data for each mesh
    analyses: dict[str, dict] = {}
    for link in chain["links"]:
        mesh_name = link.get("mesh")
        if mesh_name:
            yaml_path = analysis_dir / f"{mesh_name}.yaml"
            if yaml_path.exists():
                part_model = load_part_yaml(yaml_path)
                analyses[mesh_name] = part_model.model_dump(exclude_none=True)
            else:
                messages.append(f"WARNING: No analysis YAML for {mesh_name}")

    robot_name = chain.get("robot_name", "robot")
    robot = ET.Element("robot", name=robot_name)

    link_specs = {lk["name"]: lk for lk in chain["links"]}

    # --- Pass 1: compute all visual origins and joint origins ---
    visual_origins: dict[str, tuple[list[float], list[float]]] = {}
    for link_spec in chain["links"]:
        link_name = link_spec["name"]
        mesh_name = link_spec.get("mesh")
        if mesh_name and mesh_name in analyses:
            viz_xyz, viz_rpy = compute_visual_origin(
                analyses[mesh_name], link_spec, link_name, messages=messages
            )
            visual_origins[link_name] = (viz_xyz, viz_rpy)

    joint_origins: dict[str, list[float]] = {}
    joint_rpys: dict[str, list[float]] = {}
    for joint_spec in chain["joints"]:
        jnt_xyz = compute_joint_origin(
            joint_spec,
            link_specs,
            analyses,
            chain.get("dh_params", {}),
            messages,
        )
        joint_origins[joint_spec["name"]] = jnt_xyz
        joint_rpys[joint_spec["name"]] = joint_spec.get("origin_rpy", [0, 0, 0])

    # --- Pass 2: close surface gaps ---
    close_surface_gaps(
        chain, analyses, visual_origins, joint_origins, joint_rpys, messages
    )

    # --- Emit XML ---
    for link_spec in chain["links"]:
        link_name = link_spec["name"]
        mesh_name = link_spec.get("mesh")

        link_el = ET.SubElement(robot, "link", name=link_name)

        if mesh_name and mesh_name in analyses:
            visual = ET.SubElement(link_el, "visual")

            viz_xyz, viz_rpy = visual_origins.get(link_name, ([0, 0, 0], [0, 0, 0]))

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

        jnt_xyz = joint_origins[joint_spec["name"]]
        jnt_rpy = joint_rpys[joint_spec["name"]]
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

    fk_messages = validate_fk(chain, output_path)
    messages.extend(fk_messages)

    return messages


def _fmt_xyz(values: list[float]) -> str:
    """Format a list of 3 floats as space-separated string."""
    return " ".join(f"{v:.6f}" if isinstance(v, float) else str(v) for v in values)
