#!/usr/bin/env python3
"""Verify URDF kinematics by computing zero-config joint positions.

Usage:
    uv run python robots/Meca500-R3/verify_kinematics.py

Traces the kinematic chain at zero config (all joints = 0) and prints
the world-frame position of each joint. Use this to sanity-check that
the URDF joint origins, axes, and frame rotations produce the expected
robot geometry.

Also analyzes STL bounding boxes and cross-sections to help determine
correct visual origin offsets for each mesh.
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh


# ---------- kinematics helpers ----------

def ry(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0],
        [-sp, cp * sr, cp * cr, 0],
        [0, 0, 0, 1],
    ])


def translation(x: float, y: float, z: float) -> np.ndarray:
    m = np.eye(4)
    m[0, 3], m[1, 3], m[2, 3] = x, y, z
    return m


def rotation_matrix(axis: list[float], angle: float) -> np.ndarray:
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c, s = np.cos(angle), np.sin(angle)
    t = 1 - c
    x, y, z = ax
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
        [0, 0, 0, 1],
    ])


# ---------- URDF parser ----------

def parse_xyz(s: str) -> list[float]:
    return [float(v) for v in s.split()]


def verify_urdf(urdf_path: Path, joint_angles: dict[str, float] | None = None, quiet: bool = False):
    """Parse URDF and compute forward kinematics."""
    if joint_angles is None:
        joint_angles = {}

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_name = root.get("name", "unknown")
    if not quiet:
        print(f"Robot: {robot_name}")
        print(f"URDF:  {urdf_path}")
        print()

    # Parse joints
    joints = []
    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        origin = j.find("origin")
        xyz = parse_xyz(origin.get("xyz", "0 0 0"))
        rpy = parse_xyz(origin.get("rpy", "0 0 0"))
        axis_el = j.find("axis")
        axis = parse_xyz(axis_el.get("xyz")) if axis_el is not None else [0, 0, 1]
        joints.append({
            "name": name, "type": jtype, "parent": parent, "child": child,
            "xyz": xyz, "rpy": rpy, "axis": axis,
        })

    # Parse links with visual origins
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        vis = link.find("visual")
        if vis is not None:
            origin = vis.find("origin")
            mesh = vis.find("geometry/mesh")
            links[name] = {
                "xyz": parse_xyz(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0],
                "rpy": parse_xyz(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0],
                "mesh": mesh.get("filename") if mesh is not None else None,
            }
        else:
            links[name] = {"xyz": [0, 0, 0], "rpy": [0, 0, 0], "mesh": None}

    # Forward kinematics
    transforms = {}
    root_link = joints[0]["parent"] if joints else "base_link"
    transforms[root_link] = np.eye(4)

    config_label = "zero config" if not joint_angles else f"config {joint_angles}"
    if not quiet:
        print(f"=== Joint positions ({config_label}) ===")

    joint_positions = {}
    for joint in joints:
        parent_tf = transforms.get(joint["parent"], np.eye(4))
        xyz = joint["xyz"]
        rpy = joint["rpy"]
        angle = joint_angles.get(joint["name"], 0.0)

        origin_tf = translation(*xyz) @ rpy_to_matrix(*rpy)
        rot_tf = rotation_matrix(joint["axis"], angle)
        child_tf = parent_tf @ origin_tf @ rot_tf

        transforms[joint["child"]] = child_tf

        # World position of this joint
        world_pos = (parent_tf @ origin_tf)[:3, 3] * 1000  # mm
        joint_positions[joint["name"]] = world_pos.tolist()
        # Child frame axes in world
        child_z = child_tf[:3, 2]
        if not quiet:
            print(f"  {joint['name']:10s} → {joint['child']:10s}  "
                  f"pos=({world_pos[0]:7.1f}, {world_pos[1]:7.1f}, {world_pos[2]:7.1f})mm  "
                  f"axis={joint['axis']}  "
                  f"frame_Z=({child_z[0]:+.2f}, {child_z[1]:+.2f}, {child_z[2]:+.2f})")

    if not quiet:
        print()
    return transforms, links, joints, joint_positions


# Expected positions from Meca500 specs (mm)
# Expected zero-config positions from Meca500 DH params:
# d1=135 (J2 height), a2=135 (upper arm), a3=38 (X offset),
# d4=120 (forearm Z), d6=70 (wrist Z). J1 at base top (93mm).
EXPECTED_POSITIONS = {
    "joint_1": [0.0, 0.0, 93.0],
    "joint_2": [0.0, 0.0, 135.0],
    "joint_3": [0.0, 0.0, 270.0],
    "joint_4": [38.0, 0.0, 390.0],
    "joint_5": [38.0, 0.0, 390.0],
    "joint_6": [38.0, 0.0, 460.0],
}


def validate_against_expected(joint_positions: dict, tolerance_mm: float = 5.0) -> list[dict]:
    """Compare computed positions against expected, return per-joint results."""
    results = []
    for name, expected in EXPECTED_POSITIONS.items():
        actual = joint_positions.get(name)
        if actual is None:
            results.append({"joint": name, "pass": False, "error_mm": None, "reason": "missing"})
            continue
        error = np.linalg.norm(np.array(actual) - np.array(expected))
        results.append({
            "joint": name,
            "pass": bool(error <= tolerance_mm),
            "expected_mm": expected,
            "actual_mm": [round(v, 1) for v in actual],
            "error_mm": round(float(error), 2),
        })
    return results


# ---------- STL analysis ----------

def analyze_stl(stl_dir: Path):
    """Analyze STL meshes to determine bore centers and visual origin offsets."""
    print("=== STL Mesh Analysis ===")
    stl_files = sorted(stl_dir.glob("*.stl"))

    for stl_path in stl_files:
        mesh = trimesh.load(stl_path)
        name = stl_path.stem
        bounds = mesh.bounds
        extents = mesh.extents
        centroid = mesh.centroid

        print(f"\n  {name}:")
        print(f"    Bounds:   x[{bounds[0][0]:+.1f}, {bounds[1][0]:+.1f}]  "
              f"y[{bounds[0][1]:+.1f}, {bounds[1][1]:+.1f}]  "
              f"z[{bounds[0][2]:+.1f}, {bounds[1][2]:+.1f}]")
        print(f"    Extents:  {extents[0]:.1f} x {extents[1]:.1f} x {extents[2]:.1f} mm")
        print(f"    Centroid: ({centroid[0]:.1f}, {centroid[1]:.1f}, {centroid[2]:.1f})")

        # Cross-sections along Z (useful for parts with Z up)
        z_min, z_max = bounds[0][2], bounds[1][2]
        z_range = z_max - z_min
        if z_range > 10:
            print(f"    Z cross-sections:")
            for frac in [0.1, 0.5, 0.9]:
                z = z_min + frac * z_range
                try:
                    slc = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
                    if slc:
                        path_2d, xform = slc.to_planar()
                        polys = path_2d.polygons_full
                        if polys:
                            largest = max(polys, key=lambda p: p.area)
                            verts = np.array(largest.exterior.coords)
                            pts = np.column_stack([verts, np.zeros(len(verts)), np.ones(len(verts))])
                            pts_w = (xform @ pts.T).T[:, :3]
                            cx, cy = pts_w[:, 0].mean(), pts_w[:, 1].mean()
                            print(f"      z={z:+.1f}: center=({cx:+.1f}, {cy:+.1f}), area={largest.area:.0f}mm²")
                except Exception:
                    pass


# ---------- main ----------

def main():
    parser = argparse.ArgumentParser(description="Verify URDF kinematics")
    parser.add_argument("--json", action="store_true", help="Output structured JSON results")
    parser.add_argument("--tolerance", type=float, default=5.0, help="Tolerance in mm (default: 5)")
    args = parser.parse_args()

    robot_dir = Path(__file__).parent
    urdf_path = robot_dir / "robot.urdf"
    stl_dir = robot_dir / "stl_files"

    if not urdf_path.exists():
        if args.json:
            print(json.dumps({"error": f"{urdf_path} not found"}))
        else:
            print(f"Error: {urdf_path} not found")
        sys.exit(1)

    if args.json:
        # JSON mode: quiet FK, structured output
        _, _, _, joint_positions = verify_urdf(urdf_path, quiet=True)
        results = validate_against_expected(joint_positions, args.tolerance)
        all_pass = all(r["pass"] for r in results)
        output = {
            "urdf": str(urdf_path),
            "tolerance_mm": args.tolerance,
            "all_pass": all_pass,
            "joints": results,
        }
        print(json.dumps(output, indent=2))
        sys.exit(0 if all_pass else 1)

    # Human-readable mode
    transforms, links, joints, joint_positions = verify_urdf(urdf_path)

    # Validate against expected
    results = validate_against_expected(joint_positions, args.tolerance)
    print(f"=== Validation (tolerance={args.tolerance}mm) ===")
    for r in results:
        status = "PASS" if r["pass"] else "FAIL"
        if r["error_mm"] is not None:
            print(f"  {r['joint']:10s}: {status}  error={r['error_mm']:.1f}mm  "
                  f"expected={r['expected_mm']}  actual={r['actual_mm']}")
        else:
            print(f"  {r['joint']:10s}: {status}  ({r['reason']})")
    print()

    # Also verify at a test pose
    test_angles = {"joint_2": 0.5, "joint_3": -0.5}
    print(f"=== Test pose (J2=0.5rad, J3=-0.5rad) ===")
    verify_urdf(urdf_path, test_angles)

    # STL analysis
    if stl_dir.exists():
        analyze_stl(stl_dir)


if __name__ == "__main__":
    main()
