#!/usr/bin/env python3
"""Verify UR5 URDF kinematics by computing zero-config joint positions.

Usage:
    uv run python robots/UR5/verify_kinematics.py
    uv run python robots/UR5/verify_kinematics.py --json

At DH zero config (all joints = 0), the UR5 arm extends horizontally.
The expected joint positions are computed from manufacturer DH parameters
plus shoulder_offset and elbow_offset.
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# ---------- kinematics helpers ----------


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0],
            [-sp, cp * sr, cp * cr, 0],
            [0, 0, 0, 1],
        ]
    )


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
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
            [0, 0, 0, 1],
        ]
    )


# ---------- URDF parser ----------


def parse_xyz(s: str) -> list[float]:
    return [float(v) for v in s.split()]


def verify_urdf(
    urdf_path: Path, joint_angles: dict[str, float] | None = None, quiet: bool = False
):
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
        joints.append(
            {
                "name": name,
                "type": jtype,
                "parent": parent,
                "child": child,
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis,
            }
        )

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
        child_z = child_tf[:3, 2]
        if not quiet:
            print(
                f"  {joint['name']:25s} → {joint['child']:18s}  "
                f"pos=({world_pos[0]:8.1f}, {world_pos[1]:8.1f},"
                f" {world_pos[2]:8.1f})mm  "
                f"axis={joint['axis']}  "
                f"frame_Z=({child_z[0]:+.2f}, {child_z[1]:+.2f},"
                f" {child_z[2]:+.2f})"
            )

    if not quiet:
        print()
    return transforms, joints, joint_positions


# UR5 expected zero-config joint positions (mm).
# Computed from DH parameters + shoulder/elbow offsets.
# At zero config, the arm extends horizontally in +X direction.
#
# Trace:
#   J1: [0, 0, d1] = [0, 0, 89.159]
#   J2: J1 + [0, shoulder_offset, 0] = [0, 135.85, 89.159]
#   J3: J2 + Ry(pi/2) × [0, elbow_offset, a2]
#       = J2 + [a2, elbow_offset, 0] = [425, 16.15, 89.159]
#   J4: J3 + Ry(pi/2) × [0, 0, a3]
#       = J3 + [a3, 0, 0] = [817.25, 16.15, 89.159]
#   J5: J4 + Ry(pi) × [0, d4, 0]
#       = J4 + [0, d4, 0] = [817.25, 125.3, 89.159]
#   J6: J5 + Ry(pi) × [0, 0, d5]
#       = J5 + [0, 0, -d5] = [817.25, 125.3, -5.491]
EXPECTED_POSITIONS = {
    "shoulder_pan_joint": [0.0, 0.0, 89.159],
    "shoulder_lift_joint": [0.0, 135.85, 89.159],
    "elbow_joint": [425.0, 16.15, 89.159],
    "wrist_1_joint": [817.25, 16.15, 89.159],
    "wrist_2_joint": [817.25, 125.3, 89.159],
    "wrist_3_joint": [817.25, 125.3, -5.491],
}


def validate_against_expected(
    joint_positions: dict, tolerance_mm: float = 2.0
) -> list[dict]:
    """Compare computed positions against expected, return per-joint results."""
    results = []
    for name, expected in EXPECTED_POSITIONS.items():
        actual = joint_positions.get(name)
        if actual is None:
            results.append(
                {"joint": name, "pass": False, "error_mm": None, "reason": "missing"}
            )
            continue
        error = np.linalg.norm(np.array(actual) - np.array(expected))
        results.append(
            {
                "joint": name,
                "pass": bool(error <= tolerance_mm),
                "expected_mm": expected,
                "actual_mm": [round(v, 1) for v in actual],
                "error_mm": round(float(error), 2),
            }
        )
    return results


# ---------- main ----------


def main():
    parser = argparse.ArgumentParser(description="Verify UR5 URDF kinematics")
    parser.add_argument(
        "--json", action="store_true", help="Output structured JSON results"
    )
    parser.add_argument(
        "--tolerance", type=float, default=2.0, help="Tolerance in mm (default: 2)"
    )
    args = parser.parse_args()

    robot_dir = Path(__file__).parent
    urdf_path = robot_dir / "robot.urdf"

    if not urdf_path.exists():
        if args.json:
            print(json.dumps({"error": f"{urdf_path} not found"}))
        else:
            print(f"Error: {urdf_path} not found")
        sys.exit(1)

    if args.json:
        _, _, joint_positions = verify_urdf(urdf_path, quiet=True)
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
    _, _, joint_positions = verify_urdf(urdf_path)

    # Validate against expected
    results = validate_against_expected(joint_positions, args.tolerance)
    print(f"=== Validation (tolerance={args.tolerance}mm) ===")
    for r in results:
        status = "PASS" if r["pass"] else "FAIL"
        if r["error_mm"] is not None:
            print(
                f"  {r['joint']:25s}: {status}  error={r['error_mm']:.1f}mm  "
                f"expected={r['expected_mm']}  actual={r['actual_mm']}"
            )
        else:
            print(f"  {r['joint']:25s}: {status}  ({r['reason']})")
    print()

    # Inter-joint distances
    print("=== Inter-joint distances ===")
    names = list(EXPECTED_POSITIONS.keys())
    for i in range(len(names) - 1):
        a = np.array(joint_positions[names[i]])
        b = np.array(joint_positions[names[i + 1]])
        dist = np.linalg.norm(b - a)
        print(f"  {names[i]:25s} → {names[i + 1]:25s}: {dist:.1f}mm")
    print()

    # Arm-up test pose: J2=-90deg should stand the arm straight up
    print("=== Test pose (J2=-pi/2 = arm up) ===")
    verify_urdf(urdf_path, {"shoulder_lift_joint": -1.5707963})


if __name__ == "__main__":
    main()
