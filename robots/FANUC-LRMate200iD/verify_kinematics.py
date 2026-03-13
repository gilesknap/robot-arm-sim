#!/usr/bin/env python3
"""Verify FANUC LR Mate 200iD URDF kinematics against manufacturer specs.

Usage:
    uv run python robots/FANUC-LRMate200iD/verify_kinematics.py
    uv run python robots/FANUC-LRMate200iD/verify_kinematics.py --json

Traces the kinematic chain at zero config (all joints = 0) and a test pose,
comparing against expected positions derived from the FANUC catalog DH parameters.

DH params (standard convention):
  d = [330, 0, 0, 335, 0, 80] mm
  a = [50, 330, 35, 0, 0, 0] mm
  alpha = [-90, 0, -90, 90, -90, 0] deg
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
    urdf_path: Path,
    joint_angles: dict[str, float] | None = None,
    quiet: bool = False,
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

        world_pos = (parent_tf @ origin_tf)[:3, 3] * 1000  # mm
        joint_positions[joint["name"]] = world_pos.tolist()
        child_z = child_tf[:3, 2]
        if not quiet:
            print(
                f"  {joint['name']:10s} → {joint['child']:10s}  "
                f"pos=({world_pos[0]:7.1f}, "
                f"{world_pos[1]:7.1f}, "
                f"{world_pos[2]:7.1f})mm  "
                f"axis={joint['axis']}  "
                f"frame_Z=({child_z[0]:+.2f}, "
                f"{child_z[1]:+.2f}, "
                f"{child_z[2]:+.2f})"
            )

    if not quiet:
        print()
    return transforms, joint_positions


# FANUC LR Mate 200iD expected zero-config positions
# Computed from FANUC catalog DH parameters:
#   d = [330, 0, 0, 335, 0, 80] mm
#   a = [50, 330, 35, 0, 0, 0] mm
# At zero config, arm extends straight up then wrist extends along X.
EXPECTED_ZERO_CONFIG = {
    "joint_1": [0.0, 0.0, 330.0],  # base height
    "joint_2": [50.0, 0.0, 330.0],  # shoulder offset
    "joint_3": [50.0, 0.0, 660.0],  # upper arm
    "joint_4": [50.0, 0.0, 695.0],  # elbow offset
    "joint_5": [385.0, 0.0, 695.0],  # forearm
    "joint_6": [465.0, 0.0, 695.0],  # flange
}


def validate_positions(
    joint_positions: dict,
    expected: dict,
    tolerance_mm: float = 2.0,
) -> list[dict]:
    """Compare computed positions against expected."""
    results = []
    for name, exp in expected.items():
        actual = joint_positions.get(name)
        if actual is None:
            results.append(
                {
                    "joint": name,
                    "pass": False,
                    "error_mm": None,
                    "reason": "missing",
                }
            )
            continue
        error = np.linalg.norm(np.array(actual) - np.array(exp))
        results.append(
            {
                "joint": name,
                "pass": bool(error <= tolerance_mm),
                "expected_mm": exp,
                "actual_mm": [round(v, 1) for v in actual],
                "error_mm": round(float(error), 2),
            }
        )
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Verify FANUC LR Mate 200iD URDF kinematics"
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output structured JSON results",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=2.0,
        help="Tolerance in mm (default: 2)",
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
        _, joint_positions = verify_urdf(urdf_path, quiet=True)
        results = validate_positions(
            joint_positions, EXPECTED_ZERO_CONFIG, args.tolerance
        )
        all_pass = all(r["pass"] for r in results)
        output = {
            "urdf": str(urdf_path),
            "tolerance_mm": args.tolerance,
            "config": "zero_config",
            "all_pass": all_pass,
            "joints": results,
        }
        print(json.dumps(output, indent=2))
        sys.exit(0 if all_pass else 1)

    # Human-readable mode
    _, joint_positions = verify_urdf(urdf_path)

    # Zero config validation
    results = validate_positions(joint_positions, EXPECTED_ZERO_CONFIG, args.tolerance)
    print(f"=== Zero Config Validation (tol={args.tolerance}mm) ===")
    for r in results:
        status = "PASS" if r["pass"] else "FAIL"
        if r["error_mm"] is not None:
            print(
                f"  {r['joint']:10s}: {status}  "
                f"error={r['error_mm']:.1f}mm  "
                f"expected={r['expected_mm']}  "
                f"actual={r['actual_mm']}"
            )
        else:
            print(f"  {r['joint']:10s}: {status}  ({r['reason']})")
    print()

    # Test pose: J2 = -pi/2 (arm forward)
    import math

    print("=== Test pose: arm forward (J2 = -pi/2) ===")
    verify_urdf(urdf_path, {"joint_2": -math.pi / 2})


if __name__ == "__main__":
    main()
