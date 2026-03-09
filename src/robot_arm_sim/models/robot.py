"""Dataclasses for URDF robot model."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class URDFLink:
    """A link in a URDF robot model."""

    name: str
    mesh_path: str | None = None
    mesh_scale: list[float] = field(default_factory=lambda: [0.001, 0.001, 0.001])
    origin_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class URDFJoint:
    """A joint in a URDF robot model."""

    name: str
    joint_type: str  # "revolute", "prismatic", "fixed", "continuous"
    parent: str
    child: str
    axis: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    origin_xyz: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    limit_lower: float = -np.pi
    limit_upper: float = np.pi
    limit_effort: float = 100.0
    limit_velocity: float = 1.0


@dataclass
class URDFRobot:
    """A complete URDF robot model."""

    name: str
    links: list[URDFLink] = field(default_factory=list)
    joints: list[URDFJoint] = field(default_factory=list)

    def get_link(self, name: str) -> URDFLink | None:
        for link in self.links:
            if link.name == name:
                return link
        return None

    def get_joint(self, name: str) -> URDFJoint | None:
        for joint in self.joints:
            if joint.name == name:
                return joint
        return None

    def get_kinematic_chain(self) -> list[URDFJoint]:
        """Return joints ordered from base to tip."""
        if not self.joints:
            return []

        # Find root link (parent but never child)
        child_names = {j.child for j in self.joints}
        parent_names = {j.parent for j in self.joints}
        roots = parent_names - child_names
        if not roots:
            return list(self.joints)

        chain = []
        current = roots.pop()
        joint_by_parent = {j.parent: j for j in self.joints}
        while current in joint_by_parent:
            joint = joint_by_parent[current]
            chain.append(joint)
            current = joint.child
        return chain
