"""URDF joint validation and classification utilities for Digital Twin."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field


@dataclass(frozen=True)
class MimicParams:
    """Parameters for a mimic joint relationship."""

    master_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class JointInfo:
    """Parsed information for a single URDF joint."""

    name: str
    joint_type: str  # "revolute", "prismatic", "continuous", "planar", "floating"
    parent_link: str
    child_link: str
    mimic: MimicParams | None = None
    is_closed_chain: bool = False


@dataclass
class JointClassification:
    """Classification of all non-fixed URDF joints."""

    active: dict[str, JointInfo] = field(default_factory=dict)
    passive_mimic: dict[str, JointInfo] = field(default_factory=dict)
    passive_closed_chain: dict[str, JointInfo] = field(default_factory=dict)
    fixed: list[str] = field(default_factory=list)

    @property
    def active_names(self) -> set[str]:
        """Joint names that require external JointState data."""
        return set(self.active.keys())

    @property
    def passive_names(self) -> set[str]:
        """Joint names that are passively driven (mimic or closed-chain)."""
        return set(self.passive_mimic.keys()) | set(self.passive_closed_chain.keys())


def classify_joints(urdf_xml: str) -> JointClassification:
    """Parse URDF XML and classify joints into active and passive categories.

    Classification rules:
      - fixed joints are recorded but excluded from active/passive.
      - mimic joints (with <mimic> element) are passive.
      - closed-chain joints (child_link referenced by multiple non-fixed joints)
        are passive — the first joint claiming a child_link is kept as active.
      - all remaining joints are active.

    Args:
        urdf_xml: Raw URDF XML string (already processed by xacro if needed).

    Returns:
        JointClassification with all joints categorized.
    """
    root = ET.fromstring(urdf_xml)
    result = JointClassification()

    # Step 1: Parse all joints and collect child_link references
    parsed_joints: list[JointInfo] = []
    child_link_joints: dict[str, list[int]] = defaultdict(list)

    for joint_elem in root.findall('joint'):
        jname = joint_elem.get('name', '')
        jtype = joint_elem.get('type', '')
        if not jname:
            continue

        if jtype == 'fixed':
            result.fixed.append(jname)
            continue

        parent_elem = joint_elem.find('parent')
        child_elem = joint_elem.find('child')
        parent_link = parent_elem.get('link', '') if parent_elem is not None else ''
        child_link = child_elem.get('link', '') if child_elem is not None else ''

        mimic = None
        mimic_elem = joint_elem.find('mimic')
        if mimic_elem is not None:
            mimic = MimicParams(
                master_joint=mimic_elem.get('joint', ''),
                multiplier=float(mimic_elem.get('multiplier', '1.0')),
                offset=float(mimic_elem.get('offset', '0.0')),
            )

        info = JointInfo(
            name=jname,
            joint_type=jtype,
            parent_link=parent_link,
            child_link=child_link,
            mimic=mimic,
        )
        idx = len(parsed_joints)
        parsed_joints.append(info)

        if child_link:
            child_link_joints[child_link].append(idx)

    # Step 2: Detect closed-chain joints
    closed_chain_indices: set[int] = set()
    for indices in child_link_joints.values():
        if len(indices) <= 1:
            continue
        # First non-mimic joint is the tree joint; rest are closed-chain
        first_tree = True
        for i in indices:
            if parsed_joints[i].mimic is not None:
                continue
            if first_tree:
                first_tree = False
                continue
            closed_chain_indices.add(i)

    # Step 3: Classify
    for i, info in enumerate(parsed_joints):
        if info.mimic is not None:
            result.passive_mimic[info.name] = info
        elif i in closed_chain_indices:
            result.passive_closed_chain[info.name] = JointInfo(
                name=info.name,
                joint_type=info.joint_type,
                parent_link=info.parent_link,
                child_link=info.child_link,
                mimic=info.mimic,
                is_closed_chain=True,
            )
        else:
            result.active[info.name] = info

    return result


def parse_required_joints(urdf_xml: str) -> set[str]:
    """Extract required (non-fixed, non-mimic) joint names from URDF XML.

    Args:
        urdf_xml: Raw URDF XML string (already processed by xacro if needed).

    Returns:
        Set of joint names that require JointState data.
    """
    classification = classify_joints(urdf_xml)
    return classification.active_names


def compute_mimic_positions(
    classification: JointClassification,
    joint_positions: dict[str, float],
) -> dict[str, float]:
    """Compute mimic joint positions from their master joint positions.

    Args:
        classification: Joint classification result from classify_joints().
        joint_positions: Current positions keyed by joint name.

    Returns:
        Dict of {mimic_joint_name: computed_position} for mimic joints
        whose master joint has a known position.
    """
    result: dict[str, float] = {}
    for name, info in classification.passive_mimic.items():
        if info.mimic is None:
            continue
        master_pos = joint_positions.get(info.mimic.master_joint)
        if master_pos is not None:
            result[name] = info.mimic.multiplier * master_pos + info.mimic.offset
    return result


def validate_joints(
    required: set[str], received: set[str]
) -> tuple[set[str], set[str]]:
    """Compare required joints against received joints.

    Args:
        required: Set of joint names required by URDF.
        received: Set of joint names observed from JointState topics.

    Returns:
        Tuple of (covered, missing) joint name sets.
    """
    covered = required & received
    missing = required - received
    return covered, missing
