"""URDF joint validation utilities for Digital Twin."""

import xml.etree.ElementTree as ET


def parse_required_joints(urdf_xml: str) -> set[str]:
    """Extract required (non-fixed, non-mimic) joint names from URDF XML.

    Args:
        urdf_xml: Raw URDF XML string (already processed by xacro if needed).

    Returns:
        Set of joint names that require JointState data.
    """
    root = ET.fromstring(urdf_xml)
    required = set()
    for joint_elem in root.findall('joint'):
        jtype = joint_elem.get('type', '')
        if jtype == 'fixed':
            continue
        if joint_elem.find('mimic') is not None:
            continue
        jname = joint_elem.get('name', '')
        if jname:
            required.add(jname)
    return required


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
