"""Robot-agnostic URDF parser for Digital Twin visualization.

Inspired by rtc_urdf_bridge's UrdfAnalyzer (C++), this module provides
a self-contained Python implementation for URDF loading, link graph construction,
and joint classification — without depending on rtc_urdf_bridge.

Typical usage::

    parser = UrdfParser.from_file("/path/to/robot.urdf")
    parser = UrdfParser.from_xml(urdf_xml_string)

    # Access parsed data
    parser.robot_name            # "ur5e"
    parser.urdf_xml              # full XML string (for robot_state_publisher)
    parser.classification        # JointClassification
    parser.link_nodes            # adjacency-list link graph

    # Queries
    parser.get_joint_meta("joint1")
    parser.get_active_joint_names()
    parser.get_mimic_joint_names()

    # Computation
    mimic_pos = parser.compute_mimic_positions({"master": 1.0})
    covered, missing = parser.validate_joints(received_names)
"""

from __future__ import annotations

import logging
import os
import subprocess
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field

logger = logging.getLogger('rtc_digital_twin.urdf_parser')


# ── Data classes ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class MimicParams:
    """Parameters for a mimic joint relationship."""

    master_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class JointMeta:
    """Joint metadata (modeled after rtc_urdf_bridge's JointMeta).

    Attributes:
        name: Joint name from URDF.
        joint_type: One of "revolute", "prismatic", "continuous",
            "floating", "planar", "fixed".
        parent_link: Parent link name.
        child_link: Child link name.
        axis: Joint axis as (x, y, z) tuple.
        lower: Lower position limit (rad or m).
        upper: Upper position limit (rad or m).
        mimic: Mimic relationship parameters, or None.
        is_closed_chain: True if this joint forms a closed kinematic chain.
    """

    name: str
    joint_type: str
    parent_link: str
    child_link: str
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    lower: float = 0.0
    upper: float = 0.0
    mimic: MimicParams | None = None
    is_closed_chain: bool = False


@dataclass
class LinkNode:
    """Adjacency-list node (modeled after rtc_urdf_bridge's LinkNode).

    Attributes:
        link_name: Link name from URDF.
        parent_joint_name: Name of the joint connecting to the parent link.
        parent_index: Index of the parent LinkNode (-1 for root).
        child_indices: Indices of child LinkNodes.
    """

    link_name: str
    parent_joint_name: str = ''
    parent_index: int = -1
    child_indices: list[int] = field(default_factory=list)


@dataclass
class JointClassification:
    """Classification of all non-fixed URDF joints.

    Attributes:
        active: Joints requiring external JointState data.
        passive_mimic: Joints driven by mimic relationships.
        passive_closed_chain: Joints forming closed kinematic chains.
        fixed: Names of fixed joints.
    """

    active: dict[str, JointMeta] = field(default_factory=dict)
    passive_mimic: dict[str, JointMeta] = field(default_factory=dict)
    passive_closed_chain: dict[str, JointMeta] = field(default_factory=dict)
    fixed: list[str] = field(default_factory=list)

    @property
    def active_names(self) -> set[str]:
        """Joint names that require external JointState data."""
        return set(self.active.keys())

    @property
    def passive_names(self) -> set[str]:
        """Joint names that are passively driven (mimic or closed-chain)."""
        return set(self.passive_mimic.keys()) | set(
            self.passive_closed_chain.keys())


# ── URDF Parser ──────────────────────────────────────────────────────────────

_MOVABLE_TYPES = frozenset({
    'revolute', 'continuous', 'prismatic', 'floating', 'planar',
})


class UrdfParser:
    """Robot-agnostic URDF parser for digital twin visualization.

    Implements a parsing pipeline inspired by rtc_urdf_bridge's
    UrdfAnalyzer:

    1. URDF loading (file path with xacro auto-detection, or XML string)
    2. Link collection → LinkNode adjacency list
    3. Joint parsing → JointMeta dict (axis, limits, mimic)
    4. Adjacency graph construction (parent/child relationships)
    5. Joint classification (fixed / mimic / closed-chain / active)
    """

    def __init__(self, *, urdf_path: str = '', urdf_xml: str = '') -> None:
        """Create a parser from a file path or XML string.

        Exactly one of *urdf_path* or *urdf_xml* must be non-empty.

        Args:
            urdf_path: Path to a URDF or xacro file.
            urdf_xml: Raw URDF XML string.

        Raises:
            ValueError: If neither or both sources are provided.
            FileNotFoundError: If *urdf_path* does not exist.
            RuntimeError: If xacro processing or XML parsing fails.
        """
        if bool(urdf_path) == bool(urdf_xml):
            raise ValueError(
                'Provide exactly one of urdf_path or urdf_xml')

        if urdf_path:
            urdf_path = os.path.abspath(urdf_path)
            if not os.path.isfile(urdf_path):
                logger.error('URDF file not found: %s', urdf_path)
                raise FileNotFoundError(
                    f'URDF file not found: {urdf_path}')
            if urdf_path.endswith('.xacro'):
                logger.debug('Processing xacro: %s', urdf_path)
                urdf_xml = subprocess.check_output(
                    ['xacro', urdf_path], text=True)
            else:
                logger.debug('Loading URDF: %s', urdf_path)
                with open(urdf_path, 'r') as f:
                    urdf_xml = f.read()

        self._urdf_xml: str = urdf_xml
        self._urdf_path: str = urdf_path

        # Populated by _parse()
        self._robot_name: str = ''
        self._link_nodes: list[LinkNode] = []
        self._link_name_to_index: dict[str, int] = {}
        self._joint_meta: dict[str, JointMeta] = {}
        self._root_index: int = -1
        self._classification: JointClassification = JointClassification()

        self._parse()

    # ── Factory methods ──────────────────────────────────────────────────────

    @classmethod
    def from_file(cls, path: str) -> UrdfParser:
        """Create a parser from a URDF/xacro file path."""
        return cls(urdf_path=path)

    @classmethod
    def from_xml(cls, xml: str) -> UrdfParser:
        """Create a parser from a URDF XML string."""
        return cls(urdf_xml=xml)

    # ── Properties ───────────────────────────────────────────────────────────

    @property
    def robot_name(self) -> str:
        """Robot name from the URDF ``<robot name="...">`` attribute."""
        return self._robot_name

    @property
    def urdf_xml(self) -> str:
        """Full URDF XML string (for robot_state_publisher)."""
        return self._urdf_xml

    @property
    def classification(self) -> JointClassification:
        """Joint classification result."""
        return self._classification

    @property
    def link_nodes(self) -> list[LinkNode]:
        """Full link adjacency-list graph."""
        return self._link_nodes

    @property
    def root_index(self) -> int:
        """Index of the root link in :attr:`link_nodes`."""
        return self._root_index

    @property
    def num_links(self) -> int:
        """Total number of links."""
        return len(self._link_nodes)

    @property
    def num_joints(self) -> int:
        """Total number of joints (all types)."""
        return len(self._joint_meta)

    # ── Joint queries ────────────────────────────────────────────────────────

    def get_joint_meta(self, name: str) -> JointMeta:
        """Return metadata for the named joint.

        Raises:
            KeyError: If the joint name is not found.
        """
        return self._joint_meta[name]

    def get_active_joint_names(self) -> list[str]:
        """Return sorted list of active (actuated) joint names."""
        return sorted(self._classification.active.keys())

    def get_mimic_joint_names(self) -> list[str]:
        """Return sorted list of mimic joint names."""
        return sorted(self._classification.passive_mimic.keys())

    def get_fixed_joint_names(self) -> list[str]:
        """Return list of fixed joint names."""
        return list(self._classification.fixed)

    def get_link_index(self, link_name: str) -> int:
        """Return the index of the named link.

        Raises:
            KeyError: If the link name is not found.
        """
        return self._link_name_to_index[link_name]

    # ── Computation ──────────────────────────────────────────────────────────

    def compute_mimic_positions(
        self, joint_positions: dict[str, float],
    ) -> dict[str, float]:
        """Compute mimic joint positions from their master joint positions.

        Args:
            joint_positions: Current positions keyed by joint name.

        Returns:
            Dict of ``{mimic_joint: computed_position}`` for mimic joints
            whose master joint has a known position.
        """
        result: dict[str, float] = {}
        for name, meta in self._classification.passive_mimic.items():
            if meta.mimic is None:
                continue
            master_pos = joint_positions.get(meta.mimic.master_joint)
            if master_pos is not None:
                result[name] = (
                    meta.mimic.multiplier * master_pos + meta.mimic.offset)
        return result

    def validate_joints(
        self, received: set[str],
    ) -> tuple[set[str], set[str]]:
        """Compare required (active) joints against received joints.

        Args:
            received: Joint names observed from JointState topics.

        Returns:
            Tuple of (covered, missing) joint name sets.
        """
        required = self._classification.active_names
        covered = required & received
        missing = required - received
        if missing:
            logger.debug(
                'Joint validation: %d/%d covered, %d missing',
                len(covered), len(required), len(missing))
        return covered, missing

    # ── Internal parsing pipeline ────────────────────────────────────────────

    def _parse(self) -> None:
        """Run the full parsing pipeline (called once from __init__)."""
        root = ET.fromstring(self._urdf_xml)
        self._robot_name = root.get('name', '')
        logger.debug('Parsing URDF: robot_name=%s', self._robot_name)

        self._collect_links(root)
        self._parse_joints(root)
        self._build_adjacency_graph()
        self._classify_joints()

        c = self._classification
        logger.info(
            'URDF parsed: %s — %d links, %d joints '
            '(active=%d, mimic=%d, closed_chain=%d, fixed=%d)',
            self._robot_name, len(self._link_nodes), len(self._joint_meta),
            len(c.active), len(c.passive_mimic),
            len(c.passive_closed_chain), len(c.fixed))

    def _collect_links(self, root: ET.Element) -> None:
        """Step 1: Collect all <link> elements into LinkNode list."""
        for link_el in root.findall('link'):
            name = link_el.get('name', '')
            if not name:
                continue
            idx = len(self._link_nodes)
            self._link_name_to_index[name] = idx
            self._link_nodes.append(LinkNode(link_name=name))

    def _parse_joints(self, root: ET.Element) -> None:
        """Step 2: Parse all <joint> elements into JointMeta dict."""
        for joint_el in root.findall('joint'):
            jname = joint_el.get('name', '')
            jtype = joint_el.get('type', '')
            if not jname:
                continue

            parent_el = joint_el.find('parent')
            child_el = joint_el.find('child')
            parent_link = (
                parent_el.get('link', '') if parent_el is not None else '')
            child_link = (
                child_el.get('link', '') if child_el is not None else '')

            # Axis (default: Z)
            axis = (0.0, 0.0, 1.0)
            axis_el = joint_el.find('axis')
            if axis_el is not None:
                xyz = axis_el.get('xyz', '')
                if xyz:
                    parts = xyz.split()
                    if len(parts) == 3:
                        axis = (
                            float(parts[0]),
                            float(parts[1]),
                            float(parts[2]),
                        )

            # Limits
            lower = 0.0
            upper = 0.0
            limit_el = joint_el.find('limit')
            if limit_el is not None:
                lower = float(limit_el.get('lower', '0'))
                upper = float(limit_el.get('upper', '0'))

            # Mimic
            mimic = None
            mimic_el = joint_el.find('mimic')
            if mimic_el is not None:
                master = mimic_el.get('joint', '')
                if master:
                    mimic = MimicParams(
                        master_joint=master,
                        multiplier=float(mimic_el.get('multiplier', '1.0')),
                        offset=float(mimic_el.get('offset', '0.0')),
                    )

            meta = JointMeta(
                name=jname,
                joint_type=jtype,
                parent_link=parent_link,
                child_link=child_link,
                axis=axis,
                lower=lower,
                upper=upper,
                mimic=mimic,
            )
            self._joint_meta[jname] = meta

    def _build_adjacency_graph(self) -> None:
        """Step 3: Build parent/child relationships in the link graph."""
        child_links: set[str] = set()

        for meta in self._joint_meta.values():
            parent_idx = self._link_name_to_index.get(meta.parent_link)
            child_idx = self._link_name_to_index.get(meta.child_link)
            if parent_idx is None or child_idx is None:
                continue

            node = self._link_nodes[child_idx]
            node.parent_index = parent_idx
            node.parent_joint_name = meta.name
            self._link_nodes[parent_idx].child_indices.append(child_idx)

            child_links.add(meta.child_link)

        # Root link: not a child of any joint
        for i, node in enumerate(self._link_nodes):
            if node.link_name not in child_links:
                self._root_index = i
                break

    def _classify_joints(self) -> None:
        """Step 4: Classify joints into active/passive/fixed categories.

        Classification rules:
          - fixed → recorded in classification.fixed
          - mimic (has <mimic> tag) → passive_mimic
          - closed-chain (child_link shared by multiple non-fixed joints,
            first joint kept active, rest passive) → passive_closed_chain
          - everything else → active
        """
        result = JointClassification()

        # Separate fixed from movable, track child_link → joint indices
        movable_joints: list[JointMeta] = []
        child_link_joints: dict[str, list[int]] = defaultdict(list)

        for meta in self._joint_meta.values():
            if meta.joint_type == 'fixed':
                result.fixed.append(meta.name)
                continue

            idx = len(movable_joints)
            movable_joints.append(meta)
            if meta.child_link:
                child_link_joints[meta.child_link].append(idx)

        # Detect closed-chain joints (child_link referenced by 2+ joints)
        closed_chain_indices: set[int] = set()
        for child_link, indices in child_link_joints.items():
            if len(indices) <= 1:
                continue
            logger.debug(
                'Closed-chain candidate: link=%s shared by %d joints',
                child_link, len(indices))
            first_tree = True
            for i in indices:
                if movable_joints[i].mimic is not None:
                    continue
                if first_tree:
                    first_tree = False
                    continue
                closed_chain_indices.add(i)

        # Final classification
        for i, meta in enumerate(movable_joints):
            if meta.mimic is not None:
                result.passive_mimic[meta.name] = meta
            elif i in closed_chain_indices:
                result.passive_closed_chain[meta.name] = JointMeta(
                    name=meta.name,
                    joint_type=meta.joint_type,
                    parent_link=meta.parent_link,
                    child_link=meta.child_link,
                    axis=meta.axis,
                    lower=meta.lower,
                    upper=meta.upper,
                    mimic=meta.mimic,
                    is_closed_chain=True,
                )
            else:
                result.active[meta.name] = meta

        self._classification = result
