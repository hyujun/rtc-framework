"""Robot-shape + profile definitions for demo_controller_gui.

The GUI used to hard-code arm DoF (6) and hand motor count (10) plus the
finger-group breakdown for the assm_v1 hand. Those constants now live in a
``RobotShape`` instance, and the GUI picks one at launch via ``--robot``:

  1. ``RobotProfile.for_robot(key)`` (``ROBOT_PROFILES`` registry) returns
     the static ``RobotShape`` + TCP tf frames for the selected robot
     (``ur5e_hand`` 6+10, ``iiwa7_leap`` 7+16). Widgets size to that shape
     immediately at ``__init__`` — no "waiting for first message".
  2. ``DemoControllerGUI._warn_shape_mismatch_once`` *advisorily* warns once
     per unique mismatch when a ``JointState`` arrives whose ``name`` span
     disagrees with the chosen profile (wrong ``--robot``, or a motor order
     differing from the wire order). The fix is to relaunch with the right
     ``--robot`` value — widgets are sized once, not rebuilt live.

A future sprint may replace the warn-only path with a live Tk widget rebuild
against the observed shape — at which point ``RobotShape.from_joint_names``
becomes the actual schema source instead of a future-use helper.

This file defines the data classes + helpers only; the warn dispatch is
driven from ``app.py`` so it can sit on the Tk thread.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Order matters: the first prefix that matches a motor name wins. The
# fallback bucket "Other" catches anything not covered by the four named
# groups below (e.g. a hypothetical pinky_* / wrist_* motor).
_FINGER_PREFIX_ORDER: tuple[tuple[str, str], ...] = (
    ("thumb_", "Thumb"),
    ("index_", "Index"),
    ("middle_", "Middle"),
    ("ring_", "Ring"),
    ("pinky_", "Pinky"),
    ("little_", "Little"),
)


@dataclass(frozen=True)
class RobotShape:
    """Runtime-discovered DoF + naming for the active robot/hand pair.

    Carries no controller-specific or robot-specific state — just the
    joint-name spans the GUI needs to size widgets and to fill
    ``RobotTarget.joint_names`` correctly when publishing.

    The class is frozen so it can be safely shared between the rclpy
    executor thread (where the discovery callback fires) and the Tk
    thread (which reads it during rebuild).
    """

    arm_joint_names: tuple[str, ...]
    hand_motor_names: tuple[str, ...]
    # [(group_label, (motor_name, motor_name, ...)), ...]. Always non-empty
    # if hand_motor_names is non-empty; an "Other" bucket catches motors
    # whose name has no recognised finger prefix.
    hand_finger_groups: tuple[tuple[str, tuple[str, ...]], ...]

    # Lookup tables built once. ``field(repr=False)`` keeps __repr__ readable.
    arm_name_to_idx: dict[str, int] = field(default_factory=dict, repr=False)
    hand_name_to_idx: dict[str, int] = field(default_factory=dict, repr=False)

    def __post_init__(self) -> None:
        # ``frozen=True`` blocks normal attribute assignment; use object.__setattr__
        # to populate the lookup caches once.
        object.__setattr__(
            self, "arm_name_to_idx", {n: i for i, n in enumerate(self.arm_joint_names)}
        )
        object.__setattr__(
            self, "hand_name_to_idx", {n: i for i, n in enumerate(self.hand_motor_names)}
        )

    @property
    def arm_dof(self) -> int:
        return len(self.arm_joint_names)

    @property
    def hand_dof(self) -> int:
        return len(self.hand_motor_names)

    def matches_message(self, joint_names: list[str], expected_dof: int) -> bool:
        """Return True iff a ``JointState`` carrying these joint_names is
        consistent with this shape — i.e. the GUI does not need to rebuild.

        ``expected_dof`` selects which span to compare against (arm_dof or
        hand_dof). The comparison is exact (same order, same names) so that
        permutation differences also force a rebuild.
        """
        return tuple(joint_names) == (
            self.arm_joint_names if expected_dof == self.arm_dof else self.hand_motor_names
        )

    @classmethod
    def default_ur5e_assm(cls) -> RobotShape:
        """Pre-Phase-1 schema (UR5e + assm_v1 hand). Used as the GUI's
        startup default so widgets build immediately at app launch."""
        arm = (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )
        hand_groups = (
            ("Thumb", ("thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_fe")),
            ("Index", ("index_mcp_aa", "index_mcp_fe", "index_dip_fe")),
            ("Middle", ("middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe")),
            ("Ring", ("ring_mcp_fe",)),
        )
        hand_flat = tuple(m for _, motors in hand_groups for m in motors)
        return cls(
            arm_joint_names=arm,
            hand_motor_names=hand_flat,
            hand_finger_groups=hand_groups,
        )

    @classmethod
    def default_iiwa7_leap(cls) -> RobotShape:
        """KUKA iiwa7 (7-DoF) + LEAP Hand (16 motors) schema.

        Joint names mirror what the controller publishes on
        ``/rtc_cm/<group>/joint_states``: arm ``A1..A7`` and the bare-number
        LEAP motor ids. The motor *order* here matches the live wire order
        (thumb 12-15 first, then index/middle/ring 0-11) — not the ascending
        ``"0".."15"`` of ``config/iiwa7_leap/mujoco_simulator.yaml``, which is
        the backend's state order, not the controller's republished order — so
        ``matches_message`` passes without a spurious mismatch warning.

        The bare-number motor ids match no finger prefix, so the finger
        grouping is given explicitly here (4 motors per finger) instead of
        being inferred by ``group_hand_motors``. Position correctness does
        not depend on this order (the joint-state callback reorders by name);
        the order only sets the hand-tab layout.
        """
        arm = ("A1", "A2", "A3", "A4", "A5", "A6", "A7")
        hand_groups = (
            ("Thumb", ("12", "13", "14", "15")),
            ("Index", ("0", "1", "2", "3")),
            ("Middle", ("4", "5", "6", "7")),
            ("Ring", ("8", "9", "10", "11")),
        )
        hand_flat = tuple(m for _, motors in hand_groups for m in motors)
        return cls(
            arm_joint_names=arm,
            hand_motor_names=hand_flat,
            hand_finger_groups=hand_groups,
        )

    @classmethod
    def from_joint_names(
        cls,
        arm_joint_names: list[str] | tuple[str, ...] | None,
        hand_motor_names: list[str] | tuple[str, ...] | None,
    ) -> RobotShape:
        """Build a shape from runtime-observed joint name spans.

        Either side may be None or empty — in that case the corresponding
        span from ``default_ur5e_assm`` is reused. Hand finger grouping
        is inferred from motor name prefixes (``thumb_`` / ``index_`` /
        ``middle_`` / ``ring_`` / ``pinky_`` / ``little_``); anything
        that doesn't match falls into an ``Other`` bucket so it remains
        visible in the GUI.
        """
        default = cls.default_ur5e_assm()
        arm = tuple(arm_joint_names) if arm_joint_names else default.arm_joint_names
        hand = tuple(hand_motor_names) if hand_motor_names else default.hand_motor_names
        return cls(
            arm_joint_names=arm,
            hand_motor_names=hand,
            hand_finger_groups=group_hand_motors(hand),
        )


@dataclass(frozen=True)
class RobotProfile:
    """Static per-robot bundle the GUI selects via ``--robot``.

    Carries the joint-name :class:`RobotShape` plus the tf frame pair the
    TCP-pose lookup uses (``<tcp_parent> → <tcp_child>``). Both frames are
    robot-specific: ur5e publishes ``base → tool0_actual``; iiwa7 drives
    ``link_0 → ee_link``.
    """

    shape: RobotShape
    tcp_parent: str
    tcp_child: str

    @classmethod
    def for_robot(cls, key: str) -> RobotProfile:
        """Look up a profile by ``--robot`` key.

        Raises :class:`ValueError` on an unknown key — the caller must not
        silently fall back to a default robot, otherwise the GUI would size
        widgets for the wrong arm/hand without telling the user.
        """
        try:
            return ROBOT_PROFILES[key]
        except KeyError:
            valid = ", ".join(sorted(ROBOT_PROFILES))
            raise ValueError(f"unknown robot '{key}'; expected one of: {valid}") from None


# key matches the config/<key>/ directory name so --robot mirrors the bringup config.
ROBOT_PROFILES: dict[str, RobotProfile] = {
    "ur5e_hand": RobotProfile(
        shape=RobotShape.default_ur5e_assm(),
        tcp_parent="base",
        tcp_child="tool0_actual",
    ),
    "iiwa7_leap": RobotProfile(
        shape=RobotShape.default_iiwa7_leap(),
        tcp_parent="link_0",
        tcp_child="ee_link",
    ),
}


def group_hand_motors(
    motor_names: tuple[str, ...] | list[str],
) -> tuple[tuple[str, tuple[str, ...]], ...]:
    """Bucket motor names by recognised finger prefix.

    Preserves the insertion order of ``motor_names`` within each bucket
    (so the on-the-wire order of hand_motor_names is unchanged for
    sensible inputs that already group fingers contiguously). Buckets
    that end up empty are dropped. Anything not matching a known prefix
    lands in ``Other`` so users can still see + drive those motors.
    """
    buckets: dict[str, list[str]] = {}
    order: list[str] = []
    for name in motor_names:
        label = "Other"
        for prefix, group_label in _FINGER_PREFIX_ORDER:
            if name.startswith(prefix):
                label = group_label
                break
        if label not in buckets:
            buckets[label] = []
            order.append(label)
        buckets[label].append(name)
    return tuple((label, tuple(buckets[label])) for label in order)
