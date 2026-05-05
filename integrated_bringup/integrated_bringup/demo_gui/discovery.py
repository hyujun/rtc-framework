"""Runtime robot-shape discovery for demo_controller_gui (Phase 1).

The GUI used to hard-code arm DoF (6) and hand motor count (10) plus the
finger-group breakdown for the assm_v1 hand. Phase 1 replaces those
constants with a `RobotShape` instance that:

  1. starts from a sensible default at __init__ time so the Tk widget
     tree builds immediately (no "waiting for first message" placeholder),
  2. is *replaced* the first time a `GuiPosition` message arrives whose
     `joint_names` describe a different schema, at which point the
     widgets are torn down and rebuilt against the new shape.

This file defines the data class only; the rebuild is driven from
``app.py`` so it can sit on the Tk thread.
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
        """Return True iff a ``GuiPosition`` carrying these joint_names is
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
