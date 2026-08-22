"""Module-level constants, gain schemas, and helper builders for demo_controller_gui.

Phase 1 step 3 moved robot-shape constants (NUM_JOINTS, NUM_HAND_MOTORS,
ROBOT_JOINT_NAMES, HAND_FINGER_GROUPS, HAND_MOTOR_NAMES,
_ROBOT_NAME_TO_IDX, _HAND_NAME_TO_IDX) to ``demo_gui.discovery``.
Phase 2 then dropped the ``CONTROLLER_TYPES`` display-label dict —
controller enumeration now flows through ``demo_gui.catalog`` (live
``/rtc_cm/list_controllers`` query) and labels are produced by
``ControllerCatalog.display_label`` (override map + ``prettify_config_key``
fallback). The remaining constants here are GUI-only, robot-agnostic.

Public surface (imported by app.py):
- TARGET_LABELS, ANGLE_INDICES, JOINT_SPACE
- DUAL_TARGET_SPACE, target_panel_states
- FINGERTIP_NAMES, FORCE_PI_FINGER_NAMES, GRASP_PHASE_NAMES
- GRASP_MODE_PARAM, GRASP_MODE_UNKNOWN, GRASP_MODE_OWNERS, GRASP_MODES,
  grasp_command_enabled, grasp_mode_fg
- _DEFAULT_PRESETS, default_presets_for, preset_hand_targets, _resolve_preset_path
- GAIN_DEFS, GAIN_ROW_NAMES, GAIN_PARAM_DISPATCH,
  GAIN_GROUP_LAYOUT, GAIN_GROUP_PARENT_GRASP, GROUP_SCALARS_PER_ROW
- HAND_TAUFF_GROUP, HAND_TAUFF_SOURCE_PARAM, HAND_TAUFF_SOURCES
- SENSOR_CALIBRATIONS, _CALIB_STATE_NAMES, _CALIB_STATE_COLORS
- value-builders: _set_double, _set_double_array, _set_bool, _set_int,
  _read_only
"""

import copy
import os

from rclpy.parameter import Parameter

from rtc_msgs.msg import CalibrationCommand, CalibrationStatus

TARGET_LABELS = {
    "demo_joint_controller": [
        "q1 (deg)",
        "q2 (deg)",
        "q3 (deg)",
        "q4 (deg)",
        "q5 (deg)",
        "q6 (deg)",
    ],
    "demo_task_controller": [
        "X (m)",
        "Y (m)",
        "Z (m)",
        "Roll (deg) / q4_null",
        "Pitch (deg) / q5_null",
        "Yaw (deg) / q6_null",
    ],
    # WBC consumes joint-space goals like the joint controller; the IK /
    # null-space resolution happens inside TSID on the controller side.
    "demo_wbc_controller": [
        "q1 (deg)",
        "q2 (deg)",
        "q3 (deg)",
        "q4 (deg)",
        "q5 (deg)",
        "q6 (deg)",
    ],
}

ANGLE_INDICES = {
    "demo_joint_controller": [0, 1, 2, 3, 4, 5],
    "demo_task_controller": [3, 4, 5],
    "demo_wbc_controller": [0, 1, 2, 3, 4, 5],
}

# True -> joint space, False -> task space
JOINT_SPACE = {
    "demo_joint_controller": True,
    "demo_task_controller": False,
    "demo_wbc_controller": True,
}

# Controllers that accept a joint posture goal AND a task-space SE3 goal at
# once. WBC regulates the arm posture (nullspace reference) and a commanded
# EE SE3 jog independently (demo_wbc_controller DrainTargetSlot — "posture and
# the commanded SE3 stay independent"), so its GUI enables both target panels
# and Send Command publishes both RobotTarget messages. Others stay
# single-space per JOINT_SPACE.
DUAL_TARGET_SPACE = {"demo_wbc_controller"}


def target_panel_states(ctrl_idx: str) -> tuple[bool, bool]:
    """Return ``(joint_panel_on, task_panel_on)`` for a controller.

    Dual-space controllers (``DUAL_TARGET_SPACE``) enable both panels; the
    rest follow the binary ``JOINT_SPACE`` flag (joint XOR task).
    """
    if ctrl_idx in DUAL_TARGET_SPACE:
        return True, True
    is_joint = JOINT_SPACE.get(ctrl_idx, True)
    return is_joint, not is_joint


# Fingertip names matching controller order (4 fingertips)
FINGERTIP_NAMES = ["Thumb", "Index", "Middle", "Ring"]

# Force-PI grasp controller: 3 fingers only (no ring)
FORCE_PI_FINGER_NAMES = ["Thumb", "Index", "Middle"]

# GraspPhase enum → (display_text, bg_color, fg_color)
# Source: GraspState.msg (Force-PI grasp controllers).
GRASP_PHASE_NAMES = {
    0: ("IDLE", "#585b70", "#cdd6f4"),
    1: ("APPROACHING", "#89b4fa", "#1e1e2e"),
    2: ("CONTACT", "#f9e2af", "#1e1e2e"),
    3: ("FORCE CTRL", "#fab387", "#1e1e2e"),
    4: ("HOLDING", "#a6e3a1", "#1e1e2e"),
    5: ("RELEASING", "#f38ba8", "#1e1e2e"),
}

# WbcPhase enum → (display_text, bg_color, fg_color).
# Source: WbcState.msg (TSID-based WBC controllers; 6 reachable states +
# slots 2 & 5 reserved). Indices must match WbcState.PHASE_* constants and
# the C++ integrated_bringup::WbcPhase enum. Slot 2 (PRE_GRASP, merged into
# APPROACH) and slot 5 (RETREAT) are reserved deprecated — no controller
# publishes them; kept here so the display fallback for stale rosbags stays
# readable.
WBC_PHASE_NAMES = {
    0: ("IDLE", "#585b70", "#cdd6f4"),
    1: ("APPROACH", "#89b4fa", "#1e1e2e"),
    2: ("PRE-GRASP (deprecated)", "#6c7086", "#bac2de"),
    3: ("CLOSURE", "#fab387", "#1e1e2e"),
    4: ("HOLD", "#a6e3a1", "#1e1e2e"),
    5: ("RETREAT (deprecated)", "#6c7086", "#bac2de"),
    6: ("RELEASE", "#f38ba8", "#1e1e2e"),
    7: ("FALLBACK", "#f9e2af", "#1e1e2e"),
}

# ── Hand grasp mode (live mirror of the controller's active mode) ──────────
# The demo joint/task controllers declare `grasp_controller_type` as a string
# parameter seeded from the YAML value LoadConfig resolved
# (integrated_bringup/src/controllers/{joint,task}/parameters.cpp). It is
# *writable at runtime*: the controller accepts a change while the hand is
# quiet (no contact_stop latch, no force_pi grasp in progress) and otherwise
# refuses it with a reason. So this is a live value, not a configure-time
# constant — the GUI reads it back after every set attempt rather than
# assuming the mode it asked for.
GRASP_MODE_PARAM = "grasp_controller_type"
GRASP_MODE_FORCE_PI = "force_pi"

# Sentinel for "not fetched yet / fetch failed". Deliberately not one of the
# whitelist values so it can never be mistaken for a real mode.
GRASP_MODE_UNKNOWN = ""

# The modes the controller accepts, in enum-declaration order. SSoT is
# `ParseGraspHandMode` (integrated_bringup/src/support/demo_shared_config.cpp);
# kept literal here so a GUI that offered a mode the controller has never heard
# of would show up as a rejected set rather than agreeing with itself. An
# off-whitelist request is refused by the controller with its own reason, which
# makes this list a convenience for the operator, not a second gate.
GRASP_MODES = ("contact_stop", GRASP_MODE_FORCE_PI, "none")

# Controllers that declare the parameter. demo_wbc_controller does NOT: it runs
# its own 6-state WbcPhase FSM and handles GraspCommand directly (lifecycle.cpp
# grasp_command_srv_), so its Grasp/Release work regardless of the mode.
GRASP_MODE_OWNERS = frozenset({"demo_joint_controller", "demo_task_controller"})


def grasp_command_enabled(ctrl: str, mode: str) -> tuple[bool, str]:
    """Return ``(buttons_enabled, status_text)`` for the Grasp tab.

    The Grasp/Release buttons drive the controller's ``~/grasp_command`` srv,
    which only reaches the hand while the controller is *running*
    ``grasp_controller_type: "force_pi"``. In the other modes the srv
    replies with a rejection and the hand does nothing, so the buttons are
    disabled and the reason is put on screen instead of a fixed hint that is
    wrong in the common case.

    Fail-open on ``GRASP_MODE_UNKNOWN``: an unreachable parameter service says
    nothing about what the controller would accept, and the GUI must not become
    a second gate that blocks a command the controller would have honoured. The
    controller already answers with a precise reason when it will not act.
    """
    if ctrl not in GRASP_MODE_OWNERS:
        # WBC (or any future controller with its own grasp path) — no parameter
        # to consult, and the mode does not gate its srv.
        return True, "own grasp FSM (grasp_controller_type 무관)"
    if mode == GRASP_MODE_UNKNOWN:
        return True, "mode: unknown (parameter service unreachable)"
    if mode == GRASP_MODE_FORCE_PI:
        return True, f"mode: {mode}"
    return False, f"mode: {mode} — Grasp/Release 는 force_pi 에서만 동작"


# Status-label colours keyed by the same three cases as grasp_command_enabled.
GRASP_MODE_FG_ACTIVE = "#a6e3a1"  # force_pi / own-FSM — commands will act
GRASP_MODE_FG_BLOCKED = "#f9e2af"  # contact_stop / none — commands are inert
GRASP_MODE_FG_UNKNOWN = "#9399b2"  # not fetched / service down


def grasp_mode_fg(ctrl: str, mode: str) -> str:
    """Foreground colour for the Grasp tab's mode label."""
    if ctrl in GRASP_MODE_OWNERS and mode == GRASP_MODE_UNKNOWN:
        return GRASP_MODE_FG_UNKNOWN
    enabled, _ = grasp_command_enabled(ctrl, mode)
    return GRASP_MODE_FG_ACTIVE if enabled else GRASP_MODE_FG_BLOCKED


# Default hand presets (positions in degrees for readability, converted to rad
# at runtime). Two rosters keyed by hand DoF — a preset's positions_deg length
# MUST equal the active RobotShape.hand_dof or the publish is rejected (see
# preset_hand_targets / issue #137 finding 3). The 10-DoF set covers the UR5e
# hand variants (p1a/p1b); the 16-DoF set covers the LEAP hand (iiwa7_leap).
_DEFAULT_PRESETS_10 = {
    "open_flat": {
        "type": "open",
        "positions_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "grasp_time": 1.0,
    },
    "power_grasp": {
        "type": "close",
        "positions_deg": [30.0, 60.0, 50.0, 15.0, 70.0, 50.0, 15.0, 70.0, 50.0, 60.0],
        "grasp_time": 2.0,
    },
    "pinch_grasp": {
        "type": "close",
        "positions_deg": [40.0, 50.0, 40.0, 20.0, 55.0, 40.0, 0.0, 0.0, 0.0, 0.0],
        "grasp_time": 1.5,
    },
}

# 16-DoF LEAP hand. positions_deg map by index onto the profile's
# hand_motor_names order (thumb 4, index 4, middle 4, ring 4 — see
# RobotShape.default_iiwa7_leap). Values are demo defaults, not tuned targets;
# the point is a length-matched, usable preset set so iiwa7_leap isn't left
# with zero presets.
_DEFAULT_PRESETS_16 = {
    "open_flat": {
        "type": "open",
        "positions_deg": [0.0] * 16,
        "grasp_time": 1.0,
    },
    "power_grasp": {
        "type": "close",
        # uniform moderate flexion across all four fingers
        "positions_deg": [30.0] * 16,
        "grasp_time": 2.0,
    },
    "pinch_grasp": {
        "type": "close",
        # thumb + index emphasis, middle/ring relaxed
        "positions_deg": [40.0] * 4 + [40.0] * 4 + [0.0] * 4 + [0.0] * 4,
        "grasp_time": 1.5,
    },
}

# Backwards-compatible alias — historical name for the 10-DoF roster.
_DEFAULT_PRESETS = _DEFAULT_PRESETS_10


def default_presets_for(hand_dof: int) -> dict:
    """Robot-appropriate default hand presets, keyed by hand DoF.

    16-DoF → LEAP (iiwa7_leap); otherwise the 10-DoF UR5e-hand roster. Returns
    a deep copy so the caller can persist/edit without mutating the module
    constants. For any DoF without a dedicated roster the 10-DoF set is
    returned as a best effort — the length guard in ``preset_hand_targets``
    still rejects a mismatched publish, so no malformed target escapes.
    """
    roster = _DEFAULT_PRESETS_16 if hand_dof == 16 else _DEFAULT_PRESETS_10
    return copy.deepcopy(roster)


def preset_hand_targets(preset: dict, hand_dof: int) -> list[float] | None:
    """Return a preset's ``positions_deg`` iff its length matches ``hand_dof``.

    Guards issue #137 finding 3: a 10-DoF preset must never be published
    against a 16-name hand (or vice versa), which would emit a ``RobotTarget``
    with mismatched ``joint_names`` / ``joint_target`` lengths. Returns None on
    mismatch so the caller can skip the hand publish with a clear warning.
    """
    positions = preset.get("positions_deg", [])
    if len(positions) != hand_dof:
        return None
    return [float(p) for p in positions]


def _resolve_preset_path(hand_group: str = "") -> str:
    """Resolve the hand-presets JSON path in the workspace logging directory.

    Scoped per hand device group (``hand_presets_<hand_group>.json``) so each
    robot keeps its own roster and a 10-DoF file saved for one hand can't be
    loaded against a 16-DoF hand. Falls back to the legacy unscoped
    ``hand_presets.json`` when no group is given.
    """
    from rtc_tools.utils.session_dir import get_session_dir, resolve_logging_root

    session_dir = get_session_dir()
    if session_dir:
        logging_root = os.path.dirname(session_dir)
    else:
        logging_root = resolve_logging_root()
    os.makedirs(logging_root, exist_ok=True)
    filename = f"hand_presets_{hand_group}.json" if hand_group else "hand_presets.json"
    return os.path.join(logging_root, filename)


# Hand τ_ff (kPdFeedforward) widgets. Four numeric/bool params ride the
# float gain pipeline (GAIN_DEFS / GAIN_PARAM_DISPATCH below); the fifth,
# `hand_tauff_source`, is a string enum the controller validates and so
# cannot be a float widget — app.py renders it as a standalone combobox in
# the HAND_TAUFF_GROUP box. SSoT for names/types: wbc/parameters.cpp
# OnGainParametersSet.
HAND_TAUFF_GROUP = "Hand τ_ff"
HAND_TAUFF_SOURCE_PARAM = "hand_tauff_source"
HAND_TAUFF_SOURCES = ["gravity_comp", "tsid_tau"]

# Gain definitions per controller
# Each entry: (label, size, defaults, is_bool, group)
# The flat tuple ORDER is the on-the-wire layout sent to the controller;
# the `group` field only drives visual placement in the GUI so that arm
# and hand trajectory params (which are interleaved on the wire) can be
# displayed in functional clusters.
#   DemoJoint wire order: [robot_traj_speed, hand_traj_speed,
#       robot_max_traj_vel, hand_max_traj_vel,
#       grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips,
#       grasp_cmd, grasp_target_force]
#   DemoTask wire order:  [kp_translation x3, kp_rotation x3,
#       singularity_threshold, max_damping, null_kp,
#       enable_null_space(0/1), control_6dof(0/1),
#       traj_speed, traj_angular_speed, hand_traj_speed,
#       max_traj_vel, max_traj_angular_vel, hand_max_traj_vel,
#       grasp_contact_thresh, grasp_force_thresh, grasp_min_fingertips,
#       grasp_cmd, grasp_target_force]
GAIN_DEFS = {
    "demo_joint_controller": [
        ("robot_traj_speed", 1, [1.0], False, "Arm Trajectory"),
        ("hand_traj_speed", 1, [1.0], False, "Hand Trajectory"),
        ("robot_max_traj_vel", 1, [3.14], False, "Arm Trajectory"),
        ("hand_max_traj_vel", 1, [2.0], False, "Hand Trajectory"),
        ("grasp_contact_thresh", 1, [0.5], False, "Grasp Detection"),
        ("grasp_force_thresh", 1, [1.0], False, "Grasp Detection"),
        ("grasp_min_fingertips", 1, [2], False, "Grasp Detection"),
    ],
    "demo_task_controller": [
        ("kp_translation", 3, [5.0] * 3, False, "CLIK Gains"),
        ("kp_rotation", 3, [2.0] * 3, False, "CLIK Gains"),
        # §6.5 DLS pair (#282) — replaced the retired constant-lambda
        # "damping" parameter, which the controller no longer declares.
        ("singularity_threshold", 1, [0.02], False, "CLIK Gains"),
        ("max_damping", 1, [0.05], False, "CLIK Gains"),
        ("null_kp", 1, [0.5], False, "CLIK Gains"),
        ("null_space", 1, [1], True, "CLIK Gains"),
        ("control_6dof", 1, [0], True, "CLIK Gains"),
        ("traj_speed", 1, [0.1], False, "Arm Trajectory"),
        ("traj_angular_speed", 1, [0.5], False, "Arm Trajectory"),
        ("hand_traj_speed", 1, [1.0], False, "Hand Trajectory"),
        ("max_traj_vel", 1, [0.5], False, "Arm Trajectory"),
        ("max_traj_angular_vel", 1, [1.0], False, "Arm Trajectory"),
        ("hand_max_traj_vel", 1, [2.0], False, "Hand Trajectory"),
        ("grasp_contact_thresh", 1, [0.5], False, "Grasp Detection"),
        ("grasp_force_thresh", 1, [1.0], False, "Grasp Detection"),
        ("grasp_min_fingertips", 1, [2], False, "Grasp Detection"),
    ],
    # DemoWbc wire order: [arm_traj_speed, hand_traj_speed,
    #     arm_max_traj_vel(RO), hand_max_traj_vel(RO)]
    # TSID weights (se3/force/posture) and MPC (mpc_enable/riccati_gain_scale)
    # are intentionally not exposed here — tune them via YAML / ros2 param set.
    # Parameters live on /demo_wbc_controller/<name> (LifecycleNode); see
    # integrated_bringup/src/controllers/wbc/parameters.cpp for the
    # declare_parameter / OnGainParametersSet wiring.
    "demo_wbc_controller": [
        ("arm_traj_speed", 1, [0.5], False, "Arm Trajectory"),
        ("hand_traj_speed", 1, [1.0], False, "Hand Trajectory"),
        ("arm_max_traj_vel", 1, [2.0], False, "Arm Trajectory"),
        ("hand_max_traj_vel", 1, [4.0], False, "Hand Trajectory"),
        # Stage C-3 hand τ_ff overlay (kPdFeedforward). Defaults mirror the
        # controller's Gains struct (demo_wbc_controller.hpp). The string-enum
        # companion `hand_tauff_source` cannot ride this float pipeline — app.py
        # renders it as a standalone combobox in the HAND_TAUFF_GROUP box.
        ("hand_tauff_enable", 1, [0], True, HAND_TAUFF_GROUP),
        ("hand_tauff_gravity_gain", 1, [1.0], False, HAND_TAUFF_GROUP),
        ("hand_tauff_closure_bias", 1, [0.0], False, HAND_TAUFF_GROUP),
        ("hand_tauff_max", 1, [5.0], False, HAND_TAUFF_GROUP),
    ],
}

GAIN_ROW_NAMES = {
    "demo_joint_controller": {},
    "demo_task_controller": {
        "kp_translation": ["x", "y", "z"],
        "kp_rotation": ["rx", "ry", "rz"],
    },
    "demo_wbc_controller": {},
}


# Maps each GAIN_DEFS entry name → (declared parameter name, value-builder).
# value-builder is called with the flat list of widget values for that entry
# and must return either a `Parameter.Type` + native value pair, or None to
# skip dispatch (used for read-only caps that the controller would reject).
# `grasp_command` and `grasp_target_force` are not parameters — they go
# through the GraspCommand srv on Apply Grasp / Release.
def _set_double(v):
    return (Parameter.Type.DOUBLE, float(v[0]))


def _set_double_array(v):
    return (Parameter.Type.DOUBLE_ARRAY, [float(x) for x in v])


def _set_bool(v):
    return (Parameter.Type.BOOL, bool(v[0] > 0.5))


def _set_int(v):
    return (Parameter.Type.INTEGER, int(v[0]))


def _read_only(_):
    return None  # skip — controller has read_only=true on cap


GAIN_PARAM_DISPATCH: dict[str, dict[str, tuple[str, callable]]] = {
    "demo_joint_controller": {
        "robot_traj_speed": ("robot_trajectory_speed", _set_double),
        "hand_traj_speed": ("hand_trajectory_speed", _set_double),
        "robot_max_traj_vel": ("robot_max_traj_velocity", _read_only),
        "hand_max_traj_vel": ("hand_max_traj_velocity", _read_only),
        "grasp_contact_thresh": ("grasp_contact_threshold", _set_double),
        "grasp_force_thresh": ("grasp_force_threshold", _set_double),
        "grasp_min_fingertips": ("grasp_min_fingertips", _set_int),
    },
    "demo_task_controller": {
        "kp_translation": ("kp_translation", _set_double_array),
        "kp_rotation": ("kp_rotation", _set_double_array),
        "singularity_threshold": ("singularity_threshold", _set_double),
        "max_damping": ("max_damping", _set_double),
        "null_kp": ("null_kp", _set_double),
        "null_space": ("enable_null_space", _set_bool),
        "control_6dof": ("control_6dof", _set_bool),
        "traj_speed": ("trajectory_speed", _set_double),
        "traj_angular_speed": ("trajectory_angular_speed", _set_double),
        "hand_traj_speed": ("hand_trajectory_speed", _set_double),
        "max_traj_vel": ("max_traj_velocity", _read_only),
        "max_traj_angular_vel": ("max_traj_angular_velocity", _read_only),
        "hand_max_traj_vel": ("hand_max_traj_velocity", _read_only),
        "grasp_contact_thresh": ("grasp_contact_threshold", _set_double),
        "grasp_force_thresh": ("grasp_force_threshold", _set_double),
        "grasp_min_fingertips": ("grasp_min_fingertips", _set_int),
    },
    "demo_wbc_controller": {
        "arm_traj_speed": ("arm_trajectory_speed", _set_double),
        "hand_traj_speed": ("hand_trajectory_speed", _set_double),
        "arm_max_traj_vel": ("arm_max_traj_velocity", _read_only),
        "hand_max_traj_vel": ("hand_max_traj_velocity", _read_only),
        "hand_tauff_enable": ("hand_tauff_enable", _set_bool),
        "hand_tauff_gravity_gain": ("hand_tauff_gravity_gain", _set_double),
        "hand_tauff_closure_bias": ("hand_tauff_closure_bias", _set_double),
        "hand_tauff_max": ("hand_tauff_max", _set_double),
        # Grasp detection thresholds (layer-d): capability-aware. contact_thresh
        # is consulted only on sensor A paths (udp_hand_native +
        # ft_inferencer.enabled); force_thresh + min_fingertips are common
        # across sensor types. UI tooltip noting this is future work.
        "grasp_contact_thresh": ("grasp_contact_threshold", _set_double),
        "grasp_force_thresh": ("grasp_force_threshold", _set_double),
        "grasp_min_fingertips": ("grasp_min_fingertips", _set_int),
    },
}

# Visual placement of gain groups inside the Control tab "Gains" panel.
# Each inner list is a row of groups packed side-by-side.
# Groups not listed here fall through to the default one-per-row layout.
GAIN_GROUP_LAYOUT = {
    "demo_joint_controller": [
        ["Arm Trajectory", "Hand Trajectory"],
    ],
    "demo_task_controller": [
        ["CLIK Gains"],
        ["Arm Trajectory", "Hand Trajectory"],
    ],
    "demo_wbc_controller": [
        ["Arm Trajectory", "Hand Trajectory"],
        [HAND_TAUFF_GROUP],
    ],
}

# ── demo_compliance_controller mirrors demo_task_controller (#469 S2) ────────
#
# The compliance controller is a copy of the task controller with renamed
# identifiers until the §7 admittance law lands (S3), so it declares the same
# parameters in the same wire order, takes the same task-space goal and owns the
# same grasp-mode parameter. Nine tables above key on the controller name, and
# adding it to each by hand would be nine chances to get one wrong today plus
# nine places to keep in step tomorrow — so the mirror is stated once, here,
# where it can also say why it is a mirror.
#
# S3 REPLACES THIS, IT DOES NOT EXTEND IT: when the controller gains admittance
# gains and a wrench source, drop the affected table out of this loop and give it
# an explicit demo_compliance_controller entry. A silently extended mirror would
# quietly claim the two controllers still take the same parameters.
#
# deepcopy, not aliasing: an S3 edit to one table must not reach through and
# change the shipped controller's row.
_COMPLIANCE_MIRRORS = "demo_task_controller"
for _table in (
    TARGET_LABELS,
    ANGLE_INDICES,
    JOINT_SPACE,
    GAIN_DEFS,
    GAIN_ROW_NAMES,
    GAIN_PARAM_DISPATCH,
    GAIN_GROUP_LAYOUT,
):
    _table["demo_compliance_controller"] = copy.deepcopy(_table[_COMPLIANCE_MIRRORS])

# Not DUAL_TARGET_SPACE: like the task controller it takes one space at a time
# (JOINT_SPACE False above), and only WBC regulates posture and an SE3 jog at
# once. It IS a grasp-mode owner — the hand lane is duplicated verbatim, so it
# declares grasp_controller_type exactly as its sibling does.
GRASP_MODE_OWNERS = GRASP_MODE_OWNERS | {"demo_compliance_controller"}

# Groups whose editable widgets are rendered in the Grasp tab's
# "Grasp Detection" section instead of the Control tab's Gains panel.
# For these groups the applied mirror is skipped (the per-entry
# applied_labels slot is an empty list so zip-based iteration in
# `_update_applied_display` stays aligned).
GAIN_GROUP_PARENT_GRASP: set[str] = {"Grasp Detection"}

# Per-group override for the maximum number of scalar inputs placed in
# a single row inside the group box (default: 2 from SCALARS_PER_ROW).
GROUP_SCALARS_PER_ROW = {
    "Grasp Detection": 3,
    # Hand τ_ff: enable + gravity_gain + closure_bias + max + source combobox
    # on a single row.
    HAND_TAUFF_GROUP: 5,
}

# Sensor calibration entries displayed in the Control tab.
# Add a new dict here to expose a new sensor's calibration to the GUI;
# it must also be handled by UdpHandController::DispatchCalibrationRequest()
# and reported by UdpHandController::GetCalibrationStatus() on the driver side.
SENSOR_CALIBRATIONS = [
    {
        "label": "Barometer Bias",
        "sensor_type": CalibrationCommand.SENSOR_BAROMETER,
        "hint": "~1s, keep fingertips still",
    },
]

_CALIB_STATE_NAMES = {
    CalibrationStatus.STATE_IDLE: "IDLE",
    CalibrationStatus.STATE_RUNNING: "RUNNING",
    CalibrationStatus.STATE_COMPLETE: "COMPLETE",
    CalibrationStatus.STATE_FAILED: "FAILED",
}

_CALIB_STATE_COLORS = {
    CalibrationStatus.STATE_IDLE: "#9399b2",
    CalibrationStatus.STATE_RUNNING: "#f9e2af",
    CalibrationStatus.STATE_COMPLETE: "#a6e3a1",
    CalibrationStatus.STATE_FAILED: "#f38ba8",
}
