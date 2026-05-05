"""Module-level constants, gain schemas, and helper builders for demo_controller_gui.

Split out of scripts/demo_controller_gui.py (Phase 0a). Behavior identical —
identifier names and values preserved verbatim so importers see the same
attributes as before.

Public surface (re-exported by app.py via `from .config import *`):
- CONTROLLER_TYPES, TARGET_LABELS, ANGLE_INDICES, JOINT_SPACE
- NUM_JOINTS, NUM_HAND_MOTORS, ROBOT_JOINT_NAMES,
  HAND_FINGER_GROUPS, HAND_MOTOR_NAMES,
  _ROBOT_NAME_TO_IDX, _HAND_NAME_TO_IDX
- FINGERTIP_NAMES, FORCE_PI_FINGER_NAMES, GRASP_PHASE_NAMES
- _DEFAULT_PRESETS, _resolve_preset_path
- GAIN_DEFS, GAIN_ROW_NAMES, GAIN_PARAM_DISPATCH,
  GAIN_GROUP_LAYOUT, GAIN_GROUP_PARENT_GRASP, GROUP_SCALARS_PER_ROW
- SENSOR_CALIBRATIONS, _CALIB_STATE_NAMES, _CALIB_STATE_COLORS
- value-builders: _set_double, _set_double_array, _set_bool, _set_int,
  _read_only
"""

import os

from rclpy.parameter import Parameter

from rtc_msgs.msg import CalibrationCommand, CalibrationStatus

CONTROLLER_TYPES = {
    "demo_joint_controller": "Demo Joint Controller",
    "demo_task_controller": "Demo Task Controller",
}

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
}

ANGLE_INDICES = {
    "demo_joint_controller": [0, 1, 2, 3, 4, 5],
    "demo_task_controller": [3, 4, 5],
}

# True -> joint space, False -> task space
JOINT_SPACE = {"demo_joint_controller": True, "demo_task_controller": False}

NUM_JOINTS = 6
NUM_HAND_MOTORS = 10

ROBOT_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Hand finger groups
HAND_FINGER_GROUPS = [
    ("Thumb", ["thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_fe"]),
    ("Index", ["index_mcp_aa", "index_mcp_fe", "index_dip_fe"]),
    ("Middle", ["middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe"]),
    ("Ring", ["ring_mcp_fe"]),
]

HAND_MOTOR_NAMES = []
for _grp_name, _motors in HAND_FINGER_GROUPS:
    HAND_MOTOR_NAMES.extend(_motors)

# Lookup tables for joint_names → display index mapping
_ROBOT_NAME_TO_IDX = {n: i for i, n in enumerate(ROBOT_JOINT_NAMES)}
_HAND_NAME_TO_IDX = {n: i for i, n in enumerate(HAND_MOTOR_NAMES)}

# Fingertip names matching controller order (4 fingertips)
FINGERTIP_NAMES = ["Thumb", "Index", "Middle", "Ring"]

# Force-PI grasp controller: 3 fingers only (no ring)
FORCE_PI_FINGER_NAMES = ["Thumb", "Index", "Middle"]

# GraspPhase enum → (display_text, bg_color, fg_color)
GRASP_PHASE_NAMES = {
    0: ("IDLE", "#585b70", "#cdd6f4"),
    1: ("APPROACHING", "#89b4fa", "#1e1e2e"),
    2: ("CONTACT", "#f9e2af", "#1e1e2e"),
    3: ("FORCE CTRL", "#fab387", "#1e1e2e"),
    4: ("HOLDING", "#a6e3a1", "#1e1e2e"),
    5: ("RELEASING", "#f38ba8", "#1e1e2e"),
}

# Default hand presets (positions in degrees for readability, converted to rad at runtime)
_DEFAULT_PRESETS = {
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


def _resolve_preset_path() -> str:
    """Resolve hand_presets.json path using the workspace logging directory."""
    from rtc_tools.utils.session_dir import get_session_dir, resolve_logging_root

    session_dir = get_session_dir()
    if session_dir:
        logging_root = os.path.dirname(session_dir)
    else:
        logging_root = resolve_logging_root()
    os.makedirs(logging_root, exist_ok=True)
    return os.path.join(logging_root, "hand_presets.json")


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
#   DemoTask wire order:  [kp_translation x3, kp_rotation x3, damping, null_kp,
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
        ("damping", 1, [0.01], False, "CLIK Gains"),
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
}

GAIN_ROW_NAMES = {
    "demo_joint_controller": {},
    "demo_task_controller": {
        "kp_translation": ["x", "y", "z"],
        "kp_rotation": ["rx", "ry", "rz"],
    },
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
        "damping": ("damping", _set_double),
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
}

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
