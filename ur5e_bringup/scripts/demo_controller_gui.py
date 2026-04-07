#!/usr/bin/env python3
"""
Demo Controller GUI (ur5e_bringup)
- Select between Demo Joint Controller (index 4) and Demo Task Controller (index 5)
- Set gains per controller via ~/controller_gains
- Displays currently applied gains after "Apply Gains" is pressed
- When switching controller, current joint positions become the new target
- Periodically display current joint positions alongside the target inputs
- E-STOP status indicator via /system/estop_status
- Hand motor target UI for both Demo Joint and Demo Task controllers
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool
from rtc_msgs.msg import GuiPosition, GraspState, RobotTarget
import tkinter as tk
from tkinter import ttk, font as tkfont, messagebox
import threading

CONTROLLER_TYPES = {
    "demo_joint_controller": "Demo Joint Controller",
    "demo_task_controller":  "Demo Task Controller",
}

TARGET_LABELS = {
    "demo_joint_controller": ["q1 (deg)", "q2 (deg)", "q3 (deg)", "q4 (deg)", "q5 (deg)", "q6 (deg)"],
    "demo_task_controller":  ["X (m)", "Y (m)", "Z (m)", "Roll (deg) / q4_null", "Pitch (deg) / q5_null", "Yaw (deg) / q6_null"],
}

ANGLE_INDICES = {
    "demo_joint_controller": [0, 1, 2, 3, 4, 5],
    "demo_task_controller":  [3, 4, 5],
}

# True -> joint space, False -> task space
JOINT_SPACE = {"demo_joint_controller": True, "demo_task_controller": False}

NUM_JOINTS = 6
NUM_HAND_MOTORS = 10

ROBOT_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# Hand finger groups
HAND_FINGER_GROUPS = [
    ("Thumb",  ["thumb_cmc_aa", "thumb_cmc_fe", "thumb_mcp_fe"]),
    ("Index",  ["index_mcp_aa", "index_mcp_fe", "index_dip_fe"]),
    ("Middle", ["middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe"]),
    ("Ring",   ["ring_mcp_fe"]),
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
    0: ("IDLE",        "#585b70", "#cdd6f4"),
    1: ("APPROACHING", "#89b4fa", "#1e1e2e"),
    2: ("CONTACT",     "#f9e2af", "#1e1e2e"),
    3: ("FORCE CTRL",  "#fab387", "#1e1e2e"),
    4: ("HOLDING",     "#a6e3a1", "#1e1e2e"),
    5: ("RELEASING",   "#f38ba8", "#1e1e2e"),
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

# hand_trajectory_speed index inside the flat gains array per controller
_HAND_TRAJ_SPEED_IDX = {
    "demo_joint_controller": 1,   # gains[1]
    "demo_task_controller": 12,   # gains[12] (after 3+3+1+1+1+1+1+1)
}


def _resolve_preset_path() -> str:
    """Resolve hand_presets.json path using the workspace logging directory."""
    session_dir = os.environ.get('RTC_SESSION_DIR') or os.environ.get('UR5E_SESSION_DIR')
    if session_dir:
        logging_root = os.path.dirname(session_dir)
    else:
        logging_root = os.path.expanduser('~/ros2_ws/ur5e_ws/logging_data')
    os.makedirs(logging_root, exist_ok=True)
    return os.path.join(logging_root, 'hand_presets.json')


# Gain definitions per controller
# Each entry: (label, size, defaults, is_bool)
#   DemoJoint: [robot_traj_speed, hand_traj_speed, robot_max_traj_vel, hand_max_traj_vel]
#   DemoTask:  [kp_translation x3, kp_rotation x3, damping, null_kp,
#               enable_null_space(0/1), control_6dof(0/1),
#               traj_speed, traj_angular_speed, hand_traj_speed,
#               max_traj_vel, max_traj_angular_vel, hand_max_traj_vel]
GAIN_DEFS = {
    "demo_joint_controller": [
        ("robot_traj_speed",    1, [1.0],  False),
        ("hand_traj_speed",     1, [1.0],  False),
        ("robot_max_traj_vel",  1, [3.14], False),
        ("hand_max_traj_vel",   1, [2.0],  False),
        ("grasp_contact_thresh", 1, [0.5], False),
        ("grasp_force_thresh",  1, [1.0],  False),
        ("grasp_min_fingertips", 1, [2],   False),
    ],
    "demo_task_controller": [
        ("kp_translation",      3, [1.0] * 3, False),
        ("kp_rotation",         3, [1.0] * 3, False),
        ("damping",             1, [0.01],     False),
        ("null_kp",             1, [0.5],      False),
        ("null space",          1, [1],        True),
        ("control 6dof",        1, [0],        True),
        ("traj_speed",          1, [0.1],      False),
        ("traj_angular_speed",  1, [0.5],      False),
        ("hand_traj_speed",     1, [1.0],      False),
        ("max_traj_vel",            1, [0.5],  False),
        ("max_traj_angular_vel",    1, [1.0],  False),
        ("hand_max_traj_vel",       1, [2.0],  False),
        ("grasp_contact_thresh", 1, [0.5],     False),
        ("grasp_force_thresh",  1, [1.0],      False),
        ("grasp_min_fingertips", 1, [2],       False),
    ],
}

GAIN_COL_HEADERS = {
    "demo_joint_controller": [],
    "demo_task_controller":  ["x / rx", "y / ry", "z / rz"],
}

GAIN_ROW_NAMES = {
    "demo_joint_controller": {},
    "demo_task_controller": {
        "kp_translation": ["x", "y", "z"],
        "kp_rotation": ["rx", "ry", "rz"],
    },
}


class DemoControllerGUI(Node):
    def __init__(self):
        super().__init__('demo_controller_gui')

        # Publishers
        self.robot_cmd_pub = self.create_publisher(
            RobotTarget, '/ur5e/joint_goal', 10)
        self.hand_cmd_pub = self.create_publisher(
            RobotTarget, '/hand/joint_goal', 10)
        self.type_pub = self.create_publisher(
            String, '/ur5e/controller_type', 10)
        self.gains_pub = self.create_publisher(
            Float64MultiArray, '/ur5e/controller_gains', 10)
        self.request_gains_pub = self.create_publisher(
            Bool, '/ur5e/request_gains', 10)

        # Subscriptions
        self.current_positions = [0.0] * NUM_JOINTS
        self.current_task_positions = [0.0] * 6
        self.create_subscription(GuiPosition, '/ur5e/gui_position',
                                 self._gui_pos_cb, 10)

        self.estop_active = False
        self.create_subscription(Bool, '/system/estop_status',
                                 self._estop_cb, 10)

        self._pending_load_gains = False
        self.create_subscription(Float64MultiArray, '/ur5e/current_gains',
                                 self._current_gains_cb, 10)

        # Hand state subscription via gui_position topic
        self.current_hand_positions = [0.0] * NUM_HAND_MOTORS
        self.create_subscription(GuiPosition, '/hand/gui_position',
                                 self._hand_gui_pos_cb, 10)

        # Grasp state subscription
        self._grasp_detected = False
        self._grasp_num_active = 0
        self._grasp_max_force = 0.0
        self._grasp_force_threshold = 1.0
        self._grasp_min_fingertips = 2
        self._grasp_force_mag = [0.0] * len(FINGERTIP_NAMES)
        self._grasp_contact_flag = [0.0] * len(FINGERTIP_NAMES)
        self._grasp_inference_valid = [False] * len(FINGERTIP_NAMES)
        # Force-PI grasp controller state
        self._grasp_phase = 0
        self._grasp_target_force_val = 0.0
        self._fp_finger_s = [0.0] * len(FORCE_PI_FINGER_NAMES)
        self._fp_filtered_force = [0.0] * len(FORCE_PI_FINGER_NAMES)
        self._fp_force_error = [0.0] * len(FORCE_PI_FINGER_NAMES)
        self.create_subscription(GraspState, '/hand/grasp_state',
                                 self._grasp_state_cb, 10)

        # Hand presets (loaded from JSON)
        self._preset_path = _resolve_preset_path()
        self._presets = self._load_presets()

        # Cached current gains (updated via request/response)
        self._cached_gains: list[float] | None = None

        # 5 Hz refresh timer
        self._refresh_timer = self.create_timer(0.2, self._refresh_current_display)

        # GUI in a daemon thread
        self._gui_ready = threading.Event()
        self._gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self._gui_thread.start()
        self._gui_ready.wait()

    # ---- ROS callbacks -------------------------------------------------------

    def _gui_pos_cb(self, msg: GuiPosition):
        if len(msg.joint_positions) >= NUM_JOINTS:
            if msg.joint_names and len(msg.joint_names) >= NUM_JOINTS:
                reordered = [0.0] * NUM_JOINTS
                for mi, name in enumerate(msg.joint_names[:NUM_JOINTS]):
                    if name in _ROBOT_NAME_TO_IDX:
                        reordered[_ROBOT_NAME_TO_IDX[name]] = msg.joint_positions[mi]
                self.current_positions = reordered
            else:
                self.current_positions = list(msg.joint_positions[:NUM_JOINTS])
        if len(msg.task_positions) >= 6:
            self.current_task_positions = list(msg.task_positions[:6])

    def _hand_gui_pos_cb(self, msg: GuiPosition):
        if len(msg.joint_positions) >= NUM_HAND_MOTORS:
            if msg.joint_names and len(msg.joint_names) >= NUM_HAND_MOTORS:
                reordered = [0.0] * NUM_HAND_MOTORS
                for mi, name in enumerate(msg.joint_names[:NUM_HAND_MOTORS]):
                    if name in _HAND_NAME_TO_IDX:
                        reordered[_HAND_NAME_TO_IDX[name]] = msg.joint_positions[mi]
                self.current_hand_positions = reordered
            else:
                self.current_hand_positions = list(msg.joint_positions[:NUM_HAND_MOTORS])

    def _estop_cb(self, msg: Bool):
        self.estop_active = msg.data

    def _grasp_state_cb(self, msg: GraspState):
        self._grasp_detected = msg.grasp_detected
        self._grasp_num_active = msg.num_active_contacts
        self._grasp_max_force = msg.max_force
        self._grasp_force_threshold = msg.force_threshold
        self._grasp_min_fingertips = msg.min_fingertips
        n = min(len(msg.force_magnitude), len(FINGERTIP_NAMES))
        for i in range(n):
            self._grasp_force_mag[i] = msg.force_magnitude[i]
            self._grasp_contact_flag[i] = msg.contact_flag[i]
            self._grasp_inference_valid[i] = msg.inference_valid[i]
        # Force-PI fields
        self._grasp_phase = msg.grasp_phase
        self._grasp_target_force_val = msg.grasp_target_force
        n_fp = min(len(msg.finger_s), len(FORCE_PI_FINGER_NAMES))
        for i in range(n_fp):
            self._fp_finger_s[i] = msg.finger_s[i]
            self._fp_filtered_force[i] = msg.finger_filtered_force[i]
            self._fp_force_error[i] = msg.finger_force_error[i]

    # ---- Preset persistence -----------------------------------------------------

    def _load_presets(self) -> dict:
        try:
            with open(self._preset_path, 'r') as f:
                data = json.load(f)
            if isinstance(data, dict) and data:
                return data
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        # Write defaults
        self._save_presets_to_file(_DEFAULT_PRESETS)
        return dict(_DEFAULT_PRESETS)

    def _save_presets_to_file(self, presets: dict | None = None):
        if presets is None:
            presets = self._presets
        try:
            os.makedirs(os.path.dirname(self._preset_path), exist_ok=True)
            with open(self._preset_path, 'w') as f:
                json.dump(presets, f, indent=2)
        except OSError as e:
            self.get_logger().error(f"Failed to save presets: {e}")

    def _current_gains_cb(self, msg: Float64MultiArray):
        self._cached_gains = list(msg.data)
        if not self._pending_load_gains:
            return
        self._pending_load_gains = False
        self.root.after(0, self._fill_gains_from_data, list(msg.data))

    def _refresh_current_display(self):
        if hasattr(self, '_status_labels_values'):
            idx = self.selected_ctrl.get()
            if JOINT_SPACE.get(idx, True):
                for i in range(NUM_JOINTS):
                    val_rad = self.current_positions[i]
                    val_deg = math.degrees(val_rad)
                    self._status_labels_values[i].config(text=f"{val_rad:.4f} rad  ({val_deg:.2f}°)")
            else:
                for i in range(6):
                    val = self.current_task_positions[i]
                    if i < 3:
                        self._status_labels_values[i].config(text=f"{val:.4f} m")
                    else:
                        val_deg = math.degrees(val)
                        self._status_labels_values[i].config(text=f"{val:.4f} rad  ({val_deg:.2f}°)")

        if hasattr(self, '_task_state_labels_values'):
            for i in range(6):
                val = self.current_task_positions[i]
                if i < 3:
                    self._task_state_labels_values[i].config(text=f"{val:.4f} m")
                else:
                    val_deg = math.degrees(val)
                    self._task_state_labels_values[i].config(
                        text=f"{val:.4f} rad  ({val_deg:.2f}°)")

        if hasattr(self, '_hand_state_labels_values'):
            for i in range(NUM_HAND_MOTORS):
                val_rad = self.current_hand_positions[i]
                val_deg = math.degrees(val_rad)
                self._hand_state_labels_values[i].config(
                    text=f"{val_deg:.2f}°")

        if hasattr(self, '_estop_label'):
            if self.estop_active:
                self._estop_label.config(
                    text="  E-STOP ACTIVE  ", fg='#1e1e2e', bg='#f38ba8')
            else:
                self._estop_label.config(
                    text="  NORMAL  ", fg='#1e1e2e', bg='#a6e3a1')

        # Grasp state display
        if hasattr(self, '_grasp_detected_label'):
            if self._grasp_detected:
                self._grasp_detected_label.config(
                    text="  GRASP DETECTED  ", bg='#a6e3a1', fg='#1e1e2e')
            else:
                self._grasp_detected_label.config(
                    text="  NO GRASP  ", bg='#585b70', fg='#cdd6f4')
            self._grasp_active_label.config(
                text=f"{self._grasp_num_active}")
            self._grasp_maxforce_label.config(
                text=f"{self._grasp_max_force:.2f} N")
            self._grasp_threshold_label.config(
                text=f"{self._grasp_force_threshold:.2f} N")
            self._grasp_minfinger_label.config(
                text=f"{self._grasp_min_fingertips}")

        if hasattr(self, '_ft_force_labels'):
            for i in range(len(FINGERTIP_NAMES)):
                self._ft_force_labels[i].config(
                    text=f"{self._grasp_force_mag[i]:.2f} N")
                cf = self._grasp_contact_flag[i]
                contact_color = '#a6e3a1' if cf > 0.5 else '#585b70'
                self._ft_contact_labels[i].config(
                    text=f"{cf:.2f}", bg=contact_color,
                    fg='#1e1e2e' if cf > 0.5 else '#cdd6f4')
                iv = self._grasp_inference_valid[i]
                self._ft_valid_labels[i].config(
                    text="OK" if iv else "--",
                    fg='#a6e3a1' if iv else '#f38ba8')

        # Force-PI state display
        if hasattr(self, '_fp_phase_label'):
            phase_info = GRASP_PHASE_NAMES.get(
                self._grasp_phase, ("UNKNOWN", "#585b70", "#cdd6f4"))
            self._fp_phase_label.config(
                text=f"  {phase_info[0]}  ",
                bg=phase_info[1], fg=phase_info[2])
            self._fp_target_force_display.config(
                text=f"{self._grasp_target_force_val:.2f} N")

            for i in range(len(FORCE_PI_FINGER_NAMES)):
                self._fp_s_labels[i].config(
                    text=f"{self._fp_finger_s[i]:.3f}")
                self._fp_filt_labels[i].config(
                    text=f"{self._fp_filtered_force[i]:.2f} N")
                err = self._fp_force_error[i]
                err_color = ('#a6e3a1' if abs(err) < 0.2
                             else '#f9e2af' if abs(err) < 0.5
                             else '#f38ba8')
                self._fp_err_labels[i].config(
                    text=f"{err:+.2f} N", fg=err_color)
                # f_desired = f_measured + f_error
                f_desired = self._fp_filtered_force[i] + err
                self._fp_desired_labels[i].config(
                    text=f"{f_desired:.2f} N")

    # ---- GUI build -----------------------------------------------------------

    def _run_gui(self):
        self.root = tk.Tk()
        self.root.title("Demo Controller GUI")
        self.root.geometry("1000x900")
        self.root.resizable(True, True)
        self.root.configure(bg="#1e1e2e")

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('.', background='#1e1e2e', foreground='#cdd6f4',
                        font=('Segoe UI', 9))
        style.configure('TLabelframe', background='#1e1e2e',
                        foreground='#89b4fa', font=('Segoe UI', 9, 'bold'))
        style.configure('TLabelframe.Label', background='#1e1e2e',
                        foreground='#89b4fa')
        style.configure('TRadiobutton', background='#1e1e2e',
                        foreground='#cdd6f4')
        style.configure('TCheckbutton', background='#1e1e2e',
                        foreground='#cdd6f4')
        style.configure('TButton', background='#313244', foreground='#cdd6f4',
                        padding=3)
        style.map('TButton', background=[('active', '#45475a')])
        style.configure('Send.TButton', background='#a6e3a1',
                        foreground='#1e1e2e', font=('Segoe UI', 9, 'bold'),
                        padding=5)
        style.map('Send.TButton', background=[('active', '#94e2d5')])
        style.configure('Gains.TButton', background='#fab387',
                        foreground='#1e1e2e', font=('Segoe UI', 9, 'bold'))
        style.map('Gains.TButton', background=[('active', '#f9e2af')])
        style.configure('Switch.TButton', background='#89b4fa',
                        foreground='#1e1e2e', font=('Segoe UI', 9, 'bold'))
        style.map('Switch.TButton', background=[('active', '#b4befe')])
        style.configure('TEntry', fieldbackground='#313244',
                        foreground='#cdd6f4', insertcolor='#cdd6f4')
        style.configure('TSeparator', background='#45475a')
        style.configure('TNotebook', background='#1e1e2e', borderwidth=0)
        style.configure('TNotebook.Tab', background='#313244',
                        foreground='#cdd6f4', padding=[10, 4],
                        font=('Segoe UI', 9, 'bold'))
        style.map('TNotebook.Tab',
                  background=[('selected', '#89b4fa')],
                  foreground=[('selected', '#1e1e2e')])
        style.configure('Preset.TButton', background='#cba6f7',
                        foreground='#1e1e2e', font=('Segoe UI', 9, 'bold'))
        style.map('Preset.TButton', background=[('active', '#f5c2e7')])
        style.configure('Treeview', background='#313244', foreground='#cdd6f4',
                        fieldbackground='#313244', font=('Segoe UI', 9))
        style.configure('Treeview.Heading', background='#45475a',
                        foreground='#cdd6f4', font=('Segoe UI', 9, 'bold'))
        style.map('Treeview',
                  background=[('selected', '#585b70')],
                  foreground=[('selected', '#cdd6f4')])
        style.configure('TCombobox', fieldbackground='#313244',
                        background='#313244', foreground='#cdd6f4')
        style.map('TCombobox',
                  fieldbackground=[('readonly', '#313244')],
                  foreground=[('readonly', '#cdd6f4')])

        # Scrollable wrapper
        outer_frame = tk.Frame(self.root, bg='#1e1e2e')
        outer_frame.pack(fill='both', expand=True)

        v_scrollbar = ttk.Scrollbar(outer_frame, orient='vertical')
        v_scrollbar.pack(side='right', fill='y')
        h_scrollbar = ttk.Scrollbar(outer_frame, orient='horizontal')
        h_scrollbar.pack(side='bottom', fill='x')

        canvas = tk.Canvas(outer_frame, bg='#1e1e2e',
                           yscrollcommand=v_scrollbar.set,
                           xscrollcommand=h_scrollbar.set,
                           highlightthickness=0)
        canvas.pack(side='left', fill='both', expand=True)

        v_scrollbar.config(command=canvas.yview)
        h_scrollbar.config(command=canvas.xview)

        scrollable_frame = tk.Frame(canvas, bg='#1e1e2e')
        canvas_window = canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')

        def _on_frame_configure(_event):
            canvas.configure(scrollregion=canvas.bbox('all'))

        def _on_canvas_configure(event):
            canvas.itemconfig(canvas_window, width=event.width)

        scrollable_frame.bind('<Configure>', _on_frame_configure)
        canvas.bind('<Configure>', _on_canvas_configure)

        def _on_mousewheel(event):
            if event.num == 4:
                canvas.yview_scroll(-1, 'units')
            elif event.num == 5:
                canvas.yview_scroll(1, 'units')
            else:
                canvas.yview_scroll(int(-1 * (event.delta / 120)), 'units')

        canvas.bind_all('<MouseWheel>', _on_mousewheel)
        canvas.bind_all('<Button-4>', _on_mousewheel)
        canvas.bind_all('<Button-5>', _on_mousewheel)

        # Header
        header_frame = tk.Frame(scrollable_frame, bg='#1e1e2e')
        header_frame.pack(fill='x', padx=8, pady=(6, 2))
        tk.Label(header_frame, text="Demo Controller GUI",
                 bg='#1e1e2e', fg='#89b4fa',
                 font=('Segoe UI', 12, 'bold')).pack(side='left')
        self._estop_label = tk.Label(header_frame, text="  NORMAL  ",
                                     bg='#a6e3a1', fg='#1e1e2e',
                                     font=('Segoe UI', 9, 'bold'),
                                     padx=4, pady=2)
        self._estop_label.pack(side='right')
        tk.Label(header_frame, text="E-STOP:",
                 bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 9)).pack(side='right', padx=(0, 4))

        # ── Notebook (Tabs) ──────────────────────────────────────────────
        notebook = ttk.Notebook(scrollable_frame)
        notebook.pack(fill='both', expand=True, padx=8, pady=2)

        control_tab = tk.Frame(notebook, bg='#1e1e2e')
        notebook.add(control_tab, text='  Control  ')

        grasp_tab = tk.Frame(notebook, bg='#1e1e2e')
        notebook.add(grasp_tab, text='  Grasp  ')

        # ══════════════════════════════════════════════════════════════════
        #  CONTROL TAB (existing UI)
        # ══════════════════════════════════════════════════════════════════

        # Controller Selection (only Demo Joint / Demo Task)
        ctrl_frame = ttk.LabelFrame(control_tab, text="Controller", padding=4)
        ctrl_frame.pack(fill='x', padx=8, pady=2)

        self.selected_ctrl = tk.StringVar(value="demo_joint_controller")
        btn_row = tk.Frame(ctrl_frame, bg='#1e1e2e')
        btn_row.pack(fill='x')
        for idx, name in CONTROLLER_TYPES.items():
            ttk.Radiobutton(btn_row, text=name, value=idx,
                            variable=self.selected_ctrl,
                            command=self._on_ctrl_radio_change).pack(
                side='left', padx=2)

        switch_btn = ttk.Button(ctrl_frame, text="Switch Controller",
                                style='Switch.TButton',
                                command=self._on_switch_controller)
        switch_btn.pack(pady=(3, 0))

        self._ctrl_status = tk.StringVar(value="Active: Demo Joint Controller")
        tk.Label(ctrl_frame, textvariable=self._ctrl_status,
                 bg='#1e1e2e', fg='#a6e3a1',
                 font=('Segoe UI', 9, 'italic')).pack()

        # Gains
        self._gains_frame = ttk.LabelFrame(control_tab, text="Gains", padding=4)
        self._gains_frame.pack(fill='x', padx=8, pady=2)

        self._gain_entries: list = []
        self._gain_is_bool: list[bool] = []
        self._applied_label_widgets: list = []

        self._gains_inner = tk.Frame(self._gains_frame, bg='#1e1e2e')
        self._gains_inner.pack(fill='x')

        gains_btn_frame = tk.Frame(self._gains_frame, bg='#1e1e2e')
        gains_btn_frame.pack(pady=(3, 2))
        ttk.Button(gains_btn_frame, text="Apply Gains",
                   style='Gains.TButton',
                   command=self._publish_gains).pack(side='left', padx=4)
        ttk.Button(gains_btn_frame, text="Load Gain",
                   style='Switch.TButton',
                   command=self._request_load_gains).pack(side='left', padx=4)

        ttk.Separator(self._gains_frame, orient='horizontal').pack(
            fill='x', pady=(1, 2))
        tk.Label(self._gains_frame, text="Currently Applied:",
                 bg='#1e1e2e', fg='#a6e3a1',
                 font=('Segoe UI', 8, 'bold')).pack(anchor='w', padx=2)
        self._gains_applied_inner = tk.Frame(self._gains_frame, bg='#1e1e2e')
        self._gains_applied_inner.pack(fill='x', padx=2, pady=(0, 2))

        # Target Inputs
        pos_frame = ttk.LabelFrame(control_tab, text="Target Inputs",
                                   padding=4)
        pos_frame.pack(fill='both', expand=True, padx=8, pady=2)

        hdr_font = tkfont.Font(family='Segoe UI', size=9, weight='bold')

        # Top row: Joint + Task targets side by side
        top_target_row = tk.Frame(pos_frame, bg='#1e1e2e')
        top_target_row.pack(fill='x')

        # Left: Joint Target
        left_target_frame = tk.Frame(top_target_row, bg='#1e1e2e')
        left_target_frame.pack(side='left', fill='y', padx=(0, 2))

        tk.Label(left_target_frame, text="Joint Target",
                 bg='#1e1e2e', fg='#89b4fa', font=hdr_font).grid(
            row=0, column=0, columnspan=3, pady=(0, 1))
        for col, txt in enumerate(["Axis", "Target", "Step (±)"]):
            w = 10 if col != 2 else 6
            tk.Label(left_target_frame, text=txt, bg='#1e1e2e', fg='#89b4fa',
                     font=hdr_font, width=w, anchor='center').grid(
                row=1, column=col, padx=2, pady=(0, 2))

        _joint_axis_labels = ["q1 (deg)", "q2 (deg)", "q3 (deg)",
                               "q4 (deg)", "q5 (deg)", "q6 (deg)"]
        self._joint_target_entries: list[ttk.Entry] = []
        self._joint_step_entries: list[ttk.Entry] = []
        self._joint_step_btns: list[list] = []

        for i in range(NUM_JOINTS):
            tk.Label(left_target_frame, text=_joint_axis_labels[i],
                     bg='#1e1e2e', fg='#cdd6f4', width=10, anchor='e').grid(
                row=i + 2, column=0, padx=2, pady=1)

            ent = ttk.Entry(left_target_frame, width=12, justify='center')
            ent.insert(0, "0.0000")
            ent.grid(row=i + 2, column=1, padx=2, pady=1)
            self._joint_target_entries.append(ent)

            bf = tk.Frame(left_target_frame, bg='#1e1e2e')
            bf.grid(row=i + 2, column=2, padx=1, pady=1)
            btn_m = ttk.Button(bf, text="-", width=2,
                               command=lambda idx=i: self._add_joint_step(idx, -1))
            btn_m.pack(side='left', padx=1)
            step_ent = ttk.Entry(bf, width=5, justify='center')
            step_ent.insert(0, "1.0")
            step_ent.pack(side='left', padx=1)
            btn_p = ttk.Button(bf, text="+", width=2,
                               command=lambda idx=i: self._add_joint_step(idx, 1))
            btn_p.pack(side='left', padx=1)
            self._joint_step_entries.append(step_ent)
            self._joint_step_btns.append([btn_m, btn_p])

        # Vertical separator
        ttk.Separator(top_target_row, orient='vertical').pack(side='left', fill='y', padx=6)

        # Right: Task Target
        right_target_frame = tk.Frame(top_target_row, bg='#1e1e2e')
        right_target_frame.pack(side='left', fill='y', padx=(2, 0))

        tk.Label(right_target_frame, text="Task Target",
                 bg='#1e1e2e', fg='#89b4fa', font=hdr_font).grid(
            row=0, column=0, columnspan=3, pady=(0, 1))
        for col, txt in enumerate(["Axis", "Target", "Step (±)"]):
            w = 10 if col != 2 else 6
            tk.Label(right_target_frame, text=txt, bg='#1e1e2e', fg='#89b4fa',
                     font=hdr_font, width=w, anchor='center').grid(
                row=1, column=col, padx=2, pady=(0, 2))

        _task_axis_labels = ["X (m)", "Y (m)", "Z (m)",
                             "Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
        self._task_target_entries: list[ttk.Entry] = []
        self._task_step_entries: list[ttk.Entry] = []
        self._task_step_btns: list[list] = []

        for i in range(NUM_JOINTS):
            tk.Label(right_target_frame, text=_task_axis_labels[i],
                     bg='#1e1e2e', fg='#cdd6f4', width=10, anchor='e').grid(
                row=i + 2, column=0, padx=2, pady=1)

            ent = ttk.Entry(right_target_frame, width=12, justify='center')
            ent.insert(0, "0.0000")
            ent.grid(row=i + 2, column=1, padx=2, pady=1)
            self._task_target_entries.append(ent)

            bf = tk.Frame(right_target_frame, bg='#1e1e2e')
            bf.grid(row=i + 2, column=2, padx=1, pady=1)
            btn_m = ttk.Button(bf, text="-", width=2,
                               command=lambda idx=i: self._add_task_step(idx, -1))
            btn_m.pack(side='left', padx=1)
            step_ent = ttk.Entry(bf, width=5, justify='center')
            step_ent.insert(0, "0.01" if i < 3 else "1.0")
            step_ent.pack(side='left', padx=1)
            btn_p = ttk.Button(bf, text="+", width=2,
                               command=lambda idx=i: self._add_task_step(idx, 1))
            btn_p.pack(side='left', padx=1)
            self._task_step_entries.append(step_ent)
            self._task_step_btns.append([btn_m, btn_p])

        # Hand Motor Target
        ttk.Separator(pos_frame, orient='horizontal').pack(fill='x', pady=4)

        hand_target_label = tk.Label(pos_frame, text="Hand Motor Target",
                                     bg='#1e1e2e', fg='#89b4fa', font=hdr_font)
        hand_target_label.pack(anchor='w', padx=4)

        hand_target_frame = tk.Frame(pos_frame, bg='#1e1e2e')
        hand_target_frame.pack(fill='x', padx=4)

        self._hand_target_entries: list[ttk.Entry] = []
        self._hand_step_entries: list[ttk.Entry] = []
        self._hand_step_btns: list[list] = []

        for col_idx, (finger_name, motors) in enumerate(HAND_FINGER_GROUPS):
            col_frame = tk.Frame(hand_target_frame, bg='#1e1e2e')
            col_frame.grid(row=0, column=col_idx, padx=4, sticky='n')

            tk.Label(col_frame, text=finger_name,
                     bg='#1e1e2e', fg='#f9e2af',
                     font=hdr_font).grid(row=0, column=0, columnspan=3, pady=(0, 2))

            for row_idx, motor_name in enumerate(motors):
                short_name = motor_name.split('_', 1)[1] if '_' in motor_name else motor_name
                tk.Label(col_frame, text=short_name,
                         bg='#1e1e2e', fg='#cdd6f4', width=8, anchor='e',
                         font=('Segoe UI', 8)).grid(
                    row=row_idx + 1, column=0, padx=1, pady=1)

                ent = ttk.Entry(col_frame, width=8, justify='center')
                ent.insert(0, "0.00")
                ent.grid(row=row_idx + 1, column=1, padx=1, pady=1)
                self._hand_target_entries.append(ent)

                bf = tk.Frame(col_frame, bg='#1e1e2e')
                bf.grid(row=row_idx + 1, column=2, padx=1, pady=1)
                hand_idx = len(self._hand_target_entries) - 1
                btn_m = ttk.Button(bf, text="-", width=2,
                                   command=lambda hi=hand_idx: self._add_hand_step(hi, -1))
                btn_m.pack(side='left')
                step_ent = ttk.Entry(bf, width=4, justify='center')
                step_ent.insert(0, "1.0")
                step_ent.pack(side='left')
                btn_p = ttk.Button(bf, text="+", width=2,
                                   command=lambda hi=hand_idx: self._add_hand_step(hi, 1))
                btn_p.pack(side='left')
                self._hand_step_entries.append(step_ent)
                self._hand_step_btns.append([btn_m, btn_p])

        # Current Status
        status_frame = ttk.LabelFrame(control_tab, text="Current Status", padding=4)
        status_frame.pack(fill='both', expand=False, padx=8, pady=2)

        # Left: controller-dependent state
        left_status = tk.Frame(status_frame, bg='#1e1e2e')
        left_status.pack(side='left', fill='y', padx=(0, 2))

        tk.Label(left_status, text="Controller State",
                 bg='#1e1e2e', fg='#89b4fa',
                 font=('Segoe UI', 9, 'bold')).grid(row=0, column=0, columnspan=2, pady=(0, 2))

        self._status_labels_names: list[tk.Label] = []
        self._status_labels_values: list[tk.Label] = []

        for i in range(6):
            name_lbl = tk.Label(left_status, text=f"J{i+1}:",
                                bg='#1e1e2e', fg='#cdd6f4', width=8, anchor='e',
                                font=('Segoe UI', 9, 'bold'))
            name_lbl.grid(row=i + 1, column=0, padx=(4, 2), pady=1)
            self._status_labels_names.append(name_lbl)

            val_lbl = tk.Label(left_status, text="0.0000 rad  (0.00°)",
                               bg='#313244', fg='#a6e3a1', width=22, anchor='center',
                               font=('Courier New', 9, 'bold'))
            val_lbl.grid(row=i + 1, column=1, padx=3, pady=1)
            self._status_labels_values.append(val_lbl)

        # Vertical separator
        ttk.Separator(status_frame, orient='vertical').pack(side='left', fill='y', padx=6)

        # Middle: task state
        right_status = tk.Frame(status_frame, bg='#1e1e2e')
        right_status.pack(side='left', fill='y', padx=(2, 0))

        tk.Label(right_status, text="Task State",
                 bg='#1e1e2e', fg='#89b4fa',
                 font=('Segoe UI', 9, 'bold')).grid(row=0, column=0, columnspan=2, pady=(0, 2))

        _task_state_row_labels = ["X (m)", "Y (m)", "Z (m)", "Roll", "Pitch", "Yaw"]
        self._task_state_labels_values: list[tk.Label] = []

        for i, label in enumerate(_task_state_row_labels):
            tk.Label(right_status, text=f"{label}:",
                     bg='#1e1e2e', fg='#cdd6f4', width=8, anchor='e',
                     font=('Segoe UI', 9, 'bold')).grid(row=i + 1, column=0, padx=(4, 2), pady=1)

            val_lbl = tk.Label(right_status, text="0.0000",
                               bg='#313244', fg='#cba6f7', width=22, anchor='center',
                               font=('Courier New', 9, 'bold'))
            val_lbl.grid(row=i + 1, column=1, padx=3, pady=1)
            self._task_state_labels_values.append(val_lbl)

        # Vertical separator
        ttk.Separator(status_frame, orient='vertical').pack(side='left', fill='y', padx=6)

        # Right: hand state
        hand_status_frame = tk.Frame(status_frame, bg='#1e1e2e')
        hand_status_frame.pack(side='left', fill='y', padx=(2, 0))

        tk.Label(hand_status_frame, text="Hand State",
                 bg='#1e1e2e', fg='#89b4fa',
                 font=('Segoe UI', 9, 'bold')).grid(
            row=0, column=0, columnspan=len(HAND_FINGER_GROUPS), pady=(0, 2))

        self._hand_state_labels_values: list[tk.Label] = []

        for col_idx, (finger_name, motors) in enumerate(HAND_FINGER_GROUPS):
            col_frame = tk.Frame(hand_status_frame, bg='#1e1e2e')
            col_frame.grid(row=1, column=col_idx, padx=2, sticky='n')

            tk.Label(col_frame, text=finger_name,
                     bg='#1e1e2e', fg='#f9e2af',
                     font=('Segoe UI', 8, 'bold')).pack(anchor='center')

            for motor_name in motors:
                short_name = motor_name.split('_', 1)[1] if '_' in motor_name else motor_name
                row_frame = tk.Frame(col_frame, bg='#1e1e2e')
                row_frame.pack(fill='x', pady=1)
                tk.Label(row_frame, text=f"{short_name}:",
                         bg='#1e1e2e', fg='#cdd6f4', width=7, anchor='e',
                         font=('Segoe UI', 8)).pack(side='left')
                val_lbl = tk.Label(row_frame, text="0.00°",
                                   bg='#313244', fg='#f9e2af', width=8, anchor='center',
                                   font=('Courier New', 8, 'bold'))
                val_lbl.pack(side='left', padx=2)
                self._hand_state_labels_values.append(val_lbl)

        # Action Buttons
        btn_frame = tk.Frame(control_tab, bg='#1e1e2e')
        btn_frame.pack(pady=4)
        ttk.Button(btn_frame, text="Copy Current → Target",
                   command=self._copy_current_to_target).pack(side='left', padx=4)
        ttk.Button(btn_frame, text="Send Command",
                   style='Send.TButton',
                   command=self._publish_target).pack(side='left', padx=4)

        # ══════════════════════════════════════════════════════════════════
        #  GRASP TAB
        # ══════════════════════════════════════════════════════════════════
        self._build_grasp_tab(grasp_tab)

        # Initial gains build + initial enable/disable state
        self._rebuild_gains_ui(4)
        self._update_target_inputs_state(4)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._gui_ready.set()
        self.root.mainloop()

    # ---- Grasp tab builder -----------------------------------------------------

    def _build_grasp_tab(self, parent: tk.Frame):
        hdr_font = tkfont.Font(family='Segoe UI', size=9, weight='bold')
        mono_font = ('Courier New', 9, 'bold')

        # ── Grasp State Monitor ─────────────────────────────────────────
        state_frame = ttk.LabelFrame(parent, text="Grasp State", padding=4)
        state_frame.pack(fill='x', padx=8, pady=(4, 2))

        # Top row: big indicator + aggregate values
        top_row = tk.Frame(state_frame, bg='#1e1e2e')
        top_row.pack(fill='x', pady=(0, 4))

        self._grasp_detected_label = tk.Label(
            top_row, text="  NO GRASP  ", bg='#585b70', fg='#cdd6f4',
            font=('Segoe UI', 11, 'bold'), padx=8, pady=4)
        self._grasp_detected_label.pack(side='left', padx=(4, 12))

        agg_frame = tk.Frame(top_row, bg='#1e1e2e')
        agg_frame.pack(side='left', fill='x')

        agg_items = [
            ("Active Contacts:", '_grasp_active_label', "0"),
            ("Max Force:", '_grasp_maxforce_label', "0.00 N"),
            ("Threshold:", '_grasp_threshold_label', "1.00 N"),
            ("Min Fingers:", '_grasp_minfinger_label', "2"),
        ]
        for i, (text, attr, default) in enumerate(agg_items):
            tk.Label(agg_frame, text=text, bg='#1e1e2e', fg='#cdd6f4',
                     font=('Segoe UI', 8), anchor='e').grid(
                row=i // 2, column=(i % 2) * 2, padx=(8, 2), pady=1, sticky='e')
            lbl = tk.Label(agg_frame, text=default, bg='#313244', fg='#f9e2af',
                           font=mono_font, width=10, anchor='center')
            lbl.grid(row=i // 2, column=(i % 2) * 2 + 1, padx=(0, 8), pady=1)
            setattr(self, attr, lbl)

        # Per-fingertip table
        ttk.Separator(state_frame, orient='horizontal').pack(fill='x', pady=2)

        ft_frame = tk.Frame(state_frame, bg='#1e1e2e')
        ft_frame.pack(fill='x', padx=4)

        # Header row
        for c, txt in enumerate(["Finger", "Force (N)", "Contact", "Valid"]):
            tk.Label(ft_frame, text=txt, bg='#1e1e2e', fg='#89b4fa',
                     font=hdr_font, width=10, anchor='center').grid(
                row=0, column=c, padx=2, pady=(0, 2))

        self._ft_force_labels: list[tk.Label] = []
        self._ft_contact_labels: list[tk.Label] = []
        self._ft_valid_labels: list[tk.Label] = []

        for i, name in enumerate(FINGERTIP_NAMES):
            tk.Label(ft_frame, text=name, bg='#1e1e2e', fg='#f9e2af',
                     font=hdr_font, width=10, anchor='center').grid(
                row=i + 1, column=0, padx=2, pady=1)

            fl = tk.Label(ft_frame, text="0.00 N", bg='#313244', fg='#cdd6f4',
                          font=mono_font, width=10, anchor='center')
            fl.grid(row=i + 1, column=1, padx=2, pady=1)
            self._ft_force_labels.append(fl)

            cl = tk.Label(ft_frame, text="0.00", bg='#585b70', fg='#cdd6f4',
                          font=mono_font, width=10, anchor='center')
            cl.grid(row=i + 1, column=2, padx=2, pady=1)
            self._ft_contact_labels.append(cl)

            vl = tk.Label(ft_frame, text="--", bg='#1e1e2e', fg='#f38ba8',
                          font=mono_font, width=10, anchor='center')
            vl.grid(row=i + 1, column=3, padx=2, pady=1)
            self._ft_valid_labels.append(vl)

        # ── Force-PI Grasp Controller ──────────────────────────────────
        fp_frame = ttk.LabelFrame(parent, text="Force-PI Grasp Controller", padding=4)
        fp_frame.pack(fill='x', padx=8, pady=(2, 2))

        # Phase + target force row
        fp_top = tk.Frame(fp_frame, bg='#1e1e2e')
        fp_top.pack(fill='x', pady=(0, 4))

        tk.Label(fp_top, text="Phase:", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(4, 2))
        self._fp_phase_label = tk.Label(
            fp_top, text="  IDLE  ", bg='#585b70', fg='#cdd6f4',
            font=('Segoe UI', 10, 'bold'), padx=6, pady=2)
        self._fp_phase_label.pack(side='left', padx=(0, 12))

        tk.Label(fp_top, text="Target:", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(0, 2))
        self._fp_target_force_display = tk.Label(
            fp_top, text="0.00 N", bg='#313244', fg='#f9e2af',
            font=mono_font, width=8, anchor='center')
        self._fp_target_force_display.pack(side='left', padx=(0, 4))

        # Per-finger table (3 fingers)
        ttk.Separator(fp_frame, orient='horizontal').pack(fill='x', pady=2)
        fp_tbl = tk.Frame(fp_frame, bg='#1e1e2e')
        fp_tbl.pack(fill='x', padx=4)

        for c, txt in enumerate(["Finger", "s", "Filtered F", "F Error", "F Desired"]):
            tk.Label(fp_tbl, text=txt, bg='#1e1e2e', fg='#89b4fa',
                     font=hdr_font, width=10, anchor='center').grid(
                row=0, column=c, padx=2, pady=(0, 2))

        self._fp_s_labels: list[tk.Label] = []
        self._fp_filt_labels: list[tk.Label] = []
        self._fp_err_labels: list[tk.Label] = []
        self._fp_desired_labels: list[tk.Label] = []

        for i, name in enumerate(FORCE_PI_FINGER_NAMES):
            tk.Label(fp_tbl, text=name, bg='#1e1e2e', fg='#f9e2af',
                     font=hdr_font, width=10, anchor='center').grid(
                row=i + 1, column=0, padx=2, pady=1)

            sl = tk.Label(fp_tbl, text="0.000", bg='#313244', fg='#cdd6f4',
                          font=mono_font, width=10, anchor='center')
            sl.grid(row=i + 1, column=1, padx=2, pady=1)
            self._fp_s_labels.append(sl)

            fl = tk.Label(fp_tbl, text="0.00 N", bg='#313244', fg='#cdd6f4',
                          font=mono_font, width=10, anchor='center')
            fl.grid(row=i + 1, column=2, padx=2, pady=1)
            self._fp_filt_labels.append(fl)

            el = tk.Label(fp_tbl, text="+0.00 N", bg='#313244', fg='#a6e3a1',
                          font=mono_font, width=10, anchor='center')
            el.grid(row=i + 1, column=3, padx=2, pady=1)
            self._fp_err_labels.append(el)

            dl = tk.Label(fp_tbl, text="0.00 N", bg='#313244', fg='#cdd6f4',
                          font=mono_font, width=10, anchor='center')
            dl.grid(row=i + 1, column=4, padx=2, pady=1)
            self._fp_desired_labels.append(dl)

        # Command row: Grasp/Release buttons + target force input
        ttk.Separator(fp_frame, orient='horizontal').pack(fill='x', pady=2)
        cmd_frame = tk.Frame(fp_frame, bg='#1e1e2e')
        cmd_frame.pack(fill='x', padx=4, pady=(2, 0))

        ttk.Button(cmd_frame, text="\u25b6 Grasp", style='Send.TButton',
                   command=lambda: self._send_grasp_command(1)).pack(
            side='left', padx=4)
        ttk.Button(cmd_frame, text="\u25a0 Release",
                   command=lambda: self._send_grasp_command(2)).pack(
            side='left', padx=4)

        tk.Label(cmd_frame, text="Target Force:", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(16, 2))
        self._grasp_target_force_entry = ttk.Entry(
            cmd_frame, width=6, justify='center')
        self._grasp_target_force_entry.insert(0, "2.0")
        self._grasp_target_force_entry.pack(side='left', padx=2)
        tk.Label(cmd_frame, text="N", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left')

        # ── Hand Posture Presets ─────────────────────────────────────────
        preset_frame = ttk.LabelFrame(parent, text="Hand Posture Presets", padding=4)
        preset_frame.pack(fill='both', expand=True, padx=8, pady=(2, 4))

        # Preset Treeview
        tree_frame = tk.Frame(preset_frame, bg='#1e1e2e')
        tree_frame.pack(fill='both', expand=True, padx=4, pady=(0, 4))

        columns = ('type', 'controller', 'grasp_time', 'robot_target', 'positions')
        self._preset_tree = ttk.Treeview(
            tree_frame, columns=columns, show='tree headings',
            height=6, selectmode='browse')
        self._preset_tree.heading('#0', text='Name', anchor='w')
        self._preset_tree.heading('type', text='Type', anchor='center')
        self._preset_tree.heading('controller', text='Controller', anchor='center')
        self._preset_tree.heading('grasp_time', text='Time (s)', anchor='center')
        self._preset_tree.heading('robot_target', text='Robot Target', anchor='w')
        self._preset_tree.heading('positions', text='Hand (deg)', anchor='w')
        self._preset_tree.column('#0', width=100, minwidth=70)
        self._preset_tree.column('type', width=50, minwidth=40, anchor='center')
        self._preset_tree.column('controller', width=80, minwidth=60, anchor='center')
        self._preset_tree.column('grasp_time', width=55, minwidth=40, anchor='center')
        self._preset_tree.column('robot_target', width=220, minwidth=120)
        self._preset_tree.column('positions', width=280, minwidth=150)

        tree_scroll = ttk.Scrollbar(tree_frame, orient='vertical',
                                     command=self._preset_tree.yview)
        self._preset_tree.configure(yscrollcommand=tree_scroll.set)
        self._preset_tree.pack(side='left', fill='both', expand=True)
        tree_scroll.pack(side='right', fill='y')

        self._refresh_preset_tree()

        # Input row for new preset
        input_frame = tk.Frame(preset_frame, bg='#1e1e2e')
        input_frame.pack(fill='x', padx=4, pady=2)

        tk.Label(input_frame, text="Name:", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(0, 2))
        self._preset_name_entry = ttk.Entry(input_frame, width=14, justify='center')
        self._preset_name_entry.pack(side='left', padx=2)

        tk.Label(input_frame, text="Type:", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(8, 2))
        self._preset_type_var = tk.StringVar(value="open")
        self._preset_type_combo = ttk.Combobox(
            input_frame, textvariable=self._preset_type_var,
            values=["open", "close"], width=6, state='readonly')
        self._preset_type_combo.pack(side='left', padx=2)

        tk.Label(input_frame, text="Grasp Time (s):", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(8, 2))
        self._preset_time_entry = ttk.Entry(input_frame, width=6, justify='center')
        self._preset_time_entry.insert(0, "1.0")
        self._preset_time_entry.pack(side='left', padx=2)

        # Robot inclusion row
        robot_input_frame = tk.Frame(preset_frame, bg='#1e1e2e')
        robot_input_frame.pack(fill='x', padx=4, pady=2)

        self._preset_include_robot_var = tk.BooleanVar(value=False)
        self._preset_include_robot_cb = ttk.Checkbutton(
            robot_input_frame, text="Include Robot",
            variable=self._preset_include_robot_var,
            command=self._on_preset_include_robot_toggle)
        self._preset_include_robot_cb.pack(side='left', padx=(0, 8))

        tk.Label(robot_input_frame, text="Controller:", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(0, 2))
        self._preset_ctrl_var = tk.StringVar(value="demo_joint_controller")
        self._preset_ctrl_combo = ttk.Combobox(
            robot_input_frame, textvariable=self._preset_ctrl_var,
            values=list(CONTROLLER_TYPES.keys()), width=22, state='disabled')
        self._preset_ctrl_combo.pack(side='left', padx=2)
        self._preset_ctrl_combo.bind('<<ComboboxSelected>>',
                                     self._on_preset_ctrl_combo_change)

        tk.Label(robot_input_frame, text="Goal:", bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 8)).pack(side='left', padx=(8, 2))
        self._preset_goal_type_var = tk.StringVar(value="joint")
        self._preset_goal_type_combo = ttk.Combobox(
            robot_input_frame, textvariable=self._preset_goal_type_var,
            values=["joint", "task"], width=6, state='disabled')
        self._preset_goal_type_combo.pack(side='left', padx=2)

        # Action buttons
        action_frame = tk.Frame(preset_frame, bg='#1e1e2e')
        action_frame.pack(fill='x', padx=4, pady=(2, 0))

        ttk.Button(action_frame, text="Send Preset",
                   style='Send.TButton',
                   command=self._send_preset).pack(side='left', padx=4)
        ttk.Button(action_frame, text="Save Current Pos",
                   style='Preset.TButton',
                   command=self._save_current_as_preset).pack(side='left', padx=4)
        ttk.Button(action_frame, text="Save Target Pos",
                   style='Preset.TButton',
                   command=self._save_target_as_preset).pack(side='left', padx=4)
        ttk.Button(action_frame, text="Delete",
                   command=self._delete_preset).pack(side='left', padx=4)

        # Preset file path display
        tk.Label(preset_frame, text=f"Preset file: {self._preset_path}",
                 bg='#1e1e2e', fg='#585b70',
                 font=('Segoe UI', 7, 'italic')).pack(anchor='w', padx=4, pady=(4, 0))

    def _on_preset_include_robot_toggle(self):
        state = 'readonly' if self._preset_include_robot_var.get() else 'disabled'
        self._preset_ctrl_combo.configure(state=state)
        self._preset_goal_type_combo.configure(state=state)
        if self._preset_include_robot_var.get():
            # Sync with currently selected controller
            ctrl = self.selected_ctrl.get()
            self._preset_ctrl_var.set(ctrl)
            self._preset_goal_type_var.set(
                'joint' if JOINT_SPACE.get(ctrl, True) else 'task')

    def _on_preset_ctrl_combo_change(self, _event=None):
        ctrl = self._preset_ctrl_var.get()
        self._preset_goal_type_var.set(
            'joint' if JOINT_SPACE.get(ctrl, True) else 'task')

    def _refresh_preset_tree(self):
        for item in self._preset_tree.get_children():
            self._preset_tree.delete(item)
        for name, data in self._presets.items():
            pos_str = ", ".join(f"{v:.1f}" for v in data.get('positions_deg', []))
            ctrl = data.get('controller')
            ctrl_short = ctrl.replace('demo_', '').replace('_controller', '') if ctrl else "\u2014"
            robot_tgt = data.get('robot_target')
            if robot_tgt and len(robot_tgt) == NUM_JOINTS:
                goal = data.get('robot_goal_type', 'joint')
                robot_str = f"[{goal}] " + ", ".join(f"{v:.2f}" for v in robot_tgt)
            else:
                robot_str = "\u2014"
            self._preset_tree.insert(
                '', 'end', iid=name, text=name,
                values=(
                    data.get('type', 'open'),
                    ctrl_short,
                    f"{data.get('grasp_time', 1.0):.1f}",
                    robot_str,
                    pos_str,
                ))

    # ---- Gains UI helpers ----------------------------------------------------

    def _rebuild_gains_ui(self, ctrl_idx: int):
        for w in self._gains_inner.winfo_children():
            w.destroy()
        for w in self._gains_applied_inner.winfo_children():
            w.destroy()
        self._gain_entries.clear()
        self._gain_is_bool.clear()
        self._applied_label_widgets.clear()

        defs = GAIN_DEFS.get(ctrl_idx, [])
        headers = GAIN_COL_HEADERS.get(ctrl_idx, [])
        if not defs:
            return

        max_size = max(size for _, size, _, _ in defs)

        # Editable inputs
        if max_size > 1:
            for j, h in enumerate(headers):
                tk.Label(self._gains_inner, text=h,
                         bg='#1e1e2e', fg='#89b4fa',
                         font=('Segoe UI', 8, 'bold'),
                         width=7, anchor='center').grid(
                    row=0, column=j + 1, padx=2)

        array_row = 1
        scalar_frame = None
        row_names_map = GAIN_ROW_NAMES.get(ctrl_idx, {})

        for label, size, defaults, is_bool in defs:
            if size > 1:
                names = row_names_map.get(label)
                if names:
                    for j, name in enumerate(names):
                        tk.Label(self._gains_inner, text=name,
                                 bg='#1e1e2e', fg='#f9e2af',
                                 font=('Segoe UI', 7),
                                 width=7, anchor='center').grid(
                            row=array_row, column=j + 1, padx=2)
                    array_row += 1

                tk.Label(self._gains_inner, text=label + ':',
                         bg='#1e1e2e', fg='#cdd6f4',
                         font=('Segoe UI', 8), anchor='e').grid(
                    row=array_row, column=0, sticky='e', padx=(8, 4), pady=1)
                row_widgets = []
                for j in range(size):
                    ent = ttk.Entry(self._gains_inner, width=7, justify='center')
                    ent.insert(0, str(defaults[j]))
                    ent.grid(row=array_row, column=j + 1, padx=2, pady=1)
                    row_widgets.append(ent)
                self._gain_entries.append(row_widgets)
                self._gain_is_bool.append(False)
                array_row += 1
            else:
                if scalar_frame is None:
                    scalar_frame = tk.Frame(self._gains_inner, bg='#1e1e2e')
                    scalar_frame.grid(row=array_row, column=0,
                                      columnspan=max_size + 1,
                                      sticky='w', pady=(6, 0))
                frm = tk.Frame(scalar_frame, bg='#1e1e2e')
                frm.pack(side='left', padx=8)
                tk.Label(frm, text=label,
                         bg='#1e1e2e', fg='#cdd6f4',
                         font=('Segoe UI', 8)).pack(anchor='w')
                if is_bool:
                    var = tk.IntVar(value=int(defaults[0]))
                    ttk.Checkbutton(frm, variable=var).pack(anchor='w')
                    self._gain_entries.append([var])
                else:
                    ent = ttk.Entry(frm, width=8, justify='center')
                    ent.insert(0, str(defaults[0]))
                    ent.pack()
                    self._gain_entries.append([ent])
                self._gain_is_bool.append(is_bool)

        # Applied display (read-only mirror)
        tk.Label(self._gains_applied_inner, text="(press Apply Gains to update)",
                 bg='#1e1e2e', fg='#585b70',
                 font=('Segoe UI', 7, 'italic')).grid(
            row=0, column=0, columnspan=max_size + 1, sticky='w')

        if max_size > 1:
            for j, h in enumerate(headers):
                tk.Label(self._gains_applied_inner, text=h,
                         bg='#1e1e2e', fg='#585b70',
                         font=('Segoe UI', 8, 'bold'),
                         width=7, anchor='center').grid(
                    row=1, column=j + 1, padx=2)

        array_row_a = 2
        scalar_frame_a = None

        for label, size, defaults, is_bool in defs:
            if size > 1:
                names = row_names_map.get(label)
                if names:
                    for j, name in enumerate(names):
                        tk.Label(self._gains_applied_inner, text=name,
                                 bg='#1e1e2e', fg='#585b70',
                                 font=('Segoe UI', 7),
                                 width=7, anchor='center').grid(
                            row=array_row_a, column=j + 1, padx=2)
                    array_row_a += 1

                tk.Label(self._gains_applied_inner, text=label + ':',
                         bg='#1e1e2e', fg='#585b70',
                         font=('Segoe UI', 8), anchor='e').grid(
                    row=array_row_a, column=0, sticky='e',
                    padx=(8, 4), pady=1)
                row_labels = []
                for j in range(size):
                    lbl = tk.Label(self._gains_applied_inner, text='---',
                                   bg='#1e1e2e', fg='#585b70',
                                   font=('Courier New', 8),
                                   width=7, anchor='center')
                    lbl.grid(row=array_row_a, column=j + 1, padx=2, pady=1)
                    row_labels.append(lbl)
                self._applied_label_widgets.append(row_labels)
                array_row_a += 1
            else:
                if scalar_frame_a is None:
                    scalar_frame_a = tk.Frame(self._gains_applied_inner,
                                              bg='#1e1e2e')
                    scalar_frame_a.grid(row=array_row_a, column=0,
                                        columnspan=max_size + 1,
                                        sticky='w', pady=(4, 0))
                frm = tk.Frame(scalar_frame_a, bg='#1e1e2e')
                frm.pack(side='left', padx=8)
                tk.Label(frm, text=label,
                         bg='#1e1e2e', fg='#585b70',
                         font=('Segoe UI', 8)).pack(anchor='w')
                lbl = tk.Label(frm, text='---',
                               bg='#1e1e2e', fg='#585b70',
                               font=('Courier New', 8))
                lbl.pack(anchor='w')
                self._applied_label_widgets.append([lbl])

    def _update_applied_display(self):
        for widgets, applied_labels, is_bool in zip(
                self._gain_entries, self._applied_label_widgets,
                self._gain_is_bool):
            for w, lbl in zip(widgets, applied_labels):
                raw = w.get()
                if is_bool:
                    val = int(raw)
                    lbl.config(text="ON" if val else "OFF",
                               fg='#a6e3a1' if val else '#f38ba8')
                else:
                    try:
                        lbl.config(text=f"{float(raw):.4f}", fg='#a6e3a1')
                    except ValueError:
                        pass

    # ---- Button handlers -----------------------------------------------------

    def _update_target_inputs_state(self, ctrl_idx: int):
        is_joint = JOINT_SPACE.get(ctrl_idx, True)
        joint_state = 'normal' if is_joint else 'disabled'
        task_state  = 'disabled' if is_joint else 'normal'

        for ent in self._joint_target_entries:
            ent.configure(state=joint_state)
        for ent in self._joint_step_entries:
            ent.configure(state=joint_state)
        for btns in self._joint_step_btns:
            for btn in btns:
                btn.configure(state=joint_state)
        for ent in self._task_target_entries:
            ent.configure(state=task_state)
        for ent in self._task_step_entries:
            ent.configure(state=task_state)
        for btns in self._task_step_btns:
            for btn in btns:
                btn.configure(state=task_state)

    def _on_ctrl_radio_change(self):
        idx = self.selected_ctrl.get()
        self._rebuild_gains_ui(idx)
        self._update_target_inputs_state(idx)

    def _on_switch_controller(self):
        idx = self.selected_ctrl.get()

        msg = String()
        msg.data = idx
        self.type_pub.publish(msg)
        self.get_logger().info(
            f"Switched to controller: {CONTROLLER_TYPES[idx]}")

        self._ctrl_status.set(f"Active: {CONTROLLER_TYPES[idx]}")
        self._rebuild_gains_ui(idx)
        self._update_target_inputs_state(idx)

        _task_status_names = ["X (m)", "Y (m)", "Z (m)", "Roll", "Pitch", "Yaw"]
        if JOINT_SPACE.get(idx, True):
            self._set_joint_target_entries(self.current_positions)
            for i, name_lbl in enumerate(self._status_labels_names):
                name_lbl.config(text=f"J{i+1}:")
        else:
            self._set_task_target_entries(self.current_task_positions)
            for i, name_lbl in enumerate(self._status_labels_names):
                name_lbl.config(text=f"{_task_status_names[i]}:")

        # Hand target: initialize from current positions
        self._set_hand_target_entries(self.current_hand_positions)

    def _request_load_gains(self):
        self._pending_load_gains = True
        msg = Bool()
        msg.data = True
        self.request_gains_pub.publish(msg)
        self.get_logger().info("Requested current gains from active controller")

    def _fill_gains_from_data(self, data: list[float]):
        idx = 0
        for widgets, is_bool in zip(self._gain_entries, self._gain_is_bool):
            for w in widgets:
                if idx >= len(data):
                    return
                if is_bool:
                    w.set(1 if data[idx] > 0.5 else 0)
                else:
                    w.delete(0, tk.END)
                    w.insert(0, f"{data[idx]:.4f}")
                idx += 1

        ctrl_name = CONTROLLER_TYPES[self.selected_ctrl.get()]
        self.get_logger().info(
            f"Loaded gains for {ctrl_name}: {[f'{v:.4f}' for v in data]}")

    def _collect_gain_values(self) -> list[float] | None:
        """Collect current gain values from UI widgets. Returns None on error."""
        values: list[float] = []
        for widgets, is_bool in zip(self._gain_entries, self._gain_is_bool):
            for w in widgets:
                try:
                    values.append(float(w.get()))
                except ValueError:
                    self.get_logger().error(
                        "Invalid gain value — check all fields.")
                    return None
        return values

    def _publish_gains(self):
        values = self._collect_gain_values()
        if values is None:
            return

        msg = Float64MultiArray()
        msg.data = values
        self.gains_pub.publish(msg)

        ctrl_name = CONTROLLER_TYPES[self.selected_ctrl.get()]
        self.get_logger().info(
            f"Applied gains for {ctrl_name}: {[f'{v:.4f}' for v in values]}")

        self._update_applied_display()

    def _send_grasp_command(self, cmd: int):
        """Send grasp command (1=grasp, 2=release) via gains message.

        Appends grasp_command + grasp_target_force to the base gains array
        so the controller receives the full expected layout.
        """
        base = self._collect_gain_values()
        if base is None:
            return
        try:
            target_force = float(self._grasp_target_force_entry.get())
        except (ValueError, AttributeError):
            target_force = 2.0

        base.extend([float(cmd), target_force])

        msg = Float64MultiArray()
        msg.data = base
        self.gains_pub.publish(msg)

        cmd_name = "Grasp" if cmd == 1 else "Release"
        self.get_logger().info(
            f"Sent {cmd_name} command (target_force={target_force:.2f} N)")

    def _copy_current_to_target(self):
        idx = self.selected_ctrl.get()
        if JOINT_SPACE.get(idx, True):
            self._set_joint_target_entries(self.current_positions)
        else:
            self._set_task_target_entries(self.current_task_positions)
        self._set_hand_target_entries(self.current_hand_positions)

    def _set_joint_target_entries(self, values: list[float]):
        for i, ent in enumerate(self._joint_target_entries):
            ent.delete(0, tk.END)
            ent.insert(0, f"{math.degrees(values[i]):.4f}")

    def _set_task_target_entries(self, values: list[float]):
        for i, ent in enumerate(self._task_target_entries):
            ent.delete(0, tk.END)
            if i < 3:
                ent.insert(0, f"{values[i]:.4f}")
            else:
                ent.insert(0, f"{math.degrees(values[i]):.4f}")

    def _set_hand_target_entries(self, values: list[float]):
        for i, ent in enumerate(self._hand_target_entries):
            if i < len(values):
                ent.delete(0, tk.END)
                ent.insert(0, f"{math.degrees(values[i]):.2f}")

    def _add_joint_step(self, idx: int, sign: int):
        try:
            step_val = float(self._joint_step_entries[idx].get())
            new_val = float(self._joint_target_entries[idx].get()) + sign * step_val
            self._joint_target_entries[idx].delete(0, tk.END)
            self._joint_target_entries[idx].insert(0, f"{new_val:.4f}")
        except ValueError:
            self.get_logger().error("Invalid numerical input for joint target or step.")

    def _add_task_step(self, idx: int, sign: int):
        try:
            step_val = float(self._task_step_entries[idx].get())
            new_val = float(self._task_target_entries[idx].get()) + sign * step_val
            self._task_target_entries[idx].delete(0, tk.END)
            self._task_target_entries[idx].insert(0, f"{new_val:.4f}")
        except ValueError:
            self.get_logger().error("Invalid numerical input for task target or step.")

    def _add_hand_step(self, idx: int, sign: int):
        try:
            step_val = float(self._hand_step_entries[idx].get())
            new_val = float(self._hand_target_entries[idx].get()) + sign * step_val
            self._hand_target_entries[idx].delete(0, tk.END)
            self._hand_target_entries[idx].insert(0, f"{new_val:.2f}")
        except ValueError:
            self.get_logger().error("Invalid numerical input for hand target or step.")

    def _publish_target(self):
        idx = self.selected_ctrl.get()
        is_joint = JOINT_SPACE.get(idx, True)

        robot_msg = RobotTarget()
        robot_msg.joint_names = ROBOT_JOINT_NAMES
        try:
            if is_joint:
                robot_msg.goal_type = 'joint'
                robot_msg.joint_target = [
                    math.radians(float(e.get()))
                    for e in self._joint_target_entries]
            else:
                robot_msg.goal_type = 'task'
                task_values = []
                for i, e in enumerate(self._task_target_entries):
                    v = float(e.get())
                    if i >= 3:
                        v = math.radians(v)
                    task_values.append(v)
                robot_msg.task_target = task_values
        except ValueError:
            self.get_logger().error("Invalid numerical input for target.")
            return

        self.robot_cmd_pub.publish(robot_msg)
        data = robot_msg.joint_target if is_joint else list(robot_msg.task_target)
        self.get_logger().info(
            f"Sent robot cmd ({robot_msg.goal_type}): "
            f"{[f'{v:.4f}' for v in data]}")

        try:
            hand_values = [math.radians(float(e.get()))
                           for e in self._hand_target_entries]
        except ValueError:
            self.get_logger().error("Invalid numerical input for hand target.")
            return
        hand_msg = RobotTarget()
        hand_msg.goal_type = 'joint'
        hand_msg.joint_names = HAND_MOTOR_NAMES
        hand_msg.joint_target = hand_values
        self.hand_cmd_pub.publish(hand_msg)
        self.get_logger().info(
            f"Sent hand cmd: {[f'{v:.4f}' for v in hand_values]}")

    # ---- Preset actions --------------------------------------------------------

    def _get_selected_preset(self) -> tuple[str, dict] | None:
        sel = self._preset_tree.selection()
        if not sel:
            messagebox.showwarning("No Selection", "Select a preset first.")
            return None
        name = sel[0]
        return name, self._presets[name]

    def _send_preset(self):
        result = self._get_selected_preset()
        if result is None:
            return
        name, data = result

        # ── Robot target (if present) ──────────────────────────────────
        ctrl_name = data.get('controller')
        if ctrl_name and ctrl_name in CONTROLLER_TYPES:
            # Switch controller if needed
            if self.selected_ctrl.get() != ctrl_name:
                self.selected_ctrl.set(ctrl_name)
                self._on_switch_controller()

            goal_type = data.get(
                'robot_goal_type',
                'joint' if JOINT_SPACE.get(ctrl_name, True) else 'task')
            robot_target_vals = data.get('robot_target', [])

            if len(robot_target_vals) == NUM_JOINTS:
                robot_msg = RobotTarget()
                robot_msg.joint_names = ROBOT_JOINT_NAMES
                if goal_type == 'joint':
                    robot_msg.goal_type = 'joint'
                    robot_msg.joint_target = [
                        math.radians(v) for v in robot_target_vals]
                    self._set_joint_target_entries(robot_msg.joint_target)
                else:
                    robot_msg.goal_type = 'task'
                    task_values = [
                        math.radians(v) if i >= 3 else v
                        for i, v in enumerate(robot_target_vals)]
                    robot_msg.task_target = task_values
                    self._set_task_target_entries(task_values)
                self.robot_cmd_pub.publish(robot_msg)
                self.get_logger().info(
                    f"Preset '{name}': sent robot target ({goal_type}): "
                    f"{[f'{v:.2f}' for v in robot_target_vals]}")

        # ── Hand target ────────────���───────────────────────────────────
        positions_deg = data.get('positions_deg', [0.0] * NUM_HAND_MOTORS)
        grasp_time = data.get('grasp_time', 1.0)
        positions_rad = [math.radians(d) for d in positions_deg]

        # Calculate required hand_trajectory_speed from grasp_time
        max_dist = 0.0
        for target, current in zip(positions_rad, self.current_hand_positions):
            max_dist = max(max_dist, abs(target - current))

        if max_dist > 0.001 and grasp_time > 0.01:
            hand_traj_speed = max_dist / grasp_time
        else:
            hand_traj_speed = 1.0

        # Update gains with the new hand_trajectory_speed
        ctrl = self.selected_ctrl.get()
        speed_idx = _HAND_TRAJ_SPEED_IDX.get(ctrl)
        if speed_idx is not None:
            gains_values: list[float] = []
            for widgets, is_bool in zip(self._gain_entries, self._gain_is_bool):
                for w in widgets:
                    try:
                        gains_values.append(float(w.get()))
                    except ValueError:
                        gains_values.append(0.0)

            if speed_idx < len(gains_values):
                gains_values[speed_idx] = hand_traj_speed
                gains_msg = Float64MultiArray()
                gains_msg.data = gains_values
                self.gains_pub.publish(gains_msg)
                self.get_logger().info(
                    f"Preset '{name}': set hand_traj_speed={hand_traj_speed:.4f} "
                    f"rad/s (grasp_time={grasp_time:.2f}s, max_dist={max_dist:.4f})")

        # Publish hand target
        hand_msg = RobotTarget()
        hand_msg.goal_type = 'joint'
        hand_msg.joint_names = HAND_MOTOR_NAMES
        hand_msg.joint_target = positions_rad
        self.hand_cmd_pub.publish(hand_msg)
        self.get_logger().info(
            f"Sent preset '{name}' ({data.get('type', 'open')}): "
            f"{[f'{v:.2f}' for v in positions_deg]} deg")

        # Also update the hand target entries on the Control tab
        self._set_hand_target_entries(positions_rad)

    def _save_current_as_preset(self):
        name = self._preset_name_entry.get().strip()
        if not name:
            messagebox.showwarning("Missing Name", "Enter a preset name.")
            return
        try:
            grasp_time = float(self._preset_time_entry.get())
        except ValueError:
            messagebox.showerror("Invalid Time", "Grasp time must be a number.")
            return

        positions_deg = [math.degrees(v) for v in self.current_hand_positions]
        preset_data = {
            "type": self._preset_type_var.get(),
            "positions_deg": [round(v, 2) for v in positions_deg],
            "grasp_time": round(grasp_time, 3),
        }
        if self._preset_include_robot_var.get():
            ctrl = self._preset_ctrl_var.get()
            goal_type = self._preset_goal_type_var.get()
            preset_data["controller"] = ctrl
            preset_data["robot_goal_type"] = goal_type
            if goal_type == "joint":
                preset_data["robot_target"] = [
                    round(math.degrees(v), 4) for v in self.current_positions]
            else:
                preset_data["robot_target"] = [
                    round(v, 4) if i < 3 else round(math.degrees(v), 4)
                    for i, v in enumerate(self.current_task_positions)]
        self._presets[name] = preset_data
        self._save_presets_to_file()
        self._refresh_preset_tree()
        self.get_logger().info(f"Saved preset '{name}' from current positions")

    def _save_target_as_preset(self):
        name = self._preset_name_entry.get().strip()
        if not name:
            messagebox.showwarning("Missing Name", "Enter a preset name.")
            return
        try:
            grasp_time = float(self._preset_time_entry.get())
        except ValueError:
            messagebox.showerror("Invalid Time", "Grasp time must be a number.")
            return
        try:
            positions_deg = [float(e.get()) for e in self._hand_target_entries]
        except ValueError:
            messagebox.showerror("Invalid Input", "Hand target entries contain invalid values.")
            return

        preset_data = {
            "type": self._preset_type_var.get(),
            "positions_deg": [round(v, 2) for v in positions_deg],
            "grasp_time": round(grasp_time, 3),
        }
        if self._preset_include_robot_var.get():
            ctrl = self._preset_ctrl_var.get()
            goal_type = self._preset_goal_type_var.get()
            preset_data["controller"] = ctrl
            preset_data["robot_goal_type"] = goal_type
            if goal_type == "joint":
                try:
                    preset_data["robot_target"] = [
                        round(float(e.get()), 4)
                        for e in self._joint_target_entries]
                except ValueError:
                    messagebox.showerror(
                        "Invalid Input", "Joint target entries contain invalid values.")
                    return
            else:
                try:
                    preset_data["robot_target"] = [
                        round(float(e.get()), 4)
                        for e in self._task_target_entries]
                except ValueError:
                    messagebox.showerror(
                        "Invalid Input", "Task target entries contain invalid values.")
                    return
        self._presets[name] = preset_data
        self._save_presets_to_file()
        self._refresh_preset_tree()
        self.get_logger().info(f"Saved preset '{name}' from target entries")

    def _delete_preset(self):
        result = self._get_selected_preset()
        if result is None:
            return
        name, _ = result
        if not messagebox.askyesno("Confirm Delete",
                                   f"Delete preset '{name}'?"):
            return
        del self._presets[name]
        self._save_presets_to_file()
        self._refresh_preset_tree()
        self.get_logger().info(f"Deleted preset '{name}'")

    def _on_close(self):
        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DemoControllerGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
