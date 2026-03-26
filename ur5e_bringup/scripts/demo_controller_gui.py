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
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool
from rtc_msgs.msg import GuiPosition, RobotTarget
import tkinter as tk
from tkinter import ttk, font as tkfont
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
            self.current_positions = list(msg.joint_positions[:NUM_JOINTS])
        if len(msg.task_positions) >= 6:
            self.current_task_positions = list(msg.task_positions[:6])

    def _hand_gui_pos_cb(self, msg: GuiPosition):
        if len(msg.joint_positions) >= NUM_HAND_MOTORS:
            self.current_hand_positions = list(msg.joint_positions[:NUM_HAND_MOTORS])

    def _estop_cb(self, msg: Bool):
        self.estop_active = msg.data

    def _current_gains_cb(self, msg: Float64MultiArray):
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

        # Controller Selection (only Demo Joint / Demo Task)
        ctrl_frame = ttk.LabelFrame(scrollable_frame, text="Controller", padding=4)
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
        self._gains_frame = ttk.LabelFrame(scrollable_frame, text="Gains", padding=4)
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
        pos_frame = ttk.LabelFrame(scrollable_frame, text="Target Inputs",
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
        status_frame = ttk.LabelFrame(scrollable_frame, text="Current Status", padding=4)
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
        btn_frame = tk.Frame(scrollable_frame, bg='#1e1e2e')
        btn_frame.pack(pady=4)
        ttk.Button(btn_frame, text="Copy Current → Target",
                   command=self._copy_current_to_target).pack(side='left', padx=4)
        ttk.Button(btn_frame, text="Send Command",
                   style='Send.TButton',
                   command=self._publish_target).pack(side='left', padx=4)

        # Initial gains build + initial enable/disable state
        self._rebuild_gains_ui(4)
        self._update_target_inputs_state(4)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._gui_ready.set()
        self.root.mainloop()

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

    def _publish_gains(self):
        values: list[float] = []
        for widgets, is_bool in zip(self._gain_entries, self._gain_is_bool):
            for w in widgets:
                try:
                    values.append(float(w.get()))
                except ValueError:
                    self.get_logger().error(
                        "Invalid gain value — check all fields.")
                    return

        msg = Float64MultiArray()
        msg.data = values
        self.gains_pub.publish(msg)

        ctrl_name = CONTROLLER_TYPES[self.selected_ctrl.get()]
        self.get_logger().info(
            f"Applied gains for {ctrl_name}: {[f'{v:.4f}' for v in values]}")

        self._update_applied_display()

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

    def _on_close(self):
        self.root.destroy()
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
            rclpy.shutdown()


if __name__ == '__main__':
    main()
