#!/usr/bin/env python3
"""
UR5e Controller GUI
- Select controller type (P, PD, Pinocchio, CLIK, OSC)
- Set gains per controller via ~/controller_gains
  (gain vectors match UpdateGainsFromMsg layouts in each controller header)
- Displays currently applied gains after "Apply Gains" is pressed
- When switching controller, current joint positions become the new target
- Periodically display current joint positions alongside the target inputs
- E-STOP status indicator via /system/estop_status
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Bool
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk, font as tkfont
import threading

CONTROLLER_TYPES = {
    0: "P Controller",
    1: "PD Controller",
    2: "Pinocchio Controller",
    3: "CLIK Controller",
    4: "OSC Controller",
}

TARGET_LABELS = {
    0: ["q1 (deg)", "q2 (deg)", "q3 (deg)", "q4 (deg)", "q5 (deg)", "q6 (deg)"],
    1: ["q1 (deg)", "q2 (deg)", "q3 (deg)", "q4 (deg)", "q5 (deg)", "q6 (deg)"],
    2: ["q1 (deg)", "q2 (deg)", "q3 (deg)", "q4 (deg)", "q5 (deg)", "q6 (deg)"],
    3: ["X (m)", "Y (m)", "Z (m)", "q4_null (deg)", "q5_null (deg)", "q6_null (deg)"],
    4: ["X (m)", "Y (m)", "Z (m)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"],
}

# Target entry indices that represent angles (require rad ↔ deg conversion)
ANGLE_INDICES = {
    0: [0, 1, 2, 3, 4, 5],
    1: [0, 1, 2, 3, 4, 5],
    2: [0, 1, 2, 3, 4, 5],
    3: [3, 4, 5],
    4: [3, 4, 5],
}

# True → auto-fill target from current joint positions on controller switch
JOINT_SPACE = {0: True, 1: True, 2: True, 3: False, 4: False}

NUM_JOINTS = 6

# ── Gain definitions per controller ──────────────────────────────────────────
# Each entry: (label, size, defaults, is_bool)
#   size     : number of values (6=per-joint, 3=XYZ, 1=scalar/flag)
#   defaults : list of default values (length == size)
#   is_bool  : True → Checkbutton (0/1); False → Entry field
#
# ORDER must match UpdateGainsFromMsg layouts in each controller header:
#   P:         [kp×6]
#   PD:        [kp×6, kd×6]
#   Pinocchio: [kp×6, kd×6, enable_gravity(0/1), enable_coriolis(0/1), trajectory_speed]
#   CLIK:      [kp×3, damping, null_kp, enable_null_space(0/1)]
#   OSC:       [kp_pos×3, kd_pos×3, kp_rot×3, kd_rot×3, damping, enable_gravity(0/1),
#               trajectory_speed, trajectory_angular_speed]
GAIN_DEFS = {
    0: [
        ("kp",              6, [5.0] * 6,  False),
    ],
    1: [
        ("kp",              6, [5.0] * 6,  False),
        ("kd",              6, [0.5] * 6,  False),
    ],
    2: [
        ("kp",              6, [5.0] * 6,  False),
        ("kd",              6, [0.5] * 6,  False),
        ("gravity comp",    1, [1],         True),
        ("coriolis comp",   1, [0],         True),
        ("traj speed",      1, [1.0],       False),
    ],
    3: [
        ("kp",              3, [1.0] * 3,  False),
        ("damping",         1, [0.01],      False),
        ("null_kp",         1, [0.5],       False),
        ("null space",      1, [1],         True),
    ],
    4: [
        ("kp_pos",          3, [1.0] * 3,  False),
        ("kd_pos",          3, [0.1] * 3,  False),
        ("kp_rot",          3, [0.5] * 3,  False),
        ("kd_rot",          3, [0.05] * 3, False),
        ("damping",         1, [0.01],      False),
        ("gravity comp",    1, [0],         True),
        ("traj speed",      1, [0.1],       False),
        ("traj ang speed",  1, [0.5],       False),
    ],
}

# Column header labels for array gain rows
GAIN_COL_HEADERS = {
    0: ["J1", "J2", "J3", "J4", "J5", "J6"],
    1: ["J1", "J2", "J3", "J4", "J5", "J6"],
    2: ["J1", "J2", "J3", "J4", "J5", "J6"],
    3: ["X",  "Y",  "Z"],
    4: ["X",  "Y",  "Z"],
}


class ControllerGUI(Node):
    def __init__(self):
        super().__init__('controller_gui')

        # Publishers
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/target_joint_positions', 10)
        self.type_pub = self.create_publisher(
            Int32, '/rt_controller/controller_type', 10)
        self.gains_pub = self.create_publisher(
            Float64MultiArray, '/rt_controller/controller_gains', 10)

        # Subscriptions
        self.current_positions = [0.0] * NUM_JOINTS
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, 10)
        self.estop_active = False
        self.create_subscription(Bool, '/system/estop_status',
                                 self._estop_cb, 10)

        # 5 Hz refresh timer
        self._refresh_timer = self.create_timer(0.2, self._refresh_current_display)

        # GUI in a daemon thread
        self._gui_ready = threading.Event()
        self._gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self._gui_thread.start()
        self._gui_ready.wait()

    # ──────────────────────── ROS callbacks ───────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        if len(msg.position) >= NUM_JOINTS:
            self.current_positions = list(msg.position[:NUM_JOINTS])

    def _estop_cb(self, msg: Bool):
        self.estop_active = msg.data

    def _refresh_current_display(self):
        if not hasattr(self, '_current_labels'):
            return
        for i, lbl in enumerate(self._current_labels):
            lbl.config(text=f"{math.degrees(self.current_positions[i]):.2f}°")
        if hasattr(self, '_estop_label'):
            if self.estop_active:
                self._estop_label.config(
                    text="  E-STOP ACTIVE  ", fg='#1e1e2e', bg='#f38ba8')
            else:
                self._estop_label.config(
                    text="  NORMAL  ", fg='#1e1e2e', bg='#a6e3a1')

    # ──────────────────────── GUI build ───────────────────────────────────────

    def _run_gui(self):
        self.root = tk.Tk()
        self.root.title("UR5e Controller GUI")
        self.root.geometry("760x920")
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
                        padding=6)
        style.map('TButton', background=[('active', '#45475a')])
        style.configure('Send.TButton', background='#a6e3a1',
                        foreground='#1e1e2e', font=('Segoe UI', 10, 'bold'),
                        padding=8)
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

        # ── Header ────────────────────────────────────────────────────────────
        header_frame = tk.Frame(self.root, bg='#1e1e2e')
        header_frame.pack(fill='x', padx=12, pady=(12, 4))
        tk.Label(header_frame, text="UR5e Controller GUI",
                 bg='#1e1e2e', fg='#89b4fa',
                 font=('Segoe UI', 14, 'bold')).pack(side='left')
        self._estop_label = tk.Label(header_frame, text="  NORMAL  ",
                                     bg='#a6e3a1', fg='#1e1e2e',
                                     font=('Segoe UI', 9, 'bold'),
                                     padx=4, pady=2)
        self._estop_label.pack(side='right')
        tk.Label(header_frame, text="E-STOP:",
                 bg='#1e1e2e', fg='#cdd6f4',
                 font=('Segoe UI', 9)).pack(side='right', padx=(0, 4))

        # ── Controller Selection ───────────────────────────────────────────────
        ctrl_frame = ttk.LabelFrame(self.root, text="Controller", padding=8)
        ctrl_frame.pack(fill='x', padx=12, pady=4)

        self.selected_ctrl = tk.IntVar(value=0)
        btn_row = tk.Frame(ctrl_frame, bg='#1e1e2e')
        btn_row.pack(fill='x')
        for idx, name in CONTROLLER_TYPES.items():
            ttk.Radiobutton(btn_row, text=name, value=idx,
                            variable=self.selected_ctrl,
                            command=self._on_ctrl_radio_change).pack(
                side='left', padx=4)

        switch_btn = ttk.Button(ctrl_frame, text="Switch Controller",
                                style='Switch.TButton',
                                command=self._on_switch_controller)
        switch_btn.pack(pady=(6, 0))

        self._ctrl_status = tk.StringVar(value="Active: P Controller")
        tk.Label(ctrl_frame, textvariable=self._ctrl_status,
                 bg='#1e1e2e', fg='#a6e3a1',
                 font=('Segoe UI', 9, 'italic')).pack()

        # ── Gains ─────────────────────────────────────────────────────────────
        self._gains_frame = ttk.LabelFrame(self.root, text="Gains", padding=8)
        self._gains_frame.pack(fill='x', padx=12, pady=4)

        # _gain_entries[i]         : list of Entry/IntVar widgets for param i
        # _gain_is_bool[i]         : bool flag for param i
        # _applied_label_widgets[i]: list of tk.Label widgets for param i (display only)
        self._gain_entries: list = []
        self._gain_is_bool: list[bool] = []
        self._applied_label_widgets: list = []

        # Editable inputs
        self._gains_inner = tk.Frame(self._gains_frame, bg='#1e1e2e')
        self._gains_inner.pack(fill='x')

        ttk.Button(self._gains_frame, text="Apply Gains",
                   style='Gains.TButton',
                   command=self._publish_gains).pack(pady=(6, 4))

        # Divider + "Currently Applied" read-only section
        ttk.Separator(self._gains_frame, orient='horizontal').pack(
            fill='x', pady=(2, 4))
        tk.Label(self._gains_frame, text="Currently Applied:",
                 bg='#1e1e2e', fg='#a6e3a1',
                 font=('Segoe UI', 8, 'bold')).pack(anchor='w', padx=4)
        self._gains_applied_inner = tk.Frame(self._gains_frame, bg='#1e1e2e')
        self._gains_applied_inner.pack(fill='x', padx=4, pady=(0, 4))

        # ── Position Table ─────────────────────────────────────────────────────
        pos_frame = ttk.LabelFrame(self.root, text="Joint / Task Positions",
                                   padding=8)
        pos_frame.pack(fill='both', expand=True, padx=12, pady=4)

        hdr_font = tkfont.Font(family='Segoe UI', size=9, weight='bold')
        for col, txt in enumerate(["Axis", "Target", "Current"]):
            tk.Label(pos_frame, text=txt, bg='#1e1e2e', fg='#89b4fa',
                     font=hdr_font, width=15, anchor='center').grid(
                row=0, column=col, padx=4, pady=(0, 4))

        self._axis_labels: list[tk.Label] = []
        self._target_entries: list[ttk.Entry] = []
        self._current_labels: list[tk.Label] = []

        for i in range(NUM_JOINTS):
            albl = tk.Label(pos_frame, text=TARGET_LABELS[0][i],
                            bg='#1e1e2e', fg='#cdd6f4', width=15, anchor='e')
            albl.grid(row=i + 1, column=0, padx=4, pady=2)
            self._axis_labels.append(albl)

            ent = ttk.Entry(pos_frame, width=15, justify='center')
            ent.insert(0, "0.0000")
            ent.grid(row=i + 1, column=1, padx=4, pady=2)
            self._target_entries.append(ent)

            clbl = tk.Label(pos_frame, text="---", bg='#313244',
                            fg='#f38ba8', width=15, anchor='center',
                            font=('Courier New', 9))
            clbl.grid(row=i + 1, column=2, padx=4, pady=2)
            self._current_labels.append(clbl)

        # ── Action Buttons ─────────────────────────────────────────────────────
        btn_frame = tk.Frame(self.root, bg='#1e1e2e')
        btn_frame.pack(pady=8)
        ttk.Button(btn_frame, text="Copy Current → Target",
                   command=self._copy_current_to_target).pack(side='left', padx=8)
        ttk.Button(btn_frame, text="Send Command",
                   style='Send.TButton',
                   command=self._publish_target).pack(side='left', padx=8)

        # Initial gains build
        self._rebuild_gains_ui(0)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._gui_ready.set()
        self.root.mainloop()

    # ──────────────────────── Gains UI helpers ────────────────────────────────

    def _rebuild_gains_ui(self, ctrl_idx: int):
        """Clear and rebuild both the editable gain inputs and the applied display."""
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

        # ── Editable inputs ───────────────────────────────────────────────────
        if max_size > 1:
            for j, h in enumerate(headers):
                tk.Label(self._gains_inner, text=h,
                         bg='#1e1e2e', fg='#89b4fa',
                         font=('Segoe UI', 8, 'bold'),
                         width=7, anchor='center').grid(
                    row=0, column=j + 1, padx=2)

        array_row = 1
        scalar_frame = None

        for label, size, defaults, is_bool in defs:
            if size > 1:
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

        # ── Applied display (read-only mirror) ───────────────────────────────
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
        """Refresh the Currently Applied labels from the last sent gain values."""
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

    # ──────────────────────── Button handlers ─────────────────────────────────

    def _on_ctrl_radio_change(self):
        idx = self.selected_ctrl.get()
        self._rebuild_gains_ui(idx)
        for i, lbl in enumerate(self._axis_labels):
            lbl.config(text=TARGET_LABELS[idx][i])

    def _on_switch_controller(self):
        idx = self.selected_ctrl.get()

        msg = Int32()
        msg.data = idx
        self.type_pub.publish(msg)
        self.get_logger().info(
            f"Switched to controller {idx}: {CONTROLLER_TYPES[idx]}")

        self._ctrl_status.set(f"Active: {CONTROLLER_TYPES[idx]}")
        self._rebuild_gains_ui(idx)
        for i, lbl in enumerate(self._axis_labels):
            lbl.config(text=TARGET_LABELS[idx][i])

        if JOINT_SPACE[idx]:
            self._set_target_entries(self.current_positions)

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
        self._set_target_entries(self.current_positions)

    def _set_target_entries(self, values: list[float]):
        angle_indices = ANGLE_INDICES[self.selected_ctrl.get()]
        for i, ent in enumerate(self._target_entries):
            ent.delete(0, tk.END)
            if i in angle_indices:
                ent.insert(0, f"{math.degrees(values[i]):.4f}")
            else:
                ent.insert(0, f"{values[i]:.4f}")

    def _publish_target(self):
        try:
            angle_indices = ANGLE_INDICES[self.selected_ctrl.get()]
            values = []
            for i, e in enumerate(self._target_entries):
                v = float(e.get())
                if i in angle_indices:
                    v = math.radians(v)
                values.append(v)
        except ValueError:
            self.get_logger().error("Invalid numerical input for target.")
            return
        msg = Float64MultiArray()
        msg.data = values
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f"Sent command: {[f'{v:.4f}' for v in values]}")

    def _on_close(self):
        self.root.destroy()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
