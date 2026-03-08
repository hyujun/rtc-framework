#!/usr/bin/env python3
"""
UR5e Controller GUI
- Select controller type (P, PD, Pinocchio, CLIK, OSC)
- Set gains per controller via ~/controller_gains
- When switching controller, current joint positions become the new target
- Periodically display current joint positions alongside the target inputs
- Manually send target via "Send Command" button
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
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
    0: ["q1 (rad)", "q2 (rad)", "q3 (rad)", "q4 (rad)", "q5 (rad)", "q6 (rad)"],
    1: ["q1 (rad)", "q2 (rad)", "q3 (rad)", "q4 (rad)", "q5 (rad)", "q6 (rad)"],
    2: ["q1 (rad)", "q2 (rad)", "q3 (rad)", "q4 (rad)", "q5 (rad)", "q6 (rad)"],
    3: ["X (m)", "Y (m)", "Z (m)", "q4_null (rad)", "q5_null (rad)", "q6_null (rad)"],
    4: ["X (m)", "Y (m)", "Z (m)", "Roll (rad)", "Pitch (rad)", "Yaw (rad)"],
}

# True → auto-fill target from current joint positions on controller switch
JOINT_SPACE = {0: True, 1: True, 2: True, 3: False, 4: False}

NUM_JOINTS = 6

# ── Gain definitions per controller ─────────────────────────────────────────────
# Each entry: list of (label, default_value, is_bool)
#   is_bool=True  → displayed as a Checkbutton (0 or 1)
#   is_bool=False → displayed as an Entry field
GAIN_DEFS = {
    0: [("kp", 5.0, False)],
    1: [("kp", 5.0, False), ("kd", 0.5, False)],
    2: [
        ("kp", 5.0, False),
        ("kd", 0.5, False),
        ("gravity comp", 1, True),
        ("coriolis comp", 0, True),
    ],
    3: [
        ("kp", 1.0, False),
        ("damping", 0.01, False),
        ("null_kp", 0.5, False),
        ("null space", 1, True),
    ],
    4: [
        ("kp_pos", 1.0, False),
        ("kd_pos", 0.1, False),
        ("kp_rot", 0.5, False),
        ("kd_rot", 0.05, False),
        ("damping", 0.01, False),
        ("gravity comp", 0, True),
    ],
}


class ControllerGUI(Node):
    def __init__(self):
        super().__init__('controller_gui')

        # Publishers
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/target_joint_positions', 10)
        self.type_pub = self.create_publisher(
            Int32, '/custom_controller/controller_type', 10)
        self.gains_pub = self.create_publisher(
            Float64MultiArray, '/custom_controller/controller_gains', 10)

        # Subscriber for current joint state
        self.current_positions = [0.0] * NUM_JOINTS
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, 10)

        # Timer to refresh current-position display (5 Hz)
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

    def _refresh_current_display(self):
        if not hasattr(self, '_current_labels'):
            return
        for i, lbl in enumerate(self._current_labels):
            lbl.config(text=f"{self.current_positions[i]:.4f}")

    # ──────────────────────── GUI build ───────────────────────────────────────

    def _run_gui(self):
        self.root = tk.Tk()
        self.root.title("UR5e Controller GUI")
        self.root.geometry("600x680")
        self.root.resizable(False, False)
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

        # Header
        tk.Label(self.root, text="UR5e Controller GUI",
                 bg='#1e1e2e', fg='#89b4fa',
                 font=('Segoe UI', 14, 'bold')).pack(pady=(12, 4))

        # ── Controller Selection ──────────────────────────────────────────────
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

        # Gains content rebuilt dynamically; store widgets here
        self._gain_entries: list = []   # Entry or IntVar depending on is_bool
        self._gain_is_bool: list[bool] = []
        self._gains_inner = tk.Frame(self._gains_frame, bg='#1e1e2e')
        self._gains_inner.pack(fill='x')

        ttk.Button(self._gains_frame, text="Apply Gains",
                   style='Gains.TButton',
                   command=self._publish_gains).pack(pady=(6, 0))

        # ── Position Table ────────────────────────────────────────────────────
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

        # ── Action Buttons ────────────────────────────────────────────────────
        btn_frame = tk.Frame(self.root, bg='#1e1e2e')
        btn_frame.pack(pady=8)
        ttk.Button(btn_frame, text="Copy Current → Target",
                   command=self._copy_current_to_target).pack(side='left', padx=8)
        ttk.Button(btn_frame, text="Send Command",
                   style='Send.TButton',
                   command=self._publish_target).pack(side='left', padx=8)

        # Build initial gains view
        self._rebuild_gains_ui(0)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._gui_ready.set()
        self.root.mainloop()

    # ──────────────────────── Gains UI helpers ────────────────────────────────

    def _rebuild_gains_ui(self, ctrl_idx: int):
        """Clear and rebuild the gain widgets for the given controller."""
        for w in self._gains_inner.winfo_children():
            w.destroy()
        self._gain_entries.clear()
        self._gain_is_bool.clear()

        defs = GAIN_DEFS.get(ctrl_idx, [])
        col = 0
        for label, default, is_bool in defs:
            frm = tk.Frame(self._gains_inner, bg='#1e1e2e')
            frm.grid(row=0, column=col, padx=8, pady=2, sticky='w')

            lbl = tk.Label(frm, text=label, bg='#1e1e2e', fg='#cdd6f4',
                           font=('Segoe UI', 8))
            lbl.pack(anchor='w')

            if is_bool:
                var = tk.IntVar(value=int(default))
                chk = ttk.Checkbutton(frm, variable=var)
                chk.pack(anchor='w')
                self._gain_entries.append(var)
            else:
                ent = ttk.Entry(frm, width=8, justify='center')
                ent.insert(0, str(default))
                ent.pack()
                self._gain_entries.append(ent)
            self._gain_is_bool.append(is_bool)
            col += 1

    # ──────────────────────── Button handlers ─────────────────────────────────

    def _on_ctrl_radio_change(self):
        """Called when the user clicks a radio button (before Switch)."""
        idx = self.selected_ctrl.get()
        self._rebuild_gains_ui(idx)
        # Update axis labels preview
        for i, lbl in enumerate(self._axis_labels):
            lbl.config(text=TARGET_LABELS[idx][i])

    def _on_switch_controller(self):
        idx = self.selected_ctrl.get()

        # Publish controller type
        msg = Int32()
        msg.data = idx
        self.type_pub.publish(msg)
        self.get_logger().info(
            f"Switched to controller {idx}: {CONTROLLER_TYPES[idx]}")

        self._ctrl_status.set(f"Active: {CONTROLLER_TYPES[idx]}")

        # Rebuild gains UI and axis labels after switch
        self._rebuild_gains_ui(idx)
        for i, lbl in enumerate(self._axis_labels):
            lbl.config(text=TARGET_LABELS[idx][i])

        # Auto-fill target from current position for joint-space controllers
        if JOINT_SPACE[idx]:
            self._set_target_entries(self.current_positions)

    def _publish_gains(self):
        values: list[float] = []
        for widget, is_bool in zip(self._gain_entries, self._gain_is_bool):
            if is_bool:
                values.append(float(widget.get()))
            else:
                try:
                    values.append(float(widget.get()))
                except ValueError:
                    self.get_logger().error("Invalid gain value — check all fields.")
                    return

        msg = Float64MultiArray()
        msg.data = values
        self.gains_pub.publish(msg)
        ctrl_name = CONTROLLER_TYPES[self.selected_ctrl.get()]
        self.get_logger().info(
            f"Applied gains for {ctrl_name}: {[f'{v:.4f}' for v in values]}")

    def _copy_current_to_target(self):
        self._set_target_entries(self.current_positions)

    def _set_target_entries(self, values: list[float]):
        for i, ent in enumerate(self._target_entries):
            ent.delete(0, tk.END)
            ent.insert(0, f"{values[i]:.4f}")

    def _publish_target(self):
        try:
            values = [float(e.get()) for e in self._target_entries]
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
