"""DemoControllerGUI Tk app + main entrypoint.

Split out of scripts/demo_controller_gui.py (Phase 0a). Behavior delta = 0:
all module-level constants and value-builders moved to .config; the class
body is unchanged. Future phases may break panel-builder methods out into
their own modules.

- Select between Demo Joint / Demo Task and tune their runtime gains
- Controller switch  → /rtc_cm/switch_controller srv (single-active, D-A1)
- Gain Apply / Load  → AsyncParameterClient on the active controller's
                       LifecycleNode (params declared in DeclareGainParameters
                       in each demo's source file). Read-only caps
                       (*_max_traj_velocity) are skipped on Apply.
- Force-PI grasp     → /<active>/grasp_command srv (rtc_msgs/GraspCommand)
- E-STOP status      → /system/estop_status (subscribe)
- Hand motor target  → /<active>/hand/joint_goal (RobotTarget pub)
- Live joint state   → /<active>/<arm|hand>/gui_position (rebound on every
                       /rtc_cm/active_controller_name transition)
"""

import json
import math
import os
import threading
import tkinter as tk
from tkinter import font as tkfont, messagebox, ttk

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Bool, String

from rtc_msgs.msg import (
    CalibrationCommand,
    CalibrationStatus,
    GraspState,
    GuiPosition,
    RobotTarget,
)
from rtc_msgs.srv import GraspCommand, SwitchController

from .config import (
    _CALIB_STATE_COLORS,
    _CALIB_STATE_NAMES,
    _DEFAULT_PRESETS,
    ANGLE_INDICES,  # noqa: F401  (preserved for downstream compat)
    CONTROLLER_TYPES,
    FINGERTIP_NAMES,
    FORCE_PI_FINGER_NAMES,
    GAIN_DEFS,
    GAIN_GROUP_LAYOUT,
    GAIN_GROUP_PARENT_GRASP,
    GAIN_PARAM_DISPATCH,
    GAIN_ROW_NAMES,
    GRASP_PHASE_NAMES,
    GROUP_SCALARS_PER_ROW,
    JOINT_SPACE,
    SENSOR_CALIBRATIONS,
    TARGET_LABELS,  # noqa: F401  (preserved for downstream compat)
    _read_only,
    _resolve_preset_path,
    _set_bool,
    _set_double,
    _set_double_array,
    _set_int,
)


class DemoControllerGUI(Node):
    def __init__(self):
        super().__init__("demo_controller_gui")

        # Phase 4: target / gui_position / grasp_state / tof are owned by the
        # active controller (/<config_key>/...). We defer creating them until
        # /rtc_cm/active_controller_name tells us the namespace, and re-create
        # them whenever the name changes (controller switch).
        self._active_ctrl: str = ""
        self.robot_cmd_pub = None
        self.hand_cmd_pub = None
        self._arm_gui_sub = None
        self._hand_gui_sub = None
        self._grasp_state_sub = None

        # /rtc_cm/switch_controller srv client (single-CM scope per D-A2)
        self.switch_controller_client = self.create_client(
            SwitchController, "/rtc_cm/switch_controller"
        )

        # AsyncParameterClient + grasp_command client per controller. Created
        # lazily (and rebound on /rtc_cm/active_controller_name change) so we
        # don't pin clients against controllers that may not have come up yet.
        # Param services live on the LifecycleNode FQN /<config_key>/<config_key>;
        # grasp_command is a relative srv under /<config_key>/grasp_command.
        self._param_clients: dict[str, AsyncParameterClient] = {}
        self._grasp_clients: dict[str, rclpy.client.Client] = {}

        # Sensor calibration (Control tab)
        self.calib_cmd_pub = self.create_publisher(
            CalibrationCommand, "/hand/calibration/command", 1
        )
        # sensor_type -> latest CalibrationStatus snapshot
        self._calib_status: dict[int, CalibrationStatus] = {}
        # sensor_type -> Tk StringVar (status label text)
        self._calib_status_vars: dict[int, tk.StringVar] = {}
        # sensor_type -> tk.Label widget (for colour updates)
        self._calib_status_labels: dict[int, tk.Label] = {}
        self.create_subscription(
            CalibrationStatus, "/hand/calibration/status", self._calib_status_cb, 10
        )

        # Phase 1: runtime-discovered robot shape. Starts from a sensible
        # default (UR5e + assm_v1 hand) so widgets build immediately at
        # launch. ``_shape_mismatch_warned`` keeps gui_position callbacks
        # from spamming /rosout — one WARN per (side, observed-name-tuple).
        self._shape: RobotShape = RobotShape.default_ur5e_assm()
        self._shape_mismatch_warned: set[tuple[str, tuple[str, ...]]] = set()

        # Subscriptions (Phase 4: GuiPosition subs created by rewire helper)
        self.current_positions = [0.0] * self._shape.arm_dof
        self.current_task_positions = [0.0] * 6

        self.estop_active = False
        self.create_subscription(Bool, "/system/estop_status", self._estop_cb, 10)

        # _pending_load_gains carries a tk-thread callback to fire after
        # AsyncParameterClient.get_parameters resolves on the executor.
        self._pending_load_gains = False

        # Hand state subscription via gui_position topic (created in rewire)
        self.current_hand_positions = [0.0] * self._shape.hand_dof

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

        # Phase 4: subscribe to the active controller name and rebind
        # controller-owned topics on each change.
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            String, "/rtc_cm/active_controller_name", self._on_active_controller, latched_qos
        )

        # Hand presets (loaded from JSON)
        self._preset_path = _resolve_preset_path()
        self._presets = self._load_presets()

        # Dirty-check caches for GUI refresh (avoid redundant Tk redraws)
        self._prev_status = [""] * 6
        self._prev_task = [""] * 6
        self._prev_hand = [""] * self._shape.hand_dof
        self._prev_estop = ""
        self._prev_grasp_detected = ""
        self._prev_grasp_agg = ["", "", "", ""]
        self._prev_ft = [["", "", ""] for _ in range(len(FINGERTIP_NAMES))]
        self._prev_fp_phase = ""
        self._prev_fp_target = ""
        self._prev_fp_fingers = [["", "", "", ""] for _ in range(len(FORCE_PI_FINGER_NAMES))]

        # GUI in a daemon thread
        self._gui_ready = threading.Event()
        self._gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self._gui_thread.start()
        self._gui_ready.wait()

    # ---- ROS callbacks -------------------------------------------------------

    def _on_active_controller(self, msg: String):
        """Rewire controller-owned pubs/subs whenever the active controller
        changes (Phase 4 dynamic remapping)."""
        name = (msg.data or "").strip()
        if not name or name == self._active_ctrl:
            return
        self._active_ctrl = name
        ns = "/" + name

        # Reset old sub handles by dropping references; rclpy will unsubscribe.
        for sub_attr in ("_arm_gui_sub", "_hand_gui_sub", "_grasp_state_sub"):
            sub = getattr(self, sub_attr, None)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:
                    pass
                setattr(self, sub_attr, None)
        for pub_attr in ("robot_cmd_pub", "hand_cmd_pub"):
            pub = getattr(self, pub_attr, None)
            if pub is not None:
                try:
                    self.destroy_publisher(pub)
                except Exception:
                    pass
                setattr(self, pub_attr, None)

        self.robot_cmd_pub = self.create_publisher(RobotTarget, ns + "/ur5e/joint_goal", 10)
        self.hand_cmd_pub = self.create_publisher(RobotTarget, ns + "/hand/joint_goal", 10)
        self._arm_gui_sub = self.create_subscription(
            GuiPosition, ns + "/ur5e/gui_position", self._gui_pos_cb, 10
        )
        self._hand_gui_sub = self.create_subscription(
            GuiPosition, ns + "/hand/gui_position", self._hand_gui_pos_cb, 10
        )
        self._grasp_state_sub = self.create_subscription(
            GraspState, ns + "/hand/grasp_state", self._grasp_state_cb, 10
        )
        self.get_logger().info(f"rewired controller-owned topics to '{name}'")

    def _warn_shape_mismatch_once(self, side: str, observed_names: list[str]) -> None:
        """Emit a one-shot WARN per side when the active controller's
        joint_names span doesn't match the GUI's RobotShape.

        Phase 1 step 3 keeps Tk widgets fixed at the startup default
        (``RobotShape.default_ur5e_assm()``), so this is purely advisory:
        the user should restart the GUI after attaching a different
        robot/hand. A future phase may convert this into a live widget
        rebuild — at which point this helper goes away.
        """
        key = (side, tuple(observed_names))
        if key in self._shape_mismatch_warned:
            return
        self._shape_mismatch_warned.add(key)
        expected = self._shape.arm_joint_names if side == "arm" else self._shape.hand_motor_names
        self.get_logger().warn(
            f"{side} joint_names mismatch: GUI is wired for {list(expected)} "
            f"but controller publishes {observed_names}. Restart the GUI to "
            "pick up the new robot/hand schema."
        )

    def _gui_pos_cb(self, msg: GuiPosition):
        arm_dof = self._shape.arm_dof
        if len(msg.joint_positions) >= arm_dof:
            if msg.joint_names and len(msg.joint_names) >= arm_dof:
                names = list(msg.joint_names[:arm_dof])
                if not self._shape.matches_message(names, arm_dof):
                    self._warn_shape_mismatch_once("arm", names)
                idx = self._shape.arm_name_to_idx
                reordered = [0.0] * arm_dof
                for mi, name in enumerate(names):
                    if name in idx:
                        reordered[idx[name]] = msg.joint_positions[mi]
                self.current_positions = reordered
            else:
                self.current_positions = list(msg.joint_positions[:arm_dof])
        if len(msg.task_positions) >= 6:
            self.current_task_positions = list(msg.task_positions[:6])

    def _hand_gui_pos_cb(self, msg: GuiPosition):
        hand_dof = self._shape.hand_dof
        if len(msg.joint_positions) >= hand_dof:
            if msg.joint_names and len(msg.joint_names) >= hand_dof:
                names = list(msg.joint_names[:hand_dof])
                if not self._shape.matches_message(names, hand_dof):
                    self._warn_shape_mismatch_once("hand", names)
                idx = self._shape.hand_name_to_idx
                reordered = [0.0] * hand_dof
                for mi, name in enumerate(names):
                    if name in idx:
                        reordered[idx[name]] = msg.joint_positions[mi]
                self.current_hand_positions = reordered
            else:
                self.current_hand_positions = list(msg.joint_positions[:hand_dof])

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

    # ---- Sensor calibration -----------------------------------------------------

    def _send_calibration(self, sensor_type: int, action: int, sample_count: int = 0):
        """Publish a CalibrationCommand for the given sensor."""
        msg = CalibrationCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sensor_type = sensor_type
        msg.action = action
        msg.sample_count = sample_count
        self.calib_cmd_pub.publish(msg)
        self.get_logger().info(
            f"Calibration command published: sensor_type={sensor_type} "
            f"action={action} sample_count={sample_count}"
        )

    def _calib_status_cb(self, msg: CalibrationStatus):
        """Store latest status snapshot (label update happens on Tk thread)."""
        self._calib_status[msg.sensor_type] = msg

    # ---- Preset persistence -----------------------------------------------------

    def _load_presets(self) -> dict:
        try:
            with open(self._preset_path) as f:
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
            with open(self._preset_path, "w") as f:
                json.dump(presets, f, indent=2)
        except OSError as e:
            self.get_logger().error(f"Failed to save presets: {e}")

    # ── /rtc_cm/* + per-controller parameter helpers ─────────────────────

    def _get_param_client(self, ctrl: str) -> AsyncParameterClient:
        """Return (lazily creating) an AsyncParameterClient targeting the
        active controller's LifecycleNode. The node FQN is /<ctrl>/<ctrl>
        because each controller's LifecycleNode is constructed with
        namespace=/<ctrl> and node-name=<ctrl> (see RtControllerNode."""
        client = self._param_clients.get(ctrl)
        if client is None:
            client = AsyncParameterClient(self, f"/{ctrl}/{ctrl}")
            self._param_clients[ctrl] = client
        return client

    def _get_grasp_client(self, ctrl: str) -> "rclpy.client.Client":
        client = self._grasp_clients.get(ctrl)
        if client is None:
            client = self.create_client(GraspCommand, f"/{ctrl}/grasp_command")
            self._grasp_clients[ctrl] = client
        return client

    def _schedule_refresh(self):
        """Periodic GUI refresh scheduled on the Tk event loop (thread-safe)."""
        self._refresh_current_display()
        self.root.after(200, self._schedule_refresh)

    def _refresh_current_display(self):
        """Update display labels with dirty checking. Only redraws changed values."""
        # ── Arm Joint Positions ──
        for i in range(self._shape.arm_dof):
            val_rad = self.current_positions[i]
            text = f"{val_rad:.4f} rad  ({math.degrees(val_rad):.2f}°)"
            if self._prev_status[i] != text:
                self._prev_status[i] = text
                self._status_labels_values[i].config(text=text)

        # ── End-Effector Pose ──
        for i in range(6):
            val = self.current_task_positions[i]
            text = f"{val:.4f} m" if i < 3 else f"{val:.4f} rad  ({math.degrees(val):.2f}°)"
            if self._prev_task[i] != text:
                self._prev_task[i] = text
                self._task_state_labels_values[i].config(text=text)

        # ── Hand state ──
        for i in range(self._shape.hand_dof):
            text = f"{math.degrees(self.current_hand_positions[i]):.2f}°"
            if self._prev_hand[i] != text:
                self._prev_hand[i] = text
                self._hand_state_labels_values[i].config(text=text)

        # ── E-STOP ──
        estop_key = "1" if self.estop_active else "0"
        if self._prev_estop != estop_key:
            self._prev_estop = estop_key
            if self.estop_active:
                self._estop_label.config(text="  E-STOP ACTIVE  ", fg="#1e1e2e", bg="#f38ba8")
            else:
                self._estop_label.config(text="  NORMAL  ", fg="#1e1e2e", bg="#a6e3a1")

        # ── Grasp state ──
        grasp_key = "1" if self._grasp_detected else "0"
        if self._prev_grasp_detected != grasp_key:
            self._prev_grasp_detected = grasp_key
            if self._grasp_detected:
                self._grasp_detected_label.config(
                    text="  GRASP DETECTED  ", bg="#a6e3a1", fg="#1e1e2e"
                )
            else:
                self._grasp_detected_label.config(text="  NO GRASP  ", bg="#585b70", fg="#cdd6f4")

        agg_texts = [
            f"{self._grasp_num_active}",
            f"{self._grasp_max_force:.2f} N",
            f"{self._grasp_force_threshold:.2f} N",
            f"{self._grasp_min_fingertips}",
        ]
        agg_labels = [
            self._grasp_active_label,
            self._grasp_maxforce_label,
            self._grasp_threshold_label,
            self._grasp_minfinger_label,
        ]
        for i, (text, lbl) in enumerate(zip(agg_texts, agg_labels, strict=False)):
            if self._prev_grasp_agg[i] != text:
                self._prev_grasp_agg[i] = text
                lbl.config(text=text)

        # ── Per-fingertip ──
        for i in range(len(FINGERTIP_NAMES)):
            force_text = f"{self._grasp_force_mag[i]:.2f} N"
            if self._prev_ft[i][0] != force_text:
                self._prev_ft[i][0] = force_text
                self._ft_force_labels[i].config(text=force_text)

            cf = self._grasp_contact_flag[i]
            contact_text = f"{cf:.2f}"
            if self._prev_ft[i][1] != contact_text:
                self._prev_ft[i][1] = contact_text
                contact_color = "#a6e3a1" if cf > 0.5 else "#585b70"
                self._ft_contact_labels[i].config(
                    text=contact_text, bg=contact_color, fg="#1e1e2e" if cf > 0.5 else "#cdd6f4"
                )

            iv = self._grasp_inference_valid[i]
            valid_text = "OK" if iv else "--"
            if self._prev_ft[i][2] != valid_text:
                self._prev_ft[i][2] = valid_text
                self._ft_valid_labels[i].config(text=valid_text, fg="#a6e3a1" if iv else "#f38ba8")

        # ── Force-PI state ──
        phase_key = str(self._grasp_phase)
        if self._prev_fp_phase != phase_key:
            self._prev_fp_phase = phase_key
            phase_info = GRASP_PHASE_NAMES.get(
                self._grasp_phase, ("UNKNOWN", "#585b70", "#cdd6f4")
            )
            self._fp_phase_label.config(
                text=f"  {phase_info[0]}  ", bg=phase_info[1], fg=phase_info[2]
            )

        target_text = f"{self._grasp_target_force_val:.2f} N"
        if self._prev_fp_target != target_text:
            self._prev_fp_target = target_text
            self._fp_target_force_display.config(text=target_text)

        for i in range(len(FORCE_PI_FINGER_NAMES)):
            s_text = f"{self._fp_finger_s[i]:.3f}"
            if self._prev_fp_fingers[i][0] != s_text:
                self._prev_fp_fingers[i][0] = s_text
                self._fp_s_labels[i].config(text=s_text)

            filt_text = f"{self._fp_filtered_force[i]:.2f} N"
            if self._prev_fp_fingers[i][1] != filt_text:
                self._prev_fp_fingers[i][1] = filt_text
                self._fp_filt_labels[i].config(text=filt_text)

            err = self._fp_force_error[i]
            err_text = f"{err:+.2f} N"
            if self._prev_fp_fingers[i][2] != err_text:
                self._prev_fp_fingers[i][2] = err_text
                err_color = (
                    "#a6e3a1" if abs(err) < 0.2 else "#f9e2af" if abs(err) < 0.5 else "#f38ba8"
                )
                self._fp_err_labels[i].config(text=err_text, fg=err_color)

            f_desired = self._fp_filtered_force[i] + err
            desired_text = f"{f_desired:.2f} N"
            if self._prev_fp_fingers[i][3] != desired_text:
                self._prev_fp_fingers[i][3] = desired_text
                self._fp_desired_labels[i].config(text=desired_text)

        # ── Sensor calibration status ──
        for stype, var in self._calib_status_vars.items():
            snap = self._calib_status.get(stype)
            if snap is None:
                continue
            state_name = _CALIB_STATE_NAMES.get(snap.state, "?")
            if snap.state == CalibrationStatus.STATE_RUNNING:
                text = f"{state_name} {snap.progress_count}/{snap.target_count}"
            elif snap.state == CalibrationStatus.STATE_COMPLETE:
                text = f"{state_name} ({snap.target_count} samples)"
            else:
                text = state_name
            if var.get() != text:
                var.set(text)
                color = _CALIB_STATE_COLORS.get(snap.state, "#cdd6f4")
                lbl = self._calib_status_labels.get(stype)
                if lbl is not None:
                    lbl.config(fg=color)

    # ---- GUI build -----------------------------------------------------------

    def _run_gui(self):
        self.root = tk.Tk()
        self.root.title("Demo Controller GUI")
        self.root.geometry("1000x900")
        self.root.resizable(True, True)
        self.root.configure(bg="#1e1e2e")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background="#1e1e2e", foreground="#cdd6f4", font=("Segoe UI", 9))
        style.configure(
            "TLabelframe", background="#1e1e2e", foreground="#89b4fa", font=("Segoe UI", 9, "bold")
        )
        style.configure("TLabelframe.Label", background="#1e1e2e", foreground="#89b4fa")
        style.configure("TRadiobutton", background="#1e1e2e", foreground="#cdd6f4")
        style.configure("TCheckbutton", background="#1e1e2e", foreground="#cdd6f4")
        style.configure("TButton", background="#313244", foreground="#cdd6f4", padding=3)
        style.map("TButton", background=[("active", "#45475a")])
        style.configure(
            "Send.TButton",
            background="#a6e3a1",
            foreground="#1e1e2e",
            font=("Segoe UI", 9, "bold"),
            padding=5,
        )
        style.map("Send.TButton", background=[("active", "#94e2d5")])
        style.configure(
            "Gains.TButton",
            background="#fab387",
            foreground="#1e1e2e",
            font=("Segoe UI", 9, "bold"),
        )
        style.map("Gains.TButton", background=[("active", "#f9e2af")])
        style.configure(
            "Switch.TButton",
            background="#89b4fa",
            foreground="#1e1e2e",
            font=("Segoe UI", 9, "bold"),
        )
        style.map("Switch.TButton", background=[("active", "#b4befe")])
        style.configure(
            "TEntry", fieldbackground="#313244", foreground="#cdd6f4", insertcolor="#cdd6f4"
        )
        style.configure("TSeparator", background="#45475a")
        style.configure("TNotebook", background="#1e1e2e", borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            background="#313244",
            foreground="#cdd6f4",
            padding=[10, 4],
            font=("Segoe UI", 9, "bold"),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", "#89b4fa")],
            foreground=[("selected", "#1e1e2e")],
        )
        style.configure(
            "Preset.TButton",
            background="#cba6f7",
            foreground="#1e1e2e",
            font=("Segoe UI", 9, "bold"),
        )
        style.map("Preset.TButton", background=[("active", "#f5c2e7")])
        style.configure(
            "Treeview",
            background="#313244",
            foreground="#cdd6f4",
            fieldbackground="#313244",
            font=("Segoe UI", 9),
        )
        style.configure(
            "Treeview.Heading",
            background="#45475a",
            foreground="#cdd6f4",
            font=("Segoe UI", 9, "bold"),
        )
        style.map(
            "Treeview", background=[("selected", "#585b70")], foreground=[("selected", "#cdd6f4")]
        )
        style.configure(
            "TCombobox", fieldbackground="#313244", background="#313244", foreground="#cdd6f4"
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", "#313244")],
            foreground=[("readonly", "#cdd6f4")],
        )

        # Scrollable wrapper
        outer_frame = tk.Frame(self.root, bg="#1e1e2e")
        outer_frame.pack(fill="both", expand=True)

        v_scrollbar = ttk.Scrollbar(outer_frame, orient="vertical")
        v_scrollbar.pack(side="right", fill="y")
        h_scrollbar = ttk.Scrollbar(outer_frame, orient="horizontal")
        h_scrollbar.pack(side="bottom", fill="x")

        canvas = tk.Canvas(
            outer_frame,
            bg="#1e1e2e",
            yscrollcommand=v_scrollbar.set,
            xscrollcommand=h_scrollbar.set,
            highlightthickness=0,
        )
        canvas.pack(side="left", fill="both", expand=True)

        v_scrollbar.config(command=canvas.yview)
        h_scrollbar.config(command=canvas.xview)

        scrollable_frame = tk.Frame(canvas, bg="#1e1e2e")
        canvas_window = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")

        def _on_frame_configure(_event):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _on_canvas_configure(event):
            canvas.itemconfig(canvas_window, width=event.width)

        scrollable_frame.bind("<Configure>", _on_frame_configure)
        canvas.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(event):
            if event.num == 4:
                canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                canvas.yview_scroll(1, "units")
            else:
                canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        canvas.bind_all("<Button-4>", _on_mousewheel)
        canvas.bind_all("<Button-5>", _on_mousewheel)

        # Header
        header_frame = tk.Frame(scrollable_frame, bg="#1e1e2e")
        header_frame.pack(fill="x", padx=8, pady=(6, 2))
        tk.Label(
            header_frame,
            text="Demo Controller GUI",
            bg="#1e1e2e",
            fg="#89b4fa",
            font=("Segoe UI", 12, "bold"),
        ).pack(side="left")
        self._estop_label = tk.Label(
            header_frame,
            text="  NORMAL  ",
            bg="#a6e3a1",
            fg="#1e1e2e",
            font=("Segoe UI", 9, "bold"),
            padx=4,
            pady=2,
        )
        self._estop_label.pack(side="right")
        tk.Label(
            header_frame, text="E-STOP:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 9)
        ).pack(side="right", padx=(0, 4))

        # ── Notebook (Tabs) ──────────────────────────────────────────────
        notebook = ttk.Notebook(scrollable_frame)
        notebook.pack(fill="both", expand=True, padx=8, pady=2)

        control_tab = tk.Frame(notebook, bg="#1e1e2e")
        notebook.add(control_tab, text="  Control  ")

        grasp_tab = tk.Frame(notebook, bg="#1e1e2e")
        notebook.add(grasp_tab, text="  Grasp  ")

        # ══════════════════════════════════════════════════════════════════
        #  CONTROL TAB (existing UI)
        # ══════════════════════════════════════════════════════════════════

        # Controller Selection (only Demo Joint / Demo Task)
        ctrl_frame = ttk.LabelFrame(control_tab, text="Controller", padding=4)
        ctrl_frame.pack(fill="x", padx=8, pady=2)

        self.selected_ctrl = tk.StringVar(value="demo_joint_controller")
        btn_row = tk.Frame(ctrl_frame, bg="#1e1e2e")
        btn_row.pack(fill="x")
        for idx, name in CONTROLLER_TYPES.items():
            ttk.Radiobutton(
                btn_row,
                text=name,
                value=idx,
                variable=self.selected_ctrl,
                command=self._on_ctrl_radio_change,
            ).pack(side="left", padx=2)

        switch_btn = ttk.Button(
            ctrl_frame,
            text="Switch Controller",
            style="Switch.TButton",
            command=self._on_switch_controller,
        )
        switch_btn.pack(pady=(3, 0))

        self._ctrl_status = tk.StringVar(value="Active: Demo Joint Controller")
        tk.Label(
            ctrl_frame,
            textvariable=self._ctrl_status,
            bg="#1e1e2e",
            fg="#a6e3a1",
            font=("Segoe UI", 9, "italic"),
        ).pack()

        # Gains
        self._gains_frame = ttk.LabelFrame(control_tab, text="Gains", padding=4)
        self._gains_frame.pack(fill="x", padx=8, pady=2)

        self._gain_entries: list = []
        self._gain_is_bool: list[bool] = []
        self._applied_label_widgets: list = []

        self._gains_inner = tk.Frame(self._gains_frame, bg="#1e1e2e")
        self._gains_inner.pack(fill="x")

        gains_btn_frame = tk.Frame(self._gains_frame, bg="#1e1e2e")
        gains_btn_frame.pack(pady=(3, 2))
        ttk.Button(
            gains_btn_frame, text="Apply Gains", style="Gains.TButton", command=self._publish_gains
        ).pack(side="left", padx=4)
        ttk.Button(
            gains_btn_frame,
            text="Load Gain",
            style="Switch.TButton",
            command=self._request_load_gains,
        ).pack(side="left", padx=4)

        ttk.Separator(self._gains_frame, orient="horizontal").pack(fill="x", pady=(1, 2))
        tk.Label(
            self._gains_frame,
            text="Currently Applied:",
            bg="#1e1e2e",
            fg="#a6e3a1",
            font=("Segoe UI", 8, "bold"),
        ).pack(anchor="w", padx=2)
        self._gains_applied_inner = tk.Frame(self._gains_frame, bg="#1e1e2e")
        self._gains_applied_inner.pack(fill="x", padx=2, pady=(0, 2))

        # Target Inputs
        pos_frame = ttk.LabelFrame(control_tab, text="Target Inputs", padding=4)
        pos_frame.pack(fill="both", expand=True, padx=8, pady=2)

        hdr_font = tkfont.Font(family="Segoe UI", size=9, weight="bold")

        # Top row: Joint + Task targets side by side
        top_target_row = tk.Frame(pos_frame, bg="#1e1e2e")
        top_target_row.pack(fill="x")

        # Left: Joint Target
        left_target_frame = tk.Frame(top_target_row, bg="#1e1e2e")
        left_target_frame.pack(side="left", fill="y", padx=(0, 2))

        tk.Label(
            left_target_frame, text="Joint Target", bg="#1e1e2e", fg="#89b4fa", font=hdr_font
        ).grid(row=0, column=0, columnspan=3, pady=(0, 1))
        for col, txt in enumerate(["Axis", "Target", "Step (±)"]):
            w = 10 if col != 2 else 6
            tk.Label(
                left_target_frame,
                text=txt,
                bg="#1e1e2e",
                fg="#89b4fa",
                font=hdr_font,
                width=w,
                anchor="center",
            ).grid(row=1, column=col, padx=2, pady=(0, 2))

        _joint_axis_labels = [
            "q1 (deg)",
            "q2 (deg)",
            "q3 (deg)",
            "q4 (deg)",
            "q5 (deg)",
            "q6 (deg)",
        ]
        self._joint_target_entries: list[ttk.Entry] = []
        self._joint_step_entries: list[ttk.Entry] = []
        self._joint_step_btns: list[list] = []

        for i in range(self._shape.arm_dof):
            label = _joint_axis_labels[i] if i < len(_joint_axis_labels) else f"q{i + 1} (deg)"
            tk.Label(
                left_target_frame,
                text=label,
                bg="#1e1e2e",
                fg="#cdd6f4",
                width=10,
                anchor="e",
            ).grid(row=i + 2, column=0, padx=2, pady=1)

            ent = ttk.Entry(left_target_frame, width=12, justify="center")
            ent.insert(0, "0.0000")
            ent.grid(row=i + 2, column=1, padx=2, pady=1)
            self._joint_target_entries.append(ent)

            bf = tk.Frame(left_target_frame, bg="#1e1e2e")
            bf.grid(row=i + 2, column=2, padx=1, pady=1)
            btn_m = ttk.Button(
                bf, text="-", width=2, command=lambda idx=i: self._add_joint_step(idx, -1)
            )
            btn_m.pack(side="left", padx=1)
            step_ent = ttk.Entry(bf, width=5, justify="center")
            step_ent.insert(0, "1.0")
            step_ent.pack(side="left", padx=1)
            btn_p = ttk.Button(
                bf, text="+", width=2, command=lambda idx=i: self._add_joint_step(idx, 1)
            )
            btn_p.pack(side="left", padx=1)
            self._joint_step_entries.append(step_ent)
            self._joint_step_btns.append([btn_m, btn_p])

        # Vertical separator
        ttk.Separator(top_target_row, orient="vertical").pack(side="left", fill="y", padx=6)

        # Right: Task Target
        right_target_frame = tk.Frame(top_target_row, bg="#1e1e2e")
        right_target_frame.pack(side="left", fill="y", padx=(2, 0))

        tk.Label(
            right_target_frame, text="Task Target", bg="#1e1e2e", fg="#89b4fa", font=hdr_font
        ).grid(row=0, column=0, columnspan=3, pady=(0, 1))
        for col, txt in enumerate(["Axis", "Target", "Step (±)"]):
            w = 10 if col != 2 else 6
            tk.Label(
                right_target_frame,
                text=txt,
                bg="#1e1e2e",
                fg="#89b4fa",
                font=hdr_font,
                width=w,
                anchor="center",
            ).grid(row=1, column=col, padx=2, pady=(0, 2))

        _task_axis_labels = ["X (m)", "Y (m)", "Z (m)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
        self._task_target_entries: list[ttk.Entry] = []
        self._task_step_entries: list[ttk.Entry] = []
        self._task_step_btns: list[list] = []

        # Task target is always 6-D (X, Y, Z, Roll, Pitch, Yaw) regardless
        # of arm DoF — the controller resolves IK / null-space against the
        # current arm DoF on its end.
        for i in range(6):
            tk.Label(
                right_target_frame,
                text=_task_axis_labels[i],
                bg="#1e1e2e",
                fg="#cdd6f4",
                width=10,
                anchor="e",
            ).grid(row=i + 2, column=0, padx=2, pady=1)

            ent = ttk.Entry(right_target_frame, width=12, justify="center")
            ent.insert(0, "0.0000")
            ent.grid(row=i + 2, column=1, padx=2, pady=1)
            self._task_target_entries.append(ent)

            bf = tk.Frame(right_target_frame, bg="#1e1e2e")
            bf.grid(row=i + 2, column=2, padx=1, pady=1)
            btn_m = ttk.Button(
                bf, text="-", width=2, command=lambda idx=i: self._add_task_step(idx, -1)
            )
            btn_m.pack(side="left", padx=1)
            step_ent = ttk.Entry(bf, width=5, justify="center")
            step_ent.insert(0, "0.01" if i < 3 else "1.0")
            step_ent.pack(side="left", padx=1)
            btn_p = ttk.Button(
                bf, text="+", width=2, command=lambda idx=i: self._add_task_step(idx, 1)
            )
            btn_p.pack(side="left", padx=1)
            self._task_step_entries.append(step_ent)
            self._task_step_btns.append([btn_m, btn_p])

        # Hand Motor Target
        ttk.Separator(pos_frame, orient="horizontal").pack(fill="x", pady=4)

        hand_target_label = tk.Label(
            pos_frame, text="Hand Motor Target", bg="#1e1e2e", fg="#89b4fa", font=hdr_font
        )
        hand_target_label.pack(anchor="w", padx=4)

        hand_target_frame = tk.Frame(pos_frame, bg="#1e1e2e")
        hand_target_frame.pack(fill="x", padx=4)

        self._hand_target_entries: list[ttk.Entry] = []
        self._hand_step_entries: list[ttk.Entry] = []
        self._hand_step_btns: list[list] = []

        for col_idx, (finger_name, motors) in enumerate(self._shape.hand_finger_groups):
            col_frame = tk.Frame(hand_target_frame, bg="#1e1e2e")
            col_frame.grid(row=0, column=col_idx, padx=4, sticky="n")

            tk.Label(col_frame, text=finger_name, bg="#1e1e2e", fg="#f9e2af", font=hdr_font).grid(
                row=0, column=0, columnspan=3, pady=(0, 2)
            )

            for row_idx, motor_name in enumerate(motors):
                short_name = motor_name.split("_", 1)[1] if "_" in motor_name else motor_name
                tk.Label(
                    col_frame,
                    text=short_name,
                    bg="#1e1e2e",
                    fg="#cdd6f4",
                    width=8,
                    anchor="e",
                    font=("Segoe UI", 8),
                ).grid(row=row_idx + 1, column=0, padx=1, pady=1)

                ent = ttk.Entry(col_frame, width=8, justify="center")
                ent.insert(0, "0.00")
                ent.grid(row=row_idx + 1, column=1, padx=1, pady=1)
                self._hand_target_entries.append(ent)

                bf = tk.Frame(col_frame, bg="#1e1e2e")
                bf.grid(row=row_idx + 1, column=2, padx=1, pady=1)
                hand_idx = len(self._hand_target_entries) - 1
                btn_m = ttk.Button(
                    bf, text="-", width=2, command=lambda hi=hand_idx: self._add_hand_step(hi, -1)
                )
                btn_m.pack(side="left")
                step_ent = ttk.Entry(bf, width=4, justify="center")
                step_ent.insert(0, "1.0")
                step_ent.pack(side="left")
                btn_p = ttk.Button(
                    bf, text="+", width=2, command=lambda hi=hand_idx: self._add_hand_step(hi, 1)
                )
                btn_p.pack(side="left")
                self._hand_step_entries.append(step_ent)
                self._hand_step_btns.append([btn_m, btn_p])

        # Live Feedback: Arm Joint / End-Effector Pose / Hand Motor
        status_frame = ttk.LabelFrame(control_tab, text="Live Feedback", padding=4)
        status_frame.pack(fill="both", expand=False, padx=8, pady=2)
        status_frame.columnconfigure(0, weight=1)
        status_frame.columnconfigure(1, weight=1)

        # Row 0, Col 0: Arm Joint Positions (always q1..q6)
        joint_frame = ttk.LabelFrame(status_frame, text="Arm Joint Positions", padding=4)
        joint_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 2), pady=(0, 2))

        self._status_labels_names: list[tk.Label] = []
        self._status_labels_values: list[tk.Label] = []

        for i in range(6):
            name_lbl = tk.Label(
                joint_frame,
                text=f"J{i + 1}:",
                bg="#1e1e2e",
                fg="#cdd6f4",
                width=8,
                anchor="e",
                font=("Segoe UI", 9, "bold"),
            )
            name_lbl.grid(row=i, column=0, padx=(4, 2), pady=1)
            self._status_labels_names.append(name_lbl)

            val_lbl = tk.Label(
                joint_frame,
                text="0.0000 rad  (0.00°)",
                bg="#313244",
                fg="#a6e3a1",
                width=22,
                anchor="center",
                font=("Courier New", 9, "bold"),
            )
            val_lbl.grid(row=i, column=1, padx=3, pady=1)
            self._status_labels_values.append(val_lbl)

        # Row 0, Col 1: End-Effector Pose (always task-space)
        ee_frame = ttk.LabelFrame(status_frame, text="End-Effector Pose", padding=4)
        ee_frame.grid(row=0, column=1, sticky="nsew", padx=(2, 0), pady=(0, 2))

        _task_state_row_labels = ["X (m)", "Y (m)", "Z (m)", "Roll", "Pitch", "Yaw"]
        self._task_state_labels_values: list[tk.Label] = []

        for i, label in enumerate(_task_state_row_labels):
            tk.Label(
                ee_frame,
                text=f"{label}:",
                bg="#1e1e2e",
                fg="#cdd6f4",
                width=8,
                anchor="e",
                font=("Segoe UI", 9, "bold"),
            ).grid(row=i, column=0, padx=(4, 2), pady=1)

            val_lbl = tk.Label(
                ee_frame,
                text="0.0000",
                bg="#313244",
                fg="#cba6f7",
                width=22,
                anchor="center",
                font=("Courier New", 9, "bold"),
            )
            val_lbl.grid(row=i, column=1, padx=3, pady=1)
            self._task_state_labels_values.append(val_lbl)

        # Row 1: Hand Motor Positions (full width)
        hand_status_frame = ttk.LabelFrame(status_frame, text="Hand Motor Positions", padding=4)
        hand_status_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(2, 0))

        self._hand_state_labels_values: list[tk.Label] = []

        for col_idx, (finger_name, motors) in enumerate(self._shape.hand_finger_groups):
            col_frame = tk.Frame(hand_status_frame, bg="#1e1e2e")
            col_frame.grid(row=0, column=col_idx, padx=2, sticky="n")

            tk.Label(
                col_frame,
                text=finger_name,
                bg="#1e1e2e",
                fg="#f9e2af",
                font=("Segoe UI", 8, "bold"),
            ).pack(anchor="center")

            for motor_name in motors:
                short_name = motor_name.split("_", 1)[1] if "_" in motor_name else motor_name
                row_frame = tk.Frame(col_frame, bg="#1e1e2e")
                row_frame.pack(fill="x", pady=1)
                tk.Label(
                    row_frame,
                    text=f"{short_name}:",
                    bg="#1e1e2e",
                    fg="#cdd6f4",
                    width=7,
                    anchor="e",
                    font=("Segoe UI", 8),
                ).pack(side="left")
                val_lbl = tk.Label(
                    row_frame,
                    text="0.00°",
                    bg="#313244",
                    fg="#f9e2af",
                    width=8,
                    anchor="center",
                    font=("Courier New", 8, "bold"),
                )
                val_lbl.pack(side="left", padx=2)
                self._hand_state_labels_values.append(val_lbl)

        # Action Buttons
        btn_frame = tk.Frame(control_tab, bg="#1e1e2e")
        btn_frame.pack(pady=4)
        ttk.Button(
            btn_frame, text="Copy Current → Target", command=self._copy_current_to_target
        ).pack(side="left", padx=4)
        ttk.Button(
            btn_frame, text="Send Command", style="Send.TButton", command=self._publish_target
        ).pack(side="left", padx=4)

        # ══════════════════════════════════════════════════════════════════
        #  GRASP TAB
        # ══════════════════════════════════════════════════════════════════
        self._build_grasp_tab(grasp_tab)

        # Pre-build gains panels for all controllers (avoids destroy/recreate on switch)
        self._prebuild_gains_panels()
        self._show_gains_panel("demo_joint_controller")
        self._update_target_inputs_state("demo_joint_controller")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._gui_ready.set()
        self._schedule_refresh()
        self.root.mainloop()

    # ---- Grasp tab builder -----------------------------------------------------

    def _build_grasp_tab(self, parent: tk.Frame):
        hdr_font = tkfont.Font(family="Segoe UI", size=9, weight="bold")
        mono_font = ("Courier New", 9, "bold")

        # ── Grasp State Monitor ─────────────────────────────────────────
        state_frame = ttk.LabelFrame(parent, text="Grasp State", padding=4)
        state_frame.pack(fill="x", padx=8, pady=(4, 2))

        # Top row: big indicator + aggregate values
        top_row = tk.Frame(state_frame, bg="#1e1e2e")
        top_row.pack(fill="x", pady=(0, 4))

        self._grasp_detected_label = tk.Label(
            top_row,
            text="  NO GRASP  ",
            bg="#585b70",
            fg="#cdd6f4",
            font=("Segoe UI", 11, "bold"),
            padx=8,
            pady=4,
        )
        self._grasp_detected_label.pack(side="left", padx=(4, 12))

        agg_frame = tk.Frame(top_row, bg="#1e1e2e")
        agg_frame.pack(side="left", fill="x")

        agg_items = [
            ("Active Contacts:", "_grasp_active_label", "0"),
            ("Max Force:", "_grasp_maxforce_label", "0.00 N"),
            ("Threshold:", "_grasp_threshold_label", "1.00 N"),
            ("Min Fingers:", "_grasp_minfinger_label", "2"),
        ]
        for i, (text, attr, default) in enumerate(agg_items):
            tk.Label(
                agg_frame, text=text, bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8), anchor="e"
            ).grid(row=i // 2, column=(i % 2) * 2, padx=(8, 2), pady=1, sticky="e")
            lbl = tk.Label(
                agg_frame,
                text=default,
                bg="#313244",
                fg="#f9e2af",
                font=mono_font,
                width=10,
                anchor="center",
            )
            lbl.grid(row=i // 2, column=(i % 2) * 2 + 1, padx=(0, 8), pady=1)
            setattr(self, attr, lbl)

        # Per-fingertip table
        ttk.Separator(state_frame, orient="horizontal").pack(fill="x", pady=2)

        ft_frame = tk.Frame(state_frame, bg="#1e1e2e")
        ft_frame.pack(fill="x", padx=4)

        # Header row
        for c, txt in enumerate(["Finger", "Force (N)", "Contact", "Valid"]):
            tk.Label(
                ft_frame,
                text=txt,
                bg="#1e1e2e",
                fg="#89b4fa",
                font=hdr_font,
                width=10,
                anchor="center",
            ).grid(row=0, column=c, padx=2, pady=(0, 2))

        self._ft_force_labels: list[tk.Label] = []
        self._ft_contact_labels: list[tk.Label] = []
        self._ft_valid_labels: list[tk.Label] = []

        for i, name in enumerate(FINGERTIP_NAMES):
            tk.Label(
                ft_frame,
                text=name,
                bg="#1e1e2e",
                fg="#f9e2af",
                font=hdr_font,
                width=10,
                anchor="center",
            ).grid(row=i + 1, column=0, padx=2, pady=1)

            fl = tk.Label(
                ft_frame,
                text="0.00 N",
                bg="#313244",
                fg="#cdd6f4",
                font=mono_font,
                width=10,
                anchor="center",
            )
            fl.grid(row=i + 1, column=1, padx=2, pady=1)
            self._ft_force_labels.append(fl)

            cl = tk.Label(
                ft_frame,
                text="0.00",
                bg="#585b70",
                fg="#cdd6f4",
                font=mono_font,
                width=10,
                anchor="center",
            )
            cl.grid(row=i + 1, column=2, padx=2, pady=1)
            self._ft_contact_labels.append(cl)

            vl = tk.Label(
                ft_frame,
                text="--",
                bg="#1e1e2e",
                fg="#f38ba8",
                font=mono_font,
                width=10,
                anchor="center",
            )
            vl.grid(row=i + 1, column=3, padx=2, pady=1)
            self._ft_valid_labels.append(vl)

        # ── Grasp Detection Params ──────────────────────────────────────
        # Swapped per active controller by `_show_gains_panel`. The
        # actual per-controller widgets (contact_thresh / force_thresh /
        # min_fingertips) live inside `self._gains_grasp_inner`; the
        # flat GAIN_DEFS wire order is still honoured because widgets
        # are appended to `self._gain_entries` during `_prebuild_gains_panels`.
        grasp_gains_frame = ttk.LabelFrame(parent, text="Grasp Detection", padding=4)
        grasp_gains_frame.pack(fill="x", padx=8, pady=(2, 2))
        self._gains_grasp_inner = tk.Frame(grasp_gains_frame, bg="#1e1e2e")
        self._gains_grasp_inner.pack(fill="x")

        # ── Force-PI Grasp Controller ──────────────────────────────────
        fp_frame = ttk.LabelFrame(parent, text="Force-PI Grasp Controller", padding=4)
        fp_frame.pack(fill="x", padx=8, pady=(2, 2))

        # Phase + target force row
        fp_top = tk.Frame(fp_frame, bg="#1e1e2e")
        fp_top.pack(fill="x", pady=(0, 4))

        tk.Label(fp_top, text="Phase:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)).pack(
            side="left", padx=(4, 2)
        )
        self._fp_phase_label = tk.Label(
            fp_top,
            text="  IDLE  ",
            bg="#585b70",
            fg="#cdd6f4",
            font=("Segoe UI", 10, "bold"),
            padx=6,
            pady=2,
        )
        self._fp_phase_label.pack(side="left", padx=(0, 12))

        tk.Label(fp_top, text="Target:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)).pack(
            side="left", padx=(0, 2)
        )
        self._fp_target_force_display = tk.Label(
            fp_top,
            text="0.00 N",
            bg="#313244",
            fg="#f9e2af",
            font=mono_font,
            width=8,
            anchor="center",
        )
        self._fp_target_force_display.pack(side="left", padx=(0, 4))

        # Per-finger table (3 fingers)
        ttk.Separator(fp_frame, orient="horizontal").pack(fill="x", pady=2)
        fp_tbl = tk.Frame(fp_frame, bg="#1e1e2e")
        fp_tbl.pack(fill="x", padx=4)

        for c, txt in enumerate(["Finger", "s", "Filtered F", "F Error", "F Desired"]):
            tk.Label(
                fp_tbl,
                text=txt,
                bg="#1e1e2e",
                fg="#89b4fa",
                font=hdr_font,
                width=10,
                anchor="center",
            ).grid(row=0, column=c, padx=2, pady=(0, 2))

        self._fp_s_labels: list[tk.Label] = []
        self._fp_filt_labels: list[tk.Label] = []
        self._fp_err_labels: list[tk.Label] = []
        self._fp_desired_labels: list[tk.Label] = []

        for i, name in enumerate(FORCE_PI_FINGER_NAMES):
            tk.Label(
                fp_tbl,
                text=name,
                bg="#1e1e2e",
                fg="#f9e2af",
                font=hdr_font,
                width=10,
                anchor="center",
            ).grid(row=i + 1, column=0, padx=2, pady=1)

            sl = tk.Label(
                fp_tbl,
                text="0.000",
                bg="#313244",
                fg="#cdd6f4",
                font=mono_font,
                width=10,
                anchor="center",
            )
            sl.grid(row=i + 1, column=1, padx=2, pady=1)
            self._fp_s_labels.append(sl)

            fl = tk.Label(
                fp_tbl,
                text="0.00 N",
                bg="#313244",
                fg="#cdd6f4",
                font=mono_font,
                width=10,
                anchor="center",
            )
            fl.grid(row=i + 1, column=2, padx=2, pady=1)
            self._fp_filt_labels.append(fl)

            el = tk.Label(
                fp_tbl,
                text="+0.00 N",
                bg="#313244",
                fg="#a6e3a1",
                font=mono_font,
                width=10,
                anchor="center",
            )
            el.grid(row=i + 1, column=3, padx=2, pady=1)
            self._fp_err_labels.append(el)

            dl = tk.Label(
                fp_tbl,
                text="0.00 N",
                bg="#313244",
                fg="#cdd6f4",
                font=mono_font,
                width=10,
                anchor="center",
            )
            dl.grid(row=i + 1, column=4, padx=2, pady=1)
            self._fp_desired_labels.append(dl)

        # Command row: Grasp/Release buttons + target force input
        ttk.Separator(fp_frame, orient="horizontal").pack(fill="x", pady=2)
        cmd_frame = tk.Frame(fp_frame, bg="#1e1e2e")
        cmd_frame.pack(fill="x", padx=4, pady=(2, 0))

        ttk.Button(
            cmd_frame,
            text="▶ Grasp",
            style="Send.TButton",
            command=lambda: self._send_grasp_command(1),
        ).pack(side="left", padx=4)
        ttk.Button(cmd_frame, text="■ Release", command=lambda: self._send_grasp_command(2)).pack(
            side="left", padx=4
        )

        tk.Label(
            cmd_frame, text="Target Force:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)
        ).pack(side="left", padx=(16, 2))
        self._grasp_target_force_entry = ttk.Entry(cmd_frame, width=6, justify="center")
        self._grasp_target_force_entry.insert(0, "2.0")
        self._grasp_target_force_entry.pack(side="left", padx=2)
        tk.Label(cmd_frame, text="N", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)).pack(
            side="left"
        )

        # Hint: Grasp/Release buttons only function when the active controller
        # is loaded with grasp_controller_type: "force_pi" in its YAML.
        # In "contact_stop" mode the buttons are silently ignored by the
        # controller (a throttled WARN is emitted on /rosout).
        tk.Label(
            cmd_frame,
            text='(requires grasp_controller_type: "force_pi" in YAML)',
            bg="#1e1e2e",
            fg="#f9e2af",
            font=("Segoe UI", 7, "italic"),
        ).pack(side="left", padx=(12, 0))

        # ── Sensor Calibration ─────────────────────────────────────────
        # Triggers recalibration of hand sensors via /hand/calibration/command.
        # Status updates arrive via /hand/calibration/status subscription.
        calib_frame = ttk.LabelFrame(parent, text="Sensor Calibration", padding=4)
        calib_frame.pack(fill="x", padx=8, pady=2)

        for entry in SENSOR_CALIBRATIONS:
            stype = entry["sensor_type"]
            row = tk.Frame(calib_frame, bg="#1e1e2e")
            row.pack(fill="x", pady=1)

            tk.Label(
                row,
                text=entry["label"],
                bg="#1e1e2e",
                fg="#cdd6f4",
                font=("Segoe UI", 9, "bold"),
                width=16,
                anchor="w",
            ).pack(side="left", padx=(2, 4))

            ttk.Button(
                row,
                text="Calibrate",
                style="Send.TButton",
                command=lambda s=stype: self._send_calibration(s, CalibrationCommand.ACTION_START),
            ).pack(side="left", padx=2)
            ttk.Button(
                row,
                text="Abort",
                command=lambda s=stype: self._send_calibration(s, CalibrationCommand.ACTION_ABORT),
            ).pack(side="left", padx=2)

            status_var = tk.StringVar(value="IDLE")
            status_lbl = tk.Label(
                row,
                textvariable=status_var,
                bg="#1e1e2e",
                fg="#9399b2",
                font=("Segoe UI", 9),
                width=22,
                anchor="w",
            )
            status_lbl.pack(side="left", padx=(8, 2))
            self._calib_status_vars[stype] = status_var
            self._calib_status_labels[stype] = status_lbl

            tk.Label(
                row, text=entry["hint"], bg="#1e1e2e", fg="#585b70", font=("Segoe UI", 7, "italic")
            ).pack(side="left", padx=(4, 0))

        # ── Hand Posture Presets ─────────────────────────────────────────
        # Packed with expand=False so the LabelFrame shrinks to its
        # natural content height (≈1/2 of the previous expand-to-fill
        # size). The reclaimed space stays at the bottom of the tab.
        preset_frame = ttk.LabelFrame(parent, text="Hand Posture Presets", padding=4)
        preset_frame.pack(fill="x", expand=False, padx=8, pady=(2, 4))

        # Preset Treeview — fixed to a 7-row height; the vertical
        # scrollbar on the right takes over once more presets exist.
        tree_frame = tk.Frame(preset_frame, bg="#1e1e2e")
        tree_frame.pack(fill="x", expand=False, padx=4, pady=(0, 4))

        columns = ("type", "controller", "grasp_time", "robot_target", "positions")
        self._preset_tree = ttk.Treeview(
            tree_frame, columns=columns, show="tree headings", height=7, selectmode="browse"
        )
        self._preset_tree.heading("#0", text="Name", anchor="w")
        self._preset_tree.heading("type", text="Type", anchor="center")
        self._preset_tree.heading("controller", text="Controller", anchor="center")
        self._preset_tree.heading("grasp_time", text="Time (s)", anchor="center")
        self._preset_tree.heading("robot_target", text="Robot Target", anchor="w")
        self._preset_tree.heading("positions", text="Hand (deg)", anchor="w")
        self._preset_tree.column("#0", width=100, minwidth=70)
        self._preset_tree.column("type", width=50, minwidth=40, anchor="center")
        self._preset_tree.column("controller", width=80, minwidth=60, anchor="center")
        self._preset_tree.column("grasp_time", width=55, minwidth=40, anchor="center")
        self._preset_tree.column("robot_target", width=220, minwidth=120)
        self._preset_tree.column("positions", width=280, minwidth=150)

        tree_scroll = ttk.Scrollbar(tree_frame, orient="vertical", command=self._preset_tree.yview)
        self._preset_tree.configure(yscrollcommand=tree_scroll.set)
        self._preset_tree.pack(side="left", fill="x", expand=True)
        tree_scroll.pack(side="right", fill="y")

        self._refresh_preset_tree()

        # Input row for new preset
        input_frame = tk.Frame(preset_frame, bg="#1e1e2e")
        input_frame.pack(fill="x", padx=4, pady=2)

        tk.Label(input_frame, text="Name:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)).pack(
            side="left", padx=(0, 2)
        )
        self._preset_name_entry = ttk.Entry(input_frame, width=14, justify="center")
        self._preset_name_entry.pack(side="left", padx=2)

        tk.Label(input_frame, text="Type:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)).pack(
            side="left", padx=(8, 2)
        )
        self._preset_type_var = tk.StringVar(value="open")
        self._preset_type_combo = ttk.Combobox(
            input_frame,
            textvariable=self._preset_type_var,
            values=["open", "close"],
            width=6,
            state="readonly",
        )
        self._preset_type_combo.pack(side="left", padx=2)

        tk.Label(
            input_frame, text="Grasp Time (s):", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)
        ).pack(side="left", padx=(8, 2))
        self._preset_time_entry = ttk.Entry(input_frame, width=6, justify="center")
        self._preset_time_entry.insert(0, "1.0")
        self._preset_time_entry.pack(side="left", padx=2)

        # Robot inclusion row
        robot_input_frame = tk.Frame(preset_frame, bg="#1e1e2e")
        robot_input_frame.pack(fill="x", padx=4, pady=2)

        self._preset_include_robot_var = tk.BooleanVar(value=False)
        self._preset_include_robot_cb = ttk.Checkbutton(
            robot_input_frame,
            text="Include Robot",
            variable=self._preset_include_robot_var,
            command=self._on_preset_include_robot_toggle,
        )
        self._preset_include_robot_cb.pack(side="left", padx=(0, 8))

        tk.Label(
            robot_input_frame, text="Controller:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)
        ).pack(side="left", padx=(0, 2))
        self._preset_ctrl_var = tk.StringVar(value="demo_joint_controller")
        self._preset_ctrl_combo = ttk.Combobox(
            robot_input_frame,
            textvariable=self._preset_ctrl_var,
            values=list(CONTROLLER_TYPES.keys()),
            width=22,
            state="disabled",
        )
        self._preset_ctrl_combo.pack(side="left", padx=2)
        self._preset_ctrl_combo.bind("<<ComboboxSelected>>", self._on_preset_ctrl_combo_change)

        tk.Label(
            robot_input_frame, text="Goal:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)
        ).pack(side="left", padx=(8, 2))
        self._preset_goal_type_var = tk.StringVar(value="joint")
        self._preset_goal_type_combo = ttk.Combobox(
            robot_input_frame,
            textvariable=self._preset_goal_type_var,
            values=["joint", "task"],
            width=6,
            state="disabled",
        )
        self._preset_goal_type_combo.pack(side="left", padx=2)

        # Action buttons
        action_frame = tk.Frame(preset_frame, bg="#1e1e2e")
        action_frame.pack(fill="x", padx=4, pady=(2, 0))

        ttk.Button(
            action_frame, text="Send Preset", style="Send.TButton", command=self._send_preset
        ).pack(side="left", padx=4)
        ttk.Button(
            action_frame,
            text="Save Current Pos",
            style="Preset.TButton",
            command=self._save_current_as_preset,
        ).pack(side="left", padx=4)
        ttk.Button(
            action_frame,
            text="Save Target Pos",
            style="Preset.TButton",
            command=self._save_target_as_preset,
        ).pack(side="left", padx=4)
        ttk.Button(action_frame, text="Delete", command=self._delete_preset).pack(
            side="left", padx=4
        )

        # Preset file path display
        tk.Label(
            preset_frame,
            text=f"Preset file: {self._preset_path}",
            bg="#1e1e2e",
            fg="#585b70",
            font=("Segoe UI", 7, "italic"),
        ).pack(anchor="w", padx=4, pady=(4, 0))

    def _on_preset_include_robot_toggle(self):
        state = "readonly" if self._preset_include_robot_var.get() else "disabled"
        self._preset_ctrl_combo.configure(state=state)
        self._preset_goal_type_combo.configure(state=state)
        if self._preset_include_robot_var.get():
            # Sync with currently selected controller
            ctrl = self.selected_ctrl.get()
            self._preset_ctrl_var.set(ctrl)
            self._preset_goal_type_var.set("joint" if JOINT_SPACE.get(ctrl, True) else "task")

    def _on_preset_ctrl_combo_change(self, _event=None):
        ctrl = self._preset_ctrl_var.get()
        self._preset_goal_type_var.set("joint" if JOINT_SPACE.get(ctrl, True) else "task")

    def _refresh_preset_tree(self):
        for item in self._preset_tree.get_children():
            self._preset_tree.delete(item)
        for name, data in self._presets.items():
            pos_str = ", ".join(f"{v:.1f}" for v in data.get("positions_deg", []))
            ctrl = data.get("controller")
            ctrl_short = ctrl.replace("demo_", "").replace("_controller", "") if ctrl else "—"
            robot_tgt = data.get("robot_target")
            if robot_tgt and len(robot_tgt) == self._shape.arm_dof:
                goal = data.get("robot_goal_type", "joint")
                robot_str = f"[{goal}] " + ", ".join(f"{v:.2f}" for v in robot_tgt)
            else:
                robot_str = "—"
            self._preset_tree.insert(
                "",
                "end",
                iid=name,
                text=name,
                values=(
                    data.get("type", "open"),
                    ctrl_short,
                    f"{data.get('grasp_time', 1.0):.1f}",
                    robot_str,
                    pos_str,
                ),
            )

    # ---- Gains UI helpers ----------------------------------------------------

    def _prebuild_gains_panels(self):
        """Pre-build gains UI for all controllers to avoid destroy/recreate on switch.

        Gains are placed into functional group boxes ("CLIK Gains",
        "Arm Trajectory", "Hand Trajectory", "Grasp Detection"). The
        widget order appended to ``self._gain_entries`` still matches
        the flat GAIN_DEFS order so ``_collect_gain_values`` /
        ``_fill_gains_from_data`` continue to produce the correct
        on-the-wire layout even when groups are interleaved. Groups
        listed in ``GAIN_GROUP_PARENT_GRASP`` are routed to the Grasp
        tab (``self._gains_grasp_inner``) instead of the Control tab
        Gains panel, and their entries get no applied mirror.
        """
        self._gains_panels = {}
        self._active_gains_ctrl = None

        # Default max scalar inputs per row inside a group box.
        DEFAULT_SCALARS_PER_ROW = 2

        for ctrl_idx in CONTROLLER_TYPES:
            panel_frame = tk.Frame(self._gains_inner, bg="#1e1e2e")
            applied_frame = tk.Frame(self._gains_applied_inner, bg="#1e1e2e")
            grasp_frame = tk.Frame(self._gains_grasp_inner, bg="#1e1e2e")

            entries: list = []
            is_bool_list: list[bool] = []
            applied_labels: list = []

            defs = GAIN_DEFS.get(ctrl_idx, [])
            row_names_map = GAIN_ROW_NAMES.get(ctrl_idx, {})
            if not defs:
                self._gains_panels[ctrl_idx] = {
                    "frame": panel_frame,
                    "applied_frame": applied_frame,
                    "grasp_frame": grasp_frame,
                    "entries": entries,
                    "is_bool": is_bool_list,
                    "applied_labels": applied_labels,
                }
                continue

            # Partition groups by destination parent.
            control_groups: list[str] = []
            grasp_groups: list[str] = []
            seen = set()
            for _, _, _, _, g in defs:
                if g in seen:
                    continue
                seen.add(g)
                if g in GAIN_GROUP_PARENT_GRASP:
                    grasp_groups.append(g)
                else:
                    control_groups.append(g)

            # Resolve the Control-tab row layout: honor GAIN_GROUP_LAYOUT
            # for listed groups and append any leftover control groups
            # as a fallback row. Grasp-destined groups never enter the
            # control-tab layout.
            layout = GAIN_GROUP_LAYOUT.get(ctrl_idx)
            if layout is not None:
                laid_out = {g for row in layout for g in row}
                missing = [g for g in control_groups if g not in laid_out]
                row_layout: list[list[str]] = [
                    [g for g in row if g not in GAIN_GROUP_PARENT_GRASP] for row in layout
                ]
                row_layout = [r for r in row_layout if r]
                if missing:
                    row_layout.append(missing)
            else:
                row_layout = [[g] for g in control_groups]

            # Grasp-tab row layout: each grasp group occupies a row.
            grasp_row_layout: list[list[str]] = [[g] for g in grasp_groups]

            # One LabelFrame per group (editable panel + optional
            # applied mirror). Inside each group the array_area sits to
            # the left and the scalar_area to the right, so array-form
            # gains (kp_*) and related scalars (damping, null_kp, …)
            # stay on one row.
            def _make_group_box(parent_row: tk.Widget, title: str) -> dict:
                box = ttk.LabelFrame(parent_row, text=title, padding=4)
                box.pack(side="left", anchor="nw", padx=(0, 4))
                array_area = tk.Frame(box, bg="#1e1e2e")
                array_area.pack(side="left", anchor="nw")
                scalar_area = tk.Frame(box, bg="#1e1e2e")
                scalar_area.pack(side="left", anchor="nw", padx=(12, 0))
                return {
                    "array_area": array_area,
                    "array_row": 0,
                    "scalar_area": scalar_area,
                    "scalar_row_frame": None,
                    "scalar_in_row": 0,
                    "scalars_per_row": GROUP_SCALARS_PER_ROW.get(title, DEFAULT_SCALARS_PER_ROW),
                }

            group_boxes: dict[str, dict] = {}
            group_boxes_a: dict[str, dict] = {}
            for row_groups in row_layout:
                row_frame = tk.Frame(panel_frame, bg="#1e1e2e")
                row_frame.pack(fill="x", anchor="w", pady=(0, 3))
                row_frame_a = tk.Frame(applied_frame, bg="#1e1e2e")
                row_frame_a.pack(fill="x", anchor="w", pady=(0, 3))
                for g in row_groups:
                    group_boxes[g] = _make_group_box(row_frame, g)
                    group_boxes_a[g] = _make_group_box(row_frame_a, g)

            # Grasp-tab rows: editable only, no applied mirror.
            for row_groups in grasp_row_layout:
                row_frame_g = tk.Frame(grasp_frame, bg="#1e1e2e")
                row_frame_g.pack(fill="x", anchor="w", pady=(0, 3))
                for g in row_groups:
                    group_boxes[g] = _make_group_box(row_frame_g, g)

            def _place_array(
                box: dict, label: str, size: int, defaults: list, editable: bool
            ) -> list:
                """Place an array row inside the group's array_area."""
                area = box["array_area"]
                row = box["array_row"]
                names = row_names_map.get(label)
                name_fg = "#f9e2af" if editable else "#585b70"
                label_fg = "#cdd6f4" if editable else "#585b70"
                if names:
                    for j, name in enumerate(names):
                        tk.Label(
                            area,
                            text=name,
                            bg="#1e1e2e",
                            fg=name_fg,
                            font=("Segoe UI", 7),
                            width=7,
                            anchor="center",
                        ).grid(row=row, column=j + 1, padx=2)
                    row += 1
                tk.Label(
                    area,
                    text=label + ":",
                    bg="#1e1e2e",
                    fg=label_fg,
                    font=("Segoe UI", 8),
                    anchor="e",
                ).grid(row=row, column=0, sticky="e", padx=(8, 4), pady=1)
                widgets: list = []
                for j in range(size):
                    if editable:
                        ent = ttk.Entry(area, width=7, justify="center")
                        ent.insert(0, str(defaults[j]))
                        ent.grid(row=row, column=j + 1, padx=2, pady=1)
                        widgets.append(ent)
                    else:
                        lbl = tk.Label(
                            area,
                            text="---",
                            bg="#1e1e2e",
                            fg="#585b70",
                            font=("Courier New", 8),
                            width=7,
                            anchor="center",
                        )
                        lbl.grid(row=row, column=j + 1, padx=2, pady=1)
                        widgets.append(lbl)
                box["array_row"] = row + 1
                return widgets

            def _next_scalar_slot(box: dict) -> tk.Frame:
                """Return a new Frame slot within the group's scalar_area,
                wrapping to a new row after the group's scalars_per_row."""
                if (
                    box["scalar_row_frame"] is None
                    or box["scalar_in_row"] >= box["scalars_per_row"]
                ):
                    box["scalar_row_frame"] = tk.Frame(box["scalar_area"], bg="#1e1e2e")
                    box["scalar_row_frame"].pack(fill="x", pady=1)
                    box["scalar_in_row"] = 0
                frm = tk.Frame(box["scalar_row_frame"], bg="#1e1e2e")
                frm.pack(side="left", padx=8)
                box["scalar_in_row"] += 1
                return frm

            for label, size, defaults, is_bool, group in defs:
                in_grasp_tab = group in GAIN_GROUP_PARENT_GRASP

                if size > 1:
                    entries.append(
                        _place_array(group_boxes[group], label, size, defaults, editable=True)
                    )
                    is_bool_list.append(False)
                    if in_grasp_tab:
                        applied_labels.append([])
                    else:
                        applied_labels.append(
                            _place_array(
                                group_boxes_a[group], label, size, defaults, editable=False
                            )
                        )
                    continue

                # ---- Editable scalar ----
                frm = _next_scalar_slot(group_boxes[group])
                tk.Label(frm, text=label, bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 8)).pack(
                    anchor="w"
                )
                if is_bool:
                    var = tk.IntVar(value=int(defaults[0]))
                    ttk.Checkbutton(frm, variable=var).pack(anchor="w")
                    entries.append([var])
                else:
                    ent = ttk.Entry(frm, width=8, justify="center")
                    ent.insert(0, str(defaults[0]))
                    ent.pack()
                    entries.append([ent])
                is_bool_list.append(is_bool)

                if in_grasp_tab:
                    # Grasp-tab groups intentionally have no applied
                    # mirror; empty slot keeps list alignment for
                    # `_update_applied_display`.
                    applied_labels.append([])
                    continue

                # ---- Applied scalar mirror ----
                frm_a = _next_scalar_slot(group_boxes_a[group])
                tk.Label(frm_a, text=label, bg="#1e1e2e", fg="#585b70", font=("Segoe UI", 8)).pack(
                    anchor="w"
                )
                lbl = tk.Label(
                    frm_a, text="---", bg="#1e1e2e", fg="#585b70", font=("Courier New", 8)
                )
                lbl.pack(anchor="w")
                applied_labels.append([lbl])

            self._gains_panels[ctrl_idx] = {
                "frame": panel_frame,
                "applied_frame": applied_frame,
                "grasp_frame": grasp_frame,
                "entries": entries,
                "is_bool": is_bool_list,
                "applied_labels": applied_labels,
            }

    def _show_gains_panel(self, ctrl_idx: str):
        """Show the pre-built gains panel for the given controller (instant swap)."""
        if self._active_gains_ctrl == ctrl_idx:
            return
        if self._active_gains_ctrl is not None:
            old = self._gains_panels[self._active_gains_ctrl]
            old["frame"].pack_forget()
            old["applied_frame"].pack_forget()
            old["grasp_frame"].pack_forget()
        panel = self._gains_panels[ctrl_idx]
        panel["frame"].pack(fill="x")
        panel["applied_frame"].pack(fill="x", padx=2, pady=(0, 2))
        panel["grasp_frame"].pack(fill="x")
        self._gain_entries = panel["entries"]
        self._gain_is_bool = panel["is_bool"]
        self._applied_label_widgets = panel["applied_labels"]
        self._active_gains_ctrl = ctrl_idx

    def _update_applied_display(self):
        for widgets, applied_labels, is_bool in zip(
            self._gain_entries, self._applied_label_widgets, self._gain_is_bool, strict=False
        ):
            for w, lbl in zip(widgets, applied_labels, strict=False):
                raw = w.get()
                if is_bool:
                    val = int(raw)
                    lbl.config(text="ON" if val else "OFF", fg="#a6e3a1" if val else "#f38ba8")
                else:
                    try:
                        lbl.config(text=f"{float(raw):.4f}", fg="#a6e3a1")
                    except ValueError:
                        pass

    # ---- Button handlers -----------------------------------------------------

    def _update_target_inputs_state(self, ctrl_idx: int):
        is_joint = JOINT_SPACE.get(ctrl_idx, True)
        joint_state = "normal" if is_joint else "disabled"
        task_state = "disabled" if is_joint else "normal"

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
        self._show_gains_panel(idx)
        self._update_target_inputs_state(idx)

    def _on_switch_controller(self):
        idx = self.selected_ctrl.get()

        # Async send via /rtc_cm/switch_controller. UI updates happen
        # immediately (the switch-back flag /rtc_cm/active_controller_name
        # subscription drives the controller-owned topic rebinding); the
        # done-callback only logs success/failure.
        if not self.switch_controller_client.service_is_ready():
            self.get_logger().warn("/rtc_cm/switch_controller not ready — request dropped")
            self._ctrl_status.set(f"(srv unavailable) Active: {CONTROLLER_TYPES[idx]}")
            return

        req = SwitchController.Request()
        req.activate_controllers = [idx]
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = 1
        req.timeout.nanosec = 0
        future = self.switch_controller_client.call_async(req)

        def _on_switch_done(fut):
            try:
                resp = fut.result()
            except Exception as exc:  # pragma: no cover — rclpy executor error
                self.get_logger().error(f"switch_controller srv exception: {exc}")
                return
            if resp.ok:
                self.get_logger().info(f"Switched to {CONTROLLER_TYPES[idx]}: {resp.message}")
            else:
                self.get_logger().error(f"switch_controller rejected: {resp.message}")

        future.add_done_callback(_on_switch_done)

        self._ctrl_status.set(f"Active: {CONTROLLER_TYPES[idx]}")
        self._show_gains_panel(idx)
        self._update_target_inputs_state(idx)

        _task_status_names = ["X (m)", "Y (m)", "Z (m)", "Roll", "Pitch", "Yaw"]
        if JOINT_SPACE.get(idx, True):
            self._set_joint_target_entries(self.current_positions)
            for i, name_lbl in enumerate(self._status_labels_names):
                name_lbl.config(text=f"J{i + 1}:")
        else:
            self._set_task_target_entries(self.current_task_positions)
            for i, name_lbl in enumerate(self._status_labels_names):
                name_lbl.config(text=f"{_task_status_names[i]}:")

        # Hand target: initialize from current positions
        self._set_hand_target_entries(self.current_hand_positions)

    def _request_load_gains(self):
        ctrl = self.selected_ctrl.get()
        dispatch = GAIN_PARAM_DISPATCH.get(ctrl)
        if dispatch is None:
            self.get_logger().error(f"No parameter dispatch for {ctrl}")
            return

        # Read all parameters (read-only caps included so the user can see
        # the YAML-set values; the result fills widgets but they remain
        # disabled at apply time).
        param_names = [p[0] for p in dispatch.values()]
        client = self._get_param_client(ctrl)
        if not client.wait_for_services(timeout_sec=1.0):
            self.get_logger().error(f"parameter services for /{ctrl} unavailable")
            return

        future = client.get_parameters(param_names)

        def _on_get_done(fut):
            try:
                resp = fut.result()
            except Exception as exc:
                self.get_logger().error(f"get_parameters exception: {exc}")
                return
            # resp is rcl_interfaces.srv.GetParameters.Response → values list.
            flat: list[float] = []
            for entry, p_value in zip(dispatch.items(), resp.values, strict=False):
                # entry = (gain_def_name, (param_name, value_builder)). The
                # GUI widget order matches GAIN_DEFS exactly, so we flatten
                # in dispatch-iteration order.
                _, (_, builder) = entry
                # Reverse-extract the typed scalar/array. ParameterValue
                # carries 8 type slots (NOT_SET, BOOL, INTEGER, DOUBLE,
                # STRING, BYTE_ARRAY, BOOL_ARRAY, INTEGER_ARRAY,
                # DOUBLE_ARRAY, STRING_ARRAY).
                if builder is _set_double_array:
                    flat.extend(list(p_value.double_array_value))
                elif builder is _set_double:
                    flat.append(p_value.double_value)
                elif builder is _set_int:
                    flat.append(float(p_value.integer_value))
                elif builder is _set_bool:
                    flat.append(1.0 if p_value.bool_value else 0.0)
                elif builder is _read_only:
                    flat.append(p_value.double_value)
                else:
                    flat.append(0.0)
            self.root.after(0, self._fill_gains_from_data, flat)
            self.get_logger().info(f"Loaded {len(param_names)} parameters from /{ctrl}")

        future.add_done_callback(_on_get_done)
        self.get_logger().info(f"Requesting current parameters from /{ctrl}")

    def _fill_gains_from_data(self, data: list[float]):
        idx = 0
        for widgets, is_bool in zip(self._gain_entries, self._gain_is_bool, strict=False):
            for w in widgets:
                if idx >= len(data):
                    break
                if is_bool:
                    w.set(1 if data[idx] > 0.5 else 0)
                else:
                    w.delete(0, tk.END)
                    w.insert(0, f"{data[idx]:.4f}")
                idx += 1
            else:
                continue
            break

        ctrl_name = CONTROLLER_TYPES[self.selected_ctrl.get()]

        # Sync grasp target force entry from loaded gains.
        # Layout tail (see CLAUDE.md): [..., grasp_command, grasp_target_force].
        #   demo_joint_controller: target_force is index 8 (9 values total)
        #   demo_task_controller : target_force is index 20 (21 values total)
        tf_idx_map = {
            "demo_joint_controller": 8,
            "demo_task_controller": 20,
        }
        tf_idx = tf_idx_map.get(ctrl_name)
        if (
            tf_idx is not None
            and tf_idx < len(data)
            and hasattr(self, "_grasp_target_force_entry")
        ):
            self._grasp_target_force_entry.delete(0, tk.END)
            self._grasp_target_force_entry.insert(0, f"{data[tf_idx]:.2f}")

        self.get_logger().info(f"Loaded gains for {ctrl_name}: {[f'{v:.4f}' for v in data]}")

    def _collect_gain_values(self) -> list[float] | None:
        """Collect current gain values from UI widgets. Returns None on error."""
        values: list[float] = []
        for widgets, is_bool in zip(self._gain_entries, self._gain_is_bool, strict=False):
            for w in widgets:
                try:
                    values.append(float(w.get()))
                except ValueError:
                    self.get_logger().error("Invalid gain value — check all fields.")
                    return None
        return values

    def _publish_gains(self):
        ctrl = self.selected_ctrl.get()
        dispatch = GAIN_PARAM_DISPATCH.get(ctrl)
        if dispatch is None:
            self.get_logger().error(f"No parameter dispatch for {ctrl}")
            return

        # GAIN_DEFS and dispatch are both keyed by entry name; iterate the
        # widget rows in declared order and consume `size` widgets each.
        params: list[Parameter] = []
        widget_iter = iter(zip(self._gain_entries, self._gain_is_bool, strict=False))
        for entry in GAIN_DEFS[ctrl]:
            entry_name, size, _defaults, is_bool, _grp = entry
            try:
                widgets, _is_bool = next(widget_iter)
            except StopIteration:
                self.get_logger().error("widget/gain_def out of sync")
                return
            try:
                values = [float(w.get()) for w in widgets]
            except ValueError:
                self.get_logger().error(f"Invalid value in '{entry_name}' — fix and retry")
                return
            param_name, builder = dispatch[entry_name]
            built = builder(values)
            if built is None:
                # Read-only cap — skip silently (controller would reject).
                continue
            ptype, pvalue = built
            params.append(Parameter(name=param_name, type_=ptype, value=pvalue))

        if not params:
            self.get_logger().warn("Nothing to apply (all entries read-only?)")
            return

        client = self._get_param_client(ctrl)
        if not client.wait_for_services(timeout_sec=1.0):
            self.get_logger().error(f"parameter services for /{ctrl} unavailable")
            return

        future = client.set_parameters_atomically(params)
        ctrl_label = CONTROLLER_TYPES[ctrl]

        def _on_set_done(fut):
            try:
                resp = fut.result()
            except Exception as exc:
                self.get_logger().error(f"set_parameters exception: {exc}")
                return
            if resp.result.successful:
                self.get_logger().info(f"Applied {len(params)} gains for {ctrl_label}")
            else:
                self.get_logger().error(
                    f"set_parameters_atomically rejected: {resp.result.reason}"
                )

        future.add_done_callback(_on_set_done)

        self.root.after(0, self._update_applied_display)

    def _send_grasp_command(self, cmd: int):
        """Send a one-shot Force-PI grasp command via the active controller's
        ~/grasp_command srv. cmd: 1=grasp, 2=release.
        """
        ctrl = self.selected_ctrl.get()
        try:
            target_force = float(self._grasp_target_force_entry.get())
        except (ValueError, AttributeError):
            target_force = 2.0

        client = self._get_grasp_client(ctrl)
        if not client.service_is_ready():
            self.get_logger().error(f"/{ctrl}/grasp_command srv not ready")
            return

        req = GraspCommand.Request()
        req.command = int(cmd)
        req.target_force = target_force
        future = client.call_async(req)

        cmd_name = "Grasp" if cmd == 1 else "Release"

        def _on_grasp_done(fut):
            try:
                resp = fut.result()
            except Exception as exc:
                self.get_logger().error(f"grasp_command exception: {exc}")
                return
            if resp.ok:
                self.get_logger().info(
                    f"{cmd_name} accepted (target_force={target_force:.2f} N): {resp.message}"
                )
            else:
                self.get_logger().error(f"{cmd_name} rejected: {resp.message}")

        future.add_done_callback(_on_grasp_done)
        self.get_logger().info(f"Sent {cmd_name} command (target_force={target_force:.2f} N)")

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
        robot_msg.joint_names = list(self._shape.arm_joint_names)
        try:
            if is_joint:
                robot_msg.goal_type = "joint"
                robot_msg.joint_target = [
                    math.radians(float(e.get())) for e in self._joint_target_entries
                ]
            else:
                robot_msg.goal_type = "task"
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

        if self.robot_cmd_pub is None:
            self.get_logger().warn("robot_cmd_pub not yet bound — waiting for active controller")
            return
        self.robot_cmd_pub.publish(robot_msg)
        data = robot_msg.joint_target if is_joint else list(robot_msg.task_target)
        self.get_logger().info(
            f"Sent robot cmd ({robot_msg.goal_type}): {[f'{v:.4f}' for v in data]}"
        )

        try:
            hand_values = [math.radians(float(e.get())) for e in self._hand_target_entries]
        except ValueError:
            self.get_logger().error("Invalid numerical input for hand target.")
            return
        hand_msg = RobotTarget()
        hand_msg.goal_type = "joint"
        hand_msg.joint_names = list(self._shape.hand_motor_names)
        hand_msg.joint_target = hand_values
        if self.hand_cmd_pub is None:
            self.get_logger().warn("hand_cmd_pub not yet bound — waiting for active controller")
            return
        self.hand_cmd_pub.publish(hand_msg)
        self.get_logger().info(f"Sent hand cmd: {[f'{v:.4f}' for v in hand_values]}")

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
        ctrl_name = data.get("controller")
        if ctrl_name and ctrl_name in CONTROLLER_TYPES:
            # Switch controller if needed
            if self.selected_ctrl.get() != ctrl_name:
                self.selected_ctrl.set(ctrl_name)
                self._on_switch_controller()

            goal_type = data.get(
                "robot_goal_type", "joint" if JOINT_SPACE.get(ctrl_name, True) else "task"
            )
            robot_target_vals = data.get("robot_target", [])

            if len(robot_target_vals) == self._shape.arm_dof:
                robot_msg = RobotTarget()
                robot_msg.joint_names = list(self._shape.arm_joint_names)
                if goal_type == "joint":
                    robot_msg.goal_type = "joint"
                    robot_msg.joint_target = [math.radians(v) for v in robot_target_vals]
                    self._set_joint_target_entries(robot_msg.joint_target)
                else:
                    robot_msg.goal_type = "task"
                    task_values = [
                        math.radians(v) if i >= 3 else v for i, v in enumerate(robot_target_vals)
                    ]
                    robot_msg.task_target = task_values
                    self._set_task_target_entries(task_values)
                if self.robot_cmd_pub is None:
                    self.get_logger().warn(
                        "robot_cmd_pub not yet bound — waiting for active controller"
                    )
                else:
                    self.robot_cmd_pub.publish(robot_msg)
                    self.get_logger().info(
                        f"Preset '{name}': sent robot target ({goal_type}): "
                        f"{[f'{v:.2f}' for v in robot_target_vals]}"
                    )

        # ── Hand target ────────────────────────────────────────────────
        positions_deg = data.get("positions_deg", [0.0] * self._shape.hand_dof)
        grasp_time = data.get("grasp_time", 1.0)
        positions_rad = [math.radians(d) for d in positions_deg]

        # Calculate required hand_trajectory_speed from grasp_time
        max_dist = 0.0
        for target, current in zip(positions_rad, self.current_hand_positions, strict=False):
            max_dist = max(max_dist, abs(target - current))

        if max_dist > 0.001 and grasp_time > 0.01:
            hand_traj_speed = max_dist / grasp_time
        else:
            hand_traj_speed = 1.0

        # Update only the hand_trajectory_speed parameter on the active
        # controller. set_parameters_atomically with a single typed Parameter
        # is cheaper + more transparent than the legacy whole-gains-array
        # publish that used `_HAND_TRAJ_SPEED_IDX` to find the right slot.
        ctrl = self.selected_ctrl.get()
        client = self._get_param_client(ctrl)
        if client.wait_for_services(timeout_sec=0.5):
            param = Parameter(
                name="hand_trajectory_speed",
                type_=Parameter.Type.DOUBLE,
                value=float(hand_traj_speed),
            )
            future = client.set_parameters_atomically([param])

            def _on_done(fut):
                try:
                    resp = fut.result()
                except Exception as exc:
                    self.get_logger().error(f"hand_trajectory_speed set exception: {exc}")
                    return
                if resp.result.successful:
                    self.get_logger().info(
                        f"Preset '{name}': hand_trajectory_speed="
                        f"{hand_traj_speed:.4f} rad/s "
                        f"(grasp_time={grasp_time:.2f}s, max_dist={max_dist:.4f})"
                    )
                else:
                    self.get_logger().error(
                        f"set hand_trajectory_speed rejected: {resp.result.reason}"
                    )

            future.add_done_callback(_on_done)
        else:
            self.get_logger().warn(
                f"parameter services for /{ctrl} unavailable — hand_trajectory_speed not updated"
            )

        # Publish hand target
        hand_msg = RobotTarget()
        hand_msg.goal_type = "joint"
        hand_msg.joint_names = list(self._shape.hand_motor_names)
        hand_msg.joint_target = positions_rad
        if self.hand_cmd_pub is None:
            self.get_logger().warn("hand_cmd_pub not yet bound — waiting for active controller")
        else:
            self.hand_cmd_pub.publish(hand_msg)
            self.get_logger().info(
                f"Sent preset '{name}' ({data.get('type', 'open')}): "
                f"{[f'{v:.2f}' for v in positions_deg]} deg"
            )

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
                    round(math.degrees(v), 4) for v in self.current_positions
                ]
            else:
                preset_data["robot_target"] = [
                    round(v, 4) if i < 3 else round(math.degrees(v), 4)
                    for i, v in enumerate(self.current_task_positions)
                ]
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
                        round(float(e.get()), 4) for e in self._joint_target_entries
                    ]
                except ValueError:
                    messagebox.showerror(
                        "Invalid Input", "Joint target entries contain invalid values."
                    )
                    return
            else:
                try:
                    preset_data["robot_target"] = [
                        round(float(e.get()), 4) for e in self._task_target_entries
                    ]
                except ValueError:
                    messagebox.showerror(
                        "Invalid Input", "Task target entries contain invalid values."
                    )
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
        if not messagebox.askyesno("Confirm Delete", f"Delete preset '{name}'?"):
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
