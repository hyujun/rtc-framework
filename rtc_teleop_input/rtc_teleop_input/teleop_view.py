"""Qt window of the teleop input (``python_qt_binding``).

Shows the pad (``PadCanvas``) drawn from the frame the node published last,
an E-STOP button, the state of the input, of the controller manager (active
controller, E-STOP) and of the hold buttons, the replies of the service
calls, and the status panel. It routes keyboard and mouse to ``VirtualPad``
and decides nothing itself.

Help (``help_key`` in the window, ``help_combo`` on a real pad) numbers the
controls on the pad and lists what each one does (``model.help_entries``).
The help key is taken in both inputs, press only; Esc does not close it (Esc
is the default E-STOP key).

Keys are taken by an application-wide event filter, so a focused child widget
never eats a bound key (Space would otherwise click a focused button). Every
widget here is ``NoFocus`` for the same reason. Losing focus — the window's
focus, the window's activation or the application's — releases everything.

Values from the rclpy thread arrive only through the ``RosBridge`` signals,
which Qt queues to this (GUI) thread.
"""

from __future__ import annotations

import html
import time

from python_qt_binding.QtCore import QEvent, QObject, Qt, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from rtc_teleop_input.model import ConfigError, StatusBoard, StatusEntry, help_entries
from rtc_teleop_input.pad_canvas import PadCanvas

_LEVEL_COLORS = {0: "#2e7d32", 1: "#ef6c00", 2: "#c62828", 3: "#6d6d6d"}


def key_codes(key_names: tuple[str, ...]) -> dict[int, str]:
    """Qt key code -> configured key name (``Qt.Key_<name>``)."""
    codes = {}
    for name in key_names:
        code = getattr(Qt, "Key_" + name, None)
        if code is None:
            raise ConfigError(f"unknown key name '{name}' (expected a Qt::Key_<name> suffix)")
        codes[int(code)] = name
    return codes


class RosBridge(QObject):
    """Signals the rclpy thread emits; connected receivers run on the GUI thread."""

    status = Signal(object)  # StatusEntry
    service_result = Signal(str, bool, str)
    cm_state = Signal(str, object)  # ("active", name) | ("estop", latched)


class _KeyFilter(QObject):
    def __init__(self, window: TeleopWindow) -> None:
        super().__init__(window)
        self._window = window

    def eventFilter(self, obj, event):  # noqa: N802 (Qt override)
        kind = event.type()
        if kind in (QEvent.KeyPress, QEvent.KeyRelease):
            return self._window.handle_key(
                int(event.key()), kind == QEvent.KeyPress, event.isAutoRepeat()
            )
        return False


class TeleopWindow(QWidget):
    def __init__(self, node, bridge: RosBridge) -> None:
        super().__init__()
        self._node = node
        config = node.config
        self._virtual_input = config.input == "virtual"
        self._codes = key_codes(node.bindings.keys())
        self._help_code = next(iter(key_codes((config.help_key,)))) if config.help_key else None
        self._board = StatusBoard(config.status_topics)
        self._holds = node.actions.holds
        self._replies = {"estop": "-", **{h.key: "-" for h in self._holds}}
        self._cm = {"active": None, "estop": None}
        self.help_entries = help_entries(config)
        self.help_visible = False
        self._help_toggles_seen = node.help_toggles
        self.setWindowTitle(f"Teleop input — {config.input} → {config.output_topic}")
        self.setFocusPolicy(Qt.StrongFocus)

        self.canvas = PadCanvas(config.skin)
        self.canvas.set_interactive(self._virtual_input)
        self.canvas.stick_dragged.connect(node.virtual.drag_stick)
        self.canvas.stick_released.connect(node.virtual.release_stick)
        self.canvas.button_pressed.connect(lambda name: node.virtual.mouse_control(name, True))
        self.canvas.button_released.connect(lambda name: node.virtual.mouse_control(name, False))

        self.estop_button = QPushButton("E-STOP")
        self.estop_button.setFocusPolicy(Qt.NoFocus)
        self.estop_button.setMinimumHeight(56)
        self.estop_button.setStyleSheet(
            "QPushButton { background: #c62828; color: white; font-weight: bold;"
            " font-size: 18px; border-radius: 6px; }"
            "QPushButton:pressed { background: #8e0000; }"
        )
        # on mouse-down: a press that slides off the button still stops
        self.estop_button.pressed.connect(node.actions.request_estop)

        self.source_label = QLabel()
        self.cm_label = QLabel()
        self.deadman_label = QLabel()
        self.reset_label = QLabel()
        self.holds_label = QLabel()
        self.reply_label = QLabel()
        for label in (
            self.source_label,
            self.cm_label,
            self.deadman_label,
            self.reset_label,
            self.holds_label,
            self.reply_label,
        ):
            label.setWordWrap(True)
            label.setTextFormat(Qt.RichText)

        self.status_tree = QTreeWidget()
        self.status_tree.setFocusPolicy(Qt.NoFocus)
        self.status_tree.setColumnCount(2)
        self.status_tree.setHeaderLabels(["key", "value"])
        self.status_tree.setVisible(bool(config.status_topics))
        self._status_items: dict[str, QTreeWidgetItem] = {}

        side = QVBoxLayout()
        side.addWidget(self.estop_button)
        side.addWidget(self.source_label)
        side.addWidget(self.cm_label)
        side.addWidget(self.deadman_label)
        side.addWidget(self.reset_label)
        side.addWidget(self.holds_label)
        side.addWidget(self.reply_label)
        self.holds_label.setVisible(any(h.kind != "reset" for h in self._holds))
        side.addWidget(self.status_tree, 1)
        if not config.status_topics:
            side.addStretch(1)
        self.help_label = QLabel(self._help_text())
        self.help_label.setWordWrap(True)
        self.legend = QTreeWidget()
        self.legend.setFocusPolicy(Qt.NoFocus)
        self.legend.setRootIsDecorated(False)
        headers = ["#", "control", "function"] + (["key"] if self._virtual_input else [])
        self.legend.setColumnCount(len(headers))
        self.legend.setHeaderLabels(headers)
        for number, entry in enumerate(self.help_entries, 1):
            item = QTreeWidgetItem(self.legend)
            item.setText(0, str(number))
            item.setText(1, " + ".join(entry.controls) or "-")
            item.setText(2, entry.function)
            if self._virtual_input:
                item.setText(3, entry.keys)
        for column in range(len(headers)):
            self.legend.resizeColumnToContents(column)
        self.legend.setVisible(False)
        left = QVBoxLayout()
        left.addWidget(self.canvas, 1)
        left.addWidget(self.legend)
        left.addWidget(self.help_label)
        root = QHBoxLayout(self)
        root.addLayout(left, 3)
        root.addLayout(side, 2)

        bridge.status.connect(self.on_status)
        bridge.service_result.connect(self.on_service_result)
        bridge.cm_state.connect(self.on_cm_state)
        self._key_filter = _KeyFilter(self)
        QApplication.instance().installEventFilter(self._key_filter)
        QApplication.instance().applicationStateChanged.connect(self._on_app_state)
        self.refresh()

    # ── input ───────────────────────────────────────────────────────────

    def handle_key(self, code: int, pressed: bool, auto_repeat: bool) -> bool:
        """Returns True when the key is bound (the event is consumed)."""
        if code == self._help_code:
            if pressed and not auto_repeat:
                self.toggle_help()
            return True
        name = self._codes.get(code)
        if name is None:
            return False
        if self._virtual_input:
            self._node.virtual.key_event(name, pressed, auto_repeat)
        return True

    def toggle_help(self) -> None:
        self.help_visible = not self.help_visible
        self.legend.setVisible(self.help_visible)
        self.help_label.setVisible(not self.help_visible)
        self.canvas.set_help(
            [(n, e.controls) for n, e in enumerate(self.help_entries, 1)]
            if self.help_visible
            else []
        )

    def release_all(self) -> None:
        self._node.virtual.focus_lost()
        self.canvas.cancel_interaction()

    def _on_app_state(self, state) -> None:
        if state != Qt.ApplicationActive:
            self.release_all()

    def focusOutEvent(self, event):  # noqa: N802 (Qt override)
        self.release_all()
        super().focusOutEvent(event)

    def changeEvent(self, event):  # noqa: N802 (Qt override)
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
            self.release_all()
        super().changeEvent(event)

    def event(self, event):
        if event.type() == QEvent.WindowDeactivate:
            self.release_all()
        return super().event(event)

    # ── from the rclpy thread (queued) ──────────────────────────────────

    def on_status(self, entry: StatusEntry) -> None:
        self._board.update(entry)

    def on_cm_state(self, kind: str, value) -> None:
        self._cm[kind] = value

    def on_service_result(self, kind: str, ok: bool, message: str) -> None:
        if kind == "estop":
            text = "latched" if ok else "NOT latched"
        elif kind == "clear":
            # ok=false may still mean "lowered, but not verified": the message says which
            text = "lowered" if ok else "NOT confirmed lowered"
        elif kind.startswith("trigger:"):
            text = "ok" if ok else "failed"
        else:
            text = "ok" if ok else "refused"
        if message:
            text += f" ({html.escape(message)})"
        color = "#2e7d32" if ok else "#c62828"
        self._replies[kind] = f"<span style='color:{color}'>{text}</span>"

    # ── view tick ───────────────────────────────────────────────────────

    def refresh(self) -> None:
        node = self._node
        while self._help_toggles_seen != node.help_toggles:  # the pad combination
            self._help_toggles_seen += 1
            self.toggle_help()
        frame = node.last_frame
        # drawn from the published frame only: the window shows what went out
        self.shown_frame = frame
        layout = node.layout
        self.canvas.set_state(
            dict(zip(layout.axes, frame.axes, strict=True)),
            dict(zip(layout.buttons, frame.buttons, strict=True)),
        )
        config = node.config
        if node.relay is None:
            source = "<b>virtual</b> (keyboard · mouse)"
        elif frame.lost:
            source = (
                f"<b>pad</b> {html.escape(config.pad_topic)}: "
                "<span style='color:#c62828'><b>pad lost</b> — publishing released</span>"
            )
        else:
            source = f"<b>pad</b> {html.escape(config.pad_topic)}: connected"
        if node.relay is not None and node.relay.rejects:
            source += f" · rejected Joy: {node.relay.rejects}"
        self.source_label.setText(f"source: {source}")
        active, estop = self._cm["active"], self._cm["estop"]
        self.cm_label.setText(
            "active: "
            + (f"<b>{html.escape(active)}</b>" if active is not None else "unknown")
            + " · E-STOP: "
            + {
                None: "unknown",
                True: "<b style='color:#c62828'>LATCHED</b>",
                False: "<span style='color:#2e7d32'>clear</span>",
            }[estop]
        )
        deadman = frame.buttons[layout.button_index(config.deadman_button)]
        self.deadman_label.setText(
            f"deadman ({config.deadman_button}): "
            + ("<b style='color:#2e7d32'>held</b>" if deadman else "released")
        )
        if not config.reset_enabled:
            reset = "disabled (reset_controller_name is empty)"
        else:
            progress = node.actions.reset_progress(time.monotonic())
            reset = (
                f"hold {config.reset_button} {config.reset_hold_s:g} s with the deadman released"
            )
            if progress is not None:
                reset += f" — {progress * 100:.0f} %"
        self.reset_label.setText(f"reset_fault: {reset}")
        now = time.monotonic()
        holds = []
        for entry, hold in enumerate(self._holds):
            if hold.kind == "reset":
                continue
            line = f"{html.escape(hold.button)}: {html.escape(hold.key)}"
            progress = node.actions.progress(entry, now)
            if progress is not None:
                line += f" — {progress * 100:.0f} %"
            holds.append(line)
        self.holds_label.setText(
            f"hold {config.hold_s:g} s, deadman released:<br>" + "<br>".join(holds)
        )
        replies = [
            f"E-STOP sent {node.estop_calls_sent}, last reply: {self._replies['estop']}",
            f"reset sent {node.reset_calls_sent}, last reply: {self._replies['reset']}",
        ]
        for hold in self._holds:
            if hold.kind == "reset":
                continue
            pending = " (pending)" if node.pending(hold.key) else ""
            replies.append(
                f"{html.escape(hold.key)} sent {node.calls_sent.get(hold.key, 0)}{pending}, "
                f"last reply: {self._replies[hold.key]}"
            )
        self.reply_label.setText("<br>".join(replies))
        self._refresh_status(time.monotonic())

    def _refresh_status(self, now: float) -> None:
        for entry in self._board.rows():
            item = self._status_items.get(entry.topic)
            if item is None:
                item = QTreeWidgetItem(self.status_tree)
                item.setExpanded(True)
                self._status_items[entry.topic] = item
            # a silent publisher is not trusted to still be what it last said
            stale = entry.is_stale(now)
            level = 3 if stale else entry.level
            item.setText(0, f"{entry.topic}  [{'STALE' if stale else entry.level_name}]")
            item.setText(1, entry.message or entry.name)
            # the tooltip guesses rich text; the name comes off the network
            item.setData(0, Qt.ToolTipRole, html.escape(entry.name))
            item.setForeground(0, QColor(_LEVEL_COLORS.get(level, "#6d6d6d")))
            while item.childCount() > len(entry.values):
                item.removeChild(item.child(item.childCount() - 1))
            for i, (key, value) in enumerate(entry.values):
                child = item.child(i) if i < item.childCount() else QTreeWidgetItem(item)
                child.setText(0, key)
                child.setText(1, value)

    def _help_text(self) -> str:
        config = self._node.config
        help_hint = []
        if config.help_key:
            help_hint.append(f"{config.help_key} in this window")
        if config.help_combo and not self._virtual_input:
            help_hint.append(" + ".join(config.help_combo) + " on the pad")
        hint = f" Help: {' or '.join(help_hint)}." if help_hint else ""
        if not self._virtual_input:
            return "Mirror of the real pad. The E-STOP button still calls E-STOP." + hint
        return (
            "Keys: "
            + ", ".join(config.key_axes + config.key_buttons)
            + " — mouse: drag a stick, hold a button. Leaving the window releases everything."
            + hint
        )
