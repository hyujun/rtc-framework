"""Qt widgets of the hand joint-command GUI (``python_qt_binding``).

Draws one panel per attached hand and forwards user actions to that hand's
``HandController`` and ``PoseSequencer``; it decides nothing itself. Each
joint row shows the measured position (deg) and effort above a slider whose
red line marks the measured position. While a hand is disabled its sliders
are read-only and follow the measurement; Enable makes them the command
target. Below the sliders a table lists the hand's poses (# · name · dwell,
joint values in the tooltip) with the list, replay and file buttons; while a
replay runs the sliders, ranges and the list are locked and only Stop and
Enable stay live. Unsaved list changes mark the panel title with ``*``.
"""

from __future__ import annotations

import math
import os
import time
from pathlib import Path

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QAbstractItemView,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QStyle,
    QStyleOptionSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from rtc_tools.gui.joint_command_model import HandController, describe_link
from rtc_tools.gui.joint_command_poses import PoseSequencer

_SLIDER_STEPS = 2000
_FILE_FILTER = "YAML pose file (*.yaml *.yml)"


class MarkerSlider(QSlider):
    """Horizontal slider that also draws a marker at another value."""

    def __init__(self) -> None:
        super().__init__(Qt.Horizontal)
        self.setRange(0, _SLIDER_STEPS)
        self._marker: int | None = None

    def set_marker(self, position: int | None) -> None:
        if position != self._marker:
            self._marker = position
            self.update()

    def wheelEvent(self, event) -> None:
        # The wheel scrolls the joint list, never a target: passing the cursor
        # over an enabled slider must not move a joint.
        event.ignore()

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        if self._marker is None:
            return
        option = QStyleOptionSlider()
        self.initStyleOption(option)
        handle = self.style().subControlRect(
            QStyle.CC_Slider, option, QStyle.SC_SliderHandle, self
        )
        span = self.width() - handle.width()
        x = handle.width() // 2 + QStyle.sliderPositionFromValue(
            self.minimum(), self.maximum(), self._marker, span
        )
        painter = QPainter(self)
        painter.setPen(QPen(QColor(220, 40, 40), 2))
        painter.drawLine(x, 1, x, self.height() - 2)


def _edit(width: int) -> QLineEdit:
    edit = QLineEdit()
    edit.setFixedWidth(width)
    edit.setAlignment(Qt.AlignRight)
    return edit


class JointRow(QWidget):
    def __init__(self, panel: HandPanel, index: int, name: str) -> None:
        super().__init__()
        self._panel = panel
        self._index = index

        bold = QFont()
        bold.setBold(True)
        name_label = QLabel(name)
        name_label.setFont(bold)
        self._measured = QLabel("—")
        self._measured.setStyleSheet("color: gray;")
        top = QHBoxLayout()
        top.addWidget(name_label)
        top.addStretch()
        top.addWidget(self._measured)

        self._min = _edit(52)
        self._max = _edit(52)
        self._slider = MarkerSlider()
        self._target = _edit(64)
        self._min.returnPressed.connect(self._on_range)
        self._max.returnPressed.connect(self._on_range)
        self._slider.valueChanged.connect(self._on_slider)
        self._target.returnPressed.connect(self._on_target_text)
        bottom = QHBoxLayout()
        bottom.addWidget(self._min)
        bottom.addWidget(self._slider, 1)
        bottom.addWidget(self._max)
        bottom.addWidget(self._target)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.addLayout(top)
        layout.addLayout(bottom)
        self.refresh_range()

    @property
    def _ctrl(self) -> HandController:
        return self._panel.controller

    def _to_slider(self, deg: float) -> int:
        lo, hi = self._ctrl.ranges[self._index]
        ratio = (min(max(deg, lo), hi) - lo) / (hi - lo)
        return round(ratio * _SLIDER_STEPS)

    def _from_slider(self, position: int) -> float:
        lo, hi = self._ctrl.ranges[self._index]
        return lo + (hi - lo) * position / _SLIDER_STEPS

    def refresh_range(self) -> None:
        lo, hi = self._ctrl.ranges[self._index]
        self._min.setText(f"{lo:g}")
        self._max.setText(f"{hi:g}")

    def refresh(self, measured_deg: float | None, effort: float | None) -> None:
        ctrl = self._ctrl
        text = "—" if measured_deg is None else f"meas {measured_deg:7.2f}°"
        if effort is not None:
            text += f"   effort {effort:6.3f}"
        self._measured.setText(text)
        marker = None
        if measured_deg is not None and math.isfinite(measured_deg):
            marker = self._to_slider(measured_deg)
        self._slider.set_marker(marker)

        locked = self._panel.locked
        self._slider.setEnabled(ctrl.enabled and not locked)
        self._target.setEnabled(ctrl.enabled and not locked)
        self._min.setEnabled(not locked)
        self._max.setEnabled(not locked)
        if ctrl.enabled:
            shown = ctrl.target_deg[self._index]
        else:
            shown = measured_deg if measured_deg is not None else ctrl.target_deg[self._index]
        if not self._slider.isSliderDown() and math.isfinite(shown):
            self._slider.blockSignals(True)
            self._slider.setValue(self._to_slider(shown))
            self._slider.blockSignals(False)
        if not self._target.hasFocus() and math.isfinite(shown):
            self._target.setText(f"{shown:.1f}")

    def _on_slider(self, position: int) -> None:
        if self._ctrl.enabled and not self._panel.locked:
            self._ctrl.set_target(self._index, self._from_slider(position))

    def _on_target_text(self) -> None:
        if self._panel.locked:
            self._panel.show_message("replay in progress")
            return
        try:
            value = float(self._target.text())
        except ValueError:
            value = math.nan
        if self._ctrl.set_target(self._index, value) is None:
            self._panel.show_message(f"rejected target '{self._target.text()}'")
        self._target.clearFocus()

    def _on_range(self) -> None:
        if self._panel.locked:
            reason = "replay in progress"
        else:
            try:
                lo, hi = float(self._min.text()), float(self._max.text())
            except ValueError:
                reason = "min and max must be numbers"
            else:
                reason = self._ctrl.set_range(self._index, lo, hi)
        if reason:
            self._panel.show_message(f"range rejected: {reason}")
        self.refresh_range()


class PoseTable(QTableWidget):
    """# · name · dwell of one hand's poses; name and dwell are editable in place."""

    def __init__(self, panel: HandPanel) -> None:
        super().__init__(0, 3)
        self._panel = panel
        self._shown_revision = -1
        self.setHorizontalHeaderLabels(["#", "name", "dwell s"])
        self.verticalHeader().hide()
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.horizontalHeader().setStretchLastSection(True)
        self.setColumnWidth(0, 36)
        self.setColumnWidth(1, 160)
        self.itemChanged.connect(self._on_item_changed)

    def selected_index(self) -> int | None:
        rows = self.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def select(self, index: int | None) -> None:
        self.clearSelection()
        if index is not None and 0 <= index < self.rowCount():
            self.selectRow(index)

    def rebuild(self, seq: PoseSequencer) -> None:
        if seq.revision == self._shown_revision:
            return
        self._shown_revision = seq.revision
        selected = self.selected_index()
        self.blockSignals(True)
        self.setRowCount(len(seq.poses))
        for row, pose in enumerate(seq.poses):
            tooltip = "\n".join(f"{name}: {deg:.1f}°" for name, deg in pose.joints_deg.items())
            number = QTableWidgetItem(str(row + 1))
            number.setFlags(number.flags() & ~Qt.ItemIsEditable)
            name = QTableWidgetItem(pose.name)
            dwell = QTableWidgetItem(f"{pose.dwell_s:g}")
            dwell.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            for item in (number, name, dwell):
                item.setToolTip(tooltip)
            self.setItem(row, 0, number)
            self.setItem(row, 1, name)
            self.setItem(row, 2, dwell)
        self.blockSignals(False)
        if selected is not None and seq.poses:
            self.select(min(selected, len(seq.poses) - 1))

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        seq = self._panel.sequencer
        row = item.row()
        if item.column() == 1:
            reason = seq.rename_pose(row, item.text())
        elif item.column() == 2:
            try:
                dwell = float(item.text())
            except ValueError:
                dwell = math.nan
            reason = seq.set_dwell(row, dwell)
        else:
            return
        # Show the stored value (stripped name, or the old one if refused)
        # in place: a rebuild here would delete the item still being edited.
        pose = seq.poses[row] if row < len(seq.poses) else None
        if pose is not None:
            self.blockSignals(True)
            item.setText(pose.name if item.column() == 1 else f"{pose.dwell_s:g}")
            self.blockSignals(False)
        if not reason:
            self._shown_revision = seq.revision


class HandPanel(QGroupBox):
    def __init__(
        self, controller: HandController, sequencer: PoseSequencer, state_topic: str
    ) -> None:
        super().__init__()
        self.controller = controller
        self.sequencer = sequencer
        self._title = f"{controller.label} — {state_topic}"
        self._rows: list[JointRow] = []
        self._shown_seq_message = ""

        self._enable = QPushButton("Enable")
        self._enable.setCheckable(True)
        self._enable.clicked.connect(self._on_enable)
        self._reset = QPushButton("Reset ranges")
        self._reset.clicked.connect(self._on_reset)
        self._link = QLabel()
        buttons = QHBoxLayout()
        buttons.addWidget(self._enable)
        buttons.addWidget(self._reset)
        buttons.addStretch()
        buttons.addWidget(self._link)

        self._status = QLabel()
        self._status.setWordWrap(True)
        self._message = QLabel()
        self._message.setWordWrap(True)
        self._message.setStyleSheet("color: #b05000;")

        rows = QWidget()
        self._rows_layout = QVBoxLayout(rows)
        self._rows_layout.addStretch()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(rows)

        self._table = PoseTable(self)
        self._pose_buttons: dict[str, QPushButton] = {}
        edit_row = QHBoxLayout()
        for text, slot in (
            ("Save pose", self._on_save_pose),
            ("Update", self._on_update),
            ("Delete", self._on_delete),
            ("Up", lambda: self._on_move(-1)),
            ("Down", lambda: self._on_move(+1)),
        ):
            edit_row.addWidget(self._button(text, slot))
        play_row = QHBoxLayout()
        for text, slot in (
            ("Go to", self._on_go_to),
            ("Play", self._on_play),
            ("Stop", self._on_stop),
            ("Load…", self._on_load),
            ("Save…", self._on_save_file),
        ):
            play_row.addWidget(self._button(text, slot))

        layout = QVBoxLayout(self)
        layout.addLayout(buttons)
        layout.addWidget(self._status)
        layout.addWidget(self._message)
        layout.addWidget(scroll, 3)
        layout.addWidget(QLabel("Poses"))
        layout.addWidget(self._table, 2)
        layout.addLayout(edit_row)
        layout.addLayout(play_row)
        self.setMinimumWidth(420)
        self.refresh()

    def _button(self, text: str, slot) -> QPushButton:
        button = QPushButton(text)
        button.clicked.connect(slot)
        self._pose_buttons[text] = button
        return button

    @property
    def locked(self) -> bool:
        return self.sequencer.playing

    def show_message(self, text: str) -> None:
        self._message.setText(text)

    # ── hand ─────────────────────────────────────────────────────────────

    def _on_enable(self, checked: bool) -> None:
        if checked:
            reason = self.controller.request_enable(time.monotonic())
            if reason:
                self.show_message(f"enable refused: {reason}")
            else:
                self.show_message("")
        else:
            self.controller.disable()
            self.sequencer.step(time.monotonic())  # ends a running replay now
        self.refresh()

    def _on_reset(self) -> None:
        if self.locked:
            self.show_message("replay in progress")
            return
        reason = self.controller.reset_ranges()
        self.show_message(f"reset refused: {reason}" if reason else "")
        for row in self._rows:
            row.refresh_range()

    # ── poses ────────────────────────────────────────────────────────────

    def _on_save_pose(self) -> None:
        if self.sequencer.save_pose() is None:
            self._table.rebuild(self.sequencer)
            self._table.select(len(self.sequencer.poses) - 1)
        self.refresh()

    def _on_update(self) -> None:
        self.sequencer.update_pose(self._table.selected_index())
        self.refresh()

    def _on_delete(self) -> None:
        self.sequencer.delete_pose(self._table.selected_index())
        self.refresh()

    def _on_move(self, delta: int) -> None:
        index = self._table.selected_index()
        if self.sequencer.move_pose(index, delta) is None:
            self._table.rebuild(self.sequencer)
            self._table.select(index + delta)
        self.refresh()

    def _on_go_to(self) -> None:
        self.sequencer.go_to(self._table.selected_index())
        self.refresh()

    def _on_play(self) -> None:
        self.sequencer.play()
        self.refresh()

    def _on_stop(self) -> None:
        self.sequencer.stop()
        self.refresh()

    def _start_dir(self) -> str:
        path = self.sequencer.file_path
        return str(path.parent) if path else os.getcwd()

    def _on_load(self) -> None:
        label = self.controller.label
        if self.sequencer.dirty:
            answer = QMessageBox.question(
                self,
                f"Load poses — {label}",
                "The pose list has unsaved changes. Replace it?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        path, _ = QFileDialog.getOpenFileName(
            self, f"Load poses — {label}", self._start_dir(), _FILE_FILTER
        )
        if path:
            self.sequencer.load_file(path)
        self.refresh()

    def _on_save_file(self) -> None:
        current = self.sequencer.file_path
        start = str(current) if current else self._start_dir()
        path, _ = QFileDialog.getSaveFileName(
            self, f"Save poses — {self.controller.label}", start, _FILE_FILTER
        )
        if path:
            if not Path(path).suffix:
                path += ".yaml"  # the non-native dialog does not add the filter's suffix
            self.sequencer.save_file(path)
        self.refresh()

    # ── redraw ───────────────────────────────────────────────────────────

    def refresh(self) -> None:
        ctrl = self.controller
        seq = self.sequencer
        if ctrl.ready and not self._rows:
            for i, name in enumerate(ctrl.joint_names):
                row = JointRow(self, i, name)
                self._rows.append(row)
                self._rows_layout.insertWidget(i, row)
        self.setTitle(f"{self._title}{' *' if seq.dirty else ''}")
        self._enable.setChecked(ctrl.enabled)
        self._enable.setText("Enabled — publishing" if ctrl.enabled else "Enable")
        self._link.setText(f"link: {describe_link(ctrl.link_ok)}")
        self._status.setText(ctrl.status_line(time.monotonic()))
        if seq.message != self._shown_seq_message:
            self._shown_seq_message = seq.message
            self.show_message(seq.message)

        sample = ctrl.latest
        measured = ctrl.measured_deg() or [None] * len(self._rows)
        efforts = list(sample.effort) if sample else []
        for i, row in enumerate(self._rows):
            row.refresh(measured[i], efforts[i] if i < len(efforts) else None)

        locked = self.locked
        self._reset.setEnabled(not locked)
        self._table.rebuild(seq)
        self._table.setEditTriggers(
            QAbstractItemView.NoEditTriggers
            if locked
            else QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed
        )
        if locked and seq.current is not None and self._table.selected_index() != seq.current:
            self._table.select(seq.current)
        selected = self._table.selected_index() is not None
        buttons = self._pose_buttons
        buttons["Save pose"].setEnabled(ctrl.ready and not locked)
        for text in ("Update", "Delete", "Up", "Down"):
            buttons[text].setEnabled(selected and not locked)
        buttons["Go to"].setEnabled(ctrl.enabled and selected and not locked)
        buttons["Play"].setEnabled(ctrl.enabled and bool(seq.poses) and not locked)
        buttons["Stop"].setEnabled(locked)
        buttons["Load…"].setEnabled(ctrl.ready and not locked)
        buttons["Save…"].setEnabled(ctrl.ready and not locked)


class MainWindow(QWidget):
    def __init__(self, title: str, waiting_text: str) -> None:
        super().__init__()
        self.setWindowTitle(title)
        self._waiting = QLabel(waiting_text)
        self._waiting.setWordWrap(True)
        self._panels_layout = QHBoxLayout()
        layout = QVBoxLayout(self)
        layout.addWidget(self._waiting)
        layout.addLayout(self._panels_layout, 1)
        self.panels: dict[str, HandPanel] = {}
        self.resize(480, 720)

    def add_hand(
        self, controller: HandController, sequencer: PoseSequencer, state_topic: str
    ) -> None:
        panel = HandPanel(controller, sequencer, state_topic)
        self.panels[controller.label] = panel
        self._panels_layout.addWidget(panel)
        self._waiting.hide()
        self.resize(max(self.width(), 460 * len(self.panels)), self.height())

    def refresh(self) -> None:
        for panel in self.panels.values():
            panel.refresh()
