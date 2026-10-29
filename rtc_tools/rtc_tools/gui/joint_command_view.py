"""Qt widgets of the hand joint-command GUI (``python_qt_binding``).

Draws one panel per attached hand and forwards user actions to that hand's
``HandController``; it decides nothing itself. Each joint row shows the
measured position (deg) and effort above a slider whose red line marks the
measured position. While a hand is disabled its sliders are read-only and
follow the measurement; Enable makes them the command target.
"""

from __future__ import annotations

import math
import time

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QStyle,
    QStyleOptionSlider,
    QVBoxLayout,
    QWidget,
)

from rtc_tools.gui.joint_command_model import HandController, describe_link

_SLIDER_STEPS = 2000


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

        self._slider.setEnabled(ctrl.enabled)
        self._target.setEnabled(ctrl.enabled)
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
        if self._ctrl.enabled:
            self._ctrl.set_target(self._index, self._from_slider(position))

    def _on_target_text(self) -> None:
        try:
            value = float(self._target.text())
        except ValueError:
            value = math.nan
        if self._ctrl.set_target(self._index, value) is None:
            self._panel.show_message(f"rejected target '{self._target.text()}'")
        self._target.clearFocus()

    def _on_range(self) -> None:
        try:
            lo, hi = float(self._min.text()), float(self._max.text())
        except ValueError:
            reason = "min and max must be numbers"
        else:
            reason = self._ctrl.set_range(self._index, lo, hi)
        if reason:
            self._panel.show_message(f"range rejected: {reason}")
        self.refresh_range()


class HandPanel(QGroupBox):
    def __init__(self, controller: HandController, state_topic: str) -> None:
        super().__init__(f"{controller.label} — {state_topic}")
        self.controller = controller
        self._rows: list[JointRow] = []

        self._enable = QPushButton("Enable")
        self._enable.setCheckable(True)
        self._enable.clicked.connect(self._on_enable)
        reset = QPushButton("Reset ranges")
        reset.clicked.connect(self._on_reset)
        self._link = QLabel()
        buttons = QHBoxLayout()
        buttons.addWidget(self._enable)
        buttons.addWidget(reset)
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

        layout = QVBoxLayout(self)
        layout.addLayout(buttons)
        layout.addWidget(self._status)
        layout.addWidget(self._message)
        layout.addWidget(scroll, 1)
        self.setMinimumWidth(420)

    def show_message(self, text: str) -> None:
        self._message.setText(text)

    def _on_enable(self, checked: bool) -> None:
        if checked:
            reason = self.controller.request_enable(time.monotonic())
            if reason:
                self.show_message(f"enable refused: {reason}")
            else:
                self.show_message("")
        else:
            self.controller.disable()
        self.refresh()

    def _on_reset(self) -> None:
        reason = self.controller.reset_ranges()
        self.show_message(f"reset refused: {reason}" if reason else "")
        for row in self._rows:
            row.refresh_range()

    def refresh(self) -> None:
        ctrl = self.controller
        if ctrl.ready and not self._rows:
            for i, name in enumerate(ctrl.joint_names):
                row = JointRow(self, i, name)
                self._rows.append(row)
                self._rows_layout.insertWidget(i, row)
        self._enable.setChecked(ctrl.enabled)
        self._enable.setText("Enabled — publishing" if ctrl.enabled else "Enable")
        self._link.setText(f"link: {describe_link(ctrl.link_ok)}")
        self._status.setText(ctrl.status_line(time.monotonic()))

        sample = ctrl.latest
        measured = ctrl.measured_deg() or [None] * len(self._rows)
        efforts = list(sample.effort) if sample else []
        for i, row in enumerate(self._rows):
            row.refresh(measured[i], efforts[i] if i < len(efforts) else None)


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

    def add_hand(self, controller: HandController, state_topic: str) -> None:
        panel = HandPanel(controller, state_topic)
        self.panels[controller.label] = panel
        self._panels_layout.addWidget(panel)
        self._waiting.hide()
        self.resize(max(self.width(), 460 * len(self.panels)), self.height())

    def refresh(self) -> None:
        for panel in self.panels.values():
            panel.refresh()
