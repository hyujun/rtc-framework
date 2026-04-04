"""Joint State Publisher GUI for Digital Twin.

Provides a Qt-based GUI (similar to joint_state_publisher_gui) that allows
manual control of active URDF joints. Key differences from the standard
joint_state_publisher_gui:

- Displays values in **degrees** (internally converts to radians for publish)
- URDF joint limits are shown and enforced on sliders
- Joint values can be entered directly via editable text fields (Enter to apply)
- Only active joints are shown (mimic / closed-chain / fixed excluded)
- Integrated into rtc_digital_twin with YAML flag control

Usage (standalone)::

    ros2 run rtc_digital_twin joint_gui_node --ros-args \\
        -p robot_description:="$(xacro /path/to/robot.urdf.xacro)"

Usage (via launch)::

    ros2 launch rtc_digital_twin digital_twin.launch.py \\
        robot_description_package:=ur5e_description \\
        robot_description_path:=robots/ur5e/urdf/ur5e_with_hand.urdf.xacro \\
        use_joint_gui:=true
"""

from __future__ import annotations

import math
import signal
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from rtc_digital_twin.urdf_parser import UrdfParser, JointMeta

# Slider internal resolution (same as joint_state_publisher_gui)
_SLIDER_RANGE = 10000

# Default limits for continuous joints (degrees)
_CONTINUOUS_MIN_DEG = -360.0
_CONTINUOUS_MAX_DEG = 360.0


class JointWidget(QWidget):
    """Widget for a single joint: label + limit range + slider + editable entry."""

    def __init__(self, meta: JointMeta, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._meta = meta
        self._is_prismatic = meta.joint_type == 'prismatic'
        self._is_continuous = meta.joint_type == 'continuous'

        # Determine display range
        if self._is_prismatic:
            self._min_display = meta.lower
            self._max_display = meta.upper
            unit = 'm'
        elif self._is_continuous:
            self._min_display = _CONTINUOUS_MIN_DEG
            self._max_display = _CONTINUOUS_MAX_DEG
            unit = 'deg'
        else:
            # revolute — convert rad to deg
            self._min_display = math.degrees(meta.lower)
            self._max_display = math.degrees(meta.upper)
            unit = 'deg'

        # Handle zero-range (both limits zero or equal)
        if abs(self._max_display - self._min_display) < 1e-9:
            if self._is_prismatic:
                self._min_display = -1.0
                self._max_display = 1.0
            else:
                self._min_display = _CONTINUOUS_MIN_DEG
                self._max_display = _CONTINUOUS_MAX_DEG

        self._current_value = 0.0  # in display units (deg or m)

        font_bold = QFont('Helvetica', 9)
        font_bold.setBold(True)
        font_normal = QFont('Helvetica', 9)

        layout = QVBoxLayout()
        layout.setContentsMargins(4, 2, 4, 2)

        # Row 1: joint name + limit range + unit
        row1 = QHBoxLayout()
        name_label = QLabel(meta.name)
        name_label.setFont(font_bold)
        row1.addWidget(name_label)

        row1.addStretch()

        if not self._is_continuous:
            limit_text = f'[{self._min_display:.1f} ~ {self._max_display:.1f}]'
            limit_label = QLabel(limit_text)
            limit_label.setFont(font_normal)
            limit_label.setStyleSheet('color: gray;')
            row1.addWidget(limit_label)

        unit_label = QLabel(unit)
        unit_label.setFont(font_normal)
        unit_label.setStyleSheet('color: gray;')
        row1.addWidget(unit_label)

        layout.addLayout(row1)

        # Row 2: slider + editable entry
        row2 = QHBoxLayout()

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, _SLIDER_RANGE)
        self._slider.setFixedWidth(200)
        self._slider.setValue(self._value_to_slider(self._current_value))
        self._slider.valueChanged.connect(self._on_slider_changed)
        row2.addWidget(self._slider)

        self._entry = QLineEdit()
        self._entry.setFont(font_bold)
        self._entry.setAlignment(Qt.AlignRight)
        self._entry.setFixedWidth(70)
        self._entry.setText(f'{self._current_value:.1f}')
        self._entry.returnPressed.connect(self._on_entry_return)
        row2.addWidget(self._entry)

        layout.addLayout(row2)
        self.setLayout(layout)

    @property
    def joint_name(self) -> str:
        return self._meta.name

    @property
    def value_rad(self) -> float:
        """Current value in radians (or meters for prismatic)."""
        if self._is_prismatic:
            return self._current_value
        return math.radians(self._current_value)

    def set_value_display(self, value: float) -> None:
        """Set value in display units (deg or m), clamped to limits."""
        value = max(self._min_display, min(self._max_display, value))
        self._current_value = value
        self._slider.blockSignals(True)
        self._slider.setValue(self._value_to_slider(value))
        self._slider.blockSignals(False)
        self._entry.setText(f'{value:.1f}')

    def _value_to_slider(self, value: float) -> int:
        span = self._max_display - self._min_display
        if abs(span) < 1e-9:
            return _SLIDER_RANGE // 2
        ratio = (value - self._min_display) / span
        return int(ratio * _SLIDER_RANGE)

    def _slider_to_value(self, slider_pos: int) -> float:
        span = self._max_display - self._min_display
        return self._min_display + (slider_pos / _SLIDER_RANGE) * span

    def _on_slider_changed(self, pos: int) -> None:
        value = self._slider_to_value(pos)
        self._current_value = value
        self._entry.setText(f'{value:.1f}')

    def _on_entry_return(self) -> None:
        text = self._entry.text().strip()
        try:
            value = float(text)
        except ValueError:
            # Restore current value on invalid input
            self._entry.setText(f'{self._current_value:.1f}')
            return
        self.set_value_display(value)


class JointGuiWindow(QMainWindow):
    """Main window containing all joint widgets."""

    def __init__(self, joints: list[JointMeta]) -> None:
        super().__init__()
        self.setWindowTitle('Joint State Publisher GUI')

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout()

        # Buttons
        btn_layout = QHBoxLayout()
        zero_btn = QPushButton('Zero All')
        zero_btn.clicked.connect(self._zero_all)
        btn_layout.addWidget(zero_btn)

        center_btn = QPushButton('Center All')
        center_btn.clicked.connect(self._center_all)
        btn_layout.addWidget(center_btn)

        main_layout.addLayout(btn_layout)

        # Scroll area with joint widgets
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout()

        self._joint_widgets: list[JointWidget] = []
        for meta in joints:
            w = JointWidget(meta)
            self._joint_widgets.append(w)
            scroll_layout.addWidget(w)

        scroll_layout.addStretch()
        scroll_widget.setLayout(scroll_layout)
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll)

        central.setLayout(main_layout)

        # Window sizing
        n_joints = len(joints)
        row_height = 64
        margins = 80  # buttons + window margins
        max_visible = 7
        height = margins + min(n_joints, max_visible) * row_height
        self.setMinimumWidth(350)
        self.resize(400, max(200, height))

    def get_joint_positions(self) -> list[tuple[str, float]]:
        """Return list of (joint_name, position_rad) for all joints."""
        return [(w.joint_name, w.value_rad) for w in self._joint_widgets]

    def _zero_all(self) -> None:
        for w in self._joint_widgets:
            w.set_value_display(0.0)

    def _center_all(self) -> None:
        for w in self._joint_widgets:
            center = (w._min_display + w._max_display) / 2.0
            w.set_value_display(center)


class JointGuiNode(Node):
    """ROS2 node that publishes JointState from the GUI."""

    def __init__(self) -> None:
        super().__init__('joint_gui_node')

        self.declare_parameter('robot_description', '')
        self.declare_parameter('joint_gui.output_topic', '/joint_gui/joint_states')
        self.declare_parameter('joint_gui.publish_rate', 10.0)

        robot_description = self.get_parameter('robot_description').value
        output_topic = self.get_parameter('joint_gui.output_topic').value
        publish_rate = self.get_parameter('joint_gui.publish_rate').value

        if not robot_description:
            self.get_logger().fatal('robot_description parameter is empty')
            raise RuntimeError('robot_description parameter is required')

        # Parse URDF
        self.get_logger().debug('Parsing URDF for joint discovery...')
        try:
            parser = UrdfParser.from_xml(robot_description)
        except Exception as e:
            self.get_logger().fatal(f'Failed to parse URDF: {e}')
            raise

        active_names = parser.get_active_joint_names()  # sorted
        self._joint_metas = [
            parser.get_joint_meta(name) for name in active_names]

        self.get_logger().info(
            f'Joint GUI: {len(self._joint_metas)} active joints, '
            f'output={output_topic}, rate={publish_rate}Hz')
        self.get_logger().debug(
            f'Active joints: {active_names}')

        # Publisher
        self._pub = self.create_publisher(JointState, output_topic, 10)

        # Publish timer
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)

        # GUI window (created later in main)
        self._window: JointGuiWindow | None = None

    def set_window(self, window: JointGuiWindow) -> None:
        self._window = window

    @property
    def joint_metas(self) -> list[JointMeta]:
        return self._joint_metas

    def _publish(self) -> None:
        if self._window is None:
            return
        positions = self._window.get_joint_positions()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, rad in positions:
            msg.name.append(name)
            msg.position.append(rad)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Allow Ctrl+C to work
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    node = JointGuiNode()

    app = QApplication(sys.argv)
    window = JointGuiWindow(node.joint_metas)
    node.set_window(window)
    window.show()

    # Spin ROS2 via Qt timer
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    spin_timer.start(10)

    node.get_logger().info('Joint GUI window opened')

    try:
        app.exec_()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down (KeyboardInterrupt)')
    finally:
        node.get_logger().info('Joint GUI shutting down')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
