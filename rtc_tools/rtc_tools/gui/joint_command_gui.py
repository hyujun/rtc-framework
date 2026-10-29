"""joint_command_gui — slider GUI that commands hand joints over ROS 2.

Like ``joint_state_publisher_gui``, but for a hand node that publishes
``sensor_msgs/JointState`` and consumes ``rtc_msgs/JointCommand`` per hand:
it finds the hands that are up, shows one slider per joint with the measured
position above it, and — only for a hand the user enabled — publishes the
slider targets as a position ``JointCommand`` at ``publish_rate_hz``.

Usage::

    ros2 run rtc_tools joint_command_gui --ros-args -p config:=<gui yaml>
    ros2 run rtc_tools joint_command_gui --ros-args -p namespace:=/hand

The YAML keys, defaults and safety rules are documented in
``joint_command_model``; the threading in ``joint_command_ros``.

Not a generalisation of ``rtc_digital_twin``'s ``joint_gui``: that tool
publishes URDF joints as ``JointState`` for visualisation and has no notion
of a measured state, an enabled hand or a safety stop.
"""

from __future__ import annotations

import signal
import sys
import time

import rclpy
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QApplication

from rtc_tools.gui.joint_command_model import ConfigError
from rtc_tools.gui.joint_command_ros import CommandLoop, JointCommandNode, SpinThread
from rtc_tools.gui.joint_command_view import MainWindow

# The panels redraw at this rate; the publish tick runs at publish_rate_hz.
VIEW_RATE_HZ = 10.0


def main(args=None) -> int:
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    try:
        node = JointCommandNode()
    except ConfigError as exc:
        print(f"joint_command_gui: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2

    spin = SpinThread(node)
    loop = CommandLoop(node)
    app = QApplication(sys.argv)
    topics = {t.label: t.state_topic for t in node.candidates}
    window = MainWindow(
        "Joint command GUI",
        "looking for hands: " + ", ".join(topics.values()),
    )

    def step() -> None:
        for label in loop.step(time.monotonic()):
            window.add_hand(loop.controllers[label], topics[label])

    publish_timer = QTimer()
    publish_timer.setTimerType(Qt.PreciseTimer)  # a coarse timer may run 5 % slow
    publish_timer.timeout.connect(step)
    publish_timer.start(round(1000.0 / node.config.publish_rate_hz))
    view_timer = QTimer()
    view_timer.timeout.connect(window.refresh)
    view_timer.start(round(1000.0 / VIEW_RATE_HZ))

    spin.start()
    window.show()
    try:
        app.exec_()
    finally:
        # Closing the window stops publishing; the hands hold their last command.
        publish_timer.stop()
        view_timer.stop()
        spin.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
