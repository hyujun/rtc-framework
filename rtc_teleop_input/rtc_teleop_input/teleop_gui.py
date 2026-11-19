"""teleop_input — publishes operator input as sensor_msgs/Joy (standard gamepad layout).

``input:=virtual`` draws a pad driven by keyboard and mouse; ``input:=pad``
relays a real pad (joy's ``game_controller_node``) and mirrors it. Either
way one Joy goes out every ``1 / rate_hz`` s with ``header.frame_id`` = the
input, and the pad's buttons call the controller manager's E-STOP and
reset_fault services. Parameters: ``model.TeleopConfig``; threading:
``teleop_ros``; window: ``teleop_view``.

Usage::

    ros2 launch rtc_teleop_input teleop_input.launch.py joy_topic:=<topic>
"""

from __future__ import annotations

import signal
import sys
import time

import rclpy
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QApplication

from rtc_teleop_input.model import ConfigError
from rtc_teleop_input.teleop_ros import SpinThread, TeleopInputNode
from rtc_teleop_input.teleop_view import RosBridge, TeleopWindow

# The window redraws at this rate; the publish tick runs at rate_hz.
VIEW_RATE_HZ = 30.0


def start_timers(node, window) -> tuple[QTimer, QTimer]:
    """The publish tick and the redraw, both on the GUI thread."""
    publish_timer = QTimer()
    publish_timer.setTimerType(Qt.PreciseTimer)  # a coarse timer may run 5 % slow
    publish_timer.timeout.connect(lambda: node.step(time.monotonic()))
    publish_timer.start(round(1000.0 / node.config.rate_hz))
    view_timer = QTimer()
    view_timer.timeout.connect(window.refresh)
    view_timer.start(round(1000.0 / VIEW_RATE_HZ))
    return publish_timer, view_timer


def main(args=None) -> int:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    # Ctrl-C (and launch's SIGINT) closes the window cleanly: the handler runs
    # when Python next gets control, at the latest on the next publish tick.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    bridge = RosBridge()
    try:
        node = TeleopInputNode(
            on_status=bridge.status.emit, on_service_result=bridge.service_result.emit
        )
        window = TeleopWindow(node, bridge)
    except ConfigError as exc:
        print(f"teleop_input: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2

    spin = SpinThread(node)
    publish_timer, view_timer = start_timers(node, window)

    spin.start()
    window.show()
    try:
        app.exec_()
    finally:
        # Closing the window stops publishing: the consumer sees its input go stale.
        publish_timer.stop()
        view_timer.stop()
        spin.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
