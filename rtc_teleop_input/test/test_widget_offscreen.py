"""The window's wiring, offscreen: Qt key / focus events -> VirtualPad, queued status.

No skip on purpose: the dev and CI hosts have python_qt_binding, and a
missing binding must fail here rather than pass the focus rules untested.
"""

from __future__ import annotations

import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"  # forced: a desktop session must not matter

import threading  # noqa: E402
import time  # noqa: E402

import pytest  # noqa: E402
from python_qt_binding.QtCore import QCoreApplication, QEvent, Qt, QTimer  # noqa: E402
from python_qt_binding.QtGui import QFocusEvent, QKeyEvent  # noqa: E402
from python_qt_binding.QtTest import QTest  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

from rtc_teleop_input.model import StatusEntry  # noqa: E402
from rtc_teleop_input.teleop_gui import start_timers  # noqa: E402
from rtc_teleop_input.teleop_ros import SpinThread, TeleopInputNode  # noqa: E402
from rtc_teleop_input.teleop_view import RosBridge, TeleopWindow  # noqa: E402

LEFTSHOULDER = 9
LEFTY = 1


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def rig(app, ros, uid):
    node = TeleopInputNode(
        parameter_overrides=[
            Parameter("output_topic", value=f"/{uid}/joy"),
            Parameter("status_topics", value=[f"/{uid}/status"]),
        ]
    )
    bridge = RosBridge()
    window = TeleopWindow(node, bridge)
    window.show()
    app.processEvents()
    yield app, node, bridge, window
    window.close()
    app.removeEventFilter(window._key_filter)
    window.deleteLater()
    app.processEvents()
    node.destroy_node()


def key(window, code, pressed, auto_repeat=False):
    kind = QEvent.KeyPress if pressed else QEvent.KeyRelease
    QApplication.sendEvent(window, QKeyEvent(kind, code, Qt.NoModifier, "", auto_repeat))


def held_state(node):
    frame = node.virtual.frame()
    return frame.axes[LEFTY], frame.buttons[LEFTSHOULDER]


def test_keys_reach_the_model(rig):
    _, node, _, window = rig
    key(window, Qt.Key_W, True)
    key(window, Qt.Key_Space, True)
    assert held_state(node) == (0.25, 1)
    key(window, Qt.Key_W, False)
    key(window, Qt.Key_Space, False)
    assert held_state(node) == (0.0, 0)


def test_auto_repeat_through_qt(rig):
    _, node, _, window = rig
    key(window, Qt.Key_Space, True)
    node.virtual.frame()
    for _ in range(100):
        key(window, Qt.Key_Space, False, auto_repeat=True)
        key(window, Qt.Key_Space, True, auto_repeat=True)
        assert held_state(node)[1] == 1
    key(window, Qt.Key_Space, False)
    assert held_state(node)[1] == 0


def _focus_out(app, window):
    QApplication.sendEvent(window, QFocusEvent(QEvent.FocusOut, Qt.ActiveWindowFocusReason))


def _deactivate(app, window):
    QApplication.sendEvent(window, QEvent(QEvent.WindowDeactivate))


def _app_inactive(app, window):
    app.applicationStateChanged.emit(Qt.ApplicationInactive)


@pytest.mark.parametrize("lose_focus", [_focus_out, _deactivate, _app_inactive])
def test_focus_loss_releases_everything(rig, lose_focus):
    app, node, _, window = rig
    key(window, Qt.Key_W, True)
    key(window, Qt.Key_Space, True)
    node.virtual.drag_stick("right", 0.5, 0.5)
    node.virtual.frame()
    lose_focus(app, window)
    frame = node.virtual.frame()
    assert frame.axes == (0.0,) * 6
    assert frame.buttons == (0,) * 21
    # still physically held when focus returns: auto-repeat must not revive it
    key(window, Qt.Key_W, True, auto_repeat=True)
    key(window, Qt.Key_Space, True, auto_repeat=True)
    assert held_state(node) == (0.0, 0)


def test_window_draws_the_published_frame(rig):
    app, node, _, window = rig
    sent = []
    node._joy_pub.publish = sent.append
    key(window, Qt.Key_A, True)
    node.step(0.0)
    window.refresh()
    assert window.shown_frame is node.last_frame
    assert tuple(sent[-1].axes) == window.shown_frame.axes
    assert tuple(sent[-1].buttons) == window.shown_frame.buttons
    key(window, Qt.Key_A, False)


class _RecordingWindow(TeleopWindow):
    """Connected exactly as in production (a bound method of the window)."""

    seen: list

    def on_status(self, entry):
        self.seen.append(threading.get_ident())
        super().on_status(entry)


def test_status_and_replies_are_queued_to_the_gui_thread(app, ros, uid):
    node = TeleopInputNode(
        parameter_overrides=[
            Parameter("output_topic", value=f"/{uid}/joy"),
            Parameter("status_topics", value=[f"/{uid}/status"]),
        ]
    )
    bridge = RosBridge()
    _RecordingWindow.seen = seen = []
    window = _RecordingWindow(node, bridge)
    entry = StatusEntry(node.config.status_topics[0], "n", 2, "bad", (("k", "v"),), 0.0)
    worker = threading.Thread(target=lambda: bridge.status.emit(entry))
    worker.start()
    worker.join()
    assert seen == []  # queued: nothing ran on the worker thread
    deadline = time.monotonic() + 2.0
    while not seen and time.monotonic() < deadline:
        app.processEvents()
    assert seen == [threading.get_ident()]
    window.refresh()
    top = window.status_tree.topLevelItem(0)
    # received_s 0.0 is long ago on the monotonic clock: shown STALE, not ERROR
    assert "[STALE]" in top.text(0)
    assert (top.child(0).text(0), top.child(0).text(1)) == ("k", "v")
    window.on_status(StatusEntry(entry.topic, "n", 2, "bad", (), time.monotonic()))
    window.refresh()
    assert "[ERROR]" in top.text(0)
    assert top.childCount() == 0
    app.removeEventFilter(window._key_filter)
    window.deleteLater()
    app.processEvents()
    node.destroy_node()


def test_estop_button_fires_on_mouse_down(rig):
    app, node, _, window = rig
    # pressed and never released over the button (dragged off): still one call
    QTest.mousePress(window.estop_button, Qt.LeftButton)
    assert node.actions.step(node.virtual.frame(), 0.0).estop_calls == 1
    QTest.mouseRelease(window.estop_button, Qt.LeftButton)
    assert node.actions.step(node.virtual.frame(), 0.0).estop_calls == 0
    assert window.estop_button.focusPolicy() == Qt.NoFocus


def test_reply_text_is_escaped(rig):
    app, node, _, window = rig
    window.on_service_result("reset", False, "name '<ctrl>' & more")
    window.refresh()
    text = window.reply_label.text()
    assert "refused (name &#x27;&lt;ctrl&gt;&#x27; &amp; more)" in text
    assert "<ctrl>" not in text


def test_qt_timer_rate(rig, uid):
    """The real publish timer: arrivals match the elapsed time at 50 Hz."""
    app, node, _, window = rig
    peer = Node(f"rate_{uid}")
    received = []
    peer.create_subscription(
        Joy, f"/{uid}/joy", lambda m: received.append(time.monotonic()), QoSProfile(depth=1)
    )
    spinner = SpinThread(peer)
    spinner.start()
    try:
        deadline = time.monotonic() + 3.0
        while node.count_subscribers(f"/{uid}/joy") < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        timers = start_timers(node, window)
        QTimer.singleShot(2000, app.quit)
        app.exec_()
        for timer in timers:
            timer.stop()
        time.sleep(0.1)
        assert len(received) > 10
        elapsed = received[-1] - received[0]
        expected = elapsed * node.config.rate_hz
        assert abs(len(received) - 1 - expected) <= 2 + 0.1 * expected
    finally:
        spinner.stop()
        peer.destroy_node()
        QCoreApplication.processEvents()
