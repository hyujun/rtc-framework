"""SIGINT / SIGTERM close joint_command_gui's window — no DDS (rclpy.init is faked).

rclpy's default handlers shut the context down on SIGTERM and leave the Qt
loop running: the window stayed open with a dead node until SIGKILL.
"""

from __future__ import annotations

import os
import signal
import threading

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding.QtCore import QTimer  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402

from rtc_tools.gui import joint_command_gui  # noqa: E402

LIMIT_S = 5.0


@pytest.fixture
def app():
    saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    yield QApplication.instance() or QApplication([])
    for signum, handler in saved.items():
        signal.signal(signum, handler)


def _closes(app) -> bool:
    """Runs the loop with a Python tick (the tools' timers); True if it quit before LIMIT_S."""
    timed_out = []
    tick = QTimer()
    tick.timeout.connect(lambda: None)
    tick.start(20)
    limit = QTimer()
    limit.setSingleShot(True)
    limit.timeout.connect(lambda: (timed_out.append(True), app.quit()))
    limit.start(int(LIMIT_S * 1000))
    app.exec_()
    tick.stop()
    limit.stop()
    return not timed_out


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_a_signal_closes_the_running_window(app, signum):
    joint_command_gui.quit_on_signals(app)
    sender = threading.Timer(0.1, os.kill, (os.getpid(), signum))
    sender.start()
    assert _closes(app)
    sender.join()


def test_a_signal_before_the_loop_starts_still_closes_it(app):
    joint_command_gui.quit_on_signals(app)
    os.kill(os.getpid(), signal.SIGTERM)  # the handler runs here, before exec_
    assert _closes(app)


def test_rclpy_installs_no_signal_handlers(monkeypatch):
    seen = {}

    class Stop(Exception):
        pass

    def fake_init(**kwargs):
        seen.update(kwargs)
        raise Stop

    monkeypatch.setattr(joint_command_gui.rclpy, "init", fake_init)
    with pytest.raises(Stop):
        joint_command_gui.main([])
    assert seen.get("signal_handler_options") is SignalHandlerOptions.NO
