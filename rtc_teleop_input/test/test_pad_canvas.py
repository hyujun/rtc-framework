import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"

import pytest  # noqa: E402
from python_qt_binding.QtCore import QEvent, QPointF, Qt  # noqa: E402
from python_qt_binding.QtGui import QMouseEvent  # noqa: E402
from python_qt_binding.QtTest import QTest  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402

from rtc_teleop_input.pad_canvas import (  # noqa: E402
    BUTTON_NAMES,
    SKINS,
    TRIGGER_NAMES,
    PadCanvas,
)


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def pad(app):
    w = PadCanvas()
    w.resize(700, 450)
    yield w
    w.close()


class Spy:
    def __init__(self, pad):
        self.events = []
        pad.stick_dragged.connect(lambda s, x, y: self.events.append(("drag", s, x, y)))
        pad.stick_released.connect(lambda s: self.events.append(("srel", s)))
        pad.button_pressed.connect(lambda n: self.events.append(("press", n)))
        pad.button_released.connect(lambda n: self.events.append(("brel", n)))


def _move(pad, pos):
    ev = QMouseEvent(QEvent.MouseMove, pos, Qt.NoButton, Qt.LeftButton, Qt.NoModifier)
    QApplication.sendEvent(pad, ev)


def _press(pad, pos):
    QTest.mousePress(pad, Qt.LeftButton, Qt.NoModifier, pos.toPoint())


def _release(pad, pos):
    QTest.mouseRelease(pad, Qt.LeftButton, Qt.NoModifier, pos.toPoint())


@pytest.mark.parametrize("skin", SKINS)
def test_every_control_hit_testable(pad, skin):
    pad.set_skin(skin)
    for name in BUTTON_NAMES + TRIGGER_NAMES:
        hit = pad.control_at(pad.control_center(name))
        if name in ("leftstick", "rightstick"):
            assert hit == "stick:" + name[:-5]
        else:
            assert hit == name, (skin, name)
    assert pad.control_at(pad.control_center("stick:left")) == "stick:left"
    assert pad.control_at(pad.control_center("stick:right")) == "stick:right"
    assert pad.control_at(QPointF(1, 1)) is None


def test_drag_and_clamp(pad):
    spy = Spy(pad)
    c = pad.control_center("stick:left")
    r = pad.stick_travel_radius()
    _press(pad, c)
    assert spy.events[-1][:2] == ("drag", "left")
    _move(pad, c + QPointF(r, 0))
    assert spy.events[-1][:2] == ("drag", "left")
    assert spy.events[-1][2:] == pytest.approx((1.0, 0.0))
    _move(pad, c + QPointF(2 * r, 0))
    assert spy.events[-1][2] == pytest.approx(1.0)
    _move(pad, c + QPointF(2 * r, -2 * r))
    _, _, x, y = spy.events[-1]
    assert x**2 + y**2 == pytest.approx(1.0)
    _move(pad, c + QPointF(0, -r))
    assert spy.events[-1][3] == pytest.approx(1.0)
    _release(pad, c)
    assert [e for e in spy.events if e[0] == "srel"] == [("srel", "left")]


def test_button_press_release(pad):
    spy = Spy(pad)
    c = pad.control_center("b")
    _press(pad, c)
    _release(pad, c)
    assert spy.events == [("press", "b"), ("brel", "b")]


def test_release_outside_widget(pad):
    spy = Spy(pad)
    _press(pad, pad.control_center("b"))
    _release(pad, QPointF(-50, -50))
    assert spy.events == [("press", "b"), ("brel", "b")]


def test_not_interactive_emits_nothing(pad):
    spy = Spy(pad)
    pad.set_interactive(False)
    for name in ("b", "stick:left"):
        c = pad.control_center(name)
        _press(pad, c)
        _move(pad, c + QPointF(10, 0))
        _release(pad, c)
    assert spy.events == []


def test_cancel_interaction(pad):
    spy = Spy(pad)
    c = pad.control_center("stick:right")
    _press(pad, c)
    n = len(spy.events)
    pad.cancel_interaction()
    _release(pad, c)
    assert len(spy.events) == n


def test_bad_skin(pad):
    with pytest.raises(ValueError):
        pad.set_skin("bad")
    with pytest.raises(ValueError):
        PadCanvas("bad")


def test_no_focus(pad):
    assert pad.focusPolicy() == Qt.NoFocus


@pytest.mark.parametrize("skin", SKINS)
def test_joy_sign_convention(pad, skin):
    pad.set_skin(skin)
    base = pad.control_center("stick:left")
    pad.set_state({"leftx": 1.0}, {})
    assert pad.stick_head_center("left").x() < base.x()
    pad.set_state({"lefty": 1.0}, {})
    head = pad.stick_head_center("left")
    assert head.y() < base.y()
    assert head.x() == pytest.approx(base.x())
