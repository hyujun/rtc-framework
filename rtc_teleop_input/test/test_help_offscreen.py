"""The help view and the controller-manager state, offscreen.

F1 (help_key) toggles once per press in both inputs; the pad's combination
toggles through the node's counter; the badges sit on the canvas without
overlapping and the legend lists ``help_entries`` row for row.
"""

from __future__ import annotations

import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"  # forced: a desktop session must not matter

import itertools  # noqa: E402

import pytest  # noqa: E402
from python_qt_binding.QtCore import QEvent, QRectF, Qt  # noqa: E402
from python_qt_binding.QtGui import QKeyEvent  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from rtc_teleop_input.model import SKINS, help_entries  # noqa: E402
from rtc_teleop_input.pad_canvas import PadCanvas  # noqa: E402
from rtc_teleop_input.teleop_ros import TeleopInputNode  # noqa: E402
from rtc_teleop_input.teleop_view import RosBridge, TeleopWindow  # noqa: E402

# a profile-like setup: every role and every stick axis labelled
PARAMS = {
    "reset_controller_name": "ctrl",
    "switch_buttons": ["y:ctrl_a", "a:ctrl_b"],
    "clear_estop_button": "start",
    "clear_estop_reason": "reason",
    "trigger_buttons": ["x:/svc"],
    "key_buttons": [
        "leftshoulder:Space",
        "b:Escape",
        "back:Backspace",
        "y:1",
        "a:2",
        "start:Return",
        "x:R",
    ],
    "help_combo": ["triggerleft", "triggerright"],
    "help_labels": ["lefty:fwd", "leftx:side", "rightx:turn", "righty:height"],
}


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def make_window(app, ros, uid):
    made = []

    def make(**params):
        values = {**PARAMS, "output_topic": f"/{uid}/joy", **params}
        node = TeleopInputNode(
            parameter_overrides=[Parameter(k, value=v) for k, v in values.items()]
        )
        window = TeleopWindow(node, RosBridge())
        window.resize(1000, 600)
        window.show()
        app.processEvents()
        made.append((node, window))
        return node, window

    yield make
    for node, window in made:
        window.close()
        app.removeEventFilter(window._key_filter)
        window.deleteLater()
        app.processEvents()
        node.destroy_node()


def key(window, code, pressed, auto_repeat=False):
    kind = QEvent.KeyPress if pressed else QEvent.KeyRelease
    QApplication.sendEvent(window, QKeyEvent(kind, code, Qt.NoModifier, "", auto_repeat))


@pytest.mark.parametrize("input_", ["virtual", "pad"])
def test_f1_toggles_once_per_press(make_window, uid, input_):
    extra = {"input": "pad", "pad_topic": f"/{uid}/pad"} if input_ == "pad" else {}
    node, window = make_window(**extra)
    assert not window.help_visible
    key(window, Qt.Key_F1, True)
    assert window.help_visible and window.legend.isVisibleTo(window)
    for _ in range(20):  # held: auto-repeat pairs change nothing
        key(window, Qt.Key_F1, False, auto_repeat=True)
        key(window, Qt.Key_F1, True, auto_repeat=True)
    key(window, Qt.Key_F1, False)
    assert window.help_visible
    key(window, Qt.Key_F1, True)
    key(window, Qt.Key_F1, False)
    assert not window.help_visible and not window.legend.isVisibleTo(window)
    # Esc is the E-STOP key: it never closes the help
    key(window, Qt.Key_F1, True)
    key(window, Qt.Key_Escape, True)
    key(window, Qt.Key_Escape, False)
    assert window.help_visible
    assert window.handle_key(int(Qt.Key_F1), False, False)  # consumed, release too


def test_the_pad_combination_toggles_through_refresh(make_window, uid):
    node, window = make_window(input="pad", pad_topic=f"/{uid}/pad")
    node.help_toggles += 1
    window.refresh()
    assert window.help_visible
    node.help_toggles += 2
    window.refresh()
    assert window.help_visible  # two toggles: back to shown
    node.help_toggles += 1
    window.refresh()
    assert not window.help_visible


@pytest.mark.parametrize("input_", ["virtual", "pad"])
def test_legend_lists_the_help_entries(make_window, uid, input_):
    extra = {"input": "pad", "pad_topic": f"/{uid}/pad"} if input_ == "pad" else {}
    node, window = make_window(**extra)
    entries = help_entries(node.config)
    legend = window.legend
    assert legend.topLevelItemCount() == len(entries)
    for number, entry in enumerate(entries, 1):
        item = legend.topLevelItem(number - 1)
        assert item.text(0) == str(number)
        assert item.text(1) == " + ".join(entry.controls)
        assert item.text(2) == entry.function
        if input_ == "virtual":
            assert item.text(3) == entry.keys
    # the pad's keys mean nothing for a real pad: no key column
    assert legend.columnCount() == (4 if input_ == "virtual" else 3)


@pytest.mark.parametrize("skin", SKINS)
def test_badges_are_on_the_canvas_and_apart(make_window, skin):
    node, window = make_window(skin=skin)
    key(window, Qt.Key_F1, True)
    key(window, Qt.Key_F1, False)
    canvas = window.canvas
    for size in ((560, 360), (900, 400), (300, 300)):
        canvas.resize(*size)
        badges = canvas.help_badges()
        # every control of every row has one badge, numbered by its row
        entries = help_entries(node.config)
        expected = [n for n, e in enumerate(entries, 1) for _ in e.controls]
        assert [n for n, _, _ in badges] == expected
        area = QRectF(0, 0, canvas.width(), canvas.height())
        for _, centre, radius in badges:
            box = QRectF(centre.x() - radius, centre.y() - radius, 2 * radius, 2 * radius)
            assert area.contains(box), (size, centre)
        for (_, a, ra), (_, b, rb) in itertools.combinations(badges, 2):
            distance = ((a.x() - b.x()) ** 2 + (a.y() - b.y()) ** 2) ** 0.5
            assert distance >= ra + rb, (size, a, b)


def test_badges_are_drawn_only_with_help_on(make_window):
    _, window = make_window()
    window.toggle_help()
    assert window.canvas.help_badges()
    window.toggle_help()
    assert window.canvas.help_badges() == []
    # the pixels: a canvas of fixed size (the window's layout resizes its own)
    canvas = PadCanvas()
    canvas.resize(560, 360)
    off = canvas.grab().toImage()
    canvas.set_help([(1, ("a",)), (2, ("lefty", "triggerleft"))])
    on = canvas.grab().toImage()
    canvas.set_help([])
    again = canvas.grab().toImage()
    assert on != off
    assert again == off
    canvas.close()


def test_cm_state_is_unknown_until_a_sample(make_window):
    _, window = make_window()
    window.refresh()
    assert "active: unknown" in window.cm_label.text()
    assert "E-STOP: unknown" in window.cm_label.text()
    window.on_cm_state("active", "ctrl_<b>")
    window.on_cm_state("estop", True)
    window.refresh()
    text = window.cm_label.text()
    assert "ctrl_&lt;b&gt;" in text and "LATCHED" in text
    window.on_cm_state("estop", False)
    window.refresh()
    assert "clear" in window.cm_label.text()


def test_clear_reply_is_not_called_a_refusal(make_window):
    _, window = make_window()
    window.on_service_result("clear", False, "lowered, NOT verified")
    window.refresh()
    text = window.reply_label.text()
    assert "NOT confirmed lowered (lowered, NOT verified)" in text
    assert "refused" not in text
