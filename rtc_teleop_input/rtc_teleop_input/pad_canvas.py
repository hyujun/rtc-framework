"""Virtual gamepad drawn with QPainter (Logitech F710 and Xbox skins).

Proportions and control placement follow the controller skins of e7d/gamepad-viewer
(MIT; skins adapted from gamepadviewer.com assets), but nothing is vendored: all shapes
are drawn from code. Mirrors a ``sensor_msgs/Joy`` in the SDL GameController layout.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from python_qt_binding.QtCore import QPointF, QRectF, QSize, Qt, Signal
from python_qt_binding.QtGui import (
    QColor,
    QFont,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
)
from python_qt_binding.QtWidgets import QSizePolicy, QWidget

from rtc_teleop_input.model import DRAWN_BUTTONS, SKINS

__all__ = ["BUTTON_NAMES", "SKINS", "TRIGGER_NAMES", "PadCanvas"]
BUTTON_NAMES = DRAWN_BUTTONS
TRIGGER_NAMES = ("triggerleft", "triggerright")
# Help badges (logical units): radius, and the gap to a stick's base for the
# badge of one of its axes, placed on that axis's side of the stick.
_BADGE_R = 13.0
_BADGE_GAP = 4.0
_AXIS_SIDE = {"leftx": (-1, 0), "lefty": (0, -1), "rightx": (1, 0), "righty": (0, -1)}

# Logical drawing space; everything below is in these units and scaled to the widget.
_W, _H = 800.0, 500.0

_FACE_COLORS = {
    "a": QColor("#3fae49"),
    "b": QColor("#e0352b"),
    "x": QColor("#2b72c8"),
    "y": QColor("#f2b81c"),
}
_HIGHLIGHT = QColor("#ff9f1a")


def _circle(cx: float, cy: float, r: float) -> tuple:
    return ("circle", cx, cy, r, r)


def _rect(cx: float, cy: float, w: float, h: float) -> tuple:
    return ("rect", cx, cy, w, h)


def _face(cx: float, cy: float, off: float = 46.0, r: float = 21.0) -> dict:
    return {
        "a": _circle(cx, cy + off, r),
        "b": _circle(cx + off, cy, r),
        "x": _circle(cx - off, cy, r),
        "y": _circle(cx, cy - off, r),
    }


def _dpad(cx: float, cy: float) -> dict:
    return {
        "dpad_up": _rect(cx, cy - 31, 34, 34),
        "dpad_down": _rect(cx, cy + 31, 34, 34),
        "dpad_left": _rect(cx - 31, cy, 34, 34),
        "dpad_right": _rect(cx + 31, cy, 34, 34),
    }


_TOP = {
    "leftshoulder": _rect(215, 96, 150, 28),
    "rightshoulder": _rect(585, 96, 150, 28),
    "triggerleft": _rect(215, 50, 110, 44),
    "triggerright": _rect(585, 50, 110, 44),
}

# Per skin: buttons (shape tuples), sticks (centre, base radius, travel, head radius).
_GEOM = {
    "f710": {
        "buttons": {
            **_TOP,
            **_face(590, 200),
            **_dpad(225, 205),
            "back": _rect(345, 190, 54, 22),
            "start": _rect(455, 190, 54, 22),
            "guide": _circle(400, 160, 20),
        },
        "sticks": {
            "left": ((295, 330), 54, 38, 34),
            "right": ((505, 330), 54, 38, 34),
        },
        "mode": ((400, 232), 13),
    },
    "xbox": {
        "buttons": {
            **_TOP,
            **_face(590, 205),
            **_dpad(330, 325),
            "back": _circle(350, 205, 14),
            "start": _circle(450, 205, 14),
            "guide": _circle(400, 160, 22),
        },
        "sticks": {
            "left": ((215, 200), 54, 38, 34),
            "right": ((480, 325), 54, 38, 34),
        },
        "mode": None,
    },
}


def _mirror(x: float) -> float:
    return _W - x


def _body_path(skin: str) -> QPainterPath:
    """Outline with grips; the right half is the mirrored left half."""
    p = QPainterPath()
    if skin == "f710":
        right = [
            (180, 114),
            (620, 114),
            ((690, 114), (726, 140), (736, 200)),
            ((752, 300), (772, 400), (742, 458)),
            ((716, 496), (650, 492), (640, 438)),
            ((632, 402), (610, 398), (570, 398)),
            (230, 398),
        ]
    else:
        right = [
            (200, 122),
            (600, 122),
            ((685, 122), (745, 150), (765, 235)),
            ((790, 335), (798, 435), (745, 470)),
            ((702, 496), (655, 458), (630, 428)),
            ((605, 408), (560, 412), (520, 412)),
            (280, 412),
        ]
    p.moveTo(*right[0])
    for seg in right[1:]:
        if len(seg) == 2 and not isinstance(seg[0], tuple):
            p.lineTo(*seg)
        else:
            p.cubicTo(QPointF(*seg[0]), QPointF(*seg[1]), QPointF(*seg[2]))
    # Left half: walk the same curve backwards, mirrored.
    pts = [right[0]]
    for seg in right[1:]:
        pts.append(seg[-1] if isinstance(seg[0], tuple) else seg)
    starts = pts[:-1]
    for i in range(len(right) - 1, 0, -1):
        seg = right[i]
        end = starts[i - 1]
        if isinstance(seg[0], tuple):
            p.cubicTo(
                QPointF(_mirror(seg[1][0]), seg[1][1]),
                QPointF(_mirror(seg[0][0]), seg[0][1]),
                QPointF(_mirror(end[0]), end[1]),
            )
        else:
            p.lineTo(_mirror(end[0]), end[1])
    p.closeSubpath()
    return p


class PadCanvas(QWidget):
    """Mouse-drivable gamepad picture; geometry tables are shared by paint and hit test."""

    stick_dragged = Signal(str, float, float)
    stick_released = Signal(str)
    button_pressed = Signal(str)
    button_released = Signal(str)

    def __init__(self, skin: str = "f710", parent=None):
        super().__init__(parent)
        if skin not in SKINS:
            raise ValueError(f"unknown skin {skin!r}; expected one of {SKINS}")
        self._skin = skin
        self._axes: dict[str, float] = {}
        self._buttons: dict[str, int] = {}
        self._interactive = True
        self._active: tuple[str, str] | None = None  # ("stick"|"button", name)
        self._help: list[tuple[int, tuple[str, ...]]] = []
        # The main window owns the keyboard.
        self.setFocusPolicy(Qt.NoFocus)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(280, 180)

    def sizeHint(self):
        return QSize(560, 360)

    # -- configuration -------------------------------------------------------------
    def set_skin(self, skin: str) -> None:
        if skin not in SKINS:
            raise ValueError(f"unknown skin {skin!r}; expected one of {SKINS}")
        if skin != self._skin:
            self._skin = skin
            self.cancel_interaction()
            self.update()

    def set_interactive(self, on: bool) -> None:
        self._interactive = bool(on)
        if not on:
            self._active = None

    def cancel_interaction(self) -> None:
        self._active = None

    def set_state(self, axes: Mapping[str, float], buttons: Mapping[str, int]) -> None:
        new_axes = {
            k: float(axes.get(k, 0.0))
            for k in ("leftx", "lefty", "rightx", "righty", "triggerleft", "triggerright")
        }
        new_buttons = {k: 1 if buttons.get(k, 0) else 0 for k in BUTTON_NAMES}
        if new_axes != self._axes or new_buttons != self._buttons:
            self._axes, self._buttons = new_axes, new_buttons
            self.update()

    def set_help(self, badges: Sequence[tuple[int, Sequence[str]]]) -> None:
        """Numbered badges on controls (layout names); empty hides them."""
        new = [(number, tuple(controls)) for number, controls in badges]
        if new != self._help:
            self._help = new
            self.update()

    def help_badges(self) -> list[tuple[int, QPointF, float]]:
        """(number, centre, radius) of every badge drawn, in widget pixels."""
        s = self._scale()[0]
        return [
            (number, self._to_widget(x, y), _BADGE_R * s)
            for number, (x, y) in self._badge_layout()
        ]

    def _badge_anchor(self, name: str) -> tuple[float, float]:
        g = _GEOM[self._skin]
        side = _AXIS_SIDE.get(name)
        if side is not None:
            (cx, cy), base_r, _, _ = g["sticks"]["left" if name.startswith("left") else "right"]
            reach = base_r + _BADGE_GAP + _BADGE_R
            return cx + side[0] * reach, cy + side[1] * reach
        if name in ("leftstick", "rightstick"):
            return g["sticks"]["left" if name == "leftstick" else "right"][0]
        _, cx, cy, _, _ = g["buttons"][name]
        return cx, cy

    def _badge_layout(self) -> list[tuple[int, tuple[float, float]]]:
        """Logical badge centres; a second badge on one control goes beside the first."""
        placed: list[tuple[int, tuple[float, float]]] = []
        count: dict[str, int] = {}
        for number, controls in self._help:
            for name in controls:
                x, y = self._badge_anchor(name)
                k = count.get(name, 0)
                count[name] = k + 1
                placed.append((number, (x + k * (2 * _BADGE_R + 2), y)))
        return placed

    # -- geometry ------------------------------------------------------------------
    def _scale(self) -> tuple[float, float, float]:
        s = min(self.width() / _W, self.height() / _H)
        return s, (self.width() - _W * s) / 2, (self.height() - _H * s) / 2

    def _to_widget(self, x: float, y: float) -> QPointF:
        s, ox, oy = self._scale()
        return QPointF(ox + x * s, oy + y * s)

    def _to_logical(self, pos: QPointF) -> tuple[float, float]:
        s, ox, oy = self._scale()
        return (pos.x() - ox) / s, (pos.y() - oy) / s

    def _head_offset(self, side: str) -> tuple[float, float]:
        """Head offset in logical units; Joy + is left/up, the screen is +right/+down."""
        _, _, travel, _ = _GEOM[self._skin]["sticks"][side]
        ax = self._axes.get(f"{side}x", 0.0)
        ay = self._axes.get(f"{side}y", 0.0)
        dx, dy = -ax, -ay
        n = math.hypot(dx, dy)
        if n > 1.0:
            dx, dy = dx / n, dy / n
        return dx * travel, dy * travel

    def stick_head_center(self, side: str) -> QPointF:
        (cx, cy), _, _, _ = _GEOM[self._skin]["sticks"][side]
        ox, oy = self._head_offset(side)
        return self._to_widget(cx + ox, cy + oy)

    def stick_travel_radius(self) -> float:
        """Drawn travel radius in widget pixels (same for both sticks)."""
        return _GEOM[self._skin]["sticks"]["left"][2] * self._scale()[0]

    def control_center(self, name: str) -> QPointF:
        """Centre of a control in widget coordinates; leftstick/rightstick give the stick."""
        g = _GEOM[self._skin]
        if name in ("stick:left", "stick:right", "leftstick", "rightstick"):
            side = "left" if "left" in name else "right"
            return self._to_widget(*g["sticks"][side][0])
        _, cx, cy, _, _ = g["buttons"][name]
        return self._to_widget(cx, cy)

    def control_at(self, pos: QPointF) -> str | None:
        x, y = self._to_logical(pos)
        g = _GEOM[self._skin]
        for side, ((cx, cy), base_r, _, head_r) in g["sticks"].items():
            hx, hy = self._head_offset(side)
            if (
                math.hypot(x - cx, y - cy) <= base_r
                or math.hypot(x - cx - hx, y - cy - hy) <= head_r
            ):
                return f"stick:{side}"
        for name, (kind, cx, cy, w, h) in g["buttons"].items():
            if kind == "circle":
                if math.hypot(x - cx, y - cy) <= w:
                    return name
            elif abs(x - cx) <= w / 2 and abs(y - cy) <= h / 2:
                return name
        return None

    # -- mouse ---------------------------------------------------------------------
    def _stick_value(self, side: str, pos: QPointF) -> tuple[float, float]:
        (cx, cy), _, travel, _ = _GEOM[self._skin]["sticks"][side]
        x, y = self._to_logical(pos)
        right, up = (x - cx) / travel, -(y - cy) / travel
        n = math.hypot(right, up)
        if n > 1.0:
            right, up = right / n, up / n
        return right, up

    def mousePressEvent(self, event):
        if not self._interactive or event.button() != Qt.LeftButton or self._active:
            return
        hit = self.control_at(event.localPos())
        if hit is None:
            return
        if hit.startswith("stick:"):
            side = hit[6:]
            self._active = ("stick", side)
            self.stick_dragged.emit(side, *self._stick_value(side, event.localPos()))
        else:
            self._active = ("button", hit)
            self.button_pressed.emit(hit)

    def mouseMoveEvent(self, event):
        if self._interactive and self._active and self._active[0] == "stick":
            side = self._active[1]
            self.stick_dragged.emit(side, *self._stick_value(side, event.localPos()))

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.LeftButton or not self._active:
            return
        kind, name = self._active
        self._active = None
        if not self._interactive:
            return
        (self.stick_released if kind == "stick" else self.button_released).emit(name)

    # -- painting ------------------------------------------------------------------
    def _pressed(self, name: str) -> bool:
        return bool(self._buttons.get(name))

    def _begin(self, p, name: str, base: QColor, edge: QColor) -> bool:
        """Set pen/brush for a control; pressed = lighter fill, amber ring, pushed in."""
        on = self._pressed(name)
        if on:
            p.save()
            p.translate(0, 2)
            p.setPen(QPen(_HIGHLIGHT, 3.5))
            p.setBrush(base.lighter(140))
        else:
            p.setPen(QPen(edge, 2))
            p.setBrush(base)
        return on

    @staticmethod
    def _end(p, on: bool) -> None:
        if on:
            p.restore()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        p.fillRect(self.rect(), QColor("#dfe2e6"))
        s, ox, oy = self._scale()
        p.translate(ox, oy)
        p.scale(s, s)
        xbox = self._skin == "xbox"
        g = _GEOM[self._skin]
        edge = QColor("#8a8d93") if xbox else QColor("#14151a")
        part = QColor("#cfd1d6") if xbox else QColor("#44474d")
        text = QColor("#40434a") if xbox else QColor("#e6e7ea")

        self._paint_top(p, g, xbox, part, edge)
        self._paint_body(p, xbox, edge)
        self._paint_dpad(p, g, edge)
        self._paint_face(p, g, xbox, edge)
        self._paint_center(p, g, xbox, edge, text)
        for side in ("left", "right"):
            self._paint_stick(p, g, side, xbox, edge)
        self._paint_help(p)
        p.end()

    def _paint_help(self, p):
        for number, (x, y) in self._badge_layout():
            p.setPen(QPen(QColor("#ffffff"), 2))
            p.setBrush(QColor("#1565c0"))
            p.drawEllipse(QPointF(x, y), _BADGE_R, _BADGE_R)
            p.setPen(QColor("#ffffff"))
            p.setFont(self._font(14, bold=True))
            box = QRectF(x - _BADGE_R, y - _BADGE_R, 2 * _BADGE_R, 2 * _BADGE_R)
            p.drawText(box, Qt.AlignCenter, str(number))

    def _paint_body(self, p, xbox, edge):
        body = _body_path(self._skin)
        top = QColor("#fbfbfc") if xbox else QColor("#3a3d42")
        bottom = QColor("#d9dadd") if xbox else QColor("#222428")
        grad = QLinearGradient(0, 110, 0, 490)
        grad.setColorAt(0, top)
        grad.setColorAt(1, bottom)
        p.setPen(QPen(edge, 3))
        p.setBrush(grad)
        p.drawPath(body)
        # Darker grip shading so the handles read as separate volumes.
        p.save()
        p.setClipPath(body)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 45 if not xbox else 28))
        for cx in (700, _mirror(700)):
            p.save()
            p.translate(cx, 420)
            p.rotate(-8 if cx > 400 else 8)
            p.drawEllipse(QPointF(0, 0), 62, 105)
            p.restore()
        p.restore()
        p.setPen(QPen(QColor(255, 255, 255, 120 if xbox else 35), 2))
        p.setBrush(Qt.NoBrush)
        p.drawPath(body.translated(0, 3))

    def _bumper_path(self) -> QPainterPath:
        # Left bumper centred on (215, 96); the right one is drawn mirrored.
        _, cx, cy, w, h = _GEOM[self._skin]["buttons"]["leftshoulder"]
        x0, x1, y = cx - w / 2, cx + w / 2, cy
        q = QPainterPath()
        q.moveTo(x0, y + 14)
        q.quadTo(x0 + 2, y - 14, x0 + 34, y - 14)
        q.lineTo(x1 - 6, y - 14)
        q.quadTo(x1, y - 14, x1, y - 6)
        q.lineTo(x1, y + 14)
        q.quadTo((x0 + x1) / 2, y + 6, x0, y + 14)
        return q

    def _paint_top(self, p, g, xbox, part, edge):
        for name, label in (("leftshoulder", "LB"), ("rightshoulder", "RB")):
            _, cx, cy, w, h = g["buttons"][name]
            right = name == "rightshoulder"
            p.save()
            if right:
                p.translate(_W, 0)
                p.scale(-1, 1)
            on = self._begin(p, name, part, edge)
            p.drawPath(self._bumper_path())
            self._end(p, on)
            p.restore()
            p.setPen(QColor("#40434a") if xbox else QColor("#e6e7ea"))
            p.setFont(self._font(14, bold=True))
            p.drawText(
                QRectF(cx - w / 2, cy - h / 2 + (2 if on else 0), w, h), Qt.AlignCenter, label
            )
        for name in ("triggerleft", "triggerright"):
            _, cx, cy, w, h = g["buttons"][name]
            r = QRectF(cx - w / 2, cy - h / 2, w, h)
            p.setPen(QPen(edge, 2))
            p.setBrush(QColor("#b9bcc2"))
            p.drawRoundedRect(r, 10, 10)
            frac = max(0.0, min(1.0, -self._axes.get(name, 0.0)))
            if frac > 0:
                p.save()
                clip = QPainterPath()
                clip.addRoundedRect(r, 10, 10)
                p.setClipPath(clip)
                p.setPen(Qt.NoPen)
                p.setBrush(_HIGHLIGHT)
                p.drawRect(QRectF(r.left(), r.bottom() - h * frac, w, h * frac))
                p.restore()
            p.setPen(QColor("#24262b"))
            p.setFont(self._font(14, bold=True))
            p.drawText(r, Qt.AlignCenter, "LT" if name == "triggerleft" else "RT")

    def _paint_dpad(self, p, g, edge):
        b = g["buttons"]
        _, cx, cy, _, _ = b["dpad_up"]
        cy += 31  # pad centre
        f710 = self._skin == "f710"
        p.setPen(QPen(edge, 2))
        p.setBrush(QColor("#1b1c20") if f710 else QColor("#d8dade"))
        p.drawEllipse(QPointF(cx, cy), 64, 64)
        arrows = {
            "dpad_up": (0, -1),
            "dpad_down": (0, 1),
            "dpad_left": (-1, 0),
            "dpad_right": (1, 0),
        }
        for name, (dx, dy) in arrows.items():
            _, bx, by, w, h = b[name]
            on = self._begin(p, name, QColor("#55585f") if f710 else QColor("#4a4d54"), edge)
            p.drawRoundedRect(QRectF(bx - w / 2, by - h / 2, w, h), 5, 5)
            tri = QPainterPath()
            px, py = -dy, dx
            tri.moveTo(bx + dx * 10, by + dy * 10)
            tri.lineTo(bx - dx * 4 + px * 9, by - dy * 4 + py * 9)
            tri.lineTo(bx - dx * 4 - px * 9, by - dy * 4 - py * 9)
            tri.closeSubpath()
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("#e6e7ea"))
            p.drawPath(tri)
            self._end(p, on)
        p.setPen(QPen(edge, 2))
        p.setBrush(QColor("#33353b"))
        p.drawEllipse(QPointF(cx, cy), 9, 9)

    def _paint_face(self, p, g, xbox, edge):
        for name in ("a", "b", "x", "y"):
            _, cx, cy, r, _ = g["buttons"][name]
            col = _FACE_COLORS[name]
            on = self._begin(p, name, QColor("#2a2c31") if xbox else col, edge)
            p.drawEllipse(QPointF(cx, cy), r, r)
            p.setPen(col.lighter(120) if xbox else QColor("#ffffff"))
            p.setFont(self._font(20, bold=True))
            p.drawText(QRectF(cx - r, cy - r, 2 * r, 2 * r), Qt.AlignCenter, name.upper())
            self._end(p, on)

    def _paint_center(self, p, g, xbox, edge, text):
        b = g["buttons"]
        dark = QColor("#4b4e55")
        if xbox:
            for name in ("back", "start"):
                _, cx, cy, r, _ = b[name]
                on = self._begin(p, name, dark, edge)
                p.drawEllipse(QPointF(cx, cy), r, r)
                p.setPen(QPen(QColor("#f0f0f0"), 1.8))
                p.setBrush(Qt.NoBrush)
                if name == "back":  # view glyph: two overlapping squares
                    p.drawRect(QRectF(cx - 7, cy - 4, 9, 9))
                    p.drawRect(QRectF(cx - 3, cy - 7, 9, 9))
                else:  # menu glyph: three bars
                    for dy in (-5, 0, 5):
                        p.drawLine(QPointF(cx - 7, cy + dy), QPointF(cx + 7, cy + dy))
                self._end(p, on)
            _, cx, cy, r, _ = b["guide"]
            on = self._begin(p, "guide", QColor("#e9e9ea"), edge)
            p.drawEllipse(QPointF(cx, cy), r, r)
            p.setPen(QPen(QColor("#2a2c31"), 2.2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), r - 6, r - 6)
            p.drawLine(QPointF(cx - 9, cy - 9), QPointF(cx + 9, cy + 9))
            p.drawLine(QPointF(cx - 9, cy + 9), QPointF(cx + 9, cy - 9))
            self._end(p, on)
            return
        for name, label in (("back", "BACK"), ("start", "START")):
            _, cx, cy, w, h = b[name]
            r = QRectF(cx - w / 2, cy - h / 2, w, h)
            on = self._begin(p, name, dark, edge)
            p.drawRoundedRect(r, 10, 10)
            p.setPen(text)
            p.setFont(self._font(9, bold=True))
            p.drawText(r, Qt.AlignCenter, label)
            self._end(p, on)
        _, cx, cy, r, _ = b["guide"]
        on = self._begin(p, "guide", QColor("#8bc34a"), edge)
        p.drawEllipse(QPointF(cx, cy), r, r)
        p.setPen(QColor("#1d2b0e"))
        p.setFont(self._font(15, bold=True))
        p.drawText(QRectF(cx - r, cy - r, 2 * r, 2 * r), Qt.AlignCenter, "L")
        self._end(p, on)
        # MODE button and LED are decorative.
        (mx, my), mr = g["mode"]
        p.setPen(QPen(edge, 2))
        p.setBrush(dark)
        p.drawEllipse(QPointF(mx, my), mr, mr)
        p.setPen(text)
        p.setFont(self._font(7, bold=True))
        p.drawText(QRectF(mx - 20, my + mr, 40, 12), Qt.AlignCenter, "MODE")
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#5bd05b"))
        p.drawEllipse(QPointF(mx + 26, my), 4, 4)

    def _paint_stick(self, p, g, side, xbox, edge):
        (cx, cy), base_r, _, head_r = g["sticks"][side]
        p.setPen(QPen(edge, 2))
        p.setBrush(QColor("#1b1c20") if not xbox else QColor("#c9ccd1"))
        p.drawEllipse(QPointF(cx, cy), base_r, base_r)
        ox, oy = self._head_offset(side)
        on = self._begin(p, f"{side}stick", QColor("#4a4d54"), edge)
        hc = QPointF(cx + ox, cy + oy)
        p.drawEllipse(hc, head_r, head_r)
        p.setPen(QPen(QColor(255, 255, 255, 70), 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(hc, head_r - 8, head_r - 8)
        self._end(p, on)

    @staticmethod
    def _font(px: int, bold: bool = False) -> QFont:
        f = QFont()
        f.setPixelSize(px)
        f.setBold(bold)
        return f
