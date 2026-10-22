"""candump ``-L`` log reader for the CAN FD hand bus (epic #895).

Parses the one-line-per-frame format ``candump -L`` (and the ``-l`` file) writes::

    (1697000000.123456) can0 00404000##1
    (1697000000.123900) can0 00800040##1A0B1C2D3
    (1697000000.124000) can0 20000004#0000000000000000    <- SocketCAN error frame
    (1697000000.125000) can0 123#DEADBEEF                  <- classic frame

and classifies each frame against the hand protocol id layout (Proto §2, as
``udp_hand_driver/include/udp_hand_driver/hand_canfd_packets.hpp`` encodes it)::

    priority[28:26] · msg_type[25:22] · target[21:14] · source[13:6] · frame_idx[5:0]

The host is node 0; the right hand is node 1 and the left hand node 2 (D51).
The id arithmetic is re-derived here rather than copied as a table of
constants: a reader that has to be told its ids cannot tell a left hand from a
right one, and the two-hand gate (G-5) needs exactly that.

Timestamps are candump's: the socket's receive time, which for a frame this
host transmitted is the TX-done stamp of the loopback copy. Round trips
measured here are therefore a LOWER bound on what the node experiences.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

# SocketCAN flag bits as they appear in the 8-hex-digit id of candump -L.
EFF_MASK = 0x1FFFFFFF
ERR_FLAG = 0x20000000

NODE_HOST = 0x00
NODE_RIGHT_HAND = 0x01
NODE_LEFT_HAND = 0x02

_PRIORITY_SHIFT = 26
_MSG_TYPE_SHIFT = 22
_TARGET_SHIFT = 14
_SOURCE_SHIFT = 6
_FRAME_INDEX_MASK = 0x3F

_MSG_CMD = 0x0
_MSG_REQ = 0x1
_MSG_RESP = 0x2

# `(sec.usec) iface id#payload` — the payload part keeps the leading `#` of an
# FD frame (`##<flags><hex>`) so the two forms can be told apart afterwards.
_LINE_RE = re.compile(r"^\((\d+)\.(\d+)\)\s+(\S+)\s+([0-9A-Fa-f]{3,8})#(.*)$")


def node_id_of(hand_side: str) -> int:
    """Node id of a hand from the `hand_side` parameter spelling (D51)."""
    if hand_side == "right":
        return NODE_RIGHT_HAND
    if hand_side == "left":
        return NODE_LEFT_HAND
    raise ValueError(f"hand_side must be 'right' or 'left', got {hand_side!r}")


def make_can_id(priority: int, msg_type: int, target: int, source: int, frame_idx: int) -> int:
    return (
        ((priority & 0x07) << _PRIORITY_SHIFT)
        | ((msg_type & 0x0F) << _MSG_TYPE_SHIFT)
        | ((target & 0xFF) << _TARGET_SHIFT)
        | ((source & 0xFF) << _SOURCE_SHIFT)
        | (frame_idx & _FRAME_INDEX_MASK)
    )


def cmd_id(node: int, frame_idx: int) -> int:
    """host → `node`: command frame."""
    return make_can_id(0, _MSG_CMD, node, NODE_HOST, frame_idx)


def echo_id(node: int, frame_idx: int) -> int:
    """`node` → host: echo of a command frame (same payload, D41)."""
    return make_can_id(0, _MSG_CMD, NODE_HOST, node, frame_idx)


def req_id(node: int) -> int:
    """host → `node`: telemetry request (DLC 0, D47)."""
    return make_can_id(0, _MSG_REQ, node, NODE_HOST, 0)


def resp_id(node: int, frame_idx: int) -> int:
    """`node` → host: telemetry response frame 0, 1, 2."""
    return make_can_id(0, _MSG_RESP, NODE_HOST, node, frame_idx)


class Kind(StrEnum):
    CMD = "cmd"
    ECHO = "echo"
    REQ = "req"
    RESP = "resp"
    ERROR = "error"
    OTHER = "other"


@dataclass(frozen=True)
class Frame:
    t: float
    interface: str
    can_id: int  # 29-bit identifier, flag bits stripped (meaningless when is_error)
    is_error: bool
    fd: bool
    data: bytes
    line_no: int

    @property
    def frame_idx(self) -> int:
        return self.can_id & _FRAME_INDEX_MASK

    @property
    def all_zero(self) -> bool:
        """A payload that is present and entirely zero — DLC 0 is not 'zero'."""
        return bool(self.data) and not any(self.data)


def parse_candump_line(line: str, line_no: int = 0) -> Frame | None:
    """One frame, or None for a line that is not a frame (blank, drop notice, …)."""
    m = _LINE_RE.match(line.strip())
    if m is None:
        return None
    sec, frac, iface, id_hex, payload = m.groups()
    t = int(sec) + int(frac) / (10 ** len(frac))
    raw_id = int(id_hex, 16)
    is_error = len(id_hex) == 8 and (raw_id & ERR_FLAG) != 0
    fd = payload.startswith("#")
    if fd:
        # `##<flags nibble><data hex>`; the flags (BRS, ESI) are not kept.
        hex_data = payload[2:]
    elif payload.startswith("R"):
        hex_data = ""  # classic RTR frame carries no data
    else:
        hex_data = payload
    hex_data = hex_data.strip()
    if len(hex_data) % 2 != 0:
        return None
    try:
        data = bytes.fromhex(hex_data) if hex_data else b""
    except ValueError:
        return None
    return Frame(
        t=t,
        interface=iface,
        can_id=raw_id & EFF_MASK,
        is_error=is_error,
        fd=fd,
        data=data,
        line_no=line_no,
    )


def read_candump(path: Path) -> tuple[list[Frame], int]:
    """Every frame in file order plus the count of non-frame lines."""
    frames: list[Frame] = []
    unparsed = 0
    with path.open(encoding="utf-8", errors="replace") as fh:
        for line_no, line in enumerate(fh, start=1):
            if not line.strip():
                continue
            frame = parse_candump_line(line, line_no)
            if frame is None:
                unparsed += 1
            else:
                frames.append(frame)
    return frames, unparsed


def classify(frame: Frame, node: int) -> Kind:
    """Which hand-protocol frame this is, for the hand at `node`.

    Frames of another node (a second hand on the same bus) are OTHER, so the
    per-hand analyses stay per hand.
    """
    if frame.is_error:
        return Kind.ERROR
    base = frame.can_id & ~_FRAME_INDEX_MASK
    if base == cmd_id(node, 0):
        return Kind.CMD
    if base == echo_id(node, 0):
        return Kind.ECHO
    if frame.can_id == req_id(node):
        return Kind.REQ
    if base == resp_id(node, 0):
        return Kind.RESP
    return Kind.OTHER
