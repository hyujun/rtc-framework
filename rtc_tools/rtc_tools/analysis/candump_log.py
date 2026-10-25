"""candump ``-L`` log reader for the CAN FD hand bus (epic #895).

Parses the one-line-per-frame format ``candump -L`` (and the ``-l`` file) writes::

    (1697000000.123456) can0 00404000##1
    (1697000000.123900) can0 00800040##1A0B1C2D3
    (1697000000.124000) can0 20000004#0000000000000000    <- SocketCAN error frame
    (1697000000.125000) can0 123#DEADBEEF                  <- classic frame

and classifies each frame against the hand protocol id layout (Proto §2, as
``udp_hand_driver/include/udp_hand_driver/hand_canfd_packets.hpp`` encodes it)::

    priority[28:26] · msg_type[25:22] · target[21:14] · source[13:6] · frame_idx[5:0]

The host is node 0; the right hand is node 1 and the left hand node 2 (D51);
target 0xFF is every node. The one broadcast frame the host sends is the REQ
(``req_mode: broadcast``, #956): every hand answers it with its own RESP set,
so ``classify(..., broadcast_req=True)`` reads it as each hand's REQ. A
broadcast CMD is never sent (the hand side writes it to flash) and is never a
CMD here — its target is no hand's node id.
The id arithmetic is re-derived here rather than copied as a table of
constants: a reader that has to be told its ids cannot tell a left hand from a
right one, and the two-hand gate (G-5) needs exactly that.

Timestamps are candump's: the socket's receive time, which for a frame this
host transmitted is the TX-done stamp of the loopback copy. Round trips
measured here are therefore a LOWER bound on what the node experiences.

``python3 -m rtc_tools.analysis.candump_log <iface> <out.log>`` records the same
``-L`` lines from a raw CAN FD socket (kernel receive stamps) for a PC without
can-utils — the development PC's vcan0 SIL; the rig has candump. It stops on
SIGINT or SIGTERM and flushes every line.
"""

from __future__ import annotations

import re
import signal
import socket
import struct
import sys
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

# SocketCAN flag bits as they appear in the 8-hex-digit id of candump -L.
EFF_MASK = 0x1FFFFFFF
ERR_FLAG = 0x20000000

NODE_HOST = 0x00
NODE_RIGHT_HAND = 0x01
NODE_LEFT_HAND = 0x02
NODE_BROADCAST = 0xFF

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


def broadcast_req_id() -> int:
    """host → every hand: telemetry request (``req_mode: broadcast``), 0x007FC000."""
    return make_can_id(0, _MSG_REQ, NODE_BROADCAST, NODE_HOST, 0)


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


def classify(frame: Frame, node: int, *, broadcast_req: bool = False) -> Kind:
    """Which hand-protocol frame this is, for the hand at `node`.

    Frames of another node (a second hand on the same bus) are OTHER, so the
    per-hand analyses stay per hand. With `broadcast_req` the broadcast REQ is
    this hand's REQ too (every hand answers it); without, it is OTHER.
    """
    if frame.is_error:
        return Kind.ERROR
    if broadcast_req and frame.can_id == broadcast_req_id():
        return Kind.REQ
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


# ── recorder (a candump -L stand-in) ────────────────────────────────────────

_CAN_EFF_FLAG = 0x80000000
_CAN_ERR_FLAG = 0x20000000
_CAN_SFF_MASK = 0x7FF
_CAN_ERR_MASK = 0x1FFFFFFF
_CANFD_MTU = 72
# linux/can/raw.h; the socket module does not export it.
_CAN_RAW_ERR_FILTER = getattr(socket, "CAN_RAW_ERR_FILTER", 2)
# asm-generic/socket.h (SO_TIMESTAMP_OLD: struct timeval); not exported either.
_SO_TIMESTAMP = getattr(socket, "SO_TIMESTAMP", 29)


def format_candump_line(
    t: float, interface: str, can_id: int, data: bytes, fd: bool, flags: int = 0
) -> str:
    """One ``candump -L`` line for a frame as SocketCAN delivers it (flag bits in `can_id`)."""
    sec = int(t)
    usec = int(round((t - sec) * 1e6))
    if usec >= 1_000_000:
        sec, usec = sec + 1, usec - 1_000_000
    if can_id & _CAN_ERR_FLAG:
        id_hex = f"{(can_id & EFF_MASK) | ERR_FLAG:08X}"
    elif can_id & _CAN_EFF_FLAG:
        id_hex = f"{can_id & EFF_MASK:08X}"
    else:
        id_hex = f"{can_id & _CAN_SFF_MASK:03X}"
    body = f"#{flags & 0xF:X}{data.hex().upper()}" if fd else data.hex().upper()
    return f"({sec}.{usec:06d}) {interface} {id_hex}#{body}"


def record(interface: str, path: Path) -> int:
    """Write every frame on `interface` to `path` until SIGINT / SIGTERM; frame count."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    sock.setsockopt(socket.SOL_CAN_RAW, _CAN_RAW_ERR_FILTER, _CAN_ERR_MASK)
    sock.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMP, 1)
    sock.bind((interface,))

    def stop(_signum, _frame):
        raise KeyboardInterrupt

    # Both, explicitly: a background job of a non-interactive shell starts with
    # SIGINT ignored, and Python then installs no handler of its own.
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    count = 0
    tv = struct.Struct("@ll")
    with path.open("w", buffering=1) as out:
        try:
            while True:
                data, ancdata, _flags, _addr = sock.recvmsg(_CANFD_MTU, socket.CMSG_SPACE(tv.size))
                t = None
                for level, kind, cdata in ancdata:
                    if level == socket.SOL_SOCKET and kind == _SO_TIMESTAMP:
                        sec, usec = tv.unpack(cdata[: tv.size])
                        t = sec + usec / 1e6
                if t is None:
                    continue
                can_id, length, fd_flags = struct.unpack("=IBB2x", data[:8])
                payload = data[8 : 8 + length]
                out.write(
                    format_candump_line(
                        t, interface, can_id, payload, len(data) == _CANFD_MTU, fd_flags
                    )
                    + "\n"
                )
                count += 1
        except KeyboardInterrupt:
            pass
        finally:
            sock.close()
    return count


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 2:
        print(
            "usage: python3 -m rtc_tools.analysis.candump_log <iface> <out.log>", file=sys.stderr
        )
        return 2
    n = record(args[0], Path(args[1]))
    print(f"{n} frames from {args[0]} -> {args[1]}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
