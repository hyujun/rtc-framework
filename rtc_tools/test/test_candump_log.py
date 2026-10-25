"""candump -L reader and hand-protocol id classification (epic #895).

The id arithmetic is pinned against the constants hand_canfd_packets.hpp
documents for node 1 (right hand): CMD 0x00004000+idx, echo 0x00000040+idx,
REQ 0x00404000, RESP 0x00800040+idx — and the left hand (node 2) must land on
different ids so a two-hand bus can be split per hand.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rtc_tools.analysis.candump_log import (
    EFF_MASK,
    ERR_FLAG,
    Kind,
    classify,
    cmd_id,
    echo_id,
    node_id_of,
    parse_candump_line,
    read_candump,
    req_id,
    resp_id,
)

RIGHT = node_id_of("right")
LEFT = node_id_of("left")


def test_right_hand_ids_match_the_header_constants():
    assert cmd_id(RIGHT, 0) == 0x00004000
    assert cmd_id(RIGHT, 1) == 0x00004001
    assert echo_id(RIGHT, 0) == 0x00000040
    assert echo_id(RIGHT, 1) == 0x00000041
    assert req_id(RIGHT) == 0x00404000
    assert resp_id(RIGHT, 2) == 0x00800042


def test_left_hand_ids_differ_from_the_right_hand():
    assert cmd_id(LEFT, 0) == 0x00008000
    assert echo_id(LEFT, 0) == 0x00000080
    assert req_id(LEFT) == 0x00408000
    assert resp_id(LEFT, 0) == 0x00800080


def test_unknown_hand_side_is_an_error():
    with pytest.raises(ValueError):
        node_id_of("both")


def test_parses_an_fd_dlc0_request():
    f = parse_candump_line("(1697000000.123456) can0 00404000##1", 7)
    assert f is not None
    assert f.t == pytest.approx(1697000000.123456)
    assert f.interface == "can0"
    assert f.can_id == 0x00404000
    assert f.fd is True
    assert f.is_error is False
    assert f.data == b""
    assert f.line_no == 7
    assert not f.all_zero  # DLC 0 carries no zeros


def test_parses_an_fd_data_frame_and_drops_the_flags_nibble():
    f = parse_candump_line("(1.5) can0 00800042##1A0B1C2D3")
    assert f is not None
    assert f.data == bytes.fromhex("A0B1C2D3")
    assert f.frame_idx == 2
    assert not f.all_zero


def test_parses_a_classic_frame_and_an_rtr_frame():
    f = parse_candump_line("(2.0) can1 123#DEADBEEF")
    assert f is not None
    assert f.fd is False
    assert f.can_id == 0x123
    assert f.data == b"\xde\xad\xbe\xef"
    r = parse_candump_line("(2.1) can1 123#R")
    assert r is not None
    assert r.data == b""


def test_error_frames_are_flagged_and_never_classified_as_hand_traffic():
    f = parse_candump_line("(3.0) can0 20000004#0000000000000000")
    assert f is not None
    assert f.is_error is True
    assert (0x20000004 & ERR_FLAG) != 0
    assert f.can_id == (0x20000004 & EFF_MASK)
    assert classify(f, RIGHT) is Kind.ERROR


@pytest.mark.parametrize(
    "line",
    [
        "",
        "DROPCOUNT: dropped 3 CAN frame(s)",
        "(1.0) can0 00404000##1ABC",  # odd hex digit count
        "(1.0) can0 00404000#ZZ",
        "not a frame at all",
    ],
)
def test_non_frame_lines_return_none(line):
    assert parse_candump_line(line) is None


def test_classification_is_per_hand():
    frames = {
        "cmd": parse_candump_line("(1.0) can0 00004001##1" + "00" * 44),
        "echo": parse_candump_line("(1.0) can0 00000041##1" + "00" * 44),
        "req": parse_candump_line("(1.0) can0 00404000##1"),
        "resp": parse_candump_line("(1.0) can0 00800041##1" + "11" * 8),
        "left_cmd": parse_candump_line("(1.0) can0 00008000##1" + "00" * 44),
    }
    assert classify(frames["cmd"], RIGHT) is Kind.CMD
    assert classify(frames["echo"], RIGHT) is Kind.ECHO
    assert classify(frames["req"], RIGHT) is Kind.REQ
    assert classify(frames["resp"], RIGHT) is Kind.RESP
    assert classify(frames["left_cmd"], RIGHT) is Kind.OTHER
    assert classify(frames["left_cmd"], LEFT) is Kind.CMD
    assert classify(frames["req"], LEFT) is Kind.OTHER


def test_read_candump_counts_unparsed_lines(tmp_path: Path):
    log = tmp_path / "g3.log"
    log.write_text(
        "(1.0) can0 00404000##1\n"
        "\n"
        "DROPCOUNT: dropped 1 CAN frame(s)\n"
        "(1.001) can0 00800040##1" + "00" * 8 + "\n"
    )
    frames, unparsed = read_candump(log)
    assert [f.can_id for f in frames] == [0x00404000, 0x00800040]
    assert unparsed == 1


# ── recorder line format (a candump -L stand-in for a PC without can-utils) ──


@pytest.mark.parametrize(
    ("can_id", "data", "fd", "want_id", "want_fd"),
    [
        (0x80000000 | req_id(RIGHT), b"", True, req_id(RIGHT), True),
        (0x80000000 | resp_id(LEFT, 2), bytes(range(64)), True, resp_id(LEFT, 2), True),
        (0x20000004, bytes(8), False, 0x4, False),  # SocketCAN error frame
        (0x123, b"\xde\xad", False, 0x123, False),  # classic standard id
    ],
)
def test_recorder_lines_parse_back_to_the_same_frame(can_id, data, fd, want_id, want_fd):
    from rtc_tools.analysis.candump_log import format_candump_line

    text = format_candump_line(1697000000.123456, "vcan0", can_id, data, fd, flags=1)
    frame = parse_candump_line(text)
    assert frame is not None, text
    assert frame.t == pytest.approx(1697000000.123456, abs=1e-6)
    assert frame.interface == "vcan0"
    assert frame.can_id == want_id
    assert frame.is_error == bool(can_id & ERR_FLAG)
    assert frame.fd == want_fd
    assert frame.data == data
