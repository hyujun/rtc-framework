"""Tests for rtc_tools.utils.hand_udp_sender_example module.

UDP 프로토콜 코덱 (패킷 인코딩/디코딩), float↔uint32 변환,
UdpTimingStats, HandDataFailureDetector 를 검증합니다.
소켓 I/O 가 필요한 HandUDPSender 클래스 메서드는 제외하고,
순수 로직만 테스트합니다.
"""

from __future__ import annotations

import math
import struct
import sys

import numpy as np
import pytest

from rtc_tools.utils.hand_udp_sender_example import (
    ALL_MOTOR_DATA_COUNT,
    ALL_MOTOR_PACKET_SIZE,
    ALL_SENSOR_DATA_COUNT,
    ALL_SENSOR_RESPONSE_SIZE,
    BAROMETER_COUNT,
    CMD_READ_ALL_MOTORS,
    CMD_READ_ALL_SENSORS,
    CMD_READ_POSITION,
    CMD_READ_SENSOR0,
    CMD_READ_SENSOR1,
    CMD_READ_SENSOR2,
    CMD_READ_SENSOR3,
    CMD_READ_VELOCITY,
    CMD_SET_SENSOR_MODE,
    CMD_WRITE_POSITION,
    DEFAULT_MODE,
    DEVICE_ID,
    FAILURE_THRESHOLD,
    HEADER_SIZE,
    JOINT_MODE_JOINT,
    JOINT_MODE_MOTOR,
    MAX_FINGERTIPS,
    MAX_HAND_SENSORS,
    MOTOR_DATA_COUNT,
    MOTOR_PACKET_SIZE,
    NUM_FINGERTIPS,
    NUM_HAND_MOTORS,
    NUM_HAND_SENSORS,
    RESERVED_COUNT,
    SENSOR_MODE_NN,
    SENSOR_MODE_RAW,
    SENSOR_REQUEST_SIZE,
    SENSOR_RESPONSE_DATA_COUNT,
    SENSOR_RESPONSE_SIZE,
    SENSOR_VALUES_PER_FINGERTIP,
    TOF_COUNT,
    HandDataFailureDetector,
    UdpTimingStats,
    _format_sensor_detail,
    _is_joint_command,
    _is_sensor_command,
    decode_all_motor_response,
    decode_all_sensor_response,
    decode_motor_response,
    decode_packet,
    decode_sensor_response,
    encode_all_motor_request,
    encode_all_sensor_request,
    encode_motor_packet,
    encode_motor_read_request,
    encode_packet,
    encode_sensor_request,
    encode_set_sensor_mode,
    float_to_uint32,
    uint32_to_float,
)


# ═══════════════════════════════════════════════════════════════════════════
# Protocol constants sanity check
# ═══════════════════════════════════════════════════════════════════════════

class TestProtocolConstants:
    """프로토콜 상수의 일관성 검증."""

    def test_header_size(self):
        assert HEADER_SIZE == 3

    def test_motor_packet_size(self):
        assert MOTOR_PACKET_SIZE == HEADER_SIZE + MOTOR_DATA_COUNT * 4
        assert MOTOR_PACKET_SIZE == 43

    def test_sensor_response_size(self):
        assert SENSOR_RESPONSE_SIZE == HEADER_SIZE + SENSOR_RESPONSE_DATA_COUNT * 4
        assert SENSOR_RESPONSE_SIZE == 67

    def test_all_motor_packet_size(self):
        assert ALL_MOTOR_DATA_COUNT == NUM_HAND_MOTORS * 3  # pos + vel + cur
        assert ALL_MOTOR_PACKET_SIZE == HEADER_SIZE + ALL_MOTOR_DATA_COUNT * 4
        assert ALL_MOTOR_PACKET_SIZE == 123

    def test_all_sensor_response_size(self):
        assert ALL_SENSOR_DATA_COUNT == SENSOR_RESPONSE_DATA_COUNT * NUM_FINGERTIPS
        assert ALL_SENSOR_RESPONSE_SIZE == HEADER_SIZE + ALL_SENSOR_DATA_COUNT * 4
        assert ALL_SENSOR_RESPONSE_SIZE == 259

    def test_sensor_values_per_fingertip(self):
        assert SENSOR_VALUES_PER_FINGERTIP == BAROMETER_COUNT + TOF_COUNT
        assert SENSOR_VALUES_PER_FINGERTIP == 11

    def test_num_hand_sensors(self):
        assert NUM_HAND_SENSORS == NUM_FINGERTIPS * SENSOR_VALUES_PER_FINGERTIP
        assert NUM_HAND_SENSORS == 44

    def test_max_fingertips(self):
        assert MAX_FINGERTIPS == 8

    def test_max_hand_sensors(self):
        assert MAX_HAND_SENSORS == MAX_FINGERTIPS * SENSOR_VALUES_PER_FINGERTIP
        assert MAX_HAND_SENSORS == 88

    def test_joint_mode_values(self):
        assert JOINT_MODE_MOTOR == 0x00
        assert JOINT_MODE_JOINT == 0x01


# ═══════════════════════════════════════════════════════════════════════════
# float ↔ uint32 변환
# ═══════════════════════════════════════════════════════════════════════════

class TestFloatUint32Conversion:
    """float_to_uint32 / uint32_to_float: memcpy 동등 변환."""

    def test_roundtrip_positive(self):
        for val in [0.0, 1.0, 3.14, 1e-6, 1e6]:
            assert uint32_to_float(float_to_uint32(val)) == pytest.approx(val)

    def test_roundtrip_negative(self):
        for val in [-1.0, -3.14, -1e-6]:
            assert uint32_to_float(float_to_uint32(val)) == pytest.approx(val)

    def test_zero(self):
        raw = float_to_uint32(0.0)
        assert raw == 0
        assert uint32_to_float(0) == 0.0

    def test_one(self):
        raw = float_to_uint32(1.0)
        # IEEE 754: 1.0 = 0x3F800000
        assert raw == 0x3F800000

    def test_negative_one(self):
        raw = float_to_uint32(-1.0)
        # IEEE 754: -1.0 = 0xBF800000
        assert raw == 0xBF800000

    def test_nan_roundtrip(self):
        raw = float_to_uint32(float('nan'))
        assert math.isnan(uint32_to_float(raw))

    def test_inf_roundtrip(self):
        raw = float_to_uint32(float('inf'))
        assert uint32_to_float(raw) == float('inf')


# ═══════════════════════════════════════════════════════════════════════════
# _is_sensor_command
# ═══════════════════════════════════════════════════════════════════════════

class TestIsSensorCommand:
    """C++ hand_packets::IsSensorCommand 동일 로직."""

    def test_individual_sensor_commands(self):
        for cmd in [CMD_READ_SENSOR0, CMD_READ_SENSOR1,
                    CMD_READ_SENSOR2, CMD_READ_SENSOR3]:
            assert _is_sensor_command(cmd) is True

    def test_set_sensor_mode(self):
        assert _is_sensor_command(CMD_SET_SENSOR_MODE) is True

    def test_read_all_sensors(self):
        assert _is_sensor_command(CMD_READ_ALL_SENSORS) is True

    def test_non_sensor_commands(self):
        for cmd in [CMD_WRITE_POSITION, CMD_READ_POSITION,
                    CMD_READ_VELOCITY, CMD_READ_ALL_MOTORS]:
            assert _is_sensor_command(cmd) is False


class TestIsJointCommand:
    """C++ hand_packets::IsJointCommand 동일 로직."""

    def test_joint_commands(self):
        for cmd in [CMD_WRITE_POSITION, CMD_READ_ALL_MOTORS,
                    CMD_READ_POSITION, CMD_READ_VELOCITY]:
            assert _is_joint_command(cmd) is True

    def test_non_joint_commands(self):
        for cmd in [CMD_SET_SENSOR_MODE, CMD_READ_SENSOR0,
                    CMD_READ_ALL_SENSORS]:
            assert _is_joint_command(cmd) is False


# ═══════════════════════════════════════════════════════════════════════════
# Motor packet encoding / decoding
# ═══════════════════════════════════════════════════════════════════════════

class TestMotorPacketCodec:
    """encode_motor_packet / decode_motor_response 라운드트립."""

    def test_write_position_roundtrip(self):
        values = [0.1 * i for i in range(NUM_HAND_MOTORS)]
        pkt = encode_motor_packet(CMD_WRITE_POSITION, values)

        assert len(pkt) == MOTOR_PACKET_SIZE
        assert pkt[0] == DEVICE_ID
        assert pkt[1] == CMD_WRITE_POSITION
        assert pkt[2] == JOINT_MODE_MOTOR

        cmd, mode, decoded = decode_motor_response(pkt)
        assert cmd == CMD_WRITE_POSITION
        assert mode == JOINT_MODE_MOTOR
        for i in range(NUM_HAND_MOTORS):
            assert decoded[i] == pytest.approx(values[i], abs=1e-6)

    def test_write_position_joint_mode(self):
        """joint_mode=JOINT_MODE_JOINT 시 MODE 바이트가 0x01."""
        values = [0.0] * NUM_HAND_MOTORS
        pkt = encode_motor_packet(CMD_WRITE_POSITION, values,
                                  joint_mode=JOINT_MODE_JOINT)
        assert pkt[2] == JOINT_MODE_JOINT

        _, mode, _ = decode_motor_response(pkt)
        assert mode == JOINT_MODE_JOINT

    def test_read_request_zeros(self):
        """read request 시 데이터는 0으로 채워짐."""
        pkt = encode_motor_packet(CMD_READ_POSITION)
        assert len(pkt) == MOTOR_PACKET_SIZE

        _, _, decoded = decode_motor_response(pkt)
        assert all(v == 0.0 for v in decoded)

    def test_wrong_count_raises(self):
        with pytest.raises(AssertionError):
            encode_motor_packet(CMD_WRITE_POSITION, [1.0, 2.0])  # need 10

    def test_decode_short_buffer_raises(self):
        with pytest.raises(AssertionError):
            decode_motor_response(b'\x00' * 10)


class TestMotorReadRequest:
    """encode_motor_read_request: 3B 헤더만 전송 (C++ MakeMotorReadRequest 동일)."""

    def test_read_position_request(self):
        pkt = encode_motor_read_request(CMD_READ_POSITION)
        assert len(pkt) == HEADER_SIZE
        assert pkt[0] == DEVICE_ID
        assert pkt[1] == CMD_READ_POSITION
        assert pkt[2] == JOINT_MODE_MOTOR

    def test_read_velocity_request(self):
        pkt = encode_motor_read_request(CMD_READ_VELOCITY)
        assert len(pkt) == HEADER_SIZE
        assert pkt[1] == CMD_READ_VELOCITY

    def test_joint_mode(self):
        pkt = encode_motor_read_request(CMD_READ_POSITION,
                                        joint_mode=JOINT_MODE_JOINT)
        assert pkt[2] == JOINT_MODE_JOINT

    def test_all_motors_with_joint_mode(self):
        pkt = encode_all_motor_request(joint_mode=JOINT_MODE_JOINT)
        assert len(pkt) == HEADER_SIZE
        assert pkt[1] == CMD_READ_ALL_MOTORS
        assert pkt[2] == JOINT_MODE_JOINT


class TestLegacyAliases:
    """encode_packet / decode_packet legacy aliases."""

    def test_encode_packet(self):
        values = [float(i) for i in range(NUM_HAND_MOTORS)]
        pkt = encode_packet(CMD_WRITE_POSITION, values)
        assert len(pkt) == MOTOR_PACKET_SIZE

    def test_decode_packet(self):
        values = [float(i) for i in range(NUM_HAND_MOTORS)]
        pkt = encode_motor_packet(CMD_READ_POSITION, values)
        cmd, decoded = decode_packet(pkt)
        assert cmd == CMD_READ_POSITION
        for i in range(NUM_HAND_MOTORS):
            assert decoded[i] == pytest.approx(values[i], abs=1e-6)


# ═══════════════════════════════════════════════════════════════════════════
# Sensor packet encoding / decoding
# ═══════════════════════════════════════════════════════════════════════════

class TestSensorPacketCodec:
    """encode_sensor_request / decode_sensor_response."""

    def test_sensor_request_size(self):
        pkt = encode_sensor_request(CMD_READ_SENSOR0)
        assert len(pkt) == SENSOR_REQUEST_SIZE
        assert pkt[0] == DEVICE_ID
        assert pkt[1] == CMD_READ_SENSOR0
        assert pkt[2] == SENSOR_MODE_RAW

    def test_sensor_request_nn_mode(self):
        pkt = encode_sensor_request(CMD_READ_SENSOR1, SENSOR_MODE_NN)
        assert pkt[2] == SENSOR_MODE_NN

    def test_sensor_request_non_sensor_raises(self):
        with pytest.raises(AssertionError):
            encode_sensor_request(CMD_READ_POSITION)

    def test_sensor_request_bulk_raises(self):
        """bulk sensor cmd (0x19)는 encode_all_sensor_request 사용."""
        with pytest.raises(AssertionError):
            encode_sensor_request(CMD_READ_ALL_SENSORS)

    def test_sensor_response_decode(self):
        """67B 센서 응답 디코딩: barometer[8] + skip reserved[5] + tof[3] = 11."""
        # Build a fake 67-byte response (int32 — C++ SensorResponsePacket)
        header = struct.pack('<BBB', DEVICE_ID, CMD_READ_SENSOR0, SENSOR_MODE_RAW)
        # 16 int32 values: baro[0..7], reserved[8..12], tof[13..15]
        data_values = list(range(16))
        data = struct.pack('<16i', *data_values)
        buf = header + data

        cmd, mode, values = decode_sensor_response(buf)
        assert cmd == CMD_READ_SENSOR0
        assert mode == SENSOR_MODE_RAW
        assert len(values) == SENSOR_VALUES_PER_FINGERTIP  # 11

        # barometer: data[0..7]
        for i in range(BAROMETER_COUNT):
            assert values[i] == i

        # tof: data[13..15] (skip reserved 8..12)
        for i in range(TOF_COUNT):
            assert values[BAROMETER_COUNT + i] == BAROMETER_COUNT + RESERVED_COUNT + i

    def test_sensor_response_signed_values(self):
        """센서 데이터가 int32 (signed) 로 디코딩되는지 확인."""
        header = struct.pack('<BBB', DEVICE_ID, CMD_READ_SENSOR0, SENSOR_MODE_RAW)
        # 음수 barometer 값 포함
        data_values = [-100, -50, 0, 50, 100, 200, 300, 400,  # baro[8]
                       0, 0, 0, 0, 0,                          # reserved[5]
                       -10, 20, -30]                            # tof[3]
        data = struct.pack('<16i', *data_values)
        buf = header + data

        _, _, values = decode_sensor_response(buf)
        assert values[0] == -100  # signed negative
        assert values[1] == -50
        assert values[BAROMETER_COUNT] == -10      # tof[0]
        assert values[BAROMETER_COUNT + 2] == -30  # tof[2]

    def test_sensor_response_short_buffer_raises(self):
        with pytest.raises(AssertionError):
            decode_sensor_response(b'\x00' * 10)


# ═══════════════════════════════════════════════════════════════════════════
# Set sensor mode
# ═══════════════════════════════════════════════════════════════════════════

class TestSetSensorMode:
    def test_raw_mode(self):
        pkt = encode_set_sensor_mode(SENSOR_MODE_RAW)
        assert len(pkt) == 3
        assert pkt[1] == CMD_SET_SENSOR_MODE
        assert pkt[2] == SENSOR_MODE_RAW

    def test_nn_mode(self):
        pkt = encode_set_sensor_mode(SENSOR_MODE_NN)
        assert pkt[2] == SENSOR_MODE_NN

    def test_invalid_mode_raises(self):
        with pytest.raises(AssertionError):
            encode_set_sensor_mode(99)


# ═══════════════════════════════════════════════════════════════════════════
# All-motor (bulk) encoding / decoding
# ═══════════════════════════════════════════════════════════════════════════

class TestAllMotorCodec:
    """encode_all_motor_request / decode_all_motor_response: 123B bulk."""

    def test_request_header(self):
        pkt = encode_all_motor_request()
        assert len(pkt) == HEADER_SIZE
        assert pkt[0] == DEVICE_ID
        assert pkt[1] == CMD_READ_ALL_MOTORS

    def test_response_roundtrip(self):
        """Build a 123B response and decode."""
        positions = [0.1 * i for i in range(NUM_HAND_MOTORS)]
        velocities = [1.0 + 0.1 * i for i in range(NUM_HAND_MOTORS)]
        currents = [0.01 * i for i in range(NUM_HAND_MOTORS)]

        # Build raw packet
        header = struct.pack('<BBB', DEVICE_ID, CMD_READ_ALL_MOTORS, 0x00)
        raw_ints = []
        for v in positions + velocities + currents:
            raw_ints.append(float_to_uint32(v))
        data = struct.pack(f'<{ALL_MOTOR_DATA_COUNT}I', *raw_ints)
        buf = header + data

        assert len(buf) == ALL_MOTOR_PACKET_SIZE

        cmd, mode, pos_dec, vel_dec, cur_dec = decode_all_motor_response(buf)
        assert cmd == CMD_READ_ALL_MOTORS

        for i in range(NUM_HAND_MOTORS):
            assert pos_dec[i] == pytest.approx(positions[i], abs=1e-6)
            assert vel_dec[i] == pytest.approx(velocities[i], abs=1e-6)
            assert cur_dec[i] == pytest.approx(currents[i], abs=1e-6)

    def test_short_buffer_raises(self):
        with pytest.raises(AssertionError):
            decode_all_motor_response(b'\x00' * 50)


# ═══════════════════════════════════════════════════════════════════════════
# All-sensor (bulk) encoding / decoding
# ═══════════════════════════════════════════════════════════════════════════

class TestAllSensorCodec:
    """encode_all_sensor_request / decode_all_sensor_response: 259B bulk."""

    def test_request_header(self):
        pkt = encode_all_sensor_request()
        assert len(pkt) == HEADER_SIZE
        assert pkt[0] == DEVICE_ID
        assert pkt[1] == CMD_READ_ALL_SENSORS
        assert pkt[2] == SENSOR_MODE_RAW

    def test_request_nn_mode(self):
        pkt = encode_all_sensor_request(SENSOR_MODE_NN)
        assert pkt[2] == SENSOR_MODE_NN

    def test_response_roundtrip(self):
        """Build a 259B response and verify decoded values (int32)."""
        header = struct.pack('<BBB', DEVICE_ID, CMD_READ_ALL_SENSORS, SENSOR_MODE_RAW)

        # 4 fingertips × 16 int32 each (C++ AllSensorResponsePacket.data = int32_t[64])
        all_data = []
        for finger in range(NUM_FINGERTIPS):
            base = finger * 100
            # barometer[0..7]
            baro = [base + i for i in range(BAROMETER_COUNT)]
            # reserved[8..12]
            reserved = [999] * RESERVED_COUNT
            # tof[13..15]
            tof = [base + 50 + i for i in range(TOF_COUNT)]
            all_data.extend(baro + reserved + tof)

        data = struct.pack(f'<{ALL_SENSOR_DATA_COUNT}i', *all_data)
        buf = header + data
        assert len(buf) == ALL_SENSOR_RESPONSE_SIZE

        cmd, mode, values = decode_all_sensor_response(buf)
        assert cmd == CMD_READ_ALL_SENSORS
        assert mode == SENSOR_MODE_RAW
        assert len(values) == NUM_HAND_SENSORS  # 44

        # Check each fingertip
        for finger in range(NUM_FINGERTIPS):
            base_val = finger * 100
            offset = finger * SENSOR_VALUES_PER_FINGERTIP

            # barometer
            for i in range(BAROMETER_COUNT):
                assert values[offset + i] == base_val + i

            # tof
            for i in range(TOF_COUNT):
                assert values[offset + BAROMETER_COUNT + i] == base_val + 50 + i

    def test_short_buffer_raises(self):
        with pytest.raises(AssertionError):
            decode_all_sensor_response(b'\x00' * 100)


# ═══════════════════════════════════════════════════════════════════════════
# _format_sensor_detail
# ═══════════════════════════════════════════════════════════════════════════

class TestFormatSensorDetail:
    def test_single_sensor(self):
        sensors = list(range(SENSOR_VALUES_PER_FINGERTIP))  # [0..10]
        modes = [SENSOR_MODE_RAW]
        result = _format_sensor_detail(sensors, 1, modes)
        assert "fingertip[0]" in result
        assert "mode=raw" in result
        assert "baro=" in result
        assert "tof=" in result

    def test_nn_mode_label(self):
        sensors = [0] * SENSOR_VALUES_PER_FINGERTIP
        modes = [SENSOR_MODE_NN]
        result = _format_sensor_detail(sensors, 1, modes)
        assert "mode=nn" in result

    def test_unknown_mode_label(self):
        sensors = [0] * SENSOR_VALUES_PER_FINGERTIP
        modes = [-1]
        result = _format_sensor_detail(sensors, 1, modes)
        assert "mode=?" in result

    def test_multiple_sensors(self):
        sensors = [0] * (2 * SENSOR_VALUES_PER_FINGERTIP)
        modes = [SENSOR_MODE_RAW, SENSOR_MODE_NN]
        result = _format_sensor_detail(sensors, 2, modes)
        assert "fingertip[0]" in result
        assert "fingertip[1]" in result


# ═══════════════════════════════════════════════════════════════════════════
# UdpTimingStats
# ═══════════════════════════════════════════════════════════════════════════

class TestUdpTimingStats:
    def test_empty_stats(self):
        stats = UdpTimingStats()
        result = stats.format_stats()
        assert "0 cycles" in result
        assert "timeouts 0" in result

    def test_record_legacy_cycle(self):
        stats = UdpTimingStats(window_size=10)
        timing = {
            "cycle": 0.005,
            "write_position": 0.001,
            "read_position": 0.001,
            "read_velocity": 0.001,
            "read_sensors": 0.002,
            "timeouts": 0,
        }
        stats.record(timing)
        assert stats._total_cycles == 1
        assert len(stats._cycle_times) == 1
        assert stats._timeout_count == 0

    def test_record_bulk_cycle(self):
        stats = UdpTimingStats(window_size=10)
        timing = {
            "cycle": 0.003,
            "write_position": 0.001,
            "read_all_motors": 0.001,
            "read_all_sensors": 0.001,
            "timeouts": 1,
        }
        stats.record(timing)
        assert stats._total_cycles == 1
        assert len(stats._read_all_motor_times) == 1
        assert len(stats._read_all_sensor_times) == 1
        assert stats._timeout_count == 1

    def test_window_size_limit(self):
        stats = UdpTimingStats(window_size=5)
        for i in range(10):
            stats.record({"cycle": 0.001 * i, "timeouts": 0})
        assert len(stats._cycle_times) == 5
        assert stats._total_cycles == 10

    def test_format_includes_all_phases(self):
        stats = UdpTimingStats()
        timing = {
            "cycle": 0.005,
            "write_position": 0.001,
            "read_position": 0.001,
            "read_velocity": 0.001,
            "read_sensors": 0.002,
            "timeouts": 0,
        }
        stats.record(timing)
        result = stats.format_stats()
        assert "cycle_total" in result
        assert "write_pos" in result
        assert "read_pos" in result
        assert "read_vel" in result
        assert "read_sensors" in result

    def test_stats_str_na(self):
        """빈 deque에 대한 N/A 출력."""
        from collections import deque
        result = UdpTimingStats._stats_str(deque(), "test")
        assert "N/A" in result

    def test_stats_str_values(self):
        from collections import deque
        d = deque([0.001, 0.002, 0.003])
        result = UdpTimingStats._stats_str(d, "test")
        assert "avg=" in result
        assert "min=" in result
        assert "max=" in result
        assert "std=" in result


# ═══════════════════════════════════════════════════════════════════════════
# HandDataFailureDetector
# ═══════════════════════════════════════════════════════════════════════════

class TestHandDataFailureDetector:
    """HandDataFailureDetector: 0 데이터 / 동일 데이터 연속 감지."""

    def test_normal_data_no_exit(self):
        """정상 데이터는 sys.exit 호출 없음."""
        det = HandDataFailureDetector(threshold=3)
        for i in range(10):
            result = {
                "positions": [float(i + j) for j in range(NUM_HAND_MOTORS)],
                "sensors": [i + j for j in range(NUM_HAND_SENSORS)],
            }
            det.check(result)  # no exit

    def test_motor_none_triggers_exit(self):
        """positions=None 이 threshold회 연속이면 sys.exit."""
        det = HandDataFailureDetector(threshold=3, sensor_enabled=False)
        for _ in range(2):
            det.check({"positions": None, "sensors": []})

        with pytest.raises(SystemExit):
            det.check({"positions": None, "sensors": []})

    def test_motor_all_zero_triggers_exit(self):
        """positions 모두 0 이 threshold회 연속이면 sys.exit."""
        det = HandDataFailureDetector(threshold=3, sensor_enabled=False)
        zeros = [0.0] * NUM_HAND_MOTORS
        for _ in range(2):
            det.check({"positions": zeros, "sensors": []})

        with pytest.raises(SystemExit):
            det.check({"positions": zeros, "sensors": []})

    def test_motor_duplicate_triggers_exit(self):
        """동일 positions가 threshold회 연속 반복이면 sys.exit."""
        det = HandDataFailureDetector(threshold=3, sensor_enabled=False)
        same = [1.0 + i for i in range(NUM_HAND_MOTORS)]
        # First occurrence is not a duplicate
        det.check({"positions": same, "sensors": []})
        # Next threshold occurrences are duplicates
        for _ in range(2):
            det.check({"positions": list(same), "sensors": []})

        with pytest.raises(SystemExit):
            det.check({"positions": list(same), "sensors": []})

    def test_motor_zero_resets_on_nonzero(self):
        """0 데이터 후 정상 데이터가 오면 카운터 리셋."""
        det = HandDataFailureDetector(threshold=5, sensor_enabled=False)
        zeros = [0.0] * NUM_HAND_MOTORS
        normal = [1.0 + i for i in range(NUM_HAND_MOTORS)]

        for _ in range(4):  # just under threshold
            det.check({"positions": zeros, "sensors": []})
        det.check({"positions": normal, "sensors": []})  # reset

        # Should be able to survive 4 more zeros without exit
        for _ in range(4):
            det.check({"positions": zeros, "sensors": []})

    def test_sensor_empty_triggers_exit(self):
        """센서 데이터가 빈 리스트이면 threshold회 후 sys.exit."""
        det = HandDataFailureDetector(threshold=3, motor_enabled=False)
        for _ in range(2):
            det.check({"positions": None, "sensors": []})

        with pytest.raises(SystemExit):
            det.check({"positions": None, "sensors": []})

    def test_sensor_all_zero_triggers_exit(self):
        """센서 데이터 모두 0 이 threshold회 연속이면 sys.exit."""
        det = HandDataFailureDetector(threshold=3, motor_enabled=False)
        zeros = [0] * NUM_HAND_SENSORS
        for _ in range(2):
            det.check({"positions": None, "sensors": zeros})

        with pytest.raises(SystemExit):
            det.check({"positions": None, "sensors": zeros})

    def test_sensor_duplicate_triggers_exit(self):
        """동일 센서 데이터가 threshold회 연속이면 sys.exit."""
        det = HandDataFailureDetector(threshold=3, motor_enabled=False)
        same = list(range(1, NUM_HAND_SENSORS + 1))
        det.check({"positions": None, "sensors": same})
        for _ in range(2):
            det.check({"positions": None, "sensors": list(same)})

        with pytest.raises(SystemExit):
            det.check({"positions": None, "sensors": list(same)})

    def test_both_enabled(self):
        """motor + sensor 둘 다 활성화 시 어느 하나라도 실패하면 exit."""
        det = HandDataFailureDetector(threshold=2)
        # motor None → sensor 정상
        det.check({
            "positions": None,
            "sensors": list(range(1, NUM_HAND_SENSORS + 1)),
        })

        with pytest.raises(SystemExit):
            det.check({
                "positions": None,
                "sensors": list(range(1, NUM_HAND_SENSORS + 1)),
            })

    def test_disabled_checks_skipped(self):
        """비활성화된 검사는 건너뜀."""
        det = HandDataFailureDetector(
            threshold=2, motor_enabled=False, sensor_enabled=False)
        for _ in range(100):
            det.check({"positions": None, "sensors": []})  # no exit


# ═══════════════════════════════════════════════════════════════════════════
# Edge cases — packet boundary testing
# ═══════════════════════════════════════════════════════════════════════════

class TestPacketBoundaries:
    """패킷 크기 경계값 테스트."""

    def test_motor_packet_exact_size(self):
        """정확히 43B 패킷은 디코딩 성공."""
        pkt = encode_motor_packet(CMD_READ_POSITION)
        assert len(pkt) == 43
        cmd, mode, vals = decode_motor_response(pkt)
        assert cmd == CMD_READ_POSITION
        assert len(vals) == NUM_HAND_MOTORS

    def test_motor_packet_extra_bytes_ok(self):
        """43B보다 큰 버퍼도 디코딩 성공 (앞 43B만 사용)."""
        pkt = encode_motor_packet(CMD_READ_POSITION) + b'\xFF' * 10
        cmd, mode, vals = decode_motor_response(pkt)
        assert cmd == CMD_READ_POSITION

    def test_sensor_response_exact_size(self):
        """정확히 67B 센서 응답 디코딩 성공."""
        header = struct.pack('<BBB', DEVICE_ID, CMD_READ_SENSOR0, 0)
        data = struct.pack('<16i', *range(16))
        buf = header + data
        assert len(buf) == 67
        cmd, mode, vals = decode_sensor_response(buf)
        assert len(vals) == 11

    def test_all_motor_exact_size(self):
        """정확히 123B all-motor 응답 디코딩 성공."""
        header = struct.pack('<BBB', DEVICE_ID, CMD_READ_ALL_MOTORS, 0)
        data = struct.pack(f'<{ALL_MOTOR_DATA_COUNT}I',
                           *[float_to_uint32(0.0)] * ALL_MOTOR_DATA_COUNT)
        buf = header + data
        assert len(buf) == 123
        cmd, mode, pos, vel, cur = decode_all_motor_response(buf)
        assert len(pos) == 10
        assert len(vel) == 10
        assert len(cur) == 10

    def test_all_sensor_exact_size(self):
        """정확히 259B all-sensor 응답 디코딩 성공."""
        header = struct.pack('<BBB', DEVICE_ID, CMD_READ_ALL_SENSORS, 0)
        data = struct.pack(f'<{ALL_SENSOR_DATA_COUNT}i',
                           *range(ALL_SENSOR_DATA_COUNT))
        buf = header + data
        assert len(buf) == 259
        cmd, mode, vals = decode_all_sensor_response(buf)
        assert len(vals) == 44


# ═══════════════════════════════════════════════════════════════════════════
# Special float values
# ═══════════════════════════════════════════════════════════════════════════

class TestSpecialFloatValues:
    """특수 float 값(NaN, Inf, subnormal)의 패킷 라운드트립."""

    def test_nan_in_motor_packet(self):
        values = [float('nan')] + [0.0] * (NUM_HAND_MOTORS - 1)
        pkt = encode_motor_packet(CMD_WRITE_POSITION, values)
        _, _, decoded = decode_motor_response(pkt)
        assert math.isnan(decoded[0])

    def test_inf_in_motor_packet(self):
        values = [float('inf')] + [0.0] * (NUM_HAND_MOTORS - 1)
        pkt = encode_motor_packet(CMD_WRITE_POSITION, values)
        _, _, decoded = decode_motor_response(pkt)
        assert math.isinf(decoded[0])

    def test_very_small_value(self):
        values = [1e-38] + [0.0] * (NUM_HAND_MOTORS - 1)
        pkt = encode_motor_packet(CMD_WRITE_POSITION, values)
        _, _, decoded = decode_motor_response(pkt)
        assert decoded[0] == pytest.approx(1e-38, abs=1e-44)

    def test_very_large_value(self):
        values = [1e38] + [0.0] * (NUM_HAND_MOTORS - 1)
        pkt = encode_motor_packet(CMD_WRITE_POSITION, values)
        _, _, decoded = decode_motor_response(pkt)
        assert decoded[0] == pytest.approx(1e38, rel=1e-6)
