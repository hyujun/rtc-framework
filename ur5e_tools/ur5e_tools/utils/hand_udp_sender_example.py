#!/usr/bin/env python3
"""
핸드 UDP 송신 예제 — ur5e_hand_udp 패키지 프로토콜 기반

프로토콜 사양 (hand_packets.hpp 참조):

  Request 패킷:
    - 헤더: [ID: 0x01] [CMD: 1B] [MODE: 1B]
    - 모터 커맨드: 헤더(3B) + 10 × uint32(float) = 43 bytes, MODE=0x00
    - 센서 커맨드: 헤더(3B)만 전송, MODE=sensor_mode (0=raw, 1=nn)

  Response 패킷:
    - 헤더: [ID: 1B (echo)] [CMD: 1B (echo)] [MODE: 1B (state mode)]
    - MODE: 0 = motor, 1 = fingertip sensor
    - 모터 응답: 헤더(3B) + 10 × uint32(float) = 43 bytes
    - 센서 응답: 헤더(3B) + 16 × uint32 = 67 bytes
      → barometer[8] + reserved[5] (skip) + tof[3] = 11 유효 값

커맨드 코드:
  - WritePosition  0x01 : 모터 10개 목표 위치 전송
  - SetSensorMode  0x04 : 센서 초기화 (MODE 필드 = 0:raw / 1:nn), 3B → 3B echo
  - ReadPosition   0x11 : 모터 위치 요청 → 응답 수신 (43B)
  - ReadVelocity   0x12 : 모터 속도 요청 → 응답 수신 (43B)
  - ReadSensor0~3  0x14..0x17 : 핑거팁 센서 요청 (3B) → 응답 수신 (67B)
                          MODE 필드: 0=raw, 1=nn (전원 인가 시 기본 nn, 항상 raw로 요청)
                          센서 데이터는 uint32 원본 값 (float 변환 없음)

통신 플로우 (1 사이클):
  0. SetSensorMode(cmd=0x04) → send 3B, recv 3B (초기화, 1회)
  1. WritePosition(cmd=0x01) → send 43B
  2. ReadPosition(cmd=0x11)  → send 43B, recv 43B
  3. ReadVelocity(cmd=0x12)  → send 43B, recv 43B
  4. ReadSensor0~3(cmd=0x14..0x17) → send 3B, recv 67B × 4

ROS2 토픽 (hand_udp_node):
  - pub: /hand/joint_states  → positions[10] + velocities[10] + sensors[44] = 64
  - sub: /hand/command       → motor_commands[10]
"""

import csv
import logging
import os
import socket
import struct
import sys
import time
from datetime import datetime

import numpy as np

logger = logging.getLogger(__name__)

# ── 프로토콜 상수 (hand_packets.hpp / types.hpp 기반) ────────────────────────
DEVICE_ID = 0x01
DEFAULT_MODE = 0x00

HEADER_SIZE = 3       # ID + CMD + MODE

# 모터 패킷 상수
MOTOR_DATA_COUNT = 10
MOTOR_PACKET_SIZE = HEADER_SIZE + MOTOR_DATA_COUNT * 4  # 43 bytes

# 센서 패킷 상수
SENSOR_REQUEST_SIZE = HEADER_SIZE  # 3 bytes (헤더만)
SENSOR_RESPONSE_DATA_COUNT = 16    # barometer(8) + reserved(5) + tof(3)
SENSOR_RESPONSE_SIZE = HEADER_SIZE + SENSOR_RESPONSE_DATA_COUNT * 4  # 67 bytes

# 센서 데이터 레이아웃
BAROMETER_COUNT = 8
RESERVED_COUNT = 5    # 패킷에만 존재, 저장하지 않음
TOF_COUNT = 3
SENSOR_VALUES_PER_FINGERTIP = BAROMETER_COUNT + TOF_COUNT  # 11 (유효 값)

NUM_HAND_MOTORS = 10
NUM_FINGERTIPS = 4
NUM_HAND_SENSORS = NUM_FINGERTIPS * SENSOR_VALUES_PER_FINGERTIP  # 44

# State mode (response mode 필드)
STATE_MODE_MOTOR = 0
STATE_MODE_FINGERTIP_SENSOR = 1

# Sensor sub-mode
SENSOR_MODE_RAW = 0
SENSOR_MODE_NN = 1

# 커맨드 코드
CMD_WRITE_POSITION = 0x01
CMD_SET_SENSOR_MODE = 0x04  # 센서 초기화: 모드 변경 (MODE 필드 = raw/nn)
CMD_READ_POSITION = 0x11
CMD_READ_VELOCITY = 0x12
CMD_READ_SENSOR0 = 0x14
CMD_READ_SENSOR1 = 0x15
CMD_READ_SENSOR2 = 0x16
CMD_READ_SENSOR3 = 0x17

CMD_READ_SENSORS = [CMD_READ_SENSOR0, CMD_READ_SENSOR1,
                    CMD_READ_SENSOR2, CMD_READ_SENSOR3]


def _is_sensor_command(cmd: int) -> bool:
    """센서 커맨드인지 확인"""
    return CMD_READ_SENSOR0 <= cmd <= CMD_READ_SENSOR3


# ── 패킷 인코딩/디코딩 (hand_udp_codec.hpp 기반) ────────────────────────────

def float_to_uint32(f: float) -> int:
    """float → uint32 (memcpy 동등)"""
    return struct.unpack('<I', struct.pack('<f', f))[0]


def uint32_to_float(raw: int) -> float:
    """uint32 → float (memcpy 동등)"""
    return struct.unpack('<f', struct.pack('<I', raw))[0]


def encode_motor_packet(cmd: int, floats: list[float] | None = None) -> bytes:
    """
    모터 패킷 인코딩 (43 bytes)

    Args:
        cmd: 커맨드 코드 (예: CMD_WRITE_POSITION, CMD_READ_POSITION)
        floats: 10개 float 데이터 (None이면 0으로 채움, read request용)
    """
    header = struct.pack('<BBB', DEVICE_ID, cmd, DEFAULT_MODE)
    if floats is not None:
        assert len(floats) == MOTOR_DATA_COUNT, f"데이터는 {MOTOR_DATA_COUNT}개여야 합니다"
        data = struct.pack('<10I', *[float_to_uint32(f) for f in floats])
    else:
        data = b'\x00' * (MOTOR_DATA_COUNT * 4)
    return header + data


def encode_sensor_request(cmd: int, sensor_mode: int = SENSOR_MODE_RAW) -> bytes:
    """
    센서 read request 인코딩 (3 bytes, 헤더만)
    MODE 필드에 센서 모드를 설정 (기본 RAW).
    전원 인가 시 기본 모드는 NN이므로 항상 RAW로 요청.

    Args:
        cmd: 센서 커맨드 코드 (CMD_READ_SENSOR0 ~ CMD_READ_SENSOR3)
        sensor_mode: SENSOR_MODE_RAW (0) 또는 SENSOR_MODE_NN (1)
    """
    assert _is_sensor_command(cmd), f"센서 커맨드가 아닙니다: 0x{cmd:02x}"
    return struct.pack('<BBB', DEVICE_ID, cmd, sensor_mode)


def encode_set_sensor_mode(sensor_mode: int) -> bytes:
    """
    센서 모드 설정 패킷 인코딩 (cmd=0x04, 3 bytes)
    전원 인가 후 센서 초기화 시 호출. MODE 필드에 원하는 모드 설정.

    Args:
        sensor_mode: SENSOR_MODE_RAW (0) 또는 SENSOR_MODE_NN (1)
    """
    assert sensor_mode in (SENSOR_MODE_RAW, SENSOR_MODE_NN), \
        f"유효하지 않은 센서 모드: {sensor_mode} (0=raw, 1=nn)"
    return struct.pack('<BBB', DEVICE_ID, CMD_SET_SENSOR_MODE, sensor_mode)


def decode_motor_response(buf: bytes) -> tuple[int, int, list[float]]:
    """
    모터 response 디코딩 (43 bytes)

    Returns:
        (cmd, mode, [10 floats])
    """
    assert len(buf) >= MOTOR_PACKET_SIZE, f"패킷 크기 부족: {len(buf)} < {MOTOR_PACKET_SIZE}"
    _id, cmd, mode = struct.unpack('<BBB', buf[:HEADER_SIZE])
    raw_data = struct.unpack('<10I', buf[HEADER_SIZE:MOTOR_PACKET_SIZE])
    floats = [uint32_to_float(r) for r in raw_data]
    return cmd, mode, floats


def decode_sensor_response(buf: bytes) -> tuple[int, int, list[int]]:
    """
    센서 response 디코딩 (67 bytes)
    barometer[8] + reserved[5] (skip) + tof[3] = 11 유효 값 추출
    float 변환 없이 uint32 원본 값 반환.

    Returns:
        (cmd, mode, [11 uint32: barometer(8) + tof(3)])
    """
    assert len(buf) >= SENSOR_RESPONSE_SIZE, f"패킷 크기 부족: {len(buf)} < {SENSOR_RESPONSE_SIZE}"
    _id, cmd, mode = struct.unpack('<BBB', buf[:HEADER_SIZE])
    raw_data = struct.unpack('<16I', buf[HEADER_SIZE:SENSOR_RESPONSE_SIZE])

    # barometer[0..7] → out[0..7]
    values = [raw_data[i] for i in range(BAROMETER_COUNT)]
    # skip reserved[8..12]
    # tof[13..15] → out[8..10]
    values.extend(raw_data[BAROMETER_COUNT + RESERVED_COUNT + i]
                  for i in range(TOF_COUNT))

    return cmd, mode, values


# Legacy aliases
def encode_packet(cmd: int, floats: list[float] | None = None) -> bytes:
    """Legacy: 모터 패킷 인코딩 (센서 커맨드는 encode_sensor_request 사용)"""
    return encode_motor_packet(cmd, floats)


def decode_packet(buf: bytes) -> tuple[int, list[float]]:
    """Legacy: 모터 response 디코딩 (mode 무시)"""
    cmd, _mode, floats = decode_motor_response(buf)
    return cmd, floats


# ── HandUDPSender 클래스 ─────────────────────────────────────────────────────

class HandUDPSender:
    """
    핸드 UDP request-response 통신 클래스

    hand_controller.hpp의 HandController와 동일한 프로토콜 구현.
    모터 커맨드: 43B 송신 → 43B 수신
    센서 커맨드: 3B 송신 → 67B 수신
    """

    def __init__(self, target_ip: str = "192.168.1.2", target_port: int = 55151,
                 recv_timeout: float = 0.01, num_sensors: int = NUM_FINGERTIPS):
        """
        Args:
            target_ip: 대상 IP
            target_port: 대상 포트
            recv_timeout: 수신 타임아웃 (초)
            num_sensors: 연결된 핑거팁 센서 수 (0~4).
                         예: 2이면 센서 0,1 (cmd 0x14,0x15)만 요청.
        """
        assert 0 <= num_sensors <= NUM_FINGERTIPS, \
            f"num_sensors는 0~{NUM_FINGERTIPS} 범위여야 합니다: {num_sensors}"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(recv_timeout)  # 10ms recv timeout (hand_controller.hpp 동일)
        self.target = (target_ip, target_port)
        self.num_sensors = num_sensors

        print(f"Hand UDP Sender (motor: {MOTOR_PACKET_SIZE}B, sensor req: {SENSOR_REQUEST_SIZE}B → resp: {SENSOR_RESPONSE_SIZE}B)")
        print(f"  Target: {target_ip}:{target_port}")
        print(f"  Motors: {NUM_HAND_MOTORS}, Connected sensors: {self.num_sensors}/{NUM_FINGERTIPS}")

    def write_position(self, positions: list[float]) -> None:
        """모터 목표 위치 전송 (cmd=0x01, 43B)"""
        assert len(positions) == NUM_HAND_MOTORS
        pkt = encode_motor_packet(CMD_WRITE_POSITION, positions)
        self.sock.sendto(pkt, self.target)

    def read_position(self) -> list[float] | None:
        """모터 현재 위치 요청 및 수신 (cmd=0x11, 43B ↔ 43B)"""
        return self._request_motor(CMD_READ_POSITION)

    def read_velocity(self) -> list[float] | None:
        """모터 현재 속도 요청 및 수신 (cmd=0x12, 43B ↔ 43B)"""
        return self._request_motor(CMD_READ_VELOCITY)

    def set_sensor_mode(self, sensor_mode: int = SENSOR_MODE_RAW) -> bool:
        """
        센서 모드 설정 (cmd=0x04, 3B → 3B echo)
        전원 인가 후 센서 초기화 시 호출. NN → RAW 모드 전환.

        Args:
            sensor_mode: SENSOR_MODE_RAW (0) 또는 SENSOR_MODE_NN (1)

        Returns:
            True if echo received, False on timeout
        """
        pkt = encode_set_sensor_mode(sensor_mode)
        self.sock.sendto(pkt, self.target)
        try:
            data, _ = self.sock.recvfrom(256)
            return len(data) >= SENSOR_REQUEST_SIZE
        except socket.timeout:
            return False

    def read_sensor(self, fingertip_idx: int,
                    sensor_mode: int = SENSOR_MODE_RAW) -> tuple[list[int], int] | None:
        """
        핑거팁 센서 데이터 요청 및 수신 (cmd=0x14..0x17, 3B → 67B)
        MODE 필드에 sensor_mode를 설정하여 전송 (기본 RAW).

        Args:
            fingertip_idx: 센서 인덱스 (0 ~ num_sensors-1)
            sensor_mode: SENSOR_MODE_RAW (0) 또는 SENSOR_MODE_NN (1)

        Returns:
            (11 uint32: barometer[8] + tof[3], response_mode), or None on timeout
        """
        assert 0 <= fingertip_idx < self.num_sensors, \
            f"센서 인덱스 {fingertip_idx}은 연결된 센서 범위(0~{self.num_sensors - 1})를 초과합니다"
        return self._request_sensor(CMD_READ_SENSORS[fingertip_idx], sensor_mode)

    def poll_cycle(self, positions: list[float]) -> dict:
        """
        HandController의 1 사이클과 동일한 플로우 실행:
          1. WritePosition                    → send 43B
          2. ReadPosition                     → send 43B, recv 43B
          3. ReadVelocity                     → send 43B, recv 43B
          4. ReadSensor × num_sensors         → send 3B (MODE=raw), recv 67B

        Returns:
            dict with keys:
              - positions: [10 floats] or None
              - velocities: [10 floats] or None
              - sensors: [num_sensors × 11 uint32]
              - sensor_modes: [num_sensors × response_mode]
        """
        # 1. 목표 위치 전송
        self.write_position(positions)

        # 2. 현재 위치 수신
        pos = self.read_position()

        # 3. 현재 속도 수신
        vel = self.read_velocity()

        # 4. 센서 데이터 수신 (연결된 센서만, 각 11개 유효 값, MODE=RAW)
        sensors = []
        sensor_modes = []
        for i in range(self.num_sensors):
            result = self.read_sensor(i)
            if result is not None:
                values, response_mode = result
                sensors.extend(values)
                sensor_modes.append(response_mode)
            else:
                sensors.extend([0] * SENSOR_VALUES_PER_FINGERTIP)
                sensor_modes.append(-1)

        return {
            "positions": pos,
            "velocities": vel,
            "sensors": sensors,
            "sensor_modes": sensor_modes,
        }

    def read_cycle(self) -> dict:
        """
        Read request만 수행 (WritePosition 없음):
          1. ReadPosition          → send 43B, recv 43B
          2. ReadVelocity          → send 43B, recv 43B
          3. ReadSensor × num_sensors → send 3B (MODE=raw), recv 67B

        Returns:
            dict with keys:
              - positions: [10 floats] or None
              - velocities: [10 floats] or None
              - sensors: [num_sensors × 11 uint32]
              - sensor_modes: [num_sensors × response_mode]
        """
        pos = self.read_position()
        vel = self.read_velocity()

        sensors = []
        sensor_modes = []
        for i in range(self.num_sensors):
            result = self.read_sensor(i)
            if result is not None:
                values, response_mode = result
                sensors.extend(values)
                sensor_modes.append(response_mode)
            else:
                sensors.extend([0] * SENSOR_VALUES_PER_FINGERTIP)
                sensor_modes.append(-1)

        return {
            "positions": pos,
            "velocities": vel,
            "sensors": sensors,
            "sensor_modes": sensor_modes,
        }

    def _request_motor(self, cmd: int) -> list[float] | None:
        """모터 요청 전송(43B) 후 응답 수신(43B)"""
        pkt = encode_motor_packet(cmd)
        self.sock.sendto(pkt, self.target)
        try:
            data, _ = self.sock.recvfrom(256)
            _, _mode, floats = decode_motor_response(data)
            return floats
        except socket.timeout:
            return None

    def _request_sensor(self, cmd: int,
                        sensor_mode: int = SENSOR_MODE_RAW) -> tuple[list[int], int] | None:
        """
        센서 요청 전송(3B) 후 응답 수신(67B)
        MODE 필드에 sensor_mode를 설정하여 전송.

        Returns:
            (11 uint32, response_mode) or None on timeout
        """
        pkt = encode_sensor_request(cmd, sensor_mode)
        self.sock.sendto(pkt, self.target)
        try:
            data, _ = self.sock.recvfrom(256)
            _, response_mode, values = decode_sensor_response(data)
            return values, response_mode
        except socket.timeout:
            return None

    def close(self):
        self.sock.close()


def _format_sensor_detail(sensors: list[int], num_sensors: int,
                          sensor_modes: list[int]) -> str:
    """핑거팁별 센서 데이터를 barometer[8] + tof[3]로 포맷 (uint32 원본)"""
    lines = []
    for i in range(num_sensors):
        offset = i * SENSOR_VALUES_PER_FINGERTIP
        data = sensors[offset:offset + SENSOR_VALUES_PER_FINGERTIP]
        baro = data[:BAROMETER_COUNT]
        tof = data[BAROMETER_COUNT:]
        mode = sensor_modes[i] if i < len(sensor_modes) else -1
        mode_name = "raw" if mode == SENSOR_MODE_RAW else ("nn" if mode == SENSOR_MODE_NN else "?")
        baro_str = ', '.join(str(v) for v in baro)
        tof_str = ', '.join(str(v) for v in tof)
        lines.append(f"    fingertip[{i}] (mode={mode_name}): baro=[{baro_str}]  tof=[{tof_str}]")
    return '\n'.join(lines)


# ── CSV 로거 ──────────────────────────────────────────────────────────────────

class HandDataCsvLogger:
    """수신 데이터를 CSV로 저장하는 로거"""

    def __init__(self, num_sensors: int = NUM_FINGERTIPS,
                 output_dir: str = ".", prefix: str = "hand_data"):
        """
        Args:
            num_sensors: 연결된 핑거팁 센서 수
            output_dir: CSV 저장 디렉토리
            prefix: 파일명 접두어
        """
        self.num_sensors = num_sensors
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(output_dir, exist_ok=True)
        self.filepath = os.path.join(output_dir, f"{prefix}_{timestamp}.csv")

        # 헤더 구성
        header = ["timestamp", "cycle"]
        header.extend(f"pos_{i}" for i in range(NUM_HAND_MOTORS))
        header.extend(f"vel_{i}" for i in range(NUM_HAND_MOTORS))
        for s in range(num_sensors):
            header.extend(f"s{s}_baro_{j}" for j in range(BAROMETER_COUNT))
            header.extend(f"s{s}_tof_{j}" for j in range(TOF_COUNT))

        self._file = open(self.filepath, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)
        self._start_time = time.monotonic()

    def log(self, cycle: int, result: dict) -> None:
        """poll_cycle / read_cycle 결과를 1행으로 기록"""
        t = time.monotonic() - self._start_time
        row = [f"{t:.6f}", cycle]

        # positions (10)
        pos = result.get("positions")
        if pos:
            row.extend(f"{v:.6f}" for v in pos)
        else:
            row.extend([""] * NUM_HAND_MOTORS)

        # velocities (10)
        vel = result.get("velocities")
        if vel:
            row.extend(f"{v:.6f}" for v in vel)
        else:
            row.extend([""] * NUM_HAND_MOTORS)

        # sensors (num_sensors × 11 uint32)
        sensors = result.get("sensors", [])
        expected = self.num_sensors * SENSOR_VALUES_PER_FINGERTIP
        if len(sensors) >= expected:
            row.extend(str(v) for v in sensors[:expected])
        else:
            row.extend(str(v) for v in sensors)
            row.extend([""] * (expected - len(sensors)))

        self._writer.writerow(row)

    def close(self) -> None:
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ── Failure Detector ─────────────────────────────────────────────────────────

FAILURE_THRESHOLD = 5  # 연속 감지 횟수 임계값


class HandDataFailureDetector:
    """
    UDP 수신 데이터의 이상 상태를 감지하는 클래스.

    다음 두 가지 실패 조건을 검사:
      1. 데이터가 모두 0인 상태가 FAILURE_THRESHOLD회 이상 연속
      2. 동일한 데이터가 FAILURE_THRESHOLD회 이상 연속

    검사 대상은 motor_enabled / sensor_enabled 플래그로 선택 가능.
    """

    def __init__(self, threshold: int = FAILURE_THRESHOLD,
                 motor_enabled: bool = True, sensor_enabled: bool = True):
        """
        Args:
            threshold: 연속 감지 횟수 임계값
            motor_enabled: True이면 모터 위치 데이터 검사 활성화
            sensor_enabled: True이면 핑거팁 센서 데이터 검사 활성화
        """
        self.threshold = threshold
        self.motor_enabled = motor_enabled
        self.sensor_enabled = sensor_enabled

        # 모터 검사용
        self._motor_zero_count = 0
        self._motor_duplicate_count = 0
        self._prev_positions: list[float] | None = None

        # 센서 검사용
        self._sensor_zero_count = 0
        self._sensor_duplicate_count = 0
        self._prev_sensors: list[int] | None = None

    def check(self, result: dict) -> None:
        """
        poll_cycle / read_cycle 결과를 검사.
        실패 조건 충족 시 에러 로그 출력 후 sys.exit(1).
        """
        if self.motor_enabled:
            self._check_motor(result)
        if self.sensor_enabled:
            self._check_sensor(result)

    def _check_motor(self, result: dict) -> None:
        """모터 위치 데이터 이상 검사"""
        positions = result.get("positions")

        # 데이터 수신 실패 (None) → 0 데이터로 간주
        if positions is None:
            self._motor_zero_count += 1
            self._motor_duplicate_count = 0
            self._prev_positions = None
            if self._motor_zero_count >= self.threshold:
                logger.error(
                    "[FAILURE] 위치 데이터 수신 실패 (None)가 %d회 연속 발생. "
                    "통신 장애로 판단하여 프로그램을 종료합니다.",
                    self._motor_zero_count,
                )
                sys.exit(1)
            return

        # 모든 값이 0인지 확인
        all_zero = all(v == 0.0 for v in positions)
        if all_zero:
            self._motor_zero_count += 1
        else:
            self._motor_zero_count = 0

        if self._motor_zero_count >= self.threshold:
            logger.error(
                "[FAILURE] 위치 데이터가 모두 0인 상태가 %d회 연속 발생. "
                "센서/통신 이상으로 판단하여 프로그램을 종료합니다. "
                "마지막 데이터: %s",
                self._motor_zero_count,
                positions,
            )
            sys.exit(1)

        # 동일 데이터 반복 확인
        if self._prev_positions is not None and positions == self._prev_positions:
            self._motor_duplicate_count += 1
        else:
            self._motor_duplicate_count = 0

        if self._motor_duplicate_count >= self.threshold:
            logger.error(
                "[FAILURE] 동일한 위치 데이터가 %d회 연속 반복 발생. "
                "데이터 갱신 이상으로 판단하여 프로그램을 종료합니다. "
                "반복 데이터: %s",
                self._motor_duplicate_count + 1,  # 최초 1회 + 반복 횟수
                positions,
            )
            sys.exit(1)

        self._prev_positions = list(positions)

    def _check_sensor(self, result: dict) -> None:
        """핑거팁 센서 데이터 이상 검사"""
        sensors = result.get("sensors", [])

        # 센서 데이터가 비어있으면 수신 실패로 간주
        if not sensors:
            self._sensor_zero_count += 1
            self._sensor_duplicate_count = 0
            self._prev_sensors = None
            if self._sensor_zero_count >= self.threshold:
                logger.error(
                    "[FAILURE] 센서 데이터 수신 실패 (빈 데이터)가 %d회 연속 발생. "
                    "통신 장애로 판단하여 프로그램을 종료합니다.",
                    self._sensor_zero_count,
                )
                sys.exit(1)
            return

        # 모든 값이 0인지 확인
        all_zero = all(v == 0 for v in sensors)
        if all_zero:
            self._sensor_zero_count += 1
        else:
            self._sensor_zero_count = 0

        if self._sensor_zero_count >= self.threshold:
            logger.error(
                "[FAILURE] 센서 데이터가 모두 0인 상태가 %d회 연속 발생. "
                "센서/통신 이상으로 판단하여 프로그램을 종료합니다. "
                "마지막 데이터: %s",
                self._sensor_zero_count,
                sensors,
            )
            sys.exit(1)

        # 동일 데이터 반복 확인
        if self._prev_sensors is not None and sensors == self._prev_sensors:
            self._sensor_duplicate_count += 1
        else:
            self._sensor_duplicate_count = 0

        if self._sensor_duplicate_count >= self.threshold:
            logger.error(
                "[FAILURE] 동일한 센서 데이터가 %d회 연속 반복 발생. "
                "데이터 갱신 이상으로 판단하여 프로그램을 종료합니다. "
                "반복 데이터: %s",
                self._sensor_duplicate_count + 1,  # 최초 1회 + 반복 횟수
                sensors,
            )
            sys.exit(1)

        self._prev_sensors = list(sensors)


# ── 예제 함수 ────────────────────────────────────────────────────────────────

def example_write_only(target_ip: str = "192.168.1.2"):
    """WritePosition만 전송하는 간단한 테스트 (사인파)"""
    sender = HandUDPSender(target_ip=target_ip)

    print("\nWritePosition 전송 (사인파)...")
    print("Ctrl+C로 중지\n")

    try:
        t = 0.0
        dt = 0.002  # 500 Hz
        while True:
            # 10개 모터 사인파 위치 (0~1 범위)
            positions = [
                0.5 + 0.3 * np.sin(2 * np.pi * 0.5 * t + i * 0.2)
                for i in range(NUM_HAND_MOTORS)
            ]

            sender.write_position(positions)

            # 1초마다 상태 출력
            if int(t * 10) % 10 == 0 and t > 0:
                print(f"[{t:.1f}s] Pos: [{', '.join(f'{p:.3f}' for p in positions[:4])} ...]")

            t += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n중지됨")
    finally:
        sender.close()


def example_poll_cycle(target_ip: str = "192.168.1.2",
                       num_sensors: int = NUM_FINGERTIPS,
                       csv_dir: str | None = None):
    """HandController와 동일한 전체 poll 사이클 테스트"""
    sender = HandUDPSender(target_ip=target_ip, num_sensors=num_sensors)

    print(f"\n전체 poll 사이클 실행 (센서 {num_sensors}개, MODE=RAW)...")

    # 센서 초기화: NN → RAW 모드 전환 (전원 인가 시 기본 NN)
    if num_sensors > 0:
        print("  센서 초기화: set_sensor_mode(RAW)...")
        if sender.set_sensor_mode(SENSOR_MODE_RAW):
            print("  → 센서 모드 RAW 설정 완료")
        else:
            print("  → 센서 모드 설정 실패 (타임아웃)")

    csv_logger = None
    if csv_dir is not None:
        csv_logger = HandDataCsvLogger(num_sensors=num_sensors,
                                       output_dir=csv_dir, prefix="hand_poll")
        print(f"  CSV 저장: {csv_logger.filepath}")

    failure_detector = HandDataFailureDetector(motor_enabled=False, sensor_enabled=True)

    print(f"  WritePosition(43B) → ReadPosition(43B↔43B) → ReadVelocity(43B↔43B) → ReadSensor×{num_sensors}(3B[MODE=raw]→67B)")
    print("Ctrl+C로 중지\n")

    try:
        t = 0.0
        dt = 0.002  # 500 Hz
        cycle = 0
        while True:
            positions = [
                0.5 + 0.3 * np.sin(2 * np.pi * 0.5 * t + i * 0.2)
                for i in range(NUM_HAND_MOTORS)
            ]

            result = sender.poll_cycle(positions)
            cycle += 1

            # Failure detection: 0 데이터 또는 동일 데이터 연속 감지
            failure_detector.check(result)

            if csv_logger:
                csv_logger.log(cycle, result)

            # 1초마다 상태 출력
            if cycle % 500 == 0:
                pos_str = "None"
                if result["positions"]:
                    pos_str = f'[{", ".join(f"{p:.3f}" for p in result["positions"][:4])} ...]'
                vel_str = "None"
                if result["velocities"]:
                    vel_str = f'[{", ".join(f"{v:.3f}" for v in result["velocities"][:4])} ...]'
                print(f"[cycle {cycle}] pos={pos_str}  vel={vel_str}")
                if num_sensors > 0:
                    print(_format_sensor_detail(
                        result['sensors'], num_sensors, result['sensor_modes']))

            t += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\n중지됨 (총 {cycle} 사이클)")
        if csv_logger:
            print(f"CSV 저장 완료: {csv_logger.filepath}")
    finally:
        if csv_logger:
            csv_logger.close()
        sender.close()


def example_static_pose(target_ip: str = "192.168.1.2"):
    """고정 포즈 전송 (WritePosition only)"""
    sender = HandUDPSender(target_ip=target_ip)

    positions = [0.1 * i for i in range(NUM_HAND_MOTORS)]

    print(f"\n고정 포즈 전송: {positions}")
    print("Ctrl+C로 중지\n")

    try:
        cycle = 0
        while True:
            sender.write_position(positions)
            cycle += 1
            if cycle % 500 == 0:
                print(f"[cycle {cycle}] 전송 중...")
            time.sleep(0.002)  # 500 Hz
    except KeyboardInterrupt:
        print(f"\n중지됨 (총 {cycle} 사이클)")
    finally:
        sender.close()


def example_read_only(target_ip: str = "192.168.1.2",
                      num_sensors: int = NUM_FINGERTIPS,
                      csv_dir: str | None = None):
    """Read request만 수행 (WritePosition 없이 현재 상태 읽기)"""
    sender = HandUDPSender(target_ip=target_ip, num_sensors=num_sensors)

    print(f"\nRead only 사이클 실행 (센서 {num_sensors}개, MODE=RAW)...")

    # 센서 초기화: NN → RAW 모드 전환 (전원 인가 시 기본 NN)
    if num_sensors > 0:
        print("  센서 초기화: set_sensor_mode(RAW)...")
        if sender.set_sensor_mode(SENSOR_MODE_RAW):
            print("  → 센서 모드 RAW 설정 완료")
        else:
            print("  → 센서 모드 설정 실패 (타임아웃)")

    csv_logger = None
    if csv_dir is not None:
        csv_logger = HandDataCsvLogger(num_sensors=num_sensors,
                                       output_dir=csv_dir, prefix="hand_read")
        print(f"  CSV 저장: {csv_logger.filepath}")

    failure_detector = HandDataFailureDetector(motor_enabled=False, sensor_enabled=True)

    print(f"  ReadPosition(43B↔43B) → ReadVelocity(43B↔43B) → ReadSensor×{num_sensors}(3B[MODE=raw]→67B)")
    print("Ctrl+C로 중지\n")

    try:
        cycle = 0
        dt = 0.002  # 500 Hz
        while True:
            result = sender.read_cycle()
            cycle += 1

            # Failure detection: 0 데이터 또는 동일 데이터 연속 감지
            failure_detector.check(result)

            if csv_logger:
                csv_logger.log(cycle, result)

            if cycle % 500 == 0:
                pos_str = "None"
                if result["positions"]:
                    pos_str = f'[{", ".join(f"{p:.3f}" for p in result["positions"][:4])} ...]'
                vel_str = "None"
                if result["velocities"]:
                    vel_str = f'[{", ".join(f"{v:.3f}" for v in result["velocities"][:4])} ...]'
                print(f"[cycle {cycle}] pos={pos_str}  vel={vel_str}")
                if num_sensors > 0:
                    print(_format_sensor_detail(
                        result['sensors'], num_sensors, result['sensor_modes']))

            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\n중지됨 (총 {cycle} 사이클)")
        if csv_logger:
            print(f"CSV 저장 완료: {csv_logger.filepath}")
    finally:
        if csv_logger:
            csv_logger.close()
        sender.close()


def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    print("=" * 65)
    print("  Hand UDP Sender Example")
    print("=" * 65)
    print(f"\n  모터 패킷:  {MOTOR_PACKET_SIZE}B = [ID:1B][CMD:1B][MODE:1B][data:10×float32]")
    print(f"  센서 요청:  {SENSOR_REQUEST_SIZE}B = [ID:1B][CMD:1B][MODE:1B]")
    print(f"  센서 응답:  {SENSOR_RESPONSE_SIZE}B = [ID:1B][CMD:1B][MODE:1B][data:16×uint32]")
    print(f"             → barometer[{BAROMETER_COUNT}] + reserved[{RESERVED_COUNT}](skip) + tof[{TOF_COUNT}] = {SENSOR_VALUES_PER_FINGERTIP} 유효 값")
    print(f"  모터: {NUM_HAND_MOTORS}개, 핑거팁: 최대 {NUM_FINGERTIPS}개, 센서 값/팁: {SENSOR_VALUES_PER_FINGERTIP}개")
    print()

    # 대상 IP 입력 (포트는 55151 고정)
    target_ip = input("대상 IP (기본=192.168.1.2): ").strip() or "192.168.1.2"
    print(f"  → {target_ip}:55151")
    print()

    # 연결된 센서 수 입력
    num_sensors_str = input(f"연결된 센서 수 (0~{NUM_FINGERTIPS}, 기본={NUM_FINGERTIPS}): ").strip()
    num_sensors = int(num_sensors_str) if num_sensors_str else NUM_FINGERTIPS
    assert 0 <= num_sensors <= NUM_FINGERTIPS, f"센서 수는 0~{NUM_FINGERTIPS} 범위여야 합니다"
    print(f"  → 센서 {num_sensors}개 사용 (cmd: {', '.join(f'0x{CMD_READ_SENSORS[i]:02x}' for i in range(num_sensors)) or 'none'})")
    print()

    # CSV 저장 여부
    csv_input = input("CSV 저장 디렉토리 (빈칸=저장 안 함, '.'=현재 디렉토리): ").strip()
    csv_dir = csv_input if csv_input else None
    if csv_dir:
        print(f"  → CSV 저장: {os.path.abspath(csv_dir)}/")
    else:
        print("  → CSV 저장 안 함")
    print()

    print("모드 선택:")
    print("  1) WritePosition 사인파 (전송만)")
    print("  2) 전체 poll 사이클 (WritePos + ReadPos + ReadVel + ReadSensor)")
    print("  3) 고정 포즈 전송")
    print("  4) Read only (ReadPos + ReadVel + ReadSensor, 쓰기 없음)")
    print()

    choice = input("선택 (기본=1): ").strip() or "1"

    if choice == "1":
        example_write_only(target_ip=target_ip)
    elif choice == "2":
        example_poll_cycle(target_ip=target_ip, num_sensors=num_sensors, csv_dir=csv_dir)
    elif choice == "3":
        example_static_pose(target_ip=target_ip)
    elif choice == "4":
        example_read_only(target_ip=target_ip, num_sensors=num_sensors, csv_dir=csv_dir)
    else:
        print("잘못된 선택")


if __name__ == "__main__":
    main()
