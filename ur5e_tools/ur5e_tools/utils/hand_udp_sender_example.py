#!/usr/bin/env python3
"""
핸드 UDP 송신 예제 — ur5e_hand_udp 패키지 프로토콜 기반

프로토콜 사양 (hand_packets.hpp 참조):

  Request 패킷:
    - 헤더: [ID: 0x01] [CMD: 1B] [MODE: 0x00]
    - 모터 커맨드: 헤더(3B) + 10 × uint32(float) = 43 bytes
    - 센서 커맨드: 헤더(3B)만 전송 (데이터 없음)

  Response 패킷:
    - 헤더: [ID: 1B (echo)] [CMD: 1B (echo)] [MODE: 1B (state mode)]
    - MODE: 0 = motor, 1 = fingertip sensor
    - 모터 응답: 헤더(3B) + 10 × uint32(float) = 43 bytes
    - 센서 응답: 헤더(3B) + 16 × uint32 = 67 bytes
      → barometer[8] + reserved[5] (skip) + tof[3] = 11 유효 값

커맨드 코드:
  - WritePosition  0x01 : 모터 10개 목표 위치 전송
  - ReadPosition   0x11 : 모터 위치 요청 → 응답 수신 (43B)
  - ReadVelocity   0x12 : 모터 속도 요청 → 응답 수신 (43B)
  - ReadSensor0~3  0x14..0x17 : 핑거팁 센서 요청 (3B) → 응답 수신 (67B)

통신 플로우 (1 사이클):
  1. WritePosition(cmd=0x01) → send 43B
  2. ReadPosition(cmd=0x11)  → send 43B, recv 43B
  3. ReadVelocity(cmd=0x12)  → send 43B, recv 43B
  4. ReadSensor0~3(cmd=0x14..0x17) → send 3B, recv 67B × 4

ROS2 토픽 (hand_udp_node):
  - pub: /hand/joint_states  → positions[10] + velocities[10] + sensors[44] = 64
  - sub: /hand/command       → motor_commands[10]
"""

import socket
import struct
import time
import numpy as np

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


def encode_sensor_request(cmd: int) -> bytes:
    """
    센서 read request 인코딩 (3 bytes, 헤더만)

    Args:
        cmd: 센서 커맨드 코드 (CMD_READ_SENSOR0 ~ CMD_READ_SENSOR3)
    """
    assert _is_sensor_command(cmd), f"센서 커맨드가 아닙니다: 0x{cmd:02x}"
    return struct.pack('<BBB', DEVICE_ID, cmd, DEFAULT_MODE)


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


def decode_sensor_response(buf: bytes) -> tuple[int, int, list[float]]:
    """
    센서 response 디코딩 (67 bytes)
    barometer[8] + reserved[5] (skip) + tof[3] = 11 유효 값 추출

    Returns:
        (cmd, mode, [11 floats: barometer(8) + tof(3)])
    """
    assert len(buf) >= SENSOR_RESPONSE_SIZE, f"패킷 크기 부족: {len(buf)} < {SENSOR_RESPONSE_SIZE}"
    _id, cmd, mode = struct.unpack('<BBB', buf[:HEADER_SIZE])
    raw_data = struct.unpack('<16I', buf[HEADER_SIZE:SENSOR_RESPONSE_SIZE])

    # barometer[0..7] → out[0..7]
    values = [uint32_to_float(raw_data[i]) for i in range(BAROMETER_COUNT)]
    # skip reserved[8..12]
    # tof[13..15] → out[8..10]
    values.extend(uint32_to_float(raw_data[BAROMETER_COUNT + RESERVED_COUNT + i])
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

    def __init__(self, target_ip: str = "192.168.1.100", target_port: int = 50002,
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

    def read_sensor(self, fingertip_idx: int) -> list[float] | None:
        """
        핑거팁 센서 데이터 요청 및 수신 (cmd=0x14..0x17, 3B → 67B)

        Args:
            fingertip_idx: 센서 인덱스 (0 ~ num_sensors-1)

        Returns:
            11 floats: barometer[8] + tof[3], or None on timeout
        """
        assert 0 <= fingertip_idx < self.num_sensors, \
            f"센서 인덱스 {fingertip_idx}은 연결된 센서 범위(0~{self.num_sensors - 1})를 초과합니다"
        return self._request_sensor(CMD_READ_SENSORS[fingertip_idx])

    def poll_cycle(self, positions: list[float]) -> dict:
        """
        HandController의 1 사이클과 동일한 플로우 실행:
          1. WritePosition                    → send 43B
          2. ReadPosition                     → send 43B, recv 43B
          3. ReadVelocity                     → send 43B, recv 43B
          4. ReadSensor × num_sensors         → send 3B, recv 67B

        Returns:
            dict with keys:
              - positions: [10 floats] or None
              - velocities: [10 floats] or None
              - sensors: [num_sensors × 11 floats]
        """
        # 1. 목표 위치 전송
        self.write_position(positions)

        # 2. 현재 위치 수신
        pos = self.read_position()

        # 3. 현재 속도 수신
        vel = self.read_velocity()

        # 4. 센서 데이터 수신 (연결된 센서만, 각 11개 유효 값)
        sensors = []
        for i in range(self.num_sensors):
            s = self.read_sensor(i)
            if s is not None:
                sensors.extend(s)
            else:
                sensors.extend([0.0] * SENSOR_VALUES_PER_FINGERTIP)

        return {
            "positions": pos,
            "velocities": vel,
            "sensors": sensors,
        }

    def read_cycle(self) -> dict:
        """
        Read request만 수행 (WritePosition 없음):
          1. ReadPosition          → send 43B, recv 43B
          2. ReadVelocity          → send 43B, recv 43B
          3. ReadSensor × num_sensors → send 3B, recv 67B

        Returns:
            dict with keys:
              - positions: [10 floats] or None
              - velocities: [10 floats] or None
              - sensors: [num_sensors × 11 floats]
        """
        pos = self.read_position()
        vel = self.read_velocity()

        sensors = []
        for i in range(self.num_sensors):
            s = self.read_sensor(i)
            if s is not None:
                sensors.extend(s)
            else:
                sensors.extend([0.0] * SENSOR_VALUES_PER_FINGERTIP)

        return {
            "positions": pos,
            "velocities": vel,
            "sensors": sensors,
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

    def _request_sensor(self, cmd: int) -> list[float] | None:
        """센서 요청 전송(3B) 후 응답 수신(67B)"""
        pkt = encode_sensor_request(cmd)
        self.sock.sendto(pkt, self.target)
        try:
            data, _ = self.sock.recvfrom(256)
            _, _mode, values = decode_sensor_response(data)
            return values
        except socket.timeout:
            return None

    def close(self):
        self.sock.close()


# ── 예제 함수 ────────────────────────────────────────────────────────────────

def example_write_only():
    """WritePosition만 전송하는 간단한 테스트 (사인파)"""
    sender = HandUDPSender(target_ip="127.0.0.1", target_port=50002)

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


def example_poll_cycle(num_sensors: int = NUM_FINGERTIPS):
    """HandController와 동일한 전체 poll 사이클 테스트"""
    sender = HandUDPSender(target_ip="127.0.0.1", target_port=50002,
                           num_sensors=num_sensors)

    print(f"\n전체 poll 사이클 실행 (센서 {num_sensors}개)...")
    print(f"  WritePosition(43B) → ReadPosition(43B↔43B) → ReadVelocity(43B↔43B) → ReadSensor×{num_sensors}(3B→67B)")
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

            # 1초마다 상태 출력
            if cycle % 500 == 0:
                pos_str = "None"
                if result["positions"]:
                    pos_str = f'[{", ".join(f"{p:.3f}" for p in result["positions"][:4])} ...]'
                vel_str = "None"
                if result["velocities"]:
                    vel_str = f'[{", ".join(f"{v:.3f}" for v in result["velocities"][:4])} ...]'
                n_sensors = len(result['sensors'])
                print(f"[cycle {cycle}] pos={pos_str}  vel={vel_str}  sensors={n_sensors} values ({num_sensors}x{SENSOR_VALUES_PER_FINGERTIP})")

            t += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\n중지됨 (총 {cycle} 사이클)")
    finally:
        sender.close()


def example_static_pose():
    """고정 포즈 전송 (WritePosition only)"""
    sender = HandUDPSender(target_ip="127.0.0.1", target_port=50002)

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


def example_read_only(num_sensors: int = NUM_FINGERTIPS):
    """Read request만 수행 (WritePosition 없이 현재 상태 읽기)"""
    sender = HandUDPSender(target_ip="127.0.0.1", target_port=50002,
                           num_sensors=num_sensors)

    print(f"\nRead only 사이클 실행 (센서 {num_sensors}개)...")
    print(f"  ReadPosition(43B↔43B) → ReadVelocity(43B↔43B) → ReadSensor×{num_sensors}(3B→67B)")
    print("Ctrl+C로 중지\n")

    try:
        cycle = 0
        dt = 0.002  # 500 Hz
        while True:
            result = sender.read_cycle()
            cycle += 1

            if cycle % 500 == 0:
                pos_str = "None"
                if result["positions"]:
                    pos_str = f'[{", ".join(f"{p:.3f}" for p in result["positions"][:4])} ...]'
                vel_str = "None"
                if result["velocities"]:
                    vel_str = f'[{", ".join(f"{v:.3f}" for v in result["velocities"][:4])} ...]'
                n_sensors = len(result['sensors'])
                print(f"[cycle {cycle}] pos={pos_str}  vel={vel_str}  sensors={n_sensors} values ({num_sensors}x{SENSOR_VALUES_PER_FINGERTIP})")

            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\n중지됨 (총 {cycle} 사이클)")
    finally:
        sender.close()


def main(args=None):
    print("=" * 65)
    print("  Hand UDP Sender Example")
    print("=" * 65)
    print(f"\n  모터 패킷:  {MOTOR_PACKET_SIZE}B = [ID:1B][CMD:1B][MODE:1B][data:10×float32]")
    print(f"  센서 요청:  {SENSOR_REQUEST_SIZE}B = [ID:1B][CMD:1B][MODE:1B]")
    print(f"  센서 응답:  {SENSOR_RESPONSE_SIZE}B = [ID:1B][CMD:1B][MODE:1B][data:16×uint32]")
    print(f"             → barometer[{BAROMETER_COUNT}] + reserved[{RESERVED_COUNT}](skip) + tof[{TOF_COUNT}] = {SENSOR_VALUES_PER_FINGERTIP} 유효 값")
    print(f"  모터: {NUM_HAND_MOTORS}개, 핑거팁: 최대 {NUM_FINGERTIPS}개, 센서 값/팁: {SENSOR_VALUES_PER_FINGERTIP}개")
    print()

    # 연결된 센서 수 입력
    num_sensors_str = input(f"연결된 센서 수 (0~{NUM_FINGERTIPS}, 기본={NUM_FINGERTIPS}): ").strip()
    num_sensors = int(num_sensors_str) if num_sensors_str else NUM_FINGERTIPS
    assert 0 <= num_sensors <= NUM_FINGERTIPS, f"센서 수는 0~{NUM_FINGERTIPS} 범위여야 합니다"
    print(f"  → 센서 {num_sensors}개 사용 (cmd: {', '.join(f'0x{CMD_READ_SENSORS[i]:02x}' for i in range(num_sensors)) or 'none'})")
    print()

    print("모드 선택:")
    print("  1) WritePosition 사인파 (전송만)")
    print("  2) 전체 poll 사이클 (WritePos + ReadPos + ReadVel + ReadSensor)")
    print("  3) 고정 포즈 전송")
    print("  4) Read only (ReadPos + ReadVel + ReadSensor, 쓰기 없음)")
    print()

    choice = input("선택 (기본=1): ").strip() or "1"

    if choice == "1":
        example_write_only()
    elif choice == "2":
        example_poll_cycle(num_sensors=num_sensors)
    elif choice == "3":
        example_static_pose()
    elif choice == "4":
        example_read_only(num_sensors=num_sensors)
    else:
        print("잘못된 선택")


if __name__ == "__main__":
    main()
