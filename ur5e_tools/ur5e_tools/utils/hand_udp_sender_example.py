#!/usr/bin/env python3
"""
핸드 UDP 송신 예제 — ur5e_hand_udp 패키지 프로토콜 기반

프로토콜 사양 (hand_packets.hpp 참조):
  - 패킷 크기: 43 bytes (little-endian)
  - 헤더: [ID: 0x01] [CMD: 1B] [MODE: 0x00]
  - 데이터: 10 x uint32_t (float를 memcpy로 변환)

커맨드 코드:
  - WritePosition  0x01 : 모터 10개 목표 위치 전송
  - ReadPosition   0x11 : 모터 위치 요청 → 응답 수신
  - ReadVelocity   0x12 : 모터 속도 요청 → 응답 수신
  - ReadSensor0~3  0x14..0x17 : 핑거팁 센서 데이터 요청 → 응답 수신

통신 플로우 (1 사이클):
  1. WritePosition(cmd=0x01) → 10 float
  2. ReadPosition(cmd=0x11)  → send request, recv 10 float
  3. ReadVelocity(cmd=0x12)  → send request, recv 10 float
  4. ReadSensor0~3(cmd=0x14..0x17) → send request, recv 10 float x 4

ROS2 토픽 (hand_udp_node):
  - pub: /hand/joint_states  → positions[10] + velocities[10] + sensors[40] = 60
  - sub: /hand/command       → motor_commands[10]
"""

import socket
import struct
import time
import numpy as np

# ── 프로토콜 상수 (hand_packets.hpp 기반) ────────────────────────────────────
DEVICE_ID = 0x01
DEFAULT_MODE = 0x00

HEADER_SIZE = 3       # ID + CMD + MODE
DATA_COUNT = 10       # float 10개
PACKET_SIZE = HEADER_SIZE + DATA_COUNT * 4  # 43 bytes

NUM_HAND_MOTORS = 10
NUM_FINGERTIPS = 4
SENSOR_VALUES_PER_FINGERTIP = 10
NUM_HAND_SENSORS = NUM_FINGERTIPS * SENSOR_VALUES_PER_FINGERTIP  # 40

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


# ── 패킷 인코딩/디코딩 (hand_udp_codec.hpp 기반) ────────────────────────────

def float_to_uint32(f: float) -> int:
    """float → uint32 (memcpy 동등)"""
    return struct.unpack('<I', struct.pack('<f', f))[0]


def uint32_to_float(raw: int) -> float:
    """uint32 → float (memcpy 동등)"""
    return struct.unpack('<f', struct.pack('<I', raw))[0]


def encode_packet(cmd: int, floats: list[float] | None = None) -> bytes:
    """
    43바이트 패킷 인코딩

    Args:
        cmd: 커맨드 코드 (예: CMD_WRITE_POSITION)
        floats: 10개 float 데이터 (None이면 0으로 채움)
    """
    header = struct.pack('<BBB', DEVICE_ID, cmd, DEFAULT_MODE)
    if floats is not None:
        assert len(floats) == DATA_COUNT, f"데이터는 {DATA_COUNT}개여야 합니다"
        data = struct.pack('<10I', *[float_to_uint32(f) for f in floats])
    else:
        data = b'\x00' * (DATA_COUNT * 4)
    return header + data


def decode_packet(buf: bytes) -> tuple[int, list[float]]:
    """
    43바이트 패킷 디코딩

    Returns:
        (cmd, [10 floats])
    """
    assert len(buf) >= PACKET_SIZE, f"패킷 크기 부족: {len(buf)} < {PACKET_SIZE}"
    _id, cmd, _mode = struct.unpack('<BBB', buf[:HEADER_SIZE])
    raw_data = struct.unpack('<10I', buf[HEADER_SIZE:PACKET_SIZE])
    floats = [uint32_to_float(r) for r in raw_data]
    return cmd, floats


# ── HandUDPSender 클래스 ─────────────────────────────────────────────────────

class HandUDPSender:
    """
    핸드 UDP request-response 통신 클래스

    hand_controller.hpp의 HandController와 동일한 프로토콜 구현.
    실제 핸드 디바이스 또는 hand_udp_node 테스트용.
    """

    def __init__(self, target_ip: str = "192.168.1.100", target_port: int = 50002,
                 recv_timeout: float = 0.01):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(recv_timeout)  # 10ms recv timeout (hand_controller.hpp 동일)
        self.target = (target_ip, target_port)

        print(f"Hand UDP Sender (43-byte protocol)")
        print(f"  Target: {target_ip}:{target_port}")
        print(f"  Packet size: {PACKET_SIZE} bytes")
        print(f"  Motors: {NUM_HAND_MOTORS}, Fingertips: {NUM_FINGERTIPS}")

    def write_position(self, positions: list[float]) -> None:
        """모터 목표 위치 전송 (cmd=0x01)"""
        assert len(positions) == NUM_HAND_MOTORS
        pkt = encode_packet(CMD_WRITE_POSITION, positions)
        self.sock.sendto(pkt, self.target)

    def read_position(self) -> list[float] | None:
        """모터 현재 위치 요청 및 수신 (cmd=0x11)"""
        return self._request(CMD_READ_POSITION)

    def read_velocity(self) -> list[float] | None:
        """모터 현재 속도 요청 및 수신 (cmd=0x12)"""
        return self._request(CMD_READ_VELOCITY)

    def read_sensor(self, fingertip_idx: int) -> list[float] | None:
        """핑거팁 센서 데이터 요청 및 수신 (cmd=0x14..0x17)"""
        assert 0 <= fingertip_idx < NUM_FINGERTIPS
        return self._request(CMD_READ_SENSORS[fingertip_idx])

    def poll_cycle(self, positions: list[float]) -> dict:
        """
        HandController의 1 사이클과 동일한 플로우 실행:
          1. WritePosition
          2. ReadPosition → recv
          3. ReadVelocity → recv
          4. ReadSensor0~3 → recv x 4

        Returns:
            dict with keys: positions, velocities, sensors (or None if timeout)
        """
        # 1. 목표 위치 전송
        self.write_position(positions)

        # 2. 현재 위치 수신
        pos = self.read_position()

        # 3. 현재 속도 수신
        vel = self.read_velocity()

        # 4. 센서 데이터 수신 (4개 핑거팁)
        sensors = []
        for i in range(NUM_FINGERTIPS):
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

    def _request(self, cmd: int) -> list[float] | None:
        """요청 전송 후 응답 수신"""
        pkt = encode_packet(cmd)
        self.sock.sendto(pkt, self.target)
        try:
            data, _ = self.sock.recvfrom(256)
            _, floats = decode_packet(data)
            return floats
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


def example_poll_cycle():
    """HandController와 동일한 전체 poll 사이클 테스트"""
    sender = HandUDPSender(target_ip="127.0.0.1", target_port=50002)

    print("\n전체 poll 사이클 실행...")
    print("  WritePosition → ReadPosition → ReadVelocity → ReadSensor0~3")
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
                print(f"[cycle {cycle}] pos={pos_str}  vel={vel_str}  sensors={len(result['sensors'])} values")

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


def main(args=None):
    print("=" * 55)
    print("  Hand UDP Sender Example (43-byte protocol)")
    print("=" * 55)
    print(f"\n  패킷: {PACKET_SIZE} bytes = [ID:1B][CMD:1B][MODE:1B][data:10xfloat32]")
    print(f"  모터: {NUM_HAND_MOTORS}개, 핑거팁: {NUM_FINGERTIPS}개, 센서: {NUM_HAND_SENSORS}개")
    print()
    print("모드 선택:")
    print("  1) WritePosition 사인파 (전송만)")
    print("  2) 전체 poll 사이클 (WritePos + ReadPos + ReadVel + ReadSensor)")
    print("  3) 고정 포즈 전송")
    print()

    choice = input("선택 (기본=1): ").strip() or "1"

    if choice == "1":
        example_write_only()
    elif choice == "2":
        example_poll_cycle()
    elif choice == "3":
        example_static_pose()
    else:
        print("잘못된 선택")


if __name__ == "__main__":
    main()
