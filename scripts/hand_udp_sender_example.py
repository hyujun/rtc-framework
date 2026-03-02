#!/usr/bin/env python3
"""
커스텀 핸드 UDP 송신 예제
실제 핸드 시스템에서 이 형식으로 UDP 패킷을 보내야 합니다.
"""

import socket
import struct
import time
import numpy as np

class HandUDPSender:
    def __init__(self, target_ip="127.0.0.1", target_port=50001):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (target_ip, target_port)
        
        # 패킷 형식: 77개 double (616 bytes)
        # - motor_pos[11]      : 88 bytes
        # - motor_vel[11]      : 88 bytes
        # - motor_current[11]  : 88 bytes
        # - sensor_data[44]    : 352 bytes
        self.packet_format = '77d'  # 77개의 double
        self.packet_size = struct.calcsize(self.packet_format)
        
        print(f"🌐 Hand UDP Sender")
        print(f"   Target: {target_ip}:{target_port}")
        print(f"   Packet size: {self.packet_size} bytes")
        
    def send_hand_state(self, motor_pos, motor_vel, motor_current, sensor_data):
        """
        핸드 상태 전송
        
        Args:
            motor_pos: 11개 모터 위치 (list or np.array)
            motor_vel: 11개 모터 속도
            motor_current: 11개 모터 전류
            sensor_data: 44개 센서 데이터 (4센서 × 11개)
        """
        # 데이터 검증
        assert len(motor_pos) == 11, "motor_pos must have 11 elements"
        assert len(motor_vel) == 11, "motor_vel must have 11 elements"
        assert len(motor_current) == 11, "motor_current must have 11 elements"
        assert len(sensor_data) == 44, "sensor_data must have 44 elements"
        
        # 패킷 생성 (순서 중요!)
        packet_data = list(motor_pos) + list(motor_vel) + list(motor_current) + list(sensor_data)
        
        # 바이너리로 변환
        packet_bytes = struct.pack(self.packet_format, *packet_data)
        
        # UDP 전송
        self.sock.sendto(packet_bytes, self.target)
        
    def close(self):
        self.sock.close()


def example_sine_wave():
    """사인파 테스트 데이터 생성"""
    sender = HandUDPSender(target_ip="127.0.0.1", target_port=50001)
    
    print("\n▶️  Sending test data (sine wave)...")
    print("   Press Ctrl+C to stop\n")
    
    try:
        t = 0.0
        dt = 0.002  # 500 Hz
        
        while True:
            # 사인파 모터 위치 (0~1 범위)
            motor_pos = [0.5 + 0.3 * np.sin(2 * np.pi * 0.5 * t + i * 0.2) for i in range(11)]
            
            # 속도 (위치 미분 근사)
            motor_vel = [0.3 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t + i * 0.2) for i in range(11)]
            
            # 전류 (랜덤 노이즈)
            motor_current = [0.5 + 0.1 * np.random.randn() for _ in range(11)]
            
            # 센서 데이터 (4개 센서 × 11개)
            sensor_data = []
            for sensor_id in range(4):
                for data_id in range(11):
                    value = np.sin(2 * np.pi * 0.2 * t + sensor_id + data_id * 0.1)
                    sensor_data.append(value)
            
            # 전송
            sender.send_hand_state(motor_pos, motor_vel, motor_current, sensor_data)
            
            # 통계 출력 (1초마다)
            if int(t * 10) % 10 == 0:
                print(f"[{t:.2f}s] Pos[0]={motor_pos[0]:.3f}, Vel[0]={motor_vel[0]:.3f}, Sensor[0]={sensor_data[0]:.3f}")
            
            t += dt
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\n\n✅ Stopped")
    finally:
        sender.close()


def example_static_pose():
    """고정 포즈 전송"""
    sender = HandUDPSender(target_ip="127.0.0.1", target_port=50001)
    
    # 고정 값
    motor_pos = [0.1 * i for i in range(11)]
    motor_vel = [0.0] * 11
    motor_current = [0.5] * 11
    sensor_data = [0.0] * 44
    
    print("\n▶️  Sending static pose...")
    print("   Press Ctrl+C to stop\n")
    
    try:
        while True:
            sender.send_hand_state(motor_pos, motor_vel, motor_current, sensor_data)
            time.sleep(0.002)  # 500 Hz
    except KeyboardInterrupt:
        print("\n\n✅ Stopped")
    finally:
        sender.close()


if __name__ == "__main__":
    import sys
    
    print("=" * 50)
    print("  Custom Hand UDP Sender Example")
    print("=" * 50)
    print("\nSelect mode:")
    print("  1) Sine wave test (동적)")
    print("  2) Static pose (정적)")
    print()
    
    choice = input("Enter 1 or 2 (default=1): ").strip() or "1"
    
    if choice == "1":
        example_sine_wave()
    elif choice == "2":
        example_static_pose()
    else:
        print("Invalid choice")
        sys.exit(1)
