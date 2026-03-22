# rtc_digital_twin

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **RViz2 디지털 트윈 시각화** 패키지입니다. 500 Hz 로봇/핸드 상태를 60 Hz로 다운샘플링하여 RViz2에서 실시간 시각화합니다.

**핵심 기능:**
- UR5e 6-DOF + 10-DOF 커스텀 핸드 통합 시각화
- 핑거팁 센서 시각화 (기압계 구 + ToF 화살표 마커)
- URDF 기반 TF 트리 자동 구성 (`robot_state_publisher`)
- 설정 가능한 디스플레이 레이트 (기본 60 Hz)

---

## 패키지 구조

```
rtc_digital_twin/
├── package.xml
├── setup.py
├── setup.cfg
├── rtc_digital_twin/
│   ├── __init__.py
│   ├── digital_twin_node.py      ← 메인 시각화 노드 (500Hz→60Hz)
│   └── sensor_visualizer.py      ← 핑거팁 센서 → MarkerArray 변환
├── launch/
│   └── digital_twin.launch.py    ← RViz2 + robot_state_publisher 통합
└── config/
    ├── digital_twin.yaml         ← 파라미터 설정
    └── digital_twin.rviz         ← RViz2 디스플레이 설정
```

---

## ROS2 인터페이스

### 구독

| 토픽 | 타입 | 주파수 | 설명 |
|------|------|--------|------|
| `/joint_states` | `JointState` | 500 Hz | UR5e 6-DOF 관절 상태 |
| `/hand/joint_states` | `Float64MultiArray` | 100 Hz | [pos:10][vel:10][sensors:44] |

### 퍼블리시

| 토픽 | 타입 | 주파수 | 설명 |
|------|------|--------|------|
| `/digital_twin/joint_states` | `JointState` | 60 Hz | 로봇+핸드 통합 관절 상태 → robot_state_publisher |
| `/digital_twin/fingertip_markers` | `MarkerArray` | 60 Hz | 센서 시각화 마커 |

### 센서 시각화

핑거팁 당 11개 센서 (8 barometer + 3 ToF):

| 센서 | 마커 타입 | 스케일링 | 범위 |
|------|----------|----------|------|
| Barometer (×8) | Sphere | 크기+색상 ∝ 압력 | 0–1000 Pa |
| ToF (×3) | Arrow | 길이 ∝ 거리 | 0–0.2 m |

색상: 파랑(낮음) → 초록 → 빨강(높음) 히트맵

---

## 실행

```bash
# 기본 실행 (RViz2 포함)
ros2 launch rtc_digital_twin digital_twin.launch.py

# 핸드 비활성화
ros2 launch rtc_digital_twin digital_twin.launch.py enable_hand:=false

# 디스플레이 레이트 변경
ros2 launch rtc_digital_twin digital_twin.launch.py display_rate:=30.0

# RViz2 없이 (외부 RViz 사용)
ros2 launch rtc_digital_twin digital_twin.launch.py use_rviz:=false
```

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `rclpy` | ROS2 Python 클라이언트 |
| `sensor_msgs` | JointState 메시지 |
| `visualization_msgs` | MarkerArray |
| `ur5e_description` | URDF 모델 |
| `robot_state_publisher` | URDF → TF 변환 |
| `rviz2` | 3D 시각화 |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_digital_twin
source install/setup.bash
```

---

## 의존성 그래프 내 위치

```
ur5e_description  ← URDF 모델 제공
    ↓
rtc_digital_twin  ← RViz2 시각화 (독립, Python)
```

---

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | Python 패키지 — RViz2 노드 구조 확인 완료 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
