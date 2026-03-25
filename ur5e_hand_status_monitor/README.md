# ur5e_hand_status_monitor

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **UR5e + 핸드 통합 상태 모니터링** 패키지입니다. `rtc_status_monitor`의 로봇 감시 기능을 확장하여 핸드 디바이스(모터 데이터, 센서 데이터, 메시지 레이트)에 대한 전용 모니터링을 추가합니다.

**핵심 기능:**
- 로봇 + 핸드 통합 상태 감시 (Composition 패턴)
- 핸드 모터 데이터 품질 검사 (전-영, 중복 감지)
- 핸드 센서 데이터 품질 검사 (기압 + ToF)
- 메시지 레이트 모니터링 (최소 주파수 보장)
- 500 Hz RT 제어 스레드에 lock-free 상태 제공 (atomic)

---

## 패키지 구조

```
ur5e_hand_status_monitor/
├── CMakeLists.txt
├── package.xml
├── include/ur5e_hand_status_monitor/
│   └── ur5e_hand_status_monitor.hpp    ← 메인 통합 모니터 클래스
├── src/
│   └── ur5e_hand_status_monitor.cpp    ← 구현
└── config/
    └── ur5e_hand_status_monitor.yaml   ← 모니터링 설정
```

---

## 아키텍처

```
┌──────────────────────────────────────────────────────────┐
│  Ur5eHandStatusMonitor (Composition)                     │
│                                                          │
│  ┌─────────────────────────────┐  ┌────────────────────┐ │
│  │ RtcStatusMonitor (10 Hz)   │  │ HandMonitorLoop    │ │
│  │ - 로봇 모드/안전 모드 감시  │  │ (10 Hz jthread)    │ │
│  │ - 추적 오차 검사            │  │ - 모터 데이터 검사  │ │
│  │ - 관절 한계 검사            │  │ - 센서 데이터 검사  │ │
│  │ - 컨트롤러 상태 폴링        │  │ - 레이트 모니터링   │ │
│  └─────────────┬───────────────┘  └────────┬───────────┘ │
│                │ atomic store               │            │
│                ▼                             ▼            │
│  ┌───────────────────────────────────────────────────┐   │
│  │  500 Hz RT 제어 스레드                              │   │
│  │  isReady()             → atomic (로봇 + 핸드 통합)  │   │
│  │  getFailure()          → atomic (로봇 또는 핸드)    │   │
│  │  isJointLimitWarning() → atomic                    │   │
│  │  setJointReference()   → try_lock (비차단)          │   │
│  └───────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

---

## 핸드 장애 유형

| 장애 유형 | 트리거 | 임계값 |
|----------|--------|--------|
| `kHandTimeout` | `/hand/joint_states` 수신 > `watchdog_timeout_sec` | 1.0초 |
| `kHandMotorDataFault` | 전-영 또는 중복 모터 위치 데이터 | N회 연속 (기본 5) |
| `kHandSensorDataFault` | 전-영 또는 중복 센서 데이터 | N회 연속 (기본 5) |
| `kHandRateLow` | 메시지 주파수 < `min_rate_hz` | N회 연속 (기본 5) |

### 데이터 품질 검사 상세

**모터 검사 (`check_motor`):**
- 전-영 감지: 모든 위치 값이 ≈0 (임계값: 1e-9)
- 중복 감지: 이전 프레임과 동일한 위치 (허용치: 1e-12)
- 연속 N회 실패 시 `kHandMotorDataFault` 발생

**센서 검사 (`check_sensor`):**
- 모든 핑거팁의 기압 + ToF 값 추출
- 전-영/중복 검사 동일 로직 적용
- 연속 N회 실패 시 `kHandSensorDataFault` 발생

**레이트 모니터링:**
- 1초 윈도우 단위로 Hz 측정
- 첫 번째 저율 감지 시 경고 (`kHandRateDegraded`)
- 연속 N초 `min_rate_hz` 미달 시 장애 (`kHandRateLow`)

---

## RT 스레드 인터페이스

RT 제어 스레드에서 **절대 차단 없이** 호출 가능한 메서드입니다.

| 메서드 | 메커니즘 | 설명 |
|--------|---------|------|
| `isReady()` | `atomic<bool>` acquire | 로봇 + 핸드 통합 준비 상태 |
| `getFailure()` | `atomic<FailureType>` acquire | 현재 장애 유형 (로봇 또는 핸드) |
| `isJointLimitWarning()` | `atomic<bool>` acquire | 관절 한계 경고 |
| `getJointStateStats()` | 원자적 카운터 | 로봇 메시지 통계 |
| `getHandStats()` | 원자적 카운터 | 핸드 메시지 통계 |
| `setJointReference()` | `try_lock` (비차단) | 레퍼런스 위치/속도 전달 |
| `waitForReady()` | 블로킹 (50ms 폴링) | 초기화 대기 |

---

## 콜백 등록

| 콜백 | 시그니처 | 트리거 |
|------|---------|--------|
| `registerOnReady()` | `void()` | 로봇 + 핸드 모두 준비 완료 |
| `registerOnFailure()` | `void(FailureType, const FailureContext&)` | 장애 감지 (로봇 또는 핸드) |
| `registerOnWarning()` | `void(WarningType, const std::string&)` | 경고 조건 |
| `registerOnRecovery()` | `void()` | 자동 복구 성공 |

---

## ROS2 인터페이스

### 구독 (핸드 전용)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/hand/joint_states` | `sensor_msgs/JointState` | 핸드 모터 위치/속도 |
| `/hand/sensor_states` | `rtc_msgs/HandSensorState` | 핑거팁 센서 (기압 + ToF) |

### 로봇 토픽 (RtcStatusMonitor 위임)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 로봇 관절 피드백 |
| `/io_and_status_controller/robot_mode` | `Int32` | UR5e 동작 모드 |
| `/io_and_status_controller/safety_mode` | `Int32` | UR5e 안전 상태 |
| `/io_and_status_controller/robot_program_running` | `Bool` | 프로그램 실행 플래그 |

---

## 설정

### ur5e_hand_status_monitor.yaml

```yaml
/**:
  ros__parameters:
    status_monitor:
      # 로봇 모니터링 (RtcStatusMonitor 위임)
      watchdog_timeout_sec: 1.0
      tracking_error_pos_warn_rad: 0.05
      tracking_error_pos_fault_rad: 0.15
      tracking_error_vel_warn_rad: 0.1
      tracking_error_vel_fault_rad: 0.3
      joint_limit_warn_margin_deg: 5.0
      joint_limit_fault_margin_deg: 1.0
      controller_poll_interval_sec: 5.0
      target_controller: "scaled_joint_trajectory_controller"
      auto_recovery: false

      # 핸드 모니터링
      hand:
        enabled: true
        joint_states_topic: "/hand/joint_states"
        sensor_states_topic: "/hand/sensor_states"
        watchdog_timeout_sec: 1.0
        min_rate_hz: 30.0
        rate_fail_threshold: 5
        data_fail_threshold: 5
        check_motor: true
        check_sensor: true
```

---

## 스레딩 모델

두 개의 독립 백그라운드 스레드가 `std::jthread` + `std::stop_token`으로 관리됩니다.

| 스레드 | 주파수 | 역할 |
|--------|--------|------|
| **RtcStatusMonitor** | 10 Hz | 로봇 모드/안전/추적 오차/관절 한계/컨트롤러 상태 |
| **HandMonitorLoop** | 10 Hz | 핸드 워치독/모터 검사/센서 검사/레이트 측정 |

**동기화:**
- 원자적 변수: `hand_state_received_`, `last_hand_state_time_ns_`, `hand_failure_type_`
- 뮤텍스: `hand_data_mutex_` (모터/센서 값 보호)
- 콜백: ROS2 executor 컨텍스트에서 실행

---

## 사용 예시

```cpp
auto node = std::make_shared<rclcpp::Node>("rt_controller");
auto monitor = std::make_unique<Ur5eHandStatusMonitor>(node);

// 콜백 등록
monitor->registerOnReady([]() { /* 시스템 준비 */ });
monitor->registerOnFailure([](auto type, const auto& ctx) {
  // 장애 유형에 따라 E-STOP 트리거
});

// 백그라운드 모니터링 시작
monitor->start(kStatusMonitorConfig);

// RT 루프에서 대기
if (!monitor->waitForReady(10.0)) return EXIT_FAILURE;

// 500 Hz RT 제어 루프
while (running) {
  if (!monitor->isReady()) { /* E-STOP */ break; }
  monitor->setJointReference(q_ref, qd_ref);  // 비차단
  // ... 제어 연산 ...
}

monitor->stop();
```

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rclcpp` | ROS2 클라이언트 |
| `sensor_msgs` | JointState |
| `rtc_base` | 타입, 스레딩 유틸리티 |
| `rtc_status_monitor` | 로봇 상태 모니터링 (Composition) |
| `rtc_msgs` | HandSensorState 메시지 |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_hand_status_monitor
source install/setup.bash
```

공유 라이브러리(`libur5e_hand_status_monitor.so`)가 생성됩니다.

---

## 의존성 그래프 내 위치

```
rtc_base + rtc_status_monitor + rtc_msgs
    ↓
ur5e_hand_status_monitor  ← 로봇 + 핸드 통합 모니터
    ↑
    └── rtc_controller_manager  (선택적, RT 스레드에 atomic 상태 제공)
```

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
