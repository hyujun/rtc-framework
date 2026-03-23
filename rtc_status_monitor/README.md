# rtc_status_monitor

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **비-RT 상태 모니터링 컴포넌트** 패키지입니다. 10 Hz 모니터링 루프에서 로봇 모드, 안전 모드, 추적 오차, 관절 한계를 감시하고, 500 Hz RT 제어 스레드에 원자적(lock-free) 상태를 제공합니다.

**핵심 기능:**
- 10개 장애 유형 + 3개 경고 유형 감지
- RT 스레드에 원자적 상태 접근 (절대 차단 없음)
- 장애 발생 시 10초 상태 히스토리 기록
- 컨트롤러별 통계 수집 (패킷 수, 활성 시간, 장애 수)
- 선택적 자동 복구 (protective stop, 프로그램 연결 끊김)

---

## 패키지 구조

```
rtc_status_monitor/
├── CMakeLists.txt
├── package.xml
├── include/rtc_status_monitor/
│   ├── rtc_status_monitor.hpp     ← 메인 모니터 클래스
│   ├── failure_types.hpp          ← 장애/경고 열거형 + UR 모드 정의
│   └── state_history.hpp          ← 순환 버퍼 (10초 히스토리)
├── src/
│   └── rtc_status_monitor.cpp     ← 구현 (1176 lines)
└── config/
    └── rtc_status_monitor.yaml    ← 모니터링 설정
```

---

## 아키텍처

```
                          10 Hz 모니터 루프
                    ┌──────────────────────────┐
                    │ CheckJointStateWatchdog() │
                    │ CheckRobotStatus()        │
                    │ CheckTrackingErrors()     │
                    │ CheckJointLimits()        │
                    │ CheckControllerManager()  │
                    │ CheckReadiness()          │
                    │ PublishDiagnostics() @1Hz  │
                    └─────────┬────────────────┘
                              │ atomic store (release)
                              ▼
┌──────────────────────────────────────────────────┐
│  500 Hz RT 제어 스레드                             │
│  isReady()             → atomic load (acquire)    │
│  getFailure()          → atomic load (acquire)    │
│  isJointLimitWarning() → atomic load (acquire)    │
│  setJointReference()   → try_lock (절대 차단 안 함) │
└──────────────────────────────────────────────────┘
```

---

## 장애 유형 (FailureType)

| 장애 유형 | 트리거 | 자동 복구 |
|----------|--------|----------|
| `kNone` | 초기 상태 / 복구 성공 | — |
| `kEstop` | SafeguardStop, SystemEmergencyStop, RobotEmergencyStop | 불가 |
| `kProtectiveStop` | safety_mode = ProtectiveStop | **가능** |
| `kSafetyViolation` | safety_mode = Violation 또는 Fault | 불가 |
| `kProgramDisconnected` | 준비 상태에서 프로그램 실행 중지 | **가능** |
| `kWatchdogTimeout` | joint_states 수신 > watchdog_timeout_sec | 불가 |
| `kControllerInactive` | 대상 컨트롤러 비활성 | 불가 |
| `kHardwareFault` | 하드웨어 상태 이상 | 불가 |
| `kTrackingError` | 위치/속도 추적 오차 > fault 임계값 | 불가 |
| `kJointLimitViolation` | 관절 위치가 fault 마진 내 진입 | 불가 |
| `kHandTimeout` | `/hand/joint_states` 수신 타임아웃 | 불가 |
| `kHandMotorDataFault` | 핸드 모터 전-영/중복 데이터 감지 | 불가 |
| `kHandSensorDataFault` | 핸드 센서 전-영/중복 데이터 감지 | 불가 |
| `kHandRateLow` | 핸드 메시지 레이트 < 최소 허용 Hz | 불가 |

> 핸드 관련 장애 유형은 `ur5e_hand_status_monitor` 패키지에서 확장 사용됩니다.

## 경고 유형 (WarningType)

| 경고 유형 | 트리거 |
|----------|--------|
| `kJointLimitProximity` | 관절 위치가 warn 마진 내 진입 |
| `kTrackingErrorHigh` | 추적 오차 > warn 임계값 (fault 미만) |
| `kHighLatency` | 예약됨 (미사용) |
| `kHandRateDegraded` | 핸드 메시지 레이트 저하 (fault 미만) |

---

## 모니터링 루프 (10 Hz)

매 100 ms마다 순차적으로 실행됩니다:

1. **CheckJointStateWatchdog()** — `/joint_states` 수신 타임아웃 검사
2. **CheckRobotStatus()** — 안전 모드, 비상 정지, 프로그램 실행 상태
3. **CheckTrackingErrors()** — 레퍼런스 대비 실제 위치/속도 오차 (try_lock)
4. **CheckJointLimits()** — 관절 위치가 한계 마진에 근접
5. **CheckControllerManager()** — 대상 컨트롤러 활성 상태 (매 5초)
6. **CheckReadiness()** — 모든 조건 통합 → `is_ready_` 원자적 갱신
7. **StateHistory Push** — 순환 버퍼에 현재 상태 기록 (100 엔트리 = 10초)
8. **PublishDiagnostics()** — `/diagnostics` 토픽 퍼블리시 (매 1초)

---

## RT 스레드 인터페이스

RT 제어 스레드에서 **절대 차단 없이** 호출 가능한 메서드입니다.

| 메서드 | 메커니즘 | 설명 |
|--------|---------|------|
| `isReady()` | `atomic<bool>` acquire | 준비 상태 조회 |
| `getFailure()` | `atomic<FailureType>` acquire | 현재 장애 유형 |
| `isJointLimitWarning()` | `atomic<bool>` acquire | 관절 한계 경고 |
| `getJointStateStats()` | 원자적 카운터 | 메시지 통계 (수신 수, 타임아웃, 주파수) |
| `setJointReference()` | `try_lock` (비차단) | 레퍼런스 위치/속도 전달 |

---

## 자동 복구

`kProtectiveStop`과 `kProgramDisconnected`에 대해서만 자동 복구를 시도합니다.

1. `auto_recovery` 활성화 확인
2. 쿨다운 확인 (`recovery_interval_sec`)
3. 최대 시도 횟수 확인 (`max_recovery_attempts`)
4. UR Dashboard 서비스 호출 (unlock_protective_stop 또는 play)
5. `ClearFailure()` → 2초 대기 → safety_mode 확인
6. 성공: `recovery_attempts_` 초기화 + `on_recovery_cb_` 호출

> `auto_recovery`는 기본 비활성 (`false`)입니다. 활성화 시 주의가 필요합니다.

---

## ROS2 인터페이스

### 구독

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/joint_states` | `JointState` | SensorDataQoS | 관절 위치/속도 |
| `/io_and_status_controller/robot_mode` | `Int32` | depth=10 | 로봇 모드 |
| `/io_and_status_controller/safety_mode` | `Int32` | depth=10 | 안전 모드 |
| `/io_and_status_controller/robot_program_running` | `Bool` | depth=10 | 프로그램 실행 |
| `/ur5e/active_controller_name` | `String` | TRANSIENT_LOCAL | 컨트롤러 통계용 |

### 퍼블리셔

| 토픽 | 타입 | 주파수 | 설명 |
|------|------|--------|------|
| `/diagnostics` | `DiagnosticArray` | 1 Hz | 시스템 상태 진단 |

### 서비스 클라이언트

| 서비스 | 타입 | 주기 | 설명 |
|--------|------|------|------|
| `/controller_manager/list_controllers` | `ListControllers` | 5초 | 대상 컨트롤러 활성 확인 |

---

## 설정

### rtc_status_monitor.yaml

```yaml
/**:
  ros__parameters:
    status_monitor:
      # 토픽
      joint_states_topic:    "/joint_states"
      robot_mode_topic:      "/io_and_status_controller/robot_mode"
      safety_mode_topic:     "/io_and_status_controller/safety_mode"
      program_running_topic: "/io_and_status_controller/robot_program_running"

      # 워치독
      watchdog_timeout_sec: 1.0

      # 컨트롤러 매니저 폴링
      controller_poll_interval_sec: 5.0
      target_controller: "scaled_joint_trajectory_controller"

      # 추적 오차 임계값 (rad)
      tracking_error_pos_warn_rad:  0.05
      tracking_error_pos_fault_rad: 0.15
      tracking_error_vel_warn_rad:  0.1
      tracking_error_vel_fault_rad: 0.3

      # 관절 한계 마진 (deg)
      joint_limit_warn_margin_deg:  5.0
      joint_limit_fault_margin_deg: 1.0

      # 자동 복구
      auto_recovery: false
      max_recovery_attempts: 3
      recovery_interval_sec: 5.0

      # 로깅
      enable_controller_stats: true
      log_output_dir: ""
```

---

## 장애 로깅

장애 발생 시 상세 로그 파일이 생성됩니다:

- **파일명:** `ur5e_failure_YYYYMMDDTHHMMSS_mmm.log`
- **내용:**
  - 장애 유형, 타임스탬프, 로봇/안전 모드
  - 장애 시점 관절 상태 (실제 + 레퍼런스 + 오차)
  - 10초 상태 히스토리 (CSV, 10 Hz 샘플)
- **경로:** `log_output_dir` → `UR5E_SESSION_DIR/monitor/` → `~/.ros` → `/tmp`

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rclcpp` | ROS2 클라이언트 |
| `std_msgs` | Int32, Bool, String |
| `sensor_msgs` | JointState |
| `diagnostic_msgs` | DiagnosticArray |
| `controller_manager_msgs` | ListControllers 서비스 |
| `rtc_base` | 타입, 스레딩 유틸리티 |
| `ur_dashboard_msgs` | (선택) UR 대시보드 복구 서비스 |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_status_monitor
source install/setup.bash
```

공유 라이브러리(`librtc_status_monitor.so`)가 생성됩니다.

---

## E-STOP 시스템 연동

모니터가 장애를 감지하면 RT 컨트롤러(`RtControllerNode`)가 원자적 상태를 읽어 글로벌 E-STOP을 트리거합니다:

```
rtc_status_monitor (10 Hz)              RtControllerNode (500 Hz)
──────────────────────────              ─────────────────────────
kEstop (비상 정지)           →  getFailure()  →  TriggerGlobalEstop()
kSafetyViolation (안전 위반) →  getFailure()  →  TriggerGlobalEstop()
kTrackingError (추적 오차)   →  getFailure()  →  TriggerGlobalEstop()
kJointLimitViolation         →  getFailure()  →  TriggerGlobalEstop()
kWatchdogTimeout             →  getFailure()  →  TriggerGlobalEstop()
kProtectiveStop              →  getFailure()  →  (자동 복구 가능)
kProgramDisconnected         →  getFailure()  →  (자동 복구 가능)
```

> `registerOnFailure()` 콜백으로 `TriggerGlobalEstop(failure_type)`을 등록하면 장애 감지 즉시 E-STOP이 작동합니다.

---

## 의존성 그래프 내 위치

```
rtc_base + rclcpp + diagnostic_msgs
    ↓
rtc_status_monitor  ← 10 Hz 비-RT 상태 모니터
    ↑
    └── rtc_controller_manager  (선택적 compose, RT 스레드에 atomic 상태 제공)
```

---

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | C++20, `[[nodiscard]]`, `noexcept` 확인 완료 — 이미 적용됨 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
