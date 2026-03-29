# rtc_status_monitor

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **비-RT 상태 모니터링 컴포넌트** 패키지입니다. 10 Hz 모니터링 루프(`std::jthread`)에서 로봇 모드, 안전 모드, 추적 오차, 관절 한계, 컨트롤러 활성 상태를 감시하고, 500 Hz RT 제어 스레드에 원자적(lock-free) 상태를 제공합니다.

**핵심 기능:**
- 14개 장애 유형(`FailureType`) + 4개 경고 유형(`WarningType`) 정의 (핸드 관련 4개 포함)
- RT 스레드에 원자적 상태 접근 (`std::atomic` acquire/release, 절대 차단 없음)
- 장애 발생 시 10초 상태 히스토리 기록 (순환 버퍼, 100 엔트리 @ 10 Hz)
- 컨트롤러별 통계 수집 (패킷 수, 활성 시간, 장애 수) 및 JSON 저장
- 선택적 자동 복구 (protective stop, 프로그램 연결 끊김)
- 장애 시점 관절 상태/추적 오차/모드를 포함한 상세 로그 파일 생성

---

## 패키지 구조

```
rtc_status_monitor/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/rtc_status_monitor/
│   ├── rtc_status_monitor.hpp     ← 메인 모니터 클래스
│   ├── failure_types.hpp          ← 장애/경고 열거형, UR 모드 열거형, FailureContext
│   └── state_history.hpp          ← 순환 버퍼 (StateHistory, 10초 히스토리)
├── src/
│   └── rtc_status_monitor.cpp     ← 구현
└── config/
    └── rtc_status_monitor.yaml    ← 모니터링 설정
```

---

## 아키텍처

```
                          10 Hz 모니터 루프 (std::jthread)
                    ┌──────────────────────────┐
                    │ CheckJointStateWatchdog() │
                    │ CheckRobotStatus()        │
                    │ CheckTrackingErrors()     │
                    │ CheckJointLimits()        │
                    │ CheckControllerManager()  │  ← 매 5초 (controller_poll_interval_sec)
                    │ CheckReadiness()          │
                    │ StateHistory Push         │
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

`FailureType`은 `uint8_t` 기반 enum class이며, `std::atomic`에 저장됩니다.

| 값 | 장애 유형 | 트리거 | 자동 복구 |
|----|----------|--------|----------|
| 0 | `kNone` | 초기 상태 / 복구 성공 | -- |
| 1 | `kEstop` | SafeguardStop, SystemEmergencyStop, RobotEmergencyStop | 불가 |
| 2 | `kProtectiveStop` | safety_mode = ProtectiveStop | **가능** |
| 3 | `kSafetyViolation` | safety_mode = Violation 또는 Fault | 불가 |
| 4 | `kProgramDisconnected` | 준비 상태에서 프로그램 실행 중지 | **가능** |
| 5 | `kWatchdogTimeout` | joint_states 수신 > watchdog_timeout_sec | 불가 |
| 6 | `kControllerInactive` | 대상 컨트롤러 비활성 (준비 상태일 때만) | 불가 |
| 7 | `kHardwareFault` | 하드웨어 상태 이상 | 불가 |
| 8 | `kTrackingError` | 위치/속도 추적 오차 > fault 임계값 | 불가 |
| 9 | `kJointLimitViolation` | 관절 위치가 fault 마진 내 진입 | 불가 |
| 10 | `kHandTimeout` | 핸드 joint_states 수신 타임아웃 | 불가 |
| 11 | `kHandMotorDataFault` | 핸드 모터 전-영/중복 데이터 감지 | 불가 |
| 12 | `kHandSensorDataFault` | 핸드 센서 전-영/중복 데이터 감지 | 불가 |
| 13 | `kHandRateLow` | 핸드 메시지 레이트 < 최소 허용 Hz | 불가 |

> 핸드 관련 장애 유형(10~13)은 `failure_types.hpp`에 정의되어 있으며, `ur5e_hand_status_monitor` 패키지에서 확장 사용됩니다.

## 경고 유형 (WarningType)

| 값 | 경고 유형 | 트리거 |
|----|----------|--------|
| 0 | `kNone` | 경고 없음 |
| 1 | `kJointLimitProximity` | 관절 위치가 warn 마진 내 진입 |
| 2 | `kTrackingErrorHigh` | 추적 오차 > warn 임계값 (fault 미만) |
| 3 | `kHighLatency` | 예약됨 (미사용) |
| 4 | `kHandRateDegraded` | 핸드 메시지 레이트 저하 (fault 미만) |

---

## 모니터링 루프 (10 Hz)

매 100 ms마다 순차적으로 실행됩니다:

1. **CheckJointStateWatchdog()** -- `/joint_states` 수신 타임아웃 검사 (첫 수신 전에는 건너뜀)
2. **CheckRobotStatus()** -- 안전 모드 검사 (E-STOP, protective stop, 안전 위반) + 프로그램 실행 상태
3. **CheckTrackingErrors()** -- 레퍼런스 대비 실제 위치/속도 오차 (`try_lock`으로 비차단 접근)
4. **CheckJointLimits()** -- 관절 위치가 한계 마진에 근접 여부 확인
5. **레이트 계산** -- `/joint_states` 수신 주파수 측정 (1초 윈도우)
6. **CheckControllerManager()** -- 대상 컨트롤러 활성 상태 확인 (매 `controller_poll_interval_sec`초)
7. **CheckReadiness()** -- 모든 조건 통합 (robot_mode=Running, safety_mode=Normal, program_running, joint_states 수신) -> `is_ready_` 원자적 갱신
8. **StateHistory Push** -- 순환 버퍼에 현재 상태 기록 (100 엔트리 = 10초)
9. **PublishDiagnostics()** -- `/diagnostics` 토픽 퍼블리시 (매 1초)

---

## RT 스레드 인터페이스

RT 제어 스레드에서 **절대 차단 없이** 호출 가능한 메서드입니다.

| 메서드 | 메커니즘 | 설명 |
|--------|---------|------|
| `isReady()` | `atomic<bool>` acquire | 준비 상태 조회 |
| `getFailure()` | `atomic<FailureType>` acquire | 현재 장애 유형 |
| `isJointLimitWarning()` | `atomic<bool>` acquire | 관절 한계 경고 |
| `getJointStateStats()` | `MessageStats` 복사 | 메시지 통계 (수신 수, 타임아웃, 주파수) |
| `setJointReference()` | `try_lock` (비차단) | 레퍼런스 위치/속도 전달 |

---

## 자동 복구

`kProtectiveStop`과 `kProgramDisconnected`에 대해서만 자동 복구를 시도합니다.

1. `auto_recovery` 활성화 확인
2. 쿨다운 확인 (`recovery_interval_sec`)
3. 최대 시도 횟수 확인 (`max_recovery_attempts`)
4. UR Dashboard 서비스 호출 (빌드 시 `ur_dashboard_msgs`가 있을 때만 완전 지원)
5. `ClearFailure()` -> 2초 대기 -> safety_mode 확인
6. 성공: `recovery_attempts_` 초기화 + `on_recovery_cb_` 호출

> `auto_recovery`는 기본 비활성(`false`)입니다. `ur_dashboard_msgs` 없이 빌드하면 대시보드 호출 없이 failure clear와 재확인만 수행됩니다.

---

## 상태 히스토리 (StateHistory)

`state_history.hpp`에 정의된 고정 크기 순환 버퍼입니다.

- **용량:** 기본 100 엔트리 (`kDefaultCapacity = 100`, 10 Hz에서 10초)
- **항목 (StateHistoryEntry):** timestamp, robot_mode, safety_mode, program_running, q[6], qd[6], tracking_error[6], joint_limit_warning
- **용도:** 장애 발생 시 `LogFailureToFile()`에서 CSV 형태로 기록
- **스레드 안전:** `history_mutex_`로 보호 (모니터 스레드에서만 쓰기)

---

## ROS2 인터페이스

### 구독

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/joint_states` | `sensor_msgs/JointState` | SensorDataQoS | 관절 위치/속도 |
| `/io_and_status_controller/robot_mode` | `std_msgs/Int32` | depth=10 | 로봇 모드 |
| `/io_and_status_controller/safety_mode` | `std_msgs/Int32` | depth=10 | 안전 모드 |
| `/io_and_status_controller/robot_program_running` | `std_msgs/Bool` | depth=10 | 프로그램 실행 |
| `/ur5e/active_controller_name` | `std_msgs/String` | reliable, transient_local, depth=1 | 컨트롤러 통계용 (enable_controller_stats=true일 때만) |

### 퍼블리셔

| 토픽 | 타입 | 주파수 | 설명 |
|------|------|--------|------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | 시스템 상태 진단 |

### 서비스 클라이언트

| 서비스 | 타입 | 주기 | 설명 |
|--------|------|------|------|
| `/controller_manager/list_controllers` | `ListControllers` | `controller_poll_interval_sec` (기본 5초) | 대상 컨트롤러 + joint_state_broadcaster 활성 확인 |

---

## 설정

### config/rtc_status_monitor.yaml

모든 파라미터는 `status_monitor.` 접두사 아래에 선언됩니다.

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

      # 관절 한계 마진 (deg → 코드 내부에서 rad 변환)
      joint_limit_warn_margin_deg:  5.0
      joint_limit_fault_margin_deg: 1.0

      # 관절 한계 (rad) — 관절별 오버라이드
      joint_limits:
        joint_0: { lower: -6.2832, upper: 6.2832 }   # ±360 deg
        joint_1: { lower: -6.2832, upper: 6.2832 }
        joint_2: { lower: -3.1416, upper: 3.1416 }   # ±180 deg (elbow)
        joint_3: { lower: -6.2832, upper: 6.2832 }
        joint_4: { lower: -6.2832, upper: 6.2832 }
        joint_5: { lower: -6.2832, upper: 6.2832 }

      # 자동 복구
      auto_recovery: false
      max_recovery_attempts: 3
      recovery_interval_sec: 5.0

      # 컨트롤러별 통계
      enable_controller_stats: true

      # 로깅
      log_output_dir: ""    # 빈 문자열: RTC_SESSION_DIR/monitor/ (fallback: UR5E_SESSION_DIR → ~/.ros → /tmp)
```

### 파라미터 상세

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `joint_states_topic` | string | `"/joint_states"` | 관절 상태 토픽 |
| `robot_mode_topic` | string | `"/io_and_status_controller/robot_mode"` | 로봇 모드 토픽 |
| `safety_mode_topic` | string | `"/io_and_status_controller/safety_mode"` | 안전 모드 토픽 |
| `program_running_topic` | string | `"/io_and_status_controller/robot_program_running"` | 프로그램 실행 토픽 |
| `watchdog_timeout_sec` | double | `1.0` | joint_states 수신 타임아웃 (초) |
| `controller_poll_interval_sec` | double | `5.0` | list_controllers 호출 주기 (초) |
| `target_controller` | string | `"scaled_joint_trajectory_controller"` | 감시 대상 컨트롤러 이름 |
| `tracking_error_pos_warn_rad` | double | `0.05` | 위치 추적 오차 경고 임계값 (rad) |
| `tracking_error_pos_fault_rad` | double | `0.15` | 위치 추적 오차 장애 임계값 (rad) |
| `tracking_error_vel_warn_rad` | double | `0.1` | 속도 추적 오차 경고 임계값 (rad/s) |
| `tracking_error_vel_fault_rad` | double | `0.3` | 속도 추적 오차 장애 임계값 (rad/s) |
| `joint_limit_warn_margin_deg` | double | `5.0` | 관절 한계 경고 마진 (deg, 내부 rad 변환) |
| `joint_limit_fault_margin_deg` | double | `1.0` | 관절 한계 장애 마진 (deg, 내부 rad 변환) |
| `joint_limits.joint_N.lower` | double | UR5e 스펙 | 관절 N 하한 (rad) |
| `joint_limits.joint_N.upper` | double | UR5e 스펙 | 관절 N 상한 (rad) |
| `auto_recovery` | bool | `false` | 자동 복구 활성화 여부 |
| `max_recovery_attempts` | int | `3` | 최대 복구 시도 횟수 |
| `recovery_interval_sec` | double | `5.0` | 복구 시도 간 쿨다운 (초) |
| `enable_controller_stats` | bool | `true` | 컨트롤러별 통계 수집 활성화 |
| `log_output_dir` | string | `""` | 로그 출력 디렉토리 (빈 문자열 시 자동 결정) |

---

## 장애 로깅

장애 발생 시 상세 로그 파일이 생성됩니다:

- **파일명:** `ur5e_failure_YYYYMMDDTHHMMSS_mmm.log`
- **내용:**
  - `[HEADER]` -- 장애 유형, 타임스탬프, 로봇/안전 모드
  - `[STATE AT FAILURE]` -- 장애 시점 관절 상태 (q_actual, qd_actual, q_reference, tracking_error)
  - `[STATE HISTORY]` -- 10초 히스토리 (CSV, 10 Hz 샘플)
  - `[CONTROLLER STATE AT FAILURE]` -- 대상 컨트롤러 활성/비활성
- **경로 우선순위:** `log_output_dir` 파라미터 -> `RTC_SESSION_DIR/monitor/` -> `UR5E_SESSION_DIR/monitor/` -> `~/.ros` -> `/tmp`

### 컨트롤러 통계 JSON

`stop()` 호출 시 `controller_stats_YYYYMMDDTHHMMSS_mmm.json` 파일이 생성됩니다:
- 전체 가동 시간, joint_state 총 패킷/타임아웃/레이트
- 컨트롤러별: 활성 시간, 패킷 수, 타임아웃 수, 장애 수

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rclcpp` | ROS2 클라이언트 |
| `std_msgs` | Int32, Bool, String |
| `sensor_msgs` | JointState |
| `diagnostic_msgs` | DiagnosticArray, DiagnosticStatus, KeyValue |
| `controller_manager_msgs` | ListControllers 서비스 |
| `rtc_base` | 타입(`kNumRobotJoints`), 스레딩 유틸리티(`ThreadConfig`, `ApplyThreadConfigWithFallback`) |
| `ur_dashboard_msgs` | (선택, `find_package QUIET`) 자동 복구 시 UR 대시보드 서비스 호출. 빌드 시 있으면 `HAVE_UR_DASHBOARD_MSGS` 매크로 정의 |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_status_monitor
source install/setup.bash
```

공유 라이브러리(`librtc_status_monitor.so`)가 생성됩니다. C++20 필수.

---

## E-STOP 시스템 연동

모니터가 장애를 감지하면 RT 컨트롤러(`RtControllerNode`)가 원자적 상태를 읽어 E-STOP을 트리거합니다:

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
rtc_status_monitor  ← 10 Hz 비-RT 상태 모니터 (librtc_status_monitor.so)
    ↑
    └── rtc_controller_manager  (선택적 compose, RT 스레드에 atomic 상태 제공)
```

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
