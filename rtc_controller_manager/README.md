# rtc_controller_manager

![version](https://img.shields.io/badge/version-v0.1.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **500 Hz 실시간 컨트롤러 노드** 패키지입니다. 컨트롤러 수명 관리, 결정론적 제어 루프, 퍼블리시 오프로드, CSV 로깅, E-STOP 안전 메커니즘을 통합 관리합니다. Phase 3 구조 개편을 통해 로봇에 독립적(robot-agnostic)으로 설계되었습니다.

**핵심 기능:**
- `clock_nanosleep` 기반 500 Hz 결정론적 제어 루프
- 락-프리 SPSC 버퍼를 통한 퍼블리시/로깅 오프로드
- 런타임 컨트롤러 전환 (atomic index swap)
- 멀티 코어 스레드 분리 (5+ 스레드, CPU 어피니티)
- 글로벌 E-STOP + 연속 오버런 감지

---

## 패키지 구조

```
rtc_controller_manager/
├── CMakeLists.txt
├── package.xml
├── include/rtc_controller_manager/
│   ├── rt_controller_node.hpp             ← 메인 노드 클래스
│   ├── rt_controller_main.hpp             ← 재사용 가능 진입점 함수
│   └── controller_timing_profiler.hpp     ← 락-프리 타이밍 프로파일러
├── src/
│   ├── rt_controller_node.cpp             ← 노드 구현 (1827 lines)
│   ├── rt_controller_main.cpp             ← main() 진입점
│   └── rt_controller_main_impl.cpp        ← RtControllerMain() 구현
└── config/
    ├── rt_controller_manager.yaml         ← 제어 루프 설정
    └── cyclone_dds.xml                    ← CycloneDDS RT 설정
```

---

## 스레딩 모델

5개 이상의 스레드가 CPU 코어에 분리 배치됩니다.

| 스레드 | 코어 | 스케줄러 | 주파수 | 역할 |
|--------|------|----------|--------|------|
| **rt_loop** | 2 | SCHED_FIFO 90 | 500 Hz + 50 Hz | `clock_nanosleep` 제어 루프 + 워치독 |
| **publish_thread** | 5 | SCHED_OTHER -3 | 이벤트 | SPSC 드레인 → ROS2 `publish()` |
| **sensor_executor** | 3 | SCHED_FIFO 70 | 이벤트 | `/joint_states`, 타겟, 핸드 구독 |
| **log_executor** | 4 | SCHED_OTHER -5 | 100 Hz | `SpscLogBuffer` → CSV 드레인 |
| **aux_executor** | 5 | SCHED_OTHER | 이벤트 | E-STOP 상태 퍼블리시 |

> `mlockall(MCL_CURRENT | MCL_FUTURE)`를 `rclcpp::init()` 전에 호출하여 페이지 폴트를 방지합니다.

---

## 제어 루프 흐름 (500 Hz, 2 ms/tick)

### Phase 0: 준비 검사
- `state_received_` 플래그 확인 → 타임아웃 시 E-STOP
- Auto-hold 모드: 외부 타겟 없으면 현재 위치를 타겟으로 초기화

### Phase 1: 비차단 상태 획득 (~50-100 µs)
- `try_lock`으로 관절 위치/속도/토크 캐시 갱신 (실패 시 이전 값 재사용, ≤2 ms stale)
- 핸드 상태 읽기 (실제 UDP 또는 시뮬레이션 ROS 토픽)
- `try_lock`으로 타겟 스냅샷

### Phase 2: 제어 연산 (~0.5-5 µs)
- `timing_profiler_.MeasuredCompute(controller, state)`
- 활성 컨트롤러의 `Compute()` 호출 → `ControllerOutput` 반환

### Phase 3: 퍼블리시 오프로드 (~0.2 µs, 락-프리)
- `PublishSnapshot` 생성 → `publish_buffer_.Push()` (SPSC O(1))
- 핸드 커맨드 전송 (condvar notify, ~100 ns)

### Phase 4: 타이밍 & 로깅 (~1-2 µs)
- 위상별 소요 시간 계산 + 지터 측정
- `LogEntry` → `log_buffer_.Push()` (SPSC)
- 1000 이터레이션마다 타이밍 서머리 출력

### 워치독 (50 Hz, 매 10번째 tick)
- `device_timeouts`에 등록된 각 디바이스 그룹의 state 토픽 수신 간격이 timeout 초과 시 E-STOP

---

## 컨트롤러 관리

### 로딩 (시작 시)

1. `ControllerRegistry::Instance().GetEntries()` 조회
2. 각 컨트롤러: 팩토리로 인스턴스 생성 → `LoadConfig(YAML)` → `controllers_` 벡터에 추가
3. `initial_controller` 파라미터로 초기 활성 컨트롤러 선택

### 런타임 전환 (`/ur5e/controller_type` 구독)

1. 인덱스 유효성 검증
2. Auto-hold: 현재 위치로 `InitializeHoldPosition()` 호출
3. `active_controller_idx_.store(idx, memory_order_release)` — atomic 전환
4. `/ur5e/active_controller_name` 퍼블리시 (TRANSIENT_LOCAL)

### 게인 업데이트 (`/ur5e/controller_gains` 구독)

활성 컨트롤러의 `UpdateGainsFromMsg()` 호출 → 런타임 게인 변경

---

## E-STOP 및 안전 메커니즘

### E-STOP 트리거

| 트리거 | 조건 | 결과 |
|--------|------|------|
| `init_timeout` | `init_timeout_sec` 내 state 미수신 | `TriggerGlobalEstop("init_timeout")` + 노드 종료 |
| `robot_timeout` | `/joint_states` >100ms 미갱신 (CheckTimeouts 50Hz) | `TriggerGlobalEstop("robot_timeout")` |
| `hand_timeout` | `/hand/joint_states` >200ms 미갱신 | `TriggerGlobalEstop("hand_timeout")` |
| `consecutive_overrun` | ≥10회 연속 RT 루프 오버런 | `TriggerGlobalEstop("consecutive_overrun")` |
| 상태 모니터 실패 | Safety/tracking 위반, 관절 한계 (10Hz) | `TriggerGlobalEstop(failure_type)` |
| `hand_failure` | UDP 영/중복 데이터 감지 (50Hz) | `TriggerGlobalEstop("hand_failure")` |

### 오버런 복구

- 누락된 tick 건너뛰기 (burst 없음, 주기 재정렬)
- `overrun_count_`, `skip_count_` 원자적 카운트
- RT 루프는 E-STOP 후에도 계속 실행 (타이밍/로깅 유지)

### 비차단 뮤텍스 패턴

```cpp
std::unique_lock lock(state_mutex_, std::try_to_lock);
if (lock.owns_lock()) {
    cached_positions_ = current_positions_;  // 갱신
} else {
    // 이전 사이클 캐시 재사용 (≤2 ms stale)
}
```

---

## 타이밍 프로파일러 (`controller_timing_profiler.hpp`)

컨트롤러 `Compute()` 호출의 락-프리 타이밍 통계를 수집합니다.

| 항목 | 설명 |
|------|------|
| 히스토그램 | 20개 버킷 (100 µs 간격, 0-2000 µs) + 오버플로 버킷 |
| 통계 | min, max, mean, stddev, p95, p99 |
| 예산 초과 | 2000 µs (500 Hz = 2 ms) 초과 횟수 |

---

## ROS2 인터페이스

### 고정 구독

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ur5e/controller_type` | `Int32` | 컨트롤러 인덱스 전환 |
| `/ur5e/controller_gains` | `Float64MultiArray` | 활성 컨트롤러 게인 업데이트 |
| `/ur5e/request_gains` | `Bool` | 현재 게인 요청 |

### 고정 퍼블리셔

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/system/estop_status` | `Bool` | 글로벌 E-STOP 상태 |
| `/ur5e/active_controller_name` | `String` | 활성 컨트롤러 이름 (TRANSIENT_LOCAL) |
| `/ur5e/current_gains` | `Float64MultiArray` | 현재 게인 응답 |
| `/ur5e/joint_command` | `JointCommand` | MuJoCo 시뮬레이터용 관절 커맨드 |

### 동적 구독/퍼블리셔

컨트롤러별 `TopicConfig` YAML에 따라 동적으로 생성됩니다:
- 관절 상태, 타겟, 핸드 상태 구독
- 위치/토크 커맨드, 태스크 위치, 궤적/컨트롤러 상태 퍼블리시

### 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `control_rate` | `500.0` | 제어 주파수 (Hz) |
| `initial_controller` | `"joint_pd_controller"` | 시작 컨트롤러 |
| `auto_hold_position` | `true` | 타겟 없을 때 현재 위치 유지 |
| `device_timeout_names` | `["ur5e"]` | E-STOP 감시 대상 디바이스 그룹 (topics 키와 매칭) |
| `device_timeout_values` | `[100.0]` | 각 그룹의 state 토픽 타임아웃 (ms) |
| `enable_estop` | `true` | E-STOP 활성화 |
| `enable_logging` | `true` | CSV 로깅 활성화 |
| `init_timeout_sec` | `30.0` | 초기화 타임아웃 (초) |
| `hand_sim_enabled` | `false` | 핸드 시뮬레이션 (ROS 토픽) |

---

## 설정 파일

### rt_controller_manager.yaml

```yaml
/**:
  ros__parameters:
    control_rate: 500.0
    initial_controller: "joint_pd_controller"
    init_timeout_sec: 30.0
    auto_hold_position: true
    enable_estop: true
    device_timeout_names: ["ur5e"]
    device_timeout_values: [100.0]       # ms
    enable_logging: true
    enable_timing_log: true
    enable_device_log: true
    max_log_sessions: 10

    # 디바이스 설정 (토픽 그룹명으로 키)
    devices:
      ur5e:
        joint_state_names:
          - "shoulder_pan_joint"
          - "shoulder_lift_joint"
          - "elbow_joint"
          - "wrist_1_joint"
          - "wrist_2_joint"
          - "wrist_3_joint"
        urdf:
          package: "ur5e_description"
          path: "robots/ur5e/urdf/ur5e.urdf"
          root_link: "base_link"
          tip_link: "wrist_3_link"
        joint_limits:
          max_velocity:     [2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
          max_acceleration: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
          max_torque:       [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]
          position_lower:   [-6.28, -6.28, -3.14, -6.28, -6.28, -6.28]
          position_upper:   [6.28, 6.28, 3.14, 6.28, 6.28, 6.28]
```

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rclcpp` | ROS2 클라이언트 라이브러리 |
| `sensor_msgs` | JointState 메시지 |
| `std_msgs` | Bool, Int32, String, Float64MultiArray |
| `rtc_controller_interface` | 컨트롤러 추상 인터페이스 + 레지스트리 |
| `rtc_controllers` | 내장 컨트롤러 (P, JointPD, CLIK, OSC) |
| `rtc_base` | 로깅, 스레딩, 타입, SPSC 버퍼 |
| `rtc_communication` | 통신 유틸리티 |
| `rtc_status_monitor` | 상태 모니터링 (선택) |
| `rtc_msgs` | JointCommand 커스텀 메시지 |
| `rtc_inference` | ONNX F/T 추론 |
| `ur5e_hand_driver` | 핸드 하드웨어 인터페이스 |
| `pinocchio` | URDF 검증 |
| `yaml-cpp` | YAML 파싱 |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_controller_manager
source install/setup.bash
```

**빌드 산출물:**
- 정적 라이브러리: `librtc_controller_manager_lib.a`
- 실행 파일: `rt_controller`

---

## 의존성 그래프 내 위치

```
rtc_base + rtc_controller_interface + rtc_controllers + rtc_communication
    ↓
rtc_controller_manager  ← 500 Hz RT 제어 실행 엔진
    ↑
    └── ur5e_bringup  (로봇별 진입점 + launch 파일)
```

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
