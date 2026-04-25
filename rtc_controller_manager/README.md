# rtc_controller_manager

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **500 Hz 실시간 제어 루프 매니저** 패키지입니다. 컨트롤러 수명 관리, 결정론적 제어 루프, 퍼블리시 오프로드, CSV 로깅, E-STOP 안전 메커니즘을 통합 관리합니다. Phase 3 구조 개편을 통해 로봇에 독립적(robot-agnostic)으로 설계되었습니다.

**핵심 기능:**
- `clock_nanosleep` 기반 500 Hz 결정론적 제어 루프 (실제 로봇) / CV 기반 동기 루프 (시뮬레이션)
- 락-프리 SPSC 버퍼를 통한 퍼블리시/로깅 오프로드
- 런타임 컨트롤러 전환 (atomic index swap, 컨트롤러 이름 문자열 기반)
- 멀티 코어 스레드 분리 (5개 스레드, CPU 어피니티)
- 글로벌 E-STOP + 연속 오버런 감지
- 컨트롤러별 TopicConfig YAML 기반 동적 토픽 라우팅
- `rclcpp_lifecycle::LifecycleNode` 기반 관리된 상태 전환 (Unconfigured → Inactive → Active → Finalized)

---

## 패키지 구조

```
rtc_controller_manager/
├── CMakeLists.txt
├── package.xml
├── include/rtc_controller_manager/
│   ├── rt_controller_node.hpp             <- 메인 노드 클래스
│   ├── rt_controller_main.hpp             <- 재사용 가능 진입점 함수
│   └── controller_timing_profiler.hpp     <- 락-프리 타이밍 프로파일러
├── src/
│   ├── rt_controller_node.cpp             <- 생성자/소멸자, 콜백 그룹, 타이머, Lifecycle 콜백
│   ├── rt_controller_node_params.cpp      <- 파라미터 선언/로딩, 디바이스 설정
│   ├── rt_controller_node_device_config.cpp <- URDF/모델 파싱, 디바이스 이름 설정
│   ├── rt_controller_node_topics.cpp      <- 구독/퍼블리셔 생성, 컨트롤러 전환
│   ├── rt_controller_node_callbacks.cpp   <- 디바이스 센서/타겟 콜백 (SeqLock writer)
│   ├── rt_controller_node_rt_loop.cpp     <- 500 Hz RT 루프, 워치독, 로그 드레인
│   ├── rt_controller_node_publish.cpp     <- SPSC 드레인 → ROS2 publish (eventfd wakeup)
│   ├── rt_controller_node_estop.cpp       <- E-STOP 트리거/클리어/퍼블리시
│   └── rt_controller_main_impl.cpp        <- RtControllerMain() 구현 (라이브러리)
└── config/
    ├── rt_controller_manager.yaml         <- 제어 루프 설정
    └── cyclone_dds.xml                    <- CycloneDDS RT 성능 최적화 설정
```

---

## 스레딩 아키텍처

5개의 스레드가 CPU 코어에 분리 배치됩니다. `SelectThreadConfigs()`가 물리 코어 수에 따라 설정을 자동 선택합니다.

| 스레드 | 코어 | 스케줄러 | 주파수 | 역할 |
|--------|------|----------|--------|------|
| **rt_loop** | 2 | SCHED_FIFO 90 | 500 Hz + 50 Hz | `clock_nanosleep` 제어 루프 + 워치독 |
| **sensor_executor** | 3 | SCHED_FIFO 70 | 이벤트 | 디바이스별 JointState, MotorState, SensorState, Target 구독 |
| **log_executor** | 4 | SCHED_OTHER -5 | 100 Hz | `SpscLogBuffer` -> CSV 드레인 + 타이밍 서머리 출력 |
| **publish_thread** | 5 | SCHED_OTHER -3 | 이벤트 | SPSC 드레인 → ROS2 `publish()` (eventfd wakeup, 모든 DDS 직렬화/시스콜 처리) |
| **aux_executor** | 5 | SCHED_OTHER 0 | 이벤트 | 컨트롤러 전환, 게인 업데이트, E-STOP 상태 퍼블리시 |

> `mlockall(MCL_CURRENT | MCL_FUTURE)`를 `rclcpp::init()` 전에 호출하여 페이지 폴트를 방지합니다.

### Lifecycle 상태 전환

`RtControllerNode`는 `rclcpp_lifecycle::LifecycleNode`를 상속하며, 관리된 상태 전환을 지원합니다.

| 콜백 | 리소스 티어 | 역할 |
|------|-----------|------|
| `on_configure` | Tier 1 | 콜백 그룹, 파라미터, 컨트롤러, 퍼블리셔/구독, 타이머, eventfd |
| `on_activate` | Tier 2 | RT 루프 + 퍼블리시 오프로드 스레드 시작 |
| `on_deactivate` | — | RT 루프/퍼블리시 스레드 중지, E-STOP 클리어, 상태 초기화 |
| `on_cleanup` | — | `on_configure` 역순 리소스 해제 |
| `on_error` | — | E-STOP 트리거, 스레드 중지, 전체 정리 → SUCCESS (Unconfigured 복구) |

**안전 퍼블리셔:** `estop_pub_`, `active_ctrl_name_pub_`, `current_gains_pub_`는 `rclcpp::create_publisher` standalone으로 생성되어 lifecycle 상태와 무관하게 동작합니다.

런타임 상태 제어:
```bash
ros2 lifecycle get /rtc_controller_manager
ros2 lifecycle set /rtc_controller_manager deactivate   # RT 루프 중지
ros2 lifecycle set /rtc_controller_manager activate     # RT 루프 재시작
```

**초기화 순서 (`RtControllerMain()` — 3-Phase Lifecycle Executor):**
1. `mlockall()` → `rclcpp::init()` → 노드 생성 (Unconfigured 상태)
2. **Phase 1:** `lifecycle_executor` spin → Launch event handler가 configure/activate 트리거
   - `on_configure`: 콜백 그룹, 파라미터, 구독/퍼블리셔, 타이머, eventfd 생성
   - `on_activate`: `SelectThreadConfigs()` → `StartRtLoop()` + `StartPublishLoop()` 시작
3. **Phase 2:** Active 상태 대기 (polling)
4. **Phase 3:** sensor/log/aux 전용 executor 전환 (lifecycle services는 aux_executor에서 처리)

---

## 제어 루프 흐름 (500 Hz, 2 ms/tick)

### Phase 0: 준비 검사
- `state_received_` 플래그 확인 -> 타임아웃 시 E-STOP + 노드 종료
- Auto-hold 모드: 외부 타겟 없으면 모든 디바이스가 valid 상태일 때 현재 위치를 타겟으로 초기화
- `init_complete_` 이후 정상 루프 진입

### Phase 1: 비차단 상태 획득
- 디바이스별 `SeqLock<DeviceStateCache>::Load()`로 락-프리 상태 획득 (항상 최신 데이터, 우선순위 역전 없음)
- 활성 컨트롤러의 TopicConfig.groups 순서대로 `device_states_[slot].Load()` → `ControllerState.devices[]` 복사
- `try_lock`으로 타겟 스냅샷 (`device_target_snapshots_`)

### Phase 2: 제어 연산
- `timing_profiler_.MeasuredCompute(controller, state)`
- 활성 컨트롤러의 `Compute()` 호출 -> `ControllerOutput` 반환
- 벽시계 시간 히스토그램 자동 수집 (20 버킷, 100us 간격)

### Phase 3: 퍼블리시 오프로드 (락-프리)
- `PublishSnapshot` 생성 -> `publish_buffer_.Push()` (SPSC O(1))
- 그룹별 commands, actual, motor, sensor, inference 데이터 포함
- `publish_thread`가 드레인하여 모든 ROS2 publish() 처리

### Phase 4: 타이밍 & 로깅
- 위상별 소요 시간 계산 (state_acquire, compute, publish) + 지터 측정
- `LogEntry` -> `log_buffer_.Push()` (SPSC)
- 1000 이터레이션마다 타이밍 서머리 출력 신호 (실제 출력은 log_executor에서)

### 워치독 (50 Hz, 매 10번째 tick)
- `device_timeouts`에 등록된 각 디바이스 그룹의 state 토픽 수신 간격이 timeout 초과 시 E-STOP
- 같은 디바이스 그룹 내 kState, kMotorState, kSensorState 구독 모두 timeout 갱신 (일부 토픽 드롭 허용)

---

## 컨트롤러 관리

### 시스템 URDF + ModelConfig 파싱

노드 초기화 시 최상위 `urdf:` 파라미터 섹션에서 시스템 레벨 URDF 경로와 모델 토폴로지를 파싱합니다.

```
urdf.package + urdf.path → ament resolve → 절대 URDF 경로
urdf.root_joint_type     → "fixed" | "floating"
urdf.sub_models.<name>   → {root_link, tip_link}                (직렬 체인, map 키 = 모델명)
urdf.tree_models.<name>  → {root_link, tip_links[]}             (분기 체인, map 키 = 모델명)
urdf.passive_joints      → [string, ...]                         (잠금 관절)
```

- `sub_models`/`tree_models`의 map 키(모델명)는 `devices` 블록의 디바이스 그룹 이름과 매칭됩니다
- 디바이스별 `root_link`/`tip_link` 미지정 시 시스템 `sub_models`/`tree_models`에서 자동 해석
- 디바이스별 URDF 경로 미지정 시 시스템 URDF 경로를 폴백으로 사용
- **하위 호환**: 최상위 `urdf:` 없으면 기존 `devices.{group}.urdf` 에서 읽기

### 로딩 (시작 시)

1. `ControllerRegistry::Instance().GetEntries()` 조회
2. 시스템 `PinocchioModelBuilder` 한 번 빌드 (`std::shared_ptr`) — `system_model_config_`로 URDF 파싱 + full/sub/tree 모델 구축을 1회만 수행. 빌드 실패 시 컨트롤러는 자기 builder를 만드는 폴백 경로로 동작
3. 각 컨트롤러: 팩토리로 인스턴스 생성 -> `SetSystemModelConfig()` -> `SetSharedModelBuilder(shared)` -> 전용 `rclcpp_lifecycle::LifecycleNode`(이름 = `config_key`, 네임스페이스 = `/<config_key>`, `NodeOptions().use_global_arguments(false)`) 생성 -> `SetTargetReceivedNotifier(...)` 주입 -> `on_configure(unconfigured, node, yaml)` 호출 (base 기본 구현이 `LoadConfig(yaml)`를 내부에서 try/catch로 실행) -> `controllers_` + `controller_nodes_` 벡터에 추가
   * `use_global_arguments(false)` 는 필수: 부모 프로세스의 CLI remap (`--ros-args -r __node:=ur5e_rt_controller` 등)이 글로벌 컨텍스트에 살아 있어, default `NodeOptions` 으로 만들면 자식 노드 이름·네임스페이스가 부모로 덮어씌워지고 launch_ros 가 부모에 spurious lifecycle ACTIVATE 를 또 보내는 부작용까지 발생함
   * `SetSharedModelBuilder` 는 `RTControllerInterface::GetSharedModelBuilder()` 로 컨트롤러가 가져가는 공유 핸들. 데모 컨트롤러는 `InitArmModel` / `InitModels` 진입 시 이 핸들을 우선 사용하므로, 동일한 ModelConfig로 `xacro → tinyxml2 → Pinocchio` 파이프라인을 컨트롤러 수만큼 반복하는 비용을 제거함 (데모 빌업 기준 4회 → 1회)
4. 컨트롤러 이름과 config_key 양쪽을 `controller_name_to_idx_` 맵에 등록
5. `initial_controller` 파라미터로 초기 활성 컨트롤러 선택
6. `main()`에서 모든 `controller_nodes_`를 `aux_executor`에 attach — 컨트롤러가 자체 소유한 sub/pub 콜백이 non-RT 스레드에서 처리됨
7. CM `on_activate` -> 각 컨트롤러 `on_activate(active)` 호출(LifecyclePublisher 활성화 후) -> RT 루프 + publish 스레드 시작
8. CM `on_deactivate` -> RT/publish 스레드 정지 후 각 컨트롤러 `on_deactivate(inactive)` 호출
9. CM `on_cleanup` 진입 시 각 컨트롤러에 `on_cleanup(inactive)` 호출 후 `controller_nodes_`/`controllers_` 해제

> Lifecycle 훅 규약은 [`rtc_controller_interface/README.md`](../rtc_controller_interface/README.md#lifecycle-훅-ros2_control-정렬-기본-구현-제공) 참조. 컨트롤러는 per-node topic 소유권을 가지며(`/<config_key>/<topic>`), RT 경로에서 `node_` 접근은 금지입니다.

### 토픽 소유권 tier

| ownership | 생성 주체 | 역할 예시 | QoS 경로 |
|-----------|-----------|-----------|---------|
| `manager` (기본) | `RtControllerNode` | `state`, `joint_command`, `ros2_command`, `device_state_log`, `device_sensor_log`, `digital_twin_state` | CM의 subscription / SPSC drain → `LifecyclePublisher` |
| `controller` | 컨트롤러별 `LifecycleNode` | `target`, `gui_position`, `grasp_state`, `tof_snapshot` 등 외부 GUI/BT용 non-RT 트래픽 | 컨트롤러가 on_configure에서 sub/pub 생성, publish 스레드가 SPSC 소비 후 `controllers_[active]->PublishNonRtSnapshot(snap)` 호출 |

CM은 YAML `ownership: controller`로 표시된 entry에 대해 sub/pub 생성과 publish 디스패치를 건너뜁니다. Controller-owned target 구독 콜백은 `NotifyTargetReceived()`를 호출해 CM의 `target_received_` 게이트를 한 번 전환합니다.

### 런타임 전환 (`/{robot_ns}/controller_type` 구독, String 타입)

1. `controller_name_to_idx_` 맵에서 이름/config_key로 인덱스 조회
2. Auto-hold: 현재 위치로 `InitializeHoldPosition()` 호출
3. `active_controller_idx_.store(idx, memory_order_release)` -- atomic 전환
4. `/{robot_ns}/active_controller_name` 퍼블리시 (TRANSIENT_LOCAL)

### 게인 업데이트 (`/{robot_ns}/controller_gains` 구독)

활성 컨트롤러의 `UpdateGainsFromMsg()` 호출 -> 런타임 게인 변경

### 게인 조회 (`/{robot_ns}/request_gains` 구독)

`/{robot_ns}/current_gains`에 현재 게인 배열 퍼블리시

---

## E-STOP 및 안전 메커니즘

### E-STOP 트리거

| 트리거 | 조건 | 결과 |
|--------|------|------|
| `init_timeout` | `init_timeout_sec` 내 state 미수신 | `TriggerGlobalEstop("init_timeout")` + 노드 종료 |
| `{group}_timeout` | 디바이스 그룹의 state 토픽이 설정된 ms 초과 미갱신 (50Hz 워치독) | `TriggerGlobalEstop("{group}_timeout")` |
| `consecutive_overrun` | >= 10회 연속 RT 루프 오버런 | `TriggerGlobalEstop("consecutive_overrun")` |
| `sim_sync_timeout` | 시뮬레이션 모드에서 CV 타임아웃 (state 미수신) | `TriggerGlobalEstop("sim_sync_timeout")` + 노드 종료 |

### 글로벌 E-STOP 동작

- `TriggerGlobalEstop()`: 멱등(idempotent), `compare_exchange_strong`으로 1회만 실행
- 모든 컨트롤러에 `TriggerEstop()` + `SetHandEstop(true)` 전파
- `/system/estop_status`에 `true` 퍼블리시
- RT 루프는 E-STOP 후에도 계속 실행 (타이밍/로깅 유지)
- **RT 안전:** `estop_reason_`은 `std::array<char, 128>` 고정 크기 버퍼 (힙 할당 없음), RCLCPP 로깅은 `estop_log_pending_` atomic 플래그를 통해 non-RT `DrainLog()` 스레드에서 지연 출력

### 오버런 복구

- 누락된 tick 건너뛰기 (burst 없음, 주기 재정렬)
- `overrun_count_`, `skip_count_`, `consecutive_overruns_` 원자적 카운트
- `compute_overrun_count_`: ControlLoop() 자체가 tick 예산 초과한 횟수

### SeqLock 기반 디바이스 상태 공유

디바이스별 `SeqLock<DeviceStateCache>`로 센서 콜백(writer)과 RT 루프(reader) 간 락-프리 데이터 공유:

```cpp
// Writer (sensor callbacks, cb_group_sensor_ MutuallyExclusive — 단일 writer 보장)
auto ds = device_states_[slot].Load();
ds.positions = ...;  // 필드 수정
device_states_[slot].Store(ds);

// Reader (RT loop — 락-프리, 항상 최신 consistent snapshot)
const auto cache = device_states_[slot].Load();
```

> `DeviceStateCache`는 trivially copyable (~4.3 KB). SeqLock writer는 wait-free (2회 atomic store + memcpy), reader는 writer 완료 시까지 spin-retry (writer ~1-2 µs).

### eventfd 기반 Publish Thread Wakeup

RT thread가 `publish_buffer_.Push()` 후 `eventfd_write()`로 publish thread를 즉시 깨움:

```cpp
// RT thread (after Push)
eventfd_write(publish_eventfd_, 1);

// Publish thread (대기)
poll(&pfd, 1, 1);  // 1ms timeout, eventfd readable → 즉시 wakeup
```

> `sched_yield()` 대비 CPU 사용량 감소 + 즉시 wakeup으로 publish 지연 최소화.

---

## 타이밍 프로파일러 (`controller_timing_profiler.hpp`)

컨트롤러 `Compute()` 호출의 락-프리 타이밍 통계를 수집합니다. `TimingProfilerBase<200, 10, 2000>`을 상속합니다.

| 항목 | 설명 |
|------|------|
| 히스토그램 | 200개 버킷 (10 us 간격, 0-2000 us) + 오버플로 버킷 |
| 통계 | min, max, mean, stddev, p95, p99 |
| 예산 초과 | 2000 us (500 Hz = 2 ms) 초과 횟수 |
| 리셋 주기 | RT 루프가 1 000 tick(≈ 2 s)마다 로그 스레드에 Summary 출력 요청 → 로그 스레드는 `RCLCPP_INFO` 직후 `timing_profiler_.Reset()`을 호출. 따라서 각 출력은 **최근 ~2 s 윈도우**의 통계이며, 세션 시작 시점의 스파이크가 p99에 영구 반영되지 않음 |
| p95/p99 정확도 | (1) bucket 폭(10 µs)이 일반적인 Compute 시간(10–100 µs)보다 작아 단일 bucket 클러스터로 인한 외삽 오류 없음. (2) 오버플로 버킷(≥ 2 ms)에서 p99가 떨어질 경우 `[2000 µs, max_us]` 구간의 선형 보간값을 반환. (3) 모든 보간 결과는 `max_us`로 clamp (정의상 p95, p99 ≤ max). 이전에는 `<20, 100, 2000>` 설정에서 mean=27 µs / max=50 µs 같은 작은 값에서 p95/p99가 95/99 µs로 고정 출력되는 버그가 있었음 |

누적 over-run 카운터(`overruns`, `compute_overruns`, `skips`, `log_drops`, `pub_drops`)는 `rt_controller_node`가 별도 원자 변수로 관리 — Summary 리셋에 영향 없음.

### MPC Solve Timing Logger (aux thread, non-RT)

`on_configure`에서 `cb_group_aux_`에 1 Hz 타이머(`mpc_timing_timer_`)가 등록된다. 활성 컨트롤러의 `GetMpcSolveStats()` (기본 `nullopt`, MPC 보유 컨트롤러만 override)를 폴링해 `<session>/controller/mpc_solve_timing.csv`에 9열(`t_wall_ns,count,window,last_ns,min_ns,p50_ns,p99_ns,max_ns,mean_ns`)을 append한다. 10초마다 `RCLCPP_INFO`로 `count / window / p50 / p99 / max` 요약 로그. 컨트롤러가 `nullopt`를 반환하면 row를 쓰지 않는다 — reader는 빈 구간을 MPC 비활성 구간으로 해석.

Writer 구현: [`rtc_base/logging/mpc_solve_timing_logger.hpp`](../rtc_base/include/rtc_base/logging/mpc_solve_timing_logger.hpp) · neutral 타입 [`rtc::MpcSolveStats`](../rtc_base/include/rtc_base/timing/mpc_solve_stats.hpp).

---

## ROS2 인터페이스

### 고정 구독

| 토픽 | 타입 | 콜백 그룹 | 설명 |
|------|------|-----------|------|
| `/{robot_ns}/controller_type` | `String` | aux | 컨트롤러 이름으로 전환 |
| `/{robot_ns}/controller_gains` | `Float64MultiArray` | aux | 활성 컨트롤러 게인 업데이트 |
| `/{robot_ns}/request_gains` | `Bool` | aux | 현재 게인 요청 |

### 고정 퍼블리셔

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/system/estop_status` | `Bool` | RELIABLE/10 | 글로벌 E-STOP 상태 |
| `/{robot_ns}/active_controller_name` | `String` | TRANSIENT_LOCAL/1 | 활성 컨트롤러 이름 |
| `/{robot_ns}/current_gains` | `Float64MultiArray` | RELIABLE/10 | 현재 게인 응답 |

### 동적 구독 (컨트롤러 TopicConfig 기반)

| 역할 | 메시지 타입 | QoS | 설명 |
|------|------------|-----|------|
| `kState` | `JointState` | BEST_EFFORT/2 | 디바이스 관절 상태 |
| `kMotorState` | `JointState` | BEST_EFFORT/2 | 모터 공간 상태 |
| `kSensorState` | `HandSensorState` | BEST_EFFORT/2 | 촉각 센서 상태 |
| `kTarget` | `RobotTarget` | RELIABLE/10 | 관절/태스크 목표 |

### 동적 퍼블리셔 (컨트롤러 TopicConfig 기반)

| 역할 | 메시지 타입 | QoS | 설명 |
|------|------------|-----|------|
| `kJointCommand` | `JointCommand` | BEST_EFFORT/1 | 관절 커맨드 (position/torque) |
| `kRos2Command` | `Float64MultiArray` | RELIABLE/1 | ros2_control 호환 커맨드 |
| `kGuiPosition` | `GuiPosition` | RELIABLE/10 | GUI 현재 위치 표시 |
| `kRobotTarget` | `RobotTarget` | BEST_EFFORT/1 | 목표 위치 퍼블리시 |
| `kDeviceStateLog` | `DeviceStateLog` | BEST_EFFORT/1 | 관절/모터 상태 + 궤적 로그 |
| `kDeviceSensorLog` | `DeviceSensorLog` | BEST_EFFORT/1 | 센서 데이터 + 추론 결과 로그 |
| `kGraspState` | `GraspState` | RELIABLE/10 | 그래스프 감지 상태 |
| `kToFSnapshot` | `ToFSnapshot` | BEST_EFFORT/5 | ToF 센서 거리 + 핑거팁 SE3 포즈 |

### Digital Twin 자동 퍼블리셔

각 디바이스 그룹에 대해 `/{group}/digital_twin/joint_states` (RELIABLE/10) JointState를 자동 생성합니다. 설정 순서(joint_state_names)로 재정렬된 관절 데이터를 republish합니다.

---

## 파라미터 레퍼런스

코드에서 `DeclareAndLoadParameters()`로 선언되는 모든 파라미터입니다.

| 파라미터 | 타입 | 코드 기본값 | 설명 |
|---------|------|------------|------|
| `robot_namespace` | string | `"ur5e"` | 매니저 레벨 토픽 네임스페이스 (`/{robot_ns}/...`) |
| `control_rate` | double | `500.0` | 제어 주파수 (Hz) |
| `initial_controller` | string | `"joint_pd_controller"` | 시작 컨트롤러 (이름 또는 config_key) |
| `init_timeout_sec` | double | `5.0` | 초기화 타임아웃 (초). state 미수신 시 E-STOP |
| `auto_hold_position` | bool | `true` | 타겟 없을 때 현재 위치 자동 유지 |
| `enable_estop` | bool | `true` | E-STOP 워치독 활성화 |
| `device_timeout_names` | string[] | `[]` | E-STOP 감시 대상 디바이스 그룹 이름 |
| `device_timeout_values` | double[] | `[]` | 각 그룹의 state 토픽 타임아웃 (ms) |
| `enable_logging` | bool | `true` | CSV 로깅 활성화 |
| `enable_timing_log` | bool | `true` | 타이밍 CSV 로깅 활성화 |
| `enable_device_log` | bool | `true` | 디바이스별 CSV 로깅 활성화 |
| `log_dir` | string | `""` | 로그 디렉토리 (빈 문자열이면 자동 생성) |
| `max_log_sessions` | int | `10` | 최대 로그 세션 보관 수 |
| `use_sim_time_sync` | bool | `false` | MuJoCo 동기 루프 CV 기반 wakeup 모드 |
| `sim_sync_timeout_sec` | double | `5.0` | 시뮬레이션 동기 타임아웃 (초) |
| `kp` | double | `5.0` | (레거시) 기본 P 게인 |
| `kd` | double | `0.5` | (레거시) 기본 D 게인 |

### urdf 블록 파라미터 (시스템 레벨 URDF 설정)

| 파라미터 | 타입 | 필수 | 설명 |
|---------|------|------|------|
| `urdf.package` | string | 선택 | URDF가 포함된 ament 패키지명 |
| `urdf.path` | string | 선택 | 패키지 내 URDF 상대 경로 |
| `urdf.root_joint_type` | string | 선택 | `"fixed"` 또는 `"floating"` (기본: `"fixed"`) |
| `urdf.sub_models.<name>` | map | 선택 | 직렬 체인 모델 (root_link/tip_link). map 키가 모델명 |
| `urdf.tree_models.<name>` | map | 선택 | 분기 체인 모델 (root_link/tip_links[]). map 키가 모델명 |
| `urdf.passive_joints` | string[] | 선택 | 모든 모델에서 잠금할 관절 이름 |

> `sub_models`/`tree_models`의 map 키(모델명)는 `devices` 블록의 디바이스 그룹 이름과 매칭됩니다. 이를 통해 디바이스별 `root_link`/`tip_link`를 자동 해석할 수 있습니다.

### devices 블록 파라미터 (devices.{group_name}.* )

| 파라미터 | 타입 | 필수 | 설명 |
|---------|------|------|------|
| `joint_state_names` | string[] | 권장 | 관절 이름 (설정 순서 기준, 리오더링 기준). gains/limits 배열은 이 순서를 따릅니다. URDF (Pinocchio `model.names`) 와 *상대* 순서가 다르면 WARN + YAML/URDF 순서가 출력됩니다 (절대 인덱스 비교가 아니라 상대 순서 비교 — 디바이스가 시스템 URDF의 부분집합인 경우 false-positive 방지). |
| `joint_command_names` | string[] | 선택 | 커맨드 관절 이름 (미지정 시 joint_state_names 사용) |
| `motor_state_names` | string[] | 선택 | 모터 공간 관절 이름 |
| `sensor_names` | string[] | 선택 | 센서 이름 (촉각 핑거팁 등) |
| `safe_position` | double[] | 선택 | E-STOP 시 안전 위치 (관절 수와 동일) |
| `urdf.package` | string | 선택 | URDF 패키지 이름 |
| `urdf.path` | string | 선택 | 패키지 내 URDF 상대 경로 |
| `urdf.root_link` | string | 선택 | URDF 루트 링크 |
| `urdf.tip_link` | string | 선택 | URDF 엔드이펙터 링크 |
| `joint_limits.max_velocity` | double[] | 선택 | 최대 관절 속도 (URDF와 병합: 더 작은 값 적용) |
| `joint_limits.max_acceleration` | double[] | 선택 | 최대 관절 가속도 |
| `joint_limits.max_torque` | double[] | 선택 | 최대 관절 토크 (URDF와 병합) |
| `joint_limits.position_lower` | double[] | 선택 | 관절 위치 하한 (URDF와 병합: 더 큰 값 적용) |
| `joint_limits.position_upper` | double[] | 선택 | 관절 위치 상한 (URDF와 병합: 더 작은 값 적용) |

> URDF가 설정된 디바이스의 경우, YAML joint_limits와 URDF limits를 병합하여 더 보수적인 값을 적용합니다. YAML에 joint_limits가 없으면 URDF 값만 사용합니다.

---

## 설정 파일

### rt_controller_manager.yaml

```yaml
/**:
  ros__parameters:
    robot_namespace: "ur5e"
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
    log_dir: ""
    max_log_sessions: 10

    use_sim_time_sync: false
    sim_sync_timeout_sec: 5.0

    # System URDF (shared by all controllers)
    urdf:
      package: "ur5e_description"
      path: "robots/ur5e/urdf/ur5e.urdf"
      root_joint_type: "fixed"

    devices:
      ur5e:
        joint_state_names:
          - "shoulder_pan_joint"
          - "shoulder_lift_joint"
          - "elbow_joint"
          - "wrist_1_joint"
          - "wrist_2_joint"
          - "wrist_3_joint"
        sensor_names: []
        safe_position: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        urdf:
          root_link: "base_link"
          tip_link: "wrist_3_link"
        joint_limits:
          max_velocity:     [2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
          max_acceleration: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
          max_torque:       [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]
          position_lower:   [-6.28, -6.28, -3.14, -6.28, -6.28, -6.28]
          position_upper:   [6.28, 6.28, 3.14, 6.28, 6.28, 6.28]
```

### cyclone_dds.xml

CycloneDDS RT 성능 최적화 설정입니다. `CYCLONEDDS_URI` 환경변수로 자동 로드됩니다.

| 최적화 | 설정 | 효과 |
|--------|------|------|
| 멀티캐스트 | `AllowMulticast=spdp` | SPDP discovery만 멀티캐스트 (데이터는 유니캐스트) |
| Discovery 튜닝 | `LeaseDuration=5s`, `SPDPInterval=1s` | 빠른 participant 감지 |
| NACK/Heartbeat | `NackDelay=10ms`, `HeartbeatInterval=100ms` | 빠른 재전송 |
| 동기 전달 | `SynchronousDeliveryLatencyBound=inf` | 콜백 wake-up 지연 제거 |

> DDS 스레드 affinity는 `taskset`으로 처리 (CycloneDDS 0.11+에서 XML `<Threads>` 제거됨).

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rclcpp` | ROS2 클라이언트 라이브러리 |
| `sensor_msgs` | JointState 메시지 |
| `std_msgs` | Bool, Int32, String, Float64MultiArray |
| `rtc_controller_interface` | 컨트롤러 추상 인터페이스 + 레지스트리 |
| `rtc_controllers` | 내장 컨트롤러 (팩토리 등록) |
| `rtc_base` | 로깅, 스레딩, 타입, SPSC 버퍼, 세션 디렉토리 |
| `rtc_communication` | 네트워크 통신 (UDP 트랜시버) |
| `rtc_msgs` | JointCommand, GuiPosition, RobotTarget, DeviceStateLog, DeviceSensorLog, GraspState, HandSensorState |
| `rtc_urdf_bridge` | URDF→Pinocchio 모델 빌더 + `ModelConfig` 타입 (시스템 모델 설정) |
| `pinocchio` | URDF 기구학 검증 + joint limits 병합 |
| `yaml-cpp` | YAML 설정 파싱 |
| `ament_index_cpp` | 패키지 리소스 경로 탐색 |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_controller_manager
source install/setup.bash
```

**빌드 산출물:**
- 정적 라이브러리: `librtc_controller_manager_lib.a` (export 됨, 외부에서 link)
- 실행 파일 없음 — robot-specific bringup 패키지(예: `ur5e_bringup`의 `ur5e_rt_controller`)가 `rtc::RtControllerMain(argc, argv, node_name)`을 호출하여 자체 exec를 만든다. `node_name`은 robot bringup의 executable 이름과 일치시킨다 (exec ↔ ROS 노드 ↔ 로거 식별자 정렬). 자세한 원칙은 [agent_docs/design-principles.md](../agent_docs/design-principles.md) 참고.

---

## 의존성 그래프 내 위치

```
rtc_base + rtc_controller_interface + rtc_controllers + rtc_communication
    |
rtc_controller_manager  <- 500 Hz RT 제어 실행 엔진
    ^
    +-- ur5e_bringup  (로봇별 진입점 + launch 파일)
```

---

## 변경 내역

### v5.18.0

| 영역 | 변경 내용 |
|------|----------|
| **E-STOP RT 안전** | `estop_reason_`를 `std::string` → `std::array<char, 128>` 고정 버퍼로 변경 -- RT 경로 힙 할당 제거 |
| **E-STOP 로깅** | `RCLCPP_ERROR`/`INFO`를 `estop_log_pending_` atomic 플래그로 대체 -- non-RT `DrainLog()` 스레드에서 지연 출력 |

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
