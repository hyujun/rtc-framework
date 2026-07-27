# rtc_controller_manager


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **설정 가능한 RT 제어 루프 매니저** 패키지입니다 (`control_rate` YAML; 설계 범위 100 Hz–5 kHz, default 500 Hz). 컨트롤러 수명 관리, 결정론적 제어 루프, 퍼블리시 오프로드, CSV 로깅, E-STOP 안전 메커니즘을 통합 관리합니다. Phase 3 구조 개편을 통해 로봇에 독립적(robot-agnostic)으로 설계되었습니다.

**핵심 기능:**
- `clock_nanosleep` 기반 결정론적 제어 루프 — 주기는 `control_rate` YAML 파라미터로 설정 (default 500 Hz, 설계 범위 100 Hz–5 kHz; 상수 `rtc::kMin/kMax/kDefaultControlRateHz`) (실제 로봇) / CV 기반 동기 루프 (시뮬레이션)
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
│   ├── controller_timing_profiler.hpp     <- 락-프리 타이밍 프로파일러
│   ├── device_state_cache.hpp             <- `DeviceStateCache` POD (SeqLock payload, backend-shared)
│   ├── device_backend.hpp                 <- `DeviceBackend` 추상 인터페이스 + `DeviceBackendConfig`
│   └── device_backend_registry.hpp        <- 백엔드 type_tag 레지스트리 + `RTC_REGISTER_DEVICE_BACKEND` 매크로
├── src/
│   ├── rt_controller_node.cpp             <- 생성자/소멸자, 콜백 그룹, 타이머, Lifecycle 콜백
│   ├── rt_controller_node_params.cpp      <- 파라미터 선언/로딩, 디바이스 설정
│   ├── rt_controller_node_device_config.cpp <- URDF/모델 파싱, 디바이스 이름 설정
│   ├── rt_controller_node_subscriptions.cpp <- 구독자 생성 (SeqLock writer 콜백 등록)
│   ├── rt_controller_node_publishers.cpp    <- 퍼블리셔 생성 (per-controller LifecyclePublisher)
│   ├── rt_controller_node_services.cpp      <- 서비스 (LoadController, SwitchController 등)
│   ├── rt_controller_node_switch.cpp        <- 컨트롤러 전환 로직 + active_controller_name latched publish
│   ├── rt_controller_node_callbacks.cpp     <- 디바이스 센서/타겟 콜백 (SeqLock writer)
│   ├── rt_controller_node_rt_loop.cpp       <- RT 루프 (@ control_rate), 워치독, 로그 드레인
│   ├── rt_controller_node_publish.cpp       <- SPSC 드레인 → ROS2 publish (eventfd wakeup)
│   ├── rt_controller_node_estop.cpp         <- E-STOP 트리거/클리어/퍼블리시
│   ├── rt_controller_main_impl.cpp          <- RtControllerMain() 구현 (라이브러리)
│   └── device_backend_registry.cpp          <- 레지스트리 싱글톤 + duplicate-tag warn
└── config/
    ├── cyclone_dds.xml                      <- CycloneDDS RT 성능 최적화 설정 (실제 control_rate / devices / urdf 는 `<robot>_bringup/config/<robot>/{sim,robot}.yaml` 의 `devices.<group>.backend:` 등으로 선언)
    └── test_fixtures/controllers/           <- unit test 전용 controller YAML (config_variant=test_fixtures 로만 로드 — production bring-up 은 참조하지 않음)
```

---

## 스레딩 아키텍처 (layout v4.1)

`SelectThreadConfigs()`가 물리 코어 수에 따라 스레드 레이아웃을 자동 선택합니다 (SSoT: `rtc_base/threading/thread_config.hpp`).

| 스레드 | 코어 | 스케줄러 | 주파수 | 역할 |
|--------|------|----------|--------|------|
| **rt_loop** (rt_control) | 1 | SCHED_FIFO 90 | `control_rate` Hz (default 500) + 50 Hz | `clock_nanosleep` 제어 루프 + 워치독. tick 종료 시점에 `DeviceBackend.WriteCommand` 를 inline 호출 (actuator publish) + `nrt_publish_buffer_` push |
| **rt_callback_executor** | 2 | SCHED_FIFO 70 | 이벤트 | 디바이스별 JointState, MotorState, SensorState 구독 (`cb_group_rt_callback_`, MutuallyExclusive). DDS receive thread 가 launch-time taskset 으로 같은 Core 2 에 co-pin (CFS 유지) |
| **nrt_logging_executor** | tier-aware (4c: 0 / ≥6c: dedicated) | SCHED_OTHER -5 | 100 Hz | `cm_timing_log.csv` 드레인 + 1초 타이밍 서머리 + deferred E-STOP 메시지 |
| **nrt_publish_thread** | tier-aware (4c: 0 / ≥6c: dedicated, nrt_callback core 공유) | SCHED_OTHER 0 | 이벤트 | `nrt_publish_buffer_` (cap 16) SPSC 드레인 → `controller.PublishNonRtSnapshot` (Transforms / grasp_state / wbc_state / tof_snapshot). std::jthread + eventfd wakeup |
| **nrt_callback_executor** | tier-aware (4c: 0 / ≥6c: dedicated) | SCHED_OTHER 0 | 이벤트 | 컨트롤러 전환, E-STOP 상태 퍼블리시, RobotTarget 외부 입력, lifecycle services, 컨트롤러 LifecycleNode default group (owned subs) |

> **v4.1 변경**: RT cluster 가 Core 1 부터 시작 (Core 0 = OS / DDS / IRQ 전용). DDS receive thread 는 새 rt_callback core (Core 2) 에 co-pin. nrt_logging / nrt_callback 이 모든 ≥ 6c tier 에서 Core 0 와 분리. 정확한 tier 별 cpu_core 는 [rtc_base/include/rtc_base/threading/thread_config.hpp](../rtc_base/include/rtc_base/threading/thread_config.hpp) 또는 [docs/RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md) 표 참조.
>
> **v4 변경 (참고)**: v3 의 `rt_outbound` jthread + `publish_buffer_` SPSC + eventfd 는 제거. actuator publish 는 `rt_control` 이 inline 으로 수행 (RT-safe contract). cross-core hand-off 가 사라져 latency / 자원 동시 감소.

> `mlockall(MCL_CURRENT | MCL_FUTURE)`를 `rclcpp::init()` 전에 호출하여 페이지 폴트를 방지합니다.

### Lifecycle 상태 전환

`RtControllerNode`는 `rclcpp_lifecycle::LifecycleNode`를 상속하며, 관리된 상태 전환을 지원합니다.

| 콜백 | 리소스 티어 | 역할 |
|------|-----------|------|
| `on_configure` | Tier 1 | 콜백 그룹, 파라미터, 컨트롤러, 퍼블리셔/구독, 타이머, eventfd |
| `on_activate` | Tier 2 | RT 루프 + 퍼블리시 오프로드 스레드 시작 |
| `on_deactivate` | — | RT 루프/퍼블리시 스레드 중지, E-STOP 클리어, 상태 초기화 |
| `on_cleanup` | — | `on_configure` 역순 리소스 해제. 단 eventfd 는 예외로 device backend **해제 후** close — backend 의 state-lane sub 은 `on_deactivate` 로 죽지 않고 그 state-ready 콜백이 이 fd 를 쓰기 때문 (issue #224) |
| `on_error` | — | E-STOP 트리거, 스레드 중지, 전체 정리 → SUCCESS (Unconfigured 복구) |

**안전 퍼블리셔:** `estop_pub_`, `active_ctrl_name_pub_`는 `rclcpp::create_publisher` standalone으로 생성되어 lifecycle 상태와 무관하게 동작합니다.

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
   - `on_activate`: `SelectThreadConfigs()` → `StartRtLoop()` + `StartNrtPublishLoop()` 시작
3. **Phase 2:** Active 상태 대기 (polling)
4. **Phase 3:** rt_callback / nrt_logging / nrt_callback 전용 executor 전환 (lifecycle services는 nrt_callback_executor에서 처리)

---

## 제어 루프 흐름 (정기 tick @ `control_rate`; 예: 500 Hz → 2 ms/tick, 1 kHz → 1 ms, 2 kHz → 0.5 ms)

### Phase 0: 준비 검사
- **디바이스 준비 게이트** — 설정된 **모든** 디바이스 그룹이 timeout 안에 state 를 한 번 이상 보고했는지 확인. 하나라도 미보고면 tick 은 그대로 반환하고 `Compute()` / `WriteCommand()` 는 실행되지 않는다. `init_timeout_sec` 경과 시 `{group}_init_timeout` E-STOP + 노드 종료 (미보고 그룹 이름 포함). 준비 여부는 `backends_[slot]->LastStateStamp()` 가 단일 출처 — CM 은 별도 플래그/타임스탬프를 두지 않는다 (#198 §1/§2)
- Auto-hold 모드: 외부 타겟 없으면 모든 디바이스가 valid 상태일 때 현재 위치를 타겟으로 초기화
- `init_complete_` 이후 정상 루프 진입

### Phase 1: 비차단 상태 획득
- 활성 컨트롤러의 TopicConfig.groups 순서대로 `backends_[slot]->ReadState/ReadMotorState/ReadSensorState(cache)` → `ControllerState.devices[]` 복사 (RT-safe, 락-프리)
- 각 backend는 내부에 `SeqLock<DeviceStateCache>`를 보유 — 센서 callback (writer) + RT loop (reader) 분리
- `try_lock`으로 타겟 스냅샷 (`device_target_snapshots_`)

### Phase 2: 제어 연산
- `timing_profiler_.MeasuredCompute(controller, state)`
- 활성 컨트롤러의 `Compute()` 호출 -> `ControllerOutput` 반환
- 벽시계 시간 히스토그램 자동 수집 (20 버킷, 100us 간격)

### Phase 3: 퍼블리시 (inline actuator + non-RT 오프로드)
- `PublishSnapshot` 생성
- **Actuator publish (inline, RT-safe)**: rt_loop tick 안에서 `DeviceBackend.WriteCommand` 를 그룹별로 직접 호출 (RT-safe contract)
- **Controller-owned non-RT publish (오프로드)**: `nrt_publish_buffer_.Push()` (SPSC cap 16) → `nrt_publish_thread` 가 드레인하여 `controller.PublishNonRtSnapshot` 호출 (Transforms / grasp_state / wbc_state / tof_snapshot)
- 그룹별 commands, actual, motor, sensor, inference 데이터 포함
- **Per-group command type 해석 (`CommandType::kPdFeedforward`, C-3 S1)**: `DeviceOutput::command_type` (per-device `std::optional<CommandType>`, nullopt = 상속) 이 설정돼 있으면 `ControllerOutput::command_type` (글로벌 기본값) 을 override 한다 — `gc.command_type = dout.command_type.value_or(output.command_type)`. `DeviceOutput::feedforward[]` (per-joint Nm) 도 `GroupCommandSlot::feedforward[]` 로 그대로 복사된다. RT loop 는 그룹별로 resolve 된 `gc.command_type` 을 `WriteCommand(slot, gc.command_type)` 에 전달 — snapshot-global `command_type` 이 아니다. Mixed-command 컨트롤러(예: WBC — arm=`kPosition`, hand=`kPdFeedforward`)가 그룹마다 다른 command type 을 낼 수 있는 이유. `kPdFeedforward` = PD position-servo backbone(`values`=position target) + per-joint feedforward torque overlay; wire 상 `JointCommand.command_type = "pd_feedforward"` 문자열로 인코딩되며, feedforward 채널을 모르는 backend 는 position tracking 으로 폴백한다.

### Phase 4: 타이밍 & 로깅
- 위상별 소요 시간 계산 (state_acquire, compute, publish) + 지터 측정
- `LogEntry` -> `log_buffer_.Push()` (SPSC)
- 1000 이터레이션마다 타이밍 서머리 출력 신호 (실제 출력은 nrt_logging_executor에서)

### 워치독 (50 Hz — divisor = `control_rate` / 50, rate 무관 고정)
- **설정된 모든 디바이스 그룹**이 감시 대상이다. `device_timeout_names` 는 그룹별 timeout **값**만 공급하며, 목록에 없는 그룹은 `device_timeout_default_ms` 로 감시된다 (#198 §1 — 예전에는 목록에 없는 그룹이 워치독·준비 게이트 양쪽에서 아예 보이지 않았다)
- 각 그룹의 state 수신 간격이 timeout 초과 시 E-STOP
- 한 번도 보고하지 않은 그룹은 워치독이 아니라 **준비 게이트**가 담당한다 (Phase 0)
- Phase 4 이후 워치독은 `devices.<group>.backend.state_topic` 만 추적 (motor/sensor lane은 backend 내부의 `NotifyStateReady` 콜백 한 곳에서 합쳐서 트리거)

---

## 컨트롤러 관리

### 시스템 URDF + ModelConfig 파싱

노드 초기화 시 최상위 `urdf:` 파라미터 섹션에서 시스템 레벨 URDF 경로와 모델 토폴로지를 파싱합니다.

```
urdf.package + urdf.path → ament resolve → 절대 URDF 경로
urdf.root_joint_type     → "fixed" | "floating"
urdf.extended            → bool. true 면 sidecar <stem>.closure.yaml 로드 (loop closure)
urdf.closure_path        → sidecar 명시 override (pkg 상대). 미지정 시 URDF 경로에서 유도
urdf.sub_models.<name>   → {root_link, tip_link}                (직렬 체인, map 키 = 모델명)
urdf.tree_models.<name>  → {root_link, tip_links[]}             (분기 체인, map 키 = 모델명)
urdf.passive_joints      → [string, ...]                         (잠금 관절)
```

- `sub_models`/`tree_models`의 map 키(모델명)는 `devices` 블록의 디바이스 그룹 이름과 매칭됩니다
- 디바이스별 `root_link`/`tip_link` 미지정 시 시스템 `sub_models`/`tree_models`에서 자동 해석
- 디바이스별 URDF 경로 미지정 시 시스템 URDF 경로를 폴백으로 사용
- **Extended-URDF**: `urdf.extended: true` 면 URDF 옆 `<stem>.closure.yaml` sidecar(예: `foo.urdf.xacro` → `foo.closure.yaml`)를 로드해 공유 `PinocchioModelBuilder`가 loop closure constraint + `q_ref`를 build (`GetConstraintModels()` 등으로 노출) **하고, sidecar `actuated_joints` 외의 모든 movable 관절을 loop-passive 로 간주해 reduced/tree 서브모델에서 잠근다** — 따라서 `actuated_joints` 는 그 URDF 의 완전한 active 집합이어야 한다 (arm + closed-chain hand 병합 시 arm 관절 포함; 상세 계약·locked-count 로그는 [rtc_urdf_bridge/README.md](../rtc_urdf_bridge/README.md#extended-urdf-closed-chain-sidecar-nameclosureyaml)). closure 해석은 최상위 `urdf.path` 경로든 아래 devices-fallback 경로든 **동일하게** 실행되며, `extended: true` 인데 sidecar 미발견 시 loud `RCLCPP_ERROR` 후 loop constraint 없이 진행(plain URDF).
- **하위 호환**: 최상위 `urdf:` 없으면 기존 `devices.{group}.urdf` 에서 읽고(fallback), 이 경로도 최상위 `urdf.extended`/`urdf.closure_path` 를 그대로 적용한다

### 로딩 (시작 시)

1. `ControllerRegistry::Instance().GetEntries()` 조회
2. 시스템 `PinocchioModelBuilder` 한 번 빌드 (`std::shared_ptr`) — `system_model_config_`로 URDF 파싱 + full/sub/tree 모델 구축을 1회만 수행. 빌드 실패 시 컨트롤러는 자기 builder를 만드는 폴백 경로로 동작
3. **컨트롤러 bring-up은 3-pass 구조** (`on_configure` 시점에 `device_name_configs_` 가 이미 채워져 있도록 보장):
   * **Pass 1 — 인스턴스화 + PreConfigure**: 팩토리 생성 → `SetSystemModelConfig()` → `SetSharedModelBuilder(shared)` → 전용 `rclcpp_lifecycle::LifecycleNode`(이름 = `config_key`, 네임스페이스 = `/<config_key>`, `NodeOptions().use_global_arguments(false)`) 생성 → `PreConfigure(node, yaml)` 호출 (base가 `LoadConfig(yaml)` 만 실행 → `topic_config_` 채움; **RegisterLog / 리소스 할당 없음**) → `controllers_` + `controller_nodes_` + 임시 `controller_yamls` 보관
   * **Pass 2 — device-name 해석**: 모든 컨트롤러의 `GetTopicConfig()` 로 `controller_topic_configs_` / `active_groups_` / `group_slot_map_` 빌드 → `LoadDeviceNameConfigs()` (yaml `devices.<group>.{joint_state_names, motor_state_names, sensor_names, joint_limits, ...}` 파싱) → 각 컨트롤러에 `SetControlRate(...)` + `SetDeviceNameConfigs(map)` (후자가 `OnDeviceConfigsSet()` 훅을 트리거 → 컨트롤러가 `joint_names`/`motor_names` 등을 자기 멤버에 캐시)
   * **Pass 3 — on_configure**: 보관해 둔 `(node, yaml)` 로 `on_configure(unconfigured, node, yaml)` 호출. 멱등 가드 덕분에 base는 yaml 재파싱 안 함. 이 시점에 컨트롤러가 `RegisterLog<>(...)` 람다에 `joint_names_copy = joint_state_names` 를 capture 하므로 CSV header writer 가 정확한 폭으로 출력 (회귀: 이 단계 분리 전에는 capture 시점에 빈 vector였음)
   * **세부 사항**: `use_global_arguments(false)` 는 필수 — 부모 프로세스의 CLI remap (`--ros-args -r __node:=...`) 이 글로벌 컨텍스트에 살아 있어, default `NodeOptions` 으로 만들면 자식 노드 이름·네임스페이스가 부모로 덮어씌워지고 launch_ros 가 부모에 spurious lifecycle ACTIVATE 를 또 보내는 부작용까지 발생. `SetSharedModelBuilder` 는 `GetSharedModelBuilder()` 로 컨트롤러가 가져가는 공유 핸들 — 데모 컨트롤러는 `InitArmModel` / `InitModels` 진입 시 이 핸들을 우선 사용하므로 동일한 ModelConfig로 `xacro → tinyxml2 → Pinocchio` 파이프라인을 컨트롤러 수만큼 반복하는 비용을 제거 (데모 빌업 기준 4회 → 1회)
4. 컨트롤러 `Name()` 과 `config_key` 양쪽을 `controller_name_to_idx_` 맵에 등록 — 이 맵은 **하나의 네임스페이스**이고 `switch_controller` / `initial_controller` 가 둘 중 어느 문자열로도 해석된다. 따라서 **두 식별자 모두 전역 유일해야 한다** (아래 가드 참조)
5. `initial_controller` 파라미터로 초기 활성 컨트롤러 선택
6. `main()`에서 모든 `controller_nodes_`를 `nrt_callback_executor`에 attach — 컨트롤러가 자체 소유한 sub/pub 콜백이 non-RT 스레드에서 처리됨
7. CM `on_activate` -> 각 컨트롤러 `on_activate(active)` 호출(LifecyclePublisher 활성화 후) -> RT 루프 + publish 스레드 시작
8. CM `on_deactivate` -> RT/publish 스레드 정지 후 각 컨트롤러 `on_deactivate(inactive)` 호출
9. CM `on_cleanup` 진입 시 각 컨트롤러에 `on_cleanup(inactive)` 호출 후 `controller_nodes_`/`controllers_` 해제

#### 식별자 충돌 가드 (2-tier, issue #196 Phase 5)

`controller_name_to_idx_` 가 단일 네임스페이스이므로, 식별자가 겹치면 오퍼레이터가 선택하지 않은 컨트롤러가 조용히 도는 결과가 된다. CM 은 이를 두 단계로 거부한다 — 둘 다 **경고가 아니라 configure 실패**다.

| Tier | 시점 | 검사 대상 | 왜 거기인가 |
|---|---|---|---|
| 1 | Pass 1 **이전** | `config_key` vs `config_key` | 팩토리 호출 전이라 컨트롤러·LifecycleNode 가 하나도 안 만들어진 채 거부 |
| 2 | Pass 1 안 | `Name()` 과 `config_key` 를 이미 등록된 모든 식별자와 대조 | `Name()` 은 인스턴스의 virtual 이라 `factory()` 전에는 알 수 없다. 대신 latch 후 D1 체크포인트에서 거부 — publisher / device backend / log channel / service / timer / RT thread 는 전부 D1 이후라 아직 없다 |

**대가 — 한 컨트롤러 클래스를 두 config_key 로 등록할 수 없다.** `Name()` 은 모든 구현에서 클래스 상수이므로 같은 클래스의 두 번째 등록은 항상 첫 번째의 `Name()` 과 충돌해 bring-up 전체가 거부된다. "같은 클래스, 두 YAML 설정" 이 필요하면 인스턴스별로 `Name()` 을 다르게 돌려주는 서브클래스를 만들어야 한다. 컨트롤러 클래스를 복사한 뒤 `Name()` 문자열을 안 고치는 실수가 바로 이 경로로 잡힌다.

> Lifecycle 훅 규약은 [`rtc_controller_interface/README.md`](../rtc_controller_interface/README.md#lifecycle-훅-ros2_control-정렬-기본-구현-제공) 참조. 컨트롤러는 per-node topic 소유권을 가지며(`/<config_key>/<topic>`), RT 경로에서 `node_` 접근은 금지입니다.

### 토픽 소유 lane

토픽 소유는 3개 lane 으로 나뉜다 (issue #138: controller YAML 에는 `ownership:` field 가 없다 — YAML `topics:` entry 는 전부 controller-owned):

| 소유 lane | 생성 주체 | 역할 예시 | QoS 경로 |
|-----------|-----------|-----------|---------|
| DeviceBackend | `DeviceBackend` (CM이 group마다 1 instance 보유) | HW/sim ↔ controller 경계 트래픽: `state`, `motor_state`, `sensor_state`, `joint_command`, `ros2_command` (`devices.<group>.backend:` 선언) | Backend 자체 sub/pub + SeqLock state cache. RT loop → `WriteCommand(slot, cmd_type)` |
| CM 고정 | `RtControllerNode` (hardcode, YAML 무관) | digital twin republish (`/rtc_cm/<group>/joint_states`), `/system/estop_status`, `/rtc_cm/active_controller_name`, `/rtc_cm/list_controllers`, `/rtc_cm/switch_controller`, `/rtc_cm/reset_fault` | CM 직접 publish / service |
| controller-owned | 컨트롤러별 `LifecycleNode` (YAML `topics:` entry 전부) | 컨트롤러 입력/출력: `target`, `grasp_state`, `wbc_state`, `tof_snapshot`, `robot_transforms` | 컨트롤러가 on_configure에서 sub/pub 생성, publish 스레드가 SPSC 소비 후 `controllers_[active]->PublishNonRtSnapshot(snap)` 호출 |

원칙: CM은 **CM 고정 토픽** + **controller-node 관리** (switch / active / E-STOP) 만 소유하고, controller YAML 의 sub/pub 은 만들지 않는다 (issue #138: manager-target 경로 폐기). 컨트롤러가 생산하거나 소비하는 의미적 데이터(목표·GUI·grasp·wbc·tof) 는 컨트롤러 LifecycleNode가 직접 sub/pub. Target topic은 controller가 자체 sub하고 그 콜백에서 `DeliverTargetMessage` → `SetDeviceTarget`을 호출 — SetDeviceTarget은 SPSC 큐로 marshal한 뒤 RT thread가 `Compute()` 안에서 drain한다 (single-writer SeqLock invariant, 2026-05-17 RT-4 cleanup).

### 런타임 전환 (`/rtc_cm/switch_controller` 서비스, sync)

호출자(BT, CLI 등)가 `rtc_msgs/srv/SwitchController` 요청을 보내면 aux 스레드의
`SwitchActiveController(name, message)` 헬퍼가 다음 sync 시퀀스를 실행한다 (D-A4).

1. precondition: E-STOP idle, single-active (D-A1, D-B6) → 위반 시 `ok=false`
2. `controller_name_to_idx_` 맵에서 이름/config_key로 인덱스 조회 (`Unknown`이면 거부)
3. target controller `on_activate(prev_state)` 호출 — base는 no-op SUCCESS; controller가 첫 `Compute()` tick에서 현재 device state로 자체 seed (RT-4 controller-internal init policy)
4. `active_controller_idx_.store(idx, release)` — RT loop dispatch atomic 전환
5. RT tick 1회 대기 (`sleep_for(1.5 × dt)`, OQ-2 = sleep_for) — F-3 race benign
6. previous controller `on_deactivate(prev_state)` 호출
7. `/{robot_ns}/active_controller_name` 퍼블리시 (TRANSIENT_LOCAL, rewire trigger 영구 유지)
8. response `ok=true, message="ok"` (또는 "switched -> name")

조회용 `/rtc_cm/list_controllers` (`ListControllers.srv`) — empty request, response =
`ControllerState[]` (name/state="active|inactive"/type/is_active/claimed_groups). 두 srv
모두 `cb_group_nrt_callback_` (RT path와 분리), `rmw_qos_profile_services_default`.

### 게인 채널 (per-controller ROS 2 parameter)

Phase A~E 마이그레이션 (2026-04-26) 이후 게인 채널은 **CM 소유가 아니다**. 각 컨트롤러 LifecycleNode (`/<config_key>`) 가 자체 `declare_parameter` + `add_on_set_parameters_callback`로 노출하며, parameter 콜백은 `nrt_callback_executor` 위에서 실행되어 SeqLock writer로 mutate→Store 한다 (RT path는 `Load()` 스냅샷만 사용). `~/controller_gains` / `~/request_gains` / `~/current_gains` 토픽과 `RTControllerInterface::UpdateGainsFromMsg`/`GetCurrentGains` 가상 메서드는 이 단계에서 제거됨. 컨트롤러별 노출 항목은 [agent_docs/controllers.md](../agent_docs/controllers.md) §Gains.

Force-PI grasp 같은 one-shot 이벤트(상태가 아닌 transition)는 컨트롤러가 `~/grasp_command` ([rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv)) srv server로 별도 advertise 한다 (active controller만).

---

## E-STOP 및 안전 메커니즘

### E-STOP 트리거

| 트리거 | 조건 | 결과 |
|--------|------|------|
| `{group}_init_timeout` | `init_timeout_sec` 내 해당 그룹이 state 미보고 | `TriggerGlobalEstop("{group}_init_timeout")` + 노드 종료 |
| `{group}_timeout` | 디바이스 그룹의 state 토픽이 설정된 ms 초과 미갱신 (50Hz 워치독) | `TriggerGlobalEstop("{group}_timeout")` |
| `consecutive_overrun` | >= 10회 연속 RT 루프 오버런 | `TriggerGlobalEstop("consecutive_overrun")` |
| `sim_sync_timeout` | 시뮬레이션 모드에서 `sim_wake_eventfd_` 타임아웃 (state 미수신) | `TriggerGlobalEstop("sim_sync_timeout")` + 노드 종료 |

### 글로벌 E-STOP 동작

- `TriggerGlobalEstop()`: 멱등(idempotent), `compare_exchange_strong`으로 1회만 실행
- 모든 컨트롤러에 `TriggerEstop()` + `SetHandEstop(true)` 전파
- **actuator 로 나가는 command 를 CM 이 차단** — 아래 절 참조
- `/system/estop_status`에 `true` 퍼블리시 (지연, 아래 참조)
- RT 루프는 E-STOP 후에도 계속 실행 (타이밍/로깅 유지)
- **RT 안전:** `estop_reason_`은 `std::array<char, 128>` 고정 크기 버퍼 (힙 할당 없음). RCLCPP 로깅은 `estop_log_pending_`, `/system/estop_status` publish 는 `estop_status_pending_` atomic 플래그를 통해 non-RT `DrainLog()` (100 Hz) 에서 지연 수행 — `TriggerGlobalEstop` / `ClearGlobalEstop` 은 RT 루프에서 도달 가능하므로 plain publisher 를 그 자리에서 호출하면 RT-10 위반이다 (#198 Phase 3). 드레인은 *드레인 시점의* `global_estop_` 값을 발행하므로 두 드레인 사이의 trigger/clear 쌍은 stale 값이 아니라 최종 상태로 수렴한다. lifecycle teardown 은 `drain_timer_` 를 없애므로 `FlushEstopStatus()` 가 마지막 전이를 직접 흘린다

### Backend command-type 선언 (#198)

`DeviceBackend::AcceptsCommandType(CommandType)` — **기본값은 position-only** 다. 아무 선언도 하지 않은 backend 는 이 트리의 모든 actuator lane 이 하는 그 하나를 한다고 간주한다. 더 넓게 선언하는 것은 opt-in 이며, "`WriteCommand` 가 enum 을 받는다" 가 아니라 **wire format 이 그 구분을 실어 나른다** 는 뜻이다.

CM 은 `on_configure` 에서 (backend 생성 후 + controller `on_configure` 후 — 양쪽이 다 확정된 최초 시점) 등록된 **모든** controller 의 `GetCommandType()` 을 그 controller 가 구동하는 group 의 backend 와 대조하고, 불일치면 configure 를 거부한다 (`ValidateCommandTypeSupport`).

왜 필요했나: `WriteCommand` 의 `command_type` 인자는 **권고**였다. `ur_driver_native` 는 그것을 무시하고 받은 값을 전부 `forward_position_controller` 의 `Float64MultiArray` 로 발행한다. 따라서 torque 모드 controller 가 거기 묶이면 **N·m 가 관절 각도로 wire 에 나가고**, CM 자신의 hold command 는 (동역학 모델이 없어 kTorque 를 0.0 으로 낸다) 팔을 **놓아주는 대신 zero configuration 으로 이동시킨다.** 그 backend 소스는 이 페어링이 "validated at YAML time" 이라고 적어 두었으나 실제로 검증하는 곳은 어디에도 없었다.

현재 shipped controller config 9개는 전부 `command_type: "position"` 이라 도달 상태는 아니었다 — YAML 한 단어 거리였고, 그래서 지금 닫는다.

> **한계**: 이 검사는 controller-level `GetCommandType()` 만 본다. `ControllerOutput::devices[i].command_type` 의 per-device override 는 RT tick 마다 결정되므로 configure 시점에 알 수 없다. mixed-mode controller 를 도입할 때 이 구멍을 함께 닫아야 한다.

### Backend safe-output 계약 (#198 Phase 4)

`DeviceBackend::WriteSafeCommand()` — **pure virtual, 인자 없음.**

CM 이 호출하는 조건: actuator 에 normal command 를 보내면 안 된다고 판단했고 **hold 도 만들 수 없을 때** (device 가 non-finite 위치를 보고 → "네가 있는 곳으로 servo" 할 대상 값이 없음). 이전에는 그 경우 zero-length command 를 내보내고 fail-closed 로 간주했다. **아니다** — 세 backend 모두 zero-length 를 early-return 하므로, 하드웨어에 도달하는 것은 침묵이고 그 의미는 "하던 것을 계속하라, 모터는 물린 채" 다. 침묵은 안전 상태가 아니라 **결정의 부재**다.

인자가 없는 이유: 그 순간 CM 의 device 관점이 신뢰 불가라는 것이 요점이다. 여전히 known-good 값을 들고 있는 유일한 당사자가 backend 다 (보통 마지막으로 성공 발행한 command). `SafetyMode` enum 을 두지 않은 이유도 같다 — 가장 안전한 동작은 backend 별로 다르고 backend 가 그것을 소유한다. 진짜 drive-disable 을 갖게 된 backend 는 여기 구현하면 되고, 상위가 새 mode 를 배울 필요가 없다.

pure virtual 인 이유: 아무것도 안 하는 default 는 이것이 대체하려는 그 침묵을 그대로 재생산하고, 새 backend 작성자는 질문 자체를 받지 않게 된다.

> **정직한 범위 고지**: 현재 shipped backend 3종이 낼 수 있는 답은 "마지막 good command 재발행" 뿐이며, 물리적으로는 침묵 경로가 남긴 자리에 그대로 있다. 달라지는 것은 (a) 의도가 wire 와 rosbag 에 **명시**된다, (b) actuator lane 이 살아 있어 이를 감시하는 receiver 가 구분할 수 있다, (c) 더 나은 동작이 가능한 backend 가 도착하기 전에 **계약이 먼저 존재한다** (ARCH-3). hand 는 UDP write opcode 가 `kWritePosition` 하나뿐이라 bounded position hold 외의 선택지가 없다.

### E-STOP 시 actuator command 차단 (#198 Phase 3)

latch 가 서면 controller 가 계산한 output 은 **`DeviceBackend::WriteCommand` 에 도달하지 않는다.** `BuildHoldOutput()` (측정 위치 hold; torque 모드는 0 N·m) 로 치환된다.

- **왜 CM 이 하는가**: actuator 안전이 각 controller 의 E-STOP hook 구현 정확성에 의존해서는 안 된다. in-tree 반례가 이미 있다 — `OperationalSpaceController` 의 E-STOP 경로는 자기 주석으로 "position-scale 값을 kTorque command 로 낸다 (Do NOT rely on this as a safe torque E-STOP)" 고 명시하고, `PController` 는 `SetHandEstop` 을 override 하지 않아 base no-op 으로 빠진다
- **의도적으로 버리는 것**: in-tree `ComputeEstop` 은 전부 hold 가 아니라 설정된 `safe_position_` 으로의 slew 다. 그 후퇴는 controller 단위 정책이고 CM 은 검증할 모델이 없다. 복원은 Phase 4 의 backend safe-output 계약 (position / torque / hand 각각의 semantics) 소관이다
- **합성 순서**: #196 Phase 2b 의 output validation **뒤에** 얹힌다 — 우회가 아니다. validator 는 terminal state 에서도 계속 돌고 (`rejected_output_count_`), E-STOP 치환은 `estop_substituted_outputs_` 로 따로 센다. 둘 다 같은 hold 를 내므로 **카운터가 갈라지지 않으면 어느 가드가 막았는지 테스트가 구분할 수 없다**

### 오버런 복구

- 누락된 tick 건너뛰기 (burst 없음, 주기 재정렬)
- `overrun_count_`, `skip_count_`, `consecutive_overruns_` 원자적 카운트
- 컴퓨트 자체가 예산을 초과한 tick은 `cm_timing_log.csv`의 `t_compute_us` 컬럼으로 사후 분석 (별도 in-memory 카운터 없음)

### RT 루프 종료 (sim/robot 공통)

- Robot 모드: `clock_nanosleep` 다음 tick에 `rclcpp::ok()` 검사 → 2 ms 내 종료
- Sim 모드 (`use_sim_time_sync=true`): `WaitForNextTick()` 이 `sim_wake_eventfd_` 를 `sim_sync_timeout_sec` (기본 5 s) 만큼 `poll()` 한다. `OnRequestStop()` 이 그 fd 에 한 번 write 하므로 destructor / on_deactivate / on_shutdown 경로는 타임아웃을 기다리지 않고 즉시 join
- 이 lane 은 issue #198 Phase 2 에서 `condition_variable` 을 대체했다. producer 는 device state 콜백(rt_callback, FIFO 70), consumer 는 rt_control (FIFO 90) — 두 SCHED_FIFO 스레드가 mutex 를 공유하면 RT-4 위반이므로 wait-free write + fd wait 로 바꿨다. eventfd 는 counter 라 wake edge 유실이 없고, non-semaphore read 가 counter 를 0 으로 비우므로 state 버스트는 tick 을 여분으로 만들지 않고 하나로 합쳐진다

### SeqLock 기반 디바이스 상태 공유 (backend 내부)

디바이스 상태는 `DeviceBackend` 구현체가 자체 보유한 `SeqLock<DeviceStateCache>`로 sensor callback (writer) ↔ RT loop (reader) 락-프리 공유:

```cpp
// Backend 내부 — sensor callback (cb_group_rt_callback_ MutuallyExclusive — 단일 writer)
auto ds = state_cache_.Load();
ds.positions = ...;            // 필드 수정 (wire-format → device-config 순서로 reorder 포함)
state_cache_.Store(ds);
NotifyStateReady();            // CM 등록 callback 호출 (watchdog/digital-twin/sim-sync)

// CM RT loop (reader — 락-프리, 항상 최신 consistent snapshot)
DeviceStateCache cache{};
if (backends_[slot]) {
  backends_[slot]->ReadState(cache);
  if (backends_[slot]->HasMotorState()) backends_[slot]->ReadMotorState(cache);
  if (backends_[slot]->HasSensorState()) backends_[slot]->ReadSensorState(cache);
}
```

> `DeviceStateCache`는 trivially copyable (~4.3 KB). SeqLock writer는 wait-free (2회 atomic store + memcpy), reader는 writer 완료 시까지 spin-retry (writer ~1-2 µs). CM은 SeqLock을 직접 보유하지 않고 backend의 ReadState API를 통해 접근.

### eventfd 기반 Non-RT Publish Thread Wakeup

Layout v4: actuator publish 는 RT loop 안에서 inline 호출되므로 별도 wakeup 메커니즘이 없다. controller-owned non-RT publish 만 SPSC + eventfd 를 통해 비동기 drain:

```cpp
// RT loop (after nrt_publish_buffer_.Push)
eventfd_write(nrt_publish_eventfd_, 1);

// nrt_publish_thread (대기)
poll(&pfd, 1, 1);  // 1ms timeout, eventfd readable → 즉시 wakeup
```

> `sched_yield()` 대비 CPU 사용량 감소 + 즉시 wakeup으로 non-RT publish 지연 최소화.

---

## 타이밍 프로파일러 (`controller_timing_profiler.hpp`)

컨트롤러 `Compute()` 호출의 락-프리 타이밍 통계를 수집합니다. `TimingProfilerBase<200, 10, 2000>`을 상속합니다.

| 항목 | 설명 |
|------|------|
| 히스토그램 | 200개 버킷 (10 us 간격, 0-2000 us) + 오버플로 버킷 |
| 수집 통계 (`GetStats()`) | count, min, max, mean, stddev, p95, p99, over_budget — `cm_timing_log.csv`로도 복원 가능 |
| 예산 | `1e6 / control_rate` µs (예: 500 Hz → 2000 µs) — sim 모드 `elapsed` 계산에 사용 |
| `elapsed` 의미 | **robot**: 직전 print와의 wall-clock delta (CM 측정 실제 시간). **sim** (`use_sim_time_sync=true`): `count × period` (컨트롤러가 sim step과 lock-step이라 dt 기준 가상 시간이 컨트롤러 관점의 진실). 첫 print는 fallback으로 sim과 동일 식 |
| 리셋 주기 | RT 루프가 1 000 tick(≈ 2 s @ 500 Hz)마다 로그 스레드에 Summary 출력 요청 → 로그 스레드는 `RCLCPP_INFO` 직후 `timing_profiler_.Reset()`을 호출. 따라서 각 출력은 **직전 윈도우**(elapsed로 표시)의 mean/max이며, 세션 시작 시점의 스파이크가 영구 반영되지 않음 |
| Console summary 형식 | `<ctrl> timing: elapsed=Xs  mean=Yµs  max=Zµs  overruns=N  skips=N  nrt_pub_drops=N  timing_drops=N` — 의도적으로 슬림. 상세 percentile / over_budget / per-tick 값은 `cm_timing_log.csv`에서 사후 분석 |

누적 over-run 카운터(`overruns`, `skips`, `nrt_pub_drops`, `timing_drops`)는 `rt_controller_node`가 별도 원자 변수로 관리 — Summary 리셋에 영향 없음.

### Per-thread Timing CSV (generic infra)

CM RT loop와 MPC thread 모두 두 가지 base 인프라를 공유한다: (1) [`rtc_base/threading/periodic_rt_thread.hpp`](../rtc_base/include/rtc_base/threading/periodic_rt_thread.hpp)의 `PeriodicRtThread` — 고정 주파수 jthread + `clock_nanosleep(TIMER_ABSTIME)` 스케줄 + overrun 검출 + Pause/Resume 골격, (2) [`rtc_base/timing/thread_timing_*`](../rtc_base/include/rtc_base/timing/) — `ThreadTimingProducer<Payload, N>` + `ThreadTimingCsvLogger<Payload>`. CM은 `RtControllerNode::ControlLoopThread` (composition), MPC는 `MPCThread` (inheritance)로 base를 사용. payload는 5-컬럼 `RtTickTimingPayload` 단일 타입이며 base가 t0~t3을 자동 capture해 SPSC ring에 push:

| 채널 | Producer 멤버 | Logger | CSV 경로 | Schema (payload 컬럼) |
|------|--------------|--------|----------|----------------------|
| CM RT loop (@ `control_rate`) | `cm_timing_producer_` (`ThreadTimingProducer<RtTickTimingPayload, 512>`) | `cm_timing_logger_` | `<session>/timing/cm_timing_log.csv` | `t_state_us, t_compute_us, t_publish_us, t_total_us, jitter_us` |
| MPC thread (≤ 100 Hz, per-controller) | `MPCThread::TimingProducer()` (`ThreadTimingProducer<RtTickTimingPayload, 128>`) | `MpcTimingLogger` (controller-owned) | `<session>/timing/mpc_timing_log.csv` | `t_state_us, t_compute_us, t_publish_us, t_total_us, jitter_us` |

공통 컬럼 `t_wall_ns, tick_count`는 `ThreadTimingCsvLogger`가 자동으로 emit. `tick_count`는 producer-side monotonic 시퀀스 번호 (drop 검증용). CM/MPC 두 thread가 동일한 5-컬럼 payload schema (`rtc_base/timing/rt_tick_timing_sample.hpp`)를 공유하므로 cross-thread 분석 도구 한 세트가 두 CSV를 모두 처리한다.

CM은 `StartRtLoop`에서 `rt_loop_.SetTimingProducer<>` 한 줄로 base에 ring을 위임 — base가 매 tick payload를 push한다. `ControlLoop()` 내부의 t1/t2 마킹은 `rt_loop_.StampStateAcquired()` / `StampComputeDone()` 두 줄. `DrainLog()`가 `cm_timing_producer_.Drain(...)`로 CSV에 기록. 윈도우 INFO summary에 `timing_drops` (producer overflow 카운터)가 `pub_drops`와 함께 출력된다.

Sim 모드 (`use_sim_time_sync=true`) 에서는 `ControlLoopThread::JitterMeaningful()` 가 `false` 를 반환하여 base 가 `jitter_us` 를 0.0 으로 둔다 — wakeup 이 `sim_wake_eventfd_` (= MuJoCo step 완료) 이라 `|actual_period − 2 ms|` 는 sim cadence 잡음일 뿐 RT 지표가 아니다. 다른 6개 컬럼 (`t_state_us` / `t_compute_us` / `t_publish_us` / `t_total_us` / `t_wall_ns` / `tick_count`) 은 robot/sim 동일 의미 — body code path 가 wakeup 매커니즘과 무관하게 동일하기 때문. MPC / hand_udp producer 는 default `JitterMeaningful()=true` 유지.

MPC 측은 컨트롤러가 자체 1 Hz aux 타이머에서 `mpc_thread_->TimingProducer().Drain(...)`을 호출. 컨트롤러별 `<config_key>` 서브디렉토리에 쓰므로 전환이 발생해도 stream이 섞이지 않는다. 같은 콜백이 `MPCSolutionManager::GetSolveStats()` (handler self-report `solve_duration_ns`의 256-sample sliding 윈도우)로 10초마다 aggregate INFO 라인을 출력 — 디스크에는 기록하지 않으며 percentile은 raw CSV에서 post-process로 계산.

새 RT/soft-RT thread (예: ONNX inference)를 추가하려면 `PeriodicRtThread` 상속/composition + `OnTick` body 정의면 끝. `RTControllerInterface` 변경 없음.

---

## ROS2 인터페이스

### 고정 구독

CM 자체는 고정 게인 토픽을 더 이상 구독하지 않는다 (게인 채널이 컨트롤러 LifecycleNode parameter로 이관됨, 위 §게인 채널 참조). 고정 구독은 `rclcpp_lifecycle::LifecycleNode` 자체 lifecycle 서비스(`change_state`, `get_state` 등)만 남는다.

### 고정 서비스

| 서비스 | 타입 | 콜백 그룹 | 설명 |
|--------|------|-----------|------|
| `/rtc_cm/switch_controller` | `rtc_msgs/srv/SwitchController` | aux | 활성 컨트롤러 전환 (sync, single-active, E-STOP guard) |
| `/rtc_cm/list_controllers` | `rtc_msgs/srv/ListControllers` | aux | 모든 컨트롤러 lifecycle state 조회 |
| `/rtc_cm/reset_fault` | `rtc_msgs/srv/ResetFault` | aux | latched **controller-local** fault 해제 (#260, sync, active 한정 + 이름 명시 필수) |

#### `/rtc_cm/reset_fault` (#260)

compliance 계열은 critical fault (`nan_inf` · `pose_error_exceeded` · `sigma_below_critical` · `command_divergence`) 에서 `SAFE_STOP` 을 래치하고 **자동 복귀하지 않는다** (§10.6). 이 서비스가 그 래치의 유일한 외부 탈출구이며, 이전에는 프로세스 재시작 외에 방법이 없었다.

- **권한** — `controller_name` **필수**이고 현재 **active 컨트롤러**와 일치해야 한다. 이름 명시가 오퍼레이터 확인 단계이므로 빈 요청은 편의 기본값이 아니라 거부이고, wildcard 는 없다. 비활성 컨트롤러 대상 요청은 큐잉하지 않고 거부한다 — 큐에 남은 요청은 그 컨트롤러가 다음에 활성화되는 첫 tick 에 소비돼 아무도 그 시점에 재승인하지 않은 fault 를 푼다 (`BeginBiasCalibration()` 이 이미 거부하는 세탁 경로).
- **응답** — "전달됨"이 아니라 **실제 결과**를 보고한다. 요청 후 `1.5 × dt` 대기하고 `HasLatchedFault()` 를 다시 읽어, 해제 / no-op(원래 래치 없음) / 재래치(원인 잔존)를 구분한다. `switch_controller` 가 RT tick 관측에 쓰는 것과 같은 대기다.
- **E-8 분리** — global E-STOP 과 **서로를 풀지 않는다**. global E-STOP 이 걸린 상태에서도 reset 은 성립하며 (RT 루프는 E-STOP 중에도 `Compute()` 를 계속 호출하고 출력만 치환한다), 그 경우 응답 message 가 global latch 가 아직 남아 있음을 명시한다.

### 고정 퍼블리셔

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `/system/estop_status` | `Bool` | RELIABLE/10 | 글로벌 E-STOP 상태 |
| `/{robot_ns}/active_controller_name` | `String` | TRANSIENT_LOCAL/1 | 활성 컨트롤러의 `config_key` (snake_case). 외부 rewire 소비자가 이 페이로드를 그대로 prefix 로 사용해 `/<config_key>/...` 형태로 컨트롤러-owned 토픽 namespace 를 조립하므로, payload 는 컨트롤러 LifecycleNode 의 namespace (`/<config_key>`) 와 1:1 일치해야 한다. `RTControllerInterface::Name()` (클래스 라벨) 가 아님 |

### DeviceBackend 토픽 (Phase 4 — `devices.<group>.backend:` SSoT)

CM은 device-wire 토픽을 직접 만들지 않습니다. `DeviceBackend` 구현체 (`mujoco_native` / `ur_driver_native` / `udp_hand_native`) 가 `devices.<group>.backend:` YAML 블록에서 type + topic 을 읽어 자기 LifecycleNode 에 sub/pub 을 등록합니다. 컨트롤러 YAML 의 `topics:` 섹션은 이 lane 들을 더 이상 갖지 않습니다.

| 키 | 메시지 타입 | QoS | 설명 |
|---|---|---|---|
| `state_topic` | `JointState` | BEST_EFFORT/2 | 디바이스 관절 상태 (필수) |
| `command_topic` | `JointCommand` 또는 `Float64MultiArray` | BEST_EFFORT/1 또는 RELIABLE/1 | 관절 커맨드 → HW/sim (필수, backend 별로 페이로드 형식 다름). `WriteCommand(slot, command_type)` 의 `command_type` 은 RT loop 가 그룹별로 resolve 한 값 (`kPosition`/`kTorque`/`kPdFeedforward`) — `JointCommand.command_type` 문자열 필드로 그대로 인코딩됨 |
| `motor_topic` | `JointState` | BEST_EFFORT/2 | 모터 공간 상태 (선택, `udp_hand_native` 전용) |
| `sensor_topic` | `HandSensorState` | BEST_EFFORT/2 | 촉각 센서 상태 (선택, `udp_hand_native` 전용) |

**명령 순서 규약** (`DeviceBackendConfig::joint_command_names` doc 참조): `WriteCommand` 에 전달되는 `slot.commands` 와 발행 메시지는 **항상 `joint_command_names` 순서** — backend 는 direct-copy 하며 command-side reorder 를 하지 않습니다. wire 순서가 다른 디바이스는 **수신자** (udp_hand_node / mujoco_simulator_node) 가 메시지의 `joint_names` 로 재정렬해 흡수합니다. state-side 는 반대로 backend 가 수신자로서 첫 named 메시지에서 `msg->name` → `joint_command_names` reorder map 을 lazy build 합니다.

### CM 동적 구독 (컨트롤러 TopicConfig 기반)

| 역할 | 메시지 타입 | QoS | 설명 |
|------|------------|-----|------|
| `kTarget` | `RobotTarget` | RELIABLE/10 | 관절/태스크 목표 |

### 컨트롤러-owned 퍼블리셔 (LifecycleNode 자체 보유)

Publish 역할은 모두 **controller-owned** 입니다. (Phase 4: `kJointCommand` / `kRos2Command` 는 DeviceBackend 로 이동.) 컨트롤러 LifecycleNode가 `on_configure`에서 직접 퍼블리셔를 생성합니다. YAML role-mapped 채널은 `kRobotTransforms` 하나뿐입니다 — issue #196 Phase 5 에서 `kRobotTarget` / `kDigitalTwinState` 를 제거했습니다 (파서는 매핑했으나 publisher 를 만드는 소비자가 없어, 선언하면 조용히 죽은 토픽이 됐습니다). `PublishNonRtSnapshot()` 안에서 발행합니다 (CM은 SPSC snapshot 운반만 담당). controller YAML `topics:` entry 는 전부 controller-owned 이므로 CM은 이들에 대해 퍼블리셔를 만들지 않습니다 (issue #138: `ownership:` field 및 manager fallback 제거). Phase 4 에서 `kGuiPosition` 은 폐기되었고, GUI/외부 도구는 ctrl-agnostic `/rtc_cm/<group>/joint_states` (`sensor_msgs/JointState`) + active controller's `<config_key>/transforms` (`tf2_msgs/TFMessage`) 두 표준 토픽 조합으로 동일 정보를 얻습니다.

`GraspState` / `WbcState` / `ToFSnapshot` 은 더 이상 `PublishRole` enum / YAML role 매핑을 거치지 않습니다. 각 컨트롤러가 `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼 (integrated_bringup/support/owned_topics.hpp) 로 자체 LifecyclePublisher 를 만들고, RT compute 가 결과를 컨트롤러 멤버 `rtc::SeqLock<T>` (`grasp_state_lock_` / `wbc_state_lock_` / `tof_snapshot_lock_`) 에 Store 합니다. Publish thread 는 `PublishNonRtSnapshot()` 안에서 SeqLock 을 Load 하고 `PublishOwnedTopicsFromSnapshot` 에 `&loaded` 를 넘겨 발행합니다. CM 은 의미를 모르며 `PublishSnapshot` 도 이 세 payload 를 운반하지 않습니다 — single-producer / single-consumer 데이터에 적합한 wait-free SeqLock 핸드오프입니다.

`kRobotTransforms` (Phase 2) 는 controller당 1개 토픽 (`<config_key>/transforms`, `tf2_msgs/TFMessage`, RELIABLE/10) 으로 controller가 사용하는 frame 들을 묶어 발행합니다. RT loop는 `ControllerOutput::{arm_tip,virtual_tcp,fingertip}_pose*` (3D pose + valid flag) 를 매 tick 채우고, CM이 이를 모든 group slot에 복사합니다. Publish thread는 owner controller의 `ControllerTopicHandles::tf_slots[]` 정의 (on_configure 시 system YAML `urdf.{sub,tree}_models` 로 자동 빌드) 에 따라 valid한 slot만 TFMessage로 직렬화합니다. Active controller만 LifecyclePublisher가 활성화되므로 같은 frame_id를 다른 controller가 broadcast해도 cutover 시 정확히 1개만 발행됩니다 (D-4).

### Per-group JointState 자동 퍼블리셔

각 디바이스 그룹에 대해 `/rtc_cm/{group}/joint_states` (RELIABLE/1) JointState를 자동 생성합니다. 설정 순서(joint_state_names)로 재정렬된 관절 데이터를 republish합니다. 이 토픽은 active controller와 무관하게 항상 발행되는 측정 상태의 단일 소스이며, `rtc_digital_twin` 같은 외부 노드가 이를 merge해 `robot_state_publisher` / RViz로 공급합니다.

**어느 스레드가 발행하는가 (issue #198 Phase 2).** backend state-ready 콜백은 `cb_group_rt_callback_` (SCHED_FIFO 70, controller↔hardware RT 경계) 에서 돌므로 **mailbox 만** 수행한다 — slot 별 dirty bit 하나를 세우고 `nrt_publish_eventfd_` 에 write 한다. 실제 `ReadState` + RELIABLE publish 는 `nrt_publish_thread` 의 `DrainDigitalTwin()` 이 한다 (SCHED_OTHER nice 0). 상태 payload 는 backend 자신의 `SeqLock` 에 이미 들어 있으므로 큐가 필요 없다 — dirty bit 는 큐가 아니라 비트이며, 드레인보다 빨리 도착한 메시지는 **합쳐진다**. 즉 twin 이 밀리면 샘플 수가 줄 뿐 stale 한 값을 보이지는 않는다.

`on_configure` 중(백엔드 sub 은 이미 활성, 그러나 노드는 아직 Inactive)에 상류가 상태를 흘리면 콜백이 조기 발화할 수 있다 — 이때는 드레인이 LifecyclePublisher `is_activated()` 로 가드해 노드 `on_activate` 이전 발행을 건너뛴다("publisher is not activated" 경고 방지). 비트는 그 경우에도 소비되므로 드레인이 발행 불가 slot 에서 spin 하지 않는다.

---

## 파라미터 레퍼런스

코드에서 `DeclareAndLoadParameters()`로 선언되는 모든 파라미터입니다.

| 파라미터 | 타입 | 코드 기본값 | 설명 |
|---------|------|------------|------|
| `control_rate` | double | `500.0` | 제어 주파수 (Hz) |
| `initial_controller` | string | `""` | 시작 컨트롤러 (이름 또는 config_key). 빈 값 = robot bringup yaml이 반드시 지정 |
| `init_timeout_sec` | double | `5.0` | 초기화 타임아웃 (초). state 미수신 시 E-STOP |
| `enable_estop` | bool | `true` | E-STOP 워치독 활성화 |
| `device_timeout_names` | string[] | `[]` | 그룹별 timeout 값을 지정할 대상 이름 (감시 대상 자체는 설정된 모든 디바이스 그룹) |
| `device_timeout_values` | double[] | `[]` | 각 그룹의 state 토픽 타임아웃 (ms) |
| `device_timeout_default_ms` | double | `1000.0` | `device_timeout_names` 에 없는 그룹에 적용되는 timeout (ms) |
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
| `urdf.extended` | bool | 선택 | `true` 면 sidecar `<stem>.closure.yaml` 로드 (loop closure, 기본 `false`) |
| `urdf.closure_path` | string | 선택 | sidecar 명시 경로 (패키지 상대). 미지정 시 URDF 경로에서 유도 |
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
| `sensor_layout.primary_count_per_group` | int | 선택 | 센서 그룹당 primary block 채널 수 (예: assm_v1 hand 의 fingertip 당 barometer 8). CM 은 stride 계산에만 사용 — 의미는 device-driver 패키지 책임. |
| `sensor_layout.secondary_count_per_group` | int | 선택 | 센서 그룹당 secondary block 채널 수 (예: ToF 3) |
| `sensor_layout.values_per_group` | int | 선택 | 그룹당 총 채널 수 (= primary + secondary 가 디폴트). 명시 시 stride 로 사용. |
| `sensor_layout.inference_values_per_group` | int | 선택 | ML 추론 출력 그룹당 값 수 (예: contact(1)+F(3)+u(3) = 7) |
| `sensor_layout.has_native_contact` | bool | 선택 (default false) | Backend 가 inference slot 0 에 native contact 신호 (e.g. ONNX sigmoid prob) 를 채우는지 advertise. 컨트롤러가 native vs derived contact decision 분기에 사용 (예: udp_hand+inferencer.enabled=true → true, mujoco wrench → false). CM 은 의미를 모르고 전달만. |
| `sensor_layout.has_native_displacement` | bool | 선택 (default false) | Backend 가 inference slots 4..6 에 native displacement 를 채우는지 advertise. 위와 동일 패턴. |

> URDF가 설정된 디바이스의 경우, YAML joint_limits와 URDF limits를 병합하여 더 보수적인 값을 적용합니다. YAML에 joint_limits가 없으면 URDF 값만 사용합니다.

> `sensor_layout` 은 packed-sensor 토픽 (`HandSensorState` 등) 을 받는 디바이스 한정. CM 은 layout 의 의미를 모르고 stride/offset 계산만 수행 — robot/sensor-agnostic 원칙 유지. 미설정 디바이스의 packed-sensor 콜백은 throttled WARN 후 메시지 drop. capability bool (`has_native_*`) 은 stride 계산에는 무관하고 컨트롤러 행동 분기에만 영향.

---

## 설정 파일

### YAML 소유권 (ARCH-1)

`rtc_controller_manager` 는 **default YAML을 동봉하지 않습니다**. 모든 파라미터는 `DeclareAndLoadParameters()` 가 robot-agnostic 코드 기본값(예: `initial_controller=""`, `device_timeout_names=[]`, `urdf` / `devices` 미선언)으로 declare 합니다. 실제 production 값은 `<robot>_bringup` 패키지의 YAML이 단독 소유 — 예: `integrated_bringup/config/{ur5e_p1a,iiwa7_leap}/sim.yaml`. 새 robot 도입 시 자체 YAML을 만들어 launch에서 주입하세요. `config_variant` ROS 파라미터(예: `"ur5e_p1a"`, `"iiwa7_leap"`)가 `config/<variant>/controllers/...` 경로 조회를 결정합니다.

UR5e bringup의 예시 YAML 구조 (`integrated_bringup/config/ur5e_p1a/sim.yaml`):

```yaml
/**:
  ros__parameters:
    control_rate: 500.0
    initial_controller: "demo_wbc_controller"
    init_timeout_sec: 0.0                 # sim — disable hardware init timeout

    enable_estop: true
    device_timeout_names: ["ur5e", "hand"]
    device_timeout_values: [100.0, 100.0] # ms

    # System URDF (shared by all controllers) — robot-specific
    urdf:
      package: "robot_descriptions"
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
        safe_position: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        urdf:
          root_link: "base_link"
          tip_link: "wrist_3_link"
        joint_limits:
          max_velocity:     [2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
          # ...
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
| `std_msgs` | Bool (E-STOP), String (active controller name) — Float64MultiArray bridge moved into `ur_driver_native` DeviceBackend (Phase 4) |
| `rtc_controller_interface` | 컨트롤러 추상 인터페이스 + 레지스트리 |
| `rtc_controllers` | 내장 컨트롤러 (팩토리 등록) |
| `rtc_base` | 로깅, 스레딩, 타입, SPSC 버퍼, 세션 디렉토리 |
| `rtc_communication` | 네트워크 통신 (UDP 트랜시버) |
| `rtc_msgs` | JointCommand (HW/sim 경계 publish), RobotTarget + HandSensorState (boundary subscribe), ListControllers/SwitchController srv |
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
- 실행 파일 없음 — robot-specific bringup 패키지(예: `integrated_bringup`의 `integrated_rt_controller`)가 `rtc::RtControllerMain(argc, argv, node_name)`을 호출하여 자체 exec를 만든다. `node_name`은 robot bringup의 executable 이름과 일치시킨다 (exec ↔ ROS 노드 ↔ 로거 식별자 정렬). 자세한 원칙은 [agent_docs/design-principles.md](../agent_docs/design-principles.md) 참고.

---

## 의존성 그래프 내 위치

```
rtc_base + rtc_controller_interface + rtc_controllers + rtc_communication
    |
rtc_controller_manager  <- RT 제어 실행 엔진 (@ control_rate)
    ^
    +-- integrated_bringup  (로봇별 진입점 + launch 파일)
```

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
