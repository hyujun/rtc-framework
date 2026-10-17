# rtc_controller_manager

설정 가능한 RT 제어 루프 · 컨트롤러 lifecycle · 글로벌 E-STOP · DeviceBackend 를 소유하는 라이브러리. **유형**: 노드 패키지 (라이브러리 형태)

## 개요

`RtControllerNode` (`rclcpp_lifecycle::LifecycleNode`) 가 `control_rate` YAML 주기로 `clock_nanosleep` 제어 루프를 돌리고, 등록된 컨트롤러를 수명 관리하며, 디바이스 워치독과 글로벌 E-STOP 을 맡는다. 범위·default 는 [invariants.md](../agent_docs/invariants.md) §RT Path Invariants 가 SSoT 다. 퍼블리시·CSV 로깅은 락-프리 SPSC 로 non-RT 스레드에 넘긴다.

**이 패키지에는 실행 파일이 없다.** 정적 라이브러리 `librtc_controller_manager_lib.a` 를 export 하고, 로봇별 bringup 패키지가 `rtc::RtControllerMain(argc, argv, node_name)` 을 호출해 자기 실행 파일을 만든다. 이 workspace 의 실행 파일은 `integrated_bringup` 의 `integrated_rt_controller` 이며 ROS 노드 이름도 같다 (`/integrated_rt_controller`). 로봇 데이터 (`control_rate` · `devices` · `urdf`) 와 launch 는 [integrated_bringup](../integrated_bringup/README.md) 이 소유한다.

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 실행은 bringup 의 launch 로 한다.

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py
```

다른 터미널에서 컨트롤러 목록과 전환:

```bash
ros2 service call /rtc_cm/list_controllers rtc_msgs/srv/ListControllers "{}"
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['demo_joint_controller'], deactivate_controllers: [], strictness: 1, timeout: {sec: 1}}"
```

컨트롤러 이름은 `Name()` 또는 `config_key` 둘 다 쓸 수 있다. 응답 `ok=false` 이면 `message` 가 사유를 말한다.

## 인터페이스

### 라이브러리 헤더

| 헤더 | 내용 |
|---|---|
| `rt_controller_node.hpp` | `RtControllerNode` — 노드 본체 |
| `rt_controller_main.hpp` | `RtControllerMain()` — bringup 이 부르는 진입점 |
| `device_backend.hpp` | `DeviceBackend` 추상 인터페이스 · `DeviceBackendConfig` |
| `device_backend_registry.hpp` | type_tag 레지스트리 · `RTC_REGISTER_DEVICE_BACKEND` |
| `device_state_cache.hpp` | `DeviceStateCache` POD (backend SeqLock payload) |
| `controller_config_loader.hpp` | `rtc::LoadControllerConfig` — 컨트롤러 YAML + `include:` 합성 |
| `controller_timing_profiler.hpp` | 락-프리 `Compute()` 타이밍 통계 |

### 서비스 (`/rtc_cm/...`)

| 서비스 | 타입 | 설명 |
|---|---|---|
| `/rtc_cm/switch_controller` | `rtc_msgs/srv/SwitchController` | 활성 컨트롤러 전환 (sync, single-active). E-STOP 중에는 거부 |
| `/rtc_cm/list_controllers` | `rtc_msgs/srv/ListControllers` | 모든 컨트롤러의 state 와 진단 4필드 |
| `/rtc_cm/reset_fault` | `rtc_msgs/srv/ResetFault` | 래치된 **컨트롤러-local** fault 해제 (active 컨트롤러 이름 명시 필수) |
| `/rtc_cm/clear_estop` | `rtc_msgs/srv/ClearEstop` | **글로벌** E-STOP 래치 해제 (`reason_ack` 필수) |

컨트롤러를 따로 load 하는 서비스는 없다. 등록된 컨트롤러는 시작 때 전부 인스턴스화되고 `switch_controller` 가 활성만 고른다. 요청·응답 필드는 각 `.srv` 주석이 SSoT 다. 서비스는 RT 경로와 분리된 `cb_group_nrt_callback_` 에서 처리된다.

#### `/rtc_cm/list_controllers` 진단 필드

응답의 각 `ControllerState` 는 lifecycle 메타데이터 (`name` · `state` · `type` · `is_active` · `claimed_groups`) 외에 진단 4필드를 싣는다. 넷 다 오퍼레이터에게는 "goal 을 보냈는데 안 움직인다" 로만 보이므로 이 필드로 구분한다.

| 필드 | 의미 |
|---|---|
| `has_latched_fault` | controller-local fault 래치가 올라가 있다 → `/rtc_cm/reset_fault` 로 해제 |
| `target_drop_count` | 유효한 goal 이 큐 포화로 유실 — producer 의 **rate** 문제 |
| `target_reject_count` | 형식 불량 · `device_idx` 범위 밖으로 ingress 에서 거부 — producer 의 **payload** 문제 |
| `target_unhandled_count` | 컨트롤러가 mailbox 에 push 하고 `ApplyPendingTarget` 을 override 하지 않았다 — 컨트롤러 **배선 결함** |

- 컨트롤러별 override 는 필요 없다. CM 이 `RTControllerInterface` 의 접근자를 직접 읽는다.
- 카운터 3종은 monotonic 이고 조회가 소비하지 않는다. 진단은 두 번의 폴링 사이에 값이 **움직이는지**로 한다.
- 응답은 스냅샷이다. 한 응답 안의 두 카운터는 서로 다른 tick 에서 왔을 수 있다.
- `has_latched_fault` 는 비활성 컨트롤러도 보고한다. `reset_fault` 는 active 만 받으므로 어느 컨트롤러가 잠겼는지 먼저 알아야 이름을 댈 수 있다.

#### `/rtc_cm/reset_fault`

```bash
ros2 service call /rtc_cm/reset_fault rtc_msgs/srv/ResetFault "{controller_name: 'demo_compliance_controller'}"
```

compliance 계열은 critical fault 에서 `SAFE_STOP` 을 래치하고 자동 복귀하지 않는다. 이 서비스가 그 래치의 유일한 외부 탈출구다. 응답은 "전달됨" 이 아니라 **실제 결과**다: 해제 · no-op (래치 없음) · 재래치 (원인 잔존) · tick 미관측 (RT 루프 정지 · overrun) 을 `message` 로 구분한다. 글로벌 E-STOP 은 풀지 않는다 (아래).

### 토픽

| 토픽 | 타입 | QoS | 설명 |
|---|---|---|---|
| `/system/estop_status` (pub) | `std_msgs/Bool` | RELIABLE/1, transient_local | 글로벌 E-STOP 상태 (래치 또는 해제 검증 창) |
| `/rtc_cm/active_controller_name` (pub) | `std_msgs/String` | TRANSIENT_LOCAL/1 | 활성 컨트롤러의 `config_key`. 외부 소비자가 `/<config_key>/...` 를 조립하는 prefix 다 (`Name()` 아님) |
| `/rtc_cm/<group>/joint_states` (pub) | `sensor_msgs/JointState` | RELIABLE/1 | 그룹별 측정 상태 — 아래 §Per-group JointState 자동 퍼블리셔 |
| `<state_topic>` (sub) | `JointState` | BEST_EFFORT/2 | `devices.<group>.backend.state_topic` — 디바이스 관절 상태 (필수) |
| `<command_topic>` (pub) | `JointCommand` · `Float64MultiArray` | BEST_EFFORT/1 · RELIABLE/1 | `backend.command_topic` — backend 별 페이로드 |
| `<motor_topic>` · `<sensor_topic>` (sub) | `JointState` · `HandSensorState` | BEST_EFFORT/2 | 선택. `udp_hand_native` 전용 |

CM 은 device-wire 토픽을 직접 만들지 않는다. `DeviceBackend` 구현체 (`mujoco_native` · `ur_driver_native` · `udp_hand_native`) 가 `devices.<group>.backend:` 블록의 type 과 topic 으로 자기 sub/pub 을 만든다. 토픽 소유는 세 갈래다:

| 소유 | 생성 주체 | 예 |
|---|---|---|
| DeviceBackend | CM 이 그룹마다 1 instance 보유 | `state` · `joint_command` 등 HW/sim 경계 트래픽 |
| CM 고정 | `RtControllerNode` (YAML 무관) | 위 표의 `/system/estop_status` · `/rtc_cm/*` |
| controller-owned | 컨트롤러별 `LifecycleNode` (YAML `topics:` 전부) | `target` · `grasp_state` · `wbc_state` · `tof_snapshot` · `<config_key>/transforms` |

controller YAML 에는 `ownership:` 필드가 없고 `topics:` entry 는 전부 controller-owned 다. CM 은 controller sub/pub 을 만들지 않고 SPSC snapshot 만 운반한다 (유효한 role 문자열은 [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §퍼블리시 역할). 각 컨트롤러는 `/<config_key>` 네임스페이스의 자기 노드에서 gain 파라미터를 노출한다 ([agent_docs/controllers.md](../agent_docs/controllers.md) §Gains).

**명령 순서 규약**: `WriteCommand` 의 `slot.commands` 와 발행 메시지는 항상 `joint_command_names` 순서이며 backend 는 command-side reorder 를 하지 않는다. wire 순서가 다른 디바이스는 수신자가 메시지의 `joint_names` 로 재정렬한다. state-side 는 반대로 backend 가 첫 named 메시지에서 reorder map 을 만든다.

#### Per-group JointState 자동 퍼블리셔

각 디바이스 그룹마다 `/rtc_cm/{group}/joint_states` (RELIABLE/1) 를 자동 발행한다. `joint_state_names` 순서로 재정렬한 측정 상태이며 active controller 와 무관하게 항상 나가는 단일 소스다. `rtc_digital_twin` 이 이를 merge 해 `robot_state_publisher` / RViz 로 공급한다.

발행은 `nrt_publish` 스레드 (SCHED_OTHER) 가 한다. RT 쪽 backend state-ready 콜백은 slot 별 dirty bit 와 eventfd write 만 하므로, twin 이 밀리면 샘플이 합쳐져 줄 뿐 stale 값을 보이지 않는다. `on_configure` 중 노드가 아직 Inactive 일 때 오는 상태는 LifecyclePublisher `is_activated()` 가드로 건너뛴다.

### 스레드와 lifecycle

`SelectThreadConfigs()` 가 물리 코어 수에 따라 레이아웃을 고른다. 값의 SSoT 는 [thread_layout.yaml](../repo_scripts/config/thread_layout.yaml), tier 별 코어 배치는 [rtc_base/README.md](../rtc_base/README.md) 의 생성 매트릭스다. 표는 `ps -L` 판독용이다.

| 스레드 | 스케줄러 | 주파수 | 역할 |
|---|---|---|---|
| **rt_loop** (rt_control) | SCHED_FIFO 90 | `control_rate` Hz (+ 워치독 50 Hz) | 제어 루프. tick 끝에 `DeviceBackend.WriteCommand` 를 inline 호출하고 `nrt_publish_buffer_` 에 push |
| **rt_callback_executor** | SCHED_FIFO 70 | 이벤트 | 디바이스 JointState · MotorState · SensorState 구독 (`cb_group_rt_callback_`) |
| **nrt_logging_executor** | SCHED_OTHER -5 | 100 Hz | 타이밍 CSV 드레인 · 1초 서머리 · deferred E-STOP 로그 |
| **nrt_publish** | SCHED_OTHER 0 | 이벤트 | SPSC 드레인 → `PublishNonRtSnapshot` + per-group JointState. eventfd wakeup. `nrt_callback` 과 배치가 같고 이름만 다르다 |
| **nrt_callback_executor** | SCHED_OTHER 0 | 이벤트 | 컨트롤러 전환 · E-STOP 상태 · lifecycle 서비스 · 컨트롤러 LifecycleNode 의 구독 |

`mlockall(MCL_CURRENT | MCL_FUTURE)` 을 `rclcpp::init()` 전에 호출한다. 프로세스 시작은 `RtControllerMain()` 이 순서대로 처리한다: 노드 생성 (Unconfigured) → launch 가 configure/activate → Active 확인 → 전용 executor 로 전환.

| 콜백 | 하는 일 |
|---|---|
| `on_configure` | 콜백 그룹 · 파라미터 · 컨트롤러 · 퍼블리셔/구독 · 타이머 · eventfd 생성 |
| `on_activate` | RT 루프와 publish 스레드 시작 |
| `on_deactivate` | 스레드 중지, E-STOP 클리어, 상태 초기화 |
| `on_cleanup` | `on_configure` 역순 해제 (eventfd 만 backend 해제 **후** close) |
| `on_error` | E-STOP 트리거 후 정리 → Unconfigured 복구 |

```bash
ros2 lifecycle get /integrated_rt_controller
ros2 lifecycle set /integrated_rt_controller deactivate   # RT 루프 중지
ros2 lifecycle set /integrated_rt_controller activate     # 재시작
```

`/system/estop_status` 와 `/rtc_cm/active_controller_name` 퍼블리셔는 lifecycle 상태와 무관하게 동작한다.

### 제어 루프

한 tick (`control_rate` 주기; 500 Hz 면 2 ms) 의 흐름이다.

1. **준비 게이트** — 설정된 **모든** 디바이스 그룹이 state 를 한 번 이상 보고해야 한다. 미보고면 `Compute()` / `WriteCommand()` 없이 tick 을 반환하고, `init_timeout_sec` 이 지나면 `{group}_init_timeout` E-STOP 후 노드가 종료한다.
2. **상태 획득** — 활성 컨트롤러의 `TopicConfig.groups` 순서로 각 backend 의 SeqLock cache 를 락-프리로 읽는다.
3. **연산** — 활성 컨트롤러의 `Compute()` → `ControllerOutput`. CM 은 hold 타깃을 만들지 않고 컨트롤러가 첫 `Compute()` 에서 현재 state 로 자체 seed 한다.
4. **퍼블리시** — actuator 는 RT tick 안에서 그룹별로 `WriteCommand` inline 호출, controller-owned 토픽은 SPSC (cap 16) 로 `nrt_publish` 에 넘긴다.
5. **타이밍** — 위상별 시간 · 지터를 SPSC 로 로깅 스레드에 보낸다.

**워치독 (50 Hz)** 은 설정된 모든 그룹을 감시한다. `device_timeout_names` / `device_timeout_values` 는 그룹별 timeout **값**만 주고, 목록에 없는 그룹은 `device_timeout_default_ms` 로 감시된다. 초과하면 `{group}_timeout` E-STOP 이다. 한 번도 보고 않은 그룹은 워치독이 아니라 준비 게이트 몫이다. 오버런한 tick 은 건너뛰고 주기를 재정렬하며 (burst 없음) 연속 10회면 E-STOP 이다.

**Per-group command type**: `DeviceOutput::command_type` (per-device, nullopt = 상속) 이 있으면 `ControllerOutput::command_type` 을 override 한다. Mixed-command 컨트롤러 (예: WBC — arm `kPosition`, hand `kPdFeedforward`) 가 그룹마다 다른 타입을 낼 수 있는 이유다. `kPdFeedforward` 는 position target 에 per-joint feedforward torque 를 얹으며 wire 에는 `"pd_feedforward"` 로 인코딩된다. feedforward 채널을 모르는 backend 는 position tracking 으로 폴백한다.

**Sim 모드** (`use_sim_time_sync=true`): tick 을 시계가 아니라 sim step 완료로 깨운다. step 이 `sim_sync_timeout_sec` 안에 완성되지 않으면 `sim_sync_timeout` E-STOP 후 노드가 종료한다. tick 은 `sim_sync_tick_devices` 의 모든 그룹이 새 state 를 낸 뒤에만 한 번 돈다.

### 컨트롤러 관리

#### 시스템 URDF + ModelConfig

최상위 `urdf:` 섹션 (파라미터 표는 §설정) 의 경로와 `sub_models` / `tree_models` 로 시스템 `PinocchioModelBuilder` 를 한 번 빌드해 컨트롤러에 공유한다. 빌드가 실패하면 컨트롤러가 자기 builder 를 만든다.

- `sub_models` / `tree_models` 의 map 키(모델명)는 `devices` 의 그룹 이름과 매칭되어, 디바이스별 `root_link` / `tip_link` 를 자동 해석한다. 디바이스별 URDF 경로가 없으면 시스템 URDF 를 쓴다
- `tip_link` 가 URDF frame 이 아니면 이 노드는 WARN 만 찍고 이름을 컨트롤러에 넘긴다. 받을지는 컨트롤러가 정한다 — 팔 끝 frame 이 풀리지 않는 joint · task · compliance · wbc 컨트롤러는 `on_configure` 를 거부하고, 한 컨트롤러의 거부는 configure 전체를 실패시킨다
- `urdf.extended: true` 면 `<stem>.closure.yaml` sidecar 로 loop closure 를 만든다. sidecar `actuated_joints` 외의 movable 관절은 loop-passive 로 잠기므로 `actuated_joints` 는 그 URDF 의 완전한 active 집합이어야 한다 ([rtc_urdf_bridge/README.md](../rtc_urdf_bridge/README.md#extended-urdf-closed-chain-sidecar-nameclosureyaml)). sidecar 가 없으면 ERROR 후 plain URDF 로 진행한다
- 최상위 `urdf:` 가 없으면 `devices.{group}.urdf` 에서 읽는다 (`extended` / `closure_path` 는 최상위 값을 그대로 적용)

#### 로딩과 컨트롤러 생성

시작 때 `ControllerRegistry` 의 등록 컨트롤러를 모두 3-pass 로 만든다. Pass 1 은 팩토리 생성 · 전용 `LifecycleNode` (이름 = `config_key`, 네임스페이스 = `/<config_key>`) · `PreConfigure(yaml)`, Pass 2 는 device-name 해석, Pass 3 은 `on_configure` 다.

- `main()` 이 컨트롤러 노드를 `nrt_callback_executor` 에 attach 하므로 컨트롤러 소유 sub/pub 콜백은 non-RT 스레드에서 돈다
- 컨트롤러는 `Name()` 과 `config_key` 양쪽으로 한 네임스페이스 (`controller_name_to_idx_`) 에 등록되어, `switch_controller` / `initial_controller` 가 둘 중 아무 문자열로나 해석한다
- `initial_controller` 파라미터가 시작 컨트롤러를 고른다. Lifecycle 훅 규약은 [rtc_controller_interface/README.md](../rtc_controller_interface/README.md#lifecycle-훅-ros2_control-정렬-기본-구현-제공) 를 본다

##### 식별자 충돌 가드 (2-tier)

네임스페이스가 하나라서 식별자가 겹치면 오퍼레이터가 고르지 않은 컨트롤러가 조용히 돈다. CM 은 이를 둘 다 **경고가 아니라 configure 실패**로 거부한다.

| Tier | 시점 | 검사 대상 |
|---|---|---|
| 1 | Pass 1 **이전** | `config_key` 끼리 — 팩토리 호출 전이라 컨트롤러가 하나도 안 만들어진 채 거부 |
| 2 | Pass 1 안 | `Name()` 과 `config_key` 를 이미 등록된 모든 식별자와 대조 (`Name()` 은 인스턴스를 만든 뒤에야 알 수 있다) |

**한 컨트롤러 클래스를 두 `config_key` 로 등록할 수 없다.** `Name()` 은 클래스 상수라 같은 클래스의 두 번째 등록은 항상 첫 번째와 충돌한다. "같은 클래스, 두 YAML" 이 필요하면 인스턴스별로 `Name()` 이 다른 서브클래스를 만든다. 클래스를 복사하고 `Name()` 문자열을 안 고친 실수가 이 경로로 잡힌다.

#### 런타임 전환 (`/rtc_cm/switch_controller`)

요청의 `strictness` (STRICT / BEST_EFFORT) 로 activate/deactivate 목록을 단일 대상으로 정리한 뒤 sync 로 전환한다. 순서는 이렇다: 사전 조건 (E-STOP idle · single-active) → 이름/`config_key` 로 조회 (`Unknown` 이면 거부) → 대상 `on_activate` → 활성 index 를 atomic 으로 교체 → RT tick 1회 대기 → 이전 컨트롤러 `on_deactivate` → `/rtc_cm/active_controller_name` 발행 → `ok=true` 응답. 응답이 오면 새 컨트롤러가 이미 RT 루프에 보인다. 컨트롤러의 `on_activate` 는 base 가 no-op 이며 첫 `Compute()` tick 에서 현재 device state 로 자체 seed 한다.

#### 컨트롤러 YAML 이 없을 때 — 세 갈래

등록된 컨트롤러는 로봇과 무관하게 전부 인스턴스화되므로, 그 variant 에 YAML 이 없을 때의 거동이 계약이다.

| 상황 | 거동 |
|---|---|
| 파일 **없음**, `RTC_REGISTER_CONTROLLER` | 빈 노드로 `PreConfigure` → 내장 기본값으로 동작 |
| 파일 **없음**, `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` | **인스턴스화하지 않고 건너뜀** — `list_controllers` 에도 없다. "이 로봇은 이 컨트롤러를 쓰지 않는다" 의 표현 |
| 파일 **있음**, 파싱 실패 또는 최상위 키 불일치 | configure 전체 **거부**. 오타가 컨트롤러를 조용히 없애지 못하게 한다 |

두 번째 갈래는 기본값으로 돌 수 없는 컨트롤러용이다 (선택 기준: [rtc_controller_interface/README.md](../rtc_controller_interface/README.md#rtc_register_controller_requiring_config--config-가-없으면-건너뛴다)). 그런 컨트롤러를 일반 매크로로 등록하면 그 로봇의 모든 컨트롤러가 거부된다.

### E-STOP 및 안전 메커니즘

#### E-STOP 트리거

사유 토큰은 로그의 `GLOBAL E-STOP triggered: <reason>` 에 나온다.

| 트리거 | 조건 |
|---|---|
| `{group}_init_timeout` | `init_timeout_sec` 안에 해당 그룹이 state 미보고. 노드 종료 |
| `{group}_timeout` | 그룹의 state 토픽이 설정 ms 초과 미갱신 (50 Hz 워치독) |
| `consecutive_overrun` | 연속 10회 RT 루프 오버런 |
| `sim_sync_timeout` | sim 모드에서 step 완료를 `sim_sync_timeout_sec` 안에 못 받음. 노드 종료 |
| `invalid_controller_output_*` · `unholdable_controller_output_*` | `ControllerOutput` 이 actuator-boundary validation 을 연속 실패 (hold 를 못 만들면 첫 tick). 접미사가 `OutputRejectReason` |
| `unhonoured_command_type_*` · `unholdable_unhonoured_command_type_*` | 해소된 per-device command type 을 그 slot 의 backend 가 받지 않음. 같은 consecutive 창. 접미사가 거부된 mode |

#### 글로벌 E-STOP 동작

- `TriggerGlobalEstop()` 은 멱등이다. 모든 컨트롤러에 `TriggerEstop()` + `SetHandEstop(true)` 를 전파한다
- RT 루프는 E-STOP 후에도 계속 돌고 (타이밍 · 로깅 유지) **actuator 로 나가는 command 는 CM 이 차단**한다 (아래 §E-STOP 시 actuator command 차단)
- `/system/estop_status` 는 `true` 를 non-RT `DrainLog()` (100 Hz) 에서 늦게 발행한다 — `TriggerGlobalEstop` 이 RT 에서 도달 가능해 plain publisher 호출은 RT-10 위반이기 때문이다
- 늦게 뜬 구독자가 현재 값을 받으려면 구독자도 **transient_local** 이어야 한다 (volatile 구독자는 매칭되지만 이력을 못 받는다). in-tree 구독자는 모두 그렇다. 발행자는 만들어질 때 현재 값을 한 번 발행한다

#### 글로벌 E-STOP 해제 (`/rtc_cm/clear_estop` 서비스)

걸린 래치를 밖에서 내리는 유일한 경로다. `reset_fault` 의 글로벌 판 대응물이며 **양방향으로 분리돼 있다 (E-8)**: 둘 중 하나를 풀어도 다른 쪽은 그대로다. `message` 가 다른 쪽이 아직 걸려 있음을 알려 준다.

> **wire 계약** — `reason_ack` 의미, `ok` 판정 기준, 거부 4종의 구분은 [`rtc_msgs/srv/ClearEstop.srv`](../rtc_msgs/srv/ClearEstop.srv) 주석이 SSoT 다. 아래는 CM 쪽 메커니즘만 다룬다.

```bash
# 1) 빈 ack 로 불러 거부 메시지에서 현재 사유를 읽는다 (또는 "GLOBAL E-STOP triggered: <reason>" 로그)
ros2 service call /rtc_cm/clear_estop rtc_msgs/srv/ClearEstop "{reason_ack: ''}"
# 2) 사유를 그대로 되돌려 해제 (예: 사유가 arm_timeout 일 때)
ros2 service call /rtc_cm/clear_estop rtc_msgs/srv/ClearEstop "{reason_ack: 'arm_timeout'}"
```

- **검증 창** = `EstopVerifyWindowTicks()` = `watchdog_check_divisor_ + 2` tick (500 Hz 기준 12 tick, 24 ms). 서비스 deadline 은 그 4배다. 해제 뒤 원인 detector 가 한 번은 돌아야 해제가 검증된다
- **창 동안 hold 는 그대로다.** 서비스가 래치를 내리기 **전에** 검증 토큰을 세우고, 창을 채우면 RT 가 토큰을 내린다. 서비스 timeout 이 나도 hold 는 RT 가 창을 돌 때까지 유지된다. `ok=true` 는 RT 가 끝까지 돈 창이 자기 토큰일 때만이다
- 창 동안 `/system/estop_status` 는 true 를 유지하고 (거부된 해제는 false 를 내지 않는다) `switch_controller` 는 `E-STOP active (clear still being verified)` 로 거부된다
- **전파 중 재트리거 (`kRetriggered`)**: clear 가 전파되는 사이 다른 trigger 가 도착하면 해제를 포기하고 래치를 유지한다 (fail-closed). 이 경로에 걸릴 수 있는 것은 가드 없는 `consecutive_overrun` · `sim_sync_timeout` 뿐이다
- 컨트롤러 CSV 의 `estop_active` 는 **컨트롤러 자신의** latch 라 창 동안 이미 0 이다. CM 이 hold 중인지는 CM 카운터 `EstopVerifyHeldOutputCount` 로 본다
- `on_configure` · `on_activate` · `on_deactivate` · `on_error` 는 RT 스레드가 멈춘 상태에서 토큰과 latch 를 비운다. `ClearGlobalEstop()` 직접 호출 (테스트) 은 창이 없다

근거: [architecture-rationale.md](../docs/reference/architecture-rationale.md#controller-manager--e-stop--command-type-근거).

#### E-STOP 시 actuator command 차단

latch 가 서면 (또는 해제 검증 창 동안) 컨트롤러 output 은 `DeviceBackend::WriteCommand` 에 도달하지 않고 `BuildLatchedHoldOutput()` 로 치환된다.

| command type | hold 값 | 물리적 의미 |
|---|---|---|
| `kPosition` · `kPdFeedforward` | **latch 한 위치** | 진짜 정지 — 드라이브가 그 자세를 잡는다 |
| `kTorque` | `0 N·m` | 팔이 중력에 **sag** 한다 |

- **hold 목표는 slot 별 첫 치환 tick 의 측정(없으면 마지막 판독값)을 latch 한 값**이고 매 tick 갱신하지 않는다. 이미 hold 중인 재 trigger 는 재캡처하지 않는다. 다음 E-STOP 은 새로 잡는다
- 유한한 값이 하나도 없는 채널이 있으면 그 device 만 `WriteSafeCommand()` 를 부르고 다음 tick 에 다시 잡는다
- 이 치환은 [`rtc_controller_interface`](../rtc_controller_interface/README.md) 의 actuator-boundary output validation **뒤에** 얹힌다. 카운터는 `RejectedOutputCount` (validation) · `EstopSubstitutedOutputCount` (E-STOP 치환) · `EstopVerifyHeldOutputCount` (해제 창 hold) 로 따로 센다
- `kTorque` 의 sag 는 escalation 까지 `invalid_output_estop_ticks_` (rate 유도, 10-tick floor) 동안 이어지고 **설정으로 줄일 수 없다**. torque 페어링이 허용되면 configure 가 실측 노출 창을 WARN 으로 고지한다

#### Backend command-type 선언

`DeviceBackend::AcceptsCommandType(CommandType)` 의 **기본값은 position-only** 다. 더 넓게 선언하는 것은 opt-in 이고, "`WriteCommand` 가 enum 을 받는다" 가 아니라 **wire format 이 그 구분을 실어 나른다** 는 뜻이다. CM 은 `on_configure` 에서 등록된 모든 컨트롤러의 `GetCommandType()` 을 그 컨트롤러가 구동하는 그룹의 backend 와 대조하고 (`ValidateCommandTypePairing`), 불일치면 configure 를 거부한다.

| 질문 | 판정 | 결과 |
|---|---|---|
| **전송** — 이 wire 가 그 모드를 실을 수 있나 | `CommandTypeIsHonoured(slot 마스크, ct)` | 거짓이면 **거부** (configure FAILURE) |
| **안전** — E-STOP 치환이 여전히 "정지" 인가 | `kTorque` 여부 | 아니면 **경고** |

선언하지 않은 backend 는 torque 를 fail-closed 로 거부하므로 torque 구성은 "YAML 한 단어 + opt-in 한 backend" 다. in-tree 에서 opt-in 한 것은 `mujoco_native` (sim) 뿐이고, `integrated_bringup/config/**` 의 컨트롤러 config 는 전부 `command_type: "position"` 이다. 이 검사는 컨트롤러 단위 `GetCommandType()` 만 본다. 매 tick 정해지는 per-device 값은 아래 검사가 본다.

#### 해소된 per-device command type 의 tick-path 검사

`WriteCommand` 직전에 해소된 `devices[i].command_type` 을 backend 선언과 대조한다. `AcceptsCommandType` 은 configure 에서 한 번만 불러 slot 당 3비트 마스크로 캐시하므로 (`CacheSlotCommandTypeMasks`) tick 에서는 비트 테스트 하나다.

- **통과 기준은 `commands[]` 값의 의미다.** `kPosition` 과 `kPdFeedforward` 는 둘 다 위치 목표를 싣고 feedforward 는 추가 채널이라, position-only backend 가 `kPdFeedforward` 를 받는 것은 허용되는 폴백이며 wire 의 `command_type` 문자열도 강등하지 않는다. `kTorque` 만 물리량이 다르다
- 위 configure 게이트와 이 tick 검사는 **같은 술어**를 읽는다. 어느 축으로 도착했든 같은 wire 동작을 같게 판정하기 위해서다
- `num_channels == 0` 인 device 는 판정 대상이 아니다 (`SilenceDeviceOutput` 의 "no update" 답이라 mode selector 가 길이 0 배열을 지칭한다)
- **걸리면** 그 tick 출력은 output validation 과 **같은 hold 로 치환**되고, 같은 consecutive 창 (카운터 공유) 이 지속되면 `unhonoured_command_type_<mode>` E-STOP 으로 승격한다. 위반 tick 이 wire 에 도달하지 않는다. 카운터는 `RejectedCommandTypeCount`
- 창 안에서 사라지는 거부는 사유 토큰을 남기지 않으므로 RT 가 고정 버퍼에 한 줄을 적고 `DrainLog()` 가 WARN 으로 낸다. 재출력은 약 1초 간격이고 억제된 tick 수가 다음 줄 꼬리에 실려 "지속" 과 "1회" 가 구별된다

#### Backend safe-output 계약

`DeviceBackend::WriteSafeCommand()` 는 pure virtual 이고 인자가 없다. CM 은 actuator 에 normal command 를 보낼 수 없고 **hold 도 못 만들 때** (device 가 non-finite 위치를 보고) 부른다. zero-length command 는 fail-closed 가 아니라 "하던 것을 계속하라" 는 침묵이다. 현재 backend 3종의 답은 "마지막 good command 재발행" 이므로 물리적으로는 침묵 경로와 같은 자리에 머문다. 달라지는 것은 의도가 wire 와 rosbag 에 명시된다는 것이다. 새 backend 는 `device_backend.hpp` 의 Doxygen 계약을 따른다.

### 타이밍과 로그

`ControllerTimingProfiler` 가 `Compute()` 의 락-프리 히스토그램 (200 버킷, 10 us 간격, 0-2000 us) 을 모은다. RT 루프가 1000 tick 마다 로깅 스레드에 요청해 서머리를 내고 리셋하므로 각 출력은 **직전 윈도우**의 값이다.

```text
<ctrl> timing: elapsed=Xs  mean=Yµs  max=Zµs  overruns=N  skips=N  nrt_pub_drops=N  timing_drops=N  rt_cb_timing_drops=N
```

누적 카운터 (`overruns` · `skips` · `*_drops`) 는 서머리 리셋과 무관하다. 예산은 `1e6 / control_rate` µs 다. sim 모드의 `elapsed` 는 `count × period` (가상 시간) 이고 robot 모드는 wall-clock 이다.

타이밍 CSV 는 세션 디렉토리 `<session>/timing/` 아래에 쓰인다. 세 채널이 같은 5-컬럼 payload (`t_state_us` · `t_compute_us` · `t_publish_us` · `t_total_us` · `jitter_us`) 에 공통 컬럼 `t_wall_ns` · `tick_count` · `run_id` 를 붙인다.

| 파일 | 한 행의 단위 |
|---|---|
| `cm_timing_log.csv` | CM RT 루프 tick (`control_rate`) |
| `mpc_timing_log.csv` | MPC 스레드 tick (≤ 100 Hz, 컨트롤러별 `<config_key>` 서브디렉토리) |
| `rt_callback_timing_log.csv` | **디바이스 state 콜백** 1회 (tick 이 아니다) |

- `tick_count` 는 drop 검증용 단조 번호다. `run_id` 는 기동 식별자다 — 세션 디렉토리가 분 해상도라 같은 분의 재기동이 같은 파일에 append 되는 경계를 가른다. 한 launch 의 프로세스는 `$RTC_RUN_ID` 를 공유하므로 채널 간 join 이 된다
- `cm_timing_log.csv` 에서 `t_total_us − (t_state+t_compute+t_publish)` 잔차가 그 tick 의 post-publish tail 이다. 긴 tick 을 "publish 때문" 이라 읽기 전에 이 잔차를 먼저 본다
- sim 모드에서는 `jitter_us` 가 0 이다 (wakeup 이 sim step 이라 jitter 는 RT 지표가 아니다)
- `rt_callback` 행의 `t_compute_us` · `jitter_us` 는 0 이다. `t_total_us` 는 콜백 **본문** 시간이라 executor dispatch 는 빠지고, 합 / 관측창은 slot 2 duty 의 하한이다

기반 인프라는 [`rtc_base/threading/periodic_rt_thread.hpp`](../rtc_base/include/rtc_base/threading/periodic_rt_thread.hpp) 와 [`rtc_base/timing/`](../rtc_base/include/rtc_base/timing/) 다. 새 RT thread 는 `PeriodicRtThread` 를 상속하거나 합성하고 `OnTick` 만 정의하면 된다.

## 설정

### 설정 파일

이 패키지는 **default YAML 을 동봉하지 않는다.** 모든 파라미터는 robot-agnostic 코드 기본값으로 declare 되고 (`devices` · `urdf` 는 미선언), production 값은 `<robot>_bringup` 의 YAML 이 단독 소유한다 (ARCH-1). 예: `integrated_bringup/config/{ur5e_p1a,iiwa7_leap}/sim.yaml`. 새 robot 은 자체 YAML 을 만들어 launch 에서 주입한다.

`config_variant` 파라미터가 컨트롤러 YAML 경로 `<pkg_share>/config/<config_variant>/controllers/<config_key>.yaml` 을 정한다 (빈 값이면 `config/controllers/`). 구성은 세 갈래다:

| 무엇 | 어디 |
|---|---|
| 노드 파라미터 (`control_rate` · `devices` · `urdf` · …) | `<robot>/{sim,robot}.yaml` 의 `/**: ros__parameters:` |
| 컨트롤러 YAML | `config/<variant>/controllers/<config_key>.yaml` (이 절의 `include:`) |
| 컨트롤러 파라미터 override | CM 노드의 `<config_key>.<yaml.경로>` 파라미터 (launch 인자 · overlay) |

#### 파라미터

값은 코드 기본값이다. 로봇별 값은 위 bringup YAML 을 본다.

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `control_rate` | double | `500.0` | 제어 주파수 (Hz) |
| `initial_controller` | string | `""` | 시작 컨트롤러 (이름 또는 config_key). 빈 값이면 bringup YAML 이 지정해야 한다 |
| `init_timeout_sec` | double | `5.0` | 초기화 타임아웃 (초). state 미수신 시 E-STOP. sim 은 `0.0` 으로 끈다 |
| `enable_estop` | bool | `true` | E-STOP 워치독 활성화 |
| `device_timeout_names` | string[] | `[]` | 그룹별 timeout 을 줄 대상 이름 (감시 대상은 설정된 모든 그룹) |
| `device_timeout_values` | double[] | `[]` | 각 그룹의 state 타임아웃 (ms) |
| `device_timeout_default_ms` | double | `1000.0` | 위 목록에 없는 그룹의 timeout (ms) |
| `enable_logging` · `enable_timing_log` · `enable_device_log` | bool | `true` | CSV 로깅 전체 · 타이밍 · 디바이스별 |
| `log_dir` | string | `""` | 로그 디렉토리 (빈 값이면 자동 생성) |
| `max_log_sessions` | int | `10` | 로그 세션 보관 수. bringup launch 도 같은 트리를 정리하므로 launch 가 이 값을 자기 인자의 default 로 읽는다 — YAML 이 SSoT 이고 CLI `max_log_sessions:=N` 은 양쪽을 함께 움직인다 |
| `use_sim_time_sync` | bool | `false` | MuJoCo lock-step — RT tick 을 sim step 완료로 깨운다 |
| `sim_sync_timeout_sec` | double | `5.0` | step 이 이 시간 안에 완성되지 않으면 `sim_sync_timeout` E-STOP |
| `sim_sync_tick_devices` | string[] | `[]` (= 모든 그룹) | sim step 을 완성하는 그룹. decimated 로 발행하는 그룹과 `fake_response` 그룹은 뺀다 (넣으면 step 이 느려진다). sim-sync 에서 모르는 이름은 configure 거부, `use_sim_time_sync: false` 면 키를 읽지 않는다 |
| `config_variant` | string | `""` | 컨트롤러 YAML 탐색 디렉토리 (위) |
| `rt_layout_profile` | string | (미선언) | launch 의 thread-layout profile id (`mpc_on` / `mpc_off`). CM 은 쓰지 않고 각 컨트롤러 노드로 전달만 한다 |
| `kp` · `kd` | double | `5.0` · `0.5` | (레거시) 기본 P/D 게인 |

**`urdf.*`** (시스템 URDF, 모두 선택):

| 파라미터 | 설명 |
|---|---|
| `urdf.package` · `urdf.path` | URDF 의 ament 패키지와 상대 경로 |
| `urdf.root_joint_type` | `"fixed"` (기본) 또는 `"floating"` |
| `urdf.extended` · `urdf.closure_path` | `true` 면 closure sidecar 로드 (기본 `false`). 경로 미지정 시 URDF 경로에서 유도 |
| `urdf.sub_models.<name>` | 직렬 체인 `{root_link, tip_link}`. map 키가 모델명 |
| `urdf.tree_models.<name>` | 분기 체인 `{root_link, tip_links[]}` |
| `urdf.passive_joints` | 모든 모델에서 잠글 관절 이름 |

**`devices.<group>.*`**:

| 파라미터 | 설명 |
|---|---|
| `joint_state_names` | 관절 이름 (reorder 기준, gains/limits 배열이 이 순서를 따른다). URDF 와 *상대* 순서가 다르면 WARN |
| `joint_command_names` · `motor_state_names` · `sensor_names` | 선택. command 이름 (미지정 시 `joint_state_names`) · 모터 공간 · 센서 이름 |
| `safe_position` | E-STOP 시 안전 위치 (관절 수와 동일) |
| `urdf.{package,path,root_link,tip_link}` | 선택. 시스템 `sub_models` / `tree_models` 에서 자동 해석되지 않을 때 |
| `joint_limits.{max_velocity,max_torque,position_lower,position_upper}` | URDF 와 병합해 더 보수적인 값을 쓴다. YAML 에 없으면 URDF 값만 |
| `sensor_layout.{primary,secondary}_count_per_group` · `values_per_group` | packed-sensor 토픽의 stride. CM 은 의미를 모르고 stride 계산만 한다. 미설정이면 해당 콜백은 WARN 후 drop |
| `sensor_layout.inference_values_per_group` | ML 추론 출력의 그룹당 값 수 |
| `sensor_layout.has_native_{contact,displacement}` | backend 가 inference slot 에 native 신호를 채우는지 advertise (기본 `false`). 컨트롤러 행동 분기에만 영향 |

#### 컨트롤러 파라미터 override — `<config_key>.<yaml.경로>`

컨트롤러 YAML 은 ROS 파라미터 트리 밖에서 읽힌다. 그래서 CM 노드에 걸린 `<config_key>.<yaml.경로>` 파라미터 (launch 인자, sim overlay) 를 CM 이 configure 때 그 컨트롤러의 YAML 트리에 써 넣는다. `.` 이 YAML 경로의 구분자다. 예를 들어 `demo_x_controller.gains.kp: 2.0` 은 `demo_x_controller.yaml` 의 `gains.kp` 를 2.0 으로 바꾼다.

- **경로는 YAML 그대로여야 한다.** 쓰기는 받은 경로를 그대로 만든다. 한 단계 어긋난 키는 만들어지기만 하고 컨트롤러가 읽지 않아, 출하값으로 돈다
- **YAML 에 없는 경로는 WARN 한 줄 (`[param-override]`) 을 낸 뒤 그대로 쓴다.** 문구에 파라미터 이름, YAML 에 있는 가장 긴 접두, 그 경로로 끝나는 기존 키 후보 최대 3개가 나온다. 컨트롤러가 YAML 에 없는 키를 내장 기본값과 함께 읽을 수 있어 정당한 override 와 오타를 구별할 수 없기 때문이다
- **YAML 구조를 깨야만 쓸 수 있는 override 는 configure 를 거부한다.** 경로가 scalar · sequence 를 지나는 경우, 비어 있지 않은 map 을 가리키는 경우, 한 override 가 다른 override 의 부모 키인 경우다. ERROR 문구가 막힌 경로를 모두 적는다
- **파일은 있는데 그 키 아래 트리가 없으면 override 를 적용하지 않고 WARN** 한다 (top-level 키 오타). **config 파일이 없는 컨트롤러에 온 override 도 적용하지 않고 WARN** 한다
- **출하 bring-up 은 이 경고가 0건이어야 한다.** launch 가 쓰는 키는 Python 쪽 검사가 출하 YAML 의 키임을 고정한다 (`rtc_tools.utils.controller_config` · `integrated_bringup` pytest)

#### 컨트롤러 YAML 을 여러 파일로 나누기 — `include:`

컨트롤러 YAML 은 top-level 에 `include:` 목록을 둘 수 있다. 각 조각은 같은 트리의 일부를 `<config_key>:` 아래에 갖고, CM 이 하나의 노드로 합친 뒤에 파라미터 override 를 적용한다. 그래서 override 는 조각의 키에도 주 파일의 키와 똑같이 먹는다.

```yaml
# config/<variant>/controllers/demo_x_controller.yaml
include:
  - x/part_a.yaml          # 이 파일의 디렉토리 기준 상대경로
demo_x_controller:
  gains:
    kp: 1.0
```

```yaml
# config/<variant>/controllers/x/part_a.yaml
demo_x_controller:
  gains:
    kd: 2.0                # gains 는 두 파일의 leaf 가 합쳐진다
```

| 규칙 | 내용 |
|---|---|
| 경로 | 주 파일의 디렉토리 기준 상대경로이고 **하위 폴더** 에 있어야 한다 (`x/part_a.yaml` — `part_a.yaml` 은 거부). 절대경로와 `..` 도 거부 |
| 주 파일 | `include` 가 있으면 top-level 키는 `include` 와 `<config_key>` 둘뿐이고, `<config_key>` 는 map 이다 |
| 조각 | top-level 키는 `<config_key>` (map) 하나. 조각은 다시 include 하지 못한다 |
| 병합 | map 은 재귀로 합친다. scalar · sequence · null 은 leaf 다 — sequence 는 이어 붙이지 않는다 |
| 충돌 | 같은 leaf 를 두 파일이 적거나, 한쪽이 map 이고 다른 쪽이 leaf 면 에러 (값이 같아도) |
| 순서 | 주 파일의 키가 먼저, 그 뒤 include 순서로 각 조각의 새 키 |
| 파일 하나 | YAML 문서 하나여야 한다 (`---` 뒤의 둘째 문서는 거부). 한 map 에 같은 키를 두 번 적으면 거부 |

- **없는 조각은 "파일 없음" 이 아니다.** 주 파일이 있고 조각이 없으면 `ControllerConfigIncludeError` 로 configure 전체를 거부한다 — `config_required` 컨트롤러도 건너뛰지 않는다. 모든 충돌과 형식 오류가 같은 예외이고 문구에 파일 이름과 키 경로가 있다
- `include` 가 없는 파일은 지금까지와 똑같이 읽힌다. 조각을 하위 폴더에 두는 것은 `controllers/*.yaml` 을 컨트롤러 목록으로 읽는 독자가 조각을 컨트롤러로 세지 않게 하기 위해서다
- YAML anchor · alias 가 걸린 map 에 조각을 합치면 alias 쪽도 바뀌고 merge key (`<<`) 는 펼쳐지지 않는다. 나뉜 config 에는 쓰지 않는다
- **CM 밖에서 컨트롤러 YAML 을 경로로 읽는 코드는 같은 로더를 써야 한다.** 주 파일만 읽으면 조각의 키가 빠진 트리를 본다. C++ 는 `rtc::LoadControllerConfig(yaml_path, config_key)`, Python 은 `rtc_tools.utils.controller_config.load_controller_config(path)` 다. 한쪽을 바꾸면 함께 바꾸고, 두 로더가 같은 트리를 낸다는 것은 `config/test_include/expected_leaves.txt` 가 고정한다

#### cyclone_dds.xml

[config/cyclone_dds.xml](config/cyclone_dds.xml) 은 CycloneDDS 의 RT 설정이며 `CYCLONEDDS_URI` 로 로드된다: SPDP 만 멀티캐스트 (`AllowMulticast=spdp`), 빠른 participant 감지 (`LeaseDuration=5s` · `SPDPInterval=1s`), 빠른 재전송 (`NackDelay=10ms` · `HeartbeatInterval=100ms`), 콜백 지연 제거 (`SynchronousDeliveryLatencyBound=inf`). CycloneDDS 에 CPU affinity 설정이 없어 (0.10.5 `<Thread>` 는 `StackSize` 와 scheduling 만) DDS 스레드 affinity 는 `taskset` 으로 처리한다.

## 디렉토리

```text
rtc_controller_manager/
├── include/rtc_controller_manager/  # 공개 헤더
├── src/                             # 노드 구현 · 로더
├── config/                          # DDS 설정 · test fixture
└── test/                            # gtest
```

`src/rt_controller_node_*.cpp` 는 기능별로 나뉜다 (`params` · `device_config` · `services` · `switch` · `rt_loop` · `publish` · `estop`). `config/test_*` 는 unit test 전용이며 production bring-up 은 참조하지 않는다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| configure 가 실패하고 식별자 충돌 로그 | 컨트롤러 클래스를 복사하고 `Name()` 을 안 고쳤거나 `config_key` 중복 | `Name()` 과 `config_key` 를 전역 유일하게 |
| configure 가 `command_type` 불일치로 거부 | torque 컨트롤러가 position-only backend 에 묶임 | backend 가 `AcceptsCommandType` 을 opt-in 하는지 확인. 아니면 `command_type: "position"` |
| 컨트롤러가 `list_controllers` 에 없다 | `REQUIRING_CONFIG` 컨트롤러인데 그 variant 에 YAML 이 없음 | 의도라면 정상. 아니면 `config_variant` 와 파일 경로 확인 |
| 컨트롤러 override 가 안 먹는다 | 경로가 YAML 과 한 단계 어긋남 (`[param-override]` WARN) | WARN 의 접두 · 후보 키를 보고 경로를 고친다 |
| `clear_estop` 이 거부된다 | `reason_ack` 가 현재 사유와 다름 | 빈 ack 로 불러 거부 메시지의 사유를 그대로 되돌린다. 컨트롤러 fault 는 `reset_fault` 로 따로 푼다 |
| 해제 후에도 `estop_status` 가 true | 해제 검증 창 중이거나 원인이 남아 재 latch | 창은 수십 ms. 계속되면 사유 토큰으로 원인 확인 |
| goal 을 보냈는데 컨트롤러가 안 움직인다 | 큐 포화 · payload 거부 · 배선 결함 · fault 래치 | `list_controllers` 진단 4필드를 두 번 폴링해 어느 카운터가 움직이는지 본다 |
| sim 에서 `sim_sync_timeout` | step 이 늦거나 `sim_sync_tick_devices` 에 step 과 무관한 그룹이 있음 | 해당 그룹을 `sim_sync_tick_devices` 에서 뺀다 |

## 더 읽기

- E-STOP · command type · 3-pass 생성의 근거: [architecture-rationale.md](../docs/reference/architecture-rationale.md#controller-manager--e-stop--command-type-근거)
- 스레드·데이터 흐름 규범: [architecture.md](../agent_docs/architecture.md) · RT 불변식: [invariants.md](../agent_docs/invariants.md)
- RT 환경 설정: [rt-optimization.md](../docs/rt-optimization.md)
- 컨트롤러 인터페이스·lifecycle 훅: [rtc_controller_interface/README.md](../rtc_controller_interface/README.md)
- srv 계약: [rtc_msgs/README.md](../rtc_msgs/README.md) · 로봇별 YAML·launch: [integrated_bringup/README.md](../integrated_bringup/README.md)

## 의존성

[package.xml](package.xml) 참조 (rclcpp_lifecycle · rtc_base · rtc_controller_interface · rtc_controllers · rtc_msgs · rtc_urdf_bridge · pinocchio · yaml-cpp 등). 라이선스: MIT
