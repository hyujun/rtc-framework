# rtc_controller_interface

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

## 개요

RTC 프레임워크의 **컨트롤러 추상 인터페이스 및 플러그인 레지스트리**를 제공하는 공유 C++ 라이브러리(`librtc_controller_interface.so`)입니다. Strategy 패턴을 사용하여 CM 이 컨트롤러를 구동하기 위한 추상 기반 클래스(`RTControllerInterface`)와, 정적 초기화 시점에 컨트롤러를 자동 등록하는 싱글톤 레지스트리(`ControllerRegistry`), 그리고 등록 매크로(`RTC_REGISTER_CONTROLLER`)를 제공합니다.

> **누가 이것을 구현하는가**: 제어 *알고리즘* 이 아니라 **바인딩**이다 — integration 패키지가 소유하는, 프레임워크 계약을 알고리즘에 이어 붙이는 클래스. `rtc_controllers` 의 제어 법칙은 이 인터페이스를 몰라야 한다 ([agent_docs/design-principles.md](../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms 가 SSoT — 3계층 배치·전이 상태 포함). 이 패키지는 그 3계층 중 **base 계층**이며, 모든 바인딩이 동일하게 필요로 하는 글루(target mailbox · submodel 선택 · device 한계값 · 판독가능성 게이트 · E-STOP scaffolding)를 여기로 모으는 것이 #206 → #236 S7 의 방향이다.

---

## 핵심 컴포넌트

### 헤더 파일

| 파일 | 설명 |
|------|------|
| `include/rtc_controller_interface/rt_controller_interface.hpp` | 추상 컨트롤러 인터페이스 (`RTControllerInterface`) |
| `include/rtc_controller_interface/controller_registry.hpp` | 싱글톤 레지스트리 (`ControllerRegistry`) + `RTC_REGISTER_CONTROLLER` 매크로 |
| `include/rtc_controller_interface/controller_types.hpp` | `rtc_base/types/types.hpp` 재수출 (편의 헤더) |
| `include/rtc_controller_interface/controller_log_set.hpp` | Controller-owned CSV 로그 집합 헬퍼 (`ControllerLogSet` + `LogHandle<PodT>`) — opt-in. `rtc::ThreadCsvProducer/Logger` 페어를 typed handle 로 묶어 `<session>/controllers/<config_key>/<instance>.csv` 에 기록. 같은 LogSet 안에서 동일 `instance` 를 두 번 등록하면 `RegisterLog` 가 unbound `LogHandle` 반환 (Q-MSG-3 path-uniqueness enforcement) |

### 소스 파일

| 파일 | 설명 |
|------|------|
| `src/rt_controller_interface.cpp` | 기본 생성자, `LoadConfig()`, `ParseTopicConfig()`, 순수 검증 함수 `CheckTargetLimits()` / `ValidateControllerOutput()` 구현 |
| `src/controller_registry.cpp` | Meyer's 싱글톤 `Instance()` 및 `Register()` 구현 |

---

## RTControllerInterface (추상 기반 클래스)

`rtc` 네임스페이스에 정의되며, 복사/이동이 금지(`delete`)된 추상 클래스입니다. 소멸자는 `virtual` — `RtControllerNode`가 컨트롤러를 `std::vector<std::unique_ptr<RTControllerInterface>>`로 base 포인터를 통해 소유/파괴하므로 derived dtor가 반드시 호출되어야 한다 (비-virtual로 두면 RT 스레드/SPSC ring/모델 등 derived 멤버 cleanup이 silently skip되어 use-after-free SEGV로 이어짐 — sim shutdown 회귀의 직접 원인이었음).

### 순수 가상 메서드 (반드시 구현, 모두 `noexcept`)

```cpp
[[nodiscard]] virtual ControllerOutput Compute(const ControllerState& state) noexcept = 0;
virtual void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept = 0;
[[nodiscard]] virtual std::string_view Name() const noexcept = 0;
```

| 메서드 | 설명 |
|--------|------|
| `Compute` | RT 루프에서 호출되는 제어 연산. `ControllerOutput` 반환. **첫 호출 시 controller가 자체적으로 target slot을 현재 device state로 seed** (controller-internal init policy — base가 별도 hold-init 훅을 제공하지 않음) |
| `SetDeviceTarget` | 디바이스 인덱스 및 가변 크기 목표값 설정. 구현체는 off-RT 콜백에서 호출되며, RT 스레드와의 race를 피하기 위해 SPSC 큐로 marshal한 뒤 `Compute()` 안에서 drain한다 (single-writer SeqLock invariant) |
| `Name` | 사람이 읽을 수 있는 컨트롤러 이름 반환 |

### SetDeviceTaskTarget (default-virtual, SE3 goal 전용 슬롯)

```cpp
virtual void SetDeviceTaskTarget(int device_idx, std::span<const double> task6) noexcept {
  SetDeviceTarget(device_idx, task6);
}
```

`device_idx`용 6-DoF 태스크 공간(SE3) 목표 `(x, y, z, r, p, y)`를 전달합니다. 순수 가상이 아니라 **기본 구현이 `SetDeviceTarget`로 forwarding** 하는 default-virtual — joint 슬롯과 SE3 슬롯을 분리하지 않는 기존 태스크 컨트롤러(예: DemoTask는 device-0 버퍼를 이미 SE3 pose로 해석)는 override 없이 동작이 그대로 유지됩니다. joint 목표와 태스크 목표를 별도 슬롯에 보관해야 하는 컨트롤러(예: DemoWbc — arm joint posture 슬롯과 commanded SE3 슬롯이 분리)는 이 메서드를 override해 SE3 슬롯에만 쓰고 joint 슬롯을 clobber하지 않도록 합니다. `SetDeviceTarget`과 동일하게 `noexcept` + RT-marshal-safe (SPSC 큐 경유)여야 합니다.

`RTControllerInterface::DeliverTargetMessage()`(`rt_controller_interface.cpp`)는 컨트롤러-owned target 구독 콜백에서 호출되며, 수신한 `RobotTarget.goal_type`에 따라 디스패치를 분기합니다.

| `goal_type` | 데이터 소스 | 리오더링 | 디스패치 |
|---|---|---|---|
| `"task"` | `msg.task_target` | 없음 (그대로 전달) | `SetDeviceTaskTarget(device_idx, ordered_span)` |
| 그 외 (joint) | `msg.joint_target` | `msg.joint_names` → `device_name_configs_[group_name].joint_state_names` 순서로 재정렬 | `SetDeviceTarget(device_idx, ordered_span)` |

#### Joint 목표 limit 스크리닝 (warn-only)

joint 목표는 디스패치 직전에 `device_name_configs_[group_name].joint_limits`의 `position_lower` / `position_upper`와 대조되며, 벗어나면 위반 joint 이름·값·범위를 담은 throttled `RCLCPP_WARN`이 발행됩니다. 리오더링이 끝난 `ordered_span`에서 검사하므로 인덱스가 `joint_state_names`와 1:1로 맞습니다.

- **Warn-only** — 커맨드를 거부하거나 수정하지 않습니다. 실제 강제는 각 컨트롤러 `WriteJointCommand`의 RT-path `ClampRange`가 그대로 담당하며, 이 로그는 "왜 goal 이 요청한 곳에 안 갔는지"를 조용한 clamp 대신 보이게 하는 것이 목적입니다.
- **`joint_limits` 미설정 device 는 침묵** — `LoadDeviceLimitsFromConfig`의 ±2π fallback 은 실제 안전 범위가 아니라 placeholder 이므로, 이를 기준으로 경고하면 전부 false-positive 가 됩니다.
- **task 목표는 대상 외** — `task_target`은 Cartesian `(x, y, z, r, p, y)`라 joint limit 이 적용되지 않습니다.
- **non-finite 도 위반으로 보고** — `std::clamp(NaN, lo, hi)`는 두 비교가 모두 false 라 NaN 을 그대로 반환합니다. 즉 RT clamp 가 막지 못하므로 반드시 표면화해야 합니다 (현재는 경고만 — drop 하지 않음).
- 판정 로직은 순수 함수 `rtc::CheckTargetLimits(ordered, lim)` → `TargetLimitViolation` 으로 분리돼 있어 ROS 없이 단위 테스트됩니다. 로그는 [conventions.md](../agent_docs/conventions.md) §Logger naming 에 따라 library-level logger (`rtc_controller_interface`) + 메시지 본문 `[<controller_name>]` prefix 를 씁니다.

### Activation generation gate (#196 §3)

`rclcpp_lifecycle` 은 **publisher 만** gating 하므로 controller 의 target subscription 은 Inactive 구간에도 살아 있다. 그래서 비활성 controller 앞으로 보낸 goal 이 계속 pending-target 큐에 쌓이고, 재활성화 첫 RT tick 에서 current-state hold 를 덮어쓸 수 있었다.

큐를 activation 시점에 비우는 것은 불가능하다 — `SpscQueue` 의 consumer 는 RT thread 하나뿐이라 lifecycle thread 에서 `Pop()` 하면 SPSC 계약이 깨진다. 대신 **generation 스탬프**를 쓴다.

| 요소 | 위치 | 역할 |
|---|---|---|
| `ActivationGeneration()` | base public | 현재 activation 세대. `on_activate` 마다 +1 (deactivate 는 증분하지 않음 — 비활성 중 push 된 entry 는 직전 세대를 달고 있다가 다음 activation 이 무효화한다) |
| `IsCurrentGeneration(gen)` | base public | RT drain 측 판정 술어 |
| `ResetTargetInitialization()` | base protected virtual | `on_activate` 가 매 활성화마다 호출. 파생 controller 는 여기서 `target_initialized_` 계열 latch 만 내려 첫 tick 재-seed 를 강제한다 |

각 controller 의 `PendingTarget` 은 `generation` 필드를 갖고, off-RT `SetDeviceTarget`/`SetDeviceTaskTarget` 이 push 시점의 세대를 찍으며, RT `Compute()` drain 이 `IsCurrentGeneration` 이 아닌 entry 를 버린다. 활성화 latch reset 은 **base 가 소유**하므로 controller 는 `on_activate` override 없이도 재-seed 를 얻는다 (이 소유권 이전 전에는 `on_activate` override 가 없는 controller 들이 재활성화 시 이전 hold 를 그대로 유지했다).

> **`PendingTarget` 자체는 아직 base 소유가 아니다** — `SpscQueue` + `SeqLock<TargetSlot>` + drain 루프는 구현체마다 복제돼 있고, base 가 소유하는 것은 generation 스탬프와 latch reset 뿐이다. 이 복제가 #206 의 내용이며, #236 이 그것을 슬라이스 S7 로 흡수해 mailbox 전체를 이 패키지로 올린다 (같은 글루 계층의 submodel 선택 · device 한계값 로드 · 판독가능성 게이트와 함께). 그때 이 절은 "각 controller 의" 가 아니라 "base 의" 로 다시 쓰인다. 배치 규칙의 SSoT 는 [agent_docs/design-principles.md](../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms 의 3계층 표다.

CM 이 아닌 단위 테스트가 직접 `Compute()` 를 도는 경로는 양쪽 모두 세대 0 이라 아무것도 드롭되지 않는다.

> **Hold-init 책임 분리 (2026-05-17, RT-4)**: 과거에는 `InitializeHoldPosition(state)` 순수 가상이 CM의 auto-hold 경로에서 RT 스레드로 호출됐다. 이는 `target_mutex_` (RT-4 위반) + writer-multiplicity race (lifecycle/RT/aux 3 thread)의 근원이었고 v1 시도에서 SeqLock 단순 적용으로는 해결되지 않았다. v2 cleanup으로 hold-init은 controller 내부 책임이 되었다 — 각 controller는 `target_initialized_` atomic을 두고 `Compute()` 첫 진입 시 현재 device state로 자체 seed. CM의 auto-hold 코드 / `BuildDeviceSnapshot` / `InitializeHoldPosition` 가상 함수는 모두 삭제됐다.

### Actuator-boundary output validation (#196 §4)

`ControllerOutput` 은 out-of-tree 계약(레지스트리로 로드되는 어떤 controller 든 채운다)인데, CM 은 그것을 그대로 `DeviceBackend::WriteCommand` 로 넘긴다 — 물리 액추에이터 직전 마지막 소프트웨어 단계다. 그 사이에 검증하는 곳이 없어서 NaN 커맨드나 디바이스가 보고하지 않은 채널 수가 그대로 모션이 되어 나갔다. `CheckTargetLimits` 의 ingress 검사를 egress 쪽에 대칭으로 둔 것이 이 검증이다.

순수 함수 `rtc::ValidateControllerOutput(out, state)` → `ControllerOutputValidation` 로 분리돼 ROS 없이 단위 테스트된다 (`test/test_output_validation.cpp`). RT-safe: 할당·로깅·throw 없음.

| 검사 | 규칙 | `OutputRejectReason` |
|---|---|---|
| `valid` 플래그 | `false` → reject | `kInvalidFlag` |
| 디바이스 수 | `out.num_devices == state.num_devices` 이고 `[0, kMaxDevices]` | `kDeviceCountMismatch` |
| 채널 수 | `[0, kMaxDeviceChannels]` **그리고** `<= state.devices[i].num_channels` | `kChannelCountOutOfRange` |
| finiteness | `commands[0..nc)` 및 `feedforward[0..nc)` 전부 finite | `kNonFiniteCommand` |

검사 범위는 **액추에이터에 도달하는 필드로 한정**한다 — `commands` / `feedforward` 뿐. task pose, trajectory reference, goal position 은 log/GUI lane 이라 값이 나빠도 눈에 보일 뿐 위험하지 않고, 검사하면 RT 예산만 쓴다. `feedforward` 는 `command_type` 과 무관하게 항상 검사한다: 어떤 타입인지 결정하는 필드 자체가 같은 미검증 출력에서 오기 때문에, 타입으로 게이팅하면 오염된 타입 필드가 NaN 을 통과시키는 경로가 된다.

CM 측 정책(호출부는 `rt_controller_node_rt_loop.cpp` 의 `Compute()` 반환 직후 1곳):

- **reject → hold 대체**. `command_type` 별로 hold 를 *새로 만든다* — 디바이스/채널 수는 `state`(read path 에서 이미 bounded), command type 은 활성 controller 의 `GetCommandType()`(YAML 유래). 거부된 출력 안의 per-device `command_type` override 는 **무시**한다: 방금 검증에 실패한 데이터에서 모드 선택자를 읽으면 검사가 무의미해진다. `kPosition`/`kPdFeedforward` → 측정 위치, `kTorque` → 0 N·m (+ ff 0).
- **write 를 건너뛰지 않는다**. 커맨드 스트림의 공백을 fault 로 볼지 free-run 으로 볼지는 backend 마다 달라 거동이 비결정적이 된다.
- **연속 N tick reject → global E-STOP 승격**. `N = max(10, lround(0.1 × control_rate))` — 고정 상수는 100 Hz~5 kHz 사이에서 의미가 50배 달라진다. 하한 10 은 `kMaxConsecutiveOverruns` 선례와 정합.
- **승격 후에도 검증은 계속 돈다** — controller 의 E-STOP 경로 출력마저 invalid 면 hold 가 계속 적용된다 (terminal state 에서도 fail-closed).
- **hold 를 만들 수 없으면 즉시 승격**. state 의 *count* 는 read path 에서 bound 되지만 *값* 은 backend 에서 verbatim 복사되므로 측정 위치가 non-finite 일 수 있다. 그 경우 hold 는 rejected output 이 담고 있던 바로 그 NaN 을 그대로 내보내게 된다. 0.0 은 대체재가 아니다 — position command 에서 0.0 은 "원점으로 가라"다. 따라서 해당 device 는 **zero-length command** 로 떨어뜨리고 (backend 는 no-update 로 처리), 연속 window 를 기다리지 않고 첫 tick 에 E-STOP 한다 (reason `unholdable_controller_output_*`). window 가 정당한 이유는 hold 가 그 사이를 안전하게 만들어주기 때문인데, 바로 그 전제가 깨진 상황이기 때문이다.

> **`kTorque` hold 의 수용된 리스크**: CM 은 동역학 모델이 없어 중력보상 토크를 만들 수 없고, 0 N·m 는 중력 sag 를 뜻한다. 최대 노출은 `0.1 s` + E-STOP 전파 지연이다. 토크 모드 디바이스가 있는 구성은 이 window 를 낮추는 튜닝 여지가 있다.

`rt_controller_node_rt_loop.cpp` 의 `BoundedCount` clamp 는 삭제되지 않고 defense-in-depth 로 남는다 — validator 를 통과한 출력과 hold 경로, 그리고 backend 가 보고한 state 쪽 count 에 계속 적용된다.

### 가상 메서드 (기본 구현 제공)

| 메서드 | 기본 동작 | 설명 |
|--------|----------|------|
| `TriggerEstop()` | no-op | 비상 정지 트리거 |
| `ClearEstop()` | no-op | 비상 정지 해제 |
| `IsEstopped()` | `false` 반환 | 비상 정지 상태 확인 |
| `SetHandEstop(bool)` | no-op | 핸드 비상 정지 설정 |
| `LoadConfig(const YAML::Node&)` | 디바이스 플래그 경고 + 토픽 파싱 | YAML 설정 로드. `noexcept`가 아님 (throw 가능) |
| `GetCommandType()` | `CommandType::kPosition` | 커맨드 타입 (`kPosition` 또는 `kTorque`) |

> **`LoadConfig()` 기본 구현 동작:**
> 1. `cfg["enable_ur5e"]` / `cfg["enable_hand"]` 존재 시 deprecated 경고 출력 후 무시
> 2. `cfg["topics"]` 존재 시 `ParseTopicConfig()` 호출, 없으면 기본 토픽 유지
>
> 하위 클래스에서 오버라이드 시 `RTControllerInterface::LoadConfig(cfg)`를 먼저 호출해야 토픽 설정이 적용됩니다.

> **Observability**: 인터페이스에는 도메인-specific virtual이 없습니다. 컨트롤러가 MPC 솔버, ONNX inference 등 자체 RT/soft-RT 스레드의 per-tick timing을 CSV로 남기려면 [`rtc_base/timing/thread_timing_*`](../rtc_base/include/rtc_base/timing/) generic infra (`ThreadTimingProducer<Payload, N>` + `ThreadTimingCsvLogger<Payload>`)를 직접 owning. 새 채널을 추가해도 base 변경이 필요 없도록 의도된 구조 — 자세한 사용 예는 [`rtc_mpc/logging/mpc_timing_logger.hpp`](../rtc_mpc/include/rtc_mpc/logging/mpc_timing_logger.hpp), [`rtc_base/timing/rt_tick_timing_sample.hpp`](../rtc_base/include/rtc_base/timing/rt_tick_timing_sample.hpp).

### Lifecycle 훅 (ros2_control 정렬, 기본 구현 제공)

시그니처는 `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface`와 동형이며, 향후 `RTControllerInterface`를 `LifecycleNode` 상속형으로 전환할 때 시그니처 변경 없이 이동 가능합니다. 모든 훅은 `noexcept`입니다.

| 훅 | 시그니처 | 기본 동작 |
|---|---|---|
| `PreConfigure` | `(LifecycleNode::SharedPtr, YAML::Node) → CallbackReturn` | CM 3-pass bring-up의 1단계. `node_` 저장 + `LoadConfig(yaml_cfg)` try/catch → `topic_config_` 채움. **RegisterLog / 리소스 할당 금지** — CM이 이 결과로 `active_groups_` 와 `device_name_configs_` 를 빌드한 뒤 `SetDeviceNameConfigs()` 를 호출하고, 이어서 `on_configure()` 에서 본격 작업이 일어남 |
| `on_configure` | `(State, LifecycleNode::SharedPtr, YAML::Node) → CallbackReturn` | `PreConfigure` 경유 시 멱등 (이미 set된 `node_`/`topic_config_` 보존, LoadConfig 재호출 안 함). 직접 호출 (legacy 단위 테스트) 시 `node_` 저장 + LoadConfig. 서브클래스가 `RegisterLog<>(...)` / 파라미터 / 퍼블리셔를 만드는 시점이며, 이때 `GetDeviceNameConfig(...)` 결과는 이미 채워져 있어 헤더 writer 에 안전히 전달 가능. 실패 경로 로그는 `rclcpp::get_logger("rtc_controller_interface")` 정적 logger 사용 + 메시지 본문에 `[<controller_name>]` prefix — 네이밍 규약은 [agent_docs/conventions.md](../agent_docs/conventions.md) "Logging" 섹션 참조 |
| `on_activate` | `(State) → CallbackReturn` | activation generation 증분 (비활성 구간에 큐잉된 target 무효화) + `ResetTargetInitialization()` 호출 → `SUCCESS`. 아래 "Activation generation gate" 참조 |
| `on_deactivate` | `(State) → CallbackReturn` | no-op `SUCCESS` |
| `on_cleanup` | `(State) → CallbackReturn` | `node_.reset()` → `SUCCESS` |
| `on_shutdown` | `(State) → CallbackReturn` | `on_cleanup(state)` 위임 |
| `on_error` | `(State) → CallbackReturn` | no-op `SUCCESS` (서브클래스에서 E-STOP 트리거 등 원하는 복구 로직 override) |

> **3-pass bring-up 계약**: CM은 (1) `PreConfigure(node, yaml)` → (2) 모든 컨트롤러의 `topic_config_` 으로 `active_groups_` 빌드 + `LoadDeviceNameConfigs()` + 컨트롤러별 `SetDeviceNameConfigs(...)` → (3) `on_configure(state, node, yaml)` 순서로 호출합니다. RegisterLog 람다가 `joint_state_names` / `motor_state_names` 등 device-name 정보를 capture할 때, `OnDeviceConfigsSet` 이 이미 실행됐음이 보장됩니다. 단위 테스트에서 `on_configure`를 직접 호출하는 legacy 경로는 `node_ == nullptr` 가드로 분기하여 종전 동작을 유지합니다.
>
> **Override 규약**: 서브클래스가 `on_configure`를 오버라이드해 자체 sub/pub을 만들 때 반드시 `RTControllerInterface::on_configure(previous_state, node, yaml_cfg)`를 먼저 호출한 뒤 `node_->create_subscription(...)` / `node_->create_publisher(...)`로 확장합니다. `PreConfigure` 는 base 전용이므로 override 불필요.
>
> RT 경로(`Compute`, `SetDeviceTarget` 등)에서는 **절대 `node_`에 접근하지 않습니다** — ROS2 API 호출은 non-RT 훅에서만 수행하고, RT 경로에서는 사전 할당된 SeqLock/SPSC 버퍼를 통해 값을 읽습니다.

### LifecycleNode 접근자

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `get_lifecycle_node()` | public const noexcept | `on_configure` 성공 후 non-null. `RtControllerNode`가 `nrt_callback_executor`에 attach하기 위해 사용. `on_cleanup` 후 null |

멤버 `node_`는 `protected`로 노출되어 서브클래스가 `node_->create_subscription(...)` 형태로 직접 ROS I/O를 소유합니다. 이 네이밍 규약(`node_->`)은 향후 상속 전환 시 `this->`로 기계적 치환이 가능하도록 의도된 것입니다.

### 디바이스 설정 메서드

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `SetDeviceNameConfigs(map)` | public | 디바이스 이름/설정 맵 저장 후 `OnDeviceConfigsSet()` 호출 |
| `GetDeviceNameConfig(name)` | public const | 디바이스 이름으로 설정 조회. 미등록 시 `nullptr` 반환 |
| `GetSensorLayout(name)` | public const | `GetDeviceNameConfig(name)->sensor_layout` 의 편의 wrapper. 디바이스 미등록 또는 `sensor_layout` 블록 부재 시 `std::nullopt`. 컨트롤러가 `OnDeviceConfigsSet` 에서 1회 호출 후 capability bool 등 멤버에 cache 하는 패턴 권장 |
| `OnDeviceConfigsSet()` | protected virtual | 하위 클래스 오버라이드 포인트 (예: URDF 기구학 해석) |
| `GetPrimaryDeviceName()` | public const | 토픽 설정의 첫 번째 디바이스 이름 반환 (하드코딩 방지용) |
| `GetSecondaryDeviceName()` | public const | 토픽 설정의 두 번째 디바이스 이름 반환. 단일-디바이스 컨트롤러에선 빈 문자열 — 호출자는 `GetDeviceNameConfig(...)` 결과를 null-check |

### 시스템 모델 설정 메서드

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `SetSystemModelConfig(config)` | public | 시스템 레벨 `ModelConfig`를 복사 저장 후 `OnSystemModelConfigSet()` 호출 |
| `GetSystemModelConfig()` | public const noexcept | 저장된 `ModelConfig` 포인터 반환. 미설정 시 `nullptr` |
| `OnSystemModelConfigSet()` | protected virtual | 하위 클래스 오버라이드 포인트 (예: arm sub-model 구축) |
| `SetSharedModelBuilder(builder)` | public noexcept | `RtControllerNode`가 시스템 URDF로 한 번 빌드한 `std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder>` 를 컨트롤러에 주입 |
| `GetSharedModelBuilder()` | public const noexcept | 주입된 공유 빌더 반환. 미주입/실패 시 `nullptr` — 컨트롤러는 직접 빌드로 폴백 |

> **`SetSystemModelConfig()`** 은 `RtControllerNode`가 컨트롤러 인스턴스 생성 직후, `LoadConfig()` 호출 이전에 실행합니다. 따라서 `LoadConfig()` 내에서 `GetSystemModelConfig()`로 시스템 URDF 경로, sub_models, tree_models, passive_joints 정보에 접근할 수 있습니다.
>
> **`SetSharedModelBuilder()`** 은 `SetSystemModelConfig()` 직후 같은 단계에서 호출됩니다. 컨트롤러의 model 초기화 (`InitArmModel` / `InitModels`) 는
>
> ```cpp
> if (auto shared = GetSharedModelBuilder()) {
>   builder_ = std::move(shared);
> } else {
>   builder_ = std::make_shared<rub::PinocchioModelBuilder>(config);
> }
> ```
>
> 패턴을 따르며, 동일한 `system_model_config_` 로 N개의 컨트롤러가 각각 URDF를 다시 파싱하는 비용을 제거합니다 (xacro→tinyxml2→Pinocchio full+sub+tree 모델까지). 데모 빌업 기준 이전 4회 → 현재 1회.

### 제어 주기 설정

| 메서드 | 설명 |
|--------|------|
| `SetControlRate(double hz)` | 제어 루프 주파수 설정 (매니저가 초기화 시 호출) |
| `GetDefaultDt()` | `1.0 / control_rate_` 반환. `control_rate_`이 0 이하이면 0.002 반환 |

### 토픽 설정 접근

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `GetTopicConfig()` | public const | 컨트롤러별 토픽 라우팅 설정 반환 |
| `ParseTopicConfig(YAML::Node)` | protected static | YAML `topics:` 하위의 모든 디바이스 그룹을 동적 파싱 |

### 보호 멤버 변수

```cpp
TopicConfig topic_config_;                                      // 기본값: 빈 TopicConfig (LoadConfig가 YAML topics: 섹션으로 채움)
std::map<std::string, DeviceNameConfig> device_name_configs_;   // SetDeviceNameConfigs()에서 설정
std::unique_ptr<rtc_urdf_bridge::ModelConfig> system_model_config_;  // SetSystemModelConfig()에서 설정
std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> shared_model_builder_;  // SetSharedModelBuilder()에서 설정 (CM이 한 번 빌드 후 모든 컨트롤러에 공유)
double control_rate_{500.0};                                    // SetControlRate()에서 설정
```

---

## ControllerRegistry (싱글톤 레지스트리)

Meyer's 싱글톤 패턴 기반의 컨트롤러 레지스트리입니다. 정적 초기화 시점(`main()` 이전)에 `Register()`가 호출되고, 노드 시작 시 `GetEntries()`로 등록된 컨트롤러를 조회합니다.

### ControllerEntry 구조체

| 필드 | 타입 | 설명 |
|------|------|------|
| `config_key` | `std::string` | YAML 파일명 스템 (예: `"p_controller"`) |
| `config_subdir` | `std::string` | 설정 하위 디렉토리 (예: `"direct/"`, `"indirect/"`) |
| `config_package` | `std::string` | 설정 YAML을 소유하는 ament 패키지명 |
| `factory` | `function<unique_ptr<RTControllerInterface>(const string&)>` | URDF 경로를 받아 컨트롤러 인스턴스를 생성하는 팩토리 |

### API

| 메서드 | 설명 |
|--------|------|
| `Instance()` | 싱글톤 인스턴스 반환 (`noexcept`, static) |
| `Register(ControllerEntry)` | 컨트롤러 엔트리를 `entries_` 벡터에 추가. 동일 `config_key` 중복 등록 시 `RCLCPP_WARN` 만 하고 throw 하지 않는다 (static-init 순서가 fragile). **단 shadowing 이 실제로 일어나지는 않는다** — CM 이 bring-up 진입 시 registry 를 스캔해 중복 `config_key` 를 발견하면 configure 자체를 거부한다 (issue #196 Phase 5, `rt_controller_node_params.cpp`). 즉 warn 은 registry 계층의 한계이고, 강제는 CM 계층에서 한다 |
| `GetEntries()` | 등록된 컨트롤러 목록 반환 (`noexcept`, const 참조) |

---

## RTC_REGISTER_CONTROLLER 매크로

컨트롤러를 정적 초기화 시점에 자동 등록하는 매크로입니다. `controller_registry.hpp`에 정의되어 있습니다.

### 사용법

```cpp
RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, FactoryExpr)
```

| 파라미터 | 타입 | 예시 |
|---------|------|------|
| `config_key` | 식별자 (따옴표 없음) | `demo_joint_controller` |
| `config_subdir` | 문자열 리터럴 | `""` (production 은 flat) |
| `config_package` | 문자열 리터럴 | `"integrated_bringup"` |
| `FactoryExpr` | 코드 표현식 | `std::make_unique<DemoJointController>(urdf)` |

> `urdf` 변수는 매크로가 생성하는 람다의 파라미터 `(const std::string& urdf)`로 자동 제공됩니다.
>
> **등록 대상은 항상 downstream 의 클래스다.** `config_package` 에 `rtc_*` 패키지 이름을 넣는 형태는 ARCH-1 위반이며, 등록되는 클래스 자체도 integration 패키지가 소유하는 바인딩이어야 한다 ([agent_docs/design-principles.md](../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms). `rtc_controllers` 에는 아직 `RTControllerInterface` 를 상속한 어댑터가 남아 있어 기술적으로 등록 가능하지만, 그 어댑터들은 #236 S1–S7 에서 삭제되므로 새 등록의 대상으로 삼지 않는다.

### 매크로 전개 예시

```cpp
// 입력 (integrated_bringup/src/controllers/controller_registration.cpp)
RTC_REGISTER_CONTROLLER(demo_joint_controller, "", "integrated_bringup",
    std::make_unique<DemoJointController>(urdf))

// 전개 결과
namespace {
  [[maybe_unused]] const bool rtc_reg_demo_joint_controller = [] {
    ::rtc::ControllerRegistry::Instance().Register({
      "demo_joint_controller",  // config_key (문자열화)
      "",                       // config_subdir
      "integrated_bringup",     // config_package
      [](const std::string& urdf) {  // factory 람다
        return std::make_unique<DemoJointController>(urdf);
      }
    });
    return true;
  }();  // IIFE - main() 이전 실행
}
```

### 설정 파일 경로 규칙

```
<config_package>/config/<config_subdir><config_key>.yaml
```

예: `integrated_bringup/config/<robot>/controllers/demo_joint_controller.yaml`
(`<config_subdir>` 는 `rtc_controllers` 의 example 레이아웃처럼 `indirect/` 를 낄 수 있으나,
production 배치는 controller 당 한 파일의 flat 형태다 — [agent_docs/controllers.md](../agent_docs/controllers.md))
(production YAML 은 robot-specific bringup 패키지가 소유합니다 — ARCH-1.
`rtc_controllers/examples/controllers/` 의 동일 이름 파일은 reference
example 이며 직접 로드되지 않습니다 — `<robot>` placeholder 를 자신의
device-group 키로 치환해 복제하세요)

### 정적 라이브러리 링커 스트립 방지

정적 라이브러리에 포함된 컨트롤러는 외부 참조가 없으면 링커가 제거할 수 있습니다. 해결 방법:

```cpp
// 방법 1: Force 함수 정의 후 main()에서 호출
//   실제 사용처: integrated_bringup/src/integrated_rt_controller_main.cpp 가
//   rtc::ForceBuiltinControllerRegistration() 를 호출한다 (현재 no-op — 등록은
//   bringup 자신의 TU 에 있고, 링크 호환성을 위해 심볼만 남아 있다)
namespace rtc { void ForceBuiltinControllerRegistration() {} }

// 방법 2: --whole-archive 링커 플래그 — 등록 TU 를 담은 정적 라이브러리에 건다
target_link_libraries(my_exe
  PRIVATE -Wl,--whole-archive <registration_lib> -Wl,--no-whole-archive)
```

---

## 토픽 설정 시스템

각 컨트롤러는 YAML을 통해 디바이스별 ROS2 토픽 라우팅을 설정할 수 있습니다. `TopicConfig`는 `std::vector<std::pair<std::string, DeviceTopicGroup>>` 구조로, YAML 삽입 순서를 보존합니다.

### YAML 형식

```yaml
my_controller:
  topics:
    ur5e:
      subscribe:
        - topic: "ur5e/joint_goal"     # relative path → /<config_key>/ur5e/joint_goal
          role: "target"
      publish:
        - topic: "transforms"          # relative path → /<config_key>/transforms
          role: "robot_transforms"
    hand:
      subscribe:
        - topic: "hand/joint_goal"
          role: "target"
```

> 폐기된 flat format (`topics.subscribe` / `topics.publish` 를 직접 사용)은 마이그레이션 에러를 발생시킵니다.

**형식 검증은 fail-closed 입니다 (issue #196 Phase 5).** 원칙은 하나 — *"선언은 돼 있는데 아무것도 라우팅하지 않는 상태"* 로는 기동할 수 없다. 아래 형태들은 전부 조용히 건너뛰어져 컨트롤러가 target 구독 0개로, 아무 진단 없이 올라오던 것들입니다. 이제 `ParseTopicConfig` 가 throw 하고 Phase 1 의 fail-closed lifecycle 이 이를 configure 실패로 잇습니다.

| 잘못된 형태 | 전형적 원인 |
|---|---|
| `subscribe:` / `publish:` 값이 `{topic, role}` **sequence 가 아님** | 들여쓰기 실수로 `- ` 가 빠져 map 이 됨 |
| `subscribe:` / `publish:` 가 **빈 sequence** (`[]`) | 엔트리를 지우고 lane 키만 남김 |
| group 값이 **sequence** | `subscribe:` 줄 자체를 빠뜨리고 엔트리를 group 밑에 직접 씀 |
| group 값이 **null** (`ur5e:` 밑이 비어 있음) | 마이그레이션 중 lane 을 통째로 주석 처리 |
| group map 에 `subscribe` / `publish` 가 **둘 다 없음** | 빈 map (`ur5e: {}`), 자리표시자 |
| group map 에 **lane 이 아닌 키** | 키 오타 (`subscibe:`) — 진단이 그 키 이름을 지목 |
| 최상위 `topics:` 가 **map 이 아님** (null / 스칼라 / sequence) | 섹션 본문을 통째로 주석 처리 |
| 최상위에 `subscribe:` 또는 `publish:` | 폐기된 flat format |

진단 메시지는 **group 이름과 lane 이름을 모두** 포함하므로 (`Topic group 'hand': 'subscribe' must be ...`) 다중 group config 에서 어느 블록이 깨졌는지 바로 찾을 수 있습니다.

키 오타가 lane 둘 다를 망가뜨렸을 때만 잡히면 부족합니다 — 흔한 쪽은 **오타 하나 + 멀쩡한 lane 하나** 이고, 이 경우 group 은 "lane 이 있으므로" 통과하면서 target lane 만 조용히 사라집니다. 그래서 group map 은 화이트리스트로 검증합니다 (lane 이 아닌 키는 이름을 지목하며 거부).

단 `topics:` 아래에서 값이 **스칼라** 인 키는 계속 group 이 아닌 것으로 보고 건너뜁니다 (스칼라 설정값을 같은 섹션에 둘 수 있도록). **엔트리** 수준의 미지 키(`data_size:` 등)도 무시됩니다 — group map 화이트리스트는 group 단위에만 적용되며, 엔트리는 동작을 바꾸지 않으므로 거부해도 안전 이득이 없습니다. 엄격히 검증되는 것은 무엇이 생성될지 결정하는 `role:` 문자열입니다. `topics:` 섹션을 **아예 생략**하는 것은 정상입니다 (토픽을 선언하지 않는 컨트롤러) — 잘못 선언하는 것만 거부됩니다.

### 토픽 소유권 (issue #138)

컨트롤러 YAML `topics:` 의 모든 subscribe/publish entry 는 **controller-owned** 입니다 — `ownership` 필드는 없습니다. 컨트롤러별 `LifecycleNode` 가 `on_configure` (via `integrated_bringup/src/support/owned_topics.cpp`) 에서 `node_->create_subscription(...)` / `node_->create_publisher(...)` 로 직접 생성하며, 상대 경로는 노드 namespace `/<config_key>/...` 로 자동 해석됩니다. CM 의 publish thread 는 SPSC snapshot 을 드레인한 뒤 `controllers_[active]->PublishNonRtSnapshot(snap)` 을 호출해 controller-owned 발행을 위임합니다.

컨트롤러 YAML 밖의 두 lane 은 별도 소유입니다: device-wire state/command 는 `devices.<group>.backend:` (DeviceBackend-owned), CM 고정 퍼블리셔 (`/rtc_cm/<group>/joint_states`, `/system/estop_status`, `/rtc_cm/active_controller_name`) 는 `RtControllerNode` 가 hardcode 로 소유 (YAML 무관). Phase 4 이전의 manager/controller 2-tier 선택자 (`TopicOwnership` enum) 는 issue #138 에서 제거되었습니다.

### 구독 역할 (`role:` 문자열)

Phase 4: 디바이스 와이어 lane (`state` / `motor_state` / `sensor_state`) 은 `devices.<group>.backend:` 로 이관되어 `DeviceBackend` 가 소유합니다. 컨트롤러 YAML 의 `topics:` 섹션에서는 `target` 만 남습니다.

Phase 4 trailing cleanup: 남은 값이 singleton 이라 `SubscribeRole` enum 자체는 삭제되었습니다. `SubscribeTopicEntry` 는 `{topic_name}` 으로 단순화 되었고 (issue #138: `ownership` 필드 제거), parser 는 YAML `role:` 문자열을 documentation + drift 검출용으로 여전히 validate (`target` / 호환용 `goal` 외 거부) 합니다.

| YAML 역할 문자열 | 설명 |
|-----------------|------|
| `target` | 외부 목표 (RobotTarget) |
| `goal` | 하위 호환 별칭 |

### 퍼블리시 역할 (PublishRole)

Phase 4 (`b9a2587`): 디바이스 와이어 command lane (`joint_command` / `ros2_command`) 은 `devices.<group>.backend:` 로 이관되어 `DeviceBackend` 가 소유합니다.
Phase 4 trailing cleanup (`104796f`): 컨트롤러 소유 non-RT 토픽 (`grasp_state` / `wbc_state` / `tof_snapshot`) 은 enum/parser/snapshot 슬롯에서 모두 제거되고 각 컨트롤러가 직접 `SeqLock<{Grasp,Wbc,ToF}StateData>` + `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼 (`integrated_bringup/include/integrated_bringup/support/owned_topics.hpp`) 로 소유합니다 — 새 controller-owned 토픽을 추가할 때 이 패턴을 따르고 `PublishRole` 은 *건드리지 마십시오*. 컨트롤러 YAML `topics:` 섹션에는 아래 역할만 남습니다.

Phase 5 (issue #196): `robot_target` / `digital_twin_state` / 하위 호환 별칭 `joint_goal` 이 제거됐습니다. 파서는 이 문자열들을 `kRobotTarget` / `kDigitalTwinState` 로 매핑했지만 그 enum 값으로 publisher 를 만드는 소비자가 없어, 선언한 컨트롤러는 에러 없이 죽은 토픽을 얻었습니다. 이제 이 문자열들은 다른 오타와 동일하게 `Unknown publish role` 로 configure 를 실패시킵니다.

| YAML 역할 문자열 | enum 값 | 설명 |
|-----------------|---------|------|
| `robot_transforms` | `kRobotTransforms` | Per-controller TFMessage — controller당 1 토픽 (`<config_key>/transforms`)에 arm tip / hand fingertips / virtual TCP frame을 묶어 발행 (`tf2_msgs/TFMessage`) |

### 기본 토픽 설정

`topic_config_` 는 기본 생성자에서 비어 있고, `LoadConfig()` 가 YAML `topics:` 섹션을 만나면 그 내용으로 채웁니다. 섹션이 없으면 비어 있는 채로 유지됩니다 (rtc_* 는 robot-agnostic — 기본 device 이름을 박지 않습니다). 디바이스 와이어 토픽은 `<robot>_bringup/config/<robot>/{sim,robot}.yaml` 의 `devices.<group>.backend:` 블록에서 별도로 선언합니다.

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rtc_base` | 공유 데이터 타입 (`ControllerState`, `ControllerOutput`, `TopicConfig`, `DeviceNameConfig` 등) |
| `rtc_msgs` | 커스텀 ROS2 메시지 |
| `rtc_urdf_bridge` | URDF→Pinocchio 모델 빌더 + `ModelConfig` 타입 (시스템 모델 설정) |
| `pinocchio` | 로보틱스 기구학/동역학 (하위 패키지에 전이적 제공) |
| `rclcpp` | ROS2 클라이언트 라이브러리 |
| `rclcpp_lifecycle` | `LifecycleNodeInterface` 동형 훅 (`on_configure` 등) + `LifecycleNode::SharedPtr` 타입 |
| `yaml-cpp` | YAML 설정 파싱 |

테스트 의존성: 개별 lint (`ament_cmake_cppcheck`, `ament_cmake_lint_cmake`, `ament_cmake_xmllint`). `ament_lint_common` meta + `ament_uncrustify` 는 워크스페이스 정책 (`bdedac7`) 으로 사용 금지 — 자세한 사유는 [agent_docs/conventions.md](../agent_docs/conventions.md).

---

## 라이선스

MIT License
