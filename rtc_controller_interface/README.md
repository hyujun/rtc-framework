# rtc_controller_interface

컨트롤러 추상 인터페이스 `RTControllerInterface` 와 플러그인 레지스트리 `ControllerRegistry`. **유형**: 라이브러리 패키지

## 개요

`librtc_controller_interface.so` 를 제공한다. Controller Manager (CM) 는 이 인터페이스만 보고 컨트롤러를 구동하고, 컨트롤러는 정적 초기화 시점에 `RTC_REGISTER_CONTROLLER` 로 레지스트리에 스스로 등록한다.

구현하는 쪽은 제어 *알고리즘* 이 아니라 **바인딩**이다 — integration 패키지가 소유하며 프레임워크 계약을 알고리즘에 이어 붙이는 클래스다. `rtc_controllers` 의 제어 법칙은 이 인터페이스를 몰라야 한다 (3계층 배치: [design-principles.md](../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms). 이 패키지는 그중 base 계층으로, 모든 바인딩이 필요로 하는 글루 (target mailbox · device 한계값 · 판독가능성 게이트 · E-STOP scaffolding) 를 갖는다.

소비자: [rtc_controller_manager](../rtc_controller_manager/README.md) (구동) · [integrated_bringup](../integrated_bringup/README.md) (바인딩 소유).

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 소비하는 패키지의 `CMakeLists.txt`:

```cmake
find_package(rtc_controller_interface REQUIRED)
ament_target_dependencies(my_bindings rtc_controller_interface)
```

바인딩 하나는 세 조각이다. 실제 예는 `integrated_bringup/src/controllers/joint/` 와 `controller_registration.cpp`.

```cpp
// 1. 클래스 — 순수 가상 3개를 구현한다 (전부 noexcept)
class MyBinding : public rtc::RTControllerInterface {
 public:
  explicit MyBinding(const std::string& urdf_path);
  rtc::ControllerOutput Compute(const rtc::ControllerState& state) noexcept override;
  void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept override {
    PushPendingTarget(device_idx, target, /*is_task=*/false);  // off-RT -> RT mailbox
  }
  std::string_view Name() const noexcept override { return "my_binding"; }
 protected:
  void ApplyPendingTarget(int, std::span<const double>, bool) noexcept override;  // Compute() 가 DrainPendingTargets() 로 호출
};

// 2. 등록 — .cpp 한 곳에서 (정적 초기화)
RTC_REGISTER_CONTROLLER(my_binding, "", "my_bringup", std::make_unique<MyBinding>(urdf))
```

```yaml
# 3. <my_bringup>/config/<robot>/controllers/my_binding.yaml
topics:
  arm:
    subscribe:
      - {topic: "arm/joint_goal", role: "target"}
```

## 인터페이스

| 헤더 | 내용 |
|---|---|
| `rt_controller_interface.hpp` | `RTControllerInterface` · `CheckTargetLimits` · `ValidateControllerOutput` |
| `controller_registry.hpp` | `ControllerRegistry` · `ControllerEntry` · `RTC_REGISTER_CONTROLLER[_REQUIRING_CONFIG]` |
| `device_readability.hpp` | F5 device-readability 게이트 술어·원시연산 (헤더 전용, `rt_controller_interface.hpp` 가 재수출) |
| `controller_log_set.hpp` | opt-in CSV 로그 집합 (`ControllerLogSet` · `LogHandle<PodT>`) |
| `controller_types.hpp` | `rtc_base/types/types.hpp` 재수출 |

`RTControllerInterface` 는 복사·이동이 금지된 추상 클래스이고 소멸자는 `virtual` 이다 — CM 이 base 포인터로 소유·파괴하므로 비-virtual 이면 derived 멤버 cleanup 이 조용히 빠진다. 시그니처·`noexcept`·RT 제약의 정본은 헤더 Doxygen 이다.

### 순수 가상 메서드

| 메서드 | 설명 |
|---|---|
| `Compute(state)` | RT tick 의 제어 연산. 첫 호출에서 컨트롤러가 target slot 을 현재 device state 로 직접 seed 한다 (base 에 hold-init 훅은 없다 — RT-4) |
| `SetDeviceTarget(idx, target)` | 목표 설정. off-RT 콜백에서 불리므로 mailbox (`PushPendingTarget`) 로 marshal 한 뒤 `Compute()` 에서 drain 한다 |
| `Name()` | 사람이 읽는 컨트롤러 이름 |

### 기준 frame 과 goal 종류

`SetDeviceTaskTarget(idx, task6)` 는 6-DoF 태스크 공간 목표 `(x, y, z, r, p, y)` 슬롯이다. 순수 가상이 아니라 기본 구현이 `SetDeviceTarget` 으로 forwarding 한다. joint 슬롯과 SE3 슬롯을 따로 두는 컨트롤러만 override 한다.

| 메서드 | 설명 |
|---|---|
| `SetDeviceTaskTarget(idx, task6, frame_id)` | 기준 frame 을 받는다. `nullptr` 반환 = 수락, 문자열 리터럴 = 거부 사유. 기본 구현은 frame 을 읽지 않고 2-인자로 넘긴다 |
| `AcceptedGoalKinds(idx)` | device 가 받는 goal 종류 (`{joint, task}`, 기본은 둘 다). 받지 않는 종류는 거부·count 한다 |

3-인자만 override 하면 2-인자가 가려지므로 `using RTControllerInterface::SetDeviceTaskTarget;` 를 함께 둔다. 거부는 `GetTargetRejectCount()` 에 세어지고 `/rtc_cm/list_controllers` 의 `target_reject_count` 로 나간다.

`DeliverTargetMessage()` 는 `RobotTarget.goal_type` 으로 분기한다.

| `goal_type` | 소스 | 디스패치 |
|---|---|---|
| `"task"` | `task_target` | `SetDeviceTaskTarget(idx, task, frame_id)` — 거부면 count |
| 그 외 (joint) | `joint_target` | `joint_names` 를 `joint_state_names` 순서로 재정렬한 뒤 `SetDeviceTarget` |

joint 목표는 디스패치 직전에 `joint_limits` 와 대조해 벗어나면 throttled WARN 을 낸다 (warn-only — 강제는 각 컨트롤러의 RT clamp). `joint_limits` 가 없는 device 는 침묵한다. 판정은 순수 함수 `CheckTargetLimits` 이다.

### Target mailbox

off-RT ingress → RT drain marshal 은 이 패키지 소유다. 큐는 depth 4 (usable 3) 의 SPSC, 포화 시 **newest-drop** 이다. `SeqLock<TargetSlot>` · self-init seed · 슬롯 구조는 derived 소유다.

| 요소 | 설명 |
|---|---|
| `PushPendingTarget(idx, values, is_task)` | off-RT producer, 단일 producer. 세대 스탬프 + bound 검사 + enqueue |
| `DrainPendingTargets()` | RT consumer. 무효 세대·범위 밖 entry 를 버리고 생존분마다 `ApplyPendingTarget` 호출, 적용 수 반환 |
| `DiscardPendingTargets()` | RT consumer, apply 없이 비우기. E-STOP · device-invalid 경로용. 호출 여부는 derived 가 정한다 |
| `ApplyPendingTarget(...)` | virtual. `TargetSlot` 의미를 derived 가 소유. 기본은 `GetTargetUnhandledCount()` 만 올린다 |
| `GetTargetDropCount()` / `GetTargetRejectCount()` / `GetTargetUnhandledCount()` | 큐 포화 유실 / 형식 오류·frame 거부 / override 누락. 모두 monotonic. CM 이 `list_controllers` 로 노출하므로 컨트롤러는 override 하지 않는다 |

`rclcpp_lifecycle` 은 publisher 만 gating 하므로 target subscription 은 Inactive 구간에도 살아 있다. 그 구간에 쌓인 goal 이 재활성화 첫 tick 에 hold 를 덮지 않도록 **activation generation** 을 쓴다 (`ActivationGeneration()` · `IsCurrentGeneration(gen)`). `on_activate` 마다 +1, push 가 세대를 찍고 drain 이 현재 세대가 아닌 entry 를 버린다 (SPSC consumer 는 RT 하나뿐이라 lifecycle thread 가 큐를 비울 수 없다). 재-seed latch 는 `ResetTargetInitialization()` override 에서 내린다.

### Actuator-boundary output validation

`ControllerOutput` 은 out-of-tree 계약이라 CM 은 `WriteCommand` 직전에 `rtc::ValidateControllerOutput(out, state)` 로 검증한다. 순수 함수이고 RT-safe 다. 거부 사유 (`OutputRejectReason`):

| 사유 | 규칙 |
|---|---|
| `kInvalidFlag` | `valid == false` |
| `kDeviceCountMismatch` | `num_devices` 가 state 와 다르거나 `[0, kMaxDevices]` 밖 |
| `kChannelCountOutOfRange` | `num_channels` 가 `[0, kMaxDeviceChannels]` 밖이거나 device 가 보고한 수 초과 |
| `kNonFiniteCommand` | `commands` · `feedforward` 의 NaN/Inf (`command_type` 과 무관하게 항상 검사) |

검사 대상은 액추에이터에 닿는 `commands` / `feedforward` 뿐이다. 거부되면 CM 이 hold 로 대체하고 (write 는 건너뛰지 않는다), 연속 N tick 거부면 global E-STOP 으로 승격하며 `N = max(10, lround(0.1 × control_rate))` 이다. hold 를 만들 수 없으면 즉시 승격한다. 정책 전문과 `kTorque` hold 의 노출 창은 [rtc_controller_manager/README.md](../rtc_controller_manager/README.md) 가 갖는다.

### F5 device-readability 게이트

`devices[0]` 이 이번 tick 에 쓸 수 없는 상태 (`!valid` · 폭 부족 · 안 써진 슬롯) 에서 미보고 채널이 유한한 `0` 으로 읽히면 제어 법칙이 부분 ZERO configuration 에서 돌아 전 관절을 원점으로 당긴다. 모든 값이 유한하므로 위 validator 는 이를 잡지 못한다. **계약 SSoT 는 [compliance-conventions.md](../rtc_controllers/docs/compliance-conventions.md) §3.7 이다.** 아래는 `device_readability.hpp` 가 노출하는 이름의 색인이다.

| 이름 | 용도 |
|---|---|
| `ModelChannelBound` | 인덱싱 상한 (OOB 방어). 게이트가 아니다 |
| `IsDeviceReadable` | **게이트** — positions 를 이번 tick 에 써도 되는가 (보고됨 · 폭 충분 · 구멍 없음) |
| `IsGateClosedByWidth` · `IsGateClosedByHoles` · `FirstHoleSlot` | 게이트가 닫힌 이유 (폭 / 구멍 축, 배타적) — 진단 전용 |
| `IsSlotFresh` · `IsLaneReadable` | per-slot, `velocities`/`efforts` lane 판정. 게이트는 lane 을 보지 않으므로 그 값을 읽으면 따로 묻는다 |
| `SelfReportedChannelBound` | 선언된 `model_dim` 이 없을 때 device 자기 보고에서 폭을 채택하는 값 |
| `SilenceDeviceOutput` · `HoldTelemetryAtMeasured` · `FillCommandTail` | 닫힌 게이트의 출력 처리 (zero-length / 로그 lane 측정값 / tail 중립값). 앞의 둘은 항상 짝 |

`min(nc0, nv)` bound 는 crash 만 없애고 hazard 는 남긴다. 침묵 (zero-length) 은 정지가 아니라 "no update" 다.

### 가상 메서드 (기본 구현 제공)

| 메서드 | 기본 | 설명 |
|---|---|---|
| `TriggerEstop()` · `ClearEstop()` · `IsEstopped()` · `SetHandEstop(bool)` | no-op / `false` | E-STOP 훅 |
| `ResetFault()` | no-op | controller-local fault latch 해제 요청. 비-RT, wait-free. 실제 해제는 다음 `Compute()` tick. latch 가 없을 때는 상태를 바꾸지 않아야 한다 |
| `HasLatchedFault()` | `false` | latch 가 올라가 있는가. `list_controllers` 의 `has_latched_fault` 로 노출 |
| `LoadConfig(const YAML::Node&)` | 토픽 파싱 | `noexcept` 아님. override 하면 base 를 **먼저** 호출한다 |
| `GetCommandType()` | `kPosition` | `kPosition` 또는 `kTorque` |
| `PublishNonRtSnapshot(snap)` | no-op | controller-owned non-RT 토픽 발행 훅. CM publish 스레드가 호출 |

`ResetFault()` 와 `ClearEstop()` 은 의도적으로 분리돼 있다 (E-8) — 전자는 컨트롤러 자신의 latch, 후자는 CM global E-STOP 이고 어느 쪽도 다른 쪽을 풀지 않는다. `TriggerEstop()` / `ClearEstop()` override 는 `IsGlobalEstopped()` 를 되읽지 말고 자기 atomic 에 store 한다. CM 이 두 방향 모두 global latch 가 `true` 인 동안 전파하므로 되읽으면 trigger 와 clear 를 구분할 수 없다.

### Lifecycle 훅 (ros2_control 정렬, 기본 구현 제공)

시그니처는 `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface` 와 동형이고 모든 훅은 `noexcept` 다.

| 훅 | 기본 동작 |
|---|---|
| `PreConfigure(node, yaml)` | 3-pass 의 1단계. `node_` 저장 + `LoadConfig(yaml)` → `topic_config_`. **RegisterLog / 리소스 할당 금지** |
| `on_configure(state, node, yaml)` | `PreConfigure` 경유 시 멱등. 직접 호출 (단위 테스트) 이면 `node_` 저장 + `LoadConfig`. 서브클래스가 `RegisterLog<>` · 파라미터 · 퍼블리셔를 만드는 시점 |
| `on_activate(state)` | activation generation +1 과 `ResetTargetInitialization()` 호출 → `SUCCESS` |
| `on_deactivate(state)` | no-op `SUCCESS` |
| `on_cleanup(state)` | `node_.reset()` + 상태를 `kUnconfigured` 로 → `SUCCESS` |
| `on_shutdown(state)` | `on_cleanup` 위임 |
| `on_error(state)` | no-op `SUCCESS` (E-STOP 등 복구는 서브클래스가 override) |

`GetConfigState()` 는 `kUnconfigured` → `PreConfigure` 성공 `kPreConfigured` → `on_configure` 성공 `kConfigured` 로 간다. 어느 단계든 실패하면 `kFailed` 이고 이후 `on_configure()` 는 계속 `FAILURE` 다 — 되돌리는 길은 `on_cleanup()` 뿐이다.

**3-pass bring-up 계약**: CM 은 (1) `PreConfigure(node, yaml)` → (2) 모든 컨트롤러의 `topic_config_` 로 `active_groups_` 를 빌드하고 `LoadDeviceNameConfigs()` + 컨트롤러별 `SetDeviceNameConfigs(...)` (→ `OnDeviceConfigsSet`) → (3) `on_configure(state, node, yaml)` 순서로 호출한다. 그래서 `on_configure` 의 `RegisterLog` 가 device-name 정보를 capture 할 때 `OnDeviceConfigsSet` 은 이미 실행돼 있다. configure-time 검증을 어느 pass 에 둘지는 [invariants.md](../agent_docs/invariants.md) AP-PROC-9 가 정한다.

**Override 규약**:
- `on_configure` / `on_activate` 등을 override 하면 base 를 **먼저** 호출한 뒤 `node_->create_subscription(...)` / `create_publisher(...)` 로 확장한다. `PreConfigure` 는 override 하지 않는다.
- RT 경로 (`Compute`, `SetDeviceTarget` 등) 에서는 **절대 `node_` 에 접근하지 않는다**. ROS2 API 는 non-RT 훅에서만 쓰고 RT 는 사전 할당된 SeqLock/SPSC 버퍼로 값을 읽는다.
- 실패 경로 로그는 `rclcpp::get_logger("rtc_controller_interface")` + 메시지 본문 `[<controller_name>]` prefix ([conventions.md](../agent_docs/conventions.md) Logging).

### 접근자와 주입

| 메서드 | 설명 |
|---|---|
| `get_lifecycle_node()` | `on_configure` 성공 후 non-null, `on_cleanup` 후 null. CM 이 `nrt_callback_executor` 에 attach 할 때 쓴다 |
| `SetDeviceNameConfigs(map)` → `OnDeviceConfigsSet()` | 디바이스 이름·설정 저장 후 호출되는 protected virtual (URDF 기구학 해석 등) |
| `GetDeviceNameConfig(name)` | 미등록이면 `nullptr` |
| `GetSensorLayout(name)` | `sensor_layout` 편의 wrapper. 없으면 `std::nullopt` — `OnDeviceConfigsSet` 에서 1회 읽어 cache 한다 |
| `GetPrimaryDeviceName()` · `GetSecondaryDeviceName()` | 토픽 설정의 첫·둘째 디바이스 이름 (단일 device 면 둘째는 빈 문자열) |
| `SetSystemModelConfig(cfg)` → `OnSystemModelConfigSet()` | CM 이 인스턴스 생성 직후 `LoadConfig` 전에 주입. `GetSystemModelConfig()` 는 미설정이면 `nullptr` |
| `SetSharedModelBuilder(b)` · `GetSharedModelBuilder()` | CM 이 한 번 빌드한 `PinocchioModelBuilder` 공유. 없으면 컨트롤러가 직접 빌드로 폴백 |
| `SetControlRate(hz)` · `GetDefaultDt()` | 제어 주기. `GetDefaultDt()` 는 `1/control_rate_`, rate 가 0 이하면 fallback 상수 |
| `GetTopicConfig()` · `ParseTopicConfig(node)` | 토픽 라우팅 조회 / YAML `topics:` 파싱 (protected static) |

### ControllerRegistry

Meyer's 싱글톤. 정적 초기화 때 `Register()` 가 불리고, 노드 시작 시 `GetEntries()` 로 조회한다.

| `ControllerEntry` 필드 | 설명 |
|---|---|
| `config_key` | YAML 파일명 stem. 컨트롤러 노드 이름·namespace 도 이것 |
| `config_subdir` | 설정 하위 디렉토리 (production 은 `""`) |
| `config_package` | 설정 YAML 을 소유하는 ament 패키지 |
| `factory` | URDF 경로를 받아 인스턴스를 만드는 callable |
| `config_required` | `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` 가 세운다 |

`Instance()` 는 `noexcept`. 같은 `config_key` 를 두 번 `Register()` 하면 WARN 만 하고 throw 하지 않는다 (static-init 순서가 fragile). 강제는 CM 이 한다 — bring-up 진입 시 중복을 발견하면 configure 를 거부한다.

### RTC_REGISTER_CONTROLLER

```cpp
RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, FactoryExpr)
```

| 인자 | 형태 | 예 |
|---|---|---|
| `config_key` | 식별자 (따옴표 없음) | `demo_joint_controller` |
| `config_subdir` | 문자열 리터럴 | `""` |
| `config_package` | 문자열 리터럴 | `"integrated_bringup"` |
| `FactoryExpr` | 표현식. `urdf` (`const std::string&`) 가 람다 인자로 제공된다 | `std::make_unique<DemoJointController>(urdf)` |

매크로는 익명 namespace 의 `static` 초기화 람다로 `ControllerRegistry::Instance().Register({...})` 를 부른다 (`main()` 이전). 등록 대상은 항상 downstream 의 클래스다 — `config_package` 에 `rtc_*` 패키지를 넣으면 ARCH-1 위반이다.

설정 파일 경로는 `<config_package>/config/<config_subdir><config_key>.yaml` 이다. 예: `integrated_bringup/config/<robot>/controllers/demo_joint_controller.yaml`. production YAML 은 robot-specific bringup 패키지가 소유하고 (ARCH-1), `rtc_controllers/examples/controllers/` 의 같은 이름 파일은 `<robot>` placeholder 를 쓰는 참고용이라 직접 로드되지 않는다 ([controllers.md](../agent_docs/controllers.md)).

등록 TU 를 담은 정적 라이브러리는 외부 참조가 없으면 링커가 제거하므로 실행 파일에 `--whole-archive` 로 건다 (`integrated_bringup/CMakeLists.txt` 의 `integrated_rt_controller` 참고).

```cmake
target_link_libraries(my_exe PRIVATE -Wl,--whole-archive <registration_lib> -Wl,--no-whole-archive)
```

### `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` — config 가 없으면 건너뛴다

인자는 위와 같고 `ControllerEntry::config_required = true` 를 세운다. 한 바이너리가 모든 `config_variant` 를 섬기고 등록은 `main()` 이전이라, 컨트롤러가 "이 로봇에서는 쓰지 않겠다" 를 선언할 방법이 없다. 이 매크로가 그 선언이다 — 해당 variant 에 YAML 이 **없으면** CM 이 인스턴스화를 건너뛴다.

**기본값(`RTC_REGISTER_CONTROLLER`)이 대부분 옳다.** 이 매크로는 기본값으로 돌 수 없는 컨트롤러에만 쓴다 — 모델 경로·IO 스키마 없이는 의미가 없는 정책 컨트롤러, 또는 그 로봇의 기구 구성에서 모델을 만들 수 없는 컨트롤러가 그렇다. 일반 매크로로 등록하면 `LoadConfig` 거부 → `PreConfigure` 실패로 **그 로봇의 모든 컨트롤러** bring-up 이 거부된다.

완화되는 것은 **파일 부재** 뿐이다. 파일이 있는데 최상위 키가 틀렸거나 파싱이 안 되면 종전대로 configure 를 거부한다. 양방향 게이트 테스트: `integrated_bringup/test/test_registered_controllers_have_shipped_config.cpp` (어느 프로파일도 안 싣는 컨트롤러는 이 매크로여야 하고, 이 매크로인 컨트롤러는 최소 한 프로파일이 실어야 한다) · `rtc_controller_manager/test/test_cm_required_config.cpp`.

## 설정

컨트롤러 YAML 은 `topics:` 섹션으로 디바이스 그룹별 토픽 라우팅을 선언한다. 섹션 **전체를 생략**하는 것은 정상 (토픽을 선언하지 않는 컨트롤러) 이고, 잘못 선언한 것만 거부된다. `topic_config_` 는 기본 생성자에서 비어 있어 기본 device 이름을 박지 않는다 (rtc_* 는 robot-agnostic). 디바이스 와이어 토픽은 `<robot>_bringup/config/<robot>/{sim,robot}.yaml` 의 `devices.<group>.backend:` 에서 따로 선언한다.

### YAML 형식

```yaml
my_controller:
  topics:
    ur5e:
      subscribe:
        - topic: "ur5e/joint_goal"     # 상대 경로 -> /<config_key>/ur5e/joint_goal
          role: "target"
      publish:
        - topic: "transforms"          # 상대 경로 -> /<config_key>/transforms
          role: "robot_transforms"
```

`TopicConfig` 는 YAML 삽입 순서를 보존한다. **형식 검증은 fail-closed** 다 — "선언은 돼 있는데 아무것도 라우팅하지 않는 상태" 로는 기동할 수 없다. `ParseTopicConfig` 가 throw 하고 configure 실패로 이어지는 형태:

| 잘못된 형태 | 전형적 원인 |
|---|---|
| `subscribe:` / `publish:` 값이 `{topic, role}` sequence 가 아님 | 들여쓰기 실수로 `- ` 가 빠져 map 이 됨 |
| `subscribe:` / `publish:` 가 빈 sequence (`[]`) | 엔트리를 지우고 lane 키만 남김 |
| group 값이 sequence | `subscribe:` 줄을 빠뜨리고 엔트리를 group 밑에 직접 씀 |
| group 값이 null | lane 을 통째로 주석 처리 |
| group map 에 `subscribe` / `publish` 가 둘 다 없음 | 빈 map (`ur5e: {}`) |
| group map 에 lane 이 아닌 키 | 키 오타 (`subscibe:`) — 진단이 그 키를 지목 |
| 최상위 `topics:` 가 map 이 아님 | 섹션 본문을 주석 처리 |
| 최상위에 `subscribe:` / `publish:` | 폐기된 flat format |
| 엔트리에 `role:` 누락 | 필수 키 |

진단 메시지는 group 과 lane 이름을 모두 담는다. 단 `topics:` 아래의 **스칼라** 값 키는 group 이 아닌 것으로 보고 건너뛰고, 엔트리 수준의 미지 키 (`data_size:` 등) 도 무시한다.

### 토픽 소유권

컨트롤러 YAML `topics:` 의 모든 subscribe/publish entry 는 **controller-owned** 이다 (`ownership` 필드는 없다). 컨트롤러별 `LifecycleNode` 가 `on_configure` 에서 (`integrated_bringup/src/support/owned_topics.cpp` 경유) `node_->create_subscription(...)` / `create_publisher(...)` 로 직접 만들고, 상대 경로는 노드 namespace `/<config_key>/...` 로 해석된다. CM 의 publish thread 는 SPSC snapshot 을 드레인한 뒤 `PublishNonRtSnapshot(snap)` 으로 발행을 위임한다.

컨트롤러 YAML 밖의 lane 은 별도 소유다. device-wire state/command 는 `devices.<group>.backend:` (DeviceBackend 소유), CM 고정 퍼블리셔 (`/rtc_cm/<group>/joint_states` · `/system/estop_status` · `/rtc_cm/active_controller_name`) 는 `RtControllerNode` 소유다.

### 구독 역할 (`role:` 문자열)

디바이스 와이어 lane (`state` / `motor_state` / `sensor_state`) 은 `DeviceBackend` 가 소유하므로 컨트롤러 YAML 에는 `target` 만 남는다. `SubscribeRole` enum 은 없고 parser 가 `role:` 문자열만 검증한다 (표 밖은 `Unknown subscribe role` 로 configure 실패).

| YAML 역할 문자열 | 설명 |
|---|---|
| `target` | 외부 목표 (`RobotTarget`) |
| `goal` | 하위 호환 별칭 |

### 퍼블리시 역할 (PublishRole)

디바이스 와이어 command lane (`joint_command` / `ros2_command`) 은 `DeviceBackend` 소유다. 컨트롤러 소유 non-RT 토픽 (`grasp_state` / `wbc_state` / `tof_snapshot`) 은 enum · parser 를 거치지 않고 각 컨트롤러가 `SeqLock<...StateData>` + `Setup...Publisher` 헬퍼 (`integrated_bringup/include/integrated_bringup/support/owned_topics.hpp`) 로 소유한다. 새 controller-owned 토픽은 이 패턴을 따르고 `PublishRole` 은 **건드리지 않는다**.

| YAML 역할 문자열 | enum 값 | 설명 |
|---|---|---|
| `robot_transforms` | `kRobotTransforms` | controller 당 1 토픽 (`<config_key>/transforms`) 에 arm tip · hand fingertips · virtual TCP frame 을 묶어 발행 (`tf2_msgs/TFMessage`) |

표 밖의 문자열 — 제거된 `robot_target` · `digital_twin_state` · `joint_goal` 포함 — 은 publisher 가 없어 `Unknown publish role` 로 configure 를 실패시킨다.

## 디렉토리

```
rtc_controller_interface/
├── include/
├── src/
└── test/
```

공개 헤더는 `include/`, 구현은 `src/`, gtest 는 `test/` 에 있다 (ROS 없는 순수 함수 테스트 포함).

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| configure 가 `Unknown publish role` / `Unknown subscribe role` 로 실패 | `role:` 문자열이 위 표에 없음 | §구독 역할 · §퍼블리시 역할 의 문자열로 고친다 |
| configure 가 `Topic group '<g>': ...` 로 실패 | `topics:` 형식 오류 | 위 fail-closed 표를 대조한다 |
| 한 컨트롤러 때문에 로봇의 모든 컨트롤러 bring-up 이 거부됨 | 그 로봇 설정으로 만들 수 없는 컨트롤러를 일반 매크로로 등록 | `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` 로 바꾸고 YAML 을 싣지 않는다 |
| goal 을 보내도 안 움직이고 `target_unhandled_count` 가 오른다 | `PushPendingTarget` 만 하고 `ApplyPendingTarget` 을 override 하지 않음 | override 해서 slot 에 쓴다 |
| 재활성화 직후 목표가 옛 값으로 튄다 | `on_activate` override 가 base 를 호출하지 않음 | base `on_activate` 를 먼저 호출한다 |
| 컨트롤러가 링크됐는데 `GetEntries()` 에 없다 | 정적 라이브러리의 등록 TU 가 링커에서 제거됨 | `--whole-archive` 로 링크한다 |

## 더 읽기

- 헤더 Doxygen: [rt_controller_interface.hpp](include/rtc_controller_interface/rt_controller_interface.hpp) · [controller_registry.hpp](include/rtc_controller_interface/controller_registry.hpp) · [device_readability.hpp](include/rtc_controller_interface/device_readability.hpp)
- F5 게이트 계약: [compliance-conventions.md](../rtc_controllers/docs/compliance-conventions.md) §3.7
- actuator-boundary 정책 · `list_controllers` 진단 필드: [rtc_controller_manager/README.md](../rtc_controller_manager/README.md)
- 바인딩 3계층 배치: [design-principles.md](../agent_docs/design-principles.md) · 규범: [invariants.md](../agent_docs/invariants.md)
- 바인딩 추가 절차: [modification-guide.md](../agent_docs/modification-guide.md) · [docs/modification-procedures.md](../docs/modification-procedures.md)
- target 계약 · 카운터 도입 이력: issue #206 · #196 · #779

## 의존성

[package.xml](package.xml) — `rtc_base` · `rtc_msgs` · `rtc_urdf_bridge` · `pinocchio` · `rclcpp` · `rclcpp_lifecycle` · `yaml-cpp`. 테스트는 개별 lint 만 쓴다 (`ament_lint_common` · `ament_uncrustify` 는 워크스페이스 정책으로 금지 — [conventions.md](../agent_docs/conventions.md)). MIT License.
