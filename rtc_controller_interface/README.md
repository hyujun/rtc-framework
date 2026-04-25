# rtc_controller_interface

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

## 개요

RTC 프레임워크의 **컨트롤러 추상 인터페이스 및 플러그인 레지스트리**를 제공하는 공유 C++ 라이브러리(`librtc_controller_interface.so`)입니다. Strategy 패턴을 사용하여 모든 제어 알고리즘이 구현해야 하는 추상 기반 클래스(`RTControllerInterface`)와, 정적 초기화 시점에 컨트롤러를 자동 등록하는 싱글톤 레지스트리(`ControllerRegistry`), 그리고 등록 매크로(`RTC_REGISTER_CONTROLLER`)를 제공합니다.

---

## 핵심 컴포넌트

### 헤더 파일

| 파일 | 설명 |
|------|------|
| `include/rtc_controller_interface/rt_controller_interface.hpp` | 추상 컨트롤러 인터페이스 (`RTControllerInterface`) |
| `include/rtc_controller_interface/controller_registry.hpp` | 싱글톤 레지스트리 (`ControllerRegistry`) + `RTC_REGISTER_CONTROLLER` 매크로 |
| `include/rtc_controller_interface/controller_types.hpp` | `rtc_base/types/types.hpp` 재수출 (편의 헤더) |

### 소스 파일

| 파일 | 설명 |
|------|------|
| `src/rt_controller_interface.cpp` | 기본 생성자, `LoadConfig()`, `ParseTopicConfig()`, `MakeDefaultTopicConfig()` 구현 |
| `src/controller_registry.cpp` | Meyer's 싱글톤 `Instance()` 및 `Register()` 구현 |

---

## RTControllerInterface (추상 기반 클래스)

`rtc` 네임스페이스에 정의되며, 복사/이동이 금지(`delete`)된 추상 클래스입니다.

### 순수 가상 메서드 (반드시 구현, 모두 `noexcept`)

```cpp
[[nodiscard]] virtual ControllerOutput Compute(const ControllerState& state) noexcept = 0;
virtual void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept = 0;
virtual void InitializeHoldPosition(const ControllerState& state) noexcept = 0;
[[nodiscard]] virtual std::string_view Name() const noexcept = 0;
```

| 메서드 | 설명 |
|--------|------|
| `Compute` | RT 루프에서 호출되는 제어 연산. `ControllerOutput` 반환 |
| `SetDeviceTarget` | 디바이스 인덱스 및 가변 크기 목표값 설정 |
| `InitializeHoldPosition` | 현재 위치를 타겟으로 초기화 (auto-hold) |
| `Name` | 사람이 읽을 수 있는 컨트롤러 이름 반환 |

### 가상 메서드 (기본 구현 제공)

| 메서드 | 기본 동작 | 설명 |
|--------|----------|------|
| `TriggerEstop()` | no-op | 비상 정지 트리거 |
| `ClearEstop()` | no-op | 비상 정지 해제 |
| `IsEstopped()` | `false` 반환 | 비상 정지 상태 확인 |
| `SetHandEstop(bool)` | no-op | 핸드 비상 정지 설정 |
| `LoadConfig(const YAML::Node&)` | 디바이스 플래그 경고 + 토픽 파싱 | YAML 설정 로드. `noexcept`가 아님 (throw 가능) |
| `UpdateGainsFromMsg(span<const double>)` | no-op | 런타임 게인 업데이트 |
| `GetCurrentGains()` | `{}` 반환 | 현재 게인 배열 반환 |
| `GetCommandType()` | `CommandType::kPosition` | 커맨드 타입 (`kPosition` 또는 `kTorque`) |
| `GetMpcSolveStats()` | `std::nullopt` | MPC 루프를 보유한 컨트롤러만 오버라이드. `RtControllerNode`의 aux 1Hz 타이머가 폴링하여 `<session>/controller/mpc_solve_timing.csv`로 기록하고 10초마다 `RCLCPP_INFO` 요약. 반환 타입은 `std::optional<rtc::MpcSolveStats>` ([`rtc_base/timing/mpc_solve_stats.hpp`](../rtc_base/include/rtc_base/timing/mpc_solve_stats.hpp)). |

> **`LoadConfig()` 기본 구현 동작:**
> 1. `cfg["enable_ur5e"]` / `cfg["enable_hand"]` 존재 시 deprecated 경고 출력 후 무시
> 2. `cfg["topics"]` 존재 시 `ParseTopicConfig()` 호출, 없으면 기본 토픽 유지
>
> 하위 클래스에서 오버라이드 시 `RTControllerInterface::LoadConfig(cfg)`를 먼저 호출해야 토픽 설정이 적용됩니다.

### Lifecycle 훅 (ros2_control 정렬, 기본 구현 제공)

시그니처는 `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface`와 동형이며, 향후 `RTControllerInterface`를 `LifecycleNode` 상속형으로 전환할 때 시그니처 변경 없이 이동 가능합니다. 모든 훅은 `noexcept`입니다.

| 훅 | 시그니처 | 기본 동작 |
|---|---|---|
| `on_configure` | `(State, LifecycleNode::SharedPtr, YAML::Node) → CallbackReturn` | `node_`에 주입된 노드 저장 → `LoadConfig(yaml_cfg)`를 try/catch로 호출 → 성공 시 `SUCCESS`, YAML 파싱 throw 시 `FAILURE` 반환. 실패 경로 로그는 `rclcpp::get_logger("rtc_controller_interface")` 정적 logger 사용 + 메시지 본문에 `[<controller_name>]` prefix로 호출 주체 표시 — 네이밍 규약은 [agent_docs/conventions.md](../agent_docs/conventions.md) "Logging" 섹션 참조 |
| `on_activate` | `(State) → CallbackReturn` | no-op `SUCCESS` |
| `on_deactivate` | `(State) → CallbackReturn` | no-op `SUCCESS` |
| `on_cleanup` | `(State) → CallbackReturn` | `node_.reset()` → `SUCCESS` |
| `on_shutdown` | `(State) → CallbackReturn` | `on_cleanup(state)` 위임 |
| `on_error` | `(State) → CallbackReturn` | no-op `SUCCESS` (서브클래스에서 E-STOP 트리거 등 원하는 복구 로직 override) |

> **Override 규약**: 서브클래스가 `on_configure`를 오버라이드해 자체 sub/pub을 만들 때 반드시 `RTControllerInterface::on_configure(previous_state, node, yaml_cfg)`를 먼저 호출한 뒤 `node_->create_subscription(...)` / `node_->create_publisher(...)`로 확장합니다. 이 순서를 어기면 `LoadConfig()`가 실행되지 않아 `topic_config_`가 기본값으로 남습니다.
>
> RT 경로(`Compute`, `SetDeviceTarget` 등)에서는 **절대 `node_`에 접근하지 않습니다** — ROS2 API 호출은 non-RT 훅에서만 수행하고, RT 경로에서는 사전 할당된 SeqLock/SPSC 버퍼를 통해 값을 읽습니다.

### LifecycleNode 접근자

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `get_lifecycle_node()` | public const noexcept | `on_configure` 성공 후 non-null. `RtControllerNode`가 `aux_executor`에 attach하기 위해 사용. `on_cleanup` 후 null |

멤버 `node_`는 `protected`로 노출되어 서브클래스가 `node_->create_subscription(...)` 형태로 직접 ROS I/O를 소유합니다. 이 네이밍 규약(`node_->`)은 향후 상속 전환 시 `this->`로 기계적 치환이 가능하도록 의도된 것입니다.

### 디바이스 설정 메서드

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `SetDeviceNameConfigs(map)` | public | 디바이스 이름/설정 맵 저장 후 `OnDeviceConfigsSet()` 호출 |
| `GetDeviceNameConfig(name)` | public const | 디바이스 이름으로 설정 조회. 미등록 시 `nullptr` 반환 |
| `OnDeviceConfigsSet()` | protected virtual | 하위 클래스 오버라이드 포인트 (예: URDF 기구학 해석) |
| `GetPrimaryDeviceName()` | public const | 토픽 설정의 첫 번째 디바이스 이름 반환 (하드코딩 방지용) |

### 시스템 모델 설정 메서드

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `SetSystemModelConfig(config)` | public | 시스템 레벨 `ModelConfig`를 복사 저장 후 `OnSystemModelConfigSet()` 호출 |
| `GetSystemModelConfig()` | public const noexcept | 저장된 `ModelConfig` 포인터 반환. 미설정 시 `nullptr` |
| `OnSystemModelConfigSet()` | protected virtual | 하위 클래스 오버라이드 포인트 (예: arm sub-model 구축) |

> **`SetSystemModelConfig()`** 은 `RtControllerNode`가 컨트롤러 인스턴스 생성 직후, `LoadConfig()` 호출 이전에 실행합니다. 따라서 `LoadConfig()` 내에서 `GetSystemModelConfig()`로 시스템 URDF 경로, sub_models, tree_models, passive_joints 정보에 접근할 수 있습니다.

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
| `MakeDefaultTopicConfig(device_name)` | protected static | 기본 토픽 설정 생성. `device_name` 기본값은 `"ur5e"` |

### 보호 멤버 변수

```cpp
TopicConfig topic_config_;                                      // 기본값: MakeDefaultTopicConfig("ur5e")
std::map<std::string, DeviceNameConfig> device_name_configs_;   // SetDeviceNameConfigs()에서 설정
std::unique_ptr<rtc_urdf_bridge::ModelConfig> system_model_config_;  // SetSystemModelConfig()에서 설정
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
| `Register(ControllerEntry)` | 컨트롤러 엔트리를 `entries_` 벡터에 추가 |
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
| `config_key` | 식별자 (따옴표 없음) | `p_controller` |
| `config_subdir` | 문자열 리터럴 | `"indirect/"` |
| `config_package` | 문자열 리터럴 | `"rtc_controllers"` |
| `FactoryExpr` | 코드 표현식 | `std::make_unique<PController>(urdf)` |

> `urdf` 변수는 매크로가 생성하는 람다의 파라미터 `(const std::string& urdf)`로 자동 제공됩니다.

### 매크로 전개 예시

```cpp
// 입력
RTC_REGISTER_CONTROLLER(p_controller, "indirect/", "rtc_controllers",
    std::make_unique<PController>(urdf))

// 전개 결과
namespace {
  [[maybe_unused]] const bool rtc_reg_p_controller = [] {
    ::rtc::ControllerRegistry::Instance().Register({
      "p_controller",           // config_key (문자열화)
      "indirect/",              // config_subdir
      "rtc_controllers",        // config_package
      [](const std::string& urdf) {  // factory 람다
        return std::make_unique<PController>(urdf);
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

예: `rtc_controllers/config/indirect/p_controller.yaml`

### 정적 라이브러리 링커 스트립 방지

정적 라이브러리에 포함된 컨트롤러는 외부 참조가 없으면 링커가 제거할 수 있습니다. 해결 방법:

```cpp
// 방법 1: Force 함수 정의 후 main()에서 호출
namespace rtc { void ForcePControllerRegistration() {} }

// 방법 2: --whole-archive 링커 플래그
target_link_libraries(my_exe
  PRIVATE -Wl,--whole-archive rtc_controllers -Wl,--no-whole-archive)
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
        - topic: "/joint_states"
          role: "state"                # ownership omitted → manager (default)
        - topic: "ur5e/joint_goal"     # relative path (controller namespace)
          role: "target"
          ownership: "controller"
      publish:
        - topic: "/ur5e/joint_command"
          role: "joint_command"
          data_size: 6
        - topic: "ur5e/gui_position"   # relative path (controller namespace)
          role: "gui_position"
          ownership: "controller"
    hand:
      subscribe:
        - topic: "/hand/joint_states"
          role: "state"
```

> 폐기된 flat format (`topics.subscribe`를 직접 사용)은 마이그레이션 에러를 발생시킵니다.

### 토픽 소유권 (TopicOwnership)

각 subscribe/publish entry는 optional `ownership` 필드를 갖습니다. 누락 시 기본값은 `"manager"`.

| YAML 값 | enum 값 | 주체 | 용도 |
|---------|---------|------|------|
| `manager` (default) | `kManager` | `RtControllerNode` (CM) | RT-adjacent HW 트래픽 (state/command/log). 500 Hz 루프와 SPSC 드레인을 경유 |
| `controller` | `kController` | 컨트롤러별 `LifecycleNode` | 외부 GUI/BT/planner용 non-RT 트래픽. 상대 경로는 노드 namespace `/<config_key>/...`로 자동 해석 |

Controller-owned 토픽은 on_configure에서 컨트롤러가 직접 `node_->create_subscription(...)` / `node_->create_publisher(...)`로 생성합니다. CM의 publish thread는 SPSC snapshot을 드레인한 뒤 `controllers_[active]->PublishNonRtSnapshot(snap)` 을 호출해 controller-owned 발행을 위임합니다.

### 구독 역할 (SubscribeRole)

| YAML 역할 문자열 | enum 값 | 설명 |
|-----------------|---------|------|
| `state` | `kState` | 디바이스 관절 상태 (JointState) |
| `motor_state` | `kMotorState` | 모터 공간 상태 (JointState) |
| `sensor_state` | `kSensorState` | 센서 상태 (Float64MultiArray) |
| `target` | `kTarget` | 외부 목표 (Float64MultiArray) |
| `joint_state` | `kState` | 하위 호환 별칭 |
| `hand_state` | `kState` | 하위 호환 별칭 |
| `goal` | `kTarget` | 하위 호환 별칭 |

### 퍼블리시 역할 (PublishRole)

| YAML 역할 문자열 | enum 값 | 설명 |
|-----------------|---------|------|
| `joint_command` | `kJointCommand` | 통합 관절 커맨드 (JointCommand) |
| `ros2_command` | `kRos2Command` | ROS2 표준 커맨드 (Float64MultiArray) |
| `gui_position` | `kGuiPosition` | GUI 표시용 위치 (GuiPosition) |
| `robot_target` | `kRobotTarget` | 관절/태스크 목표 (RobotTarget) |
| `device_state_log` | `kDeviceStateLog` | 통합 상태 로그 (DeviceStateLog) |
| `device_sensor_log` | `kDeviceSensorLog` | 센서 + 추론 로그 (DeviceSensorLog) |
| `grasp_state` | `kGraspState` | BT coordinator용 grasp 상태 (GraspState) |
| `tof_snapshot` | `kToFSnapshot` | ToF 센서 + 핑거팁 포즈 스냅샷 (ToFSnapshot) |
| `digital_twin_state` | `kDigitalTwinState` | 디지털 트윈용 관절 상태 (JointState, RELIABLE QoS) |
| `joint_goal` | `kRobotTarget` | 하위 호환 별칭 |
| `position_command` | `kRos2Command` | 하위 호환 별칭 |
| `torque_command` | `kRos2Command` | 하위 호환 별칭 |
| `hand_command` | `kJointCommand` | 하위 호환 별칭 |

> 참고: `kDigitalTwinState` 역할은 `PublishRole` enum에 정의되어 있으며, YAML에서 `"digital_twin_state"` 역할 문자열로 설정할 수 있습니다. 디지털 트윈용 JointState를 RELIABLE QoS로 퍼블리시합니다.

### 기본 토픽 설정

`MakeDefaultTopicConfig("ur5e")` 호출 시 생성되는 기본 토픽:

```
ur5e.subscribe:
  /joint_states                          (kState)
  /ur5e/target_joint_positions           (kTarget)

ur5e.publish:
  /ur5e/joint_command                    (kJointCommand, data_size=6)
  /forward_position_controller/commands  (kRos2Command,  data_size=6)
  /ur5e/gui_position                     (kGuiPosition)
  /ur5e/robot_target                     (kRobotTarget)
  /ur5e/state_log                        (kDeviceStateLog)
```

> YAML에 `topics` 섹션이 없으면 위 기본값이 사용됩니다.

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rtc_base` | 공유 데이터 타입 (`ControllerState`, `ControllerOutput`, `TopicConfig`, `DeviceNameConfig` 등) |
| `rtc_msgs` | 커스텀 ROS2 메시지 |
| `rtc_urdf_bridge` | URDF→Pinocchio 모델 빌더 + `ModelConfig` 타입 (시스템 모델 설정) |
| `pinocchio` | 로보틱스 기구학/동역학 (하위 패키지에 전이적 제공) |
| `yaml-cpp` | YAML 설정 파싱 |

테스트 의존성: `ament_lint_auto`, `ament_lint_common`

---

## 라이선스

MIT License
