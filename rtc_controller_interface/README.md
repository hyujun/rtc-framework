# rtc_controller_interface

![version](https://img.shields.io/badge/version-v0.1.0-blue)
![C++20](https://img.shields.io/badge/C++-20-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **컨트롤러 추상 인터페이스 및 플러그인 레지스트리** 패키지입니다. Strategy 패턴을 사용하여 모든 제어 알고리즘이 구현해야 하는 추상 기반 클래스(`RTControllerInterface`)와, 정적 초기화 시점에 컨트롤러를 등록하는 싱글톤 레지스트리(`ControllerRegistry`)를 제공합니다.

**핵심 설계 원칙:**
- RT 경로의 모든 가상 메서드에 `noexcept` 보장
- YAML 기반 컨트롤러별 설정 및 토픽 라우팅
- 매크로 기반 자동 등록 (`RTC_REGISTER_CONTROLLER`)
- 컨트롤러별 디바이스 활성화 오버라이드

---

## 패키지 구조

```
rtc_controller_interface/
├── CMakeLists.txt
├── package.xml
├── include/rtc_controller_interface/
│   ├── rt_controller_interface.hpp    ← 추상 컨트롤러 인터페이스
│   ├── controller_registry.hpp        ← 싱글톤 플러그인 레지스트리 + 등록 매크로
│   └── controller_types.hpp           ← rtc_base 타입 재수출
└── src/
    ├── rt_controller_interface.cpp    ← 인터페이스 구현 (YAML 파싱)
    └── controller_registry.cpp        ← 레지스트리 싱글톤 구현
```

---

## 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│  rtc_controller_manager (RtControllerNode)              │
│  - 500 Hz RT 루프에서 Compute() 호출                     │
│  - LoadConfig()로 YAML 설정 로딩                         │
│  - Registry에서 컨트롤러 팩토리 조회                      │
└───────────────┬─────────────────────────────────────────┘
                │ 사용 (composition)
                ▼
┌─────────────────────────────────────────────────────────┐
│  RTControllerInterface (추상 기반 클래스)                 │  ← 이 패키지
│  - Compute(), SetDeviceTarget(), InitializeHoldPosition() │
│  - LoadConfig(), UpdateGainsFromMsg()                    │
│  - E-STOP 인터페이스                                     │
├─────────────────────────────────────────────────────────┤
│  ControllerRegistry (싱글톤)                             │
│  - RTC_REGISTER_CONTROLLER 매크로로 등록                  │
│  - GetEntries()로 등록된 컨트롤러 조회                    │
└───────────────┬─────────────────────────────────────────┘
                ▲ 구현 (상속)
                │
┌───────────────┴─────────────────────────────────────────┐
│  rtc_controllers (구현체)                                │
│  - PController, JointPDController                        │
│  - ClikController, OperationalSpaceController            │
└─────────────────────────────────────────────────────────┘
```

---

## 컴포넌트 상세

### RTControllerInterface (`rt_controller_interface.hpp`)

모든 컨트롤러가 구현해야 하는 추상 기반 클래스입니다.

#### 순수 가상 메서드 (반드시 구현, 모두 `noexcept`)

```cpp
[[nodiscard]] virtual ControllerOutput Compute(const ControllerState& state) noexcept = 0;
virtual void SetDeviceTarget(int device_idx, std::span<const double> target) noexcept = 0;
virtual void InitializeHoldPosition(const ControllerState& state) noexcept = 0;
[[nodiscard]] virtual std::string_view Name() const noexcept = 0;
```

| 메서드 | 호출 빈도 | 설명 |
|--------|----------|------|
| `Compute` | 500 Hz (RT 루프) | 제어 연산 → `ControllerOutput` 반환 |
| `SetDeviceTarget` | 이벤트 (센서 스레드) | 가변 크기 디바이스 목표 설정 (`device_idx`: 0=로봇, 1=핸드 등) |
| `InitializeHoldPosition` | 1회 (첫 상태 수신) | 현재 위치를 타겟으로 초기화 (auto-hold) |
| `Name` | UI/로깅 | 사람이 읽을 수 있는 이름 |

#### 가상 메서드 (기본 구현 제공)

```cpp
virtual void TriggerEstop() noexcept {}
virtual void ClearEstop() noexcept {}
[[nodiscard]] virtual bool IsEstopped() const noexcept { return false; }
virtual void SetHandEstop(bool) noexcept {}
virtual void LoadConfig(const YAML::Node& cfg);           // ⚠️ non-noexcept (throw 가능)
virtual void UpdateGainsFromMsg(std::span<const double> gains) noexcept { (void)gains; }
[[nodiscard]] virtual std::vector<double> GetCurrentGains() const noexcept { return {}; }
[[nodiscard]] virtual CommandType GetCommandType() const noexcept { return CommandType::kPosition; }
```

| 메서드 | 기본값 | 설명 |
|--------|--------|------|
| E-STOP (4개) | no-op | 비상 정지 인터페이스 (필요 시 오버라이드) |
| `LoadConfig` | 디바이스 플래그 + 토픽 파싱 | **하위 클래스는 `super::LoadConfig()` 먼저 호출** 후 게인 파싱 |
| `UpdateGainsFromMsg` | no-op | 런타임 게인 업데이트 (배열 레이아웃은 컨트롤러별 문서화) |
| `GetCurrentGains` | `{}` | 현재 게인 반환 (GUI "Load Gain" 기능용) |
| `GetCommandType` | `kPosition` | `kPosition` 또는 `kTorque` |

> **`LoadConfig()` 기본 구현 동작:**
> 1. `cfg["topics"]` → `ParseTopicConfig()` (없으면 기본 토픽 유지)
> 2. `cfg["enable_ur5e"]`/`cfg["enable_hand"]` → deprecated 경고 출력 후 무시

#### 디바이스 설정 메서드

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `SetDeviceNameConfigs(map)` | public | 디바이스 이름/URDF/관절 한계 설정 → `OnDeviceConfigsSet()` 호출 |
| `GetDeviceNameConfig(name)` | public const | 디바이스 이름으로 설정 조회 (`nullptr` = 미등록) |
| `OnDeviceConfigsSet()` | protected virtual | 하위 클래스 오버라이드: URDF 기구학 해석 (tip_link → end_id) |

#### 설정 접근자 & 보호 유틸리티

| 메서드 | 접근 | 설명 |
|--------|------|------|
| `GetTopicConfig()` | public const | 컨트롤러별 토픽 라우팅 설정 반환 |
| `ParseTopicConfig(YAML::Node&)` | protected static | YAML `topics:` 하위의 모든 디바이스 그룹을 동적 파싱 |
| `MakeDefaultTopicConfig()` | protected static | 기본 토픽 설정 생성 (ur5e만 포함) |

#### 보호 멤버 변수

```cpp
TopicConfig topic_config_;                                      // LoadConfig()에서 설정됨
std::map<std::string, DeviceNameConfig> device_name_configs_;   // SetDeviceNameConfigs()에서 설정됨
```

---

### ControllerRegistry (`controller_registry.hpp`)

Meyer's 싱글톤 패턴 기반 컨트롤러 레지스트리입니다.

#### ControllerEntry 구조체

| 필드 | 타입 | 설명 |
|------|------|------|
| `config_key` | `string` | YAML 파일명 스템 (예: `"p_controller"`) |
| `config_subdir` | `string` | 설정 하위 디렉토리 (`"direct/"` 또는 `"indirect/"`) |
| `config_package` | `string` | 설정 YAML을 소유하는 ament 패키지 |
| `factory` | `function<unique_ptr<RTControllerInterface>(const string&)>` | URDF 경로 → 컨트롤러 인스턴스 팩토리 |

#### API

| 메서드 | 설명 |
|--------|------|
| `Instance()` | 싱글톤 인스턴스 반환 (static) |
| `Register(ControllerEntry)` | 컨트롤러 등록 (정적 초기화 시점) |
| `GetEntries()` | 등록된 컨트롤러 목록 반환 (`noexcept`) |

---

### RTC_REGISTER_CONTROLLER 매크로

컨트롤러를 정적 초기화 시점에 자동 등록하는 매크로입니다.

```cpp
RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, FactoryExpr)
```

| 파라미터 | 타입 | 예시 |
|---------|------|------|
| `config_key` | 식별자 (따옴표 없음) | `p_controller` |
| `config_subdir` | 문자열 | `"indirect/"` |
| `config_package` | 문자열 | `"rtc_controllers"` |
| `FactoryExpr` | 코드 표현식 | `std::make_unique<PController>(urdf)` |

> `urdf` 변수는 매크로가 생성하는 람다의 파라미터 `(const std::string& urdf)`로 자동 제공됩니다.

**매크로 전개:**

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
  }();  // IIFE — main() 이전 실행
}
```

**설정 파일 경로 생성 규칙:**

```
<config_package>/config/<config_subdir><config_key>.yaml
```

예: `rtc_controllers/config/indirect/p_controller.yaml`

**정적 라이브러리 링커 스트립 방지:**

정적 라이브러리에 포함된 컨트롤러는 외부 참조가 없으면 링커가 제거합니다. 두 가지 해결법:

```cpp
// 방법 1: Force 함수 (같은 TU에서 정의)
namespace rtc { void ForcePControllerRegistration() {} }
// → main()에서 호출

// 방법 2: --whole-archive (CMakeLists.txt)
target_link_libraries(my_exe
  PRIVATE -Wl,--whole-archive rtc_controllers -Wl,--no-whole-archive)
```

---

### 토픽 설정 시스템

각 컨트롤러는 YAML을 통해 독립적인 ROS2 토픽 라우팅을 설정할 수 있습니다.

```yaml
my_controller:
  topics:
    ur5e:
      subscribe:
        - topic: "/joint_states"
          role: "joint_state"
        - topic: "/ur5e/target_joint_positions"
          role: "goal"
      publish:
        - topic: "/forward_position_controller/commands"
          role: "position_command"
        - topic: "/ur5e/current_task_position"
          role: "task_position"
          data_size: 6
    hand:
      subscribe:
        - topic: "/hand/joint_states"
          role: "hand_state"
```

#### 구독 역할 (SubscribeRole)

| 역할 | enum | 설명 |
|------|------|------|
| `state` | `kState` | 디바이스 관절 상태 |
| `sensor_state` | `kSensorState` | 센서 상태 (핸드 등) |
| `target` | `kTarget` | 목표 위치 |
| `joint_state`, `hand_state`, `goal` | — | 하위 호환 별칭 |

#### 퍼블리시 역할 (PublishRole)

| 역할 | enum | 설명 |
|------|------|------|
| `joint_command` | `kJointCommand` | 관절/모터 커맨드 |
| `ros2_command` | `kRos2Command` | ROS2 표준 커맨드 (Float64MultiArray) |
| `task_position` | `kTaskPosition` | 태스크 공간 위치 (FK) |
| `trajectory_state` | `kTrajectoryState` | 궤적 상태 |
| `controller_state` | `kControllerState` | 컨트롤러 내부 상태 |
| `position_command`, `torque_command`, `hand_command` | — | 하위 호환 별칭 |

**기본 토픽 설정** (`MakeDefaultTopicConfig()` 하드코딩):

```
ur5e.subscribe: /joint_states (kState), /ur5e/target_joint_positions (kTarget)
ur5e.publish:   /forward_position_controller/commands (kPositionCommand, 6)
                /forward_torque_controller/commands (kTorqueCommand, 6)
                /ur5e/current_task_position (kTaskPosition, 6)
                /ur5e/trajectory_state (kTrajectoryState, 18)
                /ur5e/controller_state (kControllerState, 18)
hand.subscribe: /hand/joint_states (kHandState)
```

> topics 섹션이 YAML에 없으면 위 기본값이 사용됩니다. 폐기된 flat format (`topics.subscribe`) 사용 시 마이그레이션 에러가 발생합니다.

---

### 핵심 데이터 구조 (`rtc_base/types/types.hpp`에서 정의)

#### ControllerState (Compute 입력)

일반화된 멀티 디바이스 구조 (`kMaxDevices = 4`):

| 필드 | 타입 | 설명 |
|------|------|------|
| `devices[4]` | `DeviceState[]` | 디바이스별 상태 배열 |
| `devices[i].positions[64]` | `double` | 관절 각도 (rad) |
| `devices[i].velocities[64]` | `double` | 관절 각속도 (rad/s) |
| `devices[i].efforts[64]` | `double` | 토크/힘 (N·m) |
| `devices[i].sensor_data[128]` | `int32_t` | 센서 데이터 (filtered) |
| `devices[i].sensor_data_raw[128]` | `int32_t` | 원시 센서 데이터 |
| `devices[i].num_channels` | `int` | 유효 채널 수 |
| `devices[i].valid` | `bool` | 데이터 유효성 |
| `num_devices` | `int` | 활성 디바이스 수 |
| `dt` | `double` | 시간 간격 (0.002s @500Hz) |
| `iteration` | `uint64_t` | 루프 카운터 |

#### ControllerOutput (Compute 출력)

| 필드 | 타입 | 설명 |
|------|------|------|
| `devices[4]` | `DeviceOutput[]` | 디바이스별 출력 배열 |
| `devices[i].commands[64]` | `double` | 커맨드 (위치 또는 토크) |
| `devices[i].goal_positions[64]` | `double` | 스텝 목표 (로깅용) |
| `devices[i].target_positions[64]` | `double` | 궤적 위치 (로깅용) |
| `devices[i].target_velocities[64]` | `double` | 궤적 속도 (로깅용) |
| `devices[i].num_channels` | `int` | 유효 채널 수 |
| `actual_task_positions[6]` | `double` | TCP 위치 + RPY (FK) |
| `valid` | `bool` | `false`면 커맨드 무시 |
| `command_type` | `CommandType` | `kPosition` 또는 `kTorque` |
| `num_devices` | `int` | 활성 디바이스 수 |

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rtc_base` | 공유 데이터 타입, 상수 |
| `rtc_msgs` | 커스텀 ROS2 메시지 |
| `sensor_msgs` | JointState 메시지 |
| `pinocchio` | 로보틱스 기구학/동역학 (하위 패키지에 전이적 제공) |
| `yaml-cpp` | YAML 설정 파싱 |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select rtc_controller_interface
source install/setup.bash
```

공유 라이브러리(`librtc_controller_interface.so`)가 생성됩니다.

---

## 의존성 그래프 내 위치

```
rtc_base + rtc_msgs + pinocchio + yaml-cpp
    ↓
rtc_controller_interface  ← 추상 인터페이스 + 레지스트리
    ↑
    ├── rtc_controllers        (4개 컨트롤러 구현)
    ├── rtc_controller_manager (레지스트리 조회 + Compute 호출)
    └── ur5e_bringup           (커스텀 컨트롤러 등록)
```

---

## 최적화 내역 (v0.1.1)

| 영역 | 변경 내용 |
|------|----------|
| **controller_registry** | `Instance()` 메서드에 `noexcept` 추가 (Meyer's singleton은 예외를 throw하지 않음) |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
