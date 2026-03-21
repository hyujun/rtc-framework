# rtc_controller_interface

![version](https://img.shields.io/badge/version-v0.1.0-blue)

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
│  - Compute(), SetRobotTarget(), SetHandTarget()          │
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

#### 순수 가상 메서드 (반드시 구현)

| 메서드 | 반환 | 설명 |
|--------|------|------|
| `Compute(const ControllerState&)` | `ControllerOutput` | 제어 연산 (500 Hz, `noexcept`) |
| `SetRobotTarget(span<const double, 6>)` | `void` | 로봇 목표 설정 (`noexcept`) |
| `SetHandTarget(span<const float, 10>)` | `void` | 핸드 목표 설정 (`noexcept`) |
| `InitializeHoldPosition(const ControllerState&)` | `void` | 현재 위치 유지 초기화 (`noexcept`) |
| `Name()` | `string_view` | 컨트롤러 이름 (`noexcept`) |

#### 가상 메서드 (기본 구현 제공)

| 메서드 | 기본값 | 설명 |
|--------|--------|------|
| `TriggerEstop()` | no-op | 비상 정지 트리거 |
| `ClearEstop()` | no-op | 비상 정지 해제 |
| `IsEstopped()` | `false` | 비상 정지 상태 조회 |
| `SetHandEstop(bool)` | no-op | 핸드 비상 정지 |
| `LoadConfig(const YAML::Node&)` | 토픽/디바이스 파싱 | YAML 설정 로딩 (non-noexcept) |
| `UpdateGainsFromMsg(span<const double>)` | no-op | 런타임 게인 업데이트 (`noexcept`) |
| `GetCurrentGains()` | `{}` | 현재 게인 반환 |
| `GetCommandType()` | `kPosition` | 커맨드 모드 (position/torque) |

#### 설정 접근자

| 메서드 | 설명 |
|--------|------|
| `GetTopicConfig()` | 컨트롤러별 토픽 라우팅 설정 반환 |
| `GetPerControllerDeviceFlags()` | 컨트롤러별 디바이스 활성화 오버라이드 반환 |

#### 보호 유틸리티 (하위 클래스용)

| 메서드 | 설명 |
|--------|------|
| `ParseTopicConfig(const YAML::Node&)` | YAML 토픽 섹션 파싱 (static) |
| `MakeDefaultTopicConfig()` | 기본 토픽 설정 생성 (static) |

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

> `urdf` 변수는 매크로가 생성하는 람다의 파라미터로 자동 제공됩니다.

**사용 예시:**

```cpp
#include <rtc_controller_interface/controller_registry.hpp>
#include "my_controller.hpp"

RTC_REGISTER_CONTROLLER(
    my_controller,                            // YAML 파일명
    "indirect/",                              // 카테고리
    "my_package",                             // 설정 패키지
    std::make_unique<MyController>(urdf))     // 팩토리

// 정적 라이브러리에서 링커 스트립 방지용
namespace rtc { void ForceMyControllerRegistration() {} }
```

---

### 토픽 설정 시스템

각 컨트롤러는 YAML을 통해 독립적인 ROS2 토픽 라우팅을 설정할 수 있습니다.

```yaml
my_controller:
  enable_ur5e: true
  enable_hand: true
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

| 역할 | 설명 |
|------|------|
| `joint_state` | 로봇 관절 상태 |
| `hand_state` | 핸드 모터 상태 |
| `goal` / `target` | 목표 위치 (target은 하위 호환) |

#### 퍼블리시 역할 (PublishRole)

| 역할 | 설명 |
|------|------|
| `position_command` | 위치 커맨드 |
| `torque_command` | 토크 커맨드 |
| `hand_command` | 핸드 모터 커맨드 |
| `task_position` | 태스크 공간 위치 (FK) |
| `trajectory_state` | 궤적 상태 |
| `controller_state` | 컨트롤러 내부 상태 |

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

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
