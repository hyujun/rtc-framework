# rtc_urdf_bridge

Robot-agnostic URDF parser and Pinocchio model builder for real-time control loops.

## Overview

`rtc_urdf_bridge`는 URDF 파일을 파싱하여 [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 모델을 자동으로 구축하는 C++20 라이브러리입니다. 하드코딩된 로봇 이름이나 구조 가정 없이, URDF에서 모든 정보를 런타임에 추출합니다.

### 주요 특징

- **Robot-agnostic** - 링크/관절 이름을 하드코딩하지 않음. 모든 정보를 URDF에서 런타임 추출
- **RT-safe** - `RtModelHandle`이 제어 루프에서 힙 할당, 예외, 뮤텍스 없이 동작
- **유연한 모델링** - Full model, Reduced sub-model (직렬 체인), Tree model (분기 구조) 지원
- **자동 감지** - Passive 관절, Mimic 관절, 폐쇄 체인을 URDF 메타데이터에서 자동 파싱
- **YAML 선언적 설정** - 서브모델, 트리모델, 폐쇄 체인, 패시브 관절을 YAML로 정의

## Architecture

```
[YAML Config] → [PinocchioModelBuilder]
                        │
                  [UrdfAnalyzer]          ← URDF 파싱 + 토폴로지 분석
                        │
              [KinematicChainExtractor]   ← 서브체인 / 트리 추출
                        │
              [Pinocchio Model 생성]
               ├─ Full Model (전체 관절)
               ├─ Reduced Model (서브체인)
               └─ Tree Model (분기 구조)
                        │
                 [RtModelHandle]          ← RT-safe FK, Jacobian, Dynamics
```

## Package Structure

```
rtc_urdf_bridge/
├── include/rtc_urdf_bridge/
│   ├── types.hpp                       # 데이터 구조체 및 열거형
│   ├── urdf_analyzer.hpp               # URDF 파싱 및 트리 분석
│   ├── kinematic_chain_extractor.hpp   # 서브모델/트리모델 체인 추출
│   ├── pinocchio_model_builder.hpp     # YAML→Pinocchio 모델 빌드 파이프라인
│   └── rt_model_handle.hpp             # RT-safe 계산 래퍼
├── src/
│   ├── urdf_analyzer.cpp
│   ├── kinematic_chain_extractor.cpp
│   ├── pinocchio_model_builder.cpp
│   └── rt_model_handle.cpp
├── config/                             # YAML 설정 템플릿
│   ├── serial_arm_config.yaml
│   ├── hand_tree_config.yaml
│   └── four_bar_config.yaml
├── examples/                           # 사용 예제
│   ├── example_basic_usage.cpp
│   ├── example_submodel.cpp
│   ├── example_tree_model.cpp
│   └── example_rt_integration.cpp
└── test/                               # GTest 테스트
    ├── test_urdf_analyzer.cpp
    ├── test_chain_extractor.cpp
    ├── test_model_builder.cpp
    ├── test_rt_model_handle.cpp
    └── urdf/                           # 테스트용 URDF 파일
```

## Dependencies

| 패키지 | 용도 |
|--------|------|
| pinocchio | 강체 동역학 알고리즘 (FK, Jacobian, RNEA, ABA) |
| Eigen3 | 선형대수 연산 |
| yaml-cpp | YAML 설정 파일 파싱 |
| tinyxml2 | URDF XML 파싱 |
| ament_cmake | ROS 2 빌드 시스템 |

## Build

```bash
colcon build --packages-select rtc_urdf_bridge
```

테스트 실행:

```bash
colcon test --packages-select rtc_urdf_bridge
colcon test-result --verbose
```

## Core Components

### 1. UrdfAnalyzer

URDF XML을 파싱하여 링크-관절 인접 그래프를 구축하고, 관절을 자동으로 분류합니다.

```cpp
#include "rtc_urdf_bridge/urdf_analyzer.hpp"

namespace rub = rtc_urdf_bridge;

// 파일 경로로 생성 (passive hint 선택적)
rub::UrdfAnalyzer analyzer("/path/to/robot.urdf");
rub::UrdfAnalyzer analyzer("/path/to/robot.urdf", {"some_joint_to_force_passive"});

// 또는 XML 문자열로 생성
rub::UrdfAnalyzer analyzer(xml_string, rub::UrdfAnalyzer::FromXmlTag{});

// 그래프 조회
analyzer.GetRootLinkName();          // 루트 링크 이름
analyzer.GetNumLinks();              // 전체 링크 수
analyzer.GetNumJoints();             // 전체 관절 수

// 관절 역할 (role 축) — 상호 배타적
analyzer.GetActiveJointNames();      // role == kActive
analyzer.GetPassiveJointNames();     // role == kPassive (모든 subtype)
analyzer.GetFixedJointNames();       // role == kFixed
analyzer.GetNonFixedJointNames();    // active ∪ passive (KinematicChainExtractor용)

// Passive subtype 필터
analyzer.GetPassiveJointNamesOfSubtype(rub::PassiveSubtype::kMimic);
analyzer.GetPassiveJointNamesOfSubtype(rub::PassiveSubtype::kClosedChain);
analyzer.GetPassiveJointNamesOfSubtype(rub::PassiveSubtype::kFree);

// 단일 관절 조회
analyzer.GetJointRole("joint_x");        // kFixed | kActive | kPassive
analyzer.GetPassiveSubtype("joint_x");   // kNone | kMimic | kClosedChain | kFree
analyzer.GetJointMeta("joint_x");        // has_physics, has_limit_tag 등 포함

// 경로 탐색
analyzer.FindPath("link_a", "link_b");   // 두 링크 사이 경로
analyzer.FindLCA("link_a", "link_b");    // 최소 공통 조상
analyzer.DetectClosedChains();            // 폐쇄 체인 감지
```

#### 관절 분류 규칙

role 축은 topology(`UrdfJointType`)와 직교한다. `UrdfAnalyzer`가 URDF 파싱
직후 아래 순서로 결정한다 (위에서 매치되면 종료):

| 조건 | Role | Subtype |
|------|------|---------|
| URDF `type == "fixed"` | kFixed | kNone |
| `<mimic>` 태그 존재 | kPassive | kMimic |
| closed-chain 루프 경로에 포함 | kPassive | kClosedChain |
| 생성자 `passive_hints`에 이름 명시 | kPassive | kFree |
| (그 외 non-fixed) | **kActive** | kNone |

- 기본값은 **active**. 명시적 passive 분류에 해당하지 않으면 전부 active.
- Active이지만 physics가 정의되지 않은 경우 경고가 출력된다 (예외 아님).
- mimic + closed-chain 중복 감지 시 kMimic 우선 + 경고.

**Physics 판정** (`JointMeta::has_physics`):

```
has_limit_tag == true
  AND effort > 0
  AND (type == kContinuous  OR  velocity > 0)
```

- `<transmission>` 태그는 **분류에 사용되지 않는다** (과거와 다름).
- `<limit>` 태그가 없거나 `effort="0"` 이면 warning.
- `continuous` 관절은 position limit 없이도 OK (effort/velocity만 체크).

### 2. KinematicChainExtractor

분석된 URDF 트리에서 서브모델(단일 체인)과 트리모델(분기 구조)을 추출합니다.

```cpp
#include "rtc_urdf_bridge/kinematic_chain_extractor.hpp"

rub::KinematicChainExtractor extractor(analyzer);

// 직렬 서브체인 추출 (root → tip)
auto sub = extractor.ExtractSubModel("arm", "base_link", "tool_link");
// sub.joint_names     → 경로 상 non-fixed 관절 (이후 passive 차감)
// sub.all_joint_names → 경로 상 모든 관절 (fixed 포함)
// sub.link_names      → 경로 상 모든 링크

// 트리모델 추출 (root → 여러 tip)
auto tree = extractor.ExtractTreeModel("hand", "palm_link",
    {"thumb_tip", "index_tip", "middle_tip", "ring_tip"});
// tree.branching_points → 분기점 링크 이름

// Reduced model 생성을 위한 잠금 관절 목록 계산
auto joints_to_lock = extractor.ComputeJointsToLock(sub);
```

### 3. PinocchioModelBuilder

YAML 설정과 URDF를 기반으로 Pinocchio 모델을 구축하는 메인 파이프라인입니다.

```cpp
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

// YAML 설정 파일로 빌드
rub::PinocchioModelBuilder builder("/path/to/config.yaml");

// 또는 ModelConfig 직접 전달
rub::ModelConfig config;
config.urdf_path = "/path/to/robot.urdf";
config.root_joint_type = "fixed";
config.sub_models.push_back({"arm", "base_link", "tool_link"});
rub::PinocchioModelBuilder builder(config);

// 모델 접근
auto full_model = builder.GetFullModel();           // 전체 모델
auto arm_model  = builder.GetReducedModel("arm");   // 축소 서브모델
auto hand_model = builder.GetTreeModel("hand");     // 트리모델

// 폐쇄 체인 구속 모델
const auto & constraints = builder.GetConstraintModels();

// 내부 분석기/추출기 접근
const auto & analyzer  = builder.GetAnalyzer();
const auto & extractor = builder.GetExtractor();
```

### 4. RtModelHandle

RT 제어 루프에서 안전하게 사용할 수 있는 Pinocchio 래퍼입니다. 모든 compute 함수는 `noexcept`이며, 사전 할당된 버퍼만 사용합니다.

```cpp
#include "rtc_urdf_bridge/rt_model_handle.hpp"

// [Non-RT] 초기화 - 버퍼 사전 할당
auto model = builder.GetReducedModel("arm");
rub::RtModelHandle handle(model);

// [RT 루프] - 힙 할당/예외/뮤텍스 없음
std::vector<double> q(handle.nq(), 0.0);
std::vector<double> v(handle.nv(), 0.0);

// Forward Kinematics
handle.ComputeForwardKinematics(q);
auto pos = handle.GetFramePosition(frame_id);      // 위치
auto rot = handle.GetFrameRotation(frame_id);      // 회전

// Jacobian
handle.ComputeJacobians(q);
Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, handle.nv());
handle.GetFrameJacobian(frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

// Dynamics
handle.ComputeNonLinearEffects(q, v);               // nle = C(q,v)v + g(q)
handle.ComputeInverseDynamics(q, v, a);             // RNEA: τ = M·a + C·v + g
handle.ComputeForwardDynamics(q, v, tau);           // ABA: a = M⁻¹(τ - C·v - g)
handle.ComputeMassMatrix(q);                        // M(q)
handle.ComputeConstraintDynamics(q, v, tau);        // 폐쇄 체인 구속 동역학
```

## YAML Configuration

### Serial Arm (직렬 로봇팔)

```yaml
urdf_path: "path/to/serial_6dof.urdf"
root_joint_type: "fixed"

sub_models:
  - name: "arm"
    root_link: "base_link"
    tip_link: "tool_link"
  - name: "arm_partial"
    root_link: "link_2"
    tip_link: "link_5"
```

### Tree Hand (트리 구조 핸드)

```yaml
urdf_path: "path/to/tree_hand.urdf"
root_joint_type: "fixed"

tree_models:
  - name: "hand"
    root_link: "palm_link"
    tip_links:
      - "thumb_tip"
      - "index_tip"
      - "middle_tip"
      - "ring_tip"

sub_models:
  - name: "thumb"
    root_link: "palm_link"
    tip_link: "thumb_tip"
```

### Closed Chain (폐쇄 체인)

```yaml
urdf_path: "path/to/four_bar.urdf"
root_joint_type: "fixed"

closed_chains:
  - name: "four_bar_loop"
    link_a: "link_b"
    link_b: "link_c"
    contact_type: "6D"
    offset_a:
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
    offset_b:
      xyz: [0.3, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
    baumgarte_kp: 10.0
    baumgarte_kd: 6.32
```

### Full Configuration Reference

```yaml
# URDF 소스 (둘 중 하나 필수)
urdf_path: "path/to/robot.urdf"       # 파일 경로
urdf_xml_string: "<robot>...</robot>"  # 또는 XML 문자열 직접 제공

# 루트 관절 타입
root_joint_type: "fixed"               # "fixed" 또는 "floating"

# 서브모델 (root → tip 단일 체인)
sub_models:
  - name: "model_name"
    root_link: "root_link_name"
    tip_link: "tip_link_name"

# 트리모델 (root → 여러 tip 분기)
tree_models:
  - name: "model_name"
    root_link: "root_link_name"
    tip_links: ["tip_1", "tip_2", ...]

# 폐쇄 체인 구속
closed_chains:
  - name: "constraint_name"
    link_a: "link_name_a"
    link_b: "link_name_b"
    contact_type: "6D"                 # "6D" 또는 "3D"
    offset_a: { xyz: [x,y,z], rpy: [r,p,y] }
    offset_b: { xyz: [x,y,z], rpy: [r,p,y] }
    baumgarte_kp: 10.0
    baumgarte_kd: 6.32

# 패시브 관절 override
passive_joints: ["joint_a", "joint_b"]

# 잠금 관절 기준 설정값 (radian)
lock_reference_config:
  joint_name: 0.5
```

## Model Types

| 모델 타입 | 설명 | 용도 |
|-----------|------|------|
| **Full Model** | URDF의 모든 관절 포함 | 전체 로봇 제어 |
| **Reduced Model** | root→tip 단일 체인, 나머지 관절 잠금 | 팔 단독 제어, 부분 운동학 |
| **Tree Model** | root에서 여러 tip으로 분기 | 멀티핑거 핸드 제어 |

Reduced/Tree 모델은 `pinocchio::buildReducedModel()`을 사용하여 지정된 체인 외부의 관절을 `lock_reference_config` 값으로 고정합니다. Mimic 관절은 자동으로 passive로 처리되어 잠금 대상에 포함됩니다.

## RT-Safety Guarantees

`RtModelHandle`의 모든 compute 함수는 다음을 보장합니다:

- **Zero heap allocation** - 모든 버퍼가 생성자에서 사전 할당
- **No exceptions** - 모든 compute 함수에 `noexcept` 선언
- **No mutex/lock** - thread-per-handle 모델, Model은 `const` 공유
- **No I/O** - 파일/네트워크 접근 없음

초기화(Non-RT)와 제어 루프(RT)를 명확히 분리하는 패턴을 따릅니다:

```
[Phase 1] Non-RT 초기화
  ├─ PinocchioModelBuilder 생성 (URDF 파싱, 모델 빌드)
  ├─ RtModelHandle 생성 (버퍼 사전 할당)
  └─ Frame ID 캐싱

[Phase 2] RT 제어 루프 (noexcept)
  ├─ ComputeForwardKinematics(q)
  ├─ ComputeJacobians(q)
  ├─ GetFrameJacobian(frame_id, ref, J)
  ├─ ComputeNonLinearEffects(q, v)
  └─ 제어 법칙 계산 → 토크 출력

[Phase 3] 정리
  └─ RAII 자동 해제
```

## Examples

예제 실행:

```bash
# 기본 사용법 (FK, Jacobian, 비선형 효과)
ros2 run rtc_urdf_bridge example_basic_usage config/serial_arm_config.yaml

# 서브모델 추출 및 축소 모델
ros2 run rtc_urdf_bridge example_submodel config/serial_arm_config.yaml

# 트리모델 (멀티핑거 핸드)
ros2 run rtc_urdf_bridge example_tree_model config/hand_tree_config.yaml

# RT 통합 패턴
ros2 run rtc_urdf_bridge example_rt_integration config/serial_arm_config.yaml
```

## Logging

`rtc_urdf_bridge`는 init-time 진단을 위한 계층적 sub-logger를 제공합니다. 모든 로그는 라이브러리 사용자의 ROS2 노드 로깅 파이프라인을 통해 발사되며, RT-safe `RtModelHandle` 메서드(noexcept hot path)는 절대 로깅하지 않습니다.

### Sub-logger 네임스페이스

| Sub-logger | 모듈 | 용도 |
|---|---|---|
| `urdf.analyzer` | `UrdfAnalyzer` | URDF/xacro 파싱, 그래프 구축, 관절 분류 |
| `urdf.builder` | `PinocchioModelBuilder` | full/sub/tree 모델 빌드, YAML 로드, 폐쇄 체인 |
| `urdf.chain` | `KinematicChainExtractor` | 체인/트리 추출, 잠금 관절 계산 |
| `urdf.xacro` | `ProcessXacro` | popen 기반 xacro 전처리 |
| `urdf.handle` | `RtModelHandle` | 생성/해제 단계만 (RT 경로 외) |

### 로깅 doctrine

| Severity | 사용처 |
|---|---|
| `INFO` | 1회성 모델 빌드 요약: full/sub/tree 모델 등록, 폐쇄 체인 등록, URDF 파싱 결과 (관절/링크 카운트) |
| `DEBUG` | xacro 명령 라인, 파일 로드 trace, 체인/트리 추출 디테일, YAML 로드 시작 |
| `ERROR` | 모든 `throw` 직전 발사 — 예외가 uncaught여도 사용자 노드 로그에 남음 |
| `WARN` | (현재 없음) 향후 silent skip 지점이 발견되면 사용 |
| `FATAL` | (사용 안 함) — 라이브러리 코드는 예외로 종료 신호 전달 |

핵심 원칙:
- **Hot path 무로그**: `RtModelHandle::Compute*` 계열은 `noexcept`이며 로그 호출 금지.
- **Init-only**: 모든 instrumentation은 생성자/빌드 파이프라인 1회성. throttle 불필요.
- **Throw + ERROR 쌍**: 예외 메시지를 로그로도 발사해 uncaught 시 가시성을 보장.

### 런타임 필터링

라이브러리 사용 노드에 sub-logger 레벨을 변경하면 됩니다 (노드명은 라이브러리를 로드한 ROS2 노드 이름):

```bash
# 모든 urdf.* sub-logger를 DEBUG로
ros2 service call /<node>/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'urdf.analyzer', level: 10},
              {name: 'urdf.builder',  level: 10},
              {name: 'urdf.chain',    level: 10},
              {name: 'urdf.xacro',    level: 10}]}"

# xacro 전처리 트레이스만
ros2 service call /<node>/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'urdf.xacro', level: 10}]}"
```

콘솔에서 sub-logger 이름이 보이도록 환경 변수를 설정하면 디버깅이 편합니다:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

## License

MIT
