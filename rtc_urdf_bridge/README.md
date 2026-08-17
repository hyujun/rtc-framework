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
│   ├── rt_model_handle.hpp             # RT-safe 계산 래퍼
│   ├── closure_yaml_loader.hpp         # Extended-URDF sidecar 파서
│   ├── constraint_builder.hpp          # ClosedChainInfo→RigidConstraintModel (§4b)
│   ├── inertial_validation.hpp         # 관성 물리 실현가능성 판정 (V5/V6 로드 게이트)
│   ├── loop_verification.hpp           # closure error / Jc rank·Delassus
│   ├── loop_projection.hpp             # q/v loop-consistent 사영 (로드 타임)
│   ├── closed_chain_model.hpp          # 통합 loader (BuildClosedChainModelFromExtendedUrdf)
│   ├── closed_chain_handle.hpp         # closed-chain 축약 동역학 질의 (M/g/h/J/FK, non-RT)
│   ├── rt_closed_chain_handle.hpp      # RT-safe closed-chain FK + 축약 동역학 (warm-start + 고정 K 사영, J_a, M_a/g_a/h_a)
│   └── closure_state_publisher.hpp     # Extended-URDF 폐쇄 체인 시각화 노드 (off-RT)
├── src/
│   ├── urdf_analyzer.cpp
│   ├── kinematic_chain_extractor.cpp
│   ├── pinocchio_model_builder.cpp
│   ├── inertial_validation.cpp
│   ├── rt_model_handle.cpp
│   ├── closure_state_publisher{,_main}.cpp  # 노드 구현 + 실행 진입점
│   ├── closure_yaml_loader.cpp         # Extended-URDF sidecar 파서
│   ├── constraint_builder.cpp          # ClosedChainInfo→RigidConstraintModel
│   ├── loop_verification.cpp           # closure error / Jc rank·Delassus
│   ├── loop_projection.cpp             # q/v loop-consistent 사영 (로드 타임)
│   ├── closed_chain_model.cpp          # 통합 loader
│   ├── closed_chain_handle.cpp         # closed-chain 축약 동역학 질의 (non-RT)
│   └── rt_closed_chain_handle.cpp      # RT-safe closed-chain FK + 축약 동역학
├── config/                             # YAML 설정 템플릿
│   ├── serial_arm_config.yaml
│   ├── hand_tree_config.yaml
│   └── four_bar_config.yaml
├── examples/                           # 사용 예제
│   ├── example_basic_usage.cpp
│   ├── example_submodel.cpp
│   ├── example_tree_model.cpp
│   └── example_rt_integration.cpp
└── test/                               # GTest 테스트 -- 전체 목록은 test/ 디렉토리 참조
    ├── test_urdf_analyzer.cpp
    ├── test_chain_extractor.cpp
    ├── test_model_builder.cpp
    ├── test_joint_classification.cpp   # role/subtype 분류 규칙
    ├── test_load_model_config.cpp      # YAML 로드 + 폐쇄체인/lock 빌드
    ├── test_rt_model_handle.cpp
    ├── test_payload_regressor.cpp     # Y_L 열 순서·origin 기준 관성 원소별 고정
    ├── test_frame_jacobian_fd_oracle.cpp # J 행(linear/angular)·열 순서 중심차분 oracle
    ├── test_inertial_validation.cpp    # 관성 실현가능성 게이트 V5/V6 (사유 분리·scale-aware tol)
    ├── test_real_model_inertial_gate.cpp # 실모델 11종 무경고 통과 + schunk SVH negative fixture
    ├── test_closure_yaml_loader.cpp    # sidecar 파서
    ├── test_constraint_builder.cpp     # endpoint transform / builder
    ├── test_pinocchio_builder_closure.cpp # closure_yaml_path 경로 (Extended-URDF via builder)
    ├── test_loop_closed_chain.cpp      # closure error/projection/rank/dynamics
    ├── test_loop_projection_passive.cpp # actuated 고정 passive 사영 (crank_rocker/four_bar)
    ├── test_closed_chain_handle.cpp    # 축약 M/g/h/J/FK: serial 등가·round-trip·특이 flag
    ├── test_closed_chain_fk_measurement.cpp # frozen-loop FK vs closed-chain FK 실측 오차
    ├── test_rt_closed_chain_handle.cpp # RT-safe FK: converged 등가·J_a·serial 항등·hold·singular·seed-guard·identity-NaN-hold·OOB-getter·조립분기 clamp
    ├── test_rt_closed_chain_alloc.cpp  # RT-1 센서: Update() 힙 할당 0 (전역 new 카운터, 단독 TU)
    ├── test_closure_state_publisher.cpp # 노드 end-to-end (actuated 주입→full q, loop 닫힘)
    ├── test_mjcf_comparison.cpp        # MJCF 규약 교차검증
    ├── test_xacro_processor.cpp        # xacro 전처리
    └── urdf/                           # 테스트용 URDF / closure.yaml / MJCF
```

> **참고** — SE(3) pose/velocity(twist) error 모듈은 `rtc_math` 패키지로 이전되었다
> (`rtc::math::se3`). 이 패키지는 URDF→Pinocchio 모델 *구축*만 담당하며, 모델을 써서
> task-space 제어 오차를 계산하는 코드는 [rtc_math](../rtc_math/README.md) 참조.

## Dependencies

| 패키지 | 용도 |
|--------|------|
| pinocchio | 강체 동역학 알고리즘 (FK, Jacobian, RNEA, ABA) |
| Eigen3 | 선형대수 연산 |
| yaml-cpp | YAML 설정 파일 파싱 |
| tinyxml2 | URDF XML 파싱 |
| rclcpp | `closure_state_publisher` 노드 (ROS2 C++ 클라이언트) |
| sensor_msgs | `closure_state_publisher` 노드의 `JointState` 입출력 |
| xacro (exec_depend) | xacro 전처리 (`ProcessXacro`, popen 기반) |
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
auto ctrl_model = builder.GetActuatedModel();       // actuated 제어 모델 (extended 시)

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

// Payload 관성 회귀자 (#455 Layer 2B) — Y_L = J_F(LOCAL)ᵀ · frameBodyRegressor(F)
handle.ComputePayloadRegressor(q, v, a, frame_id);  // nv × 10
Eigen::Ref<const Eigen::MatrixXd> Y = handle.GetPayloadRegressor();
```

`GetFrameJacobian` 의 출력 `J` 는 **행이 Pinocchio spatial 규약** (`rows 0..2 = linear`,
`rows 3..5 = angular` — 각속도를 먼저 쌓는 Featherstone 관례와 반대) 이고 **열은 Pinocchio
v-공간 순서**입니다. `SetJointOrder` 로 device 순서를 설정해도 이 출력은 재배열되지 않으며,
재배열되는 것은 입력 `q` 뿐입니다 (`ComputePayloadRegressor` 와 같은 비대칭). 블록을 맞바꾸거나
열을 device 순서로 착각해도 결과는 유한하고 매끄럽게 틀려 NaN·노름 게이트에 걸리지 않으므로,
`test_frame_jacobian_fd_oracle.cpp` 가 자코비안을 전혀 쓰지 않는 **중심차분 oracle** 로 두 계약을
고정합니다 — `test_rt_model_handle.cpp` 의 `ReorderedJacobianMatchesDirect` 는 자기일치 대조라
행이 뒤집혀도 green 으로 남습니다.

행 **순서**와 달리 linear 행이 *무엇의* 속도인지는 `ref_frame` 이 정합니다. `LOCAL` 과
`LOCAL_WORLD_ALIGNED` 는 프레임 원점의 속도(표현 축만 다름)지만, `WORLD` 는 **프레임 원점이
아니라 world 원점에 놓인 점의 속도** `v_O = ṗ + p × ω` 입니다. LWA 를 기대하고 `WORLD` 의
`topRows(3)` 을 TCP 선속도로 쓰면 `|p × ω|` 만큼 조용히 어긋납니다 (저장소 안에 `WORLD`
호출자는 없지만 API 가 셋을 노출하므로 테스트가 셋 다 고정합니다).

### 관성 실현가능성 게이트 (V5 / V6)

`PinocchioModelBuilder` 는 full 모델을 세운 직후 **모든 movable body 의 composite 관성**이 강체를
기술할 수 있는 값인지 판정합니다 (`inertial_validation.hpp`). 판정 대상이 URDF 의 개별 `<link>` 가
아니라 **fixed joint 를 흡수한 뒤의 `Model::inertias[]`** 인 것이 핵심입니다 — 그것이 `M(q)` 가
실제로 조립되는 값이라, 게이트의 의미가 "이 모델로 동역학을 돌려도 되는가" 로 정확히 떨어집니다.

두 레인은 **심각도가 아니라 종류가 다릅니다.**

| 레인 | 판정 | 처분 |
|---|---|---|
| **V6** — 물리적으로 불가능 | 비유한 값, `mass < 0`, 음의 고유값, 삼각부등식 위반 (`I1+I2 < I3`) | `std::runtime_error` 로 **로드 실패** |
| **V5** — 물리적으로 불완전 | movable body 의 composite `mass == 0` | **경고 + 술어** (`IsFullModelDynamicsCapable()`) |

V5 가 로드를 막지 않는 이유는 이 패키지가 kinematics 전용 소비자(FK / Jacobian / IK)와 dynamics
소비자를 함께 섬기기 때문입니다. 질량 없는 movable body 는 `M(q)` 를 그 DoF 에서 특이하게 만들지만
기하 질의에는 무해하므로, throw 하면 멀쩡한 kinematic 모델을 못 열게 됩니다. 관성에 의존하는
소비자만 `IsFullModelDynamicsCapable()` 로 자기 검사하십시오.

허용오차는 **주모멘트 크기로 정규화**됩니다 (`kInertialRelTol`). 저장소 실모델의 주모멘트가
`1.03e-7 ~ 7.07e-1` 로 6.8 decade 에 걸쳐 있어 절대 tol 로는 양 끝을 동시에 만족시킬 수 없기
때문입니다. **등호는 허용**됩니다 — 얇은 판·막대는 `I1+I2 = I3` 이 정확히 성립합니다.

오프라인 저작 게이트인 `rtc_tools` 의 `compare_mjcf_urdf` 와는 **축이 다릅니다**: 그쪽은 선언값과
collision 기하 추정값의 *비율*을 보는 informational WARN(외부 정합)이고, 여기는 기하 없이 텐서
단독의 실현가능성(내부 정합)을 봅니다. 한쪽이 다른 쪽을 대체하지 못합니다.

`ComputePayloadRegressor` 는 프레임에 강체로 매달린 payload 의 10-parameter 관성 집합 `φ_L` 에
대해 `τ_payload = Y_L · φ_L` 을 만족하는 회귀자를 만듭니다. 두 계약이 조용히 틀리기 쉬워
`test_payload_regressor.cpp` 가 원소별로 고정합니다:

- **열 순서는 `pinocchio::Inertia::toDynamicParameters()`** — `[m, m·c, I_xx, I_xy, I_yy, I_xz,
  I_yz, I_zz]` 로 **lower-triangular column-major** 이며 (`I_xz` 가 `I_yy` 뒤), 그 `I` 는
  **프레임 origin 기준**이지 CoM 기준이 아닙니다.
- **행은 PINOCCHIO 순서** (`GetTau()` 와 동일). 반면 `q`/`v`/`a` 입력은 **device 순서**로 받아
  내부에서 재배열합니다 — 이 비대칭은 이 클래스 전반의 규약입니다.

`v = a = 0` (준정적) 으로 부르면 **`I` 6열이 정확히 0** 이 됩니다. 중력만으로는 회전 관성이
관측되지 않기 때문이며 (자세를 60개 쌓아도 rank 는 10 이 아니라 4), Layer 2B 추정기가 4개만
식별하는 근거입니다.

### 5. ClosedChainHandle (closed-chain 축약 동역학 질의, non-RT)

closed-chain 로봇을 **serial chain 과 동일한 인터페이스**로 다루기 위한 핸들입니다.
호출자가 actuated(독립) 좌표 `q_a` 만 넘기면, passive DoF 를 loop 구속에 사영(closed-loop
FK)하고 독립 좌표 기준의 **축약된** 관성 `M_a`·중력 `g_a`·비선형효과 `h_a`·프레임
Jacobian `J_a`·FK 를 돌려줍니다. 구속이 없는 serial 로봇이면 축약이 항등이 되어 값이
full 모델과 같아집니다. computed-torque / task-space 컨트롤러가 loop 로봇의 동역학량을
serial 과 똑같이 얻도록 준비하는 계층입니다 (컨트롤러·TSID 통합은 이 위에 얹힘).

속도 축약 map `G`(nv×n_a, `Jc·G=0`)로 `M_a=GᵀMG`, `g_a=Gᵀg`, `J_a=J_full·G`,
`h_a=Gᵀ·rnea(q, v_full, a_drift)` (drift 항 `γ=J̇c·v` 는 동일 `Jc` 함수의 중앙차분).
planar `contact_3d` 처럼 구속이 redundant(rank<m) 여도 damped pseudo-inverse 로 처리하며,
특이 조립형상은 NaN 대신 `Status::singular` 로 flag 합니다.

> **non-RT.** `Update()` 는 반복 사영·SVD·행렬곱을 수행하므로 RT 핫패스에서 매 tick 호출
> 금지 — init / 저주기 query / off-RT 컨트롤러 준비용입니다. RT 핫패스용 축약 동역학은
> `RtClosedChainHandle::UpdateDynamics(v_a)` + `GetMassMatrix/GetGeneralizedGravity/
> GetNonLinearEffects` 가 동일 수학을 warm-start + 고정 K 사영 + damped 정규방정식 LDLT 로
> **힙 할당 없이** 제공합니다 (non-RT SVD damped-pinv 와 수치 등가; #120). loop-consistent
> frame drift `J̇_a·v_a` 는 `GetFrameClassicalAccelerationDrift(fid, ref, out)` — 같은 tick 의
> `UpdateDynamics()` 가 구한 구속-정합 drift 가속으로 2차 FK 한 상태를 읽습니다 (#173).
> 개방 체인 RT 는 `RtModelHandle`.

```cpp
#include "rtc_urdf_bridge/closed_chain_handle.hpp"

rub::PinocchioModelBuilder builder(config);   // closure_yaml_path 설정 (Extended-URDF)
rub::ClosedChainHandle handle(builder);

handle.Update(q_a_span, v_a_span);            // [non-RT] actuated q_a → 사영 + 축약 재계산
const auto& M = handle.GetMassMatrix();        // n_a × n_a
const auto& g = handle.GetGeneralizedGravity();// n_a
Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, handle.nv_independent());
handle.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J);   // 6 × n_a
```

**Topology 판정 (소비자 배선용).** closed-chain FK 는 loop-passive 관절이 프레임 상류에
있을 때만 frozen-loop 근사(reduced/tree 서브모델)와 달라집니다. `IsFrameDownstreamOfLoop(fid)`
는 프레임의 support(조상) 경로에 movable & non-actuated(=loop-passive) 관절이 있는지로 이를
판정합니다 — 소비자는 이 helper 로 **필요한 프레임만** `ClosedChainHandle` FK 로 배선하고,
loop 상류가 없는 프레임(예: loop 상류 팔 끝)은 `RtModelHandle` 로 byte-for-byte 유지해 불필요한
non-RT 사영을 회피합니다. 구속 없는 serial 등가이거나 frame 이 범위를 벗어나면 항상 `false`.
loop-passive 관절 이름 목록은 `PinocchioModelBuilder::GetClosurePassiveLockNames()` 로 조회합니다.

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

### Extended-URDF closed-chain (sidecar `<name>.closure.yaml`)

권장 방식. URDF 는 **순수 spanning-tree** 로 유지하고 (표준 `pinocchio::urdf::buildModel`
이 그대로 소비), loop closure 정보는 **별도 sidecar YAML** 로 분리한다. `frame` 참조가
1순위(가장 안전 — joint-frame placement 를 그대로 사용), `link + origin` 은 fallback
이며 origin 은 **해당 link frame 기준**으로 해석된다. `type` 은 필수이며 평면/cross
4-bar 는 과구속 회피를 위해 `contact_3d` 부터 검증한다.

```yaml
# four_bar.closure.yaml (URDF 와 같은 디렉토리)
name: four_bar
closures:
  - name: fourbar_loop_0
    type: contact_3d           # contact_3d | contact_6d (필수)
    frame_1: c1                # 1순위: URDF frame 이름 참조
    frame_2: c2
    # fallback (frame 없을 때만):
    # parent_link: link_b
    # child_link:  link_d
    # parent_origin: { xyz: [0.3, 0, 0], rpy: [0, 0, 0] }   # link frame 기준
    reference_frame: LOCAL     # LOCAL | LOCAL_WORLD_ALIGNED (WORLD 미지원)
    baumgarte: { Kp: 20.0, Kd: 5.0 }   # Kd 생략 시 2√Kp 자동
actuation:
  actuated_joints: [joint_a]   # 나머지 non-fixed 는 passive
joint_substitutions: []
```

```cpp
#include "rtc_urdf_bridge/closed_chain_model.hpp"
// 통합 loader: {Model, RigidConstraintModel 목록, actuated_joint_ids, q_ref} 반환.
auto ccm = rtc_urdf_bridge::BuildClosedChainModelFromExtendedUrdf(urdf_path, closure_yaml_path);
// 검증/사영 (로드 타임 전용):
//   loop_verification.hpp — ComputeClosureErrors / AnalyzeConstraintJacobian (rank·Delassus)
//   loop_projection.hpp   — ProjectToConstraint (q_ref, 전체 q 사영) / ProjectVelocity
//                           ProjectPassiveToConstraint (actuated 고정·passive 만 사영;
//                           actuated 스트림 → loop-consistent full q 재구성, 시각화용)
//                           ProjectPassiveWithContinuation (**스트리밍 소비자의 표준 진입점**)
```

**조립 분기(assembly branch) 주의 — 반드시 continuation 을 쓸 것.** 점(`CONTACT_3D`) 구속으로
닫은 loop 은 4-bar 처럼 조립 분기가 둘 이상이고 **모든 분기가 φ=0 을 정확히 만족**한다. 따라서
`converged` / `‖φ‖` 로는 물리적으로 틀린 분기를 검출할 수 없다 — 분기를 결정하는 것은 residual 이
아니라 seed 에서 해까지의 **homotopy 경로**다. 직전 해에서 크게 떨어진 actuated seed 를 한 번의
사영에 통째로 넘기면 반대편 분기로 착지하고, warm-start 구조상 영구 고정된다 (issue #248).

- 스트리밍 소비자(직전 해가 있는 경우)는 `ProjectPassiveWithContinuation(q_prev, q_target, …)` 을
  쓴다 — actuated 증분을 `kDefaultActuatedIncrement`(0.05) 단위 sub-step 으로 나눠 warm-start 를
  이어간다. `max_actuated_increment ≤ 0` 으로 끄면 단일 사영과 동일하다 (escape hatch).
- `ProjectPassiveToConstraint` 직접 호출은 증분이 작다고 보장될 때만.
- RT 경로(`RtClosedChainHandle`)는 sub-step loop 가 비결정적이라 쓸 수 없다 — 대신 **tick 당 seed
  증분을 균일 스케일로 클램프**하고 그 tick 을 `held` 로 보고해 tick loop 자체를 continuation
  경로로 쓴다 (고정 K 유지, 추가 연산 0). 세부는 [invariants.md](../agent_docs/invariants.md) NUM-5.

**PinocchioModelBuilder 경로 (bring-up SSoT).** raw URDF 뿐 아니라 xacro 도 소비하는
`PinocchioModelBuilder` 는 `ModelConfig::closure_yaml_path` 가 설정되면 (`buildModel` 대신)
전처리된 `full_model_` 위에서 같은 파이프라인을 실행하고 결과를 노출한다. `BuildClosedChainData`
헬퍼가 이 재사용 지점이다 (이미 빌드된 model + `ClosureSpec` → constraints/actuated/q_ref).

```cpp
rtc_urdf_bridge::ModelConfig cfg;
cfg.urdf_path = ".../foo.urdf.xacro";
cfg.closure_yaml_path = ".../foo.closure.yaml";   // 비우면 순수 URDF (기본)
rtc_urdf_bridge::PinocchioModelBuilder builder(cfg);
builder.GetConstraintModels();          // RigidConstraintModel 목록
builder.GetClosureReferenceConfig();    // loop-consistent q_ref (full model)
builder.GetClosureActuatedJointIds();   // actuation.actuated_joints → JointIndex
builder.GetClosurePassiveLockNames();   // loop-passive 관절 이름 (movable − actuated); topology 판정용
builder.IsClosureReferenceConverged();  // strict 수렴 (‖φ‖ < 1e-10) 여부 — 관측 신호
builder.IsClosureReferenceAcceptable(); // false 면 q_ref 사용 금지 (‖φ‖ > 1e-6 또는 비유한, #250)
builder.IsClosureReferenceSingular();   // true 면 q_ref 를 operating config 로 쓰지 말 것
```

integrated_bringup 은 system YAML `urdf:` 블록의 `extended: true` 로 이 경로를 켠다 (sidecar 는
URDF 옆 `<stem>.closure.yaml`; `closure_path:` 로 명시 override 가능). `extended: true` 인데 sidecar
가 없으면 top-level `urdf.path` 경로든 devices-fallback 경로든 동일하게 loud `RCLCPP_ERROR` 를 내고
(RT 모델이 loop constraint 를 잃으므로) bring-up 은 계속된다. §"Adding a New ..." 참조.

> **RT 모델 passive-lock 계약 (필독).** `closure_yaml_path` 가 설정되면 `PinocchioModelBuilder` 는
> **full model 의 movable 관절 중 `actuated_joints` 에 없는 모든 관절을 loop-passive 로 간주해
> 모든 reduced/tree 서브모델에서 잠근다** (spanning-tree URDF 에는 loop 이 없어 analyzer 가 이를
> 분류하지 못하므로 sidecar 가 유일한 출처다). 따라서 `actuation.actuated_joints` 는 그 URDF 의
> **완전한 active 집합**이어야 한다 — arm + closed-chain hand 를 병합했다면 arm 관절도 반드시
> 포함해야 하며, 빠진 관절은 조용히 잠겨 컨트롤러 DOF 가 바뀐다. 빌드 시
> `"Extended-URDF closure passive-lock: N joint 잠금 (movable M − actuated K)"` 로그의 N 이
> 예상 passive 수와 일치하는지 확인해 이 실수를 조기에 잡는다.

계층 계약: YAML I/O·파싱·`buildModel`·constraint 생성·projection 은 **로드 타임**(예외
허용). RT 경로는 `RtModelHandle::ComputeConstraintDynamics` 호출만 (사전 할당 재사용,
noexcept). MJCF 교차검증은 Pinocchio 4.0 파서 한계(`<connect>` 예외 + 2번째 worldbody
branch 미파싱)로 단일 branch 규약 일치 검증에 한정된다 (`test_mjcf_comparison.cpp` 주석 참조).

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

# Extended-URDF closure sidecar (<name>.closure.yaml) 경로.
# 설정 시 순수 spanning-tree URDF + 이 sidecar 로부터 loop constraints + q_ref +
# actuated joints 를 로드한다 (아래 inline closed_chains 대신 사용; 둘 다 설정 시 sidecar 우선).
# 상대 경로는 이 config YAML 파일 위치 기준으로 해석된다.
closure_yaml_path: "path/to/robot.closure.yaml"

# 폐쇄 체인 구속 (inline; closure_yaml_path 미설정 시)
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
  ├─ ComputePayloadRegressor(q, v, a, frame_id)   # 선택 — #455 Layer 2B
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

## Nodes

### closure_state_publisher

Extended-URDF(spanning-tree URDF + `<stem>.closure.yaml`) 폐쇄 체인을 RViz 에 시각화하는
off-RT 노드. actuated `JointState` 스트림을 입력받아 **측정된 actuated q 를 고정**하고 passive q 를
closure 구속으로 풀어(`ProjectPassiveWithContinuation`, warm-start) **loop-consistent full q** 를
만들어 전체 model 관절을 publish 한다 → `robot_state_publisher` 가 TF 로 전개해 loop 가 닫힌 채
렌더링된다. 모델 구축은 `PinocchioModelBuilder`(xacro 전처리 + closure 파이프라인) 재사용.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `urdf_path` | `""` | URDF/xacro 파일 경로 (또는 `robot_description` 중 하나 필수) |
| `robot_description` | `""` | URDF XML 문자열 직접 전달 (`urdf_path` 대안) |
| `closure_path` | `""` | **필수** — Extended-URDF sidecar `<stem>.closure.yaml` 경로 |
| `root_joint_type` | `"fixed"` | root joint 타입 (`fixed` \| `floating`) |
| `input_topic` | `/digital_twin/actuated_joint_states` | actuated `JointState` 입력 |
| `output_topic` | `/digital_twin/joint_states` | loop-consistent full `JointState` 출력 |
| `warn_on_singular` | `true` | 기준 형상 특이 시 기동 경고 |
| `max_iterations` / `tolerance` | `100` / `1e-10` | passive 사영 반복 상한 / strict 수렴 임계 (solver 반복 목표) |
| `acceptance_tolerance` | `1e-6` | 결과 수용 임계 (병진 m, #250). strict 미달이어도 이 이내면 해를 커밋 — URDF 좌표 불일치의 residual floor 대응. **solver 정지 완화 목적으로 `tolerance` 를 올리지 말 것** (초기 residual 이 임계 미만이면 refinement 를 건너뜀) |
| `max_actuated_increment` | `0.05` | continuation sub-step 당 actuated 증분 상한 (rad). 조립 분기 이탈 방지 — 위 "조립 분기 주의" 참조. `≤0` 이면 비활성 |

비수용(acceptance 초과)/특이 프레임은 **직전 loop-consistent 해를 hold**(NaN 미발행)하고 THROTTLE
WARN 을 낸다. strict 미달·acceptance 이내의 residual floor 는 정상 커밋된다 (#250).
closure 비활성(param 미설정) 로봇은 이 노드를 기동하지 않고 digital_twin 이 직접 publish 한다
(opt-in). launch 배선은 [rtc_digital_twin](../rtc_digital_twin/README.md) 참조.

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
