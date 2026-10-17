# rtc_urdf_bridge

URDF 를 파싱해 Pinocchio 모델을 만드는 robot-agnostic 라이브러리 (+ 폐쇄 체인 시각화 노드 1개). **유형**: 라이브러리 패키지

## 개요

URDF (또는 xacro) 와 YAML 설정에서 Full / Reduced (직렬 서브체인) / Tree (분기) Pinocchio 모델을 런타임에 만든다. 링크·관절 이름이나 로봇 구조를 코드에 박지 않고 모두 URDF 에서 읽는다. passive · mimic · 폐쇄 체인 관절은 URDF 메타데이터와 sidecar YAML 에서 자동 분류한다.

```
YAML → PinocchioModelBuilder → UrdfAnalyzer (파싱 · 토폴로지)
                             → KinematicChainExtractor (서브체인 · 트리)
                             → Pinocchio Model (Full / Reduced / Tree)
                             → RtModelHandle (RT-safe FK · Jacobian · 동역학)
```

- 소비자: [rtc_controller_manager](../rtc_controller_manager/README.md) (공유 builder), [rtc_digital_twin](../rtc_digital_twin/README.md) (`closure_state_publisher`)
- SE(3) pose · twist error 는 [rtc_math](../rtc_math/README.md) 담당이다. 이 패키지는 모델 구축만 한다

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 소비 패키지에서:

```cmake
find_package(rtc_urdf_bridge REQUIRED)
ament_target_dependencies(my_target rtc_urdf_bridge)   # package.xml 에 <depend>rtc_urdf_bridge</depend>
```

```cpp
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
namespace rub = rtc_urdf_bridge;

rub::PinocchioModelBuilder builder("/path/to/config.yaml");   // [non-RT] 파싱 · 모델 빌드
rub::RtModelHandle handle(builder.GetReducedModel("arm"));    // [non-RT] 버퍼 사전 할당
handle.ComputeForwardKinematics(q);                            // [RT] noexcept, 힙 할당 없음
```

예제 실행. 설정 YAML 의 `urdf_path` 는 YAML 파일 위치 기준 상대 경로이고 `../test/urdf/…` 를 가리킨다. `test/` 는 설치되지 않으므로 **소스 트리의 config 를 절대경로로** 넘긴다 (`ros2 run` 은 환경을 source 한 셸에서).

```bash
CFG=~/ros2_ws/rtc_ws/src/rtc-framework/rtc_urdf_bridge/config
ros2 run rtc_urdf_bridge example_basic_usage    $CFG/serial_arm_config.yaml   # FK · Jacobian · nle
ros2 run rtc_urdf_bridge example_submodel       $CFG/serial_arm_config.yaml   # 서브모델 축소
ros2 run rtc_urdf_bridge example_tree_model     $CFG/hand_tree_config.yaml    # 멀티핑거 트리
ros2 run rtc_urdf_bridge example_rt_integration $CFG/serial_arm_config.yaml   # RT 통합 패턴
```

## 인터페이스

### 공개 헤더

모두 `include/rtc_urdf_bridge/` 아래. 시그니처와 규약은 헤더 Doxygen 이 SSoT 다.

| 헤더 | 역할 |
|---|---|
| `types.hpp` | 데이터 구조체 · 열거형 (`ModelConfig`, `JointRole`, `PassiveSubtype`) |
| `urdf_analyzer.hpp` | URDF 파싱, 링크-관절 그래프, 관절 분류, 경로 · LCA · 루프 탐지 |
| `kinematic_chain_extractor.hpp` | 서브모델 (root→tip) · 트리모델 (root→여러 tip) 추출, 잠금 관절 계산 |
| `pinocchio_model_builder.hpp` | YAML → Full / Reduced / Tree / actuated 모델 + 구속 모델 |
| `rt_model_handle.hpp` | RT-safe FK · Jacobian · RNEA · ABA · 질량행렬 · payload regressor |
| `pinocchio_cache.hpp` / `reduced_dynamics_provider.hpp` | tick 당 1회 FK·J·M·h·g 공유 RT 캐시 / 축약 동역학 주입점 |
| `closed_chain_model.hpp` | `BuildClosedChainModelFromExtendedUrdf` 통합 loader |
| `closure_yaml_loader.hpp` · `constraint_builder.hpp` | sidecar 파서 / `ClosedChainInfo`→`RigidConstraintModel` |
| `loop_verification.hpp` · `loop_projection.hpp` | closure error · `Jc` rank · Delassus / q·v loop-consistent 사영 (로드 타임) |
| `closed_chain_handle.hpp` | 폐쇄 체인 축약 동역학 질의 (**non-RT**) |
| `rt_closed_chain_handle.hpp` | 폐쇄 체인 RT-safe FK + 축약 동역학 (warm-start + 고정 K 사영) |
| `inertial_validation.hpp` | 관성 실현가능성 판정 (V5 / V6 로드 게이트) |
| `xacro_processor.hpp` · `urdf_logging.hpp` | xacro 전처리 (`ProcessXacro`) / `urdf.*` sub-logger |

### 모델 종류

| 종류 | 설명 | 용도 |
|---|---|---|
| Full | URDF 의 모든 관절 | 전체 로봇 |
| Reduced | root→tip 단일 체인, 나머지는 `lock_reference_config` 값으로 잠금 | 팔 단독 · 부분 운동학 |
| Tree | root 에서 여러 tip 으로 분기 | 멀티핑거 핸드 |

mimic 관절은 자동으로 passive 가 되어 잠금 대상에 든다.

### 관절 분류 규칙

`UrdfAnalyzer` 가 파싱 직후 아래 순서로 결정한다 (위에서 매치되면 종료). role 은 topology (`UrdfJointType`) 와 직교한다.

| 조건 | Role | Subtype |
|---|---|---|
| URDF `type == "fixed"` | kFixed | kNone |
| `<mimic>` 태그 | kPassive | kMimic |
| closed-chain 루프 경로에 포함 | kPassive | kClosedChain |
| 생성자 `passive_hints` (YAML `passive_joints`) 에 명시 | kPassive | kFree |
| 그 외 non-fixed | **kActive** | kNone |

- 기본값은 active 다. `<transmission>` 태그는 분류에 쓰이지 않는다
- mimic 과 closed-chain 이 겹치면 kMimic 우선 + 경고
- active 인데 physics 가 없으면 경고 (예외 아님). `has_physics` = `<limit>` 존재 AND `effort > 0` AND (`continuous` 이거나 `velocity > 0`). `continuous` 는 position limit 이 없어도 된다

### 관성 게이트 (V5 / V6)

full 모델을 세운 직후 fixed joint 를 흡수한 **composite body 의 관성** 을 판정한다. `PinocchioModelBuilder` 와 `BuildClosedChainModelFromExtendedUrdf` 가 같은 게이트를 통과한다.

- **V6** (물리적으로 불가능 — 비유한 값, 음의 질량, 음의 고유값, 삼각부등식 위반, 질량 0 + 관성 ≠ 0): `std::runtime_error` 로 **로드 실패**
- **V5** (movable body 의 composite mass 와 관성이 함께 0): **경고** 만 낸다. 관성에 의존하는 소비자는 `PinocchioModelBuilder::IsFullModelDynamicsCapable()` 로 자기 검사한다

판정 기준 · 허용오차 (`kInertialRelTol`) 의 실측 근거는 [inertial_validation.hpp](include/rtc_urdf_bridge/inertial_validation.hpp) 가 SSoT 다.

### RtModelHandle

모든 compute 함수는 `noexcept` 이고 생성자에서 사전 할당한 버퍼만 쓴다 (힙 할당 · 뮤텍스 · I/O 없음, thread-per-handle). 초기화는 non-RT, compute 만 RT 루프에서 호출한다. 규칙 전문은 [invariants.md](../agent_docs/invariants.md) RT-1~RT-10.

```cpp
handle.ComputeJacobians(q);
Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, handle.nv());
handle.GetFrameJacobian(frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
handle.ComputeNonLinearEffects(q, v);          // nle = C(q,v)v + g(q)
handle.ComputeInverseDynamics(q, v, a);        // RNEA
handle.ComputePayloadRegressor(q, v, a, fid);  // Y_L (nv × 10), 선택
```

Jacobian 의 행 · 열 순서와 `ref_frame` 별 의미, payload regressor 의 열 · 행 순서는 `rt_model_handle.hpp` 의 Doxygen 이 SSoT 다. `v = a = 0` 이면 `I` 6열이 정확히 0 이다 (중력만으로는 회전 관성이 관측되지 않음).

### ClosedChainHandle (non-RT) · RtClosedChainHandle (RT)

closed-chain 로봇을 serial chain 과 같은 인터페이스로 다루는 핸들이다. actuated 좌표 `q_a` 만 넘기면 passive DoF 를 loop 구속에 사영하고, 독립 좌표 기준 `M_a` · `g_a` · `h_a` · `J_a` · FK 를 돌려준다. 구속이 없으면 full 모델과 같은 값이다. 특이 조립 형상은 NaN 대신 `Status::singular` 로 알린다.

```cpp
rub::PinocchioModelBuilder builder(config);   // closure_yaml_path 설정
rub::ClosedChainHandle handle(builder);
handle.Update(q_a, v_a);                      // [non-RT] 반복 사영 · SVD
const auto & M = handle.GetMassMatrix();      // n_a × n_a
```

> **`ClosedChainHandle::Update()` 는 non-RT** 다. 반복 사영 · SVD 를 하므로 RT 핫패스에서 매 tick 호출하지 않는다. RT 에서는 `RtClosedChainHandle::UpdateDynamics(v_a)` 를 쓰고, 개방 체인은 `RtModelHandle` 이다.

소비자 배선용으로 `IsFrameDownstreamOfLoop(fid)` 는 프레임 상류에 loop-passive 관절이 있는지 알려 준다. loop 상류가 없는 프레임은 `RtModelHandle` 로 두고, 있는 프레임만 closed-chain FK 로 배선한다.

## 설정

모델 설정 YAML (`PinocchioModelBuilder(path)`) 템플릿은 [config/](config/) 에 있다 (`serial_arm_config.yaml` · `hand_tree_config.yaml` · `four_bar_config.yaml`).

### 직렬 · 트리 · 인라인 폐쇄 체인

```yaml
urdf_path: "path/to/robot.urdf"      # 상대 경로는 이 YAML 위치 기준
root_joint_type: "fixed"             # fixed | floating

sub_models:                          # root → tip 단일 체인
  - {name: arm, root_link: base_link, tip_link: tool_link}
tree_models:                         # root → 여러 tip
  - name: hand
    root_link: palm_link
    tip_links: [thumb_tip, index_tip, middle_tip, ring_tip]
closed_chains:                       # inline (closure_yaml_path 미설정 시)
  - name: four_bar_loop
    link_a: link_b
    link_b: link_c
    contact_type: "6D"               # 6D | 3D
    offset_b: {xyz: [0.3, 0, 0], rpy: [0, 0, 0]}
    baumgarte_kp: 10.0
    baumgarte_kd: 6.32
```

### Extended-URDF closed-chain (sidecar `<name>.closure.yaml`)

권장 방식이다. URDF 는 순수 spanning-tree 로 두고 (표준 `pinocchio::urdf::buildModel` 이 그대로 소비) loop closure 는 sidecar 로 분리한다. `frame` 참조가 1순위이고 `link + origin` 은 fallback 이다 (origin 은 해당 link frame 기준). `type` 은 필수다.

```yaml
# four_bar.closure.yaml (URDF 와 같은 디렉토리)
name: four_bar
closures:
  - name: fourbar_loop_0
    type: contact_3d           # contact_3d | contact_6d (필수)
    frame_1: c1                # 1순위: URDF frame 이름
    frame_2: c2
    reference_frame: LOCAL     # LOCAL | LOCAL_WORLD_ALIGNED (WORLD 미지원)
    baumgarte: { Kp: 20.0, Kd: 5.0 }   # Kd 생략 시 2√Kp
actuation:
  actuated_joints: [joint_a]   # 나머지 non-fixed 는 passive
joint_substitutions: []
```

builder 에서 켜는 법은 `ModelConfig::closure_yaml_path` (YAML 키 `closure_yaml_path`, 상대 경로는 config YAML 기준, inline `closed_chains` 보다 우선) 이다. 결과는 아래로 조회한다.

```cpp
builder.GetConstraintModels();          // RigidConstraintModel 목록
builder.GetClosureReferenceConfig();    // loop-consistent q_ref
builder.GetClosureActuatedJointIds();   // actuated_joints → JointIndex
builder.GetClosurePassiveLockNames();   // loop-passive 관절 (movable − actuated)
builder.IsClosureReferenceAcceptable(); // false 면 q_ref 사용 금지
builder.IsClosureReferenceSingular();   // true 면 q_ref 를 operating config 로 쓰지 말 것
```

bringup 에서는 system YAML `urdf:` 블록의 `extended: true` 가 이 경로를 켠다. sidecar 는 URDF 옆 `<stem>.closure.yaml` 이고 `closure_path:` 로 override 한다 ([rtc_controller_manager](../rtc_controller_manager/README.md)). `extended: true` 인데 sidecar 가 없으면 `RCLCPP_ERROR` 후 loop constraint 없이 진행한다.

> **passive-lock 계약 (필독).** `closure_yaml_path` 가 설정되면 builder 는 full 모델의 movable 관절 중 `actuated_joints` 에 없는 **모든 관절을 loop-passive 로 간주해 모든 reduced / tree 서브모델에서 잠근다** (spanning-tree URDF 에는 loop 이 없어 sidecar 가 유일한 출처다). `actuated_joints` 는 그 URDF 의 **완전한 active 집합** 이어야 한다. arm + closed-chain hand 를 병합했다면 arm 관절도 넣는다. 빠진 관절은 조용히 잠겨 컨트롤러 DOF 가 바뀐다. 빌드 로그 `Extended-URDF closure passive-lock: N joint 잠금 (movable M − actuated K)` 의 N 이 예상 passive 수와 맞는지 확인한다.

**조립 분기.** 점 구속으로 닫은 loop 은 조립 분기가 둘 이상이고 residual 로는 틀린 분기를 못 가린다. 스트리밍 소비자는 `ProjectPassiveWithContinuation` 을 쓰고, RT 경로는 tick 당 seed 증분을 클램프한다. 규범은 [invariants.md](../agent_docs/invariants.md#num-5-세부-스펙) NUM-5.

### 전체 키 참조

```yaml
urdf_xml_string: "<robot>…</robot>"   # urdf_path 대신 (둘 중 하나 필수)
closure_yaml_path: "path/to/robot.closure.yaml"
passive_joints: [joint_a, joint_b]    # passive 강제 (kFree)
lock_reference_config:                # 잠금 관절 기준값 [rad]
  joint_name: 0.5
extra_frames:                         # 추가 frame (관성 0, OP_FRAME). key 가 frame 이름
  frame_name:
    parent: "parent_frame"            # 링크 · 관절 · frame 이름
    xyz: [x, y, z]                    # m, 부모 frame 기준
    rpy: [r, p, y]                    # rad, R = Rz·Ry·Rx
    provisional: true                 # 기본 true — 실기 소비자는 거부해야 한다
```

`extra_frames` 는 full 모델 빌드 직후 붙고 sub / tree / actuated 모델이 상속한다 (새 frame 은 끝에 붙어 기존 frame id 는 불변). 없는 부모 · 중복 이름 · 비유한 값은 생성 실패다. 로봇 config 에서는 같은 키가 `urdf.extra_frames.<name>.*` 이고 `rtc_controller_manager` 가 읽는다.

### Nodes

#### closure_state_publisher

Extended-URDF 폐쇄 체인을 RViz 에 시각화하는 off-RT 노드. actuated `JointState` 를 받아 actuated q 를 고정하고 passive q 를 closure 구속으로 풀어 (`ProjectPassiveWithContinuation`, warm-start) loop-consistent full q 를 publish 한다. `robot_state_publisher` 가 이를 TF 로 전개해 loop 가 닫힌 채 렌더링된다. closure 가 없는 로봇은 이 노드를 띄우지 않는다 ([rtc_digital_twin](../rtc_digital_twin/README.md) 이 조건부로 launch).

```bash
ros2 run rtc_urdf_bridge closure_state_publisher --ros-args \
  -p urdf_path:=/path/to/robot.urdf -p closure_path:=/path/to/robot.closure.yaml
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `urdf_path` | `""` | URDF / xacro 경로 (또는 `robot_description` 중 하나 필수) |
| `robot_description` | `""` | URDF XML 문자열 (`urdf_path` 대안) |
| `closure_path` | `""` | **필수** — sidecar `<stem>.closure.yaml` 경로 |
| `root_joint_type` | `"fixed"` | `fixed` \| `floating` |
| `input_topic` | `/digital_twin/actuated_joint_states` | actuated `JointState` 입력 |
| `output_topic` | `/digital_twin/joint_states` | loop-consistent full `JointState` 출력 |
| `warn_on_singular` | `true` | 기준 형상이 특이하면 기동 경고 |
| `max_iterations` / `tolerance` | `100` / `1e-10` | 사영 반복 상한 / strict 수렴 임계 |
| `acceptance_tolerance` | `1e-6` | 결과 수용 임계 [m]. strict 미달이어도 이 이내면 커밋한다. 정지를 풀려고 `tolerance` 를 올리지 말 것 |
| `max_actuated_increment` | `0.05` | continuation sub-step 당 actuated 증분 상한 [rad]. `≤0` 이면 비활성 |

수용 임계를 넘거나 특이한 프레임은 직전 해를 hold (NaN 미발행) 하고 THROTTLE WARN 을 낸다.

## 디렉토리

```
rtc_urdf_bridge/
├── include/rtc_urdf_bridge/
├── src/
├── config/
├── examples/
└── test/
```

`test/urdf/` 에 테스트용 URDF · `*.closure.yaml` · MJCF 가 있다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| 예제가 `urdf` 파일을 못 연다 | 설정 YAML 의 `../test/urdf` 가 설치 트리에 없음 | 소스 트리 config 를 절대경로로 넘긴다 |
| 모델 로드가 `runtime_error` (inertia) | V6 관성 위반 | URDF 의 해당 body 관성을 고친다. 로그에 body · 관절 이름이 나온다 |
| 컨트롤러 DOF 가 예상보다 적다 | sidecar `actuated_joints` 가 불완전해 관절이 passive 로 잠김 | passive-lock 로그의 N 확인, arm 관절 포함 |
| `closure_state_publisher` 가 값을 hold | 수용 임계 초과 또는 특이 형상 | WARN 로그, `acceptance_tolerance` · 기준 형상 확인 |

### 로깅

init-time 진단용 sub-logger 를 쓴다 (`RtModelHandle` 의 RT 메서드는 로깅하지 않는다). 명명 규칙은 [conventions.md](../agent_docs/conventions.md#logging) 를 따른다.

| Sub-logger | 모듈 |
|---|---|
| `urdf.analyzer` | `UrdfAnalyzer` — 파싱 · 그래프 · 관절 분류 |
| `urdf.builder` | `PinocchioModelBuilder` — 모델 빌드 · YAML · 폐쇄 체인 |
| `urdf.chain` | `KinematicChainExtractor` |
| `urdf.xacro` | `ProcessXacro` |
| `urdf.handle` | `RtModelHandle` 생성 · 해제 (RT 경로 외) |

모든 `throw` 직전에 ERROR 를 함께 남긴다. 레벨은 라이브러리를 로드한 노드에 설정한다.

```bash
ros2 service call /<node>/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'urdf.builder', level: 10}, {name: 'urdf.xacro', level: 10}]}"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"   # 콘솔에 logger 이름 표시
```

## 더 읽기

- 관성 허용오차 근거: [inertial_validation.hpp](include/rtc_urdf_bridge/inertial_validation.hpp) `kInertialRelTol`
- Jacobian · payload regressor 규약: `rt_model_handle.hpp` Doxygen
- 폐쇄 체인 수치 규범: [invariants.md](../agent_docs/invariants.md#num-5-세부-스펙)

## 의존성

[package.xml](package.xml) · MIT. pinocchio · Eigen3 · yaml-cpp · tinyxml2 · rclcpp / sensor_msgs (`closure_state_publisher` 만) · xacro (exec).
