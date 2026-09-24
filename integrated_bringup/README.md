# integrated_bringup


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

로봇별(`config/<robot>/`) **launch, 설정, 데모 컨트롤러** 통합 패키지입니다. 실제 로봇과 MuJoCo 시뮬레이션 모드를 지원하며, CPU 격리, DDS 스레드 핀닝, 세션 기반 로깅을 자동으로 설정합니다.

**핵심 기능:**
- 데모 컨트롤러: `DemoJointController`, `DemoTaskController`, `DemoComplianceController`, `DemoWbcController` (TSID QP whole-body + MPC 통합), `DemoInferenceController` (ONNX 정책) — 등록 목록은 `src/controllers/controller_registration.cpp`
- launch 파일: `robot_ur5e_p1a.launch.py` / `robot_ur5e_p1b.launch.py` (실로봇), `sim_ur5e_p1a.launch.py` / `sim_ur5e_p1b.launch.py` / `sim_iiwa7_leap.launch.py` (MuJoCo)
- GUI 도구 (컨트롤러 튜닝, 모션 에디터)
- 자동 CPU 격리 + DDS 스레드 핀닝
- 세션 디렉토리 자동 생성 및 정리

---

## 패키지 구조

```
integrated_bringup/
├── CMakeLists.txt
├── package.xml
├── include/integrated_bringup/
│   ├── controllers/
│   │   ├── demo_joint_controller.hpp   <- 관절 공간 Quintic 궤적 제어 (로봇+핸드)
│   │   ├── demo_task_controller.hpp    <- 태스크 공간 CLIK 제어 (로봇+핸드)
│   │   ├── demo_compliance_controller.hpp <- 태스크 공간 admittance 바인딩 (§7 법칙 결합)
│   │   ├── demo_inference_controller.hpp <- 학습 정책(ONNX) 바인딩. 코어는 `rtc_controllers/inference/{policy_io,reach_gate}.hpp`. link pose (cache / closed-chain FK, `policy_frame` 기준) · 관절 규약 · reach gate · object pose(TFMessage) 레인을 스스로 소유한다 — object 는 device lane 이 아니라 프레임워크 freshness 게이트가 안 걸리므로 `object_pose.timeout_sec` 이 그 책임을 진다
│   │   ├── demo_wbc_controller.hpp     <- TSID whole-body + MPC 통합
│   │   ├── demo_catching_controller.hpp <- dynamic_catching S5.1 골격: 팔 hold + 손 무성형 계단 + P-1 최소 E-STOP 계약 (훅은 요청만, reset writer 는 RT tick) + 무장 파라미터 `catching.enable`. 실기 config (claim 한 device 가 전부 `mujoco_native` 임을 증명 못 함) 에서 소비 키에 provisional·TBD 가 있으면 DISABLED 로 configure 되고 활성화를 거부한다 (L0 §5.3)
│   │   ├── catching/planner_thread.hpp <- 포구 계획기 스레드 `CatchingPlannerThread` (dynamic_catching S6-A): `PeriodicRtThread` 형제, eventfd `poll` 대기 + `planner.wake_timeout_s` 상한, 본체는 `rtc_controllers` 의 `PlannerCycle::Run`. `mpc` layout role 로 기동 (스레드 이름 `mpc_main`, E-7 결정 J)
│   │   ├── fingertip_counts.hpp        <- DeriveFingertipCounts (inference-group vs sensor-lane fingertip count SSoT, joint/task/wbc 공용)
│   │   └── wbc/                        <- WBC 전용 모듈 헤더
│   │       ├── grasp_target.hpp           <- grasp 목표 pose 구조체 + 외부 명령 enum
│   │       ├── grasp_phase_manager.hpp    <- 5-state grasp FSM (rtc_mpc::PhaseManagerBase 구현)
│   │       └── force_reference_updater.hpp <- Stage A-3 per-contact normal-force PI helper
│   ├── support/                        <- controller 인프라 (로깅 매크로, owned topics, vTCP, log 등록 헬퍼, momentum-observer 배선, compliance §7.3 관절 tail 통계)
│   │   ├── bringup_logging.hpp
│   │   ├── controller_log_registration.hpp
│   │   ├── demo_shared_config.hpp
│   │   ├── owned_topics.hpp
│   │   ├── layout_profile.hpp          <- launch layout profile (#350) 공용 정의: `LayoutProfileDropsMpc` · `ReadLayoutProfile`. `mpc` role 에 스레드를 올리는 두 컨트롤러 (DemoWbc MPC · 포구 계획기) 가 같은 규칙으로 읽는다
│   │   └── virtual_tcp.hpp
│   ├── backends/                       <- DeviceBackend 구현 (sim/robot HW 어댑터, ARCH-3 부합)
│   │   ├── mujoco_native_backend.hpp     <- sim 공용 (JointState in+out, fingertip WrenchStamped in) ◇
│   │   ├── ur_driver_native_backend.hpp  <- UR5e robot (JointState in, Float64MultiArray out)
│   │   └── udp_hand_native_backend.hpp   <- assm_v1 hand robot (joint+motor+sensor lane)
│   └── logging/                        <- DeviceStateLog/DeviceSensorLog POD mirror (kMaxJoints=16, kMaxFingertips=8)
│       ├── device_state_log_pod.hpp
│       ├── device_sensor_log_pod.hpp
│       ├── device_wbc_log_pod.hpp      <- WBC state superset: a_opt 가속도 + SE3 traj(arm)/fingertip force(hand), role-aware writer
│       ├── wbc_diag_log_pod.hpp        <- per-tick TSID/QP 진단 (solve time / λ / 수렴 / grasp), 단일 wbc_diag.csv
│       ├── pull_estimator_log_pod.hpp  <- in-plane pull-force estimate — pre-filter+filtered force + plane/basis + contact·touch·opposing mask + tick, 단일 pull_estimator.csv (3 데모 컨트롤러 공용)
│       ├── task_diag_log_pod.hpp       <- §6.5 특이점 진단 — sigma_min/lambda_sq + 적용 σ₀·λ_max + valid/ik_ok, 단일 task_diag.csv (DemoTask 전용)
│       ├── compliance_diag_log_pod.hpp <- §7 task-admittance 진단 — 소비된 wrench + source id/quality/invalid_reason + FSM/α + x̃·ν_c + K_p·K_d·Λ_d·§7.5 bound 스냅샷, 단일 compliance_diag.csv (DemoCompliance 전용)
│       ├── momentum_observer_log_pod.hpp <- 일반화 운동량 관측기 잔차 (Layer 1b) + payload 추정 (Layer 2A) — device order r + ‖r‖∞ + valid/invalid_reason/ticks_since_seed + payload wrench/mass/σ_min/λ²/fit_error/valid/reason, 단일 momentum_observer.csv (3 데모 컨트롤러 공용)
│       └── pod_fill.hpp                <- FillDeviceStateLogPod / FillDeviceSensorLogPod (device_idx + ForceFilterLogView 인자, robot-agnostic). WBC POD fill 은 controller-private (compute.cpp)
├── src/
│   ├── integrated_rt_controller_main.cpp     <- UR5e용 진입점
│   ├── controllers/
│   │   ├── controller_registration.cpp <- 데모 컨트롤러 등록 (RTC_REGISTER_CONTROLLER)
│   │   ├── joint/                      <- DemoJointController (controller/compute/lifecycle/parameters)
│   │   ├── task/                       <- DemoTaskController (controller/compute/lifecycle/parameters)
│   │   └── wbc/                        <- DemoWbcController (controller/compute/lifecycle/parameters/phase) + grasp_phase_manager.cpp + force_reference_updater.cpp
│   ├── backends/                       <- DeviceBackend 구현체 (RTC_REGISTER_DEVICE_BACKEND, --whole-archive 등록)
│   │   ├── mujoco_native_backend.cpp
│   │   ├── ur_driver_native_backend.cpp
│   │   └── udp_hand_native_backend.cpp
│   └── support/                        <- demo_shared_config / owned_topics 구현
├── config/
│   ├── ur5e_p1a/_base.yaml                 <- mode-agnostic SSoT (URDF + 모델 토폴로지 + device roster/limits + control_rate/logging)
│   ├── ur5e_p1a/robot.yaml                 <- 실제 로봇 delta (backend/토픽 + E-STOP + init 타이밍, _base 위에 overlay)
│   ├── ur5e_p1a/sim.yaml                   <- 시뮬레이션 delta (MuJoCo backend + sim-sync + 완화된 E-STOP, _base 위에 overlay)
│   └── controllers/
│       ├── demo_shared.yaml            <- DemoJoint/DemoTask 공통 파라미터 (vtcp/grasp/force_pi/pull_estimator/momentum_observer)
│       ├── demo_joint_controller.yaml  <- DemoJoint 게인/토픽
│       ├── demo_task_controller.yaml   <- DemoTask 게인/토픽
│       ├── demo_compliance_controller.yaml <- DemoCompliance 게인/토픽 (task 게인은 §7 철자 ik_kp_pos/ik_kp_rot/nullspace_kp, 값은 demo_task 와 동일 — 등가성 테스트가 고정. §7 admittance 는 K_p^a=0 hand-guiding 을 출하하고 (D-A3) bias 는 pull baseline 한 곳에서만 뺀다 (D-A5); 팔의 응답을 정하는 나머지 §7 키도 명시 등록한다 — 값은 코어 default 이되 `external_wrench.filter_enabled: false`, 파싱만 되고 이 바인딩이 안 읽는 키는 두지 않으며 세 프로필이 같다. 근거는 ur5e_p1b 사본)
│       ├── demo_inference_controller.yaml <- ONNX 정책의 관측 계약 (텐서 이름·element_names·fill·프레임·관절 규약·reach gate·`logs:` CSV lane·`closed_chain_warn_ticks` 진단). ur5e_p1b 전용, 모델은 `${RTC_POLICY_DIR}` (repo 밖)
│       ├── demo_wbc_controller.yaml    <- DemoWbc 게인/토픽/TSID/MPC
│       ├── demo_catching_controller.yaml <- DemoCatching 손 프로파일(L6 §5.1 `q_open`/`q_pre`/`q_close`/`caging_mask`/`eta_close`, 전부 provisional)·진단 플래그·CSV lane. `ur5e_p1b`·`iiwa7_leap` 만
│       └── mpc/                        <- DemoWbc handler-mode sub-configs
│           ├── phase_config.yaml       <- GraspPhaseManager 5-phase 설정
│           ├── contact_light.yaml      <- rtc_mpc ContactLightOCP factory config
│           └── contact_rich.yaml       <- rtc_mpc ContactRichOCP factory config
├── launch/
│   ├── robot_ur5e_p1a.launch.py                 <- 실제 UR5e(+assm_v1 hand) 로봇 launch (udp_hand_node 포함)
│   ├── robot_ur5e_p1b.launch.py        <- 실제 UR5e + proto_1b(closed-chain hand) 로봇 launch
│   ├── sim_ur5e_p1a.launch.py                   <- MuJoCo 시뮬레이션 launch (ur5e_p1a)
│   ├── sim_ur5e_p1b.launch.py          <- MuJoCo 시뮬레이션 launch (ur5e_p1b, closed-chain)
│   └── sim_iiwa7_leap.launch.py        <- MuJoCo 시뮬레이션 launch (iiwa7 + LEAP Hand)
├── integrated_bringup/                 <- ament_python 패키지 (GUI 모듈 · sim 도구)
│   ├── catching_sim_trials.py          <- 포구 sim 투척 드라이버 (투척마다 한 S7 순환, §Catching sim trials)
│   ├── sim_overlay.py                  <- `sim_overlay:=` 해석 (sim launch 공용)
│   └── demo_gui/
│       ├── app.py                      <- DemoControllerGUI Tk 클래스 + main()
│       ├── catalog.py                  <- /rtc_cm/list_controllers 동적 enumerator
│       ├── config.py                   <- gain 스키마, 위젯 레이아웃, 캘리브레이션 표
│       └── discovery.py                <- RobotShape (런타임 DOF/finger 추론)
└── scripts/
    ├── demo_controller_gui.py          <- 컨트롤러 튜닝 GUI 진입점 (얇은 shim)
    ├── catching_sim_trials.py          <- 포구 sim 투척 드라이버 진입점 (얇은 shim)
    └── motion_editor_gui.py            <- 모션 에디터 GUI (PyQt5)
```

> ◇ `mujoco_native_backend.hpp` 의 fingertip wrench lane 동작 (sim sensor B path):
> - 구독 입력: `devices.<group>.backend.fingertip_wrench_topics` (list 순서 = `inference_enable[f]` 인덱스)
> - 패킹: stride 7 `inference_data` 의 slot 1..3 (fx/fy/fz). slot 0 (contact_flag) 과 slot 4..6 (displacement) 은 zero-fill
> - 컨트롤러는 `devices.<group>.sensor_layout.has_native_contact=false` (sim 의 default) 로 인식하여 force-only grasp 분기 선택 (D-2 capability-aware)

---

## 기구학 모델 설정 (rtc_urdf_bridge)

데모 컨트롤러는 `rtc_urdf_bridge` 패키지를 통해 Pinocchio 기구학 모델을 구축합니다. URDF 경로와 모델 토폴로지(`sub_models`, `tree_models`, `passive_joints`)는 mode-agnostic 이므로 `ur5e_p1a/_base.yaml`의 최상위 `urdf:` 섹션에 단일 정의되며, `robot.yaml`(real HW)·`sim.yaml`(MuJoCo)은 그 위에 mode-specific delta 만 overlay 합니다 (launch 가 `[_base.yaml, <mode>.yaml]` 순으로 로드, 뒤 파일이 per-key override).

> **관절 분류 규칙**: `<transmission>` 태그는 active/passive 판정에 쓰이지 않습니다. URDF `type == "fixed"`는 kFixed, `<mimic>` 태그는 kPassive/kMimic, closed-chain 루프 참여 관절은 kPassive/kClosedChain, YAML `passive_joints:`에 명시된 관절은 kPassive/kFree, 그 외 non-fixed는 모두 기본적으로 kActive로 분류됩니다. Active이지만 `<limit effort>` 등 physics가 누락되면 경고가 출력됩니다. 상세는 `rtc_urdf_bridge/README.md` 참조.

### 설정 구조

| 설정 영역 | 위치 | 역할 |
|----------|------|------|
| `urdf:` (최상위) | `ur5e_p1a/_base.yaml` | URDF 경로, 모델 토폴로지 (sub_models/tree_models/passive_joints), 추가 frame (`extra_frames.<name>.{parent,xyz,rpy,provisional}` — ur5e_p1b·iiwa7_leap 의 `catch_frame`. `xyz` 는 S4.5 실측 포구점을 **부모 frame** 으로 옮긴 값이고 (`rpy` 가 π 회전인 손에서는 부호가 뒤집힌다), 2026-09-21 사용자 sim 확인 후 `provisional: false` 로 전환됐다 — dynamic_catching plan §10) |
| `devices:` (roster/limits) | `ur5e_p1a/_base.yaml` | 디바이스별 joint_names, joint_limits, sensor_names, sensor_layout 카운트 |
| `devices.<g>.backend` + `has_native_*` | `ur5e_p1a/{robot,sim}.yaml` | mode-specific: backend/wire 토픽, joint_command/motor 로스터, native-signal 플래그 |
| `demo_*_controller.yaml` | 컨트롤러별 YAML | 제어 게인, 토픽 라우팅 |
| `demo_shared.yaml` | 공통 YAML | DemoJoint/DemoTask 공통 파라미터 (Virtual TCP, grasp 감지, force_pi_grasp, pull_estimator, momentum_observer) |

> `sub_models`/`tree_models`의 `name`은 `devices` 블록의 디바이스 그룹 이름과 매칭됩니다 (예: sub_model `"ur5e"` = device `"ur5e"`).

### 데이터 흐름

```
ur5e_p1a/_base.yaml                            controller.yaml
┌──────────────────────────┐               ┌────────────────┐
│ urdf:                    │               │ kp, max_damping│
│   package + path ────────┼──(urdf)──→    │ trajectory_    │
│   sub_models:            │               │ speed, ...     │
│     - name: "ur5e"       │               └────────────────┘
│       root_link: "base"  │                       │
│       tip_link: "tool0"  │                       │
│   passive_joints: [...]  │                       │
│                          │                       │
│ devices:                 │                       │
│   ur5e:                  │                       │
│     joint_state_names    │                       │
│     joint_limits         │                       │
└──────────────────────────┘                       │
        │                                          │
  ament resolve + ParseSystemModelConfig() + Build shared PinocchioModelBuilder
        │                                          │
        ▼                                          ▼
  SetSystemModelConfig() + SetSharedModelBuilder() ──→ LoadConfig(): InitArmModel() ──→ OnDeviceConfigsSet()
  (ModelConfig + 공유 builder 전달)                 (공유 builder 우선, 폴백 시만 새로 빌드)   (tip/root frame 조회)
```

1. **SetSystemModelConfig() + SetSharedModelBuilder()**: `RtControllerNode`가 최상위 `urdf:` 섹션에서 파싱한 `ModelConfig` 와, 그 ModelConfig 로 한 번 빌드한 `std::shared_ptr<PinocchioModelBuilder>` 를 모든 컨트롤러에 전달합니다 (`LoadConfig()` 이전 호출).
2. **LoadConfig() / InitArmModel()**: `GetSharedModelBuilder()` 가 non-null 이면 그 핸들을 그대로 받아 `builder_` 로 사용 (URDF 재파싱 + 모델 재빌드 회피); null 이면 `GetSystemModelConfig()` 로 받은 ModelConfig 로 직접 `PinocchioModelBuilder` 를 만드는 폴백 경로. 결과적으로 sim launch 기준 URDF 파싱 4회 → 1회.
3. **OnDeviceConfigsSet()**: device YAML의 `root_link`/`tip_link`를 arm sub-model에서 프레임 인덱스로 조회하여 FK/Jacobian 연산 기준점으로 설정합니다.

### 시스템 URDF YAML 예시

```yaml
urdf:
  package: "robot_descriptions"
  path: "robots/ur5e_assm_v1/urdf/ur5e_with_hand.urdf"
  root_joint_type: "fixed"
  sub_models:
    ur5e:                     # map 키 = devices.ur5e 그룹명
      root_link: "base"
      tip_link: "tool0"
  # tree_models:              # hand FK 필요 시 활성화
  #   hand:                   # map 키 = devices.hand 그룹명
  #     root_link: "hand_base_link"
  #     tip_links: [thumb_tip_link, index_tip_link, middle_tip_link, ring_tip_link]
  passive_joints:             # arm sub-model에서 lock할 관절
    - "thumb_cmc_aa"
    - "thumb_cmc_fe"
    # ... (10개 hand joints)
```

### Closed-chain (Extended-URDF) bringup — `ur5e_p1b`

`ur5e_p1a`(mimic 기반)와 달리 `ur5e_p1b`는 **진짜 5-loop 폐쇄 체인**(proto_1b 손)입니다. URDF 는 순수 spanning-tree 이고 loop closure 는 sibling **`<stem>.closure.yaml`** sidecar 에 있습니다. `ur5e_p1b` 는 mode-agnostic 설정(`urdf:` 블록·device roster·limits·`control_rate`)을 **`config/ur5e_p1b/_base.yaml`** 에 두고, `robot.yaml`(real HW)·`sim.yaml`(MuJoCo)은 mode-specific delta(backend/토픽·E-STOP·타이밍)만 담아 launch 가 `[_base.yaml, <mode>.yaml]` 순으로 overlay 합니다(뒤 파일이 per-key override). `_base.yaml` 의 `urdf:` 가 폐쇄 체인을 켭니다:

```yaml
urdf:
  package: "hand_description"
  path: "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro"
  extended: true                                                  # loop closure 활성
  closure_path: "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml"
```

- **컨트롤러 FK**: `PinocchioModelBuilder`가 sidecar 로 5 constraint + 16 actuated(arm6+hand10) + loop-consistent q_ref 를 만들고, 각 컨트롤러(joint/task/wbc)는 `ClosedChainHandFk`(→ RT-safe `RtClosedChainHandle`, fixed-K 사영)로 loop-passive 하류 fingertip 을 loop-consistent 하게 FK 합니다. device `joint_state_names` 가 16 actuated 를 모두 커버하면 configure 시 `closed-chain hand FK active` 로그가 뜹니다(아니면 serial FK 로 graceful fallback).
- **디지털 트윈 시각화**: `config/ur5e_p1b/digital_twin.yaml` 의 `closure_path` 가 `digital_twin.launch.py`에서 `closure_state_publisher`(rtc_urdf_bridge)를 spawn 시켜 actuated 스트림에서 loop-passive 를 재구성, RViz 가 닫힌 손을 렌더합니다.
- **⚠️ residual floor (rank-deficiency)**: proto_1b 의 5 contact_3d 손가락 loop 은 **rank-deficient**(rank 11<15)이고 ring closure 좌표 불일치까지 겹쳐 DLS 사영의 도달 가능 ‖φ‖ floor(~3e-8 m)가 strict tolerance(1e-10) 위에 있습니다. `closure_state_publisher` 의 커밋 게이트는 `acceptable`(‖φ‖ ≤ acceptance tolerance, 기본 1e-6 m) 이라 floor 해도 정상 커밋되어 트윈이 추종합니다. 필요시 `closure_tolerance`(strict) / `closure_acceptance_tolerance` 를 config 로 오버라이드할 수 있습니다. **제어 경로(`RtClosedChainHandle`, fixed-K)는 converged 게이트가 없어 무영향**입니다.

실행:

```bash
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py enable_viewer:=false   # headless
```

- **⚠️ 기본 시작 컨트롤러**: `ur5e_p1b` 는 `robot.yaml`·`sim.yaml` 모두 `initial_controller: "demo_joint_controller"` 로 시작합니다 (`ur5e_p1a`·`iiwa7_leap` 는 `demo_wbc_controller` 기본). WBC 로 시작하려면 launch 오버라이드로 지정하거나 `demo_controller_gui` 의 컨트롤러 라디오에서 전환합니다:

  ```bash
  ros2 launch integrated_bringup sim_ur5e_p1b.launch.py initial_controller:=demo_wbc_controller
  ```

#### Scene overlay (`sim_overlay:=`) — 출하 씬을 건드리지 않고 바꾸기

`sim_ur5e_p1b.launch.py` 는 `config/ur5e_p1b/mujoco_simulator.yaml` **뒤**, 그리고 자기 명령줄
인자 (`model_path:=` 등) **앞**에 params 파일 하나를 더 끼울 수 있습니다. 두 노드에 모두 전달되고
각 노드는 자기 섹션만 읽습니다. 이름만 주면 `config/ur5e_p1b/sim_overlays/<name>.yaml`, `/` 를
포함하거나 `.yaml` 로 끝나면 경로로 해석하며, **해석되지 않는 값은 launch 를 실패**시킵니다
(에러에 출하 overlay 이름 목록이 실립니다). fallback 하지 않는 이유는 launch 파일의
`resolve_sim_overlay` docstring 이 갖습니다.

출하 overlay 와 각각의 용도는 `config/ur5e_p1b/sim_overlays/*.yaml` 의 헤더 주석이 소유합니다.
어느 overlay 도 공유 `mujoco_simulator.yaml` 을 건드리지 않으므로, `sim_overlay:=` 없이 띄우면
정확히 출하 씬입니다. `demo_inference_controller` 의 정책이 학습된 씬(바닥 위 원통 1개 + 학습
reset 자세)은 `inference_pole` 이고, 물체 자산은 `robot_descriptions/objects/pole` 에 있습니다.

```bash
export RTC_POLICY_DIR=/path/to/ObjectHandGraspDeployMulti5-Export-v0   # 모델은 repo 밖
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py \
    sim_overlay:=inference_pole use_cpu_affinity:=false

# 기동 후 (기본 활성은 demo_joint_controller):
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['demo_inference_controller'], \
    deactivate_controllers: ['demo_joint_controller'], strictness: 1, timeout: {sec: 1}}"
```

기동 로그의 `[inference] N observed link(s) in '<frame>'` 이 정책이 어느 프레임으로 관측하는지를
말해주고, `[inference] policy loaded: ...` 가 없으면 `RTC_POLICY_DIR` 이 안 잡힌 것입니다 (그때는
`allow_missing_model: true` 라 컨트롤러가 자세만 유지합니다). 결과 판독은
`<session>/controllers/demo_inference_controller/inference_diag.csv` — 위 로그 lane 표 참조.

---

## 공통 파라미터 (`demo_shared.yaml`)

`DemoJointController` · `DemoTaskController` · `DemoComplianceController` · `DemoWbcController` 가 다음 파라미터를 공유합니다. 기본값은 로봇별 `config/<robot>/controllers/demo_shared.yaml` 에 단일 소스로 정의되며, 네 컨트롤러가 `LoadConfig()` 시점에 동일하게 로드합니다.

| 파라미터 | 설명 |
|---------|------|
| `virtual_tcp_mode` / `virtual_tcp_offset` / `virtual_tcp_orientation` | Virtual TCP 설정 (아래 DemoTask 섹션 참조) |
| `grasp_contact_threshold` / `grasp_force_threshold` / `grasp_min_fingertips` | Grasp 감지 임계값 (capability-aware — 아래 표 참조) |
| `grasp_controller_type` | `"contact_stop"` · `"force_pi"` · `"none"` (hand 무개입, GraspState 발행은 유지). 화이트리스트 검증 — YAML 값이 그 외면 `on_configure` FAILURE, **파라미터로 들어온 값이면 ERROR 로그 + YAML 값 유지** (선언 경로는 noexcept 라 전이를 실패시킬 수 없다). 파라미터 경로는 capability 게이트도 거친다 |
| `force_pi_grasp.*` | Force-PI grasp 파라미터 + 핑거별 q_open/q_close. `fingers.finger_names` 는 슬롯 순서와 함께 **transition 의 thumb 슬롯**을 정한다 — Approaching → Contact 는 thumb + `f_contact_threshold` 를 넘은 **아무 손가락 하나**를 요구하며, 목록에 `"thumb"` 이 없으면 슬롯 0 + WARN 으로 낙하한다. 재정렬로 판정 쌍을 바꿀 수는 없다 (자세·센서 레인·`hand_finger_joint_map` 이 3중으로 어긋난다 — [grasp_tuning_guide.md](../rtc_controllers/docs/grasp_tuning_guide.md) §1.2) |
| `pull_estimator.*` | In-plane pull-force estimator — 필터/유효성 파라미터, plane normal source (`fixed` · `pinch_geometry` 화이트리스트), baseline subtraction (grasp 확립 rising edge 에 arm), `required_roles`, per-tip contact model (`tips.<role>.link/friction_coeff/…`). 키 이름은 `rtc::grasp::{PullEstimatorParams, PullContactConfig}` 필드와 1:1. **힘 부호 계약**: estimator 는 **finger-on-object** 입력을 합산하며 `force_sign` 기본값은 `+1` (fingertip 센서 발행자의 부호와 일치) — 부호가 뒤집히면 파지 중에도 all-zero estimate 가 발행되므로 어떤 프로필도 per-tip 반전을 두지 않는다. per-contact gate/diagnostic normal 은 YAML 이 아니라 매 tick pinch geometry 에서 유도된다 (±plane normal, thumb 반대부호) — 대향 집합 선택이 `contact_active` 가 아니라 `touch_active` 인 근거는 [pull_estimator_wiring.hpp](include/integrated_bringup/support/pull_estimator_wiring.hpp) 헤더 주석. 따라서 `tips.tip_names` 는 *물리적으로 참여 가능한 tip 전체*를 나열한다 (WBC 는 같은 link 를 `tsid.contacts` 에도 resolve 해야 하므로 tsid contact 이 없는 tip 은 추가 불가 — `ur5e_p1a`/`ur5e_p1b` 의 ring 이 이 이유로 제외, `iiwa7_leap` 은 ring 이 tsid contact 이라 포함). role 이름은 고유해야 하며 중복 시 `on_configure` FAILURE. baseline 은 grasp rising edge 에서 즉시 arm 되고, 투영 전 기준 wrench 로 저장돼 매 tick 현재 평면에 재투영되므로 파지 형태가 바뀌어도 따라간다. `tips.<role>.link` 는 joint/task 에선 tree-model `tip_links`, wbc 에선 `tsid.contacts` frame 과 매칭해 FK slot 을 resolve (`support/pull_estimator_wiring.hpp`; 미매칭 link 는 `on_configure` FAILURE); wbc 는 TSID λ_opt 이 아닌 **measured** R_i·f_i (contact-frame geometry) 사용. 출력은 owned POD (`GraspStateData.pull` / `WbcStateData.pull`) 로 SeqLock 저장 후 `GraspState.msg`/`WbcState.msg` 의 `pull` (`PullEstimate`) 서브메시지로 publish. 구성: `ur5e_p1a`·`ur5e_p1b` (실기 force-only inference lane) + `iiwa7_leap` (**sim 전용** — LEAP 실기 force 센서 없음, MuJoCo contact wrench; 실기 모드는 force 무효 → estimate invalid+decay). iiwa7_leap `sim.yaml` 의 `fingertip_wrench_topics` 는 sensor-lane 을 tip_links 와 정합(thumb→index→middle→ring)시키기 위해 재정렬됨 (Stage A-3 contract). **선택적 touch hysteresis** (P-16): `tips.<role>.touch_on_threshold`/`touch_off_threshold` 를 둘 다 주면 축 선택 게이트(|f_obj|)가 힘-합 게이트(f_n)와 분리된다 — 둘 다 생략(현 config 전부)하면 `touch ⊇ contact` 계약이 유지되고, 한쪽만 주면 `on_configure` FAILURE. **configure-time hard-fail**: `min_valid_contacts <= tip 수`, 모든 scalar finite, `force_sign ∈ {+1,-1}`, `plane_normal_source: fixed` 일 때 `plane_normal` finite·non-zero (P-6). `plane_normal ∥ inplane_x_reference` (fixed 소스) 는 축이 붕괴하므로 WARN + `basis_source=SEED` (P-17). `friction_coeff` 는 **가정치**이므로 `friction_utilization` 은 절대 마진이 아니라 상대 추세 지표다 (P-18). 추정은 준정적 — 물체의 in-plane `m·a` 가 외력으로 보고되며 가속 validity gate 는 아직 없다 (P-19). |
| `momentum_observer.*` | 일반화 운동량 관측기 (Layer 1b) — `enabled` (블록 부재 = disabled) + `gains` (K_I [1/s]; 스칼라면 전 관절 broadcast, 리스트면 정확히 arm dof — 그 외 길이는 configure FAILURE). **arm 디바이스(device 0) 위에서만** 돌며 세 데모 컨트롤러가 각각 하나씩 든다. 산출은 관절공간 잔차 `r = LPF(τ_ext)` 로, 손목 F/T 센서 없이 팔의 proprioception (encoder + joint torque) 만 쓴다. 관측기는 컨트롤러의 `arm_handle_` 이 아니라 **자기 소유 `RtModelHandle`** 위에서 도는데, 그 이유(좌표 계약·pinocchio `Data` 경합 회피)와 arm sub-model(open chain) 을 쓰는 이유(Coriolis 축약 경로 부재)는 [momentum_observer_wiring.hpp](include/integrated_bringup/support/momentum_observer_wiring.hpp) 헤더 주석 참조. 비용은 tick 당 arm-only 동역학 1회, 7-DoF 실측 2.13–2.17 µs (500 Hz 기준 0.11 %). **입력 lane 게이트**: `IsLaneReadable` 로 position/velocity/effort 세 lane 을 매 tick 판정하고, 하나라도 닫히면 `Hold()` 로 잔차를 **동결**한다 (0 으로 덮지 않는다 — 0 은 '외력 없음' 이라는 단언이고, 그건 판독 불가 tick 이 지지하지 않는 측정이다). E-STOP tick 도 같은 경로. **`efforts` lane 계약**: 관절 토크 [N·m] 이고 관절에 걸리는 일반화력을 **전부** 담아야 한다 — 전류를 재는 backend 는 경계에서 변환하고 raw 는 `motor_efforts` 에 남긴다 (D10). 두 조건 모두 소비자 쪽에서 검사할 수단이 없다 (fresh·full·틀린 lane 은 정상으로 보인다). 출력 채널은 둘이다 — `momentum_observer.csv` 와 `rtc_msgs/PayloadEstimate` 토픽 (D12), 둘 다 tick 당 **한 번 만들어진 행**을 공유하므로 (`FillMomentumObserverLogPod`) 저장 파일과 라이브 토픽이 같은 tick 에 다른 수치를 낼 수 없다 (배경: [momentum_observer_log_pod.hpp](include/integrated_bringup/logging/momentum_observer_log_pod.hpp) 헤더 주석) |
| `momentum_observer.payload_estimator.*` | 준정적 payload 추정 (Layer 2A) — 위 잔차를 Cartesian wrench 와 질량으로 역산한다. **서브블록 부재 = disabled** 이고 출하 config 는 `enabled: false` 다 (아래 근거). 키는 `frame` (payload 가 매달린 프레임, **arm sub-model** 기준. 모델에 없는 이름은 configure 에서 **거부** — pinocchio 는 모르는 이름에 frame 0(universe) 을 돌려주므로 그대로 두면 전 구간 all-zero Jacobian 에 투영하며 '영원히 무부하' 로 보인다), 게이트 3축 `max_arm_velocity`·`max_peripheral_velocity`·`settle_time_constants`, 수치 가드 `sigma0`·`lambda_max`·`min_sigma`·`max_fit_error`·`min_gravity`. **지배축은 손**이다 — 관측기는 손 관절을 reference 자세에 **잠근** 팔 sub-model 위에서 돌므로 (D3) 실제 손 운동은 정의상 외력이고 payload 와 구별 불가하다 (D14); 팔 축은 훨씬 낮은 속도까지 평평하므로 게이트 임계는 손 쪽이 지배한다 (sim 실측 기준 대략 10 g 오더 분해능이나, **sim 수치이므로 실기는 재튜닝 필수** — 실제 엔코더 속도 잡음이 더 크다). damping 은 `μ` 를 상수로 바닥깔지 않고 **adaptive 로 유지**한다 — 바닥을 깔면 모든 자세에서 `m̂` 이 0 쪽으로 편향되어 payload 를 **항상 과소보고**한다. `payload_sigma_min`/`payload_lambda_sq` 를 CSV 에 내보내므로 그 선택이 관측 가능하다. **출하 기본이 off 인 이유**: sim 씬에 물체가 없고 `MuJoCoSimulator::SetExternalForce` 에 ROS 노출이 없어 (D16) **알려진 질량을 매달 수단이 없다** — 대수는 단위 테스트로 고정됐지만 `m̂` 의 정확도는 positive control 로 검증된 적이 없다. tick 당 frame Jacobian 1회 + 6×6 solve 비용도 든다 |

**Grasp threshold capability matrix.** 컨트롤러는 `devices.<hand>.sensor_layout.has_native_contact` (robot/sim yaml) 를 보고 두 경로로 분기합니다.

| 키 | 활성 sensor type | 의미 |
|---|---|---|
| `grasp_contact_threshold` | sensor A only (udp_hand + ft_inferencer.enabled) | native sigmoid contact probability threshold (0..1). sensor B 에선 미사용 |
| `grasp_force_threshold` | sensor A + B 공통 | `\|F\| > threshold` [N]. sensor A 에선 contact_threshold 와 AND 결합 |
| `grasp_min_fingertips` | sensor A + B 공통 | `grasp_detected = active_count ≥ N` |

**Override 우선순위:** `demo_shared.yaml` (defaults) → `demo_*_controller.yaml` (per-controller overrides). 컨트롤러별 YAML에 동일 키를 추가하면 해당 컨트롤러에서만 덮어씁니다. 예:

```yaml
# demo_task_controller.yaml에서만 다른 grasp 임계값을 쓰고 싶을 때
demo_task_controller:
  grasp_force_threshold: 2.0   # demo_shared.yaml의 1.0을 override
  ...
```

파싱 로직은 `include/integrated_bringup/support/demo_shared_config.hpp` / `src/support/demo_shared_config.cpp`의 `ApplyDemoSharedConfig()` / `LoadDemoSharedYamlFile()` / `BuildGraspController()` / `BuildPullForceEstimator()`로 통합되어 있습니다.

---

## 데모 컨트롤러

### 토픽 소유권 (Phase 4 + trailing cleanup)

데모 컨트롤러는 non-RT 외부 통신 토픽을 **컨트롤러가 소유**합니다.

| 토픽 역할 | 소유자 | 메커니즘 / 경로 (active = demo_wbc_controller 예시) |
|-----------|--------|------------------------------------------|
| `target` (per-group joint_goal) | 컨트롤러 | YAML `topics:` `subscribe:` `role: target` — `/demo_wbc_controller/<group>/joint_goal` (구독 lane) |
| `grasp_state` (joint/task 데모만) | 컨트롤러 | **SeqLock 직접 소유** — controller 가 `SeqLock<GraspStateData>` + `SetupGraspStatePublisher()` 헬퍼 (`support/owned_topics.hpp`) 로 발행. `PublishRole` 미사용 |
| `wbc_state` (wbc 데모만) | 컨트롤러 | **SeqLock 직접 소유** — `SeqLock<WbcStateData>` + `SetupWbcStatePublisher()` |
| `payload_estimate` (3 데모 컨트롤러 공용) | 컨트롤러 | **SeqLock 직접 소유** — `SeqLock<MomentumObserverLogPod>` + `SetupPayloadEstimatePublisher()`. `PublishRole` 미사용 (D12). 발행 조건은 **관측기** 활성이지 payload 추정기 활성이 아니다 — 잔차 절반만으로도 메시지가 성립하고, 출하 config 는 `payload_estimator.enabled: false` 라 추정기 기준으로 걸면 잔차까지 wire 에서 사라진다. `joint_names` 를 함께 실어 잔차의 **device order** 를 메시지가 자기기술한다 (pinocchio order 와 짝지으면 유한하고 매끄럽고 틀린 답이 나온다) |
| `tof_snapshot` (joint/task 데모만) | 컨트롤러 | **SeqLock 직접 소유** — `SeqLock<ToFSnapshotData>` + `SetupToFSnapshotPublisher()` |
| `robot_transforms` (모든 데모 컨트롤러) | 컨트롤러 | YAML `topics:` `role: robot_transforms` (`kRobotTransforms`) — `<config_key>/transforms` (`tf2_msgs/TFMessage`, RELIABLE/10). **DemoJoint / DemoTask**: `base→tool0_actual` + `base→{thumb,index,middle,ring}_tip_link_actual` + `base→virtual_tcp_actual` = 6 frame. **DemoWbc**: `base→tool0_actual` + `base→{thumb,index,middle,ring}_tip_link_actual` (loop-consistent fingertip FK, publish 표면 — DIP 구동에 반응) + `base→wbc_alpha_actual` placeholder (D-5, valid=false). 단일 publisher per controller (D-2/D-10) — YAML entry는 첫 group의 `publish:` 에 두고 owned_topics가 system YAML `urdf.{sub,tree}_models` 로 frame slot 자동 빌드. Active controller만 LifecyclePublisher 활성 |
| `state` (per-group `joint_states`) | CM | `/rtc_cm/<group>/joint_states` (`JointState`) |
| device-wire command lane (`joint_command`, `ros2_command`) | **DeviceBackend** | `devices.<group>.backend:` SSoT — backend 구현체가 직접 publish/serialize. CM/controller YAML 에서 device-wire role 라인 사라짐 |
| `device_state_log` / `device_sensor_log` (CSV) | 컨트롤러 (`ControllerLogSet`) | `<session>/controllers/<config_key>/<device>_{state,sensor}.csv` — instance 키는 `GetPrimaryDeviceName()`/`GetSecondaryDeviceName()` 에서 동적 파생 (ur5e_p1a → `ur5e_state.csv`/`p1a_state.csv`/`p1a_sensor.csv`, iiwa7_leap → `iiwa7_state.csv`/`leap_state.csv`). **폭 계약**: 헤더와 행은 둘 다 등록 시 한 번 산출된 *config* 폭(`joint_state_names`/`motor_state_names`/`sensor_names`, POD capacity 로 clamp)을 쓴다 — 런타임 `num_channels`/`num_fingertips` 가 그보다 적으면 행을 줄이는 게 아니라 0 으로 채우고, 장치가 실제로 보고한 수는 마지막 `num_joints`/`num_motors`/`num_fingertips` 컬럼이 싣는다 |
| `device_wbc_log` / `wbc_diag_log` (CSV, WBC 전용) | `DemoWbcController` (`ControllerLogSet`) | WBC 는 arm/hand `<device>_state.csv` 를 generic DeviceStateLog 대신 superset `DeviceWbcLog` (`msg_type: integrated_bringup/DeviceWbcLog`) 로 쓴다 — TSID `a_opt` 가속도 + SE3 trajectory(arm role)/fingertip force(hand role — |F| `fingertip_force_*` + 벡터 `fingertip_fx_/fy_/fz_*`). 추가로 per-tick `wbc_diag.csv` (`msg_type: integrated_bringup/WbcDiagLog`) 에 solve time / λ / 수렴 / grasp 진단을 1행씩. Path A — POD-only, rtc_msgs `.msg` 무변경. fill 은 controller-private (`compute.cpp`); E-STOP 경로는 push 안 함. 폭 계약은 위 행과 같고 (joint/motor 축), fingertip 블록은 `kMaxFingertips` 고정폭이다 — 그 축에는 config 폭 자체가 없기 때문 (LEAP 은 tactile 0개인데 contact-force fingertip 을 보고한다). `wbc_diag.csv` 의 λ 블록도 config QP dim 폭이고 마지막 `num_contact_vars` 컬럼이 런타임 수를 싣는다 |
| `task_diag_log` (CSV, DemoTask 전용) | `DemoTaskController` (`ControllerLogSet`) | per-tick `task_diag.csv` (`msg_type: integrated_bringup/TaskDiagLog`, instance 고정 `task_diag`) — §6.5 특이점 진단. 기록하는 `sigma0`/`lambda_max` 는 그 tick 에 **실제 적용된** floor 적용 후 값 — gain 은 `set_gains()`/파라미터 콜백으로 live 변경되므로 저장된 CSV 를 그 run 의 YAML 없이 해독하려면 필요하다. `damping_active` 는 `valid && lambda_sq > 0` (= `sigma_min < sigma0`) 파생 컬럼. E-STOP·arm 미판독·reorder 무효 tick 은 `ComputeControl` 이 early-return 하므로 σ 필드가 직전 tick 값이며, `pull_estimator.csv` 와 같은 규약으로 gap 이 아니라 **`valid=0` 행**을 남긴다 (agent_docs/controllers.md 매 tick Store 계약). 특이점 진입은 CSV 와 별개로 throttled WARN 으로도 보고된다 (RT-3 throttle 예외). `rtc_tools` 플로터는 아직 이 파일을 자동 감지하지 않는다 |
| `pull_estimator_log` (CSV, 3 데모 컨트롤러 공용) | 컨트롤러 (`ControllerLogSet`) | per-tick `pull_estimator.csv` (`msg_type: integrated_bringup/PullEstimatorLog`, instance 고정 `pull_estimator`) — in-plane pull-force estimate. `PullEstimate` (Eigen) 에서 직접 fill 하므로 wire msg 에 없는 **`force_prefilter_*`** (투영·중력·baseline 차감 후, 필터 직전 값 — 원 센서 합이 아님) 와 **`tick`** (CM RT loop iteration — 행 하나가 tick 하나이므로 여기 gap = SPSC ring drop) 을 추가로 기록. wire 와 공유하는 자기기술 컬럼: `plane_normal_*` / `basis_x_*` / `basis_source` / `invalid_reason` / `contact_mask` / `touch_mask` (P-5·P-11·P-12). 세 mask 컬럼명에는 비트 순서가 박힌다 — `contact_mask[thumb|index|middle]` 처럼 configure 시점의 tip role 순서를 헤더에 스탬프하므로 저장된 CSV 를 그 run 의 ROS 로그 없이 해독할 수 있다 (P-14). `rtc_tools` 로더가 이 접미사를 벗겨 `df.attrs["mask_roles"]` 로 노출하므로 플로터는 bare 컬럼명을 쓴다. `opposing_mask` 는 thumb 비트를 세우지 않는 (thumb 대향 집합) 반면 `contact_mask` 는 힘 합에 들어간 집합, `touch_mask` 는 thumb 포함 접촉 집합이다. 등록은 `pull_estimator` YAML 블록이 활성일 때만 (wiring disabled → YAML entry silently skip, 이 경우 `on_configure` INFO 가 CSV 미생성을 명시) |
| `grasp_diag_log` (CSV, DemoJoint · DemoTask) | 컨트롤러 (`ControllerLogSet`) | per-tick `grasp_diag.csv` (`msg_type: integrated_bringup/GraspDiagLog`, instance 고정 `grasp_diag`) — Force-PI 서보 + 온라인 강성 추정기 진단. **`f_measured_*` 는 25 Hz Force-PI 뱅크** (`fingertip_force_mag_filt_grasp_`) 로, `<device>_sensor.csv` 의 `force_filtered_*` (contact_stop 뱅크, 배포 50 Hz) 와 **다른 레인**이다 — 둘 다 이름이 force 라서 틀린 쪽에서 잰 σ 는 아무 증상 없이 그럴듯한 값을 준다. `k_inst_raw_*` 는 `K_est_max` **clamp 전** 순간 추정치이고 `delta_s_*`/`delta_f_*` 는 추정기가 실제로 나눈 차분 쌍이다 — 이 셋은 CSV 의 s/f 컬럼에서 복원할 수 없다 (추정기는 두 tick 전 차분을 쓰는데 로그 행은 현재·직전 값을 실어 한 tick 어긋난다). `beta`/`alpha_ema`/`K_est_max` 를 매 tick 함께 남기므로 저장된 CSV 가 그 run 의 YAML 없이 해독된다 (`task_diag` 의 `sigma0`/`lambda_max` 와 같은 사유). 등록 게이트는 **`force_pi_grasp` 블록 존재**이지 `grasp_hand_mode` 가 아니다 — 모드는 런타임 변경 가능하므로 모드로 게이트하면 contact_stop 으로 시작해 force_pi 로 바꾼 세션이 조용히 파일을 안 남긴다. PI 법칙이 안 돈 tick 은 gap 이 아니라 **`valid=0` 행**이며 per-finger 필드는 동결이 아니라 **0** (PROC-7). `tick` 컬럼이 `pull_estimator.csv`·`<device>_sensor.csv` 와 정렬되므로 같은 run 에서 25 Hz/50 Hz 레인 교차 검증이 된다. `rtc_tools` 플로터가 자동 감지하며 `--stats` 가 Holding 구간 σ 를 숫자로 출력한다. 실기 절차는 [grasp_tuning_guide.md](../rtc_controllers/docs/grasp_tuning_guide.md) §8 |
| `momentum_observer_log` (CSV, 3 데모 컨트롤러 공용) | 컨트롤러 (`ControllerLogSet`) | per-tick `momentum_observer.csv` (`msg_type: integrated_bringup/MomentumObserverLog`, instance 고정 `momentum_observer`) — 일반화 운동량 관측기 잔차 (Layer 1b) + Layer 2A payload/Layer 2B inertial 추정. 컬럼 그룹: `t_relative_s,tick`, `r_<joint>`/`residual_inf_norm`/`valid`/`invalid_reason`/`ticks_since_seed` (잔차 — arm 디바이스 joint order), `payload_*` (Layer 2A wrench/mass/sigma_min/lambda_sq/fit_error/valid/reason), `inertial_*` (Layer 2B m̂·ĉ/ĉ/sigma_min/fit_error/rank/valid/reason) — 컬럼별 정확한 의미·단위·enum 값은 [momentum_observer_log_pod.hpp](include/integrated_bringup/logging/momentum_observer_log_pod.hpp) 필드 주석이 SSoT. 행은 **매 tick** 남으며 held·E-STOP tick 은 gap 이 아니라 `valid=0` 행 + 직전 잔차 **동결** 값이다 (`pull_estimator.csv`·`grasp_diag.csv` 와 같은 규약, PROC-7) — `tick` 의 gap 은 SPSC ring drop 만을 뜻하고, 파일 자체의 부재는 `momentum_observer` 블록이 없거나 disabled 라는 뜻이다. 같은 행이 `payload_estimate` 토픽 (`rtc_msgs/PayloadEstimate`) 도 채운다 — POD 는 tick 당 한 번 만들어져 두 lane 에 전달된다. `rtc_tools` 플로터가 자동 감지한다 — `ros2 run rtc_tools plot_rtc_log <session>/controllers/<key>/momentum_observer.csv` 가 `momentum_observer.png` 를 내고, Layer 2A/2B 가 실제로 구성된 run 에서만 `momentum_payload.png` 도 낸다. `--stats` 의 ‖r‖∞ 통계는 `valid=1` 행만 쓴다 |
| `compliance_diag_log` (CSV, DemoCompliance 전용) | `DemoComplianceController` (`ControllerLogSet`) | per-tick `compliance_diag.csv` (`msg_type: integrated_bringup/ComplianceDiagLog`, instance 고정 `compliance_diag`) — §7 task-admittance 진단. 컬럼은 소비된 wrench (`wrench_f*`/`wrench_t*`, post-conditioning LWA), 소스 축 (`wrench_source`·`quality_low`·`invalid_reason`), 파이프라인 상태 (`wrench_age`/`fade`/`stale`/`in_contact`/`bias_*`/`rejected_samples`), 법칙 상태 (`fsm_state`·`alpha`·`x_tilde_*`·`nu_c_*`·`task_origin_*`·`disp_limited`/`vel_limited`/`adm_finite`), 그리고 매 tick 파라미터 스냅샷 (`kp_*`/`kd_*`/`md_*`/`max_disp_*`). **모든 값은 사용 지점(`ComputeControl`)에서 staging 된다** — 렌치는 `ComputeSecondary` 에서 발행되고 다음 tick 의 `ComputeControl` 에서 소비되므로 (D-A14), push tail 에서 멤버를 읽으면 한 행에 *이번* tick 의 `invalid_reason` 과 *직전* tick 의 wrench 가 섞인다. 파라미터를 매 tick 싣는 이유는 `task_diag` 의 `sigma0`/`lambda_max` 와 같다 — **평평한 x̃ 가 "수렴" 인지 "§7.5 박스에 pin" 인지**를 그 run 의 YAML 없이 가르려면 bound 가 같은 줄에 있어야 하고, `disp_limited` 가 그 판정을 직접 말한다. 등록 게이트는 `external_wrench.source` 블록 존재이며 (D-A12: 기본값 없는 필수 키라 configure 한 컨트롤러는 반드시 하나를 갖는다), 법칙이 안 돈 tick (E-STOP · arm 미판독 · reorder 무효) 은 gap 이 아니라 **`valid=0` 행 + 값 0** 이다 (동결 아님 — 느리게 변하는 추정치는 동결되면 살아있는 것처럼 읽힌다, `grasp_diag` per-finger 와 같은 규약). 배선 안 된 fault 3개 (`command_divergence`·`saturation_persist`·`posture_authority_lost`) 는 **컬럼을 만들지 않는다** — 영원히 0 인 컬럼은 "안 일어남" 으로 읽힌다. Path A — POD-only, rtc_msgs `.msg` 무변경. `rtc_tools` 플로터가 `x_tilde_` 컬럼 지문으로 자동 감지하며, `--stats` 없이도 envelope (`|x_tilde|` peak 대 bound 비율) · freshness · bias 재무장 횟수를 숫자로 출력한다 (그림은 없다 — `grasp_diag` 와 같은 판단: 숫자가 산출물) |
| `inference_diag_log` (CSV, DemoInference 전용) | `DemoInferenceController` (`ControllerLogSet`) | per-tick `inference_diag.csv` (`msg_type: integrated_bringup/InferenceDiagLog`, instance 고정 `inference_diag`) — 이 컨트롤러는 **모든 실패가 hold 로 수렴**하므로 (입력 unreadable · `Run()` false · 비유한 출력 · closed-chain held · 물체 stale) 바깥에서 보면 정상 동작과 구분되지 않는다. 그래서 매 tick `held` 와 **사유 코드** (`hold_code`/`hold_reason` — `InferenceHoldReason`, 조기 반환 지점마다 1개) 를 남기고, 그 옆에 정책이 실제로 본 것을 함께 둔다: `policy_step`·`inference_count` (주기가 decimation 대로인지), `reach_phase`·`tip_distance`·`reach_hold` (게이트), `object_valid`·`object_age_s`·`object_x..z` (**policy_frame 기준** — 프레임이 어긋나면 여기서 부호로 드러난다), `closed_held`·`closed_held_ticks`·`closed_singular`·`closure_error` (폐쇄 체인 사영), `arm_lag_max` (정책 목표와 측정 관절각의 최대 차 — sim 서보의 구조적 지연이 여기 실린다), `force_<tip>` (reach gate 가 읽는 손끝별 힘). 같은 `logs:` 블록이 arm/hand `<device>_state.csv` 도 연다 (형제 컨트롤러와 같은 스키마). 파일 부재 = `logs:` 에 그 채널이 없다는 뜻 |

**외부 도구는 `/active_controller_name` (TRANSIENT_LOCAL) 구독해서 런타임에 rewire**하십시오 (BT bridge / GUI / digital_twin / shape_estimation 포함). 컨트롤러 전환 시 각 소유 토픽은 이전 네임스페이스에서 silent 되고 새 네임스페이스에서 라이브됩니다.

### DemoJointController (Index 4)

관절 공간 Quintic 궤적 생성기 -- UR5e 6-DOF 로봇 암 + 10-DOF 핸드 통합 제어기입니다. Rest-to-rest quintic 다항식으로 부드러운 궤적을 생성하며, 출력을 직접 위치 명령으로 전달합니다 (비례 게인 없음).

**타겟 메시지 레이아웃** (`/target_joint_positions`, `Float64MultiArray`):
- `data[0..5]`: 로봇 암 관절 타겟 (rad)
- `data[6..15]`: 핸드 모터 타겟 (rad), 선택 -- size < 16이면 무시

**궤적 계산:**

```
duration = max(0.01, T_speed, T_velocity)
  T_speed    = max_joint_dist / trajectory_speed
  T_velocity = (15/8) * max_joint_dist / max_traj_velocity
position_output = quintic(t, q_start, q_goal, duration)
```

**파라미터** (YAML 기본값):

| 파라미터 | YAML 기본값 | 설명 |
|---------|------------|------|
| `robot_trajectory_speed` | `0.5` rad/s | 로봇 궤적 속도 (duration 계산용) |
| `hand_trajectory_speed` | `0.5` rad/s | 핸드 궤적 속도 |
| `robot_max_traj_velocity` | `3.14` rad/s | 로봇 최대 관절 속도 (peak velocity 제한) |
| `hand_max_traj_velocity` | `2.0` rad/s | 핸드 최대 관절 속도 |
| `command_type` | `"position"` | 출력 타입 (`"position"` 또는 `"torque"`) |

**게인 업데이트 레이아웃 (4개 요소):**
`[robot_trajectory_speed, hand_trajectory_speed, robot_max_traj_velocity, hand_max_traj_velocity]`

**ContactStopHand:** 핑거팁 센서에서 힘 감지 시 핸드 궤적 출력을 현재 위치로 동결하여 과도한 hand closure를 방지합니다. 동결 위치는 측정 위치를 Bessel LPF (`fsm.contact_stop_lpf_cutoff_hz`, 기본 20 Hz) 로 통과시킨 값이라 인코더 노이즈가 desired 로 직접 실리지 않습니다.

**힘 입력 LPF (joint / task):** 동결 여부를 판정하는 `|F|` 는 핑거팁별 `fx/fy/fz` 를 축별로 Bessel LPF (`fsm.contact_stop_force_lpf_cutoff_hz`, 기본 50 Hz) 에 통과시킨 뒤 계산합니다 — 단일 노이즈 샘플이 freeze 를 latch 하지 못하게 하기 위함입니다. 세 가지가 의도된 설계입니다.

- **축별 필터 (크기 필터 아님)**: `|F|` 를 먼저 만들고 필터링하면 평균 0 인 축 노이즈가 정류되어 양의 바이어스로 남아 접촉이 없어도 임계에 접근합니다.
- **cutoff 가 위치 LPF 보다 높음**: 4차 Bessel 의 DC 군지연은 `~2.11/(2π·fc)` 이므로 20 Hz 면 ~17 ms — 이 latch 가 보호하려는 BT tick(50 ms) 의 1/3 입니다. 50 Hz 는 ~6.7 ms.
- **grasp_controller_type 과 무관하게 항상 실행**: 로그 컬럼의 의미가 런 설정에 따라 바뀌지 않도록. `force_pi` 의 `force_filter_` 는 손가락별 **`|F|` 스칼라** 에 걸리는 별개 필터이며 이 축별 필터와 무관합니다.

**Delta-spike guard (LPF 입력 전단, joint / task):** 위 LPF 로 들어가는 raw triplet 은 먼저 축별 delta guard 를 통과합니다 — 어느 축이든 마지막 **수용값** 대비 변화가 `fsm.contact_stop_force_guard_delta_n` (기본 4.0 N) 을 넘거나 NaN/Inf 면 triplet 전체를 마지막 수용값으로 대체합니다 (hold-last). 실기 세션 (260730_1017) 에서 index/ring `fz` 에 6 N 초과 샘플 6개가 각각 정확히 2 tick 지속했고, 그 6.406 N spike 하나가 필터 출력에 ~0.170 N 의 잔향을 남겼습니다 — 1 N 임계의 17 % 를 노이즈가 만든 셈입니다.

- **비교 기준은 last_accepted** (직전 raw 아님): spike 가 기준을 재설정하면 참값으로의 복귀가 거부됩니다.
- **연속 hold 상한** (`fsm.contact_stop_force_guard_max_hold_ticks`, 기본 2 tick): 이 cap 이 없으면 지속되는 진짜 접촉 step (`|Δ| > 4 N` 이 매 tick 성립) 이 영구히 거부되어 contact_stop 이 발동하지 못합니다. 2 tick 은 관측된 spike 를 전부 억제하면서 실접촉엔 필터 자체 군지연 (~6.7 ms) 대비 ~4 ms (500 Hz) 만 더합니다. **NaN/Inf 는 이 cap 을 타지 않습니다** — IIR 에 한 번 들어가면 delay line 이 영구 오염되고, 하류 max/clamp 가 그 NaN 을 그럴듯한 수로 세탁합니다.
- **dropout 시 guard 도 무효화**: 핑거팁 lane 이 invalid 가 되면 LPF primed 플래그와 함께 guard 기준도 버립니다 — 5 N 에서 끊겨 0.2 N 로 복귀한 lane 은 4.8 N step 이므로, 기준을 유지하면 stale 5 N 을 무한히 hold 하게 됩니다.
- **raw 는 불변**: guard 는 LPF 입력만 고릅니다. `GraspState`, pull estimator, grasp 판정, CSV 의 `ft_<f>_fx|fy|fz` 는 driver wire 값 그대로입니다.

CSV (`<device>_sensor.csv`) 는 같은 tick 에서 raw (`ft_<f>_fx`) → guarded LPF 입력 (`ft_<f>_fx_guarded`) → LPF 출력 (`ft_<f>_fx_filt`) → guard 판정 (`ft_<f>_force_guard_rejected`) 을 모두 남기므로, guard 효과를 오프라인에서 재구성할 수 있습니다 (raw 와 filtered 만으로는 "guard 가 hold 했다" 와 "필터가 지연됐다" 를 구분할 수 없습니다).

발행되는 `GraspState` 의 `force_magnitude` / `max_force` 는 **raw** 로 유지되므로 BT (`IsForceAbove` 등) 계약은 변하지 않습니다. `[contact_stop]` 로그는 판정에 쓰인 `fmax=` (필터) 와 `fmax_raw=` 를 함께 출력합니다. WBC 는 이 필터를 갖지 않습니다 (latch 없이 `phase.cpp` 가 raw `force_magnitude` 로 FSM 전이).

**Hold latch (contact_stop 모드 전용):** 접촉이 한 번 성립하면 동결이 **latch** 되어, 이후 접촉이 사라져도 (물체 미끄러짐 등) 명시적 release 전까지 위치를 유지합니다. Latch 는 아래 release-phase gate 또는 E-STOP 으로만 해제됩니다. 접촉 유지 중에는 LPF 출력을 계속 추종하고 (compliant), 접촉이 사라지면 마지막 LPF 출력을 고정합니다 (drift 방지). 핸드 상태 dropout (`devices[1].valid==false`) 중에도 latch 되어 있으면 마지막 명령을 유지합니다.

**Release-Phase Skip (contact_stop 모드 전용):** 사용자가 토픽(`/p1a/joint_goal`)으로 손을 여는 방향의 goal을 내린 경우에는 접촉 잔존 힘이 있더라도 contact_stop 동결을 자동으로 건너뛰고 latch 를 해제합니다. 아래 3개 조건이 모두 성립해야 release 의도로 인정됩니다 (ε = `fsm.contact_stop_release_eps` rad 히스테리시스, 기본 0.005):

- `thumb_cmc_fe`: `target > actual + ε` (각도 증가 = loosening)
- `index_mcp_fe`: `target < actual − ε` (각도 감소 = loosening)
- `middle_mcp_fe`: `target < actual − ε` (각도 감소 = loosening)

발동 시 `/rosout` 에 `[contact_stop] SKIP (release) dthumb_fe=... dindex_fe=... dmid_fe=...` 로그가 1초 간격으로 출력됩니다. 접촉 유지 중 동결은 `[contact_stop] FREEZE ...`, 접촉 없이 latch 로 유지 중일 때는 `[contact_stop] HOLD ...` 로그가 출력됩니다.

**Force-PI Grasp/Release 버튼 (GUI):** `demo_controller_gui` 의 Grasp 탭에 있는 `▶ Grasp` / `■ Release` 버튼은 **활성 모드가 `force_pi`** 일 때만 동작합니다. 그 외 모드에서 `grasp_command` srv 는 **사유와 함께 명시적으로 거부**합니다 (`ok=false` + `message`) — 조용히 무시하지 않습니다. 사유 문구의 SSoT 는 `GraspCommandRejectReason` (`demo_shared_config.hpp`) 이며 두 가지를 구분합니다: "`force_pi_grasp` 블록이 없음" (YAML 수정 필요) 과 "블록은 있는데 지금 모드가 force_pi 가 아님" (모드 전환 필요).

GUI 는 이 상태를 **버튼 아래 줄에 표시하고 직접 게이트**합니다 (패널이 Grasp 탭의 절반 폭 컬럼이라 버튼 옆에 두면 문장이 잘립니다). joint/task 컨트롤러는 활성 모드를 `grasp_controller_type` ROS 파라미터로 노출하므로 (`ros2 param get /demo_joint_controller/demo_joint_controller grasp_controller_type`), GUI 가 선택된 컨트롤러의 실제 모드를 읽어 `mode: force_pi` (녹색, 버튼 활성) 또는 `mode: contact_stop — Grasp/Release 는 force_pi 에서만 동작` (노랑, 버튼 비활성) 을 띄웁니다. 파라미터 서비스가 아직 안 뜬 경우에는 `mode: unknown` (회색) 으로 두고 버튼은 **활성 유지** — GUI 가 컨트롤러가 받아들일 명령을 막는 두 번째 게이트가 되면 안 되기 때문입니다. `demo_wbc_controller` 는 이 파라미터를 선언하지 않으며 (자체 FSM) 항상 활성입니다.

모드는 **같은 탭에서 바꿀 수도 있습니다**: readout 아래 `Grasp mode:` 콤보박스에서 값을 고르고 `Set Mode` 를 누르면 선택된 컨트롤러의 `grasp_controller_type` 을 set 합니다. 거부되면 컨트롤러가 돌려준 사유(`SetParametersResult::reason`)가 그 아래 줄에 **그대로** 표시됩니다 — 문구를 다시 쓰지 않는 이유는 그것이 "손을 펴라" 와 "PI grasp 를 놓아라" 를 가르는 유일한 정보이기 때문입니다. 사유 줄은 모드 readout 과 **별도 라벨**입니다 (readout 은 5초 catalog 폴링이 매번 덮어쓰므로 거기 두면 읽기 전에 사라집니다). 시도 후에는 성공·거부 **양쪽 모두** 파라미터를 다시 읽으므로, readout 은 GUI 가 *요청한* 모드가 아니라 컨트롤러가 *확인한* 모드입니다. 콤보박스는 실제 모드가 변할 때만 다시 seed 됩니다 — 5초 폴링도, **거부된 set 도** 조작자의 미적용 선택을 지우지 않습니다 (거부는 손을 펴고 다시 누르는 경우가 대부분이라 선택이 남아 있어야 합니다). `demo_wbc_controller` 선택 시에는 콤보박스와 `Set Mode` 가 비활성입니다 (파라미터 자체가 없음). GUI 밖에서 (`ros2 param set`) 바꾼 모드는 다음 무효화 시점까지 readout 에 반영되지 않을 수 있습니다 — 명령을 실제로 게이트하는 것은 이 표시가 아니라 컨트롤러입니다.

**모드는 런타임에 바꿀 수 있습니다.** CLI 도 같은 경로입니다:

```bash
ros2 param set /demo_joint_controller/demo_joint_controller grasp_controller_type force_pi
```

단 **손이 quiet 할 때만** 수락됩니다 — contact_stop latch 가 걸려 있지 않고, force_pi FSM 이 `kIdle` 이어야 합니다. 아니면 사유와 함께 거부되고 모드는 안 바뀝니다 (판정의 SSoT 는 `GraspModeChangeRejectReason`). `force_pi` 요청은 `force_pi_grasp` 블록이 있는 config 에서만 가능합니다 — 블록 유무가 *capability*, 모드가 *지금 어느 법칙이 도는가* 이며 두 축은 독립입니다 ([agent_docs/controllers.md](../agent_docs/controllers.md#graspcontroller-force-pi-internal-only)). 이미 활성인 모드를 다시 요청하는 것은 quiet 하지 않아도 항상 수락됩니다 (no-op).

force_pi 모드에서는 phase 전이(`Idle → Approaching → Contact → ForceControl → Holding → Releasing`)가 `[grasp:force_pi] phase X -> Y target_force=...N` 로그로 출력되며, 500Hz 제어 루프에서는 2초 간격으로 `[grasp] type=... active=.../... max_force=...N` 상태 스냅샷이 출력됩니다.

---

### DemoTaskController (Index 5)

태스크 공간 CLIK + 핸드 Quintic 궤적 -- 감쇠 의사역행렬과 영공간 보조 태스크를 사용합니다. Task-space trajectory 기반으로 TCP 목표 보간을 수행합니다.

**로봇 암 제어 법칙:**

```
pos_error = x_traj(t) - FK(q)
J^# = J^T (J J^T + lambda^2 I)^{-1}
N   = I - J^# J
dq  = kp * J^# * pos_error + traj_velocity + null_kp * N * (q_null - q)
q_des += clamp(dq, +/-v_max) * dt          (trajectory 갱신 시 q_des = q_actual로 초기화)
q_cmd  = q_des
```

**타겟 입력 형식:**

| 모드 | target[0:3] | target[3:6] |
|------|------------|-------------|
| `control_6dof=false` | TCP 위치 (x,y,z) | 영공간 관절 레퍼런스 |
| `control_6dof=true` | TCP 위치 (x,y,z) | 자세 (roll, pitch, yaw) |

**파라미터** (YAML 기본값):

| 파라미터 | YAML 기본값 | 설명 |
|---------|------------|------|
| `kp_translation` | `[5.0, 5.0, 5.0]` | 위치 비례 게인 (x, y, z) [1/s] — **정확히 3원소** 시퀀스여야 하고 과다·미달·스칼라는 configure 실패. `ros2 param` 콜백이 이미 강제하던 계약을 YAML 로더도 따른다 |
| `kp_rotation` | `[2.0, 2.0, 2.0]` | 자세 비례 게인 (rx, ry, rz) [1/s] — 같은 길이 계약 |
| `singularity_threshold` | `0.02` | σ₀ — `σ_min(J) < σ₀` 에서만 §6.5 DLS 감쇠가 붙기 시작. λ_max 와 **같은 세 지점** (로더·`ros2 param` 콜백·사용 지점) 에서 `rtc::compliance::FloorSigma0` 으로 floor — σ₀ ≤ 0 은 셸을 좁히는 게 아니라 λ²=0 을 상시 반환해 §6.5 를 **끈다** (NUM-2), 그리고 `set_gains()` 는 앞의 두 지점을 모두 우회한다. 비유한 값은 세탁하지 않고 통과 (`max(1e-6, NaN) == 1e-6` 은 어떤 자세도 들어오지 않는 셸이라, 무장한 것처럼 보이는 채로 §6.5 가 꺼진다). **σ₀ 하나가 두 분기를 함께 파라미터화**하며 σ_min 의 스케일은 6-DOF(혼합 단위 6×n)와 3-DOF(병진 전용 3×n)에서 다르다 — 이 값은 출하 config 가 쓰는 6-DOF 기준이다 |
| `max_damping` | `0.05` | λ_max — §6.5 램프의 상한. 로더·콜백·**사용 지점(tick)** 셋 다 `rtc::compliance::FloorMaxDamping` 으로 floor (NUM-1; `set_gains()` 는 SeqLock 에 POD 를 직접 써 configure 를 우회하므로 tick half 가 필요하다). 비유한 값은 세탁하지 않고 그대로 통과시켜 하류 finite 검사로 보낸다 |
| ~~`damping`~~ | — | **은퇴.** 상수 λ 를 지정하던 키. 남아 있으면 **경고 후 무시**되며 `max_damping` 으로 매핑되지 않는다 — 상수 λ 와 램프의 상한은 같은 양이 아니라 어떤 매핑도 추측이 되기 때문 (`rtc_controllers` 의 다섯 스키마가 같은 판정을 내렸다). 지우고 위 두 키를 명시할 것 |
| `null_kp` | `0.5` | 영공간 관절 센터링 게인 — YAML 로더·`ros2 param` 콜백·사용 지점 모두 `rtc::FloorNonNegativeGain` floor (NUM-6). 음수는 posture 를 목표에서 **멀어지는** 방향으로 몰고 `(I − J⁺J)` 가 그걸 task 로부터 가려 fault 없는 조용한 drift 가 된다 |
| `enable_null_space` | `false` | 영공간 활성화 (3-DOF 모드에서만 동작) |
| `control_6dof` | `true` | 6-DOF 제어 활성화 |
| `trajectory_speed` | `0.1` m/s | TCP 병진 궤적 속도 |
| `trajectory_angular_speed` | `0.5` rad/s | TCP 회전 궤적 속도 (6-DOF) |
| `hand_trajectory_speed` | `1.0` rad/s | 핸드 궤적 속도 |
| `max_traj_velocity` | `0.5` m/s | 최대 TCP 병진 속도 |
| `max_traj_angular_velocity` | `1.0` rad/s | 최대 TCP 각속도 |
| `hand_max_traj_velocity` | `2.0` rad/s | 핸드 최대 속도 |
| `virtual_tcp_mode` | `"disabled"` | Virtual TCP 모드 (아래 참조) |
| `virtual_tcp_offset` | `[0, 0, 0]` | Constant 모드 오프셋 [x,y,z] (TCP 프레임, m) |
| `fsm.pi_rotation_margin` | `0.15` rad | π 근처 quintic 궤적 분할 임계값. 범위 [0, π/2] (필수 키) |
| `fsm.contact_stop_release_eps` | `0.005` rad | contact_stop release 히스테리시스. 범위 [0, 0.1] (필수 키) |
| `fsm.contact_stop_lpf_cutoff_hz` | `20.0` Hz | contact_stop latch hold 위치 Bessel LPF cutoff. 범위 (0, control_rate/2) (선택 키, 기본 Gains 값) |
| `fsm.contact_stop_force_lpf_cutoff_hz` | `50.0` Hz | 핑거팁 힘 축별 Bessel LPF cutoff. 범위 (0, control_rate/2) (선택 키) |
| `fsm.contact_stop_force_guard_delta_n` | `4.0` N | 힘 LPF 입력 delta-spike guard 의 축별 |Δ| 거부 임계. 범위 (0, 1000] (선택 키) |
| `fsm.contact_stop_force_guard_max_hold_ticks` | `2` ticks | 지속 out-of-band 값을 수용하기까지의 연속 hold 상한 (0 = delta hold 비활성, NaN/Inf 거부는 유지). 범위 [0, 1000] (선택 키) |
| `command_type` | `"position"` | 출력 타입 — `"position"` (PD 위치 추종) / `"torque"` (직접 토크) / `"pd_feedforward"` (PD 위치 + τ_ff, mujoco_sim 은 qfrc_applied 주입·중력보상 off) |

**게인 업데이트 레이아웃 (17개 요소):**
`[kp_trans*3, kp_rot*3, singularity_threshold, max_damping, null_kp, enable_null(0/1), control_6dof(0/1), traj_speed, traj_angular_speed, hand_traj_speed, max_vel, max_angular_vel, hand_max_vel]`

#### Virtual TCP (핑거팁 기반 제어점)

기본적으로 CLIK는 tool0 (TCP) 프레임을 제어점으로 사용합니다. Virtual TCP를 활성화하면 핑거팁 기구학으로부터 계산된 가상 TCP 프레임을 제어점으로 사용합니다.

```
T_base_vtcp = T_base_tcp(q_arm) * T_tcp_vtcp(q_hand)

J_vtcp_linear  = J_tcp_linear - skew(R_tcp * d) * J_tcp_angular
J_vtcp_angular = J_tcp_angular
```

여기서 `d = T_tcp_vtcp.translation()`이고, hand joint은 arm CLIK 관점에서 상수로 취급됩니다. Hand joint이 변하면 virtual TCP도 매 루프에서 자동으로 갱신됩니다.

| 모드 | `virtual_tcp_mode` | 설명 |
|------|-------------------|------|
| 비활성화 | `"disabled"` | tool0을 제어점으로 사용 (기본값) |
| Centroid | `"centroid"` | 4개 핑거팁 위치의 평균을 virtual TCP position으로 사용 |
| Weighted | `"weighted"` | 접촉력(contact force) 기반 가중 평균 — 접촉 중인 핑거팁에 가중치 부여 |
| Constant | `"constant"` | `virtual_tcp_offset`으로 지정한 고정 오프셋 (TCP 프레임 기준) |

> **주의:** Centroid/Weighted 모드는 `tree_models`가 활성화되어 있어야 합니다 (hand FK 필요).

모드와 무관하게, 계산된 제어점이나 그것이 딛는 arm TCP pose 에 비유한(non-finite) 값이 섞이면
`ComputeVirtualTcp` 는 **invalid 을 반환**하고 그 tick 의 vtcp 갱신은 건너뛴다 (직전 제어점 유지).
가중 평균의 `total_weight` 검사나 centroid 의 활성 개수 검사는 크기 비교라 NaN 을 통과시키므로,
판정은 발행 직전 값에서 한 번 이뤄진다.

##### 제어 frame 계약

Virtual TCP 는 **매 tick 유효한 것이 아니다** — hand FK 가 fingertip pose 를 못 내는 tick (closed-chain walk-in, hand device invalid) 에는 제어점이 tool0 로 떨어진다. 이때 목표를 그대로 두면 두 frame 의 차이가 CLIK task error 로 소비되어 arm 이 튄다 (실측 p1b: 0.21 m + ~90°). 따라서 목표에는 **authoring 된 제어 frame** (`ControlFrameId` — vtcp 여부 + 참여 fingertip mask) 이 태깅되고, CLIK 는 그 frame 이 이번 tick 의 제어 frame 과 일치할 때만 돈다 ([support/virtual_tcp.hpp](include/integrated_bringup/support/virtual_tcp.hpp) `ClassifyFrameTransition`).

| 상황 | 동작 |
|---|---|
| frame 일치 | 기존 CLIK 그대로 (`disabled` 모드는 항상 여기 — 동작 불변) |
| frame 전환 + 외부 목표 없음 | hold seed 를 새 제어점으로 재-seed → 오차 0, arm 무동작 |
| frame 전환 + 외부 목표 | 목표를 보존한 채 arm hold, frame 복귀 tick 에 그대로 실행 |
| 외부 목표가 `kVtcpFrameWaitTicks`(1000 tick) 초과 대기 | 목표 만료 + WARN, 현재 frame 으로 hold 복귀 |
| 참여 fingertip 집합 변화 (centroid/weighted) | hold seed 는 재-seed, 외부 목표는 새 제어점에서 재계획 |

목표는 컨트롤러의 *의도된* frame (mode ≠ disabled) 에서 authoring 된 것으로 간주한다 — walk-in 중 잠깐 tool0 인 것을 tool0 목표로 재해석하지 않는다. GUI 는 frame 확정 전 task target 입력을 비활성화해 이 가정을 성립시킨다.

#### Closed-chain hand FK (extended-URDF 전용)

hand URDF 가 **loop closure** 를 가지면 (`urdf.extended: true` + `<stem>.closure.yaml` sidecar; 예: 4-bar 손가락 링키지), loop-passive 관절 **하류**의 fingertip 은 tree 모델(passive 를 reference 형상에 동결)로 FK 하면 운영점 이탈 시 큰 오차가 난다 (측정 ~5.6 mm/°). 이를 위해 task/joint 컨트롤러는 fingertip FK 를 **closed-chain-consistent** 로 계산하는 `ClosedChainHandFk` 헬퍼([support/closed_chain_hand_fk.hpp](include/integrated_bringup/support/closed_chain_hand_fk.hpp), `rtc_urdf_bridge::RtClosedChainHandle` 래핑)를 배선한다.

- **자동·topology-driven·dormant**: `on_configure` 에서 (a) builder 에 closure 구속이 있고 (b) fingertip 이 loop-passive 관절 하류일 때만 활성. 그 외(대부분의 plain-URDF 로봇)는 비활성 → 기존 serial `RtModelHandle` 경로가 **byte-for-byte 동일**. 현재 `ur5e_p1a`/`iiwa7_leap` 는 `extended` 미설정이라 비활성이다.
- **RT-safe**: 매 tick 측정 actuated q 로 passive DoF 를 warm-start + 고정 K=2 Newton DLS 사영(preallocated, no-alloc). 헬퍼가 status 를 내부 소비해 fingertip pose 캐시를 **per-tip** 갱신한다: 소스 유효·결과 유한(sources_ok && !held && finite closure)이면 **loop 하류** tip 은 loop-trustworthy(`!singular && closure_error<임계`)한 tick 에서만, **비하류** tip 은 유한 tick 이면 항상 갱신하고, 그 외에는 직전 유효 pose 를 hold 한다. 비하류(serial 등가) tip 의 pose 는 actuated q 만의 함수라 loop 미수렴/특이와 무관하므로, 하류 tip 이 hold 되는 tick 에도 vtcp 입력이 붕괴하지 않는다. 소스 device/channel 이 invalid 인 tick 은 사영 자체를 건너뛰어(0-fill 된 q 를 사영하면 handle 내부 warm-start seed 가 오염돼 복구 tick 재수렴 실패) 직전 loop-consistent seed 를 보존한다. 독립 관절 소스는 hand device 에 한정되지 않으므로(arm+hand 스팬 가능) 진입 게이트는 device 인덱스가 아니라 per-source validity 로 판정한다. 활성 시 loop 하류 fingertip 은 loop-consistent, 비하류 fingertip 은 full-model FK(serial 등가)로 **모두** 서비스된다. closure 가 있어도 hand-root 프레임이 full model 에서 안 풀리거나 closure 가 ill-posed 면 안전하게 serial 로 fallback.
- **WBC**: DemoWbcController 도 동일 `ClosedChainHandFk` 를 배선하되 **관찰/publish 표면 전용**이다 — `InitHandModel`(secondary hand-only tree = `p1b`) + `ConfigureClosedChainHandFk`(OnDeviceConfigsSet) + per-tick `ComputeHandFingertipFk` 가 fingertip 을 arm TCP(`tcp.act`)로 base 합성해 `task_link_poses`(kHandTip) 로 publish 한다. **이 publish-surface FK 는 TSID EOM 축약과 같은 사영을 공유한다** — provider 가 활성이면 `ConfigureClosedChainHandFk` 가 그 사영을 빌려 배선하고(`ClosedChainHandFk` borrowed 모드), fingertip 은 자기 핸들을 만들지 않는다. tick 당 `RtClosedChainHandle::Update` 가 2회 → 1회 (ur5e_p1b 실측 median 28.0 µs 회수 = 500 Hz 예산의 1.4%). 채택 조건은 두 축의 AND 다: 이번 tick 에 사영이 실제로 돌았고(provider 실행 카운터 증가) 그 입력 q 가 전부 이번 tick 측정값일 것(`arm_readable_ && hand_readable_` — cache 가 블록을 hold 한 tick 은 last-good 유지). 운동학 status 는 `UpdateDynamics` 가 held 를 세우기 **전** 스냅샷을 쓰므로 비유한 속도 tick 에도 fingertip pose 는 살아 있다. provider 비활성(비-extended·좌표 미매칭)이면 기존 owning 경로로 fallback 한다 — 이 판정은 배선 시점(configure)에 한 번 내려지므로, 배선 **후** provider 가 사라진 tick 에는 빌릴 사영이 없다: 그 tick 은 직전 pose 를 유효한 것으로 계속 내보내지 않고 fingertip TF 를 **withhold** 한다 (RT tick 이라 로깅으로는 알릴 수 없다). **TSID EOM 동역학**: extended 로봇(control model==actuated)에서는 `PinocchioCache` 의 EOM 항 `M/h/g` 를 `WbcReducedDynamicsProvider`(→ RT-safe `RtClosedChainHandle::UpdateDynamics`) 가 **loop-consistent 축약값**으로 덮는다 (open-chain frozen-loop M/h/g → 축약 M_a/g_a/h_a). **contact frame 은 loop-하류에 한해 J·oMf·dJv 가 loop-consistent 로 override** 된다 (L2-exact dJv — drift 는 `GetFrameClassicalAccelerationDrift`; held/singular tick 은 J/oMf/dJv 3값 last-good hold). task(registered) frame 은 frozen-loop 유지, CLIK(kinematic)·MPC(handler mode)는 무변경. 비-extended 는 provider 미주입 → byte-for-byte. arm TCP FK(`tip_frame_id_`)는 종전대로. serial `hand_handle_` 은 closure 활성 시 non-null gate 겸 비-extended fallback 이며, `SetJointOrder` 는 `!closed_hand_fk_.active()` 일 때만 적용한다(loop-locked DoF 를 뺀 reduced serial tree 는 device 관절 전체와 매핑되지 않으므로).

**E-STOP:** 안전 위치 `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad로 이동, 핸드는 현재 위치 유지

### DemoWbcController (Index 6)

UR5e + 10-DoF 핸드를 단일 16-DoF 모델로 통합한 whole-body controller. TSID QP가 풀어내는 최적 가속도 `a*`를 semi-implicit Euler로 적분해 위치 명령을 산출하고, 6-단계 FSM (slots 2 & 5 reserved) 이 phase별 task 가중치/contact 활성화를 자동 전환한다. Kinematic WBC(CLIK-QP position backbone)/Dynamic WBC(hand τ_ff overlay) 구조와 TSID task/constraint 세부는 [agent_docs/controllers.md](../agent_docs/controllers.md)가 SSoT — 본 절은 bringup 관점(YAML 위치·launch 사용법·GUI 연동)만 다룬다.

> Extended (closed-chain) 손에서는 제어 모델을 `PinocchioModelBuilder::GetActuatedModel()` 우선으로 선택하고 EOM은 `WbcReducedDynamicsProvider`가 loop-consistent 값으로 대체한다 (위 "Closed-chain hand FK" 절 참조). 비-extended 손은 기존 `GetTreeModel("wbc")` fallback 경로로 byte-for-byte 동일하게 동작한다.

#### 6-Phase FSM (slots 2 & 5 reserved)

모든 비-fallback phase 는 TSID QP 를 돈다. `grasp_cmd=2`(RELEASE)는 active grasp phase(`kApproach`/`kClosure`/`kHold`)에서 즉시 `kRelease`로 preempt, `grasp_cmd=0`(abort)도 동일하게 `kIdle`로 복귀. 슬롯 2·5는 reserved(publish 안 됨). GUI의 arm SE3 target / hand joint target은 `kRelease`/`kFallback`을 제외한 모든 phase에서 매 tick 상시 반영된다(phase-entry edge 대기 없음).

| Phase | 제어 모드 | 진입 조건 | 종료 조건 |
|-------|----------|----------|----------|
| `kIdle` (0) | TSID QP (SE3 hold @ current TCP + posture regulate @ init snapshot). 진입 시 `SeedHoldFromMeasured` 가 측정 config 를 `q_des_target_full_`(posture ref) + joint_goal mirror + `tcp_goal_` 로 스냅샷 → 외란 시 init 으로 복원하는 stiff hold (InitPositionHold). 진입 edge 없는 첫 enable 은 first-tick self-init 에서 동일 시드 — 단 이 self-init 은 **모든 구성 device (arm+hand) 가 valid 측정 상태를 스트림할 때까지 defer** 되고 그동안 passthrough hold 를 명령한다 (hand device 가 늦게 올라오는 startup race 에서 posture ref hand block 이 zero 로 고정돼 손가락을 0 으로 붕괴시키는 것을 방지; joint/task 컨트롤러는 arm 즉시 시드 + hand 시드만 per-device 플래그로 defer). | 초기 / `grasp_cmd=0` / `release_done_` | `grasp_cmd=1` + `robot_new_target` |
| `kApproach` (1) | TSID QP (SE3 quintic ramp → grasp pose) | kIdle 종료 | `tcp_goal_valid && ||tcp_err|| < epsilon_pregrasp` |
| `kClosure` (3) | TSID QP + contact + ForceTask | kApproach 종료 | active fingertip force ≥ N개 (`min_contacts_for_hold`) |
| `kHold` (4) | TSID QP + contact + ForceTask | kClosure 종료 | slip 감지 시 `kFallback` (RELEASE preempt 는 top-level guard) |
| `kRelease` (6) | TSID QP (SE3 hold) + 2-stage overlay (contact ramp `release_ramp_sec` → finger open) | `grasp_cmd=2` from active grasp phase (`kApproach`/`kClosure`/`kHold`) | `release_done_` (stage 1 hand trajectory 완료) |
| `kFallback` (7) | Position hold | QP 연속 실패 N회, slip/deformation | `grasp_cmd=0` (수동 복구; top-level abort guard 면제) |

#### Runtime Gains (per-controller ROS 2 parameters)

`/demo_wbc_controller` LifecycleNode 가 노출하는 게인 파라미터:

| Parameter | 타입 | 범위 / 비고 |
|-----------|------|-------------|
| `arm_trajectory_speed` | double | `[1e-6, ∞)` (clamp). 현재는 hand quintic 외 미사용 (TSID-everywhere) — 향후 fallback path 가 부활하면 활성. |
| `hand_trajectory_speed` | double | `[1e-6, ∞)` (clamp). `kRelease` stage 1 finger-open quintic duration 산출 |
| `arm_max_traj_velocity` | double (read-only) | trajectory 진행 중 arm 관절 최대 속도 [rad/s] |
| `hand_max_traj_velocity` | double (read-only) | trajectory 진행 중 hand 모터 최대 속도 [rad/s] |
| `tcp_trajectory_speed` | double | `[1e-6, ∞)` (clamp). MPC-disabled SE3 ramp 의 TCP 병진 속도 [m/s]. SE3-task active 한 phase 진입 edge 에서 `InitTcpTrajectory` 가 quintic rest-to-rest segment 를 구성 |
| `tcp_trajectory_angular_speed` | double | `[1e-6, ∞)` (clamp). MPC-disabled SE3 ramp 의 TCP 각속도 [rad/s] |
| `tcp_max_traj_velocity` | double | TCP 병진 속도 cap [m/s] (quintic peak = `1.875·d/T`) |
| `tcp_max_traj_angular_velocity` | double | TCP 각속도 cap [rad/s] (quintic peak = `1.875·d/T`) |
| `pi_rotation_margin` | double | π-rotation defense 임계 [rad]. `ang_dist > π - margin` 이면 mid-pose split 으로 2-segment trajectory |
| `se3_weight` | double | TSID `SE3Task` 가중치 (런타임 튜닝) |
| `force_weight` | double | TSID `ForceTask` 가중치 |
| `posture_weight` | double | TSID `PostureTask` 가중치 |
| `mpc_enable` | bool | MPC output consumption 런타임 토글. 빌드타임 `mpc_enabled_` (YAML `mpc.enabled`) 와 AND 결합되므로 YAML이 false면 무시됨 |
| `riccati_gain_scale` | double | `[0,1]` 자동 clamp. `RiccatiFeedback`이 `scale · K · Δx`에 적용 |

Force-PI grasp는 별도 `~/grasp_command` srv ([rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv), Phase A) — `command=GRASP/RELEASE` + `target_force` (one-shot transition, parameter 부적합).

#### YAML 구조 (`config/ur5e_p1a/controllers/demo_wbc_controller.yaml`)

주요 최상위 키: `tsid.tasks` (posture/se3_tcp/force/contact_consistency/object_wrench/internal_force/object_se3), `tsid.constraints` (eom/joint_limit/friction_cone/torque_limit), `tsid.contacts.*`/`tsid.force_pi`/`tsid.object_frame` (contact·force-PI·object 옵션), `tsid.phase_presets`, `tsid.wqp.solver`, `integration` (`force_rate_alpha` 등 필수 키), `fsm`, **`arm_dof`** (필수 — 런타임 arm DoF), **`estop.arm_safe_position`** (필수 — 길이가 `arm_dof`와 일치해야 하며 불일치 시 configure 에서 throw), **`mpc`** (`enabled`/`engine`/`max_stale_solutions`/`phase_config_path`+`contact_light_path`+`contact_rich_path`/`riccati.*`). `mpc.enabled: false`가 기본값이라 MPC는 inert이고 TSID가 self-hold한다. 각 키의 의미·기본값·제약은 YAML 자체의 인라인 주석 + [agent_docs/controllers.md](../agent_docs/controllers.md)를 SSoT로 참조.

#### MPC 통합 동작

`mpc.enabled: true`일 때 `on_activate`에서 aux 스레드로 MPC 스레드가 기동되며, `mpc.engine`이 `"mock"`(`MockMPCThread` placeholder — 선형 trajectory + identity Riccati gain, TSID self-hold와 bit-identical) 또는 `"handler"`(`HandlerMPCThread` + `MPCFactory` + `GraspPhaseManager` — 실제 Aligator ProxDDP solve, 초기화 실패 시 mock으로 자동 폴백)를 선택한다. `rtc::mpc::MPCThread`는 CM RT loop과 같은 `PeriodicRtThread` 기반 timing 인프라를 공유하며, per-MPC-tick 샘플이 `<session>/timing/mpc_timing_log.csv`에 쌓인다. 매 RT tick `ComputeTSIDPosition`이 최신 `(q, v)`를 MPC 스레드에 WriteState하고, MPC solution을 cubic-Hermite 보간한 `q_ref/v_ref/a_ff` + Riccati 피드백을 TSID reference로 주입한다 — solution 부재/stale 시 TSID self-hold로 자동 폴백. Shutdown 순서·dim-mismatch gate 등 구현 세부는 `src/controllers/wbc/` 소스와 [agent_docs/controllers.md](../agent_docs/controllers.md) 참조.

##### GraspPhaseManager 연동

`engine: "handler"`일 때 WBC 6-state FSM이 authoritative이며 `OnPhaseEnter`에서 `GraspPhaseManager::ForcePhase`로 grasp 측 FSM을 동기화한다 (첫 `ForcePhase`가 grasp manager를 mirror 모드로 latch — guard/command 자가 진행 억제). Grasp 측 phase별 OCP 설정은 `config/ur5e_p1a/controllers/mpc/phase_config.yaml`, factory 설정은 `mpc/contact_light.yaml`/`mpc/contact_rich.yaml`에서 로드된다 (두 YAML의 `mpc.model:` 블록은 구조 동일 필수 — cross-mode swap이 같은 `RobotModelHandler`를 공유).

#### 사용법 (시뮬레이션)

```bash
# MPC 비활성 (TSID self-hold만)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py initial_controller:=demo_wbc_controller

# MPC 활성 (sim_ur5e_p1a.launch.py는 launch arg 오버라이드 지원)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py \
    initial_controller:=demo_wbc_controller enable_mpc:=true

# 1. Pre-grasp 위치로 approach  (target 토픽은 active controller가 소유)
ros2 topic pub /demo_wbc_controller/ur5e/joint_goal rtc_msgs/RobotTarget \
  "{joint_target: [0.0, -1.2, 1.0, -1.4, -1.57, 0.0]}" -1

# 2. Hand close target
ros2 topic pub /demo_wbc_controller/p1a/joint_goal rtc_msgs/RobotTarget \
  "{joint_target: [0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]}" -1

# 3. grasp 시작 (Force-PI one-shot srv) + se3_weight 튜닝
ros2 service call /demo_wbc_controller/grasp_command \
  rtc_msgs/srv/GraspCommand "{command: 1, target_force: 2.0}"
ros2 param set /demo_wbc_controller se3_weight 100.0
ros2 param set /demo_wbc_controller force_weight 10.0

# 4. Riccati gain을 반으로 축소 (fade-in) + MPC 토글
ros2 param set /demo_wbc_controller mpc_enable true
ros2 param set /demo_wbc_controller riccati_gain_scale 0.5

# 5. Release
ros2 service call /demo_wbc_controller/grasp_command \
  rtc_msgs/srv/GraspCommand "{command: 2, target_force: 0.0}"
```

> `robot_ur5e_p1a.launch.py`의 `enable_mpc` launch arg는 선언만 되어 있고 OpaqueFunction을 쓰지 않으므로 런타임 `mpc_enable` 파라미터로 토글해야 한다. `sim_ur5e_p1a.launch.py`는 `ctrl_overrides`에 `demo_wbc_controller.mpc.enabled`를 주입해 launch 시점에 반영한다.

#### Anomaly 보호

- **Slip**: EMA-smoothed `|df/dt|` > `slip_rate_threshold` (기본 5.0 N/s)
- **Deformation**: `||displacement||` > `deformation_threshold` (기본 0.015 m)
- **QP 실패** (fallback 분리): position 백본인 **Kinematic (CLIK) QP** 실패만 critical — 연속 `max_qp_fail_before_fallback`회 (기본 5) → `kFallback` 진입. **Dynamic (TSID) QP** 실패는 non-critical — 해당 tick hand τ_ff 만 drop (position 은 CLIK 가 계속 소유), `kFallback` 미진입. 두 카운터는 `dyn_qp_fail_count_` / `kin_qp_fail_count_` 로 분리되며 `wbc_diag.csv` 에 `qp_fail_count`(dynamic) + `kin_qp_fail_count` 컬럼으로 기록

**E-STOP:** `estop.arm_safe_position` (YAML 필수 키, 기본 `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad — 길이는 `arm_dof` 와 일치해야 한다)로 이동, 핸드는 현재 위치 유지, contact 비활성화

---

## 로깅 (Logging)

### DemoCatchingController (dynamic_catching S5–S7)

설계·결정의 SSoT 는 [docs/dynamic_catching/IMPLEMENTATION_PLAN.md](../docs/dynamic_catching/IMPLEMENTATION_PLAN.md) 이고 (충돌 시 그 문서가 우선), 여기에는 **운용 표면**만 적는다.

- **무장되면 팔은 스스로 `planner.wait_pose` 로 관절공간 homing 하고 한 투척마다 한 순환을 돈다** (S7, 아래 §Catching sim trials·L7 §4.1). 손은 S7.1 부터 시퀀서가 소유한다. `diagnostic.hand_step: true` (출하 false) 인 프로파일만 시퀀서 없이 무성형 계단 목표를 받는다 (S4.2 측정용)
- **무장 채널**: 컨트롤러 노드의 읽기·쓰기 파라미터 `catching.enable` (기본 `false`). `ros2 param set /demo_catching_controller/demo_catching_controller catching.enable true` (노드 이름은 네임스페이스·이름이 모두 `demo_catching_controller` 다 — 앞의 절반만 쓰면 "Node not found"). **컨트롤러가 스스로 내린다** — E-STOP 발동·해제와 fault 래치에서 RT tick 이 latch 를 내리므로, 해제 후 재개는 다시 `true` 로 올리는 명시적 행위를 요구한다 (P-1 (c)). 활성화도 무장이 아니다
- **E-STOP·fault**: `TriggerEstop`/`ClearEstop`/`ResetFault`/`ResetTargetInitialization` 는 atomic 요청·epoch 만 갱신하고, 되돌리는 동작의 유일 writer 는 `Compute()` 다. `ClearEstop` 은 컨트롤러 fault 래치를 풀지 않고 `ResetFault` 는 E-STOP 을 풀지 않는다 (`/rtc_cm/reset_fault` ↔ `/rtc_cm/clear_estop` 이 서로 다른 경로다)
- **실기 config 에서의 park**: claim 한 device 가 전부 `mujoco_native` 임을 증명하지 못하면 이 configure 는 **real-arm** 으로 판정되고, 이 컨트롤러가 소비하는 키 (`control_rate`·`robot.hand.*`) 에 provisional·TBD 가 있으면 `on_configure` 는 SUCCESS 를 내되 인스턴스를 DISABLED 로 두고 `on_activate` 가 거부한다 (L0 §5.3). configure 를 실패시키지 않는 이유는 CM 이 한 컨트롤러의 configure 실패로 **전체 bring-up** 을 거부하기 때문이다. 출하 프로파일은 아직 provisional 이므로 실기에서는 이 상태가 정상이다
- **sim 에서의 park (A-S5-12, S6-A)**: sim 에서도 소비 키가 **TBD** 면 configure 를 거부하지 않고 park 한다 — CM 이 한 컨트롤러 실패로 전체를 거부하므로 거부는 "로봇이 안 뜬다" 가 된다. 범위·순서·일관성 위반 (값이 정해졌는데 틀린 것) 은 여전히 sim configure 를 거부한다. park 된 컨트롤러는 활성화되지 않으므로 상태 토픽도 없다 — GUI 패널이 미수신일 때 그 가능성을 함께 안내한다
- **팔 추종 (S5.3)**: plan 이 있으면 `SampleAt(now_lead)` → soft-catch 기준 → 확장 CLIK (위치 3행 + 접근축 2행) → `devices[0]`. 모델은 팔+손 결합 모델이고 목표 frame 은 `urdf.extra_frames.catch_frame` 이다. **손은 solve 안에서 잠근다** — catch frame 이 손바닥에 달려 있어 손 관절이 Jacobian 에 들어오지만 이 컨트롤러는 손을 명령하지 않는다. CLIK 은 **명령값**에서 평가하므로 (D-6) 측정 상태는 루프에 들어오지 않고, 실추종은 `supervisor.track_err_abort` 가 감시한다. QP 실패·box 충돌은 **QP 없이** 관절별 가속 한계로 감속해 멈추고 (`supervisor.n_qp` 연속 실패면 fault 래치), E-STOP 중에는 아무것도 명령하지 않는다
- **plan 원천 (S6-A)**: 계획기 스레드 (`catching.planner.enabled`) 또는 `diagnostic.oracle_plan` (사용자가 ground-truth 로 잰 고정 포구점 하나 — 테스트·sim 용) 이 **같은 plan box** 에 쓰고, RT 는 매 tick 한 번 읽어 `JudgePlan` (L3 §5.2 (a)~(f): 유효 · 현재 activation · 같은 트랙 · 새 `plan_id` · 게시 나이 ≤ `io.t_stale` · 마지막 리셋 이후) 을 통과한 plan 만 TRACKING 에서 받는다. 두 writer 를 함께 켜면 park. 출하 두 로봇 모두 **계획기**를 켠다 (S6-B, R-4 — p1b 의 oracle 은 off 로 남아 S5.5 고정점 재현용). APPROACH 중 새 plan 은 계획기가 교체 규칙 (§4.7) 을 통과시킨 것만 오고, RT 는 freeze 창 (`planner.freeze.T_freeze`) 밖에서만 reset 없이 재조준한다 (`SetIntercept`)
- **계획기 스레드 (S6-A)**: `mpc` layout role 을 DemoWbc MPC 와 공유한다 (E-7 결정 J) — `ps -L` 에 `mpc_main` 이 둘 이상 보이는 것은 정상이고 (paused MPC 스레드 · Aligator OpenMP 워커), CM 이 한 컨트롤러만 활성화하므로 도는 것은 하나다. 수명: configure 에서 wake eventfd, `on_activate` 첫 문장에서 profile `mpc_off` 면 거부, lazy spawn → `Resume`, deactivate `Pause`, **`on_cleanup` 에서 join** (DemoWbc MPC 와 달리 소멸자까지 두지 않는다 — 스레드가 설정보다 오래 살면 oracle 로 재구성한 뒤에도 resume 돼 plan box writer 가 둘이 된다). vision 콜백이 새 궤적 수락 시 eventfd 로 깨우고 (RT tick 은 eventfd 를 건드리지 않는다), 타이밍은 1 Hz aux 타이머가 `<session>/timing/planner_timing_log.csv` 로 drain 한다. 키: `planner.enabled` · `wake_timeout_s` (0.05) · `budget_s` (0.020) · `wait_pose` (arm 관절 순서, provisional — S6-B 부터 IK seed)
- **계획기 탐색 (S6-B)**: 모델은 로봇 config 의 `urdf.sub_models.<arm>_catch` (arm root → catch frame 부모, R-3) 를 `planner.sub_model` 로 지명한 것이고, 계획기 전용 핸들로 만든다 (오프라인 catchability 지도도 같은 항목을 쓴다). 후보 = vision 격자, 판정 게이트 (입력·IK·manipulability·`planner.workspace.catch_box` 안의 p_c·p_stop) 는 제거, 순위 게이트 (불확실성·도달시간·γ 창·commit 선행·오차 예산) 는 `planner.score.penalty` 벌점 (결정 C/D). IK 는 사전 점수 상위 `planner.max_ik` 개만, `planner.budget_s` 까지. **결정값** (`sub_model`·`freeze.T_freeze`·`workspace.catch_box`·`hand.d_eff`·`hand.r_cap`) 이 비었거나 TBD 면 park (키 이름을 로그), 실기에서 `planner.provisional: true` 면 park. 시스템 모델이 없는 구성 (단위 fixture) 은 stub 탐색
- **계획기 이벤트 CSV (S6-B, 결정 E)**: `<session>/controllers/demo_catching_controller/planner_events.csv` — idle 이 아닌 wake 마다 한 행 (후보 수 · IK 수 · 판정 게이트별 탈락 수 · 선택 후보의 순위 게이트 비트 · 교체 결정 · 탐색 시간 · 수신→게시 지연). 계획기 스레드의 SPSC ring 을 1 Hz aux 타이머가 비운다 (계획기 스레드는 파일을 쓰지 않는다). 상태 메시지는 D-20 동결이라 여기 없는 것만 이 CSV 에 있다. plan 이 없을 때 상태 메시지의 `plan_reason` 은 계획기의 첫 병목이다 (GUI 가 이름으로 표시)
- **vision frame → 모델 world (S6-C)**: `catching.io.arm_base_frame` (URDF frame, 모델 root 에 강체) · `catching.io.base_T_world` (`{yaw_deg, translation}`, $p_{base}=R_z p_{world}+t$) 로 수신 궤적·공분산을 한 번 모델 world 로 옮긴다 — 계획기·catch_box·CLIK 이 그 좌표를 쓴다. ur5e_p1b 는 root 가 `base_link` 라 Rz(180°) 가 적용되고 (기동 로그 `vision frame 'world' → model world via 'base': yaw 180.0 deg`), 키가 없으면 경고 후 항등이다 (plan §11, [L1 §1](../docs/dynamic_catching/L1_io.md))
- **vision 입력 (S5.2)**: `catching.io.traj_topic` 의 `PointCloud2` 예측 궤적을 컨트롤러가 **직접 구독**한다 (`topics:` 의 `role:` 표에 없는 lane 이라 role 을 늘리지 않는다). best_effort · `KEEP_LAST(1)` — depth 는 ARCH-6 고정이고, 백로그가 생기면 **가장 최신** 예측만 소비한다. 필드는 이름으로 찾으므로 publisher 가 offset 을 바꿔도 따라가고, 이름·타입·개수가 다르면 거부하며 사유별 카운터가 남는다 (거부와 침묵은 RT 쪽에서 구별되지 않으므로 그 카운터가 유일한 진단이다). 수신 시 한 번 `t_ref = recv_steady − (recv_wall − stamp)` 로 옮기고 (D-2), stale 은 **steady 수신 나이**로만 판정한다
- **상태 토픽 (S5.4, D-20)**: `/<config_key>/catching_state` (`rtc_msgs/CatchingState`, `KEEP_LAST(1)`). 소유 형태는 `WbcState`·`GraspState` 와 같고 `PublishRole` 은 늘리지 않는다 (E-11). **필드는 S5~S9 superset 으로 한 번 동결**돼 있으며 이후 단계는 값만 채운다 — 단계마다 열이 늘면 한 단계 전 bag 을 못 읽는다. **모든 tick 이 body 를 싣는다** (PROC-7): E-STOP·stale·plan 없음·abort tick 도 발행하고, 그 tick 에 계산하지 않은 블록은 직전 값을 남기지 않고 지운다. 그래서 값이 고정돼 보이면 컨트롤러가 정말 같은 값을 다시 계산한 것이다
- **tick 레코드 CSV (S5.4)**: `catching_diag.csv` (`logs:` 의 `integrated_bringup/CatchingDiagLog`). 상태 토픽과 **같은 POD 한 벌**에서 나오므로 파일의 숫자와 화면의 숫자가 갈릴 수 없다. tick 마다 한 행이라 **tick 간극은 드롭된 행**을 뜻한다 (#234 P-20). `plot_rtc_log catching_diag.csv` 가 기준 vs 실현 가속도·추종 오차·solve time·슈퍼바이저 모드를 한 시간축에 그린다
- **GUI**: `demo_controller_gui` Control 탭의 Catching 패널 — 모드·사유, 입력 lane (n·generation·sequence·수신 나이·지평, 거부 카운터는 0 이 아닌 것만), plan, 추종 오차·CLIK 상태, 그리고 Arm/Disarm. **관측된 무장과 요청된 무장을 따로 보여준다** — tick 이 E-STOP·fault 에서 latch 를 내리므로 파라미터 set 이 성공해도 무장됐다는 증거가 아니고, 둘이 갈리는 순간이 봐야 할 상태다
- **읽기 전용 미러 파라미터**: `hand.q_open`/`q_pre`/`q_close`/`caging_mask`/`eta_close`/`rho_eps`/`T_close_e2e`·`control.dt`·`diagnostic.hand_step`·`planner.wait_pose`·`planner.freeze.T_freeze`·`joint_cmd.lag.T_arm`·`joint_cmd.lag.lead_enable` (뒤 넷은 S8-A 시행 러너용 — §Catching sim trials) — 오프프로세스 분석기가 YAML 이 아니라 **컨트롤러가 읽은 값**을 쓰게 하려는 것이다

### 로깅 레벨

레벨 분류 기준·공통 규칙(THROTTLE 매크로, 핫패스 포맷 인자, non-RT INFO 풍부화, 서브-로거 네이밍)은 [agent_docs/conventions.md](../agent_docs/conventions.md#logging) 가 SSoT. 이 패키지의 예시:

| 레벨 | 예시 |
|------|------|
| `FATAL` | URDF 로드 실패, 컨트롤러 등록 실패 |
| `ERROR` | 모델 빌드 실패, 필수 디바이스 누락 |
| `WARN` | YAML 로드 실패 (built-in defaults 사용), Grasp 명령 무시 |
| `INFO` | Force-PI 위상 전이, CommandGrasp/Release 트리거 |
| `DEBUG` | 상세 trajectory 진행률, gain 업데이트 추적 |

**패키지 고유 규칙:**

- **RT 핫패스의 포맷 인자 수를 최소화한다.** 예: `contact_stop FREEZE` 는 `target` 과 `actual` 을 동시에 덤프하는 대신 둘의 차(`err`)만 `[+.3f,+.3f,+.3f]` 로 찍는다 — 인자 5개로 접촉 해석에 충분한 정보를 제공. RT-critical 경로에서 9개 이상의 포맷 인자는 코드 리뷰에서 challenge 대상이다.
- 메시지 본문 내 `[grasp]` / `[contact_stop]` / `[force_pi]` 같은 짧은 태그는 *기능 영역*을 나타내며, 같은 클래스 안의 여러 흐름을 구분하기 위한 용도로 허용된다.

### 서브-로거 네임스페이스

네이밍 규약: `<package>.<controller_key>` — 점(`.`) 앞은 패키지, 뒤는 컨트롤러 (자세한 설명은 [agent_docs/conventions.md](../agent_docs/conventions.md) "Logging" 섹션).

| 서브-로거 | 사용처 |
|-----------|--------|
| `integrated_bringup.demo_joint_controller` | `DemoJointController` (joint-space 데모 컨트롤러, 500 Hz 핫패스) |
| `integrated_bringup.demo_task_controller` | `DemoTaskController` (task-space 데모 컨트롤러, 500 Hz 핫패스) |
| `integrated_bringup.demo_compliance_controller` | `DemoComplianceController` (task-space admittance 바인딩, 500 Hz 핫패스) |
| `integrated_bringup.demo_wbc_controller` | `DemoWbcController` (WBC + MPC 데모 컨트롤러, 500 Hz 핫패스) |
| `integrated_bringup.demo_catching_controller` | `DemoCatchingController` (dynamic_catching S5, 500 Hz 핫패스) |
| `integrated_bringup.demo_shared_config` | `demo_shared_config` YAML 로더 (init-time, non-RT) |

### THROTTLE 주기 표준

`integrated_bringup::logging` 네임스페이스에 정의된 상수만 사용한다 (값은 [bringup_logging.hpp](include/integrated_bringup/support/bringup_logging.hpp) 가 SSoT):

| 상수 | 용도 |
|------|------|
| `kThrottleFastMs` | contact_stop FREEZE/SKIP, force_pi 위상 전이 등 빠른 진행 표시 |
| `kThrottleSlowMs` | grasp 상태 스냅샷, 일반 반복 경고 |
| `kThrottleIdleMs` | 장기 유휴 / one-shot 전이 안전 그물 |

### compliance §7.3 관절 밴드 판독

위치-출력 바인딩 두 개 (`demo_compliance` · `demo_task`) 는 명령을 스스로 적분하고 compliance §7.3 관절 tail
([joint_command_tail.hpp](../rtc_controllers/include/rtc_controllers/compliance/joint_command_tail.hpp))
로 바운드한다. 그 tail 이 **이번 tick 에 몇 개 관절을 움직였는지**를 돌려주는데, 그 값이 없으면
**밴드에 눌려 멈춘 팔과 정착한 팔이 같은 평평한 궤적**으로 보인다 — 다른 어떤 출력도 둘을 구분하지
못한다. task 축에는 `compliance_diag.csv` 의 `disp_limited` 가 그 답을 주고, 관절 축의 답이 이것이다.

채널이 둘인 이유는 종료 경로다 — CM 의 `main()` 은 SIGINT 에서 executor 만 정리하고 리턴하므로
**Ctrl-C 는 lifecycle 훅을 타지 않는다**. 따라서 요약만 두면 Ctrl-C 로 끝낸 bring-up 에서는 아무것도
안 나온다.

| 채널 | 레벨 | 언제 | 읽는 법 |
|---|---|---|---|
| 발동 중 (`... joint band engaged: N joint(s) position-clamped, M rate-rebounded`) | `WARN`, `kThrottleSlowMs` | clamp 나 rate 재바운드가 **실제로 걸린 tick** 에서만 (`if (발동)` 안이라 정상 운전은 매크로에 도달조차 안 한다) | 이 줄이 찍히는 구간의 평평한 관절 궤적은 **정착이 아니라 pin** 이다. 타임스탬프가 곧 "언제 눌렸나" |
| 활성화 요약 (`... joint band engaged this activation: ...`) | `INFO` | `on_deactivate` — 컨트롤러 **전환** 및 명시적 `ros2 lifecycle set … deactivate/shutdown`. **Ctrl-C 에는 안 뜬다** | tick 창 (`ticks A..B`) 을 그 run 의 다른 로그·CSV 와 대조. 한 번도 안 걸렸으면 **줄 자체가 없다** |

`demo_compliance` 는 §7 의 `joint_limit_margin` (δ) 을 함께 찍고, `demo_task` 는 δ 노브가 없어 raw
device 한계에서 clamp 하므로 그 사실을 문구로 밝힌다 — 후자의 clamp 는 soft margin 이 아니라 실제
관절 한계 도달이라 더 무거운 판독이다. 누적기·근거는
[support/joint_tail_stats.hpp](include/integrated_bringup/support/joint_tail_stats.hpp).

### 실시간 필터링 예시

```bash
# Force-PI 위상 전이만 DEBUG 활성화
ros2 service call /integrated_rt_controller/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'integrated_bringup.demo_joint_controller', level: 10}]}"
```

콘솔 출력에 로거 이름을 표시하려면:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

---

## Launch 파일

### robot_ur5e_p1a.launch.py -- 실제 로봇

```bash
ros2 launch integrated_bringup robot_ur5e_p1a.launch.py robot_ip:=192.168.1.10
ros2 launch integrated_bringup robot_ur5e_p1a.launch.py use_mock_hardware:=true  # 모의 테스트
```

**Launch 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_ip` | `192.168.1.10` | UR 로봇 IP |
| `use_mock_hardware` | `false` | 모의 하드웨어 사용 (Jazzy) |
| `use_fake_hardware` | `false` | [호환성] Humble용 별칭 |
| `use_cpu_affinity` | `true` | CPU 격리 + DDS 핀닝 활성화 |
| `max_log_sessions` | `10` (= `config/ur5e_p1a/_base.yaml`) | 세션 폴더 최대 보관 수. default 를 그 YAML 에서 읽고 값을 RT 노드 파라미터로도 넘기므로, launch 쪽 정리와 노드 `on_configure` 정리가 같은 수를 본다 |
| `kinematics_params_file` | `$HOME/ur5e_calibration.yaml` | `ur_calibration` 산출 factory calibration YAML. nested `ur_rsp.launch.py`가 소비 → 드라이버의 `robot_description`/TF에 실측 반영. 아래 [UR 캘리브레이션](#ur-캘리브레이션-kinematics_params_file) 참조 |
| `headless_mode` | `true` | Teach Pendant의 External Control play 없이 headless 제어. 실로봇 운용 기본 true |
| `reverse_ip` | `0.0.0.0` | UR 컨트롤러가 PC로 reverse connection 시 사용할 PC IP. multi-NIC/RT 전용 NIC에서는 명시값 권장 |
| `launch_dashboard_client` | `true` | UR Dashboard client 노드 실행 (robot mode, program/power/brake). dashboard port 차단·완전 수동 운용 시 `false` |
| `controller_spawner_timeout` | `10` | controller_manager spawner의 load/activate 대기(초). 부하로 startup이 느리면 상향 |
| `activate_joint_controller` | `true` | `initial_joint_controller`를 시작 시 active로. `false`면 로드만 하고 inactive |
| `initial_joint_controller` | `forward_position_controller` | UR 드라이버가 처음 로드/활성화할 joint controller. RTC backend가 `/forward_position_controller/commands`에 publish |
| `enable_mpc` | `""` | layout profile 선택 — cset shield 와 `DemoWbcController` 활성화 게이트가 함께 본다. `false` 면 MPC 코어를 system cpuset 에 반납하므로 MPC 가 켜진 WBC config 는 활성화를 거부한다. 빈 값/`true` 는 코어를 예약. sim launch 와 달리 YAML `mpc.enabled` 를 오버라이드하지 않는다 — 런타임 소비 토글은 컨트롤러의 `mpc_enable` 파라미터 |
| `enable_tracing` | `false` | LTTng trace 캡처 (ros2_tracing). 출력: `<session_dir>/tracing/<trace_session_name>/` (`repo_scripts/scripts/timeline.sh`로 Chrome Trace 변환 가능). 1회 `./install.sh --tracing` 설정 필요 |
| `trace_session_name` | `""` | LTTng 세션 이름 — `<session_dir>/tracing/`의 leaf 디렉토리명. 빈 값 = `"trace"` |
| `trace_events_ust` | `""` | 콤마 구분 UST 이벤트. 빈 값 = ros2_tracing `DEFAULT_EVENTS_ROS` |
| `trace_events_kernel` | `sched_switch,sched_waking,sched_wakeup,irq_handler_entry,irq_handler_exit` | 콤마 구분 커널 이벤트. 빈 값 = kernel tracing 비활성 (UST만). `lttng-modules-dkms` + `tracing` 그룹 필요 |
| `sil_mode` | hand YAML 의 `sil_mode` (없으면 `off`) | `off` / `loopmodel` / `firmware`. `firmware` 는 loopback 으로 `fake_hand_firmware` 를 띄운다. 모드별 동작은 [udp_hand_driver/README.md](../udp_hand_driver/README.md) |

> **`robot_ur5e_p1b.launch.py`** (UR5e + proto_1b closed-chain hand) 는 위 인자 집합을 **동일하게** 노출한다 — 유일한 기본값 차이는 `robot_ip` (`192.168.0.3`). p1b는 hand config·`/p1b/joint_states` gate만 다르다.

**Launch 순서:**

1. 세션 디렉토리 생성 — **launch 실행 시점**에 `OpaqueFunction` 이 수행한다 (description 을 만들기만 하는 `--show-args` 등은 파일시스템을 건드리지 않는다). 루트는 `rtc_tools.utils.session_dir.resolve_logging_root()` 의 3단 체인: `$COLCON_PREFIX_PATH` 첫 entry 의 parent → cwd 상위 `install/+src/` 탐색 → `$PWD`. (`$RTC_SESSION_DIR` 는 이 체인이 아니라 그 위의 세션 결정 단계에 있다 — launch 가 *내보내는* 값이다.) 보통 `source install/setup.bash` 상태면 ws 의 `logging_data/YYMMDD_HHMM/`, 보관 수는 `max_log_sessions`
2. 환경 변수 설정 (`CYCLONEDDS_URI`, `RMW_IMPLEMENTATION`, `RTC_SESSION_DIR`, `RTC_RUN_ID` — 이번 launch 의 런 ID. 같은 분의 재기동이 같은 세션 디렉토리를 얻으므로 타이밍 CSV 안의 런 경계가 된다)
3. `cpu_shield.sh on --robot` -- CPU 격리 활성화
4. UR 드라이버 launch (`ur_robot_driver/ur_control.launch.py`)
5. Mock 모드인 경우 `forward_position_controller` 활성화 (3초 지연)
6. Hand UDP 노드 launch (`udp_hand_driver/udp_hand_node`)
7. RT 컨트롤러 노드 launch (실행 파일 = ROS 노드 이름 = `integrated_rt_controller` — 정렬됨)
8. UR 드라이버 CPU 핀닝 -> Core 0-1 (3초 지연)
9. RT 컨트롤러 DDS 스레드 핀닝 -> Core 3 (5초 지연, 비-SCHED_FIFO 스레드만). Layout v4 에서 DDS receive thread 가 `rt_callback` (Core 3 FIFO 70) 와 같은 코어에 co-pin 되어 cache locality 공유 (CFS 유지, SCHED_FIFO 가 무조건 선점하므로 RT 결정성 영향 없음)

**Lifecycle:** udp_hand_node, integrated_rt_controller 모두 LifecycleNode 기반. 런치 시 자동 configure → activate.

**CycloneDDS 최적화** (`cyclone_dds.xml`):
멀티캐스트 비활성화, 소켓 버퍼 확대(recv 8MB/send 2MB), write batching(8us), NACK 지연 최소화(10ms), 동기 전달 활성화. 상세: [`rtc_controller_manager` README](../rtc_controller_manager/README.md#설정-파일)

#### UR 캘리브레이션 (`kinematics_params_file`)

각 UR5e는 공장 출하 시 관절 kinematics에 로봇별 오차가 있다. `ur_calibration`으로 실측 YAML을 추출해 `kinematics_params_file`로 넘기면 UR 드라이버가 publish하는 `robot_description`/TF에 반영된다.

```bash
# 1) 로봇별 calibration YAML 추출 (1회, robot_ip 대상)
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.10 \
  target_filename:=/absolute/path/to/ur5e_calibration.yaml

# 2) bringup에 전달
ros2 launch integrated_bringup robot_ur5e_p1b.launch.py \
  robot_ip:=192.168.1.10 \
  kinematics_params_file:=/absolute/path/to/ur5e_calibration.yaml
```

> **주의 (제어 모델 범위):** `kinematics_params_file`은 UR 드라이버가 publish하는 `robot_description`/TF에만 영향을 준다. RTC 컨트롤러의 Pinocchio 모델은 별도로 `config/<variant>/_base.yaml`의 `urdf:` 경로를 읽으므로, 제어 모델까지 calibration을 일치시키려면 후속 작업으로 URDF 생성 경로가 같은 YAML을 반영하도록 해야 한다 (본 범위 밖).
>
> `ur_calibration`은 `package.xml` `<exec_depend>`로 선언되어 `rosdep`(→ `ros-jazzy-ur-calibration`)이 설치한다.

---

### sim_ur5e_p1a.launch.py -- MuJoCo 시뮬레이션

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_viewer:=true
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_viewer:=false max_rtf:=10.0
```

**Launch 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `model_path` | `""` (YAML 사용) | MuJoCo scene 경로 오버라이드. `ur5e_p1b` 기본값은 `scene_with_table.xml` (= `scene.xml` + 상판 z=0.05 작업대); 맨바닥 씬이 필요하면 `model_path:=package://hand_description/robots/ur5e_p1b/mjcf/scene.xml` |
| `enable_viewer` | `""` (YAML 사용) | MuJoCo 뷰어 활성화 |
| `sync_timeout_ms` | `""` (YAML 사용) | sync 커맨드 타임아웃 (ms) |
| `max_rtf` | `""` (YAML 사용) | 최대 실시간 배율 (0.0 = 무제한) |
| `use_yaml_servo_gains` | `""` (YAML 사용) | YAML vs XML 서보 게인 사용 |
| `kp` | `""` (YAML 사용) | PD 게인 Kp 오버라이드 |
| `kd` | `""` (YAML 사용) | PD 게인 Kd 오버라이드 |
| `object_pool` † | `""` (YAML 사용) | `object_pool.enabled` 오버라이드. `false` 면 후보가 하나도 컴파일되지 않아 pool 블록이 없을 때와 `nq`/`nbody`/`ngeom` 이 같다 — 비용·회귀 비교의 대조군 |
| `object` † | `""` (YAML 사용) | 지정한 object 하나만 스폰 (`object_pool.directory` 의 서브디렉토리 이름). `object_pool.selection` 도 `fixed` 로 함께 고정한다 — 이름만 주고 selection 이 `random` 이면 다음 `o` 에서 다시 뽑히기 때문 |
| `object_seed` † | `""` (YAML 사용) | `object_pool.seed` 오버라이드. `0` = 매 실행 다름, 0 이 아니면 object/pose 순서가 재현 가능 |
| `use_cpu_affinity` | `true` | Tier 1 CPU 격리 + MuJoCo 핀닝 |
| `max_log_sessions` | `10` (= 그 변종의 노드 config YAML) | 세션 폴더 최대 보관 수. default 를 `_base.yaml` (iiwa7_leap 은 `sim.yaml`) 에서 읽고 값을 RT 노드 파라미터로도 넘긴다 — launch 와 노드가 같은 트리를 각자 정리하므로 |
| `initial_controller` | `""` (YAML 사용) | 시작 컨트롤러 이름 (예: `demo_wbc_controller`) 오버라이드 |
| `enable_mpc` | `""` (YAML 사용) | DemoWbcController의 `mpc.enabled` YAML 키를 launch 시점에 오버라이드. `true`/`false` 명시. `initial_controller:=demo_wbc_controller`와 함께 사용. 런타임 토글은 gains topic index 7로도 가능. |
| `mpc_engine` | `""` (YAML 사용) | DemoWbcController의 `mpc.engine` 오버라이드: `"mock"` = `MockMPCThread` placeholder, `"handler"` = `HandlerMPCThread` + `MPCFactory` + `GraspPhaseManager` (실제 Aligator ProxDDP solve, `mpc/phase_config.yaml`+`mpc/contact_light.yaml`+`mpc/contact_rich.yaml` 필요). 빈 값 = YAML 기본값 (현재 `demo_wbc_controller.yaml`은 `"handler"`) |
| `enable_tracing` | `false` | LTTng trace 캡처 (ros2_tracing). robot_ur5e_p1a.launch.py와 동일 시맨틱 |
| `trace_session_name` | `""` | LTTng 세션 이름. 빈 값 = `"trace"` |
| `trace_events_ust` | `""` | 콤마 구분 UST 이벤트. 빈 값 = ros2_tracing 기본 이벤트 |
| `trace_events_kernel` | `sched_switch,sched_waking,sched_wakeup,irq_handler_entry,irq_handler_exit` | 콤마 구분 커널 이벤트. 빈 값 = kernel tracing 비활성 |

빈 문자열(`""`) 기본값은 YAML 설정 파일의 값을 그대로 사용한다는 의미입니다. 명시적으로 값을 지정하면 YAML 값을 오버라이드합니다.

> **`sim_ur5e_p1b.launch.py`** (UR5e + proto_1b closed-chain hand) / **`sim_iiwa7_leap.launch.py`** (iiwa7 + LEAP Hand, hand sensor 스택 없음 — `grasp_controller_type: none`)는 위 인자 집합을 노출한다. iiwa7_leap은 `enable_mpc:=false`가 YAML 기본값(TSID self-hold 검증 후 수동 활성 권장).
>
> **공 발사는 `ur5e_p1b` 와 `iiwa7_leap` 두 프로필에 있다** (`ur5e_p1a` 에는 없다). launch 인자가 아니라
> 그 프로필의 `mujoco_simulator.yaml` 의 `projectile_ball:` 블록이 켠다. 두 프로필의 **공은 같지만
> 충돌 마스크·투척 조건은 서로 다르다** — 모델의 충돌 geom 구성과 손 위치에서 각각 유도한 값이라
> 한쪽을 다른 쪽에 복사하면 조용히 틀린다 (근거는 각 파일의 주석). 발사는 두 경로다:
> `/sim/launch_ball` (설정 분포에서 샘플링, 뷰어 `K` 와 같음) 과 `/sim/launch_ball_at`
> (호출자가 준 `(p0, v0, ω)`, 재현 가능 — D-14). 계약의 SSoT 는
> [rtc_msgs/srv/LaunchBall.srv](../rtc_msgs/srv/LaunchBall.srv) 와
> [rtc_mujoco_sim/README.md](../rtc_mujoco_sim/README.md) §Projectile Ball 이다.
>
> ball_perception 의 `sim_estimator_node` 를 이 씬에 붙일 때의 profile 은 `config/ur5e_p1b/ball_perception_sim_profile.json`
> (지평 1.0 s / 간격 0.05 s / 20 점, 측정 공분산 (5 mm)² 대각 — `projectile_ball.publish.position_noise_stddev_m` 과 짝,
> `max_future_skew_s` 0.1 — 공 토픽의 stamp 축이 sim 축이라 wall 을 위상 오차만큼 앞설 수 있다) 이다:
> `ros2 launch ball_perception_sim sim_estimator.launch.py profile_path:=$(ros2 pkg prefix integrated_bringup)/share/integrated_bringup/config/ur5e_p1b/ball_perception_sim_profile.json producer_revision:=<rtc-framework 커밋>`.
> `ball_perception_sim` 은 이 workspace 가 아니라 ball_perception 의 별도 colcon workspace 에 있으므로 그 `install/setup.bash` 를 추가로 source 해야 하고, `producer_revision` 은 필수 인자다 (출력 provenance).
> 값의 근거 (포구 제어기의 요구 사양, D-15) 는 [docs/dynamic_catching/IMPLEMENTATION_PLAN.md](../docs/dynamic_catching/IMPLEMENTATION_PLAN.md) §4.4 S3.6 결과이고,
> 공 토픽의 stamp 규약은 rtc_mujoco_sim README §Projectile Ball 의 stamp 항목이다.
>
> † 표시한 `object_pool` / `object` / `object_seed` 세 인자는 **`sim_ur5e_p1b.launch.py` 에만** 있다. `object_pool` 블록을 config 에 가진 프로필이 현재 `ur5e_p1b` 뿐이라, 다른 launch 에 인자만 달면 켜는 순간 `directory` 가 비어 Initialize 가 실패한다. 다른 프로필에 pool 을 쓰려면 그 프로필의 `mujoco_simulator.yaml` 에 블록을 먼저 넣는다 (키 전체의 SSoT 는 [rtc_mujoco_sim/config/mujoco_default.yaml](../rtc_mujoco_sim/config/mujoco_default.yaml)).

**Launch 순서:**

1. 세션 디렉토리 생성 — launch 실행 시점 (`launch_setup` 안). 워크스페이스 내 `logging_data/YYMMDD_HHMM/`, 보관 수는 `max_log_sessions`
2. `cpu_shield.sh on --sim` -- 경량 CPU 격리 (Tier 1)
3. MuJoCo 시뮬레이터 노드 launch (`rtc_mujoco_sim/mujoco_simulator_node`)
4. RT 컨트롤러 노드 launch (실행 파일 = ROS 노드 이름 = `integrated_rt_controller` — 정렬됨, params: `ur5e_p1a/_base.yaml` + `sim.yaml` + `mujoco_simulator.yaml`)
5. MuJoCo 시뮬레이터 코어 핀닝 (2초 지연, 8코어 이상일 때만)

**Lifecycle 순서:** 런치 시 mujoco_simulator → configure → activate 완료 후 integrated_rt_controller → configure → activate 순차 활성화.

`mujoco_simulator.yaml`은 `integrated_bringup/config/`에 위치하며 (UR5e 전용 robot_response 그룹/조인트/토픽), agnostic 기본값 + solver SSoT는 `rtc_mujoco_sim/config/solver_param.yaml`에서 먼저 로드된 후 본 yaml이 그 위에 오버레이됩니다.

**기동 자세**는 각 그룹의 `initial_qpos` (rad, `command_joint_names` 순서) 가 정하며, 이 값이 MJCF keyframe 보다 우선합니다 (우선순위·로그·함정은 [rtc_mujoco_sim README](../rtc_mujoco_sim/README.md#초기-자세-initial_qpos)). 세 프로필의 성격이 다릅니다:

- `ur5e_p1b` — 씬에 keyframe 이 **없어서** 이 키가 유일한 기동 자세 소스다. 값은 palm-down (`wrist_2 = 0`), 즉 손가락이 아래를 보는 대신 손바닥이 바닥을 보는 자세로, top-down 파지를 위한 것이다. `object_pool` 의 스폰 중심과 `scene_with_table.xml` 의 작업대 위치가 모두 이 자세의 손 기둥 `(-0.49, -0.23)` 에 맞춰져 있다 — 이 로봇의 base 는 `quat "0 0 0 -1"` 로 붙어 있어 작업 공간이 **−x** 쪽이므로, 다른 씬의 `+x` 좌표를 그대로 옮기면 손 뒤에 물체가 생긴다. 세 값은 대략만 맞으면 되고 (나머지는 궤적 생성이 흡수한다) 서로 기계적으로 연동되어 있지 않다.
- `ur5e_p1a` / `iiwa7_leap` — 씬 keyframe 과 **같은 값**을 명시적으로 들고 있다. 두 사본이 갈라지지 않도록 `test/test_shipped_initial_qpos.py` 가 joint 이름 기준으로 등가를 고정한다. **단 그 등가 레인은 자동으로 도는 곳이 없다** — 로컬 `colcon test` 는 pytest 를 `/usr/bin/python3` 로 돌려 `mujoco` 가 없고, 빌드·테스트 CI 는 없다 (2026-09-19 제거). 두 사본 중 하나를 고쳤으면 **손으로** 돌린다: `.venv/bin/python -m pytest src/rtc-framework/integrated_bringup/test/test_shipped_initial_qpos.py`. 같은 파일의 길이·타입 레인은 PyYAML 만 쓰므로 로컬 `colcon test` 에서 정상적으로 게이트 역할을 한다.

모든 원소에 **소수점을 찍으십시오** — 정수 리터럴이 하나라도 있으면 시퀀스 전체가 정수 배열로 추론되어 노드 생성 시점에 죽습니다. `test_shipped_sim_config` 가 이 타입을 rclcpp 로더로 고정합니다.

### Catching sim trials — 한 투척 = 한 S7 순환

S7.2 부터 포구 컨트롤러가 **스스로** 대기 자세로 간다. 무장되면 팔을 `planner.wait_pose` 로 관절공간 homing 하고 손을 q_pre 에 둔 채 기다린다 (IDLE → ARMED). 시행이 끝나면 다시 그 자세로 돌아간다 (RETREAT → ARMED). 그래서 S6 까지 쓰던 외부 정렬 (`demo_joint_controller` 전환 → 관절 목표 → 복귀, #537 결정 ④) 은 없앴다. 대기 자세 밖에서 시작하는 시행은 이제 드라이버가 가려 줄 일이 아니라 컨트롤러의 결함으로 드러나야 한다.

`catching_sim_trials` 는 떠 있는 sim 을 구동만 한다. 투척마다 다음을 한다.

1. FAULT 면 리셋 → 무장 (`catching.enable` 을 false → true 로 — 컨트롤러가 E-STOP·fault 에서 스스로 latch 를 내려도 파라미터는 true 로 남을 수 있어 같은 값의 재설정은 요청이 되지 않는다) → ARMED 대기 (컨트롤러의 homing). ARMED 순간의 측정 자세가 대기 자세에서 `--tol-q` / `--tol-qd` 를 넘으면 시행을 거부한다.
2. `/sim/launch_ball_at` 으로 투척하고 (spin ω 도 명시 — 기본 0) ground truth 와 모드 전이를 기록한다.
3. 순환이 닫히면 (RETREAT 뒤 ARMED) 끝낸다. IDLE·FAULT 로 가거나 `--record-s` (기본 12 s) 가 지나도 끝낸다. 시행의 판정은 RETREAT 진입 때 발행된 `outcome` 이다 (L7 §4.7).
4. 공을 리셋한다.

관절 이름·상태 토픽은 출하 프로파일에서 읽는다 (`--profile`, 기본 `ur5e_p1b`). **대기 자세·`T_freeze`·`T_arm`·`lead_enable`·`control.dt` 는 떠 있는 컨트롤러의 read-only 미러 파라미터에서 읽는다** (S8-A) — `sim_overlay:=` 가 이 값들을 바꿔도 설치된 YAML 은 그대로이기 때문이다. 미러가 없으면 (컨트롤러가 configure 에서 park 됨 — 그 로그가 값을 댄다) 시작하지 않는다. 미러 값은 `<out>/run_meta.json` 과 시행 기록마다 `controller_mirror` 로 남는다.

투척 계열은 `--dist` 가 고른다.

| `--dist` | 투척 | 용도 |
|---|---|---|
| `reference` (기본, S6-C 이후 불변) | 기준 투척 `--n-ref` 번 뒤에 섭동 투척 `--n-pert` 번 (속력 ×U(0.9, 1.1), 측방 U(−0.3, 0.3) m/s, `--seed`) | 회귀 세트. 같은 투척의 반복은 iid 표본이 아니므로 성공률 입력이 아니다 |
| `s35b` (`ur5e_p1b` 만) | S3.5b 90 % 상자 (거리 0.9–1.0 m · 릴리스 0.15–0.25 m · 방향 ±6° · 속력 4.65–4.85 m/s · 앙각 62–64°) 에서 `--n` 번 균등 iid, `--seed` 로 재현 | 동결 분포 (dynamic_catching plan §4.4 S8, D-S8-2). 발사 상태는 `rtc_tools.analysis.catchability_map` 의 격자 기하 그대로다. 표본의 축 값·seed·순번이 시행 기록에 남는다 |

**측정 lane (`sim_lanes:=true`).** sim 의 clock 위상 lane 과 공 접촉 truth lane 은 노드 파라미터라 기동 때만 읽힌다. `sim_lanes:=true` 는 둘을 켜고 `<session>/sim/{clock_lane,ball_contact_lane}.csv` 에 쓴다 — 시행의 lane 이 그 세션의 컨트롤러 CSV 옆에 남아 `catching_trials` (`rtc_tools`) 가 둘을 잇는다. 기본은 off (YAML 그대로).

```bash
# 1) sim (계획기는 enable_mpc:=true 가 필요하다 — mpc_off 면 활성화 거부)
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py enable_viewer:=false use_cpu_affinity:=false enable_mpc:=true sim_lanes:=true
# 2) 공 추정기 (위 sim_ur5e_p1b 절, ball_perception 의 workspace 를 추가 source)
# 3) 포구 컨트롤러로 전환
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: [demo_catching_controller], deactivate_controllers: [demo_joint_controller], strictness: 1, timeout: {sec: 3}}"
# 4) 투척 — <out>/truth_trial_NN.csv (공 ground truth) + trial_results.json (모드 전이·판정·정렬 오차)
ros2 run integrated_bringup catching_sim_trials <out> --n-ref 15 --n-pert 10
#    동결 분포에서 25 번 (seed 로 재현)
ros2 run integrated_bringup catching_sim_trials <out> --dist s35b --n 25 --seed 1
```

`trial_results.json` 의 `outcome` 이 시행 판정, `cycle_closed` 가 순환 완료 여부, `err_q_at_throw` 가 투척 순간의 정렬 오차다. `wall_t_relative_offset` 은 이 기록을 `catching_diag.csv` 의 시간축에 잇는다. 이제 homing 도 포구 컨트롤러가 하므로 모든 구간이 `catching_diag.csv` 에 행으로 남는다. `armed_at_throw` 는 투척 직전 컨트롤러가 발행한 무장 상태다. demo_controller_gui 의 Catching 패널은 같은 기준 (RETREAT 진입 때의 판정) 으로 이 패널이 본 시행 수를 판정별로 센다 (`this panel: attempts N: …`, 패널을 다시 띄우면 새로 센다).

---

## 로봇 vs 시뮬레이션 모드 비교

| 항목 | Robot 모드 | Sim 모드 |
|------|-----------|---------|
| 관절 상태 소스 | UR 드라이버 | MuJoCo 시뮬레이터 |
| 핸드 소스 | UDP (실제 하드웨어) | MuJoCo 에코백 (선택) |
| 커맨드 대상 | UR5e 로봇 | MuJoCo 물리 엔진 |
| CPU 격리 | Tier 1+2 (`--robot`) | Tier 1 (`--sim`) |
| DDS 핀닝 | UR 드라이버 + RT 컨트롤러 | MuJoCo 시뮬레이터 |
| 설정 | `ur5e_p1a/_base.yaml` + `robot.yaml` | `ur5e_p1a/_base.yaml` + `sim.yaml` + `mujoco_simulator.yaml` |

---

## GUI 도구

### demo_controller_gui

실시간 컨트롤러 선택, 게인 튜닝, 상태 모니터링 GUI입니다 (tkinter).

```bash
ros2 run integrated_bringup demo_controller_gui                      # 기본 ur5e_p1a
ros2 run integrated_bringup demo_controller_gui --robot ur5e_p1b     # ur5e + proto_1b hand
ros2 run integrated_bringup demo_controller_gui --robot iiwa7_leap   # iiwa7 + LEAP
ros2 run integrated_bringup demo_controller_gui --p1b                # 별칭 (= --robot ur5e_p1b)
ros2 run integrated_bringup demo_controller_gui --iiwa               # 별칭 (= --robot iiwa7_leap)
```

`--robot <key>` 는 arm/hand joint 스키마(이름·DoF·finger group)와 TCP tf frame 을
선택합니다. `key` 는 `config/<key>/` bringup 디렉토리명과 동일 (`ur5e_p1a` |
`ur5e_p1b` | `iiwa7_leap`); `--ur5e` / `--p1b` / `--iiwa` 별칭도 동일 dest 로 매핑됩니다. 잘못된 key 는
즉시 에러 후 종료합니다. 프로파일 정의는 `demo_gui/discovery.py` 의
`ROBOT_PROFILES` 레지스트리 — 새 로봇은 여기에 한 항목 추가. 실행 후 컨트롤러가
publish 하는 joint span 과 프로파일이 어긋나면 `/rosout` 에 one-shot WARN 이
나오며, 맞는 `--robot` 값으로 재실행하면 사라집니다.

#### 모듈 구조

GUI 는 `integrated_bringup/integrated_bringup/demo_gui/` 패키지로 구성됩니다 (`scripts/demo_controller_gui.py` 는 8-line shim):

| 모듈 | 역할 |
|---|---|
| `demo_gui/app.py` | `DemoControllerGUI` Tk 클래스 + main() — 위젯 빌드 / refresh / ROS callback / 핸들러 |
| `demo_gui/config.py` | gain 스키마 (`GAIN_DEFS`, `GAIN_PARAM_DISPATCH`), 위젯 레이아웃, FSM phase 라벨 표, 캘리브레이션 항목 — robot-agnostic GUI 표 |
| `demo_gui/discovery.py` | `RobotShape` (arm/hand DoF·finger group) + `RobotProfile` / `ROBOT_PROFILES` — `--robot` 가 선택하는 정적 로봇 프로파일 (joint 스키마 + TCP frame + 그 bringup 이 전환 가능한 컨트롤러 집합) |
| `demo_gui/catalog.py` | `ControllerCatalog` — `/rtc_cm/list_controllers` 비동기 폴러 (5 s 주기). 라디오 버튼 / preset combo / 라벨이 모두 이 catalog 결과에서 옴. |
| `demo_gui/pull.py` | Pull Force Estimate 패널의 상태기계 — `PullSnapshot` (immutable) / `PullPeakHold` / `badge_state` / `build_render`. Tk·rclpy 비의존이라 `test/test_demo_gui_pull.py` 가 디스플레이 없이 검증. |
| `demo_gui/task_frame.py` | `TaskFrameSelector` — active controller 가 **실제로 제어 중인** task frame 선택. `virtual_tcp_actual` 을 한 번이라도 관측하면 즉시 latch, 없이 fallback (`RobotProfile.tcp_child`) 만 `settle_msgs` 건이면 fallback latch. 컨트롤러 이름 하드코딩 없이 **availability** 로만 판정하며, settle window 는 closed-chain hand FK walk-in 동안 tool0 만 발행되는 창을 넘기기 위한 것이다. Tk·rclpy 비의존 (`test/test_demo_gui_task_frame.py`). |
| `demo_gui/preset_toggle.py` | preset 단축키의 순수 로직 — `PresetToggleState` (close 우선, **dispatch 성공 시에만** close↔open 전진, key-repeat 억제용 arm/rearm), keysym 정규화, preset 쌍 해석, 설정 JSON 의 검증·atomic 저장. Tk·rclpy 비의존 (`test/test_demo_gui_preset_toggle.py`). |

#### 동적 controller 발견 (`/rtc_cm/list_controllers`)

GUI 시작 시:

1. 선택된 프로파일의 *switchable* 키 집합을 오프라인 fallback 으로 라디오에 표시. 상태 라벨에 `(controllers offline)` 접미사.
2. CM 의 `/rtc_cm/list_controllers` 가 응답하면 catalog 가 수신 → Tk 스레드로 marshalling → 라디오 / preset combo / 상태 라벨이 *live* 데이터로 재구성. 5 s 마다 재조회 (controller hot-swap 자동 감지).
3. 응답된 controller 중 그 switchable 집합에 있는 것만 라디오에 노출. 나머지는 catalog 에 보존되지만 GUI 에서 선택 불가.

**switchable ≠ tunable.** 라디오 집합은 `GAIN_DEFS` 키 (= GUI 에 게인 패널이 있는 컨트롤러) 에 `RobotProfile.extra_switchable_controllers` 를 더한 것입니다 (`RobotProfile.switchable_controllers()`). 두 축이 갈라진 이유는 `demo_inference_controller` 입니다 — 전환은 되지만 ROS 파라미터를 하나도 선언하지 않아 튜닝할 게 없습니다.

필터가 **catalog 가 아니라 프로파일**에서 오는 것이 핵심입니다: CM 은 등록된 컨트롤러를 로봇과 무관하게 인스턴스화하고, 로봇마다 다른 것은 *어느 컨트롤러가 config YAML 을 갖는가* 이며, 그래서 그 사실은 `discovery.py` 의 프로파일에 있습니다. `demo_inference_controller` 는 `config/ur5e_p1b/controllers/` 에만 YAML 이 있으므로 **`--robot ur5e_p1b` 에서만** 라디오에 뜹니다 (`test_robot_profiles.py` 가 설치된 config 트리와 양방향 대조).

**예외 하나 — `/rtc_cm/list_controllers` 응답이 세 프로파일에서 동일하지는 않습니다.** `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` 로 등록된 컨트롤러(현재 `demo_inference_controller` 뿐)는 그 프로파일에 YAML 이 없으면 **인스턴스화되지 않고**, 따라서 목록에도 없습니다. 기본값으로 돌 수 없는 컨트롤러이기 때문입니다 — 정책 파일 경로도 IO 스키마도 없이 기본값으로 도는 정책 컨트롤러는 의미가 없고, 그렇다고 bring-up 을 거부하면 그 로봇의 **나머지 네 컨트롤러까지** 못 뜹니다 (실제로 `ur5e_p1a`·`iiwa7_leap` 이 그 상태였습니다). 나머지 컨트롤러는 종전대로 YAML 이 없으면 내장 기본값으로 돕니다. 게이트는 `test_registered_controllers_have_shipped_config.cpp`.

preset combo 는 라디오 집합에서 **목표를 받지 않는 컨트롤러를 뺀** 것입니다 — preset 은 robot target 을 저장하는데, `joint_goal` 을 구독하지 않는 컨트롤러 앞으로 저장된 preset 은 재생할 곳이 없습니다.

라벨은 controller config_key 를 prettify (`demo_wbc_controller` → `Demo Wbc Controller`) 한 결과를 사용합니다.

#### Variable-DOF 동작 (Phase 1)

`RobotShape` 는 시작 시 UR5e + assm_v1 hand 기본값 (6 arm, 10 hand) 으로 초기화되어 위젯이 즉시 빌드됩니다. CM이 발행하는 `/rtc_cm/{ur5e,hand}/joint_states` (Phase 4 이후 sensor_msgs/JointState) 의 `name` 필드가 GUI 의 RobotShape 와 다르면 *1회 WARN 로그* 후 사용자에게 GUI 재시작을 안내합니다 (현재 sprint 에서는 widget 동적 rebuild 미지원 — option (a)). 다른 robot/hand 에서는 startup 시 `default_ur5e_assm()` 대신 적절한 default factory 를 추가하면 동작합니다.

#### 컨트롤러별 패널

| 컨트롤러 | 게인 그룹 | 파라미터 (R/W) | RO 캡 |
|---|---|---|---|
| `demo_joint_controller` | Arm/Hand Trajectory | `robot_trajectory_speed`, `hand_trajectory_speed` | `robot_max_traj_velocity`, `hand_max_traj_velocity` |
| `demo_joint_controller` | Grasp Detection | `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips` | — |
| `demo_task_controller` | CLIK Gains | `kp_translation` (×3), `kp_rotation` (×3), `singularity_threshold`, `max_damping`, `null_kp`, `enable_null_space`, `control_6dof` | — |
| `demo_task_controller` | Arm/Hand Trajectory | `trajectory_speed`, `trajectory_angular_speed`, `hand_trajectory_speed` | `max_traj_velocity`, `max_traj_angular_velocity`, `hand_max_traj_velocity` |
| `demo_task_controller` | Grasp Detection | (joint 와 동일) | — |
| `demo_wbc_controller` | Arm/Hand Trajectory | `arm_trajectory_speed`, `hand_trajectory_speed` | `arm_max_traj_velocity`, `hand_max_traj_velocity` |
| `demo_wbc_controller` | Hand τ_ff | `hand_tauff_enable` (bool), `hand_tauff_gravity_gain`, `hand_tauff_closure_bias`, `hand_tauff_max`, `hand_tauff_source` (combobox: `gravity_comp` \| `tsid_tau`) | — |
| `demo_wbc_controller` | TSID Weights | `se3_weight`, `force_weight`, `posture_weight` | — |
| `demo_wbc_controller` | Grasp Detection | (joint 와 동일 — layer-d 에서 추가, capability-aware) | — |
| `demo_wbc_controller` | MPC | `mpc_enable` (bool), `riccati_gain_scale` | — |

WBC 패널의 `mpc_enable` 토글은 controller 측에서 YAML 의 구조적 `mpc.enabled` flag 와 AND 됩니다. YAML 에서 `mpc.enabled: false` 로 설정된 경우 GUI toggle 은 no-op 입니다 (MPC 스레드가 spawn 되지 않음). 자세한 의미는 `config/ur5e_p1a/controllers/demo_wbc_controller.yaml` 의 `mpc:` 블록 주석 참조.

**Target 패널 (관절 vs task):** `demo_joint_controller` 는 관절 목표만, `demo_task_controller` 는 task-space (EE SE3) 목표만 입력 패널이 활성화됩니다. `demo_wbc_controller` 는 **둘 다 활성화** — 암 posture (nullspace reference) 와 commanded EE SE3 jog 를 독립적으로 받기 때문 (`demo_gui/config.py` `DUAL_TARGET_SPACE`). WBC 에서 `Send Command` 는 두 `RobotTarget` (goal_type `joint` + `task`) 을 모두 publish 하며, controller `DeliverTargetMessage` 가 goal_type 별로 라우팅합니다. EE SE3 패널 값은 TF (`virtual_tcp`/`ee_link`) 가 wiring 되어 있어야 current pose 로 seed 됩니다.

**`demo_inference_controller` — 외부 명령 경로가 없는 컨트롤러:** 이 컨트롤러는 라디오에서 선택하고 `Switch Controller` 로 활성화할 수 있지만 (`--robot ur5e_p1b` 한정), **어떤 authoring 위젯도 동작하지 않습니다**. 정책이 팔과 손 명령을 모두 소유하므로 (a) `CreateOwnedTopics` 를 호출하지 않아 `joint_goal` **구독 자체가 없고** — YAML 의 `topics:` 블록은 device 그룹 *순서*를 세우는 용도입니다 — (b) `grasp_command` srv 를 선언하지 않으며 (c) ROS 파라미터가 0개라 게인 패널이 비어 있습니다. 그래서 GUI 는 관절·task 목표 패널과 `▶ Grasp` / `■ Release` 를 **비활성화하고 사유를 표시**합니다 (`demo_gui/config.py` `NO_EXTERNAL_COMMAND_CONTROLLERS`). 구독자가 없는 토픽으로의 publish 는 조용히 성공하므로, 버튼을 살려두면 "Sent hand cmd" 로그만 남고 로봇은 안 움직이는 상태와 구별되지 않습니다 — `Send Command` 와 `Send Preset` 의 hand 절반도 같은 이유로 차단됩니다. 관절 상태 readout 은 CM 소유 토픽(`/rtc_cm/<group>/joint_states`)이라 정상 동작하고, 진단(hold 사유·reach gate)은 토픽이 아니라 세션 CSV (`inference_diag.csv`) 로만 나옵니다.

**Task frame 게이트:** task 목표는 active controller 의 **제어 frame 이 확정된 뒤에만** 입력·발행할 수 있습니다. 컨트롤러 전환 직후에는 `TaskFrameSelector` 가 미확정 상태라 task target entry / step 버튼이 `disabled` 이고, EE pose 표시는 `—` 입니다 (이전 컨트롤러의 pose 를 라이브처럼 보여주지 않기 위해). frame 이 확정되면 그 시점의 실제 pose 로 **1회** seed 되고 패널이 활성화됩니다 — 전환 시점에 seed 하면 아직 이전 컨트롤러의 pose 라 "표시된 pose 를 그대로 target 으로 보냈는데 로봇이 움직이는" 결함이 됩니다. `Send Preset` 의 자동 task publish 와 preset 저장도 같은 게이트를 통과합니다. 이 게이트는 컨트롤러 측 계약의 나머지 절반이다 — `ApplyPendingTarget` 이 외부 목표를 컨트롤러의 *의도된* frame 으로 태깅할 수 있는 근거가 "미확정 창에서는 authoring 자체가 불가능하다" 이기 때문.

#### Preset 단축키 (close ↔ open 토글)

Preset 패널의 `Toggle Close` / `Open` 콤보에 고른 두 preset 을 **키 하나로 번갈아** 보냅니다 (기본 `Space`, `Set Key` 로 변경 — 캡처 창에서 `Esc` 는 취소). 선택은 preset 파일과 같은 디렉토리의 `demo_gui_settings_<hand_group>.json` 에 저장됩니다.

- **atomic dispatch**: preset 의 `controller` 가 active 가 아니면 먼저 전환합니다. 전환 여부와 무관하게 보내기 전에 최대 2 s 동안 세 가지를 기다립니다 — goal publisher 가 **그 컨트롤러 네임스페이스로 다시 만들어졌는가** (`active_controller_name` 은 rewire 보다 먼저 바뀌므로 이름만으로는 옛 publisher 에 보낼 수 있음), 그 publisher 에 **매칭된 subscriber 가 있는가** (goal publisher 는 depth 1 volatile 이라 discovery 전에 보낸 목표는 버려지는데 토글은 성공으로 셈), (task 목표라면) task frame 이 확정됐는가. E-STOP·DoF 불일치·미확정 task frame 등 사전 조건이 하나라도 실패하면 **아무것도 발행하지 않고** 사유를 로그에 남깁니다.
- **전진 규칙**: 발행에 성공했을 때만 다음 역할(close→open)로 넘어갑니다 — 거부된 close 뒤에 누르면 다시 close 입니다.
- **입력 억제**: 텍스트 입력 위젯에 포커스가 있으면 무시하고 (그 외 위젯에서는 단축키가 이깁니다 — ttk 버튼은 클릭하면 포커스를 가져가므로, 포커스된 버튼에 `Space` 를 넘기면 마지막에 누른 버튼(E-STOP·Switch 포함)이 다시 눌리고 단축키는 조용히 먹통이 됩니다), 키를 누르고 있는 동안의 auto-repeat 는 한 번으로 셉니다. 앞선 토글이 끝나기 전의 입력도 무시합니다.
- **실패 복구**: 토글 단계 중 예외가 나면 단축키 잠금을 풀고 `Preset toggle (...) aborted` 를 error 로그에 남깁니다 — 재시작 없이 다시 누를 수 있습니다.
- 두 콤보에 같은 preset 을 고르면 패널에 경고가 표시됩니다 (토글이 같은 명령만 반복).

#### Grasp/Release 버튼 동작

- **`demo_inference_controller`**: 버튼 **항상 비활성**. `grasp_command` srv 자체가 없어 누르면 거부 메시지조차 돌아오지 않으므로, 파라미터 미조회 시의 fail-open (아래) 과 달리 UNKNOWN 에서도 열리지 않습니다 — 그 fail-open 은 "컨트롤러가 받아줄 명령을 GUI 가 막지 않는다" 는 규칙인데, 여기서는 막을 대상이 없습니다.
- **`demo_joint_controller` / `demo_task_controller`**: 활성 모드가 `force_pi` 일 때만 동작. `"contact_stop"` / `"none"` 모드에서는 srv 가 **사유와 함께 거부** (`ok=false` + `message`, `GraspCommandRejectReason` 가 SSoT). GUI 는 두 컨트롤러가 노출하는 `grasp_controller_type` 파라미터를 읽어 버튼 옆에 현재 모드를 표시하고, force_pi 가 아니면 버튼을 비활성화합니다 (파라미터 미조회 시에는 fail-open — 위 "Force-PI Grasp/Release 버튼 (GUI)" 절). 그 파라미터는 런타임 설정 가능하며 (quiet gate), 같은 절에 조건이 있습니다.
- **`demo_wbc_controller`**: `grasp_controller_type` 무관 — WBC 는 자체 6-state FSM (slots 2 & 5 reserved) 으로 GraspCommand 를 직접 처리 (lifecycle.cpp 의 `grasp_command_srv_`). GRASP 명령은 `kApproach` 진입, RELEASE 는 어떤 비-terminal phase 에서도 `kRelease` 로 즉시 preempt (Approach/Closure/Hold 중 GUI 로 RELEASE 누르면 즉시 반응). phase 표시기가 WbcPhase enum 라벨로 갱신됩니다.

phase 표시기는 active controller 가 force_pi grasp publisher 인지 WBC publisher 인지에 따라 `GRASP_PHASE_NAMES` (6 상태) 또는 `WBC_PHASE_NAMES` (8 슬롯, 6 reachable — slot 2 PRE-GRASP / slot 5 RETREAT 는 deprecated reserved) 라벨 표를 자동 선택합니다 — 두 publisher 가 GUI 에 동시에 구독되지만 active 한 쪽만 발행하므로 자동 분기.

#### Pull Force Estimate 패널

`GraspState`/`WbcState` 의 `pull` (`PullEstimate`) 서브메시지를 표시합니다. 두 부모 메시지가 같은 서브메시지를 embed 하므로 콜백 하나가 양쪽을 커버하며, joint/task 는 `grasp_state` 만·WBC 는 `wbc_state` 만 발행하고 두 구독 모두 *active* 컨트롤러 네임스페이스에 바인딩되므로 동시에 살아있는 source 는 항상 하나입니다.

**Freshness — 도착 시각이 데이터에 실린다.** 콜백은 필드를 하나씩 쓰지 않고 immutable `PullSnapshot` 을 통째로 만들어 한 번에 재바인딩하며, 렌더는 그것을 지역변수로 1회만 읽습니다. 따라서 서로 다른 tick 의 필드가 섞인 화면은 구조적으로 불가능합니다. 스냅샷은 **수신측** 시각을 함께 싣고 `PULL_STALE_AFTER_S = 0.5 s` (GUI 의 5 Hz 재도색 주기 기준, `control_rate` 아님) 를 넘으면 stale 로 판정합니다 — 근거는 [demo_gui/pull.py](integrated_bringup/demo_gui/pull.py).

**배지 우선순위** (`badge_state`):

| 순위 | 조건 | 배지 | 부가 설명 |
|---|---|---|---|
| 1 | E-STOP | `UNAVAILABLE` | `E-STOP active` |
| 2 | stale · 미수신 · rewire 직후 | `STALE` / `UNAVAILABLE` | 마지막 갱신 경과 시간 |
| 3 | fresh + estimator 미wiring | `UNAVAILABLE` | `estimator disabled` |
| 4 | fresh + `slip_risk` | `SLIP RISK` | — |
| 5 | fresh + `valid` | `NO SLIP RISK` | — |
| 6 | fresh + `!valid` | `UNKNOWN` | `invalid_reason` 라벨 (not initialized / degenerate normal / required contact lost / too few contacts) |

3순위가 별도 상태인 이유: `pull_estimator.enabled: false` 이거나 FK-backed tip link 가 하나도 안 풀리면 default-constructed POD (전부 0, `valid=false`, `invalid_reason=INVALID_NONE`) 가 fresh 메시지로 도착합니다. 이 조합은 `PullEstimate.msg` 계약 ("INVALID_NONE **iff** `valid`") 밖이라 "이 메시지 뒤에 estimator 가 없다" 의 신뢰할 수 있는 표지이며, 처리하지 않으면 배지가 **"UNKNOWN: valid"** 라는 자기모순 문구를 띄웁니다 (근거: [demo_gui/pull.py](integrated_bringup/demo_gui/pull.py)). 문구는 컨트롤러 로그 `[pull_estimator] disabled — ...` 와 맞춰 두었으므로 조작자가 원인을 grep 할 수 있습니다.

fresh tick 안에서 slip 이 validity 를 앞서는 것은 estimator 가 `slip_risk`/`friction_utilization` 을 `valid` 게이트 **밖에서** 평가하기 때문입니다 (P-7) — required role 이 빠져 pull 벡터가 무효여도 아직 잡고 있는 tip 은 미끄러질 수 있습니다. E-STOP 도 매 tick 행을 발행하지만 `StageEstopPullTick` 이 all-invalid 입력 + zero normal 로 돌리므로 `INVALID_DEGENERATE_NORMAL` 이 실려 옵니다 — 이를 "UNKNOWN: degenerate normal" 로 읽으면 일어나지도 않은 기하 실패를 지목하게 되므로 배지를 `UNAVAILABLE` 로 덮습니다 (같은 경로가 contact 을 전부 지우므로 `slip_risk` 도 false 라 숨겨지는 정보는 없습니다).

**두 개의 게이트 — 값 렌더 (`build_render`).** `trusted` (배지가 STALE/UNAVAILABLE 이 아님) 가 실패하면 모든 값이 `--` 로 비워집니다. 발행이 끊긴 source 나 컨트롤러 전환 이전의 숫자는 빈 칸보다 나쁘기 때문입니다. `trusted` 는 배지 옆에서 따로 계산하지 않고 **배지에서 유도**하므로 STALE 헤더 아래에 살아있어 보이는 숫자가 뜰 수 없습니다.

| 그룹 | 게이트 | 필드 |
|---|---|---|
| Pull vector | `trusted && valid` | `magnitude` · `directional` · `leakage` · `force[3]` · `force_inplane[2]` · `basis` · `basis_x[3]` (단 `valid` 자신은 `trusted` 만 — 실패 원인을 표시해야 하므로) |
| Contacts (partial-contact 진단) | `trusted` | `friction_utilization` · `contact_mask` · `touch_mask` · `any_saturated` · `baseline_applied` · `plane_normal[3]` |
| Source | 무조건 | publisher 라벨 · age · `header.frame_id` |

`plane_normal` 만 진단 그룹에 있는 것은 `PullEstimate.msg` 계약 때문입니다 — tick 이 normal 은 가졌는데 basis 는 없을 수 있으므로 (평면은 멀쩡한데 required tip 이 빠진 경우) `valid` 로 게이팅하면 wire 가 유효하다고 말하는 값을 숨기게 됩니다. 반대로 `basis_x` 는 무효 tick 에서 항상 0 이라 `valid` 게이트가 맞습니다. `basis` 는 축 이름이 아니라 **규칙**(`reference`/`carry`/`seed`)을 표시합니다 — `force_inplane[0]` 이 설정된 reference 방향 성분인 것은 `BASIS_REFERENCE` 일 때뿐이기 때문입니다 (P-11). mask 는 LSB-first (`#.#.` = contact 0·2) 로 estimator 의 tip role 순서를 따릅니다. 슬롯 수는 GUI 의 fingertip roster (`FINGERTIP_NAMES`, 4) 이고 mask 는 uint8 · estimator 의 tip 목록 크기이므로 — 현재 모든 로봇 config 에서 둘 다 4 로 일치 — 그 밖의 비트는 조용히 버리지 않고 말미 `+` 로 표시합니다 (`#...+`).

**Peak-hold.** wire 는 `control_rate` (수백 Hz), 패널은 5 Hz 이므로 렌더 시점의 최신 스냅샷만 보면 순간적인 slip spike 를 놓칠 수 있습니다. 프레임 사이 도착한 모든 스냅샷을 `PullPeakHold` 에 접어 넣고 렌더가 프레임마다 소비·클리어하므로, `friction_utilization` 은 그 프레임의 최댓값, `slip_risk` 는 OR 로 표시됩니다 — spike 는 한 프레임 보였다 사라집니다. 메시지가 하나도 안 온 프레임(`samples == 0`)은 스냅샷을 그대로 두어 마지막 값을 지우지 않습니다. 이 accumulator 는 executor 스레드와 Tk 스레드가 함께 쓰는 mutable 상태라 `threading.Lock` 으로 감쌉니다 — 구현 근거는 [demo_gui/pull.py](integrated_bringup/demo_gui/pull.py) `PullPeakHold`.

**Rewire 리셋.** `_rewire_owned_topics` 는 핸들을 재생성하기 전에 `_reset_controller_sourced_state()` 로 컨트롤러 소유 상태를 버립니다 (P-10). 남겨두면 이전 컨트롤러의 값이 새 컨트롤러의 라이브 데이터처럼 화면에 남습니다 (새 컨트롤러가 estimator 를 안 쓰면 그 자리를 덮어쓸 값 자체가 오지 않습니다). peak-hold 도 함께 리셋되며 (spike 는 그것을 만든 source 소유), phase 라벨 표를 고르는 `_wbc_active` 도 초기화됩니다 (stale `True` 는 다음 컨트롤러의 phase 를 잘못된 enum 으로 디코딩).

#### 기타 기능

- 관절/태스크 공간 타겟 설정 (joint controller 는 6 axis, task controller 는 X/Y/Z + Roll/Pitch/Yaw)
- 핸드 모터 슬라이더 (Thumb / Index / Middle / Ring — finger 그루핑은 motor name prefix 로 자동 추론)
- E-STOP 상태 (`/system/estop_status`), 실시간 TCP/관절 위치 표시
- 핸드 자세 프리셋 저장/로드 (JSON)
- 센서 캘리브레이션 (`/<hand_group>/calibration/command` — `--robot` 프로파일의 hand group 으로 파생: p1a→`/p1a/`, p1b→`/p1b/`)

### motion_editor_gui

모션 시퀀스 편집기입니다 (PyQt5, Catppuccin Mocha 테마).

```bash
ros2 run integrated_bringup motion_editor_gui
```

- 다중 탭 인터페이스 (모션 파일 동시 편집)
- 포즈 테이블 (이름, UR5e/핸드 프리뷰, 궤적 시간, 대기 시간)
- JSON 모션 파일 로드/저장
- 포즈 삽입/삭제/복사/붙여넣기
- 선택 모션 재생

---

## 전역 설정 (`ur5e_p1a/_base.yaml` + `robot.yaml`)

> 아래는 `_base.yaml`(mode-agnostic) 과 `robot.yaml`(real-HW delta) 을 overlay 한 **병합 뷰**입니다.
> `control_rate`·`device_timeout_names`·`enable_logging`·`urdf:`·`devices` roster/limits/sensor_layout 카운트는 `_base.yaml` 에,
> `initial_controller`·`init_timeout_sec`·`enable_estop`·`device_timeout_values`·`backend`·`joint_command_names`·`motor_state_names`·`has_native_*` 는 `robot.yaml` (sim 은 `sim.yaml`) 에 있습니다.

```yaml
/**:
  ros__parameters:
    control_rate: 500.0             # _base
    initial_controller: "demo_wbc_controller"  # delta (robot.yaml / sim.yaml)
    init_timeout_sec: 30.0          # delta — 하드웨어 초기화 타임아웃 (sec)
    enable_estop: true              # delta
    device_timeout_names: ["ur5e", "hand"]
    device_timeout_values: [1000.0, 1000.0]  # ms
    enable_logging: true
    enable_timing_log: true
    enable_device_log: true
    log_dir: ""
    max_log_sessions: 10

    # System URDF + model topology (shared by all controllers)
    urdf:
      package: "robot_descriptions"
      path: "robots/ur5e_assm_v1/urdf/ur5e_with_hand.urdf"
      root_joint_type: "fixed"
      sub_models:
        ur5e:                     # map 키 = devices.ur5e 그룹명
          root_link: "base"
          tip_link: "tool0"
      # tree_models:              # hand FK 필요 시 활성화
      #   hand:                   # map 키 = devices.hand 그룹명
      #     root_link: "hand_base_link"
      #     tip_links: [thumb_tip_link, index_tip_link, ...]
      passive_joints:
        - "thumb_cmc_aa"
        - "thumb_cmc_fe"
        # ... (10개 hand joints)

    devices:
      ur5e:
        backend:                                # Phase 4 SSoT
          type: "ur_driver_native"
          state_topic:   "/joint_states"
          command_topic: "/forward_position_controller/commands"
        joint_state_names: [shoulder_pan_joint, ..., wrist_3_joint]  # 6
        joint_command_names: [shoulder_pan_joint, ..., wrist_3_joint]  # ur_driver_native packing order
        # root_link/tip_link: urdf.sub_models에서 name="ur5e"로 자동 해석
        joint_limits:
          max_velocity: [2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
          max_acceleration: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
          max_torque: [150, 150, 150, 28, 28, 28]
          position_lower: [-6.28, -6.28, -3.14, -6.28, -6.28, -6.28]
          position_upper: [6.28, 6.28, 3.14, 6.28, 6.28, 6.28]
      p1a:
        backend:                                # Phase 4 SSoT
          type: "udp_hand_native"
          state_topic:   "/p1a/joint_states"
          command_topic: "/p1a/joint_command"
          motor_topic:   "/p1a/motor_states"
          sensor_topic:  "/p1a/sensor_states"
        joint_state_names: [thumb_cmc_aa, ..., ring_mcp_fe]  # 10
        motor_state_names: [motor_1, ..., motor_10]          # 10
        sensor_names: [thumb, index, middle, ring]            # 4
        joint_limits:
          max_velocity: [1.0, ..., 1.0]
          position_lower: [0.0, ..., 0.0]
          position_upper: [1.57, ..., 1.57]
```

**시뮬레이션 delta (`ur5e_p1a/sim.yaml`) 차이점 (동일 `_base.yaml` 위에 overlay):**
- `init_timeout_sec: 0.0` (비활성화), `enable_estop: false`
- `use_sim_time_sync: true`, `sim_sync_timeout_sec: 5.0`
- `device_timeout_values: [10000.0, 10000.0]` (시작 시 여유)
- `joint_command_names` 생략 (`joint_state_names`과 동일하므로)
- 핸드에 `motor_state_names` 없음 (MuJoCo가 직접 position 제공)
- URDF sub_model 조작 프레임: `root_link: "base_link"`, `tip_link: "flange"` (MuJoCo URDF 기준)
- 양 그룹의 `backend.type: "mujoco_native"`; `state_topic` = `/<group>/joint_states`, `command_topic` = `/<group>/joint_command`; sim 모드에서는 `motor_topic`/`sensor_topic` 미설정 (MuJoCo는 단일 lane만 노출)

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `ament_cmake_python` | Python 패키지(`demo_gui`) 빌드 |
| `ament_index_cpp` | 패키지 경로 해석 (bridge YAML 로드) |
| `rclcpp` | ROS2 클라이언트 |
| `rclcpp_lifecycle` | LifecycleNode 기반 컨트롤러 |
| `rtc_controller_interface` | 컨트롤러 추상 인터페이스 |
| `rtc_controllers` | 내장 컨트롤러 |
| `rtc_controller_manager` | RT 제어 루프 (library — `RtControllerMain()` 제공, exec는 본 패키지가 소유) |
| `rtc_base` | 타입, 스레딩 |
| `rtc_msgs` | 커스텀 메시지 |
| `rtc_math` | SE(3) Lie-group 연산 (task-space error) |
| `rtc_urdf_bridge` | URDF→Pinocchio 모델 빌더 + RT-safe handle |
| `rtc_tsid` | TSID QP 프레임워크 (DemoWbcController) |
| `rtc_mpc` | MPC↔RT 인터페이스 (TripleBuffer + Hermite + Riccati + MPCThread) |
| `pinocchio` | 기구학 (FK, Jacobian) — bridge가 transitively 제공 |
| `proxsuite` | QP 솔버 (rtc_tsid가 transitively 요구) |
| `yaml-cpp` | YAML 파싱 |
| `sensor_msgs` | JointState |
| `geometry_msgs` | TF/pose 메시지 |
| `std_msgs` | 표준 메시지 |
| `tf2_msgs` | `robot_transforms` 토픽 (`tf2_msgs/TFMessage`) |
| `rclpy` | Python GUI (exec) |
| `tf2_ros` | GUI의 TF lookup (exec) |
| `repo_scripts` | CPU 격리 스크립트 (exec) |
| `rtc_tools` | `session_dir` 로깅 경로 유틸 (exec) |
| `robot_descriptions` | URDF/MJCF 모델 (data hub, exec) |
| `udp_hand_driver` | 핸드 드라이버 (exec) |
| `ur_robot_driver` | UR5e 실로봇 드라이버 launch (exec) |
| `ur_calibration` | factory calibration YAML 추출 (`kinematics_params_file`, exec) |
| `PyQt5` | 모션 에디터 GUI (exec, venv 책임 — requirements.lock) |

---

## 빌드

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select integrated_bringup
source install/setup.bash
```

**빌드 산출물:**
- 정적 라이브러리: `libintegrated_bringup_demo_controllers.a` (`--whole-archive` 링크)
- 실행 파일: `integrated_rt_controller`
- Python 스크립트: `demo_controller_gui`, `motion_editor_gui`

---

## 의존성 그래프 내 위치

```
rtc_controller_manager + rtc_controllers + repo_scripts + robot_descriptions
    |                                                         |
    |   rtc_urdf_bridge (URDF→Pinocchio 모델)           |
    |       |                                                 |
integrated_bringup  <- 멀티 로봇 통합 패키지 (ur5e_p1a / ur5e_p1b / iiwa7_leap) ─┘
    |
    ├── robot_ur5e_p1a.launch.py / robot_ur5e_p1b.launch.py           -> UR 드라이버 + RT 컨트롤러 + CPU 격리
    ├── sim_ur5e_p1a.launch.py / sim_ur5e_p1b.launch.py / sim_iiwa7_leap.launch.py -> MuJoCo + RT 컨트롤러 + CPU 격리
    ├── DemoJointController (index 4)  ─┐
    ├── DemoTaskController (index 5)   ─┤── RtModelHandle (arm sub-model) / TSID+MPC (WBC)
    ├── DemoWbcController (index 6)    ─┘
    ├── demo_controller_gui
    └── motion_editor_gui

    ur5e_p1a/_base.yaml (urdf: sub_models)  <- 시스템 URDF + 모델 토폴로지 (robot 당 config/<key>/_base.yaml)
```

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
