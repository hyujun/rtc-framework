# 단계 W — `rtc-framework` workspace 종합 분석 (L0 착수 전 필수)

- 문서 버전: v0.5 (2026-09-19)
- 상태: **단계 W 완료 (2026-09-19)** — 코드 대조 (agent 조사 + main session spot check). 결정은 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (D-x·A-x) 가 SSoT 이고, 본 문서는 확인 기록만 갖는다. 런타임·사용자 확인이 필요한 항목은 "미확인 — <단계>" 로 남겼다
- 선행: 없음. **L0 포함 모든 layer 구현의 선행 단계다.**
- 산출물: 본 문서의 기록 칸이 모두 채워진 상태 + 그 결과를 반영해 수정된 L0–L8 문서

---

## 1. 왜 이 단계가 먼저인가

포구 알고리즘은 새 workspace에 새로 짓는 것이 아니라 **이미 동작하는 `rtc-framework` 위에 얹는 것**이다. 다음 세 가지가 이미 존재한다(사용자 확정).

1. **vision 노드**가 예측 궤적을 `sensor_msgs/PointCloud2`로 발행하고 있다. 제어 알고리즘은 이 메시지를 받는 것으로 상정한다.
2. **arm-hand system의 kinematics·dynamics 알고리즘**이 이미 구현돼 있다. 포구 구현은 이것을 **참고·재사용**해야 하며, 같은 기능을 새로 만들지 않는다.
3. **joint command backend**가 실기와 시뮬레이션 양쪽에 맞게 이미 구현돼 있다. 명령은 **전부 position**이다. 제어기가 할 일은 연산 결과를 **workspace 구조에 맞는 자리에 데이터로 실어 주는 것**뿐이다.

따라서 L0–L8 문서의 "코드 확인 게이트"(각 문서 §2)는 전부 이 단계에서 한 번에 처리한다. **게이트가 비어 있는 상태로 구현을 시작하지 않는다.** 문서의 가정과 실제 코드가 다르면 문서를 먼저 고친다(각 문서 §2의 규칙).

본 문서는 **조사 계획**이다. 항목별 결론은 조사 후 "기록" 칸에 채워 넣고, 해당 layer 문서의 TBD를 닫는다. 추측으로 채우지 않는다.

## 2. 조사 원칙

- 1차 출처는 **workspace의 소스 코드**다. 기억이나 유추로 API를 적지 않는다.
- 런타임 사실(토픽 필드 레이아웃, 주기, 지연)은 **실제 실행 결과**로 확인한다(`ros2 topic echo --once`, `ros2 interface show`, rosbag).
- 모든 기록에 확인 방법과 확인 일자를 남긴다.
- 기존 코드와 본 설계가 충돌하면, 기존 코드를 따르고 문서를 고치는 쪽을 기본으로 한다. 기존 코드를 바꿔야 한다고 판단되면 그 이유를 적고 사용자 결정을 받는다.

## 3. 조사 항목

### W1. workspace 전체 구조

| ID | 조사 항목 | 방법 | 기록 |
|---|---|---|---|
| W1-1 | 저장소 위치, 기준 브랜치, 브랜치·커밋 명명 규칙 | 저장소 확인 | 닫힘 (`TBD-GIT-01`) — 본 저장소 `rtc-framework`, 기준 브랜치 `main`, 브랜치 `<type>/<kebab-slug>` (예: 현 브랜치 `docs/dynamic-catching-plan`, P-2), 커밋 Conventional Commits `type(scope): subject` ([AGENTS.md](../../AGENTS.md) §10). 확인: `git log`·AGENTS.md (2026-09-19) |
| W1-2 | 패키지 목록과 의존 그래프, 빌드 시스템(colcon 설정, 컴파일 옵션) | `colcon list`, CMakeLists | colcon + `ament_cmake`, C++20 (`CMAKE_CXX_STANDARD 20`). `.colcon/defaults.yaml` 이 `--symlink-install`·Release·compile_commands 를 적용, `build.sh` 가 표준 진입점. 패키지 목록·의존 그래프 SSoT 는 [README.md](../../README.md#패키지-구성)·[architecture.md](../../agent_docs/architecture.md) (여기 박제하지 않음). 확인: README·CMakeLists·[repo_scripts/README.md](../../repo_scripts/README.md) (2026-09-19) |
| W1-3 | ROS 2 배포판, Pinocchio 버전, Eigen 버전, QP solver | 패키지 매니페스트 | ROS 2 Jazzy (README 배지·CI 는 Jazzy 만; Humble 은 지원 표기만). package.xml 은 `pinocchio`·`eigen`·`proxsuite` 를 버전 핀 없이 의존 — dev PC 설치본 Pinocchio 4.1.0, ProxSuite 0.7.3 (QP = ProxQP dense, `QPSolverWrapper`), Eigen 3.4.0. 확인: package.xml·README + 설치본 매니페스트·Eigen 버전 매크로 (2026-09-19). 제어 PC 설치본은 미확인 — S10 |
| W1-4 | 기존 CLAUDE.md / agent_docs 규약과 본 문서 세트의 관계 | 문서 확인 | [AGENTS.md](../../AGENTS.md) 헌법이 본 문서 세트에 그대로 적용된다 (RT-1~10, ARCH-1~7, PROC-*, E-1~E-11, §6.5 Sprint Contract, §9 hard rule). 본 문서 세트는 그 아래의 기능 설계이며 충돌 시 헌법·invariants 우선, 예외는 plan 결정으로만 명문화 (D-2 의 E-1 예외). 확인: AGENTS.md·[invariants.md](../../agent_docs/invariants.md) (2026-09-19) |
| W1-5 | 포구 코드를 **새 패키지로 추가**할지 기존 패키지에 넣을지 | 구조 확인 + 사용자 결정 | 닫힘 → **D-1**: 새 패키지 없음. 수치 코어 rtc_controllers `catching` 하위 (namespace `rtc::catching`), 축 정렬 오차 `rtc_math` se3, CLIK 확장 `rtc_tsid`, 바인딩·YAML·launch·PointCloud2 파서 `integrated_bringup` (2026-09-19) |

### W2. RT 프레임워크

| ID | 조사 항목 | 해소되는 TBD | 기록 |
|---|---|---|---|
| W2-1 | 컨트롤러 기반 클래스, lifecycle 훅, non-RT 스레드 소유 규약 | TBD-RTC-18, 19 | 닫힘 — `RTControllerInterface` 상속, tick = `Compute(const ControllerState&) noexcept` → `ControllerOutput`, 등록 `RTC_REGISTER_CONTROLLER`. 코어(rtc_controllers)+바인딩(integrated_bringup) 2층 ([modification-guide.md](../../agent_docs/modification-guide.md) "Adding a New Controller"). lifecycle 훅 `on_configure`/`on_activate`/`on_deactivate` 등 noexcept. E-STOP 훅 `TriggerEstop`/`ClearEstop`/`SetHandEstop`, fault 래치 `ResetFault`/`HasLatchedFault` (E-STOP 과 분리, `/rtc_cm/reset_fault`). non-RT 스레드는 컨트롤러가 소유 (DemoWbc 의 `rtc::mpc::MPCThread` 관용구 — D-7, plan §6). 확인: 코드 심볼 (2026-09-19) |
| W2-2 | SeqLock / SPSC 원시형의 헤더 위치와 API (단일 writer 가정, reader 재시도 정책) | TBD-RTC-01 | 닫힘 — `rtc::SeqLock` (단일 writer), `rtc::SpscQueue` (rtc_base). **SeqLock payload 는 trivially copyable 필수이고 `Eigen::Vector3d` 는 아니다** (Eigen 3.4) → 궤적·PlanSnapshot 스냅샷은 `std::array` 기반 POD (plan §6, S1.2). 확인: 코드 심볼·static_assert (2026-09-19) |
| W2-3 | 시간 타입 규약 (ns 정수 / double s, `rclcpp::Time` clock type), `use_sim_time` 처리 | TBD-RTC-02, 05 | 닫힘 — RT 는 `ControllerState::t_relative_s` (steady_clock 기반 세션 상대 초)와 `ControllerState::dt` (= 1/`control_rate` 고정; sim lock-step 에서 실제 간격과 다를 수 있음). topic 경계 `header.stamp` = wall, staleness·E-STOP·deadline 판단 금지 (invariants §시계). sim 은 `/clock` 없음 (W6-1). 포구 시간 규약은 **D-2** (절대 steady ns, nrt 수신 시 1회 변환) + plan §3. 확인: 코드 심볼·invariants (2026-09-19) |
| W2-4 | RT 규칙 검사 도구 (할당 카운터, page fault 검사, 경고 수준) | 마스터 §4.2 공통 게이트 | 할당 게이트: test 전용 `ScopedNoMalloc` (rtc_base testing) 와 `ScopedAllocGate` (rtc_controllers testing) — S1·S2·S6 게이트가 사용. RT 패턴 탐지 grep 은 Stop hook `verify-changes.sh` (ARCH 탐지 SSoT) + invariants RT 탐지 패턴. page fault 전용 검사 도구는 확인하지 못함 — 필요 시 S5 에서 [testing-debug.md](../../agent_docs/testing-debug.md) sensor matrix 로 확인. 확인: 코드 심볼 (2026-09-19) |
| W2-5 | subscription callback이 도는 executor / callback group 규약 | TBD-RTC-04 | 닫힘 — 컨트롤러 소유 구독은 LifecycleNode default callback group → `nrt_callback_executor` (단일 스레드, lifecycle 서비스와 공유). RT 로의 전달은 SeqLock/SPSC. 계획 계산을 여기 올리지 않는 이유가 D-7 (plan §6·§7.2). 확인: 코드 + [architecture.md](../../agent_docs/architecture.md) §Execution Contexts (2026-09-19) |
| W2-6 | YAML 파라미터 로딩 패턴 (`generate_parameter_library` 사용 여부) | TBD-RTC-03 | 닫힘 — `generate_parameter_library` **없음**. 컨트롤러 `LoadConfig(YAML)` + `ParseXxxParams` + runtime gain 만 `declare_parameter`. 확인: 코드 심볼·grep (2026-09-19) |
| W2-7 | 로깅 도구: RT 레코드 형식, rosbag 사용 규약 | TBD-RTC-20 | 닫힘 — RT 레코드는 POD 를 SPSC 로 넘기고 aux 스레드가 CSV 로 drain (`rtc::ThreadCsvLogger`, 타이밍은 `rtc::ThreadTimingCsvLogger`, DemoWbc 의 timing CSV drain 관용구). repo 차원의 rosbag 규약은 없음 (invariants 는 topic stamp 의 rosbag 호환만 언급) — 녹화 구성은 S8·S10 에서 정한다. 확인: 코드 심볼·agent_docs grep (2026-09-19) |
| W2-8 | 기존 `DemoWbcController`의 알려진 버그(Stage C-0) 수정 여부와 본 경로 영향 | TBD-RTC-13 | 닫힘 — Stage C-0 수정(C-0.1 friction 키, C-0.2 se3_tcp task 이름, C-0.3 contact_constraint 등록)이 `main` 에 머지됨, 열린 버그 없음. DemoWbc 위치 백본은 `rtc::tsid::ClikReferenceGenerator` (TSID 는 acceleration-level 로 손 τ_ff 용) 이고, 포구는 별도 컨트롤러라 DemoWbc 를 경유하지 않는다 — 영향은 CLIK 확장(D-5) 시 DemoWbc 회귀(S2.4)뿐. 확인: `git log main`·코드 심볼 (2026-09-19) |

### W3. arm-hand system kinematics / dynamics `[재사용 대상]`

**이 항목이 W 단계의 핵심이다.** 이미 있는 것을 다시 만들지 않는다.

| ID | 조사 항목 | 해소되는 TBD | 기록 |
|---|---|---|---|
| W3-1 | 모델 로드 경로: URDF/MJCF 위치, 폐쇄 체인(P1b cross 4-bar) 처리 방식, Pinocchio `Model`/`Data` 소유 주체 | TBD-RTC-14 | 닫힘 — URDF 는 `robot_descriptions` ([README](../../robot_descriptions/README.md)), 로봇 config 의 모델 절이 참조. `PinocchioModelBuilder` (rtc_urdf_bridge) 1개를 CM 이 공유하고 `Data` 는 컨트롤러별 `PinocchioCache` 또는 스레드별 `RtModelHandle`. P1b 폐쇄 체인은 sidecar closure YAML → `ClosedChainModel`. 손바닥 frame 은 루프 상류라 팔 kinematics 에 폐쇄 체인이 끼지 않는다. 확인: 코드 심볼·config (2026-09-19) |
| W3-2 | FK / frame pose 조회 API: 함수 이름, 인자, 어느 frame을 지원하는지 | | `RtModelHandle::ComputeForwardKinematics` → `GetFrameId(name)`·`GetFramePosition`·`GetFrameRotation` (모델의 모든 frame, heap-free, 스레드별 1개). `PinocchioCache` 는 `RegisterFrame` 후 `Update(q, v)`. **모델에 frame 을 추가하는 기능은 없음** → catch frame 은 D-10/D-17 (S2.3a 빌더 확장). 확인: 코드 심볼 (2026-09-19) |
| W3-3 | Jacobian API: 함수 이름과 `ReferenceFrame` 인자 (`LOCAL`, `LOCAL_WORLD_ALIGNED`, `WORLD`). 마스터 §3의 과제별 규약을 이 API 위에 어떻게 얹을지 | TBD-RTC-12 | 닫힘 — `RtModelHandle::ComputeJacobians` + `GetFrameJacobian(frame_id, ReferenceFrame, J)` 가 `LOCAL`/`LOCAL_WORLD_ALIGNED`/`WORLD` 모두 지원. `PinocchioCache` 는 `LOCAL_WORLD_ALIGNED` 고정. 과제별 규약: 병진 LWA, 접근축 LOCAL — CLIK 확장(S2.2) 과 계획기(S6, 스레드 전용 `RtModelHandle`)에서 적용. 확인: 코드 심볼 (2026-09-19) |
| W3-4 | 폐쇄 체인 제약(`RigidConstraintModel`) 구성과 구동/수동 좌표 매핑 (`rtc_urdf_bridge`) | | 존재 확인 — rtc_urdf_bridge `BuildRigidConstraints` 가 closure YAML 에서 `pinocchio::RigidConstraintModel` 목록을 만들고 `ClosedChainModel` 이 구동 관절 id 를 따로 둔다. RT 사영은 `RtClosedChainHandle`. P1b 손 명령은 구동 좌표 position 이므로 포구 경로는 이 매핑을 직접 호출하지 않는다 (L6 §4.4 전제 성립). 확인: 코드 심볼 (2026-09-19) |
| W3-5 | 동역학 API (RNEA/ABA, mass matrix)와 본 포구 경로에서의 필요 여부 — 속도 수준 CLIK만 쓰면 불필요할 수 있다 | | 런타임 position 경로에서는 **쓰지 않음**. 예외: 가속 한계 오프라인 도출(D-16, S2.5)이 `RtModelHandle::ComputeMassMatrix`·`ComputeNonLinearEffects` (M, h) 를 쓴다. 확인: 코드 심볼 (2026-09-19) |
| W3-6 | `rtc_tsid`의 velocity-level CLIK 경로: 입력·출력, 적분 상태가 명령 기반인지 측정 기반인지 | TBD-RTC-09 | 닫힘 — `rtc::tsid::ClikReferenceGenerator`: pose(SE3) 목표 → ProxQP dense box-QP over v → q_ref·v. e·J 는 **측정 q** 에서 평가, anchor carry-forward, 실패 시 q_ref = q_meas·v = 0·false. q_c 평가 모드는 D-6 로 추가 (S2.2). 확인: 코드 심볼 (2026-09-19) |
| W3-7 | 과제(task) 클래스 구조: frame 위치 과제, 각속도 과제, 마스크 지원 방식(LOCAL 고정축 가능 여부), posture 과제 | TBD-RTC-10 | 닫힘 — `rtc_tsid` TaskBase 계열(`SE3Task`·`PostureTask` 등)은 acceleration-level 이고 `SE3Task` mask 는 LWA 행 선택이라 LOCAL 고정축 불가. CLIK 은 6 LWA 행 고정·마스크 없음. `ApproachAxisTask` 신설 대신 **D-5** CLIK 옵션 (LOCAL 접근축 2행, S2.2). 확인: 코드 심볼 (2026-09-19) |
| W3-8 | QP solver 사용 방식: ProxQP dense, box 제약 지원, warm start, 차원 고정 | TBD-RTC-11 | 닫힘 — `QPSolverWrapper` 가 `Init` 에서 최대 차원으로 ProxQP dense 객체를 1회 할당하고 `Solve` 는 update + warm start. CLIK 은 위치∩속도 box, `max_iter` 20 고정, 가속 box·status 노출 없음 → D-5 확장 (S2.2). 확인: 코드 심볼 (2026-09-19) |
| W3-9 | 기존 SE(3)/SO(3) 오차 헬퍼(U1 공유 헬퍼)의 규약과 L4 §4.5 축 정렬 오차의 공존 방식 | TBD-RTC-08 | 닫힘 — U1 = `rtc_tsid` se3_error `ComputeTaskPoseError` (LWA, BodyLog6), 저수준 `rtc_math` se3 `log3`/`exp3`/`Jlog3`. 축 정렬 오차·각속도·Jacobian 은 `rtc_math` se3 에 추가 (D-1, S2.1) 하여 공존. 확인: 코드 심볼 (2026-09-19) |
| W3-10 | catch frame 후보: 두 로봇의 frame 이름과 손바닥 바깥 법선 축 | TBD-FRAME-01 | 방식 닫힘 — 후보 p1b `l_palm_link` +z, iiwa7_leap `palm_lower` −z (URDF link 존재 확인, 축은 FK 도출). 부모 frame·위치 offset·자세는 YAML `extra_frames` 로 열고 (**D-10·D-17**, plan §10) 축 초기 제안값은 S2.3a, 위치(포켓 중심)는 S2.3b (S4.1 이후), 사용자 sim 확인 후 확정. 값은 미확인 — S2.3a/b (2026-09-19) |
| W3-11 | **독립 IK / 포즈 해석기가 있는지**와 그 API | L3 G3-7 | 닫힘 — 독립 IK 없음. `rtc::compliance::DifferentialIk` (σ_min 적응 λ, heap-free) 를 재사용 (**D-7d**, m=5, S6.2). L3 §4.2 DLS 는 새로 짜지 않는다. 확인: 코드 심볼 (2026-09-19) |
| W3-12 | **CLIK이 받는 과제 기준의 형식**: 위치+속도+가속도인지 속도만인지 | TBD-RTC-07 (L4 G4-1) | 닫힘 — **pose 만** (feedforward twist·가속 없음). twist feedforward 는 **D-5** 옵션으로 추가 (기본 off, S2.2). 확인: 코드 심볼 (2026-09-19) |

**재사용 판정 규칙.** W3에서 찾은 기능은 그대로 호출한다. 포구 전용으로 새로 만드는 것은 다음뿐이다.

- 공 궤적 샘플러(L2) — vision 메시지 전용이라 기존에 없다.
- soft-catch 기준 생성기(L4) — 포구 고유.
- 포구 계획기(L3), 손 시퀀서(L6), 슈퍼바이저(L7).

kinematics·dynamics·QP·과제 클래스·적분은 **전부 기존 것**이다. 단 v0.5 판정 (GW-B): 기존 CLIK 은 pose 목표·LWA 6행 고정이라 L5 는 얇은 어댑터로 끝나지 않는다 — `ClikReferenceGenerator` 옵션 확장(D-5·D-6, S2.2) + 새 포구 컨트롤러 (S5). 축 정렬 오차는 `rtc_math` se3 (S2.1), catch frame 은 모델 빌더 확장 (D-10, S2.3a).

### W4. joint command backend `[확정: 전부 position]`

| ID | 조사 항목 | 해소되는 TBD | 기록 |
|---|---|---|---|
| W4-1 | 제어기가 명령을 싣는 자리의 정확한 형태: ros2_control `command_interface` 핸들인지, 자체 구조체/버퍼인지. 필드 이름과 단위 | TBD-ARM-01, TBD-SIM-01 | 닫힘 — ros2_control 아님. `Compute` 가 채우는 `ControllerOutput.devices[]` (팔 device 0, 손 device 1 관례), `CommandType` `kPosition`/`kTorque`/`kPdFeedforward`, SI 단위 (rad). CM 이 `ValidateControllerOutput` → 실패·E-STOP 시 `BuildHoldOutput` 대체 → `DeviceBackend::WriteCommand` (RT 스레드 inline). 확인: 코드 심볼 (2026-09-19) |
| W4-2 | 실기 경로(UR5e `servoj`)와 시뮬레이션 경로가 같은 인터페이스인지, 전환 방법 | TBD-ARM-01 | 닫힘 — 같은 `DeviceBackend` 인터페이스. 실기 `ur_driver_native` (vendor `forward_position_controller` 토픽, position only, **지연 보상 없음**), sim `mujoco_native`. 전환은 로봇 config `devices.<g>.backend.type` (robot.yaml / sim.yaml overlay). 지연 보상이 없으므로 T_arm 식별·선행은 S10 범위. 확인: config·코드 심볼 (2026-09-19) |
| W4-3 | 명령 유효 범위·안전 검사가 backend 안에 이미 있는지 (관절 한계, 속도 제한, 이전 명령 대비 변화량) — 있으면 L5 §4.3과 중복되지 않게 정리 | | CM 의 `ValidateControllerOutput` 이 출력 유효성을 검사하고 실패 시 `BuildHoldOutput` 으로 대체한다. `ApplySafetyLayer` (compliance safety limiter) 는 production 호출 없음. 한계·변화량 제한은 CLIK box (위치∩속도, 가속은 D-5) 가 담당하고 L5 §4.3 은 이와 중복시키지 않는다. 검사 항목의 세부 범위는 S5.3 착수 시 `ValidateControllerOutput` 을 읽고 확정. 확인: 코드 심볼 (2026-09-19) |
| W4-4 | 제어 주기와 명령 타이밍 규약, `update()` 시그니처 | L4 §4.7, L6 §4.3, L8 §4.1 | 닫힘 — `update()` 없음, tick = `Compute(const ControllerState&) noexcept`. 주기는 `control_rate` YAML (default 500 Hz, 설계 범위 100–5000 Hz), h = `ControllerState::dt`. 500 Hz 고정 가정은 폐기하고 L4 ω·h 경계, L6 $T_{tick}$, L8 틱 예산은 **`dt` 기준**으로 검사 (S0.3, S1.7). 확인: 코드 심볼·README (2026-09-19) |
| W4-5 | 손(P1b) 명령 경로: 같은 backend인지 별도 노드인지, 메시지 타입·모드·주기·수신 스탬프 | TBD-HAND-02 | 닫힘 — 같은 `ControllerOutput` 의 손 device slot → `udp_hand_native` backend → `/p1b/joint_command` → 별도 프로세스 `udp_hand_node` (250 Hz). 명령 `header.stamp` 미사용, 실기 손은 feedforward 무시 = position only. sim 은 `mujoco_native`. 손 포트 추상화는 **D-11** 로 폐기. 확인: config·코드 심볼 (2026-09-19) |
| W4-6 | 지문 센서 입력 경로: state interface인지 토픽인지, frame·부호·주기 | TBD-HAND-03 | 닫힘 — 실기: `udp_hand_native` 가 받는 `HandSensorState` (finger-on-object 부호, 250 Hz) 가 device 센서 상태로 컨트롤러에 들어온다. sim: MuJoCo `WrenchStamped` — 두 경로 모두 finger-on-object 부호 (sim contact-wrench lane 은 커밋 0fcc1d23 부터 같은 부호, `rtc_msgs` FingertipSensor 주석의 반대 부호 서술은 stale). S7.3 착수 시 재확인. frame 세부는 S7.3 에서 확인. 확인: 코드 심볼 (2026-09-19) |
| W4-7 | UR 드라이버 speed scaling 상태 인터페이스 | TBD-ARM-03 | 노출 없음 — `ur_driver_native` 는 speed scaling 을 컨트롤러에 전달하지 않는다. 신호 출처 확보·감시는 S10. 확인: 코드 grep (2026-09-19) |
| W4-8 | 관절 한계 출처(URDF vs 운용 설정), 운용 가속 한계. L5 §6은 "URDF"로 단정하고 있어 다르면 충돌 | TBD-ARM-02 | 닫힘 — YAML `devices.<g>.joint_limits` 우선 + URDF 교집합 (L5 §6 의 "URDF" 단정은 틀림). `max_acceleration` (5.0 rad/s²) 은 CM 이 읽기만 하는 placeholder 로 어떤 컨트롤러도 쓰지 않음 → 가속 한계는 토크에서 도출 (**D-16**, plan §9, S2.5). 확인: config·코드 심볼 (2026-09-19) |
| W4-9 | 기존 WBC의 손 명령 경로(current feedforward 등)와 포구 경로의 충돌 여부 | TBD-RTC-17 (L6 G6-6) | 닫힘 — 충돌 없음. DemoWbc 의 손 τ_ff 는 `kPdFeedforward` 이지만 실기 P1b 는 feedforward 를 무시 (position only). CM 은 활성 컨트롤러 하나의 출력만 보내므로 포구 컨트롤러는 손 device slot 에 position 을 직접 쓴다 (D-11). 확인: 코드 심볼 (2026-09-19) |

**설계 함의.** backend가 이미 있으므로 L5의 범위는 "과제 → QP → 적분 → **backend에 $q_c$ 쓰기**"로 끝난다. 드라이버 파라미터 조정이나 명령 경로 신설은 범위 밖이다. L5 문서의 `servoj` 지연 식별(§4.4)은 **backend가 이미 보상하고 있지 않은 경우에만** 필요하다 — W4-2 결과 backend 는 지연을 보상하지 않으므로 T_arm 식별·선행은 필요하며 S10 에서 한다. 손은 포트 추상화 없이 손 device slot 에 직접 쓴다 (D-11).

### W5. vision 인터페이스 (`sensor_msgs/PointCloud2`)

| ID | 조사 항목 | 해소되는 TBD | 기록 |
|---|---|---|---|
| W5-1 | 토픽 이름 | TBD-VIS-01 | sim: `/ball_perception/debug/prediction/trajectory` (ball_perception `sim_estimator_node`). **debug 토픽, stable ABI 아님** — 제품 토픽은 미정 (ball_perception E6-F02 로 defer, D-4). 확인: ball_perception 코드 (2026-09-19) |
| W5-2 | **실제 `PointField` 배열 덤프**: 각 필드의 name·offset·datatype·count, `point_step`, `row_step`, `is_bigendian`, `is_dense` | TBD-VIS-02 | 닫힘 — little-endian, `point_step` 384, 산술이 닫힌다 (아래 표). 필드 이름으로 파싱 (**D-4**). `row_step`·`is_dense` 의 실측은 S3.4. 확인: ball_perception 발행 코드 (2026-09-19) |
| W5-3 | `t` 필드의 타입과 기준 (float64 상대 초 / uint32 상대 ns) | TBD-VIS-03 | 닫힘 — `t` 필드 없음. `horizon_ns` UINT32, `header.stamp` (= 예측 원점 시각) 기준 상대 ns. 절대 시각 변환은 D-2. 확인: ball_perception 코드 (2026-09-19) |
| W5-4 | `ax, ay, az`가 점마다 같은 상수 $g$인지, 항력이 반영된 그 시각의 총 가속도인지 | TBD-VIS-05 | 닫힘 — 상수 $g$ (중력만 모델, 항력 미반영). 확인: ball_perception 코드 (2026-09-19). 데이터 대조는 S3.4 |
| W5-5 | `header.frame_id`가 계획·기준 생성에 쓰는 `world`와 같은지 | TBD-VIS-06 | 닫힘 — `world` (S3.4 실측 2026-09-20) |
| W5-6 | 발행 주기, $N$(width)의 범위, 지평 길이, 지연 분포 | TBD-VIS-04 | 예시 sim profile: 지평 0.5 s, 간격 0.05 s, 최대 10 점, 발행 ≤ 30 Hz (ball_perception 설정 (2026-09-19)) — S0.7 결과로 목표를 지평 0.8 s·**16 점**으로 올렸다 (plan D-15; 점 수는 2026-09-20 정정 — 예측점이 `step, 2·step, …, horizon` 이라 t = 0 이 없어 0.8/0.05 = 16 이다). 단 **요구 사양은 포구 제어기가 정하고 sim profile 을 맞춘다 (D-15)**. 실측 주기·N·지연 분포는 S3.4. **S3.4 실측 (2026-09-20)**: 0.8 s 프로파일에서 30.0 Hz · N 16 · 지평 0.05…0.80 s. **S3.6 이 요구를 닫았다 (2026-09-22)**: 권장 profile 1.05 s · 0.05 s · 21 점 (plan §4.4 S3.6 결과) |
| W5-7 | ~~[요청 사항]~~ 트랙 식별·유효성 필드 | TBD-VIS-07 | 닫힘 — 요청 불필요. `generation` (uint64), `validity` (uint8), `snapshot_sequence` (uint64) 가 이미 있다 (사용자가 ball_perception 을 직접 개발, **P-3**). 사용법은 D-4, S5.2. 확인: ball_perception 코드 (2026-09-19) |
| W5-8 | 두 PC 간 시계 동기(PTP) 상태 확인 방법 | TBD-NET-01 | 미확인 — S10 (PTP 감시 신호 출처 확보 후). sim 은 단일 PC 라 해당 없음 |

**W5-7 은 더 이상 요청이 아니다 (P-3).** v0.4 는 `PointCloud2` 에 트랙 식별자·상태가 없다고 보고 유령 트랙(공을 놓친 뒤 관성 예측만 발행) 판별을 위해 필드 추가를 요청하려 했다. 실제 발행기에는 `generation`·`validity`·`snapshot_sequence` 가 이미 있어 이 위험은 필드로 닫힌다 — 사용법은 D-4 와 S5.2 (파서가 필드 이름·datatype 을 검사하고 레이아웃 해시로 변경을 진단). 남은 위험은 debug 토픽이라 stable ABI 가 아니라는 점뿐이다 (plan §12).

**`point_step` 산술이 닫힌다.** ball_perception 발행 코드의 실제 레이아웃 (little-endian, 2026-09-19 확인):

| 필드 | offset | datatype × count | 바이트 |
|---|---|---|---|
| `x, y, z, vx, vy, vz, ax, ay, az` | 0–64 | FLOAT64 × 1 (9개) | 72 |
| `covariance` (row-major px..vz, 모르면 NaN) | 72 | FLOAT64 × 36 | 288 |
| `snapshot_sequence` (uint64 low/high) | 360 | UINT32 × 2 | 8 |
| `generation` (uint64 low/high) | 368 | UINT32 × 2 | 8 |
| `horizon_ns` (header.stamp 기준 상대 ns) | 376 | UINT32 × 1 | 4 |
| `validity` (0 NOT_EVALUATED, 1 VALID) | 380 | UINT8 × 1 | 1 |
| 정렬 패딩 | 381 | — | 3 |
| 합계 = `point_step` | | | **384** |

72 + 288 + 8 + 8 + 4 + 1 + 3 = 384 이고 각 offset 이 앞 필드의 끝과 일치한다. v0.4 가 가정한 372 B·`t`·`cov` 는 폐기 (D-4). 파서는 여전히 offset 가정 없이 `PointField` 배열에서 필드 이름으로 구성한다 (L1 §5.1).

### W6. 시뮬레이션

| ID | 조사 항목 | 해소되는 TBD | 기록 |
|---|---|---|---|
| W6-1 | MuJoCo ↔ ros2_control 연동 방식, `/clock` 발행, 공(freejoint) 상태 접근 | TBD-SIM-01 | 닫힘 — `rtc_mujoco_sim` 은 ros2_control·`/clock` 없음. lock-step (명령 대기, `sync_timeout_ms` 50), `max_rtf` 1.0 은 상한만, stamp = wall `now()`, 컨트롤러는 eventfd 로 깨어남 (`use_sim_time_sync` 는 ROS `use_sim_time` 과 무관). 공: `/sim/launch_ball`·`/sim/reset_ball` (std_srvs Trigger, YAML 분포 샘플), `/sim/ball/ground_truth` (nav_msgs/Odometry, world, cov 0), 100 Hz sim-time throttle. ur5e_p1b 스폰은 손바닥 위 (낙하 테스트용), iiwa7_leap 설정 없음 → S3.2. 시간축은 **D-3** (wall + 시행별 clock 위상 오차 게이트, plan §5, S3.1a·S3.1b 검증). 확인: 코드·config (2026-09-19) |
| W6-2 | 두 MJCF의 actuator 종류·게인, 폐쇄 체인 `<equality><connect>` 구성 | TBD-SIM-01 | 닫힘 — UR 팔 `<general>` position-PD, forcerange ±150/±28 N·m. P1b 손 `<position>` kp 6000, forcerange ±3, `<equality><connect>` 5개 (p1b MJCF 는 형제 저장소 hand-description). LEAP 손 `<general>` 16개, equality 없음. iiwa7 팔 게인은 S3.2 에서 확인. 확인: MJCF (2026-09-19) |
| W6-3 | MJCF 공 유체 모델 설정 (L0 모델과 다른 항력 식인지) | TBD-SIM-02 | 닫힘 — MuJoCo 유체 모델이 아니라 `rtc_mujoco_sim` 자체 구현: 항력 ½ρC_dA\|v\|v + Magnus (tennis preset r 0.025 m, m 0.05 kg). L0 모델과의 계수 대응은 L8 fixture 에서 대조. 확인: 코드·config (2026-09-19) |
| W6-4 | 시뮬레이션에서 vision 역할을 대신할 발행기를 어디에 둘지 | | 닫힘 — 새로 만들지 않는다. ball_perception 의 `sim_estimator_node` 가 `/sim/ball/camera_position` (PointStamped + Gaussian noise) 을 구독해 예측 PointCloud2 를 발행한다. 연결·clock domain·지연 주입은 S3.4. 확인: ball_perception 코드 (2026-09-19) |

### W7. 운영 환경

| ID | 조사 항목 | 기록 |
|---|---|---|
| W7-1 | 바닥 높이·작업셀 경계의 `W` 좌표 (`TBD-WS-01`) | 미확인 — 사용자 제공 값 (D-12). 발사 영역 높이(world z 1.5–2.0 m)는 D-18 로 정의됨. 작업셀 경계는 S3.5a catchability 지도 착수 시 입력으로 함께 정한다 (plan §7.3) |
| W7-2 | 공 지름·질량·재질(반발) (`TBD-BALL-01`), 허용 충격량 (`TBD-IMP-01`) | 미확인 — 사용자 제공 값 대기 (D-12, provisional 표시). sim 은 tennis preset (r 0.025 m, m 0.05 kg) 으로 진행 |
| W7-3 | 투척 속도·거리 범위, 포구 허용 작업공간 (`TBD-BALL-02`) | 방식 닫힘 — **D-18**: 발사 영역 = base frame 수평 거리 4 m 원호, world z 1.5–2.0 m, 비행시간 T_f ≥ 1.0 s. 속도·앙각·방위 범위는 catchability 지도 (manipulability w₅ ≥ 0.1 provisional) 결과로 정한다 — S3.5a/b (plan §11) |
| W7-4 | P1b 사양 실측: 구동 좌표 정의, preshape/폐쇄 자세, 전류·토크 한계, 포켓 유효 깊이 (`TBD-HAND-04, 05`) | 미확인 — 손 프로파일 S4.1, T_close,tot 실측 S4.3 (D-11), 포켓 중심은 catch frame 제안값 S2.3b (D-17, S4.1 이후) |

## 4. 수행 순서

**완료 (2026-09-19).** 아래 순서로 코드 대조를 마쳤다. 기록은 §3 표에 있고, layer 문서 반영은 S0.3 (GW-E) 에서 진행한다.

0. ~~W5-7을 먼저 요청한다~~ — 요청 불필요 (P-3, 필드가 이미 있음).
1. W4-1, W5-2 — 완료 (코드).
2. W1 → W2 → W3 — 완료 (코드).
3. W5의 나머지 — 코드로 닫힌 것은 닫고, 실제 토픽 데이터가 필요한 W5-5·W5-6 실측은 S3.4 로 넘긴다.
4. W6, W7 — W6 완료 (코드), W7 은 사용자 값 대기 (D-12) 와 D-18 지도(S3.5a/b).
5. 기록을 각 layer 문서의 §2 게이트 표와 §10 미확정 항목에 반영하고, 어긋나는 서술을 고친다 — S0.3 진행 중.
6. 그 다음에야 구현을 시작한다 — 단계 순서는 plan §4 (S1 ∥ S2 ∥ S3 부터).

마스터 §4.1의 **$T_{close,tot}$ 선행 측정(L6a)** 은 S4 (go/no-go) 로 옮겼다 (D-11).

## 5. 이 단계의 합격 기준

| 게이트 | 기준 | 상태 (2026-09-19) |
|---|---|---|
| GW-A | W4-1, W5-2, W5-3, W5-4, W5-5가 실제 코드·데이터로 확정 | 부분 통과 — W4-1·W5-2·W5-3·W5-4 는 코드로 확정. W5-5 (`frame_id`) 는 S3.4 실측 |
| GW-B | W3-1, W3-2, W3-3, W3-6, W3-7, W3-10, W3-11, W3-12가 확정되고, L5가 어댑터로 충분한지 판정 | 통과 — 판정: 얇은 어댑터로는 부족, CLIK 확장(D-5·D-6) + 새 컨트롤러. W3-10 은 방식 확정 (D-10·D-17), 값은 S2.3a/b |
| GW-C | W2-2, W2-3이 확정되고 L1 스냅샷 브리지 설계가 그 원시형에 맞게 수정됨 | W2-2·W2-3 확정 (POD 스냅샷, D-2). L1 수정은 S0.3 진행 중 |
| GW-D | 마스터 §9.1의 `TBD-RTC-*` 전부, `TBD-VIS-*`, `TBD-ARM-01`, `TBD-FRAME-01`, W4-3(backend 중복 범위), W4-4(제어 주기), W5-6(발행 주기·$N$)이 닫힘 | 부분 통과 — `TBD-RTC-*`·`TBD-ARM-01`·W4-3·W4-4 닫힘, `TBD-VIS-01/02/03/05/07` 닫힘. `TBD-VIS-04` (W5-6 실측·요구 사양 D-15) 는 S3.4·S3.6, `TBD-VIS-06` 은 S3.4, `TBD-FRAME-01` 값은 S2.3a/b |
| GW-E | 위 결과로 L0–L8 문서를 수정하고, 남은 TBD가 §9 표에만 존재 | 진행 중 — S0.3 설계 문서 v0.5 동기화 |

**§1 "단계 W 완료" 와 위 표의 관계.** "단계 W 완료"는 **조사 작업**(코드 대조, §4)이 끝났다는 뜻이지 GW-A~E 게이트가 전부 통과했다는 뜻이 아니다 — 위 표대로 GW-A·GW-D 는 **부분 통과**, GW-E 는 **진행 중**이며 여기서 "통과"로 올려 적지 않는다. 아직 남은 부분은 각각 plan §4.4 의 단계 게이트가 이어받는다:

| 미통과 항목 | 잔여 내용 | 이어받는 plan 단계·게이트 |
|---|---|---|
| GW-A (부분) | W5-5 `frame_id` ↔ world 실측 | S3.4 (plan §4.4 S3a) |
| ~~GW-D (부분)~~ | `TBD-VIS-04` (발행 주기·N·지평 실측) | **닫힘** — 실측 S3.4 (30 Hz · 16 · 0.8 s), 요구 사양 S3.6 (지평 1.05 s · 간격 0.05 s · 21 점 권장, plan §4.4 S3.6 결과) |
| GW-D (부분) | `TBD-VIS-06` (`frame_id` 관계) | 닫힘 (S3.4: `world`) |
| GW-D (부분) | `TBD-FRAME-01` 값 (catch frame 부모·offset·자세) | S2.3a (축), S2.3b (위치, S4.1 이후) (plan §4.4 S2) |
| GW-E (진행 중) | L0–L8 문서 동기화 잔여, 남은 TBD 를 §9 표로 수렴 | S0.3 (plan §4.4 S0), 이후 각 TBD 는 위 표·§9 가 가리키는 단계 |

즉 이후 구현은 plan §4 의 단계 게이트를 따라 진행하고, 위 잔여 항목들은 "GW 통과 보류" 상태가 아니라 **그 이름 붙은 단계의 게이트**로 이관된 것이다.

GW-A~E가 통과하기 전에 나오는 "상세 구현"은 전부 추측이다. 하지 않는다.
