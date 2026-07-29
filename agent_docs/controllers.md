# Controllers

## Controller Table

이 표의 축은 **제어 법칙 (알고리즘 코어)** 이지 클래스가 아니다. `rtc_controllers` 는 순수 알고리즘
라이브러리이고 `RTControllerInterface` 구체 구현 (= 바인딩) 은 integration 패키지가 소유한다.
규칙 · 3계층 배치 · 경계 판정의 SSoT 는
[design-principles.md](design-principles.md) §`rtc_controllers` Controllers Are Pure Control
Algorithms 이며, 여기서는 반복하지 않는다 (AP-DOC-1).

> **현황 (#236 S7c 완료)** — 한때 같은 패키지에 있던 어댑터 클래스들은 삭제됐다
> (`grep -rn "public RTControllerInterface" rtc_controllers/include` 가 비어야 한다; 개수는
> 박제하지 않는다). 그것들은 **어디에도 등록돼 있지 않아 런타임 노출이 0** 이었고 — 배포되는
> 것은 아래 §배선된 컨트롤러 3종, 즉 integration 패키지의 바인딩뿐이다 — 코어 추출의
> cross-check oracle 로만 살아 있었다.
>
> 아래 표의 **"잔여 → S7 정리 완료"** 는 그 어댑터 안에 있던 글루가 어디로 갔는지를 뜻한다:
> 모든 바인딩이 동일하게 필요로 하던 공통 글루(target mailbox 등)는 S7a 에서
> `rtc_controller_interface` 로 올라갔고, 배치 고유 글루는 어댑터와 함께 삭제됐다 (바인딩은
> 자기 것을 갖는다). 아직 코어로 수렴하지 않은 **바인딩 쪽 인라인 사본**은 별개이며
> [design-principles.md](design-principles.md) §현황 이 SSoT 다 (#282).

### 알고리즘 코어 (`rtc_controllers`)

| 제어 법칙 | Type | Space | 코어 위치 | Key Feature |
|---|---|---|---|---|
| P (관절 증분) | Position | Joint | **코어 없음** — 어댑터와 함께 은퇴 (#236 D-Q1: 법칙이 `q + kp·e·dt` + 클램프라 파일 하나 값어치가 없다). 관절 PD 를 `kd = 0` 으로 쓰면 같은 법칙 | `q + kp*error*dt` incremental |
| Joint PD | Torque | Joint | **추출됨** — `joint/joint_pd_law.hpp` (S1). 무상태 free function: 궤적 **샘플**을 받고 생성기를 소유하지 않으며 (배선은 integration 계층), 오차 이력은 in/out span (E-STOP hold 정책과 공유). 잔여(mailbox·모델 바운드·궤적 배선·텔레메트리·클램프) → S7 정리 완료 | PD + Pinocchio RNEA + quintic trajectory |
| CLIK | Position | Cartesian 3/6-DOF | **부분 추출됨** — `task/task_vel_law.hpp` (S3a). 무상태 free function 2개 (6축 `ComputeTaskVelocity` · 병진전용 `ComputeTranslationVelocity` — 인라인 형태가 서로 다른 식이라 합치지 않는다): 태스크 속도 `task_vel = K_p ⊙ e + ν_ff` 만 담고, pose error 정의·궤적·**ν_ff 의 프레임 전송**(`R_traj` vs `R_current`)을 모른다 (게인은 속도형 `[1/s]`, 미분항 없음). DLS `J⁺`/영공간은 **`compliance/differential_ik` 로 흡수 완료** (S3b/#258, m=6·m=3 두 인스턴스 — `control_6dof` 가 런타임 가변 게인이라 둘 다 미리 sizing) + 영공간 자세는 `joint/posture_law.hpp` 의 `ComputePostureVelocity` (게인이 사영 **前** 로 이동 — 그래야 다른 소비자와 같은 법칙). **비-bit-identical 이관**이며 λ 규약이 compliance §6.5 로 바뀐 것이 그 목적이다 (D-S2b (b)안). 잔여(관절 tail·클램프·적분) → S7 정리 완료 | Damped Jacobian pseudoinverse + null-space |
| OSC | Torque | Cartesian 6-DOF | **부분 추출됨** — `task/task_accel_law.hpp` (S2a). 무상태 free function: 태스크 가속도 `a_task = K_p·e + K_d·(ν_d−ν) + a_ff` 만 담고, pose error 정의·궤적·모델을 모른다 (게인은 가속도형 `[1/s²]`, `impedance_law` 의 힘형과 Λ 배 차이). `Λ`/`τ`/`Nᵀ` 는 **`compliance/task_dynamics` 로 흡수 완료** (S2b, D-Q2) + 영공간 자세는 `joint/posture_law.hpp` 의 `ComputePostureTorque` (`q_ref` 인자 = 설정된 `safe_position`). **비-bit-identical 이관**이며 λ 규약이 compliance §6.5 로 바뀐 것이 그 목적이다 (D-S2b (b)안). 잔여(τ 조립·클램프·텔레메트리) → S7 정리 완료 | Full pose PD + SE3 quintic trajectory; gravity-comp damped torque E-STOP (`ĝ(q)−D·q̇`, #184) |
| Task impedance | Torque | Cartesian 3/6-DOF | **부분 추출됨** — `compliance/{impedance_law, inertia_shaping, task_dynamics, wrench_pipeline, safety_limiter, compliance_state_machine, torque_estop}`. compliance §6.3 관성 성형은 `inertia_shaping.hpp` 의 무상태 free function `ComputeShapedTaskForce` (S4): `f_cmd = B·f_task + (B−I)·f_ext`, `B = Λ_S Λ_d⁻¹` + compliance §5.2 편차 clamp 만 담고 **Λ_S 를 인자로** 받아 `TaskDynamics` 를 모른다 (수렴점 판정은 S2b). 영공간 자세 법칙은 `joint/posture_law.hpp` 의 `ComputePostureTorque` (S6) — cascade 의 인라인 루프와 **문자 그대로 동일**했던 것이 ARCH-3 발동 조건이었다. compliance §6.2 게인은 `Gains` 에 `compliance::ImpedanceParams` 로 **중첩**돼 있다 (S6b — 기본값이 코어와 같아 중복이 실제로 사라지고, 매 tick positional 재조립도 함께 사라진다; YAML 키는 불변). 잔여(Λ 게이트·selection matrix·τ 꼬리) → S7 정리 완료 | compliance §6.2 A=NONE Jacobian-transpose compliance `Jᵀ Sᵀ[Kp·Se+Kd·Sė]+τ_null+ĝ`; Λ 미사용(특이점 자유), σ_min-adaptive DLS nullspace (`nv>task_dim`), gravity-comp torque E-STOP, MuJoCo-only. 규범: [rtc_controllers/docs/compliance-conventions.md](../rtc_controllers/docs/compliance-conventions.md) |
| Task admittance | Position | Cartesian 6-DOF | **부분 추출됨** — `compliance/{admittance_integrator, differential_ik, wrench_pipeline}` + compliance §7.3 속도항은 `task/task_vel_law.hpp` 재사용 (S5, 새 코어 없음) + 영공간 자세 법칙 `joint/posture_law.hpp` 의 `ComputePostureVelocity` (S6 — 속도형 **P**, PD 의 `Kd=0` 이 아니다). 잔여(관절 tail·클램프·적분) → S7 정리 완료 | compliance §7 Rule 3 — 힘 **입력** → 운동 출력. compliant frame `Λ_d ẍ̃+K_d ẋ̃+K_p x̃ = f_ext` 를 semi-implicit Euler + `exp3` retract 로 적분하고 compliance §7.3 DLS 미분 IK 로 `q_cmd` 생성. 외부 wrench **필수** (A≠NONE, `enabled: false` 는 configure 에러), `min_desired_inertia` 하한(compliance §7.4) · `max_compliant_displacement`/velocity(compliance §7.5, `≤0`=off) · `max_return_{linear,angular}_velocity`(경계 밖 복귀 속도 상한, **상시**), E-STOP 은 position-hold(latch, 재활성화·`ClearEstop`·`ResetFault` 경계에서 무효화). impedance 와 **반대 부호**(compliance §11.4.1 P2/P3). 규범: [rtc_controllers/docs/compliance-conventions.md](../rtc_controllers/docs/compliance-conventions.md) §3.5 |
| Cascaded compliance | Torque | Cartesian 6-DOF | **부분 추출됨** — 위 두 행의 `compliance/*` 조합 + compliance §7.6 MUST-1 판정식 `compliance/bandwidth_separation.hpp` (S6, `Λ_S` 를 인자로 — 만드는 쪽은 `compliance/task_dynamics`, S2b 이후 다섯 컨트롤러 공통) + 영공간 자세 법칙 `joint/posture_law.hpp` (S6). 잔여(τ 조립·compliance §10.5 safety layer·FSM) → S7 정리 완료 | compliance §7.6 — outer admittance(느린 대역, 순응 정의) 가 만든 compliant frame `(X_c, ν_c)` 를 inner compliance §6.2 impedance(빠른 대역)가 추종. `τ = Jᵀ·α[K_p^i·e(X,X_c) + K_d^i·(ν_c−ν)] + α·Nᵀτ_posture + ĝ`. 외부 wrench **필수** (outer 의 입력). **wrench 는 정확히 한 번만 소비** — inner 에 `f_ext` 항이 없다(compliance §7.6 MUST-4 를 런타임 검사가 아니라 타입 레벨로, D19). 대역폭 분리 `ω_i/ω_a`(MUST-1) 는 seeding tick 에 `Λ_S(q₀)` 로 1회 평가해 **진단 플래그** `bandwidth_ratio_low` 로만 보고(fault 아님, D20). YAML 은 `outer:` / `inner:` 로 분리(양쪽 다 stiffness·damping 을 가진다). E-STOP 은 torque hold `ĝ−D·q̇`. 규범: [rtc_controllers/docs/compliance-conventions.md](../rtc_controllers/docs/compliance-conventions.md) §3.6 |
| Grasp (Force-PI) | Internal | Hand 3x3-DOF | **코어** — `grasp/grasp_controller.hpp`. `RTControllerInterface` 를 상속하지 않고 상위 컨트롤러가 멤버로 소유한다 (규칙이 명문화되기 전부터 이미 목표 형태) | Adaptive PI force, 6-state FSM, per-finger stiffness EMA |

`compliance/*` 는 규칙의 예시가 아니라 **기준**이다 — Eigen/span in-out, `Resize()`/`Compute()` 분리,
프레임워크 타입 무지. 새 법칙은 처음부터 그 형태로 쓴다.

### 배선된 컨트롤러 (`integrated_bringup`)

`RTControllerInterface` 구체 구현 + `RTC_REGISTER_CONTROLLER` 등록 + production YAML 을 갖는, 실제로
도는 것들이다.

| 컨트롤러 | Type | Space | Key Feature |
|---|---|---|---|
| DemoJointController | Position | Joint + Hand | Quintic trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"\|"none"` |
| DemoTaskController | Position | Cartesian + Hand | CLIK + trajectory, `grasp_controller_type: "contact_stop"\|"force_pi"\|"none"` |
| DemoWbcController | Position | TSID QP + Hand | `initial_controller` default — **`ur5e_p1a`·`iiwa7_leap` 만**. `ur5e_p1b` 는 sim·robot 둘 다 `demo_joint_controller` 다 (robot profile 별 `{sim,robot}.yaml` 이 SSoT). 6-phase FSM (Idle->Approach->Closure->Hold->Release; slots 2 & 5 reserved, RELEASE preempts from any non-terminal phase), TSID QP -> accel -> position integration across all phases, contact-aware ForceTask + FrictionCone, sensor-driven contact / slip / deformation guards, combined 16-DoF model. MPC default: `engine: "handler"` + `enabled: false` (structural gate; MPC thread inert and TSID self-holds until YAML `mpc.enabled: true` AND runtime `mpc_enable` — see line below) |

### Joint order & submodel selection

device joint order 와 primary-device submodel 처리는 컨트롤러마다 복제된 법칙-무관 boilerplate 라
#236 이 G1(프레임워크 공통 글루)로 분류한 뒤 S7c 에서 어댑터와 함께 삭제됐다. base 에 남은 것은
그 **입력** (`RTControllerInterface` 의 `GetPrimaryDeviceName()` / `GetSystemModelConfig()`) 이고,
모델을 실제로 고르고 재정렬하는 것은 바인딩 몫이다 (`integrated_bringup/src/controllers/*/controller.cpp`).
삭제된 어댑터의 동작 기록은 #172 Phase 3 · #236 에 있다.

남기는 것은 **재정렬 함정** 하나다 — 바인딩에서 그대로 재발하며, 발현하면 모든 수가 유한한 채
토크만 틀린다: device 순서로 형성한 항 (null-space·Coriolis·관절속도 `q̇`) 을 Pinocchio 순서 행렬과
곱하기 직전 `RtModelHandle::ReorderInput` (device→Pinocchio scatter, `ReorderOutput` 의 역방향) 으로
1회 gather 한다. `ν = J·q̇` 의 `q̇` 를 빠뜨리면 감쇠 토크가 틀리면서 fault 도 안 뜬다 (#236 슬라이스 4
리뷰에서 실제 발현).

### Unified arm kin&dyn (joint / task / wbc)

세 데모 컨트롤러(DemoJoint/DemoTask/DemoWbc)는 arm kinematics(FK·Jacobian)를 **결합(arm+hand) actuated
모델 위의 `rtc_urdf_bridge::PinocchioCache`** 에서 획득한다 — non-E-STOP tick당 `Update(q,v)` 1회
(`GetActuatedModel` → reduced tree `"wbc"` → full model 순으로 선택). arm-only `RtModelHandle`(`arm_handle_`)은
E-STOP TF 경로(`ComputeEstop`)와 메타데이터(`nv()`/`GetFrameId`)로만 잔존. closed-chain 손에서는 접촉
프레임 J·oMf·dJv가 loop-consistent로 격상된다(dJv는 `RtClosedChainHandle` drift API 로 L2-exact, issue #173 종결).

이 cache 배선(모델 선택 + `PinocchioCache.Init`, ext(device joint-state)→Pinocchio q/v reorder map, 매 tick
state scatter, arm-TCP FK)은 **공유 타입 `integrated_bringup::CombinedModelCache`(support/) 한 곳으로 통합**됐다
(issue #174, 이전 삼중 복제 해소). 세 컨트롤러는 멤버 `combined_cache_` 를 소유·위임한다(`InitModel`/`BuildReorderMap`/
`ExtractFullState`/`Update`/`ArmTcpPoseFromCache` + `cache()`/`model()`/`reorder_valid()`/`q()`/`v()` accessor).
WBC 는 superset 소비자로, TSID contact 프레임과 reduced-dynamics provider 는 WBC-local 로 유지하고, 모델선택은
`SelectModel`(contact frame id 파싱이 선택된 모델을 요구해 cache Init 을 뒤로 미룸) + 이후 `cache().Init` 로 분리
호출한다.

RT 진입점 `Compute(state)` 는 세 컨트롤러 공통으로 **`ReadState`(순수 raw read + `ExtractFullState`) → compute
model(`combined_cache_.Update`) → compute control law → `WriteJointCommand`(순수 command write)** 순서를 지킨다
(로그/발행용 oMf 소비는 `FillLogOutput`/`FillPublishOutput` 로 분리). cache.Update 텍스트 위치는 joint·wbc = `ComputeControl`
Stage 1, task = Compute-scope Stage 1(DrainTargetSlot self-init 이 fresh cache 의존). WBC 상세 구현:
[docs/WBC_CONTROLLER_IMPLEMENTATION.md](../docs/WBC_CONTROLLER_IMPLEMENTATION.md).

## Gains (per-controller ROS 2 parameters)

게인 채널은 ROS 2 parameter API로 노출된다 ([rtc_msgs/srv/GraspCommand.srv](../rtc_msgs/srv/GraspCommand.srv)). Legacy `~/controller_gains` / `~/request_gains` / `~/current_gains` 토픽 + `UpdateGainsFromMsg`/`GetCurrentGains` 가상 메서드는 모두 제거.

각 데모 컨트롤러는 자기 LifecycleNode (`/<config_key>`) 에서 `declare_parameter`로 게인을 노출하고, `add_on_set_parameters_callback`이 SeqLock writer 측으로 mutate→Store. RT 경로는 `gains_lock_.Load()` 스냅샷만 본다.

각 데모 컨트롤러가 노출하는 정확한 parameter list 는 코드/YAML 이 SSoT:

컨트롤러 LifecycleNode 는 **이름과 namespace 를 둘 다 `config_key` 로** 만들어지므로 (`rt_controller_node_params.cpp`: name=`config_key`, ns=`/config_key`), FQN 은 `/<config_key>/<config_key>` 다:

```bash
# 활성화 후 actual parameter list
ros2 param list /demo_wbc_controller/demo_wbc_controller
ros2 param describe /demo_wbc_controller/demo_wbc_controller <param>
```

또는 컨트롤러 source 의 `DeclareGainParameters()` 멤버 + bringup config (`integrated_bringup/config/<robot>/controllers/<config_key>.yaml`).

읽기 전용 cap parameter (`*_max_traj_velocity` 등) 는 `ParameterDescriptor::read_only=true` 로 선언. One-shot 이벤트 (Force-PI grasp) 는 `grasp_command` srv (`rtc_msgs/srv/GraspCommand`) — **상대 이름**으로 advertise 하므로 노드 namespace 기준 `/<config_key>/grasp_command` 로 해석된다 (`~/` 를 쓰면 이름이 한 번 더 중첩된다). Active controller 만 server 를 띄움.

`mpc_enable`은 빌드타임 `mpc_enabled_` (YAML `mpc.enabled`) 와 AND 결합 — YAML이 false면 런타임 1은 무시된다. `riccati_gain_scale`은 `[0,1]`로 자동 clamp.

`rtc_controllers` 는 게인 채널을 노출하지 **않는다 — 노출할 수 없는 것이 정상이다.** 파라미터 채널은 LifecycleNode 를 요구하는데 이 패키지는 노드를 만들지 않으며, 코어가 보는 것은 바인딩이 넘긴 `Params` 스냅샷뿐이다 ([design-principles.md](design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms).

런타임 튜닝 예:

```bash
ros2 param set /demo_wbc_controller/demo_wbc_controller se3_weight 150.0
ros2 param set /demo_task_controller/demo_task_controller kp_translation '[20.0, 20.0, 30.0]'

# Force-PI grasp (one-shot event)
ros2 service call /demo_task_controller/grasp_command \
    rtc_msgs/srv/GraspCommand "{command: 1, target_force: 2.0}"
```

## DemoWbcController — Kinematic WBC / Dynamic WBC

DemoWbcController의 position-mode tick은 `ComputeTSIDPosition()`이 한 gains 스냅샷으로
**`ComputeWbcCommon`**(공유 stage: state/cache/reference, no solve) → **`ComputeKinematicWbc`**
(arm/hand position) → **`ComputeDynamicWbc`**(hand τ_ff)를 순차 orchestrate한다
(`src/controllers/wbc/compute.cpp`). 두 QP는 같은 Common reference를 독립 소비한다 (decision 5).

- **Kinematic WBC** = CLIK-QP 경로 (`rtc::tsid::ClikReferenceGenerator`). arm SE3 추종(L1) + arm
  redundancy/nullspace posture(L2) + hand posture(L3)를 velocity-level box-QP로 풀어
  `q_ref = q + v_ref·dt`를 낸다. **유일한 position backbone** — CLIK 실패는 critical
  (hold-last → N회 연속 실패 시 `kFallback`). CLIK 계약은 `nq==nv` (reduced wbc 모델)이며,
  비충족 시 `on_configure`가 lifecycle transition을 FAIL시킨다 (DEC-1: integrator fallback 없음).
- **Dynamic WBC** = hand 전용 τ_ff 경로 (`hand_tauff_*` → `kPdFeedforward`). PD position
  backbone 위에 model-based feedforward torque(gravity_comp | tsid_tau)를 overlay. closure/hold
  에서만, `±hand_tauff_max` clamp, opt-in `hand_tauff_enable`.

**Commanded SE3 target** — `RobotTarget{goal_type:"task"}`(arm, device 0)은 base interface의
`SetDeviceTaskTarget`을 거쳐 commanded SE3 slot으로 들어가고, `kRelease`/`kFallback`을 제외한
**모든 phase에서 live override**로 `tcp_goal_`을 그 pose로 갱신한다 (joint posture slot과 독립; CLIK가
추종). 단 `kClosure`/`kHold`/`kRelease`/`kFallback` **진입 시점**에는 직전의 stale
commanded SE3를 clear하므로 (idle 복귀 시 grasp 명령으로 jog되는 것 방지), 해당 phase에서는
**진입 후 새로 도착한** SE3만 재적용된다. `kRelease`/`kFallback`은 자신의 SE3 goal을 소유하여 live
override를 받지 않는다. `goal_type:"joint"`(arm)은 arm posture로 들어간다.
(x,y,z,r,p,y)→SE3 변환은 ZYX(yaw·pitch·roll) 규약.

**Hand joint target** — `RobotTarget{goal_type:"joint"}`(hand, device 1)은 `kRelease`(finger-open
ramp가 hand 점유)/`kFallback`(safety hold)을 제외한 모든 phase에서 매 tick `BuildHandTargetPosture`로
`q_des_target_full_` hand block에 fold되어 CLIK posture term(`clik_kh`)이 추종한다 — phase-entry
edge를 기다리지 않는 live command. (FSM 의 hold/non-hold 판정은 hand τ_ff overlay만 게이트.)

## GraspController (Force-PI, internal only)

Selected via `grasp_controller_type: "force_pi"` in demo controller YAML (default: `"contact_stop"`). Whitelist `{force_pi, contact_stop, none}` — `none` disables all hand intervention (trajectory passes through) while GraspState aggregation/publishing continues; any other string fails `on_configure`.

**FSM**: Idle -> Approaching (position ramp) -> Contact (settle) -> ForceControl (PI + force ramp) -> Holding (anomaly monitor) -> Releasing

PI gain / threshold / slip detection 상수 default 값은 `rtc_controllers/include/.../grasp_types.hpp` 가 SSoT — controller-specific YAML 로 override 가능.

**Grasp detection thresholds (capability-aware).** `grasp_contact_threshold` · `grasp_force_threshold` · `grasp_min_fingertips` 는 데모 컨트롤러 (위 §Controller Table 의 `Demo*`) 전반에서 동일한 키로 노출되며, `devices.<hand>.sensor_layout.has_native_contact` (robot/sim yaml) 에 따라 분기한다 — sensor A (native 접촉 확률 보유) 는 두 threshold 의 AND, sensor B (force-only) 는 force_threshold 단독. 전체 표는 [integrated_bringup/README.md](../integrated_bringup/README.md) 의 "Grasp threshold capability matrix" 섹션.

## ROS2 Topics

**Controller Manager**: switch via `/rtc_cm/switch_controller` (srv, sync, single-active), query via `/rtc_cm/list_controllers` (srv), latched controller-local fault 해제는 `/rtc_cm/reset_fault` (srv, #260 — active 한정·이름 명시 필수, global E-STOP 과 **분리**된 별개 latch); `/rtc_cm/active_controller_name` (Pub, latched — rewire trigger for downstream nodes; namespace 템플릿이 아니라 `RtControllerNode` 가 박아 쓰는 절대 토픽명이다), `/system/estop_status` (Pub). Gain 채널은 더 이상 manager가 소유하지 않으며 컨트롤러별 LifecycleNode parameter로 이관 (위 §Gains 참조).

**Dynamic** (per controller TopicConfig):
- **DeviceBackend-owned (HW/sim ↔ controller boundary, Phase 4 SSoT)** — `state_topic` / `motor_topic` / `sensor_topic` (HW→controller) and `command_topic` (controller→HW/sim) are declared in `devices.<group>.backend:` (sim.yaml/robot.yaml) and owned by `DeviceBackend` impls (`mujoco_native` / `ur_driver_native` / `udp_hand_native`). CM no longer reads device-wire roles from controller YAML.
- **Controller-owned (`<config_key>/` namespace, `PublishNonRtSnapshot`)** — YAML role-mapped: `kRobotTransforms` 하나 (issue #196 Phase 5: publisher 가 없던 `kRobotTarget` / `kDigitalTwinState` 제거 — 선언하면 조용히 죽은 토픽이 됐다). CM은 SPSC snapshot 운반만 담당하며 controller YAML 토픽 퍼블리셔를 만들지 않는다 (issue #138: controller YAML entry 는 전부 controller-owned, `ownership:` field 없음). Phase 4: `kGuiPosition` 폐기 — `/rtc_cm/<group>/joint_states` + `<config_key>/transforms` 로 대체.
- **Controller-owned (no YAML role)** — `GraspState` / `WbcState` / `ToFSnapshot` 은 각 컨트롤러가 `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼로 직접 생성하고 자체 `SeqLock<T>` 로 RT compute → publish thread 전달. `PublishSnapshot` 에서 완전히 분리되어 CM 은 의미를 모름.
- **매 tick Store 계약 (PROC-7, issue #234 P-1)** — CM 은 tick 마다 새 `stamp_ns` 로 snapshot 을 push 하고 publish thread 는 그때마다 SeqLock 을 다시 Load 한다. 따라서 **Store 를 건너뛴 tick 은 "발행 안 함" 이 아니라 "직전 body 를 현재 stamp 로 재발행"** 이다. E-STOP 처럼 제어 법칙을 건너뛰는 tick 도 반드시 Store 하되, 이번 tick 에 계산하지 않은 필드는 명시적으로 무효화한다 — 각 컨트롤러의 `FillEstopPublishState()` 가 그 경로다 (센서 유래 집계는 갱신, Force-PI per-finger 진단은 0, TSID health 는 not-solved, pull 추정은 `StageEstopPullTick` 으로 `valid=false`+감쇠). `pull_estimator.csv` 도 같은 이유로 gap 이 아니라 `valid=0` 행을 남긴다.
- **`PullEstimate` 자기기술 계약 (issue #234 P-5/P-7/P-11)** — `force_inplane[2]` 는 매 tick *관측된* pinch 축에서 재구성되는 기저의 좌표이므로 그 자체로는 tick 간 비교도 역변환도 불가능하다. 따라서 wire 는 같은 tick 의 `plane_normal[3]` + `basis_x[3]` 을 함께 싣고 (e_y = n × e_x 이므로 B 가 완전히 결정됨), 어느 분기가 그 기저를 만들었는지 `basis_source` (`BASIS_REFERENCE` 일 때만 `force_inplane[0]` 이 설정된 reference 방향 성분) 로 밝힌다. 세 벡터의 기준 frame 은 parent message 의 `header.frame_id` — 각 컨트롤러가 `SetOwnedStateFrameId()` 로 system URDF YAML 의 arm root link 를 박아둔다 (robot-agnostic). `invalid_reason` 은 실패한 게이트를 지목해 degenerate normal / required role 이탈 / contact 부족을 구분하며, 이것이 WBC(TSID routing tick 추가 요구로 release·fallback tick 에서 전 contact 무효) 트레이스를 joint·task 와 비교 가능하게 만든다. `contact_mask` 는 힘 합에 실제로 들어간 집합, `touch_mask` 는 축 선택용 |f_obj| 집합 (thumb 포함) 으로 서로 다른 집합이다. `valid` 는 **pull 벡터 유효성 전용** — `friction_utilization` / `slip_risk` 는 per-contact 양이므로 그 게이트 밖에서 평가된다 (active contact 0 이면 false).
- **상호 배타**: `GraspState` 와 `WbcState` — Force-PI 데모(DemoJoint/Task)만 grasp_state, TSID 데모(DemoWbc)만 wbc_state. DemoWbcController는 `<config_key>/<secondary>/wbc_state` (RELIABLE/1) 로 발행 (secondary = `p1a` for ur5e_p1a, `leap` for iiwa7_leap).

**CM per-group JointState**: `/rtc_cm/{group}/joint_states` (RELIABLE, depth 1) -> `rtc_digital_twin` merges -> RViz2

**Hand Driver** (ur5e_p1a instance; generic driver default is `/hand/`, remapped per variant): `/p1a/joint_states`, `/p1a/motor_states`, `/p1a/sensor_states` (Pub); `/p1a/joint_command` (Sub)

**MuJoCo**: `<group.state_topic>` (Pub), `<group.command_topic>` (Sub), `/sim/status` (1Hz)

## Configuration Files

YAML config 트리:

- **Robot-specific bringup** (`integrated_bringup/config/<robot>/`) — `{sim,robot}.yaml` (CM-level: `control_rate`, `initial_controller`, `devices.<group>.backend`, `urdf`, `device_timeout_*`), `mujoco_simulator.yaml` (per-robot overlay), `digital_twin.yaml` (per-robot overlay), `controllers/<config_key>.yaml` (production controller params — controller 당 한 파일의 **flat** 배치. 유일한 하위 디렉토리는 `controllers/mpc/` (`phase_config`·`contact_light`·`contact_rich`) 이며 `demo_wbc_controller.yaml` 이 참조한다. `direct/`·`indirect/` 하위 디렉토리는 `rtc_controllers` 예제 레이아웃이지 여기가 아니며, 지금은 어느 등록 호출도 그 경로를 lookup 하지 않는다)
- **Agnostic defaults** — `rtc_mujoco_sim/config/solver_param.yaml` (MuJoCo solver SSoT), `rtc_digital_twin/config/digital_twin.yaml` (robot-agnostic display defaults), `udp_hand_driver/config/udp_hand_node.yaml` (UDP transport)

**컨트롤러 YAML 이 어디서 로드되는가**: `RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, ...)` 의 2번째 인자가 lookup 경로의 하위 디렉토리를 정한다. 저장소의 **모든** 등록 호출은 현재 `config_subdir` 로 빈 문자열을 넘기므로 (`integrated_bringup` 데모 3종 + CM 테스트) 경로는 `config/<robot>/controllers/<config_key>.yaml` 이 된다. `rtc_controllers/examples/controllers/{direct,indirect}/` 는 **파일만 있는 레퍼런스 레이아웃**이고 이를 lookup 하는 등록 호출은 없다 (#236 S7c 에서 어댑터가 삭제됨). 문서가 예제 레이아웃을 production 레이아웃으로 서술하는 오류가 반복됐으므로 (#213) 새 컨트롤러 추가 시 등록 매크로의 인자를 먼저 확인한다.

- **Reference only** — `rtc_controllers/examples/controllers/{direct,indirect}/*.yaml` (`<robot>` placeholder, production 은 위 robot-specific path 가 owner)

각 YAML 의 정확한 key list 는 파일 자체 + 해당 노드의 `declare_parameter` 호출이 SSoT — 코드에서 grep.
