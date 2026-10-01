# Controllers

컨트롤러가 지켜야 하는 **계약**이다. 제어 법칙별 코어 위치 · 배선된 컨트롤러별 상세 · 결정 근거와 실측 · 게인 / 토픽 / 설정 사용법은 헌법 밖 [docs/controllers.md](../docs/controllers.md) 가 갖는다 (같은 절 이름으로 찾는다).

## Controller Table

- 분류의 축은 클래스가 아니라 **제어 법칙 (알고리즘 코어)** 이다. `rtc_controllers` 는 순수 알고리즘 라이브러리이고 `RTControllerInterface` 구체 구현 (= 바인딩) 은 integration 패키지가 소유한다. 규칙 · 3계층 배치 · 경계 판정의 SSoT 는 [design-principles.md](design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms.
- **`compliance/*` 가 새 법칙의 기준 형태다** — Eigen/span in-out, `Resize()` / `Compute()` 분리, 프레임워크 타입 무지. 새 법칙은 처음부터 그 형태로 쓴다.
- 어떤 법칙이 어디에 있고 어떤 컨트롤러가 배선돼 있는지는 문서가 아니라 코드가 SSoT 다 (`RTC_REGISTER_CONTROLLER` 호출, `rtc_controllers/include/`).

### Joint order & submodel selection

모델을 고르고 재정렬하는 것은 바인딩 몫이다. **device 순서로 형성한 항 (null-space · Coriolis · 관절속도 `q̇`) 은 Pinocchio 순서 행렬과 곱하기 직전 `RtModelHandle::ReorderInput` 으로 1회 gather 한다** — `ν = J·q̇` 의 `q̇` 를 빠뜨리면 모든 수가 유한한 채 감쇠 토크만 틀리고 fault 도 뜨지 않는다 (AP-CTRL-5).

### Unified arm kin&dyn

- 데모 컨트롤러는 arm kinematics (FK · Jacobian) 를 **결합 (arm + hand) actuated 모델 위의 `PinocchioCache`** 에서 얻는다 — non-E-STOP tick 당 `Update(q, v)` 1회. arm-only handle 은 E-STOP TF 경로와 메타데이터로만 쓴다.
- 그 cache 배선 (모델 선택, reorder map, tick 당 state scatter, arm-TCP FK) 은 공유 타입 **`CombinedModelCache` 한 곳** 이 갖는다. 컨트롤러마다 복제하지 않는다.
- RT 진입점 `Compute(state)` 는 **`ReadState` (순수 raw read) → compute model (cache `Update`) → compute control law → `WriteJointCommand` (순수 command write)** 순서를 지킨다. 로그 / 발행용 소비는 `FillLogOutput` / `FillPublishOutput` 으로 분리한다.

## Gains (per-controller ROS 2 parameters)

- 게인 채널은 **ROS 2 parameter API** 다. 각 컨트롤러가 자기 LifecycleNode 에서 `declare_parameter` 로 노출하고, `add_on_set_parameters_callback` 이 SeqLock writer 쪽으로 mutate → Store 한다. RT 경로는 `gains_lock_.Load()` 스냅샷만 본다 (AP-CTRL-1). 게인용 토픽이나 base interface 의 게인 virtual 을 다시 만들지 않는다.
- 컨트롤러 LifecycleNode 는 이름과 namespace 가 둘 다 `config_key` 다 — FQN 은 `/<config_key>/<config_key>`.
- 읽기 전용 cap parameter 는 `ParameterDescriptor::read_only=true` 로 선언한다.
- one-shot 이벤트 (Force-PI grasp) 는 parameter 가 아니라 `grasp_command` srv 다. **상대 이름** 으로 advertise 한다 (`~/` 를 쓰면 이름이 한 번 더 중첩된다). Active controller 만 server 를 띄우고, Inactive 컨트롤러에서 거부한다.
- `mpc_enable` (런타임) 은 YAML `mpc.enabled` 와 AND 다 — YAML 이 false 면 런타임 값은 무시된다.
- `rtc_controllers` 는 게인 채널을 노출하지 않는다 — 노드를 만들지 않는 패키지이고, 코어가 보는 것은 바인딩이 넘긴 `Params` 스냅샷뿐이다.
- 노출되는 parameter 목록은 코드 (`DeclareGainParameters()`) 와 YAML 이 SSoT 다.

## DemoWbcController — Kinematic WBC / Dynamic WBC

- **Kinematic WBC (CLIK-QP) 가 유일한 position backbone 이다.** CLIK 실패는 critical 이다 (hold-last → 연속 실패 시 fallback). CLIK 계약은 `nq==nv` 이며, 충족하지 않으면 `on_configure` 가 FAIL 한다 — integrator fallback 을 두지 않는다.
- **Dynamic WBC** 는 hand 전용 τ_ff overlay 다 — closure / hold 에서만, clamp 아래, opt-in (`hand_tauff_enable`).
- **Commanded SE3 target** 은 `kRelease` / `kFallback` 을 제외한 모든 phase 에서 live override 다. 단 `kClosure` / `kHold` / `kRelease` / `kFallback` **진입 시점** 에 직전의 commanded SE3 를 clear 한다 (idle 복귀 시 묵은 명령으로 jog 되지 않게) — 그 phase 에서는 진입 후 새로 도착한 SE3 만 적용된다. (x,y,z,r,p,y) → SE3 는 ZYX 규약.
- **Hand joint target** 도 `kRelease` / `kFallback` 을 제외한 모든 phase 에서 매 tick 반영되는 live command 다.

## GraspController (Force-PI, internal only)

`grasp_controller_type` 의 whitelist 는 `{force_pi, contact_stop, none}` 이다. `none` 은 손 개입을 모두 끄되 GraspState 집계·발행은 계속한다.

- **whitelist 밖 값의 처리는 출처에 따라 다르다**: YAML 값이면 `LoadConfig` 가 throw 해 `on_configure` FAILURE, 파라미터로 들어오면 거부되고 YAML 값이 유지된다. 파라미터 경로는 capability 게이트도 통과해야 한다.
- **capability 와 모드는 다른 축이다** — `force_pi_grasp` 블록 유무가 capability (PI 컨트롤러가 만들어지는가), 모드가 "지금 어느 법칙이 손을 잡는가" 다. 빌드는 모드를 보지 않는다. 따라서 `grasp_controller_ != nullptr` 은 "force_pi 가 돌고 있다" 의 대용이 **아니다** — 제어 법칙 · diagnostics · `grasp_command` 가 각자 모드를 직접 확인한다.
- **런타임 모드 전환은 손이 quiet 할 때만 수락한다** (contact_stop latch 미engage **그리고** force_pi FSM 이 `kIdle`). 판정의 SSoT 는 `GraspModeChangeRejectReason` 이다. 이미 활성인 모드의 재요청은 항상 수락하고, 표시하는 모드는 요청한 값이 아니라 컨트롤러가 확인한 값이다.
- **quiet gate 는 RT 스레드를 멈출 수 없다** (판정이 한 tick 뒤처진다). 그 창은 compute 쪽 race closure 가 닫는다: 모드가 contact_stop 을 떠난 tick 에 latch 가 걸려 있으면 latch 를 떨구기 **전에** hand goal 을 `hand_hold_position_` 으로 재-seed 한다. 그 경로가 발화하면 결함 신호다.
- **PI 법칙을 돌리지 않는 동안 FSM 은 항상 kIdle 이다** — 모드가 force_pi 가 아닌 매 tick 과 `on_activate` 에서 `Reset()` 하고, phase 미러는 분기 밖에서 매 tick store 한다.
- **힘 피드백은 컨트롤러가 만들어 넘긴다** (`GraspController` 는 필터를 갖지 않는다). 순서는 delta-spike 가드 → **축별** LPF → magnitude 다 — `LPF(‖F‖)` 로 뒤집으면 zero-mean 노이즈가 양의 DC 로 남는다. 가드는 소비자 간 공유하고 cutoff 는 소비자별로 독립이다. 발행하는 `force_magnitude` / `max_force` 는 raw 그대로이고 (BT 계약), 서보가 본 값은 `finger_filtered_force` 다.

**FSM**: Idle → Approaching → Contact → ForceControl → Holding → Releasing

- **Approaching 은 스스로 실패하지 않는다** — 완전히 닫힌 자세에서도 접촉 판정을 계속 돌린다 (접촉은 시간 제한이 아니라 사건이다). 그래서 **kApproaching 도 RELEASE 를 소비한다**, phase 함수 맨 앞에서 — 접촉 없는 grasp 를 빠져나오는 유일한 길이다.
- **ForceControl → Holding 승급은 램프 완료를 요구한다** — dwell 은 `f_desired` 가 `f_target` 에 닿은 뒤에만 돈다. 예산 안에 목표 힘에 못 닿는 물체는 거짓 Holding 대신 kForceControl 에 머물고 호출측 timeout 이 실패로 처리한다.
- **Holding 의 grip tightening 은 rate 다** (`grip_tightening_rate` [N/s]) — per-tick 비율은 `control_rate` 의 함수가 된다. 이 분기는 integrator 동결을 건드리지 않는다 (동결은 `ApplyDeformationGuard` 가 걸고 푸는 닫힌 계약이다).
- **제거된 키가 YAML 에 남아 있으면 `LoadConfig` 가 throw 한다** (`grip_tightening_ratio`) — 로더가 모르는 키를 조용히 무시하므로, 단위가 바뀐 키를 침묵으로 받으면 값이 사라진다.
- **Grasp detection threshold 는 capability-aware 다** — `devices.<hand>.sensor_layout.has_native_contact` 에 따라 분기한다 (native 접촉 보유 = 두 threshold 의 AND, force-only = force threshold 단독).

## ROS2 Topics

- **Controller Manager**: `/rtc_cm/switch_controller` (srv, sync, single-active), `/rtc_cm/list_controllers` (srv), `/rtc_cm/reset_fault` (srv — controller-local fault latch 해제. active 한정 · 이름 명시 필수이며 global E-STOP 과 **분리된** latch 다), `/rtc_cm/active_controller_name` (latched, `RtControllerNode` 가 쓰는 절대 토픽명), `/system/estop_status`.
- **DeviceBackend-owned** — `state_topic` / `motor_topic` / `sensor_topic` / `command_topic` 은 `devices.<group>.backend:` 에서 선언하고 `DeviceBackend` 구현이 소유한다. CM 은 controller YAML 에서 device-wire role 을 읽지 않는다.
- **Controller-owned (YAML role)** — `kRobotTransforms` 하나다. publisher 가 없는 role 을 선언하지 않는다 (조용히 죽은 토픽이 된다).
- **Controller-owned (no YAML role)** — `GraspState` / `WbcState` / `ToFSnapshot` / `PayloadEstimate` 는 각 컨트롤러가 `Setup*Publisher` 헬퍼로 직접 만들고 자체 `SeqLock<T>` 로 넘긴다. CM 은 그 의미를 모른다. `GraspState` 와 `WbcState` 는 상호 배타다.
- **매 tick Store 계약 (PROC-7)** — Store 를 건너뛴 tick 은 "발행 안 함" 이 아니라 "직전 body 를 현재 stamp 로 재발행" 이다. 제어 법칙을 건너뛰는 tick 도 반드시 Store 하고, 계산하지 않은 필드는 `FillEstopPublishState()` 로 명시적으로 무효화한다. CSV 도 gap 이 아니라 `valid=0` 행을 남긴다.
- **CM per-group JointState**: `/rtc_cm/{group}/joint_states` (RELIABLE, depth 1).

**자기기술 계약** — 소비자가 다른 메시지나 시간 정렬 없이 해석할 수 있어야 한다:

- **`PullEstimate`**: `force_inplane` 은 매 tick 재구성되는 기저의 좌표이므로 같은 tick 의 `plane_normal` + `basis_x` 를 함께 싣고, 어느 분기가 그 기저를 만들었는지 `basis_source` 로 밝힌다. 벡터의 기준 frame 은 `header.frame_id` — 각 컨트롤러가 `SetOwnedStateFrameId()` 로 system URDF 의 arm root link 를 넣는다 (robot-agnostic). 실패한 게이트는 `invalid_reason` 이 지목한다. `contact_mask` (힘 합에 들어간 집합) 와 `touch_mask` (축 선택용 집합) 는 다른 집합이다. `valid` 는 pull 벡터 유효성 전용이고 per-contact 양은 그 게이트 밖에서 평가한다.
- **`PayloadEstimate`**: 잔차와 그것을 역산한 payload 를 한 메시지에 싣는다. `residual` 은 device order 이므로 `joint_names` 를 함께 싣는다. 프레임은 둘이다 — `header.frame_id` (wrench 축이 정렬된 프레임) 와 `payload_frame` (wrench 가 작용하는 점). **staleness 는 `header.stamp` 가 아니라 `tick` 으로 판정한다** (stamp 는 publish thread 의 wall clock 이다). 발행 조건은 관측기 활성이다. 관성 파라미터는 4개 (`inertial_mass` / `first_moment` / `com` + rank 진단) 이고 `I` 는 싣지 않는다 — `inertial_rank` 가 차기 전의 `inertial_valid=false` 는 정상 상태다.

## Configuration Files

- **Robot-specific bringup** (`integrated_bringup/config/<robot>/`) — `{sim,robot}.yaml` (CM-level), per-robot overlay, `controllers/<config_key>.yaml`. production controller YAML 은 controller 당 한 파일의 **flat** 배치다.
- **Agnostic defaults** 는 각 `rtc_*` 패키지의 `config/` 가 갖는다.
- **컨트롤러 YAML 이 어디서 로드되는가는 `RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, ...)` 의 인자가 정한다.** `rtc_controllers/examples/controllers/` 는 파일만 있는 레퍼런스 레이아웃이고 production 이 아니다 — 새 컨트롤러를 추가할 때 등록 매크로의 인자를 먼저 확인한다.
- 각 YAML 의 key list 는 파일 자체 + 그 노드의 `declare_parameter` 호출이 SSoT 다.

## 컨트롤러별 계약

배선된 컨트롤러 각각의 상세는 [docs/controllers.md](../docs/controllers.md) §배선된 컨트롤러. 아래는 그중 **깨면 조용히 틀리는** 것이다.

**DemoComplianceController** (task-space admittance):

- SAFE_STOP 은 **래치** 이고 유일한 탈출구는 `/rtc_cm/reset_fault` 다. 재활성화는 래치를 풀지 않으며 활성화 경계에서 미소비 요청을 버린다 (자동 복구 금지). 요청은 atomic flag 로 받아 tick 머리에서 E-STOP 분기보다 앞서 소비한다.
- 홀드 tick 의 진단 행은 FSM 의 **실제 상태** 를 싣는다 — HOLDING 을 강제로 쓰면 래치가 로그에서 사라진다.
- E-STOP · device-invalid · cold-cache 리셋은 **하강 에지** 에서 한다 (매 tick 리셋은 bias 평균을 끝내지 못한다). 하강 에지는 ComputeControl 중간에서 반환하는 홀드 (제어 프레임 대기 · 비유한 Jacobian) 에서도 돌아야 한다 — 그렇지 않으면 렌치 age 가 홀드 내내 멈춘다.
- X_d 를 측정에서 재시드하는 경로는 편차 x̃ 도 함께 붕괴시킨다. 프레임 종류가 바뀐 재시드는 wrench 샘플까지 버린다.
- 램프 재무장은 **쓸 수 있는 wrench 가 생긴 rising edge** (`valid && !stale`) 에서 한다 — bias 재진입 edge 가 아니다.
- **파지한 채로 이 컨트롤러로 전환하지 않는다** — 활성화가 hand target 을 측정값에서 재시드하므로 위치 지령 손은 물체를 놓는다. compliance 를 먼저 활성화하고 그 다음에 손을 닫는다 (절차: verify skill).
- `external_wrench.source` 는 기본값 없는 필수 키다. 그 블록은 코어 파서와 공유하므로 "코어가 안 읽는 잔재" 로 보고 지우지 않는다. 스칼라 `damping` 은 configure 거부다 (여기서 `damping` 은 K_d 6-벡터다).
- 팔의 응답을 정하는 admittance 키는 출하 YAML 에 **명시 등록** 한다 — 코어 default 가 조용히 실기를 재튜닝하지 못하게 한다.
- arm 명령은 관절 tail (`compliance/joint_command_tail.hpp`) 로 바운드한다: clamp 다음에 적분 base 기준 rate 재바운드. 밴드를 뒤집는 margin 은 `on_configure` 가 거부한다 (AP-PROC-9). `command_divergence` 는 이 바인딩에 배선하지 않는다 (근거는 fault 블록 옆 주석이 소유).

**DemoInferenceController** (ONNX 정책 바인딩):

- 입력 텐서의 이름 · 순서 · 폭 · 정규화 · 모델 경로는 전부 YAML 이다. 모든 pose 는 `inference.policy_frame` 기준이고, 물체 레인은 메시지 프레임이 URDF 의 어느 프레임인지 선언한다.
- 힘 feature 를 읽으려면 손의 `sensor_layout.inference_values_per_group` 이 충분히 넓어야 하며, 아니면 configure 거부다 (좁으면 조용히 0 이 된다).
- 정책은 `decimation` tick 마다 1회 돌고 그 사이 action 을 hold 한다 — `control_rate` 를 낮추지 않는다.
- 실패 (입력 unreadable · `Run()` false · 비유한 출력 · closed-chain held · 물체 stale) 는 **전 device 를 래치된 자세로 hold** 한다 (부분 적용 없음). recurrent state 와 reach trigger 는 액션 전체가 수락됐을 때만 전진한다.
- 활성화 경계에서는 나이가 아니라 **새 샘플** 이 기준이다 — 재활성 후 새 입력이 도착할 때까지 hold.
- 명령 tail 의 rate bound 는 **직전 명령** 기준이다 (측정 q 기준이면 서보 지연이 명령을 묶는다). activation · hold 직후에는 측정 q 에서 재시작한다.
- 엔진은 생성자 주입이고 `is_initialized()` 게이트가 configure 를 막는다.

**DemoCatchingController**: 소비 키가 provisional / TBD 이거나 계획기와 oracle 이 함께 켜지면 configure 는 성공하고 activate 만 거부한다 (park).
