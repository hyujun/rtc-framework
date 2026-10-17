# demo_controller_gui 사용 안내

`ros2 run integrated_bringup demo_controller_gui` 의 화면 동작을 정리한 사용자 참조입니다. 실행 명령과 컨트롤러별 패널 표는 [README](../README.md#demo_controller_gui) 에 있고, 이 문서는 그 밖의 패널 동작을 다룹니다. 모듈 구조와 구현 근거는 `integrated_bringup/demo_gui/*.py` 의 모듈 docstring 이 갖습니다.


## 컨트롤러 목록 (`/rtc_cm/list_controllers`)


GUI 시작 시:

1. 선택된 프로파일의 *switchable* 키 집합을 오프라인 fallback 으로 라디오에 표시. 상태 라벨에 `(controllers offline)` 접미사.
2. CM 의 `/rtc_cm/list_controllers` 가 응답하면 catalog 가 수신 → Tk 스레드로 marshalling → 라디오 / preset combo / 상태 라벨이 *live* 데이터로 재구성. 5 s 마다 재조회 (controller hot-swap 자동 감지).
3. 응답된 controller 중 그 switchable 집합에 있는 것만 라디오에 노출. 나머지는 catalog 에 보존되지만 GUI 에서 선택 불가.

상태 라벨의 `Active:` 는 라디오 선택이 아니라 CM 이 `/rtc_cm/active_controller_name` 으로 알린 컨트롤러입니다 (아직 못 받았으면 `Active: — (not reported)`). `Switch Controller` 를 누르면 topic 이 그 이름을 알릴 때까지 (이미 활성인 컨트롤러로의 전환은 응답이 올 때까지) `switching → <선택> …` 을 보이고, 거부되거나 호출이 실패하면 `(switch rejected) ` · `(switch failed) ` 를 앞에 붙여 그대로 남은 active 컨트롤러를 보입니다.

**switchable ≠ tunable.** 라디오 집합은 `GAIN_DEFS` 키 (= GUI 에 게인 패널이 있는 컨트롤러) 에 `RobotProfile.extra_switchable_controllers` 를 더한 것입니다 (`RobotProfile.switchable_controllers()`). 두 축이 갈라진 이유는 `demo_inference_controller` 입니다 — 전환은 되지만 ROS 파라미터를 하나도 선언하지 않아 튜닝할 게 없습니다.

필터가 **catalog 가 아니라 프로파일**에서 오는 것이 핵심입니다: CM 은 등록된 컨트롤러를 로봇과 무관하게 인스턴스화하고, 로봇마다 다른 것은 *어느 컨트롤러가 config YAML 을 갖는가* 이며, 그래서 그 사실은 `discovery.py` 의 프로파일에 있습니다. `demo_inference_controller` 는 `config/ur5e_p1b/controllers/` 에만 YAML 이 있으므로 **`--robot ur5e_p1b` 에서만** 라디오에 뜹니다 (`test_robot_profiles.py` 가 설치된 config 트리와 양방향 대조).

**예외 하나 — `/rtc_cm/list_controllers` 응답이 세 프로파일에서 동일하지는 않습니다.** `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` 로 등록된 컨트롤러 (`demo_joint_controller` 를 뺀 전부 — `demo_task` · `demo_wbc` · `demo_compliance` · `demo_inference` · `demo_catching` · `demo_dualarm`) 는 그 프로파일에 YAML 이 없으면 **인스턴스화되지 않고**, 따라서 목록에도 없습니다. 기본값으로 돌 수 없는 컨트롤러이기 때문입니다. 이유는 셋입니다: 정책 파일 경로도 IO 스키마도 없이 기본값으로 도는 정책 컨트롤러는 의미가 없고 (`demo_inference` · `demo_catching`), 과제 frame · 자세군의 관절 이름이 로봇마다 달라 기본값이 있을 수 없고 (`demo_dualarm` — 지금은 `g1_p1b` 만 싣습니다), 팔을 **사슬 하나** (`urdf.sub_models`) 로 전제하는 컨트롤러는 첫 device group 이 tree 인 로봇 (`g1_p1b`) 에서 모델을 만들지 못합니다 (`demo_task` · `demo_wbc` · `demo_compliance`). 그렇다고 bring-up 을 거부하면 그 로봇의 **나머지 컨트롤러까지** 못 뜹니다 (실제로 `ur5e_p1a`·`iiwa7_leap` 이 `demo_inference` 때문에 그 상태였습니다). `demo_joint_controller` 만 종전대로 YAML 이 없으면 내장 기본값으로 돕니다. 게이트는 `test_registered_controllers_have_shipped_config.cpp`.

preset combo 는 라디오 집합에서 **목표를 받지 않는 컨트롤러를 뺀** 것입니다 — preset 은 robot target 을 저장하는데, `joint_goal` 을 구독하지 않는 컨트롤러 앞으로 저장된 preset 은 재생할 곳이 없습니다.

라벨은 controller config_key 를 prettify (`demo_wbc_controller` → `Demo Wbc Controller`) 한 결과를 사용합니다.


## Variable-DOF 동작


`RobotShape` 는 시작 시 UR5e + assm_v1 hand 기본값 (6 arm, 10 hand) 으로 초기화되어 위젯이 즉시 빌드됩니다. CM이 발행하는 `/rtc_cm/{ur5e,hand}/joint_states` (Phase 4 이후 sensor_msgs/JointState) 의 `name` 필드가 GUI 의 RobotShape 와 다르면 *1회 WARN 로그* 후 사용자에게 GUI 재시작을 안내합니다 (현재 sprint 에서는 widget 동적 rebuild 미지원 — option (a)). 다른 robot/hand 에서는 startup 시 `default_ur5e_assm()` 대신 적절한 default factory 를 추가하면 동작합니다.


## 패널 동작


WBC 패널의 `mpc_enable` 토글은 controller 측에서 YAML 의 구조적 `mpc.enabled` flag 와 AND 됩니다. YAML 에서 `mpc.enabled: false` 로 설정된 경우 GUI toggle 은 no-op 입니다 (MPC 스레드가 spawn 되지 않음). 자세한 의미는 `config/ur5e_p1a/controllers/demo_wbc_controller.yaml` 의 `mpc:` 블록 주석 참조.

**Virtual TCP 패널 (Control 탭, Gains 아래):** 선택된 컨트롤러의 `virtual_tcp_mode` · `virtual_tcp_offset` · `virtual_tcp_orientation` 을 읽고 고칩니다 (`demo_joint` · `demo_task` · `demo_compliance` 만 선언 — 그 외 컨트롤러에서는 패널 전체가 비활성). 컨트롤러는 **Inactive 일 때만** set 을 받으므로 ([demo-task-controller.md](demo-task-controller.md#런타임-변경-ros-파라미터)), `Apply VTCP` 는 선택된 컨트롤러가 `/rtc_cm/active_controller_name` 의 active 컨트롤러가 아닐 때만 열리고, 그 메시지를 아직 못 받았으면 닫힙니다. 이 게이트는 안내일 뿐 판정은 컨트롤러가 하며, 거부되면 그 사유가 `refused: ` 뒤에 **그대로** 표시됩니다. Apply 는 세 값을 `set_parameters_atomically` 한 번으로 보내고, 결과와 무관하게 다시 읽어 `current:` 줄을 컨트롤러가 확인한 값으로 바꿉니다. 입력 칸은 읽은 값이 바뀔 때만 다시 채워지므로 거부된 Apply 뒤에도 편집 중인 값이 남습니다. `Load VTCP` 는 캐시를 무시하고 다시 읽어 입력 칸을 덮어씁니다 — `ros2 param set` 이나 재configure 로 바뀐 값은 Load 전에는 보이지 않을 수 있습니다. 쓰는 흐름은 대상 컨트롤러를 선택 → 값 편집 → `Apply VTCP` → `Switch Controller` 이며, 새 값은 그 활성화의 첫 tick 에 hold seed 와 함께 적용됩니다.

**Target 패널 (관절 vs task):** task 목표의 기준 frame 은 패널의 `Frame` 선택기가 정합니다 ([README](../README.md#task-목표의-기준-frame-779)): `base (<tcp_parent>)` (팔의 `root_link`, 기본) 또는 `world (scene)` → `header.frame_id` 는 각각 `tcp_parent` · `scene_world` 입니다. world 는 TF 에 없으므로 GUI 가 CM (`/integrated_rt_controller`) 의 `urdf.extra_frames.scene_world.*` 파라미터를 읽어, TF 로 읽은 base 기준 현재 pose 를 world 로 옮겨 **표시 (End-Effector Pose)·seed** 합니다. frame 을 바꾸면 패널을 다시 seed 합니다 (live pose 가 없으면 입력값 자체를 새 frame 으로 옮긴다) — base 숫자를 world 로 보내면 배치만큼 (출하 받침대 씬은 0.7 m) 어긋나기 때문입니다. 입력값이 선택한 frame 의 것이 아니면 Send 가 task 목표를 보내지 않습니다. `world (scene)` 는 CM 이 쓸 수 있는 world 를 정의할 때만 고를 수 있습니다: 없음 (실기) · `provisional` · 팔 root 가 아닌 parent 이면 base 만 남고 패널 아래 줄이 이유를 말합니다. world 는 active controller 가 바뀔 때 다시 읽습니다. preset 의 task 목표는 파일의 base 값이라 선택기와 무관하게 `tcp_parent` 로 나갑니다 — 패널에는 선택한 frame 으로 옮겨 보이고, 패널에서 저장하면 base 로 바꿔 저장합니다. Dual Arm 탭의 `world` 는 이 world 가 아니라 그 컨트롤러 모델의 frame 입니다 (R6). `demo_joint_controller` 는 관절 목표만, `demo_task_controller` 는 task-space (EE SE3) 목표만 입력 패널이 활성화됩니다. `demo_wbc_controller` 는 **둘 다 활성화** — 암 posture (nullspace reference) 와 commanded EE SE3 jog 를 독립적으로 받기 때문 (`demo_gui/config.py` `DUAL_TARGET_SPACE`). WBC 에서 `Send Command` 는 두 `RobotTarget` (goal_type `joint` + `task`) 을 모두 publish 하며, controller `DeliverTargetMessage` 가 goal_type 별로 라우팅합니다. EE SE3 패널 값은 TF (`virtual_tcp`/`ee_link`) 가 wiring 되어 있어야 current pose 로 seed 됩니다.

**`demo_inference_controller` — 외부 명령 경로가 없는 컨트롤러:** 이 컨트롤러는 라디오에서 선택하고 `Switch Controller` 로 활성화할 수 있지만 (`--robot ur5e_p1b` 한정), **어떤 authoring 위젯도 동작하지 않습니다**. 정책이 팔과 손 명령을 모두 소유하므로 (a) `CreateOwnedTopics` 를 호출하지 않아 `joint_goal` **구독 자체가 없고** — YAML 의 `topics:` 블록은 device 그룹 *순서*를 세우는 용도입니다 — (b) `grasp_command` srv 를 선언하지 않으며 (c) ROS 파라미터가 0개라 게인 패널이 비어 있습니다. 그래서 GUI 는 관절·task 목표 패널과 `▶ Grasp` / `■ Release` 를 **비활성화하고 사유를 표시**합니다 (`demo_gui/config.py` `NO_EXTERNAL_COMMAND_CONTROLLERS`). 구독자가 없는 토픽으로의 publish 는 조용히 성공하므로, 버튼을 살려두면 "Sent hand cmd" 로그만 남고 로봇은 안 움직이는 상태와 구별되지 않습니다 — `Send Command` 와 `Send Preset` 의 hand 절반도 같은 이유로 차단됩니다. 관절 상태 readout 은 CM 소유 토픽(`/rtc_cm/<group>/joint_states`)이라 정상 동작하고, 진단(hold 사유·reach gate)은 토픽이 아니라 세션 CSV (`inference_diag.csv`) 로만 나옵니다.

**Task frame 게이트:** task 목표는 active controller 의 **제어 frame 이 확정된 뒤에만** 입력·발행할 수 있습니다. 컨트롤러 전환 직후에는 `TaskFrameSelector` 가 미확정 상태라 task target entry / step 버튼이 `disabled` 이고, EE pose 표시는 `—` 입니다 (이전 컨트롤러의 pose 를 라이브처럼 보여주지 않기 위해). frame 이 확정되면 그 시점의 실제 pose 로 **1회** seed 되고 패널이 활성화됩니다 — 전환 시점에 seed 하면 아직 이전 컨트롤러의 pose 라 "표시된 pose 를 그대로 target 으로 보냈는데 로봇이 움직이는" 결함이 됩니다. `Send Preset` 의 자동 task publish 와 preset 저장도 같은 게이트를 통과합니다. 이 게이트는 컨트롤러 측 계약의 나머지 절반이다 — `ApplyPendingTarget` 이 외부 목표를 컨트롤러의 *의도된* frame 으로 태깅할 수 있는 근거가 "미확정 창에서는 authoring 자체가 불가능하다" 이기 때문.



## Preset 단축키 (close / open 토글)

Preset 패널의 `Toggle Close` / `Open` 콤보에 고른 두 preset 을 키 하나로 번갈아 보냅니다 (기본 `Space`, `Set Key` 로 변경 — 캡처 창에서 `Esc` 는 취소). 선택은 preset 파일과 같은 디렉토리의 `demo_gui_settings_<hand_group>.json` 에 저장됩니다.

- preset 의 `controller` 가 active 가 아니면 먼저 전환하고, 사전 조건이 하나라도 실패하면 아무것도 발행하지 않고 사유를 로그에 남깁니다 (E-STOP, DoF 불일치, 미확정 task frame 등).
- 발행에 성공했을 때만 다음 역할 (close → open) 로 넘어갑니다. 거부된 close 뒤에 누르면 다시 close 입니다.
- 텍스트 입력 위젯에 포커스가 있으면 단축키를 무시합니다. 키를 누르고 있는 auto-repeat 는 한 번으로 셉니다.
- 토글 도중 예외가 나면 잠금을 풀고 `Preset toggle (...) aborted` 를 error 로그에 남깁니다. 재시작 없이 다시 누를 수 있습니다.
- 두 콤보에 같은 preset 을 고르면 패널에 경고가 표시됩니다.


## Grasp / Release 버튼


- **`demo_inference_controller`**: 버튼 **항상 비활성**. `grasp_command` srv 자체가 없어 누르면 거부 메시지조차 돌아오지 않으므로, 파라미터 미조회 시의 fail-open 과 달리 UNKNOWN 에서도 열리지 않습니다 — 그 fail-open 은 "컨트롤러가 받아줄 명령을 GUI 가 막지 않는다" 는 규칙인데, 여기서는 막을 대상이 없습니다.
- **`demo_joint_controller` / `demo_task_controller`**: 활성 모드가 `force_pi` 일 때만 동작. `"contact_stop"` / `"none"` 모드에서는 srv 가 **사유와 함께 거부** (`ok=false` + `message`, `GraspCommandRejectReason` 가 SSoT). GUI 는 두 컨트롤러가 노출하는 `grasp_controller_type` 파라미터를 읽어 버튼 옆에 현재 모드를 표시하고, force_pi 가 아니면 버튼을 비활성화합니다 (파라미터 미조회 시에는 fail-open). 그 파라미터를 바꾸는 법은 [demo-shared-params.md](demo-shared-params.md#grasp-mode-전환-joint--task--compliance) 에 있습니다.
- **`demo_wbc_controller`**: `grasp_controller_type` 무관 — WBC 는 자체 6-state FSM (slots 2 & 5 reserved) 으로 GraspCommand 를 직접 처리 (lifecycle.cpp 의 `grasp_command_srv_`). GRASP 명령은 `kApproach` 진입, RELEASE 는 어떤 비-terminal phase 에서도 `kRelease` 로 즉시 preempt (Approach/Closure/Hold 중 GUI 로 RELEASE 누르면 즉시 반응). phase 표시기가 WbcPhase enum 라벨로 갱신됩니다.

phase 표시기는 active controller 가 force_pi grasp publisher 인지 WBC publisher 인지에 따라 `GRASP_PHASE_NAMES` (6 상태) 또는 `WBC_PHASE_NAMES` (8 슬롯, 6 reachable — slot 2 PRE-GRASP / slot 5 RETREAT 는 deprecated reserved) 라벨 표를 자동 선택합니다 — 두 publisher 가 GUI 에 동시에 구독되지만 active 한 쪽만 발행하므로 자동 분기.





## Pull Force Estimate 패널

`GraspState` / `WbcState` 의 `pull` (`PullEstimate`) 을 표시합니다. joint / task 는 `grasp_state` 만, WBC 는 `wbc_state` 만 발행하고 구독은 active 컨트롤러 네임스페이스에 바인딩되므로 source 는 항상 하나입니다. 마지막 수신이 0.5 s 를 넘으면 stale 입니다.

배지는 위에서부터 첫 번째로 맞는 조건을 보입니다.

| 순위 | 조건 | 배지 | 부가 설명 |
|---|---|---|---|
| 1 | E-STOP | `UNAVAILABLE` | `E-STOP active` |
| 2 | stale · 미수신 · 컨트롤러 전환 직후 | `STALE` / `UNAVAILABLE` | 마지막 갱신 경과 시간 |
| 3 | fresh + estimator 미wiring | `UNAVAILABLE` | `estimator disabled` |
| 4 | fresh + `slip_risk` | `SLIP RISK` | — |
| 5 | fresh + `valid` | `NO SLIP RISK` | — |
| 6 | fresh + `!valid` | `UNKNOWN` | `invalid_reason` 라벨 (not initialized / degenerate normal / required contact lost / too few contacts) |

`estimator disabled` 는 `pull_estimator.enabled: false` 이거나 FK 로 풀린 tip link 가 없을 때입니다 (컨트롤러 로그 `[pull_estimator] disabled — ...` 와 같은 문구).

배지가 `STALE` / `UNAVAILABLE` 이면 값은 모두 `--` 로 비웁니다. 그렇지 않을 때 그룹별 표시 조건은 다음과 같습니다.

| 그룹 | 표시 조건 | 필드 |
|---|---|---|
| Pull vector | `valid` | `magnitude` · `directional` · `leakage` · `force[3]` · `force_inplane[2]` · `basis` · `basis_x[3]` |
| Contacts (partial-contact 진단) | 항상 | `friction_utilization` · `contact_mask` · `touch_mask` · `any_saturated` · `baseline_applied` · `plane_normal[3]` |
| Source | 항상 | publisher 라벨 · age · `header.frame_id` |

패널은 5 Hz 로 다시 그리지만 프레임 사이에 도착한 샘플을 접어 `friction_utilization` 은 최댓값, `slip_risk` 는 OR 로 보이므로 순간적인 slip spike 도 한 프레임은 보입니다. 구현 근거는 `demo_gui/pull.py` docstring 입니다.


## 다중 frame 컨트롤러 (`demo_dualarm_controller`, `--robot g1_p1b`)

이 컨트롤러는 그 YAML 을 싣는 profile 에서만 GUI 에 나타납니다. GUI 가 뜰 때 `config/<robot>/controllers/demo_dualarm_controller.yaml` 을 읽어 과제 · 목표 frame · 자세군을 얻고, 화면의 세 부분을 거기서 만듭니다 — 과제 이름이나 개수는 GUI 코드에 없습니다.

- **Gains 패널** (Control 탭): README 의 컨트롤러별 패널 표의 행이 YAML 의 과제 · 자세군마다 하나씩 생깁니다 (`config.register_scalar_gain_schema`). `Load Gain` · `Apply Gains` 는 다른 컨트롤러와 같습니다. gain 이 [0, 1/dt] 밖이면 컨트롤러가 그 Apply **전체**를 거부하고 (`set_parameters_atomically`) 사유는 GUI 로그에 남습니다. `Currently Applied` 는 컨트롤러가 set 을 받아들인 뒤에만 바뀝니다 (모든 컨트롤러 공통 — 거부된 값이 적용된 것처럼 남지 않습니다).
- **Dual Arm 탭 — 과제 목표**: 과제마다 패널 하나. `Goal stated in` 에서 목표를 적을 frame (그 과제의 base frame + `clik.target_frames`) 을 고르고, 여섯 값 (m · deg, ZYX) 을 적어 `Send goal` 로 **한 번** 보냅니다 (`<task>/task_goal`). 컨트롤러가 활성일 때만 보냅니다.
  - `Measured` 열은 컨트롤러 TF 의 측정 pose 이고, 그 pose 가 표현된 frame (transforms 의 부모) 이 열 제목에 나옵니다. **목표 frame 이 그 frame 과 같을 때만** 목표 칸이 측정 pose 로 채워지고 `Copy measured → goal` 이 켜집니다. 다른 frame 을 고르면 칸을 비우고 버튼을 끕니다 — 두 frame 사이의 변환은 어느 토픽에도 없고, GUI 는 추측하지 않습니다. 그 frame 의 목표는 직접 적습니다 (`dualarm_diag.csv` 의 `<task>_ref_*` 가 그 과제의 base frame 기준 값입니다).
  - 과제의 base frame 이 아닌 frame 으로 적은 목표는 컨트롤러가 **적용하는 tick 에 한 번** base frame 으로 바꿔 그 값을 붙듭니다. 그 뒤 base frame 이 움직이면 (예: 다른 과제가 몸통을 돌리면) 적은 frame 에서 본 pose 는 목표와 달라집니다 — 패널이 이 경우를 한 줄로 알립니다.
  - pitch 가 ±90° 의 5° 안이면 패널에 한 줄이 뜹니다: ZYX 에서 roll 과 yaw 가 거의 같은 축을 돌아 step 버튼 둘이 같은 동작을 합니다. 목표 형식은 컨트롤러의 것이라 GUI 에서 바꾸지 않습니다.
- **Dual Arm 탭 — 풀이 상태**: 이 컨트롤러는 상태 토픽이 없어, `Controller` 줄은 `/rtc_cm/list_controllers` (활성 여부 · fault latch · mailbox 계수, 5 s 주기) 에서, 나머지는 **세션의 `dualarm_diag.csv` 마지막 행** (2 Hz) 에서 읽습니다 (`list_controllers` 가 15 s 넘게 답하지 않으면 그 줄도 비웁니다): tick 이 풀었는지 · hold 사유, 수렴 · 반복 · 풀이 시간, 한계 발동, fault 원인과 연속 실패, 과제별 오차 · 진행 중인 목표 번호 · 받은 / 거부한 / 버린 목표 수.
  - 세션은 `--session` → `$RTC_SESSION_DIR` → logging root 에서 **이 로그가 있는** 가장 새 세션 순으로 찾습니다 (그런 세션이 없으면 가장 새 세션). 마지막은 매번 다시 찾으므로 GUI 를 bringup 보다 먼저 띄워도 되고, 다른 도구가 더 새 세션 폴더를 만들어도 판독이 그쪽으로 넘어가지 않습니다.
  - **GUI 가 컨트롤러와 같은 머신에서 그 세션 디렉토리를 읽을 수 있어야 합니다.** 파일이 없으면 사유를, 1 s 넘게 자라지 않으면 "not running, values withheld" 를 적고 값은 비웁니다 — 멈춘 컨트롤러의 마지막 행을 현재 상태처럼 두지 않습니다.
- **Grasp 탭**의 `▶ Grasp` / `■ Release` 는 이 컨트롤러에서 꺼집니다 — `grasp_command` 서비스가 없습니다 (`config.py` `NO_GRASP_COMMAND_CONTROLLERS`).
- **preset · 단축키 설정 파일**은 `hand_presets_g1_p1b.json` · `demo_gui_settings_g1_p1b.json` 입니다 (`RobotProfile.preset_scope`). 다른 profile 은 hand group 이름 그대로인데, 이 profile 은 손이 `ur5e_p1b` 와 같고 몸과 컨트롤러 집합이 달라 파일을 나눕니다 — preset 은 robot target 과 컨트롤러 이름도 저장합니다.
- **자세 · 손 목표**는 Control 탭 그대로입니다: `Joint Target` 이 첫 군의 관절 전부 (limb 마다 한 열), `Hand Motor Target` 이 손 관절 전부를 싣고 `Send Command` 가 둘을 보냅니다. Control 탭의 `Task Target` 은 이 컨트롤러에서 꺼져 있습니다 (군 토픽의 task 목표는 컨트롤러가 거부합니다).


## 기타 기능


- 관절/태스크 공간 타겟 설정 (joint controller 는 6 axis, task controller 는 X/Y/Z + Roll/Pitch/Yaw)
- 핸드 모터 슬라이더 (Thumb / Index / Middle / Ring — finger 그루핑은 motor name prefix 로 자동 추론)
- E-STOP 상태 (`/system/estop_status`), 실시간 TCP/관절 위치 표시
- 헤더의 **Clear E-STOP** / **Reset fault** (S9a) — 모든 탭 공용. Clear E-STOP 은 두 번 부른다: 빈 `reason_ack` 로 `/rtc_cm/clear_estop` 의 거부 메시지에서 latch 사유를 읽어 보여주고, 운영자가 확인하면 그 사유를 그대로 echo 한다 (사유는 어느 토픽에도 없다; 읽을 수 없는 응답이면 추측하지 않고 멈춘다). Reset fault 는 활성 컨트롤러의 `Name()` (`list_controllers` 의 `name`, config key 아님) 으로 `/rtc_cm/reset_fault` 를 부른다. 둘 다 응답을 그대로 표시하며 다른 latch 가 남았다는 문구가 있으면 경고색이다. 어느 것도 무장을 되살리지 않는다 — 구현 [demo_gui/latch_clear.py](../integrated_bringup/demo_gui/latch_clear.py)
- 핸드 자세 프리셋 저장/로드 (JSON)
- 센서 캘리브레이션 (`/<hand_group>/calibration/command` — `--robot` 프로파일의 hand group 으로 파생: p1a→`/p1a/`, p1b→`/p1b/`)
