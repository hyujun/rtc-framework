# Controllers — 카탈로그 · 게인 · 토픽 · 설정

> 컨트롤러가 지켜야 하는 **계약**은 [agent_docs/controllers.md](../agent_docs/controllers.md) 가 갖는다. 여기는 어떤 컨트롤러가 있는지, 어떻게 켜고 조정하는지를 다룬다. 설계 근거와 실측은 [controllers-rationale.md](reference/controllers-rationale.md). 이름·키·수치의 SSoT 는 코드와 YAML 이다.

RTC 의 컨트롤러는 두 층이다. `rtc_controllers` 는 프레임워크에 의존하지 않는 **제어 법칙 라이브러리**이고, 실제로 도는 `RTControllerInterface` 구현(바인딩)은 `integrated_bringup` 이 소유한다. 층 경계 규칙은 [design-principles.md](../agent_docs/design-principles.md) 를 본다.

## Controller Table

### 배선된 컨트롤러

`integrated_bringup/src/controllers/controller_registration.cpp` 에 등록된 컨트롤러다. 목록의 SSoT 는 그 파일의 `RTC_REGISTER_CONTROLLER*` 사용처다. 상세 동작·키·로그는 [integrated_bringup/README.md](../integrated_bringup/README.md) 에 있다.

| 컨트롤러 | Type | 용도 | 출하 로봇 (`config/<robot>/controllers/`) |
|---|---|---|---|
| `demo_joint_controller` | Position | 관절 궤적 (quintic) + 손. 손 모드 `grasp_controller_type` 선택 | 전부. 새 로봇은 이것으로 bring-up |
| `demo_task_controller` | Position | Cartesian CLIK + 궤적 + 손 | `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` |
| `demo_compliance_controller` | Position | task 컨트롤러 + 외부 렌치 admittance. 렌치는 손 레인의 pull 추정 | `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` |
| `demo_wbc_controller` | Position | TSID/CLIK-QP whole-body + 손 phase FSM. 상세: [DemoWbcController](../integrated_bringup/README.md#demowbccontroller-index-6) | `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` |
| `demo_inference_controller` | Position | ONNX 정책. 입출력 텐서·정규화·모델 경로가 전부 YAML 이라 모델 교체에 재컴파일이 없다 | `ur5e_p1b` |
| `demo_catching_controller` | Position | 날아오는 물체 포획: vision 예측 → 확장 CLIK 팔 + 손 step | `ur5e_p1b` · `iiwa7_leap` |
| `demo_dualarm_controller` | Position | 양손 frame 과제를 QP 하나로 푸는 상체 제어. 상세: [DemoDualArmController](../integrated_bringup/README.md#demodualarmcontroller) | `g1_p1b` |

- 기동 시 활성 컨트롤러는 `config/<robot>/{sim,robot}.yaml` 의 `initial_controller` 가 정한다. 로봇마다 다르니 그 YAML 을 본다.
- YAML 이 없는 로봇에서는 해당 컨트롤러가 인스턴스화되지 않고 건너뛴다 (`_REQUIRING_CONFIG` 등록). `demo_joint_controller` 만 모든 로봇이 YAML 을 싣는다.
- 손 모드(`grasp_controller_type`)는 joint · task 가 노출한다 — 아래 GraspController 절.

### 알고리즘 코어 (`rtc_controllers`)

표의 축은 클래스가 아니라 **제어 법칙**이다. 위 컨트롤러들이 이 코어들을 조합해 쓴다. 규범 문서: [compliance-conventions.md](../rtc_controllers/docs/compliance-conventions.md).

| 제어 법칙 | 코어 헤더 (`rtc_controllers/include/rtc_controllers/`) |
|---|---|
| Joint PD (RNEA + quintic) | `joint/joint_pd_law.hpp` |
| 태스크 속도 (CLIK) | `task/task_vel_law.hpp` + `compliance/differential_ik.hpp` (DLS 역기구학) |
| 태스크 가속도 (OSC) | `task/task_accel_law.hpp` + `compliance/task_dynamics.hpp` |
| 영공간 자세 | `joint/posture_law.hpp` |
| Task impedance | `compliance/impedance_law.hpp` · `inertia_shaping.hpp` · `torque_estop.hpp` |
| Task admittance | `compliance/admittance_integrator.hpp` · `differential_ik.hpp` |
| Cascaded compliance (admittance → impedance) | `compliance/bandwidth_separation.hpp` + 위 두 법칙 |
| 외부 렌치 처리 | `compliance/wrench_pipeline.hpp` · `safety_limiter.hpp` |
| 관절 명령 tail (clamp · rate) | `compliance/joint_command_tail.hpp` |
| Grasp (Force-PI) | `grasp/grasp_controller.hpp` — 컨트롤러가 아니라 상위 컨트롤러가 멤버로 소유 |
| 정책 입출력 · reach gate | `inference/policy_io.hpp` · `inference/reach_gate.hpp` |

법칙이 어느 디렉토리에 있는지는 `ls rtc_controllers/include/rtc_controllers` 로 본다. 새 법칙은 `compliance/*` 처럼 Eigen/span in-out, `Resize()`/`Compute()` 분리, 프레임워크 타입 무지의 형태로 쓴다.

## 컨트롤러 전환·실행

활성 컨트롤러는 항상 하나다. 전환은 서비스로 한다.

```bash
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['demo_task_controller'], \
    deactivate_controllers: ['demo_joint_controller'], strictness: 1, timeout: {sec: 1}}"
ros2 service call /rtc_cm/list_controllers rtc_msgs/srv/ListControllers
```

- 시작부터 특정 컨트롤러로 띄우려면 launch 인자 `initial_controller:=<name>` ([integrated_bringup/README.md](../integrated_bringup/README.md)).
- 전역 E-STOP 중에는 전환이 `ok=false` 로 거부된다.

## Gains

게인은 ROS 2 parameter 다. 각 컨트롤러의 LifecycleNode 가 `declare_parameter` 로 노출하고, 변경은 RT 경로에 lock-free 로 반영된다. 이름과 namespace 가 모두 `config_key` 라 FQN 은 `/<config_key>/<config_key>` 다.

```bash
ros2 param list /demo_wbc_controller/demo_wbc_controller
ros2 param describe /demo_wbc_controller/demo_wbc_controller <param>
ros2 param set /demo_wbc_controller/demo_wbc_controller se3_weight 150.0
ros2 param set /demo_task_controller/demo_task_controller kp_translation '[20.0, 20.0, 30.0]'
```

- 정확한 파라미터 목록은 컨트롤러 소스의 `DeclareGainParameters()` 와 `integrated_bringup/config/<robot>/controllers/<config_key>.yaml` 이 SSoT 다. 서비스는 활성 컨트롤러만 띄운다.
- `*_max_traj_velocity` 같은 상한 값은 read-only 다.
- `demo_dualarm_controller` 는 과제 이름이 들어간 `tasks.<name>.gain_*` 등을 노출한다 ([README](../integrated_bringup/README.md#demodualarmcontroller)).
- WBC 의 `mpc_enable` 은 YAML `mpc.enabled` 와 AND 라, YAML 이 false 면 런타임 1 은 무시된다.
- `rtc_controllers` 자체는 노드가 없어 게인 채널이 없다. 코어는 바인딩이 넘긴 `Params` 스냅샷만 본다.

## GraspController (Force-PI, internal only)

손 파지 법칙이다. 독립 컨트롤러가 아니라 joint · task 컨트롤러가 멤버로 소유한다. 모드는 `grasp_controller_type` 으로 고른다 (기본 `contact_stop`).

| 모드 | 동작 |
|---|---|
| `contact_stop` | 접촉이 감지되면 손을 그 자리에서 멈춘다 (latch) |
| `force_pi` | 목표 힘을 PI 로 추종. FSM: Idle → Approaching → Contact → ForceControl → Holding → Releasing |
| `none` | 손에 개입하지 않고 궤적을 그대로 통과. GraspState 집계·발행은 계속 |

`force_pi` 는 YAML 에 `force_pi_grasp` 블록이 있어야 쓸 수 있다 (블록 = capability, 모드 = 지금 도는 법칙). 블록이 있어도 시작 모드는 `none` 일 수 있다.

**파지 명령** (`force_pi` 모드, 활성 컨트롤러 한정):

```bash
ros2 service call /demo_task_controller/grasp_command \
    rtc_msgs/srv/GraspCommand "{command: 1, target_force: 2.0}"   # 1=GRASP, 2=RELEASE
```

**모드 전환**은 writable 파라미터로 한다. 손이 조용할 때(contact_stop latch 없음, force_pi FSM 이 Idle)만 수락된다. 이미 활성인 모드를 다시 요청하는 것은 항상 수락된다.

```bash
ros2 param set /demo_task_controller/demo_task_controller grasp_controller_type force_pi
```

**거부 사유 확인**
- `ros2 param set` 이 실패하면 출력의 reason 에 사유가 있다 (판정 함수: `GraspModeChangeRejectReason`, `integrated_bringup/include/integrated_bringup/support/demo_shared_config.hpp`). GUI 의 Grasp 탭도 같은 문구를 보여 준다.
- `grasp_command` 는 응답의 `message` 에 사유가 온다. E-STOP 중 · 비활성 컨트롤러 · `force_pi` 가 아닌 모드 · 잘못된 `target_force` 가 대표적이다.
- 허용 값은 `force_pi` · `contact_stop` · `none` 뿐이다. YAML 에서 틀리면 `on_configure` 가 실패하고, 파라미터로 틀리면 거부되고 기존 값이 유지된다.

게인·임계 default 는 `rtc_controllers/include/rtc_controllers/grasp/grasp_types.hpp` 가 SSoT 이고 컨트롤러 YAML 로 override 한다. 튜닝: [grasp_tuning_guide.md](../rtc_controllers/docs/grasp_tuning_guide.md). 접촉 판정 임계(`grasp_contact_threshold` 등)는 손의 `sensor_layout.has_native_contact` 에 따라 갈린다 — [integrated_bringup/README.md](../integrated_bringup/README.md) "Grasp threshold capability matrix". 설계 근거: [controllers-rationale.md](reference/controllers-rationale.md).

## ROS2 Topics

**Controller Manager**
- 서비스: `/rtc_cm/switch_controller` (single-active, sync) · `/rtc_cm/list_controllers` · `/rtc_cm/reset_fault` (래치된 컨트롤러 fault 해제. 활성 컨트롤러 이름을 명시해야 하고 전역 E-STOP 과 별개다).
- 발행: `/rtc_cm/active_controller_name` (latched), `/system/estop_status`, `/rtc_cm/{group}/joint_states` (RELIABLE, depth 1 → `rtc_digital_twin` → RViz2).

**토픽의 소유자**
- 하드웨어/시뮬 경계(`state_topic` · `motor_topic` · `sensor_topic` · `command_topic`)는 `devices.<group>.backend:` 에 선언되고 `DeviceBackend` 가 소유한다. 컨트롤러 YAML 에는 쓰지 않는다.
- 컨트롤러 소유 토픽은 `<config_key>/` namespace 에 발행된다. YAML role 로 선언하는 것은 `kRobotTransforms` 하나다.
- `GraspState` · `WbcState` · `PayloadEstimate` · `ToFSnapshot` 은 각 컨트롤러가 직접 만들어 발행한다. `GraspState` 와 `WbcState` 는 배타적이다 (joint · task = grasp, wbc = wbc).
- `demo_dualarm_controller` 는 과제마다 `<task>/task_goal` (`rtc_msgs/RobotTarget`, `goal_type: task`) 을 구독한다. `header.frame_id` 가 목표의 frame 이다.

**E-STOP 중에도 매 tick 갱신된다.** CM 은 tick 마다 새 stamp 로 snapshot 을 발행하므로 컨트롤러가 갱신을 건너뛰면 직전 값이 새 stamp 로 다시 나간다. 그래서 E-STOP tick 도 상태를 쓰되 계산하지 않은 필드는 무효화한다 (규범: [invariants.md](../agent_docs/invariants.md) PROC-7).

**자기기술 메시지** — `PullEstimate` 는 좌표 기저(`plane_normal` · `basis_x` · `basis_source`)를, `PayloadEstimate` 는 `joint_names` 와 두 프레임(`header.frame_id` = 축 정렬, `payload_frame` = 작용점)을 함께 싣는다. 이 필드 없이는 값을 해석할 수 없으니 구독자는 반드시 읽는다. 신선도는 stamp 가 아니라 `tick` 으로 판단한다.

**Hand driver** (`ur5e_p1a` 예, 기본 `/hand/` 를 variant 마다 remap): `/p1a/joint_states` · `/p1a/motor_states` · `/p1a/sensor_states` (Pub), `/p1a/joint_command` (Sub).
**MuJoCo**: `<group.state_topic>` (Pub), `<group.command_topic>` (Sub), `/sim/status` (1 Hz).

## Configuration Files

- `integrated_bringup/config/<robot>/` — `{sim,robot}.yaml` (`control_rate` · `initial_controller` · `devices.<group>.backend` · `urdf` · `device_timeout_*`), `mujoco_simulator.yaml` · `digital_twin.yaml` (로봇별 overlay), `controllers/<config_key>.yaml` (컨트롤러당 한 파일, `demo_shared.yaml` 공유 블록 포함).
- 로봇 무관 default — `rtc_mujoco_sim/config/solver_param.yaml` (MuJoCo solver), `rtc_digital_twin/config/digital_twin.yaml`, `udp_hand_driver/config/udp_hand_node.yaml`.
- 컨트롤러 YAML 경로는 `RTC_REGISTER_CONTROLLER(config_key, config_subdir, config_package, ...)` 의 인자로 정해진다. 저장소의 등록은 모두 `config_subdir` 이 빈 문자열이라 `config/<robot>/controllers/<config_key>.yaml` 이다. `rtc_controllers/examples/controllers/` 는 참고용 레이아웃일 뿐 어느 등록도 lookup 하지 않는다. 새 컨트롤러를 추가하면 이 인자를 먼저 확인한다.
- 키의 의미·기본값은 YAML 인라인 주석과 `declare_parameter` 가 SSoT 다.
