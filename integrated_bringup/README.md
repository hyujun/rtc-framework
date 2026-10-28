# integrated_bringup

로봇별 launch · 설정 · 데모 컨트롤러와 GUI 를 묶어 실로봇과 MuJoCo sim 을 띄우는 통합 패키지입니다.

**유형**: 노드 패키지 (통합)

## 개요

### 구성

- **launch**: 로봇마다 실로봇용 `robot_<robot>.launch.py` 와 MuJoCo 용 `sim_<robot>.launch.py` (아래 매트릭스). CPU 격리, DDS 스레드 핀닝, 세션 디렉토리 생성도 launch 가 합니다.
- **설정**: `config/<robot>/` 에 로봇별 `_base.yaml` (mode 공통) 과 `robot.yaml` / `sim.yaml` (mode 차이), 컨트롤러별 `controllers/*.yaml`. 손 설정만 있는 `config/b7_p2/` 는 `udp_hand.launch.py` 의 `params_file` 로 띄웁니다.
- **데모 컨트롤러**: `DemoJointController` · `DemoTaskController` · `DemoComplianceController` · `DemoWbcController` (TSID + MPC) · `DemoDualArmController` · `DemoCatchingController` · `DemoInferenceController` (ONNX 정책). 등록 목록은 `src/controllers/controller_registration.cpp`.
- **GUI**: `demo_controller_gui` (컨트롤러 전환 · 게인 튜닝 · 상태 모니터링) 와 `motion_editor_gui`.
- **세션 로그**: `logging_data/YYMMDD_HHMM/` (`timing/`, `device/`, `controllers/<config_key>/`) 에 자동 저장되고 보관 수는 `max_log_sessions` (기본 10) 입니다.

### 로봇 × launch 매트릭스

| 로봇 (`config/<robot>/`) | 실로봇 launch | sim launch | 기본 시작 컨트롤러 | 컨트롤러 YAML 이 있는 데모 컨트롤러 |
|---|---|---|---|---|
| `ur5e_p1a` | `robot_ur5e_p1a.launch.py` | `sim_ur5e_p1a.launch.py` | `demo_wbc_controller` | joint · task · compliance · wbc |
| `ur5e_p1b` | `robot_ur5e_p1b.launch.py` | `sim_ur5e_p1b.launch.py` | `demo_joint_controller` | joint · task · compliance · wbc · catching · inference |
| `iiwa7_leap` | — | `sim_iiwa7_leap.launch.py` | `demo_wbc_controller` | joint · task · compliance · wbc · catching |
| `g1_p1b` | — | `sim_g1_p1b.launch.py` | `demo_joint_controller` | joint · dualarm |

- 실로봇은 UR5e 두 종 (`ur5e_p1a`, `ur5e_p1b`) 만 지원합니다. `iiwa7_leap` 과 `g1_p1b` 는 sim 전용입니다.
- YAML 이 없는 로봇에서는 `demo_joint_controller` 외 컨트롤러가 인스턴스화되지 않고 목록에도 나타나지 않습니다.
- 시작 컨트롤러는 `initial_controller:=<key>` 로 바꿉니다 (sim launch 공통 인자).

### 로봇 vs 시뮬레이션 모드 비교

| 항목 | Robot 모드 | Sim 모드 |
|---|---|---|
| 관절 상태 소스 | UR 드라이버 | MuJoCo 시뮬레이터 |
| 핸드 소스 | UDP (실제 하드웨어) | MuJoCo 에코백 (선택) |
| 커맨드 대상 | UR5e 로봇 | MuJoCo 물리 엔진 |
| CPU 격리 | Tier 1+2 (`--robot`) | Tier 1 (`--sim`) |
| DDS 핀닝 | UR 드라이버 + RT 컨트롤러 | MuJoCo 시뮬레이터 |
| 설정 | `ur5e_p1a/_base.yaml` + `robot.yaml` | `ur5e_p1a/_base.yaml` + `sim.yaml` + `mujoco_simulator.yaml` |

## 빠른 시작

빌드와 환경 설정은 [루트 README](../README.md#빠른-시작) 를 따릅니다. 빌드한 터미널에서 sim 을 띄웁니다.

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py
# viewer 없이: enable_viewer:=false
```

다른 터미널에서 GUI 를 엽니다. 로봇을 바꿨으면 `--robot` 을 맞춥니다 (`ur5e_p1a` | `ur5e_p1b` | `iiwa7_leap` | `g1_p1b`).

```bash
ros2 run integrated_bringup demo_controller_gui                  # ur5e_p1a
ros2 run integrated_bringup demo_controller_gui --robot ur5e_p1b
```

GUI 의 컨트롤러 라디오에서 컨트롤러를 고르고 `Switch Controller` 를 누르면 전환됩니다. 로그는 `logging_data/` 의 가장 새 세션 디렉토리에 쌓입니다.

## 인터페이스

### Launch 파일

`launch/` 에 launch 파일이 여섯 있다. 전부 같은 뼈대다 — 세션 디렉토리 (`logging_data/YYMMDD_HHMM/`) 를 만들고, CPU 격리 (`cpu_shield.sh`) 를 켜고, 하드웨어 쪽 (UR 드라이버 + hand UDP 노드, 또는 MuJoCo 시뮬레이터) 과 RT 컨트롤러 노드 `integrated_rt_controller` 를 띄워 lifecycle 을 순서대로 활성화한다. CycloneDDS 설정은 [rtc_controller_manager README](../rtc_controller_manager/README.md#설정-파일) 가 갖는다. 빌드는 [루트 README](../README.md#빠른-시작) 를 따른다.

| launch 파일 | 띄우는 것 | 명령 |
|---|---|---|
| `robot_ur5e_p1a.launch.py` | 실기 UR5e + p1a 손 (UDP). `robot_ip` 기본 `192.168.1.10` | `ros2 launch integrated_bringup robot_ur5e_p1a.launch.py robot_ip:=192.168.1.10` |
| `robot_ur5e_p1b.launch.py` | 실기 UR5e + proto_1b 폐쇄 체인 손. 인자는 p1a 와 같고 `robot_ip` 기본만 `192.168.0.3` | `ros2 launch integrated_bringup robot_ur5e_p1b.launch.py robot_ip:=192.168.0.3` |
| `sim_ur5e_p1a.launch.py` | MuJoCo sim 의 UR5e + p1a 손. 공 발사 없음 | `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py` |
| `sim_ur5e_p1b.launch.py` | MuJoCo sim 의 UR5e + proto_1b 손. 공 발사 · `object_pool` · 포구 시험의 기본 launch | `ros2 launch integrated_bringup sim_ur5e_p1b.launch.py enable_viewer:=false` |
| `sim_iiwa7_leap.launch.py` | MuJoCo sim 의 iiwa7 + LEAP 손 (손 sensor 스택 없음). 공 발사 있음 | `ros2 launch integrated_bringup sim_iiwa7_leap.launch.py enable_viewer:=false enable_mpc:=true` |
| `sim_g1_p1b.launch.py` | MuJoCo sim 의 Unitree G1 상체 (fixed base) + proto_1b 오른손. 실기 launch 없음 | `ros2 launch integrated_bringup sim_g1_p1b.launch.py use_cpu_affinity:=false` |

실기 launch 의 `use_mock_hardware:=true` 는 모의 하드웨어로 띄운다 (UR 드라이버의 mock). sudo 없는 머신은 sim 에 `use_cpu_affinity:=false` 를 준다. launch 가 선언하지 않은 인자는 에러 없이 무시된다 — 이름을 `ros2 launch <pkg> <file> --show-args` 로 확인한다.

#### 실기 launch 인자 (`robot_ur5e_p1a` · `robot_ur5e_p1b`)

| 인자 | 기본값 | 설명 |
|---|---|---|
| `robot_ip` | p1a `192.168.1.10` · p1b `192.168.0.3` | UR 로봇 IP |
| `use_mock_hardware` | `false` | 모의 하드웨어 사용 |
| `use_fake_hardware` | `false` | Humble 용 별칭 |
| `use_cpu_affinity` | `true` | CPU 격리 + DDS 핀닝 |
| `max_log_sessions` | `10` (`config/ur5e_p1a/_base.yaml` 에서 읽음) | 세션 폴더 최대 보관 수. launch 와 RT 노드가 같은 값을 본다 |
| `kinematics_params_file` | `~/ur5e_calibration.yaml` | UR 공장 calibration YAML. 아래 [UR 캘리브레이션](#ur-캘리브레이션-kinematics_params_file) |
| `headless_mode` | `true` | Teach Pendant 의 External Control play 없이 제어 |
| `reverse_ip` | `0.0.0.0` | UR 컨트롤러가 PC 로 reverse connection 할 PC IP. multi-NIC 이면 명시 |
| `launch_dashboard_client` | `true` | UR Dashboard client 실행. dashboard port 가 막혔으면 `false` |
| `controller_spawner_timeout` | `10` | controller spawner 의 load/activate 대기 (초) |
| `activate_joint_controller` | `true` | `initial_joint_controller` 를 시작 시 active 로. `false` 면 load 만 |
| `initial_joint_controller` | `forward_position_controller` | UR 드라이버가 처음 활성화할 joint controller |
| `enable_mpc` | `""` | CPU layout profile 선택. `false` 면 MPC 코어를 반납하므로 MPC 가 켜진 WBC config 는 활성화를 거부한다. YAML `mpc.enabled` 는 바꾸지 않는다 |
| `enable_tracing` | `false` | LTTng trace 캡처. 1회 `./install.sh --tracing` 필요. 변환은 `repo_scripts/scripts/timeline.sh` |
| `trace_session_name` · `trace_events_ust` · `trace_events_kernel` | `""` · `""` · sched/irq 이벤트 | trace 세션 이름과 UST · 커널 이벤트 (콤마 구분) |
| `sil_mode` | hand YAML 의 값 (없으면 `off`) | `off` / `loopmodel` / `firmware`. [udp_hand_driver README](../udp_hand_driver/README.md) |

#### sim launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `model_path` | `""` (YAML) | MuJoCo scene 경로 오버라이드 |
| `enable_viewer` | `""` (YAML) | MuJoCo 뷰어 |
| `viewer_input_enabled` | `""` (YAML) | `false` 면 뷰어가 world 를 바꾸는 키를 무시한다 ([뷰어 단축키](../rtc_mujoco_sim/README.md#뷰어-단축키)) |
| `sync_timeout_ms` · `max_rtf` · `use_yaml_servo_gains` | `""` (YAML) | sync 타임아웃 (ms) · 최대 실시간 배율 (0 = 무제한) · YAML vs XML 서보 게인 |
| `use_cpu_affinity` | `true` | Tier 1 CPU 격리 + MuJoCo 핀닝 |
| `max_log_sessions` | `10` (그 profile 의 YAML) | 세션 폴더 최대 보관 수 |
| `initial_controller` | `""` (YAML) | 시작 컨트롤러. 기본은 `ur5e_p1a` · `iiwa7_leap` 이 `demo_wbc_controller`, `ur5e_p1b` · `g1_p1b` 가 `demo_joint_controller`. 기동 완료 신호는 `RtControllerNode reached Active state` 로그 |
| `enable_mpc` | `""` (YAML) | `demo_wbc_controller` 의 `mpc.enabled` 를 launch 시점에 덮는다. `sim_iiwa7_leap` 은 `false` 가 YAML 기본, `sim_g1_p1b` 는 CPU layout 만 고르고 기본 `false` |
| `mpc_engine` | `""` (YAML) | `mock` / `handler` (`demo_wbc_controller` 의 MPC 엔진). `sim_g1_p1b` 에는 없다 |
| `sim_overlay` | `""` | 출하 씬을 건드리지 않고 바꾸는 overlay (`sim_ur5e_p1b` · `sim_iiwa7_leap` · `sim_g1_p1b`). [Scene overlay](#scene-overlay-sim_overlay--출하-씬을-건드리지-않고-바꾸기) |
| `sim_lanes` | `false` | clock 위상 · 공 접촉 측정 lane 을 켜고 `<session>/sim/` 에 쓴다 (`sim_ur5e_p1b` · `sim_iiwa7_leap` · `sim_g1_p1b`) |
| `catch_search_mode` · `catch_segment_mode` | `""` (출하값) | 포구 탐색 `grid` \| `nlp` 와 구간 계획기 `closed_form` \| `mpc` \| `mpc_docking` 선택 (`sim_ur5e_p1b` · `sim_iiwa7_leap`) |
| `object_pool` · `object` · `object_seed` | `""` (YAML) | `object_pool` 스폰 설정 오버라이드. **`sim_ur5e_p1b` 에만** 있다 |
| `enable_tracing` · `trace_*` | 실기와 같다 | LTTng trace |

빈 문자열 기본값은 YAML 의 값을 쓴다는 뜻이고, 값을 주면 YAML 을 덮는다. `sim_ur5e_p1b` 의 기본 scene 은 `scene_pedestal.xml` (0.7 m 받침대) 이다. 작업대 씬은 `model_path:=package://hand_description/robots/ur5e_p1b/mjcf/scene_with_table.xml` 로 고른다. 공 발사는 `ur5e_p1b` 와 `iiwa7_leap` 의 `mujoco_simulator.yaml` `projectile_ball:` 블록이 켠다 (launch 인자가 아니다). 발사 서비스 `/sim/launch_ball` · `/sim/launch_ball_at` 의 계약은 [LaunchBall.srv](../rtc_msgs/srv/LaunchBall.srv) 와 [rtc_mujoco_sim README](../rtc_mujoco_sim/README.md#projectile-ball-발사-공) 가 갖는다.

공 추정기 (`ball_perception_sim` 의 `sim_estimator_node`) 는 ball_perception 의 별도 colcon workspace 에서 띄우며, 그 workspace 의 `install/setup.bash` 를 추가로 source 해야 한다. 실행 명령은 [포구 sim 시행 안내](../docs/catching-sim-trials.md#1-한-번-던지기) 에 있다.

#### UR 캘리브레이션 (`kinematics_params_file`)

각 UR5e 는 공장 출하 시 관절 kinematics 에 로봇별 오차가 있다. `ur_calibration` 으로 실측 YAML 을 추출해 `kinematics_params_file` 로 넘기면 UR 드라이버가 publish 하는 `robot_description` / TF 에 반영된다.

```bash
# 1) 로봇별 calibration YAML 추출 (1회, robot_ip 대상)
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.10 \
  target_filename:=/absolute/path/to/ur5e_calibration.yaml

# 2) bringup 에 전달
ros2 launch integrated_bringup robot_ur5e_p1b.launch.py \
  robot_ip:=192.168.1.10 \
  kinematics_params_file:=/absolute/path/to/ur5e_calibration.yaml
```

`kinematics_params_file` 은 UR 드라이버의 `robot_description` / TF 에만 영향을 준다. RTC 컨트롤러의 Pinocchio 모델은 `config/<variant>/_base.yaml` 의 `urdf:` 경로를 따로 읽으므로 제어 모델에는 반영되지 않는다. `ur_calibration` 은 `package.xml` 의 `<exec_depend>` 라 `rosdep` (`ros-jazzy-ur-calibration`) 이 설치한다.

#### `sim_g1_p1b` — Unitree G1 상체 + proto_1b 오른손

```bash
ros2 launch integrated_bringup sim_g1_p1b.launch.py                                              # viewer
ros2 launch integrated_bringup sim_g1_p1b.launch.py enable_viewer:=false use_cpu_affinity:=false   # headless · sudo 없음
```

- device group 이 둘이다: `g1` (waist 3 + 왼팔 7 + 오른팔 7 = 17 관절, 이 순서) 과 `p1b` (손 10 관절). 모델 (URDF · MJCF) 은 `hand_description` 패키지의 `robots/unitree_g1_p1b/` 를 load 만 한다.
- 컨트롤러는 `demo_joint_controller` (기본 활성) 와 `demo_dualarm_controller` 둘이다. 팔을 사슬 하나로 전제하는 `demo_task` · `demo_wbc` · `demo_compliance` 는 YAML 이 없어 인스턴스화되지 않는다. 포구 컨트롤러 config 도 아직 없다.
- `demo_joint_controller` 의 목표는 `/demo_joint_controller/{g1,p1b}/joint_goal` (`rtc_msgs/RobotTarget`), 로그는 `<session>/controllers/demo_joint_controller/{g1_state,p1b_state,p1b_sensor}.csv` 이다.
- `enable_mpc` 는 CPU layout 만 고른다 (기본 `false`). `mpc_engine` 인자는 없다.
- `demo_dualarm_controller` 로 바꾸려면 기동 때 `initial_controller:=demo_dualarm_controller`, 실행 중에는 다음으로 switch 한다 (목표 형식과 키는 [DemoDualArmController](docs/demo-dualarm-controller.md)).

```bash
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['demo_dualarm_controller'], deactivate_controllers: ['demo_joint_controller'], strictness: 1, timeout: {sec: 3}}"
```

GUI 는 `ros2 run integrated_bringup demo_controller_gui --robot g1_p1b` ([demo_controller_gui](#demo_controller_gui)). 세션 로그는 `plot_rtc_log` 가 읽는다 ([rtc_tools README](../rtc_tools/README.md)).

#### 기동 자세

시뮬레이터의 기동 자세는 각 그룹의 `initial_qpos` (rad, `command_joint_names` 순서) 가 정하며 MJCF keyframe 보다 우선한다 ([rtc_mujoco_sim README](../rtc_mujoco_sim/README.md#초기-자세-initial_qpos)). 값을 고칠 때 **모든 원소에 소수점을 찍는다** — 정수 리터럴이 하나라도 있으면 시퀀스 전체가 정수 배열로 읽혀 노드 생성 시점에 죽는다. 길이와 keyframe 일치는 `test/test_shipped_initial_qpos.py` 가 고정한다 (keyframe 일치 레인은 mujoco 가 필요해 `colcon test` 에서 skip — [testing.md](../docs/testing.md#test-commands) 참조).

### 데모 컨트롤러

이 패키지가 등록하는 컨트롤러는 일곱이다 (목록의 SSoT 는 `src/controllers/controller_registration.cpp` 의 `RTC_REGISTER_CONTROLLER*`). 한 번에 하나만 활성이고, 로봇의 `config/<robot>/controllers/` 에 그 컨트롤러의 YAML 이 있어야 인스턴스화된다 (`demo_joint_controller` 만 모든 로봇이 싣는다).

| config_key | 하는 일 | YAML 을 싣는 로봇 | 상세 |
|---|---|---|---|
| `demo_joint_controller` | 관절 공간 quintic 궤적 + 손. 새 로봇 bring-up 용 | `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` · `g1_p1b` | [demo-joint-controller.md](docs/demo-joint-controller.md) |
| `demo_task_controller` | Cartesian CLIK + TCP 궤적 + 손 | `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` | [demo-task-controller.md](docs/demo-task-controller.md) |
| `demo_compliance_controller` | task 컨트롤러 + 외부 렌치 admittance (렌치는 손 레인의 pull 추정) | `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` | [demo-compliance-controller.md](docs/demo-compliance-controller.md) |
| `demo_wbc_controller` | TSID / CLIK-QP whole-body + 손 phase FSM (+ 선택적 MPC) | `ur5e_p1a` · `ur5e_p1b` · `iiwa7_leap` | [demo-wbc-controller.md](docs/demo-wbc-controller.md) |
| `demo_inference_controller` | ONNX 정책. 텐서 · 정규화 · 모델 경로가 전부 YAML | `ur5e_p1b` | [demo-inference-controller.md](docs/demo-inference-controller.md) |
| `demo_catching_controller` | 날아오는 물체 포획: vision 예측 → 계획기 → 확장 CLIK 팔 + 손 시퀀서 | `ur5e_p1b` · `iiwa7_leap` | [demo-catching-controller.md](docs/demo-catching-controller.md) |
| `demo_dualarm_controller` | 두 손 frame 과제 + 자세를 QP 하나로 푸는 상체 제어 | `g1_p1b` | [demo-dualarm-controller.md](docs/demo-dualarm-controller.md) |

컨트롤러가 지켜야 하는 계약은 [agent_docs/controllers.md](../agent_docs/controllers.md), 컨트롤러별 게인 · 토픽 카탈로그는 [docs/controllers.md](../docs/controllers.md) 에 있다. 공통 파라미터 · 팔 끝 frame · 손끝 FK 배선 · grasp mode 전환은 [demo-shared-params.md](docs/demo-shared-params.md), 컨트롤러가 남기는 CSV 는 [controller-logs.md](docs/controller-logs.md) 가 갖는다.

#### 컨트롤러 전환

시작할 컨트롤러는 launch 인자 `initial_controller:=<config_key>` 로 고른다 (`sim_*` launch 가 받는다. 기본값은 로봇의 `sim.yaml` / `robot.yaml`). 실행 중 전환은 서비스로 한다.

```bash
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['demo_task_controller'], \
    deactivate_controllers: ['demo_joint_controller'], strictness: 1, timeout: {sec: 1}}"
ros2 service call /rtc_cm/list_controllers rtc_msgs/srv/ListControllers
```

전역 E-STOP 중에는 전환이 `ok=false` 로 거부된다. 컨트롤러 전환 후 `/active_controller_name` (TRANSIENT_LOCAL) 이 바뀌므로 외부 도구는 이를 구독해 소유 토픽을 다시 연결한다.

### 토픽 소유권

데모 컨트롤러는 non-RT 외부 토픽을 **컨트롤러가 소유**한다 — 목표 (`/<config_key>/<group>/joint_goal`, `rtc_msgs/RobotTarget`), `grasp_state` · `wbc_state` · `catching_state`, `payload_estimate`, `<config_key>/transforms` (`tf2_msgs/TFMessage`) 가 모두 active 컨트롤러의 네임스페이스에 뜨고, 전환하면 이전 네임스페이스에서 침묵한다. group 별 `joint_states` 는 CM (`/rtc_cm/<group>/joint_states`), device 로 가는 명령 lane 은 DeviceBackend 가 소유한다. 소유자 표와 CSV 목록: [controller-logs.md](docs/controller-logs.md).

### Task 목표의 기준 frame (#779)

`rtc_msgs/RobotTarget` 의 task 목표 (`goal_type: task`) 는 `header.frame_id` 에 자기 기준 frame 을 싣는다. `.msg` 는 바뀌지 않았고 계약은 여기가 갖는다.

| 컨트롤러 | 받는 `frame_id` | 그 밖 |
|---|---|---|
| `demo_wbc` · `demo_task` · `demo_compliance` (팔, device 0) | 팔의 `root_link` (device `urdf.root_link`) — 그대로 · `scene_world` — `root_link` 로 변환 | 빈 값 · 다른 이름 · 정의되지 않은 `scene_world` 는 **거부** |
| `demo_dualarm` | 자기 `<task>/task_goal` lane 의 `clik.target_frames` (빈 값 = 과제 base) | 이 계약 밖. 그쪽 `"world"` 는 model frame 이지 `scene_world` 가 아니다 |

- **`scene_world`** 는 model frame 이다: `urdf.extra_frames.scene_world {parent: <root_link>, xyz, rpy}` (규약 `p_root = R(rpy)·p_world + xyz`) 를 CM 이 모든 모델에 더한다. 출하 정의는 `sim.yaml` (p1a · p1b · iiwa7_leap) 과 장면을 바꾸는 sim overlay 마다 있고 `_base.yaml` · `robot.yaml` 에는 **없다** — 실기는 calibration 값을 넣기 전까지 world 목표를 거부한다. `g1_p1b` 에도 없다.
- **소비자는 이 frame 하나를 읽는다** — task 목표 컨트롤러, catching 의 vision world ([demo-catching-controller.md](docs/demo-catching-controller.md#입력)), inference 의 물체 lane, demo GUI 의 `world (scene)` 선택기, `rtc_tools` 의 catching 분석 (`_base.yaml` → `sim.yaml` → overlay 병합). 자기 world 사본을 두는 소비자는 없다.
- **세 상태.** 없음 (또는 `provisional: true`) → configure 는 성공하고 world 목표만 거부한다 (configure INFO 한 줄이 받는 frame 을 말한다). `root_link` 에 rigid 하지 않거나 비유한 값 → `on_configure` 실패. CM 이 파싱하지 못하는 항목 → 기동 전체 거부. 출하 값은 장면과 대조했으므로 `provisional: false` 다 — 키를 빼면 provisional 로 파싱된다.
- **장면을 바꿀 때.** `scene_world` 는 파라미터일 뿐 장면에서 읽지 않는다. 명령줄 `model_path:=` 로 장면만 바꾸면 `sim.yaml` 의 world 가 그대로 남으니, 그 장면의 `scene_world` 를 함께 주는 sim overlay 를 쓴다.
- **변환 위치와 거부 계수.** 비-RT target ingress (`SetDeviceTaskTarget(idx, task6, frame_id)`) 에서 공용 `support/task_target_frames.hpp` 로 변환한 뒤 mailbox 에 넣는다. `root_link` 목표는 비트 그대로다. `control_6dof: false` (Task · Compliance) 에서는 x, y, z 만 변환하고 3..5 는 건드리지 않는다. 거부는 `/rtc_cm/list_controllers` 의 `target_reject_count` 로 세고 throttled WARN 이 사유를 말한다 (`rtc_controller_interface` README §기준 frame 과 goal 종류).
- **configure 거부.** `root_link` 가 이름은 있는데 모델 frame 이 아니면 실패한다. 설정이 아예 없으면 world 에서 동작하고 task 목표는 전부 거부된다.
- **생산자.** BT coordinator · shape_estimation 은 파라미터 `base_frame` (기본 `"base"`) 을 TF 조회 부모와 `frame_id` 에 함께 쓴다 — `root_link` 와 다르면 목표가 거부로 드러난다. demo GUI 는 profile 의 `tcp_parent` 를 싣는다.

**goal 종류.** 한 종류를 다른 종류로 잘못 읽는 컨트롤러는 그 종류를 거부하고 센다 (`AcceptedGoalKinds`).

| 컨트롤러 | joint 목표 | task 목표 |
|---|---|---|
| `demo_joint` | 받음 | **거부** |
| `demo_task` · `demo_compliance` | 팔: **거부** · 손: 받음 | 팔: 받음 · 손: **거부** |
| `demo_wbc` | 받음 (팔 posture · 손) | 팔: 받음 · 손: **거부** |

7-DoF 팔 (iiwa7) 의 Task · Compliance position-only 모드에서 null-space 자세 `null_target[6]` 는 외부에서 줄 수 없다 — 6 값 task 목표에는 자리가 없고 그 값을 싣던 팔 joint lane 은 막혀 있다 (seed 값 유지).

### Catching sim trials

`catching_sim_trials` 는 떠 있는 sim 에 포구 투척을 하나씩 던지고 순환 (ARMED → 포구 → RETREAT → ARMED) 의 결과를 `<out>/trial_results.json` 에 기록한다. 포구 모집단의 sim 단계는 `catch_population_trials` 가 묶음마다 sim 을 띄워 같은 일을 한다. 둘 다 설치되는 명령이다.

```bash
ros2 run integrated_bringup catching_sim_trials <out> \
  --throws-file $(ros2 pkg prefix integrated_bringup)/share/integrated_bringup/config/ur5e_p1b/throw_sets/throw_catch_set_grid_mpc.json
ros2 run integrated_bringup catch_population_trials --config <catch_population.yaml> --out <결과 폴더> --estimator-ws <추정기 workspace>
```

sim · 공 추정기 · 컨트롤러 전환을 먼저 해 둬야 한다. 전체 순서, 투척 목록 JSON 형식, 러너 · 모집단 · 평가 도구 (`tools/`) 의 인자와 결과 읽기는 [포구 sim 시행 실행 안내](../docs/catching-sim-trials.md) 에 있다.

### GUI 도구

#### demo_controller_gui

실시간 컨트롤러 선택, 게인 튜닝, 상태 모니터링 GUI 입니다 (tkinter).

```bash
ros2 run integrated_bringup demo_controller_gui                      # 기본 ur5e_p1a
ros2 run integrated_bringup demo_controller_gui --robot ur5e_p1b     # ur5e + proto_1b hand
ros2 run integrated_bringup demo_controller_gui --robot iiwa7_leap   # iiwa7 + LEAP
ros2 run integrated_bringup demo_controller_gui --robot g1_p1b       # G1 상체 + proto_1b 오른손
```

- `--robot <key>` 는 arm / hand joint 스키마와 TCP tf frame 을 고릅니다. `key` 는 `controllers/` 가 있는 `config/<key>/` 디렉토리명과 같고, 잘못된 key 는 즉시 에러로 종료합니다. 별칭은 `--ur5e` · `--p1b` · `--iiwa` · `--g1` 입니다.
- 컨트롤러가 publish 하는 joint span 이 프로파일과 어긋나면 `/rosout` 에 WARN 이 한 번 나옵니다. 맞는 `--robot` 으로 다시 실행합니다.
- `--session <dir>` 은 `demo_dualarm_controller` 의 상태 판독이 따라갈 세션 디렉토리입니다. 그 컨트롤러가 없는 프로파일에서는 쓰이지 않습니다.
- 새 로봇은 `integrated_bringup/demo_gui/discovery.py` 의 `ROBOT_PROFILES` 에 항목을 추가합니다.

컨트롤러별 게인 패널입니다 (라디오에는 프로파일이 전환 가능한 컨트롤러만 뜹니다).

| 컨트롤러 | 게인 그룹 | 파라미터 (R/W) | RO 캡 |
|---|---|---|---|
| `demo_joint_controller` | Arm/Hand Trajectory | `robot_trajectory_speed`, `hand_trajectory_speed` | `robot_max_traj_velocity`, `hand_max_traj_velocity` |
| `demo_joint_controller` | Grasp Detection | `grasp_contact_threshold`, `grasp_force_threshold`, `grasp_min_fingertips` | — |
| `demo_task_controller` | CLIK Gains | `kp_translation` (×3), `kp_rotation` (×3), `singularity_threshold`, `max_damping`, `null_kp`, `enable_null_space`, `control_6dof` | — |
| `demo_task_controller` | Arm/Hand Trajectory | `trajectory_speed`, `trajectory_angular_speed`, `hand_trajectory_speed` | `max_traj_velocity`, `max_traj_angular_velocity`, `hand_max_traj_velocity` |
| `demo_task_controller` | Grasp Detection | (joint 와 동일) | — |
| `demo_wbc_controller` | Arm/Hand Trajectory | `arm_trajectory_speed`, `hand_trajectory_speed` | `arm_max_traj_velocity`, `hand_max_traj_velocity` |
| `demo_wbc_controller` | Hand τ_ff | `hand_tauff_enable`, `hand_tauff_gravity_gain`, `hand_tauff_closure_bias`, `hand_tauff_max`, `hand_tauff_source` (`gravity_comp` \| `tsid_tau`) | — |
| `demo_wbc_controller` | TSID Weights | `se3_weight`, `force_weight`, `posture_weight` | — |
| `demo_wbc_controller` | Grasp Detection | (joint 와 동일) | — |
| `demo_wbc_controller` | MPC | `mpc_enable`, `riccati_gain_scale` | — |
| `demo_dualarm_controller` | Task `<task>` gain (과제마다) | `tasks.<task>.gain_linear`, `tasks.<task>.gain_angular` | — |
| `demo_dualarm_controller` | Posture gain | `posture.<group>.gain` (자세군마다) | — |
| `demo_dualarm_controller` | Trajectory speed | `trajectory.linear_speed`, `trajectory.angular_speed`, `trajectory.hand_speed` | `trajectory.linear_speed_max`, `trajectory.angular_speed_max`, `trajectory.hand_speed_max` |

`demo_inference_controller` 는 선택과 전환만 되고 입력 패널은 모두 꺼집니다 (외부 명령 경로 없음). 목록에 뜨는 컨트롤러 집합, Virtual TCP · Target · Pull Force 패널, Preset 단축키, 다중 frame (`g1_p1b`) 동작은 [docs/demo-controller-gui.md](docs/demo-controller-gui.md) 에 있습니다.

#### motion_editor_gui

모션 시퀀스 편집기입니다 (PyQt5 · x86_64 전용 — aarch64 venv 는 PyQt5 를 설치하지 않는다).

```bash
ros2 run integrated_bringup motion_editor_gui
```

- 다중 탭으로 모션 파일을 동시에 편집하고, 포즈 테이블 (이름, 팔 / 핸드 프리뷰, 궤적 시간, 대기 시간) 을 JSON 으로 로드 · 저장합니다.
- 포즈 삽입 · 삭제 · 복사 · 붙여넣기와 선택 모션 재생을 지원합니다.
- 팔 자세는 joint 목표입니다. Task · Compliance 가 활성이면 보내지 않고 이유를 로그에 남깁니다 ([Task 목표의 기준 frame](#task-목표의-기준-frame-779)).

## 설정

### 기구학 모델 설정

데모 컨트롤러는 `rtc_urdf_bridge` 로 Pinocchio 모델을 만듭니다. URDF 경로와 모델 토폴로지 (`sub_models`, `tree_models`, `passive_joints`) 는 mode 와 무관하므로 `config/<robot>/_base.yaml` 의 최상위 `urdf:` 에 한 번 적습니다. `robot.yaml` / `sim.yaml` 은 mode 차이만 덮고, launch 는 `[_base.yaml, <mode>.yaml]` 순으로 읽습니다 (뒤 파일이 키 단위로 덮음).

- `urdf:` 와 YAML 합성 규칙은 [rtc_controller_manager README](../rtc_controller_manager/README.md#설정-파일) 가 소유합니다.
- 컨트롤러 파라미터 override: [`<config_key>.<yaml.경로>`](../rtc_controller_manager/README.md#컨트롤러-파라미터-override--config_keyyaml경로).
- 컨트롤러 YAML 분할: [`include:`](../rtc_controller_manager/README.md#컨트롤러-yaml-을-여러-파일로-나누기--include).

| 영역 | 위치 |
|---|---|
| `urdf:` · `devices:` roster · limits | `_base.yaml` |
| `devices.<g>.backend` · `has_native_*` · `joint_command_names` | `robot.yaml` / `sim.yaml` |
| 컨트롤러 게인 · 토픽 · 공통 파라미터 | `controllers/demo_*_controller.yaml` · `demo_shared.yaml` |

`sub_models` / `tree_models` 의 map 키는 `devices` 의 디바이스 그룹 이름과 같아야 합니다 (예: sub_model `ur5e` = device `ur5e`).

```yaml
urdf:
  package: "robot_descriptions"
  path: "robots/ur5e_assm_v1/urdf/ur5e_with_hand.urdf"
  root_joint_type: "fixed"
  sub_models:
    ur5e:                     # map 키 = devices.ur5e 그룹명
      root_link: "base"
      tip_link: "tool0"
  passive_joints: ["thumb_cmc_aa", "thumb_cmc_fe"]   # arm 모델에서 lock
```

#### Closed-chain (Extended-URDF) bringup — `ur5e_p1b`

`ur5e_p1a` 는 mimic 기반이고 `ur5e_p1b` 는 5-loop 폐쇄 체인 손입니다. URDF 는 spanning tree 이고 loop closure 는 `<stem>.closure.yaml` sidecar 에 있습니다. `config/ur5e_p1b/_base.yaml` 의 `urdf:` 가 폐쇄 체인을 켭니다.

```yaml
urdf:
  package: "hand_description"
  path: "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro"
  extended: true                                                  # loop closure 활성
  closure_path: "robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml"
```

- 손 FK 를 쓰는 컨트롤러는 loop-passive 하류 fingertip 을 loop-consistent 하게 FK 합니다. device `joint_state_names` 가 16 actuated 를 모두 덮으면 `closed-chain hand FK active` 로그가 뜨고, 아니면 serial FK 로 fallback 하거나 configure 를 거부합니다.
- RViz 시각화는 `config/ur5e_p1b/digital_twin.yaml` 의 `closure_path` 를 `rtc_digital_twin` 의 `digital_twin.launch.py` 가 읽어 `closure_state_publisher` 를 띄웁니다. 사영 tolerance 는 `closure_tolerance` / `closure_acceptance_tolerance` 로 바꿉니다.

```bash
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py enable_viewer:=false   # headless
```

`ur5e_p1b` 는 `robot.yaml` · `sim.yaml` 모두 `initial_controller: "demo_joint_controller"` 로 시작합니다 (`ur5e_p1a`, `iiwa7_leap` 은 `demo_wbc_controller`). WBC 로 시작하려면 인자로 지정하거나 GUI 라디오에서 전환합니다.

```bash
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py initial_controller:=demo_wbc_controller
```

#### Scene overlay (`sim_overlay:=`) — 출하 씬을 건드리지 않고 바꾸기

`sim_ur5e_p1b`, `sim_iiwa7_leap`, `sim_g1_p1b` 는 `mujoco_simulator.yaml` 뒤, 명령줄 인자 앞에 params 파일 하나를 더 끼우는 `sim_overlay:=` 를 받습니다 (`sim_ur5e_p1a` 는 받지 않음). overlay 는 두 노드에 모두 전달되고 각 노드는 자기 섹션만 읽습니다. `sim_overlay:=` 없이 띄우면 출하 씬 그대로입니다.

- **값 해석**: 이름만 주면 `config/<robot>/sim_overlays/<name>.yaml`, `/` 가 있거나 `.yaml` 로 끝나면 경로입니다. 해석되지 않는 값은 launch 를 실패시키고 에러에 출하 overlay 이름 목록이 실립니다.
- **출하 overlay**: `config/ur5e_p1b/sim_overlays/` 에 `inference_pole`, `fingertip_grasp`, `fingertip_grasp_free` 가 있고 용도는 각 파일 헤더 주석이 갖습니다. 새 실험용 overlay 는 repo 밖에 두고 경로로 넘깁니다. overlay 는 다른 overlay 를 include 하지 못하므로 자기완결 파일이어야 합니다.
- **장면을 바꾸는 overlay 는 `urdf.extra_frames.scene_world` 도 둡니다.** 파라미터는 키 단위로 덮이므로 적지 않으면 `sim.yaml` 의 (다른 장면의) world 가 남습니다. 자세한 규칙은 [Task 목표의 기준 frame](#task-목표의-기준-frame-779) 을 봅니다.
- **키 경로가 어긋나면 경고 없이 무시되어 출하값으로 돕니다.** 컨트롤러 미러 (`ros2 param get`) 나 기동 로그로 적용을 확인합니다. `sim.yaml` 의 override 는 overlay 아래에 깔리므로, overlay 가 `robot.arm.qdd_max` 를 적지 않으면 `sim.yaml` 값으로 돕니다.
- **포구 컨트롤러**: 출하 overlay 는 없습니다. 탐색과 구간 계획기는 launch 인자 `catch_search_mode:=` · `catch_segment_mode:=` 로 바꿉니다. 지운 키 (`robot.arm.accel_limits_*`) 나 `joint_cmd.accel_constraint: box` 는 컨트롤러가 park 하고 ERROR 로그가 해당 키를 지목합니다.

`demo_inference_controller` 의 정책이 학습된 씬은 `inference_pole` 이고 모델은 repo 밖에 둡니다 (`RTC_POLICY_DIR`).

```bash
export RTC_POLICY_DIR=/path/to/ObjectHandGraspDeployMulti5-Export-v0   # 모델은 repo 밖
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py \
    sim_overlay:=inference_pole use_cpu_affinity:=false

# 기동 후 (기본 활성은 demo_joint_controller):
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: ['demo_inference_controller'], \
    deactivate_controllers: ['demo_joint_controller'], strictness: 1, timeout: {sec: 1}}"
```

기동 로그의 `[inference] N observed link(s) in '<frame>'` 이 관측 frame 이고, `[inference] policy loaded: ...` 가 없으면 `RTC_POLICY_DIR` 이 안 잡힌 것입니다 (`allow_missing_model: true` 라 컨트롤러는 자세만 유지). 결과는 `<session>/controllers/demo_inference_controller/inference_diag.csv` 로 판독합니다.

### 전역 설정

`config/<robot>/_base.yaml` (mode 공통) 위에 `robot.yaml` (실로봇) 또는 `sim.yaml` (sim) 이 얹힙니다. 값은 해당 파일을 직접 봅니다 (예: [`config/ur5e_p1a/_base.yaml`](config/ur5e_p1a/_base.yaml)).

| 키 | 파일 | 의미 |
|---|---|---|
| `control_rate` | `_base.yaml` | RT 루프 주기 (Hz). 범위는 [invariants.md](../agent_docs/invariants.md) §RT Path |
| `device_timeout_names` / `device_timeout_values` | `_base` / mode 파일 | 디바이스별 timeout (ms). sim 은 여유 있게 10000 |
| `enable_logging` · `enable_timing_log` · `enable_device_log` · `max_log_sessions` | `_base.yaml` | 세션 로깅 |
| `initial_controller` | mode 파일 | 시작 컨트롤러 (launch 인자로 override) |
| `init_timeout_sec` · `enable_estop` | mode 파일 | 실로봇은 30 s · true, sim 은 0 (끔) · false |

### 공통 파라미터 (`demo_shared.yaml`)

`DemoJoint` · `DemoTask` · `DemoCompliance` · `DemoWbc` 가 공유하는 파라미터는 로봇별 `config/<robot>/controllers/demo_shared.yaml` 에 단일 소스로 있고 네 컨트롤러가 `LoadConfig()` 때 같은 로더로 읽는다. 컨트롤러별 YAML 에 같은 키를 쓰면 그 컨트롤러만 덮어쓴다 (`demo_shared.yaml` → `demo_*_controller.yaml` 순). 키 표와 계약은 [demo-shared-params.md](docs/demo-shared-params.md) 에 있다: Virtual TCP, grasp 임계, `grasp_controller_type`, `force_pi_grasp.*`, `pull_estimator.*`, `momentum_observer.*`.

#### Grasp threshold capability matrix

컨트롤러는 `devices.<hand>.sensor_layout.has_native_contact` (robot / sim yaml) 를 보고 두 경로로 분기한다.

| 키 | 활성 sensor type | 의미 |
|---|---|---|
| `grasp_contact_threshold` | sensor A only (udp_hand + `ft_inferencer.enabled`) | native sigmoid contact probability threshold (0..1). sensor B 에선 미사용 |
| `grasp_force_threshold` | sensor A + B 공통 | `\|F\| > threshold` [N]. sensor A 에선 contact_threshold 와 AND 결합 |
| `grasp_min_fingertips` | sensor A + B 공통 | `grasp_detected = active_count ≥ N` |

### 로깅 (Logging)

레벨 분류 기준 · THROTTLE 규칙 · 서브-로거 네이밍은 [agent_docs/conventions.md](../agent_docs/conventions.md#logging) 가 SSoT 이고, 이 패키지의 로거 factory 와 THROTTLE 상수는 [bringup_logging.hpp](include/integrated_bringup/support/bringup_logging.hpp) 에 있다. 컨트롤러 로거 이름은 `integrated_bringup.<config_key>` 이고, 런타임에 컨트롤러 하나만 DEBUG 로 올릴 수 있다.

```bash
ros2 service call /integrated_rt_controller/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'integrated_bringup.demo_joint_controller', level: 10}]}"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"   # 콘솔에 로거 이름 표시
```

level 10 은 DEBUG, 30 은 WARN 이다. 컨트롤러가 세션에 남기는 CSV 는 [controller-logs.md](docs/controller-logs.md) 를 본다.

## 디렉토리

```
integrated_bringup/
├── config/          # 로봇별 YAML
│   ├── ur5e_p1a/
│   ├── ur5e_p1b/
│   ├── iiwa7_leap/
│   ├── g1_p1b/
│   └── b7_p2/       # 손 설정만 (params_file)
├── launch/          # robot_* / sim_* launch
├── include/         # 공개 헤더
├── src/             # 컨트롤러 · backend 구현
├── integrated_bringup/  # python 패키지
│   └── demo_gui/    # demo_controller_gui 모듈
├── scripts/         # ros2 run 진입점
├── tools/           # 포구 평가 · 식별 도구
├── docs/            # 사용자 참조 문서
└── test/
```

## 더 읽기

- [docs/demo-controller-gui.md](docs/demo-controller-gui.md) — GUI 패널 동작 상세
- [rtc_controller_manager README](../rtc_controller_manager/README.md#설정-파일) — YAML 합성 · override · include
- [rtc_urdf_bridge README](../rtc_urdf_bridge/README.md) — 모델 빌더와 관절 분류
- [architecture.md](../agent_docs/architecture.md) — 의존성 그래프와 threading model
- [docs/controllers.md](../docs/controllers.md) — 컨트롤러 계약

## 의존성

빌드 · 실행 의존성은 [package.xml](package.xml) 이 SSoT 입니다. 의존성 그래프는 [architecture.md](../agent_docs/architecture.md) 를 봅니다. 라이선스: MIT ([LICENSE](../LICENSE)).
