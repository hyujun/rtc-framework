# rtc_mujoco_sim

MuJoCo 3.x 기반 robot-agnostic 물리 시뮬레이터 lifecycle 노드와 GLFW 뷰어. **유형**: 노드 패키지

## 개요

실제 로봇 드라이버를 대체해 알고리즘을 개발 PC 에서 검증한다. YAML 로 정한 임의 개수의 **그룹** (robot arm, hand 등) 을 독립된 command / state 토픽으로 구동한다. 로봇별 모델 · 조인트 · 토픽 설정과 컨트롤러 통합 launch 는 bringup 패키지 (예: [integrated_bringup](../integrated_bringup/README.md)) 가 소유하고, 이 패키지는 `model_path` 를 외부 입력으로 받는다 ([design-principles.md](../agent_docs/design-principles.md)).

- **`robot_response` 그룹**: MuJoCo 물리로 구동한다. YAML 의 `command_joint_names` 와 XML 의 hinge + actuator 조인트가 **양방향으로 완전히 일치**해야 한다 (한쪽에만 있으면 Initialize 실패). 이름 기반 qpos / qvel / actuator 매핑이라 비연속 인덱스도 된다.
- **`fake_response` 그룹**: 물리 없이 1차 LPF 로 command 를 되돌려 주는 100 Hz 에코백. XML 에 없는 디바이스 (예: 손) 용이다. 토픽 이름만 검증한다.
- 한 이름이 robot 과 fake 양쪽에 있으면 에러다.
- **동기 루프**: 매 step 마다 state 발행 → 모든 robot 그룹의 command 대기 (`sync_timeout_ms`) → `n_substeps` 번 물리 진행 → `max_rtf` 로 throttle. 컨트롤러의 tick 하나가 sim step 하나다. 명령하지 않는 그룹은 `wait_for_command: false` 로 뺀다 (안 빼면 매 step 이 타임아웃을 기다린다).
- 부가 기능: [Object Pool](#object-pool) · [Object State](#object-state) · [Projectile Ball](#projectile-ball-발사-공) (투척 실험) · contact wrench 자동 발견 · 측정용 CSV lane. 동기 루프 · tick 의 배경은 [sim-rationale.md](../docs/reference/sim-rationale.md).

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. MuJoCo 가 없으면 노드 (`mujoco_simulator_node`) 는 빌드에서 빠지고, GLFW 가 없으면 뷰어 없이 headless 로만 빌드된다. MuJoCo 3.2 이상이 필요하다 ([의존성](#의존성)).

```bash
# 이 저장소만으로 도는 단독 smoke test — 그룹 설정은 bringup YAML 에서 가져온다
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py \
    params_file:=$(ros2 pkg prefix integrated_bringup)/share/integrated_bringup/config/ur5e_p1a/mujoco_simulator.yaml

# 헤드리스 10배속
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py params_file:=... enable_viewer:=false max_rtf:=10.0

# 컨트롤러까지 포함한 통합 시나리오는 bringup launch
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py
```

`mujoco_sim.launch.py` 는 노드를 띄워 `configure` → `activate` 를 자동 전환한다. `params_file` 없이는 `robot_response.groups` 가 비어 configure 가 실패한다. 상태 확인:

```bash
ros2 lifecycle get /mujoco_simulator          # active 여야 한다
ros2 topic echo /sim/status --once            # [step_count, sim_time_sec, rtf, paused]
ros2 topic hz /joint_states
ros2 lifecycle set /mujoco_simulator deactivate   # 런타임 정지 / activate 로 재개
```

뷰어 조작은 [뷰어 단축키](#뷰어-단축키) — 일시정지 `Space`, 리셋 `R`, 도움말 `F1`.

## 인터페이스

### 실행파일 · launch

| 이름 | 종류 | 설명 |
|---|---|---|
| `mujoco_simulator_node` | 실행파일 (`LifecycleNode`) | 시뮬레이터 본체. 이름 `mujoco_simulator` |
| `mujoco_sim.launch.py` | launch | 노드 단독 + 세션 디렉토리 생성 (`RTC_SESSION_DIR` 전파). 컨트롤러는 띄우지 않는다 |

launch 인자 (빈 값 = YAML 값 사용):

| 인자 | 기본값 | 설명 |
|---|---|---|
| `params_file` | `""` | `solver_param.yaml` 위에 덮을 YAML (robot 그룹 설정 등) |
| `model_path` | `""` | MJCF scene 경로 |
| `enable_viewer` | `""` | GLFW 뷰어 on/off |
| `viewer_input_enabled` | `""` | `false` 면 world 를 바꾸는 뷰어 입력 무시 ([뷰어 단축키](#뷰어-단축키)) |
| `max_rtf` | `""` | 최대 실시간 비율 (0.0 = 무제한) |
| `max_log_sessions` | `10` | 보관할 세션 폴더 수 |

세션 루트는 `rtc_tools.utils.session_dir.resolve_logging_root()` 의 3단 체인 (`$COLCON_PREFIX_PATH` → cwd 상위 `install/` + `src/` → `$PWD`) 이 정한다 ([rtc_tools/README.md](../rtc_tools/README.md)).

### 퍼블리시 토픽

모든 그룹이 같은 타입을 쓴다 — command `rtc_msgs/JointCommand`, state `sensor_msgs/JointState` (`effort` 는 관절 토크 [N·m]), 센서 `rtc_msgs/SimSensorState`. QoS 는 command subscriber 와 state publisher 모두 `BEST_EFFORT` + depth 1.

| 토픽 | 타입 | 주기 | 설명 |
|---|---|---|---|
| `<group.state_topic>` | `sensor_msgs/JointState` | 매 step ÷ `state_publish_divisor` | robot 그룹: 위치 / 속도 / 토크 |
| `<group.state_topic>` | `sensor_msgs/JointState` | 100 Hz | fake 그룹: LPF 상태 |
| `<group.sensor_topic>` | `rtc_msgs/SimSensorState` | 매 step ÷ divisor | MJCF 센서 (`sensor_topic` + `sensor_names` 설정 시) |
| `<contact_wrench.topic_prefix>/<target>/contact_wrench` | `geometry_msgs/WrenchStamped` | 매 step ÷ divisor | MJCF contact 센서 자동 발견. 비접촉 시 0 발행. [부호](#contact-wrench-자동-발견) 는 link-on-environment |
| `.../<target>/contact_state` | `std_msgs/Bool` | 전이 시 | `publish_state: true` 일 때. 값이 바뀔 때만 |
| `.../<target>/contact_point` | `geometry_msgs/PointStamped` | 매 step | `publish_debug: true` 일 때. **world frame** 접촉점 |
| `.../<target>/contact_depth` | `std_msgs/Float64` | 매 step | `publish_debug: true` 일 때. 음수가 관통 |
| `<object_state.topic>` | `tf2_msgs/TFMessage` | 매 step ÷ `publish_divisor` | 씬 전체의 free body 이름 · 프레임 · pose ([Object State](#object-state)) |
| `<projectile_ball.publish.ground_truth_topic>` | `nav_msgs/Odometry` | `sample_rate_hz` (sim 시간) | 발사된 공의 참값. twist 는 world 프레임. park 중에는 발행하지 않는다 |
| `<projectile_ball.publish.camera_topic>` | `geometry_msgs/PointStamped` | 〃 | 위 위치 + 축별 가우시안 노이즈 (카메라 관측 모사) |
| `/sim/status` | `std_msgs/Float64MultiArray` | 1 Hz | `[step_count, sim_time_sec, rtf, paused(0/1)]` |

### 구독 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `<group.command_topic>` | `rtc_msgs/JointCommand` | robot 그룹: `command_type` (`position` / `torque` / `pd_feedforward`) 로 제어 모드 자동 전환. fake 그룹: LPF 타겟 갱신 |

### 서비스

| 서비스 | 타입 | 설명 |
|---|---|---|
| `/sim/set_external_wrench` | `rtc_msgs/srv/SetExternalWrench` | body 에 알려진 외력을 걸거나 전부 해제 |
| `/sim/launch_ball` | `std_srvs/srv/Trigger` | 설정 분포에서 샘플링해 공 발사 (뷰어 `K` 와 같은 경로). 공이 비활성이면 `success: false` |
| `/sim/launch_ball_at` | `rtc_msgs/srv/LaunchBall` | 호출자가 준 `(p0, v0, ω)` (world) 로 발사. RNG 를 쓰지 않는다. 거부 시 `accepted: false` 이고 아무것도 바뀌지 않는다 |
| `/sim/reset_ball` | `std_srvs/srv/Trigger` | 공을 park 위치로 회수. 공이 비활성이면 `success: false` |

#### 외력 주입 (`/sim/set_external_wrench`)

추정기 · 관측기 검증에는 참값을 아는 부하가 필요하다. MJCF body 를 이름으로 지정해 world 축 wrench 를 건다.

```bash
# ee_link 프레임 원점에 1 kg 상당의 하중을 매단다 (아래 방향 9.81 N)
ros2 service call /sim/set_external_wrench rtc_msgs/srv/SetExternalWrench \
  "{body_name: ee_link, point: {x: 0.0, y: 0.0, z: 0.0},
    wrench: {force: {x: 0.0, y: 0.0, z: -9.81}, torque: {x: 0.0, y: 0.0, z: 0.0}},
    clear: false}"

# 전부 해제
ros2 service call /sim/set_external_wrench rtc_msgs/srv/SetExternalWrench "{clear: true}"
```

- 작용점은 `point` 이지 body 질량중심이 아니다 (`{0,0,0}` = body 프레임 원점). 질량중심 보정은 sim 이 매 tick 한다.
- **latched**: 해제 · 덮어쓰기 · 시뮬레이터 리셋 전까지 매 tick 재적용된다. 한 body 만 떼려면 그 body 에 0 wrench 를 보낸다.
- 없는 body 와 `world` 는 `accepted=false` + 사유로 거부된다.
- `JointState.effort` 에는 이 외력이 들어가지 않는다 — 관측기가 미지의 외란 토크로 보아야 하기 때문이다.

계약의 SSoT 는 [SetExternalWrench.srv](../rtc_msgs/srv/SetExternalWrench.srv) 주석, 측정 근거는 [sim-rationale.md](../docs/reference/sim-rationale.md#외력-주입-simset_external_wrench).

### 제어 모드 자동 전환

robot 그룹은 command 의 `command_type` 으로 모드를 바꾼다.

| `command_type` | 제어 모드 | Robot gravcomp | World gravity |
|---|---|---|---|
| `"position"` | Position servo | **ON** (해당 그룹 body chain 만) | ON |
| `"torque"` | Direct torque | OFF (controller 가 중력 보상 책임) | ON |
| `"pd_feedforward"` | PD servo + feedforward torque | OFF (feedforward 가 중력 포함) | ON |

- 전역 중력은 항상 켜져 있어 같은 씬의 free body (들어올릴 object) 는 정상적으로 떨어진다.
- `JointState.effort` = `qfrc_actuator + qfrc_applied + qfrc_gravcomp` — 모드와 무관하게 관절에 작용하는 일반화력 전부이며 실기와 같은 의미다.
- **torque 모드의 한계**: 서보 (`<position>` · `<velocity>`) actuator 는 torque 모드에서 `ctrlrange` clamp 가 꺼지고 `forcerange` / `actuatorfrcrange` 만 토크를 묶는다. 둘 다 없으면 한계가 없다.
- **`pd_feedforward`** (sim 전용): `kp·(q_d−q) − kd·qvel + tau_ff`. `values` = 위치 타겟, `feedforward[]` = `tau_ff` [N·m] (**중력 포함**, 빠지면 팔이 처진다), `kp[]` / `kd[]` = 런타임 게인 (sticky, all-or-nothing, 둘 다 ≥ 0). 현재 rtc 컨트롤러 스택은 이 값을 발행하지 못하고 테스트나 직접 publish 로만 도달한다.

근거 · 구현 노트는 [sim-rationale.md](../docs/reference/sim-rationale.md#제어-모드와-effort).

### Contact wrench 자동 발견

MJCF 에 `mjSENS_CONTACT` (MuJoCo ≥ 3.3.5) 가 있고 그룹 YAML 의 `contact_wrench.enabled: true` 이면 매 물리 step 마다 world-frame netforce 를 표준 `geometry_msgs/WrenchStamped` 로 발행한다.

요구사항:
- `<contact>` 는 `data="found force torque dist pos normal tangent"` + `num="1"` + `reduce="netforce"` (sensor_dim == 17).
- sensor 이름 suffix (기본 `_tip_contact` / `_contact`) 와 site 이름 suffix (기본 `_tip_ft_site` / `_ft_site`) 가 같은 target 토큰을 가리킨다 (`index_tip_contact` ↔ `index_tip_ft_site`).
- 토픽 이름은 `<topic_prefix>/<contact 센서 이름>/contact_wrench` — MJCF `<contact name>` 이 그대로 segment 가 된다 (예: `/leap_hand/index_tip_contact/contact_wrench`). suffix 는 site 를 찾는 데만 쓰인다.

| `reference_frame` | 회전 기준 | `frame_id` |
|---|---|---|
| `"body"` (기본) | ft_site 의 owning body | 그 body 의 MJCF 이름 |
| `"site"` | ft_site 자신 | 그 site 의 이름 |

- 둘은 site 가 body 대비 non-identity quat 을 가질 때만 갈라진다. 소비자가 `R_link(q)` 를 적용하는 URDF 프레임이 site 가 붙은 MJCF body 와 다르면 `"site"` 를 쓴다. 그 외의 값은 Initialize 실패다.
- **부호는 link-on-environment** — 손끝이 환경에 가하는 힘이며 실기 센서 lane 과 같은 부호다. 프로필에 per-tip 반전 (`force_sign: -1.0`) 을 두지 않는다.
- `publish_debug: true` 는 접촉점 / 깊이 lane 을 켠다. 마주 누르는지 같은 쪽으로 미는지를 가려야 하는 진단 fixture 용이고 기본은 OFF 다 (예: `integrated_bringup/config/ur5e_p1b/sim_overlays/fingertip_grasp_free.yaml`).
- 뷰어는 각 reference site 에서 주황색 화살표를 그린다 (`contact_wrench.visualize` 기본 `true`, `visualize_scale` 기본 `0.005` [m/N]). 토글은 **Shift+F**. 맨 `F` 는 MuJoCo 자체 접촉력으로 **다른 양**이다.

근거는 [sim-rationale.md](../docs/reference/sim-rationale.md#contact-wrench-lane).

### 뷰어 상태 오버레이

우상단에 항상 표시된다.

| 항목 | 설명 |
|---|---|
| Mode / Camera / Status | 동기 모드 (`sync`) / Free · Tracking · Fixed:name / running · PAUSED · perturb |
| RTF / Limit | 실시간 비율 / `max_rtf` 설정값 |
| Sim Time / Steps | 경과 시간 / 누적 제어 step |
| Contacts / World G | 활성 접촉 수 (on\|OFF) / 전역 중력 ON\|OFF |
| Robot Gravcomp | per-body 중력 보상이 켜진 그룹 인덱스 (torque 모드면 `none`) |
| Object | 현재 pool object 이름 (pool 활성 시). `o` 가 동작했는지 확인하는 유일한 관측면 |
| Integrator / Solver / Iterations / Residual | 현재 solver 설정과 통계 |
| Substeps | `n_substeps (substep_dt ms)` — 예: `4 (0.50ms)` |
| Physics Load | substep 루프 wall time / 제어 주기 (%). 100% 초과 시 실시간 유지 불가 |
| Frames | Link / Joint frame overlay ON/OFF |
| Perturb (\<body\>) | Ctrl+drag 외력 `\|F\|` (N) · `\|t\|` (Nm). 비활성 시 `-` |

### 뷰어 단축키

`viewer_input_enabled: false` (launch 인자 `viewer_input_enabled:=false`) 이면 world 를 바꾸는 입력 — 아래 "입력 차단" 열이 `차단` 인 키와 Ctrl+drag 퍼튜베이션 — 을 무시한다. 카메라 · 시각화 · 오버레이 · 스크린샷은 그대로 동작한다. 뷰어를 띄운 채 돌리는 평가 sim 이 실수로 누른 키에 흔들리지 않게 하려는 것이다. 무시된 키는 한 번 `[Viewer] Input disabled: key <name> ignored` 를 남긴다. 차단 집합의 SSoT 는 [viewer_input_policy.hpp](src/viewer/viewer_input_policy.hpp).

#### 키보드

| 키 | 기능 | 입력 차단 |
|---|---|---|
| **F1** | 도움말 오버레이 순환 (off → page1 → page2 → off) |  |
| Space | 일시정지 / 재개 | 차단 |
| Right (일시정지 중) | 한 스텝 진행 | 차단 |
| + / - | RTF 속도 2배 / 0.5배 | 차단 |
| R | 초기 자세로 리셋 (pool object 는 같은 자세로 다시 놓임) | 차단 |
| O | pool object 교체 ([Object Pool](#object-pool)) | 차단 |
| K | projectile ball 발사 (`/sim/launch_ball` 과 같음) | 차단 |
| TAB | 카메라 모드 순환 (Free → Tracking → Fixed[0..N-1] → Free) |  |
| Esc | 카메라 초기화 (Free 모드, 기본 거리/방위각) |  |
| Backspace | 시각화 옵션 초기화 |  |
| G | 전역 중력 ON/OFF (디버깅용; 로봇 gravcomp 와 무관) | 차단 |
| B | link frame overlay ON/OFF |  |
| N | 접촉 제약 ON/OFF | 차단 |
| I | integrator 순환 (Euler → RK4 → Implicit → ImplFast) | 차단 |
| S | solver 순환 (PGS → CG → Newton) | 차단 |
| ] / [ | solver 반복 횟수 x2 / /2 | 차단 |
| C / F | 접촉점 / 접촉력 표시 |  |
| Shift+F | contact wrench 화살표 (뷰어 force arrow) |  |
| T | 투명 시각화 |  |
| J / U | 관절 / 액추에이터 시각화 |  |
| E / W | 관성 / CoM 시각화 |  |
| L / A / X | 조명 / 텐던 / 볼록 껍질 시각화 |  |
| 0-5 / V | 지오메트리 그룹 토글 (V = 그룹 0 별칭) |  |
| F3 / F4 / F9 / F10 | RTF 그래프 / Solver 통계 / 센서 값 / 모델 정보 오버레이 |  |
| F5 / F6 / F7 / F8 | 와이어프레임 / 그림자 / 스카이박스 / 반사 |  |
| P | 스크린샷 저장 (PPM) |  |

#### 마우스

| 조작 | 기능 | 입력 차단 |
|---|---|---|
| Left drag | 카메라 궤도 |  |
| Shift + Left drag | 수평축 회전 |  |
| Right drag | 카메라 패닝 |  |
| Shift + Right drag | 수평 평면 패닝 |  |
| Scroll / Middle drag | 줌 인/아웃 |  |
| Dbl-click | 물체 선택 |  |
| Ctrl + Left drag | 퍼튜베이션 (토크) — status 의 `\|t\|` 가 실제 인가 토크 | 차단 |
| Ctrl + Right drag | 퍼튜베이션 (힘, 카메라 수직 평면) — status 의 `\|F\|` | 차단 |
| Ctrl + Shift + Right drag | 퍼튜베이션 (힘, 월드 수평 평면) | 차단 |

### 공개 헤더

노드 밖에서 라이브러리로 쓰는 경우다. 시그니처 · 동작은 헤더의 Doxygen 이 SSoT 다.

| 헤더 | 내용 |
|---|---|
| [mujoco_simulator.hpp](include/rtc_mujoco_sim/mujoco_simulator.hpp) | `MuJoCoSimulator` — 그룹 인덱스 기반 `SetCommand` / `GetPositions` / 콜백 등록 / `Pause` · `Resume` · `StepOnce` · `RequestReset` |
| [ros2_resource_provider.hpp](include/rtc_mujoco_sim/ros2_resource_provider.hpp) | `rtc::RegisterRos2ResourceProvider()` — MJCF 안의 `package://` URI 를 `ament_index_cpp` 로 해석 |
| [object_pool.hpp](include/rtc_mujoco_sim/object_pool.hpp) · [projectile_ball.hpp](include/rtc_mujoco_sim/projectile_ball.hpp) | pool / 공 설정 · 샘플링 순수 로직 |

## 설정

설정은 두 계층이다.

- [config/solver_param.yaml](config/solver_param.yaml) — **모든 sim launch 가 가장 먼저 로드**하는 agnostic 기본값 (`sync_timeout_ms` 50, `max_rtf` 1.0, `physics_timestep` 0.002, `n_substeps` 3, viewer 등) 과 solver 파라미터 SSoT. 우선순위는 XML `<option>` → bringup `mujoco_simulator.yaml` → 이 파일.
- [config/mujoco_default.yaml](config/mujoco_default.yaml) — launch 에 로드되지 않는 **템플릿**. 새 bringup 이 자기 `mujoco_simulator.yaml` 을 쓸 때 복사한다 (Object Pool · Object State · Projectile Ball · lane 의 전체 키와 주석이 여기 있다).

robot-specific 값 (`model_path`, `robot_response.groups`, joint 이름, 토픽, 게인) 은 bringup 에 있다 — 예: [ur5e_p1a/mujoco_simulator.yaml](../integrated_bringup/config/ur5e_p1a/mujoco_simulator.yaml).

### 노드 파라미터

기본값은 `solver_param.yaml` 기준이다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `model_path` | `""` | MJCF 경로 (**필수**, 비면 configure 가 `runtime_error`). 보통 `package://<pkg>/.../scene.xml` |
| `enable_viewer` | `true` | GLFW 뷰어 |
| `viewer_refresh_rate` | `60.0` | 뷰어 목표 refresh [Hz] |
| `viewer_input_enabled` | `true` | `false` 면 world 를 바꾸는 키와 Ctrl+drag 무시 (시작 시 1 회 읽음) |
| `viewer_camera.lookat` / `.distance` | `[0,0,0]` / `2.5` | free camera 가 바라보는 점 [m, world] 과 거리. 뷰어는 MJCF `<statistic>` 을 읽지 않으므로 로봇이 원점에 없으면 여기서 맞춘다 |
| `viewer_camera.azimuth_deg` / `.elevation_deg` | `90.0` / `-20.0` | 방위각 (90 이면 −y 쪽에서 +x 를 오른쪽에 둠) / 고각 (음수가 내려다봄) |
| `sync_timeout_ms` | `50.0` | 모든 robot 그룹 command 대기 한계 [ms]. 넘기면 받은 command 만 적용하고 step |
| `max_rtf` | `1.0` | 최대 실시간 비율 (`0.0` = 무제한) |
| `control_rate` | `500.0` | [Hz]. `physics_timestep == 1/control_rate` 가 아니면 기동 FATAL |
| `physics_timestep` | `0.002` | 제어 주기 [s]. `0.0` 이면 XML 값 사용 |
| `n_substeps` | `3` | 제어 주기당 물리 서브스텝 수. `substep_dt = physics_timestep / n_substeps` |
| `use_yaml_servo_gains` | `false` | `true` 면 YAML `servo_kp` / `servo_kd`, `false` 면 XML gainprm / biasprm. `false` + XML `<motor>` 이면 position 명령이 그대로 토크로 들어가 기동 때 경고 |
| `servo_kp` / `servo_kd` | (bringup 이 정함) | Position servo 게인 (글로벌). 그룹마다 DoF 가 다르면 그룹별로 지정한다 |
| `robot_response.groups` / `fake_response.groups` | `[]` | 물리 / LPF 그룹 이름 목록 |
| `solver.*` | — | [Constraint solver](#constraint-solver) |
| `object_pool.*` · `object_state.*` · `projectile_ball.*` | 비활성 | 아래 각 절 |
| `clock_lane.*` · `ball_contact_lane.*` | 비활성 | [측정용 CSV lane](#측정용-csv-lane) |

그룹별 파라미터 (`robot_response.<name>.` / `fake_response.<name>.`):

| 파라미터 | 설명 |
|---|---|
| `command_joint_names` | command 조인트 이름 (robot: XML actuator 와 양방향 일치). 미지정 시 `joint_names` 로 대체 |
| `state_joint_names` | state 발행 조인트. 빈 배열 = robot 은 XML 전체, fake 는 command 와 동일. 다중 그룹이면 그룹별로 명시한다 |
| `command_topic` / `state_topic` | 구독 / 발행 토픽 |
| `sensor_topic` / `sensor_names` | MJCF 센서 발행 (빈 문자열 = 비활성, 이름 `["auto"]` = XML 전체) |
| `contact_wrench.*` | `enabled` · `topic_prefix` · `sensor_name_suffixes` · `reference_site_suffixes` · `reference_frame` · `publish_state` · `publish_debug` · `allow_partial_discovery` · `visualize` · `visualize_scale` |
| `filter_alpha` | fake 전용 LPF 계수 (기본 `0.1`) |
| `servo_kp` / `servo_kd` | 그룹별 servo 게인 (미지정 시 글로벌 상속) |
| `initial_qpos` | 기동 · 리셋 자세. **robot 전용** ([초기 자세](#초기-자세-initial_qpos)) |
| `wait_for_command` | 기본 `true`. lock-step 이 이 그룹의 command 를 기다리는가 |
| `state_publish_divisor` | 기본 `1`. state · sensor · contact wrench 를 N step 에 한 번 발행 (물리는 매 step). fake 그룹은 1 만 |

`state_publish_divisor` > 1 인 그룹과 fake 그룹은 controller manager 의 `sim_sync_tick_devices` 에서 빼야 한다 — 넣으면 tick 이 그 그룹을 기다리느라 N−1 step 을 멈춘다. `wait_for_command: false` 인 그룹도 명령이 오면 적용한다.

### bringup YAML 예시

```yaml
mujoco_simulator:
  ros__parameters:
    model_path: "package://robot_descriptions/robots/ur5e_assm_v1/mjcf/scene_with_hand.xml"
    robot_response:
      groups: ["ur5e", "hand"]
      ur5e:
        command_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                              wrist_1_joint, wrist_2_joint, wrist_3_joint]
        state_joint_names:   [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                              wrist_1_joint, wrist_2_joint, wrist_3_joint]
        command_topic: "/ur5e/joint_command"
        state_topic: "/joint_states"
        # sensor_topic: "/ur5e/sim_sensors"
        # sensor_names: ["ft_sensor"]
      hand:
        command_joint_names: [thumb_cmc_aa, thumb_cmc_fe, ...]
        command_topic: "/hand/joint_command"
        state_topic: "/hand/joint_states"
    # XML 에 손이 없는 모델이면 hand 를 robot_response 에서 빼고 LPF 로:
    # fake_response:
    #   groups: ["hand"]
    #   hand: { command_joint_names: [...], command_topic: "/hand/joint_command",
    #           state_topic: "/hand/joint_states", filter_alpha: 0.1 }
```

XML 의 조인트가 모든 robot 그룹의 `state_joint_names` 합집합에 없으면 `WARNING: XML joint '...' not published by any group's state_joint_names` 가 한 번 출력된다. 다른 로봇 (kuka 등) 도 같은 형식으로 그룹만 바꾼다.

### 초기 자세 (`initial_qpos`)

그룹의 기동 자세이자 뷰어 `R` 리셋이 돌아가는 자세다. 우선순위:

| 순위 | 소스 | 비고 |
|---|---|---|
| 1 | `robot_response.<group>.initial_qpos` | rad, `command_joint_names` 순서. 크기가 그룹의 command joint 수와 다르면 Initialize 실패 |
| 2 | MJCF 의 **첫 번째** `<keyframe>` | YAML 키가 없을 때의 fallback |
| 3 | 0 | keyframe 도 없을 때 — arm 이라면 완전히 뻗은 자세 |

어느 소스가 이겼는지는 기동 로그가 그룹별로 알려준다.

```
[MuJoCoSimulator] [ur5e] initial positions from YAML initial_qpos
[MuJoCoSimulator] [p1b] initial positions from MJCF keyframe 'home'
```

**값이 전부 0 이어도 YAML 이 이긴다** — "설정됐는가" 는 값이 아니라 키의 존재로 판정한다. keyframe 이 있는 씬에 이 키를 쓰면 그 그룹에 한해 keyframe 은 무시된다.

```yaml
robot_response:
  groups: ["arm", "hand"]
  arm:
    command_joint_names: [j1, j2, j3, j4, j5, j6]
    initial_qpos: [0.0, -1.5708, 1.5708, -1.5708, 0.0, 0.0]   # 소수점 필수
```

> 모든 원소에 **소수점을 찍는다.** ROS 2 params YAML 은 시퀀스 타입을 원소에서 추론하므로 `[0, -1, 1]` 은 정수 배열이 되어 double 배열 선언과 어긋나고 노드 생성 시점에 예외로 죽는다 (`on_configure` 검증에 닿지 못한다).

### Constraint solver

`solver.*` 의 전체 키와 설명은 [config/solver_param.yaml](config/solver_param.yaml) 이 갖는다 (코드 fallback 과 shipped 값이 다르므로 표로 옮기지 않는다 — [근거](../docs/reference/sim-rationale.md#constraint-solver-설정)).

- **XML 우선순위**: MJCF `<option>` 에 명시된 속성은 YAML 값보다 우선하고, 없는 속성만 YAML 이 채운다. `<option solver="CG" iterations="50"/>` + YAML `solver: Newton, iterations: 100, cone: elliptic` → solver=CG, iterations=50 (XML), cone=elliptic (YAML).
- **런타임 변경 가능** (atomic, 뷰어 단축키 연동): `solver`, `integrator`, `iterations`, `tolerance`, `cone`, `jacobian`, `ls_iterations`, `ls_tolerance`, `noslip_iterations`, `noslip_tolerance`, `impratio`. 그 외 (`ccd_*`, `sdf_*`, flags, `contact_override`) 는 Initialize 에서 한 번 적용된다.
- `max_rtf`: `1.0` 실시간, `10.0` 10배속, `0.0` 무제한 (round-trip 속도가 곧 sim 속도).
- `n_substeps` 는 제어 주기는 두고 MuJoCo step 크기만 줄인다 (`substep_dt = 제어 주기 / n_substeps`). `ApplyCommand()` 는 루프 밖에서 1 회라 서브스텝 중 ctrl 은 변하지 않는다. 뷰어의 `>` 한 스텝은 N substep 전부 = 제어 주기 1 개다.

튜닝 레시피:

```yaml
# 매니퓰레이션 (contact slip 억제)
solver: {solver: "Newton", cone: "elliptic", impratio: 10.0, tolerance: 1.0e-10, noslip_iterations: 10}

# object grasp / pinch / in-hand
solver: {solver: "Newton", cone: "elliptic", impratio: 5.0, noslip_iterations: 15, iterations: 30,
         multiccd: true,    # finger-pad ↔ object 다점 접촉 — 핵심
         autoreset: false}  # 개발 중 numerical issue 가시화

# 안정적 보행
solver: {solver: "Newton", cone: "elliptic", impratio: 5.0, iterations: 50, tolerance: 1.0e-10}
```

### Object Pool

씬에 놓을 object 를 디렉토리에서 읽어 뷰어 **`o` 키**로 교체한다. 누를 때마다 기존 object 는 사라지고 설정 범위 안에서 새 object 가 생긴다. 후보는 `mj_compile` 전에 모두 attach 되고 `o` 는 park / unpark 만 하므로 모델은 기동 후 불변이다.

두 축의 곱으로 4 모드가 된다 — `selection` (`fixed` | `random`) × `pose` (`fixed` | `random`).

```yaml
mujoco_simulator:
  ros__parameters:
    object_pool:
      enabled: true
      directory: "package://robot_descriptions/object_sim"
      objects: ["apple", "banana", "coffeemug"]   # 키를 생략하면 디렉토리 전체
      fixed_object: "apple"                        # selection: fixed 일 때
      selection: "random"                          # fixed | random
      pose: "random"                               # fixed | random
      position:           [0.70, 0.00, 0.10]       # m, 중심
      position_variation: [0.10, 0.10, 0.00]       # m, ± 반폭
      rpy:                [0.0, 0.0, 0.0]          # rad, ZYX Euler
      rpy_variation:      [0.0, 0.0, 3.1416]       # rad, ± 반폭
      park_position: [0.0, 0.0, -50.0]
      seed: 0                                      # 0 = 매 실행 다름, >0 = 재현 가능
      avoid_repeat: true
      spawn_on_start: true
```

- 전체 키는 [mujoco_default.yaml](config/mujoco_default.yaml) 이 SSoT.
- **`objects:` 로 디렉토리 전체를 쓰려면 키를 생략한다.** `objects: []` 는 ROS 2 params YAML 이 빈 시퀀스 타입을 추론하지 못해 노드 생성 시점에 죽는다.
- 단위는 rad (degree 별칭 없음). `*_variation` 은 반폭 — 각 축을 `[중심 − variation, 중심 + variation]` 에서 균등 추출한다.
- 잘못된 설정 (후보 0 개, 없는 `directory` · `fixed_object` · `objects:` 항목, 허용 밖 `selection` / `pose`, 3 원소 아닌 `position`) 은 전부 Initialize 실패다.
- 후보 수에 비례해 비용이 든다 (16 개 초과 시 경고). `enabled: false` 면 비용이 0 이다.
- 씬이 이미 object 를 하드코딩했다면 (`scene_right_with_object.xml` 의 `apple_01`) pool 은 그것을 관리하지 않아 object 가 두 개가 된다 — 하드코딩 블록을 지운다.

근거: [sim-rationale.md](../docs/reference/sim-rationale.md#object-pool).

### Object State

씬 object 의 이름 / 기준 프레임 / pose 를 매 step 표준 `tf2_msgs/TFMessage` 하나로 발행한다 (`header.frame_id` = reference, `child_frame_id` = MJCF body 이름). **object = freejoint 를 가진 body 전부**, 단 park 된 pool 후보와 park 된 projectile ball 은 제외한다 (매 tick 판정). `/tf` 가 아닌 일반 토픽이라 sim ground truth 가 실기 TF 트리에 섞이지 않는다 — RViz 에서 보려면 `/tf` 로 재발행하는 노드를 붙인다.

```yaml
object_state:
  enabled: true
  topic: "/sim/object_transforms"   # 상대 이름이면 노드 네임스페이스 아래
  reference_body: ""                # "" = MuJoCo world
  frame_id: ""                      # "" = reference_body 이름 (world 면 "world")
  publish_divisor: 1                # N step 에 한 번 발행. rate = control_rate / N
```

- pose 는 `p_rel = R_ref^T (p_obj − p_ref)`. **`reference_body` 는 의식적으로 고른다** — base 에 180° yaw 가 있는 배치 (ur5e_p1b) 에서 world 와 base 를 헷갈리면 object 가 거울상 위치에 찍히는데 값은 그럴듯하다. 모델에 없는 이름은 startup 실패다.
- `""` (world) 는 sim 이 아는 절대 좌표를 주지만 `world` 는 대개 bringup 의 TF 트리에 없다. 로봇 base 를 주면 좌표가 곧바로 로봇 기준이다. ur5e_p1b 는 world 다.
- `frame_id` override 는 같은 프레임을 MJCF body 이름과 URDF 링크 이름이 다르게 부를 때만 쓴다.

```bash
ros2 topic echo /sim/object_transforms --once
```

### Projectile Ball (발사 공)

파지 · 추적 실험용으로 **공 하나를 던져 주는** 기능이다. `projectile_ball.enabled: true` 이면 파싱한 MJCF spec 에 공을 직접 붙이므로 씬 XML 을 고치지 않는다. **YAML 은 반경 · 질량만** 정하고 나머지 물성 (관성 분포 · 반발 · 마찰 · 항력 · 양력) 은 `ball_type` preset 이 정한다. 물성은 기동 시 고정이고 발사는 qpos / qvel 만 바꾼다. 공 geom 은 씬 default 와 무관하게 group 0 · 노란색이다.

| 상태 | 위치 | 접촉 | 발행 |
|---|---|---|---|
| park (기동 직후 · `R` · `/sim/reset_ball`) | `park_position_m` | 끔 | 안 함 (`object_state` 에서도 빠짐) |
| 발사 (`K` · `/sim/launch_ball`) | `spawn_position_m` | `collision_contype` / `collision_conaffinity` | `sample_rate_hz` |
| 발사 (`/sim/launch_ball_at`) | 요청의 `position` | 〃 | 〃 |

발사 속도는 `launch_direction` (수평, 정규화) 을 z 축으로 `launch_azimuth_variation_deg` 만큼 돌린 방향으로, `launch_angle_deg ± launch_angle_variation_deg` 만큼 올려 `launch_speed_m_s ± launch_speed_variation_m_s` 크기로 준다. `launch_noise` 가 `uniform` 이면 `*_variation` 은 반폭, `normal` 이면 표준편차다. **`launch_direction` 에 z 성분이 있으면 Initialize 가 실패한다** (앙각은 `launch_angle_deg` 만 정한다).

초기 회전 `launch_spin_rad_s` 는 **발사 프레임** 기준이다 — x = 발사 방향, z = 월드 위, y = z × x (왼쪽). backspin (공이 뜸) 은 −y, topspin 은 +y, sidespin 은 z 다.

#### `ball_type` preset

값과 출처의 코드 쪽 SSoT 는 [src/projectile_ball.cpp](src/projectile_ball.cpp) 의 preset 상수이고, 바꿀 때는 이 표도 함께 고친다. (추정) 은 측정값을 찾지 못한 값이다. 근거와 모델링 [sim-rationale.md](../docs/reference/sim-rationale.md#projectile-ball).

| 물성 | `tennis` | `hard` (야구공류) | `beanbag` |
|---|---|---|---|
| I / (m r²) | 0.55 [Cross 2003] | 0.378 [Brody] | 0.4 (추정) |
| 반발계수 (강체 바닥) | 0.75 [ITF 0.745] | 0.55 [ASTM F1887, 추정] | 0.1 (추정) |
| 미끄럼 마찰 μ | 0.6 [Cross 2003] | 0.5 [Sawicki 2003] | 0.5 (추정) |
| 비틀림 마찰 / r | 0.05 (추정) | 0.02 (추정) | 0.3 (추정) |
| 구름 저항 / r | 0.02 (추정) | 0.005 (추정) | 0.5 (추정) |
| 항력 Cd | 0.55 [자유비행 0.51 – 풍동 0.65] | 0.5 [Cross, Kensrud] | 0.5 (추정) |
| 양력 C_L(S) | S / (0.981 + 2.022 S) [Stepanek 1988] | S / (0.667 + 1.48 S) [Sawicki 2003 에 맞춤] | 없음 |

S = r·|ω⊥| / |v|. Cd 는 상수다 (던지는 속도 2–10 m/s 는 drag crisis 아래).

- 공은 `condim=6` + `priority=100` 이라 어떤 상대와 부딪혀도 공의 반발 · 마찰이 그대로 적용된다. **priority ≥ 100 인 geom 이 씬에 있으면 Initialize 가 실패한다.**
- `aerodynamics: true` 는 항력과 Magnus 양력을 공에만 건다 (MuJoCo 내장 fluid 모델은 로봇 링크까지 끌어서 쓰지 않는다). 57 g 테니스공을 5 m/s · 45° 로 던지면 2 m 지점에서 진공 궤적보다 약 5 cm 낮다.
- `solver.contact_override.enable: true` 이면 preset 반발이 적용되지 않는다. `solver.island: false` 에서는 씬 어딘가에 깊게 파묻힌 접촉이 공 접촉을 흔들 수 있다 — 공이 이유 없이 튀면 씬의 관통부터 확인한다.

#### 설정

```yaml
projectile_ball:
  enabled: false
  body_name: "projectile_ball"
  ball_type: "tennis"         # tennis | beanbag | hard
  radius_m: 0.0335
  mass_kg: 0.057
  collision_contype: 2        # 상대 geom 과 양방향으로 겹치도록 고른다 (아래)
  collision_conaffinity: 1
  aerodynamics: false         # 항력 + Magnus 양력 (공에만)
  spawn_position_m: [0.0, 0.0, 0.5]
  park_position_m: [0.0, 0.0, -50.0]
  launch_direction: [1.0, 0.0, 0.0]   # z 는 0 이어야 한다
  launch_noise: "uniform"     # uniform (variation = 반폭) | normal (variation = 표준편차)
  launch_angle_deg: 0.0
  launch_angle_variation_deg: 0.0
  launch_azimuth_variation_deg: 0.0
  launch_speed_m_s: 1.0
  launch_speed_variation_m_s: 0.0
  launch_spin_rad_s: [0.0, 0.0, 0.0]            # 발사 프레임 (x 발사방향, y 왼쪽, z 위)
  launch_spin_variation_rad_s: [0.0, 0.0, 0.0]
  seed: 0                     # 0 = 매 기동 random_device
  publish:
    sample_rate_hz: 100.0     # sim 시간 기준 (RTF 와 무관하게 sim 초당 횟수)
    frame_id: "world"
    ground_truth_topic: "/sim/ball/ground_truth"
    camera_topic: "/sim/ball/camera_position"
    position_noise_stddev_m: [0.0, 0.0, 0.0]   # 0 인 축은 노이즈 없음
```

MuJoCo 는 `(contype_A & conaffinity_B) || (contype_B & conaffinity_A)` 이면 두 geom 을 충돌시킨다. 손끝 mesh 가 `contype=1, conaffinity=2` 라면 공은 `contype=2, conaffinity=1` 로 둔다. 이 값이면 기본 마스크 (1/1) 인 바닥과도 충돌한다.

#### stamp — sim 시간축을 wall 에 사상한 값

두 공 토픽 (`ground_truth_topic` · `camera_topic`) 의 `header.stamp` 는 발행 순간의 `now()` 가 아니라 **발사 순간을 기준으로 sim 시간축을 wall 에 얹은 값**이다: `발사 wall 시각 + (sim 시각 − 발사 sim 시각) / max_rtf`. 샘플 간격이 sim 축과 정확히 `1/sample_rate_hz` 이고, 발행 순간의 stepper wake 지터가 stamp 에 실리지 않는다 (소비자의 "5 점을 0.1 s 안에" 같은 init 창이 이 지터로 깨지던 것을 막는다). 소비자가 알아야 할 사실:

- **epoch 은 wall 그대로** (ROS system time, `/clock` 없음) 다. 두 토픽이 같은 stamp 를 받으므로 truth ↔ camera stamp 짝짓기는 영향이 없다.
- **stamp 는 wall 을 수십 ms 앞설 수 있다** (그 비행 안에서 stepper 가 따라잡은 양 — 위상 오차). 그래서 sim 에서는 소비자의 future 허용치를 같은 자릿수로 둔다 (ball_perception `max_future_skew_s` 0.1 s, 컨트롤러 `sim.io.future_tol`).
- 발사 전에 쌓인 sim ↔ wall 지연은 비행의 stamp 에 들어가지 않는다 (anchor 가 발사 순간).
- **`max_rtf: 0` (무제한) 이면 종전처럼 발행 순간의 `now()`** 라 stamp 축의 동역학이 틀린다 — 추정기를 붙이는 run 은 `max_rtf: 1.0` 으로 돌린다.

측정 (stamp 간격 p05 / p95 / max, 기각한 clamp 변형) 은 [sim-rationale.md](../docs/reference/sim-rationale.md#stamp--sim-시간축을-wall-에-사상한-값).

#### 지정 상태 발사 (`/sim/launch_ball_at`)

`spawn_position_m` · 각도 · 속도 · spin 은 **샘플링 경로**의 입력이다. 한 투척을 고정한 채 다른 변수만 바꿔 재실행하려면 `/sim/launch_ball_at` 으로 `(p0, v0, ω)` 를 world 프레임에서 직접 준다. RNG 를 소비하지 않아 같은 숫자를 두 번 보내면 궤적이 비트 단위로 같고, 시드 sweep 에 끼워 넣어도 sweep 이 그대로 재생된다. 시뮬레이터 미구성 · 모델에 공 없음 (`enabled: false`) · 비유한값이면 `accepted: false` 이고 직전 발사는 그대로 비행한다. 필드 · 거부 집합의 SSoT 는 [LaunchBall.srv](../rtc_msgs/srv/LaunchBall.srv) 주석이다.

```bash
ros2 service call /sim/launch_ball std_srvs/srv/Trigger   # 설정 분포에서 샘플링
ros2 topic hz /sim/ball/ground_truth                       # 발사 후 ~sample_rate_hz × RTF

# 지정 상태 발사 — 같은 숫자면 궤적이 비트 단위로 같다
ros2 service call /sim/launch_ball_at rtc_msgs/srv/LaunchBall \
  "{position: {x: 4.0, y: 0.0, z: 1.75}, velocity: {x: -5.0, y: 0.0, z: 3.0}, \
    angular_velocity: {x: 0.0, y: -40.0, z: 0.0}}"

ros2 service call /sim/reset_ball std_srvs/srv/Trigger   # 이후 발행이 멈춘다
```

### 측정용 CSV lane

둘 다 측정 run 에서만 켠다. `csv_path` 가 비면 lane 이 켜져 있어도 거부된다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `clock_lane.enabled` / `.csv_path` | `false` / `""` | 완료 step 마다 `step, sim_time_sec, steady_ns, dropped_total` — sim 클럭과 steady 클럭의 위상 오차 측정 |
| `clock_lane.drain_rate_hz` | `20.0` | 노드가 링을 비우는 주기. `dropped_total > 0` 이면 올리고 다시 잰다 |
| `ball_contact_lane.enabled` / `.csv_path` | `false` / `""` | 공 접촉 episode (연속 substep 구간) 마다 한 줄 |

`ball_contact_lane` 열: `step`, `begin/end_sim_time_sec`, `substeps`, `impulse_{x,y,z}` (**공에 가해진** 충격량 [N·s], world), `peak_force_n`, `pos_*` · `vel_*` (첫 접촉 순간 공 상태), `first_body`, `distinct_bodies`, `dropped_total`. 설계 근거는 [sim-rationale.md](../docs/reference/sim-rationale.md#ball-contact-truth).

## 디렉토리

```
rtc_mujoco_sim/
├── include/rtc_mujoco_sim/
├── src/
│   └── viewer/
├── config/
├── launch/
└── test/
```

공개 헤더 · 시뮬레이터 · SimLoop · 노드 · pool · 공은 `src/` 와 `include/`, GLFW 뷰어는 `src/viewer/`, 테스트 fixture 는 `test/fixtures/` 에 있다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| configure 실패 `model_path` | `model_path` 가 비었다 | `params_file` 이나 `model_path:=` 로 준다 |
| configure 실패 (그룹 없음) | `params_file` 없이 띄워 `robot_response.groups` 가 비었다 | bringup 의 `mujoco_simulator.yaml` 을 `params_file` 로 준다 |
| 노드 생성 시점에 예외 | `initial_qpos` 에 정수 (`0`) 가 섞였거나 `objects: []` | 소수점을 찍는다 / `objects` 키를 생략한다 |
| 기동 FATAL | `physics_timestep` ≠ `1/control_rate` | 둘을 맞춘다 |
| 매 step 이 50 ms 멈춘다 | 명령하지 않는 robot 그룹이 command 를 기다린다 | 그 그룹에 `wait_for_command: false` |
| controller tick 이 멈춘다 | `state_publish_divisor` > 1 이거나 fake 인 그룹이 `sim_sync_tick_devices` 에 있다 | CM 설정에서 뺀다 |
| 뷰어에 scene 의 floor / 일부 geom 이 안 보인다 | group 3 이상은 기본 숨김. include 한 `<default>` 의 group 을 상속했을 수 있다 | `F10` 으로 `geom_group` 확인, 보여야 할 geom 에 group 을 명시 |
| 공이 안 그려진다 / 이유 없이 튄다 | [Projectile Ball](#projectile-ball-발사-공) 의 접촉 마스크 · 관통 접촉 참고 | `collision_contype` / `conaffinity` 와 씬의 관통부 확인 |
| 모든 object 가 거울상 위치다 | `object_state.reference_body` 를 world 와 base 중 잘못 골랐다 | [Object State](#object-state) |

## 더 읽기

- [docs/reference/sim-rationale.md](../docs/reference/sim-rationale.md) — 동기 루프 · tick, effort 정의, 외력 CoM 보정, contact wrench 부호, 공 preset 근거, stamp 실측, lane 설계
- [rtc_msgs/srv/SetExternalWrench.srv](../rtc_msgs/srv/SetExternalWrench.srv) · [LaunchBall.srv](../rtc_msgs/srv/LaunchBall.srv) — 서비스 계약
- [include/rtc_mujoco_sim/mujoco_simulator.hpp](include/rtc_mujoco_sim/mujoco_simulator.hpp) — C++ API Doxygen
- [docs/testing.md](../docs/testing.md) — 테스트 명령 · sensor matrix. 이 패키지는 `colcon test --packages-select rtc_mujoco_sim` (GLFW 렌더링 자체는 헤드리스 제약으로 테스트하지 않는다)
- [integrated_bringup](../integrated_bringup/README.md) — 이 시뮬레이터를 쓰는 robot 별 launch / YAML
- [rt-optimization.md](../docs/rt-optimization.md)

## 의존성

[package.xml](package.xml) 이 SSoT. 라이선스 MIT.

| 구분 | 패키지 | 용도 |
|---|---|---|
| ROS 2 | `rclcpp` · `rclcpp_lifecycle` | 노드, lifecycle |
| ROS 2 | `std_msgs` · `sensor_msgs` · `nav_msgs` · `geometry_msgs` · `tf2_msgs` · `std_srvs` | 토픽 / 서비스 타입 |
| ROS 2 | `rtc_msgs` · `rtc_base` | `JointCommand` · `SimSensorState` · `LaunchBall` · `SetExternalWrench`, 공통 유틸 |
| ROS 2 | `ament_index_cpp` · `tinyxml2` | `package://` 해석, MJCF `<option>` 파싱 (solver 우선순위) |
| 런타임 | `robot_descriptions` · `rtc_tools` | MJCF 모델 · 메시, 세션 디렉토리 헬퍼 |
| 외부 | **MuJoCo ≥ 3.2** | 물리 엔진 (필수 — 없으면 노드 빌드 생략). Object Pool 이 mjSpec attach API 를 쓰므로 3.0 / 3.1 은 컴파일되지 않는다 |
| 외부 | GLFW 3 | 3D 뷰어 (선택, 없으면 headless 전용) |

MuJoCo 설치와 coverage 빌드:

```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/
# MuJoCo 를 못 찾으면: colcon build 에 --cmake-args -Dmujoco_ROOT=/opt/mujoco-3.x.x
# coverage (gcov/lcov): colcon build --packages-select rtc_mujoco_sim --cmake-args -DENABLE_COVERAGE=ON -DBUILD_TESTING=ON
```
