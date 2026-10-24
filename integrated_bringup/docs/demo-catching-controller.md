# DemoCatchingController

날아오는 물체를 잡는 동적 포구 컨트롤러다. vision 이 예측한 궤적을 받아 계획기가 포구점과 시각을 정하고, RT 가 확장 CLIK 으로 팔을 몰고, 손은 시퀀서가 닫는다. config_key 는 `demo_catching_controller`, YAML 은 `ur5e_p1b` · `iiwa7_leap` 이 싣는다.

수학은 각 헤더 (`rtc_controllers/include/rtc_controllers/catching/`) 가 갖는다. 이 문서는 운용 표면만 적는다. 시행 러너와 평가 명령은 [README](../README.md) 의 Catching sim trials 절을 본다.

## 동작

- **무장되면** 팔은 스스로 `planner.wait_pose` 로 관절공간 homing 하고 한 투척마다 한 순환을 돈다. 손은 시퀀서가 소유한다 (`diagnostic.hand_step: true` 인 프로파일만 시퀀서 없이 계단 목표를 받는다 — 측정용, 출하 false).
- **대기 자세.** `planner.wait_pose_source: yaml` (출하) 이면 YAML 의 `planner.wait_pose`, `current` 면 이 컨트롤러로 switch 한 순간의 팔 자세다. `current` 는 margin 을 적용한 관절 상자 밖이면 거부되어 YAML 값을 유지한다. 채택한 자세는 diag 의 `wait_pose_*` 열과 INFO 로그 한 줄로 본다.
- **plan 원천.** 계획기 스레드 (`catching.planner.enabled`) 또는 `diagnostic.oracle_plan` (사용자가 잰 고정 포구점 하나 — 테스트 · sim 용) 이 같은 plan box 에 쓰고, RT 는 매 tick 한 번 읽어 판정한다.
- **계획기 모드.** 두 선택자가 configure 에서 정해지고 활성화 동안 바뀌지 않는다: `planner.search.mode` (`grid` | `nlp`, 출하 `grid`), `planner.segment.mode` (`closed_form` | `mpc` | `mpc_docking`, 코드 기본 `closed_form`, 출하 `mpc`). 기동 로그가 모드를 한 줄씩 말한다 (`search mode: …` · `segment mode: …`). `nlp` × `closed_form` 조합은 park 된다.

## 무장과 E-STOP

- **무장 채널.** 읽기 · 쓰기 파라미터 `catching.enable` (기본 `false`). 노드 이름의 네임스페이스와 이름이 둘 다 `demo_catching_controller` 다.

```bash
ros2 param set /demo_catching_controller/demo_catching_controller catching.enable true
```

- **컨트롤러가 스스로 내린다.** E-STOP 발동 · 해제와 fault 래치에서 RT tick 이 latch 를 내리므로, 해제 후 재개는 다시 `true` 로 올려야 한다. 활성화만으로는 무장되지 않는다.
- **E-STOP 과 fault 는 별개 경로다.** `/rtc_cm/clear_estop` 은 컨트롤러 fault 래치를 풀지 않고 `/rtc_cm/reset_fault` 는 E-STOP 을 풀지 않는다. fault 는 CLIK 실패로 끝난 시행이 연속 `supervisor.n_qp` 회이거나 정지 · 복귀가 `supervisor.deadline.stop_s` · `return_s` 를 넘길 때 선다. 원인은 CSV `fault_cause` 와 WARN 한 줄에 남는다.
- **`ResetFault` 는 팔이 정지해 있어야 받는다** (명령 속도 0, 속도 lane 판독 가능, 측정 `|q̇| ≤ supervisor.homing.qd_tol`). 거부되면 latch 가 남고 CSV `fault_reset_refused` 와 WARN 이 이유를 남긴다.

## Park (활성화 거부)

값이 정해지지 않은 키가 있으면 configure 를 거부하지 않고 park 한다 — CM 은 한 컨트롤러 실패로 전체를 거부하므로, 거부하면 로봇이 안 뜬다. park 된 컨트롤러는 `on_activate` 가 거부하고 상태 토픽도 없다.

- **실기.** claim 한 device 가 전부 `mujoco_native` 임을 증명하지 못하면 real-arm 으로 판정한다. 이 컨트롤러가 소비하는 키 (`control_rate` · `robot.hand.*` · `supervisor.deadline` · `joint_cmd.lag` 등) 나 catch frame (`urdf.extra_frames.<catch_frame>.provisional`), `robot.arm.qdd_provisional` 에 provisional · TBD 가 있으면 park 다.
- **sim.** 소비 키가 TBD 면 park, 값이 정해졌는데 범위 · 순서 · 일관성이 틀리면 configure 를 거부한다. sim 에서 `robot.arm.qdd_provisional` 은 경고다.
- 팔 · 손의 속도 lane 을 읽을 수 없으면 (위치만 읽히고 속도 slot 에 hole) ARMED 진입 · homing 도착 · 대기 자세 채택 · fault reset 이 성립하지 않고 publish 스레드가 `<arm|hand> velocity lane UNREADABLE …` WARN 을 낸다.
- 옛 키 `robot.arm.accel_limits_*` · `joint_cmd.accel_constraint: box` 가 남아 있으면 `kRemovedKey` 로 park 하고 ERROR 가 지운 키와 새 키를 댄다.

## 설정 파일

주 파일 `config/<robot>/controllers/demo_catching_controller.yaml` 이 `catching/` 아래 다섯 조각 — `search_grid.yaml` · `search_nlp.yaml` · `planner_closed_form.yaml` · `segment_mpc.yaml` · `segment_mpc_docking.yaml` — 을 `include:` 하고, CM 이 한 트리로 합친 뒤 override 를 적용한다 ([include 규칙](../../rtc_controller_manager/README.md#컨트롤러-yaml-을-여러-파일로-나누기--include)). 조각은 항상 모두 include 되고 무엇이 도는지는 위 두 선택자가 정한다. 선택된 기능의 키만 읽고 요구하며, 모르는 철자는 configure 에서 거부된다. 키별 설명은 YAML 주석이 갖는다.

## 입력

- **vision.** `catching.io.traj_topic` 의 `PointCloud2` 예측 궤적을 컨트롤러가 직접 구독한다 (best_effort, depth 1 — 백로그가 생기면 가장 최신만 쓴다). 필드는 이름으로 찾고, 이름 · 타입 · 개수가 다르면 거부하며 사유별 카운터가 남는다. stale 은 steady 수신 나이로만 판정한다.
- **vision frame.** vision world 는 모델 frame `scene_world` 다 ([Task 목표의 기준 frame](../README.md#task-목표의-기준-frame-779)). 수신 궤적 · 공분산을 그 frame 의 배치로 한 번 모델 world 로 옮긴다. 기동 로그가 변환을 한 줄로 말한다 (`vision frame 'world' → model world via 'scene_world': yaw … deg, t […] m`). 옛 키 `catching.io.arm_base_frame` · `base_T_world` 는 없고 남아 있으면 configure 가 거부한다.
- **실기에서는 vision lane 이 inert 다.** `scene_world` 가 `_base.yaml` · `robot.yaml` 에 없으므로 configure 는 성공하지만 구독하지 않고 ERROR `vision lane INERT: …` 를 한 번 낸다. 카메라 보정 값을 그 기동의 `urdf.extra_frames.scene_world` 로 넣어야 켜진다.
- 씬이 바뀌면 world 정의도 바뀐다. 받침대 씬 이전 (2026-10-09 전) 세션을 `rtc_tools` 로 다시 분석하면 0.7 m 어긋나므로 그 세션은 당시 revision 의 config 로 분석한다.

## 상태 토픽

`/<config_key>/catching_state` (`rtc_msgs/CatchingState`, `KEEP_LAST(1)`). 필드는 한 번 동결돼 있고 값만 채워진다. 모든 tick 이 body 를 싣는다 — E-STOP · stale · plan 없음 · abort tick 도 발행하고, 그 tick 에 계산하지 않은 블록은 직전 값을 남기지 않고 지운다. 그래서 값이 고정돼 보이면 컨트롤러가 정말 같은 값을 다시 계산한 것이다.

## tick 레코드 CSV

`<session>/controllers/demo_catching_controller/catching_diag.csv` (`logs:` 의 `integrated_bringup/CatchingDiagLog`). 상태 토픽과 같은 POD 한 벌에서 나오므로 파일의 숫자와 화면의 숫자가 갈리지 않는다. tick 마다 한 행이라 tick 간극은 드롭된 행이다. 컬럼 목록의 SSoT 는 [catching_diag_log_pod.hpp](../include/integrated_bringup/logging/catching_diag_log_pod.hpp) 다.

```bash
ros2 run rtc_tools plot_rtc_log <session>/controllers/demo_catching_controller/catching_diag.csv
```

기준 대 실현 가속도, 추종 오차, solve time, 슈퍼바이저 모드를 한 시간축에 그린다 (E-STOP 과 fault 구간은 색으로 음영). 구간 lane (`segment_*`) 은 `outcome_source` 뒤 17 열이고, lane 이 돌지 않은 tick 과 `closed_form` 에서는 0 이다. `mode: mpc` 에서 "기준" 은 `ref_x` 가 아니라 `segment_p_d_*` · `segment_v_ff_*` 다. `segment_event` · `segment_refusal` 의 enum 값은 pod 헤더가 갖는다. 열 이름이 `decel_*` 인 옛 recording 은 리더가 `segment_*` 로 읽는다.

`planner_events.csv` 는 계획기 wake 마다 한 행이다 (후보 수 · IK 수 · 게이트별 탈락 수 · 교체 결정 · 탐색 시간 · 수신 → 게시 지연). `outcome` 은 `idle` · `no_input` · `published` · `superseded` · `held` 중 하나다. 상태 메시지에 없는 것은 이 CSV 에만 있고, plan 이 없을 때 그 사유 (`segment_outcome` · `segment_core_reason`) 도 여기서 본다. 열 해석과 판정 규칙은 [catching-analysis-rules.md](../../docs/catching-analysis-rules.md#plot_rtc_log) 가 갖는다.

## GUI

`demo_controller_gui` 의 Catching 탭에 공 발사 · 손 단계 · 컨트롤러 상태 패널이 있다 ([demo-controller-gui.md](demo-controller-gui.md)). 상태 패널은 모드 · 사유, `segment mode:` 와 `search mode:` 줄, 입력 lane (n · generation · sequence · 수신 나이 · 지평, 0 이 아닌 거부 카운터), plan, 추종 오차 · CLIK 상태, Arm / Disarm 을 보인다. 관측된 무장과 요청된 무장을 따로 보여준다 — tick 이 E-STOP · fault 에서 latch 를 내리므로 파라미터 set 성공이 무장의 증거가 아니다. `segment mode` · `search mode` 는 컨트롤러의 읽기 전용 미러 파라미터 (`planner.segment.mode` · `planner.search.mode`) 에서 읽으므로 park 됐거나 configure 전이면 `unknown (planner.segment.mode not read)` 로 남는다. 구간을 따르는지 · 대기 구간 · lane 사건은 화면에 없고 `catching_diag.csv` 의 `segment_*` 열로 본다.

## 읽기 전용 미러 파라미터

`hand.q_open` · `hand.q_pre` · `hand.q_close` · `planner.wait_pose*` · `planner.freeze.*` · `planner.search.mode` · `planner.segment.mode` · `joint_cmd.lag.*` · 팔 예산 층 (`reference.*` · `robot.arm.qdd_max`) · 예측 격자 · segment MPC 의 예산 · 비용 키 등은 컨트롤러가 선언하는 읽기 전용 미러다 (선택된 기능의 것만). 오프프로세스 분석기가 YAML 이 아니라 컨트롤러가 읽은 값을 쓰게 하려는 것이므로, 적용 확인은 `ros2 param get` 으로 한다 — 기동 로그에는 안 찍힌다.

```bash
ros2 param get /demo_catching_controller/demo_catching_controller reference.omega
```
