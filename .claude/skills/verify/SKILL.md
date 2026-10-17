---
name: verify
description: Runtime verification recipe for this repo — launch the headless MuJoCo sim, switch controllers, observe topics/GUI/CSV/plots. Use when a change needs end-to-end runtime evidence beyond colcon test.
---

# RTC runtime verification recipe

검증 대상 surface 는 보통 넷 중 하나: ROS 토픽, demo GUI, 세션 CSV, `plot_rtc_log` figure.

## Launch (headless sim)

```bash
cd ~/ros2_ws/rtc_ws && source src/rtc-framework/repo_scripts/scripts/setup_env.sh
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_viewer:=false
```

- 백그라운드 실행 시 로그를 파일로 tee. 기동 완료 신호: `DemoWbcController timing:` 주기 로그 (기본 활성 = demo_wbc_controller).
- 종료: 노드를 이름으로 `kill -INT $(pidof integrated_rt_controller mujoco_simulator_node)`, launch 는 띄울 때 잡은 PID 로 `kill -INT` (SIGINT — 세션 CSV flush 보장). `pkill -f` · `pgrep -f` 는 쓰지 않는다 — 아래 Gotchas.
- `g1_p1b` (G1 상체 + proto_1b 오른손): `ros2 launch integrated_bringup sim_g1_p1b.launch.py enable_viewer:=false use_cpu_affinity:=false`. 컨트롤러는 `demo_joint_controller` (기본 활성) · `demo_dualarm_controller` 둘, 군은 `g1` (17) · `p1b` (10). 기동 완료 신호는 `RtControllerNode reached Active state`. 목표는 `/demo_joint_controller/{g1,p1b}/joint_goal` (`rtc_msgs/RobotTarget`, **한 번만** 보낸다), TF 는 `/demo_joint_controller/transforms` (부모 `pelvis`). `/g1/joint_states` 는 BEST_EFFORT 다 — 구독은 sensor-data QoS 로.
- `demo_dualarm_controller` (`g1_p1b` 전용, 설계 [DemoDualArmController](../../../integrated_bringup/docs/demo-dualarm-controller.md)): 위 switch 형태로 `demo_joint_controller` 와 맞바꾼다. 목표 `/demo_dualarm_controller/{right_hand,left_hand}/task_goal` (`goal_type: task`, `task_target`, `header.frame_id` = `world`·`pelvis`·`torso_link` 또는 빈 값 = 과제의 base frame) · `g1/joint_goal` (17 전부) · `p1b/joint_goal` (10 전부) 를 `ros2 topic pub --once` 로 **한 번만** 보낸다. 판독: TF `/demo_dualarm_controller/transforms` (**측정** pose, 부모 `pelvis`) 와 `controllers/demo_dualarm_controller/dualarm_diag.csv` 의 `clik_ran` · `converged` · `<task>_err_lin` · `fault_latched` (`plot_rtc_log <그 CSV> --stats` 가 풀이 · 과제별 오차 · 목표 계수를 요약한다). fault 가 걸렸으면 `ros2 service call /rtc_cm/reset_fault rtc_msgs/srv/ResetFault "{controller_name: demo_dualarm_controller}"`. sim 에는 E-STOP trigger 가 없어 E-STOP 은 gtest 로만 본다.

## Controller switch

```bash
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: [demo_task_controller], deactivate_controllers: [demo_wbc_controller], strictness: 1, timeout: {sec: 1}}"
```

- 이름: `demo_joint_controller` / `demo_task_controller` / `demo_wbc_controller` / `demo_compliance_controller` / `demo_inference_controller` / `demo_dualarm_controller` (g1_p1b 만).
- **Pure deactivate 불가** (single-active D-A1) — 항상 교체 대상을 activate 에 지정.
- 토픽: joint/task/compliance → `/<ctrl>/p1a/grasp_state`, wbc → `/<ctrl>/p1a/wbc_state`. 활성 컨트롤러는 `/rtc_cm/active_controller_name` (latched).

## GUI (Tkinter, 실제 DISPLAY 사용)

```bash
DISPLAY=:1 ros2 run integrated_bringup demo_controller_gui # 창 제목 "Demo Controller GUI"
```

- `--robot <config 디렉토리 이름>` 으로 profile 을 고른다 (기본 `ur5e_p1a`). `g1_p1b` 는 Dual Arm 탭이 더 뜬다 — 과제 목표는 `demo_dualarm_controller` 가 활성일 때만 나가고, 상태 줄은 세션의 `dualarm_diag.csv` 꼬리에서 읽으므로 **GUI 를 sim 과 같은 머신에서** 띄운다 (다른 세션을 보게 하려면 `--session <dir>`).
- 위젯을 실제로 눌러 보는 자동 확인: 같은 프로세스에서 `DemoControllerGUI(robot=...)` 를 만들고 `rclpy.spin` 을 스레드로 돌린 뒤, `gui.root.after(0, fn)` 으로 Tk 스레드에서 `Button.invoke()` · entry 편집을 한다 (버튼은 위젯 트리를 돌며 `cget("text")` 로 찾는다). 결과는 GUI 밖에서 읽는다 (`ros2 param get`, 세션 CSV).
- 창만 찍기: `PIL.ImageGrab.grab(bbox=(rootx, rooty, rootx+w, rooty+h), xdisplay=":1")` — 찍기 전에 `root.lift()` + `-topmost`. 탭 아래쪽은 스크롤 canvas 를 `yview_moveto(1.0)` 한 뒤 한 번 더 찍는다.
- 스크린샷: `xdotool`/`scrot` 미설치 — `xwininfo -root -tree` 로 window id 찾고 `xwd -id <id> -silent -out x.xwd` 후 XWD 헤더 수동 파싱으로 PNG 변환 (PIL 은 xwd 직접 못 읽음; 100-byte big-endian 헤더 + ncolors×12 skip, 32bpp BGRX).
- GUI 는 latched `active_controller_name` 기준으로 owned 토픽에 rewire — 활성 컨트롤러가 50 Hz 로 계속 발행하므로 fake 데이터 주입 시엔 (1) 실제 컨트롤러를 다른 것으로 전환해 대상 publisher 를 lifecycle-gate 시키고 (2) `active_controller_name` 에 그 이름을 fake 발행(transient_local) 후 (3) 침묵 토픽에 `ros2 topic pub`. 복원은 실제 switch 2회 (CM 이 latched name 재발행).

## 학습 정책을 sim 에서 돌리기 (`demo_inference_controller`, ur5e_p1b 전용)

**착수 전 [integrated_bringup/README.md §Scene overlay](../../../integrated_bringup/README.md#scene-overlay-sim_overlay--출하-씬을-건드리지-않고-바꾸기) 를 읽는다** — 어느 overlay 가 정책의 학습 씬인지, 이름이 안 풀릴 때의 동작, 정책 로드를 가르는 기동 로그 2줄은 그 절이 소유한다 (모델이 안 잡혀도 에러 없이 올라온다 — 아래 Gotchas). 이 절은 그 명령에 headless 인자를 더한 형태와 실행자에게만 필요한 것만 갖는다.

```bash
export RTC_POLICY_DIR=/path/to/<policy-export-dir> # 모델은 repo 밖
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py \
    sim_overlay:=inference_pole enable_viewer:=false use_cpu_affinity:=false
# 기동 후 switch_controller 로 demo_inference_controller 활성화 (위 절)
```

- **출하 YAML 을 안 고치고 컨트롤러 키를 바꾸려면** overlay 에 `integrated_rt_controller: ros__parameters: demo_inference_controller: <yaml.경로>: <값>` 을 넣는다 — `ApplyControllerParamOverrides` 가 그 ROS 파라미터를 컨트롤러 YAML 트리에 꽂는다. **경로는 YAML 그대로** 여야 한다 (예: `inference.policy_frame` — 한 단계 얕게 쓰면 만들어지기만 하고 읽히지 않는다. configure 로그에 `[param-override]` WARN 이 남으니 기동 로그에서 0건인지 본다). 반영 여부는 README 의 기동 로그로 확인.
- 판독: `<session>/controllers/demo_inference_controller/inference_diag.csv` — `held`+`hold_reason` (이 컨트롤러는 **모든 실패가 hold** 라 사유 없이는 정상과 구분 불가), `policy_step`/`inference_count` (decimation 대로인지), `reach_phase`·`tip_distance`, `object_*` (policy_frame 기준 — 프레임이 어긋나면 부호로 드러난다), `arm_lag_max`, `force_<tip>`. 같은 폴더의 `<device>_state.csv` 가 관절 lane.
- **팔 lag 은 접촉이 지배한다**: 자유 운동 구간의 하한은 sim 위치 서보의 kd/kp × 명령 속도이고 (`ur5e_p1b` 는 `mujoco_simulator.yaml` 서보 게인으로 0.05 s — 2026-09-24 전 세션은 MJCF 게인 0.2 s), 물체에 막히면 그 3 배까지 포화한다. lag 수치를 인용할 땐 첫 접촉 시각으로 구간을 갈라 보고한다.
- 종료는 자식 노드에 SIGINT (위 Launch 절의 `pidof` 형태) — `ros2 launch` 에 한 번 보낸 SIGINT 가 45 s 안에 안 끝난 적이 있다. `-9` 는 CSV flush 를 날린다.

## Inactive 컨트롤러 파라미터 set → switch → TF

`virtual_tcp_*` 처럼 **Inactive 에서만** set 을 받는 파라미터의 확인 순서 (`demo_joint` · `demo_task` · `demo_compliance`). 기본 활성은 `demo_wbc_controller` 라 그 셋은 처음에 Inactive 다.

```bash
C=/demo_task_controller/demo_task_controller
ros2 param get $C virtual_tcp_offset                       # demo_shared.yaml 값
ros2 param set $C virtual_tcp_offset "[0.0, 0.0, 0.15]"    # Inactive → successful
# 위 Controller switch 로 demo_task_controller 활성화
ros2 param set $C virtual_tcp_offset "[0.0, 0.0, 0.1]"     # Active → "can only be set while this controller is inactive"
```

- 판정: `/demo_task_controller/transforms` 의 `virtual_tcp_actual` 과 `tool0_actual` 의 거리가 set 한 offset 크기다 (constant 모드). `ros2 topic echo` 는 6 frame 을 다 찍으므로 한 메시지를 받아 두 translation 의 거리를 계산한다.
- 배열은 double 로 쓴다 — `"[0, 0, 1]"` 은 정수 배열이라 rclpy 가 타입 불일치로 거부한다.
- set 한 값은 노드 수명 동안 남는다. 확인 뒤 원래 값으로 되돌리거나 sim 을 내린다.
- 활성화 순간 팔이 튀지 않는다는 판정은 이 레시피가 아니라 gtest (`VtcpParamSetInactiveSeedsAfterActivate`) 가 갖는다.
- GUI 는 Control 탭의 Virtual TCP 패널이다: active 컨트롤러를 고르면 `Apply VTCP` 가 닫히고, Inactive 컨트롤러에서 Apply 하면 `current:` 줄이 다시 읽은 값으로 바뀐다.

## Session CSV / plots

- 세션 루트: `~/ros2_ws/rtc_ws/logging_data/<YYMMDD_HHMM>/` (`rtc_tools.utils.session_dir.resolve_logging_root`).
- 컨트롤러 CSV: `controllers/<ctrl>/<instance>.csv` — Compute() 활성 중에만 append (activity-gated).
- **세션 폴더 이름은 분 단위다.** 같은 분 안에 sim 을 다시 띄우면 두 번째 실행이 **같은 폴더의 CSV 에 이어 쓴다** (header 는 다시 쓰지 않으므로 `tick` 이 되돌아가는 행이 경계다). unit 마다 sim 을 새로 띄우는 드라이버는 `logging_data/$(date +%y%m%d_%H%M)` 이 없어질 때까지 기다린 뒤 띄운다.
- Plot: `ros2 run rtc_tools plot_rtc_log <csv> --no-show` — `--save-dir` 미지정 시 그 CSV 를 담고 있는 세션의 `plots/` 에 저장하므로 (Agg 강제) 세션 밖으로 빼고 싶을 때만 `--save-dir <dir>`.

## Gotchas

- **launch 는 기동할 때 `logging_data/` 를 `max_log_sessions` (출하 10) 로 잘라 오래된 세션을 지운다.** 평가 세션이 쌓여 있으면 확인용으로 한 번 띄우는 것도 그 원시 로그를 지운다 (두 번 일어났다). 그동안에는 모든 기동에 `max_log_sessions:=<현재 개수 + 여유>` 를 주고 (rig · 드라이버의 launch 줄에 박아 둔다), 판정에 쓸 세션은 unit 이 끝나는 즉시 unit 폴더로 복사한다.
- **켜 둔 sim 은 디스크를 채운다.** 무장된 catching 컨트롤러는 ARMED 대기 중에도 tick 마다 CSV 를 쓴다 (약 3.4 GB/h — 6.5 시간에 22 GB). 육안 확인용으로 넘길 때 이것을 알리고, 확인이 끝나면 바로 내린다. plot 은 투척 직후의 짧은 세션에서 뽑는다. 정리할 때는 `YYMMDD_HHMM` 세션만 지운다 — 옆의 `*.json` · `stats/` · `ur_plot/` 은 사용자 파일이다.
- **프로세스를 커맨드라인 (`-f`) 으로 찾지 않는다.** 에이전트의 셸 도구는 명령 전체를 `bash -c '…'` 로 돌리므로 그 셸의 커맨드라인이 패턴을 담는다 — `pkill -f <노드>` 는 자기 셸을 죽이고 (exit 144 — `ros2 topic pub` 에서 한 번, sim 종료에서 한 번), `pgrep -f` 는 래퍼 셸 PID 를 노드와 섞어 돌려준다. 노드는 `pidof <실행 파일 이름>` 으로 찾고 (argv[0] 의 전체 이름 — `pgrep -x` 처럼 15 자에서 잘려 0 건이 되지 않는다), 자기가 띄운 것은 띄울 때의 `$!` 로 죽인다 — 셸 호출이 매번 새 셸이면 `… & echo $! > <scratch>/x.pid; wait` 로 남겨 둔다. `ros2 launch` · `ros2 topic pub` 은 실행 파일이 python 이라 `pidof` 로 가를 이름이 없다 — `$!` 만 쓴다.
- **launch 프로세스만 죽이면 노드는 살아 남는다** — `mujoco_simulator_node` 와 `integrated_rt_controller` 가 그대로 남아 계속 발행한다. 그 상태로 다시 launch 하면 **새 씬을 띄운 줄 알면서 옛 sim 과 이야기한다**. 재기동은 세 단계다: launch 를 죽이고 → 남은 노드를 `pidof` 로 찾아 **PID 로** 죽이고 → SIGKILL 이 흘린 `/dev/shm/fastrtps_*` 를 지운다 (Fast DDS 참가자는 **정상 종료할 때만** 자기 세그먼트를 지운다 — SIGKILL 뒤엔 남는다).
- idle sim (무접촉) 은 grasp/pull 값 전부 0, `ft_*` inference 컬럼 NaN — 데이터 없는 figure 는 정상.
- **p1a 의 shipped scene 에는 잡을 물체가 없다** (`ur5e_assm_v1/mjcf/scene_with_hand.xml` = floor plane 만), 그래서 거기서는 **접촉이 필요한 경로 (contact_stop latch, grasp detection true) 를 sim 으로 검증 불가**. 물체가 있는 `hand_description/robots/demo/mjcf/scene_with_object.xml` 은 다른 hand 모델을 `<include>` 하므로 `model_path` 만 바꿔 끼우면 joint-name 매핑이 깨져 *오독 가능한* 결과가 나온다 — 도달 불가를 그대로 보고하는 것이 옳다.
- **p1b 는 접촉 검증이 가능하다** — 단 출하 씬은 받침대 씬 (작업대 없음, 2026-10-09) 이라 `sim_overlay:=fingertip_grasp` (고정 디스크) 또는 `fingertip_grasp_free` (손을 따라오는 디스크) 로 띄운다 (overlay 가 작업대 씬을 스스로 고른다). 두 overlay 의 배치·형상은 출하된 것이다 — **다시 탐색하지 말 것** (기하 근거는 물체 파일 주석, 결정은 `#504` AC 6). 파지 규칙 넷은 실측에서 나왔고, 전문·실측·기각 가설은 [references/p1b-fingertip-fixture.md](references/p1b-fingertip-fixture.md):
  1. compliance 를 쓸 것이면 **먼저 compliance 로 전환하고 그 다음 손을 닫는다** (`#504` — 파지한 채 전환하면 네 손끝 전부 0 N).
  2. **닫는 양은 힘으로 정한다** — 작은 스텝으로 접근해 손끝 2~7 N 밴드에서 멈춘다. `force_saturation: 15.0` N 을 넘기면 그 손끝은 contact 집합에서 빠진다.
  3. **같은 target 을 재전송하지 말 것** — 재전송마다 램프가 측정 위치에서 다시 시작해 파지를 잠깐 푼다 (CSV 는 멀쩡한데 토픽은 0 N 으로 읽힌다).
  4. 자유 디스크는 매 스텝 **힘이 작은 손가락을 한 칸** 진행시키고, 유지는 드라이버가 1 s 주기 grip 서보로 닫는다 (이때의 *새* target 은 3 의 대상이 아니다).
  - **접촉을 joint_states 의 kinematic FK 로 추정하지 말 것** — 폐쇄 체인의 수동 linkage 관절을 0 으로 둔 재구성은 손끝이 상판을 파고든 허상을 만든다. 팔이 어디까지 내려가는지 (씬 배너의 `z = 0.049` vs 실측 `shoulder_lift ≈ -0.72`) 는 아직 안 갈렸다.
  - 고정 디스크로는 "pull 이 valid 를 지속하는가" 를, 자유 디스크로는 `#177` crit#6 ① 의 grip sweep 을 판정할 수 없다 — 정착 힘이 복원 스프링에 묶이고, 다음 결정은 튜닝이 아니라 fixture 설계다 (`#861`).
- **compliance 외부 렌치는 `ur5e_p1b` · `iiwa7_leap` sim 에서 도달 가능하고 `ur5e_p1a` 는 불가** (지문력 lane 도 잡을 물체도 없다). sim 이 보여 주는 것은 compliance §7.4 envelope · bias 표류 · 램프의 기전과 회귀 red 까지다 — **정상상태 envelope 소진 (α>0 지속) 은 못 본다** (렌치가 유효해지면 팔이 고정 물체에서 걸어 나간다). vTCP ⇄ tool0 프레임 전이는 출하 `"constant"` 모드에서 flip 하지 않으므로 그 테스트는 `set_gains()` 로 모드를 명시하고 전제를 단언한다. 실측·근거: references 파일 §compliance.
- **force_pi FSM 은 물체 없이 GRASP 하면 ~1.3s 만에 Idle 로 자동 복귀**한다 (approach ramp 완료/abort). phase 가 non-Idle 인 창을 노리는 검증은 순차 CLI 로 놓친다 — 한 rclpy 프로세스 안에서 `grasp_command` 호출 직후 대상 호출을 연달아 실행할 것.
- **`demo_inference_controller` 는 정책 파일 없이도 뜬다** (`allow_missing_model: true`) — 그때는 매 tick **활성화 시점에 래치된 자세**를 유지한다. 이 경로의 회귀 센서는 hold 자체가 아니라 **drift** 다: 측정 q 를 매 tick 그대로 명령하면 position servo 오차가 0 이라 토크가 안 나오고 팔이 중력에 처진다 (실측 15 s 에 0.0147 rad, 시작 자세에서 0.2 rad 이탈). 래치 후 실측은 15 s 에 **0.000000 rad**. `/ur5e/joint_states` 를 15 s 받아 첫 샘플과의 최대 편차를 보면 된다 — **QoS 는 BEST_EFFORT depth 1** 이어야 하고 (기본 RELIABLE 구독자는 "incompatible QoS" 경고와 함께 0 메시지를 받는다) sim 은 lock-step 이라 `/ur5e/joint_command` 를 직접 구독하는 것보다 이쪽이 확실하다.
- 기본 active 컨트롤러는 launch 마다 다르다 — p1a 는 `demo_wbc_controller`, **p1b 는 `demo_joint_controller`**. p1b 에서 `DemoWbcController timing:` 로그가 안 보이는 것은 기동 실패가 아니다.
- 검증을 마쳤으면 **sim 을 내린 뒤** `verify-changes.sh --run` 을 돌린다 — sim 옆에서는 빌드하지 않는다 (기다리는 법은 CLAUDE.md §Claude Code).
