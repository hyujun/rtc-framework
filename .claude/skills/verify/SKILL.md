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
- 종료: `pkill -INT -f "sim_ur5e_p1a.launch"` (SIGINT — 세션 CSV flush 보장).

## Controller switch

```bash
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: [demo_task_controller], deactivate_controllers: [demo_wbc_controller], strictness: 1, timeout: {sec: 1}}"
```

- 이름: `demo_joint_controller` / `demo_task_controller` / `demo_wbc_controller` / `demo_compliance_controller` / `demo_inference_controller`.
- **Pure deactivate 불가** (single-active D-A1) — 항상 교체 대상을 activate 에 지정.
- 토픽: joint/task → `/<ctrl>/p1a/grasp_state`, wbc → `/<ctrl>/p1a/wbc_state`. 활성 컨트롤러는 `/rtc_cm/active_controller_name` (latched).

## GUI (Tkinter, 실제 DISPLAY 사용)

```bash
DISPLAY=:1 ros2 run integrated_bringup demo_controller_gui # 창 제목 "Demo Controller GUI"
```

- 스크린샷: `xdotool`/`scrot` 미설치 — `xwininfo -root -tree` 로 window id 찾고 `xwd -id <id> -silent -out x.xwd` 후 XWD 헤더 수동 파싱으로 PNG 변환 (PIL 은 xwd 직접 못 읽음; 100-byte big-endian 헤더 + ncolors×12 skip, 32bpp BGRX).
- GUI 는 latched `active_controller_name` 기준으로 owned 토픽에 rewire — 활성 컨트롤러가 50 Hz 로 계속 발행하므로 fake 데이터 주입 시엔 (1) 실제 컨트롤러를 다른 것으로 전환해 대상 publisher 를 lifecycle-gate 시키고 (2) `active_controller_name` 에 그 이름을 fake 발행(transient_local) 후 (3) 침묵 토픽에 `ros2 topic pub`. 복원은 실제 switch 2회 (CM 이 latched name 재발행).

## 학습 정책을 sim 에서 돌리기 (`demo_inference_controller`, ur5e_p1b 전용)

```bash
export RTC_POLICY_DIR=/path/to/<policy-export-dir> # 모델은 repo 밖
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py \
    sim_overlay:=inference_pole enable_viewer:=false use_cpu_affinity:=false
# 기동 후 switch_controller 로 demo_inference_controller 활성화 (위 절)
```

- **`sim_overlay:=inference_pole` 없이는 잡을 것이 없다** — 출하 p1b 씬은 테이블 + 무작위 메시고, 정책은 바닥 위 원통 1개 + 학습 reset 자세에서 훈련됐다. overlay 는 그 씬을 params 로 얹고 (공유 `mujoco_simulator.yaml` 무수정), 이름이 안 풀리면 launch 가 실패한다 (조용히 출하 씬으로 돌면 그 run 의 모든 수치가 다른 씬을 서술한다).
- 기동 확인 2줄: `[inference] N observed link(s) in '<frame>'` (정책이 관측하는 프레임) · `[inference] policy loaded: ... (decimation ...)`. 후자가 없으면 `RTC_POLICY_DIR` 미설정이고, 그때는 `allow_missing_model: true` 라 **자세만 유지**한다 (고장처럼 안 보인다).
- **출하 YAML 을 안 고치고 컨트롤러 키를 바꾸려면** overlay 에 `integrated_rt_controller: ros__parameters: demo_inference_controller: <yaml.경로>: <값>` 을 넣는다 — `ApplyControllerParamOverrides` 가 그 ROS 파라미터를 컨트롤러 YAML 트리에 꽂는다. **경로는 YAML 그대로** 여야 한다 (예: `inference.policy_frame` — 한 단계 얕게 쓰면 조용히 무시된다). 반영 여부는 위 기동 로그로 확인.
- 판독: `<session>/controllers/demo_inference_controller/inference_diag.csv` — `held`+`hold_reason` (이 컨트롤러는 **모든 실패가 hold** 라 사유 없이는 정상과 구분 불가), `policy_step`/`inference_count` (decimation 대로인지), `reach_phase`·`tip_distance`, `object_*` (policy_frame 기준 — 프레임이 어긋나면 부호로 드러난다), `arm_lag_max`, `force_<tip>`. 같은 폴더의 `<device>_state.csv` 가 관절 lane.
- **팔 lag 은 접촉이 지배한다**: 자유 운동 구간의 하한은 sim 위치 서보의 kv/kp (0.2 s) × 명령 속도이고, 물체에 막히면 그 3 배까지 포화한다. lag 수치를 인용할 땐 첫 접촉 시각으로 구간을 갈라 보고한다.
- 종료는 자식 노드에 SIGINT (`pkill -INT -f integrated_rt_controller; pkill -INT -f mujoco_simulator_node`) — `ros2 launch` 에 한 번 보낸 SIGINT 가 45 s 안에 안 끝난 적이 있다. `-9` 는 CSV flush 를 날린다.

## Session CSV / plots

- 세션 루트: `~/ros2_ws/rtc_ws/logging_data/<YYMMDD_HHMM>/` (`rtc_tools.utils.session_dir.resolve_logging_root`).
- 컨트롤러 CSV: `controllers/<ctrl>/<instance>.csv` — Compute() 활성 중에만 append (activity-gated).
- Plot: `ros2 run rtc_tools plot_rtc_log <csv> --no-show` — `--save-dir` 미지정 시 그 CSV 를 담고 있는 세션의 `plots/` 에 저장하므로 (Agg 강제) 세션 밖으로 빼고 싶을 때만 `--save-dir <dir>`.

## Gotchas

- `pkill -f "ros2 topic pub"` 은 자기 쉘 커맨드라인도 매칭해 self-kill (exit 144) — PID 지정 kill 사용.
- **`pkill -f "<launch>.launch.py"` 는 launch 만 죽이고 노드는 살려 둔다** — `mujoco_simulator_node` 와 `integrated_rt_controller` 가 그대로 남아 계속 발행한다. 그 상태로 다시 launch 하면 **새 씬을 띄운 줄 알면서 옛 sim 과 이야기한다**. 재기동은 세 단계다: launch 를 죽이고 → `pgrep -f "<node> --ros-args"` 로 남은 노드를 **PID 로** 죽이고 → SIGKILL 이 흘린 `/dev/shm/fastrtps_*` 를 지운다 (Fast DDS 참가자는 **정상 종료할 때만** 자기 세그먼트를 지운다 — SIGKILL 뒤엔 남는다).
- idle sim (무접촉) 은 grasp/pull 값 전부 0, `ft_*` inference 컬럼 NaN — 데이터 없는 figure 는 정상.
- **p1a 의 shipped scene 에는 잡을 물체가 없다** (`ur5e_assm_v1/mjcf/scene_with_hand.xml` = floor plane 만), 그래서 거기서는 **접촉이 필요한 경로 (contact_stop latch, grasp detection true) 를 sim 으로 검증 불가**. 물체가 있는 `hand_description/robots/demo/mjcf/scene_with_object.xml` 은 다른 hand 모델을 `<include>` 하므로 `model_path` 만 바꿔 끼우면 joint-name 매핑이 깨져 *오독 가능한* 결과가 나온다 — 도달 불가를 그대로 보고하는 것이 옳다.
- **p1b 는 다르다** — `scene_with_table.xml` + `object_pool` 로 작업대 위에 물체가 하나 올라와 있으므로 접촉 검증이 **가능하다**. 다만 손을 그냥 내리는 것으로는 안 된다: palm-down 자세에서 손가락은 옆을 향하고 **`wrist_2_link` · `l_palm_link` 가 상판에 먼저 닿아** 팔이 `shoulder_lift ≈ -0.72` 에서 멈추며, 물체까지 손 아래를 받쳐 손끝을 띄운다. 실제로 손끝에 하중을 거는 가장 짧은 경로는 **손을 닫아 둔 채 `/sim/set_external_wrench` 로 물체를 위로 밀어올리는 것** (`body_name: pool_<obj>_object`, +z 60 N) — 실측으로 `l_ring_tip_contact` 에 39 N 이 걸렸다. **진짜 파지(2 접촉 + 유효 pull estimate)가 필요하면 그 방법으로는 안 되고**, 물체를 손끝 높이에 맞춰 놓고 **팔을 전혀 안 움직인 채** 출하 palm-down 자세에서 엄지+검지만 닫는다. 2026-09-12 에 그렇게 처음 성공했고, 다섯 번 실패하면서 나온 제약이 이것들이다 — 다음에 다시 세울 때 이 순서로 확인한다:
  1. **손끝 링크의 접촉 반경은 약 10 mm 이지 브래킷 간격이 시사하는 22 mm 가 아니다.** 발행되는 `l_*_tip_bracket_actual` 프레임은 접촉면이 아니다. 눈대중 배치는 전부 빗나가거나 파묻힌다 — `mujoco.mj_geomDistance` 로 **모든** 로봇 충돌 geom 에 대해 탐색할 것.
  2. **손 높이를 관통하는 기둥형 물체는 배치 가능한 (x,y) 가 없다.** 열린 손이 이미 그 부피를 차지해 palm·dip·mcp 링크에 먼저 박힌다. 손끝만 사는 z 밴드 (기본 자세에서 약 0.335~0.365 m) 만 차지하는 **디스크**여야 한다.
  3. **tip 이 아닌 링크의 접촉은 contact 센서가 못 본다** — 따라서 grasp lane 에도 pull estimator 에도 안 잡힌다. 실측: 100 kg 물체가 수 mm 밀려나는 동안 손끝 4개 토픽이 전부 `0.000 N`. 성공한 파지에서도 접촉 링크 3개 중 1개(`l_index_pip_linkage_b`, 셋 중 가장 깊은 −2.9 mm)는 무센서였다.
  4. **자유 물체는 밀려난다.** 0.5 kg 기둥은 엄지가 그대로 밀어내고 (2.63 N 한 스텝 뒤 0 N), 5 kg 도 한 run 에 2 cm 걸어간다. 100 kg 받침이면 상판 마찰(약 1 kN)이 손이 낼 수 있는 무엇보다 커서 **손을 재는 실험**이 된다.
  5. **`force_saturation: 15.0` N 을 넘기면 그 손끝은 contact 집합에서 빠진다** — 더 세게 쥐면 접촉이 *줄어든다*. 정지 조건은 peak 이 아니라 밴드여야 한다.
  6. **엄지와 손가락은 폐쇄율 스케줄이 다르다.** 엄지 손끝은 제 구간에서 y 로 약 10 cm 쓸고, 검지 손끝은 25 % 까지 거의 안 움직이다가 x 로 6 cm 를 간다. 하나의 스칼라로 둘을 몰면 엄지가 포화(14 N)할 때 검지는 아직 1 mm 모자란다 — **손가락마다 램프를 따로** 준다.

  그리고 접촉을 **어떻게 판정하지 않을지**도 위와 같은 무게로 정해져 있다. **joint_states 를 kinematic FK 로 재현해 접촉을 추정하지 말 것**: 이 손은 폐쇄 체인이라 수동 linkage 관절이 qpos 에 따로 있고, 그것을 0 으로 둔 재구성은 손끝이 상판을 1 cm 파고든 것처럼 보이는 **허상**을 만든다. 그리고 **팔이 어디까지 내려가는지는 아직 안 갈렸다**: `scene_with_table.xml` 의 배너 주석은 palm 을 `z = 0.049` 까지 내릴 수 있다고 적는데(자유공간 유도) 위 `shoulder_lift ≈ -0.72` 는 상판·물체에 막힌 실측이고, 둘을 맞춰 본 적이 없다. 컨트롤러나 정책이 물체를 못 잡을 때 **원인이 그쪽인지 씬인지 가르려면 이 실측이 선행**한다.
- **compliance 외부 렌치의 sim 도달 가능성은 프로필마다 다르고, 2026-09-11 에 바뀌었다.** `FromPullEstimate` 는 `vtcp.valid` 없이 publish 하지 않는데, 출하 `virtual_tcp_mode` 가 세 프로필 모두 `"constant"` 가 되면서 (`1b29c1d5`; iiwa7_leap 은 종전 `"disabled"`, p1a·p1b 는 `"centroid"`) 그 게이트는 **이제 어디서나 열린다** — `kConstant` 는 손끝 참여와 무관하게 offset 이 유한하면 `valid` 다 (`integrated_bringup/include/integrated_bringup/support/virtual_tcp.hpp`). 남는 축은 **지문력 lane 이 있느냐**다: `ur5e_p1b`·`iiwa7_leap` 은 sim 에서 손끝 contact wrench 를 받고 (`sim.yaml` 의 `devices.<hand>.backend.fingertip_wrench_topics`, 부호는 실기와 같은 finger-on-object), **`ur5e_p1a` 는 그 lane 도 잡을 물체도 없다**. ⇒ 종전의 "iiwa7_leap sim 에서 한 샘플도 안 나온다" 는 **더 이상 근거가 아니다**. **2026-09-12 실측으로 닿는 것을 확인했다** (`ur5e_p1b`, 아래 p1b 항목의 파지 절차): pull estimate `valid=1` · `contact_mask=0b011`(thumb+index) · `|F̂|` 6.8 N · `leakage_bound` 0.57 N · `basis_source=REFERENCE`, 그리고 compliance 쪽 `compliance_diag.csv` 가 `wrench_valid=1`, `|f|` 최대 **7.96 N**, `bias_calibrated=1`(D-A5 100 샘플 완주 — 2026-09-04 실기에서는 22 tick 뿐이라 끝내 false 였다), **compliance §10.7 램프 α 0→1 이 0.688 s**, `|x̃|` 최대 **1.46 cm** (15 cm envelope 의 10 %, `disp_limited`·`vel_limited` 0). ⇒ **compliance §7.4 envelope·bias 표류·램프는 이제 sim 에서 관측 가능하다.** 단 **파지가 전환을 못 넘긴다** (`#504`): joint→compliance 스위치 직후 sum|f| 18.5 N → 0 N, 두 손끝이 `contact_on_threshold` 0.5 N 아래로 떨어지기까지 **238 ms**, compliance 가 본 유효 pull tick 은 **213개 / 0.494 s** 뿐이다. 그러므로 **정상상태 envelope 소진(α>0 지속)을 보려면 `#504` 가 먼저 닫혀야 한다** — 이 sim 은 기전 재현과 회귀 red 까지이고 정상상태 hand-guiding 이 아니다. 한편 프레임 전이(vTCP ⇄ tool0) 경로는 여전히 출하 설정으로 도달 불가인데, 이유가 바뀌었다: `"constant"` 는 매 tick `is_vtcp=true` 로 고정이라 kind 가 flip 하지 않는다. 그걸 겨냥한 테스트는 그대로 `set_gains()` 로 모드를 명시적으로 바꾸고 그 전제를 단언해야 한다 (`test_compliance_admittance_coupling` 의 kind-change 테스트가 그 형태).
- **force_pi FSM 은 물체 없이 GRASP 하면 ~1.3s 만에 Idle 로 자동 복귀**한다 (approach ramp 완료/abort). phase 가 non-Idle 인 창을 노리는 검증은 순차 CLI 로 놓친다 — 한 rclpy 프로세스 안에서 `grasp_command` 호출 직후 대상 호출을 연달아 실행할 것.
- **`demo_inference_controller` 는 정책 파일 없이도 뜬다** (`allow_missing_model: true`) — 그때는 매 tick **활성화 시점에 래치된 자세**를 유지한다. 이 경로의 회귀 센서는 hold 자체가 아니라 **drift** 다: 측정 q 를 매 tick 그대로 명령하면 position servo 오차가 0 이라 토크가 안 나오고 팔이 중력에 처진다 (실측 15 s 에 0.0147 rad, 시작 자세에서 0.2 rad 이탈). 래치 후 실측은 15 s 에 **0.000000 rad**. `/ur5e/joint_states` 를 15 s 받아 첫 샘플과의 최대 편차를 보면 된다 — **QoS 는 BEST_EFFORT depth 1** 이어야 하고 (기본 RELIABLE 구독자는 "incompatible QoS" 경고와 함께 0 메시지를 받는다) sim 은 lock-step 이라 `/ur5e/joint_command` 를 직접 구독하는 것보다 이쪽이 확실하다.
- 기본 active 컨트롤러는 launch 마다 다르다 — p1a 는 `demo_wbc_controller`, **p1b 는 `demo_joint_controller`**. p1b 에서 `DemoWbcController timing:` 로그가 안 보이는 것은 기동 실패가 아니다.
- 백그라운드 빌드/테스트와 Stop hook 의 colcon 동시 실행 금지 — foreground `tail --pid=<pid> -f /dev/null` 로 대기.
