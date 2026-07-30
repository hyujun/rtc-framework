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

- 이름: `demo_joint_controller` / `demo_task_controller` / `demo_wbc_controller`.
- **Pure deactivate 불가** (single-active D-A1) — 항상 교체 대상을 activate 에 지정.
- 토픽: joint/task → `/<ctrl>/p1a/grasp_state`, wbc → `/<ctrl>/p1a/wbc_state`. 활성 컨트롤러는 `/rtc_cm/active_controller_name` (latched).

## GUI (Tkinter, 실제 DISPLAY 사용)

```bash
DISPLAY=:1 ros2 run integrated_bringup demo_controller_gui   # 창 제목 "Demo Controller GUI"
```

- 스크린샷: `xdotool`/`scrot` 미설치 — `xwininfo -root -tree` 로 window id 찾고 `xwd -id <id> -silent -out x.xwd` 후 XWD 헤더 수동 파싱으로 PNG 변환 (PIL 은 xwd 직접 못 읽음; 100-byte big-endian 헤더 + ncolors×12 skip, 32bpp BGRX).
- GUI 는 latched `active_controller_name` 기준으로 owned 토픽에 rewire — 활성 컨트롤러가 50 Hz 로 계속 발행하므로 fake 데이터 주입 시엔 (1) 실제 컨트롤러를 다른 것으로 전환해 대상 publisher 를 lifecycle-gate 시키고 (2) `active_controller_name` 에 그 이름을 fake 발행(transient_local) 후 (3) 침묵 토픽에 `ros2 topic pub`. 복원은 실제 switch 2회 (CM 이 latched name 재발행).

## Session CSV / plots

- 세션 루트: `~/ros2_ws/rtc_ws/logging_data/<YYMMDD_HHMM>/` (`rtc_tools.utils.session_dir.resolve_logging_root`).
- 컨트롤러 CSV: `controllers/<ctrl>/<instance>.csv` — Compute() 활성 중에만 append (activity-gated).
- Plot: `ros2 run rtc_tools plot_rtc_log <csv> --no-show` — `--save-dir` 미지정 시 그 CSV 를 담고 있는 세션의 `plots/` 에 저장하므로 (Agg 강제) 세션 밖으로 빼고 싶을 때만 `--save-dir <dir>`.

## Gotchas

- `pkill -f "ros2 topic pub"` 은 자기 쉘 커맨드라인도 매칭해 self-kill (exit 144) — PID 지정 kill 사용.
- idle sim (무접촉) 은 grasp/pull 값 전부 0, `ft_*` inference 컬럼 NaN — 데이터 없는 figure 는 정상.
- **shipped scene 에 잡을 물체가 없다** — p1a (`ur5e_assm_v1/mjcf/scene_with_hand.xml`) · p1b (`hand_description/robots/ur5e_p1b/mjcf/scene.xml`) 둘 다 floor plane 만. 따라서 **접촉이 필요한 경로 (contact_stop latch, grasp detection true) 는 sim 으로 검증 불가**. 물체가 있는 `hand_description/robots/demo/mjcf/scene_with_object.xml` 은 다른 hand 모델을 `<include>` 하므로 `model_path` 만 바꿔 끼우면 joint-name 매핑이 깨져 *오독 가능한* 결과가 나온다 — 도달 불가를 그대로 보고하는 것이 옳다.
- **force_pi FSM 은 물체 없이 GRASP 하면 ~1.3s 만에 Idle 로 자동 복귀**한다 (approach ramp 완료/abort). phase 가 non-Idle 인 창을 노리는 검증은 순차 CLI 로 놓친다 — 한 rclpy 프로세스 안에서 `grasp_command` 호출 직후 대상 호출을 연달아 실행할 것.
- 기본 active 컨트롤러는 launch 마다 다르다 — p1a 는 `demo_wbc_controller`, **p1b 는 `demo_joint_controller`**. p1b 에서 `DemoWbcController timing:` 로그가 안 보이는 것은 기동 실패가 아니다.
- 백그라운드 빌드/테스트와 Stop hook 의 colcon 동시 실행 금지 — foreground `tail --pid=<pid> -f /dev/null` 로 대기.
