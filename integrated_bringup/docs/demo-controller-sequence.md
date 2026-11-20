# demo_controller_sequence — 전시용 hand sequence CLI

한 손의 저장된 posture sequence 를 GUI 없이 정해진 시간 동안 반복한다. 손마다 NUC 하나에서 bring-up 과 함께 띄운다.

```bash
ros2 run integrated_bringup demo_controller_sequence --robot p2 --mode loop --duration 9h
ros2 run integrated_bringup demo_controller_sequence --robot p2_left --mode loop --duration 9h
```

preset 과 sequence 는 GUI (`demo_controller_gui`) 의 Sequence 탭에서 만든 파일 (`<logging_data>/hand_presets_<robot>.json` · `demo_gui_sequences_<robot>.json`) 을 그대로 읽는다. CLI 는 이 파일을 고쳐 쓰지 않는다.

## 매일의 절차

1. NUC 마다 `ROS_DOMAIN_ID` 를 다르게 정하고 bring-up 터미널과 CLI 터미널에 같게 둔다 (같은 네트워크의 두 손 stack 은 이름이 같아 서로를 본다 — CLI 는 둘이 보이면 시작하지 않는다).
2. tmux 를 연다 (SSH 가 끊겨도 run 이 남는다). 한 창에서 bring-up, 다른 창에서 CLI.
3. CLI 는 stack 을 기다렸다가 (기본 60 s, `--startup-timeout`) 계획을 보여 준다: step 목록, 파일 경로, 속도 상한, 손 link 감시 여부, 로그 증가율과 디스크 여유. `start` 를 치면 시작한다 (`--yes` 는 묻지 않음).
4. 끝: duration 이 지나면 그 cycle 을 마치고 exit 0. 중간에 끝내려면 Ctrl-C 한 번 (cycle 끝에서), 두 번 (즉시).

## 시작하지 않는 경우 (exit 3)

| 상황 | CLI | 할 일 |
|---|---|---|
| stack 이 아직 없음 · domain/RMW 불일치 | 5 s 마다 빠진 항목과 DDS 환경변수를 출력하고 기다림 → 시간 초과 | bring-up 확인, 두 터미널의 `ROS_DOMAIN_ID` · `RMW_IMPLEMENTATION` 비교 |
| 기다리는 중 controller manager 가 사라짐 | 즉시 | 손 전원 · CAN · driver 확인 뒤 bring-up 재시작 (기동 timeout) |
| E-STOP · 다른 손의 stack · 노드 중복 · active 가 `hand_joint_controller` 아님 · `mode` 가 `tracking` 아님 · 다른 `joint_goal` publisher · 관절 이름 불일치 | 즉시 | 메시지대로 |
| 디스크가 로그를 감당하지 못함 | 계획 직후 | 공간 확보 또는 아래 로그 끄기 |

`RMW_IMPLEMENTATION` 이 비어 있으면 CLI 가 robot launch 와 같은 `rmw_cyclonedds_cpp` 와 `cyclone_dds.xml` 을 쓴다. `sim_p2.launch.py` 는 RMW 를 고정하지 않으므로 sim 에 붙일 때는 두 터미널에 같은 `RMW_IMPLEMENTATION` 을 export 한다 — 다르면 topic 은 보이지만 파라미터 서비스가 답하지 않아 "its node is visible but does not answer" 로 기다린다.

## 실행 중 멈추는 경우 (exit 1)

즉시 멈추고 더 보내지 않는다. 마지막으로 보낸 자세는 controller 가 유지한다.

- 손 `joint_states` 가 1 s 넘게 끊김, controller manager · E-STOP publisher 소실, E-STOP
- 손 전원 차단: driver 의 `link_status` 가 false (손 board 가 꺼져 상태가 오지 않는다). 복구는 전원 확인 뒤 **bring-up 재시작** 과 CLI 재실행 — CLI 는 처음 step 부터 다시 한다
- `mode` 변경 · active controller 변경 · 다른 publisher 출현
- 한 step 이 시간 안에 허용오차 (sequence 파일 `tolerance_deg`) 에 들지 못함
- dwell 중 자세가 목표에서 `max(3 × tolerance, tolerance + 5°)` 넘게 `--hold-abort-after` (기본 1 s) 넘게 벗어남 — 손이 자세를 잡지 못한다는 뜻 (모터 전원). 그보다 짧은 이탈 (관람객이 손을 만짐) 은 멈추지 않고 요약에 기록한다
- `trajectory_speed` 설정이 거부되거나 1 s 안에 답이 없음 (목표는 보내지 않는다)

자동 재시작 · E-STOP 해제는 하지 않는다.

## 요약 파일

`<logging_data>/sequence_runs/<YYMMDDTHHMMSS>_<robot>.json` — 시작, 60 s 마다, 끝에 원자적으로 쓴다. cycle 수, 보낸 목표 수, step 별 도달 횟수 · 평균/최대 도달 시간 · 최대 최종 오차, dwell 이탈 기록, 종료 사유와 exit code. 마지막 쓰기가 실패하면 exit 4.

## 로그와 디스크

출하 설정의 고주기 CSV 는 손 하나가 9 h 에 약 15 GB 를 쓴다 (#1126 실측). 전시에서는 NUC 마다 두 줄로 끈다:

- `config/<p2|p2_left>/_base.yaml` `enable_logging: false`
- `config/<p2|p2_left>/udp_hand_node_p2.yaml` `enable_timing_log: false`

세부는 [controller-logs.md](controller-logs.md#로그-끄기). CLI 는 시작 전 5 s 동안 세션 폴더의 증가율을 재서 `증가율 × duration × 1.5` 가 여유 공간보다 크면 시작하지 않고, 60 s 마다 다시 재서 모자라면 그 cycle 끝에서 멈춘다.

## exit code

| code | 뜻 |
|---|---|
| 0 | duration 종료 또는 Ctrl-C 한 번 (cycle 끝) |
| 1 | 실행 중 abort |
| 2 | 인자 · 파일 문제 (ROS 이전) |
| 3 | stack 준비 실패 (아무것도 보내지 않음) |
| 4 | 최종 요약을 쓰지 못함 |
| 130 | 즉시 중단 (Ctrl-C 두 번, SIGTERM, 터미널 닫힘) |

## GUI 와 다른 점

GUI 는 preset 파일을 고치고 (반대 손 파일은 옆으로 옮기고 기본값을 씀), 속도 설정이 실패해도 목표를 보내고, E-STOP 미수신 · mode 미상을 통과시킨다. CLI 는 읽기만 하고, 모르면 시작하지 않는다 (fail-closed). controller 는 목표를 거부해도 답하지 않으므로, 조용한 거부는 도달 timeout 으로만 드러난다.

## 한계

8~10 h × 3일 실기 운용은 검증되지 않았다. 소프트웨어 검증은 `sim_p2` (동작 · cycle) 와 `robot_p2.launch.py use_mock_hardware:=true` (link · watchdog · 기동 timeout) 에서 했다. 허용오차와 dwell 이탈 문턱은 실기 1 h run 에서 다시 정한다.
