# demo_controller_sequence — 전시용 hand sequence CLI

한 손의 출하 posture sequence 를 GUI 없이 정해진 시간 동안 반복한다. 손마다 NUC 하나에서 bring-up 과 함께 띄운다.

```bash
ros2 run integrated_bringup demo_controller_sequence --robot p2 --mode loop --duration 9h
ros2 run integrated_bringup demo_controller_sequence --robot p2_left --mode loop --duration 9h
```

재생하는 것은 git 이 출하하는 정본이다: 설치된 패키지의 `config/<robot>/demo_gui/hand_presets.json` · `sequence.json`. 그 PC 의 `logging_data` 에 있는 GUI 작업 사본은 읽지 않는다. 그래서 코드를 받고 빌드한 현장 PC 는 commit 된 posture 를 그대로 재생한다. posture 를 만들고 정본으로 올리는 일은 GUI PC 에서 한다 ([demo-controller-gui.md](demo-controller-gui.md) §출하 posture). CLI 는 아무 파일도 고쳐 쓰지 않는다.

**posture 코드**: presets + sequence 의 내용 hash 8자리다. 같은 코드면 같은 posture 다 — commit · clone · 복사본과 상관없다. GUI PC 의 promote 가 손마다 이 코드를 출력하고, CLI 는 `start` 를 묻기 전에 보여 준다. 운영자가 대조하는 것은 이 코드 하나다.

## 매일의 절차

0. 코드를 받고 빌드한다. repo 루트에서 `git pull --ff-only` 뒤 `./build.sh robot` 을 실행한다. posture 가 바뀌지 않은 날 해도 해가 없다. bring-up 은 그 **뒤에** 띄운다 — 드라이버는 보정을 시작할 때 한 번 읽는다.
1. NUC 마다 `ROS_DOMAIN_ID` 를 다르게 정하고 bring-up 터미널과 CLI 터미널에 같게 둔다 (같은 네트워크의 두 손 stack 은 이름이 같아 서로를 본다 — CLI 는 둘이 보이면 시작하지 않는다).
2. tmux 를 연다 (SSH 가 끊겨도 run 이 남는다). 한 창에서 bring-up (`enable_logging:=false`, 아래 로그와 디스크), 다른 창에서 CLI.
3. CLI 는 stack 을 기다렸다가 (기본 60 s, `--startup-timeout`) 계획을 보여 준다: posture 코드, commit, 정본 파일의 실제 경로, 보정 확인, 속도 상한, 손 link 감시 여부, 로그 증가율과 디스크 여유, step 목록 (일찍 넘어가는 step 은 `next at NN %`). **posture 코드가 그 손에 공지된 코드와 같으면** `start` 를 친다 (`--yes` 는 묻지 않음). 다르면 Ctrl-C 로 끝내고 0단계를 확인한다.
   - commit 줄은 참고용이다: 정본을 바꾼 마지막 commit 과 날짜이고, 파일이 그 commit 과 다르면 `+ local changes` 가 붙는다. git 이 답하지 못하는 설치 (복사 설치 · git 없는 사본) 에서는 나오지 않는다. 파일을 옮겨 다시 commit 한 사본 (다른 저장소의 mirror) 에서는 같은 posture 라도 hash 가 다르므로 대조에 쓰지 않는다.
4. 끝: duration 이 지나면 그 cycle 을 마치고 exit 0. 중간에 끝내려면 Ctrl-C 한 번 (cycle 끝에서), 두 번 (즉시).

## 시작하지 않는 경우

ROS 를 띄우기 전에 정본을 읽고, 문제가 있으면 원인 한 줄과 함께 exit 2 로 끝난다:

| 상황 | 할 일 |
|---|---|
| 정본 파일 (`hand_presets.json` · `sequence.json` · `calibration.yaml`) 이 없음 (`… does not exist`) | 0단계의 빌드. 정본 파일은 처음 추가될 때 빌드해야 설치본에 생긴다 |
| 정본이 출하 검사에 걸림 (side tag · 관절 수 · 위치 한계 · sequence 의 preset 참조 · `next_at_pct` 가 1–99 정수가 아님 · 100 % step 이 하나도 없음) | 메시지를 그대로 posture 담당자 (GUI PC) 에게 전달 |
| workspace 를 source 하지 않음 | `source install/setup.bash` |

stack 을 보고 나서는 exit 3 으로 끝난다 (아무것도 보내지 않는다):

| 상황 | CLI | 할 일 |
|---|---|---|
| stack 이 아직 없음 · domain/RMW 불일치 | 5 s 마다 빠진 항목과 DDS 환경변수를 출력하고 기다림 → 시간 초과 | bring-up 확인, 두 터미널의 `ROS_DOMAIN_ID` · `RMW_IMPLEMENTATION` 비교 |
| 기다리는 중 controller manager 가 사라짐 | 즉시 | 손 전원 · CAN · driver 확인 뒤 bring-up 재시작 (기동 timeout) |
| E-STOP · 다른 손의 stack · 노드 중복 (손 driver 포함) · 손 driver 가 다른 손 (`hand_sides`) 을 돌림 · active 가 `hand_joint_controller` 아님 · `mode` 가 `tracking` 아님 · 다른 `joint_goal` publisher · 관절 이름 불일치 | 즉시 | 메시지대로 |
| 실행 중인 손 driver 의 보정 (`joint_direction_signs` · `joint_position_offsets_deg`) 이 정본의 `calibration.yaml` 과 다름 | 즉시 | bring-up 재시작 (0단계 전에 띄운 bring-up 은 예전 보정으로 돈다). 그래도 같으면 posture 담당자에게 전달 |
| 디스크가 로그를 감당하지 못함 | 계획 직후 | 공간 확보 또는 아래 로그 끄기 |

`RMW_IMPLEMENTATION` 이 비어 있으면 CLI 가 robot launch 와 같은 `rmw_cyclonedds_cpp` 와 `cyclone_dds.xml` 을 쓴다. `sim_p2.launch.py` 는 RMW 를 고정하지 않으므로 sim 에 붙일 때는 두 터미널에 같은 `RMW_IMPLEMENTATION` 을 export 한다 — 다르면 topic 은 보이지만 파라미터 서비스가 답하지 않아 "its node is visible but does not answer" 로 기다린다.

## 실행 중 멈추는 경우 (exit 1)

즉시 멈추고 더 보내지 않는다. 마지막으로 보낸 자세는 controller 가 유지한다.

- 손 `joint_states` 가 1 s 넘게 끊김, controller manager · E-STOP publisher 소실, E-STOP
- 손 전원 차단: driver 의 `link_status` 가 false (손 board 가 꺼져 상태가 오지 않는다). 복구는 전원 확인 뒤 **bring-up 재시작** 과 CLI 재실행 — CLI 는 처음 step 부터 다시 한다
- `mode` 변경 · active controller 변경 · 다른 publisher 출현
- 한 step 이 시간 안에 허용오차 (sequence 파일 `tolerance_deg`) 에 들지 못함 — 일찍 넘어가는 step 의 바로 다음 step 은 컨트롤러가 이동을 늘일 수 있는 만큼 (≈ 1.95 배) 더 기다린다
- dwell 중 자세가 목표에서 `max(3 × tolerance, tolerance + 5°)` 넘게 `--hold-abort-after` (기본 1 s) 넘게 벗어남 — 손이 자세를 잡지 못한다는 뜻 (모터 전원). 그보다 짧은 이탈 (관람객이 손을 만짐) 은 멈추지 않고 요약에 기록한다
- `trajectory_speed` 설정이 거부되거나 1 s 안에 답이 없음 (목표는 보내지 않는다)

자동 재시작 · E-STOP 해제는 하지 않는다.

## 요약 파일

`<logging_data>/sequence_runs/<YYMMDDTHHMMSS>_<robot>.json` — 시작, 60 s 마다, 끝에 원자적으로 쓴다. 재생한 posture 코드 · commit · 정본 파일의 실제 경로, cycle 수, 보낸 목표 수, step 별 도달 횟수 · 평균/최대 도달 시간 · 최대 최종 오차 (일찍 넘어간 step 은 `switched` 횟수 · 전환 순간의 최대 오차 `switch_error_deg_max` · 다음 목표가 예정보다 늦게 나간 시간 `lag_s_mean` / `lag_s_max` — 도달로 세지 않는다), dwell 이탈 기록, 종료 사유와 exit code. 마지막 쓰기가 실패하면 exit 4.

## 로그와 디스크

출하 설정의 고주기 CSV 는 손 하나가 9 h 에 약 15 GB 를 쓴다 (#1126 실측). 전시에서는 bring-up 에 인자 하나를 붙여 끈다:

```bash
ros2 launch integrated_bringup robot_p2.launch.py enable_logging:=false              # 오른손
ros2 launch integrated_bringup robot_p2.launch.py side:=left enable_logging:=false   # 왼손
```

controller 의 세션 파일과 손 driver 의 `timing/hand_timing_log.csv` 가 함께 꺼진다. git 이 관리하는 YAML 은 고치지 않는다 — 고쳐 두면 그 파일이 upstream 에서 바뀌는 날 `git pull` 이 멈춘다. 세부는 [controller-logs.md](controller-logs.md#로그-끄기). CLI 는 시작 전 5 s 동안 세션 폴더의 증가율을 재서 `증가율 × duration × 1.5` 가 여유 공간보다 크면 시작하지 않고, 60 s 마다 다시 재서 모자라면 그 cycle 끝에서 멈춘다.

## exit code

| code | 뜻 |
|---|---|
| 0 | duration 종료 또는 Ctrl-C 한 번 (cycle 끝) |
| 1 | 실행 중 abort |
| 2 | 인자 · 파일 문제 (ROS 이전) |
| 3 | stack 준비 실패 (아무것도 보내지 않음) |
| 4 | 최종 요약을 쓰지 못함 |
| 130 | 즉시 중단 (Ctrl-C 두 번, SIGTERM, 터미널 닫힘) |

## 일찍 넘어가는 step (wave)

sequence 파일의 step 에 `next_at_pct` (1–99) 가 있으면 그 step 은 예상 이동 시간의 그 비율에서 다음 step 으로 넘어간다 (GUI 의 `Next %`, 100 은 키를 생략). 그 step 에는 도착 확인 · 도달 timeout · dwell 감시가 없다 — 손이 따라오는지는 100 % step 이 확인하므로 sequence 에 하나 이상 있어야 하고, 관절별 감시는 100 % step 의 자세들이 그 관절을 움직이는 만큼만 된다. 끝낼 때 (duration · Ctrl-C 한 번) 마지막 step 은 끝까지 가서 도착을 확인한다 — 요청이 마지막 step 이 넘어간 뒤에 오면 다음 cycle 끝에서 멈춘다. `lag_s_*` 는 이 CLI 의 수치다 (GUI 는 속도 set 에 왕복이 하나 더 있다).

## GUI 와 다른 점

GUI 는 logging_data 의 작업 사본에서 돌며 그것을 고치고, 속도 설정이 실패해도 목표를 보내고, E-STOP 미수신 · mode 미상을 통과시킨다. CLI 는 정본을 읽기만 하고, 모르면 시작하지 않는다 (fail-closed). sim (`sim_p2`) 에는 손 driver 가 없으므로 보정은 확인하지 않는다 (계획 화면에 그렇게 나온다). controller 는 목표를 거부해도 답하지 않으므로, 조용한 거부는 도달 timeout 으로만 드러난다.

## 한계

8~10 h × 3일 실기 운용은 검증되지 않았다. 소프트웨어 검증은 `sim_p2` (동작 · cycle) 와 `robot_p2.launch.py use_mock_hardware:=true` (link · watchdog · 기동 timeout) 에서 했다. 허용오차와 dwell 이탈 문턱은 실기 1 h run 에서 다시 정한다.
