# 전시 손 posture 운영 시트 (임시)

> 임시 문서다 — 전시 운영이 끝나면 지운다. 다른 문서는 이 파일을 가리키지 않는다. 원문은 [demo-controller-sequence.md](demo-controller-sequence.md) (현장 CLI) 와 [demo-controller-gui.md](demo-controller-gui.md#출하-posture-configrobotdemo_gui) (promote) 이고, 둘이 다르면 원문이 맞다.

- `<ws>`: colcon workspace root (`install/` · `logging_data/` 가 있는 곳)
- `<repo>`: 그 안의 repo root (`build.sh` 가 있는 곳)
- 오른손 = `p2`, 왼손 = `p2_left`. 손마다 NUC 하나.

## 이번 회차 posture 코드 (PC A 담당자가 적는다)

| 손 | posture 코드 (promote 출력의 8자리) | 적은 날 |
|---|---|---|
| 오른손 `p2` | `________` | |
| 왼손 `p2_left` | `________` | |

현장에서 대조하는 것은 **이 코드 하나**다. 계획 화면의 `commit` 줄은 PC 마다 다를 수 있으니 보지 않는다.

## ① posture 올리기 — PC A (GUI 를 쓰는 PC)

1. GUI 에서 preset · sequence 저장을 마치고 GUI 를 닫는다.
2. 브랜치를 만들고 workspace 를 source 한다.
   ```bash
   cd <repo>
   git switch main && git pull --ff-only
   git switch -c chore/update-hand-postures
   source <ws>/install/setup.bash
   ```
3. 미리 보기 (아무것도 쓰지 않는다). 고친 손만 `--robot` 으로 준다.
   ```bash
   python3 integrated_bringup/tools/demo_postures/promote.py --robot p2 --robot p2_left --check
   ```
   `check passed — nothing written` 이 나와야 한다. `refused: …` 이면 메시지대로 GUI 에서 고치고 1번부터.
4. 정본에 쓴다.
   ```bash
   python3 integrated_bringup/tools/demo_postures/promote.py --robot p2 --robot p2_left
   ```
   마지막의 `p2: posture code xxxxxxxx` · `p2_left: posture code xxxxxxxx` 를 **위 표에 적는다**.
5. (선택, 이 PC 에 손 stack 이 떠 있을 때) 미리 1분 돌려 본다. 계획 화면의 `posture code` 가 4번과 같아야 한다.
   ```bash
   ros2 run integrated_bringup demo_controller_sequence --robot p2 --mode loop --duration 1m
   ```
6. 확인하고 commit · PR · merge.
   ```bash
   git diff integrated_bringup/config/
   git add integrated_bringup/config/p2/demo_gui integrated_bringup/config/p2_left/demo_gui
   git commit -m "chore(integrated_bringup): Update the shipped hand postures"
   git push -u origin chore/update-hand-postures
   gh pr create --fill
   ```
7. 현장 PC 에 반영되면 운영자에게 손별 코드를 알린다.

보정 (손 드라이버의 `joint_direction_signs` · `joint_position_offsets_deg`) 을 바꿨으면 posture 를 새 보정에서 다시 찍고 `calibration.yaml` 을 같은 commit 에서 고친다. 다르면 promote 도 현장 CLI 도 거부한다.

## ② 매일 시작 — 현장 NUC

최신 코드가 이 NUC 에 반영돼 있다는 전제다.

1. 빌드한다 (bring-up 보다 **먼저**. posture 가 그대로인 날 해도 된다).
   ```bash
   cd <repo> && ./build.sh robot
   ```
2. 손 전원이 켜져 있는지 확인하고 tmux 를 연다.
   ```bash
   tmux new -s demo
   ```
3. 창 1 — bring-up.
   ```bash
   source <ws>/install/setup.bash
   ros2 launch integrated_bringup robot_p2.launch.py enable_logging:=false              # 오른손
   ros2 launch integrated_bringup robot_p2.launch.py side:=left enable_logging:=false   # 왼손
   ```
4. 창 2 (`Ctrl-b` 다음 `c`) — CLI. `<운영 시간>` 은 그날 시간 (예: `9h`, 1m–10h).
   ```bash
   source <ws>/install/setup.bash
   ros2 run integrated_bringup demo_controller_sequence --robot p2 --mode loop --duration <운영 시간>        # 오른손
   ros2 run integrated_bringup demo_controller_sequence --robot p2_left --mode loop --duration <운영 시간>   # 왼손
   ```
5. 계획 화면에서 두 줄을 본다.
   - `posture code xxxxxxxx — compare it with the code announced for this robot` → 위 표의 그 손 코드와 같은가
   - `calibration the running driver's = the recorded one`
6. 둘 다 맞으면 `start` 를 치고 Enter. 하나라도 다르면 `Ctrl-C` 로 끝내고 ④ 표를 본다. `RUNNING` 이 나오면 돌고 있는 것이다.

## ③ 끝내기 · 다시 붙기

- `<운영 시간>` 이 지나면 그 cycle 을 마치고 스스로 끝난다 (`exit 0: …`).
- 중간에 끝내기: CLI 창에서 `Ctrl-C` **한 번** = 이번 cycle 끝에서 멈춤, **두 번** = 바로 멈춤.
- 그다음 창 1 에서 `Ctrl-C` 로 bring-up 을 끈다.
- 창에서 빠지기: `Ctrl-b` 다음 `d` (run 은 계속된다). SSH 가 끊겼거나 다시 볼 때: `tmux attach -t demo`.
- 결과 요약: `<ws>/logging_data/sequence_runs/<시각>_<robot>.json`.

## ④ 멈췄을 때

CLI 는 `exit <번호>: <이유>` 한 줄을 남기고 끝난다. 그 이유를 보고 아래에서 고른다. 자동 재시작 · E-STOP 해제는 하지 않는다 — 사람이 한다.

| 이유에 보이는 것 | 할 일 |
|---|---|
| `does not exist` · `source it` | **A.** ② 1번 빌드부터 다시, 두 창 모두 source |
| `calibration` (보정이 다름) · controller manager 가 사라짐 · `link_status` · `joint_states` 끊김 · E-STOP | **B.** 손 전원 · CAN 케이블 · E-STOP 을 확인하고 bring-up 을 껐다 다시 띄운 뒤 CLI 를 다시 실행 (처음 step 부터 돈다) |
| 기다리다 시간 초과 (5 s 마다 빠진 항목을 출력) · 다른 손의 stack 이 보임 · 노드 중복 | **C.** 두 창에서 `echo $ROS_DOMAIN_ID` 가 같고, 다른 손 NUC 와는 다른지 확인 (⑤) |
| posture 코드가 다름 · 정본 검사 (`refused`, side tag · 관절 수 · 위치 한계 · preset 참조) · 디스크 · 그 밖의 모든 것 | **D.** 그 줄을 그대로 PC A 담당자에게 보낸다 |

계획 화면에 `next at % IGNORED` 가 보여도 시작은 된다 (모든 step 이 도착을 기다린다). 그날 안에 PC A 담당자에게 알린다.

## ⑤ 1회 설정 — NUC 마다

두 NUC 가 같은 네트워크에 있으면 `ROS_DOMAIN_ID` 가 달라야 한다 (같으면 CLI 가 다른 손의 stack 을 보고 시작하지 않는다). 아래 값은 테스트 (44–61) · sim 평가 (88) · 기본값 (0) 과 겹치지 않게 고른 것이다.

```bash
echo 'export ROS_DOMAIN_ID=21' >> ~/.bashrc   # 오른손 NUC
echo 'export ROS_DOMAIN_ID=22' >> ~/.bashrc   # 왼손 NUC
```

새 터미널 (tmux 창 포함) 을 열고 `echo $ROS_DOMAIN_ID` 로 확인한다. tmux 가 없으면 `sudo apt install tmux`.
