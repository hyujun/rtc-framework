# HandJointController — 손 하나, 관절 목표 + 모델 feedforward (중력 · 관성)

`config_key` `hand_joint_controller`. 로봇이 **position-servo 장치 하나** (팔 없는 손) 일 때의 바인딩이다. 매 tick `pd_feedforward` 를 보낸다 — `commands` 는 마지막 `<group>/joint_goal` 로 가는 quintic reference, `feedforward` 는

```
τ_ff = clamp(k_g · g(q_meas) + k_M · M(q_meas) · q̈_ref, ±τ_max)      (rtc_controllers joint/dynamics_feedforward_law.hpp)
```

이고 Coriolis 항은 법칙에 자리만 있다 (gain 0). q̈_ref 는 quintic 의 가속도라 **진행 중 궤적에서만 0 이 아니다** — 정지 자세는 중력만 받는다. k_M (`gravity_ff.inertia_gain`, 키 생략 = 0) 은 `p2` 에서 **1 로 출하**한다 (상한 `max_abs` 0.3 N·m 는 두 항의 합을 덮는다). URDF 의 관성은 링크 것뿐이라 (rotor 관성 없음) 그 몫만 보상하고, servo 의 D 항이 측정 속도에 걸리는 sim 에서는 이동 중 오차의 대부분이 servo 의 점성 지연이라 효과가 작다 — 기본 goal 에서 ≈ 0.1 %, 짧고 빠른 goal 에서 수 % (실측은 이 항을 넣은 commit 이 갖는다). 실기 효과는 펌웨어 D 항의 형태에 달려 있다 (아래 실기 절차 (4)). 진행 중에 새 goal 이 오면 quintic 은 정지에서 다시 시작하고 (그 tick 에 reference 속도가 0 으로 끊긴다) 관성항은 새 quintic 을 따른다 — 속도가 끊기는 계단 자체는 feedforward 하지 않는다. 장치의 PD (sim `<position>` servo, 펌웨어 PD) 가 오차를, feedforward 가 무게를 맡아 **낮은 게인에서 처짐 없이** 자세를 들고 있게 하는 것이 목적이며, 출하 로봇은 `p2` (proto2 오른손, [README](../README.md#sim_p2--proto2-오른손-단독-손가락-위-중력-feedforward)) 다.

## 모델

- 군의 `urdf.tree_models.<group>` 이 손이다 (root = 핑거팁 pose 를 표현할 frame, tips = 핑거팁 frame).
- **Extended URDF** (loop closure sidecar): 중력항은 `RtClosedChainHandle` 의 축약 g_a = Gᵀg, 관성은 축약 M_a = GᵀMG 를 측정 actuated 위치에서 함께 구한 것이고, 같은 사영을 핑거팁 FK 가 빌려 쓴다 (`ClosedChainHandFk` borrowed — tick 당 사영 1 회). 사영의 독립 관절은 **전부 이 군의 관절**이어야 한다 (이름 브릿지, 순서 무관); 아니면 configure 거부.
- **Plain URDF**: tree 모델이 곧 모델이다 — serial 핸들의 g(q) (k_M ≠ 0 이면 M(q) 도) 와 FK, 장치 관절 순서를 설치. M 은 k_M ≠ 0 이고 goal 을 따르는 readable tick 에만 그 tick 의 것으로 계산한다 (정지 자세는 CRBA 를 돌리지 않는다). tree 의 관절을 장치가 정확히 한 번씩 덮지 않으면 거부.

## 모드 (`mode`)

장치의 PD 게인은 이 컨트롤러가 바꿀 수 없다 — 모드는 **두 lane 이 무엇을 싣는지**만 고른다. YAML 키이자 런타임 파라미터 (`ros2 param set … mode damping`), 언제든 (active · E-STOP 중에도) 바꿀 수 있고 E-STOP 이 모드에 우선한다. 용어는 모터 펌웨어의 "damping mode" (kp 0 · kd 만, Unitree · MAB) 에서 왔다.

| 모드 | `commands` | `feedforward` | `joint_goal` |
|---|---|---|---|
| `tracking` (기본) | 마지막 goal 로 가는 quintic reference | 중력 lane (아래 hold 정책) | 수락 |
| `damping` | **매 tick 측정 위치** (목표 = 지금 자리) | 0 — 장치의 kd 만 작용 | **거부 + `GoalRejectCount`** |
| `damping_feedforward` | 매 tick 측정 위치 | `tracking` 과 같은 lane (`gravity_ff.enabled` · gain · clamp · hold) — servo 오차 없이 lane 만 검증. 궤적이 없으니 관성항은 0 | 거부 + count |

- 비‑tracking 의 goal 거부는 E-STOP 의 **조용한 폐기** (count 없음) 와 다른 정책이다 — 운영자가 보낸 goal 이 사라졌다는 사실이 카운터에 남는다.
- `tracking` 으로 돌아오는 첫 tick 에 reference 를 측정값에서 재시드한다 (goal = 지금 자리); `tracking` 을 떠나면 진행 중인 궤적은 버려진다 (reference 가 측정값으로 점프 — 크기는 그 순간의 추종 오차).
- 비‑tracking 모드로 활성화해도 첫 readable tick 부터 출력이 valid 다 (hold 가 seed 를 겸한다 — 0.1 s 동안 invalid 면 CM 의 global E-STOP).
- 판정은 `plot_rtc_log joint_ff.csv` 로 수동 — `mode` 열이 비‑tracking 구간을 음영으로 칠하고 그 구간의 drift (q(end) − q(start) · 평균 |Δq/Δt| · 평균 |τ_ff|) 를 통계로 낸다. **`analyze_hand_gravity_ff` 의 A/B/A 게이트 run 은 `tracking` 에서만** 돌린다 — 비‑tracking 행은 q_ref = q 라 |q_ref − q| 판정을 0 으로 끌어 왜곡하므로, 판정 창에 `mode != 0` 행이 있으면 게이트가 `g.mode` 로 FAIL 한다 (`mode` 열이 없는 옛 CSV 는 그 검사가 없다). demo GUI 의 패널은 [demo-controller-gui.md](demo-controller-gui.md#손-전용-프로파일-hand_joint_controller---robot-p2).

## Hold 정책 (feedforward)

| 상황 | wire 의 `feedforward` |
|---|---|
| 첫 유효 모델항 전 (활성화 직후 walk-in 포함) | 0 |
| held · singular 사영, unreadable 장치 | **모델항 (g · M) 은 직전 유효값 유지**. q̈_ref 는 reference 자신의 것 — held 사영에서는 진행 중 궤적의 값, unreadable 에서는 reference 가 동결 (궤적 시계 정지) 이라 **0** 이고 관성항도 0. gain 은 그 tick 의 값이 적용된다 |
| 법칙이 비유한 항을 만남 | 전 관절 0 (all-or-nothing) |
| `gravity_ff.enabled: false` (YAML · 런타임 파라미터) | 0, command type 은 그대로 `pd_feedforward` |
| E-STOP | 0, `commands` = 측정값. 해제 시 측정값에서 재시드, 목표는 "지금 자리" |
| `mode: damping` | 0 (`damping_feedforward` 는 이 표의 나머지 행을 그대로 따른다) |

A/B 비교는 `gravity_ff.enabled` 로만 한다 — command type 을 position 으로 바꾸면 sim 이 그 군의 자체 중력 보상을 다시 켜 다른 plant 가 된다.

## YAML (`config/<robot>/controllers/hand_joint_controller.yaml`)

| 키 | 의미 |
|---|---|
| `mode` | `tracking` · `damping` · `damping_feedforward` (위 표). 생략 = `tracking`, 오타는 load 거부. 런타임 파라미터 |
| `trajectory_speed` · `max_traj_velocity` | rad/s. quintic 의 duration = 최대 거리 / speed, 피크 속도 (15/8 · Δq / T) 가 cap 안이 되도록 연장. 전자는 런타임 파라미터, 후자는 read-only |
| `gravity_ff.enabled` · `gravity_ff.gain` · `gravity_ff.inertia_gain` · `gravity_ff.max_abs` | 런타임 파라미터 셋 (`inertia_gain` 은 [0, 2], 생략 = 0) + **필수** 상한 (N·m, 항의 합에 건다). 적용 상한은 `min(max_abs, devices.<g>.joint_limits.max_torque)` — device rating 이 placeholder 여도 YAML 키가 wire 를 묶는다. 블록 이름은 처음 실은 항을 따랐을 뿐 lane 의 모든 항을 갖는다 |
| `closure_error_threshold` | m. 핑거팁 FK 가 사영을 신뢰하는 ‖φ‖ 상한 (토크 hold 는 핸들의 held/singular 가 정한다) |
| `command_type` | `pd_feedforward` 만 |
| `topics` | device group **하나**: `<group>/joint_goal` (`target`), `transforms` (`robot_transforms` — parent = tree root, `<tip>_actual` 핑거팁) |
| `logs` | `rtc_msgs/DeviceStateLog` (`<group>_state`) · `integrated_bringup/JointFeedforwardLog` (`joint_ff`) |

## 로그와 판정

`joint_ff.csv` 의 열은 [controller-logs.md](controller-logs.md#csv-로그) (`mode` 열 = 0 tracking · 1 damping · 2 damping_feedforward). sim 의 hold 게이트는 `ros2 run rtc_tools analyze_hand_gravity_ff <session>` — `ff_enabled` 로 on/off 구간을 나눠 마지막 3 s 의 `|q_ref − q|` 를 관절급별 문턱에 대 본다 (문턱 · 관절 목록은 그 도구의 CLI 기본값, 근거는 [testing.md](../../docs/testing.md) 의 p2 행). 그림은 `plot_rtc_log joint_ff.csv`.

## 실기 (proto2, CAN FD)

`udp_hand_native` 백엔드가 `pd_feedforward` 의 `feedforward` 를 `JointCommand.feedforward` (N·m, 같은 이름 순서) 로 싣고, 드라이버가 `feedforward_kt` 로 전류로 바꿔 보낸다 (position 명령이면 같은 길이의 0). 절차 — 출하 PD 게인을 낮춘 뒤 (1) 부호 확인: 관절 하나씩 ±소각 스텝으로 `joint_states` 와 URDF 의 부호가 같은지, 다르면 드라이버의 `joint_direction_signs` 로 뒤집는다; (2) offset: 펴진 손 = 0 이 되도록 `joint_position_offsets_deg`; (3) `gravity_ff.enabled` off/on 으로 처짐 비교, `gravity_ff.gain` 으로 미세 조정; (4) 관성항은 출하 1 이다 — 펌웨어 PID 의 D 항이 속도 오차에 걸리는지 확인하고 (측정 속도에 걸리면 이동 중 오차는 점성 지연이 지배해 관성항이 거의 효과가 없다), `gravity_ff.inertia_gain` 0 / 1 로 짧고 빠른 goal 의 추종 오차를 비교한다 — 손가락 rotor 관성은 URDF 에 없으니 효과는 모델이 덮는 몫에 그친다. 실기 run 과 그 수치는 사용자가 기록한다.
