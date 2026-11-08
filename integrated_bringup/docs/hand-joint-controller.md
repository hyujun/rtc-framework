# HandJointController — 손 하나, 관절 목표 + 모델 중력 feedforward

`config_key` `hand_joint_controller`. 로봇이 **position-servo 장치 하나** (팔 없는 손) 일 때의 바인딩이다. 매 tick `pd_feedforward` 를 보낸다 — `commands` 는 마지막 `<group>/joint_goal` 로 가는 quintic reference, `feedforward` 는

```
τ_ff = clamp(k_g · g(q_meas), ±τ_max)      (rtc_controllers joint/dynamics_feedforward_law.hpp)
```

이고 Coriolis 항은 법칙에 자리만 있다 (gain 0). 장치의 PD (sim `<position>` servo, 펌웨어 PD) 가 오차를, feedforward 가 무게를 맡아 **낮은 게인에서 처짐 없이** 자세를 들고 있게 하는 것이 목적이며, 출하 로봇은 `p2` (proto2 오른손, [README](../README.md#sim_p2--proto2-오른손-단독-손가락-위-중력-feedforward)) 다.

## 모델

- 군의 `urdf.tree_models.<group>` 이 손이다 (root = 핑거팁 pose 를 표현할 frame, tips = 핑거팁 frame).
- **Extended URDF** (loop closure sidecar): 중력항은 `RtClosedChainHandle` 의 축약 g_a = Gᵀg 를 측정 actuated 위치에서 구한 것이고, 같은 사영을 핑거팁 FK 가 빌려 쓴다 (`ClosedChainHandFk` borrowed — tick 당 사영 1 회). 사영의 독립 관절은 **전부 이 군의 관절**이어야 한다 (이름 브릿지, 순서 무관); 아니면 configure 거부.
- **Plain URDF**: tree 모델이 곧 모델이다 — serial 핸들의 g(q) 와 FK, 장치 관절 순서를 설치. tree 의 관절을 장치가 정확히 한 번씩 덮지 않으면 거부.

## Hold 정책 (feedforward)

| 상황 | wire 의 `feedforward` |
|---|---|
| 첫 유효 모델항 전 (활성화 직후 walk-in 포함) | 0 |
| held · singular 사영, unreadable 장치 | **직전 유효값 유지** (reference 도 동결, 궤적 시계 정지) |
| 법칙이 비유한 항을 만남 | 전 관절 0 (all-or-nothing) |
| `gravity_ff.enabled: false` (YAML · 런타임 파라미터) | 0, command type 은 그대로 `pd_feedforward` |
| E-STOP | 0, `commands` = 측정값. 해제 시 측정값에서 재시드, 목표는 "지금 자리" |

A/B 비교는 `gravity_ff.enabled` 로만 한다 — command type 을 position 으로 바꾸면 sim 이 그 군의 자체 중력 보상을 다시 켜 다른 plant 가 된다.

## YAML (`config/<robot>/controllers/hand_joint_controller.yaml`)

| 키 | 의미 |
|---|---|
| `trajectory_speed` · `max_traj_velocity` | rad/s. quintic 의 duration = 최대 거리 / speed, 피크 속도 (15/8 · Δq / T) 가 cap 안이 되도록 연장. 전자는 런타임 파라미터, 후자는 read-only |
| `gravity_ff.enabled` · `gravity_ff.gain` · `gravity_ff.max_abs` | 런타임 파라미터 둘 + **필수** 상한 (N·m). 적용 상한은 `min(max_abs, devices.<g>.joint_limits.max_torque)` — device rating 이 placeholder 여도 YAML 키가 wire 를 묶는다 |
| `closure_error_threshold` | m. 핑거팁 FK 가 사영을 신뢰하는 ‖φ‖ 상한 (토크 hold 는 핸들의 held/singular 가 정한다) |
| `command_type` | `pd_feedforward` 만 |
| `topics` | device group **하나**: `<group>/joint_goal` (`target`), `transforms` (`robot_transforms` — parent = tree root, `<tip>_actual` 핑거팁) |
| `logs` | `rtc_msgs/DeviceStateLog` (`<group>_state`) · `integrated_bringup/JointFeedforwardLog` (`joint_ff`) |

## 로그와 판정

`joint_ff.csv` 의 열은 [controller-logs.md](controller-logs.md#csv-로그). sim 의 hold 게이트는 `ros2 run rtc_tools analyze_hand_gravity_ff <session>` — `ff_enabled` 로 on/off 구간을 나눠 마지막 3 s 의 `|q_ref − q|` 를 관절급별 문턱에 대 본다 (문턱 · 관절 목록은 그 도구의 CLI 기본값, 근거는 [testing.md](../../docs/testing.md) 의 p2 행). 그림은 `plot_rtc_log joint_ff.csv`.

## 실기 (proto2, CAN FD)

`udp_hand_native` 백엔드가 `pd_feedforward` 의 `feedforward` 를 `JointCommand.feedforward` (N·m, 같은 이름 순서) 로 싣고, 드라이버가 `feedforward_kt` 로 전류로 바꿔 보낸다 (position 명령이면 같은 길이의 0). 절차 — 출하 PD 게인을 낮춘 뒤 (1) 부호 확인: 관절 하나씩 ±소각 스텝으로 `joint_states` 와 URDF 의 부호가 같은지, 다르면 드라이버의 `joint_direction_signs` 로 뒤집는다; (2) offset: 펴진 손 = 0 이 되도록 `joint_position_offsets_deg`; (3) `gravity_ff.enabled` off/on 으로 처짐 비교, `gravity_ff.gain` 으로 미세 조정. 실기 run 과 그 수치는 사용자가 기록한다.
