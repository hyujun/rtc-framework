# dynamic_catching — 식별자 색인

코드 주석 · YAML · 테스트가 인용하는 결정 ID · 게이트 ID · 단계 이름을 푸는 문서다. 코드를 읽다 `MD-45`, `D-16`, `G3-I`, `S6-B` 를 만나면 여기서 뜻과 자리를 찾는다.

- **계획이 아니다.** 상태 · 순서 · 남은 일은 [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) 와 이슈가 갖는다. 이 문서에는 "지금 무엇이 성립하나" 와 "어디에 있나" 만 적는다 — 경위 · 날짜 · 측정값은 적지 않는다 (이슈와 git 이력에 있다)
- 수학과 구조의 서술은 [ref/](ref/) 가 갖는다. 이 문서의 한 줄과 `ref/` 가 다르면 `ref/` 와 코드가 맞다
- **새 ID 를 만들지 않는다.** 새 결정의 이유는 그 코드의 주석과 `ref/` 의 해당 절에 적는다 (AP-DOC-2). 이 표는 이미 코드에 박힌 ID 를 풀기 위해서만 있다
- ID 가 가리키는 내용이 바뀌면 그 행을 지금의 것으로 고친다. 코드에서 그 ID 의 인용이 모두 사라지면 행을 지운다

**경로 약어.** `IB` = `integrated_bringup`, `RC` = `rtc_controllers`, `RCI` = `RC/include/rtc_controllers/catching`, `RCS` = `RC/src`, `ctrl` = `IB/src/controllers/catching/controller.cpp`, `life` = `IB/src/controllers/catching/lifecycle.cpp`, `hdr` = `IB/include/integrated_bringup/controllers/demo_catching_controller.hpp`, `par` = `RCI/catching_params.hpp`, `tools` = `rtc_tools/rtc_tools/analysis`, `cfg` = `IB/config/<robot>/controllers/catching/segment_mpc.yaml`. `L3 §4.2` 는 `ref/L3_planner.md` 의 절, `MASTER` 는 `ref/CATCHING_MASTER.md`, `f` 는 `ref/mpc_multiframe_clik_formulation.md` 다. "어디에" 칸은 `코드 / 문서` 순이다.

## 1. 옛 인용 → 지금의 자리

지워진 계획 문서의 절 번호를 지금의 자리로 푸는 표다. 코드 주석의 포구 인용은 이 표대로 옮겼다 ([#709](https://github.com/hyujun/rtc-framework/issues/709)) — 표는 옛 커밋 · 이슈 · repo 밖의 산출물에 남은 인용을 읽기 위해 둔다.

| 코드가 적은 것 | 지금의 자리 |
|---|---|
| `plan §1` (결정 로그) | 이 문서의 §5.1 (`D-n`). 근거와 경위의 원문은 [#537 의 기록 코멘트](https://github.com/hyujun/rtc-framework/issues/537#issuecomment-5974542176) |
| `plan §1a` (Epic 의 성공 기준) | L8 §9 의 G8-D · G8-D2, 결과는 [#537](https://github.com/hyujun/rtc-framework/issues/537) |
| `plan §2` (workspace 분석의 결론) | 각 `ref/L<n>` §2 (코드 확인 항목) |
| `plan §3` (시간 규약) | L0 §4.5 |
| `plan §3.1` (`header.stamp` 계약) | L1 §4.1, [invariants.md](../../agent_docs/invariants.md) §Clock 시간축 규칙 |
| `plan §4` · `§4.2` · `§4.3` (단계 계획 · 상태) | 아래 §3 의 단계 표, 기록은 [#537](https://github.com/hyujun/rtc-framework/issues/537) |
| `plan §5` · `§5.1` (sim 시계 위상 오차, D-3) | L8 §4.5 |
| `plan §6` · `plan §7.2` (계획기 스레드 · 스케줄러) | L3 §5.3 |
| `plan §7.1` · `§7.3` · `§7.4` (확정 기록) | 이 문서의 해당 ID. 원문은 [#537 의 기록 코멘트](https://github.com/hyujun/rtc-framework/issues/537#issuecomment-5974542377) |
| `plan §8` (v1 계획 — 계획기의 NLP 전환 대비) | 아래 `A-4`, L3 §4.1. **config · 테스트 주석의 `plan §8 "E1-F06"` 과 `MPC plan §8` 은 MPC 계획의 측정 절이다** — 아래 줄 |
| `plan §9` (가속 box 의 도출, D-16) | L3 §4.3 |
| `plan §10` (catch frame, D-17) | L5 §11 |
| `plan §11` (catchability, D-18) | L3 §4.2 |
| `plan §12` (알려진 위험) | MASTER §10 |
| `plan §13` (GUI · plot, D-19) | L8 §11 |
| `plan §4.4 S<n>` (단계의 작업과 게이트) | 아래 §3 의 단계 표와 그 층의 `ref/L<n>` §9. 단계별 결과의 원문은 [#537 의 기록 코멘트](https://github.com/hyujun/rtc-framework/issues/537#issuecomment-5974567014) |
| `MPC plan §4` (`MD-n`) | 아래 §4 |
| `MPC plan §8 "E1-F10"`, `plan §8 "E1-F06"` 같은 측정 절 | 아래 §3 의 feature 표가 가리키는 이슈의 "측정 · 검증 기록" 코멘트 |
| `S8 sub-plan §6.4` · `§6.5` (`catching_trials.py`) | 지워진 private 계획의 절이다 — 그 함수의 docstring 이 내용을 적는다 |
| `formulation §N`, `L<n> §N`, 옛 경로 `docs/dynamic_catching/L<n>_*.md` · `CATCHING_MASTER.md` · `mpc_multiframe_clik_formulation.md` | `ref/` 의 같은 이름 문서, 같은 절 번호 |
| `IMPLEMENTATION_PLAN.md` · `WORKSPACE_ANALYSIS.md` (경로) | 지워졌다 — 이 문서와 `ref/` |

## 2. 게이트 · TBD

| 모양 | 뜻 | 자리 |
|---|---|---|
| `G<n>-<숫자>` (예: `G3-3`) | 층 n 의 코드 확인 항목 | `ref/L<n>` §2 |
| `G<n>-<글자>` (예: `G3-I`, `G7-F`, `G8-D2`) | 층 n 의 합격 게이트 — 판정 기준 | `ref/L<n>` §9 |
| `G-1` | 두 planner 의 비열등 검정 (`closed_form` 대 `mpc`) | f §6.5, 수치와 결과는 [#632](https://github.com/hyujun/rtc-framework/issues/632) |
| `TBD-<영역>-<번호>` (예: `TBD-HAND-04`, `TBD-VIS-07`, `TBD-NET-01`) | 아직 정해지지 않은 값. 실기 단계가 정하는 것이 대부분이다 | 그 영역의 `ref/L<n>` (HAND → L6, VIS → L1 · L2, NET → MASTER §5 · L7, ARM → L5, IMP → L7 §4.7) |
| `R-1`, `R-2` | 계획기 단계의 확인 항목 — R-1: 계획기가 thread layout 의 `mpc` role 을 공유하고 컨트롤러 전환 뒤 하나만 돈다 (`IB/test/test_catching_mpc_role_switch.cpp`). R-2: 한 주기의 탐색 예산과 IK 후보 사전 필터 (`planner.search.grid.budget_s` · `planner.search.grid.max_ik`) | L3 §5.3, L3 §4.1 |

## 3. 단계와 feature 의 이름

코드 주석의 `S6-B`, `S3.5b`, `E1-F07` 은 그 코드가 들어온 작업 단위의 이름이다. 뜻은 "언제 들어왔나" 이고 지금의 동작은 그 층의 `ref/` 가 적는다.

| 단계 | 무엇을 만들었나 | 지금의 서술 |
|---|---|---|
| S0 | 결정 · 문서 · 계약 (코드 없음) | MASTER |
| S1 (S1.1 – S1.9) | 순수 수치 코어 (`RCI/`) — 시간 타입, 샘플러, soft-catch, 도달시간, 전이표, 포구 자세 IK | L0 · L2 · L3 · L4 · L7 |
| S2 | 기존 `rtc_*` 의 일반화 — `rtc_math` se3, CLIK 확장, `extra_frames` | L4 §4.5, L5 |
| S3a · S3.4 | sim 기반 (`rtc_mujoco_sim` 의 공 · 발사 srv · 진단 lane), vision lane 측정 | L8 §4 |
| S3.5a · S3.5b · S3.6 | 오프라인 지도 (kinematic · gate) 와 vision 사양 | L3 §4.2, L1 §4.1 |
| S4a · S4.4 | 손 타이밍 측정과 go/no-go | L6 §4 |
| S5 (S5.1 – S5.5) | 포구 컨트롤러 골격 · 입력 · 추종 | L1, L5, L8 |
| S6 (S6-A – S6-C2) | 계획기 스레드와 탐색 | L3 |
| S7 | 손 시퀀서 · 슈퍼바이저 | L6, L7 |
| S8 (S8-A – S8-I) | sim 통합 평가 | L8 §9 |
| S9 (S9a · S9b) | E-STOP · fault 정책 | L7 §4.1 |
| pre-S10 (R1 – R7) | 실기 단계 전 정리 | L7, `rtc_controller_manager/README.md` |
| S10 | 실기 단계 | [#613](https://github.com/hyujun/rtc-framework/issues/613) |

| feature | 이슈 | 내용 |
|---|---|---|
| E0-F01 – F04 | [#624](https://github.com/hyujun/rtc-framework/issues/624) · [#625](https://github.com/hyujun/rtc-framework/issues/625) · [#626](https://github.com/hyujun/rtc-framework/issues/626) · [#647](https://github.com/hyujun/rtc-framework/issues/647) | 빌드 경로 · closed-form 기준선 측정 · formulation · 예측 격자 sweep |
| E1-F01 – F04 | [#627](https://github.com/hyujun/rtc-framework/issues/627) · [#628](https://github.com/hyujun/rtc-framework/issues/628) · [#629](https://github.com/hyujun/rtc-framework/issues/629) · [#630](https://github.com/hyujun/rtc-framework/issues/630) | MPC 코어 · 구간 payload 와 RT 샘플러 · 계획기 통합 · DECEL 의 구간 추종 |
| E1-F05 · F06 | [#631](https://github.com/hyujun/rtc-framework/issues/631) · [#632](https://github.com/hyujun/rtc-framework/issues/632) | 로그 · GUI · plot, 게이트 G-1 |
| E1-F07 – F09 | [#660](https://github.com/hyujun/rtc-framework/issues/660) · [#661](https://github.com/hyujun/rtc-framework/issues/661) · [#662](https://github.com/hyujun/rtc-framework/issues/662) | APPROACH–정지 코어 · 계획기 · RT 추종 |
| E1-F10 · F11 | [#663](https://github.com/hyujun/rtc-framework/issues/663) · [#698](https://github.com/hyujun/rtc-framework/issues/698) | `mpc` 튜닝, config 의 기능별 분리 |
| E2-F01 – F03 | [#633](https://github.com/hyujun/rtc-framework/issues/633) · [#634](https://github.com/hyujun/rtc-framework/issues/634) · [#635](https://github.com/hyujun/rtc-framework/issues/635) | G1 자산 · config · launch · joint 구동 |
| E1-F12 | [#738](https://github.com/hyujun/rtc-framework/issues/738) | 포구 탐색 · 구간 계획기의 추상 interface (`CatchSearch` · `SegmentPlanner`) |
| E1-F13 | [#739](https://github.com/hyujun/rtc-framework/issues/739) | mpc_docking 수치 코어 `MpcDockingSegmentCore` — 상대상태 · corridor · 확률 제약 · 토크 판정의 NLP (ProxQP 위 SQP). 구현한 식은 [ref/ball_catching_inverse_dynamics_mpc.md](ref/ball_catching_inverse_dynamics_mpc.md) §17 |
| E1-F14 – F21, E2-F04 이후, E3 | — | 아직 구현하지 않은 feature — [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) |

## 4. `MD-n` — MPC · dual-arm 확장의 결정

번호가 빠진 것은 측정 절차나 작업 순서의 결정이라 색인에 없다 (MD-6 · 8 · 15 · 19 · 41 · 50 · 71 · 82 · 87). 결정의 근거와 경위 (빠진 번호 포함) 는 옮기기 전 결정 로그의 원문에 있다 — [#705 의 기록 코멘트](https://github.com/hyujun/rtc-framework/issues/705#issuecomment-5974506385). "(미구현 feature 의 결정)" 은 아직 구현하지 않은 feature 를 구속한다.

| ID | 무엇이 성립하나 | 어디에 |
|---|---|---|
| MD-1 | MPC 수치 코어는 `rtc_mpc` 가 아니라 `rtc_controllers/catching` 에 있고 `QPSolverWrapper` 를 쓴다. | `RCI/mpc_segment_core.hpp` · `mpc_segment_core.cpp` / f §0 |
| MD-2 | 다중 frame CLIK 컨트롤러 `demo_dualarm_controller` 는 새 config key 로 추가될 예정이고 기존 `demo_task_controller` 는 건드리지 않는다 (미구현 feature 의 결정). | — / f §2 (구현 범위 표) |
| MD-3 | DECEL 은 soft-catch DS 를 건너뛰고 MPC 구간의 pose · twist · 관절 기준을 CLIK 에 넣는다 (`mpc` 에서는 APPROACH 부터, MD-65 가 넓힘). | `IB/src/controllers/catching/controller.cpp` `RunSegmentTick` / L5 (`RunSegmentTick`), L7 §4.3a |
| MD-4 | G1 자산은 `ur5e_p1b` 와 같은 방식으로 `hand_description` 을 런타임에 참조한다. | `IB/config/g1_p1b/` / f §8.3 |
| MD-5 | G1 은 sim 전용이다 (`DeviceBackend` 가 없다). | `IB/launch/sim_g1_p1b.launch.py` / — |
| MD-7 | MPC 의 가속 제약은 활성 관절의 토크 행이고 유도 가속 box 는 쓰지 않는다 (MPC ⊂ CLIK 를 같은 물리량으로 보장). | `RCI/mpc_segment_core_torque.hpp`, `IB/test/test_catching_planner_lane.cpp` / f §1.2, §1.3 |
| MD-9 | 계획기는 관절 노드만 게시하고 손의 pose · twist 는 RT 가 관절 기준에서 FK 로 만든다. 보간기는 없다. | `IB/src/controllers/catching/controller.cpp` (CLIK target 절), `RCI/jerk_segment.hpp`, `node_follower.hpp` / f §1.5, L7 §4.3a |
| MD-10 | 재계획 주기 · MPC 노드 간격 · 예측점 간격은 서로 다른 양이고, 격자는 포구 시각 $t_c$ 에 고정된다. | `RCI/trajectory.hpp` (격자 산술), `mpc_segment_planner.hpp`, `planner_io.hpp` / f §1.1, §1.6 |
| MD-11 | MPC 는 제동을 보장하지 않는다 — 풀이 실패 · 예산 초과면 게시하지 않고, RT 는 따를 구간이 없으면 `ABORT_SAFE` 다 (closed-form fallback 은 MD-44 가 없앰). | `RCI/mpc_segment_core.hpp` (헤더 주석) / f §1.5, L7 §4.3a |
| MD-12 | 각운동량 항은 유지하며 floating base · 왼팔 운동 생성을 위한 것이다 (미구현 feature 의 결정). | — / f §1.4 |
| MD-13 | 토크는 1차 선형화에 slack (`cost.rho_tau`) 을 두고 위치 · 속도 한계 · 종단 등식 · trust region 은 hard 다. 자기충돌 · 공–왼팔 행과 손 잠금 축소 모델은 dual-arm 몫이다 (미구현 feature 의 결정). | `RCI/mpc_segment_core_torque.hpp`, `cfg` `cost.rho_tau` · `linearization.delta_tr` / f §1.2, §1.3 |
| MD-14 | CLIK 의 제동 거리 속도 한계 · 충돌 damper · 오차 되먹임 상한은 기본 꺼짐의 opt-in 이다 (미구현 feature 의 결정). | — / f §2.2 |
| MD-16 | QP solver 는 ProxQP dense 다 (`QPSolverWrapper`). | `RCI/mpc_segment_core.hpp` / f §1.1 |
| MD-17 | rtc-framework · `hand_description` · ball_perception 은 서로의 이름을 빌드 스크립트와 manifest 에 넣지 않는다. | `IB/config/<robot>/controllers/demo_catching_controller.yaml` 의 추정기 profile 주석 / — |
| MD-18 | sim 추정기 profile (`ball_perception.sim_profile`) 은 ball_perception 저장소가 소유하고 rtc-framework 에 로봇별 사본이 없다 (MD-20 이 실행). 컨트롤러 YAML 의 예측 격자 키와의 정합은 자동 검사가 없다. | `IB/config/ur5e_p1b/controllers/demo_catching_controller.yaml` · `iiwa7_leap/…` (profile 주석) / f §0 |
| MD-20 | MD-18 의 실행 — profile 은 ball_perception 의 `sim_profile.catching.json` 이고 `ball_sim_ws` 는 pull 만 한다. | MD-18 과 같다 / — |
| MD-21 | segment MPC 의 지평 $N_s\Delta_s$ 는 정지 시간 그 자체다 (비용에 시간 항이 없어 해가 지평 전체를 쓴다). | `cfg` `horizon`, `IB/src/controllers/catching/lifecycle.cpp` (`horizon.n_nodes` 파라미터) / f §1.6 |
| MD-22 | 코어 경로 (선형화 · FK · condensing · 결과 기록) 의 Solve 할당은 0 이고 ProxQP 가 `Solve` 안에서 하는 C 할당은 알려진 한계 (#654) 다. | `RCI/mpc_segment_core.hpp` (할당 주석), `RC/test/test_catching_mpc_segment_core_approach.cpp` / — |
| MD-23 | 계획기의 구간 경로도 ProxQP 밖의 할당 0 을 단언하고 ProxQP 할당 횟수는 기록만 한다. | `RCI/mpc_segment_planner.hpp` (할당 주석), `RC/test/test_catching_approach_planner.cpp` / — |
| MD-24 | 출하 지평은 정지 시간 0.35 s 로 맞춘다 (격자는 MD-54 가 바꿈 — 정지 7 노드 × 0.05 s, 블록 {1,1,2,3}). 계획기의 $\dot q_{\max}$ 는 팔 device 의 `max_velocity` 다. | `cfg` `horizon`, `RCI/mpc_segment_planner.hpp` / f §1.6 |
| MD-25 | armature 는 제어에 쓰지 않는다 — 계획기는 코어에 0 벡터를 넘기고 코어의 `MpcSegmentCoreLimits::armature` 입력은 남는다. | `RCS/catching/mpc_segment_planner.cpp` (armature 0), `RCI/mpc_segment_planner.hpp` / f §1.1 |
| MD-26 | 기준 없는 첫 풀이의 시점 규칙 (pre-solve + 본 solve) 은 MD-56 · MD-57 · MD-58 의 "plan 과 첫 구간을 한 wake 에 푼다" 로 바뀌었다 (MD-56 이 바꿈). | `RCS/catching/planner_cycle.cpp` / L3 §5.3 |
| MD-27 | 구간 payload 는 `PlanSnapshot` 이 아니라 형제 POD `SegmentSnapshot` 과 자기 SeqLock 으로 보낸다 (관절 용량 8, 노드 용량 `kMaxSegmentNodes`). | `RCI/trajectory.hpp` (`kMaxSegmentNodes`), `IB/include/integrated_bringup/controllers/demo_catching_controller.hpp` / L0 (`kMaxSegmentNodes`), L3 §5.2 |
| MD-28 | 계획기의 초기 상태 $x_0$ 는 RT 가 보고한 구간에서 평가한다 (MD-58 이 바꿈 — 명령 외삽과 $\ddot q$ 차분 경로는 MD-70 이 지움). | `RCI/jerk_segment.hpp` (구간 평가) / f §1.5, L7 §4.3a |
| MD-29 | 구간 재계획만 한 wake 의 `CycleOutcome` 은 idle 로 남고 구간 결과는 기록의 별도 필드로 낸다. | `RCI/planner_cycle.hpp`, `RCS/catching/planner_cycle.cpp` / L3 §5.3 |
| MD-30 | `NodeFollower` 의 검증은 FK 일관성 ($T(q_{ref})=T^d$, $J\dot q_{ref}=V^{ff}$) 이고 CLIK 속도 잔차는 정보용이다. | `RCI/node_follower.hpp` / L7 §4.3a |
| MD-31 | 포구 뒤 재계획은 격자점 $k\le k_{\max}$ 에서만 하고 정지 끝은 $t_c+N_s\Delta_s$ 에 고정된다 (출하 `replan.k_max` 는 MD-61). | `cfg` `replan.k_max`, `IB/src/controllers/catching/lifecycle.cpp` (`k_max` 파라미터), `RCS/catching/planner_cycle.cpp` / f §1.6 |
| MD-32 | RT 의 구간 채택 규칙과 효력 시각 전환 규칙은 순수 함수이고 `PlannerRtState` 가 따르는 plan 의 $t_c$ 를 싣는다. | `RCI/planner_io.hpp` (RT-side admission 절) / L7 §4.3a |
| MD-33 | 게시 조건은 풀이 성공 · 예산 안 · 효력 시각 전 · `slack_max` 와 `slack_terminal_max` 가 유한하고 임계 이하일 때다. $\eta'_\tau+$`slack_max` $\le$ `joint_cmd.eta_tau` 를 configure 에서 검사한다. | `IB/src/controllers/catching/lifecycle.cpp` (configure 검사), `cfg` `publish.*` / f §1.5 |
| MD-34 | RT 의 segment lane 은 `planner.segment.mode: mpc` 에서만 돌고 전제 (샘플러, `joint_cmd.K_n`>0, $\eta_v<1$ — 여기서 $\eta_v$ 는 `planner.segment.mpc.eta_v` 다, 팔 속도 box, CLIK 위치 box, MPC 구간 계획기) 가 빠지면 park 한다 (`catch_box` 는 MD-73 이 뺌, `planner.segment.mpc.enabled` 는 MD-91 이 없앰). | `IB/include/integrated_bringup/controllers/demo_catching_controller.hpp` (mpc 전제 판정), `IB/src/controllers/catching/lifecycle.cpp` / L7 §4.3a |
| MD-35 | RT 가 새로 소유하는 구간 상태는 재무장과 E-STOP 의 reset 이 모두 되돌리고, 구간은 쓸 때마다 따르는 plan 의 id · $t_c$ 와 대조한다. 어긋나면 `ABORT_SAFE` 다. | `IB/include/integrated_bringup/logging/catching_diag_log_pod.hpp` (`kPlanMismatch`), `IB/src/controllers/catching/controller.cpp`, `IB/test/test_catching_reset_probe.cpp` / L7 §4.8 |
| MD-36 | 자세 과제의 속도 feedforward 는 자세 목표를 $q_{ref}+\dot q_{ref}/K_n$ 으로 넘겨 얻는다 (`rtc_tsid` 불변). | `IB/src/controllers/catching/controller.cpp` (CLIK target), `lifecycle.cpp` (`K_n` > 0 검사) / f §2.2, L5 (`joint_cmd.K_n`) |
| MD-37 | 구간은 COMMITTED · CLOSING · DECEL 에서 매 tick 판정하고 대기 슬롯이 비었을 때만 채택한다 (나이 상한 50 ms). 차 있으면 box 에 남긴다 (`kDeferred`). | `IB/include/integrated_bringup/controllers/demo_catching_controller.hpp` (나이 상한 · 판정), `catching_diag_log_pod.hpp` (`kDeferred`) / L7 §4.3a |
| MD-38 | 대기 구간은 node 0 시각이 오면 따르는 구간이 되고, DECEL 진입에 구간이 없으면 `ABORT_SAFE` 다. 도중 재계획 구간이 게이트를 못 지나면 버리고 따르던 구간을 계속 따른다. HOLD 는 새 구간을 받지 않는다. | `IB/src/controllers/catching/controller.cpp` (switch 절, HOLD 절) / L7 §4.1, §4.3a |
| MD-39 | 구간 전환 게이트는 관절마다 $\vert\Delta\dot q_i\vert+K_p\vert\Delta q_i\vert\le\rho_{\max}(1-\eta_v)\dot q_{\max,i}$ 이고 $\rho_{\max}$ 는 `planner.segment.mpc.switch_margin` 이다. | `IB/src/controllers/catching/controller.cpp` (switch 절), `cfg` `planner.segment.mpc.switch_margin`, `catching_diag_log_pod.hpp` (`kGateRefused`) / f §2.3, L7 §4.3a |
| MD-40 | RT 는 구간을 now_lead $+h$ ($h$ = 제어 주기) 에서 샘플한다 (계획기 쪽 보고 지연 보정은 MD-58 · MD-70 이 바꿈). | `IB/src/controllers/catching/controller.cpp` (sample 절), `RCI/planner_io.hpp` / L7 §4.3a |
| MD-42 | MPC 구간 계획기의 관절 위치 box 는 URDF 한계와 CLIK 위치 box (device 한계 − `limit_margin`) 의 교집합이고 `m_q` 는 그 안쪽이다. | `IB/src/controllers/catching/lifecycle.cpp` (MD-42 절), `demo_catching_controller.hpp` / f §3 |
| MD-43 | 폐기 — 지금은 RT 가 구간의 `catch_box` 를 검사하지 않는다 (MD-73). 분석 도구의 `segment_workspace_refused` 만 남는다. | `rtc_tools/rtc_tools/analysis/catching_trials.py` / — |
| MD-44 | 팔이 따르는 것 (segment mode) 은 섞지 않는다 — `planner.segment.mode` 가 configure 에서 `closed_form` 또는 `mpc` 를 정하고 `mpc` 는 진입 tick 에 따를 구간이 없으면 `kParamsTbd` 로 `ABORT_SAFE` 다. | `IB/include/integrated_bringup/controllers/demo_catching_controller.hpp` (`segment_mode_` 멤버), `catching_diag_log_pod.hpp` (`kNoSegment`) / L7 §4.3a · §4.2 |
| MD-45 | `mode: mpc` 는 APPROACH 부터 정지까지 팔 기준을 MPC 가 만든다 — 입력은 공 궤적과 탐색이 고른 plan ($p_c$ · $t_c$ · $a_d$), 출력은 관절 노드이고 v1 DS 법칙은 돌지 않는다. | `IB/include/integrated_bringup/controllers/demo_catching_controller.hpp` (Segment MPC follower 절), `IB/test/test_catching_supervisor_scenarios.cpp` / f §1.3 · §1.6, L7 §4.3a, L3 §4.1 |
| MD-46 | 설계는 formulation §1.3 하나이고 단일 팔은 dual-arm · waist 전용 항만 뺀 구성이며 g1_p1b 는 그 항을 더한다 (g1 쪽은 미구현 feature 의 결정). 포구 후보 ($t_c$) 는 MPC 가 아니라 v1 탐색이 고른다. | `cfg` · `IB/config/<robot>/controllers/catching/search_grid.yaml` · `planner_closed_form.yaml` (헤더 주석) / f §0 · §1.6, L3 §4.1 |
| MD-47 | E3 는 같은 MPC 에 dual arm · waist 항을 더하는 epic 이다. G-1 은 단일 팔 `mpc` 가 `closed_form` 대비 비열등한지만 판정한다 (미구현 feature 의 결정). | — / f §6.5 |
| MD-48 | key 와 그 key 를 따르는 식별자는 `segment` 로 불린다 (`planner.segment.*` · `CatchingSegmentMode`). key 를 따르는 식별자 · 구간 lane 의 CSV 열 (`segment_*`) · 로그 문구 · 표시 문구 (GUI · plot) 도 `segment` 다. `decel` 은 DECEL 상태 · closed_form 감속 법칙 (`supervisor.decel.a_dec` · `DecelTarget`) · DECEL 상태의 정지를 재는 `catching_decel` 도구에만 남는다. `mpc` 에서 범위는 APPROACH–정지다. | `RCI/mpc_segment_core.hpp` (헤더 주석) / L7 §4.3a, f §0 |
| MD-49 | 코어는 비용 · 제약을 항 단위로 조립하고 관절군별 move blocking 행렬 $E$ 를 넣을 자리만 둔다. 다관절군 일반화는 E3 몫이다 (미구현 feature 의 결정). | `RCI/mpc_segment_core.hpp` (`E` 주석) / f §1.1 |
| MD-51 | 격자는 코어의 파라미터이고 노드별 간격을 받는다 (확정 값은 MD-54). | `RC/test/test_catching_mpc_segment_core_approach.cpp` / f §1.6 |
| MD-52 | 상대속도 slack $s_v$ 는 구현되어 있고 기본 꺼짐 (`catch.rho_v` 0) 이며 포구 위치 가중 $W_p$ 는 코어 입력이다. 경로 이탈 항은 별도로 두지 않는다. | `cfg` `catch.rho_v` · `v_rel_allow`, `RCI/mpc_segment_core_catch.hpp` / f §1.3, §1.6 |
| MD-53 | 코어는 $\hat v_b$ 와 $\gamma_{ref}$ 를 받아 상대속도 목표를 $\gamma_{ref}\hat v_b$ 로 두고 slack 행은 늘 $\hat v_b$ 기준이다. | `RCI/mpc_segment_core.hpp` (`gamma_ref`), `IB/src/controllers/catching/lifecycle.cpp` (`catch.gamma_ref`) / f §1.3, §1.6 |
| MD-54 | 출하 격자는 포구 전 $\Delta_{pre}$ 0.1 s (최대 6 노드) + 정지 $\Delta_s$ 0.05 s × 7 (블록 {1,1,2,3}) 이다. | `cfg` `horizon` · `approach` / f §1.6 |
| MD-55 | 포구 전 격자는 `planner.segment.mpc.approach.n_pre_max` 가 켠다 (코드 기본 0, 출하 6). 출하의 `enabled`/`closed_form` 조합은 MD-89 가 바꿈. | `cfg` `approach.n_pre_max`, `IB/test/test_demo_catching_controller.cpp` / f §1.6, L3 §6 |
| MD-56 | 첫 구간은 탐색과 같은 wake 에서 풀어 plan 과 쌍으로 (구간을 먼저, 같은 `publish_ns`) 게시한다. 구간이 게시 조건을 못 넘으면 plan 도 게시하지 않고 예산 키는 `budget.first_s` · `budget.replan_s` 다. | `IB/src/controllers/catching/controller.cpp`, `lifecycle.cpp` (`budget.*`) / L3 §5.3, L7 §4.3a |
| MD-57 | `mpc` 에서 RT 가 plan 을 따르는 동안 계획기는 탐색을 돌리지 않고 그 wake 에 구간을 다시 푼다. 지금의 코드가 그렇다 — 결정은 "탐색은 채택 뒤에도 돈다" 로 바뀌었고 E1-F16 · F17 이 이 행을 지운다 ([MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) §4). | `IB/src/controllers/catching/controller.cpp` (`Not under mpc`), `planner_closed_form.yaml` (헤더 주석) / L3 §4.7 · §5.3 |
| MD-58 | 재계획의 $x_0$ 는 RT 가 보고한 구간 (대기 구간, 없으면 따르는 구간) 에서 평가하고 같은 포구 전 격자점은 새 예측으로 다시 푼다 (`replan.same_point`). 대기 슬롯의 교체는 node 0 시각이 같을 때다. | `IB/include/integrated_bringup/logging/catching_diag_log_pod.hpp` (`kReplaced`), `demo_catching_controller.hpp` (`segment_pending`) / L7 §4.3a, L3 §5.3 |
| MD-59 | 폐기 — 측정 전용 키 `planner.segment.mpc.shadow` 는 지워졌다. | — / — |
| MD-60 | 간격이 둘인 구간은 payload 의 `n_pre` · `dt_pre_ns` 로 싣고 노드 시각은 `SegmentNodeTimeNs` 한 함수가 정한다 (`ValidateSegmentNodes` · 샘플러가 그것을 쓴다). | `RCI/trajectory.hpp` (TWO SPACINGS), `IB/src/controllers/catching/controller.cpp` / f §1.6, L0 (`kMaxSegmentNodes`) |
| MD-61 | 포구 뒤 재계획은 격자점 `replan.k_max` (출하 2) 까지이고 정지 끝은 그대로다. | `cfg` `replan.k_max` / f §1.6, L3 §5.3 |
| MD-62 | 첫 풀이의 기준은 관절별 최소 jerk 곡선이고 게시 조건은 풀이 성공 · 예산 · 효력 시각 전 · slack · 포구 노드 위치 오차 ≤ `publish.catch_pos_err_max` · 속도 극값 · 마지막 노드의 정지다. 못 넘으면 구간도 plan 도 게시하지 않는다. | `cfg` `publish.catch_pos_err_max` · `linearization.ref_speed_fraction`, `IB/include/integrated_bringup/logging/planner_events_csv.hpp` / f §1.5 |
| MD-63 | 첫 풀이의 $\hat p_b,\hat v_b,a_d$ 는 plan 의 값이고 재계획은 가장 새 궤적을 $t_c$ 에서 읽으며 $\Sigma_p$ 는 $t_c$ 를 감싸는 두 표본을 선형 보간한다. $W_p$ 는 $\Sigma_p$ 의 고유값마다 하한 · 상한을 둔 가중이고 공분산이 없으면 상수 가중이다. | `cfg` `catch.kappa` · `sigma_floor` · `w_max` · `w_const`, `RCS/catching/mpc_segment_core.cpp` (`CatchPositionWeight`), `RCS/catching/mpc_segment_planner.cpp` / f §1.6 |
| MD-64 | 코어는 포구 전 노드 수마다 하나와 정지 격자점마다 하나이고 같은 box ($\eta_v$ · $\eta'_\tau$ · $m_q$) 를 받으며 configure 에서 한 번씩 풀어 둔다. | `cfg` `eta_tau` · `m_q`, `RCI/mpc_segment_planner.hpp` / f §1.6 |
| MD-65 | `mode: mpc` 는 언제나 APPROACH 부터 HOLD 까지 구간을 따르고 `TRACKING → APPROACH` 는 plan 과 첫 구간을 같은 tick 에 함께 채택할 때만 걸린다. | `rtc_tools/rtc_tools/analysis/catching_trials.py` (docstring), `IB/src/controllers/catching/controller.cpp` / L7 §4.1 · §4.3a, L3 §5.3 |
| MD-66 | 대기 슬롯은 하나이고 node 0 시각이 같은 더 새 구간만 교체하며 다음 격자점 구간이 기다리는 시간은 나이 상한보다 작아야 한다 (configure 검사). | `rtc_tools/rtc_tools/analysis/catching_trials.py` (docstring), `IB/src/controllers/catching/lifecycle.cpp` / L7 §4.3a |
| MD-67 | 구간 판정에 plan 의 track (`token.generation`) 대조가 있다 (어긋나면 `plan` 거부). 작업공간 검사 부분은 MD-73 이 지움. | `IB/src/controllers/catching/controller.cpp` (`JudgeSegment` 호출) / L7 §4.3a |
| MD-68 | 첫 구간 node 0 전의 APPROACH 는 채택 tick 에 seed 한 명령을 들고, 따를 구간도 대기 구간도 없으면 `kParamsTbd` → `ABORT_SAFE` 다. | `rtc_tools/rtc_tools/analysis/catching_trials.py` (docstring), `IB/src/controllers/catching/controller.cpp` / L7 §4.1 · §4.3a |
| MD-69 | RT 는 따르는 구간 (`segment_active` · `segment_seq`) 과 대기 구간 (`segment_pending` · `segment_pending_seq`) 을 `PlannerRtState` 로 보고하고 DECEL 전 추종 tick 의 감독 사유는 공 궤적 샘플로 낸다. | `IB/include/integrated_bringup/controllers/demo_catching_controller.hpp` (`PlannerRtState` 필드) / L7 §4.3a · §4.2 |
| MD-70 | MPC 구간 계획기는 `PlanFirst` 와 `Replan` 만 풀고 `approach.n_pre_max` < 1 이면 `mode: mpc` 가 park 한다 (`kSegmentModeUnmet`). 정지 구간 전용 계획기와 명령 외삽 $x_0$ 경로는 없다. | `IB/src/controllers/catching/lifecycle.cpp` (`n_pre_max` 검사), `demo_catching_controller.hpp` / L3 §5.3, L7 §4.3a |
| MD-72 | 튜닝이 arm 마다 옮긴 값 (`catch.gamma_ref`, `planner.search.grid.slice.t_lead_min` 등) 은 YAML 키이고 컨트롤러가 configure 때의 값을 읽기 전용 파라미터로 미러한다. 튜닝의 판정 기준 자체는 측정 절차다 ([#663](https://github.com/hyujun/rtc-framework/issues/663)). | `life` (MD-72 주석의 `declare`), `IB/test/test_demo_catching_controller.cpp` / — |
| MD-73 | RT 는 `catch_box` 를 검사하지 않는다 — `catch_box` 는 계획기 탐색의 것이고 `mpc` 전제에서도 빠진다. `SegmentEvent::kWorkspace` 값 3 은 번호만 남아 쓰이지 않는다. | `IB/src/controllers/catching/controller.cpp` (MD-73 주석), `catching_diag_log_pod.hpp` (`kWorkspace`) / L7 §4.3a, L3 §4.9 |
| MD-74 | CLIK 의 가속 제약은 `dynamic` 으로 출하한다 (`joint_cmd.accel_constraint: dynamic`). 포구 층의 형태는 `kinematic` · `dynamic` 뿐이고 코드 기본값은 없다 (#712 — 결정 당시의 기본값 `box` 는 없앴다). | `IB/config/iiwa7_leap/controllers/demo_catching_controller.yaml` (`accel_constraint`), `IB/src/controllers/catching/lifecycle.cpp` / L5 (`accel_constraint`) |
| MD-75 | `ur5e_p1b` 는 `catch.gamma_ref` 0.6 을 출하하고 `iiwa7_leap` 은 채택한 값이 없어 1.0 이다. | `cfg` `catch.gamma_ref` (두 로봇) / f §1.3 |
| MD-76 | `iiwa7_leap` 의 `mpc` 는 첫 풀이 기준 궤적 문제로 미달인 채 출하값 그대로다. | `IB/config/iiwa7_leap/controllers/catching/segment_mpc.yaml` (주석) / — |
| MD-77 | G1 의 device group 은 `g1` (waist 3 + 왼팔 7 + 오른팔 7 = 17 관절) 과 `p1b` (손 10 관절) 둘이다. | `IB/config/g1_p1b/` / f §0 |
| MD-78 | tree 군의 `DemoJointController` 는 모델을 `sub_models` → `tree_models` → `arm` 순으로 찾고 tree 군의 팔 끝은 군 1 tree 의 `root_link` 다. | `IB/src/controllers/joint/controller.cpp` · `lifecycle.cpp`, `IB/include/integrated_bringup/support/model_config_lookup.hpp` / — |
| MD-79 | `rtc_mujoco_sim` 은 위치 서보 게인을 쓸 때 actuator 의 biastype 을 affine 으로 맞추고 torque 모드로 되돌리면 복원한다. 게인 없는 `<motor>` 의 position 모드는 거부하지 않고 그룹당 한 번 경고한다. | `rtc_mujoco_sim/src/mujoco_sim_loop.cpp` (affine 절), `rtc_mujoco_sim/test/test_motor_servo_gains.cpp` / — |
| MD-80 | G1 config 는 waist roll · pitch `max_torque` 를 URDF 값으로 두고 catch frame 은 `ur5e_p1b` 의 값 (provisional) 이며 `derive_accel_limits` 를 쓰지 않는다. `demo_joint_controller` 만 싣는다. | `IB/config/g1_p1b/controllers/` / — |
| MD-81 | 이 저장소는 `hand_description` 의 모델을 load 하지만 검증하지 않는다 — G1 모델을 읽는 gtest 가 없고 tree 경로는 합성 fixture 로 시험한다. | `IB/test/test_model_config_lookup.cpp` / — |
| MD-83 | 손끝 FK 는 `T_root_armtip · T_tip_mount · T_handroot_fingertip` 로 합성하고 device 관절 순서를 거는 helper 를 `OnDeviceConfigsSet` 과 `InitHandModel` 에서 부른다. | `IB/src/support/hand_fk_wiring.cpp`, `IB/include/integrated_bringup/support/hand_fk_wiring.hpp`, `IB/test/test_hand_fk_wiring.cpp` / — |
| MD-84 | `compare_mjcf_urdf` 는 MJCF 를 MuJoCo 가 컴파일하는 대로 읽는다 (default class tree, actuator 의 `forcerange` × `gear`). fixed link 병합은 `--link-map` 의 `fuse:` 로 선언한다. | `rtc_tools/rtc_tools/validation/compare_mjcf_urdf.py` / — |
| MD-85 | 팔 모델이 있는데 팔 끝 frame 이 풀리지 않으면 joint · task · compliance · wbc 의 `on_configure` 가 거부하고 판정은 `support/arm_tip_resolution` 한 함수다. | `IB/include/integrated_bringup/support/arm_tip_resolution.hpp` (`ArmTipUnresolvedReason`), `IB/test/test_arm_tip_resolution.cpp` / — |
| MD-86 | sim launch 네 개는 공통화하지 않고 쓰이지 않는 인자 (`kp` · `kd`, `mpc_engine`) 만 지웠다. `sim_g1_p1b` 의 `enable_mpc` 는 CPU layout 만 고르고 컨트롤러에 닿지 않는다. 모델 이름 조회는 `FindTreeModel` 이다. | `IB/launch/sim_g1_p1b.launch.py`, `IB/include/integrated_bringup/support/model_config_lookup.hpp` / — |
| MD-88 | catching 컨트롤러의 config 는 기능별 파일로 나뉜다 — 주 파일 (QP CLIK) · `search_grid.yaml` · `planner_closed_form.yaml` · `segment_mpc.yaml`. | `IB/config/<robot>/controllers/demo_catching_controller.yaml`, `catching/*.yaml` / L3 §6 |
| MD-89 | 두 로봇의 출하 segment mode 는 `mpc` 이고 코드 기본값 (키 없음) 은 `closed_form` 이다. | `IB/config/<robot>/controllers/demo_catching_controller.yaml` (`planner.segment.mode`), `cfg` / L3 §0 · §6, L7 §6 |
| MD-90 | CM 의 일반 `include:` 가 조각을 하나의 노드로 합친 뒤 override 를 적용하고 키 경로는 그대로이며 같은 leaf 가 두 파일에 있으면 에러다. segment 는 `planner.segment.mode` 하나로 고른다 (조각은 항상 다 포함한다). | `rtc_controller_manager/src/controller_config_loader.cpp` (`kIncludeKey`), `IB/test/test_shipped_catching_config.py` / L3 §6 |
| MD-91 | `planner.segment.mpc.enabled` 는 없고 segment 는 `planner.segment.mode` 로만 고른다. 코어 설계 파라미터 (비용 가중 · slack 벌점 · 선형화 · solver) 와 탐색 파라미터는 YAML 키이며 가속도 box 는 `robot.arm.qdd_max` (주 파일) 다. `joint_limits.max_acceleration` 은 지웠다. | `cfg` `cost.*` · `linearization.*` · `solver.*`, `RCI/planner_params.hpp` / L3 §6 |
| MD-93 | 키는 기능이 갖는다 — 탐색 파일 `search_grid.yaml` = `planner.search.grid.*`, closed_form 파일 = `reference.*` · `supervisor.decel.a_dec`, mpc 파일 = `planner.segment.mpc.*`, 주 파일 = 선택자 `planner.segment.mode` 와 `robot.arm.qdd_{max,provisional}`. 한 기능이 쓰는 설계값은 전부 그 기능의 파일에 있고 두 기능이 같은 수를 읽으면 각자 key 를 갖는다 (탐색의 `planner.search.grid.reference.{v_max,omega,zeta,a_max}` · `stop.a_dec`, mpc 의 `eta_v` · `v_eps`). 복사본은 원본의 검증 규칙을 따르고, 탐색 복사본이 closed_form 법칙의 값과 다르면 `closed_form` 에서 park (`kSearchCopyDiffers`), `mpc` 에서는 WARN 이다. 옮긴 18개 옛 key 와 지운 경로 키 (`accel_limits_*`) 가 있으면 `kRemovedKey` 로 park 한다. | `IB/config/<robot>/controllers/catching/*.yaml`, `rtc::catching::kRenamedCatchingKeys` (`par`), `life` (`SearchCopiesThatDiffer`) / L3 §6 |

## 5. v1 의 결정

### 5.1 `D-n`

| ID | 무엇이 성립하나 | 어디에 |
|---|---|---|
| D-1 | 수치 코어는 `rtc_controllers` 의 `catching` 하위 (namespace `rtc::catching`), 바인딩·YAML·launch·PointCloud2 파서는 `integrated_bringup` 에 있다 | `RCI/`, `IB/src/controllers/catching/`, `IB/include/integrated_bringup/controllers/catching/traj_input.hpp` / MASTER §4 |
| D-2 | 내부 시각은 절대 steady ns 이고, nrt 수신 때 `recv_steady − (recv_wall − stamp)` 를 한 번 계산한다. `header.stamp` 는 물리 샘플 시각 복원에만 쓰고 freshness 는 `recv_steady` 로 판정한다 | `RCI/time_types.hpp` (D-2 (3) 변환), `IB/src/controllers/catching/traj_input.cpp` (D-2 conversion), `traj_input.hpp` (`kStampOverflow`) / L0 §4.5, L1 §4.1·§5.2, MASTER §3 |
| D-3 | sim 은 wall clock 을 유지하고 시행별 clock 위상 오차 δ 를 기록한다 — δ 는 판정이 아니라 공변량이다 (D-S8-4 (c) 가 바꿈) | `tools/catching_trials.py` (δ 공변량), `tools/catching_pool.py` / L8 §4.5, MASTER §2.1·§5.3 |
| D-4 | vision 입력은 ball_perception 의 PointCloud2 레이아웃을 필드 이름으로 파싱하고 `generation`·`snapshot_sequence` 를 쓰며, 구독 QoS 는 `KEEP_LAST` depth 1 이다 | `IB/.../catching/traj_input.hpp`, `RCI/trajectory.hpp` (`generation`), `tools/vision_lane.py` / L1 §1·§4 |
| D-5 | `rtc::tsid::ClikReferenceGenerator` 는 옵션 (기본 off) 으로 확장돼 있고, off 면 기존 출력이 그대로다 | `rtc_tsid/include/rtc_tsid/kinematics/clik_reference.hpp`, `rtc_controllers/test/test_catching_mpc_segment_core.cpp` (w⊥ 기본 off) / MASTER §1.2·§7 |
| D-6 | CLIK 의 오차·J 는 측정 q 가 아니라 명령 q_c 에서 평가한다 (`evaluate_at_command`); 이때 `anchor_drift_max` 는 꺼야 하고 실추종은 `TRACK_ERR` 가 감시한다 | `life` (`cfg.evaluate_at_command = true`), `ctrl` (평가 상태 prelude), `clik_reference.hpp` / L4 §5.2, MASTER §7 |
| D-7 | 계획기 스레드는 MPC 스레드와 같은 방식 (`PeriodicRtThread` 형제) 으로 만든 별도 스레드다 | `IB/.../catching/planner_thread.hpp`, `hdr` (planner thread 절), `RCI/planner_io.hpp` / L3 §5.3, MASTER §2 |
| D-7a | 계획기 스레드는 SCHED_FIFO 를 쓸 수 있고 `mpc` role 을 공유하며, 수신→게시 지연을 `PlanSnapshot` 에 싣는다 | `RCI/planner_cycle.hpp`, `RCI/planner_io.hpp` / L3 §5.3·§9 |
| D-7b | 계획기 스레드의 slot 은 thread layout 의 `mpc` role 을 공유한다 (E-7 결정 J 가 "빈 slot 에 새 role" 을 대체) | — / L3 §2 (G3-3), §5.3 |
| D-7c | 계획기는 새 궤적을 받을 때 eventfd 로 깨어난다 (event 구동, 대기 상한 있음) | `IB/.../catching/planner_thread.hpp` (WAKE SOURCE), `ctrl` (wake after both stores) / L3 §5.3 |
| D-7d | 포구 자세 IK 감쇠는 `DifferentialIk` 의 σ_min 적응 λ 를 쓴다 — 과제 스텝은 D-26 이 QP 로 바꿨다 | `RCI/catch_pose_ik.hpp`, `RCI/catch_pose_ik_params.hpp` / MASTER §1.2 |
| D-8 | γ derate 는 v1 에 없다 — 실행 중 포화는 COMMITTED 전이면 RETREAT, 이후는 ABORT_SAFE | `RCI/soft_catch.hpp` (derate 미이식), `RCI/time_feasibility.hpp`, `RCI/trajectory.hpp` / L7 §4.6, L3 §4.5, MASTER §6·§10 |
| D-9 | γ 창의 TCP 속도는 `planner.search.grid.gamma.eta_v · planner.search.grid.reference.v_max` (0 < η_v ≤ 1) 이다. mpc 구간 계획기는 자기 `planner.segment.mpc.eta_v` 를 갖는다 | `RCI/time_feasibility.hpp` (`PlanningTcpSpeed`), `par` (`planner_search_grid_gamma_eta_v`) / L3 §4.5, L0 §5.3 |
| D-10 | catch frame 은 로봇 config 의 `urdf.extra_frames.<name>` 이고 모델 빌더가 full 모델에 넣어 파생 모델이 상속한다 | `life` (`ResolveCatchFrame`), `IB/config/ur5e_p1b/_base.yaml` (urdf 절) / L5 §11, L3 §4.2 |
| D-11 | 손 명령 포트 추상화는 없다 — 손은 `ControllerOutput` 의 손 device slot 에 직접 쓰고 `T_close,e2e` 는 종단 간 실측이다 | `life` (손 관절을 IK 에서 제외하는 곳), `RCI/hand_sequencer.hpp` / L3 §4.11, L6 §4.1 |
| D-12 | 공 사양·손 토크 권위 출처·T_close,tot 실측은 사용자 값이고 값이 없으면 YAML 에 provisional 로 표시해 실기 arm 을 막는다 | `IB/config/ur5e_p1b/controllers/demo_catching_controller.yaml` (D-12 ball), `par`, `RCS/params/catching_params.cpp` / MASTER §9, L0 §5.3 |
| D-13 | E-STOP 은 어느 단계든 hold 후 비무장 `IDLE`, 해제 뒤 자동 재개 없음; `FAULT` 는 컨트롤러 소유로 global E-STOP 에 승격하지 않는다 (하위 D-S9-A~L) | `ctrl` (E-STOP·fault hooks), `IB/test/test_catching_supervisor_scenarios.cpp`, `IB/test/test_catching_cm_services.cpp` / L7 §4.1 |
| D-14 | 공 발사는 (p0, v0, ω) 를 명시하는 srv `LaunchBall` 로 한다 | `rtc_msgs/srv/LaunchBall.srv`, `rtc_mujoco_sim/test/test_projectile_ball.cpp` / L0 §5.4 |
| D-15 | vision 예측 사양의 요구는 포구 제어기가 정한다 — 수신 지평이 `io.horizon_min` 보다 짧으면 후보에서 제외하고 진단한다 (거부가 아니다) | `par` (`io_horizon_min`), `RCI/traj_ingress.hpp`, `RCI/trajectory.hpp` (`kHorizonShort`), `ctrl` / L1 §4.1, MASTER §5.1 |
| D-16 | 관절 가속 box 는 토크 한계에서 도출한 보수적 상수이고 값은 `robot.arm.qdd_max` (주 파일) 다. 탐색의 도달시간과 관절공간 정지가 쓰고, CLIK 의 가속 제약은 토크 행이다. 파일 · 경로 키 장치는 MD-91 이 없앴다 | `life` (`ApplyArmAccelBox`), `tools/derive_accel_limits.py` / L3 §4.3, L5 §6 |
| D-17 | catch frame 의 부모 · offset · 자세는 YAML 값이고 접근축은 그 frame 의 +z 다. 모델 빌드 때 읽히므로 바꾸면 재configure 해야 한다 | `par`, `life`, `IB/test/test_catch_frame_models.cpp` / L5 §11, MASTER §6 |
| D-18 | 투척 가능성은 팔 manipulability (w₅ 5행, 정의별 하한) 로 판정하고 IK 는 a_d = −v̂ 자세를 푼다 | `RCI/catch_pose_ik.hpp`, `RCI/grid_catch_search.hpp`, `tools/catchability_map.py` / L3 §4.1·§4.2 |
| D-19 | 새 상태 · CSV 는 GUI 패널과 plot 을 함께 갖는다 | `IB/integrated_bringup/demo_gui/catching.py`, `IB/test/test_demo_gui_catching.py` / L8 §11 |
| D-20 | 포구 상태는 `rtc_msgs/CatchingState` 로 나가고 `PublishRole` 은 늘지 않으며 모든 `Compute()` tick 에서 Store 한다 | `rtc_msgs/msg/CatchingState.msg`, `ctrl` (tick record), `IB/include/integrated_bringup/logging/catching_diag_log_pod.hpp` / L7 §5.2, L3 §5.2 |
| D-21 | RT 는 매 tick `Load()` 를 무조건 한 번 하고 새 스냅샷 여부는 payload 의 `snapshot_sequence` 로 판정한다 | `RCI/trajectory.hpp` (`ProvenanceToken`), `RCI/traj_ingress.hpp`, `ctrl` (lane 의 Load) / L1 §5.3, L2 §5.2 |
| D-22 | 궤적·공분산·`PlanSnapshot` 이 같은 provenance token 을 싣고 계획기는 token 불일치 결과를 버린다 | `RCI/trajectory.hpp`, `RCI/traj_ingress.hpp`, `RCI/planner_io.hpp` / L1 §5.2 |
| D-23 | activation 경계는 `ActivationGeneration()` 으로 판정하고 plan·FSM·타이머 무효화는 RT tick 이 유일 writer 다 | `ctrl` (activation 경계), `hdr`, `IB/test/test_catching_planner_lane.cpp` / L1 §5.2·§5.3 |
| D-24 | 지문 센서 lane 이 수신 steady 시각·sequence·valid 를 싣고 `TIP_STALE` 은 그 나이로 판정한다 | `IB/include/integrated_bringup/logging/catching_diag_log_pod.hpp`, `IB/include/integrated_bringup/backends/mujoco_native_backend.hpp`, `ctrl` / L7 §4.2, L1 §5.4 |
| D-25 | 포구 자세 IK 는 영공간 항 k_w∇log w₅ 로 roll manipulability 를 올리고 ρ 는 J 와 잔차 양쪽에 곱한다 (L3 의 "roll 최대화 제외" 를 번복) | `RCS/catching/catch_pose_ik.cpp`, `IB/config/ur5e_p1b/controllers/catching/search_grid.yaml` (`k_manip`) / L3 §4.2 |
| D-26 | 포구 자세 IK 의 과제 스텝은 제약 QP (ProxQP, `QPSolverWrapper`) 이고 영공간 항은 QP 밖에서 더한다 | `RCS/catching/catch_pose_ik.cpp`, `rtc_controllers/CMakeLists.txt` (S1.9 D-26) / L3 §4.2 |
| D-27 | 분석은 엄격하고 실행은 reachable 후보에 도전한다 — 성능 게이트 탈락은 후보를 지우지 않고 순위·진단이다 | `RCI/grid_catch_search.hpp` (TWO KINDS OF GATE), `RCI/rank_gates.hpp`, `RCI/gamma_rollout.hpp` / L3 §4.1·§4.6, L1 §4.1 |

### 5.2 `C-n`

`#511 C-n` 과 `Stage C-n` 은 다른 작업의 번호다.

| ID | 무엇이 성립하나 | 어디에 |
|---|---|---|
| C-1 | vision 메시지는 한 점이라도 `validity` 가 VALID 가 아니면 통째로 거부한다 | `RCI/traj_ingress.hpp`, `IB/.../catching/traj_input.hpp` (`kNotEvaluated`), `rtc_controllers/test/test_catching_traj_ingress.cpp` / L1 §4.4 |
| C-2 | `header.stamp` 기반 나이 거부는 없고 원점 지연은 진단만 한다 (미래 stamp 거부는 별개) | — / L1 §4 (D-2 (3) 와 같은 절) |
| C-3 | IK·게이트는 w₅, w₆ 는 같은 자세에서 따로 기록하고 차원이 달라 pooling 하지 않는다 (`planner.search.grid.ik.manip_min` 은 D-18 게이트로 대체) | `RCI/catch_pose_ik.hpp` (`w6`), `RCI/trajectory.hpp`, `rtc_msgs/msg/CatchingState.msg` (`plan_w6`), `tools/catchability_map.py` / L3 §4.2·§6, MASTER §6 |
| C-4 | 포구 후보마다 IK seed 는 대기 자세에 고정한다 | — / L3 §4.2 |
| C-5 | (`tools/vision_lane.py` 가 인용) 예측 일관성은 ball_perception 자신의 evaluator 로 본다 | `tools/vision_lane.py` / — |
| C-7 | reset floor 와 reset epoch 는 같은 자리에서 함께 움직이고, 계획기 wake eventfd 는 어떤 리셋에서도 비우지 않는다 | `hdr` (Reset table) / L7 §4.8 |
| C-10 | `supervisor.ready.wait_pose` 키는 없다 (있으면 파서가 거부) — 대기 자세는 `planner.wait_pose` 다 | `RCS/params/catching_params.cpp` / L7 §6 |
| C-12 | 투척 드라이버는 투척마다 wait_pose 를 맞추지 않는다 (S7 에서 제거) | `IB/integrated_bringup/catching_sim_trials.py` / — |
| C-13 | homing 은 관절공간 사다리꼴이다 (QP/CLIK 비의존) | `RCI/joint_home.hpp` / L7 §4.1 |
| C-14 | 손 명령 시각 `t_cmd` 는 손 시퀀서가 동결된 t_c 로 계산한 값 하나가 쓰이고 `PlanSnapshot::t_cmd_ns` 는 기록이다 | `RCI/hand_sequencer.hpp` / L3 §4.11, §4.7 부근 |
| C-17 | E-STOP·activation 리셋은 시퀀서를 비활성으로 두고 손은 측정 자세 latch 가 잡는다; 정지 뒤 다음 읽을 수 있는 tick 은 측정 자세에서 재시드한다 | `ctrl` (re-seed at the MEASURED pose) / L7 §4.8 |
| C-20 | sim 지문은 무잡음이라 접촉 판정에서 `f_min` 만 유효하고 `k_sigma` 는 실기 전용이다 | `par` (`supervisor_contact_*`) / L7 §6 |
| C-25 | 서보 지연 τ 0.2 용 overlay 조합은 은퇴했다 (D-S8-13 이 바꿈) — 실기 lead-off 도 `T_arm` 은 읽힌다 | `life` (T_arm 을 읽는 곳) / — |
| C-29 | 재무장 리셋은 `qp_fail_streak_` 를 지우지 않는다 (activation·fault reset·HOLD 판정만 0) | `IB/test/test_catching_reset_probe.cpp`, `hdr` (Reset table) / L7 §4.8 |
| C-30 | 재무장 뒤 손은 `Ready` (`q_pre`) 위상이다 — `Open` 이 아니다 | `RCI/hand_sequencer.hpp` / L7 §4.8 |
| C-31 | `JudgePlan` 조건 (g): `t_c − now ≤ T_freeze` 인 plan 은 `kTooLate` 로 거부한다 (R-ADMIT 과 같은 규칙) | — / L7 §4.1, L3 §4.11 |
| C-32 | 재무장은 실려 있는 명령 q_c 를 다시 seed 하지 않는다 (속도만 0) | `ctrl` ("A command that is ALREADY carried is kept") / L7 §4.8 |
| C-33 | `IDLE`·`DECEL`·`HOLD`·`RETREAT` 는 vision lane 앞에서 판정한다 (R-ORDER 의 위치) | — / L7 §4.1 |
| C-35 | `ABORT_SAFE` 의 정지는 원인과 무관하게 항상 관절공간 `JointSpaceStopStep` 이다 | `ctrl` (`RunJointSpaceAbort` 부근) / L7 §4.1, mpc_multiframe_clik_formulation §1.3 |

### 5.3 `P-n` · `A-n` · `A-S5-n`

`#234 P-n` 은 다른 작업의 번호다.

| ID | 무엇이 성립하나 | 어디에 |
|---|---|---|
| P-1 | E-STOP 최소 계약: (a) 훅은 atomic 요청·epoch 만 갱신하고 RT tick 이 유일 writer (b) 무효화는 tick 이 D-23 순서로 (c) 해제 뒤 자동 재개 없음 — 무장 latch 를 내린다 (d) ClearEstop 과 ResetFault 는 서로의 latch 를 건드리지 않는다 (e) homing 은 운동이므로 무장 latch 를 요구한다 | `hdr` (THE P-1 CONTRACT), `ctrl` (E-STOP and fault hooks), `IB/test/test_demo_catching_controller.cpp`, `IB/test/test_catching_cm_services.cpp` / L7 §4.1·§4.2·§4.8 |
| P-3 | ball_perception 요청 이슈는 만들지 않고 레이아웃 변경은 파서의 필드·datatype 검사와 해시 진단이 감지한다 | — / MASTER §5.1, L8 |
| A-2 | D-7a 스케줄러 판정은 제어 PC 측정으로 정한다 (측정은 생략돼 초기값 FIFO 유지) | — / L3 §5.3·§9 |
| A-3 | 공분산은 RT 스냅샷에서 분리해 계획기 쪽 버퍼에만 두고 (같은 token) NaN(모름) 처리도 계획기 한 곳이다 | `RCI/traj_ingress.hpp` (Covariance), `hdr` / L1 §1·§5.2, L3 §5.3, L0 §5.2 |
| A-4 | 계획기 탐색의 단일 진입점이 `CatchSearch::Plan` (구현 `GridCatchSearch::Plan`) 이고 NLP 전환은 v1 에 없다 (`mpc` 는 구간 계획이지 탐색 NLP 가 아니다) | `RCI/catch_search.hpp`, `RCI/planner_cycle.hpp` (`PlanOnce`), `RCS/catching/planner_cycle.cpp` / L3 §4.1 |
| A-5 | `DECEL` 진입은 t_c 시각 기준이고 지문 센서는 결과 판정·abort 전용이다 | — / L7 §4.1 ("감속은 시각 기준"), L4 §5, MASTER §10 |
| A-6 | COMMITTED 이후 stale 은 동결 plan 으로 계속하고 `supervisor.stale_committed_max_s` 초과 시 ABORT_SAFE 다 | `par` (`supervisor_stale_committed_max_s`), YAML `supervisor` / L7 §4.2 |
| A-7 | → D-15 로 합쳐졌다 (vision 요구 사양은 제어기가 정한다) | — / D-15 |
| A-S5-1 | 실기 config 소비 키에 provisional·TBD 가 있으면 park 한다 (configure SUCCESS · activate 거부) | `hdr`, YAML 머리 주석 / L0 §5.3, L7 §4.5 |
| A-S5-2 | `io.future_tol` 은 실기 키와 `sim.io.future_tol` 이 따로다 | `par`, `RCS/params/catching_params.cpp` / L1 |
| A-S5-3 | 조작 채널은 read-write 파라미터 `catching.enable` 이고 tick 이 E-STOP·fault 때 내린다 | `hdr` (THE ARM/DISARM CHANNEL) / L7 §4.1 |
| A-S5-4 | 같은 generation 에서 sequence 가 줄면 거부하고 generation 이 바뀌면 기대를 리셋한다 | `RCI/traj_ingress.hpp` / L1 §4.4 |
| A-S5-5 | 공분산은 시간 보간 없이 vision 격자에서만 쓰고 token 실은 snapshot 으로 계획기에만 간다 | `RCS/catching/grid_catch_search.cpp`, `RCI/traj_ingress.hpp` / L1 §5 |
| A-S5-8 | `diagnostic.oracle_plan` 을 RT tick 이 고정 `PlanSnapshot` 으로 소비한다 | `hdr`, YAML (S5.5 oracle plan) / L8 |
| A-S5-9 | 팔 지연 (G5-E) 의 주입은 설치되지 않는 테스트 fixture 의 폐루프 plant 로 한다 — 런타임 코드에 지연 주입은 없다 | `IB/test/arm_lag_fixture.hpp` / L5 §9 |
| A-S5-10 | abort 정지는 할당·QP 없는 순수 함수 ramp 다 | `hdr` (`Walk the arm command to a stop`), `life`, `ctrl` / L7 §4.1 |
| A-S5-11 | 소비 키 `reference.*` 블록은 출하 프로파일에 있어야 하고 `reference.provisional` 이 있다 | `life`, `par` / L5 §6 |
| A-S5-12 | sim 도 TBD 소비 키면 configure 를 거부하지 않고 park 한다 | `hdr`, `IB/README.md` / L7 §4.5 |

### 5.4 `D-S7-n` · `D-S8-n` · `D-S9-n`

| ID | 무엇이 성립하나 | 어디에 |
|---|---|---|
| D-S7-1 | `robot.hand.T_close_timeout` 이 없으면 2 × `T_close_e2e` 로 유도한다 | `par` (`kCloseTimeoutPerE2e`), `RCS/params/catching_params.cpp` / L6 §6 |
| D-S7-2 | `supervisor.stale_committed_max_s` 코드 기본은 0.10 s 다 | `par` (`supervisor_stale_committed_max_s`) / L7 §6 |
| D-S7-3 | 접촉 판정은 `\|F − b\| > max(f_min, k_sigma·σ̂)` 가 `n_debounce` 샘플 연속이고 `m_min` 개 손끝이 합의하면 잡은 것이다 | `par` (`supervisor_contact_*`), `rtc_controllers/test/test_catching_params.cpp` / L7 §4.4·§6 |
| D-S7-4 | `REF_SATURATED` 는 연속 `supervisor.sat_ticks` tick 이면 승격하고 코드 기본은 60 이다 | `par` (`supervisor_sat_ticks`) / L7 §4.2·§6 |
| D-S8-1 | sim 서보 지연은 L5 선행 보상을 sim overlay 로 보상한다 (출하 YAML 불변; 정확 역보상은 기각) | — / L5 §4.5 |
| D-S8-2 | 평가용 동결 분포는 로봇별 gate 지도 상자이고 투척 반복은 회귀 세트로만 쓴다 | `IB/integrated_bringup/catching_sim_trials.py`, `IB/test/test_catching_sim_trials_s8.py` / MASTER §6, L8 |
| D-S8-3 | 판정 하한 floor 는 0.35, n_valid 는 200 이고 짧은 표본으로는 판정하지 않는다 | `tools/catching_trials.py`, `tools/catching_pool.py` (`DEFAULT_N_VALID_TARGET`) / L8 §9.1 |
| D-S8-4 | (c) D-3 δ 는 판정이 아니라 공변량이고 무효는 rig 실패만이다 | `tools/catching_trials.py` / L8 §4.5 |
| D-S8-5 | 램프 연속 (⑦) 은 조건부이고 S8-B 에서 조건을 못 채워 하지 않는다 | — / L3 §4.7 |
| D-S8-6 | RETREAT 의 손 `q_pre` 대기는 `robot.hand.T_release_timeout` 으로 끝난다 (`{kRetreat, kHandTimeout, kIdle}`, 같은 tick disarm) | `par`, `hdr` (release 시각), `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.2·§4.8 |
| D-S8-7 | ν̄ (`PRED_INCONSISTENT`) 와 `io.pred.nu_reg` 는 은퇴했고 σ_ℓ 는 `planner_events.csv` 기록만 한다 | `tools/vision_lane_probe.py` / L1 §6, L7 §4.2 |
| D-S8-8 | 판정은 지문 합의에 손 관절 stall 증거 (q·토크) 를 OR 로 더한다 | `hdr` (verdict 절), `ctrl`, `IB/.../catching_diag_log_pod.hpp` / L7 §4.4 |
| D-S8-9 | γ0 arm 은 `gamma.grid: [0.0]` + `planner.search.grid.hand.d_eff: 10.0` 이다 (실험 arm 이며 출하값이 아니다) | — / — |
| D-S8-10 | CLOSING 기록이 한 tick 늦는 것은 문서 기록만이다 | — / L7 §4.1 (R-CLOSE) |
| D-S8-11 | 공 arm 은 `beanbag` preset 그대로 쓴다 (`tennis_soft` 는 없다) | — / — |
| D-S8-12 | 손 근처 투척은 S8-F 탐색이고 게이트 밖이다 | `IB/config/ur5e_p1b/sim_overlays/s8f_reach_first.yaml` / — |
| D-S8-13 | sim 팔 플랜트는 τ 0.05 s 로 서보 게인을 정하고 `joint_cmd.lag.{T_arm, lead_enable}` + `T_freeze` overlay 로 돈다 (sim 플랜트는 선택이다) | `IB/config/ur5e_p1b/controllers/demo_catching_controller.yaml`, `life`, `IB/test/test_catching_tracking.cpp` / L5 §4.5 |
| D-S8-14 | leap 묶음: 팔 서보 τ 0.05, 지도 라운드 최다 열림 자세가 출하 `planner.wait_pose`, overlay `catch_lead_on` | `IB/config/iiwa7_leap/sim_overlays/catch_lead_on.yaml`, `IB/config/iiwa7_leap/controllers/demo_catching_controller.yaml` / — |
| D-S8-15 | leap 상자는 p1b 폭 상자 중 열림 비율 최대이고, 유효성 조건은 상승 구간에 대기 자세 로봇과 2 cm 이상 떨어진 투척이다 | `IB/integrated_bringup/catching_sim_trials.py` (iiwa7_leap 상자 주석) / — |
| D-S8-16 | S8-E 본 평가 계획: floor 0.35 동결, 튜닝 데이터 비합산, 사전 선언한 top-up | `tools/catching_pool.py`, `tools/catching_trials.py`, `tools/catching_vision.py` / L8 §9.1 |
| D-S8-17 | `rtf_trial_min` < 0.95 시행이 있는 unit 은 rig 실패라 같은 seed 로 재실행한다 (원 판정과 재실행 판정을 둘 다 기록) | `IB/integrated_bringup/catching_sim_trials.py` (host 부하 감시), `IB/test/test_catching_sim_trials_host_watch.py` / L8 §9.1·§10 |
| D-S8-18 | 계획기의 도달시간 한계는 실행 envelope 을 쓸 수 있고 (sim 실험 arm — 지금은 `robot.arm.qdd_max` 를 overlay 로 바꾼다; 경로 키는 MD-91 이 없앰), `reference.omega` (탐색의 복사본 `planner.search.grid.reference.omega` 포함) 는 10 고정, 판정 unit 은 같은 seed ≥ 2 회 + McNemar 다 | `tools/catching_arm_budget.py` / — |
| D-S8-19 | γ 창 재정의 후속은 종결했다 — 창은 v_dir,max 가 묶고 계획기는 이미 γ_f = g_max 를 쓴다 | — / L3 §4.5 |
| D-S8-20 | 대기 자세 재선정 (④) 은 종결 — 출하 `planner.wait_pose` 를 유지하고 `wait_pose_source: current` 를 남긴다 | — / L7 §4.1·§4.5 |
| D-S9-A | E-STOP 중 손은 CM 이 측정 자세로 hold 하고 HOLD 중이면 공을 놓는다 (hold 목표는 latch 시점 측정 — pre-S10 Q1 이 바꿈) | `IB/test/test_catching_supervisor_scenarios.cpp`, `IB/test/test_catching_cm_services.cpp` / L7 §4.1 |
| D-S9-B | E-STOP 반응은 단계와 무관하게 일률이다 — 즉시 hold → `IDLE` (`FAULT` 는 유지), 진행 중 시행은 `Aborted` | 같은 두 테스트 / L7 §4.1 |
| D-S9-C | 해제 뒤 복귀는 비무장 `IDLE`·q_c 재시드 → 운용자 재무장 → homing 이다 | `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.1 |
| D-S9-D1 | 운동 기한 (`ABORT_SAFE` 정지 램프 · `RETREAT` 정지 단계 · 복귀 단계) 을 넘으면 `ABORT_ESCALATED` 로 `FAULT` 다 (키 `supervisor.deadline.stop_s` · `return_s`, provisional) | `ctrl`, `life`, `hdr` / L7 §4.1·§4.2·§6 |
| D-S9-D2 | `n_qp` 는 시행 단위다 — CLIK 실패로 끝난 시행마다 +1, HOLD 판정 도달이 0, E-STOP 은 불변, 0 은 fault reset·activation 도 | `ctrl`, `hdr` (`qp_fail_streak_`), `IB/test/test_catching_reset_probe.cpp` / L7 §4.1·§4.2·§4.8 |
| D-S9-D3 | `ResetFault` 는 팔이 정지하지 않았거나 속도 lane 을 읽을 수 없으면 거부한다 (latch 유지) | `hdr`, `ctrl`, `IB/.../catching_diag_log_pod.hpp` / L7 §4.1 |
| D-S9-E1 | `FAULT` 는 global E-STOP 으로 승격하지 않는다 | `IB/test/test_catching_cm_services.cpp` / L7 §4.1 |
| D-S9-E2 | sim E-STOP 주입은 repo 밖 overlay + relay rig 다 (새 srv 없음) | — / L7 §9 |
| D-S9-F | G8-H 의 테스트 공백 분기를 S9a 테스트에 포함한다 | `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §9 |
| D-S9-G | 실기 몫 (드라이브 hold 값·실기 E-stop 과 latch 의 관계·운동 기한 값) 은 S10 이다 | YAML `supervisor.deadline` 주석 / L7 §1·§6·§10 |
| D-S9-H | GUI 의 E-STOP 해제는 2 단계이고 `catching_diag` 플롯이 두 latch 를 음영으로 보인다 | `rtc_tools/rtc_tools/plotting/plotters/catching.py` / L8 §11 |
| D-S9-I | `RETREAT` ↔ `ABORT_SAFE` 순환 카운터는 두지 않는다 | — / L7 §4.1 |
| D-S9-J | `rtc_msgs` 의 `qp_fail_streak` 는 주석만 바뀌었고 wire 는 같다 | `rtc_msgs/msg/CatchingState.msg` / — |
| D-S9-K | 시행 중 팔을 읽을 수 없으면 운동을 멈추고 기한을 센다 | `ctrl`, `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.1·§9 |
| D-S9-L | E-STOP 중 `reset_fault` 는 latch 를 내리되 모드는 해제까지 `FAULT` 이고 해제 tick 에 `IDLE` 이다 | `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.1·§9 |

### 5.5 `Q<n>`

같은 번호가 여러 계열에 있다. 코드는 번호만 적으므로 주석의 문맥으로 계열을 가린다: **S7** (손 시퀀서 · 슈퍼바이저), **S9** (E-STOP · fault), **pre-S10** (실기 전 정리 — CM 쪽 코드가 많다), **S4a** (손 타이밍), **S1.9** (포구 자세 IK). `rtc_mpc` · `rtc_urdf_bridge` · `ur5e_bt_coordinator` 의 `Q<n>` 은 다른 작업의 번호다.

| ID | 무엇이 성립하나 | 어디에 |
|---|---|---|
| Q1 [S7] | 접촉 `supervisor.contact.f_min` 은 사용자 값이다 | `par` (`supervisor_contact_f_min`) / L7 §4.4 |
| Q1 [S9] | 운동 기한 키는 정지·복귀 둘이다 (D-S9-D1) | `life` / L7 §6 |
| Q1 [pre-S10] | CM 의 E-STOP hold 목표는 latch 시점의 측정으로 device 별 고정이고 해제 때 버린다 | `rtc_controller_manager/src/rt_controller_node_estop.cpp`, `rtc_controller_manager/test/test_estop_hold_latch.cpp` / — (README: `rtc_controller_manager/README.md`) |
| Q2 [S9] | `ResetFault` 는 속도 lane 을 읽을 수 없어도 거부한다 (fail-closed) | `ctrl` ("Fail-closed … Q2") / L7 §4.1 |
| Q2 [pre-S10] | 해제 검증 창 동안 hold 치환을 유지하고 통과 뒤에만 컨트롤러 출력을 낸다 | `rtc_controller_manager/src/rt_controller_node_services.cpp`, `rt_controller_node_rt_loop.cpp` / — |
| Q2 [S1.9] | `CatchPoseIk::Solve` 의 좌표는 모델 world 이고 base→world 변환은 호출자 몫이다 | `RCI/catch_pose_ik.hpp`, `RCI/catch_pose_ik_batch.hpp` / L3 §4.2 |
| Q1 [S1.9] | `CatchPoseIk` 는 `GetReducedModel("arm")` 의 arm sub-model 을 받는다 | `RCI/catch_pose_ik.hpp` / L3 §4.2 |
| Q3 [S7] | homing·복귀는 관절공간 법칙이다 (`JointSpaceHome`, QP 비의존) | `RCI/joint_home.hpp`, `par` / L7 §4.1 |
| Q3 [S9] | E-STOP 중 `reset_fault` 는 현행 유지다 (D-S9-L) | — / L7 §4.1 |
| Q3 [pre-S10] | `/system/estop_status` 는 transient_local 이다 | `rtc_controller_manager/src/rt_controller_node_publishers.cpp` / — |
| Q4 [S1.9] | `CatchPoseIkOptions` 의 YAML 파서는 `ParseCatchPoseIkParams` 가 한 곳에서 맡는다 | `RCI/catch_pose_ik_params.hpp`, `RCI/catch_pose_ik.hpp` / L3 §6 |
| Q4 [S7] | 손 `T_pre`·시각 기반 preshape 는 폐기됐다 — `q_open` 은 homing 중에만, 대기 중은 항상 `q_pre` | `RCI/hand_sequencer.hpp` (THE HAND RULE), `RCS/params/catching_params.cpp` (`T_pre` 거부), `ctrl` / L7 §4.1·§4.5, L6 §5 |
| Q4 [pre-S10] | 가속 box 의 `robot.arm.qdd_provisional: true` 도 실기 구성을 park 한다 | `life`, `hdr` (`ApplyArmAccelBox`), `IB/test/test_catching_supervisor_scenarios.cpp` / L5 §6, L7 §4.5 |
| Q5 [S7] | 손 hold 규칙은 `robot.hand.hold.mode: close_target \| measured_offset` + `hold.delta_rad` 다 | `rtc_controllers/test/test_catching_hand_sequencer.cpp` / L6 §4.4 |
| Q5 [pre-S10] | `joint_cmd.lag.provisional` 이 `T_arm` 식별 전 실기를 park 한다 | `RCS/params/catching_params.cpp`, `rtc_controllers/test/test_catching_params.cpp` / L5 §6 |
| Q6 [S7] | `REF_SATURATED` 는 연속 `sat_ticks` 이다 (D-S7-4 와 같은 내용) | `par`, `ctrl` / L7 §4.2 |
| Q6 [pre-S10] | YAML 의 `planner.search.grid.hand.provisional` 줄은 없다 (파서가 안 읽고 `planner.provisional` 이 덮는다) | — / L3 §6, L6 §6 |
| Q7 [pre-S10] | `width == 0` 인 빈 cloud 는 "트랙 없음" 으로 따로 세고 WARN 하지 않는다 (`no_track`) | — / L1 §5.1 |
| Q8 [S7] | `stale_committed_max_s` 코드 기본이 0.10 이다 (D-S7-2) | `par` / L7 §6 |
| Q9 [pre-S10] | 대기 자세 채택·ARMED 판정은 속도 lane 을 읽을 수 없으면 거부·미룬다 | `ctrl` (`ArmNotAtRestForReset` 등), `IB/.../catching_diag_log_pod.hpp` / L3 §6, L7 §4.5 |
| Q10 [S4a] | 공 직경·질량·반발은 sim 공의 값이고 D-12 가 닫히면 `d_eff`·`r_cap` 을 다시 유도한다 | `IB/config/ur5e_p1b/controllers/demo_catching_controller.yaml` (`core.ball`) / MASTER §9 |
| Q12 [S7] | RETREAT 의 판정별 hand release 분기였으나 지금은 판정과 무관하게 대기 자세 도착에서만 release 한다 (Q14 와 함께 교체) | `ctrl` (RETREAT never moves the hand), `rtc_controllers/test/test_catching_hand_sequencer.cpp` / L7 §4.1·§4.8 |
| Q12 [pre-S10] | CM 의 "검증 대기" 플래그는 서비스가 세우고 RT 루프가 내린다 | `rtc_controller_manager/include/rtc_controller_manager/rt_controller_node.hpp` / — |
| Q13 [S7] | 팔이 이미 `pose_tol` 안이면 homing 을 건너뛰고 손만 `q_pre` 로 둔다 | `ctrl` (Q13 skip), `IB/test/test_catching_supervisor_scenarios.cpp`, `par` (`supervisor_ready_pose_tol`) / L7 §4.1 |
| Q13 [pre-S10] | 검증 창 동안 발행하는 `estop_status` 는 latch ∨ 대기다 | `rtc_controller_manager/src/rt_controller_node_estop.cpp` / — |
| Q14 [S7] | (교체됨 — Q12 와 함께) 접촉 확정 뒤 abort 만 release 하던 규칙 | `ctrl` (주석), `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.8 |
| Q14 [pre-S10] | `shape_estimation` 이 처음으로 E-STOP 을 받고 그 구독은 transient_local 이다 | `shape_estimation/test/test_estop_subscription.cpp` / — |
| Q15 [S7] | 새 시행은 새 track (generation) 이 올 때까지 이전 시행의 공을 거부한다 (R-TRACK) | `ctrl`, `hdr`, `IB/test/test_catching_tracking.cpp` / L7 §4.1·§4.8 |
| Q15 [pre-S10] | `no_track` bucket 은 거부 enum 끝에 붙고 `CatchingState` .msg 는 불변이다 | — / L1 §5.1 |
| Q16 [pre-S10] | 속도 lane 비판독 판정은 손 정착 (`HandSettledAtPre`) 에도 적용한다 | `ctrl`, `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.5 |

### 5.6 `R-…` — 슈퍼바이저 driver 의 규칙

| ID | 무엇이 성립하나 | 어디에 |
|---|---|---|
| R-ADMIT | `JudgePlan` 조건 (g): `t_c − now ≤ T_freeze` 인 plan 은 `kTooLate` 로 거부한다 | `ctrl` (judge (g)) / L7 §4.1, L3 §4.11·§5.2 |
| R-CLOSE | COMMITTED → CLOSING 은 시퀀서의 `close_issued` (`now ≥ t_cmd − h/2`) 로 전이한다 | `ctrl`, `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.1, L6 §4.3 |
| R-DECEL-ENTRY | tick 의 `now` 는 `Compute` 머리에서 한 번 읽어 법칙·시퀀서·판정이 공유하고 `DECEL` 진입은 기준 상태에서 이어진다 | `ctrl` (tick 의 ONE clock read), `hdr` / L7 §4.1·§4.3 |
| R-IDLE | 움직이는 carried 명령으로 `IDLE` 에 들어오면 `IDLE` 이 `JointSpaceStopStep` 으로 정지까지 램프한다 | `ctrl` (`RunIdleMotion`), `IB/test/test_catching_reset_probe.cpp` / L7 §4.1·§4.5·§4.8 |
| R-ORDER | `IDLE`·`DECEL`·`HOLD`·`RETREAT` 는 vision lane 앞에서, COMMITTED/CLOSING 은 항상 추종 법칙을 돌린다 | `ctrl`, `hdr` / L7 §4.1·§4.2 |
| R-PREC | 한 tick 의 사유 우선순위는 ESTOP > fault reset/escalation > 준비 상실 > 법칙 실패 > 시간 전진 > 기록 전용 이다 | `ctrl` (R-PREC 주석), `IB/test/test_catching_supervisor_scenarios.cpp` / L7 §4.1·§4.2 |
| R-TRACK | 동결 후 샘플은 commit 시점 generation 에 고정되고 재무장 뒤 직전 시행의 generation 은 usable 이 아니다 | `ctrl`, `hdr`, `IB/test/test_catching_tracking.cpp` / L7 §4.1·§4.8 |
| R-WATCHDOG | homing·retreat 법칙도 `track_err_` 를 계산해 `supervisor.track_err_abort` 를 본다 (`IDLE` 은 disarm) | `ctrl` / L7 §4.1 |
