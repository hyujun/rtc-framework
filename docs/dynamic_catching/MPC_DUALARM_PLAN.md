# MPC · dual-arm catching — 구현 계획

- 작성일: 2026-09-30 (r6 — E0-F02 baseline 결과: §8, 결정 MD-19. r5 — 빌드 경로 확정: 결정 MD-17 · MD-18. r4 — 브랜치 계획 추가. r3 — formulation v0.4 확정 반영: 결정 MD-9 – MD-16, 예측 격자 sweep, 게이트 G-1 의 검정 방법)
- 상태: **E0 진행 중** — E0-F01 · E0-F02 · E0-F03 완료, 다음은 E0-F04
- 범위: DECEL 의 MPC 전환 → G1 + proto_1b bring-up 과 QP 다중 frame CLIK → MPC catch controller
- 수학적 정식화: [mpc_multiframe_clik_formulation.md](mpc_multiframe_clik_formulation.md) — v0.4, 사용자 확정 2026-09-29. 문헌 대조는 그 문서 §6, 참고 문헌과 공개 코드는 §7 · §8
- 단일 팔 포구의 기존 구현과 그 결정 로그: [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (Epic [#537](https://github.com/hyujun/rtc-framework/issues/537)) — 이하 "v1 계획"

이 문서는 이 작업 흐름의 계획 · 결정 · 상태의 SSoT 다. v1 계획의 결정 (`D-n` 등) 과 충돌하면 그 결정을 여기서 명시적으로 개정한 경우에만 이 문서가 우선한다 (§3 의 "개정" 열).

## 1. 우선순위와 진행 판단

| Epic | 구분 | 내용 |
|---|---|---|
| E0 | 필수 (선행) | 빌드 경로 · baseline · 예측 격자 sweep · formulation 개정 |
| **E1** | **필수 · 최우선** | ur5e_p1b · iiwa7_leap 의 DECEL 계획을 MPC 로 전환 |
| **E2** | **필수** | g1_p1b 의 `demo_joint_controller` · `demo_dualarm_controller` |
| E3 | 조건부 | MPC catch controller — 게이트 G-1 통과 후 착수 여부를 결정 |

순서는 E0 → E1 → E2 → E3 이다. E2 는 G-1 의 결과와 무관하게 진행한다.

### 게이트 G-1 — MPC DECEL 이 v1 과 비슷한 성능을 내는가

E1-F06 의 A/B 시험으로 판정하고, 결과는 §8 에 기록한다. 사용자가 이 결과로 E3 착수 여부를 결정한다.

| 기준 | 내용 |
|---|---|
| 포구 성공률 | 로봇별로 closed-form 대비 **비열등**. 성공률 상승은 목표가 아니다 — DECEL 은 포구 이후에만 돈다. 성공의 정의에는 "DECEL 이 끝날 때까지 공을 쥐고 있음" 이 들어간다 |
| 한계 준수 | DECEL 구간에서 관절 위치 · 속도 · 토크 한계 위반 0 |
| 정지 구간 품질 | 관절 가속 · jerk 피크와 정지 거리를 closed-form 과 나란히 보고 |
| 계산 | solve time p99 가 `planner.budget_s` 안, fallback 발동률 보고 |
| 회귀 | 기존 supervisor 시나리오 테스트가 assertion 변경 없이 통과 |

- 판정은 paired 이진 결과의 **단측 비열등 검정**으로 한다 (formulation §6.5, [Tango1998]). McNemar 검정은 "차이 없음" 을 기각하지 못했다는 것만 말하므로 비열등의 근거가 아니다. 차이의 신뢰구간을 함께 보고한다.
- 같은 투척을 **v1 끼리 먼저 비교**해 불일치율을 잰다. 필요한 N 은 비열등 한계와 이 불일치율로 정해진다.
- 비열등 한계 (절대값인지 상대값인지 포함) 와 시행 수 N 은 **시행 전에 고정**한다 (E1-F06 spec, 사용자 컨펌).
- 예측 격자는 출하 조건 (horizon 1.0 s, 20 점) 으로 고정한다. 격자의 영향은 E0-F04 가 따로 잰다.
- 대조군은 E0-F02 가 잰 현재 구성의 값이다 (§8, MD-19): `ur5e_p1b` 287/400, `iiwa7_leap` 204/400, 같은 투척의 불일치율 0.27 · 0.24. 판정은 절대 성공률이 아니라 같은 투척의 paired 비교로 하고, 검정력 계산에 이 값을 쓴다.
- 기준을 본 뒤에 바꾸지 않는다. 미정인 값으로 판정하면 `PASS(provisional)` 로 표기한다.

## 2. 관리 방식

| 층 | 위치 | 담는 것 |
|---|---|---|
| 전체 계획 | 이 문서 | epic · feature 표, 결정 로그 (`MD-n`), 게이트 결과, 상태 |
| epic · feature 세부 | 각 에이전트의 private plan (repo 에 커밋하지 않는다). **착수할 때 만든다** — 미리 만들지 않는다 | spec, Sprint Contract, 진행 기록, handoff |
| 추적 · 인계 | GitHub project [rtc-framework — MPC · dual-arm catching](https://github.com/users/hyujun/projects/2) | epic · feature 이슈, 상태, 완료 · 결정 변경 코멘트 |

- 이슈 제목은 `[EPIC] E<n> — …`, `[FEATURE] E<n>-F<nn> — …` 이고 feature 는 epic 의 sub-issue 다.
- label: `type:epic` / `type:feature`, `area:mpc-dualarm`, 필수면 `priority:p0`, 조건부면 `conditional`, sim 전용이면 `sim-only`.
- feature 를 끝내면 이슈의 Done when 을 항목별로 갱신하고, 결정이 바뀌면 이 문서 §4 를 먼저 고친다.
- feature 착수 전에 Sprint Contract 를 제시하고 컨펌받는다 (AGENTS.md §6.5).
- 브랜치는 비슷한 feature 를 묶어 만든다. 묶음과 순서는 §6 "브랜치 계획" 에 있다.
- 수치로 판정하는 게이트는 기준과 N 을 시행 전에 고정한다.

## 3. v1 계획과의 관계

### 3.1 구조 차이

| 항목 | v1 | MPC |
|---|---|---|
| 계획기가 정하는 것 | 포구 시각 · 포구점 · 접근축 · γ profile, 포구 순간의 정적 IK 자세 | 전 구간의 관절 궤적 노드열 |
| 탐색 | vision 샘플 격자 위 1차원 시간 탐색 + 후보별 IK + γ rollout | 포구 시각 격자 × condensed QP |
| 포구 전 운동 | soft-catch DS 가 매 tick 생성 | MPC 가 계획, RT 는 보간 · 추종 |
| 포구 후 정지 | TCP 직선 등감속 | MPC 궤적의 꼬리 (관절 공간) |
| 관절 한계 | 끝점 사이 도달시간 닫힌식 (필요조건, L3 §4.3) + CLIK 의 tick 별 제약 | horizon 전체의 위치 · 속도 · 토크 제약 |
| 여유 자유도 | 포구 자세의 roll 만 | 비용으로 전 구간 분배 |
| 다중 팔 · waist · 충돌 | 범위 밖 | 대상 |

### 3.2 v1 실측이 말하는 것 (sim, v1 계획 §1a · §4.4)

| 실측 | 값 |
|---|---|
| ur5e_p1b tennis 성공률 (G8-D) | 180/200, 하한 0.8506 |
| iiwa7_leap 성공률 (G8-D2) | 상자 전체 86/200, 하한 0.363 — FAIL |
| 위치 오차 분해 (tennis) | servo 2.6 mm, 예측 79 mm |
| `ref_saturated` 발생 시행 | tennis 165/200, leap 5/200 |

- 예측 오차가 추종 오차를 압도한다.
- γ 창을 묶는 것은 포구 자세의 접근축 속도다 (D-S8-19). 대기 자세로 그 속도를 올려도 성공률은 오르지 않았다 (D-S8-20).
- 문헌도 같은 방향이다. 궤적 계획 방식을 바꿔도 성공률에 유의한 차이가 없었다는 보고와, 실패의 주원인이 예측 오차라는 보고가 있다 (formulation §6.1, [Dong2020] [Bauml2010]).
- 따라서 단일 팔에서 MPC 가 성공률을 올린다고 기대하지 않는다. MPC 의 근거는 waist + dual-arm 에서 v1 구조가 표현하지 못하는 문제 (여유 자유도 분배, 협조, 충돌) 다.
- 위 표는 v1 계획이 잰 값이다. **지금 구성의 대조군은 §8 의 E0-F02 값**이며 `ur5e_p1b` 는 위 표보다 낮다 (287/400).
- 위 값은 모두 sim 값이다. v1 의 S10 실기 식별 (servo 지연, 토크 한계) 이 바뀌면 MPC 의 튜닝과 측정도 다시 한다.

### 3.3 v1 결정과의 관계

| v1 결정 | 내용 | 이 계획에서 |
|---|---|---|
| §8 (2026-09-29) | v1 은 NLP/MPC 로 전환하지 않는다 | **개정** — MD-6 |
| §8 | 두 번째 계획기 구현이 생길 때 interface 도입 (ARCH-3) | 적용 — E3-F01 |
| §8 | 전환 시 재사용 후보는 `rtc_mpc` | 채택하지 않음 — MD-1 |
| D-16 · D-S8-18 | 유도 가속 box 는 도달시간 전용, CLIK 은 토크 기반 제약 | 적용 — MD-7 |
| C-35 | `ABORT_SAFE` 는 원인과 무관하게 관절 공간 정지 | 불변 |
| L7 G7-B | DECEL 진입 시 기준 상태 연속 | 적용 — E1-F04 |
| `supervisor.sat_ticks` | 로봇별 포화 임계 (sim 분포에서 도출) | MPC DECEL 에서 재확인 — E1-F06 |
| D-2 · D-6 · D-21 | 시간 규약 · 명령값 평가 · SeqLock 소비 규약 | 적용 |
| D-7 (E-7 결정 J) | 계획기 스레드는 `mpc_main` 슬롯 공유 | 적용 — 새 스레드 없음 |
| D-15 | vision 예측 사양은 포구 제어기가 요구하고, sim profile 은 rtc-framework 의 로봇별 파일에 설정 | 요구 사양은 적용. profile 의 위치는 **개정** — MD-18 |

## 4. 결정 로그

| ID | 결정 | 근거 | 개정 | 날짜 |
|---|---|---|---|---|
| MD-1 | MPC 수치 코어는 `rtc_controllers/catching` 에 둔다 | `rtc_mpc` 의 handler 계층은 Aligator · contact plan 형태라 dense condensed QP 와 맞지 않는다. 포구 코어와 `QPSolverWrapper` 소비자가 이미 그 자리에 있다 (v1 D-1) | v1 §8 의 재사용 후보 메모 | 2026-09-29 |
| MD-2 | QP 다중 frame CLIK 컨트롤러는 새 config key `demo_dualarm_controller` 로 추가한다 | 기존 `demo_task_controller` (DLS) 와 그 소비 로봇을 건드리지 않는다 | — | 2026-09-29 |
| MD-3 | DECEL 에서는 soft-catch DS 를 건너뛰고 MPC 궤적의 pose · twist 와 관절 기준을 CLIK 에 넣는다 | DECEL 은 이미 γ≡1 이라 DS 가 목표를 그대로 따른다. CLIK 입력 형식은 유지된다 | — | 2026-09-29 |
| MD-4 | G1 자산은 `ur5e_p1b` 와 같은 방식으로 참조한다 (`hand_description` 런타임 참조) | 기존 로봇과 같은 경로 | — | 2026-09-29 |
| MD-5 | G1 은 sim 전용이다 (`iiwa7_leap` 과 동일) | G1 용 `DeviceBackend` 가 없다 | — | 2026-09-29 |
| MD-6 | MPC 도입을 v1 의 재검토 조건 (S10 실기 신호) 을 기다리지 않고 착수한다. v1 계획기 · DECEL 은 기본값으로 남는다 | 동기는 v1 의 신호가 아니라 waist + dual-arm 확장이다. 진행 판단은 게이트 G-1 | v1 §8 · §7.3 | 2026-09-29 |
| MD-7 | MPC 의 가속 제약은 **토크 기반**이다 — 활성 관절의 토크 행을 제약으로 걸고, 유도 가속 box 는 쓰지 않는다 | 유도 box 는 실행 가속을 설명하지 못한다 (v1 S8-G). CLIK 이 이미 토크 기반 제약을 쓰므로 MPC ⊂ CLIK 를 같은 물리량으로 보장한다 | — | 2026-09-29 |
| MD-8 | 필수는 E1 · E2, E3 는 G-1 결과로 착수 여부를 정한다. E1 을 가장 먼저 한다 | 사용자 결정 | — | 2026-09-29 |
| MD-9 | 계획기는 **관절 노드만** 게시한다. 손의 pose · twist 는 RT 가 관절 기준에서 FK 로 만든다. 보간기는 두지 않는다 | 회전벡터 보간은 각속도와 회전벡터 미분의 구분이 필요하고 빠뜨리면 노드마다 각속도가 끊긴다. FK 로 만들면 손의 목표와 자세 기준이 정확히 일치한다. CLIK 입력 형식은 유지된다 | — | 2026-09-29 |
| MD-10 | 재계획 주기 (예측 메시지의 주기), MPC 노드 간격, 예측점 간격은 서로 다른 양이다. MPC 격자는 포구 시각에 고정하고, 효력 시각은 파이프라인 지연 이후의 첫 격자점이다 | 예측점을 촘촘히 해도 결정변수가 늘지 않는다. COMMITTED 뒤에도 포구 시각이 격자 위에 남는다. 효력 시각이 최대 한 노드 늦어진다 | — | 2026-09-29 |
| MD-11 | MPC 는 제동을 보장하지 않는다. 안전망은 (1) QP 가 실행 불가능하거나 예산을 넘기면 새 계획을 게시하지 않고 직전 계획을 따른다, (2) 직전 계획이 없거나 오래되면 v1 의 closed-form DECEL, (3) `ABORT_SAFE` 는 불변 | 고정 move blocking 과 재선형화된 제약 아래에서는 종단 정지 제약이 재귀적 실행 가능성을 주지 않는다 | — | 2026-09-29 |
| MD-12 | 각운동량 항은 **유지**한다. 목적은 왼팔의 운동 생성과 floating base 확장이다. 효과는 포구 성공률이 아니라 각운동량 변화율의 크기로 판정한다 | 사용자 결정. 지금은 fixed base 이지만 floating base 로 확장한다. 고정 베이스 강체 sim 에서 이 항은 손의 정확도에 영향을 주지 않는다 | — | 2026-09-29 |
| MD-13 | 토크는 1차까지 선형화한다. 토크 · 자기충돌 행에는 slack 을 두고, 위치 · 속도 한계, 종단 등식, trust region, 공–왼팔 행은 hard 다. 동역학은 손을 기준 자세로 잠근 축소 모델로 계산한다 | 계수를 고정한 근사는 문헌에서 검증된 바가 없고 해석적 미분은 계산이 싸다. 재선형화로 움직이는 행만 slack 으로 완화한다 | MD-7 의 귀결 | 2026-09-29 |
| MD-14 | CLIK 에 제동 거리 기반 속도 한계 (opt-in), 충돌 damper (선택), 오차 되먹임 상한을 둔다. 앞의 둘은 기본 꺼짐이다 | 위치 box 와 가속 · 토크 한계는 한 tick 에서 양립하지 않을 수 있다. 기본값을 끄는 것은 기존 로봇의 golden 회귀를 지키기 위해서다 | — | 2026-09-29 |
| MD-15 | 예측 격자 sweep 을 한다. horizon 은 0.75 s 와 1.0 s, 조건은 8개 (formulation §1.7). v1 계획기로 먼저 돌려 기준선을 만든다 | 사용자 결정. horizon 을 나누는 이유는 추정기 (EKF) 의 정확도다. 조건은 점 수 상한 40, horizon 이 step 의 배수, 지평 요구 0.51 s 에서 나왔다. horizon 0.5 s 는 지평 요구에 못 미쳐 쓰지 않는다 | — | 2026-09-29 |
| MD-16 | QP solver 의 기본은 ProxQP dense 다. 계획기 한 주기의 p99 를 실측하고, 구조를 쓰는 solver 와의 오프라인 비교는 선택 사항이다 | 이 문제 크기의 warm start 된 MPC QP 를 잰 공개 benchmark 가 없다 | — | 2026-09-29 |
| MD-17 | rtc-framework · `hand_description` · ball_perception 은 서로를 모른다 — 빌드 스크립트와 manifest 에 서로의 이름을 넣지 않는다. `rtc_ws` 는 rtc-framework 와 `hand_description` 을 ws root 의 `colcon build` 로 함께 빌드하고, ball_perception 은 별도 workspace 의 sim 용 고정 사본에서 `ball_perception_sim` 과 그 upstream 만 빌드한다 | 사용자 결정. 세 저장소는 독립 project 다. ball_perception 의 개발용 workspace 는 코드가 계속 바뀌므로 sim 이 쓰는 사본을 따로 둔다 | — | 2026-09-29 |
| MD-18 | sim 추정기의 profile (`ball_perception.sim_profile`) 은 ball_perception 저장소가 소유하고 거기서 읽는다. rtc-framework 의 로봇별 사본은 읽지 않는다 | 사용자 결정. rtc-framework 는 제어 PC, ball_perception 은 vision PC 에서 돈다 — 서로의 파일을 볼 수 없다 (MD-17 의 귀결) | — | 2026-09-30 |
| MD-19 | G-1 의 대조군은 E0-F02 가 잰 **현재 구성의 값**이다 (§8). v1 계획의 값 (`ur5e_p1b` 180/200) 과의 차이는 조사하지 않는다 | 사용자 결정. G-1 은 같은 구성에서 같은 투척을 짝지어 비교하므로 대조군은 MPC arm 과 같은 구성이어야 한다 | §1 의 "검정력 계산에 이 baseline 을 쓴다" 가 가리키는 값 | 2026-09-30 |

MD-7 의 귀결: 토크 행은 직전 해에서의 역동역학 값과 그 미분으로 선형화한다 (MD-13). 그래서 단일 팔 정지 구간 문제도 계획기 스레드에서 동역학 모델을 평가하고, 주기마다 선형화를 다시 한다.

미결 — 해당 feature 의 spec 에서 정한다:

- MD-18 의 실행 (E0-F04 에 묶는다): rtc-framework 의 로봇별 profile 사본의 제거, ball_perception 쪽 profile 의 위치와 schema minor. 사본을 고정하던 테스트는 E0-F01 에서 삭제했다 (E-6, 사용자 승인 2026-09-30)
- E0-F04: sweep 의 시행 수와 판정 기준, 조건별 설정을 둘 위치 (MD-18 에 따라 ball_perception 쪽)
- E1-F03: 정지 구간의 노드 수와 간격, 포구 전 초기 상태의 예측 방법
- E1-F06: 비열등 한계와 N, MPC DECEL 을 기본값으로 바꿀지
- E3-F01: MPC 계획기와 v1 L3 계획기의 관계 (대체 · 병행)
- E3-F05: 기존 `DemoCatchingController` 확장과 새 컨트롤러 중 선택

## 5. 출발점 (2026-09-29 코드 대조)

| 영역 | 확인한 사실 | 계획에 주는 영향 |
|---|---|---|
| DECEL | `rtc_controllers` `catching/decel_target.hpp` 의 `EvaluateDecelTarget` 이 TCP 직선 등감속 목표를 만들고, soft-catch DS → CLIK 을 거쳐 RT tick 안에서 매번 1-step 으로 푼다 | horizon 도, 관절 가속 · jerk 최적화도 없다. E1 이 대체한다 |
| ABORT_SAFE | 같은 헤더의 `JointSpaceDecelStep` (QP 독립) | 최종 안전망으로 그대로 둔다 |
| 계획기 스레드 | `CatchingPlannerThread` 가 `mpc_main` 슬롯을 공유하고 `PlanSnapshot` 을 SeqLock 으로 RT 에 넘긴다 | 새 스레드 없이 MPC 를 얹는다 |
| QP CLIK | `rtc_tsid` `ClikReferenceGenerator` 는 Cartesian frame 과제 1개 + arm / hand posture 2군 구조다. `kDynamic` 토크 제약은 있다 | 두 번째 frame 과제는 미지원 — E2-F04 에서 일반화한다 |
| `DemoTaskController` | QP 가 아니라 DLS + null-space (`DifferentialIk`) | 고치지 않고 새 컨트롤러를 추가한다 (MD-2) |
| 충돌 거리 | capsule · 자기충돌 코어를 찾지 못했다 (grep 기준) | E3-F03 은 신규 capability 다. 거리 계산 library 는 formulation §8 |
| 관절 한계 | `ClikReferenceGenerator` 는 1-step 위치 box 를 쓰고 충돌 시 `bound_conflict` 와 실패 (직전 값 유지) 로 처리한다. `rtc_tsid` 에 가속 수준의 viability 제약 `JointLimitConstraint` 가 있다 | 제동 거리 한계는 속도 수준으로 옮겨 쓴다 (MD-14, E2-F04) |
| centroidal | `rtc_tsid` momentum task 에 있다. 설치된 Pinocchio 에 필요한 함수가 있고 고정 베이스 모델에서도 정의된다. RT 모델 핸들 노출은 미확인 | E3-F02 에서 확인한다 |
| G1 모델 | waist 3 + 양팔 7×2 로 MPC 활성 관절 n=17, 오른손 proto_1b 구동 10, 다리는 fixed. 양팔 뿌리는 모두 `torso_link` | formulation v0.3 에 반영했다. §0.3 전제는 몸통에 고정된 물체끼리의 충돌 쌍에만 성립한다 |
| G1 파일 | flat `.urdf` 는 없고 `.urdf.xacro` 만 있다. closure sidecar 와 fixed-base MJCF 는 있다 | `rtc_urdf_bridge` 가 xacro 를 직접 읽는다 (G1 파일로는 미검증 — E2-F01) |
| 오른손 | proto_1b 는 폐쇄 체인 (수동 관절 10, loop closure 5) 이고 왼손 형상 · `l_*` 이름으로 오른 손목에 장착돼 있다 | 장착 자세는 실기 미검증이다 |
| 용량 | `kMaxPlanNv`, `kMaxDeviceChannels` 모두 G1 관절 수를 담는다 | 상수 변경 불필요 |
| 예측 격자 | 예측 메시지는 sim 실측 30 Hz 로 온다. 제약 (`kCap`, `io.horizon_min`) 과 두 저장소의 현재 설정값은 formulation §1.7 의 표에 있다 | sweep 조건의 근거 (MD-15). profile 은 ball_perception 쪽으로 옮기고 (MD-18) 그쪽 JSON 을 지평 요구에 맞춰 갱신한다 (E0-F04) |
| armature | G1 URDF 에는 없고 MJCF 에만 있다 | 로봇 config 에서 읽는다 (MD-13, E1-F01 · E2-F02) |
| GUI | 컨트롤러는 런타임 발견, 로봇은 `demo_gui/discovery.py` 의 `RobotProfile` 정적 정의 | G1 profile 을 추가한다 |
| plot | 파일명 기반 log type → plotter registry | 컬럼 추가와 plotter 확장 |

### 빌드

- ws root 에 `build/` · `install/` · `log/` 가 없었다. E0-F01 이 첫 빌드를 했고, 그때의 `colcon test` 기준은 [#624](https://github.com/hyujun/rtc-framework/issues/624) 에 있다.
- `build.sh` 는 고정 목록만 `--packages-select` 로 빌드하고, 그 목록에 `hand_description` 과 `ball_perception*` 이 없다.
- `integrated_bringup` 의 `ur5e_p1b` config 는 `package://hand_description/...` 을 런타임에 참조하지만 `package.xml` 에는 선언이 없다.
- 두 패키지의 빌드 절차는 E0-F01 에서 확정했다 (MD-17). `build.sh` 와 `package.xml` 은 고치지 않는다. 절차는 [repo_scripts/README.md](../../repo_scripts/README.md) 에 있다.

## 6. Epic · Feature

### E0. 기반 정비 — [#620](https://github.com/hyujun/rtc-framework/issues/620) · 필수

게이트: 두 sim 이 기동하고 baseline 수치, 예측 격자 sweep 의 v1 기준선, formulation v0.4 가 문서에 있다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E0-F01 | [#624](https://github.com/hyujun/rtc-framework/issues/624) | 빌드 경로 확정 (rtc-framework + hand_description + ball_perception) | — | 완료 (2026-09-30) |
| E0-F02 | [#625](https://github.com/hyujun/rtc-framework/issues/625) | closed-form DECEL baseline 측정 (ur5e_p1b · iiwa7_leap) | E0-F01 | 완료 (2026-09-30) — §8 |
| E0-F03 | [#626](https://github.com/hyujun/rtc-framework/issues/626) | formulation v0.4 — G1 기구 · 단일 팔 환원형 · 토크 기반 제약 · 문헌 대조 | — | 완료 (2026-09-29) — v0.4 확정 |
| E0-F04 | [#647](https://github.com/hyujun/rtc-framework/issues/647) | 예측 격자 sweep — v1 계획기 기준선 (horizon 0.75 s · 1.0 s) | E0-F01 | 대기 |

### E1. Decel MPC — [#621](https://github.com/hyujun/rtc-framework/issues/621) · 필수 · 최우선

대상 로봇: ur5e_p1b · iiwa7_leap. 게이트: **G-1** (§1).

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E1-F01 | [#627](https://github.com/hyujun/rtc-framework/issues/627) | jerk 입력 condensed QP 코어 (토크 제약 행 · slack) | E0-F03 | 대기 |
| E1-F02 | [#628](https://github.com/hyujun/rtc-framework/issues/628) | `PlanSnapshot` 관절 노드 payload + RT 샘플러 (관절 기준에서 FK) | E1-F01 | 대기 |
| E1-F03 | [#629](https://github.com/hyujun/rtc-framework/issues/629) | 계획기 스레드 통합 — 정지 구간 선계산 | E1-F02 | 대기 |
| E1-F04 | [#630](https://github.com/hyujun/rtc-framework/issues/630) | L7 DECEL 전환 — MPC 궤적 추종 + closed-form fallback | E1-F03 | 대기 |
| E1-F05 | [#631](https://github.com/hyujun/rtc-framework/issues/631) | 로그 · plot_rtc_log · demo_controller_gui | E1-F04 | 대기 |
| E1-F06 | [#632](https://github.com/hyujun/rtc-framework/issues/632) | A/B 성능 시험 — 게이트 G-1 판정 | E0-F02, E1-F05 | 대기 |

### E2. G1 + proto_1b bring-up — [#622](https://github.com/hyujun/rtc-framework/issues/622) · 필수

sim 전용. 게이트: G1 sim 에서 두 컨트롤러가 GUI 로 구동되고 formulation §4 sanity check 가 통과한다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E2-F01 | [#633](https://github.com/hyujun/rtc-framework/issues/633) | G1 로봇 자산 — 모델 로드 · frame · MJCF scene | E0-F01 | 대기 |
| E2-F02 | [#634](https://github.com/hyujun/rtc-framework/issues/634) | config · launch — `config/g1_p1b/` · `sim_g1_p1b.launch.py` | E2-F01 | 대기 |
| E2-F03 | [#635](https://github.com/hyujun/rtc-framework/issues/635) | demo_joint_controller — G1 다중 device group 지원 | E2-F02 | 대기 |
| E2-F04 | [#636](https://github.com/hyujun/rtc-framework/issues/636) | CLIK 다중 frame 일반화 (`rtc_tsid`) | E0-F03 | 대기 |
| E2-F05 | [#637](https://github.com/hyujun/rtc-framework/issues/637) | demo_dualarm_controller — QP 다중 frame CLIK 바인딩 | E2-F03, E2-F04 | 대기 |
| E2-F06 | [#638](https://github.com/hyujun/rtc-framework/issues/638) | demo_controller_gui — G1 profile · 다중 frame 목표 | E2-F05 | 대기 |
| E2-F07 | [#639](https://github.com/hyujun/rtc-framework/issues/639) | plot_rtc_log — 다중 device group · frame 별 task error | E2-F05 | 대기 |

### E3. MPC catch controller — [#623](https://github.com/hyujun/rtc-framework/issues/623) · 조건부

착수 조건: 게이트 G-1 통과 + 사용자 결정. sim 전용. 게이트: G1 sim 에서 포구 시행이 돌고 성공률 · solve time p99 가 보고된다. 기존 두 로봇 회귀 없음.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E3-F01 | [#640](https://github.com/hyujun/rtc-framework/issues/640) | MPC 포구 항 (포구 구간 비용 · 후보 열거) · 계획기 interface 도입 | E1-F06 (G-1) | 보류 |
| E3-F02 | [#641](https://github.com/hyujun/rtc-framework/issues/641) | 전신 항 — 토크 행 · 왼팔 rest · 각운동량 | E3-F01, E2-F01 | 보류 |
| E3-F03 | [#642](https://github.com/hyujun/rtc-framework/issues/642) | 충돌 제약 — capsule 모델 · 자기충돌 · 공–왼팔 거리 | E3-F02 | 보류 |
| E3-F04 | [#643](https://github.com/hyujun/rtc-framework/issues/643) | MPC ↔ CLIK 계약 — 관절 노드 payload · RT FK · 한계 여유 | E3-F01, E2-F04 | 보류 |
| E3-F05 | [#644](https://github.com/hyujun/rtc-framework/issues/644) | catch controller 통합 — supervisor · 손 시퀀서 · vision sim profile | E3-F03, E3-F04, E2-F05 | 보류 |
| E3-F06 | [#645](https://github.com/hyujun/rtc-framework/issues/645) | 로그 · plot_rtc_log · demo_controller_gui | E3-F05 | 보류 |
| E3-F07 | [#646](https://github.com/hyujun/rtc-framework/issues/646) | 평가 — G1 sim 포구 시행 · 예측 격자 sweep · 기존 로봇 회귀 | E3-F06, E0-F04 | 보류 |

각 feature 의 범위와 Done when 은 이슈 본문에 있다.

### 브랜치 계획

feature 24개를 브랜치 17개로 묶는다. 브랜치 하나가 PR 하나다.

**묶는 기준.**

| 기준 | 내용 |
|---|---|
| 묶는다 | 같은 패키지 · 같은 파일을 고치고, 함께 있어야 동작을 확인할 수 있는 feature |
| 묶는다 | 같은 종류의 도구 작업 (로그 · plot · GUI) 과 같은 측정 도구를 쓰는 평가 |
| 따로 둔다 | escalation 이 Critical 인 feature (E-8) — security review 의 범위를 좁게 유지한다 |
| 따로 둔다 | public API 를 바꾸거나 abstract interface 를 도입하는 feature — 기능 동등성을 그 PR 만으로 판정한다 |
| 따로 둔다 | 신규 수치 코어 — code review 의 단위다 |
| 따로 둔다 | 게이트 판정 — 판정 결과와 그에 따른 결정이 구현 PR 에 섞이지 않게 한다 |

**규칙.**

- 브랜치 이름은 `type/slug` 다 (기존 관례). `type` 은 Conventional Commits 의 type 이다.
- 브랜치는 선행 브랜치가 `main` 에 merge 된 뒤 `main` 에서 만든다. PR 을 쌓지 않는다.
- PR 본문에 닫는 feature 이슈를 모두 적는다. 묶인 feature 중 일부만 끝났으면 그 이슈만 닫는다.
- 한 브랜치 안에서 feature 마다 커밋을 나눈다. 묶인 feature 도 Sprint Contract 와 Done when 은 feature 별로 판정한다.
- 묶은 feature 가 착수 뒤에 위 "따로 둔다" 기준에 걸리면 브랜치를 나눈다 (아래 표의 "나누는 조건").
- 브랜치 이름은 착수 때 확정한다. 아래 이름은 계획값이다.

**E0**

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `docs/mpc-dualarm-plan` | E0-F03 | 문서 4개 (계획, formulation, README, v1 계획 개정). 가장 먼저 올린다 — 이슈의 문서 링크가 이 merge 로 살아난다 |
| `chore/ws-first-build-path` | E0-F01 | 빌드 절차 문서 (MD-17 — 빌드 스크립트와 `package.xml` 은 고치지 않는다). 다른 feature 의 선행이라 단독으로 빨리 닫는다 |
| `feat/catching-baseline-grid-sweep` | E0-F02, E0-F04 | 둘 다 `catching_sim_trials` 로 시행을 모으고 `rtc_tools` 의 분석을 확장한다. 같은 시행 도구와 같은 host 조건을 쓴다. **나누는 조건**: sweep 의 조건별 설정이 출하 config 를 건드리게 되면 E0-F04 를 분리한다 |

E0-F04 의 ball_perception 쪽 JSON 갱신은 그 저장소 (hyujun/ball_perception) 의 브랜치와 PR 로 따로 한다.

**E1**

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `feat/catching-decel-mpc-core` | E1-F01 | 신규 수치 코어. code review 단위 |
| `feat/catching-decel-mpc-plan-path` | E1-F02, E1-F03 | payload · RT 샘플러와 계획기 스레드 통합. 계획기가 게시하고 RT 가 읽는 한 경로의 양 끝이라 함께 있어야 연속성을 시험할 수 있다. **나누는 조건**: 새 스레드가 필요해지면 (E-7) E1-F03 을 분리한다 |
| `feat/catching-decel-mpc-l7` | E1-F04 | E-8 (Critical). `[CONCERN]` 컨펌과 security review 의 범위를 이 PR 로 한정한다 |
| `feat/catching-decel-mpc-tooling` | E1-F05 | 로그 · plot · GUI |
| `docs/catching-decel-mpc-g1` | E1-F06 | 게이트 G-1 의 판정과 결과 기록. 기본값 변경이 결정되면 그 변경은 별도 브랜치다 |

**E2**

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `feat/g1-p1b-bringup` | E2-F01, E2-F02, E2-F03 | 자산, config · launch, `demo_joint_controller` 의 G1 구동. 셋이 모여야 G1 sim 에서 관절 추종을 확인할 수 있다. **나누는 조건**: E2-F03 이 `DemoJointController` 의 일반화 (공용 코드 변경) 를 요구하면 분리한다 — 기존 로봇의 회귀를 그 PR 만으로 판정한다 |
| `feat/tsid-clik-multiframe` | E2-F04 | `rtc_tsid` public API 변경. 기존 소비자 둘의 기능 동등성이 성공 기준이다 |
| `feat/demo-dualarm-controller` | E2-F05 | 신규 controller. Sprint Contract = spec |
| `feat/g1-dualarm-tooling` | E2-F06, E2-F07 | GUI 와 plot. 둘 다 `demo_dualarm_controller` 의 출력을 소비한다 |

`feat/tsid-clik-multiframe` 은 선행이 E0-F03 뿐이라 E1 과 병행할 수 있다.

**E3** (조건부 — 게이트 G-1 뒤)

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `feat/catching-mpc-catch-terms` | E3-F01 | 계획기 interface 도입 (ARCH-3) 과 포구 항. **나누는 조건**: interface 도입만으로 v1 계획기의 diff 가 크면 interface 를 먼저 분리한다 |
| `feat/catching-mpc-wholebody` | E3-F02, E3-F04 | 전신 항과 MPC ↔ CLIK 계약. 둘 다 활성 관절을 전체로 넓히는 작업이고, 계약의 sanity check 가 전신 해를 입력으로 쓴다 |
| `feat/collision-capsule-core` | E3-F03 | 신규 수치 코어. code review 단위 |
| `feat/g1-catch-controller` | E3-F05 | supervisor · 손 시퀀서 통합. E-STOP 경로를 건드리면 E-8 |
| `feat/g1-catch-tooling-eval` | E3-F06, E3-F07 | 로그 · plot · GUI 와 평가. 평가가 새 로그 컬럼을 쓴다. **나누는 조건**: 평가 결과가 설계 결정을 바꾸면 결과 기록을 `docs/` 브랜치로 분리한다 |

**순서.**

| 단계 | 브랜치 | 병행 가능 |
|---|---|---|
| 1 | `docs/mpc-dualarm-plan` | — |
| 2 | `chore/ws-first-build-path` | — |
| 3 | `feat/catching-baseline-grid-sweep` | `feat/catching-decel-mpc-core`, `feat/tsid-clik-multiframe` |
| 4 | `feat/catching-decel-mpc-plan-path` → `-l7` → `-tooling` → `docs/catching-decel-mpc-g1` | `feat/g1-p1b-bringup` |
| 5 | `feat/demo-dualarm-controller` → `feat/g1-dualarm-tooling` | — |
| 6 | E3 의 다섯 브랜치 (G-1 통과와 사용자 결정 뒤) | — |

병행은 서로 다른 패키지를 고치는 브랜치끼리만 한다. E1 이 최우선이므로 (MD-8), 병행할 여력이 없으면 단계 4 의 E1 브랜치를 먼저 한다.

## 7. 게이트 · escalation

| Feature | 사유 | 효력 |
|---|---|---|
| E1-F04 | L7 전이 동작 변경 (E-8) | Critical — 착수 전 `[CONCERN]` 과 컨펌, 완료 후 security review |
| E3-F05 | E-STOP 경로를 건드리면 E-8 | Critical |
| E1-F03 | 새 스레드가 필요해지면 E-7 | Critical |
| E2-F04 | `rtc_tsid` public API 변경, 기존 소비자 둘 | code review, 기능 동등성이 성공 기준 |
| E3-F01 | 계획기 interface 신설 (ARCH-3) | code review |
| E1-F01, E3-F03 | 신규 수치 코어 (100+ 줄) | code review |
| E2-F05 | 신규 controller | Sprint Contract = spec |
| E0-F04 | ball_perception 은 별도 저장소다 | 그쪽 변경은 그 저장소의 절차를 따른다. profile 은 그쪽이 소유한다 (MD-18) — 컨트롤러 YAML 과의 정합을 보는 자동 검사는 없으므로 양쪽 값을 함께 확인한다 |

공통 규칙:

- 기존 test assertion 은 약화하지 않는다 (PROC-6).
- RT 경로 (`Compute`, 샘플러, CLIK) 는 할당 0 을 게이트로 확인한다.
- E1 에서 v1 의 closed-form DECEL 은 기본값으로 남고, MPC 는 YAML 로 켠다.
- 실험 overlay 와 원자료는 repo 밖에 둔다.

## 8. 게이트 결과

(G-1 과 epic 게이트의 결과를 완료 시 기록한다.)

### E0-F02 — closed-form DECEL baseline (2026-09-30, [#625](https://github.com/hyujun/rtc-framework/issues/625))

v1 의 closed-form DECEL 을 현재 구성으로 잰 값이다. E1-F06 의 A/B 가 쓰는 대조군이다.

**측정 조건.**

| 항목 | 값 |
|---|---|
| 저장소 | rtc-framework `9b9dfaa1`, ball_perception `dd0f390` (sim 용 고정 사본) |
| 추정기 profile | rtc-framework 의 로봇별 사본 (두 로봇 바이트 동일), sha256 `fb61cbfc…fdd6aa`. 지평 1.0 s · 20 점 |
| overlay | 두 로봇 모두 출하 `catch_lead_on` (`T_arm` 0.05, p1b `T_freeze` 0.37 · leap 0.19) |
| 컨트롤러 | `supervisor.decel.a_dec` 10, `reference.omega` 10, `reference.a_max` p1b 30 · leap 35, `control.dt` 2 ms |
| 투척 | `--dist s35b`, tennis, seed p1b 601–604 · leap 701–704, unit 당 50 발. 같은 seed 를 두 번 (복제 a · b) |
| 시행 수 | 로봇당 400 발 (200 × 2), 무효 0 |
| host | 개발 PC, 기동 시 load average 1.9–3.8. unit 은 `--host-watch abort` (RTF 하한 0.95) |
| RTF | 판정에 쓴 unit 의 시행별 최솟값 0.980–0.993 |

unit 재실행 (v1 D-S8-17 규칙, 같은 seed): `p1b_601_a` (RTF 0.943 시행 — 원본 35/50, 재실행 37/50), `p1b_603_b` (host-watch 중단 1회), `leap_704_b` (추정기 미활성 1회, host-watch 중단 1회). 중단 시각에 host 에 빌드 · 테스트 프로세스는 없었다 — 원인 미확인.

**성공률 (truth 기반, v1 과 같은 정의 — HOLD 끝부터 release 까지 공이 손에 있음).**

| 로봇 | 복제 a | 복제 b | 합산 | Wilson 95 % | v1 계획의 값 |
|---|---|---|---|---|---|
| ur5e_p1b | 140/200 | 147/200 | 287/400 (0.718) | [0.671, 0.759] | 180/200 (0.900) |
| iiwa7_leap | 100/200 | 104/200 | 204/400 (0.510) | [0.461, 0.559] | 86/200 (0.430) |

- 무효가 없어 ITT 구간은 위와 같다.
- `ur5e_p1b` 는 v1 계획의 값보다 낮다. 부하가 없는 unit 도 31–39/50 이다. 구성의 차이는 ball_perception 의 개정 (`dc17ae8` → `dd0f390`) 과 그 뒤의 rtc-framework 변경이다. 원인은 조사하지 않고 이 값을 대조군으로 쓴다 (MD-19).

**같은 투척의 복제 불일치 (a 대 b, 로봇당 200 쌍).**

| 로봇 | 둘 다 성공 | a 만 | b 만 | 둘 다 실패 | 불일치율 ψ | Wilson 95 % | McNemar p |
|---|---|---|---|---|---|---|---|
| ur5e_p1b | 117 | 23 | 30 | 30 | 0.265 | [0.209, 0.330] | 0.41 |
| iiwa7_leap | 78 | 22 | 26 | 74 | 0.240 | [0.186, 0.304] | 0.67 |

paired 단측 비열등 검정에 필요한 쌍 수 (α 0.025 단측, 검정력 0.8, 참 차이 0, 정규 근사):

| 로봇 | 한계 0.05 (절대) | 한계 0.10 (절대) |
|---|---|---|
| ur5e_p1b | 832 (ψ 상한에서 1037) | 208 (260) |
| iiwa7_leap | 754 (954) | 189 (239) |

sim 은 같은 투척을 재현하지 않는다 — 네 번에 한 번은 결과가 바뀐다. E1-F06 의 N 은 이 표에서 정한다.

**DECEL 지표 (시행별 값의 p50 / p95 / max).** 창은 첫 DECEL tick 부터 측정 catch frame 속도가 0.02 m/s 아래로 0.05 s 머무는 첫 tick 까지다. 미분은 5 tick 평균이다. 정의와 도구: [rtc_tools README](../../rtc_tools/README.md) `catching_decel`.

| 지표 | ur5e_p1b (400 시행) | iiwa7_leap (377 시행) |
|---|---|---|
| 관절 가속 피크, 명령 [rad/s²] | 25.4 / 32.0 / 117.3 | 9.2 / 9.2 / 31.2 |
| 관절 가속 피크, 측정 [rad/s²] | 14.9 / 17.7 / 93.0 | 9.5 / 10.7 / 89.9 |
| jerk 피크, 명령 [rad/s³] | 1579 / 2087 / 20079 | 1564 / 1564 / 5785 |
| jerk 피크, 측정 [rad/s³] | 561 / 759 / 7043 | 470 / 778 / 7222 |
| 정지 거리 (손의 변위) [mm] | 332.9 / 417.6 / 475.0 | 80.1 / 161.0 / 275.1 |
| 닫힌식 정지 거리 (참고) [mm] | 240.6 / 309.3 / 358.5 | 42.4 / 94.7 / 158.4 |
| 정지 시간 [s] | 0.348 / 0.394 / 0.666 | 0.324 / 0.546 / 0.682 |
| DECEL mode 길이 (참고) [s] | 0.222 / 0.252 / 0.270 | 0.096 / 0.140 / 0.182 |
| 진입 속력, 기준 · 측정 손 [m/s] (p50) | 2.19 · 1.77 | 0.92 · 0.23 |
| 위치 한계까지의 최소 여유 [rad] | 1.453 | 0.128 |
| 속도 비 \|q̇\| / q̇_max | 0.699 / 0.896 / 0.983 | 0.462 / 0.750 / 0.979 |
| 토크 비 \|τ\| / τ_max | 0.613 / 0.740 / 1.142 | 0.429 / 0.468 / 0.906 |
| 한계 위반 시행 (위치 · 속도 · 토크) | 0 · 0 · 1 | 0 · 0 · 0 |
| RETREAT 전에 손이 정지한 시행 | 399 / 400 | 372 / 377 |

- `iiwa7_leap` 의 23 시행은 DECEL 에 들어가지 않았다 (abort 등). 성공률의 분모에는 실패로 남는다.
- 팔은 DECEL mode 가 끝난 뒤에도 0.1–0.4 s 더 움직인다. 실제 정지 거리는 닫힌식보다 p50 기준 p1b 1.4 배, leap 1.9 배다. MPC 와의 비교는 mode 길이가 아니라 위 창으로 한다.
- 최댓값은 소수의 시행이 만든다. 측정 가속 피크가 25 rad/s² 를 넘는 시행은 로봇마다 2 개이고 (99 백분위 p1b 19.5 · leap 11.6), 대부분 손이 RETREAT 전에 정지하지 못한 시행이다. `ur5e_p1b` 의 토크 위반 1 건 (`p1b_603_a` 30번, 비 1.14) 도 그중 하나다. 원인은 조사하지 않았다.
- 성공 · 실패 시행의 지표 차이는 작다 (측정 가속 p50 p1b 15.2 대 14.2, leap 9.8 대 9.2).
- `iiwa7_leap` 은 HOLD 동안 관절이 계속 움직인다 — 관절 속도 기준 (0.01 rad/s) 으로는 377 중 2 시행만 정지다. 그래서 정지는 손의 속도로 판정한다.

원자료와 실험 도구는 repo 밖에 있다. 값은 모두 sim 값이며 v1 의 S10 실기 식별이 바뀌면 다시 잰다.
