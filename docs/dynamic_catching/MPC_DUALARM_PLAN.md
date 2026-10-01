# MPC · dual-arm catching — 구현 계획

- 개정: r18 (2026-10-01) — 이력은 §9. 최초 작성 2026-09-29
- 상태: **E0 완료**, E1 진행 중. 완료: E1-F01 – F03 · E1-F07. E1-F04 는 구현이 머지됐고 sim 진입 항목이 남아 이슈가 열려 있다 (E1-F09 에서 닫는다). 다음은 E1-F08 계획기 ([#661](https://github.com/hyujun/rtc-framework/issues/661)) → F09 L7 ([#662](https://github.com/hyujun/rtc-framework/issues/662)) → F05 → F10 튜닝 ([#663](https://github.com/hyujun/rtc-framework/issues/663)) → F06 (G-1). feature 별 상태는 §6
- 범위: 단일 팔 MPC (ur5e_p1b · iiwa7_leap, APPROACH–정지) → G1 + proto_1b bring-up 과 QP 다중 frame CLIK → 같은 MPC 에 dual arm · waist 항 추가 (g1_p1b)
- 수학적 정식화: [mpc_multiframe_clik_formulation.md](mpc_multiframe_clik_formulation.md) — 구현 기준은 v0.5 (단일 팔 구성, 구현 반영 v0.5b) 이고 v0.6 ($t_c$ 를 결정변수로) 은 검토 중이다. 판의 상태는 그 문서의 개정 표가 갖는다. 문헌 대조는 그 문서 §6, 참고 문헌과 공개 코드는 §7 · §8
- 단일 팔 포구의 기존 구현과 그 결정 로그: [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (Epic [#537](https://github.com/hyujun/rtc-framework/issues/537)) — 이하 "v1 계획"

이 문서는 이 작업 흐름의 계획 · 결정 · 상태의 SSoT 다. v1 계획의 결정 (`D-n` 등) 과 충돌하면 그 결정을 여기서 명시적으로 개정한 경우에만 이 문서가 우선한다 (§3 의 "개정" 열).

**설계와 시험 순서 (MD-46 · MD-47).** MPC 는 waist + dual arm (G1) 용으로 설계한다 (formulation §1.3). ur5e_p1b · iiwa7_leap 에서는 같은 MPC 에서 dual arm · waist 전용 항과 제약만 빼고 먼저 시험하고 (E1), g1_p1b 가 준비되면 (E2) 같은 코어에 그 항을 더한다 (E3). closed_form 과 mpc 는 입력 (추정기의 공 미래 궤적) 과 출력 (CLIK 입력) 이 같은 두 planner 다 (§3.1).

## 1. 우선순위와 진행 판단

| Epic | 구분 | 내용 |
|---|---|---|
| E0 | 필수 (선행) | 빌드 경로 · baseline · 예측 격자 sweep · formulation 개정 |
| **E1** | **필수 · 최우선** | 단일 팔 MPC — ur5e_p1b · iiwa7_leap 의 팔 기준을 APPROACH 부터 정지까지 MPC 로 (§1.3 에서 dual arm · waist 항을 뺀 구성) |
| **E2** | **필수** | g1_p1b 의 `demo_joint_controller` · `demo_dualarm_controller` |
| E3 | 필수 (E2 뒤) | 같은 MPC 에 dual arm · waist 항 추가 — g1_p1b (MD-47) |

순서는 E0 → E1 → E2 → E3 이다. E2 는 E1 과 병행할 수 있다 (§6 순서 표). E3 는 E2 (g1_p1b 준비) 와 E1-F07 (코어 — 완료) 이 끝나면 착수한다 — G-1 은 E3 의 착수 조건이 아니다 (MD-47).

### 게이트 G-1 — 단일 팔 mpc planner 가 closed_form planner 와 비슷한 성능을 내는가

E1-F06 의 A/B 시험으로 판정하고, 결과는 §8 에 기록한다. 두 arm 은 planner 만 다르다 — 추정기 · supervisor · 손 시퀀서 · CLIK 은 같다 (§3.1). 판정 전에 E1-F10 이 mpc planner 를 튜닝한다 (MD-50). 사용자는 이 결과로 `mpc` 를 기본값으로 바꿀지 정한다. E3 의 착수 조건은 아니다 (MD-47).

| 기준 | 내용 |
|---|---|
| 포구 성공률 | 로봇별로 closed_form 대비 **비열등** — 목표는 성공률이 크게 다르지 않은 것이다 (MD-47). mpc 는 포구 전 궤적도 만들므로 (MD-45) 성공률이 직접 영향을 받는다. 성공의 정의에는 "DECEL 이 끝날 때까지 공을 쥐고 있음" 이 들어간다 |
| 한계 준수 | APPROACH 부터 DECEL 끝까지 관절 위치 · 속도 · 토크 한계 위반 0 |
| 궤적 품질 | 관절 가속 · jerk 피크, 정지 거리, $t_c$ 의 손 위치 · 접근축 · 상대속도 오차를 closed_form 과 나란히 보고 |
| 계산 | solve time p99 가 `planner.budget_s` 안, fallback 발동률 보고 (`mpc` 에는 fallback 이 없다, MD-44 — 따를 구간이 없어 abort 하거나 APPROACH 에 들어가지 못한 비율을 보고한다) |
| 회귀 | 기존 supervisor 시나리오 테스트가 assertion 변경 없이 통과 |

- 판정은 paired 이진 결과의 **단측 비열등 검정**으로 한다 (formulation §6.5, [Tango1998]). McNemar 검정은 "차이 없음" 을 기각하지 못했다는 것만 말하므로 비열등의 근거가 아니다. 차이의 신뢰구간을 함께 보고한다.
- 같은 투척을 **v1 끼리 먼저 비교**해 불일치율을 잰다. 필요한 N 은 비열등 한계와 이 불일치율로 정해진다.
- 비열등 한계 (절대값인지 상대값인지 포함) 와 시행 수 N 은 **튜닝 전에 고정**한다 (E1-F10 spec, 사용자 컨펌, MD-50).
- 튜닝 (E1-F10) 은 G-1 과 다른 seed 로 한다. G-1 의 투척으로 튜닝하지 않는다 (MD-50).
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

**비교 단위는 planner 다 (MD-46).** closed_form 과 mpc 는 같은 입력 (추정기의 공 미래 궤적 + 공분산) 을 받아 같은 자리의 출력 (CLIK 입력 — task pose · twist ff · 접근축, null space 자세 목표) 을 내는 두 planner 다. 추정기, supervisor FSM (commit 시각 $T_{freeze}$), 손 시퀀서 ($t_{cmd}=t_c-T_{close}$), CLIK, `ABORT_SAFE`, E-STOP 은 공통이다. planner 는 `supervisor.decel.mode` 로 고른다 (이름은 역사적, MD-48). 여기서 planner 는 스레드가 아니라 공 궤적 → CLIK 입력의 사상이다 — 이 문서의 "계획기" 는 L3 계획기 스레드를 가리킨다.

| | closed_form planner | mpc planner (E1) |
|---|---|---|
| 포구 후보 ($p_c$ · $t_c$ · $a_d$ · `q_star`) | L3 탐색 (계획기 스레드) | 같다 — MD-46 의 편차. v1 의 순위와 $\gamma_f$ 는 soft-catch DS rollout 으로 매긴다 (L3 §4.8) |
| 기준 생성 | APPROACH – CLOSING 은 soft-catch DS, DECEL 은 closed-form 직선 등감속 (RT tick) | APPROACH – 정지 전부를 MPC 가 계획 (계획기 스레드) → 관절 노드 → RT 샘플러 + FK |
| CLIK 입력 | 손 위치 · 선속도 ff · 접근축 $a_d$, 자세 목표는 대기 자세 | 손 pose · twist ff (각속도 포함) · 접근축, 자세 목표 $q_{ref}+\dot q_{ref}/K_n$ (MD-36) |

v1 과 G1 MPC 의 구조 차이:

| 항목 | v1 | MPC |
|---|---|---|
| 계획기가 정하는 것 | 포구 시각 · 포구점 · 접근축 · γ profile, 포구 순간의 정적 IK 자세 | 전 구간의 관절 궤적 노드열 |
| 탐색 | vision 샘플 격자 위 1차원 시간 탐색 + 후보별 IK + γ rollout | 포구 시각 격자 × condensed QP (단일 팔은 v1 탐색을 쓴다 — MD-46, MPC 선택은 E3-F01) |
| 포구 전 운동 | soft-catch DS 가 매 tick 생성 | MPC 가 계획, RT 는 보간 · 추종 |
| 포구 후 정지 | TCP 직선 등감속 | MPC 궤적의 꼬리 (관절 공간) |
| 관절 한계 | 끝점 사이 도달시간 닫힌식 (필요조건, L3 §4.3) + CLIK 의 tick 별 제약 | horizon 전체의 위치 · 속도 · 토크 제약 |
| 여유 자유도 | 포구 자세의 roll 만 | 비용으로 전 구간 분배 |
| 다중 팔 · waist · 충돌 | 범위 밖 | 같은 코어에 항을 더한다 (E3). 단일 팔에서는 뺀다 (MD-46) |

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
- 따라서 단일 팔에서 MPC 가 성공률을 올린다고 기대하지 않는다. MPC 의 근거는 waist + dual-arm 에서 v1 구조가 표현하지 못하는 문제 (여유 자유도 분배, 협조, 충돌) 다. 단일 팔 시험의 목표는 같은 MPC 의 성공률이 closed_form 과 크게 다르지 않은 것이다 (비열등, 튜닝은 E1-F10 — MD-47 · MD-50).
- 위 표는 v1 계획이 잰 값이다. **지금 구성의 대조군은 §8 의 E0-F02 값**이며 `ur5e_p1b` 는 위 표보다 낮다 (287/400).
- 위 값은 모두 sim 값이다. v1 의 S10 실기 식별 (servo 지연, 토크 한계) 이 바뀌면 MPC 의 튜닝과 측정도 다시 한다.

### 3.3 v1 결정과의 관계

| v1 결정 | 내용 | 이 계획에서 |
|---|---|---|
| §8 (2026-09-29) | v1 은 NLP/MPC 로 전환하지 않는다 | **개정** — MD-6 |
| §8 | 두 번째 계획기 구현이 생길 때 interface 도입 (ARCH-3) | 적용 — E3-F01. E1 의 mpc planner 는 v1 탐색을 그대로 쓰므로 두 번째 계획기 구현이 아니다 (MD-46) |
| §8 | 전환 시 재사용 후보는 `rtc_mpc` | 채택하지 않음 — MD-1 |
| D-16 · D-S8-18 | 유도 가속 box 는 도달시간 전용, CLIK 은 토크 기반 제약 | 적용 — MD-7 |
| C-35 | `ABORT_SAFE` 는 원인과 무관하게 관절 공간 정지 | 불변 |
| L7 G7-B | DECEL 진입 시 기준 상태 연속 | 적용 — E1-F04. 방식은 MD-39 · MD-40. `mpc` 에서는 APPROACH 부터 한 구간 계열을 따라 DECEL 진입이 연속이 된다 (E1-F09) |
| `supervisor.sat_ticks` | 로봇별 포화 임계 (sim 분포에서 도출) | `mpc` 에서는 soft-catch DS 가 돌지 않아 발생하지 않는다 — E1-F09 에서 확인 |
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
| MD-20 | MD-18 의 실행: sim 추정기의 profile 은 ball_perception `ball_perception_sim/config/sim_profile.catching.json` (schema 0.2, 지금까지의 사본과 같은 내용) 이다. rtc-framework 의 로봇별 사본은 제거한다. `ball_sim_ws` 의 ball_perception 은 pull 만 하고, 변경은 개발용 checkout 에서 PR 로 한다 | 사용자 결정. 0.2 를 유지해야 E0-F02 의 대조군과 같은 추정기 동작으로 돈다. 컨트롤러 YAML 과의 정합은 자동 검사가 없어 컨트롤러가 예측 격자 세 키 (`prediction.dt_expected` · `io.n_min` · `planner.slice.dt`) 를 read-only 미러로 낸다 | — | 2026-09-30 |
| MD-21 | decel MPC 의 지평 $N_s\Delta_s$ 는 **정지 시간 그 자체**다. 값은 E1-F03 이 정하고 E1-F01 코어는 받기만 한다 (테스트 기본값 $\Delta_s$ 0.05 s · $N_s$ 12 · 블록 {1,1,2,2,3,3}) | 비용이 jerk 제곱합뿐이고 시간 항이 없으므로 최적해는 지평 전체를 쓴다. v1 정지 시간 p50 0.35 s (§8) 보다 길게 잡으면 정지 거리가 늘어나 G-1 의 정지 거리 비교와 위치 여유에 직접 영향을 준다 | formulation §1.6 의 "E1-F01 spec 에서" 를 E1-F03 으로 정정 | 2026-09-30 |
| MD-22 | E1-F01 의 "Solve 할당 0" 은 **코어 자신의 경로** (선형화 · FK · condensing · 결과 기록) 에 대해 단언한다. ProxQP 가 `Solve` 안에서 하는 C 할당 (warm 12 · pre-solve 18 회, n = 7) 은 알려진 한계로 기록하고 [#654](https://github.com/hyujun/rtc-framework/issues/654) 에서 다룬다 | 사용자 결정 (2026-09-30). 같은 wrapper 를 쓰는 기존 `CatchPoseIk` 도 RT 계획기 스레드에서 같은 할당을 하고, 기존 게이트는 이것을 못 봤다 (C `malloc` 게이트가 처음 잡았다). F01 만 고치는 것보다 사용자 전체를 한 번에 다루는 편이 맞다 | #627 Done when 5 부분 충족 | 2026-09-30 |
| MD-23 | 계획기의 decel 경로도 **ProxQP 밖의 할당 0** 을 단언하고 ProxQP 의 할당 횟수는 기록만 한다. [#654](https://github.com/hyujun/rtc-framework/issues/654) 는 이 작업과 병행한다 | 사용자 결정. MD-22 와 같은 경계다. C `malloc` 게이트는 테스트 파일마다 한 번만 쓸 수 있어 준비 · 풀이 · 게시를 따로 게이트한다 | — | 2026-09-30 |
| MD-24 | 출하 지평은 $N_s$ 14 · $\Delta_s$ 0.025 s · 블록 {1,1,2,2,4,4} 로 정지 시간 0.35 s 다. 계획기의 $\dot q_{\max}$ 는 팔 device 의 `max_velocity` 출하값이다 | v1 정지 시간 p50 (ur5e_p1b 0.348 s, iiwa7_leap 0.324 s, §8) 과 맞춘다. 같은 시간이면 최소 jerk 정지 거리는 $0.4\,v_0T$ 로 v1 의 등감속 $0.5\,v_0T$ 보다 짧다. 간격을 절반으로 해 노드 1 이 효력 시각에서 25 ms 뒤에 온다. 계획기 예산을 넘으면 7 × 0.05 s 로 물러난다 | MD-21 의 테스트 기본값은 코어 테스트에만 남는다 | 2026-09-30 |
| MD-25 | **armature 는 제어에 쓰지 않는다.** 계획기는 코어에 0 벡터를 넘기고, armature 로 인한 토크 차이는 분석용 기록에만 남긴다. 코어의 `DecelMpcLimits::armature` 입력은 그대로 둔다 | 사용자 결정. 실기에서 확인하기 어렵고 기어비 · 기어 효율 때문에 부정확할 가능성이 높다. 코어 입력을 지우면 테스트와 분석 도구가 쓰는 경로가 사라지고, 비어 있는 입력을 0 으로 받게 바꾸면 빠뜨린 값이 조용히 0 이 된다 | §4 미결의 armature YAML 키 · formulation §1.3 토크 행 · M-11 | 2026-09-30 |
| MD-26 | 기준 없는 첫 주기 (pre-solve + 본 solve) 는 $t_c-$now_lead $\le T_{pre}$ (기본 0.1 s) 인 첫 COMMITTED · CLOSING wake 에서 푼다. 그때까지 아무것도 게시하지 못했으면 DECEL 의 첫 wake 에서 $k\ge1$ 로 cold 풀이한다 (MD-31 의 창 안에서만). 예산은 decel 풀이를 시작한 시각부터 잰다 | 그 전 wake 에서는 초기 상태의 외삽 구간이 `T_freeze` 만큼 길어 예측이 의미가 없다. COMMITTED 뒤에는 탐색이 없어 예산 전부를 쓸 수 있다 | — | 2026-09-30 |
| MD-27 | 노드 payload 는 `PlanSnapshot` 에 넣지 않고 형제 POD `DecelPlanSnapshot` 과 자기 SeqLock, 자기 채택 규칙으로 보낸다. 관절 용량은 8, 노드 용량은 `kMaxDecelNodes` 24 다. 노드 0 의 시각은 효력 시각이고, 격자 시각은 정수 ns 로 계산한다 | RT 는 COMMITTED 뒤에 새 `PlanSnapshot` 을 받지 않는다 (too_late · repeat 거부). 관절 용량을 `kMaxPlanNv` 32 로 잡으면 payload 가 19 KB 라 복사와 SeqLock 재시도 창이 커지고, 8 이면 4.8 KB 다 | formulation §1.5 마지막 문단 | 2026-09-30 |
| MD-28 | 초기 상태 $x_0$ 는 효력 시각에서의 RT 기준 상태 예측이다. RT 가 보고한 명령 $(q,\dot q)$ 를 lead 축 시각 (보고 시각 + `T_arm`) 에서 효력 시각까지 외삽하고, $\ddot q$ 는 계획기의 wake 사이 차분으로 추정한다. 속도는 코어의 box 로 사영하고 사영했음을 기록한다 | RT 쪽 2 ms 유한 차분은 CLIK 기준 잡음과 재시드 계단을 키운다. box 밖의 $x_0$ 는 코어가 거부하므로 사영하지 않으면 게시할 수 없다. 예측을 DS rollout 등으로 올리는 것은 E1-F04 에서 실측한 진입 차이가 허용치를 넘을 때 한다 | formulation §1.6 $x_0=\hat x(t_c)$ | 2026-09-30 |
| MD-29 | DECEL 모드의 계획기 activity 를 idle 에서 decel 계획으로 바꾼다. decel wake 의 주기 결과 (`CycleOutcome`) 는 idle 로 두고 decel 결과는 기록의 별도 필드로 낸다 | 기존 계획기 테스트가 COMMITTED wake 의 idle 결과와 게시 없음을 단언한다. 결과 코드까지 바꾸면 그 단언을 두 번째로 고치게 된다 (PROC-6) | 계획기 activity 표 (별도 커밋) | 2026-09-30 |
| MD-30 | E1-F02 의 "CLIK 잔차 0" 검증은 **FK 일관성** ($T(q_{ref})=T^d$, $J\dot q_{ref}=V^{ff}$) 으로 좁히고 CLIK 속도 잔차는 정보용으로 기록한다. 자세 과제의 속도 feedforward 는 E1-F04 가 `[CONCERN]` 으로 다룬다 | 현 `ClikReferenceGenerator` 의 자세 과제는 위치 오차 항뿐이라 $q_c=q_{ref}$ 여도 $v^\ast\ne\dot q_{ref}$ 다. feedforward 를 넣는 것은 `rtc_tsid` public API 변경 (소비자 둘) 이고, $q_{ref}$ 를 CLIK 에 실제로 넣는 것은 E1-F04 다 | formulation §1.5 · §4 항목 8 | 2026-09-30 |
| MD-31 | 정지 끝은 $t_c+N_s\Delta_s$ 에 고정한다. 포구 뒤 재계획은 격자점 $k\le k_{\max}$ (기본 4, 0.1 s) 에서만 하고, 노드 수 $N_s-k$ 인 코어를 $k$ 마다 configure 에서 만든다 | 코어의 노드 수는 Init 에서 고정이라 재계획마다 끝이 효력 시각 + $N_s\Delta_s$ 로 밀린다. 포구 뒤 첫 재계획이 포구 전 예측의 오차를 실제 RT 상태로 바로잡는 주 수단이라 포구 전에만 게시하는 안은 택하지 않았다 | — | 2026-09-30 |
| MD-32 | E1-F03 의 RT 쪽 변경은 `PlannerRtState` 에 따르는 계획의 $t_c$ 를 채우는 것뿐이다. RT tick 의 decel payload 읽기 · 채택 · 기록은 E1-F04 가 한다. 채택 규칙과 효력 시각 전환 규칙은 E1-F03 이 순수 함수로 만든다 | RT 가 소유하는 멤버를 새로 두면 trial reset 표와 E-STOP reset 경로를 고쳐야 한다 (E-8). E1-F04 가 이미 E-8 이므로 거기서 함께 다룬다 | — | 2026-09-30 |
| MD-33 | 게시 조건은 풀이 성공, 예산 안, 효력 시각 전, `slack_max` 와 `slack_terminal_max` 가 유한하고 임계 이하일 때다. 두 임계는 기본 0.1 이고 $\eta'_\tau+$ `slack_max` $\le$ `joint_cmd.eta_tau` 를 configure 에서 검사한다 | 임계 비교를 부정형으로 쓰면 NaN 이 통과한다. 합이 CLIK 의 토크 box 를 넘으면 계획이 CLIK 에서 실행될 수 없다. 종단 임계를 처음부터 조이면 손목의 정적 중력비가 큰 자세에서 늘 게시가 막힐 수 있어, 분포를 기록한 뒤 E1-F06 이 조인다 | — | 2026-09-30 |
| MD-34 | RT 의 decel lane 은 `supervisor.decel.mode: mpc` 에서만 돈다. 기본은 `closed_form` (키가 없을 때 포함) 이고 그때 RT 는 `decel_box_` 를 읽지 않으며 계획기는 decel 코어를 만들지 않는다 (`planner.decel_mpc.enabled: true` 여도 WARN 만, MD-44). `mpc` 의 전제 — 샘플러 구성, `joint_cmd.K_n` $\gt0$, $\eta_v\lt1$, 팔의 관절별 속도 box, CLIK 위치 box, `planner.workspace.catch_box`, decel 계획기 구성 (`planner.decel_mpc.enabled`) — 가 빠지면 park 한다 (활성화 거부). oracle plan profile 은 decel 계획기 없이 허용한다 | 기본값의 출력 불변 (§7). 전제가 빠진 `mpc` 가 조용히 closed-form 으로 돌면 G-1 의 MPC arm 이 v1 을 재게 된다. oracle profile 은 시험 전용이고 계획기와 함께 켤 수 없어, 그 profile 에서는 테스트가 box 의 writer 다 | — | 2026-09-30 |
| MD-35 | RT 가 새로 소유하는 상태 (채택 메모리, 대기 구간, 따르는 구간, 이 DECEL 의 법칙) 는 재무장과 E-STOP 의 reset 이 모두 되돌린다. E-STOP 경로에서 바뀌는 코드는 두 reset 함수의 대입 추가뿐이다. 구간은 쓸 때마다 따르는 plan 의 id · $t_c$ 와 대조한다. 따르는 중에 어긋나거나 샘플이 실패하면 `ABORT_SAFE` 다 | E-8 (`[CONCERN]` 컨펌 2026-09-30). reset 행이 빠져도 다른 plan 의 구간을 따르지 않게 하는 2차 방어다. 따르는 중에는 soft-catch DS 를 돌리지 않으므로 closed-form 으로 돌아가면 기준이 계단이 된다 | — | 2026-09-30 |
| MD-36 | 자세 과제의 속도 feedforward 는 호출측 등가식으로 넣는다 — 자세 목표를 $q_{ref}+\dot q_{ref}/K_n$ 으로 넘긴다. `rtc_tsid` 는 바꾸지 않는다. CLIK 의 API 는 E2-F04 가 정한다 | $K_n(q'-q)=K_n(q_{ref}-q)+\dot q_{ref}$ 로 formulation §2.2 의 $\dot q_n$ 과 같은 QP 다. CLIK 은 자세 목표를 그 식에서만 읽는다. E-8 PR 이 public API 를 건드리지 않는다 | MD-30 의 "E1-F04 가 `[CONCERN]` 으로 다룬다" 를 닫음 | 2026-09-30 |
| MD-37 | decel 구간의 채택: lane 은 COMMITTED · CLOSING · DECEL 에서 매 tick 판정하고, 대기 슬롯이 비었을 때만 채택한다 (차 있으면 box 에 두고 다음 tick 에 다시 본다). 나이 상한은 50 ms 상수이고 채택할 때 한 번만 본다. 나이의 now 는 box 를 읽은 뒤의 시계다 (허용 오차 없음). reset floor 는 옮기지 않고, 구간이 출발한 RT 상태의 시각 (`rt_state_ns`) 을 floor 와 비교하는 검사를 `DecelAdmissionContext` 의 새 필드로 더한다 | 게시에서 채택까지는 1 tick 이라 나이는 box 에 묵은 구간과 RT 정지만 거른다. $t_c$ 직전에는 $k\ge1$ 구간이 $k=0$ 구간보다 먼저 올 수 있어, 덮어쓰면 진입 tick 에 따를 구간이 없다. 계획기는 게시 시각을 저장 전에 찍으므로 읽은 뒤의 시계로는 나이가 음수가 되지 않는다. floor 와 tick 끝의 RT 상태 저장 사이에 찍힌 게시는 floor 를 통과하지만, reset tick 이 저장하는 `rt_state_ns` 는 floor 이상이고 그 앞 tick 은 미만이다. floor 를 옮기면 E-STOP 경로와 기존 단언을 고쳐야 한다 | §4 미결의 채택 나이 상한 · reset floor | 2026-09-30 |
| MD-38 | 전환: 대기 구간은 샘플 시각이 node 0 에 닿고 연속성 게이트 (MD-39) 를 지날 때 따르는 구간이 된다. DECEL 진입 tick 에 넘겨받을 구간이 없으면 `ABORT_SAFE` 다 (MD-44). DECEL 도중의 재계획 구간이 게이트를 못 지나면 그 구간만 버리고 따르던 구간을 계속 따른다. HOLD 에서는 전환하지 않는다 | MD-26 의 "DECEL 첫 wake 의 $k\ge1$ 풀이" 는 따르던 구간을 새것으로 바꾸는 데 쓴다. 법칙은 섞지 않는다 (MD-44). HOLD 는 이미 정지한 상태다 | r9 의 "진입 fallback · DECEL 도중 넘겨받기" 는 MD-44 가 대체 | 2026-09-30 |
| MD-39 | DECEL 진입의 기준 상태 연속 (v1 L7 G7-B) 은 둘로 나눠 지킨다. (1) node 0 를 RT 의 명령 상태로 정확히 만든 구간에서 진입 tick 의 기준과 명령의 차이가 1e-9 미만이다 (결정적 테스트). (2) 실제 구간은 전환 tick 에서 관절마다 $\vert\Delta\dot q_i\vert+K_p\vert\Delta q_i\vert\le\rho_{\max}(1-\eta_v)\dot q_{\max,i}$ 일 때만 넘겨받는다. $\rho_{\max}$ 는 `supervisor.decel.switch_margin` (기본 1.0, provisional) 이다 | v1 의 1e-9 는 기준 생성기를 reset 하지 않아서 성립하는 값이고, MPC 의 node 0 는 예측이다. 우변은 MPC 가 CLIK 의 되먹임을 위해 남긴 속도 여유다 (formulation §2.3). 넘는 구간은 따르지 않으므로 (진입이면 `ABORT_SAFE`, 재계획이면 따르던 구간 유지) 진입 불연속의 상한이 동작 조건이 된다. 좌변의 $K_p$ 는 고유값 상한이라 관절별 상한은 아니다 — 실측으로 조인다 | §3.3 의 "L7 G7-B — 적용" 의 적용 방식 | 2026-09-30 |
| MD-40 | RT 는 구간을 now_lead $+\,h$ 에서 샘플한다 ($h$ = 제어 주기). 계획기는 RT 가 보고한 명령을 보고 시각 + `T_arm` $+\,2h$ 의 상태로 본다 (`DecelPlannerConstants` 의 새 필드, 기본 0 이고 바인딩이 채운다) | 구간을 따르는 동안 위치 · 축 · 자세 목표는 모두 $q_{ref}(s)$ 에서 오고 CLIK 은 들고 있는 명령에서 적분하므로, 명령이 $q_{ref}(s)$ 에 있으면 해는 $\dot q_{ref}(s)$ 이고 tick 을 나가는 명령은 $q_{ref}(s+h)$ 다 (감쇠 · smoothing 항과 $h^2\ddot q$ 만 남는다). $s$ = now_lead $+\,h$ 이면 명령의 시간 label 은 now_lead $+\,2h$ 이고, 전환 전후로 명령 궤적의 시간 축이 이어지려면 계획기도 보고를 같은 label 로 봐야 한다. v1 명령과 DS 기준 사이의 시간 차는 이 규약과 무관하다 — 측정 (E1-F04 0 단계 테스트, shipped `dynamic` 행): 최고 속도에서 0.8 $h$, 중앙값 0.2 $h$. 자세 항과 적분 과도가 정하므로 1 차 모델의 2 $h$ 는 v1 에 맞지 않는다 (r9 의 유도는 이 점에서 틀렸다). 그래도 계획기는 v1 명령 (경로 (ii), 진입 전 구간) 도 $+\,2h$ 로 읽는다. 맞춰야 하는 것은 진입 전환이고, 전환은 tick 이 출발하는 명령 (직전 tick 의 것, label $t-h+\delta$) 을 구간의 $t+h$ 와 비교한다. 보고를 $+L$ 로 읽은 구간은 명령 궤적을 $L-\delta$ 만큼 늦춘 것이므로 둘은 $\delta$ 와 무관하게 $L=2h$ 에서 만난다. v1 의 $\delta$ 로 읽으면 ($L=\delta$) 전환에서 $2h$ 가 어긋난다 | MD-28 의 "보고 시각 + `T_arm`" | 2026-09-30 |
| MD-41 | E1-F04 는 tick record 의 decel 블록까지 만들고 `catching_diag` 컬럼은 E1-F05 가 낸다. 초기 상태 예측의 개선 여부 (MD-28) 는 E1-F05 직후 · E1-F06 전에 로봇당 200 발 (E0-F02 와 같은 투척) 로 판단한다 — 진입 tick 의 게이트 초과율이 5 % 를 넘거나 $\rho$ 의 p95 가 0.5 를 넘으면 고친다 | E-8 PR 을 `integrated_bringup` · `rtc_controllers` 안에 둔다. 컬럼 목록은 `rtc_tools` 의 테스트가 순서까지 고정한다. 판단 규칙은 값을 보기 전에 정한다 | §4 미결의 "초기 상태 예측의 개선 여부" | 2026-09-30 |
| MD-42 | decel 계획기의 관절 위치 box 는 URDF 한계와 CLIK 위치 box (device 한계 − `limit_margin`) 의 교집합이다. `m_q` 는 그 안쪽에 더 건다 | MPC 의 해가 CLIK 에서 실행되려면 MPC ⊂ CLIK 여야 한다 (MD-7 과 같은 원칙). URDF 한계만 쓰면 출하 구성에서 iiwa7 A7 이 CLIK box 를 2.6e-5 rad 넘고 (URDF 3.05433, device 3.0543), ur5e_p1b 는 fallback 상수의 반올림 덕에 1.5e-5 rad 로 겨우 포함된다. device 한계에서의 여유는 0.1 rad 가 된다 (E0-F02 closed-form 의 최소 여유 iiwa7_leap 0.128 rad) | MD-24 의 계획기 box (URDF 한계), E1-F03 | 2026-09-30 |
| MD-43 | RT 는 decel 구간을 채택할 때 구간의 경로를 $p_c$ 로 옮겨 본다 — 노드 $k$ 의 catch frame 위치 $p_k$ 에 대해 $p_c+(p_k-o)$ 가 모두 `planner.workspace.catch_box` 안이어야 한다. $o$ 는 이 정지의 시작점 — 진입이 넘겨받은 첫 구간의 node 0 위치 — 이고, 진입 전에 채택하는 구간은 자기 $p_0$ 다. 밖이면 채택하지 않는다 (진입이면 `ABORT_SAFE`, MD-44) | 계획기는 포구점과 closed-form 직선 정지점 $p_c+(\gamma v)^2/(2a_{dec})\,\hat v$ 이 이 box 안인 plan 만 게시한다 (L3 §4.9) — 예약하는 것은 $p_c$ 에서의 정지 **변위**다. MPC 정지는 고정 $N_s\Delta_s$ = 0.35 s 의 최소 jerk 라 변위가 약 $0.4v_0T$ 이고, $v_0\lt0.8\,a_{dec}T$ = 2.8 m/s 에서 예약보다 길다 (초과는 $v_0$ 1.4 m/s 에서 최대 약 0.1 m). 경로도 직선이 아니어서 끝점만으로는 부족하다. 노드 사이 (25 ms) 는 보지 않는다. r9 의 절대 위치 검사는 E1-F04 실시계 테스트에서 틀렸다: v1 법칙이 $t_c$ 에 손을 $p_c$ 에서 19 cm 떨어진 box 밖 (z 0.19 < 0.21) 에 두었고, closed-form 정지도 같은 자리에서 시작하는데 MPC 구간만 거부해 `mpc` 에서는 매 시행 abort 가 된다. 재계획은 정지 도중 ($t_c+k_0\Delta_s$) 에서 시작하므로 자기 $p_0$ 로 옮기면 그때까지 간 거리가 빠진다 — 그래서 기준은 정지의 시작점 $o$ 다 (E1-F04 code review, 2026-09-30) | L3 §4.9 의 정지 예약이 MPC 정지를 덮지 않음. r9 판 (절대 위치) 을 대체 | 2026-09-30 |
| MD-44 | DECEL 법칙은 섞지 않는다. 법칙은 configure 에서 `supervisor.decel.mode` 로 정하고 활성화 동안 바뀌지 않는다. `closed_form` 은 v1 closed-form DECEL 만 쓰고 계획기는 decel 코어를 돌리지 않는다. `mpc` 는 모든 DECEL 을 MPC 구간으로 한다 — 진입 tick 에 따를 구간이 없으면 (구간 없음 · 나이 · plan 불일치 · malformed · reset 전 · 게이트 · 작업공간) `kParamsTbd` 로 `ABORT_SAFE` 다. closed-form 으로 들어가는 fallback 과 DECEL 도중의 넘겨받기는 없다 | 사용자 결정 (2026-09-30). 한 시행에 두 법칙이 섞이면 G-1 의 MPC arm 이 무엇을 쟀는지 흐려지고, 전환 경로마다 연속성과 reset 을 따로 지켜야 한다. `ABORT_SAFE` 의 정지는 closed-form DECEL 이 아니라 QP 와 무관한 관절 공간 ramp 이고 전이표 행 (`{CLOSING · DECEL, kParamsTbd}`) 은 이미 있다. 대가: `mpc` 에서는 따를 구간이 없는 시행이 전부 abort 로 끝난다 — sim smoke 에서 사유별로 센다 | MD-11 (2) 를 `mpc` 에서 대체. MD-38 의 r9 판 (진입 fallback · 넘겨받기) | 2026-09-30 |
| MD-45 | `supervisor.decel.mode: mpc` 는 APPROACH 부터 정지까지 팔 기준을 MPC 가 만든다. 입력은 공의 미래 궤적과 계획기 탐색이 고른 plan ($p_c$ · $t_c$ · $a_d$), 출력은 지금과 같은 관절 노드 — RT 가 FK 로 CLIK task 목표를, $q_{ref}+\dot q_{ref}/K_n$ 로 null space 자세 목표를 만든다. 코어 · 관절 노드 payload · RT 샘플러 · 전환 게이트 · reset 처리 (MD-35 – MD-44) 는 유지하고, formulation §1.3 의 포구 항 (포구 위치 · 접근축 · 포구 구간 상대속도) 을 단일 팔 형태로 더한다. `mpc` 에서 v1 DS 법칙은 돌지 않는다. 세부 (horizon · 첫 구간 채택 · 재계획 · $t_{cmd}$ · 가중치) 는 그 기능의 계획이 정한다 | 사용자 결정 (2026-09-30). E1-F04 sim smoke (§8) 에서 진입 구간의 $x_0$ 를 v1 명령 외삽으로 정해 진입 게이트가 20/20 거부했다 ($\rho$ 4.1 – 10.1). APPROACH 진입 때 팔은 대기 자세에 정지해 있어 $x_0$ 가 정확하고, 이후 재계획은 따르는 자기 구간에서 $x_0$ 를 평가하므로 외삽이 없다. 폐기한 대안: 정지 구간만 두고 $x_0$ 만 plan · 공 궤적에서 계산 — v1 이 그 목표에 못 미치면 진입 불일치가 남는다 | MD-3 (DECEL 에서만 MPC 궤적) 을 `mpc` 에서 넓힌다. MD-41 의 진입 $x_0$ 예측 측정은 그 기능의 전환 규칙이 정해진 뒤 다시 정의한다. E3-F01 의 포구 항을 단일 팔로 G-1 전에 앞당긴다 — G-1 의 MPC arm 이 무엇을 재는지도 그 계획에서 다시 정한다 | 2026-09-30 |
| MD-46 | 설계는 formulation §1.3 하나다. 단일 팔 (ur5e_p1b · iiwa7_leap) 은 같은 MPC 에서 dual arm · waist 전용 항 — waist 억제, 왼팔 rest, 각운동량, 자기충돌, 공–왼팔, waist 토크 행 · counter-swing, 관절군별 move blocking — 만 뺀 구성이고, g1_p1b 는 같은 코어에 그 항을 더한다. dual 전용이 아닌 항 (포구 위치 · $w_\Delta$ 의 공분산 가중, 접근축, 상대속도와 slack $s_v$, 토크 행, 한계, 종단 정지, trust region) 은 단일 팔에서도 유지하고, 포구 구간 $\mathcal K_c$ 는 손별 파라미터다. 편차는 하나다 — 포구 후보 ($t_c$) 는 MPC 의 바깥 루프가 아니라 v1 탐색이 고른다 (MD-45). closed_form 과 mpc 는 입력 (추정기의 공 미래 궤적) 과 출력 (CLIK 입력) 이 같은 두 planner 이고, 추정기 · supervisor · 손 시퀀서 · CLIK · `ABORT_SAFE` 는 공통이다 (§3.1) | 사용자 결정 (2026-09-30). 단일 팔은 dual arm MPC 의 시험대다 — 뺀 것이 dual 전용 항뿐이어야 g1_p1b 에서 항을 더하는 것이 같은 코어의 확장이 된다. 후보 선택까지 MPC 로 옮기면 MPC 가 두 번째 계획기 구현이 되어 interface 도입 (ARCH-3) 이 함께 오고 선택과 생성이 한꺼번에 바뀐다 — 생성 법칙의 차이만 먼저 본다. 대가: v1 의 후보 순위와 $\gamma_f$ 는 soft-catch DS 의 rollout 으로 매긴 값이다 (L3 §4.8). $\Sigma_p$ 가 단일 팔 경로에 없으면 상수 가중으로 두고 편차로 기록한다 | MD-3 · MD-12 (단일 팔에서 각운동량 항은 뺀다) · MD-21 · MD-24 · MD-26 · MD-28 · MD-31 · MD-34 · MD-37 · MD-38 · MD-43 · MD-44 — `mpc` 에서 범위가 APPROACH–정지로 넓어진다. 새 값은 E1-F07 – F09 가 정한다 ([#660 결정 목록](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5913561760)) | 2026-09-30 |
| MD-47 | E3 는 같은 MPC 에 dual arm · waist 항을 더하는 epic 이고, 착수 조건은 E2 (g1_p1b 준비) 와 E1-F07 (코어) 이다. G-1 은 E3 의 착수 조건이 아니다 — 단일 팔 mpc planner 가 closed_form 대비 비열등한지와 `mpc` 를 기본값으로 바꿀지만 판정한다. 목표는 성공률이 closed_form 과 크게 다르지 않은 것이다 | 사용자 결정 (2026-09-30). E3 의 항은 단일 팔 결과와 무관하게 G1 에서 필요하다. 단일 팔 성공률이 목표에 못 미치면 튜닝 (E1-F10) 으로 다룬다 | MD-8 (E3 는 G-1 결과로 착수 여부를 정한다), §1 의 G-1 문장 | 2026-09-30 |
| MD-48 | `Decel*` 식별자 (`supervisor.decel.mode` · `supervisor.decel.switch_margin`, `planner.decel_mpc.*`, `DecelMpc` · `DecelPlanSnapshot` · `kMaxDecelNodes` 등) 의 이름은 바꾸지 않는다. `mpc` 에서 이들의 범위는 APPROACH–정지이고, `supervisor.decel.mode` 는 planner 를 고르는 키다. 문서는 "이름은 역사적" 으로 적는다. rename 은 E3 착수 전에 별도 refactor 로 한다 | 사용자 결정 (2026-09-30). `supervisor.decel.mode` 와 `planner.decel_mpc.*` 는 출하 YAML 의 public key 라 이름을 바꾸면 소비자 (config · 테스트 · 도구) 를 건드린다. 기능 PR 에 섞으면 기능 동등성 판정이 흐려진다 | — | 2026-09-30 |
| MD-49 | E1-F07 의 코어는 비용 · 제약을 항 단위로 조립하고, 관절군별 move blocking 행렬 $E$ 를 넣을 자리만 둔다. 다관절군 일반화는 E3 에서 한다 | 사용자 결정 (2026-09-30). 항 단위 조립이 있어야 E3 가 코어를 fork 하지 않고 항을 더한다 (design-principles P5). 쓰이지 않는 일반화를 지금 만들면 검증할 소비자가 없다 | — | 2026-09-30 |
| MD-50 | 튜닝은 별도 기능 E1-F10 이다 (순서: E1-F09 → E1-F05 → E1-F10 → E1-F06). 규칙: (1) 비열등 한계와 G-1 의 N 은 튜닝 전에 E1-F10 spec 에서 고정한다, (2) 튜닝 투척은 G-1 과 다른 seed 로 하고 closed_form 도 같은 seed 로 돌려 paired 로 비교한다, (3) planner 파라미터만 튜닝하고 공통부 (추정기 profile, $T_{freeze}$ · $T_{close}$, CLIK gain · 한계) 는 고정한다 — 공통부를 바꿔야 하면 escalate 한다. MD-41 의 200 발 측정은 모든 재계획 전환의 거부율과 $\rho$ p95 로 재정의하고 (임계 5 % · 0.5 유지) E1-F10 안에서 한다 | 사용자 결정 (2026-09-30). 게이트 판정과 튜닝이 한 PR 에 섞이지 않게 한다 (§6 "게이트 판정은 따로 둔다"). G-1 의 투척으로 튜닝하면 판정이 튜닝에 맞춰진다. 공통부를 바꾸면 closed_form 대조군 (E0-F02) 을 다시 재야 한다. MD-45 로 진입 순간의 외삽이 없어져 MD-41 의 진입 $x_0$ 측정은 뜻을 잃었다 — 남는 불연속은 재계획 전환이다 | MD-41, §1 의 "비열등 한계와 N 은 E1-F06 spec" | 2026-09-30 |
| MD-51 | E1-F07 의 격자 (포구 전 · 정지 구간의 간격) 는 **알고리즘을 formulation §1.6 대로 구현하고 검증한 뒤** 계산 시간을 재서 정한다. 격자는 코어의 파라미터다 — 노드별 간격을 받고, 안 A (포구 전 0.05 s + 정지 0.025 s × 14) 와 안 B (전부 0.05 s) 를 같은 코어가 표현한다. 임계는 지금의 추정기 게시 주기 **30 Hz** 에서 정한다: 개발 PC · Release · 7 자유도 · 표본 200 의 p99 로 재계획 (warm) ≤ 10 ms, 새 plan 의 첫 풀이 (cold) ≤ 12 ms, 풀이 실패 0. 추후 60 Hz (카메라의 refresh rate) 로 게시할 경우는 **대비만** 한다 — 코어에 게시 주기 상수를 두지 않고, 측정에 60 Hz 여유 (warm ≤ 5 ms) 를 함께 기록한다. formulation 을 바꾸는 축소 수단 (토크 slack 을 노드당 하나로, 한계 행을 일부 노드에만) 은 미리 구현하지 않고, 임계를 넘으면 수치와 함께 결정을 받는다. 격자의 확정은 측정 뒤 사용자가 한다 | 사용자 결정 (2026-10-01). 부분 구현으로 잰 시간은 포구 항의 조립 · 선형화가 빠진 값이다. warm 의 값은 sim 에서 15 ms (개발 PC 의 1.5 배, E1-F04 sim smoke) 로 주기 33.3 ms 의 45 % 이고 출하 `budget_s` 20 ms 안이다. cold 의 값은 탐색 (sim p99 8.6 ms) 과 같은 wake 에서 한 주기 안에 끝나는 값이다. 안 A 는 payload 의 간격 필드 · 균일 간격 샘플러 · 노드 용량 (E1-F08 · F09) 을 바꾸므로 계산 시간만으로 고르지 않는다. 폐기한 대안: 구현 전 spike 로 격자를 먼저 고른다 ([#660 코멘트](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5922354878)) | §4 미결의 "격자", MD-24 의 "계획기 예산을 넘으면 7 × 0.05 s 로 물러난다" 는 `mpc` 의 APPROACH–정지 격자에서 이 규칙으로 대체 | 2026-10-01 |
| MD-52 | E1-F07 의 포구 항 (포구 노드 $k_c$ 한 점). 포구 위치의 가중은 코어 입력 $W_p$ (3×3, 대칭 PSD) 다 — 코어는 추정기의 형식을 모르고, $\kappa(\Sigma_p+\sigma^2I)^{-1}$ 는 고유값에 하한 · 상한을 둔 순수 함수가 만든다. 접근축은 `rtc_math` 의 $e_a$ · $J_a$ 를 쓴다 (`CatchPoseIk` · CLIK 과 같은 오차). 상대속도는 $H_v\delta q$ 를 포함한다. slack $s_v$ 는 구현하되 코어의 기본값은 끔이다. 경로 이탈 $w_{path}$ 는 별도 항을 두지 않는다 ($\mathcal K_c=\lbrace k_c\rbrace$ 에서 $W_p$ 에 $w_{path}P_\perp$ 를 더한 것과 같다). $w_\Delta$ 의 공분산 비례는 풀이마다 받는 스칼라다. 새 파라미터의 기본값은 모두 끔이고, 그때 코어는 E1-F01 과 같은 문제를 푼다 | 사용자 결정 (2026-10-01). $\Sigma_p$ 는 계획기 스레드에 있다 (`CovarianceSnapshot`, model world 로 회전된 6×6) — 상수 가중은 공분산의 token 이 어긋날 때의 대체값이다. 출하 `planner.unc.sigma_trk` 는 0.0 이고 v1 의 오차 예산 게이트가 쓰므로 가중의 하한은 MPC 의 자기 파라미터로 둔다. 접근축의 사영 형태 $(I-a_da_d^\top)z_C$ 는 90° 에서 기울기가 0 이고 180° 에 거짓 최소가 있다. $v_{rel,allow}$ 의 값이 어디에도 없어 $s_v$ 를 켜는 것은 E1-F10 이 정한다 | MD-46 의 "$\Sigma_p$ 가 단일 팔 경로에 없으면 상수 가중" (있다 — 배선은 E1-F08), [#660 결정 목록](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5913561760) 6 의 접근축 형태 | 2026-10-01 |
| MD-53 | 상대속도의 목표는 코어가 고르지 않는다. 코어는 $\hat v_b$ 와 스칼라 $\gamma_{ref}$ 를 받아 비용의 목표를 $\gamma_{ref}\hat v_b$ 로 두고, $s_v$ 의 행은 늘 $\hat v_b$ 기준으로 건다. E1-F08 의 기본은 $\gamma_{ref}=1$ 과 방향별 가중 (formulation §1.3 — $\gamma$ 는 해의 결과) 이고, plan 의 $\gamma_f$ 를 넣는 것은 E1-F10 의 선택지다. 코어는 해의 $\gamma$ 를 결과에 적는다 | 사용자 결정 (2026-10-01). $\gamma_{ref}=1$ 에서 손의 진행 방향 속도는 $w_\parallel$ 의 당김과 jerk · 한계가 맞서는 곳에서 정해진다. v1 탐색의 후보 게이트 (정지 거리 · 충격량 · 오차 예산) 는 $\gamma_f$ 로 평가한 것이라 해의 $\gamma$ 와 다를 수 있다 (MD-46 의 한계의 연장) — 그래서 기록한다 | §4 미결의 "상대속도의 목표" | 2026-10-01 |
| MD-54 | E1-F07 의 격자는 포구 전 $\Delta_a$ 0.1 s (노드 최대 6, 노드마다 jerk 블록) + 정지 구간 $\Delta_s$ 0.05 s × 7 (블록 1 · 1 · 2 · 3) 이다 — 노드 최대 13. 측정 (7 자유도 p99, §8): 같은 격자점 재풀이 8.7 ms 는 MD-51 의 임계 (10) 안이고, 첫 풀이 16.0 ms (임계 12) 와 격자점 전진 17.9 ms (임계 10) 는 넘는다. **초과를 알고 정한 값이다.** 토크 행을 정지 구간에만 거는 것, solver 의 warm start 를 격자점 전진에 잇는 것, 임계를 다시 정하는 것은 하지 않는다. 격자는 코어의 파라미터이고 코어의 기본값은 그대로다 (포구 전 노드 0) — 값의 배선은 E1-F08 | 사용자 결정 (2026-10-01, [#660 측정 코멘트](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5924188945) 의 안 1). 측정한 네 후보 (A1 · B1 · A2 · B2) 가 모두 임계를 넘었고, 포구 전 간격을 넓히는 것은 formulation 과 코어를 바꾸지 않는다. 대가: (1) 효력 시각이 최대 0.1 s 늦다. (2) 노드 사이의 속도가 box 를 6 % 넘는다 ($\eta_v$ 0.95 에서 실제 한계의 약 1 %). (3) 간격이 둘이라 payload 의 간격 필드 (`dt_ns` 하나) 와 균일 간격 샘플러가 바뀐다 — 노드 수는 용량 24 안이다 (E1-F08 · F09). (4) MD-51 의 환산 (sim 은 개발 PC 의 1.5 배) 으로 전진 재계획은 sim 에서 약 27 ms 다. 주기 33.3 ms 안이지만 출하 `budget_s` 20 ms 를 넘는다. 첫 풀이는 약 24 ms 로, 탐색 (sim p99 8.6 ms) 과 같은 wake 에 두면 한 주기에 닿는다. 예산과 wake 배치는 E1-F08 이 정한다 | MD-51 의 "격자의 확정은 측정 뒤 사용자가 한다", §4 미결의 "격자의 값" | 2026-10-01 |

MD-7 의 귀결: 토크 행은 직전 해에서의 역동역학 값과 그 미분으로 선형화한다 (MD-13). 그래서 단일 팔 문제도 계획기 스레드에서 동역학 모델을 평가하고, 주기마다 선형화를 다시 한다.

미결 — 해당 feature 의 spec 에서 정한다:

- E1-F08 – F09: 첫 구간 · 재계획 · RT 동작 ([#660 결정 목록 8 – 11](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5913561760)), 간격이 둘인 payload 와 샘플러, 계획기의 예산과 wake 배치 (MD-54). 코어에서 넘어온 것은 [#661 통합 코멘트](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925464681)
- E1-F10: 비열등 한계 · N · 튜닝 seed · 반복 상한 (튜닝 전에, MD-50)
- E1-F06: `mpc` 를 기본값으로 바꿀지
- E3-F01: 포구 후보 선택을 MPC 로 옮길지 (MD-46 의 편차) 와 계획기 interface (ARCH-3)
- E3-F05: G1 통합의 형태 — 단일 팔은 `DemoCatchingController` 의 `mode: mpc` 로 돈다 (MD-46)

## 5. 출발점 (2026-09-29 코드 대조)

| 영역 | 확인한 사실 | 계획에 주는 영향 |
|---|---|---|
| DECEL | `rtc_controllers` `catching/decel_target.hpp` 의 `EvaluateDecelTarget` 이 TCP 직선 등감속 목표를 만들고, soft-catch DS → CLIK 을 거쳐 RT tick 안에서 매번 1-step 으로 푼다 | horizon 도, 관절 가속 · jerk 최적화도 없다. `mpc` planner 가 대체한다 (E1 — MD-45 부터는 포구 전 soft-catch DS 도) |
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
| armature | G1 URDF 에는 없고 MJCF 에만 있다 | 제어에 쓰지 않는다 (MD-25). E1-F01 코어의 벡터 입력은 분석과 테스트용으로 남는다 |
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
| E0-F04 | [#647](https://github.com/hyujun/rtc-framework/issues/647) | 예측 격자 sweep — v1 계획기 기준선 (horizon 0.75 s · 1.0 s) | E0-F01 | 완료 (2026-09-30) — §8 |

### E1. 단일 팔 MPC — [#621](https://github.com/hyujun/rtc-framework/issues/621) · 필수 · 최우선

대상 로봇: ur5e_p1b · iiwa7_leap. `mode: mpc` 의 팔 기준을 APPROACH 부터 정지까지 MPC 가 만든다 — formulation §1.3 에서 dual arm · waist 항을 뺀 구성이다 (MD-45 · MD-46). E1-F01 – F04 는 정지 구간만 다룬 첫 단계다. 게이트: **G-1** (§1).

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E1-F01 | [#627](https://github.com/hyujun/rtc-framework/issues/627) | jerk 입력 condensed QP 코어 (토크 제약 행 · slack) | E0-F03 | 완료 ([#655](https://github.com/hyujun/rtc-framework/pull/655)). 할당 0 은 코어 경로만 (MD-22) |
| E1-F02 | [#628](https://github.com/hyujun/rtc-framework/issues/628) | 관절 노드 payload (`DecelPlanSnapshot`, MD-27) + RT 샘플러 (관절 기준에서 FK) | E1-F01 | 완료 ([#656](https://github.com/hyujun/rtc-framework/pull/656)). RT tick 배선은 E1-F04 (MD-32) |
| E1-F03 | [#629](https://github.com/hyujun/rtc-framework/issues/629) | 계획기 스레드 통합 — 정지 구간 선계산 | E1-F02 | 완료 ([#656](https://github.com/hyujun/rtc-framework/pull/656)). 출하는 꺼짐 — §8 |
| E1-F04 | [#630](https://github.com/hyujun/rtc-framework/issues/630) | L7 DECEL 전환 — MPC 궤적 추종 (법칙은 configure 에서 하나, MD-44) | E1-F03 | 구현 완료 ([#658](https://github.com/hyujun/rtc-framework/pull/658)) — 결정 MD-34 – MD-44, 측정 §8. sim smoke 에서 `mode: mpc` 진입 20/20 abort 라 이슈는 열어 둔다 → MD-45, 마지막 항목은 E1-F09 에서 닫는다 |
| E1-F07 | [#660](https://github.com/hyujun/rtc-framework/issues/660) | 단일 팔 MPC 코어 — APPROACH–정지 격자, 포구 항 (위치 · 접근축 · 상대속도), 항 단위 조립 (MD-46 · MD-49) | E1-F04 | 완료 (2026-10-01, [#666](https://github.com/hyujun/rtc-framework/pull/666)) — 결정 MD-51 – MD-54, 측정 §8. 격자는 MD-54 이고 첫 풀이와 격자점 전진의 계산 시간은 임계를 넘은 채다 (E1-F08 의 예산이 받는다) |
| E1-F08 | [#661](https://github.com/hyujun/rtc-framework/issues/661) | 계획기 — TRACKING – DECEL 의 MPC 풀이, 첫 구간 · 예산, 재계획 ($x_0$ 경로 (i) 일반화), 격자의 배선과 간격이 둘인 payload (MD-54) | E1-F07 | 착수 가능 |
| E1-F09 | [#662](https://github.com/hyujun/rtc-framework/issues/662) | L7 — RT 가 APPROACH – HOLD 를 MPC 구간으로 추종, DECEL 진입은 연속 (E-8). 노드별 간격을 읽는 샘플러 (MD-54) | E1-F08 | 대기 |
| E1-F05 | [#631](https://github.com/hyujun/rtc-framework/issues/631) | 로그 · plot_rtc_log · demo_controller_gui — 포구 항 열 · APPROACH 구간 포함. `planner_events` 의 decel 열이 `rtc_tools` 의 목록에 없어 `main` 에서 테스트 하나가 실패한다 ([#631 코멘트](https://github.com/hyujun/rtc-framework/issues/631#issuecomment-5925477065)) | E1-F09 | 대기 |
| E1-F10 | [#663](https://github.com/hyujun/rtc-framework/issues/663) | mpc planner 튜닝 — closed_form 대비 성공률 비열등 (MD-50) | E1-F05 | 대기 |
| E1-F06 | [#632](https://github.com/hyujun/rtc-framework/issues/632) | A/B 성능 시험 — 게이트 G-1 판정 (closed_form 대 mpc planner) | E0-F02, E1-F10 | 대기 |

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

### E3. MPC dual arm · waist 확장 — [#623](https://github.com/hyujun/rtc-framework/issues/623) · 필수 (E2 뒤)

같은 MPC 에 dual arm · waist 항을 더한다 (MD-46 · MD-47). 착수 조건: E2 (g1_p1b 준비) 와 E1-F07 (코어 — 완료). G-1 과 무관하다. 첫 단계는 `Decel*` 이름의 rename refactor 다 (MD-48). sim 전용. 게이트: G1 sim 에서 포구 시행이 돌고 성공률 · solve time p99 가 보고된다. 기존 두 로봇 회귀 없음 — 단일 팔 구성 (더한 항의 가중 0) 의 해가 불변이다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E3-F01 | [#640](https://github.com/hyujun/rtc-framework/issues/640) | 포구 후보 선택을 MPC 로 (바깥 루프 — MD-46 의 편차를 닫는다) · 계획기 interface 도입 (ARCH-3). 포구 항 자체는 E1-F07 로 옮겼다 (MD-45) | E1-F08 | 대기 |
| E3-F02 | [#641](https://github.com/hyujun/rtc-framework/issues/641) | 전신 항을 E1 코어에 추가 — 왼팔 rest · waist 억제 · 각운동량 · waist 토크 행 · counter-swing, 관절군별 move blocking | E1-F07, E2-F01 | 대기 |
| E3-F03 | [#642](https://github.com/hyujun/rtc-framework/issues/642) | 충돌 제약 — capsule 모델 (신규 코어) 과 MPC 제약 행 (자기충돌 · 공–왼팔 거리) | E3-F02 | 대기 |
| E3-F04 | [#643](https://github.com/hyujun/rtc-framework/issues/643) | MPC ↔ CLIK 계약의 전신 확장 — payload 관절 용량 (8 → G1 $n$ 17), 왼손 FK, 한계 여유. 단일 팔 계약은 E1-F02 · MD-36 에 있다 | E1-F08, E2-F04 | 대기 |
| E3-F05 | [#644](https://github.com/hyujun/rtc-framework/issues/644) | G1 통합 — supervisor · 손 시퀀서 · vision sim profile (`mode: mpc` 의 확장) | E3-F03, E3-F04, E2-F05 | 대기 |
| E3-F06 | [#645](https://github.com/hyujun/rtc-framework/issues/645) | 로그 · plot_rtc_log · demo_controller_gui — dual arm 열 (항별 비용 분해는 E1-F05) | E3-F05 | 대기 |
| E3-F07 | [#646](https://github.com/hyujun/rtc-framework/issues/646) | 평가 — G1 sim 포구 시행 · 예측 격자 sweep · waist · 왼팔 고정 대조 · 기존 로봇 회귀 | E3-F06, E0-F04 | 대기 |

각 feature 의 범위와 Done when 은 이슈 본문에 있다.

### 브랜치 계획

feature 28개를 브랜치 21개로 묶는다. 브랜치 하나가 PR 하나다.

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
| `docs/mpc-dualarm-plan` (완료) | E0-F03 | 문서 4개 (계획, formulation, README, v1 계획 개정). 가장 먼저 올린다 — 이슈의 문서 링크가 이 merge 로 살아난다 |
| `chore/ws-first-build-path` (완료) | E0-F01 | 빌드 절차 문서 (MD-17 — 빌드 스크립트와 `package.xml` 은 고치지 않는다). 다른 feature 의 선행이라 단독으로 빨리 닫는다 |
| `feat/catching-baseline-grid-sweep` (완료) | E0-F02, E0-F04 | 둘 다 `catching_sim_trials` 로 시행을 모으고 `rtc_tools` 의 분석을 확장한다. 같은 시행 도구와 같은 host 조건을 쓴다. **나누는 조건**: sweep 의 조건별 설정이 출하 config 를 건드리게 되면 E0-F04 를 분리한다 |

E0-F04 의 ball_perception 쪽 JSON 갱신은 그 저장소 (hyujun/ball_perception) 의 브랜치와 PR 로 따로 한다.

**E1**

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `feat/catching-decel-mpc-core` (완료, [#655](https://github.com/hyujun/rtc-framework/pull/655)) | E1-F01 | 신규 수치 코어. code review 단위 |
| `feat/catching-decel-mpc-plan-path` (완료, [#656](https://github.com/hyujun/rtc-framework/pull/656)) | E1-F02, E1-F03 | payload · RT 샘플러와 계획기 스레드 통합. 계획기가 게시하고 RT 가 읽는 한 경로의 양 끝이라 함께 있어야 연속성을 시험할 수 있다. **나누는 조건**: 새 스레드가 필요해지면 (E-7) E1-F03 을 분리한다 |
| `feat/catching-decel-mpc-l7` (완료, [#658](https://github.com/hyujun/rtc-framework/pull/658)) | E1-F04 | E-8 (Critical). `[CONCERN]` 컨펌과 security review 의 범위를 이 PR 로 한정한다 |
| `feat/catching-mpc-approach-core` (완료, [#666](https://github.com/hyujun/rtc-framework/pull/666)) | E1-F07 | 신규 수치 항 (포구 항 선형화). code review 단위 |
| `feat/catching-mpc-approach-plan-path` | E1-F08 | 계획기 스레드와 payload 의 간격 (MD-54). **나누는 조건**: 새 스레드가 필요해지면 (E-7) |
| `feat/catching-mpc-approach-l7` | E1-F09 | E-8 (Critical). `[CONCERN]` 컨펌과 security review 의 범위를 이 PR 로 한정한다 |
| `feat/catching-decel-mpc-tooling` | E1-F05 | 로그 · plot · GUI |
| `exp/catching-mpc-tuning` | E1-F10 | 튜닝. 채택한 config 만 YAML 로 넣고 실험 overlay 와 원자료는 repo 밖에 둔다. **나누는 조건**: 결과 기록이 커지면 `docs/` 브랜치로 나눈다 |
| `docs/catching-decel-mpc-g1` | E1-F06 | 게이트 G-1 의 판정과 결과 기록. 기본값 변경이 결정되면 그 변경은 별도 브랜치다 |

**E2**

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `feat/g1-p1b-bringup` | E2-F01, E2-F02, E2-F03 | 자산, config · launch, `demo_joint_controller` 의 G1 구동. 셋이 모여야 G1 sim 에서 관절 추종을 확인할 수 있다. **나누는 조건**: E2-F03 이 `DemoJointController` 의 일반화 (공용 코드 변경) 를 요구하면 분리한다 — 기존 로봇의 회귀를 그 PR 만으로 판정한다 |
| `feat/tsid-clik-multiframe` | E2-F04 | `rtc_tsid` public API 변경. 기존 소비자 둘의 기능 동등성이 성공 기준이다 |
| `feat/demo-dualarm-controller` | E2-F05 | 신규 controller. Sprint Contract = spec |
| `feat/g1-dualarm-tooling` | E2-F06, E2-F07 | GUI 와 plot. 둘 다 `demo_dualarm_controller` 의 출력을 소비한다 |

`feat/tsid-clik-multiframe` 은 선행이 E0-F03 뿐이라 E1 과 병행할 수 있다.

**E3** (E2 와 E1-F07 뒤 — MD-47). 첫 브랜치는 `Decel*` 이름의 rename refactor 다 (MD-48, feature 가 아니다 — 기능 동등성이 성공 기준)

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `feat/catching-mpc-candidate-select` | E3-F01 | 계획기 interface 도입 (ARCH-3) 과 MPC 의 후보 선택. **나누는 조건**: interface 도입만으로 v1 계획기의 diff 가 크면 interface 를 먼저 분리한다 |
| `feat/catching-mpc-wholebody` | E3-F02, E3-F04 | 전신 항과 MPC ↔ CLIK 계약. 둘 다 활성 관절을 전체로 넓히는 작업이고, 계약의 sanity check 가 전신 해를 입력으로 쓴다 |
| `feat/collision-capsule-core` | E3-F03 | 신규 수치 코어. code review 단위 |
| `feat/g1-catch-controller` | E3-F05 | supervisor · 손 시퀀서 통합. E-STOP 경로를 건드리면 E-8 |
| `feat/g1-catch-tooling-eval` | E3-F06, E3-F07 | 로그 · plot · GUI 와 평가. 평가가 새 로그 컬럼을 쓴다. **나누는 조건**: 평가 결과가 설계 결정을 바꾸면 결과 기록을 `docs/` 브랜치로 분리한다 |

**순서.**

| 단계 | 브랜치 | 병행 가능 |
|---|---|---|
| 1 | `docs/mpc-dualarm-plan` (완료) | — |
| 2 | `chore/ws-first-build-path` (완료) | — |
| 3 | `feat/catching-baseline-grid-sweep` (완료), `feat/catching-decel-mpc-core` (완료) | `feat/tsid-clik-multiframe` |
| 4 | `feat/catching-decel-mpc-plan-path` (완료) → `-l7` (완료) → `feat/catching-mpc-approach-core` (완료) → **`-plan-path` (다음)** → `-l7` → `feat/catching-decel-mpc-tooling` → `exp/catching-mpc-tuning` → `docs/catching-decel-mpc-g1` | `feat/g1-p1b-bringup` |
| 5 | `feat/demo-dualarm-controller` → `feat/g1-dualarm-tooling` | — |
| 6 | rename refactor (MD-48) → E3 의 다섯 브랜치 (E2 와 E1-F07 뒤, MD-47) | — |

병행은 서로 다른 패키지를 고치는 브랜치끼리만 한다. E1 이 최우선이므로 (MD-8 · MD-47), 병행할 여력이 없으면 단계 4 의 E1 브랜치를 먼저 한다.

## 7. 게이트 · escalation

| Feature | 사유 | 효력 |
|---|---|---|
| E1-F04 | L7 전이 동작 변경 (E-8) | Critical — 착수 전 `[CONCERN]` 과 컨펌, 완료 후 security review |
| E1-F09 | L7 전이 동작 변경 — APPROACH – HOLD 추종 (E-8) | Critical — 착수 전 `[CONCERN]` 과 컨펌, 완료 후 security review |
| E3-F05 | E-STOP 경로를 건드리면 E-8 | Critical |
| E1-F03 | 새 스레드가 필요해지면 E-7 | 발동 안 함 — decel 계획기는 기존 계획기 스레드 안에서 돈다 (#656) |
| E2-F04 | `rtc_tsid` public API 변경, 기존 소비자 둘 | code review, 기능 동등성이 성공 기준 |
| E3-F01 | 계획기 interface 신설 (ARCH-3) | code review |
| E1-F01, E1-F07, E3-F03 | 신규 수치 코어 · 항 (100+ 줄) | code review |
| E3 착수 | `Decel*` rename refactor (MD-48) — public YAML key | 별도 PR, 기능 동등성이 성공 기준 |
| E2-F05 | 신규 controller | Sprint Contract = spec |
| E0-F04 | ball_perception 은 별도 저장소다 | 그쪽 변경은 그 저장소의 절차를 따른다. profile 은 그쪽이 소유한다 (MD-18) — 컨트롤러 YAML 과의 정합을 보는 자동 검사는 없으므로 양쪽 값을 함께 확인한다 |

공통 규칙:

- 기존 test assertion 은 약화하지 않는다 (PROC-6).
- RT 경로 (`Compute`, 샘플러, CLIK) 는 할당 0 을 게이트로 확인한다.
- E1 에서 closed_form planner (v1 soft-catch DS + closed-form DECEL) 가 기본값으로 남고, mpc planner 는 YAML (`supervisor.decel.mode: mpc`) 로 켠다.
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

unit 재실행 (v1 D-S8-17 규칙, 같은 seed): `p1b_601_a` (RTF 0.943 시행 — 원본 35/50, 재실행 37/50), `p1b_603_b` (host-watch 중단 1회), `leap_704_b` (추정기 미활성 1회, host-watch 중단 1회).

**host 부하의 출처.** 수집 중 에이전트의 턴이 끝날 때마다 Stop hook 이 `rtc_tools` 의 `colcon test` (2–3 분) 를 시작했다. hook 은 workspace 의 simulator 가 돌면 빌드 · 테스트를 미루지만, unit 과 unit 사이에는 simulator 가 몇 초 동안 없고 턴은 그 틈에서 끝났다. 판정에 쓴 16 unit 중 8 개가 이 테스트와 겹쳤고, 2 개 (`leap_701_a`, `p1b_602_a`) 는 에이전트가 직접 돌린 분석 · 테스트와 겹쳤다. `p1b_603_b` 의 첫 시도는 러너가 그 `colcon test` 를 원인 후보로 기록하며 중단했다. 겹친 unit 도 판정 규칙 (시행별 RTF ≥ 0.95) 은 통과했다 — `ur5e_p1b` 는 겹친 4 unit 140/200, 겹치지 않은 4 unit 147/200 이다. `leap_704_b` 의 두 번째 중단 (RTF 0.826) 시각에는 빌드 · 테스트가 없었고 원인은 확인하지 못했다.

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

**DECEL 지표 (시행별 값의 p50 / p95 / max).** 창은 첫 DECEL tick 부터 측정 catch frame 이 task pose 에서 정지한 첫 tick 까지다 — 선속도 0.02 m/s 미만이고 접근축의 각속도 0.2 rad/s 미만인 상태가 0.05 s 이어질 때 (사용자 결정 2026-09-30). 접근축 둘레의 roll 은 task 가 아니라 판정에 넣지 않는다. 미분은 5 tick 평균이다. 정의와 도구: [rtc_tools README](../../rtc_tools/README.md) `catching_decel`.

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
| 접근축 각속도 피크 [rad/s] | 0.24 / 0.27 / 1.79 | 0.34 / 0.62 / 1.76 |
| 정지 시점의 frame 전체 각속도 (roll 포함, 참고) [rad/s] | 0.039 / 0.050 / 0.879 | 0.080 / 0.158 / 0.501 |
| RETREAT 전에 task pose 가 정지한 시행 | 399 / 400 | 372 / 377 |

- `iiwa7_leap` 의 23 시행은 DECEL 에 들어가지 않았다 (abort 등). 성공률의 분모에는 실패로 남는다.
- 팔은 DECEL mode 가 끝난 뒤에도 0.1–0.4 s 더 움직인다. 실제 정지 거리는 닫힌식보다 p50 기준 p1b 1.4 배, leap 1.9 배다. MPC 와의 비교는 mode 길이가 아니라 위 창으로 한다.
- 최댓값은 소수의 시행이 만든다. 측정 가속 피크가 25 rad/s² 를 넘는 시행은 로봇마다 2 개이고 (99 백분위 p1b 19.5 · leap 11.6), 대부분 손이 RETREAT 전에 정지하지 못한 시행이다. `ur5e_p1b` 의 토크 위반 1 건 (`p1b_603_a` 30번, 비 1.14) 도 그중 하나다. 원인은 조사하지 않았다.
- 성공 · 실패 시행의 지표 차이는 작다 (측정 가속 p50 p1b 15.2 대 14.2, leap 9.8 대 9.2).
- 포구 컨트롤러는 manipulability 를 올리는 null-space 운동을 더하므로 task pose 가 멈춘 뒤에도 관절은 움직인다. 관절 속도 기준 (0.01 rad/s) 으로는 `ur5e_p1b` 400 중 308, `iiwa7_leap` 377 중 2 시행만 정지다. 그래서 정지는 task pose 로 판정한다. 접근축 조건을 더해도 위치만 본 판정과 창이 같았다 (두 로봇 모두 위 표의 값 불변) — 이 자료에서는 위치가 늦게 멈춘다.

원자료와 실험 도구는 repo 밖에 있다. 값은 모두 sim 값이며 v1 의 S10 실기 식별이 바뀌면 다시 잰다.

### E0-F04 — 예측 격자 sweep, v1 계획기 기준선 (2026-09-30, [#647](https://github.com/hyujun/rtc-framework/issues/647))

formulation §1.7 의 여덟 조건을 v1 계획기로 잰 값이다. E3-F07 의 MPC 계획기 sweep 이 쓰는 기준선이다.

**측정 조건.**

| 항목 | 값 |
|---|---|
| 저장소 | rtc-framework `d3e921ac`, ball_perception `9005aab` (sim 용 고정 사본) |
| 추정기 profile | L-50: ball_perception 의 설치본 `sim_profile.catching.json` (schema 0.2, E0-F02 의 사본과 바이트 동일, MD-20). 그 밖의 조건: 그 파일에서 `prediction` 세 키만 바꾼 저장소 밖 사본 |
| overlay | L-50: 출하 `catch_lead_on`. 그 밖: `catch_lead_on` 의 잎 전부 + `prediction.dt_expected` = `planner.slice.dt` = step, `io.n_min` = ⌈0.51 / step⌉ + 1. 둘 다 저장소 밖 |
| 컨트롤러 | 격자 세 키 밖에는 E0-F02 와 같다 (`io.horizon_min` 0.51, `planner.slice.t_max` 0.95, `planner.budget_s` 0.020, `max_ik` 8) |
| 투척 | `--dist s35b`, tennis, seed p1b 601–602 · leap 701–702, unit 당 50 발. 조건 × 로봇마다 100 발 |
| 순서 | seed 블록 둘, 블록마다 조건 순서를 고정 난수로 섞고 조건마다 p1b → leap. 32 unit |
| host | 개발 PC, 기동 시 load average 1.2–3.0. unit 은 `--host-watch abort` (RTF 하한 0.95), 드라이버는 `with_verify_hold.sh` |
| 유효성 | 무효 0, host-watch 중단 0, `rtf_trial_min` < 0.95 인 시행 0. unit 재실행 2 (투척 전 rig 실패: 추정기 `change_state` 응답 timeout 1, 기동 로그 글자 깨짐으로 드라이버 검사 오판 1) |
| 조건 적용 | 32 unit 모두 컨트롤러 미러의 격자 세 키가 조건값, 받은 메시지의 점 수 (diag `input_n`) 가 한 값이고 조건의 점 수와 같음 (사후 확인, 틀린 조건으로 판정하면 32/32 가 걸리는 양성 대조 포함) |

판정 규칙은 시행 전에 고정했다: 게이트가 없는 기준선 측정이다. "격자의 영향이 있다" 는 조건 대 L-50 의 McNemar p 를 로봇별로 Holm 보정 (7 비교) 해 0.05 아래일 때만 쓴다. 이 N (100 쌍, ψ ≈ 0.25) 에서 검출할 수 있는 차이는 약 0.14 (Holm 최악 0.18) 이고, 그보다 작은 차이는 이 sweep 이 말하지 못한다.

**성공률과 같은 투척의 비교 (L-50 대비, 로봇당 100 쌍).**

| 조건 | ur5e_p1b 성공 | 차이 [95 %] | McNemar p · Holm | iiwa7_leap 성공 | 차이 [95 %] | McNemar p · Holm |
|---|---|---|---|---|---|---|
| 1.0 s · 20 점 (50 ms, 출하) | 71/100 [0.61, 0.79] | — | — | 54/100 [0.44, 0.63] | — | — |
| 1.0 s · 25 점 (40 ms) | 60/100 [0.50, 0.69] | -0.11 [-0.21, -0.01] | 0.061 · 0.430 | 45/100 [0.36, 0.55] | -0.09 [-0.18, -0.00] | 0.078 · 0.392 |
| 1.0 s · 32 점 (31.25 ms) | 74/100 [0.65, 0.82] | +0.03 [-0.06, +0.12] | 0.678 · 1.000 | 58/100 [0.48, 0.67] | +0.04 [-0.05, +0.13] | 0.503 · 1.000 |
| 1.0 s · 40 점 (25 ms) | 62/100 [0.52, 0.71] | -0.09 [-0.19, +0.01] | 0.136 · 0.816 | 47/100 [0.38, 0.57] | -0.07 [-0.17, +0.03] | 0.265 · 1.000 |
| 0.75 s · 15 점 (50 ms) | 78/100 [0.69, 0.85] | +0.07 [-0.02, +0.16] | 0.210 · 0.925 | 50/100 [0.40, 0.60] | -0.04 [-0.12, +0.04] | 0.481 · 1.000 |
| 0.75 s · 24 점 (31.25 ms) | 75/100 [0.66, 0.82] | +0.04 [-0.05, +0.13] | 0.523 · 1.000 | 43/100 [0.34, 0.53] | -0.11 [-0.21, -0.01] | 0.043 · 0.260 |
| 0.75 s · 30 점 (25 ms) | 63/100 [0.53, 0.72] | -0.08 [-0.19, +0.03] | 0.215 · 0.925 | 50/100 [0.40, 0.60] | -0.04 [-0.12, +0.04] | 0.454 · 1.000 |
| 0.75 s · 40 점 (18.75 ms) | 63/100 [0.53, 0.72] | -0.08 [-0.18, +0.02] | 0.185 · 0.925 | 39/100 [0.30, 0.49] | -0.15 [-0.24, -0.06] | 0.004 · 0.029 |

**같은 간격에서 horizon 만 다른 쌍 (0.75 s − 1.0 s, Holm 은 세 쌍 안에서).**

| 간격 | ur5e_p1b 차이 [95 %] · Holm | iiwa7_leap 차이 [95 %] · Holm |
|---|---|---|
| 50 ms | +0.07 [-0.02, +0.16] · 0.630 | -0.04 [-0.12, +0.04] · 0.961 |
| 31.25 ms | +0.01 [-0.08, +0.10] · 1.000 | -0.15 [-0.24, -0.06] · 0.012 |
| 25 ms | +0.01 [-0.09, +0.11] · 1.000 | +0.03 [-0.06, +0.12] · 0.961 |

**예측 메시지 (컨트롤러가 받은 것), 예측 오차, 계획기.**

| 조건 | 크기 [B] | 수신 간격 p50 / p95 [ms] (p1b · leap) | 간격 ≥ 100 ms 비율 (p1b · leap) | `pred_mm` p50 / p95 (p1b · leap) | 계획기 `search_us` p99 [ms] (p1b · leap) | 후보 max (p1b · leap) |
|---|---|---|---|---|---|---|
| L-50 | 7680 | 34 / 68 · 34 / 68 | 0.014 · 0.011 | 36 / 74 · 24 / 41 | 7.2 · 8.5 | 12 · 15 |
| L-40 | 9600 | 34 / 100 · 34 / 100 | 0.028 · 0.025 | 34 / 78 · 23 / 48 | 8.7 · 10.2 | 14 · 19 |
| L-31 | 12288 | 34 / 100 · 34 / 100 | 0.071 · 0.073 | 32 / 62 · 24 / 48 | 9.8 · 10.2 | 18 · 24 |
| L-25 | 15360 | 34 / 102 · 34 / 132 | 0.082 · 0.093 | 35 / 73 · 23 / 58 | 11.6 · 13.2 | 22 · 29 |
| M-50 | 5760 | 34 / 68 · 34 / 68 | 0.007 · 0.005 | 26 / 48 · 18 / 35 | 7.1 · 8.1 | 7 · 10 |
| M-31 | 9216 | 34 / 100 · 34 / 100 | 0.025 · 0.021 | 39 / 63 · 25 / 46 | 10.2 · 13.5 | 10 · 16 |
| M-25 | 11520 | 34 / 100 · 34 / 100 | 0.048 · 0.037 | 31 / 76 · 25 / 50 | 10.9 · 13.5 | 12 · 20 |
| M-19 | 15360 | 34 / 100 · 36 / 100 | 0.067 · 0.070 | 27 / 49 · 24 / 46 | 10.6 · 13.5 | 16 · 26 |

- **성공률.** `ur5e_p1b` 는 어느 조건도 L-50 과 다르다고 말할 수 없다 (Holm p ≥ 0.43). `iiwa7_leap` 은 **M-19 (0.75 s · 40 점) 가 L-50 보다 0.15 낮다** (Holm p 0.029) — 사전 규칙상 영향이 있다고 쓰는 유일한 비교다. 같은 로봇의 M-31 − L-31 도 −0.15 (세 쌍 안의 Holm p 0.012) 이지만 50 ms · 25 ms 쌍에서는 horizon 의 차이가 보이지 않는다. 원인은 조사하지 않았다.
- **촘촘한 격자가 성공률을 올린다는 증거는 없다.** 두 로봇 모두 가장 높은 값은 50 ms 나 31.25 ms 조건이고, 25 ms 이하는 모든 비교에서 차이의 점추정이 음수다.
- **발행 주기.** 수신 간격의 p50 은 모든 조건에서 34 ms (30 Hz) 로 유지된다. 꼬리는 메시지 크기와 함께 늘어난다 — 간격이 컨트롤러의 `io.t_stale` 0.10 s 이상인 비율이 7.7 kB 이하에서 0.5–1.4 %, 15 kB 에서 6.7–9.3 % 다. 비행 중 `BALL_STALE` 로 이어진 시행은 조건마다 0–5 / 100 이라 성공률의 차이를 설명하지 못한다. 간격이 추정기의 계산에서 오는지 전송에서 오는지는 가르지 않았다.
- **예측 위치 오차.** `pred_mm` 은 첫 COMMITTED tick 의 계획 포구점과 그 시각 공의 참값 사이 거리다. commit 은 포구 전 `T_freeze` (p1b 0.37 s, leap 0.19 s) 에 있으므로 이 값은 horizon 끝이 아니라 그 lead 에서의 오차다. horizon 0.75 s 와 1.0 s 사이에 일관된 차이는 없다 (같은 간격의 p50 차이 −10 – +7 mm). `tc_axis` 가 `shifted` 인 시행은 없었다.
- **계획기.** 후보 수는 간격에 반비례해 늘고 (p1b 12 → 22, leap 15 → 29), 31.25 ms 이하에서 한 주기의 IK 가 상한 `max_ik` 8 에 닿는다. 계산 시간 p99 는 7.1 – 13.5 ms 로 예산 20 ms 안이고 예산 초과는 0 이다.
- **profile 이전의 확인.** L-50 과 E0-F02 복제 a · b 의 같은 투척 불일치율은 p1b 0.27 · 0.25, leap 0.19 · 0.21 로 E0-F02 의 복제 간 ψ 구간 안이다. 성공률 차이는 −0.07 – +0.03 이고 모든 구간이 0 을 포함한다.
- 조건별 원자료와 실험 도구는 repo 밖에 있다. 분석 도구는 `rtc_tools` `catching_grid_sweep` 이다. 값은 모두 sim 값이다.

### E1-F01 — decel MPC 코어 계산 시간 (2026-09-30, [#627](https://github.com/hyujun/rtc-framework/issues/627))

`test_catching_decel_mpc` 의 정보용 측정이다. Release, 개발 PC, 무작위 진입 상태 200 개, 실제 6 · 7 자유도 팔 URDF, $N_s$ 12 · $\Delta_s$ 0.05 s · 블록 6 (MD-21 의 테스트 기본값). cold 는 기준이 없는 첫 주기 (pre-solve + 본 solve), warm 은 직전 해를 20 ms 밀어 기준으로 쓴 다음 주기다. 예산 판정은 E1-F03 이 한다.

| 경우 | 한 주기 p99 [ms] | 본 solve 반복 p50 / p99 | 실패 |
|---|---|---|---|
| 6 자유도 cold | 6.4 | 10 / 14 | 0 / 200 |
| 6 자유도 warm | 2.6 | 2 / 2 | 0 / 200 |
| 7 자유도 cold | 9.4 | 10 / 15 | 0 / 200 |
| 7 자유도 warm | 3.2 | 2 / 2 | 0 / 200 |
| 7 자유도 warm, $w_\perp$ 켬 | 4.0 | 2 / 8 | 0 / 200 |
| 6 자유도 warm, 토크 행 활성 | 8.7 | 2 / 86 | 5 / 200 |

- **대부분이 QP 풀이다.** 노드 12 개의 토크 선형화는 p50 40 µs (6) · 50 µs (7), condensing 은 3–4 µs 다. Kronecker 구조 조립이 조밀 조립 (35–55 µs) 을 대체했지만 한 주기에서 차지하는 몫은 작다.
- **solver 설정이 시간을 정한다.** 기본 설정 (ProxQP 백엔드 자동 선택, 자명 문제의 전처리기 고정, slack 벌점 1e3) 에서는 cold p99 가 33 ms (6) · 55 ms (7) 였고 거짓 "실행 불가능" 판정이 섞였다. 백엔드 PrimalDualLDLT 고정, 매 풀이 전처리기 재계산, slack 벌점 10 으로 위 표가 됐다. 벌점 10 은 표본 150 개에서 exact 했다 (1 에서는 62 건 중 1 건이 아니었다).
- **토크 행이 걸리면 warm 주기의 약 2.5 % 가 거짓 "실행 불가능" 판정으로 실패한다.** 호출자는 직전 계획을 유지한다 (MD-11).
- **할당.** 코어 경로는 0 이고, ProxQP 는 Solve 마다 7–18 회 할당한다 (MD-22, #654).
- 재평가 토크 (pre-solve 경로 RTI 1 회, 1 kHz 표본, armature 포함) 의 최대 $|\tau|/\tau_{\max}$ 는 0.70 으로 $\eta'_\tau$ 0.7 과 같다.
- **주기 사이 속도 표류는 흡수된다.** x₀ 의 속도가 기준 node 0 에서 2.5 rad/s 벗어나도 QP 의 jerk 에 상한이 없어 7 반복 · 3.3 ms 에 풀린다. 위치 표류만 trust region 충돌로 거부한다.

### E1-F02 · E1-F03 — payload · 샘플러 · decel 계획기 (2026-09-30, [#628](https://github.com/hyujun/rtc-framework/issues/628) · [#629](https://github.com/hyujun/rtc-framework/issues/629))

`test_catching_node_follower` 와 `test_catching_decel_planner` 의 정보용 측정이다. Release, 개발 PC (RT 스케줄링 없음), 실제 6 · 7 자유도 팔 URDF. 계획기 측정은 무작위 진입 상태 200 개 (관절마다 $\pm 0.54\,\dot q_{\max}$) 이고 출하 지평 $N_s$ 14 · $\Delta_s$ 0.025 s 다 (MD-24). 계획기 시간은 decel 단계 시작부터 풀이 끝까지다 (MD-26).

| 항목 | 6 자유도 | 7 자유도 |
|---|---|---|
| cold 풀이 (pre-solve + 본 solve) p99 [ms] | 8.4 | 12.6 |
| 포구 뒤 재계획, RT 가 구간을 따름 p99 [ms] | 2.0 | 2.9 |
| 포구 뒤 재계획, RT 보고에서 예측 p99 [ms] | 1.8 | 2.2 |
| 실패 (풀이 · slack 보류) | 0 / 200 | 0 / 200 |
| armature 가 더할 토크 $\max\vert a\ddot q\vert/\tau_{\max}$ (정보용) | 0.026 | 0.007 |

- **예산 안이다.** cold 도 `budget_s` 20 ms 의 2/3 이하라 MD-24 의 후퇴안 (7 × 0.05 s) 은 필요 없다. 재계획은 shift 한 기준 덕분에 pre-solve 없이 풀린다.
- **할당.** 풀이 전에 끝나는 경로 (시각 미도래 · 최신 · 상태 없음 · 비유한 · x₀ box 밖) 는 0 이다. 재계획 한 번의 C 할당 10 회는 ProxQP 의 것이다 (MD-23, #654). 샘플러 `Sample` 은 0 이다.
- **payload 복사.** 4.9 KB 를 1 kHz 로 저장하는 writer 와 경합할 때 복사 + `Sample` 의 p99 는 0.5 µs, 최악 64 µs 다. 쉬지 않고 저장하는 writer 앞에서는 reader 가 300 ms 에 224 번만 읽는다 — SeqLock 의 재시도에 상한이 없으므로 계획기는 wake 당 한 번만 저장해야 한다 (지금 그렇게 한다).
- **armature.** MJCF 값으로 재평가한 추가 토크는 τ_max 의 3 % 미만이다. 제어에 넣지 않는 결정 (MD-25) 과 맞는다.
- **CLIK 속도 잔차 (정보용, MD-30).** 6 자유도 합성 구간에서 $\vert v^\ast-\dot q_{ref}\vert$ 는 최대 0.16 rad/s (상대 0.14) 다. 자세 과제에 속도 feedforward 가 없기 때문이며 E1-F04 가 다룬다.
- 정지 자세의 손목 정적 중력비 0.7 이상은 0 / 200 이다. 손이 없는 팔 URDF 라서 손을 잠근 sub-model 과는 다를 수 있다 — E1-F04 의 sim 에서 다시 본다.

### E1-F04 — DECEL 에서 MPC 구간 추종 (2026-09-30, [#630](https://github.com/hyujun/rtc-framework/issues/630))

`test_catching_supervisor_scenarios` (가짜 시계, 완전 servo, oracle plan — 테스트가 decel box 를 쓴다) 와 `test_catching_planner_lane` (실시계, 계획기 스레드) 의 측정이다. Release, 개발 PC, ur5e_p1b 모델, 판정은 테스트 단언이고 표의 수치는 정보용이다.

| 항목 | 값 |
|---|---|
| NormalTrial 명령 digest (기본값 · `closed_form` 명시 · 추출 리팩터 전후) | `58e18c86679c92d6` — 모두 같다 |
| v1 명령과 DS 기준의 시간 차 (MD-40, 정보용) | 최고 속도에서 0.80 $h$, 중앙값 0.20 $h$ (범위 −2.95 – 0.98 $h$) |
| G7-B′ 진입 연속 ($\Vert p_d-\mathrm{FK}(q_c)\Vert$ · $\Vert V_{ff}-J\dot q_c\Vert$ · $\Delta q$ · $\Delta\dot q$) | 모두 < 1e-9 |
| 따르는 중 catch frame 오차 $\max\Vert\mathrm{FK}(q_{out})-\mathrm{FK}(q_{ref}(t))\Vert$, $t$ = now + $h$ / $2h$ / $3h$ | 0.46 / **0.14** / 0.67 mm ($h\vert\dot p\vert_{\max}$ 0.55 mm, 0.3 rad/s bump) |
| 같은 측정의 관절 최대 오차 | 0.53 / 0.59 / 1.17 mrad |
| 실시계 한 투척, 계획기 구간의 진입 게이트 $\rho$ (3 회) | 1.13 · 1.13 · 1.18 — $\vert\Delta\dot q\vert$ 0.22 rad/s (관절 4) 가 지배 |
| tick 시간 최악 (가짜 시계, 3 회) [µs] | mpc: 채택 20 (노드 FK 15 회 포함) · 전환 25 · 추종 24 – 45 · 채택 없는 CLOSING 42 – 70. v1 한 시행 전체의 최악 111 – 121 — 추종 tick 은 soft-catch 기준을 돌리지 않아 늘지 않는다 |

- **시간 규약 (MD-40).** v1 명령은 DS 기준보다 0.2 – 0.8 $h$ 앞일 뿐이다 — 자세 행과 적분 과도가 정하므로 r9 의 "2$h$" 유도는 v1 에 맞지 않았다. 규약은 추종 쪽에서 성립한다: catch frame 에서 $2h$ 가 이웃 $h$ · $3h$ 보다 3 배 이상 가깝다. 남는 0.14 mm 는 bump 의 가속 구간에서 되먹임이 늦는 몫이고 $2h$ 양쪽에서 같은 크기다.
- **관절 오차가 손 오차보다 크다.** 여유 자유도 방향은 자세 행 ($w_{arm}$ 0.01, $K_n$ 1) 만 붙잡고 smoothing 항 ($w_{smooth}$ 0.001) 이 끈다. 게이트의 $\Delta q$ 는 관절 공간이므로 이 몫이 $\rho$ 에 들어간다 — MD-41 측정에서 따로 본다.
- **기본 게이트는 실시계 한 투척을 거부했다.** $t_{pre}$ 0.1 s 앞에서 예측한 포구 전 구간이 진입에서 $\rho$ 1.13 – 1.18 로 기본 $\rho_{\max}$ 1.0 을 넘었다. MD-44 로 그런 시행은 abort 다. 테스트는 루프가 도는지를 보려고 `switch_margin` 2.0 을 쓴다. 기본값이 얼마나 자주 거부하는지와 $x_0$ 예측을 고칠지는 MD-41 의 200 발 측정이 정한다 — $t_{pre}$ 를 줄이는 것 (외삽 오차는 $t_{pre}^3$ 에 비례) 도 후보다.
- **작업공간 검사 (MD-43 개정).** 같은 실시계 투척에서 v1 법칙이 $t_c$ 에 손을 $p_c$ 에서 19 cm, box 아래 (z 0.19 < 0.21) 에 두었다. 절대 위치 검사는 계획기의 두 구간을 모두 거부했다. $p_c$ 로 옮긴 변위 검사로 바꾼 뒤 채택된다.
- **할당 0.** mpc 의 lane · 진입 전환 · 재계획 전환 · 추종 tick 이 `ScopedAllocGate` 아래에서 0 이다 (`test_demo_catching_alloc_s7`).
- **sim smoke (2026-09-30, [#630 코멘트](https://github.com/hyujun/rtc-framework/issues/630#issuecomment-5912267455)).** ur5e_p1b, `catch_lead_on` + `mode: mpc` + `switch_margin` 2.0, s35b 20 발 (seed 604): **20/20 이 DECEL 진입에서 abort** (CLOSING → ABORT_SAFE). 순환은 모두 닫혔고 FAULT 는 없다. 임시 계측 5 발에서 구간은 매번 채택됐고 진입 게이트가 거부했다 — $\rho$ 4.1 – 10.1, 매번 관절 0, $\vert\Delta\dot q\vert$ 0.78 – 1.76 rad/s. k=0 구간의 $x_0$ 는 v1 명령을 74 – 96 ms 상수 q̈ 로 외삽한 값이고 (5 발 중 2 발은 한계에 clamp), 포구 직전의 v1 명령은 그 외삽을 벗어난다. 위 실시계 $\rho$ 1.13 – 1.18 은 팔을 고정한 fixture 의 값이었다.
- **사용자 결정 (2026-09-30).** `mode: mpc` 는 APPROACH 부터 정지까지 MPC 로 간다 — 입력은 공의 미래 궤적, 출력은 CLIK task position + null space 자세. 코어 · 관절 노드 payload · RT 샘플러 · MD-36 · MD-44 는 유지하고, formulation §1.3 의 포구 항을 단일 팔 형태로 더한다 (E3-F01 을 단일 팔로 앞당김). 진입 순간의 외삽이 없어지므로 MD-41 의 $x_0$ 예측 측정은 그 기능의 설계로 대체된다. 기능 등록과 계획은 별도 — E1-F07 – F10, framing 은 MD-46 – MD-50.

### E1-F07 — 단일 팔 MPC 코어: 정확성과 계산 시간 (2026-10-01, [#660](https://github.com/hyujun/rtc-framework/issues/660))

`test_catching_decel_mpc_approach` 의 결과다. 판정은 테스트 단언이고 시간 표는 정보용이다 (MD-51).

**정확성.**

- 기본 파라미터의 코어는 E1-F07 전과 같은 QP 를 조립한다. `main` `edc0fa4e` 에서 뽑은 golden 과 조립 행렬이 상대 1e-12 로, 해가 1e-5 로 같다 (pre-solve 경로 · $w_\perp$ 를 켠 warm 주기 · 토크 행이 걸린 출하 지평).
- 포구 위치 · 접근축 · 손 속도의 Jacobian 이 중심 차분과 상대 1e-6 으로 같다 (6 · 7 자유도, 무작위 자세). 조립된 포구 비용은 비선형 비용과 2 차 오차로 일치한다 (오차 비 3.0 – 5.5).
- $H_v$ 는 Pinocchio 의 `getPointVelocityDerivatives` 다. `getFrameVelocityDerivatives` 로 바꾸면 유한 차분 테스트가 실패한다 (mutation 으로 확인) — formulation 의 `[확인 필요]` 를 닫는다.
- 코어 경로의 C 할당은 0 이다 (포구 선형화와 조립 전체). ProxQP 는 warm 풀이 한 번에 10 회 할당한다 (MD-22, #654).
- mutation 10 종 ($H_v$ 부호 · frame 변형, 접근축 극한, jerk 가중, 교차항 전치, 위치 상수항, slack 행 부호, $W_v$ 뒤바꿈, $\gamma_{ref}$ 무시, 균일 간격 gain) 이 모두 해당 테스트에서 잡혔다.
- E1-F01 의 `AssemblePerp` 는 Hessian 을 약 1e-15 만큼 비대칭으로 만들고 있었다. 공통 조립 루틴으로 옮겨 정확히 대칭이 됐다.
- code review (2026-10-01) 에서 고친 것. (1) 포구 전 노드가 없을 때 `dt_pre` 를 검사하지 않아, 비유한 값이면 `Init` 이 성공하고 모든 풀이가 실패했다. (2) 정지 경로 항 $w_\perp$ 가 포구 전 노드에도 걸려 접근 전체를 정지 직선으로 끌었다 — 정지 구간의 노드 ($k\ge k_c$) 로 좁혔다 (포구 전 노드가 없는 E1-F01 의 문제는 그대로). (3) warm 풀이가 실패하면 코어가 solver 를 비우고 한 번 다시 푼다 (결과의 `cold_retried`). 세 가지 모두 되돌리면 실패하는 테스트가 있다. 측정 테스트는 이제 풀이 실패 0 을 단언한다.

**계산 시간** (Release, 개발 PC, 표본 200, 고정 seed, 모든 포구 항과 토크 행 켬, 한 풀이의 p50 / p99 [ms]). 표본 하나는 대기 자세에 정지한 팔이 속도 box 로 닿는 무작위 목표 자세로 가는 한 투척이다.

| 구성 (7 자유도) | 변수 · 행 | 첫 풀이 (cold) | 같은 격자점 재풀이 | 격자점 전진 |
|---|---|---|---|---|
| A1 — 포구 전 0.05 s × 12 + 정지 0.025 s × 14, 1 노드 블록 | 308 · 910 | 63.7 / 73.1 | 32.9 / 38.5 | 84.5 / 103.2 |
| B1 — 포구 전 0.05 s × 12 + 정지 0.05 s × 7, 1 노드 블록 | 245 · 665 | 30.9 / 36.4 | 16.0 / 19.5 | 37.4 / 44.2 |
| A2 — A1 에 포구 전 2 노드 블록 | 266 · 910 | 59.4 / 68.8 | 29.8 / 34.0 | 75.9 / 91.1 |
| B2 — B1 에 포구 전 2 노드 블록 | 203 · 665 | 26.9 / 31.9 | 13.7 / 16.3 | 31.9 / 38.4 |
| **C1 — 포구 전 0.1 s × 6 + 정지 0.05 s × 7, 1 노드 블록 (확정, MD-54)** | 161 · 455 | 12.7 / 16.0 | 6.7 / 8.7 | 14.7 / 17.9 |
| 임계 (MD-51) | | ≤ 12 | ≤ 10 | ≤ 10 |

- **처음 잰 네 구성 (A1 – B2) 은 모두 임계를 넘는다.** 풀이 실패는 0 / 200 이다. 6 자유도는 약 0.7 배다 (B2: 19.6 / 29.7, 9.2 / 11.3, 21.3 / 26.5).
- **확정한 C1 은 같은 격자점 재풀이만 임계 안이다.** 첫 풀이는 임계의 1.3 배, 격자점 전진은 1.8 배다. 풀이 실패는 0 / 200 이다. 6 자유도는 8.3 / 12.0, 4.6 / 6.2, 9.6 / 12.5 다. 60 Hz 여유 (재풀이 ≤ 5 ms, MD-51) 에는 들지 않는다. C1 은 세 번 쟀고 표의 값은 그중 가장 느린 실행이다 — p99 의 범위는 첫 풀이 13.6 – 16.0, 재풀이 7.3 – 8.7, 격자점 전진 16.3 – 17.9 ms 이고 판정은 세 번 모두 같다. 같은 실행들의 A1 – B2 는 위 값과 5 % 안에서 같았다.
- **시간은 QP 풀이의 것이다.** 선형화와 조립은 합쳐 약 0.1 ms 다.
- **격자점을 전진하는 재계획이 첫 풀이보다 느리다.** 격자가 $t_c$ 에 고정이라 격자점이 하나 전진하면 노드 수가 다른 코어를 쓰고, solver 의 warm start 가 이어지지 않는다 (기준만 이어진다). 그 코어가 직전에 푼 문제 (다른 투척) 의 warm start 를 그대로 쓰면 warm 풀이의 30 – 40 % 가 거짓 "실행 불가능" 으로 실패한다. 코어는 그때 solver 를 비우고 한 번 다시 풀어 풀이는 성공한다 (실패 0 / 200). 그래도 코어를 바꿀 때는 `cold_start` 를 켠다 (E1-F08) — 실패한 warm 풀이의 시간이 더해지기 때문이다: C1 의 p99 는 21.9 ms (cold 16.3 ms) 이고, 다른 격자에서는 실패한 풀이 하나가 0.66 – 1.2 s 를 쓴 표본이 있다.
- 첫 풀이 (RTI 1 회) 의 포구 위치 오차는 p50 1.8 mm · p99 6.8 mm 다 (B2; C1 은 1.8 · 6.6 mm). 노드 사이의 속도는 box 를 최대 3 % (B2), 6 % (C1) 넘는다.

**무엇이 시간을 정하는가** (B2 에서 하나씩 바꿈, p99 [ms]).

| 바꾼 것 | 첫 풀이 | 같은 격자점 | 격자점 전진 | 비고 |
|---|---|---|---|---|
| B2 그대로 | 31.9 | 16.3 | 38.4 | |
| 토크 행 끔 | 17.7 | 9.6 | 14.7 | 절반 |
| 포구 전 간격 0.1 s × 6 (노드 13) — C1 | 16.0 | 8.7 | 17.9 | 절반. 첫 측정은 15.2 · 8.4 · 17.3 |
| C1 + 토크 행 끔 | 8.7 | 5.1 | 7.7 | 임계 안. 첫 측정은 8.9 · 5.0 · 8.2 |
| 포구 전 노드 9 개 (진입이 $t_c$ 의 0.44 s 앞 — E0-F02 의 중앙값) | 30.0 | 10.4 | 23.9 | |
| jerk 가중 100 배 | 25.3 | 14.7 | 29.0 | 반복 41 → 23, 문제가 달라진다 (위치 오차 p50 3.6 mm) |
| 상대속도 항 끔 | 31.4 | 13.9 | 30.8 | warm 반복 14 → 4 |
| solver 허용오차 1e-4 | 31.0 | 15.7 | 38.3 | 차이 없음 |
| 변수 scale 만 바꿈 (같은 문제) | 32.7 | 16.4 | 37.6 | 차이 없음 — ProxQP 의 equilibration 이 이미 한다 |
| trust region 없음 | 32.6 | 11.9 | 31.5 | 위치 오차 p50 23.5 mm — 쓸 수 없다 |

- 시간을 정하는 것은 노드 수와 토크 행이다. 가중 · 허용오차 · scale 은 영향이 작다.
- $s_v$ 를 켜면 첫 풀이가 47.3 ms 로 는다 (B2). 거짓 "실행 불가능" 은 0 / 200 이다.
- **격자는 C1 이다 (MD-54, 사용자 결정 2026-10-01).** 네 후보가 임계를 넘어 MD-51 에 따라 수치와 줄이는 후보 넷을 보고했고 ([#660](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5924188945)), 포구 전 간격을 0.1 s 로 넓히는 안이 채택됐다. 첫 풀이와 격자점 전진의 초과는 남는다 — 계획기의 예산과 wake 배치가 받는다 (E1-F08, MD-54 의 근거 (4)).

## 9. 개정 이력

결정의 내용과 날짜는 §4 가, 측정은 §8 이 갖는다. 이 표는 판마다 무엇이 바뀌었는지만 적는다.

| 판 | 바뀐 것 |
|---|---|
| r18 | E1-F07 완료 반영 — 상태줄 · §6 의 표 · 브랜치 계획 갱신, 개정 이력을 이 절로 옮김 |
| r17 | E1-F07 code review 반영: 실패한 warm 풀이의 cold 재시도, 정지 경로 항의 범위, §8 |
| r16 | E1-F07 의 격자 확정 MD-54: 포구 전 0.1 s + 정지 0.05 s × 7, 측정 §8 |
| r15 | E1-F07 착수 전 결정 MD-51 – MD-53: 격자는 정확히 구현 · 검증한 뒤 측정으로 정한다 (임계는 추정기 게시 주기 30 Hz 기준, 60 Hz 는 대비), 포구 항의 형태, 상대속도의 목표 |
| r14 | 단일 팔 MPC 의 framing, MD-46 – MD-50: 설계는 formulation §1.3 하나이고 단일 팔은 dual arm · waist 항만 뺀 구성, G1 은 같은 코어에 그 항을 더한다. closed_form 과 mpc 는 입출력이 같은 두 planner 다. G-1 은 E3 를 막지 않는다. E1-F07 을 코어 · 계획기 · L7 · 튜닝 (F07 – F10) 으로 나눈다 |
| r13 | MD-45 의 기능을 E1-F07 ([#660](https://github.com/hyujun/rtc-framework/issues/660)) 로 등록 |
| r12 | E1-F04 머지 ([#658](https://github.com/hyujun/rtc-framework/pull/658)), sim smoke 에서 `mode: mpc` 진입 20/20 abort, 결정 MD-45: `mode: mpc` 는 APPROACH 부터 정지까지 MPC |
| r11 | E1-F04 구현: MD-43 을 $p_c$ 기준 변위 검사로 개정, 측정 §8 |
| r10 | E1-F04 결정 MD-44: DECEL 법칙을 섞지 않는다, 구현 착수 |
| r9 | E1-F04 착수 전 결정 MD-34 – MD-43 ([#630](https://github.com/hyujun/rtc-framework/issues/630)) |
| r8 | E1-F02 · E1-F03 spec 과 구현 ([#656](https://github.com/hyujun/rtc-framework/pull/656)): 결정 MD-23 – MD-33, 측정: §8 |
| r7 | E0-F04 예측 격자 sweep 결과: §8, 결정 MD-20 |
| r6 | E0-F02 baseline 결과: §8, 결정 MD-19 |
| r5 | 빌드 경로 확정: 결정 MD-17 · MD-18 |
| r4 | 브랜치 계획 추가 |
| r3 | formulation v0.4 확정 반영: 결정 MD-9 – MD-16, 예측 격자 sweep, 게이트 G-1 의 검정 방법 |
