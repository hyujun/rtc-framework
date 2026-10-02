# MPC · dual-arm catching — 구현 계획

- 개정: r26 (2026-10-02) — 이력은 §9. 최초 작성 2026-09-29
- 상태: **E0 완료**, E1 진행 중. 완료: E1-F01 – F05 · E1-F07 – F09. `mode: mpc` 는 계획기가 plan 과 첫 구간을 쌍으로 내고 RT 가 APPROACH 부터 HOLD 까지 그 구간을 따르는 닫힌 루프다 (sim p1b 50 발: abort 0, 성공률은 튜닝 전 0.34 vs `closed_form` 0.80). 출하 기본은 `closed_form` 그대로다. 다음은 F10 튜닝 ([#663](https://github.com/hyujun/rtc-framework/issues/663)) → F06 (G-1) 이다 — 튜닝은 F05 가 낸 열 ($t_c$ 의 계획 · CLIK · 서보 분해, lane 사건, 포구 노드) 을 읽는다 (§8 "E1-F05"). feature 별 상태는 §6
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
| 회귀 | 기존 supervisor 시나리오 테스트가 assertion 변경 없이 통과 — `closed_form` 의 테스트를 말한다. `mode: mpc` 의 테스트는 E1-F09 에서 동작이 바뀌어 spec 변경으로 다시 썼다 (MD-65, [#674](https://github.com/hyujun/rtc-framework/pull/674)) |

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
- **구현 원칙 (사용자 결정 2026-10-02).** 가장 우선은 계획한 알고리즘을 정확하게 구현하는 것이다. spec 을 바꿀 필요는 정확히 구현한 뒤 테스트 · 측정에서 드러날 때 정한다 — 구현 중에 시간이나 성공률을 추정해 spec 을 미리 줄이지 않는다.

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
| `supervisor.sat_ticks` | 로봇별 포화 임계 (sim 분포에서 도출) | `mpc` 에서는 soft-catch DS 가 돌지 않아 발생하지 않는다 — E1-F09 에서 확인했다 (추종 tick 의 `ref_valid` 0, §8) |
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
| MD-52 | E1-F07 의 포구 항 (포구 노드 $k_c$ 한 점). 포구 위치의 가중은 코어 입력 $W_p$ (3×3, 대칭 PSD) 다 — 코어는 추정기의 형식을 모르고, $\kappa(\Sigma_p+\sigma^2I)^{-1}$ 는 고유값에 하한 · 상한을 둔 순수 함수가 만든다. 접근축은 `rtc_math` 의 $e_a$ · $J_a$ 를 쓴다 (`CatchPoseIk` · CLIK 과 같은 오차). 상대속도는 $H_v\delta q$ 를 포함한다. slack $s_v$ 는 구현하되 코어의 기본값은 끔이다. 경로 이탈 $w_{path}$ 는 별도 항을 두지 않는다 ($\mathcal K_c=\lbrace k_c\rbrace$ 에서 $W_p$ 에 $w_{path}P_\perp$ 를 더한 것과 같다). $w_\Delta$ 의 공분산 비례는 풀이마다 받는 스칼라다. 새 파라미터의 기본값은 모두 끔이고, 그때 코어는 E1-F01 과 같은 문제를 푼다 | 사용자 결정 (2026-10-01). $\Sigma_p$ 는 계획기 스레드에 있다 (`CovarianceSnapshot`, model world 로 회전된 6×6) — 상수 가중은 공분산의 token 이 어긋날 때의 대체값이다. 출하 `planner.budget.sigma_trk` 는 0.0 이고 v1 의 오차 예산 게이트가 쓰므로 가중의 하한은 MPC 의 자기 파라미터로 둔다. 접근축의 사영 형태 $(I-a_da_d^\top)z_C$ 는 90° 에서 기울기가 0 이고 180° 에 거짓 최소가 있다. $v_{rel,allow}$ 의 값이 어디에도 없어 $s_v$ 를 켜는 것은 E1-F10 이 정한다 | MD-46 의 "$\Sigma_p$ 가 단일 팔 경로에 없으면 상수 가중" (있다 — 배선은 E1-F08), [#660 결정 목록](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5913561760) 6 의 접근축 형태 | 2026-10-01 |
| MD-53 | 상대속도의 목표는 코어가 고르지 않는다. 코어는 $\hat v_b$ 와 스칼라 $\gamma_{ref}$ 를 받아 비용의 목표를 $\gamma_{ref}\hat v_b$ 로 두고, $s_v$ 의 행은 늘 $\hat v_b$ 기준으로 건다. E1-F08 의 기본은 $\gamma_{ref}=1$ 과 방향별 가중 (formulation §1.3 — $\gamma$ 는 해의 결과) 이고, plan 의 $\gamma_f$ 를 넣는 것은 E1-F10 의 선택지다. 코어는 해의 $\gamma$ 를 결과에 적는다 | 사용자 결정 (2026-10-01). $\gamma_{ref}=1$ 에서 손의 진행 방향 속도는 $w_\parallel$ 의 당김과 jerk · 한계가 맞서는 곳에서 정해진다. v1 탐색의 후보 게이트 (정지 거리 · 충격량 · 오차 예산) 는 $\gamma_f$ 로 평가한 것이라 해의 $\gamma$ 와 다를 수 있다 (MD-46 의 한계의 연장) — 그래서 기록한다 | §4 미결의 "상대속도의 목표" | 2026-10-01 |
| MD-54 | E1-F07 의 격자는 포구 전 $\Delta_a$ 0.1 s (노드 최대 6, 노드마다 jerk 블록) + 정지 구간 $\Delta_s$ 0.05 s × 7 (블록 1 · 1 · 2 · 3) 이다 — 노드 최대 13. 측정 (7 자유도 p99, §8): 같은 격자점 재풀이 8.7 ms 는 MD-51 의 임계 (10) 안이고, 첫 풀이 16.0 ms (임계 12) 와 격자점 전진 17.9 ms (임계 10) 는 넘는다. **초과를 알고 정한 값이다.** 토크 행을 정지 구간에만 거는 것, solver 의 warm start 를 격자점 전진에 잇는 것, 임계를 다시 정하는 것은 하지 않는다. 격자는 코어의 파라미터이고 코어의 기본값은 그대로다 (포구 전 노드 0) — 값의 배선은 E1-F08 | 사용자 결정 (2026-10-01, [#660 측정 코멘트](https://github.com/hyujun/rtc-framework/issues/660#issuecomment-5924188945) 의 안 1). 측정한 네 후보 (A1 · B1 · A2 · B2) 가 모두 임계를 넘었고, 포구 전 간격을 넓히는 것은 formulation 과 코어를 바꾸지 않는다. 대가: (1) 효력 시각이 최대 0.1 s 늦다. (2) 노드 사이의 속도가 box 를 6 % 넘는다 ($\eta_v$ 0.95 에서 실제 한계의 약 1 %). (3) 간격이 둘이라 payload 의 간격 필드 (`dt_ns` 하나) 와 균일 간격 샘플러가 바뀐다 — 노드 수는 용량 24 안이다 (E1-F08 · F09). (4) MD-51 의 환산 (sim 은 개발 PC 의 1.5 배) 으로 전진 재계획은 sim 에서 약 27 ms 다. 주기 33.3 ms 안이지만 출하 `budget_s` 20 ms 를 넘는다. 첫 풀이는 약 24 ms 로, 탐색 (sim p99 8.6 ms) 과 같은 wake 에 두면 한 주기에 닿는다. 예산과 wake 배치는 E1-F08 이 정한다 | MD-51 의 "격자의 확정은 측정 뒤 사용자가 한다", §4 미결의 "격자의 값" | 2026-10-01 |
| MD-55 | E1-F08 의 포구 전 격자는 **켜는 값**이다 — `planner.decel_mpc.approach.n_pre_max`, 코드 기본 0. 0 이면 계획기는 E1-F03 의 정지 구간 계획기 그대로다. 출하 YAML 두 벌에는 MD-54 의 값 (정지 7 × 0.05 s · 블록 1 · 1 · 2 · 3, 포구 전 최대 6 × 0.1 s, `k_max` 2) 을 적고 `enabled: false` · `mode: closed_form` 은 그대로 둔다. 코드 기본값 (14 × 0.025 s, `k_max` 4) 은 바꾸지 않는다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q1. 정지 구간 계획기를 고정한 기존 테스트의 단언을 고치지 않는다 (PROC-6). 대가: 출하 YAML 로 `mpc` 를 켜면 E1-F09 전까지 모든 시행이 DECEL 진입에서 abort 한다 — RT 가 $t_c$ 앞에서 시작하는 구간을 받지 않는다 (MD-60). E1-F04 의 20/20 abort 와 결과는 같다 | MD-24 의 "출하 14 × 0.025 s" (코드 기본값으로 남는다) | 2026-10-01 |
| MD-56 | 첫 구간은 **탐색과 같은 wake** 에서 풀고 plan 과 **쌍으로** 게시한다 — 구간을 먼저, 같은 `publish_ns` 로. 구간이 게시 조건 (MD-62) 을 못 넘으면 plan 도 게시하지 않는다. 예산 키는 둘이다: `budget.first_s` (첫 구간) · `budget.replan_s` (그 뒤). 각각 풀이 시간의 상한이자 효력 시각을 정하는 lead 다. 쌍의 재확인은 **같은 track 의 더 새 스냅샷을 허용**한다. $t_c$ 가 $T_{freeze}$ 안으로 들어온 쌍과, node 0 가 읽히기 전에 지나는 쌍은 버린다. 쌍을 게시한 뒤 RT 상태가 그 게시를 반영할 수 있을 때 (게시 + 3 tick) 까지 탐색을 쉰다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q2. plan 만 먼저 가면 RT 는 따를 구간 없이 APPROACH 에 든다. 첫 풀이로 wake 가 궤적 주기 (33 ms) 를 넘으면 "같은 스냅샷" 재확인은 모든 쌍을 버린다 — RT 의 `JudgePlan` 도 track 만 본다. RT 가 plan N 을 채택한 사실이 보이기 전에 N+1 을 게시하면 box 의 구간이 덮인다. 대가: plan 의 도착이 첫 풀이만큼 늦는다. E0-F02 의 진입 lead 는 $T_{freeze}$ 바로 위부터라 버려지는 몫이 생긴다 — `planner.slice.t_lead_min` 의 조정은 E1-F10. 예산 값은 sim 실시계로 재서 0.035 · 0.025 s 로 확정했다 (사용자 결정 2026-10-02, §8) | MD-26 의 `t_pre` 대기 (포구 전 격자에서는 쓰지 않는다), MD-54 의 "예산과 wake 배치는 E1-F08 이 정한다" | 2026-10-01 |
| MD-57 | RT 가 plan 을 따르는 동안 (APPROACH) 계획기는 **탐색을 돌리지 않고** 그 wake 에 구간을 다시 푼다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q3. 탐색 (sim p99 8.6 ms) 과 MPC 를 한 wake 에 넣으면 주기를 넘는다. 예측이 움직이면 MPC 의 포구 노드가 따라간다. 대가: `mpc` 구성에서는 APPROACH 중의 v1 plan 전환 (L3 §4.7) 이 없다 — 후보는 첫 plan 의 것이다 (MD-46 의 한계의 연장) | — | 2026-10-01 |
| MD-58 | 재계획의 $x_0$ 는 언제나 **RT 가 보고한 구간**에서 평가한다 (MD-28 경로 (i) 의 일반화). 경로 (ii) 는 포구 전 격자에서 쓰지 않는다. 출처는 추론하지 않는다 — RT 가 대기 슬롯에 둔 구간 (`PlannerRtState::decel_pending` · `decel_pending_seq`) 이 $t_{eff}$ 보다 늦지 않게 시작하면 그것, 아니면 따르는 구간, 둘 다 없으면 풀지 않는다 (`not_followed`). 계획기는 게시한 구간 8 개를 들고 있고, RT 가 보고한 구간은 밀어내지 않는다. **같은 포구 전 격자점은 새 예측으로 다시 푼다** (`replan.same_point`, warm). 격자점이나 코어가 바뀌면 `cold_start` 다. 정지 격자점은 한 번만 푼다. 재확인은 "RT 의 새 보고로 다시 정한 출처가 같은 구간인가" 다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q4, 검토 뒤 구체화 ([#661](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5932025244)). 포구 전 간격 0.1 s 동안 예측은 세 번 갱신된다. RT 는 전환 게이트의 거부와 대기 구간의 폐기를 계획기에 알리지 않아, 시작 시각이 지났는지로 추론하면 틀린다. "`decel_seq` 가 그대로인가" 로 재확인하면 0.05 – 0.1 s 마다 전환하는 사슬에서 풀이의 10 – 70 % 를 버린다. RT 가 대기 필드를 채우는 것은 E1-F09 다 — 그 전에는 재계획이 돌지 않는다 (MD-59) | MD-28 의 경로 (ii) 와 MD-32 의 "격자점마다 한 번" (포구 전 격자에서) | 2026-10-01 |
| MD-59 | 측정 전용 키 `planner.decel_mpc.shadow`: 풀고 기록하되 **구간을 box 에 쓰지 않는다**. 재계획의 출처는 가장 최근에 푼 구간이다. E1-F09 에서 지운다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q5, 검토 뒤 구체화. E1-F09 전의 RT 는 구간을 보고하지 않아 재계획이 돌지 않는다 — 예산을 재려면 재계획이 돌아야 한다. 구간을 box 에 쓰는 측정 모드는 고치지 않은 RT 가 포구 뒤의 정지 구간을 받아 따를 수 있다. shadow 에서는 모든 시행이 DECEL 진입에서 abort 하므로 성공률은 읽지 않는다 | — | 2026-10-01 |
| MD-60 | 간격이 둘인 구간은 payload 의 필드 둘로 싣는다 — `n_pre` (포구 전 구간 수) · `dt_pre_ns`. 노드 0 는 $t_c - n_{pre}\Delta_{pre}$, 포구 노드는 $n_{pre}$ 번이고 그 뒤는 $\Delta_s$ 다. 노드 시각은 한 함수 (`DecelNodeTimeNs`) 가 정하고, 판정 (`ValidateDecelNodes`) 과 샘플러가 그것을 쓴다. 샘플러는 정수 ns 로 가르며 포구 노드의 시각은 정지 쪽이다. RT 는 `accept_pre_catch` (기본 꺼짐) 가 꺼져 있으면 포구 전 노드가 있는 구간을 거부한다 — 켜는 것은 E1-F09 다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q6. 격자가 $t_c$ 에 고정이라 노드별 간격 배열이 필요 없다 (payload 4.9 KB 유지). 샘플러는 노드 값을 검사하지 않으므로 shape 검사가 유일한 방어다 — 깨진 `n_pre` 는 노드 행렬의 열 수를 0 이하로 만든다. 균일 간격으로 읽기 · 포구 노드 한 칸 어긋남 · 정지 부분에 $\Delta_{pre}$ 를 쓰는 변형은 모두 테스트가 잡는다 | MD-27 의 균일 간격 payload, MD-54 의 대가 (3) | 2026-10-01 |
| MD-61 | 포구 뒤 재계획은 격자점 `k_max` 2 까지다 (출하). 정지의 끝은 그대로 $t_c + N_s\Delta_s$ | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q7. MD-31 의 창 (4 × 0.025 s) 과 같은 0.1 s 다. 블록이 3 개 아래로 내려가지 않는다 | MD-31 의 `k_max` 4 (출하값에서) | 2026-10-01 |
| MD-62 | v1 이 고른 $q^\star$ 에 속도 box 로 닿지 못할 때는 **기준을 줄이고 게시 조건으로 거른다**. 첫 풀이의 기준은 관절별 최소 jerk 곡선이다 — 목표를 코어의 위치 box 로 자르고, 최고 속도 $1.875\lvert d\rvert/T$ 가 $0.9\,\eta_v\dot q_{max}$ 를 넘으면 $d$ 를 줄인다. 게시 조건: 풀이 성공 · 예산 · 효력 시각 전 · slack (MD-33) · 포구 노드의 위치 오차 ≤ `publish.catch_pos_err_max` (0.02 m) · 노드 사이의 속도 극값 ≤ $\dot q_{max}$ · 마지막 노드의 정지 (코어의 기준 허용오차 1e-4). 못 넘으면 구간도 plan 도 게시하지 않는다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q8. 탐색의 도달 시간은 순위 게이트라 닿지 못하는 후보가 뽑힌다. trust region (0.1 rad) 때문에 많이 줄인 기준에서 푼 해는 포구점에 못 닿는다 — 포구 오차 조건이 그것을 거른다. 포구 전 노드 1 개는 reach 0.02 rad 에서도 오차 21.8 mm 로 걸린다 (§8). 조건은 모두 "통과" 를 양의 비교로 써서 NaN 이 통과하지 못한다. 후보를 MPC 로 고르는 것은 E3-F01 | — | 2026-10-01 |
| MD-63 | $\Sigma_p$ 의 배선. 첫 풀이의 $\hat p_b$ · $\hat v_b$ · $a_d$ 는 **plan 의 값**이고 $\Sigma_p$ 는 같은 스냅샷의 후보 표본 것이다. 재계획은 가장 새 궤적을 $t_c$ 에서 읽고, $\Sigma_p$ 는 $t_c$ 를 감싸는 두 표본의 위치 블록을 정수 ns 로 선형 보간한다 ($t_c$ 가 표본 시각이면 그 블록만). token 이 어긋나거나 값이 유한하지 않으면 상수 가중 `catch.w_const` 를 쓰고 $w_\Delta$ 의 배율은 1 이다. $w_\Delta$ 의 배율은 $\mathrm{clamp}(\mathrm{tr}\,\Sigma_p/\sigma_{ref}^2, 0, 1)$ 이고 첫 풀이는 0 이다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q9. 계수 0 과 NaN 의 곱은 NaN 이라 표본 시각에서는 한 블록만 읽는다. $\sigma_{ref}$ 는 trace 와 비교하므로 축별 $\sigma$ 가 아니다 ($\mathrm{tr}\approx3\sigma^2$). 값은 모두 provisional (E1-F10) | MD-52 의 "배선은 E1-F08" | 2026-10-01 |
| MD-64 | 코어는 포구 전 노드 수마다 하나 (출하 6) 와 정지 격자점마다 하나 (3) 다. 포구 코어는 정지 코어와 **같은 box** ($\eta_v$ · $\eta'_\tau$ · $m_q$) 를 받는다. configure 에서 코어마다 한 번 풀어 둔다 — 실패하면 configure 가 실패한다. 사전 풀이는 포구 전 격자를 켰을 때만 돈다 | 사용자 결정 (2026-10-01, [#661 결정 요청](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5925992882) · [확정](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5931729739)) 의 Q10. 코어의 노드 수는 `Init` 에서 고정이다 (MD-31). box 가 다르면 $t_c$ 에서 넘겨받는 정지 코어가 $x_0$ 를 box 밖으로 거부한다 (코어의 $\eta_v$ 기본값은 0.95, 계획기는 0.9). ProxQP 는 코어의 첫 풀이에서 가장 오래 걸린다. 포구 전 격자가 꺼진 구성에서 사전 풀이를 돌리면 기존 테스트의 수치가 움직인다 (PROC-6) | — | 2026-10-01 |
| MD-65 | `mode: mpc` 는 **언제나** APPROACH 부터 HOLD 까지 MPC 구간을 따른다. RT (CLIK) 의 입력은 closed_form 의 출력이거나 mpc 의 노드, 둘뿐이다 — E1-F04 의 "APPROACH 는 v1 DS, DECEL 만 MPC" 조합은 없앤다. `TRACKING → APPROACH` 는 plan 과 그 첫 구간을 **같은 tick 에 함께** 채택할 때만 걸린다: TRACKING 의 lane 은 채택 가능한 plan 에 대해 box 의 구간을 판정만 하고 (채택 기억을 건드리지 않는다), edge 가 둘을 함께 채택한다. 구간이 통과하지 못하면 plan 도 받지 않는다 (`NO_CATCHABLE_PLAN`). 채택 뒤 APPROACH 에서 plan 을 교체하지 않는다. `mpc` 에서 포구 전 격자가 꺼진 계획기 (`approach.n_pre_max` 0) 는 전제 미충족으로 park 한다 (`kDecelModeUnmet`). `DECEL` 진입은 전환이 아니라 따르던 구간의 연속이다 | 사용자 결정 (2026-10-02, [#662 분석](https://github.com/hyujun/rtc-framework/issues/662#issuecomment-5942328181) · [확정](https://github.com/hyujun/rtc-framework/issues/662#issuecomment-5942542607)) 의 Q1. planner 가 둘이라는 MD-45 · MD-46 의 framing 을 RT 가 그대로 갖는다. lane 이 TRACKING 에서 구간을 먼저 채택하면 edge 가 걸리지 않은 tick 뒤로 그 구간이 `repeat` 로 거부돼 쌍이 들어오지 못한다. 구간의 plan 대조는 COMMITTED 전에는 `plan_` 의 $t_c$, 뒤에는 동결이 기록한 $t_c$ 까지 본다 (E1-F04 의 대조는 COMMITTED 뒤의 값만 봐서 APPROACH 의 구간을 전부 불일치로 읽었다). 대가: 정지 구간만 푸는 계획기 (E1-F03 의 `Plan` · `t_pre_s`) 는 컨트롤러에서 도달할 수 없는 경로가 된다 — 지울지는 따로 정한다 (미결). E1-F04 의 시나리오 테스트와 lane 테스트 중 그 조합을 고정한 것은 spec 변경으로 다시 썼다 (PROC-6 근거 커밋) | MD-34 의 전제 (포구 전 격자 추가), MD-37 의 lane 범위 (TRACKING 의 쌍 판정 + APPROACH 부터), MD-38 의 "DECEL 진입 tick 의 넘겨받기", MD-44 의 "모든 DECEL 을 MPC 구간으로" (APPROACH 부터로), MD-55 의 "0 이면 정지 구간 계획기 그대로" (RT 는 park) | 2026-10-02 |
| MD-66 | 대기 슬롯은 하나, 채택 나이 상한은 50 ms 그대로다. 슬롯이 차 있을 때 **node 0 시각이 같은** 더 새 구간은 대기 구간을 교체하고 (같은 격자점의 재풀이, MD-58), 다른 격자점의 구간은 box 에서 기다린다. 다음 격자점의 구간이 기다리는 시간은 최대 `budget.replan_s` + 3 tick 이다 — configure 가 이것이 나이 상한보다 작은지 본다 (`planner.budget_s` 가 아니라 `decel_mpc.budget.replan_s` 로) | 사용자 결정 (2026-10-02) 의 Q2 와 [정정](https://github.com/hyujun/rtc-framework/issues/662#issuecomment-5942604109). 계획기는 $start+T_{arm}+B+2h$ 가 격자점 $G$ 를 넘는 wake 부터 다음 격자점을 풀고, RT 는 $now+T_{arm}+h\ge G$ 에서 $G$ 의 구간을 넘겨받는다 — `T_arm` 이 상쇄돼 기다림은 $B+h$ 이고, 전환 tick 에서는 lane 이 법칙보다 먼저 돌아 한 tick, 게시 뒤 판정까지 한 tick 이 더 든다. 0.025 + 0.006 = 31 ms 로 상한 안이라 정지 재계획은 구조적으로 버려지지 않는다. 격자 간격 (0.05 · 0.1 s) 이 $B+h$ 보다 크므로 두 격자점 앞의 구간이 먼저 오지 않는다 | MD-37 의 "비었을 때만 채택" (같은 node 0 시각은 교체), 나이 상한의 configure 검사 (`budget_s` → `budget.replan_s`) | 2026-10-02 |
| MD-67 | 구간의 판정에 **plan 의 track** 대조를 더한다 (`token.generation` — 어긋나면 `plan` 거부). 작업공간 검사 (MD-43) 는 **정지 부분**에 건다: 포구 전 노드가 있는 구간은 포구 노드부터의 경로를 포구 노드가 $p_c$ 에 오도록 옮겨 보고, 포구 뒤 재계획은 $p_c+(p_0-o)+(p_k-p_0)$ 로 본다. $o$ 는 포구 노드를 가진 구간 ($k_0=0$) 중 RT 가 마지막으로 넘겨받은 것의 포구 노드 위치 — $t_c$ 에 팔이 따르던 궤적의 포구점이다 | 사용자 결정 (2026-10-02) 의 Q3 · Q5. 계획기는 plan 의 track 으로 풀고 구간도 그 track 을 싣는다 (E1-F08 code review). RT 가 보고하는 `track_generation` 은 동결 뒤 마지막으로 소비한 track 이라 그것과 비교하면 거짓 거부가 난다 — plan 의 token 과 비교한다. 포구 전 노드는 대기 자세에서의 접근이라 L3 §4.9 의 정지 예약이 설명하지 않는다: node 0 를 $p_c$ 에 놓고 전체를 보면 접근 거리만큼 정지 부분이 밀려 모든 첫 구간이 거부된다 | MD-43 의 $o$ ("진입이 넘겨받은 첫 구간의 node 0" → 포구 노드), `JudgeDecelPlan` 의 판정 목록 | 2026-10-02 |
| MD-68 | 첫 구간의 node 0 가 오기 전의 APPROACH 는 채택 tick 에 seed 한 명령을 든다 (법칙을 돌리지 않는다). 따르는 모드에서 따르는 구간도 대기 구간도 없으면 그 자리에서 `kParamsTbd` → `ABORT_SAFE` 다. 첫 구간이 node 0 에서 전환 게이트를 못 지나도 같다. 대기 구간이 아직 due 가 아닌 것은 `DECEL` 진입 전까지 기다린다 | 사용자 결정 (2026-10-02) 의 Q4. 계획기는 첫 구간을 정지한 보고 자세에서 풀었다 (MD-62 `rest_tol`) — node 0 전에 팔을 움직이면 그 전제가 깨진다. 쌍으로 채택된 시행은 언제나 둘 중 하나를 들고 있으므로 둘 다 없다는 것은 구간이 깨졌다는 뜻이고, $t_c$ 까지 기다릴 이유가 없다. 전이표의 `{APPROACH · COMMITTED · CLOSING · DECEL, kParamsTbd} → ABORT_SAFE` 행은 이미 있다 | MD-44 의 "진입 tick 에 따를 구간이 없으면" (APPROACH 부터의 모든 따르는 tick 으로) | 2026-10-02 |
| MD-69 | RT 는 따르는 구간 (`decel_active` · `decel_seq`) 을 APPROACH 부터 HOLD 까지, 대기 구간 (`decel_pending` · `decel_pending_seq`) 을 슬롯에 있는 동안 보고한다. 게이트가 거부했거나 시행과 함께 버린 구간은 보고에서 빠진다. 공 · CLIK 감독 사유는 그대로다 — DECEL 전의 추종 tick 은 공 궤적을 lead 시각에서 한 번 샘플해 soft-catch 법칙의 tick 과 같은 사유를 낸다 (구간은 그 표본을 읽지 않는다). `planner.decel_mpc.shadow` 는 지웠다 | #660 결정 목록 항목 11 과 MD-58 · MD-59. 계획기는 재계획의 출처를 이 보고로만 정하므로 (MD-58) 보고가 없으면 재계획이 돌지 않는다. 감독 사유를 법칙에 묶어 두면 `mpc` 에서 `BALL_STALE` · `HORIZON_EXTRAP` 의 행이 조용히 사라진다. `REF_SATURATED` 는 soft-catch 기준의 사유라 `mpc` 에서는 나지 않는다 (§3.3) | MD-28 의 보고 범위 (DECEL · HOLD → APPROACH – HOLD), MD-59 (키 삭제) | 2026-10-02 |
| MD-70 | 정지 구간만 푸는 계획기 (E1-F03 의 `DecelPlanner::Plan` 과 `PlannerCycle` 의 decel 단계) 를 지운다. decel 계획기는 plan 의 첫 구간 (`PlanFirst`) 과 그 뒤 구간 (`Replan`) 만 풀고, `approach.n_pre_max` < 1 이면 configure 를 거부한다 — 키의 범위와 기본값 (0) 은 그대로이고, 0 인 profile 은 `mode: mpc` 에서 종전처럼 park 한다 (`kDecelModeUnmet`). 함께 없어지는 것: `replan.t_pre_s` 키, RT 가 보고한 명령을 외삽하는 $x_0$ 경로와 그 $\ddot q$ 추정, `DecelPlannerConstants` 의 `budget_s` · `report_lead_s`, `DecelPlannerModel` 의 $\ddot q$ 상한. `planner_events.csv` 의 `decel_h_s` · `decel_qdd_trusted` 열과 결과 `not_due` 는 남기되 더 쓰이지 않는다 (열 정리는 #631) | MD-65 뒤로 컨트롤러에서 도달할 수 없는 코드였다: `mpc` 는 plan 을 $t_c$ 앞에서 시작하는 첫 구간과 함께만 채택하고 `closed_form` 은 decel 코어를 만들지 않는다. 단위 테스트만 그 경로를 돌았고, 남겨 두면 두 계획기가 같은 box 를 쓰는 것처럼 읽힌다. 포구 뒤 격자점의 재계획 (정지 코어, MD-31) 과 payload 의 `n_pre` 0 형태는 `Replan` 이 쓰므로 남는다 | MD-26 (첫 풀이 시점 — 폐기), MD-28 의 경로 (ii) (폐기; 경로 (i) 은 MD-58), MD-40 의 계획기 쪽 보고 lead (폐기; RT 의 샘플 시각 $+h$ 는 유지), MD-55 의 "0 이면 정지 구간 계획기 그대로" | 2026-10-02 |

MD-7 의 귀결: 토크 행은 직전 해에서의 역동역학 값과 그 미분으로 선형화한다 (MD-13). 그래서 단일 팔 문제도 계획기 스레드에서 동역학 모델을 평가하고, 주기마다 선형화를 다시 한다.

미결 — 해당 feature 의 spec 에서 정한다:

- E1-F10: 비열등 한계 · N · 튜닝 seed · 반복 상한 (튜닝 전에, MD-50)
- E1-F10: ur5e_p1b 의 성공률 격차 (0.34 vs 0.80, §8) — 팔이 $t_c$ 에서 명령보다 16.7 mm 뒤에 있고, 원인은 $t_c$ 에서의 명령 가속도 × 서보 지연이다 (§8 "$t_c$ 간격의 분해"). **노드 사이 보간은 이미 있다** (`jerk_segment.hpp`, 매 tick 평가) — 더 만들 것이 아니다. 손잡이는 순서대로: (1) 속도 목표 `catch.gamma_ref` · `w_v_par` (planner 파라미터), (2) 움직일 시간 — `approach.dt_pre_s` (격자, 사용자 결정) · plan 을 더 일찍 내는 쪽 (`planner.slice.t_lead_min`, 공통부 여부 확인), (3) $t_c$ 근방 가속의 비용 (formulation 변경), (4) 지연 보상의 형태 — 시간 lead 대신 $q_{ref}+\tau\dot q_{ref}$ (공통부, 사용자 결정). 정지 부분의 작업공간 검사를 계획기의 게시 조건으로 옮기는 것 (RT 의 `catch_box` 거부 74 건) 도 남아 있다
- E1-F10: 구간의 샘플 시각 — RT 는 구간을 steady clock 으로 샘플하고 sim 의 tick 은 그 clock 위에서 간격이 고르지 않아, `mpc` 명령에 한 tick 짜리 속도 계단이 들어간다 (§8). 실기의 tick jitter 로 크기를 먼저 보고, 샘플 시각을 tick 마다 $h$ 씩 가는 축으로 바꿀지 정한다 (RT 법칙 변경 — E-8, 사용자 결정)
- E1-F10: `catching_arm_budget` 의 `e_commit` · `e_last` 는 soft-catch 의 `ref_e` 를 읽어 `mpc` unit 에서 0 이다. 튜닝이 그 도구를 쓰면 먼저 고친다 (E1-F05 는 `catching_trials` 만 고쳤다)
- E1-F06: 격리 코어 · 제어 PC 의 RT tick (sim 의 최댓값은 두 planner 모두 120 µs 를 넘었다, §8)
- E1-F10: iiwa7_leap 의 포구 전 시간 — plan 이 lead 0.21 s 근처에서 나와 첫 구간이 거의 게시되지 않는다. 간격을 줄이는 것만으로는 35 % 에서 멈춘다 (§8). 손잡이는 leap 의 `approach.dt_pre_s` 와 plan 을 더 일찍 내는 쪽 (`planner.slice.t_lead_min`, 순위 게이트) 이다
- E1-F06: `mpc` 를 기본값으로 바꿀지
- E1-F06: `decel_*` 이름 (CSV 열 · `DecelRecord` · `DecelEvent` · decel 계획기) 을 `segment_*` 로 통일할지 — `mpc` 는 APPROACH 부터 그 구간을 따르므로 이름이 뜻과 어긋난다. E1-F05 는 그대로 두었다 (기존 측정 자료와 분석이 이 이름을 읽는다). `mpc` 가 기본값이 되면 정한다
- 미배정: 항별 비용 분해 (per-term cost split) — MPC 코어가 각 항의 값을 해에서 평가해 내야 한다. E1-F05 는 포구 노드의 값 (위치 · 축 오차, $\gamma$, 상대속도, slack) 까지만 냈다. 가중을 고를 근거가 그것으로 모자랄 때 정한다
- 미배정: GUI 에 lane 상태 (따르는 구간 · 대기 구간 · 마지막 사건) 를 보일지 — `CatchingState.msg` 에 필드를 더해야 한다 (E-3, D-20 의 동결을 푸는 결정). E1-F05 는 planner 모드만 파라미터로 읽어 보인다
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

대상 로봇: ur5e_p1b · iiwa7_leap. `mode: mpc` 의 팔 기준을 APPROACH 부터 정지까지 MPC 가 만든다 — formulation §1.3 에서 dual arm · waist 항을 뺀 구성이다 (MD-45 · MD-46). E1-F01 – F04 는 정지 구간만 다룬 첫 단계이고, 그때의 정지 구간 전용 계획기는 지웠다 (MD-70). 게이트: **G-1** (§1).

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E1-F01 | [#627](https://github.com/hyujun/rtc-framework/issues/627) | jerk 입력 condensed QP 코어 (토크 제약 행 · slack) | E0-F03 | 완료 ([#655](https://github.com/hyujun/rtc-framework/pull/655)). 할당 0 은 코어 경로만 (MD-22) |
| E1-F02 | [#628](https://github.com/hyujun/rtc-framework/issues/628) | 관절 노드 payload (`DecelPlanSnapshot`, MD-27) + RT 샘플러 (관절 기준에서 FK) | E1-F01 | 완료 ([#656](https://github.com/hyujun/rtc-framework/pull/656)). RT tick 배선은 E1-F04 (MD-32) |
| E1-F03 | [#629](https://github.com/hyujun/rtc-framework/issues/629) | 계획기 스레드 통합 — 정지 구간 선계산 | E1-F02 | 완료 ([#656](https://github.com/hyujun/rtc-framework/pull/656)). 출하는 꺼짐 — §8 |
| E1-F04 | [#630](https://github.com/hyujun/rtc-framework/issues/630) | L7 DECEL 전환 — MPC 궤적 추종 (법칙은 configure 에서 하나, MD-44) | E1-F03 | 완료 ([#658](https://github.com/hyujun/rtc-framework/pull/658)) — 결정 MD-34 – MD-44, 측정 §8. sim smoke 에서 `mode: mpc` 진입 20/20 abort 였고 (→ MD-45), 마지막 항목 (sim 의 진입 연속) 은 E1-F09 가 닫았다 ([#674](https://github.com/hyujun/rtc-framework/pull/674)) |
| E1-F07 | [#660](https://github.com/hyujun/rtc-framework/issues/660) | 단일 팔 MPC 코어 — APPROACH–정지 격자, 포구 항 (위치 · 접근축 · 상대속도), 항 단위 조립 (MD-46 · MD-49) | E1-F04 | 완료 (2026-10-01, [#666](https://github.com/hyujun/rtc-framework/pull/666)) — 결정 MD-51 – MD-54, 측정 §8. 격자는 MD-54 이고 첫 풀이와 격자점 전진의 계산 시간은 임계를 넘은 채다 (E1-F08 의 예산이 받는다) |
| E1-F08 | [#661](https://github.com/hyujun/rtc-framework/issues/661) | 계획기 — TRACKING – DECEL 의 MPC 풀이, 첫 구간 · 예산, 재계획 ($x_0$ 경로 (i) 일반화), 격자의 배선과 간격이 둘인 payload (MD-54) | E1-F07 | 완료 (2026-10-02, [#673](https://github.com/hyujun/rtc-framework/pull/673)) — 결정 MD-55 – MD-64, 측정 §8. 예산 0.035 · 0.025 s 확정. RT 가 그 구간을 받는 것은 E1-F09 다. iiwa7_leap 은 이 격자로 plan 을 거의 내지 못한다 (E1-F10) |
| E1-F09 | [#662](https://github.com/hyujun/rtc-framework/issues/662) | L7 — RT 가 APPROACH – HOLD 를 MPC 구간으로 추종, DECEL 진입은 연속 (E-8). 노드별 간격을 읽는 샘플러 (MD-54) | E1-F08 | 완료 (2026-10-02, [#674](https://github.com/hyujun/rtc-framework/pull/674)) — 결정 MD-65 – MD-70, 측정 §8. p1b sim 50 발에서 구간 추종으로 HOLD 까지 50/50, abort 0. security review 는 보고할 것이 없었다. 성공률은 `closed_form` 보다 낮다 (E1-F10) |
| E1-F05 | [#631](https://github.com/hyujun/rtc-framework/issues/631) | 로그 · plot_rtc_log · demo_controller_gui — tick record 의 decel 블록과 계획기 레코드를 CSV 로, `catching_trials` 의 `mpc` 분해 · lane · 명령 열, plotter 패널, GUI 의 Catching 탭 | E1-F09 | 완료 (2026-10-02, [#678](https://github.com/hyujun/rtc-framework/pull/678)) — 확인 §8. 제어 동작 불변. 항별 비용 분해 · GUI 의 lane 상태 · `decel_*` 이름 통일은 하지 않았다 (미결) |
| E1-F10 | [#663](https://github.com/hyujun/rtc-framework/issues/663) | mpc planner 튜닝 — closed_form 대비 성공률 비열등 (MD-50) | E1-F05 | **다음** — 선행이 끝나 착수할 수 있다 |
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
| E3-F06 | [#645](https://github.com/hyujun/rtc-framework/issues/645) | 로그 · plot_rtc_log · demo_controller_gui — dual arm 열. 항별 비용 분해는 E1-F05 가 하지 않았다 (코어 변경 — 미결) | E3-F05 | 대기 |
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
| `feat/catching-mpc-approach-plan-path` (완료, [#673](https://github.com/hyujun/rtc-framework/pull/673)) | E1-F08 | 계획기 스레드와 payload 의 간격 (MD-54). 새 스레드는 필요하지 않았다 |
| `feat/catching-mpc-approach-l7` (완료, [#674](https://github.com/hyujun/rtc-framework/pull/674)) | E1-F09 | E-8 (Critical). `[CONCERN]` 컨펌과 security review 의 범위를 이 PR 로 한정했다. 정지 구간 전용 계획기의 삭제 (MD-70) 도 여기서 했다 |
| `feat/catching-decel-mpc-tooling` (완료, [#678](https://github.com/hyujun/rtc-framework/pull/678)) | E1-F05 | 로그 · plot · GUI. `verify-changes.sh --run` 의 빌드 순서 (의존 순서) 도 여기서 고쳤다 — 이 브랜치의 검증에서 드러났다 |
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
| 4 | `feat/catching-decel-mpc-plan-path` (완료) → `-l7` (완료) → `feat/catching-mpc-approach-core` (완료) → `-plan-path` (완료) → `-l7` (완료) → **`feat/catching-decel-mpc-tooling` (다음)** → `exp/catching-mpc-tuning` → `docs/catching-decel-mpc-g1` | `feat/g1-p1b-bringup` |
| 5 | `feat/demo-dualarm-controller` → `feat/g1-dualarm-tooling` | — |
| 6 | rename refactor (MD-48) → E3 의 다섯 브랜치 (E2 와 E1-F07 뒤, MD-47) | — |

병행은 서로 다른 패키지를 고치는 브랜치끼리만 한다. E1 이 최우선이므로 (MD-8 · MD-47), 병행할 여력이 없으면 단계 4 의 E1 브랜치를 먼저 한다.

## 7. 게이트 · escalation

| Feature | 사유 | 효력 |
|---|---|---|
| E1-F04 | L7 전이 동작 변경 (E-8) | Critical — 착수 전 `[CONCERN]` 과 컨펌, 완료 후 security review |
| E1-F09 | L7 전이 동작 변경 — APPROACH – HOLD 추종 (E-8) | Critical — 착수 전 `[CONCERN]` 과 컨펌, 완료 후 security review. 둘 다 거쳤다 ([#674](https://github.com/hyujun/rtc-framework/pull/674)) |
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

`test_catching_node_follower` 와 `test_catching_decel_planner` 의 정보용 측정이다 (계획기 쪽 측정은 정지 구간 계획기의 것이고, 그 계획기와 측정 테스트는 MD-70 으로 지웠다 — 수치는 기록으로 남긴다). Release, 개발 PC (RT 스케줄링 없음), 실제 6 · 7 자유도 팔 URDF. 계획기 측정은 무작위 진입 상태 200 개 (관절마다 $\pm 0.54\,\dot q_{\max}$) 이고 출하 지평 $N_s$ 14 · $\Delta_s$ 0.025 s 다 (MD-24). 계획기 시간은 decel 단계 시작부터 풀이 끝까지다 (MD-26).

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

### E1-F08 — APPROACH–정지 계획기 (2026-10-02, [#661](https://github.com/hyujun/rtc-framework/issues/661))

`test_catching_node_follower` (§4) · `test_catching_approach_planner` · `test_catching_approach_cycle` 와 `integrated_bringup` 의 lane 테스트 결과다. 판정은 테스트 단언이고 시간은 기록이다.

**정확성.**

- 간격이 둘인 구간의 샘플은 임의 시각에서 jerk 를 적분한 값과 1e-9 안으로 같다 (포구 전 노드 1 · 2 · 6 개, 표본 400). 샘플러를 균일 간격으로 읽게 · 포구 노드를 한 칸 어긋나게 · 정지 부분에 $\Delta_{pre}$ 를 쓰게 바꾸면 각각 테스트가 실패한다.
- 첫 구간의 node 0 는 RT 가 보고한 자세이고 (device 순서, 코어의 box 밖이면 box 로 사영) 포구 노드는 공에 0.02 m 안으로 닿는다. 같은 격자점 재풀이는 warm 이고 node 0 의 시각이 같다. 격자점 전진과 정지 코어로의 인계 (k = 0 · 1 · 2) 는 cold 이고 node 0 가 출처 구간 위에 있다. 정지의 끝은 언제나 $t_c + N_s\Delta_s$ 다.
- 재계획의 출처는 RT 의 보고다: 보고가 없거나 · ring 에 없는 seq 이거나 · 다른 plan 이면 풀지 않는다. 같은 격자점 재풀이를 12 번 게시해도 RT 가 따르는 구간은 ring 에 남는다.
- 사이클: 구간이 plan 보다 먼저 저장되고 `publish_ns` 가 같다. 구간이 보류되면 plan 도 게시되지 않는다. 풀이 중에 같은 track 의 새 스냅샷이 와도 쌍은 게시되고, 다른 track 이면 버린다. 쌍 직후의 wake 는 탐색하지 않는다. RT 가 plan 을 따르는 동안 탐색은 0 번 돌았다. 저장 순서를 바꾸게 · 같은 스냅샷을 요구하게 · 쌍 직후 대기를 없애게 바꾸면 각각 테스트가 실패한다.
- lane (실제 컨트롤러, 실시계): 계획기가 plan 과 첫 구간을 쌍으로 게시하고, RT 는 plan 만 채택한다. 구간은 채택되지 않고 시행은 DECEL 진입에서 `no_segment` 로 끝난다 — E1-F09 전의 동작이다 (MD-55).
- 할당: 풀이 앞에서 끝나는 경로 (`not_at_rest` · `too_late` · `not_followed` · `no_ball` · `up_to_date`) 는 C 할당까지 0, 풀이를 지나는 경로 (첫 풀이 · 같은 격자점 · 전진 · 정지 코어) 는 operator new 0 이다 (MD-23 과 같은 경계).
- 구현 중에 찾은 것. (1) 컨트롤러는 plan 을 받기 전의 TRACKING 에서 `cmd_seeded` 를 false 로 보고한다 (측정 자세, 속도 0). 첫 풀이가 seeded 를 요구하면 쌍이 한 번도 게시되지 않는다 — 단위 테스트의 fixture 는 seeded 로 넣고 있어 lane 테스트가 잡았다. 첫 풀이는 그 보고에서 출발한다. (2) 포구 전 노드 1 개는 reach 0.02 rad 에서도 포구 오차 21.8 mm 로 보류된다 (6 자유도, 공 속도 5 m/s). 원인은 가르지 않았다 — MD-62 의 게시 조건이 보류로 처리한다. (3) 속도 극값 검사는 가속도의 NaN 을 통과시켰다 (부호 비교가 false) — 가속도의 유한성을 따로 본다.

**code review 에서 고친 결함** (2026-10-02, 처리 표는 [#661](https://github.com/hyujun/rtc-framework/issues/661#issuecomment-5941994104)). 넷 모두 수정을 되돌리면 실패하는 테스트가 있다.

- 재계획의 공은 **plan 의 track** 것만 쓴다 (`FollowedTrack`), 구간도 그 track 을 싣는다. RT 는 동결 뒤에도 마지막으로 소비한 track 을 보고하므로 그 값으로 가르면 다른 공이 포구 목표가 된다.
- 첫 풀이의 시작 자세도 코어의 위치 box 로 사영한다 (`x0_clamped`). 대기 자세가 한계의 $m_q$ 안쪽이면 코어가 매 wake 거부해 plan 이 게시되지 않았다.
- 계획기는 위치 box 를 코어에서 읽는다. 코어의 여유는 $m_q$ 와 범위의 절반 중 작은 쪽이라, 따로 만든 box 는 좁은 관절에서 뒤집혔다.
- 코어가 거부했거나 실패한 풀이 뒤의 풀이는 cold 다 — 그 solver 에는 다른 문제의 반복값이 남아 있다.

**기록** (Release, 개발 PC, 그 테스트만 단독 실행 3 회의 범위).

| | 6 자유도 | 7 자유도 |
|---|---|---|
| 코어 수 | 9 (포구 6 + 정지 3) | 9 |
| configure (그중 사전 풀이) [ms] | 47 – 48 (38 – 39) | 71 – 72 (63 – 64) |
| 메모리 증가 (사전 풀이 뒤) [MB] | 31 | 30 |
| 사전 풀이 중 가장 느린 풀이 [ms] | 8.6 – 8.8 | 17.7 – 18.3 |
| 그 뒤 시행의 첫 풀이, 포구 전 노드 6 – 2 개 중 최댓값 [ms] | 8.5 – 8.6 | 14.2 – 14.4 |

- 사전 풀이의 문제 (공이 이미 손에 있는 포구) 는 시행의 문제와 달라 두 줄은 같은 문제의 전후가 아니다. 코어의 첫 풀이가 치르는 몫이 시행 밖으로 나갔다는 것만 말한다.
- 사이클 테스트의 쌍 wake (6 자유도, 실시계): 탐색 1.0 ms + 첫 풀이 5.7 ms (포구 전 노드 5 개). 한 시행의 재계획은 29 번이었다 — 같은 격자점 22, 전진 4, 정지 코어 3 (15 ms 마다 wake).
- 풀이를 지나는 경로의 C 할당은 첫 풀이와 정지 코어 풀이를 합쳐 20 회다 (ProxQP, #654).

**sim 실시계** (2026-10-01, shadow — MD-59, 로봇당 200 발: `s35b` 50 발 × seed 4 개, `T_arm` 0.05, 개발 PC). 8 unit 모두 재시도 없이 끝났고 host 부하로 버린 시행은 없다. shadow 에서 시행은 DECEL 진입에 abort 하므로 성공률은 읽지 않는다. 예산은 `budget.first_s` 0.035 · `budget.replan_s` 0.025 였고 예산을 넘겨 보류된 풀이는 0 회다. 이 값으로 확정했다 (사용자 결정 2026-10-02) — 한 wake 의 최대 24.8 ms 와 재계획의 최대 10.2 ms 에 대해 1.4 배 · 2.4 배의 여유이고, 제어 PC 의 값이 없어 줄이지 않는다. 계획기 스레드에서 33.3 ms 를 넘긴 wake 는 0 회다.

| [ms] | ur5e_p1b: 횟수 | p50 | p99 | 최대 | iiwa7_leap: 횟수 | p50 | p99 | 최대 |
|---|---|---|---|---|---|---|---|---|
| 한 wake: 탐색 + 첫 풀이 | 221 | 11.2 | 18.1 | 20.0 | 921 | 9.8 | 16.0 | 24.8 |
| 첫 풀이 | 221 | 7.0 | 12.7 | 14.1 | 921 | 5.9 | 8.8 | 10.9 |
| 재계획, 같은 격자점 | 954 | 1.8 | 3.9 | 5.4 | 13 | 2.3 | — | 3.2 |
| 재계획, 격자 전진 | 485 | 4.5 | 8.7 | 10.2 | 1 | 3.5 | — | 3.5 |
| 재계획, 정지 코어 (k = 0 · 1) | 370 | 2.3 | 4.6 | 5.2 | 24 | 2.1 | — | 3.7 |
| 계획기 스레드, 모든 wake | 15,153 | 0.0 | 9.9 | 20.0 | 69,952 | 0.0 | 8.0 | 24.8 |

| | ur5e_p1b | iiwa7_leap |
|---|---|---|
| 첫 풀이 시점의 lead [s] (최소 / p50 / 최대) | 0.375 / 0.436 / 0.593 | 0.191 / 0.211 / 0.303 |
| 첫 구간의 포구 전 노드 수 | 3 개 193 회, 2 개 16, 4 개 11, 5 개 1 | 1 개 987 회, 2 개 1 |
| 첫 풀이: 게시 / 전체 | 199 / 221 | 17 / 988 |
| 첫 풀이의 보류 사유 | `catch_error` 22 | `catch_error` 684, `slack` 216, `too_late` 67 |
| 쌍 경로의 `superseded` | 0 | 4 |
| 게시된 쌍 중 RT 가 plan 을 채택한 것 | 199 / 199 | 14 / 17 |
| plan 을 받지 못한 시행 | 1 / 200 | 186 / 200 |

- 첫 풀이의 p99 는 개발 PC 의 scratch 측정 (같은 코어, 표본 200) 의 1.8 – 2.2 배다 — p1b 의 포구 전 노드 2 · 3 · 4 개가 9.2 · 12.1 · 14.1 ms (개발 PC 5.0 · 5.5 · 7.1), leap 의 1 개가 8.8 ms (4.7). 문제 집합이 달라 같은 문제의 비는 아니다.
- 보류는 포구 전 노드가 적을수록 많다 (p1b, 보류 / 풀이). 첫 풀이: 2 개 14 / 16, 3 개 7 / 193. 같은 격자점: 1 개 46 / 339, 2 개 6 / 383, 3 개 6 / 232. 격자 전진: 1 개 77 / 258, 2 개 24 / 216, 3 개 0 / 11. 사유는 전부 `catch_error` 이고 `speed` 와 코어 사유는 0 회다.
- **leap 은 이 격자로 plan 을 거의 내지 못한다.** leap 의 plan 은 lead 0.21 s 근처에서 처음 나온다 (`T_freeze` 0.19). 첫 구간이 쓸 수 있는 시간은 lead − `T_arm` − `budget.first_s` − 2 tick = 약 0.12 s 라 0.1 s 격자 (MD-54) 로는 노드 1 개이고, 그 풀이는 게시 조건에서 보류된다. plan 은 첫 구간과 쌍으로만 게시되므로 (MD-62) 팔이 움직이지 않는다. 계산 시간의 문제가 아니다.
- p1b 에서 plan 을 받지 못한 1 발은 첫 풀이 6 회가 모두 `catch_error` 로 보류된 시행이다. leap 에서 채택되지 않은 쌍 3 건은 lead 0.198 – 0.201 s 에서 게시됐다 (RT 의 거부 사유는 확인하지 않았다).
- 게시된 구간 사이의 간격 (p1b, 한 시행 안) 은 p50 34 ms, p99 133 ms, 최대 292 ms 다 — 재계획이 연달아 보류되면 RT 는 앞 구간을 따른다.
- **leap 의 포구 전 간격을 줄인 변형** (사용자 결정 2026-10-02 로 더 잼, 출하값은 바꾸지 않았다). 같은 투구에서 `approach.dt_pre_s` 만 바꿨다.

  | 포구 전 간격 [s] (`n_pre_max`) | 발 | plan 이 채택된 시행 | 한 wake 최대 [ms] |
  |---|---|---|---|
  | 0.1 (6) — 출하 | 200 | 14 (7 %) | 24.8 |
  | 0.05 (6) | 50 | 14 | 22.8 |
  | 0.04 (8) | 200 | 70 (35 %) | 25.8 |
  | 0.03 (10) | 50 | 18 | 30.1 |

  0.04 s 의 200 발에서 첫 풀이의 게시는 포구 전 시간을 따른다: 0.08 s (노드 2 개) 는 7 / 400, 0.12 s (3 개) 는 51 / 301, 0.16 s (4 개) 는 13 / 15 다. 보류 사유는 전부 `catch_error` 다. leap 은 포구 전에 0.15 s 쯤, lead 로는 0.24 s 쯤이 있어야 첫 구간이 게시되는데 plan 은 lead 0.21 s (p50) 에서 나온다 — 간격만으로는 풀리지 않는다 (E1-F10). 0.04 s 에서도 예산은 맞는다 (첫 풀이 최대 17.2 ms, 재계획 최대 12.5 ms, 33.3 ms 를 넘긴 wake 0 회).
- 말하지 못하는 것. 포구 오차의 크기와 $x_0$ 속도의 분포는 이 측정의 CSV 에 열이 없었다 (E1-F05 가 `decel_catch_pos_err` · `decel_x0_speed` 로 냈다) — `approach.rest_tol` 은 잠정값으로 남는다. 정지 코어 k = 2 의 행은 없다. shadow 의 재계획은 가장 새 게시 구간에서 출발하므로 RT 보고에서 출발하는 닫힌 루프 (E1-F09) 의 보류율은 다를 수 있다. 제어 PC 의 시간은 재지 않았다.


### E1-F09 — L7: APPROACH – HOLD 를 MPC 구간으로 추종 (2026-10-02, [#662](https://github.com/hyujun/rtc-framework/issues/662))

`test_catching_supervisor_scenarios` (가짜 시계, oracle plan — 테스트가 쌍과 재계획을 쓴다), `test_catching_planner_lane` (실시계, 계획기 스레드), `test_demo_catching_alloc_s7` 와 sim 실시계의 결과다. 판정은 테스트 단언과 sim 의 abort 수이고, 시간과 성공률은 기록이다.

**정확성 (테스트).**

- `closed_form` 의 NormalTrial 명령 digest 는 `58e18c86679c92d6` 그대로다 (키 없음 · 명시 모두).
- 쌍: plan 과 첫 구간이 같은 tick 에 채택되고, 구간이 없거나 · 낡았거나 · 다른 plan 것이거나 · 다른 track 것이거나 · malformed 이거나 · reset 전 상태에서 풀렸거나 · 정지 부분이 `catch_box` 를 벗어나면 plan 도 채택되지 않는다 (`NO_CATCHABLE_PLAN` 으로 TRACKING 에 머문다). 작업공간 검사는 정지 부분만 본다 — 접근 구간을 포함해 node 0 부터 읽으면 거부되는 box 에서 쌍이 채택된다.
- 추종: node 0 전에는 seed 한 명령이 그대로다 (명령 변화 0). node 0 의 전환은 $\Delta q$ · $\Delta\dot q$ · $\Vert p_d-\mathrm{FK}(q_c)\Vert$ < 1e-9 (G7-B′). 움직이는 팔의 명령에서 node 0 를 만든 포구 전 재계획도 같다 ($\Vert V_{ff}-J\dot q_c\Vert$ 포함). 한 구간이 APPROACH · COMMITTED · CLOSING · DECEL · HOLD 에서 따라지고 각 모드에서 보고된다. DECEL 진입 tick 은 전환이 아니다 ($\Delta p_d$ 가 $h\,v_{ff}$ 와 10 % 안).
- 시간 규약 (MD-40) 은 그대로다: catch frame 오차가 $t$ = now + $h$ / $2h$ / $3h$ 에서 0.53 / **0.14** / 0.59 mm (374 tick, $h\vert\dot p\vert_{\max}$ 0.53 mm), 관절 0.74 / 1.05 / 1.63 mrad.
- 대기 슬롯: 같은 node 0 시각의 더 새 구간은 교체, 다음 격자점의 구간은 box 에서 기다렸다가 전환 다음 tick 에 채택, 80 ms 를 기다린 구간은 `aged` 로 버려진다.
- 따를 구간이 없으면 abort: 첫 구간이 node 0 에서 게이트를 못 지나면 ($\rho$ 5.0) 그 tick 에 APPROACH → `ABORT_SAFE` (`ParamsTbd`) 다. 공 stale 은 `BALL_STALE` → RETREAT, CLIK 실패는 `QP_FAILED` → `ABORT_SAFE` 로 `closed_form` 과 같다. APPROACH 에서 다른 plan 은 받지 않는다.
- E-STOP: 첫 구간이 기다리는 중 · APPROACH 추종 중 · DECEL (재계획이 대기 중) · tick 사이의 발동+해제 모두에서 대기 · 따르는 구간이 버려지고 보고에서 빠진다. 재무장 뒤 box 에 남은 옛 구간으로는 plan 이 채택되지 않고, 새 쌍은 채택된다.
- lane (실제 계획기, 실시계, 출하 CLIK 형태 `dynamic`): 쌍이 한 tick 에 채택되고 HOLD 까지 간다. 계획기가 재계획의 출처를 못 찾은 wake 는 0 회 (출처가 있는 재계획 wake 269 – 270 회), 구간 4 개가 차례로 따라졌고 전환 $\rho$ 는 최대 0.07, 게이트 거부 0 회 (3 회 실행). 같은 테스트를 fixture 의 `box` CLIK 로 돌리면 재계획 3 개가 게이트에서 거부되고 첫 구간만 따라진다 — CLIK 의 상수 가속 box (2.03 rad/s²) 가 MPC 의 가속을 못 따라가 명령이 구간보다 늦는다 (MD-7 의 MPC ⊂ CLIK 가 깨진 구성).
- 할당 0: 쌍 채택 · node 0 대기 · 같은 node 0 교체 · 전환 3 회 · 다섯 모드의 추종 tick 이 `ScopedAllocGate` 아래에서 0 이다.

**tick 시간 (가짜 시계, `test_demo_catching_alloc_s7`, 개발 PC).** mpc: 쌍 채택 4 µs · 대기 15 – 18 · 채택/교체 21 · 전환 25 – 35 · 추종 61 – 78, mpc tick 전체 최악 61 – 78. 같은 테스트의 v1 시행 최악은 122 µs 다.

**sim 실시계** (2026-10-02, 개발 PC, `s35b` 50 발, `catch_lead_on` + `mode: mpc`, `T_arm` 0.05, ur5e_p1b seed 601 · iiwa7_leap seed 701). `catching_diag.csv` 에 decel 블록 열이 없어 (#631) tick record 의 decel 블록을 CSV 로 내는 임시 패치를 측정 빌드에만 넣었다 — CSV writer 는 aux drain 스레드에서 돌고 RT tick 의 코드는 커밋과 같다. 같은 빌드 · 같은 날 `closed_form` 대조 unit (p1b, 같은 seed) 을 하나 돌렸다. 세 unit 모두 host-watch 통과 (시행별 RTF 최솟값 0.978 – 0.994).

| ur5e_p1b, 50 발 | `mpc` | `closed_form` (같은 날) |
|---|---|---|
| plan 이 채택된 시행 | 50 | 50 |
| APPROACH → COMMITTED → CLOSING → DECEL → HOLD → RETREAT | 50 | 50 |
| `ABORT_SAFE` | **0** | 0 |
| 성공 (truth: HOLD 끝부터 release 까지 공이 손에 있음) | 17 (0.34) | 40 (0.80) |
| supervisor 판정 `CAPTURED` | 14 | 38 |
| 시행당 따른 구간 수 (p50 / 범위) | 6 / 2 – 6 | — |
| $t_c$ 의 명령–측정 간격 `servo_mm` (p50 / p95) | 16.7 / 20.8 | 2.5 / 3.8 |
| 손–공 최근접 `d_min_mm` (p50 / p95) | 26.1 / 49.1 | 4.0 / 45.9 |
| 접촉 시 손 속도 [m/s] (p50) | 2.29 | 1.91 |
| RT tick, APPROACH – HOLD [µs]: p50 / p99 / p99.9 / 최대 | 38.9 / 74.3 / 111.1 / 215.4 | 38.5 / 73.0 / 116.6 / 143.7 |
| 그중 120 µs 를 넘은 tick | 25 / 30,637 (만 tick 당 8.2) | 20 / 27,791 (7.2) |

- **전환 게이트 거부로 인한 abort 는 0 이다.** 게이트 거부 자체가 0 회다: 첫 전환의 $\rho$ 는 50 회 모두 0.000 (정지한 seed 명령과 node 0 가 같다 — `x0_clamped` 0 회), 재계획 전환 221 회의 $\rho$ 는 p50 0.147 · p95 0.320 · 최대 0.589 로 기본 $\rho_{\max}$ 1.0 안이다. #630 의 마지막 항목 (sim 의 진입 연속) 이 이것으로 닫힌다 — E1-F04 smoke 의 $\rho$ 4.1 – 10.1 은 v1 명령의 외삽에서 온 것이었다.
- lane 의 사건 (시행 50 개의 tick): 채택 271 (쌍 50 포함) · 교체 207 · 전환 271 · `catch_box` 거부 74 · 게이트 거부 0. 다음 격자점의 구간이 box 에서 기다린 것이 163 번 (p50 5 tick, 최대 13 tick = 26 ms) 이고 그 뒤 153 번은 채택, 10 번은 `catch_box` 거부, **나이로 버려진 것은 0** 이다 (MD-66 의 상한 31 ms 안). node 0 를 기다리며 명령을 든 시간은 plan 채택부터 첫 전환까지 p50 68 ms (최대 128 ms) 다 — 명령을 든 채 COMMITTED 로 넘어간다 (r26 정정: 처음 적은 p50 50 · 최대 106 ms 는 APPROACH tick 만 센 값이었다).
- **`catch_box` 거부 74 건** 은 재계획의 정지 변위가 box 를 벗어난 것이다 (MD-43). 거부된 구간은 따라지지 않고 계획기는 RT 가 보고한 앞 구간에서 다시 푼다. 계획기는 이 검사를 모른다 — 게시 조건에 넣을지는 E1-F10.
- **tick 시간은 같은 날의 `closed_form` 과 같다.** 중앙값 +0.4 µs, p99 +1.3 µs 이고 120 µs 초과 비율도 같다 (8.2 vs 7.2 / 만 tick). 이날은 host 의 꼬리가 E0-F02 의 날 (120 µs 초과 1.3 / 만 tick, 최대 154.5) 보다 두꺼웠다: 느린 tick 은 CM 의 state · publish 단계도 3 – 4 배 느리고 이웃 tick 이 함께 느리며, 계획기가 풀이 중이던 것은 25 개 중 0 개다 — 법칙의 비용이 아니다. 단일 최댓값은 mpc 가 높다 (215 vs 144 µs). sim 은 `use_cpu_affinity:=false` 라 격리된 코어의 값이 아니다.
- **성공률은 `closed_form` 보다 크게 낮다** (0.34 vs 0.80, 튜닝 전). mpc 의 팔은 $t_c$ 에서 명령보다 16.7 mm 뒤에 있다 (`closed_form` 2.5 mm). 첫 구간의 node 0 가 0.1 s 격자에 묶여 plan 채택 뒤 p50 68 ms 를 기다렸다가 0.2 – 0.4 s 안에 포구 속도까지 가속하므로 $t_c$ 에서 가속이 크고 손 속도도 높다 (2.29 vs 1.91 m/s). sim 의 위치 서보는 1 차 지연이라 `T_arm` 의 시간 lead 는 등속 성분만 보상한다. 원인의 분해 (지연 모델 · 격자 · 가중치 · 손 시퀀서 시각) 와 조정은 E1-F10 이다 — 이 기능은 값을 바꾸지 않았다. 분석 도구의 `clik_mm` · `ref_vs_true_mm` 은 soft-catch 기준 열을 읽으므로 mpc 에서는 뜻이 없다 (#631).
- 계획기 (p1b): 첫 풀이 54 회 중 50 게시 · 4 보류 (`catch_error`), 한 wake 최대 20.1 ms. 같은 격자점 252 (게시 235) · 격자 전진 113 (게시 100) · 정지 코어 게시 170 (k = 0 · 1 · 2 가 62 · 56 · 52 — shadow 에서는 없던 k = 2 가 돈다). 재계획 최대 10.8 ms, 33.3 ms 를 넘긴 wake 0 회. 게시된 구간 사이의 간격 p50 34 ms. shadow 의 보류율 (격자 전진 21 %) 은 닫힌 루프에서 11.5 % 였다.
- **iiwa7_leap (정보용).** plan 이 채택된 시행은 2 / 50 (첫 풀이 268 회 중 게시 2 — `catch_error` 181 · `slack` 62 · `too_late` 20, E1-F08 과 같은 양상). 그 둘은 모두 첫 구간을 따라 HOLD 까지 갔고 abort 는 0 이다 (truth 1 성공 · 1 실패). 재계획 6 개는 모두 게이트에서 거부됐다 ($\rho$ 2.0 – 5.5) — leap 의 출하 CLIK 는 `box` 가속 제약이라 위 lane 테스트의 `box` 경우와 같다. 표본 2 개로는 더 말하지 못한다 (E1-F10).
- 측정의 흠. leap unit 의 첫 시도는 46 번째 시행에서 끊겼다 (rc 127 — 돌고 있는 `run_unit.sh` 를 편집했다). 같은 seed 로 다시 돌린 것이 위 값이고, 끊긴 것은 `leap_701.fail1` 로 남겼다.
- **$t_c$ 간격의 분해** (2026-10-02 추가 분석, 같은 unit 두 개의 `catching_diag.csv` · `cm_timing_log.csv` — [#663](https://github.com/hyujun/rtc-framework/issues/663#issuecomment-5944027621)). 검토한 가설은 "노드 간격이 RT 주기보다 넓어 오차가 난다" 였다.
  - 노드 사이는 RT 가 매 tick 닫힌식으로 평가한다 (MD-9, `SampleJerkTrajectory` — MPC 의 모델에 대해 정확하고 노드에서 $C^2$). 노드 간격은 RT 쪽 오차의 원인이 아니다.
  - `mpc` 의 명령은 $t_c$ 를 겨냥한 tick 에서 속도 2.58 m/s, $d\lvert v\rvert/dt$ **+22.6 m/s²** 다 (`closed_form` 2.19 m/s, −7.1 m/s²). sim 서보는 1 차 지연 ($\hat\tau$ 50.1 – 51.2 ms, $R^2\ge$ 0.98) 이고 `T_arm` 의 시간 lead 는 등속 성분만 보상한다. `q_cmd` 만 그 지연에 통과시킨 예측이 24.7 mm (실측 16.7), `closed_form` 3.8 mm (2.5) 다. `mpc` 안에서 간격과 명령 가속도의 상관은 0.85 이고, 두 planner 의 손–공 간격 차 (29.7 vs 17.2 mm) 는 이 항의 차와 같은 크기다.
  - 가속 중인 이유: `mpc` 는 공 속도 (3.2 m/s) 전부를 목표로 하고 (`catch.gamma_ref` 1.0 — `closed_form` 은 $\gamma_f$ 0.69 배), 명령이 움직인 시간이 0.286 s 로 0.1 s 짧다 (`closed_form` 0.384 s). 비용에 포구 노드의 가속 항이 없다.
  - 시간축. steady clock 위의 tick 간격 오차가 sim 에서 p01 −1.2 · p99 +2.2 ms 다 (0.5 ms 초과 7.8 %). `mpc` 명령의 한 tick 속도 계단 (> 0.06 rad/s) 279 개 중 99 % 가 간격이 0.3 ms 넘게 어긋난 tick 바로 다음이고 (전체 tick 의 9 %), 구간 전환에서 생긴 것은 31 개다. `closed_form` 은 같은 창에서 2 개다 — soft-catch 기준은 tick 마다 적분해 어긋남을 걸러낸다. 성공률에 주는 영향과 실기에서의 크기는 재지 않았다.
- security review (E-8, 2026-10-02): 보고할 취약점 없음. 구간의 모양을 읽는 새 인덱싱 (`n_pre` 로 시작하는 작업공간 검사, 전환 때의 포구 노드 위치, 계획기의 코어 슬롯) 은 모두 `ValidateDecelNodes` 와 샘플러의 모양 검사 뒤에서만 돈다 ([#662](https://github.com/hyujun/rtc-framework/issues/662#issuecomment-5943640738)).
- 말하지 못하는 것. 제어 PC 의 tick 시간. 격리된 코어에서의 꼬리. p1b 의 seed 하나 (50 발) 밖의 성공률. 원자료: `~/rtc_eval/e1-f09/`, 도구: 에이전트 private plan 의 `mpc-e1-f09-tools`.

### E1-F05 — 로그 · 도구 (2026-10-02, [#631](https://github.com/hyujun/rtc-framework/issues/631))

판정은 테스트와 "임시 패치 없이 읽히는가" 이고, 아래 sim 수치는 도구가 내는 값의 기록이다 — 성공률과 tick 은 같은 날의 대조가 없어 비교하지 않는다.

- **제어 동작 불변.** RT tick 과 tick record (POD) 는 건드리지 않았다 — CSV writer (log drain) 와 계획기 레코드의 두 필드뿐이다. `closed_form` 의 NormalTrial 명령 digest 는 `58e18c86679c92d6` 그대로이고 (`test_catching_supervisor_scenarios` 의 기록값을 읽어 대조 — 그 테스트는 digest 를 단언하지 않는다), `mode: mpc` 의 시나리오 · lane 테스트 단언은 바꾸지 않았다. lane 테스트에서 바뀐 것은 MD-70 으로 나오지 않게 된 결과 `not_due` 를 "기다린 결과" 집합에서 뺀 것 하나다 (PROC-6 별도 커밋).
- **기존 자료의 재현** (`~/rtc_eval/e1-f09`, 도구를 다시 돌림). `closed_form` unit 의 중앙값은 저장된 요약과 한 자리도 다르지 않다 (`clik_mm` 1.973 · `ref_vs_true_mm` 14.54 · `servo_mm` 2.52). `mpc` unit 에서 `catching_trials` 의 새 열이 private 스크립트의 값을 그대로 낸다: 채택 271 · 교체 207 · 전환 271 · `catch_box` 거부 74 · 게이트 거부 0 · 나이 0, 시행당 구간 p50 6 (2 – 6), 전환 $\rho$ 최대 0.589, 명령 2.58 m/s · $d\lvert v\rvert/dt$ +22.6 m/s² · 움직인 시간 0.286 s (`closed_form` 2.19 · −7.1 · 0.384).
- **E1-F09 기록의 정정.** node 0 대기 시간 (위 "E1-F09" 에 반영): plan 채택부터 첫 전환까지 p50 68 ms (최대 128 ms), 명령이 실제로 움직이기 시작하기까지 p50 84 ms. 그 unit 의 `clik_mm` 913.8 · `ref_vs_true_mm` 916.6 은 soft-catch 기준의 0 벡터를 기준으로 읽은 값이었다 — 지금은 구간의 목표가 없는 로그에서 두 값을 NaN (`ref_source` `none`) 으로 낸다.
- **sim 확인** (ur5e_p1b, `s35b` 10 발, seed 611, `catch_lead_on` + `mode: mpc`, 커밋 `190808c5` — dirty 0, 임시 패치 없음; host-watch 통과, 시행별 RTF 최솟값 0.98). 새 열이 닫힌 루프에서 기록되고 (`catching_diag.csv` 139 열 중 decel 17, `planner_events.csv` 78 열) 10 발 모두 `ref_source` `segment` 로 분해됐다:

  | $t_c$ 의 분해, 중앙값 (10 발) | `mpc` |
  |---|---|
  | 계획: 구간 기준 − 실제 공 `ref_vs_true_mm` | 13.2 |
  | CLIK: 명령 − 구간 기준 `clik_mm` | 4.6 |
  | 서보: 측정 − 명령 `servo_mm` | 14.0 |
  | 손–공 `total_mm` | 24.3 |
  | 명령 속력 / $d\lvert v\rvert/dt$ / 가속 | 2.69 m/s / +18.9 / 22.4 m/s² |
  | node 0 대기 / 명령이 움직인 시간 | 74 ms / 0.288 s |

  - lane (10 발): 채택 56 · 교체 38 · 전환 56 · 대기 37 회 (최장 12 tick) · `catch_box` 거부 17 · 게이트 거부 0 · 나이 0, 재계획 전환 $\rho$ 최대 0.279.
  - 계획기: 첫 풀이 11 회 중 게시 10 · 보류 1. 보류된 wake 의 행은 `search_valid` 1 · `plan_valid` 0 · `outcome` `held` · `decel_outcome` `catch_error` 이고 **포구 위치 오차는 35.5 mm** (상한 20 mm) 다 — E1-F08 이 읽지 못한 값이다. 첫 풀이의 $x_0$ 속도는 11 회 모두 0.
  - 포구 노드 (해에서 FK 로 다시 평가): 게시된 첫 풀이 10 개의 $\gamma$ 는 0.48 – 0.57, 상대속도 1.42 – 1.88 m/s, 위치 오차 0.8 – 9.1 mm. 포구 항을 가진 게시 구간 전체에서는 위치 오차 p50 8.1 mm (최대 19.8), $\gamma$ p50 0.75, 상대속도 p50 0.82 m/s 다. 속도 목표는 `catch.gamma_ref` 1.0 이다 — 해의 $\gamma$ 가 그보다 낮다는 것만 적고 해석은 E1-F10 에 둔다.
  - 풀이 시간 (ms, p50 / 최대): 첫 풀이 9.3 / 13.2 · 같은 격자점 2.4 / 4.1 · 격자 전진 4.4 / 9.5 · 정지 코어 1.9 / 5.0.
  - GUI: Catching 탭에 세 패널이 있고 `decel law: mpc` 가 컨트롤러의 미러에서 읽혀 표시된다 (화면 확인).
- G3-D 의 두 비율 (같은 unit): `search_valid_ratio` p50 0.25 (탐색이 돈 wake 중 유효한 plan 을 낸 비율 — 239 행 중 탐색이 돈 것은 107), `pair_published_ratio` p50 1.0 (시도한 첫 구간 중 게시된 비율; 11 회 중 10). `plan_valid_ratio` 는 모든 행이 분모라 0.17 이다.
- code review ([#631 코멘트](https://github.com/hyujun/rtc-framework/issues/631#issuecomment-5945207908)): 10 건, 모두 반영. 그중 측정의 뜻에 닿는 것은 위 두 비율의 분모다 (처음 구현은 모든 wake 로 나눴다).
- 하네스: `verify-changes.sh --run` 이 변경 패키지를 이름순으로 빌드 · 테스트해, 상류 헤더와 하류를 함께 바꾼 첫 실행에서 하류가 옛 상류 라이브러리로 테스트됐다 (거짓 red). 의존 순서로 고쳤고, 이번 실행에서 빌드가 끝나지 않은 패키지에 의존하는 패키지는 검증하지 않는다.
- 말하지 못하는 것. 10 발 한 unit 이다 — 성공률 (5 / 10) 과 위 중앙값은 분포의 추정이 아니다. `closed_form` 의 같은 날 대조는 돌리지 않았다. iiwa7_leap 은 돌리지 않았다. 원자료: `~/rtc_eval/e1-f05/`.

## 9. 개정 이력

결정의 내용과 날짜는 §4 가, 측정은 §8 이 갖는다. 이 표는 판마다 무엇이 바뀌었는지만 적는다.

| 판 | 바뀐 것 |
|---|---|
| r26 | E1-F05 완료 반영 ([#678](https://github.com/hyujun/rtc-framework/pull/678)) — 상태줄 · §6 의 표 · 브랜치 계획 (`main` 에서 실패한다던 테스트는 99c2188e 로 이미 통과했다 — 그 문장 삭제), §8 "E1-F05" (도구의 확인, code review, 하네스), §8 "E1-F09" 의 node 0 대기 시간을 제자리에서 정정, 미결에서 E1-F05 항목을 빼고 `decel_*` 이름 통일 · 항별 비용 분해 · GUI 의 lane 상태 · `catching_arm_budget` 의 `mpc` 기준을 넣음 |
| r25 | §8 에 $t_c$ 간격의 분해 (명령 가속도 × 서보 지연, 구간의 샘플 시각과 sim 의 tick 간격), 미결의 E1-F10 항목을 그 결과로 다시 씀 — 노드 사이 보간은 이미 있음을 명시 |
| r24 | E1-F09 머지 뒤 정리 — E1-F05 를 다음 차례로 표시하고 넘긴 범위를 표에 적음, G-1 회귀 기준이 가리키는 테스트를 명시, §2 에 구현 원칙 (2026-10-02 사용자 결정 — 그때 #662 에만 적혀 있었다) |
| r23 | E1-F09 완료 반영 ([#674](https://github.com/hyujun/rtc-framework/pull/674)) — 상태줄 · §6 의 표 · 브랜치 계획 갱신, security review 결과 (§8), 미결에 E1-F05 · F06 · F10 으로 넘긴 것 추가 |
| r22 | 결정 MD-70: 정지 구간만 푸는 계획기와 `replan.t_pre_s` 삭제 (사용자 결정 2026-10-02). 미결에서 그 항목 제거 |
| r21 | E1-F09 구현: 결정 MD-65 – MD-69 (`mpc` 는 언제나 APPROACH – HOLD 추종 · 쌍 채택 · 대기 슬롯의 교체와 나이 · track 과 정지 부분의 작업공간 검사 · node 0 전 유지 · 보고 범위), `shadow` 삭제, 측정 §8 (sim: abort 0, tick 은 `closed_form` 과 같음, 성공률은 낮음). 미결에 정지 구간 전용 계획기의 삭제 여부 추가 |
| r20 | E1-F08 완료 반영 ([#673](https://github.com/hyujun/rtc-framework/pull/673)) — 상태줄 · §6 의 표 · 브랜치 계획 갱신 |
| r19 | E1-F08 구현: 결정 MD-55 – MD-64 (포구 전 격자는 켜는 값, 쌍 게시, RT 보고에서 출발하는 재계획, 간격이 둘인 payload, shadow), 측정 §8 (sim 실시계: 예산 확정, leap 의 포구 전 시간 부족), code review 반영. MD-52 의 키 이름 정정 (`planner.budget.sigma_trk`) |
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
