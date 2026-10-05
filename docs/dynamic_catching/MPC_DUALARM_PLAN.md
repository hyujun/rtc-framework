# MPC · dual-arm catching 과 실기 단계 — 계획

- 상태: **E0 완료 · E1 진행 중 (단일 팔 MPC 의 E1-F01 – F11 과 계획기 interface 의 E1-F12 는 끝났고 NLP search · mpc_docking 의 E1-F13 – F21 이 남았다 — E1-F13 이 다음) · E2 진행 중 (E2-F04 가 다음 — E1 과 병행할 수 있다)** · E3 · 실기 대기. 이 줄은 epic 의 상태만 적는다 — feature 의 상태는 §3 의 표가 갖는다 (§2 "상태는 한 곳에만")
- 범위: 단일 팔 MPC (ur5e_p1b · iiwa7_leap, APPROACH–정지 — 끝났다) 와 그 위의 NLP search · mpc_docking (E1 의 남은 feature) · G1 + proto_1b bring-up 과 QP 다중 frame CLIK (E2) → 같은 MPC 에 dual arm · waist 항 추가 (E3, g1_p1b) → 실기 단계 (`ur5e_p1b`)

**이 문서는 구현이 끝나면 지우는 파일이다.** 상태 · 순서 · 남은 일의 범위 · 아직 정하지 않은 것 · 관리 규칙만 갖고, 영구히 보관할 정보는 갖지 않는다. 영구 정보의 자리:

| 무엇 | 어디에 |
|---|---|
| 수학 · 구조 (지금 구현의 서술) | [ref/](ref/) — [mpc_multiframe_clik_formulation.md](ref/mpc_multiframe_clik_formulation.md) (구현한 것과 아직 구현하지 않은 설계의 구분은 그 문서 §0), [CATCHING_MASTER.md](ref/CATCHING_MASTER.md), `L0` – `L8` |
| 결정 ID (`MD-n` · `D-n` · `G…`) 의 뜻, 단계 · feature 의 이름과 이슈 번호, 옛 절 인용의 새 자리 | [ID_INDEX.md](ID_INDEX.md) (§1 · §3 · §4) |
| 측정 · 검증 기록 | 각 feature 이슈의 코멘트 ("측정 · 검증 기록" 으로 시작하는 코멘트가 이 문서에 있던 기록이다) |
| `MD-n` 의 근거와 경위 (옮기기 전 결정 로그의 원문) | [#705 의 기록 코멘트](https://github.com/hyujun/rtc-framework/issues/705#issuecomment-5974506385) — 고치지 않는 기록이다 |
| 미결 항목 | 이슈 — §5 |

## 1. 우선순위와 진행 판단

| Epic | 구분 | 내용 |
|---|---|---|
| E0 | feature 가 모두 끝남 | 기반 정비 |
| **E1** | **필수** | 단일 팔 MPC (ur5e_p1b · iiwa7_leap 의 팔 기준을 APPROACH 부터 정지까지 MPC 로) 는 끝났다. 남은 것은 단일 arm-hand 의 두 모듈 — 포구 후보를 NLP 로 고르는 탐색 (`nlp`) 과 inner-loop NMPC planner (`mpc_docking`) — 과 기존 탐색 · planner 와의 비교다 |
| **E2** | **필수** | g1_p1b 의 `demo_joint_controller` · `demo_dualarm_controller` |
| E3 | 필수 (E2 뒤) | 같은 MPC 에 dual arm · waist 항 추가 — g1_p1b |
| 실기 (HW) | 필수 · sim 전용이 아니다 | 실기 단계 — 1차 목표 로봇은 `ur5e_p1b` ([#613](https://github.com/hyujun/rtc-framework/issues/613)) |

- **E1 의 남은 feature 를 먼저 한다 (E1-F13 부터).** E2 는 E1 의 선행이 아니고 고치는 패키지가 달라 (E2-F04 는 `rtc_tsid`) 다른 세션에서 병행할 수 있다. E3 는 E2 (g1_p1b 준비) 와 E1-F07 (코어) 이 끝나면 착수하고, E3 의 첫 단계는 `Decel*` 이름의 rename refactor 다 (MD-48). 그 rename 은 E1-F12 (interface) 뒤에 한다 — 같은 파일을 고친다.
- **설계 (MD-46 · MD-47).** MPC 는 waist + dual arm (G1) 용으로 설계한다 (formulation §1.3). 단일 팔은 같은 MPC 에서 dual arm · waist 전용 항과 제약만 뺀 구성이었고, g1_p1b 는 같은 코어에 그 항을 더한다 (E3). closed_form 과 mpc 는 입력 (추정기의 공 미래 궤적) 과 출력 (CLIK 입력) 이 같은 두 planner 이고 추정기 · supervisor · 손 시퀀서 · CLIK · `ABORT_SAFE` · E-STOP 은 공통이다.
- **E1 의 남은 feature.** 탐색 하나 (`nlp`) 와 planner 하나 (`mpc_docking`) 를 더한다. 설계 자료는 [ref/ball_catching_inverse_dynamics_mpc.md](ref/ball_catching_inverse_dynamics_mpc.md) 다 (§10 이 planner, §11 이 탐색 — 아직 구현을 서술하지 않는다). 수치 코어는 하나다: `mpc_docking` 은 그 코어로 구간을 풀고 `nlp` 는 같은 코어로 후보를 평가한다. 둘 다 계획기 스레드 안에서 돌고, 위의 공통부와 계획기 → RT 계약의 형태, 출하 기본값은 바꾸지 않는다. 코드는 로봇을 모르게 쓰고 시험은 `ur5e_p1b` · `iiwa7_leap` 둘에서 한다.
- 실기와 E2 · E3 의 선후는 이 문서가 정하지 않았다 (§5).

### 게이트 G-1 — 단일 팔 mpc planner 가 closed_form planner 와 비슷한 성능을 내는가

두 planner 만 다른 paired A/B 시험으로 "closed_form 대비 비열등" 을 판정하는 게이트다 (정의: formulation §6.5). **결과: 두 로봇 FAIL** ([#632](https://github.com/hyujun/rtc-framework/issues/632) — 한계 · 시행 수 · seed · 수치는 거기에 있다). 남은 순서에 주는 뜻:

- G-1 은 E3 의 착수 조건이 아니다 (MD-47). E3 의 항은 단일 팔 결과와 무관하게 G1 에서 필요하다.
- 사용자는 FAIL 을 알고 두 로봇의 출하 DECEL 법칙을 `mpc` 로 정했다 (MD-89). 따라서 `mpc` 가 출하 기본값이지만 closed_form 대비 비열등은 확인되지 않았고, lead 를 끈 출하 구성으로 잰 적도 없다.
- `iiwa7_leap` 은 `mpc` 가 포구 계획을 거의 내지 못한다 — 구조 문제가 열려 있다 (#710).

## 2. 관리 방식

| 층 | 위치 | 담는 것 |
|---|---|---|
| 전체 계획 | 이 문서 | 상태, epic · feature 표, 브랜치 계획, 남은 feature 를 구속하는 결정, 아직 정하지 않은 것 |
| epic · feature 세부 | 각 에이전트의 private plan (repo 에 커밋하지 않는다). **착수할 때 만든다** — 미리 만들지 않는다 | spec, Sprint Contract, 진행 기록, handoff |
| 추적 · 인계 | GitHub project [rtc-framework — MPC · dual-arm catching](https://github.com/users/hyujun/projects/2) | epic · feature 이슈, 보드의 Status, 완료 · 결정 변경 코멘트, 측정 · 검증 기록 |

- 이슈 제목은 `[EPIC] E<n> — …`, `[FEATURE] E<n>-F<nn> — …` 이고 feature 는 epic 의 sub-issue 다. 실기 epic 의 이슈는 [#613](https://github.com/hyujun/rtc-framework/issues/613) 이고 feature 이슈는 착수할 때 만든다 (§3).
- label: `type:epic` / `type:feature`, `area:mpc-dualarm`, 필수면 `priority:p0`, 조건부면 `conditional`, sim 전용이면 `sim-only`.
- feature 를 끝내면 이슈의 Done when 을 항목별로 갱신한다. **결정의 이유는 코드 주석과 `ref/` 에, 경위는 이슈에 적는다. 새 `MD-n` 을 만들지 않는다.**
- **상태는 한 곳에만 적는다.** feature 의 상태 (완료 · 다음 · 대기, PR, 결과 한 줄) 는 §3 feature 표의 상태 열이, epic 의 상태는 문서 머리의 상태줄이 갖는다. 브랜치 계획과 순서 표, epic 이슈의 본문, project 의 readme 에는 상태를 적지 않는다 — epic 의 feature 목록과 열림 · 닫힘은 sub-issue 가 보여 주고, 보드의 Done 은 이슈가 닫힐 때 자동으로 옮겨진다. feature 를 끝낼 때 손으로 고치는 곳은 셋이다: §3 의 그 행 (다음 feature 의 행에 "다음" 을 옮긴다), 그 feature 이슈의 Done when, epic 이슈의 머지 코멘트.
- feature 착수 전에 Sprint Contract 를 제시하고 컨펌받는다 (AGENTS.md §6.5). 브랜치는 비슷한 feature 를 묶어 만든다 (§3 "브랜치 계획").
- 수치로 판정하는 게이트는 기준과 N 을 시행 전에 고정하고, 기준을 본 뒤에 바꾸지 않는다. 미정인 값으로 판정하면 `PASS(provisional)` 로 표기한다.
- **구현 원칙 (사용자 결정).** 가장 우선은 계획한 알고리즘을 정확하게 구현하는 것이다. spec 을 바꿀 필요는 정확히 구현한 뒤 테스트 · 측정에서 드러날 때 정한다 — 구현 중에 시간이나 성공률을 추정해 spec 을 미리 줄이지 않는다.
- **참고 문서를 구현하는 feature 는 구현 대조표를 낸다 (사용자 결정).** `ref/` 의 설계 자료를 구현하는 feature (지금은 E1-F13 · F14 · F16 · F17) 는 사용자가 "정확하게 구현됐는가" 를 직접 확인할 수 있게, 참고 문서의 수학 · 구현할 알고리즘 · 구현한 것을 나란히 놓은 표를 그 feature 이슈의 코멘트로 낸다.
  - 행은 참고 문서의 식 · 제약 · 비용 항 하나씩이다. 그 feature 가 맡은 절의 식은 빠짐없이 행으로 둔다 — 구현하지 않는 것도 "구현하지 않음" 과 이유를 적어 남긴다.
  - 착수 때 (Sprint Contract 와 함께): 참고 문서의 수학 (절 · 식) ↔ 구현할 알고리즘 (이산화 · 선형화 · 행의 형태 · 자료 구조 · 호출 순서) ↔ 다르게 하는 곳과 이유. 다르게 하는 곳은 이 표로 승인을 받는다.
  - 완료 때 (PR 을 올리기 전): 같은 행에 구현한 것 (파일 · 함수, 코드가 실제로 계산하는 식) · 착수 때의 계획과 달라진 곳 · 그 행을 확인하는 테스트를 더한다.
  - 대조표는 이슈에 남고, `ref/` 는 구현한 식으로 고쳐 쓴다.
- 기존 test assertion 은 약화하지 않는다 (PROC-6). RT 경로 (`Compute`, 샘플러, CLIK) 는 할당 0 을 게이트로 확인한다. 실험 overlay 와 원자료는 repo 밖에 둔다.

## 3. Epic · Feature

**끝난 것.** E0 (기반 정비), E1 의 단일 팔 MPC (E1-F01 – F11) 와 계획기 interface (E1-F12) 는 끝났다 — feature 와 이슈는 [ID_INDEX.md](ID_INDEX.md) §3. E2-F01 – F03 (G1 자산 · config · launch · `demo_joint_controller` 의 G1 구동) 도 끝났다 — 같은 곳.

### E1. 단일 팔 — NLP search · mpc_docking — [#621](https://github.com/hyujun/rtc-framework/issues/621) · 필수

sim 전용. 게이트: 새 탐색 · planner 를 기존 것과 같은 투척으로 비교한 판정이 나고 (E1-F20), 그 결과로 출하 기본값을 바꿀지 정한다 — PASS 가 조건이 아니다. feature 의 범위와 Done when 은 각 이슈가 갖는다. 참고 문서를 구현하는 feature (E1-F13 · F14 · F16 · F17) 는 구현 대조표를 낸다 (§2). 표의 순서가 진행 순서다 (E1-F15 는 E1-F13 · F14 와 병행한다).

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E1-F13 | [#739](https://github.com/hyujun/rtc-framework/issues/739) | mpc_docking 수치 코어 — 상대상태 · corridor · 확률 제약 · 토크 판정의 NLP (ProxQP 위 SQP) | E1-F12 (끝) | **다음** |
| E1-F14 | [#740](https://github.com/hyujun/rtc-framework/issues/740) | NLP search 코어 — 후보별 NLP 풀이로 포구 후보를 고른다 (바깥 루프) | E1-F13 | 대기 |
| E1-F15 | [#741](https://github.com/hyujun/rtc-framework/issues/741) | mpc_docking 의 입력 식별 (sim) — 포획 기하 · 속도 집합 · 폐쇄 창 | — (E1-F16 앞에 끝낸다) | 대기 |
| E1-F16 | [#742](https://github.com/hyujun/rtc-framework/issues/742) | 계획기 스레드 통합 — search · planner 선택 키, YAML 조각, `PlannerCycle` 배선 | E1-F12 (끝) · F13 · F14, E1-F15 의 값 | 대기 |
| E1-F17 | [#743](https://github.com/hyujun/rtc-framework/issues/743) | RT 계약 · supervisor — mpc_docking 구간의 추종 | E1-F16 | 대기 |
| E1-F18 | [#744](https://github.com/hyujun/rtc-framework/issues/744) | 로그 · plot_rtc_log · demo_controller_gui | E1-F17 | 대기 |
| E1-F21 | [#747](https://github.com/hyujun/rtc-framework/issues/747) | 포구 가능 판정 지도 — 탐색이 어떤 공을 받는다고 판정하는가 (발사 위치 · 거리 · 비행시간 · 종단 속도). 오프라인 지도와 sim 의 판정 대 결과, 기술 통계로 보고한다 | E1-F16, sim 쪽은 E1-F17 · F18 | 대기 |
| E1-F19 | [#745](https://github.com/hyujun/rtc-framework/issues/745) | 튜닝 (판정 전, 판정과 다른 seed) | E1-F18 · F21 | 대기 |
| E1-F20 | [#746](https://github.com/hyujun/rtc-framework/issues/746) | 비교 평가 — search 둘 (`grid` · `nlp`), planner 셋 (`closed_form` · `mpc` · `mpc_docking`) | E1-F19, [#717](https://github.com/hyujun/rtc-framework/issues/717) | 대기 |

### E2. G1 + proto_1b bring-up — [#622](https://github.com/hyujun/rtc-framework/issues/622) · 필수

sim 전용. 게이트: G1 sim 에서 두 컨트롤러가 GUI 로 구동되고 formulation §4 sanity check 가 통과한다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E2-F04 | [#636](https://github.com/hyujun/rtc-framework/issues/636) | CLIK 다중 frame 일반화 (`rtc_tsid`) | E0-F03 (끝) | **다음** (E2 안에서) — 선행은 끝났다. E1 과 병행할 수 있다 |
| E2-F05 | [#637](https://github.com/hyujun/rtc-framework/issues/637) | demo_dualarm_controller — QP 다중 frame CLIK 바인딩 | E2-F03 (끝), E2-F04 | 대기 |
| E2-F06 | [#638](https://github.com/hyujun/rtc-framework/issues/638) | demo_controller_gui — G1 profile · 다중 frame 목표 | E2-F05 | 대기 |
| E2-F07 | [#639](https://github.com/hyujun/rtc-framework/issues/639) | plot_rtc_log — 다중 device group · frame 별 task error | E2-F05 | 대기 |

### E3. MPC dual arm · waist 확장 — [#623](https://github.com/hyujun/rtc-framework/issues/623) · 필수 (E2 뒤)

같은 MPC 에 dual arm · waist 항을 더한다 (MD-46 · MD-47). 착수 조건: E2 (g1_p1b 준비) 와 E1-F07 (코어, 끝). 첫 단계는 `Decel*` 이름의 rename refactor 다 (MD-48, #711) — 그 앞에 두기로 한 E1-F12 는 끝났다. sim 전용. 게이트: G1 sim 에서 포구 시행이 돌고 성공률 · solve time p99 가 보고된다. 기존 두 로봇 회귀 없음 — 단일 팔 구성 (더한 항의 가중 0) 의 해가 불변이다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E3-F01 | [#640](https://github.com/hyujun/rtc-framework/issues/640) | G1 구성의 포구 후보 선택 (바깥 루프 — MD-46 의 편차를 닫는다). 계획기 interface · search 선택 키 · 단일 팔의 바깥 루프는 E1-F12 · F14 · F16 으로 옮겼다 | E1-F08 (끝), E1-F12 (끝) · F14 | 대기 |
| E3-F02 | [#641](https://github.com/hyujun/rtc-framework/issues/641) | 전신 항을 E1 코어에 추가 — 왼팔 rest · waist 억제 · 각운동량 · waist 토크 행 · counter-swing, 관절군별 move blocking | E1-F07 (끝), E2-F01 (끝) | 대기 |
| E3-F03 | [#642](https://github.com/hyujun/rtc-framework/issues/642) | 충돌 제약 — capsule 모델 (신규 코어) 과 MPC 제약 행 (자기충돌 · 공–왼팔 거리) | E3-F02 | 대기 |
| E3-F04 | [#643](https://github.com/hyujun/rtc-framework/issues/643) | MPC ↔ CLIK 계약의 전신 확장 — payload 관절 용량 (8 → G1 $n$ 17), 왼손 FK, 한계 여유. 단일 팔 계약은 `ID_INDEX.md` MD-36 · `ref/L7_supervisor.md` §4.3a | E1-F08 (끝), E2-F04 | 대기 |
| E3-F05 | [#644](https://github.com/hyujun/rtc-framework/issues/644) | G1 통합 — supervisor · 손 시퀀서 · vision sim profile (`mode: mpc` 의 확장) | E3-F03, E3-F04, E2-F05 | 대기 |
| E3-F06 | [#645](https://github.com/hyujun/rtc-framework/issues/645) | 로그 · plot_rtc_log · demo_controller_gui — dual arm 열. 항별 비용 분해는 코어 변경이라 아직 없다 | E3-F05 | 대기 |
| E3-F07 | [#646](https://github.com/hyujun/rtc-framework/issues/646) | 평가 — G1 sim 포구 시행 · 예측 격자 sweep · waist · 왼팔 고정 대조 · 기존 로봇 회귀 | E3-F06 | 대기 |

### 실기 (HW) — [#613](https://github.com/hyujun/rtc-framework/issues/613) · 필수 · **sim 전용이 아니다**

1차 목표 로봇은 `ur5e_p1b`. 진행 순서는 bag replay (재스탬프) → 가상 공 → 저속 실투척 → 상향이다. feature 이슈는 착수할 때 만든다 (이슈 번호 칸 "—"). 체크리스트 (도구 · 사용자 결정 · 실기에서 재야 닫히는 것 · park 키) 는 #613 이 갖는다 — 이 문서는 그것을 복제하지 않는다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| HW-F01 | — | 단계 D — 실기 기본 측정. 게이트 G8-A · G5-F · G6-D · G6-E · D-S9-G · 운동 기한의 실기 값. 절차의 내용은 `ref/`: 지연 식별 `ref/L5_joint_cmd.md` §4.4 · §9, 손 식별 · `T_close,tot` `ref/L6_hand.md` §4 · §9, D-S9-G `ref/L7_supervisor.md` §4.1 · §10, G8-A `ref/L8_bringup.md` §9 | #613 의 도구 · 사용자 결정 중 이 단계가 쓰는 것 | 대기 |
| HW-F02 | — | 단계 E — bag replay · 가상 공. 게이트 G8-F (abort 경로 전부) · G7-F. 절차는 `ref/L8_bringup.md` §9.2, `ref/L7_supervisor.md` §9 | HW-F01, **실기 planner 결정** (아래), #613 의 도구 (bag 재스탬프 replay · 가상 공 발행기 · speed scaling · PTP 감시) | 대기 |
| HW-F03 | — | 단계 F — 실투척 (저속 → 상향). 게이트 G8-G ("원인 분해" 는 항목별 수치 기록으로 판정) | HW-F02 | 대기 |
| HW-F04 | — | GUI · plot 의 실기 항목 (실기 모드 표시, 실기 세션 CSV 의 plot). #613 이 순서를 정하지 않았다 | — | 대기 |

**실기 planner (선행 조건).** 사용자 결정: 지금은 planner 를 정하지 않는다. 실기 기본 측정 (단계 D) 은 planner 와 무관하게 진행하고, **단계 E 앞에서** `closed_form` 과 `mpc` 중 어느 쪽으로 갈지 정한다. 지금 실기 config 는 planner 를 덮지 않아 출하값 `supervisor.decel.mode: mpc` 가 그대로 실기에 간다. `mpc` 로 가려면 먼저 닫아야 하는 것 넷 (정지 구간과 작업셀 경계 — E-8 에 걸린다, G8-F 에 mpc 의 abort 경로, #654 의 ProxQP heap 할당 (RT-1 수용 예외 여부), `mpc` 를 실기에서 잰 적이 없다는 것) 은 #613 의 "planner" 절이 갖는다.

### 브랜치 계획

브랜치 하나가 PR 하나다. 이 절은 묶음과 순서만 적는다 — 어느 브랜치가 끝났는지와 그 PR 은 위 feature 표의 상태 열에 있다. 브랜치 이름은 착수 때 확정한다 (아래는 계획값). 실기의 브랜치는 feature 이슈를 만들 때 정한다.

| 기준 | 내용 |
|---|---|
| 묶는다 | 같은 패키지 · 같은 파일을 고치고 함께 있어야 동작을 확인할 수 있는 feature. 같은 종류의 도구 작업 (로그 · plot · GUI) 과 같은 측정 도구를 쓰는 평가 |
| 따로 둔다 | escalation 이 Critical 인 feature (E-8) · public API 를 바꾸거나 abstract interface 를 도입하는 feature (기능 동등성을 그 PR 만으로 판정) · 신규 수치 코어 (code review 의 단위) · 게이트 판정 |

- 브랜치 이름은 `type/slug` (`type` 은 Conventional Commits 의 type). 브랜치는 선행 브랜치가 `main` 에 merge 된 뒤 `main` 에서 만든다 — PR 을 쌓지 않는다.
- PR 본문에 닫는 feature 이슈를 모두 적는다. 한 브랜치 안에서 feature 마다 커밋을 나누고 Sprint Contract 와 Done when 은 feature 별로 판정한다. 묶은 feature 가 착수 뒤에 "따로 둔다" 기준에 걸리면 브랜치를 나눈다.

| 브랜치 | feature | 묶은 이유 · 나누는 조건 |
|---|---|---|
| `feat/catching-docking-core` | E1-F13 | 신규 수치 코어. code review 단위 |
| `feat/catching-nlp-search-core` | E1-F14 | 신규 수치 코어. code review 단위 |
| `feat/catching-docking-ident` | E1-F15 | `rtc_tools` · `integrated_bringup/tools` 만 고친다 — 코어 둘과 병행한다 |
| `feat/catching-docking-planner` | E1-F16 | 선택 키 · YAML 조각 · 배선 |
| `feat/catching-docking-rt` | E1-F17 | RT 법칙. E-STOP · reset 경로를 건드리면 E-8 |
| `feat/catching-docking-tooling` | E1-F18 | 로그 · plot · GUI |
| `feat/catching-search-verdict-map` | E1-F21 | 판정 지도의 도구와 측정. **나누는 조건**: `grid` 의 오프라인 지도를 먼저 만들면 브랜치를 나눈다 (그 선행인 E1-F12 는 끝났다) |
| `feat/catching-docking-tune` | E1-F19 | 튜닝. 판정과 한 PR 에 섞지 않는다 |
| `docs/catching-docking-eval` | E1-F20 | 게이트 판정 |
| `feat/tsid-clik-multiframe` | E2-F04 | `rtc_tsid` public API 변경. 기존 소비자 둘의 기능 동등성이 성공 기준이다 |
| `feat/demo-dualarm-controller` | E2-F05 | 신규 controller. Sprint Contract = spec |
| `feat/g1-dualarm-tooling` | E2-F06, E2-F07 | GUI 와 plot. 둘 다 `demo_dualarm_controller` 의 출력을 소비한다 |
| (rename refactor) | — | `Decel*` 이름 (MD-48, #711) — feature 가 아니다. 기능 동등성이 성공 기준이고 public YAML key 를 건드리므로 별도 PR. 그 앞에 두기로 한 E1-F12 (interface) 는 끝났다 |
| `feat/catching-mpc-candidate-select` | E3-F01 | G1 구성의 후보 선택. interface 는 E1-F12 가 넣었다 |
| `feat/catching-mpc-wholebody` | E3-F02, E3-F04 | 전신 항과 MPC ↔ CLIK 계약. 둘 다 활성 관절을 전체로 넓히는 작업이고 계약의 sanity check 가 전신 해를 입력으로 쓴다 |
| `feat/collision-capsule-core` | E3-F03 | 신규 수치 코어. code review 단위 |
| `feat/g1-catch-controller` | E3-F05 | supervisor · 손 시퀀서 통합. E-STOP 경로를 건드리면 E-8 |
| `feat/g1-catch-tooling-eval` | E3-F06, E3-F07 | 로그 · plot · GUI 와 평가. 평가가 새 로그 컬럼을 쓴다. **나누는 조건**: 평가 결과가 설계 결정을 바꾸면 결과 기록을 `docs/` 브랜치로 분리한다 |

**순서 — E1.** 위 표의 순서대로다: interface → 코어 둘 → planner 통합 → RT → tooling → 판정 지도 → 튜닝 → 평가. `feat/catching-docking-ident` 는 코어 둘과 병행하고 planner 통합 앞에 끝낸다. E2 의 브랜치는 E1 의 브랜치와 병행할 수 있다 (패키지가 다르다).

**순서 — E2 · E3.** (1) `feat/tsid-clik-multiframe` → (2) `feat/demo-dualarm-controller` → `feat/g1-dualarm-tooling` → (3) rename refactor → E3 의 다섯 브랜치 (E2 와 E1-F07 뒤, MD-47). 병행은 서로 다른 패키지를 고치는 브랜치끼리만 한다.

### 게이트 · escalation

| Feature | 사유 | 효력 |
|---|---|---|
| E1-F13 | solver 의 heap 할당 (RT-1 → E-1) — 계획기 스레드는 FIFO 일 수 있고 ProxQP 의 할당은 수용된 예외가 아니다 (#654) | Critical — 착수 전 `[CONCERN]` 과 컨펌 |
| E1-F17 | E-STOP · reset 경로를 건드리면 E-8 | Critical — 착수 전 `[CONCERN]` 과 컨펌 |
| E3-F05 | E-STOP 경로를 건드리면 E-8 | Critical |
| 실기 (HW) | `mpc` 의 정지 구간과 작업셀 경계 (RT 가 `catch_box` 를 검사하지 않는다), `mpc` 구간의 샘플 시각을 바꾸는 RT 법칙 변경 — 둘 다 E-8 (#613) | Critical — 착수 전 `[CONCERN]` 과 컨펌 |
| E2-F04 | `rtc_tsid` public API 변경, 기존 소비자 둘 | code review, 기능 동등성이 성공 기준 |
| E1-F13 · F14 | 신규 수치 코어 (100+ 줄). E1-F14 는 두 번째 탐색 구현 | code review |
| E1-F16 | lifecycle 의 configure 경로 | security review 권고 |
| E3-F03 | 신규 수치 코어 (100+ 줄) | code review |
| E3 착수 | `Decel*` rename refactor (MD-48) — public YAML key | 별도 PR, 기능 동등성이 성공 기준 |
| E2-F05 | 신규 controller | Sprint Contract = spec |

## 4. 남은 feature 를 구속하는 결정

아직 구현하지 않은 feature 를 구속하는 `MD-n` 이다. 나머지 `MD-n` 은 [ID_INDEX.md](ID_INDEX.md) §4. E1-F13 – F21 을 구속하는 결정 (solver · 조합 · 선택 키 · 식의 형태 · 1 차 범위 · 판정 방식) 은 `MD-n` 이 아니다 — 각 feature 이슈의 범위에 적혀 있고 선택지와 이유는 [#621 의 결정 코멘트](https://github.com/hyujun/rtc-framework/issues/621#issuecomment-5974865100) 에 있다.

| ID | 무엇을 정했나 | feature |
|---|---|---|
| MD-2 | QP 다중 frame CLIK 컨트롤러는 새 config key `demo_dualarm_controller` 로 추가한다. 기존 `demo_task_controller` (DLS) 와 그 소비 로봇은 건드리지 않는다 | E2-F05 |
| MD-14 | CLIK 에 제동 거리 기반 속도 한계 (opt-in), 충돌 damper (선택), 오차 되먹임 상한을 둔다. 앞의 둘은 기본 꺼짐이다 (기존 로봇의 golden 회귀를 지키기 위해) | E2-F04 |
| MD-36 | 자세 과제의 속도 feedforward 는 자세 목표를 $q_{ref}+\dot q_{ref}/K_n$ 으로 넘겨 얻는다 (`rtc_tsid` 불변). 다중 frame CLIK 의 API 는 E2-F04 가 정한다 | E2-F04 |
| MD-86 | `sim_g1_p1b` 의 `enable_mpc` 는 CPU layout 만 고른다. G1 의 포구 컨트롤러가 오면 그 기본값을 그 feature 에서 다시 정한다 | E2-F05 · E3-F05 |
| MD-12 | 각운동량 항은 유지한다 — 목적은 왼팔의 운동 생성과 floating base 확장이고, 효과는 포구 성공률이 아니라 각운동량 변화율의 크기로 판정한다 | E3-F02 |
| MD-13 | 토크는 1차까지 선형화하고 토크 · 자기충돌 행에는 slack 을 둔다. 위치 · 속도 한계, 종단 등식, trust region, 공–왼팔 행은 hard 다. 동역학은 손을 기준 자세로 잠근 축소 모델로 계산한다 | E3-F02 · F03 |
| MD-46 | 설계는 formulation §1.3 하나다. g1_p1b 는 같은 코어에 dual arm · waist 전용 항 (waist 억제, 왼팔 rest, 각운동량, 자기충돌, 공–왼팔, waist 토크 행 · counter-swing, 관절군별 move blocking) 을 더한다. 포구 후보 ($t_c$) 를 MPC 의 바깥 루프가 고르게 하는 것이 닫아야 할 편차다 | E1-F14 · E3-F01 · F02 |
| MD-47 | E3 의 착수 조건은 E2 (g1_p1b 준비) 와 E1-F07 (코어) 이다. G-1 은 조건이 아니다 | E3 |
| MD-48 | `Decel*` 식별자의 rename 은 E3 착수 전에 별도 refactor 로 한다 | E3 착수 (E1-F12 뒤 — 끝났다) |
| MD-49 | 코어는 비용 · 제약을 항 단위로 조립하고 관절군별 move blocking 행렬 $E$ 의 자리만 둔다. 다관절군 일반화는 E3 에서 한다 | E3-F02 |

## 5. 아직 정하지 않은 것

열린 결정은 이슈가 갖는다. 제목은 이슈 제목이다.

**남은 feature 의 결정**

- [#746](https://github.com/hyujun/rtc-framework/issues/746) — E1-F20: 비교 평가 — 투척 모집단 (search 비교를 G-1 의 상자 밖에서도 볼지) 은 E1-F21 의 결과를 보고 시행 전에 정한다
- [#636](https://github.com/hyujun/rtc-framework/issues/636) — E2-F04: CLIK 다중 frame 일반화 (과제 종류 · 상대 Jacobian · 제동 거리 한계 등이 이 이슈의 열린 결정이다)
- [#640](https://github.com/hyujun/rtc-framework/issues/640) — E3-F01: G1 구성의 MPC 포구 후보 선택 (바깥 루프)
- [#642](https://github.com/hyujun/rtc-framework/issues/642) — E3-F03: 충돌 제약 — capsule 모델 · 자기충돌 · 공–왼팔 거리
- [#645](https://github.com/hyujun/rtc-framework/issues/645) — E3-F06: 로그 · plot_rtc_log · demo_controller_gui (dual arm 열)

**mpc planner · catching 의 남은 일**

- [#710](https://github.com/hyujun/rtc-framework/issues/710) — iiwa7_leap: `mode: mpc` 가 포구 계획을 내지 못한다 — 첫 풀이의 기준 궤적 (E1-F20 뒤에 다시 본다)
- [#711](https://github.com/hyujun/rtc-framework/issues/711) — refactor(catching): `Decel*` · `decel_*` 이름을 뜻에 맞게 — MD-48 의 rename, E3 착수 전 (선행인 E1-F12 는 끝났다)
- [#755](https://github.com/hyujun/rtc-framework/issues/755) — catching: 포구 층에서 발화하지 않는 `JOINT_CONFLICT` 경로 — 지울지 정한다 (#712 후속)
- [#713](https://github.com/hyujun/rtc-framework/issues/713) — catching: 조정되지 않은 채 출하된 mpc planner 의 값 — `cost.w_perp` · `publish.slack_terminal_max` (E1-F20 뒤)
- [#716](https://github.com/hyujun/rtc-framework/issues/716) — catching: formulation 의 포구 구간 다중 노드 (K_c) 와 그 위의 경로 이탈 항 `w_path` 가 구현에 없다 (E1-F20 뒤)
- [#715](https://github.com/hyujun/rtc-framework/issues/715) — integrated_bringup: 세 군 (두 번째 손) 지원 — 보류

**이 문서가 정하지 않은 것.** 실기 epic 과 E2 · E3 의 선후 (실기는 `ur5e_p1b` 이고 E2 · E3 는 G1 sim 이라 서로의 선행이 아니다). 실기 planner 는 단계 E 앞에서 정한다 (§3).
