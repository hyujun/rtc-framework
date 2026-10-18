# MPC · dual-arm catching 과 실기 단계 — 계획

- 상태: **E0 · E1 · E2 완료** · E3 · 실기 대기 (E3 앞에 남은 단독 이슈는 없다 — §5). 이 줄은 epic 의 상태만 적는다 — feature 의 상태는 §3 의 표가, 끝난 feature 의 이름은 [ID_INDEX.md](ID_INDEX.md) §3 이 갖는다 (§2 "상태는 한 곳에만")
- 범위: 단일 팔 MPC (ur5e_p1b · iiwa7_leap, APPROACH–정지 — 끝났다) 와 그 위의 NLP search · mpc_docking (E1 의 남은 feature) · G1 + proto_1b bring-up 과 QP 다중 frame CLIK (E2) → 같은 MPC 에 dual arm · waist 항 추가 (E3, g1_p1b) → 실기 단계 (`ur5e_p1b`)

**이 문서는 구현이 끝나면 지우는 파일이다.** 상태 · 순서 · 남은 일의 범위 · 아직 정하지 않은 것 · 관리 규칙만 갖고, 영구히 보관할 정보는 갖지 않는다. 영구 정보의 자리:

| 무엇 | 어디에 |
|---|---|
| 수학 · 구조 (지금 구현의 서술) | [ref/](ref/) — [mpc_multiframe_clik_formulation.md](ref/mpc_multiframe_clik_formulation.md) (구현한 것과 아직 구현하지 않은 설계의 구분은 그 문서 §0), [grid_search_closed_form_formulation.md](ref/grid_search_closed_form_formulation.md) (격자 탐색과 `closed_form` 구간의 수식 통합본), [CATCHING_MASTER.md](ref/CATCHING_MASTER.md), `L0` – `L8`, [ball_catching_inverse_dynamics_mpc.md](ref/ball_catching_inverse_dynamics_mpc.md) (§17 이 구현 — mpc_docking 수치 코어와 NLP search 의 탐색 코어, §0 – §16 은 설계 자료) |
| 결정 ID (`MD-n` · `D-n` · `G…`) 의 뜻, 단계 · feature 의 이름과 이슈 번호, 옛 절 인용의 새 자리 | [ID_INDEX.md](ID_INDEX.md) (§1 · §3 · §4) |
| 측정 · 검증 기록 | 각 feature 이슈의 코멘트 ("측정 · 검증 기록" 으로 시작하는 코멘트가 이 문서에 있던 기록이다) |
| `MD-n` 의 근거와 경위 (옮기기 전 결정 로그의 원문) | [#705 의 기록 코멘트](https://github.com/hyujun/rtc-framework/issues/705#issuecomment-5974506385) — 고치지 않는 기록이다 |
| 미결 항목 | 이슈 — §5 |

## 1. 우선순위와 진행 판단

| Epic | 구분 | 내용 |
|---|---|---|
| E0 | feature 가 모두 끝남 | 기반 정비 |
| E1 | feature 가 모두 끝남 | 단일 팔 MPC (ur5e_p1b · iiwa7_leap 의 팔 기준을 APPROACH 부터 정지까지 MPC 로), 단일 arm-hand 의 두 모듈 — 포구 후보를 NLP 로 고르는 탐색 (`nlp`) 과 inner-loop NMPC planner (`mpc_docking`) —, 그리고 기존 탐색 · planner 와 같은 투척으로 잰 기록. 출하 기본값은 두 로봇 모두 `grid` × `mpc` 그대로다 |
| E2 | feature 가 모두 끝남 | g1_p1b 의 `demo_joint_controller` · `demo_dualarm_controller` 와 그 GUI · plot |
| **E3** | **필수** (선행 E2 는 끝났다) | 같은 MPC 에 dual arm · waist 항 추가 — g1_p1b |
| 실기 (HW) | 필수 · sim 전용이 아니다 | 실기 단계 — 1차 목표 로봇은 `ur5e_p1b` ([#613](https://github.com/hyujun/rtc-framework/issues/613)) |

- **E1 은 끝났다.** 마지막 feature 인 비교 평가 E1-F20 은 판정이 아니라 지금 상태의 기록으로 끝났고 (2026-10-11, 남은 일은 §5 의 "E1 에서 넘어온 남은 일"), 그 기록을 본 뒤 **출하 기본값은 두 로봇 모두 `grid` × `mpc` 로 두었다** (사용자 결정, [#813](https://github.com/hyujun/rtc-framework/issues/813) — MD-89 를 잇는다). 그 앞의 단독 이슈 T ([#803](https://github.com/hyujun/rtc-framework/issues/803)) · [#807](https://github.com/hyujun/rtc-framework/issues/807) 은 끝났고, S ([#804](https://github.com/hyujun/rtc-framework/issues/804)) · R ([#805](https://github.com/hyujun/rtc-framework/issues/805)) 는 보류로 닫았다 (재개 조건은 각 이슈의 닫는 코멘트). E1 뒤에 남은 단독 이슈는 보류 중인 #715 하나다 (§5 의 순서표).
- **설계 (MD-46 · MD-47).** MPC 는 waist + dual arm (G1) 용으로 설계한다 (formulation §1.3). 단일 팔은 같은 MPC 에서 dual arm · waist 전용 항과 제약만 뺀 구성이었고, g1_p1b 는 같은 코어에 그 항을 더한다 (E3). closed_form 과 mpc 는 입력 (추정기의 공 미래 궤적) 과 출력 (CLIK 입력) 이 같은 두 planner 이고 추정기 · supervisor · 손 시퀀서 · CLIK · `ABORT_SAFE` · E-STOP 은 공통이다.
- **E1 이 넣은 것.** 탐색 하나 (`nlp`) 와 planner 하나 (`mpc_docking`) 는 코어 · 계획기 배선 (E1-F16) 과 RT 쪽 (E1-F17) 까지 들어왔다 — 로그 (E1-F18) · 판정 지도 (E1-F21) · 튜닝 (E1-F19) · 같은 투척으로 잰 기록 (E1-F20) 도 끝났다. 설계 자료는 [ref/ball_catching_inverse_dynamics_mpc.md](ref/ball_catching_inverse_dynamics_mpc.md) 다 (§10 이 planner, §11 이 탐색 — 구현한 것은 그 문서의 §17 에 적는다). 수치 코어는 하나이고 이미 있다 (`MpcDockingSegmentCore`, E1-F13): `mpc_docking` 은 그 코어로 구간을 풀고 `nlp` 는 같은 코어로 후보를 평가한다. 탐색 코어도 있고 (`NlpCatchSearch`, E1-F14) 계획기의 한 주기가 부른다 (E1-F16). 둘 다 계획기 스레드 안에서 돌고, 위의 공통부와 계획기 → RT 계약의 payload, 출하 기본값 (선택 키의 값) 은 바꾸지 않는다. **탐색은 RT 가 plan 을 채택한 뒤에도 돈다** — 처음 채택한 plan 의 $t_c$ 에서 `t_stop_plan` 앞까지, 공을 잡기 직전까지만이고 모든 조합에서다 (사용자 결정, §4). 그래서 출하 조합 `grid` × `mpc` 의 동작도 바뀐다: 채택 뒤의 wake 는 탐색을 먼저 돌리고, 탐색이 다른 plan 을 고르면 그 plan 과 첫 구간을 쌍으로 게시해 RT 가 APPROACH 중에 둘을 함께 바꾸며 (아니면 구간을 재계획한다), 손의 폐쇄 지령 시각은 commit 뒤에도 지령 전까지 다시 맞춰진다 (E1-F16 · F17 — L3 §5.3, L7 §4.3a, L6 §4.3). 코드는 로봇을 모르게 쓰고 시험은 `ur5e_p1b` · `iiwa7_leap` 둘에서 한다. 새 구현의 이름은 값 + interface 의 순서다 — 구간 계획기는 `<값>SegmentPlanner` · `<값>SegmentCore` (`MpcSegmentPlanner` · `MpcSegmentCore` 와 같다), 탐색은 `<값>CatchSearch` (`GridCatchSearch` 와 같다). `Decel*` 이름을 새로 쓰지 않는다 (MD-48).
- 실기와 E2 · E3 의 선후는 이 문서가 정하지 않았다 (§5).

### 게이트 G-1 — 단일 팔 mpc planner 가 closed_form planner 와 비슷한 성능을 내는가

두 planner 만 다른 paired A/B 시험으로 "closed_form 대비 비열등" 을 판정하는 게이트다 (정의: formulation §6.5). **결과: 두 로봇 FAIL** ([#632](https://github.com/hyujun/rtc-framework/issues/632) — 한계 · 시행 수 · seed · 수치는 거기에 있다). 남은 순서에 주는 뜻:

- G-1 은 E3 의 착수 조건이 아니다 (MD-47). E3 의 항은 단일 팔 결과와 무관하게 G1 에서 필요하다.
- 사용자는 FAIL 을 알고 두 로봇의 출하 DECEL 법칙을 `mpc` 로 정했다 (MD-89). 따라서 `mpc` 가 출하 기본값이지만 closed_form 대비 비열등은 확인되지 않았고, lead 를 끈 출하 구성으로 잰 적도 없다. G-1 은 탐색이 채택에서 멈추는 `mpc` 와 옛 폐쇄 지령 시각으로 쟀다 — E1-F16 · F17 뒤의 `grid` × `mpc` 는 채택 뒤에도 탐색해 APPROACH 중에 plan 을 교체할 수 있고, 폐쇄 지령 시각은 `T_close_lead` 로 바뀐 데다 (`iiwa7_leap`: $t_c-0.1037\to t_c-0.0577$) commit 뒤에 공의 통과 시각으로 다시 맞춰진다. 그 `mpc` 로 closed_form 과 견준 적은 없다.
- `iiwa7_leap` 의 `grid` × `mpc` 는 `grid` IK 의 두 값을 고친 뒤에야 포구 계획을 낸다 (#710 — `k_manip` 을 0 으로, `manipulability_min` 을 0.1 로, L3 부록 D). 그 전의 기록인 E1-F20 에서는 620 발 가운데 게시 0 이었고 (탐색 수용 537, `catch_error` 로 보류 527), sim 에서 던진 것은 하한까지 0 으로 둔 unit 하나이고 후보 상자 180 발 중 게시 79 · 포구 59 다 — 출하 값 (하한 0.1) 은 오프라인으로만 봤다. `nlp` × `mpc_docking` (포구 97 – 99) 과의 차이는 첫 풀이의 기준 쪽에 있었고, #819 가 기준을 자르는 문턱을 1.2 로 올려 줄였다 (§5). `ur5e_p1b` 의 `grid` × `mpc` 는 후보 상자에서 게시 98 / 360 · 포구 64 로 `closed_form` (60) 과 같은 수를 잡는다 — 잡는 투척은 다르다 (§5). 그 기록을 보고도 두 로봇의 기본값은 `grid` × `mpc` 로 두었다 (#813).

## 2. 관리 방식

| 층 | 위치 | 담는 것 |
|---|---|---|
| 전체 계획 | 이 문서 | 상태, epic · feature 표, 브랜치 계획, 남은 feature 를 구속하는 결정, 아직 정하지 않은 것 |
| epic · feature 세부 | 각 에이전트의 private plan (repo 에 커밋하지 않는다). **착수할 때 만든다** — 미리 만들지 않는다 | spec, Sprint Contract, 진행 기록, handoff |
| 추적 · 인계 | GitHub project [rtc-framework — MPC · dual-arm catching](https://github.com/users/hyujun/projects/2) | epic · feature 이슈, 보드의 Status, 완료 · 결정 변경 코멘트, 측정 · 검증 기록 |

- 이슈 제목은 `[EPIC] E<n> — …`, `[FEATURE] E<n>-F<nn> — …` 이고 feature 는 epic 의 sub-issue 다. 실기 epic 의 이슈는 [#613](https://github.com/hyujun/rtc-framework/issues/613) 이고 feature 이슈는 착수할 때 만든다 (§3).
- label: `type:epic` / `type:feature`, `area:mpc-dualarm`, 필수면 `priority:p0`, 조건부면 `conditional`, sim 전용이면 `sim-only`.
- feature 를 끝내면 이슈의 Done when 을 항목별로 갱신한다. **결정의 이유는 코드 주석과 `ref/` 에, 경위는 이슈에 적는다. 새 `MD-n` 을 만들지 않는다.**
- **상태는 한 곳에만 적는다.** feature 의 상태 (다음 · 대기, PR, 결과 한 줄) 는 §3 feature 표의 상태 열이, epic 의 상태는 문서 머리의 상태줄이 (feature 를 세지 않는다), **순서는 feature 면 §3 의 "순서" 줄이, feature 번호 없는 단독 이슈면 §5 의 순서표가** 갖는다 — 다른 절은 그 자리를 가리키고 다시 적지 않는다. 브랜치 계획과 순서 표, epic 이슈의 본문, project 의 readme, 그리고 **`ref/` 문서의 머리와 [README.md](README.md) 의 문서 표** 에는 상태를 적지 않는다 (그 둘은 어느 feature 가 무엇을 구현했는가만 적는다 — 남은 feature 와 그 상태를 적으면 feature 가 끝날 때 어긋난다. 문서 자신의 구현 범위, 곧 어느 절이 구현됐는가는 그 문서의 내용이다) — epic 의 feature 목록과 열림 · 닫힘은 sub-issue 가 보여 주고, 보드의 Done 은 이슈가 닫힐 때 자동으로 옮겨진다. feature 를 끝낼 때 손으로 고치는 곳은 넷이다: §3 의 그 행을 지우고 (끝난 feature 는 표에 없다) [ID_INDEX.md](ID_INDEX.md) §3 에 행을 더한다, 남긴 일이 있으면 §5, 그 feature 이슈의 Done when, epic 이슈의 머지 코멘트. 순서가 바뀌면 그 한 곳을 고친다.
- feature 착수 전에 Sprint Contract 를 제시하고 컨펌받는다 (AGENTS.md §6.5). 브랜치는 비슷한 feature 를 묶어 만든다 (§3 "브랜치 계획").
- 수치로 판정하는 게이트는 기준과 N 을 시행 전에 고정하고, 기준을 본 뒤에 바꾸지 않는다. 포구 수로 arm 을 가를 때는 seed 하나로 가르지 않는다 — 같은 arm 도 추정기 seed 에 따라 잡는 투척이 바뀐다 (E1-F20: `ur5e_p1b` 의 `closed_form` 이 seed 둘에서 29 · 31 발을 잡았고 겹침은 5 발). 미정인 값으로 판정하면 `PASS(provisional)` 로 표기한다.
- **구현 원칙 (사용자 결정).** 가장 우선은 계획한 알고리즘을 정확하게 구현하는 것이다. spec 을 바꿀 필요는 정확히 구현한 뒤 테스트 · 측정에서 드러날 때 정한다 — 구현 중에 시간이나 성공률을 추정해 spec 을 미리 줄이지 않는다.
- **참고 문서를 구현하는 feature 는 구현 대조표를 낸다 (사용자 결정).** `ref/` 의 설계 자료를 구현하는 feature (끝난 E1-F13 · F14 · F16 · F17 의 표는 #739 · #740 · #742 · #743 에 있다) 는 사용자가 "정확하게 구현됐는가" 를 직접 확인할 수 있게, 참고 문서의 수학 · 구현할 알고리즘 · 구현한 것을 나란히 놓은 표를 그 feature 이슈의 코멘트로 낸다.
  - 행은 참고 문서의 식 · 제약 · 비용 항 하나씩이다. 그 feature 가 맡은 절의 식은 빠짐없이 행으로 둔다 — 구현하지 않는 것도 "구현하지 않음" 과 이유를 적어 남긴다.
  - 착수 때 (Sprint Contract 와 함께): 참고 문서의 수학 (절 · 식) ↔ 구현할 알고리즘 (이산화 · 선형화 · 행의 형태 · 자료 구조 · 호출 순서) ↔ 다르게 하는 곳과 이유. 다르게 하는 곳은 이 표로 승인을 받는다.
  - 완료 때 (PR 을 올리기 전): 같은 행에 구현한 것 (파일 · 함수, 코드가 실제로 계산하는 식) · 착수 때의 계획과 달라진 곳 · 그 행을 확인하는 테스트를 더한다.
  - 대조표는 이슈에 남는다. `ref/` 의 설계 자료는 고쳐 쓰지 않는다 — 구현한 내용을 문서 끝의 새 절에 원래 절과 대응시켜 적고, 그 절에는 최종 구현만 적는다 (사용자 결정, [README.md](README.md) 의 `ref/` 규칙).
- 기존 test assertion 은 약화하지 않는다 (PROC-6). RT 경로 (`Compute`, 샘플러, CLIK) 는 할당 0 을 게이트로 확인한다. 실험 overlay 와 원자료는 repo 밖에 둔다.

## 3. Epic · Feature

**끝난 것.** 아래 표들에 없는 feature 는 끝났다 — 이름과 이슈는 [ID_INDEX.md](ID_INDEX.md) §3 이 갖는다 (E0 · E1 · E2 는 전부). `Decel*` 이름의 rename (MD-48, [#711](https://github.com/hyujun/rtc-framework/issues/711)) 도 끝났다.

### E1. 단일 팔 — NLP search · mpc_docking — [#621](https://github.com/hyujun/rtc-framework/issues/621) · 필수

sim 전용. 게이트: 새 탐색 · planner 를 기존 것과 같은 투척으로 잰 기록이 나고 (E1-F20 — 판정이 아니다: 대조군 · 한계 · PASS 가 없다), 그 기록을 보고 출하 기본값을 바꿀지 정한다. **결과: 기록이 났고 기본값은 바꾸지 않았다** (2026-10-11 — 두 로봇 모두 `grid` × `mpc`, [#813](https://github.com/hyujun/rtc-framework/issues/813)). 기록은 [#746](https://github.com/hyujun/rtc-framework/issues/746) 이 갖고, 남은 일은 §5 의 "E1 에서 넘어온 남은 일" 이 갖는다. feature 는 모두 끝났다 — [ID_INDEX.md](ID_INDEX.md) §3.

### E2. G1 + proto_1b bring-up — [#622](https://github.com/hyujun/rtc-framework/issues/622) · 필수

sim 전용. 게이트: G1 sim 에서 두 컨트롤러가 GUI 로 구동되고 formulation §4 sanity check 가운데 CLIK 의 것 — 항목 4 (단일 frame 극한) · 5 (정지) · 6 (결합 부호) · 8 (일치) — 이 통과한다 (나머지 항목은 MPC 의 것이라 E3 가 받는다, [#622](https://github.com/hyujun/rtc-framework/issues/622)). **결과: 충족** (2026-10-08). feature 는 모두 끝났다 — [ID_INDEX.md](ID_INDEX.md) §3.

| 조건 | 근거 |
|---|---|
| 항목 4 · 8 | E2-F04 — 코어 수준 (golden 재생 · 단일 과제 폐루프의 비트 일치, $v^\ast=\dot q_{ref}$). [#636 완료 코멘트](https://github.com/hyujun/rtc-framework/issues/636#issuecomment-6050425021) |
| 항목 5 · 6 | E2-F05 — gtest 와 G1 sim. [#637 측정 기록](https://github.com/hyujun/rtc-framework/issues/637#issuecomment-6052266629) |
| 두 컨트롤러의 GUI 구동 | E2-F06 — `demo_controller_gui --robot g1_p1b` 의 위젯으로 `demo_joint_controller` 의 관절 목표와 `demo_dualarm_controller` 의 과제 · 자세 · 손 목표, 전환, gain 변경. [#638 결과](https://github.com/hyujun/rtc-framework/issues/638#issuecomment-6054460386) |

게이트가 보지 않은 것: fault latch 의 GUI 표시 (sim 에 fault 를 걸 수단이 없어 단위 테스트로만 봤다), E-STOP (sim 에 trigger 가 없어 gtest 로만 봤다, E2-F05).

### E3. MPC dual arm · waist 확장 — [#623](https://github.com/hyujun/rtc-framework/issues/623) · 필수 (E2 뒤)

같은 MPC 에 dual arm · waist 항을 더한다 (MD-46 · MD-47). 착수 조건: E2 (g1_p1b 준비, 끝) 와 E1-F07 (코어, 끝). sim 전용. 게이트: G1 sim 에서 포구 시행이 돌고 성공률 · solve time p99 가 보고된다. 기존 두 로봇 회귀 없음 — 단일 팔 구성 (더한 항의 가중 0) 의 해가 불변이다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E3-F03 | [#642](https://github.com/hyujun/rtc-framework/issues/642) | 충돌 제약 — capsule 모델 (신규 코어) 과 MPC 제약 행 (자기충돌 · 공–왼팔 거리) | E3-F02 (끝) | 대기 |
| E3-F04 | [#643](https://github.com/hyujun/rtc-framework/issues/643) | MPC ↔ CLIK 계약의 전신 확장 — 왼손 FK, 한계 여유, 17 관절 구간의 RT 평가 (payload 관절 용량은 #827 이 올렸다 — 끝). 구간이 만든 desired value 의 소비자는 `demo_dualarm_controller` 다. 단일 팔 계약은 `ID_INDEX.md` MD-36 · `ref/L7_supervisor.md` §4.3a. E2-F04 가 넣은 것: CLIK 의 자세 속도 feedforward (`qd_posture_ff`) 와 관절군별 자세 과제 — 전신 구간의 $\dot q_{ref}$ 는 그 입력으로 간다 | E1-F08 (끝), E2-F04 (끝), E3-F02 (끝 — 군 · 전신 항의 키는 formulation §14) | 다음 |
| E3-F05 | [#644](https://github.com/hyujun/rtc-framework/issues/644) | G1 통합 — supervisor · 손 시퀀서 · vision sim profile (`mode: mpc` 의 확장) | E3-F03, E3-F04, E2-F05 (끝) | 대기 |
| E3-F06 | [#645](https://github.com/hyujun/rtc-framework/issues/645) | 로그 · plot_rtc_log · demo_controller_gui — dual arm 열. 항별 비용 분해는 코어 변경이라 아직 없다. E2-F06 · F07 에서 넘어온 것: GUI 의 `g1_p1b` profile 과 tree 군의 관절 묶음, 컨트롤러 YAML 에서 만드는 gain 행, CSV 꼬리로 읽는 상태 줄 (`demo_dualarm_controller` 에는 상태 토픽이 없다 — G1 포구 컨트롤러의 상태를 무엇으로 보일지는 이 feature 가 정한다. `rtc_msgs` 를 고치면 E-3), 열 이름에서 과제 · 관절을 읽는 plot 의 helper 는 [#645 의 인계 코멘트](https://github.com/hyujun/rtc-framework/issues/645#issuecomment-6054618892) | E3-F05 | 대기 |
| E3-F07 | [#646](https://github.com/hyujun/rtc-framework/issues/646) | 평가 — G1 sim 포구 시행 · 예측 격자 sweep · waist · 왼팔 고정 대조 · 기존 로봇 회귀 | E3-F06 | 대기 |

### 실기 (HW) — [#613](https://github.com/hyujun/rtc-framework/issues/613) · 필수 · **sim 전용이 아니다**

1차 목표 로봇은 `ur5e_p1b`. 진행 순서는 bag replay (재스탬프) → 가상 공 → 저속 실투척 → 상향이다. feature 이슈는 착수할 때 만든다 (이슈 번호 칸 "—"). 체크리스트 (도구 · 사용자 결정 · 실기에서 재야 닫히는 것 · park 키) 는 #613 이 갖는다 — 이 문서는 그것을 복제하지 않는다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| HW-F01 | — | 단계 D — 실기 기본 측정. 게이트 G8-A · G5-F · G6-D · G6-E · D-S9-G · 운동 기한의 실기 값. 절차의 내용은 `ref/`: 지연 식별 `ref/L5_joint_cmd.md` §4.4 · §9, 손 식별 · `T_close,tot` `ref/L6_hand.md` §4 · §9, D-S9-G `ref/L7_supervisor.md` §4.1 · §10, G8-A `ref/L8_bringup.md` §9 | #613 의 도구 · 사용자 결정 중 이 단계가 쓰는 것 | 대기 |
| HW-F02 | — | 단계 E — bag replay · 가상 공. 게이트 G8-F (abort 경로 전부) · G7-F. 절차는 `ref/L8_bringup.md` §9.2, `ref/L7_supervisor.md` §9 | HW-F01, **실기 planner 결정** (아래), #613 의 도구 (bag 재스탬프 replay · 가상 공 발행기 · speed scaling · PTP 감시) | 대기 |
| HW-F03 | — | 단계 F — 실투척 (저속 → 상향). 게이트 G8-G ("원인 분해" 는 항목별 수치 기록으로 판정) | HW-F02 | 대기 |
| HW-F04 | — | GUI · plot 의 실기 항목 (실기 모드 표시, 실기 세션 CSV 의 plot). #613 이 순서를 정하지 않았다 | — | 대기 |

**실기 planner (선행 조건).** 사용자 결정: 지금은 planner 를 정하지 않는다. 실기 기본 측정 (단계 D) 은 planner 와 무관하게 진행하고, **단계 E 앞에서** `closed_form` 과 `mpc` 중 어느 쪽으로 갈지 정한다. 지금 실기 config 는 planner 를 덮지 않아 출하값 `planner.segment.mode: mpc` 가 그대로 실기에 간다. `mpc` 로 가려면 먼저 닫아야 하는 것 셋 (정지 구간과 작업셀 경계 — E-8 에 걸린다, G8-F 에 mpc 의 abort 경로, `mpc` 를 실기에서 잰 적이 없다는 것) 은 #613 의 "planner" 절이 갖는다. 첫째의 재료로, `mpc` 의 정지 구간이 어디인지는 sim 에서 쟀다 (#713 — 손은 공의 직선 위에서 멈춘다; 수는 #613 의 코멘트). 경계의 검사는 여전히 없다. 결정의 재료로, E1-F20 에서 속도 한계 위반과 `ABORT_SAFE` 는 `closed_form` 에 몰려 있었다 (`ur5e_p1b` 47 · 69 발, `iiwa7_leap` 209 · 34 발; `mpc` 는 0 – 2 발 — [#746](https://github.com/hyujun/rtc-framework/issues/746)). ProxQP 의 heap 할당은 선행이 아니다 — 기록된 RT-1 위반으로 정해졌다 (사용자 결정 2026-10-10, #654).

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
| `feat/catching-mpc-clik-contract` | E3-F04 | MPC ↔ CLIK 계약의 전신 확장 — `integrated_bringup` 의 RT · 왼손 FK · 한계 여유. 계약의 sanity check 는 E3-F02 의 해를 입력으로 쓴다 |
| `feat/collision-capsule-core` | E3-F03 | 신규 수치 코어. code review 단위 |
| `feat/g1-catch-controller` | E3-F05 | supervisor · 손 시퀀서 통합. E-STOP 경로를 건드리면 E-8 |
| `feat/g1-catch-tooling-eval` | E3-F06, E3-F07 | 로그 · plot · GUI 와 평가. 평가가 새 로그 컬럼을 쓴다. **나누는 조건**: 평가 결과가 설계 결정을 바꾸면 결과 기록을 `docs/` 브랜치로 분리한다 |

**순서 — E1.** 남은 feature 브랜치는 없다 (§1). feature 번호 없는 남은 단독 이슈는 보류 중인 #715 하나다 (§5 의 "mpc planner · catching 의 남은 일").

**순서 — E3.** E3-F04 (`feat/catching-mpc-clik-contract`) → E3-F03 → E3-F05 → E3-F06 · F07 (착수 조건인 E2 · E1-F07 · #827 과 E3-F01 · F02 는 끝났다 — MD-47, `ID_INDEX.md` §3). E3-F02 앞에 두었던 #819 는 끝났다 (§5). 병행은 서로 다른 파일을 고치는 브랜치끼리만 한다. 그 앞에 두었던 `mpc` 의 법칙을 고치는 단독 이슈 (#834 · #716) 도 끝났다 (§5).

### 게이트 · escalation

| Feature | 사유 | 효력 |
|---|---|---|
| docking 기능의 실기 선택 (feature 를 열 때) | 실기 configuration 이 두 docking 기능 (`nlp` · `mpc_docking`) 가운데 하나를 고르면 `robot.hand.docking.provisional` 이 참인 동안 configure 가 park 한다 (E1-F16) — 손의 포획 집합이 sim 식별값이다. solver 의 heap 할당은 이 조건이 아니다: 기록된 RT-1 위반이다 (`agent_docs/invariants.md`, 사용자 결정 2026-10-10 · #654) | Critical — 그 flag 를 내리거나 그 park 를 풀려면 착수 전 `[CONCERN]` 과 컨펌. flag 를 내릴 때 GUI 의 `sim only` 표시와 두 조각 머리의 SIM ONLY 를 같이 고친다 |
| E3-F05 | E-STOP 경로를 건드리면 E-8 | Critical |
| 실기 (HW) | `mpc` 의 정지 구간과 작업셀 경계 (RT 도 탐색도 포구점 · 정지점의 위치를 판정하지 않는다 — MD-73 · L3 §4.9), `mpc` 구간의 샘플 시각을 바꾸는 RT 법칙 변경 — 둘 다 E-8 (#613) | Critical — 착수 전 `[CONCERN]` 과 컨펌 |
| E3-F03 | 신규 수치 코어 (100+ 줄) | code review |

## 4. 남은 feature 를 구속하는 결정

아직 구현하지 않은 feature 를 구속하는 `MD-n` 이다. 나머지 `MD-n` 은 [ID_INDEX.md](ID_INDEX.md) §4. 끝난 E1-F19 · F20 을 구속한 결정 (solver · 조합 · 선택 키 · 식의 형태 · 1 차 범위 · 판정 방식) 은 `MD-n` 이 아니다 — 각 feature 이슈의 범위에 적혀 있고 선택지와 이유는 [#621 의 결정 코멘트](https://github.com/hyujun/rtc-framework/issues/621#issuecomment-5974865100) 에 있다. 그 가운데 **D5-2 ("바깥 루프는 RT 가 plan 을 채택할 때까지만") 는 뒤집혔다**: 탐색은 채택 뒤에도 돈다 — 처음 채택한 plan 의 $t_c$ 에서 `t_stop_plan` 앞까지, 모든 조합 (`grid` × `mpc` 포함) 에서. 채택 뒤의 탐색은 E1-F16 이, 교체의 게시와 RT 의 교체 쌍 채택은 E1-F17 이 구현했다 (L3 §5.3 · L7 §4.3a). 무엇을 어느 feature 가 맡는지는 [#740 의 확정 코멘트](https://github.com/hyujun/rtc-framework/issues/740#issuecomment-6009380305) (N10 – N12) 와 #742 · #743 의 본문에 있다.

| ID | 무엇을 정했나 | feature |
|---|---|---|
| MD-14 | CLIK 의 충돌 damper 는 선택 항이고 기본 꺼짐이다 (기존 로봇의 golden 회귀를 지키기 위해). E3-F03 의 거리 코어 뒤에 넣는다. 같은 결정의 제동 거리 한계와 오차 되먹임 상한은 E2-F04 가 넣었다 (`ID_INDEX.md` MD-14) | E3-F03 뒤 |
| MD-86 | `sim_g1_p1b` 의 `enable_mpc` 는 CPU layout 만 고른다. G1 의 포구 컨트롤러가 오면 그 기본값을 그 feature 에서 다시 정한다 (E2-F05 는 바꾸지 않았다) | E3-F05 |
| MD-12 | 각운동량 항은 유지한다 — 목적은 왼팔의 운동 생성과 floating base 확장이고, 효과는 포구 성공률이 아니라 각운동량 변화율의 크기로 판정한다 (항은 E3-F02 가 넣었다) | E3-F07 |
| MD-13 | 토크는 1차까지 선형화하고 토크 · 자기충돌 행에는 slack 을 둔다 (단일 팔의 토크 slack 은 노드당 하나 — #857). 위치 · 속도 한계, 종단 등식, trust region, 공–왼팔 행은 hard 다. 동역학은 손을 기준 자세로 잠근 축소 모델로 계산한다 (토크 쪽은 E3-F02 까지 그대로 — 17 관절 모델) | E3-F03 |
| MD-46 | 설계는 formulation §1.3 하나다. g1_p1b 는 같은 코어에 dual arm · waist 전용 항 (waist 억제, 왼팔 rest, 각운동량, 자기충돌, 공–왼팔, waist 토크 행 · counter-swing, 관절군별 move blocking) 을 더한다. **편차의 뜻이 바뀌었다 (D7, 2026-10-13)**: 포구 후보 ($t_c$) 는 G1 구성에서도 탐색이 고른다 — 탐색 모델은 waist + 오른팔 사슬이고 전신 항은 구간에만 들어가 후보의 순위에 들어가지 않는다. formulation 의 "후보마다 §1.3 의 QP 를 푸는 바깥 루프" 는 구현하지 않는 설계로 남긴다 (D1′) | E1-F14 (탐색 코어 — 끝) · F16 (계획기가 부른다 — 끝) · E3-F01 (모델 둘 — 끝) · F02 (전신 항 — 끝) |
| MD-47 | E3 의 착수 조건은 E2 (g1_p1b 준비) 와 E1-F07 (코어) 이다. G-1 은 조건이 아니다 | E3 |

## 5. 아직 정하지 않은 것

열린 결정은 이슈가 갖는다. 제목은 이슈 제목이다.

**남은 feature 의 결정**

- [#646](https://github.com/hyujun/rtc-framework/issues/646) — E3-F07: 안 B (`nlp` 에 전신 항) 로 옮길지의 **문턱** (D17 — E3-F02 가 [완료 코멘트](https://github.com/hyujun/rtc-framework/issues/641#issuecomment-6231319410) 에서 잰 시간과 함께 넘긴 결정, [#646 의 결정 요청](https://github.com/hyujun/rtc-framework/issues/646#issuecomment-6231320420)). 사슬에만 의존하는 항을 탐색에도 켜는 것은 하지 않는다 (탐색은 운동학만 — #640 D8 과 같은 뜻)
- [#642](https://github.com/hyujun/rtc-framework/issues/642) — E3-F03: 충돌 제약 — capsule 모델 · 자기충돌 · 공–왼팔 거리
- [#645](https://github.com/hyujun/rtc-framework/issues/645) — E3-F06: 로그 · plot_rtc_log · demo_controller_gui (dual arm 열) — G1 포구 컨트롤러의 상태를 GUI 에 무엇으로 보일지 (E2-F06 은 상태 토픽 없이 세션 CSV 의 꼬리를 읽었다 — GUI 가 컨트롤러와 같은 머신에 있어야 한다. `rtc_msgs/CatchingState` 에 열을 더하면 E-3)

**E1 에서 넘어온 남은 일**

E1 의 끝난 일 (E1-F19 · F20 · F21, T [#803](https://github.com/hyujun/rtc-framework/issues/803), [#807](https://github.com/hyujun/rtc-framework/issues/807)) 이 남긴 것만 적는다. 기록과 수는 각 이슈가 갖는다 ([ID_INDEX.md](ID_INDEX.md) §3 의 feature 행). 이슈는 열지 않았다.

- **포구 순간의 lateral 오차를 줄이는 자리 (#807).** 공이 손의 입구 평면을 `ρ_ref` 에서 10 mm 안으로 지나면 잡고 15 mm 밖이면 못 잡는다 (`ur5e_p1b`; `iiwa7_leap` 에는 그 기준이 옮겨지지 않는다). lateral 의 몫은 추정기의 예측과 추종이 반반이고 계획은 아니다. 줄일 자리는 셋이다.
  - 마지막 풀이를 t_c 에 가깝게 한다 (재계획의 격자 · 예산 · 채택). 예측의 오차는 그 풀이가 앞을 본 거리를 따라간다 — 350 ms 안이면 5 mm, 450 ms 밖이면 33 mm 다.
  - 추정기의 0.3 – 0.5 s 예측과 자료 나이 (ball_perception).
  - 서보 지연 보상 (lateral 11 mm, 접근축 +5 – 11 mm).

  측정 자세 · 재시각화 (R #805) 를 다시 열 근거는 없다 — [단계 3](https://github.com/hyujun/rtc-framework/issues/807#issuecomment-6098255382).
- **`mpc_docking` · `nlp` 의 끊긴 풀이 몫 (E1-F19 의 "≤ 5 %" 미충족).** T 가 100 ms 넘는 꼬리를 없앤 뒤에도 몫은 31 – 78 % 다. 끊긴 풀이는 예산을 1 – 3 ms 넘겨 끝난다. 남은 수단은 반복 수와 예산이다. 시험하지 않은 것: `core.solver.max_iter_warm` 의 10 밖 값 (20 은 unit 하나로만 봤다), ProxQP warm start 에서 반복값만 잇기 — [#803 F-1](https://github.com/hyujun/rtc-framework/issues/803#issuecomment-6097153277).
- **값으로 풀리지 않은 것 (E1-F19 1 부).** 아래 넷은 그대로다 — [#745 의 머지 코멘트](https://github.com/hyujun/rtc-framework/issues/745#issuecomment-6062693925).
  - 포구 시각에 손이 접근축으로 앞선다. 다만 #807 이 보니 그 축은 포구를 가르지 않는다.
  - 공분산 유효성 검사가 확률 행과 함께 꺼진다 (두 로봇 모두 확률 행이 꺼져 있다).
  - 마지막 구간의 교체 풀이.
  - `iiwa7_leap` 의 `speed` 보류.
- **sim 에서 아직 확인하지 않은 것.**
  - `grid` 의 교체 규칙이 plan 을 유지하는 동작 ([#793 의 merge 코멘트](https://github.com/hyujun/rtc-framework/issues/793#issuecomment-6077965184)).
  - `T_close_lead` 를 옮긴 효과 (`ur5e_p1b` — [L6 §4.6](ref/L6_hand.md)).
  - `iiwa7_leap` 의 `publish.slack_c_max` 0.06 m 가 통로 관문을 사실상 여는 것 (통로 입구 반지름의 2.7 배). 게시가 열리는 S (#804) 에서 다시 본다.
  - `core.sqp.mu_max` 1e2 가 `grid` × `mpc_docking` 에서 풀리는 문제를 거부하는지. 같은 seed 의 unit 한 쌍씩으로만 봤다.
  - 첫 풀이의 wake 사이 기억에서 출발한 풀이가 실행 불가로 끝난 점에 머무는지. `grid` 에서는 그 기억이 거의 쓰이지 않는다 (572 건 가운데 12 건, [#803 T](https://github.com/hyujun/rtc-framework/issues/803#issuecomment-6095998706)).
  - `iiwa7_leap` 의 `nlp` × `mpc_docking` 을 2 / 25 로 적은 앞선 수와 E1-F20 의 196 / 360 이 왜 다른지.

**mpc planner · catching 의 남은 일**

- [#715](https://github.com/hyujun/rtc-framework/issues/715) — integrated_bringup: 세 군 (두 번째 손) 지원 — 보류
- [#887](https://github.com/hyujun/rtc-framework/issues/887) — rtc_controllers: `mpc` 코어의 진짜 infeasible 풀이 시간 (warm + cold 재시도) 을 예산 안으로 (#880 에서 넘어온 것)

**`mpc planner · catching 의 남은 일` 의 순서 (권장).** 이 문서에서 남은 이슈의 시기는 이 표 하나가 갖는다 — 다른 절은 여기를 가리키고 시기를 다시 적지 않는다. 묶지 않는다 — 이슈 하나가 브랜치 하나이다. 이슈에 정해진 시기 안에서 고른 순서다.

| 순서 | 이슈 | 착수 조건 | 브랜치 | 착수 때 정할 것 |
|---|---|---|---|---|
| 1 | #887 | 충족 — 조건 없음 | 착수 때 확정 | 수렴하는 warm 풀이 (same · advance) 의 반복 분포 — warm 상한 · 재시도 판단의 근거를 던지기 전에 고정 |
| — | #715 | 보류 — 착수 조건이 없다 | — | — |

- **묶지 않는 이유.** `mpc` 의 법칙을 한 PR 에서 두 곳 바꾸면 측정이 원인을 가르지 못한다. PR 을 쌓지 않으므로 같은 파일을 고친다는 것은 묶을 이유가 아니다.
- **끝난 단독 이슈** — 측정과 결정의 기록은 각 이슈가 갖는다. 남긴 일만 적는다.
  - [#880](https://github.com/hyujun/rtc-framework/issues/880) (2026-10-19) — `mpc` 코어의 ProxQP 근사 infeasibility 판정을 껐다 (`solver.eps_primal_inf` 0, 새 키) — 오프라인 첫 풀이에서 그것이 실패시킨 5,452 / 약 26만 이 모두 실행 가능한 QP 였다. 그 대신 정말 실행 불가능한 QP 를 묶는 `solver.max_iter` 를 200 → 60, `w_perp` · `w_path` 의 파서 상한을 1e4 → 3e3 으로 내렸고, warm-up 을 box 가운데로 되돌렸다 (#857 의 우회 제거). 오프라인 게시는 그대로이고, sim (두 로봇 × 세 chunk) 의 `qp_failed` 는 29 → 0 이며 #716 · #857 의 가드를 지났다 (`ur5e_p1b` 의 토크 한계 1 투척은 같은 코드의 drift unit 에서도 나는 변동으로 판정 — D13). 남긴 일: 정말 실행 불가능한 풀이의 시간 (→ #887); stale warm start 의 cold 재시도는 남는다 (수렴하지 못하는 warm 풀이).
  - [#857](https://github.com/hyujun/rtc-framework/issues/857) (2026-10-18) — `mpc` 의 토크 slack 을 노드당 하나로 바꿨다 (노드 안에서 가장 크게 넘는 관절에 벌점 — formulation §1.6; 벌점이 정확한 조건은 노드의 토크 행 승수의 합). 오프라인 첫 풀이의 게시는 두 로봇 모두 그대로이고, sim (두 로봇 × 세 chunk) 은 #716 의 가드와 `slack` 보류 가드를 지났다 — 첫 풀이 p99 는 약 30 ms 에서 약 10 ms 로 줄었다. 남긴 일: ProxQP 의 근사 infeasibility 판정이 퇴화한 QP 를 infeasible 로 보는 것 — planner 의 warm-up 자세를 box 가운데에서 비켜 우회했다 (→ #880); `cost.rho_tau` 10 은 노드당 법칙에서 다시 sweep 하지 않았다.
  - [#716](https://github.com/hyujun/rtc-framework/issues/716) (2026-10-16) — `mpc` 의 풀이 시간 예산을 `budget.first_s` 0.040 · `replan_s` 0.030 s 로 올렸다 (두 로봇, #834 에서 넘어온 것 — [측정 ①](https://github.com/hyujun/rtc-framework/issues/716#issuecomment-6188682991)). 폐쇄 창의 경로 이탈 항 `cost.w_path` (formulation §1.6 — 포구 구간 $\mathcal K_c$ 대신 손이 닫히는 창의 포구 전 노드) 를 구현하고 0 으로 출하한다 — `ur5e_p1b` sim 에서 창의 이탈은 줄었지만 포구로 이어지지 않았다 ([측정 ②](https://github.com/hyujun/rtc-framework/issues/716#issuecomment-6200092609), 사용자 결정 D10 a). 남긴 일: 노드당 토크 slack (→ #857, 착수 조건은 E3-F02 의 변형 측정 — [#641 D6](https://github.com/hyujun/rtc-framework/issues/641#issuecomment-6216709591)), `n_pre_max` 를 다시 올리는 판정 (측정 ① 의 기록이 근거가 될 수 있다), 실기 PC 에서의 예산 재측정 (#613 의 단계). 포구 구간을 여러 노드로 두는 것 ($\mathcal K_c$) 은 하지 않았다 — formulation §1.6 은 단일 팔의 포구 구간이 한 노드인 것을 손 폐쇄 명령 시각의 귀결로 적고, 여러 노드로 두려면 세 planner 가 함께 쓰는 그 규칙이 바뀐다 (이슈 없음)
  - [#834](https://github.com/hyujun/rtc-framework/issues/834) (2026-10-15, PR [#839](https://github.com/hyujun/rtc-framework/pull/839)) — 포구 전 격자의 상한 `approach.n_pre_max` 를 `ur5e_p1b` 7 로 올렸다 (`iiwa7_leap` 6, `dt_pre_s` 0.1 그대로). 남긴 일: 풀이 시간 예산 (#716 이 올렸다, 위), 아래 #819 의 `delta_tr` 재측정 · `n_pre` 하한은 이 격자를 기준선으로 잰다. 탐색의 도달 게이트를 격자 시간에 견주는 안은 판단하지 않았다 — [닫는 코멘트](https://github.com/hyujun/rtc-framework/issues/834#issuecomment-6182203068)
  - [#819](https://github.com/hyujun/rtc-framework/issues/819) (2026-10-14, PR [#835](https://github.com/hyujun/rtc-framework/pull/835)) — 첫 풀이의 기준을 자르는 문턱 `linearization.ref_speed_fraction` 를 두 로봇 모두 1.2 로 출하한다 (식은 formulation §13; 실제 상한은 `delta_tr` — 기준이 box 를 넘는 면적 ≤ 2·`delta_tr`). 남긴 일: 남은 `catch_error` 보류의 다음 수단 (첫 기준의 끝을 $(q^\ast,\dot q^\ast)$ 로 · 초기화 QP), `delta_tr` 재측정 · `n_pre` 하한, 도달을 $E\le2\delta$ 로 닫힌 형태로 묶는 것 (법칙 변경) — [출하와 검증](https://github.com/hyujun/rtc-framework/issues/819#issuecomment-6165060436)
  - [#822](https://github.com/hyujun/rtc-framework/issues/822) (2026-10-14, PR [#837](https://github.com/hyujun/rtc-framework/pull/837)) — 값 sweep 도구 (`mk_overlay.py --set`, `run_unit.sh` 의 `EXPECT_KV` · 실행 이미지 sha256, `summarize.py --vary`). 남긴 일: sweep 의 재시도 루프 · 분석 호출을 도구로 올릴지 — #834 는 repo 밖 드라이버 (`~/rtc_eval/i834/tools/`) 로 돌렸다 — [계획 확정 2](https://github.com/hyujun/rtc-framework/issues/822#issuecomment-6166798318)
  - [#713](https://github.com/hyujun/rtc-framework/issues/713) (2026-10-12) — 정지 경로 항 `cost.w_perp` 100 을 두 로봇에 출하한다 (정지 끝 이탈 중앙값 2 – 2.7 mm, #819 뒤에도 그대로). 남긴 일: `delta_tr` 재측정 뒤에 같은 측정 (`segment_stop_*` · `stop_*`) 으로 한 번 더 본다 — [2 차](https://github.com/hyujun/rtc-framework/issues/713#issuecomment-6123763984)
  - [#710](https://github.com/hyujun/rtc-framework/issues/710) (2026-10-11) — `iiwa7_leap` 의 `grid` IK 값 (`k_manip` 0, `manipulability_min` 0.1, L3 부록 D). 재지 않은 채로 둔 것: 출하 값의 sim unit, `grid` × `closed_form` 에서의 결과, leap 의 throw set 라벨 — [0-C](https://github.com/hyujun/rtc-framework/issues/710#issuecomment-6105692143)

**이 문서가 정하지 않은 것.** 실기 epic 과 E3 의 선후 (실기는 `ur5e_p1b` 이고 E3 는 G1 sim 이라 서로의 선행이 아니다). 실기 planner 는 단계 E 앞에서 정한다 (§3).
