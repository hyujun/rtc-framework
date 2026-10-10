# MPC · dual-arm catching 과 실기 단계 — 계획

- 상태: **E0 완료 · E1 진행 중 (남은 feature 는 §3 의 표, 그 앞의 일과 순서는 §1) · E2 완료 (E2-F01 – F07 — 게이트 충족, §3)** · E3 · 실기 대기. 이 줄은 epic 의 상태만 적는다 — feature 의 상태는 §3 의 표가, 끝난 feature 의 이름은 [ID_INDEX.md](ID_INDEX.md) §3 이 갖는다 (§2 "상태는 한 곳에만")
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
| **E1** | **필수** | 단일 팔 MPC (ur5e_p1b · iiwa7_leap 의 팔 기준을 APPROACH 부터 정지까지 MPC 로) 는 끝났다. 남은 것은 단일 arm-hand 의 두 모듈 — 포구 후보를 NLP 로 고르는 탐색 (`nlp`) 과 inner-loop NMPC planner (`mpc_docking`) — 과 기존 탐색 · planner 와의 비교다 |
| E2 | feature 가 모두 끝남 | g1_p1b 의 `demo_joint_controller` · `demo_dualarm_controller` 와 그 GUI · plot |
| **E3** | **필수** (선행 E2 는 끝났다) | 같은 MPC 에 dual arm · waist 항 추가 — g1_p1b |
| 실기 (HW) | 필수 · sim 전용이 아니다 | 실기 단계 — 1차 목표 로봇은 `ur5e_p1b` ([#613](https://github.com/hyujun/rtc-framework/issues/613)) |

- **E1 의 남은 일과 순서 — 순서는 이 항목에만 적는다.** 남은 feature 는 비교 평가 E1-F20 하나이고, 그 앞에 feature 번호 없는 단독 이슈 넷이 있다 (이슈 하나가 브랜치 하나): **T** ([#803](https://github.com/hyujun/rtc-framework/issues/803) — 풀이의 기한 검사를 QP · 재시도 · probe 루프 안으로; 조건부였고 E1-F19 가 그 조건을 채웠다), **S** ([#804](https://github.com/hyujun/rtc-framework/issues/804) — 사용자 결정 D2: 조건에 맞는 폐쇄 속도가 없으면 후보를 거부하지 않고 낼 수 있는 최대 속도로 계획한다, velocity-set 의 잔차는 실행 가능 판정에서 뺀다), **R** ([#805](https://github.com/hyujun/rtc-framework/issues/805) — 폐쇄 재시각화를 측정 자세로; lead 에서 유도되는 값과 그 검증기도 여기서 본다), **포구 순간의 손–공 오차의 조사** ([#807](https://github.com/hyujun/rtc-framework/issues/807) — 코드 · 값 무변경). **정해진 순서는 S → R → E1-F20 이고, T 와 #807 의 자리는 정하지 않았다** — 권고는 T → S → R ([#803 의 코멘트](https://github.com/hyujun/rtc-framework/issues/803#issuecomment-6094239428)), #807 은 R 의 "착수 전 확인" 과 겹쳐 R 앞이 자연스럽다. E2 는 E1 의 선행이 아니었고 끝났다. E3 의 착수 조건인 E2 (g1_p1b 준비) 와 E1-F07 (코어) 은 둘 다 끝났다 — E3 의 브랜치가 E1 의 남은 브랜치와 병행할 수 있는지는 고치는 패키지로 정한다 (§3 의 병행 규칙).
- **설계 (MD-46 · MD-47).** MPC 는 waist + dual arm (G1) 용으로 설계한다 (formulation §1.3). 단일 팔은 같은 MPC 에서 dual arm · waist 전용 항과 제약만 뺀 구성이었고, g1_p1b 는 같은 코어에 그 항을 더한다 (E3). closed_form 과 mpc 는 입력 (추정기의 공 미래 궤적) 과 출력 (CLIK 입력) 이 같은 두 planner 이고 추정기 · supervisor · 손 시퀀서 · CLIK · `ABORT_SAFE` · E-STOP 은 공통이다.
- **E1 의 남은 feature.** 탐색 하나 (`nlp`) 와 planner 하나 (`mpc_docking`) 는 코어 · 계획기 배선 (E1-F16) 과 RT 쪽 (E1-F17) 까지 들어왔다 — 로그 (E1-F18) · 판정 지도 (E1-F21) · 튜닝 (E1-F19) 도 끝났고 남은 것은 평가다. 설계 자료는 [ref/ball_catching_inverse_dynamics_mpc.md](ref/ball_catching_inverse_dynamics_mpc.md) 다 (§10 이 planner, §11 이 탐색 — 구현한 것은 그 문서의 §17 에 적는다). 수치 코어는 하나이고 이미 있다 (`MpcDockingSegmentCore`, E1-F13): `mpc_docking` 은 그 코어로 구간을 풀고 `nlp` 는 같은 코어로 후보를 평가한다. 탐색 코어도 있고 (`NlpCatchSearch`, E1-F14) 계획기의 한 주기가 부른다 (E1-F16). 둘 다 계획기 스레드 안에서 돌고, 위의 공통부와 계획기 → RT 계약의 payload, 출하 기본값 (선택 키의 값) 은 바꾸지 않는다. **탐색은 RT 가 plan 을 채택한 뒤에도 돈다** — 처음 채택한 plan 의 $t_c$ 에서 `t_stop_plan` 앞까지, 공을 잡기 직전까지만이고 모든 조합에서다 (사용자 결정, §4). 그래서 출하 조합 `grid` × `mpc` 의 동작도 바뀐다: 채택 뒤의 wake 는 탐색을 먼저 돌리고, 탐색이 다른 plan 을 고르면 그 plan 과 첫 구간을 쌍으로 게시해 RT 가 APPROACH 중에 둘을 함께 바꾸며 (아니면 구간을 재계획한다), 손의 폐쇄 지령 시각은 commit 뒤에도 지령 전까지 다시 맞춰진다 (E1-F16 · F17 — L3 §5.3, L7 §4.3a, L6 §4.3). 코드는 로봇을 모르게 쓰고 시험은 `ur5e_p1b` · `iiwa7_leap` 둘에서 한다. 새 구현의 이름은 값 + interface 의 순서다 — 구간 계획기는 `<값>SegmentPlanner` · `<값>SegmentCore` (`MpcSegmentPlanner` · `MpcSegmentCore` 와 같다), 탐색은 `<값>CatchSearch` (`GridCatchSearch` 와 같다). `Decel*` 이름을 새로 쓰지 않는다 (MD-48).
- 실기와 E2 · E3 의 선후는 이 문서가 정하지 않았다 (§5).

### 게이트 G-1 — 단일 팔 mpc planner 가 closed_form planner 와 비슷한 성능을 내는가

두 planner 만 다른 paired A/B 시험으로 "closed_form 대비 비열등" 을 판정하는 게이트다 (정의: formulation §6.5). **결과: 두 로봇 FAIL** ([#632](https://github.com/hyujun/rtc-framework/issues/632) — 한계 · 시행 수 · seed · 수치는 거기에 있다). 남은 순서에 주는 뜻:

- G-1 은 E3 의 착수 조건이 아니다 (MD-47). E3 의 항은 단일 팔 결과와 무관하게 G1 에서 필요하다.
- 사용자는 FAIL 을 알고 두 로봇의 출하 DECEL 법칙을 `mpc` 로 정했다 (MD-89). 따라서 `mpc` 가 출하 기본값이지만 closed_form 대비 비열등은 확인되지 않았고, lead 를 끈 출하 구성으로 잰 적도 없다. G-1 은 탐색이 채택에서 멈추는 `mpc` 와 옛 폐쇄 지령 시각으로 쟀다 — E1-F16 · F17 뒤의 `grid` × `mpc` 는 채택 뒤에도 탐색해 APPROACH 중에 plan 을 교체할 수 있고, 폐쇄 지령 시각은 `T_close_lead` 로 바뀐 데다 (`iiwa7_leap`: $t_c-0.1037\to t_c-0.0577$) commit 뒤에 공의 통과 시각으로 다시 맞춰진다. 그 `mpc` 로 closed_form 과 견준 적은 없다.
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
- **상태는 한 곳에만 적는다.** feature 의 상태 (다음 · 대기, PR, 결과 한 줄) 는 §3 feature 표의 상태 열이, epic 의 상태는 문서 머리의 상태줄이 (feature 를 세지 않는다), **남은 일의 순서는 §1 의 한 항목이** 갖는다 — 다른 절은 그 자리를 가리키고 다시 적지 않는다. 브랜치 계획과 순서 표, epic 이슈의 본문, project 의 readme, 그리고 **`ref/` 문서의 머리와 [README.md](README.md) 의 문서 표** 에는 상태를 적지 않는다 (그 둘은 어느 feature 가 무엇을 구현했는가만 적는다 — 남은 feature 와 그 상태를 적으면 feature 가 끝날 때 어긋난다. 문서 자신의 구현 범위, 곧 어느 절이 구현됐는가는 그 문서의 내용이다) — epic 의 feature 목록과 열림 · 닫힘은 sub-issue 가 보여 주고, 보드의 Done 은 이슈가 닫힐 때 자동으로 옮겨진다. feature 를 끝낼 때 손으로 고치는 곳은 넷이다: §3 의 그 행을 지우고 (끝난 feature 는 표에 없다) [ID_INDEX.md](ID_INDEX.md) §3 에 행을 더한다, 남긴 일이 있으면 §5, 그 feature 이슈의 Done when, epic 이슈의 머지 코멘트. 순서가 바뀌면 §1 의 그 항목 하나를 고친다.
- feature 착수 전에 Sprint Contract 를 제시하고 컨펌받는다 (AGENTS.md §6.5). 브랜치는 비슷한 feature 를 묶어 만든다 (§3 "브랜치 계획").
- 수치로 판정하는 게이트는 기준과 N 을 시행 전에 고정하고, 기준을 본 뒤에 바꾸지 않는다. 미정인 값으로 판정하면 `PASS(provisional)` 로 표기한다.
- **구현 원칙 (사용자 결정).** 가장 우선은 계획한 알고리즘을 정확하게 구현하는 것이다. spec 을 바꿀 필요는 정확히 구현한 뒤 테스트 · 측정에서 드러날 때 정한다 — 구현 중에 시간이나 성공률을 추정해 spec 을 미리 줄이지 않는다.
- **참고 문서를 구현하는 feature 는 구현 대조표를 낸다 (사용자 결정).** `ref/` 의 설계 자료를 구현하는 feature (끝난 E1-F13 · F14 · F16 · F17 의 표는 #739 · #740 · #742 · #743 에 있다) 는 사용자가 "정확하게 구현됐는가" 를 직접 확인할 수 있게, 참고 문서의 수학 · 구현할 알고리즘 · 구현한 것을 나란히 놓은 표를 그 feature 이슈의 코멘트로 낸다.
  - 행은 참고 문서의 식 · 제약 · 비용 항 하나씩이다. 그 feature 가 맡은 절의 식은 빠짐없이 행으로 둔다 — 구현하지 않는 것도 "구현하지 않음" 과 이유를 적어 남긴다.
  - 착수 때 (Sprint Contract 와 함께): 참고 문서의 수학 (절 · 식) ↔ 구현할 알고리즘 (이산화 · 선형화 · 행의 형태 · 자료 구조 · 호출 순서) ↔ 다르게 하는 곳과 이유. 다르게 하는 곳은 이 표로 승인을 받는다.
  - 완료 때 (PR 을 올리기 전): 같은 행에 구현한 것 (파일 · 함수, 코드가 실제로 계산하는 식) · 착수 때의 계획과 달라진 곳 · 그 행을 확인하는 테스트를 더한다.
  - 대조표는 이슈에 남는다. `ref/` 의 설계 자료는 고쳐 쓰지 않는다 — 구현한 내용을 문서 끝의 새 절에 원래 절과 대응시켜 적고, 그 절에는 최종 구현만 적는다 (사용자 결정, [README.md](README.md) 의 `ref/` 규칙).
- 기존 test assertion 은 약화하지 않는다 (PROC-6). RT 경로 (`Compute`, 샘플러, CLIK) 는 할당 0 을 게이트로 확인한다. 실험 overlay 와 원자료는 repo 밖에 둔다.

## 3. Epic · Feature

**끝난 것.** 아래 표들에 없는 feature 는 끝났다 — 이름과 이슈는 [ID_INDEX.md](ID_INDEX.md) §3 이 갖는다 (E0 · E2 는 전부, E1 은 표의 것만 남았다). `Decel*` 이름의 rename (MD-48, [#711](https://github.com/hyujun/rtc-framework/issues/711)) 도 끝났다.

### E1. 단일 팔 — NLP search · mpc_docking — [#621](https://github.com/hyujun/rtc-framework/issues/621) · 필수

sim 전용. 게이트: 새 탐색 · planner 를 기존 것과 같은 투척으로 비교한 판정이 나고 (E1-F20), 그 결과로 출하 기본값을 바꿀지 정한다 — PASS 가 조건이 아니다. feature 의 범위와 Done when 은 각 이슈가 갖는다. 참고 문서를 구현하는 feature 는 구현 대조표를 낸다 (§2). 표의 순서가 진행 순서다.

| Feature | 이슈 | 내용 | 선행 | 상태 |
|---|---|---|---|---|
| E1-F20 | [#746](https://github.com/hyujun/rtc-framework/issues/746) | 비교 평가 — search 둘 (`grid` · `nlp`), planner 셋 (`closed_form` · `mpc` · `mpc_docking`). E1-F19 에서 넘어온 것 (채택값, 쓴 seed — 1 부 801 · 901 · 821 · 822, 2 부는 1001 – 1156 의 대역이라 판정은 1xxx 를 쓰지 않는다, 판정의 설계에 닿는 것 — 같은 값 · 같은 seed 의 반복이 50 투척에 ±4 – 5 건 흔들린다): [#746 의 인계 코멘트](https://github.com/hyujun/rtc-framework/issues/746#issuecomment-6062694993) · [2 부의 인계](https://github.com/hyujun/rtc-framework/issues/746#issuecomment-6093942041). 투척 모집단은 E1-F21 의 결과로 정했다 — search 비교와 planner 셋 비교 모두 로봇별 D-1 세트 (130 발) + 후보 상자 목록 (180 발) 이다 ([search](https://github.com/hyujun/rtc-framework/issues/746#issuecomment-6082876755) · [planner](https://github.com/hyujun/rtc-framework/issues/746#issuecomment-6083281041) — G-1 의 `s35b` 상자는 쓰지 않고 #632 의 수치와 이어지지 않는다) | §1 의 단독 이슈 | 대기 |

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
| E3-F01 | [#640](https://github.com/hyujun/rtc-framework/issues/640) | G1 구성의 포구 후보 선택 (바깥 루프 — MD-46 의 편차를 닫는다). 계획기 interface · search 선택 키 · 단일 팔의 바깥 루프는 E1-F12 · F14 · F16 으로 옮겼다 | E1-F08 (끝), E1-F12 (끝) · F14 | 대기 |
| E3-F02 | [#641](https://github.com/hyujun/rtc-framework/issues/641) | 전신 항을 E1 코어에 추가 — 왼팔 rest · waist 억제 · 각운동량 · waist 토크 행 · counter-swing, 관절군별 move blocking | E1-F07 (끝), E2-F01 (끝) | 대기 |
| E3-F03 | [#642](https://github.com/hyujun/rtc-framework/issues/642) | 충돌 제약 — capsule 모델 (신규 코어) 과 MPC 제약 행 (자기충돌 · 공–왼팔 거리) | E3-F02 | 대기 |
| E3-F04 | [#643](https://github.com/hyujun/rtc-framework/issues/643) | MPC ↔ CLIK 계약의 전신 확장 — payload 관절 용량 (8 → G1 $n$ 17), 왼손 FK, 한계 여유. 단일 팔 계약은 `ID_INDEX.md` MD-36 · `ref/L7_supervisor.md` §4.3a. E2-F04 가 넣은 것: CLIK 의 자세 속도 feedforward (`qd_posture_ff`) 와 관절군별 자세 과제 — 전신 구간의 $\dot q_{ref}$ 는 그 입력으로 간다 | E1-F08 (끝), E2-F04 (끝) | 대기 |
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

**실기 planner (선행 조건).** 사용자 결정: 지금은 planner 를 정하지 않는다. 실기 기본 측정 (단계 D) 은 planner 와 무관하게 진행하고, **단계 E 앞에서** `closed_form` 과 `mpc` 중 어느 쪽으로 갈지 정한다. 지금 실기 config 는 planner 를 덮지 않아 출하값 `planner.segment.mode: mpc` 가 그대로 실기에 간다. `mpc` 로 가려면 먼저 닫아야 하는 것 넷 (정지 구간과 작업셀 경계 — E-8 에 걸린다, G8-F 에 mpc 의 abort 경로, #654 의 ProxQP heap 할당 (RT-1 수용 예외 여부), `mpc` 를 실기에서 잰 적이 없다는 것) 은 #613 의 "planner" 절이 갖는다.

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
| `docs/catching-docking-eval` | E1-F20 | 게이트 판정 |
| `feat/catching-mpc-candidate-select` | E3-F01 | G1 구성의 후보 선택. interface 는 E1-F12 가 넣었다 |
| `feat/catching-mpc-wholebody` | E3-F02, E3-F04 | 전신 항과 MPC ↔ CLIK 계약. 둘 다 활성 관절을 전체로 넓히는 작업이고 계약의 sanity check 가 전신 해를 입력으로 쓴다 |
| `feat/collision-capsule-core` | E3-F03 | 신규 수치 코어. code review 단위 |
| `feat/g1-catch-controller` | E3-F05 | supervisor · 손 시퀀서 통합. E-STOP 경로를 건드리면 E-8 |
| `feat/g1-catch-tooling-eval` | E3-F06, E3-F07 | 로그 · plot · GUI 와 평가. 평가가 새 로그 컬럼을 쓴다. **나누는 조건**: 평가 결과가 설계 결정을 바꾸면 결과 기록을 `docs/` 브랜치로 분리한다 |

**순서 — E1.** 남은 feature 브랜치는 평가 하나다 — 그 앞의 단독 이슈와 순서는 §1.

**순서 — E3.** E3 의 다섯 브랜치 (착수 조건인 E2 와 E1-F07 은 끝났다, MD-47). 병행은 서로 다른 패키지를 고치는 브랜치끼리만 한다.

### 게이트 · escalation

| Feature | 사유 | 효력 |
|---|---|---|
| E1-F20 | solver 의 heap 할당 (RT-1 → E-1) — 계획기 스레드는 FIFO 일 수 있고 ProxQP 의 할당은 수용된 예외가 아니다 (#654). 두 docking 기능 (`nlp` · `mpc_docking`) 은 "실기의 FIFO 계획기 스레드에서는 #654 뒤에 돌린다" 는 조건으로 들어왔고 (#739 · #740 · #742), 실기 configuration 이 그 가운데 하나를 고르면 configure 가 park 한다 (E1-F16) | Critical — 그 조건을 벗어나거나 그 park 를 풀려면 착수 전 `[CONCERN]` 과 컨펌 |
| E3-F05 | E-STOP 경로를 건드리면 E-8 | Critical |
| 실기 (HW) | `mpc` 의 정지 구간과 작업셀 경계 (RT 도 탐색도 포구점 · 정지점의 위치를 판정하지 않는다 — MD-73 · L3 §4.9), `mpc` 구간의 샘플 시각을 바꾸는 RT 법칙 변경 — 둘 다 E-8 (#613) | Critical — 착수 전 `[CONCERN]` 과 컨펌 |
| E3-F03 | 신규 수치 코어 (100+ 줄) | code review |

## 4. 남은 feature 를 구속하는 결정

아직 구현하지 않은 feature 를 구속하는 `MD-n` 이다. 나머지 `MD-n` 은 [ID_INDEX.md](ID_INDEX.md) §4. E1-F20 을 구속하는 결정 (solver · 조합 · 선택 키 · 식의 형태 · 1 차 범위 · 판정 방식 — 끝난 E1-F19 의 것과 같은 곳) 은 `MD-n` 이 아니다 — 각 feature 이슈의 범위에 적혀 있고 선택지와 이유는 [#621 의 결정 코멘트](https://github.com/hyujun/rtc-framework/issues/621#issuecomment-5974865100) 에 있다. 그 가운데 **D5-2 ("바깥 루프는 RT 가 plan 을 채택할 때까지만") 는 뒤집혔다**: 탐색은 채택 뒤에도 돈다 — 처음 채택한 plan 의 $t_c$ 에서 `t_stop_plan` 앞까지, 모든 조합 (`grid` × `mpc` 포함) 에서. 채택 뒤의 탐색은 E1-F16 이, 교체의 게시와 RT 의 교체 쌍 채택은 E1-F17 이 구현했다 (L3 §5.3 · L7 §4.3a). 무엇을 어느 feature 가 맡는지는 [#740 의 확정 코멘트](https://github.com/hyujun/rtc-framework/issues/740#issuecomment-6009380305) (N10 – N12) 와 #742 · #743 의 본문에 있다.

| ID | 무엇을 정했나 | feature |
|---|---|---|
| MD-14 | CLIK 의 충돌 damper 는 선택 항이고 기본 꺼짐이다 (기존 로봇의 golden 회귀를 지키기 위해). E3-F03 의 거리 코어 뒤에 넣는다. 같은 결정의 제동 거리 한계와 오차 되먹임 상한은 E2-F04 가 넣었다 (`ID_INDEX.md` MD-14) | E3-F03 뒤 |
| MD-86 | `sim_g1_p1b` 의 `enable_mpc` 는 CPU layout 만 고른다. G1 의 포구 컨트롤러가 오면 그 기본값을 그 feature 에서 다시 정한다 (E2-F05 는 바꾸지 않았다) | E3-F05 |
| MD-12 | 각운동량 항은 유지한다 — 목적은 왼팔의 운동 생성과 floating base 확장이고, 효과는 포구 성공률이 아니라 각운동량 변화율의 크기로 판정한다 | E3-F02 |
| MD-13 | 토크는 1차까지 선형화하고 토크 · 자기충돌 행에는 slack 을 둔다. 위치 · 속도 한계, 종단 등식, trust region, 공–왼팔 행은 hard 다. 동역학은 손을 기준 자세로 잠근 축소 모델로 계산한다 | E3-F02 · F03 |
| MD-46 | 설계는 formulation §1.3 하나다. g1_p1b 는 같은 코어에 dual arm · waist 전용 항 (waist 억제, 왼팔 rest, 각운동량, 자기충돌, 공–왼팔, waist 토크 행 · counter-swing, 관절군별 move blocking) 을 더한다. 포구 후보 ($t_c$) 를 MPC 의 바깥 루프가 고르게 하는 것이 닫아야 할 편차다 | E1-F14 (탐색 코어 — 끝) · F16 (계획기가 부른다 — 끝) · E3-F01 · F02 |
| MD-47 | E3 의 착수 조건은 E2 (g1_p1b 준비) 와 E1-F07 (코어) 이다. G-1 은 조건이 아니다 | E3 |
| MD-49 | 코어는 비용 · 제약을 항 단위로 조립하고 관절군별 move blocking 행렬 $E$ 의 자리만 둔다. 다관절군 일반화는 E3 에서 한다 | E3-F02 |

## 5. 아직 정하지 않은 것

열린 결정은 이슈가 갖는다. 제목은 이슈 제목이다.

**남은 feature 의 결정**

- [#746](https://github.com/hyujun/rtc-framework/issues/746) — E1-F20: 비교 평가 — 투척 모집단은 정했다 (§3 의 그 행). 그 목록에서 `grid` 를 쓰는 arm 셋이 비교가 되는지는 #800 의 결과 (`grid` 가 낸 plan 은 본 자료 전부에서 순위 게이트에 걸려 있다 — [결과](https://github.com/hyujun/rtc-framework/issues/800#issuecomment-6089985934) · [결정](https://github.com/hyujun/rtc-framework/issues/800#issuecomment-6090237604)) 를 보고 시행 전 고정 때 정한다
- [#640](https://github.com/hyujun/rtc-framework/issues/640) — E3-F01: G1 구성의 MPC 포구 후보 선택 (바깥 루프)
- [#642](https://github.com/hyujun/rtc-framework/issues/642) — E3-F03: 충돌 제약 — capsule 모델 · 자기충돌 · 공–왼팔 거리
- [#645](https://github.com/hyujun/rtc-framework/issues/645) — E3-F06: 로그 · plot_rtc_log · demo_controller_gui (dual arm 열) — G1 포구 컨트롤러의 상태를 GUI 에 무엇으로 보일지 (E2-F06 은 상태 토픽 없이 세션 CSV 의 꼬리를 읽었다 — GUI 가 컨트롤러와 같은 머신에 있어야 한다. `rtc_msgs/CatchingState` 에 열을 더하면 E-3)

**E1-F21 에서 넘어온 일**

- `grid` 의 교체 규칙이 plan 을 유지하는 동작 ([#793 의 merge 코멘트](https://github.com/hyujun/rtc-framework/issues/793#issuecomment-6077965184)) 은 볼지 정하지 않았다 — plan 이 게시되는 조합이 생기기 전에는 sim 에서 보이지 않는다

**E1-F19 에서 넘어온 일**

- **종료 기준의 미충족.** D-2 튜닝 세트의 확인 unit (로봇 · 조합마다 2 개) 에서 탐색 수용 (≥ 70 %) 은 `ur5e_p1b` 16 / 28 · `iiwa7_leap` 8 / 14, plan 게시 (≥ 80 %) 는 `nlp` × `mpc_docking` 9 / 18 · 0 / 14 와 `grid` × `mpc_docking` 3 / 51 · 0 / 45, 끊긴 풀이 (≤ 5 %) 는 39 – 70 % 다. 추종 (HOLD · `ABORT_SAFE`) 은 게시된 `ur5e_p1b` 의 12 발에서 채웠고 `iiwa7_leap` 은 확인 unit 에서 게시가 없어 판정할 표본이 없다 (2 부의 다른 unit 을 다 합쳐 게시 7 발). 기준은 바꾸지 않았고 값은 사용자 결정으로 넣었다 — [#745 의 결과 코멘트](https://github.com/hyujun/rtc-framework/issues/745#issuecomment-6093512941) · [결정](https://github.com/hyujun/rtc-framework/issues/745#issuecomment-6093537834)
- **값으로 더 움직이지 않은 셋.** 풀이 시간 (T [#803](https://github.com/hyujun/rtc-framework/issues/803) 의 착수 조건 — 끊긴 풀이가 어느 값의 unit 에서도 31 % 아래로 내려가지 않았다), plan 의 게시 관문과 `grid` 가 고른 포구점 (S [#804](https://github.com/hyujun/rtc-framework/issues/804) 의 입력 — velocity-set 만으로 거부된 후보는 0 건이다), 포구 순간의 위치 오차 ([#807](https://github.com/hyujun/rtc-framework/issues/807) — 게시된 `ur5e_p1b` 의 66 발 가운데 포구 4, 손이 공에서 p50 34 mm; 착수는 지시로)
- **확인하지 못한 값.** `iiwa7_leap` 의 `publish.slack_c_max` 0.06 m 는 통로 입구 반지름의 2.7 배라 통로 관문을 사실상 여는데, 2 부에서 이 로봇이 게시한 구간 (7 unit 의 26 개) 은 모두 통로 slack 이 0 이었다. 0 으로 돌린 unit 하나에서는 통로를 6.4 mm 벗어난 (다른 관문은 지난) 첫 풀이 하나가 보류됐다 — 두 값 사이는 재지 않았고, 게시가 열리는 S 에서 다시 본다. `core.sqp.mu_max` 1e2 (벌점 probe 없음) 가 `grid` × `mpc_docking` 에서 풀리는 문제를 거부하는지는 같은 seed 의 unit 한 쌍씩으로만 보았다 (게시 1 대 1 · 0 대 0 — 차이 없음)
- **`T_close_lead` 를 옮긴 것의 여파 (`ur5e_p1b`).** `grid` 탐색의 commit-lead 순위 게이트의 문턱이 0.3615 → 0.383 s 로 올라 평가 overlay 의 후보 창 하한 (0.37 s) 을 넘었다 — 그 13 ms 안의 후보는 이제 벌점 비트를 받는다 (거부가 아니라 순위; `grid` 를 쓰는 모든 조합에 닿는다). 평가 overlay `catch_lead_on` 의 `T_freeze` 0.37 s 는 설계 하한보다 13 ms 작고 (configure 의 검사는 통과한다), `io.horizon_min` 0.51 s 는 그 유도식의 합보다 13 ms 작다 (진단만 바뀌는 값이다). 둘과 그 시험 (`test_catch_lead_overlays.py` 는 lead 가 아니라 `T_close_e2e` 로 하한을 계산한다) 은 2 부에서 옮기지 않았고, lead 의 규칙과 검증기를 고치는 R 이 함께 본다 ([#805 의 코멘트](https://github.com/hyujun/rtc-framework/issues/805#issuecomment-6094381336)). sim 의 포구 시행에서는 lead 를 옮긴 효과를 확인하지 못했다 ([ref/L6_hand.md](ref/L6_hand.md) §4.6)
- 교체 쌍의 채택 · 전환은 여전히 sim 에서 실행되지 않았다 (확인 unit 에서 시도 2, 풀이 0)
- **1 부가 남긴 "값으로 풀리지 않은 것"** 은 그대로다 — 포구 시각의 가속 (손이 접근축으로 앞선다 — #807 · R 이 닿는다), 확률 행과 함께 꺼지는 공분산 유효성 검사 (2 부 뒤로는 두 로봇 모두 확률 행이 꺼져 있다), 마지막 구간의 교체 풀이, `nlp` 의 `solve_s` 상한을 configure 가 검사하지 않는 것, `iiwa7_leap` 의 `speed` 보류: [#745 의 머지 코멘트](https://github.com/hyujun/rtc-framework/issues/745#issuecomment-6062693925). 그 목록의 "폐쇄 창의 축 (0.55 m/s 기준)" 은 2 부가 출하 기준 속력에서 다시 잰 창으로 바꿨다 ([ref/L6_hand.md](ref/L6_hand.md) §4.6)

**mpc planner · catching 의 남은 일**

- [#710](https://github.com/hyujun/rtc-framework/issues/710) — iiwa7_leap: `mode: mpc` 가 포구 계획을 내지 못한다 — 첫 풀이의 기준 궤적
- [#755](https://github.com/hyujun/rtc-framework/issues/755) — catching: 포구 층에서 발화하지 않는 `JOINT_CONFLICT` 경로 — 지울지 정한다 (#712 후속)
- [#713](https://github.com/hyujun/rtc-framework/issues/713) — catching: 조정되지 않은 채 출하된 mpc planner 의 값 — `cost.w_perp` · `publish.slack_terminal_max`
- [#716](https://github.com/hyujun/rtc-framework/issues/716) — catching: formulation 의 포구 구간 다중 노드 (K_c) 와 그 위의 경로 이탈 항 `w_path` 가 구현에 없다
- [#715](https://github.com/hyujun/rtc-framework/issues/715) — integrated_bringup: 세 군 (두 번째 손) 지원 — 보류

**`mpc planner · catching 의 남은 일` 다섯의 순서 (권장).** 이 문서에서 다섯 이슈의 시기는 이 표 하나가 갖는다 — 다른 절은 여기를 가리키고 시기를 다시 적지 않는다. 묶지 않는다 — 이슈 하나가 브랜치 하나이다. 이슈에 정해진 시기 (#710 · #713 · #716 은 E1-F20 뒤) 안에서 고른 순서이고, "착수 때 정할 것" 은 아직 열려 있다.

| 순서 | 이슈 | 착수 조건 | 브랜치 | 착수 때 정할 것 |
|---|---|---|---|---|
| 1 | #755 | 충족됐다 — E2-F04 (#636) 는 끝났고 `bound_conflict` 를 다시 쓰지 않는다 ([결정 코멘트](https://github.com/hyujun/rtc-framework/issues/755#issuecomment-6049207494)) | 지우면 단독이다 (`rtc_msgs` 를 고치면 E-3). 그대로 두면 브랜치가 없다 | 지울 범위 |
| 2 | #713 | E1-F20 뒤, #755 뒤 | 단독 — 정지 직선 · 이탈 거리의 로그 열과 `cost.w_perp` 의 값 | — |
| 3 | #710 | E1-F20 의 `iiwa7_leap` 결과 | 결과에 따라 단독 또는 없음 | `mpc` 를 고칠지, leap 의 출하 기본값을 바꿀지 |
| — | #716 | E1-F20 뒤 | 정하지 않았다 | 단일 팔에서도 할지, G1 구성의 항으로 E3 에서 할지 (권장은 E3) |
| — | #715 | 보류 — 착수 조건이 없다 | — | — |

- **묶지 않는 이유.** #710 · #716 의 범위는 E1-F20 의 결과가 나와야 정해진다. `mpc` 의 법칙을 한 PR 에서 두 곳 바꾸면 측정이 원인을 가르지 못한다 (#710 은 leap 의 계획 게시율, #713 은 정지 구간의 직선 이탈을 본다). PR 을 쌓지 않으므로 같은 파일을 고친다는 것은 묶을 이유가 아니다.
- **#755 와 #713 은 CSV 의 열을 바꾼다.** #755 가 지우면 `catching_diag` 의 열 둘이 빠지고, #713 이 `planner_events` 에 열을 더한다. 새 열은 새 이름으로 생긴다.
- **#755 의 "msg 는 두고 전이 행만 지운다" 는 그대로는 되지 않는다.** 전이표는 어느 행에도 쓰이지 않는 사유를 거부하고 (`transition_table.hpp` 끝의 `static_assert`), 사유의 값은 `rtc_msgs` 의 상수와 번호가 같아야 한다. `rtc_msgs` 를 고치지 않는 형태는 발화하는 한 줄만 지우고 사유와 행을 두는 것이다.
- **#716 에서 정하지 않은 것.** formulation §1.6 은 단일 팔의 포구 구간이 한 노드인 것을 손 폐쇄 명령 시각 ($t_c-T_{close}$ — 폐쇄가 $t_c$ 에 끝난다) 의 귀결로 적는다. 여러 노드로 두려면 세 planner 가 함께 쓰는 그 규칙이 바뀐다.

**이 문서가 정하지 않은 것.** 실기 epic 과 E3 의 선후 (실기는 `ur5e_p1b` 이고 E3 는 G1 sim 이라 서로의 선행이 아니다). 실기 planner 는 단계 E 앞에서 정한다 (§3).
