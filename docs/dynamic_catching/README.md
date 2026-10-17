# dynamic_catching — 포구 알고리즘 문서

vision 이 예측한 공의 궤적을 받아 팔과 손으로 공을 잡는 컨트롤러의 설계·구조 문서 모음이다. **유형**: 문서 폴더 색인

## 개요

구현은 기존 패키지에 있다 — `rtc_controllers` 의 `catching/` (수치 코어), `rtc_math` se3 (축 정렬), `rtc_tsid` (CLIK), `integrated_bringup` (컨트롤러 · YAML · launch). 이 폴더는 그 구현의 수학과 구조를 적는다.

**처음 읽는다면** [ref/CATCHING_MASTER.md](ref/CATCHING_MASTER.md) (문제 정의와 층 구조) 로 시작해 `L0` → `L8` 순으로 내려간다. 입력인 예측 궤적의 형식은 [ref/L1_io.md](ref/L1_io.md) 가 소유한다.

| 찾는 것 | 자리 |
|---|---|
| 수학과 구조 — 지금 구현이 무엇을 하는가 | [ref/](ref/) |
| 코드 주석의 `MD-45` · `D-16` · `G3-I` · `S6-B` · `plan §9` 가 무슨 뜻인가 | [ID_INDEX.md](ID_INDEX.md) |
| 지금 상태, 남은 feature, 아직 정하지 않은 것 | [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) — 구현이 끝나면 지우는 계획 문서 |
| 값 (YAML 키의 값) | `integrated_bringup/config/<robot>/controllers/` 의 YAML |
| 운용 방법 (launch · 투척 시행 · 분석) | [docs/catching-sim-trials.md](../catching-sim-trials.md) · [rtc_tools/README.md](../../rtc_tools/README.md) |
| 측정 · 검증 기록, 결정의 경위 | 이슈 ([더 읽기](#더-읽기)) |

같은 사실은 한 곳에만 적는다. 계획 문서에는 영구히 보관할 정보를 넣지 않는다.

### 문서를 고칠 때

- 목적은 **지금 구현의 수학과 구조를 표현하는 것**이다 — 경위의 기록이 아니다. 구현과 다르게 적힌 곳은 지금 구현으로 고쳐 쓰되, **수학이 달라지는 수정은 사용자의 승인을 받는다.**
- **설계 자료는 고쳐 쓰지 않는다 (사용자 결정).** 구현보다 먼저 쓴 설계 문서 (지금은 `ball_catching_inverse_dynamics_mpc.md`) 는 설계 절을 그대로 두고, 구현한 내용을 문서 끝의 새 절에 원래 절과 대응시켜 적는다. 달라진 경위는 feature 이슈의 구현 대조표가 갖는다.
- **절 번호는 바꾸지 않는다.** 코드 주석이 `L3 §6`, `formulation §1.3` 식으로 인용한다. 새 절은 문서 끝에 더한다.
- 이 폴더의 파일은 이름을 바꾸거나 옮기지 않는다 (코드 주석이 경로와 ID 를 인용한다).

## 디렉토리

```
docs/dynamic_catching/
├── README.md              이 색인
├── ID_INDEX.md            코드 주석 ID 의 색인
├── MPC_DUALARM_PLAN.md    MPC · dual-arm 확장 계획 (임시)
└── ref/                   수학과 구조 (아래 표)
```

| 문서 | 내용 |
|---|---|
| [CATCHING_MASTER.md](ref/CATCHING_MASTER.md) | 문제 정의, 계층 구조, 층 사이의 제약, 이론의 출처, 위험 |
| [L0_core.md](ref/L0_core.md) | 시간 타입과 판정별 비교 축, 용량 상수, 파라미터 검증, 공 운동 모델 (fixture 전용) |
| [L1_io.md](ref/L1_io.md) | `PointCloud2` 예측 궤적의 수신 · 검증, `header.stamp` 사용 계약, 스냅샷 전달 |
| [L2_prediction.md](ref/L2_prediction.md) | 궤적 타입, 5 차 Hermite 샘플러 |
| [L3_planner.md](ref/L3_planner.md) | 포구 시각 · 포구점 · 접근축의 탐색 (IK, 도달시간, γ 창, 순위), 계획기 스레드, 탐색·구간 모드 선택 키 (§6) |
| [L4_reference.md](ref/L4_reference.md) | soft-catch 기준 (`closed_form`), 접근축 정렬 |
| [L5_joint_cmd.md](ref/L5_joint_cmd.md) | CLIK 과제와 제약, 팔 지연의 선행 보상, catch frame |
| [L6_hand.md](ref/L6_hand.md) | 손 시퀀서, 손 프로파일, 폐쇄 시간의 식별 |
| [L7_supervisor.md](ref/L7_supervisor.md) | 상태 머신, abort 사유, 감속, `mpc` 의 구간 추종, 결과 판정, E-STOP · fault |
| [L8_bringup.md](ref/L8_bringup.md) | 컨트롤러 통합, sim 기반, 평가, GUI · plot, 탐색의 판정 지도 도구 |
| [grid_search_closed_form_formulation.md](ref/grid_search_closed_form_formulation.md) | 격자 탐색 (`GridCatchSearch`) 과 `closed_form` 팔 기준 법칙을 한 문제의 순서로 적은 수식. L3 · L4 · L7 의 통합본 |
| [mpc_multiframe_clik_formulation.md](ref/mpc_multiframe_clik_formulation.md) | MPC planner 의 문제 (단일 팔 구성은 §1.6), 다중 frame CLIK (식 §2, `rtc_tsid` 구현 §10, 바인딩 §11), 미구현 dual-arm · waist 설계 |
| [ball_catching_inverse_dynamics_mpc.md](ref/ball_catching_inverse_dynamics_mpc.md) | NLP search (§11) · mpc_docking (§10) 의 설계 자료 (§0 – §16) 와 그 구현 (§17, 17.$n$ 이 §$n$ 의 구현) |

로그 · 도구와 판정 지도의 도구는 §17 이 아니라 [L3_planner.md](ref/L3_planner.md) §8 · 부록 C, [L8_bringup.md](ref/L8_bringup.md) §11 · §12 가 적는다.

## 더 읽기

- 이슈: Epic [#537](https://github.com/hyujun/rtc-framework/issues/537) (단일 팔 포구), project [MPC · dual-arm catching](https://github.com/users/hyujun/projects/2), 실기 [#613](https://github.com/hyujun/rtc-framework/issues/613)
- 지워진 문서: v1 계획 (`IMPLEMENTATION_PLAN.md`) 과 workspace 분석 기록 (`WORKSPACE_ANALYSIS.md`) 의 내용은 [ID_INDEX.md](ID_INDEX.md) §1 로 옮겼고, 원문은 git 이력에 있다.

```bash
git show 5278ca25:docs/dynamic_catching/IMPLEMENTATION_PLAN.md   # 지우기 직전
git show b0ea0996:docs/dynamic_catching/IMPLEMENTATION_PLAN.md   # 2026-09-29 압축 전의 전문
```
