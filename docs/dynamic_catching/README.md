# dynamic_catching — 포구 알고리즘 문서

vision 이 예측한 공의 궤적을 받아 팔과 손으로 공을 잡는 컨트롤러의 문서다. 구현은 기존 `rtc-framework` 패키지에 있다 — `rtc_controllers` 의 `catching/` (수치 코어), `rtc_math` se3 (축 정렬), `rtc_tsid` (CLIK), `integrated_bringup` (컨트롤러 · YAML · launch).

## 무엇이 어디에 있나

| 찾는 것 | 자리 |
|---|---|
| 수학과 구조 — 지금 구현이 무엇을 하는가 | [ref/](ref/) |
| 코드 주석의 `MD-45` · `D-16` · `G3-I` · `S6-B` · `plan §9` 가 무슨 뜻인가 | [ID_INDEX.md](ID_INDEX.md) |
| 지금 상태, 남은 feature, 아직 정하지 않은 것 | [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) — MPC · dual-arm 확장과 실기 단계의 계획. 구현이 끝나면 지운다 |
| 값 (YAML 키의 값) | `integrated_bringup/config/<robot>/controllers/` 의 YAML |
| 운용 방법 (launch · 도구 · 분석) | `integrated_bringup/README.md`, `rtc_tools/README.md` |
| 측정 · 검증 기록, 결정의 경위 | 이슈 — Epic [#537](https://github.com/hyujun/rtc-framework/issues/537) (단일 팔 포구), project [MPC · dual-arm catching](https://github.com/users/hyujun/projects/2), 실기 [#613](https://github.com/hyujun/rtc-framework/issues/613) |

같은 사실은 한 곳에만 적는다. 계획 문서에는 영구히 보관할 정보를 넣지 않는다 — 구현이 끝나면 지우는 파일이다.

## `ref/` — 수학과 구조

| 문서 | 내용 |
|---|---|
| [CATCHING_MASTER.md](ref/CATCHING_MASTER.md) | 문제 정의, 계층 구조, 층 사이의 제약, 이론의 출처, 위험 |
| [L0_core.md](ref/L0_core.md) | 시간 타입과 판정별 비교 축, 용량 상수, 파라미터 검증, 공 운동 모델 (fixture 전용) |
| [L1_io.md](ref/L1_io.md) | `PointCloud2` 예측 궤적의 수신 · 검증, `header.stamp` 사용 계약, 스냅샷 전달 |
| [L2_prediction.md](ref/L2_prediction.md) | 궤적 타입, 5 차 Hermite 샘플러 |
| [L3_planner.md](ref/L3_planner.md) | 포구 시각 · 포구점 · 접근축의 탐색 (IK, 도달시간, γ 창, 순위), 계획기 스레드 |
| [L4_reference.md](ref/L4_reference.md) | soft-catch 기준 (`closed_form`), 접근축 정렬 |
| [L5_joint_cmd.md](ref/L5_joint_cmd.md) | CLIK 과제와 제약, 팔 지연의 선행 보상, catch frame |
| [L6_hand.md](ref/L6_hand.md) | 손 시퀀서, 손 프로파일, 폐쇄 시간의 식별 |
| [L7_supervisor.md](ref/L7_supervisor.md) | 상태 머신, abort 사유, 감속, `mpc` 의 구간 추종, 결과 판정, E-STOP · fault |
| [L8_bringup.md](ref/L8_bringup.md) | 컨트롤러 통합, sim 기반, 평가, GUI · plot |
| [grid_search_closed_form_formulation.md](ref/grid_search_closed_form_formulation.md) | 구현된 격자 탐색 (`planner.search.grid.*`, `GridCatchSearch`) 과 `closed_form` planner 의 팔 기준 법칙 (soft-catch DS → 등감속 정지) 을 한 문제의 순서로 적은 수식 — L3 · L4 · L7 에 흩어진 수학의 통합본이고 코드의 분기와 대응한다. 결정 · YAML 범위 · 게이트 · 디버깅은 그 층별 문서가 갖는다 |
| [mpc_multiframe_clik_formulation.md](ref/mpc_multiframe_clik_formulation.md) | MPC planner 의 문제 (구현된 단일 팔 구성은 §1.6), 아직 구현하지 않은 dual-arm · waist 항과 다중 frame CLIK 의 설계 |
| [ball_catching_inverse_dynamics_mpc.md](ref/ball_catching_inverse_dynamics_mpc.md) | 단일 arm-hand 의 NLP search (§11) · mpc_docking (§10) 의 설계 자료 (§0 – §16) 와 그 구현 (§17 — 17.$n$ 이 §$n$ 의 구현). 설계 절은 고치지 않고, 구현한 feature 가 §17 에 구현한 내용을 적는다 (지금은 mpc_docking 의 수치 코어 17.1 – 17.10, E1-F13 — 과 NLP search 의 탐색 코어 17.11 · 17.12, E1-F14). 남은 구현은 E1-F16 – F21 ([MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) §3 — 그 둘이 꽂힐 interface 는 E1-F12 가 넣었다) |

- 목적은 **지금 구현의 수학과 구조를 표현하는 것** 이다 — 경위의 기록이 아니다
- 구현과 다르게 적힌 곳은 지금 구현으로 고쳐 쓴다. **수학이 달라지는 수정은 사용자의 승인을 받고 한다.** 그 밖에는 될 수 있으면 고치지 않는다
- **설계 자료는 예외다 — 고쳐 쓰지 않는다 (사용자 결정).** 구현보다 먼저 쓴 설계 문서 (지금은 `ball_catching_inverse_dynamics_mpc.md`) 는 원래 설계와 구현을 견주어 볼 수 있게 설계 절을 그대로 두고, 구현한 내용을 문서 끝의 새 절에 원래 절과 대응시켜 적는다. 그 절에는 최종 구현만 적는다 — 달라진 경위는 feature 이슈의 구현 대조표가 갖는다
- 절 번호는 바꾸지 않는다. 코드 주석이 `L3 §6`, `formulation §1.3` 식으로 인용한다. 새 절은 문서 끝에 더한다
- planner 는 둘이다: `closed_form` 과 `mpc` (출하 기본값). 포구 후보의 탐색은 두 planner 가 공유하고, `mpc` 에서는 APPROACH 부터 정지까지 팔 기준을 MPC 구간이 만든다. 두 번째 탐색 (`nlp`) 과 세 번째 planner (`mpc_docking`) 는 아직 고를 수 없다 — 둘이 같이 쓸 수치 코어 (`MpcDockingSegmentCore`, E1-F13) 와 탐색 코어 (`NlpCatchSearch`, E1-F14) 가 있고 그것들을 부르는 것은 테스트뿐이다 (남은 것은 E1-F16 – F21). 그 둘이 꽂힐 추상 interface (`CatchSearch` · `SegmentPlanner`) 는 있다 (E1-F12, [L3 §4.1](ref/L3_planner.md))

## 입력 계약

vision (ball_perception 의 `sim_estimator_node`) 이 `sensor_msgs/PointCloud2` 로 **예측 궤적** 을 발행한다. 점 하나가 $(p, v, a)$ + 공분산 $\Sigma_{6\times6}$ (NaN = 모름) + `horizon_ns` + `generation` · `validity` · `snapshot_sequence` 이고, `header.stamp` 가 예측 원점 시각이다. 제어 PC 는 이 궤적을 **재전파하지 않고 그대로 신뢰** 하며 샘플 사이만 보간한다 (형식과 검증은 [ref/L1_io.md](ref/L1_io.md), 보간은 [ref/L2_prediction.md](ref/L2_prediction.md)).

vision 은 vision PC, 제어기는 제어 PC 에서 돈다. 두 쪽은 위 토픽으로만 만나고 서로의 파일을 읽지 않는다 — 추정기의 profile 은 ball_perception 저장소가 소유한다 (MD-17 · MD-18).

## 지워진 문서

v1 계획 (`IMPLEMENTATION_PLAN.md`) 과 workspace 분석 기록 (`WORKSPACE_ANALYSIS.md`) 은 지웠다. 내용의 새 자리는 [ID_INDEX.md](ID_INDEX.md) §1 이고, 원문은 git 이력에 있다.

```bash
git show 5278ca25:docs/dynamic_catching/IMPLEMENTATION_PLAN.md   # 지우기 직전
git show b0ea0996:docs/dynamic_catching/IMPLEMENTATION_PLAN.md   # 2026-09-29 압축 전의 전문
```

v0.4 의 참조 구현 (헤더 4 개와 단독 테스트) 은 `rtc_controllers` 의 `catching/` 으로 이식한 뒤 지웠다 — 이식한 헤더의 머리 주석이 원본의 커밋을 적는다.
