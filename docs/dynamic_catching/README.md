# dynamic_catching — 포구 알고리즘 설계 문서

구현 대상은 **기존 `rtc-framework` workspace** 다. 새 workspace를 만들지 않고, 이미 있는
kinematics·dynamics·CLIK/QP·joint command backend를 재사용한다(마스터 §1.2).

- 설계 문서: `CATCHING_MASTER.md` + `WORKSPACE_ANALYSIS.md` + `L0_core.md` … `L8_bringup.md` (헤더 버전 v0.5 — 이후 개정은 plan 의 결정을 따라 해당 절만 고친다)
- 구현: rtc_controllers `catching/` (수치 코어) · `rtc_math` se3 (축 정렬) · `rtc_tsid` (CLIK 확장) · `integrated_bringup` (바인딩·YAML·launch). 배치 결정은 plan D-1

## 구현 계획 (living document)

전체 구현 계획·결정 로그·단계 상태는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) 가 SSoT 다. 아래 설계 문서와
충돌하면 **IMPLEMENTATION_PLAN.md 의 결정이 우선한다**. 구현이 끝나면 prune 한다.

단계의 상태는 plan 의 상태줄과 §4.3 표가 갖는다 — 여기에 사본을 적지 않는다.
진행 기록은 Epic [#537](https://github.com/hyujun/rtc-framework/issues/537), S10 의 잔여 작업은 [#613](https://github.com/hyujun/rtc-framework/issues/613) 이다.
plan 은 2026-09-29 에 제자리 압축했다 (절 번호·식별자 불변) — 압축 전 전문은 `git show b0ea0996:docs/dynamic_catching/IMPLEMENTATION_PLAN.md`.

## 확장 — MPC · dual-arm

MPC 는 waist + dual-arm (G1) 용으로 설계하고, ur5e_p1b · iiwa7_leap 에서 dual arm · waist 항을 뺀 같은 MPC 로 먼저 시험한 뒤 (APPROACH–정지), G1 + proto_1b bring-up 을 거쳐 같은 코어에 그 항을 더한다. closed_form 과 mpc 는 입력 (공의 미래 궤적) 과 출력 (CLIK 입력) 이 같은 두 planner 다. 이 확장은 별도 계획으로 관리한다. 계획·결정(`MD-n`)·상태의
SSoT 는 [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md), 정식화는 [mpc_multiframe_clik_formulation.md](mpc_multiframe_clik_formulation.md),
추적은 GitHub project [rtc-framework — MPC · dual-arm catching](https://github.com/users/hyujun/projects/2) 다.

## 단계 W (완료) 와 문서 동기화 상태

단계 W 는 코드 대조로 끝났다(2026-09-19). 기록은 `WORKSPACE_ANALYSIS.md`, 요약은 plan §2 에 있다.
설계 문서 전체(마스터·W·L0~L8)는 S0.3 에서 plan 결정에 맞춰 v0.5 로 동기화했다 — ros2_control·`update()`
전제 제거, 시간 규약 D-2, 손 포트 추상화 삭제(D-11), derate v1 제외(D-8), L0·L2·L3·L4 §5 의 코드 복사본을
헤더 포인터로 대체. 상태는 plan §4 의 S0 행이 SSoT 다.

## 입력 계약

vision(ball_perception `sim_estimator_node`)이 `sensor_msgs/PointCloud2` 로 **예측 궤적**을 발행한다(마스터 §5,
D-4). 점 하나가 $(p, v, a)$ + 공분산 $\Sigma_{6\times6}$(NaN = 모름) + `horizon_ns` + `generation`·`validity`·
`snapshot_sequence` 이고, `header.stamp` 가 예측 원점 시각이다. debug 토픽이라 stable ABI 가 아니다. 제어 PC는 이
궤적을 **재전파하지 않고 그대로 신뢰**하며, 샘플 사이만 보간한다(L2).

vision 은 vision PC, 제어기는 제어 PC 에서 돈다. 두 쪽은 위 토픽으로만 만나고 서로의 파일을 읽지 않는다 — 추정기의
profile 은 ball_perception 저장소가 소유한다 ([MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) MD-17 · MD-18).

## `ref/` — 참고 자료의 원본

계획에 참고하는 자료 (수학적 알고리즘 · 전체 구조) 의 원본을 [ref/](ref/) 에 보관한다.

- 목적은 **지금 구현의 수학과 구조를 표현하는 것** 이다 — 경위의 기록이 아니다
- 구현과 다르게 적힌 곳은 지금 구현으로 고쳐 쓴다. **수학이 달라지는 수정은 사용자의 승인을 받고 한다.** 그 밖에는 될 수 있으면 고치지 않는다
- 아직 구현과 맞추지 않았다. 알려진 차이는 [#705](https://github.com/hyujun/rtc-framework/issues/705) 의 "코드와 어긋난 곳" 에 있다
- 2026-10-04 에 이 폴더의 같은 이름 문서를 그대로 복사했다. 원문과 다른 것은 상대 링크의 경로뿐이다 (폴더가 한 단계 깊어져서). 이 폴더 바로 아래의 같은 이름 문서는 재정비 (#705) 의 대상이다
- 들어 있는 것: `mpc_multiframe_clik_formulation.md` · `ball_catching_inverse_dynamics_mpc.md` · `CATCHING_MASTER.md` · `L0_core.md` … `L8_bringup.md`

## 파일

| 파일 | Layer | 내용 |
|---|---|---|
| `IMPLEMENTATION_PLAN.md` | — | 결정 로그·단계 상태·게이트 결과 (SSoT) |
| `CATCHING_MASTER.md` | 전체 | 문제 정의·계층 구조·교차 제약·이론 출처 표 |
| `WORKSPACE_ANALYSIS.md` | W | `rtc-framework` 분석 항목과 코드 대조 기록 (단계 W 완료, 2026-09-19) |
| `L0_core.md` | L0 | 시간 타입·용량 상수·파라미터 검증, 공 운동 모델 (fixture 전용) |
| `L1_io.md` | L1 | `PointCloud2` 예측 궤적 수신·파싱·검증, 스냅샷 브리지, 로봇·센서 상태 |
| `L2_prediction.md` | L2 | 궤적 타입, 5차 Hermite 샘플러 |
| `L3_planner.md` | L3 | 포구 시각·포구점·접근축·γ 결정 (IK·도달시간·γ 창·계획기 스레드) |
| `L4_reference.md` | L4 | soft-catch DS, 접근축 정렬, 복귀 |
| `L5_joint_cmd.md` | L5 | `ClikReferenceGenerator` 확장 옵션과 팔 명령 바인딩 |
| `L6_hand.md` | L6 | 손 시퀀서·손 프로파일·`T_close` 식별 |
| `L7_supervisor.md` | L7 | 상태 머신·접촉 판정·감속·abort |
| `L8_bringup.md` | L8 | 컨트롤러 통합·launch·YAML·sim 기반·로깅·시스템 검증 |
| `MPC_DUALARM_PLAN.md` | — | MPC · dual-arm 확장의 epic·feature 계획, 결정 로그, 게이트 결과 (SSoT) |
| `mpc_multiframe_clik_formulation.md` | — | waist + dual-arm MPC 계획기와 다중 frame CLIK 의 수학적 정리 |

## 삭제된 참조 구현

v0.4 검증 산출물이던 참조 헤더 4개 (`traj_sampler.hpp`·`soft_catch_reference.hpp`·`time_feasibility.hpp`·
`ball_dynamics.hpp`) 와 단독 테스트 (`test_l0.cpp`·`test_l2.cpp`·`test_l3.cpp`·`test_l4.cpp`·`verify_l3.py`) 는
S1 에서 rtc_controllers `catching` 으로 이식된 뒤 pre-S10 R1 (2026-09-28) 에서 삭제됐다. 빌드가 참조하지 않았고,
알려진 결함은 이식 때 모두 고쳤다 (plan §4.4 S1 결과). 설계 문서의 수치 중 이 테스트들이 낸 것은 기록으로 남기며,
원본은 삭제 직전 커밋에서 읽는다:

```bash
git show 482d18b3:docs/dynamic_catching/<파일>
```

| 삭제된 파일 | 이식 위치 |
|---|---|
| `traj_sampler.hpp` · `test_l2.cpp` | `rtc_controllers/include/rtc_controllers/catching/traj_sampler.hpp` · `test/test_catching_traj_sampler.cpp` |
| `soft_catch_reference.hpp` · `test_l4.cpp` | `catching/soft_catch.hpp` · `test/test_catching_soft_catch.cpp` (축 정렬은 `rtc_math` se3 `axis_align.hpp`) |
| `time_feasibility.hpp` · `test_l3.cpp` · `verify_l3.py` | `catching/time_feasibility.hpp` · `test/test_catching_time_feasibility.cpp` (`verify_l3.py` 의 닫힌식 결과는 고정 테이블로) |
| `ball_dynamics.hpp` · `test_l0.cpp` | `rtc_controllers/test/include/rtc_controllers/testing/catching_ball_fixture.hpp` · `test/test_catching_ball_fixture.cpp` |
