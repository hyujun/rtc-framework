# dynamic_catching — 포구 알고리즘 설계 문서와 참조 구현

구현 대상은 **기존 `rtc-framework` workspace** 다. 새 workspace를 만들지 않고, 이미 있는
kinematics·dynamics·CLIK/QP·joint command backend를 재사용한다(마스터 §1.2).

- 문서 버전: v0.5 (2026-09-19)
- 설계 문서: `CATCHING_MASTER.md` + `WORKSPACE_ANALYSIS.md` + `L0_core.md` … `L8_bringup.md`
- 참조 구현: 아래 헤더 4개와 테스트. **v0.4 검증 산출물**이다(컴파일·실행, ASan/UBSan 통과). 알려진 결함은
  아래 "참조 구현의 알려진 결함" 절이며 S1 이식 시 고친다 — 이 폴더의 헤더는 고치지 않는다.

## 구현 계획 (living document)

전체 구현 계획·결정 로그·단계 상태는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) 가 SSoT 다. 아래 설계 문서와
충돌하면 그 문서의 결정이 우선한다. 구현이 끝나면 prune 한다.

## 단계 W (완료) 와 문서 동기화 상태

단계 W 는 코드 대조로 끝났다(2026-09-19). 기록은 `WORKSPACE_ANALYSIS.md`, 요약은 plan §2 에 있다.
설계 문서 전체(마스터·W·L0~L8)는 S0.3 에서 plan 결정에 맞춰 v0.5 로 동기화한다 — ros2_control·`update()`
전제 제거, 시간 규약 D-2, 손 포트 추상화 삭제(D-11), derate v1 제외(D-8), L0·L2·L3·L4 §5 의 코드 복사본을
헤더 포인터로 대체. 상태는 plan §4 의 S0 행이 SSoT 다.

## 입력 계약

vision(ball_perception `sim_estimator_node`)이 `sensor_msgs/PointCloud2` 로 **예측 궤적**을 발행한다(마스터 §5,
D-4). 점 하나가 $(p, v, a)$ + 공분산 $\Sigma_{6\times6}$(NaN = 모름) + `horizon_ns` + `generation`·`validity`·
`snapshot_sequence` 이고, `header.stamp` 가 예측 원점 시각이다. debug 토픽이라 stable ABI 가 아니다. 제어 PC는 이
궤적을 **재전파하지 않고 그대로 신뢰**하며, 샘플 사이만 보간한다(L2).

## 파일

| 파일 | Layer | 내용 |
|---|---|---|
| `WORKSPACE_ANALYSIS.md` | W | `rtc-framework` 분석 항목과 코드 대조 기록 (단계 W 완료, 2026-09-19) |
| `traj_sampler.hpp` | L2 | vision 궤적의 5차 Hermite 보간($C^2$), 외삽 플래그, 형식 검사 |
| `soft_catch_reference.hpp` | L4 (L3·L5 공용) | `GammaProfile`, `SoftCatchTranslation`, 접근축 정렬, `criticallyDampedError` |
| `time_feasibility.hpp` | L3 | 관절 최소 도달시간 `tMinChecked`, γ 창 `gammaWindow`, `maxCatchableSpeed` |
| `ball_dynamics.hpp` | L0 | 공 운동 모델, RK4, 상태전이행렬 — **시뮬레이션·테스트 fixture 전용**(L0 §1, S1.6) |
| `test_l2.cpp` | L2 | 샘플 복원, $C^2$ 연속성, 순수 중력 정확성, 외삽·커서·형식 검사 |
| `test_l4.cpp` | L4 | §4.9 표, §4.3 점프식, §4.7 이산 안정 경계, §4.5 축 정렬, C3/D3 회귀 |
| `test_l3.cpp` | L3 | `tMin` 검산 + `cases.txt` 생성, γ 창·[R1] sanity, §4.8 표 재현 |
| `test_l0.cpp` | L0 | §4.4 sanity 4종 + 회귀(저속 ∂A/∂k, `propagate` truncated) |
| `verify_l3.py` | L3/L4 | `tMin` vs 선형계획, `cases.txt` 대조, 축 오차 Jacobian vs 유한차분 |
| `_REVIEW_catching_2026-09-18.md` | — | v0.1 대상 리뷰 (v0.2가 반영한 항목의 근거). 이 저장소에 없고 참조 구현 빌드와 무관하다 |

L4의 병진 DS는 `soft_catch_reference.hpp` 하나뿐이다. L3의 γ rollout과 L4 실행이 **같은 코드**를 쓴다.
접근축 정렬은 S2.1 에서 `rtc_math` se3 로 옮긴다(D-1).

## 빌드·실행

```bash
for t in l0 l2 l3 l4; do
  g++ -std=c++20 -O2 -Wall -Wextra -I/usr/include/eigen3 test_$t.cpp -o test_$t && ./test_$t
done
python3 verify_l3.py        # numpy, scipy 필요. test_l3 가 만든 cases.txt 를 읽는다
```

네 실행 파일 모두 실패 시 exit code 1을 낸다. S1.1 에서 rtc_controllers 의 `catching` 하위 디렉토리로 이식하며 GTest로 옮긴다(D-1).

## 참조 구현의 알려진 결함 (S1 에서 수정)

기존 테스트 밖에서 확인한 것이다. 이식 단계별 요구는 plan §4 S1 이 SSoT 다.

- `n > kMaxSamples` 에서 traj `check`·`sampleAt`·L1 readTraj 가 범위 밖 읽기 (ASan 확인) — 점 개수를 먼저 검사 (S1.2)
- `sampleAt(NaN)` 이 valid, `interpolate` 가 극소 간격 허용 — NaN 거부, `dt_min` 미만 거부 (S1.2)
- `SoftCatchTranslation::step` 이 NaN 목표로 영구 오염, saturated 미검출 — 상태 보존 + invalid (S1.4)
- `axisAlignJacobian` 이 antiparallel·deadband 에서 NaN·폭주 — `rtc_math` se3 이식 시 유한 보장 (S2.1)
- `tMinChecked` 가 한계 ≤ 0 이면 t = 0 을 표시 없이 반환 — 잘못된 한계는 flag (S1.5)
- `directionalSpeedMax` 는 투영 $\hat v^\top J_p\dot q$ 여야 하고 qd_max 0 가드 필요 (S1.5)
- `derateGamma` 결함 — D-8 로 v1 에서 제외, 이식하지 않는다
- rollout 예산 추정 ~26 ms vs 예산 10 ms — coarse-to-fine 필요 (S6.3)
- ζ ≠ 1·ω·h 범위 검증 필요 (S1.7), 명명 규칙을 repo 에 맞춤 (namespace `rtc`, 함수 PascalCase)
- SeqLock 에 싣는 타입은 `std::array` 기반 POD 여야 한다 (Eigen 멤버 금지, plan §6)

## 주의

- 참조 구현은 **ROS 의존이 없다**. `PointCloud2` 파싱(L1 §5.1)은 `integrated_bringup` 바인딩에서 구현하며(D-1),
  본 폴더의 코드는 그 결과를 받는 순수 수치 부분만 담는다.
- 이 폴더는 브랜치 `docs/dynamic-catching-plan` 에 커밋된다(A-2).
