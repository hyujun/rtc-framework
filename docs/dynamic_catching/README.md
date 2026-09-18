# dynamic_catching — 포구 알고리즘 설계 문서와 참조 구현

구현 대상은 **기존 `rtc-framework` workspace** 다. 새 workspace를 만들지 않고, 이미 있는
kinematics·dynamics·CLIK/QP·joint command backend를 재사용한다(마스터 §1.2).

- 문서 버전: v0.4
- 설계 문서: `CATCHING_MASTER.md` + `WORKSPACE_ANALYSIS.md` + `L0_core.md` … `L8_bringup.md`
- 참조 구현: 아래 헤더 4개. 문서 작성 시 컴파일·실행으로 확인한 코드다.

## 구현 계획 (living document)

전체 구현 계획·결정 로그·단계 상태는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) 가 SSoT 다. 아래 설계 문서와
충돌하면 그 문서의 결정이 우선한다. 구현이 끝나면 prune 한다.

## 착수 전 필수 — 단계 W

**`WORKSPACE_ANALYSIS.md` 를 먼저 끝낸다.** 이 문서 세트는 `rtc-framework` 의 실제 구조를
보지 않은 상태에서 작성됐다. 각 layer 문서 §2의 "코드 확인 게이트"가 그 미확인 항목이고,
단계 W가 한 번에 처리한다. 게이트가 빈 상태에서 나오는 상세 구현은 추측이다.

우선 확인 두 가지: **제어기가 명령을 어디에 싣는가**(W4-1), **vision 메시지의 실제 필드
레이아웃**(W5-2).

## 입력 계약

vision 노드가 `sensor_msgs/PointCloud2` 로 **예측 궤적**을 발행한다(마스터 §5). 점 하나가
$(p, v, a, t, \Sigma_{6\times6})$ 이고, `header.stamp` 가 예측 기준 시각이다. 제어 PC는 이
궤적을 **재전파하지 않고 그대로 신뢰**하며, 샘플 사이만 보간한다(L2).

## 파일

| 파일 | Layer | 내용 |
|---|---|---|
| `WORKSPACE_ANALYSIS.md` | W | `rtc-framework` 분석 항목(W1~W6)과 vision 측 요청 사항. **모든 구현의 선행 단계** |
| `traj_sampler.hpp` | L2 | vision 궤적의 5차 Hermite 보간($C^2$), 외삽 플래그, 형식 검사 |
| `soft_catch_reference.hpp` | L4 (L3·L5 공용) | `GammaProfile`, `SoftCatchTranslation`, 접근축 정렬, `criticallyDampedError` |
| `time_feasibility.hpp` | L3 | 관절 최소 도달시간 `tMinChecked`, γ 창 `gammaWindow`, `maxCatchableSpeed` |
| `ball_dynamics.hpp` | L0 | 공 운동 모델, RK4, 상태전이행렬 — **시뮬레이션 fixture 전용**(L0 §1) |
| `test_l2.cpp` | L2 | 샘플 복원, $C^2$ 연속성, 순수 중력 정확성, 외삽·커서·형식 검사 |
| `test_l4.cpp` | L4 | §4.9 표, §4.3 점프식, §4.7 이산 안정 경계, §4.5 축 정렬, C3/D3 회귀 |
| `test_l3.cpp` | L3 | `tMin` 검산 + `cases.txt` 생성, γ 창·[R1] sanity, §4.8 표 재현 |
| `test_l0.cpp` | L0 | §4.4 sanity 4종 + 회귀(저속 ∂A/∂k, `propagate` truncated) |
| `verify_l3.py` | L3/L4 | `tMin` vs 선형계획, `cases.txt` 대조, 축 오차 Jacobian vs 유한차분 |
| `_REVIEW_catching_2026-09-18.md` | — | v0.1 대상 리뷰 (v0.2가 반영한 항목의 근거). Dropbox 폴더에만 있고 참조 구현 빌드와 무관하다 |

L4의 병진 DS는 `soft_catch_reference.hpp` 하나뿐이다. L3의 γ rollout과 L5의 접근축 과제가
**같은 헤더**를 쓴다.

## 빌드·실행

```bash
for t in l0 l2 l3 l4; do
  g++ -std=c++20 -O2 -Wall -Wextra -I/usr/include/eigen3 test_$t.cpp -o test_$t && ./test_$t
done
python3 verify_l3.py        # numpy, scipy 필요. test_l3 가 만든 cases.txt 를 읽는다
```

네 실행 파일 모두 실패 시 exit code 1을 낸다. workspace 패키지로 이식할 때 GTest로 옮긴다.

## 주의

- 이 폴더는 Dropbox CloudStorage(online-only) 위에 있어 로컬 셸에서 파일을 열지 못하는 경우가
  있다. L0 착수 전에 실제 git 리포지토리로 옮긴다(`TBD-GIT-01`, W1-1).
- 참조 구현은 **ROS 의존이 없다**. `PointCloud2` 파싱(L1 §5.1)은 workspace 안에서 구현하며,
  본 폴더의 코드는 그 결과를 받는 순수 수치 부분만 담는다.
