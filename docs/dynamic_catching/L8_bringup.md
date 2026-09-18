# L8 — Bringup: 컨트롤러 통합, launch·YAML, 시뮬레이션 fixture, 로깅·지표, 시스템 검증

- 브랜치: `feat/catching-L8-bringup`
- 패키지: `catching_bringup`, `catching_sim_fixtures` (테스트 전용)
- 선행: 단계 W, L0–L7
- 비고: 컨트롤러 기반 클래스·lifecycle·로깅은 workspace의 기존 구조를 따른다(W2-1, W2-7). 본 layer는 L0–L7을 그 구조에 **끼워 넣는다**.
- 산출물: `CatchingController` 플러그인, launch 3종, YAML 3종, 투척 생성기, 시뮬레이션 vision 발행기(`PointCloud2`), 기록기, 지표 스크립트

---

## 1. 범위 / 비범위

범위:
- L0–L7을 하나의 ros2_control 컨트롤러로 묶는다.
- 시뮬레이션(`iiwa7_leap`, `ur5e_p1b`)과 실기(`ur5e_p1b`) 실행 구성을 제공한다.
- 시스템 수준 검증을 수행한다.

비범위: vision 노드 구현. 단 시뮬레이션에서 그 역할을 대신하는 fixture는 포함한다(계약 준수가 목적).

## 2. 코드 확인 게이트

단계 W에서 처리한다(W2, W4, W6).

| ID | 확인 항목 | 기록 |
|---|---|---|
| G8-1 | 컨트롤러 기반 클래스·lifecycle 훅, non-RT 스레드 소유 규약 | TBD-RTC-19 (W2-1) |
| G8-2 | MuJoCo ↔ ros2_control 연동 방식(패키지·플러그인), `/clock` 발행, 공(freejoint) 상태 접근 방법 | TBD-SIM-01 |
| G8-3 | 두 MJCF의 지문 센서 정의(`<force>`/`<torque>` 쌍)와 발행 경로 | TBD-HAND-03 |
| G8-4 | 기존 로깅 도구(RT 레코드 형식, rosbag 사용 규약) | TBD-RTC-20 |
| G8-5 | 컨트롤러가 backend 명령 자리를 어떻게 잡는지(claim), 실기/시뮬레이션 전환 | TBD-ARM-01 (W4-1, W4-2) |

## 3. 참고자료

[R8] NEES/NIS, [R13] Wilson 구간, [R14] MuJoCo, [R11] UR 드라이버.

## 4. 수학적 이론

### 4.1 RT 틱 내 실행 순서

한 틱 안의 순서를 고정한다. 순서가 바뀌면 1틱 지연이 생긴다.

1. 상태 읽기: `RobotSnapshot`(L1), `PredictedTrajectory`·`PlanSnapshot` `tryRead` (L1)
2. 공 상태: `traj::sampleAt(tr, t_rel, hint_)`, 선행 $T_{lead}$ 적용 (L2 §4.4, L5 §4.5)
3. 감독: `CatchSupervisor::update` → 모드, 대상, γ, intercept 설정 (L7)
4. 기준 생성: `SoftCatchTranslation::step`, `axisAlignOmega` (L4)
5. 관절 명령: `CatchTaskAdapter::update` (L5)
6. 손: `HandSequencer::update` → 포트 (L6)
7. 명령 쓰기: $q_c$ 를 **기존 joint command backend**가 읽는 자리에 싣는다 (W4-1 규약). 손은 L6 포트로.
8. RT 상태 발행: `RtStateSnapshot`(L3용) SeqLock write, 기록 레코드 SPSC push

### 4.2 투척 생성 (시뮬레이션) `[논문 외 설계]`

목표 포구 영역 안의 점 $p_T$와 비행시간 $T_f$를 표본 추출한다. 발사점 $p_0$에서 $T_f$ 뒤 $p_T$에 도달하는 초기속도 $v_0$를 구한다.

1. 무항력 초기값: $v_0^{(0)}=\dfrac{p_T-p_0-\tfrac12gT_f^2}{T_f}$
2. 항력 보정(슈팅, 뉴턴 반복): 최종 위치 오차 $r=p(T_f;v_0)-p_T$, 상태전이행렬의 위치-속도 블록 $\Phi_{pv}=\partial p(T_f)/\partial v_0$ (L0 `rk4WithStm`)로

$$v_0\leftarrow v_0-\Phi_{pv}^{-1}r$$

   수 회 반복으로 수렴한다(항력이 작아 초기값이 가까움).

3. MJCF 유체 모델과 L0 모델이 다르므로, 생성된 투척의 MuJoCo 실제 도달점은 약간 다르다. 이 차이는 의도된 모델 불일치로 기록한다.

표본 분포(속도, 거리, 영역)는 TBD-BALL-02 확정 후 정한다.

### 4.3 시뮬레이션 vision 발행기 (`PointCloud2`)

MuJoCo 참값 → 측정 모사(노이즈, 지연, 누락, 이상치) → 참조 EKF + 예측(L0 fixture 라이브러리 사용, 60 Hz) → **vision과 같은 `PointCloud2` 레이아웃으로 발행**.

레이아웃은 W5-2에서 확정한 실제 `PointField` 배열을 그대로 복제한다. 그래야 L1 파서가 시뮬레이션과 실기에서 같은 경로를 탄다. 레이아웃 해시가 일치하는지 테스트로 고정한다(G8-B2).

- 스탬프는 측정 샘플링 시각(sim time)으로 하고, 실제 발행은 지연만큼 늦춘다.
- 참조 EKF는 vision 노드 구현이 아니라 **계약 검증용**이다. 두 구현이 달라도 메시지 계약만 같으면 제어 PC 코드는 바뀌지 않는다.
- 이것이 `ball_dynamics.hpp`가 남아 있는 이유다(L0 §1). 실시간 제어 경로에서는 쓰지 않는다.
- 참값 토픽(`/ball/truth`)을 별도로 발행해 지표 계산에만 쓴다. 제어 경로에서는 쓰지 않는다.
- 실행 위치: 기본은 제어 PC(시뮬레이션과 같은 머신). 네트워크 경로 검증 단계에서는 vision PC에서 실행한다 `[권장]`.

### 4.4 지표

**추정 일관성 (시뮬레이션).** 발행 상태와 참값의 NEES:

$$\epsilon_x=(x_{true}-\hat x)^\top P^{-1}(x_{true}-\hat x)$$

평균이 상태 차원(6, $k$ 미추정 시)에 가까운지 본다([R8]). $k$를 추정하지 않으면 $P$의 $k$ 행·열이 0이므로 6×6 블록만 쓴다.

**포구 지표 (시행별).**
- $t_c$에서 간극: $\Vert p_{true}(t_c)-p_C(t_c)\Vert$ (catch frame 실제 위치)
- 상대속도: $\Vert v_{true}(t_c)-\dot p_C(t_c)\Vert$
- **충격량** $\Delta p=m_{ball}\Vert v_{true}(t_c)-\dot p_C(t_c)\Vert$, 접촉 지속 $\Delta t_{imp}$, 평균·최대 접촉력, 손가락 관절 최대 토크 (시뮬레이션 접촉 참값) — L7 §4.7
- 계획 $\gamma_f$ 와 **실제 사용 $\gamma_f$**(하향 후), 하향 횟수, 포화 발생, 전이 사유, 결과

**성공률 신뢰구간 (Wilson, [R13]).** $n$회 중 $s$회 성공, $\hat p=s/n$, $z$는 정규 분위수:

$$\frac{\hat p+\frac{z^2}{2n}\pm z\sqrt{\frac{\hat p(1-\hat p)}{n}+\frac{z^2}{4n^2}}}{1+\frac{z^2}{n}}$$

**오차 예산 검증.** L3 §4.6 예측 간극 분포와 실제 간극 분포를 비교한다(모델 검증).

## 5. C++ 구현

### 5.1 컨트롤러 구조

```cpp
class CatchingController /* : RTC 컨트롤러 기반 클래스 (G8-1) */ {
 public:
  // non-RT
  CallbackReturn on_configure(...);   // YAML 로드·검증(L0), **기존 kinematics 핸들 취득**(W3),
                                      // PointCloud2 FieldMap 구성(L1 §5.1), 버퍼 할당, 구독 생성
  CallbackReturn on_activate(...);    // q_c = q_meas, 기준 상태 초기화, 계획·기록 스레드 시작, Mode::Idle
  CallbackReturn on_deactivate(...);  // 스레드 정지 요청·join, 마지막 명령 유지
  // RT
  return_type update(const rclcpp::Time& time, const rclcpp::Duration& period);  // §4.1 순서

 private:
  TrajReceiver receiver_;             // L1 PointCloud2 수신·파싱 (non-RT callback group)
  CatchPlanner planner_;              // L3 (non-RT thread)
  traj::PredictedTrajectory traj_;    // L1 → L2, int hint_ 커서 동반
  CatchSupervisor sup_;               // L7
  ref::SoftCatchTranslation trans_;   // L4
  joint::CatchTaskAdapter adapter_;   // L5
  HandSequencer hand_;                // L6
  std::unique_ptr<HandCommandPort> hand_port_;   // configure에서 생성 (in_loop / async)
  LogRing log_ring_;                  // SPSC → 기록 스레드
};
```

### 5.2 기록 레코드

```cpp
struct TickRecord {                   // 고정 크기, POD
  Nanoseconds t;
  std::uint8_t mode, reason;
  std::uint32_t track_epoch, plan_id;   // track_epoch: L1 §4.4 (PointCloud2 에 track_id 없음)
  float gamma, gamma_d, gamma_dd;       // TranslationOutput 이 셋 다 돌려준다 (L4 §5.1)
  std::array<float, 3> x_ref, xd_ref, xdd_ref, u_des, e, ed, ball_p, ball_v;
  // xdd_ref = 실현 가속도, u_des = 포화 전 DS 요구값 (L4 §5.2). 둘을 함께 기록해야
  // 포화 구간 해석이 된다. e, ed 는 한 틱 이전 기준이다.
  std::array<float, 7> q_cmd, q_meas;
  float hand_rho;
  std::array<float, 4> tip_force;     // 센서 수 상한 (G8-3)
  std::uint8_t flags;                 // sat, bound_active, bound_conflict, stale, ...
};
```

- 기록 스레드: 링에서 꺼내 바이너리 파일로 쓴다. 플롯 스크립트는 파일을 읽는다.
- 토픽 기록: rosbag2로 vision `PointCloud2`, `/ball/truth`(시뮬레이션), 진단, plan 이벤트를 기록한다.

### 5.3 launch 구성

| 이름 | 로봇 | 시계 | 손 포트 | 공 입력 |
|---|---|---|---|---|
| `sim_iiwa7_leap.launch.py` | `iiwa7_leap` | sim | `in_loop` | 시뮬레이션 발행기 |
| `sim_ur5e_p1b.launch.py` | `ur5e_p1b` | sim | `in_loop` | 시뮬레이션 발행기 (선택: `servoj` 지연 에뮬레이션) |
| `hw_ur5e_p1b.launch.py` | `ur5e_p1b` | system (PTP) | `async` | vision PC |

### 5.4 YAML 파일

- `config/catching_common.yaml`: layer 공통 기본값 (`core`, `io`, `prediction`, `planner`, `reference`, `joint_cmd`, `supervisor`, `logging`)
- `config/catching_iiwa7_leap.yaml`: `robot.*`, 시뮬레이션 값
- `config/catching_ur5e_p1b.yaml`: `robot.*`, 실기 식별값 (`T_arm`, `T_close`, `T_link` 등)

로드 순서는 common → robot이며, robot 파일이 덮어쓴다. 로드 후 L0 검증기를 실행한다.

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `logging.ring_capacity` | int | – | 4096 | 256–65536 | 기록 스레드 지연 흡수 |
| `logging.decimation` | int | – | 1 | 1–50 | 기록 부하 |
| `logging.dir` | string | – | `TBD` | – | 운용 환경 |
| `sim.throw.target_box` | box | m | `TBD` | – | TBD-BALL-02 |
| `sim.throw.launch_box` | box | m | `TBD` | – | TBD-BALL-02 |
| `sim.throw.flight_time` | double[2] | s | `TBD` | – | TBD-BALL-02 |
| `sim.meas.rate` | double | Hz | 60 | – | 확정 조건 |
| `sim.meas.sigma` | double[3] | m | `TBD` | ≥0 | 실측 또는 가정 명시 |
| `sim.meas.delay` | double[2] | s | `TBD` | ≥0 | 균일 분포 범위 |
| `sim.meas.dropout` | double | – | 0.0 | 0–0.5 | – |
| `sim.meas.outlier_rate` | double | – | 0.0 | 0–0.1 | – |
| `sim.ekf.q_acc` | double | m²/s³ | `TBD` | ≥0 | fixture EKF 공정잡음 (L2는 더 이상 쓰지 않음) |
| `sim.vision.layout` | — | – | `TBD` | – | 실기 `PointField` 배열 복제 (W5-2) |
| `sim.trials` | int | – | 200 | ≥30 | 신뢰구간 폭과 연동 |

**실기에서 검증 불가능한 것 `[권장]`.** 실기에는 $p_{true}(t_c)$ 가 없으므로 "예측 간극 분포"를 직접 잴 수 없다. 관측 가능한 것은 포획/실패 이진 결과와 지문 접촉 시각뿐이다. 따라서 실기 게이트(G8-G)는 다음으로 대체한다.

1. 포획률 대 $\sigma_c$ 의 로지스틱 회귀로 유효 $r_{cap}/\kappa_\sigma$ 를 역추정.
2. 접촉 센서 조합·접촉 시각 편차를 간극의 대용 지표로 사용.
3. 저속 구간 1회 한정 외부 계측(마커 또는 고속 카메라) 캠페인.

셋 다 하지 않으면 **"실기 vision 공분산은 미검증 가정"** 임을 마스터 §10 위험 목록에 올린 채로 진행한다.

## 7. 단위 기술 구현 순서

- **L8.1** G8 게이트.
- **L8.2** 컨트롤러 골격 + §4.1 순서 + lifecycle.
- **L8.3** 기록 링·기록 스레드·플롯 스크립트.
- **L8.4** 투척 생성기(슈팅 수렴 테스트 포함).
- **L8.5** 시뮬레이션 vision 발행기(`PointCloud2`) + NEES 검증.
- **L8.6** `sim_iiwa7_leap` 폐루프 → 시스템 게이트 G8-A~C.
- **L8.7** `sim_ur5e_p1b` 폐루프 → G8-D.
- **L8.8** 실기 단계적 도입 (§9.2).

## 8. 디버깅 방법

- 틱 실행시간 분해: §4.1 단계별 시간 히스토그램.
- 결과별 타임라인 자동 생성: 실패·abort 시행만 모아 L7 §8 그래프를 만든다.
- 재현성: 투척 난수 시드와 YAML 해시를 시행 기록에 넣는다.
- 틱 예산 분해: L1 스냅샷 복사 바이트, SeqLock 재시도 횟수, QP 반복 수·solve time을 따로 기록한다. 전형 40–90 µs / 2 ms 예산이라 총량이 아니라 **분산의 꼬리**가 위험하다.
- 오차 원인 분해: 간극을 L3 §4.6의 항별($A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$, 추종, 시계)로 분해해 표로 만든다. 시뮬레이션은 참값이 있으므로 각 항을 직접 계산할 수 있고, $A\perp B$ 가정도 검증할 수 있다(G8-C2).
- 충격 구간 분해: $t_c$ 전후 100 ms의 접촉력·관절 토크·$q-q_c$ 괴리를 겹쳐 그린다(L7 §4.7).

## 9. 검증 방법과 합격 게이트

### 9.1 시뮬레이션

| 게이트 | 기준 | 태그 |
|---|---|---|
| G8-A | 전체 시행에서 RT 위반 0 (page fault, 할당, 틱 초과 기준은 RTC 규약). **CPU 격리 확인 포함** — non-RT 계획 스레드(`budget_s` 10 ms)가 RT와 같은 코어에 놓이면 QP 실행시간 분산이 통째로 흔들린다 | `[SIM-ANY]` |
| G8-A2 | **연속 2회 투척**과 **abort 직후 재투척** 시나리오에서 L7 §4.8 재무장 리셋 목록이 전부 동작 (옛 plan 재사용 0, 복귀 위치가 홈, 첫 틱 `bound_conflict` 0) | `[SIM-ANY]` |
| G8-B | 시뮬레이션 발행기 NEES 평균이 [R8] 구간 안 | `[SIM-ANY]` |
| G8-B2 | 시뮬레이션 발행기의 `PointCloud2` 레이아웃 해시가 실기 vision 것과 일치 (L1 파서가 동일 경로) | `[SIM-ANY]` |
| G8-B3 | 실기 vision 공분산의 일관성: 참값이 없으므로 예측 간극 분포로 간접 검증(G8-C2와 짝) | `[HW-P1B]` |
| G8-C | `iiwa7_leap`에서 $\gamma=0$ 고정 대 계획 γ ablation: 간극·상대속도·**충격량**·접촉력 분포 비교 | `[SIM-ANY]` |
| G8-C2 | 오차 예산 모델 검증: L3 §4.6 직교 분해식의 예측 간극 분포와 실제 분포 비교. 참값이 있으므로 $A=p_{true}-\hat p_{live}$, $B=\hat p_{live}-p_c$ 의 직교성도 직접 확인한다 | `[SIM-ANY]` |
| G8-C3 | γ 하향(L7 §4.6) ablation: 하향 경로 유무에 따른 abort율·성공률 비교 | `[SIM-ANY]` |
| G8-D | `ur5e_p1b`에서 성공률(Wilson 95% 구간), 결과별 사유 분포. 합격 하한은 사용자 결정 | `[SIM-P1B]` |
| G8-E | `ur5e_p1b` + `servoj` 지연 에뮬레이션에서 선행 보상 유무 비교 | `[SIM-P1B]` |

### 9.2 실기 단계적 도입 `[권장]`

1. **공 없이 재생.** 기록된 vision `PointCloud2` bag을 재생한다. 저속 설정(투척 속도 축소 bag)으로 로봇·손 동작과 타이밍을 확인한다.
2. **가상 공.** 제어 PC 내부 가상 궤적으로 전 구간(감속·복귀 포함)을 확인한다.
3. **실제 공, 저속 토스.** 부드러운 공, 짧은 거리, 낮은 속도로 시작한다.
4. **속도 단계 상향.** 각 단계에서 G8-D 지표를 기록하고 다음 단계로 간다.

각 단계 진입 전 `T_arm`, `T_close_tot`, 시계 offset, 그리고 **예측 일관성 지표 $\bar\nu$**(L1 §4.5)를 확인한다. v0.2가 쓰던 vision NIS는 `PointCloud2`에 없어 삭제됐다(마스터 §5.1).

| 게이트 | 기준 | 태그 |
|---|---|---|
| G8-F | 단계 1–2 완료, abort 경로 전부 실기 동작 확인 | `[HW-P1B]` |
| G8-G | 단계 3 이후 성공률·간극 분포 기록, 시뮬레이션 대비 차이 원인 분해 | `[HW-P1B]` |

## 10. 미확정 항목

TBD-RTC-19, TBD-RTC-20, TBD-SIM-01, TBD-HAND-03, TBD-ARM-01, TBD-BALL-02, `sim.meas.*`, `logging.dir`.
