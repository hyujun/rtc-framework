# L0 — Core: 공용 타입, 파라미터 검증, 공 동역학(시뮬레이션 fixture 전용)

- 문서 버전: v0.5 (2026-09-19) — 결정·단계의 SSoT 는 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) (충돌 시 plan 우선)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 배치 `[확정 D-1]`: rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`, ROS 비의존 순수 코드). 새 패키지를 만들지 않는다
- 단계: **S1** — 시간 타입(S1.3), 공용 궤적 타입의 POD 규칙(S1.2, 타입 정의는 L2), 파라미터 검증 로직(S1.7), `ball_dynamics` 의 test fixture 이전(S1.6)
- 선행: 단계 W (완료, plan §2)
- 산출물: 공용 타입 헤더(시간 타입·용량 상수), 파라미터 검증기, `ball_dynamics.hpp`(test fixture 전용)

---

## 1. 범위 / 비범위

범위: 시간·좌표 기본 타입, YAML 파라미터 검증기(활성 구성 TBD·provisional 검출, 교차제약 포함), 그리고 **시뮬레이션 fixture용** 공 운동 모델과 수치 적분.

비범위:
- 공 상태 추정·예측 (vision 노드 소관).
- **메시지 정의.** 입력은 vision(ball_perception)이 이미 발행하는 `sensor_msgs/PointCloud2`이고 제어 PC가 정의하지 않는다(마스터 §5, D-4). 공 발사 srv 는 `rtc_msgs` 에 추가하지만(D-14) S3 범위이며 L0 과 무관하다.
- 실시간 경로의 궤적 전파 (제어 PC는 재전파하지 않는다, 마스터 §5.2).
- YAML 로딩 자체. repo 관례(`LoadConfig` + `ParseXxxParams`)를 따르는 컨트롤러 바인딩이 한다(§5.3, S5.1).

**`ball_dynamics.hpp`의 위상 `[확정]`.** vision이 예측을 전담하므로 **실시간 제어 경로에서 쓰이지 않는다.** S1.6 에서 test fixture 전용 위치로 옮긴다(프로덕션 라이브러리에 넣지 않는다). 남는 용도는 두 가지뿐이다.

1. 시뮬레이션 fixture: MuJoCo 참값에서 vision 역할을 대신하는 테스트용 발행기·궤적 생성(L8 §4.3).
2. 투척 조건 역산: 목표 지점에 도달하는 초기속도를 슈팅으로 푸는 데 상태전이행렬이 필요하다(L8 §4.2). S3.5a catchability 지도 도구가 궤적 생성에 이 모델을 쓸지 sim 의 항력·Magnus 구현을 쓸지는 S3.5a 에서 정한다.

따라서 `kModelVersion`, 모델 버전 협상, 항력계수 $k$ 의 실시간 식별은 전부 삭제한다. $k$ 식별은 fixture를 실제 궤적에 맞출 때만 쓴다.

## 2. 코드 확인 게이트

단계 W 에서 처리했다(plan §2, `WORKSPACE_ANALYSIS.md` W1, W2, W6).

| ID | 확인 항목 | 방법 | 결과 기록 |
|---|---|---|---|
| G0-1 | SeqLock/SPSC 원시형 헤더 위치와 API (단일 writer 가정, reader 재시도 정책) | 소스 확인 (W2-2) | 닫힘 — `rtc::SeqLock` (단일 writer `Store`, reader `Load`; **재시도 상한 없음** — 단일 writer·유한 쓰기 시간을 설계 불변식으로 둔다), `rtc::SpscQueue`. 둘 다 payload 가 **trivially copyable** 이어야 한다(`static_assert`). `Eigen::Vector3d` 멤버는 불가 → `std::array` 기반 POD (W2-2, plan §6) |
| G0-2 | 시간 타입 규약 (ns 정수 / double s, clock type) | 소스 확인 (W2-3) | 닫힘 `[확정 D-2 (1)(2)]` — 내부 표현은 절대 steady `int64` ns, 타입 `BallTime`/`NowReal`/`NowLead` (plan §3, §4.5). RT 의 `ControllerState::t_relative_s` 는 steady 기반 세션 상대시각, `ControllerState::dt` = 1/`control_rate` 고정. `header.stamp` 를 수신 경계에서 1회 변환하는 것은 D-2 (3) 이며 E-1 기록된 예외로 승인됐다 (2026-09-19, S0.6, plan §3.1). stale 판정에는 쓰지 않는다 (W2-3) |
| G0-3 | 기존 YAML 파라미터 로딩 패턴 | 소스 확인 (W2-6) | 닫힘 — `LoadConfig(YAML)` + `ParseXxxParams` (non-RT, `on_configure`) + runtime gain 만 `declare_parameter`. `generate_parameter_library` 는 쓰지 않는다 (W2-6) |
| G0-4 | 포구 코드 배치·이름 | 사용자 결정 (W1-5) | 닫힘 `[확정 D-1]` — rtc_controllers `catching` 하위 디렉토리, namespace `rtc::catching` |
| G0-5 | sim 의 공 유체 모델 — fixture 한정 | sim 확인 (W6-3) | 닫힘 — `rtc_mujoco_sim` 이 항력 $\tfrac12\rho C_dA\Vert v\Vert v$ + Magnus 를 자체 구현한다(MJCF 유체 모델 아님, tennis preset r 0.025 m, m 0.05 kg). 본 모델(§4.1)은 Magnus 가 없으므로 $k$ 식별 잔차에 회전 효과가 남는다 (W6-3) |

게이트 결과가 본 문서 가정과 다르면, 본 문서를 먼저 수정한 뒤 구현한다.

## 3. 참고자료

[R8] 공분산 전파·변분방정식, [R14] MuJoCo. RK4와 변분방정식은 수치해석 표준 내용(등급 a).

## 4. 수학적 이론

### 4.1 공 운동 모델 `[시뮬레이션 fixture 전용]`

**이 절은 실시간 제어 경로와 무관하다**(§1). fixture·투척 조건 역산에만 쓴다.

상태 $x=[p;\,v;\,k]\in\mathbb R^7$ (`W` 기준). 이차 항력 모델:

$$\dot p=v,\qquad \dot v=g-k\Vert v\Vert v,\qquad \dot k=0,\qquad k=\frac{\rho C_dA}{2m}$$

- $k$는 공기밀도 $\rho$, 항력계수 $C_d$, 단면적 $A$, 질량 $m$에서 계산할 수 있으나, **sim 의 공 모델은 Magnus 항을 포함하므로(G0-5) 투척 데이터로 $k$를 최소제곱 식별한다** (§7).
- 회전(Magnus) 효과는 모델에 넣지 않는다. 잔차는 vision 추정기의 공정잡음이 흡수한다고 가정한다(가정 명시).

### 4.2 Jacobian

$A=\partial f/\partial x$:

$$A=\begin{bmatrix}0&I_3&0\\0&-k\Big(\Vert v\Vert I_3+\dfrac{vv^\top}{\Vert v\Vert}\Big)&-\Vert v\Vert v\\0&0&0\end{bmatrix}$$

유도: $\partial(\Vert v\Vert v)/\partial v=\Vert v\Vert I+v\,\partial\Vert v\Vert/\partial v=\Vert v\Vert I+vv^\top/\Vert v\Vert$.

$\Vert v\Vert\to0$에서 $vv^\top/\Vert v\Vert$ 항이 정의되지 않으므로 **그 항에만** $\Vert v\Vert\leftarrow\max(\Vert v\Vert,v_\epsilon)$로 하한을 둔다. 비행 중 공 속도는 0이 되지 않지만(정점에서도 수평 성분 존재) 수직 투척을 위한 방어다.

$\partial\dot v/\partial k=-\Vert v\Vert v$ 와 $-k\Vert v\Vert I$ 항에는 clamp를 걸지 않는다. clamp를 걸면 $\Vert v\Vert<v_\epsilon$ 에서 $A$ 와 $f$ 가 서로 다른 모델이 되어 STM이 $v_\epsilon/\Vert v\Vert$ 배 틀린다(`test_l0.cpp` 테스트 5가 회귀 검사).

### 4.3 이산화: RK4와 상태전이행렬

상태는 고전 RK4로 적분한다(국소 오차 $O(h^5)$, 전역 $O(h^4)$).

상태전이행렬 $\Phi(t,t_0)=\partial x(t)/\partial x(t_0)$는 변분방정식

$$\dot\Phi=A(x(t))\,\Phi,\qquad \Phi(t_0)=I$$

을 **상태와 같은 RK4 단계**로 함께 적분한다. 상태만 RK4로 적분하고 $\Phi\approx I+Ah$로 근사하면 1차 정확도로 떨어진다.

### 4.4 Sanity check (구현 테스트로 고정)

1. $k=0$이면 $p(t)=p_0+v_0t+\tfrac12gt^2$와 일치 (특수해).
2. 역학적 에너지 $E=\tfrac12\Vert v\Vert^2-g^\top p$는 비증가: $\dot E=v^\top(g-k\Vert v\Vert v)-g^\top v=-k\Vert v\Vert^3\le0$.
3. 스텝을 절반으로 줄이면 전역 오차가 약 1/16 (4차 수렴).
4. $\Phi$와 중심 유한차분 Jacobian 일치.
5. (회귀) $\Vert v\Vert<v_\epsilon$ 에서 $\partial\dot v/\partial k$ 가 닫힌식 $-\Vert v\Vert v$ 와 일치. 유한차분은 $g=9.81$ 에 묻혀 자릿수 손실이 나므로 닫힌식과 직접 비교한다.
6. (회귀) `propagate()` 의 `truncated` 계약. $n=$`max_steps` 는 포화를 뜻하지 않는다(정확히 나누어떨어지는 경우가 있다).

`test_l0.cpp` 실행 결과(v0.2): 해석해 오차 $1.6\times10^{-14}$ m, 에너지 증가분 0, 오차비 15.25, STM 유한차분 차이 $2.8\times10^{-9}$ (0.5 s 구간, $h=2$ ms), 저속 $\partial A/\partial k$ 상대오차 0, `truncated` 계약 통과.

### 4.5 시간 표현 `[확정 D-2]`

시간 규약의 SSoT 는 plan §3 이다. L0 은 그 타입만 제공한다.

- **내부 표현은 절대 steady `int64` ns.** 상대시각(double 초)은 수치 코어(샘플러·γ·rollout) 경계에서만 만들고, 원점이 다른 상대시각끼리 비교하지 않는다.
- **세 타입.** `BallTime`(공의 물리 시각 — $t_c$, $t_{cmd}$, 궤적 점 시각), `NowReal`(매 tick steady 실측 now), `NowLead`(= now + $T_{arm}$). 셋은 서로 암묵 변환되지 않는 강한 타입이고, 비교는 **타입별 오버로드로만** 제공한다 — 어떤 판정이 어떤 now 와 비교하는지(plan §3 표)가 타입으로 고정된다. 예: 샘플링·γ·DECEL 진입은 `NowLead` 대 `BallTime`, commit·손 명령·접촉 창은 `NowReal` 대 `BallTime`.
- **메시지 나이·stale** 은 `BallTime` 과 무관하게 steady 수신 시각 차(now_steady − recv_steady)로만 잰다. `header.stamp` 를 수신 시 1회 `BallTime` 원점으로 변환하는 것은 D-2 (3) 이며 E-1 기록된 예외로 승인됐다(L1 §4.1, plan §3.1, S0.6).
- 매 tick 의 now 는 steady 실측이며 tick 수 × `dt` 로 계산하지 않는다.
- 테스트는 **$T_{arm}\ne0$ fixture 필수** ($T_{arm}=0$ 이면 두 축이 같아져 버그가 숨는다).

## 5. C++ 구현

### 5.1 `ball_dynamics.hpp` (참조 구현, 검증 완료)

v0.5 에서 코드 복사본을 삭제했다. **SSoT 는 같은 폴더의 `ball_dynamics.hpp` (v0.4)** 다. S1 이식 시 변경:

- 위치: test fixture 전용 (S1.6). RT·프로덕션 타깃에서 include 하지 않는다
- 명명: namespace `rtc::catching` 아래 fixture 네임스페이스, 함수 PascalCase (`f`/`jacobian`/`rk4`/`rk4WithStm`/`propagate` → `F`/`Jacobian`/`Rk4`/`Rk4WithStm`/`Propagate`)
- GTest 로 §4.4 테스트 6종 이식 (참조: `test_l0.cpp`)
- 동작·`noexcept`·무할당은 그대로 유지 (fixture 라 RT 게이트는 적용하지 않지만 바꿀 이유가 없다)

**`truncated` 계약.** v0.1은 반환값(스텝 수)만으로 상한 도달을 감지하게 했는데, $T=0.2$, `h_max=2 ms`, `max_steps=100` 이면 정확히 $n=100$ 이 나오면서도 포화가 아니다. v0.2는 이를 구조체 필드로 옮겼다. 참고로 이 문제에서 RK4의 정확도는 과분하다 — $T=0.5$ 를 $h=5$ ms로 잘라 적분해도 미세 스텝 대비 위치 오차가 $3.4\times10^{-12}$ m다. `truncated`는 정확도보다 **가정 위반 감지**(너무 오래된 입력)를 위한 신호다.

### 5.2 공용 타입

v0.4 의 `types.hpp` 스케치를 대체한다. 헤더 이름·배치는 S1.1 골격에서 정한다.

- **시간 타입** `BallTime`, `NowReal`, `NowLead` (§4.5, S1.3). 각각 `int64` ns 하나를 감싼 trivially copyable 타입
- **용량 상수** `kMaxArmDof = 7`, `kMaxHandDof = 16`, `kMaxFingertips = 4`. 궤적 용량 컴파일타임 상수 `kCap` 은 **공용 궤적 타입(S1.2, L2 §5.1)이 단독 소유**한다 — v0.4 의 `kMaxTrajSamples` 중복 상수와 `static_assert` 짝맞춤은 삭제. 값은 S0.7 vision 지평 손계산 제안값으로 정해 provisional 로 두고, 런타임 상한 `n_max ≤ kCap` 은 S3.6 이 정한다. `n_max` 가 `kCap` 을 넘으면 `kCap` 을 올리고 S1 게이트를 재실행한다(backfill, plan §4.2)
- **SeqLock payload 규칙 `[확정]`.** `rtc::SeqLock`·`rtc::SpscQueue` 에 싣는 모든 타입(궤적 스냅샷, `PlanSnapshot`, RT 상태 POD)은 trivially copyable POD 다 — 벡터는 `std::array<double, 3>` 등으로, **Eigen 멤버 금지** (G0-1, plan §6). 계산 측은 `Eigen::Map` 으로 본다. 각 타입 정의에 `static_assert(std::is_trivially_copyable_v<…>)` 를 둔다
- **공분산 버퍼** — RT 스냅샷에 넣지 않고 계획기 쪽 버퍼에만 둔다 `[확정 A-3]`. NaN 원소는 "모름"이며 해석은 계획기 한 곳에서 한다. 파서 → 계획기 전달 수단은 S5.2/S6 에서 확정한다(SeqLock 을 쓰면 위 POD 규칙 적용)
- **트랙 식별** — v0.4 의 `TrackEpoch` 는 삭제. vision 의 `generation`(uint64)을 궤적 스냅샷에 그대로 싣는다(D-4, L1 §4.4)

`PlanSnapshot`은 L3, 로봇 상태 스냅샷은 L1 §5.4, 공용 궤적 타입은 L2 §5.1 (S1.2) 에서 정의한다. 궤적 타입을 L1 이 아닌 공용 타입으로 두어 L1 → L2 의존 역전을 없앤다(S0.3).

### 5.3 파라미터 검증기 (S1.7)

- **로딩 경로.** 컨트롤러 `LoadConfig(YAML)` → 층별 `ParseXxxParams` 가 구조체를 채우고(non-RT, `on_configure`), 검증기는 그 구조체를 받는다. runtime 변경 가능한 gain 만 `declare_parameter` 로 연다(G0-3). RT tick 에서는 검증하지 않는다.
- **활성 구성 TBD 검사.** 문자열 `"TBD"` 또는 NaN 이 남은 필드 중 **현재 활성 구성이 참조하는 키만** 실패로 센다(sim/실기, `lead_enable` 등에 따라 검사 집합 결정 — 마스터 §6).
- **범위 검사.** 각 필드 `(name, value, lo, hi, unit)` 표(층별 §6).
- **교차제약.** 마스터 §6 표를 구현한다. v0.5 변경: `v_tcp_max = η_v · reference.v_max` ($0<\eta_v\le1$) `[확정 D-9]` 가 "같은 값" 행을 대체하고, 이름이 둘이던 값 5쌍(`n_min`, `derate_step`, `ed_jump_max`, `a_dec`, ramp)은 단일 키가 되어 일치 검사 대상에서 빠진다(plan S0.3). γ derate 키는 v1 범위 밖이다(D-8).
- **ζ·ω·h 검사 (`dt` 기준).** $h$ = configure 시 `control_rate`(100–5000 Hz)로 정해지는 `ControllerState::dt`. $s=\omega h$ 가 이산 안정 경계 $2\sqrt2-2\approx0.828$ 이상이면 `armable=false`, 정확도 권장 $s\le0.05$ 초과면 경고(L4 §4.7). 500 Hz 고정 가정은 쓰지 않는다. `reference.zeta` ≠ 1 이면 `armable=false` — 계획기의 종단 오차 닫힌해가 $\zeta=1$ 에서만 유효하다(L4 §4.4 임계감쇠 닫힌해).
- **provisional 처리.** D-12 사용자 값(공 사양 등), D-17 catch frame(`provisional: true`), D-18 `planner.catchability.manipulability_min` 처럼 YAML 에 provisional 표시된 값은 sim 구성에서는 경고와 함께 허용하고, **실기 구성에서는 `armable=false`** 로 arm 을 막는다(plan §7.1 D-12, §10).
- **결과.** `ValidationReport{bool armable; 고정 용량 실패 키 목록; 경고 목록}`. `armable=false`면 L7이 `ARMED` 진입을 거부한다.

### 5.4 메시지 — 없음

v0.3에서 자체 메시지 패키지를 폐기했다. 입력은 vision의 `sensor_msgs/PointCloud2`(마스터 §5.1)이고, 파싱은 L1 §5.1이 맡는다(`integrated_bringup` 바인딩, D-1). 발사 srv(D-14)는 sim 용이며 S3.2 에서 `rtc_msgs` 에 추가한다. 상태 발행은 `PublishRole` 없이 한다(S5.4).

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `core.ball.diameter` | double | m | `TBD` | 0.02–0.3 | TBD-BALL-01, D-12 (provisional). L3 $d_{eff}$·L7 충격량에 쓰임 |
| `core.ball.mass` | double | kg | `TBD` | 0.005–1.0 | TBD-BALL-01, D-12 (provisional). L7 §4.7 충격량 |
| `core.ball.restitution` | double | – | `TBD` | 0–1 | TBD-BALL-01, D-12 (provisional). L3 §4.5 $d(1+1/e)$ |
| `sim.ball.gravity` | double[3] | m/s² | `[0, 0, -9.81]` | 크기 9.7–9.9 | fixture 전용 |
| `sim.ball.drag_k` | double | 1/m | `TBD` | 0–0.2 | fixture 전용, §7 식별값 |
| `sim.ball.v_eps` | double | m/s | 1e-3 | 1e-6–1e-1 | 특이점 방어 (fixture) |
| `sim.integrator.h_max` | double | s | 0.002 | 1e-4–0.01 | fixture 적분 스텝 |
| `sim.integrator.max_steps` | int | – | 100 | 1–500 | fixture 연산량 상한 |

공 물성(`core.ball.*`)은 제어 경로에도 필요하므로 남기고, 운동 모델 파라미터는 `sim.*`로 옮겼다. sim 쪽 공 물성(tennis preset)과 `core.ball.*` 가 같아야 하는지는 D-12 값이 정해질 때 함께 본다.

## 7. 단위 기술 구현 순서

- **S1.3** 시간 타입 + 오버로드 비교 + $T_{arm}\ne0$ fixture 테스트.
- **S1.2** (L2 와 공동) 공용 궤적 타입 POD 화, 용량 상수.
- **S1.7** 파라미터 검증기 + 활성 구성 TBD·provisional·교차제약·ζ·ω·h 테스트.
- **S1.6** `ball_dynamics.hpp` 를 test fixture 로 이식 + §4.4 테스트 6종 (참조: `test_l0.cpp`, GTest). **fixture 전용이므로 RT 게이트는 적용하지 않는다.**
- **S3.8** 시뮬레이션 $k$ 식별 도구: MuJoCo에서 서로 다른 초기속도로 공을 던져 참값 궤적(`/sim/ball/ground_truth`)을 기록하고, $k$를 스칼라 최소제곱으로 추정. 산출값을 `sim.ball.drag_k`에 기록. fixture 가 실제로 필요해질 때까지 미뤄도 된다.

## 8. 디버깅 방법

- 해석해 대조 실패: `g` 부호·축 방향, 단위(deg/rad 혼동 아님을 확인), RK4 계수 오타 순으로 확인.
- STM 불일치: Jacobian의 $vv^\top/\Vert v\Vert$ 항 누락, $\partial a/\partial k$ 부호, 중간 단계 $\Phi$ 갱신식 확인.
- $k$ 식별 잔차가 큰 경우: sim 공 모델의 Magnus 항(G0-5)이 원인인지 먼저 본다 — 스핀 0 발사로 재식별해 비교한다. 잔차 크기를 기록하고 사용자에게 보고한다(모델 확장은 사용자 결정).
- 검증기가 sim 에서 통과하는데 실기에서 막힘: provisional 표시가 남은 키(§5.3)를 먼저 본다.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G0-A | §4.4 여섯 테스트 통과 (해석해 오차 < 1e-9 m, 오차비 12–20, STM 차이 < 1e-6, 저속 ∂A/∂k 상대오차 < 1e-12, `truncated` 계약) | `[SIM-ANY]` |
| G0-B | 모든 함수 `noexcept`, 시간 타입·검증기 외 RT 사용 경로 할당 0 (`ScopedNoMalloc`·`ScopedAllocGate`), SeqLock payload 타입 `static_assert` trivially copyable | `[SIM-ANY]` |
| G0-C | 활성 구성의 TBD 필드가 있는 YAML에서 `armable=false`, 비활성 구성 키의 TBD 는 통과. `robot.hand.q_close != q_pre`(L6 §4.2) 검사 포함. D-9 교차제약, $\omega h\ge0.828$(100·500·5000 Hz 각각), $\zeta\ne1$, 실기 구성의 provisional 값 → `armable=false` | `[SIM-ANY]` |
| G0-D | 시뮬레이션 $k$ 식별 후 1 s 궤적 위치 RMS 잔차 기록 (합격 임계는 사용자 결정) | `[SIM-P1B]` |
| G0-E | 시간 타입: 다른 타입끼리 비교가 컴파일되지 않음, $T_{arm}\ne0$ fixture 에서 plan §3 표의 판정별 비교 대상 고정 | `[SIM-ANY]` |

## 10. 미확정 항목

TBD-BALL-01 (D-12 값 대기), `kCap` 제안값 (S0.7 → S1.2), 런타임 `n_max ≤ kCap` (S3.6), 공분산 버퍼 전달 수단 (S5.2/S6), fixture 사용 여부 (S3.5a). TBD-SIM-02·TBD-WS-02·TBD-RTC-01~03 은 닫힘 (§2).
