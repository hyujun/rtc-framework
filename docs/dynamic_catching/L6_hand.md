# L6 — Hand: 손 시퀀서, 명령 포트 추상화, `T_close` 식별

- 브랜치: `feat/catching-L6-hand`
- 패키지: `catching_hand`
- 선행: 단계 W, L0, L1
- 비고: 손 명령 경로와 지문 센서 경로는 workspace의 기존 구조를 따른다(마스터 §1.2, W4-5·W4-6). 본 layer는 **시각 제어와 프로파일**을 맡는다.
- 산출물: `hand_profile.hpp`, `hand_command_port.hpp` (in-loop / async 구현), `hand_sequencer.hpp`, `tools/identify_hand_close.py`

---

## 1. 범위 / 비범위

범위:
- preshape → 폐쇄 → 유지 → 해제의 시각 제어
- 손별 자세·한계를 YAML 프로파일로 분리
- RT 루프에서 손 명령을 내보내는 두 경로:
  - 시뮬레이션: 같은 RT 루프의 command interface
  - 실기 P1b: 별도 드라이브 노드로 비동기 전달
- 폐쇄 시간 $T_{close}$와 전달 지연 $T_{link}$ 식별

비범위:
- 파지력 제어(grasp matrix 기반 내력 제어는 기존 WBC 과제)
- 접촉 판정(L7)
- 폐쇄 체인 기구학 자체(기존 `rtc_urdf_bridge` 재사용)

## 2. 코드 확인 게이트

| ID | 확인 항목 | 기록 |
|---|---|---|
| G6-1 | P1b 구동 좌표 정의(10 actuated DoF: thumb 4, index 3, middle 2, ring 1)와 URDF/MJCF 일치 | TBD-HAND-05 |
| G6-2 | P1b 드라이브 노드의 명령 메시지 타입, 모드(위치/전류/토크), 주기, 수신 스탬프 제공 여부 | TBD-HAND-02 |
| G6-3 | P1b 명령 가능한 전류·토크 한계 (관절 모터 1.5 Nm 기준) | TBD-HAND-05 |
| G6-4 | 시뮬레이션 두 손(MJCF)의 actuator 종류·게인, 폐쇄 체인 `<equality><connect>` 구성 | TBD-SIM-01 |
| G6-5 | 지문 센서의 frame·부호 규약 (접촉력이 +인지 −인지) | TBD-HAND-03 |
| G6-6 | 기존 WBC의 손 명령 경로(current feedforward 등)와 충돌 여부 | TBD-RTC-17 |

## 3. 참고자료

[R1] caging 전략(넓은 preshape, 한쪽 개구부 진입, 엄지 차단)과 폐쇄 시간 요구의 근거, [R14] MuJoCo actuator.

## 4. 수학적 이론

### 4.1 폐쇄 시간 요구 (L3 §4.5 재게)

$$\gamma\ge1-\frac{d_{eff}}{\Vert v\Vert\,T_{close,tot}},\qquad T_{close,tot}=T_{close}+T_{link}+T_{tick}$$

손 성능은 $T_{close,tot}$ 하나로 요약되어 계획에 들어간다. 따라서 이 값의 **정의와 측정 절차**가 L6의 핵심 산출물이다.

**이 값이 시스템 전체의 실현 가능성을 결정한다.** 받을 수 있는 최대 공 속력은

$$\Vert v\Vert_{\max}=\min(v_{dir,\max},v_{\max})+\frac{d_{eff}}{T_{close,tot}}$$

$v_{dir,\max}=1.5$ m/s, $d_{eff}=4$ cm, $T_{close,tot}=60$ ms면 2.17 m/s뿐이다. 6 m/s를 받으려면 $T_{close,tot}\le8.9$ ms가 필요하고, 이는 [R1]의 DLR-Hand-II(5 ms 급)와 같은 수준이다.

그래서 **L6.1–L6.3(가능하면 L6.5까지)을 L3보다 먼저 한다**(마스터 §4.1). 이 값을 모른 채 `planner.gamma.*`, `reference.a_max`, rollout 창을 튜닝하면 재작업이 확정이다. 자릿수가 예산을 넘으면 목표 투척 속도(`TBD-BALL-02`)부터 낮춰야 한다.

### 4.2 $T_{close}$의 정의 `[권장]`

- 명령 시각 $t_{cmd}$: RT 루프가 폐쇄 명령을 기록한 틱 시각.
- 폐쇄 진행률(구동 좌표 기준):

$$\rho(t)=\min_{i\in\mathcal C}\frac{(q_i(t)-q_i^{pre})\,s_i}{|q_i^{cls}-q_i^{pre}|},\qquad s_i=\mathrm{sign}(q_i^{cls}-q_i^{pre})$$

  $\mathcal C$는 caging에 필요한 관절 집합(YAML)이다. **$\mathcal C$의 모든 관절은 $|q_i^{cls}-q_i^{pre}|>\epsilon_\rho$ 를 만족해야 한다** — 그렇지 않으면 0으로 나누고 $s_i$도 정의되지 않는다. L0 파라미터 검증기가 강제한다(`armable=false`).
- $T_{close}(\eta)=\inf\{t-t_{cmd}:\rho(t)\ge\eta\}$. $\eta$는 공이 빠져나갈 수 없는 진행률이며, 손 형상에서 정한다(YAML).

$T_{link}$는 실기에서 RT 명령 기록 시각과 드라이브 노드 수신 시각의 차이다. 드라이브 노드가 수신 스탬프를 주지 않으면 $T_{link}$를 따로 분리하지 않고 $T_{close,tot}$를 종단 간(RT 기록 → 인코더 $\rho\ge\eta$)으로 측정한다. 두 시각이 같은 시계여야 한다.

### 4.3 명령 시각의 양자화

RT 틱 $h$ 단위로만 명령할 수 있으므로 $t_{cmd}$를 넘지 않는 마지막 틱이 아니라 **처음으로 $t\ge t_{cmd}-h/2$인 틱**에서 명령한다(가장 가까운 틱으로 반올림). 오차는 $\pm h/2$의 영평균이다.

$T_{tick}=h/2$는 그 오차의 **worst case를 γ 창 예산에 넣는 값**이지, 명령 시각을 당기는 값이 아니다. L3 §4.11이 $t_{cmd}=t_c-T_{close}-T_{link}$로 $T_{tick}$을 빼지 않는 것과 일관된다.

### 4.4 폐쇄 명령 형태

$T_{close}$를 최소화하려면 폐쇄 자세로의 계단 명령 + actuator 속도 한계가 기본이다. 폐쇄 후 파지력은 전류(토크) 한계로 제한한다. 공과 손가락에 과도한 힘이 가해지는 것을 막기 위함이다.

폐쇄 체인 P1b는 구동 좌표에서 명령한다. 수동 관절은 폐쇄 제약으로 결정된다(기존 `rtc_urdf_bridge`, Pinocchio 4.0 `RigidConstraintModel`).

### 4.5 포켓 유효 깊이 $d_{eff}$ 산정 `[권장]`

1. 기하 추정: preshape 자세에서 FK로 손바닥 평면과 폐쇄 시 손가락이 형성하는 차단선 사이 거리를 접근축 방향으로 측정한다. 공 반지름을 뺀다.
2. 실험 보정: 시뮬레이션과 실기에서 저속 투척으로 "폐쇄 늦음" 경계를 찾는다.
3. 반발 허용 여부(L3 §4.5)에 따라 $d$ 또는 $d(1+1/e)$를 쓴다.

## 5. C++ 구현

### 5.1 프로파일

```cpp
inline constexpr int kMaxHandDof = 16;  // L0 §5의 정의와 동일해야 한다. 이식 시 L0 헤더를 include 하고 이 줄은 지운다.
using HandVec = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxHandDof, 1>;

struct HandProfile {                 // YAML에서 로드 (손별)
  int n{0};
  HandVec q_open, q_pre, q_close;    // 구동 좌표
  HandVec effort_limit_hold;         // 폐쇄 후 전류/토크 한계 (단위는 G6-2 결과)
  std::array<bool, kMaxHandDof> caging_mask{};
  double eta_close{0.9};
  double T_close{0.0}, T_link{0.0};  // 식별값 [s]
  double T_pre{0.3};                 // preshape 시작 선행시간 [s]
  double T_hold{1.0};                // 유지 시간 [s]
  double T_close_timeout{0.3};       // 폐쇄 완료 대기 상한 [s]
};
```

### 5.2 명령 포트 (RT에서 호출)

```cpp
struct HandCommand {
  Nanoseconds t{0};
  std::uint8_t mode{0};              // OPEN, PRESHAPE, CLOSE, HOLD, RELEASE
  HandVec q_target;
  HandVec effort_limit;
  std::uint32_t seq{0};
};

class HandCommandPort {              // 가상 함수 호출은 RT에서 허용 (할당 없음)
 public:
  virtual ~HandCommandPort() = default;
  virtual void send(const HandCommand& c) noexcept = 0;
};

// 시뮬레이션: 같은 RT 루프의 command interface에 직접 기록
class InLoopHandPort final : public HandCommandPort { /* command interface 핸들 보유 */ };

// 실기 P1b: RT → SPSC → non-RT 발행 스레드 → 드라이브 노드
class AsyncHandPort final : public HandCommandPort {
 public:
  void send(const HandCommand& c) noexcept override { (void)ring_.tryPush(c); /* 실패 시 카운터 */ }
  // non-RT: ring_에서 꺼내 메시지로 변환·발행 (G6-2 타입)
};
```

규칙:
- RT는 발행하지 않는다.
- `AsyncHandPort`는 명령 기록 시각 `c.t`를 메시지에 실어 보내, 드라이브 쪽 수신 시각과 비교할 수 있게 한다.

### 5.3 시퀀서

```cpp
class HandSequencer {
 public:
  enum class Phase : std::uint8_t { Open, Preshape, Close, Hold, Release };
  void arm(const PlanTiming& pt) noexcept;    // t_c, t_cmd (L3), 동결 전 갱신 허용
  void abort(bool open) noexcept;             // L7
  [[nodiscard]] Phase update(Nanoseconds now, double h, const HandVec& q_meas,
                             HandCommandPort& port) noexcept;
  [[nodiscard]] double closeProgress() const noexcept;   // ρ(t)
};
```

전이 규칙:
- `Open → Preshape`: $t\ge t_c-T_{pre}$
- `Preshape → Close`: §4.3 규칙의 틱
- `Close → Hold`: $\rho\ge\eta$ 또는 `T_close_timeout` 경과(타임아웃이면 플래그)
- `Hold → Release`: L7 지시

## 6. YAML 파라미터 (손별 `robot.hand.*`)

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `robot.hand.name` | string | – | `p1b` / `leap` | – | 로봇 구성 |
| `robot.hand.joint_names` | string[] | – | `TBD` | – | G6-1 |
| `robot.hand.q_open`, `q_pre`, `q_close` | double[n] | rad | `TBD` | 관절 한계 내 | TBD-HAND-05 |
| `robot.hand.caging_mask` | bool[n] | – | `TBD` | – | 손 형상 |
| `robot.hand.eta_close` | double | – | `TBD` | 0.5–1 | §4.2 |
| `robot.hand.rho_eps` | double | rad | 0.02 | >0 | §4.2 $|q^{cls}_i-q^{pre}_i|$ 하한 (L0 검증기가 강제) |
| `robot.hand.effort_limit_hold` | double[n] | G6-2 | `TBD` | ≤ 모터 한계 | G6-3 |
| `robot.hand.T_close` | double | s | `TBD` | ≥0 | §4.2 실측 |
| `robot.hand.T_link` | double | s | `TBD` (sim: 0) | ≥0 | §4.2 실측 |
| `robot.hand.T_pre` | double | s | 0.3 | 0–1 | 튜닝 |
| `robot.hand.T_hold` | double | s | 1.0 | 0–5 | 튜닝 |
| `robot.hand.T_close_timeout` | double | s | `TBD` | > `T_close` | 실측 후 |
| `robot.hand.port` | enum | – | `in_loop`(sim) / `async`(hw) | – | 구동 경로 |
| `robot.hand.async.topic` | string | – | `TBD` | – | TBD-HAND-02 |
| `robot.hand.async.ring_capacity` | int | – | 64 | 8–1024 | – |

## 7. 단위 기술 구현 순서

**L6a (L3보다 먼저, 마스터 §4.1)**

- **L6.1** G6 게이트, 손 프로파일 YAML 두 벌(P1b, LEAP) 초안.
- **L6.2** `InLoopHandPort` + 시퀀서 + 시뮬레이션 시각 테스트.
- **L6.3** `tools/identify_hand_close.py`: 계단 명령 반복(≥20회), $\rho(t)$와 $T_{close}(\eta)$ 분포(평균, 최대, 99%) 산출. **산출값으로 §4.1의 $\Vert v\Vert_{\max}$를 계산해 `TBD-BALL-02`(목표 투척 속도)와 대조하고, 초과하면 사용자에게 보고한다.**
- **L6.5** 실기 $T_{close,tot}$ 종단 간 식별 `[HW-P1B]`. 실기 접근 가능하면 L6a에 포함한다.

번호가 L6a에서 L6.1→L6.2→L6.3→L6.5로 건너뛰는 것은 의도적이다. 번호는 **의존 순서가 아니라 작업 항목 식별자**이고, L6.4(`AsyncHandPort`)는 TBD-HAND-02 확정을 기다려야 해서 L6b로 내려갔다. L6.5는 그 확정과 무관하게 실기만 있으면 할 수 있다.

**L6b (L7 직전)**

- **L6.4** `AsyncHandPort` + 드라이브 노드 연동(TBD-HAND-02 확정 후).
- **L6.6** 시뮬레이션 손 actuator를 실측 계단 응답에 맞추는 보정(선택) `[SIM-P1B]`.

## 8. 디버깅 방법

- 기록: 위상 전이 시각, 명령 seq, $t_{cmd}$ 목표와 실제 틱 차이, $\rho(t)$, 타임아웃 플래그, async 링 드롭 수.
- 폐쇄가 늦다: 명령 시각 오차(양자화 규칙) → $T_{link}$ 분포 → actuator 게인·속도 한계 순으로 확인.
- 폐쇄 후 손가락이 튄다: 파지 한계 전환 시점과 값을 확인.
- 실기와 시뮬레이션 $T_{close}$ 차이가 크다: L6.6 보정 전에는 시뮬레이션 성공률을 `[SIM-P1B]` 한정 결과로 표기한다.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G6-A | 시퀀서 명령 시각 오차 ≤ $h/2$ (1000회) | `[SIM-ANY]` |
| G6-B | RT 경로 할당 0, async 링 드롭 0 (정상 부하) | `[SIM-ANY]` |
| G6-C | 시뮬레이션 두 손의 $T_{close}(\eta)$ 분포 산출 | `[SIM-P1B]` |
| G6-D | 실기 P1b $T_{close,tot}$ 분포 산출, 99% 값을 YAML에 반영 | `[HW-P1B]` |
| G6-E | 고정 공 fixture에서 폐쇄 후 유지 성공 (반복 횟수는 사용자 결정) | `[HW-P1B]` |

## 10. 미확정 항목

TBD-HAND-01~05, TBD-SIM-01, TBD-RTC-17.
