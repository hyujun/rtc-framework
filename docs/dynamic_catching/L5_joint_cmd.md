# L5 — Joint Command: 기존 CLIK/QP 어댑터, 관절 한계, backend에 $q_c$ 싣기

- 브랜치: `feat/catching-L5-joint-cmd`
- 패키지: `catching_joint_cmd` (**얇은 어댑터**) + 기존 kinematics·`rtc_tsid`·joint command backend 재사용
- 선행: 단계 W, L0, L4
- 산출물: `catch_task_adapter.{hpp,cpp}`, (필요 시) `arm_lag_identification` 도구

---

## 1. 범위 / 비범위

**이 layer는 새 제어기가 아니라 어댑터다** (마스터 §1.2). workspace에 이미 있는 것을 그대로 쓴다.

| 이미 있는 것 (재사용) | 본 layer가 하는 것 |
|---|---|
| FK, frame pose, Jacobian, 폐쇄 체인 (W3-1~4) | 그 API를 호출해 과제를 구성 |
| velocity-level CLIK, 과제 클래스, QP solver, 적분 (W3-6~8) | 포구용 과제(병진 + 접근축 + posture)를 그 구조에 등록 |
| joint command backend — 실기·시뮬레이션 공용, **전부 position** (W4) | 계산된 $q_c$ 를 그 자리에 **데이터로 싣기** |

범위:
- L4의 catch frame 기준(병진 + 접근축 각속도)을 기존 CLIK의 과제 형식으로 변환한다.
- 포구에 필요한 관절 위치·속도·가속 한계를 강제한다(기존 backend가 이미 하고 있지 않은 부분만, W4-3).
- 결과 $q_c$ 를 backend가 읽는 자리에 쓴다.

비범위:
- **토크 제어.** 명령은 전부 position으로 확정(마스터 §1.3).
- **명령 경로·드라이버 설정.** backend와 드라이버 소관. 본 구현은 감시만 한다(L7).
- kinematics·dynamics·QP 구현. 이미 있다.
- 손 관절(L6).

## 2. 코드 확인 게이트

단계 W에서 처리한다(`WORKSPACE_ANALYSIS.md` W3, W4). **W4-1이 이 layer의 출력 형식을 결정하므로 가장 먼저 확인한다.**

| ID | 확인 항목 | 기록 |
|---|---|---|
| G5-1 | **명령을 싣는 자리의 정확한 형태**: `command_interface` 핸들인지 자체 버퍼인지, 필드 이름·단위·쓰기 시점 | TBD-ARM-01 (W4-1) |
| G5-2 | backend가 이미 하는 일: 관절 한계 검사, 속도·변화량 제한, 지연 보상이 들어 있는지 | W4-3 |
| G5-3 | velocity-level CLIK 경로의 입력·출력·적분 상태(명령 기반인지 측정 기반인지) | TBD-RTC-09 (W3-6) |
| G5-4 | frame 위치 과제와 각속도 과제의 마스크 지원 방식 (LOCAL 고정축 마스크 가능 여부) | TBD-RTC-10 (W3-7) |
| G5-5 | QP solver 사용 방식: box 제약, warm start, 고정 차원 | TBD-RTC-11 (W3-8) |
| G5-6 | frame Jacobian 함수 이름·`ReferenceFrame` 인자 | TBD-RTC-12 (W3-3) |
| G5-7 | 실기/시뮬레이션 경로 전환 방법, MuJoCo actuator 종류·게인 | TBD-SIM-01 (W4-2, W6-2) |
| G5-8 | 기존 `DemoWbcController`의 알려진 버그(Stage C-0) 수정 여부와 본 경로에 대한 영향 | TBD-RTC-13 (W2-8) |

## 3. 참고자료

[R9] Pinocchio, [R10] ProxQP, [R11] UR ROS 2 드라이버(position/velocity 인터페이스, `servoj` 보간), [R7] 각속도 규약.

## 4. 수학적 이론

### 4.1 속도 수준 CLIK를 쓰는 이유

관절 명령이 전부 position이므로 QP 출력 $\dot q$를 한 번 적분해 $q_c$를 만든다. 가속도 QP 출력을 두 번 적분하면 드리프트와 솔버 잡음 증폭이 생기므로 쓰지 않는다(기존 프로젝트 결론과 동일).

이 구조는 workspace에 이미 있다(W3-6). 본 layer는 그 CLIK에 포구 과제를 등록하고 결과를 backend에 넘길 뿐이다.

### 4.2 과제 정의

CLIK 내부 상태 $q_c$(명령 관절 위치) 기준으로 catch frame 자세 $(x_C,R_{WC})$를 계산한다.

**병진 과제**

$$J_p(q_c)\dot q=v_p^\ast,\qquad v_p^\ast=\dot x_{ref}+K_p\big(x_{ref}-x_C(q_c)\big)$$

$J_p$는 `LOCAL_WORLD_ALIGNED` 기준이다(마스터 §3).

**명령 공간 폐루프임을 명시한다 `[권장]`.** $x_C$는 CLIK 내부 상태 $q_c$의 FK이지 측정 $q$의 FK가 아니다. 따라서 $K_p(x_{ref}-x_C(q_c))$는 **적분 드리프트 보정 항일 뿐 외란 제거 항이 아니다.** 실제 운동은 $\dot x_{ref}$ feedforward가 만든다. 결과로 세 가지가 따라온다.

1. 실기 추종 오차는 이 루프 밖에 있다. 측정 $q$는 L7의 `TRACK_ERR` 감시에만 쓰인다.
2. L3 §4.6의 $\sigma_{trk}$는 **실행 중 관측되지 않는 오프라인 식별값**이다. $T_{arm}$ 모델의 정확도가 그대로 포구 오차로 간다.
3. §4.4의 1차+지연 모델에서 1차 성분이 유의하면 선행 보상만으로 위상이 맞지 않고(§4.5), 그 잔차가 $\sigma_{trk}$의 지배 성분이 된다.

이것은 position 인터페이스 로봇에서 표준적인 선택이며 바꾸자는 뜻이 아니다. 다만 오차 예산을 읽을 때 전제로 알고 있어야 한다.

**접근축 과제 (5-DoF).** catch frame의 LOCAL $z$축이 손바닥 바깥 법선이면(TBD-FRAME-01), roll 제거는 LOCAL 각속도의 $z$ 성분을 빼는 **고정 마스크** $S=\mathrm{diag}(1,1,0)$로 표현된다.

$$S\,J_\omega^{L}(q_c)\dot q=S\,\omega_{ref}^{L},\qquad \omega_{ref}^{L}=R_{WC}^\top\omega_{ref}$$

$\omega_{ref}=K_a\,e_a$ 이고 $e_a=\theta\hat u$ 는 회전벡터 오차다(L4 §4.5). $e_a\perp z$ 이므로 $\omega_{ref}^L$ 의 $z$ 성분이 0이고 $S$가 정보를 버리지 않는다. 고정축 마스크만 지원하는 TSID 구조에 그대로 들어간다(G5-4). 이 성질은 v0.1의 $\sin$ 기반 오차에서도 성립했으므로 **L5 구조 자체는 바뀌지 않는다** — 바뀌는 것은 `axisAlignOmega`가 돌려주는 값의 크기뿐이다($K_a\sin\theta\to K_a\theta$).

$J_\omega^L$은 Pinocchio `LOCAL` 기준이다. 병진 과제와 reference frame이 다르므로 함수 인자를 반드시 확인한다(G5-4, 마스터 §3).

동치 표현(W 기준): $J_a=f'(c)mm^\top+f(c)[a_d]_\times[z]_\times$ 를 $J_\omega^{W}$에 곱한다(L4 §4.5). 마스크 방식이 불가능할 때만 쓰며, $\theta\to\pi$ 에서 발산한다.

**자세(posture) 과제.** 6축에서 5-DoF 과제를 풀면 1자유도가 남는다. 이를 관절 중립 자세 쪽으로 약하게 당긴다.

$$\dot q=K_n(q_n-q_c)$$

### 4.3 QP

$$\min_{\dot q}\ \tfrac12\Vert J_p\dot q-v_p^\ast\Vert^2_{W_p}+\tfrac12\Vert S(J^L_\omega\dot q-\omega^L_{ref})\Vert^2_{W_a}+\tfrac12\Vert\dot q-K_n(q_n-q_c)\Vert^2_{W_n}+\tfrac\lambda2\Vert\dot q\Vert^2+\tfrac\mu2\Vert\dot q-\dot q_{prev}\Vert^2$$

$$\text{s.t.}\quad \ell\le\dot q\le\upsilon$$

$$\ell_i=\max\Big(-\dot q_{\max,i},\ \beta\frac{q_{\min,i}+m_q-q_{c,i}}{\Delta t},\ \dot q_{prev,i}-\ddot q_{\max,i}\Delta t\Big)$$

$$\upsilon_i=\min\Big(\dot q_{\max,i},\ \beta\frac{q_{\max,i}-m_q-q_{c,i}}{\Delta t},\ \dot q_{prev,i}+\ddot q_{\max,i}\Delta t\Big)$$

Hessian은 $H=J_p^\top W_pJ_p+J^{L\top}_\omega SW_aSJ^L_\omega+W_n+(\lambda+\mu)I\succ0$이고($\lambda>0$), 차원 $n$과 제약 수가 고정이다.

**반복 상한을 고정한다 `[권장]`.** 차원만 고정하면 최악 실행시간이 묶이지 않는다 — 특이 자세나 경계 충돌 시 QP 반복이 급증한다. RT 틱에서 유일하게 상한이 없는 항목이므로 `max_iter` 를 박고, 초과하거나 수렴 실패하면 `solver_status != 0` 으로 보고한 뒤 $\dot q^\ast=\beta_{qp}\dot q_{prev}$ ($\beta_{qp}<1$) 로 감쇠 폴백하고 L7 `QP_FAILED` → `ABORT_SAFE` 로 넘긴다. v0.2는 `solver_status` 를 구조체에 선언해 놓고 abort 사유 표에 연결하지 않아, 풀지 못한 해를 그대로 적분하게 돼 있었다.

**경계 충돌 규칙 `[권장]`.** 관절 한계 근처에서 가속 한계 때문에 $\ell_i>\upsilon_i$가 될 수 있다. 이때 **가속 한계를 유지하고** 위치 한계 쪽으로 가장 가까운 값을 쓴다.

$$\ell_i=\upsilon_i=\mathrm{clamp}\Big(\mathrm{proj}_{[p_{lo,i},\,p_{hi,i}]}(\dot q_{prev,i}),\ \dot q_{prev,i}-\ddot q_{\max,i}\Delta t,\ \dot q_{prev,i}+\ddot q_{\max,i}\Delta t\Big)$$

그리고 `bound_conflict` 플래그를 올린다(L7 `ABORT_SAFE` 사유).

v0.1은 반대로 **가속 항을 빼고** 위치 한계를 우선했는데, 그러면 $\dot q^\ast$가 한 틱에 $\pm\ddot q_{\max}\Delta t$를 넘어 점프할 수 있다. position 인터페이스에서는 $q_c$ 기울기의 불연속이고, UR 드라이버나 로봇 제어기가 보호 정지를 걸 수 있다. 어차피 이 상황은 `JOINT_CONFLICT`로 abort 대상이므로, **abort 경로로 안전하게 빠져나가는 것**이 위치 한계를 1틱 더 정확히 지키는 것보다 중요하다.

위치 한계 침범은 `robot.arm.limit_margin`(기본 0.05 rad)이 흡수한다. 마진을 소진할 만큼 오래 충돌이 지속되면 그때는 이미 abort 중이다.

**적분과 출력.** $q_c\leftarrow q_c+\dot q^\ast\Delta t$. **적분 상태의 소유자를 하나로 정한다** — 기존 CLIK이 내부 적분 상태를 갖고 있으면(G5-3) 여기서 또 적분하지 않는다. 두 곳에서 적분하면 상태가 이원화되어 서로 다른 $q_c$ 가 생긴다. 이 $q_c$ 를 backend가 읽는 자리에 쓴다(G5-1). 단위·순서·쓰기 시점은 backend 규약을 따른다.

**중복 방지.** backend가 이미 관절 한계나 변화량 제한을 걸고 있으면(G5-2) 여기서 같은 검사를 두 번 하지 않는다. 두 곳의 임계가 다르면 조용히 클리핑되어 QP 해와 실제 명령이 달라지고, 그 차이가 L7 `TRACK_ERR`로 나타난다. 어느 쪽에서 강제할지 W4-3 결과를 보고 정하고, 문서에 명시한다.

**QP·과제·적분은 기존 구현을 쓴다.** §4.3의 비용·제약은 그 구조에 무엇을 등록할지를 적은 것이지, 새 QP를 짜라는 뜻이 아니다(W3-7, W3-8).

### 4.4 `servoj` 지연 모델과 식별 `[논문 외 설계]`

**먼저 backend가 이미 보상하고 있는지 확인한다(G5-2, W4-2).** 이미 보상한다면 이 절 전체와 §4.5, `joint_cmd.lag.*`, `sim.arm_lag.*`가 불필요하다. 아래는 보상이 없을 때만 수행한다.

`servoj`는 현재 상태와 목표 사이를 보간하므로 명령 대비 실제 관절에 지연이 생긴다([R11]). 두 가지 모델을 둔다.

- 순수 지연: $q(t)\approx q_c(t-T_{arm})$
- 1차 + 지연: $G(s)=e^{-\tau s}/(T_fs+1)$. 포구 대역 주파수 $f$에서의 등가 지연은 $T_{eq}(f)=\big(2\pi f\tau+\arctan(2\pi fT_f)\big)/(2\pi f)$.

식별 절차(관절별):
1. 운용과 **같은 드라이버 파라미터**로 여기(excitation) 궤적을 명령한다. 권장은 multisine(포구 운동 대역 포함)과 실제 포구형 궤적 두 종류다.
2. $\dot q_c$와 $\dot q$의 정규화 상호상관 최대 위치로 $\hat\tau$를 초기 추정한다.
3. 1차 + 지연 모델을 최소제곱으로 맞추고, 포구 대역의 $T_{eq}$를 계산한다.
4. $T_{arm}=\max_iT_{eq,i}$ (보수적)와 관절별 값을 모두 기록한다.

### 4.5 지연 보상: 예측 선행 `[논문 외 설계]`

순수 지연이 지배적이면 명령 궤적을 $T_{arm}$만큼 앞당기면 된다. 방법은 두 가지를 함께 쓴다.

1. L2 궤적 샘플러가 $t_{rel}=10^{-9}(t_{now}-t_{ref})+T_{arm}$ 로 샘플링해 L4에 넘긴다(L2 §4.4의 $T_{lead}=T_{arm}$). vision 예측을 앞당겨 읽는 것이지 제어 PC가 전파하는 것이 아니다.
2. L4의 γ 프로파일과 L3의 $t_c$ 판정도 같은 선행을 적용한다. 즉 명령 경로가 $t_c-T_{arm}$에 포구점에 도달한다.

손 명령 시각 $t_{cmd}$(L3 §4.11)는 **실제 시각** 축이므로 이 선행을 적용하지 않는다.

1차 필터 성분이 크면 선행만으로는 위상이 완전히 맞지 않는다. 시뮬레이션 에뮬레이션(§4.6)으로 잔여 오차를 측정해 판정한다.

### 4.6 시뮬레이션 동등성

MuJoCo position actuator는 `servoj`와 동특성이 다르다. `ur5e_p1b` 시뮬레이션에는 식별된 $G(s)$를 명령 경로에 넣는 에뮬레이션 옵션을 둔다(`sim.arm_lag.*`). `iiwa7_leap` 시뮬레이션도 position 명령 경로를 써서 구조를 맞춘다 `[권장]`.

### 4.7 Sanity check

1. 정지 목표: 과제 오차가 지수적으로 감소하고 roll은 posture 과제로 정해진다.
2. frame Jacobian과 유한차분 FK 일치.
3. 관절 한계 접근 시 위반 0.
4. 알려진 지연을 주입한 시뮬레이션에서 식별값이 주입값을 회복.

## 5. C++ 구현

### 5.1 어댑터 인터페이스

```cpp
namespace catching::joint {

inline constexpr int kMaxArmDof = 7;   // L0 §5의 정의와 동일해야 한다. 이식 시 L0 헤더를 include 하고 이 줄은 지운다.
using VecN = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxArmDof, 1>;   // 고정 최대 크기, 힙 없음
using MatN = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxArmDof, kMaxArmDof>;

struct CatchTaskRef {
  Eigen::Vector3d x_ref, xd_ref;   // W
  Eigen::Vector3d w_ref;           // W, z에 수직
};

struct JointCmdOutput {
  VecN q_cmd, qd_cmd;
  bool bound_active{false};        // 어떤 관절이든 한계 활성
  bool bound_conflict{false};      // ℓ > υ 발생
  double solve_time_s{0.0};
  int solver_status{0};
};

class CatchTaskAdapter {
 public:
  // configure 단계 (non-RT): 기존 kinematics 핸들 취득, 과제 등록, QP 크기 고정, 버퍼 사전 할당.
  // 모델·QP·과제 클래스는 **workspace 것을 참조로 받는다** — 여기서 새로 만들지 않는다 (W3).
  bool configure(const ArmConfig& cfg, KinematicsHandle& kin, ClikHandle& clik);

  // RT: 과제 기준 갱신 → 기존 CLIK/QP 호출 → 적분. q_c 는 output 으로만 돌려주고,
  //     backend 에 쓰는 것은 호출자(L8 컨트롤러)가 W4-1 규약에 맞춰 한다.
  // noexcept 를 실제로 성립시키려면 내부에서 예외를 삼켜야 한다. 이 함수는 기존 QP
  // solver 와 kinematics 를 감싸는데, 둘 다 입력 검증·workspace 재할당에서 던질 수 있다.
  //   try { ... } catch (...) { out.solver_status = kThrew; out.qd_cmd = beta * qd_prev; }
  // 감싸지 않으면 예외가 새는 순간 std::terminate 로 RT 루프가 죽는다 (마스터 §4.2).
  [[nodiscard]] JointCmdOutput update(const CatchTaskRef& ref, double dt) noexcept;

  // 활성화·재무장 시 (L7 §4.8). q_c = q_meas **와 qd_prev = 0** 을 함께 한다 —
  // qd_prev 를 남기면 첫 틱 가속 경계가 직전 시행의 속도 기준이라 bound_conflict 오abort 가 난다.
  void resetState(const VecN& q_meas) noexcept;
};

}  // namespace catching::joint
```

`VecN`/`MatN`은 최대 크기가 고정된 Eigen 타입이라 힙을 쓰지 않는다. UR5e는 $n=6$, iiwa7은 $n=7$로 configure 단계에서 정한다.

### 5.2 경계 계산 (RT)

```cpp
inline void jointVelocityBounds(const VecN& q_c, const VecN& qd_prev, const ArmLimits& L,
                                double dt, VecN& lo, VecN& hi, bool& conflict) noexcept {
  conflict = false;
  for (int i = 0; i < q_c.size(); ++i) {
    const double p_lo = L.beta * (L.q_min[i] + L.margin - q_c[i]) / dt;
    const double p_hi = L.beta * (L.q_max[i] - L.margin - q_c[i]) / dt;
    double p_lo2 = p_lo, p_hi2 = p_hi;
    if (p_lo2 > p_hi2) { p_lo2 = p_hi2 = 0.5 * (p_lo2 + p_hi2); }   // 가동범위 < 2*margin 방어
    const double a_lo = qd_prev[i] - L.qdd_max[i] * dt;   // 가속 한계 (항상 지킨다)
    const double a_hi = qd_prev[i] + L.qdd_max[i] * dt;
    double l = std::max({-L.qd_max[i], p_lo2, a_lo});
    double h = std::min({ L.qd_max[i], p_hi2, a_hi});
    if (l > h) {                                          // 가속 한계 우선 (§4.3)
      const double target = std::clamp(qd_prev[i], p_lo2, p_hi2); // 위치 한계에 가장 가까운 값
      l = h = std::clamp(std::clamp(target, -L.qd_max[i], L.qd_max[i]), a_lo, a_hi);
      conflict = true;                                    // L7 ABORT_SAFE
    }
    lo[i] = l; hi[i] = h;
  }
}
```

### 5.3 기존 구현 재사용 방식 `[권장]`

- 병진 과제: 기존 frame 위치 과제에 `x_ref`, `xd_ref` 입력(G5-1 형식 확인).
- 접근축 과제: 기존 frame 각속도 과제에 LOCAL 마스크 `(1,1,0)`(G5-4), 기준 `R_WC^T w_ref` 입력. `w_ref`는 L4 `axisAlignOmega(z_cmd, a_d, p)`의 반환값이다. 마스크 미지원이면 새 과제 클래스 `ApproachAxisTask`를 `rtc_tsid`에 추가한다. robot-agnostic 규칙상 frame 이름은 YAML로 받는다.
- posture 과제, 정규화, 경계: 기존 구성 재사용.
- 기존 U1 포즈 오차 헬퍼와 충돌하지 않도록, 접근축 과제는 전체 SO(3) 오차를 쓰지 않는다(roll 기준이 없기 때문).

### 5.4 지연 식별 도구 (non-RT, Python)

```python
# tools/identify_arm_lag.py (요지)
def xcorr_delay(qd_cmd, qd_meas, dt, max_lag_s):
    """정규화 상호상관 최대 지연 [s] (관절별)."""
    ...
def fit_fopdt(t, q_cmd, q_meas):
    """G(s) = exp(-tau s)/(T s + 1) 최소제곱 적합 → (tau, T)."""
    ...
def equivalent_delay(tau, T, f):
    return (2*np.pi*f*tau + np.arctan(2*np.pi*f*T)) / (2*np.pi*f)
```

입력: rosbag(명령 `q_c`, 측정 `q`, 공통 스탬프). 출력: 관절별 $(\tau,T_f,T_{eq})$ YAML 조각과 적합 잔차 그래프.

## 6. YAML 파라미터

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `robot.arm.joint_names` | string[] | – | 로봇별 | – | URDF |
| `robot.arm.catch_frame` | string | – | `TBD` | – | TBD-FRAME-01 |
| `robot.arm.q_min`, `q_max` | double[n] | rad | URDF | – | URDF 한계 |
| `robot.arm.qd_max` | double[n] | rad/s | `TBD` | ≤ 데이터시트 | UR5e 데이터시트는 전 관절 ±180°/s(π rad/s). 운용 여유율은 사용자 결정 |
| `robot.arm.qdd_max` | double[n] | rad/s² | `TBD` | >0 | TBD-ARM-02 |
| `robot.arm.limit_margin` | double | rad | 0.05 | 0–0.3 | 튜닝 |
| `robot.arm.limit_beta` | double | – | 0.5 | 0–1 | 한계 접근 감속 |
| `robot.arm.q_nominal` | double[n] | rad | `TBD` | – | posture 과제 |
| `joint_cmd.K_p` | double | 1/s | 20.0 | 1–100 | CLIK 대역 (L4 `k_axis`보다 크게) |
| `joint_cmd.W_p`, `W_a`, `W_n` | double | – | 1.0, 0.5, 1e-3 | >0 | 튜닝 |
| `joint_cmd.lambda`, `mu` | double | – | 1e-4, 1e-3 | >0 | 정규화 |
| `joint_cmd.qp.max_iter` | int | – | `TBD` | >0 | §4.3 반복 상한 (W3-8 확인 후) |
| `joint_cmd.qp.beta_fallback` | double | – | 0.5 | 0–1 | §4.3 실패 시 감쇠 |
| `joint_cmd.qp.n_fail_fault` | int | – | 3 | 1–20 | 연속 실패 시 `FAULT` (L7 §4.1) |
| `joint_cmd.K_n` | double | 1/s | 1.0 | 0–10 | posture |
| `joint_cmd.lag.T_arm` | double | s | `TBD` | ≥0 | §4.4 식별 |
| `joint_cmd.lag.per_joint` | double[n] | s | `TBD` | ≥0 | §4.4 |
| `joint_cmd.lag.lead_enable` | bool | – | false | – | 식별 전에는 false |
| `joint_cmd.track_err_abort` | double | rad | `TBD` | >0 | **`supervisor.track_err_abort`(L7 §6)와 같은 키다.** 판정은 L7이 하고 L5는 참조만 한다 — 값을 두 곳에 두지 않는다 |
| `sim.arm_lag.enable` | bool | – | false | – | §4.6 |
| `sim.arm_lag.tau`, `T_f` | double | s | `TBD` | ≥0 | §4.4 식별값 |

## 7. 단위 기술 구현 순서

- **L5.0** 단계 W의 W3·W4 결과 확인. 특히 **W4-1(명령을 싣는 자리)과 G5-2(backend가 이미 하는 일)**. 이 둘이 아래 항목의 범위를 결정한다.
- **L5.1** 게이트 결과를 마스터 §9에 기록하고, 중복되는 항목(한계 강제, 지연 보상)을 본 문서에서 제거한다.
- **L5.2** 경계 계산 + 충돌 규칙 테스트.
- **L5.3** 어댑터: 병진 과제, 접근축 과제(마스크 또는 `ApproachAxisTask`), posture.
- **L5.4** 정지 목표 수렴 테스트, 유한차분 Jacobian 테스트.
- **L5.5** RT 검사: QP 차원 고정, 할당 0, solve time 분포.
- **L5.6** (backend가 지연을 보상하지 않는 경우에만) 지연 식별 도구 + 시뮬레이션 주입 지연 회복 테스트.
- **L5.7** (동상) 예측 선행 보상(L2 `prediction.lead` 연동) + 에뮬레이션에서 효과 측정.
- **L5.8** 실기 식별 `[HW-P1B]`.
- **L5.9** backend 쓰기 경로 결선과 왕복 확인: 쓴 $q_c$ 가 실제로 그 값으로 실기·시뮬레이션에 나가는지 1:1 대조.

## 8. 디버깅 방법

- 기록: $q_c$, $q$, $\dot q^\ast$, 경계 $(\ell,\upsilon)$, 활성 관절 인덱스, 과제 잔차 $\Vert J_p\dot q-v_p^\ast\Vert$, 축 잔차, solve time, solver status.
- 과제 잔차가 크다: 한계 활성 여부, 특이 자세(조작도 $\sqrt{\det JJ^\top}$ 기록), 가중치 비율 확인.
- 접근축이 roll과 섞인다: 마스크 frame(LOCAL vs WORLD)과 catch frame 축 정의(TBD-FRAME-01) 확인.
- 명령 대비 측정 지연이 식별값과 다르다: 드라이버 파라미터가 식별 때와 같은지, 시뮬레이션이면 에뮬레이션 설정 확인.
- 관절 한계 근처에서 떨림: `limit_beta`, `limit_margin`, 가속 경계와의 충돌 빈도 확인.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G5-A | 정지 목표에서 위치 오차 < 1 mm, 축 오차 < 0.5° 수렴 | `[SIM-ANY]` |
| G5-B | 무작위 기준 1e4 틱에서 속도·가속 한계 위반 0, 위치 한계 위반은 `limit_margin` 이내 (§4.3 충돌 규칙) | `[SIM-ANY]` |
| G5-B2 | 경계 충돌 유도 시나리오: `bound_conflict` 발생, $\vert\dot q^\ast-\dot q_{prev}\vert\le\ddot q_{\max}\Delta t$ 유지, L7이 `ABORT_SAFE`로 전이 | `[SIM-ANY]` |
| G5-C | RT: page fault 0, 할당 0, QP 차원 고정, solve time 99.9% < 예산 (예산은 사용자 결정) | `[SIM-ANY]` |
| G5-C2 | backend 왕복: 어댑터가 쓴 $q_c$ 와 실제 명령이 전 틱에서 일치 (실기·시뮬레이션 각각) | `[SIM-P1B]` / `[HW-P1B]` |
| G5-C3 | QP `max_iter` 고정, 초과 시 `solver_status != 0` + 감쇠 폴백, L7 `QP_FAILED` 전이. `update()` 안에서 예외가 새지 않음(주입 테스트) | `[SIM-ANY]` |
| G5-C4 | `resetState()` 후 첫 틱에 `bound_conflict` 미발생 (재무장 시나리오) | `[SIM-ANY]` |
| G5-D | 주입 지연(예: 30 ms) 식별 오차 < 2 ms | `[SIM-ANY]` |
| G5-E | 에뮬레이션 지연 하에서 선행 보상 전후 $t_c$ 위치 오차 비교 기록 | `[SIM-P1B]` |
| G5-F | 실기 `T_arm` 식별 및 YAML 확정 | `[HW-P1B]` |

## 10. 미확정 항목

TBD-RTC-09~13, TBD-ARM-01(명령을 싣는 자리), TBD-ARM-02, TBD-SIM-01, TBD-FRAME-01, `robot.arm.qd_max` 운용 여유율, `joint_cmd.track_err_abort`. 그리고 **backend가 이미 하는 일의 범위**(W4-3) — 이것이 확정돼야 §4.3의 한계 강제와 §4.4의 지연 식별을 남길지 뺄지 결정된다.
