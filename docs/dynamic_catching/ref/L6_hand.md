# L6 — Hand: 손 시퀀서, 손 프로파일, `T_close` 식별

이 문서는 현재 구현의 **손 계층** — 시각 제어 (시퀀서), 손별 프로파일, 종단 간 폐쇄 시간 $T_{close}$ 의 정의와 식별 절차 — 을 표현한다. 손은 둘이다: **p1b** (10 구동 관절, 폐쇄 체인, 실기 경로 `udp_hand_native`) 와 **leap** (16 구동 관절, 폐쇄 체인 없음, sim 구성). `g1_p1b` 는 p1b 와 같은 손이다 (포구 컨트롤러 config 는 아직 없다 — 그 손의 프로파일은 같은 절차 (§4.2 · §4.5) 로 새로 식별한다).

- 코드 배치: 시퀀서 · 프로파일 순수 로직은 `rtc_controllers` `catching` (`hand_sequencer.hpp`, `catching_params.hpp` 의 `HandProfile`), YAML · 식별 도구 배선은 `integrated_bringup`. 새 패키지를 만들지 않는다
- 손 명령은 포구 컨트롤러의 `ControllerOutput` 손 device slot 에 직접 쓴다. 본 layer 는 **시각 제어와 프로파일**을 맡는다

---

## 1. 범위 / 비범위

범위:
- preshape → 폐쇄 → 유지 → 해제의 시각 제어
- 손별 자세 · 한계를 YAML 프로파일로 분리
- 손 명령을 `ControllerOutput` 의 손 device slot (device 1 관례) 에 position 목표로 기록. 전달은 기존 `DeviceBackend` 가 한다:
  - 시뮬레이션: `mujoco_native` (p1b · leap 모두)
  - 실기 p1b: `udp_hand_native` → `/p1b/joint_command` → 별도 프로세스 `udp_hand_node` (250 Hz, **position 만** — feedforward 무시, 명령 `header.stamp` 미사용)
- 종단 간 폐쇄 시간 $T_{close,tot}$ 식별 ($T_{link}$ 는 분리해서 재지 않는다 — 드라이버가 명령 stamp 를 읽지 않는다)

비범위:
- 파지력 제어 (grasp matrix 기반 내력 제어는 기존 WBC 과제)
- 접촉 판정 (L7). 지문 부호는 sim · 실기 모두 finger-on-object 이고 변환 지점은 `rtc::grasp::PullContactConfig::force_sign` 하나다
- 폐쇄 체인 기구학 자체 (기존 `rtc_urdf_bridge` 재사용)
- 손 명령 포트 추상화 — 두지 않는다 (손은 device slot)

## 2. 코드 확인 게이트

이 절의 확인 사실은 §4 · §5 가 서술한다. 구동 관절 이름은 device config (`devices.<hand>.joint_state_names`) 가 정하고 MJCF actuator 순서와 다를 수 있으므로 **이름으로 대응**한다. 유효 위치 한계는 YAML ∩ URDF 의 교집합이다. 실기 명령은 position 만이므로 토크 한계를 직접 명령할 수 없다 (§4.4).

## 3. 참고자료

[R1] caging 전략 (넓은 preshape, 한쪽 개구부 진입, 엄지 차단) 과 폐쇄 시간 요구의 근거, [R14] MuJoCo actuator.

## 4. 수학적 이론

### 4.1 폐쇄 시간 요구 (L3 §4.5 재게)

$$\gamma\ge1-\frac{d_{eff}}{\Vert v\Vert\,T_{close,tot}},\qquad T_{close,tot}=T_{close,e2e}+T_{tick}$$

$T_{close,e2e}$ 는 RT 가 폐쇄 명령을 기록한 tick 부터 인코더 $\rho\ge\eta$ 까지의 **종단 간 실측값**이다 — 잰 폐쇄 시간 **만** 뜻한다. 폐쇄를 얼마나 일찍 지령하는가는 다른 값 $T_{close,lead}$ (§4.3) 다. 실기 p1b 에서는 backend 발행 → `udp_hand_node` 250 Hz 주기 양자화 → 모터 응답이 모두 이 값에 들어간다. sim 에서는 `mujoco_native` lock-step 이라 전달 지연이 사실상 0 이고 $T_{close,e2e}$ 는 MJCF 게인이 정한다.

손 성능은 $T_{close,tot}$ 하나로 요약되어 계획에 들어간다. 따라서 이 값의 **정의와 측정 절차**가 L6 의 핵심 산출물이다.

**이 값이 시스템 전체의 실현 가능성을 결정한다.** 받을 수 있는 최대 공 속력은

$$\Vert v\Vert_{\max}=\min(v_{dir,\max},v_{\max})+\frac{d_{eff}}{T_{close,tot}}$$

($v_{dir,\max}$ 는 포구 자세의 방향 속도 상한 — L3 §4.5.) 예: $d_{eff}=4$ cm, $T_{close,tot}=60$ ms, $v_{dir,\max}=1.5$ m/s 면 2.17 m/s 뿐이고, 6 m/s 를 받으려면 $T_{close,tot}\le8.9$ ms 가 필요하다 ([R1] 의 DLR-Hand-II 5 ms 급).

그래서 이 값은 목표 투척 속도를 정하기 전의 **go/no-go** 다. 이 값을 모른 채 `planner.search.grid.gamma.*`, `reference.a_max`, rollout 창을 튜닝하면 재작업이 확정이다. 게이트: 목표 투척 속도 (catchability 지도가 정한 범위) 에서 γ 창이 비지 않을 것 — 비면 목표를 낮춘 뒤 진행한다. 토크에서 도출한 보수적 팔 가속 box 가 $v_{dir,\max}$ 를 낮출 수 있으므로 함께 판정한다.

### 4.2 $T_{close}$의 정의 `[권장]`

- 명령 시각 $t_{cmd}$: RT 루프가 폐쇄 명령을 손 device slot 에 기록한 tick 의 steady 시각 (now_real).
- 폐쇄 진행률 (구동 좌표 기준, 손 device 의 측정 관절 위치):

$$\rho(t)=\min_{i\in\mathcal C}\frac{(q_i(t)-q_i^{pre})\,s_i}{|q_i^{cls}-q_i^{pre}|},\qquad s_i=\mathrm{sign}(q_i^{cls}-q_i^{pre})$$

  $\mathcal C$ 는 caging 에 필요한 관절 집합 (YAML `caging_mask`) 이다. **$\mathcal C$ 의 모든 관절은 $|q_i^{cls}-q_i^{pre}|>\epsilon_\rho$ 를 만족해야 한다** (`rho_eps`) — 그렇지 않으면 0 으로 나누고 $s_i$ 도 정의되지 않는다. 파라미터 검증기가 강제한다 (`armable=false`).
- $T_{close,e2e}(\eta)=\inf\{t-t_{cmd}:\rho(t)\ge\eta\}$. $\eta$ 는 공이 빠져나갈 수 없는 진행률이며, **판단이 아니라 실측이다**: 손이 공을 실제로 쥔 시행의 정지 $\rho$ 의 중앙값 바로 아래로 정한다. 빈 허공에서만 완주하는 값 (예: 0.9) 은 도달 불가능한 임계라서, 그 값으로 잰 $T_{close,e2e}$ 는 공을 쥐는 동작의 시간이 아니다.

명령 기록 시각과 인코더 수신 시각은 모두 RT 의 steady 시계로 잰다. 인코더 샘플의 원격 stamp 를 비교에 쓰지 않는다 (L0 §4.5 "메시지 stale · 나이" 행, `header.stamp` 판단 금지). 분포는 계단 응답 반복으로 구한다 (손당 200 회 — 99 % 는 20 회로 말할 수 없다). sim 의 출하값은 200 회 시행의 **평균**이다 (표준편차는 두 손 모두 0.2 ms 미만 — sim 손은 거의 매번 같은 tick 에 닫힌다). 이 값이 명령 시각을 정하던 때에는 보수 쪽 p99 였고, 지금은 명령 시각을 정하지 않으므로 잰 값 그대로다.

$T_{close,e2e}$ 는 **자세 쌍의 성질**이다 (이동하는 관절 집합과 이동량이 바뀐다). 자세를 바꾸면 이월하지 않고 다시 잰다. 병목은 가장 먼 거리를 가야 하는 관절의 이동량과 토크 포화이므로, 자세를 정할 때 $T_{close}$ 를 목적함수에 넣는다. 토크 포화가 지배하는 손 (p1b) 과 관성 · 강성이 지배하는 손 (leap) 은 운용 토크 한계에 대한 민감도가 다르다.

시간 축은 steady 시계로 잰다 — 분석기의 tick 수 × dt 축은 CSV 한 행이 한 tick 일 때만 맞다. sim 은 모든 device 의 state 가 새로 도착했을 때 CM 이 한 번 tick 하고 (`sim_sync_tick_devices`), steady 축과 tick×dt 축은 rtf 1.0 에서 일치해야 한다.

#### 잡히지 않을 때 무엇을 의심하는가

공이 palm 에 **닿았는데** 파지가 성립하지 않으면 원인은 둘 중 하나다 — **손 관절 속도** (곧 $T_{close,e2e}$: 공이 튕겨 나가기 전에 닫히지 못한다) 또는 **`q_pre` / `q_close` 자세** (닫히기는 하는데 그 형상이 공을 가두지 못한다). 접촉 자체가 없으면 그건 손 문제가 아니라 L2/L3 의 조준 문제이므로 이 계층을 먼저 의심하지 않는다. 이 둘이 §4.2 와 §4.5 가 각각 재는 양이고, 그래서 두 값이 §4.1 go/no-go 의 입력이다.

### 4.3 명령 시각의 양자화

시간 비교는 L0 §4.5 규약을 따른다. 손 명령에는 팔 지연 선행 ($T_{arm}$) 을 적용하지 않는다 — 비교 대상은 **now_real** (매 tick steady 실측) 이다.

- Preshape: **시각 조건이 아니다.** 팔이 `wait_pose` 에 도착하면 (또는 재무장으로 `Ready` 에 놓이면) 손은 즉시 `q_pre` 를 지시받는다 — ARMED~COMMITTED 내내 그 상태가 유지된다 (L7 §4.1). `q_open` 은 팔이 homing 중일 때만 쓴다.
- Close: $t_{cmd}=t_c-T_{close,lead}$. 시퀀서가 **동결된** $t_c$ 와 프로파일에서 계산하는 단일 출처다 (계획기의 $t_{cmd}$ 는 기록일 뿐 입력이 아니다). 시퀀서는 COMMITTED 진입에서 이 시각으로 무장한다. `closed_form` 에서는 그것으로 끝이다 — 한 번 정하고 갱신하지 않는다. 팔이 구간 계획기 (`mpc` · `mpc_docking`) 의 구간을 따를 때는 폐쇄가 지령될 때까지 아래 "commit 뒤의 지령 시각" 이 옮긴다.

**손의 시간은 둘이고 서로 다른 일을 한다.**

| 값 | 무엇 | 어디서 읽는가 |
|---|---|---|
| `robot.hand.T_close_e2e` | **잰** 폐쇄 시간 (§4.2) | `T_close_timeout` · `T_release_timeout` 의 유도와 검사, 탐색의 $T_{close,tot}=T_{close,e2e}+h/2$ (γ 창 · 최대 포획 속력 · $d_{eff}$ 의 유도, L3 §4.5), docking 코어의 $\delta_0$ |
| `robot.hand.T_close_lead` | 공이 포구점 (catch frame 의 원점) 에 닿기 **얼마 전에** 폐쇄를 지령하는가 — 손의 **설계값**. 식별한 유지 창 (그 축에서 명령 → 원점 도달) 의 가운데로 정한다 | 시퀀서의 $t_{cmd}$, 계획기의 $t_{cmd}$ 기록, $T_{freeze}$ 하한 (L3 §4.11), 탐색의 commit 게이트 |

두 값이 같을 이유는 없다. 키가 없으면 $T_{close,lead}=T_{close,e2e}$ 이고 컨트롤러가 configure 에서 WARN 한다. 유지 창이 어디에 있는가는 손마다 다르다 — 한 손은 막 닫혔을 때 공이 도착하는 것이 맞고 (명령 → 원점 도달이 $T_{close,e2e}$ 근처), 다른 손은 아직 닫히는 중에 공이 도착해야 맞다 (§4.6).

**$t_c$ 의 축.** 시퀀서가 lead 를 빼는 $t_c$ 가 팔의 운동에서 어느 순간인가는 **팔이 따르는 구간을 만든 계획기**가 정한다 — plan 을 낸 탐색이 아니다. `closed_form` 과 `mpc` 구간 계획기는 $t_c$ 에 catch frame 의 원점을 공 위에 놓으므로 (`grid` 든 `nlp` 든) `T_close_lead` 를 적힌 그대로 쓴다. `mpc_docking` 구간 계획기는 $t_c$ 를 공이 **입구 평면** (원점 앞 `robot.hand.docking.s_ent`) 을 지나는 순간에 두므로 컨트롤러가 configure 에서 한 번 환산한다.
$$T_{close,lead}^{(t_c)}=T_{close,lead}-\frac{s_{ent}}{c_{ref}},\qquad c_{ref}=-\nu_{ref,z}$$
($c_{ref}$ 는 그 코어의 기준 접근 속력 `core.catch.nu_ref`). 시퀀서 · RT 의 $t_{cmd}$ (접촉 판정 창) · 탐색이 plan 에 적는 $t_{cmd}$ 가 이 값 하나를 쓴다 (mirror `hand.T_close_lead`, `hand.T_close_lead_from_t_c`). **docking 코어는 따로다**: `nlp` 탐색과 `mpc_docking` 의 코어는 포구 노드가 언제나 입구 평면 통과이므로, 어느 구간 계획기 아래에서든 자기 기준 속력으로 환산한 lead 로 명목 폐쇄 순간 $\delta_0=T_{close,e2e}-(T_{close,lead}-s_{ent}/c_{ref})$ 을 받는다 (mirror `hand.docking.closure.delta_0`). 환산한 lead 가 음수면 (명령이 입구 통과 뒤가 된다) park 한다 (`kMpcDockingInvalid`). `nlp` × `mpc` 는 그래서 탐색의 모형 (입구 통과가 $t_c$) 과 실행 (원점 도달이 $t_c$) 이 $s_{ent}/c_{ref}$ 만큼 다른 조합이다 — 탐색의 해를 그대로 실행하는 것은 `nlp` × `mpc_docking` 뿐이다.

**commit 뒤의 지령 시각 (구간 계획기).** 동결은 $t_c$ 를 묶지만 공의 예측은 그 뒤에도 갱신된다. 팔이 구간을 따르는 mode 에서는 `COMMITTED` 동안, 폐쇄가 지령되기 전까지, 공이 **lead 를 센 그 평면** 을 지나는 시각 $\hat t_x$ 를 다시 풀어 지령 시각을 옮긴다 (formulation ball_catching_inverse_dynamics_mpc.md §12.7 의 M2).
$$t_{cmd}=\hat t_x-T_{close,lead}^{(t_c)}$$
- **어느 평면인가.** 위 "$t_c$ 의 축" 과 하나의 규칙이다 — 그 계획기의 $t_c$ 가 공을 두는 곳. `mpc` 는 catch frame 의 원점을 지나는 평면 ($s=0$), `mpc_docking` 은 입구 평면 ($s=s_{ent}$) 이고, 둘 다 catch frame 의 $z$ 축에 수직이다. 평면과 lead 는 한 쌍이다: 입구 축으로 환산한 lead 를 쓰는 구성만 입구 평면을 쓰고, 그 밖은 원점 축의 lead 와 $s=0$ 이다.
- **무엇으로 푸는가.** 손은 RT 가 들고 있는 구간 (그 시각에 팔이 있을 구간) 위에, 공은 동결된 track 의 가장 새 예측 위에 놓는다. 식 · 풀이 · 창은 formulation §17.16 이 갖는다.
- **언제 푸는가.** `COMMITTED` 의 tick 가운데 입력이 바뀐 tick 에만 — 동결된 track 의 새 예측이 들어왔거나, 손을 읽는 구간이 바뀐 tick: 따르는 구간이 바뀌었거나, 대기 슬롯에 구간이 들어오거나 바뀌었거나, 대기 구간이 따라지지 못하고 버려진 tick (전환 게이트의 거부 · plan 불일치 · 샘플 실패). 풀이는 대기 구간을 그 node 0 부터 읽으므로 대기 슬롯의 변화도 입력의 변화다.
- **실패한 풀이.** 통과 시각을 얻지 못하면 (그 시각의 공이 예측 밖이거나 손이 구간 밖, 동결된 $t_c$ 에서 포구 전 간격 하나보다 먼 통과, 수렴하지 않음) 지령 시각은 **그대로다** — 처음에는 $t_c-T_{close,lead}^{(t_c)}$, 한 번 옮긴 뒤에는 마지막으로 얻은 값.
- **지나간 시각.** 옮긴 $t_{cmd}$ 가 이미 지났으면 그 tick 에 폐쇄를 지령한다. 지령한 뒤에는 옮기지 않는다 (`HandSequencer::Retime` 은 commit 과 폐쇄 지령 사이에서만 듣는다).
- **함께 움직이는 것 · 움직이지 않는 것.** RT 가 가진 $t_{cmd}$ (접촉 판정 창의 시작, L7 §4.4) 가 같은 값으로 옮겨진다. `DECEL` 진입 ($now_{lead}\ge t_c$) 과 판정 창의 끝은 동결된 $t_c$ 그대로다 — 팔의 구간 격자가 그 $t_c$ 에 닻 내려 있다.

RT 틱 $h$ 단위로만 명령할 수 있으므로 $t_{cmd}$ 를 넘지 않는 마지막 틱이 아니라 **처음으로 now_real ≥ $t_{cmd}-h/2$ 인 틱**에서 명령한다 (가장 가까운 틱으로 반올림, `HandCommandDueRounded` — L7 §4.1 R-CLOSE). 오차는 $\pm h/2$ 의 영평균이다. $h$ 는 `ControllerState::dt` (= 1/`control_rate`, 500 Hz 고정이 아니다) 이고, G6-A 는 실측 tick 간격으로 판정한다.

$T_{tick}=h/2$ 는 그 오차의 **worst case 를 γ 창 예산에 넣는 값**이지, 명령 시각을 당기는 값이 아니다. $t_{cmd}$ 식에서 $T_{tick}$ 을 빼지 않는 것과 일관된다.

### 4.4 폐쇄 명령 형태

$T_{close}$ 를 최소화하려면 폐쇄 자세로의 계단 position 명령 + actuator 속도 한계가 기본이다.

**폐쇄 후 파지력 제한은 position 목표로 표현한다.** 실기 p1b 는 position 만 받고 feedforward 를 무시하므로 전류 (토크) 한계로 유지하는 것은 명령할 수 없다. 유지 단계에서는 목표 위치를 조정해 servo 오차 × 게인으로 힘을 제한한다. 규칙은 `robot.hand.hold.mode` 다: `close_target` (유지 목표 = `q_close`, 출하) \| `measured_offset` (폐쇄 완료 시점 측정 자세에서 `hold.delta_rad` 만큼 되돌린 목표). sim · 실기 각각의 servo 게인 차이 기록은 실기 단계의 항목이다 (G6-E) `[권장]`. 모터 자체 토크 한계 (`joint_limits.max_torque`, sim forcerange) 는 최종 상한이다.

폐쇄 체인 p1b 는 구동 좌표에서 명령한다. 수동 관절은 폐쇄 제약으로 결정된다 (기존 `rtc_urdf_bridge`, Pinocchio `RigidConstraintModel`). leap 은 폐쇄 체인이 없다.

**catch frame 과 폐쇄 체인.** catch frame 은 손바닥 (`l_palm_link` / `palm_lower`) 에 붙는 모델 빌더 추가 frame 이고 부모 · offset · 자세는 로봇 config YAML (`_base.yaml`) 로 연다 (L5 §11). 손바닥은 폐쇄 체인 루프의 **상류**이므로 catch frame FK · Jacobian 은 팔 관절만으로 정해지며, 폐쇄 체인 사영이 `held` (NUM-5) 여도 영향을 받지 않는다. catch frame 위치는 §4.5 의 포구점을 부모 frame 으로 옮긴 값이다 — 그 값은 catch frame 좌표이고 YAML `xyz` 는 부모 frame 좌표이므로, `palm_lower` 처럼 rpy 가 π 회전인 손에서는 **부호가 뒤집힌다**.

### 4.5 포켓 유효 깊이 $d_{eff}$ 와 포획 반경 $r_{cap}$ `[권장]` (TBD-HAND-04)

**두 값이 무엇인가.** 둘 다 **공 중심** 좌표계의 양이라 공 반지름이 이미 포함돼 있다 (표면 간 거리를 재는 보정을 다시 적용하면 두 번 빼게 된다).

- $r_{cap}$ — catch frame 에서 본 포획 영역의 **측면** 내접 반지름. L3 §4.6 게이트의 우변이다. 접근축 방향 오차는 $d_{eff}$ 와 폐쇄 타이밍이 흡수한다 (L3 §4.5).
- $d_{eff}$ — **포켓 깊이가 아니다.** L3 §4.5 의 창은 $d_{eff}/T_{close,tot}$ 로만 쓰이므로 $d_{eff}$ 는 **손이 실제로 흡수하는 접촉 상대속도 $v_{rel}$ × $T_{close,tot}$ 의 곱**이다.

$$d_{eff}=v_{rel}\,T_{close,tot}$$

  $v_{rel}$ 은 시각 발동 (아래 step 1 의 규칙) 으로 날려 넣은 공이 유지되는 상대속도 허용량의 실측이다. 유효 조건은 런타임 손 발동이 같은 **시각 발동** ($t_{cmd}=t_c-T_{close,lead}$, §4.3) 이라는 것이다. $d_{eff}$ 는 $T_{close,tot}$ 를 잰 폐쇄 시간으로 곱하므로 $T_{close,e2e}$ 가 바뀌면 같은 $v_{rel}$ 로 다시 유도한다. 포켓의 기하 깊이는 접촉 물리량으로 MASTER TBD-HAND-04 에 따로 남는다. 값은 `catching/search_grid.yaml` 의 `planner.search.grid.hand.d_eff` · `r_cap` 이다 (주석에 산정식이 있다). 두 값은 **YAML 의 `planner.search.grid.hand.*` 이지 `robot.hand.*` 가 아니다.**

산정 절차:

1. 기하 · 접촉 추정 (sim): 기하만으로는 답이 나오지 않는다 — 빈 `q_close` 는 주먹이라 공을 둘러싸지 않고, 그 자세의 자유공간은 포켓이 아니라 손가락이 쓸고 지나간 부피다. $q_{close}$ 는 **명령**이지 도달하는 자세가 아니다 (공이 있으면 손가락이 걸려 $q_{pre}$ 와 $q_{close}$ 사이에서 멈춘다). 그래서 접촉 시뮬레이션으로 한다: 팔을 고정하고 손을 $q_{pre}$ 로 정착시킨 뒤, 공을 preshape 손바닥 위 안착 높이에 **놓고** (떨어뜨리면 충돌 속도가 손가락을 튕겨 낙하를 재게 된다), 접근축 −z 로 중력을 걸어 앉히고, $q_{close}$ 를 명령한 뒤, 멈춘 자세에서 catch frame 세 축 ±방향으로 중력을 걸어 공이 남는지 본다. 안착 높이는 **충돌 geom 으로만** 잰다. 격자 위 유지 점의 집합에서 $r_{cap}$ (내접 반지름) 과 포구점 (catch frame 좌표) 을 얻는다. 같은 격자의 정지 $\rho$ 중앙값이 $\eta$ 의 근거다 (§4.2).
2. **투척 보정 — 실기에서만 한다 (절차).** 저속 투척으로 "폐쇄 늦음" 경계를 찾는다. 시퀀서가 있어야 한다 — 시퀀서 없이 비-RT 러너의 지연으로 폐쇄 시각을 맞추면 지터 × 공 속력이 $d_{eff}$ 와 같은 자릿수다. sim 에서는 보정하지 않는다 (sim 의 경계는 손의 흡수 · 반발이 정하고 실기 값이 아니다). 실기 투척에서 얻은 경계로 $d_{eff}$ · $r_{cap}$ 을 보정하고 provisional 표시를 푼다.
3. 반발 허용 여부 (L3 §4.5) 에 따라 $d$ 또는 $d(1+1/e)$ 를 쓴다. 시각 발동 fly-in 으로 잰 실효 $v_{rel}$ 은 이 둘 사이에 놓인다.

산출값은 **provisional** 이며 사용자 승인 대상이다. 산정식 · 실험값 · provisional 표시를 YAML 과 이 문서에 함께 기록한다 (G6-F).

**자세는 공의 자세다.** `q_pre` · `q_close` 와 위 값들은 특정 공 (출하 sim 공 = ITF Type 2 테니스공) 에 대해 정한 것이다. 공 사양이 바뀌면 재산정한다 — 작은 공까지 하나의 `q_close` 로 잡으려면 맨 계단 위치 명령으로는 안 되고 §4.4 의 유지 규칙이 필요하다. 자세는 사람이 만든 후보가 아니라 위 접촉 시험을 적합도로 한 탐색으로 정한다 — 실패 양상은 둘이다: (1) 나란한 손가락들이 서로 다른 시각에 도착해 공을 옆으로 짜낸다, (2) `q_close` 가 손가락이 멈추는 자세보다 깊으면 파지가 성립한 뒤에도 아직 안 닿은 관절이 계속 말려 공을 다시 민다. 그래서 `q_close` 는 주먹이 아니라 공에 걸려 멈추는 자세 근처다.

### 4.6 식별한 폐쇄 창과 포획 집합 (sim, provisional)

`mpc_docking` 구간 계획기와 `nlp` 탐색이 읽는 손의 값은 YAML 의 `robot.hand.docking.*` (L3 §6 · integrated_bringup README) 이고, 이 절은 그 값이 **무엇을 잰 것인지**와 출하값을 적는다. 식별 도구는 `integrated_bringup/tools/docking_ident` 다: 열린 손 (preshape) 에 공을 catch frame 의 한 직선을 따라 날려 넣고 정해진 순간에 `q_close` 를 지령한 뒤 닫힘 이후 "유지" 를 판정한다. 값은 **sim 식별이라 provisional** 이다 (sim 손의 흡수 · 반발은 실기의 것이 아니다, §4.5) — 원자료는 저장소 밖에 있다. 상자 `w040`: 접근 속력 대역은 `ur5e_p1b` 0.5 – 0.6 m/s, `iiwa7_leap` 1.6 – 1.7 m/s 이고 식별은 그 밖을 날리지 않았다.

- **폐쇄 창.** 폐쇄가 끝나는 순간이 공의 도달에 대해 어디에 있을 때 유지되는가. 명령 → 원점 도달의 창과 같은 것을 "폐쇄 완료 − 입구 평면 통과" 축에서 적은 것이 `closure.delta_lo` · `delta_hi` 이다 (폐쇄가 명령 뒤 $T_{close,e2e}$ 에 끝나는 축). 정해진 명목 순간 $\delta_0=T_{close,e2e}-T^{(t_c)}_{close,lead}$ 는 키가 아니다.
- **lateral 집합.** 공의 중심이 입구 평면을 지나는 위치 (catch frame $x,y$) 가운데 유지되는 5 mm 셀의 집합이다. 셀은 **유지율 규칙**으로 고른다: 가운데 조건에서 유지한 셀과 그 둘레 두 링을 후보로 삼아 5 조건 × 8 회, 어느 것도 건너뛰지 않고 날려, 40 회의 유지율이 기준 셀 (가운데 조건 유지가 가장 많은 셀, 같으면 40 회 유지율이 높은 것) 의 것과 5 %p 안이면 집합에 든다. 다각형은 그 셀들 안에 들어가는 볼록 다각형이다.
- **속도 집합.** 접근축 속력은 상자의 대역이고, 옆 속도의 상한은 유지율이 영속도 링의 것과 5 %p 안에 드는 마지막 링이다.

측정값 (2026-10-07):

| | `ur5e_p1b` | `iiwa7_leap` |
|---|---|---|
| `T_close_e2e` (200 회 평균) | 0.278 s (표준편차 0) | 0.098 s (표준편차 0.14 ms) |
| `T_close_lead` — 명령 → 원점 도달의 유지 창 | 0.2805 s (260.5 – 300.5 ms 의 가운데) | 0.0577 s (31.7 – 83.7 ms 의 가운데) |
| 입구 평면 축으로 환산한 lead (`mpc_docking` 이 실행하는 값) / docking 코어의 $\delta_0$ | 0.2678 s / 10.2 ms | 0.0256 s / 72.4 ms |
| `docking.s_ent` | 7.0 mm | 53.0 mm |
| 통로 `r_ent` / `tan_theta` | 17.7 mm / 0.4599 | 22.1 mm / 0.2764 |
| lateral 집합 | 후보 147 셀 중 **4 셀**, 내접원 중심 (−5, −5) mm 반지름 2.5 mm, 가장 가까운 면 2.3 mm | 후보 459 셀 중 **43 셀 — 한 영역이 아니라 흩어져 있다**, 내접원 중심 (−20, 20) mm 반지름 3.5 mm, 가장 가까운 면 3.3 mm |
| 접근 속력 대역 `c_min`–`c_cap_max` / `v_perp_max` | 0.5 – 0.6 m/s / 0.125 m/s | 1.6 – 1.7 m/s / 0.325 m/s |
| `closure.delta_lo` / `delta_hi` | −8.50 / +29.17 ms | +47.43 / +97.48 ms |
| 검증 (집합에서 뽑은 300 조건) | 277 유지, 95 % 하한 0.893 | 284 유지, 하한 0.920 |
| 접근축 가속 (5 · 9.81 m/s² · 대기 자세의 중력) | 300 / 300 / 300 | 287 / 275 / 289 (유지율 0.932 / 0.886 / 0.940) |
| `planner.search.grid.hand.d_eff` (1.0 m/s × $T_{close,tot}$) | 0.279 m | 0.099 m |

- `a_brake` (5.0) · `c_ent_max` (대역의 위 끝) 은 식별한 값이 아니라 접근 envelope 의 설계값이다. 반발계수 0.732 는 sim 공의 강체 면 값이고 손 자체의 것은 재지 않았다. 접촉점 (`contact_point_hand`) 은 첫 접촉 위치다.
- **`iiwa7_leap` 은 공이 도달한 뒤에 닫혀야 유지되는 손이다** — 폐쇄 완료는 원점 도달 14 – 66 ms 뒤의 창 안에 있어야 하고, 출하 `grid` × `mpc` 에서 폐쇄 지령은 $t_c-0.1037$ 에서 $t_c-0.0577$ 로 옮겨졌다 (옛 규칙은 폐쇄를 도달 6 ms 전에 끝내 창에서 20 ms 벗어났다). `ur5e_p1b` 의 창은 폐쇄 완료가 도달 22.5 ms 전 – 17.5 ms 뒤이고, 막 닫혔을 때 공이 오는 쪽이다.
- 두 로봇의 lateral 집합은 **작다.** `ur5e_p1b` 는 4 셀이 (40 · 40 · 38 · 38 / 40 회 유지) 집합이고 근처의 열두 셀은 32 – 37 회 유지에 그쳤다 — 가운데는 매번 유지하지만 (영속도에서 160 / 160) 그 둘레는 0.8 – 0.95 다. `iiwa7_leap` 은 43 셀이 기준 셀 (40 / 40) 의 5 %p 안이지만 한 영역을 이루지 않고 (손은 약 60 × 60 mm 에서 셀당 0.8 – 0.95 로, 흩어진 셀에서 38 회 이상 유지한다) 다각형은 가장 큰 연결 덩어리 안에 든다 (그 중심에서 영속도 157 / 160 유지). 가장 가까운 면까지의 거리가 `ur5e_p1b` 2.3 mm · `iiwa7_leap` 3.3 mm 라서 lateral chance 행은 공의 위치를 각각 십분의 수 mm · 1 mm 안팎으로 아는 경우에만 공을 받는다.

## 5. C++ 구현

### 5.1 프로파일

손별 YAML 프로파일 (`robot.hand.*`) 의 필드 정의는 `rtc_controllers/include/rtc_controllers/catching/catching_params.hpp` 의 `HandProfile` 이다. 고정 최대 크기 (`std::array`, 손 DoF ≤ 16) 로 힙을 쓰지 않는다. 구동 관절 수는 손 device 의 채널 수와 일치해야 한다 (검증기).

### 5.2 명령 포트 (RT에서 호출)

두지 않는다 — 손은 `ControllerOutput` 손 device slot 이다. 발행 · 전달은 CM 이 `DeviceBackend::WriteCommand` 로 한다 (`mujoco_native` / `udp_hand_native`).

### 5.3 시퀀서

- 입력: $t_c$ (동결된 값), now_real, $h$ = `ControllerState::dt`, 손 device 측정 관절 위치 · 속도, L7 지시 (`Home` / `Ready` / `Commit(t_c)` / `Retime(t_x)` / `Abort` / `Release`)
- 출력: 손 device slot (device 1) 의 position 목표 (`devices[1].commands`, `CommandType::kPosition`), 현재 phase, $\rho(t)$, `close_issued`, `at_target` (`q_tol` · `qd_tol` 판정), 타임아웃 플래그
- phase: Open, Preshape, Close, Hold, Release (msg 상수 그대로)
- 포구 컨트롤러 `Compute` 안에서 매 tick 호출 (RT). 진단은 SPSC 로 aux drain
- ROS 비의존 순수 조각 `hand_sequencer.hpp` (`rtc::catching`, 할당 0, noexcept, `now` 를 인자로 받아 자체 시계를 갖지 않는다). 비유한 측정은 그 자체로 전이를 만들지 않는다 ($\rho=0$ 이고 손은 어떤 목표에도 도달하지 않은 것) — Close 의 timeout 은 그래도 Close 를 끝낸다

전이 규칙 (한 tick 에 최대 한 번 — 이번 tick 에 들어간 phase 는 아직 명령되지 않았으므로 판정하지 않는다):
- `Open → Preshape`: 팔이 `wait_pose` 에 도착했다는 L7 지시 (`Ready`) 에서 — **시각 조건이 아니다**. `Ready()` 는 아직 닫히지 않은 commit 을 취소한다
- `Preshape → Close`: `Commit(t_c)` 가 무장된 뒤 `now_real ≥ t_cmd-h/2` (`HandCommandDueRounded`, §4.3). **무장은 COMMITTED 진입에서만** 이고, 무장된 폐쇄는 그 뒤의 `ABORT_SAFE` 에서도 $t_{cmd}$ 가 되면 지령된다 (아래 `Abort`) — `RETREAT` 진입이 아직 지령되지 않은 폐쇄를 푼다 (L7 §4.8). commit 은 시행당 한 번 (동결된 $t_c$ 는 하나). 무장된 $t_{cmd}$ 는 `Retime(t_x)` 가 $t_x-T_{close,lead}$ 로 옮긴다 (§4.3 "commit 뒤의 지령 시각") — commit 뒤 · 폐쇄 지령 전에만 듣고, 이미 지난 시각도 받는다 (다음 `Update` 가 지령한다 — 컨트롤러는 같은 tick 에 `Update` 를 부른다)
- `Close → Hold`: $\rho\ge\eta$ 또는 `T_close_timeout` 경과 (타임아웃이면 플래그). 유지 목표는 `hold.mode` 에 따른다 (§4.4)
- `Hold → Release`: L7 지시 (시각은 L7 §4.8 "RETREAT 순서" — 판정과 무관하게 팔이 대기 자세에 도착한 뒤. RETREAT 복귀 중에는 손을 열지 않는다)
- `Release` 목표는 **`q_pre`** 다 (`q_open` 은 homing 전용) — `at_target` (`q_tol`, `qd_tol`) 이면 `Preshape` 로 복귀해 다음 시행의 ARMED 준비를 마친다. 도달을 기다리는 쪽은 L7 이다: `RETREAT` 의 release 뒤 `T_release_timeout` 안에 도달하지 못하면 L7 이 `HAND_TIMEOUT` 으로 `IDLE` 에 가고 disarm 한다 (시퀀서 자체는 시계를 추가로 갖지 않는다)
- `Abort` 지시: COMMITTED 이후면 `t_cmd` 규칙대로 마저 닫고 `Hold` 로, 그 전이면 `q_pre` 유지 (abort 중인 supervisor 는 계속 `Update` 를 부른다)

## 6. YAML 파라미터 (손별 `robot.hand.*`)

값 · 기본값 · 범위는 출하 YAML (`integrated_bringup/config/{ur5e_p1b,iiwa7_leap}/controllers/demo_catching_controller.yaml` 의 `robot.hand`) 과 파서 (`rtc_controllers/src/params/catching_params.cpp`) · `HandProfile` 이 갖는다. 손 device 의 관절 이름은 `devices.<hand>.joint_state_names` 가 정하고 이 표의 키가 아니다. 손 프로파일 값은 전부 **provisional** 이다 — provisional 값은 활성 구성 TBD 검사가 실기 arm 을 막는다.

| 키 | 단위 | 뜻 |
|---|---|---|
| `robot.hand.q_open`, `q_pre`, `q_close` | rad | 구동 좌표 position 목표 (homing / preshape / 폐쇄). 관절 한계 안 (YAML ∩ URDF). `q_pre` · `q_close` 는 함께 주어야 한다 |
| `robot.hand.caging_mask` | – | $\mathcal C$ (§4.2). 없으면 모든 관절을 본다 (fail-closed) |
| `robot.hand.eta_close` | – | §4.2 $\eta$ |
| `robot.hand.rho_eps` | rad | §4.2 $|q^{cls}_i-q^{pre}_i|$ 하한 (검증기가 강제) |
| `robot.hand.hold.mode` | – | §4.4 유지 목표 규칙: `close_target` \| `measured_offset` |
| `robot.hand.hold.delta_rad` | rad | `measured_offset` 에서만 쓰는 여유량. Close 가 timeout 으로 끝나 측정이 비유한인 관절은 `q_close` 를 쓴다 |
| `robot.hand.T_close_e2e` | s | §4.2 종단 간 **실측** — 잰 폐쇄 시간만 뜻한다 (sim 은 200 회 시행의 평균, 실기는 G6-D). 시한 · γ 창 · $\delta_0$ 가 읽는다. 명령 시각은 정하지 않는다 |
| `robot.hand.T_close_lead` | s | §4.3 폐쇄를 공이 포구점 (원점) 에 닿기 얼마 전에 지령하는가 — 설계값, 원점 도달 축. 없으면 `T_close_e2e` 이고 configure 가 WARN 한다 (`HandProfile::CloseLead()`). `mpc_docking` 구간 계획기 아래에서는 컨트롤러가 입구 평면 축으로 환산해 쓴다 (§4.3). 검증기: $T_{freeze}\ge T_{close,lead}+T_{arm}+h$ |
| `robot.hand.docking.*` | – | §4.6 입구 평면 · 통로 · lateral 집합 · 속도 집합 · 폐쇄 창 · 충격 (`provisional: true`). 항상 파싱하고 **`nlp` 탐색 또는 `mpc_docking` 이 선택됐을 때만 요구한다** — 비었거나 TBD 면 park (`kMpcDockingInvalid`, 키 이름을 적는다). 키 목록과 값은 integrated_bringup README |
| `robot.hand.T_hold` | s | HOLD 의 길이 (재무장 · 시퀀서 도착 판정과 함께 튜닝) |
| `robot.hand.T_close_timeout` | s | Close 가 $\eta$ 에 못 닿고 Hold 로 넘어가는 시한. 키가 없으면 파서가 $2\,T_{close,e2e}$ 로 유도한다. 검증기: `> T_close_e2e` |
| `robot.hand.T_release_timeout` | s | `RETREAT` 의 `q_pre` 도착 대기 (L7 `HAND_TIMEOUT`). 키가 없으면 파서가 $m\,T_{close,e2e}$ 로 유도한다, $m=2\max\!\big(1/\eta,\ \ln(S_{\max}/q_{tol})/\ln\tfrac{1}{1-\eta}\big)$, $S_{\max}=\max_i|q_{close,i}-q_{pre,i}|$ (전 관절). 근거: $T_{close,e2e}$ 는 $\rho$ 가 $\eta$ 에 닿는 시각까지만 재지만 release 는 `q_pre` 에 **정착**해야 하므로 두 극한 플랜트 (토크 포화 — 이동 ∝ 거리 → $1/\eta$, 1차 선형 — `q_tol` 까지 로그 정착) 중 느린 쪽에 close timeout 과 같은 2 배를 곱한다. 상수 배수는 한 값으로 두 극한 플랜트를 못 덮는다. 개방이 1차 모델보다 느린 손은 키를 명시한다 (키가 있으면 유도하지 않는다). 검증기: 유한 · > 0 · `> T_close_e2e` |
| `robot.hand.capture.rho_min`, `rho_max` | – | L7 §4.4 손 관절 증거: 관절별 $\rho$ 가 이 띠 안이면 "도중에 멈춤". `rho_max` < 1 은 `q_close` 에 닿은 빈 손을, `rho_min` 은 닫히지 않은 채 다른 것을 미는 손가락을 제외한다. 블록이 없으면 손 증거 꺼짐 |
| `robot.hand.capture.effort_frac_min` | – | 같은 관절의 $s_i\tau_i/\tau_{\max,i}$ 하한. $\tau_{\max}$ 는 손 device 의 `joint_limits.max_torque` (없으면 시행을 park) |
| `robot.hand.capture.t_persist` | s | 막힘이 판정 tick 까지 끊기지 않아야 하는 시간. `supervisor.contact.t_confirm` 과 의미가 달라 따로 둔다 |
| `robot.hand.capture.min_joints` | – | stalled 관절 수 하한 |
| `robot.hand.capture.provisional` | – | `hold.mode: close_target` 에서만 허용 (검증기) — `measured_offset` 의 빈 손은 $\eta$ 교차 + `delta_rad` 에 멈춰 띠 안에 들어온다. 실기 구성을 막는다 (실기 손 드라이버의 effort lane 은 관절 토크가 아닐 수 있다 — `udp_hand` 는 전류) |
| `robot.hand.provisional` | – | 손 프로파일 전체의 provisional 표시 |
| `robot.hand.q_tol` | rad | §5.3 `at_target` 판정 — 최소 caging 이동량의 ~0.1 |
| `robot.hand.qd_tol` | rad/s | §5.3 정착 검사 $\Vert\dot q\Vert_\infty$ — 바이어스 학습 창 (L7 §4.4) 은 손 정지 후에만 연다 |

`robot.hand.T_pre` 는 쓰지 않는다 (파서가 거부한다 — 시각 기반 preshape 는 없다, §4.3 · §5.3).

## 7. 단위 기술 구현 순서

구현 순서의 기록은 두지 않는다. 남은 실기 항목은 실기 $T_{close,tot}$ 종단 간 식별 (G6-D, `[HW-P1B]`) 과 $d_{eff}$ · $r_{cap}$ 투척 보정 (§4.5 step 2) 이다.

## 8. 디버깅 방법

- 기록: 위상 전이 시각, $t_{cmd}$ 목표와 실제 틱 차이, 실측 tick 간격, $\rho(t)$, 타임아웃 플래그, 손 device slot 에 쓴 목표.
- 폐쇄가 늦다: 명령 시각 오차 (양자화 규칙) → 실기면 `udp_hand_node` 주기 (250 Hz) 와 링크 상태 → actuator 게인 · 속도 한계 순으로 확인.
- 폐쇄 후 손가락이 튄다: 유지 목표 전환 시점과 값을 확인.
- 실기와 시뮬레이션 $T_{close}$ 차이가 크다: sim $T_{close}$ 는 MJCF 게인에 의존하므로 시뮬레이션 성공률은 `[SIM-P1B]` 한정 결과로 표기한다.
- 손 명령이 안 나간다: 포구 컨트롤러가 활성인지 (활성 컨트롤러는 하나), `ValidateControllerOutput` 실패로 `BuildHoldOutput` 이, 또는 E-STOP · 해제 검증 창으로 `BuildLatchedHoldOutput` 이 대체했는지 확인.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G6-A | 시퀀서 명령 시각 오차 ≤ $h/2$ (1000 회, 실측 tick 간격 기준), 비교 축이 now_real 임을 $T_{arm}\neq0$ fixture 로 확인 | `[SIM-ANY]` |
| G6-B | RT 경로 할당 0, 손 device slot 에 쓴 목표와 backend 가 쓴 명령이 전 틱 일치. 컨트롤러 쪽 반쪽은 목표를 수락한 tick 부터 손 slot 명령이 clamp(목표) 와 bit-equal 인 것 (CM 쪽 반쪽은 RT 루프 테스트) | `[SIM-ANY]` |
| G6-C | 시뮬레이션 두 손의 $T_{close,e2e}(\eta)$ 분포 산출 (평균 · 최대 · 99 %, steady 시계와 tick × `dt` 두 축으로 재고 log drop 0), §4.1 go/no-go 판정 기록 — go/no-go 는 목표 속도에서 γ 창이 비지 않는 것이고, 입력 ($T_{close}$, $d_{eff}$, 가속 box, $\eta_v$, 목표 속도) 중 하나라도 provisional 이면 PASS(provisional) 로 기록한다 | `[SIM-P1B]` |
| G6-D | 실기 p1b $T_{close,tot}$ 종단 간 분포 산출, 99 % 값을 YAML 에 반영 | `[HW-P1B]` |
| G6-E | 고정 공 fixture 에서 폐쇄 후 유지 성공 (position 목표 유지 규칙, 반복 횟수는 사용자 결정) | `[HW-P1B]` |
| G6-F | $d_{eff}$ **와 $r_{cap}$** 의 산정식 · 실측값 · provisional 표시가 YAML (`planner.search.grid.hand.*`) 과 이 문서에 기록됨. YAML 의 표시는 값 옆 키가 아니라 블록 전체의 `planner.provisional` 이다 | `[SIM-ANY]` |

## 10. 미확정 항목

- TBD-HAND-01, TBD-HAND-04 의 투척 보정 (sim 보정은 하지 않고 실기에서만 한다 — §4.5 step 2), `index_mcp_aa_joint` 위치 한계 불일치 (TBD-HAND-05 — YAML 과 URDF · MJCF ctrlrange 가 다르다 — 유효 한계는 교집합), 손 프로파일 값 (provisional)
- **p1b 의 폐쇄 속도** — $T_{close,e2e}$ 가 **최소 비행시간에 그대로 들어간다.** 폐쇄 병목은 가장 먼 거리를 가는 관절 (엄지 `thumb_cmc_fe`) 의 이동량이므로 자세를 다시 찾을 때 $T_{close}$ 를 목적함수에 넣는다. p1b $d_{eff}$ 의 포켓 깊이 쪽 값은 스캔 상한에 걸린 하한값이다
- **자세 탐색 도구는 저장소에 없다** (작업용으로만 있었다). 시각 발동 fly-in 과 열린 손의 접촉 스캔은 `integrated_bringup/tools/docking_ident` 에 있다 (`mpc_docking` 의 입력 식별용 — 로봇 profile 이름만 받는다). 자세를 다시 찾거나 `g1_p1b` 의 손을 식별하려면 탐색 쪽이 다시 필요하다 — 편입이 후속 항목이다
- **폐쇄 창 · lateral 집합 · 속도 집합은 sim 식별이다 (provisional).** 값은 §4.6 과 `robot.hand.docking` 에 있고 실기의 손에서 다시 재야 한다 — sim 손의 흡수 · 반발은 실기의 것이 아니다. lateral 집합이 작다 (`ur5e_p1b` 4 셀, `iiwa7_leap` 43 셀이 흩어짐) 는 것은 재측정한 그대로이며 규칙을 느슨하게 해서 키운 것이 아니다. `T_close_lead` 도 같은 식별에서 나온 설계값이라 provisional 이다
- TBD-HAND-03 지문 잡음 (실기 σ) — 부호 · frame 은 닫혀 있다 (접촉 판정은 바이어스를 뺀 크기 $\Vert F-b\Vert$ 만 쓴다). sim 지문 lane 은 잡음이 0 이라 `NOT_EVALUATED(sim 무잡음)` 이고 값은 실기에서 잰다
