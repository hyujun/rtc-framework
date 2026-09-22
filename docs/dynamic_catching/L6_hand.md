# L6 — Hand: 손 시퀀서, 손 프로파일, `T_close` 식별

- 문서 버전: v0.5 (2026-09-19)
- 브랜치: 단계별 `type/kebab-slug` (main 기준, 마스터 §4.2)
- 코드 배치 `[확정 D-1]`: 시퀀서·프로파일 순수 로직은 rtc_controllers `catching`, YAML·식별 도구 배선은 `integrated_bringup`. 새 패키지를 만들지 않는다
- 단계: **S4** = L6a (손 프로파일 S4.1, $T_{close}$ 식별 S4.2·S4.3, 받을 수 있는 공 속력 S4.4 — go/no-go) · **S7.1** (시퀀서 → 손 device slot)
- 선행: 단계 W, L0, L1
- 비고: 손 명령은 포구 컨트롤러의 `ControllerOutput` 손 device slot 에 직접 쓴다 `[확정 D-11]`. 본 layer 는 **시각 제어와 프로파일**을 맡는다.
- 산출물: 손 프로파일 타입·파서, 손 시퀀서, $T_{close}$ 식별 도구

---

## 1. 범위 / 비범위

범위:
- preshape → 폐쇄 → 유지 → 해제의 시각 제어
- 손별 자세·한계를 YAML 프로파일로 분리
- 손 명령을 `ControllerOutput` 의 손 device slot (device 1 관례) 에 position 목표로 기록 `[확정 D-11]`. 전달은 기존 `DeviceBackend` 가 한다:
  - 시뮬레이션: `mujoco_native` (P1b·LEAP 모두)
  - 실기 P1b: `udp_hand_native` → `/p1b/joint_command` → 별도 프로세스 `udp_hand_node` (250 Hz, **position 만** — feedforward 무시)
- 종단 간 폐쇄 시간 $T_{close,tot}$ 식별 (D-11: $T_{link}$ 분리 측정 안 함)

비범위:
- 파지력 제어(grasp matrix 기반 내력 제어는 기존 WBC 과제)
- 접촉 판정 (L7, S7.3). 지문 부호는 sim·실기 모두 finger-on-object (0fcc1d23)
- 폐쇄 체인 기구학 자체(기존 `rtc_urdf_bridge` 재사용)
- 손 명령 포트 추상화 — v0.5 에서 삭제 (D-11)

## 2. 코드 확인 게이트

단계 W 에서 확인 완료 (2026-09-19, plan §2 · `WORKSPACE_ANALYSIS.md`).

| ID | 확인 항목 | 기록 |
|---|---|---|
| G6-1 | P1b 구동 좌표 (10 actuated: thumb 4, index 3, middle 2, ring 1) 와 URDF/MJCF 일치 | 닫힘 — 개수 일치: 로봇 config `devices.p1b.joint_state_names` 10개, URDF `proto_1b.urdf` 의 effort 3.0 revolute 10개 (나머지 revolute 10개는 effort 0 수동 관절), MJCF `<position>` actuator 10개. MJCF actuator 순서는 YAML 순서와 다르므로 **이름으로 대응**한다. 참고: `index_mcp_aa_joint` 위치 한계가 YAML (−0.349/0.524) 과 URDF·MJCF ctrlrange (−0.122/0.297) 에서 다르다 — 유효 한계는 YAML ∩ URDF 교집합이라 URDF 값 (TBD-HAND-05 로 기록) |
| G6-2 | P1b 드라이브 경로의 메시지·모드·주기·스탬프 | 닫힘 — `udp_hand_native` backend 가 `/p1b/joint_command` 발행, `udp_hand_node` 250 Hz, position 만, **명령 `header.stamp` 미사용** → $T_{link}$ 분리 불가, 종단 간 측정 (D-11) |
| G6-3 | P1b 명령 가능한 토크 한계 | 닫힘 — 설정·모델값은 일치: 관절당 `max_torque` 3.0 N·m (로봇 config YAML = URDF effort), MJCF forcerange ±3 N·m. 실기 명령은 position 만이므로 토크 한계를 직접 명령할 수 없다 (§4.4). CATCHING_MASTER §1.3 의 1.5 N·m 는 작성 시점 사용자 진술이며, 이 값을 그대로 운용 한계로 쓸 수 없다 — nominal·continuous·peak·설정값(3.0 N·m) 중 무엇을 운용 한계로 쓸지와 그 출처는 **D-12 미결정** (plan §7.3) |
| G6-4 | sim 두 손 MJCF actuator·폐쇄 체인 | 닫힘 — P1b: `<position>` kp 6000 (kv 250), forcerange ±3, `<equality><connect>` 5개 (MJCF 는 형제 저장소 hand-description). LEAP: `<general>` 16개, equality 없음 |
| G6-5 | 지문 센서 frame·부호 규약 | 부호 닫힘 — 실기 `HandSensorState` (250 Hz) 와 sim `WrenchStamped` 모두 finger-on-object (커밋 0fcc1d23, 변환 지점은 `rtc::grasp::PullContactConfig::force_sign` 하나). `rtc_msgs` FingertipSensor 주석의 반대 부호 서술은 stale. frame 세부는 S7.3 에서 확인 (TBD-HAND-03 의 frame·잡음 부분 유지) |
| G6-6 | 기존 WBC 손 명령 경로와의 충돌 | 닫힘 — 활성 컨트롤러는 한 번에 하나다. 포구 컨트롤러가 활성이면 DemoWbc 의 손 τ_ff 경로(`kPdFeedforward`)는 무관하다. 실기 손은 position 만 받으므로 손 명령은 position 목표로만 표현한다 |

## 3. 참고자료

[R1] caging 전략(넓은 preshape, 한쪽 개구부 진입, 엄지 차단)과 폐쇄 시간 요구의 근거, [R14] MuJoCo actuator.

## 4. 수학적 이론

### 4.1 폐쇄 시간 요구 (L3 §4.5 재게)

$$\gamma\ge1-\frac{d_{eff}}{\Vert v\Vert\,T_{close,tot}},\qquad T_{close,tot}=T_{close,e2e}+T_{tick}$$

$T_{close,e2e}$ 는 RT 가 폐쇄 명령을 기록한 tick 부터 인코더 $\rho\ge\eta$ 까지의 **종단 간 실측값**이다 `[확정 D-11]`. 실기 P1b 에서는 backend 발행 → `udp_hand_node` 250 Hz 주기 양자화 → 모터 응답이 모두 이 값에 들어간다. v0.4 의 $T_{link}$ 분리는 `udp_hand_node` 가 명령 stamp 를 읽지 않아 불가능하므로 삭제한다. sim 에서는 `mujoco_native` lock-step 이라 전달 지연이 사실상 0 이고 $T_{close,e2e}$ 는 MJCF 게인이 정한다.

손 성능은 $T_{close,tot}$ 하나로 요약되어 계획에 들어간다. 따라서 이 값의 **정의와 측정 절차**가 L6 의 핵심 산출물이다.

**이 값이 시스템 전체의 실현 가능성을 결정한다.** 받을 수 있는 최대 공 속력은

$$\Vert v\Vert_{\max}=\min(v_{dir,\max},v_{\max})+\frac{d_{eff}}{T_{close,tot}}$$

$v_{dir,\max}=1.5$ m/s, $d_{eff}=4$ cm, $T_{close,tot}=60$ ms면 2.17 m/s뿐이다. 6 m/s를 받으려면 $T_{close,tot}\le8.9$ ms가 필요하고, 이는 [R1]의 DLR-Hand-II(5 ms 급)와 같은 수준이다.

그래서 **S4 (L6a) 를 S5 이전에 go/no-go 로 한다** (plan §4). 이 값을 모른 채 `planner.gamma.*`, `reference.a_max`, rollout 창을 튜닝하면 재작업이 확정이다. 게이트: 목표 투척 속도(D-12, D-18 catchability 지도가 정한 범위)에서 γ 창이 비지 않을 것 — 비면 목표를 낮춘 뒤 진행한다 (S4.4). 토크에서 도출한 보수적 팔 가속 box (D-16) 가 $v_{dir,\max}$ 를 낮출 수 있으므로 S4.4 에서 함께 판정한다.

### 4.2 $T_{close}$의 정의 `[권장]`

- 명령 시각 $t_{cmd}$: RT 루프가 폐쇄 명령을 손 device slot 에 기록한 tick 의 steady 시각 (now_real).
- 폐쇄 진행률(구동 좌표 기준, 손 device 의 측정 관절 위치):

$$\rho(t)=\min_{i\in\mathcal C}\frac{(q_i(t)-q_i^{pre})\,s_i}{|q_i^{cls}-q_i^{pre}|},\qquad s_i=\mathrm{sign}(q_i^{cls}-q_i^{pre})$$

  $\mathcal C$는 caging에 필요한 관절 집합(YAML)이다. **$\mathcal C$의 모든 관절은 $|q_i^{cls}-q_i^{pre}|>\epsilon_\rho$ 를 만족해야 한다** — 그렇지 않으면 0으로 나누고 $s_i$도 정의되지 않는다. L0 파라미터 검증기가 강제한다(`armable=false`).
- $T_{close,e2e}(\eta)=\inf\{t-t_{cmd}:\rho(t)\ge\eta\}$. $\eta$는 공이 빠져나갈 수 없는 진행률이며, 손 형상에서 정한다(YAML).

명령 기록 시각과 인코더 수신 시각은 모두 RT 의 steady 시계로 잰다. 인코더 샘플의 원격 stamp 를 비교에 쓰지 않는다 (plan §3 "메시지 stale·나이" 행, `header.stamp` 판단 금지).

#### S4.2 실측 (sim, 2026-09-20) `[확정]`

손당 200 회 계단, CSV drop 0, $\eta=0.9$. **steady 축** (위 정의) 기준:

| 손 | $\mathcal C$ | 평균 | 최대 | p99 |
|---|---|---|---|---|
| P1b (10 DoF, forcerange 3.0 N·m) | 9 관절 | 396.2 ms | 399.1 ms | **398.6 ms** |
| LEAP (16 DoF, forcerange 0.95 N·m) | 13 관절 | 225.6 ms | 234.3 ms | **230.6 ms** |

이 표는 첫 측정 ($\eta=0.9$, 사용자 제공 자세) 이다. 출하 YAML 의 `T_close_e2e` 는 §4.5 가 $\eta$ 를 실측으로 다시 정한 뒤의 p99 이고 (LEAP 103.7 ms @ $\eta$ 0.5), **P1b 는 2026-09-21 에 자세가 바뀌어 다시 쟀다** — 탐색한 자세 (§4.5), $\mathcal C$ 10 관절, $\eta=0.7$, 200 회, drop 0: 평균 278.6 / 최대 280.7 / p99 **280.5 ms**. $T_{close,e2e}$ 는 자세 쌍의 성질이라 (이동하는 관절 집합과 이동량이 바뀐다) 자세를 바꾸면 이월하지 않고 다시 잰다. 어느 값이든 §4.1 의 예시 60 ms 보다 **한 자릿수 크다** — S4.4 는 이 값으로 $\Vert v\Vert_{\max}$ 를 다시 계산해야 하고, γ 창이 비면 목표 속도를 낮춘다.

**실기 모터 실측과 관절 속도 추정 (2026-09-22).** 사용자 실기 측정: `thumb_cmc_fe` 의 최대 속력 약 **7 rad/s, 그때 토크 약 2 N·m**. P1b 의 구동 관절 10 개는 전부 같은 모터가 관절에 직접 붙어 있다 (관절별 전달비 차이 없음 — 수동 관절은 링크 폐쇄로 따라간다). sim 은 손 관절에 속도 한계가 없고 `damping` 0.5 라 포화 시 종단속도가 3.0/0.5 = **6 rad/s @ 3 N·m** 이다 — 실측점과 다르다. 실측점을 모터 envelope (URDF 무부하 10.384 rad/s 를 지나는 직선 — 그 값이 모터 사양인지는 미확인) 와 부하 ($\tau_c+7b=2$; 점성/쿨롱 분해는 한 점으로 가를 수 없어 양끝을 본다) 로 넣고 출하 `q_pre`→`q_close` 계단을 폐쇄 체인 MJCF 에서 돌렸다 (baseline 은 출하 280.5 ms 를 276 ms 로 재현, 긴 스윙 positive control 은 7.0 rad/s @ 2.0 N·m 재현).

- 정상 최대속도는 전 관절 ≈ 7 rad/s 이지만, **폐쇄 이동량이 짧아 그 속도에 닿는 것은 `thumb_cmc_fe` (1.571 rad) 뿐**이다. 나머지는 피크 1.0–5.0 rad/s 의 가속 지배다 (`thumb_cmc_aa` 0.51 rad → 3.8–5.0, `middle_dip_fe` 0.69 → 3.0–4.3, `index_mcp_fe` 0.34 → 2.9–4.0, `ring_mcp_fe` 0.37 → 1.5–2.0, `index_dip_fe` 0.11 → 1.0–1.4)
- $T_{close}(\eta=0.7)$ 추정: 점성 240 / 반반 266 / 쿨롱 352 ms, 반사관성 0.02 → 194 · 0.10 → 310, 토크 cap 2.0 N·m → 298. **190–350 ms** 이고 불확도는 최대속도가 아니라 마찰 분해와 반사관성에서 온다 — 계단 응답의 가속 구간이 있으면 닫힌다. 출하 sim 값 280.5 ms 는 그 범위 안이다
- 병목은 `thumb_cmc_fe` 의 이동량 (탐색한 자세가 엄지를 −60° → +30° 로 굴린다 — 탐색의 목적함수에 폐쇄 시간이 없었다), 그다음 `middle_dip_fe`·`ring_mcp_fe`
- 실측 2 N·m 는 "운용 한계 1.5 N·m" 진술 (plan §7.3 D-12 손 토크) 과 양립하지 않는다

토크가 10배 작은 LEAP 이 오히려 빠른 이유는 관절 관성이다 (armature 0.003–0.006 vs P1b 0.05): P1b 는 **토크 포화 지배**라 운용 한계를 1.5 N·m 로 잡으면 $T_{close}$ 가 대략 $\sqrt2$ 배가 되고, LEAP 은 **강성 지배**라 그 민감도가 없다.

**tick×dt 축은 이 설정에서 쓸 수 없다.** 분석기는 CSV 한 행 = 한 tick = `dt` 로 보지만, sim 에서 RT 루프가 `control_rate` 보다 빠르게 돈다 (실측 P1b 810 Hz · LEAP 581 Hz, 설정은 둘 다 500 Hz). 그래서 행 간격이 `dt` 가 아니고 tick 축이 실제보다 길게 나온다. 대신 **sim 의 rtf 가 정확히 1.0000** 임을 sim 자체 로그로 확인했으므로 (steps/s 500.0, sim_time = wall) steady 값이 곧 물리 시간이고, 호스트 스톨 보정이 따로 필요 없다. 분석기가 띄우는 "tick axis NOT trusted — dropped CSV row" 경고는 **원인 진단이 틀렸다**: 드롭은 0 이었고 (`Controller CSV logging dropped` 0 건), 실제 원인은 루프 과속과 activity-gated 로깅의 구간 공백이다.

#### 잡히지 않을 때 무엇을 의심하는가 `[사용자, 2026-09-20]`

공이 palm 에 **닿았는데** 파지가 성립하지 않으면 원인은 둘 중 하나다 — **손 관절 속도** (곧 $T_{close,e2e}$: 공이 튕겨 나가기 전에 닫히지 못한다) 또는 **`q_pre` / `q_close` 자세** (닫히기는 하는데 그 형상이 공을 가두지 못한다). 접촉 자체가 없으면 그건 손 문제가 아니라 L2/L3 의 조준 문제이므로 이 계층을 먼저 의심하지 않는다. 이 둘이 S4.2 와 S4.5 가 각각 재는 양이고, 그래서 두 값이 S4.4 go/no-go 의 입력이다.

### 4.3 명령 시각의 양자화

시간 비교는 plan §3 규약을 따른다 `[확정 D-2]`. 손 명령에는 팔 지연 선행($T_{arm}$)을 적용하지 않는다 — 비교 대상은 **now_real** (매 tick steady 실측) 이다.

- Preshape: now_real ≥ $t_c-T_{pre}$
- Close: now_real ≥ $t_{cmd}$ 판정을 아래 양자화 규칙으로 한다. $t_{cmd}=t_c-T_{close,e2e}$ (L3 §4.11 의 $T_{close}+T_{link}$ 를 종단 간 값으로 대체)

RT 틱 $h$ 단위로만 명령할 수 있으므로 $t_{cmd}$를 넘지 않는 마지막 틱이 아니라 **처음으로 now_real ≥ $t_{cmd}-h/2$ 인 틱**에서 명령한다(가장 가까운 틱으로 반올림). 오차는 $\pm h/2$의 영평균이다. $h$ 는 `ControllerState::dt` (= 1/`control_rate`, 500 Hz 고정이 아니다) 이고, sim lock-step 에서는 실제 tick 간격이 이와 다를 수 있으므로 G6-A 는 실측 tick 간격으로 판정한다.

$T_{tick}=h/2$는 그 오차의 **worst case를 γ 창 예산에 넣는 값**이지, 명령 시각을 당기는 값이 아니다. $t_{cmd}$ 식에서 $T_{tick}$ 을 빼지 않는 것과 일관된다.

### 4.4 폐쇄 명령 형태

$T_{close}$를 최소화하려면 폐쇄 자세로의 계단 position 명령 + actuator 속도 한계가 기본이다.

**폐쇄 후 파지력 제한은 position 목표로 표현한다.** 실기 P1b 는 position 만 받고 feedforward 를 무시하므로(G6-2, G6-6) v0.4 의 "전류(토크) 한계로 유지" 는 명령할 수 없다. 유지 단계에서는 목표 위치를 조정해 servo 오차 × 게인으로 힘을 제한한다 (예: 폐쇄 완료 시점 측정 자세 쪽으로 목표를 되돌림). 구체적 규칙은 S7.1 에서 정하고 sim·실기 각각의 servo 게인 차이를 기록한다 `[권장]`. 모터 자체 토크 한계(3.0 N·m, sim forcerange ±3)는 최종 상한이다.

폐쇄 체인 P1b는 구동 좌표에서 명령한다. 수동 관절은 폐쇄 제약으로 결정된다(기존 `rtc_urdf_bridge`, Pinocchio `RigidConstraintModel`).

**catch frame 과 폐쇄 체인.** catch frame 은 손바닥(`l_palm_link` / `palm_lower`) 에 붙는 모델 빌더 추가 frame 이고 부모·offset·자세는 로봇 config YAML 로 연다 `[확정 D-10, D-17]` (plan §10). 손바닥은 폐쇄 체인 루프의 **상류**이므로 catch frame FK·Jacobian 은 팔 관절만으로 정해지며, 폐쇄 체인 사영이 `held` (NUM-5) 여도 영향을 받지 않는다. catch frame 위치 초기 제안값(포켓 중심)은 **아래 §4.5 의 실측 포구점**을 부모 frame 으로 옮긴 값이다 — S2.3b (2026-09-21, plan §10). 그 값은 catch frame 좌표이고 YAML `xyz` 는 부모 frame 좌표이므로, `palm_lower` 처럼 rpy 가 π 회전인 손에서는 **부호가 뒤집힌다**. "preshape 손끝 중심" 을 쓰던 이전 정의는 기각됐다 (근거: plan §10).

### 4.5 포켓 유효 깊이 $d_{eff}$ 산정 `[권장]` (S4.5, TBD-HAND-04)

1. 기하 추정: preshape 자세에서 FK로 손바닥 평면과 폐쇄 시 손가락이 형성하는 차단선 사이 거리를 접근축(catch frame +z) 방향으로 측정한다. 공 반지름을 뺀다. 같은 FK 에서 포획 반경 $r_{cap}$ (L3 §4.6 게이트 우변, 같은 TBD-HAND-04) 도 함께 산정한다 — S4.5 (plan §4.4 S4a, 2026-09-20).
2. 실험 보정: 시뮬레이션과 실기에서 저속 투척으로 "폐쇄 늦음" 경계를 찾는다. **S7.1 (손 시퀀서) 이후에 한다** — 시퀀서 없이 비-RT 러너의 지연으로 폐쇄 시각을 맞추면 지터 × 공 속력이 $d_{eff}$ 와 같은 자릿수다 (plan §4.4 S4a).
3. 반발 허용 여부(L3 §4.5)에 따라 $d$ 또는 $d(1+1/e)$를 쓴다.

산출값은 **provisional** 이며 사용자 승인 대상이다 (plan §4.4 S4a). 산정식·실험값·provisional 표시를 YAML (L3 §6 `planner.hand.*` — **§6 의 `robot.hand.*` 가 아니다**) 과 이 문서에 함께 기록한다 (G6-F).

#### S4.5 실측 (sim, 2026-09-20) `[확정]`

| 손 | 시행 | 파지 성공 | $r_{cap}$ | $d_{eff}$ | 포구점 (catch frame) | $\rho$ (정지) |
|---|---|---|---|---|---|---|
| LEAP (16 DoF) | 793 | **216** | **31.0 mm** | **80 mm** | (−0.035, +0.015, 0.069) | 0.50 (중앙값) |
| P1b (10 DoF), 사용자 제공 자세 | 851 | **0** | 없음 | 없음 | — | 1.00 (정지 안 함) |
| P1b (10 DoF), **탐색한 자세** (2026-09-21, 출하) | 445 | **214** | **24 mm** | **≥ 95 mm** (스캔 상한) | (+0.015, +0.145, 0.052) | 0.74 (중앙값) |

공은 출하 sim 공 = ITF Type 2 테니스공 (반지름 33.5 mm, 질량 57 g).

**기하만으로는 답이 안 나온다.** 빈 `q_close` 는 주먹이라 아무것도 둘러싸지 않는다 — 두 손 모두 그 자세의 자유공간에 지름 67 mm 공이 들어갈 닫힌 영역이 **0 개**다. 그 자세의 자유공간을 재면 포켓이 아니라 손가락이 **쓸고 지나간 부피**를 재게 된다 (LEAP 에서 0.77 L). $q_{close}$ 는 **명령**이지 도달하는 자세가 아니다: 공이 있으면 손가락이 걸려 $q_{pre}$ 와 $q_{close}$ 사이에서 멈춘다.

그래서 step 1 을 **접촉 시뮬레이션**으로 한다 — 팔을 고정하고 손을 $q_{pre}$ 로 정착시킨 뒤, 공을 preshape 손바닥 위 안착 높이에 **놓고** (떨어뜨리면 충돌 속도 1.3 m/s 로 손가락이 튕겨내 낙하를 재게 된다), 접근축 −z 로 중력을 걸어 앉히고, $q_{close}$ 를 명령한 뒤, 멈춘 자세에서 catch frame 세 축 ±방향으로 중력을 걸어 공이 남는지 본다. 안착 높이는 **충돌 geom 으로만** 잰다 (p1b 손 geom 42 개 중 충돌용은 21 개뿐이다). 투척이 없으므로 step 2 를 당긴 것이 아니다 — step 2 를 S7.1 뒤로 미룬 이유는 투척 타이밍 지터이고, 여기엔 그 항이 없다.

두 값 모두 **공 중심** 좌표계의 양이라 공 반지름이 이미 포함돼 있다. 위 step 1 의 "공 반지름을 뺀다" 는 표면 간 거리를 잴 때의 보정이고, 여기에 다시 적용하면 두 번 빼게 된다. $r_{cap}$ 은 **측면** 허용량으로 읽는다 — 접근축 방향 오차는 $d_{eff}$ 와 폐쇄 타이밍이 흡수한다 (L3 §4.5).

**P1b 는 사용자 제공 자세로는 이 공을 잡지 못한다.** 접촉 이력이 원인을 직접 말한다: $\rho$ 0.01–0.15 에서 검지(또는 약지) **하나**가 먼저 닿고, 맞은편 접촉이 생기기 전에 — 엄지는 도착조차 못 한다 — 공을 옆으로 밀어낸다. 공 크기 탓도 아니다: 반지름 25 mm 로도 851 중 0, 20 mm 에서야 109 중 9 가 잡힌다 (이 저장소가 같은 손으로 이미 파지하는 38 mm 원판 fixture 와 같은 자리다). 즉 **포획 영역이 비어 있어** $r_{cap}$ 이 작은 것이 아니라 존재하지 않고, $d_{eff}$ 는 통과할 포구점이 없다. 이것은 위 §4.2 "잡히지 않을 때 무엇을 의심하는가" 가 가리키는 **pre/close 자세** 쪽 원인이다.

#### P1b 자세는 탐색으로 정했다 (2026-09-21) `[확정]`

사용자가 두 번째 자세를 제공했지만 그것도 0 이었고, 손으로 만든 후보 40 여 개 (고정한 공을 감싸게 해 멈춘 자세를 `q_close` 로 쓰는 방법 포함) 도 자유 공에서는 전부 실패했다. 실패 양상은 둘이다. (1) 나란한 세 손가락이 서로 다른 시각에 도착해 공을 $\pm x$ 로 짜낸다 — 이 손에는 그 방향을 막는 것이 없다. (2) `q_close` 가 손가락이 멈추는 자세보다 깊으면, 파지가 성립한 뒤에도 아직 안 닿은 관절이 계속 말려 공을 다시 밀어낸다.

그래서 `q_pre`·`q_close` 20 개 값을 **위 시험 그대로** (놓기 → 계단 → 세 축 ±g 흔들기) 에 대고 진화 탐색했다 (사용자 자세 두 벌이 seed, 12 개 놓는 위치 중 유지한 수가 적합도, 부분 점수로 기울기). 7 세대에서 상위 5 개체가 12/12 에 도달했다. 찾은 자세는 제공받은 자세와 세 가지가 다르다.

- 손가락이 **절반만** 닫는다 (MCP −30…−43°, 제공 자세는 −60…−80°). `q_close` 는 주먹이 아니라 공에 걸려 멈추는 자세 근처다.
- `index_mcp_aa` 가 움직인다 (+8° → +17°, URDF 한계). 제공 자세가 한 번도 쓰지 않은 관절이고, 검지를 안쪽으로 돌려 $x$ 방향 탈출을 막는다.
- 엄지는 집지 않고 **뒤로 구른다** (`thumb_cmc_fe` −60° → +30°). 엄지 끝은 손바닥 위 8–11 cm 에 떠 있어 놓인 공에 닿지 못하고, 엄지 **뿌리**가 −y 쪽 받침이 된다.

검증은 탐색이 보지 못한 조건 (5 mm 격자 445 점) 에서 했다: 214 점 유지, $r_{cap}$ 24 mm, 정지 $\rho$ 중앙값 0.74. $d_{eff}$ 는 스캔 상한 (+95 mm) 에서도 아직 잡혀 있었으므로 **하한**이다. `preshape` 는 거의 편 손이다 — 제공 자세처럼 컵 모양으로 오므리면 공이 컵 **위에** 올라앉는다.

**이 자세는 이 공의 자세다.** 같은 자세로 지름 60 mm 는 452 중 212, 50 mm 는 459 중 159 를 잡지만 40 mm 이하는 0 이다 (손가락이 닿지 않는다). 작은 공까지 하나의 `q_close` 로 잡으려면 맨 계단 위치 명령으로는 안 되고 §4.4 의 유지 규칙 (S7.1) 이 필요하다.

#### 놓인 공과 날아드는 공은 다르다 (fly-in, 2026-09-21)

위 스캔은 공을 **놓고** 닫는다 — 포획 영역의 기하를 재는 시험이지 포구 시험이 아니다. 공을 접근축을 따라 던져 넣고, 공 중심이 preshape 의 엄지·검지·중지 손끝이 이루는 평면을 지날 때 `q_close` 계단을 쏘면 (67 mm 공):

| 손 | 0.25 m/s | 0.5 m/s | 1 m/s | 2 m/s |
|---|---|---|---|---|
| LEAP (304 점) | 18 | 54 | **74** | 63 |
| P1b 탐색 자세 (130 점) | 9 | 3 | 0 | 0 |

- LEAP 은 1 m/s 에서 가장 잘 잡는다. 발동 위치가 고정이면 한 속력에서만 타이밍이 맞는다 — §4.3 이 위치가 아니라 $t_c-T_{close,e2e}$ 로 명령하는 이유를 그대로 보여 준다.
- **P1b 는 0.5 m/s 부터 거의 못 잡는다.** 폐쇄가 LEAP 보다 약 2.7 배 느려서 (280 ms 대 104 ms) 손가락이 도착하기 전에 공이 손바닥에서 튕겨 나간다. 자세 문제를 걷어낸 뒤 남은 것은 §4.2 의 첫 번째 용의자인 **관절 속도**이고, S4.4 가 다룰 양이다.
- 이 표는 발동 규칙 하나에 대한 결과라 상한이 아니다 — 시퀀서 (S7.1) 가 $t_{cmd}$ 로 명령하면 달라진다.

#### 시각 발동 fly-in — 손이 실제로 흡수하는 상대속도 (S4.4, 2026-09-22) `[확정]`

위 표의 발동을 §4.3 의 규칙 그대로 **시각**으로 바꿨다: $t_{cmd}=t_c-T_{close,e2e}$, $t_c$ 는 공 중심이 catch 원점에 닿는 시각 (무중력 등속이라 결정적이고, python 단독 sim 이라 발동 지터가 0 이다). 자세와 $T_{close,e2e}$ 는 **출하 YAML** 에서 읽는다. 측면 격자는 catch 원점 ±40 mm 의 81 점, 67 mm 공.

| 상대속력 [m/s] | 0.1 | 0.3 | 0.5 | 0.75 | 1.0 | 1.5 | 2.0 |
|---|---|---|---|---|---|---|---|
| LEAP 유지 (/81) | 64 | 65 | 63 | 62 | **50** | 6 | 0 |
| LEAP 중심 3×3 (/9) | 9 | 9 | 9 | 9 | 9 | 2 | 0 |
| P1b 유지 (/81) | 25 | 19 | 15 | 9 | 8 | 6 | 3 |
| P1b 중심 3×3 (/9) | 7 | 8 | 8 | 7 | **7** | 5 | 3 |

- ⚠️ **위 "P1b 는 0.5 m/s 부터 거의 못 잡는다" 는 위치 발동 규칙의 산물이었다.** 시각 발동이면 P1b 도 포구점 중심에서는 1.0 m/s 까지 잡는다 (0.5 m/s: 130 중 3 → 81 중 15). 폐쇄가 느린 손일수록 발동을 앞당겨야 하므로 위치 고정 발동의 편향이 크다.
- **LEAP 의 실효 상대속도 허용량은 약 1.0–1.25 m/s** 다 — 공식의 `d` (0.080/0.1047 = 0.76 m/s) 와 `d(1+1/e)` (1.78 m/s) 사이. 즉 L3 §4.5 가 채택한 `d` 는 보수적이고 반발 허용값은 낙관적이다.
- **P1b 는 중심 기준 약 1.0 m/s** 로 공식값 0.34 m/s 보다 크다 — $d_{eff}$ 0.095 m 가 스캔 상한에 걸린 **하한**이었기 때문이다. 대신 속력이 오르면 유지 면적이 줄어든다: 유효 $r_{cap}$ 가 0.1 m/s 에서 28 mm, 1.0 m/s 에서 16 mm. 면적이 저속의 절반 이상인 범위로 보면 약 0.5 m/s 다.
- **`planner.hand.d_eff` 의 뜻 `[확정 2026-09-22 사용자]`.** 이 키는 포켓 깊이가 아니라 **위 fly-in 허용 상대속도 × $T_{close,tot}$** 다 — P1b 1.0 × 0.2815 = **0.2815 m**, LEAP 1.0 × 0.1047 = **0.1047 m**. L3 §4.5 의 창은 $d_{eff}/T_{close,tot}$ 로만 쓰므로 식·코드는 그대로고, S4.5 의 포켓 깊이 (0.095 / 0.080 m) 는 접촉 물리량으로 이 문서와 MASTER TBD-HAND-04 에 남는다. 근거는 S3.5b gate 지도 (포켓 깊이로는 `ur5e_p1b` 가 0 — plan §4.4 S3.5b 결과). 유효 조건은 런타임 손 발동이 위와 같은 **시각 발동**이어야 한다는 것과 아래 한계 그대로다 (plan §7.3).
- 한계: 손 frame 을 등속으로 뒀다 (팔이 감속할 때의 비관성 항 없음), 발동 시각 오차 민감도는 재지 않았다, LEAP 의 저속 시행은 발동 전에 공이 손가락에 먼저 닿는다 (0.1 m/s 에서 81 중 51).
- S4.4 의 판정표는 이 실측값 (**1.0 m/s**) 을 상대속도 허용량으로 쓴다 (plan §11). 도구는 private 이다 (`flyin_timed.py`, S4a `flyin.py` 의 rig 를 상속).

#### $\eta$ 는 판단이 아니라 실측이다 `[확정 2026-09-20]`

§4.2 는 $\eta$ 를 "공이 빠져나갈 수 없는 진행률" 로 정의한다. S4.5 가 그것을 **쟀다**: LEAP 이 공을 실제로 쥔 216 시행의 정지 $\rho$ 는 중앙값 **0.50**, 최대 0.67 이다. 출하 프로파일이 처음 쓰던 0.9 는 보수적인 값이 아니라 **도달 불가능한 값**이었다 — 793 시행 중 $\rho$ 가 0.96 을 넘은 적이 없다. $T_{close,e2e}$ 는 $\rho\ge\eta$ 까지의 시간이므로, $\eta=0.9$ 로 잰 값은 **손이 빈 허공에서만 완주하는 동작**의 시간이었다.

S4.2 의 같은 200 시행을 $\eta$ 별로 다시 읽으면 (p99, ms):

| $\eta$ | 0.3 | 0.5 | 0.7 | 0.9 |
|---|---|---|---|---|
| P1b | 212.5 | **282.7** | 343.1 | 398.6 |
| LEAP | 72.0 | **103.7** | 140.3 | 230.6 |

LEAP 의 출하 YAML 은 $\eta=0.5$ 와 그 열의 p99 를 쓴다. 손이 빨라진 것이 아니라 **임계가 실제로 쥐는 지점으로 옮겨간 것**이다. 위 표의 P1b 행은 **사용자 제공 자세**의 값이라 출하값이 아니다: 탐색한 자세에서 P1b 가 공을 쥔 214 시행의 정지 $\rho$ 중앙값은 **0.74** 이고, 출하 YAML 은 그 바로 아래인 $\eta=0.7$ 과 그 자세로 다시 잰 p99 **280.5 ms** 를 쓴다 (§4.2).

**S4.4 로 넘기는 값** (LEAP, $T_{tick}=h/2=1$ ms, $v_{dir,\max}=1.5$ m/s 예시):

$$\Vert v\Vert_{\max}=1.5+\frac{0.080}{0.1037+0.001}=2.26\ \text{m/s}$$

P1b 는 같은 식으로 $1.5+0.095/(0.2805+0.001)=$ **1.84 m/s** 인데, 이것은 **공식값**이다 — 위치 발동 fly-in 표에서 이 손이 받은 속력은 그보다 훨씬 낮다 (시각 발동으로는 공식보다 **높다** — 위 "시각 발동 fly-in"). **두 식의 $v_{dir,\max}=1.5$ m/s 는 예시값이다**: S4.4 가 수락 자세마다 푼 값은 중앙값 1.1–1.4 m/s 이고 자세에 따라 0.4–3.5 m/s 로 퍼진다 (plan §4.4 S4.4 결과). LEAP 을 $\eta=0.9$ 로 잡으면 1.85 m/s 다. $d_{eff}$ 가 §4.1 예시의 4 cm 보다 두 배 커서 상한이 예시의 2.17 m/s 와 같은 자리로 올라왔지만, S0.7 가정값 8.4 m/s 와는 여전히 3.7 배 차이다 — S4.4 는 목표 속도를 낮춘다.

## 5. C++ 구현

인터페이스 이름·시그니처는 S1.8·S7.1 에서 repo 규약(namespace `rtc::catching`, PascalCase 함수, noexcept·할당 0)으로 확정한다. 아래는 요구 사항이다.

### 5.1 프로파일

손별 YAML 프로파일 (S4.1). 고정 최대 크기(`std::array`, 손 DoF ≤ 16) 로 두어 힙을 쓰지 않는다.

| 필드 | 의미 |
|---|---|
| `n` | 구동 관절 수 (P1b 10, LEAP 16) — 손 device 의 채널 수와 일치해야 한다 (검증기) |
| `q_open`, `q_pre`, `q_close` | 구동 좌표 position 목표 [rad] |
| 유지 목표 규칙 파라미터 | §4.4, S7.1 에서 확정 (v0.4 의 `effort_limit_hold` 는 삭제) |
| `caging_mask` | $\mathcal C$ |
| `eta_close` | §4.2 $\eta$ |
| `T_close_e2e` | 종단 간 식별값 [s] (v0.4 의 `T_close`·`T_link` 두 필드를 대체) |
| `T_pre`, `T_hold`, `T_close_timeout` | [s] |

### 5.2 명령 포트 (RT에서 호출)

v0.5 에서 삭제 — 손은 `ControllerOutput` 손 device slot (D-11). `HandCommandPort`·`InLoopHandPort`·`AsyncHandPort` 와 RT→SPSC→발행 스레드 경로는 만들지 않는다. 발행·전달은 CM 이 `DeviceBackend::WriteCommand` 로 한다 (`mujoco_native` / `udp_hand_native`).

### 5.3 시퀀서

- 입력: plan 스냅샷의 $t_c$, $t_{cmd}$ (L3, 동결 전 갱신 허용), now_real, $h$ = `ControllerState::dt`, 손 device 측정 관절 위치, L7 지시 (abort·release)
- 출력: 손 device slot (device 1) 의 position 목표 (`devices[1].commands`, `CommandType::kPosition`), 현재 phase, $\rho(t)$, 타임아웃 플래그
- phase: Open, Preshape, Close, Hold, Release
- 포구 컨트롤러 `Compute` 안에서 매 tick 호출 (RT). 진단은 SPSC 로 aux drain

전이 규칙:
- `Open → Preshape`: now_real ≥ $t_c-T_{pre}$
- `Preshape → Close`: §4.3 규칙의 틱
- `Close → Hold`: $\rho\ge\eta$ 또는 `T_close_timeout` 경과(타임아웃이면 플래그)
- `Hold → Release`: L7 지시

## 6. YAML 파라미터 (손별 `robot.hand.*`)

손 프로파일 값은 전부 **provisional** 이다 (S4.1 초안, S4.2·S4.3 식별 후 갱신). provisional 값은 활성 구성 TBD 검사가 실기 arm 을 막는다 (D-12 와 같은 규칙).

| 키 | 타입 | 단위 | 기본값 | 범위 | 근거 |
|---|---|---|---|---|---|
| `robot.hand.name` | string | – | `p1b` / `leap` | – | 로봇 구성 |
| `robot.hand.joint_names` | string[] | – | 로봇 config | – | `devices.<hand>.joint_state_names` 참조 (사본 금지, G6-1) |
| `robot.hand.q_open`, `q_pre`, `q_close` | double[n] | rad | `TBD` (provisional) | 관절 한계 내 (YAML ∩ URDF) | S4.1 |
| `robot.hand.caging_mask` | bool[n] | – | `TBD` | – | 손 형상 |
| `robot.hand.eta_close` | double | – | `TBD` | 0.5–1 | §4.2 |
| `robot.hand.rho_eps` | double | rad | 0.02 | >0 | §4.2 $|q^{cls}_i-q^{pre}_i|$ 하한 (L0 검증기가 강제) |
| `robot.hand.hold.*` | – | – | `TBD` | – | §4.4 유지 목표 규칙 (S7.1) |
| `robot.hand.T_close_e2e` | double | s | `TBD` | ≥0 | §4.2 종단 간 실측 (sim S4.2, 실기 S4.3·S10) |
| `robot.hand.T_pre` | double | s | 0.3 | 0–1 | 튜닝 |
| `robot.hand.T_hold` | double | s | 1.0 | 0–5 | 튜닝 |
| `robot.hand.T_close_timeout` | double | s | `TBD` | > `T_close_e2e` | 실측 후 |

v0.4 의 `robot.hand.effort_limit_hold`, `robot.hand.T_link`, `robot.hand.port`, `robot.hand.async.*` 는 삭제한다 (D-11).

## 7. 단위 기술 구현 순서

**L6a = S4a·S4.4 (S5 이전, go/no-go)**

- **L6.1** (S4.1) ✅ **완료 2026-09-20** — 두 벌 모두 `q_pre`/`q_close` 는 **사용자 제공** (TBD-HAND-05 충족, demo GUI 에서 자세를 잡아 `hand_presets_<group>.json` 으로 회수). **P1b 는 2026-09-21 에 그 자세를 seed 로 탐색한 값으로 교체했다** (§4.5 — 제공 자세는 출하 공을 파지하지 못한다). 손 프로파일 YAML 두 벌(P1b 10 DoF, LEAP 16 DoF) — provisional. 파서·검증기는 S1.7 의 `rtc::catching::HandProfile` 이 이미 있고, S4.1 은 `q_open`·`eta_close`·`T_close_e2e` 를 더한다 (나머지 §5.1 필드는 S7.1). 자세는 sim 에서 사용자 확인 후 S4.2 로 간다.
- **L6.2** (S4.2) ✅ **완료 2026-09-20** (결과: §4.2 "S4.2 실측"). $T_{close}$ 식별 도구: 손 device slot 에 계단 position 명령 반복(**손당 200 회** — 99 % 는 20 회로 말할 수 없다) + 기존 `DeviceStateLog` CSV (`<hand>_state.csv` 의 `command_*`·`actual_pos_*`·`t_relative_s`) → $\rho(t)$, $T_{close,e2e}(\eta)$ 분포(평균, 최대, 99%) 산출 (sim, 두 손). sim lock-step 의 steady 값은 호스트 스톨을 포함하므로 steady·tick×dt 두 축으로 보고한다. **S4.0 선행 필요** — `DemoJointController` 는 손 목표를 quintic 궤적으로 보간해 계단 응답을 낼 수 없으므로, S4.0 의 포구 컨트롤러 최소 골격(손 계단 진단 모드, 팔은 현재 자세 hold)이 있어야 이 계단 명령을 낼 수 있다 (plan §4.2 DAG, §4.4 S4a).
- **L6.3** (S4.4) 산출값으로 §4.1 의 $\Vert v\Vert_{\max}$ 를 계산해 목표 투척 속도와 대조하고, γ 창이 비면 사용자에게 보고하고 목표를 낮춘다. 목표 속도는 **S3.5a kinematic catchability 지도**가 정한 범위이고 (D-18, D-12 는 최종 성공률 floor·시행 수와 손 토크 권위 출처만 남는다), 이 판정(S4.4 go/no-go)의 결과는 다시 S3.5b(gate-catchable 지도)·S3.6(vision 요구 사양)으로 흐른다 (plan §4.2 DAG, §4.4 S3b·S4.4).
- **L6.5** (S4.3) 실기 $T_{close,tot}$ 종단 간 식별 `[HW-P1B]`. **S10 으로 이월** (2026-09-20 사용자 결정 — S4.0 골격은 sim 전용이고 실기 팔 hold 는 E-8 승인 대상이다, plan §4.4 S4a).
- **L6 §4.5** (S4.5) ✅ **완료 2026-09-20** (결과: §4.5 "S4.5 실측") — 포켓 유효 깊이 $d_{eff}$ 와 포획 반경 $r_{cap}$ 산정, provisional, 사용자 승인. LEAP $r_{cap}$ 31.0 mm · $d_{eff}$ 80 mm, P1b $r_{cap}$ 24 mm · $d_{eff}$ ≥ 95 mm (2026-09-21, 탐색한 자세 — 제공 자세로는 851 중 0 이었다). sim 투척 보정은 S7.1 뒤 (G6-F, TBD-HAND-04). S4.4 go/no-go 의 입력.

번호 L6.4 는 v0.4 의 `AsyncHandPort` 였고 D-11 로 삭제했다. 번호는 작업 항목 식별자라 재부여하지 않는다.

**L6b = S7.1**

- **L6.7** 손 시퀀서 → 손 device slot (§5.3), 시각 테스트 (G6-A).
- **L6.6** 시뮬레이션 손 actuator를 실측 계단 응답에 맞추는 보정(선택) `[SIM-P1B]`.

## 8. 디버깅 방법

- 기록: 위상 전이 시각, $t_{cmd}$ 목표와 실제 틱 차이, 실측 tick 간격, $\rho(t)$, 타임아웃 플래그, 손 device slot 에 쓴 목표.
- 폐쇄가 늦다: 명령 시각 오차(양자화 규칙) → 실기면 `udp_hand_node` 주기(250 Hz)와 링크 상태 → actuator 게인·속도 한계 순으로 확인.
- 폐쇄 후 손가락이 튄다: 유지 목표 전환 시점과 값을 확인.
- 실기와 시뮬레이션 $T_{close}$ 차이가 크다: L6.6 보정 전에는 시뮬레이션 성공률을 `[SIM-P1B]` 한정 결과로 표기한다 (plan §12: sim $T_{close}$ 는 MJCF 게인에 의존).
- 손 명령이 안 나간다: 포구 컨트롤러가 활성인지 (활성 컨트롤러 하나만, G6-6), `ValidateControllerOutput` 실패로 `BuildHoldOutput` 이 대체했는지 확인.

## 9. 검증 방법과 합격 게이트

| 게이트 | 기준 | 태그 |
|---|---|---|
| G6-A | 시퀀서 명령 시각 오차 ≤ $h/2$ (1000회, 실측 tick 간격 기준), 비교 축이 now_real 임을 $T_{arm}\neq0$ fixture 로 확인 | `[SIM-ANY]` |
| G6-B | RT 경로 할당 0, 손 device slot 에 쓴 목표와 backend 가 쓴 명령이 전 틱 일치 (v0.4 의 async 링 드롭 기준은 D-11 로 무관) | `[SIM-ANY]` |
| G6-C | 시뮬레이션 두 손의 $T_{close,e2e}(\eta)$ 분포 산출, S4.4 go/no-go 판정 기록 | `[SIM-P1B]` |
| G6-D | 실기 P1b $T_{close,tot}$ 종단 간 분포 산출, 99% 값을 YAML에 반영 | `[HW-P1B]` |
| G6-E | 고정 공 fixture에서 폐쇄 후 유지 성공 (position 목표 유지 규칙, 반복 횟수는 사용자 결정) | `[HW-P1B]` |
| G6-F | $d_{eff}$ **와 $r_{cap}$** 의 산정식·실측값·provisional 표시가 YAML (`planner.hand.*`) 과 이 문서에 기록됨 (S4.5) — LEAP PASS, P1b PASS (2026-09-21, 탐색한 자세 — §4.5). 둘 다 provisional | `[SIM-ANY]` |

## 10. 미확정 항목

- TBD-HAND-01, TBD-HAND-04 의 투척 보정 (S7.1 후 — 기하값은 두 손 모두 S4.5 로 provisional 닫힘), `index_mcp_aa_joint` 위치 한계 불일치, 손 프로파일 값 (provisional)
- **P1b 의 폐쇄 속도** — 자세는 2026-09-21 에 다시 정했고, 시각 발동이면 중심에서 1.0 m/s 까지 잡는다 (§4.5). 남은 문제는 $T_{close,e2e}$ 0.28 s 가 **최소 비행시간에 그대로 들어간다**는 것이다 (0.59 s, plan §4.4 S4.4 결과) — 폐쇄 병목은 `thumb_cmc_fe` 의 이동량 1.57 rad (§4.2) 이므로 자세를 다시 찾을 때 $T_{close}$ 를 목적함수에 넣는다. P1b $d_{eff}$ 는 스캔 상한에 걸린 하한값이다
- **S4.5 시험 도구는 저장소에 없다** — 접촉 스캔·fly-in·자세 탐색 스크립트는 작업용으로만 있었다. 자세나 공을 바꾸면 다시 필요하므로 `rtc_tools` 편입이 후속 항목이다
- 닫힘: TBD-HAND-05 (S4.1, 사용자 제공 자세)
- TBD-HAND-03 지문 부호: sim 발행 부호 재확인 후 정규화 필요 여부 (S7.3)
- 유지 단계 position 목표 규칙 (S7.1)
- 닫힘: TBD-HAND-02 (D-11, 종단 간 측정), TBD-SIM-01 (G6-4), TBD-RTC-17 (G6-6)
