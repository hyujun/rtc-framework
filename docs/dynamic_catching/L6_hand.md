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
| G6-3 | P1b 명령 가능한 토크 한계 | 닫힘 — 관절당 `max_torque` 3.0 N·m (로봇 config YAML = URDF effort), MJCF forcerange ±3 N·m. 실기 명령은 position 만이므로 토크 한계를 직접 명령할 수 없다 (§4.4) |
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

**catch frame 과 폐쇄 체인.** catch frame 은 손바닥(`l_palm_link` / `palm_lower`) 에 붙는 모델 빌더 추가 frame 이고 부모·offset·자세는 로봇 config YAML 로 연다 `[확정 D-10, D-17]` (plan §10). 손바닥은 폐쇄 체인 루프의 **상류**이므로 catch frame FK·Jacobian 은 팔 관절만으로 정해지며, 폐쇄 체인 사영이 `held` (NUM-5) 여도 영향을 받지 않는다. catch frame 위치 초기 제안값(포켓 중심)은 S4.1 손 프로파일의 preshape 자세에서 손가락 끝 중심으로 산출한다 (plan §10, S2.3).

### 4.5 포켓 유효 깊이 $d_{eff}$ 산정 `[권장]`

1. 기하 추정: preshape 자세에서 FK로 손바닥 평면과 폐쇄 시 손가락이 형성하는 차단선 사이 거리를 접근축(catch frame +z) 방향으로 측정한다. 공 반지름을 뺀다.
2. 실험 보정: 시뮬레이션과 실기에서 저속 투척으로 "폐쇄 늦음" 경계를 찾는다.
3. 반발 허용 여부(L3 §4.5)에 따라 $d$ 또는 $d(1+1/e)$를 쓴다.

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

**L6a = S4 (S5 이전, go/no-go)**

- **L6.1** (S4.1) 손 프로파일 YAML 두 벌(P1b 10 DoF, LEAP 16 DoF) 초안 — provisional.
- **L6.2** (S4.2) $T_{close}$ 식별 도구: 손 device slot 에 계단 position 명령 반복(≥20회) + CSV → $\rho(t)$, $T_{close,e2e}(\eta)$ 분포(평균, 최대, 99%) 산출 (sim, 두 손).
- **L6.3** (S4.4) 산출값으로 §4.1 의 $\Vert v\Vert_{\max}$ 를 계산해 목표 투척 속도(D-12, D-18 지도)와 대조하고, γ 창이 비면 사용자에게 보고하고 목표를 낮춘다.
- **L6.5** (S4.3) 실기 $T_{close,tot}$ 종단 간 식별 `[HW-P1B]`. 실기 접근 가능하면 S4 에 포함하고, 아니면 S10.

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

## 10. 미확정 항목

- TBD-HAND-01, TBD-HAND-04, TBD-HAND-05 (`index_mcp_aa_joint` 위치 한계 불일치 포함), 손 프로파일 값 (provisional)
- TBD-HAND-03 지문 부호: sim 발행 부호 재확인 후 정규화 필요 여부 (S7.3)
- 유지 단계 position 목표 규칙 (S7.1)
- 닫힘: TBD-HAND-02 (D-11, 종단 간 측정), TBD-SIM-01 (G6-4), TBD-RTC-17 (G6-6)
