# dynamic_catching — 전체 구현 계획 (living document)

- 상태: **pre-S10 완료 (2026-09-29) — S10 (실기) 착수 대기**. S10 추적 issue 는 [#613](https://github.com/hyujun/rtc-framework/issues/613) 이다. R1–R7 과 리뷰 후속 (PR #612, issue #606–#611 · 리뷰 후속 결정 D-1·D-2 — §1 의 D-1·D-2 와 다른 번호 계열) 의 결과는 §4.4 pre-S10, S10 으로 넘긴 것은 §4.4 S10 "pre-S10 에서 이월". 단계별 상태·PR·게이트 결과는 §4.3 표가, 상세·수치는 §4.4 단계별 절이 SSoT 다 (이 줄은 요약만 둔다). 승인이 막는 단계는 승인 전에 착수하지 않는다 (§4.1)
- 최종 갱신: 2026-09-29. 이날 문서를 제자리에서 압축했다 — 절 번호와 식별자는 바꾸지 않았다. 압축 전 전문은 `git show b0ea0996:docs/dynamic_catching/IMPLEMENTATION_PLAN.md`, 갱신 이력은 `git log -- docs/dynamic_catching/IMPLEMENTATION_PLAN.md`
- Epic: [#537](https://github.com/hyujun/rtc-framework/issues/537)
- 수명: 구현 완료 시 prune 한다. 이 문서는 **전체 계획과 결정의 SSoT** 이고, 단계별 상세 작업(sub-plan)은 각 에이전트의 private plan 에서 관리한다 ([AGENTS.md](../../AGENTS.md) §6.6).
- 저장 위치: [handoff.md](../../agent_docs/handoff.md) §5 는 plan 파일을 커밋하지 않는다. 이 문서는 같은 폴더의 설계 문서(v0.5)와 함께 리뷰되어야 하는 결정 로그라서 설계 문서와 같은 브랜치에 커밋한다 — 사용자 결정 P-2 (§7.1). cross-tool 인계면은 여전히 issue #537 이다.
- 갱신 규칙: 결정·단계 상태·게이트 결과가 바뀔 때마다 이 문서를 먼저 고친다. 이 문서가 구체화되면 같은 폴더의 설계 문서를 이 문서에 맞춰 갱신·동기화한다. 설계 문서(`CATCHING_MASTER.md`, `L0_core.md` … `L8_bringup.md`)와 충돌하면 **이 문서의 결정이 우선**한다. 틀린 내용은 옆에 날짜 붙은 정정을 덧붙이지 않고 **그 자리에서 고쳐 쓴다** (이력은 git 에 있다). 표 셀에는 경위가 아니라 판정을 적는다.

## 1. 결정 로그

ID 는 한 번만 정의한다. 사용자 결정은 D-·C-·P- 로, 착수 전 합의한 세부 결정은 A- 로 적는다 (§7.1).

| ID | 결정 | 상태 | 근거 요약 |
|---|---|---|---|
| D-1 | 코드 배치: 수치 코어는 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`), 축 정렬 오차는 `rtc_math` se3, CLIK 확장은 `rtc_tsid`, 바인딩·YAML·launch·PointCloud2 파서는 `integrated_bringup` | **확정** | 코어가 robot-agnostic 판정([design-principles.md](../../agent_docs/design-principles.md))을 통과한다. `compliance`·`grasp`·`inference` 코어와 같은 선례. 새 패키지를 만들지 않는다 |
| D-2 | 시간 규약: (1) t_c·t_cmd 는 공의 **물리 시각** 단일 정의, 판정별 비교 대상 '지금' 고정(§3). (2) 내부 표현은 절대 steady ns, 상대시각은 수치 코어 경계에서만. (3) nrt 수신 시 `t_ref_steady = recv_steady − (recv_wall − stamp)` 를 1회 계산한다. `header.stamp` 는 **물리 샘플 시각 복원에만** 쓰고, freshness·stale 은 `recv_steady` 로만 판정한다 (§3.1) | **확정** — (3) 은 E-1 승인 (2026-09-19, S0.6), invariants.md 에 기록된 예외 | t_c·t_cmd 가 stamp 에서 파생되므로 deadline 판정 (COMMITTED·CLOSING·DECEL) 이 wall clock 점프에 노출된다 — [invariants.md](../../agent_docs/invariants.md) §Clock 시간축 규칙의 기록된 예외로 허용한다. 기각한 대안 `t_ref_steady = recv_steady` 는 예외가 필요 없지만 전송·추정 지연(수~수십 ms)을 T_arm·T_freeze 예산에 그대로 넣는다 |
| D-3 | sim 시간축: wall clock 유지 + 시행별 clock 오차 게이트 (§5) | **채택 — 판정식 재정의, 검증 후 재검토** | D-2 변환이 실기와 같은 경로로 동작. `/clock` 방식은 RT 루프에 sim 전용 시간 원천이 필요해 D-2 와 충돌. 2026-09-22 부터 sim 공 lane 의 stamp 는 **발사 순간을 기준으로 sim 시간축을 wall 에 얹은 값** (stepper wake 지터만 제거 — `rtc_mujoco_sim` README §Projectile Ball stamp) 이고, D-3 의 위상 오차 정의는 그대로다 |
| D-4 | vision 입력: ball_perception 의 실제 PointCloud2 레이아웃 채택, 필드 이름으로 파싱, `generation`·`validity`·`snapshot_sequence` 사용, NaN 공분산 = 모름. 구독 QoS 는 `KEEP_LAST` depth **1** 고정 (ARCH-6), reliability 만 S3.4 에서 실측해 정한다 | **확정** | 실제 발행기 존재. 설계 문서 §5 의 372 B·`t`·`cov` 가정 폐기. depth 는 ARCH-6 의 강제 사항이라 TBD 대상이 아니다 |
| D-5 | CLIK: `rtc::tsid::ClikReferenceGenerator` 를 옵션(기본 off)으로 확장. 옵션을 넣기 전에 기존 동작 golden-vector 회귀를 먼저 만든다 (S2.2a) | **확정** | P5 일반화. off 시 기존 출력 bit-identical, 기존 assertion 수정 필요 시 즉시 E-6 |
| D-6 | CLIK 오차·J 를 명령값 q_c 에서 평가하는 옵션 추가 | **확정** | 측정 q 평가는 servo 지연을 루프에 품고 선행 보상과 이중 보상. 실추종은 `TRACK_ERR` 로 별도 감시, 재앵커 시점 규정 필수 |
| D-7 | 계획기 스레드는 **기존 MPC 스레드와 같은 생성 방식**으로 만들고 기능만 planner 로 한다 | **확정** — D-7b~d 확정. D-7a 측정은 2026-09-23 사용자 결정으로 생략, 초기값 FIFO 유지 (§6, §7.2). 배치는 E-7 승인 (S6) | nrt_callback executor 는 단일 스레드라 계획 계산(수십 ms)을 올리면 궤적 수신·서비스가 막힌다 |
| D-8 | γ derate 는 v1 에서 제외. 실행 중 포화 → COMMITTED 전 RETREAT, 이후 ABORT_SAFE. S8 에서 포화 빈도 측정 후 재설계안 도입 여부 결정 | **확정** | 참조 구현 probe: Frozen 분기 γ_min 미보장, 완화 분기 무효, 기본 램프가 가속 피크를 키움, 램프가 t_c 초과 가능 |
| D-9 | `ComputeGammaWindow` 의 TCP 속도 = η_v · `reference.v_max` (0 < η_v ≤ 1). 마스터 §6 교차제약을 이 식으로 수정 | **확정** | 계획이 한계 끝을 쓰면 실행 중 예측 변화로 포화. D-8 로 derate 가 빠져 유일한 완충 |
| D-10 | catch frame: 후보 p1b `l_palm_link` +z, iiwa7_leap `palm_lower` −z (FK 도출) + 포켓 중심 offset. `rtc_urdf_bridge` 모델 빌더가 로봇 config 의 `urdf.extra_frames.<name>` 을 **full 모델에** frame 으로 추가하고, 파생 모델(sub·tree·actuated)은 그것을 상속한다 (§10) | **확정** (값은 D-17) | CLIK 은 모델 frame id 만 받아 binding 로컬 offset 은 CLIK 까지 못 간다. 파생 모델은 모두 full 모델에서 `buildReducedModel` 로 만들어지므로 한 곳에서 추가하면 된다 |
| D-11 | 손 명령 포트 추상화 폐기. 손은 `ControllerOutput` 의 손 device slot 에 직접 기록. T_link 분리 측정 대신 종단 간 T_close,tot 실측 | **확정** | P1b·LEAP 모두 이미 device group. `udp_hand_node` 는 명령 stamp 를 읽지 않는다 |
| D-12 | 사용자 제공 값: 공 사양, 실기 T_close,tot 측정 시점, 성공률 하한·시행 수, **P1b 손 관절 운용 토크 한계의 권위 출처** (§7.3). 투척 목표는 D-18, 관절 가속 한계는 D-16, catch frame 은 D-17 로 대체 | **방식 확정, 값 일부 확정** — 성공률 floor **0.35** (2026-09-24 회신, 2026-09-26 동결 D-S8-16) · 시행 수 n_valid **200** (D-S8-3). 실기 T_close,tot 측정은 S10 (S4.3 이월, 2026-09-20). **대기 → S10 #613**: 공 사양 (sim 은 tennis provisional) · 손 토크 권위 출처 (sim 3.0 · 실기 1.5 N·m provisional, 그때까지 G7-B3 토크 비교 NOT_EVALUATED) | 추측 금지. 임시값은 YAML 에 provisional 표시, 값에 의존하는 게이트는 NOT_EVALUATED (§4.1) |
| D-13 | E-STOP·fault 정책 (E-8) — 하위 결정 D-S9-A~L (§4.4 S9): E-STOP 은 어느 단계든 CM 의 측정 자세 hold (손 포함, 공을 놓는 것이 확정 동작) → 비무장 `IDLE`, 해제 뒤 자동 재개 없음; `FAULT` 는 컨트롤러 소유로 global E-STOP 에 승격하지 않고, 원인에 운동 기한 추가·`n_qp` 시행 단위·정지 전 reset 거부 | **확정** (2026-09-27 사용자, #537 5854178226 → 5854969250) — S9a (PR #589 → `f4bc14ed`) · S9b (PR #590 → `18507711`) 머지 | 컨트롤러는 E-STOP 중 출력에 관여할 수 없고 (CM 치환), "재차 치명" 경로·정지 미추종 감시가 코드에 없었다. 대상이 공이라 놓쳐도 위험이 없고, CM 정지 경로는 실기 직전에 바꾸지 않는다 |
| D-14 | 공 발사 API: (p0, v0, ω) 명시 srv 를 `rtc_msgs` 에 추가 (Adding a New Message Type, PROC-3) | **확정** — E-3 승인 (2026-09-19, S0.8) | 파라미터 설정 + Trigger 는 경합·재현성 약함. E-3 판단은 §7.1 |
| D-15 | vision 예측 사양(지평·간격·점 수·발행률)은 **포구 제어기가 요구 사양을 정하고**, sim 에서는 공 투척 설정과 ball_perception sim profile 을 그 요구에 맞춰 설정한다. 제어기는 수신 궤적의 지평이 요구보다 짧으면 계획 후보에서 제외·진단한다 | **확정** — sim profile **1.0 s / 0.05 s / 20 점 / ≤ 30 Hz** (S3.6, 2026-09-22, provisional), 설정됨 (2026-09-22 사용자): 로봇별 사본 `ball_perception_sim_profile.json` — 위치는 [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) MD-18 (2026-09-30) 로 개정: profile 은 ball_perception 저장소의 `ball_perception_sim/config/sim_profile.catching.json` 이 소유하고, 이 사본은 E0-F04 ([#647](https://github.com/hyujun/rtc-framework/issues/647)) 에서 제거했다 | 값은 S3.6 이 기구학 reachable 창 (D-27) 과 T_det 재실측으로 낸 H_req 0.99 s 에서 왔다 — 종전 0.8 s / 16 점을 버린 이유와 **t = 0 없는** 예측점 산식은 §4.4 S3.6 결과 |
| D-16 | 관절 가속 한계는 **토크 한계에서 도출**한다 (§9). 시뮬레이션 추정은 교차 검증용. YAML 의 기존 `max_acceleration` 값은 쓰지 않는다 | **확정** — 퇴화 분기·가중·오라클은 §9 | 가속 데이터 없음, 토크 데이터 있음. 기존 `max_acceleration` (5.0 rad/s²) 은 CM 이 읽기만 하고 어떤 컨트롤러도 쓰지 않는 placeholder 였다 — 로봇 YAML 의 그 키는 E1-F11 (#698) 에서 지웠다 |
| D-17 | catch frame 의 부모 frame·위치 offset·자세는 **YAML 로 열어 둔다**. 초기값은 S2.3a(축)·S2.3b(위치)에서 제안하고, 사용자가 sim 에서 확인해 갱신한다 (§10). 값은 모델 빌드 시 읽히므로 바꾸면 컨트롤러를 다시 configure 해야 한다 | **확정** | 사용자 결정 |
| D-18 | 투척 목표는 **arm manipulability 기반 포구 가능성(catchability)** 으로 정한다. 발사 영역 (arm base frame 기준 수평 거리 √(x²+y²) = 4 m 의 원호 — 좌우 투척 포함, world z 1.5–2.0 m, **비행시간 T_f ≥ 1.0 s** — 사용자 2026-09-19) 에서 출발한 궤적 위 포구 후보마다, 손바닥 +z 가 공 진행 방향을 마주보는 자세(a_d = −v̂)의 IK 해에서 manipulability 를 재고, threshold 이상인 후보가 있으면 잡을 수 있는 공, 없으면 포기. 이 판정으로 투척 속도·각도 범위를 정한다. threshold 초기값 0.1 (provisional; 출하값은 로봇별 — `ur5e_p1b` 0.1 · `iiwa7_leap` 0.174, §11) | **확정** — 정의 세부는 §11. sim 발사 영역의 개정 (릴리스 높이 0.2–0.5 m 등) 은 §7.3 "D-18 개정" | 사용자 결정 |
| D-19 | 단계마다 **`demo_controller_gui` 갱신과 `plot_rtc_log` 로 CSV 플롯을 구현·확인**한다. 각 단계 게이트에 GUI 확인과 plot 회귀 테스트를 포함한다. S0 (코드 없음)·S1 (ROS·GUI 비의존 순수 코어) 은 면제한다 (§13) | **확정** | 사용자 결정. 면제 근거는 §13 |
| D-20 | 포구 상태는 `rtc_msgs` 에 **새 상태 메시지**를 추가해 GUI 로 보낸다 (`WbcState`·`GraspState` 선례, `PublishRole` 을 늘리지 않는 controller-owned `SeqLock<T>` 패턴). **S5 에서 S5~S9 필드 superset 을 한 번에 동결**하고 이후 단계는 값만 채운다. 모든 `Compute()` tick 에서 Store (PROC-7) | **확정** — E-3 승인 (2026-09-19, S0.8) | 필드를 단계마다 더하면 매번 `rtc_msgs` 변경·PROC-3 전체 빌드·테스트가 반복된다 |
| D-21 | SeqLock 소비 계약: RT 소비자는 매 tick `Load()` 를 무조건 한 번 하고, 새 스냅샷 여부는 **payload 안의** `snapshot_sequence`·provenance token (D-22) 으로 판정한다. `SeqLock::sequence()` 와 `Load()` 를 따로 읽어 짝짓지 않는다. 스냅샷 용량은 컴파일타임 상수 `kCap` (S1.2) | **확정** (§7.4 F4) | 두 호출 사이에 writer 가 끼면 옛 payload 와 새 sequence 가 짝지어진다. repo 의 다른 SeqLock 소비자도 무조건 `Load()` 관용구다. 최악 재시도 시간은 G1-C 로 측정한다 |
| D-22 | provenance token: 궤적 스냅샷·공분산 버퍼·`PlanSnapshot` 은 같은 identity `{activation_generation, generation, snapshot_sequence, traj_recv_ns}` 를 싣고, `PlanSnapshot` 은 계산 기준 `{rt_iteration, rt_state_ns}` 와 `publish_ns` 를 더한다. 계획기는 계산 시작과 게시 직전에 최신 token 을 다시 보고, 대체되었거나 짝이 안 맞는 결과(궤적 N ↔ 공분산 N−1 포함)는 버린다. RT 소비자는 activation·generation 일치, `snapshot_sequence` 단조, source 나이·state 나이 상한을 fail-closed 로 검사한다 | **확정** (§7.4 F5) | 궤적·공분산이 다른 버퍼로 가서 N/N−1 혼합을 막을 수단이 없었다. MPC 선례(`MPCSolution::timestamp_ns`)보다 넓은 이유: 포구는 절대 시각 판정이라 출처 나이가 곧 안전 조건이다 |
| D-23 | activation 경계: vision ingress 와 계획기는 base 의 `ActivationGeneration()` 을 스냅샷에 싣고, RT 소비는 `IsCurrentGeneration()` 이 아니면 무효로 본다. `PeriodicRtThread::Pause()` 는 진행 중 iteration 을 멈추지 않으므로 quiescence 대신 generation 으로 판정한다. lifecycle·E-STOP 훅은 atomic 요청(또는 epoch)만 갱신하고, plan·궤적·공분산·손·FSM·타이머 무효화는 **RT tick 이 유일 writer** 로 수행한다 | **확정** (§7.4 F6) | lifecycle 은 publisher 만 게이트하므로 컨트롤러 소유 구독은 비활성 중에도 산다. base target mailbox 의 generation 게이트는 그 mailbox 에만 적용된다 (`rt_controller_interface.hpp`) |
| D-24 | 지문 센서 freshness: (a) `rtc_base` `DeviceState` 센서 lane 에 `recv_steady_ns`·`sequence`·`valid` 를 추가하고 backend 3종이 채워 `ControllerState` 로 전달 (PROC-3, P5 — 같은 gap 이 grasp 에도 있다), (b) 포구 컨트롤러 소유 mailbox 로 센서 토픽을 따로 구독 | **확정 (2026-09-22 사용자): (a)** — 배선은 S5.2e, PROC-3 전체 회귀 포함 (§7.3) | 종전 `last_state_ns_` 는 관절 상태 콜백에서만 갱신됐다. 관절이 fresh 한 채 센서만 멈추면 옛 힘을 새 접촉으로 볼 수 있다 |
| D-25 | S1.9 구현에서 확정한 두 가지: (1) **L3 §4.2 의 roll manipulability 최대화 제외를 번복**한다 — 영공간 항 $k_w\nabla\log w_5$ 로 구현했고 seed 규정·결정성 요구는 그대로다 (국소 최대이지 전역 roll 탐색이 아니다). 최대화 대상은 게이트 정의와 무관하게 항상 $w_5$ 다 (C-3 비교가 공정해진다). (2) **$\rho$ 는 $J$ 와 잔차 양쪽에 곱하는 과제 가중**이다 — 잔차에만 곱한 v0.5 식은 차원이 맞지 않았다 | **확정** (2026-09-20 사용자 결정, S1.9) | (1) 여유 자유도를 seed 의 우연 대신 조건수에 쓴다. `planner.ik.k_manip` = 0 이면 번복 전 동작이라 기준선이자 fallback 이다. (2) 양쪽 가중이어야 $\rho$ [m/rad] 가 단위 변환이 되고 $\lambda^2=0$ 에서 회전 행이 1 스텝에 정렬된다 |
| D-26 | 포구 자세 IK 의 **과제 스텝을 제약 QP 로** 바꾼다 (L3 §4.2 `[확정 D-7d]` 의 `DifferentialIk` 전면 재사용을 번복). $\dot q_{clik}$ 은 관절 한계·스텝 제한을 부등식 제약으로 갖는 QP 가 풀고 (ProxQP, `rtc_tsid::QPSolverWrapper`), $\dot q_n=N\dot q_{sec}$ 는 QP **밖에서** 더한다 — $\dot q_d=\dot q_{clik}+\dot q_n$. `DifferentialIk` 는 $N$ 을 만드는 용도로 남는다 | **확정** (2026-09-20 사용자 결정, A·B 비교 측정 후 — §4.4 S1.9 결과) | $\mu=10^{-4}$ 에서 QP 가 DLS 대비 수락률·잔차·한계 활성 비율에서 근소 우위, 시간 +22%. 기각: manipulability 항을 QP cost 에 넣는 초안 (사용자 지시) — 영공간 속도는 CLIK 출력에 더하는 것이고 cost 에 섞으면 우선순위가 soft 해진다. 비용: rtc_controllers→rtc_tsid production 의존 신설 (순환 없음, ARCH-2 아님 — rtc_controller_manager 까지 ProxSuite 전이), rtc_tsid 에 `ResetWarmStart()` 신설 (후보 간 결정성) |
| D-27 | **분석과 실행의 기준을 분리한다.** catchability 분석 (S3.5a·S3.5b·S4.4) 은 **정교하게** 유지한다 — 게이트를 느슨하게 하거나 층·판정을 지우지 않는다. **실행 (런타임)** 은 기구학적으로 reachable 한 후보면 **도전한다**: 성능 게이트 (γ 창·도달시간·commit 선행) 탈락은 후보를 *지우는* 필터가 아니라 **우선순위와 진단**이다. 따라서 vision 지평 요구 (S3.6) 는 **기구학 reachable 창**에서 읽는다 | **확정** (2026-09-22 사용자) | 게이트 체인은 낙관적 상한 (S4.4) · 충분조건 (토크 층) · provisional 상수 (`d_eff`·`a_dec`·선행시간) 위에 있어 그 교집합에 목표를 *동결* 하면 잡을 수 있는 투척도 시도하지 못한다 (지도는 우선순위의 근거라 정확해야 한다). 지평이 짧으면 후보가 계획기에 닿지 못한다 (L1 §4.1) — 비용은 예측점 몇 개다 (결정 당시 추정 n 16 → 19; 설정한 profile 은 20 점, `kCap` 40 안 — §4.4 S3.6). **안전 게이트는 판정으로 남는다**: 정지점·작업공간 (§4.9) 은 팔이 어디에 멈추는지의 문제이고, soft 화 경계는 E-8 대상이라 **S6 설계에서 정한다** (§4.4 S6) |

## 1a. Sprint Contract (A-1 승인, 2026-09-19)

Epic 기준 하나와, **각 단계 착수 시 그 단계의 `[SPRINT]` 기준**을 따로 확정한다 (단계 sub-plan 의 `## Spec`).

```
[SPRINT] 1) iiwa7_leap·ur5e_p1b MuJoCo 에서 ball_perception PointCloud2 예측만을 입력으로,
            동결한 목표 투척 분포(D-18 지도, S3.5b)에서 포구 성공률의 Wilson 95% 하한 ≥ floor(D-12),
            로봇별 게이트 (G8-D·G8-D2). 무효 시행을 포함한 전체 발사 수와 무효 사유를 함께 보고         2) 전 RT 경로 할당 0·noexcept·RT-1~10 준수, 기존 컨트롤러 테스트 assertion 무수정 green
         3) 설계 문서 v0.5 가 코드와 일치하고, 이 문서에 단계별 게이트 결과가 기록됨
```

**floor 와 시행 수.** 기준 1 의 floor 는 **0.35**, 시행 수는 n_valid 200 이다 (D-S8-3). 사용자가 2026-09-24 에 S8-B 튜닝 세트 p̂ 94/200 = 0.47 (Wilson [0.40, 0.54]) 을 보고 정했고 (#537 5808613906 의 0.5 는 그 전 provisional 값), 2026-09-26 S8-E 계획 확정 때 동결했다 (D-S8-16, #537 5841651825) — S8-E 데이터를 본 뒤에는 바꾸지 않는다. 시행 수는 S0.9 검정력 표로 먼저 정했다. 실기(S10)는 Epic 기준 밖이며 S10 착수 시 별도 기준을 세운다.

**기준 1 의 판정 정의 (2026-09-24 S8 결정, §4.4 S8).** ① **포구 성공은 sim truth 로 판정한다** — HOLD 끝부터 대기 자세 release 까지 공이 손에 있으면 성공이다. 슈퍼바이저 판정 (지문 힘만 보므로 링크·손바닥 위 공을 Missed 로 읽는다, S7) 은 truth 대비 혼동행렬로 병기한다. ② **무효는 rig 실패만** (srv 거부·lane drop·sim stall·미발사) 이고 "plan 없음·abort" 는 실패로 센다. 무효를 실패로 센 ITT 하한을 병기한다. D-3 clock 위상 오차는 무효 사유가 아니라 공변량이다 (D-S8-4 (c), §5). ③ **floor 는 사용 목적에서 사용자가 사전 고정**한다 — floor **0.35**, n_valid 200, 통과 = 성공 ≥ 84/200 (p̂ ≥ 0.42), 검정력 0.8 은 참 p ≈ 0.45 부터 (참 p 0.47 에서 0.93, 0.40 에서 0.31). beanbag arm·G8-D2 도 같은 0.35 (D-S8-16). 무효의 기계 판정·n_valid 보충 규칙은 §4.4 D-S8-16 ①·①b.

**최종 판정 — 기준 1 은 로봇별이므로 `ur5e_p1b` 충족 · `iiwa7_leap` 미충족.** 수치는 §4.4 S8 (게이트 표·S8-E 결과) 과 §4.4 pre-S10 이 갖는다.

- G8-D (`ur5e_p1b` tennis): **PASS 180/200 · 하한 0.8506** — 현 출하 profile 재평가 값 (리뷰 후속 결정 D-2, 2026-09-29; §4.4 S8 게이트 표). 그 전 판정도 나란히 둔다: S8-E 93/200 · 하한 0.3972 PASS (D-S8-17 host 부하 unit 재실행 반영), 사전 규칙 원 판정 83/200 · 하한 0.3489 FAIL
- beanbag arm: **PASS**
- G8-D2 (`iiwa7_leap`): **FAIL** — S8-E 83/200 · 하한 0.3489. pre-S10 R6 의 사후 보정 재판정은 원 판정과 나란히 둔다

**G8-D2 는 조건부다 (2026-09-22 사용자 결정, S3.5b 후).** 1차 목표 로봇은 `ur5e_p1b` (G8-D). 규칙: **실측 선행시간에서 `iiwa7_leap` gate 지도가 비어 있지 않으면 G8-D2 를 평가하고, 비어 있으면 `NOT_EVALUATED(선행시간)` 로 그 실측값과 함께 보고한다** (S3.5b 지도는 가정 선행시간 0.24 s 에서 비어 있었다 — 막는 것은 손이 아니라 선행시간, §4.4 S3.5b 결과). 조건은 2026-09-22 에 충족됐다 — 첫 계획 0.215 s 에서 지도가 열렸다 (§4.4 S3.6 "T_det 실측"). 평가 대상 판정은 창이 좁고 L 이 가정값이라 provisional 이다. S8-D 가 첫 계획 p50 0.195 s 에서 지도 176/180 으로 재확인했다. 기준에서 빼지 않는다 — LEAP 은 fly-in 에서 더 강한 손이다 (L6 §4.5). `ur5e_p1b` 의 열림이 기댄 세 가정 (fly-in 등가 `d_eff` · D-16 토크 층 · 다시 고른 대기 자세) 은 S6 착수 조건이었다 (§7.3 "1차 목표 로봇").

**G8-D2 판정 방식** 은 D-S8-16 ③·⑥ 과 D-S8-15 (§4.4 S8) 가 정한다 — S8-D 100 발은 판정에 쓰지 않는다 (D-S8-3 의 데이터 분리).

### S0.9 검정력 표 (2026-09-19, 스크립트 실행값 — 에이전트 산출, 4 셀 독립 재계산으로 확인)

- 정의: Wilson score 95% 하한, 양측 z = 1.96 (문서에 단측·양측 명시가 없어 보수적으로 둔다 — 단측 z = 1.645 면 n 이 준다). K ~ Binomial(n, p) exact
- 표 값: 로봇당 유효 시행 수 n. P(Wilson_LB(K, n) ≥ f) ≥ 0.8 인 최소 n 이다. 검정력은 n 에 대해 톱니 모양이라, **그 뒤 n = 5000 까지 다시 0.8 아래로 떨어지지 않는** 최소 n 을 쓴다 (처음 넘는 n 보다 3–37 크다). p ≤ f 는 불가
- p̂ = p 를 대입한 결정론적 최소 n 은 표본 변동을 무시하므로 기준으로 쓰지 않는다

| 가정 성공률 p \ floor f | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 |
|---|---|---|---|---|---|
| 0.6 | 198 | 불가 | 불가 | 불가 | 불가 |
| 0.7 | 48 | 186 | 불가 | 불가 | 불가 |
| 0.8 | 19 | 44 | 158 | 불가 | 불가 |
| 0.9 | 11 | 18 | 35 | 112 | 불가 |
| 0.95 | 8 | 11 | 21 | 40 | 254 |

- 읽는 법: 성공률 p 인 정책을 n 회 돌리면 80% 이상의 확률로 G8-D·G8-D2 를 통과한다. 예: p 0.9, f 0.7 → 35 회. 90% 검정력은 대체로 +10–30% (같은 셀 43)
- 벽시계: 발사 수 = ⌈n / (1 − 무효율)⌉, 시간 = 발사 수 × t_trial, 로봇마다. 무효율 5 % (§5 제안) 에서 p 0.9·f 0.7 → 37 발사, t_trial 10/30 s 이면 6.2/18.5 분. 최악 셀 p 0.95·f 0.9 → 268 발사, 44.7–134 분. t_trial 은 미측정 가정
- floor 를 가정 성공률에 붙일수록 n 이 급증한다 (p − f = 0.05 면 254, 0.1 이면 112–198, 0.2 면 35–48). D-12 floor 는 이 표와 S8 벽시계 예산으로 정한다
- 규모 비교: §5 의 D-3 제안 200 회 규모면 25 셀 중 24 셀이 80% 검정력을 넘는다 (예외 p 0.95·f 0.9). D-3 시행과 S8 시행은 목적이 다른 별도 시행이다

## 2. 단계 W 결론 요약

코드 대조 결과 (2026-09-19). 확인 기록은 [WORKSPACE_ANALYSIS.md](WORKSPACE_ANALYSIS.md) §3 기록 칸이 갖고, 괄호의 W 번호가 그 행이다. W 번호가 없는 사실은 그 표에 없어 여기가 유일한 기록이다.

- 명령 경로: `RTControllerInterface::Compute` → `ControllerOutput` → CM `DeviceBackend::WriteCommand` (ros2_control 아님). backend 는 실기 UR `ur_driver_native` (vendor `forward_position_controller` 토픽), sim `mujoco_native`, P1b 손 `udp_hand_native` (W4-1·W4-2·W4-5)
- 없음이 확인된 것: UR 지연 보상 (W4-2), speed scaling 노출 (W4-7), `ApplySafetyLayer` 의 production 호출 (W4-3), 독립 IK (W3-11), sim `/clock` (W6-1), 그리고 스트리밍 목표를 받는 컨트롤러, Pinocchio offset frame 추가 기능, sim 공 접촉 truth 출력, sim 명령 지연 주입
- 시간 (W2-3): RT 의 `ControllerState::t_relative_s` 는 실기에서 steady clock 기반, sim 에서 `iteration × dt` (#566). `ControllerState::dt` 는 항상 1/`control_rate` 이고 sim lock-step 에서도 tick 한 번 = sim step 한 번이다 — **#566 (2026-09-23) 이전의 팔+손 sim 측정은 tick 이 최대 2 배로 돌아 `dt` 적분 법칙이 sim 보다 ~1.67× 빨랐다** (S6 의 CLIK 오차·서보 지연·`track_err_abort` 근거·L4 기준 오프셋 포함, 재측정 대상). `header.stamp` staleness 판단 금지
- CLIK (W3-3·W3-6~W3-8): `ClikReferenceGenerator` 는 pose 목표만·LWA 6행 고정·측정 q 에서 e·J 평가·위치∩속도 box·`max_iter` 20 고정이고, 가속 box·feedforward·마스크·상태 노출이 없다. `PinocchioCache` Jacobian 은 LWA 고정. `Manipulability()` 는 damped 6×6 Gram 의 LDLT 곱이라 게이트로 재사용하지 않는다 (§11). production 소비자는 DemoWbc 하나
- 재사용 대상: `rtc::SeqLock`, `rtc::SpscQueue` (W2-2), `rtc::compliance::DifferentialIk` (S1.9 는 영공간 투영 N 만 쓴다 — 과제 스텝은 D-26 이후 QP), `rtc_math` se3 `log3`/`exp3` (W3-9), `QPSolverWrapper`, base 의 `ActivationGeneration()`/`IsCurrentGeneration()`, `thread_layout.yaml` 의 `profiles:`, `repo_scripts/scripts/verify_rt_runtime.sh`
- lifecycle: `PeriodicRtThread::Pause()` 는 요청 플래그 store 뿐이고 pause 게이트는 루프 최상단이라 진행 중 iteration 은 끝까지 돈다. lifecycle 은 publisher 만 활성화·비활성화한다 (D-23)
- 모델: ur5e_p1b 결합 모델 nv 는 full tree **26** (UR5e 6 + P1b 20 revolute), actuated 축약 모델 **16** (팔 6 + 손 10). DemoWbc·CLIK 의 control model 은 actuated 모델이 있으면 그것이므로 ur5e_p1b 의 CLIK nv 는 16 이다. 이전 기록의 22 는 근거가 없다. 로봇 config 의 `urdf.*` 는 rclcpp 파라미터로 읽고 `urdf.sub_models.<name>.*` 처럼 map key 로 파싱한다 (list-of-dict 불가)
- 지문 센서 (W4-6): P1b 실기 `HandSensorState` 250 Hz 와 sim contact-wrench lane (커밋 0fcc1d23, 2026-09-09 이후) 이 같은 finger-on-object 부호다. 변환 지점은 `rtc::grasp::PullContactConfig::force_sign` 하나이고, `rtc_msgs` FingertipSensor 주석은 PR #538 로 고쳐졌다 (2026-09-19). 센서 lane 에는 수신 시각·sequence 가 없었다 (D-24, S5.2e 에서 추가)
- sim 공 (W6-1·W6-3·W6-4 — 서비스·토픽, 항력·Magnus 자체 구현, iiwa7_leap 설정 없음): 공 샘플 발행은 sim time 으로 게이트되고 stamp 는 wall 이다 (2026-09-22 뒤 공 lane stamp 는 D-3). 기존 RTF 는 200 step 구간 평균이고, throttle 기준은 `max_rtf` 가 바뀔 때만 재설정된다 (§5)
- sim 모델: ur5e_p1b sim 의 MJCF 는 형제 저장소 hand-description 의 사본이고 (W6-2) `model_pairs.yaml` 게이트에 ur5e_p1b 쌍이 없다 (§9)
- vision (W5-1·W5-2·W5-7·W6-4 — 필드 레이아웃): ball_perception `sim_estimator_node` 의 예측 궤적 PointCloud2 는 debug 토픽이고 **stable ABI 아님** — 제품 ABI 는 ball_perception E6-F02 로 defer. `frame_id` 는 profile 값 (예시 `world`)
- 참조 구현 테스트 (S1 이식 뒤 삭제 — [README](README.md#삭제된-참조-구현)): l0/l2/l3/l4 + `verify_l3.py` 전부 통과, ASan/UBSan 통과. 테스트 밖 결함: `n > kMaxSamples` 범위 밖 읽기 (`traj_sampler.hpp` `check()` 가 상한 초과를 기록만 하고 계속 인덱싱), NaN 목표 영구 오염, derate 결함 (D-8), 한계 무효 시 `tMinChecked` 가 구분 불가한 0 반환. 이름 변경 (`catching::*`·camelCase → repo 규약) 은 S1.1 이식 때 했다

## 3. 시간 규약 (D-2)

| 판정 | 비교 대상 | 비고 |
|---|---|---|
| 궤적 샘플링, γ 프로파일, 기준 생성 | now_lead = now + T_arm | 팔 명령은 T_arm 뒤 실현 |
| CLOSING→DECEL (= DECEL 진입) | now_lead ≥ t_c | 감속 대상 전환도 팔 명령 |
| 궤적 지평 끝(외삽) 경고 | now_lead | 샘플링 시각 기준 |
| APPROACH→COMMITTED | t_c − now ≤ T_freeze | now = 실제. T_freeze 하한에 T_arm 포함 |
| COMMITTED→CLOSING, 손 Close | now ≥ t_cmd | 손은 선행 보상 없음. 손 Preshape 는 시각 조건이 아니다 — 팔 `wait_pose` 도착이 지시한다(Q4, #537 S7 결정 2026-09-23, `T_pre` 폐기) |
| 접촉 판정 창 [t_cmd, t_c + T_conf] | 실제 | |
| 메시지 stale·나이 | now_steady − recv_steady | repo 시계 규칙 |

타입: `BallTime` (물리 시각, 절대 steady ns), `NowReal`, `NowLead` — 비교는 타입별 오버로드로만. **T_arm ≠ 0 fixture 필수** (T_arm = 0 이면 두 축이 같아져 버그가 숨는다). 매 tick 의 now 는 steady 실측이며 tick 수 × dt 로 계산하지 않는다.

### 3.1 `header.stamp` 사용 계약 (D-2 (3), E-1 승인 2026-09-19)

| 용도 | 쓰는 값 | 비고 |
|---|---|---|
| 물리 샘플 시각 복원 | `t_ref_steady = recv_steady − (recv_wall − stamp)`, 수신 콜백에서 1회 | 이 문서가 요청하는 E-1 예외의 유일한 대상 |
| freshness·stale·watchdog | `now_steady − recv_steady` | stamp 를 쓰지 않는다 (현행 규칙 그대로) |
| 원점 지연 `recv_wall − stamp` | 진단 발행 (분포·점프) | C-2: 양수 쪽 나이 거부는 두지 않는다 |
| 미래 stamp (`recv_wall − stamp < −future_tol`) | 메시지 거부 + 카운터 | 변환 신뢰 불가 판정 (fail-closed), 예외 문구에 포함 |
| 절대 시각 지평 검사 | 마지막 점 `BallTime` 대 now_lead | feasibility 판정이며 deadline 아님. 원점 오차는 지평을 짧게 보이게 하는 fail-closed 방향 |

S0.6 에서 승인된 예외 문구 (2026-09-19, [invariants.md](../../agent_docs/invariants.md) §Clock 시간축 규칙에 기록):

> **기록된 예외 — 원격 예측 궤적의 물리 샘플 시각 (dynamic_catching, D-2).** 수신 콜백은 `t_ref_steady = recv_steady − (recv_wall − stamp)` 를 1회 계산해 원격 예측의 물리 시각축을 steady 로 옮긴다. 다음을 모두 만족할 때만 허용하고 하나라도 깨지면 E-1 이다: ① freshness·stale·watchdog 판정은 `now_steady − recv_steady` 로만 한다, ② 보정항이 음수로 `future_tol` 을 넘으면 메시지를 거부하고 센다, ③ 송·수신이 같은 호스트의 CLOCK_REALTIME 을 공유하거나 PTP 동기가 검증된 경우로 한정한다 (실기는 S10 에서 재확인), ④ 보정항 분포를 진단으로 발행해 점프를 관측할 수 있게 한다. 이 예외는 t_c·t_cmd 같은 deadline 판정을 stamp 에서 파생시키므로, wall clock 점프는 그 판정 오차로 그대로 들어간다. `header.stamp` 로 staleness·E-STOP 을 판단하는 것은 여전히 금지다.

## 4. 단계 계획

각 단계 착수 시 sub-plan 을 private plan 에 만들고, 완료 시 §4.3 표의 상태와 게이트 결과를 갱신한다. 단계 번호는 유지하고, 의존 때문에 쪼갠 부분은 sub-stage 로 나눈다 (S3a/S3b, S4a/S4.4, S2.3a/b 등).

### 4.1 승인·결정 게이트와 NOT_EVALUATED 규칙

| 게이트 | 내용 | 막는 단계 |
|---|---|---|
| S0.6 `[CONCERN] E-1` | §3.1 예외 문구 — **승인 2026-09-19**, invariants.md 에 기록 | (해제) S1.3 의 D-2 변환 함수, S5.2 |
| S0.8 `[CONCERN] E-3` | D-14 (.srv)·D-20 (.msg) 을 한 번에 발화 (§7.1) — **승인 2026-09-19** | (해제) S3.2 발사 srv, S5.4 상태 메시지 |
| S5 착수 전 `[CONCERN] E-8` | P-1 최소 E-STOP 계약 (§4.4 S5) — **승인 2026-09-22**, S5.1 에서 구현 (PR #564) | (해제) S5.1 |
| S6 착수 전 `[CONCERN] E-7` | 전 tier × profile 배치표, tier 4 정책, `catching_on/off` (§6) — **승인 2026-09-23** (결정 J: 새 layout role 없이 `mpc` role 재사용) | (해제) S6.1 |
| D-24 결정 | 지문 센서 freshness 경로 — **(a) 확정 2026-09-22** | (해제) S5.2e, S7.3 |
| D-12 값 | floor·시행 수 (확정, §1 D-12)·공 사양·손 토크 권위 출처 (대기 → S10 #613) | S4.4 판정, S8 (충격 게이트 G7-B3 — S7.3 에서 이월, 2026-09-24; 토크 비교는 권위 출처 확정 전까지 NOT_EVALUATED) |

**NOT_EVALUATED 규칙.** 게이트 항목은 PASS 기준과 산출물을 가진다. 판정에 필요한 값이 아직 없으면 그 항목은 `NOT_EVALUATED(<없는 값>)` 로 기록하고 통과로 세지 않는다. 값이 정해지면 다시 판정한다. provisional 값으로 판정한 통과는 `PASS(provisional)` 로 적고, 값 확정 시 재판정한다.

### 4.2 단계 DAG

```
S0 ─┬─ S1  : S1.1–S1.8            (S1.3 의 D-2 변환 함수는 S0.6 승인 후)
    ├─ S2  : S2.1, S2.2a → S2.2b, S2.3a, S2.5, 마지막에 S2.4
    ├─ S3a : S3.1a, S3.2*, S3.3, S3.4          (* S0.8 승인 후; S3.7·S3.8 은 2026-09-20 결정으로 제외 — §4.4)
    └─ S4a : S4.0 → S4.1, S4.2, S4.5                (S4.3 실기 측정은 2026-09-20 결정으로 S10 — §4.4)

S4.5 ──────────────────────────────► S2.3b (포켓 중심 offset)
S1, S2.1 ──────────────────────────► S1.9 (IK 회전 행이 S2.1 의 축 정렬 회전벡터를 쓴다 — 2026-09-19 사용자 결정)
S0.7, S1.9, S2.3a, S2.3b, S3.2 ────► S3.5a (kinematic catchability 지도)
S3.5a, S4.2, S4.5, S2.5, S1.5, S1.7 ► S4.4 (go/no-go)
S4.4 ─► S3.5b (gate-catchable 지도) ─► S3.6 (vision 요구 사양) ─► D-12 투척 분포 동결 (사용자)
S3.6 ─► S1.2 backfill (n_max ≤ kCap 확인, 초과 시 kCap 상향 후 S1 게이트 재실행)

S1, S2, S3a, S4.4(go), S0.6, S0.8, E-8 승인, D-24 ─► S5
S5, S3.6, E-7 승인 ─► S6 ─► S7 ─► S8 (S3.1b 부하 clock 위상 포함) ─► S9 ─► pre-S10 ─► S10
```

S1 ∥ S2 ∥ S3a ∥ S4a 는 서로 독립이다. S4.0 은 S5 의 컨트롤러 골격 일부를 앞당긴 것이다 (손 계단 명령을 낼 컨트롤러가 없으면 S4.2 를 할 수 없고, `DemoJointController` 는 손 목표를 quintic 궤적으로 보간해 계단 응답을 줄 수 없다).

### 4.3 단계 상태

행마다 상태·날짜·PR → 커밋과 게이트 판정만 둔다. 측정값·경위·이월 항목의 내용은 각 단계의 §4.4 절이 갖는다.

| 단계 | 상태 | 게이트 결과 |
|---|---|---|
| S0 결정·문서 v0.5·계약 | 완료 (2026-09-19) | S0.2 W 기록 완료 · S0.3 설계 문서 12개 v0.5 동기화 (`validate_docs` clean) · 정합화 개정 (§7.4) · 승인 #537 · S0.7 필요 지평 0.46–0.86 s (S0.7 의 R2 가 지배)·`kCap` 40 · S0.9 (§1a) — §4.4 S0 결과 |
| S1 순수 수치 코어 | S1.1~S1.8 완료 (2026-09-19, PR #541) · S1.9 완료 (2026-09-20, S2.1 뒤) | 이식·회귀·RT·시간·검증기 PASS · S1.8 PASS (G7-C 임계 NOT_EVALUATED) · S1.9 PASS · backfill PASS (S3.6: `n_max` 20 ≤ `kCap` 40) — §4.4 S1 결과 |
| S2 기존 rtc_* 일반화 | 완료 (2026-09-20, PR #545~#549 + 마감 PR) · S2.3b 완료 (2026-09-21) | se3·동등성·CLIK·extra frame PASS · 가속 도출 PASS(provisional) · G5-C 예산 NOT_EVALUATED (S5 에서 판정) · S2.3b `catch_frame.xyz` 확정·frame 규약 PASS — §4.4 S2 결과 |
| S3a 시뮬레이션 기반 | 완료 (2026-09-20, PR #553) | e2e·frame·PROC-3·GUI·plot PASS · D-3 무부하 NOT_EVALUATED (§5.1, 재판정은 §5) · S3.7·S3.8 범위 밖 (결정 B·C) — §4.4 S3a 결과 |
| S4a 손 타이밍 측정 | S4.0·S4.1·S4.2·S4.5 완료 (2026-09-21, 브랜치 `feat/s4a-hand-timing`) · S4.3 은 S10 으로 이월 | 결정 Q1~Q10 · $T_{close,e2e}$ P1b 280.5 ms · LEAP 103.7 ms · $r_{cap}$ / $d_{eff}$ LEAP 31.0 / 80 mm, P1b 24 / ≥ 95 mm — §4.4 S4a |
| S3.5a kinematic catchability 지도 | 완료 (2026-09-21) | PASS(provisional) · 수락 투척 `ur5e_p1b` 1418/3402 · `iiwa7_leap` 1499/3402 (로봇별 문턱 0.174 로는 1158) · G3-I 런타임 반쪽 NOT_EVALUATED (S6.2) — §11 지도 결과 |
| S4.4 go/no-go | 완료 (2026-09-22) — **조건부 go** (사용자) | 현 씬 γ 창 빔 · 조건 셋 아래 `iiwa7_leap` 39 · `ur5e_p1b` 11 / 3808 `PASS(provisional)` · D-16·D-18 개정 `[제안]` (§7.3) — §4.4 S4.4 결과 |
| S3.5b gate 지도 | 완료 (2026-09-22) | `ur5e_p1b` PASS(provisional) 590 / 2835 · `iiwa7_leap` 빔 (선행시간, §1a) · rollout `NOT_EVALUATED(S6)` · q̇ᵘ 동치 `NOT_EVALUATED(S6.2)` — §4.4 S3.5b 결과 |
| S3.6 vision 사양 | 완료 (2026-09-22) | PASS(provisional) · H_req 0.99 s → 1.0 s / 20 점, `n_max` 20 · `io.horizon_min` 0.51 s (S0.7 의 R1 하한) · `io.n_min` 12 · `io.t_stale`·`io.future_tol` [제안] — §4.4 S3.6 결과·T_det 재실측 |
| S5 포구 컨트롤러 골격·입력·추종 | 완료 (2026-09-23, PR #564 → `4c0fb751`) | S5.1 P-1 (a)~(d) · A-S5-1 · TSAN clean · S5.2e D-24 (a) (PROC-3 전체 5518/0) · S5.2 G1-A~C·E·F·H·I·J green, A-S5-4, ASan/UBSan clean · S5.3 G5-A · G5-B · G5-C · G5-E · G7-H (d) PASS · S5.4 `rtc_msgs/CatchingState` 동결 (G8-H 구조) · S5.5 PASS · `/code-review` 9 건 (A-S5-13, A-S5-14~16) · 남은 게이트 G5-C2 — §4.4 S5 |
| S6 계획기 스레드 | 완료 (2026-09-23, PR #568 → `9bf404a8`) | E-7 결정 J · S6-A (A-S5-12) · S6-B · S6-C (G3-C PASS) · S6-C2 (결정 K) · **R-1** (§7.3 E-7 확인 항목 — pre-S10 R1–R7 과 다른 계열) PASS, policy `NOT_EVALUATED(EPERM)` · #350 배선 수정 · PROC-3 5711 / 0 · #537 결정 ①–⑦ (5789859038 · 5793765020; #566 → PR #567) · 결정 ⑦ 이월 → S8 · S6-D 생략 (`s6d.sh` #537 5793878042) — §4.4 S6 |
| S7 손 시퀀서·슈퍼바이저 | 완료 (2026-09-24, PR #571 → `c61fd32a`) | G6-A · G7-A · G7-B · G7-D · G7-G · G8-A2 PASS · G7-E 기록 · 포획 0/25 · `sat_ticks` 60 (provisional) · C-20 · C-12 · G7-B3 이월 → S8 · 토크 NOT_EVALUATED (D-12) · 리뷰 Critical 0, 6 건 수정 · 이월 #537 5804113629 — §4.4 S7 |
| S8 sim 통합 평가 | 완료 (2026-09-27) — S8-A · S8-B (PR #575 → `ac34893a`, #574) · S8-C (PR #579 → `126e3513`) · S8-D (PR #581) · S8-E (PR #582 → `9e7967b5`) · S8-F-1 (PR #583 → `3278963a`) · S8-G (PR #584 → `769f99e9`) · S8-H · S8-I · S8-I-2. 후속 ②①④ 종결, ③ 미진행 (#537 5846528236) | G8-D PASS · beanbag PASS · G8-D2 FAIL (§1a) · G8-B FAIL (pre-S10 재정의 뒤 PASS) · G8-C PASS · G8-C2 FAIL · G8-E PASS (sim) · G8-H FAIL · G7-B3 기록 · S3.1b 충족 · S8-F 게이트 밖 · 결정 D-S8-1~12 + C-25 (#537 5804952754 → 5807767896), D-S8-3, D-S8-13, D-S8-16, D-S8-17, D-S8-18, D-S8-19, D-S8-20 · 대조 C-1~C-24 — §4.4 S8 |
| S9 E-STOP·fault 정책 (D-13) | 완료 (2026-09-27) — S9a PR #589 → `f4bc14ed` · S9b PR #590 → `18507711` (CM 발견 #588) | 정책 PASS (기존 assertion 무수정) · G8-H 9/9 · S9b `/security-review` Critical 0 · §13 S9 PASS — §4.4 S9 |
| pre-S10 S10 착수 전 잔여 | 완료 (2026-09-29) — R1 PR #596 → `70057aef` · R2 PR #597 → `1307576f` (#588 닫힘) · R3 PR #599 → `6b6dcb96` · R4 PR #603 → `1b64a9f2` · R6 (코드 변경 없음) · R7 issue #600–#602 (닫음: PR #616 → `6085499a` · #617 → `a4f187c7` · #618 → `b6da7384`) · R5 PR #605 → `5abfbeab` · #615 → `9f020ef8` · 리뷰 후속 PR #612 (#606–#611, 결정 D-1·D-2) | §4.4 pre-S10 |
| S10 실기 단계 도입 | 대기 (pre-S10 뒤) — 추적 issue #613 | — (§4.4 S10) |

### 4.4 단계별 작업과 게이트

게이트 표의 "판정 입력" 은 그 항목을 판정하는 데 필요한 값과 그 출처 단계다. 표에 없는 입력을 쓰는 게이트는 없다.

#### S0 결정·문서 v0.5·계약 (코드 없음)

**상태: 완료 (2026-09-19).** 게이트 전부 충족 — 문서 (`validate_docs` 변경 md clean·`git diff --check` clean: 설계 문서 12개 v0.5 헤더 동기화, 이 문서 포함 13 files clean, 정합화 개정 §7.4), W 기록 (`WORKSPACE_ANALYSIS.md` §3 칸 전부 채움), 승인 (S0.6·S0.8 기록이 issue #537 코멘트), S0.7 (아래), S0.9 (§1a).

- S0.1 결정 D-1~D-24 기록 (D-12 는 값이 준비되는 대로, D-13 은 S9, D-24 는 S5 전; D-21~D-24 는 정합화 개정에서 추가)
- S0.2 `WORKSPACE_ANALYSIS.md` 기록 칸을 §2 로 채움 · S0.3 설계 문서 v0.5 개정 (명명 규칙 `rtc::catching`·PascalCase 는 규칙만, 개명은 S1.1)
- S0.4 삭제 — ball_perception 은 사용자가 직접 개발 중이라 요청 이슈 불필요 (P-3) · S0.5 Epic issue #537 + Sprint Contract
- S0.6 `[CONCERN] E-1` §3.1 예외 문구 승인 (2026-09-19, invariants.md §Clock 시간축 규칙에 별도 커밋) · S0.8 `[CONCERN] E-3` D-14·D-20 (§7.1) 승인 (2026-09-19)
- S0.7 vision 지평 손계산 (아래 결과) · S0.9 Epic 검정력 표 — (가정 성공률 × floor) 격자에서 Wilson 95% 하한이 floor 를 넘는 시행 수, D-12 floor 확정 시 S8 벽시계 예산 입력 (§1a)

**S0.7 결과 (2026-09-19, 에이전트 산출 — 스크립트는 저장소에 없고 식과 가정표로 재현된다).** 탄도는 2차 항력 (L0 §4.1) **a = −g ẑ − k‖v‖v**, Magnus 없음 (sim 은 Magnus 도 구현 — L0 G0-5), 수치 적분 + 슈팅. 지평 H 는 `header.stamp` 부터 마지막 점까지, 간격 0.05 s. plan 반영 시각은 t_c − H + L (L = L_vis + T_pub + T_plan + `t_horizon_margin`).

- (R1) commit 조건 (§3, L3 §4.11): H ≥ T_freeze + L, T_freeze ≥ T_close,tot + T_arm + T_margin
- (R2) 팔 이동 조건 (L3 §4.3 + γ 창 T_w): H ≥ T_arm + T_margin + T_move + L, T_move = max(t_min(대기 자세 → q*), T_w)
- 점 수 n = ⌈H/0.05⌉ — ball_perception 예측점은 `step, 2·step, …, horizon` 이라 t = 0 점이 없다 (`prediction.hpp` 계약). S3.6 `n_max` 도 같은 산식
- 계획 가능한 투척은 T_f − T_det ≥ H_req 인 것뿐이다 — 지평을 늘려도 넘지 못한다

| 가정 | 값 (출처) |
|---|---|
| 발사 · 포구점 | 4 m, z 1.5–2.0 m (D-18) · ρ_c 0.3–0.8 m, z_c 0.4–1.0 m (base = world z=0, S3.2 전 가정) |
| `t_horizon_margin`, T_pub, T_plan | 0.05, 1/30, 0.010 + 1/60 s (L2 기본값, D-15 예시, L3 `budget_s`) |
| L_vis, T_det, T_margin · T_close,tot, T_arm | 0.03, 0.10, 0.02 s · 0.01–0.10, 0–0.10 s (가정, T_arm TBD) |
| T_w, 관절 t_min | 0.3–0.6 s, ā = 10 rad/s², ω̄ = π rad/s (L3 `window_grid`, 참조 `test_l3`; D-16 box TBD) |
| 항력 k | 0.0229 1/m (L2 §4 검증 표, D-12 대기). `sim.ball.drag_k` 는 **TBD 로 남는다** — S3.8 이 S3a 범위 밖 (2026-09-20) |

| 항목 | 결과 |
|---|---|
| 도달 가능 투척 | 최소 v₀ 4.5–5.8 m/s, 포구 속력 ≥ 5.9 m/s (최소 v₀ 는 T_f 0.8–0.9 s 부근). T_f 1.0 s 면 앙각 40–53°, v₀ 4.7–5.9 m/s, 포구 속력 6.1–7.2 m/s |
| L · R1 | 0.140 s · H 0.17–0.36 s (n 4–8) |
| R2 | T_move 0.30/0.45/0.60 s × T_arm 0–0.10 s → H 0.46–0.86 s (n 10–18), 먼 q* (Δq 1.5 rad) 면 H ≈ 1.0 s. sweep 전 구간에서 R2 가 지배 |

- **`kCap` = 40 (2026-09-19 사용자 결정, provisional).** sweep 최대 n 18 × 2 = 36 을 8 의 배수로 올림. 0.05 s 간격에서 약 2.0 s 지평 (40 번째 점 2.00 s), 스냅샷 매 tick 복사 약 3.2 KB. S3.6 `n_max` 20 ≤ 40 이라 backfill PASS (§4.4 S1)
- **0.5 s profile: R1 충족, R2 부족** (T_arm 0.05 s 에서 T_move ≤ 0.29 s 필요, T_w 최솟값 0.3 s). 첫 점이 +0.05 s 라 실지평은 0.50 s. **D-18 거리·속도 조정으로는 안 메워진다** — R1·R2 는 로봇 쪽 선행시간이고 발사 조건은 T_f − T_det ≥ H_req 만 정한다
- **T_f ≥ 1.0 s (D-18, 2026-09-19)**: T_f − T_det ≥ 0.9 s > 최대 H_req 0.86 s 라 지평만 충분하면 R2 까지 계획 가능 (먼 q* 는 T_f ≥ 1.1 s). 포구 속력 하한 6.1 m/s 로 S4.4 부담이 커진다
- **sim profile 지평 0.8 s / 16 점 채택 (2026-09-19, 설정은 사용자 — D-15)** — 이후 S3.6 이 1.0 s / 20 점으로 정했다 (§4.4 S3.6)
- **지평 요구 = R1 (2026-09-19 사용자 결정).** 계획기는 `APPROACH` 동안 지평 안 후보로 먼저 출발했다 교체하고 (L7, L3 §4.7) 도달시간은 현재 명령 상태 (q_c, q̇_c) 에서 검사하므로 (L3 §4.3), R2 (대기 자세 정지 출발) 는 **첫 plan 이 안 나오는 최악 경우의 기록값**이다 — R2 로 `io.horizon_min` 을 잡으면 profile 을 통째로 거부할 수 있다. 지평 < T_f 라 첫 목표는 항상 작업공간 가장자리 (catchability S3.5a, 첫 plan 시각·교체·탈락 사유 S8)
- **대기 자세 = 겨냥점 근처 (2026-09-19 사용자 결정)** — 첫 이동을 줄인다. IK seed (D-18)·지도 겨냥점 (§11) 과 같은 자세, 구체 자세는 S3.5a
- 포구 속력 ≥ 5.9 m/s 는 CATCHING_MASTER §4.1 우려 (6 m/s 에 T_close,tot ≤ 8.9 ms) 를 4 m 투척에 확인한다 — S4.4 go/no-go 핵심 입력

#### S1 순수 수치 코어 (ROS 비의존)

**상태: 완료.** S1.1~S1.8 (2026-09-19, PR #541), S1.9 (2026-09-20, S2.1 머지 후, PR #552). GUI·plot 면제 (D-19, §13).

- S1.1 `catching` 골격, 참조 테스트 GTest 이식 (`rtc::catching` + PascalCase) · S1.6 `ball_dynamics` 는 test fixture 전용
- S1.2 공용 궤적 타입 + Hermite 샘플러: SeqLock POD (`std::array`, Eigen 멤버 금지 — §6), 용량 `kCap` (provisional), 런타임 `n_max ≤ kCap` 은 S3.6. 점 개수를 인덱싱 전에 검사, NaN·`dt_min` 미만 거부, provenance token (D-22)
- S1.3 시간 타입 `BallTime`/`NowReal`/`NowLead` (§3), D-2 변환 · S1.4 soft-catch 기준 생성기 (NaN 가드, derate 없음 D-8)
- S1.5 도달 가능성 (`time_feasibility`, 방향 속력, 정지거리·오차 예산, 잘못된 한계 flag) · S1.7 파라미터 검증기 (활성 키 TBD, 교차제약 D-9, ζ·ω·h, provisional 실기 차단) · S1.8 L7 순수 조각 (감속 목표, 전이표, 접촉 debounce)
- S1.9 포구 자세 IK 루프 + catchability 판정 (L3 §4.2, §11): `DifferentialIk` (m=5) 반복, seed = wait_pose, w₅·w₆ fail-closed, 영공간 $\log w_5$ 상승 (D-25). **S3.5a/b 지도 도구와 S6.2 런타임이 이 함수 하나를 쓴다** (ARCH-3)

| 게이트 | PASS 기준 | 판정 |
|---|---|---|
| 이식 | 참조 테스트 GTest 통과 (L0 G0-A, L2 G2-A~D·G2-G, L3 G3-A·G3-B, L4 G4-A~C·G4-F). G4-D 는 S2.1, derate 는 D-8 로 제외 (G4-E) | PASS, 임계 무수정 |
| 회귀 | `n > kCap`·NaN·`dt_min` 미만 ASan/UBSan 무오류 + invalid (G2-H, G1-A), 비단조 쌍 (G2-G), NaN 목표 가드 (G4-I) | PASS |
| RT · 시간 | 할당 0·`noexcept` (G0-B, G2-E, G4-G) · 교차 시간 축 비교 컴파일 불가, T_arm ≠ 0 fixture (G0-E) | PASS · PASS |
| 검증기 | G0-C 전 항목 | PASS |
| S1.8 | G7-A 표 완전성, G7-B 기준 상태 연속 < 1e-9, G7-C 오경보율 기록, G7-D 할당 0 | PASS · G7-C NOT_EVALUATED(임계 — 사용자). 기록: k_σ = 3 합성 잡음 0/20000 |
| S1.9 | 수렴·스텝 제한, w₅·w₆ FD 대조, 퇴화 입력 탈락 + 사유 (G3-I 함수 부분), roll sweep 국소 최대 (D-25), 할당 0 (G3-K 함수 부분) | PASS |
| backfill | S3.6 의 `n_max ≤ kCap` (초과 시 `kCap` 상향 후 재실행) | **PASS** (2026-09-22: `n_max` 20 ≤ `kCap` 40 — §4.4 S3.6) |

**S1 결과 (S1.1~S1.8).** rtc_controllers `include/rtc_controllers/catching/` (헤더 전용) + `src/params/catching_params.cpp`, 7 스위트 130 케이스, ASan/UBSan 보고 0. 알려진 참조 결함 (§2 참조 구현 항목의 `check()`·`tMinChecked`) 은 이식 때 고쳤다.

- G3-A 는 독립 python 닫힌식 고정표 42행 < 1e-9 (verify_l3.py (삭제됨)), G3-B 는 §4.8 표 ±5%. 참조 L2 fixture 60 → 40 점 (`kCap`) 은 사용자 승인 fixture 변경. 참조 G4-B 의 dt = 0 읽기는 `Evaluate()` 로 분리 (L4 §5.1), L3 §5.2 `q_star` 용량은 `kMaxPlanNv` (32)
- RT 기록 (개발 PC, 비 RT): 스냅샷 복사 3240 B/tick, 복사 + `SampleAt` 최악 3.4 µs, `Step` 최악 0.2–0.4 µs. 0.05 s 간격 16 점 보간 오차는 위치 2.0e-11 m, 가속 1.9e-7 m/s² — S3.6 간격 선택의 입력
- 검증기의 ωh ≥ 0.828 경계는 `reference.omega` [1, 25] 안에서 도달 불가라 공식만 검증하고 게이트 문구를 고쳤다 (2026-09-19 사용자, L0 §5.3·§9)
- 리뷰·수치 감사로 바뀐 동작: FAULT 에서 ESTOP 해제가 래치를 풀지 않음 (L7 §4.2 사유표에 FAULT 예외 — P-1·S5.1(d)), 없는 YAML 섹션은 기본값 + 검증기 차단, 빈 손 배열 거부, wire 시각 뺄셈 포화, `Evaluate()` 의 NaN-valid fail-open 제거

**S1.9 결과 (2026-09-20).** `rtc::catching::CatchPoseIk` (`catch_pose_ik.hpp`/`.cpp`), `test_catch_pose_ik.cpp` 25 케이스, URDF fixture `serial_6r_wrist.urdf`, rtc_tsid `QPSolverWrapper::ResetWarmStart()`. 설계 변경 D-25 (roll manipulability 최대화 번복 · ρ 과제 가중 정정)·D-26 (과제 스텝 제약 QP) 은 L3 §4.2·§6·§10 에 반영.

**A·B 비교 (사용자 결정 근거, 2 fixture × 500 후보; bench 는 폐기).**

| candidate | accept | w₅ p50 | resid p50 | limit 활성 | µs p50 | QP 비수렴 |
|---|---|---|---|---|---|---|
| 6R A: DLS | 98.8% | 0.0203 | 8.3e-08 | 1.2% | 1444 | – |
| 6R B: QP μ=1e-8 / 1e-6 | 99.0 / 99.2% | 0.0201 | 1.4e-07 / 8.8e-08 | 1.4 / 2.0% | 2061 / 1783 | **483** / 0 |
| **6R B: QP μ=1e-4** | **99.2%** | 0.0201 | 7.8e-08 | **1.0%** | 1777 | 0 |
| 6R B: QP μ=1e-2 | 93.4% | 0.0212 | 2.1e-07 | 1.3% | 1775 | 0 |
| 7R A: DLS | 98.2% | 0.0817 | 4.3e-07 | 5.5% | 1820 | – |
| 7R B: QP μ=1e-8 / 1e-6 | 98.2 / 98.6% | 0.0800 / 0.0822 | 7.4e-07 / 4.3e-07 | 5.9 / 4.1% | 2512 / 2230 | **460** / 0 |
| **7R B: QP μ=1e-4** | **98.8%** | 0.0820 | 4.5e-07 | 4.7% | 2227 | 0 |
| 7R B: QP μ=1e-2 | 88.2% | 0.0904 | 6.2e-07 | 5.0% | 2241 | 0 |

- **선택 B, μ = 1e-4 (사용자, 2026-09-20)** — 수락률·잔차·한계 활성이 A 이상, 시간 +22%. **μ 는 절벽이 있는 손잡이다**: 1e-8 이면 $J^\top J$ (rank ≤ 5) 정칙화가 모자라 QP 가 대부분 수렴하지 않는다 (A 의 σ_min 적응 λ 가 하던 일). 제약은 $\dot q_{clik}$ 만 묶으므로 $\dot q_d$ 축소·한계 clamp 는 여전히 필요하다
- 할당 0 은 최적화·sanitizer 빌드에서 확인 (정상 빌드의 0 은 `g.noalias() = -(Jᵀe)` 때문에 거짓 green 이었다 — 곱과 부호 반전 분리). `TheAllocationGatesAreArmed`·`TheTaskQpItselfAllocatesNothing` 로 고정. mutation 10/10 red, UBSan 잔여는 기존 third-party UB
- 결정성: 후보마다 `ResetWarmStart()` cold start 라 사이에 다른 후보를 끼워도 bit-identical — 지도·런타임 동치 (§11) 의 전제
- `/code-review` (PR #552) 5건: device 관절 순서 핸들 거부 (`kJointOrderMismatch`), 실패한 FD 탐침은 수렴 아님 (G3-G 신호 역전 방지, `manip_grad_failures`), 수락 후 QP 실패는 $q^\ast$ 반환 (`qp_failures`), σ_min·λ² 는 $q^\ast$ 값, $q_n$ = clamp 된 seed (L3 §4.2·§6)
- **`planner.ik` 기본값 — 2026-09-21 확정**: §11 지도의 제안값 (`k_manip` 0.5 · `max_iter` 40) 을 출하 config 로. `mu`·`qp_eps_abs` 를 포함한 나머지는 기본값 그대로 (§11). `alpha_max` 는 여전히 TBD 라 인자로 받는다

#### S2 기존 rtc_* 일반화 (code review 대상)

**상태: 완료 (2026-09-20, PR #545~#549 + 마감 PR), S2.3b 완료 (2026-09-21).** 사용자 결정 (2026-09-19): PR 순서 S2.1 → S2.2 · S2.3a · S2.5 → S2.4, S2.2a 는 행 선택형 확장 (L5 §5.1), S2.5 도구는 rtc_tools python. golden 비트 일치는 Release 빌드에서만 판정 (sanitizer 는 상대 1e-12).

- S2.1 `rtc_math` se3 축 정렬 오차·각속도·Jacobian · S2.2a CLIK 확장 구조 + 기존 동작 golden-vector 회귀
- S2.2b `ClikReferenceGenerator` 옵션 (D-5·D-6): twist feedforward, LOCAL 접근축 2행, 가속 box + `bound_conflict`, q̇ 평활, status·반복·solve time 노출, `max_iter` 설정, q_c 평가 모드
- S2.3a `rtc_urdf_bridge` extra frame (D-10, D-17, §10): `urdf.extra_frames.<name>.{parent,xyz,rpy,provisional}` → CM 파서 → `ModelConfig` → full 모델 `addFrame`. 불완전 항목은 실패
- S2.3b catch frame 위치 = S4.5 실측 포구점을 부모 frame 으로 옮긴 값 ("preshape 손끝 중심" 기각). **완료 2026-09-21** — 값·frame 규약 대조 PASS·`provisional: false` 전환은 §10
- S2.5 관절 가속 한계 도출 도구 (D-16, §9) · S2.4 DemoWbc 회귀 (assertion 무수정), downstream, Doxygen·README, `/code-review`

| 게이트 | PASS 기준 | 판정 |
|---|---|---|
| 동등성 | 옵션 off 에서 golden 해시 일치·bit-identical (G5-A2), 기존 assertion 무수정 (PROC-6, E-6) | PASS — `test_clik_golden.cpp` 2520 값, mutation 11/11 |
| CLIK | G5-A, G5-B, G5-B2, G5-C3, 할당 0 (G5-C 할당 부분) | PASS · G5-C solve time 예산은 S2 에서 NOT_EVALUATED(사용자 값) → **S5.3 에서 PASS** (400/1500 µs, §4.4 S5) |
| se3 | G4-D | PASS |
| extra frame | catch frame 이 full·sub·tree·actuated 에 같은 부모 기준 위치, 잘못된 부모·중복은 configure 실패, provisional 은 실기 arm 차단 (G0-C) | PASS |
| 가속 도출 | §9 산출물이 YAML 에 기록, 퇴화 시 비채택, iiwa7 sim 교차 검증 도출값 ≤ 달성 가속 (입력 η_τ) | PASS(provisional) — 표본 범위가 S3.5a 대기 자세 전 |
| 문서 · GUI·plot | Doxygen·README (PROC-1) · §13 S2 행 | PASS · PASS (main `d7715c90`, `clik_valid` 100 %) |

- **se3**: `axis_align.hpp`, 11 케이스. G4-D: exp 잔차 < 1e-12, 유한차분 < 1e-5 (1–170°), 데드밴드 주변 유한 (처리 방식 L4 §4.5)
- **CLIK**: `test_clik_options.cpp` 25 케이스. G5-A 정지 목표 < 1 mm·< 0.5°, G5-B 1e4 tick 위반 0 (`bound_conflict` 706 tick, 위치 초과 최대 0.054 rad < margin 0.1). 리뷰로 실패 뒤 `v_prev` 0 초기화, 명령값 모드 상태 검사는 ResetAnchor 까지 유지. **G5-B2·G5-C3 의 L7 전이·abort 부분은 S5.3·S7**
- **extra frame**: 네 모델 위치 1e-12 일치. `test_catch_frame_models`: 손가락 굽힘 시 끝점 중심이 catch frame +z 로 이동 (rpy 반전 시 red), ur5e_p1b full nq = nv = 26·actuated 16, iiwa7_leap full·wbc nv 23. 축: p1b `l_palm_link` rpy 0, iiwa7_leap `palm_lower` rpy [π,0,0]
- **가속 도출**: `rtc_tools derive_accel_limits` (분석 도구 — 출력은 출하물도 컨트롤러 입력도 아니다) → 값은 `integrated_bringup/config/<robot>/controllers/catching/search_grid.yaml` 의 `robot.arm.qdd_max` · `qdd_provisional` (E1-F11; 이전에는 `derived_accel_limits.yaml` 파일과 provenance). **η_τ = 0.8 (사용자 확정 2026-09-20)**, 관절 box 전체·±max_velocity, 20000 표본 + Powell 정제. **ur5e_p1b 2.03 rad/s²** (binding shoulder_lift, 중력) · **iiwa7_leap 9.20 rad/s²** (binding A2), provisional. 교차 검증 RNEA 최악 0.78·0.85, iiwa MuJoCo `mj_inverse` 0.67. **UR5e 값은 S0.7 가정 ā = 10 rad/s² 보다 크게 낮다** — S3.5a 대기 자세 주변으로 재생성한 값이 S4.4 입력

**단계 중 발견 (기존 상태·범위 밖).**

- p1b idle sim 에서 DemoWbc 의 **TSID QP 가 거의 매 tick 수렴하지 않는다** (`qp_converged` 0.5 %, main `f0e03c42` A/B 에서도 같아 S2 회귀 아님, CLIK 쪽 `clik_valid` 100 %) → #551 (open)
- `QPSolverWrapper` 가 비유한 해 한 번 뒤 영구 실패 (NaN warm start, ProxQP 는 NaN 에도 SOLVED) — golden 기록 전에 선수정 (사용자 결정). DemoWbc 주석의 모델 크기를 실측값으로 정정 (E-9)
- 기록만: CM `system_model_config_` 가 재configure 마다 sub/tree 모델을 누적, SE3 경로의 base 정렬 오차 × world 정렬 J 불일치 (현 구성에서 잠재), `anchor_drift_max` clamp 가 비유한 `q_ref` 를 세탁 (실경로 도달 불가)
- S5 배선: `ValidateCatchingParams` 는 `demo_catching_controller` configure (`integrated_bringup/src/controllers/catching/lifecycle.cpp`) 가 호출한다. `CheckCatchFrameProvisional` 은 아직 production 호출자가 없다

#### S3a 시뮬레이션 기반 (`rtc_mujoco_sim`, robot-agnostic)

**상태: 완료 (2026-09-20, PR #553).** 범위는 S3.1a · S3.2 · S3.3 · S3.4 (2026-09-20 사용자 결정).

> - **S3.7 (팔 지연 에뮬레이션·식별) 제외** — 에뮬레이션 지연을 넣지 않고 `arm_lag` 파라미터도 없다. 지연 식별은 실기 S10 (#613, L5 §7 L5.9), 게이트 `G5-D` 제외. **단 sim 팔 actuator 자체가 1차 지연이다** — MJCF 기본 게인에서 시정수 ≈ 200 ms 였고, 출하 `ur5e_p1b` sim 은 D-S8-13 으로 τ 0.05 s 다 (L5 §4.4·§4.4 S8) — "지연 없음" 은 *에뮬레이션 지연 0* 으로만 참이다
> - **S3.8 (공 항력 k 식별) 제외** — 예측은 `ball_perception` 이 주므로 `sim.ball.drag_k` 는 **fixture 전용 TBD**. 게이트 `G0-D` 는 L0 §9 에 남고 S3.5a 로 이월
> - **파급:** `G5-E`·`G8-E` 는 지연 주입을 **테스트 fixture 전용**으로 두는 완화로 확정 (2026-09-22 사용자, §7.3). G5-E 는 fixture 판정 (S5.3 PASS), **G8-E 는 sim 런타임 lead on/off 로 판정** (D-S8-1, §4.4 S8). `planner.budget.sigma_trk` (L3 §6) 는 S10 (#613) 까지 TBD

- S3.1a D-3 무부하 검증 (§5) · S3.3 공 접촉 truth (시각·충격량·접촉력), truth 주기 상향, per-step `(sim_time, steady_now)` 진단 lane
- S3.2 발사 srv (D-14, PROC-3) + iiwa7_leap projectile 설정 + **world ↔ arm base 를 MuJoCo·Pinocchio FK 대조로 확정**. 발사 계통은 이미 있었고 (`WORKSPACE_ANALYSIS.md` W6-1) 없던 명령 인터페이스를 `rtc_msgs` srv + 기존 writer 경로로 추가 (선례 `SetExternalWrench.srv`, `/sim/launch_ball_at`). iiwa7_leap 은 LEAP 손가락 충돌 마스크까지 새로 유도
- S3.4 `sim_estimator_node` 연결 — **측정만** (정책은 S5.2). 도구 `rtc_tools` `vision_lane_probe` / `camera_relay` / `analyze_vision_lane` (D-4 대로 필드 이름 디코딩). 당시 프로파일 0.8 s / 0.05 s / 16 점, 공분산 대각 2.5e-5 (노이즈 5 mm), 입력 best_effort, `ros_system_time` + `use_sim_time:=false` (rtc 에 `/clock` 없음 → **설정으로 닫힘**)

| 항목 | `ur5e_p1b` (20 발사) | `iiwa7_leap` (10 발사) |
|---|---|---|
| 발행 주기 · N · 지평 (TBD-VIS-04) | 30.0 Hz (p05 30.3 / p95 29.7) · **16** · 0.05…0.80 s | 30.0 Hz · 16 · 0.05…0.80 s |
| stamp→수신 지연 | p50 32 / p95 41 / max 430 ms (30 Hz 주기 포함; estimator 처리 p50 13 µs / p95 663 µs) | — |
| `frame_id` (TBD-VIS-06) | `world` ×856 | `world` ×427 |
| validity | VALID 806 / 빈(INVALID clear) 50 — 비행당 ≈2.5 clear | VALID 393 / 빈 34 |
| 공분산 NaN | 0 | 0 |
| `snapshot_sequence` 되감김 / generation 변화 | 0 / 39 | 0 / 29 |
| 구독 reliability (TBD-VIS-08) | best_effort = reliable identity 동일 856/856 | 427/427 |

- **유령 트랙 (TBD-VIS-07)**: 입력이 끊기면 VALID 예측은 다음 tick 한 건 (≤34 ms) 뒤 **침묵**, INVALID 스냅샷 없이 `track_status` 만 +100 ms COASTING · +500 ms LOST ⇒ 소비자는 침묵을 소실로 읽어야 한다 (`io.t_stale`)
- 지연 50 ms 는 전부 수용 (capture stamp 로 흡수), 드롭 30 % 는 1001/1400 수용. sim 재시작에서 `clock_reset` 미발동 — 그 함정은 `/clock` + `use_sim_time` 전환 뒤에만 유효
- **ball_perception 결함 (rtc 밖, 그쪽 세션에 보고)**: `debug.enabled_topics` 에 `prediction/trajectory` 만 있으면 `needs_samples()` 가 샘플을 기록하지 않아 발행 0건. 우회 = 다른 debug 토픽 동반
- S3.6·S5.2 로 넘긴 것: `io.n_min`·`io.t_stale`·`io.future_tol`·`io.horizon_min`, 되감김 정책 (관측되지 않음), validity 부분 수용 정책

| 게이트 | PASS 기준 | 판정 |
|---|---|---|
| e2e | 발사 → PointCloud2 수신, 같은 seed 재발사 truth 동일 | **PASS** — seed 42: `ur5e_p1b` max \|Δ\| 7.9e-11 m, `iiwa7_leap` 자유비행 0.0 (팔에 튄 뒤 7.95 mm 는 팔 제어 비결정성) |
| D-3 무부하 | §5 판정 (구성별 무효율 상한) | 2종 × 200 발사 — 실측·ε_clk,alloc 은 §5.1. **r_cap 확정 후 재판정: LEAP PASS, P1b PASS(provisional)** (§5.1) |
| frame | world ↔ base FK 잔차 < 1e-6 m, §11 기록 | `iiwa7_leap` **PASS** · `ur5e_p1b` **FAIL** — 프레임은 항등, 잔차는 MJCF↔URDF 치수 차이라 **오차 예산의 모델 항으로 센다** (사용자 2026-09-20; 수치·결정은 §11) |
| PROC-3 · GUI·plot | 전체 빌드·테스트 · §13 S3 행 | **PASS** (5198 tests, 0 failures) · **PASS** |

#### S4a 손 타이밍 측정

**상태: S4.0·S4.1·S4.2·S4.5 완료 (2026-09-21, 브랜치 `feat/s4a-hand-timing`).** 결정 Q1~Q10 권장안 승인 (2026-09-20). $T_{close,e2e}$ P1b **280.5 ms** (η=0.7) · LEAP **103.7 ms** (η=0.5). 놓인 공은 잡지만 날아드는 공은 0.5 m/s 부터 거의 못 잡는다 (폐쇄 속도) → S4.4.

> - **S4.3 (실기 T_close,tot) 은 S10 (#613) 으로 이월** — 실기 계단은 이 컨트롤러가 실기 팔을 hold 해야 해 E-8 승인 전 팔 명령 경로가 열린다 (L6 §7 L6.5). G6-D `NOT_EVALUATED(실기)`, S4.4 는 sim 값으로 `PASS(provisional)` (§4.1)
> - **S4.5 는 `d_eff` 와 `r_cap` 을 둘 다 산정한다** (같은 TBD-HAND-04 의 접근축 깊이 / 포획 반경; D-3 를 여는 것은 `r_cap`). L6 §4.5 step 2 (저속 투척 보정) 는 sim 에서 하지 않고 실기 S10 (#613) 에서만 한다 (사용자 2026-09-29, §7.3). 공 크기는 sim 공을 provisional 로 쓴다 — 출하값은 `core.ball.diameter` 0.067 (`projectile_ball` 반지름 0.0335 m, ITF Type 2; 이 결정을 적을 때의 `radius_m` 0.025 · 지름 0.05 는 그 전 값이다). `d_eff`·`r_cap` 은 이 지름에 대해 산정한 값이라 **D-12 공 사양 확정 시 재산정** 한다 (Q10)
> - **범위 정리**: S4.1 은 S1.7 `HandProfile` 에 `q_open`·`eta_close`·`T_close_e2e` 만 더한다 (`T_pre`·`T_hold`·`T_close_timeout`·`hold.*` 는 S7.1). 측정은 기존 `DeviceStateLog` (`<hand>_state.csv`) + 오프라인 분석기 (C++ ρ 함수는 S7.1). G6-B 는 바인딩 반쪽만 (CM 반쪽은 `test_rt_loop_pipeline`). **손당 200 회** (L6.2 의 ≥20 회로는 99 % 불가). T_close 는 steady 와 tick×dt 두 축으로 보고 (YAML 은 steady p99)

- S4.0 `demo_catching_controller` 최소 골격 (S5.1 에서 앞당김, `integrated_bringup`, YAML `config/<robot>/controllers/demo_catching_controller.yaml` — `ur5e_p1b`·`iiwa7_leap` 만). `diagnostic.hand_step: true` 일 때 손 `joint_goal` 을 무성형 (clamp 만) 통과, 팔은 hold, `rtc_msgs` 변경 없음. **sim 전용 가드**: `backend.type` 이 `mujoco_native` 가 아니면 **활성화를 거부** (configure 는 SUCCESS 로 DISABLED — configure 거부는 sim·실기 공유 config 때문에 실기 bring-up 전체를 막아 2026-09-21 리뷰로 바꿨다). 가드는 S5.1 에서 E-8 승인과 함께 없앤다
- S4.1 손 프로파일 YAML (P1b 10 DoF, LEAP 16 DoF) `robot.hand.{q_open,q_pre,q_close,caging_mask,eta_close,rho_eps}`, 전부 provisional (TBD-HAND-05). LEAP 은 사용자 제공 자세, P1b 는 **탐색한 자세** (2026-09-21, L6 §4.5)
- S4.2 T_close 식별 도구 (`rtc_tools`): 계단 반복 러너 + `<hand>_state.csv` → ρ(t)·T_close,e2e(η) 분석기
- S4.5 `d_eff`·`r_cap` (L6 §4.5 step 1·3) — 접촉 시뮬레이션 (preshape 에 공을 놓고 닫은 뒤 ±g 로 흔듦). **LEAP**: 793 중 216 파지 → `r_cap` 31.0 mm · `d_eff` 80 mm. **P1b**: 사용자 자세 851 중 0 → `q_pre`/`q_close` 진화 탐색 (seed = 사용자 자세), 미사용 격자 445 중 214 → `r_cap` 24 mm · `d_eff` ≥ 95 mm (스캔 상한), η 0.7, `T_close_e2e` 280.5 ms

| 게이트 | PASS 기준 | 판정 |
|---|---|---|
| 손 명령 | 수락 tick 부터 손 slot 명령 == clamp(목표) bit-equal (G6-B 바인딩 반쪽), 팔 명령 변화 0, `Compute()` 할당 0 | — |
| 골격 | 두 sim 프로파일 configure→activate, `ur5e_p1a` 불변 (`test_registered_controllers_have_shipped_config`), 비-`mujoco_native` 활성화 거부, 플래그 off 에서 손 목표 거부, 비활성 중 목표 미사용 | — |
| 프로파일 | sim 구성 검증 에러 0, dof == 손 채널 수, YAML ∩ URDF 한계 안 (입력: 사용자 자세 확인) | — |
| T_close | 두 손 분포 (평균·최대·99%) steady·tick×dt 두 축, log drop 0 (G6-C 산출 부분) | P1b 280.5 ms · LEAP 103.7 ms |
| d_eff·r_cap | G6-F: 산정식·값·provisional 이 `planner.hand.*`·L6 §4.5 에 기록, D-3 갱신 | **LEAP·P1b PASS(provisional)** (사용자 승인 2026-09-20 LEAP · 2026-09-21 P1b). 투척 보정은 `NOT_EVALUATED` — sim 보정은 하지 않고 실기 S10 (#613) 에서만 한다 (사용자 2026-09-29, §7.3) |
| GUI·plot | §13 S4 행 | — |

- "—" 인 행의 판정은 이 절에 기록된 적이 없다 (단계 완료는 §4.3)

#### S3b·S4.4 지도·go/no-go·vision 사양

**완료 (2026-09-22).** 정의: S3.5a kinematic 지도 (D-18, 타이밍 provisional, 결과는 §11) → S4.4 go/no-go (S3.5a 속도 범위·S4.2 T_close·S4.5 d_eff·S2.5 가속 box·S1.7 η_v 로 받을 수 있는 최대 공 속력을 계산해 목표 속도를 확정하거나 낮춘다) → S3.5b gate-catchable 지도 (S4.4 값으로 IK + w + 도달시간 + γ 창 + 정지거리 체인을 다시 돌린 것이 목표 투척 분포) → S3.6 vision 요구 사양 (D-15: 목표 분포의 검출 이후 포구 창 종료까지 최대 비행 시간 → 지평, L2 보간 게이트를 만족하는 간격 → 점 수 `n = ⌈H_req/간격⌉` (t = 0 없음, §S0.7 산식 정정 2026-09-20) → 런타임 `n_max` ≤ `kCap` (넘으면 S1.2 backfill), sim profile 설정은 사용자). 사용자가 D-12 투척 분포를 동결한다.

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| go/no-go | 목표 속도에서 γ 창이 비지 않음 (G6-C 의 판정 부분) | T_close (S4.2), d_eff (S4.5), 가속 box (S2.5), η_v (S1.7), 목표 속도 (S3.5a) — 하나라도 provisional 이면 PASS(provisional) |
| 지도 | 같은 입력에서 S1.9 판정이 런타임과 같음 (G3-I), 두 지도와 탈락 사유 분포 기록 | — |
| 사양 | 지평·간격·점 수·`n_max` 가 여기 기록되고 `n_max ≤ kCap` | — |

**S4.4 결과 (2026-09-22) `[조건부 go — 사용자 결정]`.** 도구 `rtc_tools catch_speed_budget` (수락 후보별 팔 속도 LP·토크 한계 방향 가속·stroke → γ 창). 값은 **낙관적 상한**이라 창이 비면 확정적, 열리면 provisional (S3.5b 가 닫는다). 후보별 $v_{dir,\max}$ 는 S3.5a 산출물 (`candidates.csv`) 과 관절 정격의 순수 함수라 S4.4 가 첫 산출물로 낸다 (§4.2 DAG 불변).

- **원 D-18 투척 (4 m, T_f ≥ 1.0 s, 릴리스 1.5–2.0 m) 에서는 두 로봇 모두 γ 창이 빈다.** 거리·T_f 는 추정기 정확도용이라 제약이 아니지만 (2026-09-21 사용자) 풀어도 같다: 포구 속력 바닥은 $\sqrt{g(R-\Delta z)}$ 이고 현 씬 (base world z = 0) 은 포구점이 전부 하강 구간 ($\Delta z\le-0.5$ m, ≥ 3.0 m/s) — 거리 1–4 m × 릴리스 1.0–2.0 m × T_f ≥ 0.25 s 9450 투척에서 `ur5e_p1b` 1 (η_v = 1 에서만) · `iiwa7_leap` 0.
- **팔·가속·손.** v̂ 방향 속력 상한 (LP, η_v 0.9) 중앙값 1.1–1.2 / 1.4 m/s (`ur5e_p1b` / `iiwa7_leap`, 최대 3.5 / 1.8); 최소노름은 LP 의 0.90 / 0.82 배 (S6.3 입력). 토크 한계 방향 가속보다 box 가 훨씬 작아 **D-16 box 가 구속한다** (수치는 §9 "발견"). 시각 발동 fly-in 상대속도 허용량은 두 손 약 **1.0 m/s** (L6 §4.5, 공식 0.34 / 0.76) 이나 하강 구간은 3–4 m/s ($T_{close}$ 25–30 ms 에 해당) 에서야 10–50 % 가 열린다.
- **조건부 go 의 세 조건.** ① **낙차** — 포구점이 릴리스 높이 −0.25 m 이상 (결과는 릴리스 − base 높이에만 의존, 최적 0…+0.2 m: 위로 던져 정점 근처에서 받는다). ② **가속 한계** — 상수 box 가중 최적화로는 부족하다 (중앙값 `ur5e_p1b` 1.95 → 5.1 m/s², p05 0.6; `iiwa7_leap` 8.9 → 15.9, p05 4.5); 자세·방향 의존 한계 구현은 S6. ③ **선행시간** — 최소 비행시간 $T_{det}+L+T_{close,tot}+T_{arm}+T_{margin}$ (S0.7 R1) 에 손 폐쇄가 그대로 들어간다: `ur5e_p1b` 0.59 s (병목 `thumb_cmc_fe` 1.57 rad, L6 §4.2) · `iiwa7_leap` 0.41 s; T_f 하한 0.3 → 0.6 s 에서 열린 투척 147 → 11 / 112 → 3 (/3808). 이 하한은 R1 만 담았고 R2 (대기 자세 → q\*) 까지 넣은 판정은 S3.5b.
- **열리는 영역 (base 1.0 m 가정 씬, 상대속도 1.0 m/s, 상한).** `iiwa7_leap` **39 / 3808** (거리 1.0–1.5 m, 발사 2.0–5.25 m/s, 앙각 55–75°, T_f 0.45–0.60 s, 포구 1.2–2.6 m/s), `ur5e_p1b` **11 / 3808** (1.0–1.5 m, 4.25–5.5 m/s, 55–70°, T_f 0.60–0.80 s, 포구 2.7–3.8 m/s; 관절 속도 현 config 면 0–1). 거리 ≥ 2.0 m 는 0.
- **사용자 확정 (2026-09-22, §7.3).** 1차 목표 로봇 **`ur5e_p1b`** — `iiwa7_leap` 은 짧은 비행만 열리고 그 비행에 팔을 옮길 시간이 0.12–0.2 s 뿐이다 (S3.5b). D-18 개정: sim 씬은 그대로, **릴리스 0.2–0.5 m** (열린 포구점이 base 위 0.12–0.99 m 라 바닥이 구속하지 않는다; wrapper 씬은 MJCF `<include>` 가 resource provider 를 안 타 `ur5e_p1b` 에서 불가). D-16 개정 §9. `reference.v_max`·η_v 는 §7.3 결정 D (L3 §4.5, L4 §6). 원자료는 repo 밖 — `catch_speed_budget` 인자가 SSoT.

**S3.5b 결과 (2026-09-22) `[ur5e_p1b PASS(provisional) · iiwa7_leap 지도 빔]`.** 도구 `rtc_controllers catch_gate_batch` (`time_feasibility.hpp` 런타임 함수로 도달시간·γ 창·정지점) + `rtc_tools catch_gate_map` (commit 선행, 토크 층, 집계). 설정: 현 씬·릴리스 0.2–0.5 m (결정 A), 관절 속도 정격 × η_v 0.9, `reference.v_max` 도출값 3.9 / 2.0 m/s (결정 D), 가속 box 출하값, 회전자 관성 MJCF `armature`, 첫 계획 T_det + L = 0.24 s (S0.7 가정), T_arm 0.05 s, `planner.time.margin` 0.03 s, `planner.gamma.margin` 0.1 m/s, **`supervisor.decel.a_dec` 10 m/s² (provisional)**. rollout 은 `NOT_EVALUATED(S6)` (결정 E).

- **게이트·층.** commit 선행 ($t_c-t_{plan}\ge T_{close,tot}+T_{arm}+T_{margin}$, L3 §4.11) → 도달시간 → γ 창 → 정지점 (`planner.workspace.catch_box` TBD). 도달시간 두 층 (결정 B): `box` = 출하 box 로 C++ 판정, `torque` = wait → q\* 경로 전체에서 $|M\ddot q+h|\le\eta_\tau\tau_{\max}$ 인 최대 경로 가속 (충분조건, 런타임 대응물 없음). 손은 공식 `d_eff` (0.095 / 0.080 m → 0.34 / 0.76 m/s) 와 fly-in 1.0 m/s 등가 $d_{eff}=v_{rel}T_{close,tot}$ (**0.2815 / 0.1047 m**) 두 값.
- **G3-I.** 도달시간·γ 창·정지점은 런타임과 같은 함수 (열 bit-exact 테스트). q̇ᵘ 생산자는 python 이라 그 동치는 `NOT_EVALUATED(S6.2)`.
- **넓은 격자 (거리 1.0–1.5 m × 릴리스 0.2–0.5 m × 2.0–6.0 m/s × 앙각 40–80° = 1377 투척).**

| | `ur5e_p1b` | `iiwa7_leap` |
|---|---|---|
| kinematic 수락 (출하 → 다시 고른 대기 자세) | 786 → 767 | 508 → 374 |
| **공식 `d_eff`**: 도달시간 외 전부 통과 | **0** (확정적으로 빔) | 5 → 3, 열린 투척 0 |
| fly-in 1.0 m/s: 같은 후보 | 11 | 26 → 16 |
| 열린 투척 `box` / `torque`, 출하 대기 자세 | 0 / 0 | 0 / 0 |
| 열린 투척 `box` / `torque`, 다시 고른 자세 | 0 / **9** | 0 / 0 |

- **`ur5e_p1b` 허용 오차 상자 (세밀 격자 2835 투척).** 기준 투척 **거리 1.0 m · 릴리스 0.2 m · 4.75 m/s · 앙각 60°**, 대기 자세 **`[0.212, −1.376, 1.107, −1.978, −3.296, 0.121]`** (그 투척의 q\*, provisional). `torque` **590 / 2835 (20.8 %)**, `box` 6. **90 % 상자: 거리 0.9–1.0 m · 릴리스 0.15–0.25 m · 방향 ±6° · 속력 4.65–4.85 m/s · 앙각 62–64°** (180 투척 중 90.6 %). 열린 후보 T_f 0.61–0.80 s, 포구 속력 2.7–3.85 m/s, 포구점은 릴리스 위 0.44–0.71 m, γ 창 폭 중앙값 0.12, 도달시간 여유 중앙값 0.20 s. 속력 ±0.1 m/s · 앙각 ±1° 는 sim 발사기 허용 오차다.
- **`iiwa7_leap` 은 0** (기준 1.25 m · 0.2 m · 3.25 m/s · 70°, 대기 자세 세 라운드): 도달시간 예산 0.12–0.15 s 대 토크 층 최소 이동 0.13 s (최선 여유 −0.004 s). 7축 영공간 log w₅ 상승 (D-25) 이 q\* 를 seed 에서 0.35–0.5 rad 옮겨 대기 자세가 수렴하지 않는다 (통과 후보 1026 → 341 → 124). **막는 것은 선행시간** — 첫 계획 0.14 s 면 `torque` **65 / 2835**.
- **게이트 단독 탈락** (`ur5e_p1b`, 3838 후보): commit 2289 · 도달시간 `box` 3813 / `torque` 3150 · γ 창 3400 · 정지점 1524 — 하나를 풀어서는 열리지 않는다.
- **provisional 의 출처.** 열린 것은 전부 fly-in `d_eff` (출하 0.095 m 면 빔) · `torque` 층 (출하 box 면 0–6, D-16 런타임 구현은 S6) · 다시 고른 대기 자세 (출하 §11 자세는 4 m lob 용, `wrist_2` 가 1.72 rad 떨어짐) 위에 있다. 정지점 게이트가 단독으로 40 % 를 거르므로 `a_dec` 값이 필요하다. 런타임 최소노름 v_dir,max 는 LP 의 중앙값 0.97 배 (최소 0.86) 라 S6.3 의 여유 회수가 곧 허용 오차다. `ur5e_p1b` D-3 은 3.85 m/s 에서 FAIL (§5.1).
- **사용자 확정 (2026-09-22, §1a·§7.3).** ① **`planner.hand.d_eff` = fly-in 등가** (값·근거는 §7.3 `planner.hand.d_eff` 행; 런타임 식·코드 변경 없음). ② **`supervisor.decel.a_dec` 10 m/s² provisional** (`reference.a_max` 확정 시 ≤ 검사). ③ **1차 로봇 `ur5e_p1b`, `iiwa7_leap` 은 G8-D2 조건부** (§1a — 실측 선행시간에서 지도가 열리면 평가; 아래 "T_det 실측" 에서 열렸다). ④ 선행시간 실측은 S3.6 을 막지 않는다 (T_det 는 S3.4 rig 로 병행, L 은 S5–S6 후). 이로써 `[SPRINT]` 3 의 정지 조건이 풀려 S3.6 을 착수했다. 러너 `run_gate.sh`·원자료는 repo 밖.

**S3.6 결과 (2026-09-22) `[PASS(provisional) — ur5e_p1b]`.** `catch_gate_map` 요약에 층별 열린 후보의 t_c·포구 속력·투척별 포구 창의 min/p50/max (`open_candidates`) 를 추가하고 S3.5b 와 같은 인자 ($T_{close,tot}$ 0.2815 s = `d_eff` 0.2815 m 는 우연의 일치) 로 두 격자를 다시 만들었다. 지평 요구는 **R1** (R2 아님, §7.1), 예측점 `step, …, horizon` (t = 0 없음).

- **gate 통과 후보.** 90 % 상자: `torque` 163 / 180 (90.6 %), `box` 3, t_c 0.612 / 0.660 / 0.732 s (min / p50 / max). 세밀 격자: 590 / 6, t_c 최대 **0.804** s, 포구 속력 2.72 / 3.20 / 3.85 m/s.
- **기구학 reachable 창 (D-27 — 이것이 요구다).** 실행은 기구학적으로 도달 가능하면 도전하므로 kinematic 후보 전체 (`gate_map.csv` 모든 행) 의 포구 창 끝을 덮어야 한다: 상자 max **0.876** s (178 투척 1467 후보), 세밀 격자 0.396 / 0.804 / **0.948** s (2399 투척 16028 후보). commit 선행만 걸어도 최댓값은 같다.
- **산식과 값.** 첫 예측이 창 끝까지 보여야 첫 plan 부터 고를 수 있다: $H_{req}=\max t_c-T_{det}+t_{horizon\_margin}+T_{margin}$ (0.05 L2 §4.6 + 0.03). 구속하는 것은 **가장 이른 검출** (재실측 0.040 s): **0.948 − 0.040 + 0.08 = 0.988 s → n = ⌈0.988 / 0.05⌉ = 20** (상자만 0.916 s · n 19; gate 통과 창 참고 0.844 s · n 17). (가정 T_det 0.10 s 로는 0.928 s · n 19, stamp 수정 전 첫 실측으로는 1.001 s · n 21.) 간격 **0.05 s** 유지 (S1.2 보간 오차 2.0e-11 m ≪ G2-C 1e-10 m). ball_perception 은 `horizon % step == 0`·`horizon / step ≤ max_points` 를 강제하고 producer 상한 1000 점은 구속하지 않는다.
- **sim profile (D-15): 1.0 s / 0.05 s / 20 점 / ≤ 30 Hz — 설정됨 (2026-09-22 사용자 결정 ⑥)**, 로봇별 사본 `ball_perception_sim_profile.json` (`sim_estimator.launch.py profile_path:=`; 위치는 MD-18 로 개정 — D-15 행). 종전 0.8 s / 16 점은 창 끝 0.78 s 까지만 봐 늦게 잡는 후보가 조용히 빠진다 (D-27 이 피하려는 손실); 비용은 예측점 4 개 (약 +0.8 KB/tick). $H_{req}=1.028-T_{det}$ 라 1.0 s / 20 점은 창 끝을 12 ms 여유로 덮고 **0.95 s** / 19 점은 38 ms 모자란다. stamp 수정 전 (버스트가 만든 0.024 s 검출) 에는 **1.05 s / 21 점**이 필요했다 — 그래서 ⑤ 를 먼저 고치고 profile 을 한 번만 정했다. 확인: `ros2 run rtc_tools vision_lane_probe <prefix>` → `analyze_vision_lane <prefix>` 가 30 Hz · 20 · 0.05…1.0 s (재실측 run: VALID 2123 · 부분 무효 0).
- **런타임 값 (provisional, 기록처 L1 §6 · L2 §6).** `io.horizon_min` = ($T_{close,tot}$ 0.2815 + T_arm 0.05 + T_margin 0.03) + L 0.14 = 0.5015 → **0.51 s** (올림 — 내림은 R1 을 1.5 ms 미달하는 궤적을 통과시킨다). **`io.n_min` = 12** = ⌈0.51 / 0.05⌉ **+ 1** — n 점이 덮는 창은 (n−1)·step 이라 +1 이 없으면 11 점이 0.50 s 로 요구보다 10 ms 짧다 (S3.6 원 산식의 11 을 S5.2 가 정정, `/code-review` 2026-09-22; 출하 YAML `n_min: 12`). `prediction.dt_expected` **0.05 s**. **`n_max` = 20** — 설정한 profile 의 점 수와 같게 둔다 (런타임 상한이 profile 을 거부하면 안 된다); 20 ≤ `kCap` 40 → **S1.2 backfill PASS**. 구현 (S5.2) 은 `n_max` 키를 두지 않고 런타임 상한을 `kCap` 으로 둔다 — 20 점은 vision profile 의 속성이라, profile 이 바뀌면 거부가 아니라 진단의 점 수로 드러난다 (`integrated_bringup` 의 catching `lifecycle.cpp`). `io.t_stale` 0.10 s·`io.future_tol` 1e-3 s 는 S3.4 실측 기반 [제안] 이었고 **S5.2 에서 확정** (sim overlay `sim.io.future_tol` 0.1 — L1 §6).
- **한계 (열려 있음).**
  1. L 0.14 s 는 가정값 — `io.horizon_min` 은 L 에, H_req 는 T_det 에 걸려 있다. L 은 0.14 s 를 유지하고 실기 확정은 S10 (#613) 이다 (사용자 2026-09-29, §7.3).
  2. 목표 분포가 provisional 셋 (fly-in `d_eff`·토크 층·다시 고른 대기 자세) 위에 있다.
  3. H_req 는 "첫 예측이 창 끝까지 본다" 는 보수적 기준 — 연속 재계획·현재 명령 상태 기준 도달시간 (L3 §4.3) 에서는 더 작을 수 있다. 택한 이유는 D-27 과 비용이 점 몇 개뿐이라는 것.
  4. `iiwa7_leap` 은 이 시점 `NOT_EVALUATED(선행시간)` — 지도가 열리면 같은 요약으로 H_req 를 다시 읽는다 (G8-D2 조건부; 아래에서 열렸다).
  5. H_req 의 덧셈은 도구가 아니라 이 문서가 한다 (T_det·margin 은 이 문서의 결정).
  6. 창 끝은 이 격자·대기 자세·`max_reach` 1.1 m·`min_catch_height` 0.1 m 에서의 값 — 격자를 넓히면 다시 읽는다.
  7. D-27 의 "성능 게이트를 우선순위로" 는 S3.6 시점 런타임 구현이 없었다 — 판정으로 남는 게이트는 S6-B 에서 정했다.

**T_det 실측 (2026-09-22, S3.6 병행 task) `[sim 실측 — L 은 여전히 가정값]`.** 도구 `rtc_tools vision_lane_probe` + `analyze_vision_lane` 의 `detection_latencies`. `ur5e_p1b` headless bring-up (`max_rtf` 1.0) + `sim_estimator_node` (당시 profile 0.95 s / 19 점, 측정 공분산 2.5e-5 m²), `/sim/launch_ball_at` (D-14) 기준 투척 24 발 (`p₀` (1.0, 0, 0.2), `v₀` (−2.375, 0, 4.1136)).

- **정의.** T_det = 발사 (비행의 첫 truth 샘플 — park 중에는 truth 가 발행되지 않는다) → 첫 VALID 예측, 양자화 100 Hz. 축은 D-2 대로 한 시계씩: `recv` (프로브 steady) 와 `stamp` (발행자). 유령 트랙 (TBD-VIS-07) 은 세지 않는다. **H_req 에는 `stamp` 축**이 들어간다 (L 이 전송·발행을 이미 센다).
- **결과.** `stamp` min 0.024 · p05 0.027 · p50 0.075 · p95 0.322 · max 0.690 s (`recv` 0.056 · 0.059 · 0.133 · 0.363 · 0.729). S0.7 가정 0.10 s 는 중앙값 근처였고 첫 계획 p50 0.215 s ≈ 지도의 0.24 s — 지도 판정은 중앙값 기준 유효. 0.95 s / 19 점 profile 은 실제로 구성·발행됐다 (VALID 944 · 부분 무효 0).
- **꼬리의 원인과 해소 (rtc 쪽 sim 공 lane 결함).** 발행 gate 는 sim time 인데 stamp 가 wall `now()` 라 stepper 버스트가 stamp 에 실렸고 (간격 25 ms 초과 9.2 %), ball_perception init (`min_points` 5 / `max_span_s` 0.1 s) 이 재시도해 T_det 이 늘어났다. **해소 (사용자 결정 ⑤, 2026-09-22 구현)**: 두 공 토픽의 stamp 를 발사 순간 기준 sim 시간축을 wall 에 얹은 값으로 (`ProjectileBallSample::stamp_steady_ns`, `rtc_mujoco_sim` README §Projectile Ball stamp; D-2·D-3 불변). 남는 비행 안 위상 오차는 양방향이라 sim 은 profile `max_future_skew_s` 0.1·`io.future_tol` sim overlay 0.1 로 덮는다.
- **실측 lead 로 지도 재실행** (첫 계획 0.215 s): `ur5e_p1b` kinematic 은 종전과 같고 열린 투척 `torque` **652 / 2835 (23.0 %)** · `box` 9. 첫 계획 시각에 의존하는 게이트는 도달 예산·commit 여유뿐이라 한 지도에서 정확히 재계산된다:

| 첫 계획 [s] | 0.164 (T_det min) | 0.215 (p50) | 0.240 (가정) | 0.30 | 0.40 | 0.44 | 0.45 |
|---|---|---|---|---|---|---|---|
| 열린 투척 `torque` | 733 | 652 | 590 | 353 | 45 | 10 | **0** |

- **이 표는 하한이다** — 지도는 대기 자세 정지 출발 단일 계획이고 런타임은 현재 명령 상태에서 재며 교체한다 (L3 §4.3, §4.7; 측정은 S8 "연속 재계획 측정"). 단 검출이 늦은 시행은 정지 출발이 실제와 같다 (수정 전 첫 계획 0.30 s 초과 33 %·0.45 s 초과 8 % — 재실측에서 해소).
- **G8-D2 (`iiwa7_leap`) — 지도가 비어 있지 않다.** 첫 계획 0.215 s 에서 `torque` **11 / 2835** (`box` 0, t_c 0.444–0.468 s) → **`NOT_EVALUATED(선행시간)` 이 아니라 평가 대상** (판정은 S8). 창이 좁다: lead **0.14 s 면 65 · 0.19 s 면 27 · 0.24 s 면 0**. L 가정값이라 provisional.
- **그 전에 고친 설정 회귀.** `iiwa7_leap` YAML 에서 `planner.ik` (`k_manip` 0.5 · `max_iter` 40) 과 `planner.catchability` (`arm_5row` **0.174**) 가 PR [#559](https://github.com/hyujun/rtc-framework/pull/559) 에서 빠져 `ParseCatchingParams` 가 경고 없이 기본값 (`k_manip` 0, 임계 0.1) 을 썼다 (`CheckActiveTbd` 는 부재를 못 잡는다) — 지도 수용 2178 → 2802 투척. 복원 후 S3.5b 의 2178 투척 / 15313 후보 / `v_max` 2.0198 재현으로 검증. 영향은 지도뿐 (런타임 소비는 S6.2).

**T_det 재실측 (2026-09-22, 공 lane stamp 수정 후) `[sim 실측 — L 은 여전히 가정값]`.** 같은 rig·24 발, profile 1.0 s / 20 점. stamp 간격 p05 / p50 / p95 10.00 ms · max 40 ms, 25 ms 초과 2 / 9164 (0.02 %); 수신 간격 버스트 (p95 39 ms) 는 남아 있다.

| 축 | min | p05 | p50 | p95 | max |
|---|---|---|---|---|---|
| `recv` (소비자 체감) | 0.032 | 0.033 | 0.058 | 0.110 | 0.115 |
| `stamp` (발행자, H_req 입력) | **0.040** | 0.040 | 0.050 | 0.090 | 0.100 |

- 24 투척 전부 검출, 꼬리 소멸 (중앙값 0.050 s = estimator 초기화 창 5 점 × 10 ms). **첫 계획 시각 min 0.180 · p50 0.190 · p95 0.230 · max 0.240 s** — 전부 지도 가정 0.24 s 안 (수정 전 54 %) 이라 `ur5e_p1b` 열린 투척 ≥ 652, `iiwa7_leap` 은 0.19 s 27 · 0.215 s 11 로 **G8-D2 평가 대상이 모든 비행에서 유지**된다.
- **stamp−wall 차이는 비행별로 재야 한다 (D-3).** 발사 오프셋은 10 ms 폭 안, 비행 안 위상 오차는 대부분 ±50 ms. 두 비행에서 sim 이 정지 뒤 따라잡아 (RTF 0.3–0.4x → 1.8x) stamp 가 wall 을 최대 1.9 s · 0.36 s 앞섰고 `max_future_skew_s` 초과 샘플이 버려졌다 — D-3 게이트가 무효로 걸러야 하는 시행이다 (§5). 원자료는 repo 밖 — 설정과 `analyze_vision_lane`·`catch_gate_map` 인자가 SSoT.

#### S5 포구 컨트롤러 골격·입력·추종 (Adding a New Controller)

**완료 (2026-09-23 머지, PR #564 → `4c0fb751`).** 착수 조건 (S0.6·S0.8, `[CONCERN] E-8`, D-24) 은 2026-09-22 충족 (E-8 승인 · D-24 (a)); G5-E substrate·QP solve time 예산도 같은 날 확정 (§7.3). 정의:

- S5.1 컨트롤러 완성 (S4.0 확장: YAML, lifecycle, 재무장). **E-STOP·fault 최소 계약 (P-1, E-8 — 승인 2026-09-22 사용자, 문구 그대로)**: (a) `TriggerEstop`·`ClearEstop`·`ResetFault`·`ResetTargetInitialization` 훅은 atomic 요청·epoch 만 갱신하고 reset 의 유일 writer 는 RT tick, (b) 해제 후 자동 재개 금지 — q_c·CLIK 앵커를 q_meas 로 reseed, (c) plan·궤적·공분산·손·FSM·타이머 무효화는 D-23 순서로, (d) `ClearEstop` 은 컨트롤러 fault 를 풀지 않는다 (base 계약: 두 경로는 별개). 전체 물리 정책은 S9
- S5.2 PointCloud2 구독 (nrt, `KEEP_LAST(1)`) → 필드 이름 파서 → SeqLock 스냅샷 (공분산 제외, A-3), D-2 변환, `generation`/`validity`/`snapshot_sequence` — C-1 (한 점이라도 무효면 거부) 과 되감김 정책은 S5.2 소유. 스냅샷에 `ActivationGeneration()`·token (D-22, D-23), 지평 부족 진단 (D-15), 공분산은 같은 token 의 계획기 버퍼. S5.2e RT 상태 POD 에 D-24 센서 freshness
- S5.3 스트리밍 기준 → 확장 CLIK → 팔 명령, QP 비의존 관절공간 abort, 예측 선행 보상 (L5.8), backend 왕복 대조 (L5.10)
- S5.4 CSV 로그·상태 메시지 신설 (D-20, `rtc_msgs`, E-3·PROC-3; `PublishRole` 을 늘리지 않음, E-11). S5~S9 필드 superset 동결, 모든 early-return 분기에서 Store 하고 계산 안 한 필드는 무효화 (PROC-7)
- S5.5 ground truth 고정 포구점 (oracle plan) 으로 추종 검증

| 게이트 | PASS 기준 | 판정 |
|---|---|---|
| 추종 | G4-H, G5-A~B2, G5-C3, G5-C4 | G5-A·G5-B PASS (결과) |
| RT | 할당 0 (G1-D, G5-C), QP solve time 예산 **p99 ≤ 400 µs (20 %) · 최대 ≤ 1500 µs (75 %)** — 500 Hz tick 2000 µs 기준, 분위수로 건다 (확정 2026-09-22 사용자, provisional; S1.9 의 1777 µs 는 오프라인 IK 한 번이지 tick 당 CLIK QP 가 아니다, L5 §9 G5-C) | **G5-C PASS** |
| 입력 | G1-A~C, G1-E, G1-F, G1-I (backlog 뒤 최신 `snapshot_sequence` 만 수락, ARCH-6) | PASS |
| SeqLock | 찢어진 스냅샷 0·최악 재시도 기록 (G1-C), writer 끼어들기에서 최신 누락 0 (G1-H, D-21) | PASS |
| activation | 비활성 중 받은 궤적을 재활성 첫 tick 에 소비 안 함 (G1-J, D-23) | PASS |
| E-8 | G7-H (a) 재activate 첫 tick 이 옛 자세 명령 안 함 (b) race 에서 reset writer 는 RT tick 하나 (c) 자동 재개 0 (d) `ClearEstop` 후 latched fault 유지; `/security-review` | (d) PASS (S5.3) |
| PROC-7 | G8-H: early-return 분기마다 그 tick body 가 실림 | **PASS** — `Compute()` 단일 exit + tick 머리 기본 생성으로 구조적 (`DemoCatchingRecord.*`, positive control red) |
| 선행 보상 | G5-C2 (backend 왕복), G5-E (지연 에뮬레이션 전후) | **(ㄱ) fixture 전용 지연 주입** (§7.3 "G5-E substrate"; `sim.ball.*` 레인 선례, S3.7 제외로 없던 substrate). **G5-E PASS**, G5-C2 남음 |
| PROC-3 | `rtc_msgs`·`rtc_base` 변경 후 전체 빌드·테스트 | **PASS** (5518 / 0) |
| GUI·plot | §13 S5 행 | **PASS** |

**결과 (2026-09-23, PR #564 → `4c0fb751`).**

- P-1 (a)~(d) · 무장 latch `catching.enable` · A-S5-1 실기 park, TSAN clean; 이름 기반 파서 + SeqLock + A-S5-4 되감김 정책, G1-A~C·E·F·H·I·J green, ASan/UBSan clean. `rtc_msgs/CatchingState` + `catching_diag.csv` 는 같은 POD 한 벌에서 나온다.
- S5.3 실 모델 폐루프: G5-A (정지 목표 1.4 s 에 < 1 mm · < 0.5°, 독립 pinocchio 오라클) · G5-B (box 200 tick 위반 0) · G5-C (실측 p99·max 가 예산 안) · G5-E (지연 50 ms 에서 77 → 71 mm) · G7-H (d) (QP 실패 streak → FAULT 래치).
- **S5.5 PASS** (sim `260922_2249`, 공 6회): oracle 포구점 대비 0.48 mm · 축 0.057°, 241639 행 tick 간극 0, solve median 21.7 / p99 79.4 / max 1411.5 µs, CLIK 미수렴 0. `headless=false` 재실행 (`260923_0021`): 0.36 mm 정착, 단 콜드 스타트 3 회 중 2 회 `track_err_abort` 0.3 초과로 ABORT_SAFE (sim 서보 lag — 이 임계는 S6 #537 결정 ① 에서 바뀌었다, §4.4 S6). 세션은 삭제, 수치는 #537 코멘트.
- CLIK 과제공간 스윕 (`test_catching_clik_sweep`, 4 × 3 × 3): solver 건강 (미수렴 0 · solve max 72 µs) 이나 포구 자세 1 mm 정착에 1.43 s 대 공 예산 0.25–0.6 s — 제약은 시간이고 S6 의 t_c·대기 자세 선택 입력.
- 착수 중 블로커: 출하 프로파일에 `reference:` 블록이 없어 ur5e_p1b sim 전체가 configure 실패 → `reference.provisional` (invented key) 로 닫음. **`/code-review` 9 건 전부 실제 결함** — 가장 큰 것은 운동 중 무장 해제가 관절 명령을 한 tick 만에 세우던 전이표 간극 (A-S5-13; 나머지 A-S5-14~16).
- **남은 게이트: G5-C2 (backend 왕복).**

#### S6 계획기 스레드 (D-7)

**완료 (2026-09-23, PR #568 → `9bf404a8`).** 착수 조건 (S5, S3.6, `[CONCERN] E-7` — §6) 충족, 착수 결정은 §7.3 "S6 착수 시 확정". 정의:

- **D-27 런타임 설계.** reachable 하면 도전 — 성능 게이트 (γ 창·도달시간·commit 선행) 탈락은 제거가 아니라 **순위·진단**. 정한 것: 판정으로 남는 게이트 (정지점·작업공간은 E-8 접점), 통과 후보가 없을 때의 순위 함수·`[CONCERN]`, 놓친 시행의 진단 코드 (S8 은 시도/성공 분리 보고). 지도 기준은 불변. 구현 S6-B
- S6.1 골격: **새 layout role 없이 `mpc` role 재사용** (E-7 결정 J — §6), activate 게이트 (`planner.enabled` && `mpc_off` 면 FAILURE), `PlannerRtState`·`plan_box_` SeqLock (oracle 은 같은 box 의 대체 writer), A-S5-12 (sim park). S6.2 포구 자세 IK·catchability (스레드 전용 모델 handle, 전멸 시 사유 코드). S6.3 γ 창·rollout, 예산 초과 시 coarse-to-fine. S6.4 후보 선택·hysteresis·commit/freeze, `PlanSnapshot` (token·`publish_ns`, D-22) 게시 전 token 재검사
- S6.5 D-7a 측정 (§7.2) — **생략 (사용자, 2026-09-23)**: 초기값 FIFO 유지
- S6.6 NLP 전환 대비 (A-4, §8): 탐색 전략을 단일 진입 함수 뒤에 둔 경계 유지, 전환 신호 (G3-G, 계획 성공률, 예산 초과율) 를 S6·S8 에서 기록
- ~~S3.1b D-3 부하 재검증~~ — **S6 에서 수행되지 않았다** (2026-09-24 확인). S8 시행에서 clock lane 을 켜 누적 ≥ 200 으로 수행 (§4.4 S8, §5)

| 게이트 | PASS 기준 | 판정 입력·결과 |
|---|---|---|
| 계획 | G3-A~E, G3-C 예산 준수, **R-2** 연산시간 실측 (후보당 IK · 필터 후 후보 수 · `PlanOnce` p50/p99/max · 초과율) | `planner.budget_s` 0.020 (L3 §6). G3-C 합성 1000 투구 p99 < `budget_s` |
| IK·catchability | G3-G (수렴률), G3-I (런타임·지도 동치) | — |
| RT | G3-K (할당 0·noexcept·로깅 없음) | — |
| token·race | G3-L: eventfd coalescing, 계산 중 새 스냅샷, 옛 plan 게시, deactivate·Pause race 에서 대체된 plan 소비 0 (D-22, D-23) | — |
| 스레드 | 결정 J 로 layout 변경 0, 대신 **R-1**: `SwitchController` 로 `mpc_main` 두 TID 의 affinity·Paused/Running 직접 검사 | **PASS** (결과) |
| D-7a | G3-J — 제어 PC 전용 | **생략 (사용자, 2026-09-23)** |
| D-3 부하 | §5 판정 | S3.1b 미수행 → S8 |
| GUI·plot | §13 S6 행 | — |

**결과 (2026-09-23, PR #568 → `9bf404a8`).**

- **S6-A** 골격 (eventfd 대기 · `JudgePlan` (a)~(f) · 키 4개). **R-1**: 단위 `test_catching_mpc_role_switch` (두 `mpc_main` 같은 CPU, switch 뒤 하나만 running, policy `NOT_EVALUATED(EPERM)`) + sim `sim_iiwa7_leap` wbc↔catching (계획기만 ~20 Hz wake, `enable_mpc:=false` 면 switch `ok=False`); 검증기 rc=2 는 dev PC 권한 탓이다.
- **S6-B** 판정/순위 게이트 분리·점수 J·교체·동결·catch sub-model·계획기 이벤트 CSV·GUI. **S6-C** γ rollout (coarse-to-fine), vision world → 모델 world 변환을 궤적 수신에서. **S6-C2** CLIK 가속 제약 `box|kinematic|dynamic` (결정 K), L3 §4.7 런타임 전환 규칙. **S6-D** (D-7a) 생략 — layout 초기값 (tier ≥ 8 `mpc_main` FIFO 60 · slot 3), 재판정 시 `s6d.sh` (#537 코멘트 5793878042) 로 §7.2 적용.
- **#537 결정 (2026-09-23).** ① p1b `track_err_abort` **1.54 rad** (#566 뒤 t_c 전 최대 0.772 rad × 2). ② p1b `accel_constraint: dynamic` (`eta_tau` 0.8; leap 은 box·0.3) (근거는 아래 "#566 위 재측정"). ③a 오프라인 진단 (#537 코멘트 5789859038) 이 sim 버그 #566 (§2 시간 항목) 을 찾아 PR #567 로 고쳤고 S6 sim 수치는 그 위에서 재측정했다. ④ 투척 드라이버 `catching_sim_trials` (투척마다 wait_pose 정렬 — S7 에서 제거, C-12). ⑥ `planner.switch.e_jump_max`·`ed_jump_max` → 가속 예산 `planner.switch.eta_jump` (L3 §4.7 규칙 2, 옛 키 거부). ⑦ (a): `refreshed` 0 — 첫 plan lead 0.42 s < T_w 0.6 s 라 램프 재시작 계단 ≈ 11 m/s² 가 예산 5.25 를 넘는다 (규칙이 맞게 거부) → **램프 연속 채택은 S8 로 이월**.
- **#566 위 재측정** (dynamic 50 투 · box 25 투, #537 코멘트 5793765020): CLIK 추종 p50/p95 dynamic 2.7 / 72 mm 대 box 128 / 561 mm → ② 유지; t_c 오차는 sim 서보 지연 (~125 mm) 이 지배.
- 착수 중 수정 3 건: `ApplyThreadConfig` EPERM 시 이름 누락 (rtc_base) · `rt_layout_profile` 이 컨트롤러에 안 닿아 #350 activate 게이트가 죽어 있음 (CM) · leap YAML 의 안 읽히던 `planner.ik` 사본. `/code-review` 10 건 반영 (스레드를 configuration 에 묶고 on_cleanup join). PROC-3 5711 / 0 실패 (77 skip).
- **이월**: ⑦ 램프 연속 → S8, S3.1b → S8.

#### S7 손 시퀀서·슈퍼바이저

**완료 (2026-09-24, PR #571 → `c61fd32a`).** 착수 조건 (S6, `[SPRINT] S7`, `[CONCERN] E-8` — §7.3 "S7 착수 전·중 확정") 충족, 커밋 1 은 docs 정정. 아래는 구현한 설계 (#537 S7 결정 2026-09-23) 이고 현행 전문은 L6·L7 이 갖는다.

- **S7.1 손 시퀀서 → 손 device slot.** `q_open` 은 IDLE homing 중에만, 팔이 `wait_pose` 에 도착하면 ARMED~COMMITTED 내내 `q_pre` — 시각 기반 preshape (`PreshapeDue`) 와 `T_pre` 키는 폐기 (L7 §4.5 조건 6 = 손 `q_pre` 도달). 위상: Open(homing) → Preshape → Close(t_cmd) → Hold → Release(목표 `q_pre`) → Preshape. 모드별: 비무장 IDLE latch · 무장 IDLE homing q_open (이미 `pose_tol` 안이면 q_pre) · ARMED~COMMITTED q_pre · CLOSING/DECEL/HOLD Close→Hold · ABORT_SAFE 진행 중 phase 계속 · FAULT 마지막 출력 유지 · 시행 후 disarm 은 마지막 출력을 latch · E-STOP·activation 리셋은 시퀀서 inactive + 측정 자세 latch (S9 이월).
- **S7.2 FSM driver** (전이표 불변, L7 §4.1). homing·retreat 는 **관절공간** per-joint 사다리꼴 (QP/CLIK 비의존), homing 은 무장 latch 필요 (P-1 (e)), 이미 `pose_tol` 안이면 생략. **driver 규칙 — 전문 L7 §4.1 "S7.2 driver 규칙"**: R-PREC (한 tick 의 사유 우선순위 고정, ESTOP 최우선) · R-ORDER (IDLE·DECEL·HOLD·RETREAT 는 vision 판정 앞, COMMITTED/CLOSING 은 항상 법칙 실행) · R-IDLE (carried 속도는 `JointSpaceDecelStep` 으로 정지) · R-WATCHDOG (homing·retreat 도 `track_err_abort`) · R-DECEL-ENTRY (tick 의 now 를 한 번 읽어 공유) · R-ADMIT (`JudgePlan` (g) `t_c − now ≤ T_freeze` 면 `kTooLate`, L3 §4.11) · R-CLOSE (`now ≥ t_cmd − h/2`, `HandCommandDueRounded`) · R-TRACK (동결 후 샘플은 commit 시점 `generation` 에 고정). COMMITTED 이후 stale 은 동결 plan 으로 계속, `supervisor.stale_committed_max_s` 초과 시 ABORT_SAFE (A-6).
- **S7.3** 접촉 판정 (finger-on-object), `TIP_STALE` (D-24), 감속, 충격량 예산. 바이어스·잡음 **EMA**, 디바운서는 지문 샘플 (`inference_sequence`) 단위, 표본 부족이면 `Undetermined`, `REF_SATURATED` 는 연속 `supervisor.sat_ticks` tick — L7 §4.2·§4.4.
- **S7.4** 리셋을 `ResetForRearm()` 과 `ResetTrialState()` 로 분리 (멤버별 소유·면제는 L7 §4.8 표). **RETREAT 순서** (release 규칙은 2026-09-24 교체): 진입 → 정지 램프 → 관절공간 복귀 → 대기 자세 도착에서 손 Release(q_pre) → `q_tol` → `ResetForRearm` → ARMED; 복귀 중에는 판정과 무관하게 손을 열지 않는다 (지문 판정의 Missed 는 "공 없음" 이 아니다). **IDLE 순서**: 무장 → 밖이면 q_open → homing → q_pre, 안이면 q_pre 만 → ARMED. 전문 L7 §4.8.
- **PROC-6/E-6 고지 목록 (착수 커밋 1 에서 확정)**: fixture `planner.wait_pose` = fixture home 으로 대부분 회피하나 (i) `test_catching_tracking.cpp` 의 700-tick APPROACH 단언, (iii) `CatchingTorqueTest` 의 포화 유도 중 매 tick APPROACH 단언, (iv) fault 사이클 tick 수 단언은 회피가 안 되어 별도 spec 변경 커밋 + 근거 대상이다. 그 커밋은 코드 변경 앞에 따로 들어갔다 — `ab5c8c09` (S5/S6 law suite: 단언은 그대로, 입력만 새 spec), `4cd5730f` (RETREAT 개방 규칙·`sat_ticks` 50), `13153df6` (`sat_ticks` 50 → 60).

| 게이트 | PASS 기준 | 판정 입력·결과 |
|---|---|---|
| FSM | G7-A, G7-B, G7-D | **PASS** |
| 센서 stale | G7-G: 지문 dropout 에서 `TIP_STALE` 또는 `Undetermined`, 옛 힘 판정 0 | D-24. **PASS** |
| 접촉 | G7-C (오경보율 기록) | `supervisor.contact.f_min` **0.2 N** (사용자 값); sim 지문 무잡음이라 `k_sigma` 는 실기 전용, 오경보율은 합성 가우시안 |
| 충격·토크 | G7-B3 | 손 토크 권위 한계 (D-12, §7.3 — sim 3.0 · 실기 1.5 N·m provisional): 확정 S10 (#613) 전까지 토크 비교 `NOT_EVALUATED`. 충격량 상관 → S8 |
| 재무장 | G8-A2, 리셋 표 완전성 + 런타임 poison 테스트 | **PASS** |
| 결과 판정 | G7-E | 기록 (결과) |
| PROC-7 | G8-H 를 S7 분기 (COMMITTED stale·DECEL/HOLD·homing 중 disarm·RETREAT 중 E-STOP) 까지 확장 | — |
| GUI·plot | §13 S7 행 | 사용자 육안 확인 완료 |

**결과 (2026-09-24, PR #571 → `c61fd32a`).** 한 투척이 한 순환을 돈다: IDLE(homing) → ARMED → TRACKING → APPROACH → COMMITTED($t_c-T_{freeze}$) → CLOSING($t_{cmd}$) → DECEL($t_c$) → HOLD($T_{hold}$) → RETREAT → ARMED.

- **단위 PASS**: G6-A (합성 1000 시행 ±h/2, mutation red) · G7-A (시나리오 21) · G7-B (DECEL 진입 $e=\dot e=0$) · G7-D (할당 0, 최악 tick 152 µs) · G7-G · G8-A2 (리셋 표 린터 + poison probe + 재무장 floor 회귀).
- **sim `260923_2336`** (ur5e_p1b, 25 투척): 순환 25/25, G6-A ∈ [−0.95, −0.55] ms, 판정 25 전부 Missed · 참값 일치 24/25 (G7-E). 불일치 1 건 (손바닥에 얹힌 공이 Missed → 즉시 release → 낙하) 이 RETREAT release 규칙 교체의 근거. 정지 바이어스 잡음 0 (C-20: f_min 만 유효).
- **재측정 `260924_0013`**: 순환 25/25, Captured 1 · Missed 23 · Aborted 1, 참값 일치 23/24 (손바닥 위 공 false-Missed 1), release 25 회 모두 RETREAT 안·대기 자세에서 (측정 편차 최대 0.0041 rad). G6-A 22 회 [−0.98, −0.55] ms, 3 회 −3.04 · −4.09 · −4.12 ms 는 닫힘 tick 자체가 늦게 돈 비-RT 호스트 효과다 (규칙은 첫 만족 tick 에 발행; 판정은 단위 G6-A).
- **`supervisor.sat_ticks` 60 (provisional, 사용자 2026-09-24).** `ref_saturated` 연속 길이 max 44 · p99 42.5 · median 11 tick (`260923_2336`; 100 → 50 으로 내림) → 재측정 (`260924_0013`) max 50 · p99 47.6 · median 11 에서 50 이 1 회 발화 (CLOSING → ABORT_SAFE, 공은 이미 빗나감) → 60.
- **리뷰**: `/security-review` Critical 0, `/code-review` 10 건 중 6 건 수정 (ABORT_SAFE 탈출·판정 덮어쓰기·RETREAT 정지의 서보 대기·읽기 불가 팔의 track_err·hold NaN·`delta_rad` 검사 범위). 드라이버 외부 정렬 제거 (C-12).
- **이월** (사용자 결정 2026-09-24) → S8 (#537 코멘트 5804113629, 목록은 §4.4 S8 "이월 받은 것"). 포획 0/25 의 원인은 공이 $t_c$ 약 40 ms 전 도달한 것 (sim 서보 지연). 손 토크 비교 (D-12) → S10 (#613).

#### S8 sim 통합 평가

- **ur5e_p1b → iiwa7_leap** (2026-09-22 결정으로 `ur5e_p1b` 가 1차, G8-D2 는 조건부 — §1a·§7.3). Wilson CI, NEES, 소거실험(γ, lead). **γ 포화 빈도 측정 → D-8 재검토 입력**
- **상태: 완료 (2026-09-27).** A substrate → B lead 보상 → C 조건부 RT → D leap → E 본 평가 → F 손 근처 투척 (탐색, 게이트 밖), 이어 S8-F-1 후속 ②→①→④→③ (2026-09-26 사용자, #537 5846528236): ② S8-G · ① S8-H (D-S8-19 종결) · ④ S8-I (D-S8-20 종결) · ③ 손 반발 흡수는 진행하지 않음 (사용자) → S9. PR: S8-B #575 → `ac34893a` · S8-C #579 → `126e3513` · S8-D #581 · S8-E #582 → `9e7967b5` · S8-F-1 #583 → `3278963a` · S8-G #584 → `769f99e9` · S8-I #586 → `a471d8b8` · G8-B profile #595 → `482d18b3`. S8-B 후속 issue #574.
- **이월 받은 것.** S7 (#537 5804113629): 포획 0/25 · 지문만 보는 판정의 false-Missed · `sat_ticks`·`stale_committed_max_s` 최종값 · G7-B3 · 손 $q_{pre}$ 대기 timeout 부재 · ν̄ · σ_ℓ 소비자 · CLOSING 1 tick 늦은 기록 → D-S8-3~10. S6 (결정 ⑦, #537 5793765020): 교체 채택 시 γ 램프 연속 (없으면 `refreshed` 가 램프 중 거부돼 첫 plan 만 쓴다, L3 §4.7) → D-S8-5 트리거 미충족으로 하지 않음.
- **준비 대조 (2026-09-24, 코드 대조 C-1~C-24).** sim 팔은 지연 0 이 아니다 — MJCF PD kd/kp = 0.2 s 의 1차 지연 (L5 §4.6) 이 S6 서보 성분 ~125 mm 와 포획 0/25 의 원인. 러너는 `sim.throw_region` 을 안 읽는다 (D-S8-2). S3.1b 는 S8 에서 clock lane 으로 잰다. NEES 는 ball_perception_sim `sim_capture_evaluate` 재사용 (P5). τ 0.2 는 `io.horizon_min` 0.51 → 0.66 등을 요구했다 (C-25) — D-S8-13 으로 은퇴. 파일럿 `260924_1218` (25 발, lead off): τ̂ 200 ms · 서보 122 mm · CLIK 2 mm · 25/25 Missed.

**S8 확정 결정 (2026-09-24 사용자, #537 코멘트 5804952754 → 5807767896; 이후 행은 날짜 표기).**

| ID | 확정 |
|---|---|
| D-S8-1 (a) + C-25 (a) | sim 서보 지연은 **L5 §4.5 선행 보상을 sim overlay 로** (출하 YAML 불변); lead-off arm 도 같은 T_freeze. 기각 (c) 정확 역보상 $u=q_d+\tau\dot q_d$ — S10 이 쓸 순수지연 기구를 검증하지 않는다. τ 0.2 overlay 집합 (T_freeze 0.52 등) 은 **D-S8-13 으로 은퇴** |
| D-S8-2 (a) | 동결 분포 = 로봇별 gate 지도 상자 (p1b S3.5b 90 % 상자, leap S8-D 상자 — D-S8-15), `catchability_map` 표본 재사용 (P5), seed 재현. 기준 투척 반복은 iid 가 아니라 회귀 세트로만 |
| D-S8-3 | floor **0.35** (D-12 — 사용자 사전 고정 2026-09-24; 0.5 provisional (#537 5808613906) 을 S8-B 튜닝 p̂ 0.47 을 보고 하향), **2026-09-26 동결 (D-S8-16)**; **n_valid 200**. 기각 "floor = p̂ − 0.2" — 순환 (Monte Carlo 로 참 p 0.3/0.5/0.7/0.9 에서 통과율 0.72/0.71/0.70/0.80). 튜닝 데이터 비합산, 조기 종료 없음 |
| D-S8-4 (c) | D-3 δ 는 **판정이 아니라 공변량** — n 25–50 에선 검정력이 없어 같은 seed 부하 A/B 로 상계. 무효 = rig 실패만, plan 없음·abort 는 실패, ITT 하한 병기 |
| D-S8-5 (a) | ⑦ 램프 연속은 조건부 — `held_jump` ‖Δp_c‖ p95 > 12 mm 또는 APPROACH 교체 ≥ 20 %. S8-B 에서 미충족 → 하지 않음 |
| D-S8-6 (a) | RETREAT 손 $q_{pre}$ 대기 timeout `{kRetreat, kHandTimeout, kIdle}` + 같은 tick disarm, 키 `robot.hand.T_release_timeout`; E-STOP 경로 아님 (E-8 비해당, `/security-review` 수행). 시계는 `kReturn → kRelease` tick 시작. RETREAT → IDLE 이 trial 상태를 리셋하지 않던 결함도 닫음 (`ResetTrialScope`, L7 §4.8) |
| D-S8-7 (a) | ν̄ (`PRED_INCONSISTENT`)·`io.pred.nu_reg` 은퇴 — 예측 일관성은 `/ball_perception/debug/{innovation,nis}` 를 오프라인으로; σ_ℓ 는 `planner_events.csv` 기록만 |
| D-S8-8 (b) | false-Missed (링크·손바닥 위 공) 대책 = 지문 합의에 **손 관절 q·토크 증거를 OR 로** (2026-09-25 사용자; robot-agnostic, ARCH-1). 판정식·임계는 S8-C. 기각: min-ρ + 전 관절 토크 median · `contact.t_confirm` 재사용 |
| D-S8-9 (a) | p1b 먼저, leap 은 S8-B 뒤. γ0 arm = `gamma.grid: [0.0]` + **`planner.hand.d_eff: 10.0`** (#537 5809415678 — grid 만으론 공 > 1.0 m/s 에서 γ_min > 0; d_eff 가 빼는 것은 γ 창 순위 벌점 (D-27) 뿐) |
| D-S8-10 (a) | CLOSING 1 tick 늦은 기록은 문서 기록만 (시각 무영향) |
| D-S8-11 | 공 arm 은 `beanbag` preset 그대로 — 반발에 마찰·비틀림·구름도 바뀌므로 **반발 + 마찰 복합 효과**로 보고. `tennis_soft` 는 만들지 않는다 |
| D-S8-12 | 손 근처 투척 = **S8-F** (탐색, 게이트 밖) |
| D-S8-13 (2026-09-24 사용자, #537 5808985333 → 5809415678) | **sim 팔 플랜트 τ 0.05 s** — `config/ur5e_p1b/mujoco_simulator.yaml` 서보 게인 (`use_yaml_servo_gains: true`, 팔 kp ×4). 이유: τ 0.2 는 T_freeze 0.52 를 요구해 S3.5b 상자를 0/180 으로 닫고, τ 0.05 = 설계 T_arm 이라 163/180 이 열린다. overlay `joint_cmd.lag.{T_arm: 0.05, lead_enable}` + **`T_freeze: 0.37`**. **sim 플랜트는 선택이지 UR5e 값이 아니다** — G8-D 는 "서보 지연 ≈ T_arm" 조건부, S10 이 닫는다. 대안 τ 0.025 (kp ×8, 또는 kp ×12·kd ×1.5 — τ̂ 25–26 ms · 포화 0 · s35b 163/180) 도 가능했으나 **0.05 유지** (사용자 2026-09-29). 성공률이 너무 낮게 나오면 PD 게인을 바꿔 τ 를 달리한 시험을 할 수 있다 — 그때 τ 0.05 위의 S8 수치는 재측정 대상이다 |
| D-S8-14 (2026-09-26 사용자, #537 5835585007 → 5835714797) | leap 묶음: 팔 kp ×4 → τ 0.05 · 지도 라운드 최다 열림 자세를 출하 `planner.wait_pose` 로 · S8-D 는 검증 100 발 보고만 (판정은 S8-E) · profile p1b 동일 · overlay `catch_lead_on` (T_freeze 0.19) · seed 스모크 505 · 교정 501·502 · 검증 503·504 |
| D-S8-15 (2026-09-26 사용자, #537 5835714797 → 5835793705) | leap 상자 = p1b 폭 상자 중 **열림 비율 최대**, 하한 50 % (p1b 는 ≥ 90 %). **유효성 조건 (사후 승인 #537 5841231282)**: 상승 구간 (정점 + 20 ms) 에 대기 자세 로봇과 2 cm 이상 떨어진 투척만 (지도는 비행 중 충돌을 안 본다). p̂ = 상자 전체 1차 + 지도-열림 부분집합 공변량 |
| D-S8-16 (2026-09-26 사용자, #537 5841469779 → 5841651825) | **S8-E 본 평가 계획** (아래 상세). **floor 0.35 동결** |
| D-S8-17 (2026-09-26 사용자, #537 5842632283 → 5843117293) | **host 부하 unit 재실행** — 결과 게시 뒤·재실행 전 선언. `rtf_trial_min` < 0.95 시행이 있는 unit 은 rig 실패 → 같은 seed 재실행 (원본 `<unit>.loaded`, 최대 4 회 후 `NOT_EVALUATED(host 부하)`). 이유: 기전이 rig 이고 규칙이 결과를 안 본다. **원 판정·재실행 판정 둘 다 기록.** leap 은 해당 없음 — **G8-D2 는 이 규칙으로 바뀌지 않는다**. 기각: 원 판정만 · 시행 단위 `sim_slow` + 보충 seed (모집단이 바뀐다) |
| D-S8-18 (2026-09-27 사용자, #537 5847236227 R1–R4) | R1 출하 `reference.a_max` 21 → **30** (provisional, 실기 S10) · R2 계획기 도달시간 한계 = **실행 envelope** (sim 은 `sim.yaml` 이 `accel_limits_path` 를 `derived_accel_limits_s8g_envelope.yaml` 로; `robot.yaml` 은 D-16 유지) · R3 `reference.omega` **10 고정** · R4 **복제 규율** (판정 unit 같은 seed ≥ 2 회 + McNemar). 근거 S8-G |
| D-S8-19 (2026-09-27 사용자, #537 5847660329 권장 B) | **후속 ① (γ 창 재정의) 종결 — sprint 로 열지 않는다.** 창을 묶는 것은 포구 자세 v_dir,max 이고 계획기가 이미 γ_f = g_max 를 쓴다 (S8-H) |
| D-S8-20 (2026-09-27 사용자 Q1 = A → 기준 미달, #537 5852588785 · 5853132529) | **④ (대기 자세) 종결 — 출하 `planner.wait_pose` 유지.** robust 자세 P1' 은 기전을 실현했으나 포획 20 대 18/112 (p 0.86), 기준 (+10 발 & p < 0.05) 미달 (S8-I). 남김: `wait_pose_source: current` · `planner_events` γ 창 기록 · 도구 robust 목적 (점 DLS 는 쓰지 않는다). 실험 overlay 는 repo 밖 |
| 설계 (이의 없음) | G8-C·G8-E 는 **2×2 factorial** (lead × γ) · G8-D 성공 = **truth 기반** |

- **D-S8-16 상세.** ① 무효 = 시행별 `invalid_reason` 한 개, 우선순위 `srv_refused` (`accepted == false`) > `not_launched` (accepted ∧ `n_truth_rows == 0`) > `controller_silent` (시행 창 diag 0 행) > `lane_drop` (clock lane 에 그 `launch_seq` 없음 또는 `dropped_total` 증가) > `sim_stall` (`sim_time_sec` 간격 > 5 × step); 그 외 (plan 없음·abort·HAND_TIMEOUT·미종결) 는 실패. 기동 단위 실패는 같은 seed 로 unit 재실행 · ①b n_valid < 200 이면 사전 선언 보충 seed · ② 모든 unit 에 `record:=true` capture + `vision_lane_probe --dump` · ②b G8-C2 p̂_live = diag `input_snapshot_sequence` ↔ dump `snapshot_sequence` **정확 결합** (L8 §5.2) · ②c G8-B 원 판정식 = 시행 부트스트랩 CI 가 3 을 포함 (D-1 로 재정의) · ③ leap 도 새 seed n_valid 200 (S8-D 100 발은 판정에 안 씀) · ④ beanbag 은 같은 seed, McNemar · ⑤ S3.1b 는 S8-E 로 충족 · ⑥ seed p1b 601–604 (보충 605→), leap 701–704 (705→) · ⑦ §7.3 네 항목은 제안·기록만.

**S8-A substrate (2026-09-24 `[SPRINT]` 컨펌) — RT tick 바이트 동일.** 컨트롤러 read-only 미러 (`planner.wait_pose`·`T_freeze`·`joint_cmd.lag.{T_arm,lead_enable}` — overlay 값을 러너가 읽는다) · `catching_sim_trials --dist s35b` · `sim_lanes:=true` · `rtc_tools catching_trials` (Wilson·τ̂ LS·t_c 분해·A⊥B·충격량) · `vision_lane_probe --dump` · GUI 시행 카운트. 로봇 상수 없음 (ARCH-1). 골든 = 파일럿 재현 (τ̂ 199–204 ms · 서보 122.2 mm · truth 0/25). 파일럿 (`260924_1218`) 의 나머지 관측: 계획 γ_f 0.49–0.69 (DECEL 진입 tick 의 `ref_gamma` 1.0 은 계획값이 아니다) · 손 토크 최대 3.06 N·m = forcerange 클램프 · "지연 0 이면 12/25 가 r_cap 안" 은 포구율 예측이 아니라 필요조건 기하 검사다 (±120 ms 창 최근접 거리).

**S8-B lead 보상 (2026-09-24, PR #575 → `ac34893a`, #537 5810866922).** 2×2 × 4 블록 × 25 seed (arm 마다 재기동). 성공 lead on·계획 γ **47/100** · lead off·계획 γ 4 · lead on·γ0 2 · lead off·γ0 1. **G8-E PASS (sim)** — 서보 잔여 중앙값 2.6 vs 94.4 mm (p 3.9e-18). **G8-C** 47 vs 2 (d_eff overlay 교란 아님 확인). D-S8-5 미충족 (held Δp_c 는 `NOT_EVALUATED`). 튜닝 (seed 201–204) **94/200 = 0.47** ([0.40, 0.54]) → floor 0.35. 제안값 p1b `sat_ticks` **80** · `stale_committed_max_s` **0.10** · `track_err_abort` **0.42**. D-S8-8 측정: false-Captured 0, **false-Missed 가 truth 성공의 43–57 %**.

**S8-C 조건부 RT (2026-09-25, PR #579 → `126e3513`, SPRINT #537 5823089966 → 5823291259).** 손 관절 캡처 판정 (L7 §4.4): caging 관절이 ρ·$|\dot q|$·토크 비 조건을 `min_joints` 이상 `t_persist` 동안 만족하면 포획 (지문과 OR). 임계 **`rho_min` 0.4 · `rho_max` 0.95 · `effort_frac_min` 0.5 · `t_persist` 0.1**. 검증: false-Captured **0/52** · false-Missed **8/48** (지문만 22/48; 남은 6 은 관절 증거로 불가시) · **G7-E 92/100** · **G7-C** 빈 손 오경보 0/20000 (실기 창 통과 ≈ 0.54 — S10). `T_release_timeout` 기본값 = profile 유도 E2E 배수 (L6 §6: p1b 2.36 · leap 1.45 s); p1b 실측 2 × p99 = 1.03 s PASS. `capture` 블록 provisional. 새 테스트 mutation red (E-6 대상은 별도 spec 커밋), G7-D 할당 0 (손 증거·timeout 경로), 기존 assertion 무수정.

**S8-D leap (2026-09-26, PR #581; #537 5835585007 → 5835793705 · 5836802390 · 사후 승인 5841231282).** τ̂ **51.3–52.8 ms**. 지도 (첫 계획 0.215 s · d_eff 0.1047) 대기 자세 라운드가 수렴하지 않아 (7 축) 최다 열림 (244) 자세를 출하로 (값은 leap `demo_catching_controller.yaml` 의 `planner.wait_pose`). 최대 상자 (앙각 86–88°) 는 공이 상승 중 손에 부딪혀 유효성 조건 추가 → **동결 상자 0.95–1.05 m · 릴리스 0.10–0.20 m · ±6° · 2.85–3.05 m/s · 78–80°** (열림 164/180 — 첫 계획 0.215 s 기준; 실측 p50 0.195 s 에서는 176/180). 씬은 `scene_right.xml`. 실측 첫 계획 p50 **0.195** s 에서 지도가 비지 않아 **G8-D2 평가 대상**. 검증 **44/100** ([0.35, 0.54]; 판정은 S8-E), 슈퍼바이저 × truth 완전 일치, 캡처 false-Captured 0/56 · false-Missed 0/44.
- **G3-D** `plan_valid` p50 0.17, 교체 0/200. `track_err_abort` **0.48** · `T_release_timeout` **5.0 s** (유도 1.454 s 는 교정 37 회 timeout — 손 관절 감쇠·armature 가 서보 kd 에 더해진 느린 개방과, q_tol 0.01 이 MCP·엄지 마찰 사대역 (frictionloss/kp = 0.016 rad) 안에 있는 것; q_tol 은 기본값 유지) · `effort_frac_min` **0.33**. CLIK 27–29 mm (p1b 2 mm — 7 축 CLIK, 후속).

**S8-E 본 평가 (2026-09-26, PR #582 → `9e7967b5`: `f0ff9818` · `8baf18cc` · `79a025de` · `879957c0`; #537 5842632283 · 5843117293 · 5843232090).** tennis·beanbag (seed 601–604) · leap (701–704), 각 4 × 50 발. RT tick 무변경.
- **무효 0** → ITT = Wilson. **원 판정**: tennis **83/200** (하한 **0.3489** FAIL, 통과선 84) · beanbag 167/200 · leap **83/200** (0.3489 FAIL).
- **host 부하**: tennis 3 unit·beanbag 1 unit 에서 도중에 다른 세션의 테스트가 겹쳐 sim 이 느려졌다 (`rtf_trial_min` < 0.95 시행 28·13). 공 stamp 가 sim 축이라 steady 나이 검사가 stale 로 본다 (`BALL_STALE`); `sim_stall` 은 이것을 못 잡는다. D-S8-17 재실행에서 private 드라이버가 unit 도중 host 의 colcon build/test·pytest·ctest 출현을 감시·중단했다 (repo 러너에는 #601 에서 `--host-watch` 로 이식 — 판정량은 프로세스가 아니라 truth 행에서 읽은 RTF). 부하 시행 회복 (604 1 → 13/20) 이 기전 확인.
- **재실행 반영**: **G8-D 93/200 (하한 0.3972) PASS** · beanbag **175/200** (0.8220) · **G8-D2 83/200 FAIL** (대상 아님). **sim 재현 편차: 같은 투척에서 unit 당 ±5 발** (원인 미확인).
- G7-E: tennis 재실행 CAPTURED 79/79 참, false-Missed 14 · beanbag 일치 142/200 (링크·손바닥 위 공) · leap 197/200. beanbag 대 tennis McNemar 91 대 9 (p 3.3e-18). leap 미종결 9 발은 `TRACK_CHANGED` 반복.
- **S3.1b 부하 검증 충족** (수치는 §5, S3.1a 는 §5.1). t_c 분해 (tennis / leap): 서보 2.6 / 6.7 · pred 79 / 44 mm.
- 도구 (사용법 SSoT 는 rtc_tools README): `catching_trials` `invalid_reason`·ITT·`--floor`, `catching_pool` 합산·McNemar, lane 발사 시각 정렬, `--eval-samples` 는 첫 접촉 전 표본만, `--probe-dump`.
- **후속 배정 (§4.4 pre-S10).** ① G8-B → profile 수정 + D-1 · ② G8-C2 → #600 (닫음 — 식 유지) · ③ G8-H → S9a (9/9) · ④ 분석기 `t_c` 열 ±100 ms (sim 이 느릴 때) → #602 (닫음) · ⑤ leap `TRACK_CHANGED`·판정 없음 9 → 20 → R6 (평가 기하) · ⑥ host 부하 감시 → #601 (R7, 닫음).

**후속 ① G8-B profile 수정 (2026-09-28, PR #595 → `482d18b3`).** 원인은 추정기가 아니라 profile (q 1.0 · drag 없음). `ball_perception_sim_profile.json` → **schema 0.2 · q 0.01 · `process.drag` (`quadratic_still_air`, k 0.02 ± 0.01 1/m)** (지평 1.0 s / 0.05 s / 20 점 불변). 재평가 NEES tennis **1.60 / 1.21 / 1.71** (0.1 / 0.25 / 0.5 s) · leap 0.5 s `NOT_EVALUATED`, coverage_95 0.97–1.00 — 원 판정식 FAIL, **D-1 로 PASS**. 포획 tennis **180/200** (0.851) · beanbag **196/200** (0.950) · leap **86/200** (0.363, PASS 로 올리지 않음). 옛 profile 대조 75/200 (McNemar 111 대 6) — profile 효과. 실제 공의 q·k 는 ADR-0008 소관.

| NEES (0.1 / 0.25 / 0.5 s) | S8-E (옛 profile) | PR #595 뒤 |
|---|---|---|
| p1b tennis | 0.58 / 0.28 / 0.18 (CI [0.57, 0.59] 등; 재실행 뒤 같음) | 1.60 / 1.21 / 1.71 (CI [1.55, 1.65] / [1.16, 1.25] / [1.59, 1.82]) |
| beanbag | 0.59 / 0.28 / 0.17 | 1.57 / 1.19 / 1.67 |
| leap | 0.60 / 0.32 / `NOT_EVALUATED(첫 접촉 전 표본 0)` | 2.13 / 2.41 / `NOT_EVALUATED(첫 접촉 전 표본 0)` |
| coverage_95 · 편향 | 1.00 · ≤ 18 mm | 0.97–1.00 · ≤ 2.6 mm (종전 0.5 s 에서 −19 / +17 mm) |
| capture discontinuity | tennis 921 · beanbag 321 · leap 1034 | 2917 · 3018 · 3987 (접촉·바운스를 더 빨리 거부) |

beanbag·leap 의 arm 별 CI 는 원문에도 없다 — D-1 판정은 세 arm 의 CI 상한 1.25–2.63 · coverage 0.974–0.996 로 했다.

**S8-F 손 근처 투척 (탐색, Epic SPRINT·G8-D·D-18 밖; #537 5843905916 → 5844391177).** 질문: 손 반경 ≤ 20 cm 로 오는 공을 작은 보정으로 더 빠르게 받을 수 있나. **S8-F-1** = 출하 자세 + 상자 확장 (sim 전용, 출하 `catch_box` 는 안전 gate 로 유지) + 도달 우선 점수 A/B; S8-F-2 (K 자세) → S8-I.

**S8-F-1 결과 (2026-09-26, #537 5845711680, PR #583 → `3278963a`).** 1068 발, 무효 0, `/code-review` 9 건 반영 뒤 결과 불변. 정면 절벽 3.5 m/s → 3/8, **v50(r 0) 3.63 m/s** [3.00, 3.97], r 의존 약함 (pred 79 mm 가 r_cap 24 mm 를 지배). beanbag 절벽 38/56 대 tennis 4/56 → 절반 이상은 반발. **A/B 17 vs 21 (p 0.58) — 효과 없음** (plan 93–100 % 가 순위 gate 에 걸려 penalty 균일). "`v_max` 3.5 (η_v → 3.15) 가 γ_max 를 정한다" 는 **틀렸다** (S8-H: v_dir,max). 도구 `catching_hand_near`.

**S8-G sim 팔 예산 (후속 ②; #537 5846760878; PR #584 → `769f99e9`).** 가설: 팔 예산이 포획을 막는다. 참조 잔여 $(1+\omega T)e^{-\omega T}$ = **0.174** 로 ω × 가용 시간이 지배하고 예측 몫이 팔 몫보다 크다. 계획기 box **2.03** rad/s² 는 실행을 안 묶으면서 순위 gate 만 떨군다 (사전 분석·envelope 값은 §9). 스윕: **ω10·a30 abort 0 · 포화 1.1 % · 포획 10/56** (a21 11/56, abort 7) — ω 15·20 은 예측 잡음을 따라가 2–5/56. 사전 등록 채택 규칙 (e 최소 ∧ …) 은 ω20·a30 (3/56) 을 골라 판정량이 틀렸다. envbox 는 `rank_reach` 실패를 줄였으나 (§9) 포획 무영향. 리뷰 9 건 반영. 같은 seed 재현 11 대 4 (p 0.039) → R4. **S8-B~S8-F-1 수치는 a_max 21 · box 2.03 시절.** `sim.yaml` override 는 box 를 안 핀한 모든 sim overlay 에 미친다.

**S8-H → 후속 ① 종결 (2026-09-27, #537 5847660329; D-S8-19).** 가설: γ 창을 실측 추종 능력으로 재정의하면 포획이 는다. 391 committed 시행: 창 **96 % 비어 있음** (g_min 0.81 > g_max 0.39), g_max 를 묶는 항 v_dir,max **99 %** (p50 2.07 m/s) · η_v·v_max **0 %**, 비면 `ChooseGamma` 가 γ_f = g_max. uncertainty·error_budget·rollout 100 % 실패는 참 판정. 지렛대: v_dir,max ≤ 1.5 → 성공 0 %, 2.5–3.0 → 20 % (④); 손 v_rel ≥ 2.5 → ≤ 8 % (③).

**S8-I ④ wait_pose (2026-09-27, #537 5850509543 · 5852292540 · 5853132529; PR #586 → `a471d8b8`; D-S8-20).** 가설: v_dir,max 가 큰 대기 자세가 포획을 올린다. 기능 `wait_pose_source: current` (정지 tick 의 q_meas 채택, 상자 밖·E-STOP 아래 거부; sim switch 검증 오차 ≤ 1.2e-4 rad · 무장 시 이동 ≤ 1e-4 rad). S8-I 점 DLS 최적 P1/P2 는 특이점 근방 바늘 봉우리라 포구 자세에서 무너져 (런타임 v_dir,max 2.1 → 1.7/1.5) 15·5 대 P0 18/112 (P2 p 0.007) — P2 악화는 S8-H 인과를 지지, 틀린 것은 목적함수. `/code-review` 10 건 반영. S8-I-2 robust 목적 P1': v_dir,max p50 **2.62/2.66** (P0 2.11/2.21) 로 기전 실현, 포획 **20 대 18/112 (p 0.86)** → 포획률은 손 (③) 이 지배, ④ 종결. 실험 overlay 17 개 제거 (재현은 git 이력 + `sim_overlay:=<경로>`), `ur5e_p1a`·`iiwa7_leap` 프로파일 유지.

- 연속 재계획 측정 (S0.7 R1 결정의 확인): 발사 → 첫 plan 시각, APPROACH 중 교체, 첫 plan 전 탈락 사유, R2 최악 경우 근접 비율 — S8-B·D·E 가 기록 (교체 0, 첫 plan 0.20 s).

**게이트 (최종; L8 §9.1 은 축약 사본).**

| 게이트 | PASS 기준 | 최종 판정 |
|---|---|---|
| G8-A | RT 위반 0 — 판정은 제어 PC | `NOT_EVALUATED(제어 PC)` → S10 (#613). dev PC smoke 주기 초과 tick 시행 tennis 27/200 (재실행 2/200) · beanbag 16/200 · leap 3/200 |
| G8-A2 | 연속·abort 직후 재투척 재무장 | PASS (S8-B 600/600 · 5/5) |
| G8-B | 예측 NEES (D-S8-16 ②c) | **PASS** (2026-09-29, PR #595 + D-1 — 기준 문구는 §4.4 pre-S10 "리뷰 후속"). S8-E 원 판정식 FAIL (NEES 0.17–0.60, 과대·보수적) |
| G8-B2 | 레이아웃 해시 | PASS (S5.2) |
| G8-C | γ 소거 | PASS (S8-B, 47 대 2) |
| G8-C2 (= G3-H) | L3 §4.6 직교 분해 (A⊥B) | **FAIL** — 정확 결합 591/591, A·B 음의 상관 (백색화 교차공분산 대각 ≈ −0.64), E\|A\|²+E\|B\|² 가 E\|A+B\|² 의 ≈ 2.1–2.7 배. profile 수정 뒤 재측정은 1.1–1.7 배로 부호가 남는다 — 판정은 FAIL 그대로, 예산식은 유지 (L3 §4.6 "측정과 결정", #600 닫음) |
| G8-C3 (= G3-E) | 삭제 (D-8) | 기록만: `ref_saturated` 연속 > 0 시행 tennis 165/200 (원 158/200) · beanbag 161/200 · leap 5/200 |
| G8-D (p1b) | truth 성공 Wilson 97.5 % 단측 하한 ≥ floor 0.35 (D-12), n_valid 200 | **PASS 180/200 · 0.8506** (D-2, 2026-09-29). 이전: S8-E 83/200 · 0.3489 FAIL → D-S8-17 재실행 93/200 · 0.3972 PASS |
| G8-D2 (leap) | 같음 (조건부 §1a) | 세 판정 병기 (아래) |
| G8-E | 계획 γ 에서 lead 반영 서보 잔여 ‖FK(q_meas(t_c)) − FK(q_cmd(t_c − T_lead))‖ 중앙값 감소 (쌍 Wilcoxon) | PASS (sim, S8-B) — 50 ms 플랜트 위 검증, UR5e 이득은 예측 안 함 |
| G8-H | early-return 분기 body 테스트 | **PASS 9/9 (S9a)**; S8-E 는 FAIL 2/9 — E-STOP·stale 외 7 분기 테스트 공백, Compute 는 매 tick 발행해 결함 아님 |
| G7-B3 | Δp 대 ∫F dt 상관 | 기록 · 토크 `NOT_EVALUATED(sim clamp)`. tennis 원 n 199 기울기 1.70 [1.28, 2.00] · 절편 −0.029 N·s · Spearman 0.53 (p 9e-16) → 재실행 1.60 [1.11, 1.81] · 0.27 (p 1e-4); beanbag 원 0.75 · ρ −0.18 → 재실행 0.10 · −0.40; leap −0.09 · 0.00 |
| G7-E · G3-D | 일치율 · plan 유효율 | 기록 · PASS (교체 0/600) |
| clock (D-3) | 공변량 (D-S8-4) | S3.1b 충족 (S8-E) |
| GUI·plot | §13 S8 행 | — |

- **G8-D2 — 원 판정은 지우지 않고 나란히 둔다.** ① S8-E **83/200 · 0.3489 FAIL** (D-S8-17 은 leap 에 해당 없음) ② pre-S10 R6 보정 재판정 (다른 기전: 평가 기하) — 대기 손과 간격 ≥ 20 mm 인 90 발에서 **41/90 (0.357) PASS**, profile 뒤 **45/90 (0.399) PASS**; 사후 기준·n < 200 (§4.4 pre-S10 R6 결과) ③ profile 뒤 상자 전체 **86/200 (0.363) — PASS 가 아니다** (재실행 분산 안).
- leap 지도-열림 부분집합 83/190 (하한 0.3683) 은 공변량. beanbag: **196/200 · 0.9497** (D-2; S8-E 175/200, 원 167/200). 통과선·검정력은 §1a 기준 1 ③.
- **G8-E 판정량 교체 (2026-09-24, E-9, #537 5808509678 · 5808613906).** 종전 ‖FK(q_meas(t_c)) − FK(q_cmd(t_c))‖ 는 lead on 에서 의도된 선행 (`MakeNowLead`) 을 잔여에 더해 거짓 FAIL (S8-B 93 vs 94 mm) — `cmd_meas_gap_mm` 로 기록만, `ref_vs_true` 도 ref(t_c − T_lead) 로 읽는다.

#### S9 E-STOP·fault 정책 (D-13)

- D-13 결정 → 구현 → `/security-review` (E-8)
- 결정 시 검토할 부작용: E-STOP 중 손도 측정 자세 유지 → position servo 간극이 0 이 되어 파지력 소실 가능 (→ D-S9-A)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 정책 | D-13 정책의 E-STOP 발동·해제 시나리오 테스트 전부 통과, S5 E-8 race 테스트 재통과 | D-13 |
| 리뷰 | `/security-review` 에서 Critical 0 (지적 사항은 해소하거나 사용자 수용 기록) | — |
| GUI·plot | §13 S9 행 | — |

**S10 착수 전 필수 — 완료 (2026-09-27).** 정책 PASS (시나리오 테스트 전체 실패 0, 기존 assertion 무수정) · 리뷰 PASS (S9a `/code-review`, S9b `/security-review` Critical 0) · §13 S9 PASS.

**D-13 확정 (2026-09-27 사용자 — #537 코드 대조 5854178226 → 확정 5854557211 → 검토 5854881940 → 개정 5854950396 → D-S9-K 5854969250 → S9b 착수 전 결정 5855489704).** 미결정 항목은 없다. 착수 전 코드 대조의 결론: E-STOP latch 중 CM 이 컨트롤러 출력 전체를 버리고 모든 device 를 측정 위치로 치환하므로 "파지력 소실" 은 구조적으로 확정이고 해제·리셋 뒤에도 간극 0 이 유지된다 (#504 와 같은 기전). 나머지 공백 (L7 §4.1 "재차 치명 조건" 경로 부재, 성공 tick 에 0 이 되던 streak, 측정 팔을 보지 않는 `ABORT_SAFE`·기한 없는 `RETREAT`, GUI 해제 부재) 은 아래 결정이 다룬다. 소프트웨어 E-STOP 을 거는 서비스는 없다 (해제만 — 새 srv 도 두지 않았다, D-S9-E2). sim position 모드 그룹은 중력 보상이 따로 켜져 hold 중 처짐은 실기 (S10) 몫이다.

| ID | 결정 | 상태 |
|---|---|---|
| D-S9-A | E-STOP 중 손은 **현행 유지** — CM 이 측정 자세로 hold, `HOLD` 중이면 공을 놓는다. 테스트로 고정. device 별 hold 정책 (CM 변경) 은 범위 밖 (필요하면 별도 issue). pre-S10 R2 (#588) 에서 "측정 자세" = 정지 tick 의 측정으로 latch (creep 제거, 공을 놓는 동작은 같다) | 확정 |
| D-S9-B | 단계별 반응은 **일률** — 즉시 hold → `IDLE` (`FAULT` 는 유지), 진행 중 시행은 `Aborted`, 판정된 시행은 보존. 감속이 필요한 조건은 컨트롤러 소유 (`ABORT_SAFE`·`FAULT`) | 확정 |
| D-S9-C | 해제 뒤 복귀는 **현행이 최종** — 비무장 `IDLE`·q_c 재시드 → 운용자 재무장 → homing, 채택된 `wait_pose` 유지. 운용 절차 (해제 → fault 있으면 reset → 공 제거 → 무장) 는 문서로 둔다 | 확정 |
| D-S9-D1 | **운동 기한 → `FAULT`** — `ABORT_SAFE` 정지 램프 · `RETREAT` 정지 단계 · `RETREAT` 복귀 단계. 사유는 기존 `ABORT_ESCALATED` (새 필드 없음, 전이표 94 → 95 행). `RETREAT` 중 latch 면 복귀를 멈추고, `ABORT_SAFE` 는 abort 시점 `track_err_` 를 읽지 않는다 (오발화). 키는 **정지·복귀 둘** (Q1 = b — 하나면 정지 기한이 복귀에 맞춰 느슨해진다). 기본값 = S8 sim 관측 최대 × 2, 하한 0.5 s → ur5e_p1b 4.28 / 14.76 s, iiwa7_leap 4.28 / 6.09 s (S9b 결과). **provisional** — 실기 구성은 S10 에서 값을 정할 때까지 막힌다 | 확정 |
| D-S9-D2 | `n_qp` 는 **시행 단위** — CLIK 실패로 끝난 시행마다 +1, `HOLD` 판정 도달 시행 (Captured·Missed·Undetermined) 이 0. CLIK 외 abort·**E-STOP 은 불변**, 0 은 fault reset·activation 만 (재무장 리셋 아님 — C-29 probe 가 재무장 뒤 유지를 단언) | 확정 |
| D-S9-D3 | `ResetFault` 는 **팔이 정지하지 않았으면 거부** — 명령 정지 ∧ 측정 \|q̇\| ≤ homing 도착 허용치 (새 키 없음), 한 tick 판정. 속도 lane 판독 불가도 거부 (fail-closed, Q2 = a — 다시 부를 수 있어 비용이 낮다; S8-I 대기 자세·ARMED 판정의 같은 공백은 pre-S10 R3 Q9). latch 유지, 사유는 diag 열 + WARN | 확정 |
| D-S9-E1 | `FAULT` 를 global E-STOP 으로 승격하지 않는다 (두 latch 는 별개, P-1 (d)) | 확정 |
| D-S9-E2 | sim E-STOP 주입은 **repo 밖 overlay + relay rig** (`enable_estop: true` · 짧은 `device_timeout_values` · 대상 그룹을 `sim_sync_tick_devices` 에서 제외 · 상태 토픽을 relay 로 우회 뒤 종료). 새 srv 없음. 측정은 정지 거리 하나 + hold drift 1 회 | 확정 |
| D-S9-F | G8-H 테스트 공백 7 분기를 S9a 에 포함 (`HORIZON_EXTRAP` 은 새 시나리오 필요) | 확정 |
| D-S9-G | **S10 (#613) 이월**: 드라이브 hold 반응의 실기 값 · 실기 E-stop/보호정지와 소프트웨어 latch 의 관계 (repo 에 신호 출처 없음) · 해제 뒤 드라이버 재개 절차 · 운동 기한의 실기 값 | 확정 |
| D-S9-H | GUI 해제는 **2 단계** (빈 확인값으로 사유 조회 → 확인 뒤 해제; 사유가 토픽에 없다), 포구 패널이 아닌 **공용 위치**. `reset_fault` 버튼, `catching_diag` 플롯에 E-STOP·fault 음영. `CatchingState` 동결 유지 | 확정 |
| D-S9-I | abort 순환 (`RETREAT` ↔ `ABORT_SAFE`) 카운터는 넣지 않는다 — 재현이 먼저, 기록만 | 확정 |
| D-S9-J | `rtc_msgs` `qp_fail_streak` 필드 **주석만** (wire 불변), PROC-3 전체 빌드·테스트 | 확정 |
| D-S9-K | **시행 중 팔을 읽을 수 없게 되면 운동을 멈추고 기한을 센다** (stale 과 달리 CM watchdog 이 잡지 않는다) — `RETREAT` 는 복귀를 시작·진행하지 않고, 그 시간이 D-S9-D1 기한에 포함돼 넘으면 `FAULT`, 다시 읽히면 이어 간다. 기각: 기한 정지 (보이지 않는 팔에 운동 명령이 끝나지 않는다) · 세기만 (기한까지 보이지 않는 팔을 움직인다) | 확정 (#537 5854969250) |
| D-S9-L | **E-STOP 중 `reset_fault` 는 현행 유지** (검토 M4) — latch 는 그 tick 에 내려가고 모드는 해제까지 `FAULT`, 해제 tick 에 `FAULT_RESET` → `IDLE` (비무장). L7 §4.1 에 뜻을 적는다. 기각: reset 거부 (CM 이 "원인이 남아 있다" 로 오보고) · 즉시 `IDLE` (R-PREC 위반) | 확정 (2026-09-27 사용자, Q3 = a) |

Q1–Q3 은 #537 5855489704 (2026-09-27 사용자) 이다.

**S9 는 둘로 나눈다 (2026-09-27 사용자).** S9a 는 RT 코드를 바꾸지 않고 현행 정책을 고정하고, S9b 는 그 테스트 위에서 `FAULT` 를 확장한다.

| | S9a — 현행 정책 고정 | S9b — FAULT 확장 |
|---|---|---|
| RT 코드 | 불변 (컨트롤러 RT 소스 diff 0) | D-S9-D1·D2·D3·K + latch 된 채 `IDLE`·E-STOP 중 reset 의 정리 |
| 내용 | 단계별 E-STOP 테스트 · 포구 컨트롤러 × 실제 CM 노드 테스트 · G8-H · GUI · 플롯 · sim · 문서 (D-S9-A·B·C·E1) | 전이표·리셋 표·새 YAML 키·거부 사유 관측·`rtc_msgs` 주석 |
| 범위 | `integrated_bringup` (테스트·GUI) · `rtc_tools` | `integrated_bringup` 포구 컨트롤러 · `rtc_controllers` · `rtc_msgs` 주석 1 곳 |
| 리뷰 | `/code-review` | `/security-review` Critical 0 + `/code-review` |
| 상태 | **머지** (PR #589 → `f4bc14ed`) | **머지** (PR #590 → `18507711`) |

**`[SPRINT] S9a` (#537 5854950396) · `[SPRINT] S9b` (+ 5854969250, 5855489704) 는 모두 충족했다** (G7-A 전이표 완전성·G7-H race·RT 할당 0 재통과 포함). 공통 제약: `rtc_controller_manager`·`rtc_base` 불변, 포구 컨트롤러 × CM 테스트는 public 헤더만 쓰는 shim (ARCH-4). actuator 에 나간 명령은 CM 의 기존 치환 테스트와 sim ③ 이 본다.

**S9a 결과 (2026-09-27, PR #589 → `f4bc14ed`, RT 소스 diff 0).**
- **단계별 E-STOP** (`EstopInStageTest.*` 5 건): 다섯 단계 모두 정지 tick 에 `IDLE`·`ESTOP`·비무장, 재개 없음, 팔·손 출력 = 정지 tick 의 **측정** 자세 (`HOLD`·`RETREAT` 는 닫힌 자세). `outcome` 은 진행 중 시행이면 정지 동안 `Aborted`, 해제의 리셋이 `None` 으로; 판정된 시행 (`RETREAT`) 은 정지 tick 에 `None` — 판정은 진입 tick 에 이미 발행돼 있다 (L7 §4.1 에 기록).
- **G8-H 9/9** (`TickBodyTest.*` 7 건 + 기존 2; `HAND_TIMEOUT` 은 `RETREAT` 해제 기한으로만 도달). **포구 컨트롤러 × 실제 CM** (`test_catching_cm_services` 5 건): `clear_estop` 2 단계 해제 (fault 는 남기고 그렇다고 답함), `reset_fault` 는 `Name()` 만 받고 `IDLE`·비무장, E-STOP 중 `reset_fault` 는 fault 만. GUI·플롯은 D-S9-H 대로 ("latch 는 내려갔으나 미검증" 응답은 거부와 구별).
- **센서**: 테스트 실패 0, 시나리오 바이너리 41.2 s (≤ 120 s), mutation 3 종 검출 (손 latch 를 유지하는 리셋은 `COMMITTED` 에서 구별 불가). 편집 hook 의 F401 자동수정 제외 (`7373bd4d`) 를 함께 넣었다.
- **S9b 로 넘긴 것** (①②③ 반영됨): ① `catching.enable` 파라미터 설명이 "E-STOP 에 스스로 내린다" 로 오도 ② `ServiceResetRequests` 의 낡은 주석 (fault 경로는 S5.3 이후 있다) ③ E-STOP 중 reset 뒤 모드 (→ D-S9-L) ④ `FAULT_RESET`·`ABORT_ESCALATED` 는 한 tick 만 보인다 (기록만). **범위 밖 후속 (미배정)**: GUI 가 사유를 CM 거부 문장에서 잘라 읽는다 — 구조화된 필드는 `rtc_msgs` 변경 (E-3) 이라 별도 issue 거리.
- **sim ③ (repo 밖 rig — 끊는 그룹 `device_timeout` 50 ms, 다른 그룹 1000 ms; 팔·손 각 1 기동, `CLOSING` 중 끊음)**: E-STOP 발동 (`ur5e_timeout` · `p1b_timeout`, 끊김 → latch 71 ms · 65 ms), CSV·플롯 구간 확인, GUI 로 해제 (원인이 남으면 "cleared, then re-latched" 로 거부), 해제 뒤 비무장 `IDLE`·재개 없음. **정지 거리 (기록)**: 손 끊김 기동 (팔은 실제 제동) ‖q̇‖ 1.76 rad/s → 0.277 s, 최대 0.030 rad (shoulder_lift). 팔 끊김 기동은 **끊긴 순간의 자세로 되돌아간다** (CM 의 "측정" 이 마지막 stale 샘플 — ‖q̇‖ 1.40 → 0.288 s, shoulder_lift +0.094 rad 역행, `q_at_cut` 과 1.5e-3 rad 이내). **hold drift (10 s)**: 끊긴 device ≤ 5.4e-4 rad, 살아 있는 device thumb_mcp 0.032 · wrist_2 0.012 rad. 공을 쥔 채의 E-STOP 은 `HOLD` 에 이르지 않아 관측하지 못했다.
- **CM 발견 → #588** (사용자 승인으로 분리; ①②③ 은 pre-S10 R2 에서 닫음): ① 살아 있는 device 가 E-STOP 동안 밀린다 (thumb_mcp 약 3.4 mrad/s, 137 s 에 0.46 rad) — hold 를 매 tick 측정으로 다시 만들었다 ② volatile `/system/estop_status` 라 latch 뒤에 뜬 GUI 는 "NORMAL" ③ 거부된 해제도 latch 를 2 tick 내린다 ④ python relay 오발화 1 회 (rig 취약성) ⑤ latch 전 timeout 구간 (실기 `robot.yaml` 1000 ms) 동안 CM 은 stale 상태로 돈다 — 실기 몫 (S10, D-S9-G).

**S9b 결과 (2026-09-27, PR #590 → `18507711`).** `/security-review` Critical 0 (High·Medium 0) · `/code-review` 6 건 중 4 건 반영, 2 건은 아래 판단 사항.
- **구현**: `MotionDeadlineOverrun` (escalation 계층, 준비 상실보다 앞) · `LatchFault` (**latch tick 에 비무장**) · `NoteLawVerdict` 의 시행 단위 `n_qp` · `ArmNotAtRestForReset` (\|q̇\| ≤ `homing.qd_tol`) · 리셋이 `RETREAT` 단계를 되돌리면 기한 시계도 재시작. 전이표 95 행 (`RETREAT × ABORT_ESCALATED → FAULT`), 키 `supervisor.deadline.{stop_s, return_s, provisional}` (TBD·비양수 거부, provisional 은 실기 park).
- **관측**: CSV `fault_cause` (latch 동안: 1 `n_qp` · 2 정지 기한 · 3 복귀 기한) · `fault_reset_refused` (거부 tick: 1 명령 램프 · 2 측정 속도 · 3 lane 판독 불가), WARN, `rtc_tools` 요약 줄, GUI "QP fail streak N trial(s)". 상태 메시지 필드는 그대로다.
- **기본값 산정 (D-S9-D1)**: S8 sim `catching_diag` 36 세션 (RETREAT 2474 회, `ABORT_SAFE` 121 회). 단계 경계는 로그에 없어 추정 — 정지 끝은 max\|Δq_cmd\| 가 비증가에서 증가로 바뀌는 tick, 복귀 끝은 손 `Release` 위상의 첫 tick. 최대: ur5e_p1b `ABORT_SAFE` 1.546 s · 정지 2.138 s (S8-F `lhs_ss_912` — 팔이 `track_err` 0.47 에서 0.42 아래로 따라오는 데 2.1 s) · 복귀 7.380 s (S8-F `lhs_rf_912` — wrist_3 3.4 rad 복귀), iiwa7_leap 정지 0.066 s · 복귀 3.044 s (abort 없음). envelope `qdd_max` 세션은 최대를 정하지 않았다. → ur5e_p1b **4.28 / 14.76 s**, iiwa7_leap 규칙값 0.50 (하한) / 6.09 s — **`stop_s` 는 ur5e_p1b 와 같은 4.28** (아래 ②), 파서 기본은 큰 쪽 (4.28 / 14.76). 모두 provisional.
- **테스트**: `FaultExtensionTest.*` 13 건 외, **기존 assertion 무수정** (E-6 후보 `QpFailuresAbortAndTheThirdLatchesAFaultThatResetClears` 도 통과), fixture YAML 셋은 `deadline.provisional: false`. mutation 8/8, 21 패키지 build·test 실패 0 (PROC-3). sim 은 SPRINT 항목이 아니라 돌리지 않았다.
- **남긴 판단 → 결정 (2026-09-27 사용자, #537)**: ① 기한 **전** disarm 이면 `RETREAT → IDLE` 이고 `IDLE` 램프 (R-IDLE) 에는 기한이 없다 — **현행 유지** (새 운동을 명령하지 않고, 바꾸면 전이표 변경 (E-8)) ② iiwa7_leap `stop_s` 0.50 은 하한일 뿐 (코퍼스에 abort 가 없다) — **4.28 로 설정**. leap 측정은 새 투척 설계가 필요하고 값이 바뀌는 것은 최악 정지가 2.14 s 를 넘을 때뿐이라 **S10 (실기 값) 또는 별도 실험으로 남긴다.**

#### pre-S10 — S10 착수 전 잔여 (2026-09-28)

범위는 **S10 (실기) 착수를 제외하고** sim·코드·문서로 닫을 수 있는 것만이다 (사용자 2026-09-28, #537 5867427524). 실기 몫은 아래 S10 "pre-S10 에서 이월". 결정 Q1–Q11 은 #537 5867427524 · #588 5867430789, 검토가 더한 Q12–Q16 은 #537 5867745984. R 하나 = PR 하나로 순차 진행했다. **전부 완료 (2026-09-29).**

| R | 내용 | 상태 |
|---|---|---|
| R1 | 문서 정리 — S9 머지 기록, 이 절, S10 이월 목록, §12 위험 행, 참조 구현 삭제 (Q8) 와 그것을 가리키던 코드 주석 | 완료 (PR #596 → `70057aef`) |
| R2 | CM global E-STOP (#588, E-8) — hold 목표 latch (Q1), 해제 검증 창 동안 hold 유지 (Q2·Q12·Q13), `estop_status` transient_local (Q3·Q14) | 완료 (PR #597 → `1307576f`, #588 닫힘) |
| R3 | 포구 안전 게이트 정합 (E-8) — `derived_accel_limits` provisional 실기 park (Q4), `joint_cmd.lag.provisional` 신설 (Q5), `planner.hand.provisional` 삭제 (Q6), 대기 자세·ARMED·손 정착 판정의 속도 lane 판독 (Q9·Q16) | 완료 (PR #599 → `6b6dcb96`) |
| R4 | 빈 vision cloud 분류 — `width == 0` 을 "트랙 없음" bucket 으로 (Q7·Q15) | 완료 (PR #603 → `1b64a9f2`, R6·R7 기록과 한 PR) |
| R5 | 마감 — #537 갱신, 원자료 정리 (Q11) | 완료 (2026-09-29) — S8 원자료 22 GB 삭제, 남긴 것은 아래 |
| R6 | leap `TRACK_CHANGED` 조사 (코드 변경 없음, Q10) | 완료 (2026-09-29, #537 5881444338) — 결정: 보정 기준 재판정만 |
| R7 | issue 분리 3 건 — G8-C2 예산식 · 부하 감시 러너 이식 · 분석기 `t_c` 열 (Q10) | 완료 (2026-09-29) — #600 · #601 · #602, 셋 다 같은 날 닫음 (PR #618 · #616 · #617) |

- **R5 가 남긴 private 원자료**: S8-G `w20_a21.fail1` · R6 재판정 근거 (간격 표) · 드라이버 스크립트 사본 · S8-E 와 profile 수정 뒤 G8-B 재평가 unit 의 `eval_report.json` 과 로그. S8-E 의 capture·eval (Q11 예외) 과 G8-B 재평가 unit 의 capture·eval·trials 는 G8-B 가 PASS 로 끝나 보존 사유가 해소돼 2026-09-29 에 삭제했다 (사용자 결정, 662 MB) — 표본 단위 재분석은 재실행이 필요하다. `session_copy`·probe dump 6.6 GB 는 같은 날 먼저 삭제했다.

| # | 결정 (2026-09-28 사용자, 모두 권장안) |
|---|---|
| Q1 | CM hold 목표 = E-STOP latch 시점의 측정으로 device 별 고정 (그때 읽을 수 없는 채널은 마지막 판독값), 해제 시 폐기. torque 모드의 0 N·m 는 불변 |
| Q2 | 해제 검증 창 동안 hold 치환을 유지하고, 통과한 뒤에만 컨트롤러 출력을 낸다 |
| Q3 | `/system/estop_status` 를 transient_local 로 — latch 뒤에 뜬 구독자도 현재 값을 받는다 |
| Q4 | `derived_accel_limits` 의 `provisional: true` 도 실기 park (자기 키 이름을 사유로, 기존 `supervisor` 미설정 사유와 섞지 않는다) |
| Q5 | `joint_cmd.lag.provisional` 신설 — `T_arm` 식별 전 실기 park |
| Q6 | YAML 의 `planner.hand.provisional` 삭제 (파서가 읽지 않는다 — 상위 `planner.provisional` 이 덮는다) |
| Q7 | `width == 0` 을 "트랙 없음" 으로 따로 세고 WARN 하지 않는다. `0 < width < n_min` 은 계속 `kShape` |
| Q8 | docs 참조 구현 전부 삭제, README 파일 표 갱신 |
| Q9 | 대기 자세 채택·ARMED 판정에 속도 lane 판독 여부 — 읽을 수 없으면 채택 거부·준비 안 됨 |
| Q10 | R6 leap 조사 진행. G8-B 는 ball_perception 세션에 인계. G8-C2·부하 감시·`t_c` 열은 이 repo issue (R7) |
| Q11 | S8 원자료는 R6·R7 뒤 삭제. 예외: S8-E 의 capture·eval (G8-B 종료 확인까지) 과 S8-G 의 `w20_a21.fail1` unit |
| Q12 | Q2 구현: 별도 "검증 대기" 플래그 — 서비스가 clear 전에 세우고 **RT 루프가** 검증 tick 수 뒤 내린다. 치환 조건 = latch ∨ 대기, `IsGlobalEstopped()` 의미는 그대로. 기각: latch 해제 자체를 미루기 — "clear 직후 해제됨" 을 단언하는 기존 E-STOP 테스트와 충돌 (E-6) |
| Q13 | 검증 창 동안 발행하는 `estop_status` = latch ∨ 대기 — 거부된 해제가 "해제됨" 토글을 남기지 않는다 |
| Q14 | Q3 로 `shape_estimation` 이 처음으로 E-STOP 을 받는 것을 수용 — 그 구독은 transient_local 이라 volatile 발행자와 QoS 가 맞지 않아 한 번도 연결된 적이 없던 잠재 결함이다. R2 의 E-8 범위·테스트에 포함 |
| Q15 | Q7 은 거부 enum 끝에 bucket 추가 (`CatchingState` 의 거부 histogram 은 이름 동반 동적 배열이라 .msg 불변), GUI 거부 요약에서 제외. 기각: 새 msg 필드 (E-3) |
| Q16 | Q9 범위에 손 정착 판정 (`HandSettledAtPre`) 포함 — 같은 결함 유형 |

**R2 결과 (2026-09-29, PR #597 → `1307576f`; #588 5873297397 · 5873552427).** 구현 세부 결정 (a)–(e) (#588 5872080357): 창 안 재 trigger 도 재캡처 없음 · latch 는 slot 단위로 slot 별 첫 치환 tick 에 캡처 + 창 동안 switch 거부 · 읽힌 적 없는 채널은 cache 값 · Phase 2b (E-STOP 아닌 거부) hold 는 per-tick 유지 · 창 치환은 별도 카운터. Q12 의 "검증 대기 플래그" 는 호출마다 새 `uint32` 토큰 + RT 쪽 CAS (판정과 해제 사이에 새 해제가 끼면 그 창을 지우지 않게). 메커니즘 서술의 SSoT 는 `rtc_controller_manager/README.md` §글로벌 E-STOP 해제·§E-STOP 시 actuator command 차단.
- **SPRINT**: 새 `test_estop_hold_latch` 19 건 (positive control 로 main 에서 red 확인), TSAN 새 race 없음, 전 패키지 build·test 실패 0, 기존 assertion 무수정
- **sim (repo 밖 relay rig, latch+0.5 s ~ +60 s 최대 관절 이동)**: 손 끊고 팔 측정 main 0.085 rad → **1.8e-9 rad**, 팔 끊고 손 측정 2.8e-3 → **4.6e-7 rad** (main 쪽이 "> 0.1 rad" 추정에 못 미친 것은 공 없는 rig 의 손 부하가 거의 없어서다). latch 뒤 기동한 transient_local echo 는 `true` (main 은 durability 불일치로 수신 0)
- **리뷰**: `/security-review` 보고 대상 0. 머지 전 고친 결함 둘 — 창 판정과 Phase 2c 사이에 해제 전체가 끼면 1 tick 이 치환 없이 나가는 틈 (latch 스냅숏을 토큰보다 먼저 읽어 닫음), mid-window deactivate 뒤 `estop_status` 가 true 로 고착
- **결정 (2026-09-29 사용자)**: ① 구독자도 transient_local — GUI (`demo_gui/app.py`) · motion editor · BT bridge (volatile reader 는 transient_local writer 의 이력을 받지 않아 발행자만으로는 늦게 뜬 GUI 가 여전히 NORMAL 이었다) ② shape_estimation 의 E-STOP abort 경로는 `/shape/explore` accept 교착으로 도달 불가 — issue 분리 안 함 (§12)
- **축소한 SPRINT 항목**: ⑤ 의 "shape_estimation 탐색 abort" 는 위 교착 때문에 테스트 불가 → "구독이 CM writer 와 matched, volatile 대조는 unmatched" (`test_estop_subscription`)

**R3 결과 (2026-09-29, PR #599 → `6b6dcb96`; `[CONCERN] E-8` + `[SPRINT]` #537 5880338514).** 메커니즘 서술의 SSoT 는 L5 §6 (`joint_cmd.lag.provisional`·`robot.arm.qdd_max` 행) · L7 §4.5 · L3 §6 (`planner.wait_pose_source`·`planner.hand.*` 행) · `integrated_bringup/README.md` "실기 config 에서의 park" 이다.

- **구현 세부 결정 (사용자 컨펌 2026-09-29)**: box park 사유는 새 enum 없이 `kConsumedValues` 이고 키 이름은 configure 로그가 갖는다 · box 가 **로드된** 경우에만, supervisor 미설정 판정 뒤에 본다 · `accel_limits_path` 는 절대경로 허용 (테스트 fixture 는 출하 파일의 flag 만 바꾼 임시 사본을 읽는다 — 값 복제 없음) · 속도 lane 비판독은 움직이는 팔과 같은 모양 (무장 전 미룸, 무장 뒤 `WaitPoseRefusal::kVelocityUnreadable` 거부) · homing 도착도 같은 판정이라 비판독이면 대기 자세를 계속 명령
- **SPRINT**: 새 테스트 16 건, 기존 assertion 무수정, mutation 7/7, 테스트 실패 0 (positive control 은 main 에 절대경로만 넣은 상태에서 — main 은 절대경로를 못 읽어 fixture 전체가 park 된다)
- **sim 스모크 (overlay 없음, `s35b` seed 1)**: ur5e_p1b 10 발 ARMED 10/10 · 순환 10/10 · abort·FAULT·대기 자세 거부 0, iiwa7_leap configure·activate 성공 (두 로봇 모두 새 두 키의 WARN). 포획 0/10 은 기록만 — 10 발은 포획률 (~16 %) 에 대해 검정력이 없어 판정에서 뺐다
- **리뷰**: `/security-review` 보고 대상 0. `/code-review` 반영 — 재구성이 `SetupArmCommand` 의 이른 return 을 타면 이전 configure 의 box 를 판정하던 것 (진입 시 초기화), sim WARN 을 park 판정 옆으로, **속도 lane 이 닫혀도 아무 표시가 없던 것 (축별 edge WARN, 사용자 2026-09-29)**
- **남긴 것 (의도)**: 무장 뒤에는 한 tick 의 hole 도 채택을 거부한다 (homing 이 그 tick 에 목표를 필요로 한다) · box park 은 YAML 키 park 뒤라 실기 운영자는 두 번에 나눠 알게 된다 · 절대경로의 사본으로 flag 를 풀 수 있다 (운영자 config 는 신뢰 입력)

**R4 결과 (2026-09-29, PR #603 → `1b64a9f2`; `[SPRINT]` #537 5881206029).** 메커니즘 서술의 SSoT 는 L1 §5.1 (형식 검사) 이다.

- **구현 세부 결정 (사용자 컨펌 2026-09-29)**: 분류 자리는 byte order·`frame_id`·`height` 검사 뒤 — `height ≠ 1` 이거나 다른 frame 의 빈 cloud 는 그 결함 그대로 거부한다. bucket 은 거부 enum 의 끝 (`no_track`, histogram 15 칸 — 앞선 index 불변, .msg 불변), 경고 여부는 `IsCloudDefect()` 하나가 정한다. GUI 는 이름으로 걸러 거부 요약에서 뺀다. 가설 확인은 R3 스모크 기록 (35 s 에 `refused (shape)` 16 회) 으로 갈음. `no_track` 조건은 리뷰 후속 #611 에서 좁혔다
- **SPRINT**: 새 테스트 7 건, 기존 assertion 무수정 (GUI 테스트 이름 목록에 `no_track` 추가뿐), mutation 3/3, 실패 0
- **sim 스모크 (ur5e_p1b, overlay 없음, `s35b` seed 1, 10 발)**: `vision message refused` 경고 **0 회** (R3 스모크 16 회) · `no_track` **133** · 수락 765 · 다른 bucket 전부 0 · 순환 10/10. 포획 0/10 은 기록만 (R3 와 같은 이유)
- **리뷰**: `/code-review` 9 건 중 반영 2 — histogram 쓰기의 범위 가드 · 테스트 log sink 의 lock. **남긴 것 (의도)**: "트랙 없음" 을 슈퍼바이저가 소비하지 않는다 — 마지막 수락 예측은 steady 나이로 stale 이 될 때까지 살아 있다 (Q7 은 분류·경고만, stale 경로 불변이 SPRINT) · GUI 에 `no_track` 수를 따로 보이지 않는다 (`ros2 topic echo` 로 읽는다) · 이름 배열이 counts 보다 짧은 version skew 에서는 `#14` 로 보인다 (이름 없는 카운터를 숨기지 않는다는 기존 테스트가 고정)

**R6 결과 (2026-09-29, #537 5881444338; 코드 변경 없음).** leap 의 `TRACK_CHANGED` 는 추정기의 설계 동작이고 원인은 **평가 기하**다 — 수치·발별 표는 코멘트가 갖는다.

- **기전**: S8-E leap 의 판정 없음 9 발 전부에서 공이 발사 +0.10–0.16 s, 상승 중에 대기 손 (`hand.q_pre`) 의 검지 손끝에 닿는다. 추정기는 접촉마다 NIS 연속 거부 → 불연속 → 재초기화로 `generation` 을 올리고 (컨트롤러 `input_generation` 과 정확 일치), 튕겨 나간 공의 바닥 바운스마다 되풀이한다. `lost_timeout` 은 발사 시점의 정상 경로뿐이고 FOV 는 해당 없다 (sim 추정기는 position 모드)
- **S8-D 유효성 검사 (D-S8-15) 가 놓친 이유**: private clearance 검사가 팔 7 축만 대기 자세로 두고 손은 qpos0 로 뒀다. 같은 방법을 기록된 `q_pre` 로 다시 계산하면 200 발 중 간격 < 0 이 47 발 (실제 손끝 접촉 44 발 전부 포함), 기준 20 mm 미달이 110 발 (#537 5881444338 의 107 은 오기). 포획은 손끝 접촉 5/44 대 나머지 78/156
- **profile 수정 뒤의 판정 없음 9 → 20 · ABORTED 0 → 3** 도 전부 같은 44 발 안이다
- **결정 (2026-09-29 사용자): 보정 기준으로 재판정만 한다** — leap 투척 상자·대기 자세는 재선정하지 않고 sim 도 다시 돌리지 않는다. 보정 기준 = 손을 기록된 `q_pre` 로 둔 최소 간격 ≥ 20 mm (D-S8-15 의 기준값 그대로), 통과 90/200. **leap G8-D2 재판정 (sim truth, Wilson 97.5 % 단측 하한, floor 0.35)**: S8-E **41/90** (p̂ 0.456, 하한 **0.357**, PASS — 상자 전체는 83/200 · 0.3489 FAIL) · profile 수정 뒤 **45/90** (0.500, 하한 **0.399**, PASS — 전체 86/200 · 0.363). 기준 미달 110 발은 42/110 · 41/110, 간격 < 0 인 47 발은 7/47 · 8/47. 두 실행의 투척은 동일하다 (같은 seed, 속도 차 0). **한계**: 기준을 결과를 본 뒤에 적용했고 (사전 등록 아님), n 90 은 D-S8-16 의 n_valid 200 에 못 미치며, 간격 계산은 private 검사의 방법을 재현한 것 (발사 +0.25 s 까지) 이다 — 원 판정은 지우지 않고 나란히 둔다
- `TRACK_CHANGED` 로직은 바꿀 것이 없고 ball_perception 인계도 없다. 부수 관찰 (기록만): 계획할 수 없는 트랙이 이어지면 사이클이 TRACKING 에 열린 채 남는다 (러너는 `record_s` 시간초과) · 직전 공의 마지막 스냅숏이 늦게 방출돼 발사 직후 `TRACK_CHANGED` 가 한 번 난다 (세 arm 의 63–71 %, 결과 영향 관찰 안 됨)

**리뷰 후속 (2026-09-29, PR #612; 결정·`[CONCERN] E-8`·`[SPRINT]` #537 5882031244 → 5882065390).** pre-S10 의 PR #596–#605 을 main `5abfbeab` 에서 읽기 전용으로 리뷰해 결함 여섯을 issue 로 나누고 한 브랜치에서 고쳤다. 새 테스트는 모두 수정 전 코드에서 먼저 red 였다 (기존 assertion 무수정).

| issue | 결함 | 수정 |
|---|---|---|
| [#606](https://github.com/hyujun/rtc-framework/issues/606) | 손 속도 lane 비판독 (Q16) 이 `HandSettledAtPre` 에만 적용돼 RETREAT → ARMED 재무장·접촉 baseline 학습 (시퀀서 `at_target`) 은 막지 못했다 | 시퀀서가 위치·속도 판독을 가른다 (ρ·hold offset 은 위치만, 정착은 둘 다). 비판독이면 손 속도를 넘기지 않고, RETREAT 는 release timeout 으로 끝난다 |
| [#607](https://github.com/hyujun/rtc-framework/issues/607) | `on_error` 뒤 새 발행자에 이력이 없어 latch 중에도 늦은 구독자가 NORMAL 로 읽었다 | 발행자를 만들 때 현재 값을 발행 (첫 configure 도 `false` 를 명시) |
| [#608](https://github.com/hyujun/rtc-framework/issues/608) | 대기 중 lifecycle 전이가 창을 비우면 해제 서비스가 검증 없이 `ok=true` | RT loop 가 끝까지 돈 창의 토큰을 기록하고 (`estop_verified_token_`), 서비스는 자기 토큰일 때만 `ok` |
| [#609](https://github.com/hyujun/rtc-framework/issues/609) | `robot.arm.accel_limits_*`·`catch_frame` 이 재configure 에 남아 이전 box 로 activate | configure 마다 초기화. read_only 미러 파라미터는 rclcpp 제약으로 첫 configure 값에 머문다 (설명에 명시) |
| [#610](https://github.com/hyujun/rtc-framework/issues/610) | 속도 lane WARN 뒤 위치 gate 가 닫히면 거짓 "readable again" 과 중복 WARN | tick 이 "판정 가능했는가" 를 함께 싣고, edge 기억은 activation 마다 새로 시작 |
| [#611](https://github.com/hyujun/rtc-framework/issues/611) | `width 0` 이면 payload 가 있어도 `no_track` (경고·GUI 표시 없음) | `row_step` 0 ∧ `data` 빈 경우만 `no_track`, 아니면 `size` (추정기의 빈 cloud 가 그 형태) |

- **결정 D-1 (G8-B 판정식, 사용자 2026-09-29 — 이 리뷰 후속의 번호이며 §1 결정 로그의 D-1 과 무관)**: "시행 부트스트랩 95 % CI 가 3 을 포함" → **"CI 상한 ≤ 3 ∧ coverage_95 ≥ 0.95"**. 이유: profile 의 `sigma_per_m` 0.01 은 실제 공의 항력 불확실성용이고 sim 공의 실제 오차는 0.0005 (k 0.0205 대 0.02) 라, sim 에서 NEES 3 을 맞추려면 sim 에만 맞는 값을 써야 한다. 막아야 할 방향은 과소 추정이다. 과대 쪽은 NEES 하한이 아니라 commit 시점 예측 오차로 본다 — p1b tennis 같은 코드에서 옛 profile p50 73.6 mm (hold 반경 67 mm 초과, 포획 75/200) 대 새 profile 21.7 mm (180/200). 임계는 두 점뿐이라 정하지 않았다 (기록만)
- **결정 D-2 (G8-D 공식 판정, 사용자 2026-09-29 — 마찬가지로 §1 의 D-2 와 무관)**: 출하 profile 이 바뀌었으므로 PR #595 뒤 재평가 값으로 갱신 (S8 게이트 표의 성공률 행, L8 §9.1 G8-D). leap 은 R6 결정 그대로
- **정정**: #537 5880281668 은 leap 판정 없음 증가를 "옛 profile 의 틀린 예측" 으로 설명했으나 원인은 R6 의 손끝 접촉이다. 그 코멘트의 p1b tennis 예측 오차에 같은 접촉 오염이 있는지는 미확인

**R7 결과 (2026-09-29).** issue 셋을 만들었다 — 원자료 삭제 (Q11) 뒤에도 읽히도록 근거 수치를 본문에 실었고, 초안을 코드와 대조하며 계획의 서술 둘을 바로잡았다.

- [#600](https://github.com/hyujun/rtc-framework/issues/600) L3 §4.6 예산식 재검토 — 구현은 문서식과 일치하고 (재검토 대상은 직교 가정), 검사는 순위 게이트다. S8-E 측정이 vision profile 수정 전이라 재측정이 첫 단계. **닫음**: 재측정에서 음의 상관이 남았고 (비 1.1–1.7), 식은 유지 — 런타임 검사는 `σ_ℓ = σ_c` 대입이라 직교 가정을 쓰지 않는다 (L3 §4.6)
- [#601](https://github.com/hyujun/rtc-framework/issues/601) `catching_sim_trials` host 부하 감시 — private 드라이버가 본 것은 loadavg 가 아니라 **빌드·테스트 프로세스의 존재**였다. **닫음**: 러너는 증상 (RTF) 을 판정하고 프로세스는 원인 후보로만 기록한다
- [#602](https://github.com/hyujun/rtc-framework/issues/602) 분석기 `t_c` 열 — `plan_t_c_s` 의 기록 (잔여 시간) 과 분석기의 변환은 옳다. 결함은 그 `t_c` 가 **steady 축**이라 RTF < 1 에서 어긋나는데 분해 열·δ(t_c) 에 표시가 없는 것. **닫음** ((a) 안): `tc_axis` 열과 중앙값 제외

#### S10 실기 단계 도입 (HW-P1B)

**상태: 대기 (pre-S10 뒤 — pre-S10 은 2026-09-29 완료).** 남은 작업은 issue [#613](https://github.com/hyujun/rtc-framework/issues/613) 이 추적한다 — 게이트 정의와 결정은 이 절이 소유하고 (충돌 시 이 절이 우선), 체크리스트와 진행 기록은 issue 가 갖는다.

- bag replay(재스탬프 도구) → 가상 공 → 저속 실투척 → 상향
- 실기 T_close,tot 종단 간 측정 (S4.3 에서 이월, 2026-09-20 — L6 §7 L6.5, G6-D). S4.0 의 sim 전용 가드는 S5.1 에서 E-8 승인과 함께 이미 없어진 상태여야 한다
- T_arm 식별·선행, speed scaling·PTP 감시(신호 출처 확보 후), D-2 예외의 ③ 조건(PTP 동기) 확인

**pre-S10 에서 이월 (2026-09-28 — 착수 전 확인 사항, 실기 없이는 닫히지 않는다).**

- 실기 단계: D (G8-A · G5-F · G6-D · G6-E · D-S9-G · 운동 기한 실기 값) → E (bag replay·가상 공 — G8-F · G7-F) → F (실투척 — G8-G)
- 실기 도구 현황 (2026-09-28 코드 대조): launch·`hand_close` 분석은 있다, `servo_lag_ls` 는 일부, bag 재스탬프 replay · 가상 공 발행기 · speed scaling·PTP 감시는 없다. 게이트 도구는 G6-D 만 준비됐고 G5-F 일부, G6-E · G7-F · G8-F · G8-G 는 절차부터 없다
- 실기 park 키: provisional 이면 실기 구성을 막는 키 9 개 (pre-S10 R3 에서 `derived_accel_limits.<group>.provisional` — 지금은 `robot.arm.qdd_provisional` — ·`joint_cmd.lag.provisional` 추가). 각 키의 실기 값을 정해야 무장할 수 있다
- #588 hold creep 의 실기 크기 (R2 가 sim 에서 닫았다 — 60 s drift 0.085 → 1.8e-9 rad; 실기 드라이브에서의 값은 여기서 잰다)
- 결정: 실기 성공 기준 · G8-G 대체 지표 · D-12 손 토크 권위 출처 · 공 사양 · speed scaling·PTP (v1 은 운용 절차로 두는 것을 권장)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 실기 | G5-F, G6-D, G6-E, G7-F, G8-F, G8-G (G8-G 의 "원인 분해" 는 항목별 수치 기록으로 판정) | 실기 |
| GUI·plot | §13 S10 행 | — |

## 5. D-3 검증 계획 (채택 조건부)

D-3 은 **검증 결과로 다시 검토한다.** 기존 RTF 신호 (200 step 구간 평균) 는 0.5 s 비행에 표본이 1~2 개라 짧은 스톨을 지우고, 스톨 뒤 따라잡는 구간도 가린다 (L8 §4.5). 그래서 비율이 아니라 **시행별 clock 위상 오차**로 판정한다.

**측정 (S3.3 의 per-step lane).** 발사 시각을 원점으로 비행 구간의 매 step 에서 δ(t) = (steady(t) − steady₀) − (sim(t) − sim₀). 시행별로 δ_max = max|δ| 와 max pause (한 step 의 Δwall − Δsim 최대값) 를 기록한다. 뒤처짐·따라잡음 양방향을 모두 본다.

**시행 유효 조건.** v_max·δ_max + ½·a_bound·δ_max² ≤ ε_clk,alloc 이고 max pause ≤ ε_clk,alloc / v_max.

- v_max: 목표 투척 분포의 최대 공 속력 (S3.5b 전에는 S0.7 가정값)
- a_bound: g + 항력 가속 상한 (항력 k — S3.8 이 2026-09-20 결정으로 빠졌으므로 L0 §4.1 의 문서 대표값 0.0229 1/m)
- ε_clk,alloc: L3 §4.6 오차 예산 중 시계 항 ‖v‖δ 에 할당한 몫. r_cap 확정 (S4.5, 2026-09-20) 으로 판정이 열렸고 (§5.1 재판정), 고정 예산이 아니라 **목표 속력에 비례**한다

### 5.1 S3.1a 실측 (2026-09-20)

로봇 2종 × 발사 200 회, **풀 bring-up** (포구 스택 없음 = §5 의 "무부하"; 컨트롤러 없는 독립 노드 구동은 sim 이 `sync_timeout_ms` 를 기다려 ~20 Hz 로 도는 다른 실험). `/sim/launch_ball_at` 지정 발사로 모든 시행이 같은 방출 상태 `p0 = (4.81, 0.04, 1.75) m`, `v0 = (−4.0, 0, 3.5805) m/s`, `ω = 0` 이고 재현된다. 도구: `analyze_clock_phase` / `run_clock_phase_trials`.

| 구성 | 시행 | 거부 | lane drop | δ_max p50 / p95 / max | max pause p50 / p95 / max | 부호 (뒤처짐 / 따라잡음) | 95 % 를 통과시키는 ε_clk,alloc |
|---|---|---|---|---|---|---|---|
| `ur5e_p1b` | 200 | 0 | **0** | 1.514 / 4.931 / **18.212** ms | 1.627 / 4.171 / 18.388 ms | 196 / 4 | 49.808 mm |
| `iiwa7_leap` | 200 | 0 | **0** | 0.438 / 2.701 / **8.671** ms | 0.402 / 2.671 / 4.628 ms | 135 / 65 | 22.989 mm |

- **drop 0 이 꼬리를 믿을 근거다** (lane 이 넘치면 가장 큰 δ 가 빠진다)
- ε 열은 95 % 를 통과시키는 **역산 제안값**이다 (측정 당시 판정 **NOT_EVALUATED**). **사용자 확정 2026-09-20 (권장안)**: 두 구성을 덮는 ε_clk,alloc ≥ **49.8 mm** (p1b 가 구속) 를 예산이 확보해야 할 **하한**으로 둔다 — 측정의 역산이지 예산이 아니다

#### D-3 재판정 (2026-09-20, r_cap 확정 후)

위 49.8 / 23.0 mm 는 **`v_max` = 8.4 m/s (S0.7 가정값, `clock_phase.py` `DEFAULT_V_MAX_M_S`) 에서 역산한 값**이다. ε_clk = ‖v‖δ 는 속력에 비례하고, S4.2·S4.5 가 받을 수 있는 최대 속력을 LEAP **2.26 m/s** · P1b **1.84 m/s** 로 내렸으므로 (L6 §4.5) 필요한 ε 도 ×0.269 · ×0.219 로 준다. 예산 우변은 L3 §4.6 의 `n_σ`=2 로 `r_cap/n_σ`. 각 손은 **자기 $\Vert v\Vert_{\max}$** 에서 판정한다 — 더 빠른 공은 시계와 무관하게 못 받는다.

| 구성 | ε_clk,alloc @8.4 m/s | 손의 $\Vert v\Vert_{\max}$ | 그 속력에서 필요한 ε | r_cap | 예산 `r_cap/n_σ` | 시계 항 비중 | 판정 |
|---|---|---|---|---|---|---|---|
| `iiwa7_leap` | 22.989 mm | 2.26 m/s | **6.19 mm** | 31.0 mm | 15.5 mm | 40 % (분산 16 %) | **PASS** |
| `ur5e_p1b` | 49.808 mm | 1.84 m/s | **10.9 mm** | 24 mm | 12.0 mm | 91 % (분산 82 %) | **PASS(provisional)** — 2026-09-21 |

- **LEAP PASS** — 시계 항을 뺀 나머지 (vision σ_c·σ_ℓ, 추종 σ_trk) 에 √(15.5² − 6.19²) = **14.2 mm** 가 남는다
- **P1b 는 여유 없이 통과 (2026-09-21).** r_cap 은 사용자 제공 자세에선 없었고 탐색으로 자세를 다시 정한 뒤 생겼다 (L6 §4.5). 나머지에 √(12.0² − 10.9²) = **5.0 mm** 만 남고, 공통 속력 2.26 m/s 에서는 13.4 mm > 12.0 mm 로 **FAIL**
- **P1b 의 1.84 m/s 는 공식값이지 시연값이 아니다** — 손끝 평면 통과 시 폐쇄를 발동한 fly-in sim 에서 P1b 는 0.25 m/s 130 중 9, 0.5 m/s 3, 1 m/s 이상 0 (L6 §4.5). 실제 속력이 낮으면 필요한 ε 도 줄어 D-3 은 PASS 쪽 그대로이고, 목표 속력은 S4.4 의 몫이다
- 판정은 목표 속력에 묶여 있다 (8.4 m/s 면 LEAP 도 23.0 mm > 15.5 mm 로 FAIL). **S4.4 이후 (2026-09-22)**: 조건부 go 포구 속력 `iiwa7_leap` 1.2–2.6 m/s 는 PASS 유지 (22.989 × 2.6/8.4 = 7.1 mm < 15.5), `ur5e_p1b` 2.7–3.8 m/s 는 3.8 에서 49.808 × 3.8/8.4 = **22.5 mm > 12.0 mm 로 FAIL** — `ur5e_p1b` 조건부 go 의 네 번째 조건이 D-3 이다. **S3.5b (2026-09-22)**: gate 지도가 연 2.7–3.85 m/s 에서도 FAIL (3.85 에서 22.8 mm)
- `analyze_clock_phase` 는 판정을 내지 않는다 — D-S8-4 (c) 이후 분포 (공변량) 와, `--eps-mm` 을 줄 때의 "valid under eps" 비율만 낸다. `--plot` 이 시행별 분포를 그린다 (§13 S3 CSV 플롯 요구)
- 두 구성은 완전히 동등하지 않다 — `ur5e_p1b` 는 컨트롤러 5개 (비활성 `demo_inference_controller` 포함), `iiwa7_leap` 은 4개를 인스턴스화하므로 차이 일부는 bring-up 에서 올 수 있다

| 항목 | 방법 | 기준 |
|---|---|---|
| 무부하 (S3.1a) | 구성별 (로봇 2종) 발사 ≥ 200 회 | 무효율 ≤ 5 % — **실측 완료 2026-09-20 (§5.1). LEAP PASS, P1b PASS(provisional)** (2026-09-21, 각 손의 $\Vert v\Vert_{\max}$ 기준) |
| 부하 (S3.1b) | 포구 컨트롤러 + 계획기 + `sim_estimator_node` 동시 구동, 구성별 발사 ≥ 200 회 — **충족 (S8-E, 2026-09-26)**, 아래 | **D-S8-4 (c) 로 교체**: ε_clk,alloc 은 판정이 아니라 **공변량** — 시행별 δ(t_commit)·δ(t_c)·tick overrun 수·최대 tick 간격을 기록하고, D-3 효과는 같은 seed 투척의 부하 A/B (affinity on·최소 프로세스 대 인위 부하) 로 상계한다 |
| 예측 일관성 | vision 예측 궤적 대 ground truth (같은 wall 시각 축) | 오차가 δ 가 큰 구간에서만 커지는지 확인 (기록) |
| stamp 도메인 | `sim_estimator_node` 의 stamp 가 wall 인지 | `use_sim_time=false` 에서 wall — **확인 2026-09-20** (S3.4: origin stamp = capture stamp = sim 의 wall `now()`, 재시작에도 역행 없음) |

- **S3.1b** 는 S6 에서 하지 않았고 S8-E 자체 시행 (p1b tennis·beanbag 400 + leap 200) 이 ≥ 200 을 채운다 (S8-B 튜닝 200 은 요약 합산 병기 — D-S8-16 ⑤; clock lane 은 `sim_lanes:=true`, 2026-09-24). **결과**: 시계 공변량 시행 600 + S8-B 요약 198, |δ(t_commit)| p50/p95/max leap 0.06 / 1.1 / 5.1 ms · tennis 재실행 0.24 / 1.25 / 5.1 ms (원 0.27 / 118 / 230 ms 는 host 부하 unit, D-S8-17; §4.4 S8-E 결과)
- 공변량 전환 근거 — 파일럿 (`260924_1218`, non-RT 개발 PC): δ_max p50 4.5 / p95 10.7 / max 16.7 ms, ε 12 mm 이면 유효 7/25 라 무효율 상한을 그대로 쓰면 개발 PC run 이 전부 NOT_EVALUATED 가 된다

시행 수 200·무효율 5 % 는 제안값이다 (사용자 확인, §7.3). 무효율이 상한을 넘으면 `/clock` 방식을 포함해 D-3 을 다시 결정하되, 이 상한은 **S3.1a 무부하 판정에만** 쓴다 — **S8 (2026-09-24, D-S8-4 (c))**: clock 위상 오차는 무효 사유가 아니라 공변량이다 — 무효·ITT 정의는 §1a 기준 1 ②.

## 6. D-7 계획기 스레드 구성

기존 MPC 스레드와 같은 방식으로 생성하고 기능만 planner 로 한다. **구현된 사양 (클래스·기동·수명·데이터·배치) 은 [L3 §5.3](L3_planner.md)** 이 갖고, 이 절은 결정과 L3 에 없는 근거·제약만 둔다.

**E-7 결과 (2026-09-23 사용자 승인, 결정 J).** 계획기는 MPC 와 **같은 역할** (RT 루프에 해를 공급하는 solver 스레드) 이므로 **새 layout role 없이 기존 `mpc` role 을 쓴다** (`SelectThreadConfigs().mpc.main`, 스레드 이름 `mpc_main`). 레이아웃·generator·oracle·검증기 표 변경 0, profile 은 `mpc_on`/`mpc_off` 그대로 (결정 B — launch 의 `enable_mpc` 하나). CM 은 active 컨트롤러를 하나만 두고 이전 것을 deactivate (`Pause`) 하므로 같은 코어에서 도는 FIFO 는 하나다 — R-1 switch 테스트가 확인한다. D-7a 는 `mpc_main` 값을 바꾸므로 MPC 에도 적용되고, 둘이 갈리면 그때 role 을 분리한다 (별 E-7).

**2026-09-19 분석 중 L3 에 없는 것.**

- **기반.** `rtc::PeriodicRtThread` (rtc_base) 는 진입 시 `ApplyThreadConfigVerbose` 가 **실패해도 무시하고 계속 실행**한다 (§7.2 결정 방식 4 의 근거). `rtc::mpc::MPCThread` 는 그 위의 얇은 subclass 로 DemoWbc 가 소유하고 소멸자에서만 join 한다 (use-after-free 수정 이력)
- **클래스.** `MPCThread`/`MPCHandlerBase` 가 아니라 `PeriodicRtThread` 의 **형제 subclass** — 상속하면 PlanSnapshot 을 `MPCSolution` 에 억지로 넣게 된다. 같은 기반의 4번째 소비자라 P5·ARCH-3 을 만족한다. 스레드 소유는 `integrated_bringup` 바인딩 (D-1)
- **수명.** DemoWbc 관용구 (aux 타이머가 `planner_timing_log.csv` drain) 에 **join 만 `on_cleanup`** (S6-A `/code-review`: 소멸자까지 살리면 재구성한 설정에서 resume 돼 plan box writer 가 둘이 된다). `Pause()` 뒤 한 번 더 게시될 수 있는 plan 은 RT 가 activation generation 으로 거른다 (D-23) — DemoWbc 가 MPC 해에 이 장치를 두지 않은 것은 선례가 아니라 미해결 gap 이다
- **데이터.** RT → planner `rtc::SeqLock<RtStatePod>`, planner → RT `rtc::SeqLock<PlanSnapshotPod>`. **SeqLock payload 는 trivially copyable 이어야 하고 `Eigen::Vector3d` 는 아니다** (Eigen 3.4 에서 확인) → PlanSnapshot·궤적 스냅샷은 `std::array<double, N>` 기반 POD (S1.2). 소비는 D-21, 출처는 D-22
- **J 로 불필요해진 것.** 새 role 을 전제한 E-7 작업 목록 (전 tier × profile 배치표, generator 의 shield 코어 도출, `catching_on/off` profile, tier 4 정책, `all_configs` absent-role 필터 — 없으면 zero-init cpu_core 0 이 허위 충돌, #349) 은 기록으로만 남는다. role 을 분리하게 되면 압축 전 원문 (`b0ea0996`) 을 본다

**복사하지 않을 것.** MPC 스레드의 cross-mode swap 은 phase 전환 때 그 스레드에서 handler 를 새로 만든다 (heap·YAML·try/catch) — invariants.md §RT Path 의 알려진 위반이라 템플릿으로 쓰지 않는다. 통계 mutex 와 `fprintf` 는 E-9 결정으로 제거 (2026-09-19, §7.3).

세부 선택 D-7a~d 는 §7.

## 7. 세부 결정과 후속 결정

### 7.1 확정 (2026-09-19)

사용자 결정 (P-):

- P-1 S9 이전 E-STOP 임시 기준: 해제 후 자동 재개 금지 + q_c·CLIK 앵커 q_meas reseed 만 (S5.1), S5 전 `[CONCERN] E-8`, S5 게이트에 race oracle·`/security-review` (§4.4 S5). **E-8 승인 2026-09-22 (사용자, (a)~(d) 문구 그대로)**. 전체 정책은 S9
- P-2 `docs/dynamic_catching/` 는 브랜치 `docs/dynamic-catching-plan` 에 커밋
- P-3 Epic issue 만 생성, ball_perception 요청 이슈는 만들지 않는다 (사용자가 직접 개발 중 — 레이아웃 변경은 S5.2 파서의 필드·datatype 검사와 해시 진단이 감지)

세부 결정 (A-·D-7x·C-):

- D-7b slot: 빈 slot 에 새 role — **E-7 결정 J 로 대체 (§6)**
- D-7c 기동: event 구동 — 새 궤적 수신 시 eventfd 로 깨우고 대기 상한을 둔다. `JitterMeaningful()` false
- D-7d IK 감쇠: `DifferentialIk` 의 σ_min 적응 λ 를 수용하고 L3 G3-G (IK 수렴률) 로 검증, 부족하면 그때 일반화
- D-12 추측하지 않고, 값이 없으면 YAML 에 provisional 로 표시해 TBD 검사가 실기 arm 을 막게 한다
- A-1 Sprint Contract: Epic 기준 + 단계별 `[SPRINT]` (§1a)
- A-2 D-7a 판정 기준: §7.2 결정 방식 6
- A-3 공분산 (TBD-COV-01): RT 스냅샷에서 분리해 계획기 쪽 버퍼에만 (같은 token, D-22), NaN (모름) 처리도 계획기 한 곳
- A-4 계획기 탐색: 1차원 시간 탐색 + IK 로 시작, NLP 전환 경계 유지 (§8)
- A-5 DECEL 은 t_c 시각 기준 진입, 지문 센서는 결과 판정·abort 전용 (L7 §4.1 권장 채택)
- A-6 COMMITTED 이후 stale 은 동결 plan 으로 계속, 상한 초과 시 ABORT_SAFE (L7 §4.2 권장 채택)
- A-7 → D-15 (vision 요구 사양은 제어기가 정하고 sim 을 맞춘다)
- S0.7 후속 (2026-09-19, 사용자 — 근거 §4.4 S0): 지평 요구는 R1 (`io.horizon_min` 도 R1, R2 는 첫 plan 실패 최악값 기록), 대기 자세는 겨냥점 근처 (구체 자세는 S3.5a), `kCap` 40 (provisional)
- D-15 sim profile 지평: 0.8 s, 간격 0.05 s, **16 점** (0.05…0.80 s, t = 0 없음), ≤ 30 Hz (2026-09-19, S0.7 권장). 설정은 사용자, S3.4 가 실측 확인. **S3.6 (2026-09-22) 이 1.0 s / 20 점으로 올렸다** (§4.4 S3.6, §7.3)
- D-18 manipulability: 팔 관절 열 5행 w₅ (병진 3 + 접근축 2), threshold 는 이 정의에 대한 값이고 출하값은 로봇별이다 (`ur5e_p1b` 0.1 · `iiwa7_leap` 0.174, §11)
- D-18 발사 영역: base frame 수평 거리 4 m 원호 (좌우 투척 포함), world z 1.5–2.0 m, 발사점 → 포구점 T_f ≥ 1.0 s (2026-09-19, S0.7 후 사용자 — 짧은 직선 투척 제외, 상한은 지도 결과)
- C-1 vision `validity`: 한 점이라도 VALID 가 아니면 메시지 전체 거부 (S3.4 에서 부분 무효가 나오면 S5.2 전 재검토)
- C-2 `header.stamp` 기반 나이 거부는 두지 않고 원점 지연 (수신 wall − stamp) 은 진단만 (오래된 원점은 지평 검사가 거른다). 미래 stamp 거부는 별개 (§3.1)
- C-3 `planner.ik.manip_min` 은 D-18 게이트로 대체. IK·게이트는 w₅, 검증은 w₆ 일 수 있다 — §11 의 w₅/w₆ 병행 규칙
- C-4 포구 후보마다 IK seed 는 대기 자세 고정. IK 반복·예산 증가는 S6.3 에서 측정
- S2 (2026-09-19~20 사용자): PR 단위 S2.1 → S2.2 → S2.3a·S2.5 → 마감, S2.2a 행 선택형 확장 (L5 §5.1), S2.5 도구는 rtc_tools python, golden 비교는 Release 비트 일치·sanitizer 상대 1e-12, `QPSolverWrapper` 비유한 회복은 golden 전에 선수정, **η_τ = 0.8** (§9, S3.5a 뒤 표본 범위만 바꿔 재생성)
- E-1 (S0.6, 승인 2026-09-19): D-2 (3) 의 stamp 사용을 §3.1 문구 그대로 invariants.md §Clock 시간축 규칙의 기록된 예외로 둔다
- E-3 (S0.8, 승인 2026-09-19): D-14 (.srv)·D-20 (.msg) 를 한 번에 발화 (규칙이 인터페이스 "추가" 를 다루지 않고 선례가 `f95ca5aa` .msg 발화 · `4d98c15f` .srv 미발화로 갈려 보수적으로). `PublishRole` 불변이라 E-11 미발화, PROC-3 은 각 변경 때

### 7.2 D-7a 스케줄러 — RT(SCHED_FIFO) 검토, 측정으로 확정

사용자 방침: timing 이 중요하므로 RT 로 검토하고, 이득이 크지 않으면 SCHED_OTHER 로 한다.

**timing 이 영향을 주는 곳.** planner 는 RT 루프와 다른 코어에서 SeqLock 으로만 결과를 넘기므로 스케줄링 클래스는 RT 루프 결정성과 무관하다. 영향을 받는 것은 **vision 도착 → plan 게시 지연과 그 꼬리 (p99·최대)** — commit (t_c − T_freeze) 시점의 plan 신선도다. D-2 에 따라 plan 의 시각은 절대 steady 시각이라 **늦은 plan 이 틀린 시각을 쓰지는 않는다 — 남은 시간만 준다.** 지연 중 DDS 수신 → 파싱 → eventfd 는 `nrt_callback_executor` (SCHED_OTHER, 단일 스레드, lifecycle 서비스와 공유) 라 FIFO 와 무관하고, FIFO 가 줄이는 것은 깨어남 지연과 선점으로 늘어나는 탐색 꼬리뿐이다.

**비용.** FIFO 이면 planner 코드가 RT-1~10 에 구속되지만 이미 사전 할당·noexcept 이고 진단은 SPSC 로 aux drain 이라 **추가 비용은 작다**. 개발 PC 는 PREEMPT_RT 가 아니고 `sched_rt_runtime_us` 950000 (95% throttle) 이라 **판정은 제어 PC 에서만** 한다.

**결정 방식.**

1. planner 코드는 스케줄링 클래스와 무관하게 **RT-1~10 준수로 작성**한다 (S6 게이트: `ScopedAllocGate`·`ScopedNoMalloc` 할당 0, noexcept, 로깅은 SPSC). 그러면 FIFO/OTHER 는 `thread_layout.yaml` 값으로만 바뀐다. RT 쪽 `PlanSnapshot` 읽기는 writer 가 OTHER 여도 D-21 의 측정 (최악 재시도 시간) 을 통과해야 한다
2. 초기값은 **FIFO** (rt_callback 보다 낮은 우선순위)
3. 제어 PC 에 부하 (포구 컨트롤러 + sim 또는 실기 드라이버 + vision) 를 건 채 두 정책을 각각 측정: 수신 → 게시 지연 p50·p99·최대, 예산 초과율
4. 설정값은 증거가 아니다 — 각 run 에서 planner 스레드 이름의 모든 TID 의 policy·priority·논리 CPU·cpuset mask 를 `/proc` 에서 기록 (`verify_rt_runtime.sh`), 기대와 다르면 그 run 은 NOT_EVALUATED
5. 지연은 `{snapshot_sequence, recv_steady_ns, wake_ns, publish_ns}` 이벤트 레코드를 SPSC 로 남겨 잰다 (공용 timing CSV 는 수신 → 게시를 표현하지 못한다)
6. 판정 (A-2): FIFO 가 p99 지연을 `planner.budget_s` 의 10% 이상 줄이지도, 예산 초과율을 줄이지도 못하면 **SCHED_OTHER 로 전환**. 표본 ≥ 1000 시행 (L3 G3-C 와 같은 규모)
7. 상류 구간 (nrt_callback 수신) 이 지배적이면 D-7e 로 수신 경로를 따로 검토한다

**결과 (2026-09-23).** 3~6 의 제어 PC 측정은 사용자 결정으로 **생략**했다. 정책은 2 의 초기값 (FIFO) 그대로이고, 코드는 1 에 따라 RT-1~10 을 지키므로 나중에 OTHER 로 바꿔도 `thread_layout.yaml` 값만 바뀐다. 재판정하면 3~7 을 따른다 — 측정 CLI 는 #537 코멘트 5793878042 (`s6d.sh`, dev PC 에서 끝까지 동작 확인).

### 7.3 단계별 확정 기록과 열린 항목

실제로 열린 것은 첫 표뿐이고 나머지는 단계별 확정 기록이다. S10 몫의 진행은 추적 이슈 S10 (#613) 이 갖는다.

**열린 항목**

| 항목 | 상태 | 소유·단계 |
|---|---|---|
| D-12 손 토크 (S7.3 충격 게이트 입력) | P1b 손 관절 운용 한계 (nominal·continuous·peak·설정값 중 무엇) 와 권위 출처. 설정·모델값은 3.0 N·m (YAML `max_torque` = URDF `effort` = MJCF `forcerange`), 1.5 N·m 는 작성 시점 사용자 진술 (CATCHING_MASTER §1.3), 실기 실측 (2026-09-22 사용자) `thumb_cmc_fe` 7 rad/s 에서 약 2 N·m 는 1.5 와 양립하지 않는다 (L6 §4.2). 확정 전: sim G7-B3 3.0 · 실기 1.5 provisional 병기, 토크 비교 `NOT_EVALUATED` | 사용자 → S10 (#613) |
| D-12 공 사양 · 실기 T_close,tot | S4.4 는 출하 sim 공 (ITF Type 2) 으로 판정 — 공 사양이 바뀌면 fly-in (L6 §4.5) 부터 다시. 실기 T_close,tot 은 S10 단계 D (G6-D) | 사용자 → S10 (#613) |
| 실기 공분산 검증 수단 (G8-G 대체) | L8 §6 의 세 가지 중 선택; 셋 다 안 하면 §12 위험으로 둔다 | S10 (#613) |
| S10 로 넘긴 실기 값 | 출하 `joint_cmd.lag.T_arm` (L5 §6) · `reference.a_max` (출하 30, provisional) · 실기 가속 envelope (D-S8-18) · `budget.sigma_trk` (출하 0) · `track_err_abort` · `d_eff`·`r_cap` 투척 보정 (L6 §4.5 step 2 — sim 보정은 생략: sim 값은 MJCF 게인에 의존해 실기 $T_{close,tot}$ 측정 전에는 보정해도 잠정값이다, 사용자 2026-09-29) · 선행시간 L 의 실기 확정 | S10 (#613) |
| TBD-WS-01 (바닥·작업셀 경계) | 닫은 기록 없음. sim 은 S6 결정 I 의 `catch_box` 가 대신하고 실기 `catch_box` 는 provisional. 실기 park 키 값 결정과 함께 닫는다 (사용자 2026-09-29) | S10 (#613) |

**확정된 결정 (종전 '사용자 결정이 필요한 것')**

| 주제 / ID | 최종 결정 (날짜, 누가) | 소유 절 |
|---|---|---|
| E-8 | 승인 (2026-09-22 사용자): P-1 최소 계약 (a)~(d) 문구 그대로, S5 게이트 G7-H 4종 + `/security-review` | §4.4 S5 |
| E-7 | 승인 (2026-09-23 사용자, 결정 J): 새 role 없이 `mpc` role 재사용 | §6 |
| D-24 | (a) (2026-09-22 사용자): `rtc_base` `DeviceState` 센서 lane 에 `recv_steady_ns`·`sequence`·`valid` (P5, PROC-3 전체 회귀). (b) 포구 전용 mailbox 는 중복 lane 이라 기각. 배선 S5.2e | §4.4 S5 |
| G5-E substrate | (ㄱ) fixture 전용 지연 주입, 런타임 불변 (2026-09-22 사용자) — `prediction.lead` = $T_{arm}$ 효과를 S6 전에 본다. (ㄴ) `NOT_EVALUATED(substrate)` 는 부호·크기를 S10 에서 처음 보게 돼 기각. G5-E S5.3 PASS. G8-E 는 D-S8-1 (2026-09-24) 로 sim 런타임 lead on/off (S8-B) | L5 §9, L8 §9, §4.4 S8 |
| QP solve time 예산 | 2026-09-22 사용자, provisional: tick 2000 µs 기준 p99 ≤ 400 µs (20 %) · 최대 ≤ 1500 µs (75 %) — 분위수는 L8 의 "분산의 꼬리", 20 % 는 backend 왕복·로깅·계획기 스냅샷 몫 | §4.4 S5, L5 §9 G5-C |
| sim 공 lane 지터 (결정 ⑤) | 2026-09-22 사용자, 구현: 공 토픽 stamp = 발사 순간 기준 sim 시간축을 wall 에 얹은 값 (D-2·D-3 불변), sim future 허용치 `max_future_skew_s` 0.1. 기각: throttle anchor·wall clamp·발행률 상향·wall 타이머·`init.max_span_s` 완화. 25 ms 초과 간격 9.2 → 0.02 % | §4.4 T_det 재실측 |
| sim profile | 2026-09-22 사용자: 파일·지평·점 수는 §1 D-15. 같은 파일에 공분산 (5 mm)² 대각, `max_future_skew_s` 0.1. 종전 1.05 s / 21 점은 버스트 검출 탓 | §4.4 S3.6 |
| D-12 floor·시행 수 | D-S8-3 (2026-09-24): n_valid 200, floor 0.35, 2026-09-26 동결 (D-S8-16) — 경위는 §1a. 공은 tennis + `beanbag` arm (D-S8-11, D-S8-16 ④) | §1a 기준 1 |
| D-18 개정 (S3.5b 결정 A) | 2026-09-22 사용자: sim 은 현 씬, 릴리스 base 기준 0.2–0.5 m, 거리 1.0–1.5 m. 4 m·T_f ≥ 1.0 s·릴리스 1.5–2.0 m 는 추정기 정확도용이지 제약이 아니다 (2026-09-21 사용자). base 를 올린 씬은 실기 rig 높이가 정해질 때 | §4.4 S3.5b·S4.4 |
| D-18 T_f 상한 | 전제가 대체됨: T_f ≥ 1.0 s 창이 제약에서 빠졌고 목표 분포는 S3.5b 상자로 정의된다 (T_f 는 그 결과값). 별도 상한값은 기록되지 않았다 | §4.4 S3.5b·S3.6 |
| D-16 개정 (결정 B) | 2026-09-22 사용자: 도달시간 게이트는 상수 box 층과 토크 검사 층 병기, 판정은 토크 층 (provisional); 필요 시점이 S3.5b 전으로 당겨졌다. 런타임은 닫힘: CLIK `accel_constraint: dynamic` (결정 K, S6-C2), 계획기 도달시간 한계 = 실행 envelope (D-S8-18, 2026-09-27; 실기 envelope 은 S10) | §9 |
| 1차 목표 로봇 | `ur5e_p1b` (2026-09-22 사용자). G8-D2 조건부 규칙 (T_det 실측 뒤 평가 대상) 과 S6 착수 조건 세 가정은 §1a | §1a |
| `reference.v_max`·η_v (결정 D) | `v_max` = 수락 후보 LP $v_{dir,\max}$ 최대 / η_v, η_v 는 관절 속도 한계에도 적용. p1b 정격 3.1416 rad/s 는 `sim.yaml` overlay 로만 (실기 `_base.yaml` 불변) | L4 §6 |
| `planner.hand.d_eff` (TBD-HAND-04) | 2026-09-22 사용자: 시각 발동 fly-in 허용 상대속도 × $T_{close,tot}$ — P1b 0.2815 m, LEAP 0.1047 m. 포켓 깊이 (0.095 / 0.080 m, S4.5) 가 아니다 (그 값으로는 p1b 지도가 0). provisional (런타임 손 발동도 시각 발동이어야 하고 (S4a), 지터 0 가정). 대안 키 `planner.hand.v_rel_max` 는 채택 기록 없음 | L6 §4.5 |
| `supervisor.decel.a_dec` | 10 m/s² (2026-09-22 사용자, provisional): 정지점 게이트가 후보의 40 % 를 거르고 S4.4 토크 한계 $\hat v$ 방향 가속 (p1b 중앙값 43, 회전자 관성 10 배 가정 21–23 m/s²) 의 절반 아래. "a_max 확정 시 ≤ 재검": a_max 는 30 (D-S8-18, provisional) — 기록된 두 값으로는 10 ≤ 30 이고 검증기가 a_dec ≤ a_max 를 요구한다 (A-S5-11). 재검 수행 기록은 없다 | L7 §4.3 |
| 선행시간 T_det | 실측 (2026-09-22, 24 비행; 발사 → 첫 VALID 예측, S3.6): 결정 ⑤ 뒤 stamp p50 0.050 · max 0.100 s, 첫 계획 시각 전부 지도 가정 0.24 s 안, H_req 0.99 s · n 20. L 은 아래 "선행시간 L" 행 | §4.4 T_det 재실측 |
| rollout 게이트 (L3 §4.8) | 결정 E (2026-09-22 사용자): S3.5b 는 `NOT_EVALUATED(S6)` (함수가 S6.3 전에 없음). S6-C 가 구현 | §4.4 S6 |
| D-3 제안값 | 시행 수 200 확정 (2026-09-20; 시행 수·무효율 상한은 제안값). r_cap 뒤 재판정 LEAP PASS · P1b PASS(provisional), S4.4 속력으로 재계산. S8 에서 δ 는 공변량 (D-S8-4 (c)) | §5·§5.1 |
| d_eff · r_cap (접촉 sim) | 2026-09-20 사용자 승인: LEAP 0.080 · 0.031 m, P1b (자세 재탐색 2026-09-21) `d_eff` ≥ 0.095 · `r_cap` 0.024 m (provisional). `d_eff` 키는 위 fly-in 등가값으로 재정의 | §4.4 S4a (S4.5) |
| NLP 전환 (§8) | 2026-09-29 사용자: v1 은 전환하지 않는다. S8-E 신호 (첫 plan 0.20 s · plan 유효율 p50 0.13–0.17 · 교체 0) 가 전환을 시사하지 않는다. MPC 도입은 검토 중. 재검토 조건: S10 실기의 IK 수렴률 저하·예산 초과. **개정 (2026-09-29)**: MPC 도입을 별도 계획으로 착수 — [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) MD-6 | §8 |
| 선행시간 L | 2026-09-29 사용자: 0.14 s 유지 (provisional). L 을 따로 잰 기록은 없으나 S8-D 실측 첫 계획 (T_det + L) p50 0.195 s 와 T_det 실측 p50 0.050 s 의 차 0.145 s 가 가정값과 맞는다 — 서로 다른 실행의 중앙값이라 근사다. `io.horizon_min` 0.51 s 가 이 값에 걸린다. 실기 확정은 S10 (#613) | §4.4 S3.6 |
| QP solve time 예산 조이기 | 2026-09-29 사용자: 조이지 않는다 (p99 ≤ 400 · 최대 ≤ 1500 µs 유지). S5 실측 p99 79.4 µs 는 여유가 크지만 최대 1411.5 µs 가 예산에 가깝고 개발 PC (비-RT) 측정이다. 제어 PC 판정 G8-A (S10 #613) 에서 다시 본다 | §4.4 S5 |
| σ_ℓ abort | 2026-09-29 사용자: v1 제외. σ_ℓ 는 `planner_events.csv` 기록만 (D-S8-7 (a)) 이고 stale 은 steady 수신 나이로 판정한다. 실기 공분산 검증 수단 결정 (S10 #613) 과 함께 재검토 | §4.4 S8 |
| D-7e (조건부) | 2026-09-29 사용자: 보류 종결 — vision 수신 경로가 지연을 지배할 때의 대안인데, 트리거인 §7.2 제어 PC 측정이 생략돼 (2026-09-23) 조건이 관측된 적 없다. 재개 조건: S10 제어 PC 측정에서 상류 구간이 지배적일 때 (재판정 CLI #537 5793878042) | §7.2 |

**S7 착수 전·중 확정 (#537 S7 결정 2026-09-23 · 2026-09-24)**

| 항목 | 내용 |
|---|---|
| homing 위치 | 관절공간 (per-joint 사다리꼴, QP/CLIK 비의존), 무장 latch 아래에서만 (P-1 (e)) — L4 §5.3·L7 §4.1 |
| `REF_SATURATED` | 연속 `supervisor.sat_ticks` tick 이면 승격 (D-S7-4). S7 60 provisional → p1b 80 (S8-B), leap 60 (S8-D, 자료가 임계를 정하지 못함); 코드 기본 60 은 키 없는 config 용 |
| hold 규칙 | `robot.hand.hold.mode: close_target \| measured_offset` + `hold.delta_rad`, 출하 `close_target` |
| ν̄ / `PRED_INCONSISTENT` | 은퇴 (D-S8-7 (a)): 생산자를 만들지 않고 `io.pred.nu_reg` 도 은퇴 (L1 §6). 예측 일관성은 `/ball_perception/debug/{innovation,nis}` 를 `vision_lane_probe` 가 기록해 오프라인으로 (S8-A). `kPredInconsistent` 는 발화 0 명시 면제 (L7 §4.2) |
| σ_ℓ 소비자 | 기록만 (D-S8-7 (a); D-8 포화 빈도와 함께 S8 로 넘겼던 것): `planner_events.csv` `sigma_l` 을 `stale_committed_max_s` 입력으로 |
| RETREAT release | 닫힌 손은 판정과 무관하게 대기 자세 도착에서만 `q_pre` 로 연다, 안 나간 commit 은 진입 시 취소. Q12 · Q14 교체 — 지문만 보는 판정이 손바닥 위 공을 Missed 로 읽어 떨어뜨렸다 (sim `260923_2336`). L7 §4.1 |
| G7-B3 충격량 상관 | S8 이월 → S8-E 기록 (판정 임계 없음), 토크 `NOT_EVALUATED(sim clamp)` |

**S8 착수 시 확정 (2026-09-24 사용자 — #537 5804952754 → 5807767896)**

결정 표 (D-S8-1~12 · C-25) 는 §4.4 S8 이 SSoT 다. 여기는 §7.3 행에 미친 파급만.

| 항목 | 파급 |
|---|---|
| `joint_cmd.lag.T_arm` (sim) | 출하 0. S8-B overlay 만 0.05 + `lead_enable` + `T_freeze` 0.37 (D-S8-13 — L3 §4.11 하한 0.3615, 검증기는 0.36 도 통과). C-25 조합은 은퇴, 실험 overlay 는 D-S8-20 으로 repo 에서 제거 |
| D-3 판정 | 무효율 상한 → 공변량 (D-S8-4 (c)) |
| `sat_ticks`·`stale_committed_max_s`·`track_err_abort` | S8-B 로 닫힘 (#537 5810866922): p1b 80 · 0.10 · 0.42; leap 은 S8-D (2026-09-26) 60 · 0.10 · 0.48 (첫 homing 0.238 rad × 2). §4.4 S8-B·S8-D |
| RETREAT 손 대기 timeout | `robot.hand.T_release_timeout` (D-S8-6, S8-C, PR #579 → `126e3513`); 키가 없으면 $T_{close,e2e}$ 배수 $m=2\max(1/\eta,\ \ln(S_{\max}/q_{tol})/\ln\tfrac1{1-\eta})$ (#537 5823291259). S8-B hang 0 (G8-A2 600/600). SSoT L6 §6 |
| G8-D 성공 정의 | truth 기반 + 슈퍼바이저 혼동행렬, 무효 = rig 실패만, ITT 하한 (§1a) |
| CLOSING 1 tick 늦은 기록 | 문서 기록만 (D-S8-10 (a)) |
| L3 가중치 · NLP · `a_max` · `sigma_trk` | 제안·기록만 (D-S8-16 ⑦) — 단계별 결론: |

- **S8-E** (#537 5842632283): `sigma_trk` sim 0.005 m 제안 (서보 잔여 p95) · `a_max` 21 유지 제안 · 가중치 변경 제안 없음 (교체 0/600) · NLP 신호 없음
- **S8-F-1** (#537 5846528236): `v_max`·`a_max` 가 속력 절벽이라 재측정이 S8-F 후속 ② 1 순위; 가중치 A/B 효과 없음 → 후속 ① (γ 창)
- **S8-G** (D-S8-18): 출하 `a_max` 21 → 30 (provisional, 실기 S10), `sigma_trk` 제안 유지
- **S8-H** (D-S8-19): 후속 ① 종결 — 가중치·창 어느 쪽도 순위를 바꿀 정보가 없다. L3 가중치 튜닝은 이것으로 닫혔다

**단계에서 정할 것 (TBD-WS-01 외 전부 닫힘)**

- S1.7 키 이름 — `planner.gamma.eta_v`·`supervisor.decel.a_dec`, 가속 box 는 S2.5 에서 `derived_accel_limits.<group>.qdd_max`. 새 키 `core.ball.provisional`·`planner.catchability.manipulability_min.provisional`·`robot.hand.provisional` (기본 true, 2026-09-19 사용자), catch frame (D-17) 은 S2.3a
- G0-C ωh 경계 — 범위 유지, 게이트 문구 수정 (2026-09-19 사용자, L0 §5.3·§9)
- L7 전이표 (S1.8) 해석 3건 — S7.2 확인 (PR #571): `Reason::kNone` = 정상 전진, IDLE homing 은 `kIdle` 안 (별도 Mode 아님), ARMED→IDLE 은 전용 사유 없이 `kParamsTbd` 재사용
- S2.2a CLIK 확장 구조 (행 선택형) · S2.2 세부 — 닫힘 (2026-09-19, L5 §5.1, §4.4 S2)
- S3.1a ε_clk 할당 비율 — 닫힘 (2026-09-20, §5.1)
- S5.2 — 되감김 A-S5-4·A-S5-14, 공분산 A-S5-5 (token 유지 D-22, 시간 보간 규칙 없음, S6 후보는 vision 격자), `frame_id` 는 S3.4 실측 `world` · 모델 world 변환은 S6, 유령 트랙은 침묵 = 소실 (`io.t_stale`). C-1 을 바꾼 기록은 없다 (S3.4 부분 무효 0)
- S5.3 abort 식 — A-S5-10 · S7 항목 — 위 S7 표, `stale_committed_max_s` 는 S8-B

**S5 착수 시 확정 (2026-09-22 사용자, 권장안 그대로 — #537 5773959608)**

착수 대조의 정정 8건 중 남길 것: D-24 의 `valid` 는 `DeviceState::inference_enable` 로 이미 있어 추가분은 `recv_steady_ns`·`sequence` 둘, 센서 lane backend 는 2종 + CM 복사 (`ur_driver_native` 는 lane 없음).

| ID | 결정 | 근거 |
|---|---|---|
| A-S5-1 | 실기 config 소비 키에 provisional·TBD 가 있으면 DISABLED park (configure SUCCESS · activate 거부) — S4.0 sim 전용 가드 대체. sim 은 A-S5-12 | CM 은 한 컨트롤러 configure 실패로 전체를 거부한다 |
| A-S5-2 | `io.future_tol` 실기 1e-3 (본 키) · sim 0.1 (`sim.io.future_tol`), `io.t_stale` 0.10 | 한 파일 공유라 sim 값은 `sim:` 섹션; 공 lane stamp 가 sim 축 (⑤) |
| A-S5-3 | 조작 채널 = 파라미터 `catching.enable` (기본 false), tick 이 E-STOP·fault 때 latch 를 내린다 | msg·srv 는 E-3 대상; P-1 (c) 가 메커니즘이 된다 |
| A-S5-4 | 되감김: 같은 generation 에서 이하이면 거부, generation 이 바뀌면 리셋 | S3.4 되감김 0 — fail-closed 최소형 |
| A-S5-5 | S5.2d 의 $\bar\nu$ 는 S7 로 (이후 은퇴), $J$ 만 진단, 공분산은 token 실은 `CovarianceSnapshot` 으로 계획기에만 | 시각 보간 규칙이 없어 임의 정의 박제를 피한다 |
| A-S5-6 | D-16 box 는 `robot.arm.accel_limits_path` (결정 당시 이름 `accel_limits_file`) + `accel_limits_group` 으로 파일을 읽고 `adopted: false`·부재는 거부 | 도구 산출 파일이 SSoT, 값 복사 금지 (L5 §6) |
| A-S5-7 | CLIK 한계는 device config ∩ URDF 에서 `limit_margin` 안쪽, `qd_max` 는 `max_velocity` | 사본 금지 (L5 §6) |
| A-S5-8 | S5.5 oracle plan `diagnostic.oracle_plan:` 을 RT tick 이 고정 `PlanSnapshot` 으로 소비 | 계획기 전에 S5 추종을 보려면 |
| A-S5-9 | G5-E 지연 주입은 미설치 fixture 의 폐루프 plant | (ㄱ) 의 구현, 런타임 불변 |
| A-S5-10 | abort 감속: $\dot q_i\leftarrow\mathrm{sign}(\dot q_i)\max(\lvert\dot q_i\rvert-\ddot q_{\max,i}\Delta t,\,0)$, $q_c\leftarrow\mathrm{clamp}(q_c+\dot q\Delta t)$ — 할당·QP 없는 순수 함수 | L7 §4.1 이 S5.3 에 남긴 자리 |
| A-S5-11 (당시 `[제안]`) | `reference.provisional` 신설 (기본 true) + 출하 `reference:` 블록: `v_max` p1b 3.5 · leap 1.8 m/s, `a_max` p1b 21 · leap 35 m/s² (S4.4 의 보수적 끝, `a_dec` 10 위 — 검증기 요구). 블록 provisional 이면 sim 경고·실기 차단. 값은 3-1 로 채택, p1b `a_max` 는 이후 30 | 사고 (2026-09-22): 소비 키인 `reference.*` 블록이 출하 프로파일에 없어 sim configure 거부 → CM 이 전체를 거부해 ur5e_p1b sim 로봇이 안 떴다 |
| A-S5-12 | sim 도 park — 채택 (2026-09-23 사용자, 3-2), S6-A 구현. 범위·일관성 위반 (caging 간극 등) 은 설정 실수라 여전히 configure FAILURE | A-S5-11 사고가 같은 근거를 sim 에서 실증 ("로봇이 안 뜬다") |
| A-S5-13 | 운동 중 모드에서 ARMED 조건 상실 → `ABORT_SAFE` (전이표 7행) | 행이 없으면 관절 명령이 언다 — disarm 후 4 s·3240 tick `APPROACH` 유지 실측 (`/code-review`) |
| A-S5-14 | 스냅샷 newness = (epoch, 번호) + `seen` 플래그 (L1 §5.3) | 번호 재시작·0 번 센티넬 겸용으로 새 표본을 이미 소비한 것으로 읽었다 |
| A-S5-16 | 수신축 clock 읽기를 `rtc::SteadyNowNs` (rtc_base/types.hpp) 하나로 | 6벌 사본 중 하나가 이미 `rtc` 네임스페이스라 `redefinition` 으로 빌드가 깨졌다 |
| A-S5-15 | 센티넬: `input_age_s` 미수신은 음수, `q_cmd` 는 실제 나간 명령 (hold latch 포함), 명령 없는 tick 은 NaN | `input_age_s` 가 `now − 0` 이라 steady-clock uptime 을 나이로 실었다 (373966.66 s × 28476행); `q_cmd` 는 hold 중 0.0 이라 없는 수 rad 오차를 만들었다. 둘 다 그럴듯한 수라 소비자가 못 거른다 |

**S6 착수 시 확정 (2026-09-23 사용자 — #537 5785207660 → 5786035728)**

착수 대조 정정 19건 중 구현에 영향을 준 것: `plan_box_`·`PlannerRtState` 신설, `planner.wake_timeout_s` 가 주기 상한, 새 `Reason` 없이 token 불일치·나이 초과를 "plan 없음" 으로 읽음, 후보는 vision 격자 그대로 (공분산 보간 규칙 없음).

| ID | 결정 | 반영 |
|---|---|---|
| E-7 · J | mpc role 재사용, 이름 `mpc_main` (§6) | S6-A |
| B | profile 은 `mpc_on`/`mpc_off`, launch `enable_mpc` 하나 | S6-A |
| C | 판정 게이트 = 입력 유한성 · IK 수렴 · manipulability · `catch_box` + `p_stop`. 순위·진단 = 불확실성 · 도달시간 · γ 창 · commit 선행 · rollout · 오차 예산 (D-27) | S6-B |
| D | 점수 J + 순위 탈락 벌점 → 최소 J. 판정 통과 0 일 때만 plan 없음 | S6-B |
| E | `PlanSnapshot::reason` 은 plan 없음 사유 전용, 순위 탈락은 CSV 비트마스크 (msg D-20 동결) | S6-B |
| F | box 층은 순위 항, 계획 시점 토크 층은 `NOT_EVALUATED(runtime)`; 실행 시점 토크는 `dynamic` 이 강제 (S6-C2). 계획기 한계는 이후 envelope (D-S8-18) | S6-B · S6-C2 |
| G | `T_freeze` p1b 0.36 · leap 0.19 s (provisional) | S6-B |
| H | `planner.wake_timeout_s` 0.05 s | S6-A |
| I | `catch_box` = base 축정렬 상자; sim 은 S3.5b 열린 후보 외접 + 0.1 m, 실기 provisional | S6-B |
| K | `joint_cmd.accel_constraint: box\|kinematic\|dynamic` — dynamic 은 $M(q)(v-v_{prev})/\Delta t + h \le \eta_\tau\tau_{\max}$ ($\eta_\tau$ 0.8), 행은 hard (실패 → QP 비의존 abort), box 는 비트 동일. 구현 `73b8ca49`·`64dc47e1`. 출하: p1b `dynamic` (추종 p95 68 대 624 mm), leap `box` — 2026-10-02 에 leap 도 `dynamic` 으로 바꿨다 (MPC 계획 MD-74) | S6-C2 |
| L | `planner.wait_pose` (provisional) p1b `[0.212, −1.376, 1.107, −1.978, −3.296, 0.121]` · leap `[0, 1.0, 0, −1.2, 0, 1.2, 0]`, IK seed 전용 | S6-A·B |
| 3-1 | `reference.a_max` 21 / 35 채택 (provisional) | — |
| 3-2 | A-S5-12 채택 | S6-A |
| 3-3 | `track_err_abort` = 관측 피크의 2 배 이상 (provisional). 현재 p1b 0.42 (S8-B, τ 0.05 서보 피크 0.21 × 2; 그 전 값은 #566 이전·τ 0.2 서보 측정), leap 0.48 (S8-D) | S6-C → S8 |
| 3-4 | A-S5-13 채택 | — |
| R-1 | `mpc_main` 재사용 + WBC ↔ 포구 switch 테스트 (sim `SwitchController` 왕복 + `verify_rt_runtime.sh`) | S6-A |
| R-2 | `planner.budget_s` 0.020 + IK 후보 ≤ 8 사전 필터 + 연산시간 실측 (후보당 IK 6R p99 2197 · 7R p99 2415 µs, 2026-09-23 개발 PC) | S6-B·C |
| R-3 | 계획기 sub-model `urdf.sub_models.<arm>_catch` + `planner.sub_model` | S6-B |
| R-4 | 출하 p1b 는 oracle off · planner on (동시 on 은 park) | S6-B |

정한 것 (묻지 않음): `planner.slice.dt` 0.05 · `slice.t_lead_min` = `T_freeze` · `slice.t_max` = 지평 − `t_horizon_margin` · `sigma_trk`·`clock_err` 0 provisional · 탐색은 TRACKING·APPROACH 에서만 (COMMITTED 이후 `monitorOnly`) · RT 는 `plan_id` 변화 + token 일치 + 나이 ≤ `io.t_stale` 로 새 plan 수락 (L3 §5.2).

**repo drift (전부 닫힘, 2026-09-19)** — PR [#538](https://github.com/hyujun/rtc-framework/pull/538) (FingertipSensor 주석·controllers.md DemoWbc 행·p1b `index_mcp_aa_joint` 주석) · (E-9) MPC 경로 mutex·`fprintf` 를 RT 에 맞춤 (cross-mode swap 할당은 invariants.md 알려진 위반) · (PROC-7) DemoWbc Store 누락 수정 · timing CSV 8열 · E-3 의 "추가" 도 E-3.

**기존 미결정 (정리)** — D-13 확정 (2026-09-27, D-S9-A~L, §4.4 S9; 구현 S9a·S9b) · L3 가중치 S8-H 로 종결 · D-18 지도 초기 탐색 범위 (§11 제안: 방위 ±90°, 편차 ±10°) 는 S3.5a 로 수행 · 실기 공분산 검증은 위 열린 항목, D-7e·NLP 전환은 위 2026-09-29 결정.

### 7.4 정합화 개정 기록 (2026-09-19)

외부 리뷰 finding 16건과 1차 리뷰를 현재 코드·규약으로 재검증했다. 반증된 finding 은 없고, 처방을 바꾼 것은 "수정" 으로 적는다.

| finding | 판정 | 반영 |
|---|---|---|
| F1 E-3 누락 | 수정 — 규칙 모호, 선례가 `.msg` 발화·`.srv` 미발화로 갈림 | D-14·D-20, S0.8, §7.1 |
| F2 E-8 시점 | 확인 — 신규 컨트롤러 E-STOP hold 회귀를 E-8 로 분류한 선례 (`aebcd8e6`) | P-1, S5 게이트 |
| F3 DAG 역의존 6건 | 확인 (+ L3 "착수 전 S4" 문구, 미배정 L5.7·L5.8·L5.10·k 식별) | §4.2, S1.9·S2.3a/b·S3.5a/b·S4.0·S4.5·S3.1a/b·S3.8 |
| F4 ReadTraj race | 확인 | D-21 |
| F5 provenance | 확인 | D-22 |
| F6 quiescence | 수정 — activation generation 은 base 에 이미 있어 재사용 | D-23 |
| F7 TIP_STALE | 확인 — 센서 lane 에 수신 시각·sequence 없음 | D-24, S7 게이트 |
| F8 extra_frames | 확인 — rclcpp 파라미터는 list-of-dict 불가 | D-10, §10, S2.3a |
| F9 AC 누락 | 확인 | §4.4 게이트 표, §5, G8-D2 |
| F10 스케줄러 검증 | 수정 — 외부 검증기 `verify_rt_runtime.sh` 가 이미 있음. `all_configs`·absent-role 필터 추가 발견 | §6, §7.2 |
| F11 PROC-7 | 확인 (+ DemoWbc 선례 구멍) | D-20, S5.4, §7.3 |
| F12 손 토크 | 수정 — 설정·모델값은 3.0 N·m 로 일치, L7 의 1.5 N·m 는 G6-3 오인용 | §7.3 (D-12), L7 |
| F13 QoS | 확인 | D-4, S5.2 |
| F14 수치 | 수정 — NUM-7 이 지배, SVD 는 FIFO planner 에서 고정 크기만 | §11, S1.9 |
| F15 상태 drift | 확인 | 헤더·§4.3·§7.3 |
| F16 문서 결함 | 확인 ((b) 는 12개로 정정) | L8·§4.3·D-19·S3.4/S5.2 |
| D-2/E-1 | 예외 필요 — 근거는 staleness 가 아니라 deadline 파생 | D-2, §3.1, S0.6 |
| 1차 리뷰: 비-RT writer SeqLock 무한 재시도 | 반증 (blocker 아님) — backend 3종이 관절 상태에 이미 쓰는 경로 | D-21 은 G1-C 측정으로만 |

## 8. 계획기 NLP 전환 대비 (A-4)

1차원 시간 탐색 + IK 로 시작하지만, 문제가 복잡해지면 MPC 처럼 NLP 로 바꿀 수 있어야 한다.

- **경계:** 계획기 코어는 "입력 스냅샷(궤적 + 공분산 + 로봇 상태) → `PlanSnapshot`" 단일 진입 함수로 둔다. 스레드(§6), 입출력 SeqLock, RT 쪽 소비(L4·L7), 게이트 순서의 앞단(불확실성·도달시간 사전 필터)은 탐색 전략과 독립으로 설계한다
- **추상 interface 는 지금 만들지 않는다.** 구현이 하나뿐인 abstract interface 는 ARCH-3 위반이다. NLP 구현이 실제로 생기는 시점에 두 구현을 두고 interface 를 도입한다
- **스케줄링과의 관계:** NLP solver 가 할당·예외를 쓰면 RT-1~10 을 지킬 수 없으므로, 그때 D-7a 는 `thread_layout.yaml` 값만 바꿔 SCHED_OTHER 로 간다 (§7.2 결정 방식 1)
- **전환 판단 신호 (S6·S8 기록):** IK 수렴률 (L3 G3-G), 계획 성공률, 예산 초과율, 1차원 분해가 놓치는 후보(시각·자세 결합) 사례
- 전환 시 재사용 후보: `rtc_mpc` 의 solver 기반·스레드 관용구 (그 시점에 조사)
- **전환 여부 (사용자 2026-09-29): v1 은 전환하지 않는다.** S8-E 에 전환을 시사하는 신호가 없었다 (§7.3). MPC 도입은 검토 중이고, 재검토 조건은 S10 실기에서 IK 수렴률 저하·예산 초과가 관측될 때다
- **개정 (사용자 2026-09-29, [MPC_DUALARM_PLAN.md](MPC_DUALARM_PLAN.md) MD-6):** MPC 도입을 별도 계획으로 착수한다 — 재검토 조건을 기다리지 않는다. 동기는 v1 의 신호가 아니라 waist + dual-arm 확장이다. v1 계획기·DECEL 은 기본값으로 남는다. 단일 팔 mpc planner 가 v1 과 비슷한 성능을 내는지 (G-1) 는 기본값 전환만 판정하고, dual-arm 확장의 착수 조건은 아니다 (MD-47). 단일 팔의 mpc planner 는 v1 의 탐색을 그대로 쓰므로 두 번째 계획기 구현이 아니다 (MD-46). 위 경계·interface·스케줄링 규칙은 그대로 적용한다

## 9. 관절 가속 한계 도출 (D-16)

가속 데이터는 없고 토크 한계는 있다: YAML `devices.<group>.joint_limits.max_torque`, URDF `effort`, MJCF `forcerange` 가 같은 값이다 (UR5e 150·150·150·28·28·28 N·m, iiwa7 200 N·m).

**방법 (오프라인 도구 `derive_accel_limits`, S2.5).**

1. 포구 작업공간·대기 자세 주변에서 관절 자세 q 와 속도 q̇ 를 표본 추출한다. q̇ 는 계획이 실제로 쓰는 속도 집합 (CLIK 속도 한계 이내) 으로 제한한다
2. 각 표본에서 Pinocchio 로 M(q), 중력 g(q), 속도항 c(q, q̇) 를 구한다 — 손을 포함한 결합 모델 (nv 는 §2)
3. 중력은 별도 헤드룸으로 뺀다: 관절 i 의 동적 토크 여유 τ_dyn,i = η_τ · τ_max,i − |g_i(q)| − |c_i(q, q̇)|. 모든 |q̈_j| ≤ a_j 에 대해 Σ_j |M_ij| a_j ≤ τ_dyn,i 가 성립하는 충분조건을 세우고, a = s · w 로 두어 최대 s 를 구한다 (LP)
4. 관절 가중 w 의 기본값은 균일 (w_j = 1). 대안 (τ_max 비례, 관성 대각 비례) 을 쓸 때는 계획기 도달시간 식 (L3 §4.3) 이 쓰는 한계와 같은 벡터여야 한다
5. 전 표본의 최소값을 보수적 상수 box 로 채택하고, 표본 범위·η_τ·w·모델 버전·일자·**τ_dyn ≤ 0 인 표본 비율**·관절별 binding 제약을 provenance 로 YAML 에 기록한다
6. **퇴화 시 채택 금지.** τ_dyn ≤ 0 인 표본이 있거나 s 가 0 에 가까우면 자동 채택하지 않고 사용자 판단으로 넘긴다 (표본 범위 축소, η_τ 조정, 자세 의존 한계 검토)

- η_τ < 1 은 접촉 충격 (L7 충격량 예산) 과 모델 오차를 위한 여유다. **0.8 로 확정** (2026-09-20 사용자 결정, S2.5 결과는 §4.4 S2)
- **교차 검증 (sim) 은 iiwa7 로 한다** (`model_pairs.yaml` 게이트 안). ur5e_p1b 의 MJCF 는 게이트 밖 형제 저장소 사본이고 `armature`·`damping`·`frictionloss` 가 Pinocchio M·h 에 없어 도출값을 낙관적으로 확인해 줄 수 있으므로, 모델 자기정합 오라클 (τ := RNEA 로 정답 생성) 로 검증한다
- **실기 주의:** UR 은 position 명령을 받는 쪽 컨트롤러가 자체 가속·보호 정지 기준을 가질 수 있다 (repo 밖, 미확인). S10 에서 식별한다

**발견: box 는 약 50 배 · 10 배 보수적이다 (S4.4, 2026-09-22).** 설계 때 본 충분조건의 보수성 "2~4 배" 는 틀렸다. 포구 자세에서 공 진행 방향으로 낼 수 있는 가속을 토크 한계로 직접 풀면 (`catch_speed_budget`, $|M\ddot q+h|\le\eta_\tau\tau_{\max}$, 접근축 유지, 회전자 관성 포함) 중앙값 `ur5e_p1b` 43 · `iiwa7_leap` 79 m/s² 인데 box 는 같은 자세에서 0.74 · 6.8 m/s² 다 (회전자 관성 10 배 가정에서도 21–23 · 35–36). 이 box 로는 `ur5e_p1b` 의 γ 창이 어떤 투척에서도 열리지 않는다 (§4.4 S4.4 결과).

- 출처는 표본 속도 범위가 아니라 (q̇ = 0 이어도 `ur5e_p1b` 2.03 → 5.33 rad/s²) **최악 부호 충분조건** (box 표현에는 정확한 조건이라 조일 수 없다) 과 **자세와 무관한 단일 상수** (`shoulder_lift` 가 표본의 93 % 에서 구속) 다. 표본을 수락 q\* 범위로 좁혀도 `ur5e_p1b` **2.03 그대로**, `iiwa7_leap` 9.26–9.91 (출하 9.20)
- 한계: 관절 속도를 정격 (3.1416 rad/s) 으로 두면 `ur5e_p1b` box 가 퇴화한다 (s\* = −0.82) — "정격 속도로 계획" 과 이 도출법은 양립하지 않는다. 가중을 최적화한 상수 box 의 최선도 `ur5e_p1b` 5.1 · `iiwa7_leap` 15.9 m/s² (방향 가속 중앙값) 로 부족·경계선이다
- 반대 방향의 결함: Pinocchio $M$ 에 회전자 반사관성이 없어 (URDF 에 없다) 그 항만 보면 box 는 낙관적이다 — MJCF `armature` (UR5e 0.1, iiwa7 0.15–0.25 kg·m²) 를 인자로 넣어 도출한다

**D-16 개정 (2026-09-22 확정, §7.3 결정 B).** 계획기 (L3 §4.3 도달시간·§4.8 rollout) 는 후보마다 토크 한계를 직접 검사하는 **자세·방향 의존 한계**를 쓰고, CLIK 에 넘기는 box 는 가중 벡터 (`qdd_max`) 로 둔다. 회전자 관성은 `derive_accel_limits` 에 인자로 넣는다. S3.5b 는 두 층을 병기한다 (`catch_gate_map` 의 `box`·`torque` 층).

**상수 box 의 원래 전제 — "계획기 도달시간과 CLIK 가속 box 가 같은 한계를 써야 계획이 실행과 일치한다" (자세 의존 한계는 v1 범위 밖, RT 매 tick 토크 여유 감시 용도로만 검토) — 는 더 이상 성립하지 않는다 (D-S8-18, 2026-09-27).** 결정 K (S6-C2) 로 CLIK 은 `accel_constraint: dynamic` (토크) 을 실행하고 box 는 계획기 도달시간 (L3 §4.3) 에만 남았다.

- **S8-G 사전 분석 (2026-09-26, #537 5846659919 — S8-F-1 9 unit·1068 발, `rtc_tools catching_arm_budget`)**: 실행된 관절 가속은 활성 tick p95 14–37 rad/s² (관절별; max 76–250) 로 box 를 tick 의 99.8 % 에서 넘고, 같은 운동의 도달시간이 box 로는 0.60–0.89 s (가용 lead 0.29–0.52 s) 라 `rank_reach` 93–100 % 실패, envelope 로는 0.15–0.25 s 로 96 % 통과. 토크 사용률 max 0.61–0.83 (forcerange 도달 tick 0). box 는 실행을 묶지 않으면서 순위 gate 와 `w_t` 항만 무의미하게 한다
- **S8-G 실험 (AC-3)**: sim overlay `s8g_w10_a21_envbox` 로 `robot.arm.accel_limits_path` 를 `derived_accel_limits_s8g_envelope.yaml` (`--write-envelope-box`: unit 별 p95 의 관절별 max = [20.3, 30.9, 37.0, 21.5, 14.2, 29.0] rad/s², `provisional`, sim 전용) 로 돌려 같은 56 발을 비교했다
- **원칙 채택 (2026-09-27 D-S8-18)**: 계획기 도달시간 한계 = 실행이 실제로 내는 envelope. sim 은 `sim.yaml` 의 `robot.arm.accel_limits_path` override 로 envelope 파일을 쓰고 (`rank_reach` 95 → 14 %), `robot.yaml`·출하 `derived_accel_limits.yaml` 은 토크 도출 box 를 유지한다 (실기 envelope 은 S10 측정 뒤) — *E1-F11 주석: 파일 · 경로 키는 없어졌고, sim 은 `sim.yaml` 이 `robot.arm.qdd_max` · `qdd_provisional` 을 덮는다.*

## 10. catch frame YAML (D-17)

catch frame 은 모델 빌더가 추가하는 frame 이고 (D-10), 사용자가 sim 에서 확인하며 바꿀 수 있게 로봇 config 에 연다. `urdf.*` 는 rclcpp 파라미터라 list-of-dict 를 못 담으므로 `urdf.sub_models.<name>.*` 와 같은 **map key** 형태다.

```yaml
# 로봇 config (예: integrated_bringup ur5e_p1b _base.yaml 의 urdf 절) — 스키마
urdf:
  extra_frames:
    catch_frame:                 # map key = frame 이름
      parent: l_palm_link        # 부모 frame
      xyz: [0.0, 0.0, 0.0]       # m, 부모 frame 기준 — 포켓 중심 (스키마의 자리값. 출하값은 아래 "초기 제안값 산출")
      rpy: [0.0, 0.0, 0.0]       # rad, 부모 frame 기준 — 결과 frame 의 +z 가 손바닥 바깥 법선
      provisional: true          # 사용자 확인 전
```

포구 컨트롤러 YAML 은 frame 이름만 참조한다 (`catch_frame: catch_frame`). 접근축은 규약상 이 frame 의 +z 다.

**전달 경로 (S2.3a).** CM 파서 (`list_parameters({"urdf.extra_frames"})`, `ParseSubModels` 와 같은 방식) → `rtc_urdf_bridge::ModelConfig` (yaml-cpp `LoadModelConfig` 도 같은 키를 읽거나 명시적으로 거부) → `PinocchioModelBuilder` 가 `BuildFullModel()` 직후 full 모델에 추가. sub·tree·actuated 모델은 `buildReducedModel` 로 상속하고 (부모 관절이 잠기면 조상 관절에 붙는다) 네 모델 모두에서 존재·위치를 검증한다 (S2 게이트). 모델 빌드 시 읽히므로 값을 바꾸면 컨트롤러를 다시 configure 해야 한다.

**초기 제안값 산출.**

- 축 (S2.3a): FK 로 도출 — p1b `l_palm_link` +z (rpy 0), iiwa7_leap `palm_lower` −z (x 축 π 회전으로 +z 로 뒤집음)
- 위치 (S2.3b, 2026-09-21 확정): **S4.5 의 실측 포구점** (L6 §4.5 표) 을 부모 frame 으로 옮긴 값 — p1b `[0.015, 0.145, 0.052]`, iiwa7_leap `[-0.035, -0.015, -0.069]`
- **`provisional: false` 전환 완료 (2026-09-21)** — 두 로봇 모두 사용자가 렌더로 확인. `test_catch_frame_models` 의 tripwire 는 PROC-6 근거를 달아 `EXPECT_TRUE(provisional)` → `EXPECT_FALSE` 로 바꿨다 ("확인된 상태가 출하된다" 를 고정)
- 검증기는 `provisional: true` 인 catch frame 으로 실기 arm 을 막는다 (D-12 와 같은 규칙)

**"preshape 손끝 중심" (v0.x 의 위치 정의) 은 기각했다 (2026-09-21)** — S4.5 가 포구점을 실제로 잰 뒤로는 쓸 이유가 없다.

- **게이트가 실측점 중심이다** — $r_{cap}$ (L3 §4.6 게이트 우변) 은 포구점 중심의 **측면** 허용량이라 원점을 옮기면 게이트가 p1b 20.9 mm, iiwa7_leap 81.7 mm 어긋난다
- **$t_c$ 의 의미와 충돌한다** — $t_{cmd} = t_c - T_{close,e2e}$ (L6 §4.3) 라 $t_c$ 는 손이 $\eta$ 까지 닫힌 시각이고 원점은 그때 공의 자리여야 한다. 손끝 중심은 포켓 **입구**로 접근축 방향 약 $d_{eff}$ 떨어져 있어 (iiwa7_leap: 포구점 catch z 0.069 + $d_{eff}$ 0.080 = 0.149 ≈ 손끝 평면 0.146), γ 창에서 $d_{eff}$ 를 두 번 세게 된다
- **정의가 값을 못 정한다** — "손가락 끝" body 가 여럿이다 (iiwa7_leap `*_tip_head` 중심 catch z 0.146 대 `fingertip`/`*_tip_link` 0.0969, 같은 자세에서 **51.2 mm** 차이)
- 손끝 중심은 **교차 확인**으로만 쓴다 (포구점과 같은 +z 쪽, 차이가 $d_{eff}$ 자릿수 — 둘 다 성립)

**frame 규약 검증 (S2.3b 게이트, 2026-09-21).** 실측은 MuJoCo palm **body** frame, YAML 은 URDF parent **link** frame 이고 두 관례는 실제로 갈리므로 (§11 의 `upper_arm_link`; `wrist_3_link` body 원점은 올바른 변환에서도 **100 mm** 어긋난다) 같은 q 에서 두 엔진 FK 를 대조했다. 무작위 팔 자세 16 세트, 잔차 분해는 $L = T_{mj}T_{pin}^{-1}$ (palm 이 일치할 때만 상수) 와 $R = T_{pin}^{-1}T_{mj}$ (base 가 일치할 때만 상수).

| 로봇 | palm 방향 잔차 | catch frame 원점 왕복 오차 | 판정 |
|---|---|---|---|
| `iiwa7_leap` | 7.7e-5 ° | **1e-7 m** | PASS |
| `ur5e_p1b` | 3.4e-6 ° | **1.4 mm** (평균 1.15 mm) | PASS — 잔차는 $\varepsilon_{model,p1b}$ (§11), frame 불일치 아님 |

- `l_palm_link`·`tool0` 는 두 모델에서 일치하므로 catch frame 의 앵커로 유효하다. 이름이 같은 다른 link 는 그렇지 않다 — 앵커는 실제로 쓰는 frame 으로 확인한다
- `ur5e_p1b` 의 1.4 mm 는 §11 의 8.3e-4 m 와 같은 항 (MJCF↔URDF 치수 차이) 을 다른 지점·통계로 본 값이다. sim 으로 줄일 수 없으므로 **다른 오차 항과 합치지 않는다** (§11 사용자 결정 2026-09-20)
- 재현: `verify_catch_frame.py` (private plan 쪽 도구 — repo 에 커밋하지 않는다). 방법은 위 두 잔차 분해와 왕복 대조이고, 그것이 이 문서가 갖는 SSoT 다

## 11. 투척 목표와 catchability 판정 (D-18)

**판정.** 공 궤적의 포구 후보점 p_c 마다 L3 §4.2 의 포구 자세(catch frame +z = a_d = −v̂(t_c), 즉 손바닥 바깥 법선이 날아오는 공을 마주봄)를 IK 로 구하고, 그 해 q* 에서 manipulability w(q*) 를 잰다. w ≥ `planner.catchability.manipulability_min` 인 후보가 하나라도 있으면 잡을 수 있는 공이다. 이 판정은 기존 게이트(IK 수렴, 도달시간, γ 창, 정지거리)에 **추가되는 AND 조건**이다 — manipulability 만으로 시간 안에 도달할 수 있다는 보장은 없다. 그래서 지도는 kinematic 지도(S3.5a)와 전체 게이트 지도(S3.5b)를 따로 낸다.

**같은 판정을 두 곳에서 쓴다.** (1) 오프라인 catchability 지도 (S3.5a/b) 가 발사 조건을 정하고, (2) 런타임 계획기 (S6.2) 가 후보를 거른다. 둘은 S1.9 의 같은 함수·같은 seed·같은 YAML 키를 써야 지도와 실제 판정이 어긋나지 않는다.

**fail-closed 수치 규칙 (NUM-7, NUM-1).** 전문은 L3 §4.2 ("입력 방어"·"fail-closed 수치 규칙") 가 갖는다. 요지: a_d = −v/‖v‖ 는 `‖v‖ ≥ v_eps` 와 `std::isfinite(‖v‖)` 를 **둘 다** 검사하고 실패는 사유 코드로 탈락시킨다 (clamp 로 덮지 않는다). w₅·w₆ 는 고정 크기 분해로 계산하고 (동적 크기 `JacobiSVD<MatrixXd>` 는 할당 — RT-1), 판정은 `det > 0` 이 아니라 모든 중간값 `isfinite` 와 `w ≥ threshold` 다. damped 인 `ClikReferenceGenerator::Manipulability` 는 게이트로 재사용하지 않는다.

**manipulability 정의 (확정).**

- catch frame 의 **팔 관절 열**만 쓴다 (손 관절은 손바닥 frame 에 영향이 없다 — 손바닥은 폐쇄 체인 상류, §2). 행은 포구 과제와 같은 **5행** — 병진 3 (LOCAL_WORLD_ALIGNED) + 접근축 2 (LOCAL x·y 각속도) 이고, 포구에 무관한 손바닥 법선 둘레 roll 은 뺀다. w₅ = √det(J₅ J₅ᵀ). 기존 CLIK 진단값 (6×6 damped) 은 roll·damping 때문에 같은 값이 아니다
- **w₅/w₆ 병행 (C-3).** IK 와 게이트는 w₅ 로 푼다. 검증용 w₆ = √det(J₆ J₆ᵀ) (팔 열 6×6, roll 포함, damping 없음):
  - 지도 도구(S3.5a/b)와 런타임 계획기(S6.2)는 매 후보에서 **w₅ 와 w₆ 를 모두 계산·기록**한다 (CSV·`PlanSnapshot`)
  - 게이트 정의는 YAML `definition` (`arm_5row` 기본, `arm_6row` 선택) 으로 바꿀 수 있고, **threshold 는 정의별로 따로 둔다** — 단위·차원이 달라 같은 수치를 쓸 수 없다
  - **상승 대상은 정의와 무관하게 항상 w₅ 다** (D-25). 그래서 q\* 가 `definition` 에 의존하지 않고, w₅·w₆ 는 **같은 자세에서 잰 두 값**이라 비교가 성립한다. roll 은 대기 자세 seed + w₅ 상승으로 결정적으로 정해지므로 (C-4) 지도와 런타임의 w₆ 도 같은 값이 된다. `arm_6row` 로 판정한다는 것은 직접 올리지 않은 값으로 게이트한다는 뜻이다
  - 어느 정의로 판정할지는 S3.5a/b 지도로 정했다 — **`arm_5row` 유지** (아래 지도 결과)
- m 와 rad 가 섞여 w 의 크기는 정의에 따라 달라진다. **threshold 는 위 정의에 대한 값**이고 (출하값은 로봇별 — 아래 YAML), 정의를 바꾸면 다시 맞춘다

**포구 자세의 여유 자유도.** 6축 UR5e 에서 5행 과제는 roll 1 자유도와 IK 해 가지가 남아 w 가 그 선택에 따라 달라진다. 그래서 **대기 자세(wait_pose)에서 시작하는 같은 IK** 로 정하고 남은 roll 은 **영공간 log w₅ 상승으로 쓴다** (D-25, 2026-09-20 번복). 상승은 seed 해 가지 안의 **국소 최대**라 결정성은 같은 seed·같은 키에 달려 있고, `planner.ik.k_manip` = 0 이면 번복 전 동작이다 (세부 L3 §4.2).

**frame 규약 (함정 주의).**

- "arm base frame" 은 ur5e_p1b `base` (URDF `base`), iiwa7_leap `link_0` 다 — 로봇 config 의 `urdf.sub_models.<arm>.root_link` 와 같은 값이고, 컨트롤러 config 의 CLIK `base_frame` (`controllers/demo_wbc_controller.yaml`·`controllers/mpc/*.yaml`) 이 이것을 이름으로 참조한다. **로봇 config 최상단에는 `base_frame` 키가 없다**
- ur5e_p1b 에서 URDF `base` 와 `base_link` 는 원점이 같고 z 축 둘레 180° 차이다. `base_link` 로 두면 +x 가 반대가 되어 공이 등 뒤에서 날아온다 — 그래도 그럴듯한 결과가 나오므로 조용히 틀린다. sim 에서는 MJCF 의 로봇 body 가 world 에 180° z 회전으로 놓여 있다
- 발사 높이 z 는 world 기준이고 거리는 base 기준이다. world ↔ base 변환은 가정하지 않고 **같은 q 에서 MuJoCo FK 와 Pinocchio FK 를 대조**해 S3.2 에서 확정했다 (아래). 아래 키는 이름에 프레임을 붙여 (`_base`·`_world`) 섞이지 않게 한다

**world ↔ base 대조 실측 (S3.2, 2026-09-20; ur5e_p1b 행 정정 2026-09-21 사용자 컨펌).** 두 엔진(MuJoCo `mj_forward` · Pinocchio `forwardKinematics`)을 **씬 파일** 위에서 같은 q 로 돌려 비교했다. 무작위 q 8 세트 (trial 0 은 q=0).

| 로봇 | base frame | **world_T_base (확정)** | 잔차 | 게이트 < 1e-6 m |
|---|---|---|---|---|
| `iiwa7_leap` | `link_0` | **항등** (p = 0, R = I) | **4.5e-16 m** (축선) | **PASS** |
| `ur5e_p1b` | `base` (URDF) | **항등** (p = 0, R = I) | **1.46e-3 m** (`l_palm_link`·`tool0` 원점, 무작위 자세 16 세트 최대. 관절 축선으로는 8.3e-4 m) | **FAIL** — 치수 차이, 아래 |

- ur5e_p1b 행은 처음 `Rz(180°)` 로 적혀 있었다. Pinocchio `Data::oMi` 는 URDF 모델 root (`base_link`) 기준이라 그 180° 는 world→`base_link` 였고, 180° 를 두 번 센 것이다. 정정 검정은 두 모델이 실제로 일치하는 frame (`tool0`·`l_palm_link`) 을 probe 로 썼다 — `wrist_3_link` 처럼 이름만 같은 link 는 올바른 변환에서도 body 원점이 100 mm 어긋나므로, 앵커는 **실제로 쓰는 frame** 으로 확인한다
- $\varepsilon_{model,p1b}$ 는 회전 라벨과 무관한 치수 차이 항이라 정정의 영향을 받지 않는다. catch frame 지점의 값은 **1.5 mm** 로 읽는다 (아래 사용자 결정)

**⚠️ 위 표만으로는 좌표를 넘길 수 없다 — 변환이 두 개다 (2026-09-21).** 위 표는 world ↔ **계획기가 이름 붙인 base frame** (`sub_models.<arm>.root_link`) 의 변환이다. 그런데 `CatchPoseIk::Solve` 와 `catch_pose_ik_batch` 는 `p_c`·`v` 를 **모델 world** (Pinocchio universe = URDF 모델 root) 로 받고, 이 둘이 같은 frame 이 아니다.

| 로봇 | MuJoCo world → `sub_models` base frame | MuJoCo world → **모델 root** (판정기 입력) |
|---|---|---|
| `iiwa7_leap` | 항등 (`link_0`) | **항등** — 모델 root 가 `link_0` 이다 |
| `ur5e_p1b` | **항등** (`base`) | **Rz(180°)** — 모델 root 는 `base_link` 이고 `base` = root·Rz(180°) 다 |

즉 `ur5e_p1b` 에서 **world 좌표를 판정기에 그대로 넣으면 x·y 가 뒤집혀** "공이 등 뒤에서 날아온다" 가 그대로 발생한다 (원점이 같고 z 둘레 180° 다른 frame 이 둘 있기 때문). 그래서 지도 도구는 두 변환을 **분리해서** 받는다: `base_T_world` 는 인자 (`--world-yaw-deg`·`--world-translation-m`), `model_world_T_base` 는 `--arm-base-frame` frame 의 배치를 **모델에서 읽어** 합성한다 (모델 root 에 강체가 아니면 거부). **도구 모듈에는 어떤 로봇 값도 박지 않는다** — 변환은 항상 인자이고 180° 는 테스트가 고정한다. 런타임 쪽의 같은 변환 (S6-C) 은 L1 §1 이 갖는다.

**영구 게이트로 만들지 않는다 (2026-09-20 사용자 결정, 권장안).** 게이트로 두려면 `ur5e_p1b` 에 0.83 mm 를 예외 허용치로 박아 알려진 실패를 정상으로 고정하고 MuJoCo 를 `integrated_bringup` test dep 으로 넣어야 한다. 대신 **새 로봇 프로파일이 추가되거나 `hand_description` MJCF 가 고쳐질 때** 축선 대조를 다시 돌린다 — 재현 경로는 `rtc_tools compare_mjcf_urdf` 에 관절 축선 FK 대조를 얹는 후속 작업 (P5). C++ 프로브는 보존하지 않았고 방법은 아래가 갖는다.

- **비교 대상은 body/link 원점이 아니라 관절 축선이다.** UR5e 는 MJCF body 와 URDF link 프레임 관례가 달라 (`upper_arm_link` 이 shoulder_offset 0.138 m 어긋난다) 원점 비교는 **파일 관례를 잰다** (기존 `compare_mjcf_urdf` 게이트가 축선을 쓰는 이유)
- ⚠️ **단일 링크의 "implied transform 이 상수" 는 증거가 못 된다.** `shoulder_link` 의 implied transform 은 8 세트에서 1e-17 로 상수지만, 두 모델의 차이가 pan 축(z) 둘레 회전 + z 방향 이동이면 q 와 무관하게 상수로 나온다 — 실제 차이가 그 형태였다. 축선 가설 검정 (모델 root 기준) 으로 갈랐다: `yaw 0°` → 1.66 m, `yaw 180°` → 8.3e-4 m
- ⚠️ **`ur5e_p1b` 의 FAIL 은 두 모델이 다르기 때문이다.** 축 방향은 8.5e-7 ° 로 일치하고 잔차는 **순수 치수 차이**다. shoulder_pan·shoulder_lift·elbow 는 **정확히 0** (1e-16) 이고 wrist 부터 벌어진다: shoulder 높이 URDF `0.1625` vs MJCF `0.163` → **0.5 mm**, wrist_1 URDF `0.3922` vs MJCF `0.392` → **0.2 mm**, wrist_2 누적 → 0.71 mm (최악 8.3e-4 m). 뿌리는 `ur5e_p1b` 의 MJCF 가 `hand_description` 패키지에 있는 **#392 수정 밖의 Menagerie 사본**이라는 것이다 (관성 불일치는 알려져 있었고, **운동학 불일치는 여기서 처음 측정됐다**)
  - ⇒ **`ur5e_p1b` 의 sim 포구점은 계통적으로 편향된다** — 축선 0.8 mm, catch frame 부모 (`l_palm_link`) 원점 **1.46e-3 m** (포구점에 걸리는 값). 공 반지름 33.5 mm 대비 작지만 sim 으로 없앨 수 없다
  - **사용자 결정 (2026-09-20): `hand_description` 을 고치지 않는다.** 2026-08-29 결정(별개 패키지)을 유지하고 이 항을 **sim 의 바닥값**으로 받는다. 대신 다음을 지킨다:
    - S3.5a/b 지도와 S8 오차 예산에서 `ε_model,p1b` = **1.5 mm** (catch frame 지점) 를 **분리된 계통 항**으로 센다 — 합쳐 평균내면 안 줄어드는 항이 줄어드는 것처럼 보인다
    - **sim 실측으로 이 항을 검증하지 않는다** — sim 이 편향의 출처다 (실기 S10 (#613) 에서만 갈린다)
    - `iiwa7_leap` 에는 이 항이 없다 (4.5e-16 m) — 두 로봇의 sim 포구 정확도 차이를 **로봇 차이로 읽지 않는다**
- **재현 방법**: 두 엔진을 직접 링크한 프로그램으로 관절 축선(`mjData::xanchor`/`xaxis` vs `Data::oMi`)을 비교한다. pinocchio 4.x 는 `-DNDEBUG` 와 `BOOST_MPL_LIMIT_{LIST,VECTOR}_SIZE=30` 없이는 컴파일되지 않는다 (repo 안에서는 `pinocchio::pinocchio` 타깃이 넣어 준다)

**YAML.** `planner.catchability` 는 출하 키다 (두 로봇의 `controllers/catching/search_grid.yaml`). `sim.throw_region` 은 제안 스키마로 남았다 (아래 ⚠️).

```yaml
planner:
  catchability:
    definition: "arm_5row"          # arm_5row (기본) | arm_6row — 게이트에 쓸 정의
    manipulability_min:             # 로봇별 (사용자 2026-09-21), provisional
      arm_5row: 0.1                 # ur5e_p1b 출하값. iiwa7_leap 출하값은 0.174
      arm_6row: "TBD"               # w₆ 로 판정할 때. 파서가 비유한으로 남겨 fail-closed
      provisional: true
sim:
  throw_region:                     # S3.5a/b 지도 도구·발사 설정 입력 (제안 — 키 미생성)
    base_frame: "base"              # 로봇 config 의 CLIK base_frame 과 일치해야 함 (검증기)
    distance_base_m: 4.0            # base frame 수평 거리 √(x²+y²)
    azimuth_base_rad: [TBD, TBD]    # 원호 위 발사 위치의 방위 (base +x = 0). 지도 결과로 채움
    z_world_m: [1.5, 2.0]           # 사람이 손으로 던지는 릴리스 높이 가정 (world)
    heading_offset_rad: [TBD, TBD]  # 수평 발사 방향 − (발사점 → aim_point) 방향. 지도 결과로 채움
    aim_point_base_m: [TBD, TBD, TBD]  # 겨냥점 (base frame). 초기 제안: 대기 자세의 catch frame 위치
    speed_m_s: [TBD, TBD]           # 지도 결과로 채움
    elevation_rad: [TBD, TBD]       # 수평면 기준 앙각. 지도 결과로 채움
    flight_time_s: [1.0, TBD]       # 발사 → 포구 비행시간. 하한 확정 (D-18), 상한은 지도 결과
```

- `manipulability_min`: 처음 제안은 두 로봇 공통 `arm_5row: 0.1` 이었고, 아래 지도 결과 ("0.1 은 `ur5e_p1b` 에서 한계선") 뒤 로봇별 문턱으로 대체됐다. 산정 근거는 출하 파일 주석이 갖는다 (각 팔의 task-feasible median w₅ 비율로 0.1 을 환산). 키 누락은 in-code 기본값 (임계 0.1, `k_manip` 0) 으로 경고 없이 해석된다 (§4.4 S3b·S4.4 절의 `iiwa7_leap` 설정 회귀)
- ⚠️ **2026-09-24 확인 (S8 준비 C-3)**: `sim.throw_region` 블록은 **YAML 키로 만들어지지 않았다** — 지도 도구 (`catch_gate_map`) 는 CLI 인자를 쓰고, 시행 러너는 동결 분포를 인자 (`catching_sim_trials --dist`, S8-A) 로 받아 `catchability_map.Throw`/`generate_throw_grid`/`throw_to_launch_request` 로 표본을 뽑는다 (D-S8-2, §4.4 S8). 위 키 이름은 지도 도구의 제안 출력이 그대로 쓴다

**지도 도구의 구성 (S3.5a, 2026-09-21).** 판정은 런타임과 같은 `CatchPoseIk::Solve` 여야 하고 (S1.9), 격자·비행·집계는 python 관행이다. 둘을 **C++ 배치 실행파일 + python 오케스트레이터**로 잇는다 (선례 `rtc_math/se3_error_compare`). pybind 는 기각했다 (선례 없음, venv `FindPython` 함정 — AGENTS.md §9.2, pinocchio 4.x 전용 플래그). python 재구현은 "같은 함수" 를 어긴다.

- **C++ 옵션 파서** `rtc::catching::ParseCatchPoseIkParams` — `planner.ik.*`·`planner.catchability.*` → `CatchPoseIkOptions` (S1.9 가 "No YAML parser yet (Q4)" 로 남긴 자리). S6.2 런타임이 이것을 그대로 쓴다 ("같은 YAML 키" 의 실체). `planner.ik` 아래 미지 키는 **거부**한다. `alpha_max` 는 L3 §6 (TBD) 과 구조체 기본값이 어긋나 **값을 코드 쪽으로 맞췄다** (L3 §6 = `0.26 (provisional)`; 닫는 근거는 θ 분포). 활성 `manipulability_min` 이 TBD 면 비유한으로 남겨 `kOptionsInvalid` 로 **fail-closed** 한다 (w₅ 의 0.1 은 차원이 다른 w₆ 게이트의 대체값이 될 수 없다, C-3)
- **배치 실행파일** `catch_pose_ik_batch` (`rtc_controllers`; ARCH-7 은 design-principles.md §"ARCH-7 의 범위" 의 **오프라인 검사 도구** 예외). 입력은 ModelConfig YAML·sub-model·catch frame·옵션 YAML·seed CSV·후보 CSV, 출력은 후보별 `reason`·`q*`·`w5`·`w6`·`iterations`·`sigma_min`·`qp_*` CSV. 테스트가 고정하는 두 계약: **열이 solver 의 double 을 bit-exact 로 싣는다** (G3-I 비교용), **후보 순서를 바꿔도 판정이 같다** (python 이 샤딩·재개한다). `--dump-frame` 은 catch frame 배치를 선언값과 대조한다 (S2.3b 검사)
- **python 순수 모듈** `rtc_tools.analysis.catchability_map` — 격자, 항력 비행 (고정 스텝 RK4, 적분 차수까지 테스트), base↔world 변환 (항상 인자), provenance
- **함정 두 개.** (1) 출하 `urdf.sub_models.<arm>` 은 flange (`tool0`/`ee_link`) 에서 끝나 **catch frame 이 그 sub-model 에 없다** — 지도는 arm root → catch frame 부모 link 까지의 sub-model 을 따로 선언해 쓴다 (손 관절은 `buildReducedModel` 이 잠그고, 손바닥은 루프 상류라 FK·Jacobian 은 정확하다). S6.2 런타임도 같은 모델이 필요하다. (2) `LoadModelConfig` 스키마는 출하 robot config 와 **다르다** (`urdf_path`, `sub_models` 가 sequence) — 번역은 python 쪽이 한다
- **항력.** sim 항력은 무차원 `Cd` **preset (constexpr, YAML 미노출)** 이고 스칼라 $k$ [1/m] 인 `sim.ball.drag_k` 는 TBD 다 (§7: preset 에서 환산해 쓸 수 없다). 그래서 지도는 **sim 의 힘 법칙 자체**를 적분하고 (ω = 0 이라 Magnus 소멸), ρ·$C_d$ 는 기본값 없는 필수 인자로 받아 출처를 provenance 에 남긴다. 환산값 $k = \rho C_d A/2m$ = **0.0205 1/m** 은 참고로만 기록한다 (문서 대표값 0.0229 와 12 % 차이)

**지도 결과 (S3.5a, 2026-09-21) `[PASS(provisional)]`.** 격자: 거리 4 m, 방위 6 × 60°, world z {1.5, 1.8, 2.0}, 방향 편차 {−10°, 0, +10°}, 속력 5.00–7.00 m/s (0.25 간격 9), 앙각 32–56° (4° 간격 7) = **투척 3402 개**. 비행은 sim 힘 법칙 RK4 (2 ms), 포구 후보는 T_f ∈ [1.0, 2.4] s 를 25 ms 로 훑고 도달권·최소 높이로 걸렀다. 판정은 `catch_pose_ik_batch`, seed 는 Pass A 에서 고른 상위 2 개. 투척 수는 로봇이 한 자세에서 기다리므로 **최선 seed 하나** 기준이고, 상자 수락률의 분모는 상자 안 **전체 격자 투척**이다 (코드리뷰 #556 이 두 seed 합집합·후보 0 투척 누락 집계를 도구에서 고쳤다).

| | `ur5e_p1b` | `iiwa7_leap` |
|---|---|---|
| 수락 투척 (**단일 대기 자세**, 최선 seed) | **1418 / 3402 (41.7 %)** | **1499 / 3402 (44.1 %)** — 문턱 0.1 일 때. 로봇별 문턱 0.174 로 확정된 뒤 출하 config 로는 **1158 / 3402** 이고, 이 열의 나머지 값도 문턱 0.1 기준이다 |
| 판정한 후보 / 수락 (그 seed) | 6888 / 4264 | 5298 / 4019 |
| 탈락 사유 (그 seed) | `below_manip_min` 2241, `not_converged` 383 | `not_converged` 683, `below_manip_min` 596 |
| w₅ (수락) | min 0.100 · p05 0.108 · **median 0.147** · max 0.208 | min 0.100 · p05 0.112 · **median 0.226** · max 0.273 |
| w₆ (수락) | min 0.0035 · median 0.047 · max 0.111 | min 0.0013 · median 0.109 · max 0.144 |
| θ (수락) | max 1.22e-2 rad = **0.047 × α_max** | max 2.99e-2 rad = **0.115 × α_max** |
| ε = 1.5 mm 경계 뒤집힘 (두 seed 의 수락 후보 합산) | **81 / 8446 (0.96 %)** | 23 / 8030 (0.29 %) |
| wait_pose 차점 seed 격차 | 1.55 %p | 0.25 %p |

- **방위는 구속하지 않는다** (두 팔 다 base z 둘레 대칭, Pass A 에서 12 방위 확인). 구속하는 것은 **속력 × 앙각의 결합**이고 수락 집합은 그 평면의 **능선**이라 축정렬 상자로 표현되지 않는다: 외접 상자는 격자 전체이고 그 안의 수락률은 41.7 / 44.1 % 다 (도구가 `box_accepted_fraction` 으로 같이 낸다)
- **4 m 지도의 `sim.throw_region` 제안 (provisional, S3.5b 가 대체 — 아래 γ 절)**: 거리 4 m, 방위 전 범위, world z 1.5–2.0 m, 방향 편차 ±10°, **앙각 44–56°**, **속력 5.0–6.5 m/s**. 상자 안 수락률 `ur5e_p1b` **70.2 %** · `iiwa7_leap` **76.9 %** (상자 안 격자 투척 1512 개 기준), 수락 투척의 75 / 78 % 를 덮는다
- **α_max 는 구속하지 않는다** — θ 가 콘의 12 % 를 넘은 적이 없다. L3 §6 의 provisional 0.26 rad 을 그대로 둔다 (닫는 근거가 이 분포다)
- **`arm_5row` 0.1 은 `ur5e_p1b` 에서 한계선이다.** 수락 w₅ 의 median 0.147, p05 0.108 로 문턱에 붙어 있고 `iiwa7_leap` 은 median 0.226 으로 두 배 여유다. 같은 숫자가 두 팔에서 다른 뜻이 되므로 문턱을 로봇별로 두기를 권했고, **로봇별 문턱으로 확정됐다** (`ur5e_p1b` 0.1 · `iiwa7_leap` 0.174, 위 YAML)
- **`arm_6row` 로 바꾸려면 문턱이 30배 작아야 한다.** 같은 수락 집합의 w₆ 하한이 0.0035 (`ur5e_p1b`) / 0.0013 (`iiwa7_leap`) 다. 6행 정의는 6축 팔의 손목 특이점에서 0 으로 내려가는데 그 자세가 포구에는 멀쩡하므로, **정의는 `arm_5row` 를 유지하고** `arm_6row` 는 기록만 한다 (C-3)
- **`planner.ik` 제안 (provisional)**: `k_manip` = **0.5**, `max_iter` = **40**. 나머지 (`sigma0`·`lambda_max`·`dq_step_max`·`manip_grad_tol`) 는 L3 §6 기본값에서 손댈 근거가 없었다
- ⚠️ **`k_manip = 0` 으로는 아무것도 통과하지 못한다.** 그럴듯한 대기 자세에서 w₅ 가 0.06–0.09 로 문턱 아래에 머물고, D-25 상승을 켜야 (k_manip 0.5) 0.10–0.21 로 올라온다. **게이트가 통과 가능한 것은 상승 때문이고**, "0.1 문턱 + 상승 꺼짐" 조합을 출하 config 에 남겨 두면 안 된다
- **`wait_pose` 제안 (provisional)**: `ur5e_p1b` `[0, −1.4, 0.9, −1.9, −1.5708, 0]`, `iiwa7_leap` `[0, 1.0, 0, −1.2, 0, 1.2, 0]`. **민감도는 낮다** — Pass A 에서 `iiwa7_leap` 5 개 seed 가 커버리지 27/56 으로 동일 (mean log w₅ 로만 갈림), `ur5e_p1b` 도 차점과 1.55 %p. 고정점 확인은 1 회로 끝났다
- **$\varepsilon_{model,p1b}$ 는 분리해 센다.** 1.5 mm 섭동으로 수락 후보의 **0.96 %** 가 뒤집힌다 (`below_manip_min` 99, `not_converged` 40). `iiwa7_leap` 의 0.29 % 는 이 항이 없는 로봇의 수치적 한계선일 뿐이므로 **두 값을 로봇 차이로 읽지 않는다** (위 사용자 결정 2026-09-20)
- ⚠️ **이것은 kinematic 지도다 — 손은 이 공을 못 잡는다.** 수락 후보의 포구 시점 속력은 두 팔 모두 **6.5–8.3 m/s** (median 7.35) 로 $\Vert v\Vert_{\max}$ 공식값 (`iiwa7_leap` 2.26 · `ur5e_p1b` 1.84 m/s) 의 **3.5–4.5 배**이고, P1b fly-in 실측은 그보다도 낮다 (§5.1 D-3 재판정, L6 §4.5). 그래서 **S4.4 가 거리(4 m)나 목표 속력을 되돌려야 했고** 도구는 거리를 인자로 받는다
- `not_converged` 가 758 / 1366 인 것은 max_iter 40 의 부족과 애초에 도달 불가한 후보가 섞여 있어 갈리지 않는다 — max_iter sweep 은 후속으로 둔다

**속력 격차를 어떻게 닫는가 `[사용자 결정 2026-09-21]`.** 투척을 좁히는 대신 **팔 궤적으로 공과 손의 상대속도를 줄이는** 방향으로 계획기를 갱신한다. 메커니즘은 이미 설계에 있다 — L3 §4.5 의 γ 창이 손 폐쇄 하한을 **상대속도 $(1-\gamma)\Vert v\Vert$** 로 쓰고 `SoftCatchTranslation` (L4) 이 그 프로파일을 실행한다. 지도가 공급할 것은 **요구되는 γ 값**이다.

$$\gamma_{\min}=1-\frac{d_{eff}}{\Vert v\Vert\,T_{close,tot}},\qquad \gamma_{\max}=\frac{\min(v_{dir,\max},\ \eta_vv_{\max})}{\Vert v\Vert}$$

| 로봇 | $d_{eff}$ / $T_{close,tot}$ | $\Vert v\Vert$ 6.5 → 8.3 m/s 에서 $\gamma_{\min}$ | 팔이 내야 하는 속력 $\gamma_{\min}\Vert v\Vert$ |
|---|---|---|---|
| `ur5e_p1b` | 0.095 m / 0.2815 s | 0.948 → 0.959 | **6.2 → 8.0 m/s** |
| `iiwa7_leap` | 0.080 m / 0.1047 s | 0.882 → 0.908 | **5.7 → 7.5 m/s** |

($T_{close,tot}=T_{close,e2e}+h/2$, L3 §4.11.) 팔 속도의 출처는 **관절 정격**이고 (사용자 결정 2026-09-21 — `reference.v_max` 는 URDF 에도 제조사 자료에도 없는 양이다), 그 아래에서 구속하는 것은 자세마다 다른 $v_{dir,\max}$ 다. 후보별 $v_{dir,\max}$ 는 S3.5b 가 아니라 **S4.4 가 냈다** (§4.4).

**S4.4 가 잰 것.** 수락 후보마다 $v_{dir,\max}$ (LP, 접근축 유지, 정격 관절 속도 × η_v 0.9) 와 토크 한계 방향 가속을 풀고, 창이 열리는 조건 $\Vert v\Vert+\text{margin}\le v_{arm}+v_{rel}$ 을 **낙차 × 상대속도 허용량**의 표로 냈다 ($v_{rel}$ = 손이 흡수하는 상대속도; 공식 $d_{eff}/T_{close,tot}$ = 0.34 / 0.76 m/s, 시각 발동 fly-in 실측은 두 손 모두 약 **1.0 m/s** — L6 §4.5). 셀은 kinematic 수락 후보 중 창이 열리는 비율, $v_{rel}$ = 1.0 m/s, 토크 한계 가속·stroke·시간 포함:

| 낙차 $\Delta z=z_c-z_0$ [m] | `ur5e_p1b` | `iiwa7_leap` | $T_f\ge0.5$ s 만 (`ur5e_p1b` / `iiwa7_leap`) |
|---|---|---|---|
| ≤ −0.5 (**현 sim 씬의 전부**) | **0 %** | **0 %** | 0 / 0 |
| −0.5 … −0.25 | 0 % | 0 % | 0 / 0 |
| −0.25 … 0 | 2 % | 13 % | 0 / 0 |
| 0 … +0.25 | 9 % | 23 % | 0 / 2 % |
| +0.25 … +0.7 | 14 % | 7 % | 5 % / 0 |

- **구속하는 것은 관절 속도 정격 × 낙차다.** $\gamma_{\max}$ 의 실제 값은 중앙값 $v_{dir,\max}$ 1.1–1.4 m/s 를 6.5–8.3 m/s 로 나눈 **0.13–0.22** 다 (필요한 $\gamma_{\min}$ 은 0.88–0.96). 가속은 구속하지 않는다 — 구속하는 것은 D-16 box 의 도출법이다 (§9)
- $\Delta z\le-0.5$ m 를 10–50 % 열려면 $v_{rel}$ 3–4 m/s 가 필요하다 ($d_{eff}$ 0.095 m 에서 $T_{close}$ 25–30 ms) — 이 손들의 자릿수가 아니다
- 그래서 **조건부 go** (2026-09-22 사용자 결정) 의 목표는 **낙차를 없애는 것**이다 (base 높이 부근에서 위로 던져 정점 근처에서 받는다). 조건 셋과 열리는 영역 (`iiwa7_leap` 39 · `ur5e_p1b` 11 / 3808 투척, base 1.0 m 가정 씬) 은 §4.4 S4.4 결과가 SSoT 다. 4 m 지도의 `sim.throw_region` 제안을 **S3.5b 가 대체했다**: `ur5e_p1b` 는 거리 0.9–1.0 m · 릴리스 0.15–0.25 m · 방향 편차 ±6° · 속력 4.65–4.85 m/s · 앙각 62–64° (provisional, 기준 투척 1.0 m · 0.2 m · 4.75 m/s · 60° 주변), `iiwa7_leap` 은 없음 — §4.4 S3.5b 결과

**발사 조건 공간.** 발사점은 방위 φ 와 높이 z, 발사 속도는 크기·앙각·수평 방향으로 정한다. 수평 방향은 "발사점 → 겨냥점(aim point) 방향 + 편차" 로 두어 좌우로 빗나가는 투척을 표현한다. 포구 후보는 비행시간 T_f ≥ 1.0 s 인 것만 남긴다 (D-18) — 항력 포함 T_f 1.0 s 에서 앙각 40–53°, v₀ 4.7–5.9 m/s, 정점 z 2.2–2.8 m 이고 T_f 가 길수록 모두 커지는 lob 이다 (S0.7 결과). 지도 도구의 초기 탐색 범위 (제안) 는 φ ∈ [−π/2, π/2] (정면 반원), 편차 ∈ [−10°, +10°] 이고, 잡을 수 있는 범위는 지도 결과로 정한다.

**지도 도구 출력 (S3.5a/b).** 격자별로 (a) 포구 가능 여부, (b) 최대 w 와 그 후보의 t_c·p_c·q*, (c) 탈락 사유(IK 실패·w 미달·비유한 입력·도달 불가·γ 창 없음)를 내고, 잡을 수 있는 범위를 `sim.throw_region` 의 방위·속도·앙각·방향 편차로 제안한다 (방위별 포구 가능 구간 그림 포함). 사용자가 q* 자세를 보고 threshold 를 갱신하면 지도를 다시 돌린다. 4 m 는 vision 지평 요구(S0.7)와 가속 box(S2.5)를 동시에 가장 어렵게 만드는 설정이라, 둘이 타협을 요구하면 되돌아오는 값이 거리다.

## 12. 알려진 위험

열린 위험을 먼저 두고, 닫힌 위험은 표 아래에 "위험 — 종결 (어디서, 어떻게)" 한 줄로 둔다.

| 위험 | 닫는 단계 |
|---|---|
| vision 토픽이 stable ABI 가 아니다 (D-4) | S5.2 레이아웃 해시 진단으로 감지, 제품 ABI 는 ball_perception |
| vision 지평이 짧으면 계획 가능한 포구 창이 준다. sim profile 은 S3.6 이 1.0 s / 20 점으로 설정했고 (D-15, 2026-09-22; S0.7 은 0.8 s) 지평 요구는 연속 재계획 기준 R1 이다. 가장자리 후보가 전멸하면 정지 출발(R2, 먼 q* 는 약 1.0 s)로 떨어진다. 지평을 늘릴 주체는 외부 repo 이고 요청 채널이 없다 (P-3). S3.6: gate 통과 창 끝은 0.73 s 지만 **기구학 reachable 창은 0.95 s** 이고 설정한 1.0 s / 20 점은 그 창 끝을 12 ms 여유로 덮는다 (stamp 수정 전의 이른 검출 — T_det p05 0.027 s — 기준으로는 **1.05 s / 21 점**이 필요했다, §4.4 S3.6). 지평이 모자라면 늦게 잡는 후보가 계획기에 도달하지 못한다 | S3.5a 가장자리 catchability, S8 첫 plan 측정, S3.6 |
| 실기 시계 오프셋이 미래 방향이면 지평 검사는 통과하고 t_c 만 늦어진다 (6 m/s·10 ms = 6 cm). L3 §4.6 의 ε_clk 는 분산 항이라 bias 를 모델링하지 않는다. sim 은 같은 호스트라 드러나지 않는다 | S10 (#613) (D-2 예외 ③ 조건, TBD-NET-01) |
| sim T_close 는 MJCF 게인에 의존 — 실기 측정 전까지 S4 결론은 잠정 | S4.3, S10 (#613) |
| γ derate 제외(D-8)로 abort 가 늘 수 있다 | S8 측정 (시행별 `ref_saturated` max streak, G8-C3) — **S8-E 기록 (2026-09-26)**, 판정 없음 (수치는 §4.4 S8 게이트 표 G8-C3) |
| sim 팔 actuator 는 1차 지연이다 (MJCF 게인 τ ≈ 200 ms → D-S8-13 으로 p1b sim 은 **τ 0.05 s**) — 선행 보상 (순수 지연) 은 비-순수지연분을 남기고, sim 에서 잰 lead 이득은 UR5e 이득을 예측하지 않는다 (L5 §4.4) | S8-B (G8-E 잔여 보고), S10 (#613) |
| sim 이 실시간보다 느려지면 (host 부하, RTF < 1) 공 stamp 가 벽시계보다 뒤처져 steady 나이 검사가 입력을 `BALL_STALE` 로 끊는다 — 성공률이 host 부하에 좌우된다 (S8-E 원 판정: tennis 부하 시행 5/28 성공). `sim_stall` 규칙은 sim 스텝 간격만 봐서 못 잡는다 | D-S8-17 (unit 재실행 규칙, 2026-09-26). unit 도중 부하 감시·중단은 **#601 에서 닫음** — `catching_sim_trials --host-watch abort` 가 투척마다 truth 행의 RTF (sim 0.25 s 창 최솟값 < 0.95) 를 판정해 런을 끝내고 (exit code 3), 같은 seed 로 unit 을 다시 돌린다 (사용법은 integrated_bringup README). 원인 쪽 (sim stamp 축) 은 그대로라 감시를 끈 실행은 여전히 host 에 좌우된다. 같은 원인으로 분석기의 `t_c` 분해 열·δ(t_c) 가 어긋난다 — **#602 에서 표시를 닫음**: 행마다 `tc_axis` (`ok`/`shifted`/`unknown`) 와 stamp 축 `t_c` 와의 차를 내고 `shifted` 행은 `t_c` 중앙값에서 뺀다. 분해를 stamp 축으로 다시 계산하지는 않는다 (#602 (b) 안 — 하지 않음) |
| sim 폐루프 재현 편차 — 부하가 없던 시행도 같은 투척에서 unit 당 ±5 발 달라진다 (S8-E D-S8-17 재실행, 원인 미확인 — 노드 간 메시지 타이밍 추정). G8-D 의 원 판정 83 FAIL 대 재실행 93 PASS (통과선 84) 는 부하 제거와 이 편차를 함께 담는다 | **미배정 (후속)** — #613 도 이것을 담당 없는 위험으로 적고 있다 |
| L3 §4.6 직교 분해의 가정 (A⊥B) 이 성립하지 않는다 (A·B 음의 상관, G8-C2 FAIL — profile 수정 뒤에도 부호가 남는다) — 어긋남의 방향은 보수적이다 | **#600 에서 닫음 (식 유지)** — 런타임 검사는 `σ_ℓ = σ_c` 대입이라 직교 가정을 쓰지 않는다. 재검토 조건 (γ 의존 예산을 판정에 쓸 때) 과 그때 먼저 볼 전제 (`σ_ℓ ≤ σ_c`) 는 L3 §4.6 "측정과 결정". 실기 짝은 G8-B3 (S10, #613) |
| iiwa7_leap 투척 상자의 공이 **상승 중 대기 손에 닿는다** — S8-D 유효성 검사 (D-S8-15) 가 손을 qpos0 로 두고 간격을 재어 `hand.q_pre` 의 손끝을 못 봤다. S8-E leap 200 발 중 손끝 접촉 44 발에 판정 없음·`TRACK_CHANGED` 반복이 전부 들어 있다 (pre-S10 R6). leap 의 G8-D·G8-D2 수치는 이 상자 위의 값이다 | **보정 기준 재판정으로 닫음** (사용자 2026-09-29, 수치는 §4.4 pre-S10 R6 결과). 상자·대기 자세는 그대로라 같은 상자로 다시 던지면 같은 접촉이 난다 — leap 을 다시 평가할 때는 clearance 검사의 손 자세부터 `q_pre` 로 |
| 토크에서 도출한 보수적 가속 box (D-16) 가 받을 수 있는 공 속력을 낮출 수 있다 | S4.4 |
| 1차원 분해가 복잡한 경우를 놓치면 NLP 전환 (§8) 이 필요해 S6 재작업 | S6·S8 신호 — S8-E (2026-09-26): 전환을 시사하는 신호 없음. v1 은 전환하지 않는다 (사용자 2026-09-29, §7.3·§8) |
| `shape_estimation` 이 `/system/estop_status` 를 transient_local 로 구독해 volatile 발행자와 연결되지 않았다 — E-STOP 에 탐색을 abort 하는 경로가 죽어 있다 | 구독 연결은 **pre-S10 R2 에서 닫음** (Q14). abort 경로는 여전히 도달 불가 — `/shape/explore` accept 핸들러가 자기 single-threaded executor 안에서 `switch_controller` 응답을 기다려 늘 timeout 나고 탐색이 시작되지 않는다 (issue 분리 안 함, 사용자 2026-09-29) |
| UR 벤더 position 컨트롤러 자체의 가감속·보호 정지 | S10 (#613) |
| 브랜치 장기 체류로 main 과 어긋난다 (#538 로 1회 발현, 2026-09-19 병합으로 해소) | 단계 착수 때마다 main 병합 |

**닫힌 위험.**

- C-1 전체 거부가 포구 직전 기아가 될 수 있다 (지평 끝이 지면·작업셀 밖에 닿을수록 무효 점이 생김) — 종결 (sim 실측 기준). 재검토 조건은 "부분 무효가 실제로 나오면" 이었고, S3.4 validity 히스토그램 (§4.4 S3a 표) 과 S3.6 profile run (부분 무효 0, §4.4 S3.6) 에서 발동하지 않아 S5.2 는 C-1 을 그대로 구현했다. 실기 vision 의 validity 는 측정되지 않았다
- T_freeze 0.52 (T_arm 0.2 파급, C-25) 가 S3.5b 목표 분포의 열린 후보를 지울 위험 — 종결 (S8-B 지도 재실행, 2026-09-24): 발현 (0/180) 뒤 D-S8-13 (sim τ 0.05, T_freeze 0.37) 으로 163/180
- D-3 이 검증에서 떨어지면 S3·S5 시간 경로 재작업 — 종결 (S3.1a, S3.1b 는 S8 시행 누적으로 충족 — S8-E, 2026-09-26; δ 는 공변량, D-S8-4 (c)). host 부하 (RTF < 1) 는 위 표의 행
- S2.2 CLIK 확장이 출하 컨트롤러 DemoWbc 를 회귀시킬 위험 — 종결 (S2 결과, 2026-09-20): 동등성 PASS (기존 `test_clik_reference`·DemoWbc assertion 무수정) · S2.4 GUI·plot PASS, DemoWbc TSID QP 미수렴은 회귀가 아니다 (§4.4 S2)
- w₅·w₆ 를 매 후보 계산하면 계획 예산을 먹는다 (L3 §4.8 은 coarse-to-fine 이 필요하다고 봤다) — 종결 (S6, 2026-09-23): R-2 사전 필터·연산시간 실측 (§7.3 R-2), S6-C rollout coarse-to-fine, G3-C PASS (§4.4 S6 게이트 표). 제어 PC 측정 (D-7a) 은 생략됐다
- D-13 을 S9 로 미뤄 S5~S7 의 abort·FAULT·손 유지 경로가 정책 확정 때 바뀔 위험 — 종결 (S9, 2026-09-27): S9a 현행 정책 고정 · S9b FAULT 확장, 기존 assertion 무수정
- E-STOP 중 살아 있는 device 가 서서히 밀린다 (CM 이 hold 를 매 tick 의 측정값으로 다시 만듦, sim thumb 약 3.4 mrad/s; 해제 검증 동안 출력이 새고 latch 뒤 구독자는 "NORMAL" 을 봄, #588) — 종결 (pre-S10 R2): slot 별 hold latch · 해제 검증 창 동안 hold 유지 · `estop_status` transient_local (§4.4 pre-S10). 실기 creep 크기는 S10 (#613)
- `derived_accel_limits` 가 `adopted` 만 검사해 `provisional: true` 가 실기에서도 통과하던 위험 (두 로봇 출하 파일 모두 provisional) — 종결 (pre-S10 R3, Q4): provisional 또는 키 부재는 sim 경고 · 실기 park. 실기 box 확인은 S10 (#613)
- `joint_cmd.lag` 의 `T_arm` 0 · `lead_enable` false 가 실기에서 경고 없이 통과하던 위험 (`T_arm` 미식별) — 종결 (pre-S10 R3, Q5): `joint_cmd.lag.provisional` (기본 true). 값은 S10 (#613)
- 대기 자세 채택·ARMED 판정·손 정착 판정이 속도 lane 판독 여부를 묻지 않아 읽을 수 없는 lane 의 0 을 정지로 보던 위험 — 종결 (pre-S10 R3, Q9·Q16): 세 판정 모두 `IsLaneReadable(kVelocity)` 를 먼저 묻는다 (비판독 = 채택 미룸/거부 · 대기 자세 아님 · 정착 아님)

## 13. GUI·plot 단계별 구현과 확인 (D-19)

단계마다 그 단계가 만든 상태·로그를 **`demo_controller_gui` 에서 보이게 하고, CSV 를 `plot_rtc_log` 로 그릴 수 있게** 한다. 둘 다 단계 게이트에 들어간다.

**면제.** S0 은 코드가 없다. S1 은 ROS·GUI 비의존 순수 코어이고 실행 산출물이 GTest 뿐이라 GUI 에 붙일 상태도, `plot_rtc_log` 가 읽을 CSV 도 만들지 않는다. S1 코드가 만드는 값은 S5 이후 CSV·상태 메시지로 처음 드러나며 그 단계의 행이 확인한다.

**기존 패턴 (따른다).**

- GUI (`integrated_bringup` demo_gui): 컨트롤러 목록은 `/rtc_cm/list_controllers` 로 자동 발견된다. 새 컨트롤러는 목표 형태·게인 스키마 (`GAIN_DEFS` 등, config 모듈) 와 상태 메시지 패널을 추가한다 — DemoWbc 는 `WbcState`, grasp 는 `GraspState` 를 구독한다. 회귀는 integrated_bringup 의 `test_demo_gui_*` 테스트
- plot (`rtc_tools` plotting): 새 CSV 는 파일명·컬럼으로 종류를 판별하고 (log_type 모듈), 전용 plotter 와 pipeline 등록을 더한다 — `wbc_diag`·`compliance_diag` 가 선례. 회귀는 rtc_tools 의 `test_plot_rtc_log.py`. 스레드 timing CSV 는 공통 8열 스키마라 timing plotter 를 재사용한다 (수신 → 게시 지연은 별도 이벤트 레코드, §7.2)
- GUI 육안 확인은 `verify` skill 의 sim 절차를 따른다

**단계별 항목.**

| 단계 | GUI · plot 항목 | 상태 |
|---|---|---|
| S2 | GUI 변경 없음 (DemoWbc 패널이 CLIK 옵션 off 에서 그대로). `wbc_diag` 플롯 회귀 | 완료 (S2.4 GUI·plot PASS, CSV 새 열 없음 — §4.4 S2) |
| S3 | Control 탭 ball 패널 (sim 공 발사 D-14 srv·발사 조건 입력·공 상태), `test_demo_gui_ball_launch.py`. clock 위상 오차 (δ·pause) `analyze_clock_phase --plot` (결과 §5.1). catchability 지도 그림은 지도 도구 자체 플롯 | 완료 (2026-09-20) |
| S4 | Control 탭 Hand Step 패널 (open·preshape·closed step + 라이브 ρ), `test_demo_gui_hand_step.py` 11 케이스. T_close 식별 CSV → `analyze_hand_close --plot` | 도구 완료 (2026-09-20), 실측 완료 (2026-09-21, §4.4 S4a) |
| S5 | Control 탭 Catching 패널 (`demo_gui/catching.py` — 모드·사유, 입력 lane, plan, 추종 오차·CLIK 상태, Arm/Disarm), `test_demo_gui_catching.py` 23 케이스. `catching_diag.csv` = `plot_rtc_log` 1급 log type (컬럼 지문 `track_err_rad`+`ref_gamma`, 4단 공유 x축 figure), `test_plot_rtc_log.py` 14 케이스 (C++ 헤더 ↔ python 컬럼 oracle 포함) | 완료 (2026-09-22) — 육안: sim 폐루프 plan·추종 표시, Disarm 이 컨트롤러에 반영 (`armed=false`) |
| S6 | plan 표시 (t_c·p_c·γ_f·w₅·w₆·탈락 사유·plan 나이). `planner_timing_log.csv` (timing plotter 재사용), plan 이벤트 CSV (후보별 게이트 결과, 수신 → 게시 지연) | S6-B 에서 구현 (계획기 이벤트 CSV·GUI, §4.4 S6) |
| S7 | 슈퍼바이저 모드·사유·결과, 손 위상, 접촉 센서·freshness 줄 (`test_demo_gui_catching.py`). `catching_diag` 모든 패널에 모드 전이선 + `catching_hand` figure + 시도별 판정 집계 (`test_plot_rtc_log.py` TestCatchingS7) | 구현 (2026-09-23), 육안 확인 사용자 완료 (§4.4 S7) |
| S8 | Catching 패널의 연속 투척 진행 카운트 (결과 엣지를 로컬로 셈 — `CatchingState` 는 S5 동결, 새 필드 없음). `rtc_tools` `catching_trials` (시행 요약 CSV → 로봇별 truth 기반 Wilson 성공률·무효 발사·ITT 하한, 소거실험 2×2, t_c 분해, D-3 공변량) + `catching_pool` (arm 별 unit 합산·Wilson·McNemar). NEES 는 ball_perception_sim `sim_capture_evaluate` (P5) | S8-A 완료. S8-E (2026-09-26, `79a025de`·`879957c0`): 무효 분류·ITT·`--floor`·G7-B3 회귀·`--eval-samples` (G8-B)·`--probe-dump` (G8-C2) — 산출은 요약 JSON·CSV, 새 plot·GUI 없음 |
| S9 | 헤더 공용 행 "Clear E-STOP" (2 단계 — 사유 조회 → 확인 뒤 해제) · "Reset fault", fault 원인 문구. `catching_diag` 전 패널에 `estop_active`·`fault_latched` 구간 음영, CSV `fault_cause`·`fault_reset_refused` + `rtc_tools` 요약 줄 | 완료 (S9a PR #589 · S9b PR #590; sim ③ 확인). pre-S10 R2 (PR #597): `/system/estop_status` 구독이 transient_local 이라 E-STOP 뒤에 떠도 현재 상태를 보인다 |
| S10 | 실기 모드 표시 (provisional 값 차단 상태 포함). 실기 세션 CSV 가 같은 plot 으로 그려지는지 | 미구현 — S10 (#613) |

단계 행에서 정한 규칙 (각 패널·plotter 가 지킨다):

- S3 ball 패널은 `/sim/launch_ball_at` 과 **같은 집합을 거부**하고 (필드 이름을 대며) 피드를 never/live/stale **셋**으로 구분한다 (앞의 둘은 화면에서 같아 보이면서 정반대를 뜻한다). ball_perception 없이도 "never received" 로 정직하게 동작한다
- S4 Hand Step 패널은 자세를 **컨트롤러의 읽기 전용 파라미터**에서 읽고, ρ 는 `rtc_tools.analysis.hand_close` 를 import 한다 (사본 금지 — 화면 값과 run·보고서 값이 갈리지 않게)
- S5 Catching 패널은 **관측된 무장과 요청된 무장을 따로 보여준다** (tick 이 E-STOP·fault 에서 latch 를 내리므로 파라미터 set 성공은 무장의 증거가 아니다). 거부 카운터는 0 이 아닌 것만 보인다. GUI roster 누락은 `extra_switchable_controllers` 로 닫았다 (목표 패널은 `NO_EXTERNAL_COMMAND_CONTROLLERS`)
- S5 `catching_diag` 플롯은 **무효 tick 을 NaN 으로 끊는다** — PROC-7 이 0 으로 지우므로 그대로 그리면 손이 매 tick 원점으로 간 것처럼 읽힌다

**게이트 공통.** 해당 단계의 (1) GUI 패널이 sim 에서 값을 표시하고 조작이 컨트롤러에 반영됨 (육안 확인 + `test_demo_gui_*` 추가), (2) 새 CSV 가 `plot_rtc_log` 로 파싱·플롯되고 `test_plot_rtc_log.py` 에 회귀 케이스가 있음.

**포구 상태 메시지 (D-20).** 결정·동결·Store 규칙은 §1 D-20 (E-11 · PROC-7), 구현은 S5.4 의 `rtc_msgs/CatchingState` 다. 필드는 모드·사유·입력 상태·token·plan·w₅/w₆·손 위상·센서 freshness·결과다.

## 14. 추적 매트릭스

### 14.1 규약 → 단계 게이트

| 규약 | 단계 | 게이트 항목 |
|---|---|---|
| E-1 | S0.6 → S1.3·S5.2 | §3.1 예외 승인 전 D-2 변환 함수 착수 금지 |
| E-3 | S0.8 → S3.2 (D-14)·S5.4 (D-20) | 승인 기록 없으면 `rtc_msgs` 변경 금지 |
| E-6·PROC-6 | S2·S8-C | 기존 assertion 수정 필요 시 즉시 중단. S8-C 의 ⑦ 채택 시 `test_catching_planner_search.cpp`·`test_catching_soft_catch.cpp` 의 미분 0 단언은 별도 spec 커밋 |
| E-7 | S6 착수 전 | §6 배치표 `[CONCERN]`, S6 스레드 게이트 |
| E-8 | S5 착수 전·S5 게이트·S9 | P-1 최소 계약, race oracle, `/security-review` 2회 (S5 최소 정책, S9 최종) |
| E-9 | §7.3 | MPC RT 분류 drift |
| E-11 | S5.4 | `PublishRole` 을 늘리지 않음 |
| PROC-3 | S3.2·S5.4·(D-24 (a) 시 S5.2e) | `rtc_msgs`·`rtc_base` 변경 후 전체 빌드·테스트 |
| PROC-7 | S5·S7 | early-return 분기별 Store 실패 경로 테스트 |
| ARCH-3 | S1.9·§8 | 구현 하나뿐인 interface 신설 없음 |
| ARCH-6 | S5.2 | 구독 `KEEP_LAST(1)` (hook Phase 0b), backlog latest-only 테스트 |
| RT-1~10 | S1·S5·S6·S7·S8-C | G0-B·G1-D·G2-E·G4-G·G5-C·G3-K·G7-D 할당 0 (S8-A·S8-B 는 RT tick 무변경 — `git diff main -- rtc_controllers integrated_bringup/src/controllers/catching/controller.cpp` 비어 있음) |
| NUM-1·NUM-7 | S1.9·S1.5 | a_d guard, w fail-closed, 한계 무효 flag |

### 14.2 설계 문서 구현 단위 → 단계

| 단위 | 단계 |
|---|---|
| L0 시간 타입·궤적 POD·검증기·fixture | S1.3·S1.2·S1.7·S1.6 |
| L0 공 항력 k 식별 | ~~S3.8~~ → **미배정** (2026-09-20 S3a 범위 밖, fixture 필요 시 S3.5a) |
| L1 궤적 타입·개수 검사·D-2 변환 | S1.2·S1.3 (변환은 S0.6 후) |
| L1 파서·OnCloud·물리 일관성·J/ν·RT 상태 POD | S5.2a~e |
| L2 궤적 타입·Check·Hermite·NowLead | S1.2a·S1.2b·S1.3 |
| L2 지평 감시·sim 재생 | S5 |
| L3.1·L3.3·L3.5 (도달 가능성·γ 창·오차 예산) | S1.5 |
| L3.2 포구 자세 IK·catchability 함수 | S1.9 (함수) → S6.2 (배선) |
| L3.4·L3.6·L3.7 (rollout·선택·스레드·D-7a) | S6.3·S6.4·S6.1·S6.5 |
| L4.1–L4.3·L4.5·L4.7 | S1.4 |
| L4.4 축 정렬 | S2.1 |
| L5.1–L5.5 CLIK 확장 | S2.2a·S2.2b·S2.4 |
| L5.6 바인딩·abort | S5.3 |
| L5.7 지연 식별 도구 | ~~S3.7~~ → **S10** (#613) (2026-09-20, L5.9 에 흡수) |
| L5.8 선행 보상 | S5.3 |
| L5.9 실기 식별 | S10 (#613) |
| L5.10 backend 왕복 | S5.3 |
| L6.1·L6.2·L6.5 | S4.1·S4.2·S4.3 |
| L6 d_eff·r_cap 산정 (§4.5) | S4.5 (완료 2026-09-20) |
| L6.3 go/no-go | S4.4 |
| L6.7·L6.6 | S7.1 |
| L7.1–L7.3 | S1.8 (L7.3 은 S7.3 에서 결합) |
| L7.5 | S7.2 |
| L7.6 충격량 예산 | S8 (G7-B3 와 함께 이월, #537 결정 2026-09-24) — G7-B3 상관은 S8-E 기록 (2026-09-26, §4.4 S8-E 결과), 예산 `TBD-IMP-01` 은 미확정 |
| L7.7 | S10 (#613) |
| L7.8 E-STOP 훅 | S5.1 (P-1) → S9 |
| L8.2 골격 | S4.0 → S5.1 |
| L8.3 기록·상태 publisher | S5.4 |
| L8.4 D-3·발사 srv·truth | S3.1a·S3.2·S3.3 (부하 재검증 S3.1b 는 S8 시행 누적 — S8-E 로 충족, 2026-09-26) |
| L8.5 vision 연결·지도·사양·NEES | S3.4·S3.5a/b·S3.6 |
| L8.6·L8.7 폐루프 평가 | S8 (S8-E 판정 2026-09-26 — §4.4 S8-E 결과) |
| L8.8 실기 | S10 (#613) |
