# dynamic_catching — 전체 구현 계획 (living document)

- 상태: **S0 완료** (2026-09-19) — S0.1~S0.9 게이트 PASS. S0.7 이 0.5 s profile 부족을 보고해 sim profile 지평을 0.8 s 로 정했고 (D-15), 지평 요구는 R1·대기 자세는 겨냥점 근처·`kCap` 40 으로 정했다 (§7.1). 남은 승인은 S5·S6 착수 전 E-8·E-7. 승인이 막는 단계는 승인 전에 착수하지 않는다 (§4.1)
- 최종 갱신: 2026-09-19 (S0.7·S0.9 결과 기록. 정합화 개정의 finding 재검증 결과는 §7.4)
- Epic: [#537](https://github.com/hyujun/rtc-framework/issues/537)
- 수명: 구현 완료 시 prune 한다. 이 문서는 **전체 계획과 결정의 SSoT** 이고, 단계별 상세 작업(sub-plan)은 각 에이전트의 private plan 에서 관리한다 ([AGENTS.md](../../AGENTS.md) §6.6).
- 저장 위치: [handoff.md](../../agent_docs/handoff.md) §5 는 plan 파일을 커밋하지 않는다. 이 문서는 같은 폴더의 설계 문서(v0.5)와 함께 리뷰되어야 하는 결정 로그라서 설계 문서와 같은 브랜치에 커밋한다 — 사용자 결정 P-2 (§7.1). cross-tool 인계면은 여전히 issue #537 이다.
- 갱신 규칙: 결정·단계 상태·게이트 결과가 바뀔 때마다 이 문서를 먼저 고친다. 이 문서가 구체화되면 같은 폴더의 설계 문서·참조 구현을 이 문서에 맞춰 갱신·동기화한다. 설계 문서(`CATCHING_MASTER.md`, `L0_core.md` … `L8_bringup.md`)와 충돌하면 **이 문서의 결정이 우선**한다.

## 1. 결정 로그

ID 는 한 번만 정의한다. 사용자 결정은 D-·C-·P- 로, 착수 전 합의한 세부 결정은 A- 로 적는다 (§7.1).

| ID | 결정 | 상태 | 근거 요약 |
|---|---|---|---|
| D-1 | 코드 배치: 수치 코어는 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`), 축 정렬 오차는 `rtc_math` se3, CLIK 확장은 `rtc_tsid`, 바인딩·YAML·launch·PointCloud2 파서는 `integrated_bringup` | **확정** | 코어가 robot-agnostic 판정([design-principles.md](../../agent_docs/design-principles.md))을 통과한다. `compliance`·`grasp`·`inference` 코어와 같은 선례. 새 패키지를 만들지 않는다 |
| D-2 | 시간 규약: (1) t_c·t_cmd 는 공의 **물리 시각** 단일 정의, 판정별 비교 대상 '지금' 고정(§3). (2) 내부 표현은 절대 steady ns, 상대시각은 수치 코어 경계에서만. (3) nrt 수신 시 `t_ref_steady = recv_steady − (recv_wall − stamp)` 를 1회 계산한다. `header.stamp` 는 **물리 샘플 시각 복원에만** 쓰고, freshness·stale 은 `recv_steady` 로만 판정한다 (§3.1) | **확정** — (3) 은 E-1 승인 (2026-09-19, S0.6), invariants.md 에 기록된 예외 | (3) 은 stale 판정에 stamp 를 쓰지 않지만, t_c·t_cmd 가 stamp 에서 파생되어 COMMITTED·CLOSING·DECEL 판정(deadline)이 wall clock 점프에 노출된다 — [invariants.md](../../agent_docs/invariants.md) §Clock 시간축 규칙의 기록된 예외로 허용한다. 기각한 대안: 보정항을 버리고 `t_ref_steady = recv_steady` 로 두면 예외는 불필요하지만 전송·추정 지연(수~수십 ms)이 T_arm·T_freeze 예산에 그대로 들어간다 |
| D-3 | sim 시간축: wall clock 유지 + 시행별 clock 오차 게이트 (§5) | **채택 — 판정식 재정의, 검증 후 재검토** | D-2 변환이 실기와 같은 경로로 동작. `/clock` 방식은 RT 루프에 sim 전용 시간 원천이 필요해 D-2 와 충돌 |
| D-4 | vision 입력: ball_perception 의 실제 PointCloud2 레이아웃 채택, 필드 이름으로 파싱, `generation`·`validity`·`snapshot_sequence` 사용, NaN 공분산 = 모름. 구독 QoS 는 `KEEP_LAST` depth **1** 고정 (ARCH-6), reliability 만 S3.4 에서 실측해 정한다 | **확정** | 실제 발행기 존재. 설계 문서 §5 의 372 B·`t`·`cov` 가정 폐기. depth 는 ARCH-6 의 강제 사항이라 TBD 대상이 아니다 |
| D-5 | CLIK: `rtc::tsid::ClikReferenceGenerator` 를 옵션(기본 off)으로 확장. 옵션을 넣기 전에 기존 동작 golden-vector 회귀를 먼저 만든다 (S2.2a) | **확정** | P5 일반화. off 시 기존 출력 bit-identical, 기존 assertion 수정 필요 시 즉시 E-6 |
| D-6 | CLIK 오차·J 를 명령값 q_c 에서 평가하는 옵션 추가 | **확정** | 측정 q 평가는 servo 지연을 루프에 품고 선행 보상과 이중 보상. 실추종은 `TRACK_ERR` 로 별도 감시, 재앵커 시점 규정 필수 |
| D-7 | 계획기 스레드는 **기존 MPC 스레드와 같은 생성 방식**으로 만들고 기능만 planner 로 한다 | **확정** — D-7b~d 확정, D-7a 는 RT 준수 코드 + 측정으로 확정 (§6, §7.2). 배치는 S6 착수 전 E-7 승인 | nrt_callback executor 는 단일 스레드라 계획 계산(수십 ms)을 올리면 궤적 수신·서비스가 막힌다 |
| D-8 | γ derate 는 v1 에서 제외. 실행 중 포화 → COMMITTED 전 RETREAT, 이후 ABORT_SAFE. S8 에서 포화 빈도 측정 후 재설계안 도입 여부 결정 | **확정** | 참조 구현 probe: Frozen 분기 γ_min 미보장, 완화 분기 무효, 기본 램프가 가속 피크를 키움, 램프가 t_c 초과 가능 |
| D-9 | `gammaWindow` 의 TCP 속도 = η_v · `reference.v_max` (0 < η_v ≤ 1). 마스터 §6 교차제약을 이 식으로 수정 | **확정** | 계획이 한계 끝을 쓰면 실행 중 예측 변화로 포화. D-8 로 derate 가 빠져 유일한 완충 |
| D-10 | catch frame: 후보 p1b `l_palm_link` +z, iiwa7_leap `palm_lower` −z (FK 도출, 확인 필요) + 포켓 중심 offset. `rtc_urdf_bridge` 모델 빌더가 로봇 config 의 `urdf.extra_frames.<name>` 을 **full 모델에** frame 으로 추가하고, 파생 모델(sub·tree·actuated)은 그것을 상속한다 (§10) | **확정** (값은 D-17) | CLIK 은 모델 frame id 만 받는다. binding 로컬 offset 은 CLIK 까지 못 간다. 파생 모델은 모두 full 모델에서 `buildReducedModel` 로 만들어지므로 한 곳에서 추가하면 된다 |
| D-11 | 손 명령 포트 추상화 폐기. 손은 `ControllerOutput` 의 손 device slot 에 직접 기록. T_link 분리 측정 대신 종단 간 T_close,tot 실측 | **확정** | P1b·LEAP 모두 이미 device group. `udp_hand_node` 는 명령 stamp 를 읽지 않는다 |
| D-12 | 사용자 제공 값: 공 사양, 실기 T_close,tot 측정 시점, 성공률 하한·시행 수, **P1b 손 관절 운용 토크 한계의 권위 출처** (§7.3). 투척 목표는 D-18, 관절 가속 한계는 D-16, catch frame 은 D-17 로 대체 | **방식 확정, 값 대기** | 추측 금지. 임시값은 YAML 에 provisional 표시, 값에 의존하는 게이트는 NOT_EVALUATED (§4.1) |
| D-13 | E-STOP·fault 정책 (E-8) | **보류 — 최종 정책은 S9** (§4). S5 의 최소 계약은 P-1 | 사용자 결정 |
| D-14 | 공 발사 API: (p0, v0, ω) 명시 srv 를 `rtc_msgs` 에 추가 (Adding a New Message Type, PROC-3) | **확정** — E-3 승인 (2026-09-19, S0.8) | 파라미터 설정 + Trigger 는 경합·재현성 약함. E-3 판단은 §7.1 |
| D-15 | vision 예측 사양(지평·간격·점 수·발행률)은 **포구 제어기가 요구 사양을 정하고**, sim 에서는 공 투척 설정과 ball_perception sim profile 을 그 요구에 맞춰 설정한다. 제어기는 수신 궤적의 지평이 요구보다 짧으면 계획 후보에서 제외·진단한다 | **확정** | ball_perception 은 sim 이 주는 위치로 미래 궤적을 만드는 노드이고 사용자가 직접 설정한다. 기존 예시 profile 은 지평 0.5 s, 간격 0.05 s, 최대 10 점, ≤ 30 Hz 였고, S0.7 결과로 **sim profile 목표를 지평 0.8 s, 간격 0.05 s, 17 점 (t = 0 포함), ≤ 30 Hz 로 정했다** (사용자 결정 2026-09-19). 최종 요구는 S3.6 이 목표 투척 분포에서 다시 산출한다 |
| D-16 | 관절 가속 한계는 **토크 한계에서 도출**한다 (§9). 시뮬레이션 추정은 교차 검증용. YAML 의 기존 `max_acceleration` 값은 쓰지 않는다 | **확정** — 도출 절차의 퇴화 분기·가중·오라클은 §9 | 가속 데이터 없음, 토크 데이터 있음. 기존 `max_acceleration` (5.0 rad/s²) 은 CM 이 읽기만 하고 어떤 컨트롤러도 쓰지 않는 placeholder |
| D-17 | catch frame 의 부모 frame·위치 offset·자세는 **YAML 로 열어 둔다**. 초기값은 S2.3a(축)·S2.3b(위치)에서 제안하고, 사용자가 sim 에서 확인해 실제 값으로 갱신한다 (§10). 값은 모델 빌드 시 읽히므로 바꾸면 컨트롤러를 다시 configure 해야 한다 | **확정** | 사용자 결정 |
| D-18 | 투척 목표는 **arm manipulability 기반 포구 가능성(catchability)** 으로 정한다. 발사 영역 (arm base frame 기준 수평 거리 √(x²+y²) = 4 m 의 원호 — 좌우로 흩어진 투척 포함, world z 1.5–2.0 m = 사람이 손으로 던지는 높이, **비행시간 T_f ≥ 1.0 s** — 사용자 결정 2026-09-19, S0.7 후) 에서 출발한 궤적 위 포구 후보마다, 손바닥 +z 가 공 진행 방향을 마주보는 자세(a_d = −v̂)의 IK 해에서 manipulability 를 재고, threshold 이상인 후보가 있으면 잡을 수 있는 공, 없으면 포기. 이 판정으로 투척 속도·각도 범위를 정한다. threshold 초기값 0.1 (provisional, 사용자가 sim 에서 자세를 보고 갱신) | **확정** — 정의 세부는 §11 | 사용자 결정 |
| D-19 | 단계마다 **`demo_controller_gui` 갱신과 `plot_rtc_log` 로 CSV 플롯을 구현·확인**한다. 각 단계 게이트에 GUI 확인과 plot 회귀 테스트를 포함한다. S0 (코드 없음)·S1 (ROS·GUI 비의존 순수 코어, 실행 산출물은 GTest 뿐) 은 면제한다 (§13) | **확정** | 사용자 결정. 면제 근거는 §13 |
| D-20 | 포구 상태는 `rtc_msgs` 에 **새 상태 메시지**를 추가해 GUI 로 보낸다 (`WbcState`·`GraspState` 선례, `PublishRole` 을 늘리지 않는 controller-owned `SeqLock<T>` 패턴). **S5 에서 S5~S9 필드 superset 을 한 번에 동결**하고 이후 단계는 값만 채운다. 모든 `Compute()` tick 에서 Store (PROC-7) | **확정** — E-3 승인 (2026-09-19, S0.8) | 사용자 결정. 필드를 단계마다 더하면 매번 `rtc_msgs` 변경·PROC-3 전체 빌드·테스트가 반복된다 |
| D-21 | SeqLock 소비 계약: RT 소비자는 매 tick `Load()` 를 무조건 한 번 하고, 새 스냅샷 여부는 **payload 안의** `snapshot_sequence`·provenance token (D-22) 으로 판정한다. `SeqLock::sequence()` 와 `Load()` 를 따로 읽어 짝짓지 않는다. 스냅샷 용량은 컴파일타임 상수 `kCap` (S1.2) | **확정** (§7.4 F4) | 두 호출 사이에 writer 가 끼면 옛 payload 와 새 sequence 가 짝지어져 최신 스냅샷을 놓친다. repo 의 다른 SeqLock 소비자는 모두 무조건 `Load()` 관용구를 쓴다. 비-RT writer → RT reader 는 backend 3종이 관절 상태에 이미 쓰는 경로이며, 최악 재시도 시간은 G1-C 로 측정한다 |
| D-22 | provenance token: 궤적 스냅샷·공분산 버퍼·`PlanSnapshot` 은 같은 identity `{activation_generation, generation, snapshot_sequence, traj_recv_ns}` 를 싣고, `PlanSnapshot` 은 여기에 계산 기준 `{rt_iteration, rt_state_ns}` 와 `publish_ns` 를 더한다. 계획기는 계산 시작과 게시 직전에 최신 token 을 다시 보고, 대체되었거나 짝이 안 맞는 결과(궤적 N ↔ 공분산 N−1 포함)는 버린다. RT 소비자는 token 의 activation·generation 일치, `snapshot_sequence` 단조, source 나이·state 나이 상한을 fail-closed 로 검사한다 | **확정** (§7.4 F5) | `PlanSnapshot` 에 출처 필드가 없고 궤적·공분산이 다른 버퍼로 가서 N/N−1 혼합을 막을 수단이 없었다. MPC 선례(`MPCSolution::timestamp_ns`)보다 넓은 이유: 포구는 절대 시각 판정이라 출처 나이가 곧 안전 조건이다 |
| D-23 | activation 경계: vision ingress 와 계획기는 base 의 `ActivationGeneration()` 을 스냅샷에 싣고, RT 소비는 `IsCurrentGeneration()` 이 아니면 무효로 본다. `PeriodicRtThread::Pause()` 는 진행 중 iteration 을 멈추지 않으므로 quiescence 를 기다리지 않고 generation 으로 판정한다. lifecycle·E-STOP 훅은 atomic 요청(또는 epoch)만 갱신하고, plan·궤적·공분산·손·FSM·타이머 무효화는 **RT tick 이 유일 writer** 로 수행한다 | **확정** (§7.4 F6) | lifecycle 은 publisher 만 게이트하므로 컨트롤러 소유 구독은 비활성 중에도 산다. base target mailbox 의 generation 게이트는 그 mailbox 에만 적용된다 (`rt_controller_interface.hpp`) |
| D-24 | 지문 센서 freshness: (a) **권장** `rtc_base` `DeviceState` 센서 lane 에 `recv_steady_ns`·`sequence`·`valid` 를 추가하고 backend 3종이 채워 `ControllerState` 로 전달 (PROC-3, P5 — 같은 gap 이 grasp 에도 있다), (b) 포구 컨트롤러 소유 mailbox 로 센서 토픽을 따로 구독 | **결정 대기** — S5 착수 전 (§7.3) | 현재 `last_state_ns_` 는 관절 상태 콜백에서만 갱신된다. 관절이 fresh 한 채 센서만 멈추면 옛 힘을 새 접촉으로 볼 수 있다 |

## 1a. Sprint Contract (A-1 승인, 2026-09-19)

Epic 기준 하나와, **각 단계 착수 시 그 단계의 `[SPRINT]` 기준**을 따로 확정한다 (단계 sub-plan 의 `## Spec`).

```
[SPRINT] 1) iiwa7_leap·ur5e_p1b MuJoCo 에서 ball_perception PointCloud2 예측만을 입력으로,
            동결한 목표 투척 분포(D-18 지도, S3.5b)에서 포구 성공률의 Wilson 95% 하한 ≥ floor(D-12),
            로봇별 게이트 (G8-D·G8-D2). 무효 시행을 포함한 전체 발사 수와 무효 사유를 함께 보고
         2) 전 RT 경로 할당 0·noexcept·RT-1~10 준수, 기존 컨트롤러 테스트 assertion 무수정 green
         3) 설계 문서 v0.5 가 코드와 일치하고, 이 문서에 단계별 게이트 결과가 기록됨
```

기준 1 은 D-12 의 floor·시행 수가 정해지기 전까지 **NOT_EVALUATED** 다. 필요한 시행 수는 S0.9 검정력 표로 먼저 정한다. 실기(S10)는 Epic 기준 밖이며 S10 착수 시 별도 기준을 세운다.

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

코드 대조 결과 (2026-09-19). 상세 기록은 `WORKSPACE_ANALYSIS.md` §3 기록 칸에 있다.

- 명령 경로: 컨트롤러는 `RTControllerInterface::Compute` 에서 `ControllerOutput` 을 채우고, CM 이 `DeviceBackend::WriteCommand` 로 보낸다. ros2_control 이 아니다. backend type: 실기 UR `ur_driver_native` (vendor `forward_position_controller` 토픽으로 발행), sim `mujoco_native`, P1b 손 `udp_hand_native`
- 없음이 확인된 것: UR 지연 보상, speed scaling 노출, `ApplySafetyLayer` 의 production 호출, 스트리밍 목표를 받는 컨트롤러, 독립 IK, Pinocchio offset frame 추가 기능, sim `/clock`, sim 공 접촉 truth 출력, sim 명령 지연 주입
- 시간: RT 의 `ControllerState::t_relative_s` 는 steady clock 기반, `ControllerState::dt` 는 항상 1/`control_rate` (sim lock-step 에서 실제 간격과 다를 수 있음). `header.stamp` staleness 판단 금지
- CLIK: `ClikReferenceGenerator` 는 pose 목표만, LWA 6행 고정, 측정 q 에서 e·J 평가, 위치∩속도 box, `max_iter` 20 은 내부 `QPSolverWrapper` 고정값. 가속 box·feedforward·마스크·상태 노출 없음. `PinocchioCache` Jacobian 은 `LOCAL_WORLD_ALIGNED` 고정. `Manipulability()` 는 damped 6×6 Gram 의 LDLT 곱이라 게이트로 재사용하지 않는다 (§11). production 소비자는 DemoWbc 하나
- 재사용 대상: `rtc::SeqLock`, `rtc::SpscQueue`, `rtc::compliance::DifferentialIk` (1-step damped pseudoinverse, 임의 행 수 m — 반복 루프는 S1.9 가 새로 쓴다), `rtc_math` se3 `log3`/`exp3`, `QPSolverWrapper`, base 의 `ActivationGeneration()`/`IsCurrentGeneration()`, `thread_layout.yaml` 의 `profiles:`, `repo_scripts/scripts/verify_rt_runtime.sh`
- lifecycle: `PeriodicRtThread::Pause()` 는 요청 플래그 store 뿐이고 pause 게이트는 루프 최상단이라 진행 중 iteration 은 끝까지 돈다. lifecycle 은 publisher 만 활성화·비활성화한다 (D-23)
- 모델: ur5e_p1b 결합 모델 nv 는 full tree **26** (UR5e 6 + P1b 20 revolute), actuated 축약 모델 **16** (팔 6 + 손 10). DemoWbc·CLIK 의 control model 은 actuated 모델이 있으면 그것이므로 ur5e_p1b 의 CLIK nv 는 16 이다. 이전 기록의 22 는 근거가 없다. 로봇 config 의 `urdf.*` 는 rclcpp 파라미터로 읽고 `urdf.sub_models.<name>.*` 처럼 map key 로 파싱한다 (list-of-dict 불가)
- 지문 센서: P1b 실기 `HandSensorState` 250 Hz, finger-on-object 부호. sim 의 `WrenchStamped` contact-wrench lane 도 커밋 0fcc1d23 (2026-09-09) 이후 **같은 부호** (변환 지점은 `rtc::grasp::PullContactConfig::force_sign` 하나). `rtc_msgs` FingertipSensor 주석은 PR #538 로 고쳐졌고 이 브랜치에 병합됐다 (2026-09-19). 센서 lane 에는 수신 시각·sequence 가 없다 (D-24)
- sim 공: `/sim/launch_ball`·`/sim/reset_ball` (Trigger), `/sim/ball/ground_truth` (Odometry), `/sim/ball/camera_position` (PointStamped + noise), 항력·Magnus 자체 구현, iiwa7_leap 설정 없음. 공 샘플 발행은 sim time 으로 게이트되고 stamp 는 wall 이다. 기존 RTF 는 200 step 구간 평균이고, throttle 기준은 `max_rtf` 가 바뀔 때만 재설정된다 (§5)
- sim 모델: ur5e_p1b sim 이 로드하는 MJCF 는 형제 저장소 hand-description 의 사본이고 `model_pairs.yaml` 게이트에 ur5e_p1b 쌍이 없다 (§9)
- vision: 형제 workspace 의 ball_perception 저장소 `ball_perception_sim` 패키지 `sim_estimator_node` 가 `/sim/ball/camera_position` 을 구독해 예측 궤적 PointCloud2 를 debug 토픽으로 발행한다 (point_step 384, `horizon_ns` u32, `generation`, `validity`, `covariance` NaN=모름, `frame_id` 는 profile 값 — 예시 `world`). **stable ABI 아님** — 제품 ABI 는 ball_perception E6-F02 로 defer
- 참조 구현 테스트: l0/l2/l3/l4 + `verify_l3.py` 전부 통과, ASan/UBSan 통과. 단 테스트 밖 결함 확인: `n > kMaxSamples` 범위 밖 읽기(`traj_sampler.hpp` `check()` 가 상한 초과를 기록만 하고 계속 인덱싱), NaN 목표 영구 오염, derate 결함(D-8), 한계 무효 시 `tMinChecked` 가 구분 불가한 0 반환. 참조 헤더는 `catching::*` namespace·camelCase 그대로이며 이름 변경은 S1.1 에서 이식하며 한다

## 3. 시간 규약 (D-2)

| 판정 | 비교 대상 | 비고 |
|---|---|---|
| 궤적 샘플링, γ 프로파일, 기준 생성 | now_lead = now + T_arm | 팔 명령은 T_arm 뒤 실현 |
| CLOSING→DECEL (= DECEL 진입) | now_lead ≥ t_c | 감속 대상 전환도 팔 명령 |
| 궤적 지평 끝(외삽) 경고 | now_lead | 샘플링 시각 기준 |
| APPROACH→COMMITTED | t_c − now ≤ T_freeze | now = 실제. T_freeze 하한에 T_arm 포함 |
| COMMITTED→CLOSING, 손 Preshape·Close | now ≥ t_cmd, now ≥ t_c − T_pre | 손은 선행 보상 없음 |
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
| S5 착수 전 `[CONCERN] E-8` | P-1 최소 E-STOP 계약 (§4.4 S5) | S5.1 |
| S6 착수 전 `[CONCERN] E-7` | 전 tier × profile 배치표, tier 4 정책, `catching_on/off` (§6) | S6.1 |
| D-24 결정 | 지문 센서 freshness 경로 | S5.2e, S7.3 |
| D-12 값 | floor·시행 수·공 사양·손 토크 권위 출처 | S4.4 판정, S7.3 충격 게이트, S8 |

**NOT_EVALUATED 규칙.** 게이트 항목은 PASS 기준과 산출물을 가진다. 판정에 필요한 값이 아직 없으면 그 항목은 `NOT_EVALUATED(<없는 값>)` 로 기록하고 통과로 세지 않는다. 값이 정해지면 다시 판정한다. provisional 값으로 판정한 통과는 `PASS(provisional)` 로 적고, 값 확정 시 재판정한다.

### 4.2 단계 DAG

```
S0 ─┬─ S1  : S1.1–S1.9            (S1.3 의 D-2 변환 함수는 S0.6 승인 후)
    ├─ S2  : S2.1, S2.2a → S2.2b, S2.3a, S2.5, 마지막에 S2.4
    ├─ S3a : S3.1a, S3.2*, S3.3, S3.4, S3.7, S3.8   (* S0.8 승인 후)
    └─ S4a : S4.0 → S4.1, S4.2, S4.3, S4.5

S4.1 ──────────────────────────────► S2.3b (포켓 중심 offset)
S0.7, S1.9, S2.3a, S2.3b, S3.2 ────► S3.5a (kinematic catchability 지도)
S3.5a, S4.2, S4.5, S2.5, S1.5, S1.7 ► S4.4 (go/no-go)
S4.4 ─► S3.5b (gate-catchable 지도) ─► S3.6 (vision 요구 사양) ─► D-12 투척 분포 동결 (사용자)
S3.6 ─► S1.2 backfill (n_max ≤ kCap 확인, 초과 시 kCap 상향 후 S1 게이트 재실행)

S1, S2, S3a, S4.4(go), S0.6, S0.8, E-8 승인, D-24 ─► S5
S5, S3.6, E-7 승인 ─► S6 (S3.1b 부하 RTF 포함) ─► S7 ─► S8 ─► S9 ─► S10
```

S1 ∥ S2 ∥ S3a ∥ S4a 는 서로 독립이다. S4.0 은 S5 의 컨트롤러 골격 일부를 앞당긴 것이다 (손 계단 명령을 낼 컨트롤러가 없으면 S4.2 를 할 수 없고, `DemoJointController` 는 손 목표를 quintic 궤적으로 보간해 계단 응답을 줄 수 없다).

### 4.3 단계 상태

| 단계 | 상태 | 게이트 결과 |
|---|---|---|
| S0 결정·문서 v0.5·계약 | 완료 (2026-09-19) | S0.2 W 기록 칸 전부 채움. S0.3 설계 문서 12개(v0.5 헤더) 동기화, 이 문서 포함 `validate_docs` 13 files clean. 정합화 개정 (§7.4). 승인: issue #537 코멘트. S0.7 필요 지평 0.46–0.86 s (R2 지배)·`kCap` 40 제안, 0.5 s profile 부족 (§4.4 S0 결과). S0.9 검정력 표 (§1a) |
| S1 순수 수치 코어 | 대기 | — |
| S2 기존 rtc_* 일반화 | 대기 | — |
| S3a 시뮬레이션 기반 | 대기 | — |
| S4a 손 타이밍 측정 | 대기 | — |
| S3b·S4.4 지도·go/no-go·vision 사양 | 대기 | — |
| S5 포구 컨트롤러 골격·입력·추종 | 대기 | — |
| S6 계획기 스레드 | 대기 | — |
| S7 손 시퀀서·슈퍼바이저 | 대기 | — |
| S8 sim 통합 평가 | 대기 | — |
| S9 E-STOP·fault 정책 (D-13) | 대기 | — |
| S10 실기 단계 도입 | 대기 | — |

### 4.4 단계별 작업과 게이트

게이트 표의 "판정 입력" 은 그 항목을 판정하는 데 필요한 값과 그 출처 단계다. 표에 없는 입력을 쓰는 게이트는 없다.

#### S0 결정·문서 v0.5·계약 (코드 없음)

- S0.1 결정 D-1~D-24 기록 (D-12 는 값이 준비되는 대로, D-13 은 S9, D-24 는 S5 전) — 완료 (D-21~D-24 는 정합화 개정에서 추가)
- S0.2 `WORKSPACE_ANALYSIS.md` 기록 칸을 §2 로 채운다 — 완료
- S0.3 설계 문서 v0.5 개정 — 완료. 참조 코드 명명 규칙(namespace `rtc::catching`, 함수 PascalCase)은 **규칙만** 문서화했고 헤더 이름 변경은 S1.1 에서 이식하며 한다
- S0.4 (삭제 — ball_perception 은 사용자가 직접 개발 중이라 요청 이슈 불필요, P-3)
- S0.5 Epic issue #537 생성 + Sprint Contract — 완료
- S0.6 `[CONCERN] E-1` — §3.1 예외 문구 승인 — 완료 (2026-09-19, invariants.md §Clock 시간축 규칙에 별도 커밋으로 기록)
- S0.7 vision 지평 손계산 — D-18 발사 영역(4 m, z 1.5–2.0 m)과 속도 가정으로 탄도식 비행 시간을 구하고, "포구점이 예측 지평 안에 드는 시각 ≥ t_c − T_freeze" 가 되려면 필요한 지평·점 수를 산출한다. 결과로 (a) S1.2 의 `kCap` 제안값 (필요 점 수 × 2), (b) 현 예시 profile (지평 0.5 s) 로 충분한지를 사용자에게 보고한다. 부족하면 D-18 의 거리·속도 범위부터 조정한다 — 완료 (2026-09-19, 아래 결과. 부족분은 거리·속도 조정으로 메워지지 않음을 확인)
- S0.8 `[CONCERN] E-3` — D-14·D-20 (§7.1) — 승인 완료 (2026-09-19)
- S0.9 Epic 검정력 표 — (가정 성공률 × floor) 격자에서 Wilson 95% 하한이 floor 를 넘는 데 필요한 시행 수. D-12 floor 확정 시 S8 벽시계 예산이 불가능한 조합을 배제하는 입력 — 완료 (2026-09-19, §1a)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 문서 | `validate_docs.py --files` 변경 md 전부 clean, `git diff --check` clean | — |
| W 기록 | `WORKSPACE_ANALYSIS.md` §3 기록 칸 공란 0 | — |
| 승인 | S0.6·S0.8 승인 기록이 issue #537 코멘트에 있음 | 사용자 |
| S0.7 | 필요 지평·점 수·`kCap` 제안값과 계산식이 이 문서에 기록됨 | D-18 |
| S0.9 | 검정력 표가 §1a 아래에 기록됨 | — |

**S0.7 결과 (2026-09-19, 스크립트 실행값 — 에이전트 산출, 탄도 대표값·문서 인용값 재확인).** 지평 H 는 `header.stamp` 부터 마지막 점까지, 점 간격 0.05 s.

- 탄도 (2차 항력, L0 §4.1 모델): **a = −g ẑ − k‖v‖v**. Magnus 항은 없다 (sim 은 Magnus 도 구현 — L0 G0-5). 수평 이동 d = 4 − ρ_c 와 높이 z₀ → z_c 를 시간 T_f 에 잇는 v₀ 를 수치 적분 (rtol 1e-10) + 슈팅으로 푼다. 무항력 해 (v_x = d/T_f, v_z = (Δz + gT_f²/2)/T_f, Δz = z_c − z₀) 를 초기값으로 쓴다. 아래 탄도 수치는 모두 항력 포함값이다 (2026-09-19 재계산)
- 포구점이 지평에 처음 드는 시각은 t_c − H, plan 에 반영되는 시각은 t_c − H + L (L = L_vis + T_pub + T_plan + `t_horizon_margin`)
- (R1) commit 조건 — S0.7 원문 조건 (§3, L3 §4.11): H ≥ T_freeze + L, T_freeze ≥ T_close,tot + T_arm + T_margin
- (R2) 팔 이동 조건 (L3 §4.3 도달시간 + γ 창 T_w): H ≥ T_arm + T_margin + T_move + L, T_move = max(t_min(대기 자세 → q*), T_w)
- H_req = max(R1, R2), 점 수 n = ⌈H_req/0.05⌉ + 1. sweep 전 구간에서 R2 가 지배한다
- 계획 가능한 투척은 T_f − T_det ≥ H_req 인 것뿐이다 — 지평을 늘려도 이 한계는 넘지 못한다

| 가정 | 값 | 출처 |
|---|---|---|
| 발사 거리·높이 | 4 m, z 1.5–2.0 m | D-18 |
| 포구점 | ρ_c 0.3–0.8 m, z_c 0.4–1.0 m (base 를 world z=0) | 가정 — world ↔ base 는 S3.2 |
| `t_horizon_margin`, T_pub, T_plan | 0.05, 1/30, 0.010 + 1/60 s | L2 기본값, D-15 예시, L3 `budget_s` |
| L_vis, T_det, T_margin | 0.03, 0.10, 0.02 s | 가정 (sim 은 같은 호스트) |
| T_close,tot, T_arm | 0.01–0.10, 0–0.10 s | 가정 (T_close,tot 문서 예시 60 ms, T_arm TBD) |
| T_w, 관절 t_min | 0.3–0.6 s, ā = 10 rad/s², ω̄ = π rad/s | L3 `window_grid`, 참조 구현 `test_l3` 값 (D-16 box TBD) |
| 항력 k | 0.0229 1/m | L2 §4 검증 표의 값 (공 사양 D-12 대기, sim 식별값 `sim.ball.drag_k` 는 S3.8) |

| 항목 | 결과 |
|---|---|
| 도달 가능 투척 (v₀ 4–12 m/s, 포구점 기하 27 조합) | 최소 v₀ 4.5–5.8 m/s (포구점별), 포구 속력 ≥ 5.9 m/s. 최소 v₀ 는 T_f 0.8–0.9 s 부근 |
| T_f ≥ 1.0 s 인 투척 (D-18) | T_f 1.0 s 에서 앙각 40–53°, v₀ 4.7–5.9 m/s, 정점 z 2.2–2.8 m, 포구 속력 6.1–7.2 m/s, 하강각 약 60°. T_f 1.5 s 면 앙각 68–72°, v₀ 6.8–7.7 m/s, 정점 3.7–4.3 m, 포구 속력 7.7–8.4 m/s (포구점 기하 27 조합) |
| 대표 (z₀ 1.75, ρ_c 0.5, z_c 0.6) | T_f 0.5/0.7/0.9/1.0 s → 앙각 1/19/38/46°, v₀ 7.3/5.5/5.1/5.3 m/s, 포구 속력 8.1/6.9/6.7/6.8 m/s. 무항력 대비 같은 T_f 에서 v₀ +2.7–4.3 %, 포구 속력 −3.4–3.8 %, 앙각 −0.5–1.2° |
| L | 0.140 s |
| R1 | H 0.17–0.36 s (n 5–9) |
| R2 | T_move 0.30/0.45/0.60 s × T_arm 0–0.10 s → H 0.46–0.86 s (n 11–19). 대기 자세에서 먼 q* (Δq 1.5 rad → t_min 0.79 s) 이면 H ≈ 1.0 s |

- **`kCap` 제안 40 (provisional)** — sweep 최대 n 19 × 2 = 38 을 8 의 배수로 올림. T_f 상한 1.04 s 도 n 22 로 담는다. 먼 q* (H ≈ 1.0 s, n 21) 까지 여유 2배를 요구하면 부족하므로 S3.6 의 `n_max` 가 넘으면 backfill (§4.2)
- **0.5 s profile 판정: R1 충족, R2 부족.** R2 는 T_arm 0.05 s 에서 T_move ≤ 0.29 s 일 때만 충족하는데 T_w 격자 최솟값이 0.3 s 다. 10 점의 첫 점이 horizon 0 이면 실제 지평은 0.45 s 로 더 짧다 (S3.4 실측)
- **D-18 거리·속도 조정으로는 부족분이 메워지지 않는다.** R1·R2 는 로봇 쪽 선행시간이라 발사 조건과 독립이다. 거리·속도가 정하는 것은 T_f − T_det ≥ H_req 뿐이다 — H_req 0.66 s (T_move 0.45, T_arm 0.05) 이면 T_f ≥ 0.76 s, 즉 θ ≳ 25°·v₀ 5–5.5 m/s 의 lob 만 남는다
- **T_f ≥ 1.0 s 결정 (D-18, 2026-09-19) 의 효과:** T_f − T_det ≥ 0.9 s 가 sweep 최대 H_req 0.86 s 를 넘으므로, 지평만 충분하면 목표 분포의 모든 투척이 R2 까지 계획 가능하다 (먼 q* 의 H ≈ 1.0 s 는 T_f ≥ 1.1 s 부터). 필요 지평 자체는 로봇 쪽 값이라 그대로 0.46–0.86 s 이고 0.5 s profile 부족 판정도 그대로다. 대신 포구 속력 하한이 5.9 → 6.1 m/s 로 오르고 T_f 와 함께 커진다 — S4.4 부담은 커진다
- **권장 → 채택 (2026-09-19)**: sim profile 지평 0.8 s (17 점 @ 0.05 s, 설정은 사용자 — D-15). 0.8 s 는 R2 를 T_move + T_arm ≤ 0.64 s 까지 덮는다 — T_move 0.45 s 전부, T_move 0.60 s 는 T_arm ≤ 0.04 s 일 때. 먼 q* (H ≈ 1.0 s) 는 덮지 못하므로 대기 자세를 겨냥점 가까이 두어 T_move 를 줄이는 권장은 남는다 (§7.3). R2 채택 여부는 아래 결정
- **지평 요구 = R1 (2026-09-19 사용자 결정).** 궤적은 검출 직후부터 매 수신마다 갱신되고, 계획기는 `APPROACH` 동안 지평 안 후보로 먼저 출발했다가 더 나은 후보로 교체한다 (L7 `TRACKING → APPROACH`, L3 §4.7 교체 히스테리시스). 도달시간 검사는 대기 자세가 아니라 **현재 명령 상태 (q_c, q̇_c)** 에서 한다 (L3 §4.3). 따라서 R2 (대기 자세 정지 출발) 는 지평 요구가 아니라 **첫 plan 이 나오지 않는 최악 경우의 기록값**이다. R2 로 `io.horizon_min` 을 잡으면 0.8 s profile 을 통째로 거부할 수 있다. 예 (T_f 1.0 s): 0.10 s 첫 예측 (지평 끝 0.9 s) → 약 0.24 s 첫 plan, 작업공간 가장자리 후보로 출발 → 0.25 s 이후 1.0 s 지점이 지평에 들어와 교체 → `t_c − T_freeze` 동결. 지평 (0.8 s) < T_f (≥ 1.0 s) 이므로 첫 목표는 항상 작업공간 가장자리다 — 가장자리 후보의 catchability 는 S3.5a, 첫 plan 시각·교체 횟수·탈락 사유는 S8 에서 잰다
- **대기 자세 = 겨냥점 근처 (2026-09-19 사용자 결정).** 연속 재계획에서 대기 자세가 정하는 것은 정지 출발인 첫 이동이다 — 가까울수록 가장자리 후보의 도달시간 탈락과 교체 시 오차 점프가 준다. IK seed (D-18)·지도 겨냥점 (§11) 과 같은 자세다. 구체 자세는 S3.5a 방위별 포구 가능 구간으로 정한다
- **`kCap` = 40 (2026-09-19 사용자 결정, provisional).** 0.05 s 간격에서 약 1.95 s 지평까지 담고, 스냅샷 매 tick 복사는 약 3.2 KB 다. S3.6 `n_max` 가 넘으면 backfill (§4.2)
- 포구 속력 ≥ 5.9 m/s (T_f ≥ 1.0 s 에서 ≥ 6.1 m/s) 는 CATCHING_MASTER §4.1 의 우려(6 m/s 에 T_close,tot ≤ 8.9 ms 필요)를 4 m 투척에 대해 확인한다 — 거리를 줄여도 크게 내려가지 않으므로 S4.4 go/no-go 의 핵심 입력이다
- 스크립트는 저장소에 두지 않았다. 식과 가정표만으로 재현된다

#### S1 순수 수치 코어 (ROS 비의존)

- S1.1 rtc_controllers `catching` 하위 디렉토리 골격, 참조 테스트를 GTest 로 이식하며 이름을 `rtc::catching` + PascalCase 로 바꾼다
- S1.2 궤적 타입(공용) + Hermite 샘플러. SeqLock 에 싣는 타입(궤적 스냅샷, PlanSnapshot)은 trivially copyable POD (`std::array` 기반, Eigen 멤버 금지 — §6). 용량은 컴파일타임 상수 `kCap` (S0.7 제안값, provisional) 이고 S3.6 은 런타임 `n_max ≤ kCap` 만 정한다. 점 개수 `[n_min, n_max]` 를 파서·`Check`·RT 읽기 모두에서 **인덱싱 전에** 검사한다 (참조 `check()` 의 상한 초과 후 인덱싱 결함 수정). NaN 입력 거부, `dt_min` 미만은 경고가 아니라 거부. provenance token 필드 (D-22)
- S1.3 시간 타입 `BallTime`/`NowReal`/`NowLead` (§3). D-2 변환 함수는 S0.6 (E-1) 승인 후
- S1.4 soft-catch 기준 생성기: NaN 가드(비유한 목표 시 상태 보존 + invalid), derate 없음(D-8)
- S1.5 도달 가능성: `time_feasibility`, 방향 속력(투영 v̂ᵀJ_p q̇, 0 가드), 정지거리·오차 예산. 잘못된 한계는 구분 가능한 flag (참조 `tMinChecked` 의 0 반환 수정)
- S1.6 `ball_dynamics` 는 test fixture 전용 위치로
- S1.7 파라미터 검증 로직: 활성 구성 키만 TBD 검사, 교차제약 표(D-9 반영), ζ·ω·h 검사(`dt` 기준), provisional 값의 실기 arm 차단
- S1.8 L7 순수 조각: 감속 목표, 전이표를 데이터로, 접촉 debounce
- S1.9 포구 자세 IK 반복 루프 + catchability 판정 함수 (L3 §4.2, §11): `DifferentialIk` (m=5) 를 반복 호출하는 루프, 수렴 판정·스텝 제한·seed = wait_pose, 해에서 w₅·w₆ 계산과 fail-closed 판정 (§11). 입력은 `RtModelHandle` (rtc_controllers 는 이미 `rtc_urdf_bridge` 에 의존). **S3.5a/b 지도 도구와 S6.2 런타임이 이 함수 하나를 쓴다.** 추상 interface 는 만들지 않는다 (ARCH-3)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 이식 | 참조 테스트 전부 GTest 통과 (L0 G0-A, L2 G2-A~D·G2-G, L3 G3-A·G3-B, L4 G4-A~D·G4-F) | — |
| 회귀 | `n > kCap`·NaN·비단조·`dt_min` 미만 입력에서 ASan/UBSan 무오류 + invalid (G2-H, G1-A 의 해당 항목), NaN 목표 가드 (G4-I) | — |
| RT | 대상 경로 `ScopedNoMalloc`·`ScopedAllocGate` 할당 0, `noexcept` (G0-B, G2-E, G4-G) | — |
| 시간 | 다른 시간 타입끼리 비교가 컴파일되지 않음, T_arm ≠ 0 fixture (G0-E) | — |
| 검증기 | 활성 구성 TBD·provisional·교차제약·ζ·ω·h (G0-C) | — |
| S1.9 | 합성 기구학에서 수렴·스텝 제한, w₅·w₆ 유한차분 대조, zero speed·NaN/Inf·rank-deficient·near-singular 입력에서 후보 탈락 + 사유 코드 (G3-I 의 함수 부분) | — |
| backfill | S3.6 의 `n_max ≤ kCap`. 초과하면 `kCap` 상향 후 이 표 재실행 | S3.6 |

GUI·plot: 면제 (D-19, §13).

#### S2 기존 rtc_* 일반화 (code review 대상)

- S2.1 `rtc_math` se3 에 축 정렬 오차·각속도·Jacobian (deadband 에서 유한), 유한차분 테스트
- S2.2a CLIK 확장 구조 결정 1쪽 ("행 집합 선택형 확장" 대 "`QPSolverWrapper`·se3 오차만 공유하는 formulation 클래스") + **기존 동작 golden-vector 회귀** (기록한 q 열 → q_ref 해시). golden 테스트가 생기기 전에는 기존 위치∩속도 box 코드를 재구조화하지 않는다
- S2.2b `ClikReferenceGenerator` 옵션 (D-5·D-6), 옵션별 개별 커밋: twist feedforward, LOCAL 접근축 2행, 가속 box + `bound_conflict`, 직전 q̇ 평활 항, status·반복·solve time 노출, `max_iter` 설정, q_c 평가 모드
- S2.3a `rtc_urdf_bridge` extra frame 기구 (D-10, D-17, §10): 로봇 config `urdf.extra_frames.<name>.{parent,xyz,rpy,provisional}` → CM 파서 (`list_parameters` map key, `ParseSubModels` 와 같은 방식) → `ModelConfig` 필드 → yaml-cpp `LoadModelConfig` 경로(같은 키 지원 또는 명시적 거부) → `PinocchioModelBuilder` 가 `BuildFullModel()` 직후 full 모델에 `addFrame`. 축(rpy) 초기 제안값 산출. 결합 모델 nv (full 26 / actuated 16) 를 테스트로 고정
- S2.3b (S4.1 이후) 포켓 중심 offset 제안값 — preshape 자세에서 손가락 끝 위치 중심 (§10)
- S2.5 관절 가속 한계 도출 도구 (D-16, §9)
- S2.4 (S2 의 마지막) DemoWbc 회귀(기존 assertion 무수정), `rtc_tsid`·`rtc_math`·`rtc_urdf_bridge` downstream 빌드·테스트, public header Doxygen·패키지 README 갱신, `/code-review`

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 동등성 | 옵션 전부 off 에서 golden-vector 해시 일치·기존 CLIK 출력 bit-identical (G5-A2), 기존 테스트 assertion 무수정 green (PROC-6, E-6) | — |
| CLIK | G5-A, G5-B, G5-B2, G5-C3, 할당 0 (G5-C 의 할당 부분) | G5-C 의 solve time 예산은 사용자 결정 → 그 부분 NOT_EVALUATED |
| se3 | G4-D | — |
| extra frame | catch frame 이 full·sub·tree·actuated 네 모델 모두에 존재하고 같은 부모 기준 위치를 가짐, 없는 부모·중복 이름은 configure 실패, provisional frame 은 실기 arm 차단 (G0-C) | — |
| 가속 도출 | §9 의 산출물(infeasible 비율·binding 제약·provenance) 이 YAML 에 기록됨, 퇴화 시 채택하지 않음, iiwa7 sim 교차 검증에서 도출값 ≤ 달성 가속 | η_τ (S2.5 제안, 사용자 확인) |
| 문서 | public header Doxygen, README 갱신 (PROC-1) | — |
| GUI·plot | §13 S2 행 | — |

#### S3a 시뮬레이션 기반 (`rtc_mujoco_sim`, robot-agnostic)

- S3.1a D-3 무부하 검증 (§5) — 결과에 따라 D-3 재검토
- S3.2 발사 srv (D-14, S0.8 E-3 승인 후, PROC-3), iiwa7_leap projectile 설정, 투척용 스폰 위치. **world ↔ arm base 변환을 같은 q 의 MuJoCo FK·Pinocchio FK 대조로 확정**한다 (S3.5a 의 선행)
- S3.3 공 접촉 truth(시각·충격량·접촉력) 출력, truth 발행 주기 상향, per-step `(sim_time, steady_now)` 진단 lane (§5 판정용)
- S3.4 `sim_estimator_node` 연결 — **측정만 한다** (정책은 S5.2): clock domain(`use_sim_time=false`), `frame_id` 와 world 관계, 발행 주기·N·지평 실측 (TBD-VIS-04/06), 구독 reliability 비교, validity 패턴 히스토그램, 재시작 시 `snapshot_sequence` 거동, 유령 트랙(관성 예측만 발행) 시 `validity` 거동, 지연·드롭 주입
- S3.7 팔 지연 에뮬레이션 (L5 §4.6 `sim.arm_lag`) + 지연 식별 도구 (L5.7): step 응답으로 T_arm,sim 을 순수지연과 시상수로 분리 식별하고 σ_trk sim 초기값을 산출한다 (MuJoCo position servo 응답은 순수 지연이 아니다)
- S3.8 공 항력 k 식별 도구 (L0 §7, G0-D)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| e2e | 발사 → PointCloud2 수신 end-to-end, 같은 seed 재발사 시 truth 궤적 동일 | — |
| D-3 무부하 | §5 판정 (구성별 무효율 상한) | ε_clk 할당 (r_cap, TBD-HAND-04) → 없으면 NOT_EVALUATED, δ·pause 분포는 기록 |
| frame | world ↔ base FK 대조 잔차 < 1e-6 m, 결과가 §11 에 기록됨 | — |
| 지연 식별 | G5-D (주입 지연 식별 오차 < 2 ms) | — |
| k | G0-D (잔차 기록, 합격 임계는 사용자 결정 → NOT_EVALUATED) | 사용자 |
| PROC-3 | S3.2 의 `rtc_msgs` 변경 후 전체 빌드·테스트 | — |
| GUI·plot | §13 S3 행 | — |

#### S4a 손 타이밍 측정

- S4.0 포구 컨트롤러 최소 골격 (S5.1 에서 앞당김): 등록·lifecycle·config 로드와 **손 계단 진단 모드**만. 팔은 현재 자세 hold, sim 전용, 상태 메시지 없음 (CSV 만). E-STOP 훅은 base 기본 동작과 CM 측 hold 방어선에 맡긴다 — 팔 명령 경로·CLIK 앵커가 생기는 S5.1 이 E-8 대상이다
- S4.1 손 프로파일 YAML (P1b 10 DoF, LEAP 16 DoF)
- S4.2 T_close 식별 도구: S4.0 의 손 device slot 계단 명령 + CSV → ρ-min 분포 (sim)
- S4.3 가능하면 실기 T_close,tot 측정
- S4.5 포켓 유효 깊이 `d_eff` 산정 (L6 §4.5: 기하 추정 + sim 실험 보정) — provisional, 사용자 승인 대상 (TBD-HAND-04)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 손 명령 | 손 device slot 에 쓴 목표와 backend 명령이 전 tick 일치 (G6-B), S4.0 에서 팔 명령 변화 0 | — |
| T_close | 두 손의 T_close,e2e(η) 분포(평균·최대·99%) 산출 (G6-C 의 산출 부분) | — |
| d_eff | G6-F: 산정식·실험값·provisional 표시가 YAML 과 이 문서에 기록됨 | 사용자 승인 |
| GUI·plot | §13 S4 행 | — |

#### S3b·S4.4 지도·go/no-go·vision 사양

- S3.5a kinematic catchability 지도 (D-18, §11): 발사 영역 × 발사 속도·각도 격자 → 궤적 → 포구 후보 → S1.9 함수 (IK + w₅/w₆) → 잡을 수 있는 발사 조건 범위. 타이밍 값은 provisional
- S4.4 go/no-go: S3.5a 의 속도 범위, S4.2 T_close, S4.5 d_eff, S2.5 가속 box, S1.7 η_v 로 받을 수 있는 최대 공 속력을 계산해 목표 속도를 확정하거나 낮춘다
- S3.5b gate-catchable 지도: S4.4 값으로 전체 게이트 체인(IK + w + 도달시간 + γ 창 + 정지거리)을 다시 돌린다. 이 지도가 목표 투척 분포가 된다
- S3.6 vision 요구 사양 산출 (D-15): 목표 분포에서 "검출 이후 포구 창 종료까지 최대 비행 시간" → 필요 지평, L2 보간 게이트를 만족하는 간격 → 점 수 → 런타임 `n_max` (≤ `kCap`, 넘으면 S1.2 backfill). 결과를 ball_perception sim profile 설정값으로 제시 (설정은 사용자)
- 사용자가 D-12 투척 분포를 동결한다

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| go/no-go | 목표 속도에서 γ 창이 비지 않음 (G6-C 의 판정 부분) | T_close (S4.2), d_eff (S4.5), 가속 box (S2.5), η_v (S1.7), 목표 속도 (S3.5a) — 하나라도 provisional 이면 PASS(provisional) |
| 지도 | 지도와 런타임 판정 동치: 같은 입력에서 S1.9 함수 판정이 같음 (G3-I), kinematic·gate 두 지도와 탈락 사유 분포 기록 | — |
| 사양 | S3.6 의 지평·간격·점 수·`n_max` 가 이 문서에 기록되고 `n_max ≤ kCap` | — |

#### S5 포구 컨트롤러 골격·입력·추종 (Adding a New Controller)

착수 조건: S0.6·S0.8 승인, `[CONCERN] E-8` 승인, D-24 결정.

- S5.1 컨트롤러 완성 (S4.0 확장): YAML, lifecycle, 재무장. **E-STOP·fault 최소 계약 (P-1, E-8)**: (a) `TriggerEstop`·`ClearEstop`·`ResetFault`·`ResetTargetInitialization` 훅은 atomic 요청·epoch 만 갱신하고 reset 의 유일 writer 는 RT tick, (b) 해제 후 자동 재개 금지 — q_c·CLIK 앵커를 q_meas 로 reseed, (c) plan·궤적·공분산·손·FSM·타이머 무효화는 D-23 순서로, (d) `ClearEstop` 은 컨트롤러 fault 를 풀지 않는다 (base 계약: 두 경로는 별개). 전체 물리 정책은 S9
- S5.2 PointCloud2 구독 (nrt, `KEEP_LAST(1)`, reliability 는 S3.4 결과) → 필드 이름 파서 → SeqLock 스냅샷 (공분산 제외, A-3). D-2 변환, `generation`/`validity`/`snapshot_sequence` 처리 — C-1 정책(한 점이라도 무효면 거부)을 구현하고, S3.4 측정에서 부분 무효·재시작 되감김이 나왔으면 착수 전에 C-1 재검토와 되감김 정책을 정한다. 스냅샷에 `ActivationGeneration()`·token (D-22, D-23). 지평이 요구(D-15)보다 짧으면 진단. 공분산은 같은 token 을 가진 계획기 쪽 버퍼에만. S5.2e RT 상태 POD 에 D-24 의 센서 freshness
- S5.3 스트리밍 기준 → 확장 CLIK → 팔 명령. QP 비의존 관절공간 abort 경로. 예측 선행 보상 (L5.8), backend 왕복 대조 (L5.10)
- S5.4 CSV 로그, 상태 publisher (`PublishRole` 을 늘리지 않음, E-11) — 포구 상태 메시지 신설 (D-20, `rtc_msgs`, S0.8 E-3 승인 후, PROC-3). **S5~S9 필드 superset 동결**, `Compute()` 의 모든 early-return 분기(E-STOP·stale·generation 불일치·지평 부족·plan 없음·abort)에서 Store 하고 그 tick 에 계산하지 않은 필드는 무효화 (PROC-7). DemoWbc 의 `!target_initialized_` early-return 은 Store 를 빠뜨리므로 선례로 쓰지 않는다
- S5.5 ground truth 기반 고정 포구점(oracle plan)으로 추종 검증

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 추종 | G4-H, G5-A~B2, G5-C3, G5-C4 | — |
| RT | 할당 0 (G1-D, G5-C 의 할당 부분), QP solve time | solve time 예산은 사용자 결정 → NOT_EVALUATED |
| 입력 | G1-A~C, G1-E, G1-F, G1-I (nrt 콜백을 일부러 막아 backlog 를 만든 뒤 최신 `snapshot_sequence` 만 수락, ARCH-6) | — |
| SeqLock | writer 부하에서 찢어진 스냅샷 0·최악 재시도 시간 기록 (G1-C). writer 끼어들기를 재현하는 결정적 테스트에서 최신 스냅샷 누락 0 (G1-H, D-21) | — |
| activation | 비활성 중 받은 궤적이 재활성 첫 tick 에 소비되지 않음 (G1-J, D-23) | — |
| E-8 | G7-H: (a) deactivate → 다른 컨트롤러가 팔 이동 → 재activate 시 첫 tick 이 옛 자세를 명령하지 않음, (b) trigger·clear·deactivate race 에서 reset writer 가 RT tick 하나, (c) 자동 재개 0, (d) `ClearEstop` 후에도 latched fault 유지. `/security-review` (최소 정책) | — |
| PROC-7 | G8-H: early-return 분기마다 그 tick 의 body 가 실렸는지 보는 실패 경로 테스트 (`EstopTickPublishesThisTicksBody*` 선례) | — |
| 선행 보상 | G5-C2 (backend 왕복), G5-E (지연 에뮬레이션에서 선행 보상 전후 기록) | S3.7 |
| PROC-3 | S5.4 (및 D-24 (a) 선택 시 S5.2e) 의 `rtc_msgs`·`rtc_base` 변경 후 전체 빌드·테스트 | — |
| GUI·plot | §13 S5 행 | — |

#### S6 계획기 스레드 (D-7)

착수 조건: S5, S3.6, `[CONCERN] E-7` 승인 (§6).

- S6.1 스레드 골격: MPC 스레드 생성 방식 그대로 (§6). RT-1~10 준수 코드, 초기 FIFO, thread layout role 추가 (Adding a New Thread), `ValidateSystemThreadConfigs` 의 `all_configs` 와 이름 목록 등록, absent-role 필터, `catching_on/off` profile, activate 게이트
- S6.2 포구 자세 IK·catchability: S1.9 함수를 스레드 전용 모델 handle 로 배선. 후보가 모두 탈락하면 plan 없음(포기)을 사유 코드와 함께 기록
- S6.3 γ 창·rollout (S1 코드 호출), 예산 초과 시 coarse-to-fine
- S6.4 후보 선택·hysteresis·commit/freeze, `PlanSnapshot` SeqLock (token·`publish_ns`, D-22), 계산 시작·게시 직전 token 재검사
- S6.5 D-7a 측정 (§7.2)
- S6.6 NLP 전환 대비 (A-4, §8): 탐색 전략을 계획기 코어의 단일 진입 함수 뒤에 두어, 1차원 탐색 + IK 를 NLP 로 바꿔도 스레드·입출력 스냅샷·RT 쪽 소비 코드는 그대로 두는 경계를 유지. 전환 판단 신호(IK 수렴률 G3-G, 계획 성공률, 예산 초과율)를 S6·S8 에서 기록
- S3.1b D-3 부하 재검증: 포구 컨트롤러 + 계획기 + `sim_estimator_node` 동시 구동 (§5)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 계획 | G3-A~E, G3-C 예산 준수 | `planner.budget_s` (L3) |
| IK·catchability | G3-G (수렴률 기록), G3-I (런타임과 지도 동치) | — |
| RT | G3-K (한 사이클 할당 0·noexcept·로깅 없음) | — |
| token·race | G3-L: eventfd coalescing, 계산 중 새 스냅샷 도착, 같은 generation 의 옛 plan 게시, deactivate·Pause race 에서 대체된 plan 소비 0 (D-22, D-23) | — |
| 스레드 | Adding a New Thread 3 oracle, `all_configs` 등록 (누락 시 그 role 만 전 규칙에서 빠진다 — #349 D15), 전 tier × profile `gen_thread_layout.py --check` | — |
| D-7a | G3-J — 제어 PC 에서만 판정. 개발 PC 결과는 NOT_EVALUATED(제어 PC) | 제어 PC |
| D-3 부하 | §5 판정 (부하 구성) | ε_clk 할당 |
| GUI·plot | §13 S6 행 | — |

#### S7 손 시퀀서·슈퍼바이저

- S7.1 손 시퀀서 → 손 device slot
- S7.2 FSM (전이표 = 데이터, Reason 완전), IDLE→wait_pose homing. DECEL 은 시각 기준 진입 (A-5, now_lead ≥ t_c), 지문 센서는 결과 판정·abort 전용. COMMITTED 이후 stale 은 동결 plan 으로 계속, `supervisor.stale_committed_max_s` 초과 시 ABORT_SAFE (A-6, L7 §4.2 — 초기값은 제안값이고 S7 착수 시 어느 물리량으로 조일지 정한다)
- S7.3 접촉 판정 (sim·실기 모두 finger-on-object — 착수 시 재확인), `TIP_STALE` (D-24 freshness), 감속, 충격량 예산
- S7.4 abort·retreat·재무장 리셋 (레이어별 소유자가 등록하는 단일 표, L7 §4.8), 연속 투척

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| FSM | G7-A, G7-B, G7-D | — |
| 센서 stale | G7-G: 관절 fresh + 지문 센서 dropout negative control 에서 `TIP_STALE` 또는 결과 `Undetermined` 발화, 옛 힘을 새 접촉으로 판정 0 | D-24 |
| 접촉 | G7-C (오경보율 기록) | 임계는 사용자 결정 → NOT_EVALUATED |
| 충격·토크 | G7-B3 | 손 토크 권위 한계 (D-12) → 확정 전 NOT_EVALUATED |
| 재무장 | G8-A2, 리셋 표 완전성 (모든 stateful 멤버가 표에 있거나 명시적 면제) | — |
| 결과 판정 | G7-E | — |
| PROC-7 | G8-H 를 S7 에서 늘어난 early-return 분기까지 확장 | — |
| GUI·plot | §13 S7 행 | — |

#### S8 sim 통합 평가

- iiwa7_leap → ur5e_p1b. Wilson CI, NEES, 소거실험(γ, lead). **γ 포화 빈도 측정 → D-8 재검토 입력**
- 연속 재계획 측정 (S0.7 R1 결정의 확인): 시행마다 발사 → 첫 plan 시각, `APPROACH` 중 교체 횟수·사유, 첫 plan 전 후보 탈락 사유 분포. 첫 plan 이 가장자리 후보 전멸로 늦어져 R2 최악 경우에 가까운 시행 비율을 보고한다

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 시스템 | G8-A, G8-B, G8-B2, G8-C, G8-C2, G8-C3, G8-E | — |
| 성공률 | G8-D (ur5e_p1b)·G8-D2 (iiwa7_leap): Wilson 95% 하한 ≥ floor, 로봇별. 전체 발사 수·무효 수·무효 사유를 같이 보고하고, 무효율이 §5 상한을 넘으면 그 run 은 NOT_EVALUATED | floor·시행 수 (D-12, S0.9) |
| clock 오차 (D-3) | S3.1a·S3.1b 결과가 §5 기준 통과 | — |
| GUI·plot | §13 S8 행 | — |

#### S9 E-STOP·fault 정책 (D-13)

- D-13 결정 → 구현 → `/security-review` (E-8)
- 결정 시 검토할 부작용: E-STOP 중 손도 측정 자세 유지 → position servo 간극이 0 이 되어 파지력 소실 가능

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 정책 | D-13 정책의 E-STOP 발동·해제 시나리오 테스트 전부 통과, S5 E-8 race 테스트 재통과 | D-13 |
| 리뷰 | `/security-review` 에서 Critical 0 (지적 사항은 해소하거나 사용자 수용 기록) | — |
| GUI·plot | §13 S9 행 | — |

**S10 착수 전 필수.**

#### S10 실기 단계 도입 (HW-P1B)

- bag replay(재스탬프 도구) → 가상 공 → 저속 실투척 → 상향
- T_arm 식별·선행, speed scaling·PTP 감시(신호 출처 확보 후), D-2 예외의 ③ 조건(PTP 동기) 확인

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 실기 | G5-F, G6-D, G6-E, G7-F, G8-F, G8-G (G8-G 의 "원인 분해" 는 항목별 수치 기록으로 판정) | 실기 |
| GUI·plot | §13 S10 행 | — |

## 5. D-3 검증 계획 (채택 조건부)

D-3 은 **검증 결과를 바탕으로 추가 검토한다.** 기존 RTF 신호(200 step 구간 평균)는 0.5 s 비행 동안 표본이 1~2 개라 짧은 스톨을 평균이 지우고, throttle 기준이 `max_rtf` 변경 시에만 재설정되어 스톨 뒤에는 RTF > 1 로 따라잡는 구간이 생긴다. 따라서 비율이 아니라 **시행별 clock 위상 오차**로 판정한다.

**측정 (S3.3 의 per-step lane).** 발사 시각을 원점으로 비행 구간의 매 step 에서 δ(t) = (steady(t) − steady₀) − (sim(t) − sim₀). 시행별로 δ_max = max|δ| 와 max pause (한 step 의 Δwall − Δsim 최대값) 를 기록한다. 양방향(뒤처짐·따라잡음)을 모두 본다.

**시행 유효 조건.** v_max·δ_max + ½·a_bound·δ_max² ≤ ε_clk,alloc 이고 max pause ≤ ε_clk,alloc / v_max.

- v_max: 목표 투척 분포의 최대 공 속력 (S3.5b 전에는 S0.7 가정값)
- a_bound: g + 항력 가속 상한 (S3.8 의 k)
- ε_clk,alloc: L3 §4.6 오차 예산 중 시계 항 ‖v‖δ 에 할당한 몫. r_cap (TBD-HAND-04) 과 할당 비율이 정해지기 전까지 이 판정은 **NOT_EVALUATED** 이고 δ_max·pause 분포만 기록한다. 할당 비율은 S3.1a 에서 제안하고 사용자가 확인한다

| 항목 | 방법 | 기준 |
|---|---|---|
| 무부하 (S3.1a) | 구성별 (로봇 2종) 발사 ≥ 200 회 | 무효율 ≤ 5 % |
| 부하 (S3.1b) | 포구 컨트롤러 + 계획기 + `sim_estimator_node` 동시 구동, 구성별 발사 ≥ 200 회 | 무효율 ≤ 5 % |
| 예측 일관성 | vision 예측 궤적 대 ground truth (같은 wall 시각 축) | 오차가 δ 가 큰 구간에서만 커지는지 확인 (기록) |
| stamp 도메인 | `sim_estimator_node` 의 stamp 가 wall 인지 | `use_sim_time=false` 에서 wall |

시행 수 200·무효율 5 % 는 제안값이다 (사용자 확인, §7.3). 어느 구성이든 무효율이 상한을 넘으면 `/clock` 방식을 포함해 D-3 을 다시 결정한다. 성공률 평가(S8)는 유효 시행으로 계산하되 전체 발사 수와 무효 사유를 함께 보고해, 무효 제외가 성공률을 편향하지 않는지 드러낸다.

## 6. D-7 계획기 스레드 구성

기존 MPC 스레드와 같은 방식으로 생성하고 기능만 planner 로 한다. 분석 결과 (2026-09-19):

**MPC 스레드의 실체.** 공용 기반은 `rtc::PeriodicRtThread` (rtc_base threading, 헤더 전용)다: jthread + stop token, 스레드 진입 시 `ApplyThreadConfigVerbose` (실패해도 무시하고 계속 실행), `clock_nanosleep` 주기, overrun 카운터, Pause/Resume, t0..t3 timing payload. `WaitForNextTick`·`JitterMeaningful` 은 virtual 이다. `rtc::mpc::MPCThread` 는 그 위의 얇은 subclass 이고 (`OnTick` = 상태 읽기 → `Solve` → 결과 게시), DemoWbc 가 `std::unique_ptr` 로 소유한다. 수명: `on_activate` 에서 layout profile 게이트 → lazy spawn → `Resume`, `on_deactivate` 에서 `Pause`, **join 은 소멸자에서만** (use-after-free 수정 이력). 배치는 `repo_scripts/config/thread_layout.yaml` 의 `mpc_main` role (tier 6 이상 slot 3 FIFO 60, tier 4 는 slot 3 OTHER).

**planner 구성 (권장).**

- 클래스: `PeriodicRtThread` 의 **형제 subclass** (catching planner thread). `MPCThread`/`MPCHandlerBase` 를 상속하면 PlanSnapshot 을 `MPCSolution` 에 억지로 넣게 되므로 쓰지 않는다. 같은 기반을 쓰는 4번째 소비자라 P5·ARCH-3 을 만족한다. 탐색 코어는 rtc_controllers `catching`, 스레드 소유는 `integrated_bringup` 바인딩 (D-1)
- 수명: DemoWbc 관용구 그대로 (configure 에서 전 버퍼 할당, activate 게이트 → spawn → resume, deactivate pause, 소멸자 join, aux 타이머로 `planner_timing_log.csv` drain). `Pause()` 는 진행 중 iteration 을 멈추지 않으므로 deactivate 뒤에도 한 번 게시될 수 있다 — RT 쪽이 `PlanSnapshot` 의 activation generation 으로 거른다 (D-23). DemoWbc 가 MPC 해에 이 장치를 두지 않은 것은 선례가 아니라 미해결 gap 이다
- 데이터: RT → planner `rtc::SeqLock<RtStatePod>`, planner → RT `rtc::SeqLock<PlanSnapshotPod>`. **SeqLock payload 는 trivially copyable 이어야 하고 `Eigen::Vector3d` 는 아니다** (Eigen 3.4 에서 확인) → PlanSnapshot·궤적 스냅샷은 `std::array<double, N>` 기반 POD (S1.2). 소비는 D-21, 출처는 D-22
- 기동: L3 는 새 궤적마다 깨어나는 event 구동이다 → `WaitForNextTick` 을 eventfd 대기 + 제한 시간으로 override, `JitterMeaningful()` false. eventfd 는 여러 신호가 한 번으로 합쳐지므로 깨어난 뒤 항상 최신 스냅샷을 읽는다
- 배치 (E-7, Adding a New Thread): manifest role·전 tier·profile, `gen_thread_layout.py --write`, `SystemThreadConfigs` 필드, `ValidateSystemThreadConfigs` 의 `all_configs` 고정 크기 배열·이름 목록, rt_callback 보다 낮은 우선순위 검사 (현재 `mpc.main` 에만 코드로 있음), generator 의 shield 코어 도출(누락 시 cpuset 밖 pin 이 EINVAL 로 조용히 실패), 3 oracle, launch profile 배선과 activate 게이트 테스트
- **E-7 `[CONCERN]` 에 담을 것 (S6 착수 전):**
  - 전 tier × profile 배치표. `thread_layout.yaml` 은 모든 tier 에 모든 role 의 행을 요구한다
  - tier 4 (slot 0–3 뿐): 권장 = `mpc_main` 선례처럼 slot 3 SCHED_OTHER (성능 판정 대상 아님, 개발 smoke 만). 대안 = tier 4 에서 포구 비지원 (activate 거부)
  - tier 6 은 빈 slot 5, tier 8 이상은 빈 slot 6·7. 최고 slot 이 올라가면 `test_thread_layout_tiers.cpp` 가 6-core 박스에서 상위 tier 를 skip 한다
  - `catching_on`(default)·`catching_off` profile (`drops: [<planner role>]`), `verifier_order` 등록. `mpc_on/off` 와 곱해진 4 조합이 `gen_thread_layout.py --check` 와 self-test 에 들어가는지 확인
  - profile 이 role 을 드롭할 수 있으므로 `all_configs` 조립부에 absent-role 필터를 한 번 되살린다 (없으면 zero-init 된 cpu_core 0 이 실재 slot 으로 읽혀 허위 충돌 — #349)
  - activate 게이트: DemoWbc 선례처럼 on_activate 첫 문장에서 profile 이 planner 를 드롭했는데 config 가 planner 를 요구하면 FAILURE

**복사하지 않을 것.** 현 MPC 경로는 문서상 RT 로 분류되지만 `MPCSolutionManager::PublishSolution` 의 mutex·try/catch, `HandlerMPCThread` 의 `fprintf` 가 있다 — planner 템플릿으로 쓰지 않는다 (E-9 기록 대상, §7.3).

세부 선택 D-7a~d 는 §7.

## 7. 세부 결정과 후속 결정

### 7.1 확정 (2026-09-19)

사용자 결정 (P-):

- P-1 S9 이전 개발 기간의 E-STOP 임시 기준: 해제 후 자동 재개 금지 + q_c·CLIK 앵커를 q_meas 로 reseed 만 구현 (S5.1). 임시 기준이어도 E-STOP 경로이므로 **S5 착수 전 `[CONCERN] E-8`** 을 올리고 S5 게이트에 race oracle 과 `/security-review` 를 넣는다 (§4.4 S5). 전체 정책은 S9
- P-2 `docs/dynamic_catching/` 는 브랜치 `docs/dynamic-catching-plan` 에 커밋
- P-3 Epic issue 만 생성. ball_perception 쪽 요청 이슈는 만들지 않는다 (사용자가 직접 개발 중) — 레이아웃 변경은 S5.2 파서의 필드 이름·datatype 검사와 레이아웃 해시 진단이 감지한다

세부 결정 (A-·D-7x·C-):

- D-7b slot: 빈 slot 에 새 role (dev PC tier 6 = slot 5). 제어 PC (tier 12) 의 빈 slot 이 E-core 라는 기록은 repo 사실이 아니므로 S6 에서 실측·확인
- D-7c 기동 방식: event 구동 — 새 궤적 수신 시 eventfd 로 깨우고 대기 시간 상한을 둔다. `JitterMeaningful()` false
- D-7d IK 감쇠 법칙: `DifferentialIk` 의 σ_min 적응 λ 를 수용하고 L3 G3-G (IK 수렴률)로 검증. 부족하면 그때 `DifferentialIk` 를 일반화
- D-12 추측하지 않고, 값이 준비되기 전에는 YAML 에 provisional 로 표시해 활성 구성 TBD 검사가 실기 arm 을 막게 한다
- A-1 Sprint Contract: Epic 기준 + 단계별 `[SPRINT]` (§1a)
- A-2 D-7a 판정 기준: FIFO 가 수신 → plan 게시 지연 p99 를 `planner.budget_s` 의 10% 이상 줄이거나 예산 초과율을 줄이면 FIFO 유지, 둘 다 아니면 SCHED_OTHER. 표본 ≥ 1000 시행 (L3 G3-C 와 같은 규모)
- A-3 공분산(TBD-COV-01): RT 스냅샷에서 분리, 계획기 쪽 버퍼에만 둔다 (같은 token, D-22). NaN(모름) 처리도 계획기 한 곳에서
- A-4 계획기 탐색: 1차원 시간 탐색 + IK 로 시작하되 NLP 전환을 염두에 둔 경계를 유지한다 (§8)
- A-5 DECEL 은 t_c 시각 기준 진입, 지문 센서는 결과 판정·abort 전용 (L7 §4.1 권장 채택)
- A-6 COMMITTED 이후 stale 은 동결 plan 으로 계속하고 상한 초과 시 ABORT_SAFE (L7 §4.2 권장 채택)
- A-7 → D-15 (vision 요구 사양은 제어기가 정하고 sim 을 맞춘다)
- S0.7 후속 (2026-09-19, 사용자 결정 — 근거는 §4.4 S0 결과): 지평 요구는 R1 (`io.horizon_min` 도 R1 기준, R2 는 첫 plan 실패 최악값 기록), 대기 자세는 겨냥점 근처 (구체 자세는 S3.5a), `kCap` 40 (provisional)
- D-15 sim profile 지평: 0.8 s, 간격 0.05 s, 17 점, ≤ 30 Hz (2026-09-19, S0.7 권장 채택). ball_perception sim profile 설정은 사용자가 하고, S3.4 가 실측으로 확인한다
- D-18 manipulability 정의: 팔 관절 열 5행 w₅ (병진 3 + 접근축 2), threshold 0.1 은 이 정의에 대한 값
- D-18 발사 영역: base frame 수평 거리 4 m 원호 위 (좌우 투척 포함), world z 1.5–2.0 m (사람 투척 릴리스 높이), 발사점 → 포구점 비행시간 T_f ≥ 1.0 s (2026-09-19, S0.7 결과 후 사용자 결정 — 짧은 직선 투척을 목표 분포에서 뺀다. 상한은 지도 결과)
- C-1 vision `validity`: 한 점이라도 VALID 가 아니면 메시지 전체를 거부한다. S3.4 실측에서 부분 무효가 실제로 나오면 S5.2 착수 전에 재검토
- C-2 `header.stamp` 기반 나이 거부는 두지 않는다. 원점 지연(수신 wall − stamp)은 진단으로만 남긴다 (오래된 원점은 절대 시각 기반 지평 검사가 거른다). 미래 stamp 거부는 별개 (§3.1)
- C-3 `planner.ik.manip_min` 은 D-18 게이트로 대체한다. **IK·게이트는 w₅ 로 풀고, 검증은 w₆ 로 해야 할 수도 있다** — §11 의 w₅/w₆ 병행 규칙
- C-4 포구 후보마다 IK seed 는 대기 자세로 고정한다. IK 반복·예산 증가는 S6.3 에서 측정
- E-1 (S0.6, 승인 2026-09-19): D-2 (3) 의 stamp 사용을 §3.1 문구 그대로 invariants.md §Clock 시간축 규칙의 기록된 예외로 둔다
- E-3 (S0.8, 승인 2026-09-19): D-14 (.srv)·D-20 (.msg). 규칙 텍스트는 새 인터페이스 "추가" 를 다루지 않고 선례가 갈린다 — 새 `.msg` (`f95ca5aa` PayloadEstimate) 는 E-3 을 발화·컨펌했고, 새 `.srv` (`4d98c15f` SetExternalWrench) 는 "append-only 라 ABI 파괴 아님" 으로 발화하지 않았다. 보수적으로 둘을 한 번에 발화해 승인받았다. 두 경우 모두 `PublishRole` 을 늘리지 않으므로 E-11 은 발화하지 않는다. PROC-3 은 각 변경 때 수행한다

### 7.2 D-7a 스케줄러 — RT(SCHED_FIFO) 검토, 측정으로 확정

사용자 방침: timing 이 중요하므로 RT 로 검토하고, RT 로 얻는 이득이 크지 않으면 SCHED_OTHER 로 한다.

**planner 의 timing 이 어디에 영향을 주는가.** planner 는 RT 루프와 다른 코어에서 돌고 SeqLock 으로만 결과를 넘기므로, 스케줄링 클래스는 RT 루프의 결정성에 영향을 주지 않는다. 영향을 받는 것은 **vision 메시지 도착 → plan 게시까지의 지연과 그 꼬리(p99·최대)** 이고, 이것이 commit (t_c − T_freeze) 시점의 plan 신선도와 계획 가능한 남은 시간을 정한다. D-2 에 따라 plan 의 시각은 절대 steady 시각이므로, 늦은 plan 이 틀린 시각을 쓰지는 않는다 — 늦을수록 남은 시간이 줄 뿐이다.

**지연 구성과 FIFO 가 줄일 수 있는 부분.**

| 구간 | 실행 문맥 | FIFO planner 로 개선? |
|---|---|---|
| DDS 수신 → 구독 콜백 → 파싱 → eventfd | `nrt_callback_executor` (SCHED_OTHER, 단일 스레드, lifecycle 서비스와 공유) | **아니다** — planner 와 무관한 상류 |
| eventfd → planner 깨어남 | planner 스레드 | 예. 격리된 전용 코어면 차이가 작고, 공유 코어면 크다 |
| 탐색 계산 (예산 `planner.budget_s`) | planner 스레드 | 선점(커널 스레드·다른 CFS 작업)으로 늘어나는 꼬리만 줄인다. 계산 속도 자체는 코어 종류(P/E)가 정한다 |
| SeqLock 게시 → RT 가 읽음 | RT 루프 | 최대 1 tick, 스케줄링과 무관 |

**비용.** FIFO 이면 planner 코드가 RT-1~10 에 구속된다 (execution context 가 RT 여부를 정한다). 다만 planner 는 이미 사전 할당·noexcept 로 설계하고 (§6: `RtModelHandle`, `DifferentialIk`, 참조 헤더 모두 할당·예외 없음), 진단은 SPSC 로 aux drain 에 넘기므로 **추가 비용은 작다**. 그 밖에: 한 코어에 RT role 은 하나만, OS 코어 금지 (`ValidateSystemThreadConfigs`), rt_callback 보다 낮은 우선순위 규칙은 현재 `mpc.main` 에만 코드로 있으므로 planner role 에도 같은 검사를 추가해야 한다. 개발 PC 는 PREEMPT_RT 커널이 아니고 `sched_rt_runtime_us` 950000 (95% throttle) 이라 FIFO 효과를 여기서 판정할 수 없다 — **판정은 제어 PC 에서** 한다.

**결정 방식.**

1. planner 코드는 스케줄링 클래스와 무관하게 **RT-1~10 준수로 작성**한다 (S6 게이트: `ScopedAllocGate`·`ScopedNoMalloc` 할당 0, noexcept, 로깅은 SPSC). 그러면 FIFO/OTHER 는 `thread_layout.yaml` 값으로 바뀌고 planner 코드는 바뀌지 않는다. RT 쪽 `PlanSnapshot` 읽기는 writer 가 OTHER 여도 D-21 의 측정(최악 재시도 시간)을 통과해야 한다
2. 초기값은 **FIFO** (rt_callback 보다 낮은 우선순위, 검사 추가)
3. S6 에서 제어 PC 에 부하(포구 컨트롤러 + sim 또는 실기 드라이버 + vision)를 건 상태로 두 정책을 각각 측정한다: vision 수신 → plan 게시 지연의 p50·p99·최대, 예산 초과율
4. **실제 배치 확인.** `ApplyThreadConfig` 실패는 경고만 남기고 계속 실행되므로 설정값을 증거로 쓰지 않는다. 각 run 에서 planner 스레드 이름에 맞는 모든 TID 의 policy·priority·논리 CPU·cpuset mask 를 `/proc` 에서 기록한다 (`verify_rt_runtime.sh` 에 planner 행을 더한다). 기대와 다르면 그 run 은 NOT_EVALUATED
5. **지연 기록.** 공용 timing CSV (8열) 는 한 스레드의 tick 내부 구간만 담아 수신 → 게시 지연을 표현하지 못한다. `{snapshot_sequence, recv_steady_ns, wake_ns, publish_ns}` 이벤트 레코드를 SPSC 로 따로 남긴다
6. 판정 기준 (A-2): FIFO 가 p99 지연을 `planner.budget_s` 의 10% 이상 줄이지 못하고 예산 초과율도 줄이지 못하면 **SCHED_OTHER 로 전환**. 표본 ≥ 1000 시행
7. 측정에서 상류 구간 (nrt_callback 수신)이 지배적이면 D-7e 로 수신 경로를 따로 검토한다

### 7.3 미결정

**사용자 결정이 필요한 것**

| 항목 | 내용 | 필요 시점 |
|---|---|---|
| E-8 승인 | P-1 최소 계약 (§4.4 S5) | S5 전 |
| E-7 승인 | §6 의 배치표·tier 4 정책·profile | S6 전 |
| D-24 결정 | 지문 센서 freshness 경로 (a)/(b) | S5 전 |
| D-12 손 토크 | P1b 손 관절 한계: 설정·모델값은 모두 3.0 N·m (YAML `max_torque`, URDF `effort`, MJCF `forcerange`) 이고, 1.5 N·m 는 작성 시점 사용자 진술 (CATCHING_MASTER §1.3). nominal·continuous·peak·설정값 중 무엇을 운용 한계로 쓸지와 그 출처 | S7.3 충격 게이트 전 |
| D-12 나머지 | 공 사양, 실기 T_close,tot 측정 시점, 성공률 floor·시행 수 | S4.4, S8 |
| D-3 제안값 | 시행 수 200·무효율 5 %·ε_clk 할당 비율 (§5) | S3.1a |
| d_eff | S4.5 산정값 승인 | S4.4 |
| D-18 T_f 상한 | 하한 1.0 s 는 확정. 상한(정점 높이·포구 속력이 커진다)은 S3.5a 지도 결과로 제안 | S3.5b |

**단계에서 정할 것 (결정은 해당 단계)**

- S1.7 η_v·D-16 가속 box 의 YAML 키 이름 (plan 이 이름을 정하지 않음)
- S2.2a CLIK 확장 구조 (행 선택형 vs formulation 클래스)
- S2.2 CLIK: 관절별 속도 한계 (현재 `v_limit` 스칼라), q_c 평가용 캐시 분리, q_c 모드 실패 후 재앵커 규칙, `anchor_drift_max` 와 `TRACK_ERR` 중복 정리
- S3.1a ε_clk 할당 비율 제안
- S5.2 (S3.4 측정 결과로) C-1 재검토 여부, vision 재시작 시 `snapshot_sequence` 되감김 처리, `frame_id` ↔ world, 유령 트랙 처리, 공분산의 시간 보간 정의와 nrt 파서 → 계획기 버퍼 전달 방식 (D-22 token 유지)
- S5.3 QP 비의존 관절공간 abort 식
- S6.2 `DifferentialIk` 의 σ₀·λ_max 키
- S7 homing 을 IDLE 하위 단계로 둘지 별도 Mode 로 둘지, `REF_SATURATED` 판정식, 손 hold 힘 한계를 position 목표로 표현하는 규칙, `stale_committed_max_s` 를 조일 물리량 (공분산 성장·포획 반경 오차 할당·abort 정지거리)
- TBD-WS-01 (바닥·작업셀 경계) — S3.5a catchability 지도에서 작업셀 경계를 입력으로 쓸 때 함께 정한다

**repo drift (이 작업 범위 밖 — 별도 브랜치로 처리)**

- PR [#538](https://github.com/hyujun/rtc-framework/pull/538) 로 세 항목(FingertipSensor 주석, controllers.md DemoWbc 행, p1b `index_mcp_aa_joint` 주석)이 main 에 반영됐고 (2026-09-19, 값 변경 없음), 같은 날 이 브랜치에 origin/main 을 병합해 세 항목이 여기서도 고쳐진 상태다
- (E-9) MPC 경로가 문서상 RT 로 분류되지만 mutex·`fprintf` 를 쓴다 (§6) — 문서와 코드 중 어느 쪽에 맞출지 설계 판단 필요
- (PROC-7) DemoWbc `Compute()` 의 `!target_initialized_` early-return 이 `wbc_state_lock_.Store` 를 하지 않는다
- `rtc_tools` plotting `log_type.py` 의 "unified 7-col schema" 주석 — 실제 timing CSV 는 `run_id` 가 더해진 8열
- invariants.md E-3 행이 새 인터페이스 "추가" 를 다루는지 정의하지 않아 같은 이슈(#135) 안에서 판정이 갈렸다 — 한 줄 정의 추가 후보

**기존 미결정**

- D-7e (조건부) vision 수신 경로가 지연을 지배할 때의 대안 — 7.2 측정 결과가 나오면
- D-13 E-STOP·fault 정책 — S9
- L3 후보 점수 가중치 — 튜닝, S6~S8
- 실기 공분산 검증 수단 (L8 §6 의 세 가지 중) — S10
- NLP 전환 여부 (§8 신호) — S6·S8 결과 후
- D-18 지도 도구의 초기 탐색 범위 (§11 제안: 방위 ±90°, 방향 편차 ±10°, 겨냥점 = 대기 자세 catch frame) — S3.5a 착수 전 확인

### 7.4 정합화 개정 기록 (2026-09-19)

외부 리뷰 finding 16건과 1차 리뷰를 현재 코드·규약으로 재검증했다. 반증된 finding 은 없고, 처방을 바꾼 것은 "수정" 으로 적는다.

| finding | 판정 | 반영 |
|---|---|---|
| F1 E-3 누락 | 수정 — 규칙 모호, 선례가 `.msg` 발화·`.srv` 미발화로 갈림 | D-14·D-20, S0.8, §7.1 |
| F2 E-8 시점 | 확인 — 신규 컨트롤러 E-STOP hold 회귀가 E-8 로 분류된 선례 (`aebcd8e6`) | P-1, S5 게이트 |
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

## 9. 관절 가속 한계 도출 (D-16)

가속 데이터는 없고 토크 한계는 있다: YAML `devices.<group>.joint_limits.max_torque`, URDF `effort`, MJCF `forcerange` 가 같은 값이다 (UR5e 150·150·150·28·28·28 N·m, iiwa7 200 N·m). 기존 YAML `max_acceleration` (5.0 rad/s²) 은 출처 없는 placeholder 라 쓰지 않는다.

**방법 (오프라인 도구, S2.5).**

1. 포구 작업공간·대기 자세 주변에서 관절 자세 q 와 속도 q̇ 를 표본 추출한다. q̇ 는 계획이 실제로 쓰는 속도 집합(CLIK 속도 한계 이내)으로 제한한다
2. 각 표본에서 Pinocchio 로 M(q), 중력 g(q), 속도항 c(q, q̇) 를 구한다 — 손을 포함한 결합 모델 (nv 는 §2)
3. 중력은 별도 헤드룸으로 뺀다: 관절 i 의 동적 토크 여유 τ_dyn,i = η_τ · τ_max,i − |g_i(q)| − |c_i(q, q̇)|. 모든 |q̈_j| ≤ a_j 에 대해 Σ_j |M_ij| a_j ≤ τ_dyn,i 가 성립하는 충분조건을 세우고, a = s · w 로 두어 최대 s 를 구한다 (LP)
4. 관절 가중 w 의 기본값은 균일 (w_j = 1) 이다. 대안 (τ_max 비례, 관성 대각 비례) 을 쓸 때는 계획기 도달시간 식 (L3 §4.3) 이 쓰는 한계와 같은 벡터여야 한다
5. 전 표본의 최소값을 보수적 상수 box 로 채택하고, 표본 범위·η_τ·w·모델 버전·일자·**τ_dyn ≤ 0 인 표본 비율**·관절별 binding 제약을 provenance 로 YAML 에 기록한다
6. **퇴화 시 채택 금지.** τ_dyn ≤ 0 인 표본이 있거나 s 가 0 에 가까우면 자동 채택하지 않고 사용자 판단으로 넘긴다 (표본 범위 축소, η_τ 조정, 자세 의존 한계 검토)

- 충분조건(삼각부등식)은 전 관절이 최악 부호로 동시 가속한다고 보므로 2~4 배 보수적일 수 있다. 그 영향은 S4.4 에서 받을 수 있는 공 속력과 함께 판정한다
- η_τ < 1 은 접촉 충격(L7 충격량 예산)과 모델 오차를 위한 여유다. 값은 S2.5 에서 제안한다
- 상수 box 를 쓰는 이유: 계획기(L3 도달시간)와 CLIK 가속 box 가 같은 한계를 써야 계획이 실행과 일치한다. 자세 의존 한계는 v1 범위 밖이며, RT 에서 매 tick 토크 여유를 감시하는 용도로만 검토한다
- **교차 검증 (sim) 은 iiwa7 로 한다.** iiwa7 은 `model_pairs.yaml` 게이트 안이라 MJCF 와 URDF 의 일치가 검증돼 있다. ur5e_p1b 가 로드하는 MJCF 는 형제 저장소의 사본이고 게이트 밖이며, MJCF 의 `armature`·`damping`·`frictionloss` 는 Pinocchio M·h 가 모르는 항이라 도출값을 낙관적으로 확인해 줄 수 있다. ur5e_p1b 는 모델 자기정합 오라클(τ := RNEA 로 정답 생성)로 대신 검증한다
- **실기 주의:** UR 은 position 명령을 받는 쪽 컨트롤러가 자체 가속·보호 정지 기준을 가질 수 있다 (repo 밖, 미확인). S10 에서 식별한다

## 10. catch frame YAML (D-17)

catch frame 은 모델 빌더가 추가하는 frame 이다 (D-10). 사용자가 sim 에서 확인하며 바꿀 수 있도록 로봇 config 에 연다. 로봇 config 의 `urdf.*` 는 rclcpp 파라미터로 읽히고 list-of-dict 를 담을 수 없으므로, `urdf.sub_models.<name>.*` 와 같은 **map key** 형태로 둔다.

```yaml
# 로봇 config (예: integrated_bringup ur5e_p1b _base.yaml 의 urdf 절) — 스키마
urdf:
  extra_frames:
    catch_frame:                 # map key = frame 이름
      parent: l_palm_link        # 부모 frame
      xyz: [0.0, 0.0, 0.0]       # m, 부모 frame 기준 — 포켓 중심
      rpy: [0.0, 0.0, 0.0]       # rad, 부모 frame 기준 — 결과 frame 의 +z 가 손바닥 바깥 법선
      provisional: true          # 사용자 확인 전
```

포구 컨트롤러 YAML 은 frame 이름만 참조한다 (`catch_frame: catch_frame`). 접근축은 규약상 이 frame 의 +z 다.

**전달 경로 (S2.3a).** CM 파서가 `list_parameters({"urdf.extra_frames"})` 로 이름을 모으고 (`ParseSubModels` 와 같은 방식) → `rtc_urdf_bridge::ModelConfig` 의 새 필드 → yaml-cpp `LoadModelConfig` 경로도 같은 키를 읽거나 명시적으로 거부 → `PinocchioModelBuilder` 가 `BuildFullModel()` 직후 full 모델에 frame 을 추가. sub·tree·actuated 모델은 모두 full 모델에서 `buildReducedModel` 로 만들어지므로 frame 을 상속한다. 부모 관절이 잠긴 모델에서는 frame 이 조상 관절에 붙는다. 네 모델 모두에서 frame 존재와 위치를 검증한다 (S2 게이트). 값은 모델 빌드 시 읽히므로 바꾸면 컨트롤러를 다시 configure 해야 한다.

**초기 제안값 산출.**

- 축 (S2.3a): FK 로 도출한 후보 — p1b `l_palm_link` +z (rpy 0), iiwa7_leap `palm_lower` −z (x 축 π 회전으로 +z 로 뒤집음)
- 위치 (S2.3b, S4.1 이후): 손 preshape 자세(S4.1 손 프로파일)에서 손가락 끝 위치들의 중심을 부모 frame 에 표현한 값을 포켓 중심 제안값으로 한다
- 제안값은 근거(자세·계산식)와 함께 PR 에 적고, 사용자가 sim 에서 확인한 뒤 `provisional: false` 로 갱신한다
- 검증기는 `provisional: true` 인 catch frame 으로 실기 arm 을 막는다 (D-12 와 같은 규칙)

## 11. 투척 목표와 catchability 판정 (D-18)

**판정.** 공 궤적의 포구 후보점 p_c 마다 L3 §4.2 의 포구 자세(catch frame +z = a_d = −v̂(t_c), 즉 손바닥 바깥 법선이 날아오는 공을 마주봄)를 IK 로 구하고, 그 해 q* 에서 manipulability w(q*) 를 잰다. w ≥ `planner.catchability.manipulability_min` 인 후보가 하나라도 있으면 잡을 수 있는 공이다. 이 판정은 기존 게이트(IK 수렴, 도달시간, γ 창, 정지거리)에 **추가되는 AND 조건**이다 — manipulability 만으로 시간 안에 도달할 수 있다는 보장은 없다. 그래서 지도는 kinematic 지도(S3.5a)와 전체 게이트 지도(S3.5b)를 따로 낸다.

**같은 판정을 두 곳에서 쓴다.** (1) 오프라인 catchability 지도 (S3.5a/b) 가 발사 조건을 정하고, (2) 런타임 계획기 (S6.2) 가 후보를 거른다. 둘은 S1.9 의 같은 함수와 같은 YAML 키를 써야 지도와 실제 판정이 어긋나지 않는다.

**fail-closed 수치 규칙 (NUM-7, NUM-1).**

- a_d = −v/‖v‖: `‖v‖ ≥ v_eps` 와 `std::isfinite(‖v‖)` 를 **둘 다** 검사한다. 실패는 후보 탈락(사유 코드)이며 `std::max` 류 clamp 로 덮지 않는다
- w₅·w₆: 고정 크기 분해(사전 할당 LDLT 의 D 곱, 또는 고정 크기 `JacobiSVD` 의 특이값 곱·log 곱)로 계산한다. planner 가 FIFO 이면 동적 크기 `JacobiSVD<MatrixXd>` 는 할당하므로 RT-1 위반이다. 판정은 `det > 0` 이 아니라 모든 중간값의 `isfinite` 와 `w ≥ threshold` 로 한다 — 특이 근처에서 반올림으로 det 가 음수가 되거나 NaN 이 나면 탈락이다
- 기존 `ClikReferenceGenerator::Manipulability` 는 damped (μ² > 0) 라 특이 자세에서도 w > 0 을 내고, `det > 0.0` 검사가 NaN 을 0 으로 바꾼다. 게이트로 재사용하지 않는다

**manipulability 정의 (확정).**

- Jacobian: catch frame 의 **팔 관절 열**만 쓴다. 손 관절은 손바닥 frame 에 영향이 없다 (손바닥은 폐쇄 체인 상류, §2)
- 행: 포구 과제와 같은 **5행** — 병진 3 (LOCAL_WORLD_ALIGNED) + 접근축 2 (LOCAL x·y 각속도). 손바닥 법선 둘레 회전(roll)은 포구에 무관하므로 뺀다. w₅ = √det(J₅ J₅ᵀ)
- 기존 CLIK 진단값 (`ClikReferenceGenerator::Manipulability`, 6×6 damped) 은 roll 을 포함하고 damping 이 있어 같은 값이 아니다
- **w₅/w₆ 병행 (C-3).** IK 와 게이트는 w₅ 로 푼다. 검증은 roll 까지 포함한 w₆ = √det(J₆ J₆ᵀ) (팔 열 6×6, damping 없음) 로 해야 할 수도 있으므로:
  - 지도 도구(S3.5a/b)와 런타임 계획기(S6.2)는 매 후보에서 **w₅ 와 w₆ 를 모두 계산·기록**한다 (CSV·`PlanSnapshot`)
  - 게이트 정의는 YAML `definition` (`arm_5row` 기본, `arm_6row` 선택) 으로 바꿀 수 있게 하고, **threshold 는 정의별로 따로 둔다** — 단위·차원이 달라 같은 수치를 쓸 수 없다
  - w₆ 는 IK 가 남긴 roll 에 의존한다. roll 은 대기 자세 seed + 자세 과제로 결정적으로 정해지므로 (C-4) 지도와 런타임의 w₆ 도 같은 값이 된다
  - 어느 정의로 판정할지는 S3.5a/b 지도에서 두 값의 분포와 사용자의 sim 자세 확인을 보고 정한다
- 단위가 섞여 있어 (m 와 rad) w 의 크기는 정의에 따라 달라진다. **threshold 0.1 은 위 정의에 대한 값**이고, 정의를 바꾸면 다시 맞춰야 한다

**포구 자세의 여유 자유도.** 6축 UR5e 에서 5행 과제는 1 자유도(손바닥 법선 둘레 roll)와 IK 해 가지가 남아 w 가 그 선택에 따라 달라진다. 런타임과 지도가 같은 해를 쓰도록 **대기 자세(wait_pose)에서 시작하는 같은 IK(자세 과제 포함)** 로 정한다. roll 을 w 최대화로 고르는 방식은 v1 범위 밖 (필요하면 S8 이후).

**frame 규약 (함정 주의).**

- "arm base frame" 은 로봇 config 의 CLIK `base_frame` 이다: ur5e_p1b `base` (URDF `base`), iiwa7_leap `link_0`
- ur5e_p1b 에서 URDF `base` 와 `base_link` 는 z 축 둘레 180° 차이다. `base_link` 로 두면 +x 가 반대가 되어 공이 등 뒤에서 날아온다 — 그래도 그럴듯한 결과가 나오므로 조용히 틀린다. sim 에서는 MJCF 의 로봇 body 가 world 에 180° z 회전으로 놓여 있다
- 발사 높이 z 는 world 기준이고 거리는 base 기준이다. world ↔ base 변환은 가정하지 않고, **같은 q 에서 MuJoCo FK 와 Pinocchio FK 를 대조**해 S3.2 에서 확정한다 (S3.5a 의 선행). 확정 전까지 아래 키는 이름에 프레임을 붙여 섞이지 않게 한다

**YAML (제안).**

```yaml
planner:
  catchability:
    definition: "arm_5row"          # arm_5row (기본) | arm_6row — 게이트에 쓸 정의
    manipulability_min:
      arm_5row: 0.1                 # provisional — 사용자가 sim 에서 자세 확인 후 갱신
      arm_6row: TBD                 # w₆ 로 판정할 때. S3.5a/b 지도 결과로 제안
sim:
  throw_region:                     # S3.5a/b 지도 도구·발사 설정 입력
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

**발사 조건 공간.** 발사점은 방위 φ 와 높이 z 로, 발사 속도는 크기·앙각·수평 방향으로 정한다. 수평 방향은 "발사점에서 겨냥점(aim point)을 향한 방향 + 편차" 로 두어, 사람이 로봇 쪽으로 던지되 좌우로 빗나가는 투척을 표현한다. 포구 후보는 비행시간 T_f ≥ 1.0 s 인 것만 남긴다 (D-18) — 항력 포함 기준 T_f 1.0 s 에서 앙각 40–53°, v₀ 4.7–5.9 m/s, 정점 z 2.2–2.8 m 이고 T_f 가 길수록 모두 커지는 lob 이다 (S0.7 결과). 지도 도구의 초기 탐색 범위 (제안): φ ∈ [−π/2, π/2] (로봇 정면 반원), 편차 ∈ [−10°, +10°] — 탐색 범위일 뿐이며, 잡을 수 있는 범위는 지도 결과로 정한다.

**지도 도구 출력 (S3.5a/b).** 발사 속도·각도 격자별로 (a) 포구 가능 여부, (b) 최대 w 와 그 후보의 t_c·p_c·q*, (c) 탈락 사유(IK 실패·w 미달·비유한 입력·도달 불가·γ 창 없음)를 표로 내고, 잡을 수 있는 발사 조건 범위를 `sim.throw_region` 의 방위·속도·앙각·방향 편차로 제안한다 (방위별 포구 가능 구간 그림 포함). 사용자가 sim 에서 q* 자세를 보고 threshold 를 갱신하면 지도를 다시 돌린다. 4 m 는 vision 지평 요구(S0.7)와 가속 box(S2.5)를 동시에 가장 어렵게 만드는 설정이다 — 둘이 타협을 요구하면 되돌아오는 값이 거리다.

## 12. 알려진 위험

| 위험 | 닫는 단계 |
|---|---|
| vision 토픽이 stable ABI 가 아니다 (D-4) | S5.2 레이아웃 해시 진단으로 감지, 제품 ABI 는 ball_perception |
| vision 지평이 짧으면 계획 가능한 포구 창이 준다 — S0.7 로 sim profile 을 0.8 s 로 올렸고 (D-15) 지평 요구는 연속 재계획 기준 R1 이다. 첫 목표가 작업공간 가장자리라 그 후보가 전멸하면 정지 출발(R2, 먼 q* 는 약 1.0 s)로 떨어진다. 지평을 늘릴 주체는 외부 repo 이고 요청 채널이 없다 (P-3) | S3.5a 가장자리 catchability, S8 첫 plan 측정, S3.6 |
| C-1 전체 거부가 포구 직전 기아가 될 수 있다 (지평 끝이 지면·작업셀 밖에 닿을수록 무효 점이 생김) | S3.4 validity 히스토그램 → S5.2 착수 전 재검토 |
| 실기 시계 오프셋이 미래 방향이면 지평 검사는 통과하고 t_c 만 늦어진다 (6 m/s·10 ms = 6 cm). L3 §4.6 의 ε_clk 는 분산 항이라 bias 를 모델링하지 않는다. sim 은 같은 호스트라 드러나지 않는다 | S10 (D-2 예외 ③ 조건, TBD-NET-01) |
| sim T_close 는 MJCF 게인에 의존 — 실기 측정 전까지 S4 결론은 잠정 | S4.3, S10 |
| γ derate 제외(D-8)로 abort 가 늘 수 있다 | S8 측정 |
| D-3 이 검증에서 떨어지면 S3·S5 시간 경로 재작업 | S3.1a, S3.1b |
| 토크에서 도출한 보수적 가속 box (D-16) 가 받을 수 있는 공 속력을 낮출 수 있다 | S4.4 |
| 1차원 분해가 복잡한 경우를 놓치면 NLP 전환 (§8) 이 필요해 S6 재작업 | S6·S8 신호 |
| S2.2 CLIK 확장이 출하 컨트롤러 DemoWbc 를 회귀시킬 수 있다 | S2.2a golden-vector, S2.4 |
| D-13 을 S9 로 미뤄 S5~S7 의 abort·FAULT·손 유지 경로가 정책 확정 때 바뀔 수 있다 | S5 E-8 최소 계약으로 범위를 좁히고 S9 에서 재통과 |
| UR 벤더 position 컨트롤러 자체의 가감속·보호 정지 | S10 |
| w₅·w₆ 를 매 후보 계산하면 계획 예산을 먹는다 (L3 §4.8 은 이미 coarse-to-fine 이 필요하다고 본다) | S6.3 예산 측정 |
| 브랜치 장기 체류로 main 과 어긋난다 (#538 로 1회 발현, 2026-09-19 병합으로 해소) | 단계 착수 때마다 main 병합 |

## 13. GUI·plot 단계별 구현과 확인 (D-19)

단계마다 그 단계가 만든 상태·로그를 **`demo_controller_gui` 에서 보이게 하고, CSV 를 `plot_rtc_log` 로 그릴 수 있게** 한다. 둘 다 단계 게이트에 들어간다.

**면제.** S0 은 코드가 없다. S1 은 ROS·GUI 비의존 순수 코어이고 실행 산출물이 GTest 뿐이라 GUI 에 붙일 상태도, `plot_rtc_log` 가 읽을 CSV 도 만들지 않는다. S1 코드가 만드는 값은 S5 이후 CSV·상태 메시지로 처음 드러나며 그 단계의 행이 확인한다.

**기존 패턴 (따른다).**

- GUI (`integrated_bringup` demo_gui): 컨트롤러 목록은 `/rtc_cm/list_controllers` 로 자동 발견된다. 새 컨트롤러는 목표 형태·게인 스키마 (`GAIN_DEFS` 등, config 모듈) 와 상태 메시지 패널을 추가한다 — DemoWbc 는 `WbcState`, grasp 는 `GraspState` 를 구독한다. 회귀는 integrated_bringup 의 `test_demo_gui_*` 테스트
- plot (`rtc_tools` plotting): 새 CSV 는 파일명·컬럼으로 종류를 판별하고 (log_type 모듈), 전용 plotter 와 pipeline 등록을 더한다 — `wbc_diag`·`compliance_diag` 가 선례. 회귀는 rtc_tools 의 `test_plot_rtc_log.py`. 스레드 timing CSV 는 공통 8열 스키마라 timing plotter 를 재사용한다 (수신 → 게시 지연은 별도 이벤트 레코드, §7.2)
- GUI 육안 확인은 `verify` skill 의 sim 절차를 따른다

**단계별 항목.**

| 단계 | GUI | CSV · plot |
|---|---|---|
| S2 | 변경 없음 (DemoWbc 패널이 CLIK 옵션 off 에서 그대로 동작하는지 확인) | `wbc_diag` 플롯이 그대로인지 회귀 확인. CLIK 새 진단(status·반복·solve time·`bound_conflict`)을 기존 CSV 에 더하면 plotter 반영 |
| S3 | sim 공 발사(D-14 srv) 버튼·발사 조건 입력, 공 상태(ground truth·vision 예측 수신 여부) 표시 | clock 위상 오차 CSV (δ·pause, §5), catchability 지도 결과 (w₅·w₆ 분포, 방위별 포구 가능 구간) 는 지도 도구 자체 플롯 |
| S4 | 손 step 명령·T_close 식별 실행 | T_close 식별 CSV → ρ(t)·T_close 분포 플롯 |
| S5 | 포구 컨트롤러 패널: 모드·입력 상태(n·generation·수신 나이·지평), 기준 vs 실제 추종 오차, CLIK 상태, arm/disarm | `catching_diag.csv` (tick 별: 입력 스냅샷 token, L4 기준, CLIK 상태, q_c vs q) → 새 plot 종류 + 회귀 테스트 |
| S6 | plan 표시: t_c·p_c·γ_f·w₅·w₆·탈락 사유·plan 나이 | `planner_timing_log.csv` (timing plotter 재사용), plan 이벤트 CSV (후보별 게이트 결과, 수신 → 게시 지연) → plot |
| S7 | 슈퍼바이저 모드·사유·결과, 손 위상, 접촉 센서·센서 freshness | 전이 로그·손 위상·접촉 CSV → plot (전이 시각선을 추종 플롯에 겹침) |
| S8 | 연속 투척 진행 표시 | 시행 요약 CSV → 로봇별 성공률 (Wilson 구간, 전체·무효 발사 수)·NEES·소거실험 플롯 (오프라인 평가 스크립트) |
| S9 | E-STOP·fault 상태·해제 흐름 | E-STOP/fault 사건이 CSV 와 플롯에 드러나는지 |
| S10 | 실기 모드 표시 (provisional 값 차단 상태 포함) | 실기 세션 CSV 가 같은 plot 으로 그려지는지 |

**게이트 공통.** 해당 단계의 (1) GUI 패널이 sim 에서 값을 표시하고 조작이 컨트롤러에 반영됨 (육안 확인 + `test_demo_gui_*` 추가), (2) 새 CSV 가 `plot_rtc_log` 로 파싱·플롯되고 `test_plot_rtc_log.py` 에 회귀 케이스가 있음.

**포구 상태 메시지 (D-20).** `rtc_msgs` 에 포구 상태 메시지를 새로 추가해 GUI 가 구독한다 (`WbcState`·`GraspState` 선례, controller-owned `SeqLock<T>` + 전용 publisher, `PublishRole` 을 늘리지 않음 — E-11). Adding a New Message Type 절차, S0.8 E-3 승인 후, PROC-3 전체 빌드·테스트. 필드(모드·사유·입력 상태·token·plan·w₅/w₆·손 위상·센서 freshness·결과)는 **S5 에서 S5~S9 superset 으로 한 번에 동결**하고 이후 단계는 값만 채운다. 모든 `Compute()` tick 에서 Store 하고 그 tick 에 계산하지 않은 필드는 무효화한다 (PROC-7).

## 14. 추적 매트릭스

### 14.1 규약 → 단계 게이트

| 규약 | 단계 | 게이트 항목 |
|---|---|---|
| E-1 | S0.6 → S1.3·S5.2 | §3.1 예외 승인 전 D-2 변환 함수 착수 금지 |
| E-3 | S0.8 → S3.2 (D-14)·S5.4 (D-20) | 승인 기록 없으면 `rtc_msgs` 변경 금지 |
| E-6·PROC-6 | S2 | 기존 assertion 수정 필요 시 즉시 중단 |
| E-7 | S6 착수 전 | §6 배치표 `[CONCERN]`, S6 스레드 게이트 |
| E-8 | S5 착수 전·S5 게이트·S9 | P-1 최소 계약, race oracle, `/security-review` 2회 (S5 최소 정책, S9 최종) |
| E-9 | §7.3 | MPC RT 분류 drift |
| E-11 | S5.4 | `PublishRole` 을 늘리지 않음 |
| PROC-3 | S3.2·S5.4·(D-24 (a) 시 S5.2e) | `rtc_msgs`·`rtc_base` 변경 후 전체 빌드·테스트 |
| PROC-7 | S5·S7 | early-return 분기별 Store 실패 경로 테스트 |
| ARCH-3 | S1.9·§8 | 구현 하나뿐인 interface 신설 없음 |
| ARCH-6 | S5.2 | 구독 `KEEP_LAST(1)` (hook Phase 0b), backlog latest-only 테스트 |
| RT-1~10 | S1·S5·S6·S7 | G0-B·G1-D·G2-E·G4-G·G5-C·G3-K·G7-D 할당 0 |
| NUM-1·NUM-7 | S1.9·S1.5 | a_d guard, w fail-closed, 한계 무효 flag |

### 14.2 설계 문서 구현 단위 → 단계

| 단위 | 단계 |
|---|---|
| L0 시간 타입·궤적 POD·검증기·fixture | S1.3·S1.2·S1.7·S1.6 |
| L0 공 항력 k 식별 | S3.8 |
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
| L5.7 지연 식별 도구 | S3.7 |
| L5.8 선행 보상 | S5.3 |
| L5.9 실기 식별 | S10 |
| L5.10 backend 왕복 | S5.3 |
| L6.1·L6.2·L6.5 | S4.1·S4.2·S4.3 |
| L6 d_eff 산정 (§4.5) | S4.5 |
| L6.3 go/no-go | S4.4 |
| L6.7·L6.6 | S7.1 |
| L7.1–L7.3 | S1.8 (L7.3 은 S7.3 에서 결합) |
| L7.5·L7.6 | S7.2·S7.3 |
| L7.7 | S10 |
| L7.8 E-STOP 훅 | S5.1 (P-1) → S9 |
| L8.2 골격 | S4.0 → S5.1 |
| L8.3 기록·상태 publisher | S5.4 |
| L8.4 D-3·발사 srv·truth | S3.1a·S3.2·S3.3 (부하 재검증 S3.1b) |
| L8.5 vision 연결·지도·사양·NEES | S3.4·S3.5a/b·S3.6 |
| L8.6·L8.7 폐루프 평가 | S8 |
| L8.8 실기 | S10 |
