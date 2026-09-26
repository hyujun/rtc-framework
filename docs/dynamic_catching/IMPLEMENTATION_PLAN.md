# dynamic_catching — 전체 구현 계획 (living document)

- 상태: **S8 진행 중 — S8-A·S8-B·S8-C·S8-D·S8-E·S8-F-1 완료** (S8-F-1: 2026-09-26, PR #583 → `3278963a` — 손 근처 투척 탐색 1차, 1068 발: 정면 v50(r 0) **3.6 m/s** [3.0, 4.0], commit 99.9 % 인데 게시 plan 의 93–100 % 가 도달 순위 gate 실패, 도달 우선 점수 A/B 효과 없음 (p 0.58), beanbag 38/56 — 막는 것은 팔 속도 한계 (`v_max` → γ 하락 → v_rel) 와 손 반발; §4.4 S8-F-1 결과. **후속 순서 (2026-09-26 사용자 결정, 권장 순서 채택 — #537 5846528236)**: ② sim 팔 예산 `reference.v_max`·`a_max` 실측 → ① γ 창을 실측 추종 능력으로 재정의 (도달 gate 판정 승격은 비권장) → ④ S8-F-2 wait_pose 변경 → ③ 손 반발 흡수 (실기 검증 항목 동반); 다음 = ② `[SPRINT]`) (S8-E: 2026-09-26, PR #582 → `9e7967b5` — 본 평가 600 발, 무효 0: **G8-D (p1b tennis) PASS 93/200** (Wilson 97.5 % 단측 하한 0.3972, D-S8-17 host 부하 unit 재실행 반영; 사전 규칙 원 판정 83/200 · 0.3489 FAIL 병기) · beanbag 175/200 PASS · **G8-D2 (leap) FAIL 83/200** (0.3489) · G8-B·G8-C2 (= G3-H)·G8-H FAIL · G8-A `NOT_EVALUATED(제어 PC)`; #537 계획 5841651825 → 결과 5842632283 → D-S8-17 5843117293 → 재실행 5843232090; §4.4 S8-E 결과); S8-D: 2026-09-26, PR #581 — `iiwa7_leap` sim 폐루프, G8-D2 평가 대상 44/100, G3-D 기록, 계획 밖 판단 3 건 사후 승인 #537 5841231282; §4.4 S8-D 결과; S8-B: 2026-09-24, PR #575 → `ac34893a` — 측정 기반, p1b sim 팔 서보 τ 0.05 s, lead 보상 2×2, 슈퍼바이저 임계, floor 0.35; S8-C: 2026-09-25, PR #579 → `126e3513` — RETREAT 손 `T_release_timeout`, 손 관절 캡처 판정 근거 (G7-E 재기록 92/100); §4.3 S8 행·§4.4 S8-B·S8-C 결과, 다음 S8-D), **S7 완료** (2026-09-24, PR #571 → `c61fd32a`), **S6 완료** (2026-09-23, PR #568 → `9bf404a8`), **S0 완료**, **S1.1~S1.8 완료** (2026-09-19, PR #541), **S2 완료** (2026-09-20, PR #545~#549; **S2.3b 2026-09-21**), **S1.9 완료** (2026-09-20), **S3a 완료** (2026-09-20, PR #553), **S4a 완료** (2026-09-21, 브랜치 `feat/s4a-hand-timing` — S4.3 실기는 S10). **S3.5a 완료** (2026-09-21 — 도구 + 두 로봇 지도, PASS(provisional); §11 지도 결과). **S3.5b 완료** (2026-09-22 — `ur5e_p1b` PASS(provisional), `iiwa7_leap` 지도 빔; §4.4 S3.5b 결과). **T_det 실측 완료** (2026-09-22 — 24 비행 × 2 run; 첫 run 이 sim 공 lane 의 stamp 지터 결함을 드러내 **같은 날 고쳤다** (stamp = 발사 기준 sim 시간축을 wall 에 얹은 값, `rtc_mujoco_sim`) — 재실측 stamp 축 p50 0.050 · max 0.100 s, 첫 계획 시각 max 0.24 s 로 전 비행이 지도 가정 안; `iiwa7_leap` G8-D2 평가 대상 — §4.4 T_det 실측·재실측). **S3.6 완료** (2026-09-22 — vision 요구 사양 provisional: **기구학 reachable 창 기준** (D-27) + **T_det 재실측** H_req **0.99 s** · 간격 0.05 s · `n_max` **20** ≤ `kCap` 40 (S1.2 backfill PASS) · `io.horizon_min` 0.51 s; sim profile **1.0 s / 20 점 설정** (`integrated_bringup/config/ur5e_p1b/ball_perception_sim_profile.json`, 사용자 결정); §4.4 S3.6 결과·T_det 재실측). **S5 완료** (2026-09-23, PR #564 → `4c0fb751` — 포구 컨트롤러 골격·입력·추종, 상태 메시지·CSV·GUI, sim 폐루프; §4.3 S5 행). **S5 앞 결정 6건 확정** (2026-09-22 사용자 — E-8 승인 · D-24 (a) · G5-E fixture 주입 · QP solve time 예산 · 공 lane stamp 수정 · sim profile; §7.3). **S4.4 완료** (2026-09-22 — **조건부 go**: 현 sim 씬·D-18 투척에서는 두 로봇 모두 γ 창이 비고, 낙차·가속 한계·선행시간 세 조건 아래에서만 열린다; D-16·D-18 개정은 `[제안]`; §4.4 S4.4 결과·§9·§11). S0 은 S0.1~S0.9 게이트 PASS. S0.7 이 0.5 s profile 부족을 보고해 sim profile 지평을 0.8 s 로 정했고 (D-15), 지평 요구는 R1·대기 자세는 겨냥점 근처·`kCap` 40 으로 정했다 (§7.1). E-7 (2026-09-23, 결정 J) 과 E-8 (2026-09-22, S5 에서 구현) 은 승인됐다. 승인이 막는 단계는 승인 전에 착수하지 않는다 (§4.1)
- 최종 갱신: 2026-09-26 (S8-F-1 머지 PR #583 + 후속 순서 결정 ②→①→④→③ — 상태 줄, §4.3 S8 행, §4.4 S8-F-1 결과 후속, §7.3 `a_max` 행; S8-F-1 완료 — 상태 줄, §4.3 S8 행, §4.4 S8-F 재도출·S8-F-1 결과, L6 §4.5 sim 입력; S8-E 완료 — 상태 줄, §1a 판정, §4.3 S8 행, §4.4 D-S8-17·S8-E 결과·게이트 표 판정어, §7.3 L3 가중치·NLP·`a_max`·`sigma_trk` 행 (제안만), §12 위험, §13 S8 행, §14.2 L7.6·L8.4·L8.6; S8-E 계획 확정 — 상태 줄, §1a floor 동결, §4.3 S8 행, §4.4 D-S8-16·S8-E 계획·게이트 표 성공률/clock/이월 행, §7.3 D-12·3-3 행; S8-D — 상태 줄, §4.3 S8 행, §4.4 S8-D 결과·D-S8-2 개정·D-S8-14/15 (상자 유효성 조건 사후 승인)·이월 표 G3-D, §7.3 3-3·임계값 행·`REF_SATURATED` 행 leap; 2026-09-25: S8-C 완료 — 상태 줄, §4.3 S8 행, §4.4 S8-C 결과, D-S8-6·D-S8-8 재기록, §7.3 RETREAT 손 대기 timeout, 이월 표 G7-E 정리; 2026-09-24: S8-B 마감 — §1a·D-S8-3 floor 0.35, §4.3 S8 행, §4.4 S8-B 결과·D-S8-9 d_eff 서술, §7.3 `sat_ticks`·`track_err_abort`·3-3; 2026-09-22: S3.5b gate 지도 §4.3·§4.4·§7.3, S4.4 정정 §4.4; S4.4 조건부 go §4.3·§4.4·§5.1·§7.3·§9·§11, 시각 발동 fly-in·실기 모터 추정 L6 §4.2·§4.5; 2026-09-21: P1b 손 자세 탐색·S4.5 재실행·D-3 P1b 판정 §4.4 S4a·§5.1; S4a 착수 전 코드 대조·범위 결정 §4.4 S4a; S3a 게이트 결과 §4.4 S3a·§5.1·§11, D-3 ε 하한 §7.3)
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
| D-3 | sim 시간축: wall clock 유지 + 시행별 clock 오차 게이트 (§5) | **채택 — 판정식 재정의, 검증 후 재검토** | D-2 변환이 실기와 같은 경로로 동작. `/clock` 방식은 RT 루프에 sim 전용 시간 원천이 필요해 D-2 와 충돌. 2026-09-22 부터 sim 공 lane 의 stamp 는 **발사 순간을 기준으로 sim 시간축을 wall 에 얹은 값** (epoch 은 wall 그대로, wall 과의 차이 = 비행 안 위상 오차 δ 양방향, stepper 의 wake 지터만 제거 — `rtc_mujoco_sim` README §Projectile Ball stamp) 이고, D-3 의 위상 오차 정의는 그대로다 |
| D-4 | vision 입력: ball_perception 의 실제 PointCloud2 레이아웃 채택, 필드 이름으로 파싱, `generation`·`validity`·`snapshot_sequence` 사용, NaN 공분산 = 모름. 구독 QoS 는 `KEEP_LAST` depth **1** 고정 (ARCH-6), reliability 만 S3.4 에서 실측해 정한다 | **확정** | 실제 발행기 존재. 설계 문서 §5 의 372 B·`t`·`cov` 가정 폐기. depth 는 ARCH-6 의 강제 사항이라 TBD 대상이 아니다 |
| D-5 | CLIK: `rtc::tsid::ClikReferenceGenerator` 를 옵션(기본 off)으로 확장. 옵션을 넣기 전에 기존 동작 golden-vector 회귀를 먼저 만든다 (S2.2a) | **확정** | P5 일반화. off 시 기존 출력 bit-identical, 기존 assertion 수정 필요 시 즉시 E-6 |
| D-6 | CLIK 오차·J 를 명령값 q_c 에서 평가하는 옵션 추가 | **확정** | 측정 q 평가는 servo 지연을 루프에 품고 선행 보상과 이중 보상. 실추종은 `TRACK_ERR` 로 별도 감시, 재앵커 시점 규정 필수 |
| D-7 | 계획기 스레드는 **기존 MPC 스레드와 같은 생성 방식**으로 만들고 기능만 planner 로 한다 | **확정** — D-7b~d 확정, D-7a 는 RT 준수 코드 + 측정으로 확정 (§6, §7.2) — 측정은 2026-09-23 사용자 결정으로 생략, 초기값 FIFO 유지. 배치는 S6 착수 전 E-7 승인 | nrt_callback executor 는 단일 스레드라 계획 계산(수십 ms)을 올리면 궤적 수신·서비스가 막힌다 |
| D-8 | γ derate 는 v1 에서 제외. 실행 중 포화 → COMMITTED 전 RETREAT, 이후 ABORT_SAFE. S8 에서 포화 빈도 측정 후 재설계안 도입 여부 결정 | **확정** | 참조 구현 probe: Frozen 분기 γ_min 미보장, 완화 분기 무효, 기본 램프가 가속 피크를 키움, 램프가 t_c 초과 가능 |
| D-9 | `gammaWindow` 의 TCP 속도 = η_v · `reference.v_max` (0 < η_v ≤ 1). 마스터 §6 교차제약을 이 식으로 수정 | **확정** | 계획이 한계 끝을 쓰면 실행 중 예측 변화로 포화. D-8 로 derate 가 빠져 유일한 완충 |
| D-10 | catch frame: 후보 p1b `l_palm_link` +z, iiwa7_leap `palm_lower` −z (FK 도출, 확인 필요) + 포켓 중심 offset. `rtc_urdf_bridge` 모델 빌더가 로봇 config 의 `urdf.extra_frames.<name>` 을 **full 모델에** frame 으로 추가하고, 파생 모델(sub·tree·actuated)은 그것을 상속한다 (§10) | **확정** (값은 D-17) | CLIK 은 모델 frame id 만 받는다. binding 로컬 offset 은 CLIK 까지 못 간다. 파생 모델은 모두 full 모델에서 `buildReducedModel` 로 만들어지므로 한 곳에서 추가하면 된다 |
| D-11 | 손 명령 포트 추상화 폐기. 손은 `ControllerOutput` 의 손 device slot 에 직접 기록. T_link 분리 측정 대신 종단 간 T_close,tot 실측 | **확정** | P1b·LEAP 모두 이미 device group. `udp_hand_node` 는 명령 stamp 를 읽지 않는다 |
| D-12 | 사용자 제공 값: 공 사양, 실기 T_close,tot 측정 시점, 성공률 하한·시행 수, **P1b 손 관절 운용 토크 한계의 권위 출처** (§7.3). 투척 목표는 D-18, 관절 가속 한계는 D-16, catch frame 은 D-17 로 대체 | **방식 확정, 값 대기** (성공률 하한 **0.35** (0.5 provisional 에서 S8-B 뒤 하향)·시행 수 n_valid 200 은 2026-09-24 회신 — D-S8-3) | 추측 금지. 임시값은 YAML 에 provisional 표시, 값에 의존하는 게이트는 NOT_EVALUATED (§4.1) |
| D-13 | E-STOP·fault 정책 (E-8) | **보류 — 최종 정책은 S9** (§4). S5 의 최소 계약은 P-1 | 사용자 결정 |
| D-14 | 공 발사 API: (p0, v0, ω) 명시 srv 를 `rtc_msgs` 에 추가 (Adding a New Message Type, PROC-3) | **확정** — E-3 승인 (2026-09-19, S0.8) | 파라미터 설정 + Trigger 는 경합·재현성 약함. E-3 판단은 §7.1 |
| D-15 | vision 예측 사양(지평·간격·점 수·발행률)은 **포구 제어기가 요구 사양을 정하고**, sim 에서는 공 투척 설정과 ball_perception sim profile 을 그 요구에 맞춰 설정한다. 제어기는 수신 궤적의 지평이 요구보다 짧으면 계획 후보에서 제외·진단한다 | **확정** | ball_perception 은 sim 이 주는 위치로 미래 궤적을 만드는 노드이고 사용자가 직접 설정한다. 기존 예시 profile 은 지평 0.5 s, 간격 0.05 s, 최대 10 점, ≤ 30 Hz 였고, S0.7 결과로 **sim profile 목표를 지평 0.8 s, 간격 0.05 s, 16 점, ≤ 30 Hz 로 정했다** (사용자 결정 2026-09-19; 점 수는 2026-09-20 에 17 → **16** 으로 정정 — 예측점은 `step, 2·step, …, horizon` 이라 **t = 0 을 포함하지 않는다**. 지평 0.05…0.80 s). 최종 요구는 S3.6 이 목표 투척 분포에서 다시 산출한다 — **S3.6 결과 (2026-09-22, provisional)**: 기구학 reachable 창 기준 (D-27) 과 **T_det 재실측** (공 lane stamp 수정 후) 으로 H_req **0.99 s** → **1.0 s / 0.05 s / 20 점 / ≤ 30 Hz**. 종전 0.8 s / 16 점은 창 끝 0.78 s 까지만 담아 늦게 잡는 후보를 계획기에 올리지 못한다. **설정됨 (2026-09-22 사용자)**: `integrated_bringup/config/ur5e_p1b/ball_perception_sim_profile.json` — §4.4 S3.6 결과·T_det 재실측 |
| D-16 | 관절 가속 한계는 **토크 한계에서 도출**한다 (§9). 시뮬레이션 추정은 교차 검증용. YAML 의 기존 `max_acceleration` 값은 쓰지 않는다 | **확정** — 도출 절차의 퇴화 분기·가중·오라클은 §9 | 가속 데이터 없음, 토크 데이터 있음. 기존 `max_acceleration` (5.0 rad/s²) 은 CM 이 읽기만 하고 어떤 컨트롤러도 쓰지 않는 placeholder |
| D-17 | catch frame 의 부모 frame·위치 offset·자세는 **YAML 로 열어 둔다**. 초기값은 S2.3a(축)·S2.3b(위치)에서 제안하고, 사용자가 sim 에서 확인해 실제 값으로 갱신한다 (§10). 값은 모델 빌드 시 읽히므로 바꾸면 컨트롤러를 다시 configure 해야 한다 | **확정** | 사용자 결정 |
| D-18 | 투척 목표는 **arm manipulability 기반 포구 가능성(catchability)** 으로 정한다. 발사 영역 (arm base frame 기준 수평 거리 √(x²+y²) = 4 m 의 원호 — 좌우로 흩어진 투척 포함, world z 1.5–2.0 m = 사람이 손으로 던지는 높이, **비행시간 T_f ≥ 1.0 s** — 사용자 결정 2026-09-19, S0.7 후) 에서 출발한 궤적 위 포구 후보마다, 손바닥 +z 가 공 진행 방향을 마주보는 자세(a_d = −v̂)의 IK 해에서 manipulability 를 재고, threshold 이상인 후보가 있으면 잡을 수 있는 공, 없으면 포기. 이 판정으로 투척 속도·각도 범위를 정한다. threshold 초기값 0.1 (provisional, 사용자가 sim 에서 자세를 보고 갱신) | **확정** — 정의 세부는 §11 | 사용자 결정 |
| D-19 | 단계마다 **`demo_controller_gui` 갱신과 `plot_rtc_log` 로 CSV 플롯을 구현·확인**한다. 각 단계 게이트에 GUI 확인과 plot 회귀 테스트를 포함한다. S0 (코드 없음)·S1 (ROS·GUI 비의존 순수 코어, 실행 산출물은 GTest 뿐) 은 면제한다 (§13) | **확정** | 사용자 결정. 면제 근거는 §13 |
| D-20 | 포구 상태는 `rtc_msgs` 에 **새 상태 메시지**를 추가해 GUI 로 보낸다 (`WbcState`·`GraspState` 선례, `PublishRole` 을 늘리지 않는 controller-owned `SeqLock<T>` 패턴). **S5 에서 S5~S9 필드 superset 을 한 번에 동결**하고 이후 단계는 값만 채운다. 모든 `Compute()` tick 에서 Store (PROC-7) | **확정** — E-3 승인 (2026-09-19, S0.8) | 사용자 결정. 필드를 단계마다 더하면 매번 `rtc_msgs` 변경·PROC-3 전체 빌드·테스트가 반복된다 |
| D-21 | SeqLock 소비 계약: RT 소비자는 매 tick `Load()` 를 무조건 한 번 하고, 새 스냅샷 여부는 **payload 안의** `snapshot_sequence`·provenance token (D-22) 으로 판정한다. `SeqLock::sequence()` 와 `Load()` 를 따로 읽어 짝짓지 않는다. 스냅샷 용량은 컴파일타임 상수 `kCap` (S1.2) | **확정** (§7.4 F4) | 두 호출 사이에 writer 가 끼면 옛 payload 와 새 sequence 가 짝지어져 최신 스냅샷을 놓친다. repo 의 다른 SeqLock 소비자는 모두 무조건 `Load()` 관용구를 쓴다. 비-RT writer → RT reader 는 backend 3종이 관절 상태에 이미 쓰는 경로이며, 최악 재시도 시간은 G1-C 로 측정한다 |
| D-22 | provenance token: 궤적 스냅샷·공분산 버퍼·`PlanSnapshot` 은 같은 identity `{activation_generation, generation, snapshot_sequence, traj_recv_ns}` 를 싣고, `PlanSnapshot` 은 여기에 계산 기준 `{rt_iteration, rt_state_ns}` 와 `publish_ns` 를 더한다. 계획기는 계산 시작과 게시 직전에 최신 token 을 다시 보고, 대체되었거나 짝이 안 맞는 결과(궤적 N ↔ 공분산 N−1 포함)는 버린다. RT 소비자는 token 의 activation·generation 일치, `snapshot_sequence` 단조, source 나이·state 나이 상한을 fail-closed 로 검사한다 | **확정** (§7.4 F5) | `PlanSnapshot` 에 출처 필드가 없고 궤적·공분산이 다른 버퍼로 가서 N/N−1 혼합을 막을 수단이 없었다. MPC 선례(`MPCSolution::timestamp_ns`)보다 넓은 이유: 포구는 절대 시각 판정이라 출처 나이가 곧 안전 조건이다 |
| D-23 | activation 경계: vision ingress 와 계획기는 base 의 `ActivationGeneration()` 을 스냅샷에 싣고, RT 소비는 `IsCurrentGeneration()` 이 아니면 무효로 본다. `PeriodicRtThread::Pause()` 는 진행 중 iteration 을 멈추지 않으므로 quiescence 를 기다리지 않고 generation 으로 판정한다. lifecycle·E-STOP 훅은 atomic 요청(또는 epoch)만 갱신하고, plan·궤적·공분산·손·FSM·타이머 무효화는 **RT tick 이 유일 writer** 로 수행한다 | **확정** (§7.4 F6) | lifecycle 은 publisher 만 게이트하므로 컨트롤러 소유 구독은 비활성 중에도 산다. base target mailbox 의 generation 게이트는 그 mailbox 에만 적용된다 (`rt_controller_interface.hpp`) |
| D-24 | 지문 센서 freshness: (a) **권장** `rtc_base` `DeviceState` 센서 lane 에 `recv_steady_ns`·`sequence`·`valid` 를 추가하고 backend 3종이 채워 `ControllerState` 로 전달 (PROC-3, P5 — 같은 gap 이 grasp 에도 있다), (b) 포구 컨트롤러 소유 mailbox 로 센서 토픽을 따로 구독 | **확정 (2026-09-22 사용자): (a)** — 배선은 S5.2e, PROC-3 전체 회귀 포함 (§7.3) | 현재 `last_state_ns_` 는 관절 상태 콜백에서만 갱신된다. 관절이 fresh 한 채 센서만 멈추면 옛 힘을 새 접촉으로 볼 수 있다 |
| D-25 | S1.9 구현에서 확정한 두 가지: (1) **L3 §4.2 의 roll manipulability 최대화 제외를 번복**한다 — 영공간 항 $k_w\nabla\log w_5$ 로 구현했고, seed 규정·결정성 요구는 그대로다 (국소 최대이지 전역 roll 탐색이 아니다). 최대화 대상은 게이트 정의와 무관하게 항상 $w_5$ 라 $q^\ast$ 가 정의에 의존하지 않는다 (C-3 비교가 공정해진다). (2) **$\rho$ 는 $J$ 와 잔차 양쪽에 곱하는 과제 가중**이다 — 잔차에만 곱한 v0.5 식은 차원이 맞지 않고 같은 절의 "1-스텝 정확 정렬" 과도 모순이었다 | **확정** (2026-09-20 사용자 결정, S1.9) | (1) 여유 자유도를 seed 가 남긴 우연에 맡기는 대신 조건수에 쓴다. `planner.ik.k_manip` = 0 이면 번복 전 동작으로 정확히 되돌아가므로 기준선이자 fallback 이 된다. (2) $\rho$ [m/rad] 가 단위 변환으로 쓰이려면 양쪽 가중이어야 하고, 그래야 $\lambda^2=0$ 에서 $W$ 가 상쇄돼 회전 행이 1 스텝에 정렬된다 |
| D-26 | 포구 자세 IK 의 **과제 스텝을 제약 QP 로** 바꾼다 (L3 §4.2 `[확정 D-7d]` 의 `DifferentialIk` 전면 재사용을 번복). $\dot q_{clik}$ 은 관절 한계·스텝 제한을 부등식 제약으로 갖는 QP 가 풀고 (ProxQP, `rtc_tsid::QPSolverWrapper`), $\dot q_n=N\dot q_{sec}$ 는 QP **밖에서** 더한다 — $\dot q_d=\dot q_{clik}+\dot q_n$. `DifferentialIk` 는 $N$ 을 만드는 용도로 남는다 | **확정** (2026-09-20 사용자 결정, A·B 비교 측정 후 — §4.4 S1.9 결과) | $\mu=10^{-4}$ 에서 QP 가 DLS 대비 수락률·잔차·한계 활성 비율에서 근소 우위, 시간 +22%. manipulability 항을 QP cost 에 넣는 초안은 **폐기**했다 (2026-09-20 사용자 지시) — CLIK 출력은 $\dot q_{clik}$ 이고 2차 과제는 영공간 관절 속도라 둘은 더하는 것이지 합치는 것이 아니며, cost 에 섞으면 우선순위가 soft 해진다. 비용: rtc_controllers→rtc_tsid production 의존 신설 (순환 없음, ARCH-2 아님 — 다만 rtc_controller_manager 까지 ProxSuite 가 전이된다), rtc_tsid 에 `ResetWarmStart()` 신설 (후보 간 결정성) |
| D-27 | **분석과 실행의 기준을 분리한다.** catchability 분석 (S3.5a·S3.5b·S4.4) 은 지금처럼 **정교하게** 유지한다 — 게이트를 느슨하게 하거나 층·판정을 지우지 않는다. 반면 **실행 (런타임)** 은 기구학적으로 reachable 한 후보면 **도전한다**: 성능 게이트 (γ 창·도달시간·commit 선행) 탈락은 계획 후보를 *지우는* 필터가 아니라 **우선순위와 진단**이다. 따라서 vision 지평 요구 (S3.6) 는 gate 통과 창이 아니라 **기구학 reachable 창**에서 읽는다 | **확정** (2026-09-22 사용자) | 분석 쪽: 게이트 체인은 낙관적 상한 (S4.4) · 충분조건 (토크 층) · provisional 상수 (`d_eff`·`a_dec`·선행시간) 위에 서 있어 그 교집합을 목표로 *동결* 하면 실제로 잡을 수 있는 투척을 시도조차 못 한다 — 그러나 지도가 부정확해도 된다는 뜻은 아니다 (지도는 우선순위의 근거이므로 정확해야 한다). 지평 쪽: 지평이 짧으면 후보가 **계획기에 도달하지 못해** 시도 자체가 불가능하다 (L1 §4.1 지평 요구 제외) — 비용은 예측점 3 개 (n 16 → 19, `kCap` 40 안). **안전 게이트는 판정으로 남는다**: 정지점·작업공간 (§4.9) 은 팔이 어디에 멈추는지의 문제이고, 어느 게이트를 soft 화하는지의 경계는 E-8 대상이라 **S6 설계에서 별도로 정한다** (§4.4 S6) |

## 1a. Sprint Contract (A-1 승인, 2026-09-19)

Epic 기준 하나와, **각 단계 착수 시 그 단계의 `[SPRINT]` 기준**을 따로 확정한다 (단계 sub-plan 의 `## Spec`).

```
[SPRINT] 1) iiwa7_leap·ur5e_p1b MuJoCo 에서 ball_perception PointCloud2 예측만을 입력으로,
            동결한 목표 투척 분포(D-18 지도, S3.5b)에서 포구 성공률의 Wilson 95% 하한 ≥ floor(D-12),
            로봇별 게이트 (G8-D·G8-D2). 무효 시행을 포함한 전체 발사 수와 무효 사유를 함께 보고         2) 전 RT 경로 할당 0·noexcept·RT-1~10 준수, 기존 컨트롤러 테스트 assertion 무수정 green
         3) 설계 문서 v0.5 가 코드와 일치하고, 이 문서에 단계별 게이트 결과가 기록됨
```

기준 1 의 floor 는 **0.35** 이고 (2026-09-24 사용자 결정 — 처음 값 0.5 provisional (#537 5808613906) 을 S8-B 튜닝 세트 p̂ 94/200 = 0.47 (Wilson [0.40, 0.54]) 을 보고 낮췄다), 시행 수는 n_valid 200 이다 (D-S8-3). floor 는 **2026-09-26 S8-E 계획 확정 시점에 0.35 로 동결됐다** (D-S8-16, #537 5841651825) — S8-E 데이터를 본 뒤에는 바꾸지 않는다. **S8-E 판정 (2026-09-26, §4.4 S8-E 결과)**: G8-D (p1b tennis) **93/200 · 하한 0.3972 PASS** (D-S8-17 host 부하 unit 재실행 반영; 사전 규칙 원 판정 83/200 · 0.3489 FAIL 병기), G8-D2 (leap) **83/200 · 0.3489 FAIL**, beanbag arm 175/200 PASS — 기준 1 은 로봇별이므로 `ur5e_p1b` 충족 · `iiwa7_leap` 미충족. 필요한 시행 수는 S0.9 검정력 표로 먼저 정했다. 실기(S10)는 Epic 기준 밖이며 S10 착수 시 별도 기준을 세운다.

**기준 1 의 판정 정의 (2026-09-24 S8 결정, §4.4 S8).** ① **포구 성공은 sim truth 로 판정한다** — HOLD 끝부터 대기 자세 release 까지 공이 손에 있으면 성공이다. 슈퍼바이저 판정 (지문 힘만 보므로 링크·손바닥 위 공을 Missed 로 읽는다, S7) 은 truth 대비 혼동행렬로 병기한다. ② **무효는 rig 실패만** (srv 거부·lane drop·sim stall·미발사) 이고 "plan 없음·abort" 는 실패로 센다. 무효를 실패로 센 ITT 하한을 병기한다. D-3 clock 위상 오차는 무효 사유가 아니라 공변량이다 (D-S8-4 (c), §5). ③ **floor 는 사용 목적에서 사용자가 사전 고정**하고 n_valid 는 200 이다 (D-S8-3) — floor **0.35 (2026-09-26 동결, D-S8-16)**, 통과 = 성공 ≥ 84/200 (p̂ ≥ 0.42), 검정력 0.8 은 참 p ≈ 0.45 부터 (참 p 0.47 에서 0.93, 0.40 에서 0.31). beanbag arm·G8-D2 도 같은 0.35 (2026-09-26 확인, D-S8-16). 무효의 기계 판정 정의·n_valid 보충 규칙은 §4.4 D-S8-16 ①·①b.

**G8-D2 는 조건부다 (2026-09-22 사용자 결정, S3.5b 후).** 1차 목표 로봇은 `ur5e_p1b` (G8-D). `iiwa7_leap` 은 S3.5b gate 지도가 가정 선행시간 (T_det + L = 0.24 s) 에서 비어 있어 목표 투척 분포가 없다 — 막는 것은 손이 아니라 선행시간 하나다 (0.14 s 면 65 / 2835 열림). 기준 문구는 그대로 두고 판정 규칙만 정한다: **실측 선행시간에서 `iiwa7_leap` gate 지도가 비어 있지 않으면 G8-D2 를 평가하고, 비어 있으면 `NOT_EVALUATED(선행시간)` 로 그 실측값과 함께 보고한다.** **2026-09-22 실측으로 이 조건은 충족됐다** — T_det 실측 (stamp p50 0.075 s) + L 0.14 = 첫 계획 0.215 s 에서 지도가 `torque` 11 / 2835 로 비지 않는다 (§4.4 T_det 실측). G8-D2 는 평가 대상이고, 창이 좁고 L 이 가정값이라 provisional 이다. 기준에서 빼지 않는다 — LEAP 은 fly-in 에서 더 강한 손이다 (L6 §4.5). 반대로 `ur5e_p1b` 의 열림은 세 가정 (fly-in 등가 `d_eff` · D-16 토크 층 · 다시 고른 대기 자세) 위에 있으므로 그 셋은 S6 착수 조건이다 (§7.3 "1차 목표 로봇"). **S8-D 재확인 (2026-09-26)**: τ 0.05 sim 플랜트·재선정 대기 자세·동결 상자에서 실측 첫 계획 p50 0.195 s (194 발) 에 지도 176/180 — 평가 대상 유지, 검증 44/100 (판정은 S8-E, §4.4 S8-D 결과). **판정 방식 (2026-09-26, D-S8-16 ③)**: 검증 100 발은 판정에 쓰지 않고 (Wilson 하한 반올림 전 0.3467, D-S8-3 의 데이터 분리) leap 도 새 seed 701–704 로 n_valid 200 을 돌린다; p̂ 는 상자 전체 1차, 지도-열림 부분집합 공변량. **S8-E 결과 (2026-09-26)**: 상자 전체 83/200 (하한 0.3489) **FAIL**, 지도-열림 부분집합 83/190 (하한 0.3683) 은 공변량.

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
- 시간: RT 의 `ControllerState::t_relative_s` 는 실기에서 steady clock 기반, sim 에서 `iteration × dt` (#566). `ControllerState::dt` 는 항상 1/`control_rate` 이고 sim lock-step 에서도 tick 한 번 = sim step 한 번이다 — **#566 (2026-09-23) 이전의 팔+손 sim 측정은 tick 이 최대 2 배로 돌아 `dt` 적분 법칙이 sim 보다 ~1.67× 빨랐다** (S6 의 CLIK 오차·서보 지연·`track_err_abort` 근거·L4 기준 오프셋 포함, 재측정 대상). `header.stamp` staleness 판단 금지
- CLIK: `ClikReferenceGenerator` 는 pose 목표만, LWA 6행 고정, 측정 q 에서 e·J 평가, 위치∩속도 box, `max_iter` 20 은 내부 `QPSolverWrapper` 고정값. 가속 box·feedforward·마스크·상태 노출 없음. `PinocchioCache` Jacobian 은 `LOCAL_WORLD_ALIGNED` 고정. `Manipulability()` 는 damped 6×6 Gram 의 LDLT 곱이라 게이트로 재사용하지 않는다 (§11). production 소비자는 DemoWbc 하나
- 재사용 대상: `rtc::SeqLock`, `rtc::SpscQueue`, `rtc::compliance::DifferentialIk` (S1.9 는 영공간 투영 N 만 쓴다 — 과제 스텝은 D-26 이후 QP), `rtc_math` se3 `log3`/`exp3`, `QPSolverWrapper`, base 의 `ActivationGeneration()`/`IsCurrentGeneration()`, `thread_layout.yaml` 의 `profiles:`, `repo_scripts/scripts/verify_rt_runtime.sh`
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
| S6 착수 전 `[CONCERN] E-7` | 전 tier × profile 배치표, tier 4 정책, `catching_on/off` (§6) | S6.1 |
| D-24 결정 | 지문 센서 freshness 경로 | S5.2e, S7.3 |
| D-12 값 | floor·시행 수·공 사양·손 토크 권위 출처 | S4.4 판정, S8 (충격 게이트 G7-B3 — S7.3 에서 이월, 2026-09-24) |

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
S5, S3.6, E-7 승인 ─► S6 ─► S7 ─► S8 (S3.1b 부하 clock 위상 포함) ─► S9 ─► S10
```

S1 ∥ S2 ∥ S3a ∥ S4a 는 서로 독립이다. S4.0 은 S5 의 컨트롤러 골격 일부를 앞당긴 것이다 (손 계단 명령을 낼 컨트롤러가 없으면 S4.2 를 할 수 없고, `DemoJointController` 는 손 목표를 quintic 궤적으로 보간해 계단 응답을 줄 수 없다).

### 4.3 단계 상태

| 단계 | 상태 | 게이트 결과 |
|---|---|---|
| S0 결정·문서 v0.5·계약 | 완료 (2026-09-19) | S0.2 W 기록 칸 전부 채움. S0.3 설계 문서 12개(v0.5 헤더) 동기화, 이 문서 포함 `validate_docs` 13 files clean. 정합화 개정 (§7.4). 승인: issue #537 코멘트. S0.7 필요 지평 0.46–0.86 s (R2 지배)·`kCap` 40 제안, 0.5 s profile 부족 (§4.4 S0 결과). S0.9 검정력 표 (§1a) |
| S1 순수 수치 코어 | S1.1~S1.8 완료 (2026-09-19, PR #541), S1.9 는 S2.1 후 | 이식·회귀·RT·시간 PASS, 검증기 PASS, S1.8 PASS (G7-C 임계 NOT_EVALUATED), backfill **PASS (2026-09-22, S3.6: `n_max` 21 ≤ `kCap` 40)** — §4.4 S1 결과 |
| S2 기존 rtc_* 일반화 | 완료 (2026-09-20, PR #545~#549 + 마감 PR). **S2.3b 완료 (2026-09-21)** | se3·동등성·CLIK·extra frame PASS, 가속 도출 PASS(provisional), G5-C solve time 예산 NOT_EVALUATED — §4.4 S2 결과. S2.3b: 두 로봇의 `catch_frame.xyz` 를 S4.5 실측 포구점으로 확정, frame 규약 대조 PASS (§10), 사용자 확인 후 `provisional: false` 전환 완료 |
| S3a 시뮬레이션 기반 | 완료 (2026-09-20, PR #553) | e2e·frame·PROC-3·GUI·plot PASS, D-3 무부하 NOT_EVALUATED (분포 §5.1, ε 하한 49.8 mm 채택, r_cap 후 판정) — §4.4 S3a 결과. S3.7·S3.8 은 범위 밖 (결정 B·C) |
| S4a 손 타이밍 측정 | S4.0·S4.1·S4.2·S4.5 완료 (2026-09-21, 브랜치 `feat/s4a-hand-timing`) | 결정 Q1~Q10 확정. $T_{close,e2e}$ P1b 280.5 ms (η=0.7) · LEAP 103.7 ms (η=0.5). S4.5: **LEAP** $r_{cap}$ 31.0 mm · $d_{eff}$ 80 mm, **P1b** $r_{cap}$ 24 mm · $d_{eff}$ ≥ 95 mm — P1b 는 사용자 제공 자세 두 벌이 테니스공을 파지하지 못해 (851 중 0) **자세를 탐색으로 다시 정했다** (2026-09-21). 놓인 공은 잡지만 날아드는 공은 0.5 m/s 부터 거의 못 잡는다 (폐쇄 속도) → S4.4. S4.3 은 S10 으로 — §4.4 S4a |
| S3.5a kinematic catchability 지도 | 완료 (2026-09-21) — 도구(C++ 파서·배치 CLI·python 코어) + 두 로봇 지도 | PASS(provisional). 수락 투척 (단일 대기 자세) `ur5e_p1b` 1418/3402 · `iiwa7_leap` 1499/3402 (`iiwa7_leap` 은 문턱 0.1 일 때의 값 — 그 뒤 확정된 로봇별 문턱 0.174 로는 **1158**, §11). `sim.throw_region`·`wait_pose`·`planner.ik`·문턱 제안 모두 provisional (사용자 확정 대상). G3-I 는 지도 반쪽만 — 런타임 반쪽 NOT_EVALUATED (S6.2). ⚠️ 수락 후보의 ‖v(t_c)‖ 6.5–8.3 m/s 대 ‖v‖max 1.84/2.26 m/s → S4.4 가 거리·목표 속력을 되돌려야 한다 — §11 지도 결과 |
| S4.4 go/no-go | 완료 (2026-09-22) — **조건부 go** (사용자 결정) | 현 sim 씬·D-18 투척 (4 m, T_f ≥ 1.0 s, 릴리스 1.5–2.0 m) 에서는 **두 로봇 모두 γ 창이 비어 있다** (낙관적 상한으로도 0 %). 조건 셋 — ① 낙차 (포구점이 릴리스 높이 −0.25 m 이상), ② D-16 상수 가속 box 를 자세·방향 의존 한계로 교체, ③ T_f 0.45–0.8 s 를 감당하는 선행시간 — 을 만족하면 열린다: base 1.0 m 가정 씬에서 `iiwa7_leap` 39 · `ur5e_p1b` 11 / 3808 투척 (`PASS(provisional)`, 상한). D-16·D-18 개정은 `[제안]` — §4.4 S4.4 결과·§9·§11 |
| S3.5b gate 지도 | 완료 (2026-09-22) — **`ur5e_p1b` PASS(provisional) · `iiwa7_leap` 지도 빔** | 현 씬·릴리스 0.2–0.5 m. `ur5e_p1b`: 기준 투척 (1.0 m · 0.2 m · 4.75 m/s · 60°) 주변 590 / 2835, 90 % 상자는 속력 4.65–4.85 m/s · 앙각 62–64° — **fly-in 실측 상대속도 + 토크 검사 도달시간 + 다시 고른 대기 자세** 아래에서만 (출하 `d_eff`·출하 가속 box·출하 대기 자세 각각으로는 0). `iiwa7_leap`: 0 — 선행시간이 막는다 (첫 계획 0.24 → 0.14 s 면 65 / 2835). rollout 은 `NOT_EVALUATED(S6)`, q̇ᵘ 동치는 `NOT_EVALUATED(S6.2)` — §4.4 S3.5b 결과 |
| S3.6 vision 사양 | 완료 (2026-09-22) — **PASS(provisional)**: `ur5e_p1b` **기구학 reachable 창** (D-27) 에서 H_req **0.99 s** (T_det **재실측** min 0.040 s — 공 lane stamp 수정 후), 간격 0.05 s, n 20 → `n_max` 20, sim profile **1.0 s / 20 점 설정** (gate 통과 창만 보면 0.84 s · n 17), `io.horizon_min` 0.51 s (R1, 올림) · `io.n_min` **12** (S5.2 정정: ⌈H/step⌉ **+1** — n 점이 덮는 창은 (n−1)·step 이라 11 점은 0.50 s 로 10 ms 짧다; `/code-review` 2026-09-22) · `prediction.dt_expected` 0.05. `io.t_stale`·`io.future_tol` 은 [제안]. `iiwa7_leap` 은 실측 선행시간에서 지도가 열려 G8-D2 평가 대상. T_det 실측·재실측 완료 (§4.4) | — §4.4 S3.6 결과·T_det 재실측 |
| S5 포구 컨트롤러 골격·입력·추종 | **완료** (2026-09-23 머지, PR #564 → `4c0fb751`) | S5.2e: D-24 (a) 센서 lane (PROC-3 전체 5518/0). S5.1: P-1 (a)~(d) · 무장 latch `catching.enable` · A-S5-1 실기 park · TSAN clean (positive control 로 검출 확인). S5.2: 이름 기반 PointCloud2 파서 + SeqLock + D-2 + A-S5-4 되감김 정책, G1-A~C·E·F·H·I·J green, ASan/UBSan clean (경계 제거 사본에서 stack-buffer-overflow 검출). S5.3: 실 모델 폐루프에서 G5-A (정지 목표 1.4 s 에 위치 < 1 mm · 축 < 0.5°, 독립 pinocchio 오라클) · G5-B (위치·속도 box 200 tick 위반 0) · **G5-C PASS** (실측 p99·max 가 예산 400/1500 µs 안, XML 기록) · **G5-E PASS** (지연 fixture 50 ms 에서 77 → 71 mm, 방향 일치) · **G7-H (d) PASS** (QP 실패 streak → FAULT 래치, `ClearEstop` 이 안 푼다 — S5.1 에서 미룬 절반). S5.4: `rtc_msgs/CatchingState` (S5~S9 superset 동결) + `catching_diag.csv` — 상태 메시지와 CSV 가 **같은 POD 한 벌**에서 나오므로 화면과 파일이 갈릴 수 없다. `Compute()` 가 단일 exit 이고 레코드를 tick 머리에서 기본 생성하므로 PROC-7 이 열거가 아니라 구조적 성질이다 (G8-H). **S5.5 PASS** — sim 폐루프 실측 (`260922_2249`, 공 6회): 포구 frame 이 oracle 포구점에서 **0.48 mm · 축 0.057°** (독립 pinocchio FK 로 로그된 관절값에서 재계산), 241639 행 **tick 간극 0**, solve time median 21.7 / p99 79.4 / max 1411.5 µs (G5-C 예산 안), CLIK 미수렴 0. §13 S5 PASS (패널 육안 + Disarm 이 컨트롤러에 반영, `test_demo_gui_catching.py` 23 · `test_plot_rtc_log.py` 14). **착수 중 발견한 블로커 1건**: S5.3 이 `reference.*` 를 소비 부분집합에 넣었는데 출하 프로파일에 `reference:` 블록이 없어 sim configure 가 실패했고, CM 이 한 컨트롤러 실패로 전체를 거부하므로 **ur5e_p1b sim 로봇 전체가 안 떴다**. `reference.provisional` (신설 invented key) 로 닫았다. **머지 전 `/code-review` 9건 전부 실제 결함** — 가장 큰 것은 운동 중 무장 해제가 관절 명령을 한 tick 만에 세우던 전이표 간극 (A-S5-13; 나머지 A-S5-14~16). **`headless=false` 재실행** (`260923_0021`): 포구 frame 이 oracle 포구점에서 0.36 mm 로 정착하지만 콜드 스타트 3회 중 2회 첫 approach 에서 `track_err_abort` 0.3 을 넘겨 ABORT_SAFE 를 거친다 (sim 위치 서보 lag — 사용자 결정 대기, #537). **QP CLIK 과제공간 스윕** (`test_catching_clik_sweep`, 공 속도 4 × 강하각 3 × 방위 3, 출하 gain): solver 는 전 구간 건강 (미수렴 0 · bound conflict 0 · solve max 72 µs), 포구 자세 하나를 1 mm 에 놓는 데 **1.43 s** (축 1° 0.85 s) 인데 공 예산은 **0.25–0.6 s** 다 — 제약은 solver 가 아니라 시간이고 S6 의 t_c·대기 자세 선택 입력이다. (`260922_2249` 세션은 삭제됐고 위 수치는 #537 코멘트에 남아 있다.) 남은 게이트: G5-C2 (backend 왕복) |
| S6 계획기 스레드 | **완료 (2026-09-23)** — PR #568 머지 (`9bf404a8`). S6-D (D-7a 제어 PC 측정) 는 사용자 결정으로 생략. **S6-B** 탐색 (판정/순위 게이트 분리·점수 J·교체·동결, catch sub-model, 계획기 이벤트 CSV·GUI) · **S6-C** γ rollout (coarse-to-fine) · G3-C 합성 1000 투구 p99 < `budget_s` · vision world → 모델 world 변환을 궤적 수신에서 · **S6-C2** CLIK 가속 제약 `box|kinematic|dynamic` (결정 K) · L3 §4.7 런타임 전환 규칙. **#537 결정 반영 (2026-09-23)**: ① p1b `track_err_abort` (1.73 → #566 뒤 재측정으로 **1.54 rad**) · ② p1b `accel_constraint: dynamic` (`eta_tau` 0.8; leap 은 box·0.3 유지) · ③a 오프라인 진단 (#537 코멘트 5789859038) · ④ sim 투척 드라이버 `catching_sim_trials` (투척마다 wait_pose 정렬). **③a 가 찾은 sim 버그 #566 은 main 에 머지됨 (PR #567)** — 팔+손 sim 에서 한 step 에 tick 이 최대 2 회 돌아 `dt` 적분 법칙이 ~1.67× 빨랐으므로 **① 1.73 rad 의 근거 (0.864 rad peak) 와 S6 sim 수치 전부가 재측정 대상**. **⑥ (2026-09-23)**: `planner.switch.e_jump_max`·`ed_jump_max` → 가속 예산 `planner.switch.eta_jump` (L3 §4.7 규칙 2 — 교체가 $u_{des}$ 에 넣는 계단의 상계, RT 가 γ 램프를 재시작하며 채택하는 항 포함; 옛 키는 파서가 거부). **#566 위 재측정** (dynamic 50투 · box 25투, #537 코멘트 5793765020): CLIK 추종 p50/p95 dynamic 2.7/72 mm vs box 128/561 mm → ② 유지, 단 t_c 합계 오차는 비슷하다 (dynamic 은 sim 서보 지연 ~125 mm 가 지배). ① 은 t_c 전 최대 0.772 rad × 2 = 1.54. **결정 ⑦ (a)**: ⑥ 뒤에도 `refreshed` 0 — 첫 plan lead 0.42 s < T_w 0.6 s 라 채택 때 이미 램프 중이고, 램프 재시작 계단 ≈ 11 m/s² 가 예산 5.25 를 넘는다 (규칙이 맞게 거부). 램프 연속 채택은 **S8 로 이월**. **S6-D 생략 (사용자, 2026-09-23)**: D-7a 는 측정 없이 layout 초기값 (tier ≥ 8 `mpc_main` FIFO 60 · slot 3) 을 유지한다. 재판정이 필요해지면 측정 CLI `s6d.sh` (#537 코멘트 5793878042) 로 §7.2 기준을 적용한다 | E-7 승인 (결정 J: 새 layout role 없이 `mpc` role 재사용). 착수 시 확정 결정은 §7.3 "S6 착수 시 확정". **S6-A**: 계획기 스레드 골격 (mpc role · eventfd 대기 · stub `PlanOnce`) · `PlannerRtState`/`plan_box_` (oracle 도 같은 box) · `JudgePlan` (a)~(f) · A-S5-12 sim park · 키 4개. **R-1**: 단위 (`test_catching_mpc_role_switch`: 두 `mpc_main` 이 같은 CPU, switch 뒤 하나만 running, policy `NOT_EVALUATED(EPERM)`) + sim 실측 (`sim_iiwa7_leap` wbc↔catching: 계획기만 ~20 Hz wake, 역전환 시 반대, `enable_mpc:=false` 면 catching switch `ok=False`). 검증기 rc=2 는 dev PC 권한 (FAIL/WARN 집합이 wbc·catching active 어느 쪽이든 동일) 이고, 같은 이름 `mpc_main` 중 **유휴 OpenMP 워커 TID** 를 골랐다. **착수 중 발견·수정 3건**: `ApplyThreadConfig` 가 EPERM 이면 이름을 안 붙임 (rtc_base) · launch 의 `rt_layout_profile` 이 컨트롤러 노드에 안 닿아 **#350 activate 게이트가 실배선에서 죽어 있었음** (CM) · leap YAML 의 안 읽히던 `planner.ik` 사본. `/code-review` 10건 반영 (스레드를 configuration 에 묶음 — on_cleanup join). PROC-3 전체 **5711 / 0 실패** (77 skip) |
| S7 손 시퀀서·슈퍼바이저 | **완료 (2026-09-24)** — PR #571 머지 (`c61fd32a`). 리뷰: `/security-review` Critical 0, `/code-review` 10 건 중 6 건 수정 (ABORT_SAFE 탈출·판정 덮어쓰기·RETREAT 정지의 서보 대기·읽기 불가 팔의 track_err·hold NaN·`delta_rad` 검사 범위). GUI·plot 육안 확인은 사용자 완료. `sat_ticks` 60 (provisional). 한 투척이 컨트롤러 안에서 한 순환을 돈다: IDLE(관절공간 homing, 손 q_open)→ARMED(손 q_pre)→TRACKING→APPROACH→COMMITTED($t_c-T_{freeze}$)→CLOSING(손 닫힘 $t_{cmd}$)→DECEL($t_c$)→HOLD($T_{hold}$)→RETREAT(정지·복귀·release)→ARMED. **단위**: G6-A (합성 1000 시행 ±h/2, mutation 으로 red 확인) · G7-A (시나리오 21, L7 §9 행 + S7 추가분) · G7-B (DECEL 진입 $e=\dot e=0$) · G7-D (무장 모드 전부 할당 0, 최악 tick 152 µs) · G7-G (지문 dropout → TIP_STALE·Undetermined, 옛 힘 판정 0) · G8-A2 (리셋 표 존재 린터 + poison probe + 빠른 재무장 floor 회귀) PASS. **sim 실측** (`260923_2336`, ur5e_p1b, 기준 15 + 변형 10): 25/25 순환 완료, G6-A 닫힘 명령 $(t_c-now)-T_{close,e2e}$ ∈ [−0.95, −0.55] ms, G7-B 25 진입 $|e|=|\dot e|=0$, `ref_saturated` 연속 길이 max 44 · p99 42.5 · median 11 tick (sat_ticks 100 에서 발화 0 → 사용자 결정으로 50, 재측정 뒤 60, 2026-09-24), 정지 바이어스 잡음 0 (sim 지문 lane 무잡음, C-20: f_min 만 유효), 판정 25 전부 Missed · 공 참값과 일치 24/25 (G7-E). **불일치 1건**: 공이 지문이 아닌 손가락 링크·손바닥에 얹혀 HOLD 를 넘겼는데 m_min 지문 합의가 없어 Missed → RETREAT 진입 즉시 release → 복귀 중 낙하 — 지문만 보는 판정의 한계, S8. **재측정 (`260924_0013`, release 규칙 교체 + sat_ticks 50 후, 같은 25 투척)**: 순환 25/25. 판정 Captured 1 · Missed 23 · Aborted 1, 공 참값과 일치 23/24 (불일치는 다시 손바닥 위 공의 false-Missed 1). RETREAT 진입 25 회 모두 손은 Hold, release 25 회 모두 RETREAT 안·대기 자세에서 (측정 편차 최대 0.0041 rad). Captured 와 false-Missed 두 공 모두 손에 실린 채 복귀했고 대기 자세에서 손이 열리며 떨어졌다. `ref_saturated` 연속 길이 max 50 · p99 47.6 · median 11 — **sat_ticks 50 이 1 회 발화** (trial 14, CLOSING → ABORT_SAFE → Aborted, 공은 이미 빗나감). 발화하지 않은 연속 길이 중 가장 긴 것은 CLOSING 의 40·33. 사용자 결정으로 60 으로 올렸다 (2026-09-24, provisional). G6-A 는 22 회가 [−0.98, −0.55] ms, 3 회가 −3.04 · −4.09 · −4.12 ms 다. 이 3 회는 닫힘 tick 자체가 2.5 ms 늦게 돈 경우다 (실시계 tick 간격 4.5 ms). 이번 호스트의 >3 ms tick 은 411 개로 지난 측정의 87 개보다 많았다 (`use_cpu_affinity:=false`, 비-RT). 규칙은 판정식을 만족한 첫 tick 에 발행했고, h/2 한계는 정규 격자에서만 성립한다 — 단위 G6-A 가 판정한다. **포획 0/25** 는 S6 의 입력 그대로 (공이 $t_c$ 약 40 ms 전에 도달, sim 서보 지연 — S8). G7-B3 충격량 상관은 S8 로 이월했다 (사용자 결정 2026-09-24). 토크 비교는 D-12 전이라 NOT_EVALUATED. 드라이버의 외부 정렬은 제거 (C-12) | — |
| S8 sim 통합 평가 | **진행 중 — S8-A·S8-B·S8-C·S8-D·S8-E·S8-F-1 완료, 후속 순서 ②→①→④→③ 확정 (2026-09-26 사용자, #537 5846528236 — §4.4 S8-F-1 결과 후속; 다음 ② sim 팔 예산 실측 `[SPRINT]`) (S8-F-1: 2026-09-26, PR #583 머지 `3278963a` — 상세는 §4.4 S8-F-1 결과; S8-C: 2026-09-25, PR #579 머지 `126e3513`; S8-D: 2026-09-26, PR #581 — 상세는 §4.4 S8-D 결과; S8-E: 2026-09-26, PR #582 머지 `9e7967b5`)**. S8-B (2026-09-24, PR #575 머지 `ac34893a`): p1b sim 팔 서보 τ 0.2 → 0.05 s (D-S8-13) · 2×2 lead on·계획 γ 47/100 (G8-E PASS sim, G8-C 47 대 2) · 튜닝 94/200 · p1b `sat_ticks` 80 · `stale_committed_max_s` 0.10 · `track_err_abort` 0.42 · floor 0.35 (D-S8-3) — 상세는 §4.4 S8-B, 후속 #574. S8-C: RETREAT 손 `T_release_timeout` + 손 관절 캡처 판정 근거 — 상세는 §4.4 S8-C 결과. **S8-E 완료 (2026-09-26; 계획 D-S8-16 #537 5841469779 → 5841651825 · 결과 5842632283 · D-S8-17 5843117293 · 재실행 5843232090; `f0ff9818` · `8baf18cc` · `79a025de` · `879957c0`)** — G8-D (p1b tennis) PASS 93/200 (D-S8-17 재실행 반영, 원 판정 83/200 FAIL 병기) · beanbag PASS 175/200 · G8-D2 (leap) FAIL 83/200 · G8-B·G8-C2·G8-H FAIL, 상세는 §4.4 S8-E 결과. 아래는 착수 시 기록. 커밋 1 docs 정정 · **S8-A 측정 substrate 구현** (`[SPRINT]` 컨펌 2026-09-24): 컨트롤러 read-only 미러 (`planner.wait_pose`·`planner.freeze.T_freeze`·`joint_cmd.lag.{T_arm,lead_enable}`) · `catching_sim_trials --dist s35b` (S3.5b 90 % 상자 iid, seed 재현) · `sim_lanes:=true` · `rtc_tools` `catching_trials` (파일럿 골든 재현: τ̂ LS 199–204 ms 6 관절 · 서보 중앙값 122.2 mm · CLIK 2.2 mm · 25/25 Missed · ε 12 mm 유효 7/25 @ v_max 3.85 m/s; truth 기반 성공 0/25) · `vision_lane_probe --dump` · `analyze_clock_phase` 공변량 요약 · GUI 시행 카운트. RT tick 무변경. **S7 에서 이월 (#537 코멘트 5804113629)**: 포획 0/25 (공이 $t_c$ 약 40 ms 전 도달 — 아래 sim 서보 지연) · 지문만 보는 판정의 false-Missed (링크·손바닥 위 공) · `supervisor.sat_ticks` (60 provisional)·`stale_committed_max_s` 최종값 · G7-B3 충격량 상관 (사용자 결정 2026-09-24) · RETREAT·IDLE 의 손 $q_{pre}$ 대기에 timeout·사유가 없고 끝내 닫히지 않는 서보 차이가 FAULT 로 승격되지 않는 것 · ν̄ (PRED_INCONSISTENT) · σ_ℓ 모니터 소비자 · CLOSING 이 닫힘 명령보다 1 tick 늦게 기록되는 것 (시각엔 영향 없음). **S6 에서 이월 (결정 ⑦, #537 코멘트 5793765020)**: 교체 채택 시 γ 램프 연속 ($\dot\gamma,\ddot\gamma$ 이어 받기) — RT 채택부·γ 프로파일 형태·계획기 rollout 동시 변경 (RT-1~10, `/code-review`). 이것 없이는 `refreshed` 가 램프 중 거부돼 첫 plan 만 쓴다 (L3 §4.7). sim 서보 지연 (`joint_cmd.lag` 가 sim 0) 이 t_c 오차를 지배하는 것도 여기서 본다. **S8 준비 (2026-09-24)**: 코드 대조 C-1~C-24 — sim 팔은 MJCF PD 의 kd/kp = 0.2 s 로 시정수 200 ms 의 1차 지연 ("지연 0" 은 거짓 전제), 러너는 `sim.throw_region` 을 안 읽음, S3.1b 미수행, Wilson·NEES·충격량 도구 부재 (§4.4 S8 준비 대조). **결정 D-S8-1~12 + C-25 확정 (2026-09-24, #537 코멘트 5804952754 → 5807767896)** — 표는 §4.4 S8 (§7.3 "S8 착수 시 확정" 은 그 파급만). **파일럿** (`260924_1218`, 기존 러너 기준 15 + 변형 10, lead off, 개발 PC non-RT): 1차 지연 LS 적합 τ̂ **200 ms** (6 관절 199–204 ms; 속도 xcorr 피크 70–98 ms 는 1차 지연의 τ 가 아니다) · t_c 분해 중앙값 CLIK **2 mm** · 서보 **122 mm** · 공이 t_c 약 40 ms 전 손에 닿음 · 25/25 Missed · 계획 γ_f **0.49–0.69** (DECEL 진입 tick 의 `ref_gamma` 1.0 은 계획값이 아니다) → 동결 p_c 경로 오차 54 mm 의 (1−γ_f) ≈ 22 mm 가 t_c 에 남음 · 첫 plan 0.18–0.24 s · APPROACH 중 교체 **0/25** (파일럿 보고의 3/25 는 발사 후 고정 4 s 창이 RETREAT→ARMED 뒤 다음 순환의 plan 까지 센 것 — S8-A 도구가 시행별 순환 끝까지만 세어 정정) · 손 토크 최대 3.06 N·m = forcerange 클램프 · D-3 부하 δ_max p50 4.5 / p95 10.7 / max 16.7 ms (ε 12 mm 면 유효 7/25). "지연 0 이면 12/25 가 r_cap 안" 은 포구율 예측이 아니라 **필요조건 기하 검사**다 (±120 ms 창 최근접 거리). 단위: **S8-A** 측정 substrate (RT tick 바이트 동일) → **S8-B** lead 보상 (sim overlay) → **S8-C** 조건부 RT (손 timeout · ⑦ · 판정) → **S8-D** leap → **S8-E** 본 평가 → **S8-F** 손 근처 투척 (탐색, 게이트 밖) — §4.4 S8 | — |
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
- H_req = max(R1, R2), 점 수 n = ⌈H_req/0.05⌉. sweep 전 구간에서 R2 가 지배한다
  - **2026-09-20 정정 — 종전 `+ 1` 을 뺐다.** ball_perception 의 예측점은 `step, 2·step, …, horizon` 이라 **t = 0 점이 없다** (`ball_perception_estimation/src/prediction.cpp` 의 `k = 1 … ⌈H/step⌉`; `prediction.hpp` 가 계약으로 명문화). 이 산식에서 파생된 아래 R1·R2 의 n 은 전부 1 씩 줄었고, S3.6 의 `n_max` 산출도 같은 산식을 쓴다
- 계획 가능한 투척은 T_f − T_det ≥ H_req 인 것뿐이다 — 지평을 늘려도 이 한계는 넘지 못한다

| 가정 | 값 | 출처 |
|---|---|---|
| 발사 거리·높이 | 4 m, z 1.5–2.0 m | D-18 |
| 포구점 | ρ_c 0.3–0.8 m, z_c 0.4–1.0 m (base 를 world z=0) | 가정 — world ↔ base 는 S3.2 |
| `t_horizon_margin`, T_pub, T_plan | 0.05, 1/30, 0.010 + 1/60 s | L2 기본값, D-15 예시, L3 `budget_s` |
| L_vis, T_det, T_margin | 0.03, 0.10, 0.02 s | 가정 (sim 은 같은 호스트) |
| T_close,tot, T_arm | 0.01–0.10, 0–0.10 s | 가정 (T_close,tot 문서 예시 60 ms, T_arm TBD) |
| T_w, 관절 t_min | 0.3–0.6 s, ā = 10 rad/s², ω̄ = π rad/s | L3 `window_grid`, 참조 구현 `test_l3` 값 (D-16 box TBD) |
| 항력 k | 0.0229 1/m | L2 §4 검증 표의 값 (공 사양 D-12 대기; `sim.ball.drag_k` 는 **TBD 로 남는다** — S3.8 이 2026-09-20 결정으로 S3a 범위 밖) |

| 항목 | 결과 |
|---|---|
| 도달 가능 투척 (v₀ 4–12 m/s, 포구점 기하 27 조합) | 최소 v₀ 4.5–5.8 m/s (포구점별), 포구 속력 ≥ 5.9 m/s. 최소 v₀ 는 T_f 0.8–0.9 s 부근 |
| T_f ≥ 1.0 s 인 투척 (D-18) | T_f 1.0 s 에서 앙각 40–53°, v₀ 4.7–5.9 m/s, 정점 z 2.2–2.8 m, 포구 속력 6.1–7.2 m/s, 하강각 약 60°. T_f 1.5 s 면 앙각 68–72°, v₀ 6.8–7.7 m/s, 정점 3.7–4.3 m, 포구 속력 7.7–8.4 m/s (포구점 기하 27 조합) |
| 대표 (z₀ 1.75, ρ_c 0.5, z_c 0.6) | T_f 0.5/0.7/0.9/1.0 s → 앙각 1/19/38/46°, v₀ 7.3/5.5/5.1/5.3 m/s, 포구 속력 8.1/6.9/6.7/6.8 m/s. 무항력 대비 같은 T_f 에서 v₀ +2.7–4.3 %, 포구 속력 −3.4–3.8 %, 앙각 −0.5–1.2° |
| L | 0.140 s |
| R1 | H 0.17–0.36 s (n 4–8) |
| R2 | T_move 0.30/0.45/0.60 s × T_arm 0–0.10 s → H 0.46–0.86 s (n 10–18). 대기 자세에서 먼 q* (Δq 1.5 rad → t_min 0.79 s) 이면 H ≈ 1.0 s |

- **`kCap` 제안 40 (provisional)** — sweep 최대 n 18 × 2 = 36 을 8 의 배수로 올림 (2026-09-20 산식 정정 후에도 **40 그대로**). T_f 상한 1.04 s 도 n 21 로 담는다. 먼 q* (H ≈ 1.0 s, n 20) 까지 여유 2배를 요구하면 부족하므로 S3.6 의 `n_max` 가 넘으면 backfill (§4.2)
- **0.5 s profile 판정: R1 충족, R2 부족.** R2 는 T_arm 0.05 s 에서 T_move ≤ 0.29 s 일 때만 충족하는데 T_w 격자 최솟값이 0.3 s 다. **(2026-09-20 해소)** 10 점의 첫 점은 horizon 0 이 아니라 **+0.05 s** 이므로 0.5 s profile 의 실지평은 0.45 s 가 아니라 **0.50 s** 다 — S3.4 의 실측 항목 하나가 미리 닫혔다. 부족 판정 자체는 그대로다
- **D-18 거리·속도 조정으로는 부족분이 메워지지 않는다.** R1·R2 는 로봇 쪽 선행시간이라 발사 조건과 독립이다. 거리·속도가 정하는 것은 T_f − T_det ≥ H_req 뿐이다 — H_req 0.66 s (T_move 0.45, T_arm 0.05) 이면 T_f ≥ 0.76 s, 즉 θ ≳ 25°·v₀ 5–5.5 m/s 의 lob 만 남는다
- **T_f ≥ 1.0 s 결정 (D-18, 2026-09-19) 의 효과:** T_f − T_det ≥ 0.9 s 가 sweep 최대 H_req 0.86 s 를 넘으므로, 지평만 충분하면 목표 분포의 모든 투척이 R2 까지 계획 가능하다 (먼 q* 의 H ≈ 1.0 s 는 T_f ≥ 1.1 s 부터). 필요 지평 자체는 로봇 쪽 값이라 그대로 0.46–0.86 s 이고 0.5 s profile 부족 판정도 그대로다. 대신 포구 속력 하한이 5.9 → 6.1 m/s 로 오르고 T_f 와 함께 커진다 — S4.4 부담은 커진다
- **권장 → 채택 (2026-09-19)**: sim profile 지평 0.8 s (16 점 @ 0.05 s, 설정은 사용자 — D-15). 0.8 s 는 R2 를 T_move + T_arm ≤ 0.64 s 까지 덮는다 — T_move 0.45 s 전부, T_move 0.60 s 는 T_arm ≤ 0.04 s 일 때. 먼 q* (H ≈ 1.0 s) 는 덮지 못하므로 대기 자세를 겨냥점 가까이 두어 T_move 를 줄이는 권장은 남는다 (§7.3). R2 채택 여부는 아래 결정
- **지평 요구 = R1 (2026-09-19 사용자 결정).** 궤적은 검출 직후부터 매 수신마다 갱신되고, 계획기는 `APPROACH` 동안 지평 안 후보로 먼저 출발했다가 더 나은 후보로 교체한다 (L7 `TRACKING → APPROACH`, L3 §4.7 교체 히스테리시스). 도달시간 검사는 대기 자세가 아니라 **현재 명령 상태 (q_c, q̇_c)** 에서 한다 (L3 §4.3). 따라서 R2 (대기 자세 정지 출발) 는 지평 요구가 아니라 **첫 plan 이 나오지 않는 최악 경우의 기록값**이다. R2 로 `io.horizon_min` 을 잡으면 0.8 s profile 을 통째로 거부할 수 있다. 예 (T_f 1.0 s): 0.10 s 첫 예측 (지평 끝 0.9 s) → 약 0.24 s 첫 plan, 작업공간 가장자리 후보로 출발 → 0.25 s 이후 1.0 s 지점이 지평에 들어와 교체 → `t_c − T_freeze` 동결. 지평 (0.8 s) < T_f (≥ 1.0 s) 이므로 첫 목표는 항상 작업공간 가장자리다 — 가장자리 후보의 catchability 는 S3.5a, 첫 plan 시각·교체 횟수·탈락 사유는 S8 에서 잰다
- **대기 자세 = 겨냥점 근처 (2026-09-19 사용자 결정).** 연속 재계획에서 대기 자세가 정하는 것은 정지 출발인 첫 이동이다 — 가까울수록 가장자리 후보의 도달시간 탈락과 교체 시 오차 점프가 준다. IK seed (D-18)·지도 겨냥점 (§11) 과 같은 자세다. 구체 자세는 S3.5a 방위별 포구 가능 구간으로 정한다
- **`kCap` = 40 (2026-09-19 사용자 결정, provisional).** 0.05 s 간격에서 약 2.0 s 지평까지 담고 (첫 점 0.05 s, 40 번째 2.00 s), 스냅샷 매 tick 복사는 약 3.2 KB 다. S3.6 `n_max` 가 넘으면 backfill (§4.2)
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
- S1.9 **(완료 2026-09-20. S2.1 머지 후 착수 — IK 회전 행 $e_a^C$ 가 S2.1 의 `rtc_math` 축 정렬 회전벡터다, L3 §4.2)** 포구 자세 IK 반복 루프 + catchability 판정 함수 (L3 §4.2, §11): `DifferentialIk` (m=5) 를 반복 호출하는 루프, 수렴 판정·스텝 제한·seed = wait_pose, 해에서 w₅·w₆ 계산과 fail-closed 판정 (§11), 영공간 $\log w_5$ 상승 (D-25). 입력은 `RtModelHandle` (rtc_controllers 는 이미 `rtc_urdf_bridge` 에 의존). **S3.5a/b 지도 도구와 S6.2 런타임이 이 함수 하나를 쓴다.** 추상 interface 는 만들지 않는다 (ARCH-3)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 이식 | 참조 테스트 전부 GTest 통과 (L0 G0-A, L2 G2-A~D·G2-G, L3 G3-A·G3-B, L4 G4-A~C·G4-F). G4-D (축 정렬) 는 S2.1, derate 테스트는 D-8 로 기능과 함께 제외 (G4-E) | — |
| 회귀 | `n > kCap`·NaN·`dt_min` 미만 입력에서 ASan/UBSan 무오류 + invalid (G2-H, G1-A 의 해당 항목), 비단조 쌍 invalid (G2-G), NaN 목표 가드 (G4-I) | — |
| RT | 대상 경로 `ScopedNoMalloc`·`ScopedAllocGate` 할당 0, `noexcept` (G0-B, G2-E, G4-G) | — |
| 시간 | 다른 시간 타입끼리 비교가 컴파일되지 않음, T_arm ≠ 0 fixture (G0-E) | — |
| 검증기 | 활성 구성 TBD·provisional·교차제약·ζ·ω·h (G0-C) | — |
| S1.8 | 전이표 완전성 검사 (G7-A 의 표 부분), 감속 전환 시 기준 상태 연속 < 1e-9 (G7-B), 합성 잡음 접촉 오경보율 기록 (G7-C — 임계는 사용자 결정이라 NOT_EVALUATED), 할당 0 (G7-D) | G7-C 임계: 사용자 |
| S1.9 | 합성 기구학에서 수렴·스텝 제한, w₅·w₆ 유한차분 대조, zero speed·NaN/Inf·rank-deficient·near-singular 입력에서 후보 탈락 + 사유 코드 (G3-I 의 함수 부분), w₅ 상승이 roll sweep 국소 최대에 도달 (D-25), 할당 0 (G3-K 함수 부분) | — |
| backfill | S3.6 의 `n_max ≤ kCap`. 초과하면 `kCap` 상향 후 이 표 재실행 | S3.6 |

GUI·plot: 면제 (D-19, §13).

**S1 결과 (2026-09-19, S1.1~S1.8).** 코드: rtc_controllers `include/rtc_controllers/catching/` (헤더 전용) + `src/params/catching_params.cpp`, 테스트 `test/test_catching_*.cpp` 7 스위트 130 케이스. 측정: colcon (ws root) 로 rtc_controllers 전체 652 케이스 green·새 경고 0, 같은 7 스위트를 ASan/UBSan (+`_GLIBCXX_ASSERTIONS`) 별도 빌드로 실행해 보고 0 (레시피: [testing-debug.md](../../agent_docs/testing-debug.md) sensor matrix). positive control: `SampleAt` 의 `n > kCap` 검사를 지운 사본은 G2-H 케이스에서 중단된다. `/code-review` (브랜치 전체) 반영 4건: FAULT 에서 ESTOP 해제가 래치를 풀던 전이 행 (L7 §4.2 사유표 "전 구간 → IDLE" 을 그대로 옮긴 것 — P-1·S5.1(d) 와 모순이라 사유표에 FAULT 예외를 적었다), 없는 YAML 섹션이 `YAML::InvalidNode` 를 던지던 parser (이제 기본값으로 읽고 검증기가 막는다), 빈 손 배열 통과, `Check()`·`Interpolate` 의 wire 시각 int64 뺄셈 overflow (포화 연산. 되돌린 사본은 UBSan 에서 중단).

| 게이트 | 판정 | 근거 |
|---|---|---|
| 이식 | PASS | G0-A 6종, G2-A~D·G2-G, G3-A (verify_l3.py 독립 python 닫힌식 고정표 42행 < 1e-9 — LP 대조 ≤ 1.2e-5 s. 참조의 `cases.txt` 는 C++ 가 자기 출력을 대조하는 순환이라 대체), G3-B (§4.8 표 ±5%), G4-A~C·G4-F — 임계 무수정. 참조 L2 fixture 60 점 → 40 점 (`kCap`) 과 ns 반올림 시각에서의 참값 적분은 fixture 변경 (사용자 승인). 참조 G4-B 의 `step(…, dt=0)` 읽기는 `Evaluate()` 로 |
| 회귀 | PASS | G2-H·G1-A 해당분 (개수 경계 선검사, NaN/Inf, `dt_min` 미만, 한계 무효), G2-G, G4-I (비유한 목표·t·dt·명령 오버플로 → 상태 보존 + invalid + saturated, 다음 유한 입력에서 쌍둥이 실행과 bit-identical) |
| RT | PASS | `ScopedAllocGate` + `ScopedNoMalloc` 할당 0: `SampleAt`·`Check`·`Step`·`Evaluate`·도달시간/γ 창 게이트·감속 목표·debounce. 전부 `noexcept`. 기록 (개발 PC, 비 RT 커널): 스냅샷 복사 3240 B/tick, 복사 + `SampleAt` 최악 3.4 µs, `Step` 최악 0.2–0.4 µs |
| 시간 | PASS | G0-E — 교차 축 비교·산술·변환이 컴파일되지 않음을 `static_assert` 로, T_arm = 50 ms fixture 에서 §3 표의 판정별 축 고정. D-2 (3) 변환은 미래 stamp·오버플로 거부 |
| 검증기 | PASS | G0-C 전 항목 (사유 코드까지 단언). ωh ≥ 0.828 경계는 `reference.omega` 범위 [1, 25] 안에서 도달 불가 (100 Hz 에서도 s ≤ 0.25) 라 범위 밖 ω 로 공식만 검증했고, 게이트 문구를 그에 맞게 고쳤다 (2026-09-19 사용자 결정, L0 §5.3·§9) |
| S1.8 | PASS · G7-C NOT_EVALUATED(임계) | G7-A 표 완전성 (도달 불가·미사용 사유·중복 칸 각각 음성 테스트), G7-B 진입 시 e = ė = 0 정확·τ_s 연속, G7-D 할당 0. G7-C 오경보: k_σ = 3 합성 잡음 **0/20000** (debounce 후) 기록 |
| S1.9 | PASS | 아래 S1.9 결과 |
| backfill | **PASS** (2026-09-22, S3.6: `n_max` 20 ≤ `kCap` 40 — §4.4 S3.6 결과) | — |

- 기록: 0.05 s 간격 16 점 (D-15 sim profile) 보간 오차 — 위치 2.0e-11 m, 가속 1.9e-7 m/s² (1/60 s 간격은 3.2e-14 m). S3.6 간격 선택의 입력
- 구현 중 정정 (설계 문서 반영): L3 §5.2 `q_star` 용량도 "`kCap`" 이라 불러 궤적 용량과 이름이 겹침 → `kMaxPlanNv` (32). L4 §5.1 "dt ≤ 0 invalid" 와 참조 G4-B 의 dt = 0 읽기 충돌 → `Evaluate()` 분리. `SampleAt` 은 비단조 쌍을 구조적으로 선택하지 않으므로 G2-G 는 `Interpolate` 직접 호출로만 도달한다
- 수치 감사 (read-only 에이전트) finding 전부 반영: `Evaluate()` 가 a_max < 0·demand 0 에서 NaN 을 valid 로 내던 fail-open (blocking), 1 ms 미만 γ 램프·100 µs 미만 보간 구간·int64 시각 오버플로·`TRest` 직접 호출·+Inf 속도 입력 — 각 회귀 테스트 포함

**S1.9 결과 (2026-09-20).** 코드: `rtc_controllers/include/rtc_controllers/catching/catch_pose_ik.hpp` + `src/catching/catch_pose_ik.cpp` (`rtc::catching::CatchPoseIk`), 테스트 `test/test_catch_pose_ik.cpp` 25 케이스 + 공용 fixture 헤더 `test/include/rtc_controllers/testing/catch_arm_fixture.hpp`, 신규 URDF fixture `rtc_urdf_bridge/test/urdf/serial_6r_wrist.urdf`, rtc_tsid `QPSolverWrapper::ResetWarmStart()` + 그 테스트. 설계 변경 3건 (D-25 roll manipulability 최대화 번복 · ρ 과제 가중 정정, D-26 과제 스텝 제약 QP) 을 L3 §4.2·§6·§10, architecture.md dep graph, rtc_controllers README 에 반영했다.

**A·B 비교 (사용자 결정 근거).** 두 후보는 $\dot q_{clik}$ 계산 **한 줄만** 다르고 수락 조건·영공간 항·종료 규칙을 공유한다. 2 fixture × 500 후보, 동일 후보 리스트. bench 는 결정 후 폐기했고 표만 남긴다.

| candidate | accept | w₅ p50 | w₅ p10 | resid p50 | limit 활성 | µs p50 | µs p99 | QP 비수렴 |
|---|---|---|---|---|---|---|---|---|
| 6R A: DLS | 98.8% | 0.0203 | 0.0044 | 8.3e-08 | 1.2% | 1444 | 2167 | – |
| 6R B: QP μ=1e-8 | 99.0% | 0.0201 | 0.0043 | 1.4e-07 | 1.4% | 2061 | 2175 | **483** |
| 6R B: QP μ=1e-6 | 99.2% | 0.0201 | 0.0043 | 8.8e-08 | 2.0% | 1783 | 1897 | 0 |
| **6R B: QP μ=1e-4** | **99.2%** | 0.0201 | 0.0043 | 7.8e-08 | **1.0%** | 1777 | 1877 | 0 |
| 6R B: QP μ=1e-2 | 93.4% | 0.0212 | 0.0057 | 2.1e-07 | 1.3% | 1775 | 2396 | 0 |
| 7R A: DLS | 98.2% | 0.0817 | 0.0148 | 4.3e-07 | 5.5% | 1820 | 2089 | – |
| 7R B: QP μ=1e-8 | 98.2% | 0.0800 | 0.0148 | 7.4e-07 | 5.9% | 2512 | 2680 | **460** |
| 7R B: QP μ=1e-6 | 98.6% | 0.0822 | 0.0147 | 4.3e-07 | 4.1% | 2230 | 3490 | 0 |
| **7R B: QP μ=1e-4** | **98.8%** | 0.0820 | 0.0146 | 4.5e-07 | 4.7% | 2227 | 2455 | 0 |
| 7R B: QP μ=1e-2 | 88.2% | 0.0904 | 0.0240 | 6.2e-07 | 5.0% | 2241 | 2530 | 0 |

- **선택은 B (사용자, 2026-09-20).** μ=1e-4 에서 수락률·잔차·한계 활성이 A 이상이고 시간은 +22% 다
- **μ 는 절벽이 있는 손잡이다.** 1e-8 이면 $J^\top J$ (rank ≤ 5) 에 대한 정칙화가 모자라 QP 가 대부분의 반복에서 수렴하지 않는다. A 의 σ_min 적응 λ 는 스스로 맞추던 것이므로, 이는 B 가 새로 들여온 비용이다
- **QP 가 사려던 것을 절반만 샀다.** 제약은 $\dot q_{clik}$ 만 묶고 실제로 움직이는 것은 $\dot q_d$ 라 $\Vert\dot q_d\Vert_\infty$ 축소와 한계 clamp 가 여전히 필요하다. limit 활성 비율 차이가 그만큼만 나는 이유다
- **첫 측정은 틀렸다** — bench 가 `QPSolverConfig` 를 만들고 `Init()` 에 넘기지 않아 B 가 기본값 (eps_abs 1e-6) 으로 돌았고, B 의 수락률이 74.6%/61.6% 로 나왔다. 위 표는 수정 후 값이다

측정: rtc_tsid 235 · rtc_controllers **682** · rtc_controller_manager 215 케이스 green (colcon, ws root), 전체 22 패키지 빌드 성공, 변경 패키지 경고 0. 같은 스위트를 ASan/UBSan 별도 빌드 (rtc_tsid·rtc_controllers 동일 플래그, +`_GLIBCXX_ASSERTIONS`, `-DEIGEN_MALLOC_ALREADY_ALIGNED=1`) 로 돌려 25/25 green · ASan 0 건. UBSan 잔여 3 종 (`LLT.h:66`, `SelfAdjointEigenSolver.h:76`, `CoreEvaluators.h:1264` 의 초기화 전 `ComputationInfo`/enum load) 은 각각 기존 `test_dls_convergence` 와 `test_qp_solver_wrapper` 에서도 재현되므로 **이 변경 이전부터 있던 third-party UB** 다.

**할당 0 의 거짓 green 과 그 수정.** 정상 빌드는 할당 0 을 보고했지만 sanitizer 빌드는 IK **반복당 Eigen 할당 1 건**을 봤다. 원인은 `g.noalias() = -(Jᵀe)` — `noalias()` 는 맨 곱셈에만 임시를 없애고, 단항 음수가 감싸면 Product 를 런타임 크기 임시로 평가한다. 두 문장으로 쪼개 (곱 → 제자리 부호 반전) 0 이 됐다. 이 과정에서 드러난 두 가지를 테스트로 박았다:

- `TheAllocationGatesAreArmed` — 두 게이트가 **실제로 발화하는지** 먼저 잰다. 첫 시도의 `new double` + `delete` 대조는 컴파일러가 쌍을 제거해 무효였다 ([expr.new]/10). `std::vector` 로 바꿨다
- `TheTaskQpItselfAllocatesNothing` — `QPSolverWrapper::Solve` 의 "compute 경로 할당 없음" 은 헤더 주석의 **미검증 주장**이었다. 양쪽 빌드에서 0 으로 측정했다

positive control (10종 mutant, 각각 빌드·실행해 red 확인 — 뒤 5개는 아래 리뷰 반영분):

| mutant | red 가 된 테스트 |
|---|---|
| 스텝 제한 제거 | `EveryStepObeysTheInfinityNormBound`, `TighterStepBoundCostsMoreIterations` |
| 반복별 관절 한계 clamp 제거 | `AcceptedPoseRespectsJointLimits` |
| 영공간 투영 N 제거 | `AscentReachesTheRollSweepLocalMaximum`, `ReversingTheGradientSignDescendsInstead` |
| 회전 행을 LOCAL 대신 LWA 로 | 9 케이스 (수렴·w 대조·상승 전부) |
| FD 기울기 부호 반전 | 상승 3 케이스 |
| 관절 순서 거부 제거 | `ADeviceOrderedHandleIsRejectedRatherThanSilentlySolved` |
| 실패한 FD 탐침이 허용오차 비교로 빠짐 | `AnUnusableGradientProbeIsNotReportedAsAConvergedAscent` |
| QP 실패가 수락된 자세를 버림 | `AQpFailureAfterAcceptanceKeepsTheAcceptedPose` |
| σ_min·λ² 를 마지막 반복 값으로 둠 | `TheProjectorDiagnosticsDescribeQStarNotTheLastIterate` |
| $q_n$ 을 clamp 안 한 seed 로 | `AnOutOfLimitSeedIsNotAPermanentPosturePull` |

- **오라클은 3단계**다. (1) dense determinant 로 w 산술 대조 (1e-9), (2) **FK 만 쓰는 유한차분 Jacobian** 으로 frame·행 규약 고정 (Jacobian API 미사용), (3) roll 1°×±90° sweep 에서 후보마다 전체 6D IK 를 풀어 만든 w₅(ψ) 곡선의 같은 가지 국소 최대와 함수의 w₅(q\*) 대조 (기울기 구현과 독립)
- **선형 행의 frame 은 w₅ 가 고정하지 못한다** — LWA 와 LOCAL 은 $T=\mathrm{diag}(R,I_2)$ 만큼만 다르고 det 는 $\det(R)^2=1$ 배라 값이 같다 (테스트가 이 불변성을 단언한다). 그 규약을 고정하는 것은 수렴 테스트다 (잔차가 world 벡터라 LOCAL J 를 쓰면 발산)
- 결정성: 같은 입력 2회 + **사이에 다른 후보를 끼운 3회차**가 bit-identical. QP 는 후보마다 `ResetWarmStart()` 로 cold start 하므로 성립한다 (지도와 런타임 동치의 전제, §11)
**`/code-review` (브랜치 전체, PR #552) finding 5건 반영.** 전부 "유한하고 그럴듯하며 자기 실행에 대해 거짓인 결과" 로, 크래시도 NaN 도 아니라서 위 스위트가 전부 green 인 채로 통과하던 것들이다. 각각 회귀 테스트 1개 + mutation red 확인:

| finding | 증상 | 수정 |
|---|---|---|
| device 관절 순서 | `SetJointOrder` 가 걸린 핸들에서 `ComputeJacobians` 입력(device)과 Jacobian 열·한계·$\dot q$(Pinocchio)가 섞여 **모든 후보가 조용히 틀린다**. 실기 배선이 실제로 그런 핸들을 만든다 (`momentum_observer_wiring`) | `HasJointReorder()` 를 거부 (`kJointOrderMismatch`, 새 사유 코드). L3 §4.2 |
| 실패한 FD 탐침 | 탐침이 못 쓰게 나오면 `grad_norm`=0 → `manip_converged=true` 로 루프가 끊겨, **상승이 한 번도 안 돈 자세**가 "수렴" 으로 보고된다 (G3-G 신호 역전) | 탐침 실패는 수렴 아님 + `manip_grad_failures` 카운터 신설 |
| QP 실패의 범위 | 이미 허용오차를 만족한 $q^\ast$ 가 있어도 이후 QP 비수렴이 그것을 버리고 `kQpFailed` + 전부 0 인 `q` 를 냈다. D-25 상승이 생기면서 비로소 도달 가능해진 경로 | 수락 전이면 거부(그대로), 수락 후면 $q^\ast$ 반환 + `qp_failures` 로 조기 종료 기록 |
| σ_min·λ² 의 귀속 | "at q\*" 로 문서화됐으나 **마지막 반복** 값을 실었다 (w₅·w₆ 는 $q^\ast$ 에서 재평가하면서 이쪽만 빠졌다) | 수락된 반복의 값을 따로 들고 종료 시 교체 |
| clamp 안 한 $q_n$ | 한계 밖 seed 가 **영원히 감쇠하지 않는** 자세 인력점이 되어, clamp 와 매 반복 싸운다 | $q_n$ = clamp 된 seed (`q_ref_`). L3 §4.2·§6 |

재측정: rtc_controllers 682 케이스 green, downstream (`rtc_controller_manager`·`integrated_bringup`) 빌드 성공. 같은 sanitizer 레시피로 `test_catch_pose_ik` 25/25 green·ASan 0 건 — 새 코드의 할당 0 은 **최적화 빌드에서** 재확인했다 (정상 빌드의 0 은 위 거짓 green 사례 때문에 근거로 쓰지 않는다). UBSan 은 위 3 종 중 2 종 (`LLT.h:66`, `SelfAdjointEigenSolver.h:76`) 이 이 스위트에서 재현되고 새 보고는 없다.

- 미결: `planner.ik` 의 provisional 기본값 (`sigma0`, `lambda_max`, `dq_step_max`, `k_manip`, `manip_grad_tol`, `mu`, `qp_eps_abs`) 은 S3.5a 지도 실측으로 제안하고 사용자가 확정한다 (L3 §10) — **2026-09-21 확정**: `k_manip` 0.5 · `max_iter` 40 (출하 config 반영), 나머지는 기본값에서 손댈 근거가 없었다 (§11 지도 결과). `alpha_max` 는 여전히 TBD 라 함수는 인자로 받는다

#### S2 기존 rtc_* 일반화 (code review 대상)

- S2.1 `rtc_math` se3 에 축 정렬 오차·각속도·Jacobian (deadband 에서 유한), 유한차분 테스트
- S2.2a CLIK 확장 구조 결정 1쪽 ("행 집합 선택형 확장" 대 "`QPSolverWrapper`·se3 오차만 공유하는 formulation 클래스") + **기존 동작 golden-vector 회귀** (기록한 q 열 → q_ref 해시). golden 테스트가 생기기 전에는 기존 위치∩속도 box 코드를 재구조화하지 않는다
- S2.2b `ClikReferenceGenerator` 옵션 (D-5·D-6), 옵션별 개별 커밋: twist feedforward, LOCAL 접근축 2행, 가속 box + `bound_conflict`, 직전 q̇ 평활 항, status·반복·solve time 노출, `max_iter` 설정, q_c 평가 모드
- S2.3a `rtc_urdf_bridge` extra frame 기구 (D-10, D-17, §10): 로봇 config `urdf.extra_frames.<name>.{parent,xyz,rpy,provisional}` → CM 파서 (`list_parameters` map key, `ParseSubModels` 와 같은 방식) → `ModelConfig` 필드 → yaml-cpp `LoadModelConfig` 경로(같은 키 지원 또는 명시적 거부) → `PinocchioModelBuilder` 가 `BuildFullModel()` 직후 full 모델에 `addFrame`. 축(rpy) 초기 제안값 산출. 결합 모델 nv (full 26 / actuated 16) 를 테스트로 고정
- S2.3b (S4.5 이후) 포켓 중심 offset 제안값 — S4.5 실측 포구점을 부모 frame 으로 옮긴 값 (§10; "preshape 손끝 중심" 은 기각)
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

**S2 결과 (2026-09-20, S2.1·S2.2a·S2.2b·S2.3a·S2.5·S2.4 — PR #545~#549).** 착수 전 코드 재확인에서 계획 서술 정정 5건을 보고했다 (2026-09-19): G5-5 의 `max_iter` 는 CLIK 하드코딩이 아니라 `QPSolverConfig` 기본값이고 반복 수·solve time 은 `SolveResult` 에 이미 있다 (CLIK 노출만 없다). extra frame 파서는 `ParseSubModels` 의 "불완전 항목 조용히 건너뛰기" 를 따르지 않고 실패시킨다. G5-B2·G5-C3 의 L7 전이·abort 경로 부분은 S5.3·S7 에서 판정한다. S2.5 표본 범위는 대기 자세 (S3.5a) 전이라 결과가 provisional 이다. golden 비트 일치는 Release 빌드에서만 판정한다 (sanitizer 빌드는 최적화 수준이 달라 상대 1e-12). 사용자 결정 (2026-09-19): PR 은 S2.1 먼저 머지 → S2.2 · S2.3a · S2.5 → S2.4 마감, S2.2a 는 행 선택형 확장 (L5 §5.1), S2.5 도구는 rtc_tools python.

S2.4 중 발견 (기존 상태, 이 단계 범위 밖): p1b idle sim 에서 DemoWbc 의 **TSID (dynamics) QP 가 거의 매 tick 수렴하지 않는다** (`qp_converged` 0.5 %, `qp_fail_count` 가 tick 마다 증가). S2 이전 wrapper (main `f0e03c42`) 로 되돌린 같은 조건의 A/B 에서도 0.5 % 라 S2 회귀가 아니다 (수렴 판정에서 유한성 조건만 뺀 경우 2 %). CLIK (kinematic) 쪽은 세 run 모두 `clik_valid` 100 %. 원인 조사는 별도 이슈 후보다.

S2.3a 중 발견: DemoWbc 주석의 "full 모델 nq=26, nv=21 (mimic)" 은 어느 로봇과도 맞지 않아 실측값으로 고쳤다 (E-9, 주석을 코드에 맞춤). CM `system_model_config_` 는 configure 사이에 초기화되지 않아 sub/tree 모델이 재configure 마다 누적된다 (기록만 — extra frame 파서는 파싱 전에 목록을 비운다). S5 배선 항목: `CheckCatchFrameProvisional`·`ValidateCatchingParams` 는 아직 production 호출자가 없다.

S2.2a 중 발견 (2026-09-19): `QPSolverWrapper` 는 비유한 해 한 번 뒤 모든 solve 가 실패했다 (warm start 가 NaN 에서 출발, ProxQP 는 NaN 에서도 SOLVED 보고) — golden 기록 전에 별도 수정으로 고쳤다 (사용자 결정: 선수정). 기록만 한 기존 동작 두 건: SE3 경로의 base 정렬 오차 × world 정렬 J (base 가 world 에 대해 회전하면 불일치, 현 구성에서 잠재), `anchor_drift_max` clamp 가 비유한 `q_ref` (dt = +inf) 를 유한값으로 세탁 (dt 는 control_rate 고정이라 실경로 도달 불가).

| 게이트 | 판정 | 근거 |
|---|---|---|
| se3 (S2.1) | PASS | `rtc_math/include/rtc_math/se3/axis_align.hpp`, `rtc_math/test/test_axis_align.cpp` 11 케이스. G4-D: exp 잔차 < 1e-12 (0.5° 격자 × 20 축), 1° 격자 ‖ω‖ 변화 < 1.5·K_a·π/180 (K_a 8, w_max 6), 유한차분 < 1e-5 (1–170°), 두 데드밴드 주변 ‖m‖ 1 … 1e-18 격자에서 출력 전부 유한·상한 이내. 무효 입력은 0 + 무효. 할당 0 (`ScopedNoMalloc`), noexcept `static_assert`. 데드밴드 처리 방식은 L4 §4.5 표. positive control: 급수 분기의 부호 조건 제거·Jacobian 상한 제거·데드밴드 J ≠ 0·단위 검사 제거·ω 포화 제거 5개 변형이 각각 해당 테스트에서 실패. `/code-review` (브랜치) finding 2건 반영: 아주 작은 $\epsilon_{\sin}$·floor (≲ 1e-103) 에서 반평행 근처 J_a 가 overflow 해도 유효로 보고되던 것 (하한 1e-12 + 비유한 결과 무효 처리), 큰 게인에서 ‖K_a e_a‖ overflow 로 ω 가 포화 대신 무효가 되던 것 (K_a‖e_a‖ 비교). colcon (ws root, Release) rtc_math 33 케이스 green, ASan/UBSan 빌드 보고 0 |
| 동등성 (S2.2a·b) | PASS | `rtc_tsid/test/test_clik_golden.cpp`: 4 시나리오 2520 값 (tick 당 ok·q_ref·v_ref·manipulability·오차 노름) 을 IEEE-754 비트로 기록. Release 는 비트 일치, sanitizer 빌드는 상대 1e-12. 기록 입력이 모든 분기를 지나는지 별도 테스트로 확인하고, 분기별 mutation 11개가 모두 golden 을 red 로 만든다. S2.2b 의 모든 커밋에서 비트 일치. 기존 `test_clik_reference`·DemoWbc (integrated_bringup 1233) assertion 무수정 green |
| CLIK (S2.2b) | PASS · G5-C solve time 예산 NOT_EVALUATED(사용자 값) | `rtc_tsid/test/test_clik_options.cpp` 25 케이스. G5-A: 정지 목표 위치 < 1 mm·축 < 0.5°. G5-B: 랜덤 목표 1e4 tick 에서 속도·가속 위반 0 (ProxQP eps 1e-6 이내), `bound_conflict` 706 tick, 위치 초과 최대 0.054 rad < margin 0.1 (≈ v²/2a). G5-B2 (CLIK 쪽): 충돌 시 가속 한계 유지 + 관절 bit 보고. G5-C3 (CLIK 쪽): `max_iter` 준수, status 노출, false 반환. G5-C 할당 부분: 옵션 전부 on·두 오버로드 교대 1000 tick 할당 0. 옵션별 mutation 9개 전부 red. `/code-review` (S2.2b 범위) finding 반영: 실패한 호출 뒤 `v_prev` 가 마지막 성공 속도로 남아 다음 tick 가속 창이 0 출력에서 그 속도로 점프하던 것 (실패 시 0 으로 초기화), 명령값 모드의 상태 검사가 실패 한 번으로 한 tick 꺼지던 것 (ResetAnchor 전까지 유지) — 각각 회귀 테스트와 mutation 확인. ASan 0, UBSan 은 ProxSuite 내부 bool 읽기 1건뿐 (기존 테스트에서도 발생, testing-debug.md). G5-B2·G5-C3 의 L7 전이·abort 경로 부분은 S5.3·S7 |
| extra frame (S2.3a) | PASS | `rtc_urdf_bridge` `ModelConfig::extra_frames` → full 모델 OP_FRAME, 파생 모델 상속. `test_extra_frames` (four_bar closure): full·sub·tree·actuated 네 모델의 부모 기준 위치 1e-12 일치 (부모 관절이 잠긴 모델 포함), 기존 frame id·관성 불변, 없는 부모·중복·빈 이름·비유한 값 실패, `LoadModelConfig` 불완전 항목 거부 — 부모 placement 합성을 빼는 mutation red (처음엔 fixture 의 부모가 전부 joint 기준 identity 라 공허했고 c1 로 바꿔 잡았다). CM: `urdf.extra_frames` 파싱·재configure·없는 부모·불완전 항목 → configure 거부 (`test_cm_config_pipeline`). 검증기: `CheckCatchFrameProvisional` 은 sim 경고·실기 차단 (G0-C, 호출자는 S5). 실모델 `test_catch_frame_models`: 출하 config 를 읽어 네 모델 존재·위치, 손가락을 곧게 편 자세에서 0 에서 먼 한계 쪽으로 25·50 % 굽힐 때 끝점 중심이 catch frame +z 로 이동 (p1b 3.2/4.6 cm, iiwa 10.1/7.9 cm, rpy 를 뒤집으면 red), ur5e_p1b full nq = nv = 26·actuated 16, iiwa7_leap full·wbc nv 23. 축 초기 제안: p1b `l_palm_link` rpy 0, iiwa7_leap `palm_lower` rpy [π,0,0], provisional. xyz 는 **S2.3b 에서 확정** (2026-09-21): p1b `[0.015, 0.145, 0.052]`, iiwa7_leap `[-0.035, -0.015, -0.069]` — §10. `/code-review` finding 0 |
| 가속 도출 (S2.5) | PASS(provisional) — 표본 범위는 S3.5a 대기 자세 전 | `rtc_tools derive_accel_limits` (§9 절차) + `integrated_bringup/config/<robot>/derived_accel_limits.yaml` (키 `derived_accel_limits.<group>.qdd_max`, provenance 포함). η_τ = 0.8 (사용자 확정 2026-09-20), 균일 가중, 관절 한계 box 전체·±max_velocity, 20000 표본 + 최악 10 표본 Powell 정제 (표본 최소값은 표본 수에 따라 계속 내려간다: p1b 2000 → 5.0, 20000 → 4.1 rad/s², 정제값 2.03 으로 수렴). ur5e_p1b **2.03 rad/s²** (binding 거의 전부 shoulder_lift, 중력), iiwa7_leap **9.20 rad/s²** (binding A2). 퇴화 없음 (τ_dyn ≤ 0 표본 0). 교차 검증: RNEA (부호 패턴 전부) 최악 0.78·0.85, iiwa MuJoCo `mj_inverse` (iiwa7_with_leap_right, armature·damping 포함, 접촉·관절 한계 구속 제외, gear·방향별 forcerange) 0.67 — "도출값 ≤ 달성 가속" PASS. 첫 MuJoCo 실행의 12.8 배는 자기 충돌 접촉력이 섞인 검증 스크립트 결함이었다. pytest 15, 도구 mutation 6/6 red. `/code-review` finding 2건 (MuJoCo 한계의 gear·비대칭) 반영. **UR5e 값은 S0.7 가정 (ā = 10 rad/s²) 보다 크게 낮다** — 전 범위·전 속도라 가장 보수적이고 지배항이 중력이므로, S3.5a 대기 자세 주변으로 표본 범위를 좁혀 재생성한 값이 S4.4 입력이다 |
| 문서 (S2.4) | PASS | public header Doxygen (`axis_align.hpp`, `clik_reference.hpp`, `types.hpp`·builder, `catching_params.hpp`), README (rtc_math se3, rtc_tsid, rtc_urdf_bridge, integrated_bringup, rtc_controllers, rtc_tools, 루트·architecture 의존 그래프), L4 §4.5, L5 §5.1. `validate_docs` clean |
| GUI·plot (S2.4) | PASS | §13 S2 행 (2026-09-20, merge 후 main `d7715c90`, ur5e_p1b headless sim): 기동 로그에 `catch_frame` 추가 (full nq = nv = 26, actuated 16), DemoWbc 전환 후 `[wbc] CLIK reference enabled` (tip frame idx 0 — extra frame 이 기존 id 를 밀지 않음), overrun 0, `wbc_state` 발행. `demo_controller_gui --robot ur5e_p1b` 가 활성 컨트롤러·p1b 손 관절을 표시 (Xephyr 캡처 육안 확인). `wbc_diag.csv` 80627 행이 `plot_rtc_log` 로 solver·contacts 두 figure 로 그려진다. CSV 에 새 열을 더하지 않았으므로 plotter 변경은 없다. `test_demo_gui_*`·`test_plot_rtc_log.py` 는 전체 회귀에 포함 (integrated_bringup 1236, rtc_tools 698 green). CLIK 지표: `clik_valid` 100 %, `kin_qp_fail_count` 0 |

#### S3a 시뮬레이션 기반 (`rtc_mujoco_sim`, robot-agnostic)

> **2026-09-20 범위 축소 (사용자 결정).** S3a 는 **S3.1a · S3.2 · S3.3 · S3.4** 4 항목이다.
> - **S3.7 (팔 지연 에뮬레이션·식별) 제외** — **sim 은 지연이 없다고 보고 구현한다.** 주입도 에뮬레이션도 하지 않고 `arm_lag` 파라미터를 신설하지 않는다. 지연 식별은 실기 (S10, L5 §7 L5.9) 몫이고 게이트 `G5-D` 는 여기서 빠진다. **정정 (2026-09-24)**: "지연이 없다" 는 *에뮬레이션 지연을 넣지 않는다* 는 뜻으로만 참이다 — sim 팔 actuator 자체가 시정수 ≈ 200 ms 의 1차 지연이다 (L5 §4.4 정정, §4.4 S8).
> - **S3.8 (공 항력 k 식별) 제외** — 공 위치 예측은 `ball_perception` 이 준다. rtc 는 자체 탄도 모델로 예측하지 않으므로 `sim.ball.drag_k` 는 **fixture 전용 TBD** 로 남고 (L0 §7 이 이미 "fixture 가 실제로 필요해질 때까지 미뤄도 된다" 고 적었다) 게이트 `G0-D` 는 여기서 빠진다 — L0 §9 표에는 남되 fixture 가 필요해지는 시점 (S3.5a) 으로 이월한다.
> - **파급 (S5 착수 시 결정, S3a 는 막지 않는다):** `G5-E` (L5 §9) 와 `G8-E` (L8 §9) 는 둘 다 "**에뮬레이션 지연 하에서** 선행 보상 전후를 비교" 하는 게이트라 sim 에 지연이 없으면 **측정이 공허해진다**. 부수로 `planner.budget.sigma_trk` (L3 §6, "L5 실측") 의 sim 초기값 출처가 사라져 S10 까지 TBD 로 남는다. **가장 싼 완화** — 지연 주입을 출하 YAML 파라미터가 아니라 **테스트 fixture 전용**으로 두면 세 게이트가 살아남으면서 sim 런타임은 지연 0 을 유지한다 (`rtc_controllers` 의 `sim.ball.*` 이 이미 그런 fixture 전용 레인이다). **→ 이 완화로 확정 (2026-09-22 사용자, §7.3 G5-E substrate 행).** **정정 (2026-09-24, S8 준비)**: 전제 "sim 에 지연이 없다" 는 거짓이었다 — sim 팔 actuator 자체가 kd/kp = 0.2 s 의 1차 지연이다 (L5 §4.4 정정). G5-E 는 fixture 위 판정 (S5.3 PASS) 그대로이고, **G8-E 는 sim 런타임 lead on/off 로 판정한다** (D-S8-1, §4.4 S8)

- S3.1a D-3 무부하 검증 (§5) — 결과에 따라 D-3 재검토
- S3.2 발사 srv (D-14, S0.8 E-3 승인 후, PROC-3), iiwa7_leap projectile 설정, 투척용 스폰 위치. **world ↔ arm base 변환을 같은 q 의 MuJoCo FK·Pinocchio FK 대조로 확정**한다 (S3.5a 의 선행)
  - **이것은 신규 구현이 아니라 재설계다 (2026-09-20 코드 확인).** `rtc_mujoco_sim` 에 발사 계통이 이미 통째로 있다 — mjSpec freejoint 구·프리셋·이차항력+Magnus·노이즈 발사 샘플링·truth/카메라 발행·테스트 20 개 (§3 현황, `WORKSPACE_ANALYSIS.md` W6-1, `L8_bringup.md` §2 가 정확히 적고 있다). **없는 것은 명령 인터페이스 하나뿐**이다: `/sim/launch_ball`·`/sim/reset_ball` 이 `std_srvs/Trigger` 라 콜백이 request 를 **이름조차 받지 않고**, 발사 조건이 YAML + 시드 RNG 에서만 나와 D-14 의 `(p0, v0, ω)` 명시가 안 된다. 작업은 "`rtc_msgs` 에 srv 하나 + 기존 writer 로 가는 두 번째 경로" 이고 선례는 `rtc_msgs/srv/SetExternalWrench.srv` 다
  - iiwa7_leap 은 `projectile_ball:`·`object_state:` 설정이 **없고 공용 기본값도 없다** (기본 `enabled:false` 는 C++ 에서 온다) — ur5e 설정 복사가 아니라 LEAP 손가락 충돌 마스크까지 재유도해야 한다
- S3.3 공 접촉 truth(시각·충격량·접촉력) 출력, truth 발행 주기 상향, per-step `(sim_time, steady_now)` 진단 lane (§5 판정용)
- S3.4 `sim_estimator_node` 연결 — **측정만 한다** (정책은 S5.2): clock domain(`use_sim_time=false`), `frame_id` 와 world 관계, 발행 주기·N·지평 실측 (TBD-VIS-04/06), 구독 reliability 비교, validity 패턴 히스토그램, 재시작 시 `snapshot_sequence` 거동, 유령 트랙(관성 예측만 발행) 시 `validity` 거동, 지연·드롭 주입
  - **완료 2026-09-20.** 도구: `rtc_tools` `vision_lane_probe` / `camera_relay` / `analyze_vision_lane` (D-4 대로 필드 이름 디코딩, 다르면 거부). 프로파일: 0.8 s / 0.05 s / 16 점, `position_covariance_m2` 대각 2.5e-5 (sim 노이즈 5 mm), 입력 best_effort, `ros_system_time` + `use_sim_time:=false` (rtc 에 `/clock` 없음 → **설정으로 닫힘**). 풀 bring-up, 지정 발사 (`/sim/launch_ball_at`).
  - | 항목 | `ur5e_p1b` (20 발사) | `iiwa7_leap` (10 발사) |
    |---|---|---|
    | 발행 주기 · N · 지평 (TBD-VIS-04) | 30.0 Hz (p05 30.3 / p95 29.7) · **16** · 0.05…0.80 s | 30.0 Hz · 16 · 0.05…0.80 s |
    | stamp→수신 지연 | p50 32 / p95 41 / max 430 ms (30 Hz 주기 포함; estimator 처리 p50 13 µs / p95 663 µs) | — |
    | `frame_id` (TBD-VIS-06) | `world` ×856 | `world` ×427 |
    | validity | VALID 806 / 빈(INVALID clear) 50 — 비행당 ≈2.5 clear (발사 초기화 + 손 충돌 discontinuity + 회수) | VALID 393 / 빈 34 |
    | 공분산 NaN | 0 | 0 |
    | `snapshot_sequence` 되감김 / generation 변화 | 0 / 39 (≈2 per flight) | 0 / 29 |
    | 구독 reliability (TBD-VIS-08) | best_effort = reliable **identity 동일** 856/856, 편측 0 | 427/427, 편측 0 |
  - **유령 트랙 (TBD-VIS-07, `camera_relay --drop-after-s 0.4`, 10 비행)**: 공이 계속 나는데 입력이 끊기면 VALID 예측은 **다음 30 Hz tick 한 건 (≤34 ms)** 까지만, 이후 **침묵**. INVALID 스냅샷은 발행되지 않고 `track_status` 만 +100 ms COASTING (`coasting_timeout_s`), +500 ms LOST (`lost_timeout_s`) 로 diagnostics 에 나온다. ⇒ 소비자는 침묵을 소실로 읽어야 하며 (`io.t_stale`), validity 만 보면 안 된다
  - **지연 50 ms** (`--delay-s 0.05`, stamp 불변): 1404/1404 수용, stale 폐기 0, VALID 비율 불변 — capture stamp 기반이라 전송 지연은 흡수된다. **드롭 30 %** (`--drop-prob 0.3`): 1001/1400 수용, 초기화 횟수 불변 (2/비행), 발행 p95 가 15 Hz 로 얇아진다 (예측은 입력 step 마다)
  - **sim 재시작** (estimator 는 유지): 카메라 공백 11.8 s 에도 `clock_reset` **미발동**, `snapshot_sequence` 3→1352 단조, generation 연속 — stamp 가 wall `now()` 라 역행이 없다. 준비 세션이 예상한 "재시작 = clock_reset latch" 함정은 `/clock` + `use_sim_time` 전환 (S5/S6) 뒤에만 유효
  - ⚠️ **ball_perception 결함 (rtc 밖)**: `debug.enabled_topics` 에 `prediction/trajectory` 만 있으면 `needs_samples()` (`estimator_node.hpp:125`) 가 그 토픽을 빼놓아 샘플을 기록하지 않고, 토픽은 있는데 **발행이 0건**이다. 우회 = 다른 debug 토픽 동반. 그쪽 세션에 보고
  - 남은 것 (S3.6·S5.2): `io.n_min`·`io.t_stale`·`io.future_tol`·`io.horizon_min` 값, 되감김 정책 (되감김은 관측되지 않았다), validity 부분 수용 정책

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| e2e | 발사 → PointCloud2 수신 end-to-end, 같은 seed 재발사 시 truth 궤적 동일. **PASS 2026-09-20**: 두 로봇 모두 발사 → `prediction/trajectory` 수신 (p1b 856 · iiwa 427). 같은 seed (42) 로 sim 재시작 후 첫 발사 truth: `ur5e_p1b` 222 샘플 max \|Δ\| **7.9e-11 m**; `iiwa7_leap` 는 t = 0.9 s 까지 **0.0**, 제어 중인 팔에 맞고 튄 뒤 (t ≥ 1.33 s) 7.95 mm — 발사·자유비행은 동일하고 차이는 팔 제어의 run 간 비결정성 | **확정** |
| D-3 무부하 | §5 판정 (구성별 무효율 상한). **측정 완료 2026-09-20 (§5.1)**: 로봇 2종 × 200 발사, 거부 0, lane drop 0. δ_max max 는 `ur5e_p1b` 18.212 ms · `iiwa7_leap` 8.671 ms. 95 % 를 덮는 ε_clk,alloc 제안 **49.8 mm** (p1b 가 구속) | ε_clk 할당 (r_cap, TBD-HAND-04) → **r_cap 확정 후 §5.1 에서 재판정**: LEAP PASS, P1b PASS(provisional) (각 손의 $\Vert v\Vert_{\max}$ 기준). 할당 비율은 사용자 확정 2026-09-20 (§5.1) |
| frame | world ↔ base FK 대조 잔차 < 1e-6 m, 결과가 §11 에 기록됨. **측정 완료 2026-09-20 (§11)**: `iiwa7_leap` **PASS** (항등, 4.5e-16 m) · `ur5e_p1b` **FAIL** — 프레임은 **항등**이고 (2026-09-21 정정, §11: 처음 `Rz(180°)` 로 적힌 것은 180° 를 두 번 센 것이다) 잔차 8.3e-4 m (축선) / **1.46e-3 m** (`l_palm_link`·`tool0`) 는 MJCF↔URDF 치수 차이라 **sim 에서 줄일 수 없다**. 임계는 낮추지 않는다 — **사용자 결정 2026-09-20: `hand_description` 을 고치지 않고 sim 바닥값으로 받아 오차 예산의 모델 항으로 센다** | **확정** |
| PROC-3 | S3.2 의 `rtc_msgs` 변경 후 전체 빌드·테스트 — **PASS** (S3.2 직후 22 패키지 5147 tests; S3a 마감 시점 재실행 5198 tests, 0 failures) | **확정** |
| GUI·plot | §13 S3 행 — **PASS** (GUI 공 발사 패널 `test_demo_gui_ball_launch.py` 15, δ·pause 플롯 `analyze_clock_phase --plot`; §13 S3 행에 기록) | **확정** |

#### S4a 손 타이밍 측정

> **2026-09-20 착수 전 코드 대조·사용자 결정 (Q1~Q10 권장안 승인).** S4a 는 **S4.0 · S4.1 · S4.2 · S4.5** 4 항목이다.
> - **S4.3 (실기 T_close,tot) 은 S10 으로 이월** — S4.0 은 sim 전용이고, 실기에서 계단을 내려면 이 컨트롤러가 실기 팔을 hold 해야 해 E-8 승인 전 팔 명령 경로가 열린다. L6 §7 L6.5 가 이미 "아니면 S10" 을 허용한다. 게이트 G6-D 는 `NOT_EVALUATED(실기)` 이고 S4.4 는 sim 값으로 `PASS(provisional)` 판정한다 (§4.1)
> - **S4.5 는 `d_eff` 와 `r_cap` 을 둘 다 산정한다** — 둘은 같은 TBD-HAND-04 이지만 다른 양이고 (접근축 깊이 / L3 §4.6 게이트 우변의 포획 반경), D-3 판정 (§5.1) 을 여는 것은 `r_cap` 이다. 기하 추정은 preshape 자세의 MuJoCo FK 로 한다. **L6 §4.5 step 2 (sim 저속 투척 보정) 는 S7.1 이후로 이월** — 시퀀서 없이 러너의 wall 지연으로 계단을 쏘면 nrt 지터 × 공 속력이 `d_eff` 와 같은 자릿수다. 공 반지름 입력은 sim 공 (`radius_m` 0.025) 을 provisional 로 쓰고 `core.ball.diameter: 0.05` 로 기록한다 — D-12 공 사양 확정 시 재산정
> - **코드 대조로 정정된 서술**: (1) S4.1 은 백지가 아니다 — S1.7 의 `rtc::catching::HandProfile` 파서·검증기가 이미 있고 S4.1 은 `q_open`·`eta_close`·`T_close_e2e` 3 필드만 더한다 (`T_pre`·`T_hold`·`T_close_timeout`·`hold.*` 는 S7.1). 출하 `catching:` YAML 은 S4.1 이 첫 생산자다. (2) "CSV 만" 은 **새 CSV 가 아니다** — 기존 `DeviceStateLog` (`<hand>_state.csv`) 가 tick 마다 `command_*`·`actual_pos_*`·`t_relative_s` (tick 시작 steady clock, L6 §4.2 충족) 를 남긴다. ρ(t)·T_close 는 오프라인 분석기가 계산하고 C++ ρ 함수는 S7.1 이다. (3) G6-B 의 CM 반쪽 (`ControllerOutput` → `WriteCommand` 무성형 복사) 은 `test_rt_loop_pipeline` 이 이미 고정하므로 S4.0 은 바인딩 반쪽만 새로 단언한다. (4) L6.2 의 "≥20 회" 로는 99 % 를 말할 수 없어 **손당 200 회**로 한다. (5) sim lock-step 에서 steady 값은 호스트 스톨 (§5.1) 을 포함하므로 T_close 는 **steady 와 tick×dt 두 축**으로 보고한다 (YAML 에는 L6 정의대로 steady p99)

- S4.0 포구 컨트롤러 최소 골격 (S5.1 에서 앞당김): `demo_catching_controller` (`integrated_bringup`, YAML 은 `config/<robot>/controllers/demo_catching_controller.yaml` — L8 §5 가 S5.1 로 미뤘던 이름을 여기서 정한다). `RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG` 로 등록하고 YAML 은 `ur5e_p1b`·`iiwa7_leap` 에만 싣는다. 등록·lifecycle·config 로드와 **손 계단 진단 모드**만: 기존 손 `joint_goal` (`RobotTarget`) 을 YAML `diagnostic.hand_step: true` 일 때만 **무성형** (한계 clamp 만) 으로 손 device slot 에 통과시킨다 — `rtc_msgs` 변경 없음. 팔은 활성 첫 tick 자세 hold, 상태 메시지 없음. **sim 전용은 코드로 강제한다**: claim 한 device 의 `backend.type` 이 `mujoco_native` 가 아니면 **활성화를 거부한다** (configure 는 SUCCESS 를 내되 DISABLED 상태로 들어가 계단 구독·로그·프로파일 파라미터를 하나도 만들지 않는다). 이 가드는 S5.1 에서 E-8 승인과 함께 없앤다.
>   - **2026-09-21 정정 (코드 리뷰)**: 원래 결정은 "configure 를 거부" 였는데, 그러면 **로봇 전체가 못 뜬다** — sim 과 실기가 `config/<variant>/controllers/` 를 공유하므로 실기 p1b bring-up 도 이 컨트롤러를 인스턴스화하고, CM 은 **어느** 컨트롤러든 configure 가 실패하면 `bring_up_failed` 를 물고 전체 configure 를 거부한다. 의도는 "이 컨트롤러가 실기에서 돌면 안 된다" 였지 "실기 로봇이 뜨면 안 된다" 가 아니었다. 활성화 전에는 아무것도 명령되지 않으므로 활성화 거부가 그 의도를 그대로 담는다. E-STOP 훅은 base 기본 동작과 CM 측 hold 방어선에 맡긴다 — 팔 명령 경로·CLIK 앵커가 생기는 S5.1 이 E-8 대상이다
- S4.1 손 프로파일 YAML (P1b 10 DoF, LEAP 16 DoF): `robot.hand.{q_open,q_pre,q_close,caging_mask,eta_close,rho_eps}`, 전부 provisional. 자세는 에이전트 초안 (P1b 는 `force_pi_grasp` 자세, LEAP 은 FK 유도) → sim 에서 **사용자 확인 후** S4.2 측정 (TBD-HAND-05). **결과**: LEAP 은 사용자 제공 자세, P1b 는 사용자 제공 자세 두 벌이 출하 공을 파지하지 못해 그것을 seed 로 **탐색한 자세** (2026-09-21, L6 §4.5)
- S4.2 T_close 식별 도구 (`rtc_tools`): pre→close 계단 반복 러너 + `<hand>_state.csv` → ρ(t)·T_close,e2e(η) 분포 분석기 (sim, 손당 200 회)
- S4.5 `d_eff`·`r_cap` 산정 (L6 §4.5 step 1·3) — **완료 2026-09-20, 사용자 승인**. 기하만으로는 답이 안 나와 (빈 `q_close` 는 주먹이라 아무것도 둘러싸지 않는다) **접촉 시뮬레이션**으로 했다: 공을 preshape 손바닥에 놓고 닫은 뒤 ±g 로 흔든다. LEAP 793 중 216 파지 → `r_cap` 31.0 mm · `d_eff` 80 mm. **P1b 는 사용자 제공 자세로 851 중 0** — 손가락 하나가 ρ 0.01–0.15 에서 먼저 닿아 공을 밀어낸다 (반지름 20 mm 라야 잡힌다). 두 번째 사용자 자세와 손으로 만든 후보 40 여 개도 0 이어서 **2026-09-21 에 `q_pre`/`q_close` 20 개 값을 같은 시험에 대고 진화 탐색**했고 (사용자 자세가 seed), 탐색에 쓰지 않은 격자에서 445 중 214 파지 → `r_cap` 24 mm · `d_eff` ≥ 95 mm (스캔 상한), 정지 ρ 0.74 → η 0.7, `T_close_e2e` 재실측 280.5 ms (L6 §4.5). `r_cap` 으로 §5.1 D-3 를 재판정했다 (LEAP PASS, P1b PASS(provisional))

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 손 명령 | 손 계단 수락 tick 부터 손 device slot 명령 == clamp(목표) bit-equal (G6-B 바인딩 반쪽; CM 반쪽은 `test_rt_loop_pipeline`), S4.0 에서 팔 명령 변화 0, `Compute()` 할당 0 | — |
| 골격 | 두 sim 프로파일에서 configure→activate, `ur5e_p1a` bring-up 불변 (`test_registered_controllers_have_shipped_config`), 비-`mujoco_native` backend 에서 configure 거부, 진단 플래그 off 에서 손 목표 거부, 비활성 중 받은 목표가 재활성 첫 tick 에 안 쓰임 | — |
| 프로파일 | 출하 YAML 이 sim 구성 검증 에러 0, dof == 손 device 채널 수, 전 값이 YAML ∩ URDF 한계 안 | 사용자 자세 확인 |
| T_close | 두 손의 T_close,e2e(η) 분포(평균·최대·99%) 를 steady·tick×dt 두 축으로 산출, log drop 0 (G6-C 의 산출 부분) | — |
| d_eff·r_cap | G6-F: **LEAP·P1b PASS(provisional)** — 산정식·실측값·provisional 표시가 `planner.hand.*` 와 L6 §4.5 에 기록, D-3 갱신. P1b 는 탐색한 자세 기준 (2026-09-21). 투척 보정은 `NOT_EVALUATED` — S7.1 로 선행조건은 풀렸고 아직 하지 않았다 (sim S8 · 실기 S10) | 사용자 승인 2026-09-20 (LEAP) · 2026-09-21 (P1b 탐색 자세 적용) |
| GUI·plot | §13 S4 행 | — |

#### S3b·S4.4 지도·go/no-go·vision 사양

- S3.5a kinematic catchability 지도 (D-18, §11): 발사 영역 × 발사 속도·각도 격자 → 궤적 → 포구 후보 → S1.9 함수 (IK + w₅/w₆) → 잡을 수 있는 발사 조건 범위. 타이밍 값은 provisional
- S4.4 go/no-go: S3.5a 의 속도 범위, S4.2 T_close, S4.5 d_eff, S2.5 가속 box, S1.7 η_v 로 받을 수 있는 최대 공 속력을 계산해 목표 속도를 확정하거나 낮춘다
- S3.5b gate-catchable 지도: S4.4 값으로 전체 게이트 체인(IK + w + 도달시간 + γ 창 + 정지거리)을 다시 돌린다. 이 지도가 목표 투척 분포가 된다
- S3.6 vision 요구 사양 산출 (D-15): 목표 분포에서 "검출 이후 포구 창 종료까지 최대 비행 시간" → 필요 지평, L2 보간 게이트를 만족하는 간격 → 점 수 (**`n = ⌈H_req/간격⌉`, t = 0 없음 — §S0.7 산식 정정 2026-09-20**) → 런타임 `n_max` (≤ `kCap`, 넘으면 S1.2 backfill). 결과를 ball_perception sim profile 설정값으로 제시 (설정은 사용자)
- 사용자가 D-12 투척 분포를 동결한다

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| go/no-go | 목표 속도에서 γ 창이 비지 않음 (G6-C 의 판정 부분) | T_close (S4.2), d_eff (S4.5), 가속 box (S2.5), η_v (S1.7), 목표 속도 (S3.5a) — 하나라도 provisional 이면 PASS(provisional) |
| 지도 | 지도와 런타임 판정 동치: 같은 입력에서 S1.9 함수 판정이 같음 (G3-I), kinematic·gate 두 지도와 탈락 사유 분포 기록 | — |
| 사양 | S3.6 의 지평·간격·점 수·`n_max` 가 이 문서에 기록되고 `n_max ≤ kCap` | — |

**S4.4 결과 (2026-09-22) `[조건부 go — 사용자 결정]`.** 도구: `rtc_tools catch_speed_budget` (지도의 수락 후보별 팔 속도 LP·토크 한계 방향 가속·stroke → γ 창). 모든 값은 **낙관적 상한**이다 (bang-bang ramp, 자세를 q\* 에 고정, 도달 구) — 창이 비면 결론은 확정적이고 열리면 provisional 이며 S3.5b 가 닫는다.

- **순서 정리.** 후보별 $v_{dir,\max}$ 는 S3.5a 산출물 (`candidates.csv` 의 q\*·v̂) 과 관절 정격의 순수 함수라 S4.4 의 어떤 결정에도 의존하지 않는다 — S4.4 가 첫 산출물로 낸다 (§4.2 DAG 는 그대로). S3.5b 에는 "S4.4 확정값으로 전체 게이트 체인을 다시 돌린다" 만 남는다.
- **T_f ≥ 1.0 s·거리 4 m 는 제약이 아니다 (2026-09-21 사용자).** 둘은 궤적 추정기의 정확도를 확보하려던 값이다. 풀어도 결과는 같다: 포구 속력의 바닥은 비행시간을 최적으로 골라도 $\sqrt{g(R-\Delta z)}$ ($R$ = 릴리스 → 포구 직선거리, $\Delta z=z_c-z_0$) 이고, 현 sim 씬은 base 가 world z = 0 이라 포구점이 전부 **하강 구간** ($\Delta z\le-0.5$ m, 포구 속력 ≥ 3.0 m/s) 이다. 거리 1–4 m × 릴리스 1.0–2.0 m × T_f ≥ 0.25 s 의 9450 투척에서 창이 열린 것은 `ur5e_p1b` 1 (η_v = 1 에서만, 경계선) · `iiwa7_leap` 0.
- **팔.** 수락 자세에서 v̂ 방향 속력 상한 (LP, 정격, η_v 0.9) 의 중앙값은 `ur5e_p1b` 1.1–1.2 · `iiwa7_leap` 1.4 m/s (최대 3.5 / 1.8). w₅ 를 올린 포구 자세는 v̂ 로 빠른 자세가 아니다. 최소노름 (L3 §4.5 식) 은 LP 의 0.90 / 0.82 배 (중앙값) — 7축에서 버려지는 여유는 S6.3 의 입력.
- **가속은 물리적으로 구속하지 않는다 — D-16 box 가 구속한다.** 토크 한계로 직접 푼 방향 가속 ($|M\ddot q+h|\le0.8\tau_{\max}$, 접근축 유지, 회전자 관성 포함) 은 중앙값 43 / 79 m/s² 인데 같은 자세에서 box 가 주는 값은 0.74 / 6.8 m/s² 다 (§9). 토크 한계 층의 판정표는 속도 상한만 쓴 층과 거의 같다.
- **손.** 시각 발동 fly-in 으로 잰 상대속도 허용량은 두 손 모두 약 **1.0 m/s** (L6 §4.5) — 공식값 0.34 / 0.76 m/s 보다 크다. 그래도 표는 안 열린다: $\Delta z\le-0.5$ m 구간은 허용량 3–4 m/s 에서야 10–50 % 가 열린다 (= $T_{close}$ 25–30 ms).
- **조건부 go 의 세 조건.**
  1. **낙차** — 포구점이 릴리스 높이 −0.25 m 이상. 결과는 (릴리스 높이 − base 높이) 에만 의존하고 (병진 불변이 데이터에서 그대로 재현된다) 최적은 0…+0.2 m: **base 높이 부근에서 위로 던져 정점 근처에서 받는다.**
  2. **가속 한계** — 상수 box 의 가중 최적화로는 부족하다 (`ur5e_p1b` 방향 가속 중앙값 1.95 → 5.1 m/s², p05 0.6; `iiwa7_leap` 8.9 → 15.9, p05 4.5). `ur5e_p1b` 는 자세·방향 의존 한계 (계획기가 후보마다 토크 한계를 검사) 가 필요하다. 구현은 S6.
  3. **선행시간** — 최소 비행시간은 $T_{det}+L+T_{close,tot}+T_{arm}+T_{margin}$ (S0.7 R1) 이고 **손 폐쇄 시간이 그대로 들어간다**: `ur5e_p1b` 0.59 s · `iiwa7_leap` 0.41 s. 열리는 투척의 T_f 는 0.3–0.8 s 에 몰려 있어 이 하한이 지배 지렛대다 (base 1.0 m, /3808: T_f 하한 0.3 → 0.6 s 에서 `ur5e_p1b` 147 → 11, `iiwa7_leap` 112 → 3).
- **열리는 영역 (base 1.0 m 가정 씬, 상대속도 1.0 m/s, 정격 속도, `catch_speed_budget` 재현값).** `iiwa7_leap` **39 / 3808** — 거리 1.0–1.5 m, 발사 2.0–5.25 m/s, 앙각 55–75°, T_f 0.45–0.60 s, 포구 속력 1.2–2.6 m/s. `ur5e_p1b` **11 / 3808** — 거리 1.0–1.5 m, 발사 4.25–5.5 m/s, 앙각 55–70°, T_f 0.60–0.80 s, 포구 속력 2.7–3.8 m/s. `ur5e_p1b` 는 관절 속도를 현 config (2.0/3.0 rad/s) 로 두면 0–1 이다. 거리 ≥ 2.0 m 는 0.
- ⚠️ **정정 (2026-09-22, S3.5b).** 위 조건 3 의 하한은 S0.7 의 **R1 (commit)** 만 담았고 **R2 (대기 자세 → q\* 이동, L3 §4.3)** 를 뺐다 — S0.7 은 "sweep 전 구간에서 R2 가 지배한다" 고 적어 두었다. 위 39 / 11 은 선언대로 상한이지만, 도달시간 게이트를 얹으면 **출하 대기 자세로는 두 로봇 모두 0** 이고 (`ur5e_p1b` 는 `wrist_2` 가 열린 q\* 전부에서 1.72 rad 떨어져 있다 — 대기 자세가 4 m lob 용이다), 아래 ① 의 "1차 목표 `iiwa7_leap`" 권장은 **철회**한다: `iiwa7_leap` 은 팔이 느려 느린 공 = 짧은 비행 (T_f 0.45–0.60 s) 만 열리고 그 비행에는 팔을 옮길 시간이 0.12–0.2 s 뿐이다. 또 ② 의 "`rtc_mujoco_sim` 쪽 wrapper scene" 은 `ur5e_p1b` 에서 **불가**다 (hand-description `scene_with_table.xml` 머리 주석의 실측 2건 — MJCF `<include>` 는 resource provider 를 타지 않고 `meshdir` 는 top-level 파일 기준으로 풀린다). 대신 **sim 은 씬을 바꾸지 않아도 된다** — 결과는 (릴리스 − base 높이) 에만 의존하고 열린 포구점은 전부 base 위 0.12–0.99 m 라 바닥이 구속하지 않으므로, 현 씬에서 릴리스를 0.2–0.5 m 로 두면 같은 셀이다. 아래 제안 넷의 확정 내용은 §7.3, 그 결과는 "S3.5b 결과".
- **`[제안]` — 사용자 확정 대상 (2026-09-22 확정·일부 철회 — 위 정정과 §7.3).** ① `iiwa7_leap` 을 1차 목표로, `ur5e_p1b` 는 $T_{close}$ 단축 (폐쇄 병목은 `thumb_cmc_fe` 이동량 1.57 rad — L6 §4.2) 과 관절 속도 config 상향이 선행된 뒤. ② D-18 개정: base 0.8–1.0 m, 릴리스 = base + 0…0.2 m, 거리 1.0–1.5 m, 앙각 55–75°, T_f 0.45–0.8 s (씬은 `rtc_mujoco_sim` 쪽 wrapper — `hand_description` 은 고치지 않는다). ③ D-16 개정: §9. ④ `reference.v_max` 는 실측이 아니라 **도출값** (수락 후보의 LP $v_{dir,\max}$ 최대) 으로 두고 η_v 를 관절 속도 한계에도 적용 (L3 §4.5, L4 §6).
- 재현 스크립트·원자료 (씬 sweep, fly-in, box 재도출, 프로토타입) 는 repo 에 두지 않는다. 방법은 위 식과 `catch_speed_budget` 의 인자가 SSoT 다.

**S3.5b 결과 (2026-09-22) `[ur5e_p1b PASS(provisional) · iiwa7_leap 지도 빔 — S3.6 로 넘어가지 않는다]`.** 도구: `rtc_controllers catch_gate_batch` (후보 CSV → `time_feasibility.hpp` 의 런타임 함수 그대로 도달시간·γ 창·정지점) + `rtc_tools catch_gate_map` (그 입력·출력, commit 선행, 토크 검사 도달시간 층, 집계). 설정: **현 sim 씬, 릴리스 0.2–0.5 m** (결정 A), 관절 속도 정격 × η_v 0.9, `reference.v_max` 도출값 (`ur5e_p1b` 3.9 · `iiwa7_leap` 2.0 m/s, 결정 D), 가속 box 는 출하값, 회전자 관성은 MJCF `armature`, 첫 계획 시각 T_det + L = 0.24 s (S0.7 **가정값**), T_arm 0.05 s, `planner.time.margin` 0.03 s, `planner.gamma.margin` 0.1 m/s, **`supervisor.decel.a_dec` 10 m/s² (provisional — 키가 TBD)**. rollout (L3 §4.8) 은 `NOT_EVALUATED(S6)` (결정 E).

- **게이트 순서와 층.** commit 선행 ($t_c-t_{plan}\ge T_{close,tot}+T_{arm}+T_{margin}$, L3 §4.11 — 런타임 함수가 아직 없어 python) → 도달시간 → γ 창 → 정지점 (도달 구·바닥 — `planner.workspace.catch_box` 가 TBD). 도달시간은 **두 층** (결정 B): `box` = 출하 상수 box 로 C++ 판정, `torque` = 그 이동 wait → q\* 를 하나의 bang-bang/사다리꼴 경로 프로파일로 보내며 경로 전체에서 $|M\ddot q+h|\le\eta_\tau\tau_{\max}$ 인 최대 경로 가속을 이분 탐색 (충분조건, 런타임 대응물 없음). 손 폐쇄는 **두 값**으로 돌렸다: 공식 `planner.hand.d_eff` (0.095 / 0.080 m → 상대속도 0.34 / 0.76 m/s) 와 시각 발동 fly-in 실측 상대속도 1.0 m/s 에 해당하는 등가 $d_{eff}=v_{rel}T_{close,tot}$ (0.2815 / 0.1047 m — L6 §4.5).
- **G3-I (지도 = 런타임 판정).** 도달시간·γ 창·정지점은 같은 함수다 (C++ 실행파일, 열 bit-exact·순서 불변 테스트). **q̇ᵘ (v_dir,max 의 DLS 단위 속도) 의 생산자는 런타임에 아직 없어** python 이 만들어 넘긴다 — 그 부분의 동치는 `NOT_EVALUATED(S6.2)`.
- **넓은 격자 (거리 1.0–1.5 m × 릴리스 0.2–0.5 m × 2.0–6.0 m/s × 앙각 40–80° = 1377 투척, 단일 대기 자세).**

| | `ur5e_p1b` | `iiwa7_leap` |
|---|---|---|
| kinematic 수락 투척 (출하 대기 자세 → 다시 고른 자세) | 786 → 767 | 508 → 374 |
| **공식 `d_eff`**: 도달시간을 뺀 나머지를 다 통과한 후보 | **0** (대기 자세와 무관 — **확정적으로 빔**) | 5 → 3, 열린 투척 0 |
| fly-in 상대속도 1.0 m/s: 같은 후보 | 11 | 26 → 16 |
| 열린 투척 `box` / `torque`, 출하 대기 자세 | 0 / 0 | 0 / 0 |
| 열린 투척 `box` / `torque`, 다시 고른 대기 자세 | 0 / **9** | 0 / 0 |

- **기준 투척 주변의 허용 오차 상자 (세밀 격자 2835 투척: 거리 ±0.1 m, 릴리스 ±0.05 m, 방향 편차 ±6°, 속력 ±0.4 m/s (0.1 간격), 앙각 ±6° (2° 간격), fly-in 상대속도).**
  - `ur5e_p1b` — 기준 투척 **거리 1.0 m · 릴리스 0.2 m · 4.75 m/s · 앙각 60°**, 대기 자세 `[0.212, −1.376, 1.107, −1.978, −3.296, 0.121]` (그 투척의 q\*; 다음 라운드 제안과 0.11 rad 안). `torque` 층 **590 / 2835 (20.8 %)**, `box` 층 6. **90 % 이상 열리는 상자: 거리 0.9–1.0 m · 릴리스 0.15–0.25 m · 방향 편차 ±6° · 속력 4.65–4.85 m/s · 앙각 62–64°** (격자 180 투척 중 90.6 %). 열린 후보는 T_f 0.61–0.80 s, 포구 속력 2.7–3.85 m/s, 포구점은 릴리스보다 0.44–0.71 m 위, γ 창 폭 중앙값 0.12, 도달시간 여유 중앙값 0.20 s. 속력 ±0.1 m/s · 앙각 ±1° 는 사람이 아니라 **sim 발사기 (`/sim/launch_ball_at`) 의 허용 오차**다.
  - `iiwa7_leap` — 기준 투척 거리 1.25 m · 릴리스 0.2 m · 3.25 m/s · 앙각 70°. 대기 자세를 세 라운드 다시 골라도 **0**. 도달시간 예산이 0.12–0.15 s 인데 토크 층 최소 이동 시간이 0.13 s 부터다 (최선 여유 −0.004 s). 대기 자세는 수렴하지 않는다 — 7축에서 영공간 log w₅ 상승 (D-25) 이 q\* 를 seed 에서 0.35–0.5 rad 씩 옮기고, 그사이 나머지 게이트를 통과하는 후보가 1026 → 341 → 124 로 준다.
  - **`iiwa7_leap` 을 막는 것은 선행시간이다.** 같은 지도에서 첫 계획 시각만 0.24 → **0.14 s** 로 당기면 `torque` 층 **65 / 2835** 가 열린다 (T_f 0.42–0.47 s, 포구 속력 1.6–2.0 m/s, 여유 중앙값 0.04 s). T_det 0.10 · L 0.14 s 는 가정값이고 sim 실측이 없다.
- **각 게이트가 단독으로 거르는 후보 (`ur5e_p1b`, 넓은 격자, 다시 고른 자세, fly-in, 3838 후보).** commit 2289 · 도달시간 `box` 3813 / `torque` 3150 · γ 창 3400 · 정지점 1524. 게이트 하나를 풀어서는 열리지 않는다.
- **읽는 법.**
  1. **열린 것은 전부 fly-in 실측 상대속도 위에 서 있다.** 런타임 γ 창은 `planner.hand.d_eff / T_close,tot` 를 쓰므로, 출하 `d_eff` (0.095 m) 로는 `ur5e_p1b` 지도가 비어 있다. 실측을 게이트에 싣려면 `d_eff` 의 뜻 (TBD-HAND-04) 을 "포켓 깊이" 에서 "fly-in 으로 잰 허용 상대속도 × $T_{close,tot}$" 로 바꾸는 결정이 필요하다 (§7.3).
  2. **출하 가속 box 로는 열리지 않는다** (`box` 층 0–6). D-16 개정의 런타임 구현 (S6) 이 선행 조건이다.
  3. **대기 자세를 다시 골라야 한다.** 출하 제안값 (§11) 은 4 m lob 용이고 `wrist_2` 가 1.7 rad 떨어져 있다. `ur5e_p1b` 의 새 제안은 위 자세 (provisional).
  4. `a_dec` 10 m/s² 는 provisional 이고 정지점 게이트가 단독으로 후보의 40 % 를 거른다 — `supervisor.decel.a_dec` 값이 필요하다.
  5. 최소노름 v_dir,max (런타임 식) 는 LP 보다 작다 — 열린 `ur5e_p1b` 후보에서 중앙값 0.97 배 (최소 0.86), γ 창 폭 중앙값이 0.12 라 S6.3 의 여유 회수가 곧 허용 오차다.
  6. `ur5e_p1b` 의 D-3 은 3.85 m/s 에서 여전히 FAIL 이다 (§5.1).
- **1차 목표 로봇 (결정 C 의 입력).** 이 결과로는 `ur5e_p1b` 만 목표 투척 분포를 가진다. `iiwa7_leap` 은 선행시간 실측 (T_det·L) 이 0.14 s 근처로 나오거나 q\* 의 seed 의존을 줄이지 않는 한 비어 있다. **확정 (2026-09-22 사용자): `ur5e_p1b` 1차, G8-D2 조건부** (§1a, §7.3).
- **위 결과에서 사용자가 확정한 것 (2026-09-22).** ① `planner.hand.d_eff` 는 **허용 상대속도 × $T_{close,tot}$** (fly-in 등가값 0.2815 / 0.1047 m) 로 정의한다 — 지도가 쓴 정의 그대로, 런타임 식·코드 변경 없음 (L3 §4.5, L6 §4.5). ② `supervisor.decel.a_dec` **10 m/s²** (provisional — `reference.a_max` 확정 시 ≤ 검사). ③ 1차 로봇 `ur5e_p1b`, G8-D2 조건부 (§1a). ④ 선행시간 실측은 S3.6 을 막지 않는다 — T_det 만 지금 S3.4 rig 로 잴 수 있고 L 은 계획기가 생기는 S5–S6 후에야 값이 있다; 별도 task 로 병행.
- S3.6 은 위 결정 후 착수한다 — 이 시점에서는 `[SPRINT]` 3 의 정지 조건 (`iiwa7_leap` 지도가 빔) 으로 시작하지 않았다. 원자료와 러너 (`run_gate.sh`) 는 repo 에 두지 않는다; 방법은 두 도구의 인자가 SSoT 다.

**S3.6 결과 (2026-09-22) `[PASS(provisional) — ur5e_p1b · iiwa7_leap NOT_EVALUATED(선행시간)]`.** 도구: `rtc_tools catch_gate_map` 요약에 층별 **열린 후보의 t_c(= 비행시간)·포구 속력·투척별 포구 창 [min t_c, max t_c] 의 min/p50/max** (`open_candidates`) 를 추가하고, S3.5b 와 같은 인자 (현 sim 씬 · 대기 자세 `[0.212, −1.376, 1.107, −1.978, −3.296, 0.121]` · `d_eff` 0.2815 m · $T_{close,tot}$ 0.2815 s (`close_total_s` — P1b 는 fly-in 상대속도가 1.0 m/s 라 두 수가 우연히 같다) · `a_dec` 10 · 첫 계획 0.24 s · T_arm 0.05 · `planner.time.margin` 0.03) 로 두 격자를 다시 만들었다. 지평 요구는 **R1** 이고 R2 로 잡지 않는다 (§7.1); 예측점은 `step, …, horizon` 이라 t = 0 이 없다 (n = ⌈H/step⌉).

- **입력 (90 % 상자 180 투척: 거리 0.9–1.0 m · 릴리스 0.15–0.25 m · 방향 ±6° · 속력 4.65–4.85 m/s · 앙각 62–64°).** kinematic 178, `torque` 층 열린 투척 **163 / 180 (90.6 %, S3.5b 와 동일)**, `box` 층 3. 열린 후보 326 개: t_c 0.612 / 0.660 / 0.732 s (min / p50 / max), 포구 속력 2.72 / 3.17 / 3.56 m/s, 투척별 포구 창 시작 0.612–0.732 s · **끝 0.636 / 0.684 / 0.732 s**. 세밀 격자 2835 (재현): `torque` **590 / `box` 6 — S3.5b 와 동일**, 열린 후보 967 개, t_c 0.612 / 0.684 / **0.804** s, 포구 속력 2.72 / 3.20 / 3.85 m/s (S3.5b 본문의 "T_f 0.61–0.80 s · 2.7–3.85 m/s" 가 이 값이다).
- **기구학 reachable 창 (D-27 기준 — 이것이 요구다).** 실행은 게이트 교집합이 아니라 **기구학적으로 도달 가능한 후보면 도전**하므로, 지평은 그 후보들이 계획기에 도달할 만큼 길어야 한다. 같은 두 격자의 kinematic 후보 전체 (게이트 필터 없이, `gate_map.csv` 의 모든 행) 에서 투척별 포구 창 끝: 상자 0.732 / 0.852 / **0.876** s (min / median / max, 178 투척 1467 후보), 세밀 격자 0.396 / 0.804 / **0.948** s (2399 투척 16028 후보). commit 선행만 걸어도 (손을 명령할 plan 이 아예 있어야 한다는 물리적 하한) 최댓값은 같다 (0.876 / 0.948 s).
- **산식과 값.** 검출 시각 T_det 0.10 s (S0.7 가정값) 의 첫 예측이 포구 창 끝까지 보여야 첫 plan 부터 그 후보를 고를 수 있다: $H_{req} = \max t_c - T_{det} + t_{horizon\_margin} + T_{margin}$ (= `prediction.t_horizon_margin` 0.05, L2 §4.6 — 계획기는 마지막 점 − 0.05 s 안에서만 고른다; + `planner.time.margin` 0.03). **기구학 창 (요구)**: T_det 는 **실측값**을 쓴다 (아래 "T_det 실측") — 구속하는 것은 **가장 이른 검출**이다 (검출이 이를수록 첫 예측의 origin 이 이르고 그만큼 더 멀리 봐야 한다): 0.948 − 0.040 (재실측 min) + 0.08 = **0.988 s** → **n = ⌈0.988 / 0.05⌉ = 20** (90 % 상자만이면 0.916 s · n 19). **gate 통과 창 (하한·참고)**: 0.804 − 0.040 + 0.08 = 0.844 s → n 17. 이력: 가정 T_det 0.10 s 로는 0.928 s · n 19, 공 lane stamp 수정 전의 첫 실측 (p05 0.027 s — 버스트가 만든 값) 으로는 1.001 s · n 21 이었다 — 실측이 더 이르게 검출해 요구가 **올랐고**, stamp 수정으로 가장 이른 검출이 0.040 s 가 되어 한 점 내려왔다. 간격은 S1.2 실측 (0.05 s 간격 보간 오차 2.0e-11 m ≪ G2-C 1e-10 m, §4.4 S1 결과) 으로 **0.05 s** 를 유지한다 — 더 촘촘하면 점 수만 는다. ball_perception 은 `horizon % step == 0`·`horizon / step ≤ max_points` 를 강제하고 producer 상한은 1000 점이라 사양을 구속하지 않는다.
- **sim profile (D-15): 1.0 s / 0.05 s / 20 점 / ≤ 30 Hz — 설정됨 (2026-09-22 사용자 결정 ⑥).** 파일은 `integrated_bringup/config/ur5e_p1b/ball_perception_sim_profile.json` (`sim_estimator.launch.py profile_path:=` 로 준다; ball_perception 은 `horizon % step == 0`·`horizon / step ≤ max_points` 를 요구하고 producer 상한 `kMaxPredictionPoints` 는 1000). 종전 0.8 s / 16 점을 유지하면 계획기가 보는 창 끝은 0.78 s 까지이므로 (지평 − `t_horizon_margin`) **기구학 창의 끝 (0.95 s) 쪽 후보 = 늦게 잡는 자세는 시도 대상에서 조용히 빠진다** — D-27 이 피하려는 바로 그 손실이다. 비용은 예측점 4 개 (스냅샷 복사 약 +0.8 KB/tick, `kCap` 40 안). H_req 는 T_det 에 1:1 로 민감하다 ($H_{req} = 1.028 - T_{det}$): 가장 이른 검출 0.040 s 기준으로 지평 H 가 덮는 t_c 는 $0.040 + H - 0.08$ 이라 **1.0 s / 20 점이 창 끝 0.948 s 를 12 ms 여유로 덮고**, 0.95 s / 19 점은 38 ms 모자란다. 공 lane stamp 수정 전에는 버스트가 만든 0.024 s 검출 때문에 1.05 s / 21 점이 필요했다 — 그래서 ⑤ 를 먼저 고치고 profile 을 한 번만 정했다. 확인 CLI: `ros2 run rtc_tools vision_lane_probe <prefix>` 로 비행 몇 건을 기록한 뒤 `ros2 run rtc_tools analyze_vision_lane <prefix>` 의 발행 주기·N·지평 행이 30 Hz · 20 · 0.05…1.0 s 인지 본다. **이 profile 은 2026-09-22 재실측 run 이 그대로 썼다** — 정확히 20 점 · 지평 0.05…1.0 s · VALID 2123 건 · 부분 무효 0 (아래 "T_det 재실측").
- **런타임 값 (provisional — 기록처는 L1 §6 · L2 §6).** `io.horizon_min` = R1 하한 $T_{freeze} + L$ = ($T_{close,tot}$ 0.2815 + T_arm 0.05 + T_margin 0.03) + L 0.14 = 0.5015 → **0.51 s** (10 ms 로 **올림** — 내림하면 R1 을 1.5 ms 미달하는 궤적이 통과한다; L 0.14 s 는 S0.7 가정값); `io.n_min` = ⌈0.51 / 0.05⌉ = **11** (0.05 s 간격에서 0.55 s); `prediction.dt_expected` **0.05 s**; **`n_max` = 20** — 설정한 profile 의 점 수와 같게 둔다 (런타임 상한이 profile 을 거부하면 안 된다). **20 ≤ `kCap` 40 → S1.2 backfill PASS** (§4.3 S1 행). `io.t_stale` **[제안] 0.10 s**·`io.future_tol` **[제안] 1e-3 s** 는 S3.4 실측 (발행 30 Hz, 드롭 30 % 에서 p95 15 Hz, 유령 트랙 침묵 ≤ 34 ms 뒤 정지, 같은 호스트 wall clock) 기반의 제안이고 **확정은 S5.2** (L1 §6 근거 칸).
- **한계.** ① T_det 는 **sim 실측** (아래) 이고 L 0.14 s 는 여전히 가정값이다 — `io.horizon_min` 은 L 에 걸려 있고 H_req 는 T_det 에 걸려 있다 (L 은 S5–S6 후). ② 목표 분포 자체가 provisional 셋 (fly-in `d_eff`·토크 층·다시 고른 대기 자세) 위의 값이다 (§4.4 S3.5b 결과). ③ **H_req 는 "첫 예측이 창 끝까지 본다" 는 보수적 기준이다.** 연속 재계획에서는 후보가 더 늦게 지평에 들어와도 되고 (지평 요구를 R1 로 둔 2026-09-19 결정), 도달시간도 정지 출발이 아니라 현재 명령 상태에서 재므로 (L3 §4.3) 실제 필요 지평은 이보다 작을 수 있다 — 이 기준을 택한 이유는 D-27 이고 (첫 plan 부터 시도할 수 있어야 한다) 비용이 예측점 몇 개뿐이기 때문이다. R2 (대기 자세 정지 출발) 는 요구가 아니다. ④ `iiwa7_leap` 은 지도가 비어 `NOT_EVALUATED(선행시간)` — 실측 선행시간에서 지도가 열리면 같은 요약 블록으로 H_req 를 다시 읽는다 (G8-D2 조건부, §1a). ⑤ 요약이 주는 것은 분포뿐이고 H_req 의 덧셈은 이 문서가 한다 — 도구에 T_det·margin 을 넣지 않은 것은 두 값이 이 문서의 결정이기 때문이다. ⑥ 기구학 창의 끝은 **이 격자·이 대기 자세·`max_reach` 1.1 m·`min_catch_height` 0.1 m** 에서의 값이다 (지도 창 0.3–1.4 s 안이라 잘리지 않았다) — 투척 격자를 넓히면 다시 읽는다. ⑦ D-27 의 "성능 게이트를 우선순위로" 는 런타임 구현이 없다 (S6) — 지금 바뀐 것은 **지평 요구를 어느 창에서 읽는가** 뿐이고, 어느 게이트가 판정으로 남는지는 S6 설계에서 정한다.
- 원자료 (두 격자의 `gate_map.csv`·요약) 는 repo 에 두지 않는다; 재현은 §4.4 S3.5b 결과의 인자 + 위 격자 정의로 한다.

**T_det 실측 (2026-09-22, S3.6 병행 task) `[sim 실측 — L 은 여전히 가정값]`.** 도구: `rtc_tools vision_lane_probe` + `analyze_vision_lane` 의 `detection_latencies` (S3.4 rig 에 추가). 설정: `ur5e_p1b` headless 풀 bring-up (`enable_viewer:=false`, `max_rtf` 1.0) + ball_perception `sim_estimator_node` (profile 0.95 s / 0.05 s / 19 점 / 30 Hz, 측정 공분산 2.5e-5 m² 대각), `/sim/launch_ball_at` (D-14) 로 **기준 투척 24 발** (거리 1.0 m · 릴리스 0.2 m · 4.75 m/s · 앙각 60°, aim 0 — `catchability_map` 의 Throw 규약 그대로: `p₀` (1.0, 0, 0.2), `v₀` (−2.375, 0, 4.1136)).

- **정의.** T_det = 발사 → 첫 VALID 예측. 발사 시각은 **그 비행의 첫 ground-truth 샘플**이다 — 시뮬레이터는 공이 park 상태면 truth·camera 를 아예 발행하지 않으므로 (`PublishProjectileBall`) truth 의 침묵이 비행을 가른다. 양자화는 공 발행 주기 하나 (`publish.sample_rate_hz` 100 Hz). 두 축을 **각각 한 시계 안에서** 낸다 (D-2): `recv` (프로브 steady, 전송 포함) 와 `stamp` (발행자 stamp, 전송 제외). 발사 전부터 VALID 이던 트랙 (유령, TBD-VIS-07) 은 generation 이 그때 것이라 검출로 세지 않는다.
- **결과 (24 비행, 전부 검출, 미검출 0).** 단위 s.

| 축 | min | p05 | p50 | p95 | max |
|---|---|---|---|---|---|
| `recv` (소비자 체감) | 0.056 | 0.059 | 0.133 | 0.363 | 0.729 |
| `stamp` (발행자, H_req 입력) | 0.024 | 0.027 | 0.075 | 0.322 | 0.690 |

- **H_req 에 들어가는 것은 `stamp` 축이다** — 예측의 지평은 자기 origin stamp 기준이고, L (= L_vis + T_pub + T_plan + margin) 이 전송·발행을 이미 센다. `recv` 를 쓰면 L_vis·T_pub 을 두 번 센다. 그리고 **구속하는 것은 가장 이른 검출** (p05 0.027 s) 이다 — 위 "산식과 값".
- **S0.7 가정값 0.10 s 는 중앙값 근처였다** (`stamp` p50 0.075 · `recv` p50 0.133). 첫 계획 시각 T_det + L 0.14 로 보면 **p50 0.215 s 로 지도가 쓴 0.24 s 와 사실상 같다** — S3.5b·S4.4 의 지도 판정은 중앙값 기준으로는 유효하다. 다만 min 0.164 · p95 0.462 · max 0.830 s 로 **꼬리가 크다**.
- ⚠️ **꼬리의 원인은 vision 이 아니라 sim 공 lane 이다 (rtc 쪽 rig 결함).** `max_rtf` 1.0 · RTF 1.0x 인데도 공 lane 의 **wall stamp 간격**이 p05 2.7 / p50 10.0 / p95 33.6 / max 52 ms 로 흔들리고 **9.2 % 가 25 ms 를 넘는다**. 발행 gate 는 **sim time** 기준인데 (`ShouldPublishProjectileBallSample`) stamp 는 **wall `now()`** 라 stepper 의 버스트가 그대로 stamp 에 실린다. ball_perception 의 init 은 `min_points` 5 를 `max_span_s` 0.1 s 안에 요구하므로 간격이 25 ms 를 넘으면 초기화가 실패·재시도한다 — 그래서 T_det 이 0.69 s 까지 늘어진다. 균일한 실기 카메라에서는 나지 않는 모드다. **해소 (2026-09-22 사용자 결정 ⑤, 같은 날 구현)**: 진짜 결함은 간격이 아니라 **stamp 가 샘플 시각이 아니라는 것** (`stamp_is_capture_time: true` 전제가 이 lane 에서 거짓) 이었다. 발행률 상향은 한 번 깨어난 사이의 두 샘플이 거의 같은 stamp 에 다른 위치를 실어 속도 추정을 더 튀게 하므로 철회했고, wall 타이머 발행은 `mjData` 를 다른 스레드에서 읽어야 해 기각, `init.max_span_s` 완화는 증상만 덮어 기각. 채택: **두 공 토픽의 stamp 를 발사 순간을 기준으로 sim 시간축을 wall 에 얹은 값으로** (`ProjectileBallSample::stamp_steady_ns`, `rtc_mujoco_sim` — epoch 은 wall 그대로라 D-2·D-3 불변, 발사 전에 쌓인 sim↔wall 지연은 비행의 stamp 에 들어가지 않는다 — 남는 것은 비행 안의 위상 오차 δ 뿐 (양방향: stepper 가 따라잡는 동안은 stamp 가 wall 을 수십 ms 앞선다) 이고 그것이 D-3 가 재는 양. 그래서 sim 에서는 소비자의 future 허용치가 그 폭을 덮어야 한다 — profile `max_future_skew_s` 0.1, `io.future_tol` 은 sim overlay 0.1 (L1 §6); README §Projectile Ball stamp). 재실측은 아래.
- **부수 확인 — 0.95 s / 19 점 profile 이 실제로 돈다.** 같은 run 에서 예측은 정확히 **19 점 · 지평 0.05…0.95 s · VALID 944 건 · 부분 무효 0** 이었다 (빈 INVALID clear 1343 건은 대부분 착지·바운드 뒤의 재초기화다 — 이 투척은 2 s 동안 공을 살려 둔다). 권장 profile 이 1.05 s / 21 점으로 올라갔지만, 0.95 s / 19 점이 구성·발행된다는 것은 확인됐다.
- **실측 lead 로 `ur5e_p1b` gate 지도를 다시 돌렸다** (`catch_gate_map`, 같은 인자에 `--detection-s 0.075 --latency-s 0.14` = 첫 계획 0.215 s). kinematic 은 2399 투척 / 16028 후보로 **종전과 완전히 같고** (설정이 그대로라는 확인), 열린 투척은 `torque` **652 / 2835 (23.0 %)** · `box` 9 — 가정값 0.24 s 의 590 / 6 보다 낫다.
- **첫 계획 시각 민감도 (같은 지도, `gate_map.csv` 에서 직접).** 이 지도의 두 게이트만 first_plan 에 의존하고 (도달 예산·commit 여유가 같은 상수만큼 평행이동) γ·정지점·토크 도달 *시간* 은 무관하므로, 한 번 만든 지도에서 정확히 재계산된다 — 0.24 s 에서 590 / 6 이 나와 종전 실행과 일치하는 것이 그 검증이다.

| 첫 계획 [s] | 0.164 (T_det min) | 0.215 (p50) | 0.240 (가정) | 0.30 | 0.40 | 0.44 | 0.45 |
|---|---|---|---|---|---|---|---|
| 열린 투척 `torque` | 733 | 652 | 590 | 353 | 45 | 10 | **0** |

- **이 표는 하한이다 — 연속 재계획이 들어 있지 않다.** 지도는 **대기 자세에서 정지 상태로 단 한 번** 계획하는 것으로 판정하지만 (`catch_gate_map` 머리말, 토크 층도 rest-to-rest), 런타임은 도달시간을 **현재 명령 상태 $(q_c,\dot q_c)$** 에서 재고 (L3 §4.3) 매 수신마다 더 나은 후보로 교체한다 (§4.7) — 팔이 이미 목표 근처로 가 있는 상태에서 다시 재는 것이므로 실제로 열리는 투척은 이보다 많다. 그 효과의 측정은 S8 의 "연속 재계획 측정" 항목이고, 교체는 점수 문턱·오차 점프 한계·`COMMITTED` 동결로 제한된다.
- ⚠️ **꼬리는 그 예외다 — 거기서는 하한이 곧 실제다.** 측정된 24 비행의 첫 계획 시각은 0.164–0.830 s 인데, **0.30 s 를 넘는 비행이 33 % · 0.45 s (지도가 비는 값) 를 넘는 비행이 8 %** 다. 늦은 검출 시행은 **첫 예측이 그때 처음 도착**하는 경우라 그 전에 움직일 수단 자체가 없고, 따라서 정지 출발 가정이 실제와 일치한다 — 그 시행에는 계획 가능한 후보가 **아예 없다**. D-27 (reachable 하면 도전) 도 후보가 계획기에 도달해야 성립하므로, S7–S8 착수 전에 대응이 필요하다.
- **G8-D2 판정 (`iiwa7_leap`) — 지도가 비어 있지 않다.** §1a 의 조건부 규칙대로 실측 선행시간에서 다시 돌렸다 (첫 계획 0.215 s): `torque` **11 / 2835**, `box` 0, 열린 후보의 t_c 0.444–0.468 s. **따라서 G8-D2 는 `NOT_EVALUATED(선행시간)` 가 아니라 평가 대상이다** (판정 자체는 S8 의 몫). 단 창이 좁다 — 같은 지도에서 0.14 s 면 65, 0.19 s 면 27, **0.24 s 면 0** 이고 측정된 24 비행 중 lead ≤ 0.24 s 인 것은 54 % 다. L 0.14 s 가 아직 가정값이라 provisional 이다.
- ⚠️ **그 재실행 전에 설정 회귀를 고쳤다.** `iiwa7_leap` 의 `demo_catching_controller.yaml` 에서 `planner.ik` (`k_manip` 0.5 · `max_iter` 40) 과 `planner.catchability` (`arm_5row` **0.174**) 이 PR [#559](https://github.com/hyujun/rtc-framework/pull/559) 에서 **통째로 빠져** 있었다 (`ur5e_p1b` 는 정상이고, 같은 파일의 주석은 그 키들이 기록돼 있다고 말한다). 키가 없으면 `ParseCatchingParams` 가 in-code 기본값 (`k_manip` **0**, 임계 **0.1**) 으로 조용히 해석하고 경고도 내지 않으므로 — `CheckActiveTbd` 는 TBD 만 잡지 부재는 못 잡는다 — 오프라인 지도가 로봇별 임계 0.174 대신 0.1 로 돌아 kinematic 수용이 2178 → 2802 투척으로 느슨해졌다. 복원 후 재실행이 S3.5b 의 **2178 투척 / 15313 후보 / `v_max` 2.0198** 을 그대로 재현하는 것이 복원 검증이고, 위 민감도의 0.14 → 65 · 0.24 → 0 도 S3.5b 실행과 일치한다. 런타임은 아직 이 키들을 소비하지 않으므로 (S6.2) 영향 범위는 지도뿐이다.

**T_det 재실측 (2026-09-22, 공 lane stamp 수정 후) `[sim 실측 — L 은 여전히 가정값]`.** 같은 rig·같은 투척 24 발, profile **1.0 s / 0.05 s / 20 점** (설정된 파일). 공 lane stamp 간격: **p05 10.00 / p50 10.00 / p95 10.00 / max 40 ms, 25 ms 초과 2 / 9164 (0.02 %)** — 수정 전 9.2 % 였다. 수신 간격은 그대로 p05 2.9 / p95 39 ms 로 버스트가 남아 있다 — 지터가 stamp 에서 빠졌을 뿐 stepper 는 그대로다. 초과 2 건은 주기의 정수배 (40 ms) 로, 버스트 안에서 프로브의 `KEEP_LAST(1)` 이 놓친 샘플이다.

| 축 | min | p05 | p50 | p95 | max |
|---|---|---|---|---|---|
| `recv` (소비자 체감) | 0.032 | 0.033 | 0.058 | 0.110 | 0.115 |
| `stamp` (발행자, H_req 입력) | **0.040** | 0.040 | 0.050 | 0.090 | 0.100 |

- **24 투척 전부 검출.** 분석기가 26 비행 중 둘을 미검출로 보고하는데, 그것은 던진 공이 아니라 착지 후 바닥에 멈춘 공의 짧은 조각 (z 0.033 m, 69·31 샘플) 이 truth lane 의 0.3 s 이상 정지 (`--flight-gap-s`) 로 잘려 나온 것이다 — 멈춘 공에 VALID 예측이 없는 것이 맞다.
- **꼬리가 사라졌다**: `stamp` max 0.690 → **0.100 s**, `recv` max 0.729 → 0.115 s. 첫 VALID 앞의 예측 수는 0–2 (수정 전 최대 16). 중앙값 0.050 s 는 estimator 초기화 창 (5 점 × 10 ms) 그대로다.
- **첫 계획 시각 T_det + L 0.14 는 min 0.180 · p50 0.190 · p95 0.230 · max 0.240 s** — 24 비행 전부가 지도의 가정 0.24 s 안이다 (수정 전에는 54 %). 위 민감도 표로 읽으면 `ur5e_p1b` 는 모든 비행에서 열린 투척 ≥ 652 (0.215 s 기준), `iiwa7_leap` 은 0.19 s 에서 27 · 0.215 s 에서 11 로 **G8-D2 평가 대상이 모든 비행에서 유지**된다.
- **H_req 가 한 점 내려왔다**: 가장 이른 검출이 0.024 → 0.040 s 라 0.988 s · n 20 (위 "산식과 값"). profile 1.0 s / 20 점은 이 run 이 그대로 썼다 — 20 점 · 0.05…1.0 s · VALID 2123 · 부분 무효 0.
- ⚠️ **stamp 축과 wall 의 차이는 비행별로 재야 한다 (D-3).** 발사 순간의 stamp−wall 오프셋은 24 비행에서 10 ms 폭 안에 있었고 (발사 anchor 가 동작한다는 확인), 비행 안 위상 오차는 대부분 ±50 ms 였다. 그런데 **두 비행에서 sim 이 정지했다 뒤따라잡았다** (`/sim/status` RTF 0.3–0.4x → 1.8x): 그 비행의 stamp 는 wall 을 최대 1.9 s · 0.36 s 앞섰고, ball_perception 은 `max_future_skew_s` 0.1 을 넘는 샘플을 버렸다. 종전 wall stamp 였다면 같은 비행이 1.8 배 빠른 공으로 보였을 것이다 — 어느 쪽이든 그 시행은 D-3 게이트가 무효로 걸러야 하는 시행이고, 새 축에서는 "데이터 없음" 으로, 옛 축에서는 "틀린 데이터" 로 나타난다는 차이가 있다. throttle 이 정지 뒤 RTF > 1 로 따라잡는 것은 기존 동작이다 (§5).
- 재현: 이전 run 과 같은 러너 (`vision_lane_probe` + `analyze_vision_lane`, 24 발 `/sim/launch_ball_at`). 원자료는 repo 에 두지 않는다.
- 원자료 (CSV 4종·지도) 는 repo 에 두지 않는다; 방법은 위 설정과 `analyze_vision_lane`·`catch_gate_map` 의 인자가 SSoT 다.

#### S5 포구 컨트롤러 골격·입력·추종 (Adding a New Controller)

착수 조건: S0.6·S0.8 승인, `[CONCERN] E-8` 승인, D-24 결정 — **모두 충족 (2026-09-22 사용자: E-8 승인 · D-24 (a))**. 착수 시 결정 둘 (G5-E substrate · QP solve time 예산) 도 같은 날 확정됐다 (§7.3).

- S5.1 컨트롤러 완성 (S4.0 확장): YAML, lifecycle, 재무장. **E-STOP·fault 최소 계약 (P-1, E-8 — 승인 2026-09-22 사용자, 문구 그대로)**: (a) `TriggerEstop`·`ClearEstop`·`ResetFault`·`ResetTargetInitialization` 훅은 atomic 요청·epoch 만 갱신하고 reset 의 유일 writer 는 RT tick, (b) 해제 후 자동 재개 금지 — q_c·CLIK 앵커를 q_meas 로 reseed, (c) plan·궤적·공분산·손·FSM·타이머 무효화는 D-23 순서로, (d) `ClearEstop` 은 컨트롤러 fault 를 풀지 않는다 (base 계약: 두 경로는 별개). 전체 물리 정책은 S9
- S5.2 PointCloud2 구독 (nrt, `KEEP_LAST(1)`, reliability 는 S3.4 결과) → 필드 이름 파서 → SeqLock 스냅샷 (공분산 제외, A-3). D-2 변환, `generation`/`validity`/`snapshot_sequence` 처리 — C-1 정책(한 점이라도 무효면 거부)을 구현하고, S3.4 측정에서 부분 무효·재시작 되감김이 나왔으면 착수 전에 C-1 재검토와 되감김 정책을 정한다. 스냅샷에 `ActivationGeneration()`·token (D-22, D-23). 지평이 요구(D-15)보다 짧으면 진단. 공분산은 같은 token 을 가진 계획기 쪽 버퍼에만. S5.2e RT 상태 POD 에 D-24 의 센서 freshness
- S5.3 스트리밍 기준 → 확장 CLIK → 팔 명령. QP 비의존 관절공간 abort 경로. 예측 선행 보상 (L5.8), backend 왕복 대조 (L5.10)
- S5.4 CSV 로그, 상태 publisher (`PublishRole` 을 늘리지 않음, E-11) — 포구 상태 메시지 신설 (D-20, `rtc_msgs`, S0.8 E-3 승인 후, PROC-3). **S5~S9 필드 superset 동결**, `Compute()` 의 모든 early-return 분기(E-STOP·stale·generation 불일치·지평 부족·plan 없음·abort)에서 Store 하고 그 tick 에 계산하지 않은 필드는 무효화 (PROC-7). DemoWbc 의 `!target_initialized_` early-return 은 Store 를 빠뜨리므로 선례로 쓰지 않는다
- S5.5 ground truth 기반 고정 포구점(oracle plan)으로 추종 검증

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 추종 | G4-H, G5-A~B2, G5-C3, G5-C4 | — |
| RT | 할당 0 (G1-D, G5-C 의 할당 부분), QP solve time | solve time 예산 **확정 (2026-09-22 사용자, provisional)**: `control_rate` 500 Hz 의 tick 2000 µs 기준 **p99 ≤ 400 µs (20 %) · 최대 ≤ 1500 µs (75 %)** — 평균이 아니라 분위수로 건다 (L8: 총량이 아니라 분산의 꼬리가 위험). S1.9 표의 1777 µs 는 오프라인 포구 자세 IK 한 번이지 tick 당 CLIK QP 가 아니다. S5 실측이 훨씬 작으면 그때 조인다 (L5 §9 G5-C) |
| 입력 | G1-A~C, G1-E, G1-F, G1-I (nrt 콜백을 일부러 막아 backlog 를 만든 뒤 최신 `snapshot_sequence` 만 수락, ARCH-6) | — |
| SeqLock | writer 부하에서 찢어진 스냅샷 0·최악 재시도 시간 기록 (G1-C). writer 끼어들기를 재현하는 결정적 테스트에서 최신 스냅샷 누락 0 (G1-H, D-21) | — |
| activation | 비활성 중 받은 궤적이 재활성 첫 tick 에 소비되지 않음 (G1-J, D-23) | — |
| E-8 | G7-H: (a) deactivate → 다른 컨트롤러가 팔 이동 → 재activate 시 첫 tick 이 옛 자세를 명령하지 않음, (b) trigger·clear·deactivate race 에서 reset writer 가 RT tick 하나, (c) 자동 재개 0, (d) `ClearEstop` 후에도 latched fault 유지. `/security-review` (최소 정책) | — |
| PROC-7 | G8-H: early-return 분기마다 그 tick 의 body 가 실렸는지 보는 실패 경로 테스트 (`EstopTickPublishesThisTicksBody*` 선례) | **PASS** (2026-09-22). `Compute()` 를 단일 exit 으로 두고 레코드를 tick 머리에서 기본 생성하므로 "모든 분기에서 Store" 와 "계산 안 한 필드 무효화" 가 둘 다 **구조적**이다. 테스트: E-STOP·stale·plan 없음·법칙 미배선 tick 이 각각 자기 tick 의 body 를 싣는지 (`DemoCatchingRecord.*`), 그리고 실모델에서 **추종 tick 이 채운 기준·solve 블록이 다음 비추종 tick 에 지워지는지** (`ATickThatDoesNotRunTheLawClearsTheBlocksItDidNotCompute`). positive control: tick 머리의 기본 생성 1줄을 지우면 그 테스트가 red |
| 선행 보상 | G5-C2 (backend 왕복), G5-E (지연 에뮬레이션에서 선행 보상 전후 기록) | **확정 (2026-09-22 사용자): (ㄱ) fixture 전용 지연 주입** — 테스트에서만 쓰는 지연 큐로 `prediction.lead` = $T_{arm}$ 의 보상을 S5 에서 관측한다, 런타임 경로 불변 (`rtc_controllers` 의 `sim.ball.*` 레인 선례, §4.4 S3a 각주). S3.7 이 2026-09-20 결정으로 빠져 substrate 가 없던 것을 이렇게 닫는다 |
| PROC-3 | S5.4 및 S5.2e (D-24 (a) 확정) 의 `rtc_msgs`·`rtc_base` 변경 후 전체 빌드·테스트 | **PASS** — `./build.sh full` + 전체 `colcon test` |
| GUI·plot | §13 S5 행 | **PASS** — §13 S5 행에 기록 |

#### S6 계획기 스레드 (D-7)

착수 조건: S5, S3.6, `[CONCERN] E-7` 승인 (§6).

- **D-27 의 런타임 설계 (2026-09-22 추가).** 실행은 기구학적으로 reachable 하면 도전한다 — 성능 게이트 (γ 창·도달시간·commit 선행) 탈락을 후보 제거로 쓰지 않고 **순위·진단**으로 쓴다. 정해야 할 것: ① 어느 게이트가 판정으로 남는가 (정지점·작업공간은 팔이 어디에 멈추는지의 문제라 E-8 접점) · ② 통과 후보가 없을 때의 순위 함수와 `[CONCERN]` 보고 · ③ 시도했으나 놓친 시행의 진단 코드 (S8 성공률은 **시도/성공을 분리 보고**한다). 지도 쪽 기준은 바꾸지 않는다 (D-27: 분석은 정교하게).

- S6.1 스레드 골격: MPC 스레드 생성 방식 그대로 (§6). **새 layout role 을 만들지 않고 `mpc` role 을 재사용한다** (E-7 결정 J, 2026-09-23) — `SelectThreadConfigs().mpc.main` 을 받아 스레드 이름도 `mpc_main` 이다. CM 이 active 컨트롤러를 하나만 두고 전환 시 이전 것을 deactivate (`Pause`) 하므로 같은 slot 에서 동시에 도는 FIFO 는 하나다. RT-1~10 준수 코드, activate 게이트 (`planner.enabled` && profile `mpc_off` 면 첫 문장 FAILURE — DemoWbc 와 같은 형태), RT → 계획기 `PlannerRtState` SeqLock, 계획기 → RT `plan_box_` SeqLock (oracle 은 같은 box 의 대체 writer 로 이전, 두 writer 가 동시에 켜지는 설정은 park), A-S5-12 (sim park)
- S6.2 포구 자세 IK·catchability: S1.9 함수를 스레드 전용 모델 handle 로 배선. 후보가 모두 탈락하면 plan 없음(포기)을 사유 코드와 함께 기록
- S6.3 γ 창·rollout (S1 코드 호출), 예산 초과 시 coarse-to-fine
- S6.4 후보 선택·hysteresis·commit/freeze, `PlanSnapshot` SeqLock (token·`publish_ns`, D-22), 계산 시작·게시 직전 token 재검사
- S6.5 D-7a 측정 (§7.2) — **생략 (사용자, 2026-09-23)**: 초기값 FIFO 유지
- S6.6 NLP 전환 대비 (A-4, §8): 탐색 전략을 계획기 코어의 단일 진입 함수 뒤에 두어, 1차원 탐색 + IK 를 NLP 로 바꿔도 스레드·입출력 스냅샷·RT 쪽 소비 코드는 그대로 두는 경계를 유지. 전환 판단 신호(IK 수렴률 G3-G, 계획 성공률, 예산 초과율)를 S6·S8 에서 기록
- ~~S3.1b D-3 부하 재검증: 포구 컨트롤러 + 계획기 + `sim_estimator_node` 동시 구동 (§5)~~ — **S6 에서 수행되지 않았다** (2026-09-24 확인). S8 시행에서 clock lane 을 켜 누적 ≥ 200 으로 수행한다 (§4.4 S8, §5)

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 계획 | G3-A~E, G3-C 예산 준수, **R-2 계획기 연산시간 실측·보고** (후보당 IK · 사전 필터 후 후보 수 · `PlanOnce` 한 사이클 p50/p99/max · 예산 초과율) | `planner.budget_s` 0.020 (R-2, L3 §6) |
| IK·catchability | G3-G (수렴률 기록), G3-I (런타임과 지도 동치) | — |
| RT | G3-K (한 사이클 할당 0·noexcept·로깅 없음) | — |
| token·race | G3-L: eventfd coalescing, 계산 중 새 스냅샷 도착, 같은 generation 의 옛 plan 게시, deactivate·Pause race 에서 대체된 plan 소비 0 (D-22, D-23) | — |
| 스레드 | **결정 J 로 layout 변경 0** (manifest·generator·3 oracle·`all_configs`·검증기 표 불변). 대신 **R-1**: WBC ↔ 포구 컨트롤러 `SwitchController` 로 `mpc_main` 두 TID 의 affinity·Paused/Running 을 직접 검사 (단위 + launch_testing). 검증기는 같은 이름의 스레드 중 하나만 검사하므로 그것에 기대지 않는다 | — |
| D-7a | G3-J — 제어 PC 에서만 판정. 개발 PC 결과는 NOT_EVALUATED(제어 PC). **생략 (사용자, 2026-09-23)** — 초기값 FIFO 유지 | 제어 PC |
| D-3 부하 | §5 판정 (부하 구성) | ε_clk 할당 |
| GUI·plot | §13 S6 행 | — |

#### S7 손 시퀀서·슈퍼바이저

착수 조건: S6 완료(충족). 커밋 1 은 이 docs 정정이며, 코드는 그 뒤 사용자 지시로 착수한다 (`[SPRINT] S7`, `[CONCERN] E-8` 승인 완료 — §7.3).

- S7.1 손 시퀀서 → 손 device slot. **손 상태 규칙 변경 (#537 S7 결정 2026-09-23)**: `q_open` 은 IDLE homing 중에만 쓰고, 팔이 `wait_pose` 에 도착하면 손은 ARMED~COMMITTED 내내 항상 `q_pre` 다 — 시각 기반 preshape(`t_c − T_pre`, `PreshapeDue`)와 `T_pre` 키는 폐기한다 (L7 §4.5 조건 6 은 "손 `q_pre` 도달" 로 정정). 손 위상 순환: Open(homing) → Preshape(wait_pose 도착~COMMITTED) → Close(t_cmd) → Hold → Release(목표 `q_pre`) → Preshape. 손 모드별 정책: 비무장 IDLE = latch(정지), 무장 IDLE homing = q_open(팔이 이미 `pose_tol` 안이면 q_pre 로 바로 — homing 생략), ARMED~COMMITTED = q_pre, CLOSING/DECEL/HOLD = Close→Hold, ABORT_SAFE = 진행 중이던 phase 계속, FAULT = 마지막 출력 유지, 시행 후 disarm 은 마지막 시퀀서 출력을 latch 로 복사, E-STOP·activation 리셋은 시퀀서 inactive + 측정 자세 latch(현행 재latch 정책 유지, S9 이월).
- S7.2 FSM driver 완성 (전이표는 그대로, L7 §4.1). homing·retreat 는 **관절공간** (per-joint 사다리꼴, QP/CLIK 비의존) — `retreat_reference.hpp` 는 코드에 없으므로 L4 §5.3·L7 §4.1 을 이 방식으로 정정한다. homing 은 **무장 latch 를 요구한다**(활성화는 무장이 아니다, P-1 (e)) — 비무장 IDLE 은 활성화 자세를 유지한다. 팔이 이미 `pose_tol` 안이면 homing 을 생략한다. **driver 규칙 (#537 S7 결정 2026-09-23)**: R-PREC(사유 우선순위: ESTOP > fault reset/escalation > 준비 상실 `kParamsTbd` > 법칙 실패·치명 > 시간 전진 `kNone` > 기록 전용, L7 §4.2), R-ORDER(IDLE(homing)·DECEL·HOLD·RETREAT 는 vision lane 판정 앞, COMMITTED/CLOSING 은 항상 법칙을 돌리고 stale·지평 밖은 외삽 샘플로 계속 + 기록만), R-IDLE(IDLE tick 에서 carried 관절 속도가 있으면 `JointSpaceDecelStep` 으로 정지까지 램프), R-WATCHDOG(homing·retreat 법칙도 `track_err_abort` 를 본다), R-DECEL-ENTRY(tick 의 now 를 머리에서 한 번 읽어 법칙·시퀀서·판정에 같은 값을 넘긴다), R-ADMIT(`JudgePlan` 에 (g) `t_c − now ≤ T_freeze` 거부 `kTooLate` 추가, 검증기 `T_freeze ≥ T_close_e2e + T_arm + margin` — L3 §4.11), R-CLOSE(COMMITTED→CLOSING 은 시퀀서 `close_issued`, 규칙 `now ≥ t_cmd − h/2` — `HandCommandDueRounded` 신설), R-TRACK(동결 후 샘플은 commit 시점 `generation` 에 고정, 다른 generation 스냅샷은 stale 로 기록만 계속; 재무장 뒤 직전 시행 generation 은 새 번호가 올 때까지 usable 로 보지 않는다). COMMITTED 이후 stale 은 동결 plan 으로 계속, `supervisor.stale_committed_max_s` 초과 시 ABORT_SAFE (A-6, L7 §4.2).
- S7.3 접촉 판정 (sim·실기 모두 finger-on-object — 착수 시 재확인), `TIP_STALE` (D-24 freshness), 감속, 충격량 예산. 바이어스·잡음 창은 원형 버퍼가 아니라 **EMA** (`supervisor.contact.baseline_alpha`, L7 §4.4/§4.8 정정). 디바운서는 tick 이 아니라 지문 **샘플**(`inference_sequence` 변화) 단위로 먹인다. 바이어스 표본 수가 `contact.n_baseline_min` 미만이면 결과는 `Undetermined`(Missed 오판 금지). `REF_SATURATED` 는 연속 `supervisor.sat_ticks` tick 판정으로 승격한다.
- S7.4 abort·retreat·재무장 리셋, 연속 투척. 현재 리셋 함수는 `ResetTrialState` 하나뿐이다 — S7.4 는 `ResetForRearm()`(RETREAT→ARMED)과 `ResetTrialState()`(= 그것 + activation/E-STOP 몫)으로 분리한다 (레이어별 소유자가 등록하는 단일 표, L7 §4.8). `qp_fail_streak_` 는 `ResetForRearm` 에서 **면제**(지우면 FAULT 에스컬레이션이 영영 도달 불가), 재무장은 시퀀서를 Ready(q_pre) 로 둔다(Open 아님), `reset_floor_ns_` 는 재무장에서도 `planner_reset_epoch_` bump 와 같은 자리에서 갱신, eventfd 는 어떤 리셋에서도 drain 하지 않는다(의도적 면제), carried 팔 명령은 재무장에서 유지한다(qd 만 0). **RETREAT 순서 (#537 S7 결정 2026-09-23, release 규칙은 2026-09-24 교체 — 판정과 무관)**: 진입 → 정지 램프 → 관절공간 복귀(이미 `pose_tol` 안이면 생략) → 대기 자세 도착에서 손 Release(q_pre) → 손 `q_tol` 도달 → `ResetForRearm` → ARMED. 복귀 중에는 판정이 무엇이든 손을 열지 않는다 (지문 판정의 Missed 는 "공 없음" 이 아니다 — sim `260923_2336` 에서 손바닥에 얹힌 공이 Missed 로 판정돼 진입 즉시 열린 손에서 떨어졌다). 닫힘 명령 전의 commit 만 진입 시 취소한다. **IDLE 순서**: 검증 통과 + 무장 → 팔이 `pose_tol` 밖이면 손 q_open → homing → 도착 → 손 q_pre 지시, 안이면 손만 q_pre → 손 도달 + §4.5 → ARMED.
- **PROC-6/E-6 고지 목록 (착수 커밋 1 에서 확정)**: fixture 프로파일의 `planner.wait_pose` = fixture home 으로 대부분 회피하나, (i) `test_catching_tracking.cpp` 의 700-tick APPROACH 단언, (iii) `CatchingTorqueTest` 의 포화 유도 중 매 tick APPROACH 단언, (iv) fault 사이클 tick 수 단언 — 이 셋은 회피가 안 되어 별도 spec 변경 커밋 + 근거가 필요하다 (착수 순서 §5 커밋 6 앞).

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| FSM | G7-A, G7-B, G7-D | — |
| 센서 stale | G7-G: 관절 fresh + 지문 센서 dropout negative control 에서 `TIP_STALE` 또는 결과 `Undetermined` 발화, 옛 힘을 새 접촉으로 판정 0 | D-24 |
| 접촉 | G7-C (오경보율 기록) | 임계 `supervisor.contact.f_min` **0.2 N** (사용자 값) — sim fingertip lane 은 잡음 0 이라(§4.4 S3.5b·L7 §4.4) `k_sigma` 는 실기 전용, 오경보율은 합성 가우시안으로 기록 |
| 충격·토크 | G7-B3 | 손 토크 권위 한계 (D-12): sim **3.0 N·m**, 실기 **1.5 N·m** provisional 병기 (§7.3) — 확정은 S10, 그 전까지 토크 비교는 `NOT_EVALUATED` |
| 재무장 | G8-A2, 리셋 표 완전성(모든 stateful 멤버가 표에 있거나 명시적 면제) + 런타임 poison 테스트 | — |
| 결과 판정 | G7-E | — |
| PROC-7 | G8-H 를 S7 에서 늘어난 분기(COMMITTED stale·DECEL/HOLD·homing 중 disarm·RETREAT 중 E-STOP)까지 확장 | — |
| GUI·plot | §13 S7 행 | — |

#### S8 sim 통합 평가

- **ur5e_p1b → iiwa7_leap** (2026-09-22 결정으로 `ur5e_p1b` 가 1차, G8-D2 는 조건부 — §1a·§7.3; 종전 "iiwa7_leap → ur5e_p1b" 는 S3.5b 이전 서술). Wilson CI, NEES, 소거실험(γ, lead). **γ 포화 빈도 측정 → D-8 재검토 입력**
- **S8 준비 대조 (2026-09-24)**: ① sim 팔은 지연 0 이 아니다 — 두 로봇의 팔 관절 MJCF `<general>` PD 가 모두 kd/kp = 0.2 s 라 관성과 무관하게 **시정수 ≈ 200 ms 의 1차 지연**이고 (L5 §4.6), S6 의 서보 성분 ~125 mm 와 포획 0/25 의 원인이다 (보상 방식은 사용자 결정 D-S8-1). ② 시행 러너 `catching_sim_trials` 는 `sim.throw_region` 을 읽지 않는다 — 그 키는 어디에도 없고, 고정 기준 투척 + 섭동만 쓴다 (동결 분포는 D-S8-2). ③ S3.1b (D-3 부하) 는 수행된 적이 없어 S8 시행에서 clock lane 을 켜 함께 잰다. ④ Wilson·소거실험·간극/충격량 짝짓기 도구는 없다 (예측 lane 기록은 첫 점 + `cov_nan` 뿐) — S8-A 가 만든다. 단 NEES 는 새로 만들지 않는다: ball_perception_sim 의 `sim_evaluator_node`/`sim_capture_evaluate` 가 지평별 NEES·부호 있는 항력 편향·coverage 를 이미 낸다 (P5). ⑤ `T_arm` 을 0.2 로 올리면 `CheckFreezeCoversClose` (T_freeze ≥ T_close_e2e + T_arm + h = 0.4825 s) 가 출하 `T_freeze` 0.36 을 거부해 활성이 조용히 실패하고, `io.horizon_min` 0.51 (T_arm 0.05 도출값) 도 0.66 (`n_min` 15) 로 같이 올려야 configure 가 통과한다 (C-25). T_freeze 0.52 는 계획 후보 창 하한을 올려 S3.5b 90 % 상자 (t_c 0.61–0.73 s) 의 중앙값 투척이 열린 후보를 잃을 수 있다 → S8-B 첫 단계로 지도를 재실행한다. **결과 (2026-09-24)**: 0/180 으로 닫혔고, D-S8-13 이 sim 플랜트를 τ 0.05 로 바꿔 ⑤ 의 0.2 파급 전체를 은퇴시켰다

**S8 확정 결정 (2026-09-24 사용자, #537 코멘트 5804952754 → 5807767896).**

| ID | 확정 |
|---|---|
| D-S8-1 (a) + C-25 (a) | sim 서보 지연은 **L5 §4.5 선행 보상을 sim overlay 로 켜서** 다룬다 — overlay 집합 `joint_cmd.lag.{T_arm: τ̂, lead_enable: true}` + `planner.freeze.T_freeze: 0.52` + `io.{horizon_min: 0.66, n_min: 15}` (출하 YAML 불변). lead-off 비교 arm 도 **같은 T_freeze 0.52** 로 (검증기는 T_arm 을 보고, 선행이 꺼지면 RT·계획기는 0 을 쓴다 — lead 만 다르다). S8-B 첫 단계 = `catch_gate_map --arm-delay-s τ̂` + T_freeze 0.52 로 S3.5b 지도 재실행, 상자가 비면 D-S8-2 재결정. 1차 지연 정확 역보상 $u=q_d+\tau\dot q_d$ (c) 는 기각 — L5 순수지연 선행 기구 (S10 이 쓸 것) 를 검증하지 않는다. **→ D-S8-13 으로 개정 (τ 0.2 overlay 집합은 은퇴)** |
| D-S8-2 (a) | 동결 분포 = 로봇별 gate 지도 상자 — `ur5e_p1b` 는 S3.5b 90 % 상자, `iiwa7_leap` 은 S8-D 재실행 상자 (**임계는 로봇별**, D-S8-15: leap 은 열림 비율 최대·하한 50 %, 2026-09-26 개정 — 종전 "≥ 90 %" 는 p1b 의 값). 표본은 `catchability_map.Throw`/`generate_throw_grid`/`throw_to_launch_request` 재사용 (P5), seed 로 재현. 기준 투척 반복은 iid 가 아니므로 G8-D 에서 빼고 회귀 세트로만 쓴다 |
| D-S8-3 | floor 는 **사용 목적에서 사용자가 사전 고정** (**0.35**, 2026-09-24 — 0.5 provisional (#537 5808613906) 을 S8-B 튜닝 p̂ 0.47 을 보고 하향; S8-C 데이터로 조정 가능, S8-E 착수 시 동결 → **2026-09-26 동결 완료 (D-S8-16)** — §1a 기준 1), **n_valid 200** (벽시계 ≈ 17 min/로봇/공). "floor = p̂ − 0.2" 는 순환이라 기각 (Monte Carlo 로 참 p 0.3/0.5/0.7/0.9 에서 통과율 0.72/0.71/0.70/0.80). 파일럿·튜닝 데이터는 본 평가에 합치지 않고, 중간 판정·조기 종료 없음 |
| D-S8-4 (c) | D-3 δ 는 **판정이 아니라 공변량** — δ(t_commit)·δ(t_c)·시행별 tick overrun 수·최대 tick 간격. δ–결과 상관은 n 25–50 에서 검정력이 없어 (r 0.3 에 n 85) **같은 seed 투척의 부하 A/B** 로 D-3 효과를 상계한다. 무효 = rig 실패만 (srv 거부·lane drop·sim stall·미발사), "plan 없음·abort" 는 실패. ITT 하한 (무효 = 실패) 병기 (§5 · 아래 게이트 표) |
| D-S8-5 (a) | ⑦ 램프 연속은 조건부 — **트리거 = S8-B (lead on) 에서 `held_jump` 로 막힌 갱신의 ‖Δp_c‖ p95 > r_cap/n_σ (12 mm)** 또는 APPROACH 중 교체 요구가 시행의 ≥ 20 % |
| D-S8-6 (a) | RETREAT 의 손 $q_{pre}$ 대기에 timeout: `{kRetreat, kHandTimeout, kIdle}` + 같은 tick disarm, 키 `robot.hand.T_release_timeout` (S8-C). IDLE 은 hang 이 없어 제외. E-STOP 경로가 아니다 (E-8 비해당, `/security-review` 는 수행). **구현 (S8-C, #537 5823291259)**: 시계는 복귀 도착 tick (`kReturn → kRelease`) 에 시작, 같은 tick 정착이면 재무장 우선. 이 행을 넣으면서 **RETREAT → IDLE 이 trial 상태를 리셋하지 않던 기존 결함**을 함께 닫았다 (`ResetTrialScope` — L7 §4.8) |
| D-S8-7 (a) | ν̄ (`PRED_INCONSISTENT`)·`io.pred.nu_reg` 은퇴 — 생산자를 만들지 않는다. 예측 일관성은 추정기 `/ball_perception/debug/{innovation,nis}` 를 probe 가 기록해 오프라인으로 본다. σ_ℓ 도 `planner_events.csv` 기록만 (소비자 없음) |
| D-S8-8 (b) | false-Missed (링크·손바닥 위 공) 는 **측정 후 결정** — 손 관절 q·토크는 device lane `p1b_state.csv` 에 이미 있다 (~~diag 열 추가 불필요~~ — S8-C 에서 판정 tick 의 증거와 판정 출처를 diag 에 싣기로 정정: `hand_stalled_n`·`hand_effort_frac`·`hand_blocked_s` (끊김 없는 막힘의 지속 시간 [s])·`outcome_source`, 같은 투척의 "지문만 판정" 재구성과 짝 비교용). **측정 (2026-09-25, S8-B 산출물, #537 5822932236)**: 판정 × sim truth 에서 false-Captured 0, **false-Missed 가 truth 성공의 43–57 %** (2×2 lead on·계획 γ 27/47, 튜닝 40/94, 재측정 52/93). 성공률 (truth) 과 release (대기 자세, 판정 무관) 에는 영향 없음. **결정 (b) 판정 근거 추가 (2026-09-25 사용자)**: 지문 합의에 손 관절 q·토크 증거를 더한다 — 런타임은 컨트롤러가 이미 받는 손 device 상태를 쓰고 (CSV 는 오프라인 확인용), 판정식·임계는 S8-C SPRINT 에서 정한다 (robot-agnostic, ARCH-1). 바꾸면 G7-E·G7-C 재기록. **판정식 (S8-C, #537 5823291259 — 독립 검토 반영)**: caging 관절별로 $\rho_{\min}\le\rho_i\le\rho_{\max}$ ∧ $|\dot q_i|\le\dot q_{tol}$ ∧ $s_i\tau_i/\tau_{\max,i}\ge\kappa$ 인 관절이 `min_joints` 이상 (Hold, lane readable) 이고 `t_persist` 동안 끊기지 않으면 포획 (지문 조건과 OR, 미확정 우선) — L7 §4.4. 기각: min-ρ + 전 관절 토크 median (공에 안 닿는 관절이 좌우), `contact.t_confirm` 재사용 (의미가 다름). 시행 전체 손 토크 최대는 S8-B 500/500 발 3.06 N·m 로 분리력 0 이라 판정 tick 창에서 본다. G7-C 는 truth 비교가 아니라 합성 잡음 오경보율이라 순수 함수 잡음 테스트로 따로 기록 (빈 손 q_close·q_pre 0/20000) |
| D-S8-9 (a) | `ur5e_p1b` 먼저, leap (S8-D) 은 S8-B 뒤 (지도 재실행이 lead 값을 쓴다). G8-C 는 p1b 에서 γ = 0 arm — **정정 (2026-09-24, #537 5809415678)**: `planner.gamma.grid: [0.0]` 만으로는 γ = 0 이 아니다. γ 창 하한 γ_min = 1 − d_eff/(v_ball·T_close,tot) 가 공 속력 > 1.0 m/s 에서 0 보다 크고, grid 가 창 밖이면 rollout 이 γ_min 을 시도한다 (smoke γ_f 0.40–0.46). 그래서 γ0 arm = grid `[0.0]` + **`planner.hand.d_eff: 10.0`** 이다 (γ_min 0). d_eff 는 MaxCatchableSpeed 도 올리지만 그 효과는 γ 창 순위 벌점 (+10, 후보 제거 아님 — D-27) 이 빠지는 것뿐이고 §4.6 예산에는 들어가지 않는다. **S8-B 2×2 확인 (2026-09-24)**: lead on 쌍 100 에서 두 γ arm 모두 100/100 commit (γ0 만 시도 0), commit 시각 차 중앙값 −0.1 ms, 계획 arm 후보가 γ 창을 통과한 76 쌍에서 성공 40 대 0 — G8-C 는 이 교란으로 설명되지 않는다 |
| D-S8-10 (a) | CLOSING 이 닫힘 명령보다 1 tick 늦게 기록되는 것은 문서 기록만 (시각 무영향) |
| D-S8-11 | 공 arm 은 `beanbag` preset 그대로 — preset 은 반발 외에 마찰 0.6→0.5·비틀림 0.05→0.3·구름 0.02→0.5 도 바꾸므로 (공 geom priority 100 이라 손에도 적용) **반발 + 마찰 복합 효과**로 보고한다. `tennis_soft` 는 만들지 않는다 |
| D-S8-12 | 손 근처 투척 = **S8-F** (탐색, 게이트 밖) — 아래 |
| D-S8-13 (2026-09-24 사용자, #537 5808985333 → 5809415678) | **sim 팔 플랜트 = τ 0.05 s**, 출하 `config/ur5e_p1b/mujoco_simulator.yaml` 의 서보 게인으로 (`use_yaml_servo_gains: true`, 팔 kp ×4 = 8000/2000, kd 400/100 그대로, 손은 MJCF 값 6000/250 을 그룹별로 옮김). PD 의 램프 추종 지연은 관성과 무관하게 kd/kp 라 τ̂ 50.2–50.9 ms (6 관절, R² ≥ 0.99), 팔 토크 최대 93–103 / 150 N·m · 포화 0. 근거: τ 0.2 (MJCF menagerie 게인) 는 T_freeze 0.52 를 요구해 S3.5b 상자를 0/180 으로 닫고, 격자를 넓힌 τ 0.2 지도의 ≥ 90 % 상자는 속력 4.95 · 앙각 66° 한 점이다; τ 0.05 는 설계가 유도된 T_arm 과 같아 commit 선행 0.37 에서 s35b 가 163/180 그대로 열린다 (D-S8-2 유지). S8-B overlay 집합 = `joint_cmd.lag.{T_arm: 0.05, lead_enable}` + `planner.freeze.T_freeze: 0.37` (L3 §4.11 하한 T_close,tot + T_arm + margin = 0.3615; 출하 0.36 은 T_arm 0 유도라 이 하한에 못 미친다 — 검증기 `CheckFreezeCoversClose` 는 margin 없이 0.3325 만 보므로 통과는 한다), `horizon_min`·`n_min` 은 출하값 (이미 T_arm 0.05 유도). **sim 플랜트는 선택이지 UR5e 의 값이 아니다** — G8-D p̂ 는 "실효 서보 지연 ≈ T_arm 인 팔" 조건부이고 S10 T_arm 식별이 닫는다. 검토: τ 0.025 (kp ×8, 또는 kp ×12·kd ×1.5) 도 τ̂ 25–26 ms · 포화 0 · s35b 163/180 (commit 0.36) 로 가능 — 채택 여부는 사용자 |
| D-S8-14 (2026-09-26 사용자, #537 5835585007 → 5835714797) | `iiwa7_leap` 묶음: sim 팔 kp ×4 (12000/3000, kd 유지, `use_yaml_servo_gains: true`, 손 2.5/0.125 그룹별) → τ 0.05 · 지도 라운드의 `proposed_wait_pose` 중 가장 많이 연 것을 출하 `planner.wait_pose` 로 · 상자 임계 완화 (D-S8-15) · S8-D 는 검증 100 발 p̂·Wilson 보고만 (판정·floor 동결은 S8-E) · profile p1b 동일 (1.0 s / 0.05 s / 20) · overlay `catch_lead_on` (T_arm 0.05 + lead, T_freeze 출하 0.19) + `catch_lead_on_unbounded` (슈퍼바이저 임계 끔, 교정용) · seed 스모크 505 · 교정 501·502 · 검증 503·504 · 지도 0 |
| D-S8-15 (2026-09-26 사용자, #537 5835714797 → 5835793705) | leap 상자: p1b s35b 와 같은 폭 (거리 0.1 m · 릴리스 0.1 m · 편차 ±6° · 속력 0.2 m/s · 앙각 2°, 격자 180 투척) 중 **열림 비율 최대**, 하한 50 % (미달이면 중단·재질문). **유효성 조건 추가 (2026-09-26 사용자 사후 승인, #537 5841231282)**: 상승 구간 (정점 + 20 ms 까지) 에 공 중심이 대기 자세 로봇과 2 cm 이상 떨어진 투척만 센다 (MuJoCo 정적 검사 — 지도는 비행 중 공–로봇 충돌을 보지 않는다). p̂ 는 상자 전체 (1차) + 지도-열림 부분집합 (공변량, `catching_trials --gate-map`) 병기. G3-D = 시행별 launch→commit 계획 주기의 `plan_valid` 비율 + APPROACH 교체 횟수 분포 |
| D-S8-16 (2026-09-26 사용자, #537 5841469779 → 5841651825) | **S8-E 본 평가 계획 확정** — ① 무효 (rig 실패) 는 시행별 `invalid_reason` 한 개로 기계 판정, 우선순위 `srv_refused` (launch srv `accepted == false`) > `not_launched` (accepted ∧ `n_truth_rows == 0`) > `controller_silent` (`wall_t_relative_offset` 없음 = 시행 창에 컨트롤러 diag 0 행) > `lane_drop` (clock lane 에 그 시행 `launch_seq` 세그먼트 없음 또는 `dropped_total` 증가) > `sim_stall` (시행 창 안 `sim_time_sec` 간격 > 5 × nominal step); 그 외 (plan 없음·abort·HAND_TIMEOUT·순환 미종결) 는 실패. 요약은 `n_total`·사유별 `n_invalid`·`n_valid`·Wilson(n_valid)·ITT Wilson(n_total). 기동 단위 실패 (추정기 activate 유실·오염) 는 같은 seed 로 unit 재실행, 원본 보존 · ①b 4 × 50 발 뒤 n_valid < 200 이면 **사전 선언 보충 seed** 로 부족분만 순서대로 · ② G8-B·G8-C2·G7-B3 의 **캡처 (`sim_estimator.launch.py record:=true` + `vision_lane_probe --dump`) 는 모든 unit 에서 필수**, 분석 코드는 스모크 데이터로 본 시행 전에 만들어 파이프라인을 검증하되 본 시행은 그 완성에 의존하지 않는다 · ②b G8-C2 의 p̂_live 는 commit tick 의 diag `input_snapshot_sequence` 를 probe dump `snapshot_sequence` 와 **정확 결합** (L8 §5.2), dump 에 없는 시행만 commit 직전 마지막 prediction 으로 근사하고 근사 수 보고 · ②c G8-B 판정식: 표본을 발사 창으로 시행에 묶은 시행별·지평별 평균 NEES 의 시행 부트스트랩 95 % CI 가 3 을 포함하면 PASS, coverage_95·부호 있는 편향은 보고, NaN > 10 % bin 은 `NOT_EVALUATED` · ③ G8-D2 는 **leap 도 새 seed 로 n_valid 200** (S8-D 검증 100 발은 판정에 쓰지 않는다 — Wilson 하한 반올림 전 0.3467, D-S8-3 의 데이터 분리); p̂ 는 상자 전체 1차·지도-열림 부분집합 공변량 (D-S8-15 미결 닫힘) · ④ beanbag arm 은 tennis 와 **같은 seed 200**, McNemar 쌍 비교, floor 0.35 · ⑤ D-3 S3.1b 는 S8-E 자체 (p1b 400 + leap 200) 로 ≥ 200 충족, S8-B 튜닝 200 은 요약 JSON 합산 병기 · ⑥ seed: p1b tennis·beanbag 601–604 (같은 투척), 보충 605→, 스모크 600; leap 701–704, 보충 705→, 스모크 700 · ⑦ §7.3 네 항목 (L3 가중치·NLP 신호·`reference.a_max`·`budget.sigma_trk`) 은 제안·기록만. **floor 0.35 는 이 시점에 동결** (D-S8-3). 커밋 분할 C1 rtc_tools 도구 → C2 beanbag overlay → 스모크 → C3 G8-B/C2 도구 → 본 시행 → C4 docs; 구현 착수는 별도 지시 |
| D-S8-17 (2026-09-26 사용자, #537 5842632283 → 5843117293) | **host 부하 unit 재실행 규칙 — S8-E 결과 게시 뒤, 재실행 전 선언** (5842632283 의 (b)). `rtf_trial_min` (clock lane 의 sim/steady 비, 시행 중 0.25 s 창 최소값) < 0.95 인 시행이 하나라도 있는 unit 은 **기동 단위 rig 실패 (host 부하)** — 같은 seed 로 unit 을 재실행하고 원본은 `<unit>.loaded` 로 보존한다 (D-S8-16 ①의 기동 단위 재실행과 같은 틀). 결과 (성공 여부) 는 보지 않고 세 arm 모두에 똑같이 적용하며, 재실행 unit 에도 같은 규칙 (최대 4 회, 넘으면 그 unit 은 `NOT_EVALUATED(host 부하)`). 대상: tennis 601 (6 발)·603 (2)·604 (20), beanbag 602 (13); tennis 602 (최소 0.95)·beanbag 601/603/604·leap 701–704 (최소 ≥ 0.98) 는 해당 없음 — **G8-D2 FAIL 은 그대로**. 드라이버 보강: unit 도중 같은 host 에 colcon build/test·pytest·ctest 가 나타나면 그 unit 을 즉시 중단 (부하 시도로 보존) 하고 한가해진 뒤 다시 돈다. **판정 기록은 둘 다** — 원 판정 (사전 규칙, tennis 83/200 FAIL) 과 재실행 판정 (93/200 PASS, 5843232090); 판정 모집단은 같은 200 투척 (seed 601–604). 근거: 기전이 rig 이고 (§4.4 S8-E 결과 — 사전 규칙 ① `sim_stall` 은 sim 스텝 간격만 봐서 이 부하를 잡지 못한다) 규칙이 결과를 보지 않으며 같은 투척을 다시 재므로 모집단이 깨끗하다. 기각: (a) 사전 규칙 판정만 기록 (원 판정으로 병기됨) · (c) 시행 단위 무효 사유 `sim_slow` + 보충 seed 605→ (모집단이 보충 투척으로 일부 바뀐다) |
| 설계 (이의 없음) | G8-C·G8-E 는 **2×2 factorial** (lead × γ) · G8-D 성공 = **truth 기반** (아래) |

**단위와 순서.** A → B → C (B 데이터 필요) → D (B 의 lead 지도) → E (D-12 floor 사전 고정 필요) → F (A 도구·B lead; E 의 문서 마감과 독립).

- **S8-A 측정 substrate — RT tick 바이트 동일.** 허용 변경: lifecycle 의 read-only 파라미터 미러 (`wait_pose`·`T_arm`·`lead_enable`·`T_freeze`), `catching_sim_trials` 의 동결 분포 표본 (`--dist`·seed·ω 기록, 기본 인자 출력 불변)·재무장 False→True, `sim_ur5e_p1b.launch.py` 의 `sim_lanes:=` (접촉 truth·clock lane 을 `<session>/sim/` 에), `rtc_tools` (`catching_trials` 오프라인 평가 신설 — Wilson·τ̂ LS·t_c 분해·D-3 공변량·A⊥B·충격량 짝짓기, `vision_lane_probe` 전 지평·공분산·innovation·nis 덤프, `analyze_clock_phase` verdict), GUI 진행 카운트. 새 rtc_tools 코드는 로봇 상수를 갖지 않는다 (catch frame·프레임 합성·dt 는 config — ARCH-1). 골든 = 파일럿 재현
- **S8-B lead 보상 (sim overlay).** ① 지도 재실행 (완료 — D-S8-13: sim τ 0.05, s35b 163/180) ② 2×2 factorial **4 arm × 4 블록 × 25 seed = 16 기동** (≈ 400 발; arm 마다 같은 seed 목록, 블록 안 arm 순서는 seed 로 무작위). overlay 는 기동 때만 읽혀 arm 을 바꾸려면 재기동해야 하므로 종전 "한 세션·블록 무작위" 는 성립하지 않는다 (2026-09-24 정정, #537 5808613906). 세션 디렉터리가 분 단위 이름이라 `max_log_sessions:=40`·기동 사이 분 경계 대기 → G8-E·G8-C ③ 튜닝 세트 ≥ 200 (별도 seed) → `sat_ticks` (시행당 오발화 ≤ 0.5 %)·`stale_committed_max_s`·`track_err_abort` 제안값을 두 YAML 에 명시 ④ p̂ 보고. G8-A2 sim 시나리오 (연속 2 회·abort 직후 재투척) 도 이 세션에서 기록. **결과 (2026-09-24, #537 5810866922; τ 0.05 플랜트, 개발 PC non-RT)**: 2×2 400 발 (쌍 불일치 0) 성공 lead on·계획 γ **47/100** · lead off·계획 γ 4 · lead on·γ0 2 · lead off·γ0 1. **G8-E PASS (sim)** — lead 반영 서보 잔여 중앙값 2.6 vs 94.4 mm, 100/100 쌍 감소 (Wilcoxon p 3.9e-18); 종전 판정량 (`cmd_meas_gap_mm`) 은 93 vs 94 mm 라 거짓 FAIL 이었을 것. G8-C (lead on) 성공 47 vs 2 (McNemar 47:2), t_c 총 간극 26 vs 70 mm, 접촉 v_rel 1.10 vs 2.89 m/s. lead × γ 상호작용 유의. D-S8-5 트리거 미충족 (APPROACH 교체 0 % 계획 γ · 14–16 % γ0; held Δp_c 는 로그에 없어 NOT_EVALUATED). G8-A2 600/600 순환, abort 직후 재투척 5/5. 튜닝 세트 (seed 201–204) **94/200 = 0.47** (Wilson [0.40, 0.54]) — 당시 floor 0.5 미달, 이를 보고 사용자가 floor 를 0.35 로 낮췄다 (§1a 기준 1; 튜닝 데이터는 본 평가에 합치지 않는다). `sat_ticks` 60 이 튜닝 3/200·2×2 lead on 2/100 에서 발화 (튜닝 3 건 중 2 건은 truth 성공) → 판정을 끈 재측정 (같은 200 발) 연속 max 61 · p99 58 · ≥ 70 0/200 → **p1b 80**. `stale_committed_max_s` 는 COMMITTED 스냅샷 age 최대 65 ms (< t_stale) 라 **0.10 유지·명시**. `track_err_abort` 는 t_c 전 피크 0.21 rad (전 arm) · RETREAT 0.058 → **p1b 0.42** (종전 1.54 는 τ 0.2). iiwa7_leap 은 현값 (60 · 0.10 · 0.3) 을 명시만 하고 S8-D 에서 잰다. 사고: 추정기 activate 응답 유실 1 기동 (rig 실패, 같은 seed 재실행) · Stop hook 의 `colcon test` fixture 가 실제 `logging_data/<분>` 에 써서 1 기동 diag 헤더 오염 (헤더 교체로 복구)
- **S8-C 조건부 RT.** D-S8-6 손 timeout · (트리거 충족 시) ⑦ 램프 연속 — E-6 대상 테스트는 별도 spec 커밋 · (D-S8-8 (b), 2026-09-25) 판정에 손 관절 q·토크 근거 추가 → G7-E·G7-C 재기록 · `T_release_timeout` 은 E2E 배수 기본값 + sim 측정 확인 (§7.3) — 배수는 상수가 아니라 profile 에서 유도한다 (L6 §6: p1b 2.36 s · leap 1.45 s; 상수 3 은 leap 을 매번 disarm). ⑦ 은 D-S8-5 트리거 미충족 (APPROACH 교체 0 %) 으로 하지 않는다. RT-1~10, `/code-review`·`/security-review`
  - **S8-C 결과 (2026-09-25, 브랜치 `feat/s8c-hand-timeout`, SPRINT #537 5823089966 → 보정 5823291259).** 측정 구성은 S8-B 튜닝과 같다 (p1b sim τ 0.05, overlay `catch_lead_on`, `--dist s35b`, 드라이버 `s8c/drv` 는 private). **교정** (seed 301·302, 100 발, 손 증거 끔): truth 실패 51 발은 모든 관절 ρ ≥ 0.9999·토크 ≤ 0.007 τ_max, truth 성공의 막힌 관절은 ρ 0.797–0.924·토크 ≥ 0.999 τ_max (대부분 `thumb_cmc_fe`) → 간극 중점으로 `rho_min` 0.4 · `rho_max` 0.95 · `effort_frac_min` 0.5 · `t_persist` 0.1 (SPRINT 후보 0.9 는 ring 관절 0.924 를 놓쳐 조정). **검증** (seed 303·304, 100 발, 임계 동결 overlay): false-Captured **0/52** (Wilson 상한 0.069) · 결합 false-Missed **8/48** (0.167, Wilson [0.087, 0.296] < 0.43 PASS) · 같은 투척 지문만 22/48 → 8, 손 증거가 올린 14 발 전부 truth 성공 · **G7-E 재기록: 판정·truth 일치 92/100 (지문만 78/100)**. 잔여 8 = 손가락이 공 위로 `q_close` 까지 닫힌 경우 6 (관절 증거로 불가시) + 막힌 관절의 $|\dot q|$ > `qd_tol` 2 (공이 흔들림 — 키를 늘리지 않고 기록만; 교정의 3 발도 같은 원인). **G7-C 재기록**: 합성 잡음 (σ_q 5 mrad·σ_q̇ 0.02·σ_τ 10 %) 빈 손 오경보 0/20000 (q_close·q_pre 각각), stalled 관절 샘플 검출률 0.988 (실기 `t_persist` 창 통과 ≈ 0.54 — S10). **`T_release_timeout`**: release → $q_{pre}$ 200 발 506–516 ms (p99 516) → 2 × p99 = 1.03 s ≤ 유도값 2.36 s PASS, hang 0. p1b 출하 YAML 에 `capture` 블록 (provisional) 을 넣었다. 단위: `test_catching_hand_capture` 13 (mutation ≤→<·AND→OR red) · params +17 · 전이표 +1 · 시나리오 +7 (persistence 제거·disarm 제거 mutation red) · reset probe +2 · G7-D alloc +1 (손 증거 평가 + timeout 경로 할당 0) · `rtc_tools` `catching_trials` +12 (`hand_hold_window.csv`·`tips_only_verdict`·`t_release_to_pre_ms`)
- **S8-D leap.** leap sim profile + 투척 세트, τ̂ 지도 재실행으로 G8-D2 평가 또는 `NOT_EVALUATED(선행시간)` + 실측값, G3-D
  - **S8-D 결과 (2026-09-26, PR #581, SPRINT #537 5835585007 → 확정 5835793705 · 결과 5836802390 · 계획 밖 판단 3 건 (상자 유효성 조건 · 씬 교체 · `T_release_timeout` 5.0) 사후 승인 5841231282; 개발 PC non-RT, 드라이버 `s8d/drv`·지도 `s8d/map` 은 private).** **플랜트**: leap sim 팔 kp ×4 (D-S8-14) → τ̂ LS **51.3–52.8 ms** (7 관절, R² ≥ 0.98, 교정·검증 동일). **지도** (`catchability_map` + `catch_gate_map`, T_arm 0.05 · 첫 계획 0.215 s · d_eff 0.1047 · a_dec 10): 대기 자세 라운드 4 회 (탐색 격자 840, S3.5b nom3 자세에서 출발) 열림 67 → **244** → 103 → 98 — 수렴하지 않아 (7 축 영공간) 두 번째 라운드 자세 `[-0.7174, 1.5729, 1.7260, -0.9021, -0.2900, 1.2010, 2.1347]` 를 출하 `planner.wait_pose` 로 (종전 출하 자세는 0). 세밀 격자 (4200·4900 투척, 편차 ±6°) 에서 p1b 폭 상자의 최대는 **앙각 86–88° · 180/180** 이었으나 스모크에서 **공이 상승 중 대기 손에 부딪혔다** (truth 최고 높이 0.30 m) — 지도는 비행 중 공–로봇 충돌을 보지 않는다. 그래서 **유효성 조건** "상승 구간 (정점 + 20 ms 까지) 에 대기 자세 로봇과 2 cm 이상 떨어짐" (MuJoCo 정적 검사, private `clearance.py` — 해당 스모크 투척을 28 ms 접촉으로 재현) 을 더해 다시 골랐다: **동결 상자 거리 0.95–1.05 m · 릴리스 0.10–0.20 m · 편차 ±6° · 속력 2.85–3.05 m/s · 앙각 78–80°** (180 투척 모두 비충돌; 열림 164/180 @ 0.215 s, 격자 안쪽). **씬**: 출하 `scene_right_with_object.xml` 은 대기 자세 아래 테이블·물체가 있어 homing 중 손가락이 사과·상판에 걸려 A2 가 막히고 `track_err` 로 해제됐다 → 두 overlay 가 `scene_right.xml` (바닥만 — 지도의 가정) 로 바꾼다. **선행시간**: 실측 첫 계획 (APPROACH 진입) 194 발 p50 **0.195** · p95 0.237 · max 0.293 s → 상자 열림 176 / 121 / 14 (각 180 중) — **지도가 비지 않으므로 G8-D2 는 평가 대상** (§1a). **G8-D2 (D-S8-14 D4: 판정은 S8-E)**: 검증 seed 503·504 (100 발, `catch_lead_on`, 임계 동결) truth 성공 **44/100** (Wilson [0.35, 0.54]), 지도-열림 부분집합 42/95 ([0.35, 0.54]); 교정 seed 501·502 는 49/100. 무효 (rig 실패) 1 기동 (추정기 activate 응답 유실, 같은 seed 재실행); 계획 없음 5 발은 실패로 셌다. 슈퍼바이저 × truth: CAPTURED 44 전부 성공 · MISSED 51 전부 실패. **G3-D**: 시행별 launch → commit 계획 주기 중 `plan_valid` 비율 p50 0.17 (p05 0.11 · p95 0.25, 검증) — 첫 유효 plan 전 주기가 대부분이다; APPROACH 교체 **0 / 200 시행** (교정·검증). **슈퍼바이저 임계** (교정은 `catch_lead_on_unbounded` 로 판정 끔, S8-B 방식): ref 포화 연속 최대 0 → `sat_ticks` 60 유지 (자료가 임계를 정하지 못함); COMMITTED 스냅샷 나이 최대 66 ms < t_stale → `stale_committed_max_s` 0.10 유지; 추종 오차 최대 t_c 전 0.096 · DECEL/HOLD 0.142 · RETREAT 0.060 · **첫 homing 0.238 rad** (IDLE·armed 도 검사됨) → `track_err_abort` **0.48** (2 × 0.238, §7.3 3-3). 검증에서 세 임계 발화 0 (포화 연속 max 30, 오차 max 0.108, 나이 max 98 ms). **`T_release_timeout`** (S8-C 이월): 유도값 1.454 s 로는 교정 100 순환 중 **37 회 HAND_TIMEOUT** — RETREAT 진입 → 전 관절 q_tol 0.01 이내 p99 2.47 s. 원인은 손 관절 감쇠·armature 가 서보 kd 에 더해지는 느린 개방과 q_tol 0.01 이 MCP·엄지 마찰 사대역 (frictionloss/kp = 0.016 rad) 안에 있는 것 → **명시값 5.0 s** (2 × p99, q_tol 은 기본값 유지) · 검증 release → q_pre p50 1.39 · p99 1.69 s, HAND_TIMEOUT 0. **캡처 판정** (S8-C 이월): 교정에서 truth 실패의 모든 관절 토크 ≤ 0.10 × max_torque, truth 성공의 막힌 관절 ≥ 0.57 (주로 검지 5·6·7) — ρ 는 가르지 못한다 (실패 관절 ρ 0.68 까지) → `effort_frac_min` **0.33** (간극 중점), `rho_min` 0.4 · `rho_max` 0.95 · `t_persist` 0.1 은 p1b 값. 검증: false-Captured **0/56** (Wilson 상한 0.064) · 결합 false-Missed **0/44** (같은 투척 지문만 5/44, 손 증거가 올린 5 발 전부 truth 성공). **분해** (검증 중앙값): t_c 총 간극 36–41 mm · CLIK 27–29 mm (p1b 2 mm — 7 축 CLIK 이 기준을 덜 따른다, 후속) · 서보 7 mm · 공의 첫 손 접촉이 t_c 보다 약 58 ms 이르다. 코드: `integrated_bringup/integrated_bringup/sim_lanes.py` (두 sim launch 공용) · leap `sim_lanes:=true` · leap overlay 2 · leap profile json · `FROZEN_DISTRIBUTIONS["s35b"]["iiwa7_leap"]` · `rtc_tools catching_trials` G3-D 열·`--gate-map`·sim 전용 프로파일의 `sim.yaml` 로봇 설정 · leap YAML (`wait_pose`·`T_release_timeout`·`capture`·임계·주석). RT tick 무변경
- **S8-E 본 평가 (계획 D-S8-16 2026-09-26, #537 5841469779 → 5841651825 — 완료 2026-09-26).** floor **0.35 동결** · n_valid 200 (보충 규칙 ①b) · 성공 = sim truth · 무효 = rig 실패만 (기계 판정 5 종 ①) · Wilson 97.5 % 단측 하한 (z 1.96) + ITT 하한 + 슈퍼바이저 × truth 혼동행렬 + δ(t_commit)·δ(t_c)·tick overrun 공변량. arm: `ur5e_p1b` tennis (seed 601–604, overlay `catch_lead_on`) · `beanbag` (같은 seed, overlay `catch_lead_on_beanbag` = `mujoco_simulator.projectile_ball.ball_type` + 동일 컨트롤러 절; McNemar 쌍 비교, 마찰 변화 병기 D-S8-11) · `iiwa7_leap` (seed 701–704, S8-D 동결값) — 각 50 발 × 4 기동, 총 600 발. 모든 unit 에 `record:=true` capture + probe dump (②). 착수 전 확인된 도구 공백 (#537 5841469779) — 분석기 모집단이 launch srv `accepted` 하나뿐이라 truth 없는 시행이 실패에 섞임 · `nees_summary`·`independence_test` 호출자 0 · 단일 세션 입력 · G7-B3 집계 없음 · `ref_saturated_max_streak` 전체 max 하나 — 은 C1·C3 가 닫았다 (아래 도구). 커밋 (PR #582 → `9e7967b5`): `f0ff9818` docs (D-S8-16) · `8baf18cc` C2 beanbag overlay · `79a025de` C1 무효 분류·pool · `879957c0` C3 시간 정렬·G8-B·G8-C2; rtc_tools 1038 테스트 · integrated_bringup overlay 테스트 green. RT tick 무변경 (브랜치 diff 에 컨트롤러 소스 없음)
  - **S8-E 결과 (2026-09-26, #537 결과 5842632283 · D-S8-17 5843117293 · 재실행 5843232090; 개발 PC non-RT, `ROS_DOMAIN_ID=88`, viewer off, 드라이버는 private).** 12 unit 전부 DONE, **무효 5 종 (srv_refused · not_launched · controller_silent · lane_drop · sim_stall) 전부 0** → 보충 seed 불필요, ITT 하한 = Wilson 하한. **사전 규칙 원 판정 (5842632283)**: tennis **83/200** (p̂ 0.415, 하한 **0.3489**, FAIL — 통과선 84) · beanbag 167/200 (0.835, 하한 0.7773, PASS) · leap **83/200** (0.415, 0.3489, FAIL). **host 부하 발견**: tennis 4 unit 중 3 개와 beanbag 1 개에서 unit 도중 sim 이 실시간보다 느려졌다 (`rtf_trial_min` = lane 의 sim/steady 비의 시행 중 0.25 s 창 최소값; < 0.95 시행 tennis 601: 6 · 602: 0 · 603: 2 · 604: **20**, beanbag 602: 13) — unit 시작 때 부하 게이트는 통과했지만 도중에 같은 host 에서 다른 세션의 테스트가 겹쳤다; leap 은 전 구간 ≥ 0.98. 그 시행의 성공은 tennis 5/28 (18 %) 대 나머지 78/172 (45 %, 하한 0.3809), beanbag 4/13 대 163/187. **기전**: sim 공 stamp 는 발사 기준 sim 시간축 ("발사 시 벽시계 + sim 경과") 이라 RTF < 1 이면 벽시계보다 뒤처지고, 컨트롤러의 steady 시계 나이 검사가 입력을 stale 로 본다 — 부하 unit 의 미종결 시행은 전부 `BALL_STALE` 로 TRACKING→ARMED (tennis_604 4 발, beanbag_602 2 발), 스모크의 부하 beanbag ABORT 도 `BALL_STALE_LONG`. 사전 규칙 ① `sim_stall` 은 sim 스텝 간격만 봐서 이것을 잡지 못한다. **D-S8-17 (사용자 (b), 재실행 전 선언 — §4.4 표)** 으로 tennis 601·603·604 · beanbag 602 를 같은 seed 로 재실행 (13:21–13:38): 네 unit 모두 **첫 시도에** `rtf_trial_min` < 0.95 시행 0 (드라이버가 unit 도중 host 의 build/test 출현을 감시), 원본은 `<unit>.loaded`, 무효 0. **재실행 반영 판정 (최종)**: **G8-D tennis 93/200 (p̂ 0.465, 하한 0.3972) PASS** · beanbag **175/200** (0.875, 하한 0.8220) PASS · **G8-D2 leap 83/200 (0.3489) FAIL** (재실행 대상 아님) — 원 판정 FAIL 과 재실행 판정 PASS 를 **둘 다 기록**한다. unit 별 원본 → 재실행 (같은 투척, 부하 걸렸던 시행 / 부하 없던 시행): tennis 601 23 → 27 (6 발 3 → 2 / 44 발 20 → 25) · 603 23 → 21 (2 발 1 → 0 / 48 발 22 → 21) · 604 15 → 23 (20 발 **1 → 13** / 30 발 14 → 10) · beanbag 602 34 → 42 (13 발 **4 → 12** / 37 발 30 → 30) — 부하 시행의 회복이 `BALL_STALE` 기전의 확인이다. **주의 — sim 재현 편차**: 부하가 없던 시행도 같은 투척에서 unit 당 ±5 발 달라진다 (원인 미확인, 노드 간 메시지 타이밍으로 추정). 재실행은 부하 효과 제거와 함께 이 편차도 새로 뽑는다. **슈퍼바이저 × truth (G7-E 기록)**: tennis 재실행 CAPTURED 79/79 참 · false-Missed 14 (원 68/69 · 15 · 일치 184/200) · beanbag 재실행 false-Missed 53 · false-Captured 1 (원 56 · 1 · 일치 142/200 — 공이 지문이 아닌 링크·손바닥에 실림, S7 과 같은 한계) · leap 일치 197/200. **McNemar** tennis 대 beanbag (같은 200 투척): 재실행 tennis 만 성공 9 · beanbag 만 91, p 3.3e-18 (원 13 · 97, exact p 4.7e-17) — beanbag 이 압도적으로 높다 (반발 0.1, D-S8-11 복합 효과). **leap**: 상자 전체 83/200 이 1차 (D-S8-15/16 ③), 지도-열림 부분집합 83/190 (하한 0.3683) 은 공변량; 순환 미종결 9 발은 전부 TRACKING↔ARMED 반복 (`TRACK_CHANGED`) 으로 plan 없이 끝남 = 실패. **공변량 (D-3 S3.1b 충족)**: 시계 공변량이 있는 시행 600 (+ S8-B 튜닝 요약 198, 합산 병기) ≥ 200. |δ(t_commit)| p50/p95/max leap 0.06 / 1.1 / 5.1 ms; tennis 원 0.27 / 118 / 230 ms · δ_max max 12.4 s (부하 unit) → 재실행 0.24 / 1.25 / 5.1 ms, 재실행 `rtf_trial_min` 최소 0.95. t_c 분해 중앙값 (tennis / leap, 원 판정 데이터): CLIK 2.0 / 27.1 mm · 서보 2.6 / 6.7 mm · pred 79 / 44 mm · 첫 손 접촉 − t_c −23.5 / −57.2 ms
  - **나머지 게이트 (5842632283; 재실행이 바꾼 값은 5843232090 로 표기).** **G8-A** `NOT_EVALUATED(제어 PC)` — dev PC smoke: 주기 초과 tick 이 있는 시행 tennis 27/200 (368 tick; 재실행 뒤 2/200) · beanbag 16/200 · leap 3/200 (sim 모드 jitter 는 항상 0). **G8-A2** PASS (S8-B 600/600 · 재투척 5/5 기록) — S8-E 600 발도 RETREAT→ARMED 재무장 585/600, 미종결 15 는 모두 계획 전 TRACKING↔ARMED (위). **G8-H FAIL (9 분기 중 2)** — `EstopTickPublishesThisTicksBody` 선례 (그 tick 의 식별 필드 + 블록) 를 따르는 테스트는 E-STOP·stale 입력 둘뿐 (`test_demo_catching_controller.cpp:1740`·`:1755`); generation 불일치 (`TRACK_CHANGED`)·지평 부족·plan 없음 (`:1770` 은 plan 블록만, tick 식별 없음)·ABORT_SAFE·S7 손 단계 조기 반환·S8-C `HAND_TIMEOUT`·손 관절 캡처는 모드·사유만 본다. Compute 자체는 매 tick 무조건 기록을 발행하므로 (`controller.cpp:2581`) 결함이 아니라 **테스트 공백**. **G8-B FAIL** (3 arm, 지평 0.1·0.25 s; leap 0.5 s 는 `NOT_EVALUATED(첫 접촉 전 표본 0)`) — 목표 시각 ≤ 첫 접촉 표본만. 평균 NEES (0.1 / 0.25 / 0.5 s) tennis 0.58 / 0.28 / 0.18 (CI [0.57, 0.59] 등; 재실행 뒤 같음) · beanbag 0.59 / 0.28 / 0.17 · leap 0.60 / 0.32 — 3 보다 한참 작다: **공분산이 과대 (보수적)**, coverage_95 1.00, NaN 0 %, 편향 ≤ 18 mm. **G8-B2** PASS (S5.2 레이아웃 해시 테스트 기록). **G8-C** PASS (S8-B 기록, 계획 γ 47 대 γ 0 2). **G8-C2 = G3-H FAIL (직교성 기각)** — 정확 결합 tennis 196 · beanbag 197 · leap 191 (근사 0; 재실행 반영 pool 은 tennis 200 · beanbag 200 · leap 191 = 591/591). tennis 원 |A| 64 mm · |B| 98 mm · E|A+B|² 7136 대 E|A|²+E|B|² 16660 mm² (재실행 7978 대 16984) — 백색화 교차공분산 대각 ≈ −0.64 (CI 가 0 을 안 포함): A 와 B 가 **음의 상관**이고 L3 §4.6 직교 분해는 간극 분산을 약 2.3 배 (원 tennis) 과대추정한다. beanbag·leap 도 같은 방향 (leap 4588 대 12474) — 세 비는 ≈ 2.1–2.7 배. |A| 64 mm 는 같은 구간의 예측 오차 크기 (0.25 s 중앙값 39 mm · 0.5 s 79 mm) 와 맞는다. **G8-C3 · G3-E** 삭제 — 기록만: `ref_saturated` 연속 > 0 시행 tennis 158/200 (p50 17 · p95 53 · max 68 tick; 재실행 165/200 · p95 50) · beanbag 161/200 · leap 5/200. **G8-E** PASS (sim, S8-B). **G7-B3** 충격량 상관 기록 · 토크 `NOT_EVALUATED(sim clamp)` — 첫 손–공 접촉, 예측 Δp = m·v_rel 대 ∫F dt: tennis 원 n 199 기울기 1.70 [1.28, 2.00] · 절편 −0.029 N·s · Spearman 0.53 (p 9e-16) → 재실행 기울기 1.60 [1.11, 1.81] · Spearman 0.27 (p 1e-4); beanbag 원 기울기 0.75 · ρ −0.18 → 재실행 0.10 · ρ −0.40; leap 기울기 −0.09 · ρ 0.00 — 판정 임계가 정의돼 있지 않아 보고만. **G7-E** 기록 (위 혼동행렬). **G3-D** PASS (기록) — plan 유효율 p50 tennis 0.14 · beanbag 0.13 · leap 0.17, APPROACH 중 교체 0/600, 첫 plan 0.20 s
  - **도구 (C1 `79a025de` · C3 `879957c0`, 사용법 SSoT 는 rtc_tools README).** `catching_trials`: 시행마다 `invalid_reason` 한 개 (① 우선순위), truth·중앙값·공변량은 유효 시행만, 요약에 `n_valid`·ITT 하한·`--floor` 판정 (PASS / FAIL)·`ref_saturated` 연속 분포·G7-B3 충격량 회귀·steady 시계로 결합한 tick overrun 공변량. `catching_pool`: unit 을 선언 순서로 합산해 n_valid 목표에서 절단, Wilson 단측 판정, (seed, idx) 쌍 McNemar. **lane 기준 시각 정렬**: sim-sync 에서 컨트롤러 `t_relative` 는 sim 축인데 러너의 시행별 offset (wall − t_relative 중앙값) 은 sim 이 실시간보다 느리면 흐른다 — 부하 스모크 unit 이 발사 짝 3 중 2 를 잃고 창이 최대 1 s 어긋났다. 그래서 시행을 realtime 발사 시각으로 lane 발사와 짝짓고, 발사 시각은 lane 의 sim 축에서 (세션 상수 sim − t_relative 는 commit 마다 plan bridge 에서), 발사 기준 stamp 는 시행별 상수 하나로 옮긴다; lane 이 없으면 중앙값 offset 이 fallback 이고 요약이 어느 경로인지 적는다. `--eval-samples` 는 `sim_capture_evaluate` 예측 표본을 유효 시행에 묶되 **목표 시각이 첫 접촉 전인 것만** 쓴다 — 평가기는 접촉 후 참값까지 넣어 0.5 s z 편향이 −1.19 m 였다 (발사 창 해석). `--probe-dump` 는 commit tick 의 소비 스냅샷을 probe dump 와 정확 결합한다 (G8-C2 A/B 분해·독립성 검정)
  - **후속 (범위 밖, 기록).** ① G8-B 공분산 과대 (NEES ≈ 0.2–0.6) → ball_perception 쪽 · ② G8-C2 음의 상관 → L3 §4.6 예산식 재검토 입력 · ③ G8-H 테스트 공백 7 분기 (generation 불일치·지평 부족·plan 없음·ABORT_SAFE·S7 손 단계 조기 반환·`HAND_TIMEOUT`·손 관절 캡처) → S9 또는 별도 PR · ④ 분석기의 `t_c` 열은 sim 이 느려지면 ±100 ms 어긋난다 (`plan_t_c_s` 가 steady 잔여 시간) — G8-C2 는 정확한 stamp t_c 를 쓰고, 분해 열은 골든 수치 보존을 위해 그대로 뒀다 · ⑤ leap 의 `TRACK_CHANGED` 반복 9/200 · ⑥ unit 도중 host 부하 감시·중단 (D-S8-17 드라이버 보강) 은 private 드라이버에만 있다 — repo 러너 `catching_sim_trials` 에는 없고 `catching_trials` 는 `rtf_trial_min` 을 공변량으로만 낸다 (§12)
- **S8-F 손 근처 투척 (탐색, Epic SPRINT·G8-D·D-18 밖).** 질문: 손 반경 ≤ 20 cm 로 오는 공을 작은 보정만으로 더 빠르게 받을 수 있는가. 비행 시간 T 는 중력이 상한 (2·v_z0/g) 이라 (도착 속력 v, T ≥ T_min, 측방 오프셋 r ≤ 0.2 m, 오프셋 각 ψ, Δz) 로 매개화하고 거리·앙각·입사각은 도출·기록한다 (종전 "d = 0.7·v" 는 도달 조건 $v_{min}^2=g(h+\sqrt{h^2+d^2})$ 로 불가능). 설계: 중앙 손 위치 r 0 속력 절벽 40–60 발 → 손 위치 3 곳 (사용자가 viewer 로 정한 `wait_pose` overlay) × LHS ≈ 300 발 → 로지스틱 GLM 의 v50(r) 지도 + 행별 Wilson, 총 ≈ 950 발 ≈ 80–100 min. 러너 `--aim-at-hand` 는 `catchability_map.integrate_flight` 위 shooting, 목표점 통과 오차를 truth 로 ≤ 2 mm 검증. 사전 예상: 허용 v_rel ≈ 1 m/s (L6) 라 v ≳ 2 m/s 에서 성공 ≈ 0. 결과는 별도 docs 커밋 (게이트 표 무영향). **재도출 (2026-09-26, #537 5843905916 → 개정 5844345538 → 확정 5844391177)**: `T_arm` 0.05·`T_freeze` 0.37·첫 plan 0.20 으로 T_min 0.57 (worst 0.61) → 설계 T [0.65, 0.8]; 자유도는 (v, 입사각 α, T) 셋이고 Δz 는 도출량 — 테이블 상판 (릴리스 공 표면 ≥ 0.084 m) 때문에 **정면 입사로 v < 3.5 m/s 는 불가**, 느린 공은 85° 급강하 lob 만; 항력 무시 시 릴리스 0.1 m 오차라 `aim_at_hand` 는 도착 상태에서 RK4 **역적분**. 사용자 의도 (임의 대기 자세에서 switch · 우선순위 = 시간 내 도달 > 상대속도) 를 코드와 대조해 세 곳이 어긋남을 확인했다: `wait_pose` 는 YAML 고정 (채택 기능 없음), `catch_box` 는 출하 자세 주변 판정 gate, 도달시간·γ 창은 둘 다 순위 gate 인데 w_γ 5 vs w_t 1. 그래서 **S8-F-1** = 출하 wait_pose 고정 + overlay 로 상자 넓힘 (x·y ±1.1 · z [0.15, 1.2]) + 도달 우선 점수 (w_t 5 · w_γ 1) 와 출하 점수의 A/B, **S8-F-2** (wait_pose K 자세 · switch 시점 채택 기능) 는 후속. 사전 예상은 S8-E (v 3.2 에서 46.5 %) 로 반증돼 v50(r 0) ≈ 3–4 m/s 로 다시 썼다
  - **S8-F-1 결과 (2026-09-26, #537 5845711680; PR #583 머지 `3278963a` — `/code-review` 9 건 반영 뒤 재분석 결과 불변, 5846071918; 개발 PC non-RT).** 9 unit **1068 발** (절벽 tennis 56 · beanbag 56 · lob 56 · LHS reach_first 600 · shipped_score 300, 같은 seed 앞 300 이 A/B 쌍), 무효 0, 조준 검증 전 시행 0.000 mm (truth 대 항력 모델 RMS max 1.9 mm), rig 실패 3 회 재실행. **COMMIT 은 1011/1012** — 계획기는 거의 항상 시도하지만 게시 plan 의 93–100 % 가 `rank_reach`, 95–100 % 가 `rank_gamma` 순위 gate 에 걸린 채 commit 된다 (두 단계 GLM 의 COMMIT 단계 퇴화). **CATCH (truth)**: tennis 절벽 (r 0 · T 0.65 · 정면) 3.5 → **3/8**, 4.0·4.5 → 0, 5.0 → 1, ≥ 5.5 → 0; LHS 600 발 29 (4.8 %), **v50(r 0) 3.63 [3.00, 3.97]** · r 0.05 → 3.41 [2.83, 3.75] · r 0.10 → 2.83 [1.85, 3.48] · r ≥ 0.15 미해결 (< 3.5), v ≥ 4.5 는 r 전 구간 10/399; Wilson 셀 v [3.5, 4) 에서 r 0 → 5/12 · r [0.15, 0.2) → 2/11 (r 의존 약함 — pred 79 mm 가 r_cap 24 mm 를 지배한다는 S8-E 분해와 일치). 기구: γ_f 계획 중앙값 0.67 (v 3.5) → 0.24 (v 7) — γ_max = η_v·v_max/v = 3.15/v — 로 접촉 v_rel 1.36 → 5.1 m/s, 공–포구점 최근접 d_min 4 → 40–65 mm (r_cap 24), `REF_SATURATED` abort 12/56. 접촉 v_rel 로지스틱 P(catch) = σ(0.33 − 1.16 v_rel): 1.0 → 0.30 · 1.4 → 0.22 · 2.0 → 0.12. **beanbag 절벽 38/56** (7, 4, 6, 7, 5, 5, 4 / 8) — 7 m/s (v_rel 2.25) 까지 절벽 없음 → tennis 절벽의 절반 이상은 반발. lob (85°) 6/56 — 2.5 m/s 3/14 (γ_f 0.44 · v_rel 1.44). **A/B 도달 우선 점수 대 출하 점수 (같은 300 발): catch 17 vs 21 (McNemar p 0.58) · commit 299 vs 300 — 효과 없음** (후보 전부가 두 순위 gate 에 같이 걸려 penalty 균일). **해석**: 막는 것은 계획기 우선순위가 아니라 팔 속도·가속 한계 (`reference.v_max` 3.5 → γ 하락 → v_rel 초과; commit 뒤 0.37 s 안 도달 실패 → d_min·`REF_SATURATED`) 와 손의 반발 흡수. **후속 순서 (2026-09-26 사용자 결정 — 권장 순서 채택, #537 5846528236)**: ② `reference.v_max`·`a_max` 를 sim 플랜트 (τ 0.05, kp/kd) 가 실제로 내는 값으로 재측정 — 가장 싸고 나머지 셋의 입력이며 이 예산이 곧 절벽 위치 (γ_max = η_v·v_max/v) 다; sim 예산은 sim 판정에만 쓰고 실기 값은 S10 → ① γ 창을 `v_max` 단일값이 아니라 실측 추종 능력 (t_lead 안 도달·추종 속도) 으로 재정의하고 같은 `hand_cliff` 로 A/B — 도달 gate 의 판정 승격은 plan 93–100 % 를 없애 시도 자체가 사라지므로 **비권장** → ④ S8-F-2 wait_pose 변경 (overlay 만) — r 의존이 약해 자세보다 속도 예산이 지배하므로 ② 확정 뒤에야 K 자세 결과를 해석할 기준이 생긴다 → ③ 손 반발 흡수 (컴플라이언스·폐쇄 프로파일) — beanbag 38/56 대 tennis 4/56 이 상한이고 v ≳ 5 에서는 손이 유일한 여지, sim 손 접촉 모델 대 실기 검증 항목 동반. 각 항목은 착수 시 `[SPRINT]` 로 성공 기준을 #537 에 올린다. S8-F-1 판정 unit 9 개의 sim 세션은 S8-E 규약대로 unit 폴더 `session_copy` 에 보존. 도구: overlay `s8f_reach_first`/`s8f_shipped_score`/`s8f_reach_first_beanbag`, `catchability_map.aim_at_hand`, 러너 `--dist hand_cliff|hand_lob|hand_lhs`, `rtc_tools catching_hand_near` (2 단계 GLM · v50(r) · Wilson · A/B · planner_events). 게이트 표 무변경
- 연속 재계획 측정 (S0.7 R1 결정의 확인): 시행마다 발사 → 첫 plan 시각, `APPROACH` 중 교체 횟수·사유, 첫 plan 전 후보 탈락 사유 분포. 첫 plan 이 가장자리 후보 전멸로 늦어져 R2 최악 경우에 가까운 시행 비율을 보고한다

| 게이트 | PASS 기준 | 판정 입력 |
|---|---|---|
| 시스템 | G8-A, G8-B, G8-B2, G8-C, G8-C2, G8-C3, G8-E. **S8-E 판정 (2026-09-26, 수치는 §4.4 S8-E 결과)**: G8-A `NOT_EVALUATED(제어 PC)` (dev PC smoke — 주기 초과 tick 이 있는 시행 tennis 27/200 (368 tick, 원; 재실행 2/200) · beanbag 16/200 · leap 3/200, sim 모드 jitter 는 항상 0) · G8-B **FAIL** (NEES ≈ 0.17–0.60 ≪ 3 — 공분산 과대 (보수적); leap 0.5 s `NOT_EVALUATED(첫 접촉 전 표본 0)`) · G8-B2 PASS (S5.2 레이아웃 해시) · G8-C PASS (S8-B, 계획 γ 47 대 γ 0 2) · G8-C2 (= G3-H) **FAIL** (A·B 음의 상관 — 직교 분해가 간극 분산을 ≈ 2.1–2.7 배 과대추정) · G8-C3 삭제 — 기록만 · G8-E PASS (sim, S8-B) | — |
| 성공률 | G8-D (ur5e_p1b)·G8-D2 (iiwa7_leap): **truth 기반 성공** (HOLD 끝부터 대기 자세 release 까지 공이 손에 있음) 의 Wilson 95% 하한 (97.5 % 단측, z 1.96) ≥ floor, 로봇별, n_valid 200. 슈퍼바이저 판정은 truth 대비 혼동행렬로 병기. 무효 = rig 실패만 (srv 거부·lane drop·sim stall·미발사; 기계 판정 5 종·우선순위는 D-S8-16 ①) — "plan 없음·abort" 는 실패. 전체 발사 수·무효 수·무효 사유·ITT 하한 (무효 = 실패)·δ 공변량을 함께 보고 (D-S8-3·D-S8-4 (c)). beanbag arm 은 같은 seed·같은 floor 로 병기, tennis 대 McNemar (D-S8-16 ④). **S8-E 판정 (2026-09-26)**: G8-D (p1b tennis) **PASS** 93/200 · 하한 0.3972 (D-S8-17 재실행 반영) — 원 판정 83/200 · 0.3489 **FAIL** 병기; beanbag **PASS** 175/200 · 0.8220 (원 167/200 · 0.7773 PASS); G8-D2 (leap) **FAIL** 83/200 · 0.3489; 무효 0 이라 ITT 하한 동일 | floor 0.35 (D-12 — 사용자 사전 고정, 2026-09-26 동결 D-S8-16), n_valid 200 (D-S8-3; 보충 규칙 D-S8-16 ①b), seed 601–604 / 701–704 (D-S8-16 ⑥) |
| 소거실험 | G8-C (γ)·G8-E (lead) 는 2×2 factorial (lead {off, on} × γ {0, 계획}) 같은 seed 쌍 분석 — 성공 McNemar, t_c 간극·도착 시각 오차·v_rel·충격량 Wilcoxon, 상호작용 보고. **G8-E 판정 = 계획 γ 에서 lead 반영 t_c 서보 잔여 ‖FK(q_meas(t_c)) − FK(q_cmd(t_c − T_lead))‖ (T_lead = diag `t_arm_s`) 중앙값이 lead on 에서 감소 (부호 일치 + 쌍 Wilcoxon)**, 잔여 (1차 지연의 비-순수지연분) 보고, 성공률 차는 기록. **2026-09-24 교체 (E-9, #537 5808509678 · 5808613906)**: 종전 판정량 ‖FK(q_meas(t_c)) − FK(q_cmd(t_c))‖ 는 lead on 에서 q_cmd(t) 가 t + T_arm 을 겨냥하므로 (`MakeNowLead`) 의도된 선행을 잔여에 더해 **거짓 FAIL** 한다 (사전 확인 3 투척: 186 mm ↑ vs lead 반영 35 mm ↓, lead off 153). 종전 값은 `cmd_meas_gap_mm` 로 기록만 하고, `ref_vs_true` 도 ref(t_c − T_lead) 로 읽는다 (`rtc_tools catching_trials`). lead off 에서는 두 값이 같다. sim G8-E 는 **50 ms 1차 플랜트** (D-S8-13 의 sim 서보 게인) 위 검증이며 UR5e 이득을 예측하지 않는다 | τ̂ (S8-A) |
| clock 오차 (D-3) | S3.1a 는 §5.1. **S3.1b (부하) 는 S8-E 자체 시행 (p1b tennis·beanbag 400 + leap 200) 으로 ≥ 200 을 충족**하고 S8-B 튜닝 200 은 요약 JSON 합산으로 병기한다 (D-S8-16 ⑤; S8-B 원본 세션은 삭제돼 ct 산출물만 남았다). ε_clk,alloc 은 **판정이 아니라 공변량**이다 (D-S8-4 (c)) — 무효율 상한으로 run 을 NOT_EVALUATED 로 만들지 않는다. **S8-E (2026-09-26): S3.1b 충족 (공변량)** — 시계 공변량 시행 600 (+ S8-B 튜닝 요약 198 합산 병기) ≥ 200 | — |
| 이월 | G7-B3 (손–공 첫 접촉 episode 만, 예측 Δp 대 측정 ∫F dt 의 기울기·절편 CI, Spearman n ≥ 50; 토크는 `NOT_EVALUATED(sim clamp)` — 손 토크가 forcerange 3.0 에 클램프된다) · G8-H (S8-C 의 새 분기 포함 기록 — S7 결과는 표에 PASS 로 기록돼 있지 않다; S8-C 는 시나리오·reset probe 테스트만 추가했고 이 구조적 기록 자체는 아직 갱신하지 않았다) · G3-D 는 S8-D 에서 leap 기록 (plan_valid p50 0.17, 교체 0/200 — §4.4 S8-D 결과; 판정은 S8-E) · G3-E = G8-C3 (L8 §9.1 v0.5 삭제 — `REF_SATURATED` 빈도 분포 기록만, 판정 없음) · G3-H = G8-C2 (p̂_live 는 diag `input_snapshot_sequence` ↔ probe dump `snapshot_sequence` 정확 결합, D-S8-16 ②b). **S8-E 판정 (2026-09-26)**: G7-B3 충격량 상관 **기록** (판정 임계 없음) · 토크 `NOT_EVALUATED(sim clamp)` · G7-E **기록** (슈퍼바이저 × truth 일치) · G8-H **FAIL** (9 분기 중 2 — 테스트 공백이지 결함이 아니다: Compute 는 매 tick 무조건 기록을 발행, `controller.cpp:2581`) · G3-D **PASS (기록)** · G3-E = G8-C3 삭제 — 기록만 · G3-H = G8-C2 **FAIL** | — |
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
- 실기 T_close,tot 종단 간 측정 (S4.3 에서 이월, 2026-09-20 — L6 §7 L6.5, G6-D). S4.0 의 sim 전용 가드는 S5.1 에서 E-8 승인과 함께 이미 없어진 상태여야 한다
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
- a_bound: g + 항력 가속 상한 (항력 k — S3.8 이 2026-09-20 결정으로 빠졌으므로 L0 §4.1 의 문서 대표값 0.0229 1/m)
- ε_clk,alloc: L3 §4.6 오차 예산 중 시계 항 ‖v‖δ 에 할당한 몫. **r_cap 이 정해지면 판정이 열린다 (S4.5, 2026-09-20 — §5.1 재판정)**. ε_clk,alloc 은 고정 예산이 아니라 **목표 속력에 비례**한다는 점이 재판정의 핵심이다

### 5.1 S3.1a 실측 (2026-09-20)

로봇 2종 × 발사 200 회, **풀 bring-up** (포구 스택 없음 = §5 의 "무부하"; 컨트롤러 없는 독립 노드 구동은 sim 이 매 step `sync_timeout_ms` 를 기다려 ~20 Hz 로 도는 **다른 실험**이다). 발사는 `/sim/launch_ball_at` 지정 발사라 모든 시행이 동일 방출 상태 `p0 = (4.81, 0.04, 1.75) m`, `v0 = (−4.0, 0, 3.5805) m/s`, `ω = 0` 이고 시드 RNG 를 소비하지 않아 재현 가능하다. 도구: `analyze_clock_phase` / `run_clock_phase_trials`.

| 구성 | 시행 | 거부 | lane drop | δ_max p50 / p95 / max | max pause p50 / p95 / max | 부호 (뒤처짐 / 따라잡음) | 95 % 를 통과시키는 ε_clk,alloc |
|---|---|---|---|---|---|---|---|
| `ur5e_p1b` | 200 | 0 | **0** | 1.514 / 4.931 / **18.212** ms | 1.627 / 4.171 / 18.388 ms | 196 / 4 | 49.808 mm |
| `iiwa7_leap` | 200 | 0 | **0** | 0.438 / 2.701 / **8.671** ms | 0.402 / 2.671 / 4.628 ms | 135 / 65 | 22.989 mm |

- **drop 0 이 꼬리를 믿을 근거다.** lane 이 넘쳤다면 가장 큰 δ·긴 pause 가 정확히 빠진 채 분포가 멀쩡해 보인다. 두 구성 모두 누적 drop 이 0 이다
- **무효율은 아직 계산하지 않는다** — ε_clk,alloc 이 없으면 유효/무효를 가를 수 없다. 판정은 **NOT_EVALUATED** 이고, 위 ε 열은 각 구성의 95 % 를 통과시키는 **역산 제안값**이다. 이 값을 임계로 채택하면 임계가 예산이 아니라 측정의 재서술이 된다
- **할당 비율 — 사용자 확정 2026-09-20 (권장안)**: 두 구성을 모두 덮는 ε_clk,alloc ≥ **49.8 mm** (p1b 가 구속) 를 예산이 확보해야 하는 **하한**으로 채택한다. 이 값은 측정의 역산이지 예산이 아니다

#### D-3 재판정 (2026-09-20, r_cap 확정 후)

위 49.8 / 23.0 mm 는 **`v_max` = 8.4 m/s (S0.7 가정값) 에서 역산한 값**이고 (`clock_phase.py` `DEFAULT_V_MAX_M_S`), ε_clk = ‖v‖δ 는 **속력에 비례**한다. S4.2·S4.5 가 받을 수 있는 최대 속력을 LEAP **2.26 m/s** · P1b **1.84 m/s** 로 내렸으므로 (L6 §4.5) 필요한 ε 도 ×0.269 · ×0.219 로 줄어든다. 예산 우변은 L3 §4.6 의 `n_σ`=2 로 `r_cap/n_σ` 다. 각 손은 **자기 $\Vert v\Vert_{\max}$** 에서 판정한다 — 그보다 빠른 공은 시계와 무관하게 손이 못 받는다.

| 구성 | ε_clk,alloc @8.4 m/s | 손의 $\Vert v\Vert_{\max}$ | 그 속력에서 필요한 ε | r_cap | 예산 `r_cap/n_σ` | 시계 항 비중 | 판정 |
|---|---|---|---|---|---|---|---|
| `iiwa7_leap` | 22.989 mm | 2.26 m/s | **6.19 mm** | 31.0 mm | 15.5 mm | 40 % (분산 16 %) | **PASS** |
| `ur5e_p1b` | 49.808 mm | 1.84 m/s | **10.9 mm** | 24 mm | 12.0 mm | 91 % (분산 82 %) | **PASS(provisional)** — 2026-09-21 |

- **LEAP 은 PASS 다.** 시계 항을 뺀 나머지 (vision σ_c·σ_ℓ, 추종 σ_trk) 에 √(15.5² − 6.19²) = **14.2 mm** 가 남는다
- **P1b 는 통과하지만 여유가 없다 (2026-09-21).** 사용자 제공 자세로는 포획 영역이 비어 r_cap 이 존재하지 않았고, 자세를 탐색으로 다시 정한 뒤 r_cap 24 mm 가 생겨 판정이 열렸다 (L6 §4.5). 시계 항이 예산의 91 % 를 쓰므로 나머지 (vision·추종) 에 √(12.0² − 10.9²) = **5.0 mm** 만 남는다. 공통 속력 2.26 m/s 에서는 필요 13.4 mm > 예산 12.0 mm 로 **FAIL** 이다
- ⚠️ **P1b 의 1.84 m/s 는 공식값이지 시연된 값이 아니다.** L3 §4.5 식은 폐쇄가 제때 시작되면 $d_{eff}$ 구간 안에서 공이 잡힌다고 가정하는데, 손끝 평면 통과 시 폐쇄를 발동한 fly-in sim 에서 P1b 는 0.25 m/s 130 중 9, 0.5 m/s 3, 1 m/s 이상 0 이었다 (L6 §4.5). 실제로 받을 수 있는 속력이 더 낮으면 필요한 ε 은 그만큼 더 줄어들므로 **D-3 판정 자체는 그대로 PASS 쪽**이고, P1b 의 목표 속력을 얼마로 둘지는 S4.4 의 몫이다
- ⚠️ 판정이 목표 속력에 묶여 있다. S4.4 가 속력을 다시 정하면 이 표를 다시 계산한다 — 8.4 m/s 를 그대로 노리면 LEAP 도 FAIL (필요 23.0 mm > 예산 15.5 mm) 이다
- **S4.4 이후 (2026-09-22)**: 조건부 go 의 포구 속력은 `iiwa7_leap` 1.2–2.6 · `ur5e_p1b` 2.7–3.8 m/s 다 (§4.4). ε 은 속력에 비례하므로 `iiwa7_leap` 은 위 PASS 가 유지되고 (필요 ≤ 22.989 × 2.6/8.4 = 7.1 mm < 15.5), `ur5e_p1b` 는 3.8 m/s 에서 필요 49.808 × 3.8/8.4 = **22.5 mm > 예산 12.0 mm 로 FAIL** 이다 — `ur5e_p1b` 의 조건부 go 에 D-3 이 네 번째 조건으로 붙는다. **S3.5b (2026-09-22)**: gate 지도가 연 `ur5e_p1b` 투척의 포구 속력은 2.7–3.85 m/s 라 판정은 그대로 FAIL 이다 (3.85 m/s 에서 필요 22.8 mm > 12.0 mm)
- `analyze_clock_phase` 는 여전히 `NOT_EVALUATED` 를 출력한다 (판정 문자열이 하드코딩돼 있고 r_cap 을 읽지 않는다). 도구 갱신은 S4.4 가 목표 속력을 확정한 뒤로 미룬다 — 지금 고치면 임계가 다시 바뀐다
- ⚠️ **두 구성은 완전히 동등하지 않다** — `ur5e_p1b` 는 컨트롤러 5개 (비활성 `demo_inference_controller` 포함), `iiwa7_leap` 은 4개를 인스턴스화한다 (그 프로파일은 정책 config 를 싣지 않아 CM 이 건너뛴다). 차이의 일부는 로봇이 아니라 bring-up 에서 올 수 있다
- 플롯 (§13 S3 의 CSV 플롯 요구): `analyze_clock_phase --plot` 이 시행별 δ_max·max pause 분포를 낸다

| 항목 | 방법 | 기준 |
|---|---|---|
| 무부하 (S3.1a) | 구성별 (로봇 2종) 발사 ≥ 200 회 | 무효율 ≤ 5 % — **실측 완료 2026-09-20 (§5.1). LEAP PASS, P1b PASS(provisional)** (2026-09-21, 각 손의 $\Vert v\Vert_{\max}$ 기준) |
| 부하 (S3.1b) | 포구 컨트롤러 + 계획기 + `sim_estimator_node` 동시 구동, 구성별 발사 ≥ 200 회. **S6 에서 수행되지 않았고 S8-E 자체 시행 (p1b tennis·beanbag 400 + leap 200) 으로 ≥ 200 을 충족한다** (S8-B 튜닝 200 은 요약 합산 병기 — D-S8-16 ⑤) — **충족 (S8-E, 2026-09-26)**: 시계 공변량 시행 600 + S8-B 요약 198, |δ(t_commit)| p50/p95/max leap 0.06 / 1.1 / 5.1 ms · tennis 재실행 0.24 / 1.25 / 5.1 ms (원 0.27 / 118 / 230 ms — host 부하 unit, D-S8-17; §4.4 S8-E 결과) (clock lane 을 `sim_lanes:=true` 로 켬, 2026-09-24) | **D-S8-4 (c) 로 교체**: ε_clk,alloc 은 판정이 아니라 **공변량** — 시행별 δ(t_commit)·δ(t_c)·tick overrun 수·최대 tick 간격을 기록하고, D-3 효과는 같은 seed 투척의 부하 A/B (affinity on·최소 프로세스 대 인위 부하) 로 상계한다. 파일럿 (`260924_1218`, non-RT 개발 PC): δ_max p50 4.5 / p95 10.7 / max 16.7 ms, ε 12 mm 이면 유효 7/25 — 무효율 상한을 그대로 쓰면 개발 PC 에서 run 이 전부 NOT_EVALUATED 가 된다 |
| 예측 일관성 | vision 예측 궤적 대 ground truth (같은 wall 시각 축) | 오차가 δ 가 큰 구간에서만 커지는지 확인 (기록) |
| stamp 도메인 | `sim_estimator_node` 의 stamp 가 wall 인지 | `use_sim_time=false` 에서 wall — **확인 2026-09-20** (S3.4: 예측 origin stamp = 카메라 capture stamp = sim 의 wall `now()`; sim 재시작에도 역행 없음) |

시행 수 200·무효율 5 % 는 제안값이다 (사용자 확인, §7.3). 어느 구성이든 무효율이 상한을 넘으면 `/clock` 방식을 포함해 D-3 을 다시 결정한다. ~~성공률 평가(S8)는 유효 시행으로 계산하되 전체 발사 수와 무효 사유를 함께 보고해, 무효 제외가 성공률을 편향하지 않는지 드러낸다.~~ **S8 (2026-09-24, D-S8-4 (c))**: 성공률 평가에서 clock 위상 오차는 무효 사유가 아니다 — **무효 = rig 실패만** (srv 거부·lane drop·sim stall·미발사), "plan 없음·abort" 는 실패, δ 는 공변량으로 병기하고 무효를 실패로 센 ITT 하한을 함께 보고한다 (§1a · §4.4 S8). 위 무효율 상한은 S3.1a 무부하 판정에만 남는다.

## 6. D-7 계획기 스레드 구성

기존 MPC 스레드와 같은 방식으로 생성하고 기능만 planner 로 한다.

**E-7 결과 (2026-09-23 사용자 승인, 결정 J).** 계획기는 MPC 와 **같은 역할** (RT 루프에 해를 공급하는 solver 스레드) 이므로 **새 layout role 을 만들지 않고 기존 `mpc` role 을 그대로 쓴다** — `SelectThreadConfigs().mpc.main` 을 `SpawnMpcThreadIfNeeded` 와 같은 방식으로 받는다. 레이아웃 값·manifest·generator·3 oracle·`all_configs`·`rt_cores()`·검증기 표 변경 0, profile 은 `mpc_on`/`mpc_off` 그대로 (결정 B — launch 의 `enable_mpc` 하나). 스레드 이름은 `mpc_main` 이라 검증기의 기대 표가 포구 컨트롤러가 active 일 때도 성립한다. 두 컨트롤러가 함께 configure 돼 `mpc_main` 이 둘 존재할 수 있지만 CM 은 active 컨트롤러를 하나만 두고 이전 것을 deactivate (`Pause`, CV 블록) 하므로 같은 코어에 동시에 도는 FIFO 는 하나다 — 이것을 R-1 switch 테스트가 확인한다. D-7a (FIFO/OTHER) 판정은 `mpc_main` 값을 바꾸는 것이라 MPC 에도 적용되고, 둘이 갈리면 그때 role 을 분리한다 (별 E-7). 아래 2026-09-19 분석의 "E-7 `[CONCERN]` 에 담을 것" 목록은 새 role 을 전제한 것이라 **J 로 대체**됐다 (기록으로 남긴다).

분석 결과 (2026-09-19):

**MPC 스레드의 실체.** 공용 기반은 `rtc::PeriodicRtThread` (rtc_base threading, 헤더 전용)다: jthread + stop token, 스레드 진입 시 `ApplyThreadConfigVerbose` (실패해도 무시하고 계속 실행), `clock_nanosleep` 주기, overrun 카운터, Pause/Resume, t0..t3 timing payload. `WaitForNextTick`·`JitterMeaningful` 은 virtual 이다. `rtc::mpc::MPCThread` 는 그 위의 얇은 subclass 이고 (`OnTick` = 상태 읽기 → `Solve` → 결과 게시), DemoWbc 가 `std::unique_ptr` 로 소유한다. 수명: `on_activate` 에서 layout profile 게이트 → lazy spawn → `Resume`, `on_deactivate` 에서 `Pause`, **join 은 소멸자에서만** (use-after-free 수정 이력). 배치는 `repo_scripts/config/thread_layout.yaml` 의 `mpc_main` role (tier 6 이상 slot 3 FIFO 60, tier 4 는 slot 3 OTHER).

**planner 구성 (권장).**

- 클래스: `PeriodicRtThread` 의 **형제 subclass** (catching planner thread). `MPCThread`/`MPCHandlerBase` 를 상속하면 PlanSnapshot 을 `MPCSolution` 에 억지로 넣게 되므로 쓰지 않는다. 같은 기반을 쓰는 4번째 소비자라 P5·ARCH-3 을 만족한다. 탐색 코어는 rtc_controllers `catching`, 스레드 소유는 `integrated_bringup` 바인딩 (D-1)
- 수명: DemoWbc 관용구 (configure 에서 전 버퍼 할당, activate 게이트 → spawn → resume, deactivate pause, aux 타이머로 `planner_timing_log.csv` drain) — **단 join 은 `on_cleanup` 에서** (S6-A 구현 시 `/code-review` 정정: 소멸자까지 살려 두면 oracle 로 재구성한 설정에서 resume 돼 plan box writer 가 둘이 된다). `Pause()` 는 진행 중 iteration 을 멈추지 않으므로 deactivate 뒤에도 한 번 게시될 수 있다 — RT 쪽이 `PlanSnapshot` 의 activation generation 으로 거른다 (D-23). DemoWbc 가 MPC 해에 이 장치를 두지 않은 것은 선례가 아니라 미해결 gap 이다
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

**복사하지 않을 것.** MPC 스레드의 cross-mode swap 은 phase 전환 때 그 스레드에서 handler 를 새로 만든다 (heap·YAML·try/catch) — invariants.md §RT Path 의 알려진 위반이며 planner 템플릿으로 쓰지 않는다. 통계 mutex 와 `fprintf` 는 E-9 결정으로 제거됐다 (2026-09-19, §7.3).

세부 선택 D-7a~d 는 §7.

## 7. 세부 결정과 후속 결정

### 7.1 확정 (2026-09-19)

사용자 결정 (P-):

- P-1 S9 이전 개발 기간의 E-STOP 임시 기준: 해제 후 자동 재개 금지 + q_c·CLIK 앵커를 q_meas 로 reseed 만 구현 (S5.1). 임시 기준이어도 E-STOP 경로이므로 **S5 착수 전 `[CONCERN] E-8`** 을 올리고 S5 게이트에 race oracle 과 `/security-review` 를 넣는다 (§4.4 S5). **E-8 승인 2026-09-22 (사용자, (a)~(d) 문구 그대로)**. 전체 정책은 S9
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
- D-15 sim profile 지평: 0.8 s, 간격 0.05 s, **16 점** (지평 0.05…0.80 s, t = 0 없음 — 2026-09-20 정정), ≤ 30 Hz (2026-09-19, S0.7 권장 채택). ball_perception sim profile 설정은 사용자가 하고, S3.4 가 실측으로 확인한다. S3.6 (2026-09-22) 이 목표 분포와 T_det 재실측으로 **1.0 s / 20 점**으로 올려 설정했다 (§4.4 S3.6 결과, §7.3)
- D-18 manipulability 정의: 팔 관절 열 5행 w₅ (병진 3 + 접근축 2), threshold 0.1 은 이 정의에 대한 값
- D-18 발사 영역: base frame 수평 거리 4 m 원호 위 (좌우 투척 포함), world z 1.5–2.0 m (사람 투척 릴리스 높이), 발사점 → 포구점 비행시간 T_f ≥ 1.0 s (2026-09-19, S0.7 결과 후 사용자 결정 — 짧은 직선 투척을 목표 분포에서 뺀다. 상한은 지도 결과)
- C-1 vision `validity`: 한 점이라도 VALID 가 아니면 메시지 전체를 거부한다. S3.4 실측에서 부분 무효가 실제로 나오면 S5.2 착수 전에 재검토
- C-2 `header.stamp` 기반 나이 거부는 두지 않는다. 원점 지연(수신 wall − stamp)은 진단으로만 남긴다 (오래된 원점은 절대 시각 기반 지평 검사가 거른다). 미래 stamp 거부는 별개 (§3.1)
- C-3 `planner.ik.manip_min` 은 D-18 게이트로 대체한다. **IK·게이트는 w₅ 로 풀고, 검증은 w₆ 로 해야 할 수도 있다** — §11 의 w₅/w₆ 병행 규칙
- C-4 포구 후보마다 IK seed 는 대기 자세로 고정한다. IK 반복·예산 증가는 S6.3 에서 측정
- S2 (2026-09-19~20 사용자 결정): PR 단위 S2.1 → S2.2 → S2.3a·S2.5 → 마감, S2.2a 행 선택형 확장 (L5 §5.1), S2.5 도구는 rtc_tools python, golden 비교는 Release 비트 일치·sanitizer 상대 1e-12, `QPSolverWrapper` 비유한 회복은 golden 전에 선수정, **η_τ = 0.8** (§9, S3.5a 뒤 표본 범위만 바꿔 재생성)
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

**결과 (2026-09-23).** 3~6 의 제어 PC 측정은 사용자 결정으로 **생략**했다. 정책은 2 의 초기값 (FIFO) 그대로이고, 계획기 코드는 1 에 따라 RT-1~10 을 지키므로 나중에 OTHER 로 바꿔도 `thread_layout.yaml` 값만 바뀐다. 재판정용 측정 CLI 는 #537 코멘트 5793878042 (`s6d.sh`, dev PC 에서 끝까지 동작 확인).

### 7.3 미결정

**사용자 결정이 필요한 것**

| 항목 | 내용 | 필요 시점 |
|---|---|---|
| ~~E-8 승인~~ | **승인 (2026-09-22 사용자)**: P-1 최소 계약 (a)~(d) 를 문구 그대로 (§4.4 S5). S5 게이트의 G7-H 4종 + `/security-review` 는 그대로 | ~~S5 전~~ |
| ~~E-7 승인~~ | **승인 (2026-09-23 사용자, 결정 J)**: 새 role 없이 `mpc` role 재사용 (§6) | ~~S6 전~~ |
| ~~D-24 결정~~ | **확정 (2026-09-22 사용자): (a)** `rtc_base` `DeviceState` 센서 lane 에 `recv_steady_ns`·`sequence`·`valid` (P5 — grasp 의 같은 gap 도 닫힌다, PROC-3 전체 회귀). (b) 는 포구 전용 mailbox 가 device 경로와 공존하는 중복 lane 이라 기각 | ~~S5 전~~ → S5.2e |
| ~~G5-E substrate~~ | **확정 (2026-09-22 사용자): (ㄱ)** fixture 전용 지연 주입 (테스트 전용 지연 큐, 런타임 불변) — `prediction.lead` = $T_{arm}$ 의 효과를 S6 전에 한 번은 관측한다. (ㄴ) `NOT_EVALUATED(substrate)` 는 선행 보상의 부호·크기를 S10 실기에서 처음 보게 되므로 기각 (§4.3 S5, L5 §9 G5-E, L8 §9 G8-E). **G8-E 부분은 2026-09-24 대체 (D-S8-1)**: sim 팔이 시정수 ≈ 200 ms 의 1차 지연이라 (L5 §4.4 정정) fixture 없이 **sim 런타임 lead on/off 비교**가 가능하다 — G8-E 는 S8-B 2×2 factorial 위에서 판정한다 (§4.4 S8). G5-E (fixture, S5.3 PASS) 는 그대로 | ~~S5 착수 시~~ → S5.3 · G8-E → S8-B |
| ~~QP solve time 예산~~ | **확정 (2026-09-22 사용자, provisional)**: tick 2000 µs (`control_rate` 500 Hz) 기준 **p99 ≤ 400 µs (20 %) · 최대 ≤ 1500 µs (75 %)**. 분위수로 거는 이유는 L8 의 "총량이 아니라 분산의 꼬리" 이고, 20 % 는 backend 왕복·로깅·계획기 스냅샷이 같은 tick 을 나눠 쓰는 것을 감안한 값. S5 실측이 훨씬 작으면 조인다 (§4.3 S5 RT 행, L5 §9 G5-C) | ~~S5 게이트~~ → S5 실측 후 재검 |
| ~~sim 공 lane 지터~~ | **확정 (2026-09-22 사용자) · 구현 완료**: 두 공 토픽의 stamp 를 **발사 순간을 기준으로 sim 시간축을 wall 에 얹은 값**으로 (`rtc_mujoco_sim`, epoch 은 wall 그대로, wall 과의 차이는 비행 안 위상 오차 δ 양방향 — D-2·D-3 불변; 첫 구현은 throttle 기준 anchor 였는데 발사 전 누적 지연 (컨트롤러 연결 전 idle·pause, 초 단위) 이 모든 stamp 에 박혀 포구 시각이 그만큼 이르게 틀리는 문제를 `/code-review` 가 잡아 발사 anchor 로 바꿨고, wall 을 안 앞서게 clamp 하는 변형은 sim 이 발사 pace 를 앞설 때마다 지터를 되돌려 (재실측 25 ms 초과 3.8 %) 기각했다 — 대신 sim 의 future 허용치를 δ 폭으로: profile `max_future_skew_s` 0.1). 발행률 상향 (속도 추정을 더 튀게 함) · wall 타이머 (`mjData` 잠금) · `init.max_span_s` 완화 (증상만 덮음) 는 기각. 재실측: stamp 간격 25 ms 초과 9.2 % → 0.02 %, T_det max 0.69 → 0.10 s (§4.4 T_det 재실측) | ~~S7–S8 전~~ |
| ~~sim profile 설정~~ | **확정·설정 (2026-09-22 사용자, ⑤ 후 한 번)**: `integrated_bringup/config/ur5e_p1b/ball_perception_sim_profile.json` = **1.0 s / 0.05 s / 20 점 / ≤ 30 Hz**, 측정 공분산 (5 mm)² 대각 (`position_noise_stddev_m` 과 짝), `max_future_skew_s` **0.1** (공 lane stamp 축이 sim 축이라 wall 을 δ 만큼 앞설 수 있다 — ⑤). 수정 전 필요했던 1.05 s / 21 점은 버스트가 만든 0.024 s 검출 때문이었다 (§4.4 S3.6 결과) | ~~S7–S8 전~~ |
| D-12 손 토크 | P1b 손 관절 한계: 설정·모델값은 모두 3.0 N·m (YAML `max_torque`, URDF `effort`, MJCF `forcerange`) 이고, 1.5 N·m 는 작성 시점 사용자 진술 (CATCHING_MASTER §1.3). **실기 실측 (2026-09-22 사용자): `thumb_cmc_fe` 최대 7 rad/s 에서 약 2 N·m — 운용 한계 1.5 N·m 와 양립하지 않는다** (L6 §4.2). nominal·continuous·peak·설정값 중 무엇을 운용 한계로 쓸지와 그 출처 | S7.3 충격 게이트 전 |
| D-12 나머지 | 공 사양, 실기 T_close,tot 측정 시점, 성공률 floor·시행 수. S4.4 는 출하 sim 공 (ITF Type 2) 으로 판정했다 — 공 사양이 바뀌면 fly-in (L6 §4.5) 부터 다시. **2026-09-24 (D-S8-3)**: 시행 수는 **n_valid 200** 확정, floor 는 사용자가 사용 목적에서 사전 고정 — **0.35** (0.5 provisional (#537 5808613906) 에서 S8-B 뒤 하향, **2026-09-26 동결** D-S8-16 — §1a 기준 1). 공은 tennis 1차 + `beanbag` preset arm (D-S8-11, 같은 seed 200 D-S8-16 ④) | S3.5b, floor 동결 완료 (2026-09-26) |
| ~~D-18 개정~~ | **확정 (2026-09-22 사용자, S3.5b 결정 A)**: sim 은 **현 씬을 유지하고 릴리스 높이를 0.2–0.5 m** (base 기준 +0.2…+0.5 m) 로 둔다 — 결과가 (릴리스 − base 높이) 에만 의존해 base 를 올린 씬과 같은 셀이다. 거리 1.0–1.5 m. 기존 4 m·T_f ≥ 1.0 s·릴리스 1.5–2.0 m 는 추정기 정확도를 위한 값이었고 제약이 아니다 (2026-09-21 사용자). base 를 올리는 씬은 실기 rig 의 base 높이가 정해질 때 (값 필요) — `ur5e_p1b` 는 hand-description 쪽 sibling scene 이어야 한다 (§4.4 S4.4 정정). 앙각·속력·T_f 범위는 §4.4 "S3.5b 결과" | ~~S3.5b 전~~ |
| ~~D-16 개정~~ | **확정 (2026-09-22 사용자, 결정 B)**: 도달시간 게이트는 **두 층을 병기**한다 — 출하 상수 box (런타임 함수) 와 **토크 검사 층** (그 이동 wait → q\* 가 $\eta_\tau\tau_{\max}$ 안에 드는가, 회전자 관성 포함). 판정은 토크 층이고 provisional — 런타임 구현 (계획기의 자세 의존 한계, CLIK 의 가중 box) 은 S6 에서 설계한다 (§9). **필요 시점이 "S6 전" 에서 "S3.5b 전" 으로 당겨졌다** — 도달시간 게이트의 $\bar a$ 가 지도를 정한다 | ~~S6 전~~ → S6 (런타임) |
| ~~1차 목표 로봇~~ | **확정 (2026-09-22 사용자, S3.5b 후)**: 1차 `ur5e_p1b`. G8-D2 (`iiwa7_leap`) 는 **조건부** — 실측 선행시간에서 gate 지도가 비어 있지 않으면 평가, 비어 있으면 `NOT_EVALUATED(선행시간)` 로 실측값과 함께 보고 (§1a). 기준에서 빼지 않는다. **`ur5e_p1b` 의 S6 착수 조건 셋**: fly-in 등가 `d_eff` (아래 행), D-16 토크 층의 런타임 구현 (§9), 다시 고른 대기 자세 (§11) — 이 셋이 빠지면 지도가 0 이었으므로 "1차" 는 무조건이 아니다. D-3 은 3.85 m/s 에서 FAIL (§5.1), 관절 속도 정격은 `sim.yaml` overlay 로만 (결정 D) | ~~S5 전~~ |
| ~~`reference.v_max`·η_v 적용 범위~~ | **확정 (2026-09-22 사용자, 결정 D)**: `v_max` 는 도출값 (수락 후보의 LP $v_{dir,\max}$ 최대 / η_v), η_v 는 관절 속도 한계에도 적용 — L3 §4.5, L4 §6. `ur5e_p1b` 의 관절 속도 정격 (2.0/3.0 → 3.1416 rad/s) 은 **`sim.yaml` overlay 로만** 올린다 (overlay 는 key 단위로 덮으므로 실기 `_base.yaml` 은 불변; 배선은 S5) | ~~S6 전~~ |
| ~~`planner.hand.d_eff` 의 뜻 (TBD-HAND-04)~~ | **확정 (2026-09-22 사용자)**: `planner.hand.d_eff` ≡ **시각 발동 fly-in 으로 잰 허용 상대속도 × $T_{close,tot}$** — P1b 1.0 × 0.2815 = **0.2815 m**, LEAP 1.0 × 0.1047 = **0.1047 m** (L6 §4.5). 포켓 깊이 (0.095 / 0.080 m, S4.5 접촉 sim) 는 접촉 물리량으로 L6·MASTER 에 남고 이 키에는 들어가지 않는다. 근거: 출하 포켓 깊이로는 `ur5e_p1b` gate 지도가 대기 자세와 무관하게 0 — §4.4 S3.5b 결과. 조건 둘: ① fly-in 은 **시각 발동** ($t_{cmd}=t_c-T_{close,e2e}$) 에서 잰 값이라 런타임 손 발동도 그 규칙이어야 유효 (S4a), ② 손 frame 등속·발동 지터 0 가정이라 값은 provisional. 대안으로 남긴 것: 별도 키 `planner.hand.v_rel_max` [m/s] — S6 가 γ 창을 배선할 때 재검토 | ~~S3.6 전~~ |
| ~~`supervisor.decel.a_dec`~~ | **확정 (2026-09-22 사용자)**: **10 m/s²** (provisional). 정지점 게이트가 단독으로 후보의 40 % 를 거르는 값이고, S4.4 가 토크 한계로 푼 $\hat v$ 방향 가속 (p1b 중앙값 43, 회전자 관성 10 배 가정 21–23 m/s², L4 §6) 의 절반 아래. `reference.a_max` 가 D-16 개정과 함께 정해지면 ≤ 검사 (L7 §4.3) | ~~S3.6 전~~ → a_max 확정 시 재검 |
| ~~선행시간 T_det~~ · L | **T_det 실측 완료 (2026-09-22, 24 비행)**: `stamp` 축 p50 0.075 · p05 0.027 · p95 0.322 s (§4.4 T_det 실측). 첫 계획 시각 T_det + L 은 p50 0.215 s 로 지도의 가정값 0.24 s 와 사실상 같고, H_req 는 가장 이른 검출에 구속돼 0.93 → **1.00 s** 로 올랐다. 꼬리 (max 0.69 s) 는 sim 공 lane 의 wall stamp 지터 때문이었고 **같은 날 고쳤다** (결정 ⑤, 아래 행) — 재실측 stamp p50 0.050 · max 0.100 s, 첫 계획 시각 max 0.24 s 로 24 비행 전부 지도 가정 안, H_req 0.99 s · n 20 (§4.4 T_det 재실측). `iiwa7_leap` 은 이 lead 에서 지도가 비지 않아 **G8-D2 가 평가 대상이 됐다** (11 / 2835, §1a). **L 0.14 s 는 여전히 가정값** — 계획기가 있는 S5–S6 후에 잰다 | ~~S3.6 병행~~ (T_det 닫힘) · S6 후 (L) |
| rollout 게이트 (L3 §4.8) | **결정 E (2026-09-22 사용자)**: S3.5b 에서는 `NOT_EVALUATED(S6)` — 함수가 S6.3 전에는 없다. 앞당기려면 `reference.a_max` 결정이 먼저 | S6 |
| ~~D-3 제안값~~ | 닫힘 (2026-09-20): 시행 수 200 확정. **r_cap 확정 후 재판정 완료 (§5.1)** — LEAP PASS (필요 6.19 mm ≤ 예산 15.5 mm, 2.26 m/s 기준), P1b PASS(provisional) (필요 10.9 mm ≤ 예산 12.0 mm, 1.84 m/s 기준 — 2026-09-21, 자세 탐색 후). 판정이 목표 속력에 묶여 있으므로 S4.4 가 속력을 확정하면 재계산 | ~~S3.1a~~ → ~~r_cap~~ → S4.4 |
| ~~d_eff · r_cap~~ | 닫힘 (2026-09-20 사용자 승인): LEAP `d_eff` 0.080 m · `r_cap` 0.031 m (접촉 sim, provisional). P1b 는 사용자 제공 자세로 파지 불가여서 자세를 탐색으로 다시 정한 뒤 (2026-09-21) `d_eff` ≥ 0.095 m (스캔 상한) · `r_cap` 0.024 m. 투척 보정은 미수행 (S7.1 로 선행조건 해소, sim S8 · 실기 S10) | ~~S4.4~~ |
| D-18 T_f 상한 | 하한 1.0 s 는 확정. 상한(정점 높이·포구 속력이 커진다)은 S3.5a 지도 결과로 제안 | S3.5b |

**S7 착수 전 확정 (#537 S7 결정 2026-09-23 — 코드 미착수, 착수 커밋 1(이 docs 정정)에서 문서화)**

| 항목 | 내용 |
|---|---|
| D-12 손 토크 (S7.3 게이트 입력) | sim G7-B3 는 설정·모델값 그대로 **3.0 N·m**, 실기는 **1.5 N·m** provisional 병기 — 운용 한계(nominal·continuous·peak·설정값 중 무엇을 쓸지)의 최종 확정은 **S10**. 그 전까지 토크 비교는 `NOT_EVALUATED` (L6 §4.2 실측 2 N·m 은 1.5 N·m 와 양립하지 않음을 이미 기록) |
| homing 위치 | **관절공간** (per-joint 사다리꼴, QP/CLIK 비의존) — `retreat_reference.hpp` 는 코드에 없다. **무장 latch 아래에서만** 실행한다 (활성화는 무장이 아니다, P-1 (e)) — L4 §5.3·L7 §4.1 |
| `REF_SATURATED` 판정식 | `ref.saturated` 가 연속 `supervisor.sat_ticks` tick (기본 **60**, provisional — 5 는 단위 fixture 의 정상 접근 포화 10·76 tick 을 잘라 100 으로 올렸고, sim 기준 15 + 변형 10 투척 (`260923_2336`) 의 정상 연속 길이 max 44 · p99 42.5 를 보고 50 으로 내렸다. 50 재측정 `260924_0013` 에서 이미 빗나간 공의 CLOSING 에서 1 회 발화, 미발화 최장 40·33 → 60, #537 결정 2026-09-24) 이면 승격 — 최종값은 S8 에서 정한다 (D-S7-4). **S8-B (2026-09-24)**: p1b **80** (판정을 끈 재측정 연속 max 61 · p99 58), iiwa7_leap 60 명시 — **S8-D**: 판정을 끈 교정에서 포화 연속 max 0 이라 자료가 임계를 정하지 못해 60 유지, 코드 기본값 60 은 키가 없는 config 용 |
| hold 규칙 | `robot.hand.hold.mode: close_target \| measured_offset` + `hold.delta_rad` — 출하 기본은 `close_target` |
| `supervisor.stale_committed_max_s` | 출하 **0.10 s** provisional. 조일 물리량(정지거리·공분산 성장 등)은 S8 에서 정해 조인다 |
| ν̄ / `PRED_INCONSISTENT` | S7 에서도 **미구현** — 생산자가 없다 (A-S5-5). `Reason` 은 표에 남고 발화는 0 (명시 면제). S8 이월 → **은퇴 (2026-09-24, D-S8-7 (a))**: 생산자를 만들지 않고 `io.pred.nu_reg` 도 은퇴 (L1 §6). 예측 일관성은 추정기 `/ball_perception/debug/{innovation,nis}` 를 `vision_lane_probe` 가 기록해 오프라인으로 본다 (S8-A). `Reason::kPredInconsistent` 는 전이표에 남되 발화 0 인 명시 면제 (L7 §4.2) |
| σ_ℓ 모니터 소비자 | S7 착수 대상이 아니다 — `planner_cycle.cpp` 가 σ_ℓ 를 갱신은 하지만 소비자가 없다. **S8 이월** (D-8 포화 빈도 측정과 함께) → **기록만 (2026-09-24, D-S8-7 (a))**: RT 소비자를 두지 않는다. `planner_events.csv` 의 `sigma_l` 로 S8-B 튜닝 세트에서 σ_ℓ(age) 를 보고 `stale_committed_max_s` 를 정하는 입력으로만 쓴다. σ_ℓ abort 는 S9 |

**S7 중 확정 (#537 결정 2026-09-24)**

| 항목 | 내용 |
|---|---|
| RETREAT release | RETREAT 는 손을 움직이지 않는다 — 닫힌 손은 판정 (Captured·Missed·Undetermined·Aborted) 과 무관하게 대기 자세 도착에서만 `q_pre` 로 연다. 아직 안 나간 commit 은 진입 시 취소 (손은 이미 `q_pre`). Q12 (Missed/Aborted 는 진입 즉시 Release)·Q14 (접촉 확정 후 abort 만 예외) 를 교체 — 지문만 보는 판정이 링크·손바닥 위 공을 Missed 로 읽어 포구 지점에서 떨어뜨렸다 (sim `260923_2336`). L7 §4.1 RETREAT 순서 |
| `supervisor.sat_ticks` | **60** provisional (100 → 50 → 60): 50 재측정 (`260924_0013`) 에서 이미 빗나간 공의 CLOSING 에서 1 회 발화, 미발화 최장 40·33. → **S8-B: p1b 80** (YAML 명시, 아래 S8 행) |
| G7-B3 충격량 상관 | **S8 이월** (사용자 결정) — S7 은 산출하지 않았다 |

**S8 착수 시 확정 (2026-09-24 사용자 — #537 코멘트 5804952754 → 5806998220 → 5807128700 → 5807193226 → 5807451403 → 5807720680 → 5807767896)**

결정 표 (D-S8-1~12 · C-25) 와 단위 분할은 §4.4 S8 이 SSoT 다. 이 절은 다른 §7.3 행에 미친 파급만 적는다.

| 항목 | 파급 |
|---|---|
| G8-E substrate | fixture (2026-09-22) → sim 런타임 lead on/off (D-S8-1) — 위 "G5-E substrate" 행 |
| `joint_cmd.lag.T_arm` (sim) | 출하 YAML 은 0 그대로. S8-B overlay (`sim_overlays/catch_lead_*`) 에서만 0.05 + `lead_enable`, 함께 `planner.freeze.T_freeze` 0.37 (D-S8-13 — L3 §4.11 하한 0.3615; 검증기는 0.36 도 통과시키므로 overlay 가 설계 하한을 지킨다). ~~τ̂ 0.2 · T_freeze 0.52 · horizon 0.66 · n_min 15~~ (C-25, D-S8-13 으로 은퇴). 출하 `T_arm` 값은 S10 (L5 §6) |
| D-3 판정 | 무효율 상한 → 공변량 (D-S8-4 (c), §5 표) |
| ν̄·σ_ℓ | 위 두 행 (D-S8-7 (a)) |
| `supervisor.sat_ticks`·`stale_committed_max_s`·`track_err_abort` | **S8-B 로 닫힘 (2026-09-24 사용자 승인, #537 5810866922)** — 두 로봇 YAML 에 명시: ur5e_p1b `sat_ticks` 80 · `stale_committed_max_s` 0.10 · `track_err_abort` 0.42, iiwa7_leap 60 · 0.10 · 0.3 → **S8-D (2026-09-26) 재측정 후 60 · 0.10 · 0.48** (포화 연속 max 0 · 스냅샷 나이 max 66 ms · 첫 homing 추종 오차 0.238 rad × 2). 근거는 §4.4 S8-B·S8-D 결과 |
| RETREAT 손 대기 timeout | `robot.hand.T_release_timeout` 신설 (D-S8-6, S8-C — 구현·반영 완료, PR #579 → `126e3513`). **값의 근거 (2026-09-25 사용자, 권장안)**: `T_close_timeout` 선례처럼 키가 없으면 $T_{close,e2e}$ 배수를 기본값으로 쓰고 (배수는 S8-C 에서 정함), S8-C sim 에서 release → $q_{pre}$ 도달 시간 분포를 재어 기본값의 여유를 확인한다. S8-B 는 hang 0 (G8-A2 600/600) 이고 그 분포는 남아 있지 않다. **배수 (S8-C, #537 5823291259)**: $m=2\max(1/\eta,\ \ln(S_{\max}/q_{tol})/\ln\tfrac1{1-\eta})$ — 토크 포화·1차 선형 두 극한 중 느린 쪽의 2 배 (L6 §6). 반영: L6 §5.3·§6, L7 §4.2·§4.4·§4.8 (손 키는 L6 §6 이 SSoT 라 L7 §6 에는 두지 않는다) |
| G8-D 성공 정의 | truth 기반 + 슈퍼바이저 혼동행렬, 무효 = rig 실패만, ITT 하한 (§1a) |
| L3 점수 가중치 · NLP 전환 · `reference.a_max` provisional · `budget.sigma_trk` | S8 데이터로 **제안만** — 가중치는 S8-E 뒤, NLP 전환 신호는 기록만 (결정 S9 전), `a_max` provisional 해제는 S8-E 실측 후, `sigma_trk` 는 서보 잔여로 제안. **2026-09-26 확인 (D-S8-16 ⑦)**: S8-E 는 넷 모두 제안·기록만 하고 값·YAML 은 바꾸지 않는다. **S8-E 제안 (2026-09-26, #537 5842632283 — 제안만, 값·YAML 무변경, 결정은 사용자)**: ① `budget.sigma_trk` (출하 0): p1b sim 서보 잔여 p95 5.0 mm · leap 7.9 mm → sim 에서는 0.005 m 제안 — 순위만 바꾸는 키라 실기 값은 S10 · ② `reference.a_max` (21, provisional): `ref_saturated` 가 p1b 시행의 약 80 % 에서 떠 21 이 자주 묶이지만 올릴 근거 (토크 한계 가속) 가 sim 에 없다 → provisional 유지 제안 · ③ L3 점수 가중치: APPROACH 중 교체가 600 발 중 0 이라 가중치 비교 데이터가 없다 → 변경 제안 없음 · ④ NLP 전환 신호 (기록): 첫 plan 0.20 s, plan 유효율 p50 0.13–0.17, 교체 0 — 전환을 시사하는 신호 없음. **S8-F-1 뒤 (2026-09-26 사용자, #537 5846528236)**: ② 의 "`a_max` provisional 유지" 는 재검토 — S8-F-1 에서 `reference.v_max`·`a_max` 가 곧 속력 절벽 (γ_max = η_v·v_max/v → 접촉 v_rel) 이라 두 값을 sim 플랜트 실측으로 재측정하는 것이 S8-F 후속 1 순위 (§4.4 S8-F-1 결과 후속 ②); ③ 가중치는 S8-F-1 A/B (도달 우선 대 출하 점수, McNemar p 0.58) 로도 효과 없음 — 순위 gate penalty 균일이 원인이라 가중치가 아니라 γ 창 재정의 (후속 ①) 로 간다 |
| CLOSING 1 tick 늦은 기록 | 문서 기록만 (D-S8-10 (a)) — 닫힘 명령 시각에 영향 없음 |

**단계에서 정할 것 (결정은 해당 단계)**

- ~~S1.7 η_v·D-16 가속 box 의 YAML 키 이름~~ — S1.7 에서 닫음: η_v 는 L3 §6 의 `planner.gamma.eta_v`, a_dec 는 `supervisor.decel.a_dec` 로 문서에 이미 있었다. 가속 box 키는 S2.5 에서 — `integrated_bringup/config/<robot>/derived_accel_limits.yaml` 의 `derived_accel_limits.<group>.qdd_max` 로 정했다 (컨트롤러 배선은 S5). S1.7 이 새로 둔 키: `core.ball.provisional`·`planner.catchability.manipulability_min.provisional`·`robot.hand.provisional` (문서는 provisional 을 산문으로만 표시, 기본값 true = fail-closed). catch frame 의 provisional (D-17) 은 `urdf.extra_frames` 쪽이라 S2.3a 에서 검증기에 연결. 세 키의 이름·기본값은 2026-09-19 사용자 승인
- ~~G0-C 의 ωh 경계 도달 불가~~ — 닫힘 (2026-09-19 사용자 결정): 범위는 그대로 두고 게이트 문구를 "범위 검사가 ωh 안정을 함의, 경계 공식은 범위 밖 ω 로 단위 검증" 으로 고쳤다 (L0 §5.3·§9)
- L7 전이표 (S1.8) 의 해석 3건 — **S7.2 에서 그대로 확인 (PR #571)**: `Reason::kNone` = 각 상태의 정상 전진, IDLE homing 은 `kIdle` 안, ARMED→IDLE (§4.5 조건 위반) 은 전용 사유가 없어 `kParamsTbd` 재사용 (`transition_table.hpp` 헤더)
- ~~S2.2a CLIK 확장 구조, S2.2 CLIK 세부 (관절별 속도 한계, q_c 평가 cache, 실패 후 재앵커, `anchor_drift_max`)~~ — 닫힘 (2026-09-19, L5 §5.1 표)
- ~~S3.1a ε_clk 할당 비율 제안~~ — 닫힘 (2026-09-20, §7.3 표 D-3 행)
- S5.2 (S3.4 측정 결과로) C-1 재검토 여부, vision 재시작 시 `snapshot_sequence` 되감김 처리, `frame_id` ↔ world, 유령 트랙 처리, 공분산의 시간 보간 정의와 nrt 파서 → 계획기 버퍼 전달 방식 (D-22 token 유지)
- S5.3 QP 비의존 관절공간 abort 식
- S7 homing 을 IDLE 하위 단계로 둘지 별도 Mode 로 둘지, `REF_SATURATED` 판정식, 손 hold 힘 한계를 position 목표로 표현하는 규칙, `stale_committed_max_s` 를 조일 물리량 (공분산 성장·포획 반경 오차 할당·abort 정지거리)
- TBD-WS-01 (바닥·작업셀 경계) — S3.5a catchability 지도에서 작업셀 경계를 입력으로 쓸 때 함께 정한다

**S5 착수 시 확정 (2026-09-22 사용자, 권장안 그대로 — #537 코멘트 5773959608)**

착수 전 코드 대조에서 계획 서술의 정정 8건이 함께 나왔다 (D-24 의 `valid` 는 `DeviceState::inference_enable` 로 이미 있고 추가분은 `recv_steady_ns`·`sequence` 둘 · 센서 lane 을 채우는 backend 는 3종이 아니라 2종 + CM 복사 (`ur_driver_native` 는 lane 자체가 없다) · 슈퍼바이저 본체는 S7.2 라 S5 는 S1.8 전이표 위의 얇은 driver · 컨트롤러 YAML 에는 sim overlay 가 없다 · §13 의 arm/disarm 채널이 없다 · `PinocchioCache::RegisterFrame` 호출부가 아직 없다 · `WbcState` 는 aux 타이머가 아니라 `PublishNonRtSnapshot` 로 발행한다 · `derived_accel_limits.yaml` 에 런타임 소비자가 없다).

| ID | 결정 | 근거 |
|---|---|---|
| A-S5-1 | S4.0 의 sim 전용 activation 가드를 **provisional 값의 실기 차단**으로 교체한다. backend 판정은 `ValidateCatchingParams` 의 `real_arm_config` 축을 고르는 데만 쓰고, 실기 config 에서 **소비 키**에 provisional·TBD 가 있으면 S4.0 과 같은 DISABLED (configure SUCCESS · activate 거부) 로 park 한다. sim 은 종전대로 configure FAILURE | E-8 승인으로 가드의 근거가 사라졌고 남는 규칙은 L0 §5.3 · L8 §5.4 다. park 인 이유는 2026-09-21 정정 그대로 (실기 p1b bring-up 이 같은 config dir 에서 이 컨트롤러를 인스턴스화하고, CM 은 한 컨트롤러의 configure 실패로 전체를 거부한다) |
| A-S5-2 | `io.future_tol` 은 실기값 **1e-3** 를 본 키에, sim 값 **0.1** 은 `sim.io.future_tol` 에 둔다 (sim config 에서만 활성·본 키를 덮음). `io.t_stale` 은 **0.10** 단일 키 | 컨트롤러 YAML 은 sim·실기가 한 파일을 공유하므로 sim 전용 값은 `sim:` 섹션 키여야 한다 (`sim.ball.drag_k` 선례). 값 자체는 공 lane stamp 가 sim 축이라는 D-3·⑤ 에서 온다 (profile `max_future_skew_s` 0.1 과 짝) |
| A-S5-3 | 조작 채널은 컨트롤러 노드의 읽기·쓰기 파라미터 **`catching.enable`** (기본 false). 파라미터 콜백은 atomic 만 갱신하고 RT tick 이 소비하며, **tick 이 E-STOP·fault 때 이 latch 를 내린다** | §13 GUI 의 arm/disarm 이 채널 없이 적혀 있었다. msg·srv 신설은 E-3 대상이고 파라미터로 표현되는 것에 쓸 이유가 없다. tick 이 내리므로 P-1 (c) "자동 재개 금지" 가 운용 규율이 아니라 메커니즘이 된다 |
| A-S5-4 | `snapshot_sequence` 되감김: 같은 generation 안에서 직전 수락값 이하이면 거부·카운트, **generation 이 바뀌면 last_seq 리셋**. 회복 경로는 generation 변화 또는 재활성화 | S3.4 에서 되감김 0 회 — 정책은 fail-closed 최소형 (L1 §4.4 그대로) |
| A-S5-5 | S5.2d 의 $\bar\nu$ 는 **S7 로 미룬다**. S5.2 는 $J$ (같은 generation 직전 궤적과의 점프) 만 진단으로 계산하고, 공분산은 token 을 실은 `CovarianceSnapshot` 으로 계획기(S6)에만 | 소비자 `PRED_INCONSISTENT` 가 L7 이고 **공분산의 시각 보간 규칙이 아직 없다** (L1 §4.5 미결) — 규칙 없이 구현하면 임의 정의가 박제된다. $J$ 는 규칙이 이미 있다 |
| A-S5-6 | D-16 가속 box 는 컨트롤러 YAML 키 `robot.arm.accel_limits_file` (변형 dir 상대, 기본 `derived_accel_limits.yaml`) + `robot.arm.accel_limits_group` 으로 **읽는다**. `adopted: false` 또는 파일 부재는 거부 | 도구 산출 파일이 SSoT 이고 값 복사는 금지 (L5 §6: `max_acceleration` placeholder 사용 금지). S5.3 이 첫 런타임 소비자 |
| A-S5-7 | CLIK 의 `q_min`/`q_max` 는 device config ∩ URDF 에서 `robot.arm.limit_margin` 만큼 안쪽, `qd_max` 는 device config `max_velocity` (관절별). `ur5e_p1b` 정격 상향은 `sim.yaml` overlay 로만 (결정 D) | L5 §6 "사본 금지" — 같은 값의 두 번째 출처를 만들지 않는다 |
| A-S5-8 | S5.5 의 oracle plan 은 `diagnostic.oracle_plan:` (`enabled`·`p_c`·`a_d`·`t_c_offset_s`·`gamma_f`) 로 주고, RT tick 이 고정 `PlanSnapshot` 을 만들어 소비한다 | §13 S5 의 "기준 vs 실제 추종 오차" 를 sim 에서 보이려면 런타임 plan 원천이 필요한데 계획기는 S6 다 |
| A-S5-9 | G5-E 의 지연 주입은 `test/include` 아래 fixture (설치되지 않음) 의 폐루프 plant 로 한다 — 런타임 경로 불변 | 확정 (ㄱ) 의 구현 형태. `catching_ball_fixture.hpp` 와 같은 격리 (미설치 → 프로덕션 타깃이 include 할 수 없다) |
| A-S5-10 | QP 비의존 abort 감속식: 관절별 $\dot q_i\leftarrow\mathrm{sign}(\dot q_i)\max(\lvert\dot q_i\rvert-\ddot q_{\max,i}\Delta t,\,0)$, $q_c\leftarrow\mathrm{clamp}(q_c+\dot q\Delta t)$ — 순수 코어 함수 (할당·QP 없음) | L7 §4.1 이 "정확한 식은 S5.3" 으로 남긴 자리 |
| A-S5-11 `[제안]` | **`reference.provisional` 신설** (invented key, 기존 셋과 같은 모양·같은 기본값 true) + 출하 프로파일에 `reference:` 블록 — `v_max` 는 L4 §6 의 확정 도출값을 전사 (p1b 3.5 · leap 1.8 m/s), `a_max` 는 **미결**이라 블록 전체를 provisional 로 두어 sim 은 경고, 실기는 차단한다. 값은 S4.4 실측의 보수적 끝 (p1b 21 · leap 35 m/s², 10× 회전자 관성 가정) 이고 `supervisor.decel.a_dec` = 10 위에 있다 (검증기 요구) | S5.3 이 `reference.*` 를 소비 부분집합에 넣었는데 출하 프로파일에 그 블록이 없었다. 소비 키의 TBD 는 sim 에서 configure 를 **거부**하고, CM 은 한 컨트롤러의 configure 실패로 **전체 컨트롤러**를 거부하므로 2026-09-22 실측에서 ur5e_p1b sim 로봇 전체가 안 떴다. **`a_max` 값 자체는 사용자 확정이 필요하다** (L4 §6 은 D-16 개정과 함께 정한다고 남겨 둠) |
| A-S5-12 | A-S5-1 의 "sim 은 configure FAILURE" 를 **sim 도 park** 로 바꿀 것을 제안 | 실기를 park 로 바꾼 근거 ("CM 이 한 컨트롤러 실패로 전체를 거부한다") 가 sim 에서 **동일하게** 성립하고, 2026-09-22 에 그 결과를 실제로 관측했다 — 증상이 "포구가 안 된다" 가 아니라 "로봇이 안 뜬다" 였다. A-S5-11 이 이번 발현은 막았지만 규칙은 그대로다. **채택 (2026-09-23 사용자, S6 착수 시 결정 3-2)** — S6-A 가 구현한다: sim 에서 소비 키의 TBD 는 configure FAILURE 가 아니라 park (activate 거부, 로봇은 뜬다). 범위·일관성 위반 (범위 밖 값, caging 간극) 은 여전히 sim configure FAILURE — 그것은 "아직 안 정한 값" 이 아니라 설정 실수다 |
| A-S5-13 | 준비(ARMED) 조건 상실을 **운동 중인 모드에서는 `ABORT_SAFE` 로** 보낸다 (전이표 7행 추가; `ABORT_SAFE` 자신은 행 없음 — 드라이버가 그 한 모드에서만 통과시킨다) | 전이 행이 없으면 모드가 그대로 남고, 드라이버가 법칙 호출을 멈춘 채 `WriteDeviceCommand` 는 실려 있던 명령을 계속 내보내 **관절 명령이 그 자리에서 언다** — L5 감속 계약이 금지하는 1-tick 무한 감속이다. 그리고 `IDLE` 로 안 돌아와 운전자의 disarm 이 화면에서 무효다. 실측: disarm 후 4 s·3240 tick 전부 `mode=APPROACH armed=false` (2026-09-23 `/code-review`) |
| A-S5-14 | 스냅샷 newness 는 번호가 아니라 **(epoch, 번호) 쌍 + `seen` 플래그**로 판정한다 (L1 §5.3 갱신) | ㄱ. A-S5-4 로 새 epoch 은 번호를 다시 시작할 수 있어, 겹친 번호의 첫 스냅샷이 반복으로 읽히면 **새 공의 표본을 소비하면서 TRACK_CHANGED 를 안 낸다**. ㄴ. 0 은 합법 `snapshot_sequence` 인데 `last_consumed == 0` 이 "미소비" 센티넬을 겸해, 0번으로 시작하는 lane 과 trial reset 직후가 이미 소비된 것으로 읽힌다. track 축의 `track_seen_` 과 같은 비대칭을 sequence 축에서 해소 |
| A-S5-16 | 수신축 clock 읽기를 `rtc::SteadyNowNs` (rtc_base/types.hpp, 나이 헬퍼 옆) 하나로 모은다 | 이 브랜치가 같은 `std::chrono` 표현을 3벌 새로 더했고, 전수 확인에서 **6벌**이 세 패키지에 흩어져 있었다 — 그중 `rtc_mujoco_sim/projectile_ball.hpp` 는 **이미 `rtc` 네임스페이스에** 있어서, rtc_base 에 넣는 순간 `redefinition` 으로 전체 빌드가 깨졌다. 즉 중복이 "정리하면 좋은 것" 이 아니라 **이미 컴파일 단위를 나눠야만 공존하던 상태**였다. 사본마다 "backend 와 같은 시계" 라는 주석이 붙어 있었고 그 주장은 우연으로만 참이었다 |
| A-S5-15 | 진단 두 필드의 **센티넬 정착** — `input_age_s` 미수신은 음수, `q_cmd` 는 실제 나간 명령(hold latch 포함)이고 명령이 없는 tick 은 NaN | 전자는 `now − 0` 이라 **steady-clock uptime** 을 나이로 실었다 (실측 373966.66 s × 28476행). 후자는 hold 중 0.0 을 실어 `‖q_meas − q_cmd‖` 를 오프라인으로 계산하면 존재하지 않는 수 rad 오차가 나왔다. 둘 다 그럴듯한 수라 소비자가 걸러낼 수 없다 |

**S6 착수 시 확정 (2026-09-23 사용자 — #537 코멘트 5785207660 → 5785720197 → 5785863361 → 5785978354 → 5786035728)**

착수 전 코드 대조에서 계획 서술의 정정 19건이 나왔다. 구현에 영향을 준 것: `SeqLock<PlanSnapshot>` 은 없었고 `plan_` 은 RT tick 소유 멤버였다 (→ `plan_box_` 신설) · RT → 계획기 상태 POD 가 전무했다 (→ `PlannerRtState`) · `cov_box_` 는 독자가 0 이었다 (계획기가 첫 독자) · `PeriodicRtThread::Start` 는 `frequency_hz ≤ 0` 이면 no-op 이다 (→ `planner.wake_timeout_s` 가 주기 상한) · `planner.budget_s` 는 실재 키가 아니었다 · `EvaluateReason` 에 plan 검사가 없었다 (→ 새 `Reason` 없이 token 불일치·나이 초과를 "plan 없음" 으로 읽는다) · `PlanOnce`·rollout·점수·히스테리시스 함수는 C++ 에 없다 (S6 가 신규 작성) · $\dot q^u$ 런타임 생산자가 없다 · 공분산 시각 보간 규칙이 없다 (→ 후보는 vision 격자 그대로).

| ID | 결정 | 반영 |
|---|---|---|
| E-7 · J | 승인. 계획기는 mpc role 재사용, 이름 `mpc_main` (§6) | S6-A |
| B | profile 은 `mpc_on`/`mpc_off` 그대로 — launch `enable_mpc` 하나, `enable_catching` 없음 | S6-A |
| C | **판정 게이트** = 입력 유한성 · IK 수렴 · manipulability (`arm_5row`) · 작업공간 `catch_box` + 정지점 `p_stop` 포함. **순위·진단** = 불확실성 · 도달시간 · γ 창 · commit 선행 · rollout · 오차 예산 (D-27) | S6-B |
| D | 점수 J (L3 §4.10) + 순위 게이트 탈락마다 벌점 → 최소 J. 판정 통과 0 일 때만 plan 없음 (`PlanReason` = 첫 병목) | S6-B |
| E | `PlanSnapshot::reason` 은 plan 없음 사유 전용. 시도했으나 순위 게이트 탈락은 계획기 CSV 의 게이트 비트마스크 (msg 는 D-20 동결) | S6-B |
| F | box 층 `TMinChecked` 는 순위 항. 토크 층 런타임은 `NOT_EVALUATED(runtime)` — K 의 dynamic 부등식이 들어오면 그것이 겸한다. **S6-C2 재판정 (2026-09-23)**: 계획 시점의 토크 층은 여전히 `NOT_EVALUATED(runtime)` 이고 (계획기의 도달시간 순위 항은 box 층), 실행 시점의 토크 한계는 `joint_cmd.accel_constraint: dynamic` 이 **실행층에서 강제**한다 (L5 §4.3). 둘은 다른 질문이다 — 계획기는 "그 시간 안에 갈 수 있는가" 를 보수적으로 순위에 반영하고, CLIK 은 "이 tick 의 명령이 토크 안인가" 를 hard 로 지킨다 | S6-B · S6-C2 |
| G | `planner.freeze.T_freeze` p1b 0.36 s · leap 0.19 s (§4.11 하한식, provisional) | S6-B |
| H | `planner.wake_timeout_s` 0.05 s | S6-A |
| I | `planner.workspace.catch_box` = base 축정렬 상자, sim 은 S3.5b 열린 후보의 p_c·p_stop 외접 상자 + 0.1 m, 실기 provisional | S6-B |
| K | CLIK QP 가속 제약을 YAML 선택형 `joint_cmd.accel_constraint: box\|kinematic\|dynamic` 으로 — kinematic 은 $\dot J$, dynamic 은 $M(q)(v-v_{prev})/\Delta t + h(q,v_{prev}) \le \eta_\tau\tau_{\max}$ (둘 다 $v$ 에 선형). `rtc_tsid` 일반화 → `/code-review`, 기본 `box` 면 기존 출력 비트 동일. **구현 (S6-C2, `73b8ca49`·`64dc47e1`)**: $v_{prev}$ 는 cache 가 평가된 명령 속도의 팔 성분 (h·J̇ 와 같은 상태), 행은 단위 norm·hard (못 지키면 호출 실패 → QP 비의존 abort), $\tau_{\max}$ 는 팔 device `max_torque`, $\eta_\tau$ 기본 0.8. kinematic 은 추종 과제 행만 묶어 영공간은 속도 box 뿐이다 (문서화). box 비트 일치 (golden). **출하 형태 (2026-09-23 사용자)**: p1b `dynamic` — 대기 자세 정렬 sim A/B 에서 CLIK 추종 p95 68 mm (box 624 mm), leap 은 측정 없어 `box` (L5 §4.3) | S6-C2 |
| L | `planner.wait_pose` 신설 (rad, arm 관절 순서, provisional) — p1b `[0.212, −1.376, 1.107, −1.978, −3.296, 0.121]` · leap `[0, 1.0, 0, −1.2, 0, 1.2, 0]`. IK seed 전용 (homing 은 S7.2) | 키 S6-A · 소비 S6-B |
| 3-1 | `reference.a_max` 21 / 35 채택 (provisional 해제는 S8 실측 후) | — |
| 3-2 | A-S5-12 채택 — sim 도 park | S6-A |
| 3-3 | `supervisor.track_err_abort` 는 S6-C 재관측 후 **여유 있게** (관측 피크의 2 배 이상, provisional). **적용 (2026-09-23 사용자, 같은 날 재측정으로 갱신)**: p1b **1.54 rad** — #566 lock-step sim 재측정 (dynamic, 대기 자세 정렬, 공 50회) t_c 전 최대 0.772 rad × 2. 처음 값 1.73 (0.864 × 2) 은 한 step 에 tick 이 최대 2 번 돌던 sim (#566) 에서 잰 것이라 폐기. sim 위치 서보 lag 포함 (T_arm 선행 없음), 실기 값은 S10. leap 은 측정 없어 0.3 유지. **S8-B 에서 교체 (2026-09-24)**: p1b **0.42 rad** — 1.54 는 τ 0.2 s 서보에서 잰 값이고, τ 0.05 s 서보 (D-S8-13) 의 t_c 전 피크 0.21 rad × 2 (§7.3 임계값 행). **S8-D (2026-09-26)**: leap **0.48 rad** — τ 0.05 s 서보에서 검사 구간 전체의 최대 (첫 homing 0.238 rad) × 2 (§4.4 S8-D 결과) | S6-C |
| 3-4 | A-S5-13 ok | — |
| R-1 | `mpc_main` 재사용 + **WBC ↔ 포구 switch 테스트로 실제 적용 검증** (단위: 같은 프로세스에서 두 TID·affinity·Paused/Running, launch_testing: sim `SwitchController` 왕복 + `verify_rt_runtime.sh`) | S6-A |
| R-2 | `planner.budget_s` **0.020** + 사전 필터 (IK 후보 ≤ 8) + **계획기 연산시간 실측·보고**. baseline (2026-09-23 개발 PC): 후보당 IK 6R p50 1789 · p99 2197 µs, 7R p50 2224 · p99 2415 µs → 20 점 격자 전부면 36–44 ms | S6-B · S6-C |
| R-3 | 계획기 전용 sub-model: `_base.yaml` `urdf.sub_models.<arm>_catch` + 컨트롤러 키 `planner.sub_model` | S6-B |
| R-4 | 출하 p1b YAML 은 S6-B 에서 oracle off · planner on (동시 on 은 park) | S6-B |

정한 것 (묻지 않음): `planner.slice.dt` = vision 간격 0.05 · `slice.t_lead_min` = `T_freeze` · `slice.t_max` = 지평 − `prediction.t_horizon_margin` · `budget.sigma_trk`·`clock_err` 0 provisional (순위 항) · 계획기는 mode ∈ {TRACKING, APPROACH} 에서만 탐색하고 COMMITTED 이후는 `monitorOnly`, 그 외 mode 에서는 깨어나도 게시하지 않는다 · `plan_id` 는 계획기 단조 카운터이고 RT 는 `plan_id` 변화 + token 일치 + 게시 나이 ≤ `io.t_stale` 로 새 plan 을 받는다 (L3 §5.2).

**repo drift (이 작업 범위 밖 — 별도 브랜치로 처리)**

- PR [#538](https://github.com/hyujun/rtc-framework/pull/538) 로 세 항목(FingertipSensor 주석, controllers.md DemoWbc 행, p1b `index_mcp_aa_joint` 주석)이 main 에 반영됐고 (2026-09-19, 값 변경 없음) — 닫힘
- ~~(E-9) MPC 경로의 mutex·`fprintf`~~ — 닫힘 (2026-09-19 사용자 결정: 코드를 RT 에 맞춤). 통계는 SeqLock, 실패 보고는 aux 타이머로 옮겼다. cross-mode swap 의 할당은 invariants.md 에 알려진 위반으로 기록 (해소는 별도)
- ~~(PROC-7) DemoWbc `!target_initialized_` early-return 의 Store 누락~~ — 닫힘 (2026-09-19, E-8 승인). 나머지 세 컨트롤러는 감사 결과 clean
- ~~timing CSV "7-col" 주석~~ — 닫힘 (2026-09-19, 8열로 정정)
- ~~invariants.md E-3 의 "추가" 정의~~ — 닫힘 (2026-09-19 사용자 결정: 추가도 E-3)

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
- ⚠️ **S4.4 판정 (2026-09-22): "2~4 배" 가 아니다.** 포구 자세에서 공 진행 방향으로 낼 수 있는 가속을 토크 한계로 직접 풀면 (`catch_speed_budget`, $|M\ddot q+h|\le\eta_\tau\tau_{\max}$, 접근축 유지, 회전자 관성 포함) 중앙값이 `ur5e_p1b` 43 · `iiwa7_leap` 79 m/s² 인데, 이 box 가 같은 자세에서 주는 값은 0.74 · 6.8 m/s² 다 — **약 50 배 · 10 배.** 회전자 관성을 MJCF 값의 10 배로 잡아도 21–23 · 35–36 m/s² 다. 이 box 를 쓰면 `ur5e_p1b` 는 어떤 투척 조건에서도 γ 창이 열리지 않는다 (§4.4 S4.4 결과)
  - 보수성의 출처: 표본 속도 범위의 몫은 작다 (q̇ = 0 으로 도출해도 `ur5e_p1b` 2.03 → 5.33 rad/s²). 나머지는 **최악 부호 충분조건** (전 관절이 같은 크기로 동시에 가속 — box 표현에 대해서는 정확한 조건이라 조일 수 없다) 과 **자세와 무관한 단일 상수** (`shoulder_lift` 가 표본의 93 % 에서 구속) 다
  - 표본 범위를 포구 자세로 좁혀도 움직이지 않는다: 관측된 수락 q\* 범위 (최대폭·p95 폭 모두) 에서 `ur5e_p1b` **2.03 그대로**, `iiwa7_leap` 9.26–9.91 (출하 9.20). "대기 자세 ± 작은 폭" 은 q\* 를 대표하지 못한다 (`ur5e_p1b` 어깨 ±2.6 rad · 손목 ±4.5 rad)
  - **관절 속도를 정격으로 두면 `ur5e_p1b` box 가 퇴화한다** (전 관절 3.1416 rad/s: s\* = −0.82, 채택 불가). "정격 속도로 계획" 과 이 도출법은 양립하지 않는다. `--weights tau_max` 는 손목을 0.39 rad/s² 로 **낮춘다**
  - 상수 box 안에서의 최선 (관절별 가중을 방향 가속 중앙값에 대해 최적화, 회전자 관성 포함): `ur5e_p1b` 1.95 → 5.1 m/s² (p05 0.6) · `iiwa7_leap` 8.9 → 15.9 (p05 4.5). `iiwa7_leap` 은 경계선, `ur5e_p1b` 는 부족하다
  - ⚠️ **반대 방향의 결함: 이 도구의 Pinocchio $M$ 에는 회전자 반사관성이 없다** (URDF 에 없다). 그 항만 보면 box 는 낙관적이다 — MJCF `armature` (UR5e 0.1, iiwa7 0.15–0.25 kg·m²) 를 더해 다시 도출해야 한다
  - **D-16 개정 (2026-09-22 확정, §7.3 결정 B — S3.5b 는 두 층 병기로 먼저 쓴다: `catch_gate_map` 의 `box`·`torque` 층)**: 계획기 (L3 §4.3 도달시간·§4.8 rollout) 는 후보마다 토크 한계를 직접 검사하는 **자세·방향 의존 한계**를 쓰고, CLIK 에 넘기는 box 는 가중 벡터 (`qdd_max` 는 이미 벡터다) 로 둔다. "계획기와 CLIK 이 같은 한계를 쓴다" 는 아래 원칙과 부딪치므로 설계는 S6 에서 다시 정한다. 회전자 관성은 `derive_accel_limits` 에 인자로 넣는다
- η_τ < 1 은 접촉 충격(L7 충격량 예산)과 모델 오차를 위한 여유다. **0.8 로 확정** (2026-09-20 사용자 결정, S2.5 결과는 §4.4 S2)
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
      xyz: [0.0, 0.0, 0.0]       # m, 부모 frame 기준 — 포켓 중심 (스키마의 자리값. 출하값은 아래 "초기 제안값 산출")
      rpy: [0.0, 0.0, 0.0]       # rad, 부모 frame 기준 — 결과 frame 의 +z 가 손바닥 바깥 법선
      provisional: true          # 사용자 확인 전
```

포구 컨트롤러 YAML 은 frame 이름만 참조한다 (`catch_frame: catch_frame`). 접근축은 규약상 이 frame 의 +z 다.

**전달 경로 (S2.3a).** CM 파서가 `list_parameters({"urdf.extra_frames"})` 로 이름을 모으고 (`ParseSubModels` 와 같은 방식) → `rtc_urdf_bridge::ModelConfig` 의 새 필드 → yaml-cpp `LoadModelConfig` 경로도 같은 키를 읽거나 명시적으로 거부 → `PinocchioModelBuilder` 가 `BuildFullModel()` 직후 full 모델에 frame 을 추가. sub·tree·actuated 모델은 모두 full 모델에서 `buildReducedModel` 로 만들어지므로 frame 을 상속한다. 부모 관절이 잠긴 모델에서는 frame 이 조상 관절에 붙는다. 네 모델 모두에서 frame 존재와 위치를 검증한다 (S2 게이트). 값은 모델 빌드 시 읽히므로 바꾸면 컨트롤러를 다시 configure 해야 한다.

**초기 제안값 산출.**

- 축 (S2.3a): FK 로 도출한 후보 — p1b `l_palm_link` +z (rpy 0), iiwa7_leap `palm_lower` −z (x 축 π 회전으로 +z 로 뒤집음)
- 위치 (S2.3b, 2026-09-21 확정): **S4.5 의 실측 포구점** (L6 §4.5 표) 을 부모 frame 으로 옮긴 값을 쓴다 — p1b `[0.015, 0.145, 0.052]`, iiwa7_leap `[-0.035, -0.015, -0.069]`
- 제안값은 근거(자세·계산식)와 함께 PR 에 적고, 사용자가 sim 에서 확인한 뒤 `provisional: false` 로 갱신한다 — **완료 2026-09-21**: 두 로봇 모두 렌더로 확인받아 `false` 로 전환했다. 그 전환이 `test_catch_frame_models` 의 `EXPECT_TRUE(provisional)` tripwire 를 뒤집으므로 PROC-6 에 따라 근거를 달고 assertion 을 `EXPECT_FALSE` 로 바꿨다 (이제 "확인된 상태가 출하된다" 를 고정한다)
- 검증기는 `provisional: true` 인 catch frame 으로 실기 arm 을 막는다 (D-12 와 같은 규칙)

**"preshape 손끝 중심" 은 기각했다 (2026-09-21).** v0.x 까지 이 절은 위치 제안값을 손 preshape 자세의 손가락 끝 위치 중심으로 산출한다고 적었다. S4.5 가 두 손의 포구점을 실제로 재고 나니 그 정의를 쓸 이유가 없다.

- **게이트가 실측점 중심으로 정의돼 있다.** $r_{cap}$ (L3 §4.6 게이트 우변) 은 그 포구점을 중심으로 잰 **측면** 허용량이다. 원점을 다른 곳에 두면 게이트가 두 점 사이 거리만큼 통째로 어긋난다 — p1b 20.9 mm, iiwa7_leap 81.7 mm (아래 측정).
- **$t_c$ 의 의미와 충돌한다.** $t_{cmd} = t_c - T_{close,e2e}$ (L6 §4.3) 이므로 $t_c$ 는 손이 이미 $\eta$ 까지 닫힌 시각이다. 그때 공이 있는 자리가 원점이어야 한다. 손끝 중심은 포켓 **입구**이고 둘의 차이는 접근축 방향으로 대략 $d_{eff}$ 다 (iiwa7_leap: 포구점 catch z 0.069 + $d_{eff}$ 0.080 = 0.149 ≈ 손끝 평면 0.146). 입구를 원점으로 삼으면 γ 창에서 $d_{eff}$ 를 두 번 세게 된다.
- **정의 자체가 값을 못 정한다.** "손가락 끝" 에 해당하는 body 가 한 로봇 안에 여럿이다. iiwa7_leap 의 `*_tip_head` 중심은 catch z 0.146, `fingertip`/`*_tip_link` 중심은 0.0969 — 같은 자세에서 **51.2 mm** 차이다. 실측 포구점에는 이 자유도가 없다.
- 손끝 중심은 버리지 않고 **교차 확인**으로 쓴다: 실측 포구점과 같은 쪽(+z)에 있어야 하고, 접근축 차이가 $d_{eff}$ 자릿수여야 한다. 둘 다 성립한다.

**frame 규약 검증 (S2.3b 게이트, 2026-09-21).** 실측은 MuJoCo 의 palm **body** frame 에서 쟀고 YAML 은 URDF 의 parent **link** frame 을 쓴다. 두 관례는 이 저장소에서 실제로 갈리므로 (§11 의 `upper_arm_link`; 이번 측정에서도 `wrist_3_link` 의 body 원점은 올바른 변환에서도 **100 mm** 어긋난다) 값을 옮기기 전에 같은 q 에서 두 엔진 FK 를 대조했다. 무작위 팔 자세 16 세트, 잔차 분해는 $L = T_{mj}T_{pin}^{-1}$ (palm 이 일치할 때만 상수) 와 $R = T_{pin}^{-1}T_{mj}$ (base 가 일치할 때만 상수).

| 로봇 | palm 방향 잔차 | catch frame 원점 왕복 오차 | 판정 |
|---|---|---|---|
| `iiwa7_leap` | 7.7e-5 ° | **1e-7 m** | PASS |
| `ur5e_p1b` | 3.4e-6 ° | **1.4 mm** (평균 1.15 mm) | PASS — 잔차는 $\varepsilon_{model,p1b}$ (§11), frame 불일치 아님 |

- `l_palm_link`·`tool0` 는 두 모델에서 일치하므로 (위 표) catch frame 의 앵커로 유효하다. 이름이 같은 다른 link 는 그렇지 않다 — 앵커는 반드시 실제로 쓰는 frame 으로 확인한다.
- `ur5e_p1b` 의 1.4 mm 는 §11 의 8.3e-4 m 와 같은 항(MJCF↔URDF 치수 차이)을 다른 지점·다른 통계로 본 값이다. sim 으로 줄일 수 없으므로 **다른 오차 항과 합치지 않는다** (§11 사용자 결정 2026-09-20).
- 재현: `verify_catch_frame.py` (도구는 private plan 쪽에 있다 — repo 에 커밋하지 않는다). 방법은 위 두 잔차 분해와 왕복 대조이고, 그것이 이 문서가 갖는 SSoT 다.

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
  - w₆ 는 IK 가 남긴 roll 에 의존한다. roll 은 대기 자세 seed + **w₅ 상승** (D-25) 으로 결정적으로 정해지므로 (C-4) 지도와 런타임의 w₆ 도 같은 값이 된다
  - **상승 대상은 정의와 무관하게 항상 w₅ 다** (D-25). 그래서 q\* 가 `definition` 에 의존하지 않고, 위 두 분포가 **같은 자세에서 잰 두 값**이라 비교가 성립한다. `arm_6row` 로 판정한다는 것은 직접 올리지 않은 값으로 게이트한다는 뜻이다
  - 어느 정의로 판정할지는 S3.5a/b 지도에서 두 값의 분포와 사용자의 sim 자세 확인을 보고 정한다
- 단위가 섞여 있어 (m 와 rad) w 의 크기는 정의에 따라 달라진다. **threshold 0.1 은 위 정의에 대한 값**이고, 정의를 바꾸면 다시 맞춰야 한다

**포구 자세의 여유 자유도.** 6축 UR5e 에서 5행 과제는 1 자유도(손바닥 법선 둘레 roll)와 IK 해 가지가 남아 w 가 그 선택에 따라 달라진다. 런타임과 지도가 같은 해를 쓰도록 **대기 자세(wait_pose)에서 시작하는 같은 IK** 로 정하고, 남은 roll 은 **영공간 log w₅ 상승으로 쓴다** (D-25, 2026-09-20 번복 — v0.5 까지는 v1 범위 밖이라고 적었다). 상승은 seed 가 놓인 해 가지 안의 **국소 최대**이지 전역 roll 탐색이 아니므로, 결정성은 여전히 같은 seed·같은 키에 달려 있다. `planner.ik.k_manip` = 0 이면 번복 전 동작 그대로다.

**frame 규약 (함정 주의).**

- "arm base frame" 은 ur5e_p1b `base` (URDF `base`), iiwa7_leap `link_0` 다 — 로봇 config 의 `urdf.sub_models.<arm>.root_link` 와 같은 값이고, 컨트롤러 config 의 CLIK `base_frame` (`controllers/demo_wbc_controller.yaml`·`controllers/mpc/*.yaml`) 이 이것을 이름으로 참조한다. **로봇 config 최상단에는 `base_frame` 키가 없다** — v0.x 까지 여기에 "로봇 config 의 CLIK `base_frame`" 이라고 적혀 있었고, 그 키를 찾으러 가면 없다 (2026-09-21 정정)
- ur5e_p1b 에서 URDF `base` 와 `base_link` 는 z 축 둘레 180° 차이다. `base_link` 로 두면 +x 가 반대가 되어 공이 등 뒤에서 날아온다 — 그래도 그럴듯한 결과가 나오므로 조용히 틀린다. sim 에서는 MJCF 의 로봇 body 가 world 에 180° z 회전으로 놓여 있다
- 발사 높이 z 는 world 기준이고 거리는 base 기준이다. world ↔ base 변환은 가정하지 않고, **같은 q 에서 MuJoCo FK 와 Pinocchio FK 를 대조**해 S3.2 에서 확정한다 (S3.5a 의 선행). 확정 전까지 아래 키는 이름에 프레임을 붙여 섞이지 않게 한다

**world ↔ base 대조 실측 (S3.2, 2026-09-20).** 두 엔진(MuJoCo `mj_forward` · Pinocchio `forwardKinematics`)을 **씬 파일** 위에서 같은 q 로 돌려 비교했다. 무작위 q 8 세트 (trial 0 은 q=0).

| 로봇 | base frame | **world_T_base (확정)** | 축선 잔차 | 게이트 < 1e-6 m |
|---|---|---|---|---|
| `iiwa7_leap` | `link_0` | **항등** (p = 0, R = I) | **4.5e-16 m** | **PASS** |
| `ur5e_p1b` | `base` (URDF) | **항등** (p = 0, R = I) — 아래 정정 | **1.46e-3 m** | **FAIL** — 아래 |

> **정정 (2026-09-21, 사용자 컨펌).** 이 행은 `Rz(180°), p = 0` 으로 적혀 있었고 **틀렸다**. 180° 가 두 번 세어졌다.
>
> 원인: 위 측정은 MuJoCo world 를 Pinocchio `Data::oMi` 와 대조했는데 `oMi` 는 **URDF 모델 root** 기준이고, 이 URDF 의 모델 root (`universe`/`world`) 는 **`base_link`** 다 (회전 0°, 직접 측정). `base` = root·Rz(180°) 이므로 그 측정의 Rz(180°) 는 world→**`base_link`** 이고, 표가 이름 붙인 world→`base` 는 그 합성 결과 **항등**이다.
>
> 가설 검정을 두 모델이 실제로 일치하는 frame (`tool0`·`l_palm_link`) 으로 다시 돌린 결과:
>
> | probe | root = `base` | root = `base_link` |
> |---|---|---|
> | `tool0` · `l_palm_link` | 항등 → **1.46e-3 m** · Rz(180°) → 1.798 m | 항등 → 1.798 m · Rz(180°) → **1.46e-3 m** |
>
> 왜 이 probe 여야 하는가: `wrist_3_link` 는 **올바른** 변환에서도 body 원점이 100 mm 어긋난다 (MJCF body frame ≠ URDF link frame, 아래 `upper_arm_link` 와 같은 축). 이름이 같은 link 를 아무거나 앵커로 쓰면 물리가 아니라 파일 관례를 재게 된다 — 앵커는 **실제로 쓰는 frame** 으로 확인한다.
>
> 걸려 있던 것: 지도 도구가 이 표를 그대로 써서 world 발사점을 `base` 로 옮기면 x·y 가 뒤집혀 **아래 본문이 예고한 "공이 등 뒤에서 날아온다" 가 그대로 발생한다.** 실제로 그 함정을 경고하는 문단 바로 위에서 표가 그 함정에 걸려 있었다. 그래서 S3.5a 도구는 변환을 **인자로만** 받고 어떤 로봇 값도 박지 않는다.
>
> $\varepsilon_{model,p1b}$ 는 영향받지 않는다 — 회전 라벨과 무관한 치수 차이 항이다. 잔차 수치는 8.3e-4 m (관절 축선 기준) → **1.46e-3 m** (`l_palm_link`·`tool0` 원점, 무작위 자세 16 세트의 최대) 로 갱신했다. 같은 항을 사슬의 더 아래 지점에서 본 값이므로 커진 것이 정상이고, 아래 "사용자 결정" 의 0.8 mm 를 **1.5 mm 로 읽는다**.

**⚠️ 위 표만으로는 좌표를 넘길 수 없다 — 변환이 두 개다 (2026-09-21).** 위 표는 world ↔ **계획기가 이름 붙인 base frame** (`sub_models.<arm>.root_link`) 의 변환이다. 그런데 `CatchPoseIk::Solve` 와 `catch_pose_ik_batch` 는 `p_c`·`v` 를 **모델 world** (Pinocchio universe = URDF 모델 root) 로 받고, 이 둘이 같은 frame이 아니다.

| 로봇 | MuJoCo world → `sub_models` base frame | MuJoCo world → **모델 root** (판정기 입력) |
|---|---|---|
| `iiwa7_leap` | 항등 (`link_0`) | **항등** — 모델 root 가 `link_0` 이다 |
| `ur5e_p1b` | **항등** (`base`) | **Rz(180°)** — 모델 root 는 `base_link` 이고 `base` = root·Rz(180°) 다 |

즉 `ur5e_p1b` 에서 **world 좌표를 판정기에 그대로 넣으면 틀린다.** "world_T_base 는 항등이다" 만 읽고 넘기면 x·y 가 뒤집혀, 바로 아래 본문이 경고하는 "공이 등 뒤에서 날아온다" 가 그대로 발생한다 — 라벨 오류(위)를 고친 뒤에도 남아 있는 두 번째 함정이고, 원인은 같다: 이 URDF 에는 원점이 같고 z 둘레 180° 다른 frame 이 둘 있다.

그래서 지도 도구는 두 변환을 **분리해서** 받는다: `base_T_world` 는 인자 (`--world-yaw-deg`·`--world-translation-m`), `model_world_T_base` 는 `--arm-base-frame` 으로 지정한 frame 의 배치를 **모델에서 읽어** 합성한다 (그 frame 이 모델 root 에 대해 강체가 아니면 거부한다). 어느 쪽도 모듈 코드에 박지 않고, 180° 는 테스트가 고정한다.

**영구 게이트로 만들지 않는다 (2026-09-20 사용자 결정, 권장안).** 두 로봇의 값은 위 표로 닫혔고, 게이트로 두려면 `ur5e_p1b` 에 0.83 mm 를 예외 허용치로 박아 알려진 실패를 정상으로 고정해야 하며 MuJoCo 를 `integrated_bringup` 의 test dep 으로 새로 넣어야 한다. 대신 **새 로봇 프로파일이 추가되거나 `hand_description` MJCF 가 고쳐질 때** 축선 대조를 한 번 다시 돌린다 — 재현 경로는 `rtc_tools compare_mjcf_urdf` 에 관절 축선 FK 대조를 얹는 후속 작업 (pinocchio·mujoco python 이 이미 그 도구의 의존이다, P5). 이번 측정의 C++ 프로브는 세션 scratch 였고 보존하지 않는다 — 방법(축선 비교·가설 검정)은 아래 본문이 갖는다.

- **비교 대상은 body/link 프레임 원점이 아니라 관절 축선이다.** UR5e 는 MJCF body 프레임과 URDF link 프레임의 관례가 달라 (`upper_arm_link` 이 정확히 shoulder_offset 0.138 m 만큼 어긋난다) 이름이 같은 body↔link 를 원점으로 비교하면 **물리가 아니라 파일 관례를 재게 된다**. 저장소의 기존 `compare_mjcf_urdf` 게이트가 축선을 쓰는 이유와 같다
- ⚠️ **단일 링크의 "implied transform 이 상수" 는 증거가 못 된다.** `shoulder_link` 의 implied transform 은 8 세트에서 1e-17 로 상수지만, 두 모델의 그 프레임 차이가 **pan 축(z) 둘레 회전 + z 방향 이동**이면 q 와 무관하게 상수로 나온다 — 그리고 실제 차이가 정확히 그 형태였다. 그래서 `world_T_base = I` 와 `Rz(180°)` 가 이 링크로는 구별되지 않는다. 가설 검정(축선)으로 갈랐다: `yaw 0°` → 1.66 m, `yaw 180°` → 8.3e-4 m
- ⚠️ **`ur5e_p1b` 의 FAIL 은 프레임 미확정이 아니라 두 모델이 다르기 때문이다.** 축 방향은 8.5e-7 ° 로 완벽하고 (회전은 일치), 잔차는 **순수 치수 차이**다. shoulder_pan·shoulder_lift·elbow 는 **정확히 0** (1e-16) 이고 wrist 부터 벌어진다:
  - shoulder 높이 — URDF `0.1625` vs MJCF `0.163` → **0.5 mm**
  - wrist_1 — URDF `0.3922` vs MJCF `0.392` → **0.2 mm**
  - wrist_2 누적 → 0.71 mm (최악 8.3e-4 m)
  - 뿌리는 `ur5e_p1b` 의 MJCF 가 `hand_description` 패키지에 있는 **#392 수정 밖의 Menagerie 사본**이라는 것이다 (관성이 어긋난다는 것은 알려져 있었고, **운동학도 어긋난다는 것이 여기서 처음 측정됐다**). 사용자 결정 (2026-08-29) 으로 `hand_description` 은 고치지 않으므로 **이 0.8 mm 는 sim 의 바닥값**이고 sim 안에서 줄일 수 없다
  - ⇒ **`ur5e_p1b` 의 sim 포구점은 계통적으로 편향된다.** 축선으로 0.8 mm, catch frame 부모 (`l_palm_link`) 원점으로는 **1.46e-3 m** (2026-09-21, 무작위 자세 16 세트 최대). 포구점에 걸리는 값은 후자다. 공 반지름 33.5 mm 대비 작지만 sim 으로는 측정해 없앨 수 없는 항이다
  - **사용자 결정 (2026-09-20): `hand_description` 을 고치지 않는다.** 2026-08-29 결정(별개 패키지)을 유지하고, 이 0.8 mm 를 **sim 의 바닥값**으로 받는다. 대신 다음을 지킨다:
    - S3.5a/b 지도와 S8 오차 예산에서 `ε_model,p1b` = **1.5 mm** (catch frame 지점의 값, 위) 를 **분리된 계통 항**으로 센다 — 다른 항과 합쳐 평균내면 sim 을 아무리 돌려도 안 줄어드는 항이 줄어드는 것처럼 보인다
    - **sim 실측으로 이 항을 검증하려 하지 않는다.** sim 이 곧 편향의 출처이므로 자기 자신을 오라클로 쓰는 셈이다 (실기 S10 에서만 갈린다)
    - `iiwa7_leap` 에는 이 항이 없다 (항등·4.5e-16 m). 두 로봇의 sim 포구 정확도를 비교할 때 **이 차이를 로봇 차이로 읽지 않는다**
- **재현 방법**: 두 엔진을 직접 링크한 프로그램으로 관절 축선(`mjData::xanchor`/`xaxis` vs `Data::oMi`)을 비교한다. pinocchio 4.x 는 `-DNDEBUG` 와 `BOOST_MPL_LIMIT_{LIST,VECTOR}_SIZE=30` 없이는 컴파일되지 않는다 (repo 안에서는 `pinocchio::pinocchio` 타깃이 넣어 준다)

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

> ⚠️ **2026-09-24 확인 (S8 준비 C-3)**: 위 `sim.throw_region` 블록은 제안으로 남았고 **YAML 키로 만들어지지 않았다** — 지도 도구 (`catch_gate_map`) 는 CLI 인자를 쓰고, 시행 러너는 동결 분포를 인자 (`catching_sim_trials --dist`, S8-A) 로 받아 `catchability_map.Throw`/`generate_throw_grid`/`throw_to_launch_request` 로 표본을 뽑는다 (D-S8-2, §4.4 S8).

**지도 도구의 구성 (S3.5a, 2026-09-21 착수).** 판정은 런타임과 같은 `CatchPoseIk::Solve` 여야 하고 (위 S1.9), 격자·비행·집계는 python 관행이다. 그 둘을 잇는 방식을 **C++ 배치 실행파일 + python 오케스트레이터**로 정했다. pybind 는 기각했다 — 저장소에 선례가 없고, venv 의 `FindPython` 함정 (AGENTS.md §9.2) 과 정면으로 부딪치며, pinocchio 4.x 는 전용 컴파일 플래그를 요구한다. python 으로 판정을 다시 구현하는 것은 "같은 함수" 를 어긴다. 선례는 `rtc_math/se3_error_compare` (Eigen-only 오프라인 도구 + python 짝) 다.

- **C++ 옵션 파서** `rtc::catching::ParseCatchPoseIkParams` — `planner.ik.*`·`planner.catchability.*` → `CatchPoseIkOptions`. S1.9 가 "No YAML parser yet (Q4)" 로 남긴 자리이고, S6.2 런타임이 이것을 그대로 쓴다 ("같은 YAML 키" 의 실체). `planner.ik` 아래 미지 키는 **거부**한다 (이 절을 통째로 소유하므로). `alpha_max` 는 L3 §6 이 TBD, 구조체 기본값은 0.26 rad 인 불일치를 이 파서가 드러냈고, **값은 코드 쪽으로 맞췄다** (L3 §6 = `0.26 (provisional)`; 열려 있는 것은 값이 아니라 닫는 근거인 θ 분포다). 활성 `manipulability_min` 이 TBD 면 비유한으로 남겨 `kOptionsInvalid` 로 **fail-closed** 한다 (w₅ 의 0.1 은 차원이 다른 w₆ 게이트의 보수적 대체값이 될 수 없다, C-3).
- **배치 실행파일** `catch_pose_ik_batch` (`rtc_controllers`, ARCH-7 은 design-principles.md §"ARCH-7 의 범위" 의 **오프라인 검사 도구** 예외 — bringup chain 에 없고 로봇을 모른다). 입력은 ModelConfig YAML·sub-model 이름·catch frame 이름·옵션 YAML·seed CSV·후보 CSV, 출력은 후보별 `reason`·`q*`·`w5`·`w6`·`iterations`·`sigma_min`·`qp_*` CSV. 두 계약을 테스트가 고정한다 — **열이 solver 의 double 을 bit-exact 로 싣는다** (G3-I 비교가 반올림된 열로는 성립하지 않는다) 와 **후보 순서를 바꿔도 판정이 같다** (python 이 격자를 샤딩·재개한다). `--dump-frame` 은 생산 빌더로 catch frame 배치를 선언값과 대조한다 (S2.3b 검사).
- **python 순수 모듈** `rtc_tools.analysis.catchability_map` — 격자, 항력 비행 (고정 스텝 RK4, 적분 차수까지 테스트), base↔world 변환, provenance. 변환은 **항상 인자**이고 로봇 값을 박지 않는다 (위 ⚠️).
- **함정 두 개.** (1) 출하 `urdf.sub_models.<arm>` 은 flange (`tool0`/`ee_link`) 에서 끝나 **catch frame 이 그 sub-model 에 없다** — 지도는 arm root → catch frame 부모 link 까지의 sub-model 을 따로 선언해 쓴다 (손 관절은 `buildReducedModel` 이 잠그고, 손바닥은 루프 상류라 catch frame FK·Jacobian 은 정확하다). S6.2 런타임도 같은 모델이 필요하므로 그때 출하 config 에 들어가야 한다. (2) `LoadModelConfig` 의 스키마는 출하 robot config 와 **다르다** (`urdf_path`, `sub_models` 가 map 이 아니라 sequence) — 번역은 python 쪽이 한다.
- **항력.** sim 의 항력은 무차원 `Cd` **preset (constexpr, YAML 미노출)** 이고 문서의 스칼라 $k$ [1/m] 는 `sim.ball.drag_k` = TBD 이며 §7 이 "preset 에서 환산해 쓸 수 없다" 고 못박았다. 그래서 지도는 **sim 의 힘 법칙 자체**를 적분하고 (ω = 0 이라 Magnus 소멸), ρ·$C_d$ 는 기본값 없는 필수 인자로 받아 출처 file:line 을 provenance 에 남긴다. 반경·질량은 YAML 에서 읽는다. 환산값 $k = \rho C_d A/2m$ = **0.0205 1/m** 은 참고로만 기록한다 (문서 대표값 0.0229 와 12 % 차이).

**지도 결과 (S3.5a, 2026-09-21) `[PASS(provisional)]`.** 격자: 거리 4 m, 방위 6 × 60°, world z {1.5, 1.8, 2.0}, 방향 편차 {−10°, 0, +10°}, 속력 5.00–7.00 m/s (0.25 간격 9), 앙각 32–56° (4° 간격 7) = **투척 3402 개**. 비행은 sim 힘 법칙 RK4 (2 ms), 포구 후보는 T_f ∈ [1.0, 2.4] s 를 25 ms 로 훑고 도달권·최소 높이로 걸렀다. 판정은 `catch_pose_ik_batch`, seed 는 Pass A 에서 고른 상위 2 개.

> **정정 (2026-09-21, 코드리뷰 #556).** 이 절의 투척 수는 처음에 **두 seed 의 합집합**으로 적혔다 (`ur5e_p1b` 1426 · `iiwa7_leap` 1503) — 도구의 `summarize_throws` 가 seed 를 구분하지 않았다. 로봇은 **한 자세**에서 기다리므로 아래 값은 최선 seed 하나의 것이다 (차이 0.3–0.6 %). 더 컸던 것은 제안 상자의 수락률이다: 후보가 0 개인 투척이 `throw_summary.csv` 에서 빠져 분모가 작았고 (81.5 / 97.2 %), 상자 안 **전체 격자 투척** 기준으로는 70.2 / 76.9 % 다. 두 결함 모두 도구에서 고쳤다.

| | `ur5e_p1b` | `iiwa7_leap` |
|---|---|---|
| 수락 투척 (**단일 대기 자세**, 최선 seed) | **1418 / 3402 (41.7 %)** | **1499 / 3402 (44.1 %)** — 문턱 0.1 일 때. 아래에서 권한 로봇별 문턱이 0.174 로 확정된 뒤 출하 config 로는 **1158 / 3402** 이고, 이 열의 나머지 값도 문턱 0.1 기준이다 |
| 판정한 후보 / 수락 (그 seed) | 6888 / 4264 | 5298 / 4019 |
| 탈락 사유 (그 seed) | `below_manip_min` 2241, `not_converged` 383 | `not_converged` 683, `below_manip_min` 596 |
| w₅ (수락) | min 0.100 · p05 0.108 · **median 0.147** · max 0.208 | min 0.100 · p05 0.112 · **median 0.226** · max 0.273 |
| w₆ (수락) | min 0.0035 · median 0.047 · max 0.111 | min 0.0013 · median 0.109 · max 0.144 |
| θ (수락) | max 1.22e-2 rad = **0.047 × α_max** | max 2.99e-2 rad = **0.115 × α_max** |
| ε = 1.5 mm 경계 뒤집힘 (두 seed 의 수락 후보 합산) | **81 / 8446 (0.96 %)** | 23 / 8030 (0.29 %) |
| wait_pose 차점 seed 격차 | 1.55 %p | 0.25 %p |

- **방위는 구속하지 않는다.** 두 팔 다 base z 둘레 대칭이라 전 방위가 같은 결과를 낸다 (Pass A 에서 12 방위 전부 확인). 구속하는 것은 **속력 × 앙각의 결합**이고, 수락 집합은 그 평면에서 **능선**이다 — 앙각이 낮거나 속력이 높으면 궤적이 도달권을 비껴간다. 그래서 축정렬 상자로는 표현이 안 된다: 수락 집합의 외접 상자는 격자 전체이고 그 안의 수락률은 41.7 / 44.1 % 에 그친다 (도구가 `box_accepted_fraction` 으로 그 사실을 같이 낸다).
- **`sim.throw_region` 제안 (provisional)**: 거리 4 m, 방위 전 범위, world z 1.5–2.0 m, 방향 편차 ±10°, **앙각 44–56°**, **속력 5.0–6.5 m/s**. 그 상자 안 수락률은 `ur5e_p1b` **70.2 %** · `iiwa7_leap` **76.9 %** (상자 안 격자 투척 1512 개 기준) 이고, 수락 투척의 75 / 78 % 를 덮는다.
- **α_max 는 구속하지 않는다** — θ 가 콘의 12 % 를 넘은 적이 없다. L3 §6 의 provisional 0.26 rad 을 그대로 둔다 (닫는 근거는 이 분포다).
- **`arm_5row` 0.1 은 `ur5e_p1b` 에서 한계선이다.** 수락 w₅ 의 median 이 0.147, p05 가 0.108 로 문턱에 붙어 있다. `iiwa7_leap` 은 median 0.226 으로 두 배 여유다. 같은 숫자가 두 팔에서 전혀 다른 뜻이 되므로 **문턱을 로봇별로 두는 것을 권한다** (§4.4 후속).
- **`arm_6row` 로 바꾸려면 문턱이 30배 작아야 한다.** 같은 수락 집합의 w₆ 하한이 0.0035 (`ur5e_p1b`) / 0.0013 (`iiwa7_leap`) 다. 6행 정의는 6축 팔의 손목 특이점에서 0 으로 내려가는데 그 자세가 포구에는 멀쩡하므로, **정의는 `arm_5row` 를 유지하고** `arm_6row` 는 기록만 한다 (C-3).
- **`planner.ik` 제안 (provisional)**: `k_manip` = **0.5** (0 이 아니어야 한다 — 아래), `max_iter` = **40**. 나머지 (`sigma0`·`lambda_max`·`dq_step_max`·`manip_grad_tol`) 는 L3 §6 기본값에서 손댈 근거가 없었다.
- ⚠️ **출하 기본값 `k_manip = 0` 으로는 아무것도 통과하지 못한다.** 그럴듯한 대기 자세에서 w₅ 가 0.06–0.09 로 문턱 아래에 머문다. D-25 의 영공간 log w₅ 상승을 켜야 (k_manip 0.5) 비로소 0.10–0.21 로 올라온다. 즉 **게이트가 통과 가능한 것은 상승 때문이고**, "0.1 문턱 + 상승 꺼짐" 조합은 출하 config 에 남겨 두면 안 된다.
- **`wait_pose` 제안 (provisional)**: `ur5e_p1b` `[0, −1.4, 0.9, −1.9, −1.5708, 0]`, `iiwa7_leap` `[0, 1.0, 0, −1.2, 0, 1.2, 0]`. **민감도는 낮다** — Pass A 에서 `iiwa7_leap` 은 5 개 seed 가 커버리지 27/56 으로 전부 동일했고 (mean log w₅ 로만 갈렸다), `ur5e_p1b` 도 차점과 1.55 %p 차이다. 7-DoF 여유와 영공간 상승이 seed 의 영향을 흡수한다. 고정점 확인은 이 낮은 민감도 때문에 1 회로 끝났다.
- **$\varepsilon_{model,p1b}$ 는 분리해 센다.** 1.5 mm 섭동으로 수락 후보의 **0.96 %** 가 뒤집힌다 (`below_manip_min` 99, `not_converged` 40). `iiwa7_leap` 의 0.29 % 는 이 항이 없는 로봇의 수치적 한계선일 뿐이므로 **두 값을 로봇 차이로 읽지 않는다** (§11 사용자 결정 2026-09-20).
- ⚠️ **이것은 kinematic 지도다 — 손은 이 공을 못 잡는다.** 수락된 후보의 포구 시점 속력은 두 팔 모두 **6.5–8.3 m/s** (median 7.35) 인데, $\Vert v\Vert_{\max}$ 공식값은 `iiwa7_leap` 2.26 · `ur5e_p1b` 1.84 m/s 다 — **3.5–4.5 배**다. 게다가 P1b 의 fly-in 실측은 0.25 m/s 에서 130 중 9, 0.5 m/s 3, ≥1 m/s 0 이다 (L6 §4.5). 팔이 갈 수 있다는 것과 손이 닫힌다는 것은 다른 이야기이고, **S4.4 는 거리(4 m)나 목표 속력을 되돌려야 한다** — 그래서 도구는 거리를 인자로 받는다. 이 지도는 그 협상의 입력이지 결론이 아니다.
- `not_converged` 가 758 / 1366 인 것은 max_iter 40 의 부족과 애초에 도달 불가한 후보가 섞여 있어 갈리지 않는다 — max_iter sweep 은 후속으로 둔다.

**속력 격차를 어떻게 닫는가 `[사용자 결정 2026-09-21]`.** 투척을 좁히는 대신 **팔 궤적으로 공과 손의 상대속도를 줄이는** 방향으로 계획기를 갱신한다.

그 메커니즘은 새로 만들 것이 아니라 **이미 이 설계다** — L3 §4.5 의 γ 창이 손 폐쇄 하한을 절대속력이 아니라 **상대속도 $(1-\gamma)\Vert v\Vert$** 로 쓰고, `SoftCatchTranslation` (L4) 이 그 프로파일을 실행한다. 그러므로 지도가 공급할 것은 메커니즘이 아니라 **요구되는 γ 값**이다.

$$\gamma_{\min}=1-\frac{d_{eff}}{\Vert v\Vert\,T_{close,tot}},\qquad \gamma_{\max}=\frac{\min(v_{dir,\max},\ \eta_vv_{\max})}{\Vert v\Vert}$$

| 로봇 | $d_{eff}$ / $T_{close,tot}$ | $\Vert v\Vert$ 6.5 → 8.3 m/s 에서 $\gamma_{\min}$ | 팔이 내야 하는 속력 $\gamma_{\min}\Vert v\Vert$ |
|---|---|---|---|
| `ur5e_p1b` | 0.095 m / 0.2815 s | 0.948 → 0.959 | **6.2 → 8.0 m/s** |
| `iiwa7_leap` | 0.080 m / 0.1047 s | 0.882 → 0.908 | **5.7 → 7.5 m/s** |

($T_{close,tot}=T_{close,e2e}+h/2$ — 2026-09-22 정정. 처음에는 $h$ 를 더한 0.2825 / 0.1057 s 로 적혀 있었다, L3 §4.11.)

> **정정 (2026-09-22, S4.4).** 이 자리에는 "출하 $\eta_vv_{\max}$ 는 두 로봇 모두 1.5 m/s 이고 창이 다섯 배 가까이 비어 있다, 선행 조건은 `reference.v_max` 실측이다" 라고 적혀 있었다. 그 1.5 m/s 는 $\Vert v\Vert_{\max}$ 의 예시에서 역산한 값이지 어떤 한계도 아니었고, `reference.v_max` 는 URDF 에도 제조사 자료에도 없는 양이다 (URDF 는 관절 정격 [rad/s] 만 준다). 사용자 결정 (2026-09-21) 으로 팔 속도의 출처는 **관절 정격**이며, 그 아래에서 구속하는 것은 자세마다 다른 $v_{dir,\max}$ 다. 후보별 $v_{dir,\max}$ 계산도 S3.5b 가 아니라 **S4.4 가 냈다** (§4.4).

**S4.4 가 잰 것.** 수락 후보마다 $v_{dir,\max}$ (LP, 접근축 유지, 정격 관절 속도 × η_v 0.9) 와 토크 한계 방향 가속을 풀고, 창이 열리는 조건 $\Vert v\Vert+\text{margin}\le v_{arm}+v_{rel}$ 을 **낙차 × 상대속도 허용량**의 표로 냈다 ($v_{rel}$ = 손이 흡수하는 상대속도; 공식 $d_{eff}/T_{close,tot}$ = 0.34 / 0.76 m/s, 시각 발동 fly-in 실측은 두 손 모두 약 **1.0 m/s** — L6 §4.5). 셀은 kinematic 수락 후보 중 창이 열리는 비율, $v_{rel}$ = 1.0 m/s, 토크 한계 가속·stroke·시간 포함:

| 낙차 $\Delta z=z_c-z_0$ [m] | `ur5e_p1b` | `iiwa7_leap` | $T_f\ge0.5$ s 만 (`ur5e_p1b` / `iiwa7_leap`) |
|---|---|---|---|
| ≤ −0.5 (**현 sim 씬의 전부**) | **0 %** | **0 %** | 0 / 0 |
| −0.5 … −0.25 | 0 % | 0 % | 0 / 0 |
| −0.25 … 0 | 2 % | 13 % | 0 / 0 |
| 0 … +0.25 | 9 % | 23 % | 0 / 2 % |
| +0.25 … +0.7 | 14 % | 7 % | 5 % / 0 |

- **구속하는 것은 관절 속도 정격 × 낙차다.** $\gamma_{\max}$ 의 실제 값은 중앙값 $v_{dir,\max}$ 1.1–1.4 m/s 를 6.5–8.3 m/s 로 나눈 **0.13–0.22** 다 (필요한 $\gamma_{\min}$ 은 0.88–0.96). 가속은 구속하지 않는다 — 구속하는 것은 D-16 box 의 도출법이다 (§9).
- $\Delta z\le-0.5$ m 를 10–50 % 열려면 $v_{rel}$ 3–4 m/s 가 필요하다 ($d_{eff}$ 0.095 m 에서 $T_{close}$ 25–30 ms) — 이 손들의 자릿수가 아니다.
- 그래서 **조건부 go** (2026-09-22 사용자 결정) 의 목표는 투척을 좁히는 것이 아니라 **낙차를 없애는 것**이다: base 높이 부근에서 위로 던져 정점 근처에서 받는다. 조건 셋과 열리는 영역 (`iiwa7_leap` 39 · `ur5e_p1b` 11 / 3808 투척, base 1.0 m 가정 씬) 은 §4.4 S4.4 결과가 SSoT 다. 아래 `sim.throw_region` 제안 (4 m, 앙각 44–56°, 5.0–6.5 m/s) 은 4 m kinematic 지도의 제안이고 **S3.5b 가 대체했다**: `ur5e_p1b` 는 거리 0.9–1.0 m · 릴리스 0.15–0.25 m · 방향 편차 ±6° · 속력 4.65–4.85 m/s · 앙각 62–64° (provisional, 기준 투척 1.0 m · 0.2 m · 4.75 m/s · 60° 주변), `iiwa7_leap` 은 없음 — §4.4 S3.5b 결과.

**발사 조건 공간.** 발사점은 방위 φ 와 높이 z 로, 발사 속도는 크기·앙각·수평 방향으로 정한다. 수평 방향은 "발사점에서 겨냥점(aim point)을 향한 방향 + 편차" 로 두어, 사람이 로봇 쪽으로 던지되 좌우로 빗나가는 투척을 표현한다. 포구 후보는 비행시간 T_f ≥ 1.0 s 인 것만 남긴다 (D-18) — 항력 포함 기준 T_f 1.0 s 에서 앙각 40–53°, v₀ 4.7–5.9 m/s, 정점 z 2.2–2.8 m 이고 T_f 가 길수록 모두 커지는 lob 이다 (S0.7 결과). 지도 도구의 초기 탐색 범위 (제안): φ ∈ [−π/2, π/2] (로봇 정면 반원), 편차 ∈ [−10°, +10°] — 탐색 범위일 뿐이며, 잡을 수 있는 범위는 지도 결과로 정한다.

**지도 도구 출력 (S3.5a/b).** 발사 속도·각도 격자별로 (a) 포구 가능 여부, (b) 최대 w 와 그 후보의 t_c·p_c·q*, (c) 탈락 사유(IK 실패·w 미달·비유한 입력·도달 불가·γ 창 없음)를 표로 내고, 잡을 수 있는 발사 조건 범위를 `sim.throw_region` 의 방위·속도·앙각·방향 편차로 제안한다 (방위별 포구 가능 구간 그림 포함). 사용자가 sim 에서 q* 자세를 보고 threshold 를 갱신하면 지도를 다시 돌린다. 4 m 는 vision 지평 요구(S0.7)와 가속 box(S2.5)를 동시에 가장 어렵게 만드는 설정이다 — 둘이 타협을 요구하면 되돌아오는 값이 거리다.

## 12. 알려진 위험

| 위험 | 닫는 단계 |
|---|---|
| vision 토픽이 stable ABI 가 아니다 (D-4) | S5.2 레이아웃 해시 진단으로 감지, 제품 ABI 는 ball_perception |
| vision 지평이 짧으면 계획 가능한 포구 창이 준다 — S0.7 로 sim profile 을 0.8 s 로, S3.6 이 1.0 s / 20 점으로 올려 설정했고 (D-15, 2026-09-22) 지평 요구는 연속 재계획 기준 R1 이다. 첫 목표가 작업공간 가장자리라 그 후보가 전멸하면 정지 출발(R2, 먼 q* 는 약 1.0 s)로 떨어진다. 지평을 늘릴 주체는 외부 repo 이고 요청 채널이 없다 (P-3). S3.6: gate 통과 창은 끝이 0.73 s 라 0.8 s profile 안이지만 **기구학 reachable 창은 0.95 s** 이고 T_det 실측이 가정보다 일러서 (p05 0.027 s) D-27 기준으로는 **1.05 s / 21 점**이 필요하다 — 0.8 s 를 유지하면 늦게 잡는 후보가 계획기에 도달하지 못한다 | S3.5a 가장자리 catchability, S8 첫 plan 측정, S3.6 |
| C-1 전체 거부가 포구 직전 기아가 될 수 있다 (지평 끝이 지면·작업셀 밖에 닿을수록 무효 점이 생김) | S3.4 validity 히스토그램 → S5.2 착수 전 재검토 |
| 실기 시계 오프셋이 미래 방향이면 지평 검사는 통과하고 t_c 만 늦어진다 (6 m/s·10 ms = 6 cm). L3 §4.6 의 ε_clk 는 분산 항이라 bias 를 모델링하지 않는다. sim 은 같은 호스트라 드러나지 않는다 | S10 (D-2 예외 ③ 조건, TBD-NET-01) |
| sim T_close 는 MJCF 게인에 의존 — 실기 측정 전까지 S4 결론은 잠정 | S4.3, S10 |
| γ derate 제외(D-8)로 abort 가 늘 수 있다 | S8 측정 (시행별 `ref_saturated` max streak, G8-C3) — **S8-E 기록 (2026-09-26)**: 연속 > 0 시행 tennis 165/200 (재실행; 원 158/200) · beanbag 161/200 · leap 5/200, 판정 없음 (§4.4 S8-E 결과) |
| sim 팔 actuator 는 1차 지연이다 (MJCF 게인 τ ≈ 200 ms → D-S8-13 으로 p1b sim 은 **τ 0.05 s**) — 선행 보상 (순수 지연) 은 비-순수지연분을 남기고, sim 에서 잰 lead 이득은 UR5e 이득을 예측하지 않는다 (L5 §4.4 정정) | S8-B (G8-E 잔여 보고), S10 |
| ~~T_freeze 0.52 (T_arm 0.2 파급, C-25) 가 S3.5b 목표 분포의 열린 후보를 지울 수 있다~~ — 발현 (0/180) → D-S8-13 (sim τ 0.05, T_freeze 0.37: 163/180) 으로 해소 | S8-B 지도 재실행 (완료 2026-09-24) |
| D-3 이 검증에서 떨어지면 S3·S5 시간 경로 재작업 | S3.1a, S3.1b (S8 시행 누적 — δ 는 공변량, D-S8-4 (c)) — **S3.1b 충족 (S8-E, 2026-09-26)**; host 부하 (RTF < 1) 는 아래 행 |
| sim 이 실시간보다 느려지면 (host 부하, RTF < 1) 발사 기준 sim 시간축의 공 stamp 가 벽시계보다 뒤처져 컨트롤러의 steady 나이 검사가 입력을 `BALL_STALE` 로 끊는다 — 성공률이 host 부하에 좌우된다 (S8-E 원 판정: tennis 부하 시행 5/28 성공). 사전 무효 규칙 `sim_stall` 은 sim 스텝 간격만 봐서 못 잡는다 | D-S8-17 (unit 재실행 규칙, 2026-09-26). unit 도중 부하 감시·중단은 private 드라이버에만 있다 (repo 러너 `catching_sim_trials` 에 없음; `catching_trials` 는 `rtf_trial_min` 을 공변량으로만 낸다) — 후속 |
| sim 폐루프 재현 편차 — 부하가 없던 시행도 같은 투척에서 unit 당 ±5 발 달라진다 (S8-E D-S8-17 재실행, 원인 미확인 — 노드 간 메시지 타이밍 추정). G8-D 의 원 판정 83 FAIL 대 재실행 93 PASS (통과선 84) 는 부하 제거와 이 편차를 함께 담는다 | 미배정 (후속) |
| L3 §4.6 직교 분해 오차 예산이 간극 분산을 과대추정한다 (A·B 음의 상관, S8-E G8-C2 FAIL) — 예산식이 보수적이다 | L3 §4.6 예산식 재검토 (후속) |
| 토크에서 도출한 보수적 가속 box (D-16) 가 받을 수 있는 공 속력을 낮출 수 있다 | S4.4 |
| 1차원 분해가 복잡한 경우를 놓치면 NLP 전환 (§8) 이 필요해 S6 재작업 | S6·S8 신호 — S8-E (2026-09-26): 전환을 시사하는 신호 없음 (§7.3) |
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
| S3 | sim 공 발사(D-14 srv) 버튼·발사 조건 입력, 공 상태(ground truth·vision 예측 수신 여부) 표시 — **완료 2026-09-20**: `demo_controller_gui` Control 탭의 ball 패널. 패널이 `/sim/launch_ball_at` 과 **같은 집합을 거부**하고(필드 이름을 대며) 피드를 never/live/stale **셋**으로 구분한다 (브링업 중 앞의 둘은 화면에서 같아 보이면서 정반대를 뜻한다). vision 예측은 토픽 구독뿐이라 ball_perception 없이도 "never received" 로 정직하게 동작한다. 게이트: `test_demo_gui_ball_launch.py` | clock 위상 오차 CSV (δ·pause, §5) — **완료 2026-09-20**: `analyze_clock_phase --plot` (합성 픽스처에 4 ms 스톨을 주입해 δ_max·max pause 로 복원되는 것을 확인). 결과는 §5.1. catchability 지도 결과 (w₅·w₆ 분포, 방위별 포구 가능 구간) 는 지도 도구 자체 플롯 |
| S4 | 손 step 명령·T_close 식별 실행 — **도구 완료 2026-09-20**: `demo_controller_gui` Control 탭의 Hand Step 패널 (Load profile / Step → open·preshape·closed + 라이브 ρ). 자세는 패널에 박지 않고 **컨트롤러의 읽기 전용 파라미터**에서 읽는다 — 화면의 자세와 run 이 쓴 자세가 갈리면 확인용 스크린샷이 엉뚱한 것을 확인하게 된다. ρ 는 `rtc_tools.analysis.hand_close` 를 import 한다 (사본 금지 — 화면 값과 보고서 값이 갈리지 않게). 테스트 `test_demo_gui_hand_step.py` 11 케이스 | T_close 식별 CSV → ρ(t)·T_close 분포 플롯 — **도구 완료**: `analyze_hand_close --plot` (시행별·분포·steady vs tick 산점). **실측은 미완** |
| S5 | 포구 컨트롤러 패널 — **완료 2026-09-22**: `demo_controller_gui` Control 탭의 Catching 패널 (`demo_gui/catching.py`, 순수 python). 모드·사유, 입력 lane (n·generation·sequence·수신 나이·지평), plan, 추종 오차·CLIK 상태, Arm/Disarm. **관측된 무장과 요청된 무장을 따로 보여준다** — tick 이 E-STOP·fault 에서 latch 를 내리므로 파라미터 set 성공은 무장의 증거가 아니고, 둘이 갈리는 순간이 봐야 할 상태다. 거부 카운터는 0 이 아닌 것만 이름과 함께 (0 의 행렬이 0 아닌 하나를 가린다). 컨트롤러가 GUI roster 에 없어 **전환 자체가 불가능했던 것**을 함께 닫았다 (`extra_switchable_controllers`; 목표 패널은 `NO_EXTERNAL_COMMAND_CONTROLLERS` — 두 lane 모두 목표를 거부하므로 보내기 버튼의 유일한 결과가 거부 카운터다). 육안: sim 폐루프에서 APPROACH 중 `p_c=(+0.570,+0.190,+0.400) t_c=+1.492 s law: SOLVED 1 it, 48 us track err 0.0001 rad`, Disarm 클릭이 컨트롤러에 반영 (mode IDLE·`armed=false`). 게이트 `test_demo_gui_catching.py` 23 케이스 | `catching_diag.csv` — **완료**: `plot_rtc_log` 의 1급 log type (`catching_diag`, 컬럼 지문 `track_err_rad`+`ref_gamma`), 기준 vs p_c · 실현 가속도 vs 포화 전 요구값 · 추종 오차 vs solve time · 슈퍼바이저 모드의 4단 공유 x축 figure + 통계. **무효 tick 은 NaN 으로 끊는다** — PROC-7 이 0 으로 지우므로 그대로 그리면 매 tick 손이 원점으로 간 것처럼 읽힌다 (첫 실측 플롯이 그랬다). 회귀 `test_plot_rtc_log.py` 14 케이스 (C++ 헤더 ↔ python 컬럼 목록 oracle 포함) |
| S6 | plan 표시: t_c·p_c·γ_f·w₅·w₆·탈락 사유·plan 나이 | `planner_timing_log.csv` (timing plotter 재사용), plan 이벤트 CSV (후보별 게이트 결과, 수신 → 게시 지연) → plot |
| S7 | 슈퍼바이저 모드·사유·결과, 손 위상, 접촉 센서·센서 freshness | 전이 로그·손 위상·접촉 CSV → plot (전이 시각선을 추종 플롯에 겹침) — **구현** (2026-09-23): GUI 패널에 결과·손 위상·지문 줄 (`test_demo_gui_catching.py`), `catching_diag` 모든 패널에 모드 전이선 + `catching_hand` figure + 시도별 판정 집계 (`test_plot_rtc_log.py` TestCatchingS7). 육안 확인은 PR 전 |
| S8 | 연속 투척 진행 표시 — **S8-A**: Catching 패널이 결과 엣지를 로컬로 세는 카운트 (시행·판정별; 새 메시지 필드 없음 — `CatchingState` 는 S5 에서 동결) | 시행 요약 CSV → 로봇별 성공률 (truth 기반 Wilson 구간, 전체·무효 발사 수·ITT 하한)·소거실험 (2×2) · t_c 분해 · D-3 공변량 플롯 — **S8-A** `rtc_tools` `catching_trials` (오프라인 평가, 로봇 상수 없음). NEES 는 ball_perception_sim `sim_capture_evaluate` 산출을 쓴다 (P5). **S8-E (2026-09-26, `79a025de`·`879957c0`)**: `catching_trials` 무효 분류·ITT·`--floor` 판정·G7-B3 회귀·`--eval-samples` (G8-B)·`--probe-dump` (G8-C2) + `catching_pool` (arm 별 unit 합산·Wilson·McNemar) — 산출은 요약 JSON·CSV 이고 새 plot·GUI 변경은 없다 (rtc_tools README) |
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
| L5.7 지연 식별 도구 | ~~S3.7~~ → **S10** (2026-09-20, L5.9 에 흡수) |
| L5.8 선행 보상 | S5.3 |
| L5.9 실기 식별 | S10 |
| L5.10 backend 왕복 | S5.3 |
| L6.1·L6.2·L6.5 | S4.1·S4.2·S4.3 |
| L6 d_eff·r_cap 산정 (§4.5) | S4.5 (완료 2026-09-20) |
| L6.3 go/no-go | S4.4 |
| L6.7·L6.6 | S7.1 |
| L7.1–L7.3 | S1.8 (L7.3 은 S7.3 에서 결합) |
| L7.5 | S7.2 |
| L7.6 충격량 예산 | S8 (G7-B3 와 함께 이월, #537 결정 2026-09-24) — G7-B3 상관은 S8-E 기록 (2026-09-26, §4.4 S8-E 결과), 예산 `TBD-IMP-01` 은 미확정 |
| L7.7 | S10 |
| L7.8 E-STOP 훅 | S5.1 (P-1) → S9 |
| L8.2 골격 | S4.0 → S5.1 |
| L8.3 기록·상태 publisher | S5.4 |
| L8.4 D-3·발사 srv·truth | S3.1a·S3.2·S3.3 (부하 재검증 S3.1b 는 S8 시행 누적 — S8-E 로 충족, 2026-09-26) |
| L8.5 vision 연결·지도·사양·NEES | S3.4·S3.5a/b·S3.6 |
| L8.6·L8.7 폐루프 평가 | S8 (S8-E 판정 2026-09-26 — §4.4 S8-E 결과) |
| L8.8 실기 | S10 |
