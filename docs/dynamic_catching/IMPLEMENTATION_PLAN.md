# dynamic_catching — 전체 구현 계획 (living document)

- 상태: **S0 진행 중** (결정 반영 단계)
- 최종 갱신: 2026-09-19
- Epic: [#537](https://github.com/hyujun/rtc-framework/issues/537)
- 수명: 구현 완료 시 prune 한다. 이 문서는 **전체 계획과 결정의 SSoT** 이고, 단계별 상세 작업(sub-plan)은 각 에이전트의 private plan 에서 관리한다 ([AGENTS.md](../../AGENTS.md) §6.6).
- 갱신 규칙: 결정·단계 상태·게이트 결과가 바뀔 때마다 이 문서를 먼저 고친다. 이 문서가 구체화되면 같은 폴더의 설계 문서·참조 구현을 이 문서에 맞춰 갱신·동기화한다 (S0.3, 이후 결정 변경 시마다). 설계 문서(`CATCHING_MASTER.md`, `L0_core.md` … `L8_bringup.md`)와 충돌하면 **이 문서의 결정이 우선**하며, 해당 설계 문서는 S0.3 에서 v0.5 로 고친다.

## 1. 결정 로그

| ID | 결정 | 상태 | 근거 요약 |
|---|---|---|---|
| D-1 | 코드 배치: 수치 코어는 rtc_controllers 의 `catching` 하위 디렉토리 (namespace `rtc::catching`), 축 정렬 오차는 `rtc_math` se3, CLIK 확장은 `rtc_tsid`, 바인딩·YAML·launch·PointCloud2 파서는 `integrated_bringup` | **확정** | 코어가 robot-agnostic 판정([design-principles.md](../../agent_docs/design-principles.md))을 통과한다. `compliance`·`grasp`·`inference` 코어와 같은 선례. 새 패키지를 만들지 않는다 |
| D-2 | 시간 규약: (1) t_c·t_cmd 는 공의 **물리 시각** 단일 정의, 판정별 비교 대상 '지금' 고정(§3). (2) 내부 표현은 절대 steady ns, 상대시각은 수치 코어 경계에서만. (3) nrt 수신 시 `t_ref_steady = recv_steady − (recv_wall − stamp)` 1회 변환, 원격 stamp 를 시간 원점으로 쓰는 것을 [invariants.md](../../agent_docs/invariants.md) 에 E-1 예외로 명문화 | **확정** | L3 §4.11 과 §5.2 의 축 서술 모순 해소. 원점이 다른 상대시각끼리 비교하는 버그 차단. `header.stamp` staleness 금지 규칙 유지 |
| D-3 | sim 시간축: wall clock 유지 + 시행별 RTF 게이트 (비행 구간 Δsim/Δwall < 0.99 시행 무효) | **채택 — 검증 후 추가 검토 필수** (§5) | D-2 변환이 실기와 같은 경로로 동작. `/clock` 방식은 RT 루프에 sim 전용 시간 원천이 필요해 D-2 와 충돌 |
| D-4 | vision 입력: ball_perception 의 실제 PointCloud2 레이아웃 채택, 필드 이름으로 파싱, `generation`·`validity`·`snapshot_sequence` 사용, NaN 공분산 = 모름 | **확정** | 실제 발행기 존재. 설계 문서 §5 의 372 B·`t`·`cov` 가정 폐기 |
| D-5 | CLIK: `rtc::tsid::ClikReferenceGenerator` 를 옵션(기본 off)으로 확장 | **확정** | P5 일반화. off 시 기존 출력 bit-identical, 기존 assertion 수정 필요 시 즉시 E-6 |
| D-6 | CLIK 오차·J 를 명령값 q_c 에서 평가하는 옵션 추가 | **확정** | 측정 q 평가는 servo 지연을 루프에 품고 선행 보상과 이중 보상. 실추종은 `TRACK_ERR` 로 별도 감시, 재앵커 시점 규정 필수 |
| D-7 | 계획기 스레드는 **기존 MPC 스레드와 같은 생성 방식**으로 만들고 기능만 planner 로 한다 | **확정** — D-7b~d 확정, D-7a 는 RT 준수 코드 + 측정으로 확정 (§6, §7) | nrt_callback executor 는 단일 스레드라 계획 계산(수십 ms)을 올리면 궤적 수신·서비스가 막힌다 |
| D-8 | γ derate 는 v1 에서 제외. 실행 중 포화 → COMMITTED 전 RETREAT, 이후 ABORT_SAFE. S8 에서 포화 빈도 측정 후 재설계안 도입 여부 결정 | **확정** | 참조 구현 probe: Frozen 분기 γ_min 미보장, 완화 분기 무효, 기본 램프가 가속 피크를 키움, 램프가 t_c 초과 가능 |
| D-9 | `gammaWindow` 의 TCP 속도 = η_v · `reference.v_max` (0 < η_v ≤ 1). 마스터 §6 교차제약을 이 식으로 수정 | **확정** | 계획이 한계 끝을 쓰면 실행 중 예측 변화로 포화. D-8 로 derate 가 빠져 유일한 완충 |
| D-10 | catch frame: 후보 p1b `l_palm_link` +z, iiwa7_leap `palm_lower` −z (FK 도출, 확인 필요) + 포켓 중심 offset. offset 은 `rtc_urdf_bridge` 모델 빌더가 YAML 선언 frame 을 Pinocchio 모델에 추가하는 방식 | **확정** (값은 D-12) | CLIK 은 모델 frame id 만 받는다. binding 로컬 offset 은 CLIK 까지 못 간다 |
| D-11 | 손 명령 포트 추상화 폐기. 손은 `ControllerOutput` 의 손 device slot 에 직접 기록. T_link 분리 측정 대신 종단 간 T_close,tot 실측 | **확정** | P1b·LEAP 모두 이미 device group. `udp_hand_node` 는 명령 stamp 를 읽지 않는다 |
| D-13 | E-STOP·fault 정책 (E-8) | **보류 — 다른 기능 전부 구현 후 마지막에 결정** (§4) | 사용자 결정 |
| D-14 | 공 발사 API: (p0, v0, ω) 명시 srv 를 `rtc_msgs` 에 추가 (Adding a New Message, PROC-3) | **확정** | 파라미터 설정 + Trigger 는 경합·재현성 약함 |
| D-15 | vision 예측 사양(지평·간격·점 수·발행률)은 **포구 제어기가 요구 사양을 정하고**, sim 에서는 공 투척 설정과 ball_perception sim profile 을 그 요구에 맞춰 설정한다. 제어기는 수신 궤적의 지평이 요구보다 짧으면 계획 후보에서 제외·진단한다 | **확정** | ball_perception 은 sim 이 주는 위치로 미래 궤적을 만드는 노드이고 사용자가 직접 설정한다. 현재 예시 profile 은 지평 0.5 s, 간격 0.05 s, 최대 10 점, ≤ 30 Hz |
| D-16 | 관절 가속 한계는 **토크 한계에서 도출**한다 (§9). 시뮬레이션 추정은 교차 검증용. YAML 의 기존 `max_acceleration` 값은 쓰지 않는다 | **확정** | 가속 데이터 없음, 토크 데이터 있음. 기존 `max_acceleration` (5.0 rad/s²) 은 CM 이 읽기만 하고 어떤 컨트롤러도 쓰지 않는 placeholder |
| D-17 | catch frame 의 부모 frame·위치 offset·자세는 **YAML 로 열어 둔다**. 초기값은 S2.3 에서 제안하고, 사용자가 sim 에서 확인해 실제 값으로 갱신한다 (§10) | **확정** | 사용자 결정 |
| D-18 | 투척 목표는 **arm manipulability 기반 포구 가능성(catchability)** 으로 정한다. 발사 영역 (arm base frame 기준 수평 거리 √(x²+y²) = 4 m 의 원호 — 좌우로 흩어진 투척 포함, world z 1.5–2.0 m = 사람이 손으로 던지는 높이) 에서 출발한 궤적 위 포구 후보마다, 손바닥 +z 가 공 진행 방향을 마주보는 자세(a_d = −v̂)의 IK 해에서 manipulability 를 재고, threshold 이상인 후보가 있으면 잡을 수 있는 공, 없으면 포기. 이 판정으로 투척 속도·각도 범위를 정한다. threshold 초기값 0.1 (provisional, 사용자가 sim 에서 자세를 보고 갱신) | **확정** — 정의 세부는 §11 | 사용자 결정 |
| D-12 | 사용자 제공 값: 투척 목표는 D-18 로 대체, 공 사양, 실기 T_close,tot 측정 시점, 성공률 하한·시행 수. 관절 가속 한계는 D-16, catch frame 은 D-17 로 대체 | **방식 확정, 값 대기** | 추측 금지. 임시값은 YAML 에 provisional 표시 |

## 1a. Sprint Contract (A-1 승인, 2026-09-19)

Epic 기준 하나와, **각 단계 착수 시 그 단계의 `[SPRINT]` 기준**을 따로 확정한다 (단계 sub-plan 의 `## Spec`).

```
[SPRINT] 1) iiwa7_leap·ur5e_p1b MuJoCo 에서 ball_perception PointCloud2 예측만을 입력으로,
            목표 투척 분포(D-12)에서 포구 성공률의 Wilson 95% 하한 ≥ floor(D-12), 로봇별 보고
         2) 전 RT 경로 할당 0·noexcept·RT-1~10 준수, 기존 컨트롤러 테스트 assertion 무수정 green
         3) 설계 문서 v0.5 가 코드와 일치하고, 이 문서에 단계별 게이트 결과가 기록됨
```

실기(S10)는 Epic 기준 밖이며 S10 착수 시 별도 기준을 세운다.

## 2. 단계 W 결론 요약

코드 대조 결과 (2026-09-19). 상세 기록은 S0.2 에서 `WORKSPACE_ANALYSIS.md` 기록 칸에 옮긴다.

- 명령 경로: 컨트롤러는 `RTControllerInterface::Compute` 에서 `ControllerOutput` 을 채우고, CM 이 `DeviceBackend::WriteCommand` 로 보낸다. ros2_control 이 아니다. 실기 UR 은 vendor `forward_position_controller` 토픽, sim 은 `mujoco_native`, P1b 손은 `udp_hand_native`
- 없음이 확인된 것: UR 지연 보상, speed scaling 노출, `ApplySafetyLayer` 의 production 호출, 스트리밍 목표를 받는 컨트롤러, 독립 IK, Pinocchio offset frame 추가 기능, sim `/clock`
- 시간: RT 의 `ControllerState::t_relative_s` 는 steady clock 기반, `ControllerState::dt` 는 항상 1/`control_rate` (sim lock-step 에서 실제 간격과 다를 수 있음). `header.stamp` staleness 판단 금지
- CLIK: `ClikReferenceGenerator` 는 pose 목표만, LWA 6행 고정, 측정 q 에서 e·J 평가, 위치∩속도 box, `max_iter` 20 고정. 가속 box·feedforward·마스크·상태 노출 없음. `PinocchioCache` Jacobian 은 `LOCAL_WORLD_ALIGNED` 고정
- 재사용 대상: `rtc::SeqLock`, `rtc::SpscQueue`, `rtc::compliance::DifferentialIk`, `rtc_math` se3 `log3`/`exp3`, `QPSolverWrapper`
- 지문 센서: P1b 실기 `HandSensorState` 250 Hz, finger-on-object 부호. sim 은 `WrenchStamped` env-on-link **반대 부호**
- sim 공: `/sim/launch_ball`·`/sim/reset_ball` (Trigger), `/sim/ball/ground_truth` (Odometry), `/sim/ball/camera_position` (PointStamped + noise), 항력·Magnus 자체 구현, iiwa7_leap 설정 없음
- vision: 형제 workspace 의 ball_perception 저장소 `ball_perception_sim` 패키지 `sim_estimator_node` 가 `/sim/ball/camera_position` 을 구독해 예측 궤적 PointCloud2 를 debug 토픽으로 발행한다 (point_step 384, `horizon_ns` u32, `generation`, `validity`, `covariance` NaN=모름). **stable ABI 아님** — 제품 ABI 는 ball_perception E6-F02 로 defer
- 참조 구현 테스트: l0/l2/l3/l4 + `verify_l3.py` 전부 통과, ASan/UBSan 통과. 단 테스트 밖 결함 확인: `n > kMaxSamples` 범위 밖 읽기(ASan), NaN 목표 영구 오염, derate 결함(D-8)

## 3. 시간 규약 (D-2 확정본)

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

## 4. 단계 계획

순서: S0 → (S1 ∥ S2 ∥ S3) → S4 → S5 → S6 → S7 → S8 → **S9 (D-13 E-STOP 정책)** → S10 실기

각 단계 착수 시 sub-plan 을 private plan 에 만들고, 완료 시 이 표의 상태와 게이트 결과를 갱신한다.

| 단계 | 상태 | 게이트 결과 |
|---|---|---|
| S0 결정·문서 v0.5·계약 | 진행 중 | — |
| S1 순수 수치 코어 | 대기 | — |
| S2 기존 rtc_* 일반화 | 대기 | — |
| S3 시뮬레이션 기반 | 대기 | — |
| S4 손 타이밍 (go/no-go) | 대기 | — |
| S5 포구 컨트롤러 골격·입력·추종 | 대기 | — |
| S6 계획기 스레드 | 대기 | — |
| S7 손 시퀀서·슈퍼바이저 | 대기 | — |
| S8 sim 통합 평가 | 대기 | — |
| S9 E-STOP·fault 정책 (D-13) | 대기 | — |
| S10 실기 단계 도입 | 대기 | — |

### S0 결정·문서 v0.5·계약 (코드 없음)

- S0.1 결정 D-1~D-14 확정 (D-12 는 값이 준비되는 대로, D-13 은 S9)
- S0.2 `WORKSPACE_ANALYSIS.md` 기록 칸을 §2 로 채운다 (확인 방법·일자 포함)
- S0.3 설계 문서 v0.5 개정
  - ros2_control·`update()` 전제 → `Compute`/`ControllerOutput`/`DeviceBackend`
  - `generate_parameter_library` → `LoadConfig` + `ParseXxxParams` + runtime gain `declare_parameter`
  - L5 "얇은 어댑터" → CLIK 확장(D-5·D-6) + 새 컨트롤러
  - RT tick 안 try/catch 제거 (RT-2), FAULT 시 RT 에서 deactivate 제거
  - 시간 서술을 §3 으로 통일 (L1·L2·L3·L6·L7), 500 Hz 고정 → `dt` 기준
  - 손 포트 추상화 삭제 (D-11), γ derate 를 v1 범위 밖으로 (D-8), η_v 교차제약 (D-9)
  - L0·L2·L3·L4 문서의 코드 블록 삭제 → 헤더가 SSoT (L2 블록은 v0.2 그대로이고, 나머지는 원래 헤더와 byte 동일했으나 헤더 포맷 적용 커밋 47158227 이후 어긋남)
  - 이름이 둘인 같은 값 5쌍을 단일 키로 (`n_min`, `derate_step`, `ed_jump_max`, `a_dec`, ramp)
  - L7 Reason enum 완전화, IDLE→wait_pose homing 추가, 신규 스레드 서술을 D-7 로 교체
  - L1 이 L2 타입에 의존하는 역전 해소 (궤적 타입을 공용 타입으로)
  - 참조 코드 명명: namespace `rtc`, 함수 PascalCase
  - `controllers.md` 의 DemoWbc "TSID QP → accel → position integration" drift 는 별도 수정 대상으로 기록
- S0.4 (삭제 — ball_perception 은 사용자가 직접 개발 중이라 요청 이슈 불필요, §7.1 A-3)
- S0.5 Epic issue [#537](https://github.com/hyujun/rtc-framework/issues/537) 생성 완료 (단계 완료·게이트 결과·결정 변경을 코멘트로 남긴다) + Sprint Contract

게이트: `validate_docs.py` 통과, W 기록 칸 비어 있지 않음, 결정 로그 갱신.

### S1 순수 수치 코어 (ROS 비의존)

- S1.1 rtc_controllers `catching` 하위 디렉토리 골격, 참조 테스트를 GTest 로 이식
- S1.2 궤적 타입(공용) + Hermite 샘플러. SeqLock 에 싣는 타입(궤적 스냅샷, PlanSnapshot)은 trivially copyable POD (`std::array` 기반, Eigen 멤버 금지 — §6). 점 개수 `[n_min, kMaxSamples]` 를 파서·check·RT 읽기 모두에서 먼저 검사 (용량 `kMaxSamples` 는 S3.6 요구 사양에 여유를 두어 정하고, 그 전에는 provisional), NaN 입력 거부, `dt_min` 미만은 경고가 아니라 거부
- S1.3 시간 타입 `BallTime`/`NowReal`/`NowLead` (§3)
- S1.4 soft-catch 기준 생성기: NaN 가드(비유한 목표 시 상태 보존 + invalid), derate 없음(D-8)
- S1.5 도달 가능성: `time_feasibility`, 방향 속력(투영 v̂ᵀJ_p q̇, 0 가드), 정지거리·오차 예산. 잘못된 한계는 flag
- S1.6 `ball_dynamics` 는 test fixture 전용 위치로
- S1.7 파라미터 검증 로직: 활성 구성 키만 TBD 검사, 교차제약 표(D-9 반영), ζ·ω·h 검사(`dt` 기준)
- S1.8 L7 순수 조각: 감속 목표, 전이표를 데이터로, 접촉 debounce

게이트: 기존 참조 테스트 전부 GTest 통과, 범위 밖 읽기·NaN 회귀 테스트, ASan/UBSan, `ScopedNoMalloc`·`ScopedAllocGate` 할당 0.

### S2 기존 rtc_* 일반화 (code review 대상)

- S2.1 `rtc_math` se3 에 축 정렬 오차·각속도·Jacobian (deadband 에서 유한), 유한차분 테스트
- S2.2 `ClikReferenceGenerator` 옵션 (D-5·D-6): twist feedforward, LOCAL 접근축 2행, 가속 box + `bound_conflict`, 직전 q̇ 평활 항, status·반복·solve time 노출, `max_iter` 설정, q_c 평가 모드. 착수 후 첫 설계 리뷰에서 "행 집합 선택형 확장" 과 "`QPSolverWrapper`·se3 오차만 공유하는 formulation 클래스" 중 하나로 확정
- S2.3 `rtc_urdf_bridge` 모델 빌더: YAML 선언 추가 frame (D-10, D-17). 스키마는 §10. 초기 제안값 산출(§10) 후 사용자 sim 확인
- S2.5 관절 가속 한계 도출 도구 (D-16, §9): 토크 한계 + 동역학 모델 → 보수적 가속 box, provenance 포함 YAML 출력. sim 교차 검증
- S2.4 DemoWbc 회귀(기존 assertion 무수정), `rtc_tsid`·`rtc_urdf_bridge` downstream 빌드·테스트, `/code-review`

게이트: 옵션 off 시 기존 출력 동일, 기존 테스트 전부 green, 할당 0. S2.5 는 도출값이 sim 교차 검증(§9)과 모순되지 않음.

### S3 시뮬레이션 기반 (`rtc_mujoco_sim`, robot-agnostic)

- S3.1 **D-3 검증** (§5) — 결과에 따라 D-3 재검토
- S3.2 발사 srv (D-14), iiwa7_leap projectile 설정, 투척용 스폰 위치
- S3.3 공 접촉 truth(시각·충격량·접촉력) 출력, truth 발행 주기 상향, sim time 진단 출력(RTF 게이트용)
- S3.4 `sim_estimator_node` 연결: clock domain(`use_sim_time=false`), `frame_id` 와 world 관계, 발행 주기·N·지평 실측 (TBD-VIS-04/06), 지연·드롭 주입
- S3.5 catchability 지도 도구 (D-18, §11): 발사 영역 × 발사 속도·각도 격자 → 궤적 → 포구 후보 IK → manipulability → 잡을 수 있는 발사 조건 범위. 결과를 발사 srv(D-14) 설정으로 사용
- S3.6 vision 요구 사양 산출 (D-15): 목표 투척 분포에서 "검출 이후 포구 창 종료까지 최대 비행 시간" → 필요 지평, L2 보간 게이트를 만족하는 간격 → 점 수 → 파서 용량. 결과를 ball_perception sim profile 설정값으로 제시 (설정은 사용자)

게이트: 발사 → PointCloud2 수신 end-to-end, seed 재현성, RTF 게이트 동작.

### S4 손 타이밍 선행 측정 (go/no-go)

- S4.1 손 프로파일 YAML (P1b 10 DoF, LEAP 16 DoF)
- S4.2 T_close 식별 도구: 손 device slot step + CSV → ρ-min 분포 (sim)
- S4.3 가능하면 실기 T_close,tot 측정
- S4.4 받을 수 있는 최대 공 속력 계산 → 목표 투척 속도(D-12) 확정 또는 하향

게이트: 목표 속도에서 γ 창이 비지 않음. 비면 목표를 낮춘 뒤 진행.

### S5 포구 컨트롤러 골격·입력·추종 (Adding a New Controller)

- S5.1 컨트롤러 등록, YAML, lifecycle, 재무장. E-STOP·fault 훅은 **최소 동작만** (S9 전 임시 기준, §7.1 A-1)
- S5.2 PointCloud2 구독(nrt) → 필드 이름 파서 → SeqLock 스냅샷 (공분산 제외, A-3). D-2 변환, `generation`/`validity`/`snapshot_sequence` 처리, 지평이 요구(D-15)보다 짧으면 진단. 공분산은 계획기 쪽 버퍼에만
- S5.3 스트리밍 기준 → 확장 CLIK → 팔 명령. QP 비의존 관절공간 abort 경로
- S5.4 CSV 로그, 상태 publisher (`PublishRole` 없이)
- S5.5 ground truth 기반 고정 포구점(oracle plan)으로 추종 검증

게이트: L4 G4-H·L5 G5-A~C4 (sim), 할당 0.

### S6 계획기 스레드 (D-7)

- S6.1 스레드 골격: MPC 스레드 생성 방식 그대로 (§6). RT-1~10 준수 코드, 초기 FIFO, thread layout role 추가 (E-7 절차)
- S6.2 포구 자세 IK: `DifferentialIk` (m=5) + 스레드 전용 모델 handle. IK 직후 manipulability 게이트 (D-18) — 기준 미달 후보 제외, 후보가 모두 탈락하면 plan 없음(포기)을 사유 코드와 함께 기록. S3.5 도구와 **같은 함수·같은 YAML 키**를 쓴다
- S6.3 γ 창·rollout (S1 코드 호출), 예산 초과 시 coarse-to-fine
- S6.4 후보 선택·hysteresis·commit/freeze, `PlanSnapshot` SeqLock
- S6.5 D-7a 측정: 제어 PC 부하 상태에서 FIFO·OTHER 각각 수신 → plan 게시 지연 p50·p99·최대, 예산 초과율 → §7.2 기준으로 정책 확정
- S6.6 NLP 전환 대비 (A-4, §8): 탐색 전략을 계획기 코어의 단일 진입 함수 뒤에 두어, 1차원 탐색 + IK 를 NLP 로 바꿔도 스레드·입출력 스냅샷·RT 쪽 소비 코드는 그대로 두는 경계를 유지. 전환 판단 신호(IK 수렴률 G3-G, 계획 성공률, 예산 초과율)를 S6·S8 에서 기록

게이트: L3 G3-A~E, G3-C 예산 준수, Adding a New Thread 3 oracle.

### S7 손 시퀀서·슈퍼바이저

- S7.1 손 시퀀서 → 손 device slot
- S7.2 FSM (전이표 = 데이터, Reason 완전), IDLE→wait_pose homing. DECEL 은 시각 기준 진입 (A-5, now_lead ≥ t_c), 지문 센서는 결과 판정·abort 전용. COMMITTED 이후 stale 은 동결 plan 으로 계속, `supervisor.stale_committed_max` 초과 시 ABORT_SAFE (A-6, 초기값은 vision 발행 주기의 3배로 제안하고 S8 에서 조정)
- S7.3 접촉 판정 (sim·실기 부호 정규화), 감속, 충격량 예산
- S7.4 abort·retreat·재무장 리셋, 연속 투척

게이트: L7 G7-A~E, L8 G8-A2.

### S8 sim 통합 평가

- iiwa7_leap → ur5e_p1b. Wilson CI, NEES, 소거실험(γ, lead). **γ 포화 빈도 측정 → D-8 재검토 입력**

게이트: L8 G8-A~E (성공률 하한은 D-12).

### S9 E-STOP·fault 정책 (D-13)

- D-13 결정 → 구현 → `/security-review` (E-8)
- 결정 시 검토할 부작용: E-STOP 중 손도 측정 자세 유지 → position servo 간극이 0 이 되어 파지력 소실 가능

게이트: E-STOP 발동·해제 시나리오 테스트, security review 통과. **S10 착수 전 필수.**

### S10 실기 단계 도입 (HW-P1B)

- bag replay(재스탬프 도구) → 가상 공 → 저속 실투척 → 상향
- T_arm 식별·선행, speed scaling·PTP 감시(신호 출처 확보 후)

게이트: L5 G5-F, L6 G6-D/E, L7 G7-F, L8 G8-F/G.

## 5. D-3 검증 계획 (채택 조건부)

D-3 은 **검증 결과를 바탕으로 추가 검토한다.** S3.1 에서 다음을 측정하고, 하나라도 기준을 못 넘으면 `/clock` 방식을 포함해 D-3 을 다시 결정한다.

| 항목 | 방법 | 기준 |
|---|---|---|
| 공 비행 중 RTF | 비행 구간 Δsim_time / Δwall (sim time 진단 출력) | ≥ 0.99 이 대부분의 시행에서 유지 (구성별 무효 비율 기록) |
| 부하 시 RTF | 포구 컨트롤러 + 계획기 + `sim_estimator_node` 동시 구동 | 위와 같음 |
| 예측 일관성 | vision 예측 궤적 vs ground truth (같은 wall 시각 축) | 오차가 RTF < 1 구간에서만 커지는지 확인 |
| stamp 도메인 | `sim_estimator_node` 의 stamp 가 wall 인지 | `use_sim_time=false` 에서 wall |

## 6. D-7 계획기 스레드 구성

기존 MPC 스레드와 같은 방식으로 생성하고 기능만 planner 로 한다. 분석 결과 (2026-09-19):

**MPC 스레드의 실체.** 공용 기반은 `rtc::PeriodicRtThread` (rtc_base threading, 헤더 전용)다: jthread + stop token, 스레드 진입 시 `ApplyThreadConfigVerbose` (실패해도 무시하고 계속 실행), `clock_nanosleep` 주기, overrun 카운터, Pause/Resume, t0..t3 timing payload. `rtc::mpc::MPCThread` 는 그 위의 얇은 subclass 이고 (`OnTick` = 상태 읽기 → `Solve` → 결과 게시), DemoWbc 가 `std::unique_ptr` 로 소유한다. 수명: `on_activate` 에서 layout profile 게이트 → lazy spawn → `Resume`, `on_deactivate` 에서 `Pause`, **join 은 소멸자에서만** (use-after-free 수정 이력). 배치는 `repo_scripts/config/thread_layout.yaml` 의 `mpc_main` role (tier 6 이상 slot 3 FIFO 60, tier 4 는 OTHER).

**planner 구성 (권장).**

- 클래스: `PeriodicRtThread` 의 **형제 subclass** (catching planner thread). `MPCThread`/`MPCHandlerBase` 를 상속하면 PlanSnapshot 을 `MPCSolution` 에 억지로 넣게 되므로 쓰지 않는다. 같은 기반을 쓰는 4번째 소비자라 P5·ARCH-3 을 만족한다. 탐색 코어는 rtc_controllers `catching`, 스레드 소유는 `integrated_bringup` 바인딩 (D-1)
- 수명: DemoWbc 관용구 그대로 (configure 에서 전 버퍼 할당, activate 게이트 → spawn → resume, deactivate pause, 소멸자 join, aux 타이머로 `planner_timing_log.csv` drain)
- 데이터: RT → planner `rtc::SeqLock<RtStatePod>`, planner → RT `rtc::SeqLock<PlanSnapshotPod>`. **SeqLock payload 는 trivially copyable 이어야 하고 `Eigen::Vector3d` 는 아니다** (Eigen 3.4 에서 확인) → PlanSnapshot·궤적 스냅샷은 `std::array<double, N>` 기반 POD 로 설계 (S1.2 에 반영)
- 기동: L3 는 새 궤적마다 깨어나는 event 구동이다 → `WaitForNextTick` 을 eventfd 대기 + 제한 시간으로 override, `JitterMeaningful()` false
- 배치 변경은 E-7 이며 Adding a New Thread 절차를 따른다: manifest role·전 tier·profile, `gen_thread_layout.py --write`, `SystemThreadConfigs` 필드, `ValidateSystemThreadConfigs` 고정 크기 배열·이름 목록, generator 의 shield 코어 도출(누락 시 cpuset 밖 pin 이 EINVAL 로 조용히 실패), 3 oracle, launch profile 배선과 activate 게이트 테스트

**복사하지 않을 것.** 현 MPC 경로는 문서상 RT 로 분류되지만 `MPCSolutionManager::PublishSolution` 의 mutex·try/catch, `HandlerMPCThread` 의 `fprintf` 가 있다 — planner 템플릿으로 쓰지 않는다. (repo 문서와 코드의 drift 로 별도 기록 대상.)

세부 선택 D-7a~d 는 §7.

## 7. 세부 결정과 후속 결정

### 7.1 확정 (2026-09-19)

- D-7b slot: 빈 slot 에 새 role (dev PC tier 6 = slot 5). 제어 PC (tier 12) 의 빈 slot 이 E-core 라는 기록은 repo 사실이 아니므로 S6 에서 실측·확인
- D-7c 기동 방식: event 구동 — 새 궤적 수신 시 eventfd 로 깨우고 대기 시간 상한을 둔다. `JitterMeaningful()` false
- D-7d IK 감쇠 법칙: `DifferentialIk` 의 σ_min 적응 λ 를 수용하고 L3 G3-G (IK 수렴률)로 검증. 부족하면 그때 `DifferentialIk` 를 일반화
- A-1 S9 이전 개발 기간의 E-STOP 임시 기준: 해제 후 자동 재개 금지 + q_c·CLIK 앵커를 q_meas 로 reseed 만 구현 (S5.1). 전체 정책은 S9
- A-2 `docs/dynamic_catching/` 는 브랜치 `docs/dynamic-catching-plan` 에 커밋
- A-3 Epic issue 만 생성. ball_perception 쪽 요청 이슈는 만들지 않는다 (사용자가 직접 개발 중) — 레이아웃 변경은 S5.2 파서의 필드 이름·datatype 검사와 레이아웃 해시 진단이 감지한다
- D-12 추측하지 않고, 값이 준비되기 전에는 YAML 에 provisional 로 표시해 활성 구성 TBD 검사가 실기 arm 을 막게 한다
- A-1 Sprint Contract: Epic 기준 + 단계별 `[SPRINT]` (§1a)
- A-2 D-7a 판정 기준: FIFO 가 수신 → plan 게시 지연 p99 를 `planner.budget_s` 의 10% 이상 줄이거나 예산 초과율을 줄이면 FIFO 유지, 둘 다 아니면 SCHED_OTHER. 표본 ≥ 1000 시행 (L3 G3-C 와 같은 규모)
- A-3 공분산(TBD-COV-01): RT 스냅샷에서 분리, 계획기 쪽 버퍼에만 둔다. NaN(모름) 처리도 계획기 한 곳에서
- A-4 계획기 탐색: 1차원 시간 탐색 + IK 로 시작하되 NLP 전환을 염두에 둔 경계를 유지한다 (§8)
- A-5 DECEL 은 t_c 시각 기준 진입, 지문 센서는 결과 판정·abort 전용 (L7 §4.1 권장 채택)
- A-6 COMMITTED 이후 stale 은 동결 plan 으로 계속하고 상한 초과 시 ABORT_SAFE (L7 §4.2 권장 채택)
- A-7 → D-15 (vision 요구 사양은 제어기가 정하고 sim 을 맞춘다)
- D-18 manipulability 정의: 팔 관절 열 5행 w₅ (병진 3 + 접근축 2), threshold 0.1 은 이 정의에 대한 값
- D-18 발사 영역: base frame 수평 거리 4 m 원호 위 (좌우 투척 포함), world z 1.5–2.0 m (사람 투척 릴리스 높이)

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

1. planner 코드는 스케줄링 클래스와 무관하게 **RT-1~10 준수로 작성**한다 (S6 게이트: `ScopedAllocGate`·`ScopedNoMalloc` 할당 0, noexcept, 로깅은 SPSC). 그러면 FIFO/OTHER 는 `thread_layout.yaml` 값 하나로 바뀌고 코드 변경이 없다
2. 초기값은 **FIFO** (rt_callback 보다 낮은 우선순위, 검사 추가)
3. S6 에서 제어 PC 에 부하(포구 컨트롤러 + sim 또는 실기 드라이버 + vision)를 건 상태로 두 정책을 각각 측정한다: vision 수신 → plan 게시 지연의 p50·p99·최대, 예산 초과율
4. 판정 기준 (A-2 확정): FIFO 가 p99 지연을 `planner.budget_s` 의 10% 이상 줄이지 못하고 예산 초과율도 줄이지 못하면 **SCHED_OTHER 로 전환**. 표본 ≥ 1000 시행
5. 측정에서 상류 구간 (nrt_callback 수신)이 지배적이면 D-7e 로 수신 경로를 따로 검토한다

### 7.3 미결정

- D-7e (조건부) vision 수신 경로가 지연을 지배할 때의 대안 — 7.2 측정 결과가 나오면
- D-12 값들
- D-13 E-STOP·fault 정책 — S9
- S2.2 CLIK 확장 구조 (행 선택형 vs formulation 클래스) — S2 첫 설계 리뷰
- L3 후보 점수 가중치 — 튜닝, S6~S8
- 실기 공분산 검증 수단 (L8 §6 의 세 가지 중) — S10
- NLP 전환 여부 (§8 신호) — S6·S8 결과 후
- D-18 지도 도구의 초기 탐색 범위 (§11 제안: 방위 ±90°, 방향 편차 ±10°, 겨냥점 = 대기 자세 catch frame) — S3.5 착수 전 확인

## 8. 계획기 NLP 전환 대비 (A-4)

1차원 시간 탐색 + IK 로 시작하지만, 문제가 복잡해지면 MPC 처럼 NLP 로 바꿀 수 있어야 한다.

- **경계:** 계획기 코어는 "입력 스냅샷(궤적 + 공분산 + 로봇 상태) → `PlanSnapshot`" 단일 진입 함수로 둔다. 스레드(§6), 입출력 SeqLock, RT 쪽 소비(L4·L7), 게이트 순서의 앞단(불확실성·도달시간 사전 필터)은 탐색 전략과 독립으로 설계한다
- **추상 interface 는 지금 만들지 않는다.** 구현이 하나뿐인 abstract interface 는 ARCH-3 위반이다. NLP 구현이 실제로 생기는 시점에 두 구현을 두고 interface 를 도입한다
- **스케줄링과의 관계:** NLP solver 가 할당·예외를 쓰면 RT-1~10 을 지킬 수 없으므로, 그때 D-7a 는 `thread_layout.yaml` 값만 바꿔 SCHED_OTHER 로 간다 (코드 변경 없음 — §7.2 결정 방식 1)
- **전환 판단 신호 (S6·S8 기록):** IK 수렴률 (L3 G3-G), 계획 성공률, 예산 초과율, 1차원 분해가 놓치는 후보(시각·자세 결합) 사례
- 전환 시 재사용 후보: `rtc_mpc` 의 solver 기반·스레드 관용구 (그 시점에 조사)

## 9. 관절 가속 한계 도출 (D-16)

가속 데이터는 없고 토크 한계는 있다: YAML `devices.<group>.joint_limits.max_torque`, URDF `effort`, MJCF `forcerange` 가 같은 값이다 (UR5e 150·150·150·28·28·28 N·m, iiwa7 200 N·m). 기존 YAML `max_acceleration` (5.0 rad/s²) 은 출처 없는 placeholder 라 쓰지 않는다.

**방법 (오프라인 도구, S2.5).**

1. 포구 작업공간·대기 자세 주변에서 관절 자세 q 와 속도 q̇ (속도 한계 이내)를 표본 추출한다
2. 각 표본에서 Pinocchio 로 M(q), h(q, q̇) (RNEA) 를 구한다 — 손을 포함한 결합 모델
3. 모든 |q̈_j| ≤ a_j 에 대해 |M q̈ + h|_i ≤ η_τ · τ_max,i 가 성립하는 충분조건 Σ_j |M_ij| a_j + |h_i| ≤ η_τ · τ_max,i 를 세우고, a = s · w (관절별 가중 w) 로 두어 최대 s 를 구한다 (LP)
4. 전 표본의 최소값을 보수적 상수 box 로 채택하고, 표본 범위·η_τ·모델 버전·일자를 provenance 로 YAML 에 기록한다

- η_τ < 1 은 접촉 충격(L7 충격량 예산)과 모델 오차를 위한 여유다. 값은 S2.5 에서 제안한다
- 상수 box 를 쓰는 이유: 계획기(L3 도달시간)와 CLIK 가속 box 가 같은 한계를 써야 계획이 실행과 일치한다. 자세 의존 한계는 v1 범위 밖이며, RT 에서 매 tick 토크 여유를 감시하는 용도로만 검토한다
- **교차 검증 (sim):** MJCF `forcerange` 가 같은 토크 한계를 가지므로, sim 에서 큰 step 명령으로 달성 가속을 측정해 도출값이 달성 가능 범위 안인지 확인한다
- **실기 주의:** UR 은 position 명령을 받는 쪽 컨트롤러가 자체 가속·보호 정지 기준을 가질 수 있다 (repo 밖, 미확인). S10 에서 식별한다

## 10. catch frame YAML (D-17)

catch frame 은 모델 빌더가 추가하는 frame 이다 (D-10). 사용자가 sim 에서 확인하며 바꿀 수 있도록 로봇 config 에 연다.

```yaml
# 로봇 config (예: integrated_bringup ur5e_p1b _base.yaml 의 모델 절) — 스키마 제안
extra_frames:
  - name: catch_frame
    parent: l_palm_link          # 부모 frame
    xyz: [0.0, 0.0, 0.0]         # m, 부모 frame 기준 — 포켓 중심
    rpy: [0.0, 0.0, 0.0]         # rad, 부모 frame 기준 — 결과 frame 의 +z 가 손바닥 바깥 법선
    provisional: true            # 사용자 확인 전
```

포구 컨트롤러 YAML 은 frame 이름만 참조한다 (`catch_frame: catch_frame`). 접근축은 규약상 이 frame 의 +z 다.

**초기 제안값 산출 (S2.3).**

- 축: FK 로 도출한 후보 — p1b `l_palm_link` +z (rpy 0), iiwa7_leap `palm_lower` −z (x 축 π 회전으로 +z 로 뒤집음)
- 위치: 손 preshape 자세(S4.1 손 프로파일)에서 손가락 끝 위치들의 중심을 부모 frame 에 표현한 값을 포켓 중심 제안값으로 한다
- 제안값은 근거(자세·계산식)와 함께 PR 에 적고, 사용자가 sim 에서 확인한 뒤 `provisional: false` 로 갱신한다
- 검증기는 `provisional: true` 인 catch frame 으로 실기 arm 을 막는다 (D-12 와 같은 규칙)

## 11. 투척 목표와 catchability 판정 (D-18)

**판정.** 공 궤적의 포구 후보점 p_c 마다 L3 §4.2 의 포구 자세(catch frame +z = a_d = −v̂(t_c), 즉 손바닥 바깥 법선이 날아오는 공을 마주봄)를 IK 로 구하고, 그 해 q* 에서 manipulability w(q*) 를 잰다. w ≥ `planner.catchability.manipulability_min` 인 후보가 하나라도 있으면 잡을 수 있는 공이다. 이 판정은 기존 게이트(IK 수렴, 도달시간, γ 창, 정지거리)에 **추가되는 AND 조건**이다 — manipulability 만으로 시간 안에 도달할 수 있다는 보장은 없다.

**같은 판정을 두 곳에서 쓴다.** (1) 오프라인 catchability 지도 (S3.5) 가 발사 조건을 정하고, (2) 런타임 계획기 (S6.2) 가 후보를 거른다. 둘은 같은 함수와 같은 YAML 키를 써야 지도와 실제 판정이 어긋나지 않는다.

**manipulability 정의 (확정).**

- Jacobian: catch frame 의 **팔 관절 열**만 쓴다. 손 관절은 손바닥 frame 에 영향이 없다 (손바닥은 폐쇄 체인 상류, §2)
- 행: 포구 과제와 같은 **5행** — 병진 3 (LOCAL_WORLD_ALIGNED) + 접근축 2 (LOCAL x·y 각속도). 손바닥 법선 둘레 회전(roll)은 포구에 무관하므로 뺀다. w₅ = √det(J₅ J₅ᵀ)
- 기존 CLIK 진단값 (`ClikReferenceGenerator::Manipulability`, 6×6 damped) 은 roll 을 포함하므로 같은 값이 아니다. 로그에는 둘 다 남겨 비교한다
- 단위가 섞여 있어 (m 와 rad) w 의 크기는 정의에 따라 달라진다. **threshold 0.1 은 위 정의에 대한 값**이고, 정의를 바꾸면 다시 맞춰야 한다

**포구 자세의 여유 자유도.** 6축 UR5e 에서 5행 과제는 1 자유도(손바닥 법선 둘레 roll)와 IK 해 가지가 남아 w 가 그 선택에 따라 달라진다. 런타임과 지도가 같은 해를 쓰도록 **대기 자세(wait_pose)에서 시작하는 같은 IK(자세 과제 포함)** 로 정한다. roll 을 w 최대화로 고르는 방식은 v1 범위 밖 (필요하면 S8 이후).

**frame 규약 (함정 주의).**

- "arm base frame" 은 로봇 config 의 CLIK `base_frame` 이다: ur5e_p1b `base` (URDF `base`), iiwa7_leap `link_0`
- ur5e_p1b 에서 URDF `base` 와 `base_link` 는 z 축 둘레 180° 차이다. `base_link` 로 두면 +x 가 반대가 되어 공이 등 뒤에서 날아온다 — 그래도 그럴듯한 결과가 나오므로 조용히 틀린다
- 발사 높이 z 는 world 기준이고 x 는 base 기준이다. world ↔ base 변환은 가정하지 않고, **같은 q 에서 MuJoCo FK 와 Pinocchio FK 를 대조**해 S3.5 착수 시 확정한다

**YAML (제안).**

```yaml
planner:
  catchability:
    manipulability_min: 0.1        # provisional — 사용자가 sim 에서 자세 확인 후 갱신
    definition: "arm_5row"          # 위 정의. 바꾸면 threshold 재보정
sim:
  throw_region:                     # S3.5 지도 도구·발사 설정 입력
    base_frame: "base"              # 로봇 config 의 CLIK base_frame 과 일치해야 함 (검증기)
    distance_m: 4.0                 # base frame 수평 거리 √(x²+y²)
    azimuth_rad: [TBD, TBD]         # 원호 위 발사 위치의 방위 (base +x = 0). 지도 결과로 채움
    z_world_m: [1.5, 2.0]           # 사람이 손으로 던지는 릴리스 높이 가정
    heading_offset_rad: [TBD, TBD]  # 수평 발사 방향 − (발사점 → aim_point) 방향. 지도 결과로 채움
    aim_point_base_m: [TBD, TBD, TBD]  # 겨냥점 (base frame). 초기 제안: 대기 자세의 catch frame 위치
    speed_m_s: [TBD, TBD]           # 지도 결과로 채움
    elevation_rad: [TBD, TBD]       # 지도 결과로 채움
```

**발사 조건 공간.** 발사점은 방위 φ 와 높이 z 로, 발사 속도는 크기·앙각·수평 방향으로 정한다. 수평 방향은 "발사점에서 겨냥점(aim point)을 향한 방향 + 편차" 로 두어, 사람이 로봇 쪽으로 던지되 좌우로 빗나가는 투척을 표현한다. 지도 도구의 초기 탐색 범위 (제안): φ ∈ [−π/2, π/2] (로봇 정면 반원), 편차 ∈ [−10°, +10°] — 탐색 범위일 뿐이며, 잡을 수 있는 범위는 지도 결과로 정한다.

**지도 도구 출력 (S3.5).** 발사 속도·각도 격자별로 (a) 포구 가능 여부, (b) 최대 w 와 그 후보의 t_c·p_c·q*, (c) 탈락 사유(IK 실패·w 미달·도달 불가)를 표로 내고, 잡을 수 있는 발사 조건 범위를 `sim.throw_region` 의 방위·속도·앙각·방향 편차로 제안한다 (방위별 포구 가능 구간 그림 포함). 사용자가 sim 에서 q* 자세를 보고 threshold 를 갱신하면 지도를 다시 돌린다.

## 12. 알려진 위험

- vision 토픽이 stable ABI 가 아니다 (D-4)
- sim T_close 는 MJCF 게인에 의존 — 실기 측정 전까지 S4 결론은 잠정
- γ derate 제외(D-8)로 abort 가 늘 수 있다 — S8 에서 측정
- D-3 이 검증에서 떨어지면 S3·S5 시간 경로 재작업
- 토크에서 도출한 보수적 가속 box (D-16) 가 받을 수 있는 공 속력을 낮출 수 있다 — S4.4 에서 함께 판정
- vision 지평이 짧으면 (현 예시 profile 0.5 s) 계획 가능한 포구 창이 줄어든다 — S3.6 요구 사양으로 sim profile 을 맞춘다
- 1차원 분해가 복잡한 경우를 놓치면 NLP 전환 (§8) 이 필요해 S6 재작업
