# 포구 평가·분석 도구의 판정 규칙

포구 sim 평가 도구가 낸 숫자를 **어떻게 읽는가** 의 규칙과 그 이유다. 실행 방법은 [integrated_bringup/docs/catching-sim-trials.md](../../catching-sim-trials.md) · [rtc_tools/README.md](../../../rtc_tools/README.md), 열 정의와 절차는 각 도구의 머리 docstring 이 갖는다. 도구마다 H3 하나씩 두며, 새 도구 절은 끝에 붙인다. `S*` · `E*` · `G*` · `D-*` 는 [ID_INDEX](../ID_INDEX.md) 와 L3 · L6 · L7 · L8 의 항목 번호다.

## integrated_bringup/tools

### 기록과 판정을 가른다

- `trial_results.json` 의 `final_outcome` 은 supervisor 가 **마지막으로 발행한 label** 이지 판정이 아니다. 새 cycle 이 시작되지 않은 투척에는 앞 투척의 label 이 남는다. 성공은 `catching_trials` 의 `truth_success` 로 센다.
- `summarize.py --unit` (N arm 기록) 은 **판정하지 않는다** (대조군 · 비열등 한계 · PASS/FAIL 없음). 판정 규칙(G-1)은 `--cf`/`--mpc` 형식만 갖는다.
- arm (search × segment × 공) 은 플래그나 디렉토리 이름이 아니라 **unit 의 `mirror.txt`** (컨트롤러에서 되읽은 값) 에서 읽는다. `conditions.txt` 의 `expect_*` 가 미러와 다르면 거부한다.
- `controller_bin_sha256` 이 unit 사이에 둘 이상이면 거부한다. 값 하나만 다른 unit 은 `--vary <미러>` 로 arm 에 넣는다.
- 끊긴 풀이(코어가 기한에 끊은 풀이)의 시간은 분포에 넣지 않고 개수와 비율로 낸다 ([planner_solves](#planner_solves)).

### 재시도와 host 부하

- 재시도는 **rig 실패**(`host_busy` · `estimator not activated` · `startup`)만 한다. 그 밖의 실패는 투척이 원인일 수 있으므로 실패한 자리에 남긴다.
- 성공률 판정 unit 은 `--host-watch abort` 로 돌리고, exit code 3 이면 같은 seed 로 unit 전체를 다시 돌린다. 시행 단위 무효 처리와 보충 투척은 모집단을 바꾸므로 하지 않는다 ([L8 §4.5](L8_bringup.md) · D-S8-17).
- `detections[].processes` 는 원인 후보일 뿐 판정이 아니다. 목록이 비어도 sim 이 느렸으면 부하다.

### `probe` dump 가 열의 뜻을 바꾼다

`PROBE_DUMP=1` unit 은 계획 포구점이 dump 의 지평 점과 일치할 때 realtime − steady 를 다시 잡는다(`refine_wall_offset`). 그래서 `tc_stamp_minus_tc_ms` · `tc_axis_source` · `tc_axis` 는 dump 없는 unit 과 같은 규칙의 값이 아니다. 짝지어 비교할 때는 `tc_axis_source` 를 먼저 본다.

### `docking_ident`

식별 값(통과 평면 · lateral 집합 · 속도 집합 · 폐쇄 창)의 판정 규칙(유지율, 5 %p 하락 한계)과 출하값은 [L6 §4.6](L6_hand.md), 절차와 격자는 `report.py` 머리가 갖는다. 출력은 전부 provisional 이다 (sim 의 손 흡수·반발은 실기 값이 아니다). 폐쇄 시각은 두 축으로 읽는다: 폐쇄 명령 → $s_{ent}$ 통과, 폐쇄 명령 → 원점 도착. 날릴 때의 `T_close_e2e` 가 이 축에 상수로 들어 있어 profile 값이 `selfcheck` 때와 다르면 상수 없는 꼴도 함께 적는다.

## rtc_tools

### 공통 규약

- **측정 도구다.** `NOT_EVALUATED(<사유>)` 는 입력이 없어 계산하지 않았다는 뜻이고 통과가 아니다. 모르는 값은 NaN 이며 0 으로 대신하지 않는다.
- **로봇 상수가 코드에 없다** (ARCH-1). 한계 · 프레임 · 게인은 `--config-dir` 의 로봇 profile (`_base.yaml` → `sim.yaml` → `--overlay`, CM 과 같은 키 단위 병합) 과 러너가 `run_meta.json` 에 남긴 컨트롤러 미러에서 읽는다.
- **판정 대상은 `catching_trials` 의 유효 시행이다.** 무효는 rig 실패뿐이고, 판정 실패 (plan 없음 · abort · 놓침) 는 유효한 실패다.
- 쌍 비교는 `(kind, seed, sample_idx)` 로 같은 투척을 짝짓고 짝 없는 시행은 따로 센다. 분위수는 nearest-rank 라 표본이 100 미만이면 p99 는 최댓값이다.

### plot_rtc_log: 로그 종류별 열과 figure

파일 이름과 열 지문으로 종류를 감지한다.

**`catching_diag.csv`** — 동적 포구 tick 레코드 (지문 `track_err_rad` + `ref_gamma`). 모든 tick 이 한 행이라 tick 간극은 드롭된 행이다.
- 패널은 그릴 것이 있을 때만 생긴다. `segment_*` 열이 없는 로그와 `closed_form` 로그 (열이 전부 0) 는 segment 패널이 없다. `decel_*` 옛 이름은 별칭 표 (`rtc_tools/utils/catching_keys.py`) 로 읽고, 옛·새 이름이 섞이면 거부한다.
- 명령 kinematics 패널: `q_cmd_*` 를 기록된 시간축으로 미분한 관절 최대 속도 · 가속 · jerk (5 행 box 평활, log 축, 1e-3 아래는 그리지 않는다 — 멈춘 명령은 반올림 잡음으로 미분된다).
- 통계: `mode: mpc` 는 구간을 따른 tick 을 "법칙이 돈 tick" 으로 세고 segment 블록 (따른 tick · 구간 수 · 사건과 거부의 이름별 수 · 전환 $\rho$ p50/p95/max) 을 낸다.

**`planner_events.csv`** — 계획기 스레드의 non-idle wake 당 한 행 (지문 `n_in_window` + `rank_error_budget`). 시간축은 `wake_ns` 기준 첫 행 이후 경과초다 (`t_relative_s` · `t_wall_ns` 없음).
- 내용: 후보 funnel (`n_in_window` / `n_ik` / `n_pass`), judgement-reject 히스토그램, rank-gate 실패 비트마스크 (§4.8 rollout gate 포함), `decision`, wake 의 `outcome` (`idle` · `no_input` · `published` · `superseded` · `held`), `recv_to_publish_ms` (publish 된 행만). `segment_outcome` · `segment_core_reason` · `segment_kind` 는 문자열 열이다.
- **코어가 기한에 끊은 풀이** (`segment_core_reason` == `deadline`) 는 solve 패널에서 빈 삼각형이고 통계 분포에 넣지 않는다 ([planner_solves](#planner_solves)). 보류된 교체의 첫 풀이 (`replacement_*`) 는 마름모다.
- `nlp` 의 가장 느린 후보 풀이는 wake 행으로 가를 수 없다: `nlp_rej_deadline` > 0 은 끊긴 풀이 · 기한 초과 풀이 · 늦게 끝난 wake 의 유효 후보를 모두 포함하므로 "끊겼을 수 있다" 고만 적는다.
- 통계의 `search_valid` 대 `plan_valid`: 구간 계획기는 plan 을 첫 구간과 함께만 내고, 채택 뒤 탐색이 낸 plan 은 `held` 로 기록만 한다.

**`dualarm_diag.csv`** — 다중 frame CLIK 의 tick 레코드 (지문 `clik_ran` + `<task>_err_lin`, `accel_*` 분기보다 먼저 판정).
- `tick` 은 CM loop counter 라 다른 컨트롤러가 활성인 동안에도 세므로 간극은 **떨어진 행이거나 이 컨트롤러가 돌지 않은 구간**이다. 간극 뒤 첫 행이 `reseeded` 이거나 시드 전 hold 면 돌지 않은 구간, 아니면 `DROPPED ROWS`. 선은 간극에서 끊는다.
- 과제 = `<x>_err_lin` · `<x>_err_ang` · `<x>_valid` 가 모두 있는 접두 `<x>` (헤더 순서 = 설정 순서), 관절 = `q_cmd_` 뒤.
- 값은 **계산한 tick 에서만** 그리고 센다 (나머지의 0 은 계산 안 한 0): 풀이 시간 · 반복 · status 는 `reached_solve` tick (입력이 거부된 호출은 `clik_ran` = 1 이지만 풀이 값이 없다), `track_err` 는 `clik_ran` = 1, 과제 열은 `<task>_valid` = 1. 목표 점선은 `goal_sequence` 가 0 이 아닌 값으로 바뀐 tick 만이다 (재시드의 0 복귀는 목표가 아니다).
- `<task>_err_*` 에는 서보 지연이 없고 `_task_pose` 의 cmd 대 meas 에는 지연이 그대로 들어 있다. `brake_*` mask 의 bit 는 모델 속도 index 라 관절 이름으로 풀지 않는다.

**그 밖의 종류**
- `compliance_diag` (지문 `x_tilde_`): **통계 전용, figure 없음** — envelope · freshness · bias 수치가 산출물이다.
- `wbc_log`: `<dev>_state.csv` 는 파일명으로 generic state_log 와 구분되지 않는다. `accel_*` 열 지문으로 열 fallback 단계에서 분류한다.
- **힘만 있는 손 (`sensor_layout.values_per_group: 0`)**: `<name>_raw_*` / `<name>_filt_*` 블록의 부재를 gate 로 단일 `fingertip_force.png` 를 만든다. `‖F‖` 는 **필터된 성분의 norm** 이다 (컨트롤러가 임계와 비교하는 값). delta-spike guard 오버레이는 `ft_*_force_guard_rejected` 열이 있는 세션만.
- **타이밍 CSV**: 세션 디렉토리는 분 해상도라 같은 분의 재기동이 같은 파일에 append 된다. `run_id` 가 여럿이면 마지막 런만 그린다 (파일 전체로 `n / span` 을 내면 어느 런에도 없던 레이트가 나온다).

### derive_accel_limits

토크 한계만 있을 때, 표본 $(q,\dot q)$ 마다 $\sum_j |M_{ij}| a_j \le \eta_\tau \tau_{max,i} - |g_i| - |c_i|$ 를 만족하는 최대 $a = s\,w$ 를 구해 (최악 부호 결합의 충분조건) 전 표본의 최소값을 상수 box 로 낸다. 판정식은 [L3 §4.3](L3_planner.md).
- 출력은 분석 산출물이다. 컨트롤러가 읽는 box 는 `catching/search_grid.yaml` 의 `robot.arm.qdd_max` 다. `--eta-tau` 는 기본값이 없다.
- 손 관절은 URDF 중립 · 속도 0 으로 고정하고 손 가속 결합은 provenance 에만 적는다.
- 퇴화 ($\tau_{dyn} \le 0$ 표본, 또는 $s^\star$ < `--min-accel`) 면 `adopted: false` 와 exit 2. 교차 검증 실패는 exit 3.
- 교차 검증: `--check rnea` (모든 부호 패턴에서 RNEA 토크 $\le \eta_\tau \tau_{max}$), `--check mujoco --mjcf <xml>` (`mj_inverse`, armature · damping 포함, 접촉 · 관절 한계 끔, actuator forcerange 이하).

### clock_phase · hand_close · vision_lane

**clock_phase** (D-3, S3.1a): 비율 (RTF) 이 아니라 시행별 clock 위상 오차 $\delta(t) = (\text{steady} - \text{steady}_0) - (\text{sim} - \text{sim}_0)$ 로 본다 ([L8 §4.5](L8_bringup.md)). 시행별 $\delta_{max} = \max|\delta|$ 와 max pause (한 step 의 $\Delta$wall $-$ $\Delta$sim 최대) 를 낸다. 창은 **비행 구간**이다.
- $\delta$ 는 판정이 아니라 **공변량**이다 (D-S8-4 (c)). `--eps-mm` 을 주면 그 $\varepsilon$ 에서 L8 §4.5 유효 조건을 만족하는 시행 비율을 "valid under eps — COVARIATE" 로 병기한다. 95 % 를 통과시키는 $\varepsilon$ 는 측정의 역산인 **제안값**이다.
- lane 의 누적 drop 을 그대로 보고한다 (drop 이 있으면 꼬리가 빠진 채 분포가 멀쩡해 보인다).

**hand_close** (S4.2): [L6 §4.2](L6_hand.md) 의 $\rho(t) = \min_{i\in C}\big((q_i - q_i^{pre}) s_i / |q_i^{cls} - q_i^{pre}|\big)$ 와 $T_{close,e2e}(\eta) = \inf\{t - t_{cmd} : \rho \ge \eta\}$ 를 `<hand>_state.csv` 에서 낸다. 시행은 command lane (`command_*` 가 바뀐 첫 행 = $t_{cmd}$) 으로 자른다.
- 프로파일과 `dt` 는 컨트롤러의 **읽기 전용 파라미터**에서 JSON sidecar 로 받는다 (출하 YAML 이 아니다). sidecar 에 `dt` 가 없으면 tick 축을 **신뢰 불가**로 찍는다 (추정한 dt 는 자기를 검사하지 못한다).
- 시간축 둘: `steady` (lock-step sim 에서는 호스트 스톨 포함) 와 `tick × dt` (sim 시간, 결정적). 차이가 스톨이다.
- 시행 구간은 `pre` / `close` / `other` 로 분류한다. 자기 travel 의 25 % 밖이면 `other` 이고, `other` 는 시행을 끝낼 수는 있어도 시작하지는 못한다.
- $\rho$ 는 **최소**다 (한 손가락만 늦어도 손 전체가 못 감싼 것). caging 집합은 프로파일이 정한다 (닫힐 때 펴지는 관절을 넣으면 진행으로 오독한다).
- `mean − T_close_lead` 는 폐쇄가 $t_c$ 뒤 어디에 끝나는가 — 폐쇄 창을 쓰는 계획기의 $\delta_0$ 다 (시퀀서는 $t_c - T_{close,lead}$ 에 닫는다). `T_close_lead` 는 sidecar 의 `T_close_lead_at_run`, 없으면 `T_close_e2e_at_run` ("falling back" 표시), 둘 다 없으면 "계산할 수 없음". $T_{close,e2e}$ 는 잰 폐쇄 시간이고 명령 시각을 정하지 않는다 (L6 §4.3). tick 양자화 `dt / sqrt(12)` 는 표준편차 옆에 적고 더하지 않는다.

**vision_lane** (S3.4, D-4): `prediction/trajectory` (PointCloud2) 를 **필드 이름으로** 디코딩한다 (이름 · offset · datatype · count · `point_step` · endianness 중 하나라도 다르면 거부). 발행 주기 · N · 지평 (TBD-VIS-04), `frame_id` (VIS-06), 측정 손실 뒤 `validity` (VIS-07), best_effort 대 reliable 구독 손실 (VIS-08) 과 $T_{det}$ 를 **측정만** 한다 (정책은 S5.2).
- `sim_estimator_node` 는 `debug.enabled_topics` 에 `prediction/trajectory` **만** 있으면 샘플을 기록하지 않아 발행이 0 건이다. 다른 debug 토픽을 하나 이상 같이 켠다.
- **$T_{det}$** (발사 → 첫 VALID 예측): 비행은 truth lane 의 **침묵**이 가른다 (`--flight-gap-s`, 기본 0.3 s). 두 축을 각각 한 시계 안에서 낸다 (D-2): `recv` (프로브 steady, 전송 포함 = 소비자 체감) 와 `stamp` (발행자 stamp, 전송 제외). 발사 전부터 VALID 이던 트랙 (유령, VIS-07) 은 검출로 세지 않는다. 검출 없는 비행은 NaN.
- `--dump` (S8-A, 기본 off): 예측 메시지마다 모든 지평 점과 `debug/innovation` (capture stamp) · `debug/nis` (stamp 없음 — 프로브 수신 시각만) 을 CSV 로 쓴다. G8-C2 (A⊥B) 의 입력이며 $\bar\nu$ 생산자가 아니다 (D-S8-7 (a)).

### catching_trials

`catching_sim_trials` 한 run 의 컨트롤러 세션 CSV, trials dir (`trial_results.json` + `run_meta.json` + 시행별 truth CSV), `sim_lanes:=true` 의 clock · 접촉 lane 을 시행당 한 행으로 잇는다. `--throws-file` 로 던진 시행은 `throw_id` 를 갖는다.

**시각 정렬 (`time_alignment`)**. sim-sync 에서 `t_relative_s` 는 `iteration × dt` 라 sim 시간축과 세션 상수 $c = \text{sim} - t_{rel}$ 만큼만 다르다. clock lane 이 있으면 lane 기준으로 놓는다: 발사 짝짓기는 `launch_wall_time` (REALTIME) 대 lane `steady_ns` (MONOTONIC) 의 상수 하나, $c$ 는 commit tick 의 `planner_events.csv` steady 시각과 직전 lane 행 sim 시간으로 구한 중앙값이다. `--probe-dump` 가 있으면 계획 `p_c` 가 snapshot 점과 일치하는 stamp 로 offset 을 다시 잰다. 러너의 `wall_t_relative_offset` 은 lane 이 없을 때만 쓴다 (`median_offset`).
- 행 열 `abort_in_window`: 그 창의 mode 경로에 HOLD → RETREAT 판정 tick 이 있는가, `ABORT_SAFE` tick 이 있는가 (`catching_decel --success hold` 가 읽는다).
- truth 파일은 `trial_results.json` 옆의 것이 먼저다. 첫 stamp 가 `launch_wall_time` 에서 5 s 넘게 떨어진 파일은 거부한다. truth 없이 유효인 시행은 `validity.valid_without_truth_file` 에 나열한다 (파이프라인 결함).

**무효 판정**. 무효는 rig 실패뿐이다. 모든 시행에 `invalid_reason` 한 개 (빈 문자열 = 유효) 를 우선순위대로 붙인다.

| 사유 | 조건 |
|---|---|
| `srv_refused` | 기록 `accepted` 거짓 |
| `not_launched` | accepted 이고 `n_truth_rows == 0` (필드가 없는 구 기록은 판정 안 함) |
| `controller_silent` | `wall_t_relative_offset` 없음, 또는 시행 창에 diag 행 0 |
| `lane_drop` | clock lane 에 짝지어진 발사 세그먼트 없음, 또는 시행 행에서 `dropped_total` 증가 |
| `sim_stall` | 시행 행 안 연속 `sim_time_sec` 간격 > 5 × nominal step (lane 전체 간격의 중앙값) |

그 외 (plan 없음 · abort · HAND_TIMEOUT · 순환 미종결 · Missed) 는 유효한 **실패**다. lane 규칙의 시행 행은 그 `launch_seq` 행 중 발사부터 시행 끝 (+ diag 창 margin) 까지다. `dropped_total` 은 누적이라 세그먼트 첫 행 바로 앞 행과 비교한다. clock lane 이 없으면 lane 규칙은 `NOT_EVALUATED`.

**요약 모집단**. `truth` · `medians` · `d3` · `g3d` · `gate_map` · G8-C3 · G7-B3 는 **유효 시행만** 센다. `truth` 에는 `lower_975` (Wilson $z$ = 1.96, 97.5 % 단측 하한) 와 `itt` (무효를 실패로 센 `n_total` 기준) 가 있다. `--floor` 를 주면 `truth.verdict` 는 PASS / FAIL (`lower_975 ≥ floor`) 이고, `--n-valid-target` 에 `n_valid` 가 모자라면 `INSUFFICIENT_N`.

**truth 성공** (G8-D, L8 §9). HOLD 끝 (첫 RETREAT tick) 부터 대기 자세 release (손 위상 RELEASE) 까지 모든 truth 샘플이 catch frame 에서 `--hold-radius-m` (기본: 프로파일의 공 지름) 안에 있다. 슈퍼바이저 판정 대비 혼동행렬을 낸다. 손 토크는 sim forcerange 클램프라 판정하지 않는다.

**$t_c$ 분해**. 첫 COMMITTED tick 에서 $t + $ `plan_t_c_s` 를 $t_c$ 로 잡고 `clik_mm` (`‖FK(q_cmd) − ref‖`) · `servo_mm` (`‖FK(q_meas(t_c)) − FK(q_cmd(t_c − T_lead))‖`) · `pred_mm` (`‖p_c − p_true(t_c)‖`) · `ref_vs_true_mm` · `total_mm` 을 낸다. `plan_gamma_f` 는 계획 $\gamma_f$ 다 (`ref_gamma` 는 DECEL 진입 tick 에 1.0 으로 뛴다). 공이 $t_c$ 전에 로봇에 맞으면 truth 를 첫 접촉에서 자르고 직전 0.25 s 이차 적합을 $t_c$ 로 외삽한다 (`truth_extrapolated_ms`).
- **`t_c` 열의 축 (`tc_axis`)**: `t_c` 열은 sim 축 tick 에 steady 로 센 잔여 시간을 더한 값이라, commit 과 포구 사이에 sim 이 벽시계보다 느리면 공이 `p_c` 에 닿는 순간에서 벗어난다. `tc_axis` = `ok` / `shifted` (stamp 축 $t_c$ 와의 차 `tc_stamp_minus_tc_ms` > `--tc-shift-max-ms`, 기본 5 ms) / `unknown` (판정 불가 — 맞다는 뜻이 아니다) / 빈 값 (commit 없음). `tc_axis_source` = `plan_point` (probe dump 의 계획 snapshot 점) 또는 `planner_wake` (계획 steady $t_c$ + lane 의 realtime − steady). 요약 `medians` 는 `shifted` 행을 빼고 낸다. 행 단위로 읽을 때는 `tc_axis` 를 함께 본다.
- **lead** (S8-B): `joint_cmd.lag.lead_enable` 이 켜지면 tick $t$ 의 `q_cmd` · `ref` 는 $t + T_{arm}$ 을 겨냥하므로 명령 쪽 (서보 · CLIK · `ref_vs_true`) 은 $t_c - T_{lead}$ tick 에서 읽는다. $T_{lead}$ 는 diag `t_arm_s` 열 → 러너 미러 (`joint_cmd.lag.*`) → 0 순이고, 열과 미러가 다르면 다른 세션으로 보고 거부한다. 같은 tick 의 `‖FK(q_meas) − FK(q_cmd)‖` 는 `cmd_meas_gap_mm` 로 기록만 한다 (lead on 에서 의도된 선행이 서보 악화로 읽히므로).
- **서보 지연 $\hat\tau$**: 관절별 1차 지연 최소제곱 ($q_{cmd} - q_{meas} \approx \tau\,\dot q_{meas}$, 움직이는 tick) + 시행 클러스터 부트스트랩 CI.
- **기준의 출처 (`ref_source`)**: 분해의 "기준" 은 그 시행의 법칙이 CLIK 에 준 것이다. lead tick 에 `segment_following` = 1 이고 `segment_p_d_*` 가 있으면 `segment` (`mode: mpc`), 아니고 `ref_valid` = 1 이면 `ref_x` (`soft_catch`, `closed_form`), 둘 다 아니면 `none` 이고 `clik_mm` · `ref_vs_true_mm` 는 NaN 이다 (법칙이 돌지 않은 tick 의 `ref_x` 는 0 벡터라 기준으로 읽으면 CLIK 오차가 원점 거리가 된다).
- **$t_c$ 의 명령 (`cmd_*`)**: FK(`q_cmd`) 만으로 lead tick 의 명령 속력 `cmd_speed_tc` · 속력 변화율 `cmd_dvdt_tc` (양수면 아직 가속 중) · 가속도 크기 `cmd_accel_tc`, 첫 APPROACH 부터 명령 속력이 0.02 m/s 를 넘는 첫 tick 까지 `cmd_hold_s` (`mpc` 는 첫 구간 node 0 을 기다린 시간, `closed_form` 은 ≈ 0), 거기서 lead tick 까지 `cmd_move_s`. 지연 있는 위치 서보는 가속 중인 명령에 $\tfrac12\tau^2 a$ 를 남기고 시간 lead 는 그것을 보상하지 않으므로 이 열이 `servo_mm` 의 원인 쪽 관측량이다. `cmd_hold_s` 를 뺀 넷은 `shifted` 행을 중앙값에서 뺀다.
- **segment lane (`segment_*`)**: 시행당 따른 구간 수와 사건 수 (`segment_admitted` · `_replaced` · `_switches` · `_gate_refused` · `_deferred` · `_aged`, 옛 `catch_box` 거부 `_workspace_refused` 는 MD-73 뒤 로그에서 0), 첫 전환 $\rho$ `segment_rho_first`, 첫 APPROACH → 첫 전환 `segment_wait_node0_ms`. 전환은 `switched` 와 `plan_switched` 를 함께 세고 `segment_admitted` 는 `admitted` 만 센다. **tick 이 아니라 사건을 센다** (대기 · `aged` 는 매 tick 반복되므로 연속 tick 을 하나로). 열이 없는 diag 는 전부 NaN, 열이 있는 `closed_form` 시행은 0 (ρ · 대기는 NaN).
- **catch frame 분해 (`cf_*` · `ent_*`)**: $p_{true}(t_c) - \mathrm{FK}(q_{meas}(t_c)) = [p_{true} - \mathrm{ref}] + [\mathrm{ref} - \mathrm{FK}(q_{cmd})] + [\mathrm{FK}(q_{cmd}) - \mathrm{FK}(q_{meas})]$ 의 네 항 (`tot` = `ref` + `clik` + `servo`) 을 `cf_<항>_{x,y,s}_mm` 로 낸다 ($x, y$ lateral, $s$ 접근축 +z, 공 쪽에서 본 부호라 `tot` 은 손 안 공 중심 좌표). **날 좌표**다: 계획이 겨냥한 점 (docking 풀이는 $s = s_{ent}$) 을 빼지 않는다.
  - 손의 포구 집합 (`catching.robot.hand.docking`; 없거나 TBD 면 NaN) 이 있으면 `*_lateral_margin_mm` ($\min_i(b_i - a_i\rho)$, 양수면 안) 과 **입구 평면 통과** `ent_*` 를 낸다. 통과는 $t_c \pm$ `arrival_window` 에서 자유비행 공이 $s_{ent}$ 를 내려 지나는 ($t_c$ 에 가장 가까운) 순간의 보간이고 `ent_cross_ms` = 통과 − $t_c$ 다 (docking 의 $t_c$ 는 계획된 통과 시각).
  - 이 열들도 `t_c` 열에서 읽으므로 `tc_stamp_minus_tc_ms` 가 크면 접근축 좌표에 공의 진행이 섞인다. 요약 `catch_frame`: commit 한 시행이 10 개 미만이면 `NOT_EVALUATED(n < 10)`.
- **마지막 구간의 wake (`last_seg_*`, `--clock-lane`)**: 팔은 RT 가 마지막으로 전환한 구간으로 포구에 닿고 그 풀이가 아는 공은 그 wake 만큼 낡았다. lead tick 에 따르던 `segment_seq` 를 **이 시행 안 · $t_c$ 앞에 그 번호를 게시한 `planner_events.csv` 행**과 짝짓는다 (`last_seg_kind` = `first` / `same` / `advance`).
  - wake 가 읽은 snapshot: 계획기의 `snapshot_sequence` 는 탐색을 돈 wake 만 적으므로 없으면 **wake 시각 + 6 ms 까지 tick 이 읽은 가장 새 snapshot** 을 쓴다. `last_seg_wake_join` = `planner` / `exact` / `ambiguous` (구간 안에 새 snapshot 도착 — 새 것을 준다) / `conflict` (계획기 기록을 읽은 tick 없음) / `none`. `last_seg_pred_age_ms` 는 컨트롤러가 받은 뒤 wake 까지 (diag `input_age_s`, 추정기 지연 불포함).
- **정지의 직선 이탈 (`stop_*`, `--clock-lane`)**: 포구 뒤 손이 공의 직선에서 벗어나는 정도. 정지의 끝은 따르던 구간이 마지막 노드를 지난 순간 (diag `segment_held` 가 처음 서는 tick) 이고 팔은 lead 하나 뒤 (`t_end`) 에 거기 있다. 직선은 그 구간 풀이가 적은 `segment_stop_line_*` 다. `stop_exec_*` 는 측정, `stop_plan_*` 는 풀이가 자기 해에 대해 적은 값이다 (그 구간 노드만이라 최댓값의 구간이 실행과 같지 않다). commit 이 없거나 `segment_held` 가 서지 않으면 NaN. 실행 열은 `shifted` 행을 중앙값에서 뺀다.
- **닫는 창의 경로 이탈 (`win_*`)**: 닫기 명령 시각은 이름뿐인 $t_c - T_{close,lead}$ 가 아니라 **로그에서 읽는다**: diag `hand_phase` 가 `CLOSE`(2) 로 들어서는 tick 중 $t_c$ 에 가장 가까운 것. 창은 $[t_{cmd}, t_{cmd} + T_{close,e2e}]$ (runner 미러 `hand.T_close_e2e`, 없으면 프로파일). `win_exec_dev_*` 는 창 안 측정 FK 가 공의 자유비행 **경로** (선분까지의 최단 거리이므로 경로를 따른 시간 오차는 벌하지 않는다) 에서 떨어진 거리다. 필요한 입력이 없거나 창이 기록보다 길면 NaN. 기준이 명령 tick 이므로 `shifted` 행도 중앙값에서 빼지 않는다.
- **그 wake 의 예측과 계획 (`pred_wake_*` · `plan_wake_*`, `--probe-dump`)**: `cf_ref_*` 를 $p_{true}(t_c) - \mathrm{ref} = [p_{true} - \hat p_{wake}(t_c)]$ (예측: 추정기 몫) $+ [\hat p_{wake}(t_c) - \mathrm{ref}]$ (계획: 풀이 몫) 로 가른다. `plan_wake_lateral_margin_mm` 은 **풀이가 본** 공의 margin 이라 docking 풀이에서는 양수인 것이 결합 자체의 검사다. `pred_wake_join` = `exact` / `missing` (키는 아는데 probe 가 받지 못함) / 빈 값 (키 없음).

**D-3 · RTF · tick overrun 공변량**. 무효 규칙이 아니다.
- D-3: `clock_phase` 로 $\delta_{max}$ · max pause · $\delta(t_{commit})$ · $\delta(t_c)$ 와 `--eps-mm` 별 유효 수를 낸다. 짝 없는 시행은 `unpaired_trials` (절반 미만이 짝지어지면 다른 run 으로 보고 거부). `--v-max` 는 기본값이 없고 없으면 `NOT_EVALUATED(v_max not given)`.
- RTF: `rtf_flight` (발사 ~ 첫 접촉 또는 sim 0.6 s) · `rtf_trial_min` (시행 끝까지 sim 0.25 s 창 중 최솟값).
- tick overrun: `cm_timing_log.csv` 의 시행 창 안 `t_total_us` > 제어 주기인 tick 수 (`tick_overrun_n`) 와 최대 `jitter_us`. `t_wall_ns` 는 이름과 달리 steady_clock 이라 clock lane 으로 잇는다. sim 모드 CM 은 `jitter_us` 를 0 으로 낸다.

**G8-B (`--eval-samples`)**. prediction 행 중 `time_ns` ≥ 발사 stamp 이고 **`time_ns + horizon_ns` ≤ 첫 접촉** (`t_first_impact`, 없으면 마지막 truth stamp) 인 것만 시행에 묶는다 (그 뒤 표본은 손 안의 · 리셋된 공과 비교한 값). 요약 `g8b`: 지평별 합산 평균 NEES ($\Sigma\,\mathrm{NEES} / \Sigma\,n$, 시행 평균의 평균이 아님) · 시행 부트스트랩 95 % CI · **CI ∋ 3 이면 PASS** · NaN > 10 % 이면 `NOT_EVALUATED` · coverage_95 ($\chi^2_3$ 0.95 = 7.815 이하 비율). `--eval-report` 는 `g8b_unrestricted` 로 판정 없이 옮긴다 (접촉 후 truth 포함).

**G8-C2 (`--probe-dump`)**. commit tick 마다 $A = p_{true}(t_c) - \hat p_{live}(t_c)$, $B = \hat p_{live}(t_c) - p_c$ (sim world; $p_c$ 가 계획 snapshot 점과 µm 단위로 일치하는 것이 프레임 검사). $t_c$ 는 계획 snapshot 에서 $p_c$ 인 점의 stamp + horizon (`c2_tc_source: plan_point`; 없으면 계획 steady $t_c$ 등으로 대체, `c2_tc_minus_tc_ms` 가 `t_c` 열과의 차). $\hat p_{live}$ 는 commit tick 의 diag `input_snapshot_sequence` · `input_generation` 을 dump 에 정확 결합 (`c2_join: exact`), 없으면 그 시각 이전 마지막 snapshot (`approx`). 요약 `c2`: $E|A+B|^2$ 대 $E|A|^2+E|B|^2$ 와 교차항 · `independence_test` ($n$ < 100 이면 `NOT_EVALUATED`).

**손-관절 캡처 witness**. diag 의 `hand_stalled_n` / `hand_effort_frac` / `hand_blocked_s` (연속 불통 지속 [s], 0 = 안 걸림) 에서 시행당 `outcome_source` (0 none · 1 fingertip · 2 hand · 3 both) 를 읽는다.
- HOLD 끝 tick 에 판정되지만 그 tick 은 이미 RETREAT 로 기록되므로 `outcome_source` 는 **첫 RETREAT 행**에서, `*_judge` 열은 **마지막 HOLD 행**에서 읽는다. RETREAT 가 ABORT_SAFE 등에서 들어왔으면 NaN.
- `tips_only_verdict` 는 손 witness 없이 손끝만 있었다면의 재구성이다 (`CAPTURED` + `outcome_source == 2` → `MISSED`).
- **`hand_blocked_s_judge` 는 판정이 비교한 값보다 정확히 한 tick 짧다**: `JudgeOutcome` 은 다음 tick 에서 `t_persist` 와 비교하므로 오프라인 재현은 `hand_blocked_s_judge + dt >= t_persist` 다 (`hand_persist_met` 열, `t_persist_s` 는 `catching.robot.hand.capture.t_persist`, TBD · 없음이면 NaN).
- $\rho_{min}$ / $\rho_{max}$ / `qd_tol` / `effort_frac_min` 캘리브레이션: `[t_hold_end − --hold-window-s, t_hold_end)` (기본 0.1, SSoT `Settings.hold_window_s`) 에서 caging 관절별 극값을 HOLD 끝에서 RETREAT 로 들어간 시행 (ABORT_SAFE 경유 제외) 만 `hand_hold_window.csv` 에 쓴다. `capture_would_fire` 는 `EvaluateHandCapture` (hand_capture.hpp) 의 오프라인 미러다. 입력이 빠져도 세션 분석은 계속되지만 `q_pre`/`q_close` 길이가 다르거나 비면 YAML 블록의 모순이라 `SystemExit`.

**그 밖의 요약 블록**
- `ref_saturated` 시행별 max streak (G8-C3): 빈도 분포만 기록한다. 게이트는 삭제됐다 (D-S8-16).
- G7-B3 충격량 상관 (`g7b3`): 예측 $\Delta p = m\,v_{rel}$ 대 측정 $\int F\,dt$ 의 OLS 기울기 · 절편 + 부트스트랩 95 % CI, Spearman $\rho$. 통과 임계는 없어 보고만 하고 $n$ < 50 이면 `NOT_EVALUATED`, 토크는 `NOT_EVALUATED(sim clamp)`.
- G3-D plan validity (`--clock-lane`): `planner_events.csv` 행을 시행의 `[발사, 첫 COMMITTED tick]` (커밋 못 하면 `[발사, 시행 끝]`) 창에 넣는다. `plan_valid_ratio` 의 분모는 창 안 **모든** 행이라 `mode: mpc` 에서는 두 번 낮게 읽힌다 (plan 은 첫 구간과 함께만 게시되고 따르는 중의 구간 풀이 wake 도 행을 낸다). 그래서 구간 열이 있는 로그는 **탐색** (`search_cycles` = outcome 이 `idle` · `no_input` 이 아닌 행, `search_valid_ratio`) 과 **쌍** (`pair_attempt_cycles` = `segment_kind` `first`, `pair_published_ratio`; `closed_form` 은 NaN) 을 따로 낸다. 시간축은 시행별 오프셋 `ClockLane.trial_offsets[idx]` 를 쓴다 (세션 중앙값은 RTF < 1 에서 드리프트한다). lane 이 없으면 `g3d` 키가 없다.
- 투척마다의 판정과 거부 사유 (`plan_verdict_window`): 창은 **투척의 비행** `[발사, 첫 충돌]` 이다 (바닥 · 로봇 · 손 어느 것이든 첫 충돌, 없으면 시행 끝 — 시행 끝까지 세면 가장 잦은 사유가 기록 길이를 탄다. 오프라인 탐색 지도의 창과 같다).
  - `plan_verdict`: `published` (유효 plan 이 한 번이라도 게시) · `withheld` (탐색은 찾았으나 게시 안 됨) · `no_plan` (탐색이 한 번도 못 찾음) · `no_search` (탐색한 wake 없음).
  - plan 을 못 낸 투척은 `plan_reject` (가장 잦은 사유, 동률이면 나중 것 — `catch_search_map` 과 같은 함수 `most_frequent_reason`) 와 `plan_reject_last` (마지막 wake) 를 낸다. 어느 쪽이 "그" 사유인지는 고르지 않는다 (앞쪽 wake 는 예측에서, 뒤쪽은 남은 시간에서 걸린다). 형태: `search:<이름>` · `segment:<outcome>[/<코어 사유>]` · `cycle:<outcome>[/<decision>]`.
- `--gate-map DIR`: `--dist` box 시행 (여섯 축 값을 가진 것 — reference/varied 는 검정 `None`) 마다 `catch_gate_map` 의 가장 가까운 grid 투척을 찾는다 (축별 grid 간격 중앙값으로 정규화한 거리, 단일 값 축 제외, 동률은 `throw_index` 작은 쪽). `map_open` = `seed_id` 에서 `reason_torque == "none"` 후보가 하나라도 있는 투척.
- 로봇 config 의 sim world ↔ model world 는 모델 frame `scene_world` 를 `frame_placement_in_model_world` 로 합성한다 (없거나 `provisional` 이면 거부, 옛 `catching.io.arm_base_frame` · `base_T_world` 도 거부).
- `segment_event` · `segment_refusal` 은 CSV 에 정수로 나오므로 이름표를 C++ enum 과 테스트로 대조한다 (enumerator 를 끼우거나 빼면 뒤 값이 밀린다).

### planner_solves

`planner_events.csv` 의 풀이 시간 (`segment_solve_us`) 을 읽는 규칙이다. `plot_rtc_log` 통계, `catching_trials` 의 `first_solve_cut`, `unit_report.py` 가 부른다.
- **규칙**: `segment_core_reason` 이 `deadline` 인 행이 코어가 기한에 끊은 풀이 (`cut_at_deadline`) 이고 그 시간은 끊긴 시각 — 풀이 시간의 하한일 뿐이다. `segment_outcome` 의 `budget` 으로 가리지 않는다: 기한이 없는 코어의 `budget` 과, 다른 사유 (`infeasible` 등) 로 끝나며 예산을 넘긴 풀이는 끝까지 푼 시간이다.
- `summarise_times` 는 끝난 풀이의 분포와 끊긴 풀이의 수 · 가장 이른 끊긴 시각을 나눠 낸다. `quantile_with_bound` 는 두 종류가 섞인 집합의 분위수와 "하한인가" 를 함께 돌려준다 (끊긴 값을 기록된 대로 넣은 분위수는 참 분위수의 하한이고, 가장 이른 끊긴 시각보다 작으면 참값이다).
- `nlp_summary` 는 기한 사유로 거부한 wake 를 따로 세고 끊겼다고 단정하지 않는다. `first_wake_summary` 는 투척 `track_generation` 마다 정지에서 출발한 가장 이른 첫 풀이를 센다. 열 이름 접미는 C++ 이름 함수와 테스트로 대조한다.

### catching_pool

arm 마다 여러 unit 을 합쳐 판정한다. 입력은 unit 마다 `catching_trials` 출력 dir 이다. `decel_*` 이름의 옛 출력과 `invalid_reason` 열이 없는 CSV 는 거부한다.
- **보충 절단** (D-S8-16 ①b): CT_DIR 은 준 순서대로 (사전 선언 seed 순, 보충 unit 은 마지막), 시행은 `idx` 순으로 읽는다. 누적 유효 수가 `--n-valid-target` (기본 200) 에 닿는 시행까지 포함하고 그 뒤는 `beyond_target` 으로 유효 · ITT 양쪽에서 뺀다. 전부 합쳐도 모자라면 `INSUFFICIENT_N`.
- **McNemar** (arm 2 개 이상): 두 arm 에서 모두 포함 · 유효인 `(seed, idx)` 쌍만으로 A 성공 ∧ B 실패 ($b$) · A 실패 ∧ B 성공 ($c$) · 정확 이항 양측 $p$. seed 없는 유효 행 수는 따로 적는다. 한 arm 에 같은 `(seed, idx)` 가 두 번 나오면 거부한다.
- **D-3 S3.1b** (D-S8-16 ⑤): arm 들의 clock 공변량 시행 수 (포함 · 유효 · 유한 `delta_max_ms`) 만으로 ≥ 200 (`met`) 을 판정한다. `--extra-d3` 요약의 `d3.paired` 는 모집단이 달라 나열만 하고 더하지 않으며, 요약 JSON 의 분위수도 합칠 수 없어 출처별로 적는다.
- **G8-B · G8-C2 · RTF · 시각 정렬**: 합산 시행의 행 열에서 재계산한다 (G8-C2 는 시행당 $(A, B)$ 하나라 표본 = 시행 클러스터). unit 별 `time_alignment` 를 옮겨 적는다. D-3 · tick overrun · G8-C3 · G7-B3 · gate-map 도 세션과 같은 함수로 합산 시행에서 다시 계산한다.

### catching_hand_near

`catching_sim_trials --dist hand_*` 로 던진 unit (손 반경 ≤ 0.2 m 로 오는 공을 얼마나 빠르게까지 받는가, S8-F) 에서 **plan 이 있었는가 (COMMIT)** 와 **plan 이 있을 때 잡았는가 (CATCH)** 를 따로 적합한다. 성공 하나로 뭉치면 "못 도달" 과 "도달했는데 놓침" 이 섞인다.
- **조준 검증**: 러너의 `aim_error_m` 최대 · p50 과 `--aim-tol-mm` (기본 2) 초과 목록, `model_rms_m` (첫 접촉 전 truth 대 모델) 최대 · 초과 목록. `aim_error_m` 은 조준에 쓴 공 파라미터로 다시 적분한 값이라 항력 법칙이 틀려도 0 이므로 `model_rms_m` 이 그것을 잡는다. 하나라도 넘으면 rc 1 (던진 곳이 설계와 다르다). `aim_error_m` 이 한 시행에도 없으면 `pass: null` ("no data") 이고 rc 1.
- **grid arm** (`hand_cliff` · `hand_lob`): 속력 하나에 대한 로지스틱의 $v_{50}$ + 시행 부트스트랩 CI (유한 복제 20 미만이면 NaN).
- **LHS arm** (`hand_lhs`): 설계 인자 $[1, v, r, r^2, T-0.7, \alpha, v r, \sin\psi, \cos\psi]$ 의 로지스틱 GLM (IRLS, 기울기에 1e-4 ridge — 절벽은 완전 분리라 ridge 없이 발산) 을 commit · catch | commit · 전체 성공에 적합한다. $r$ 별 $v_{50}(r)$ 과 두 단계 곱 $P(\text{plan})P(\text{catch}|\text{plan}) = 0.5$ 의 $v_{50}$ 을 `v50_map.csv` 로 낸다. pooled Wilson (`wilson_cells.csv`) 이 모델 없는 검산이다. 도달량 (Δz · 입사각) 은 도출량이라 GLM 에 넣지 않는다.
- **$v_{rel}$**: commit 시행의 **측정** 접촉 상대속도 (`contact_v_rel`) 하나에 대한 로지스틱 → `v_rel50` ([L6 §4.5](L6_hand.md) fly-in 허용량의 폐루프 대응값).
- **A/B** (`--ab A B`): 두 arm 라벨 (`run_meta.json` 의 `arm`) 을 `(kind, seed, sample_idx)` 로 짝지어 (`kind` 없이는 같은 seed 의 cliff 와 lhs 가 겹친다) catch 와 commit 각각 McNemar 정확 검정.
- **planner_events**: `rej_workspace` 는 탐색에 `catch_box` 가 있던 세션에만 있고 그 뒤 세션에서는 0 이다 (L3 §4.9).

### catching_arm_budget

"팔이 못 간다" 를 네 층 — **P** 플랜트 (서보 지연 $\tau$ · 토크 여유 · 도달 속도) · **R** L4 참조 ($\omega$ 수렴, `a_max` / `v_max` 포화) · **C** CLIK (속도 box = device 정격, 가속 제약) · **B** 계획기 (도달시간이 판정하는 D-16 box; 결정 K 이후 CLIK 이 실행하는 것과 다르다) — 중 어디가 묶였는지 unit · 시행 단위로 나눈다.
- **잔여 분해**: 참조 자기 오차 ‖e‖ 를 commit 과 마지막 활성 tick ($t_c - T_{arm}$) 에서 읽어 `catching_trials` 의 `ref_vs_true` 옆에 놓는다 — 차이가 예측 몫 (하한 `pred_live_lb`). 임계감쇠 2차 참조의 이론 잔여 $(1+\omega T)e^{-\omega T}$ 를 함께 찍어 $\omega T$ 산술을 포화 · 추종과 구별한다.
- **C**: 실행된 관절 가속 **envelope** (활성 tick 의 5-tick 평균 $|\ddot q_{cmd}|$ p95 / max), 속도 box 적중 비율, 계획기 box 초과 tick 비율. **P**: device lane 이 없으면 토크 사용률은 NaN.
- **B**: 같은 운동 (commit → 마지막 활성 tick 의 $\Delta q$, 초기 속도 포함) 의 도달시간을 D-16 box 와 실행 envelope 로 **두 번** 재 ([L3 §4.3](L3_planner.md) 닫힌해) commit 시점 가용 lead 와 비교한다 — box 의 비관이 숫자가 된다.
- **한계의 출처** (ARCH-1): 관절은 diag `q_cmd_*`, 정격은 프로파일 device 명세, $\omega$ / `a_max` / `v_max` / $\eta_v$ / `planner.search.grid.time.margin` / box 는 러너가 `run_meta.json` 에 남긴 컨트롤러 **미러** (TBD 잎도 컨트롤러가 실행한 기본값으로 낸다). 없으면 컨트롤러 YAML → `sim.yaml` 의 `<controller>.catching` → `--overlay` 순으로 합성하고 `budget.source` 로 보고한다.
- `--write-envelope-box`: unit 들의 envelope p95 를 관절별 max 로 모아 sim override 조각 (`robot.arm.qdd_max` + `qdd_provisional: true`) 으로 쓴다. 토크 도출이 아니라 실행값이므로 sim 전용이다.

### catching_decel

포구 뒤 팔을 세우는 방식 (v1 closed-form 가상 목표, MPC 궤적의 꼬리) 을 같은 잣대로 잰다.
- **창은 mode 가 아니라 손의 task pose 가 정한다**: 시행의 첫 `DECEL` tick 부터 측정 catch frame 이 정지한 첫 tick 까지. 정지는 선속도 < `--rest-speed` (0.02 m/s) 이고 접근축 각속도 < `--rest-axis-rate` (0.2 rad/s) 인 상태가 `--rest-s` (0.05 s) 이어지는 첫 구간의 첫 tick. 첫 `RETREAT` tick 을 넘지 않고, 그 전에 멈추지 못하면 `rest_reached = false`. mode 길이 (`mode_decel_s`) 는 참고값이다 (v1 은 가상 목표가 멈출 때 `DECEL` 을 떠나지만 팔은 서보 지연만큼 더 움직인다).
- **관절이 아니라 task pose 인 이유**: null-space 운동이 task pose 정지 뒤에도 관절을 움직여, 관절 속도로 판정하면 정지가 아니라 `HOLD` 의 길이를 읽는다. 접근축 둘레 roll 은 null space 라 판정에 넣지 않는다. 관절 기준 `joint_rest_reached` 와 roll 포함 `angular_speed_*` 는 참고값.
- **가속 · jerk 피크**: 창 안 관절 · tick 최댓값, 명령과 측정 각각 (`--smooth-ticks` 기본 5 box 평균). v1 기준 가속은 진입과 정지에서 계단이라 평활하지 않은 jerk 는 tick 길이의 함수다 — `jerk_cmd_raw_peak` 는 참고값. 닫힌식 정지 거리 $\|\dot x_s\|^2/(2 a_{dec})$ 도 참고값이다.
- **한계 여유**: 위치 한계까지 최소 거리, $|\dot q_{meas}|/\dot q_{max}$ · $|\text{effort}|/\tau_{max}$ 최댓값. 위반 = 여유 < 0 또는 비 > 1 인 시행 수. 정격은 `catching_arm_budget._device_limits` 와 같은 합성. 넷 중 하나라도 없으면 거부한다.
- **요약**: `DECEL` 에 들어가지 않은 시행은 지표에서 빠지고 성공률 분모에는 남는다 (abort 는 실패). 두 번째 `DECEL` 구간은 세지 않는다. `pooled` 는 위치 인자 unit 만 합치고, `--a`/`--b` 는 `pooled_a` · `pooled_b` 를 내며 합친 `pooled` 는 `--same-arm` (같은 arm 의 복제) 일 때만 낸다.
- **`--a` / `--b`**: `(kind, seed, sample_idx)` 로 짝지은 truth 성공 2×2 표, 불일치율 $\psi$, McNemar 정확 검정, paired 단측 비열등 검정이 요구하는 쌍 수 $(z_\alpha + z_\beta)^2(\psi - d^2)/(\delta + d)^2$ 를 `--margin` 마다 $\hat\psi$ 와 $\psi$ 상한에서 낸다 (설계: [mpc_multiframe_clik_formulation](mpc_multiframe_clik_formulation.md) §6.5). 공통 throw 가 없거나 불일치 쌍이 없으면 쌍 수는 `null` 이다 (0 은 "쌍이 필요 없다" 로 읽힌다).
- **성공 정의 `--success`**: `truth` (기본, `truth_success`) 또는 `hold` = `truth_success` ∧ `hold_verdict` ∧ ¬`abort_in_window`. 두 열은 `catching_trials` 가 같은 시행 창에서 내므로 판정이 창 하나에 선다. 두 열이 없는 옛 평가에 `hold` 를 주면 거부한다.
- **비열등 검정** (게이트 G-1): `pairs.noninferiority` 에 `--margin` 마다 Tango (1998) score 검정 $H_0: p_b - p_a \le -\text{margin}$ — $Z = (x_b - x_a - n\delta_0)/\sqrt{n[2\tilde q_a + \delta_0(1-\delta_0)]}$, $\tilde q_a$ 는 경계의 제한 MLE (2 차식의 큰 근), $p = 1 - \Phi(Z)$, 비열등 ⇔ $p$ < `--alpha` (단측). $\hat d$ 와 같은 수준의 Wald 구간 (`paired_difference`) 은 margin 과 무관해 `pairs.wald` 에 한 번만 낸다.
  - 검정력은 삼항 분포를 전부 열거한 정확값이다 (`--design-psi` · `--design-n`, 없으면 관측 $\hat\psi$ 와 유효 쌍 수; `--power-diff` 의 참 차이마다, $|d| > \psi$ 인 칸은 `null`). `size` 는 $\hat\psi$ 의 $H_0$ 경계에서의 정확 기각률. `--alpha` ∈ (0, 0.5), `--design-psi` ∈ [0, 1], `--design-n` ≥ 1 이 아니면 unit 을 읽기 전에 거부한다.

### catching_grid_sweep

vision 예측 격자 (horizon × 점 수) 를 조건마다 바꾸고 같은 투척 (같은 seed) 을 던진 unit 들을 조건 = arm 으로 묶어 비교한다. 로봇마다 따로 돌린다.
- **조건은 unit 이 말한다**: 컨트롤러 미러의 `prediction.dt_expected` · `io.n_min` · `planner.search.grid.slice.dt` 를 arm 마다 옮겨 적고 한 arm 의 unit 끼리 다르면 거부한다. 세 키가 없는 unit 은 거부하며 `--allow-unknown-grid` 면 `grid_known: false` 로 받는다. 받은 점 수는 새 snapshot 을 받은 tick 의 `input_n` 최빈값이다.
- **메시지 주기**는 새 snapshot 사이 간격이며 `--flight-gap-s` (기본 0.5 s) 보다 긴 간격은 뺀다. 추정기의 발행 주기가 아니라 **컨트롤러가 받은** 주기다. 크기 = `point_step` 384 B × 점 수.
- **오차**: `pred_mm` · `total_mm` 은 `shifted` 시행을 빼고, `contact_v_rel` 과 `approach_plan_switches` 분포는 유효 시행 전부를 쓴다. `rtf_trial_min` < `--rtf-min` (0.95) 인 시행 수와 값이 없는 시행 수 (`rtf_unknown`, 모르는 것은 통과가 아니다) 는 세기만 한다.
- **계획기**: 창 안 후보가 있던 주기 (`n_in_window` > 0) 의 `search_us` 분포. `--budget-s` 를 주면 p99 가 넘는 arm 을 표시한다. 파일이 없는 unit 은 거부하지 않고 `planner_events_missing` 으로 센다.
- **비교**: `(kind, seed, sample_idx)` 짝 2×2, paired 차이 $p_B - p_A$ 와 Wald 95 % 구간 ($((b+c) - (c-b)^2/n)/n^2$), McNemar 정확 $p$, family 안의 Holm 보정 $p$. `--ref` 대비 비교가 한 family, `--pair` 가 다른 family 다.

### catching_capture_set

`mpc_docking` ([ball_catching_inverse_dynamics_mpc](ball_catching_inverse_dynamics_mpc.md) §6 · §8.4 · §17.6–§17.8) 이 받는 손 · 포획 쪽 값 — 통과 평면 $s_{ent}$, lateral 집합 $\mathcal C_\perp$, 속도 집합 $\mathcal V_{cap}$, 폐쇄 창 — 을 fly-in 판정에서 계산하는 **순수 함수**다 (CLI 없음). 판정 rig 는 `integrated_bringup/tools/docking_ident/`.
- **판정은 격자의 셀에 속한다.** 셀의 시행 전부가 유지돼야 그 셀이 유지다 (`held_cells`). 구간의 끝은 모두 셀의 가장자리다.
- **상자** (`held_boxes` · `box_by_min_width`): (closing speed, 폐쇄 시각) 지도에서 전부 유지된 직사각형. 폐쇄 시각 폭이 요청값 이상인 것 가운데 closing speed 상한이 가장 큰 것, 같으면 면적이 큰 것.
- **폐쇄 시각의 축이 둘이다.** 유지 여부는 공이 **원점**에 닿는 시각 ($\delta^O$), 코어의 창은 $s = s_{ent}$ **통과** ($\delta = \delta^O + s_{ent}/c$) 기준이다. 코어는 모든 속도에 창 하나를 쓰므로 상자에서 얻는 창은 $s_{ent}(1/c_{lo} - 1/c_{hi})$ 만큼 좁다 (`entrance_window`, 빈 창도 그대로 돌려준다).
- **lateral 다각형** (`capture_polygon`): 유지된 셀 안의 가장 큰 격자점 중심 원에서 시작해 방향 고정 8 면을 한 칸씩 밀어 넓힌 볼록 다각형 (코어의 `face_a` · `face_b`). 재지 않은 곳은 유지가 아니다.
- **통과 평면** (`ApproachTable.entrance`): preshape 손의 정적 접촉 격자에서 lateral 집합의 모든 점을 지나는 모든 기울기의 직선이 무접촉인 가장 낮은 높이. 둘러싼 열 중 하나라도 닿으면 닿은 것으로 하고, lateral 격자를 벗어난 직선은 닿은 것으로 세되 `scan_limited` 로 표시한다.
- **옆 속도의 상한** (`largest_held_radius`): 속도 고리의 유지율을 옆 속도 없는 기준 고리와 견주어 허용 폭보다 더 낮지 않으면 통과, 통과가 이어지는 마지막 고리의 바깥 가장자리가 상한이다 (시행 하나의 판정이 집합 안에서도 확실하지 않아 "고리의 시행 전부 유지" 로는 기준 고리도 통과하지 못한다). 검증은 `clopper_pearson_lower` (유지율의 단측 정확 하한).

### catching_wait_pose_search

`catch_speed_budget` 이 **수락된** 포구 자세의 속력 예산을 답한다면, 이 도구는 공이 보이기 전의 **대기 자세** (`planner.wait_pose`, IK seed) 를 바꾸면 방향 속력 상한이 얼마나 오르는가를 답한다. 목적함수는 **자기 접근축 기준** $\hat v(q) = -\mathrm{axis}(q)$ (axis = catch frame +z, model world) 다: 손-근처 투척 (S8-F) 생성기 (`catchability_map.aim_at_hand`) 가 공을 대기 자세 자신의 palm normal 로 조준하기 때문이다.
- **목적** `--objective`: 기본 `dls` = `directional_speed_dls` (런타임 `DirectionalSpeedMax` 가 $\gamma$ 창에 넣는 값), `lp` = `directional_speed_lp` (LP 물리 상한), `robust`. 두 값은 행마다 내고 DLS 는 LP 를 넘지 않는다.
- **`robust`**: 한 점의 정적 DLS 최대값은 $j_w$ 특이점 옆의 바늘 봉우리가 되기 쉽고 계획기의 실제 IK 포구 자세는 거기서 떨어져 속력이 훨씬 낮다. 그래서 **이웃**의 값이다: `--robust-samples` (기본 32) 개의 관절 섭동 $|\delta|_\infty \le$ `--robust-eps-rad` (기본 0.1) 에 대해 접근축 방향 DLS 를 재고 그 p10 을 명목 LP 로 cap 한 값 (`v_dir_robust`). 섭동 집합은 탐색당 한 번 뽑아 모든 후보에 같은 오프셋을 쓴다. `sigma_min` (5 행 $[J_p; J_w]$ 의 최소 특이값, 단위가 섞인 조건수 지표) 에 `--min-sigma` 제약이 걸린다. IK 수락 · 충돌은 보지 않는다.
- **한계의 출처** (ARCH-1): **탐색 관절 상자는 런타임이 대기 자세를 받아 주는 상자** (`joint_limits.position_lower/upper` 를 `catching.robot.arm.limit_margin` 만큼 안으로 당기고 URDF 한계와 교집합) 이고, 속도 box 는 `catching_arm_budget._device_limits` 와 같은 합성, $\eta_v$ · `planner.wait_pose` 는 catching 컨트롤러 YAML 이다.
- **정련**: 벌점(soft) Nelder–Mead 라 국소해가 하드 제약 (`--radius-m` · `--axis-tol-deg` · `--box` · `--min-z`) 을 벗어나면 제약을 만족하는 raw 표본으로 되돌린다 — 보고되는 모든 행이 제약을 어기지 않는 것은 이 fallback 의 보장이다. 참조 행은 `--box` 를 만족하지 않아도 비교 기준으로 실린다.
- **자체 검증 (fail-closed)**: 모든 보고 행이 FK($q$) 재계산 위치가 보고값과 1e-9 m 이내이고, URDF 관절 한계 안이며, `v_dir_dls` 가 `v_dir_lp` 를 상대 1e-3 + 1e-6 m/s 넘게 초과하지 않아야 하고 (탐색 중 그런 후보는 "DLS 아티팩트" 로 버린다), `v_dir_robust` 도 LP 를 넘지 않아야 한다. 하나라도 어기면 아무것도 쓰지 않고 종료한다.
- **LP 의 의미**: 접근축 회전 속도를 0 으로 고정할 뿐 — 그 자세를 유지하며 이동이 가능한지, 충돌이 없는지, IK/gate-map 이 받아줄지는 보지 않는다 (kinematic 상한). 나머지는 sim smoke 가 본다.

### catchability_map

투척 grid → 항력 비행 → 포구 후보 → **C++ judge** → 집계 · 제안 · 플롯. 판정은 재구현하지 않고 `catch_pose_ik_batch` (런타임 계획기와 같은 `CatchPoseIk::Solve`, 같은 YAML 키, [L3 §4.2](L3_planner.md)) 를 CSV 로 호출한다.
- **headline 은 seed 하나의 수치다.** `--seed` 를 여러 개 줘도 로봇은 한 자세에서 기다리므로 `accepted_throws` · `throw_region.yaml` · `throw_summary.csv` · 사유 히스토그램 · $w/\theta$ 분포 · $\varepsilon$ 경계 · 플롯은 전부 seed 순위 1위 (`rank_wait_pose_seeds` best) 하나를 기술하고 그 id 와 관절 벡터가 `results.headline_seed` · `throw_region.wait_pose_seed_id` / `wait_pose_q` 에 적힌다. "어느 seed 든 하나라도 받으면 accepted" 인 합집합은 `accepted_throws_any_seed` 라는 **이름으로만** 남고 coverage 가 아니다. 비율의 분모는 **전체 격자** (`accepted_fraction_denominator`) 이고 후보가 judge 까지 간 throw 수는 `throws_with_candidates`. `summarize_throws` · `propose_throw_region` 은 여러 seed 의 행을 `seed_id` 없이 받으면 거부한다.
- **seed 비교**는 포구 가능 throw 비율 내림차순, 동률이면 평균 $\log w_5$. 서로 다른 throw 집합은 비교를 거부한다. runner-up 의 coverage 차이가 대기 자세 민감도다. `throw_summary.csv` 는 후보 0 개인 throw 도 `candidates=0` · `accepted=0` 으로 낸다.
- **`--throws-file <list.json>`**: grid 대신 `catching_throw_list/1` (SIM WORLD) 의 투척을 파일 순서대로 평가한다. `throw_id` 가 `throw_index` 로 나가 `catch_gate_map --map-dir` 이 이어받는다. 프레임 변환이 없으므로 리스트를 만든 쪽과 같은 `--robot-config` 층을 준다. $\omega$ ≠ 0 투척은 거부한다 (스핀 항 없음). grid 축 인자와 같이 주면 오류. `throw_summary.csv` 의 6 축은 항목이 6 개 다 있으면 그대로, 하나도 없으면 arm base 기준 유도, 일부만 있으면 오류.
- **judge 의 프레임은 arm base 가 아니라 Pinocchio MODEL WORLD (URDF 모델 root) 다.** 모델 root 가 `base_link` 이고 CLIK `base_frame` 이 `base` 로 z 축 180° 차이인 로봇이 있다 ([L3 §4.2](L3_planner.md) frame 규약). world 좌표를 그대로 넣으면 x · y 가 뒤집혀 "공이 등 뒤에서 온다" — 그런데 수치는 전부 그럴듯하다. 그래서 `--robot-config` 의 `urdf.extra_frames.scene_world` 를 world 로 읽는다: `parent` 가 arm base frame, 배치 ($p_{parent} = R(rpy)\,p_{world} + xyz$) 가 `base_T_world`, `model_world_T_base` 는 parent 의 FK 로 **합성**한다 (vision lane 과 같은 합성). 없거나 `provisional` 이면 판정 전에 거부하고, 손으로 world 를 적던 `--world-yaw-deg` · `--world-translation-m` · `--arm-base-frame` 도 거부한다. 쓴 frame 은 `provenance.yaml` 의 `frames` 에 남고 `catch_gate_map` · `catch_speed_budget` 이 거기서 읽는다.
- **출하 `urdf.sub_models.<arm>` 은 flange 에서 끝나 catch frame 이 그 sub-model 에 없다.** `write_model_config` 가 arm root → catch frame parent 의 sub-model `arm_catch` 를 추가 선언한다 (catch frame FK · 팔 관절 Jacobian 은 정확하다). 로봇 config 가 `urdf.sub_models.<arm>_catch` 를 이미 선언하면 이름 그대로 쓰되 root/tip 이 어긋나면 거부한다 (지도와 런타임 계획기가 다른 모델에서 판정하지 않게, G3-I).
- **shard 재사용**: **행 / ID 가 맞고 입력 fingerprint sidecar 가 일치할 때만** 재사용한다 (`shard_is_reusable`) — 후보 id 는 실행마다 `0..N-1` 이라 id 만으로는 `--params` · `--seed` · config · 격자를 바꿔 재실행해도 옛 verdict 가 재생된다. fingerprint 는 입력 파일 sha256, sub-model · catch frame 이름, judge 실행파일의 정체 (경로 + 크기 + mtime, 내용 해시가 아니다 — 링크된 공유 라이브러리 변경은 못 잡으므로 그때는 `shards*/` 를 지운다). 비정상 종료 · 짧은 CSV 는 stderr 를 달아 raise 한다.
- **`q*` 열이 비는 것은 세 번째 상태다** (pose 없음 ≠ q = 0). judge 는 `none` / `below_manip_min` / `rank_deficient` 에만 pose 를 쓰므로 나머지 사유는 `q*` 가 공란이고 파서는 `None` 으로 구분하며, 절반만 찬 행은 거부한다.
- **$w_5$ · $w_6$ 은 따로 보고한다** (같은 $q^\star$ 의 두 측정이나 차원이 달라 pooling 하지 않는다). $\theta$ 분포는 `fraction_below` 와 `fraction_near_limit` 을 둘 다 낸다 ("$\alpha_{max}$ 의 몇 % 안" 이 양쪽으로 읽혀 결론이 뒤집힌다). 접근축 cone 이 binding 하는지는 `fraction_near_limit` 이 답한다.
- **`alpha_max` 는 judge 가 실제로 적용한 값이다** (`resolve_alpha_max`). `--params` 가 `planner.search.grid.ik.alpha_max` 를 주면 그 값이다. 파일 형태는 judge loader 와 같은 세 가지 (root 의 `catching:` map / root 에 `planner:` 가 있는 catching tree / `<controller_name>: {catching: ...}`) 만 받고 그 밖은 에러다. `include:` 로 나뉜 컨트롤러 config 는 합성되지 않으므로 `controllers/catching/search_grid.yaml` 조각을 넘긴다. `--alpha-max-rad` 는 기본값이 없고, params 가 값을 주는데 다른 값을 넘기면 에러 (진실의 출처가 둘), 같으면 허용, params 미지정일 때만 단독으로 쓰인다. 둘 다 없으면 judge in-code 기본값 0.26 의 미러를 쓰고 provenance 에 미러임을 적는다.
- **$\varepsilon$ 경계 개수**: 받아들여진 후보의 $p_c$ 를 축별 $\pm\varepsilon$ 로 옮긴 6 사본을 **다시 judge 에 넣어** 하나라도 거부되면 경계 위다 ($w_5$ 여유 같은 대용값 추정이 아니다).
- **힘 법칙은 sim 과 같다**: 중력 + 이차 항력 $-\tfrac12\rho C_d A|v|v$ (`ComputeProjectileBallAeroForce`). **각속도 0 가정**이라 Magnus 항이 없다 (spin 을 실어 발사한 궤적은 이 모델과 다르다).
- **공기밀도 · 항력계수 · 반지름 · 질량은 기본값이 없다.** $C_d$ · $\rho$ 는 C++ `constexpr` preset 에만 있고 YAML 로 노출되지 않아, python 에 박으면 preset 이 바뀌어도 아무것도 실패하지 않는다. 호출자가 값과 **출처 라벨 (`file:line`)** 을 함께 넘기고 provenance 가 기록한다.
- **프레임 변환은 인자로만 받는다.** `base` / `base_link` 를 잘못 고르면 downrange 부호만 뒤집힌다. 같은 $q$ 에서 MuJoCo FK ↔ Pinocchio FK 로 확정한 변환을 호출자가 넘긴다.
- **손 근처 투척 (S8-F)** `aim_at_hand`: 도착 상태 (목표점 = $p_c$ + 접근축에 수직한 평면의 오프셋, 도착 속도 = 접근축의 연직면에서 $\alpha$ 만큼 돌린 방향) 에서 같은 힘 법칙을 **RK4 로 역적분**해 릴리스 상태와 도출량을 낸다 (shooting 없이 결정론). 항력을 빼면 5 m/s · 0.8 s 에서 손을 약 0.1 m 비껴간다. 공 표면이 `floor_z_m` 아래로 가는 비행은 `ValueError`. `aim_check_from_truth` 는 truth 첫 표본에서 모델을 적분해 통과 오차를 낸다 (truth 는 손에서 끊기므로 표본만으로는 통과를 읽을 수 없다; `free_flight_prefix` 가 속도 도약 > max(0.3 m/s, 3 g Δt) 로 첫 접촉을 잘라 그 앞 표본과 모델의 RMS 를 함께 낸다).

### catch_speed_budget

`catchability_map` 이 "팔이 그 자세에 갈 수 있는가" 라면 이 도구는 수락 후보에 대해 "거기서 공과 함께 움직일 수 있는가" 를 답한다. 계획기 $\gamma$ 창 ([L3 §4.5](L3_planner.md)) 은 $\|v\| + \text{margin} \le v_{arm} + v_{rel}$ 일 때 열린다. $v_{arm}$ 은 $q^\star$ 에서 공 진행 방향 $\hat v$ 로 낼 수 있는 팔 속력, $v_{rel}$ 은 손이 흡수하는 상대속도 (`d_eff / T_close,tot` 또는 실측). 후보별:
1. `v_dir_max_lp`: 접근축을 유지한 채 관절 속도 box 안에서 $\hat v$ 로 낼 수 있는 최대 속력 (LP, 물리 상한).
2. `v_dir_max_dls`: L3 §4.5 최소노름 추정 (런타임 `DirectionalSpeedMax` 의 거울). LP 이하이며 차이는 최소노름 해가 버리는 여유 자유도다.
3. **토크 한계 방향 가속**: $\max a$ s.t. $J_5\ddot q + \dot J_5\dot q = [a\hat v; 0\;0]$, $|M\ddot q + h| \le \eta_\tau \tau_{max}$ 를 속력 ramp 를 따라 풀고 stroke · 시간을 적분한 $v_{arm}$.

전부 **낙관적 상한**이다 (bang-bang, 자세 고정, 도달 구). 창이 비면 결론은 확정적이고 열리면 provisional 이다.
- `cell_table.csv` 의 값은 **투척 수**이고 분모는 `throw_summary.csv` 의 전체 격자다. `drop_table.csv` 의 셀은 창이 열리는 후보 비율이다.
- **기본값이 없는 인자는 전부 결정이다**: `--velocity-source` (`config` = 실행이 강제하는 `joint_limits.max_velocity`, `model` = URDF `<limit velocity>`), `--rotor-inertia` (URDF 에는 회전자 반사관성이 없다 — 빼면 모든 가속이 과대평가되므로 0 은 명시해야 한다), 선행시간 다섯 항. 최소 비행시간은 $T_{det} + L + T_{close,tot} + T_{arm} + T_{margin}$ (L1 §4.1 D-15).
- **프레임**: judge 의 `p_model` · `v_model` 은 모델 world (URDF root) 이고 `LOCAL_WORLD_ALIGNED` Jacobian 프레임과 같아 그대로 쓴다. 바닥은 world 에서 수평이므로 여유 높이는 `p_world_z` 열, stroke 하강은 world +z 방향으로 잰다. `p_world_z` 가 옮긴 `p_model` 과 어긋나면 종료한다.
- **자체 검증 (fail-closed)**: 모든 수락 $q^\star$ 에서 catch frame FK 가 `p_model` 과 `--fk-tolerance-m` (기본 2.5 mm = judge `eps_pos` + 여유) 안이어야 한다. 벗어나면 관절 순서 · frame · 모델이 judge 와 다르므로 종료한다. 최소노름 속력이 LP 를 넘어도 종료한다.

### catch_gate_map

`catchability_map` 의 수락 후보에 계획기의 나머지 게이트를 건다: 도달시간 ([L3 §4.3](L3_planner.md)), $\gamma$ 창 (§4.5), 정지점 (§4.9), commit 선행 (§4.11, $t_c - t_{plan} \ge T_{close,tot} + T_{arm} + T_{margin}$, 런타임 함수가 없어 python 이 건다). **판정은 python 이 하지 않는다**: `catch_gate_batch` 가 `time_feasibility.hpp` 의 런타임 함수를 부른다 (G3-I).
- **python 이 만드는 입력**: $\dot q^u$ (L3 §4.5 DLS 단위 속도). `reference.v_max` 는 `--v-max-m-s derived` 면 수락 후보 LP `v_dir_max` 최대 / $\eta_v$ (S4.4: TCP 항은 관절 정격 안에서 구속하지 않는다). 감쇠 $\lambda$ 는 `catching.planner.search.grid.gamma.unit_speed_damping`, 없으면 `catch_speed_budget.DEFAULT_DLS_DAMPING`.
- **python 이 거는 경계**: $p_{stop}$ 이 도달 구 · 바닥 안인가 (런타임 탐색에는 정지점 판정이 없다, L3 §4.9 — 이 경계는 지도의 것이다). 정지점 높이는 world 로 옮겨 잰다 (기울어진 world 에서 모델 z 차이 근사는 틀린다).
- **도달시간은 두 층**을 항상 같이 낸다. `box` = 컨트롤러 YAML 가속 box (`catching.robot.arm.qdd_max`) 로 C++ 가 판정한다. 런타임 트리는 controller YAML + ROS 파라미터 override 이므로 지도의 box · $\lambda$ 는 **런타임과 같은 파일을 `--overlay <파일>` (반복 가능, leaf 단위로 얹음) 으로 줄 때에만** 같고, 안 주면 출하 값의 지도다 (`gate_map_summary.yaml` 의 `overlays`). `torque` = **그 이동**이 토크 한계 안인가: 전 관절이 하나의 bang-bang / 사다리꼴 프로파일로 대기 자세 → $q^\star$ 로 가고 경로 전체에서 $|M\ddot q + h| \le \eta_\tau \tau_{max}$ (회전자 관성 포함) 인 최대 경로 가속을 이분 탐색한다. 충분조건이고 런타임 대응물이 없어 **provisional** 이다 (D-16 개정).
- **대기 자세는 하나다.** 지도가 여러 seed 로 판정됐으면 `--seed-id` 필수 (합집합은 과대평가). `proposed_wait_pose` 는 $\gamma$ · 정지 게이트를 통과한 $q^\star$ 의 관절별 midrange 이고 새 seed 로 `catchability_map` 부터 다시 돌리는 고정점 반복의 입력이다.
- rollout ([L3 §4.8](L3_planner.md)) 은 판정하지 않는다. 열린 셀은 `PASS(provisional)` 이고 빈 지도는 확정적이다. `gate_cell_table.csv` 의 분모는 전체 격자이고, `open_candidates` 는 층이 아무것도 열지 않으면 `null` 이다. FK($q^\star$) 가 `p_model` 과 어긋나면 종료한다.

### catch_search_map

투척마다 계획기가 볼 wake 열을 만들어 `catch_search_batch` (런타임 `CatchSearch::Plan`, grid · nlp) 에 넘기고 그 판정을 투척별로 줄인다. **판정은 python 이 하지 않는다.**
- **투척**: gate 지도와 같은 여섯 발사 축 격자 (`throw_id` = `catchability_map` 의 `throw_index`, `--grid-origin arm_base` 일 때) · Latin hypercube 표본 (`--sample`) · 또는 `--throws-file`. 돌린 투척은 `throw_list.json` (`catching_throw_list/1`, sim world) 이다.
- **wake**: 예측은 비행 모델 자체 (`integrate_flight` 출하 항력 RK4) 다. wake 는 발사 뒤 `--detection-delay-s` 부터 `--vision-period-s` 마다, 공이 바닥 (`--min-catch-height-m`, sim world) 위 · `--horizon-s` 안에 있는 동안 생기고, snapshot 은 wake 뒤 $k\,dt$ 의 상태다 (dt 기본 `prediction.dt_expected`). 좌표는 model world 이다.
- **binding**: 컨트롤러가 configure 에서 묶어 탐색에 넘기는 값을 합성 트리 (컨트롤러 YAML + `_base.yaml` · `sim.yaml` + `--overlay`) 와 로봇 config (장치 한계 = YAML ∩ URDF) 에서 만든다. 값마다 출처를 `binding_<search>_sources.yaml` 에 적는다. 컨트롤러가 **내장 상수**로 돌 키 (없거나 `TBD` 인 `planner.search.grid.gamma.eta_v` 등) 는 키 이름으로 거부한다 (사본을 두지 않는다). 컨트롤러도 TBD 로 두는 값은 `.nan`. 팔 관절의 URDF 한계를 CM 이 다른 관절 항목에서 읽게 되는 경우는 거부한다.
- **줄임**: 투척별 수용 (어느 wake 든 plan) · 첫 수용 wake · $t_c$ · lead, 거부 투척의 대표 사유 (sim 의 `plan_reject` 와 **같은 함수**, 동률은 나중 것). 축 값 `flight_time_s` · `terminal_speed_m_s` · `closest_distance_m` 은 비행이 `planner.wait_pose` 포구점에 가장 가까운 순간의 값이다 (탐색과 무관).
- **요약** (기술 통계만): `--sim-trials` 의 `catching_trials.csv` 를 `throw_id` 로 이은 sim 표 (수용 · 게시 = `published` / 수용 · 미게시 = `withheld` / 거부 = `no_plan` · `no_search`, × `truth_success`, 무효 시행 제외) 와 오프라인 일치율 (수용 ∧ 게시 + 미수용 ∧ 미게시). 수용 · 미게시는 plan 을 붙든 층 (`segment` · `cycle` · `search`) 으로 나뉜다.
- **구간 판정** (`--judge-segments`, 기본 꺼짐): 탐색이 끝난 뒤 **탐색이 수용한 투척만** 같은 wake 로 계획기 주기 전체 (`PlannerCycle::Run`: 탐색 → 첫 구간 풀이 → 쌍 게시) 를 돌린다. 쌍 게시 ⇔ `outcome == published` ∧ `search_valid == 1`. `closed_form` 은 구간 계획기가 없어 실행 전에 거부한다.
  - `verdicts_<search>.csv` 의 `segment_*` 열은 탐색이 거부했거나 걸러진 투척에서 **빈 칸**이다 (거부 = `False` 와 구별되는 "판정 안 함").
  - 오프라인에 없는 것 (sim 에서 본다): 쌍 게시 뒤 replan · 교체, 추정 잡음 (공분산 0), 실행 시계가 만드는 것 (예산 초과 풀이, 풀이 중 더 새 예측이 와 버려진 쌍, 오래된 RT 보고), 벽시계 거부, plan 을 따르는 중의 탐색, 컨트롤러 park 조건.
- **사전 판정이 붙은 투척**: 목록 항목의 `screen` 이 비어 있지 않으면 (`catch_throw_design` 이 붙인다) 탐색 없이 그 이름으로 거부한다. `--judge-screened` 는 그 투척에도 탐색을 돌린다. `screen` 은 그것을 낸 구 · lead 하한 · 검출 지연 (`meta.design`) 에 대해서만 필요조건이다: 이 실행의 구가 더 크거나 자리가 다르거나, lead 하한이 더 작거나 (모르거나), 첫 wake 가 더 이르면 (`screen_basis`) `--judge-screened` 없이는 거부한다. 목록의 `meta.unlaunchable` 은 지도의 행으로만 들어간다.
- **어느 탐색을 돌릴지**는 `--search` 가 정한다 (트리의 `planner.search.mode` 가 아니다). 그 탐색의 맵이 트리에 없으면 binding 이 거부한다. `planner.segment.mode` 는 lead · nlp 한계의 속도 여유 · `follows_segments` 를 정한다.
- 판정은 투척 하나로 정해지므로 `--jobs` 의 shard 분할과 무관하다. `verdicts_*.csv` 의 `search_wall_us` 는 실행마다 다르다. 탐색이 설정을 거부하면 (batch 종료 코드 3) 요약의 `refused_configuration` 에 적고 지도는 없다. 구간 계획기가 거부하면 `segment.refused_configuration` 에 적고 구간 열은 비며 탐색 지도는 그대로다.

### catch_throw_design

탐색을 돌리지 않는다. 발사점 (base 축 기준 거리 · 방위, world 높이) · 통과점 (대기 자세 포구점의 world x · y 오프셋, 높이는 포구점 + `--pass-plane-offset-m`) · 발사각 격자에서 통과점을 **지나는** 발사 속력을 항력 모델로 푼다 (`solve_launch_speed`). `throw_id` 는 여섯 축 (`distance_m` · `azimuth_deg` · `release_height_m` · `pass_dx_m` · `pass_dy_m` · `elevation_deg`) 곱에서의 순번이다. 적분은 `catchability_map` 과 같은 힘 법칙 · RK4 · 스텝 수다.
- **열**: `aim_deviation_deg` (kinematic 지도의 여섯 축이 다 있어 `--gate-map` join 이 성립), `apex_margin_m` (정점 − 통과면; `--apex-margin-flag-m` 아래는 `apex_flag`), `reach_last_s` (도달 구 안의 마지막 시각) 와 `lead_available_s` (= −검출 지연), `body_flag` (통과점과 대기 자세 관절 원점 폴리라인의 거리 — 표시만).
- **사전 판정** (`screen`): `screen:shoot_ascending` (하강 교차 없음) · `screen:shoot_fail` (미수렴 또는 `--speed-max-m-s` 초과) · `screen:reach` (비행이 구 밖) · `screen:time` (lead 상한 < 두 탐색의 lead 하한 중 작은 값). 구 = batch `--print-reach-bound` (반경 + 두 탐색 `ik.eps_pos` 중 큰 값) + `robot.hand.docking.s_ent`. nlp 하한 키가 없거나 `TBD` 면 시간 판정을 하지 않는다. 앞의 둘은 발사 상태가 없어 목록에 들어가지 않고 `meta.unlaunchable` 로 간다.
- **발사각 축**은 범위다: `--max-elevation-deg` · `--min-elevation-deg` · `--elevation-step-deg` (셋 다 필수) 가 `max − k·step` (≥ min) 을 오름차순으로 만든다. 최대각은 언제나 값이고 최소각은 격자에 놓일 때만 값이다.
- **제한** (`--min-pass-time-s`, 선택): 항력 풀이 `pass_time_s` 가 그보다 짧으면 (같으면 아님) `limit:pass_time`. `screen` 과 독립이다: `screen` 은 어느 탐색도 받을 수 없다는 뜻이라 지도가 검증하지만, 제한된 투척은 수용될 수 있다.
- **`select`**: `--subset` 부분 격자, `--ids-file`, 또는 `--refine-from <verdicts.csv …>` (부분 격자에서 한 축으로 한 칸 떨어진 두 점의 판정이 다르면 그 사이 격자점을 더함, 재귀 없음). **제한된 투척은 세 경로 모두에서 목록에 쓰지 않는다.** 목록 `meta.limit` 이 기준값과 뺀 수를 적어 지도의 수용률 분모를 복원하게 한다. `--max-throws` 는 뺀 뒤의 수로 세어 넘으면 쓰지 않고 끝난다.
- **`--segment-mode`**: `planner.segment.mode` 한 값만 `--overlay` 뒤에 덮어쓴다. 바꾼 값은 provenance 의 `overrides` 에 남는다.
- **`plan-set`**: 수용된 투척을 `throw_id` 오름차순 `catching_throw_list/1` 로 쓴다. 판정이 목록을 덮지 못하거나 수용된 id 가 목록에 없으면 거부한다. `--tier search` (기본) 는 `accepted` 열 (탐색 집단), `--tier segment` 는 `segment_accepted` 참 (구간 집단). 구간 집단은 `--judge-segments` 판정표만 받는다 (그 열이 없거나 탐색 수용 투척 중 그 칸이 빈 것이 있으면 거부 — 구간 계획기가 설정을 거부한 판정표).
- **`sample`**: 층 (`--strata` 축 값 조합) 마다 같은 비율, 정수는 최대 잔여 방식 (합 = min(N, 크기), 동률은 층 키 순서). 선택은 `sha256("<seed>:pick:<id>")` · `sha256("<seed>:order:<id>")` 로 정해 버전 · PC 가 달라도 같다. meta 의 `plan_set_sha256` 은 여러 PC 결과를 `throw_id` 와 함께 합치는 열쇠다.

### catch_population

설정 파일 하나 (`<config>/<robot>/catch_population.yaml`; 형식은 `utils/catch_population.py` docstring, 없는 키 · 모르는 키 · 범위 밖 값은 키 이름으로 거부) 가 모집단과 거르는 방법을 정한다. **로봇은 파일이 놓인 폴더다**: 판정에 쓰는 컨트롤러 설정이 그 폴더이고, 조합 (`arms`) 이 출하값에서 바꾸는 것은 `planner.search.mode` 와 `planner.segment.mode` 둘뿐이다.

```
설계 ─제한─▶ 모집단 ─탐색─▶ search 집단 ─구간─▶ segment 집단 ─뽑기─▶ 던질 묶음 ─(2 단계: sim)─▶ 쌍 게시 ─▶ 포구
```

- **판정도 설계도 직접 하지 않는다.** 있는 도구의 `main` 을 차례로 부른다: `catch_throw_design design` → `select` (모집단 = 발사 불가 · 제한을 뺀 목록) → 조합마다 `catch_search_map --judge-segments` **한 번** → `plan-set` → `sample`.
- **이어 돌리기**: 산출물이 있는 단계는 건너뛴다. 결과 폴더는 설정 하나의 것이다: `config.yaml` (사본) 과 바이트가 다른 파일을 주면 거부하고, 비어 있지 않은데 `config.yaml` 이 없는 폴더도 거부한다.
- **빈 집단은 결과다**: 한 발도 수용하지 않은 층은 집단 파일을 쓰지 않고 (0 발), 던질 것이 없는 조합은 묶음도 없다. 탐색이나 구간 계획기가 설정을 거부한 조합은 그 문장과 함께 "집단 없음" 이다. manifest 와 보고에 적고 나머지 조합을 계속한다.
- **`inputs.json`**: 판정에 쓴 설정 폴더 YAML 전부의 sha256. 이후 파일이 바뀌었으면 **바뀐 파일의 이름을 대며 거부한다** (컨트롤러 YAML 을 고친 뒤 같은 `--out` 으로 돌리면 "이미 있는" 지도와 집단이 새 설정의 것으로 넘어가기 때문). 판정기 (`catch_search_batch`) 바이너리 sha256 `judge_sha256` 은 **거부하지 않고 경고한다** (다른 바이너리로 이어 돌리면 stderr `WARNING`, 판정기 동작이 바뀌었으면 새 `--out`).
- **설정 파일은 쓴 대로 읽거나 거부한다**: 따옴표 없는 `-40:40:20` 은 YAML 이 60 진수 정수로 읽으므로 거부하고 (범위는 `'lo:hi:step'` 문자열), 같은 키를 두 번 쓴 파일도 거부한다.
- **던질 목록은 지금 있는 집단의 것이어야 한다**: `sample.json` 이 적은 집단 sha256 이 집단 파일과 다르면 다시 뽑는다.
- **`catch_population_report`** 는 폴더의 파일만 읽는다 (판정하지 않는다). 판정하지 않은 집단은 `not judged`, 조합들의 지도를 서로 다른 판정기 바이너리가 판정했으면 머리에 `judge binaries: N different` 줄이 붙는다. 3D plot 의 비행은 집단당 `--max-flights` 개를 `throw_id` 해시로 골라 매번 같고 그린 수를 제목에 적는다.
- **sim 결과가 있으면** (`<조합>/sim_throws.csv`: `arm,seed,chunk` 뒤에 `catching_trials.csv` 열 전부) 조합 · 추정기 seed 마다 던짐 → 쌍 게시 → 포구 (`truth_success`) 의 수와 표를 더한다. `invalid_reason` 이 있는 시행은 던진 투척이 아니다.
- **뽑아 던진 조합**: 표의 수는 **던진 투척 안의 수**이고 (비율 옆 구간은 그 안의 Wilson 95 %), 집단 전체는 **추정**이다: `sample.json` 의 층마다 (집단의 수 ÷ 던진 수) 로 되돌려 더한다 (`stratified_estimate`). 던진 투척이 없는 층은 추정 밖으로 따로 센다. 집단을 전부 던졌으면 추정 없이 그 수를 적는다. 조합 이름은 `<search>_<segment>` 다.
