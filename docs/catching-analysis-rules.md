# 포구 분석 도구의 판정 규칙

`rtc_tools.analysis` 의 포구 분석 도구가 **무엇을 읽고 무엇을 판정하는가** 만 둔다. 실행 방법은 [rtc_tools/README.md](../rtc_tools/README.md), 열 정의와 절차는 각 도구의 머리 docstring, 투척 실행은 [catching-sim-trials.md](catching-sim-trials.md) 가 갖는다.

## 공통 규약

- 측정 도구다. `NOT_EVALUATED(<사유>)` 는 입력이 없어 계산하지 않았다는 뜻이고 통과가 아니다. 모르는 값은 NaN 이며 0 으로 대신하지 않는다.
- 로봇 상수는 코드에 없다. 한계 · 프레임 · 게인은 `--config-dir` 의 profile (`_base.yaml` → `sim.yaml` → `--overlay`) 과 러너가 `run_meta.json` 에 남긴 컨트롤러 미러에서 읽는다.
- 판정 대상은 `catching_trials` 의 유효 시행이다. 무효는 rig 실패뿐이고 plan 없음 · abort · 놓침은 유효한 실패다.
- 쌍 비교는 `(kind, seed, sample_idx)` 로 같은 투척을 짝짓는다. 분위수는 nearest-rank 라 표본 100 미만의 p99 는 최댓값이다.

## plot_rtc_log

- 로그 종류는 파일 이름과 열 지문으로 정한다. `tick` 간극은 드롭된 행이다 (`dualarm_diag` 는 간극 뒤 첫 행이 재시드면 컨트롤러가 돌지 않은 구간).
- 명령 kinematics 는 `q_cmd_*` 를 기록 시간축으로 미분하고 5 행 box 평활, 1e-3 아래는 그리지 않는다.
- `planner_events.csv` 의 기한에 끊긴 풀이는 분포에 넣지 않는다 ([planner_solves](#planner_solves)).
- 값은 계산한 tick 에서만 그리고 센다 (나머지의 0 은 계산하지 않은 0).
- 타이밍 CSV 에 `run_id` 가 여럿이면 마지막 런만 그린다.

## derive_accel_limits

- 표본 $(q,\dot q)$ 마다 $\sum_j |M_{ij}| a_j \le \eta_\tau \tau_{max,i} - |g_i| - |c_i|$ 를 만족하는 최대 $a = s\,w$ 를 구하고 전 표본의 최소를 상수 box 로 낸다 (최악 부호 결합의 충분조건). `--eta-tau` 는 기본값이 없다.
- 퇴화 ($\tau_{dyn} \le 0$ 표본, 또는 $s^\star$ < `--min-accel`, 기본 0.1) → `adopted: false`, exit 2. 교차 검증 (`--check rnea` · `--check mujoco`) 실패 → exit 3.
- 출력은 분석 산출물이다. 컨트롤러가 읽는 box 는 `robot.arm.qdd_max` 다.

## clock_phase

- 시행별 위상 오차 $\delta(t) = (\text{steady} - \text{steady}_0) - (\text{sim} - \text{sim}_0)$, 창은 발사부터 비행 끝까지. $\delta_{max} = \max|\delta|$, max pause = 한 step 의 $\Delta$wall $- \Delta$sim 최대.
- $\delta$ 는 **공변량**이지 판정이 아니다. `--eps-mm` 을 주면 $v_{max}\delta_{max} + \tfrac12 a_{bound}\delta_{max}^2 \le \varepsilon$ 와 max pause $\le \varepsilon / v_{max}$ 를 만족하는 비율을 "valid under eps" 로 병기한다. 95 % 를 통과시키는 $\varepsilon$ 는 제안값이다.
- lane 의 누적 drop 을 그대로 보고한다.

## hand_close

- $\rho(t) = \min_{i\in C}\big((q_i - q_i^{pre}) s_i / |q_i^{cls} - q_i^{pre}|\big)$, $T_{close,e2e}(\eta) = \inf\{t - t_{cmd} : \rho \ge \eta\}$. $\rho$ 는 최소 (한 손가락이 늦으면 손 전체가 늦다), caging 집합 $C$ 는 프로파일이 정한다.
- 시행은 command lane 으로 자른다. 명령 행이 자기 travel 의 25 % 밖이면 `other` 이고 시행을 끝낼 수만 있다.
- 시간축 둘: `steady` (호스트 스톨 포함) 와 `tick × dt` (sim 시간). sidecar 에 `dt` 가 없으면 tick 축은 신뢰 불가로 찍는다.
- `mean − T_close_lead` 는 폐쇄가 $t_c$ 뒤 언제 끝나는가다. tick 양자화 `dt / sqrt(12)` 는 표준편차 옆에 적고 더하지 않는다.

## vision_lane

- `prediction/trajectory` 는 필드 이름으로 디코딩한다. 이름 · offset · datatype · count · `point_step` · endianness 중 하나라도 다르면 그 메시지를 거부하고 센다.
- 발행 주기 · N · 지평 · `frame_id` · 측정 손실 뒤 `validity` · best_effort 대 reliable 손실을 측정만 한다.
- $T_{det}$ (발사 → 첫 VALID 예측): 비행은 truth lane 의 침묵 (`--flight-gap-s`, 기본 0.3 s) 으로 가른다. `recv` (프로브 steady) 와 `stamp` (발행자 stamp) 두 축을 각각 한 시계 안에서 낸다. 발사 전부터 VALID 이던 트랙은 검출이 아니다. 검출 없는 비행은 NaN.

## catching_trials

- **무효** (우선순위 순, 첫 일치 하나): `srv_refused` (기록 `accepted` 거짓) · `not_launched` (truth 행 0) · `controller_silent` (오프셋 없음 또는 창에 diag 행 0) · `lane_drop` (짝 발사 없음 또는 `dropped_total` 증가) · `sim_stall` (sim 시간 간격 > 5 × 공칭 step). clock lane 이 없으면 lane 규칙은 `NOT_EVALUATED`.
- **truth 성공**: HOLD 끝 (첫 RETREAT tick) 부터 대기 자세 release 까지 모든 truth 샘플이 catch frame 에서 `--hold-radius-m` (기본 공 지름) 안. truth 첫 stamp 가 발사에서 5 s 넘게 떨어진 파일은 거부한다.
- **판정**: 유효 시행의 Wilson 하한 (z 1.96, 97.5 % 단측) `lower_975 ≥ --floor` → PASS / FAIL, `n_valid` < `--n-valid-target` → `INSUFFICIENT_N`. ITT 는 무효를 실패로 센다.
- **$t_c$ 분해**: 첫 COMMITTED tick 의 $t$ + `plan_t_c_s` 가 $t_c$. 공이 $t_c$ 전에 로봇에 맞으면 직전 0.25 s 이차 적합으로 외삽한다. `tc_axis` = `shifted` 는 stamp 축 $t_c$ 와의 차 > `--tc-shift-max-ms` (기본 5 ms) 이고 중앙값에서 뺀다.
- 명령 쪽 (서보 · CLIK · `ref_vs_true`) 은 lead 가 켜지면 $t_c - T_{lead}$ tick 에서 읽는다. 기준은 `segment` → `ref_x` → `none` (NaN) 순이다.
- `cmd_hold_s` 는 첫 APPROACH 부터 명령 속력 > 0.02 m/s 첫 tick 까지. 마지막 구간 wake 의 snapshot 은 wake + 6 ms 까지 tick 이 읽은 가장 새 것이다. catch frame 요약은 commit 시행 < 10 이면 `NOT_EVALUATED`.
- **공변량** (판정 아님): clock 위상 (짝 시행이 절반 미만이면 다른 run 으로 보고 거부), RTF (`rtf_flight` 발사 ~ 첫 접촉 또는 sim 0.6 s, `rtf_trial_min` sim 0.25 s 창 최솟값), tick overrun (`t_total_us` > 제어 주기).
- **예측 NEES** (`--eval-samples`): `time_ns + horizon_ns` ≤ 첫 접촉인 표본만. 지평별 $\Sigma\,\mathrm{NEES} / \Sigma\,n$ 의 시행 부트스트랩 95 % CI ∋ 3 → PASS, NaN > 10 % → `NOT_EVALUATED`, coverage 는 $\chi^2_3$ 0.95 = 7.815 이하 비율.
- **오차 예산 A⊥B** (`--probe-dump`): commit tick 마다 $A = p_{true}(t_c) - \hat p_{live}(t_c)$, $B = \hat p_{live}(t_c) - p_c$. $n$ < 100 이면 독립성 검정 `NOT_EVALUATED`.
- **손-관절 캡처**: `outcome_source` 는 첫 RETREAT 행, `*_judge` 는 마지막 HOLD 행에서 읽는다. 오프라인 재현은 `hand_blocked_s_judge + dt >= t_persist`.
- **그 밖**: `ref_saturated` streak 은 분포만 기록한다. 충격량 상관은 임계 없이 보고하고 $n$ < 50 이면 `NOT_EVALUATED`. plan 판정 창은 `[발사, 첫 충돌]` 이고 `plan_reject` 는 [catch_search_map](#catch_search_map) 과 같은 `most_frequent_reason` 이다.

## planner_solves

- `segment_core_reason` == `deadline` 인 풀이가 기한에 끊긴 풀이이고 그 시간은 하한이다. `segment_outcome` 의 `budget` 으로 가리지 않는다 (끝까지 돌아 예산을 넘긴 풀이도 `budget` 이다).
- `summarise_times` 는 끝난 풀이의 분포와 끊긴 풀이의 수 · 가장 이른 끊긴 시각을 따로 낸다. `quantile_with_bound` 의 분위수는 가장 이른 끊긴 시각보다 작으면 참값, 아니면 하한이다.

## catching_pool

- unit 은 준 순서 (사전 선언 seed 순, 보충 unit 마지막), 시행은 `idx` 순으로 읽는다. 누적 유효 수가 `--n-valid-target` (기본 200) 에 닿는 시행까지 포함하고 그 뒤는 `beyond_target` 으로 유효 · ITT 양쪽에서 뺀다. 모자라면 `INSUFFICIENT_N`.
- McNemar (arm 2 개 이상): 두 arm 모두에서 포함 · 유효인 같은 투척 쌍의 정확 이항 양측 $p$. 한 arm 에 같은 투척이 두 번이면 거부한다.
- clock 위상 표본 수는 arm 들의 포함 · 유효 · 유한 `delta_max_ms` 시행만으로 ≥ 200 을 판정한다. `--extra-d3` 는 나열만 하고 더하지 않는다.
- `decel_*` 옛 출력과 `invalid_reason` 열이 없는 CSV 는 거부한다.

## catching_hand_near

- plan 이 있었는가 (COMMIT) 와 plan 이 있을 때 잡았는가 (CATCH) 를 따로 적합하고 곱한다.
- 조준 검증: `aim_error_m` 또는 `model_rms_m` (첫 접촉 전 truth 대 모델) 이 `--aim-tol-mm` (기본 2 mm) 를 넘는 시행이 하나라도 있으면 rc 1. `aim_error_m` 이 없으면 `pass: null`, rc 1.
- grid arm: 속력 하나의 로지스틱 $v_{50}$ + 시행 부트스트랩 CI (유한 복제 < 20 이면 NaN). LHS arm: 로지스틱 GLM (IRLS, 기울기 ridge 1e-4) 에서 $r$ 별 $v_{50}(r)$ 과 $P(\text{plan})P(\text{catch}|\text{plan}) = 0.5$ 의 $v_{50}$.
- `--ab A B`: 두 arm 라벨을 `(kind, seed, sample_idx)` 로 짝지어 catch · commit 각각 McNemar 정확 검정.

## catching_arm_budget

- "팔이 못 간다" 를 플랜트 (서보 지연 · 토크 · 속도) · soft-catch 참조 ($\omega$, `a_max` / `v_max` 포화) · CLIK (속도 box · 가속 제약) · 계획기 (도달시간의 가속 box) 네 층으로 가른다.
- 참조 잔여 ‖e‖ 를 commit 과 마지막 활성 tick 에서 읽고 이론 잔여 $(1+\omega T)e^{-\omega T}$ 를 함께 낸다.
- 실행 envelope = 활성 tick 의 5-tick 평균 $|\ddot q_{cmd}|$ p95 / max. 같은 $\Delta q$ 의 도달시간 (`t_min_joint`, `TMinChecked` 와 같은 닫힌식) 을 가속 box 와 envelope 로 두 번 재 commit 시점 가용 lead 와 비교한다.
- `--write-envelope-box` 는 envelope p95 의 관절별 max 를 `qdd_provisional: true` 로 쓴다 (sim 전용).

## catching_decel

- 창은 첫 `DECEL` tick 부터 catch frame 이 정지한 첫 tick (첫 `RETREAT` 이전) 까지. 정지 = 선속도 < `--rest-speed` (0.02 m/s) 이고 접근축 각속도 < `--rest-axis-rate` (0.2 rad/s) 가 `--rest-s` (0.05 s) 이어짐. 못 멈추면 `rest_reached = false`.
- 관절이 아니라 task pose 로 판정한다 (null-space 운동). 접근축 roll 은 넣지 않는다.
- 가속 · jerk 피크는 `--smooth-ticks` (기본 5) box 평균. 한계 위반 = 여유 < 0 또는 비 > 1 인 시행 수.
- 성공 `--success`: `truth` (기본) 또는 `hold` = `truth_success` ∧ `hold_verdict` ∧ ¬`abort_in_window`.
- `--a` / `--b`: 짝 2×2, McNemar 정확 검정, Tango score 비열등 검정 $H_0: p_b - p_a \le -\text{margin}$ (비열등 ⇔ $p$ < `--alpha`, 기본 0.025, 단측), 정확 검정력. 불일치 쌍이 없으면 필요 쌍 수는 `null`.

## catching_grid_sweep

- arm 의 조건은 미러의 `prediction.dt_expected` · `io.n_min` · `planner.search.grid.slice.dt` 이고 한 arm 의 unit 끼리 다르면 거부한다.
- 메시지 주기는 컨트롤러가 받은 새 snapshot 간격 (`--flight-gap-s`, 기본 0.5 s 초과 간격 제외).
- `pred_mm` · `total_mm` 은 `shifted` 시행을 뺀다. `rtf_trial_min` < `--rtf-min` (0.95) 과 RTF 없는 시행은 세기만 한다.
- 비교: 짝 차이 $p_B - p_A$ 의 Wald 95 % 구간, McNemar 정확 $p$, family 안 Holm 보정.

## catching_capture_set

- 셀의 시행 전부가 유지돼야 셀이 유지다 (`held_cells`).
- 상자: (closing speed, 폐쇄 시각) 지도에서 전부 유지된 직사각형 중 폭 ≥ 요청값이고 closing speed 상한이 가장 큰 것 (`box_by_min_width`). 통과 평면 기준 창은 $s_{ent}(1/c_{lo} - 1/c_{hi})$ 만큼 좁다 (`entrance_window`).
- 통과 평면 $s_{ent}$: lateral 집합의 모든 점과 허용 기울기의 직선이 무접촉인 가장 낮은 높이.
- 옆 속도 고리는 유지율이 기준 고리 − 허용 폭 이상이면 통과 (`rings_within_drop`), 하한은 `clopper_pearson_lower`.

## catching_wait_pose_search

- 목적은 대기 자세 자신의 접근축 $\hat v(q) = -\mathrm{axis}(q)$ 방향 속력: `dls` (기본, `DirectionalSpeedMax`) · `lp` (물리 상한) · `robust` (`--robust-samples` 32 개, $|\delta|_\infty \le$ `--robust-eps-rad` 0.1 섭동의 DLS p10, LP 로 cap).
- 탐색 상자는 런타임이 대기 자세를 받아 주는 상자다 (관절 한계를 `limit_margin` 만큼 안으로, URDF 한계와 교집합).
- 자체 검증: FK 재계산 1e-9 m 이내, URDF 한계 안, `v_dir_dls` ≤ `v_dir_lp` (상대 1e-3 + 1e-6 m/s). 하나라도 어기면 아무것도 쓰지 않는다.

## catchability_map

- 판정은 `catch_pose_ik_batch` (런타임과 같은 `CatchPoseIk::Solve`) 가 하고 python 은 하지 않는다.
- headline 은 seed 순위 1위 하나의 수치이고 비율의 분모는 전체 격자다. 합집합은 `accepted_throws_any_seed` 이름으로만 남는다.
- judge 의 프레임은 모델 world 다 (`scene_world` 를 합성, 없거나 provisional 이면 거부).
- shard 는 행 · ID 와 입력 fingerprint 가 맞을 때만 재사용한다. `alpha_max` 는 judge 가 실제로 쓴 값이다 (params, 없으면 judge 기본 0.26 rad 의 미러).
- $\varepsilon$ 경계: 수락 $p_c$ 를 축별 $\pm\varepsilon$ 로 옮긴 6 사본을 다시 judge 에 넣어 하나라도 거부되면 경계 위다.
- 비행은 중력 + 이차 항력 $-\tfrac12\rho C_d A|v|v$, 각속도 0. 공 상수는 기본값이 없고 호출자가 출처와 함께 넘긴다.

## catch_speed_budget

- 수락 후보의 $\gamma$ 창 (`ComputeGammaWindow`) 이 열리는가: `v_dir_max_lp` (접근축 유지 LP 상한), `v_dir_max_dls` (`DirectionalSpeedMax`, LP 이하), 토크 한계 방향 가속을 적분한 $v_{arm}$. 전부 낙관적 상한이라 빈 창은 확정, 열린 창은 provisional.
- `--velocity-source` · `--rotor-inertia` · 선행시간 다섯 항은 기본값이 없다.
- 자체 검증: 수락 $q^\star$ 의 FK 가 `p_model` 과 `--fk-tolerance-m` (2.5 mm) 안, 최소노름 속력 ≤ LP. 어기면 종료한다.

## catch_gate_map

- 수락 후보에 도달시간 (`ReachTimeFeasible`) · $\gamma$ 창 · 정지점 (`StoppingPoint`) · commit 선행 $t_c - t_{plan} \ge T_{close,tot} + T_{arm} + T_{margin}$ 을 건다. 판정은 `catch_gate_batch` 가 런타임 함수로 한다.
- 도달시간은 두 층: `box` (컨트롤러 가속 box) 와 `torque` (대기 자세 → $q^\star$ 이동이 $|M\ddot q + h| \le \eta_\tau \tau_{max}$ 인 최대 경로 가속, provisional).
- 정지점이 도달 구 · 바닥 안인가는 python 이 건다 (높이는 world 로 잰다).
- 대기 자세는 하나다: 여러 seed 로 판정된 지도는 `--seed-id` 필수. rollout 은 판정하지 않아 열린 셀은 `PASS(provisional)`.

## catch_search_map

- 투척마다 wake 열을 만들어 `catch_search_batch` (런타임 `CatchSearch::Plan`) 에 넘긴다. 판정은 python 이 하지 않는다.
- wake 는 발사 뒤 `--detection-delay-s` 부터 `--vision-period-s` 마다, 공이 바닥 위 · 지평 안에 있는 동안 생긴다. 예측은 비행 모델 자체 (공분산 0).
- binding 은 컨트롤러 설정 트리에서 만든다. 컨트롤러가 내장 상수로 돌 키 (없거나 TBD) 는 키 이름으로 거부한다.
- 투척별 수용 = 어느 wake 든 plan. 거부 사유는 `most_frequent_reason` (settling wake 제외, 동률은 나중 것).
- `--judge-segments`: 탐색이 수용한 투척만 `PlannerCycle::Run` 을 돌리고 쌍 게시 ⇔ `outcome == published` ∧ `search_valid == 1`. 판정 안 한 칸은 빈 칸이다.
- `screen` 이 붙은 투척은 탐색 없이 거부한다. 이 실행의 구 · lead 하한 · 첫 wake 가 `screen_basis` 와 다르면 `--judge-screened` 없이는 거부한다.

## catch_throw_design

- 탐색을 돌리지 않는다. 통과점을 지나는 발사 속력을 항력 모델로 푼다 (`solve_launch_speed`).
- `screen`: `shoot_ascending` (하강 교차 없음) · `shoot_fail` (미수렴 또는 속력 상한 초과) · `reach` (비행이 구 밖) · `time` (lead 상한 < 두 탐색의 lead 하한 중 작은 값). 앞의 둘은 `meta.unlaunchable` 로 간다.
- `--min-pass-time-s`: `pass_time_s` 가 그보다 짧으면 `limit:pass_time` 이고 어느 선택 경로에서도 목록에 쓰지 않는다.
- `plan-set`: 판정표가 목록을 덮지 못하면 거부한다. `sample`: 층마다 같은 비율 (최대 잔여 방식), 선택은 seed 와 id 의 sha256 으로 정한다.

## catch_population

- 설정 파일 하나가 모집단과 거르는 법을 정하고, 없는 · 모르는 · 범위 밖 키는 키 이름으로 거부한다. 조합이 바꾸는 것은 `planner.search.mode` 와 `planner.segment.mode` 뿐이다.
- 판정은 있는 도구의 `main` 을 차례로 부른다: `catch_throw_design design` → `select` → `catch_search_map --judge-segments` → `plan-set` → `sample`.
- 판정에 쓴 설정 YAML 의 sha256 이 바뀌면 바뀐 파일 이름을 대며 거부한다. 판정기 바이너리가 다르면 경고만 한다.
- 빈 집단은 결과다. 던진 투척 안의 수는 Wilson 95 % 와 함께 적고, 집단 전체는 층별 역가중 추정 (`stratified_estimate`) 이다.
