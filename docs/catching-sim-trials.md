# 포구 sim 시행 실행 안내

`demo_catching_controller` 를 MuJoCo sim 에서 시험하는 방법이다. 투척 하나는 포구 순환 하나 (ARMED → 포구 → RETREAT → ARMED) 이고, 이 문서는 그 투척을 던지는 명령, 입력 파일 형식, 결과 파일과 읽는 법을 정리한다. 컨트롤러 진단 CSV 의 열은 [catching_diag_log_pod.hpp](../integrated_bringup/include/integrated_bringup/logging/catching_diag_log_pod.hpp), launch 인자는 [README](../integrated_bringup/README.md#launch-파일) 가 갖는다.

## 1. 한 번 던지기

`catching_sim_trials` 는 **떠 있는 sim 을 구동만 한다.** sim · 공 추정기 · 컨트롤러 전환은 호출자가 한다.

```bash
# 1) sim (계획기는 enable_mpc:=true 가 필요하다 — mpc_off 면 활성화를 거부한다)
ros2 launch integrated_bringup sim_ur5e_p1b.launch.py enable_viewer:=false use_cpu_affinity:=false enable_mpc:=true sim_lanes:=true
# 2) 공 추정기 (ball_perception 의 colcon workspace 를 추가로 source 한 셸에서)
ros2 launch ball_perception_sim sim_estimator.launch.py \
  profile_path:=$(ros2 pkg prefix ball_perception_sim)/share/ball_perception_sim/config/sim_profile.catching.json \
  producer_revision:=<rtc-framework 커밋>
# 3) 포구 컨트롤러로 전환
ros2 service call /rtc_cm/switch_controller rtc_msgs/srv/SwitchController \
  "{activate_controllers: [demo_catching_controller], deactivate_controllers: [demo_joint_controller], strictness: 1, timeout: {sec: 3}}"
# 4) 투척
ros2 run integrated_bringup catching_sim_trials <out> \
  --throws-file $(ros2 pkg prefix integrated_bringup)/share/integrated_bringup/config/ur5e_p1b/throw_sets/throw_catch_set_grid_mpc.json
```

`iiwa7_leap` 은 같은 순서에서 launch 를 `sim_iiwa7_leap.launch.py` 로, 러너에 `--profile iiwa7_leap` 과 `config/iiwa7_leap/throw_sets/` 의 파일을 준다. `ball_perception_sim` 은 이 workspace 가 아니라 ball_perception 의 별도 colcon workspace 에 있다.

탐색 · 구간 계획기 조합은 launch 인자 `catch_search_mode:=grid|nlp` · `catch_segment_mode:=closed_form|mpc|mpc_docking` 으로 고른다 (`sim_ur5e_p1b` · `sim_iiwa7_leap`; 빈 값은 출하값, 허용 목록 밖의 값은 launch 가 거부한다). `nlp` × `closed_form` 은 조합이 아니다 (`nlp` 탐색은 결과를 넘길 구간 계획기가 필요하다). 러너는 투척 전에 컨트롤러의 미러 파라미터 (`planner.search.mode` · `planner.segment.mode`) 를 되읽어 조합을 확인한다.

러너는 투척마다 다음을 한다.

1. FAULT 면 리셋하고 무장한다 (`catching.enable` 을 false → true) 뒤 ARMED 를 기다린다 (컨트롤러가 스스로 대기 자세 `planner.wait_pose` 로 homing 한다). ARMED 순간의 측정 자세가 대기 자세에서 `--tol-q` / `--tol-qd` 를 넘으면 그 시행을 거부한다.
2. `/sim/launch_ball_at` 으로 투척하고 (spin ω 도 명시, 기본 0) ground truth 와 모드 전이를 기록한다.
3. 순환이 닫히면 (RETREAT 뒤 ARMED) 끝낸다. IDLE · FAULT 로 가거나 `--record-s` 가 지나도 끝낸다. 시행의 판정은 RETREAT 진입 때 발행된 `outcome` 이다.
4. 공을 리셋한다.

러너는 대기 자세를 `planner.wait_pose` 미러로 읽으므로 `planner.wait_pose_source: yaml` 이 전제다. 관절 이름 · 상태 토픽은 출하 profile (`--profile`, 기본 `ur5e_p1b`) 에서, 대기 자세 · 시간 상수 · 팔 예산 · 예측 격자는 떠 있는 컨트롤러의 read-only 미러 파라미터에서 읽는다. 하나라도 없으면 러너는 시작하지 않는다. 미러 값은 `<out>/run_meta.json` 과 시행 기록의 `controller_mirror` 에 남는다.

## 2. 투척을 정하는 방법

투척은 **투척 목록 파일** (`--throws-file`) 이나 **손 근처 설계** (`--dist hand_*`) 로 정한다. 둘 다 없으면 인자 오류다.

### 투척 목록 파일

출하 세트는 `config/<robot>/throw_sets/throw_catch_set_<search>_<segment>.json` 이다. 그 로봇의 투척 모집단에서 sim 이 **잡은 투척** 이고 한 번의 실행의 결과다 (세트에 없는 투척은 "놓친 투척" 이 아니라 뽑히지 않았거나 놓친 것이다). 직접 만들 때의 형식은 다음과 같다.

```json
{
  "schema": "catching_throw_list/1",
  "frame": "sim_world",
  "meta": {"출처": "자유 형식, run_meta.json 에 그대로 남는다"},
  "throws": [
    {"throw_id": 0, "pos": [x, y, z], "vel": [vx, vy, vz], "omega": [wx, wy, wz], "kind": "map"}
  ]
}
```

- `schema` · `frame` 은 위 값이어야 한다. `pos` (m) · `vel` (m/s) 은 sim world 기준이다.
- `throws` 는 비어 있지 않고, 항목마다 정수 `throw_id` (파일 안에서 유일 — 오프라인 지도와 sim 결과를 잇는 키), 유한한 실수 3개의 `pos` · `vel` 이 필요하다. `omega` (rad/s) 의 기본은 0, `kind` 의 기본은 `"list"` 다.
- 그 밖의 키는 그대로 `trial_results.json` 에 실린다. 드라이버가 쓰는 키 (`idx` · `outcome` · `seed` 등) 는 거부된다.
- 목록으로 던진 run 은 `run_meta.json` 의 `args.seed` 와 모든 시행의 `seed` 가 `null` 이다. 출처는 `run_meta.json` 의 `throws_file` (절대 경로 · `sha256` · `n_throws` · `meta`) 이다.

### 손 근처 설계 (`--dist`)

| `--dist` | 투척 |
|---|---|
| `hand_cliff` | 정면 · 속력 {3.5, 4, 4.5, 5, 5.5, 6, 7} m/s × 8 회 (`--n` 무시), 비행 시간 0.65 s |
| `hand_lob` | 수평 아래 85° 급강하 · 속력 {2.5, 3, 3.5, 4} m/s × 14 회 (`--n` 무시) |
| `hand_lhs` | 속력 [3.5, 7] · 비행 시간 [0.65, 0.8] s · 오프셋 [0, 0.2] m · 방향 [0, 360)° · 입사각 [−10°, +15°] 의 Latin hypercube `--n` 발 (`--seed` 로 재현) |

떠 있는 컨트롤러의 `wait_pose` 를 FK 해 얻은 포구점과 접근축 (catch frame +z) 을 기준으로, 도착 속력 · 비행 시간 · 오프셋 · 입사각을 정하고 출하 항력 법칙으로 역적분해 릴리스를 낸다. 릴리스 공 표면이 `--floor-z` 아래면 다시 뽑는다. 시행 기록에 조준 검증 (`aim_error_m`: 법칙으로 적분한 비행이 목표점을 얼마나 비껴가는지) 이 남는다.

```bash
ros2 run integrated_bringup catching_sim_trials <out> --dist hand_cliff --seed 901 --arm <overlay 이름>
ros2 run integrated_bringup catching_sim_trials <out> --dist hand_lhs --n 150 --seed 911 --arm <overlay 이름>
```

`--arm` 은 sim 을 띄운 overlay 의 이름을 `run_meta.json` 에 남기는 라벨이다.

## 3. 러너 플래그

| 플래그 | 기본값 | 뜻 |
|---|---|---|
| `out_dir` (위치 인자) | — | `truth_trial_NN.csv` 와 `trial_results.json` 이 쓰이는 곳 |
| `--profile` | `ur5e_p1b` | `config/<profile>` 의 출하 profile (`iiwa7_leap` 등) |
| `--config-dir` | 설치된 share | profile 의 config 디렉토리를 대체 |
| `--throws-file` / `--dist` | — | 투척 정하기 (위). 둘 중 하나는 필수 |
| `--n` | `25` | `--dist hand_lhs` 에서 뽑는 투척 수 |
| `--seed` | — | `--dist` 의 재현 seed. 목록 run 은 쓰지 않는다 |
| `--limit N` | — | 시리즈의 앞 N 발만 (smoke) |
| `--arm` | — | `run_meta.json` 의 라벨 |
| `--tol-q` · `--tol-qd` | `0.03` rad · `0.05` rad/s | ARMED 순간의 정렬 검사 |
| `--home-timeout` | `30.0` s | ARMED 까지 기다리는 시간 |
| `--record-s` | `12.0` s | 투척당 최대 기록 시간 |
| `--end-on-ball-low` | 꺼짐 | 아래 |
| `--ball-low-margin-m` · `--ball-low-grace-s` | 코드의 상수 | `--end-on-ball-low` 의 문턱 높이 (`--floor-z` 위) 와 outcome 을 더 기다리는 sim 초 |
| `--floor-z` | `0.05` m | 손 근처 설계에서 공 표면이 닿을 수 있는 가장 낮은 z (작업 테이블 상판) |
| `--drag-coefficient` · `--air-density` (+ `-source`) · `--urdf` | `0.55` · `1.204` | 손 근처 설계의 항력 법칙 입력과 그 출처 라벨 |
| `--host-watch` · `--host-rtf-min` · `--host-window` | `warn` · `0.95` · `0.25` s | host 부하 감시 (아래) |

**공 높이로 끝내기 (`--end-on-ball-low`).** 컨트롤러가 plan 을 한 번도 내지 않으면 FSM 이 TRACKING → ARMED 로 돌아가고 RETREAT 가 없어 시행이 `--record-s` 를 다 쓴다. 이 플래그는 **캐치 사이클이 한 번도 열리지 않은 시행** (APPROACH 이후 모드를 본 적 없음) 을 공이 `--floor-z` + `--ball-low-margin-m` 아래로 내려온 시점부터 grace 만 더 기록하고 끝낸다. 열린 사이클은 닫힘이나 cap 으로만 끝난다. 켜면 시행 기록에 `end_reason` (`ball_low` · `cycle_closed` · `fault` · `cap`) 등이 생기고, 끄면 그 키는 없다.

**host 부하 감시 (`--host-watch`).** sim 이 벽시계보다 느리게 돌면 (RTF < 1 — 같은 host 의 빌드 · 테스트 등) 공 stamp 가 뒤처져 정상 입력이 `BALL_STALE` 로 끊기고, 성공률이 포구가 아니라 host 를 잰다. 러너는 sim 시간 `--host-window` 창 중 가장 느린 속도를 투척마다 재고 `--host-rtf-min` 아래면 부하로 본다.

| `--host-watch` | 부하를 본 투척에서 |
|---|---|
| `warn` (기본) | ERROR 로그를 남기고 계속 던진다. 시행 기록에 `host_rtf_min` · `host_lag_max_ms` · `host_busy`, `run_meta.json` 에 `host_watch` |
| `abort` | 그 투척까지 기록하고 런을 끝낸다 — **exit code 3** |
| `off` | 재지 않는다 (위 키가 생기지 않는다) |

성공률 판정에 쓰는 unit 은 `abort` 로 돌리고, exit code 3 이면 같은 seed 로 unit 전체를 다시 돌린다. `max_rtf` ≠ 1 로 띄운 sim 은 `--host-rtf-min` 을 맞추거나 `off` 로 한다. 같은 머신에서 빌드 · 테스트 · `ros2` CLI · 기록 분석을 같이 돌리지 않는다.

## 4. 결과 읽기

`<out>` 에 시행마다 `truth_trial_NN.csv` (공 ground truth), 전체 `trial_results.json`, `run_meta.json` 이 생긴다.

- `outcome` — 시행 판정 (RETREAT 진입 때 발행된 값). `cycle_closed` — 순환이 닫혔는가.
- `err_q_at_throw` — 투척 순간의 대기 자세 정렬 오차. `armed_at_throw` — 투척 직전 컨트롤러가 발행한 무장 상태.
- `truth_csv` — `<out>` 기준 파일 이름. `wall_t_relative_offset` — 이 기록을 `catching_diag.csv` 의 시간축에 잇는 오프셋 (sim 이 벽시계보다 느리면 흐른다).
- 같은 시행의 컨트롤러 쪽 기록은 세션 폴더 (`logging_data/<세션>/controllers/demo_catching_controller/`) 에 있고, 두 기록을 잇고 성공률을 내는 것은 `rtc_tools` 의 `catching_trials` 다 ([rtc_tools README](../rtc_tools/README.md)). `sim_lanes:=true` 로 띄웠다면 `<세션>/sim/{clock_lane,ball_contact_lane}.csv` 도 같은 세션에 남아 `catching_trials` 가 시행에 잇는다.
- 성공의 기준은 `final_outcome` 이 아니라 `catching_trials` 의 `truth_success` 다 ([catching_trials.py](../rtc_tools/rtc_tools/analysis/catching_trials.py)). `final_outcome` 은 마지막으로 발행된 label 이라 새 순환이 시작되지 않은 투척에는 앞 투척의 값이 남는다.

## 5. 포구 모집단의 sim 단계 (`catch_population_trials`)

포구 모집단 평가의 2 단계다. 1 단계 (`rtc_tools` 의 `catch_population`) 가 조합 (`<search>_<segment>`) 마다 남긴 묶음 파일 (`<결과 폴더>/<조합>/throws/chunk_NNN.json`) 을 sim 에서 던지고 잡은 투척을 모은다. **설치되는 명령이다** — git 저장소나 source tree 없이, ROS 와 이 workspace 를 source 한 환경에서 돈다. 설정 파일은 1 단계와 같은 `config/<robot>/catch_population.yaml` 이고 로봇은 그 파일이 놓인 폴더다.

```bash
# 긴 실행은 with_verify_hold.sh 로 띄운다 (묶음 사이에는 sim 이 없다 — 그 틈에 빌드 · 테스트가 시작되지 않게)
repo_scripts/scripts/with_verify_hold.sh \
  ros2 run integrated_bringup catch_population_trials \
    --config $(ros2 pkg prefix integrated_bringup)/share/integrated_bringup/config/ur5e_p1b/catch_population.yaml \
    --out <결과 폴더> --estimator-ws <추정기의 colcon workspace>
# 무엇을 띄울지만 본다 / 던지지 않고 다시 모은다
ros2 run integrated_bringup catch_population_trials --config <파일> --out <결과 폴더> --estimator-ws <ws> --dry-run
ros2 run integrated_bringup catch_population_trials --config <파일> --out <결과 폴더> --collect-only
```

| 인자 | 뜻 |
|---|---|
| `--config` · `--out` | 설정 파일과 1 단계의 결과 폴더 (필수). 결과 폴더의 `config.yaml` (1 단계의 사본) 과 `sim` 절 밖이 다르면 거부한다 |
| `--estimator-ws` | 추정기 (`ball_perception`) 의 colcon workspace. 기본은 `$BALL_SIM_WS`. 추정기를 띄우는 셸에서만 그 workspace 의 `install/setup.bash` 를 source 한다 |
| `--profile` | 추정기의 profile. 기본은 `$PROFILE`, 없으면 그 workspace 의 출하 catching profile. 그 `prediction.horizon_s` 가 설정의 `timing.prediction_horizon_s` 와 다르면 던지기 전에 거부한다 |
| `--arms` · `--seeds` | 던질 조합과 추정기 seed. 기본은 설정의 전부 · `sim.estimator_seeds` |
| `--max-chunks N` · `--throws-limit N` | 조합마다 묶음 0 … N−1 만 · 묶음마다 앞 N 발만 (smoke) |
| `--viewer on\|off` | MuJoCo viewer. 기본은 설정의 `sim.viewer` |
| `--domain-id` | 그 실행의 `ROS_DOMAIN_ID`. 기본은 `$EVAL_DOMAIN`, 없으면 88 |
| `--collect-only` | 던지지 않는다. 분석이 없는 unit 을 분석하고 결과 파일을 다시 만든다 |
| `--dry-run` | 그 실행의 unit 과 띄울 명령을 찍고 끝낸다 |

**묶음 하나가 unit 하나이고 sim 기동 한 번이다.** unit 은 `<결과 폴더>/sim/<조합>/seed_<seed>/chunk_NNN/` 이고 `status` (`DONE` \| `FAIL:<이유>`), `conditions.txt`, `mirror.txt`, `launch.log`, `est.log`, `trials.log`, `trials/`, `session/`, `ct/` 등을 남긴다. 한 unit 은 sim 기동 → 준비 확인 → 컨트롤러 미러를 출하값과 대조 → 추정기 기동 · 컨트롤러 전환 · `catching_sim_trials --throws-file <묶음>` → 종료와 프로세스 정리 → 세션 기록을 unit 으로 옮김 → 분석 (`catching_tc_vector`, sim 이 없을 때) 순으로 돈다. sim 은 viewer 가 있어도 world 를 바꾸는 키를 무시하게 (`viewer_input_enabled:=false`) 띄워지므로, **viewer 를 켠 실행 중에는 그 창에 키를 넣지 않는다.**

**이어 던지기와 재시도.** `DONE` 인 unit 은 건너뛴다. 실패한 unit 은 `<chunk>.failN` 으로 치우고 다시 던진다 — 명령을 띄울 때 한 번 (사람의 결정), 그 뒤로는 **rig 실패** (`host_busy` · `estimator not activated` · `startup` · `abnormal_start`) 일 때만 `sim.retries` 번까지. 그 밖의 실패 (`FAIL:mirror …` · `FAIL:trials rc=…` · `FAIL:switch` · `FAIL:sim exited` · `FAIL:contaminated …` · `FAIL:launch killed` · `FAIL:signal`) 는 실패한 자리에 남고 다음 unit 으로 간다. `<결과 폴더>/STOP` 파일이 있으면 다음 기동 전에 멈춘다. 진행은 `<결과 폴더>/sim/progress.log` 에 남는다. SIGINT · SIGTERM · SIGHUP 은 도는 unit 과 그것이 띄운 것을 끝낸다.

**결과** (조합마다, 실행 끝과 `--collect-only` 에서 다시 만든다):

| 파일 | 내용 |
|---|---|
| `<조합>/sim_throws.csv` | `DONE` 인 모든 unit 의 `ct/catching_trials.csv` 의 모든 행, 앞에 `arm,seed,chunk` 세 열. 투척 × seed 마다 한 행 |
| `<조합>/throw_catch_set.json` | `catching_throw_list/1` — 던진 투척 가운데 한 seed 에서라도 잡은 것 (`truth_success`). 투척마다 `caught_seeds` · `thrown_seeds`, meta 에 던진 집단과 sha256 · 던진 수 · 쌍 게시 수 · 포구 수. 잡은 투척이 없으면 파일을 쓰지 않는다 |

**catch 집단은 "그 seed 에서 잡은 투척" 이다.** 던지지 않은 투척은 놓친 투척이 아니다 — 집단에 없는 투척은 뽑히지 않았거나, 아직 던지지 않았거나 (`seeds[].chunks` · `chunks_total` · `partial_chunks`), 던졌으나 놓친 것이고 meta 가 그것을 가른다.

**다른 PC 의 unit 을 합친다.** unit 디렉토리를 이 결과 폴더의 `sim/<조합>/seed_<seed>/` 아래로 복사하고 `--collect-only` 를 돌린다. unit 의 `throws_file_sha256` 이 이 폴더의 묶음 파일 sha256 과 다르면 그 unit 을 이름으로 거부하고 아무것도 쓰지 않는다. sha256 은 **한 결과 폴더 안의 식별**이므로, 다른 PC 에서 던지려면 이 폴더의 묶음 파일을 바이트 그대로 가져가 던진다.

## 6. 평가 도구 (`tools/`)

`integrated_bringup/tools/` 의 스크립트는 **설치하지 않는다. 측정하는 workspace 의 source tree 에서 실행한다.** repo 는 스크립트의 자리에서, colcon workspace 는 그 두 단계 위에서 찾는다 (`RTC_WS` 로 다른 곳을 가리킨다). repo 가 `<workspace>/src` 아래에 없으면 `run_unit.sh` 는 거부한다 — unit 에 적히는 revision 이 빌드된 tree 의 것이어야 해서다. 도구는 repo 안에 아무것도 쓰지 않는다. 원자료 · 실험 overlay 는 repo 에 넣지 않는다. unit 이 도는 동안 같은 머신에서 다른 것을 돌리지 않는다 (`host_busy` 로 unit 이 버려진다).

### `tools/catching_eval/` — 포구 sim 평가

| 스크립트 | 사용법 |
|---|---|
| `run_unit.sh` | `DATA=<자료 디렉토리> BALL_SIM_WS=<추정기 ws> THROWS_FILE=<목록> run_unit.sh <out> <p1b\|leap> <overlay.yaml> <n (무시)> <seed>`. unit 하나: sim 기동 → 미러 대조 → 추정기 → `catching_sim_trials --throws-file … --host-watch abort` → 세션을 unit 으로 옮김. `<out>/status` 가 `DONE` 또는 `FAIL:<이유>`. tree 가 dirty 하면 거부한다 (`ALLOW_DIRTY=1` 로 푼다). `THROWS_LIMIT=N` 은 앞 N 발. 목록이 없으면 거부한다. seed 는 추정기에만 간다 |
| `run_all.sh` | `DATA=… PLAN=<plan 파일> run_all.sh`. plan 의 unit 을 차례로 돌린다. 다시 띄우면 `DONE` 은 건너뛴다. rig 실패만 `<dir>.fail<N>` 으로 치우고 같은 seed 로 `MAX_TRY` (기본 3) 번까지 다시 돌린다. `$DATA/STOP` 이 있으면 그 unit 뒤에 멈춘다. `with_verify_hold.sh` 로 띄운다 |
| `analyse_unit.sh` | `analyse_unit.sh <ur5e_p1b\|iiwa7_leap> <unit>...`. 끝난 unit 에 `catching_trials` 와 `tc_vector.py` 를 돌려 `<unit>/ct/` 에 쓴다. unit 을 모으는 중에는 돌리지 않는다 |
| `mk_overlay.py` | `mk_overlay.py <out.yaml> <p1b\|leap> <closed_form\|mpc\|mpc_docking> [grid\|nlp [tennis\|beanbag\|hard]] [--set <key>=<value>]…`. 한 arm 의 overlay (구간 mode, 선택적으로 탐색 mode · sim 의 공) 를 쓴다. `--set` 은 컨트롤러 값 하나를 더 적고 (key 는 `catching` 아래의 미러 이름), 출하 트리에 없는 key · 다른 타입의 값은 거부한다. `catch_population_trials` 는 overlay 대신 launch 인자를 쓰므로 이 도구는 `run_unit.sh` 용이다 |
| `summarize.py` | G-1 판정: `--cfg <config dir> --overlay-cf … --overlay-mpc … --cf <unit>… --mpc <unit>…`. N arm 의 기록: `--cfg <config dir> --unit <unit>… [--vary <미러>]… [--all-pairs] [--allow-mixed-build] [--json out.json]` (아무것도 판정하지 않는다) |
| `unit_report.py` | `unit_report.py <unit>... [--json out.json]`. unit 하나가 기록한 것을 그대로 낸다 (투척 수 · HOLD · ABORT_SAFE · `final_outcome` 별 수, 계획기 wake, 구간 풀이 표, RT 구간 lane). 판정하지 않는다 |
| `tc_vector.py` | `analyse_unit.sh` 가 부르는 shim (분석은 설치되는 `integrated_bringup.catching_tc_vector`) |
| `check_startup.py` · `check_segment_mode.py` · `expect_kv.py` | `run_unit.sh` 가 부르는 보조 (기동 판정 · mode 줄 대조 · 기대값 대조) |
| `make_conditions.py` | 예측 격자 조건의 profile · overlay · `conditions.tsv` · plan 을 `$DATA/conditions/` 에 쓴다. 이 plan 은 `run_all.sh` 의 plan 형식이 아니다 — 조건의 행을 `run_unit.sh` 에 손으로 넘긴다 |

`run_unit.sh` · `run_all.sh` 의 환경변수:

| 환경변수 | 뜻 |
|---|---|
| `DATA` | 평가의 자료 디렉토리. **기본값이 없다 — 없으면 거부한다** |
| `BALL_SIM_WS` | 추정기의 colcon workspace. **없으면 거부한다** |
| `PLAN` | `run_all.sh` 의 plan 파일. **없으면 거부한다** |
| `PROFILE` | 추정기 profile. 기본은 `BALL_SIM_WS` 의 출하 catching profile |
| `EXPECT_MODE` · `EXPECT_SEARCH` (기본 `grid`) · `EXPECT_KV` · `EXPECT_BALL` 등 | 컨트롤러 미러에서 읽어 대조할 값. 하나라도 다르면 `FAIL:overlay …` 로 거부한다. `EXPECT_KV` 는 `미러 이름=값` 을 `;` 로 이은 것 |
| `ARM` · `HOST_WATCH` · `EVAL_DOMAIN` | 시행 기록의 arm 라벨 (기본 `mpc`), `--host-watch` (기본 `abort`), `ROS_DOMAIN_ID` (기본 88) |
| `ENABLE_VIEWER` | `1` 이면 viewer 를 띄운다 (기본 `0`, headless) |
| `PROBE_DUMP` | `1` 이면 `rtc_tools` 의 `vision_lane_probe --dump` 를 unit 옆에서 돌린다 (기본 `0`) |
| `ONLY` · `MAX_TRY` · `RETRY_ON` · `IDLE_GRACE` · `IDLE_MAX_S` | `run_all.sh` — 접두어로 고른 unit 만, 재시도 횟수, 재시도할 실패 (status 에 대는 확장 정규식), unit 사이 대기 |

plan 의 한 줄은 unit 하나다 (`#` 로 시작하면 주석):

```text
<dir> <robot p1b|leap> <overlay 경로> <n> <seed> <mode mpc|closed_form|mpc_docking> [<미러 이름=값;…|-> [<search grid|nlp> [<공>]]]
```

조건 하나는 `conditions.tsv` 의 행을 `run_unit.sh` 에 손으로 넘겨 돌린다.

```bash
# conditions.tsv 의 행: cond robot profile overlay dt n_min points
DATA=<자료 디렉토리> BALL_SIM_WS=<추정기 workspace> \
PROFILE=<profile 열> \
EXPECT_KV='prediction.dt_expected=<dt 열>;io.n_min=<n_min 열>;planner.search.grid.slice.dt=<dt 열>' \
  integrated_bringup/tools/catching_eval/run_unit.sh <out> <robot 열> <overlay 열> <n> <seed>
```

### `tools/catch_frame/` — catch frame 확인

workspace 환경을 source 한 python 으로 실행한다.

```bash
python3 integrated_bringup/tools/catch_frame/verify_catch_frame.py <ur5e_p1b|iiwa7_leap> [--samples 12] [--seed 0]
python3 integrated_bringup/tools/catch_frame/show_catch_frame.py <ur5e_p1b|iiwa7_leap> --out <png> [--viewer] [--settle 6.0]
```

`verify_catch_frame.py` 는 MJCF 의 palm body frame 과 URDF 의 palm link frame 이 같은지 관절 표본 위에서 대조하고, 스크립트에 적힌 pocket 점을 부모 link offset 으로 옮긴 값 (`proposed_xyz_parent_frame_m`) 을 JSON 으로 낸다. 출하 YAML 의 `urdf.extra_frames.catch_frame` · `robot.hand.q_pre` 는 읽지 않으므로 출하 값과의 일치는 JSON 과 YAML 을 직접 대조한다. `show_catch_frame.py` 는 출하 catch frame 을 preshape 손 위에 그린다 (`--viewer` 는 육안 확인용 viewer).

### `tools/docking_ident/` — `mpc_docking` 의 손 · 포획 입력 식별

`mpc_docking` 이 받는 값 (통과 평면 · lateral 집합 · 속도 집합 · 폐쇄 창) 을 sim 에서 재는 도구다. 팔을 대기 자세에 세우고 손을 `q_pre` 에 정착시킨 뒤 출하 sim 공을 catch frame 의 직선을 따라 날리고, 정해진 시각에 `q_close` 를 명령해 유지되는가를 본다. 값을 YAML 에 옮기는 것은 도구의 일이 아니다 — 보고서를 읽고 사람이 `robot.hand.docking` · `T_close_lead` 에 적는다.

```bash
( cd <workspace> \
  && source <repo>/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 \
  && export DATA=<자료 디렉토리> OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  && python3 <repo>/integrated_bringup/tools/docking_ident/run_ident.py ur5e_p1b all \
  && python3 <repo>/integrated_bringup/tools/docking_ident/report.py ur5e_p1b )
```

- `run_ident.py <profile> <stage> [--workers 6] [--box w040]…` — 단계는 `selfcheck` · `map-coarse` · `map-fine` · `lateral` · `vperp` · `static` · `verify` · `accel` (`all` 은 이 순서로 전부). 단계마다 `$DATA/<profile>/` 에 저장하고, 끊겨도 이미 있는 것은 다시 날리지 않는다. `--box` 는 상자마다 날리는 단계 (`lateral` · `vperp` · `verify` · `accel`) 를 그 상자로 좁힌다.
- `report.py <profile>` — 저장된 판정에서 값을 계산해 보고서를 낸다. simulator 를 쓰지 않아 원자료에서 언제든 다시 계산된다.
- `rig_config.py` · `rig.py` — 위 두 스크립트가 쓰는 입력 읽기와 MuJoCo rig 이다 (직접 실행하지 않는다).
- `DATA` 가 없으면 거부하고, `OMP_NUM_THREADS=1` 이 아니어도 거부한다. **측정하는 동안 빌드를 돌리지 않는다.** store 의 결과가 이번 실행이 묻는 조건과 다르면 거부한다 — 상자나 규칙을 바꿨으면 그 단계부터의 store 를 치우고 다시 날린다.
- 테스트: `test/test_docking_ident_config.py` 는 colcon 에서 돈다. `test/test_docking_ident_rig.py` 는 mujoco 가 필요해 colcon 에서는 skip 이므로, rig 를 고쳤으면 venv 로 직접 돌린다 ([testing.md](testing.md#test-commands)).

## 7. 추정기 profile 과 맞춰야 하는 값

추정기가 쓰는 profile 은 ball_perception 저장소의 `ball_perception_sim/config/sim_profile.catching.json` 이다 (두 로봇 공용, 이 저장소는 사본을 두지 않는다). 이 저장소의 값 중 profile 과 맞아야 하는 것은 자동으로 검사되지 않는다. profile 을 바꾸면 함께 확인한다:

| profile | 이 저장소 | 관계 |
|---|---|---|
| `prediction.step_s` (0.05) · `horizon_s` (1.0) · `max_points` (20) | 컨트롤러 YAML (`catching:` 아래) `prediction.dt_expected` · `io.n_min` · `planner.search.grid.slice.dt` · `planner.search.grid.slice.t_max` | `dt_expected` = `slice.dt` = step, `n_min` = ⌈`io.horizon_min` / step⌉ + 1 ≤ 점 수 ≤ `kCap` 40. 떠 있는 컨트롤러의 값은 read-only 미러 파라미터로 읽는다 |
| `time.max_future_skew_s` (0.1) | 컨트롤러 YAML (`catching:` 아래) `sim.io.future_tol` | 같은 자릿수 — 공 토픽의 stamp 축이 sim 축이라 wall 을 위상 오차만큼 앞설 수 있다 |
| `measurement.position_covariance_m2` ((5 mm)² 대각) | `mujoco_simulator.yaml` `projectile_ball.publish.position_noise_stddev_m` | 짝 |

예측 지평 · stamp 의 수신 계약은 [traj_input.hpp](../integrated_bringup/include/integrated_bringup/controllers/catching/traj_input.hpp) 가,
공 토픽의 stamp 규약은 [rtc_mujoco_sim/README.md](../rtc_mujoco_sim/README.md) §Projectile Ball 의 stamp 항목이다.
