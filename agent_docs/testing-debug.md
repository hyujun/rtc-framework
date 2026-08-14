# Testing & Debugging

## Sensor Matrix (변경 유형별 필수 검증)

[CLAUDE.md](../CLAUDE.md) §5의 상세판. 변경 위치에 따라 필수 sensor + 추가 sensor를 실행한다.

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` | 전체 downstream ([invariants.md](invariants.md) PROC-3) |
| `rtc_math/` | `colcon test --packages-select rtc_math` (Pinocchio 발견 시 `test_se3_module` log6/Jlog6 교차검증 + 유한차분) | `se3_error_compare` S1–S5 실험 + `plot_se3_compare.py` (선택) |
| `rtc_msgs/` | 위 + `./build.sh full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | `colcon test --packages-select rtc_controller_interface` (registry·interface·mailbox·output-validation gtest) | downstream controller 빌드 |
| `rtc_controllers/` RT path | core 법칙 gtest (`test_*_core` + `test_dls_convergence` — ScopedAllocGate/ScopedNoMalloc 무장 TU) + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | (MPC CSV는 `<session>/controllers/<config_key>/...` 경로로 컨트롤러가 자체 기록) |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_timing_log.csv` p50/p99/max 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O, contact_wrench) | `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py` smoke. Contact wrench: `ros2 topic hz /<prefix>/<target>/contact_wrench` 후 fingertip 으로 객체 접촉 → magnitude 가시화 |
| `rtc_urdf_bridge/` | gtest (URDF/model parsing, xacro, chain extractor) | 실제 URDF 파싱 smoke |
| `rtc_inference/` | ONNX engine unit test | 실제 모델 로드 smoke |
| `rtc_communication/` | UDP loopback + CAN/CANFD loopback (vcan0 없으면 skip) + RS485 serial loopback (PTY, 상시 실행) + Transceiver lifecycle/decode/callback | vcan0 셋업 후 CAN 테스트 실행 (`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`), RS485는 PTY라 셋업 불필요, 실제 HW UDP/CAN/RS485(USB-RS485+Dynamixel) 테스트 (선택) |
| `rtc_digital_twin/` | pytest + RViz2 smoke | `/rtc_cm/{group}/joint_states` hz |
| `rtc_tools/` (pytest) | pytest. **`robot_descriptions/` 의 MJCF/URDF 를 고쳤다면 `test_real_model_pairs.py` 가 실제 게이트다** — `robots/model_pairs.yaml` 의 쌍마다 `compare_mjcf_urdf` 를 발사한다 (#392). 로컬 colcon 은 pytest 를 `/usr/bin/python3` 로 돌려 mujoco 를 못 보므로 **skip 된다** — 실제로 돌리려면 `PYTHONPATH=rtc_tools:$PYTHONPATH .venv/bin/python -m pytest rtc_tools/test/test_real_model_pairs.py` | GUI/plot 수동 smoke |
| `repo_scripts/` | `test_rt_common` + `test_install_deps` (ONNX tarball digest 검증) + shell unit test | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape_estimation/snapshot` topic echo |
| `integrated_bringup/` demo FSM | demo_wbc FSM/integration/output + demo_joint grasp·URDF paths + demo_task CLIK/contact_stop (URDF-backed iiwa7_leap fixture 공유, 노드 비생성) + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `udp_hand_driver/` | 단위 gtest (hand_packets, codec, FT, failure detector) + UDP loopback | `ros2 topic hz /p1a/joint_states` (ur5e_p1a; 드라이버 standalone 기본은 `/hand/`) |
| `ur5e_bt_coordinator/` | BT gtest — Tier-2 는 inject(DDS-free, `inject_fixture.hpp`)/e2e(real-DDS, `test_helpers.hpp`) 분리, suite 목록은 CMakeLists `TIER2_INJECT`/`TIER2_E2E` (#154) | 실제 grasp 시나리오 smoke |
| Launch (`integrated_bringup/launch/*.py`) | `colcon test --packages-select integrated_bringup --ctest-args -R 'test_launch_'` — 평가 센서 `test_launch_description_evaluates` (5개 launch × 4 인자 조합을 `LaunchContext` 만으로 실제 평가; ROS 그래프 불요, ~1.4 s) + AST 센서 `test_launch_shield_wiring` · `test_launch_hand_affinity_wiring` (어느 헬퍼를 부르는가). **import 스모크(`rtc_tools/test/test_launch_imports.py`)로는 `OpaqueFunction` 본문이 안 돈다** — #397 이 그 틈으로 sim 3개를 죽인 채 전 배터리 green 이었다 | 짧은 실기/sim smoke (`ros2 launch ...`) |
| Launch / YAML config | 위 + 변경 YAML parse | config 로드 검증 |
| Threading (`ApplyThreadConfig`) | `rtc_base` thread-config gtest + RT perms | `check_rt_setup.sh --summary` |
| RT 회귀 의심 / RT path 미상 | `mpc_timing_log.csv`·`cm_timing_log.csv` p99 검사 | `enable_tracing:=true` + Perfetto 분석 (§Tracing) |
| RT host 환경 검증 | `check_rt_setup.sh --summary` + `verify_rt_runtime.sh` | `cyclictest --mlockall --smp -p 80 -i 200` / `rtla osnoise top` — [invariants.md](invariants.md) §RT Host / Runtime Preconditions RT-HOST-1~3 |

## Test Commands

```bash
# All tests
colcon test --event-handlers console_direct+
colcon test-result --verbose

# Single package
colcon test --packages-select ur5e_bt_coordinator --event-handlers console_direct+

# Single test (C++)
colcon test --packages-select rtc_controllers --ctest-args -R test_grasp_controller

# Single test (Python)
colcon test --packages-select rtc_digital_twin --pytest-args -k test_urdf_parser
```

## Revert-verification — 새 가드를 추가했을 때

가드(검증·거부 경로)와 그 테스트를 함께 추가하면 **테스트 통과는 가드가 동작한다는 증거가 아니다** — 그 가드를 지워도 통과할 수 있다. 추가한 가드마다 **하나씩 원복 → 대응 테스트가 실제로 실패하는지 확인 → 복구**. [CLAUDE.md](../CLAUDE.md) §5.5 의 "에이전트 자기 평가는 신뢰 불가" 가 자기가 쓴 테스트에도 그대로 적용된다.

**가장 흔한 false green — 층이 겹치는 가드.** 새 가드 A·B 가 같은 잘못된 입력을 모두 거부하면 B 의 테스트는 A 에 흡수되어, B 를 통째로 지워도 통과한다. B 는 커버리지가 있는 것처럼 보이면서 실제로는 자유롭게 삭제 가능한 상태다. 이때는 테스트를 "throw 하는가" 가 아니라 **그 가드만이 만드는 관측 가능한 차이** (진단 메시지 내용, 거부 시점, 부작용 유무) 로 옮겨야 pin 이 성립한다.

> 실측 (#204): 신규 가드 4개 중 flat `publish:` 탐지가 false green 이었다 — 원복해도 전 테스트 통과. group-shape 가드가 같은 config 을 이미 거부하고 있었다. 테스트를 "마이그레이션 진단을 주는가" 로 옮겨 pin 을 성립시켰다. 나머지 3개는 각각 자기 테스트만 정확히 실패. 거부 시점을 pin 한 예도 같은 PR 에 있다 — backend `Configure()` 카운터로 "controller 는 생성됐지만 device wiring 전" 경계를 고정.
>
> **같은 false green 이 형제 절반에서 재발했다 (#204 post-review).** 위 수정은 flat 탐지의 `publish` 쪽만 pin 했고, `subscribe` 쪽은 `EXPECT_THROW` 로 남아 똑같이 원복해도 통과했다. 층이 겹치는 가드를 하나 고쳤으면 **대칭 위치의 나머지 절반도 같은 기준으로 다시 측정**한다 — 한쪽을 진단 pin 으로 옮겼다는 사실 자체가 다른 쪽도 흡수되고 있다는 신호다.

**두 번째 false green — 관측 채널이 fallback 에 가려질 때.** 가드를 원복(mutation-check)해도, 테스트가 assert 하는 값이 다른 경로로도 같은 값을 내면 vacuous 다 — 이때는 가드가 아니라 *관측 지점*이 잘못된 것이다. E-STOP recovery drain(#242)이 그 예: OSC/TaskImpedance 는 gravity-comp 컨트롤러라 정지(q̇=0) 시 hold torque ≈ ĝ(q) 인데, E-STOP 중 큐잉돼 leak 된 target 이 충분히 멀면 recovery tick 에서 SAFE_STOP 을 latch 시키고 그 hold 역시 ĝ(q) 를 내므로 **joint torque 로 assert 하면 leak 유무와 무관하게 통과**한다. 관측을 fallback 이 건드리지 않는 채널로 옮겨야 pin 이 성립한다 — OSC 는 goal echo(`task_goal_positions`, drain 이 쓰는 slot 을 직접 반영), TaskImpedance 는 `diag.pose_error`(SAFE_STOP step *이전에* 기록되고 `ComputeEstop` handoff 로 보존). 실측(#242): torque assertion 으로 두 번 vacuous 를 거친 뒤 채널을 옮겨 pin.

**세 번째 false green — 게이트가 닫힌 쪽에서 *수치적으로 inert* 할 때.** "게이트를 닫은 케이스를 넣었다" 는 것만으로는 그 게이트가 고정되지 않는다. 게이트를 지웠을 때 실행되는 경로가 **닫힌 것과 같은 값**을 내면 출력 대조는 원복해도 통과한다. 실측 (#236 S5): `nullspace_active = (nv > 6) && (nullspace_kp != 0.0)` 에서 `&& nullspace_kp != 0.0` 을 지워도 비트 레인 **전부 green** — `nullspace_kp = 0` 이면 게이트 없는 경로가 `0.0·Δq` 의 signed zero 를 만들고, 그것을 사영해 더하는 것은 exact no-op 이기 때문이다. 게이트 자체를 관측하는 출력 (여기서는 `diag.nullspace_active`) 을 tick 마다 대조하도록 옮겨야 pin 이 성립한다. **곱셈으로 꺼지는 게인·플래그 (`k = 0`, `α = 0`) 는 전부 이 형태**이므로, 그런 게이트는 출력이 아니라 진단 플래그로 pin 한다 — 그리고 그 플래그는 `EXPECT_FALSE` 만으로는 배선이 고정되지 않으니 양성 케이스를 함께 둔다.

원복은 **파일 단위 restore** 로 되돌린다 — `git checkout -- .` 은 아직 커밋하지 않은 작업까지 함께 날린다 (#204 에서 실제 발생).

단 **검증 대상 파일 자체가 미커밋일 때** (가드를 방금 썼고 아직 커밋 전 — revert-verification 의 표준 상황) 는 `git checkout -- <file>` 도 그 작업을 날린다. 명시적 백업 사본에서 복구해야 하는데, 여기에 함정이 하나 더 있다:

> **mtime 을 보존하는 복사로 복구하면 (`cp -p`, `shutil.copy2`) make 가 재컴파일을 건너뛴다.** 복구된 소스는 최신인데 빌드 트리에는 **원복된 바이너리**가 남아, 그 상태로 테스트하면 결과를 반대로 읽는다 (#204 post-review 에서 실제 발생 — 복구 후 "2 failures" 를 보고 회귀로 오독). 초록으로 오독되는 방향도 똑같이 가능하다. 백업 복구 후에는 반드시 `touch <파일>` 하고 재빌드한다.

```bash
# 사전: 미커밋 작업이 있으면 명시적 백업
cp <파일> /tmp/<파일>.orig          # -p 금지 (mtime 보존 → 아래 touch 를 잊으면 stale 바이너리)

# 가드 1개 원복 → 빌드 → 해당 실행파일만 → 복구
colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select <pkg> --ctest-args -R <test_exe>
cp /tmp/<파일>.orig <파일> && touch <파일>   # 미커밋 작업이 없으면 git checkout -- <파일>
```

빌드가 실제로 돌았는지는 `colcon build` 의 소요 시간으로 확인된다 — 0.x 초로 끝났으면 재컴파일이 안 된 것이다.

## Test fixtures — robot URDF 해석

robot 모델이 필요한 gtest fixture 는 URDF 를 **`robot_descriptions/robots/<name>/`** (repo 체크인) 에서 해석한다. **`deps/src/...` 나 `/usr/local/...` 경로를 박지 말 것** — `build-isolated-deps` 가 CI 아티팩트에서 `deps/src` 를 삭제하고 (`deps/install` 만 업로드), `/usr/local` 은 deps 격리 정책상 부재라, 두 경로 모두 CI 에서 fixture `buildModel` throw 또는 `GTEST_SKIP` → 0 coverage 를 유발한다 (rtc_tsid/rtc_mpc/integrated_bringup 에서 실제 발현, panda.urdf vendor 로 해소). 해석 방식: compile macro (`RTC_PANDA_URDF_PATH`, CMake 가 repo-상대 `robot_descriptions` 경로 주입, env/`-D` override) 또는 `ament_index_cpp::get_package_share_directory("robot_descriptions")`.

> **CI install set ≠ local — cross-package ament lookup 은 stub/mock 필수.** CI **Python Test** 잡은 `rtc_tools` / `rtc_msgs` / `rtc_digital_twin` 만 install/source 한다. `rtc_tools` 테스트가 **다른 패키지**를 `ament_index` 로 resolve 하면 (예: `get_package_share_directory("repo_scripts")` — 런치 pinning 의 `rt_common_path()` / `cpu_shield_path()`) CI 에서만 `PackageNotFoundError` 로 실패한다. **로컬은 전 패키지가 install 돼 있어 통과하므로 이 회귀는 로컬 sensor 로 안 잡힌다** (#151 에서 실제 발현). 해석 경로 자체가 아니라 렌더된 산출물(bash snippet 등)만 검증하는 테스트는 경로 헬퍼를 monkeypatch 로 stub 한다 — production 런치는 전 패키지 install 상태라 무해. (동일 축의 C++ 판이 바로 위 URDF fixture 함정.)

## 비동기 결과를 기다리는 법 — sleep 대기와 executor pump 는 다른 원시다

고정 sleep 대신 **관측 가능한 진행** (tick / solve / recv 카운터) 을 폴링한다. 공유 헬퍼는 `rtc::testing::WaitUntil` (`rtc_base/test/include/rtc_base/testing/wait_until.hpp`, `rtc_base/test/test_wait_until.cpp` 가 계약을 pin) 이며, 설치되지 않으므로 소비자는 `../rtc_base/test/include` **소스 트리 경로**로 가져온다 (`no_malloc_scope.hpp` 와 동일 레이아웃; ament symlink install 이 `install(PATTERN EXCLUDE)` 를 무시해 `include/` 에 두면 런타임 트리로 실려 나간다).

이 헬퍼는 **오직 잔다**. Executor 를 pump 해야 하는 테스트는 자기 TU 에 local spin 헬퍼를 두며, 둘을 섞지 않는 것이 [invariants.md](invariants.md) **PROC-8** 이다 (근거·양쪽 실패 모드·자동 gate 는 그쪽이 SSoT).

Suite 고유의 poll 예산이 있으면 헬퍼를 감싸지 말고 **인자로 넘긴다** — `WaitUntil(pred, timeout, poll)`. 예산이 assertion 옆에 보이는 편이 낫고, 같은 이름의 wrapper 는 `using namespace rtc::testing;` 이 들어오는 순간 모호해진다. 호출 지점이 많아 예산을 한 곳에 묶어야 한다면 **다른 이름**으로 얇게 감싸고 그 값의 근거를 상수 옆에 남긴다 (`udp_hand_driver` 의 `PollUntil` = CommLoop 한 tick).

## RT-1 zero-allocation 게이트 — 두 종류이고 서로의 맹점을 덮는다

RT tick 이 heap 을 안 만진다는 주장(RT-1)을 재는 sensor 는 **두 개**이고, 어느 것을 쓸지는 취향이 아니라 두 질문으로 결정된다: **(a) 측정 대상이 Eigen 을 쓰는가, (b) 측정 대상 코드가 테스트와 같은 TU 에 인스턴스화되는가.**

| 게이트 | 보는 것 | 못 보는 것 |
|---|---|---|
| `rtc::testing::ScopedAllocGate` (`rtc_controllers/test/include/rtc_controllers/testing/alloc_gate.hpp`) — 전역 `operator new` 교체 | `operator new` 를 타는 모든 할당 (`std::vector`, header-inline helper, 다른 TU 포함) | **Eigen 할당 전부** — `internal::aligned_malloc` 이 `std::malloc` 을 직접 부르고 `operator new` 를 타지 않는다 |
| `rtc::testing::ScopedNoMalloc` (`rtc_base/test/include/rtc_base/testing/no_malloc_scope.hpp`) — `eigen_assert` + `EIGEN_RUNTIME_NO_MALLOC` | Eigen 동적 할당 (Release 에서도 유효) | non-Eigen heap; **그리고 다른 TU 의 Eigen** — 이 매크로는 정의된 TU 안의 Eigen inline 만 계측한다 |

따라서:

- **순수 Eigen 코어 (header-inline 법칙)** → **둘 다** 무장한다. 하나만 쓰면 가장 유력한 RT-1 회귀 (인자·임시가 `Eigen::VectorXd` 같은 runtime-sized 로 퇴화) 가 green 으로 통과한다. `test_task_accel_core` / `test_task_vel_core` 가 이 형태.
- **Eigen-free 코어** → `ScopedAllocGate` 만. Eigen 트립와이어는 여기서 진짜로 vacuous 하다 (`test_joint_pd_core`).
- **컨트롤러 `Compute()` 처럼 라이브러리 TU 에 컴파일된 코드** → `ScopedAllocGate` 만. Eigen 트립와이어는 관측 대상이 다른 TU 라 vacuous 하다 (integrated_bringup 의 `test_task_dls_convergence` — 바인딩 `compute.cpp` 경로가 이 형태).

**게이트는 반드시 RAII 로 무장한다** — `g_alloc_active = true; … = false;` 같은 맨 대입은 측정 구역에 `ASSERT_*` 가 들어오는 순간 disarm 이 실행되지 않고, 읽는 곳이 없으므로 이후 모든 테스트가 계수되는 상태가 조용히 남는다.

**추가한 게이트는 mutation 으로 fail-closed 를 확인한다** — 측정 구역에 `std::vector<double>`(→ operator-new 게이트만 발동) 과 runtime-sized `Eigen::VectorXd`(→ Eigen 게이트만 발동) 를 각각 넣어 본다. `new` + 즉시 `delete` 쌍은 컴파일러가 elide 하므로 mutation 이 성립하지 않는다 (외부 sink 로 값을 흘려야 한다).

`alloc_gate.hpp` 는 교체 `operator new` 를 **정의**하므로 바이너리당 정확히 한 TU 에서만 include 한다 (두 번째 TU 는 링크 에러 — fail-closed). 해당 타깃에는 `-Wno-mismatched-new-delete` 가 필요하다 (`new`→`malloc` / `delete`→`free` 짝에 GCC 가 program-wide false-positive).

## Test 측정

테스트 카운트·suite 목록은 박제하지 않는다 ([anti-patterns.md](anti-patterns.md) AP-DOC-1). 최신 카운트·suite 명은 직접 측정:

```bash
rm -rf build/<pkg>/test_results                 # ← 누적 XML 제거 (아래 함정)
colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

**계측 함정 — 비교 가능한 수치를 낼 때 필수:**

- **`colcon test-result` 는 디렉토리에 남아 있는 XML 을 전부 합산한다.** CMakeLists 에서 타깃을 빼거나 브랜치를 되돌린 뒤 재측정하면 **사라진 타깃의 옛 결과가 그대로 더해진다** — 숫자가 그럴듯해서 자체 검산 없이는 안 걸린다 (#236 슬라이스 3 에서 baseline 을 298 대신 300 으로 오측하고 존재하지 않는 drift 원인까지 보고했다).
- **`--test-result-base` 의 *범위*가 총계를 바꾼다 — 같은 트리, 같은 실행인데도.** `build/<pkg>` 를 주면 `build/<pkg>/Testing/<타임스탬프>/Test.xml`(CTest, 실행마다 **새 디렉토리로 누적**)까지 합산하고, `build/<pkg>/test_results` 를 주면 gtest/pytest/lint XML 만 센다. 실측: `integrated_bringup` 이 각각 **639 / 599**. 어느 쪽도 틀리지 않았고 **단위가 다를 뿐**이므로, 회귀 비교는 반드시 **같은 범위**로 한다. 옛 수치와 안 맞을 때 stale 로 단정하기 전에 범위부터 맞춰 볼 것 — 실제로 이 차이를 stale XML 로 오진한 적이 있다.
- **gtest case 수와 ctest entry 수를 함께 센다.** `rtc_controllers 333` = gtest 315 + ctest 18. 옛 수치와 비교할 땐 **단위가 같은지** 먼저 확인한다.
- **lint 도 소스 파일당 1 entry 를 낸다.** `cppcheck.xunit.xml` 의 `tests="N"` 은 그 패키지의 소스 파일 수이고 전부 **skipped** 로 집계된다. 따라서 테스트 `.cpp` 를 한 개 추가하면 총계는 **1(케이스가 1개일 때) + 1(ctest 타깃) + 1(cppcheck 파일)** 로 오르고 skipped 도 +1 이 된다 — 델타를 gtest 케이스 수만으로 예측하면 매번 어긋난다. 증감을 "삭제·신설 목록과 1:1" 로 설명해야 하는 슬라이스에서는 이 세 항을 분리해 적는다.

- **timeout·crash 는 gtest XML 에 `<failure>` 로 안 남는다 — CTest `Test.xml` 에만 잡힌다.** 타임아웃난 바이너리는 죽을 때 자기 `--gtest_output` XML 을 **아예 못 쓰거나 부분만 쓰므로**, `test_results/*.xml` 을 `<failure>` 로 훑는 검사는 그 런을 **clean 으로 판정한다**. 실측 (#345 검증): 22패키지 병렬 `colcon test` 에서 `ur5e_bt_coordinator::test_condition_nodes` 가 60 s CTest timeout 을 냈는데(단독 재실행 **0.47 s** 통과 — 부하 flake), gtest XML 전체에 `<failure>` 가 **0건**이었다. 따라서 **판정은 `colcon test-result` 의 요약(errors 를 포함)이나 `build/<pkg>/Testing/*/Test.xml` 로 하고**, gtest XML 직접 grep 을 유일 센서로 쓰지 않는다. 위 `--test-result-base` 범위 항과 짝이다: `build/<pkg>` 범위여야 `Test.xml` 이 합산에 들어온다.
- **`--test-result-base <경로>` 를 기본값 아닌 곳으로 주면 총계가 조용히 바이너리 수로 줄어든다.** 각 gtest 타깃의 `--gtest_output=xml:...` 경로는 **configure 시점에 `build/<pkg>/test_results/` 로 박히므로** 이 옵션을 따라오지 않는다. 커스텀 경로에는 CTest 의 `Test.xml` 만 떨어지고, `colcon test-result --test-result-base <그 경로>` 는 그것만 합산해 **바이너리 1개당 1건** 을 보고한다 (실측: 189 대신 20). 실패가 아니라 *그럴듯하게 작은 수*라 자체 검산 없이는 회귀로 오독하기 쉽다. CLAUDE.md §9.1 cwd drift 를 피하려고 절대경로 result-base 를 습관화하면 정확히 이 함정을 밟는다 — cwd 는 `cd <rtc_ws> &&` 로 고정하고 **result-base 는 건드리지 않는 것**이 맞다. 굳이 분리하려면 판정을 `build/<pkg>/test_results/*.gtest.xml` 의 per-file `tests="N"` 으로 한다.

신규 테스트 개수를 주장할 땐 총계 차이가 아니라 `grep -c '^TEST(' <파일>` 또는 per-target XML 의 `tests="N"` 으로 교차검증한다.

대표 suite 명은 `<pkg>/CMakeLists.txt` 에서 `ament_add_gtest()` / `ament_add_pytest_test()` grep — 코드 자체가 SSoT 이므로 문서 박제 불필요.

### Coverage 측정 (gcov/gcovr)

build.sh wrapper 는 없고, runtime PC 에 `lcov`/`gcovr` 가 없을 수 있다 (`gcov` 만 존재). gcovr 은 venv 에 설치 (`pip` 부재 — venv 는 `uv` 기반):

```bash
source repo_scripts/scripts/setup_env.sh
uv pip install gcovr                           # 분석 전용 도구 — runtime 에 영향 없음
# coverage 빌드: 패키지 CMakeLists 의 ENABLE_COVERAGE 옵션 사용 (--coverage 플래그 주입)
colcon build --packages-select <pkg> --cmake-args -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug
find build/<pkg> -name '*.gcda' -delete         # 누적 카운트 초기화 (정확한 측정)
colcon test  --packages-select <pkg>
SRC=src/rtc-framework/<pkg>
gcovr --root "$SRC" --object-directory build/<pkg> --filter "$SRC/src/" --print-summary
# 측정 후: clean Release 재빌드로 install tree 원복 (coverage 빌드가 install 을 덮음 →
# downstream 이 gcov 심볼을 링크하게 됨)
rm -rf build/<pkg> install/<pkg> && colcon build --packages-select <pkg>
```

> `ENABLE_COVERAGE` 옵션은 패키지 CMakeLists 에 개별 정의한다 (보유 패키지: `grep -rl ENABLE_COVERAGE src/rtc-framework/*/CMakeLists.txt`). 미보유 패키지 측정 시 동일 `option(ENABLE_COVERAGE ... OFF)` + `add_compile_options(--coverage ...)` 블록을 추가한다. header-only 패키지는 `--filter "$SRC/include/"`, src 기반은 `--filter "$SRC/src/"`.

> **Python (ament_python) 패키지는 gcov/gcovr 가 아니라 `coverage`** (venv 설치, `uv pip install coverage`): `gcovr` 은 C++ 전용이라 `.py` 를 계측하지 못한다. CMakeLists/`ENABLE_COVERAGE` 도 무관 (ament_python 은 CMake 없음 — colcon 이 `test/test_*.py` 자동 발견). 측정: `cd <pkg> && python3 -m coverage run --source=<pkg_module> -m pytest test/ -q && python3 -m coverage report -m`. 측정 후 `.coverage` 아티팩트 삭제. 보유 Python 패키지: `rtc_digital_twin`, `rtc_tools`.

> **`rtc_mpc` 측정 함정**: 위 recipe 의 `-DCMAKE_BUILD_TYPE=Debug` 를 `rtc_mpc` 에 그대로 쓰면 안 된다. (1) Debug 빌드가 proxsuite all-zero-C assert 를 표면화한다 (Release 는 NDEBUG 로 숨김 — `feedback_proxsuite_zero_c_rejection`). (2) aligator 0.19.0 의 contact_rich MPC 는 `LD_PRELOAD=$DEPS/install/lib/libmimalloc.so` 없이는 `free(): invalid pointer` 로 죽는다 (`feedback_aligator_requires_mimalloc`). `ENABLE_COVERAGE` 옵션 추가 자체(default OFF)는 무해하나, 실제 측정은 두 우회를 모두 적용해야 한다.

> **`integrated_bringup` 측정 함정 — Debug 에서 `test_demo_task_controller` 가 abort 한다.** `NonFiniteJacobianHoldsInsteadOfSolvingWithAStaleInverse` 가 의도적으로 non-finite Jacobian 을 주입하는데, 그 결과 회전행렬이 비유니터리가 되어 pinocchio `rpy.hxx` 의 `assert(R.isUnitary())` 가 발동한다 (Release 는 NDEBUG 로 무력 — 위 proxsuite 항과 **같은 범주**: Debug-only assert 가 coverage 빌드에서만 표면화). 증상은 그 바이너리 하나가 통째로 죽어 `test-result` 총계가 **그 바이너리의 케이스 수만큼** 줄고 gcda 도 안 남는 것이다 (절대 수치는 박제하지 않는다 — 그 XML 의 `tests="N"` 을 직접 읽는다). **coverage run 의 이 실패는 회귀가 아니므로 자기 변경 탓으로 오진하지 말 것** — 판정은 Release 재실행으로 한다.

> **정규 트리를 오염시키지 말고 `--build-base`/`--install-base` 를 분리하라.** 위 recipe 의 "측정 후 clean Release 재빌드" 대신, coverage 빌드를 **scratchpad 절대경로**의 별도 트리로 내보내면 (`--build-base /tmp/.../cov/build --install-base /tmp/.../cov/install`) ws-root incremental cache 가 애초에 안 더러워지므로 원복 재빌드가 불필요하다 (CLAUDE.md §9.1 은 그대로 — 호출은 여전히 ws root 에서). 이때 `gcovr --object-directory` 도 그 트리를 가리켜야 한다. `gcovr` 없이 `gcov` 만 있을 때는 `gcov -b -p <build>/CMakeFiles/<target>.dir/<path>/<file>.cpp.gcno` — 확장자 포함 `*.cpp.gcno` 를 **직접** 넘겨야 한다 (`-o <dir>` + 소스 경로 조합은 `<file>.gcno` 를 찾아 "cannot open notes file" 로 실패).

**LifecycleNode 노드(예: `rtc_controller_manager`) 유닛 커버리지 패턴**: `on_configure` 파이프라인 (params 로딩·device backend·publisher 생성) 과 private RT 헬퍼(`CheckTimeouts`/`CreateDeviceBackends`)는 friend accessor (`rtc::ControllerLifecycleTestAccess`, 헤더의 `friend` 선언으로 이름 고정) 로 private 멤버를 주입·호출해 **실 robot/RT 권한 없이** 구동한다 — `CreateDeviceBackends` 등은 `group_slot_map_`/`device_name_configs_` 주입 + registry fake backend 만으로 동작하고, `on_activate` 의 RT 루프는 `ApplyThreadConfig` 반환값을 버리므로 (SCHED_OTHER fallback) 테스트 샌드박스에서도 안전하다.

> **ROS 노드를 만드는 테스트는 전용 `ROS_DOMAIN_ID` 로 격리** (`ament_add_gtest(... ENV ROS_DOMAIN_ID=<n>)`): `colcon test` 는 패키지를 병렬 실행하므로, 노드를 생성·소멸하는 테스트가 기본 도메인 0 을 공유하면 discovery 버스와 Fast DDS SHM port 객체 (`/dev/shm/fastrtps_port<N>` + 그 named mutex) 를 함께 쓴다. 피해는 두 층이다 — **타 패키지의 pub/sub 매칭 테스트가 깨지고** (PR #187: `ur5e_bt_coordinator` `test_rewire_gate` 가 피해자, 실패 시각이 가해 테스트 창과 일치), 심하면 **rmw endpoint 생성·소멸에서 그대로 멈춘다** (#401: ctest 가 60 s 에 죽여 결과 파일이 안 남는다. 부하 하 재현율 7/18, 격리 후 0). 후자는 "느려짐" 이 아니라 전 스레드 `futex_do_wait` 정지이므로 timeout 상향으로는 안 없어진다.
>
> **배정 단위는 패키지 하나에 도메인 하나**다 — colcon 이 병렬화하는 것은 패키지이고 한 패키지 안에서 ctest 는 직렬로 돌기 때문에, 같은 패키지의 타깃들이 한 번호를 공유하는 것은 안전하다. **번호를 손으로 고르고 주석에 "어디까지 찼다" 를 적는 방식은 실패했다** — `udp_hand_driver` 가 "44-52 are taken" 을 근거로 53 을 골랐는데 `integrated_bringup` 이 이미 쓰고 있었다 (#401). 현재 배정은 `python3 repo_scripts/scripts/validate_test_domains.py --list` 가 소스에서 **파생**하고 (박제된 표는 없다 — AP-DOC-1), 같은 스크립트가 `repo_scripts` 의 `test_domain_allocation` 으로 등록되어 차단한다. 값은 반드시 literal 이어야 한다 — `ENV ROS_DOMAIN_ID=${VAR}` 는 게이트가 못 보므로 그 패키지가 아무것도 주장하지 않은 것처럼 보인다.
>
> **이 규칙은 이제 강제된다 — 그전까지는 문서에만 있었고 지켜지지 않았다.** 게이트가 원래 보던 것은 "주장된 번호가 겹치는가" 뿐이라 **아무것도 주장하지 않은 테스트들이 전부 도메인 0 을 공유하는 경우**가 안 보였다. #401 이 `ur5e_bt_coordinator` 를 55 로 뺀 뒤에도 노드를 만드는 타깃 15개가 5개 패키지에 걸쳐 0 에 남아 있었다 (피해자만 버스에서 뺐고 버스는 그대로였다). 지금은 게이트가 **테스트 소스**를 읽어 (`rclcpp::init(` / `rclpy.init(`, fixture 헤더까지 include 추적) 참가자를 여는 타깃이 도메인을 주장하지 않으면 차단한다. **주석·docstring 의 산문은 코드가 아니다** — 첫 실행에서 `thread_config.hpp` 의 `//` 주석 하나와 launch 테스트의 docstring 이 오탐 5건을 냈다.
>
> **`ament_python` 패키지는 substrate 가 다르다** — CMakeLists 가 아예 없어 `ENV` 를 못 쓴다. claim 은 `test/conftest.py` 의 `os.environ["ROS_DOMAIN_ID"] = "<n>"` 로 적고 (pytest 가 테스트 모듈보다 먼저 import 하므로 `rclpy.init()` 전에 잡힌다), 게이트가 그 값을 같은 배정 공간에서 검사한다 — python claim 이 CMake claim 과 충돌하면 red 다.

**`.venv` 격리 원칙 (Hard rule)**: `.venv`는 runtime PC가 본 workspace 외에 다른 control project들과 공존하는 환경에서 workspace dependency를 격리하기 위한 **의도된 설계**다. `colcon test` / `colcon build` / `ros2 run` / `ros2 launch`가 venv 활성 상태에서 실패하면 **반드시 근본 원인을 해결**한다 (sys.path / shebang / wrapper / dep resolution 디버그). gtest 바이너리 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등은 **금지** — 격리를 무력화해 runtime PC에서 다른 project의 site-packages가 끼어들면 silent breakage. 신호 (`Testing/Temporary/LastTest.log` Start/End 동일 초)가 재발하면 `env -i` 깨끗한 셸에서 `setup_env.sh` source 후 `sys.path` 순서 점검부터.

## Live Debug Topics

런타임 문제 탐지용 토픽. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상.

| Topic | 발행 주체 | 언제 보나 |
|-------|----------|----------|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 파악 (timeout name / trigger thread) |
| `/rtc_cm/active_controller_name` | 동일 (TRANSIENT_LOCAL) | Controller switch 확인. BT / GUI / digital_twin은 이 토픽으로 리와이어 |
| `/<config_key>/<config_key>/get_parameters` (srv) | active 데모 컨트롤러의 LifecycleNode | Runtime gain 값 조회 (`ros2 param get`) |
| `/forward_position_controller/commands` | **robot 모드 + `ur_driver_native` backend 전용** | RT loop 건강성 — `ros2 topic hz` 로 설정된 `control_rate` (default 500 Hz) 매칭 확인. **sim 에는 이 토픽이 없다** — sim 의 커맨드 lane 은 device group 당 하나이고 (`devices.<group>.backend.command_topic`, 예: `/ur5e/joint_command` + `/p1a/joint_command`) robot 당 하나가 아니므로, sim 에서 이 토픽의 침묵을 "RT loop 정지" 로 읽으면 오진이다 |
| `<session>/timing/cm_timing_log.csv` | CM RT loop @ `control_rate` (`rtc::ThreadTimingProducer<RtTickTimingPayload>`) drained by `DrainLog()` log thread | RT loop per-tick timing — 8 cols `t_wall_ns,tick_count,run_id,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us`. p50/p99 등은 post-process 계산. **`run_id` 로 먼저 그룹핑한다** — 세션 디렉토리가 분 해상도라 같은 분의 재기동이 같은 파일에 append 되고, 파일 전체로 `n / span` 을 내면 어느 런에도 없던 레이트가 나온다 (#376; `plot_rtc_log` 는 마지막 런을 자동 선택하고 무엇을 버렸는지 출력한다. `--run-id` 로 다른 런 선택). **Sim 모드 (`use_sim_time_sync=true`) 에서는 `jitter_us` 컬럼이 항상 0.0** — CV wakeup 이라 `\|actual_period − budget\|`이 sim cadence 잡음일 뿐 RT 지표가 아니기 때문 (`PeriodicRtThread::JitterMeaningful()` override). 다른 6개 컬럼은 robot/sim 동일 의미 |
| `<session>/timing/mpc_timing_log.csv` | per-controller LifecycleNode 1 Hz aux drains `MPCThread::TimingProducer()` | **Per-MPC-tick raw 샘플** — CM과 동일한 8-col 스키마 (`run_id` + RtTickTimingPayload). 한 row = 한 main-loop iteration. p50/p99/max는 post-process로 계산 (예: `awk` / pandas). aggregate INFO 라인은 controller 로그에 10 s마다 출력 (handler self-report `solve_duration_ns` 256-sample 윈도우). 두 CSV 모두 같은 generic infra + 동일 payload (`rtc_base/timing/rt_tick_timing_sample.hpp`) — 새 thread 추가 시 payload 재사용 |
| `<session>/timing/rt_callback_timing_log.csv` | rt_callback thread (slot 2, FIFO 70) — 각 device state 콜백의 `DeviceBackend::StateLaneTimingScope`, drained by 같은 `DrainLog()` | **Per-state-callback raw 샘플** (tick 당이 아니라 **콜백 당** 1행: arm joint + hand joint/motor/sensor). 같은 8-col 스키마지만 의미가 lane-specific — `t_state_us`=decode, `t_publish_us`=mailbox hand-off (notify 안 하는 hand motor/sensor 는 0), `t_total_us`=slot 2 duty 분자, `t_compute_us`/`jitter_us`=0. **dispatch 간격은 연속 행 `t_wall_ns` 차분**으로 본다. 이 lane 이 존재하는 이유는 slot 2 가 aux 통합(#349)의 대상인데 계측이 없었기 때문 — "93% 유휴"는 slot 1 수치다 |
| `/rtc_cm/{group}/joint_states` | CM (per-group, RELIABLE) | Device 그룹별 건강성; `rtc_digital_twin`이 merge |
| `/sim/status` | `rtc_mujoco_sim` 1 Hz | Sim 건강성 — 중단 시 sim sync timeout E-STOP |
| `<contact_wrench.topic_prefix>/<target>/contact_wrench` | `rtc_mujoco_sim` per-target (`mjSENS_CONTACT` netforce + world→link transform, env-on-link sign convention) | Fingertip 접촉 force/torque 확인. `RViz2 → WrenchStamped` display 또는 `ros2 topic echo`. 비접촉 시 0 발행. 활성화 조건: 그룹 YAML `contact_wrench.enabled: true` + MJCF 에 `<sensor><contact>` (data=`found force torque dist pos normal tangent` num=1 reduce=netforce) |
| `/p1a/joint_states`, `/p1a/motor_states`, `/p1a/sensor_states` | `udp_hand_driver` (ur5e_p1a; generic driver default `/hand/`) | Hand UDP 건강성 |
| `/shape_estimation/snapshot` (action feedback) | `shape_estimation` | ToF 기반 추정 진행 상황 |

## Debugging

| Symptom | Fix |
|---------|-----|
| `ApplyThreadConfig()` warns | `sudo usermod -aG realtime $USER` + re-login |
| E-STOP on startup | Set `init_timeout_sec: 0.0` for sim |
| High jitter (>200us) | Check `taskset` pinning, verify `isolcpus`, `check_rt_setup.sh --summary` |
| Hand timeout E-STOP | Check UDP link, `recv_timeout_ms: 0.4` |
| Controller not found | Use config_key (e.g. "demo_task_controller") or Name() |
| `ament_cmake_test` missing on `colcon test` | `.venv` overlay가 system site-packages를 가림. 재활성화 + ROS 2 환경 재로드 |

```bash
# exec name = ROS node name = "integrated_rt_controller" (only exec from integrated_bringup;
# rtc_controller_manager is library-only). Use the same name for pgrep and
# lifecycle calls:
#   ros2 lifecycle list /integrated_rt_controller
PID=$(pgrep -f integrated_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
# RT loop 건강성: robot + ur_driver_native 면 아래 토픽, sim 이면 device group 별 command_topic
ros2 topic hz /forward_position_controller/commands
ros2 topic echo /system/estop_status
./repo_scripts/scripts/check_rt_setup.sh --summary
```

## RT Permissions

```bash
sudo groupadd realtime && sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# Re-login required. Optional: isolcpus, nohz_full, or cpu_shield.sh
```

## CPU Shield (cset) 검증 — issue #151

`cpu_shield.sh` 는 cpuset 을 *만들기만* 하고, 런치가 CM 을 그 안으로 `adopt` 한다.
격리가 실제로 서는지는 **실기(SMT/hybrid, 예: NUC13 4P+8E)** 에서만 검증된다 —
sim 단일 실행으로 대체 불가 ([design-principles.md](design-principles.md) sim-noise 원칙).

```bash
# 1) shield cpuset 이 CM 전체 span 을 덮는가 (NUC13 12c → user=2-7, #380 이후)
sudo ./repo_scripts/scripts/cpu_shield.sh on --robot
cset shield -s          # "user" == get_cm_shield_cpus 출력과 일치해야
# 2) 런치(shield-on) 후 CM 이 user cpuset 에 들어갔는가
CM=$(pgrep -nf integrated_rt_controller)
grep Cpus_allowed_list /proc/$CM/status      # == 2-7 (부분집합 비교는 shield 축소를 놓친다)
# 3) activate 후 RT/nrt 스레드가 제대로 pin·FIFO 되었는가
ps -eLo comm,psr,cls,rtprio -p $CM | grep -E "rt_control|rt_callback|nrt_"
#   기대: rt_control psr=2/FF/90, rt_callback psr=4, nrt psr=12·13
# 4) EINVAL 회귀 없음 (shield 가 pin 을 깨뜨리지 않음)
grep -rE "rc=22|setaffinity failed|Thread config failed" ~/.ros/log/<run>/  # 결과 없어야
# 5) 게이트가 활성 shield 를 재활성 안 함 (cset-aware)
#   두 번째 런치 로그에 "CPU shield already active (cset user cpuset present)"
```

## Tracing

CSV timing logs (`cm_timing_log.csv` / `mpc_timing_log.csv` / `hand_udp_timing_log.csv`) 은 **per-tick 총 시간** 만 기록한다. 어느 thread 가 어느 core 에서 언제 run 했는지 / 어떤 callback 이 시간을 쓰는지 알아내려면 LTTng 트레이스를 캡처해 분석한다.

세부 명령·event 선택·permission 분기·뷰어 사용법은 [../docs/tracing.md](../docs/tracing.md) 참조 (ros2_tracing / babeltrace2 / Perfetto operational guide).

핵심 진입점만:

```bash
./install.sh --tracing                                              # 1회 setup
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true   # 캡처
./repo_scripts/scripts/timeline.sh                                  # Perfetto JSON 변환
```
