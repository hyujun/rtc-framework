# Testing & Debugging

## Sensor Matrix (변경 유형별 필수 검증)

[CLAUDE.md](../CLAUDE.md) §5의 상세판. 변경 위치에 따라 필수 sensor + 추가 sensor를 실행한다.

| 변경 위치 | 필수 Sensor | 추가 Sensor |
|----------|------------|------------|
| `rtc_base/` | `colcon test --packages-select rtc_base` | 전체 downstream ([invariants.md](invariants.md) PROC-3) |
| `rtc_math/` | `colcon test --packages-select rtc_math` (Pinocchio 발견 시 `test_se3_module` log6/Jlog6 교차검증 + 유한차분) | `se3_error_compare` S1–S5 실험 + `plot_se3_compare.py` (선택) |
| `rtc_msgs/` | 위 + `./build.sh full` (msg gen 전파) | downstream pub/sub 테스트 |
| `rtc_controller_interface/` | `test_core_controllers` + controller registry tests | downstream controller 빌드 |
| `rtc_controllers/` RT path | `test_core_controllers` + grasp 관련 gtest | RT scheduling 확인 (`ps -eLo cls,rtprio`) |
| `rtc_controllers/` gains/config | 위 + 해당 controller YAML 로드 smoke | `ros2 topic echo /rtc_cm/active_controller_name` |
| `rtc_controller_manager/` | RT loop timing (`/system/estop_status`) | (MPC CSV는 `<session>/controllers/<config_key>/...` 경로로 컨트롤러가 자체 기록) |
| `rtc_tsid/` | QP/task/constraint gtest | TSID performance tests |
| `rtc_mpc/` | gtest (types, TripleBuffer, Riccati, SolutionManager) | `mpc_timing_log.csv` p50/p99/max 회귀 |
| `rtc_mujoco_sim/` | gtest (parse, lifecycle, solver, I/O, contact_wrench) | `ros2 launch integrated_bringup sim_ur5e_p1a.launch.py` smoke. Contact wrench: `ros2 topic hz /<prefix>/<target>/contact_wrench` 후 fingertip 으로 객체 접촉 → magnitude 가시화 |
| `rtc_urdf_bridge/` | gtest (URDF/model parsing, xacro, chain extractor) | 실제 URDF 파싱 smoke |
| `rtc_inference/` | ONNX engine unit test | 실제 모델 로드 smoke |
| `rtc_communication/` | UDP loopback + CAN/CANFD loopback (vcan0 없으면 skip) + RS485 serial loopback (PTY, 상시 실행) + Transceiver lifecycle/decode/callback | vcan0 셋업 후 CAN 테스트 실행 (`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`), RS485는 PTY라 셋업 불필요, 실제 HW UDP/CAN/RS485(USB-RS485+Dynamixel) 테스트 (선택) |
| `rtc_digital_twin/` | pytest + RViz2 smoke | `/rtc_cm/{group}/joint_states` hz |
| `rtc_tools/` (pytest) | pytest | GUI/plot 수동 smoke |
| `repo_scripts/` | `test_rt_common` + shell unit test | `check_rt_setup.sh --summary` |
| `shape_estimation*/` | ToF + exploration gtest | `/shape_estimation/snapshot` topic echo |
| `integrated_bringup/` demo FSM | demo_wbc FSM/integration/output + demo_joint grasp·URDF paths + demo_task CLIK/contact_stop (URDF-backed iiwa7_leap fixture 공유, 노드 비생성) + grasp_phase_manager + virtual_tcp | BT coordinator 통합 |
| `udp_hand_driver/` | 단위 gtest (hand_packets, codec, FT, failure detector) + UDP loopback | `ros2 topic hz /p1a/joint_states` (ur5e_p1a; 드라이버 standalone 기본은 `/hand/`) |
| `ur5e_bt_coordinator/` | BT gtest — Tier-2 는 inject(DDS-free, `inject_fixture.hpp`)/e2e(real-DDS, `test_helpers.hpp`) 분리, suite 목록은 CMakeLists `TIER2_INJECT`/`TIER2_E2E` (#154) | 실제 grasp 시나리오 smoke |
| Launch / YAML | `ros2 launch ... --print` + 짧은 smoke | config 로드 검증 |
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

가드(검증·거부 경로)와 그 테스트를 함께 추가하면 **테스트 통과는 가드가 동작한다는 증거가 아니다** — 그 가드를 지워도 통과할 수 있다. 추가한 가드마다 **하나씩 원복 → 대응 테스트가 실제로 실패하는지 확인 → 복구**. §5.5 의 "에이전트 자기 평가는 신뢰 불가" 가 자기가 쓴 테스트에도 그대로 적용된다.

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

## RT-1 zero-allocation 게이트 — 두 종류이고 서로의 맹점을 덮는다

RT tick 이 heap 을 안 만진다는 주장(RT-1)을 재는 sensor 는 **두 개**이고, 어느 것을 쓸지는 취향이 아니라 두 질문으로 결정된다: **(a) 측정 대상이 Eigen 을 쓰는가, (b) 측정 대상 코드가 테스트와 같은 TU 에 인스턴스화되는가.**

| 게이트 | 보는 것 | 못 보는 것 |
|---|---|---|
| `rtc::testing::ScopedAllocGate` (`rtc_controllers/test/include/rtc_controllers/testing/alloc_gate.hpp`) — 전역 `operator new` 교체 | `operator new` 를 타는 모든 할당 (`std::vector`, header-inline helper, 다른 TU 포함) | **Eigen 할당 전부** — `internal::aligned_malloc` 이 `std::malloc` 을 직접 부르고 `operator new` 를 타지 않는다 |
| `rtc::testing::ScopedNoMalloc` (`rtc_base/test/include/rtc_base/testing/no_malloc_scope.hpp`) — `eigen_assert` + `EIGEN_RUNTIME_NO_MALLOC` | Eigen 동적 할당 (Release 에서도 유효) | non-Eigen heap; **그리고 다른 TU 의 Eigen** — 이 매크로는 정의된 TU 안의 Eigen inline 만 계측한다 |

따라서:

- **순수 Eigen 코어 (header-inline 법칙)** → **둘 다** 무장한다. 하나만 쓰면 가장 유력한 RT-1 회귀 (인자·임시가 `Eigen::VectorXd` 같은 runtime-sized 로 퇴화) 가 green 으로 통과한다. `test_task_accel_core` / `test_task_vel_core` 가 이 형태.
- **Eigen-free 코어** → `ScopedAllocGate` 만. Eigen 트립와이어는 여기서 진짜로 vacuous 하다 (`test_joint_pd_core`).
- **컨트롤러 `Compute()` 처럼 라이브러리 TU 에 컴파일된 코드** → `ScopedAllocGate` 만. Eigen 트립와이어는 관측 대상이 다른 TU 라 vacuous 하다 (`test_task_impedance_controller` 등).

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

**두 가지 계측 함정 — 비교 가능한 수치를 낼 때 필수:**

- **`colcon test-result` 는 디렉토리에 남아 있는 XML 을 전부 합산한다.** CMakeLists 에서 타깃을 빼거나 브랜치를 되돌린 뒤 재측정하면 **사라진 타깃의 옛 결과가 그대로 더해진다** — 숫자가 그럴듯해서 자체 검산 없이는 안 걸린다 (#236 슬라이스 3 에서 baseline 을 298 대신 300 으로 오측하고 존재하지 않는 drift 원인까지 보고했다).
- **gtest case 수와 ctest entry 수를 함께 센다.** `rtc_controllers 333` = gtest 315 + ctest 18. 옛 수치와 비교할 땐 **단위가 같은지** 먼저 확인한다.
- **lint 도 소스 파일당 1 entry 를 낸다.** `cppcheck.xunit.xml` 의 `tests="N"` 은 그 패키지의 소스 파일 수이고 전부 **skipped** 로 집계된다. 따라서 테스트 `.cpp` 를 한 개 추가하면 총계는 **1(케이스가 1개일 때) + 1(ctest 타깃) + 1(cppcheck 파일)** 로 오르고 skipped 도 +1 이 된다 — 델타를 gtest 케이스 수만으로 예측하면 매번 어긋난다. 증감을 "삭제·신설 목록과 1:1" 로 설명해야 하는 슬라이스에서는 이 세 항을 분리해 적는다.

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

**LifecycleNode 노드(예: `rtc_controller_manager`) 유닛 커버리지 패턴**: `on_configure` 파이프라인 (params 로딩·device backend·publisher 생성) 과 private RT 헬퍼(`CheckTimeouts`/`CreateDeviceBackends`)는 friend accessor (`rtc::ControllerLifecycleTestAccess`, 헤더의 `friend` 선언으로 이름 고정) 로 private 멤버를 주입·호출해 **실 robot/RT 권한 없이** 구동한다 — `CreateDeviceBackends` 등은 `group_slot_map_`/`device_name_configs_` 주입 + registry fake backend 만으로 동작하고, `on_activate` 의 RT 루프는 `ApplyThreadConfig` 반환값을 버리므로 (SCHED_OTHER fallback) 테스트 샌드박스에서도 안전하다.

> **Full-pipeline 테스트는 전용 `ROS_DOMAIN_ID` 로 격리** (`ament_add_gtest(... ENV ROS_DOMAIN_ID=<n>)`): CI 의 `colcon test` 는 패키지를 병렬 실행하므로, 실 bring-up (다수 LifecycleNode/pub 생성·소멸 반복) 을 돌리는 테스트가 기본 도메인을 공유하면 DDS discovery churn 으로 **동시 실행 중인 타 패키지의 pub/sub 매칭 테스트가 깨진다** (PR #187 에서 `ur5e_bt_coordinator` `test_rewire_gate` 가 실측 피해 — 실패 시각이 가해 테스트 실행 창과 정확히 겹침). 도메인 번호는 기존 사용처 (`grep -rn 'ROS_DOMAIN_ID=' --include=CMakeLists.txt`) 와 충돌하지 않게 배정.

**`.venv` 격리 원칙 (Hard rule)**: `.venv`는 runtime PC가 본 workspace 외에 다른 control project들과 공존하는 환경에서 workspace dependency를 격리하기 위한 **의도된 설계**다. `colcon test` / `colcon build` / `ros2 run` / `ros2 launch`가 venv 활성 상태에서 실패하면 **반드시 근본 원인을 해결**한다 (sys.path / shebang / wrapper / dep resolution 디버그). gtest 바이너리 직접 실행, venv deactivate 후 colcon 호출, `PYTHONPATH` 강제 우회 등은 **금지** — 격리를 무력화해 runtime PC에서 다른 project의 site-packages가 끼어들면 silent breakage. 신호 (`Testing/Temporary/LastTest.log` Start/End 동일 초)가 재발하면 `env -i` 깨끗한 셸에서 `setup_env.sh` source 후 `sys.path` 순서 점검부터.

## Live Debug Topics

런타임 문제 탐지용 토픽. `ros2 topic echo` / `ros2 topic hz` / `ros2 bag record` 대상.

| Topic | 발행 주체 | 언제 보나 |
|-------|----------|----------|
| `/system/estop_status` | `rtc_controller_manager` | E-STOP 원인 파악 (timeout name / trigger thread) |
| `/rtc_cm/active_controller_name` | 동일 (TRANSIENT_LOCAL) | Controller switch 확인. BT / GUI / digital_twin은 이 토픽으로 리와이어 |
| `/<config_key>/<config_key>/get_parameters` (srv) | active 데모 컨트롤러의 LifecycleNode | Runtime gain 값 조회 (`ros2 param get`) |
| `/forward_position_controller/commands` | **robot 모드 + `ur_driver_native` backend 전용** | RT loop 건강성 — `ros2 topic hz` 로 설정된 `control_rate` (default 500 Hz) 매칭 확인. **sim 에는 이 토픽이 없다** — sim 의 커맨드 lane 은 device group 당 하나이고 (`devices.<group>.backend.command_topic`, 예: `/ur5e/joint_command` + `/p1a/joint_command`) robot 당 하나가 아니므로, sim 에서 이 토픽의 침묵을 "RT loop 정지" 로 읽으면 오진이다 |
| `<session>/timing/cm_timing_log.csv` | CM RT loop @ `control_rate` (`rtc::ThreadTimingProducer<RtTickTimingPayload>`) drained by `DrainLog()` log thread | RT loop per-tick timing — 7 cols `t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us`. p50/p99 등은 post-process 계산. **Sim 모드 (`use_sim_time_sync=true`) 에서는 `jitter_us` 컬럼이 항상 0.0** — CV wakeup 이라 `\|actual_period − budget\|`이 sim cadence 잡음일 뿐 RT 지표가 아니기 때문 (`PeriodicRtThread::JitterMeaningful()` override). 다른 6개 컬럼은 robot/sim 동일 의미 |
| `<session>/timing/mpc_timing_log.csv` | per-controller LifecycleNode 1 Hz aux drains `MPCThread::TimingProducer()` | **Per-MPC-tick raw 샘플** — CM과 동일한 7-col 스키마 (RtTickTimingPayload). 한 row = 한 main-loop iteration. p50/p99/max는 post-process로 계산 (예: `awk` / pandas). aggregate INFO 라인은 controller 로그에 10 s마다 출력 (handler self-report `solve_duration_ns` 256-sample 윈도우). 두 CSV 모두 같은 generic infra + 동일 payload (`rtc_base/timing/rt_tick_timing_sample.hpp`) — 새 thread 추가 시 payload 재사용 |
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
# 1) shield cpuset 이 CM 전체 span 을 덮는가 (NUC13 12c → user=2-9,12-13)
sudo ./repo_scripts/scripts/cpu_shield.sh on --robot
cset shield -s          # "user" == get_cm_shield_cpus 출력과 일치해야
# 2) 런치(shield-on) 후 CM 이 user cpuset 에 들어갔는가
CM=$(pgrep -nf integrated_rt_controller)
grep Cpus_allowed_list /proc/$CM/status      # ⊆ {2-9,12-13}
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
