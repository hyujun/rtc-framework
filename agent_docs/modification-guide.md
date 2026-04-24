# Modification Guide

## Workflow Loop

모든 수정 작업은 이 순서 ([CLAUDE.md](../CLAUDE.md#L67) §4 요약판의 상세).

```
1. Locate   → grep / Glob (known symbol) OR Explore agent (broad search)
              파일의 RT / aux / robot-specific 역할 판단
2. Read     → package.xml + CMakeLists.txt + target file + 인접 테스트
              invariants.md 중 영향받는 항목 확인
3. Edit     → minimal, single-concern. RT path 여부 재확인.
              auto/lerp/RT-forbidden 자체 grep
4. Build    → ./build.sh -p <pkg> (단일) 또는 ./build.sh full (rtc_base/rtc_msgs 변경 시)
5. Test     → CLAUDE.md §5 Sensor matrix 참조. 버그 수정 시 회귀 테스트 추가
6. Verify   → 본 문서 Completion Checklist 8항목 통과
```

### Workflow Fail-Safe

각 단계 실패 시 대응. "Try harder"는 실패 응답이 아니다 — 누락된 capability를 엔지니어링하거나 [CLAUDE.md](../CLAUDE.md#L115) §6 Escalate.

| 실패 단계 | 증상 | 대응 |
|----------|------|------|
| 1. Locate | 파일을 찾을 수 없음 | `Agent` subagent로 broad search. "찾았다고 추정" 금지 |
| 2. Read | 컨텍스트 불충분 (호출자 / 테스트 미확인) | 인접 파일 + 테스트 추가 읽기. 추측하지 말 것 |
| 3. Edit | Invariant 위반 유혹 | [invariants.md](invariants.md) 확인 후 [CLAUDE.md](../CLAUDE.md#L115) §6 Escalate. 우회로 찾지 말 것 |
| 4. Build | 빌드 실패 | 에러 메시지를 **먼저** 기록. 원인 파악 전 재시도 금지 |
| 5. Test | 테스트 실패 | assertion 수정 금지 ([anti-patterns.md](anti-patterns.md) AP-PROC-4). 새 코드를 고칠 것 |
| 6. Verify | Checklist 항목 실패 | 해당 항목까지 rollback, 재실행. 부분 완료 주장 금지 ([anti-patterns.md](anti-patterns.md) AP-PROC-1) |

## Adding a New Controller

1. Header in `rtc_controllers/include/rtc_controllers/{direct|indirect}/` -- inherit `RTControllerInterface`, implement `Compute()`, `SetDeviceTarget()`, `InitializeHoldPosition()`, `Name()` (all `noexcept`)
2. Source in `rtc_controllers/src/controllers/{direct|indirect}/` -- `LoadConfig()` for YAML, `UpdateGainsFromMsg()` for runtime gains
3. Gains struct must be trivially copyable (plain arrays/bools/doubles/floats/ints; no `std::string`/`std::vector`/virtuals). Store as `rtc::SeqLock<Gains> gains_lock_` — RT path snapshots once with `const auto gains = gains_lock_.Load();` at method entry; aux-thread writers use Load/mutate/Store. `set_gains`/`get_gains` accessors delegate to the SeqLock.
4. YAML in `rtc_controllers/config/controllers/` -- must include `topics:` section. Tag non-RT external topics (target / gui_position / grasp_state / tof_snapshot / ...) with `ownership: controller` and use **relative paths** (`ur5e/joint_goal`, not `/ur5e/joint_goal`). Manager-owned entries keep absolute paths and may omit `ownership` (defaults to `manager`). See `ur5e_bringup/controllers/owned_topics.{hpp,cpp}` for the standard create/activate/publish helper the 3 demos share.
5. If the controller owns any topics: override `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `PublishNonRtSnapshot` and delegate to the `owned_topics` helper (or inline the equivalent). Always call the base `RTControllerInterface::on_configure` / `on_cleanup` first.
6. Register via `RTC_REGISTER_CONTROLLER()` macro. Robot-specific controllers go in `ur5e_bringup/` with registration in `ur5e_bringup/src/controllers/controller_registration.cpp`

## Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`, add to `CMakeLists.txt` `rosidl_generate_interfaces()`
2. If used in publish offload: add `PublishRole` enum in `rtc_base/types/types.hpp` + YAML mapping in `rtc_controller_interface/src/rt_controller_interface.cpp`

## Adding a New Device Group

1. Add device config in YAML under `devices:` + timeout entry in `device_timeout_names`/`values`
2. Add topic routing in each controller's YAML `topics:` section
3. If kinematics needed: add `sub_models` or `tree_models` entry under `urdf:`
4. Handle new device index in controller `Compute()` / `SetDeviceTarget()`

## Adding a New Thread

1. Define `ThreadConfig` for all core tiers in `rtc_base/threading/thread_config.hpp`
2. Add to `SystemThreadConfigs`, update `ValidateSystemThreadConfigs()` + `SelectThreadConfigs()`
3. Call `ApplyThreadConfig()` at thread entry; use SCHED_FIFO for RT threads

## Updating an Existing Package

When modifying code in any package, complete ALL of the following steps. Code changes without corresponding documentation and metadata updates are considered incomplete.

### 1. Unit Tests -- Update & Run

- Identify affected tests: check `<package>/test/` directory
- Update existing tests if public API, signatures, behavior, or types changed
- Add new tests for new functionality (happy path + edge cases + error handling)
- C++ tests: add `ament_add_gtest()` in `CMakeLists.txt` for new test files
- Python tests: follow `test_*.py` naming convention
- Run and verify:
  ```bash
  ./build.sh -p <package_name>
  colcon test --packages-select <package_name> --event-handlers console_direct+
  colcon test-result --verbose
  ```
- All tests must pass before proceeding

### 2. CMakeLists.txt -- Verify & Update

- Source files: update `add_library()` / `add_executable()` if `.cpp` files added/renamed/removed
- Header install: verify `install(DIRECTORY include/ ...)` for new public headers
- Dependencies: add `find_package()` / `ament_target_dependencies()` as needed
- Test targets: add `ament_add_gtest()` / `ament_add_pytest_test()` for new tests
- Message generation: update `rosidl_generate_interfaces()` for new `.msg`/`.srv`/`.action`
- Build cleanly: `./build.sh -p <package_name>`

### 3. package.xml -- Verify & Update

- Dependencies: add `<build_depend>`, `<exec_depend>`, `<depend>`, `<test_depend>` tags
- Version: bump per semver for significant changes
- Consistency: `package.xml` deps must match `CMakeLists.txt` `find_package()` calls

### 4. Config Files (YAML) -- Verify & Update

- Update all relevant YAML files if parameters added/removed/renamed:
  - Controller configs: `rtc_controllers/config/controllers/{direct|indirect}/*.yaml`
  - Robot/sim configs: `ur5e_bringup/config/ur5e_robot.yaml`, `ur5e_sim.yaml`
  - MuJoCo configs: `rtc_mujoco_sim/config/*.yaml`
  - Hand driver: `ur5e_hand_driver/config/hand_udp_node.yaml`
  - Digital twin: `rtc_digital_twin/config/digital_twin.yaml`
- Update default values and inline YAML comments (valid ranges, units)
- Update `topics:` section if topic names or device groups changed

### 5. Documentation -- Verify & Update

- **Package README.md** (mandatory): API docs, usage examples, parameter descriptions, dependencies
- **Cross-package docs** (when change crosses package boundary): root `README.md`, `docs/*.md`
- **Inline docs**: Doxygen comments for new/changed public headers, YAML comments for parameters

### 6. Final Verification

```bash
./build.sh -p <package_name>
colcon test --packages-select <package_name> [<dependent_packages>...] --event-handlers console_direct+
colcon test-result --verbose
```

If the change touches `rtc_base` or `rtc_msgs`, build and test all downstream packages.

## Completion Checklist

After every code modification, self-check:

- [ ] Build succeeded with 0 warnings
- [ ] All existing tests pass
- [ ] New tests added for new public API
- [ ] No forbidden patterns in RT paths (see `agent_docs/conventions.md`)
- [ ] `CMakeLists.txt` updated (sources, deps, install, tests)
- [ ] `package.xml` updated (deps synced, version bump if needed)
- [ ] YAML config files updated (parameters, defaults, comments)
- [ ] Package `README.md` updated (API, parameters, examples)
- [ ] Doxygen comments added/updated for public headers
