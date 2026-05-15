# Modification Guide

## Workflow Loop

모든 수정 작업은 이 순서 ([CLAUDE.md](../CLAUDE.md#L67) §4 요약판의 상세).

```
0. Type     → "수정"인가 "추가(새 기능/컨트롤러/메시지/디바이스/스레드)"인가?
              추가라면 [design-principles.md](design-principles.md) 5원칙 + 본 문서
              "Adding a New ..." 절을 먼저 읽는다.
              · rtc_*에 추가 → P1·P2 (zero source edit, robot 상수 금지) +
                ARCH-3 (interface-first; 같은 종류 두 번째 구현이면 base부터)
              · ur5e_* / shape_estimation에 추가 → 재사용 가능한 부분이
                rtc_*에 존재하는지 / 일반화해 끌어올릴 수 있는지 먼저 검토
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

**※ 4·5·6은 [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Stop hook이 turn 종료 시 자동 실행/차단한다 — 사전 수동 실행은 빠른 피드백용. Hook 한계: 변경 패키지만 빌드, 60s timeout per package, README/CMake만 검사 (package.xml/YAML은 미검). Pure-format commit (모든 변경 .cpp/.hpp/.py 가 `clang-format`/`ruff format` round-trip 결과와 동일) 은 ARCH grep + README/CMake co-update 단계만 자동 skip 되고 build/test 는 그대로 돈다.**

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
2. Source in `rtc_controllers/src/controllers/{direct|indirect}/` -- `LoadConfig()` for YAML. Runtime gains는 컨트롤러 자체 LifecycleNode (`/<config_key>`) 의 ROS 2 parameter로 노출 — `on_configure`에서 `DeclareGainParameters()` + `add_on_set_parameters_callback(OnGainParametersSet)`. Read-only 캡(`*_max_traj_velocity`)은 `ParameterDescriptor::read_only=true`. Force-PI 같은 one-shot 이벤트는 [rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv) 같은 srv 채널을 별도로 마련하고 `~/grasp_command`로 advertise (active controller만 server를 띄움).
3. Gains struct must be trivially copyable (plain arrays/bools/doubles/floats/ints; no `std::string`/`std::vector`/virtuals). Store as `rtc::SeqLock<Gains> gains_lock_` — RT path snapshots once with `const auto gains = gains_lock_.Load();` at method entry; aux-thread writers (parameter callback / srv handler) use Load/mutate/Store. `set_gains`/`get_gains` accessors delegate to the SeqLock and are used by tests.
4. YAML in `rtc_controllers/examples/controllers/` (reference only) or `<robot>_bringup/config/<robot>/controllers/` (production) -- must include `topics:` section. **Subscribe roles** (`role: target`) and **publish roles** (`role: robot_target` / `robot_transforms` / `digital_twin_state`) tagged with `ownership: controller` use **relative paths** (`<group>/joint_goal`, not `/<group>/joint_goal`). Manager-owned entries keep absolute paths and may omit `ownership` (defaults to `manager`). **Controller-owned non-RT topics that are NOT in PublishRole** (`grasp_state` / `wbc_state` / `tof_snapshot` since `104796f`): controller owns its own `SeqLock<{Grasp,Wbc,ToF}StateData>` writer + calls the `Setup{Grasp,Wbc,ToF}*Publisher` helper from `integrated_bringup/support/owned_topics.hpp` — no `topics:` entry, no PublishRole; topic name is hardcoded in the helper (`<config_key>/<group>/{grasp_state,wbc_state,snapshot}`). See `integrated_bringup/support/owned_topics.hpp` + `src/support/owned_topics.cpp` for the create/activate/publish helpers the 3 demos share.
5. If the controller owns any topics: override `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `PublishNonRtSnapshot` and delegate to the `owned_topics` helper (or inline the equivalent). Always call the base `RTControllerInterface::on_configure` / `on_cleanup` first. **CM bring-up 은 3-pass** (`PreConfigure` → `SetDeviceNameConfigs` → `on_configure`) 라서 `on_configure` 안에서 `RegisterLog<>(...)` 의 람다가 `joint_state_names`/`motor_state_names` 등을 capture할 때 `OnDeviceConfigsSet()` 가 이미 실행됐음이 보장됩니다 — 별도 시점 조정 불필요. `PreConfigure` 는 base 전용이므로 override 금지.
6. Register via `RTC_REGISTER_CONTROLLER()` macro. Robot-specific controllers go in `integrated_bringup/` with registration in `integrated_bringup/src/controllers/controller_registration.cpp`

## Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`, add to `CMakeLists.txt` `rosidl_generate_interfaces()`
2. **Decide ownership before adding a PublishRole.** PublishRole is reserved for CM-/manager-published streams (`kRobotTarget`, `kDigitalTwinState`, `kRobotTransforms`). For controller-owned non-RT topics (Grasp/Wbc/ToF style), do NOT add a PublishRole — instead: declare `SeqLock<MyData>` member on the controller, write a `SetupMyDataPublisher()` helper in `integrated_bringup/support/owned_topics.hpp`, and have the controller call it from `on_configure`/`on_activate` (RT loop pushes via the SeqLock writer; aux thread reads + publishes). See the Grasp/Wbc/ToF wiring in `104796f` as the canonical pattern.
3. If you genuinely need a new PublishRole: add it to `rtc_base/types/types.hpp` `PublishRole` enum + `PublishRoleToString()` + YAML mapping in `rtc_controller_interface/src/rt_controller_interface.cpp` parser. Add a `GroupCommandSlot` field if it's per-device.

## Adding a New Device Group

1. Add device entry in `<robot>_bringup/config/<robot>/{sim,robot}.yaml` under `devices:`. **`devices.<group>.backend:` is the SSoT** (since `b9a2587`, 2026-05-15) — declare `backend.type:` (e.g. `ros2_topic`, `udp_hand`, `mujoco_sim`) + backend-specific config (topics, transport endpoints). The CM no longer reads device-wire roles from controller YAML; backend impls own the read/write lanes.
2. Add timeout entry in `device_timeout_names`/`values`.
3. If a new backend type is needed: implement the `DeviceBackend` interface (`rtc_controller_manager/include/rtc_controller_manager/device_backend.hpp`) + register via `RTC_REGISTER_DEVICE_BACKEND(my_backend)` macro. Override `ReadState()` / `WriteCommand()` (RT-safe) and the `OnConfigure*` / `OnActivate*` lifecycle hooks as needed (base provides default no-op impls).
4. If the controller needs to consume the new group: add subscribe topic routing in the controller's YAML `topics:` section (`role: target` typical), and handle the new device index in controller `Compute()` / `SetDeviceTarget()`.
5. If kinematics needed: add `sub_models` or `tree_models` entry under `urdf:`.

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
  - Controller configs: `rtc_controllers/examples/controllers/{direct|indirect}/*.yaml` (reference) + `integrated_bringup/config/<variant>/controllers/*.yaml` (production)
  - Robot/sim configs: `integrated_bringup/config/ur5e_hand/robot.yaml`, `ur5e_hand/sim.yaml`
  - MuJoCo configs: `rtc_mujoco_sim/config/*.yaml`
  - Hand driver: `udp_hand_driver/config/udp_hand_node.yaml`
  - Digital twin: robot-agnostic default in `rtc_digital_twin/config/digital_twin.yaml`; robot-specific overlay (URDF, source topics, sensor topic, fingertip names) in `<robot>_bringup/config/digital_twin_<robot>.yaml` (UR5e: `integrated_bringup/config/ur5e_hand/digital_twin.yaml`)
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

If the change touches `rtc_base` or `rtc_msgs`, build and test all downstream packages (PROC-3). The Stop hook automates this — see [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Phase 2 PROC-3 fallback.

**Fork-Join option** for PROC-3 (downstream 패키지가 서로 독립하면): `Agent` tool로 패키지별 subagent를 worktree isolation에 띄워 병렬 빌드/테스트 가능. 직렬 `./build.sh full` 보다 빠르지만 disk + RAM 비용 큼. 권장 임계: downstream 4 패키지 이상 + 각 빌드 5분 이상일 때만. 결과는 main 컨텍스트로 요약 반환 — context window 절약.

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
