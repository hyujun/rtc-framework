# Modification Guide

## Workflow Loop

모든 수정 작업은 이 순서 ([CLAUDE.md](../CLAUDE.md) §4 요약판의 상세).

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

각 단계 실패 시 대응. "Try harder"는 실패 응답이 아니다 — 누락된 capability를 엔지니어링하거나 [CLAUDE.md](../CLAUDE.md) §6 Escalate.

| 실패 단계 | 증상 | 대응 |
|----------|------|------|
| 1. Locate | 파일을 찾을 수 없음 | `Agent` subagent로 broad search. "찾았다고 추정" 금지 |
| 2. Read | 컨텍스트 불충분 (호출자 / 테스트 미확인) | 인접 파일 + 테스트 추가 읽기. 추측하지 말 것 |
| 3. Edit | Invariant 위반 유혹 | [invariants.md](invariants.md) 확인 후 [CLAUDE.md](../CLAUDE.md) §6 Escalate. 우회로 찾지 말 것 |
| 4. Build | 빌드 실패 | 에러 메시지를 **먼저** 기록. 원인 파악 전 재시도 금지 |
| 5. Test | 테스트 실패 | assertion 수정 금지 ([anti-patterns.md](anti-patterns.md) AP-PROC-4). 새 코드를 고칠 것 |
| 6. Verify | Checklist 항목 실패 | 해당 항목까지 rollback, 재실행. 부분 완료 주장 금지 ([anti-patterns.md](anti-patterns.md) AP-PROC-1) |

## Adding a New Controller

1. Header in `rtc_controllers/include/rtc_controllers/{direct|indirect}/` -- inherit `RTControllerInterface`, implement `Compute()`, `SetDeviceTarget()`, `InitializeHoldPosition()`, `Name()` (all `noexcept`)
2. Source in `rtc_controllers/src/controllers/{direct|indirect}/` -- `LoadConfig()` for YAML. Runtime gains는 컨트롤러 자체 LifecycleNode (`/<config_key>`) 의 ROS 2 parameter로 노출 — `on_configure`에서 `DeclareGainParameters()` + `add_on_set_parameters_callback(OnGainParametersSet)`. Read-only 캡(`*_max_traj_velocity`)은 `ParameterDescriptor::read_only=true`. Force-PI 같은 one-shot 이벤트는 [rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv) 같은 srv 채널을 별도로 마련하고 `~/grasp_command`로 advertise (active controller만 server를 띄움).
3. Gains struct must be trivially copyable (plain arrays/bools/doubles/floats/ints; no `std::string`/`std::vector`/virtuals). Store as `rtc::SeqLock<Gains> gains_lock_` — RT path snapshots once with `const auto gains = gains_lock_.Load();` at method entry; aux-thread writers (parameter callback / srv handler) use Load/mutate/Store. `set_gains`/`get_gains` accessors delegate to the SeqLock and are used by tests.
4. YAML in `rtc_controllers/examples/controllers/` (reference only) or `<robot>_bringup/config/<robot>/controllers/` (production) -- must include `topics:` section. **Subscribe roles** (`role: target`) 와 **publish roles** (`role: robot_target` / `robot_transforms` / `digital_twin_state`) 에 `ownership: controller` 태그하면 **relative path** 사용 (`<group>/joint_goal`, not `/<group>/joint_goal`). Manager-owned entries 는 absolute path, `ownership` 생략 가능 (default `manager`). **Controller-owned non-RT topics that are NOT in PublishRole** (`grasp_state` / `wbc_state` / `tof_snapshot`): controller 가 자체 `SeqLock<{Grasp,Wbc,ToF}StateData>` writer 소유 + `integrated_bringup/support/owned_topics.hpp` 의 `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼 호출 — `topics:` entry 없음, PublishRole 없음; topic name 은 helper 에 hardcoded.
5. If the controller owns any topics: override `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `PublishNonRtSnapshot` and delegate to the `owned_topics` helper (or inline the equivalent). Always call the base `RTControllerInterface::on_configure` / `on_cleanup` first. **CM bring-up 은 3-pass** (`PreConfigure` → `SetDeviceNameConfigs` → `on_configure`) 라서 `on_configure` 안에서 `RegisterLog<>(...)` 의 람다가 `joint_state_names`/`motor_state_names` 등을 capture할 때 `OnDeviceConfigsSet()` 가 이미 실행됐음이 보장됩니다 — 별도 시점 조정 불필요. `PreConfigure` 는 base 전용이므로 override 금지.
6. Register via `RTC_REGISTER_CONTROLLER()` macro. Robot-specific controllers go in `integrated_bringup/` with registration in `integrated_bringup/src/controllers/controller_registration.cpp`

## Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`, add to `CMakeLists.txt` `rosidl_generate_interfaces()`
2. **Decide ownership before adding a PublishRole.** PublishRole is reserved for CM-/manager-published streams (`kRobotTarget`, `kDigitalTwinState`, `kRobotTransforms`). Controller-owned non-RT topics (Grasp/Wbc/ToF 스타일) 는 PublishRole 추가 금지 — 대신: controller 에 `SeqLock<MyData>` 멤버 선언, `integrated_bringup/support/owned_topics.hpp` 에 `SetupMyDataPublisher()` 헬퍼 작성, controller 가 `on_configure`/`on_activate` 에서 호출 (RT loop pushes via the SeqLock writer; aux thread reads + publishes). 기존 Grasp/Wbc/ToF wiring 이 canonical pattern.
3. If you genuinely need a new PublishRole: add it to `rtc_base/types/types.hpp` `PublishRole` enum + `PublishRoleToString()` + YAML mapping in `rtc_controller_interface/src/rt_controller_interface.cpp` parser. Add a `GroupCommandSlot` field if it's per-device.

## Adding a New Device Group

1. Add device entry in `<robot>_bringup/config/<robot>/{sim,robot}.yaml` under `devices:`. **`devices.<group>.backend:` is the SSoT** — declare `backend.type:` (e.g. `ros2_topic`, `udp_hand`, `mujoco_sim`) + backend-specific config (topics, transport endpoints). CM 은 더 이상 controller YAML 에서 device-wire role 을 읽지 않으며 backend 구현체가 read/write lane 소유.
2. Add timeout entry in `device_timeout_names`/`values`.
3. If a new backend type is needed: implement the `DeviceBackend` interface (`rtc_controller_manager/include/rtc_controller_manager/device_backend.hpp`) + register via `RTC_REGISTER_DEVICE_BACKEND(my_backend)` macro. Override `ReadState()` / `WriteCommand()` (RT-safe) and the `OnConfigure*` / `OnActivate*` lifecycle hooks as needed (base provides default no-op impls).
4. If the controller needs to consume the new group: add subscribe topic routing in the controller's YAML `topics:` section (`role: target` typical), and handle the new device index in controller `Compute()` / `SetDeviceTarget()`.
5. If kinematics needed: add `sub_models` or `tree_models` entry under `urdf:`.

## Adding a New Thread

1. Define `ThreadConfig` for all core tiers in `rtc_base/threading/thread_config.hpp`
2. Add to `SystemThreadConfigs`, update `ValidateSystemThreadConfigs()` + `SelectThreadConfigs()`
3. Call `ApplyThreadConfig()` at thread entry; use SCHED_FIFO for RT threads

## Updating an Existing Package

코드 변경은 *대응 문서·메타데이터 동기화*를 포함해야 완료 ([invariants.md](invariants.md) PROC-1). 동기화 대상은:

- **Tests** — `<package>/test/` 의 affected suite 갱신 + 신규 동작에 대한 test 추가 (RT-7: 기존 assertion 수정 금지, 새 코드 수정)
- **CMakeLists.txt** — source / install / `find_package` / `ament_add_gtest` / `rosidl_generate_interfaces` 일관성
- **package.xml** — deps · version. `CMakeLists.txt` `find_package` 와 1:1 매칭
- **YAML config** — 추가/제거/이름변경된 parameter, `topics:` 섹션, valid range·unit 주석. Robot-specific 값은 `<robot>_bringup/config/<robot>/...`, 기본값은 agnostic 패키지에
- **Doc** — Package README.md (API / parameter / usage), inline Doxygen, cross-package 변경이면 root README + `docs/*.md`

검증:

```bash
./build.sh -p <package>
colcon test --packages-select <package> [<deps>...] --event-handlers console_direct+
colcon test-result --verbose
```

`rtc_base` / `rtc_msgs` 변경은 전체 downstream 빌드·테스트 (PROC-3) — [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Phase 2 PROC-3 fallback 이 자동 수행. downstream ≥4 패키지 + 각 빌드 ≥5 분이면 `Agent` worktree fork-join 으로 병렬 build/test 가 직렬 `./build.sh full` 보다 빠름 (disk+RAM 비용 증가).

## Completion Checklist

[.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Stop hook 이 turn 종료 시 자동 수행: build (변경 패키지) · test · README/CMake co-update · ARCH grep. Hook 차단 ≠ "build 실패" — `package.xml` / YAML / Doxygen 은 hook 가 검증하지 않으므로 에이전트가 직접 확인해야 한다.

수동 self-check (hook 미커버 항목):

- [ ] `package.xml` deps · version (CMake `find_package` 와 일치)
- [ ] YAML default · 범위·unit 주석
- [ ] Doxygen public header 갱신
- [ ] RT path 변경 시 `[invariants.md](invariants.md)` RT-1~8 grep 자가검사
