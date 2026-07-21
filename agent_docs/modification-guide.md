# Modification Guide

## Workflow Loop

모든 수정 작업은 이 순서 ([CLAUDE.md](../CLAUDE.md) §4 요약판의 상세).

```
0. Type     → "수정"인가 "추가(새 기능/컨트롤러/메시지/디바이스/스레드)"인가?
              추가라면 [design-principles.md](design-principles.md) 5원칙 + 본 문서
              "Adding a New ..." 절을 먼저 읽는다.
              · rtc_*에 추가 → P1·P2 (zero source edit, robot 상수 금지) +
                ARCH-3 (interface-first; 같은 종류 두 번째 구현이면 base부터)
              · integration 패키지에 추가 → 재사용 가능한 부분이
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

**※ 4·5·6은 [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) Stop hook이 turn 종료 시 자동 실행/차단한다 — 사전 수동 실행은 빠른 피드백용. Hook 한계: 변경 패키지만 빌드, 60s timeout per package, README/CMake만 검사 (package.xml/YAML은 미검). 또한 **무한 차단이 아니다** — 8회 연속 `exit 2` 후 Claude Code 가 hook 을 override 하고 turn 을 종료하므로 (code.claude.com/docs best-practices), 지속 실패 시 에이전트가 주입된 리포트에 직접 대응해야 한다. Pure-format commit (모든 변경 .cpp/.hpp/.py 가 `clang-format`/`ruff format` round-trip 결과와 동일) 은 ARCH grep + README/CMake co-update 단계만 자동 skip 되고 build/test 는 그대로 돈다.**

### Workflow Fail-Safe

각 단계 실패 시 대응. "Try harder"는 실패 응답이 아니다 — 누락된 capability를 엔지니어링하거나 [CLAUDE.md](../CLAUDE.md) §6 Escalate.

| 실패 단계 | 증상 | 대응 |
|----------|------|------|
| 1. Locate | 파일을 찾을 수 없음 | `Agent` subagent로 broad search. "찾았다고 추정" 금지 |
| 2. Read | 컨텍스트 불충분 (호출자 / 테스트 미확인) | 인접 파일 + 테스트 추가 읽기. 추측하지 말 것 |
| 3. Edit | Invariant 위반 유혹 | [invariants.md](invariants.md) 확인 후 [CLAUDE.md](../CLAUDE.md) §6 Escalate. 우회로 찾지 말 것 |
| 4. Build | 빌드 실패 | 에러 메시지를 **먼저** 기록. 원인 파악 전 재시도 금지 |
| 5. Test | 테스트 실패 | assertion 을 통과시키려 약화 금지 ([anti-patterns.md](anti-patterns.md) AP-PROC-4 / [invariants.md](invariants.md) PROC-6). 새 코드를 고칠 것 — 단 test 가 진짜 틀렸으면 별도 commit + 근거 + E-6 |
| 6. Verify | Checklist 항목 실패 | 해당 항목까지 rollback, 재실행. 부분 완료 주장 금지 ([anti-patterns.md](anti-patterns.md) AP-PROC-1) |

## Sprint Contract & Spec (착수 전 성공 기준)

언제 Sprint Contract 를 협상하고 무엇이 면제인지는 [CLAUDE.md](../CLAUDE.md) §6.5 (always-loaded 헌법) 가 trigger 목록의 SSoT. 여기엔 *포맷과 절차* 만 둔다 (가끔만 필요하므로 on-demand).

코드 수정 시작 *전* 1~3줄로 성공 기준을 사용자에게 제시하고 컨펌받는다:

```
[SPRINT] <task 한 줄 요약>
Done when:
  - <검증 가능 기준 1>
  - <검증 가능 기준 2>
  - <...>
Out of scope: <명시적으로 하지 않을 것 — drift 방지>
```

기준은 **객관 검증 가능** 해야 한다 (예: "test_X 통과", "rtc_* 에 ur5e grep 0건", "rtc_cm 빌드 0 warning"). "코드가 깔끔하다", "잘 작동한다" 같은 주관 기준은 금지. 이 컨트랙트는 task 종료 시 [CLAUDE.md](../CLAUDE.md) §11 보고에서 항목별 충족 여부를 체크한다.

**Spec-driven (신규 abstract interface · controller · 메시지 · 디바이스 추가 시):** Sprint Contract = spec. 구현 전 `~/.claude/plans/<slug>.md` 에 *왜 필요한가 · API surface · 검토한 alternatives* 를 1-paragraph spec 으로 박는다 (Specify *before* Implement). 같은 파일이 이후 handoff artifact · 진행 progress 도 누적하므로 `## Spec` / `## Progress` / `## Handoff` 섹션으로 구분 ([CLAUDE.md](../CLAUDE.md) §6.6 handoff 경로와 동일 파일).

## Adding a New Controller

1. Header in `rtc_controllers/include/rtc_controllers/{direct|indirect}/` -- inherit `RTControllerInterface`, implement `Compute()`, `SetDeviceTarget()`, `InitializeHoldPosition()`, `Name()` (all `noexcept`). **`Name()` 은 전역 유일해야 한다** — CM 은 `Name()` 과 `config_key` 를 하나의 lookup 네임스페이스에 넣으므로, 컨트롤러 클래스를 복사하고 `Name()` 문자열을 안 고치면 bring-up 전체가 거부된다 (경고 아님; [rtc_controller_manager/README.md](../rtc_controller_manager/README.md) §식별자 충돌 가드). 같은 이유로 **한 클래스를 두 `config_key` 로 등록할 수 없다**. **Layering rule** ([design-principles.md](design-principles.md) §Backend / Controller Layering): controllers read only the `DeviceStateCache` slots they consume — derived quantities (`force_magnitude`, `in_contact`, slip, ...) are computed in the controller, never pulled from the backend, and never added to `DeviceStateCache`.
2. Source in `rtc_controllers/src/controllers/{direct|indirect}/` -- `LoadConfig()` for YAML. Runtime gains는 컨트롤러 자체 LifecycleNode (`/<config_key>`) 의 ROS 2 parameter로 노출 — `on_configure`에서 `DeclareGainParameters()` + `add_on_set_parameters_callback(OnGainParametersSet)`. Read-only 캡(`*_max_traj_velocity`)은 `ParameterDescriptor::read_only=true`. Force-PI 같은 one-shot 이벤트는 [rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv) 같은 srv 채널을 별도로 마련하고 `~/grasp_command`로 advertise (active controller만 server를 띄움).
3. Gains struct must be trivially copyable (plain arrays/bools/doubles/floats/ints; no `std::string`/`std::vector`/virtuals). Store as `rtc::SeqLock<Gains> gains_lock_` — RT path snapshots once with `const auto gains = gains_lock_.Load();` at method entry; aux-thread writers (parameter callback / srv handler) use Load/mutate/Store. `set_gains`/`get_gains` accessors delegate to the SeqLock and are used by tests.
4. YAML in `rtc_controllers/examples/controllers/` (reference only) or `integrated_bringup/config/<robot>/controllers/` (production) -- must include `topics:` section. **Subscribe roles** (`role: target`) 와 **publish role** (`role: robot_transforms` — #196 Phase 5 에서 publisher 가 없던 `robot_target` / `joint_goal` / `digital_twin_state` 는 제거, 이제 configure 실패) 은 전부 controller-owned 이므로 (issue #138: `ownership:` field 없음) **relative path** 를 쓰면 노드 namespace `/<config_key>/` 아래로 해석됩니다 (`<group>/joint_goal`, not `/<group>/joint_goal`). Device-wire lane 은 controller YAML 이 아니라 `devices.<group>.backend:` 에 선언합니다. **Controller-owned non-RT topics that are NOT in PublishRole** (`grasp_state` / `wbc_state` / `tof_snapshot`): controller 가 자체 `SeqLock<{Grasp,Wbc,ToF}StateData>` writer 소유 + `integrated_bringup/support/owned_topics.hpp` 의 `Setup{Grasp,Wbc,ToF}*Publisher` 헬퍼 호출 — `topics:` entry 없음, PublishRole 없음; topic name 은 helper 에 hardcoded.
5. If the controller owns any topics: override `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `PublishNonRtSnapshot` and delegate to the `owned_topics` helper (or inline the equivalent). Always call the base `RTControllerInterface::on_configure` / `on_cleanup` first, and `on_activate` 오버라이드는 반드시 base 를 호출해야 한다 — base 가 activation generation 증분 + `ResetTargetInitialization()` 을 수행하므로 (#196 §3), 누락 시 비활성 구간에 쌓인 stale target 이 재활성화 첫 tick 에 적용된다. target-init latch reset 은 `on_activate` 안에 직접 쓰지 말고 `ResetTargetInitialization()` 오버라이드에 둔다. **CM bring-up 은 3-pass** (`PreConfigure` → `SetDeviceNameConfigs` → `on_configure`) 라서 `on_configure` 안에서 `RegisterLog<>(...)` 의 람다가 `joint_state_names`/`motor_state_names` 등을 capture할 때 `OnDeviceConfigsSet()` 가 이미 실행됐음이 보장됩니다 — 별도 시점 조정 불필요. `PreConfigure` 는 base 전용이므로 override 금지.
6. Register via `RTC_REGISTER_CONTROLLER()` macro — `config_key` 는 링크되는 모든 TU 에 걸쳐 유일해야 하고, 어떤 컨트롤러의 `Name()` 과도 겹치면 안 된다 (1번 항목의 `Name()` 유일성과 같은 네임스페이스). Robot-specific controllers go in `integrated_bringup/` with registration in `integrated_bringup/src/controllers/controller_registration.cpp`

## Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`, add to `CMakeLists.txt` `rosidl_generate_interfaces()`
2. **Decide ownership before adding a PublishRole.** PublishRole is reserved for controller-owned YAML-declared output streams that actually have a publisher wired up — after #196 Phase 5 that is `kRobotTransforms` alone. Controller-owned non-RT topics (Grasp/Wbc/ToF 스타일) 는 PublishRole 추가 금지 — 대신: controller 에 `SeqLock<MyData>` 멤버 선언, `integrated_bringup/support/owned_topics.hpp` 에 `SetupMyDataPublisher()` 헬퍼 작성, controller 가 `on_configure`/`on_activate` 에서 호출 (RT loop pushes via the SeqLock writer; aux thread reads + publishes). 기존 Grasp/Wbc/ToF wiring 이 canonical pattern.
3. If you genuinely need a new PublishRole: add it to `rtc_base/types/types.hpp` `PublishRole` enum + `PublishRoleToString()` + YAML mapping in `rtc_controller_interface/src/rt_controller_interface.cpp` parser, **그리고 같은 변경 안에서 `integrated_bringup/src/support/owned_topics.cpp` 의 switch 에 실제 publisher 를 만든다** — 매핑만 추가하고 소비자를 빠뜨리면 선언한 컨트롤러가 에러 없이 죽은 토픽을 얻는다 (#196 Phase 5 가 그렇게 방치돼 있던 role 2개를 제거했다). Add a `GroupCommandSlot` field if it's per-device.

## Adding a New Device Group

1. Add device entry in `integrated_bringup/config/<robot>/{sim,robot}.yaml` under `devices:`. **`devices.<group>.backend:` is the SSoT** — declare `backend.type:` (registered tags: step 3) + backend-specific config (topics, transport endpoints). CM 은 더 이상 controller YAML 에서 device-wire role 을 읽지 않으며 backend 구현체가 read/write lane 소유.
2. Optionally add a timeout entry in `device_timeout_names`/`values`. 설정된 모든 device group 은 자동으로 준비 게이트 + 워치독 대상이 되며, 목록에 없으면 `device_timeout_default_ms` 가 적용된다 (#198) — 목록 누락이 감시 누락을 뜻하지는 않는다.
3. 현재 등록된 backend type 은 3종이다 — `mujoco_native` (sim), `ur_driver_native` (UR RTDE), `udp_hand_native` (hand UDP). 전부 `integrated_bringup/src/backends/` 에 있고 `RTC_REGISTER_DEVICE_BACKEND` 로 등록되며, `devices.<group>.backend.type` (sim.yaml / robot.yaml) 이 이 tag 로 dispatch 한다. 기존 backend 에 새 설정 키만 필요하면 backend 를 추가하지 말고 그 키를 먼저 검토한다.
4. If a new backend type is needed: implement the `DeviceBackend` interface (`rtc_controller_manager/include/rtc_controller_manager/device_backend.hpp`) + register via `RTC_REGISTER_DEVICE_BACKEND(my_backend)` macro. Override `ReadState()` / `WriteCommand()` (RT-safe) and the `OnConfigure*` / `OnActivate*` lifecycle hooks as needed (base provides default no-op impls). **Layering rule** ([design-principles.md](design-principles.md) §Backend / Controller Layering): backend packs all raw HW values into `DeviceStateCache` (zero-fill unused stride slots — do not skip), never derived quantities. Derived values (`force_magnitude`, `in_contact`, slip rate, ...) belong in the controller.
5. If the controller needs to consume the new group: add subscribe topic routing in the controller's YAML `topics:` section (`role: target` typical), and handle the new device index in controller `Compute()` / `SetDeviceTarget()`.
6. If kinematics needed: add `sub_models` or `tree_models` entry under `urdf:`.

## Adding a New Thread

1. Define `ThreadConfig` for all core tiers in `rtc_base/threading/thread_config.hpp`
2. Add to `SystemThreadConfigs`, update `ValidateSystemThreadConfigs()` + `SelectThreadConfigs()`
3. Call `ApplyThreadConfig()` at thread entry; use SCHED_FIFO for RT threads

## Adding a New Package (new colcon directory)

새 `<pkg>/package.xml` 디렉토리를 추가하면 **두 build SSoT를 모두** 갱신해야 한다 — 서로 다른 colcon 셀렉터를 쓰므로 누락 시 실패 양상이 다르다:

1. **[repo_scripts/scripts/lib/rt_common.sh](../repo_scripts/scripts/lib/rt_common.sh) `get_base_packages()` (또는 `get_robot_packages()`)** — `build.sh` / `install.sh` 가 `--packages-select`(**비전이**)로 소비. 누락 시 그 패키지를 의존하는 downstream 의 클린 `./build.sh` 가 `find_package(<pkg>)` 에서 실패. rtc_* 빌드 의존이 없으면 `rtc_base` 직후처럼 앞쪽에 둔다.
2. **[.github/ci-packages.yml](../.github/ci-packages.yml)** — CI 는 `build` 를 `--packages-up-to`(**전이**)로 빌드하므로 빌드 자체는 안 깨지지만, gtest / cppcheck / coverage 는 `test_cpp_*` · `lint_cpp` · `coverage_paths` 에 명시해야 **실행**된다. `test_cpp_*` 는 두 갈래이며 의미가 다르다 — **`test_cpp_gated`** 는 실패 시 PR 을 빨갛게 만들고, **`test_cpp_besteffort`** 는 CI 환경에서 불안정한 패키지(TSID solve-time, mimalloc grasp timing 의존)라 coverage 수집만 하고 PR 을 막지 않는다. 즉 "CI 에 있으나 게이트하지 않는" 패키지가 존재하므로, besteffort 목록의 테스트가 로컬에서 깨졌다면 CI green 을 근거로 넘기면 안 된다. 커버리지 리포트 설정은 [codecov.yml](../codecov.yml) 이 소유한다. 의도적으로 제외하면 파일 상단 "의도적 누락" 목록에 사유와 함께 기록 — 안 그러면 silent gap (테스트가 CI 에서 한 번도 안 돌아도 green). Header-only 패키지는 `rtc_base` 선례(test + lint + include-only coverage)를 따른다.

README 패키지 표·count, [architecture.md](architecture.md) dependency graph 는 아래 §Updating an Existing Package 의 Doc 동기화 규칙(PROC-1)을 따른다.

## Updating an Existing Package

코드 변경은 *대응 문서·메타데이터 동기화*를 포함해야 완료 ([invariants.md](invariants.md) PROC-1). 동기화 대상은:

- **Tests** — `<package>/test/` 의 affected suite 갱신 + 신규 동작에 대한 test 추가 (PROC-6: 기존 assertion 을 통과시키려 약화 금지 — 새 코드를 고치되, test 가 진짜 틀렸으면 별도 commit + 근거 + E-6)
- **CMakeLists.txt** — source / install / `find_package` / `ament_add_gtest` / `rosidl_generate_interfaces` 일관성
- **package.xml** — deps · version. `CMakeLists.txt` `find_package` 와 1:1 매칭
- **YAML config** — 추가/제거/이름변경된 parameter, `topics:` 섹션, valid range·unit 주석. Robot-specific 값은 `integrated_bringup/config/<robot>/...`, 기본값은 agnostic 패키지에
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
- [ ] RT path 변경 시 [invariants.md](invariants.md) §위반 탐지 패턴 의 `detect` 블록으로 자가검사 (RT-1~RT-10, RT-7 은 은퇴)
