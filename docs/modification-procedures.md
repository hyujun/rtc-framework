# Modification Procedures — 단계별 절차와 그 이유

> **이 문서는 헌법이 아니다.** 수정·추가 작업의 **규범** (어떤 게이트를 지나야 하고 무엇이 성립해야 하는가) 은 [agent_docs/modification-guide.md](../agent_docs/modification-guide.md) 가 갖고, 여기에 다시 적지 않는다. 여기는 그 규범을 실행하는 단계의 순서 · 파일 위치 · 각 단계가 왜 그런지다. guide 의 절이 여기에 없으면 덧붙일 절차가 없다는 뜻이다 (Workflow Fail-Safe · Sprint Contract · Completion Checklist). 둘이 어긋나면 agent_docs 쪽이 옳다. 파일 위치·이름은 기록 시점의 것이다.

## Workflow Loop

단계 목록과 "4·5·6 은 반드시 수행한다" 는 [modification-guide.md](../agent_docs/modification-guide.md) §Workflow Loop 가 갖는다. 단계별로 덧붙일 것:

- **Locate** — 심볼을 알면 grep / Glob, 범위가 넓으면 탐색용 subagent. "찾았다고 추정" 하지 않는다
- **Edit** — 저장 전에 `auto` (Eigen expression) · quaternion `lerp` · RT 금지 호출을 스스로 grep 한다 ([invariants.md](../agent_docs/invariants.md) §위반 탐지 패턴)
- **Build · Test** — `./build.sh`·`colcon test` 를 직접 돌리는 것은 빠른 피드백용이다. Claude Code 에서 turn 을 끝낼 수 있는 verdict 는 `.claude/hooks/verify-changes.sh --run` 만 남긴다
- **차단이 반복될 때** — Stop hook 의 재진입은 `stop_hook_active` 로 가드되어 stop cycle 당 1회만 발화하므로 turn 이 무한히 물리지는 않는다. 8회 연속 차단 뒤의 override 는 Claude Code 의 동작이다 ([공식 best-practices](https://code.claude.com/docs/en/best-practices), "Give Claude a way to verify its work")
- hook 이 *무엇을* 검사하고 무엇이 blocking 인지 (변경 집합 산정 · non-blocking checklist · pure-format skip) 는 [verify-changes.sh](../.claude/hooks/verify-changes.sh) 헤더 주석이 SSoT 다 — 여기에 옮겨 적지 않는다

## Adding a New Controller

**먼저 [design-principles.md](../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms 의 3계층 배치표를 읽는다** — 새 컨트롤러는 **코어(법칙) + 바인딩(프레임워크 계약)** 둘로 나눠 쓰며, 한 클래스로 쓰지 않는다. 경계 판정은 "이 코드가 `RTControllerInterface` 의 존재를 알아야 하는가?" 한 줄이다. 아래 1–2 가 코어, 4–7 이 바인딩이다.

*(`rtc_controllers` 에 있던 상속 어댑터는 #236 S1–S7 에서 삭제됐다 — 옛 커밋에서 그 모양을 발견하더라도 템플릿으로 삼지 않는다. 복사할 출발점은 `integrated_bringup/src/controllers/` 의 바인딩이다.)*

1. **코어 헤더** — `rtc_controllers/include/rtc_controllers/<family>/` 에 법칙만. 입출력은 Eigen / `std::span` 이고 `ControllerState`/`ControllerOutput`·lifecycle·mailbox 를 모른다. `Resize()`(off-RT, 할당 허용) / `Compute()`(`noexcept`, heap-free) 분리. 참조 구현은 `rtc_controllers/include/rtc_controllers/compliance/` 의 `task_dynamics.hpp`·`impedance_law.hpp`.
2. **코어 파라미터** — 코어 옆에 `Params` POD + `ParseXxxParams(YAML::Node)` 자유 함수 (yaml-cpp 만 의존, 비-RT). 프레임워크 타입을 참조하지 않으므로 코어와 같은 층에 남는다.
3. **바인딩 클래스** — integration 패키지(`integrated_bringup/`)에서 `RTControllerInterface` 를 상속하고 코어를 멤버로 소유한다. `Compute()`, `SetDeviceTarget()`, `Name()` (all `noexcept`) 구현 + `ControllerState` 해체 → 코어 호출 → `ControllerOutput` 조립. **`Name()` 은 전역 유일해야 한다** — CM 은 `Name()` 과 `config_key` 를 하나의 lookup 네임스페이스에 넣으므로, 컨트롤러 클래스를 복사하고 `Name()` 문자열을 안 고치면 bring-up 전체가 거부된다 (경고 아님; [rtc_controller_manager/README.md](../rtc_controller_manager/README.md) §식별자 충돌 가드). 같은 이유로 **한 클래스를 두 `config_key` 로 등록할 수 없다**. `DeviceStateCache` 에서 무엇을 읽고 무엇을 스스로 계산해야 하는지는 [design-principles.md](../agent_docs/design-principles.md) §Backend / Controller Layering.
4. **Runtime gains** — 바인딩의 LifecycleNode (`/<config_key>`) ROS 2 parameter 로 노출: `on_configure` 에서 `DeclareGainParameters()` + `add_on_set_parameters_callback(OnGainParametersSet)`. Read-only 캡(`*_max_traj_velocity`)은 `ParameterDescriptor::read_only=true`. Force-PI 같은 one-shot 이벤트는 [rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv) 같은 srv 채널을 별도로 마련하고 **상대 이름** `"grasp_command"` 로 advertise 한다 — 노드 namespace 기준 `/<config_key>/grasp_command` 로 해석되며, `~/` 를 쓰면 이름이 한 번 더 중첩된다 (active controller만 server를 띄움). **코어는 파라미터 채널을 갖지 않는다** — 노드를 만들지 않기 때문이며, 스냅샷을 인자로 받는다.
5. Gains struct must be trivially copyable (plain arrays/bools/doubles/floats/ints; no `std::string`/`std::vector`/virtuals — `rtc::SeqLock` 의 타입 요구, [rtc_base/README.md](../rtc_base/README.md)). Store as `rtc::SeqLock<Gains> gains_lock_` — RT path snapshots once with `const auto gains = gains_lock_.Load();` at method entry; aux-thread writers (parameter callback / srv handler) use Load/mutate/Store. `set_gains`/`get_gains` accessors delegate to the SeqLock and are used by tests.

   **게인 하한은 로더와 tick 양쪽에 건다.** `set_gains()` 는 방금 그 SeqLock 에 POD 를 직접 쓰므로 configure 의 floor 를 통째로 우회한다 — 코어 파서(2번)가 거는 것과 **같은 심볼**을 바인딩 `Compute()` 에서 한 번 더 부른다: 영공간 자세 게인과 compliance §5.3 안전층 게인은 `rtc::FloorNonNegativeGain` (NUM-6; 자세 게인은 활성 게이트 **판정 앞**에), compliance §6.5 DLS 의 λ_max 와 σ₀ 는 `compliance::FloorMaxDamping` / `compliance::FloorSigma0` (둘 다 NUM-1 — NUM-2 는 `dt` guard 라 무관하다). 하한을 `std::max(0.0, ·)` 로 손수 쓰지 않는다 — `std::max` 는 `a < b ? b : a` 라 `max(x, NaN) == x` 이고, 그러면 비유한 게인이 *그럴듯한* 값으로 세탁돼 기존 `nan_inf` SAFE_STOP 을 지운다 (두 헬퍼는 비유한 값을 그대로 통과시켜 그 fault 로 보낸다). λ_max·σ₀ 쪽 첫 in-tree 준수 사례는 `demo_task_controller` 다 (#282) — 새 바인딩도 그 패키지 테스트에 "tick 에서 λ_max / σ₀ 가 floor 된다" 케이스를 함께 넣는다 (#301 의 강제 지점). σ₀ 쪽은 관측 지점을 고르는 데 주의가 필요하다: 잘 조건화된 자세에서는 floor 여부와 무관하게 λ²=0 이라 테스트가 공허해지므로, 랭크 결손 자세(σ_min=0)에서 λ_max 가 명령을 움직이는지로 판정한다. 근거는 [invariants.md](../agent_docs/invariants.md) NUM-1 / NUM-6 이 SSoT.
6. **YAML** — production 은 `integrated_bringup/config/<robot>/controllers/` (바인딩과 같은 패키지가 소유한다; `rtc_controllers/examples/controllers/` 는 `<robot>` placeholder 를 쓰는 **참고용 example** 이라 그대로 로드되지 않으므로 — [rtc_controllers/README.md](../rtc_controllers/README.md) §사용 모델 — 새 컨트롤러가 여기 파일을 추가하지 않는다). `topics:` 섹션을 반드시 포함한다. 어떤 `role:` 문자열이 유효하고 어느 lane 이 controller YAML 밖에 사는지(device-wire → `devices.<group>.backend:`)는 [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §토픽 소유권 · §구독 역할 · §퍼블리시 역할 이 SSoT — **거기 없는 문자열은 오타와 동일하게 configure 를 실패시키므로 추측하지 말고 표를 본다.**
7. **바인딩이 토픽을 소유한다면** — `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `PublishNonRtSnapshot` 을 override 하고 `owned_topics` 헬퍼에 위임한다 (또는 동등 코드를 인라인). 훅별 기본 동작 · 3-pass bring-up 계약 · `on_configure` override 규약은 [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §Lifecycle 훅 이 SSoT. **그 문서가 다루지 않는 한 가지**: `on_activate` override 도 반드시 base 를 먼저 호출해야 한다 — base 가 activation generation 증분 + `ResetTargetInitialization()` 을 수행하므로 (#196 §3), 누락하면 비활성 구간에 쌓인 stale target 이 재활성화 첫 tick 에 적용된다. target-init latch reset 은 `on_activate` 안에 직접 쓰지 말고 `ResetTargetInitialization()` override 에 둔다.
8. Register via `RTC_REGISTER_CONTROLLER()` macro — `config_key` 는 링크되는 모든 TU 에 걸쳐 유일해야 하고, 어떤 컨트롤러의 `Name()` 과도 겹치면 안 된다 (3번 항목의 `Name()` 유일성과 같은 네임스페이스). 등록 대상은 항상 바인딩이며 `integrated_bringup/src/controllers/controller_registration.cpp` 에 둔다 (`config_package` 는 자기 패키지 — `rtc_*` 패키지 이름을 넣으면 ARCH-1 위반)

## Adding a New Message Type

1. Create `rtc_msgs/msg/MyMessage.msg`, add to `CMakeLists.txt` `rosidl_generate_interfaces()`
2. **`PublishRole` 을 늘리기 전에 소유 형태부터 정한다.** 기본 답은 **추가하지 않는 것** 이다 — controller-owned non-RT 토픽은 controller 에 `SeqLock<MyData>` 멤버를 두고 `integrated_bringup/include/integrated_bringup/support/owned_topics.hpp` 에 `SetupMyDataPublisher()` 헬퍼를 작성해 `on_configure`/`on_activate` 에서 호출한다 (RT loop 이 SeqLock writer 로 push, aux thread 가 읽어서 발행). 기존 Grasp/Wbc/ToF wiring 이 canonical pattern 이다. 왜 이 경로가 기본값이고 `PublishRole` 에 무엇이 남아 있는지는 [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §퍼블리시 역할, 소유권 규칙 자체는 [design-principles.md](../agent_docs/design-principles.md) §Controller-YAML Topics Are Controller-Owned.
3. 그래도 새 `PublishRole` 이 필요하다면 **네 곳을 같은 변경 안에서** 고친다 — `rtc_base/types/types.hpp` 의 enum, `PublishRoleToString()`, `rtc_controller_interface/src/rt_controller_interface.cpp` 의 YAML 파서 매핑, 그리고 `integrated_bringup/src/support/owned_topics.cpp` 의 switch 에 **실제 publisher**. 앞의 셋만 하면 선언한 컨트롤러가 에러 없이 죽은 토픽을 얻는다 (#196 Phase 5 가 그 상태로 방치돼 있던 role 을 제거한 경위는 위 §퍼블리시 역할). per-device 면 `GroupCommandSlot` 필드를 추가한다. **E-11 이므로 착수 전 `[CONCERN]`** ([AGENTS.md](../AGENTS.md) §6).

## Adding a New Device Group

1. Add device entry in `integrated_bringup/config/<robot>/{sim,robot}.yaml` under `devices:`. **`devices.<group>.backend:` is the SSoT** — declare `backend.type:` (registered tags: step 3) + backend-specific config (topics, transport endpoints). CM 은 더 이상 controller YAML 에서 device-wire role 을 읽지 않으며 backend 구현체가 read/write lane 소유.
2. Optionally add a timeout entry in `device_timeout_names`/`values`. 설정된 모든 device group 은 자동으로 준비 게이트 + 워치독 대상이 되며, 목록에 없으면 `device_timeout_default_ms` 가 적용된다 (#198) — 목록 누락이 감시 누락을 뜻하지는 않는다.
3. 현재 등록된 backend type 은 3종이다 — `mujoco_native` (sim), `ur_driver_native` (UR RTDE), `udp_hand_native` (hand UDP). 전부 `integrated_bringup/src/backends/` 에 있고 `RTC_REGISTER_DEVICE_BACKEND` 로 등록되며, `devices.<group>.backend.type` (sim.yaml / robot.yaml) 이 이 tag 로 dispatch 한다. 기존 backend 에 새 설정 키만 필요하면 backend 를 추가하지 말고 그 키를 먼저 검토한다. **단일 backend 전용 신규 key 는 `rtc_base` 타입 확장이 아니라 그 backend 의 `Configure()` 에서 nested ROS 2 param 으로 읽는다** — `declare_parameter("devices." + group + ".backend." + <key>, default)`; YAML 의 `/**: ros__parameters:` 블록이 자동 주입하므로 CM·`rtc_base` 변경이 0 이다 (`DeviceBackendConfig` 는 고정 필드만 담고 미지의 YAML key 를 조용히 버리며, 자매 타입이 사는 `rtc_base/types/types.hpp` 변경은 PROC-3 전체 빌드를 부른다). 같은 key 가 두 번째 backend 에 필요해지는 순간이 `rtc_base` 승격 트리거다.
4. If a new backend type is needed: implement the `DeviceBackend` interface (`rtc_controller_manager/include/rtc_controller_manager/device_backend.hpp`) + register via `RTC_REGISTER_DEVICE_BACKEND(my_backend)` macro. Override `ReadState()` / `WriteCommand()` (RT-safe) and the `OnConfigure*` / `OnActivate*` lifecycle hooks as needed (base provides default no-op impls). `DeviceStateCache` 에 무엇을 채우고 무엇을 채우지 않아야 하는지는 [design-principles.md](../agent_docs/design-principles.md) §Backend / Controller Layering — 위 §Adding a New Controller 3번이 가리키는 것과 **같은 규칙의 반대편**이다.
5. If the controller needs to consume the new group: add subscribe topic routing in the controller's YAML `topics:` section (`role: target` typical), and handle the new device index in controller `Compute()` / `SetDeviceTarget()`.
6. If kinematics needed: add `sub_models` or `tree_models` entry under `urdf:`.

### Renaming a Device Group

Group 이름은 config YAML 밖에도 박혀 있어서, `devices:` 블록만 고치면 **조용히 dead topic** 이 남는다 (실제 재발 2회). 다음을 전부 grep 한다:

- **Python 스크립트 / GUI** — `integrated_bringup/scripts/`, demo GUI 등이 group 이름으로 토픽을 조립하는 경로
- **C++ 기본값 (멤버 in-class initializer / struct default / `declare_parameter`)** — 노드가 `"hand"` 같은 group 명을 *기본값*으로 들고 있으면 YAML 을 고쳐도 미지정 실행 경로에서 old 이름이 되살아난다 (예: `ur5e_bt_coordinator` 의 `TopicNamer::hand_group`, `BTCoordinatorNode::hand_group_`)
- **Group 파생 helper** — `GetSecondaryDeviceName()` 류 이름 조립 로직
- **CSV / 로그 컬럼명**, **[architecture.md](../agent_docs/architecture.md) · [controllers.md](../agent_docs/controllers.md) 의 토픽 표**

검증 신호: rename 후 `ros2 topic list` 에 old 이름 토픽이 남아 있거나, 구독자 0인 신규 토픽이 보이면 위 중 하나가 갱신 안 된 것이다.

## Adding a New Thread

스레드 배치는 **선언형 manifest 한 곳**이 소유한다 — tier 상수를 손으로 쓰지 않는다 (issue #153 M1).

1. **[repo_scripts/config/thread_layout.yaml](../repo_scripts/config/thread_layout.yaml)** 에 role 을 추가하고 **모든 tier** 에 slot/policy/priority/nice 를 준다. 새 스레드가 컨트롤러 프로세스 안에서 돌면 `in_controller_process: true` + `verifier_order` 에도 넣는다 (그래야 `verify_rt_runtime.sh` 가 검사한다)
2. `python3 repo_scripts/scripts/gen_thread_layout.py --write` — C++ 상수·shell 헬퍼·Python 미러·README 표가 함께 갱신된다. 생성 파일은 **직접 편집 금지**이며 `--check` 가 CI 에서 차단한다
3. `SystemThreadConfigs` ([thread_config.hpp](../rtc_base/include/rtc_base/threading/thread_config.hpp)) 에 필드를 추가하고, generator 의 `field_order` 에 **구조체 선언 순서 그대로** 같은 키를 넣는다. 어긋남은 손이 아니라 게이트가 잡는다 — manifest role 과 `field_order` 가 불일치하면 생성기가 즉시 멈추고, 구조체 순서와 `field_order` 가 어긋나면 designated initializer 가 컴파일 에러를 낸다 (positional init 이던 시절엔 같은 타입이라 조용히 이웃 role 의 slot·priority 를 가져갔다). 관계 규칙이 필요하면 `ValidateSystemThreadConfigs()` ([thread_utils.hpp](../rtc_base/include/rtc_base/threading/thread_utils.hpp)) 에 넣되, 호스트보다 큰 tier 는 그 validator 로 검사할 수 없으므로 (`cpu_core` 를 호스트 코어 수와 대조한다) manifest 레벨 불변식은 `gen_thread_layout.py` 의 `run_self_test()` 에 함께 넣는다
4. **각 언어의 리터럴 oracle 을 갱신**한다 — `rtc_base/test/test_thread_layout_tiers.cpp` · `repo_scripts/test/test_rt_common.sh` · `rtc_tools/test/test_thread_layout.py`. 이 셋은 생성물이 아니라 손으로 쓴 기대값이며, 잘못된 manifest 는 세 언어에서 사이좋게 일치하므로 **`--check` 로는 절대 잡히지 않는다**. 여기를 안 고치면 새 스레드는 검증 없이 배포된다
5. 스레드 진입점에서 `ApplyThreadConfig()` 호출; RT 스레드는 SCHED_FIFO

레이아웃 **값** 변경 (기존 role 의 코어 이동) 은 리팩터가 아니라 thread model 변경이므로 [AGENTS.md](../AGENTS.md) §6 **E-7** 대상이다.

## Adding a New Package (new colcon directory)

새 `<pkg>/package.xml` 디렉토리를 추가하면 build SSoT 를 갱신해야 한다 (C++ CI 와 그 패키지 목록은 2026-09-19 에 제거됐다):

- **[repo_scripts/scripts/lib/rt_common.sh](../repo_scripts/scripts/lib/rt_common.sh) `get_base_packages()` (또는 `get_robot_packages()`)** — `build.sh` / `install.sh` 가 `--packages-select`(**비전이**)로 소비. 누락 시 그 패키지를 의존하는 downstream 의 클린 `./build.sh` 가 `find_package(<pkg>)` 에서 실패. rtc_* 빌드 의존이 없으면 `rtc_base` 직후처럼 앞쪽에 둔다.

README 패키지 표·count, [architecture.md](../agent_docs/architecture.md) dependency graph 는 아래 §Updating an Existing Package 의 Doc 동기화 규칙(PROC-1)을 따른다.

## Updating an Existing Package

동기화 대상 목록 (Tests · CMakeLists.txt · package.xml · YAML config · Doc) 은 [modification-guide.md](../agent_docs/modification-guide.md) §Updating an Existing Package 가 갖는다. 그 항목의 사례와 명령:

- **package.xml 의 예외가 실제로 나타나는 곳** — rosdep 이 해결할 수 없는 source-install 의존은 `rtc_mpc` 의 `aligator`·`fmt` 처럼 `deps/install` 에서 `CMAKE_PREFIX_PATH` 로 찾는 것이다. `<depend>` 로 올리면 rosdep 이 실패하므로 CMake 에만 두고 사유를 주석으로 남긴다 (그 파일의 기존 주석이 예시). 같은 이유의 다른 발현: `ament_python` 은 jazzy rosdep DB 에 없어 `<buildtool_depend>ament_python</buildtool_depend>` 을 선언하면 `rosdep resolve --ignore-src` 가 ERROR 를 낸다 — 발견하면 삭제한다. `<test_depend>` 는 없는 런타임 결합을 과장하지 않으면서 빌드 순서는 똑같이 얻는다
- **CMakeLists.txt 에서 맞춰 볼 곳** — source / install / `find_package` / `ament_add_gtest` / `rosidl_generate_interfaces`
- **YAML 의 robot-specific 값** 은 `integrated_bringup/config/<robot>/...` 에 둔다

검증:

```bash
./build.sh --tests -p <package>   # build.sh 는 기본으로 테스트를 빌드하지 않는다
colcon test --packages-select <package> [<deps>...] --event-handlers console_direct+
colcon test-result --verbose
```

`rtc_base` / `rtc_msgs` 변경의 전체 downstream 빌드·테스트 (PROC-3) 는 [.claude/hooks/verify-changes.sh](../.claude/hooks/verify-changes.sh) `--run` 의 PROC-3 경로가 수행한다. downstream ≥4 패키지 + 각 빌드 ≥5 분이면 `Agent` worktree fork-join 으로 병렬 build/test 가 직렬 `./build.sh full` 보다 빠르다 (disk+RAM 비용 증가).

## Inferential review 트리거 (LLM-as-judge, 수동 trigger)

트리거 목록은 [modification-guide.md](../agent_docs/modification-guide.md) §Inferential review 트리거 가 갖는다. 각 트리거가 왜 거기 있는가:

- `rtc_base` / `rtc_msgs` 변경 — downstream 전 패키지에 영향이 간다
- Abstract interface 신설 / 두 번째 구현 — base 누락과 `#ifdef` 분기 유혹은 빌드·테스트가 못 본다
- PR 준비 — `/code-review ultra` 는 현재 branch 를, `/code-review ultra <PR#>` 는 GitHub PR 을 본다. `/ultrareview` 는 deprecated alias
- `/simplify` 는 재사용·단순화 전용이다 — 버그 탐지는 `/code-review`

수동 trigger 인 이유: inferential 은 비용·지연이 크고 non-deterministic 이므로 모든 변경에 자동 적용하면 ROI 가 음성이다. 목록은 "false-negative 비용 > inferential 비용" 인 경우만 추렸다.

## Post-task housekeeping (상세)

항목 목록·요구 수준은 [AGENTS.md](../AGENTS.md) §11, 실행 기준은 [modification-guide.md](../agent_docs/modification-guide.md) §Post-task housekeeping 이 갖는다. 그 위에 덧붙일 것:

- **Issue 동기화의 이유** — issue 는 durable 결정 기록이자 cross-tool 인계면이다 ([handoff.md](../agent_docs/handoff.md) §5). 갱신 없이 닫으면 다음 도구가 읽을 것이 없다
- **Scratch 파일** — repo-root / `/tmp` 에 남은 분석 스크립트·중간 산출물·로그 덤프는 그 내용이 다른 곳 (git log, `docs/*.md`, issue) 에 보존됐는지 확인한 뒤 지운다
- **Branch prune 명령** — 로컬 merged branch 는 `git branch -d <branch>`, stale remote-tracking ref 는 `git fetch --prune`. 원격 branch 삭제는 merge 확인 후에만 한다 (GitHub auto-delete 미설정 시)
- **Claude Code 의 memory·harness 정리** 는 [CLAUDE.md](../CLAUDE.md) §Claude Code 의 Housekeeping 이 가리키는 곳이 갖는다
