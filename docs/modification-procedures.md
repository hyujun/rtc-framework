# Modification Procedures — 추가·변경 작업의 단계별 절차

컨트롤러 · 메시지 · device group · 스레드 · 패키지를 추가하거나 바꿀 때 따라 하는 **단계별 절차**다. 게이트 같은 규범은 [agent_docs/modification-guide.md](../agent_docs/modification-guide.md) 가 갖고, 둘이 어긋나면 그쪽이 옳다.

## Adding a New Controller

먼저 [design-principles.md](../agent_docs/design-principles.md) §`rtc_controllers` Controllers Are Pure Control Algorithms 의 3계층 배치표를 읽는다. 컨트롤러는 **코어(제어 법칙) + 바인딩(프레임워크 계약)** 으로 나눠 쓴다. 1–2 가 코어, 3–8 이 바인딩이다.

1. **코어 헤더** — `rtc_controllers/include/rtc_controllers/<family>/` 에 법칙만 둔다. 입출력은 Eigen / `std::span` 이고 `ControllerState`·lifecycle 을 모른다. `Resize()` (off-RT) 와 `Compute()` (`noexcept`, heap-free) 를 분리한다. 참조: `rtc_controllers/include/rtc_controllers/compliance/impedance_law.hpp`.
2. **코어 파라미터** — 코어 옆에 `Params` POD + `ParseXxxParams(YAML::Node)` 자유 함수 (yaml-cpp 만 의존, 비-RT).
3. **바인딩 클래스** — `integrated_bringup/` 에서 `RTControllerInterface` 를 상속하고 코어를 멤버로 소유한다. `Compute()`, `SetDeviceTarget()`, `Name()` (모두 `noexcept`) 를 구현한다. **`Name()` 은 전역 유일해야 한다.** 클래스를 복사하고 `Name()` 을 안 고치면 bring-up 전체가 거부된다 ([rtc_controller_manager/README.md](../rtc_controller_manager/README.md) §식별자 충돌 가드).
4. **Runtime gains** — 바인딩의 LifecycleNode (`/<config_key>`) ROS 2 parameter 로 노출한다: `on_configure` 에서 `DeclareGainParameters()` + `add_on_set_parameters_callback(OnGainParametersSet)`. one-shot 이벤트는 [rtc_msgs/srv/GraspCommand](../rtc_msgs/srv/GraspCommand.srv) 처럼 srv 로 두고 **상대 이름** `"grasp_command"` 로 advertise 한다 (`~/` 를 쓰면 이름이 중첩된다).
5. **Gains 저장** — gains struct 는 trivially copyable (`std::string`/`std::vector`/virtual 금지, [rtc_base/README.md](../rtc_base/README.md)) 이고 `rtc::SeqLock<Gains> gains_lock_` 에 둔다. RT 경로는 메서드 진입에서 `const auto gains = gains_lock_.Load();` 로 한 번 스냅샷하고, aux 스레드 writer 는 Load → 수정 → Store 한다. 게인 하한은 로더와 tick 양쪽에 건다 — NUM-6 · [modification-guide.md](../agent_docs/modification-guide.md) 참조.
6. **YAML** — production 파일은 `integrated_bringup/config/<robot>/controllers/` 에 둔다. `rtc_controllers/examples/controllers/` 는 `<robot>` placeholder 를 쓰는 참고용이라 새 파일을 추가하지 않는다 ([rtc_controllers/README.md](../rtc_controllers/README.md) §사용 모델). `topics:` 섹션을 반드시 포함하고, 유효한 `role:` 문자열은 [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §토픽 소유권 · §구독 역할 · §퍼블리시 역할 표에서 고른다 — 표에 없는 문자열은 configure 를 실패시킨다.
7. **토픽을 소유하면** — `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `PublishNonRtSnapshot` 을 override 하고 `owned_topics` 헬퍼에 위임한다 (계약: [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §Lifecycle 훅). `on_activate` override 는 base 를 **먼저** 호출한다 — 누락하면 비활성 구간의 stale target 이 재활성화 첫 tick 에 적용된다. target-init latch reset 은 `ResetTargetInitialization()` override 에 둔다.
8. **등록** — `RTC_REGISTER_CONTROLLER()` 로 등록한다. `config_key` 는 모든 TU 에서 유일하고 어떤 `Name()` 과도 겹치지 않아야 한다. 등록은 `integrated_bringup/src/controllers/controller_registration.cpp` 에 두며 `config_package` 는 자기 패키지다 (`rtc_*` 이름을 넣으면 ARCH-1 위반).

## Adding a New Message Type

1. `rtc_msgs/msg/MyMessage.msg` 를 만들고 `rtc_msgs/CMakeLists.txt` 의 `rosidl_generate_interfaces()` 에 추가한다.
2. **`PublishRole` 은 기본적으로 추가하지 않는다.** controller-owned non-RT 토픽은 컨트롤러에 `SeqLock<MyData>` 멤버를 두고 `integrated_bringup/include/integrated_bringup/support/owned_topics.hpp` 에 `SetupMyDataPublisher()` 헬퍼를 만들어 `on_configure`/`on_activate` 에서 호출한다 (RT 가 SeqLock 에 push, aux thread 가 발행). 기존 Grasp/Wbc/ToF wiring 이 모범이다. 배경: [rtc_controller_interface/README.md](../rtc_controller_interface/README.md) §퍼블리시 역할.
3. 새 `PublishRole` 이 꼭 필요하면 **네 곳을 한 변경 안에서** 고친다: `rtc_base/include/rtc_base/types/types.hpp` 의 enum, `PublishRoleToString()`, `rtc_controller_interface/src/rt_controller_interface.cpp` 의 YAML 파서 매핑, `integrated_bringup/src/support/owned_topics.cpp` 의 switch 에 **실제 publisher**. 앞의 셋만 하면 죽은 토픽이 에러 없이 생긴다. per-device 면 `GroupCommandSlot` 필드를 추가한다. E-11 이므로 착수 전 `[CONCERN]` ([AGENTS.md](../AGENTS.md) §6).

## Adding a New Device Group

1. `integrated_bringup/config/<robot>/{sim,robot}.yaml` 의 `devices:` 아래에 항목을 추가한다. **`devices.<group>.backend:` 가 SSoT** 다 — `backend.type:` 과 backend 별 설정 (topics, transport endpoint) 을 선언한다.
2. 필요하면 `device_timeout_names`/`values` 에 timeout 을 추가한다 (없으면 `device_timeout_default_ms`).
3. 등록된 backend type: `mujoco_native` (sim), `ur_driver_native` (UR RTDE), `udp_hand_native` (hand UDP). `integrated_bringup/src/backends/` 에서 `RTC_REGISTER_DEVICE_BACKEND` 로 등록된다. 설정 키만 더 필요하면 backend 를 새로 만들지 않는다. **단일 backend 전용 key 는 `rtc_base` 타입이 아니라 그 backend 의 `Configure()` 에서 nested ROS 2 param 으로 읽는다**: `declare_parameter("devices." + group + ".backend." + <key>, default)`. 같은 key 가 두 번째 backend 에 필요해지면 `rtc_base` 로 승격한다.
4. 새 backend type 이 필요하면 `DeviceBackend` (`rtc_controller_manager/include/rtc_controller_manager/device_backend.hpp`) 를 구현하고 `RTC_REGISTER_DEVICE_BACKEND(my_backend)` 로 등록한다. `ReadState()` / `WriteCommand()` (RT-safe) 를 override 하고 `OnConfigure*` / `OnActivate*` 는 필요한 것만 override 한다. `DeviceStateCache` 에 채울 내용은 [design-principles.md](../agent_docs/design-principles.md) §Backend / Controller Layering.
5. 컨트롤러가 새 group 을 소비하면 YAML `topics:` 에 subscribe routing (`role: target` 이 일반적) 을 추가하고 `Compute()` / `SetDeviceTarget()` 에서 새 device index 를 처리한다.
6. 운동학이 필요하면 `urdf:` 아래에 `sub_models` 또는 `tree_models` 를 추가한다.

### Renaming a Device Group

group 이름은 config YAML 밖에도 박혀 있어서 `devices:` 만 고치면 조용히 dead topic 이 남는다. 다음을 grep 한다.

- **Python 스크립트 / GUI** — `integrated_bringup/scripts/`, demo GUI 등 group 이름으로 토픽을 조립하는 경로
- **C++ 기본값** (in-class initializer / struct default / `declare_parameter`) — `"hand"` 같은 기본값이 남으면 미지정 경로에서 옛 이름이 되살아난다
- **Group 파생 helper** — `GetSecondaryDeviceName()` 류
- **CSV / 로그 컬럼명**, **문서의 토픽 서술** ([docs/controllers.md](controllers.md) §ROS2 Topics, [architecture.md](../agent_docs/architecture.md) §RT vs non-RT Topic Ownership)

검증: rename 후 `ros2 topic list` 에 옛 이름 토픽이나 구독자 0인 신규 토픽이 보이면 위 중 하나가 빠진 것이다.

## Adding a New Thread

스레드 배치는 **선언형 manifest 한 곳**이 소유한다. tier 상수를 손으로 쓰지 않는다.

1. [repo_scripts/config/thread_layout.yaml](../repo_scripts/config/thread_layout.yaml) 에 role 을 추가하고 **모든 tier** 에 slot/policy/priority/nice 를 준다. 컨트롤러 프로세스 안에서 돌면 `in_controller_process: true` 와 `verifier_order` 에도 넣는다 (`verify_rt_runtime.sh` 가 검사하게 된다).
2. `python3 repo_scripts/scripts/gen_thread_layout.py --write` — C++ 상수·shell 헬퍼·Python 미러·README 표가 함께 갱신된다. 생성 파일은 직접 편집하지 않으며 `--check` 가 CI 에서 차단한다.
3. `SystemThreadConfigs` ([thread_config.hpp](../rtc_base/include/rtc_base/threading/thread_config.hpp)) 에 필드를 추가하고 generator 의 `field_order` 에 **구조체 선언 순서 그대로** 같은 키를 넣는다. 관계 규칙은 `ValidateSystemThreadConfigs()` ([thread_utils.hpp](../rtc_base/include/rtc_base/threading/thread_utils.hpp)), manifest 불변식은 `gen_thread_layout.py` 의 `run_self_test()` 에 둔다.
4. **각 언어의 리터럴 oracle 을 갱신**한다: `rtc_base/test/test_thread_layout_tiers.cpp` · `repo_scripts/test/test_rt_common.sh` · `rtc_tools/test/test_thread_layout.py`. 손으로 쓴 기대값이라 `--check` 로는 잡히지 않는다 — 안 고치면 새 스레드가 검증 없이 배포된다.
5. 스레드 진입점에서 `ApplyThreadConfig()` 를 호출한다. RT 스레드는 SCHED_FIFO.

기존 role 의 코어 이동 같은 레이아웃 **값** 변경은 thread model 변경이므로 [AGENTS.md](../AGENTS.md) §6 **E-7** 대상이다.

## Adding a New Package (new colcon directory)

새 `<pkg>/package.xml` 디렉토리를 추가하면 [repo_scripts/scripts/lib/rt_common.sh](../repo_scripts/scripts/lib/rt_common.sh) 의 `get_base_packages()` (또는 `get_robot_packages()`) 에 패키지를 넣는다. `build.sh` / `install.sh` 가 이 목록을 `--packages-select` (비전이) 로 소비하므로, 누락하면 이 패키지에 의존하는 downstream 의 클린 `./build.sh` 가 `find_package(<pkg>)` 에서 실패한다. README 패키지 표와 [architecture.md](../agent_docs/architecture.md) dependency graph 는 아래 절의 문서 동기화를 따른다.

## Updating an Existing Package

동기화 대상 (Tests · CMakeLists.txt · package.xml · YAML config · Doc) 은 [modification-guide.md](../agent_docs/modification-guide.md) §Updating an Existing Package 가 갖는다. 사례:

- **package.xml 예외** — rosdep 이 못 푸는 source-install 의존 (`rtc_mpc` 의 `aligator`·`fmt` 등) 은 `<depend>` 가 아니라 CMake 에서 `CMAKE_PREFIX_PATH` 로 찾고 사유를 주석으로 남긴다. `ament_python` 은 jazzy rosdep DB 에 없으므로 `<buildtool_depend>` 로 선언하지 않는다.
- **CMakeLists.txt** — source / install / `find_package` / `ament_add_gtest` / `rosidl_generate_interfaces` 를 맞춰 본다.
- **robot-specific YAML 값** 은 `integrated_bringup/config/<robot>/...` 에 둔다.

### 빌드·테스트

```bash
./build.sh --tests -p <package>   # build.sh 는 기본으로 테스트를 빌드하지 않는다
colcon test --packages-select <package> [<deps>...] --event-handlers console_direct+
colcon test-result --verbose
```

`colcon` 은 colcon workspace root (`~/ros2_ws/rtc_ws`) 에서 실행한다 ([AGENTS.md](../AGENTS.md) §9.1). `rtc_base` / `rtc_msgs` 를 바꿨으면 downstream 전체를 빌드·테스트한다 (PROC-3).

### 테스트 단언을 바꾸는 commit 쌍 (PROC-6)

규범은 [invariants.md](../agent_docs/invariants.md) PROC-6 · E-6 이고, 여기는 순서만 갖는다.

1. **spec · 법칙 commit** — 코드 · 문서만 바꾼다. 아직 고치지 않은 단언 때문에 red 이므로, 본문에 "의도된 red" 와 다음 commit 이 고칠 단언 목록을 적는다. 이 commit 을 한 번 빌드해 red 집합이 그 목록과 같은지 본다 — 목록 밖의 red 는 E-6 로 escalate 한다.
2. **E-6 commit** — 단언만 다시 고정하고, 무엇을 무엇으로 바꿨는지 · 왜 회귀 은폐가 아닌지를 본문에 적는다. 두 commit 은 연달아 넣고, 검증은 2 뒤의 트리로 한다.
3. **1 과 2 사이에서 멈출 때** (escalation 으로 사용자 결정을 기다릴 때 등) — red HEAD 로 멈추지 않는다. `git branch wip/<slug>` 로 1 을 보관하고 branch 를 마지막 green commit 으로 되돌린 뒤 그 트리의 검증을 남긴다. 재개할 때 `wip/<slug>` 로 돌아가고 그 branch 를 지운다. (Claude Code 의 Stop hook 은 turn 끝 트리의 green verdict 를 요구하므로 이 경우 turn 을 끝낼 수 없다.)
4. **squash 머지** (사용자 지시일 때만) — 2 의 근거를 squash 본문에 옮겨 적는다. 지키는 것은 commit 개수가 아니라 단언 변경의 감사 가능성이다.

### PR 전 리뷰와 정리

- PR 을 열기 전에 [Inferential review 트리거](../agent_docs/modification-guide.md#inferential-review-트리거-llm-as-judge-수동-trigger) 표에서 이번 변경이 어느 review (code · security · 재사용·단순화) 에 걸리는지 보고 그 review 를 받는다. 목록은 그 표에만 있다.
- merge 된 로컬 branch 는 `git branch -d <branch>`, stale remote-tracking ref 는 `git fetch --prune`. 원격 branch 는 merge 확인 후에만 지운다.
