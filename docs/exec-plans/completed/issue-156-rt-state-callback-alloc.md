# issue-156: integrated_bringup RT state callback 의 lazy reorder 동적 할당 제거

Issue: #156 — `fix(integrated_bringup): RT state callback의 lazy reorder 동적 할당 제거`

## Goal

세 `DeviceBackend` (`mujoco_native` / `ur_driver_native` / `udp_hand_native`) 의
`OnJointState` 를 allocation-free 로 만든다. 첫 named `JointState` 수신 시의
lazy reorder map 구축 (`std::vector<int> map(...)` + `std::move`) 이
`rt_callback` cb group (FIFO 70 executor) 에서 실행되어 invariants.md
§RT callback rule ("mailbox-only", 동적 할당 금지) 을 위반한다 — 이를
fixed-capacity storage + bounded loop 로 교체한다.

## 이슈 진단 검증 결과 (착수 전 grep — 이슈 본문은 가설로 취급)

- **위반 실재 확인**: 세 파일 모두 verbatim 동일한 ~20줄 lazy-build 블록 존재.
  - `integrated_bringup/src/backends/mujoco_native_backend.cpp:210-225`
  - `integrated_bringup/src/backends/ur_driver_native_backend.cpp:70-85`
  - `integrated_bringup/src/backends/udp_hand_native_backend.cpp:76-91`
- **RT path 여부 확인**: state sub 는 `SubscriptionOptions.callback_group = state_cb_group`
  으로 생성 (`ur_driver_native_backend.cpp:34-43` 주석: "rt_callback callback group (FIFO 70)").
  invariants.md §RT path 정의의 "SCHED_FIFO dedicated-core 로 실행되는 모든 thread" 에 해당.
- **위반 범위는 이슈보다 좁음**: 같은 rt_callback lane 의 `OnWrench` (mujoco),
  `OnMotorState` / `OnSensorState` (udp_hand) 는 이미 fixed-size + SeqLock 만 사용 — 대상 아님.
- **이슈 본문의 부정확 1건**: "CM's BuildDeviceReorderMap" 주석이 참조하는 함수는
  현재 `rtc_controller_manager` 에 존재하지 않음 (stale 주석). 이번 수정에서 주석 갱신.
- **기반 존재**: `rtc::kMaxDeviceChannels = 64` (`rtc_base/types/types.hpp:44`),
  `DeviceStateCache` 는 이미 fixed array. 소비 루프의 dest 인덱스는 이미
  `idx < kMaxDeviceChannels` 로 가드됨.
- **P5 검색**: `rtc_base`/`rtc_controller_manager` 에 기존 reorder utility 없음 — 신규 helper 필요.

## Acceptance criteria (= Sprint Contract)

1. 세 backend `OnJointState` 경로에서 `std::vector` / `new` / `resize` /
   `push_back` / `assign` 제거 — `grep -n "std::vector" integrated_bringup/src/backends/*.cpp`
   의 잔존 hit 는 비-RT `Configure` 경로 (`cmd_msg_.data.assign`, `wrench_subs_`) 만.
   세 헤더의 `std::vector<int> state_reorder_` 멤버도 fixed-capacity 로 교체.
2. named(순서 뒤섞임 포함) / un-named / unknown-joint / capacity 초과 JointState 에서
   기존 reorder 동작이 유지됨을 신규 unit test 가 assert 하고, helper 호출 구간의
   allocation count 0 을 operator-new 카운터 test 가 assert 한다.
3. `integrated_bringup` 빌드 + `colcon test --packages-select integrated_bringup` 전체 green
   (기존 `test_device_backends_registered` / `test_mujoco_native_backend` /
   `test_controller_target_cb_group_invariant` 포함).

## Out of scope (이슈 명시)

- test fixture 의 `../robot_descriptions/.../panda.urdf` source-tree 경로 의존
  (참고: rtc_mpc/rtc_tsid 의 동일 패턴은 의도적 configure-time fixture 로 판정된 전례 있음 — 재-flag 금지)
- README / package description 의 iiwa7+LEAP·DemoWbc 반영 부족
- WBC MPC config fallback 의 `ur5e_p1a` 기본 경로
- CM / rclcpp executor 레벨의 메시지 take 시 할당 (프레임워크 소관)

## 설계

### 1. 공통 helper (신규 header-only)

`integrated_bringup/include/integrated_bringup/backends/joint_state_reorder.hpp`

```cpp
struct JointStateReorder {
  std::array<int, rtc::kMaxDeviceChannels> map{};  // msg idx -> device slot (-1 = unknown)
  int size{0};                                     // 0 => identity direct-copy
};

// 첫 named msg 에서 one-shot 호출. bounded loop 만 사용, 할당 0
// (std::string operator== 비교만 — 할당 없음). msg name 수는
// kMaxDeviceChannels 로 clamp.
void BuildJointStateReorder(const std::vector<std::string>& msg_names,
                            const std::vector<std::string>& ref_names,
                            JointStateReorder& out) noexcept;

// mailbox write: reorder(또는 size==0 시 identity) 로 pos/vel/eff 를
// DeviceStateCache 에 복사. num_channels 는 kMaxDeviceChannels 로 clamp.
void WriteJointStateToCache(const sensor_msgs::msg::JointState& msg,
                            const JointStateReorder& reorder,
                            rtc::DeviceStateCache& ds) noexcept;
```

배치 근거: 세 사용처가 전부 `integrated_bringup/src/backends/` 이므로 로컬 helper.
4번째 외부 backend 패키지가 생기면 그때 `rtc_controller_manager`
(`device_state_cache.hpp` 옆) 로 승격 (P5 — 재사용 시점에 일반화).
`rtc_*` 승격을 지금 하지 않는 이유: PROC-3 전체 downstream 재검증 + §5.5
code-review trigger 비용 대비 현재 소비자가 한 패키지뿐.

### 2. 세 backend 교체 (동작 등가)

- 헤더: `std::vector<int> state_reorder_` → `JointStateReorder state_reorder_`.
  `std::atomic<bool> state_reorder_built_` 는 유지 (acquire/release 시맨틱 불변 —
  MutuallyExclusive cb group 이라 map 은 callback thread 단독 접근, 기존과 동일).
- `OnJointState`: lazy-build 블록 → `BuildJointStateReorder(...)`,
  복사 블록 → `WriteJointStateToCache(...)`. SeqLock `Load`/`Store`,
  `last_state_ns_` steady_clock 갱신, `NotifyStateReady()` 는 그대로.
- stale 주석 "CM's BuildDeviceReorderMap" / "CM's DeviceJointStateCallback" 갱신.

동작 등가성 매트릭스 (기존 → 신규):

| 입력 | 기존 | 신규 |
|---|---|---|
| named, 순서 뒤섞임 | map 경유 reorder | 동일 (`size>0`) |
| un-named (name 비어있음) | build skip → identity copy | 동일 (`size==0`) |
| named 이지만 `ref_names` 비어있음 | vector 비어있음 → identity | 동일 (`size==0`) |
| unknown joint name | map entry -1 → skip | 동일 |
| msg joint 수 > 64 | map 은 full-size, dest 가드로 보호 | **msg idx ≥64 는 drop (clamp)** — 이슈 완료기준이 명시 허용한 유일한 동작 변화 |
| `num_channels` | raw count (unclamped) | kMaxDeviceChannels clamp (소비자 OOB 방어) — 구현 시 소비자 grep 재확인 |

### 3. 테스트

- **신규 `integrated_bringup/test/test_joint_state_reorder.cpp`**:
  위 매트릭스 6 케이스 + allocation-guard (TU-local global `operator new/delete`
  카운터로 `Build`+`Write` 호출 전후 delta == 0 assert; new/delete 짝 유지 —
  GCC mismatched-new-delete false-positive 회피).
- **`test_mujoco_native_backend.cpp` 확장**: 기존 fixture 로 named/뒤섞인
  JointState 를 실제 topic publish → `ReadState` 가 device-order 로 나오는지
  end-to-end 1 케이스 (세 backend 가 같은 helper 에 위임하므로 대표 1개면 충분).
- CMakeLists.txt 에 신규 test 등록 (PROC-1).

### 4. 재발 방지 static check — 사용자 결정 필요 (Warning)

`.claude/rules/rt-path.md` 의 path glob 이 `rtc_*/**` 만 커버해서
`integrated_bringup` backend (실제 RT path) 편집 시 rule 이 로드되지 않음 —
이번 위반이 유입된 구조적 원인으로 추정. 옵션:
- (권고) glob 에 `integrated_bringup/**/*.cpp|hpp` 추가 — 비-RT 코드 편집 시에도
  reminder 가 뜨지만 rule 본문이 이미 "비-RT 는 면제" 를 명시하므로 비용 낮음
- (대안) allocation-guard unit test 만으로 재발 방지 (이미 §3 에 포함) — harness 무변경

## Next action

**완료 (2026-07-15).** Sprint Contract 컨펌 + §4 권고안(glob 확장) 승인 후 구현·검증 종결.

- 구현 commit: `b6f770b` (helper + 세 backend + tests + CMake), `8f1faf6` (rt-path glob 확장 + CLAUDE.md §3 동기화)
- Sprint 기준 3/3 충족: (1) grep 잔존 hit 는 비-RT `Configure` 경로만 (`cmd_msg_` presize, wrench param/sub 생성) (2) 신규 unit test 8 케이스 + mujoco e2e 1 케이스 green (3) `colcon test --packages-select integrated_bringup` 230 tests 0 failures

## Decisions and rationale

- **helper 를 integrated_bringup 로컬로**: 위 §설계 1 배치 근거 참조.
- **`state_reorder_built_` atomic 유지**: 동시성 시맨틱 변경 없이 diff 최소화.
- **fixed array 는 `std::array<int, kMaxDeviceChannels>` + `int size`**: 이슈 제안
  그대로. `size==0` 이 기존 `vector::empty()` 분기를 1:1 대체.
- **allocation-guard 를 helper 단위 unit test 로**: `OnJointState` 는 private 이고
  executor 경유 호출은 rclcpp 자체 할당과 분리 불가 — helper 를 직접 호출해야
  카운터가 결정적.

## Evidence

착수 전 검증:

- `grep -rn "state_reorder_" integrated_bringup/` — 세 backend 3쌍(헤더 멤버 + cpp 사용) 외 없음
- `grep -rn "constexpr.*kMaxDeviceChannels"` → `rtc_base/types/types.hpp:44` (= 64)
- `grep -rn "reorder" rtc_controller_manager/src/` → hit 0 (BuildDeviceReorderMap 부재 확인)
- `udp_hand_native_backend.cpp` `OnMotorState`/`OnSensorState`, `mujoco_native_backend.cpp`
  `OnWrench` 직접 열람 — fixed-size + SeqLock/throttled-WARN 만, 추가 위반 없음

구현 후 검증 (ws root, setup_env source — §9.1/§9.2 준수):

- `colcon build --packages-select integrated_bringup` — 성공
- `colcon test --packages-select integrated_bringup` — **230 tests, 0 errors, 0 failures**
- allocation-guard test: `Build`+`Write` 호출 구간 operator-new delta **0** 실측
- `num_channels` clamp 확정 근거: `rt_controller_node_rt_loop.cpp:90-94` 가
  `copy_n(cache.positions.data(), num_channels, ...)` 로 unclamped 사용 — 64 초과
  wire 값이 cache 를 벗어나면 OOB. clamp 는 방어이자 이슈 완료기준 3 충족.

## Failed approaches

- **allocation-guard test 첫 작성본의 기대값 스왑** — 역순 named msg 에서
  `positions[0]` 은 j0 값(0.0)이어야 하는데 j5 값(5.0)을 기대. helper 는 정상,
  테스트 버그. identity copy 와 구분되는 값 배치는 유지한 채 기대값만 정정.
- **e2e 테스트에서 `auto pub = LifecycleNode::create_publisher(...)`** —
  `LifecyclePublisher` 가 반환되어 `on_activate()` 전 publish 전량 drop
  ("state message never delivered"). `pub->on_activate()` 추가로 해결.
  (기존 fixture 의 wrench pub 는 base-pointer 타입 선언이라 gate 우회로 동작 —
  auto-memory `feedback_lifecycle_publisher_gate_bypass` 의 두 경로와 일치.)

## Constraints / pending human decisions

없음 — Sprint Contract·§4 glob 확장 모두 사용자 승인 완료 (2026-07-15).

## Workspace

- 구현 branch: `fix/issue-156-rt-state-callback-alloc` (`a5916fa` plan pin →
  `b6f770b` fix → `8f1faf6` harness) → `main` merge 후 branch prune

## Pointers

- Issue #156
- `integrated_bringup/src/backends/mujoco_native_backend.cpp:201-266` (`OnJointState`)
- `integrated_bringup/src/backends/ur_driver_native_backend.cpp:61-126`
- `integrated_bringup/src/backends/udp_hand_native_backend.cpp:69-131`
- `integrated_bringup/include/integrated_bringup/backends/{mujoco,ur_driver,udp_hand}_native_backend.hpp` (멤버 선언)
- `rtc_base/include/rtc_base/types/types.hpp:44` (`kMaxDeviceChannels`)
- `rtc_controller_manager/include/rtc_controller_manager/device_state_cache.hpp` (`DeviceStateCache`)
- `agent_docs/invariants.md` §RT callback rule (mailbox-only)
- `.claude/rules/rt-path.md` (path glob — §4 결정 대상)
- `integrated_bringup/test/test_mujoco_native_backend.cpp` (fixture 재사용)
- 선행 유사 close-out: `docs/exec-plans/completed/issue-160-e2e-clock-split.md`
