# ur5e_bt_coordinator 테스트 픽스처 분리 — DDS-free inject tier (issue #154, Direction 3)

## Goal

`ur5e_bt_coordinator` Tier-2 테스트의 잔존 FastDDS-churn flakiness (~1–2 intermittent fail/run) 를 제거한다. 이슈 #154 owner 코멘트의 Direction 3 (test-only state-inject seam) 채택: 캐시 상태만 필요한 6개 바이너리를 DDS 경로에서 떼어내고, service/rebind 경로 자체가 테스트 대상인 4개만 real-DDS e2e tier 로 남긴다.

## Acceptance criteria (Sprint Contract, 2026-07-15 사용자 컨펌)

1. 기존 assertion 전량 보존 — 테스트 diff 에 `EXPECT`/`ASSERT` 수정·삭제 0건 (E-6)
2. Release 빌드에서 `ctest --test-dir build/ur5e_bt_coordinator` **10회 연속 green** (ws-root 에서 실행, CLAUDE.md §9.1)
3. `bt_ros_bridge` 리팩터는 동작 중립 — subscription 이 추출된 핸들러를 그대로 호출, public API/ABI 무변경 (`rtc_msgs` 무관 → E-3 아님)

## Out of scope

- e2e 4개 바이너리의 executor 교체 (이슈 Direction 4) — 분리 후에도 그 바이너리에서 flake 가 남을 때만 **별도 후속 이슈**로
- BT 노드·브릿지의 production 동작 변경, QoS 변경 (ARCH-6 유지)
- `rtc_msgs` / `shape_estimation_msgs` ABI (E-3)

## Current state

- **Phase 1–4 구현 완료** (2026-07-15, branch `fix/issue-154-test-fixture-split`):
  - Phase 1 (`e68d028`): 콜백 람다 10개 → `On*` private 핸들러 추출 + `BridgeStateInjector` friend 선언. 22/22 통과.
  - Phase 2+3 (`f50e3fb`): `test/inject_fixture.hpp` 신설 (injector + `InjectTestFixture`), 6개 바이너리 base-class 교체 (assertion diff 0건 — `git diff -U0 | grep 'EXPECT\|ASSERT'` 빈 출력 확인), `test_hand_nodes.cpp` raw `PublishUntilObserved` 1곳 injector 치환, CMake `TIER2_INJECT`(6)/`TIER2_E2E`(4) 분리, `test_helpers.hpp` 미사용 helper/publisher prune.
  - Phase 4: Release 10회 연속 ctest **10/10 green** (아래 Evidence).
- Phase 5 진행 중: testing-debug.md sensor matrix 행 + 패키지 README 테스트 절 갱신 완료. `/code-review` (8-angle find → 1-vote verify) 완료 — CONFIRMED 4·PLAUSIBLE 2 전부 반영: `<map>` 직접 include (test_condition_nodes.cpp + bt_ros_bridge.hpp), `assm_v1_joint_names` 2중 복제 → `test/hand_joint_names.hpp` 공용화, inject 쪽 미사용 `TickUntilComplete` 사본 삭제, injector `bridge` 멤버 네이밍 (struct public), `Spin(dur)` 실제 sleep 화, `kRosContextEnv` inline→static. REFUTED 4 (Env shutdown 소유권 / RPY hand-roll 이동코드 / 10-핸들러 mirror 완전성 by-design / friend 폭 D2·D4 확정사항). 남은 것: PR (`Closes #154`) → merge 후 auto-memory prune + artifact `completed/` 이동.
- 이슈 #154 착수 코멘트 (구현 세부 3가지) — 미게시 (Phase 0 잔여분, PR 게시로 갈음 가능 여부는 사용자 판단).

## Next action

1. Branch 생성: `fix/issue-154-test-fixture-split` (main @ 0059712 기준)
2. **Phase 1** — `src/bt_ros_bridge.cpp` 의 subscription 콜백 람다 10개를 private 멤버 핸들러로 추출:
   `OnArmJointState` / `OnHandJointState` / `OnWorldTarget` / `OnActiveController` / `OnEstop` / `OnShapeEstimate` / `OnGraspState` / `OnWbcState` / `OnTransforms` / `OnToFSnapshot`.
   람다는 핸들러 1줄 호출로 축소. `OnActiveController` 는 기존 순서 유지 필수: `RewireControllerTopics(name)` **먼저**, `active_controller_` 세팅 나중 (bt_ros_bridge.cpp 콜백 주석의 client-bound 보장).
   헤더 (`include/ur5e_bt_coordinator/bt_ros_bridge.hpp`) 에 핸들러 선언 + `friend struct rtc_bt::test::BridgeStateInjector;` (forward-declare 필요: `namespace rtc_bt::test { struct BridgeStateInjector; }`).
   검증: 빌드 + 기존 전체 테스트 무변경 통과 → commit `refactor(ur5e_bt_coordinator): extract bridge subscription handlers for test seam (#154)`
3. **Phase 2** — `test/inject_fixture.hpp` 신설:
   - `BridgeStateInjector` (`namespace rtc_bt::test`): 각 `On*` 핸들러로 메시지를 만들어 포워딩하는 thin friend. 필드 직접 조작 금지 — 핸들러 경유만 (아래 Decisions D2).
   - `InjectTestFixture`: bridge 노드 + `BtRosBridge` + injector 만. mock controller 3개·픽스처 publisher 8개·background spin thread·`PublishUntilObserved` 재시도 루프 전부 없음. `rclcpp::init` 은 `::testing::Environment` 로 1회/binary 공식화.
   - 기존 helper 와 **동일 이름·시그니처** 제공 (`PublishArmState`, `PublishHandState`, `PublishGraspState`, `PublishWorldTarget`, `PublishEstop`, `PublishShapeEstimate`, `SetActiveAlias`, `Spin`) → 테스트 본문 무변경으로 base class 만 교체. (구현 보정: `TickUntilComplete` 는 inject 6개가 미사용이라 사본 제공 안 함 — e2e 원본만 유지; `Spin` 은 no-op 대신 요청 duration 만 실제 sleep — cross-tier wall-clock 의미 보존, /code-review finding.)
   - `SetActiveAlias` → injector → `OnActiveController` → 실제 `RewireControllerTopics` 호출 (Decisions D3 함정 1).
4. **Phase 2 계속** — 6개 바이너리 전환: `test_data_nodes`, `test_condition_nodes`, `test_shape_nodes`, `test_hand_nodes`, `test_move_to_joints`, `test_move_to_pose`. include + base class 교체 외 본문 무변경. 예외 1곳: `test/test_hand_nodes.cpp:208` 의 raw `PublishUntilObserved` (11-joint OOB 케이스) → injector 직접 호출로 치환 (assertion 은 그대로).
5. **Phase 3** — `CMakeLists.txt` `TIER2_TESTS` → `TIER2_INJECT`(6) / `TIER2_E2E`(4: `test_set_gains`, `test_grasp_control`, `test_switch_controller`, `test_service_singlethread`) 분리. `test/test_helpers.hpp` 에서 inject 전환 후 미사용이 된 helper prune (e2e 4개가 아직 쓰는 것은 유지).
   → commit `test(ur5e_bt_coordinator): split DDS-free inject fixture from e2e tier (#154)`
6. **Phase 4** — acceptance: Release 빌드 → 10회 연속 ctest green → 결과를 이 파일 Evidence 절에 기록. assertion parity diff 확인.
7. **Phase 5** — 종결: docs 동기화 (`agent_docs/testing-debug.md` sensor matrix 의 ur5e_bt_coordinator 행에 inject/e2e tier 구분 반영, 패키지 `README.md` 테스트 절 — PROC-1). `/code-review` (다파일·다패키지 트리거, CLAUDE.md §5.5) → PR (body 에 10-run evidence + `Closes #154`). Merge 후: auto-memory `reference_ur5e_bt_coordinator_test_flaky_known.md` prune (+MEMORY.md 인덱스 라인), 이 artifact → `completed/` 이동.

## Decisions and rationale

- **D1 — Direction 3 채택, 1/2/4 기각** (이슈 코멘트 2026-07-14 분석 그대로): 1(global Environment)은 사실상 적용됨·cosmetic, 2(ROS_DOMAIN_ID)는 init 후 변경 불가 + node churn 잔존, 4(MultiThreadedExecutor)는 원인 아님. 3만 대부분의 테스트를 DDS 경로에서 제거.
- **D2 — seam 은 "필드 찌르기 friend" 가 아니라 "콜백 body 를 핸들러로 추출 + friend 가 핸들러 호출"**: inject 경로 == production 콜백 경로가 영구 보장 (drift 불가), 동일 mutex·동일 파싱 코드라 E-6 을 구조적으로 만족. health 스탬프 (`health_mutex_`) 도 자동 보존 — `test_data_nodes.cpp:118` `GetTopicHealthReflectsReceiptAndStaleness` 가 핸들러의 health 쓰기에 의존하므로 필드-직접-주입이었다면 깨졌음 (함정 2).
- **D3 — inject 픽스처도 `RewireControllerTopics` 실호출 필수** (함정 1): `PublishArmTarget`/`PublishHandTarget` (bt_ros_bridge.cpp:269-295) 이 `arm_target_pub_`/`hand_target_pub_` 를 null 체크 없이 dereference 하는데 이 pub 들은 rewire 에서만 생성됨. `move_to_*`/`hand_nodes` 가 tick 중 target publish → 미호출 시 segfault. publisher/client *생성*은 동기·discovery 무관이라 DDS-free 목표와 충돌 없음 (구독자 없는 publish 는 no-op).
- **D4 — `#ifdef <PKG>_TESTING` 방식 기각**: `bt_nodes` 라이브러리가 production 타겟과 test 타겟에 공유 링크 (CMakeLists.txt) — ifdef 는 라이브러리 빌드를 갈라야 함. friend 선언은 런타임 비용 0, ABI 불변.
- **D5 — helper 이름 유지** (`Publish*` 를 inject 픽스처에서도 그대로): 테스트 본문 diff 를 base-class 교체로 최소화 → assertion parity 리뷰가 자명해짐. 의미상 rename (`Inject*`) 은 하지 않음.
- **D6 — e2e 4개는 현행 `RosTestFixture` 유지**: 4개만 남으면 per-binary node churn 이 latency 임계 아래로 내려간다는 가설 (이슈 코멘트) — Phase 4 에서 검증, 실패 시 Out of scope 의 후속 이슈.

## Evidence

- 재현 기준선 (이슈 본문, 2026-07-14): Release 빌드 후 5× ctest → 0–2 fail/run, 실패 집합 가변 (`IsGraspPhase_AllPhases` timeout, `MoveToJointsTest.WithinToleranceBoundary`/`PoseNameLookup` ~1/4 in-binary flake — 단독 `--gtest_filter` 로는 통과).
- 재현 명령 (ws-root 에서, §9.1):
  ```
  source src/rtc-framework/repo_scripts/scripts/setup_env.sh
  colcon build --packages-select ur5e_bt_coordinator --cmake-args -DCMAKE_BUILD_TYPE=Release
  for i in $(seq 10); do ctest --test-dir build/ur5e_bt_coordinator; done
  ```
- 구현 후 10-run 결과 (2026-07-15, Release, ws-root + setup_env.sh, branch @ `f50e3fb`):
  **run 1–10 전부 "100% tests passed, 0 tests failed out of 22"** — 10/10 연속 green (acceptance 2 충족).
  부수 관찰: 22-test 전체 suite 시간 15.6s → 8.2s (inject tier 가 delivery-wait 제거).

## Failed approaches

- 이전 point-fix 라운드 (commit f6cc942, 22a4269 — 2026-07 초): deliver-before-tick race·rebind race·SpinUntil 제거·context 1회/binary 로 13 deterministic → 0–2 intermittent 까지 줄였으나, **현재 픽스처 형태 안에서의 point-fix 로는 잔존분이 안 닫힘** (이슈 본문 결론). 같은 축의 추가 point-fix (sleep 상향, 재시도 강화) 는 재시도하지 말 것.

## Constraints / pending human decisions

- E-6: assertion 약화 절대 금지 — acceptance 1항이 그 검증.
- PROC-1: CMake/README/testing-debug.md 동기화는 Phase 3·5 에 포함됨.
- §9.1: colcon/ctest 는 반드시 ws-root (`~/ros2_ws/rtc_ws`) 에서 + `setup_env.sh` source. §9.2: venv 우회 금지.
- Pending: 없음 (Sprint Contract 컨펌 완료). e2e 잔존 flake 발견 시에만 후속 이슈 여부를 사용자와 재논의.

## Workspace

- Branch: `fix/issue-154-test-fixture-split` (main @ `0059712` 기준) — Phase 1 `e68d028`, Phase 2+3 `f50e3fb`, docs+artifact 커밋 후속
- `git status`: untracked `docs/WBC_CONTROLLER_IMPLEMENTATION.md` 1건 — **이 task 와 무관, 사용자 소유, 건드리지 말 것**

## Pointers

- Issue: https://github.com/hyujun/rtc-framework/issues/154 (owner 분석 코멘트 2026-07-14 포함)
- Refs: ec89e93 (depth-1 QoS, ARCH-6), f6cc942, 22a4269 (선행 point-fix)
- `ur5e_bt_coordinator/src/bt_ros_bridge.cpp` — 추출 대상 콜백: 생성자 65-166 (arm/hand joint, world_target, active_ctrl, estop, shape), `RewireControllerTopics` 576-733 (transforms 615, grasp_state 622, tof 667, wbc_state 679; target pub 711-714)
- `ur5e_bt_coordinator/src/bt_ros_bridge.cpp:269-295` — `PublishArmTarget`/`PublishHandTarget` null-deref (D3 근거)
- `ur5e_bt_coordinator/include/ur5e_bt_coordinator/bt_ros_bridge.hpp` — friend 선언·핸들러 선언 추가 위치 (cached state 322-338, health 340-359)
- `ur5e_bt_coordinator/test/test_helpers.hpp` — 현행 `RosTestFixture` (e2e 4개가 계속 사용; helper 시그니처의 원본)
- `ur5e_bt_coordinator/test/test_hand_nodes.cpp:208` — raw `PublishUntilObserved` 치환 1곳
- `ur5e_bt_coordinator/test/test_data_nodes.cpp:118` — health 스탬프 의존 테스트 (D2 근거)
- `ur5e_bt_coordinator/CMakeLists.txt:142-153` — `TIER2_TESTS` 분리 지점
- `agent_docs/testing-debug.md` — Phase 5 sensor matrix 동기화 대상
- CLAUDE.md §6.5 (Sprint Contract), §9.1 (colcon CWD), agent_docs/invariants.md PROC-6/E-6
