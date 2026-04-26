# rtc_cm Lifecycle & Switch Service — Archive

**Status: complete (Phase 0 → 6 done, 2026-04-26).** Live spec moved to README + controllers.md.

## What shipped

- `/rtc_cm/switch_controller` (`rtc_msgs/srv/SwitchController`) — sync, single-active (D-A1), STRICT default, BEST_EFFORT trims to first activate target. E-STOP active → `ok=false, message="E-STOP active"` (D-B6). Pure-deactivate rejected (no replacement). `timeout` field accepted but informational — sync helper bounded by `sleep_for(1.5 × dt)` + controller hooks (~ms).
- `/rtc_cm/list_controllers` (`rtc_msgs/srv/ListControllers`) — empty request, response = `ControllerState[]` (name/state/type/is_active/claimed_groups). Order matches `controllers_` insertion.
- Per-controller lifecycle state: `RtControllerNode::controller_states_` (`vector<atomic<int>>`, 0=Inactive 1=Active). RT loop dispatch unchanged (still `active_controller_idx_.load(acquire)`).
- `RTControllerInterface::on_activate(prev_state, ControllerState const& device_snapshot)` — D-A5 hold-init responsibility moved into controller. Base impl calls `InitializeHoldPosition(snapshot)` when `snapshot.num_devices > 0`; empty snapshot (sensor not received) skips cleanly.
- `rtc::mpc::MPCThread::Pause() / Resume() / Paused()` — cv-based, idempotent, RequestStop wakes paused thread (D-OQ6).
- BT `SwitchController` 노드: srv only. fire-and-forget publish + `/active_controller_name` polling 모두 제거.
- Removed: `/<robot_ns>/controller_type` String topic (CM sub + BT pub), `BtRosBridge::PublishSelectController`, `RtControllerNode::controller_selector_sub_`, `SwitchController` BT 노드의 `use_service` port.
- Kept: `/<robot_ns>/active_controller_name` latched topic (D-A6) — shape_estimation, digital_twin, BT bridge가 controller-owned topic rewire trigger로 사용.

## Commits

| Phase | 내용 | Commit |
|---|---|---|
| 0 | Design lockdown | `9aa830f` |
| 1 | rtc_msgs schema (ControllerState msg + ListControllers/SwitchController srv) | `f5d3204` |
| 1.5 | DemoWbc idempotent (P-1) + MPCThread Pause/Resume (P-2) | `e3d2c70` |
| 2 | CM controller-level lifecycle state | `b4b31dc` (+ docs `3cfea24`) |
| 3 | `/rtc_cm/...` srv 도입 | `3783730` |
| 4 | BT `SwitchController` srv 마이그레이션 | `e186d6b` |
| 5 | `/<robot_ns>/controller_type` legacy 제거 | `55b10f5` |
| 6 | 문서 sync + plan archive | this commit |

## Closed decisions

D-A1 single-active, D-A2 `/rtc_cm/...` global namespace (single-CM 가정), D-A3 rtc_msgs 자체 srv, D-A4 sync srv, D-A5 hold-init은 controller `on_activate(prev, snapshot)`, D-A6 legacy topic 병행 후 제거 (active_controller_name latched 영구 유지), D-A7 4-state ros2_control 호환, D-B6 E-STOP 중 거부, D-OQ6 MPC thread Pause/Resume, OQ-1 = `vector<atomic<int>>` (4-state 미사용 + bitmask 부적합), OQ-2 = `sleep_for(1.5×dt)` (F-3 race benign).

## Where the spec lives now

- Switch flow + srv 인터페이스: [`rtc_controller_manager/README.md`](../rtc_controller_manager/README.md#런타임-전환-rtc_cmswitch_controller-서비스-sync) §"런타임 전환" + §"고정 서비스"
- Controller / topic 표: [`agent_docs/controllers.md`](controllers.md#ros2-topics) §"ROS2 Topics" — Controller Manager 행
- Lifecycle hook 시그니처: [`rtc_controller_interface/README.md`](../rtc_controller_interface/README.md)
- Test 매트릭스: [`agent_docs/testing-debug.md`](testing-debug.md) (rtc_controller_manager 35: lifecycle 9 + switch_service 9 + timing_profiler 17; ur5e_bt_coordinator switch_controller 7 = 4 base + 3 srv)

## Retrospective notes

- `rclcpp::QoS srv_qos(rclcpp::ServicesQoS());` 는 most-vexing-parse — `const rmw_qos_profile_t srv_qos = rmw_qos_profile_services_default;` 사용
- `create_service<>()` 4-arg signature `(name, callback, qos, callback_group)` — `callback_group` 안 넘기면 default group으로 들어감 → 반드시 `cb_group_aux_` 명시
- BT srv 테스트는 `MultiThreadedExecutor` 가 add_node 충돌 발생 → background `spin_some` 루프 사용 + 부모 fixture의 `Spin()` 을 sleep no-op로 shadow
- Friend bridge (`ControllerLifecycleTestAccess`) 같은 namespace(rtc::)지만 별도 TU에 정의해도 두 test 바이너리가 독립적으로 link됨
