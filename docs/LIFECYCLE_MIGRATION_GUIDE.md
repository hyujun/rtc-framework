# Lifecycle Node Migration — Implementation Guide

> **상태: 완료** — 모든 5개 C++ 노드 + 6개 Launch 파일 마이그레이션 완료 (2026-04-16).
>
> 이 문서는 ROS2 `rclcpp::Node` → `rclcpp_lifecycle::LifecycleNode` 마이그레이션 구현 가이드입니다.
> 계획 파일: `/home/junho/.claude/plans/composed-jingling-lake.md`

---

## 1. 프로젝트 개요

**RTC Framework** — URDF 기반 매니퓰레이터를 위한 로봇 비의존적 실시간 제어 프레임워크.
- C++20, ROS2 Humble/Jazzy, 500Hz RT 루프, lock-free SPSC, SeqLock
- 13개 `rtc_*` (로봇 비의존) + 4개 `ur5e_*` (로봇 특화) 패키지

---

## 2. 변환 대상 (C++ 노드 5개)

| # | Node Class | Package | Source Files | Complexity |
|---|-----------|---------|-------------|------------|
| 1 | `HandUdpNode` | `ur5e_hand_driver` | `src/hand_udp_node.cpp` (class+main 단일 파일) | Low |
| 2 | `ShapeEstimationNode` | `shape_estimation` | `include/.../shape_estimation_node.hpp` + `src/shape_estimation_node.cpp` + `src/shape_estimation_node_main.cpp` | Medium |
| 3 | `BtCoordinatorNode` | `ur5e_bt_coordinator` | `include/.../bt_coordinator_node.hpp` + `src/bt_coordinator_node.cpp` + `src/main.cpp` | Medium |
| 4 | `MuJoCoSimulatorNode` | `rtc_mujoco_sim` | `src/mujoco_simulator_node.cpp` (class+main 단일 파일) | Medium |
| 5 | `RtControllerNode` | `rtc_controller_manager` | `include/.../rt_controller_node.hpp` + 8개 src 파일 + `src/rt_controller_main_impl.cpp` | **High** |

**수정 불필요 패키지:**
- `rtc_controller_interface` — `rclcpp` 의존성 없음. 순수 Strategy 패턴 (`RTControllerInterface`, `ControllerRegistry`). 노드 타입과 완전히 분리.
- `rtc_controllers` — `RTControllerInterface`만 구현. ROS2 노드 참조 없음.
- Python 노드(`rtc_digital_twin`) — 변환 범위에서 제외 (사용자 결정).

---

## 3. 핵심 설계 원칙

### ROS2 Lifecycle 설계 철학 (design.ros2.org)

1. **Constructor는 비어있어야 한다** — 노드 인스턴스화만, 리소스 할당 없음
2. **2-tier 리소스 모델:**
   - **Tier 1 (on_configure):** Publishers, subscribers, 버퍼, 디바이스 핸들, 파라미터 로딩
   - **Tier 2 (on_activate):** 하드웨어 활성화, 스레드 시작. **"significant time이 소요되는 준비 작업은 하지 않는다"**
3. **on_cleanup = on_configure의 정확한 역순** — 모든 `create_*` → `.reset()` 대응
4. **on_shutdown은 어떤 primary state에서든 올 수 있음** — Unconfigured, Inactive, Active 모두
5. **부모 클래스 callback 체이닝 필수** — `LifecycleNode::on_activate(state)` 호출 누락 시 LifecyclePublisher 미활성화 (Nav2 #3710 버그)
6. **Managed timer는 없다** — timer callback에서 state guard 필요
7. **on_error 기본 FAILURE → Finalized (복구 불가)** — SUCCESS 반환으로 Unconfigured 복귀

### 설계 결정

| 결정 | 내용 |
|------|------|
| **Publisher 전략** | 데이터 publisher → `LifecyclePublisher`. E-STOP publisher → `rclcpp::create_publisher` standalone function으로 일반 Publisher 유지 |
| **Lifecycle 전환** | Launch 파일 event handler (`LifecycleNode` action + `OnStateTransition`) |
| **RT 루프** | `on_activate`에서 시작, `on_deactivate`에서 중지 (ros2_control과 다름 — jthread 기반이므로 적절) |
| **BtRosBridge** | `rclcpp::Node::SharedPtr` → `rclcpp_lifecycle::LifecycleNode::SharedPtr` (3곳) |
| **RtControllerMain** | Phase1: lifecycle executor → Phase2: Active 대기 → Phase3: 전용 executor(sensor/log/aux) 전환 |

---

## 4. 현재 코드 구조 (변환 전)

### 4.1 HandUdpNode (`ur5e_hand_driver/src/hand_udp_node.cpp`)

```
line 49: class HandUdpNode : public rclcpp::Node {
line 51:   HandUdpNode() : Node("hand_udp_node") {
            // Constructor에서 모든 작업:
            // - declare_parameter ~30개 (line 53-105)
            // - HandController 생성 (line 168-174)
            // - publisher 생성 (line 206-225): joint_state_pub_, motor_state_pub_,
            //   sensor_state_pub_, sensor_monitor_pub_, link_status_pub_
            // - subscriber 생성 (line 270-312): joint_command_sub_, calib_cmd_sub_
            // - calib_status_pub_ + calib_status_timer_ (line 317-325)
            // - PreallocateMessages() (line 249)
            // - EventLoop callback 설정 (line 252-256)
            // - controller_->Start() (line 331)
            // - fake_tick_timer_ (line 346-369)
            // - FailureDetector (line 372-400)
          }
line 418: ~HandUdpNode() { SaveCommStats(); failure_detector_->Stop(); controller_->Stop(); }

// main (line 끝):
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandUdpNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

**Publisher 타입 (현재):**
- `joint_state_pub_`: `rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr` (BEST_EFFORT, depth 1)
- `motor_state_pub_`: 동일
- `sensor_state_pub_`: `rclcpp::Publisher<rtc_msgs::msg::HandSensorState>::SharedPtr` (BEST_EFFORT, depth 1)
- `sensor_monitor_pub_`: `rclcpp::Publisher<rtc_msgs::msg::HandSensorState>::SharedPtr` (RELIABLE, depth 10)
- `link_status_pub_`: `rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr` (TRANSIENT_LOCAL, depth 1) — **일반 Publisher 유지**
- `calib_status_pub_`: `rclcpp::Publisher<rtc_msgs::msg::CalibrationStatus>::SharedPtr` (TRANSIENT_LOCAL)

### 4.2 ShapeEstimationNode

```
Header: shape_estimation/include/shape_estimation/shape_estimation_node.hpp
line 39: class ShapeEstimationNode : public rclcpp::Node {
line 44:   ShapeEstimationNode();  // constructor에서 모든 초기화

Source: shape_estimation/src/shape_estimation_node.cpp
// Constructor에서: DeclareParameters() → 알고리즘 객체 초기화 → 
//   callback group → subs/pubs/services/timers → InitExploration()

Main: shape_estimation/src/shape_estimation_node_main.cpp
// rclcpp::init → make_shared → rclcpp::spin → rclcpp::shutdown

내부 상태 머신: enum class State { kStopped, kRunning, kPaused, kSingleShot };
Action server: rclcpp_action::Server<ExploreShape>
```

### 4.3 BtCoordinatorNode

```
Header: ur5e_bt_coordinator/include/ur5e_bt_coordinator/bt_coordinator_node.hpp
line 39: class BtCoordinatorNode : public rclcpp::Node {
line 41:   explicit BtCoordinatorNode(const rclcpp::NodeOptions& options);
line 44:   void Initialize();  // ★ two-phase init (shared_from_this 필요)

Source: ur5e_bt_coordinator/src/bt_coordinator_node.cpp
line 53-57: Constructor — DeclareParameters()만
line 59-128: Initialize() — bridge 생성, BT 노드 등록, 트리 로드, timer 생성

Main: ur5e_bt_coordinator/src/main.cpp
// make_shared → node->Initialize() → rclcpp::spin

BtRosBridge (변경 필요):
  Header: ur5e_bt_coordinator/include/ur5e_bt_coordinator/bt_ros_bridge.hpp
  line 46:  explicit BtRosBridge(rclcpp::Node::SharedPtr node);
  line 151: void LoadPoseOverrides(rclcpp::Node::SharedPtr node);
  line 174: rclcpp::Node::SharedPtr node_;
  Source: ur5e_bt_coordinator/src/bt_ros_bridge.cpp
  line 13:  BtRosBridge::BtRosBridge(rclcpp::Node::SharedPtr node)
  line 401: void BtRosBridge::LoadPoseOverrides(rclcpp::Node::SharedPtr node)
```

### 4.4 MuJoCoSimulatorNode (`rtc_mujoco_sim/src/mujoco_simulator_node.cpp`)

```
line 42: class MuJoCoSimulatorNode : public rclcpp::Node {
line 44-61: Constructor — DeclareAndLoadParameters → CreateSimulator → CreateGroupHandles → CreateTimers → sim_->Start()
line 63-65: Destructor — sim_->Stop()

★ 버그: line 489-490:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_status_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/sim/status", rclcpp::QoS(10));
  // 멤버 초기화 리스트에서 create_publisher 호출 — LifecycleNode에서 crash 유발
  // → on_configure로 이동 필수

Main: line 511-525 (같은 파일 하단)
// rclcpp::init → make_shared<MuJoCoSimulatorNode>() → rclcpp::spin → rclcpp::shutdown

GroupRosHandles (line 30-38):
  state_pub: rclcpp::Publisher<JointState>::SharedPtr
  sensor_pub: rclcpp::Publisher<SimSensorState>::SharedPtr
  cmd_sub: rclcpp::Subscription<JointCommand>::SharedPtr
  fake_timer: rclcpp::TimerBase::SharedPtr
```

### 4.5 RtControllerNode (`rtc_controller_manager`)

```
Header: rtc_controller_manager/include/rtc_controller_manager/rt_controller_node.hpp
line 53: class RtControllerNode : public rclcpp::Node

Source files (8개):
  src/rt_controller_node.cpp           — Constructor, Destructor, CreateCallbackGroups, CreateTimers
  src/rt_controller_node_params.cpp    — DeclareAndLoadParameters (컨트롤러 로딩)
  src/rt_controller_node_topics.cpp    — CreateSubscriptions, CreatePublishers, ExposeTopicParameters
  src/rt_controller_node_callbacks.cpp — DeviceJointStateCallback 등
  src/rt_controller_node_rt_loop.cpp   — RtLoopEntry, ControlLoop, CheckTimeouts
  src/rt_controller_node_publish.cpp   — PublishLoopEntry, DrainLog
  src/rt_controller_node_estop.cpp     — TriggerGlobalEstop, ClearGlobalEstop
  src/rt_controller_node_device_config.cpp — LoadDeviceNameConfigs, BuildDeviceReorderMap

Main entry: src/rt_controller_main_impl.cpp
  rtc::RtControllerMain() — mlockall → rclcpp::init → node 생성 →
    StartRtLoop/StartPublishLoop → 3 executor(sensor/log/aux) → join → Stop → shutdown

ur5e_bringup에서 재사용: ur5e_bringup/src/ur5e_rt_controller_main.cpp
  → rtc::RtControllerMain(argc, argv) 호출

Public API:
  GetSensorGroup(), GetLogGroup(), GetAuxGroup()  — executor에서 사용
  StartRtLoop(ThreadConfig), StopRtLoop()         — → private로 이동
  StartPublishLoop(ThreadConfig), StopPublishLoop() — → private로 이동

Publisher 구조체 (header):
  struct PublisherEntry { rclcpp::Publisher<Float64MultiArray>::SharedPtr publisher; ... };
  struct JointCommandPublisherEntry { rclcpp::Publisher<JointCommand>::SharedPtr publisher; ... };
  template<typename MsgT> struct TypedPublisherEntry { rclcpp::Publisher<MsgT>::SharedPtr publisher; ... };
  struct DigitalTwinEntry { rclcpp::Publisher<JointState>::SharedPtr publisher; ... };

안전 publisher (일반 Publisher 유지):
  line 153: rclcpp::Publisher<Bool>::SharedPtr estop_pub_;
  line 154: rclcpp::Publisher<String>::SharedPtr active_ctrl_name_pub_;
  line 155: rclcpp::Publisher<Float64MultiArray>::SharedPtr current_gains_pub_;
```

---

## 5. 구현 순서 (9 Steps)

### Step 1 — Infrastructure: rclcpp_lifecycle 의존성 추가

**목적:** 코드 변경 없이 빌드 의존성만 추가하여 빌드 검증.

**수정 파일 (12개):**

| Package | CMakeLists.txt 추가 내용 | package.xml 추가 내용 |
|---------|------------------------|---------------------|
| `rtc_controller_manager` | `find_package(rclcpp_lifecycle REQUIRED)`, `ament_target_dependencies(rtc_controller_manager_lib ... rclcpp_lifecycle)`, `ament_export_dependencies(... rclcpp_lifecycle)` | `<depend>rclcpp_lifecycle</depend>` |
| `rtc_mujoco_sim` | `find_package(rclcpp_lifecycle REQUIRED)`, mujoco_simulator_node target에 추가 | `<depend>rclcpp_lifecycle</depend>` |
| `ur5e_hand_driver` | `find_package(rclcpp_lifecycle REQUIRED)`, hand_udp_node target에 추가 | `<depend>rclcpp_lifecycle</depend>` |
| `ur5e_bt_coordinator` | `find_package(rclcpp_lifecycle REQUIRED)`, bt target에 추가 | `<depend>rclcpp_lifecycle</depend>` |
| `shape_estimation` | `find_package(rclcpp_lifecycle REQUIRED)`, shape target에 추가 | `<depend>rclcpp_lifecycle</depend>` |
| `ur5e_bringup` | `find_package(rclcpp_lifecycle REQUIRED)`, ur5e_rt_controller target에 추가 | `<depend>rclcpp_lifecycle</depend>` |

**검증:** `./build.sh full` — 0 errors, 0 new warnings

---

### Step 2 — HandUdpNode 변환

**수정 파일:** `ur5e_hand_driver/src/hand_udp_node.cpp`, `ur5e_hand_driver/launch/hand_udp.launch.py`

**변환 요약:**
```cpp
// BEFORE
class HandUdpNode : public rclcpp::Node {
  HandUdpNode() : Node("hand_udp_node") { /* 전체 초기화 */ }
};

// AFTER
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

class HandUdpNode : public rclcpp_lifecycle::LifecycleNode {
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  HandUdpNode() : LifecycleNode("hand_udp_node") {}  // 비움

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) {
    // 기존 constructor의 모든 코드 이동 (controller_->Start() 제외)
    // publisher: create_publisher → LifecyclePublisher 반환
    // link_status_pub_만 rclcpp::create_publisher(...) standalone으로 일반 Publisher
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) {
    LifecycleNode::on_activate(state);  // ★ 부모 호출
    controller_->Start();
    // fake_tick_timer_, FailureDetector 시작
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) {
    // FailureDetector 중지, controller_->Stop(), fake_tick_timer_ 취소
    LifecycleNode::on_deactivate(state);  // ★ 부모 호출
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) {
    // on_configure의 역순: 모든 timer/sub/pub .reset(), controller_.reset()
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) {
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      on_deactivate(state);
    return on_cleanup(state);
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State&) {
    // controller Stop, 전체 cleanup, return SUCCESS (복구 가능)
  }
};
```

**Publisher 타입 변경:**
```cpp
// BEFORE
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
// AFTER
rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

// link_status_pub_만 일반 Publisher 유지:
// on_configure에서: link_status_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
//     this->get_node_topics_interface(), link_status_topic, link_pub_qos);
```

**Main 변경:**
```cpp
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandUdpNode>();
  rclcpp::spin(node->get_node_base_interface());  // ★ get_node_base_interface()
  rclcpp::shutdown();
}
```

**Launch 변경 (`hand_udp.launch.py`):**
```python
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# Node(...) → LifecycleNode(...)
hand_udp_node = LifecycleNode(
    package='ur5e_hand_driver', executable='hand_udp_node',
    name='hand_udp_node', output='screen', parameters=[...], emulate_tty=True,
)

# auto-configure → auto-activate event handler chain
register_activate = RegisterEventHandler(OnStateTransition(
    target_lifecycle_node=hand_udp_node,
    start_state='configuring', goal_state='inactive',
    entities=[EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(hand_udp_node),
        transition_id=Transition.TRANSITION_ACTIVATE,
    ))],
))
trigger_configure = EmitEvent(event=ChangeState(
    lifecycle_node_matcher=matches_action(hand_udp_node),
    transition_id=Transition.TRANSITION_CONFIGURE,
))

# LaunchDescription에 추가:
# [... args ..., hand_udp_node, register_activate, trigger_configure]
```

**검증:** `./build.sh -p ur5e_hand_driver && ros2 launch ur5e_hand_driver hand_udp.launch.py use_fake_hand:=true`

---

### Step 3 — ShapeEstimationNode 변환

**수정 파일:** header, source, main, launch

- 동일 패턴: constructor 비움 → on_configure로 이동
- 내부 FSM (`kStopped/kRunning/kPaused`)은 lifecycle과 독립 유지
- timer callback에 state guard 추가: `if (get_current_state().id() != PRIMARY_STATE_ACTIVE) return;`
- action server는 on_configure에서 생성

---

### Step 4 — BtCoordinatorNode 변환

**수정 파일:** header, source, main, bt_ros_bridge.hpp, bt_ros_bridge.cpp, launch

- **`Initialize()` 메서드 제거** → `on_configure`로 흡수
- `shared_from_this()`는 `on_configure`에서 안전 (lifecycle node는 반드시 shared_ptr로 관리)
- **BtRosBridge 타입 변경 (3곳):**
  ```cpp
  rclcpp::Node::SharedPtr → rclcpp_lifecycle::LifecycleNode::SharedPtr
  ```
- tick_timer, watchdog_timer → on_activate에서 생성, on_deactivate에서 취소
- `tree_->haltTree()` in on_deactivate

---

### Step 5 — MuJoCoSimulatorNode 변환

**수정 파일:** `rtc_mujoco_sim/src/mujoco_simulator_node.cpp`, launch 2개

- **★ 선행 수정:** `sim_status_pub_` 멤버 초기화 리스트 버그 (line 489-490) → on_configure로 이동
- GroupRosHandles의 `state_pub`, `sensor_pub` → LifecyclePublisher 타입
- `sim_->Start()` → on_activate, `sim_->Stop()` → on_deactivate
- fake timers, status_timer → on_activate에서 생성

---

### Step 6 — RtControllerNode 헤더 변경

**수정 파일:** `rtc_controller_manager/include/rtc_controller_manager/rt_controller_node.hpp`

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

class RtControllerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RtControllerNode();
  ~RtControllerNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

  rclcpp::CallbackGroup::SharedPtr GetSensorGroup() const { return cb_group_sensor_; }
  rclcpp::CallbackGroup::SharedPtr GetLogGroup() const { return cb_group_log_; }
  rclcpp::CallbackGroup::SharedPtr GetAuxGroup() const { return cb_group_aux_; }

private:
  void StartRtLoop(const rtc::ThreadConfig&);    // ★ public → private
  void StartPublishLoop(const rtc::ThreadConfig&);
  void StopRtLoop();
  void StopPublishLoop();
  // ...

  // Publisher 구조체 변경:
  struct PublisherEntry {
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    // ...
  };
  // (JointCommandPublisherEntry, TypedPublisherEntry, DigitalTwinEntry도 동일)

  // 안전 publisher (일반 rclcpp::Publisher 유지):
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_ctrl_name_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_gains_pub_;

  rtc::SystemThreadConfigs thread_configs_{};  // 추가
};
```

---

### Step 7 — RtControllerNode lifecycle 콜백 구현

**수정 파일:** `rt_controller_node.cpp`, `rt_controller_node_topics.cpp`, `rt_controller_node_estop.cpp`, `rt_controller_node_publish.cpp`

- Constructor: `LifecycleNode(...)` + `logger_(nullptr)` 만. **CreateCallbackGroups 포함하여 모든 로직 제거.**
- `on_configure`: 기존 constructor 로직 전체 (CreateCallbackGroups → DeclareAndLoadParameters → CreateSubscriptions → CreatePublishers → ExposeTopicParameters → CreateTimers → eventfd)
- `on_activate`: `LifecycleNode::on_activate(state)` + `SelectThreadConfigs()` + `StartRtLoop` + `StartPublishLoop`
- `on_deactivate`: `StopRtLoop` + `StopPublishLoop` + `ClearGlobalEstop` + state reset + `LifecycleNode::on_deactivate(state)`
- `on_cleanup`: eventfd close, logger drain, 모든 sub/pub/timer `.reset()`, controllers clear
- `on_error`: `TriggerGlobalEstop("lifecycle_error")` + Stop threads + cleanup + return SUCCESS

**안전 publisher 생성 (rt_controller_node_topics.cpp):**
```cpp
// on_configure 내에서:
estop_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
    this->get_node_topics_interface(),
    "/system/estop_status",
    rclcpp::QoS(10).transient_local());
```

---

### Step 8 — RtControllerMain() 재설계

**수정 파일:** `rtc_controller_manager/src/rt_controller_main_impl.cpp`

**3-phase executor 패턴:**

```
Phase 1: lifecycle_executor로 lifecycle service 처리
  → launch event handler의 configure/activate 수신
  → on_configure (callback groups 생성), on_activate (RT loop 시작)

Phase 2: Active 상태 대기 (polling)

Phase 3: 전용 executor(sensor/log/aux)로 전환
  → lifecycle_executor에서 node 제거
  → sensor_executor.add_callback_group(node->GetSensorGroup(), ...)
  → log_executor.add_callback_group(node->GetLogGroup(), ...)
  → aux_executor.add_callback_group(node->GetAuxGroup(), ...)
  → aux_executor에 default callback group 추가 (lifecycle services 계속 처리)
  → 전용 스레드에서 spin

Graceful shutdown:
  → trigger_deactivate() → trigger_cleanup() → rclcpp::shutdown()
```

---

### Step 9 — Launch 파일 + 통합 테스트

**수정 Launch 파일:**
- `ur5e_hand_driver/launch/hand_udp.launch.py`
- `shape_estimation/launch/shape_estimation_node.launch.py`
- `ur5e_bt_coordinator/launch/bt_coordinator.launch.py`
- `rtc_mujoco_sim/launch/mujoco_sim.launch.py`
- `ur5e_bringup/launch/sim.launch.py`
- `ur5e_bringup/launch/robot.launch.py`

**공통 패턴:** `Node(...)` → `LifecycleNode(...)` + auto-configure/activate event handler

**통합 테스트:**
```bash
./build.sh full
colcon test --event-handlers console_direct+
colcon test-result --verbose

ros2 launch ur5e_bringup sim.launch.py
ros2 lifecycle list /mujoco_simulator
ros2 lifecycle get /rt_controller       # expect: active
ros2 topic hz /forward_position_controller/commands  # ~500Hz
ros2 lifecycle set /mujoco_simulator deactivate  # → sim stops
ros2 lifecycle set /mujoco_simulator activate    # → sim restarts
```

---

## 6. 주의사항 체크리스트

- [ ] 모든 `on_activate`에서 `LifecycleNode::on_activate(state)` 호출 (LifecyclePublisher 활성화)
- [ ] 모든 `on_deactivate`에서 `LifecycleNode::on_deactivate(state)` 호출
- [ ] `on_cleanup`이 `on_configure`의 정확한 역순인지 확인
- [ ] `on_shutdown`이 어떤 state에서든 안전한지 확인
- [ ] `on_error`가 SUCCESS 반환 (복구 가능)인지 확인
- [ ] Timer callback에 state guard 추가했는지 확인
- [ ] `rclcpp::spin(node)` → `rclcpp::spin(node->get_node_base_interface())` 변경
- [ ] E-STOP publisher가 `rclcpp::create_publisher` (standalone) 사용인지 확인
- [ ] MuJoCo `sim_status_pub_` 멤버 초기화 리스트 버그 수정 (on_configure로 이동)
- [ ] BtRosBridge의 3곳 `Node::SharedPtr` → `LifecycleNode::SharedPtr` 변경
- [ ] BtCoordinatorNode의 `Initialize()` 메서드 제거 (on_configure로 흡수)
- [ ] RtControllerNode의 `StartRtLoop/StopRtLoop` public → private 이동
- [ ] `rtc_controller_manager_lib`의 `ament_export_dependencies`에 `rclcpp_lifecycle` 추가
- [ ] 기존 159 테스트 모두 통과 확인

---

## 7. Risk Matrix

| Risk | Severity | Mitigation |
|------|----------|------------|
| E-STOP publisher Inactive에서 미동작 | **Critical** | `rclcpp::create_publisher` standalone으로 일반 Publisher |
| 부모 on_activate 미호출 | **High** | 모든 callback에서 체이닝 (Nav2 #3710) |
| MuJoCo member-init crash | **High** | on_configure로 이동 |
| RtControllerMain lifecycle service 미처리 | **High** | 3-phase executor 패턴 |
| Timer Inactive에서 실행 | Medium | state guard 추가 |
| on_error 기본 Finalized | Medium | SUCCESS 반환 |
| BtRosBridge 타입 불일치 | Medium | 3곳 타입 변경 |
| on_cleanup 리소스 누수 | Medium | 역순 .reset() 체크리스트 |
