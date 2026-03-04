// ── Includes: project header first, then ROS2, then C++ stdlib ────────────────
//
// ── Available Pinocchio-based controllers ────────────────────────────────────
//
// Three model-based controllers are available as drop-in replacements.
// For any of them: change the controller_ member type (≈line 340) to
//   std::unique_ptr<urtc::RTControllerInterface>
// and remove the set_gains() call inside DeclareAndLoadParameters().
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ 1. PinocchioController — joint-space PD + gravity / Coriolis           │
// │    Target: 6 joint angles [q0..q5] (same as PDController)              │
// │    #include "ur5e_rt_controller/controllers/pinocchio_controller.hpp"  │
// │                                                                         │
// │    controller_(std::make_unique<urtc::PinocchioController>(            │
// │        "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",          │
// │        urtc::PinocchioController::Gains{                               │
// │            .kp = 5.0, .kd = 0.5,                                       │
// │            .enable_gravity_compensation  = true,                        │
// │            .enable_coriolis_compensation = false}))                     │
// ├─────────────────────────────────────────────────────────────────────────┤
// │ 2. ClikController — Closed-Loop IK, Cartesian position control (3-DOF) │
// │    Target: [x, y, z, null_q3, null_q4, null_q5]                        │
// │      [0..2] = desired TCP position in world frame (metres)              │
// │      [3..5] = null-space reference for joints 3–5 (radians)            │
// │    #include "ur5e_rt_controller/controllers/clik_controller.hpp"       │
// │                                                                         │
// │    controller_(std::make_unique<urtc::ClikController>(                 │
// │        "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",          │
// │        urtc::ClikController::Gains{                                    │
// │            .kp = 1.0, .damping = 0.01, .null_kp = 0.5}))              │
// ├─────────────────────────────────────────────────────────────────────────┤
// │ 3. OperationalSpaceController — OSC, full 6-DOF Cartesian PD control  │
// │    Target: [x, y, z, roll, pitch, yaw]  (metres / radians, ZYX)       │
// │    #include "ur5e_rt_controller/controllers/                           │
// │              operational_space_controller.hpp"                          │
// │                                                                         │
// │    controller_(std::make_unique<urtc::OperationalSpaceController>(     │
// │        "/opt/ros/humble/share/ur_description/urdf/ur5e.urdf",          │
// │        urtc::OperationalSpaceController::Gains{                        │
// │            .kp_pos = 1.0, .kd_pos = 0.1,                               │
// │            .kp_rot = 0.5, .kd_rot = 0.05, .damping = 0.01}))          │
// └─────────────────────────────────────────────────────────────────────────┘
// ────────────────────────────────────────────────────────────────────────────
#include "ur5e_rt_controller/controllers/pd_controller.hpp"
#include "ur5e_rt_controller/controller_timing_profiler.hpp"
#include "ur5e_rt_controller/data_logger.hpp"
#include "ur5e_rt_controller/log_buffer.hpp"
#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include "ur5e_rt_controller/thread_config.hpp"
#include "ur5e_rt_controller/thread_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/mman.h>  // mlockall

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// ── CustomController ───────────────────────────────────────────────────────────
//
// 500 Hz position controller node with multi-threaded executors.
//
// CallbackGroup assignment:
//   - cb_group_rt_:     control_timer_, timeout_timer_  (RT core)
//   - cb_group_sensor_: joint_state_sub_, target_sub_, hand_state_sub_  (Sensor core)
//   - cb_group_log_:    drain_timer_  (non-RT core)
//   - cb_group_aux_:    estop_pub_  (aux core)
class CustomController : public rclcpp::Node {
 public:
  CustomController()
      : Node("custom_controller"),
        controller_(std::make_unique<urtc::PDController>()),
        logger_(std::make_unique<urtc::DataLogger>("/tmp/ur5e_control_log.csv"))
  {
    CreateCallbackGroups();
    DeclareAndLoadParameters();
    CreateSubscriptions();
    CreatePublishers();
    CreateTimers();

    RCLCPP_INFO(get_logger(), "CustomController ready — %.0f Hz, E-STOP: %s",
                control_rate_, enable_estop_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "CallbackGroups enabled: RT, Sensor, Log, Aux");
  }

  ~CustomController() override {
    if (logger_) {
      logger_->Flush();
    }
  }

  // Public accessors for main() to retrieve callback groups
  rclcpp::CallbackGroup::SharedPtr GetRtGroup()     const { return cb_group_rt_; }
  rclcpp::CallbackGroup::SharedPtr GetSensorGroup() const { return cb_group_sensor_; }
  rclcpp::CallbackGroup::SharedPtr GetLogGroup()    const { return cb_group_log_; }
  rclcpp::CallbackGroup::SharedPtr GetAuxGroup()    const { return cb_group_aux_; }

 private:
  // ── CallbackGroup creation ──────────────────────────────────────────────────
  void CreateCallbackGroups() {
    cb_group_rt_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_sensor_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_log_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_aux_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  // ── Initialisation helpers ──────────────────────────────────────────────────
  void DeclareAndLoadParameters() {
    declare_parameter("control_rate",    500.0);
    declare_parameter("kp",              5.0);
    declare_parameter("kd",              0.5);
    declare_parameter("enable_logging",  true);
    declare_parameter("robot_timeout_ms", 100.0);
    declare_parameter("hand_timeout_ms",  200.0);
    declare_parameter("enable_estop",    true);

    control_rate_   = get_parameter("control_rate").as_double();
    enable_logging_ = get_parameter("enable_logging").as_bool();
    enable_estop_   = get_parameter("enable_estop").as_bool();

    robot_timeout_ = std::chrono::milliseconds(
        static_cast<int>(get_parameter("robot_timeout_ms").as_double()));
    hand_timeout_ = std::chrono::milliseconds(
        static_cast<int>(get_parameter("hand_timeout_ms").as_double()));

    const urtc::PDController::Gains gains{
        .kp = get_parameter("kp").as_double(),
        .kd = get_parameter("kd").as_double(),
    };
    controller_->set_gains(gains);
  }

  void CreateSubscriptions() {
    // Assign subscriptions to cb_group_sensor_
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_sensor_;

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          JointStateCallback(std::move(msg));
        },
        sub_options);

    target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_joint_positions", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          TargetCallback(std::move(msg));
        },
        sub_options);

    hand_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          HandStateCallback(std::move(msg));
        },
        sub_options);
  }

  void CreatePublishers() {
    // Fix 5: RealtimePublisher avoids heap allocation / DDS lock on the RT path.
    // The message buffer is pre-allocated once here so ControlLoop() only does
    // a std::copy + trylock — both O(1) and lock-free on the fast path.
    auto raw_pub = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands", 10);
    rt_command_pub_ = std::make_unique<
        realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(raw_pub);
    rt_command_pub_->msg_.data.resize(urtc::kNumRobotJoints, 0.0);

    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/system/estop_status", 10);
  }

  void CreateTimers() {
    const auto control_period = std::chrono::microseconds(
        static_cast<int>(1'000'000.0 / control_rate_));

    // Assign control_timer_ and timeout_timer_ to cb_group_rt_
    control_timer_ = create_wall_timer(
        control_period,
        [this]() { ControlLoop(); },
        cb_group_rt_);

    if (enable_estop_) {
      timeout_timer_ = create_wall_timer(
          20ms,
          [this]() { CheckTimeouts(); },
          cb_group_rt_);
    }

    // Fix 1: drain the SPSC log ring buffer from the log thread (Core 4).
    // File I/O stays entirely out of the 500 Hz RT thread.
    drain_timer_ = create_wall_timer(
        10ms,
        [this]() { DrainLog(); },
        cb_group_log_);
  }

  // ── Subscription callbacks ──────────────────────────────────────────────────
  void JointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < urtc::kNumRobotJoints) {
      return;
    }
    {
      std::lock_guard lock(state_mutex_);
      std::copy_n(msg->position.begin(), urtc::kNumRobotJoints,
                  current_positions_.begin());
      std::copy_n(msg->velocity.begin(), urtc::kNumRobotJoints,
                  current_velocities_.begin());
      last_robot_update_ = now();
    }
    // Fix 2: release-store after the mutex — RT thread reads with acquire,
    // so the C++ memory model guarantees it sees the written positions/velocities.
    state_received_.store(true, std::memory_order_release);
  }

  void TargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < urtc::kNumRobotJoints) {
      return;
    }
    // Fix 3: build a local copy while the mutex is held, then call
    // SetRobotTarget() with the copy — avoids a data race where the RT thread
    // could overwrite target_positions_ between mutex release and the call.
    std::array<double, urtc::kNumRobotJoints> local_target;
    {
      std::lock_guard lock(target_mutex_);
      std::copy_n(msg->data.begin(), urtc::kNumRobotJoints,
                  target_positions_.begin());
      local_target = target_positions_;
    }
    // Fix 2: release-store after mutex released
    target_received_.store(true, std::memory_order_release);
    controller_->SetRobotTarget(local_target);
  }

  void HandStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr /*msg*/) {
    {
      std::lock_guard lock(hand_mutex_);
      last_hand_update_ = now();
    }
    // Fix 2: release-store after mutex released
    hand_data_received_.store(true, std::memory_order_release);
  }

  // ── 50 Hz watchdog (E-STOP) ─────────────────────────────────────────────────
  void CheckTimeouts() {
    const auto now_time = now();
    bool robot_timed_out = false;
    bool hand_timed_out  = false;

    // Fix 2: acquire-load the atomic flag; if set, lock mutex to read timestamp
    if (state_received_.load(std::memory_order_acquire)) {
      std::lock_guard lock(state_mutex_);
      robot_timed_out = (now_time - last_robot_update_) > robot_timeout_;
    }
    if (hand_data_received_.load(std::memory_order_acquire)) {
      std::lock_guard lock(hand_mutex_);
      hand_timed_out = (now_time - last_hand_update_) > hand_timeout_;
    }

    if (robot_timed_out && !controller_->IsEstopped()) {
      RCLCPP_ERROR(get_logger(), "Robot data timeout — triggering E-STOP");
      controller_->TriggerEstop();
      PublishEstopStatus(true);
    }

    if (hand_timed_out) {
      controller_->SetHandEstop(true);
      if (!hand_estop_logged_) {
        RCLCPP_WARN(get_logger(), "Hand data timeout — hand E-STOP active");
        hand_estop_logged_ = true;
      }
    } else {
      if (hand_estop_logged_) {
        RCLCPP_INFO(get_logger(), "Hand data restored — hand E-STOP cleared");
        hand_estop_logged_ = false;
      }
      controller_->SetHandEstop(false);
    }
  }

  // ── 500 Hz control loop ─────────────────────────────────────────────────────
  void ControlLoop() {
    // Fix 2: acquire-load atomics — no mutex needed for the readiness check
    if (!state_received_.load(std::memory_order_acquire) ||
        !target_received_.load(std::memory_order_acquire)) {
      return;
    }

    urtc::ControllerState state{};
    {
      std::lock_guard lock(state_mutex_);
      state.robot.positions  = current_positions_;
      state.robot.velocities = current_velocities_;
    }
    // Fix 4: copy target_positions_ into target_snapshot_ while holding the
    // mutex, then use only the snapshot.  The old code set state.robot.dt and
    // state.iteration inside target_mutex_, which was incorrect.
    {
      std::lock_guard lock(target_mutex_);
      target_snapshot_ = target_positions_;
    }
    state.robot.dt  = 1.0 / control_rate_;
    state.iteration = loop_count_;

    // Measure Compute() wall-clock time via ControllerTimingProfiler.
    const urtc::ControllerOutput output =
        timing_profiler_.MeasuredCompute(*controller_, state);

    // Fix 5: trylock is non-blocking; if the DDS layer still holds the message
    // we simply skip publishing this cycle (jitter < 2 ms is acceptable).
    if (rt_command_pub_->trylock()) {
      std::copy(output.robot_commands.begin(), output.robot_commands.end(),
                rt_command_pub_->msg_.data.begin());
      rt_command_pub_->unlockAndPublish();
    }

    // Fix 1: push log entry to the SPSC ring buffer — O(1), no syscall.
    // DrainLog() (log thread, Core 4) pops entries and writes the CSV file.
    if (enable_logging_) {
      const urtc::LogEntry entry{
          .timestamp         = now().seconds(),
          .current_positions = state.robot.positions,
          .target_positions  = target_snapshot_,   // Fix 4: snapshot, not raw member
          .commands          = output.robot_commands,
          .compute_time_us   = timing_profiler_.LastComputeUs(),
      };
      log_buffer_.Push(entry);  // silently drops if buffer is full
    }

    ++loop_count_;
    // Print timing summary every 1 000 iterations.
    if (loop_count_ % 1000 == 0) {
      RCLCPP_INFO(get_logger(), "%s",
                  timing_profiler_.Summary(
                      std::string(controller_->Name())).c_str());
    }
  }

  // Fix 1: file I/O stays exclusively in the log thread (Core 4).
  void DrainLog() {
    if (!logger_) return;
    logger_->DrainBuffer(log_buffer_);
  }

  void PublishEstopStatus(bool estopped) {
    std_msgs::msg::Bool msg;
    msg.data = estopped;
    estop_pub_->publish(msg);
  }

  // ── ROS2 handles ────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_rt_;
  rclcpp::CallbackGroup::SharedPtr cb_group_sensor_;
  rclcpp::CallbackGroup::SharedPtr cb_group_log_;
  rclcpp::CallbackGroup::SharedPtr cb_group_aux_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  hand_state_sub_;

  // Fix 5: RealtimePublisher with pre-allocated message — no heap allocation
  // on the RT path. trylock()/unlockAndPublish() are lock-free on the fast path.
  std::unique_ptr<realtime_tools::RealtimePublisher<
      std_msgs::msg::Float64MultiArray>>                             rt_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  estop_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr drain_timer_;   // Fix 1: log drain (log thread)

  // ── Domain objects ──────────────────────────────────────────────────────────
  std::unique_ptr<urtc::PDController>    controller_;
  std::unique_ptr<urtc::DataLogger>      logger_;
  urtc::ControlLogBuffer                 log_buffer_{};  // Fix 1: SPSC ring buffer
  urtc::ControllerTimingProfiler         timing_profiler_{};  // Compute() timing

  // ── Shared state (guarded by per-domain mutexes) ────────────────────────────
  std::array<double, urtc::kNumRobotJoints> current_positions_{};
  std::array<double, urtc::kNumRobotJoints> current_velocities_{};
  std::array<double, urtc::kNumRobotJoints> target_positions_{};
  // Fix 4: RT-local snapshot of target — written and read only in ControlLoop()
  std::array<double, urtc::kNumRobotJoints> target_snapshot_{};

  mutable std::mutex state_mutex_;
  mutable std::mutex target_mutex_;
  mutable std::mutex hand_mutex_;

  // Fix 2: atomic flags — safe to read without a mutex in the RT thread.
  // Written with release, read with acquire to guarantee visibility ordering.
  std::atomic<bool> state_received_{false};
  std::atomic<bool> target_received_{false};
  std::atomic<bool> hand_data_received_{false};

  rclcpp::Time              last_robot_update_;
  rclcpp::Time              last_hand_update_;
  std::chrono::milliseconds robot_timeout_{100};
  std::chrono::milliseconds hand_timeout_{200};

  // ── Parameters ──────────────────────────────────────────────────────────────
  double control_rate_{500.0};
  bool   enable_logging_{true};
  bool   enable_estop_{true};
  bool   hand_estop_logged_{false};

  std::size_t loop_count_{0};
};

// ── Entry point ────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  // Fix 7: mlockall BEFORE rclcpp::init.
  // MCL_CURRENT locks pages already mapped; MCL_FUTURE ensures every page
  // allocated afterwards (including DDS/RMW heaps) is also locked.
  // Calling mlockall after rclcpp::init leaves the DDS stack unprotected.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] mlockall failed — page faults possible\n");
    fprintf(stderr, "       Check: /etc/security/limits.conf @realtime memlock unlimited\n");
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<CustomController>();

  // 2. Create executors for each callback group
  rclcpp::executors::SingleThreadedExecutor rt_executor;
  rclcpp::executors::SingleThreadedExecutor sensor_executor;
  rclcpp::executors::SingleThreadedExecutor log_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  // 3. Add callback groups to respective executors
  rt_executor.add_callback_group(
      node->GetRtGroup(), node->get_node_base_interface());
  sensor_executor.add_callback_group(
      node->GetSensorGroup(), node->get_node_base_interface());
  log_executor.add_callback_group(
      node->GetLogGroup(), node->get_node_base_interface());
  aux_executor.add_callback_group(
      node->GetAuxGroup(), node->get_node_base_interface());

  // 4. Helper lambda to create thread with RT config
  auto make_thread = [](auto& executor, const urtc::ThreadConfig& cfg) {
    return std::thread([&executor, cfg]() {
      if (!urtc::ApplyThreadConfig(cfg)) {
        fprintf(stderr, "[WARN] Thread config failed for '%s' (need realtime permissions)\n",
                cfg.name.c_str());
      } else {
        fprintf(stdout, "[INFO] Thread '%s' configured:\n%s",
                cfg.name.c_str(),
                urtc::VerifyThreadConfig().c_str());
      }
      executor.spin();
    });
  };

  // Fix 9: select 6-core or 4-core thread configs at runtime based on the
  // number of online CPUs detected via sysconf(_SC_NPROCESSORS_ONLN).
  const auto cfgs = urtc::SelectThreadConfigs();

  auto t_rt     = make_thread(rt_executor,     cfgs.rt_control);
  auto t_sensor = make_thread(sensor_executor, cfgs.sensor);
  auto t_log    = make_thread(log_executor,    cfgs.logging);
  auto t_aux    = make_thread(aux_executor,    cfgs.aux);

  // 6. Wait for threads to finish
  t_rt.join();
  t_sensor.join();
  t_log.join();
  t_aux.join();

  rclcpp::shutdown();
  return 0;
}
