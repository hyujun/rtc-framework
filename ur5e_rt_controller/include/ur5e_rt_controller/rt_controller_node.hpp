#pragma once

// ── Project headers ───────────────────────────────────────────────────────────
#include "ur5e_rt_base/logging/data_logger.hpp"
#include "ur5e_rt_base/logging/log_buffer.hpp"
#include "ur5e_rt_controller/controller_timing_profiler.hpp"
#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include <ur5e_hand_udp/hand_controller.hpp>
#include <ur5e_status_monitor/ur5e_status_monitor.hpp>

// ── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ur5e_msgs/msg/joint_command.hpp>

// ── C++ stdlib ────────────────────────────────────────────────────────────────
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// ── RtControllerNode ──────────────────────────────────────────────────────────
//
// 500 Hz position controller node with multi-threaded executors.
//
// CallbackGroup assignment:
//   - cb_group_rt_:     control_timer_, timeout_timer_  (RT core)
//   - cb_group_sensor_: joint_state_sub_, target_sub_, hand_state_sub_  (Sensor core)
//   - cb_group_log_:    drain_timer_  (non-RT core)
//   - cb_group_aux_:    estop_pub_  (aux core)
class RtControllerNode : public rclcpp::Node
{
public:
  RtControllerNode();
  ~RtControllerNode() override;

  // Public accessors for main() to retrieve callback groups
  rclcpp::CallbackGroup::SharedPtr GetRtGroup()     const {return cb_group_rt_;}
  rclcpp::CallbackGroup::SharedPtr GetSensorGroup() const {return cb_group_sensor_;}
  rclcpp::CallbackGroup::SharedPtr GetLogGroup()    const {return cb_group_log_;}
  rclcpp::CallbackGroup::SharedPtr GetAuxGroup()    const {return cb_group_aux_;}

private:
  // ── Session directory helpers ─────────────────────────────────────────────
  // Resolves session directory from UR5E_SESSION_DIR env var or creates one.
  std::filesystem::path ResolveAndSetupSessionDir();

  // ── Initialisation helpers ────────────────────────────────────────────────
  void CreateCallbackGroups();
  void DeclareAndLoadParameters();
  void CreateSubscriptions();
  void CreatePublishers();
  void ExposeTopicParameters();
  void CreateTimers();

  // ── Subscription callbacks ────────────────────────────────────────────────
  void JointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg);
  void RobotTargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void HandTargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ── Joint name validation (v5.14.0) ──────────────────────────────────────
  void LoadAndValidateJointNames();
  void BuildJointStateIndexMap(const std::vector<std::string>& msg_names);
  void BuildHandStateIndexMap(const std::vector<std::string>& source_names);

  // ── Timer callbacks ───────────────────────────────────────────────────────
  void CheckTimeouts();   // 50 Hz watchdog (E-STOP)
  void ControlLoop();     // 500 Hz control loop
  void DrainLog();        // Log drain (non-RT core)

  void PublishEstopStatus(bool estopped);

  // Save hand communication & timing statistics to JSON on shutdown.
  void SaveHandStats() const;

  /// Trigger a global E-Stop that propagates to all subsystems.
  /// Safe to call from any thread. Idempotent — second call is a no-op.
  void TriggerGlobalEstop(std::string_view reason) noexcept;
  /// Clear global E-Stop and re-enable all subsystems.
  void ClearGlobalEstop() noexcept;
  [[nodiscard]] bool IsGlobalEstopped() const noexcept
  {
    return global_estop_.load(std::memory_order_acquire);
  }

  // ── ROS2 handles ──────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_rt_;
  rclcpp::CallbackGroup::SharedPtr cb_group_sensor_;
  rclcpp::CallbackGroup::SharedPtr cb_group_log_;
  rclcpp::CallbackGroup::SharedPtr cb_group_aux_;

  // ── Configurable topic subscriptions (created from controller YAML) ──────
  // Key = topic name, value = subscription handle (kept alive for node lifetime)
  std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subscriptions_;

  // Fixed control subscriptions (always present)
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr              controller_selector_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  controller_gains_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               request_gains_sub_;

  // ── Configurable topic publishers (created from controller YAML) ──────────
  // Key = topic name, value = publisher + pre-allocated message
  struct PublisherEntry {
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    std_msgs::msg::Float64MultiArray msg;
  };
  std::unordered_map<std::string, PublisherEntry> topic_publishers_;
  std::mutex                                      cmd_pub_mutex_;

  // Fixed publishers (always present)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            active_ctrl_name_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_gains_pub_;

  // JointCommand publisher (MuJoCo / 외부 시뮬레이터용)
  rclcpp::Publisher<ur5e_msgs::msg::JointCommand>::SharedPtr     joint_command_pub_;
  ur5e_msgs::msg::JointCommand                                   joint_command_msg_;  // pre-allocated

  // Per-controller topic config cache (index = controller index)
  std::vector<ur5e_rt_controller::TopicConfig> controller_topic_configs_;

  // ── Device enable/disable flags ──────────────────────────────────────────
  ur5e_rt_controller::DeviceEnableFlags global_device_flags_;
  std::vector<ur5e_rt_controller::DeviceEnableFlags> controller_device_flags_;
  [[nodiscard]] ur5e_rt_controller::DeviceEnableFlags ResolveDeviceFlags(
      const ur5e_rt_controller::PerControllerDeviceFlags & per_ctrl) const noexcept;

  // Read-only parameter guard handle (topic params immutable after init)
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr drain_timer_;  // log drain (log thread)

  // ── Domain objects ────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<ur5e_rt_controller::RTControllerInterface>> controllers_;
  std::atomic<int> active_controller_idx_{1};
  std::unique_ptr<ur5e_rt_controller::DataLogger> logger_;
  ur5e_rt_controller::ControlLogBuffer              log_buffer_{};              // SPSC ring buffer
  ur5e_rt_controller::ControllerTimingProfiler      timing_profiler_{};         // Compute() timing

  // ── Status Monitor (optional, non-RT) ──────────────────────────────────────
  std::unique_ptr<ur5e_status_monitor::UR5eStatusMonitor> status_monitor_;
  bool enable_status_monitor_{false};

  // ── Hand Controller (직접 소유, event-driven) ──────────────────────────────
  std::unique_ptr<ur5e_rt_controller::HandController> hand_controller_;
  bool enable_hand_{false};
  // RT-local hand state cache — updated from HandController or sim each tick
  ur5e_rt_controller::HandState cached_hand_state_{};

  // ── Hand Simulation (ROS 토픽 기반, MuJoCo fake response 연동) ────────────
  bool hand_sim_enabled_{false};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  hand_sim_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hand_sim_state_sub_;
  std_msgs::msg::Float64MultiArray hand_sim_cmd_msg_;  // pre-allocated
  mutable std::mutex sim_hand_mutex_;
  ur5e_rt_controller::HandState sim_hand_state_{};

  // ── Shared state (guarded by per-domain mutexes) ──────────────────────────
  std::array<double, ur5e_rt_controller::kNumRobotJoints> current_positions_{};
  std::array<double, ur5e_rt_controller::kNumRobotJoints> current_velocities_{};
  std::array<double, ur5e_rt_controller::kNumRobotJoints> target_positions_{};

  // Joint torques from effort field (guarded by state_mutex_)
  std::array<double, ur5e_rt_controller::kNumRobotJoints> current_torques_{};
  std::array<double, ur5e_rt_controller::kNumRobotJoints> cached_torques_{};

  // RT-local snapshot of target — written and read only in ControlLoop()
  std::array<double, ur5e_rt_controller::kNumRobotJoints> target_snapshot_{};

  // RT-local cached copies — updated via try_lock to avoid blocking the RT
  // thread.  If the mutex is contended, the previous cycle's data is reused
  // (stale by at most one cycle = 2 ms, acceptable for position control).
  std::array<double, ur5e_rt_controller::kNumRobotJoints> cached_positions_{};
  std::array<double, ur5e_rt_controller::kNumRobotJoints> cached_velocities_{};

  mutable std::mutex state_mutex_;
  mutable std::mutex target_mutex_;

  // Timing summary flag — set by RT thread, consumed by log thread.
  // Avoids std::string allocation + RCLCPP_INFO on the 500 Hz path.
  std::atomic<bool> print_timing_summary_{false};

  // Atomic flags — safe to read without a mutex in the RT thread.
  // Written with release, read with acquire to guarantee visibility ordering.
  std::atomic<bool> state_received_{false};
  std::atomic<bool> target_received_{false};

  rclcpp::Time              last_robot_update_;
  std::chrono::milliseconds robot_timeout_{100};

  // ── Named joint mapping (v5.14.0) ────────────────────────────────────────
  std::vector<std::string> robot_joint_names_;
  std::vector<std::string> hand_motor_names_;
  std::vector<std::string> fingertip_names_;
  // JointState msg의 name 순서 → 내부 인덱스 매핑 (첫 수신 시 빌드)
  std::vector<int> joint_state_reorder_;   // joint_state_reorder_[msg_idx] = internal_idx
  bool joint_state_map_built_{false};

  // Hand motor name → index map (sim hand state 수신 시 이름 기반 reorder)
  // 외부 노드의 hand_motor_names와 내부 hand_motor_names_ 간 매핑
  std::vector<int> hand_state_reorder_;
  bool hand_state_map_built_{false};

  // ── Parameters ────────────────────────────────────────────────────────────
  double      control_rate_{500.0};
  bool        enable_logging_{true};
  bool        enable_estop_{true};

  std::size_t loop_count_{0};

  // ── Initialization timeout ──────────────────────────────────────────────────
  bool     init_complete_{false};
  uint64_t init_wait_ticks_{0};
  uint64_t init_timeout_ticks_{2500};  // default 5s at 500Hz

  // ── Auto-hold position (목표 미입력 시 현재 위치 유지) ─────────────────────
  bool auto_hold_position_{true};

  // Baseline for log timestamps — captured on first ControlLoop() iteration.
  // steady_clock reads CLOCK_MONOTONIC via vDSO (no kernel entry on Linux).
  std::chrono::steady_clock::time_point log_start_time_{};

  // ── Global E-Stop ──────────────────────────────────────────────────────────
  std::atomic<bool> global_estop_{false};
  std::string       estop_reason_;          // NRT only — written in TriggerGlobalEstop

  // ── Per-tick timing & overrun detection ──────────────────────────────────
  // Previous loop start time — used for jitter calculation.
  std::chrono::steady_clock::time_point prev_loop_start_{};
  // Tick budget in µs — computed from control_rate_ in DeclareAndLoadParameters().
  double budget_us_{2000.0};
  // Overrun counter — incremented (relaxed) when t_total > budget. Read by log thread.
  std::atomic<uint64_t> overrun_count_{0};
};
