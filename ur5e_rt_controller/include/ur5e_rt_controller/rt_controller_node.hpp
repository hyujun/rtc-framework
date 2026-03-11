#pragma once

// ── Project headers ───────────────────────────────────────────────────────────
#include "ur5e_rt_base/data_logger.hpp"
#include "ur5e_rt_base/log_buffer.hpp"
#include "ur5e_rt_controller/controller_timing_profiler.hpp"
#include "ur5e_rt_controller/rt_controller_interface.hpp"

// ── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

// ── C++ stdlib ────────────────────────────────────────────────────────────────
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
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
  // ── Log file helpers ──────────────────────────────────────────────────────
  // Returns "<log_dir>/ur5e_control_log_YYMMDD_HHMM.csv"
  static std::string GenerateLogFilePath(const std::string & log_dir);
  // Removes oldest matching log files when count exceeds max_files.
  static void CleanupOldLogFiles(const std::filesystem::path & log_dir, int max_files);

  // ── Initialisation helpers ────────────────────────────────────────────────
  void CreateCallbackGroups();
  void DeclareAndLoadParameters();
  void CreateSubscriptions();
  void CreatePublishers();
  void CreateTimers();

  // ── Subscription callbacks ────────────────────────────────────────────────
  void JointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg);
  void TargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void HandStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ── Timer callbacks ───────────────────────────────────────────────────────
  void CheckTimeouts();   // 50 Hz watchdog (E-STOP)
  void ControlLoop();     // 500 Hz control loop
  void DrainLog();        // Log drain (non-RT core)

  void PublishEstopStatus(bool estopped);

  // ── ROS2 handles ──────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_rt_;
  rclcpp::CallbackGroup::SharedPtr cb_group_sensor_;
  rclcpp::CallbackGroup::SharedPtr cb_group_log_;
  rclcpp::CallbackGroup::SharedPtr cb_group_aux_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  hand_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr              controller_selector_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  controller_gains_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  std_msgs::msg::Float64MultiArray                               cmd_msg_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;
  std_msgs::msg::Float64MultiArray                               torque_cmd_msg_;
  std::mutex                                                     cmd_pub_mutex_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr task_pos_pub_;
  std_msgs::msg::Float64MultiArray                               task_pos_msg_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            active_ctrl_name_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr drain_timer_;  // log drain (log thread)

  // ── Domain objects ────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<ur5e_rt_controller::RTControllerInterface>> controllers_;
  std::atomic<int> active_controller_idx_{1};
  std::unique_ptr<ur5e_rt_controller::DataLogger> logger_;
  ur5e_rt_controller::ControlLogBuffer              log_buffer_{};              // SPSC ring buffer
  ur5e_rt_controller::ControllerTimingProfiler      timing_profiler_{};         // Compute() timing

  // ── Shared state (guarded by per-domain mutexes) ──────────────────────────
  std::array<double, ur5e_rt_controller::kNumRobotJoints> current_positions_{};
  std::array<double, ur5e_rt_controller::kNumRobotJoints> current_velocities_{};
  std::array<double, ur5e_rt_controller::kNumRobotJoints> target_positions_{};
  // RT-local snapshot of target — written and read only in ControlLoop()
  std::array<double, ur5e_rt_controller::kNumRobotJoints> target_snapshot_{};

  // RT-local cached copies — updated via try_lock to avoid blocking the RT
  // thread.  If the mutex is contended, the previous cycle's data is reused
  // (stale by at most one cycle = 2 ms, acceptable for position control).
  std::array<double, ur5e_rt_controller::kNumRobotJoints> cached_positions_{};
  std::array<double, ur5e_rt_controller::kNumRobotJoints> cached_velocities_{};

  mutable std::mutex state_mutex_;
  mutable std::mutex target_mutex_;
  mutable std::mutex hand_mutex_;

  // Timing summary flag — set by RT thread, consumed by log thread.
  // Avoids std::string allocation + RCLCPP_INFO on the 500 Hz path.
  std::atomic<bool> print_timing_summary_{false};

  // Atomic flags — safe to read without a mutex in the RT thread.
  // Written with release, read with acquire to guarantee visibility ordering.
  std::atomic<bool> state_received_{false};
  std::atomic<bool> target_received_{false};
  std::atomic<bool> hand_data_received_{false};

  rclcpp::Time              last_robot_update_;
  rclcpp::Time              last_hand_update_;
  std::chrono::milliseconds robot_timeout_{100};
  std::chrono::milliseconds hand_timeout_{200};

  // ── Parameters ────────────────────────────────────────────────────────────
  double      control_rate_{500.0};
  bool        enable_logging_{true};
  bool        enable_estop_{true};
  bool        hand_estop_logged_{false};

  std::size_t loop_count_{0};
};
