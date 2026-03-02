// ── Includes: project header first, then ROS2, then C++ stdlib ────────────────
#include "ur5e_rt_controller/controllers/pd_controller.hpp"
#include "ur5e_rt_controller/data_logger.hpp"
#include "ur5e_rt_controller/rt_controller_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// ── CustomController ───────────────────────────────────────────────────────────
//
// 500 Hz position controller node. Reads /joint_states and
// /target_joint_positions, computes PD commands, and publishes to
// /forward_position_controller/commands. A 50 Hz watchdog timer triggers
// E-STOP if robot or hand data becomes stale.
class CustomController : public rclcpp::Node {
 public:
  CustomController()
      : Node("custom_controller"),
        controller_(std::make_unique<urtc::PDController>()),
        logger_(std::make_unique<urtc::DataLogger>("/tmp/ur5e_control_log.csv"))
  {
    DeclareAndLoadParameters();
    CreateSubscriptions();
    CreatePublishers();
    CreateTimers();

    RCLCPP_INFO(get_logger(), "CustomController ready — %.0f Hz, E-STOP: %s",
                control_rate_, enable_estop_ ? "ON" : "OFF");
  }

  ~CustomController() override {
    if (logger_) {
      logger_->Flush();
    }
  }

 private:
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
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          JointStateCallback(std::move(msg));
        });

    target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_joint_positions", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          TargetCallback(std::move(msg));
        });

    hand_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          HandStateCallback(std::move(msg));
        });
  }

  void CreatePublishers() {
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/system/estop_status", 10);
  }

  void CreateTimers() {
    const auto control_period = std::chrono::microseconds(
        static_cast<int>(1'000'000.0 / control_rate_));
    control_timer_ = create_wall_timer(
        control_period, [this]() { ControlLoop(); });

    if (enable_estop_) {
      timeout_timer_ = create_wall_timer(
          20ms, [this]() { CheckTimeouts(); });
    }
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
      state_received_    = true;
    }
  }

  void TargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < urtc::kNumRobotJoints) {
      return;
    }
    {
      std::lock_guard lock(target_mutex_);
      std::copy_n(msg->data.begin(), urtc::kNumRobotJoints,
                  target_positions_.begin());
      target_received_ = true;
    }
    // Forward target to controller via interface.
    controller_->SetRobotTarget(target_positions_);
  }

  void HandStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr /*msg*/) {
    std::lock_guard lock(hand_mutex_);
    last_hand_update_   = now();
    hand_data_received_ = true;
  }

  // ── 50 Hz watchdog (E-STOP) ─────────────────────────────────────────────────
  void CheckTimeouts() {
    const auto now_time = now();
    bool robot_timed_out = false;
    bool hand_timed_out  = false;

    {
      std::lock_guard lock(state_mutex_);
      if (state_received_) {
        robot_timed_out = (now_time - last_robot_update_) > robot_timeout_;
      }
    }
    {
      std::lock_guard lock(hand_mutex_);
      if (hand_data_received_) {
        hand_timed_out = (now_time - last_hand_update_) > hand_timeout_;
      }
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
    if (!state_received_ || !target_received_) {
      return;
    }

    // Snapshot shared state under lock.
    urtc::ControllerState state{};
    {
      std::lock_guard lock(state_mutex_);
      state.robot.positions  = current_positions_;
      state.robot.velocities = current_velocities_;
    }
    {
      std::lock_guard lock(target_mutex_);
      state.robot.dt        = 1.0 / control_rate_;
      state.iteration       = loop_count_;
    }

    const urtc::ControllerOutput output = controller_->Compute(state);

    // Publish command (zero on E-STOP is handled inside PDController::Compute).
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.assign(output.robot_commands.begin(),
                        output.robot_commands.end());
    command_pub_->publish(cmd_msg);

    if (enable_logging_ && logger_) {
      logger_->LogControlData(now().seconds(),
                              state.robot.positions,
                              target_positions_,
                              output.robot_commands);
    }

    ++loop_count_;
    if (loop_count_ % 500 == 0) {
      RCLCPP_DEBUG(get_logger(), "ControlLoop: %zu iterations", loop_count_);
    }
  }

  void PublishEstopStatus(bool estopped) {
    std_msgs::msg::Bool msg;
    msg.data = estopped;
    estop_pub_->publish(msg);
  }

  // ── ROS2 handles ────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  hand_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  estop_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // ── Domain objects ──────────────────────────────────────────────────────────
  std::unique_ptr<urtc::PDController> controller_;
  std::unique_ptr<urtc::DataLogger>   logger_;

  // ── Shared state (guarded by per-domain mutexes) ────────────────────────────
  std::array<double, urtc::kNumRobotJoints> current_positions_{};
  std::array<double, urtc::kNumRobotJoints> current_velocities_{};
  std::array<double, urtc::kNumRobotJoints> target_positions_{};

  mutable std::mutex state_mutex_;
  mutable std::mutex target_mutex_;
  mutable std::mutex hand_mutex_;

  bool state_received_{false};
  bool target_received_{false};
  bool hand_data_received_{false};

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
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomController>());
  rclcpp::shutdown();
  return 0;
}
