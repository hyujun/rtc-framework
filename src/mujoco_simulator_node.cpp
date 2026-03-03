// ── Includes: project first, then ROS2, then C++ stdlib ──────────────────────
#include "ur5e_rt_controller/mujoco_simulator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// ── MuJoCoSimulatorNode ────────────────────────────────────────────────────────
//
// ROS2 node that drives a MuJoCo physics simulation of the UR5e and
// presents the same topic interface as the real UR driver.
//
// Published topics (replacing UR driver):
//   /joint_states          sensor_msgs/JointState      @ control_freq Hz
//   /hand/joint_states     std_msgs/Float64MultiArray  @ 100 Hz
//
// Subscribed topics (from custom_controller):
//   /forward_position_controller/commands  std_msgs/Float64MultiArray
//   /hand/command                          std_msgs/Float64MultiArray
//
// The custom_controller node can be launched without modification.
// Only the robot_ip / UR driver launch needs to be replaced with this node.
//
class MuJoCoSimulatorNode : public rclcpp::Node {
 public:
  MuJoCoSimulatorNode()
      : Node("mujoco_simulator")
  {
    DeclareAndLoadParameters();
    CreateSimulator();
    CreatePublishers();
    CreateSubscriptions();
    CreateTimers();

    // Wire state callback: called from sim thread at control_freq Hz
    sim_->SetStateCallback(
        [this](const std::array<double, 6>& pos,
               const std::array<double, 6>& vel) {
          PublishJointState(pos, vel);
        });

    sim_->Start();

    RCLCPP_INFO(get_logger(),
                "MuJoCo simulator ready — model: %s  freq: %.0f Hz  viewer: %s",
                model_path_.c_str(), control_freq_,
                enable_viewer_ ? "ON" : "OFF");
  }

  ~MuJoCoSimulatorNode() override {
    if (sim_) { sim_->Stop(); }
  }

 private:
  // ── Parameter loading ────────────────────────────────────────────────────────
  void DeclareAndLoadParameters() {
    declare_parameter("model_path",    std::string(""));
    declare_parameter("control_freq",  500.0);
    declare_parameter("enable_viewer", true);
    declare_parameter("realtime",      true);
    declare_parameter("sim_speed",     1.0);
    declare_parameter("enable_hand_sim", true);
    declare_parameter("hand_filter_alpha", 0.1);

    // Initial joint positions (UR5e safe pose)
    declare_parameter("initial_joint_positions",
                      std::vector<double>{0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0});

    model_path_         = get_parameter("model_path").as_string();
    control_freq_       = get_parameter("control_freq").as_double();
    enable_viewer_      = get_parameter("enable_viewer").as_bool();
    realtime_           = get_parameter("realtime").as_bool();
    sim_speed_          = get_parameter("sim_speed").as_double();
    enable_hand_sim_    = get_parameter("enable_hand_sim").as_bool();
    hand_filter_alpha_  = get_parameter("hand_filter_alpha").as_double();

    // Resolve model path: if empty or relative, resolve via ament
    if (model_path_.empty() || model_path_[0] != '/') {
      const std::string share_dir =
          ament_index_cpp::get_package_share_directory("ur5e_rt_controller");
      const std::string rel = model_path_.empty()
                                  ? "models/ur5e/scene.xml"
                                  : model_path_;
      model_path_ = share_dir + "/" + rel;
    }

    const auto init_vec =
        get_parameter("initial_joint_positions").as_double_array();
    for (std::size_t i = 0; i < 6 && i < init_vec.size(); ++i) {
      initial_positions_[i] = init_vec[i];
    }
  }

  // ── Simulator creation ───────────────────────────────────────────────────────
  void CreateSimulator() {
    urtc::MuJoCoSimulator::Config cfg{
        .model_path       = model_path_,
        .control_freq     = control_freq_,
        .enable_viewer    = enable_viewer_,
        .realtime         = realtime_,
        .sim_speed        = sim_speed_,
        .initial_positions = initial_positions_,
    };
    sim_ = std::make_unique<urtc::MuJoCoSimulator>(std::move(cfg));

    if (!sim_->Initialize()) {
      RCLCPP_FATAL(get_logger(),
                   "Failed to load MuJoCo model from '%s'. "
                   "Check the model_path parameter.",
                   model_path_.c_str());
      throw std::runtime_error("MuJoCo model load failed");
    }
  }

  // ── Publishers ───────────────────────────────────────────────────────────────
  void CreatePublishers() {
    // /joint_states: mimics UR driver output consumed by custom_controller
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10));

    // /hand/joint_states: 11 DOF hand state (simulated via low-pass filter)
    hand_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", rclcpp::QoS(10));

    // /sim/estop_status: re-publish whether sim is running (for monitoring)
    sim_status_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/sim/status", rclcpp::QoS(10));
  }

  // ── Subscriptions ────────────────────────────────────────────────────────────
  void CreateSubscriptions() {
    // Receive position commands from custom_controller
    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          CommandCallback(msg);
        });

    // Receive normalized hand commands (0.0–1.0) — drives the hand filter
    hand_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/command",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          HandCommandCallback(msg);
        });
  }

  // ── Timers ───────────────────────────────────────────────────────────────────
  void CreateTimers() {
    // Hand joint state publisher @ 100 Hz
    if (enable_hand_sim_) {
      hand_pub_timer_ = create_wall_timer(
          10ms, [this]() { PublishHandState(); });
    }

    // Simulation status @ 1 Hz
    status_timer_ = create_wall_timer(
        1s, [this]() { PublishSimStatus(); });
  }

  // ── Subscription callbacks ───────────────────────────────────────────────────

  // Called when custom_controller publishes a position command.
  void CommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Command message has %zu elements (expected 6) — ignored",
                           msg->data.size());
      return;
    }
    std::array<double, 6> cmd{};
    std::copy_n(msg->data.begin(), 6, cmd.begin());
    sim_->SetCommand(cmd);
  }

  // Called when a hand command is received.
  // The command updates the target for the low-pass filter.
  void HandCommandCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 11) { return; }
    std::lock_guard lock(hand_mutex_);
    std::copy_n(msg->data.begin(), 11, hand_target_.begin());
  }

  // ── Publisher helpers ────────────────────────────────────────────────────────

  // Called from SimLoop thread at control_freq Hz.
  // rclcpp::Publisher::publish() is thread-safe in ROS2 Humble.
  void PublishJointState(const std::array<double, 6>& positions,
                         const std::array<double, 6>& velocities) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name         = {kJointNames.begin(), kJointNames.end()};
    msg.position     = {positions.begin(),   positions.end()};
    msg.velocity     = {velocities.begin(),  velocities.end()};
    // Effort not simulated; fill with zeros
    msg.effort.assign(6, 0.0);
    joint_state_pub_->publish(msg);
  }

  // Called by hand_pub_timer_ at 100 Hz.
  // Advances a 1st-order low-pass filter toward hand_target_.
  void PublishHandState() {
    {
      std::lock_guard lock(hand_mutex_);
      for (std::size_t i = 0; i < 11; ++i) {
        hand_state_[i] +=
            hand_filter_alpha_ * (hand_target_[i] - hand_state_[i]);
      }
    }

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(hand_state_.begin(), hand_state_.end());
    hand_state_pub_->publish(msg);
  }

  void PublishSimStatus() {
    auto msg   = std_msgs::msg::Bool();
    msg.data   = sim_->IsRunning();
    sim_status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "Simulator running=%s  steps=%lu  (%.1f Hz)",
                sim_->IsRunning() ? "true" : "false",
                static_cast<unsigned long>(sim_->StepCount()),
                control_freq_);
  }

  // ── Joint name table (must match UR driver / custom_controller) ──────────────
  static constexpr std::array<const char*, 6> kJointNames = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint",
  };

  // ── ROS2 handles ─────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   hand_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                sim_status_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hand_cmd_sub_;

  rclcpp::TimerBase::SharedPtr hand_pub_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // ── Domain objects ────────────────────────────────────────────────────────────
  std::unique_ptr<urtc::MuJoCoSimulator> sim_;

  // ── Hand simulation state (low-pass filter) ───────────────────────────────────
  mutable std::mutex            hand_mutex_;
  std::array<double, 11>        hand_state_{};   // current filtered state
  std::array<double, 11>        hand_target_{};  // target from /hand/command

  // ── Parameters ───────────────────────────────────────────────────────────────
  std::string            model_path_;
  double                 control_freq_{500.0};
  bool                   enable_viewer_{true};
  bool                   realtime_{true};
  double                 sim_speed_{1.0};
  bool                   enable_hand_sim_{true};
  double                 hand_filter_alpha_{0.1};
  std::array<double, 6>  initial_positions_{0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
};

// ── Entry point ────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MuJoCoSimulatorNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "[mujoco_simulator_node] Fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
