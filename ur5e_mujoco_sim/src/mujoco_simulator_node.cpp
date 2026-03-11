// ── Includes: project first, then ROS2, then C++ stdlib ──────────────────────
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"
#include "ur5e_mujoco_sim/ros2_resource_provider.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
// Simulation modes (set via "sim_mode" parameter):
//   "free_run"  — physics advances as fast as possible (default).
//                 Best for algorithm validation and trajectory generation.
//                 NOTE: rt_controller E-STOP must be disabled since the
//                 wall-clock publish interval may exceed robot_timeout_ms.
//   "sync_step" — publishes state, waits for one command, takes one step.
//                 Step latency ≈ Compute() time → direct timing measurement.
//
// Published topics (replacing UR driver):
//   /joint_states          sensor_msgs/JointState      @ sim frequency
//   /hand/joint_states     std_msgs/Float64MultiArray  @ 100 Hz
//   /sim/status            std_msgs/Float64MultiArray  @ 1 Hz
//                          data: [step_count, sim_time_sec, rtf, paused(0/1)]
//
// Subscribed topics (from rt_controller):
//   /forward_position_controller/commands  std_msgs/Float64MultiArray
//   /hand/command                          std_msgs/Float64MultiArray
//
// The rt_controller node can be launched without modification.
// Only the robot_ip / UR driver launch needs to be replaced with this node.
//
class MuJoCoSimulatorNode : public rclcpp::Node {
 public:
  MuJoCoSimulatorNode()
      : Node("mujoco_simulator")
  {
    // Register custom package:// VFS resource provider
    urtc::RegisterRos2ResourceProvider();

    DeclareAndLoadParameters();
    CreateSimulator();
    CreatePublishers();
    CreateSubscriptions();
    CreateTimers();

    // Wire state callback: called from sim thread at control_freq Hz
    sim_->SetStateCallback(
        [this](const std::array<double, 6>& pos,
               const std::array<double, 6>& vel,
               const std::array<double, 6>& eff) {
          PublishJointState(pos, vel, eff);
        });

    sim_->Start();

    const std::string max_rtf_str =
        max_rtf_ > 0.0 ? std::to_string(max_rtf_) + "x" : "unlimited";
    RCLCPP_INFO(get_logger(),
                "MuJoCo simulator ready — model: %s  mode: %s  viewer: %s"
                "  max_rtf: %s",
                model_path_.c_str(), sim_mode_.c_str(),
                enable_viewer_ ? "ON" : "OFF",
                max_rtf_str.c_str());
  }

  ~MuJoCoSimulatorNode() override {
    if (sim_) { sim_->Stop(); }
  }

 private:
  // ── Parameter loading ────────────────────────────────────────────────────────
  void DeclareAndLoadParameters() {
    declare_parameter("model_path",    std::string(""));
    declare_parameter("enable_viewer", true);
    declare_parameter("enable_hand_sim", true);
    declare_parameter("hand_filter_alpha", 0.1);

    // Simulation mode: "free_run" (default) or "sync_step"
    declare_parameter("sim_mode", std::string("free_run"));
    // free_run: publish /joint_states every N steps (1 = every step)
    declare_parameter("publish_decimation", 1);
    // sync_step: max time to wait for a command before using previous (ms)
    declare_parameter("sync_timeout_ms", 50.0);
    // Maximum Real-Time Factor (0.0 = unlimited)
    declare_parameter("max_rtf", 0.0);

    // Initial joint positions (UR5e safe upright pose)
    declare_parameter("initial_joint_positions",
                      std::vector<double>{0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0});

    // Physics timestep validation (0.0 = use XML value without validation)
    declare_parameter("physics_timestep", 0.0);

    // Position servo gains
    // false: XML 원본 gainprm/biasprm, true: servo_kp/kd 기반 YAML gain
    declare_parameter("use_yaml_servo_gains", false);
    declare_parameter("servo_kp",
                      std::vector<double>{500.0, 500.0, 500.0, 150.0, 150.0, 150.0});
    declare_parameter("servo_kd",
                      std::vector<double>{400.0, 400.0, 400.0, 100.0, 100.0, 100.0});

    model_path_          = get_parameter("model_path").as_string();
    enable_viewer_       = get_parameter("enable_viewer").as_bool();
    enable_hand_sim_     = get_parameter("enable_hand_sim").as_bool();
    hand_filter_alpha_   = get_parameter("hand_filter_alpha").as_double();
    sim_mode_            = get_parameter("sim_mode").as_string();
    publish_decimation_  = static_cast<int>(get_parameter("publish_decimation").as_int());
    sync_timeout_ms_     = get_parameter("sync_timeout_ms").as_double();
    max_rtf_             = get_parameter("max_rtf").as_double();
    physics_timestep_    = get_parameter("physics_timestep").as_double();
    use_yaml_servo_gains_ = get_parameter("use_yaml_servo_gains").as_bool();

    const auto kp_vec = get_parameter("servo_kp").as_double_array();
    const auto kd_vec = get_parameter("servo_kd").as_double_array();
    for (std::size_t i = 0; i < 6 && i < kp_vec.size(); ++i) {
      servo_kp_[i] = kp_vec[i];
    }
    for (std::size_t i = 0; i < 6 && i < kd_vec.size(); ++i) {
      servo_kd_[i] = kd_vec[i];
    }

    // Resolve model path: if empty, default to package:// URI
    if (model_path_.empty()) {
      model_path_ = "package://ur5e_description/robots/ur5e/mjcf/scene.xml";
    } else if (model_path_.find("package://") != 0 && model_path_[0] != '/') {
      // If someone passed a relative path, assume it's relative to ur5e_description
      model_path_ = "package://ur5e_description/" + model_path_;
    }

    const auto init_vec =
        get_parameter("initial_joint_positions").as_double_array();
    for (std::size_t i = 0; i < 6 && i < init_vec.size(); ++i) {
      initial_qpos_[i] = init_vec[i];
    }
  }

  // ── Simulator creation ───────────────────────────────────────────────────────
  void CreateSimulator() {
    const urtc::MuJoCoSimulator::SimMode mode =
        (sim_mode_ == "sync_step")
            ? urtc::MuJoCoSimulator::SimMode::kSyncStep
            : urtc::MuJoCoSimulator::SimMode::kFreeRun;

    if (sim_mode_ != "free_run" && sim_mode_ != "sync_step") {
      RCLCPP_WARN(get_logger(),
                  "Unknown sim_mode '%s' — defaulting to 'free_run'",
                  sim_mode_.c_str());
    }

    urtc::MuJoCoSimulator::Config cfg{
        .model_path          = model_path_,
        .mode                = mode,
        .enable_viewer       = enable_viewer_,
        .publish_decimation  = publish_decimation_,
        .sync_timeout_ms     = sync_timeout_ms_,
        .max_rtf             = max_rtf_,
        .initial_qpos        = initial_qpos_,
        .physics_timestep    = physics_timestep_,
        .use_yaml_servo_gains = use_yaml_servo_gains_,
        .servo_kp            = servo_kp_,
        .servo_kd            = servo_kd_,
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
    // /joint_states: mimics UR driver output consumed by rt_controller
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10));

    // /hand/joint_states: 11 DOF hand state (simulated via low-pass filter)
    hand_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", rclcpp::QoS(10));

    // /sim/status: [step_count, sim_time_sec, rtf, paused(0/1)]
    sim_status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/sim/status", rclcpp::QoS(10));
  }

  // ── Subscriptions ────────────────────────────────────────────────────────────
  void CreateSubscriptions() {
    // Receive position commands from rt_controller
    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          CommandCallback(msg);
        });

    // Receive torque commands from rt_controller (direct torque controllers)
    torque_command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_torque_controller/commands",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          TorqueCommandCallback(msg);
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

  // Called when rt_controller publishes a position command.
  void CommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Command message has %zu elements (expected 6) — ignored",
                           msg->data.size());
      return;
    }
    // Auto-switch to position servo mode on first position command after torque.
    if (sim_->IsInTorqueMode()) {
      sim_->SetControlMode(false);
      RCLCPP_INFO(get_logger(), "Actuator mode → position servo");
    }
    std::array<double, 6> cmd{};
    std::copy_n(msg->data.begin(), 6, cmd.begin());
    sim_->SetCommand(cmd);
  }

  // Called when rt_controller publishes a torque command (direct torque controllers).
  void TorqueCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Torque command message has %zu elements (expected 6) — ignored",
                           msg->data.size());
      return;
    }
    // Auto-switch to direct torque mode on first torque command.
    if (!sim_->IsInTorqueMode()) {
      sim_->SetControlMode(true);
      RCLCPP_INFO(get_logger(), "Actuator mode → direct torque");
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
                         const std::array<double, 6>& velocities,
                         const std::array<double, 6>& efforts) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name         = {kJointNames.begin(), kJointNames.end()};
    msg.position     = {positions.begin(),   positions.end()};
    msg.velocity     = {velocities.begin(),  velocities.end()};
    msg.effort       = {efforts.begin(),     efforts.end()};
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
    const bool   is_running = sim_->IsRunning();
    const bool   is_paused  = sim_->IsPaused();
    const double rtf        = sim_->GetRtf();
    const double sim_time   = sim_->SimTimeSec();
    const auto   steps      = sim_->StepCount();

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {
        static_cast<double>(steps),  // [0] step count
        sim_time,                    // [1] sim time (s)
        rtf,                         // [2] real-time factor
        is_paused ? 1.0 : 0.0,      // [3] paused flag
    };
    sim_status_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "Simulator running=%s%s  steps=%lu  sim_time=%.2f s  rtf=%.1fx",
                is_running ? "true" : "false",
                is_paused  ? " [PAUSED]" : "",
                static_cast<unsigned long>(steps),
                sim_time,
                rtf);
  }

  // ── Joint name table (must match UR driver / rt_controller) ──────────────
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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   sim_status_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_sub_;
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
  bool                   enable_viewer_{true};
  bool                   enable_hand_sim_{true};
  double                 hand_filter_alpha_{0.1};
  std::string            sim_mode_{"free_run"};
  int                    publish_decimation_{1};
  double                 sync_timeout_ms_{50.0};
  double                 max_rtf_{0.0};
  std::array<double, 6>  initial_qpos_{0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
  double                 physics_timestep_{0.0};
  bool                   use_yaml_servo_gains_{false};
  std::array<double, 6>  servo_kp_{500.0, 500.0, 500.0, 150.0, 150.0, 150.0};
  std::array<double, 6>  servo_kd_{400.0, 400.0, 400.0, 100.0, 100.0, 100.0};
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
