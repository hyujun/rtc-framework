// ── Includes: project first, then ROS2, then C++ stdlib ──────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "rtc_mujoco_sim/ros2_resource_provider.hpp"
#include <rtc_base/types/types.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rtc_msgs/msg/joint_command.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;
namespace urtc = rtc;

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
//   <robot_topics.state_topic>   sensor_msgs/JointState      @ sim frequency
//   <fake_hand_response.state_topic>  std_msgs/Float64MultiArray  @ 100 Hz
//   /sim/status                  std_msgs/Float64MultiArray  @ 1 Hz
//
// Subscribed topics (from rt_controller):
//   <robot_topics.command_topic>  ur5e_msgs/JointCommand (command_type으로 position/torque 결정)
//   <fake_hand_response.command_topic>  std_msgs/Float64MultiArray
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
        [this](const std::vector<double>& pos,
               const std::vector<double>& vel,
               const std::vector<double>& eff) {
          PublishJointState(pos, vel, eff);
        });

    sim_->Start();

    const std::string max_rtf_str =
        max_rtf_ > 0.0 ? std::to_string(max_rtf_) + "x" : "unlimited";
    RCLCPP_INFO(get_logger(),
                "MuJoCo simulator ready — model: %s  mode: %s  viewer: %s"
                "  max_rtf: %s  robot_joints: %d",
                model_path_.c_str(), sim_mode_.c_str(),
                enable_viewer_ ? "ON" : "OFF",
                max_rtf_str.c_str(),
                sim_->NumRobotJoints());
  }

  ~MuJoCoSimulatorNode() override {
    if (sim_) { sim_->Stop(); }
  }

 private:
  // ── Parameter loading ────────────────────────────────────────────────────────
  void DeclareAndLoadParameters() {
    declare_parameter("model_path",    std::string(""));
    declare_parameter("enable_viewer", true);

    // Fake hand response (시뮬레이션 전용 핸드 LPF)
    declare_parameter("fake_hand_response.enable", true);
    declare_parameter("fake_hand_response.filter_alpha", 0.1);
    declare_parameter("fake_hand_response.command_topic", std::string("/hand/command"));
    declare_parameter("fake_hand_response.state_topic", std::string("/hand/joint_states"));

    // Hand motor names (이름 기반 매핑용 — 빈 배열이면 기본값 사용)
    declare_parameter("hand_motor_names", std::vector<std::string>{});

    // Robot topics (fake_hand_response 패턴과 동일)
    declare_parameter("robot_topics.command_topic",
                      std::string("/ur5e/joint_command"));
    declare_parameter("robot_topics.state_topic",
                      std::string("/joint_states"));

    // Simulation mode: "free_run" (default) or "sync_step"
    declare_parameter("sim_mode", std::string("free_run"));
    // free_run: publish /joint_states every N steps (1 = every step)
    declare_parameter("publish_decimation", 1);
    // sync_step: max time to wait for a command before using previous (ms)
    declare_parameter("sync_timeout_ms", 50.0);
    // Maximum Real-Time Factor (0.0 = unlimited)
    declare_parameter("max_rtf", 0.0);

    // control_rate (launch에서 rt_controller config로부터 전달)
    declare_parameter("control_rate", 500.0);

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
    fake_hand_enable_    = get_parameter("fake_hand_response.enable").as_bool();
    fake_hand_alpha_     = get_parameter("fake_hand_response.filter_alpha").as_double();
    fake_hand_cmd_topic_ = get_parameter("fake_hand_response.command_topic").as_string();
    fake_hand_state_topic_ = get_parameter("fake_hand_response.state_topic").as_string();
    robot_command_topic_ = get_parameter("robot_topics.command_topic").as_string();
    robot_state_topic_   = get_parameter("robot_topics.state_topic").as_string();
    sim_mode_            = get_parameter("sim_mode").as_string();
    publish_decimation_  = static_cast<int>(get_parameter("publish_decimation").as_int());
    sync_timeout_ms_     = get_parameter("sync_timeout_ms").as_double();
    max_rtf_             = get_parameter("max_rtf").as_double();
    control_rate_        = get_parameter("control_rate").as_double();
    physics_timestep_    = get_parameter("physics_timestep").as_double();
    use_yaml_servo_gains_ = get_parameter("use_yaml_servo_gains").as_bool();

    servo_kp_ = get_parameter("servo_kp").as_double_array();
    servo_kd_ = get_parameter("servo_kd").as_double_array();

    // Hand motor names (이름 기반 순서 보장)
    hand_motor_names_ = get_parameter("hand_motor_names").as_string_array();
    if (hand_motor_names_.empty()) {
      hand_motor_names_ = urtc::kDefaultHandMotorNames;
    }
    if (fake_hand_enable_) {
      std::string names_str;
      for (std::size_t i = 0; i < hand_motor_names_.size(); ++i) {
        if (i > 0) names_str += ", ";
        names_str += hand_motor_names_[i];
      }
      RCLCPP_INFO(get_logger(), "Hand motor names (%zu): [%s]",
                  hand_motor_names_.size(), names_str.c_str());
    }

    // Resolve model path: if empty, default to package:// URI
    if (model_path_.empty()) {
      model_path_ = "package://ur5e_description/robots/ur5e/mjcf/scene.xml";
    } else if (model_path_.find("package://") != 0 && model_path_[0] != '/') {
      // If someone passed a relative path, assume it's relative to ur5e_description
      model_path_ = "package://ur5e_description/" + model_path_;
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
        .physics_timestep    = physics_timestep_,
        .use_yaml_servo_gains = use_yaml_servo_gains_,
        .servo_kp            = servo_kp_,
        .servo_kd            = servo_kd_,
        .command_topic       = robot_command_topic_,
        .state_topic         = robot_state_topic_,
    };
    sim_ = std::make_unique<urtc::MuJoCoSimulator>(std::move(cfg));

    if (!sim_->Initialize()) {
      RCLCPP_FATAL(get_logger(),
                   "Failed to load MuJoCo model from '%s'. "
                   "Check the model_path parameter.",
                   model_path_.c_str());
      throw std::runtime_error("MuJoCo model load failed");
    }

    // ── physics_timestep vs control_rate 검증 ───────────────────────────────
    // 물리 스텝 주파수가 제어 주파수보다 낮으면 시뮬레이션 의미 없음
    const double xml_dt = sim_->GetPhysicsTimestep();
    if (xml_dt > 0.0 && control_rate_ > 0.0) {
      const double physics_freq = 1.0 / xml_dt;
      if (physics_freq < control_rate_) {
        RCLCPP_FATAL(get_logger(),
            "Physics frequency (%.1f Hz = 1/%.4fs) < control_rate (%.1f Hz). "
            "MuJoCo cannot step fast enough for the controller. "
            "Decrease physics_timestep or decrease control_rate.",
            physics_freq, xml_dt, control_rate_);
        throw std::runtime_error("physics_timestep vs control_rate mismatch");
      }
      RCLCPP_INFO(get_logger(),
          "Physics vs control rate OK: physics=%.1f Hz >= control=%.1f Hz",
          physics_freq, control_rate_);
    }

    // Build joint name→index map from XML-discovered joints
    BuildJointNameIndexMap();
  }

  // ── Publishers ───────────────────────────────────────────────────────────────
  void CreatePublishers() {
    // Robot state: mimics UR driver output consumed by rt_controller
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        robot_state_topic_, rclcpp::QoS(10));

    // hand state (simulated via low-pass filter) — 토픽명 YAML 설정
    if (fake_hand_enable_) {
      hand_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          fake_hand_state_topic_, rclcpp::QoS(10));
    }

    // /sim/status: [step_count, sim_time_sec, rtf, paused(0/1)]
    sim_status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/sim/status", rclcpp::QoS(10));
  }

  // ── Subscriptions ────────────────────────────────────────────────────────────
  void CreateSubscriptions() {
    // Use BEST_EFFORT to match the rt_controller publisher QoS (BEST_EFFORT + depth 1).
    rclcpp::QoS cmd_qos{10};
    cmd_qos.best_effort();

    // 단일 command subscription: JointCommand msg (command_type으로 position/torque 결정)
    command_sub_ = create_subscription<rtc_msgs::msg::JointCommand>(
        robot_command_topic_,
        cmd_qos,
        [this](const rtc_msgs::msg::JointCommand::SharedPtr msg) {
          JointCommandCallback(msg);
        });

    // Receive normalized hand commands (0.0–1.0) — drives the hand filter
    if (fake_hand_enable_) {
      hand_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
          fake_hand_cmd_topic_,
          rclcpp::QoS(10),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            HandCommandCallback(msg);
          });
    }
  }

  // ── Timers ───────────────────────────────────────────────────────────────────
  void CreateTimers() {
    // Hand joint state publisher @ 100 Hz (fake response)
    if (fake_hand_enable_) {
      hand_pub_timer_ = create_wall_timer(
          10ms, [this]() { PublishHandState(); });
    }

    // Simulation status @ 1 Hz
    status_timer_ = create_wall_timer(
        1s, [this]() { PublishSimStatus(); });
  }

  // ── Subscription callbacks ───────────────────────────────────────────────────

  // Called when a hand command is received.
  // The command updates the target for the low-pass filter.
  void HandCommandCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 10) { return; }
    std::lock_guard lock(hand_mutex_);
    std::copy_n(msg->data.begin(), 10, hand_target_.begin());
  }

  // Called when rt_controller publishes a JointCommand.
  // command_type으로 position/torque 자동 전환.
  void JointCommandCallback(const rtc_msgs::msg::JointCommand::SharedPtr msg) {
    if (msg->values.empty()) { return; }

    const int nj = sim_->NumRobotJoints();
    const bool is_torque = (msg->command_type == "torque");

    // 이름 기반 매핑: joint_names가 비어있으면 positional fallback
    std::vector<double> cmd(static_cast<std::size_t>(nj), 0.0);
    if (msg->joint_names.empty()) {
      // Positional fallback
      const std::size_t n = std::min(msg->values.size(),
                                     static_cast<std::size_t>(nj));
      std::copy_n(msg->values.begin(), n, cmd.begin());
    } else {
      // 첫 수신 시 command message의 joint_names와 XML 이름 비교 검증
      if (!joint_indices_resolved_from_msg_) {
        const std::vector<std::string> names(
            msg->joint_names.begin(), msg->joint_names.end());
        if (sim_->ResolveJointIndices(names)) {
          RCLCPP_INFO(get_logger(),
              "ResolveJointIndices updated from JointCommand message (%zu joints)",
              names.size());
        } else {
          RCLCPP_WARN(get_logger(),
              "ResolveJointIndices from JointCommand: some names not found in XML");
        }
        // name→index 맵도 갱신
        joint_name_index_map_.clear();
        BuildJointNameIndexMap();
        joint_indices_resolved_from_msg_ = true;
      }
      for (std::size_t i = 0; i < msg->joint_names.size() && i < msg->values.size(); ++i) {
        auto it = joint_name_index_map_.find(msg->joint_names[i]);
        if (it != joint_name_index_map_.end()) {
          cmd[it->second] = msg->values[i];
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Unknown joint name in JointCommand: '%s'",
                               msg->joint_names[i].c_str());
        }
      }
    }

    if (is_torque) {
      if (!sim_->IsInTorqueMode()) {
        sim_->SetControlMode(true);
        RCLCPP_INFO(get_logger(), "Actuator mode → direct torque (via JointCommand)");
      }
    } else {
      if (sim_->IsInTorqueMode()) {
        sim_->SetControlMode(false);
        RCLCPP_INFO(get_logger(), "Actuator mode → position servo (via JointCommand)");
      }
      sim_->EnforcePositionServoGravity();
    }
    sim_->SetCommand(cmd);
  }

  void BuildJointNameIndexMap() {
    const auto& names = sim_->GetJointNames();
    for (std::size_t i = 0; i < names.size(); ++i) {
      joint_name_index_map_[names[i]] = i;
    }
    RCLCPP_INFO(get_logger(), "Built joint name→index map (%zu joints)",
                names.size());
  }

  // ── Publisher helpers ────────────────────────────────────────────────────────

  // Called from SimLoop thread at control_freq Hz.
  // rclcpp::Publisher::publish() is thread-safe in ROS2 Humble.
  void PublishJointState(const std::vector<double>& positions,
                         const std::vector<double>& velocities,
                         const std::vector<double>& efforts) {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name         = sim_->GetJointNames();
    msg.position     = positions;
    msg.velocity     = velocities;
    msg.effort       = efforts;
    joint_state_pub_->publish(msg);
  }

  // Called by hand_pub_timer_ at 100 Hz.
  // Advances a 1st-order low-pass filter toward hand_target_.
  void PublishHandState() {
    {
      std::lock_guard lock(hand_mutex_);
      for (std::size_t i = 0; i < 10; ++i) {
        hand_state_[i] +=
            fake_hand_alpha_ * (hand_target_[i] - hand_state_[i]);
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

  // ── ROS2 handles ─────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   hand_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   sim_status_pub_;

  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr    command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hand_cmd_sub_;

  rclcpp::TimerBase::SharedPtr hand_pub_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // ── Domain objects ────────────────────────────────────────────────────────────
  std::unique_ptr<urtc::MuJoCoSimulator> sim_;

  // ── Hand simulation state (low-pass filter) ───────────────────────────────────
  mutable std::mutex            hand_mutex_;
  std::array<double, 10>        hand_state_{};   // current filtered state
  std::array<double, 10>        hand_target_{};  // target from /hand/command

  // ── Parameters ───────────────────────────────────────────────────────────────
  std::string            model_path_;
  bool                   enable_viewer_{true};
  bool                   fake_hand_enable_{true};
  double                 fake_hand_alpha_{0.1};
  std::string            fake_hand_cmd_topic_{"/hand/command"};
  std::string            fake_hand_state_topic_{"/hand/joint_states"};
  std::vector<std::string> hand_motor_names_;
  std::string            robot_command_topic_{"/ur5e/joint_command"};
  std::string            robot_state_topic_{"/joint_states"};
  std::string            sim_mode_{"free_run"};
  int                    publish_decimation_{1};
  double                 sync_timeout_ms_{50.0};
  double                 max_rtf_{0.0};
  double                 control_rate_{500.0};
  double                 physics_timestep_{0.0};
  bool                   use_yaml_servo_gains_{false};
  std::vector<double>    servo_kp_{500.0, 500.0, 500.0, 150.0, 150.0, 150.0};
  std::vector<double>    servo_kd_{400.0, 400.0, 400.0, 100.0, 100.0, 100.0};

  // ── Named joint mapping ───────────────────────────────────────────────────────
  std::unordered_map<std::string, std::size_t> joint_name_index_map_;
  bool joint_indices_resolved_from_msg_{false};
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
