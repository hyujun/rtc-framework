// ── mujoco_simulator_node.cpp ─────────────────────────────────────────────────
// ROS2 node for MuJoCo simulation with multi-group support.
// robot_response groups use MuJoCo physics; fake_response groups use LPF.
// All groups use rtc_msgs/JointCommand for commands, sensor_msgs/JointState
// for state publishing.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "rtc_mujoco_sim/ros2_resource_provider.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rtc_msgs/msg/joint_command.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;
namespace urtc = rtc;

// ── GroupRosHandles ──────────────────────────────────────────────────────────────

struct GroupRosHandles {
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr cmd_sub;
  rclcpp::TimerBase::SharedPtr fake_timer;  // fake groups only
  std::size_t group_idx{0};
  std::unordered_map<std::string, std::size_t> name_index_map;
  bool indices_resolved{false};
};

// ── MuJoCoSimulatorNode ────────────────────────────────────────────────────────

class MuJoCoSimulatorNode : public rclcpp::Node {
 public:
  MuJoCoSimulatorNode()
      : Node("mujoco_simulator")
  {
    urtc::RegisterRos2ResourceProvider();

    DeclareAndLoadParameters();
    CreateSimulator();
    CreateGroupHandles();
    CreateTimers();

    sim_->Start();

    RCLCPP_INFO(get_logger(),
                "MuJoCo simulator ready — model: %s  mode: %s  viewer: %s  groups: %zu",
                model_path_.c_str(), sim_mode_.c_str(),
                enable_viewer_ ? "ON" : "OFF",
                sim_->NumGroups());
  }

  ~MuJoCoSimulatorNode() override {
    if (sim_) { sim_->Stop(); }
  }

 private:
  // ── Parameter loading ────────────────────────────────────────────────────────
  void DeclareAndLoadParameters() {
    declare_parameter("model_path", std::string(""));
    declare_parameter("enable_viewer", true);
    declare_parameter("sim_mode", std::string("free_run"));
    declare_parameter("publish_decimation", 1);
    declare_parameter("sync_timeout_ms", 50.0);
    declare_parameter("max_rtf", 0.0);
    declare_parameter("control_rate", 500.0);
    declare_parameter("physics_timestep", 0.0);
    declare_parameter("use_yaml_servo_gains", false);
    declare_parameter("servo_kp",
                      std::vector<double>{500.0, 500.0, 500.0, 150.0, 150.0, 150.0});
    declare_parameter("servo_kd",
                      std::vector<double>{400.0, 400.0, 400.0, 100.0, 100.0, 100.0});

    // robot_response / fake_response group names
    declare_parameter("robot_response.groups", std::vector<std::string>{});
    declare_parameter("fake_response.groups", std::vector<std::string>{});

    model_path_          = get_parameter("model_path").as_string();
    enable_viewer_       = get_parameter("enable_viewer").as_bool();
    sim_mode_            = get_parameter("sim_mode").as_string();
    publish_decimation_  = static_cast<int>(get_parameter("publish_decimation").as_int());
    sync_timeout_ms_     = get_parameter("sync_timeout_ms").as_double();
    max_rtf_             = get_parameter("max_rtf").as_double();
    control_rate_        = get_parameter("control_rate").as_double();
    physics_timestep_    = get_parameter("physics_timestep").as_double();
    use_yaml_servo_gains_ = get_parameter("use_yaml_servo_gains").as_bool();
    servo_kp_ = get_parameter("servo_kp").as_double_array();
    servo_kd_ = get_parameter("servo_kd").as_double_array();

    // Parse robot_response groups
    auto robot_groups = get_parameter("robot_response.groups").as_string_array();
    for (const auto& gname : robot_groups) {
      DeclareGroupParams("robot_response", gname);
      urtc::JointGroupConfig gc;
      gc.name = gname;
      gc.is_robot = true;
      gc.joint_names = get_parameter("robot_response." + gname + ".joint_names").as_string_array();
      gc.command_topic = get_parameter("robot_response." + gname + ".command_topic").as_string();
      gc.state_topic = get_parameter("robot_response." + gname + ".state_topic").as_string();

      // Optional per-group servo gains
      try {
        gc.servo_kp = get_parameter("robot_response." + gname + ".servo_kp").as_double_array();
        gc.servo_kd = get_parameter("robot_response." + gname + ".servo_kd").as_double_array();
      } catch (...) {}

      group_configs_.push_back(std::move(gc));
    }

    // Parse fake_response groups
    auto fake_groups = get_parameter("fake_response.groups").as_string_array();
    for (const auto& gname : fake_groups) {
      DeclareGroupParams("fake_response", gname);
      declare_parameter("fake_response." + gname + ".filter_alpha", 0.1);

      urtc::JointGroupConfig gc;
      gc.name = gname;
      gc.is_robot = false;
      gc.joint_names = get_parameter("fake_response." + gname + ".joint_names").as_string_array();
      gc.command_topic = get_parameter("fake_response." + gname + ".command_topic").as_string();
      gc.state_topic = get_parameter("fake_response." + gname + ".state_topic").as_string();
      gc.filter_alpha = get_parameter("fake_response." + gname + ".filter_alpha").as_double();

      group_configs_.push_back(std::move(gc));
    }

    // Resolve model path
    if (model_path_.empty()) {
      model_path_ = "package://ur5e_description/robots/ur5e/mjcf/scene.xml";
    } else if (model_path_.find("package://") != 0 && model_path_[0] != '/') {
      model_path_ = "package://ur5e_description/" + model_path_;
    }
  }

  void DeclareGroupParams(const std::string& section, const std::string& gname) {
    const auto prefix = section + "." + gname + ".";
    declare_parameter(prefix + "joint_names", std::vector<std::string>{});
    declare_parameter(prefix + "command_topic", std::string(""));
    declare_parameter(prefix + "state_topic", std::string(""));
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
        .groups              = group_configs_,
    };
    sim_ = std::make_unique<urtc::MuJoCoSimulator>(std::move(cfg));

    if (!sim_->Initialize()) {
      RCLCPP_FATAL(get_logger(),
                   "Failed to initialize MuJoCo simulator from '%s'",
                   model_path_.c_str());
      throw std::runtime_error("MuJoCo initialization failed");
    }

    // Physics vs control rate validation
    const double xml_dt = sim_->GetPhysicsTimestep();
    if (xml_dt > 0.0 && control_rate_ > 0.0) {
      const double physics_freq = 1.0 / xml_dt;
      if (physics_freq < control_rate_) {
        RCLCPP_FATAL(get_logger(),
            "Physics frequency (%.1f Hz) < control_rate (%.1f Hz)",
            physics_freq, control_rate_);
        throw std::runtime_error("physics_timestep vs control_rate mismatch");
      }
    }
  }

  // ── Create per-group publishers, subscribers, callbacks ─────────────────────
  void CreateGroupHandles() {
    const auto num_groups = sim_->NumGroups();
    group_handles_.resize(num_groups);

    rclcpp::QoS cmd_qos{10};
    cmd_qos.best_effort();

    for (std::size_t idx = 0; idx < num_groups; ++idx) {
      auto& h = group_handles_[idx];
      h.group_idx = idx;

      const auto& joint_names = sim_->GetJointNames(idx);
      const bool is_robot = sim_->IsGroupRobot(idx);

      // Build name → index map
      for (std::size_t j = 0; j < joint_names.size(); ++j) {
        h.name_index_map[joint_names[j]] = j;
      }

      // Topic names from config (groups_ order matches group_configs_ order)
      const std::string& cmd_topic = group_configs_[idx].command_topic;
      const std::string& state_topic = group_configs_[idx].state_topic;

      // Publisher
      h.state_pub = create_publisher<sensor_msgs::msg::JointState>(
          state_topic, rclcpp::QoS(10));

      // Subscriber (JointCommand for all groups)
      h.cmd_sub = create_subscription<rtc_msgs::msg::JointCommand>(
          cmd_topic,
          cmd_qos,
          [this, idx](const rtc_msgs::msg::JointCommand::SharedPtr msg) {
            GroupCommandCallback(idx, msg);
          });

      // State callback for robot groups
      if (is_robot) {
        sim_->SetStateCallback(idx,
            [this, idx](const std::vector<double>& pos,
                        const std::vector<double>& vel,
                        const std::vector<double>& eff) {
              PublishGroupState(idx, pos, vel, eff);
            });
      }

      RCLCPP_INFO(get_logger(), "Group[%zu] '%s' %s — cmd: %s  state: %s  joints: %zu",
                  idx, group_configs_[idx].name.c_str(),
                  is_robot ? "ROBOT" : "FAKE",
                  cmd_topic.c_str(), state_topic.c_str(),
                  joint_names.size());
    }
  }

  // ── Timers ───────────────────────────────────────────────────────────────────
  void CreateTimers() {
    // Fake response timers (100 Hz per fake group)
    for (std::size_t idx = 0; idx < sim_->NumGroups(); ++idx) {
      if (sim_->IsGroupRobot(idx)) continue;
      group_handles_[idx].fake_timer = create_wall_timer(
          10ms, [this, idx]() { PublishFakeState(idx); });
    }

    // Simulation status @ 1 Hz
    status_timer_ = create_wall_timer(
        1s, [this]() { PublishSimStatus(); });
  }

  // ── Command callback (all groups) ──────────────────────────────────────────

  void GroupCommandCallback(std::size_t group_idx,
                            const rtc_msgs::msg::JointCommand::SharedPtr msg) {
    if (msg->values.empty()) { return; }

    const int nj = sim_->NumGroupJoints(group_idx);
    const bool is_robot = sim_->IsGroupRobot(group_idx);
    auto& h = group_handles_[group_idx];

    // Build command vector using name-based mapping
    std::vector<double> cmd(static_cast<std::size_t>(nj), 0.0);
    if (msg->joint_names.empty()) {
      const auto n = std::min(msg->values.size(),
                               static_cast<std::size_t>(nj));
      std::copy_n(msg->values.begin(), n, cmd.begin());
    } else {
      for (std::size_t i = 0; i < msg->joint_names.size() && i < msg->values.size(); ++i) {
        auto it = h.name_index_map.find(msg->joint_names[i]);
        if (it != h.name_index_map.end()) {
          cmd[it->second] = msg->values[i];
        }
      }
    }

    if (is_robot) {
      const bool is_torque = (msg->command_type == "torque");
      if (is_torque) {
        if (!sim_->IsInTorqueMode(group_idx)) {
          sim_->SetControlMode(group_idx, true);
          RCLCPP_INFO(get_logger(), "Group[%zu] → torque mode", group_idx);
        }
      } else {
        if (sim_->IsInTorqueMode(group_idx)) {
          sim_->SetControlMode(group_idx, false);
          RCLCPP_INFO(get_logger(), "Group[%zu] → position servo mode", group_idx);
        }
        sim_->EnforcePositionServoGravity();
      }
      sim_->SetCommand(group_idx, cmd);
    } else {
      // Fake group: update LPF target
      sim_->SetFakeTarget(group_idx, cmd);
    }
  }

  // ── State publishing ────────────────────────────────────────────────────────

  void PublishGroupState(std::size_t group_idx,
                          const std::vector<double>& positions,
                          const std::vector<double>& velocities,
                          const std::vector<double>& efforts) {
    if (group_idx >= group_handles_.size()) return;
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name.assign(sim_->GetJointNames(group_idx).begin(),
                    sim_->GetJointNames(group_idx).end());
    msg.position = positions;
    msg.velocity = velocities;
    msg.effort   = efforts;
    group_handles_[group_idx].state_pub->publish(msg);
  }

  void PublishFakeState(std::size_t group_idx) {
    sim_->AdvanceFakeLPF(group_idx);
    auto state = sim_->GetFakeState(group_idx);

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name.assign(sim_->GetJointNames(group_idx).begin(),
                    sim_->GetJointNames(group_idx).end());
    msg.position = state;
    msg.velocity.resize(state.size(), 0.0);
    msg.effort.resize(state.size(), 0.0);
    group_handles_[group_idx].state_pub->publish(msg);
  }

  void PublishSimStatus() {
    const bool   is_running = sim_->IsRunning();
    const bool   is_paused  = sim_->IsPaused();
    const double rtf        = sim_->GetRtf();
    const double sim_time   = sim_->SimTimeSec();
    const auto   steps      = sim_->StepCount();

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {
        static_cast<double>(steps),
        sim_time,
        rtf,
        is_paused ? 1.0 : 0.0,
    };
    sim_status_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "Simulator running=%s%s  steps=%lu  sim_time=%.2f s  rtf=%.1fx",
                is_running ? "true" : "false",
                is_paused  ? " [PAUSED]" : "",
                static_cast<unsigned long>(steps),
                sim_time, rtf);
  }

  // ── Members ─────────────────────────────────────────────────────────────────

  std::unique_ptr<urtc::MuJoCoSimulator> sim_;
  std::vector<urtc::JointGroupConfig> group_configs_;
  std::vector<GroupRosHandles> group_handles_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_status_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/sim/status", rclcpp::QoS(10));

  rclcpp::TimerBase::SharedPtr status_timer_;

  // Parameters
  std::string model_path_;
  bool        enable_viewer_{true};
  std::string sim_mode_{"free_run"};
  int         publish_decimation_{1};
  double      sync_timeout_ms_{50.0};
  double      max_rtf_{0.0};
  double      control_rate_{500.0};
  double      physics_timestep_{0.0};
  bool        use_yaml_servo_gains_{false};
  std::vector<double> servo_kp_;
  std::vector<double> servo_kd_;
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
