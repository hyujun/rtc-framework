// ── mujoco_simulator_node.cpp
// ───────────────────────────────────────────────── ROS2 lifecycle node for
// MuJoCo simulation with multi-group support. robot_response groups use MuJoCo
// physics; fake_response groups use LPF. All groups use rtc_msgs/JointCommand
// for commands, sensor_msgs/JointState for state publishing.
//
// Lifecycle states:
//   Unconfigured -> on_configure -> Inactive -> on_activate -> Active
//   Active -> on_deactivate -> Inactive -> on_cleanup -> Unconfigured
//
// Tier 1 (on_configure): parameters, simulator, publishers, subscribers.
// Tier 2 (on_activate): sim Start, timers.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "rtc_mujoco_sim/ros2_resource_provider.hpp"
#include <rtc_msgs/msg/joint_command.hpp>
#include <rtc_msgs/msg/sim_sensor.hpp>
#include <rtc_msgs/msg/sim_sensor_state.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;
namespace urtc = rtc;

// ── GroupRosHandles
// ──────────────────────────────────────────────────────────────

struct GroupRosHandles {
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
  rclcpp_lifecycle::LifecyclePublisher<rtc_msgs::msg::SimSensorState>::SharedPtr sensor_pub;
  // Contact wrench publishers (one per discovered target). Size matches
  // sim_->GetContactWrenchInfos(group_idx). Empty when ContactWrenchConfig
  // is disabled or no mjSENS_CONTACT sensors exist in the model.
  std::vector<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>::SharedPtr>
      contact_wrench_pubs;
  std::vector<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr>
      contact_state_pubs;
  // Last published contact_state per target — Bool topic transitions only.
  std::vector<bool> contact_state_last;
  rclcpp::Subscription<rtc_msgs::msg::JointCommand>::SharedPtr cmd_sub;
  rclcpp::TimerBase::SharedPtr fake_timer;  // fake groups only
  std::size_t group_idx{0};
  std::unordered_map<std::string, std::size_t> name_index_map;
  bool indices_resolved{false};
};

// ── MuJoCoSimulatorNode
// ────────────────────────────────────────────────────────

class MuJoCoSimulatorNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  MuJoCoSimulatorNode() : LifecycleNode("mujoco_simulator") {
    urtc::RegisterRos2ResourceProvider();
    // Lifecycle design: constructor is intentionally minimal.
    // All resource allocation happens in on_configure().
  }

  ~MuJoCoSimulatorNode() override {
    if (sim_ && sim_->IsRunning()) {
      sim_->Stop();
    }
  }

  /// Tier 1: parameters, simulator, publishers, subscribers.
  CallbackReturn on_configure(const rclcpp_lifecycle::State& /*state*/) override {
    DeclareAndLoadParameters();
    CreateSimulator();
    CreateGroupHandles();

    sim_status_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/sim/status", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(),
                "[MuJoCoSimulatorNode] Configured — model: %s  viewer: %s  "
                "groups: %zu  max_rtf: %.1f",
                model_path_.c_str(), enable_viewer_ ? "ON" : "OFF", sim_->NumGroups(), max_rtf_);
    return CallbackReturn::SUCCESS;
  }

  /// Tier 2: start simulation and timers.
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
    LifecycleNode::on_activate(state);

    sim_->Start();
    CreateTimers();

    RCLCPP_INFO(get_logger(), "[MuJoCoSimulatorNode] Activated");
    return CallbackReturn::SUCCESS;
  }

  /// Stop simulation and timers.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
    // Cancel all timers
    if (status_timer_)
      status_timer_->cancel();
    for (auto& h : group_handles_) {
      if (h.fake_timer)
        h.fake_timer->cancel();
    }

    if (sim_ && sim_->IsRunning()) {
      sim_->Stop();
    }

    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "[MuJoCoSimulatorNode] Deactivated");
    return CallbackReturn::SUCCESS;
  }

  /// Release all resources (reverse order of on_configure).
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*state*/) override {
    status_timer_.reset();
    for (auto& h : group_handles_) {
      h.fake_timer.reset();
      h.cmd_sub.reset();
      h.sensor_pub.reset();
      h.contact_wrench_pubs.clear();
      h.contact_state_pubs.clear();
      h.contact_state_last.clear();
      h.state_pub.reset();
    }
    group_handles_.clear();
    sim_status_pub_.reset();
    sim_.reset();
    group_configs_.clear();

    RCLCPP_INFO(get_logger(), "[MuJoCoSimulatorNode] Cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override {
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      on_deactivate(state);
    }
    return on_cleanup(state);
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State& /*state*/) override {
    RCLCPP_ERROR(get_logger(), "[MuJoCoSimulatorNode] Error — attempting recovery");
    if (status_timer_)
      status_timer_->cancel();
    for (auto& h : group_handles_) {
      if (h.fake_timer)
        h.fake_timer->cancel();
    }
    if (sim_ && sim_->IsRunning()) {
      sim_->Stop();
    }

    status_timer_.reset();
    for (auto& h : group_handles_) {
      h.fake_timer.reset();
      h.cmd_sub.reset();
      h.sensor_pub.reset();
      h.contact_wrench_pubs.clear();
      h.contact_state_pubs.clear();
      h.contact_state_last.clear();
      h.state_pub.reset();
    }
    group_handles_.clear();
    sim_status_pub_.reset();
    sim_.reset();
    group_configs_.clear();

    return CallbackReturn::SUCCESS;
  }

 private:
  // ── Parameter loading
  // ────────────────────────────────────────────────────────
  void DeclareAndLoadParameters() {
    declare_parameter("model_path", std::string(""));
    declare_parameter("enable_viewer", true);
    declare_parameter("sync_timeout_ms", 50.0);
    declare_parameter("max_rtf", 0.0);
    declare_parameter("control_rate", 500.0);
    declare_parameter("physics_timestep", 0.0);
    declare_parameter("n_substeps", 1);
    declare_parameter("viewer_refresh_rate", 60.0);
    declare_parameter("use_yaml_servo_gains", false);
    declare_parameter("servo_kp", std::vector<double>{500.0, 500.0, 500.0, 150.0, 150.0, 150.0});
    declare_parameter("servo_kd", std::vector<double>{400.0, 400.0, 400.0, 100.0, 100.0, 100.0});

    // ── Solver parameters (solver_param.yaml)
    // ─────────────────────────────────
    declare_parameter("solver.solver", std::string("Newton"));
    declare_parameter("solver.cone", std::string("pyramidal"));
    declare_parameter("solver.jacobian", std::string("auto"));
    declare_parameter("solver.integrator", std::string("Euler"));
    declare_parameter("solver.iterations", 100);
    declare_parameter("solver.tolerance", 1e-8);
    declare_parameter("solver.ls_iterations", 50);
    declare_parameter("solver.ls_tolerance", 0.01);
    declare_parameter("solver.noslip_iterations", 0);
    declare_parameter("solver.noslip_tolerance", 1e-6);
    declare_parameter("solver.ccd_iterations", 50);
    declare_parameter("solver.ccd_tolerance", 1e-6);
    declare_parameter("solver.sdf_iterations", 10);
    declare_parameter("solver.sdf_initpoints", 40);
    declare_parameter("solver.impratio", 1.0);
    declare_parameter("solver.warmstart", true);
    declare_parameter("solver.refsafe", true);
    declare_parameter("solver.island", false);
    declare_parameter("solver.eulerdamp", true);
    declare_parameter("solver.filterparent", true);
    declare_parameter("solver.multiccd", false);
    declare_parameter("solver.autoreset", true);
    declare_parameter("solver.nativeccd", true);
    declare_parameter("solver.contact_override.enable", false);
    declare_parameter("solver.contact_override.o_margin", 0.0);
    declare_parameter("solver.contact_override.o_solref", std::vector<double>{0.02, 1.0});
    declare_parameter("solver.contact_override.o_solimp",
                      std::vector<double>{0.9, 0.95, 0.001, 0.5, 2.0});
    declare_parameter("solver.contact_override.o_friction",
                      std::vector<double>{1.0, 1.0, 0.005, 0.0001, 0.0001});

    declare_parameter("robot_response.groups", std::vector<std::string>{});
    declare_parameter("fake_response.groups", std::vector<std::string>{});

    model_path_ = get_parameter("model_path").as_string();
    enable_viewer_ = get_parameter("enable_viewer").as_bool();
    sync_timeout_ms_ = get_parameter("sync_timeout_ms").as_double();
    max_rtf_ = get_parameter("max_rtf").as_double();
    control_rate_ = get_parameter("control_rate").as_double();
    physics_timestep_ = get_parameter("physics_timestep").as_double();
    n_substeps_ = get_parameter("n_substeps").as_int();
    viewer_refresh_rate_ = get_parameter("viewer_refresh_rate").as_double();
    use_yaml_servo_gains_ = get_parameter("use_yaml_servo_gains").as_bool();
    servo_kp_ = get_parameter("servo_kp").as_double_array();
    servo_kd_ = get_parameter("servo_kd").as_double_array();

    solver_config_.solver = get_parameter("solver.solver").as_string();
    solver_config_.cone = get_parameter("solver.cone").as_string();
    solver_config_.jacobian = get_parameter("solver.jacobian").as_string();
    solver_config_.integrator = get_parameter("solver.integrator").as_string();
    solver_config_.iterations = static_cast<int>(get_parameter("solver.iterations").as_int());
    solver_config_.tolerance = get_parameter("solver.tolerance").as_double();
    solver_config_.ls_iterations = static_cast<int>(get_parameter("solver.ls_iterations").as_int());
    solver_config_.ls_tolerance = get_parameter("solver.ls_tolerance").as_double();
    solver_config_.noslip_iterations =
        static_cast<int>(get_parameter("solver.noslip_iterations").as_int());
    solver_config_.noslip_tolerance = get_parameter("solver.noslip_tolerance").as_double();
    solver_config_.ccd_iterations =
        static_cast<int>(get_parameter("solver.ccd_iterations").as_int());
    solver_config_.ccd_tolerance = get_parameter("solver.ccd_tolerance").as_double();
    solver_config_.sdf_iterations =
        static_cast<int>(get_parameter("solver.sdf_iterations").as_int());
    solver_config_.sdf_initpoints =
        static_cast<int>(get_parameter("solver.sdf_initpoints").as_int());
    solver_config_.impratio = get_parameter("solver.impratio").as_double();
    solver_config_.warmstart = get_parameter("solver.warmstart").as_bool();
    solver_config_.refsafe = get_parameter("solver.refsafe").as_bool();
    solver_config_.island = get_parameter("solver.island").as_bool();
    solver_config_.eulerdamp = get_parameter("solver.eulerdamp").as_bool();
    solver_config_.filterparent = get_parameter("solver.filterparent").as_bool();
    solver_config_.multiccd = get_parameter("solver.multiccd").as_bool();
    solver_config_.autoreset = get_parameter("solver.autoreset").as_bool();
    solver_config_.nativeccd = get_parameter("solver.nativeccd").as_bool();
    solver_config_.contact_override.enable =
        get_parameter("solver.contact_override.enable").as_bool();
    solver_config_.contact_override.o_margin =
        get_parameter("solver.contact_override.o_margin").as_double();
    {
      auto v = get_parameter("solver.contact_override.o_solref").as_double_array();
      for (std::size_t i = 0; i < std::min(v.size(), std::size_t{2}); ++i)
        solver_config_.contact_override.o_solref[i] = v[i];
    }
    {
      auto v = get_parameter("solver.contact_override.o_solimp").as_double_array();
      for (std::size_t i = 0; i < std::min(v.size(), std::size_t{5}); ++i)
        solver_config_.contact_override.o_solimp[i] = v[i];
    }
    {
      auto v = get_parameter("solver.contact_override.o_friction").as_double_array();
      for (std::size_t i = 0; i < std::min(v.size(), std::size_t{5}); ++i)
        solver_config_.contact_override.o_friction[i] = v[i];
    }

    auto robot_groups = get_parameter("robot_response.groups").as_string_array();
    for (const auto& gname : robot_groups) {
      DeclareGroupParams("robot_response", gname);
      urtc::JointGroupConfig gc;
      gc.name = gname;
      gc.is_robot = true;
      gc.joint_names = get_parameter("robot_response." + gname + ".joint_names").as_string_array();
      gc.command_joint_names =
          get_parameter("robot_response." + gname + ".command_joint_names").as_string_array();
      gc.state_joint_names =
          get_parameter("robot_response." + gname + ".state_joint_names").as_string_array();
      gc.command_topic = get_parameter("robot_response." + gname + ".command_topic").as_string();
      gc.state_topic = get_parameter("robot_response." + gname + ".state_topic").as_string();
      gc.sensor_topic = get_parameter("robot_response." + gname + ".sensor_topic").as_string();
      gc.sensor_names =
          get_parameter("robot_response." + gname + ".sensor_names").as_string_array();

      try {
        gc.servo_kp = get_parameter("robot_response." + gname + ".servo_kp").as_double_array();
        gc.servo_kd = get_parameter("robot_response." + gname + ".servo_kd").as_double_array();
      } catch (...) {
      }

      LoadContactWrenchConfig("robot_response", gname, gc.contact_wrench);

      group_configs_.push_back(std::move(gc));
    }

    auto fake_groups = get_parameter("fake_response.groups").as_string_array();
    for (const auto& gname : fake_groups) {
      DeclareGroupParams("fake_response", gname);
      declare_parameter("fake_response." + gname + ".filter_alpha", 0.1);

      urtc::JointGroupConfig gc;
      gc.name = gname;
      gc.is_robot = false;
      gc.joint_names = get_parameter("fake_response." + gname + ".joint_names").as_string_array();
      gc.command_joint_names =
          get_parameter("fake_response." + gname + ".command_joint_names").as_string_array();
      gc.state_joint_names =
          get_parameter("fake_response." + gname + ".state_joint_names").as_string_array();
      gc.command_topic = get_parameter("fake_response." + gname + ".command_topic").as_string();
      gc.state_topic = get_parameter("fake_response." + gname + ".state_topic").as_string();
      gc.sensor_topic = get_parameter("fake_response." + gname + ".sensor_topic").as_string();
      gc.sensor_names = get_parameter("fake_response." + gname + ".sensor_names").as_string_array();
      gc.filter_alpha = get_parameter("fake_response." + gname + ".filter_alpha").as_double();

      group_configs_.push_back(std::move(gc));
    }

    // model_path is required and must be supplied by the robot bringup YAML
    // or via launch arg. Relative paths are interpreted relative to the
    // current working directory by MuJoCo's mj_loadXML.
    // Use `package://<pkg>/<rel>` to resolve through ament; the
    // robot-specific bringup is expected to do that substitution before
    // passing the param.
    if (model_path_.empty()) {
      throw std::runtime_error(
          "mujoco_simulator: model_path parameter is required and must be set "
          "by the robot bringup config (e.g. "
          "package://<pkg>/path/to/scene.xml)");
    }
  }

  void DeclareGroupParams(const std::string& section, const std::string& gname) {
    const auto prefix = section + "." + gname + ".";
    declare_parameter(prefix + "joint_names", std::vector<std::string>{});
    declare_parameter(prefix + "command_joint_names", std::vector<std::string>{});
    declare_parameter(prefix + "state_joint_names", std::vector<std::string>{});
    declare_parameter(prefix + "command_topic", std::string(""));
    declare_parameter(prefix + "state_topic", std::string(""));
    declare_parameter(prefix + "sensor_topic", std::string(""));
    declare_parameter(prefix + "sensor_names", std::vector<std::string>{});
    // Per-group PD overrides; empty default = inherit the global servo_kp/kd.
    // Required when groups have different DoF since a single global
    // servo_kp/kd array cannot match multiple distinct joint counts.
    declare_parameter(prefix + "servo_kp", std::vector<double>{});
    declare_parameter(prefix + "servo_kd", std::vector<double>{});
    // Contact wrench publishing (opt-in). enabled+topic_prefix are required
    // to activate; suffix lists default to the "_tip_contact"/"_tip_ft_site"
    // convention but are fully YAML-overridable for any MJCF naming scheme.
    declare_parameter(prefix + "contact_wrench.enabled", false);
    declare_parameter(prefix + "contact_wrench.topic_prefix", std::string(""));
    declare_parameter(prefix + "contact_wrench.sensor_name_suffixes",
                      std::vector<std::string>{"_tip_contact", "_contact"});
    declare_parameter(prefix + "contact_wrench.reference_site_suffixes",
                      std::vector<std::string>{"_tip_ft_site", "_ft_site"});
    declare_parameter(prefix + "contact_wrench.publish_state", false);
    declare_parameter(prefix + "contact_wrench.publish_debug", false);
    declare_parameter(prefix + "contact_wrench.allow_partial_discovery", false);
  }

  void LoadContactWrenchConfig(
      const std::string& section, const std::string& gname,
      urtc::JointGroupConfig::ContactWrenchConfig& cwc) {
    const auto prefix = section + "." + gname + ".contact_wrench.";
    cwc.enabled = get_parameter(prefix + "enabled").as_bool();
    cwc.topic_prefix = get_parameter(prefix + "topic_prefix").as_string();
    cwc.sensor_name_suffixes =
        get_parameter(prefix + "sensor_name_suffixes").as_string_array();
    cwc.reference_site_suffixes =
        get_parameter(prefix + "reference_site_suffixes").as_string_array();
    cwc.publish_state = get_parameter(prefix + "publish_state").as_bool();
    cwc.publish_debug = get_parameter(prefix + "publish_debug").as_bool();
    cwc.allow_partial_discovery =
        get_parameter(prefix + "allow_partial_discovery").as_bool();
  }

  // ── Simulator creation
  // ───────────────────────────────────────────────────────
  void CreateSimulator() {
    urtc::MuJoCoSimulator::Config cfg{
        .model_path = model_path_,
        .window_title = {},
        .enable_viewer = enable_viewer_,
        .sync_timeout_ms = sync_timeout_ms_,
        .max_rtf = max_rtf_,
        .physics_timestep = physics_timestep_,
        .n_substeps = static_cast<int>(n_substeps_),
        .viewer_refresh_rate = viewer_refresh_rate_,
        .use_yaml_servo_gains = use_yaml_servo_gains_,
        .servo_kp = servo_kp_,
        .servo_kd = servo_kd_,
        .solver_config = solver_config_,
        .groups = group_configs_,
    };
    sim_ = std::make_unique<urtc::MuJoCoSimulator>(std::move(cfg));

    if (!sim_->Initialize()) {
      RCLCPP_FATAL(get_logger(),
                   "[MuJoCoSimulatorNode] Failed to initialize MuJoCo "
                   "simulator from '%s'",
                   model_path_.c_str());
      throw std::runtime_error("MuJoCo initialization failed");
    }

    const double xml_dt = sim_->GetPhysicsTimestep();
    if (xml_dt > 0.0 && control_rate_ > 0.0) {
      const double physics_freq = 1.0 / xml_dt;
      if (physics_freq < control_rate_) {
        RCLCPP_FATAL(get_logger(),
                     "[MuJoCoSimulatorNode] Physics frequency (%.1f Hz) < "
                     "control_rate (%.1f Hz)",
                     physics_freq, control_rate_);
        throw std::runtime_error("physics_timestep vs control_rate mismatch");
      }
    }
  }

  // ── Create per-group publishers, subscribers, callbacks
  // ─────────────────────
  void CreateGroupHandles() {
    const auto num_groups = sim_->NumGroups();
    group_handles_.resize(num_groups);

    rclcpp::QoS cmd_qos{1};
    cmd_qos.best_effort();

    for (std::size_t idx = 0; idx < num_groups; ++idx) {
      auto& h = group_handles_[idx];
      h.group_idx = idx;

      const auto& cmd_joint_names = sim_->GetJointNames(idx);
      const bool is_robot = sim_->IsGroupRobot(idx);

      for (std::size_t j = 0; j < cmd_joint_names.size(); ++j) {
        h.name_index_map[cmd_joint_names[j]] = j;
      }

      const std::string& cmd_topic = group_configs_[idx].command_topic;
      const std::string& state_topic = group_configs_[idx].state_topic;

      rclcpp::QoS state_pub_qos{1};
      state_pub_qos.best_effort();
      h.state_pub = create_publisher<sensor_msgs::msg::JointState>(state_topic, state_pub_qos);

      h.cmd_sub = create_subscription<rtc_msgs::msg::JointCommand>(
          cmd_topic, cmd_qos, [this, idx](const rtc_msgs::msg::JointCommand::SharedPtr msg) {
            GroupCommandCallback(idx, msg);
          });

      if (is_robot) {
        sim_->SetStateCallback(
            idx,
            [this, idx](const std::vector<double>& pos, const std::vector<double>& vel,
                        const std::vector<double>& eff) { PublishGroupState(idx, pos, vel, eff); });
      }

      const std::string& sensor_topic = group_configs_[idx].sensor_topic;
      if (is_robot && !sensor_topic.empty() && sim_->HasSensors(idx)) {
        rclcpp::QoS sensor_qos{1};
        sensor_qos.best_effort();
        h.sensor_pub = create_publisher<rtc_msgs::msg::SimSensorState>(sensor_topic, sensor_qos);

        sim_->SetSensorCallback(idx,
                                [this, idx](const std::vector<urtc::JointGroup::SensorInfo>& infos,
                                            const std::vector<double>& values) {
                                  PublishGroupSensors(idx, infos, values);
                                });
      }

      // ── Contact wrench publishers (one per auto-discovered target) ──
      const auto& cw_cfg = group_configs_[idx].contact_wrench;
      if (is_robot && cw_cfg.enabled && !cw_cfg.topic_prefix.empty() &&
          sim_->HasContactWrenches(idx)) {
        const auto& cw_infos = sim_->GetContactWrenchInfos(idx);
        rclcpp::QoS cw_qos = rclcpp::SensorDataQoS().keep_last(1);

        h.contact_wrench_pubs.reserve(cw_infos.size());
        if (cw_cfg.publish_state) {
          h.contact_state_pubs.reserve(cw_infos.size());
          h.contact_state_last.assign(cw_infos.size(), false);
        }

        for (const auto& cw_info : cw_infos) {
          const std::string wrench_topic =
              cw_cfg.topic_prefix + "/" + cw_info.target_name + "/contact_wrench";
          h.contact_wrench_pubs.push_back(
              create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, cw_qos));
          if (cw_cfg.publish_state) {
            const std::string contact_state_topic =
                cw_cfg.topic_prefix + "/" + cw_info.target_name + "/contact_state";
            h.contact_state_pubs.push_back(
                create_publisher<std_msgs::msg::Bool>(contact_state_topic, cw_qos));
          }
          RCLCPP_INFO(get_logger(),
                      "[MuJoCoSimulatorNode] Group[%zu] contact_wrench target='%s' "
                      "frame=%s topic=%s",
                      idx, cw_info.target_name.c_str(), cw_info.frame_id.c_str(),
                      wrench_topic.c_str());
        }

        sim_->SetContactWrenchCallback(
            idx, [this, idx](
                     const std::vector<urtc::JointGroup::ContactWrenchInfo>& infos,
                     const std::vector<urtc::JointGroup::ContactWrenchSample>& samples) {
              PublishGroupContactWrenches(idx, infos, samples);
            });
      }

      RCLCPP_INFO(get_logger(),
                  "[MuJoCoSimulatorNode] Group[%zu] '%s' %s — cmd: %s  "
                  "state: %s  sensor: %s  cmd_joints: %d  state_joints: %d",
                  idx, group_configs_[idx].name.c_str(), is_robot ? "ROBOT" : "FAKE",
                  cmd_topic.c_str(), state_topic.c_str(),
                  sensor_topic.empty() ? "(none)" : sensor_topic.c_str(), sim_->NumGroupJoints(idx),
                  sim_->NumStateJoints(idx));
    }
  }

  // ── Timers
  // ───────────────────────────────────────────────────────────────────
  void CreateTimers() {
    for (std::size_t idx = 0; idx < sim_->NumGroups(); ++idx) {
      if (sim_->IsGroupRobot(idx))
        continue;
      group_handles_[idx].fake_timer =
          create_wall_timer(10ms, [this, idx]() { PublishFakeState(idx); });
    }

    status_timer_ = create_wall_timer(1s, [this]() { PublishSimStatus(); });
  }

  // ── Command callback (all groups) ──────────────────────────────────────────

  void GroupCommandCallback(std::size_t group_idx,
                            const rtc_msgs::msg::JointCommand::SharedPtr msg) {
    if (msg->values.empty()) {
      return;
    }

    const int nj = sim_->NumGroupJoints(group_idx);
    const bool is_robot = sim_->IsGroupRobot(group_idx);
    auto& h = group_handles_[group_idx];

    // Remap a per-joint array into group index order. joint_names empty → use
    // the message order directly (truncated to nj); otherwise map by name.
    // Unlisted joints stay 0 — fine for command/feedforward, NOT for gains
    // (validated for full coverage separately below).
    auto remap = [&](const std::vector<double>& src) {
      std::vector<double> out(static_cast<std::size_t>(nj), 0.0);
      if (msg->joint_names.empty()) {
        std::copy_n(src.begin(), std::min(src.size(), static_cast<std::size_t>(nj)), out.begin());
      } else {
        for (std::size_t i = 0; i < msg->joint_names.size() && i < src.size(); ++i) {
          auto it = h.name_index_map.find(msg->joint_names[i]);
          if (it != h.name_index_map.end())
            out[it->second] = src[i];
        }
      }
      return out;
    };

    const std::vector<double> cmd = remap(msg->values);

    if (!is_robot) {
      sim_->SetFakeTarget(group_idx, cmd);
      return;
    }

    urtc::JointControlMode mode = urtc::JointControlMode::kPosition;
    if (msg->command_type == "torque")
      mode = urtc::JointControlMode::kTorque;
    else if (msg->command_type == "pd_feedforward")
      mode = urtc::JointControlMode::kPdFeedforward;

    if (sim_->GetControlMode(group_idx) != mode) {
      RCLCPP_INFO(get_logger(), "[MuJoCoSimulatorNode] Group[%zu] → %s mode", group_idx,
                  mode == urtc::JointControlMode::kTorque          ? "torque"
                  : mode == urtc::JointControlMode::kPdFeedforward ? "pd_feedforward"
                                                             : "position servo");
    }

    std::vector<double> feedforward;
    if (mode == urtc::JointControlMode::kPdFeedforward)
      feedforward = remap(msg->feedforward);

    // Runtime PD gains — all-or-nothing: kp/kd must both be present, cover every
    // group joint, and be non-negative. Partial/short/negative → ignore (keep
    // current gains) with a throttled warning. Empty kp & kd → keep current.
    std::vector<double> kp;
    std::vector<double> kd;
    if (!msg->kp.empty() || !msg->kd.empty()) {
      bool ok = (msg->kp.size() == msg->kd.size());
      if (ok) {
        ok = msg->joint_names.empty() ? (static_cast<int>(msg->kp.size()) == nj)
                                      : (msg->kp.size() == msg->joint_names.size() &&
                                         GainsCoverAllJoints(h, msg, nj));
      }
      if (ok) {
        for (double v : msg->kp)
          ok = ok && (v >= 0.0);
        for (double v : msg->kd)
          ok = ok && (v >= 0.0);
      }
      if (ok) {
        kp = remap(msg->kp);
        kd = remap(msg->kd);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[MuJoCoSimulatorNode] Group[%zu] gain update ignored: kp/kd must "
                             "both be non-negative and cover all %d joints",
                             group_idx, nj);
      }
    }

    sim_->StageCommand(group_idx, mode, cmd, feedforward, kp, kd);
  }

  // True iff every group joint index is named exactly once in msg->joint_names.
  static bool GainsCoverAllJoints(const GroupRosHandles& handle,
                                  const rtc_msgs::msg::JointCommand::SharedPtr& msg, int nj) {
    if (static_cast<int>(msg->joint_names.size()) < nj)
      return false;
    std::vector<bool> seen(static_cast<std::size_t>(nj), false);
    int covered = 0;
    for (const auto& name : msg->joint_names) {
      auto it = handle.name_index_map.find(name);
      if (it != handle.name_index_map.end() && it->second < static_cast<std::size_t>(nj) &&
          !seen[it->second]) {
        seen[it->second] = true;
        ++covered;
      }
    }
    return covered == nj;
  }

  // ── State publishing
  // ────────────────────────────────────────────────────────

  void PublishGroupState(std::size_t group_idx, const std::vector<double>& positions,
                         const std::vector<double>& velocities,
                         const std::vector<double>& efforts) {
    if (group_idx >= group_handles_.size())
      return;
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name.assign(sim_->GetStateJointNames(group_idx).begin(),
                    sim_->GetStateJointNames(group_idx).end());
    msg.position = positions;
    msg.velocity = velocities;
    msg.effort = efforts;
    group_handles_[group_idx].state_pub->publish(msg);
  }

  void PublishGroupSensors(std::size_t group_idx,
                           const std::vector<urtc::JointGroup::SensorInfo>& infos,
                           const std::vector<double>& values) {
    if (group_idx >= group_handles_.size() || !group_handles_[group_idx].sensor_pub)
      return;
    auto msg = rtc_msgs::msg::SimSensorState();
    msg.header.stamp = now();
    int offset = 0;
    for (const auto& info : infos) {
      rtc_msgs::msg::SimSensor s;
      s.name = info.name;
      s.sensor_type = info.type;
      s.values.assign(values.begin() + offset, values.begin() + offset + info.dim);
      offset += info.dim;
      msg.sensors.push_back(std::move(s));
    }
    group_handles_[group_idx].sensor_pub->publish(msg);
  }

  // Publishes one WrenchStamped per discovered contact target (link frame).
  // found=false → wrench=0 still published every tick (SensorDataQoS stale guard).
  // If publish_state is enabled, a Bool is also emitted on transitions only.
  void PublishGroupContactWrenches(
      std::size_t group_idx,
      const std::vector<urtc::JointGroup::ContactWrenchInfo>& infos,
      const std::vector<urtc::JointGroup::ContactWrenchSample>& samples) {
    if (group_idx >= group_handles_.size())
      return;
    auto& handle = group_handles_[group_idx];
    if (handle.contact_wrench_pubs.size() != infos.size())
      return;
    const auto stamp = now();
    for (std::size_t i = 0; i < infos.size(); ++i) {
      const auto& cw_info = infos[i];
      const auto& sample = samples[i];
      geometry_msgs::msg::WrenchStamped wmsg;
      wmsg.header.stamp = stamp;
      wmsg.header.frame_id = cw_info.frame_id;
      wmsg.wrench.force.x = sample.force[0];
      wmsg.wrench.force.y = sample.force[1];
      wmsg.wrench.force.z = sample.force[2];
      wmsg.wrench.torque.x = sample.torque[0];
      wmsg.wrench.torque.y = sample.torque[1];
      wmsg.wrench.torque.z = sample.torque[2];
      handle.contact_wrench_pubs[i]->publish(wmsg);

      if (i < handle.contact_state_pubs.size() && handle.contact_state_pubs[i]) {
        const bool last = (i < handle.contact_state_last.size())
                              ? handle.contact_state_last[i]
                              : false;
        if (sample.found != last) {
          std_msgs::msg::Bool bmsg;
          bmsg.data = sample.found;
          handle.contact_state_pubs[i]->publish(bmsg);
          if (i < handle.contact_state_last.size()) {
            handle.contact_state_last[i] = sample.found;
          }
        }
      }
    }
  }

  void PublishFakeState(std::size_t group_idx) {
    sim_->AdvanceFakeLPF(group_idx);
    auto state = sim_->GetFakeState(group_idx);

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name.assign(sim_->GetStateJointNames(group_idx).begin(),
                    sim_->GetStateJointNames(group_idx).end());
    msg.position = state;
    msg.velocity.resize(state.size(), 0.0);
    msg.effort.resize(state.size(), 0.0);
    group_handles_[group_idx].state_pub->publish(msg);
  }

  void PublishSimStatus() {
    const bool is_running = sim_->IsRunning();
    const bool is_paused = sim_->IsPaused();
    const double rtf = sim_->GetRtf();
    const double sim_time = sim_->SimTimeSec();
    const auto steps = sim_->StepCount();

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {
        static_cast<double>(steps),
        sim_time,
        rtf,
        is_paused ? 1.0 : 0.0,
    };
    sim_status_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "[MuJoCoSimulatorNode] Simulator running=%s%s  steps=%lu  "
                "sim_time=%.2f s  rtf=%.1fx",
                is_running ? "true" : "false", is_paused ? " [PAUSED]" : "",
                static_cast<unsigned long>(steps), sim_time, rtf);
  }

  // ── Members
  // ─────────────────────────────────────────────────────────────────

  std::unique_ptr<urtc::MuJoCoSimulator> sim_;
  std::vector<urtc::JointGroupConfig> group_configs_;
  std::vector<GroupRosHandles> group_handles_;

  // sim_status_pub_ initialized in on_configure (NOT in member initializer
  // list).
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_status_pub_;

  rclcpp::TimerBase::SharedPtr status_timer_;

  // Parameters
  std::string model_path_;
  bool enable_viewer_{true};
  double sync_timeout_ms_{50.0};
  double max_rtf_{0.0};
  double control_rate_{500.0};
  double physics_timestep_{0.0};
  int64_t n_substeps_{1};
  double viewer_refresh_rate_{60.0};
  bool use_yaml_servo_gains_{false};
  std::vector<double> servo_kp_;
  std::vector<double> servo_kd_;
  urtc::SolverConfig solver_config_;
};

// ── Entry point
// ────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MuJoCoSimulatorNode>();
    // Constructor is minimal — launch event handler triggers
    // configure/activate.
    rclcpp::spin(node->get_node_base_interface());
  } catch (const std::exception& e) {
    fprintf(stderr, "[MuJoCoSimulatorNode] Fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
