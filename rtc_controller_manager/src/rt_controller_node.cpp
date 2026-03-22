// ── Includes: project header first, then ROS2, then C++ stdlib ──────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include "rtc_controller_interface/controller_registry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rtc_base/logging/session_dir.hpp>
#include <rtc_base/threading/thread_utils.hpp>
#include <yaml-cpp/yaml.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <sched.h>    // sched_yield
#include <time.h>     // clock_nanosleep, clock_gettime, CLOCK_MONOTONIC, TIMER_ABSTIME

#include <algorithm>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <set>
#include <string_view>
#include <unordered_map>

using namespace std::chrono_literals;
namespace urtc = rtc;

// ── Controller registry ──────────────────────────────────────────────────────
//
// To add a new controller:
//   1. Implement RTControllerInterface in your package
//   2. Create a config/controllers/<subdir>/<key>.yaml
//   3. Use RTC_REGISTER_CONTROLLER() macro in a .cpp file
//   4. Link the final executable against your library
//
// See rtc_controllers/src/controller_registration.cpp for built-in examples.
// ─────────────────────────────────────────────────────────────────────────────

// ── Constructor / destructor ──────────────────────────────────────────────────
RtControllerNode::RtControllerNode()
: Node("rt_controller"), logger_(nullptr)
{
  CreateCallbackGroups();
  DeclareAndLoadParameters();
  CreateSubscriptions();
  CreatePublishers();
  ExposeTopicParameters();
  CreateTimers();

  // 초기 활성 컨트롤러 이름 publish (transient_local이므로 구독자가 늦게 붙어도 수신)
  {
    std_msgs::msg::String ctrl_name_msg;
    ctrl_name_msg.data = std::string(
      controllers_[active_controller_idx_.load(std::memory_order_acquire)]->Name());
    active_ctrl_name_pub_->publish(ctrl_name_msg);
  }

  // ── Status Monitor (optional) ────────────────────────────────────────────
  if (enable_status_monitor_) {
    status_monitor_ = std::make_unique<rtc::RtcStatusMonitor>(
        shared_from_this());

    status_monitor_->registerOnFailure(
        [this](rtc::FailureType type,
               const rtc::FailureContext & ctx) {
          (void)type;
          TriggerGlobalEstop(ctx.description);
        });

    status_monitor_->registerOnReady(
        [this]() {
          RCLCPP_INFO(get_logger(), "StatusMonitor: system ready");
        });

    const auto cfgs = rtc::SelectThreadConfigs();
    status_monitor_->start(cfgs.status_monitor);
    RCLCPP_INFO(get_logger(), "RtcStatusMonitor started (10 Hz, Core %d)",
                cfgs.status_monitor.cpu_core);
  }

  RCLCPP_INFO(get_logger(), "RtControllerNode ready — %.0f Hz, E-STOP: %s",
              control_rate_, enable_estop_ ? "ON" : "OFF");
  RCLCPP_INFO(get_logger(),
      "Threading: clock_nanosleep RT loop + SPSC publish offload + "
      "Sensor/Log/Aux executors");
}

RtControllerNode::~RtControllerNode()
{
  // Stop RT loop + publish thread (idempotent — safe if already stopped by main)
  StopRtLoop();
  StopPublishLoop();

  if (status_monitor_) {
    status_monitor_->stop();
  }
  if (logger_) {
    // 종료 시 drain_timer_가 멈춘 후에도 ring buffer에 남은 항목을 모두 기록
    logger_->DrainBuffer(log_buffer_);
    logger_->Flush();
  }
}

// ── Session directory helpers ─────────────────────────────────────────────────
std::filesystem::path RtControllerNode::ResolveAndSetupSessionDir()
{
  // Dynamically resolve workspace logging dir using ament_index
  std::string default_logging_root = "/tmp/ur5e_logging_data";
  try {
    std::string share_dir =
      ament_index_cpp::get_package_share_directory("rtc_controller_manager");
    std::filesystem::path ws_path = std::filesystem::path(share_dir)
      .parent_path()
      .parent_path()
      .parent_path()
      .parent_path();
    default_logging_root = (ws_path / "logging_data").string();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
        get_logger(),
        "Could not resolve workspace via ament_index: %s. Using fallback: %s",
        e.what(), default_logging_root.c_str());
  }

  return urtc::ResolveSessionDir(default_logging_root);
}

// ── CallbackGroup creation ────────────────────────────────────────────────────
void RtControllerNode::CreateCallbackGroups()
{
  // cb_group_rt_ removed — ControlLoop() + CheckTimeouts() run in
  // a dedicated clock_nanosleep jthread (RtLoopEntry), not an executor.
  cb_group_sensor_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_log_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_aux_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

// ── Initialisation helpers ────────────────────────────────────────────────────
void RtControllerNode::DeclareAndLoadParameters()
{
  declare_parameter("control_rate", 500.0);
  declare_parameter("kp", 5.0);
  declare_parameter("kd", 0.5);
  declare_parameter("enable_logging", true);
  declare_parameter("log_dir", std::string(""));  // launch 파일이 세션 디렉토리로 덮어씀
  declare_parameter("max_log_sessions", 10);
  declare_parameter("enable_timing_log", true);
  declare_parameter("enable_robot_log", true);
  declare_parameter("enable_hand_log", true);
  declare_parameter("enable_device_log", true);
  declare_parameter("enable_estop", true);
  declare_parameter("enable_status_monitor", false);
  declare_parameter("init_timeout_sec", 5.0);
  declare_parameter("auto_hold_position", true);
  declare_parameter("initial_controller", "joint_pd_controller");

  // ── Device timeouts (replaces robot_timeout_ms / enable_ur5e / enable_hand) ─
  declare_parameter("device_timeout_names", std::vector<std::string>{});
  declare_parameter("device_timeout_values", std::vector<double>{});

  // Joint/Motor/Fingertip names (v5.14.0 named messaging)
  declare_parameter("robot_joint_names", std::vector<std::string>{});
  declare_parameter("hand_motor_names", std::vector<std::string>{});
  declare_parameter("hand_joint_names", std::vector<std::string>{});
  declare_parameter("hand_fingertip_names", std::vector<std::string>{});
  declare_parameter("hand_sensor_names", std::vector<std::string>{});


  control_rate_ = get_parameter("control_rate").as_double();
  budget_us_ = 1.0e6 / control_rate_;  // tick budget in µs (e.g., 2000.0 at 500 Hz)

  const double init_timeout_sec = get_parameter("init_timeout_sec").as_double();
  init_timeout_ticks_ = static_cast<uint64_t>(init_timeout_sec * control_rate_);
  auto_hold_position_ = get_parameter("auto_hold_position").as_bool();
  enable_logging_ = get_parameter("enable_logging").as_bool();
  enable_estop_ = get_parameter("enable_estop").as_bool();
  enable_status_monitor_ = get_parameter("enable_status_monitor").as_bool();

  // NOTE: Logging setup is deferred until after controller loading,
  // because active_groups_ must be known to determine which logs to create.

  std::string urdf_path = "";
  try {
    std::string share_dir =
      ament_index_cpp::get_package_share_directory("ur5e_description");
    urdf_path = share_dir + "/robots/ur5e/urdf/ur5e.urdf";
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Could not resolve urdf path: %s", e.what());
  }

  // ── Joint name loading & URDF validation (v5.14.0) ─────────────────────
  LoadAndValidateJointNames();

  // hand_motor_names_ 기반 identity map 빌드 (sim/UDP 모두 동일 이름 순서 사용)
  // Build device reorder maps from joint names for known groups
  // (robot reorder is built on first message; hand identity map built here)

  // ── Instantiate and configure all registered controllers ─────────────────
  // Controllers are registered via RTC_REGISTER_CONTROLLER() macro at static
  // init time.  Each factory constructs the controller with default gains,
  // then LoadConfig() reads per-controller overrides from its YAML file.
  std::unordered_map<std::string, int> name_to_idx;
  const auto & entries = urtc::ControllerRegistry::Instance().GetEntries();

  for (std::size_t i = 0; i < entries.size(); ++i) {
    const auto & entry = entries[i];
    auto ctrl = entry.factory(urdf_path);

    // Config path: <config_package>/config/controllers/<subdir>/<key>.yaml
    try {
      const std::string pkg_dir =
        ament_index_cpp::get_package_share_directory(entry.config_package);
      const std::string yaml_path = pkg_dir + "/config/controllers/" +
        entry.config_subdir + entry.config_key + ".yaml";
      const YAML::Node file_node = YAML::LoadFile(yaml_path);
      ctrl->LoadConfig(file_node[entry.config_key]);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "Config load failed for '%s' (pkg=%s, %s) — using defaults",
        ctrl->Name().data(), entry.config_package.c_str(), e.what());
    }

    // Register both the class name (e.g. "PDController") and the config-key
    // alias (e.g. "pd_controller") so either form works as initial_controller.
    name_to_idx[std::string(ctrl->Name())] = static_cast<int>(i);
    name_to_idx[entry.config_key] = static_cast<int>(i);

    controllers_.push_back(std::move(ctrl));
  }

  // Cache per-controller topic configs and build active_groups_ + group_slot_map_
  controller_topic_configs_.reserve(controllers_.size());
  for (const auto & ctrl : controllers_) {
    controller_topic_configs_.push_back(ctrl->GetTopicConfig());

    const auto & tc = controller_topic_configs_.back();
    std::string groups_info;
    for (const auto & [name, group] : tc.groups) {
      if (!group.subscribe.empty() || !group.publish.empty()) {
        groups_info += name + "(" +
            std::to_string(group.subscribe.size()) + "sub+" +
            std::to_string(group.publish.size()) + "pub) ";
      }
    }
    RCLCPP_INFO(get_logger(), "Controller '%s': %s",
        ctrl->Name().data(), groups_info.c_str());
  }

  // ── Build active_groups_ (union of all controllers' groups) ──────────────
  {
    int slot_idx = 0;
    for (const auto & tc : controller_topic_configs_) {
      for (const auto & [name, group] : tc.groups) {
        if (!group.subscribe.empty() || !group.publish.empty()) {
          if (active_groups_.insert(name).second) {
            group_slot_map_[name] = slot_idx++;
          }
        }
      }
    }
    if (slot_idx > urtc::PublishSnapshot::kMaxGroups) {
      RCLCPP_FATAL(get_logger(), "Too many device groups (%d > %d)",
          slot_idx, urtc::PublishSnapshot::kMaxGroups);
      rclcpp::shutdown();
      return;
    }
  }

  // Build device reorder maps for known groups
  if (auto it = group_slot_map_.find("hand"); it != group_slot_map_.end()) {
    BuildDeviceReorderMap(it->second, hand_motor_names_);
  }

  // ── Deferred logging setup (needs active_groups_) ────────────────────────
  if (enable_logging_) {
    const std::string log_dir_param = get_parameter("log_dir").as_string();
    const int max_sessions = get_parameter("max_log_sessions").as_int();

    std::filesystem::path session_dir;
    if (!log_dir_param.empty()) {
      session_dir = std::filesystem::path(log_dir_param);
      std::filesystem::create_directories(session_dir);
      urtc::EnsureSessionSubdirs(session_dir);
    } else {
      session_dir = ResolveAndSetupSessionDir();
    }
    const auto logging_root = session_dir.parent_path();
    urtc::CleanupOldSessions(logging_root, max_sessions);

    const bool enable_timing = get_parameter("enable_timing_log").as_bool();
    const bool enable_device = get_parameter("enable_device_log").as_bool();
    const bool enable_robot  = enable_device && active_groups_.contains("ur5e");
    const bool enable_hand   = enable_device && active_groups_.contains("hand");
    const auto ctrl_dir = session_dir / "controller";
    std::filesystem::create_directories(ctrl_dir);

    const std::string timing_path = enable_timing
        ? (ctrl_dir / "timing_log.csv").string() : "";
    const std::string robot_path = enable_robot
        ? (ctrl_dir / "robot_log.csv").string() : "";
    const std::string hand_path = enable_hand
        ? (ctrl_dir / "hand_log.csv").string() : "";

    const auto yaml_joint_names = get_parameter("robot_joint_names").as_string_array();
    auto yaml_motor_names = get_parameter("hand_joint_names").as_string_array();
    if (yaml_motor_names.empty()) {
      yaml_motor_names = get_parameter("hand_motor_names").as_string_array();
    }
    auto yaml_sensor_names = get_parameter("hand_sensor_names").as_string_array();
    if (yaml_sensor_names.empty()) {
      yaml_sensor_names = get_parameter("hand_fingertip_names").as_string_array();
    }
    logger_ = std::make_unique<urtc::DataLogger>(
        timing_path, robot_path, hand_path,
        yaml_joint_names, yaml_motor_names, yaml_sensor_names,
        static_cast<int>(yaml_joint_names.empty() ? urtc::kNumRobotJoints : yaml_joint_names.size()),
        static_cast<int>(yaml_motor_names.empty() ? urtc::kNumHandMotors : yaml_motor_names.size()),
        static_cast<int>(yaml_sensor_names.size() * urtc::kSensorValuesPerFingertip));
    RCLCPP_INFO(get_logger(),
        "Logging to: %s/controller/ (max_sessions=%d)",
        session_dir.string().c_str(), max_sessions);
  }

  // ── Parse device_timeouts & match to active topic groups ─────────────────
  {
    const auto timeout_names = get_parameter("device_timeout_names").as_string_array();
    const auto timeout_values = get_parameter("device_timeout_values").as_double_array();
    for (std::size_t i = 0; i < timeout_names.size() && i < timeout_values.size(); ++i) {
      const auto & name = timeout_names[i];
      if (!active_groups_.contains(name)) {
        RCLCPP_WARN(get_logger(),
            "Device timeout '%s' has no matching topic group — ignored", name.c_str());
        continue;
      }
      std::string state_topic;
      for (const auto & tc : controller_topic_configs_) {
        state_topic = tc.GetSubscribeTopicName(name, urtc::SubscribeRole::kState);
        if (!state_topic.empty()) break;
      }
      if (state_topic.empty()) {
        RCLCPP_WARN(get_logger(),
            "Device timeout '%s' has no state subscription — ignored", name.c_str());
        continue;
      }
      DeviceTimeoutEntry entry;
      entry.group_name = name;
      entry.state_topic = state_topic;
      entry.timeout = std::chrono::milliseconds(static_cast<int>(timeout_values[i]));
      device_timeouts_.push_back(std::move(entry));
      RCLCPP_INFO(get_logger(), "Device timeout: '%s' → watching '%s' (%dms)",
          name.c_str(), state_topic.c_str(), static_cast<int>(timeout_values[i]));
    }
  }

  // Skip init wait if no device timeouts configured
  if (device_timeouts_.empty()) {
    state_received_.store(true, std::memory_order_release);
    target_received_.store(true, std::memory_order_release);
    RCLCPP_INFO(get_logger(), "No device timeouts configured — skipping init wait");
  }

  // Resolve initial_controller parameter → controller index
  const std::string initial_ctrl = get_parameter("initial_controller").as_string();
  const auto it = name_to_idx.find(initial_ctrl);
  if (it != name_to_idx.end()) {
    active_controller_idx_.store(it->second);
  } else {
    RCLCPP_WARN(get_logger(),
      "Unknown initial_controller '%s', defaulting to joint_pd_controller",
      initial_ctrl.c_str());
    const auto pd_it = name_to_idx.find("joint_pd_controller");
    active_controller_idx_.store(pd_it != name_to_idx.end() ? pd_it->second : 1);
  }
}

void RtControllerNode::CreateSubscriptions()
{
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = cb_group_sensor_;

  // ── Helper: find DeviceTimeoutEntry for a given state topic ──────────────
  auto find_timeout_entry = [this](const std::string & topic) -> DeviceTimeoutEntry* {
    for (auto & dt : device_timeouts_) {
      if (dt.state_topic == topic) return &dt;
    }
    return nullptr;
  };

  // ── Create subscriptions for all active device groups ────────────────────
  std::set<std::string> created_topics;

  for (const auto & tc : controller_topic_configs_) {
    for (const auto & [group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name)) continue;

      const int slot = group_slot_map_[group_name];

      for (const auto & entry : group.subscribe) {
        if (!created_topics.insert(entry.topic_name).second) continue;

        DeviceTimeoutEntry * dt_ptr = find_timeout_entry(entry.topic_name);

        switch (entry.role) {
          case urtc::SubscribeRole::kState: {
            auto sub = create_subscription<sensor_msgs::msg::JointState>(
              entry.topic_name, 10,
              [this, slot, dt_ptr](sensor_msgs::msg::JointState::SharedPtr msg) {
                DeviceJointStateCallback(slot, std::move(msg));
                if (dt_ptr) {
                  dt_ptr->last_update = std::chrono::steady_clock::now();
                  dt_ptr->received = true;
                }
              },
              sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [%s/state]: %s (slot %d)",
                        group_name.c_str(), entry.topic_name.c_str(), slot);
            break;
          }
          case urtc::SubscribeRole::kSensorState: {
            auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
              entry.topic_name, 10,
              [this, slot](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                DeviceSensorCallback(slot, std::move(msg));
              },
              sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [%s/sensor_state]: %s (slot %d)",
                        group_name.c_str(), entry.topic_name.c_str(), slot);
            break;
          }
          case urtc::SubscribeRole::kTarget: {
            auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
              entry.topic_name, 10,
              [this, slot](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                DeviceTargetCallback(slot, std::move(msg));
              },
              sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [%s/target]: %s (slot %d)",
                        group_name.c_str(), entry.topic_name.c_str(), slot);
            break;
          }
        }
      }
    }
  }

  // ── Fixed control subscriptions (always present) ──────────────────────────
  controller_selector_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/ur5e/controller_type", 10,
    [this](std_msgs::msg::Int32::SharedPtr msg) {
      int idx = msg->data;
      if (idx >= 0 && idx < static_cast<int>(controllers_.size())) {
        // 컨트롤러 전환 시 현재 위치로 hold position 초기화 (target 공백 방지)
        if (auto_hold_position_ &&
            state_received_.load(std::memory_order_acquire)) {
          urtc::ControllerState hold_state{};
          {
            std::lock_guard lock(device_state_mutex_);
            int di = 0;
            for (const auto& [gname, ggroup] : controller_topic_configs_[idx].groups) {
              const int slot = group_slot_map_.at(gname);
              auto& dev = hold_state.devices[di];
              const auto& cache = device_states_[slot];
              dev.num_channels = cache.num_channels;
              dev.positions = cache.positions;
              dev.velocities = cache.velocities;
              dev.efforts = cache.efforts;
              dev.valid = cache.valid;
              ++di;
            }
            hold_state.num_devices = di;
          }
          hold_state.dt = 1.0 / control_rate_;
          controllers_[idx]->InitializeHoldPosition(hold_state);
        }
        active_controller_idx_.store(idx, std::memory_order_release);
        RCLCPP_INFO(get_logger(), "Switched to controller: %s",
                      controllers_[idx]->Name().data());
        std_msgs::msg::String ctrl_name_msg;
        ctrl_name_msg.data = std::string(controllers_[idx]->Name());
        active_ctrl_name_pub_->publish(ctrl_name_msg);
      } else {
        RCLCPP_WARN(get_logger(), "Invalid controller index: %d", idx);
      }
      },
      sub_options);

  controller_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ur5e/controller_gains", 10,
    [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      const int idx = active_controller_idx_.load(std::memory_order_acquire);
      controllers_[static_cast<std::size_t>(idx)]->UpdateGainsFromMsg(msg->data);
      RCLCPP_INFO(get_logger(), "Gains updated for %s",
        controllers_[static_cast<std::size_t>(idx)]->Name().data());
      },
      sub_options);

  request_gains_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/ur5e/request_gains", 10,
    [this](std_msgs::msg::Bool::SharedPtr /*msg*/) {
      const int idx = active_controller_idx_.load(std::memory_order_acquire);
      const auto gains = controllers_[static_cast<std::size_t>(idx)]->GetCurrentGains();
      std_msgs::msg::Float64MultiArray gains_msg;
      gains_msg.data = gains;
      current_gains_pub_->publish(gains_msg);
      RCLCPP_INFO(get_logger(), "Published current gains for %s (%zu values)",
        controllers_[static_cast<std::size_t>(idx)]->Name().data(), gains.size());
      },
      sub_options);
}

void RtControllerNode::CreatePublishers()
{
  // BEST_EFFORT + depth 1: minimises DDS overhead on the 500 Hz RT path.
  rclcpp::QoS cmd_qos{1};
  cmd_qos.best_effort();

  // Helper: create a publisher for a publish entry if not already created.
  auto create_pub = [&](const urtc::PublishTopicEntry & entry,
                        const std::string & group_name) {
    const int slot = group_slot_map_[group_name];

    switch (entry.role) {
      case urtc::PublishRole::kJointCommand: {
        if (joint_command_publishers_.count(entry.topic_name) > 0) { return; }
        JointCommandPublisherEntry jce;
        jce.publisher = create_publisher<rtc_msgs::msg::JointCommand>(
            entry.topic_name, cmd_qos);
        if (slot == 0) {
          jce.msg.joint_names = robot_joint_names_;
          jce.msg.values.resize(robot_joint_names_.size(), 0.0);
        } else {
          jce.msg.joint_names = hand_motor_names_;
          jce.msg.values.resize(hand_motor_names_.size(), 0.0);
        }
        jce.msg.command_type = "position";
        joint_command_publishers_[entry.topic_name] = std::move(jce);
        RCLCPP_INFO(get_logger(), "  Publish [%s/joint_command]: %s (JointCommand)",
                    group_name.c_str(), entry.topic_name.c_str());
        return;
      }
      default:
        break;
    }

    // Float64MultiArray publishers (kRos2Command, kTaskPosition, etc.)
    if (topic_publishers_.count(entry.topic_name) > 0) { return; }

    int data_size = entry.data_size;
    if (data_size <= 0) {
      switch (entry.role) {
        case urtc::PublishRole::kRos2Command:
          data_size = (slot == 0)
              ? static_cast<int>(robot_joint_names_.size())
              : static_cast<int>(hand_motor_names_.size());
          break;
        case urtc::PublishRole::kTaskPosition:
          data_size = 6;
          break;
        case urtc::PublishRole::kTrajectoryState:
        case urtc::PublishRole::kControllerState:
          data_size = 18;
          break;
        default:
          data_size = 6;
          break;
      }
    }

    PublisherEntry pe;
    pe.publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
        entry.topic_name, cmd_qos);
    pe.msg.data.resize(static_cast<std::size_t>(data_size), 0.0);
    topic_publishers_[entry.topic_name] = std::move(pe);

    const char * role_str = urtc::PublishRoleToString(entry.role);
    RCLCPP_INFO(get_logger(), "  Publish [%s/%s]: %s (size=%d)",
                group_name.c_str(), role_str, entry.topic_name.c_str(), data_size);
  };

  // ── Create publishers for all active device groups ────────────────────────
  for (const auto & tc : controller_topic_configs_) {
    for (const auto & [group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name)) continue;
      for (const auto & entry : group.publish) {
        create_pub(entry, group_name);
      }
    }
  }

  // ── Fixed publishers (always present) ─────────────────────────────────────
  estop_pub_ =
    create_publisher<std_msgs::msg::Bool>("/system/estop_status", 10);

  rclcpp::QoS latch_qos{1};
  latch_qos.transient_local();
  active_ctrl_name_pub_ =
    create_publisher<std_msgs::msg::String>("/ur5e/active_controller_name", latch_qos);

  current_gains_pub_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/ur5e/current_gains", 10);
}

// ── Expose topic configuration as read-only ROS2 parameters ─────────────────
// After CreatePublishers(), the final topic mapping is known.  Expose it as
// read-only parameters so that `ros2 param list/get` can introspect the
// active topic configuration at runtime.
//
// Parameter naming convention:
//   controllers.<ctrl_name>.subscribe.<role>   = topic name
//   controllers.<ctrl_name>.publish.<role>     = topic name
//
// All "controllers.*" parameters are rejected at set-time to preserve RT
// safety — topic routing must not change after initialisation.
void RtControllerNode::ExposeTopicParameters()
{
  for (std::size_t i = 0; i < controllers_.size(); ++i) {
    const auto & tc = controller_topic_configs_[i];
    const std::string prefix =
        "controllers." + std::string(controllers_[i]->Name());

    // All device groups (dynamic)
    for (const auto & [group_name, group] : tc.groups) {
      for (const auto & entry : group.subscribe) {
        const std::string param_name =
            prefix + "." + group_name + ".subscribe." + urtc::SubscribeRoleToString(entry.role);
        if (!has_parameter(param_name)) {
          declare_parameter(param_name, entry.topic_name);
        }
      }
      for (const auto & entry : group.publish) {
        const std::string param_name =
            prefix + "." + group_name + ".publish." + urtc::PublishRoleToString(entry.role);
        if (!has_parameter(param_name)) {
          declare_parameter(param_name, entry.topic_name);
        }
      }
    }
  }

  // ── Read-only guard: reject any runtime mutation of topic parameters ────
  param_callback_handle_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & params)
          -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto & p : params) {
          if (p.get_name().rfind("controllers.", 0) == 0) {
            result.successful = false;
            result.reason =
                "Topic parameters are read-only after initialisation";
            return result;
          }
        }
        result.successful = true;
        return result;
      });

  RCLCPP_INFO(get_logger(),
      "Topic parameters exposed (read-only) — use 'ros2 param list' to inspect");
}

void RtControllerNode::CreateTimers()
{
  // control_timer_ and timeout_timer_ removed — ControlLoop() + CheckTimeouts()
  // now run inside RtLoopEntry() (clock_nanosleep jthread, Core 2, SCHED_FIFO 90).

  // Drain the SPSC log ring buffer from the log thread (Core 4).
  // File I/O stays entirely out of the 500 Hz RT thread.
  drain_timer_ =
    create_wall_timer(10ms, [this]() {DrainLog();}, cb_group_log_);
}

// ── Subscription callbacks (unified per-device) ──────────────────────────────
void RtControllerNode::DeviceJointStateCallback(
    int device_slot, sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.empty()) return;

  auto& reorder = device_reorder_maps_[device_slot];
  if (!reorder.built && !msg->name.empty()) {
    BuildDeviceReorderMap(device_slot, msg->name);
  }

  std::lock_guard lock(device_state_mutex_);
  auto& ds = device_states_[device_slot];
  ds.num_channels = static_cast<int>(msg->position.size());

  if (reorder.built && !msg->name.empty()) {
    for (std::size_t src = 0; src < msg->position.size() &&
         src < reorder.reorder.size(); ++src) {
      const int idx = reorder.reorder[src];
      if (idx >= 0 && idx < urtc::kMaxDeviceChannels) {
        ds.positions[idx] = msg->position[src];
        if (src < msg->velocity.size()) ds.velocities[idx] = msg->velocity[src];
        if (src < msg->effort.size()) ds.efforts[idx] = msg->effort[src];
      }
    }
  } else {
    for (std::size_t i = 0; i < msg->position.size() &&
         i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
      ds.positions[i] = msg->position[i];
    }
    for (std::size_t i = 0; i < msg->velocity.size() &&
         i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
      ds.velocities[i] = msg->velocity[i];
    }
    for (std::size_t i = 0; i < msg->effort.size() &&
         i < static_cast<std::size_t>(urtc::kMaxDeviceChannels); ++i) {
      ds.efforts[i] = msg->effort[i];
    }
  }
  ds.valid = true;
  state_received_.store(true, std::memory_order_release);
}

void RtControllerNode::DeviceTargetCallback(
    int device_slot, std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.empty()) return;
  {
    std::lock_guard lock(target_mutex_);
    const int n = std::min(static_cast<int>(msg->data.size()),
                           urtc::kMaxDeviceChannels);
    for (int i = 0; i < n; ++i) {
      device_targets_[device_slot][i] = msg->data[i];
    }
  }
  target_received_.store(true, std::memory_order_release);
  int idx = active_controller_idx_.load(std::memory_order_acquire);
  controllers_[idx]->SetDeviceTarget(device_slot,
      std::span<const double>(msg->data.data(), msg->data.size()));
}

// ── 50 Hz watchdog (E-STOP) ───────────────────────────────────────────────────
void RtControllerNode::CheckTimeouts()
{
  if (device_timeouts_.empty()) { return; }

  const auto now = std::chrono::steady_clock::now();
  for (auto & dt : device_timeouts_) {
    if (!dt.received) continue;
    if ((now - dt.last_update) > dt.timeout && !IsGlobalEstopped()) {
      TriggerGlobalEstop(dt.group_name + "_timeout");
      return;
    }
  }
}

bool RtControllerNode::AllTimeoutDevicesReceived() const noexcept
{
  for (const auto & dt : device_timeouts_) {
    if (!dt.received) return false;
  }
  return true;
}

// ── 500 Hz control loop ───────────────────────────────────────────────────────
void RtControllerNode::ControlLoop()
{
  // ── Phase 0: tick start + readiness check ──────────────────────────────
  const auto t0 = std::chrono::steady_clock::now();

  if (!state_received_.load(std::memory_order_acquire)) {
    // 로봇 state가 아직 수신되지 않음 — 제어 불가
    if (!init_complete_ && init_timeout_ticks_ > 0 &&
        ++init_wait_ticks_ > init_timeout_ticks_) {
      RCLCPP_FATAL(get_logger(),
          "Initialization timeout (%.1f s): robot=%d, target=%d",
          static_cast<double>(init_timeout_ticks_) / control_rate_,
          state_received_.load(std::memory_order_relaxed) ? 1 : 0,
          target_received_.load(std::memory_order_relaxed) ? 1 : 0);
      TriggerGlobalEstop("init_timeout");
      rclcpp::shutdown();
    }
    return;
  }

  // Auto-hold: 외부 goal 미수신 시 현재 위치를 목표로 자동 설정
  if (!target_received_.load(std::memory_order_acquire)) {
    if (auto_hold_position_) {
      int idx = active_controller_idx_.load(std::memory_order_acquire);
      const auto & active_tc = controller_topic_configs_[
          static_cast<std::size_t>(idx)];

      // state는 수신됨 — 현재 위치를 읽어 target으로 초기화
      urtc::ControllerState hold_state{};
      {
        std::lock_guard lock(device_state_mutex_);
        int di = 0;
        for (const auto& [gname, ggroup] : active_tc.groups) {
          const int slot = group_slot_map_.at(gname);
          auto& dev = hold_state.devices[di];
          const auto& cache = device_states_[slot];
          dev.num_channels = cache.num_channels;
          dev.positions = cache.positions;
          dev.velocities = cache.velocities;
          dev.efforts = cache.efforts;
          dev.valid = cache.valid;
          ++di;
        }
        hold_state.num_devices = di;
      }
      hold_state.dt = 1.0 / control_rate_;

      controllers_[idx]->InitializeHoldPosition(hold_state);

      target_received_.store(true, std::memory_order_release);
      RCLCPP_INFO(get_logger(),
          "Auto-hold: initialized target from current position (%s)",
          controllers_[idx]->Name().data());
    } else {
      // auto_hold 비활성: 기존 동작 (timeout까지 대기)
      if (!init_complete_ && init_timeout_ticks_ > 0 &&
          ++init_wait_ticks_ > init_timeout_ticks_) {
        RCLCPP_FATAL(get_logger(),
            "Initialization timeout (%.1f s): robot=%d, target=%d",
            static_cast<double>(init_timeout_ticks_) / control_rate_,
            1,
            target_received_.load(std::memory_order_relaxed) ? 1 : 0);
        TriggerGlobalEstop("init_timeout");
        rclcpp::shutdown();
      }
      return;
    }
  }
  init_complete_ = true;

  // Global E-Stop: controller Compute() handles safe-position internally.
  // We still run the full loop to keep logging and timing active.

  // ── Phase 1: non-blocking state acquisition ────────────────────────────
  // try_lock avoids blocking the RT thread when the sensor thread holds the
  // mutex.  On contention the previous cycle's cached data is reused
  // (stale by at most 2 ms — acceptable).
  const int active_idx_for_tc = active_controller_idx_.load(std::memory_order_acquire);
  const auto & active_tc = controller_topic_configs_[
      static_cast<std::size_t>(active_idx_for_tc)];

  urtc::ControllerState state{};
  {
    std::unique_lock lock(device_state_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      cached_device_states_ = device_states_;
    }
  }
  int di = 0;
  for (const auto& [gname, ggroup] : active_tc.groups) {
    const int slot = group_slot_map_.at(gname);
    auto& dev = state.devices[di];
    const auto& cache = cached_device_states_[slot];
    dev.num_channels = cache.num_channels;
    dev.positions = cache.positions;
    dev.velocities = cache.velocities;
    dev.efforts = cache.efforts;
    dev.sensor_data = cache.sensor_data;
    dev.sensor_data_raw = cache.sensor_data_raw;
    dev.num_sensor_channels = cache.num_sensor_channels;
    dev.valid = cache.valid;
    ++di;
  }
  state.num_devices = di;

  {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      device_target_snapshots_ = device_targets_;
    }
  }
  state.dt = 1.0 / control_rate_;
  state.iteration = loop_count_;

  const auto t1 = std::chrono::steady_clock::now();  // end of state acquisition

  // ── Phase 2: compute control law ───────────────────────────────────────
  int active_idx = active_controller_idx_.load(std::memory_order_acquire);

  // Measure Compute() wall-clock time via ControllerTimingProfiler.
  const urtc::ControllerOutput output =
    timing_profiler_.MeasuredCompute(*controllers_[active_idx], state);

  const auto t2 = std::chrono::steady_clock::now();  // end of compute

  // ── Phase 3: push publish snapshot to SPSC buffer (lock-free, O(1)) ────
  // All ROS2 publish() calls are offloaded to the non-RT publish thread.
  {
    urtc::PublishSnapshot snap{};
    snap.command_type          = output.command_type;
    snap.actual_task_positions = output.actual_task_positions;
    snap.stamp_ns              = std::chrono::steady_clock::now()
                                   .time_since_epoch().count();
    snap.active_controller_idx = active_idx;

    // Per-group commands → group_commands slots
    int gi = 0;
    for (const auto & [gname, ggroup] : active_tc.groups) {
      if (gi >= urtc::PublishSnapshot::kMaxGroups) break;
      auto& gc = snap.group_commands[gi];
      const auto& dout = output.devices[gi];
      gc.num_channels = dout.num_channels;
      gc.commands = dout.commands;
      gc.goal_positions = dout.goal_positions;
      gc.target_positions = dout.target_positions;
      gc.target_velocities = dout.target_velocities;
      gc.actual_positions = state.devices[gi].positions;
      gc.actual_velocities = state.devices[gi].velocities;
      ++gi;
    }
    snap.num_groups = gi;

    static_cast<void>(publish_buffer_.Push(snap));
  }

  const auto t3 = std::chrono::steady_clock::now();  // end of publish

  // ── Phase 4: per-phase timing + log push ───────────────────────────────
  const double t_state_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  const double t_compute_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  const double t_publish_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();
  const double t_total_us =
      std::chrono::duration<double, std::micro>(t3 - t0).count();

  // Jitter = |actual_period - expected_period|
  double jitter_us = 0.0;
  if (loop_count_ > 0) {
    const double actual_period_us =
        std::chrono::duration<double, std::micro>(t0 - prev_loop_start_).count();
    jitter_us = std::abs(actual_period_us - budget_us_);
  }
  prev_loop_start_ = t0;

  // Compute overrun — ControlLoop() itself exceeded tick budget.
  // (Distinguished from RT loop overrun which includes sleep jitter.)
  if (t_total_us > budget_us_) {
    compute_overrun_count_.fetch_add(1, std::memory_order_relaxed);
  }

  // Push log entry to the SPSC ring buffer — O(1), no syscall.
  // DrainLog() (log thread, Core 4) pops entries and writes the CSV file.
  if (enable_logging_) {
    if (loop_count_ == 0) {
      log_start_time_ = t0;
    }
    const double timestamp =
        std::chrono::duration<double>(t0 - log_start_time_).count();

    urtc::LogEntry entry{};
    entry.timestamp          = timestamp;
    entry.t_state_acquire_us = t_state_us;
    entry.t_compute_us       = t_compute_us;
    entry.t_publish_us       = t_publish_us;
    entry.t_total_us         = t_total_us;
    entry.jitter_us          = jitter_us;
    entry.actual_task_positions = output.actual_task_positions;
    entry.command_type       = output.command_type;

    // Per-device logging
    for (int dvi = 0; dvi < state.num_devices; ++dvi) {
      auto& dl = entry.devices[dvi];
      const auto& dout = output.devices[dvi];
      const auto& dstate = state.devices[dvi];
      dl.num_channels = dout.num_channels;
      dl.valid = dstate.valid;
      for (int j = 0; j < dout.num_channels; ++j) {
        dl.goal_positions[j] = dout.goal_positions[j];
        dl.actual_positions[j] = dstate.positions[j];
        dl.actual_velocities[j] = dstate.velocities[j];
        dl.efforts[j] = dstate.efforts[j];
        dl.commands[j] = dout.commands[j];
        dl.trajectory_positions[j] = dout.target_positions[j];
        dl.trajectory_velocities[j] = dout.target_velocities[j];
      }
      dl.num_sensor_channels = dstate.num_sensor_channels;
      for (int j = 0; j < dstate.num_sensor_channels; ++j) {
        dl.sensor_data[j] = static_cast<float>(dstate.sensor_data[j]);
        dl.sensor_data_raw[j] = static_cast<float>(dstate.sensor_data_raw[j]);
      }
    }
    entry.num_devices = state.num_devices;
    static_cast<void>(log_buffer_.Push(entry));  // silently drops if buffer is full
  }

  ++loop_count_;
  // Signal the log thread to print timing summary every 1 000 iterations.
  // The actual std::string allocation + RCLCPP_INFO happens in DrainLog()
  // on the non-RT log thread (Core 4), keeping the RT path free of heap
  // allocation and potential syscalls.
  if (loop_count_ % 1000 == 0) {
    print_timing_summary_.store(true, std::memory_order_relaxed);
  }
}

// File I/O and diagnostic logging stay exclusively in the log thread (Core 4).
void RtControllerNode::DrainLog()
{
  if (logger_) {
    logger_->DrainBuffer(log_buffer_);
  }

  // Print timing summary when signalled by the RT thread.
  // std::string allocation + RCLCPP_INFO happen here (non-RT), not in ControlLoop().
  if (print_timing_summary_.exchange(false, std::memory_order_relaxed)) {
    int idx = active_controller_idx_.load(std::memory_order_acquire);
    const auto overruns = overrun_count_.load(std::memory_order_relaxed);
    const auto compute_overruns = compute_overrun_count_.load(std::memory_order_relaxed);
    const auto skips = skip_count_.load(std::memory_order_relaxed);
    const auto log_drops = log_buffer_.drop_count();
    const auto pub_drops = publish_buffer_.drop_count();
    RCLCPP_INFO(get_logger(),
        "%s  overruns=%lu  compute_overruns=%lu  skips=%lu  "
        "log_drops=%lu  pub_drops=%lu",
        timing_profiler_
          .Summary(std::string(controllers_[static_cast<std::size_t>(idx)]->Name()))
          .c_str(),
        static_cast<unsigned long>(overruns),
        static_cast<unsigned long>(compute_overruns),
        static_cast<unsigned long>(skips),
        static_cast<unsigned long>(log_drops),
        static_cast<unsigned long>(pub_drops));
  }
}

// ── RT loop (clock_nanosleep) ──────────────────────────────────────────────────
//
// Replaces create_wall_timer() with a tight POSIX absolute-time sleep loop.
// Eliminates ~50-200 µs of executor/epoll dispatch jitter.  Overrun recovery
// skips missed ticks and realigns to the next period boundary — no burst.
//
// Threading: runs as std::jthread on Core 2, SCHED_FIFO 90.
//            CheckTimeouts() inlined every 10th tick (50 Hz).

void RtControllerNode::RtLoopEntry(const urtc::ThreadConfig& cfg)
{
  urtc::ApplyThreadConfig(cfg);

  const int64_t period_ns = static_cast<int64_t>(1.0e9 / control_rate_);
  struct timespec next_wake{};
  clock_gettime(CLOCK_MONOTONIC, &next_wake);
  rt_loop_running_.store(true, std::memory_order_release);

  uint32_t timeout_tick = 0;

  while (rt_loop_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    // Advance to next absolute wake time
    next_wake.tv_nsec += period_ns;
    if (next_wake.tv_nsec >= 1'000'000'000L) {
      next_wake.tv_sec  += next_wake.tv_nsec / 1'000'000'000L;
      next_wake.tv_nsec %= 1'000'000'000L;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake, nullptr);

    // ── Overrun detection & recovery ──────────────────────────────────────
    // If ControlLoop() took longer than one period, next_wake is already in
    // the past.  clock_nanosleep returns immediately → burst.  We detect
    // this by checking how far behind we are and skip missed ticks.
    struct timespec now_ts{};
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    const int64_t now_ns  = now_ts.tv_sec  * 1'000'000'000L + now_ts.tv_nsec;
    const int64_t wake_ns = next_wake.tv_sec * 1'000'000'000L + next_wake.tv_nsec;
    const int64_t lag_ns  = now_ns - wake_ns;

    if (lag_ns > period_ns) {
      // Skip missed ticks — realign to the next period boundary
      const int64_t missed_ticks = lag_ns / period_ns;
      const int64_t advance_ns   = missed_ticks * period_ns;
      next_wake.tv_nsec += static_cast<long>(advance_ns);
      if (next_wake.tv_nsec >= 1'000'000'000L) {
        next_wake.tv_sec  += next_wake.tv_nsec / 1'000'000'000L;
        next_wake.tv_nsec %= 1'000'000'000L;
      }
      overrun_count_.fetch_add(1, std::memory_order_relaxed);
      skip_count_.fetch_add(static_cast<uint64_t>(missed_ticks),
                            std::memory_order_relaxed);

      // Consecutive overrun safety — E-STOP if sustained
      const auto consecutive =
          consecutive_overruns_.fetch_add(1, std::memory_order_relaxed) + 1;
      if (consecutive >= kMaxConsecutiveOverruns) {
        TriggerGlobalEstop("consecutive_overrun");
      }
    } else {
      // Normal tick — reset consecutive counter
      consecutive_overruns_.store(0, std::memory_order_relaxed);
    }

    ControlLoop();

    // CheckTimeouts() — every 10th tick (= 50 Hz)
    if (enable_estop_ && ++timeout_tick % 10 == 0) {
      CheckTimeouts();
    }
  }
}

void RtControllerNode::StartRtLoop(const urtc::ThreadConfig& rt_cfg)
{
  rt_loop_thread_ = std::jthread([this, rt_cfg]() {
    RtLoopEntry(rt_cfg);
  });
}

void RtControllerNode::StopRtLoop()
{
  rt_loop_running_.store(false, std::memory_order_release);
  if (rt_loop_thread_.joinable()) {
    rt_loop_thread_.join();
  }
}

// ── Publish offload thread ────────────────────────────────────────────────────
//
// Drains the SPSC publish buffer and performs all ROS2 publish() calls.
// Runs on a non-RT core (Core 5/6, SCHED_OTHER nice -3).
// All DDS serialization, string allocation, and sendto() syscalls happen here,
// keeping the RT path free of unbounded-latency operations.

void RtControllerNode::PublishLoopEntry(const urtc::ThreadConfig& cfg)
{
  urtc::ApplyThreadConfig(cfg);
  publish_running_.store(true, std::memory_order_release);

  urtc::PublishSnapshot snap{};

  while (publish_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (!publish_buffer_.Pop(snap)) {
      sched_yield();
      continue;
    }

    const auto & active_tc = controller_topic_configs_[
        static_cast<std::size_t>(snap.active_controller_idx)];

    // Shared timestamp for JointCommand messages
    const auto sec  = static_cast<int32_t>(snap.stamp_ns / 1'000'000'000L);
    const auto nsec = static_cast<uint32_t>(snap.stamp_ns % 1'000'000'000L);
    const char * cmd_type_str =
        (snap.command_type == urtc::CommandType::kTorque) ? "torque" : "position";

    // Helper: publish a single topic entry from snapshot data
    auto publish_entry = [&](const urtc::PublishTopicEntry & pt, int group_idx) {
      switch (pt.role) {
        case urtc::PublishRole::kJointCommand: {
          auto jc_it = joint_command_publishers_.find(pt.topic_name);
          if (jc_it == joint_command_publishers_.end()) { return; }
          auto & jce = jc_it->second;
          jce.msg.header.stamp.sec = sec;
          jce.msg.header.stamp.nanosec = nsec;
          jce.msg.command_type = cmd_type_str;
          const auto & gc = snap.group_commands[group_idx];
          for (int i = 0; i < gc.num_channels
                   && i < static_cast<int>(jce.msg.values.size()); ++i) {
            jce.msg.values[i] = gc.commands[i];
          }
          jce.publisher->publish(jce.msg);
          return;
        }
        default:
          break;
      }

      auto it = topic_publishers_.find(pt.topic_name);
      if (it == topic_publishers_.end()) { return; }
      auto & pe = it->second;

      switch (pt.role) {
        case urtc::PublishRole::kRos2Command: {
          const auto & gc = snap.group_commands[group_idx];
          for (int i = 0; i < gc.num_channels
                   && i < static_cast<int>(pe.msg.data.size()); ++i) {
            pe.msg.data[i] = gc.commands[i];
          }
          pe.publisher->publish(pe.msg);
          break;
        }
        case urtc::PublishRole::kTaskPosition:
          std::copy(snap.actual_task_positions.begin(),
                    snap.actual_task_positions.end(),
                    pe.msg.data.begin());
          pe.publisher->publish(pe.msg);
          break;
        case urtc::PublishRole::kTrajectoryState: {
          const auto & gc = snap.group_commands[group_idx];
          const int n = std::min(gc.num_channels, 6);
          for (int i = 0; i < n; ++i) {
            pe.msg.data[i] = gc.goal_positions[i];
            pe.msg.data[i + 6] = gc.target_positions[i];
            pe.msg.data[i + 12] = gc.target_velocities[i];
          }
          pe.publisher->publish(pe.msg);
          break;
        }
        case urtc::PublishRole::kControllerState: {
          const auto & gc = snap.group_commands[group_idx];
          const int n = std::min(gc.num_channels, 6);
          for (int i = 0; i < n; ++i) {
            pe.msg.data[i] = gc.actual_positions[i];
            pe.msg.data[i + 6] = gc.actual_velocities[i];
            pe.msg.data[i + 12] = gc.commands[i];
          }
          pe.publisher->publish(pe.msg);
          break;
        }
        default:
          break;
      }
    };

    // Publish all device groups uniformly
    int group_idx = 0;
    for (const auto & [group_name, group] : active_tc.groups) {
      for (const auto & pt : group.publish) {
        publish_entry(pt, group_idx);
      }
      ++group_idx;
    }
  }
}

void RtControllerNode::StartPublishLoop(const urtc::ThreadConfig& pub_cfg)
{
  publish_thread_ = std::jthread([this, pub_cfg]() {
    PublishLoopEntry(pub_cfg);
  });
}

void RtControllerNode::StopPublishLoop()
{
  publish_running_.store(false, std::memory_order_release);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void RtControllerNode::PublishEstopStatus(bool estopped)
{
  std_msgs::msg::Bool msg;
  msg.data = estopped;
  estop_pub_->publish(msg);
}

// ── Device sensor callback (unified per-device) ──────────────────────────────
void RtControllerNode::DeviceSensorCallback(
    int device_slot, std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard lock(device_state_mutex_);
  auto& ds = device_states_[device_slot];
  for (std::size_t i = 0; i < msg->data.size() &&
       i < static_cast<std::size_t>(urtc::kMaxSensorChannels); ++i) {
    ds.sensor_data[i] = static_cast<int32_t>(msg->data[i]);
  }
  ds.num_sensor_channels = static_cast<int>(
      std::min(msg->data.size(), static_cast<std::size_t>(urtc::kMaxSensorChannels)));
}

// ── Global E-Stop ──────────────────────────────────────────────────────────────
void RtControllerNode::TriggerGlobalEstop(std::string_view reason) noexcept
{
  // Idempotent — only the first call logs and propagates.
  bool expected = false;
  if (!global_estop_.compare_exchange_strong(expected, true,
          std::memory_order_acq_rel, std::memory_order_relaxed)) {
    return;  // already estopped
  }

  estop_reason_ = std::string(reason);

  // Propagate to all controllers — TriggerEstop is safe from any thread.
  for (auto & ctrl : controllers_) {
    ctrl->TriggerEstop();
    ctrl->SetHandEstop(true);
  }
  PublishEstopStatus(true);

  RCLCPP_ERROR(get_logger(), "GLOBAL E-STOP triggered: %s", estop_reason_.c_str());
}

void RtControllerNode::ClearGlobalEstop() noexcept
{
  if (!global_estop_.load(std::memory_order_acquire)) {
    return;
  }
  global_estop_.store(false, std::memory_order_release);

  for (auto & ctrl : controllers_) {
    ctrl->ClearEstop();
    ctrl->SetHandEstop(false);
  }
  PublishEstopStatus(false);

  RCLCPP_INFO(get_logger(), "GLOBAL E-STOP cleared (was: %s)", estop_reason_.c_str());
  estop_reason_.clear();
}

// ── Joint name loading & URDF validation (v5.14.0) ──────────────────────────

void RtControllerNode::LoadAndValidateJointNames()
{
  // 1. YAML에서 이름 로드 (hand_joint_names → hand_motor_names fallback)
  robot_joint_names_ = get_parameter("robot_joint_names").as_string_array();
  hand_motor_names_  = get_parameter("hand_joint_names").as_string_array();
  if (hand_motor_names_.empty()) {
    hand_motor_names_ = get_parameter("hand_motor_names").as_string_array();
  }
  hand_sensor_names_ = get_parameter("hand_sensor_names").as_string_array();
  if (hand_sensor_names_.empty()) {
    hand_sensor_names_ = get_parameter("hand_fingertip_names").as_string_array();
  }
  fingertip_names_   = hand_sensor_names_;

  // 비어있으면 기본값 사용
  if (robot_joint_names_.empty()) {
    robot_joint_names_ = urtc::kDefaultRobotJointNames;
    RCLCPP_WARN(get_logger(),
                "No robot_joint_names in YAML — using defaults");
  }
  if (hand_motor_names_.empty()) {
    hand_motor_names_ = urtc::kDefaultHandMotorNames;
  }
  if (fingertip_names_.empty()) {
    fingertip_names_ = urtc::kDefaultFingertipNames;
  }

  // 2. 개수 검증 (warn only — runtime count may differ from compile-time default)
  if (robot_joint_names_.size() != static_cast<std::size_t>(urtc::kNumRobotJoints)) {
    RCLCPP_WARN(get_logger(),
                "robot_joint_names has %zu entries (default %d) — using YAML count",
                robot_joint_names_.size(), urtc::kNumRobotJoints);
  }

  // 3. URDF active joint 검증
  std::string urdf_path;
  try {
    urdf_path = ament_index_cpp::get_package_share_directory("ur5e_description") +
                "/robots/ur5e/urdf/ur5e.urdf";
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Cannot resolve URDF path — skipping joint name validation");
    return;
  }

  try {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);

    // Pinocchio model_.names[0] = "universe" (고정), [1..njoints-1] = 실제 조인트
    std::vector<std::string> urdf_joint_names;
    for (int j = 1; j < model.njoints; ++j) {
      urdf_joint_names.push_back(model.names[static_cast<std::size_t>(j)]);
    }

    // 비교: YAML의 각 이름이 URDF에 존재하는지
    bool all_found = true;
    bool order_match = true;
    for (std::size_t i = 0; i < robot_joint_names_.size(); ++i) {
      auto it = std::find(urdf_joint_names.begin(), urdf_joint_names.end(),
                          robot_joint_names_[i]);
      if (it == urdf_joint_names.end()) {
        RCLCPP_ERROR(get_logger(),
                     "YAML joint '%s' NOT FOUND in URDF",
                     robot_joint_names_[i].c_str());
        all_found = false;
      } else {
        const auto urdf_idx = static_cast<std::size_t>(
            std::distance(urdf_joint_names.begin(), it));
        if (urdf_idx != i) {
          order_match = false;
        }
      }
    }

    if (!all_found) {
      // 사용 가능한 URDF 조인트 목록 출력
      std::string avail;
      for (const auto& n : urdf_joint_names) { avail += "  " + n + "\n"; }
      RCLCPP_ERROR(get_logger(),
                   "Joint name mismatch — falling back to defaults.\n"
                   "Available URDF joints:\n%s", avail.c_str());
      robot_joint_names_ = urtc::kDefaultRobotJointNames;
    } else if (!order_match) {
      // 이름은 모두 존재하지만 순서가 다름
      std::string remap_info;
      for (std::size_t i = 0; i < robot_joint_names_.size(); ++i) {
        auto it = std::find(urdf_joint_names.begin(), urdf_joint_names.end(),
                            robot_joint_names_[i]);
        const auto urdf_idx = static_cast<std::size_t>(
            std::distance(urdf_joint_names.begin(), it));
        remap_info += "  YAML[" + std::to_string(i) + "] \"" +
                      robot_joint_names_[i] + "\" → URDF index " +
                      std::to_string(urdf_idx) + "\n";
      }
      RCLCPP_WARN(get_logger(),
                   "Joint name ORDER mismatch between YAML and URDF:\n%s"
                   "Verify gains/limits match the YAML order.",
                   remap_info.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "Joint names validated: YAML matches URDF (%zu/%zu)",
                  robot_joint_names_.size(), urdf_joint_names.size());
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "URDF validation failed: %s", e.what());
  }

  // 로그 출력
  RCLCPP_INFO(get_logger(), "Robot joints: [%s]",
              [&]{
                std::string s;
                for (std::size_t i = 0; i < robot_joint_names_.size(); ++i) {
                  if (i > 0) s += ", ";
                  s += robot_joint_names_[i];
                }
                return s;
              }().c_str());
  // Hand motor 개수 검증 (warn only — runtime count may differ from compile-time default)
  if (hand_motor_names_.size() != static_cast<std::size_t>(urtc::kNumHandMotors)) {
    RCLCPP_WARN(get_logger(),
                "hand_motor_names has %zu entries (default %d) — using YAML count",
                hand_motor_names_.size(), urtc::kNumHandMotors);
  }

  RCLCPP_INFO(get_logger(), "Hand motors (%zu): [%s]",
              hand_motor_names_.size(),
              [&]{
                std::string s;
                for (std::size_t i = 0; i < hand_motor_names_.size(); ++i) {
                  if (i > 0) s += ", ";
                  s += hand_motor_names_[i];
                }
                return s;
              }().c_str());
  RCLCPP_INFO(get_logger(), "Fingertips (%zu): [%s]",
              fingertip_names_.size(),
              [&]{
                std::string s;
                for (std::size_t i = 0; i < fingertip_names_.size(); ++i) {
                  if (i > 0) s += ", ";
                  s += fingertip_names_[i];
                }
                return s;
              }().c_str());
}

void RtControllerNode::BuildDeviceReorderMap(
    int device_slot, const std::vector<std::string>& msg_names)
{
  // Determine reference names based on slot
  const auto& ref_names = (device_slot == 0) ? robot_joint_names_ : hand_motor_names_;
  if (ref_names.empty()) return;

  auto& map = device_reorder_maps_[device_slot];
  map.reorder.resize(msg_names.size(), -1);

  for (std::size_t msg_i = 0; msg_i < msg_names.size(); ++msg_i) {
    for (std::size_t ref_i = 0; ref_i < ref_names.size(); ++ref_i) {
      if (msg_names[msg_i] == ref_names[ref_i]) {
        map.reorder[msg_i] = static_cast<int>(ref_i);
        break;
      }
    }
  }
  map.built = true;
  RCLCPP_INFO(get_logger(), "Built device reorder map for slot %d (%zu names)",
              device_slot, msg_names.size());
}
