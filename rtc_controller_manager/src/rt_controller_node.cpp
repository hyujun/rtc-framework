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
  declare_parameter("robot_timeout_ms", 100.0);
  declare_parameter("enable_estop", true);
  declare_parameter("enable_status_monitor", false);
  declare_parameter("init_timeout_sec", 5.0);
  declare_parameter("auto_hold_position", true);
  declare_parameter("initial_controller", "joint_pd_controller");

  // ── 디바이스 활성화 플래그 (글로벌 기본값) ────────────────────────────────
  declare_parameter("enable_ur5e", true);
  declare_parameter("enable_hand", false);
  global_device_flags_.enable_ur5e = get_parameter("enable_ur5e").as_bool();
  global_device_flags_.enable_hand = get_parameter("enable_hand").as_bool();


  // Joint/Motor/Fingertip names (v5.14.0 named messaging)
  declare_parameter("robot_joint_names", std::vector<std::string>{});
  declare_parameter("hand_motor_names", std::vector<std::string>{});
  declare_parameter("hand_fingertip_names", std::vector<std::string>{});


  control_rate_ = get_parameter("control_rate").as_double();
  budget_us_ = 1.0e6 / control_rate_;  // tick budget in µs (e.g., 2000.0 at 500 Hz)

  const double init_timeout_sec = get_parameter("init_timeout_sec").as_double();
  init_timeout_ticks_ = static_cast<uint64_t>(init_timeout_sec * control_rate_);
  auto_hold_position_ = get_parameter("auto_hold_position").as_bool();
  enable_logging_ = get_parameter("enable_logging").as_bool();
  enable_estop_ = get_parameter("enable_estop").as_bool();
  enable_status_monitor_ = get_parameter("enable_status_monitor").as_bool();

  if (enable_logging_) {
    // 세션 디렉토리 결정: log_dir 파라미터 → UR5E_SESSION_DIR 환경변수 → 자체 생성
    const std::string log_dir_param = get_parameter("log_dir").as_string();
    const int max_sessions = get_parameter("max_log_sessions").as_int();

    std::filesystem::path session_dir;
    if (!log_dir_param.empty()) {
      // Launch 파일이 세션 디렉토리를 직접 전달한 경우
      session_dir = std::filesystem::path(log_dir_param);
      std::filesystem::create_directories(session_dir);
      urtc::EnsureSessionSubdirs(session_dir);
    } else {
      // Standalone 실행: 자체 세션 디렉토리 생성
      session_dir = ResolveAndSetupSessionDir();
    }

    // 세션 폴더 cleanup (logging_data 루트 기준)
    const auto logging_root = session_dir.parent_path();
    urtc::CleanupOldSessions(logging_root, max_sessions);

    const bool enable_timing = get_parameter("enable_timing_log").as_bool();
    const bool enable_robot  = get_parameter("enable_robot_log").as_bool()
                               && global_device_flags_.enable_ur5e;
    const bool enable_hand   = get_parameter("enable_hand_log").as_bool()
                               && global_device_flags_.enable_hand;
    const auto ctrl_dir = session_dir / "controller";
    std::filesystem::create_directories(ctrl_dir);

    const std::string timing_path = enable_timing
        ? (ctrl_dir / "timing_log.csv").string() : "";
    const std::string robot_path = enable_robot
        ? (ctrl_dir / "robot_log.csv").string() : "";
    const std::string hand_path = enable_hand
        ? (ctrl_dir / "hand_log.csv").string() : "";

    // joint/motor/fingertip names는 아직 로드 전일 수 있으므로 YAML에서 직접 읽기
    const auto yaml_joint_names = get_parameter("robot_joint_names").as_string_array();
    const auto yaml_motor_names = get_parameter("hand_motor_names").as_string_array();
    const auto yaml_fingertip_names = get_parameter("hand_fingertip_names").as_string_array();
    logger_ = std::make_unique<urtc::DataLogger>(
        timing_path, robot_path, hand_path,
        yaml_joint_names, yaml_motor_names, yaml_fingertip_names);
    RCLCPP_INFO(get_logger(),
        "Logging to: %s/controller/ (max_sessions=%d)",
        session_dir.string().c_str(), max_sessions);
  }

  // E-STOP 타임아웃: ur5e 비활성 시 robot_timeout 비활성화
  if (global_device_flags_.enable_ur5e) {
    robot_timeout_ = std::chrono::milliseconds(
        static_cast<int>(get_parameter("robot_timeout_ms").as_double()));
  } else {
    robot_timeout_ = std::chrono::milliseconds(0);
  }

  // Hand 비활성 시 로그
  if (!global_device_flags_.enable_hand) {
    RCLCPP_INFO(get_logger(), "Hand disabled (enable_hand=false)");
  }

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
  BuildHandStateIndexMap(hand_motor_names_);

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

  // Cache per-controller topic configs and resolve device flags
  controller_topic_configs_.reserve(controllers_.size());
  controller_device_flags_.reserve(controllers_.size());
  for (const auto & ctrl : controllers_) {
    controller_topic_configs_.push_back(ctrl->GetTopicConfig());
    controller_device_flags_.push_back(
        ResolveDeviceFlags(ctrl->GetPerControllerDeviceFlags()));

    const auto & tc = controller_topic_configs_.back();
    const auto & df = controller_device_flags_.back();
    RCLCPP_INFO(get_logger(),
        "Controller '%s': ur5e=%s (%zu+%zu topics), hand=%s (%zu+%zu topics)",
        ctrl->Name().data(),
        df.enable_ur5e ? "ON" : "OFF",
        tc.ur5e.subscribe.size(), tc.ur5e.publish.size(),
        df.enable_hand ? "ON" : "OFF",
        tc.hand.subscribe.size(), tc.hand.publish.size());
  }

  // ur5e가 어떤 컨트롤러에서도 활성화되지 않으면 joint_states 대기 skip
  {
    bool any_ur5e = false;
    for (const auto & df : controller_device_flags_) {
      if (df.enable_ur5e) { any_ur5e = true; break; }
    }
    if (!any_ur5e) {
      state_received_.store(true, std::memory_order_release);
      target_received_.store(true, std::memory_order_release);
      RCLCPP_INFO(get_logger(), "No controller enables ur5e — skipping init wait");
    }
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

  // ── Determine which device groups are enabled by any controller ──────────
  // Topics are created at startup for the union of all enabled controllers,
  // so runtime controller switching finds pre-existing topics.
  bool any_ur5e = false, any_hand = false;
  for (const auto & df : controller_device_flags_) {
    if (df.enable_ur5e) { any_ur5e = true; }
    if (df.enable_hand) { any_hand = true; }
  }

  // ── Collect unique subscribe topics from all controllers ──────────────────
  std::set<std::string> created_joint_state_topics;
  std::set<std::string> created_target_topics;

  for (std::size_t ci = 0; ci < controller_topic_configs_.size(); ++ci) {
    const auto & tc = controller_topic_configs_[ci];

    // ur5e 토픽: any controller enables ur5e → 생성
    if (any_ur5e) {
      for (const auto & entry : tc.ur5e.subscribe) {
        switch (entry.role) {
          case urtc::SubscribeRole::kState:
            if (created_joint_state_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<sensor_msgs::msg::JointState>(
                entry.topic_name, 10,
                [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                  JointStateCallback(std::move(msg));
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [ur5e/state]: %s",
                          entry.topic_name.c_str());
            }
            break;
          case urtc::SubscribeRole::kTarget:
            if (created_target_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
                entry.topic_name, 10,
                [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                  RobotTargetCallback(std::move(msg));
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [ur5e/target]: %s",
                          entry.topic_name.c_str());
            }
            break;
          default:
            break;
        }
      }
    }

    // hand 토픽: any controller enables hand → 생성
    if (any_hand) {
      for (const auto & entry : tc.hand.subscribe) {
        switch (entry.role) {
          case urtc::SubscribeRole::kState:
            if (created_joint_state_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<sensor_msgs::msg::JointState>(
                entry.topic_name, 10,
                [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                  if (msg->position.size() < 1) { return; }
                  std::lock_guard lock(hand_state_mutex_);

                  if (hand_state_map_built_) {
                    const std::size_t n = std::min(hand_state_reorder_.size(),
                                                   msg->position.size());
                    for (std::size_t src_i = 0; src_i < n; ++src_i) {
                      const int idx = hand_state_reorder_[src_i];
                      if (idx >= 0 && idx < urtc::kNumHandMotors) {
                        const auto uidx = static_cast<std::size_t>(idx);
                        cached_hand_state_.motor_positions[uidx] =
                            static_cast<float>(msg->position[src_i]);
                        if (src_i < msg->velocity.size()) {
                          cached_hand_state_.motor_velocities[uidx] =
                              static_cast<float>(msg->velocity[src_i]);
                        }
                      }
                    }
                  } else {
                    for (std::size_t i = 0; i < urtc::kNumHandMotors
                             && i < msg->position.size(); ++i) {
                      cached_hand_state_.motor_positions[i] =
                          static_cast<float>(msg->position[i]);
                    }
                    for (std::size_t i = 0; i < urtc::kNumHandMotors
                             && i < msg->velocity.size(); ++i) {
                      cached_hand_state_.motor_velocities[i] =
                          static_cast<float>(msg->velocity[i]);
                    }
                  }
                  cached_hand_state_.valid = true;
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [hand/state]: %s",
                          entry.topic_name.c_str());
            }
            break;
          case urtc::SubscribeRole::kSensorState:
            if (created_joint_state_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
                entry.topic_name, 10,
                [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                  std::lock_guard lock(hand_state_mutex_);
                  for (std::size_t i = 0; i < msg->data.size()
                           && i < cached_hand_state_.sensor_data.size(); ++i) {
                    cached_hand_state_.sensor_data[i] =
                        static_cast<int32_t>(msg->data[i]);
                  }
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [hand/sensor_state]: %s",
                          entry.topic_name.c_str());
            }
            break;
          case urtc::SubscribeRole::kTarget:
            if (created_target_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
                entry.topic_name, 10,
                [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                  HandTargetCallback(std::move(msg));
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [hand/target]: %s",
                          entry.topic_name.c_str());
            }
            break;
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
            std::lock_guard lock(state_mutex_);
            hold_state.robot.positions = current_positions_;
            hold_state.robot.velocities = current_velocities_;
            hold_state.robot.torques = current_torques_;
          }
          hold_state.robot.dt = 1.0 / control_rate_;
          hold_state.dt = hold_state.robot.dt;
          {
            std::lock_guard lock(hand_state_mutex_);
            hold_state.hand = cached_hand_state_;
          }
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

  // ── Determine which device groups are enabled by any controller ──────────
  bool any_ur5e = false, any_hand = false;
  for (const auto & df : controller_device_flags_) {
    if (df.enable_ur5e) { any_ur5e = true; }
    if (df.enable_hand) { any_hand = true; }
  }

  // Helper: create a publisher for a publish entry if not already created.
  auto create_pub = [&](const urtc::PublishTopicEntry & entry,
                        const char * device_prefix,
                        bool is_hand) {
    switch (entry.role) {
      case urtc::PublishRole::kJointCommand: {
        // JointCommand publisher (rtc_msgs/JointCommand)
        if (joint_command_publishers_.count(entry.topic_name) > 0) { return; }
        JointCommandPublisherEntry jce;
        jce.publisher = create_publisher<rtc_msgs::msg::JointCommand>(
            entry.topic_name, cmd_qos);
        if (!is_hand) {
          jce.msg.joint_names = robot_joint_names_;
          jce.msg.values.resize(urtc::kNumRobotJoints, 0.0);
        } else {
          jce.msg.joint_names = hand_motor_names_;
          jce.msg.values.resize(urtc::kNumHandMotors, 0.0);
        }
        jce.msg.command_type = "position";
        joint_command_publishers_[entry.topic_name] = std::move(jce);
        RCLCPP_INFO(get_logger(), "  Publish [%s/joint_command]: %s (JointCommand)",
                    device_prefix, entry.topic_name.c_str());
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
          data_size = is_hand ? urtc::kNumHandMotors : urtc::kNumRobotJoints;
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
                device_prefix, role_str, entry.topic_name.c_str(), data_size);
  };

  // ── Create publishers for enabled device groups ───────────────────────────
  for (const auto & tc : controller_topic_configs_) {
    if (any_ur5e) {
      for (const auto & entry : tc.ur5e.publish) {
        create_pub(entry, "ur5e", false);
      }
    }
    if (any_hand) {
      for (const auto & entry : tc.hand.publish) {
        create_pub(entry, "hand", true);
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

    // ur5e 디바이스 그룹
    for (const auto & entry : tc.ur5e.subscribe) {
      const std::string param_name =
          prefix + ".ur5e.subscribe." + urtc::SubscribeRoleToString(entry.role);
      if (!has_parameter(param_name)) {
        declare_parameter(param_name, entry.topic_name);
      }
    }
    for (const auto & entry : tc.ur5e.publish) {
      const std::string param_name =
          prefix + ".ur5e.publish." + urtc::PublishRoleToString(entry.role);
      if (!has_parameter(param_name)) {
        declare_parameter(param_name, entry.topic_name);
      }
    }

    // hand 디바이스 그룹
    for (const auto & entry : tc.hand.subscribe) {
      const std::string param_name =
          prefix + ".hand.subscribe." + urtc::SubscribeRoleToString(entry.role);
      if (!has_parameter(param_name)) {
        declare_parameter(param_name, entry.topic_name);
      }
    }
    for (const auto & entry : tc.hand.publish) {
      const std::string param_name =
          prefix + ".hand.publish." + urtc::PublishRoleToString(entry.role);
      if (!has_parameter(param_name)) {
        declare_parameter(param_name, entry.topic_name);
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

// ── Subscription callbacks ────────────────────────────────────────────────────
void RtControllerNode::JointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < static_cast<std::size_t>(urtc::kNumRobotJoints)) {
    return;
  }

  // 이름 기반 매핑: msg->name이 있으면 첫 수신 시 맵 빌드 (이후 인덱스만 사용)
  if (!msg->name.empty() && !joint_state_map_built_) {
    BuildJointStateIndexMap(msg->name);
  }

  {
    std::lock_guard lock(state_mutex_);

    if (joint_state_map_built_ && !msg->name.empty()) {
      // 이름 기반: msg의 각 인덱스를 내부 인덱스로 매핑
      for (std::size_t msg_i = 0; msg_i < msg->position.size() &&
           msg_i < joint_state_reorder_.size(); ++msg_i) {
        const int idx = joint_state_reorder_[msg_i];
        if (idx >= 0 && idx < urtc::kNumRobotJoints) {
          const auto uidx = static_cast<std::size_t>(idx);
          current_positions_[uidx] = msg->position[msg_i];
          if (msg_i < msg->velocity.size()) {
            current_velocities_[uidx] = msg->velocity[msg_i];
          }
          if (msg_i < msg->effort.size()) {
            current_torques_[uidx] = msg->effort[msg_i];
          }
        }
      }
    } else {
      // Positional fallback (기존 동작)
      std::copy_n(msg->position.begin(), urtc::kNumRobotJoints,
                  current_positions_.begin());
      std::copy_n(msg->velocity.begin(), urtc::kNumRobotJoints,
                  current_velocities_.begin());
      if (msg->effort.size() >= static_cast<std::size_t>(urtc::kNumRobotJoints)) {
        std::copy_n(msg->effort.begin(), urtc::kNumRobotJoints,
                    current_torques_.begin());
      }
    }
    last_robot_update_ = std::chrono::steady_clock::now();
  }
  state_received_.store(true, std::memory_order_release);
}

void RtControllerNode::RobotTargetCallback(
    std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < urtc::kNumRobotJoints) {
    return;
  }
  std::array<double, urtc::kNumRobotJoints> local_target;
  {
    std::lock_guard lock(target_mutex_);
    std::copy_n(msg->data.begin(), urtc::kNumRobotJoints,
                target_positions_.begin());
    local_target = target_positions_;
  }
  target_received_.store(true, std::memory_order_release);
  int active_idx = active_controller_idx_.load(std::memory_order_acquire);
  controllers_[active_idx]->SetRobotTarget(local_target);
}

void RtControllerNode::HandTargetCallback(
    std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < urtc::kNumHandMotors) {
    return;
  }
  std::array<float, urtc::kNumHandMotors> hand_target;
  for (std::size_t i = 0; i < urtc::kNumHandMotors; ++i) {
    hand_target[i] = static_cast<float>(msg->data[i]);
  }
  int active_idx = active_controller_idx_.load(std::memory_order_acquire);
  controllers_[active_idx]->SetHandTarget(hand_target);
}

// ── 50 Hz watchdog (E-STOP) ───────────────────────────────────────────────────
void RtControllerNode::CheckTimeouts()
{
  // ur5e 비활성 시 robot timeout 체크 skip
  if (!global_device_flags_.enable_ur5e) { return; }

  const auto now_time = std::chrono::steady_clock::now();
  bool robot_timed_out = false;

  // Non-blocking: try_lock avoids stalling the RT loop if sensor thread
  // holds the mutex. On contention, skip this check (retried in 20 ms).
  if (state_received_.load(std::memory_order_acquire)) {
    std::unique_lock lock(state_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      robot_timed_out = (now_time - last_robot_update_) > robot_timeout_;
    }
  }

  if (robot_timed_out && !IsGlobalEstopped()) {
    TriggerGlobalEstop("robot_timeout");
  }
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
      // state는 수신됨 — 현재 위치를 읽어 target으로 초기화
      urtc::ControllerState hold_state{};
      {
        std::lock_guard lock(state_mutex_);
        cached_positions_ = current_positions_;
        cached_velocities_ = current_velocities_;
        cached_torques_ = current_torques_;
      }
      hold_state.robot.positions = cached_positions_;
      hold_state.robot.velocities = cached_velocities_;
      hold_state.robot.torques = cached_torques_;
      hold_state.robot.dt = 1.0 / control_rate_;
      hold_state.dt = hold_state.robot.dt;

      // Hand state — ROS 토픽에서 읽기
      {
        std::lock_guard lock(hand_state_mutex_);
        hold_state.hand = cached_hand_state_;
      }

      int idx = active_controller_idx_.load(std::memory_order_acquire);
      controllers_[idx]->InitializeHoldPosition(hold_state);

      // node-level target도 동기화
      {
        std::lock_guard lock(target_mutex_);
        target_positions_ = cached_positions_;
      }
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
  urtc::ControllerState state{};
  {
    std::unique_lock lock(state_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      cached_positions_ = current_positions_;
      cached_velocities_ = current_velocities_;
      cached_torques_ = current_torques_;
    }
  }
  state.robot.positions = cached_positions_;
  state.robot.velocities = cached_velocities_;
  state.robot.torques = cached_torques_;

  // Hand state — ROS 토픽에서 읽기 (try_lock으로 RT 안전)
  {
    std::unique_lock lock(hand_state_mutex_, std::try_to_lock);
    // lock 실패 시 이전 사이클 데이터 재사용 (≤2ms 지연, 제어에 무해)
  }
  state.hand = cached_hand_state_;

  {
    std::unique_lock lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      target_snapshot_ = target_positions_;
    }
  }
  state.robot.dt = 1.0 / control_rate_;
  state.dt = state.robot.dt;  // Keep top-level dt in sync with robot.dt
  state.robot.iteration = loop_count_;
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
    const auto & active_df = controller_device_flags_[
        static_cast<std::size_t>(active_idx)];
    urtc::PublishSnapshot snap{};
    snap.command_type          = output.command_type;
    snap.actual_task_positions = output.actual_task_positions;
    snap.stamp_ns              = std::chrono::steady_clock::now()
                                   .time_since_epoch().count();
    snap.active_controller_idx = active_idx;
    snap.ur5e_enabled          = active_df.enable_ur5e;
    snap.device_enabled        = active_df.enable_hand;
    // Copy robot arrays (size 6 → size kMaxRobotDOF)
    for (std::size_t i = 0; i < urtc::kNumRobotJoints; ++i) {
      snap.robot_commands[i]        = output.robot_commands[i];
      snap.goal_positions[i]        = output.goal_positions[i];
      snap.actual_target_positions[i] = output.actual_target_positions[i];
      snap.target_velocities[i]     = output.target_velocities[i];
      snap.actual_positions[i]      = state.robot.positions[i];
      snap.actual_velocities[i]     = state.robot.velocities[i];
    }
    // Copy device commands (size kNumHandMotors → kMaxDeviceChannels)
    for (std::size_t i = 0; i < urtc::kNumHandMotors; ++i) {
      snap.device_commands[i] = output.hand_commands[i];
    }
    static_cast<void>(publish_buffer_.Push(snap));
  }

  // Hand commands are now published via JointCommand topics in PublishLoopEntry()
  // — no direct HandController ownership in rt_controller_node.

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
    entry.device_valid       = state.hand.valid;
    entry.num_device_channels = urtc::kNumHandMotors;
    // Robot arrays (size 6 → kMaxRobotDOF)
    for (std::size_t i = 0; i < urtc::kNumRobotJoints; ++i) {
      entry.goal_positions[i]        = output.goal_positions[i];
      entry.actual_positions[i]      = state.robot.positions[i];
      entry.actual_velocities[i]     = state.robot.velocities[i];
      entry.actual_torques[i]        = state.robot.torques[i];
      entry.robot_commands[i]        = output.robot_commands[i];
      entry.trajectory_positions[i]  = output.actual_target_positions[i];
      entry.trajectory_velocities[i] = output.target_velocities[i];
    }
    // Device goal/actual/velocity/command 복사 (크기가 다른 배열)
    for (std::size_t i = 0; i < urtc::kNumHandMotors; ++i) {
      entry.device_goal[i] = output.hand_goal_positions[i];
      entry.device_actual[i] = state.hand.motor_positions[i];
      entry.device_velocities[i] = state.hand.motor_velocities[i];
      entry.device_commands[i] = output.hand_commands[i];
    }
    // Sensor data 복사 (int32_t → float 변환)
    for (std::size_t i = 0; i < urtc::kMaxHandSensors && i < entry.sensor_data.size(); ++i) {
      entry.sensor_data[i] = static_cast<float>(state.hand.sensor_data[i]);
      entry.sensor_data_raw[i] = static_cast<float>(state.hand.sensor_data_raw[i]);
    }
    entry.num_sensor_channels = urtc::kMaxHandSensors;
    entry.num_fingertips = state.hand.num_fingertips;
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
    auto publish_entry = [&](const urtc::PublishTopicEntry & pt, bool is_hand) {
      switch (pt.role) {
        case urtc::PublishRole::kJointCommand: {
          auto jc_it = joint_command_publishers_.find(pt.topic_name);
          if (jc_it == joint_command_publishers_.end()) { return; }
          auto & jce = jc_it->second;
          jce.msg.header.stamp.sec = sec;
          jce.msg.header.stamp.nanosec = nsec;
          jce.msg.command_type = cmd_type_str;
          if (!is_hand) {
            std::copy_n(snap.robot_commands.begin(), snap.num_robot_joints,
                        jce.msg.values.begin());
          } else {
            for (std::size_t i = 0; i < urtc::kNumHandMotors
                     && i < snap.device_commands.size(); ++i) {
              jce.msg.values[i] = static_cast<double>(snap.device_commands[i]);
            }
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
        case urtc::PublishRole::kRos2Command:
          if (!is_hand) {
            std::copy_n(snap.robot_commands.begin(), snap.num_robot_joints,
                        pe.msg.data.begin());
          } else {
            for (std::size_t i = 0; i < urtc::kNumHandMotors
                     && i < snap.device_commands.size(); ++i) {
              pe.msg.data[i] = static_cast<double>(snap.device_commands[i]);
            }
          }
          pe.publisher->publish(pe.msg);
          break;
        case urtc::PublishRole::kTaskPosition:
          std::copy(snap.actual_task_positions.begin(),
                    snap.actual_task_positions.end(),
                    pe.msg.data.begin());
          pe.publisher->publish(pe.msg);
          break;
        case urtc::PublishRole::kTrajectoryState:
          std::copy_n(snap.goal_positions.begin(), 6, pe.msg.data.begin());
          std::copy_n(snap.actual_target_positions.begin(), 6, pe.msg.data.begin() + 6);
          std::copy_n(snap.target_velocities.begin(), 6, pe.msg.data.begin() + 12);
          pe.publisher->publish(pe.msg);
          break;
        case urtc::PublishRole::kControllerState:
          std::copy_n(snap.actual_positions.begin(), 6, pe.msg.data.begin());
          std::copy_n(snap.actual_velocities.begin(), 6, pe.msg.data.begin() + 6);
          std::copy_n(snap.robot_commands.begin(), 6, pe.msg.data.begin() + 12);
          pe.publisher->publish(pe.msg);
          break;
        default:
          break;
      }
    };

    // Publish ur5e topics
    if (snap.ur5e_enabled) {
      for (const auto & pt : active_tc.ur5e.publish) {
        publish_entry(pt, false);
      }
    }

    // Publish hand topics
    if (snap.device_enabled) {
      for (const auto & pt : active_tc.hand.publish) {
        publish_entry(pt, true);
      }
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

// ── Device flag resolution ─────────────────────────────────────────────────────
urtc::DeviceEnableFlags RtControllerNode::ResolveDeviceFlags(
    const urtc::PerControllerDeviceFlags & per_ctrl) const noexcept
{
  urtc::DeviceEnableFlags resolved = global_device_flags_;
  if (per_ctrl.enable_ur5e.has_value()) {
    resolved.enable_ur5e = per_ctrl.enable_ur5e.value();
  }
  if (per_ctrl.enable_hand.has_value()) {
    resolved.enable_hand = per_ctrl.enable_hand.value();
  }
  return resolved;
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
  // 1. YAML에서 이름 로드
  robot_joint_names_ = get_parameter("robot_joint_names").as_string_array();
  hand_motor_names_  = get_parameter("hand_motor_names").as_string_array();
  fingertip_names_   = get_parameter("hand_fingertip_names").as_string_array();

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

  // 2. 개수 검증
  if (robot_joint_names_.size() != static_cast<std::size_t>(urtc::kNumRobotJoints)) {
    RCLCPP_ERROR(get_logger(),
                 "robot_joint_names has %zu entries (expected %d) — using defaults",
                 robot_joint_names_.size(), urtc::kNumRobotJoints);
    robot_joint_names_ = urtc::kDefaultRobotJointNames;
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
  // Hand motor 개수 검증
  if (hand_motor_names_.size() != static_cast<std::size_t>(urtc::kNumHandMotors)) {
    RCLCPP_ERROR(get_logger(),
                 "hand_motor_names has %zu entries (expected %d) — using defaults",
                 hand_motor_names_.size(), urtc::kNumHandMotors);
    hand_motor_names_ = urtc::kDefaultHandMotorNames;
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

void RtControllerNode::BuildJointStateIndexMap(
    const std::vector<std::string>& msg_names)
{
  joint_state_reorder_.resize(msg_names.size(), -1);

  for (std::size_t msg_i = 0; msg_i < msg_names.size(); ++msg_i) {
    for (std::size_t our_i = 0; our_i < robot_joint_names_.size(); ++our_i) {
      if (msg_names[msg_i] == robot_joint_names_[our_i]) {
        joint_state_reorder_[msg_i] = static_cast<int>(our_i);
        break;
      }
    }
    if (joint_state_reorder_[msg_i] < 0) {
      RCLCPP_DEBUG(get_logger(),
                   "JointState name '%s' not in robot_joint_names — ignored",
                   msg_names[msg_i].c_str());
    }
  }
  joint_state_map_built_ = true;

  RCLCPP_INFO(get_logger(), "Built JointState name→index map from incoming message");
}

void RtControllerNode::BuildHandStateIndexMap(
    const std::vector<std::string>& source_names)
{
  hand_state_reorder_.resize(source_names.size(), -1);

  for (std::size_t src_i = 0; src_i < source_names.size(); ++src_i) {
    for (std::size_t our_i = 0; our_i < hand_motor_names_.size(); ++our_i) {
      if (source_names[src_i] == hand_motor_names_[our_i]) {
        hand_state_reorder_[src_i] = static_cast<int>(our_i);
        break;
      }
    }
    if (hand_state_reorder_[src_i] < 0) {
      RCLCPP_WARN(get_logger(),
                  "Hand motor name '%s' from source not in hand_motor_names — ignored",
                  source_names[src_i].c_str());
    }
  }
  hand_state_map_built_ = true;

  RCLCPP_INFO(get_logger(), "Built hand state name→index map (%zu entries)",
              source_names.size());
}
