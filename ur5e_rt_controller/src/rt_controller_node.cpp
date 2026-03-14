// ── Includes: project header first, then ROS2, then C++ stdlib ──────────────
#include "ur5e_rt_controller/rt_controller_node.hpp"

#include "ur5e_rt_controller/controllers/direct/joint_pd_controller.hpp"
#include "ur5e_rt_controller/controllers/direct/operational_space_controller.hpp"
#include "ur5e_rt_controller/controllers/indirect/p_controller.hpp"
#include "ur5e_rt_controller/controllers/indirect/clik_controller.hpp"
#include "ur5e_rt_controller/controllers/indirect/ur5e_hand_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ur5e_rt_base/logging/session_dir.hpp>
#include <ur5e_rt_base/threading/thread_utils.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <ctime>
#include <functional>
#include <set>
#include <string_view>
#include <unordered_map>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// ── Controller registry ──────────────────────────────────────────────────────
//
// To add a new controller:
//   1. Create include/ur5e_rt_controller/controllers/my_controller.hpp
//      (inherit RTControllerInterface, implement all pure virtuals + LoadConfig
//       + UpdateGainsFromMsg)
//   2. Create config/controllers/my_controller.yaml
//   3. Add one #include line above and one entry to MakeControllerEntries() below.
//
// No other changes to this file are required.
// ─────────────────────────────────────────────────────────────────────────────
namespace
{
struct ControllerEntry
{
  // config_key: YAML filename stem (e.g. "pd_controller" →
  //   config/controllers/<config_subdir>/pd_controller.yaml)
  // config_subdir: "direct/" for torque controllers, "indirect/" for position controllers
  std::string_view config_key;
  std::string_view config_subdir;
  std::function<std::unique_ptr<urtc::RTControllerInterface>(const std::string &)> factory;
};

std::vector<ControllerEntry> MakeControllerEntries()
{
  return {
    {
      "p_controller", "indirect/",
      [](const std::string & p) {return std::make_unique<urtc::PController>(p);}
    },
    {
      "joint_pd_controller", "direct/",
      [](const std::string & p) {return std::make_unique<urtc::JointPDController>(p);}
    },
    {
      "clik_controller", "indirect/",
      [](const std::string & p) {
        return std::make_unique<urtc::ClikController>(p, urtc::ClikController::Gains{});
      }
    },
    {
      "operational_space_controller", "direct/",
      [](const std::string & p) {
        return std::make_unique<urtc::OperationalSpaceController>(
          p, urtc::OperationalSpaceController::Gains{});
      }
    },
    {
      "ur5e_hand_controller", "indirect/",
      [](const std::string & p) {
        return std::make_unique<urtc::UrFiveEHandController>(p);
      }
    },
    // ── Add new controllers here ─────────────────────────────────────────────
  };
}
}  // namespace

// ── Constructor / destructor ──────────────────────────────────────────────────
RtControllerNode::RtControllerNode()
: Node("rt_controller"), logger_(nullptr)
{
  CreateCallbackGroups();
  DeclareAndLoadParameters();
  CreateSubscriptions();
  CreatePublishers();
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
    status_monitor_ = std::make_unique<ur5e_status_monitor::UR5eStatusMonitor>(
        shared_from_this());

    status_monitor_->registerOnFailure(
        [this](ur5e_status_monitor::FailureType type,
               const ur5e_status_monitor::FailureContext & ctx) {
          (void)type;
          TriggerGlobalEstop(ctx.description);
        });

    status_monitor_->registerOnReady(
        [this]() {
          RCLCPP_INFO(get_logger(), "StatusMonitor: system ready");
        });

    const auto cfgs = ur5e_rt_controller::SelectThreadConfigs();
    status_monitor_->start(cfgs.status_monitor);
    RCLCPP_INFO(get_logger(), "UR5eStatusMonitor started (10 Hz, Core %d)",
                cfgs.status_monitor.cpu_core);
  }

  RCLCPP_INFO(get_logger(), "RtControllerNode ready — %.0f Hz, E-STOP: %s",
              control_rate_, enable_estop_ ? "ON" : "OFF");
  RCLCPP_INFO(get_logger(), "CallbackGroups enabled: RT, Sensor, Log, Aux");
}

RtControllerNode::~RtControllerNode()
{
  if (hand_controller_) {
    hand_controller_->Stop();
  }
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
      ament_index_cpp::get_package_share_directory("ur5e_rt_controller");
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
  cb_group_rt_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
  declare_parameter("initial_controller", "joint_pd_controller");

  control_rate_ = get_parameter("control_rate").as_double();
  budget_us_ = 1.0e6 / control_rate_;  // tick budget in µs (e.g., 2000.0 at 500 Hz)

  const double init_timeout_sec = get_parameter("init_timeout_sec").as_double();
  init_timeout_ticks_ = static_cast<uint64_t>(init_timeout_sec * control_rate_);
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
    const bool enable_robot  = get_parameter("enable_robot_log").as_bool();
    const bool enable_hand   = get_parameter("enable_hand_log").as_bool();
    const auto ctrl_dir = session_dir / "controller";
    std::filesystem::create_directories(ctrl_dir);

    const std::string timing_path = enable_timing
        ? (ctrl_dir / "timing_log.csv").string() : "";
    const std::string robot_path = enable_robot
        ? (ctrl_dir / "robot_log.csv").string() : "";
    const std::string hand_path = enable_hand
        ? (ctrl_dir / "hand_log.csv").string() : "";

    logger_ = std::make_unique<urtc::DataLogger>(timing_path, robot_path, hand_path);
    RCLCPP_INFO(get_logger(),
        "Logging to: %s/controller/ (max_sessions=%d)",
        session_dir.string().c_str(), max_sessions);
  }

  robot_timeout_ = std::chrono::milliseconds(
      static_cast<int>(get_parameter("robot_timeout_ms").as_double()));

  // ── Hand Controller (직접 UDP 통신, event-driven) ──────────────────────────
  // hand_udp_node.yaml의 파라미터를 launch에서 로드
  declare_parameter("target_ip", std::string(""));
  declare_parameter("target_port", 0);
  declare_parameter("recv_timeout_ms", 10);
  declare_parameter("enable_write_ack", false);
  declare_parameter("sensor_decimation", 1);

  const std::string hand_ip = get_parameter("target_ip").as_string();
  const int hand_port = static_cast<int>(get_parameter("target_port").as_int());
  enable_hand_ = !hand_ip.empty() && hand_port > 0;

  if (enable_hand_) {
    const int hand_recv_timeout = static_cast<int>(
        get_parameter("recv_timeout_ms").as_int());
    const bool hand_write_ack = get_parameter("enable_write_ack").as_bool();
    const int sensor_decimation = static_cast<int>(
        get_parameter("sensor_decimation").as_int());
    const auto cfgs = ur5e_rt_controller::SelectThreadConfigs();

    hand_controller_ = std::make_unique<urtc::HandController>(
        hand_ip, hand_port, cfgs.udp_recv,
        hand_recv_timeout, hand_write_ack, sensor_decimation);
    hand_controller_->SetEstopFlag(&global_estop_);

    if (hand_controller_->Start()) {
      RCLCPP_INFO(get_logger(), "HandController started: %s:%d (event-driven)",
                  hand_ip.c_str(), hand_port);
    } else {
      RCLCPP_ERROR(get_logger(), "HandController failed to start: %s:%d",
                   hand_ip.c_str(), hand_port);
      hand_controller_.reset();
      enable_hand_ = false;
    }
  }

  std::string urdf_path = "";
  try {
    std::string share_dir =
      ament_index_cpp::get_package_share_directory("ur5e_description");
    urdf_path = share_dir + "/robots/ur5e/urdf/ur5e.urdf";
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Could not resolve urdf path: %s", e.what());
  }

  std::string config_dir;
  try {
    config_dir = ament_index_cpp::get_package_share_directory("ur5e_rt_controller") +
      "/config/controllers/";
  } catch (...) {
  }

  // ── Instantiate and configure all registered controllers ─────────────────
  // Each factory constructs the controller with default gains, then
  // LoadConfig() reads per-controller overrides from its YAML file.
  std::unordered_map<std::string, int> name_to_idx;
  const auto entries = MakeControllerEntries();

  for (std::size_t i = 0; i < entries.size(); ++i) {
    const auto & entry = entries[i];
    auto ctrl = entry.factory(urdf_path);

    if (!config_dir.empty()) {
      try {
        const std::string yaml_path = config_dir + std::string(entry.config_subdir) +
        std::string(entry.config_key) + ".yaml";
        const YAML::Node file_node = YAML::LoadFile(yaml_path);
        ctrl->LoadConfig(file_node[std::string(entry.config_key)]);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(),
          "Config load failed for '%s' (%s) — using defaults",
          ctrl->Name().data(), e.what());
      }
    }

    // Register both the class name (e.g. "PDController") and the config-key
    // alias (e.g. "pd_controller") so either form works as initial_controller.
    name_to_idx[std::string(ctrl->Name())] = static_cast<int>(i);
    name_to_idx[std::string(entry.config_key)] = static_cast<int>(i);

    controllers_.push_back(std::move(ctrl));
  }

  // Cache per-controller topic configs for fast access in ControlLoop()
  controller_topic_configs_.reserve(controllers_.size());
  for (const auto & ctrl : controllers_) {
    controller_topic_configs_.push_back(ctrl->GetTopicConfig());
    const auto & tc = controller_topic_configs_.back();
    RCLCPP_INFO(get_logger(), "Controller '%s': %zu subscribe topics, %zu publish topics",
                ctrl->Name().data(), tc.subscribe.size(), tc.publish.size());
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

  // ── Collect unique subscribe topics from all controllers ──────────────────
  // Each topic is created once; the callback is determined by the role.
  std::set<std::string> created_joint_state_topics;
  std::set<std::string> created_target_topics;

  for (const auto & tc : controller_topic_configs_) {
    for (const auto & entry : tc.subscribe) {
      switch (entry.role) {
        case urtc::SubscribeRole::kJointState:
          if (created_joint_state_topics.insert(entry.topic_name).second) {
            auto sub = create_subscription<sensor_msgs::msg::JointState>(
              entry.topic_name, 10,
              [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                JointStateCallback(std::move(msg));
              },
              sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [joint_state]: %s",
                        entry.topic_name.c_str());
          }
          break;

        case urtc::SubscribeRole::kHandState:
          // Hand state는 HandController에서 직접 읽으므로 ROS 구독 생략
          RCLCPP_INFO(get_logger(), "  Subscribe [hand_state]: %s (skipped — direct UDP)",
                      entry.topic_name.c_str());
          break;

        case urtc::SubscribeRole::kTarget:
          if (created_target_topics.insert(entry.topic_name).second) {
            auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
              entry.topic_name, 10,
              [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                TargetCallback(std::move(msg));
              },
              sub_options);
            topic_subscriptions_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  Subscribe [target]: %s",
                        entry.topic_name.c_str());
          }
          break;
      }
    }
  }

  // ── Fixed control subscriptions (always present) ──────────────────────────
  controller_selector_sub_ = create_subscription<std_msgs::msg::Int32>(
      "~/controller_type", 10,
    [this](std_msgs::msg::Int32::SharedPtr msg) {
      int idx = msg->data;
      if (idx >= 0 && idx < static_cast<int>(controllers_.size())) {
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
      "~/controller_gains", 10,
    [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      const int idx = active_controller_idx_.load(std::memory_order_acquire);
      controllers_[static_cast<std::size_t>(idx)]->UpdateGainsFromMsg(msg->data);
      RCLCPP_INFO(get_logger(), "Gains updated for %s",
        controllers_[static_cast<std::size_t>(idx)]->Name().data());
      },
      sub_options);

  request_gains_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/request_gains", 10,
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

  // ── Collect unique publish topics from all controllers ────────────────────
  for (const auto & tc : controller_topic_configs_) {
    for (const auto & entry : tc.publish) {
      if (topic_publishers_.count(entry.topic_name) > 0) {
        continue;  // already created
      }

      // Determine message size from role defaults or explicit config
      int data_size = entry.data_size;
      if (data_size <= 0) {
        switch (entry.role) {
          case urtc::PublishRole::kPositionCommand:
          case urtc::PublishRole::kTorqueCommand:
            data_size = urtc::kNumRobotJoints;
            break;
          case urtc::PublishRole::kHandCommand:
            data_size = urtc::kNumHandMotors;
            break;
          case urtc::PublishRole::kTaskPosition:
            data_size = 6;
            break;
        }
      }

      PublisherEntry pe;
      pe.publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
          entry.topic_name, cmd_qos);
      pe.msg.data.resize(static_cast<std::size_t>(data_size), 0.0);
      topic_publishers_[entry.topic_name] = std::move(pe);

      RCLCPP_INFO(get_logger(), "  Publish [%s]: %s (size=%d)",
                  entry.role == urtc::PublishRole::kPositionCommand ? "position_cmd" :
                  entry.role == urtc::PublishRole::kTorqueCommand   ? "torque_cmd" :
                  entry.role == urtc::PublishRole::kHandCommand     ? "hand_cmd" :
                                                                      "task_pos",
                  entry.topic_name.c_str(), data_size);
    }
  }

  // ── Fixed publishers (always present) ─────────────────────────────────────
  estop_pub_ =
    create_publisher<std_msgs::msg::Bool>("/system/estop_status", 10);

  rclcpp::QoS latch_qos{1};
  latch_qos.transient_local();
  active_ctrl_name_pub_ =
    create_publisher<std_msgs::msg::String>("~/active_controller_name", latch_qos);

  current_gains_pub_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("~/current_gains", 10);
}

void RtControllerNode::CreateTimers()
{
  const auto control_period = std::chrono::microseconds(
      static_cast<int>(1'000'000.0 / control_rate_));

  // Assign control_timer_ and timeout_timer_ to cb_group_rt_
  control_timer_ = create_wall_timer(
      control_period, [this]() {ControlLoop();}, cb_group_rt_);

  if (enable_estop_) {
    timeout_timer_ =
      create_wall_timer(20ms, [this]() {CheckTimeouts();}, cb_group_rt_);
  }

  // Fix 1: drain the SPSC log ring buffer from the log thread (Core 4).
  // File I/O stays entirely out of the 500 Hz RT thread.
  drain_timer_ =
    create_wall_timer(10ms, [this]() {DrainLog();}, cb_group_log_);
}

// ── Subscription callbacks ────────────────────────────────────────────────────
void RtControllerNode::JointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < urtc::kNumRobotJoints) {
    return;
  }
  {
    std::lock_guard lock(state_mutex_);
    std::copy_n(msg->position.begin(), urtc::kNumRobotJoints,
                current_positions_.begin());
    std::copy_n(msg->velocity.begin(), urtc::kNumRobotJoints,
                current_velocities_.begin());
    if (msg->effort.size() >= static_cast<std::size_t>(urtc::kNumRobotJoints)) {
      std::copy_n(msg->effort.begin(), urtc::kNumRobotJoints,
                  current_torques_.begin());
    }
    last_robot_update_ = now();
  }
  // Fix 2: release-store after the mutex — RT thread reads with acquire,
  // so the C++ memory model guarantees it sees the written positions/velocities.
  state_received_.store(true, std::memory_order_release);
}

void RtControllerNode::TargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < urtc::kNumRobotJoints) {
    return;
  }
  // Fix 3: build a local copy while the mutex is held, then call
  // SetRobotTarget() with the copy — avoids a data race where the RT thread
  // could overwrite target_positions_ between mutex release and the call.
  std::array<double, urtc::kNumRobotJoints> local_target;
  {
    std::lock_guard lock(target_mutex_);
    std::copy_n(msg->data.begin(), urtc::kNumRobotJoints,
                target_positions_.begin());
    local_target = target_positions_;
  }
  // Fix 2: release-store after mutex released
  target_received_.store(true, std::memory_order_release);
  int active_idx = active_controller_idx_.load(std::memory_order_acquire);
  controllers_[active_idx]->SetRobotTarget(local_target);

  // If the message contains additional values for hand motors
  // (data[6..15]), forward them to SetHandTarget().
  constexpr std::size_t kHandOffset = static_cast<std::size_t>(urtc::kNumRobotJoints);
  constexpr std::size_t kHandSize   = static_cast<std::size_t>(urtc::kNumHandMotors);
  if (msg->data.size() >= kHandOffset + kHandSize) {
    std::array<float, urtc::kNumHandMotors> hand_target;
    for (std::size_t i = 0; i < kHandSize; ++i) {
      hand_target[i] = static_cast<float>(msg->data[kHandOffset + i]);
    }
    controllers_[active_idx]->SetHandTarget(hand_target);
  }
}

// ── 50 Hz watchdog (E-STOP) ───────────────────────────────────────────────────
void RtControllerNode::CheckTimeouts()
{
  const auto now_time = now();
  bool robot_timed_out = false;

  // Fix 2: acquire-load the atomic flag; if set, lock mutex to read timestamp
  if (state_received_.load(std::memory_order_acquire)) {
    std::lock_guard lock(state_mutex_);
    robot_timed_out = (now_time - last_robot_update_) > robot_timeout_;
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

  if (!state_received_.load(std::memory_order_acquire) ||
    !target_received_.load(std::memory_order_acquire))
  {
    // Initialization timeout — check if we've waited too long for data.
    if (!init_complete_ && ++init_wait_ticks_ > init_timeout_ticks_) {
      RCLCPP_FATAL(get_logger(),
          "Initialization timeout (%.1f s): robot=%d, target=%d, hand=%s",
          static_cast<double>(init_timeout_ticks_) / control_rate_,
          state_received_.load(std::memory_order_relaxed) ? 1 : 0,
          target_received_.load(std::memory_order_relaxed) ? 1 : 0,
          enable_hand_ ? (hand_controller_ && hand_controller_->IsRunning() ? "running" : "stopped") : "disabled");
      TriggerGlobalEstop("init_timeout");
      rclcpp::shutdown();
    }
    return;
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

  // Hand state — HandController에서 직접 읽기 (이전 tick에서 준비된 state)
  if (hand_controller_) {
    cached_hand_state_ = hand_controller_->GetLatestState();
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

  // ── Phase 3: non-blocking publish ──────────────────────────────────────
  if (cmd_pub_mutex_.try_lock()) {
    // Publish to all topics configured for the active controller
    const auto & pub_topics = controller_topic_configs_[
        static_cast<std::size_t>(active_idx)].publish;

    for (const auto & pt : pub_topics) {
      auto it = topic_publishers_.find(pt.topic_name);
      if (it == topic_publishers_.end()) { continue; }

      auto & pe = it->second;
      switch (pt.role) {
        case urtc::PublishRole::kPositionCommand:
          if (output.command_type == urtc::CommandType::kPosition) {
            std::copy(output.robot_commands.begin(), output.robot_commands.end(),
                      pe.msg.data.begin());
            pe.publisher->publish(pe.msg);
          }
          break;
        case urtc::PublishRole::kTorqueCommand:
          if (output.command_type == urtc::CommandType::kTorque) {
            std::copy(output.robot_commands.begin(), output.robot_commands.end(),
                      pe.msg.data.begin());
            pe.publisher->publish(pe.msg);
          }
          break;
        case urtc::PublishRole::kHandCommand:
          // Hand command는 HandController로 직접 전송 (아래 Phase 3.5에서 처리)
          break;
        case urtc::PublishRole::kTaskPosition:
          std::copy(output.actual_task_positions.begin(),
                    output.actual_task_positions.end(),
                    pe.msg.data.begin());
          pe.publisher->publish(pe.msg);
          break;
      }
    }

    cmd_pub_mutex_.unlock();
  }

  // ── Phase 3.5: hand command 전송 + 다음 tick용 state 읽기 요청 ────────
  // Event-driven pipeline: hand thread가 write → read 수행, 다음 tick에서 사용
  if (hand_controller_) {
    hand_controller_->SendCommandAndRequestStates(output.hand_commands);
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

  // Overrun detection — RT-safe: no string alloc, just atomic increment.
  if (t_total_us > budget_us_) {
    overrun_count_.fetch_add(1, std::memory_order_relaxed);
  }

  // Push log entry to the SPSC ring buffer — O(1), no syscall.
  // DrainLog() (log thread, Core 4) pops entries and writes the CSV file.
  if (enable_logging_) {
    if (loop_count_ == 0) {
      log_start_time_ = t0;
    }
    const double timestamp =
        std::chrono::duration<double>(t0 - log_start_time_).count();

    const urtc::LogEntry entry{
      .timestamp = timestamp,
      // Timing
      .t_state_acquire_us = t_state_us,
      .t_compute_us = t_compute_us,
      .t_publish_us = t_publish_us,
      .t_total_us = t_total_us,
      .jitter_us = jitter_us,
      // Robot
      .goal_positions = output.goal_positions,
      .target_positions = output.actual_target_positions,
      .target_velocities = output.target_velocities,
      .actual_positions = state.robot.positions,
      .actual_velocities = state.robot.velocities,
      // Hand
      .hand_valid = state.hand.valid,
      .hand_goal_positions = output.hand_goal_positions,
      .hand_commands = output.hand_commands,
      .hand_actual_positions = state.hand.motor_positions,
      .hand_actual_velocities = state.hand.motor_velocities,
      .hand_sensors = state.hand.sensor_data,
    };
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
    const auto drops = log_buffer_.drop_count();
    RCLCPP_INFO(get_logger(), "%s  overruns=%lu  log_drops=%lu",
                timing_profiler_
      .Summary(std::string(controllers_[static_cast<std::size_t>(idx)]->Name()))
      .c_str(),
      static_cast<unsigned long>(overruns),
      static_cast<unsigned long>(drops));
  }
}

void RtControllerNode::PublishEstopStatus(bool estopped)
{
  std_msgs::msg::Bool msg;
  msg.data = estopped;
  estop_pub_->publish(msg);
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
