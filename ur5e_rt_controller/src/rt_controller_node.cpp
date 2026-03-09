// ── Includes: project header first, then ROS2, then C++ stdlib ──────────────
#include "ur5e_rt_controller/rt_controller_node.hpp"

#include "ur5e_rt_controller/controllers/clik_controller.hpp"
#include "ur5e_rt_controller/controllers/operational_space_controller.hpp"
#include "ur5e_rt_controller/controllers/p_controller.hpp"
#include "ur5e_rt_controller/controllers/pd_controller.hpp"
#include "ur5e_rt_controller/controllers/pinocchio_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <ctime>
#include <functional>
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
  // Key used both as the YAML filename stem (e.g. "pd_controller" →
  // config/controllers/pd_controller.yaml) and as the alias accepted by
  // the initial_controller ROS parameter.
  std::string_view config_key;
  std::function<std::unique_ptr<urtc::RTControllerInterface>(const std::string &)> factory;
};

std::vector<ControllerEntry> MakeControllerEntries()
{
  return {
    {
      "p_controller",
      [](const std::string &) { return std::make_unique<urtc::PController>(); }
    },
    {
      "pd_controller",
      [](const std::string &) { return std::make_unique<urtc::PDController>(); }
    },
    {
      "pinocchio_controller",
      [](const std::string & p) {
        return std::make_unique<urtc::PinocchioController>(p, urtc::PinocchioController::Gains{});
      }
    },
    {
      "clik_controller",
      [](const std::string & p) {
        return std::make_unique<urtc::ClikController>(p, urtc::ClikController::Gains{});
      }
    },
    {
      "operational_space_controller",
      [](const std::string & p) {
        return std::make_unique<urtc::OperationalSpaceController>(
          p, urtc::OperationalSpaceController::Gains{});
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

  RCLCPP_INFO(get_logger(), "RtControllerNode ready — %.0f Hz, E-STOP: %s",
              control_rate_, enable_estop_ ? "ON" : "OFF");
  RCLCPP_INFO(get_logger(), "CallbackGroups enabled: RT, Sensor, Log, Aux");
}

RtControllerNode::~RtControllerNode()
{
  if (logger_) {
    logger_->Flush();
  }
}

// ── Log file helpers ──────────────────────────────────────────────────────────
std::string RtControllerNode::GenerateLogFilePath(const std::string & log_dir)
{
  const auto now = std::chrono::system_clock::now();
  const auto time_t = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&time_t, &local_tm);
  char timestamp[16];
  std::strftime(timestamp, sizeof(timestamp), "%y%m%d_%H%M", &local_tm);
  return log_dir + "/ur5e_control_log_" + timestamp + ".csv";
}

// Removes oldest matching log files when count exceeds max_files.
// Files are matched by prefix "ur5e_control_log_" and ".csv" extension;
// alphabetical sort equals chronological order due to the timestamp format.
void RtControllerNode::CleanupOldLogFiles(
  const std::filesystem::path & log_dir,
  int max_files)
{
  if (!std::filesystem::exists(log_dir)) {
    return;
  }
  std::vector<std::filesystem::path> files;
  for (const auto & entry : std::filesystem::directory_iterator(log_dir)) {
    const auto & p = entry.path();
    if (p.extension() == ".csv") {
      const std::string stem = p.stem().string();
      if (stem.rfind("ur5e_control_log_", 0) == 0) {
        files.push_back(p);
      }
    }
  }
  std::sort(files.begin(), files.end());
  while (static_cast<int>(files.size()) > max_files) {
    std::filesystem::remove(files.front());
    files.erase(files.begin());
  }
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

  // Dynamically resolve workspace logging dir using ament_index
  std::string default_log_dir = "/tmp/ur5e_logging_data"; // fallback
  try {
    std::string share_dir =
      ament_index_cpp::get_package_share_directory("ur5e_rt_controller");
    std::filesystem::path ws_path = std::filesystem::path(share_dir)
      .parent_path()
      .parent_path()
      .parent_path()
      .parent_path();
    default_log_dir = (ws_path / "logging_data").string();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
        get_logger(),
        "Could not resolve workspace via ament_index: %s. Using fallback: %s",
        e.what(), default_log_dir.c_str());
  }

  declare_parameter("log_dir", default_log_dir);
  declare_parameter("max_log_files", 10);
  declare_parameter("robot_timeout_ms", 100.0);
  declare_parameter("hand_timeout_ms", 200.0);
  declare_parameter("enable_estop", true);
  declare_parameter("initial_controller", "pd_controller");

  control_rate_   = get_parameter("control_rate").as_double();
  enable_logging_ = get_parameter("enable_logging").as_bool();
  enable_estop_   = get_parameter("enable_estop").as_bool();

  if (enable_logging_) {
    const std::string log_dir_str  = get_parameter("log_dir").as_string();
    const int         max_log_files = get_parameter("max_log_files").as_int();
    const std::filesystem::path log_dir{log_dir_str};
    std::filesystem::create_directories(log_dir);
    const std::string log_file = GenerateLogFilePath(log_dir_str);
    logger_ = std::make_unique<urtc::DataLogger>(log_file);
    CleanupOldLogFiles(log_dir, max_log_files);
    RCLCPP_INFO(get_logger(), "Logging to: %s (max_log_files=%d)",
                log_file.c_str(), max_log_files);
  }

  robot_timeout_ = std::chrono::milliseconds(
      static_cast<int>(get_parameter("robot_timeout_ms").as_double()));
  hand_timeout_ = std::chrono::milliseconds(
      static_cast<int>(get_parameter("hand_timeout_ms").as_double()));

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
  } catch (...) {}

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
        const std::string yaml_path = config_dir + std::string(entry.config_key) + ".yaml";
        const YAML::Node file_node  = YAML::LoadFile(yaml_path);
        ctrl->LoadConfig(file_node[entry.config_key]);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(),
          "Config load failed for '%s' (%s) — using defaults",
          ctrl->Name().data(), e.what());
      }
    }

    // Register both the class name (e.g. "PDController") and the config-key
    // alias (e.g. "pd_controller") so either form works as initial_controller.
    name_to_idx[std::string(ctrl->Name())]    = static_cast<int>(i);
    name_to_idx[std::string(entry.config_key)] = static_cast<int>(i);

    controllers_.push_back(std::move(ctrl));
  }

  // Resolve initial_controller parameter → controller index
  const std::string initial_ctrl = get_parameter("initial_controller").as_string();
  const auto it = name_to_idx.find(initial_ctrl);
  if (it != name_to_idx.end()) {
    active_controller_idx_.store(it->second);
  } else {
    RCLCPP_WARN(get_logger(),
      "Unknown initial_controller '%s', defaulting to pd_controller",
      initial_ctrl.c_str());
    const auto pd_it = name_to_idx.find("pd_controller");
    active_controller_idx_.store(pd_it != name_to_idx.end() ? pd_it->second : 1);
  }
}

void RtControllerNode::CreateSubscriptions()
{
  // Assign subscriptions to cb_group_sensor_
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = cb_group_sensor_;

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
    [this](sensor_msgs::msg::JointState::SharedPtr msg) {
      JointStateCallback(std::move(msg));
      },
      sub_options);

  target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/target_joint_positions", 10,
    [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      TargetCallback(std::move(msg));
      },
      sub_options);

  hand_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/hand/joint_states", 10,
    [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      HandStateCallback(std::move(msg));
      },
      sub_options);

  controller_selector_sub_ = create_subscription<std_msgs::msg::Int32>(
      "~/controller_type", 10,
    [this](std_msgs::msg::Int32::SharedPtr msg) {
      int idx = msg->data;
      if (idx >= 0 && idx < static_cast<int>(controllers_.size())) {
        active_controller_idx_.store(idx, std::memory_order_release);
        RCLCPP_INFO(get_logger(), "Switched to controller: %s",
                      controllers_[idx]->Name().data());
      } else {
        RCLCPP_WARN(get_logger(), "Invalid controller index: %d", idx);
      }
      },
      sub_options);

  // Gains subscriber — ~/controller_gains [Float64MultiArray]
  // Each controller defines its own flat-array layout in UpdateGainsFromMsg().
  // See the controller's header comment for the exact layout.
  controller_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/controller_gains", 10,
    [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      const int idx = active_controller_idx_.load(std::memory_order_acquire);
      controllers_[static_cast<std::size_t>(idx)]->UpdateGainsFromMsg(msg->data);
      RCLCPP_INFO(get_logger(), "Gains updated for %s",
        controllers_[static_cast<std::size_t>(idx)]->Name().data());
      },
      sub_options);
}

void RtControllerNode::CreatePublishers()
{
  // Pre-allocate the command message once. ControlLoop() uses try_lock() to
  // avoid blocking on the RT path — same semantics as RealtimePublisher.
  cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);
  cmd_msg_.data.resize(urtc::kNumRobotJoints, 0.0);

  estop_pub_ =
    create_publisher<std_msgs::msg::Bool>("/system/estop_status", 10);
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
}

void RtControllerNode::HandStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr /*msg*/)
{
  {
    std::lock_guard lock(hand_mutex_);
    last_hand_update_ = now();
  }
  // Fix 2: release-store after mutex released
  hand_data_received_.store(true, std::memory_order_release);
}

// ── 50 Hz watchdog (E-STOP) ───────────────────────────────────────────────────
void RtControllerNode::CheckTimeouts()
{
  const auto now_time = now();
  bool robot_timed_out = false;
  bool hand_timed_out = false;

  // Fix 2: acquire-load the atomic flag; if set, lock mutex to read timestamp
  if (state_received_.load(std::memory_order_acquire)) {
    std::lock_guard lock(state_mutex_);
    robot_timed_out = (now_time - last_robot_update_) > robot_timeout_;
  }
  if (hand_data_received_.load(std::memory_order_acquire)) {
    std::lock_guard lock(hand_mutex_);
    hand_timed_out = (now_time - last_hand_update_) > hand_timeout_;
  }

  int active_idx = active_controller_idx_.load(std::memory_order_acquire);
  if (robot_timed_out && !controllers_[active_idx]->IsEstopped()) {
    RCLCPP_ERROR(get_logger(), "Robot data timeout — triggering E-STOP");
    controllers_[active_idx]->TriggerEstop();
    PublishEstopStatus(true);
  }

  if (hand_timed_out) {
    controllers_[active_idx]->SetHandEstop(true);
    if (!hand_estop_logged_) {
      RCLCPP_WARN(get_logger(), "Hand data timeout — hand E-STOP active");
      hand_estop_logged_ = true;
    }
  } else {
    if (hand_estop_logged_) {
      RCLCPP_INFO(get_logger(), "Hand data restored — hand E-STOP cleared");
      hand_estop_logged_ = false;
    }
    controllers_[active_idx]->SetHandEstop(false);
  }
}

// ── 500 Hz control loop ───────────────────────────────────────────────────────
void RtControllerNode::ControlLoop()
{
  // Fix 2: acquire-load atomics — no mutex needed for the readiness check
  if (!state_received_.load(std::memory_order_acquire) ||
    !target_received_.load(std::memory_order_acquire))
  {
    return;
  }

  urtc::ControllerState state{};
  {
    std::lock_guard lock(state_mutex_);
    state.robot.positions  = current_positions_;
    state.robot.velocities = current_velocities_;
  }
  // Fix 4: copy target_positions_ into target_snapshot_ while holding the
  // mutex, then use only the snapshot.  The old code set state.robot.dt and
  // state.iteration inside target_mutex_, which was incorrect.
  {
    std::lock_guard lock(target_mutex_);
    target_snapshot_ = target_positions_;
  }
  state.robot.dt  = 1.0 / control_rate_;
  state.iteration = loop_count_;

  int active_idx = active_controller_idx_.load(std::memory_order_acquire);

  // Measure Compute() wall-clock time via ControllerTimingProfiler.
  const urtc::ControllerOutput output =
    timing_profiler_.MeasuredCompute(*controllers_[active_idx], state);

  // Non-blocking: skip this cycle if publisher mutex is contended.
  if (cmd_pub_mutex_.try_lock()) {
    std::copy(output.robot_commands.begin(), output.robot_commands.end(),
              cmd_msg_.data.begin());
    cmd_pub_->publish(cmd_msg_);
    cmd_pub_mutex_.unlock();
  }

  // Fix 1: push log entry to the SPSC ring buffer — O(1), no syscall.
  // DrainLog() (log thread, Core 4) pops entries and writes the CSV file.
  if (enable_logging_) {
    const double current_time = now().seconds();
    if (loop_count_ == 0) {
      start_time_ = current_time;
    }

    const urtc::LogEntry entry{
      .timestamp         = current_time - start_time_,
      .current_positions = state.robot.positions,
      .target_positions  = output.actual_target_positions,
      .commands          = output.robot_commands,
      .compute_time_us   = timing_profiler_.LastComputeUs(),
    };
    static_cast<void>(log_buffer_.Push(entry));  // silently drops if buffer is full
  }

  ++loop_count_;
  // Print timing summary every 1 000 iterations.
  if (loop_count_ % 1000 == 0) {
    int idx = active_controller_idx_.load(std::memory_order_acquire);
    RCLCPP_INFO(get_logger(), "%s",
                timing_profiler_
      .Summary(std::string(controllers_[idx]->Name()))
      .c_str());
  }
}

// Fix 1: file I/O stays exclusively in the log thread (Core 4).
void RtControllerNode::DrainLog()
{
  if (!logger_) {
    return;
  }
  logger_->DrainBuffer(log_buffer_);
}

void RtControllerNode::PublishEstopStatus(bool estopped)
{
  std_msgs::msg::Bool msg;
  msg.data = estopped;
  estop_pub_->publish(msg);
}
