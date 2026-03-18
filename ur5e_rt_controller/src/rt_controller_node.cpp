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
  RCLCPP_INFO(get_logger(),
      "Threading: clock_nanosleep RT loop + SPSC publish offload + "
      "Sensor/Log/Aux executors");
}

RtControllerNode::~RtControllerNode()
{
  // Stop RT loop + publish thread (idempotent — safe if already stopped by main)
  StopRtLoop();
  StopPublishLoop();

  if (hand_controller_) {
    SaveHandStats();
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

// ── Hand stats export (mirrors HandUdpNode::SaveCommStats) ───────────────────
void RtControllerNode::SaveHandStats() const
{
  if (!hand_controller_) return;

  const auto stats = hand_controller_->comm_stats();
  const auto elapsed = std::chrono::steady_clock::now() - log_start_time_;
  const double elapsed_sec = std::chrono::duration<double>(elapsed).count();
  const double avg_rate_hz = (elapsed_sec > 0.0)
      ? static_cast<double>(stats.total_cycles) / elapsed_sec : 0.0;

  std::string output_dir;
  const char* session_env = std::getenv("UR5E_SESSION_DIR");
  if (session_env != nullptr && session_env[0] != '\0') {
    output_dir = std::string(session_env) + "/hand";
  } else {
    output_dir = "/tmp";
  }

  const std::string path = output_dir + "/hand_udp_stats.json";
  std::ofstream ofs(path);
  if (!ofs.is_open()) return;

  const auto ts = hand_controller_->timing_stats();

  const bool is_bulk = (hand_controller_->communication_mode() ==
                        urtc::HandCommunicationMode::kBulk);
  const char* mode_str = is_bulk ? "bulk" : "individual";

  ofs << std::fixed
      << "{\n"
      << "  \"comm_stats\": {\n"
      << "    \"communication_mode\": \"" << mode_str << "\",\n"
      << "    \"recv_timeout_ms\": " << std::setprecision(3) << hand_controller_->recv_timeout_ms() << ",\n"
      << "    \"total_cycles\": "     << stats.total_cycles    << ",\n"
      << "    \"recv_ok\": "          << stats.recv_ok         << ",\n"
      << "    \"recv_timeout\": "     << stats.recv_timeout     << ",\n"
      << "    \"recv_error\": "       << stats.recv_error       << ",\n"
      << "    \"event_skip_count\": " << stats.event_skip_count << ",\n"
      << "    \"avg_rate_hz\": "      << std::setprecision(2) << avg_rate_hz  << ",\n"
      << "    \"elapsed_sec\": "      << std::setprecision(2) << elapsed_sec  << "\n"
      << "  },\n"
      << "  \"timing_stats\": {\n"
      << "    \"count\": " << ts.count << ",\n"
      << "    \"total_us\": {"
      << " \"mean\": " << std::setprecision(1) << ts.mean_us
      << ", \"min\": "  << ts.min_us
      << ", \"max\": "  << ts.max_us
      << ", \"stddev\": " << ts.stddev_us
      << ", \"p95\": "  << ts.p95_us
      << ", \"p99\": "  << ts.p99_us
      << " },\n"
      << "    \"write_us\": {"
      << " \"mean\": " << ts.write.mean_us
      << ", \"min\": " << ts.write.min_us
      << ", \"max\": " << ts.write.max_us
      << " },\n";

  if (is_bulk) {
    ofs << "    \"read_all_motor_us\": {"
        << " \"mean\": " << ts.read_all_motor.mean_us
        << ", \"min\": " << ts.read_all_motor.min_us
        << ", \"max\": " << ts.read_all_motor.max_us
        << " },\n"
        << "    \"read_all_sensor_us\": {"
        << " \"mean\": " << ts.read_all_sensor.mean_us
        << ", \"min\": " << ts.read_all_sensor.min_us
        << ", \"max\": " << ts.read_all_sensor.max_us
        << ", \"sensor_cycles\": " << ts.sensor_cycle_count
        << " },\n";
  } else {
    ofs << "    \"read_pos_us\": {"
        << " \"mean\": " << ts.read_pos.mean_us
        << ", \"min\": " << ts.read_pos.min_us
        << ", \"max\": " << ts.read_pos.max_us
        << " },\n"
        << "    \"read_vel_us\": {"
        << " \"mean\": " << ts.read_vel.mean_us
        << ", \"min\": " << ts.read_vel.min_us
        << ", \"max\": " << ts.read_vel.max_us
        << " },\n"
        << "    \"read_sensor_us\": {"
        << " \"mean\": " << ts.read_sensor.mean_us
        << ", \"min\": " << ts.read_sensor.min_us
        << ", \"max\": " << ts.read_sensor.max_us
        << ", \"sensor_cycles\": " << ts.sensor_cycle_count
        << " },\n";
  }

  ofs << "    \"over_budget\": " << ts.over_budget << "\n"
      << "  }\n"
      << "}\n";
  ofs.close();

  RCLCPP_INFO(get_logger(),
      "Hand stats saved to %s (cycles=%lu, rate=%.1f Hz)",
      path.c_str(), stats.total_cycles, avg_rate_hz);
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
  declare_parameter("use_fake_hand", false);  // launch argument로만 전달
  global_device_flags_.enable_ur5e = get_parameter("enable_ur5e").as_bool();
  global_device_flags_.enable_hand = get_parameter("enable_hand").as_bool();

  // JointCommand 발행 (MuJoCo / 외부 시뮬레이터 연동)
  declare_parameter("joint_command_topic", std::string("/ur5e/joint_command"));

  // Hand simulation (MuJoCo fake response 연동)
  declare_parameter("hand_sim_enabled", false);
  declare_parameter("hand_command_topic", std::string("/hand/command"));
  declare_parameter("hand_state_topic", std::string("/hand/joint_states"));

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

  // ── Hand Controller (직접 UDP 통신, event-driven) ──────────────────────────
  // hand_udp_node.yaml의 파라미터를 launch에서 로드
  declare_parameter("target_ip", std::string(""));
  declare_parameter("target_port", 0);
  declare_parameter("recv_timeout_ms", 10.0);
  declare_parameter("enable_write_ack", false);
  declare_parameter("sensor_decimation", 1);
  declare_parameter("communication_mode", std::string{"individual"});
  declare_parameter("baro_lpf_enabled", false);
  declare_parameter("baro_lpf_cutoff_hz", 30.0);
  declare_parameter("tof_lpf_enabled", false);
  declare_parameter("tof_lpf_cutoff_hz", 15.0);

  const std::string hand_ip = get_parameter("target_ip").as_string();
  const int hand_port = static_cast<int>(get_parameter("target_port").as_int());
  hand_sim_enabled_ = get_parameter("hand_sim_enabled").as_bool();
  const bool use_fake_hand = get_parameter("use_fake_hand").as_bool();

  // Hand 전체 비활성 시 모든 hand 통신 skip
  if (!global_device_flags_.enable_hand) {
    hand_sim_enabled_ = false;
    enable_hand_ = false;
    RCLCPP_INFO(get_logger(), "Hand disabled (enable_hand=false)");
  } else if (use_fake_hand) {
    // Fake mode: echo-back HandController (UDP 소켓 없이 내부 echo-back)
    hand_sim_enabled_ = false;
    const auto cfgs = ur5e_rt_controller::SelectThreadConfigs();
    const auto fake_ft_names = get_parameter("hand_fingertip_names").as_string_array();
    hand_controller_ = std::make_unique<urtc::HandController>(
        "", 0, cfgs.udp_recv, 10, false, 1, urtc::kDefaultNumFingertips, true,
        fake_ft_names);
    static_cast<void>(hand_controller_->Start());
    enable_hand_ = true;
    RCLCPP_INFO(get_logger(), "HandController started in FAKE mode (echo-back)");
  } else {
    enable_hand_ = !hand_ip.empty() && hand_port > 0 && !hand_sim_enabled_;
  }

  if (hand_sim_enabled_ && global_device_flags_.enable_hand) {
    // ROS 토픽 기반 핸드 통신 (MuJoCo fake response 연동)
    const std::string hand_cmd_topic = get_parameter("hand_command_topic").as_string();
    const std::string hand_state_topic = get_parameter("hand_state_topic").as_string();

    hand_sim_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        hand_cmd_topic, rclcpp::QoS(10));
    hand_sim_cmd_msg_.data.resize(10, 0.0);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_sensor_;
    hand_sim_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        hand_state_topic, rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() < 10) { return; }
          std::lock_guard lock(sim_hand_mutex_);

          if (hand_state_map_built_) {
            // 이름 기반 reorder: source index → internal index
            const std::size_t n = std::min(hand_state_reorder_.size(),
                                           msg->data.size());
            for (std::size_t src_i = 0; src_i < n; ++src_i) {
              const int idx = hand_state_reorder_[src_i];
              if (idx >= 0 && idx < urtc::kNumHandMotors) {
                const auto uidx = static_cast<std::size_t>(idx);
                sim_hand_state_.motor_positions[uidx] =
                    static_cast<float>(msg->data[src_i]);
              }
            }
            // Velocities: [10..19] (same reorder)
            constexpr std::size_t vel_offset = urtc::kNumHandMotors;
            for (std::size_t src_i = 0; src_i < n
                     && (vel_offset + src_i) < msg->data.size(); ++src_i) {
              const int idx = hand_state_reorder_[src_i];
              if (idx >= 0 && idx < urtc::kNumHandMotors) {
                sim_hand_state_.motor_velocities[static_cast<std::size_t>(idx)] =
                    static_cast<float>(msg->data[vel_offset + src_i]);
              }
            }
          } else {
            // Positional fallback (이름 매핑 미빌드 시)
            for (std::size_t i = 0; i < urtc::kNumHandMotors && i < msg->data.size(); ++i) {
              sim_hand_state_.motor_positions[i] = static_cast<float>(msg->data[i]);
            }
            constexpr std::size_t vel_offset = urtc::kNumHandMotors;
            for (std::size_t i = 0; i < urtc::kNumHandMotors
                     && (vel_offset + i) < msg->data.size(); ++i) {
              sim_hand_state_.motor_velocities[i] =
                  static_cast<float>(msg->data[vel_offset + i]);
            }
          }

          // Sensors: [20..63] (4 fingertips × 11 values) — always positional
          constexpr std::size_t sensor_offset = urtc::kNumHandMotors * 2;
          const std::size_t num_sensor_values = msg->data.size() > sensor_offset
              ? msg->data.size() - sensor_offset : 0;
          for (std::size_t i = 0; i < num_sensor_values
                   && i < sim_hand_state_.sensor_data.size(); ++i) {
            sim_hand_state_.sensor_data[i] =
                static_cast<uint32_t>(msg->data[sensor_offset + i]);
          }
          sim_hand_state_.valid = true;
        },
        sub_opts);

    RCLCPP_INFO(get_logger(),
        "Hand simulation mode: cmd=%s, state=%s (UDP disabled)",
        hand_cmd_topic.c_str(), hand_state_topic.c_str());
  } else if (enable_hand_ && !hand_controller_) {
    const double hand_recv_timeout =
        get_parameter("recv_timeout_ms").as_double();
    const bool hand_write_ack = get_parameter("enable_write_ack").as_bool();
    const int sensor_decimation = static_cast<int>(
        get_parameter("sensor_decimation").as_int());
    const auto hand_ft_names = get_parameter("hand_fingertip_names").as_string_array();
    const std::string hand_comm_mode_str = get_parameter("communication_mode").as_string();
    const auto hand_comm_mode = (hand_comm_mode_str == "bulk")
        ? urtc::HandCommunicationMode::kBulk
        : urtc::HandCommunicationMode::kIndividual;
    const bool baro_lpf_enabled = get_parameter("baro_lpf_enabled").as_bool();
    const double baro_lpf_cutoff_hz = get_parameter("baro_lpf_cutoff_hz").as_double();
    const bool tof_lpf_enabled = get_parameter("tof_lpf_enabled").as_bool();
    const double tof_lpf_cutoff_hz = get_parameter("tof_lpf_cutoff_hz").as_double();
    const auto cfgs = ur5e_rt_controller::SelectThreadConfigs();

    hand_controller_ = std::make_unique<urtc::HandController>(
        hand_ip, hand_port, cfgs.udp_recv,
        hand_recv_timeout, hand_write_ack, sensor_decimation,
        urtc::kDefaultNumFingertips, false, hand_ft_names, hand_comm_mode,
        tof_lpf_enabled, tof_lpf_cutoff_hz,
        baro_lpf_enabled, baro_lpf_cutoff_hz);
    hand_controller_->SetEstopFlag(&global_estop_);

    if (hand_controller_->Start()) {
      RCLCPP_INFO(get_logger(), "HandController started: %s:%d (event-driven)",
                  hand_ip.c_str(), hand_port);
      if (!hand_controller_->IsSensorInitialized()) {
        RCLCPP_WARN(get_logger(),
            "Hand sensor init failed — sensors may remain in NN mode");
      }
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

  // ── Joint name loading & URDF validation (v5.14.0) ─────────────────────
  LoadAndValidateJointNames();

  // hand_motor_names_ 기반 identity map 빌드 (sim/UDP 모두 동일 이름 순서 사용)
  BuildHandStateIndexMap(hand_motor_names_);

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
          case urtc::SubscribeRole::kJointState:
            if (created_joint_state_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<sensor_msgs::msg::JointState>(
                entry.topic_name, 10,
                [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                  JointStateCallback(std::move(msg));
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [ur5e/joint_state]: %s",
                          entry.topic_name.c_str());
            }
            break;
          case urtc::SubscribeRole::kGoal:
            if (created_target_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
                entry.topic_name, 10,
                [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                  RobotTargetCallback(std::move(msg));
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [ur5e/goal]: %s",
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
          case urtc::SubscribeRole::kHandState:
            if (hand_sim_enabled_) {
              RCLCPP_INFO(get_logger(), "  Subscribe [hand/hand_state]: %s (via ROS — sim)",
                          entry.topic_name.c_str());
            } else {
              RCLCPP_INFO(get_logger(), "  Subscribe [hand/hand_state]: %s (direct UDP/fake)",
                          entry.topic_name.c_str());
            }
            break;
          case urtc::SubscribeRole::kGoal:
            if (created_target_topics.insert(entry.topic_name).second) {
              auto sub = create_subscription<std_msgs::msg::Float64MultiArray>(
                entry.topic_name, 10,
                [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                  HandTargetCallback(std::move(msg));
                },
                sub_options);
              topic_subscriptions_.push_back(sub);
              RCLCPP_INFO(get_logger(), "  Subscribe [hand/goal]: %s",
                          entry.topic_name.c_str());
            }
            break;
          default:
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
          if (hand_controller_) {
            hold_state.hand = hand_controller_->GetLatestState();
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
                        const char * device_prefix) {
    if (topic_publishers_.count(entry.topic_name) > 0) {
      return;
    }
    // hand_sim_enabled_ 시 kHandCommand는 hand_sim_cmd_pub_ (RELIABLE)이 담당
    // — BEST_EFFORT 중복 publisher 생성 방지 (MuJoCo subscriber QoS 호환)
    if (entry.role == urtc::PublishRole::kHandCommand && hand_sim_enabled_) {
      return;
    }

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
        case urtc::PublishRole::kTrajectoryState:
        case urtc::PublishRole::kControllerState:
          data_size = 18;
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
        create_pub(entry, "ur5e");
      }
    }
    if (any_hand) {
      for (const auto & entry : tc.hand.publish) {
        create_pub(entry, "hand");
      }
    }
  }

  // ── JointCommand publisher (MuJoCo / 외부 시뮬레이터용) ────────────────────
  {
    const std::string jc_topic = get_parameter("joint_command_topic").as_string();
    if (!jc_topic.empty()) {
      joint_command_pub_ = create_publisher<ur5e_msgs::msg::JointCommand>(
          jc_topic, cmd_qos);

      // Pre-allocate message
      joint_command_msg_.joint_names = robot_joint_names_;
      joint_command_msg_.values.resize(urtc::kNumRobotJoints, 0.0);
      joint_command_msg_.command_type = "position";

      RCLCPP_INFO(get_logger(), "  Publish [joint_command]: %s (JointCommand msg)",
                  jc_topic.c_str());
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
          "Initialization timeout (%.1f s): robot=%d, target=%d, hand=%s",
          static_cast<double>(init_timeout_ticks_) / control_rate_,
          state_received_.load(std::memory_order_relaxed) ? 1 : 0,
          target_received_.load(std::memory_order_relaxed) ? 1 : 0,
          hand_sim_enabled_ ? "sim" :
          (enable_hand_ ? (hand_controller_ && hand_controller_->IsRunning() ? "running" : "stopped") : "disabled"));
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

      // Hand state도 읽기
      if (hand_controller_) {
        hold_state.hand = hand_controller_->GetLatestState();
      } else if (hand_sim_enabled_) {
        std::lock_guard lock(sim_hand_mutex_);
        hold_state.hand = sim_hand_state_;
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
            "Initialization timeout (%.1f s): robot=%d, target=%d, hand=%s",
            static_cast<double>(init_timeout_ticks_) / control_rate_,
            1,
            target_received_.load(std::memory_order_relaxed) ? 1 : 0,
            hand_sim_enabled_ ? "sim" :
            (enable_hand_ ? (hand_controller_ && hand_controller_->IsRunning() ? "running" : "stopped") : "disabled"));
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

  // Hand state — HandController 또는 ROS 토픽에서 읽기
  if (hand_controller_) {
    cached_hand_state_ = hand_controller_->GetLatestState();
  } else if (hand_sim_enabled_) {
    std::unique_lock lock(sim_hand_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      cached_hand_state_ = sim_hand_state_;
    }
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
    const urtc::PublishSnapshot snap{
      .robot_commands        = output.robot_commands,
      .command_type          = output.command_type,
      .actual_task_positions = output.actual_task_positions,
      .goal_positions        = output.goal_positions,
      .actual_target_positions = output.actual_target_positions,
      .target_velocities     = output.target_velocities,
      .actual_positions      = state.robot.positions,
      .actual_velocities     = state.robot.velocities,
      .hand_commands         = output.hand_commands,
      .stamp_ns              = std::chrono::steady_clock::now()
                                 .time_since_epoch().count(),
      .active_controller_idx = active_idx,
      .ur5e_enabled          = active_df.enable_ur5e,
      .hand_enabled          = active_df.enable_hand,
      .hand_sim_enabled      = hand_sim_enabled_,
    };
    static_cast<void>(publish_buffer_.Push(snap));
  }

  // ── Phase 3.5: hand command 전송 (RT path — condvar notify, ~100ns) ────
  // HandController::SendCommandAndRequestStates()는 non-blocking condvar
  // notify로 hand UDP EventLoop를 깨움. 실제 I/O는 hand thread에서 수행.
  // hand_sim_cmd_pub_ publish는 publish thread로 이동됨 (snap.hand_sim_enabled).
  {
    const auto & active_df_35 = controller_device_flags_[
        static_cast<std::size_t>(active_idx)];
    if (active_df_35.enable_hand && hand_controller_) {
      hand_controller_->SendCommandAndRequestStates(output.hand_commands);
    }
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

    const urtc::LogEntry entry{
      .timestamp = timestamp,
      // Timing
      .t_state_acquire_us = t_state_us,
      .t_compute_us = t_compute_us,
      .t_publish_us = t_publish_us,
      .t_total_us = t_total_us,
      .jitter_us = jitter_us,
      // Robot — 카테고리 1: Goal State
      .goal_positions = output.goal_positions,
      // Robot — 카테고리 2: Current State
      .actual_positions = state.robot.positions,
      .actual_velocities = state.robot.velocities,
      .actual_torques = state.robot.torques,
      .actual_task_positions = output.actual_task_positions,
      // Robot — 카테고리 3: Control Command
      .robot_commands = output.robot_commands,
      .command_type = output.command_type,
      // Robot — 카테고리 4: Trajectory State
      .trajectory_positions = output.actual_target_positions,
      .trajectory_velocities = output.target_velocities,
      // Hand — 카테고리 1: Goal State
      .hand_valid = state.hand.valid,
      .hand_goal_positions = output.hand_goal_positions,
      // Hand — 카테고리 2: Current State
      .hand_actual_positions = state.hand.motor_positions,
      .hand_actual_velocities = state.hand.motor_velocities,
      .hand_sensors = state.hand.sensor_data,
      // Hand — 카테고리 3: Control Command
      .hand_commands = output.hand_commands,
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

    // Helper: publish a single topic entry from snapshot data
    auto publish_entry = [&](const urtc::PublishTopicEntry & pt) {
      auto it = topic_publishers_.find(pt.topic_name);
      if (it == topic_publishers_.end()) { return; }

      auto & pe = it->second;
      switch (pt.role) {
        case urtc::PublishRole::kPositionCommand:
          if (snap.command_type == urtc::CommandType::kPosition) {
            std::copy(snap.robot_commands.begin(), snap.robot_commands.end(),
                      pe.msg.data.begin());
            pe.publisher->publish(pe.msg);
          }
          break;
        case urtc::PublishRole::kTorqueCommand:
          if (snap.command_type == urtc::CommandType::kTorque) {
            std::copy(snap.robot_commands.begin(), snap.robot_commands.end(),
                      pe.msg.data.begin());
            pe.publisher->publish(pe.msg);
          }
          break;
        case urtc::PublishRole::kHandCommand:
          // Hand command는 RT 경로에서 HandController로 직접 전송 (Phase 3.5)
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
      }
    };

    // Publish ur5e topics
    if (snap.ur5e_enabled) {
      for (const auto & pt : active_tc.ur5e.publish) {
        publish_entry(pt);
      }
    }

    // Publish hand topics (ROS only — hand UDP is handled in RT Phase 3.5)
    if (snap.hand_enabled) {
      for (const auto & pt : active_tc.hand.publish) {
        publish_entry(pt);
      }
    }

    // JointCommand publish (MuJoCo / external simulator)
    if (joint_command_pub_) {
      // Convert monotonic ns → ROS2 Time
      const auto sec  = static_cast<int32_t>(snap.stamp_ns / 1'000'000'000L);
      const auto nsec = static_cast<uint32_t>(snap.stamp_ns % 1'000'000'000L);
      joint_command_msg_.header.stamp.sec = sec;
      joint_command_msg_.header.stamp.nanosec = nsec;
      joint_command_msg_.command_type =
          (snap.command_type == urtc::CommandType::kTorque) ? "torque" : "position";
      std::copy(snap.robot_commands.begin(), snap.robot_commands.end(),
                joint_command_msg_.values.begin());
      joint_command_pub_->publish(joint_command_msg_);
    }

    // Hand sim command (MuJoCo fake hand — ROS topic)
    if (snap.hand_sim_enabled && hand_sim_cmd_pub_) {
      for (std::size_t i = 0; i < 10 && i < snap.hand_commands.size(); ++i) {
        hand_sim_cmd_msg_.data[i] = static_cast<double>(snap.hand_commands[i]);
      }
      hand_sim_cmd_pub_->publish(hand_sim_cmd_msg_);
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
