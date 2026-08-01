#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"
#include <rtc_base/threading/thread_utils.hpp>
#include <rtc_base/tracing/trace_scope.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

UdpHandNode::UdpHandNode() : LifecycleNode("udp_hand_node") {
  // Lifecycle design: the constructor allocates no *resources* — publishers,
  // subscriptions, timers and the controller are on_configure's job.
  //
  // The two exceptions below describe the process *thread layout*, which main()
  // must know before it spins and therefore before any lifecycle transition can
  // occur (issue #345):
  //   - cb_group_aux_ is handed to the aux executor at startup, and must be the
  //     same object across an on_cleanup -> on_configure cycle (see the member).
  //   - aux_cpu_slot configures the hand_aux_io thread main() creates. Launch /
  //     YAML values are applied at declare time, so declaring here still picks
  //     up the configured value.
  cb_group_aux_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                        /*automatically_add_to_executor_with_node=*/false);
  declare_parameter("aux_cpu_slot", 0);
}

int UdpHandNode::AuxCpuSlot() const {
  return static_cast<int>(get_parameter("aux_cpu_slot").as_int());
}

UdpHandNode::~UdpHandNode() {
  // Safety net — idempotent cleanup in case lifecycle callbacks were not
  // invoked (e.g. SIGTERM without graceful shutdown).
  if (failure_detector_)
    failure_detector_->Stop();
  if (controller_ && controller_->IsRunning())
    controller_->Stop();
  SaveCommStats();
}

UdpHandNode::CallbackReturn UdpHandNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
  // ── Parameters ─────────────────────────────────────────────────────
  declare_parameter("target_ip", std::string{"192.168.1.2"});
  declare_parameter("target_port", 55151);
  declare_parameter("publish_rate", 100.0);
  // Self-clocked CommLoop cadence (Hz). Drives the autonomous read/state-publish
  // period; write is command-gated. Independent of publish_rate (which only sets
  // the link_status decimation ratio below).
  declare_parameter("loop_rate_hz", 500.0);
  declare_parameter("recv_timeout_ms", 10.0);
  declare_parameter("enable_failure_detector", true);
  declare_parameter("failure_threshold", 5);
  declare_parameter("check_motor", true);
  declare_parameter("check_sensor", true);
  declare_parameter("min_rate_hz", 30.0);
  declare_parameter("rate_fail_threshold", 5);
  declare_parameter("check_link", true);
  // Link-down declared as a time budget (ms), converted to a consecutive-recv-
  // failure cycle count at on_configure via loop_rate_hz / comm_decimation. A
  // time param stays meaningful across loop-rate / decimation changes, unlike a
  // raw cycle count. Default 100 ms sits an order below the CM device_timeout
  // (1000 ms) so the hand link-down latches first.
  declare_parameter("link_fail_timeout_ms", 100.0);
  // Independent startup-grace windows. startup_grace_ms warms the RATE check,
  // link_startup_grace_ms spans the ARP / first-packet transient, and
  // sensor_startup_grace_ms optionally spans a firmware ADC stabilization period.
  declare_parameter("startup_grace_ms", 1000.0);
  declare_parameter("link_startup_grace_ms", 100.0);
  declare_parameter("sensor_startup_grace_ms", 0.0);

  // SIL entry point. Device-side firmware-process spawn is the launch layer's
  // job (a node cannot spawn a peer process); this param drives the node-side
  // effects, interpreted in the target_ip block below:
  //   off       — real hardware. use_fake_hand_=false, socket to target_ip.
  //   loopmodel — controller-side in-process LPF. use_fake_hand_=true, no socket.
  //   firmware  — device-side fake_hand_firmware over loopback. use_fake_hand_=false,
  //               target_ip forced to 127.0.0.1 (launch spawns the peer process).
  declare_parameter("sil_mode", std::string{"off"});
  // Fake-hand dynamics (sil_mode=loopmodel only). The CommLoop RT thread runs
  // exactly as in real mode; the commanded pose is driven through a first-order
  // LPF to produce read-back pos, its derivative (vel), and a PD-torque effort.
  // See udp_hand_controller.hpp StepFakeModel / README sil_mode table.
  declare_parameter("fake_lpf_time_constant_s", 0.1);
  declare_parameter("fake_effort_stiffness", 1.0);
  declare_parameter("fake_effort_damping", 0.1);

  // Whole-cycle UDP decimation (load-reduction test knob): 1 = communicate every
  // cycle (default). N>1 skips the entire UDP transaction on N-1 of every N
  // EventLoop cycles. See udp_hand_controller.hpp / README "통신 decimation".
  declare_parameter("comm_decimation", 1);

  // Sensor-read decimation: 1 = read sensors every comm cycle (default). N>1
  // reads sensors on 1 of every N comm cycles (motor/joint reads unaffected —
  // unlike comm_decimation, which skips the whole transaction).
  declare_parameter("sensor_decimation", 1);

  declare_parameter("joint_state_names", std::vector<std::string>{});
  declare_parameter("motor_state_names", std::vector<std::string>{});
  declare_parameter("hand_fingertip_names", std::vector<std::string>{});

  // Per-joint position offset in degrees (joint_state_names order, 10 values).
  // Converted to radians into joint_offset_rad_ below. Empty → all-zero.
  declare_parameter("joint_position_offsets_deg", std::vector<double>{});

  declare_parameter("communication_mode", std::string{"individual"});

  declare_parameter("baro_lpf_enabled", false);
  declare_parameter("baro_lpf_cutoff_hz", 30.0);
  declare_parameter("tof_lpf_enabled", false);
  declare_parameter("tof_lpf_cutoff_hz", 15.0);

  declare_parameter("drift_detection_enabled", false);
  declare_parameter("drift_threshold", 5.0);
  declare_parameter("drift_window_size", 2500);

  declare_parameter("ft_inferencer.enabled", false);
  declare_parameter("ft_inferencer.num_fingertips", 4);
  declare_parameter("ft_inferencer.history_length", udp_hand_driver::kFTHistoryLength);
  declare_parameter("ft_inferencer.model_paths", std::vector<std::string>{});
  declare_parameter("ft_inferencer.calibration_enabled", true);
  declare_parameter("ft_inferencer.calibration_samples", 500);
  for (const auto& name : {"thumb", "index", "middle", "ring"}) {
    declare_parameter("ft_inferencer." + std::string(name) + "_max", std::vector<double>(16, 1.0));
  }

  // ── SIL mode → use_fake_hand_ + effective target_ip ──────────────────
  const std::string sil_mode = get_parameter("sil_mode").as_string();
  if (sil_mode != "off" && sil_mode != "loopmodel" && sil_mode != "firmware") {
    RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(),
                 "Invalid sil_mode '%s' (expected \"off\", \"loopmodel\", or \"firmware\")",
                 sil_mode.c_str());
    return CallbackReturn::FAILURE;
  }
  use_fake_hand_ = (sil_mode == "loopmodel");
  std::string target_ip = get_parameter("target_ip").as_string();
  if (sil_mode == "firmware") {
    if (target_ip != "127.0.0.1") {
      RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                  "sil_mode=firmware → target_ip forced to 127.0.0.1 (was %s)", target_ip.c_str());
    }
    target_ip = "127.0.0.1";
  }
  const int target_port = static_cast<int>(get_parameter("target_port").as_int());
  const double recv_timeout_ms = get_parameter("recv_timeout_ms").as_double();
  const std::string comm_mode_str = get_parameter("communication_mode").as_string();
  const auto comm_mode = (comm_mode_str == "bulk")
                             ? udp_hand_driver::HandCommunicationMode::kBulk
                             : udp_hand_driver::HandCommunicationMode::kIndividual;

  // ── Sensor protocol (firmware version seam: 1a / 1b) ─────────────────
  declare_parameter("protocol_version", std::string{"1a"});
  const std::string protocol_version = get_parameter("protocol_version").as_string();
  auto sensor_protocol = udp_hand_driver::CreateSensorProtocol(protocol_version);
  if (!sensor_protocol) {
    RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(),
                 "Unsupported protocol_version '%s' (expected \"1a\" or \"1b\")",
                 protocol_version.c_str());
    return CallbackReturn::FAILURE;
  }
  // Cache the publish-layout capability off the hot path. Force-layout protocols
  // (1b) route sensor decode only through the bulk path, so they require bulk.
  sensor_uses_force_layout_ = !sensor_protocol->RunsSensorPostProcess();
  if (sensor_uses_force_layout_ && comm_mode != udp_hand_driver::HandCommunicationMode::kBulk) {
    RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(),
                 "protocol_version '%s' (force sensor layout) requires "
                 "communication_mode \"bulk\" (got \"%s\")",
                 protocol_version.c_str(), comm_mode_str.c_str());
    return CallbackReturn::FAILURE;
  }

  const bool baro_lpf_enabled = get_parameter("baro_lpf_enabled").as_bool();
  const double baro_lpf_cutoff_hz = get_parameter("baro_lpf_cutoff_hz").as_double();
  const bool tof_lpf_enabled = get_parameter("tof_lpf_enabled").as_bool();
  const double tof_lpf_cutoff_hz = get_parameter("tof_lpf_cutoff_hz").as_double();

  udp_hand_driver::FingertipFTInferencer::Config ft_config;
  ft_config.enabled = get_parameter("ft_inferencer.enabled").as_bool();
  ft_config.num_fingertips =
      static_cast<int>(get_parameter("ft_inferencer.num_fingertips").as_int());
  ft_config.history_length =
      static_cast<int>(get_parameter("ft_inferencer.history_length").as_int());
  ft_config.model_paths = get_parameter("ft_inferencer.model_paths").as_string_array();

  {
    const std::string models_dir =
        ament_index_cpp::get_package_share_directory("udp_hand_driver") + "/models/";
    for (auto& p : ft_config.model_paths) {
      if (!p.empty() && p[0] != '/') {
        p = models_dir + p;
      }
    }
  }

  ft_config.calibration_enabled = get_parameter("ft_inferencer.calibration_enabled").as_bool();
  ft_config.calibration_samples =
      static_cast<int>(get_parameter("ft_inferencer.calibration_samples").as_int());

  const std::array<const char*, 4> ft_param_names = {"thumb", "index", "middle", "ring"};
  for (int f = 0; f < 4 && f < ft_config.num_fingertips; ++f) {
    auto max_vec = get_parameter("ft_inferencer." +
                                 std::string(ft_param_names[static_cast<std::size_t>(f)]) + "_max")
                       .as_double_array();
    for (int b = 0; b < udp_hand_driver::kFTInputSize && b < static_cast<int>(max_vec.size());
         ++b) {
      ft_config.input_max[static_cast<std::size_t>(f)][static_cast<std::size_t>(b)] =
          static_cast<float>(max_vec[static_cast<std::size_t>(b)]);
    }
  }

  const bool drift_enabled = get_parameter("drift_detection_enabled").as_bool();
  const double drift_threshold = get_parameter("drift_threshold").as_double();
  const int drift_window_size = static_cast<int>(get_parameter("drift_window_size").as_int());

  // ── UdpHandController ─────────────────────────────────────────────────
  const auto ft_names = get_parameter("hand_fingertip_names").as_string_array();
  num_fingertips_ = udp_hand_driver::kDefaultNumFingertips;
  // use_fake_hand_ was derived from sil_mode in the target_ip block above.
  const int comm_decimation = static_cast<int>(get_parameter("comm_decimation").as_int());
  const int sensor_decimation = static_cast<int>(get_parameter("sensor_decimation").as_int());
  const double loop_rate_hz = get_parameter("loop_rate_hz").as_double();
  const double fake_lpf_time_constant_s = get_parameter("fake_lpf_time_constant_s").as_double();
  const double fake_effort_stiffness = get_parameter("fake_effort_stiffness").as_double();
  const double fake_effort_damping = get_parameter("fake_effort_damping").as_double();
  controller_ =
      std::make_unique<udp_hand_driver::UdpHandController>(udp_hand_driver::UdpHandControllerConfig{
          .target_ip = target_ip,
          .target_port = target_port,
          .recv_timeout_ms = recv_timeout_ms,
          .sensor_decimation = sensor_decimation,
          .num_fingertips = num_fingertips_,
          .use_fake_hand = use_fake_hand_,
          .fingertip_names = ft_names,
          .communication_mode = comm_mode,
          .tof_lpf_enabled = tof_lpf_enabled,
          .tof_lpf_cutoff_hz = tof_lpf_cutoff_hz,
          .baro_lpf_enabled = baro_lpf_enabled,
          .baro_lpf_cutoff_hz = baro_lpf_cutoff_hz,
          .ft_config = std::move(ft_config),
          .drift_detection_enabled = drift_enabled,
          .drift_threshold = drift_threshold,
          .drift_window_size = drift_window_size,
          .comm_decimation = comm_decimation,
          .loop_rate_hz = loop_rate_hz,
          .fake_lpf_time_constant_s = fake_lpf_time_constant_s,
          .fake_effort_stiffness = fake_effort_stiffness,
          .fake_effort_damping = fake_effort_damping,
      });
  controller_->SetSensorProtocol(std::move(sensor_protocol));

  // ── Topic names ──────────────────────────────────────────────────
  declare_parameter("command_topic", std::string("/hand/joint_command"));
  declare_parameter("joint_state_topic", std::string("/hand/joint_states"));
  declare_parameter("motor_state_topic", std::string("/hand/motor_states"));
  declare_parameter("sensor_topic", std::string("/hand/sensor_states"));
  declare_parameter("link_status_topic", std::string("/hand/link_status"));
  declare_parameter("calibration_command_topic", std::string("/hand/calibration/command"));
  declare_parameter("calibration_status_topic", std::string("/hand/calibration/status"));
  declare_parameter("calibration_status_rate_hz", 5.0);
  const std::string cmd_topic = get_parameter("command_topic").as_string();
  const std::string joint_state_topic = get_parameter("joint_state_topic").as_string();
  const std::string motor_state_topic = get_parameter("motor_state_topic").as_string();
  const std::string sensor_topic = get_parameter("sensor_topic").as_string();
  const std::string link_status_topic = get_parameter("link_status_topic").as_string();
  const std::string calib_cmd_topic = get_parameter("calibration_command_topic").as_string();
  const std::string calib_status_topic = get_parameter("calibration_status_topic").as_string();
  const double calib_status_rate_hz = get_parameter("calibration_status_rate_hz").as_double();

  // ── Publishers (LifecyclePublisher — gated by lifecycle state) ─────
  rclcpp::QoS sensor_pub_qos{1};
  sensor_pub_qos.best_effort();

  joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, sensor_pub_qos);
  motor_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(motor_state_topic, sensor_pub_qos);
  sensor_state_pub_ =
      create_publisher<rtc_msgs::msg::HandSensorState>(sensor_topic, sensor_pub_qos);

  sensor_monitor_pub_ =
      create_publisher<rtc_msgs::msg::HandSensorState>(sensor_topic + "/monitor", rclcpp::QoS{1});

  // Link status: standalone rclcpp::Publisher (NOT LifecyclePublisher).
  // Must remain publishable in any lifecycle state — link status is a
  // safety-relevant signal that should report even when Inactive.
  rclcpp::QoS link_pub_qos{1};
  link_pub_qos.transient_local();
  link_status_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
      this->get_node_topics_interface(), link_status_topic, link_pub_qos);
  // Convert the time-based link-fail budget to a cycle count once here; the three
  // consumers (failure detector CheckLink, publish link_ok, stats JSON link_ok)
  // all read this single member so they cannot drift. loop_rate_hz /
  // comm_decimation are the on_configure locals above.
  {
    const double link_fail_timeout_ms = get_parameter("link_fail_timeout_ms").as_double();
    // Motor/joint attempted every comm cycle → comm-rate cycles. Sensor attempted
    // every sensor_decimation-th comm cycle → the extra divisor gives the shorter
    // sensor-rate cycle count for the SAME wall-clock budget (#1 unit conversion).
    const int link_fail_cycles = udp_hand_driver::LinkFailCyclesFromTimeoutMs(
        link_fail_timeout_ms, loop_rate_hz, comm_decimation);
    const int link_fail_cycles_sensor = udp_hand_driver::LinkFailCyclesFromTimeoutMs(
        link_fail_timeout_ms, loop_rate_hz, comm_decimation, sensor_decimation);
    link_fail_threshold_ = static_cast<uint64_t>(link_fail_cycles);
    link_fail_threshold_sensor_ = static_cast<uint64_t>(link_fail_cycles_sensor);
    RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                "link_fail_timeout_ms=%.1f -> comm=%d cycles, sensor=%d cycles "
                "(loop_rate=%.1f Hz, comm_decimation=%d, sensor_decimation=%d)",
                link_fail_timeout_ms, link_fail_cycles, link_fail_cycles_sensor, loop_rate_hz,
                comm_decimation, sensor_decimation);
  }

  ft_enabled_ = ft_config.enabled;

  // ── Hand joint/motor/fingertip names ────────────────────────────────
  auto joint_names = get_parameter("joint_state_names").as_string_array();
  if (joint_names.empty()) {
    joint_names = udp_hand_driver::kDefaultHandMotorNames;
  }
  joint_names_ = joint_names;
  auto motor_names = get_parameter("motor_state_names").as_string_array();
  if (motor_names.empty()) {
    motor_names = udp_hand_driver::kDefaultHandMotorNames;
  }
  motor_names_ = motor_names;
  auto fingertip_names = get_parameter("hand_fingertip_names").as_string_array();
  if (fingertip_names.empty()) {
    fingertip_names = udp_hand_driver::kDefaultFingertipNames;
  }
  fingertip_names_ = fingertip_names;

  // ── Joint position offsets (degree YAML → radian) ──────────────────
  // Boundary conversion (degrees only at the config edge; radians internal).
  // Applied at +offset on publish and −offset on command so the round-trip
  // is identity. Size must match kNumHandMotors; otherwise fall back to zero
  // (same lenient pattern as the *_names params above).
  joint_offset_rad_.fill(0.0f);
  {
    const auto offsets_deg = get_parameter("joint_position_offsets_deg").as_double_array();
    if (!offsets_deg.empty()) {
      if (offsets_deg.size() != static_cast<std::size_t>(udp_hand_driver::kNumHandMotors)) {
        RCLCPP_WARN(
            ::udp_hand_driver::logging::NodeLogger(),
            "joint_position_offsets_deg size %zu (expected %d) — ignoring, using zero offset",
            offsets_deg.size(), udp_hand_driver::kNumHandMotors);
      } else {
        constexpr double kDeg2Rad = M_PI / 180.0;
        for (std::size_t i = 0; i < offsets_deg.size(); ++i) {
          joint_offset_rad_[i] = static_cast<float>(offsets_deg[i] * kDeg2Rad);
        }
        RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                    "Joint position offsets applied (%d joints, degree YAML → radian)",
                    udp_hand_driver::kNumHandMotors);
      }
    }
  }

  // ── Link status decimation ─────────────────────────────────────────
  // link_status is decimated relative to the CommLoop cadence: one link_status
  // publish per (loop_rate_hz / publish_rate) state cycles (e.g. 500/100 = 5 →
  // ~100 Hz link_status at a 500 Hz loop). Derived from loop_rate_hz, not a
  // hardcoded 500, so a non-500 loop keeps the intended link_status rate.
  const double publish_rate = get_parameter("publish_rate").as_double();
  // State-publish callbacks fire once per comm cycle — i.e. at
  // loop_rate_hz / comm_decimation, not loop_rate_hz — because decimated skip
  // cycles do no publish. Base the link_status decimation on that effective
  // cadence so the intended publish_rate holds when comm_decimation > 1. Guard
  // publish_rate > 0 (a non-positive value would divide by zero / yield garbage).
  const int comm_dec_guarded = udp_hand_driver::ClampCommDecimation(comm_decimation);
  const double effective_state_hz = loop_rate_hz / static_cast<double>(comm_dec_guarded);
  link_decimation_ =
      (publish_rate > 0.0) ? std::max(1, static_cast<int>(effective_state_hz / publish_rate)) : 1;

  // ── Pre-allocate ROS2 messages ─────────────────────────────────────
  PreallocateMessages();

  // ── NRT publish lane ───────────────────────────────────────────────
  // Pull, not push (issue #345). The CommLoop stores a POD snapshot into the
  // SeqLock and nothing else; this timer polls the sequence from the default
  // executor thread and publishes whatever the latest snapshot is.
  //
  // Poll faster than the producer. With a same-rate poll, producer and consumer
  // drift in and out of phase and every coincidence where two comm cycles land
  // between two polls costs a publish that is never recovered — the topic rate
  // would sit measurably below loop_rate_hz. Oversampling makes "at most one new
  // sample per poll" the common case instead of the lossy one, at the price of
  // wakeups that cost one atomic load when there is nothing new.
  last_published_seq_ = controller_->state_sequence();
  const auto publish_period = std::chrono::duration<double>(
      1.0 / (loop_rate_hz * udp_hand_driver::kPublishPollOversampling));
  publish_timer_ =
      create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
                        [this]() { PollAndPublish(); });

  // ── Subscriptions ──────────────────────────────────────────────────
  rclcpp::QoS cmd_sub_qos{1};
  cmd_sub_qos.best_effort();

  joint_command_sub_ = create_subscription<rtc_msgs::msg::JointCommand>(
      cmd_topic, cmd_sub_qos, [this](rtc_msgs::msg::JointCommand::SharedPtr msg) {
        if (!controller_->IsRunning()) {
          return;
        }
        if (msg->values.size() < static_cast<std::size_t>(udp_hand_driver::kNumHandMotors)) {
          RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                      "JointCommand values size %zu (expected %d)", msg->values.size(),
                      udp_hand_driver::kNumHandMotors);
          return;
        }
        // L2 under the executor's ros2:callback_* (L1) — the name-reorder +
        // offset + SendCommandAndRequestStates body (guards excluded above).
        RTC_TRACE_SCOPE("hand_joint_command_cb");
        // The publisher labels values in its own (controller / URDF) joint_names
        // order, which can differ from this hand's firmware slot order
        // (joint_names_ = joint_state_names). Build the firmware-slot ← message
        // map once and remap; fall back to positional consumption (with a
        // one-shot warning) when the message omits names or a firmware joint is
        // unmatched.
        if (!cmd_name_reorder_ready_ && !msg->joint_names.empty()) {
          cmd_name_reorder_ =
              udp_hand_driver::BuildCommandNameReorder(joint_names_, msg->joint_names);
          bool complete = true;
          for (std::size_t i = 0; i < static_cast<std::size_t>(udp_hand_driver::kNumHandMotors);
               ++i) {
            if (cmd_name_reorder_[i] < 0) {
              complete = false;
              break;
            }
          }
          if (complete) {
            cmd_name_reorder_ready_ = true;
          } else {
            RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                        "JointCommand joint_names do not cover all %d hand joints — using "
                        "positional order",
                        udp_hand_driver::kNumHandMotors);
          }
        }
        std::array<float, udp_hand_driver::kNumHandMotors> cmd;
        for (std::size_t i = 0; i < static_cast<std::size_t>(udp_hand_driver::kNumHandMotors);
             ++i) {
          // Pick the source value by name (firmware slot i ← published index),
          // then subtract the per-joint offset: controller frame → firmware
          // frame. offset stays aligned to firmware slot order (joint_names_).
          std::size_t src = i;
          if (cmd_name_reorder_ready_) {
            src = static_cast<std::size_t>(cmd_name_reorder_[i]);
          }
          const float value =
              (src < msg->values.size()) ? static_cast<float>(msg->values[src]) : 0.0f;
          cmd[i] = value - joint_offset_rad_[i];
        }
        controller_->SendCommandAndRequestStates(cmd);
      });

  calib_cmd_sub_ = create_subscription<rtc_msgs::msg::CalibrationCommand>(
      calib_cmd_topic, rclcpp::QoS(1).reliable(),
      [this](rtc_msgs::msg::CalibrationCommand::SharedPtr msg) {
        if (!controller_ || !controller_->IsRunning()) {
          RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                      "Calibration command ignored: controller not running");
          return;
        }
        RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                    "Calibration command: sensor_type=%u action=%u sample_count=%u",
                    static_cast<unsigned>(msg->sensor_type), static_cast<unsigned>(msg->action),
                    static_cast<unsigned>(msg->sample_count));
        controller_->RequestCalibration(msg->sensor_type, msg->action, msg->sample_count);
      });

  rclcpp::QoS calib_status_qos{1};
  calib_status_qos.reliable().transient_local();
  calib_status_pub_ =
      create_publisher<rtc_msgs::msg::CalibrationStatus>(calib_status_topic, calib_status_qos);

  const int status_period_ms =
      (calib_status_rate_hz > 0.0) ? static_cast<int>(1000.0 / calib_status_rate_hz) : 200;
  calib_status_timer_ = create_wall_timer(std::chrono::milliseconds(status_period_ms),
                                          [this]() { PublishCalibrationStatus(); });

  // Cache config values needed by on_activate
  target_ip_ = target_ip;
  target_port_ = target_port;
  comm_mode_str_ = comm_mode_str;

  {
    std::string names_str;
    for (std::size_t i = 0; i < motor_names.size(); ++i) {
      if (i > 0)
        names_str += ", ";
      names_str += motor_names[i];
    }
    RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "Hand motor order (%zu): [%s]",
                motor_names.size(), names_str.c_str());
  }

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
              "UdpHandNode configured: target %s:%d, direct publish from "
              "EventLoop, comm=%s, protocol=%s",
              target_ip.c_str(), target_port, comm_mode_str.c_str(), protocol_version.c_str());
  return CallbackReturn::SUCCESS;
}

UdpHandNode::CallbackReturn UdpHandNode::on_activate(const rclcpp_lifecycle::State& state) {
  // Parent call activates LifecyclePublishers — must be called first.
  LifecycleNode::on_activate(state);

  start_time_ = std::chrono::steady_clock::now();

  // ── Per-tick timing CSV setup ──────────────────────────────────────
  // Inject producer before Start() so the CommLoop thread sees a non-null
  // producer on its first iteration. The CommLoop (PeriodicRtThread) computes
  // jitter against its own loop_rate_hz period budget.
  controller_->SetTimingProducer(&hand_udp_timing_producer_);

  if (!controller_->Start()) {
    RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(),
                 "Failed to start UdpHandController to %s:%d", target_ip_.c_str(), target_port_);
    return CallbackReturn::FAILURE;
  }

  // Fake mode needs no node-side self-tick: the controller's CommLoop RT thread
  // self-clocks at loop_rate_hz exactly as in real mode and latches the staged
  // command each cycle (see UdpHandController::RunFakeCommCycle). Commands flow
  // through the normal command sub → SendCommandAndRequestStates staging path.

  // Failure Detector (skipped in fake mode)
  const bool enable_fd = get_parameter("enable_failure_detector").as_bool() && !use_fake_hand_;
  if (enable_fd) {
    udp_hand_driver::UdpHandFailureDetector::Config fd_cfg;
    fd_cfg.failure_threshold = static_cast<int>(get_parameter("failure_threshold").as_int());
    fd_cfg.check_motor = get_parameter("check_motor").as_bool();
    fd_cfg.check_sensor = get_parameter("check_sensor").as_bool();
    // 1b force layout → detector checks fx,fy,fz in sensor_force, not the int32
    // sensor_data buffer (which stays zero on 1b and would false-trigger).
    fd_cfg.sensor_force_layout = sensor_uses_force_layout_;
    // The rate detector measures controller_.cycle_count() growth, which
    // increments once per comm cycle — i.e. at loop_rate_hz / comm_decimation,
    // since decimated skip cycles do no I/O and no cycle accounting. A
    // min_rate_hz set against the nominal loop rate would therefore false-
    // trigger when comm_decimation > 1. Scale the floor down to match.
    const int comm_dec = udp_hand_driver::ClampCommDecimation(
        static_cast<int>(get_parameter("comm_decimation").as_int()));
    const double min_rate_nominal = get_parameter("min_rate_hz").as_double();
    fd_cfg.min_rate_hz = min_rate_nominal / static_cast<double>(comm_dec);
    if (comm_dec > 1) {
      RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                  "min_rate_hz scaled by comm_decimation=%d: %.1f -> %.1f Hz", comm_dec,
                  min_rate_nominal, fd_cfg.min_rate_hz);
    }
    fd_cfg.rate_fail_threshold = static_cast<int>(get_parameter("rate_fail_threshold").as_int());
    fd_cfg.check_link = get_parameter("check_link").as_bool();
    // Per-channel cycle counts converted from link_fail_timeout_ms at
    // on_configure — the SAME values the publish / stats link_ok reads through
    // UdpHandController::LinkDown (single source, no drift).
    fd_cfg.link_fail_threshold = static_cast<int>(link_fail_threshold_);
    fd_cfg.link_fail_threshold_sensor = static_cast<int>(link_fail_threshold_sensor_);
    // Split startup grace (#2): rate warmup (long) vs link grace (short). The
    // rate grace suppresses the polling-rate E-STOP while the loop reaches steady
    // cadence; the link grace only spans the ARP/firmware-boot transient, so a
    // dead link latches well below the CM device_timeout (1000 ms) instead of
    // waiting out the full rate warmup.
    fd_cfg.startup_grace_ms = get_parameter("startup_grace_ms").as_double();
    fd_cfg.link_startup_grace_ms = get_parameter("link_startup_grace_ms").as_double();
    fd_cfg.sensor_startup_grace_ms = get_parameter("sensor_startup_grace_ms").as_double();

    // The failure detector is an aux thread of the *hand driver* process, which
    // the launch taskset-pins to its own core (logical 11 on NUC13, in the cset
    // "system" cpuset). Reuse nrt_logging's SCHED_OTHER policy / nice, but drop
    // the CPU pin (cpu_core = -1): the dedicated nrt core (slot 8 -> logical 12)
    // is owned by the CM's cset "user" cpuset (issue #151), so pinning this
    // separate process's thread there EINVALs under an active shield. Inheriting
    // the hand process affinity keeps the detector beside the driver it monitors.
    auto cfgs = rtc::SelectThreadConfigs();
    auto fd_thread_cfg = cfgs.nrt_logging;
    fd_thread_cfg.cpu_core = -1;
    failure_detector_ = std::make_unique<udp_hand_driver::UdpHandFailureDetector>(
        *controller_, fd_cfg, fd_thread_cfg);
    failure_detector_->SetFailureCallback([this](const std::string& reason) {
      RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(), "Hand failure detected: %s",
                   reason.c_str());
      // Persist the forensic counters at failure time — the process may be
      // torn down (E-STOP) before any graceful-teardown save runs.
      SaveCommStats();
    });
    failure_detector_->Start();
    RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
                "UdpHandFailureDetector started (50 Hz, threshold=%d)", fd_cfg.failure_threshold);
  }

  // ── Periodic stats save (crash-safety) ─────────────────────────────
  // Fixed 10 s interval (constant, not a ROS param): keeps the stats JSON at
  // most 10 s stale after SIGKILL / power loss. Quiet (verbose=false).
  {
    constexpr auto kStatsSavePeriod = std::chrono::seconds(10);
    // cb_group_aux_: blocking JSON write. Off the default executor thread, which
    // also serves the joint-command subscription (issue #345).
    stats_save_timer_ =
        create_wall_timer(kStatsSavePeriod, [this]() { SaveCommStats(false); }, cb_group_aux_);
  }

  // ── Per-tick timing CSV: open once, (re)start the drain timer ──────
  // Two lifetimes, deliberately separated (issue #345). The logger opens on the
  // first activation only. The drain timer is recreated on *every* activation
  // because StopRuntime() now cancels it to quiesce the aux lane — keeping both
  // on one flag would either skip on_deactivate's straggler drain (if the flag
  // were cleared) or leave the timer dead after deactivate -> activate (if not).
  if (!hand_udp_timing_opened_) {
    if (!hand_udp_timing_logger_.Open()) {
      RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                  "UdpHandTimingLogger::Open() failed — timing CSV disabled");
    } else {
      RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "Hand UDP tick-timing CSV: %s",
                  hand_udp_timing_logger_.Path().c_str());
    }
    hand_udp_timing_opened_ = true;
  }
  // cb_group_aux_: this is the heaviest callback in the process — one 1 Hz
  // burst writes up to kHandUdpTimingBufferCapacity (512) CSV rows, two orders
  // of magnitude more per invocation than CM's equivalent drain, which its own
  // nrt_logging thread runs at 100 Hz (rt_tick_timing_sample.hpp) (issue #345).
  hand_udp_timing_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this]() { DrainHandUdpTiming(); }, cb_group_aux_);

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode activated");
  return CallbackReturn::SUCCESS;
}

void UdpHandNode::DrainHandUdpTiming() noexcept {
  // Two callers now: the 1 Hz aux timer (hand_aux_io thread) and on_deactivate's
  // straggler drain (executor thread). hand_udp_timing_producer_ is SPSC — a
  // *single* consumer — so the two must never overlap. StopRuntime() cancels the
  // timer and waits out any in-flight callback before on_deactivate drains, and
  // this lock enforces the invariant rather than relying on that ordering (#345).
  std::lock_guard lock(aux_lane_mutex_);
  if (!controller_)
    return;
  hand_udp_timing_producer_.Drain(
      [this](const rtc::RtTickTimingSample& s) { hand_udp_timing_logger_.Log(s); });
  const std::uint64_t drops = hand_udp_timing_producer_.DropCount();
  if (drops > hand_udp_timing_drop_baseline_) {
    const std::uint64_t delta = drops - hand_udp_timing_drop_baseline_;
    hand_udp_timing_drop_baseline_ = drops;
    RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                "Hand UDP timing SPSC dropped %lu samples since last drain",
                static_cast<unsigned long>(delta));
  }
}

UdpHandNode::CallbackReturn UdpHandNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  StopRuntime();
  if (controller_) {
    // Drop the producer pointer so a stopped EventLoop cannot push into a
    // potentially-recreated buffer on a future activation.
    controller_->SetTimingProducer(nullptr);
  }
  // Drain any stragglers so the CSV reflects everything pushed before stop. Safe
  // as the sole consumer here: StopRuntime() above cancelled the aux timer and
  // waited out any in-flight aux callback (issue #345).
  if (hand_udp_timing_opened_) {
    DrainHandUdpTiming();
  }

  // Parent call deactivates LifecyclePublishers.
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode deactivated");
  return CallbackReturn::SUCCESS;
}

UdpHandNode::CallbackReturn UdpHandNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
  SaveCommStats();
  ReleaseResources();

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode cleaned up");
  return CallbackReturn::SUCCESS;
}

UdpHandNode::CallbackReturn UdpHandNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
  }
  return on_cleanup(state);
}

UdpHandNode::CallbackReturn UdpHandNode::on_error(const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_ERROR(::udp_hand_driver::logging::NodeLogger(), "UdpHandNode error — attempting recovery");
  // Preserve the order: cancel timers → stop threads → save stats → release.
  StopRuntime();
  SaveCommStats();
  ReleaseResources();

  // SUCCESS -> Unconfigured (recoverable), not Finalized.
  return CallbackReturn::SUCCESS;
}

// ── Teardown helpers (shared by on_deactivate / on_cleanup / on_error) ───────

void UdpHandNode::StopRuntime() {
  // Quiesce the aux lane first. cancel() guarantees no *further* invocation, but
  // says nothing about one already running on the hand_aux_io thread, so take
  // aux_lane_mutex_ empty as a barrier once no new callback can start. The
  // detector is stopped before the barrier because its failure callback is the
  // third SaveCommStats caller — joining it first makes the barrier conclusive
  // rather than a snapshot (issue #345).
  if (stats_save_timer_) {
    stats_save_timer_->cancel();
    stats_save_timer_.reset();
  }
  // Timer only — `hand_udp_timing_opened_` stays true so a later on_activate
  // reuses the open CSV instead of re-running Open().
  if (hand_udp_timing_timer_) {
    hand_udp_timing_timer_->cancel();
    hand_udp_timing_timer_.reset();
  }
  if (failure_detector_) {
    failure_detector_->Stop();
    failure_detector_.reset();
  }
  { std::lock_guard<std::mutex> quiesce(aux_lane_mutex_); }
  // Call Stop() unconditionally (it is idempotent). The IsRunning() guard was
  // removed because the E-Stop self-exit path now clears running_ from inside
  // the CommLoop (#4), so a guarded Stop() would skip the required Join() +
  // socket close + stats log after an E-Stop.
  if (controller_) {
    controller_->Stop();
  }
}

void UdpHandNode::ReleaseResources() {
  // Aux-lane timers first, for the same reason StopRuntime() cancels them: their
  // callbacks run on the hand_aux_io thread and touch controller_, which this
  // function resets below (issue #345). on_cleanup reaches here without
  // StopRuntime(), so the reset cannot rely on that path alone.
  stats_save_timer_.reset();
  hand_udp_timing_timer_.reset();
  calib_status_timer_.reset();
  calib_cmd_sub_.reset();
  joint_command_sub_.reset();
  link_status_pub_.reset();
  sensor_monitor_pub_.reset();
  sensor_state_pub_.reset();
  motor_state_pub_.reset();
  joint_state_pub_.reset();
  calib_status_pub_.reset();
  controller_.reset();
}
