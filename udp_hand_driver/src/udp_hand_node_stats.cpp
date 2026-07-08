// UdpHandNode::SaveCommStats — persist comm/timing statistics to
// <session>/device/hand_udp_stats.json. Split out of udp_hand_node_lifecycle.cpp
// so the ~150-line manual JSON writer does not crowd the lifecycle callbacks.
// See udp_hand_node.hpp for the declaration and threading contract.

#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"
#include <rtc_base/logging/session_dir.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <system_error>

void UdpHandNode::SaveCommStats(bool verbose) const {
  if (!controller_)
    return;

  // Serialize concurrent saves (periodic timer / failure callback / teardown)
  // so two writers cannot interleave into a corrupt JSON.
  std::lock_guard lock(save_stats_mutex_);

  const auto stats = controller_->comm_stats();
  const bool fd_failed = failure_detector_ ? failure_detector_->failed() : false;

  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  const double elapsed_sec = std::chrono::duration<double>(elapsed).count();
  const double avg_rate_hz =
      (elapsed_sec > 0.0) ? static_cast<double>(stats.total_cycles) / elapsed_sec : 0.0;

  const std::filesystem::path session = rtc::ResolveSessionDir();
  std::filesystem::path output_dir_path = session / "device";
  std::error_code dir_ec;
  std::filesystem::create_directories(output_dir_path, dir_ec);
  const std::string output_dir = output_dir_path.string();

  const std::string path = output_dir + "/hand_udp_stats.json";
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(), "Failed to save hand stats to %s",
                path.c_str());
    return;
  }

  const auto ts = controller_->timing_stats();

  const bool is_bulk =
      (controller_->communication_mode() == udp_hand_driver::HandCommunicationMode::kBulk);
  const char* mode_str = is_bulk ? "bulk" : "individual";

  ofs << "{\n"
      << "  \"comm_stats\": {\n"
      << "    \"communication_mode\": \"" << mode_str << "\",\n"
      << "    \"recv_timeout_ms\": " << std::fixed << std::setprecision(3)
      << controller_->recv_timeout_ms() << ",\n"
      << "    \"total_cycles\": " << stats.total_cycles << ",\n"
      << "    \"recv_ok\": " << stats.recv_ok << ",\n"
      << "    \"recv_timeout\": " << stats.recv_timeout << ",\n"
      << "    \"recv_error\": " << stats.recv_error
      << ",\n"
      // cmd_mismatch / mode_mismatch surface the two read-drop causes: a wrong
      // CMD echo, and a MODE echo that fails the verify gate. A rising
      // mode_mismatch with a lenient protocol (1b) means the gate would have
      // dropped valid frames — the InitPositionHold-to-zero regression signature.
      << "    \"cmd_mismatch\": " << stats.cmd_mismatch << ",\n"
      << "    \"mode_mismatch\": " << stats.mode_mismatch << ",\n"
      << "    \"comm_decimation\": " << controller_->comm_decimation() << ",\n"
      << "    \"comm_decimation_skip_count\": " << stats.comm_decimation_skip_count << ",\n";

  // Per-request-kind breakdown (link-down forensics): which request kind eats
  // the failures, and whether drops are timeouts or stale/desync mismatches.
  ofs << "    \"per_request\": {\n";
  for (int k = 0; k < udp_hand_driver::kNumRequestKinds; ++k) {
    const auto& pk = stats.per_kind[static_cast<std::size_t>(k)];
    ofs << "      \"" << udp_hand_driver::kRequestKindNames[static_cast<std::size_t>(k)]
        << "\": { \"ok\": " << pk.ok << ", \"timeout\": " << pk.timeout
        << ", \"error\": " << pk.error << ", \"cmd_mismatch\": " << pk.cmd_mismatch
        << ", \"mode_mismatch\": " << pk.mode_mismatch
        << ", \"short_or_decode\": " << pk.short_or_decode << " }"
        << (k + 1 < udp_hand_driver::kNumRequestKinds ? "," : "") << "\n";
  }
  ofs << "    },\n"
      << "    \"last_unexpected_cmd\": " << static_cast<unsigned>(stats.last_unexpected_cmd)
      << ",\n"
      << "    \"last_unexpected_len\": " << static_cast<unsigned>(stats.last_unexpected_len)
      << ",\n"
      << "    \"avg_rate_hz\": " << std::fixed << std::setprecision(2) << avg_rate_hz << ",\n"
      << "    \"elapsed_sec\": " << std::fixed << std::setprecision(2) << elapsed_sec << ",\n"
      << "    \"failure_detected\": " << (fd_failed ? "true" : "false") << ",\n"
      << "    \"consecutive_recv_failures\": " << controller_->consecutive_recv_failures() << ",\n"
      << "    \"link_ok\": "
      << (controller_->consecutive_recv_failures() < link_fail_threshold_ ? "true" : "false")
      << "\n"
      << "  },\n"
      << "  \"timing_stats\": {\n"
      << "    \"count\": " << ts.count << ",\n"
      << "    \"total_us\": {" << " \"mean\": " << std::setprecision(1) << ts.mean_us
      << ", \"min\": " << ts.min_us << ", \"max\": " << ts.max_us
      << ", \"stddev\": " << ts.stddev_us << ", \"p95\": " << ts.p95_us
      << ", \"p99\": " << ts.p99_us << " },\n"
      << "    \"write_us\": {" << " \"mean\": " << ts.write.mean_us
      << ", \"min\": " << ts.write.min_us << ", \"max\": " << ts.write.max_us << " },\n";

  if (is_bulk) {
    ofs << "    \"read_all_motor_us\": {" << " \"mean\": " << ts.read_all_motor.mean_us
        << ", \"min\": " << ts.read_all_motor.min_us << ", \"max\": " << ts.read_all_motor.max_us
        << " },\n"
        << "    \"read_all_joint_motor_us\": {" << " \"mean\": " << ts.read_all_joint_motor.mean_us
        << ", \"min\": " << ts.read_all_joint_motor.min_us
        << ", \"max\": " << ts.read_all_joint_motor.max_us << " },\n"
        << "    \"read_all_sensor_us\": {" << " \"mean\": " << ts.read_all_sensor.mean_us
        << ", \"min\": " << ts.read_all_sensor.min_us << ", \"max\": " << ts.read_all_sensor.max_us
        << ", \"sensor_cycles\": " << ts.sensor_cycle_count << " },\n";
  } else {
    ofs << "    \"read_pos_us\": {" << " \"mean\": " << ts.read_pos.mean_us
        << ", \"min\": " << ts.read_pos.min_us << ", \"max\": " << ts.read_pos.max_us << " },\n"
        << "    \"read_joint_pos_us\": {" << " \"mean\": " << ts.read_joint_pos.mean_us
        << ", \"min\": " << ts.read_joint_pos.min_us << ", \"max\": " << ts.read_joint_pos.max_us
        << " },\n"
        << "    \"read_vel_us\": {" << " \"mean\": " << ts.read_vel.mean_us
        << ", \"min\": " << ts.read_vel.min_us << ", \"max\": " << ts.read_vel.max_us << " },\n"
        << "    \"read_sensor_us\": {" << " \"mean\": " << ts.read_sensor.mean_us
        << ", \"min\": " << ts.read_sensor.min_us << ", \"max\": " << ts.read_sensor.max_us
        << ", \"sensor_cycles\": " << ts.sensor_cycle_count << " },\n";
  }

  if (ts.sensor_cycle_count > 0) {
    ofs << "    \"sensor_proc_us\": {" << " \"mean\": " << ts.sensor_proc.mean_us
        << ", \"min\": " << ts.sensor_proc.min_us << ", \"max\": " << ts.sensor_proc.max_us
        << " },\n";
  }

  if (ts.ft_infer_count > 0) {
    ofs << "    \"ft_infer_us\": {" << " \"mean\": " << ts.ft_infer.mean_us
        << ", \"min\": " << ts.ft_infer.min_us << ", \"max\": " << ts.ft_infer.max_us
        << ", \"count\": " << ts.ft_infer_count << " },\n";
  }

  ofs << "    \"actual_sensor_rate_hz\": " << std::setprecision(1)
      << controller_->actual_sensor_rate_hz() << ",\n"
      << "    \"over_budget\": " << ts.over_budget << "\n"
      << "  }\n"
      << "}\n";
  ofs.close();

  // Periodic (verbose=false) saves stay quiet — a 10 s INFO cadence would
  // drown the log while adding nothing over the final teardown summary.
  if (!verbose)
    return;

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "%s", controller_->TimingSummary().c_str());

  const double total = static_cast<double>(stats.total_cycles > 0 ? stats.total_cycles : 1);
  const double ok_pct = 100.0 * static_cast<double>(stats.recv_ok) / total;
  const double timeout_pct = 100.0 * static_cast<double>(stats.recv_timeout) / total;
  const double error_pct = 100.0 * static_cast<double>(stats.recv_error) / total;
  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
              "Hand stats saved: %s | %lu cycles in %.1fs "
              "(avg=%.1fHz ok=%.1f%% timeout=%.1f%% err=%.1f%% fd_failed=%d)",
              path.c_str(), static_cast<unsigned long>(stats.total_cycles), elapsed_sec,
              avg_rate_hz, ok_pct, timeout_pct, error_pct, fd_failed ? 1 : 0);
}
