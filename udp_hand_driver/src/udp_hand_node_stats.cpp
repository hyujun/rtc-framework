// UdpHandNode::SaveCommStats — persist comm/timing statistics to
// <session>/device/hand_stats<suffix>.json, one file per hand (suffix =
// "_<side>" under hand_sides, or when hand_side was set explicitly, D51 · D71). Split out of
// udp_hand_node_lifecycle.cpp so the ~150-line manual JSON writer does not crowd the lifecycle
// callbacks. See udp_hand_node.hpp for the declaration and threading contract.
//
// Key set: test/fixtures/hand_stats_keys.txt (proto1, individual; a
// superset gate) and hand_stats_keys_proto2.txt (proto2, bulk; exact) —
// append, never rename, and regenerate the fixture in the same commit.

#include "udp_hand_driver/canfd_hand_comm.hpp"
#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"
#include <rtc_base/logging/session_dir.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <system_error>

void UdpHandNode::SaveCommStats(bool verbose) const {
  // Serialize concurrent saves (aux-lane timer / failure callback / teardown) so
  // two writers cannot interleave into a corrupt JSON.
  //
  // The hands are read INSIDE the lock on purpose: the periodic save runs on
  // the hand_aux_io thread, so a check outside would race with
  // ReleaseResources() clearing units_ on the executor thread — read the guard
  // and dereference under the same critical section that teardown's quiesce
  // barrier waits on (issue #345).
  std::lock_guard lock(aux_lane_mutex_);
  // No run since the last configure: nothing to describe, and writing would
  // overwrite the previous run's JSON with an empty one (#927).
  if (!has_run_)
    return;
  for (const auto& unit : units_) {
    SaveHandStatsLocked(*unit, verbose);
  }
}

void UdpHandNode::SaveCommStats(const HandUnit& unit, bool verbose) const {
  // One hand only — its failure detector's callback (D74). Same lock and the
  // same run guard as the all-hands save above; `unit` outlives the detector
  // that calls this (StopRuntime() joins the detectors before units_ goes).
  std::lock_guard lock(aux_lane_mutex_);
  if (!has_run_)
    return;
  SaveHandStatsLocked(unit, verbose);
}

void UdpHandNode::SaveHandStatsLocked(const HandUnit& unit, bool verbose) const {
  if (!unit.controller)
    return;
  const auto& controller = *unit.controller;
  const auto stats = controller.comm_stats();
  // A torn-down detector leaves its verdict in failure_latched (#927 I-1).
  const bool fd_failed =
      unit.failure_detector ? unit.failure_detector->failed() : unit.failure_latched;

  // CAN FD-only counters, one snapshot for the WARN below and the JSON. A
  // cmd_ack the firmware disagrees with (#945 D63) fails no stage outright, so
  // say it once per kind per run — before the file is opened, so an I/O
  // failure does not hide it; the mode is never switched here.
  udp_hand_driver::CanFdCommStats cs{};
  udp_hand_driver::canfd::CmdAck cmd_ack = udp_hand_driver::canfd::CmdAck::kEcho;
  if (unit.canfd_comm != nullptr) {
    cs = unit.canfd_comm->canfd_stats();
    cmd_ack = unit.canfd_comm->config().cmd_ack;
    const auto& write_echo =
        stats.per_kind[static_cast<std::size_t>(udp_hand_driver::RequestKind::kWriteEcho)];
    const auto& joint_read =
        stats.per_kind[static_cast<std::size_t>(udp_hand_driver::RequestKind::kJointRead)];
    switch (udp_hand_driver::CmdAckMismatchDecision(cmd_ack, write_echo, joint_read.ok,
                                                    cs.echo_unexpected)) {
      case udp_hand_driver::CmdAckMismatch::kFirmwareSilent:
        if (!unit.warned_cmd_ack_silent) {
          unit.warned_cmd_ack_silent = true;
          RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                      "%scmd_ack echo, but %llu writes timed out on the echo and none saw it while "
                      "RESP sets complete: the firmware does not echo CMD frames — set cmd_ack "
                      "none",
                      unit.log_tag.c_str(), static_cast<unsigned long long>(write_echo.timeout));
        }
        break;
      case udp_hand_driver::CmdAckMismatch::kFirmwareEchoes:
        if (!unit.warned_cmd_ack_echoes) {
          unit.warned_cmd_ack_echoes = true;
          RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                      "%scmd_ack none, but %llu CMD echoes arrived (canfd.echo_unexpected): the "
                      "firmware echoes CMD frames — set cmd_ack echo",
                      unit.log_tag.c_str(), static_cast<unsigned long long>(cs.echo_unexpected));
        }
        break;
      case udp_hand_driver::CmdAckMismatch::kNone:
        break;
    }
  }

  // The controller's run length: frozen at the last completed cycle once the
  // loop has stopped, so an E-Stop's idle tail is not counted (#927 I-2).
  const double elapsed_sec = controller.ElapsedSeconds();
  const double avg_rate_hz =
      (elapsed_sec > 0.0) ? static_cast<double>(stats.total_cycles) / elapsed_sec : 0.0;

  const std::filesystem::path session = rtc::ResolveSessionDir();
  std::filesystem::path output_dir_path = session / "device";
  std::error_code dir_ec;
  std::filesystem::create_directories(output_dir_path, dir_ec);
  const std::string output_dir = output_dir_path.string();

  const std::string path = output_dir + "/hand_stats" + unit.file_suffix + ".json";
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(), "%sFailed to save hand stats to %s",
                unit.log_tag.c_str(), path.c_str());
    return;
  }

  const auto ts = controller.timing_stats();

  const bool is_bulk =
      (controller.communication_mode() == udp_hand_driver::HandCommunicationMode::kBulk);
  const char* mode_str = is_bulk ? "bulk" : "individual";

  // transport is the type of the installed link, not the protocol string
  // (loopmodel "2" reads canfd too: the comm was constructed even though its
  // socket never opened — sil_mode is not recorded here).
  const char* transport_str = (unit.canfd_comm != nullptr) ? "canfd" : "udp";

  ofs << "{\n"
      << "  \"comm_stats\": {\n"
      << "    \"communication_mode\": \"" << mode_str << "\",\n"
      << "    \"protocol_version\": \"" << protocol_version_ << "\",\n"
      << "    \"transport\": \"" << transport_str
      << "\",\n"
      // On CAN FD this is the per-stage deadline (D42); the key is shared.
      << "    \"recv_timeout_ms\": " << std::fixed << std::setprecision(3)
      << controller.recv_timeout_ms() << ",\n"
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
      << "    \"comm_decimation\": " << controller.comm_decimation() << ",\n"
      << "    \"comm_decimation_skip_count\": " << stats.comm_decimation_skip_count
      << ",\n"
      // Datagrams dropped by the cycle-start drain (desync backlog). A rising
      // count with 0.4 ms timeout means the drain is actively unmasking stale
      // reads — see Phase 2 recv-stabilization notes.
      << "    \"stale_drained\": " << stats.stale_drained
      << ",\n"
      // D39: JointCommand messages whose feedforward was zeroed for a size
      // mismatch (position still applied). Every protocol, so a proto1 run
      // reads 0 rather than lacking the key.
      << "    \"feedforward_size_mismatch\": "
      << unit.feedforward_size_mismatch.load(std::memory_order_relaxed) << ",\n"
      << "    \"feedforward_nonfinite\": "
      << unit.feedforward_nonfinite.load(std::memory_order_relaxed) << ",\n";

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
  ofs << "    },\n";

  // CAN FD-only counters (tx errno classes, error frames, echo payload
  // mismatches and unexpected echoes, RESP-set assembler) — the link's own
  // SeqLock snapshot, taken above — and the configured cmd_ack (#945). Absent
  // on UDP so the
  // proto1 key set is exactly what it was.
  if (unit.canfd_comm != nullptr) {
    ofs << "    \"canfd\": {\n"
        << "      \"cmd_ack\": \"" << udp_hand_driver::canfd::CmdAckName(cmd_ack) << "\",\n"
        << "      \"tx_eagain\": " << cs.tx_eagain << ",\n"
        << "      \"tx_enobufs\": " << cs.tx_enobufs << ",\n"
        << "      \"tx_enetdown\": " << cs.tx_enetdown << ",\n"
        << "      \"tx_other\": " << cs.tx_other << ",\n"
        << "      \"error_frames\": " << cs.error_frames << ",\n"
        << "      \"bus_off\": " << cs.bus_off << ",\n"
        << "      \"echo_payload_mismatch\": " << cs.echo_payload_mismatch << ",\n"
        << "      \"echo_unexpected\": " << cs.echo_unexpected << ",\n"
        << "      \"assembler\": { \"sets_completed\": " << cs.assembler.sets_completed
        << ", \"sets_abandoned\": " << cs.assembler.sets_abandoned
        << ", \"duplicate\": " << cs.assembler.duplicate
        << ", \"outside_set\": " << cs.assembler.outside_set
        << ", \"bad_index\": " << cs.assembler.bad_index
        << ", \"bad_length\": " << cs.assembler.bad_length << " }\n"
        << "    },\n";
  }

  ofs << "    \"last_unexpected_cmd\": " << static_cast<unsigned>(stats.last_unexpected_cmd)
      << ",\n"
      << "    \"last_unexpected_len\": " << static_cast<unsigned>(stats.last_unexpected_len)
      << ",\n"
      << "    \"avg_rate_hz\": " << std::fixed << std::setprecision(2) << avg_rate_hz << ",\n"
      << "    \"elapsed_sec\": " << std::fixed << std::setprecision(2) << elapsed_sec << ",\n"
      << "    \"failure_detected\": " << (fd_failed ? "true" : "false") << ",\n"
      << "    \"consecutive_recv_failures\": " << controller.consecutive_recv_failures()
      << ",\n"
      // Strict per-channel decision (#1), same source as the detector E-STOP.
      << "    \"link_ok\": "
      << (!controller.LinkDown(static_cast<uint32_t>(link_fail_threshold_),
                               static_cast<uint32_t>(link_fail_threshold_sensor_))
              ? "true"
              : "false")
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
      << controller.actual_sensor_rate_hz() << ",\n"
      << "    \"over_budget\": " << ts.over_budget << "\n"
      << "  }\n"
      << "}\n";
  ofs.close();

  // Periodic (verbose=false) saves stay quiet — a 10 s INFO cadence would
  // drown the log while adding nothing over the final teardown summary.
  if (!verbose)
    return;

  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "%s%s", unit.log_tag.c_str(),
              controller.TimingSummary().c_str());

  const double total = static_cast<double>(stats.total_cycles > 0 ? stats.total_cycles : 1);
  const double ok_pct = 100.0 * static_cast<double>(stats.recv_ok) / total;
  const double timeout_pct = 100.0 * static_cast<double>(stats.recv_timeout) / total;
  const double error_pct = 100.0 * static_cast<double>(stats.recv_error) / total;
  RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(),
              "%sHand stats saved: %s | %lu cycles in %.1fs "
              "(avg=%.1fHz ok=%.1f%% timeout=%.1f%% err=%.1f%% fd_failed=%d)",
              unit.log_tag.c_str(), path.c_str(), static_cast<unsigned long>(stats.total_cycles),
              elapsed_sec, avg_rate_hz, ok_pct, timeout_pct, error_pct, fd_failed ? 1 : 0);
}
