#ifndef UR5E_HAND_DRIVER_HAND_UDP_TIMING_LOGGER_HPP_
#define UR5E_HAND_DRIVER_HAND_UDP_TIMING_LOGGER_HPP_

// Per-EventLoop-tick timing CSV writer for the hand UDP node.
//
// Mirrors rtc::mpc::MpcTimingLogger structurally: a thin wrapper around
// rtc::ThreadTimingCsvLogger<RtTickTimingPayload> that resolves the canonical
// `<session>/timing/hand_udp_timing_log.csv` path and pre-binds the unified
// RtTickTimingPayload header / row writers (the same schema CM and MPC emit,
// so analysis scripts can join across threads).
//
// Phase mapping for the hand UDP loop (see hand_controller.hpp::EventLoop):
//   t_state_us    UDP write + read phase (post-condvar to post-sensor-read)
//   t_compute_us  sensor processing + F/T inference
//   t_publish_us  state SeqLock store + EventLoop callback
//   t_total_us    end-of-tick − start-of-tick
//   jitter_us     |actual_period − expected_period| against previous tick

#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_base/timing/thread_timing_csv_logger.hpp"

#include <filesystem>
#include <system_error>

namespace rtc::hand {

class HandUdpTimingLogger {
public:
  HandUdpTimingLogger() = default;
  ~HandUdpTimingLogger() = default;

  HandUdpTimingLogger(const HandUdpTimingLogger &) = delete;
  HandUdpTimingLogger &operator=(const HandUdpTimingLogger &) = delete;

  /// Resolve the CSV path under `<session>/timing/`, create the parent
  /// directory, and open the file. Schema header is written on first
  /// creation; existing files are appended without a duplicate header.
  /// Returns false on filesystem errors.
  [[nodiscard]] bool Open() noexcept {
    try {
      const auto session = ResolveSessionDir();
      const auto timing_dir = TimingDir(session);
      std::error_code ec;
      std::filesystem::create_directories(timing_dir, ec);
      const auto path = timing_dir / "hand_udp_timing_log.csv";
      return inner_.Open(path, &rtc::WriteRtTickTimingHeader,
                         &rtc::WriteRtTickTimingRow);
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool IsOpen() const noexcept { return inner_.IsOpen(); }

  [[nodiscard]] const std::filesystem::path &Path() const noexcept {
    return inner_.Path();
  }

  /// Append one row for a single hand UDP tick sample. No-op if not open.
  void Log(const rtc::RtTickTimingSample &s) noexcept { inner_.Log(s); }

private:
  rtc::ThreadTimingCsvLogger<rtc::RtTickTimingPayload> inner_;
};

} // namespace rtc::hand

#endif // UR5E_HAND_DRIVER_HAND_UDP_TIMING_LOGGER_HPP_
