#ifndef UDP_HAND_DRIVER_UDP_HAND_TIMING_LOGGER_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_TIMING_LOGGER_HPP_

// Per-EventLoop-tick timing CSV writer for the hand UDP node.
//
// Mirrors rtc::mpc::MpcTimingLogger structurally: a thin wrapper around
// rtc::ThreadTimingCsvLogger<rtc::RtTickTimingPayload> that resolves the canonical
// `<session>/timing/hand_udp_timing_log.csv` path and pre-binds the unified
// rtc::RtTickTimingPayload header / row writers (the same schema CM and MPC emit,
// so analysis scripts can join across threads).
//
// Phase mapping for the hand UDP loop (see hand_controller.hpp::EventLoop):
//   t_state_us    UDP write + read phase (post-condvar to post-sensor-read)
//   t_compute_us  sensor processing + F/T inference
//   t_publish_us  state rtc::SeqLock store + EventLoop callback
//   t_total_us    end-of-tick − start-of-tick
//   jitter_us     |actual_period − expected_period| against previous tick

#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_base/timing/thread_timing_csv_logger.hpp"

#include <filesystem>
#include <system_error>

namespace udp_hand_driver {

class UdpHandTimingLogger {
 public:
  UdpHandTimingLogger() = default;
  ~UdpHandTimingLogger() = default;

  UdpHandTimingLogger(const UdpHandTimingLogger&) = delete;
  UdpHandTimingLogger& operator=(const UdpHandTimingLogger&) = delete;

  /// Resolve the CSV path under `<session>/timing/`, create the parent
  /// directory, and open the file. Schema header is written on first
  /// creation; existing files are appended without a duplicate header.
  /// Returns false on filesystem errors.
  [[nodiscard]] bool Open() noexcept {
    try {
      const auto session = rtc::ResolveSessionDir();
      const auto timing_dir = rtc::TimingDir(session);
      std::error_code ec;
      std::filesystem::create_directories(timing_dir, ec);
      const auto path = timing_dir / "hand_udp_timing_log.csv";
      return inner_.Open(path, &rtc::WriteRtTickTimingHeader, &rtc::WriteRtTickTimingRow);
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool IsOpen() const noexcept { return inner_.IsOpen(); }

  [[nodiscard]] const std::filesystem::path& Path() const noexcept { return inner_.Path(); }

  /// Append one row for a single hand UDP tick sample. No-op if not open.
  void Log(const rtc::RtTickTimingSample& s) noexcept { inner_.Log(s); }

 private:
  rtc::ThreadTimingCsvLogger<rtc::RtTickTimingPayload> inner_;
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_UDP_HAND_TIMING_LOGGER_HPP_
