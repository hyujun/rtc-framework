#ifndef RTC_MPC_LOGGING_MPC_TIMING_LOGGER_HPP_
#define RTC_MPC_LOGGING_MPC_TIMING_LOGGER_HPP_

// Per-MPC-tick timing CSV writer.
//
// The transport (SPSC ring) and the CSV header / row formatting shell are
// generic — see `rtc_base/timing/thread_timing_*` and
// `rtc_base/timing/rt_tick_timing_sample.hpp`. This header is a thin wrapper
// that resolves the canonical
// `<session>/controllers/<config_key>/mpc_timing_log.csv` path and pre-binds
// the unified RtTickTimingPayload header / row writers (the same schema the
// CM RT loop emits, so analysis scripts can join across threads).
//
// CSV schema:
//   t_wall_ns,tick_count,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us
// One row per MPC main-loop iteration. The first two columns come from
// `ThreadTimingCsvLogger`, the next five from `WriteRtTickTimingRow`.

#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"
#include "rtc_base/timing/thread_timing_csv_logger.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

namespace rtc::mpc {

class MpcTimingLogger {
public:
  MpcTimingLogger() = default;
  ~MpcTimingLogger() = default;

  MpcTimingLogger(const MpcTimingLogger &) = delete;
  MpcTimingLogger &operator=(const MpcTimingLogger &) = delete;

  /// Resolve the CSV path under `<session>/controllers/<config_key>/`,
  /// create the parent directory, and open the file. Schema header is
  /// written on first creation; existing files are appended without a
  /// duplicate header. Returns false on filesystem errors.
  [[nodiscard]] bool Open(std::string_view config_key) noexcept {
    try {
      const auto session = ResolveSessionDir();
      const auto controller_dir =
          session / "controllers" / std::string(config_key);
      std::error_code ec;
      std::filesystem::create_directories(controller_dir, ec);
      const auto path = controller_dir / "mpc_timing_log.csv";
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

  /// Append one row for a single MPC tick sample. No-op if not open.
  void Log(const rtc::RtTickTimingSample &s) noexcept { inner_.Log(s); }

private:
  rtc::ThreadTimingCsvLogger<rtc::RtTickTimingPayload> inner_;
};

} // namespace rtc::mpc

#endif // RTC_MPC_LOGGING_MPC_TIMING_LOGGER_HPP_
