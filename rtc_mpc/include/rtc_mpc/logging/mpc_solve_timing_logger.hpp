#ifndef RTC_MPC_LOGGING_MPC_SOLVE_TIMING_LOGGER_HPP_
#define RTC_MPC_LOGGING_MPC_SOLVE_TIMING_LOGGER_HPP_

// Per-MPC-tick timing CSV writer.
//
// The transport (SPSC ring) and the CSV header / row formatting shell are
// now generic — see `rtc_base/timing/thread_timing_*`. This header provides:
//   * `MpcSolveTimingLogger`: a thin wrapper around
//     `rtc::ThreadTimingCsvLogger<rtc::MpcTimingPayload>` that resolves the
//     canonical `<session>/controllers/<config_key>/mpc_solve_timing.csv`
//     path and pre-binds the MpcTimingPayload header / row writers.
//   * `MpcSolveSampleBuffer` (re-exported from rtc_base): the per-tick
//     producer the MPC manager owns.
//
// CSV schema:
//   t_wall_ns,tick_count,solve_ns
// One row per MPC solve. `tick_count` is the producer's monotonic
// sequence number (post-increment); the previous schema named this
// column `count`, identical semantics — renamed for cross-thread
// consistency with the CM RT loop CSV.

#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/timing/mpc_solve_sample.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

namespace rtc::mpc
{

class MpcSolveTimingLogger {
public:
  MpcSolveTimingLogger() = default;
  ~MpcSolveTimingLogger() = default;

  MpcSolveTimingLogger(const MpcSolveTimingLogger &) = delete;
  MpcSolveTimingLogger & operator=(const MpcSolveTimingLogger &) = delete;

  /// Resolve the CSV path under `<session>/controllers/<config_key>/`,
  /// create the parent directory, and open the file. The schema header is
  /// written on first creation; existing files are appended without a
  /// duplicate header. Returns false on filesystem errors.
  [[nodiscard]] bool Open(std::string_view config_key) noexcept
  {
    try {
      const auto session = ResolveSessionDir();
      const auto controller_dir =
        session / "controllers" / std::string(config_key);
      std::error_code ec;
      std::filesystem::create_directories(controller_dir, ec);
      const auto path = controller_dir / "mpc_solve_timing.csv";
      return inner_.Open(path, &rtc::WriteMpcTimingHeader,
                         &rtc::WriteMpcTimingRow);
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool IsOpen() const noexcept {return inner_.IsOpen();}

  [[nodiscard]] const std::filesystem::path & Path() const noexcept
  {
    return inner_.Path();
  }

  /// Append one row for a single MPC solve sample. No-op if not open.
  void Log(const rtc::MpcSolveSample & s) noexcept {inner_.Log(s);}

private:
  rtc::ThreadTimingCsvLogger<rtc::MpcTimingPayload> inner_;
};

} // namespace rtc::mpc

#endif // RTC_MPC_LOGGING_MPC_SOLVE_TIMING_LOGGER_HPP_
