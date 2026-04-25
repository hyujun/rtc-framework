#ifndef RTC_MPC_LOGGING_MPC_SOLVE_TIMING_LOGGER_HPP_
#define RTC_MPC_LOGGING_MPC_SOLVE_TIMING_LOGGER_HPP_

#include "rtc_base/logging/session_dir.hpp"
#include "rtc_base/timing/mpc_solve_stats.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>

// Periodic CSV appender for MPC solve-timing windows. Writes one row per
// snapshot to `<session>/controllers/<config_key>/mpc_solve_timing.csv` with
// columns:
//   t_wall_ns,count,window,last_ns,min_ns,p50_ns,p99_ns,max_ns,mean_ns
//
// The caller is expected to invoke `Log()` from a non-RT callback (typ.
// 1 Hz aux executor of the controller's LifecycleNode). First call opens
// the file and writes the header; later calls append in text mode with
// std::flush after every row so an aborted session still produces a
// readable file.
//
// Header-only. Thread-safety: not intended for concurrent use — one writer
// per controller LifecycleNode is the expected topology.

namespace rtc::mpc {

class MpcSolveTimingLogger {
public:
  MpcSolveTimingLogger() = default;
  ~MpcSolveTimingLogger() = default;

  MpcSolveTimingLogger(const MpcSolveTimingLogger &) = delete;
  MpcSolveTimingLogger &operator=(const MpcSolveTimingLogger &) = delete;

  /// Resolve the CSV path (session_dir/controllers/<config_key>/
  /// mpc_solve_timing.csv), create parent directories, and open the file in
  /// append mode. Returns false on filesystem errors — caller may retry or
  /// skip. `config_key` is typically the controller's YAML root key (which
  /// also matches its LifecycleNode namespace).
  [[nodiscard]] bool Open(std::string_view config_key) noexcept {
    try {
      const auto session = ResolveSessionDir();
      const auto controller_dir =
          session / "controllers" / std::string(config_key);
      std::error_code ec;
      std::filesystem::create_directories(controller_dir, ec);
      path_ = controller_dir / "mpc_solve_timing.csv";
      const bool is_new = !std::filesystem::exists(path_, ec);
      out_.open(path_, std::ios::out | std::ios::app);
      if (!out_) {
        return false;
      }
      if (is_new) {
        out_ << "t_wall_ns,count,window,last_ns,min_ns,p50_ns,p99_ns,max_ns,"
                "mean_ns\n";
        out_.flush();
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool IsOpen() const noexcept { return out_.is_open(); }

  [[nodiscard]] const std::filesystem::path &Path() const noexcept {
    return path_;
  }

  /// Append one row. No-op if not open. Uses CLOCK_REALTIME-ish wall time
  /// from the steady_clock epoch for monotonicity-safe plotting.
  void Log(const rtc::MpcSolveStats &s) noexcept {
    if (!out_.is_open())
      return;
    const auto now = std::chrono::steady_clock::now();
    const auto t_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          now.time_since_epoch())
                          .count();
    out_ << t_ns << ',' << s.count << ',' << s.window << ',' << s.last_ns << ','
         << s.min_ns << ',' << s.p50_ns << ',' << s.p99_ns << ',' << s.max_ns
         << ',' << s.mean_ns << '\n';
    out_.flush();
  }

private:
  std::filesystem::path path_;
  std::ofstream out_;
};

} // namespace rtc::mpc

#endif // RTC_MPC_LOGGING_MPC_SOLVE_TIMING_LOGGER_HPP_
