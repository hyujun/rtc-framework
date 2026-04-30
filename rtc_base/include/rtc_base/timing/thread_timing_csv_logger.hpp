#ifndef RTC_BASE_TIMING_THREAD_TIMING_CSV_LOGGER_HPP_
#define RTC_BASE_TIMING_THREAD_TIMING_CSV_LOGGER_HPP_

// Per-tick CSV appender shared by all RT/soft-RT threads that want timing
// CSV output (CM, MPC, …). One CSV row per ThreadTimingSample, drained by
// a non-RT consumer.
//
// Schema:
//   t_wall_ns,tick_count,<payload columns…>
//
// The first two columns are always written by this logger so cross-thread
// analysis scripts can rely on them. Payload columns are emitted by the
// caller-provided `extra_header` / `row_writer`. CM and MPC both bind
// rtc::RtTickTimingPayload (5 phase-timing columns) — see
// rtc_base/timing/rt_tick_timing_sample.hpp — so a single set of tooling
// consumes both CSVs.
//
// Writer side:
//   - `Open(path)` creates the file, writes the schema header, opens in
//     append mode for restartable sessions (no duplicate header on reopen).
//   - `Log(sample)` appends one row + flushes (so an aborted session still
//     produces a readable file).
//   - Header-only; one writer per file. Not thread-safe.
//
// Consumer:
//   - Pair with ThreadTimingProducer<Payload, N>::Drain(...) to forward
//     samples FIFO into Log(). See examples in rtc_mpc /
//     rtc_controller_manager.

#include "rtc_base/timing/thread_timing_sample.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
#include <ostream>
#include <string>
#include <string_view>
#include <system_error>

namespace rtc {

template <typename Payload> class ThreadTimingCsvLogger {
public:
  using Sample = ThreadTimingSample<Payload>;

  /// Function called once at first-open to emit comma-prefixed payload
  /// column names, e.g. ",t_compute_us,t_publish_us". Empty payload =
  /// empty string. Caller writes the leading comma.
  using HeaderWriter = std::function<void(std::ostream &)>;

  /// Function called per row to emit comma-prefixed payload values in the
  /// same order as the header, e.g. ",12.3,4.5".
  using RowWriter = std::function<void(std::ostream &, const Payload &)>;

  ThreadTimingCsvLogger() = default;
  ~ThreadTimingCsvLogger() = default;

  ThreadTimingCsvLogger(const ThreadTimingCsvLogger &) = delete;
  ThreadTimingCsvLogger &operator=(const ThreadTimingCsvLogger &) = delete;

  /// Resolve / create the file at `path` and store the per-row writer.
  /// On first creation the schema header is written; on append (file
  /// already exists) the existing header is preserved. Returns false on
  /// filesystem errors — caller may retry or skip.
  [[nodiscard]] bool Open(const std::filesystem::path &path,
                          const HeaderWriter &header_writer,
                          RowWriter row_writer) noexcept {
    try {
      path_ = path;
      std::error_code ec;
      if (path_.has_parent_path()) {
        std::filesystem::create_directories(path_.parent_path(), ec);
      }
      const bool is_new = !std::filesystem::exists(path_, ec);
      out_.open(path_, std::ios::out | std::ios::app);
      if (!out_) {
        return false;
      }
      if (is_new) {
        out_ << "t_wall_ns,tick_count";
        if (header_writer) {
          header_writer(out_);
        }
        out_ << '\n';
        out_.flush();
      }
      row_writer_ = std::move(row_writer);
      return true;
    } catch (...) {
      return false;
    }
  }

  [[nodiscard]] bool IsOpen() const noexcept { return out_.is_open(); }

  [[nodiscard]] const std::filesystem::path &Path() const noexcept {
    return path_;
  }

  /// Append one row. No-op if not open.
  void Log(const Sample &s) noexcept {
    if (!out_.is_open()) {
      return;
    }
    out_ << s.t_wall_ns << ',' << s.tick_count;
    if (row_writer_) {
      row_writer_(out_, s.payload);
    }
    out_ << '\n';
    out_.flush();
  }

private:
  std::filesystem::path path_;
  std::ofstream out_;
  RowWriter row_writer_;
};

} // namespace rtc

#endif // RTC_BASE_TIMING_THREAD_TIMING_CSV_LOGGER_HPP_
