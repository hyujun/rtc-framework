#ifndef RTC_BASE_LOGGING_THREAD_CSV_LOGGER_HPP_
#define RTC_BASE_LOGGING_THREAD_CSV_LOGGER_HPP_

// Generic CSV file appender for controller-owned data logging.
//
// Sibling of rtc::ThreadTimingCsvLogger (rtc_base/timing/), differing in
// two ways:
//   - The header line is written verbatim by the caller-supplied
//     HeaderWriter — there is no `t_wall_ns,tick_count` prefix the way
//     the timing logger has. The Payload owns the entire row (timestamp
//     column included if the producer chose to put one there).
//   - Log() invokes the caller-supplied RowWriter on the whole row —
//     no preamble bytes are written by this class.
//
// Open() creates the file if missing and writes the header. If the file
// already exists (restartable session) the header is preserved and
// subsequent rows append. Header-only; one writer per file. Not
// thread-safe.

#include <filesystem>
#include <fstream>
#include <functional>
#include <ostream>
#include <system_error>

namespace rtc {

template <typename Payload>
class ThreadCsvLogger {
 public:
  /// Function called once at first-open to emit the entire header line
  /// (no leading or trailing newline — the logger appends '\n').
  using HeaderWriter = std::function<void(std::ostream&)>;

  /// Function called per row to emit the entire row (no leading or
  /// trailing newline — the logger appends '\n' and flushes).
  using RowWriter = std::function<void(std::ostream&, const Payload&)>;

  ThreadCsvLogger() = default;
  ~ThreadCsvLogger() = default;

  ThreadCsvLogger(const ThreadCsvLogger&) = delete;
  ThreadCsvLogger& operator=(const ThreadCsvLogger&) = delete;

  /// Resolve / create the file at `path` and store the per-row writer.
  /// On first creation the header is written via `header_writer`; on
  /// append (file already exists) the existing header is preserved.
  /// Returns false on filesystem errors.
  [[nodiscard]] bool Open(const std::filesystem::path& path, const HeaderWriter& header_writer,
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
      if (is_new && header_writer) {
        header_writer(out_);
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

  [[nodiscard]] const std::filesystem::path& Path() const noexcept { return path_; }

  /// Append one row. No-op if not open.
  void Log(const Payload& payload) noexcept {
    if (!out_.is_open() || !row_writer_) {
      return;
    }
    row_writer_(out_, payload);
    out_ << '\n';
    out_.flush();
  }

 private:
  std::filesystem::path path_;
  std::ofstream out_;
  RowWriter row_writer_;
};

}  // namespace rtc

#endif  // RTC_BASE_LOGGING_THREAD_CSV_LOGGER_HPP_
