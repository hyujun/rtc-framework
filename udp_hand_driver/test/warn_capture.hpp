#pragma once

// Test-only: captures WARN lines for one scope, for checks whose only
// observable is the log — a configure warning that changes no state, or how
// often a detector repeats a warning. Every line, captured or not, is still
// handed on to the handler the capture replaced, so a failing case keeps its
// console log. The rcutils output handler is process-global: one capture at a
// time, and the code under test may log from any thread.

#include <rcutils/logging.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace udp_hand_driver::test_support {

class WarnCapture {
 public:
  WarnCapture() {
    // Before the first log line rcutils is not initialized yet, and its lazy
    // initialization would install the default handler over this one.
    (void)rcutils_logging_initialize();
    Previous().store(rcutils_logging_get_output_handler());
    Active().store(this);
    rcutils_logging_set_output_handler(&WarnCapture::Handler);
  }

  ~WarnCapture() {
    rcutils_logging_set_output_handler(Previous().load());
    Active().store(nullptr);
  }

  WarnCapture(const WarnCapture&) = delete;
  WarnCapture& operator=(const WarnCapture&) = delete;

  // Captured WARN lines containing `needle`.
  int Count(const std::string& needle) {
    const std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(std::count_if(lines_.begin(), lines_.end(), [&](const auto& line) {
      return line.find(needle) != std::string::npos;
    }));
  }

 private:
  static constexpr std::size_t kLineBufferSize = 1024;

  // Function-local statics: the handler is a plain C function pointer with no
  // user-data slot. Atomic, because the handler runs on the logging thread.
  static std::atomic<WarnCapture*>& Active() {
    static std::atomic<WarnCapture*> active{nullptr};
    return active;
  }

  static std::atomic<rcutils_logging_output_handler_t>& Previous() {
    static std::atomic<rcutils_logging_output_handler_t> previous{nullptr};
    return previous;
  }

  static void Handler(const rcutils_log_location_t* location, int severity, const char* name,
                      rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
    if (WarnCapture* self = Active().load();
        self != nullptr && severity == RCUTILS_LOG_SEVERITY_WARN) {
      std::array<char, kLineBufferSize> buf{};
      va_list copy;
      va_copy(copy, *args);
      (void)std::vsnprintf(buf.data(), buf.size(), format, copy);
      va_end(copy);
      const std::lock_guard<std::mutex> lock(self->mutex_);
      self->lines_.emplace_back(buf.data());
    }
    if (const rcutils_logging_output_handler_t previous = Previous().load(); previous != nullptr) {
      previous(location, severity, name, timestamp, format, args);
    }
  }

  std::mutex mutex_;
  std::vector<std::string> lines_;
};

}  // namespace udp_hand_driver::test_support
