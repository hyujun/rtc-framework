// Log capture for RtControllerNode tests whose only observable is a log line.
//
// Some bring-up paths deliberately do not refuse, do not change the
// CallbackReturn, and add no member to the node — an advisory, a warning about
// a parameter override that will not be read. Inventing a counter on
// RtControllerNode purely so a test could read it would put test-only state in
// the RT node, so the assertion goes to the sink the message actually reaches.
//
// The rcutils output handler is process-global. A fixture installs the sink in
// SetUp and restores the previous handler in TearDown, so the capture covers
// one test case and cannot reach a test in another gtest binary. Every line is
// still handed on to the handler the sink replaced, so a failing case keeps
// its log (the ERROR that explains a refused configure) on the console.
#pragma once

#include <rcutils/logging.h>

#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rtc {

class LogSink {
 public:
  // Clears what an earlier case captured, then takes over the handler. A
  // second Install before Restore keeps the handler the first one replaced —
  // recording the sink as its own predecessor would leave Restore unable to
  // give the original back, and every later case's log would vanish into it.
  static void Install() {
    {
      const std::lock_guard<std::mutex> lock(Mutex());
      Lines().clear();
    }
    const rcutils_logging_output_handler_t current = rcutils_logging_get_output_handler();
    if (current != &LogSink::Handler) {
      Previous().store(current);
    }
    rcutils_logging_set_output_handler(&LogSink::Handler);
  }

  static void Restore() {
    const rcutils_logging_output_handler_t previous = Previous().exchange(nullptr);
    if (previous != nullptr) {
      rcutils_logging_set_output_handler(previous);
    }
  }

  // Captured lines at `severity` containing every needle. Matching on
  // substrings rather than the whole message keeps the assertions from pinning
  // wording that carries no contract — but each needle a test passes is a
  // claim the operator has to receive, not decoration.
  static std::vector<std::string> Matching(int severity, const std::vector<std::string>& needles) {
    const std::lock_guard<std::mutex> lock(Mutex());
    std::vector<std::string> hits;
    for (const auto& [sev, text] : Lines()) {
      if (sev != severity) {
        continue;
      }
      bool all = true;
      for (const auto& needle : needles) {
        if (text.find(needle) == std::string::npos) {
          all = false;
          break;
        }
      }
      if (all) {
        hits.push_back(text);
      }
    }
    return hits;
  }

 private:
  // Longer than any message the suites assert on; vsnprintf truncates and
  // NUL-terminates regardless, so an overlong line degrades to a shorter
  // capture rather than corrupting the sink.
  static constexpr std::size_t kLineBufferSize = 1024;

  static void Handler(const rcutils_log_location_t* location, int severity, const char* name,
                      rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
    std::array<char, kLineBufferSize> buf{};
    va_list copy;
    va_copy(copy, *args);
    (void)std::vsnprintf(buf.data(), buf.size(), format, copy);
    va_end(copy);
    {
      const std::lock_guard<std::mutex> lock(Mutex());
      Lines().emplace_back(severity, std::string(buf.data()));
    }
    // Outside the lock: the previous handler (rclcpp's) takes its own.
    if (const rcutils_logging_output_handler_t previous = Previous().load(); previous != nullptr) {
      previous(location, severity, name, timestamp, format, args);
    }
  }

  // Function-local statics rather than data members: the handler is a plain C
  // function pointer with no user-data slot, so the sink has to be reachable
  // without an instance.
  static std::mutex& Mutex() {
    static std::mutex m;
    return m;
  }

  static std::vector<std::pair<int, std::string>>& Lines() {
    static std::vector<std::pair<int, std::string>> lines;
    return lines;
  }

  // Atomic: Handler reads it on whichever thread logs, while the test thread
  // installs and restores.
  static std::atomic<rcutils_logging_output_handler_t>& Previous() {
    static std::atomic<rcutils_logging_output_handler_t> handler{nullptr};
    return handler;
  }
};

}  // namespace rtc
