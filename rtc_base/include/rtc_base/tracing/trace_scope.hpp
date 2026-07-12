// RTC_TRACE_SCOPE — RAII scoped tracing span for RT hot-path instrumentation.
//
// Usage (inside any function, including RT-tick `noexcept` code):
//
//     void DemoWbcController::ComputeControl(...) noexcept {
//       RTC_TRACE_SCOPE("DemoWbcController::ComputeControl");
//       ...                       // span_begin fired here
//     }                           // span_end fired on scope exit (incl. early return)
//
// Nesting these across the call stack (RT tick → CM::Compute → controller →
// sub-step) produces a nested flame stack in Perfetto once the CTF trace is
// run through rtc_tools' ctf_to_chrome_trace converter.
//
// Compile-time gate: instrumentation only exists when the translation unit is
// built with RTC_TRACING_ENABLED defined — which rtc_base sets (PUBLIC) on the
// `rtc_tracing` target when the CMake option RTC_ENABLE_TRACING is ON. In a
// default build the macro expands to a no-op statement, pulls in no LTTng
// header, links no lttng-ust, and adds zero RT hot-path cost. See
// rtc_base/CMakeLists.txt and docs/tracing.md.

#ifndef RTC_BASE_TRACING_TRACE_SCOPE_HPP
#define RTC_BASE_TRACING_TRACE_SCOPE_HPP

#if defined(RTC_TRACING_ENABLED)

// lttng headers are not warning-clean under this repo's -Wpedantic/-Wconversion
// wall; isolate them so call-site TUs stay quiet. The tracepoint() expansions
// inside Scope's methods are covered by the same push/pop region.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

#include "rtc_base/tracing/rtc_tracepoints.hpp"

namespace rtc::trace {

// Emits rtc:span_begin(name) on construction and rtc:span_end on destruction.
// `name` must outlive the Scope — always pass a string literal. noexcept: the
// underlying tracepoint() is wait-free and never throws. Non-copyable,
// non-movable so the begin/end pair is strictly bound to one lexical scope.
class Scope {
 public:
  explicit Scope(const char* name) noexcept { tracepoint(rtc, span_begin, name); }

  ~Scope() noexcept { tracepoint(rtc, span_end); }

  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
  Scope(Scope&&) = delete;
  Scope& operator=(Scope&&) = delete;
};

}  // namespace rtc::trace

#pragma GCC diagnostic pop

#define RTC_TRACE_DETAIL_CONCAT(a, b) a##b
#define RTC_TRACE_DETAIL_CONCAT2(a, b) RTC_TRACE_DETAIL_CONCAT(a, b)
// Unique variable name per call site so nested scopes in one function don't
// -Wshadow each other.
#define RTC_TRACE_SCOPE(name) \
  ::rtc::trace::Scope RTC_TRACE_DETAIL_CONCAT2(rtc_trace_scope_, __LINE__)(name)

#else  // !RTC_TRACING_ENABLED — no-op, zero cost, no dependency.

#define RTC_TRACE_SCOPE(name) \
  do {                        \
  } while (false)

#endif  // RTC_TRACING_ENABLED

#endif  // RTC_BASE_TRACING_TRACE_SCOPE_HPP
