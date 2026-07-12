// LTTng UST tracepoint provider for the RTC framework (`rtc:*` events).
//
// This header follows LTTng's mandatory double-inclusion layout: it is
// re-read once per translation unit that defines TRACEPOINT_CREATE_PROBES /
// TRACEPOINT_DEFINE (the probe definition TU, rtc_tracepoints.cpp) and once
// per call-site TU that only needs the tracepoint() declarations
// (rtc_base/tracing/trace_scope.hpp). Do NOT add a plain `#pragma once` — the
// multi-read guard below is intentional and required by <lttng/tracepoint.h>.
//
// Two events form a nested B/E span pair on the emitting thread:
//   rtc:span_begin(name)  — push a slice labelled `name` (a compile-time
//                           string literal; passed as a pointer, no alloc).
//   rtc:span_end          — pop the innermost open slice.
// The RAII helper in trace_scope.hpp guarantees begin/end are balanced even
// across early returns, so Perfetto renders a correct flame stack.
//
// RT-safety: lttng-ust's tracepoint() writes into a lock-free, wait-free,
// per-CPU ring buffer pre-allocated at session start — no malloc, no lock,
// no throw on the hot path. When the `rtc` provider is not enabled in the
// active LTTng session the call is a single predicted-untaken branch.

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER rtc

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "rtc_base/tracing/rtc_tracepoints.hpp"

#if !defined(RTC_BASE_TRACING_RTC_TRACEPOINTS_HPP) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define RTC_BASE_TRACING_RTC_TRACEPOINTS_HPP

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(rtc, span_begin, TP_ARGS(const char*, name), TP_FIELDS(ctf_string(name, name)))

TRACEPOINT_EVENT(rtc, span_end, TP_ARGS(), TP_FIELDS())

#endif /* RTC_BASE_TRACING_RTC_TRACEPOINTS_HPP */

#include <lttng/tracepoint-event.h>
