// Single translation unit that materializes the `rtc:*` LTTng UST probes.
//
// TRACEPOINT_CREATE_PROBES generates the provider's probe descriptors and
// TRACEPOINT_DEFINE emits the one-and-only definition of the tracepoint
// symbols. They must be defined in exactly ONE object per process image;
// this file is compiled into the shared library `librtc_tracing.so`, which
// every tracing-enabled executable links. Call-site TUs (via trace_scope.hpp)
// include the same provider header WITHOUT these macros, so they only see the
// tracepoint() declarations. Do not define TRACEPOINT_DEFINE anywhere else.
//
// Built only when the CMake option RTC_ENABLE_TRACING is ON.

#define TRACEPOINT_CREATE_PROBES
#define TRACEPOINT_DEFINE

#include "rtc_base/tracing/rtc_tracepoints.hpp"
