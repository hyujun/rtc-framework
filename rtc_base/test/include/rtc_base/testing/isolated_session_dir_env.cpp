// Registers the binary's rtc::testing::IsolatedSessionDir from a static
// initializer. Never included: rtc_base_isolate_test_sessions()
// (cmake/rtc_base-test-include.cmake.in) compiles it into every `test_*`
// executable of the calling package, so a new test binary cannot forget it.
#include "rtc_base/testing/isolated_session_dir.hpp"

namespace {
[[maybe_unused]] const auto& kRegistered = rtc::testing::SessionDir();
}  // namespace
