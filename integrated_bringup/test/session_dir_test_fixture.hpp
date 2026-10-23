// Every gtest binary in this package logs into one throwaway session directory
// — rtc_base's shared Environment (rtc_base/testing/isolated_session_dir.hpp
// owns the contract and the layout). CMakeLists.txt registers it in each
// `test_*` executable (rtc_base_isolate_test_sessions()), so a test file does
// not register it itself. `SessionDir().Dir()` is for the test that wants to
// read a file back.
//
// Per-file registration was the old contract and seven binaries never did it:
// every `colcon test` (the Stop hook's included) left sessions in the real
// logging_data, next to sim runs started the same minute (2026-09-24).
#pragma once

#include "rtc_base/testing/isolated_session_dir.hpp"

namespace integrated_bringup::testfx {

using rtc::testing::IsolatedSessionDir;
using rtc::testing::SessionDir;

}  // namespace integrated_bringup::testfx
