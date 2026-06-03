// test_urdf_path.hpp — shared resolver for URDF fixtures under
// rtc_urdf_bridge/test/urdf/. Owned by rtc_urdf_bridge; also consumed by
// rtc_controllers tests via a relative include path (see rtc_controllers
// test CMakeLists). The path is resolved relative to this header's own
// source location (__FILE__ inside an inline function always expands to the
// header file, not the including translation unit), so the sibling urdf/
// directory is found regardless of which package's test binary includes it.
// The header is never installed, so this source-tree assumption holds only
// at test-compile time, which is the only context that includes it.
#pragma once

#include <filesystem>
#include <string>

namespace rtc::test {

// Absolute path to <filename> under rtc_urdf_bridge/test/urdf/.
inline std::string TestUrdfPath(const std::string& filename) {
  return (std::filesystem::path(__FILE__).parent_path() / "urdf" / filename)
      .string();
}

}  // namespace rtc::test
