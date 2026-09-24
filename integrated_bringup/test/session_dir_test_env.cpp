// Registers the package-wide IsolatedSessionDir (session_dir_test_fixture.hpp).
// Linked into every `test_*` gtest executable by CMakeLists.txt, so a new test
// binary cannot forget it.
#include "session_dir_test_fixture.hpp"

namespace integrated_bringup::testfx {

IsolatedSessionDir& SessionDir() {
  // Function-local so a test file's own static initializer that asks for the
  // directory gets the same instance, whatever the translation-unit order.
  static auto* const env =
      static_cast<IsolatedSessionDir*>(::testing::AddGlobalTestEnvironment(new IsolatedSessionDir));
  return *env;
}

namespace {
[[maybe_unused]] const IsolatedSessionDir& kRegistered = SessionDir();
}  // namespace

}  // namespace integrated_bringup::testfx
