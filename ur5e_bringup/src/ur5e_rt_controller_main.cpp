// ── UR5e entry point ─────────────────────────────────────────────────────────
//
// Demo controllers are registered via --whole-archive linking (no Force
// function needed).  Built-in controllers still use the Force pattern
// because rtc_controllers is linked through ament (transitive static lib).
//
// Node name = executable name = "ur5e_rt_controller" so that pgrep, log
// prefixes, ROS introspection (`ros2 node list`) and the binary on disk all
// share the same identifier. rtc_controller_manager is robot-agnostic and
// does not own a runtime identity — see agent_docs/design-principles.md.
#include "rtc_controller_manager/rt_controller_main.hpp"

// Force-link built-in controller registrations from the static library
namespace rtc {
void ForceBuiltinControllerRegistration();
}

int main(int argc, char **argv) {
  rtc::ForceBuiltinControllerRegistration();
  return rtc::RtControllerMain(argc, argv, "ur5e_rt_controller");
}
