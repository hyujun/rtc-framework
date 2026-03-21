// ── UR5e entry point ─────────────────────────────────────────────────────────
//
// Demo controllers are registered via --whole-archive linking (no Force
// function needed).  Built-in controllers still use the Force pattern
// because rtc_controllers is linked through ament (transitive static lib).
#include "rtc_controller_manager/rt_controller_main.hpp"

// Force-link built-in controller registrations from the static library
namespace rtc { void ForceBuiltinControllerRegistration(); }

int main(int argc, char ** argv)
{
  rtc::ForceBuiltinControllerRegistration();
  return rtc::RtControllerMain(argc, argv);
}
