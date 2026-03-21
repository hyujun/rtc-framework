// ── Generic entry point (built-in controllers only) ──────────────────────────
#include "rtc_controller_manager/rt_controller_main.hpp"

// Force-link built-in controller registrations from the static library
namespace rtc { void ForceBuiltinControllerRegistration(); }

int main(int argc, char ** argv)
{
  rtc::ForceBuiltinControllerRegistration();
  return rtc::RtControllerMain(argc, argv);
}
