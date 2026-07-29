// ── Built-in controller registration
// ──────────────────────────────────────────
//
// rtc_controllers provides reusable control LAWS (compliance/, joint/, task/),
// their configuration schemas (params/) and helper components (GraspController,
// trajectory generators).  It no longer provides any class that implements the
// framework's controller contract: the adapters that used to live here were
// deleted in #236 S7c, so there is nothing here that COULD be registered.
// Everything is consumed as *library symbols* only.
//
// (`grep -rn "public RTControllerInterface" include/` is the check, not a count
// kept in prose — AP-DOC-1.)
//
// Downstream <robot>_bringup packages (e.g. integrated_bringup) register the
// controllers they expose via RTC_REGISTER_CONTROLLER, keeping the set of
// runtime-selectable controllers minimal and robot-specific.
//
// ForceBuiltinControllerRegistration() is retained as a no-op so existing
// main() entry points that reference it continue to link unchanged.

namespace rtc {
void ForceBuiltinControllerRegistration() {}
}  // namespace rtc
