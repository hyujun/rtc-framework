// ── Built-in controller registration
// ──────────────────────────────────────────
//
// rtc_controllers provides four reusable controller classes (PController,
// JointPDController, ClikController, OperationalSpaceController) plus helper
// components (GraspController, trajectory generators).  These are consumed as
// *library symbols* only; they are intentionally NOT auto-registered into
// ControllerRegistry.
//
// Downstream <robot>_bringup packages (e.g. ur5e_bringup) register the
// controllers they expose via RTC_REGISTER_CONTROLLER, keeping the set of
// runtime-selectable controllers minimal and robot-specific.
//
// ForceBuiltinControllerRegistration() is retained as a no-op so existing
// main() entry points that reference it continue to link unchanged.

namespace rtc {
void ForceBuiltinControllerRegistration() {}
} // namespace rtc
