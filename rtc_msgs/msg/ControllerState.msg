# Snapshot of a single controller's metadata + lifecycle state, returned by
# /rtc_cm/list_controllers. Mirrors ros2_control's ControllerState semantics
# but trimmed to fields RTC actually tracks (no chaining / no dynamic load).

# Controller instance name as registered with RTC_REGISTER_CONTROLLER and
# returned by RTControllerInterface::Name().
string name

# Lifecycle state. One of:
#   "unconfigured" — pre-on_configure (CM not yet configured)
#   "inactive"    — on_configure succeeded, not the active controller
#   "active"      — currently dispatched by the RT loop
#   "finalized"   — on_cleanup completed (CM shutting down)
string state

# Controller type — registry plugin name key used in RTC_REGISTER_CONTROLLER.
string type

# Convenience flag: true iff state == "active". Equivalent information,
# kept for clients that only need a boolean check.
bool is_active

# Device topic group names this controller knows about
# (controller_topic_configs_[i].groups keys, ownership-agnostic).
# Useful for diagnostics — does not imply exclusive ownership.
string[] claimed_groups
