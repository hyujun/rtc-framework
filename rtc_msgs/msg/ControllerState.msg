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

# ── Per-controller diagnostics (issue #287) ─────────────────────────────────
#
# All four are already tracked exactly inside RTControllerInterface and had no
# observation path out of the process. What they share is that every one of
# them fails SILENTLY: to an operator each looks like "I sent a goal and the
# arm did not move", with no way to tell WHICH of the four happened. The CM
# reads them straight off the base accessors — no per-controller override.

# True while a controller-local fault latch is up
# (RTControllerInterface::HasLatchedFault()). The compliance family latches
# SAFE_STOP on a critical cause and never auto-recovers; /rtc_cm/reset_fault is
# the only exit and it requires naming the controller, so this field is how a
# caller finds out WHICH controller to name.
bool has_latched_fault

# Well-formed goals the target mailbox dropped because its queue was full when
# the producer pushed (RTControllerInterface::GetTargetDropCount()). Blames the
# producer's RATE: the goal was valid, it just arrived faster than one control
# tick could drain. Cumulative since construction, monotonic.
uint64 target_drop_count

# Goals refused before they reached the queue — malformed message payload, or a
# device_idx outside ControllerState::kMaxDevices
# (RTControllerInterface::GetTargetRejectCount()). Blames the producer's
# PAYLOAD, as opposed to target_drop_count's rate. Monotonic.
uint64 target_reject_count

# Drained entries that reached the base's default (no-op) ApplyPendingTarget
# (RTControllerInterface::GetTargetUnhandledCount()). Nonzero means the
# controller pushes onto the mailbox but never overrode ApplyPendingTarget, so
# every goal it accepts is popped and thrown away — a wiring defect in the
# controller itself, not in the producer. Monotonic.
uint64 target_unhandled_count
