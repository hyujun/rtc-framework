#include "rtc_mpc/reference/gait_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace rtc::mpc {
namespace {

// First tile index worth visiting for time x on a gait anchored at `anchor`:
// one tile early so every phase start ≤ x is seen, never before the anchor.
[[nodiscard]] std::int64_t FirstTile(double anchor, double period, double x) noexcept {
  const double k = std::floor((x - anchor) / period) - 1.0;
  return k > 0.0 ? static_cast<std::int64_t>(k) : 0;
}

[[nodiscard]] double PhaseStart(const GaitTemplate& g, double anchor, std::int64_t k,
                                std::size_t j) noexcept {
  return anchor + static_cast<double>(k) * g.period + g.switching_time[j];
}

// Mode of gait `g` (anchored at `anchor`) at time `x ≥ anchor`.
[[nodiscard]] std::uint32_t GaitModeAt(const GaitTemplate& g, double anchor, double x) noexcept {
  std::uint32_t mode = g.mode[0];
  for (std::int64_t k = FirstTile(anchor, g.period, x);; ++k) {
    for (std::size_t j = 0; j < g.n_phases; ++j) {
      if (PhaseStart(g, anchor, k, j) > x) {
        return mode;
      }
      mode = g.mode[j];
    }
  }
}

// Appends the mode changes of the window being built; merges equal modes.
struct EventWriter {
  ContactSchedule& out;
  bool overflow{false};

  void Push(double t, std::uint32_t mode) noexcept {
    if (mode == out.mode[out.n_events]) {
      return;
    }
    if (out.n_events >= kMaxScheduleEvents) {
      overflow = true;
      return;
    }
    out.event_time[out.n_events] = t;
    ++out.n_events;
    out.mode[out.n_events] = mode;
  }
};

// Phase starts of `g` strictly inside (from, to), pushed in order.
void TileEvents(const GaitTemplate& g, double anchor, double from, double to,
                EventWriter& w) noexcept {
  for (std::int64_t k = FirstTile(anchor, g.period, from);; ++k) {
    for (std::size_t j = 0; j < g.n_phases; ++j) {
      const double start = PhaseStart(g, anchor, k, j);
      if (start >= to) {
        return;
      }
      if (start > from) {
        w.Push(start, g.mode[j]);
      }
    }
  }
}

[[nodiscard]] bool SingleMode(const GaitTemplate& g) noexcept {
  for (std::size_t j = 1; j < g.n_phases; ++j) {
    if (g.mode[j] != g.mode[0]) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::uint32_t ModeBefore(const GaitTemplate& g, std::size_t j) noexcept {
  return g.mode[j == 0 ? g.n_phases - 1 : j - 1];
}

// The phase of `next` that starts at the switch: a full-support phase entered
// from `before` (the stride carries on — the foot that was in swing lands, the
// other lifts next), else its first full-support phase entered from a
// non-full one, else phase 0 (a single-mode gait, or one without full support).
[[nodiscard]] std::size_t EntryPhase(const GaitTemplate& next, std::uint32_t before) noexcept {
  if (SingleMode(next)) {
    return 0;
  }
  const std::uint32_t u = ModeUnion(next);
  for (std::size_t j = 0; j < next.n_phases; ++j) {
    if (next.mode[j] == u && ModeBefore(next, j) != u && ModeBefore(next, j) == before) {
      return j;
    }
  }
  for (std::size_t j = 0; j < next.n_phases; ++j) {
    if (next.mode[j] == u && ModeBefore(next, j) != u) {
      return j;
    }
  }
  return 0;
}

}  // namespace

GaitSchedulerError GaitScheduler::Init(const GaitTable& table, double horizon_s) noexcept {
  if (ValidateGaitTable(table) != GaitTableError::kNoError) {
    return GaitSchedulerError::kInvalidTable;
  }
  if (!std::isfinite(horizon_s) || horizon_s <= 0.0) {
    return GaitSchedulerError::kInvalidHorizon;
  }
  const double window = horizon_s + table.switch_lead;
  // Interior intervals are ≥ MinPhase except the one a single-mode switch cuts,
  // plus the window's cut first interval: n_events ≤ window / min_phase + 2.
  const double bound = std::ceil(window / MinPhase(table) - 1e-9) + 2.0;
  if (bound > static_cast<double>(kMaxScheduleEvents)) {
    return GaitSchedulerError::kCapacity;
  }
  table_ = table;
  horizon_ = horizon_s;
  // schedule_id and epoch keep counting across Init: an id never names two contents.
  const std::uint64_t id = schedule_.schedule_id;
  schedule_ = ContactSchedule{};
  schedule_.schedule_id = id;
  schedule_.epoch = epoch_;
  current_ = target_ = 0;
  pending_ = anchored_ = false;
  last_t_ = 0.0;
  requested_gait_.store(-1, std::memory_order_release);
  reset_requested_.store(false, std::memory_order_release);
  policy_enabled_.store(true, std::memory_order_release);
  initialized_ = true;
  return GaitSchedulerError::kNoError;
}

void GaitScheduler::RequestGait(std::uint32_t index) noexcept {
  policy_enabled_.store(false, std::memory_order_release);
  requested_gait_.store(static_cast<std::int64_t>(index), std::memory_order_release);
}

void GaitScheduler::SetPolicyEnabled(bool enabled) noexcept {
  policy_enabled_.store(enabled, std::memory_order_release);
}

void GaitScheduler::RequestReset() noexcept {
  reset_requested_.store(true, std::memory_order_release);
}

double GaitScheduler::SwitchTime(double t_r) const noexcept {
  const GaitTemplate& g = table_.gaits[current_];
  const double earliest = t_r + table_.switch_lead;
  const double limit = WindowEnd(t_r);
  const std::uint32_t full = ModeUnion(g);
  if (SingleMode(g)) {
    return earliest;
  }
  for (std::int64_t k = FirstTile(anchor_, g.period, earliest);; ++k) {
    for (std::size_t j = 0; j < g.n_phases; ++j) {
      const double start = PhaseStart(g, anchor_, k, j);
      if (start >= limit) {
        return limit;
      }
      const std::uint32_t before = ModeBefore(g, j);
      if (start >= earliest && g.mode[j] == full && before != full) {
        return start;
      }
    }
  }
}

void GaitScheduler::Commit(std::uint32_t gait, double t) noexcept {
  if (!pending_) {
    if (gait == current_) {
      return;
    }
    t_switch_ = SwitchTime(t);
    target_ = gait;
    pending_ = true;
  } else {
    if (gait == target_) {
      return;
    }
    if (gait == current_) {
      pending_ = false;  // back to the running gait: cancel
    } else {
      target_ = gait;  // last request wins, t_s stays
    }
  }
  ++epoch_;
  last_commit_ = t;
  if (pending_) {
    next_anchor_ = EntryAnchor();
    CompleteIfDue(t);  // a single-mode gait with zero lead switches at once
  }
}

void GaitScheduler::CompleteIfDue(double t) noexcept {
  if (pending_ && t_switch_ <= t) {
    anchor_ = next_anchor_;
    current_ = target_;
    pending_ = false;
  }
}

double GaitScheduler::EntryAnchor() const noexcept {
  const GaitTemplate& run = table_.gaits[current_];
  const GaitTemplate& next = table_.gaits[target_];
  const std::uint32_t before =
      GaitModeAt(run, anchor_, std::nextafter(t_switch_, -std::numeric_limits<double>::infinity()));
  return t_switch_ - next.switching_time[EntryPhase(next, before)];
}

bool GaitScheduler::Build(double t, ContactSchedule& out) const noexcept {
  out = ContactSchedule{};
  out.t0 = t;
  out.t_end = WindowEnd(t);
  out.n_contacts = table_.n_contacts;
  out.epoch = epoch_;
  const GaitTemplate& run = table_.gaits[current_];
  out.mode[0] = GaitModeAt(run, anchor_, t);
  EventWriter w{out};
  const bool switching = pending_ && t_switch_ < out.t_end;
  TileEvents(run, anchor_, t, switching ? t_switch_ : out.t_end, w);
  if (switching) {
    const GaitTemplate& next = table_.gaits[target_];
    w.Push(t_switch_, GaitModeAt(next, next_anchor_, t_switch_));
    TileEvents(next, next_anchor_, t_switch_, out.t_end, w);
  }
  return !w.overflow;
}

GaitSchedulerError GaitScheduler::Advance(double t, const FilteredCommand& cmd,
                                          const PlanarVelocity& measured) noexcept {
  if (!initialized_) {
    return GaitSchedulerError::kNotInitialised;
  }
  if (!std::isfinite(t)) {
    return GaitSchedulerError::kNonFiniteTime;
  }
  if (anchored_ && t < last_t_) {
    return GaitSchedulerError::kTimeWentBack;
  }
  const bool reset = reset_requested_.exchange(false, std::memory_order_acq_rel);
  if (!anchored_ || reset) {
    anchor_ = t;
    current_ = target_ = 0;
    pending_ = false;
    last_commit_ = t;
    ++epoch_;
    anchored_ = true;
  }
  CompleteIfDue(t);

  const std::int64_t req = requested_gait_.exchange(-1, std::memory_order_acq_rel);
  if (req >= 0) {
    if (req < static_cast<std::int64_t>(table_.n_gaits)) {
      Commit(static_cast<std::uint32_t>(req), t);
    }
  } else if (policy_enabled_.load(std::memory_order_acquire) &&
             t >= last_commit_ + table_.min_dwell) {
    const GaitTemplate& g = table_.gaits[target_gait()];
    const double cx = cmd.valid ? std::abs(cmd.v_x) : 0.0;
    const double cy = cmd.valid ? std::abs(cmd.v_y) : 0.0;
    const double cw = cmd.valid ? std::abs(cmd.omega_z) : 0.0;
    const double mx = std::abs(measured.v_x);
    const double my = std::abs(measured.v_y);
    const double mw = std::abs(measured.omega_z);
    const double f_lin = g.faster_above[kLinearAxis];
    const double f_ang = g.faster_above[kAngularAxis];
    const double s_lin = g.slower_below[kLinearAxis];
    const double s_ang = g.slower_below[kAngularAxis];
    const double tol_lin = g.measured_tolerance[kLinearAxis];
    const double tol_ang = g.measured_tolerance[kAngularAxis];
    const bool faster = (cx > f_lin || cy > f_lin || cw > f_ang) &&
                        (mx > f_lin - tol_lin || my > f_lin - tol_lin || mw > f_ang - tol_ang);
    const bool slower = cx < s_lin && cy < s_lin && cw < s_ang && mx < s_lin + tol_lin &&
                        my < s_lin + tol_lin && mw < s_ang + tol_ang;
    const std::uint32_t idx = target_gait();
    if (faster && idx + 1 < table_.n_gaits) {
      Commit(idx + 1, t);
    } else if (slower && idx > 0) {
      Commit(idx - 1, t);
    }
  }

  if (!Build(t, scratch_)) {
    return GaitSchedulerError::kCapacity;
  }
  scratch_.schedule_id = schedule_.schedule_id;
  if (std::memcmp(&scratch_, &schedule_, sizeof(ContactSchedule)) != 0) {
    scratch_.schedule_id = schedule_.schedule_id + 1;
    schedule_ = scratch_;
  }
  last_t_ = t;
  return GaitSchedulerError::kNoError;
}

}  // namespace rtc::mpc
