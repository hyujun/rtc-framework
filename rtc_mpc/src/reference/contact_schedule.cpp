#include "rtc_mpc/reference/contact_schedule.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rtc::mpc {
namespace {

// Number of events at or before t — the index of the mode that holds at t.
[[nodiscard]] std::size_t ModeIndex(const ContactSchedule& s, double t) noexcept {
  const auto first = s.event_time.begin();
  const auto last =
      first + static_cast<std::ptrdiff_t>(std::min<std::size_t>(s.n_events, kMaxScheduleEvents));
  return static_cast<std::size_t>(std::upper_bound(first, last, t) - first);
}

// Interval i of the window: [IntervalStart, IntervalEnd).
[[nodiscard]] double IntervalStart(const ContactSchedule& s, std::size_t i) noexcept {
  return i == 0 ? s.t0 : s.event_time[i - 1];
}

[[nodiscard]] double IntervalEnd(const ContactSchedule& s, std::size_t i) noexcept {
  return i == s.n_events ? s.t_end : s.event_time[i];
}

}  // namespace

std::uint32_t ModeAt(const ContactSchedule& s, double t) noexcept {
  return s.mode[ModeIndex(s, t)];
}

bool NextEvent(const ContactSchedule& s, double t, double& t_event,
               std::uint32_t& mode_after) noexcept {
  const std::size_t i = ModeIndex(s, t);
  if (i >= s.n_events) {
    return false;
  }
  t_event = s.event_time[i];
  mode_after = s.mode[i + 1];
  return true;
}

bool SampleNodes(const ContactSchedule& s, double t_start, double dt, std::size_t horizon,
                 std::span<std::uint32_t> out) noexcept {
  if (horizon >= out.size() || !std::isfinite(t_start) || !std::isfinite(dt) || dt <= 0.0) {
    return false;
  }
  const double t_last = t_start + static_cast<double>(horizon) * dt;
  if (t_start < s.t0 || !(t_last <= s.t_end)) {
    return false;
  }
  for (std::size_t k = 0; k <= horizon; ++k) {
    out[k] = ModeAt(s, t_start + static_cast<double>(k) * dt);
  }
  return true;
}

std::size_t SwingIntervals(const ContactSchedule& s, std::size_t contact,
                           std::span<SwingInterval> out) noexcept {
  if (contact >= s.n_contacts || contact >= 8 * sizeof(std::uint32_t)) {
    return 0;
  }
  const std::uint32_t bit = 1U << contact;
  std::size_t count = 0;
  bool in_swing = false;
  SwingInterval cur{};
  for (std::size_t i = 0; i <= s.n_events; ++i) {
    const bool swing = (s.mode[i] & bit) == 0U;
    const double start = IntervalStart(s, i);
    if (swing && !in_swing) {
      cur = SwingInterval{};
      cur.lift_off = start;
      cur.open_start = i == 0;
      in_swing = true;
    } else if (!swing && in_swing) {
      cur.touch_down = start;
      if (count < out.size()) {
        out[count] = cur;
      }
      ++count;
      in_swing = false;
    }
  }
  if (in_swing) {
    cur.touch_down = s.t_end;
    cur.open_end = true;
    if (count < out.size()) {
      out[count] = cur;
    }
    ++count;
  }
  return count;
}

ContactPlanError ToContactPlan(const ContactSchedule& s, const RobotModelHandler& model,
                               ContactPlan& out) {
  if (static_cast<int>(s.n_contacts) != model.n_contacts()) {
    return ContactPlanError::kContactCountMismatch;
  }
  ContactPlan plan;
  plan.frames = model.contact_frames();
  plan.phases.reserve(s.n_events + 1);
  for (std::size_t i = 0; i <= s.n_events; ++i) {
    ContactPhase phase;
    phase.t_start = IntervalStart(s, i) - s.t0;
    phase.t_end = IntervalEnd(s, i) - s.t0;
    for (std::size_t k = 0; k < s.n_contacts; ++k) {
      if ((s.mode[i] >> k) & 1U) {
        phase.active_frame_ids.push_back(plan.frames[k].frame_id);
      }
    }
    plan.phases.push_back(std::move(phase));
  }
  out = std::move(plan);
  return ContactPlanError::kNoError;
}

}  // namespace rtc::mpc
