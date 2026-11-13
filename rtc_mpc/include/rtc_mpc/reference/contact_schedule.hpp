#ifndef RTC_MPC_REFERENCE_CONTACT_SCHEDULE_HPP_
#define RTC_MPC_REFERENCE_CONTACT_SCHEDULE_HPP_

/// @file contact_schedule.hpp
/// @brief A window of contact modes over time: the gait scheduler's output.
///
/// `ContactSchedule` covers `[t0, t_end)`. An **event** is an instant where the
/// mode changes (two phases with the same mode are one interval, so a stance
/// window has no event). Mode `mode[i]` holds on the half-open interval
/// `[event_time[i−1], event_time[i])` with `event_time[−1] = t0` and
/// `event_time[n_events] = t_end`; outside the window the end modes hold.
/// Bit k of a mode is contact k in the gait table's contact order (ARCH-1).
///
/// Times are seconds on the scheduler owner's clock (mpc_main's configure-
/// relative steady time). They do not leave the MPC thread: RT and GUI
/// consumers get per-node modes, not this window.
///
/// Identity: `schedule_id` changes exactly when any other field changes (equal
/// id ⇒ every field equal, unused slots 0); `epoch` counts the gait commits and
/// resets behind it — a different axis from `MPCSolution::plan_id` and
/// `model_epoch`.
///
/// Every helper here is noexcept and heap-free except @ref ToContactPlan
/// (configure-time only).

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/types/contact_plan_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace rtc::mpc {

inline constexpr std::size_t kMaxScheduleEvents = 31;

struct ContactSchedule {
  double t0{0.0};                                       ///< window start [s]
  double t_end{0.0};                                    ///< window end [s]
  std::array<double, kMaxScheduleEvents> event_time{};  ///< strictly increasing, in (t0, t_end)
  std::array<std::uint32_t, kMaxScheduleEvents + 1> mode{};  ///< `n_events + 1` used
  std::uint64_t schedule_id{0};  ///< 0 = never built; +1 per content change
  std::uint64_t epoch{0};        ///< +1 per gait commit / reset
  std::uint32_t n_contacts{0};
  std::uint32_t n_events{0};
};

static_assert(std::is_trivially_copyable_v<ContactSchedule>);
static_assert(sizeof(ContactSchedule) == sizeof(double) * (2 + kMaxScheduleEvents) +
                                             sizeof(std::uint32_t) * (kMaxScheduleEvents + 1) +
                                             2 * sizeof(std::uint64_t) + 2 * sizeof(std::uint32_t),
              "ContactSchedule must carry no padding: equal fields ⇔ equal bytes");

/// @brief The mode at `t` (half-open: at an event the new mode holds).
[[nodiscard]] std::uint32_t ModeAt(const ContactSchedule& s, double t) noexcept;

/// @brief The first event strictly after `t`. false (outputs untouched) if none.
[[nodiscard]] bool NextEvent(const ContactSchedule& s, double t, double& t_event,
                             std::uint32_t& mode_after) noexcept;

/// @brief `out[k] = ModeAt(t_start + k·dt)` for `k = 0 … horizon`. false (out
///        untouched) when `out` is shorter than `horizon + 1`, `dt ≤ 0`, an
///        input is non-finite, or a node falls outside `[t0, t_end]`.
[[nodiscard]] bool SampleNodes(const ContactSchedule& s, double t_start, double dt,
                               std::size_t horizon, std::span<std::uint32_t> out) noexcept;

/// @brief A fixed grid of node times: node `i` at `anchor + i·dt`, computed
///        from the integer index and never accumulated, so one index names one
///        time bit for bit whichever tick computes it. A horizon whose node 0
///        is grid node `m` has its node `k` at `Time(m + k)`.
struct NodeGrid {
  double anchor{0.0};  ///< [s] time of node index 0
  double dt{0.0};      ///< [s] node spacing

  [[nodiscard]] constexpr double Time(std::int64_t index) const noexcept {
    return anchor + static_cast<double>(index) * dt;
  }
};

/// @brief The grid cell holding `t`: the largest `i` with `Time(i) ≤ t`, so
///        `Time(i) ≤ t < Time(i + 1)` with both sides computed by `Time`.
///        false (out untouched) for a non-finite input, `dt ≤ 0`, or an index
///        beyond ±2^52.
[[nodiscard]] bool NodeIndexAt(const NodeGrid& grid, double t, std::int64_t& out) noexcept;

/// @brief `out[k] = ModeAt(grid.Time(first + k))` for `k = 0 … horizon`.
///        false (out untouched) when `out` is shorter than `horizon + 1`, the
///        grid is not valid, node 0 is before `t0`, or the last node is past
///        `t_end` by more than rounding (a few ulp — such a node reads the end
///        mode, as any time past the window does).
[[nodiscard]] bool SampleGridNodes(const ContactSchedule& s, const NodeGrid& grid,
                                   std::int64_t first, std::size_t horizon,
                                   std::span<std::uint32_t> out) noexcept;

/// @brief One stretch without contact `k`. An end cut by the window is flagged.
struct SwingInterval {
  double lift_off{0.0};    ///< [s] (t0 when `open_start`)
  double touch_down{0.0};  ///< [s] (t_end when `open_end`)
  bool open_start{false};  ///< already in swing at t0
  bool open_end{false};    ///< still in swing at t_end
};

/// @brief The swing stretches of contact `contact`, in time order. Writes the
///        first `out.size()` and returns how many there are (0 if `contact`
///        is out of range).
[[nodiscard]] std::size_t SwingIntervals(const ContactSchedule& s, std::size_t contact,
                                         std::span<SwingInterval> out) noexcept;

enum class ContactPlanError {
  kNoError = 0,
  kContactCountMismatch,  ///< `s.n_contacts != model.n_contacts()`
};

/// @brief The window as a `ContactPlan` (times relative to `t0`, frames from
///        `model`). Configure-time only — it allocates.
[[nodiscard]] ContactPlanError ToContactPlan(const ContactSchedule& s,
                                             const RobotModelHandler& model, ContactPlan& out);

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_CONTACT_SCHEDULE_HPP_
