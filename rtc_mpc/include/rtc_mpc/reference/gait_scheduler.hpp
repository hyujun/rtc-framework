#ifndef RTC_MPC_REFERENCE_GAIT_SCHEDULER_HPP_
#define RTC_MPC_REFERENCE_GAIT_SCHEDULER_HPP_

/// @file gait_scheduler.hpp
/// @brief Tiles gait templates over a moving window into a @ref ContactSchedule
///        and runs the automatic gait transition policy.
///
/// Owner: the MPC thread (SCHED_FIFO — RT path). `Init` is configure time;
/// `Advance` and the owner accessors run on the owner thread, allocate nothing
/// and throw nothing. `RequestGait`, `SetPolicyEnabled` and `RequestReset` may
/// be called from any thread (lock-free atomics, consumed by the next Advance).
///
/// Each `Advance(t, …)`:
///  1. a pending `RequestReset` re-anchors gait 0 at `t` (epoch +1);
///  2. a pending switch whose time `t_s ≤ t` completes: the target gait runs
///     from `t_s` (its entry phase, below);
///  3. a `RequestGait(i)` commits gait `i` (it also turned the policy off —
///     the request latches until `SetPolicyEnabled(true)`);
///  4. with the policy on and `t ≥ last commit + min_dwell`, the rule of
///     @ref GaitTemplate moves the target one gait up or down (faster first),
///     clamped to `[0, n_gaits − 1]`;
///  5. the window `[t, t + horizon + switch_lead)` is rebuilt.
///
/// A commit at `t_r` (epoch +1) fixes the switch time once:
/// `t_s` = the first event of the running gait at or after `t_r + switch_lead`
/// whose new mode is that gait's full support (@ref ModeUnion); a gait with a
/// single mode switches at `t_r + switch_lead`; if the window has no such
/// event, `t_s` = the window end at `t_r`. A later commit while the switch is
/// pending replaces its target (last request wins) and keeps `t_s`; a commit
/// back to the running gait cancels it.
///
/// At `t_s` the target gait enters at a full-support phase: one entered from
/// the running gait's mode just before `t_s` when it has one (the stride
/// carries on: the swing foot lands into double support and the other lifts
/// next), else its first full-support phase, else phase 0 (a single-mode gait
/// or one without full support). It is anchored so that phase starts at `t_s`.
///
/// `kCapacity` cannot occur after a successful Init (the Init bound); if it
/// did, the gait state would have advanced while the published schedule stays
/// the previous one.
///
/// Tiling: phase j of tile k starts at `anchor + k·T + switching_time[j]`
/// (integer tile index, no accumulated sums).
///
/// Two owner-thread readers for the reference builder: @ref PhaseInfo (the
/// running and pending gaits' anchors and periods, for @ref GaitPhase at any
/// node time) and @ref SwingStart (the lift-off of a swing already under way
/// at `t`, which the window — it starts at `t` — cannot show).

#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/gait_table.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace rtc::mpc {

/// @brief What @ref GaitPhase needs: the running gait's tiling, and the
///        pending switch's (its target applies from `t_switch` on).
struct GaitPhaseInfo {
  double anchor{0.0};       ///< [s] running gait, phase 0 of tile 0
  double period{0.0};       ///< [s] running gait
  double t_switch{0.0};     ///< [s] valid when `pending`
  double next_anchor{0.0};  ///< [s] target gait, valid when `pending`
  double next_period{0.0};  ///< [s] target gait, valid when `pending`
  bool pending{false};
  bool single_mode{false};       ///< running gait has one mode (no stride to phase)
  bool next_single_mode{false};  ///< target gait has one mode
};

/// @brief The gait phase at `t`: `((t − anchor) mod period) / period ∈ [0, 1)`
///        of the running gait, or of the target gait for `t ≥ t_switch` when a
///        switch is pending. Tile-index arithmetic, strictly inside `[0, 1)`.
[[nodiscard]] double GaitPhase(const GaitPhaseInfo& info, double t) noexcept;

/// @brief Whether the gait in force at `t` (the target from `t_switch` on
///        when a switch is pending, else the running one) has a single mode.
[[nodiscard]] bool SingleModeAt(const GaitPhaseInfo& info, double t) noexcept;

/// @brief Measured planar base velocity (v_x, v_y in the yaw-aligned base frame).
struct PlanarVelocity {
  double v_x{0.0};      ///< [m/s]
  double v_y{0.0};      ///< [m/s]
  double omega_z{0.0};  ///< [rad/s]
};

enum class GaitSchedulerError {
  kNoError = 0,
  kInvalidTable,    ///< @ref ValidateGaitTable refused the table
  kInvalidHorizon,  ///< horizon not finite or ≤ 0
  kCapacity,        ///< `ceil((horizon + switch_lead) / MinPhase) + 2 > kMaxScheduleEvents`
  kNotInitialised,  ///< Advance before a successful Init
  kNonFiniteTime,   ///< `t` NaN / ±inf
  kTimeWentBack,    ///< `t` below the previous Advance
};

class GaitScheduler {
 public:
  /// @brief Take `table` (validated) and the MPC horizon. On failure nothing
  ///        changes. The first Advance after a successful Init anchors gait 0.
  [[nodiscard]] GaitSchedulerError Init(const GaitTable& table, double horizon_s) noexcept;

  /// @brief Run one step at `t` (see file comment). `cmd` with `valid == false`
  ///        reads as zero; only `v_x · v_y · omega_z · valid` are read. On error
  ///        the schedule stays as it was.
  [[nodiscard]] GaitSchedulerError Advance(double t, const FilteredCommand& cmd,
                                           const PlanarVelocity& measured) noexcept;

  [[nodiscard]] const ContactSchedule& schedule() const noexcept { return schedule_; }

  // ── Any thread ──────────────────────────────────────────────────────────────
  /// @brief Force gait `index` and turn the policy off (latched). An index out
  ///        of range is dropped at the next Advance.
  void RequestGait(std::uint32_t index) noexcept;
  void SetPolicyEnabled(bool enabled) noexcept;
  void RequestReset() noexcept;

  [[nodiscard]] bool policy_enabled() const noexcept {
    return policy_enabled_.load(std::memory_order_acquire);
  }

  // ── Owner thread ────────────────────────────────────────────────────────────
  [[nodiscard]] std::uint32_t current_gait() const noexcept { return current_; }

  /// @brief The pending switch's target, else the running gait.
  [[nodiscard]] std::uint32_t target_gait() const noexcept { return pending_ ? target_ : current_; }

  [[nodiscard]] bool switch_pending() const noexcept { return pending_; }

  [[nodiscard]] double switch_time() const noexcept { return t_switch_; }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }

  [[nodiscard]] const GaitTable& table() const noexcept { return table_; }

  /// @brief The tiling @ref GaitPhase reads. false (out untouched) before the
  ///        first Advance anchored a gait.
  [[nodiscard]] bool PhaseInfo(GaitPhaseInfo& out) const noexcept;

  /// @brief When contact `contact` is in swing at `t` (its bit off in the
  ///        running gait's mode), the time that swing started: the last phase
  ///        start in `(since, t]` where the bit went off, `since` being when
  ///        the running gait took over (its first anchoring, a reset, or the
  ///        last completed switch). If the bit was already off at `since` and
  ///        never went on → off after it, `t_lo = since` and `clamped` is set
  ///        (a gait without full support, or a reset into a swing phase).
  ///        false (outputs untouched) when the contact is in stance at `t`,
  ///        out of range, or nothing is anchored yet.
  [[nodiscard]] bool SwingStart(std::size_t contact, double t, double& t_lo,
                                bool& clamped) const noexcept;

 private:
  void Commit(std::uint32_t gait, double t) noexcept;
  [[nodiscard]] double SwitchTime(double t_r) const noexcept;
  [[nodiscard]] bool Build(double t, ContactSchedule& out) const noexcept;
  /// The pending target's entry phase (see file comment), which starts at t_s.
  [[nodiscard]] std::size_t EntryPhaseOfTarget() const noexcept;
  void CompleteIfDue(double t) noexcept;

  /// `t + horizon + switch_lead`, summed in that order (the documented window end).
  [[nodiscard]] double WindowEnd(double t) const noexcept {
    return t + horizon_ + table_.switch_lead;
  }

  GaitTable table_{};
  ContactSchedule schedule_{};
  ContactSchedule scratch_{};
  double horizon_{0.0};
  double anchor_{0.0};
  double t_switch_{0.0};
  double next_anchor_{0.0};     ///< the pending target's anchor (its entry phase at t_s)
  std::size_t entry_phase_{0};  ///< the pending target's entry phase, set at commit
  /// After a completed switch: the running gait's entry phase starts at
  /// exactly `taken_start_` (t_s), not at the anchor's recomputation of it.
  bool entry_taken_{false};
  std::size_t taken_phase_{0};
  double taken_start_{0.0};
  double since_{0.0};  ///< when the running gait took over (SwingStart's floor)
  double last_commit_{0.0};
  double last_t_{0.0};
  std::uint64_t epoch_{0};
  std::uint32_t current_{0};
  std::uint32_t target_{0};
  bool pending_{false};
  bool anchored_{false};
  bool initialized_{false};

  std::atomic<std::int64_t> requested_gait_{-1};
  std::atomic<bool> policy_enabled_{true};
  std::atomic<bool> reset_requested_{false};
  static_assert(std::atomic<std::int64_t>::is_always_lock_free);
  static_assert(std::atomic<bool>::is_always_lock_free);
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_GAIT_SCHEDULER_HPP_
