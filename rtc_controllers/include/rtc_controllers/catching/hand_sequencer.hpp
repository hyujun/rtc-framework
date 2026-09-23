// ── Hand sequencer (L6 §5.3, dynamic_catching S7.1) ──────────────────────────
// What the hand is told to do, tick by tick, over one trial:
//
//   Open ──Ready──▶ Preshape ──(t_cmd, after Commit)──▶ Close ──ρ≥η or timeout──▶ Hold
//     ▲                ▲                                                             │
//     └──Home──        └────────────── at q_pre ◀── Release ◀──────Release───────────┘
//
// THE HAND RULE (#537 S7 decision Q4, 2026-09-23). `q_open` is used only while
// the arm is homing; from the moment the arm is at its wait pose the hand
// waits at `q_pre`, and that is its posture from ARMED through COMMITTED. So
// there is no timed preshape (`t_c − T_pre`): Preshape is entered by the
// supervisor's `Ready()`, and Release ends at `q_pre` rather than at `q_open`.
//
// THE CLOSE INSTANT (L6 §4.3). t_cmd = t_c − T_close_e2e, from the FROZEN t_c
// the supervisor commits to and the hand profile — one source, so the planner's
// own `t_cmd_ns` is a record, not an input (#537 S7, C-14). The command goes
// out on the first tick with now ≥ t_cmd − h/2 (`HandCommandDueRounded`): the
// tick nearest t_cmd, so the error is at most h/2 on a regular tick grid
// instead of up to h late. `now` is the REAL axis — T_arm is the arm servo's
// lag and has nothing to say about when the hand is told to close.
//
// WHAT IS NOT HERE. The supervisor mode (the caller maps modes to the calls
// below), the clock (`now` is an argument, so the core is deterministic under
// test), and the output latch the caller falls back to while this is
// inactive. An aborting supervisor simply keeps calling Update: a close that is
// already committed completes on time (L7 hand policy: ABORT_SAFE continues
// the current phase), and one that is not stays at q_pre.
//
// Pure numeric core: fixed-size, no allocation, noexcept, no ROS. A non-finite
// measurement never produces a transition by itself — ρ is then 0 and the hand
// is not "at" anything — and the timeout still closes the Close phase.
#pragma once

#include "rtc_controllers/catching/catching_params.hpp"  // HandProfile, kMaxHandDof
#include "rtc_controllers/catching/time_types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace rtc::catching {

/// Mirrors CatchingState.msg `HAND_PHASE_*` (dense, same order).
enum class HandPhase : std::uint8_t {
  kOpen = 0,
  kPreshape = 1,
  kClose = 2,
  kHold = 3,
  kRelease = 4
};

/// Everything the sequencer reads, resolved. Built from a validated
/// HandProfile by FromProfile; `Valid()` is the sequencer's own fail-closed
/// check, not a second copy of the validator's ranges.
struct HandSequencerConfig {
  std::array<double, kMaxHandDof> q_open{};
  std::array<double, kMaxHandDof> q_pre{};
  std::array<double, kMaxHandDof> q_close{};
  std::array<bool, kMaxHandDof> caging_mask{};
  int dof{0};
  double eta_close{1.0};
  std::int64_t t_close_e2e_ns{0};
  std::int64_t t_close_timeout_ns{0};
  HandHoldMode hold_mode{HandHoldMode::kCloseTarget};
  double hold_delta_rad{0.0};
  double q_tol{0.01};
  double qd_tol{0.05};

  [[nodiscard]] bool Valid() const noexcept {
    if (dof <= 0 || dof > static_cast<int>(kMaxHandDof)) {
      return false;
    }
    if (!std::isfinite(eta_close) || !(eta_close > 0.0) || eta_close > 1.0) {
      return false;
    }
    if (t_close_e2e_ns < 0 || !(t_close_timeout_ns > t_close_e2e_ns)) {
      return false;
    }
    if (!std::isfinite(q_tol) || !(q_tol > 0.0) || !std::isfinite(qd_tol) || !(qd_tol > 0.0)) {
      return false;
    }
    if (!std::isfinite(hold_delta_rad) || hold_delta_rad < 0.0) {
      return false;
    }
    bool any_caging = false;
    for (int i = 0; i < dof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      if (!std::isfinite(q_open[u]) || !std::isfinite(q_pre[u]) || !std::isfinite(q_close[u])) {
        return false;
      }
      if (caging_mask[u]) {
        // ρ divides by this gap (L6 §4.2) — the validator's rho_eps floor is
        // what makes it safe; here it only has to be non-zero.
        if (!(std::abs(q_close[u] - q_pre[u]) > 0.0)) {
          return false;
        }
        any_caging = true;
      }
    }
    return any_caging;
  }

  /// Copy a hand profile into a config. The TBD-capable fields must already
  /// be resolved (the validator's job); an unresolved one yields a config
  /// that `Valid()` refuses rather than a guessed number.
  [[nodiscard]] static HandSequencerConfig FromProfile(const HandProfile& p) noexcept {
    HandSequencerConfig c{};
    c.q_open = p.q_open;
    c.q_pre = p.q_pre;
    c.q_close = p.q_close;
    c.caging_mask = p.caging_mask;
    c.dof = (p.tbd || p.q_open_tbd) ? 0 : p.dof;
    c.eta_close = p.eta_close.tbd ? std::nan("") : p.eta_close.value;
    const auto to_ns = [](const TbdDouble& v) noexcept -> std::int64_t {
      return (v.tbd || !std::isfinite(v.value))
                 ? -1
                 : static_cast<std::int64_t>(std::llround(v.value * 1e9));
    };
    c.t_close_e2e_ns = to_ns(p.T_close_e2e);
    c.t_close_timeout_ns = to_ns(p.T_close_timeout);
    c.hold_mode = p.hold_mode;
    c.hold_delta_rad = p.hold_delta_rad;
    c.q_tol = p.q_tol;
    c.qd_tol = p.qd_tol;
    return c;
  }
};

/// One tick's answer. `active == false` means "not mine this tick": the
/// caller commands its own latch and ignores every other field.
struct HandSequencerOutput {
  std::array<double, kMaxHandDof> target{};
  int dof{0};
  bool active{false};
  HandPhase phase{HandPhase::kOpen};
  /// L6 §4.2 closure fraction, clamped to [0, 1] for reporting (η ≤ 1, so the
  /// clamp never changes a Close → Hold decision). 0 on a non-finite reading.
  double rho{0.0};
  /// The close command has gone out in this trial (sticky until the next
  /// Ready/Home/Deactivate) — the supervisor's COMMITTED → CLOSING cue.
  bool close_issued{false};
  /// The close went out on THIS tick.
  bool close_issued_now{false};
  /// Settled at `target`: max |q − target| ≤ q_tol and max |q̇| ≤ qd_tol.
  bool at_target{false};
  /// Close ended by T_close_timeout rather than by ρ ≥ η (sticky, like
  /// close_issued). L6 §5.3's timeout flag.
  bool timeout{false};
};

class HandSequencer {
 public:
  /// False (and the sequencer stays inactive) for an invalid config.
  [[nodiscard]] bool Configure(const HandSequencerConfig& cfg) noexcept {
    configured_ = cfg.Valid();
    if (configured_) {
      cfg_ = cfg;
    }
    Deactivate();
    return configured_;
  }

  [[nodiscard]] bool Configured() const noexcept { return configured_; }

  /// Hand the hand back to the caller's latch (E-STOP, activation, a reset
  /// that does not know what the hand is doing).
  void Deactivate() noexcept {
    active_ = false;
    phase_ = HandPhase::kOpen;
    ClearTrial();
  }

  /// Homing: open to q_open (Q4: the one use of q_open).
  void Home() noexcept { Enter(HandPhase::kOpen); }

  /// The arm is at its wait pose: wait at q_pre. Cancels a commit that has not
  /// closed yet.
  void Ready() noexcept { Enter(HandPhase::kPreshape); }

  /// Arm the close for t_cmd = t_c − T_close_e2e. Only from Preshape, and
  /// only once per trial — the catch instant is frozen at COMMITTED, and a
  /// second commit would be a second, different instant. Returns whether the
  /// commit took.
  bool Commit(BallTime t_c) noexcept {
    if (!active_ || phase_ != HandPhase::kPreshape || commit_armed_ || close_issued_) {
      return false;
    }
    t_cmd_ = BallTime{detail::SatSub(t_c.ns, cfg_.t_close_e2e_ns)};
    commit_armed_ = true;
    return true;
  }

  /// Open to q_pre (Q4) and wait there. From any active phase; the Release
  /// ends in Preshape once the hand has settled.
  void Release() noexcept {
    if (!active_) {
      return;
    }
    phase_ = HandPhase::kRelease;
    commit_armed_ = false;
  }

  [[nodiscard]] bool Active() const noexcept { return active_; }

  [[nodiscard]] HandPhase Phase() const noexcept { return phase_; }

  [[nodiscard]] bool CommitArmed() const noexcept { return commit_armed_; }

  [[nodiscard]] bool CloseIssued() const noexcept { return close_issued_; }

  [[nodiscard]] BallTime TCmd() const noexcept { return t_cmd_; }

  [[nodiscard]] NowReal CloseCommandTime() const noexcept { return close_cmd_; }

  /// L6 §4.2's ρ for a measured hand pose (unclamped; NaN on a non-finite
  /// reading). Public so the supervisor's logs and the tests compute it the
  /// same way the transition does.
  [[nodiscard]] double Rho(std::span<const double> q) const noexcept {
    const auto n = static_cast<std::size_t>(cfg_.dof);
    if (!configured_ || q.size() < n) {
      return std::nan("");
    }
    double rho = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      if (!cfg_.caging_mask[i]) {
        continue;
      }
      const double span = cfg_.q_close[i] - cfg_.q_pre[i];
      const double s = span > 0.0 ? 1.0 : -1.0;
      const double r = (q[i] - cfg_.q_pre[i]) * s / std::abs(span);
      if (!std::isfinite(r)) {
        return std::nan("");
      }
      rho = std::min(rho, r);
    }
    return rho;
  }

  /// One tick. `q` / `qd` are the hand's measured positions and velocities
  /// (device order, at least `dof` wide — a narrower span is "not readable"
  /// and makes the hand at nothing). `h_ns` is this tick's period.
  [[nodiscard]] HandSequencerOutput Update(NowReal now, std::int64_t h_ns,
                                           std::span<const double> q,
                                           std::span<const double> qd) noexcept {
    HandSequencerOutput out{};
    if (!active_) {
      return out;
    }
    const auto n = static_cast<std::size_t>(cfg_.dof);
    const bool readable = q.size() >= n && qd.size() >= n;
    const double rho = readable ? Rho(q) : std::nan("");

    // ── Transitions, in phase order. At most one per tick: a phase entered
    // this tick has not been commanded yet, so judging it now would judge a
    // target nobody has seen.
    switch (phase_) {
      case HandPhase::kPreshape:
        if (commit_armed_ && HandCommandDueRounded(now, t_cmd_, h_ns)) {
          phase_ = HandPhase::kClose;
          commit_armed_ = false;
          close_issued_ = true;
          close_cmd_ = now;
          out.close_issued_now = true;
        }
        break;
      case HandPhase::kClose: {
        const bool closed = std::isfinite(rho) && rho >= cfg_.eta_close;
        const bool timed_out = detail::SatSub(now.ns, close_cmd_.ns) >= cfg_.t_close_timeout_ns;
        if (closed || timed_out) {
          timeout_ = timeout_ || (!closed && timed_out);
          EnterHold(readable ? q : std::span<const double>{});
        }
        break;
      }
      case HandPhase::kRelease:
        if (readable && AtTarget(cfg_.q_pre, q, qd)) {
          phase_ = HandPhase::kPreshape;
        }
        break;
      case HandPhase::kOpen:
      case HandPhase::kHold:
        break;
    }

    const std::array<double, kMaxHandDof>& target = Target();
    out.target = target;
    out.dof = cfg_.dof;
    out.active = true;
    out.phase = phase_;
    out.rho = std::isfinite(rho) ? std::clamp(rho, 0.0, 1.0) : 0.0;
    out.close_issued = close_issued_;
    out.at_target = readable && AtTarget(target, q, qd);
    out.timeout = timeout_;
    return out;
  }

 private:
  void Enter(HandPhase phase) noexcept {
    if (!configured_) {
      return;
    }
    active_ = true;
    phase_ = phase;
    ClearTrial();
  }

  void ClearTrial() noexcept {
    commit_armed_ = false;
    close_issued_ = false;
    timeout_ = false;
    t_cmd_ = BallTime{};
    close_cmd_ = NowReal{};
  }

  void EnterHold(std::span<const double> q) noexcept {
    phase_ = HandPhase::kHold;
    hold_target_ = cfg_.q_close;
    if (cfg_.hold_mode != HandHoldMode::kMeasuredOffset || q.empty()) {
      // kCloseTarget — and the offset form without a reading, which has no
      // measured pose to offset from and so falls back to the posture the
      // profile says a closed hand has.
      return;
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(cfg_.dof); ++i) {
      if (!cfg_.caging_mask[i]) {
        continue;
      }
      const double s = (cfg_.q_close[i] - cfg_.q_pre[i]) > 0.0 ? 1.0 : -1.0;
      hold_target_[i] = q[i] + s * cfg_.hold_delta_rad;
    }
  }

  [[nodiscard]] const std::array<double, kMaxHandDof>& Target() const noexcept {
    switch (phase_) {
      case HandPhase::kOpen:
        return cfg_.q_open;
      case HandPhase::kClose:
        return cfg_.q_close;
      case HandPhase::kHold:
        return hold_target_;
      case HandPhase::kPreshape:
      case HandPhase::kRelease:
        break;
    }
    return cfg_.q_pre;
  }

  [[nodiscard]] bool AtTarget(const std::array<double, kMaxHandDof>& target,
                              std::span<const double> q,
                              std::span<const double> qd) const noexcept {
    for (std::size_t i = 0; i < static_cast<std::size_t>(cfg_.dof); ++i) {
      // Written as "within" so a NaN fails it.
      if (!(std::abs(q[i] - target[i]) <= cfg_.q_tol) || !(std::abs(qd[i]) <= cfg_.qd_tol)) {
        return false;
      }
    }
    return true;
  }

  HandSequencerConfig cfg_{};
  bool configured_{false};
  bool active_{false};
  HandPhase phase_{HandPhase::kOpen};
  bool commit_armed_{false};
  bool close_issued_{false};
  bool timeout_{false};
  BallTime t_cmd_{};
  NowReal close_cmd_{};
  std::array<double, kMaxHandDof> hold_target_{};
};

}  // namespace rtc::catching
