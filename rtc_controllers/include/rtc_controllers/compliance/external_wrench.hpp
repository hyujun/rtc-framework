// ── External wrench input contract (spec §3.2.1 input side, §10.6 staleness) ─
// The compliance controllers are PURE control algorithms — they own no node, no
// subscription and no message type (issue #236 scope contract, D8/D10). External
// F/T data therefore arrives as an ARGUMENT through a non-RT setter, exactly the
// idiom SetDeviceTarget already uses, and crosses to the RT tick through a
// SeqLock (RT-4). Whoever reads the sensor — an F/T driver, a momentum observer
// (#135), a fingertip estimator — is an outside caller, not a polymorphic
// implementation inside the controller, which is why there is no WrenchSourceBase
// here (D13; ARCH-3 fires on a second CONCRETE implementation, and there is none).
//
// ── Staleness without a clock (D9) ──────────────────────────────────────────
// §10.6 makes the staleness policy a MUST *and* forbids holding the last value
// when it expires. Reading steady_clock on the RT tick to age a sample would be
// a syscall-shaped cost on the hot path, and the producer's own header stamp is
// not trustworthy (clock skew, replayed bags). So freshness is measured in
// RECEIPT events: the writer stamps a monotonically increasing generation, and
// the RT side counts ticks since the generation last changed —
//
//     age = ticks_since_change × state.dt
//
// This makes a re-sent IDENTICAL value fresh (a value-change detector would call
// a genuinely-alive sensor holding a constant force "stale"), needs no clock, and
// keeps the contract meaningful even with the transport layer removed entirely.
//
// The counter advances only on ticks that actually Read(), and Reset() zeroes it
// on (re)activation. Consequence, stated so nobody reads more into `age` than is
// there: a producer that dies while the controller is E-STOPped or inactive is
// detected within one `wrench_timeout` AFTER control resumes, not during the gap.
#pragma once

#include <rtc_base/threading/seqlock.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace rtc::compliance {

/// Wrench ordering is [force; torque] = [fx,fy,fz, tx,ty,tz] — the repo-wide
/// convention (compliance-conventions.md §2). Sign: POSITIVE = the wrench the
/// environment applies ON the robot.
inline constexpr std::size_t kWrenchDim = 6;

using Wrench6 = std::array<double, kWrenchDim>;

/// SeqLock payload: the raw sample plus its receipt stamp. `generation == 0`
/// means "never written" and is never produced by WrenchInput::Set().
struct WrenchSample {
  Wrench6 value{};
  std::uint32_t generation{0};
};

static_assert(std::is_trivially_copyable_v<WrenchSample>, "WrenchSample must be SeqLock-safe");

/// One RT-tick view of the input.
struct WrenchRead {
  Wrench6 value{};    ///< raw sample, in the producer's frame (unconditioned)
  double age{0.0};    ///< s since the last NEW sample (ticks × dt)
  double fade{0.0};   ///< §10.6 fade-out factor: 1 fresh → 0 fully expired
  bool valid{false};  ///< false until the first sample ever arrives
  bool stale{false};  ///< age > timeout (→ ComplianceFaults::wrench_timeout)
};

/// Non-RT writer ⇄ RT reader exchange for a 6D external wrench.
///
/// Threading: Set() is the SINGLE writer (SeqLock invariant) and is assumed to
/// run off the RT thread, like SetDeviceTarget. Read() is the RT consumer and
/// must be called at most once per control tick — it advances the age counter.
class WrenchInput {
 public:
  /// Publish a new sample. Non-RT, wait-free, noexcept.
  void Set(std::span<const double, kWrenchDim> w) noexcept {
    WrenchSample s;
    for (std::size_t i = 0; i < kWrenchDim; ++i)
      s.value[i] = w[i];
    // Skip 0 on wrap-around: it is the reserved "never written" stamp.
    if (++next_generation_ == 0)
      next_generation_ = 1;
    s.generation = next_generation_;
    lock_.Store(s);
  }

  /// RT: snapshot the sample and age it. `timeout` (s) is §10.6's
  /// `wrench_timeout`; `fadeout` (s) is `wrench_fadeout_time` — the linear ramp
  /// to ZERO that replaces holding the last value (§10.6 MUST). `fadeout <= 0`
  /// degenerates to an immediate drop to 0, which is legal but discontinuous.
  [[nodiscard]] WrenchRead Read(double dt, double timeout, double fadeout) noexcept {
    const WrenchSample s = lock_.Load();
    WrenchRead r;
    if (s.generation == 0) {
      ticks_since_change_ = 0;  // nothing has ever arrived — nothing to age
      return r;
    }
    if (s.generation != last_generation_) {
      last_generation_ = s.generation;
      ticks_since_change_ = 0;
    } else if (ticks_since_change_ < kTickCounterMax) {
      ++ticks_since_change_;  // saturating: a long-dead producer must not wrap
    }
    r.value = s.value;
    r.valid = true;
    r.age = static_cast<double>(ticks_since_change_) * (dt > 0.0 ? dt : 0.0);
    r.stale = r.age > timeout;
    if (!r.stale) {
      r.fade = 1.0;
    } else if (fadeout > 0.0) {
      r.fade = std::clamp(1.0 - (r.age - timeout) / fadeout, 0.0, 1.0);
    } else {
      r.fade = 0.0;
    }
    return r;
  }

  /// RT: forget the receipt history so the next Read() treats whatever sample is
  /// present as fresh. Called on (re)activation, where the controller re-seeds
  /// everything else too — an activation must not inherit an age accrued while
  /// the controller was not running.
  void ResetTiming() noexcept {
    last_generation_ = 0;
    ticks_since_change_ = 0;
  }

 private:
  static constexpr std::uint32_t kTickCounterMax = 0xFFFFFFFFu;

  SeqLock<WrenchSample> lock_;
  std::uint32_t next_generation_{0};     ///< writer-private (single writer)
  std::uint32_t last_generation_{0};     ///< RT-private
  std::uint32_t ticks_since_change_{0};  ///< RT-private
};

/// RUNNING_FREE_SPACE ⇄ RUNNING_CONTACT detector (§10.6: "히스테리시스 필수").
/// Separate from the state machine so the hysteresis itself is testable without
/// driving a whole FSM, and so the caller decides WHICH norm feeds it (this
/// controller uses ‖f‖ of the conditioned LWA force).
class ContactHysteresis {
 public:
  /// `enter` is the contact_threshold; `exit` should be strictly below it (the
  /// caller clamps). Equal thresholds degenerate to a plain comparator, which
  /// chatters at the boundary — that is the configuration error the clamp
  /// prevents, not something this class silently repairs.
  bool Update(double magnitude, double enter, double exit) noexcept {
    if (in_contact_)
      in_contact_ = magnitude >= exit;
    else
      in_contact_ = magnitude > enter;
    return in_contact_;
  }

  void Reset() noexcept { in_contact_ = false; }

  [[nodiscard]] bool in_contact() const noexcept { return in_contact_; }

 private:
  bool in_contact_{false};
};

}  // namespace rtc::compliance
