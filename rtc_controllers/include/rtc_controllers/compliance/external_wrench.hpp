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
#include <atomic>
#include <cmath>
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
  /// This tick observed a generation the RT side had not seen before — i.e. the
  /// producer really did deliver, as opposed to the RT loop re-reading a value it
  /// already has. Anything that must count SENSOR READINGS rather than CONTROL
  /// TICKS needs this: at 500–5000 Hz RT against a 100–1000 Hz F/T source the two
  /// differ by the rate ratio, so a per-tick count silently averages the same
  /// reading several times over (§3.2.1's N-sample bias average). A ResetTiming()
  /// makes whatever sample is present count as new, matching its contract.
  bool is_new{false};
};

/// Non-RT writer ⇄ RT reader exchange for a 6D external wrench.
///
/// Threading: Set() is the SINGLE writer (SeqLock invariant) and is assumed to
/// run off the RT thread, like SetDeviceTarget. Read() is the RT consumer and
/// must be called at most once per control tick — it advances the age counter.
class WrenchInput {
 public:
  /// Publish a new sample. Non-RT, wait-free, noexcept.
  ///
  /// A sample with ANY non-finite component is DROPPED here, at the entry point,
  /// and counted. Nothing downstream can filter it: the conditioner's deadband
  /// and saturation are comparisons, and every comparison against NaN is false,
  /// so a single garbage reading propagates to the compliant frame, raises
  /// `ComplianceFaults::nan_inf` and LATCHES SAFE_STOP — which `ClearEstop()`
  /// deliberately does not clear, so one bad packet costs a process restart.
  /// Dropping instead leaves the previous sample to age out through the normal
  /// §10.6 staleness path (fade → ZERO → DEGRADED), which is what a sensor
  /// producing garbage should look like.
  void Set(std::span<const double, kWrenchDim> w) noexcept {
    WrenchSample s;
    for (std::size_t i = 0; i < kWrenchDim; ++i) {
      if (!std::isfinite(w[i])) {
        rejected_.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      s.value[i] = w[i];
    }
    // Skip 0 on wrap-around: it is the reserved "never written" stamp.
    if (++next_generation_ == 0)
      next_generation_ = 1;
    s.generation = next_generation_;
    lock_.Store(s);
  }

  /// Number of samples rejected by the finiteness gate above. Saturating-free
  /// wrap is fine — it is a diagnostic counter, read for "is the sensor sending
  /// garbage", not for exact accounting.
  [[nodiscard]] std::uint32_t rejected_samples() const noexcept {
    return rejected_.load(std::memory_order_relaxed);
  }

  /// RT: snapshot the sample and age it. `timeout` (s) is §10.6's
  /// `wrench_timeout`; `fadeout` (s) is `wrench_fadeout_time` — the linear ramp
  /// to ZERO that replaces holding the last value (§10.6 MUST). `fadeout <= 0`
  /// degenerates to an immediate drop to 0, which is legal but discontinuous.
  [[nodiscard]] WrenchRead Read(double dt, double timeout, double fadeout) noexcept {
    const WrenchSample s = lock_.Load();
    WrenchRead r;
    // `disowned_generation_` is the sample Invalidate() saw in the slot; until
    // the producer replaces it, this reads exactly like "nothing has arrived".
    if (s.generation == 0 || (disowned_ && s.generation == disowned_generation_)) {
      ticks_since_change_ = 0;  // nothing has ever arrived — nothing to age
      return r;
    }
    disowned_ = false;
    if (s.generation != last_generation_) {
      last_generation_ = s.generation;
      ticks_since_change_ = 0;
      r.is_new = true;
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

  /// RT: ResetTiming(), and additionally DISOWN whatever sample is sitting in
  /// the slot right now — Read() reports "nothing has arrived" until the
  /// producer publishes a different generation.
  ///
  /// ResetTiming() alone re-dates the sample present at reset to age 0, which
  /// is right when the producer is alive (an activation must not inherit an age
  /// accrued while the controller was not running) and wrong when it is not: a
  /// producer that died during the stop leaves its last reading in the slot, and
  /// re-dating it revives that reading as fresh at the exact moment control
  /// resumes. §10.6's rule is that an expired wrench goes to ZERO and is never
  /// held; a reset that resurrects it is that rule inverted.
  ///
  /// RT-private state only — the SeqLock keeps its single (non-RT) writer.
  void Invalidate() noexcept {
    disowned_generation_ = lock_.Load().generation;
    disowned_ = true;
    ResetTiming();
  }

 private:
  static constexpr std::uint32_t kTickCounterMax = 0xFFFFFFFFu;

  SeqLock<WrenchSample> lock_;
  std::atomic<std::uint32_t> rejected_{0};  ///< writer-incremented, any-thread read
  std::uint32_t next_generation_{0};        ///< writer-private (single writer)
  std::uint32_t last_generation_{0};        ///< RT-private
  std::uint32_t ticks_since_change_{0};     ///< RT-private
  std::uint32_t disowned_generation_{0};    ///< RT-private
  bool disowned_{false};                    ///< RT-private
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
