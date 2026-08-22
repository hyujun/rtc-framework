// ── ComplianceWrenchSource: which physical estimate drives the admittance law ─
//
// The compliance core takes its external wrench through one non-RT setter and
// deliberately does not care where it came from (compliance-conventions.md §3.1:
// "컨트롤러는 노드·구독·메시지 타입을 만들지 않는다" — the transport layer is
// outside rtc_controllers). This header is the binding-side half of that split:
// it turns one concrete estimate into the exact tuple WrenchPipeline wants, and
// it lives in integrated_bringup precisely so rtc_controllers does not have to
// learn about grasps.
//
// WHY A FREE FUNCTION AND A POD, NOT A `WrenchSourceBase`. That abstraction was
// examined and explicitly deferred (compliance-conventions.md §5, D13): ARCH-3
// fires at the second concrete implementation *inside* the controller, and a
// wrench source is not one — it is an outside party calling a setter. A
// second adapter (momentum observer, #469 S6) would still be a second free
// function over a different input type, sharing nothing but this return POD, so
// a virtual `Wrench6 Get()` would be a degenerate interface over two functions
// that never dispatch. The verdict struct is the real interface.
//
// RT: FromPullEstimate is called from the controller tick — noexcept, no
// allocation, no lock, no logging, no throw, fixed-size Eigen only. The one
// exception is ParseComplianceWrenchSource, which runs at configure (it takes a
// string and throws); it lives here anyway because an enum and the spelling that
// selects it are one contract, and splitting them is how a value gets renamed on
// one side only.
#pragma once

#include "integrated_bringup/support/pull_estimator_wiring.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace integrated_bringup {

/// Which physical estimate feeds the compliance law. Configure-time, exclusive:
/// one source per controller instance, never a fusion (#469 D-A4, which is why
/// #470's "two or more 6D sources combined" trigger does not fire here).
///
/// `kMomentumObserver` is deliberately ABSENT rather than declared-and-rejected.
/// An enum value with no implementation turns a config typo into a runtime
/// throw that reads as a bug in the controller; a value that simply does not
/// exist is rejected by the same parser path as any other misspelling, which is
/// what an operator can act on. It is added when its adapter is (#469 S6).
///
/// Values are wire constants (they reach the CSV diagnostic lane) — never
/// renumber.
enum class ComplianceWrenchSource : std::uint8_t {
  kPullEstimator = 0,
};

/// The YAML spelling of `kPullEstimator`. Kept next to the enum so the wire
/// name and the value cannot be renamed apart.
inline constexpr std::string_view kPullEstimatorSourceName = "pull_estimator";

/// Resolve `external_wrench.source` (configure-time, non-RT, throws).
///
/// FAIL-CLOSED BY OMISSION. There is no default and no fallback: a wrench source
/// that silently resolved to something would let a typo run the arm off an
/// estimate the operator did not select, and the compliance controller exists to
/// be driven by one specific measurement. The throw reaches CM as a configure
/// FAILURE, which refuses the bring-up — the same answer any other malformed
/// controller YAML gets.
///
/// The message names the accepted values rather than saying "invalid", because
/// the interesting failure is `momentum_observer`: it is a source that exists in
/// the design (#469 S6) and does not exist yet in code, so the operator needs to
/// be told which of the two situations they are in.
[[nodiscard]] inline ComplianceWrenchSource ParseComplianceWrenchSource(const std::string& name) {
  if (name == kPullEstimatorSourceName) {
    return ComplianceWrenchSource::kPullEstimator;
  }
  throw std::runtime_error("external_wrench.source: unknown value '" + name + "'. Accepted: '" +
                           std::string(kPullEstimatorSourceName) +
                           "'. (A momentum-observer source is designed but not implemented — "
                           "#469 S6.)");
}

/// One tick's answer from a wrench source: what to publish, where it acts, and
/// what the state machine should make of it.
///
/// Deliberately plain data. The tick needs all of these together and the
/// decisions behind them are cheap; returning them as one value keeps the
/// "should I publish?" question from being answered twice in slightly different
/// ways at the two call sites that need it.
struct WrenchSourceVerdict {
  /// [f; τ] = [fx,fy,fz, tx,ty,tz], SI, base-aligned. Handed to
  /// `WrenchPipeline::Publish` verbatim, so the caller passes
  /// `R_world_sensor = I` — this vector is ALREADY in the reference (robot
  /// base) frame and no rotation is owed anywhere on this path.
  rtc::compliance::Wrench6 wrench{};
  /// Where the wrench acts, world frame → `WrenchPipeline::Update`'s
  /// `p_sensor`. With the virtual TCP also serving as the task frame the lever
  /// arm is zero and `Ad^{-T}` is the identity, so no moment is invented; when
  /// they differ, `(p_sensor − p_task) × f` is the physically correct transport.
  Eigen::Vector3d p_apply{Eigen::Vector3d::Zero()};
  /// False ⇒ the caller must NOT call `Publish` this tick. Withholding is the
  /// mechanism, not an oversight: see the note on staleness below.
  bool publish{false};
  /// Feed to `ComplianceFaults::quality_low` (DEGRADED, recoverable — never
  /// SAFE_STOP, compliance-conventions.md §3.2).
  bool quality_low{false};
  /// `rtc::grasp::PullInvalidReason` as a wire constant; 0 iff the estimator
  /// called the tick valid. Diagnostic only — the CSV lane (#469 S4) is what
  /// separates "the geometry went degenerate" from "the thumb dropped out".
  std::uint8_t reason{0};
};

/// Pack the in-plane pull estimate as a compliance wrench (#469 D-A2).
///
/// THE FORCE HALF IS `force_filtered` VERBATIM. It is already a robot-base
/// frame vector (pull_estimator_wiring.hpp: "reference (robot base) frame"), so
/// no rotation happens here and none is owed downstream. The two scalar
/// alternatives were rejected: `magnitude` has no sign, so any pull direction
/// would push the same way and nothing would ever come back; `force_inplane[1]`
/// rides a basis whose `e_y` rotates with the observed pinch geometry every
/// tick, so integrating it against a fixed base axis mixes frames — finitely,
/// smoothly and wrongly.
///
/// THE TORQUE HALF IS IDENTICALLY ZERO, and that is a measurement, not a
/// placeholder: the estimator projects onto the pinch plane and produces no
/// moment at all. Synthesising one from the force and a lever arm here would
/// duplicate — with a worse lever arm — the `Ad^{-T}` transport the pipeline
/// already does from `p_apply`.
///
/// SIGN. `F̂ = −P∥(Σ finger-on-object)`, and quasi-static Newton on the grasped
/// object makes that the in-plane part of what the environment pulls with. That
/// is the compliance convention ("환경이 로봇에 가하는 힘이 양수",
/// compliance-conventions.md §3.1) unchanged, so nothing is negated here. This
/// exact spot was wrong once — the 2026-07-22 p1b hardware run, where an
/// inverted default flipped every contact gate and a solid grasp published an
/// all-zero estimate — which is why the sign is pinned end to end against a
/// known load rather than argued for (#177 crit#4).
///
/// WITHHOLDING IS HOW STALENESS WORKS (#469 D-A7). `publish == false` is not an
/// error path: the pipeline ages its last sample, fades it towards zero and
/// raises `wrench_timeout` (§10.6, which forbids holding the last value). If
/// this returned a zero wrench on bad ticks instead, that would be a *fresh*
/// sample of zero — the age would never grow, staleness would never be
/// reported, and the bias-calibration gate counts new samples rather than
/// ticks, so it would be fed duplicates too.
[[nodiscard]] inline WrenchSourceVerdict FromPullEstimate(const PullEstimatorWiring& w,
                                                          const VirtualTcpResult& vtcp) noexcept {
  WrenchSourceVerdict verdict;

  // A disabled wiring owns no estimator (the YAML block was absent, or the hand
  // has no fingertip links). There is nothing to publish and nothing to fault
  // on — a controller configured without this source must not spend its life in
  // DEGRADED because of it.
  if (!w.enabled()) {
    return verdict;
  }

  const rtc::grasp::PullEstimate& est = w.estimator->estimate();
  verdict.reason = static_cast<std::uint8_t>(est.invalid_reason);

  verdict.wrench[0] = est.force_filtered.x();
  verdict.wrench[1] = est.force_filtered.y();
  verdict.wrench[2] = est.force_filtered.z();
  // wrench[3..5] stay zero — see the torque note above.

  verdict.p_apply = vtcp.world_pose.translation();

  // Finiteness is tested explicitly, never inferred from the gates around it
  // (NUM-7). Every comparison below is false for NaN, so a non-finite estimate
  // would pass `valid`, pass the leakage test, and reach `WrenchInput::Set`,
  // whose own guard drops it — leaving a rejected-sample counter as the only
  // trace while the arm quietly runs on a fading stale wrench. Cheaper to not
  // send it. `vtcp.valid` is checked as well as the numbers: an invalid virtual
  // TCP resolves to the identity, whose origin is a perfectly finite and
  // completely wrong place to hang a force.
  const bool numbers_usable =
      est.force_filtered.allFinite() && vtcp.valid && verdict.p_apply.allFinite();

  verdict.publish = est.valid && numbers_usable;

  if (verdict.publish) {
    // `leakage_bound` is Σ|f_n,i|·sin(δθ) — an upper bound on how much of the
    // grip could be masquerading as pull through axis misalignment. Comparing
    // it to the estimate itself rather than to a tuned threshold keeps a knob
    // out of the config and states the real condition: once the bound reaches
    // the signal, the estimate could be entirely leakage and not even its
    // direction survives.
    verdict.quality_low =
        est.slip_risk || est.any_saturated || (est.leakage_bound >= est.magnitude);
  } else {
    // The estimator called this tick valid and we withheld it anyway. Say so
    // now instead of letting it surface as a timeout `wrench_timeout` seconds
    // later — and note the signature this leaves in the diagnostic lane:
    // quality_low with `reason == 0` is exactly "the pull was fine, the numbers
    // or the application point were not", which no ordinary invalid tick can
    // produce because those all carry a non-zero reason.
    verdict.quality_low = est.valid;
  }

  return verdict;
}

}  // namespace integrated_bringup
