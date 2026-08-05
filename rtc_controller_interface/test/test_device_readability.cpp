// Contract tests for the base F5 device-readability gate (issue #236 S7b).
//
// The contract itself is §3.7 of rtc_controllers/docs/compliance-conventions.md
// and decision B of issue #265. This file pins the parts of it that are
// expressible without a robot: which states the gate refuses, what the refusal
// looks like on the wire, why the OOB bound is NOT a substitute for the gate,
// and how the tail of an over-reporting device is filled.
//
// No URDF fixture here on purpose. The gate is a predicate over counts, so a
// model would only add the serial_6dof vacuity trap (§3.3: every joint is
// +Z-coaxial, so ĝ(q) ≡ 0 and TCP translation is shape-invariant — a gravity
// assertion passes with the gate deleted). Behaviour that genuinely needs FK
// is pinned against a real controller in integrated_bringup.

#include "rtc_controller_interface/device_readability.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

using rtc::CommandType;
using rtc::DeviceOutput;
using rtc::DeviceState;
using rtc::FillCommandTail;
using rtc::FirstHoleSlot;
using rtc::HoldTelemetryAtMeasured;
using rtc::IsDeviceReadable;
using rtc::IsGateClosedByHoles;
using rtc::IsGateClosedByWidth;
using rtc::IsSlotFresh;
using rtc::kMaxDeviceChannels;
using rtc::ModelChannelBound;
using rtc::SelfReportedChannelBound;
using rtc::SilenceDeviceOutput;

namespace {

// A device that reported `n` channels, each holding a distinguishable value.
// Slots past `n` keep DeviceState's zero-initialisation — which is precisely
// the hazard the gate exists for: they read as a perfectly finite 0.0.
DeviceState MakeDevice(int n, bool valid = true) {
  DeviceState dev;
  dev.valid = valid;
  dev.num_channels = n;
  for (int i = 0; i < n && i < static_cast<int>(kMaxDeviceChannels); ++i) {
    dev.positions[static_cast<std::size_t>(i)] = 1.0 + static_cast<double>(i);
  }
  return dev;
}

// The same device, plus a record of which slots this tick's message missed
// (issue #284). Kept apart from MakeDevice so every pre-existing case above
// keeps testing the hole-free default, which is the state every producer that
// predates the field reports.
DeviceState MakeDeviceWithHoles(int n, uint64_t holes) {
  DeviceState dev = MakeDevice(n);
  dev.hole_mask = holes;
  return dev;
}

// ── IsDeviceReadable — the gate ──────────────────────────────────────────────

TEST(IsDeviceReadableTest, RefusesADeviceThatHasNotReported) {
  const DeviceState dev = MakeDevice(0, /*valid=*/false);
  EXPECT_FALSE(IsDeviceReadable(dev, 6));
}

TEST(IsDeviceReadableTest, RefusesADeviceNarrowerThanTheModel) {
  // The only condition that actually fires in production (#265: `valid` is a
  // latching flag and a mid-run dropout stalls the stamp instead) — a startup
  // configuration mismatch.
  EXPECT_FALSE(IsDeviceReadable(MakeDevice(5), 6));
  EXPECT_FALSE(IsDeviceReadable(MakeDevice(1), 6));
}

TEST(IsDeviceReadableTest, AcceptsExactWidthAndOverReporting) {
  // Over-reporting is a NORMAL input, not a fault: num_channels is the wire
  // length and BuildJointStateReorder is built to skip unreferenced names. The
  // gate must not turn a broad state topic into a permanent DEGRADE.
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(6), 6));
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(16), 6));
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(static_cast<int>(kMaxDeviceChannels)), 6));
}

TEST(IsDeviceReadableTest, UnresolvedModelDimDegradesToAValidityCheck) {
  // A controller whose runtime DOF is not resolved yet (unit fixtures that
  // bypass YAML, or the first tick before self-init) knows of nothing missing.
  EXPECT_TRUE(IsDeviceReadable(MakeDevice(3), 0));
  EXPECT_FALSE(IsDeviceReadable(MakeDevice(3, /*valid=*/false), 0));
}

// ── IsGateClosedByWidth — why a closed gate is closed (issue #307) ───────────

TEST(IsGateClosedByWidthTest, IsExactlyTheReportableHalfOfAClosedGate) {
  // The predicate's whole job is to split IsDeviceReadable's false into the
  // cause an operator can act on and the one CM already diagnoses. Pinning it
  // as a partition rather than as two independent cases is what makes it
  // falsifiable: an implementation that reported on `!valid` too, or that
  // dropped the `valid` term, satisfies neither direction below.
  for (const int reported : {0, 1, 5, 6, 7, 16}) {
    for (const bool valid : {false, true}) {
      const DeviceState dev = MakeDevice(reported, valid);
      const bool closed = !IsDeviceReadable(dev, 6);
      const bool reportable = IsGateClosedByWidth(dev, 6);
      EXPECT_TRUE(!reportable || closed) << "reported a gate that is open";
      EXPECT_EQ(reportable, closed && dev.valid) << "the two causes must partition the closure";
    }
  }
}

TEST(IsGateClosedByWidthTest, StaysSilentOnADeviceThatHasNotReported) {
  // §3.7: CM's startup gate refuses to run Compute() until every configured
  // device has reported, so a binding only sees this for a group with no
  // backend at all — which CM's init timeout already names. Reporting it here
  // would duplicate that and fire on every fixture that bypasses YAML.
  EXPECT_FALSE(IsGateClosedByWidth(MakeDevice(0, /*valid=*/false), 6));
  EXPECT_FALSE(IsGateClosedByWidth(MakeDevice(6, /*valid=*/false), 6));
}

TEST(IsGateClosedByWidthTest, StaysSilentWhileTheModelDimIsUnresolved) {
  // The gate itself degrades to a validity check at model_dim <= 0 (nothing is
  // known to be missing), so there is nothing to report either. Without this,
  // every fixture that bypasses YAML would warn on its first tick.
  EXPECT_FALSE(IsGateClosedByWidth(MakeDevice(3), 0));
  EXPECT_FALSE(IsGateClosedByWidth(MakeDevice(0), 0));
}

TEST(IsGateClosedByWidthTest, DoesNotFireOnOverReporting) {
  // Over-reporting is a normal input (see the gate's own test above); a
  // diagnostic that fired on it would warn on correct hardware with a broad
  // state topic.
  EXPECT_FALSE(IsGateClosedByWidth(MakeDevice(16), 6));
  EXPECT_FALSE(IsGateClosedByWidth(MakeDevice(6), 6));
}

// ── Per-slot freshness — the gate's third term (issue #284) ──────────────────
//
// The width test alone was NECESSARY, NOT SUFFICIENT: with an active reorder
// map the slots actually written are the matched reference indices, so a
// message wide enough to pass could still leave holes behind it. These pin the
// term that closed that, including the two boundaries where a mask and an int
// width meet — an unresolved model_dim and a model wider than the capacity.

TEST(IsDeviceReadableTest, RefusesADeviceWithAHoleInsideTheModelWidth) {
  // The shape of the counterexample pinned in integrated_bringup's
  // test_joint_state_reorder: three channels reported, slot 0 never written.
  // Before this term the device passed at model_dim 3 and the control law read
  // a stale sentinel as if it were a measurement.
  DeviceState dev = MakeDevice(3);
  dev.hole_mask = 1ULL << 0;
  EXPECT_FALSE(IsDeviceReadable(dev, 3));
  // The same device is usable by anything that does not reach the holed slot —
  // there is no such width here (model_dim counts from slot 0), so only the
  // degenerate width qualifies.
  EXPECT_TRUE(IsDeviceReadable(dev, 0));
}

TEST(IsDeviceReadableTest, IgnoresHolesOutsideTheModelWidth) {
  // A broad state topic leaves the slots past the model unwritten as a matter
  // of course, and the persistent cache keeps whatever was there. That is not
  // this gate's business: the consumer never indexes them. A mask term applied
  // unbounded would turn every correctly over-reporting device into a
  // permanent degrade — the exact failure the width test is careful to avoid.
  DeviceState dev = MakeDevice(16);
  dev.hole_mask = (1ULL << 6) | (1ULL << 40) | (1ULL << 63);
  EXPECT_TRUE(IsDeviceReadable(dev, 6));
  EXPECT_FALSE(IsGateClosedByHoles(dev, 6));
  // One slot lower and the same mask does close it — the boundary is exclusive.
  EXPECT_FALSE(IsDeviceReadable(dev, 7));
}

TEST(IsDeviceReadableTest, AHoleFreeMaskIsTheDefaultAndChangesNothing) {
  // Polarity check, and the reason this term could be folded into the existing
  // predicate instead of shipped as a second one: DeviceState is an aggregate
  // built with {} all over the repository and by every producer that predates
  // the field. Zero must mean "no claim", not "nothing is fresh".
  const DeviceState untouched{};
  EXPECT_EQ(0U, untouched.hole_mask);
  for (const int reported : {0, 1, 5, 6, 7, 16}) {
    for (const bool valid : {false, true}) {
      const DeviceState dev = MakeDevice(reported, valid);
      EXPECT_EQ(valid && reported >= 6, IsDeviceReadable(dev, 6))
          << "a device that fills no mask must be judged exactly as before";
    }
  }
}

TEST(IsDeviceReadableTest, UnresolvedModelDimIgnoresTheMaskToo) {
  // The model_dim <= 0 degrade has to survive the new term, or every fixture
  // that bypasses YAML starts failing the moment a backend reports holes.
  DeviceState dev = MakeDevice(3);
  dev.hole_mask = ~0ULL;
  EXPECT_TRUE(IsDeviceReadable(dev, 0));
  EXPECT_TRUE(IsDeviceReadable(dev, -1));
  EXPECT_FALSE(IsGateClosedByHoles(dev, 0));
}

TEST(IsDeviceReadableTest, AModelWiderThanTheCapacitySaturatesRatherThanShifting) {
  // 1ULL << 64 is undefined behaviour and 64 is the DEFAULT capacity here, not
  // a corner. The saturating branch also has to keep the verdict honest: a
  // model wider than the device can ever report is unreadable on the width
  // term alone, so the mask must not accidentally rescue it.
  DeviceState dev = MakeDevice(static_cast<int>(kMaxDeviceChannels));
  EXPECT_TRUE(IsDeviceReadable(dev, static_cast<int>(kMaxDeviceChannels)));
  EXPECT_FALSE(IsDeviceReadable(dev, static_cast<int>(kMaxDeviceChannels) + 1));
  dev.hole_mask = 1ULL << (kMaxDeviceChannels - 1);
  EXPECT_FALSE(IsDeviceReadable(dev, static_cast<int>(kMaxDeviceChannels)));
}

TEST(SlotMaskBelowTest, SaturatesAtBothEnds) {
  EXPECT_EQ(0U, rtc::SlotMaskBelow(0));
  EXPECT_EQ(0U, rtc::SlotMaskBelow(-5));
  EXPECT_EQ(0b1U, rtc::SlotMaskBelow(1));
  EXPECT_EQ(0b111111U, rtc::SlotMaskBelow(6));
  EXPECT_EQ(~0ULL, rtc::SlotMaskBelow(static_cast<int>(kMaxDeviceChannels)));
  EXPECT_EQ(~0ULL, rtc::SlotMaskBelow(static_cast<int>(kMaxDeviceChannels) + 1000));
}

// ── IsGateClosedByHoles / FirstHoleSlot — the third diagnosis (issue #284) ───

TEST(IsGateClosedByHolesTest, ThreeCausesPartitionAClosedGate) {
  // Same partition property the width diagnostic is pinned by, extended to the
  // cause this issue added. A closed gate with NO diagnosis is the failure
  // #307 exists to prevent, and adding a term to the gate without a matching
  // diagnostic would have reintroduced it silently.
  for (const int reported : {0, 3, 6, 16}) {
    for (const bool valid : {false, true}) {
      for (const uint64_t holes : {uint64_t{0}, uint64_t{1} << 2, ~uint64_t{0}}) {
        DeviceState dev = MakeDevice(reported, valid);
        dev.hole_mask = holes;
        const bool closed = !IsDeviceReadable(dev, 6);
        const int diagnosed = static_cast<int>(IsGateClosedByWidth(dev, 6)) +
                              static_cast<int>(IsGateClosedByHoles(dev, 6));
        EXPECT_LE(diagnosed, 1) << "the two reportable causes must be mutually exclusive";
        // An open gate is never diagnosed; a closed one is diagnosed unless the
        // cause is the unreported device, which CM names instead.
        EXPECT_EQ(closed && dev.valid, diagnosed == 1)
            << "reported=" << reported << " valid=" << valid << " holes=" << holes;
      }
    }
  }
}

TEST(FirstHoleSlotTest, NamesTheLowestHoleTheModelActuallyReaches) {
  DeviceState dev = MakeDevice(16);
  dev.hole_mask = (1ULL << 2) | (1ULL << 4);
  // Lowest, not any — the operator repairs one declared name at a time.
  EXPECT_EQ(2, FirstHoleSlot(dev, 6));
  // Bounded to the model: a hole past the width is not this gate's business,
  // so reporting it would send the operator after a name that is working.
  EXPECT_EQ(-1, FirstHoleSlot(dev, 2));
  EXPECT_EQ(4, FirstHoleSlot(MakeDeviceWithHoles(16, 1ULL << 4), 6));
}

TEST(FirstHoleSlotTest, ReturnsMinusOneWhenThereIsNothingToName) {
  EXPECT_EQ(-1, FirstHoleSlot(MakeDevice(6), 6));
  EXPECT_EQ(-1, FirstHoleSlot(MakeDeviceWithHoles(6, ~0ULL), 0));
}

// ── IsSlotFresh — per-slot, for readers that need specific channels ──────────

TEST(IsSlotFreshTest, AnswersPerSlotRatherThanPerPrefix) {
  // The whole reason this exists: a reader that wants slot 7 must not be
  // refused by a hole at slot 3, and must not be served a hole at slot 7 just
  // because the slots below it are fine. IsDeviceReadable cannot express either
  // — it is a prefix question.
  const DeviceState dev = MakeDeviceWithHoles(16, 1ULL << 3);
  EXPECT_FALSE(IsSlotFresh(dev, 3));
  EXPECT_TRUE(IsSlotFresh(dev, 7));
  EXPECT_TRUE(IsSlotFresh(dev, 0));
  // The prefix predicate refuses slot 7's reader here, which is the over-refusal
  // the per-slot form avoids.
  EXPECT_FALSE(IsDeviceReadable(dev, 8));
}

TEST(IsSlotFreshTest, RefusesUnreportedDevicesAndOutOfRangeSlots) {
  EXPECT_FALSE(IsSlotFresh(MakeDevice(16, /*valid=*/false), 3));
  // Past what the device reported — the mask says nothing about slots the
  // device does not have, so the honest answer is "not fresh", not "no hole".
  EXPECT_FALSE(IsSlotFresh(MakeDevice(6), 6));
  EXPECT_FALSE(IsSlotFresh(MakeDevice(6), 99));
  EXPECT_FALSE(IsSlotFresh(MakeDevice(6), -1));
  // The shift must not run at or past the mask's width even if a device somehow
  // reports more channels than the capacity.
  DeviceState wide = MakeDevice(static_cast<int>(kMaxDeviceChannels));
  wide.num_channels = static_cast<int>(kMaxDeviceChannels) + 8;
  EXPECT_FALSE(IsSlotFresh(wide, static_cast<int>(kMaxDeviceChannels)));
  EXPECT_TRUE(IsSlotFresh(wide, static_cast<int>(kMaxDeviceChannels) - 1));
}

TEST(IsSlotFreshTest, AgreesWithTheGateOnEverySlotOfAPrefix) {
  // The two predicates are different questions, but they must not contradict:
  // a readable prefix means every slot in it is fresh, and a slot the gate's
  // width covers is holed exactly when the gate's mask term says so.
  for (const uint64_t holes : {uint64_t{0}, uint64_t{1} << 2, uint64_t{1} << 5, ~uint64_t{0}}) {
    const DeviceState dev = MakeDeviceWithHoles(8, holes);
    bool all_fresh = true;
    for (int i = 0; i < 8; ++i) {
      all_fresh = all_fresh && IsSlotFresh(dev, i);
    }
    EXPECT_EQ(IsDeviceReadable(dev, 8), all_fresh) << "holes=" << holes;
  }
}

// ── SelfReportedChannelBound — the width a LATCHING caller may adopt ─────────

TEST(SelfReportedChannelBoundTest, GivesTheOldAnswerOnAHoleFreeDevice) {
  // Every producer that predates hole_mask and every fixture that leaves the
  // mask zero must be unaffected — this is why adopting the helper at the six
  // self-init sites changed no existing test.
  EXPECT_EQ(6, SelfReportedChannelBound(MakeDevice(6), 32));
  EXPECT_EQ(32, SelfReportedChannelBound(MakeDevice(40), 32));
  EXPECT_EQ(4, SelfReportedChannelBound(MakeDevice(4), 32));
  EXPECT_EQ(0, SelfReportedChannelBound(MakeDevice(0), 32));
}

TEST(SelfReportedChannelBoundTest, TrimsToTheHoleFreePrefix) {
  // The wire says 22 channels, but the reorder map only ever wrote 16 of them.
  // Adopting 22 is what silences the device from the next tick onward.
  EXPECT_EQ(16, SelfReportedChannelBound(MakeDeviceWithHoles(22, ~((1ULL << 16) - 1)), 32));
  // The prefix ends at the LOWEST hole, not the highest written slot.
  EXPECT_EQ(3, SelfReportedChannelBound(MakeDeviceWithHoles(22, (1ULL << 3) | (1ULL << 9)), 32));
  // A hole at slot 0 leaves nothing adoptable; the gate then degrades to the
  // plain validity check, which is the honest answer while nothing is known.
  EXPECT_EQ(0, SelfReportedChannelBound(MakeDeviceWithHoles(22, 1ULL << 0), 32));
}

TEST(SelfReportedChannelBoundTest, IgnoresHolesTheCapAlreadyExcludes) {
  // A hole at slot 40 is not a 32-DOF caller's business — trimming for it would
  // hand back a narrower DOF than the device can actually serve.
  EXPECT_EQ(32, SelfReportedChannelBound(MakeDeviceWithHoles(40, 1ULL << 40), 32));
  // Nor is a hole past what the device reported at all.
  EXPECT_EQ(6, SelfReportedChannelBound(MakeDeviceWithHoles(6, 1ULL << 9), 32));
}

TEST(SelfReportedChannelBoundTest, TheAdoptedWidthAlwaysPassesTheGate) {
  // THE PROPERTY THE CALL SITES REST ON. Their comments assert that a DOF
  // resolved from the device's own report "can never re-open the gate this
  // branch just passed" — true of the width term alone, and false of the hole
  // term, which is what this helper repairs. Those branches LATCH, so a width
  // the gate refuses is permanent silence plus a seed frozen on unwritten slots.
  for (const int reported : {0, 1, 6, 22, 40, static_cast<int>(kMaxDeviceChannels)}) {
    for (const uint64_t holes : {uint64_t{0}, uint64_t{1}, uint64_t{1} << 3, uint64_t{1} << 21,
                                 ~uint64_t{0}, uint64_t{0xDEADBEEF}}) {
      for (const int cap : {1, 6, 32, static_cast<int>(kMaxDeviceChannels)}) {
        const DeviceState dev = MakeDeviceWithHoles(reported, holes);
        const int adopted = SelfReportedChannelBound(dev, cap);
        EXPECT_TRUE(IsDeviceReadable(dev, adopted))
            << "reported=" << reported << " holes=" << holes << " cap=" << cap;
        EXPECT_GE(adopted, 0);
        EXPECT_LE(adopted, std::min(reported, cap));
      }
    }
  }
}

TEST(SelfReportedChannelBoundTest, StillReportsAWidthForADeviceThatHasNotReported) {
  // Deliberately NOT gated on `valid`: the callers check readability themselves
  // before adopting, and folding validity in here would return 0 for a device
  // that simply has not arrived yet — indistinguishable from one whose slot 0
  // is holed.
  EXPECT_EQ(6, SelfReportedChannelBound(MakeDevice(6, /*valid=*/false), 32));
}

// ── ModelChannelBound — OOB defence only ─────────────────────────────────────

TEST(ModelChannelBoundTest, ClampsAnOverReportingDeviceToTheModelWidth) {
  EXPECT_EQ(ModelChannelBound(16, 6), 6);
  EXPECT_EQ(ModelChannelBound(6, 6), 6);
}

TEST(ModelChannelBoundTest, ClampsToWhatTheDeviceActuallyReported) {
  EXPECT_EQ(ModelChannelBound(4, 6), 4);
}

TEST(ModelChannelBoundTest, ClampsNegativeInputsToZero) {
  // The result is used as a loop bound and as a std::span length, where a
  // negative int would convert to an enormous std::size_t.
  EXPECT_EQ(ModelChannelBound(-1, 6), 0);
  EXPECT_EQ(ModelChannelBound(6, -1), 0);
}

TEST(ModelChannelBoundTest, BoundsAFixedWidthBufferAgainstAFullWidthDevice) {
  // The J5/T8 shape: a 32-wide safe_position_ indexed by a device that may
  // report up to kMaxDeviceChannels (64). Without the bound this is an
  // out-of-range read on every channel past 32.
  constexpr int kFixedWidth = 32;
  const int nc0 = static_cast<int>(kMaxDeviceChannels);
  ASSERT_GT(nc0, kFixedWidth) << "fixture is vacuous unless the device can out-report the buffer";

  std::array<double, kFixedWidth> safe_position{};
  const int n = ModelChannelBound(nc0, kFixedWidth);
  EXPECT_EQ(n, kFixedWidth);
  EXPECT_LE(static_cast<std::size_t>(n), safe_position.size());
}

// ── Role separation — decision B pinned in code ──────────────────────────────

TEST(GateVsBoundTest, BoundAloneLeavesAPartiallyZeroConfiguration) {
  // The claim decision B rests on, made falsifiable: for a device NARROWER
  // than the model, applying the bound to a scatter into a PERSISTENT buffer
  // produces byte-for-byte the same configuration as reading the unreported
  // channels outright. The bound removed the crash and kept the hazard.
  constexpr int kNv = 6;
  const DeviceState dev = MakeDevice(4);

  // What ExtractFullState / CopyToEigen do: scatter min(nc0, nv) entries into a
  // buffer that persists across ticks (zero on the first one).
  std::array<double, kNv> bounded{};
  const int n = ModelChannelBound(dev.num_channels, kNv);
  ASSERT_EQ(n, 4);
  for (int i = 0; i < n; ++i) {
    bounded[static_cast<std::size_t>(i)] = dev.positions[static_cast<std::size_t>(i)];
  }

  // What an unbounded read would have produced: DeviceState::positions is
  // zero-filled past num_channels, so the unreported joints read as 0.0.
  std::array<double, kNv> unbounded{};
  for (int i = 0; i < kNv; ++i) {
    unbounded[static_cast<std::size_t>(i)] = dev.positions[static_cast<std::size_t>(i)];
  }

  EXPECT_EQ(bounded, unbounded) << "if these ever differ, the bound really did remove the hazard "
                                   "and decision B needs revisiting";
  EXPECT_EQ(bounded[4], 0.0);
  EXPECT_EQ(bounded[5], 0.0);

  // ...and the gate is the only thing in this header that refuses that state.
  EXPECT_FALSE(IsDeviceReadable(dev, kNv));
}

TEST(GateVsBoundTest, TheTwoPredicatesDisagreeOnExactlyTheDangerousState) {
  // Misusing the bound AS a gate reads as "nothing was truncated, so we are
  // fine". That test passes on the one state the gate refuses, which is why
  // the two must not share a name.
  constexpr int kNv = 6;

  const DeviceState narrow = MakeDevice(4);
  EXPECT_EQ(ModelChannelBound(narrow.num_channels, kNv), narrow.num_channels)
      << "bound is a no-op here — it truncates nothing and reports nothing wrong";
  EXPECT_FALSE(IsDeviceReadable(narrow, kNv));

  const DeviceState wide = MakeDevice(16);
  EXPECT_LT(ModelChannelBound(wide.num_channels, kNv), wide.num_channels)
      << "bound DOES truncate here — the case it is actually for";
  EXPECT_TRUE(IsDeviceReadable(wide, kNv)) << "and this state is perfectly usable";
}

// ── SilenceDeviceOutput — the answer to a false gate ─────────────────────────

TEST(SilenceDeviceOutputTest, ZeroLengthIsDistinctFromAZeroCommand) {
  DeviceOutput out;
  out.num_channels = 6;
  for (std::size_t i = 0; i < 6; ++i) {
    out.commands[i] = 0.0;  // the REJECTED alternative: nc0 real zeros
  }

  SilenceDeviceOutput(out);

  // A backend loops to min(num_channels, values.size()), so zero-length is
  // "no update" while six zeros are six real 0 N·m / 0 rad commands. The two
  // are only distinguishable by this count.
  EXPECT_EQ(out.num_channels, 0);
}

TEST(SilenceDeviceOutputTest, TouchesOnlyTheDeviceItIsGiven) {
  // §3.7 secondary passthrough: a hand does not stop being commandable because
  // the arm's state went missing.
  rtc::ControllerOutput output;
  output.num_devices = 2;
  output.devices[0].num_channels = 6;
  output.devices[1].num_channels = 10;
  for (std::size_t i = 0; i < 10; ++i) {
    output.devices[1].commands[i] = 0.5 + static_cast<double>(i);
  }

  SilenceDeviceOutput(output.devices[0]);

  EXPECT_EQ(output.devices[0].num_channels, 0);
  EXPECT_EQ(output.devices[1].num_channels, 10);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(output.devices[1].commands[i], 0.5 + static_cast<double>(i));
  }
  EXPECT_EQ(output.num_devices, 2) << "the device count still describes the state, not the gate";
}

// ── HoldTelemetryAtMeasured — what a silenced tick LOOKS like ────────────────
//
// Silencing the wire does not silence the log: the device state POD copies the
// output's position-semantics arrays bounded by the DEVICE's channel count, not
// the output's, and carries no field for the emitted width. So the reference
// lanes of a silenced tick are the only thing standing between "no command" and
// a recorded row that reads as "commanded every joint to the origin".

TEST(HoldTelemetryAtMeasuredTest, ReferenceLanesReportTheParkedPosition) {
  DeviceOutput out;
  const DeviceState dev = MakeDevice(6);
  SilenceDeviceOutput(out);

  HoldTelemetryAtMeasured(out, dev.num_channels, dev.positions);

  EXPECT_EQ(out.num_channels, 0) << "the wire stays silent";
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(out.target_positions[i], dev.positions[i]) << "channel " << i;
    EXPECT_DOUBLE_EQ(out.trajectory_positions[i], dev.positions[i]) << "channel " << i;
    EXPECT_NE(out.trajectory_positions[i], 0.0)
        << "a row of zeros is the reading this function exists to prevent";
  }
}

TEST(HoldTelemetryAtMeasuredTest, LeavesTheGoalAndTheCommandsToTheBinding) {
  DeviceOutput out;
  const DeviceState dev = MakeDevice(6);
  for (std::size_t i = 0; i < 6; ++i) {
    out.goal_positions[i] = 7.0 + static_cast<double>(i);
  }
  SilenceDeviceOutput(out);

  HoldTelemetryAtMeasured(out, dev.num_channels, dev.positions);

  for (std::size_t i = 0; i < 6; ++i) {
    // What a goal MEANS is per-binding (a joint hold target, a TCP pose split
    // across the first three slots), so this primitive must not overwrite it.
    EXPECT_DOUBLE_EQ(out.goal_positions[i], 7.0 + static_cast<double>(i)) << "channel " << i;
    // A parked joint is not moving, and nothing was commanded.
    EXPECT_DOUBLE_EQ(out.target_velocities[i], 0.0) << "channel " << i;
    EXPECT_DOUBLE_EQ(out.commands[i], 0.0) << "channel " << i;
  }
}

TEST(HoldTelemetryAtMeasuredTest, StopsAtTheShorterOfTheTwoSpans) {
  DeviceOutput out;
  std::array<double, 3> measured{1.5, 2.5, 3.5};

  // num_channels claims more than `measured` covers: the extra channels have no
  // measurement to park at, so they keep their fresh zero rather than reading
  // past the span.
  HoldTelemetryAtMeasured(out, 9, measured);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(out.trajectory_positions[i], measured[i]);
  }
  for (std::size_t i = 3; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(out.trajectory_positions[i], 0.0) << "channel " << i;
  }
}

TEST(HoldTelemetryAtMeasuredTest, IgnoresANegativeChannelCount) {
  DeviceOutput out;
  const DeviceState dev = MakeDevice(6);

  // A negative int converted to std::size_t would be enormous.
  HoldTelemetryAtMeasured(out, -4, dev.positions);

  EXPECT_DOUBLE_EQ(out.trajectory_positions[0], 0.0);
  EXPECT_DOUBLE_EQ(out.target_positions[0], 0.0);
}

// ── FillCommandTail — the [model_bound, nc0) domain split ────────────────────

// Every tail fixture pre-fills with a sentinel: a zero-initialised array would
// satisfy the torque assertion with the function deleted.
constexpr double kSentinel = -12345.0;

std::array<double, kMaxDeviceChannels> SentinelCommands() {
  std::array<double, kMaxDeviceChannels> commands;
  commands.fill(kSentinel);
  return commands;
}

TEST(FillCommandTailTest, TorqueTailIsZero) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(9);

  FillCommandTail(commands, /*model_bound=*/6, /*num_channels=*/9, CommandType::kTorque,
                  dev.positions);

  EXPECT_DOUBLE_EQ(commands[5], kSentinel) << "the modelled channels are not the tail's business";
  EXPECT_DOUBLE_EQ(commands[6], 0.0);
  EXPECT_DOUBLE_EQ(commands[7], 0.0);
  EXPECT_DOUBLE_EQ(commands[8], 0.0);
  EXPECT_DOUBLE_EQ(commands[9], kSentinel) << "and neither is anything past what was reported";
}

TEST(FillCommandTailTest, PositionTailHoldsTheMeasuredValue) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(9);

  FillCommandTail(commands, /*model_bound=*/6, /*num_channels=*/9, CommandType::kPosition,
                  dev.positions);

  // 0.0 here would be "servo to the origin" on every channel the model does
  // not cover — the failure this split exists to prevent.
  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[7], 8.0);
  EXPECT_DOUBLE_EQ(commands[8], 9.0);
  EXPECT_NE(commands[6], 0.0);
}

TEST(FillCommandTailTest, PdFeedforwardTailHoldsTheMeasuredValue) {
  // kPdFeedforward is a position-servo backbone with a torque overlay, so its
  // `commands` lane follows the position rule, not the torque one.
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(9);

  FillCommandTail(commands, 6, 9, CommandType::kPdFeedforward, dev.positions);

  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[8], 9.0);
}

TEST(FillCommandTailTest, FallsBackToZeroWhereEvenTheMeasurementIsMissing) {
  auto commands = SentinelCommands();
  const std::vector<double> short_measurement{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  FillCommandTail(commands, 6, 9, CommandType::kPosition, short_measurement);

  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[7], 0.0);
  EXPECT_DOUBLE_EQ(commands[8], 0.0);
}

TEST(FillCommandTailTest, NeverWritesPastTheCommandSpan) {
  std::array<double, 8> commands;
  commands.fill(kSentinel);
  const DeviceState dev = MakeDevice(static_cast<int>(kMaxDeviceChannels));

  FillCommandTail(commands, 6, static_cast<int>(kMaxDeviceChannels), CommandType::kPosition,
                  dev.positions);

  EXPECT_DOUBLE_EQ(commands[6], 7.0);
  EXPECT_DOUBLE_EQ(commands[7], 8.0);
}

TEST(FillCommandTailTest, IsEmptyWhenTheModelCoversEveryReportedChannel) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(6);

  FillCommandTail(commands, ModelChannelBound(dev.num_channels, 6), dev.num_channels,
                  CommandType::kPosition, dev.positions);

  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_DOUBLE_EQ(commands[i], kSentinel) << "channel " << i;
  }
}

TEST(FillCommandTailTest, IgnoresNegativeBounds) {
  auto commands = SentinelCommands();
  const DeviceState dev = MakeDevice(3);

  FillCommandTail(commands, -4, -1, CommandType::kPosition, dev.positions);

  EXPECT_DOUBLE_EQ(commands[0], kSentinel);
}

}  // namespace
