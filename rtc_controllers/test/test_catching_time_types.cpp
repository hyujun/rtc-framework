// G0-E — time axes of the catching core (plan §3, L0 §4.5) and the D-2 stamp
// conversion (plan §3.1, E-1 recorded exception).
//
// Two halves:
//  • compile-time: no comparison exists between different time types, and each
//    §3 decision accepts only the now-type it is defined on. These are
//    static_asserts on requires-expressions, so a regression is a build break.
//  • run-time, on a T_arm ≠ 0 fixture: each decision flips at the instant its
//    OWN axis says, and would flip at a different instant on the other axis.
//    With T_arm = 0 the two axes coincide and none of these cases could tell a
//    swapped axis apart (plan §3).
#include "rtc_controllers/catching/time_types.hpp"

#include <gtest/gtest.h>

#include <concepts>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace {

using rtc::catching::AgeNs;
using rtc::catching::BallTime;
using rtc::catching::CommitDue;
using rtc::catching::ConvertRemoteStamp;
using rtc::catching::DecelDue;
using rtc::catching::HandCommandDue;
using rtc::catching::HorizonExceeded;
using rtc::catching::InContactWindow;
using rtc::catching::LeadSecondsUntil;
using rtc::catching::MakeNowLead;
using rtc::catching::NowLead;
using rtc::catching::NowReal;
using rtc::catching::PreshapeDue;
using rtc::catching::SampleBallTime;
using rtc::catching::SecondsBetween;
using rtc::catching::StampStatus;

// ── compile-time half ───────────────────────────────────────────────────────

template <typename A, typename B>
concept LessComparable = requires(A a, B b) { a < b; };

template <typename A, typename B>
concept EqualComparable = requires(A a, B b) { a == b; };

template <typename A, typename B>
concept Subtractable = requires(A a, B b) { a - b; };

// Same axis: ordered.
static_assert(std::totally_ordered<BallTime>);
static_assert(std::totally_ordered<NowReal>);
static_assert(std::totally_ordered<NowLead>);

// Different axes: no <, no ==, in either order.
static_assert(!LessComparable<BallTime, NowReal> && !LessComparable<NowReal, BallTime>);
static_assert(!LessComparable<BallTime, NowLead> && !LessComparable<NowLead, BallTime>);
static_assert(!LessComparable<NowReal, NowLead> && !LessComparable<NowLead, NowReal>);
static_assert(!EqualComparable<BallTime, NowReal> && !EqualComparable<BallTime, NowLead>);
static_assert(!EqualComparable<NowReal, NowLead>);
// No arithmetic that would hand back a bare number to compare across axes.
static_assert(!Subtractable<BallTime, NowReal> && !Subtractable<NowLead, BallTime>);
// Nor comparison against a raw integer.
static_assert(!LessComparable<BallTime, std::int64_t> && !LessComparable<NowReal, std::int64_t>);

// No implicit conversion between the axes.
static_assert(!std::is_convertible_v<NowReal, NowLead> && !std::is_convertible_v<NowLead, NowReal>);
static_assert(!std::is_convertible_v<NowReal, BallTime> &&
              !std::is_convertible_v<BallTime, NowLead>);

// Each §3 decision takes exactly its own now-type.
static_assert(std::is_invocable_v<decltype(&DecelDue), NowLead, BallTime>);
static_assert(!std::is_invocable_v<decltype(&DecelDue), NowReal, BallTime>);
static_assert(std::is_invocable_v<decltype(&HorizonExceeded), NowLead, BallTime>);
static_assert(!std::is_invocable_v<decltype(&HorizonExceeded), NowReal, BallTime>);
static_assert(std::is_invocable_v<decltype(&LeadSecondsUntil), NowLead, BallTime>);
static_assert(!std::is_invocable_v<decltype(&LeadSecondsUntil), NowReal, BallTime>);
static_assert(std::is_invocable_v<decltype(&CommitDue), NowReal, BallTime, std::int64_t>);
static_assert(!std::is_invocable_v<decltype(&CommitDue), NowLead, BallTime, std::int64_t>);
static_assert(std::is_invocable_v<decltype(&HandCommandDue), NowReal, BallTime>);
static_assert(!std::is_invocable_v<decltype(&HandCommandDue), NowLead, BallTime>);
static_assert(!std::is_invocable_v<decltype(&PreshapeDue), NowLead, BallTime, std::int64_t>);
static_assert(
    !std::is_invocable_v<decltype(&InContactWindow), NowLead, BallTime, BallTime, std::int64_t>);
// Message age cannot be measured from a ball instant (repo clock rule).
static_assert(std::is_invocable_v<decltype(&AgeNs), NowReal, NowReal>);
static_assert(!std::is_invocable_v<decltype(&AgeNs), NowReal, BallTime>);

// SeqLock payload rule (G0-B).
static_assert(std::is_trivially_copyable_v<BallTime> && std::is_trivially_copyable_v<NowReal> &&
              std::is_trivially_copyable_v<NowLead>);

// ── run-time half: T_arm ≠ 0 fixture ────────────────────────────────────────

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kTArm = 50 * kMs;  // T_arm = 50 ms — the axes differ by this

TEST(CatchingTimeTypes, LeadAxisIsNowPlusTArm) {
  const NowReal now{1'000 * kMs};
  EXPECT_EQ(MakeNowLead(now, kTArm).ns, 1'050 * kMs);
}

// CLOSING→DECEL is on the lead axis: 30 ms before t_c on the real axis the arm
// command is already realised after t_c, so DECEL is due; it would not be on
// the real axis.
TEST(CatchingTimeTypes, DecelDecidedOnLeadAxis) {
  const BallTime t_c{2'000 * kMs};
  const NowReal now{t_c.ns - 30 * kMs};
  EXPECT_TRUE(DecelDue(MakeNowLead(now, kTArm), t_c));
  EXPECT_LT(now.ns, t_c.ns);  // …which the real axis would not yet call

  const NowReal early{t_c.ns - 60 * kMs};
  EXPECT_FALSE(DecelDue(MakeNowLead(early, kTArm), t_c));
}

// Sampling reads the trajectory T_arm ahead of now.
TEST(CatchingTimeTypes, SamplingOffsetOnLeadAxis) {
  const BallTime t{2'000 * kMs};
  const NowReal now{1'900 * kMs};
  EXPECT_DOUBLE_EQ(LeadSecondsUntil(MakeNowLead(now, kTArm), t), 0.05);  // not 0.10
}

// Horizon exhaustion is on the lead axis; the last sample itself is not past it.
TEST(CatchingTimeTypes, HorizonOnLeadAxis) {
  const BallTime last{3'000 * kMs};
  EXPECT_FALSE(HorizonExceeded(NowLead{last.ns}, last));
  EXPECT_TRUE(HorizonExceeded(NowLead{last.ns + 1}, last));
  const NowReal now{last.ns - 40 * kMs};
  EXPECT_TRUE(HorizonExceeded(MakeNowLead(now, kTArm), last));
}

// APPROACH→COMMITTED is on the real axis: t_c − now ≤ T_freeze.
TEST(CatchingTimeTypes, CommitDecidedOnRealAxis) {
  const BallTime t_c{2'000 * kMs};
  const std::int64_t t_freeze = 200 * kMs;
  const NowReal before{t_c.ns - 230 * kMs};  // lead axis would already say ≤ T_freeze
  EXPECT_FALSE(CommitDue(before, t_c, t_freeze));
  EXPECT_LE(t_c.ns - MakeNowLead(before, kTArm).ns, t_freeze);
  EXPECT_TRUE(CommitDue(NowReal{t_c.ns - 200 * kMs}, t_c, t_freeze));  // boundary inclusive
}

// Hand commands have no lead compensation: real axis.
TEST(CatchingTimeTypes, HandCommandsOnRealAxis) {
  const BallTime t_cmd{1'940 * kMs};
  const BallTime t_c{2'000 * kMs};
  const NowReal now{t_cmd.ns - 20 * kMs};
  EXPECT_FALSE(HandCommandDue(now, t_cmd));  // lead axis would already be past t_cmd
  EXPECT_TRUE(HandCommandDue(NowReal{t_cmd.ns}, t_cmd));

  const std::int64_t t_pre = 150 * kMs;
  EXPECT_FALSE(PreshapeDue(NowReal{t_c.ns - t_pre - 1}, t_c, t_pre));
  EXPECT_TRUE(PreshapeDue(NowReal{t_c.ns - t_pre}, t_c, t_pre));
}

TEST(CatchingTimeTypes, ContactWindowClosedOnRealAxis) {
  const BallTime t_cmd{1'940 * kMs};
  const BallTime t_c{2'000 * kMs};
  const std::int64_t t_conf = 80 * kMs;
  EXPECT_FALSE(InContactWindow(NowReal{t_cmd.ns - 1}, t_cmd, t_c, t_conf));
  EXPECT_TRUE(InContactWindow(NowReal{t_cmd.ns}, t_cmd, t_c, t_conf));
  EXPECT_TRUE(InContactWindow(NowReal{t_c.ns + t_conf}, t_cmd, t_c, t_conf));
  EXPECT_FALSE(InContactWindow(NowReal{t_c.ns + t_conf + 1}, t_cmd, t_c, t_conf));
}

TEST(CatchingTimeTypes, AgeIsSteadyReceiveDifference) {
  EXPECT_EQ(AgeNs(NowReal{5'000 * kMs}, NowReal{4'970 * kMs}), 30 * kMs);
}

TEST(CatchingTimeTypes, SecondsBetweenIsOriginFree) {
  EXPECT_DOUBLE_EQ(SecondsBetween(BallTime{1'000 * kMs}, BallTime{1'250 * kMs}), 0.25);
  EXPECT_DOUBLE_EQ(SecondsBetween(BallTime{-3 * kMs}, BallTime{-1 * kMs}), 0.002);
}

// ── D-2 (3) stamp conversion ────────────────────────────────────────────────

// t_ref_steady = recv_steady − (recv_wall − stamp): the transport delay is
// carried over onto the steady axis.
TEST(CatchingStampConversion, CarriesTransportDelayOntoSteadyAxis) {
  const NowReal recv_steady{7'000 * kMs};
  const std::int64_t recv_wall = 1'758'000'000'000 * kMs;
  const std::int64_t stamp = recv_wall - 12 * kMs;  // published 12 ms before receipt
  const auto c = ConvertRemoteStamp(recv_steady, recv_wall, stamp, 5 * kMs);
  ASSERT_TRUE(c.IsOk());
  EXPECT_EQ(c.t_ref.ns, 6'988 * kMs);
  EXPECT_EQ(c.origin_delay_ns, 12 * kMs);
}

// A stamp slightly in the future is within tolerance; beyond it the conversion
// is untrustworthy and rejected (fail-closed), with the delay still reported so
// the diagnostic can count it.
TEST(CatchingStampConversion, RejectsStampBeyondFutureTolerance) {
  const NowReal recv_steady{7'000 * kMs};
  const std::int64_t recv_wall = 1'000'000 * kMs;
  const std::int64_t tol = 2 * kMs;
  const auto at_tol = ConvertRemoteStamp(recv_steady, recv_wall, recv_wall + tol, tol);
  EXPECT_TRUE(at_tol.IsOk());
  EXPECT_EQ(at_tol.t_ref.ns, 7'000 * kMs + tol);

  const auto past_tol = ConvertRemoteStamp(recv_steady, recv_wall, recv_wall + tol + 1, tol);
  EXPECT_EQ(past_tol.status, StampStatus::kFutureStamp);
  EXPECT_FALSE(past_tol.IsOk());
  EXPECT_EQ(past_tol.origin_delay_ns, -(tol + 1));
}

TEST(CatchingStampConversion, RejectsNegativeToleranceAndOverflow) {
  const NowReal recv_steady{7'000 * kMs};
  EXPECT_EQ(ConvertRemoteStamp(recv_steady, 0, 0, -1).status, StampStatus::kInvalidTolerance);

  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
  constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
  // recv_wall − stamp overflows (garbage stamp).
  EXPECT_EQ(ConvertRemoteStamp(recv_steady, kMax, -1, 0).status, StampStatus::kOverflow);
  // recv_steady − delay overflows.
  EXPECT_EQ(ConvertRemoteStamp(NowReal{kMin + 1}, kMax / 2, 0, 0).status, StampStatus::kOverflow);
}

TEST(CatchingStampConversion, SampleTimeAddsHorizonWithOverflowCheck) {
  BallTime out{};
  ASSERT_TRUE(SampleBallTime(BallTime{1'000 * kMs}, 800 * kMs, out));
  EXPECT_EQ(out.ns, 1'800 * kMs);
  EXPECT_FALSE(SampleBallTime(BallTime{std::numeric_limits<std::int64_t>::max()}, 1, out));
}

// Numerical-audit regression: instants near the int64 limits (a corrupted
// PlanSnapshot t_c) saturate instead of overflowing — and saturate in the
// fail-safe direction (a garbage t_c at +max is never due, at −max always).
TEST(CatchingTimeTypes, ExtremeInstantsSaturateInsteadOfOverflowing) {
  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
  constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
  const NowReal now{-5'000 * kMs};
  EXPECT_FALSE(CommitDue(now, BallTime{kMax}, 200 * kMs));  // t_c − now would overflow
  EXPECT_TRUE(CommitDue(NowReal{5'000 * kMs}, BallTime{kMin}, 200 * kMs));
  EXPECT_FALSE(PreshapeDue(NowReal{0}, BallTime{kMax}, 0));
  EXPECT_TRUE(PreshapeDue(NowReal{0}, BallTime{kMin}, 1));
  EXPECT_FALSE(InContactWindow(NowReal{0}, BallTime{kMax}, BallTime{kMax}, kMax));
  EXPECT_EQ(MakeNowLead(NowReal{kMax - 1}, kTArm).ns, kMax);
  EXPECT_EQ(AgeNs(NowReal{kMax}, NowReal{kMin}), kMax);
  EXPECT_GT(LeadSecondsUntil(NowLead{kMin}, BallTime{kMax}), 9.2e9);
}

}  // namespace
