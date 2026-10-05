// The RT-side admission and switch rules of a segment (planner_io.hpp:
// JudgeSegment, ChooseSegment, JudgeSegmentSwitch) — the contract a segment is
// taken under, whichever planner made it. Pure functions: no model, no solver.
// (MPC E1-F03 #629. The payload's own checks and the pre-catch admission cases
// are in test_catching_node_follower.cpp.)
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace {

using rtc::catching::AdmittedSegment;
using rtc::catching::ChooseSegment;
using rtc::catching::JudgeSegment;
using rtc::catching::JudgeSegmentSwitch;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::NowReal;
using rtc::catching::SegmentAdmissionContext;
using rtc::catching::SegmentChoice;
using rtc::catching::SegmentRefusal;
using rtc::catching::SegmentSnapshot;
using rtc::catching::SegmentSwitchVerdict;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kDt = 25 * kMs;                     // Δ_s (MD-24)
constexpr std::int64_t kT0 = 1'727'000'000'000'000'000LL;  // realistic steady ns
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

std::size_t Idx(int k, int j) {
  return static_cast<std::size_t>(k * kMaxSegmentNv + j);
}

SegmentSnapshot AdmissibleSegment() {
  SegmentSnapshot p{};
  p.valid = true;
  p.token.activation_generation = 3;
  p.plan_id = 7;
  p.segment_seq = 4;
  p.t_c_ns = kT0 + 500 * kMs;
  p.t0_ns = p.t_c_ns;
  p.dt_ns = kDt;
  p.n_nodes = 14;
  p.nv = 6;
  p.publish_ns = kT0 + 400 * kMs;
  return p;
}

SegmentAdmissionContext AdmissionContext() {
  SegmentAdmissionContext c;
  c.activation_generation = 3;
  c.plan_active = true;
  c.plan_id = 7;
  c.plan_t_c_ns = kT0 + 500 * kMs;
  c.now = NowReal{kT0 + 450 * kMs};
  c.max_age_ns = 500 * kMs;
  c.reset_floor_ns = kT0;
  return c;
}

TEST(SegmentAdmission, JudgesInTheDocumentedOrder) {
  const SegmentSnapshot good = AdmissibleSegment();
  const SegmentAdmissionContext ctx = AdmissionContext();
  EXPECT_EQ(JudgeSegment(good, ctx, AdmittedSegment{}), SegmentRefusal::kNone);
  EXPECT_EQ(JudgeSegment(good, ctx, AdmittedSegment{true, 3}), SegmentRefusal::kNone);
  auto judged = [&](auto mutate_plan, auto mutate_ctx, AdmittedSegment admitted = {}) {
    SegmentSnapshot p = good;
    SegmentAdmissionContext c = ctx;
    mutate_plan(p);
    mutate_ctx(c);
    return JudgeSegment(p, c, admitted);
  };
  const auto none_p = [](SegmentSnapshot&) {};
  const auto none_c = [](SegmentAdmissionContext&) {};
  EXPECT_EQ(judged([](SegmentSnapshot& p) { p.valid = false; }, none_c), SegmentRefusal::kInvalid);
  EXPECT_EQ(judged([](SegmentSnapshot& p) { p.token.activation_generation = 2; }, none_c),
            SegmentRefusal::kActivation);
  EXPECT_EQ(judged(none_p, [](SegmentAdmissionContext& c) { c.plan_active = false; }),
            SegmentRefusal::kPlan);
  EXPECT_EQ(judged([](SegmentSnapshot& p) { p.plan_id = 8; }, none_c), SegmentRefusal::kPlan);
  EXPECT_EQ(judged([](SegmentSnapshot& p) { p.t_c_ns += 1; }, none_c), SegmentRefusal::kPlan);
  EXPECT_EQ(judged(none_p, none_c, AdmittedSegment{true, 4}), SegmentRefusal::kRepeat);
  EXPECT_EQ(judged(none_p, none_c, AdmittedSegment{true, 5}), SegmentRefusal::kRepeat);
  EXPECT_EQ(judged([](SegmentSnapshot& p) { p.publish_ns = 0; }, none_c), SegmentRefusal::kAged);
  EXPECT_EQ(judged(none_p, [](SegmentAdmissionContext& c) { c.now = NowReal{kT0 + 399 * kMs}; }),
            SegmentRefusal::kAged)
      << "a publish instant in the future";
  EXPECT_EQ(judged(none_p, [](SegmentAdmissionContext& c) { c.max_age_ns = 10 * kMs; }),
            SegmentRefusal::kAged);
  EXPECT_EQ(judged(none_p, [](SegmentAdmissionContext& c) { c.max_age_ns = 0; }),
            SegmentRefusal::kNone)
      << "max_age 0 disables the age bound";
  EXPECT_EQ(judged(none_p, [](SegmentAdmissionContext& c) { c.reset_floor_ns = kT0 + 401 * kMs; }),
            SegmentRefusal::kBeforeReset);
  EXPECT_EQ(judged([](SegmentSnapshot& p) { p.qd[Idx(p.n_nodes, 0)] = kNan; }, none_c),
            SegmentRefusal::kMalformed);
  // First reason wins: invalid AND wrong plan → invalid.
  EXPECT_EQ(judged(
                [](SegmentSnapshot& p) {
                  p.valid = false;
                  p.plan_id = 9;
                },
                none_c),
            SegmentRefusal::kInvalid);
}

TEST(SegmentAdmission, SegmentSwitchesAtItsEffectiveInstant) {
  const std::int64_t t0 = kT0 + 100 * kMs;
  EXPECT_EQ(ChooseSegment(false, false, 0, t0), SegmentChoice::kNone);
  EXPECT_EQ(ChooseSegment(false, true, t0, t0 - 1), SegmentChoice::kNone)
      << "a pending segment is not sampled before its node 0";
  EXPECT_EQ(ChooseSegment(true, true, t0, t0 - 1), SegmentChoice::kCurrent);
  EXPECT_EQ(ChooseSegment(true, true, t0, t0), SegmentChoice::kPending);
  EXPECT_EQ(ChooseSegment(false, true, t0, t0 + 1), SegmentChoice::kPending);
  EXPECT_EQ(ChooseSegment(true, false, t0, t0 + 1), SegmentChoice::kCurrent);
}

TEST(SegmentAdmission, AStatePredictedBeforeTheFloorIsBeforeReset) {
  // MD-37: the planner stamps publish_ns after it reads the RT state, so a
  // segment can pass the publish floor and still come from the tick before
  // the reset. The state floor catches it; 0 leaves the check off.
  SegmentSnapshot p = AdmissibleSegment();
  SegmentAdmissionContext c = AdmissionContext();
  c.reset_floor_ns = kT0 + 300 * kMs;
  p.rt_state_ns = kT0 + 299 * kMs;
  EXPECT_EQ(JudgeSegment(p, c, AdmittedSegment{}), SegmentRefusal::kNone) << "floor off by default";
  c.state_floor_ns = kT0 + 300 * kMs;
  EXPECT_EQ(JudgeSegment(p, c, AdmittedSegment{}), SegmentRefusal::kBeforeReset);
  p.rt_state_ns = kT0 + 300 * kMs;
  EXPECT_EQ(JudgeSegment(p, c, AdmittedSegment{}), SegmentRefusal::kNone)
      << "at the floor is after";
}

TEST(SegmentSwitch, PassesInsideTheHeadroomAndNamesTheFirstJointPastIt) {
  // MD-39: |Δq̇_i| + K_p|Δq_i| ≤ ρ_max (1 − η_v) q̇_max,i per joint.
  const std::array<double, 3> qmax{2.0, 2.0, 4.0};  // headroom d = 0.2, 0.2, 0.4 at η_v 0.9
  std::array<double, 3> q_c{0.0, 0.0, 0.0};
  std::array<double, 3> qd_c{0.0, 0.0, 0.0};
  const std::array<double, 3> q_ref{0.0, 0.0, 0.0};
  const std::array<double, 3> qd_ref{0.0, 0.0, 0.0};
  auto judge = [&](double rho_max) {
    return JudgeSegmentSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 3, 20.0, 0.9, rho_max);
  };
  SegmentSwitchVerdict v = judge(1.0);
  EXPECT_TRUE(v.pass);
  EXPECT_EQ(v.rho, 0.0);
  EXPECT_EQ(v.joint, -1);

  q_c[2] = 0.005;  // K_p·Δq = 0.1 → ρ_2 = 0.25
  qd_c[0] = 0.1;   // ρ_0 = 0.5
  v = judge(1.0);
  EXPECT_TRUE(v.pass);
  EXPECT_NEAR(v.rho, 0.5, 1e-12);
  EXPECT_NEAR(v.dq_max, 0.005, 1e-15);
  EXPECT_NEAR(v.dqd_max, 0.1, 1e-15);
  v = judge(0.4);
  EXPECT_FALSE(v.pass);
  EXPECT_EQ(v.joint, 0);
  EXPECT_NEAR(v.rho, 0.5, 1e-12) << "ρ is recorded over every joint, not up to the refusal";

  qd_c[0] = std::numeric_limits<double>::quiet_NaN();
  v = judge(1.0);
  EXPECT_FALSE(v.pass) << "a NaN difference refuses";
  EXPECT_EQ(v.joint, 0);
  EXPECT_TRUE(std::isinf(v.rho));
  qd_c[0] = 0.0;

  // No headroom (η_v = 1, or a missing q̇_max) refuses rather than divides.
  EXPECT_FALSE(JudgeSegmentSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 3, 20.0, 1.0, 1.0).pass);
  const std::array<double, 3> no_limit{2.0, 0.0, 4.0};
  v = JudgeSegmentSwitch(q_c, qd_c, q_ref, qd_ref, no_limit, 3, 20.0, 0.9, 1.0);
  EXPECT_FALSE(v.pass);
  EXPECT_EQ(v.joint, 1);
  // Bad arguments refuse outright.
  EXPECT_FALSE(JudgeSegmentSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 4, 20.0, 0.9, 1.0).pass);
  EXPECT_FALSE(JudgeSegmentSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 3, 20.0, 0.9, 0.0).pass);
  EXPECT_FALSE(JudgeSegmentSwitch(q_c, qd_c, q_ref, qd_ref, qmax, 0, 20.0, 0.9, 1.0).pass);
}

}  // namespace
