/// @file clik_golden_scenarios.hpp
/// @brief The scenarios of the CLIK golden-vector regression, apart from the
///        recorded table and its assertions (test_clik_golden.cpp keeps those).
///
/// Two suites run them: test_clik_golden through the SE3 Compute() overload,
/// against the recorded table, and test_clik_multiframe through the
/// multi-frame overload, against the first — so the inputs must be ONE
/// definition. `compute` is the only thing a suite supplies.
///
/// Changing anything here changes what the recorded table means: the rules in
/// test_clik_golden.cpp's header apply (E-6 / PROC-6).

#pragma once

#include "panda_fixture.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace rtc::tsid::test::clik_golden {

constexpr int kNv = 9;  // Panda: 7 arm + 2 finger, nq == nv
// Per tick: ok, q_ref[nv], v_ref[nv], manipulability, tcp_error_norm.
constexpr int kRecordWidth = 1 + 2 * kNv + 2;
constexpr double kDt = 0.01;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// The SE3 task gain of every scenario (all six rows).
constexpr double kTaskGain = 5.0;

using Vec6 = Eigen::Matrix<double, 6, 1>;

struct Scenario {
  const char* name;
  int ticks;
  double v_limit;
  bool position_box;
  double anchor_drift_max;
  bool registered_base;
};

// Fixed order; the table is the concatenation of these runs.
inline constexpr Scenario kScenarios[] = {
    // Measured reseed, both boxes on, large target offset (velocity clamp),
    // q leaves the envelope on both sides (collapse + re-clamp), hand posture.
    {"reseed_boxes", 40, 0.5, true, 0.0, true},
    // Carry-forward with reseed edges, drift clamp, a non-finite target tick
    // (failure branch → forced measured re-anchor on the next call).
    {"carry_forward_drift", 40, 1.5, true, 0.02, false},
    // Velocity box off: the collapse stays raw (unbounded) above q_max.
    {"no_velocity_box", 20, 0.0, true, 0.0, true},
    // Position box off: velocity box only, carry-forward, and a dt = +inf tick
    // (non-finite output branch → forced measured re-anchor).
    {"no_position_box", 20, 1.5, false, 0.0, false},
};

/// What one tick hands to the Compute() call under test.
struct Tick {
  const PinocchioCache& cache;
  int tcp_idx;
  int base_idx;  ///< −1 for the universe scenarios
  const pinocchio::SE3& des;
  const Eigen::VectorXd& q_posture;
  double dt;
  bool reseed;
};

class ClikGoldenScenarioTest : public PandaTest {
 protected:
  void SetUp() override {
    PandaTest::SetUp();
    ASSERT_EQ(robot_info_.nq, kNv);
    ASSERT_EQ(robot_info_.nv, kNv);

    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg));
    tcp_idx_ = cache_.RegisterFrame("panda_hand", model_->getFrameId("panda_hand"));
    // panda_link1, not panda_link0: link0 sits at the world origin, so a base
    // transform bug (e.g. ignoring base_frame_idx) would be invisible.
    base_idx_ = cache_.RegisterFrame("panda_link1", model_->getFrameId("panda_link1"));
    ASSERT_GE(tcp_idx_, 0);
    ASSERT_GE(base_idx_, 0);

    q_home_.resize(kNv);
    q_home_ << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02;
    q_min_ = model_->lowerPositionLimit;
    q_max_ = model_->upperPositionLimit;
  }

  // Measured q at tick k: a deterministic multi-joint oscillation around home,
  // plus scenario-specific excursions past the joint envelope.
  [[nodiscard]] Eigen::VectorXd MeasuredQ(int scenario, int k) const {
    Eigen::VectorXd q = q_home_;
    for (int j = 0; j < 7; ++j) {
      q(j) += 0.05 * std::sin(0.3 * k + 0.7 * j);
    }
    q(7) = 0.02 + 0.01 * std::sin(0.2 * k);
    q(8) = 0.02 + 0.01 * std::cos(0.2 * k);
    if (scenario == 0 || scenario == 2) {
      // Past q_max on joint 3 for ticks 10–14, past q_min on joint 5 for 20–24.
      if (k >= 10 && k < 15) {
        q(3) = q_max_(3) + 0.01 * (k - 9);
      }
      if (k >= 20 && k < 25) {
        q(5) = q_min_(5) - 0.01 * (k - 19);
      }
    }
    return q;
  }

  [[nodiscard]] Eigen::VectorXd MeasuredV(int k) const {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
    for (int j = 0; j < 7; ++j) {
      v(j) = 0.015 * std::cos(0.3 * k + 0.7 * j);
    }
    return v;
  }

  /// Runs scenario `s`, calling `compute(gen, tick)` — a bool — once per tick.
  template <typename ComputeFn>
  [[nodiscard]] std::vector<double> RunScenario(int s, ComputeFn&& compute) {
    const Scenario& sc = kScenarios[s];
    ClikReferenceGenerator gen;
    ClikReferenceGenerator::Config cfg;
    cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
    cfg.hand_v_idx = {7, 8};
    cfg.damping_sq = 1e-4;
    cfg.v_limit = sc.v_limit;
    cfg.anchor_drift_max = sc.anchor_drift_max;
    if (sc.position_box) {
      cfg.q_min = q_min_;
      cfg.q_max = q_max_;
    }
    gen.Init(kNv, cfg);
    gen.SetTaskGain(Vec6::Constant(kTaskGain));
    gen.SetPostureGains(0.5, 2.0);

    // Target: home TCP pose shifted and rotated, then swept along a circle.
    cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
    const auto& tip = cache_.registered_frames[static_cast<size_t>(tcp_idx_)];
    const auto& base = cache_.registered_frames[static_cast<size_t>(base_idx_)];
    const pinocchio::SE3 home_in_base = sc.registered_base ? base.oMf.actInv(tip.oMf) : tip.oMf;

    Eigen::VectorXd q_posture = q_home_;
    q_posture(7) = 0.035;  // hand posture away from the measured fingers
    q_posture(8) = 0.005;

    std::vector<double> out;
    out.reserve(static_cast<size_t>(sc.ticks * kRecordWidth));
    for (int k = 0; k < sc.ticks; ++k) {
      pinocchio::SE3 des = home_in_base;
      des.translation() +=
          Eigen::Vector3d(0.25 + 0.05 * std::cos(0.1 * k), 0.05 * std::sin(0.1 * k), -0.1);
      des.rotation() =
          des.rotation() * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
      if (s == 1 && k == 25) {
        des.translation()(0) = kNaN;  // non-finite target → QP failure branch
      }
      // dt = +inf passes the dt > 0 precondition, the QP stays finite and
      // q_ref = q + v·dt does not → the non-finite-output branch.
      // Scenario 3 because it has no drift clamp — the clamp would pull the
      // infinite q_ref back to q_meas ± anchor_drift_max and hide the branch.
      const double dt = (s == 3 && k == 12) ? std::numeric_limits<double>::infinity() : kDt;
      // Carry-forward in scenario 1 except at the reseed edges 0 and 18, and in
      // scenario 3 after tick 0 (so the re-anchor after its failure is visible).
      const bool reseed = (s == 1) ? (k == 0 || k == 18) : (s == 3) ? (k == 0) : true;

      cache_.Update(MeasuredQ(s, k), MeasuredV(k));
      const bool ok = compute(gen, Tick{cache_, tcp_idx_, sc.registered_base ? base_idx_ : -1, des,
                                        q_posture, dt, reseed});
      out.push_back(ok ? 1.0 : 0.0);
      for (int i = 0; i < kNv; ++i) {
        out.push_back(gen.QRef()(i));
      }
      for (int i = 0; i < kNv; ++i) {
        out.push_back(gen.VRef()(i));
      }
      out.push_back(gen.Manipulability());
      out.push_back(gen.TcpErrorNorm());
    }
    return out;
  }

  PinocchioCache cache_;
  int tcp_idx_{-1};
  int base_idx_{-1};
  Eigen::VectorXd q_home_;
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;
};

}  // namespace rtc::tsid::test::clik_golden
