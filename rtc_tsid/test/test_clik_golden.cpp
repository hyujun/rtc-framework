/// @file test_clik_golden.cpp
/// @brief Golden-vector regression of ClikReferenceGenerator (dynamic_catching
///        S2.2a, gate G5-A2 — docs/dynamic_catching/L5_joint_cmd.md §5.1).
///
/// Pins the pre-extension behaviour so the S2.2b options can be added with
/// "all options off ⇒ bit-identical output" checked, not assumed. The golden
/// table (golden/clik_golden_data.inc) was recorded at 6d4a3561 — main
/// f0e03c42 plus the QPSolverWrapper non-finite recovery fix, before any S2.2
/// CLIK change — and must never be re-recorded to make this test pass: a
/// mismatch means the default path changed (E-6 / PROC-6).
///
/// Inputs are open-loop: every tick's measured q comes from a fixed sequence,
/// not from the previous q_ref, so the recorded "q sequence → outputs" map is
/// independent of the output under test. Four scenarios together reach every
/// branch of Compute(): velocity clamp, position box, box collapse (both
/// directions, with and without the velocity re-clamp), carry-forward anchor,
/// measured reseed, anchor drift clamp, both failure branches (QP failure on a
/// non-finite target; non-finite output on dt = +inf) and their forced
/// re-anchor, hand posture, universe and registered (non-identity) base frames.
///
/// Comparison: bit-exact (memcmp of the IEEE-754 bits) when the build defines
/// RTC_CLIK_GOLDEN_EXACT (CMake sets it for Release without sanitizers — the
/// configuration the table was recorded in). Other configurations use a
/// relative tolerance, because a different optimisation level may reorder
/// floating-point operations.
///
/// Re-recording (only when the default path is INTENDED to change, in its own
/// commit with the reason): run the binary with RTC_CLIK_GOLDEN_OUT=<path>.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/kinematics/clik_reference.hpp"

namespace rtc::tsid {
namespace {

// Recorded table: kClikGoldenBits (IEEE-754 bits), kClikGoldenHash (FNV-1a 64).
#include "golden/clik_golden_data.inc"

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;
constexpr int kNv = 9;  // Panda: 7 arm + 2 finger, nq == nv
// Per tick: ok, q_ref[nv], v_ref[nv], manipulability, tcp_error_norm.
constexpr int kRecordWidth = 1 + 2 * kNv + 2;
constexpr double kDt = 0.01;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// ProxQP eps_abs (QPSolverConfig default): the box holds only to this tolerance.
constexpr double kSolverEps = 1e-6;

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
constexpr Scenario kScenarios[] = {
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

class ClikGoldenTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;
    YAML::Node config;
    robot_info_.Build(*model_, config);
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

  [[nodiscard]] std::vector<double> RunScenario(int s) {
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
    gen.SetTaskGain(Vec6::Constant(5.0));
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
      const bool ok = gen.Compute(cache_, tcp_idx_, sc.registered_base ? base_idx_ : -1, des,
                                  q_posture, dt, reseed);
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

  [[nodiscard]] std::vector<double> RunAll() {
    std::vector<double> all;
    for (int s = 0; s < static_cast<int>(std::size(kScenarios)); ++s) {
      const std::vector<double> r = RunScenario(s);
      all.insert(all.end(), r.begin(), r.end());
    }
    return all;
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  PinocchioCache cache_;
  int tcp_idx_{-1};
  int base_idx_{-1};
  Eigen::VectorXd q_home_;
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;
};

[[nodiscard]] std::uint64_t Bits(double x) {
  std::uint64_t b = 0;
  std::memcpy(&b, &x, sizeof(b));
  return b;
}

[[nodiscard]] double FromBits(std::uint64_t b) {
  double x = 0.0;
  std::memcpy(&x, &b, sizeof(x));
  return x;
}

[[nodiscard]] std::uint64_t Fnv1a(const std::vector<double>& values) {
  std::uint64_t h = 14695981039346656037ULL;
  for (const double v : values) {
    std::uint64_t b = Bits(v);
    for (int i = 0; i < 8; ++i) {
      h ^= (b & 0xFFU);
      h *= 1099511628211ULL;
      b >>= 8U;
    }
  }
  return h;
}

// Location of flat index i as scenario / tick / field, for failure messages.
[[nodiscard]] std::string Where(size_t i) {
  size_t base = 0;
  for (const Scenario& sc : kScenarios) {
    const size_t n = static_cast<size_t>(sc.ticks * kRecordWidth);
    if (i < base + n) {
      const size_t off = i - base;
      const size_t tick = off / kRecordWidth;
      const size_t f = off % kRecordWidth;
      std::string field;
      if (f == 0) {
        field = "ok";
      } else if (f <= kNv) {
        field = "q_ref[" + std::to_string(f - 1) + "]";
      } else if (f <= 2 * kNv) {
        field = "v_ref[" + std::to_string(f - 1 - kNv) + "]";
      } else if (f == 2 * kNv + 1) {
        field = "manipulability";
      } else {
        field = "tcp_error_norm";
      }
      return std::string(sc.name) + " tick " + std::to_string(tick) + " " + field;
    }
    base += n;
  }
  return "out of range";
}

void WriteGolden(const std::vector<double>& values, const char* path) {
  std::ofstream f(path);
  f << "// Generated by test_clik_golden (RTC_CLIK_GOLDEN_OUT). Do not edit.\n"
       "// ClikReferenceGenerator outputs recorded at 6d4a3561 — see the\n"
       "// test file header before re-recording.\n"
       "// clang-format off\n"
       "constexpr std::uint64_t kClikGoldenHash = 0x"
    << std::hex << Fnv1a(values) << "ULL;\n"
    << "constexpr std::uint64_t kClikGoldenBits[] = {\n";
  for (size_t i = 0; i < values.size(); ++i) {
    f << "0x" << std::hex << Bits(values[i]) << "ULL,";
    f << (((i + 1) % kRecordWidth == 0) ? "\n" : " ");
  }
  f << "};\n// clang-format on\n";
}

// G5-A2 / S2.2a: default path reproduces the recorded table.
TEST_F(ClikGoldenTest, DefaultPathMatchesRecordedTable) {
  const std::vector<double> got = RunAll();
  if (const char* out = std::getenv("RTC_CLIK_GOLDEN_OUT"); out != nullptr) {
    WriteGolden(got, out);
    GTEST_SKIP() << "recorded " << got.size() << " values to " << out;
  }

  constexpr size_t kExpected = std::size(kClikGoldenBits);
  ASSERT_EQ(got.size(), kExpected) << "scenario set changed — the table no longer lines up";

#ifdef RTC_CLIK_GOLDEN_EXACT
  RecordProperty("golden_mode", "bit_exact");
  size_t mismatches = 0;
  for (size_t i = 0; i < kExpected; ++i) {
    if (Bits(got[i]) != kClikGoldenBits[i]) {
      if (++mismatches <= 10) {
        ADD_FAILURE() << Where(i) << ": got " << std::hexfloat << got[i] << ", recorded "
                      << FromBits(kClikGoldenBits[i]);
      }
    }
  }
  EXPECT_EQ(mismatches, 0U);
  EXPECT_EQ(Fnv1a(got), kClikGoldenHash);
#else
  RecordProperty("golden_mode", "relative_1e-12");
  double worst = 0.0;
  size_t worst_i = 0;
  for (size_t i = 0; i < kExpected; ++i) {
    const double want = FromBits(kClikGoldenBits[i]);
    // The table legitimately holds NaN (tcp_error_norm on the NaN-target tick)
    // and ±inf; identical specials are a match.
    if ((std::isnan(want) && std::isnan(got[i])) || want == got[i]) {
      continue;
    }
    const double scale = std::max(1.0, std::abs(want));
    const double rel = std::abs(got[i] - want) / scale;
    if (!(rel <= worst)) {  // also catches NaN on either side
      worst = std::isnan(rel) ? std::numeric_limits<double>::infinity() : rel;
      worst_i = i;
    }
  }
  EXPECT_LE(worst, 1e-12) << "worst at " << Where(worst_i);
#endif
}

// The table is not vacuous: every branch it claims to cover actually fired.
// These are properties of the recorded inputs, checked on the current run.
TEST_F(ClikGoldenTest, ScenariosReachEveryBranch) {
  const std::vector<double> got = RunAll();
  auto at = [&](int s, int tick, int field) {
    size_t base = 0;
    for (int i = 0; i < s; ++i) {
      base += static_cast<size_t>(kScenarios[i].ticks * kRecordWidth);
    }
    return got[base + static_cast<size_t>(tick * kRecordWidth + field)];
  };
  auto v_ref = [&](int s, int tick, int j) { return at(s, tick, 1 + kNv + j); };

  // Velocity clamp active somewhere in scenario 0 (v_limit 0.5).
  bool clamp_hit = false;
  for (int k = 0; k < kScenarios[0].ticks; ++k) {
    for (int j = 0; j < 7; ++j) {
      clamp_hit = clamp_hit || std::abs(std::abs(v_ref(0, k, j)) - 0.5) < 1e-6;
    }
  }
  EXPECT_TRUE(clamp_hit) << "velocity clamp never active";

  // Collapse above q_max (joint 3, ticks 10–14): only motion back, bounded by v_limit.
  for (int k = 10; k < 15; ++k) {
    EXPECT_LE(v_ref(0, k, 3), 0.0) << k;
    EXPECT_GE(v_ref(0, k, 3), -0.5 - kSolverEps) << k;
  }
  // Collapse below q_min (joint 5, ticks 20–24).
  for (int k = 20; k < 25; ++k) {
    EXPECT_GE(v_ref(0, k, 5), 0.0) << k;
    EXPECT_LE(v_ref(0, k, 5), 0.5 + kSolverEps) << k;
  }
  // Velocity box off: the raw collapse exceeds any sane limit above q_max.
  bool raw_collapse = false;
  for (int k = 10; k < 15; ++k) {
    raw_collapse = raw_collapse || v_ref(2, k, 3) < -0.5;
  }
  EXPECT_TRUE(raw_collapse) << "unbounded collapse branch not reached";

  // Failure branches: scenario 1 tick 25 (QP), scenario 3 tick 12 (non-finite
  // output); success on every other tick, including right after each.
  EXPECT_EQ(at(1, 25, 0), 0.0);
  EXPECT_EQ(at(3, 12, 0), 0.0);
  for (int k = 0; k < kScenarios[3].ticks; ++k) {
    if (k != 12) {
      EXPECT_EQ(at(3, k, 0), 1.0) << k;
    }
  }
  for (int k = 0; k < kScenarios[1].ticks; ++k) {
    if (k != 25) {
      EXPECT_EQ(at(1, k, 0), 1.0) << k;
    }
  }
  // Drift clamp active in scenario 1: some |q_ref − q_meas| sits at 0.02.
  bool drift_hit = false;
  for (int k = 1; k < kScenarios[1].ticks; ++k) {
    const Eigen::VectorXd q = MeasuredQ(1, k);
    for (int j = 0; j < kNv; ++j) {
      drift_hit = drift_hit || std::abs(std::abs(at(1, k, 1 + j) - q(j)) - 0.02) < 1e-12;
    }
  }
  EXPECT_TRUE(drift_hit) << "anchor drift clamp never active";

  // Hand posture drives the fingers (hand v_ref nonzero).
  EXPECT_GT(std::abs(v_ref(3, 5, 7)) + std::abs(v_ref(3, 5, 8)), 1e-6);
}

}  // namespace
}  // namespace rtc::tsid
