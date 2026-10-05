// E1-F07 (#660): the single-arm MPC core from APPROACH to the stop — the
// pre-catch grid and the catch terms added to MpcSegmentCore (mpc_segment_core.hpp;
// formulation §1.6, MD-51 – MD-53). The stop-segment behaviour E1-F01
// pinned stays in test_catching_mpc_segment_core.cpp; this suite owns what is new,
// plus the regression that the new code leaves the old problem alone.
//
// The allocation gates: like the E1-F01 suite, this binary links THREE sensors
// (CMakeLists note); malloc_gate.hpp is the one that sees pinocchio's and
// ProxQP's own allocations.
#include "rtc_controllers/catching/jerk_segment.hpp"
#include "rtc_controllers/catching/mpc_segment_core.hpp"
#include "rtc_controllers/catching/mpc_segment_core_catch.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"
#include "rtc_math/se3/axis_align.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::catching::MpcSegmentCore;
using rtc::catching::MpcSegmentCoreInput;
using rtc::catching::MpcSegmentCoreLimits;
using rtc::catching::MpcSegmentCoreParams;
using rtc::catching::MpcSegmentCoreReason;
using rtc::catching::MpcSegmentCoreReasonName;
using rtc::catching::MpcSegmentCoreResult;
using rtc::testing::mpc_segment_core::ArmModel;
using rtc::testing::mpc_segment_core::LimitsFromModel;
using rtc::testing::mpc_segment_core::Rank;
using rtc::testing::mpc_segment_core::RealArm6;
using rtc::testing::mpc_segment_core::RealArm7;
using rtc::testing::mpc_segment_core::RestInput;
using rtc::testing::mpc_segment_core::Synthetic6R;
using rtc::testing::mpc_segment_core::UseAsReference;

// ── Golden regression: the E1-F01 problem is untouched ───────────────────────
// The values below were captured from the build of `main` at edc0fa4e, BEFORE
// any E1-F07 change to the core, by running this test with
// RTC_MPC_SEGMENT_CORE_GOLDEN_PRINT=1. They pin what "the new parameters default to
// off" has to mean: with defaults the core assembles the SAME QP and returns
// the same stop.
//
// Two tolerances, for two different things:
//   • the assembled QP (H, g, A, b, C, l, u of both problems) at RELATIVE 1e-12
//     — assembly is plain arithmetic, so a refactor that keeps the uniform grid
//     bit-compatible stays far inside this, and anything that changes the
//     problem does not;
//   • the solution at 1e-5 — ProxQP stops at eps_abs = 1e-6, so the iterate
//     path (and the last digits of the answer) may move for reasons that are
//     not a regression.
// A matrix is pinned through six numbers: four fixed pseudo-random projections,
// the sum of |entries|, and a signature of where the ±inf bounds sit.
//
// Regenerating these constants is a change of an existing assertion (PROC-6):
// it needs its own commit and a reason.

struct Probe {
  std::vector<double> tight;  // matrix fingerprints
  std::vector<double> scale;  // per-entry comparison scale
  std::vector<double> loose;  // solution values
};

void AddMatrix(Probe& p, const Eigen::Ref<const Eigen::MatrixXd>& m) {
  double proj[4] = {0.0, 0.0, 0.0, 0.0};
  double abs_sum = 0.0;
  double inf_sig = 0.0;
  for (Eigen::Index c = 0; c < m.cols(); ++c) {
    for (Eigen::Index r = 0; r < m.rows(); ++r) {
      const double v = m(r, c);
      const double idx = static_cast<double>(c * m.rows() + r + 1);
      if (!std::isfinite(v)) {
        inf_sig += v > 0.0 ? idx : -idx;
        continue;
      }
      abs_sum += std::abs(v);
      for (int i = 0; i < 4; ++i) {
        proj[i] += v * std::sin(0.7 + 1.3 * i + 0.37 * static_cast<double>(r) +
                                0.91 * static_cast<double>(c));
      }
    }
  }
  for (const double v : proj) {
    p.tight.push_back(v);
    p.scale.push_back(abs_sum);
  }
  p.tight.push_back(abs_sum);
  p.scale.push_back(abs_sum);
  p.tight.push_back(inf_sig);
  p.scale.push_back(0.0);  // exact
}

void AddQp(Probe& p, const rtc::tsid::QPData& qp) {
  AddMatrix(p, qp.H);
  AddMatrix(p, qp.g);
  AddMatrix(p, qp.A);
  AddMatrix(p, qp.b);
  AddMatrix(p, qp.C);
  AddMatrix(p, qp.l);
  AddMatrix(p, qp.u);
}

void AddSolution(Probe& p, const MpcSegmentCoreResult& r) {
  const Eigen::Index N = r.q.cols() - 1;
  for (const Eigen::Index k : {N / 2, N}) {
    for (Eigen::Index j = 0; j < r.q.rows(); ++j) {
      p.loose.push_back(r.q(j, k));
    }
  }
  for (Eigen::Index j = 0; j < r.qd.rows(); ++j) {
    p.loose.push_back(r.qd(j, N / 2));
  }
  p.loose.push_back(r.slack_max);
  p.loose.push_back(r.tau_ratio_max);
}

[[nodiscard]] bool GoldenPrintMode() {
  const char* e = std::getenv("RTC_DECEL_MPC_GOLDEN_PRINT");
  return e != nullptr && e[0] != '\0' && e[0] != '0';
}

void PrintGolden(const char* name, const Probe& p) {
  std::printf("// %s\nconst std::vector<double> kGolden_%s_tight = {\n", name, name);
  for (std::size_t i = 0; i < p.tight.size(); ++i) {
    std::printf("    %.17g,%s", p.tight[i], (i % 3 == 2) ? "\n" : "");
  }
  std::printf("};\nconst std::vector<double> kGolden_%s_loose = {\n", name);
  for (std::size_t i = 0; i < p.loose.size(); ++i) {
    std::printf("    %.17g,%s", p.loose[i], (i % 3 == 2) ? "\n" : "");
  }
  std::printf("};\n");
}

void ExpectGolden(const char* name, const Probe& p, const std::vector<double>& tight,
                  const std::vector<double>& loose) {
  if (GoldenPrintMode()) {
    PrintGolden(name, p);
    return;
  }
  ASSERT_EQ(p.tight.size(), tight.size()) << name << ": golden is missing or stale";
  ASSERT_EQ(p.loose.size(), loose.size()) << name << ": golden is missing or stale";
  for (std::size_t i = 0; i < tight.size(); ++i) {
    EXPECT_LE(std::abs(p.tight[i] - tight[i]), 1e-12 * p.scale[i])
        << name << " matrix fingerprint " << i << " (matrix " << i / 6 << ", entry " << i % 6
        << "): " << p.tight[i] << " vs golden " << tight[i];
  }
  for (std::size_t i = 0; i < loose.size(); ++i) {
    EXPECT_NEAR(p.loose[i], loose[i], 1e-5) << name << " solution value " << i;
  }
}

// A solved trajectory shifted by `t_shift` onto the same uniform grid (the
// E1-F03 warm start), x_0 on it.
void ShiftUniform(const MpcSegmentCoreResult& r, double dt, double t_shift,
                  MpcSegmentCoreInput& in) {
  const Eigen::Index n = r.q.rows();
  const Eigen::Index cols = r.q.cols();
  in.q_ref.resize(n, cols);
  in.qd_ref.resize(n, cols);
  in.qdd_ref.resize(n, cols);
  Eigen::VectorXd q(n), qd(n), qdd(n);
  for (Eigen::Index k = 0; k < cols; ++k) {
    ASSERT_TRUE(rtc::catching::SampleJerkTrajectory(
        r.q, r.qd, r.qdd, dt, t_shift + static_cast<double>(k) * dt, q, qd, qdd));
    in.q_ref.col(k) = q;
    in.qd_ref.col(k) = qd;
    in.qdd_ref.col(k) = qdd;
  }
  in.reference_valid = true;
  in.q0 = in.q_ref.col(0);
  in.qd0 = in.qd_ref.col(0);
  in.qdd0 = in.qdd_ref.col(0);
}

// GOLDEN-CONSTANTS-BEGIN (main edc0fa4e, Release, 2026-10-01)
// clang-format off
const std::vector<double> kGolden_kinematic_6r_tight = {
    2.0975223758848318, 2.7779410800051276, -0.61133040610582645,
    -3.1050014150769387, 72, 0,
    0, 0, 0,
    0, 0, 0,
    -59.38174127892043, 265.83283673065381, 201.60168614972119,
    -157.97640694307051, 4680, 0,
    -0.92880220269491764, -0.27344680166202939, 0.7825088044234525,
    0.6920871788054288, 6.6100000000000012, 0,
    -47.845731539272457, 12.957330264087243, 54.777872874763482,
    16.348703393003422, 5851.4999999999982, 0,
    -11.868524095846992, 2.770794872940336, 13.350892861588102,
    4.3719015301940214, 379.14125000000001, -31176,
    10.102816417685368, -3.9644701962100481, -12.223798684891936,
    -2.5752334628926681, 348.05875000000015, 31176,
    -130.07049534937607, 142.66874043045968, 206.39793724236256,
    -32.246327544735337, 17613.536458333318, 0,
    34.016148859569945, -4.0527307803653807, -36.184350332526989,
    -15.305811876619925, 783.14268426074, 0,
    -59.38174127892043, 265.83283673065381, 201.60168614972119,
    -157.97640694307051, 4680, 0,
    -0.92880220269491764, -0.27344680166202939, 0.7825088044234525,
    0.6920871788054288, 6.6100000000000012, 0,
    -40.494620849123912, 15.041481766404518, 48.541778355706896,
    10.928255932606628, 6080.1136838575658, 0,
    -1.6849979155651313, 5.9263525914139139, 4.8555826680041481,
    -3.3286272394520235, 241.73210783102456, -12996,
    -0.30093980165202872, -10.385672670115063, -5.2553707458163412,
    7.5740616331274948, 239.3589697650076, 41544,
};
const std::vector<double> kGolden_kinematic_6r_loose = {
    0.29304347827120991, -0.57077898550867301, 0.96473188434607815,
    -0.43638586877565005, 0.66057427535957158, 0.035336956519668374,
    0.31782608699112119, -0.58855072467436809, 0.98240579681597517,
    -0.44445652284699633, 0.66903623187851069, 0.022652173906903616,
    0.21847826096056278, -0.15815217393229861, 0.15521739089000494,
    -0.073369565796760558, 0.085108695633854953, -0.11804347828152421,
    9.9153826788695908e-10, 0.15826776759386685,
};
const std::vector<double> kGolden_perp_7r_warm_tight = {
    2801.3844433248487, 812.00863468594707, -2366.9617261017702,
    -2078.3276129488627, 97010.820090870926, 0,
    7.961049401475039, 11.699519328399447, -1.7018339698399219,
    -12.609996515290852, 896.93957280152745, 0,
    -581.89921998457589, -568.08604116270158, 277.97451884657289,
    716.80175752058472, 5460, 0,
    -1.6856807043757249, -1.7819803626403938, 0.73232538509907885,
    2.1737727280125005, 10.77701765349331, 0,
    -193.57991460613135, -112.78696789666985, 133.23915101317476,
    184.0696015425882, 7278.5938111046671, 0,
    -0.64533938131083246, 1.9854522435180963, 1.7075516801731703,
    -1.0719160949936044, 258.26169350279378, -17682,
    -0.98432333109613901, -3.1253706752685857, -0.68774265820787661,
    2.7574299643371081, 244.7175139729338, 56532,
};
const std::vector<double> kGolden_perp_7r_warm_loose = {
    0.041480331310868312, 0.60932859159018793, 0.031904729961790786,
    -1.1321943400897407, -0.099206589481820401, 1.0468901356426472,
    0.050230860680266803, 0.044609656871085306, 0.61235768085460585,
    0.025590225409024083, -1.1233776422020949, -0.12357193560195749,
    1.0889197253091853, 0.059565808107798462, -0.047770274510690464,
    0.11453451680123873, -0.087831031975010757, 0.0057225133662009292,
    -0.22289881502152767, 0.36905517939029348, 0.08746208059266776,
    1.0690346466812067e-19, 0.25913673094721879,
};
const std::vector<double> kGolden_torque_6r_shipped_tight = {
    3.2347628215285917, 4.2330258732956212, -0.97010389624027171,
    -4.7520291850724652, 84, 0,
    0, 0, 0,
    0, 0, 0,
    -51.490194366363788, 150.07842905745466, 131.78180231573768,
    -79.575473550461822, 2467.5, 0,
    -2.0767861445900437, -0.60824766145748344, 1.7513750706830005,
    1.5452292212374961, 3.8999999999999995, 0,
    17.335286277490262, 5.0918909642138699, -14.611136540667632,
    -12.908814783219158, 2327.3906249999964, 0,
    1.9468471742839515, 7.6442714678225432, 2.1428201523778587,
    -6.4978677063941657, 732.21186240929228, -42420,
    -1.7776094797471198, -7.7296437623382461, -2.3577318244745555,
    6.4682627598226539, 728.39686240929416, 42420,
    -0.1940816098776379, 7.9939403756395437, 4.4708209830344146,
    -5.6020616237357697, 860.86747233072936, 0,
    -7.467835549570184, -5.949010396074943, 4.2851289247191602,
    8.2415443318103563, 853.85325515671457, 0,
    -51.490194366363788, 150.07842905745466, 131.78180231573768,
    -79.575473550461822, 2467.5, 0,
    -2.0767861445900437, -0.60824766145748344, 1.7513750706830005,
    1.5452292212374961, 3.8999999999999995, 0,
    15.414920859183034, 22.825870781345131, -3.2031334664911491,
    -24.539539681774503, 3109.1215106139766, 0,
    -0.12206112941742642, 3.5573733582133977, 2.0252475420222815,
    -2.4738706678818829, 356.5874231716374, -17682,
    0.37089731444078289, -2.7755130442413343, -1.8557902907744903,
    1.7826695863312867, 279.9213647816191, 56532,
};
const std::vector<double> kGolden_torque_6r_shipped_loose = {
    0.16859800818720766, -1.284430544142992, 1.412748293515969,
    -1.529527408444711, -1.513626786963971, 0.056381895561385335,
    0.20723669591725716, -1.3032159792697957, 1.4375108958600751,
    -1.5140558260540979, -1.5012248551643173, 0.068752721063892755,
    0.59035817515702815, -0.29103943270132687, 0.3862614989009,
    0.24138846019385102, 0.19330582448190434, 0.19306985684777545,
    7.8335270652385944e-11, 0.69999989417309105,
};
// clang-format on
// GOLDEN-CONSTANTS-END

// Default parameters, the pre-solve path: both QPs and the stop.
TEST(MpcSegmentCoreGolden, DefaultCoreOnThePresolvePath) {
  const ArmModel arm = Synthetic6R();
  const MpcSegmentCoreParams p;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
            MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.30, -0.25, 0.20, -0.15, 0.35, -0.30;
  in.qdd0 << 1.0, -0.5, 0.8, 0.0, -1.2, 0.4;
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  ASSERT_TRUE(res.presolved);
  Probe probe;
  AddQp(probe, mpc.PresolveQp());
  AddQp(probe, mpc.MainQp());
  AddSolution(probe, res);
  ExpectGolden("kinematic_6r", probe, kGolden_kinematic_6r_tight, kGolden_kinematic_6r_loose);
}

// w_⊥ on (the frame-Jacobian path the E1-F01 dense oracle cannot check,
// because both assemblies share AssemblePerp), a warm cycle on a shifted
// reference, armature added.
TEST(MpcSegmentCoreGolden, PerpendicularTermOnAWarmCycle) {
  const ArmModel arm = RealArm7();
  MpcSegmentCoreParams p;
  p.w_perp = 10.0;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model, 0.2)),
            MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  pinocchio::Data data(*arm.model);
  pinocchio::framesForwardKinematics(*arm.model, data, arm.q_nominal);
  in.p_c = data.oMf[arm.frame].translation() + Eigen::Vector3d(0.02, -0.01, 0.03);
  in.d_hat = Eigen::Vector3d(0.3, -0.4, 0.2).normalized();
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  MpcSegmentCoreInput warm = in;
  ShiftUniform(res, p.dt, 0.02, warm);
  ASSERT_TRUE(mpc.Solve(warm, res)) << MpcSegmentCoreReasonName(res.reason);
  ASSERT_FALSE(res.presolved);
  Probe probe;
  AddQp(probe, mpc.MainQp());
  AddSolution(probe, res);
  ExpectGolden("perp_7r_warm", probe, kGolden_perp_7r_warm_tight, kGolden_perp_7r_warm_loose);
}

// The shipped stop horizon (MD-24) with a torque row that binds.
TEST(MpcSegmentCoreGolden, ShippedHorizonWithABindingTorqueRow) {
  const ArmModel arm = RealArm6();
  MpcSegmentCoreParams p;
  p.n_nodes = 14;
  p.dt = 0.025;
  p.n_blocks = 6;
  p.block_sizes = {1, 1, 2, 2, 4, 4};
  MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.1);
  // Fixed numbers, not a search: the first joint's limit sits below what this
  // stop asks of it, so its rows bind and the slack is exercised.
  lim.tau_max *= 10.0;
  lim.tau_max[0] = 15.0;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 1.2, -0.6, 0.8, 0.5, 0.4, 0.4;
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  // The premise: a torque row is at (or past) its bound.
  ASSERT_GE(res.tau_ratio_max, p.eta_tau - 1e-3) << "the torque rows are not active";
  Probe probe;
  AddQp(probe, mpc.PresolveQp());
  AddQp(probe, mpc.MainQp());
  AddSolution(probe, res);
  ExpectGolden("torque_6r_shipped", probe, kGolden_torque_6r_shipped_tight,
               kGolden_torque_6r_shipped_loose);
}

// ── Approach-grid fixtures ───────────────────────────────────────────────────

using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

struct Grid {
  int n_pre;
  double dt_pre;
  int n_stop;
  double dt;
  int pre_block;                 // nodes per pre-catch block
  std::vector<int> stop_blocks;  // Σ = n_stop
};

// Two spacings, the catch node strictly inside: every "is the grid uniform?"
// shortcut gives a different answer here.
const Grid kSmallGrid{4, 0.05, 6, 0.025, 1, {1, 2, 3}};

MpcSegmentCoreParams GridParams(const Grid& g) {
  MpcSegmentCoreParams p;
  p.n_pre = g.n_pre;
  p.dt_pre = g.dt_pre;
  p.n_nodes = g.n_stop;
  p.dt = g.dt;
  p.block_sizes.fill(0);
  std::size_t b = 0;
  for (int k = 0; k < g.n_pre; k += g.pre_block) {
    p.block_sizes[b++] = std::min(g.pre_block, g.n_pre - k);
  }
  for (const int s : g.stop_blocks) {
    p.block_sizes[b++] = s;
  }
  p.n_blocks = static_cast<int>(b);
  return p;
}

// Node instants, computed here and not read from the core.
std::vector<double> NodeTimes(const Grid& g) {
  std::vector<double> t;
  for (int k = 0; k <= g.n_pre + g.n_stop; ++k) {
    t.push_back(k <= g.n_pre ? k * g.dt_pre : g.n_pre * g.dt_pre + (k - g.n_pre) * g.dt);
  }
  return t;
}

// The catch frame's position, +z axis and velocity from pinocchio's forward
// kinematics — none of the core's Jacobian code.
struct CatchPose {
  Eigen::Vector3d p, z, v;
};

CatchPose CatchPoseAt(const ArmModel& arm, pinocchio::Data& data, const Eigen::VectorXd& q,
                      const Eigen::VectorXd& qd) {
  pinocchio::forwardKinematics(*arm.model, data, q, qd);
  pinocchio::updateFramePlacements(*arm.model, data);
  CatchPose c;
  c.p = data.oMf[arm.frame].translation();
  c.z = data.oMf[arm.frame].rotation().col(2);
  c.v = pinocchio::getFrameVelocity(*arm.model, data, arm.frame, pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();
  return c;
}

// z tilted by theta toward (the part of `hint` across z).
Eigen::Vector3d TiltedAxis(const Eigen::Vector3d& z, double theta, const Eigen::Vector3d& hint) {
  const Eigen::Vector3d u = (hint - hint.dot(z) * z).normalized();
  return std::cos(theta) * z + std::sin(theta) * u;
}

// Symmetric positive definite with no axis aligned to the world's: a weight
// that is silently treated as diagonal, or transposed against a non-symmetric
// factor, changes the answer.
Eigen::Matrix3d SkewWeight(double a, double b, double c) {
  const Eigen::Matrix3d r =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
  const Eigen::Matrix3d m = r * Eigen::Vector3d(a, b, c).asDiagonal() * r.transpose();
  return 0.5 * (m + m.transpose());
}

// Rest-to-rest minimum-jerk reach from q0 to q1 arriving AT the catch node,
// held after it; x_0 on it. What a planner would hand the first solve of a plan.
void ReachReference(const MpcSegmentCore& mpc, const Eigen::VectorXd& q0, const Eigen::VectorXd& q1,
                    MpcSegmentCoreInput& in) {
  const int N = mpc.NumNodes();
  const double T = mpc.NodeTime(mpc.CatchNode());
  const Eigen::Index n = q0.size();
  const Eigen::VectorXd d = q1 - q0;
  in.q_ref.resize(n, N + 1);
  in.qd_ref.resize(n, N + 1);
  in.qdd_ref.resize(n, N + 1);
  for (int k = 0; k <= N; ++k) {
    const double s = std::min(mpc.NodeTime(k) / T, 1.0);
    const double s2 = s * s;
    in.q_ref.col(k) = q0 + d * (10.0 * s2 * s - 15.0 * s2 * s2 + 6.0 * s2 * s2 * s);
    in.qd_ref.col(k) = d * ((30.0 * s2 - 60.0 * s2 * s + 30.0 * s2 * s2) / T);
    in.qdd_ref.col(k) = d * ((60.0 * s - 180.0 * s2 + 120.0 * s2 * s) / (T * T));
  }
  in.reference_valid = true;
  in.q0 = q0;
  in.qd0 = Eigen::VectorXd::Zero(n);
  in.qdd0 = Eigen::VectorXd::Zero(n);
}

// ── 1. Grid ──────────────────────────────────────────────────────────────────

// Response of the scalar triple integrator at instant t to unit jerk on
// [t0, t1] — the integral form on ARBITRARY instants.
double UnitJerkResponseAt(int m, double t, double t0, double t1) {
  if (t0 >= t) {
    return 0.0;
  }
  const double a = t - t0;
  const double b = t - std::min(t1, t);
  switch (m) {
    case 0:
      return (a * a * a - b * b * b) / 6.0;
    case 1:
      return (a * a - b * b) / 2.0;
    default:
      return std::min(t1, t) - t0;
  }
}

TEST(MpcSegmentCoreApproach, StageGainsMatchTheIntegralOracleOnAMixedGrid) {
  const ArmModel arm = Synthetic6R();
  for (const Grid& g : {kSmallGrid, Grid{5, 0.04, 6, 0.025, 2, {2, 2, 2}}}) {
    const MpcSegmentCoreParams p = GridParams(g);
    MpcSegmentCore mpc;
    ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
              MpcSegmentCoreReason::kNone);
    const std::vector<double> t = NodeTimes(g);
    const int N = g.n_pre + g.n_stop;
    ASSERT_EQ(mpc.NumNodes(), N);
    ASSERT_EQ(mpc.CatchNode(), g.n_pre);
    for (int k = 0; k <= N; ++k) {
      EXPECT_NEAR(mpc.NodeTime(k), t[static_cast<std::size_t>(k)], 1e-15);
    }
    for (int m = 0; m < 3; ++m) {
      for (int k = 0; k <= N; ++k) {
        int k0 = 0;
        for (int b = 0; b < p.n_blocks; ++b) {
          const int size = p.block_sizes[static_cast<std::size_t>(b)];
          double oracle = 0.0;
          for (int i = k0; i < k0 + size; ++i) {
            oracle += UnitJerkResponseAt(m, t[static_cast<std::size_t>(k)],
                                         t[static_cast<std::size_t>(i)],
                                         t[static_cast<std::size_t>(i + 1)]);
          }
          k0 += size;
          EXPECT_NEAR(mpc.StageGain(m, k, b), p.u_scale * oracle,
                      1e-12 * p.u_scale + 1e-9 * std::abs(p.u_scale * oracle))
              << "m=" << m << " k=" << k << " b=" << b;
        }
      }
    }
    EXPECT_EQ(mpc.TerminalRank(), 2 * arm.model->nv);
  }
}

// The jerk cost is a time integral: a block's weight is Σ Δ_k/Δ_s, so two
// 0.05 s nodes before the catch weigh four 0.025 s nodes after it.
TEST(MpcSegmentCoreApproach, JerkCostIsWeightedByTheIntervalLength) {
  const ArmModel arm = Synthetic6R();
  const int n = arm.model->nv;
  MpcSegmentCoreParams p = GridParams(Grid{4, 0.05, 6, 0.025, 2, {1, 2, 3}});  // blocks 2,2 | 1,2,3
  p.w_delta = 0.0;
  Eigen::VectorXd r(n);
  r << 1.0, 2.0, 0.5, 3.0, 1.5, 0.7;
  p.jerk_weight = r;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
            MpcSegmentCoreReason::kNone);
  const Eigen::MatrixXd& h = mpc.MainQp().H;
  const double expected[5] = {2.0 * (0.05 / 0.025), 2.0 * (0.05 / 0.025), 1.0, 2.0, 3.0};
  for (int b = 0; b < 5; ++b) {
    for (int j = 0; j < n; ++j) {
      EXPECT_NEAR(h(b * n + j, b * n + j), r[j] * expected[b], 1e-12) << "b=" << b << " j=" << j;
    }
  }
}

TEST(MpcSegmentCoreApproach, InitValidatesTheGridAndTheCatchParameters) {
  const ArmModel arm = Synthetic6R();
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model);
  const auto init = [&](const MpcSegmentCoreParams& p) {
    MpcSegmentCore mpc;
    return mpc.Init(*arm.model, arm.frame, p, lim);
  };
  ASSERT_EQ(init(GridParams(kSmallGrid)), MpcSegmentCoreReason::kNone);

  // A block across the catch node (pre-catch nodes 0..3, a 2-node block on 3..4).
  MpcSegmentCoreParams across = GridParams(kSmallGrid);
  across.n_blocks = 5;
  across.block_sizes = {1, 2, 2, 2, 3};
  EXPECT_EQ(init(across), MpcSegmentCoreReason::kBlocksAcrossCatch);

  // Six blocks in all, but only two after the catch node: the terminal
  // equality would fix both. The Init rank self-check cannot see this.
  MpcSegmentCoreParams two_after = GridParams(Grid{4, 0.05, 6, 0.025, 1, {3, 3}});
  ASSERT_EQ(two_after.n_blocks, 6);
  EXPECT_EQ(init(two_after), MpcSegmentCoreReason::kBlocksTooFew);

  // Capacities: the stop segment keeps the payload's bound, the horizon has
  // the core's own, and the block array has kMaxSegmentNodes slots.
  MpcSegmentCoreParams long_stop = GridParams(kSmallGrid);
  long_stop.n_nodes = rtc::catching::kMaxSegmentNodes + 1;
  EXPECT_EQ(init(long_stop), MpcSegmentCoreReason::kParamsInvalid);
  const int max_pre = rtc::catching::kMaxMpcNodes - 6;
  EXPECT_EQ(init(GridParams(Grid{max_pre, 0.05, 6, 0.025, 2, {1, 2, 3}})),
            MpcSegmentCoreReason::kNone);
  EXPECT_EQ(init(GridParams(Grid{max_pre + 1, 0.05, 6, 0.025, 3, {1, 2, 3}})),
            MpcSegmentCoreReason::kParamsInvalid);
  MpcSegmentCoreParams many_blocks = GridParams(Grid{12, 0.05, 14, 0.025, 1, {1, 1, 2, 2, 4, 4}});
  ASSERT_EQ(init(many_blocks), MpcSegmentCoreReason::kNone);  // 18 blocks, 26 nodes
  many_blocks = GridParams(Grid{12, 0.05, 14, 0.025, 1, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2}});
  many_blocks.n_blocks = rtc::catching::kMaxSegmentNodes + 1;  // 25 > the array
  EXPECT_EQ(init(many_blocks), MpcSegmentCoreReason::kParamsInvalid);

  MpcSegmentCoreParams p = GridParams(kSmallGrid);
  p.dt_pre = 0.0;
  EXPECT_EQ(init(p), MpcSegmentCoreReason::kParamsInvalid) << "dt_pre";
  p = GridParams(kSmallGrid);
  p.dt_pre = kNan;
  EXPECT_EQ(init(p), MpcSegmentCoreReason::kParamsInvalid) << "dt_pre NaN";
  // dt_pre enters the node times even when no pre-catch node uses it: a core
  // that accepted a non-finite one would report a good Init and fail every Solve.
  ASSERT_EQ(init(MpcSegmentCoreParams{}), MpcSegmentCoreReason::kNone);
  for (const double bad : {kNan, kInf, -0.05}) {
    p = MpcSegmentCoreParams{};
    p.dt_pre = bad;
    EXPECT_EQ(init(p), MpcSegmentCoreReason::kParamsInvalid)
        << "dt_pre " << bad << " with n_pre = 0";
  }
  p = MpcSegmentCoreParams{};
  p.catch_terms = true;  // no pre-catch node: the catch node would be x_0
  EXPECT_EQ(init(p), MpcSegmentCoreReason::kParamsInvalid) << "catch_terms without n_pre";
  for (double MpcSegmentCoreParams::*field :
       {&MpcSegmentCoreParams::w_axis, &MpcSegmentCoreParams::w_v_par,
        &MpcSegmentCoreParams::w_v_perp, &MpcSegmentCoreParams::rho_v,
        &MpcSegmentCoreParams::v_rel_allow}) {
    for (const double bad : {-1.0, kNan, kInf}) {
      p = GridParams(kSmallGrid);
      p.catch_terms = true;
      p.*field = bad;
      EXPECT_EQ(init(p), MpcSegmentCoreReason::kParamsInvalid) << bad;
    }
  }
  p = GridParams(kSmallGrid);
  p.catch_terms = true;
  p.rho_v = 1.0;  // slack on, no bound to be slack against
  p.v_rel_allow = 0.0;
  EXPECT_EQ(init(p), MpcSegmentCoreReason::kParamsInvalid) << "rho_v without v_rel_allow";
  for (const double bad : {0.0, -0.1, 3.2, kNan}) {
    p = GridParams(kSmallGrid);
    p.axis_theta_max = bad;
    EXPECT_EQ(init(p), MpcSegmentCoreReason::kParamsInvalid) << "axis_theta_max " << bad;
  }
}

// ── 2. The linearisation seam ────────────────────────────────────────────────

struct Seam {
  Eigen::MatrixXd j6;
  Mat3X j_v, j_w, l_a, h_v, dv;
  rtc::catching::CatchLinearization lin;
  MpcSegmentCoreReason why{MpcSegmentCoreReason::kNone};
};

Seam Linearize(const ArmModel& arm, pinocchio::Data& data, const Eigen::VectorXd& q,
               const Eigen::VectorXd& v, const Eigen::Vector3d& a_d, double theta_max = 3.0) {
  const Eigen::Index n = arm.model->nv;
  Seam s;
  s.j6.setZero(6, n);
  for (Mat3X* m : {&s.j_v, &s.j_w, &s.l_a, &s.h_v, &s.dv}) {
    m->setZero(3, n);
  }
  s.why = rtc::catching::LinearizeCatchAt(*arm.model, data, arm.frame, q, v, a_d, theta_max, true,
                                          true, s.j6, s.j_v, s.j_w, s.l_a, s.h_v, s.dv, s.lin);
  return s;
}

// Each Jacobian against a central difference of the NONLINEAR output. The
// fixture is built to break the symmetries a wrong Jacobian hides behind: a
// catch frame with a lever arm and a tilt, v ≠ 0 (H_v ≡ 0 otherwise), and a
// target axis off every world axis and off z.
TEST(CatchLinearization, JacobiansMatchCentralDifferences) {
  for (const ArmModel& arm : {Synthetic6R(), RealArm7()}) {
    const Eigen::Index n = arm.model->nv;
    pinocchio::Data data(*arm.model);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    for (int trial = 0; trial < 5; ++trial) {
      Eigen::VectorXd q = arm.q_nominal;
      Eigen::VectorXd v(n);
      for (Eigen::Index j = 0; j < n; ++j) {
        q[j] += 0.5 * uni(rng);
        v[j] = uni(rng);
      }
      const CatchPose at = CatchPoseAt(arm, data, q, v);
      const Eigen::Vector3d a_d =
          TiltedAxis(at.z, 0.3 + 0.1 * trial, Eigen::Vector3d(0.3, -0.5, 0.8));
      const Seam s = Linearize(arm, data, q, v, a_d);
      ASSERT_EQ(s.why, MpcSegmentCoreReason::kNone) << arm.name;
      EXPECT_LE((s.lin.p - at.p).norm(), 1e-12);
      EXPECT_LE((s.lin.z - at.z).norm(), 1e-12);
      EXPECT_NEAR(s.lin.e_a.norm(), 0.3 + 0.1 * trial, 1e-9);
      // J_v is also the velocity map.
      EXPECT_LE((s.j_v * v - at.v).norm(), 1e-10) << arm.name;
      ASSERT_GT(s.h_v.norm(), 1e-2) << "H_v vanished: the fixture does not exercise it";

      const double h = 1e-6;
      for (Eigen::Index j = 0; j < n; ++j) {
        Eigen::VectorXd qp = q;
        Eigen::VectorXd qm = q;
        qp[j] += h;
        qm[j] -= h;
        const CatchPose cp = CatchPoseAt(arm, data, qp, v);
        const CatchPose cm = CatchPoseAt(arm, data, qm, v);
        const Eigen::Vector3d dp = (cp.p - cm.p) / (2.0 * h);
        const Eigen::Vector3d de = (rtc::math::se3::AxisAlignError(cp.z, a_d).error -
                                    rtc::math::se3::AxisAlignError(cm.z, a_d).error) /
                                   (2.0 * h);
        const Eigen::Vector3d dv = (cp.v - cm.v) / (2.0 * h);
        const auto tol = [](const Eigen::Vector3d& x) { return 1e-6 * std::max(1.0, x.norm()); };
        EXPECT_LE((s.j_v.col(j) - dp).norm(), tol(dp)) << arm.name << " J_v col " << j;
        EXPECT_LE((s.l_a.col(j) - de).norm(), tol(de)) << arm.name << " L_a col " << j;
        EXPECT_LE((s.h_v.col(j) - dv).norm(), tol(dv))
            << arm.name << " H_v col " << j << ": " << s.h_v.col(j).transpose() << " vs "
            << dv.transpose();
      }
    }
  }
}

// rtc_math returns J_a = 0 inside the aligned deadband; the seam substitutes
// the limit so the axis term keeps its curvature when the reference is aligned.
TEST(CatchLinearization, AxisJacobianIsContinuousThroughAlignment) {
  const ArmModel arm = Synthetic6R();
  pinocchio::Data data(*arm.model);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(arm.model->nv);
  const Eigen::Vector3d z = CatchPoseAt(arm, data, arm.q_nominal, v).z;
  const Eigen::Vector3d hint(0.3, -0.5, 0.8);

  const Seam aligned = Linearize(arm, data, arm.q_nominal, v, z);
  ASSERT_EQ(aligned.why, MpcSegmentCoreReason::kNone);
  EXPECT_LE(aligned.lin.e_a.norm(), 1e-12);
  // The limit: [z]×[z]× J_ω = −(I − z zᵀ) J_ω — rank 2, not zero.
  const Eigen::MatrixXd limit = (z * z.transpose() - Eigen::Matrix3d::Identity()) * aligned.j_w;
  EXPECT_LE((aligned.l_a - limit).norm(), 1e-12);
  EXPECT_EQ(Rank(aligned.l_a), 2);

  // 1e-7 is INSIDE the deadband (sinθ < 1e-6), the others outside.
  for (const double theta : {1e-2, 1e-4, 1e-7}) {
    const Seam s = Linearize(arm, data, arm.q_nominal, v, TiltedAxis(z, theta, hint));
    ASSERT_EQ(s.why, MpcSegmentCoreReason::kNone) << theta;
    EXPECT_LE((s.l_a - aligned.l_a).norm(), 2.0 * theta * aligned.j_w.norm() + 1e-12)
        << "theta " << theta;
  }

  EXPECT_EQ(Linearize(arm, data, arm.q_nominal, v, TiltedAxis(z, 1.0, hint), 0.5).why,
            MpcSegmentCoreReason::kCatchAxisOutOfRange);
  EXPECT_EQ(Linearize(arm, data, arm.q_nominal, v, TiltedAxis(z, 0.4, hint), 0.5).why,
            MpcSegmentCoreReason::kNone);
  EXPECT_EQ(Linearize(arm, data, arm.q_nominal, v, -z, 3.1).why,
            MpcSegmentCoreReason::kCatchAxisOutOfRange)
      << "antiparallel";
  EXPECT_EQ(Linearize(arm, data, arm.q_nominal, v, 2.0 * z).why,
            MpcSegmentCoreReason::kCatchAxisOutOfRange)
      << "a_d not unit";
  Seam bad = Linearize(arm, data, arm.q_nominal, v, z);
  bad.j_v.resize(3, 2);
  EXPECT_EQ(rtc::catching::LinearizeCatchAt(*arm.model, data, arm.frame, arm.q_nominal, v, z, 3.0,
                                            true, true, bad.j6, bad.j_v, bad.j_w, bad.l_a, bad.h_v,
                                            bad.dv, bad.lin),
            MpcSegmentCoreReason::kDimMismatch);
}

// ── 3. The assembled catch cost against the nonlinear cost ───────────────────
// What the Jacobian test cannot see: the residual's CONSTANT part (the offset
// of the free response from the reference, the sign of p̂_b and of γ v̂_b), the
// weight, and the condensing through the stage gains. The catch terms'
// contribution to the QP is isolated as the difference between two cores that
// differ only in catch_terms; it is a quadratic F_lin(z). The reference is
// placed so that the trajectory of a chosen z* passes THROUGH it at the catch
// node — there the linear model is exact and its gradient is the nonlinear
// cost's gradient, so along z* + h·z_0 the two costs differ by O(h²) only. A
// wrong constant, sign or weight leaves an O(h) error and the ratio test fails.
struct TermCase {
  const char* name;
  bool pos, axis, vel;
};

class CatchCostOrder : public ::testing::TestWithParam<TermCase> {};

TEST_P(CatchCostOrder, MatchesTheNonlinearCostToSecondOrder) {
  const TermCase tc = GetParam();
  const ArmModel arm = Synthetic6R();
  const int n = arm.model->nv;
  MpcSegmentCoreParams off = GridParams(kSmallGrid);
  off.rho_tau = 0.0;
  off.w_delta = 0.0;
  off.delta_tr = kInf;
  MpcSegmentCoreParams on = off;
  on.catch_terms = true;
  on.w_axis = tc.axis ? 40.0 : 0.0;
  on.w_v_par = tc.vel ? 3.0 : 0.0;
  on.w_v_perp = tc.vel ? 25.0 : 0.0;
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model);
  MpcSegmentCore with, without;
  ASSERT_EQ(with.Init(*arm.model, arm.frame, on, lim), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(without.Init(*arm.model, arm.frame, off, lim), MpcSegmentCoreReason::kNone);
  const int kc = with.CatchNode();
  const int N = with.NumNodes();
  const int nb = with.NumBlocks();
  const int nu = n * nb;

  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.20, -0.10, 0.15, -0.20, 0.10, 0.05;
  in.qdd0 << 0.5, -0.3, 0.4, 0.2, -0.5, 0.3;
  const double t = with.NodeTime(kc);
  const Eigen::VectorXd q_f = in.q0 + t * in.qd0 + 0.5 * t * t * in.qdd0;
  const Eigen::VectorXd v_f = in.qd0 + t * in.qdd0;
  const auto state_at_catch = [&](const Eigen::VectorXd& z, Eigen::VectorXd& q,
                                  Eigen::VectorXd& v) {
    q = q_f;
    v = v_f;
    for (int b = 0; b < nb; ++b) {
      q += with.StageGain(0, kc, b) * z.segment(b * n, n);
      v += with.StageGain(1, kc, b) * z.segment(b * n, n);
    }
  };
  Eigen::VectorXd z_star(nu), z_dir(nu);
  for (int b = 0; b < nb; ++b) {
    for (int j = 0; j < n; ++j) {
      z_star[b * n + j] = 0.02 * std::sin(0.9 * b + 0.5 * j + 0.3);
      z_dir[b * n + j] = 0.02 * std::cos(1.7 * b + 0.8 * j + 0.1);
    }
  }
  Eigen::VectorXd q_star(n), v_star(n);
  state_at_catch(z_star, q_star, v_star);
  ASSERT_GT((q_star - q_f).norm(), 5e-3) << "the reference must sit OFF the free response";
  ASSERT_GT(v_star.norm(), 0.1) << "H_v needs a moving reference";

  // The reference only has to pass through (q*, v*) at the catch node and end
  // at rest; it is a linearisation point, not a trajectory.
  in.q_ref = q_star.replicate(1, N + 1);
  in.qd_ref.setZero(n, N + 1);
  in.qd_ref.col(kc) = v_star;
  in.qdd_ref.setZero(n, N + 1);
  in.reference_valid = true;

  pinocchio::Data data(*arm.model);
  const CatchPose star = CatchPoseAt(arm, data, q_star, v_star);
  in.p_b = star.p + Eigen::Vector3d(0.03, -0.02, 0.04);
  in.a_d = TiltedAxis(star.z, 0.25, Eigen::Vector3d(0.3, -0.5, 0.8));
  in.v_b = Eigen::Vector3d(0.8, -0.5, 0.6);
  in.gamma_ref = 0.7;
  in.w_p = tc.pos ? SkewWeight(900.0, 2500.0, 400.0) : Eigen::Matrix3d(Eigen::Matrix3d::Zero());

  MpcSegmentCoreResult r_on, r_off;
  with.ResizeResult(r_on);
  without.ResizeResult(r_off);
  // The QP itself is not the subject; both are assembled before it runs.
  (void)with.Solve(in, r_on);
  (void)without.Solve(in, r_off);
  const Eigen::MatrixXd h_c =
      with.MainQp().H.topLeftCorner(nu, nu) - without.MainQp().H.topLeftCorner(nu, nu);
  const Eigen::VectorXd g_c = with.MainQp().g.head(nu) - without.MainQp().g.head(nu);
  ASSERT_GT(h_c.norm(), 0.0) << "the catch terms added nothing";
  EXPECT_EQ((with.MainQp().H - with.MainQp().H.transpose()).cwiseAbs().maxCoeff(), 0.0)
      << "H must be exactly symmetric";

  const Eigen::Vector3d d_hat = in.v_b.normalized();
  const Eigen::Matrix3d w_v =
      on.w_v_par * d_hat * d_hat.transpose() +
      on.w_v_perp * (Eigen::Matrix3d::Identity() - d_hat * d_hat.transpose());
  const auto f_nl = [&](const Eigen::VectorXd& z) {
    Eigen::VectorXd q(n), v(n);
    state_at_catch(z, q, v);
    const CatchPose c = CatchPoseAt(arm, data, q, v);
    double f = 0.0;
    if (tc.pos) {
      const Eigen::Vector3d r = c.p - in.p_b;
      f += 0.5 * r.dot(in.w_p * r);
    }
    if (tc.axis) {
      f += 0.5 * on.w_axis * rtc::math::se3::AxisAlignError(c.z, in.a_d).error.squaredNorm();
    }
    if (tc.vel) {
      const Eigen::Vector3d r = c.v - in.gamma_ref * in.v_b;
      f += 0.5 * r.dot(w_v * r);
    }
    return f;
  };
  const auto f_lin = [&](const Eigen::VectorXd& z) { return 0.5 * z.dot(h_c * z) + g_c.dot(z); };
  double change = 0.0;
  const auto error_at = [&](double h) {
    const Eigen::VectorXd z = z_star + h * z_dir;
    change = f_nl(z) - f_nl(z_star);
    return std::abs((f_lin(z) - f_lin(z_star)) - change);
  };
  const double e1 = error_at(1.0);
  const double change1 = std::abs(change);
  const double e2 = error_at(0.5);
  ASSERT_GT(change1, 1e-6) << tc.name << ": the cost did not move — nothing is being compared";
  ASSERT_GT(e1, 1e-10) << tc.name << ": the error is below what can be measured";
  EXPECT_LT(e1, 0.2 * change1) << tc.name << ": first-order mismatch";
  const double ratio = e1 / e2;
  EXPECT_GT(ratio, 3.0) << tc.name << " e(h)=" << e1 << " e(h/2)=" << e2;
  EXPECT_LT(ratio, 5.5) << tc.name << " e(h)=" << e1 << " e(h/2)=" << e2;
}

INSTANTIATE_TEST_SUITE_P(Terms, CatchCostOrder,
                         ::testing::Values(TermCase{"position", true, false, false},
                                           TermCase{"axis", false, true, false},
                                           TermCase{"velocity", false, false, true},
                                           TermCase{"all", true, true, true}),
                         [](const ::testing::TestParamInfo<TermCase>& i) {
                           return std::string(i.param.name);
                         });

// The stop-path term w_⊥ on a grid with pre-catch nodes. The cost it models is
// the off-line distance over the STOP segment only (nodes k_c..N): the line
// through p_c is where the hand stops, not how it approaches. Same construction
// as CatchCostOrder — the term is the difference of two cores, and the
// reference is the trajectory of z* itself, so every node's linear model is
// exact there and the two costs differ by O(h²). A term that also covered the
// pre-catch nodes leaves an O(h) error. The grid has a LONG pre-catch part so
// those nodes carry a measurable share of the cost change, and the step is
// small so the O(h²) error sits far below that share.
TEST(MpcSegmentCoreApproach, StopPathTermCoversTheStopSegmentOnly) {
  const ArmModel arm = Synthetic6R();
  const int n = arm.model->nv;
  MpcSegmentCoreParams off = GridParams(Grid{6, 0.1, 4, 0.05, 1, {1, 1, 2}});
  off.rho_tau = 0.0;
  off.w_delta = 0.0;
  off.delta_tr = kInf;
  MpcSegmentCoreParams on = off;
  on.w_perp = 300.0;
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model);
  MpcSegmentCore with, without;
  ASSERT_EQ(with.Init(*arm.model, arm.frame, on, lim), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(without.Init(*arm.model, arm.frame, off, lim), MpcSegmentCoreReason::kNone);
  const int kc = with.CatchNode();
  const int N = with.NumNodes();
  const int nb = with.NumBlocks();
  const int nu = n * nb;
  ASSERT_GT(kc, 1);

  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.20, -0.10, 0.15, -0.20, 0.10, 0.05;
  in.qdd0 << 0.5, -0.3, 0.4, 0.2, -0.5, 0.3;
  const auto node_q = [&](const Eigen::VectorXd& z, int k) {
    const double t = with.NodeTime(k);
    Eigen::VectorXd q = in.q0 + t * in.qd0 + 0.5 * t * t * in.qdd0;
    for (int b = 0; b < nb; ++b) {
      q += with.StageGain(0, k, b) * z.segment(b * n, n);
    }
    return q;
  };
  Eigen::VectorXd z_star(nu), z_dir(nu);
  for (int b = 0; b < nb; ++b) {
    for (int j = 0; j < n; ++j) {
      z_star[b * n + j] = 0.004 * std::sin(0.9 * b + 0.5 * j + 0.3);
      z_dir[b * n + j] = 4e-6 * std::cos(1.7 * b + 0.8 * j + 0.1);
    }
  }
  in.q_ref.resize(n, N + 1);
  for (int k = 0; k <= N; ++k) {
    in.q_ref.col(k) = node_q(z_star, k);
  }
  in.qd_ref.setZero(n, N + 1);
  in.qdd_ref.setZero(n, N + 1);
  in.reference_valid = true;

  pinocchio::Data data(*arm.model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
  in.p_c = CatchPoseAt(arm, data, node_q(z_star, kc), zero).p + Eigen::Vector3d(0.03, -0.02, 0.04);
  in.d_hat = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
  const Eigen::Matrix3d p_perp = Eigen::Matrix3d::Identity() - in.d_hat * in.d_hat.transpose();

  MpcSegmentCoreResult r_on, r_off;
  with.ResizeResult(r_on);
  without.ResizeResult(r_off);
  // The QP itself is not the subject; both are assembled before it runs.
  (void)with.Solve(in, r_on);
  (void)without.Solve(in, r_off);
  const Eigen::MatrixXd h_c =
      with.MainQp().H.topLeftCorner(nu, nu) - without.MainQp().H.topLeftCorner(nu, nu);
  const Eigen::VectorXd g_c = with.MainQp().g.head(nu) - without.MainQp().g.head(nu);
  ASSERT_GT(h_c.norm(), 0.0) << "the stop-path term added nothing";

  // The off-line cost summed over nodes k_first..N.
  const auto f_nl = [&](const Eigen::VectorXd& z, int k_first) {
    double f = 0.0;
    for (int k = k_first; k <= N; ++k) {
      const Eigen::Vector3d r = p_perp * (CatchPoseAt(arm, data, node_q(z, k), zero).p - in.p_c);
      f += 0.5 * on.w_perp * r.squaredNorm();
    }
    return f;
  };
  const auto f_lin = [&](const Eigen::VectorXd& z) { return 0.5 * z.dot(h_c * z) + g_c.dot(z); };
  double change = 0.0;
  const auto error_at = [&](double h) {
    const Eigen::VectorXd z = z_star + h * z_dir;
    change = f_nl(z, kc) - f_nl(z_star, kc);
    return std::abs((f_lin(z) - f_lin(z_star)) - change);
  };
  const double e1 = error_at(1.0);
  const double change1 = std::abs(change);
  const double e2 = error_at(0.5);
  // The pre-catch nodes' share of the cost change: the scale of the O(h) error
  // a term over ALL nodes leaves. It must be measurable, and the error must
  // sit far below it.
  const Eigen::VectorXd z1 = z_star + z_dir;
  const double pre_change =
      std::abs((f_nl(z1, 1) - f_nl(z_star, 1)) - (f_nl(z1, kc) - f_nl(z_star, kc)));
  ASSERT_GT(change1, 1e-6) << "the cost did not move — nothing is being compared";
  ASSERT_GT(e1, 1e-10) << "the error is below what can be measured";
  ASSERT_GT(pre_change, 1e-2 * change1) << "the pre-catch nodes barely move the cost";
  EXPECT_LT(e1, 0.1 * pre_change) << "the term reaches the pre-catch nodes (or is wrong at first "
                                     "order): e(h)="
                                  << e1 << " change=" << change1;
  const double ratio = e1 / e2;
  EXPECT_GT(ratio, 3.0) << "e(h)=" << e1 << " e(h/2)=" << e2;
  EXPECT_LT(ratio, 5.5) << "e(h)=" << e1 << " e(h/2)=" << e2;
}

// ── 4. Structured assembly == dense assembly, catch terms included ───────────

// A reach on the 7-dof arm with every term on. Shared by the oracle, the
// allocation and the fail-closed tests.
struct ReachFixture {
  ArmModel arm;
  MpcSegmentCoreParams params;
  MpcSegmentCoreLimits limits;
  Eigen::VectorXd q_target;
  MpcSegmentCoreInput input;  // catch inputs set; the reference is added per core
};

ReachFixture MakeReach7() {
  ReachFixture f{RealArm7(), GridParams(kSmallGrid), {}, {}, {}};
  f.params.catch_terms = true;
  f.params.w_axis = 50.0;
  f.params.w_v_par = 2.0;
  f.params.w_v_perp = 20.0;
  f.params.rho_v = 2.0;
  f.params.v_rel_allow = 0.3;
  f.limits = LimitsFromModel(*f.arm.model, 0.2);
  f.q_target = f.arm.q_nominal;
  Eigen::VectorXd d(7);
  d << 0.10, -0.08, 0.06, 0.09, -0.07, 0.10, 0.05;
  f.q_target += d;
  pinocchio::Data data(*f.arm.model);
  const CatchPose target =
      CatchPoseAt(f.arm, data, f.q_target, Eigen::VectorXd::Zero(f.arm.model->nv));
  f.input = RestInput(f.arm.q_nominal);
  f.input.p_b = target.p + Eigen::Vector3d(0.015, -0.010, 0.020);
  f.input.a_d = TiltedAxis(target.z, 0.10, Eigen::Vector3d(0.3, -0.5, 0.8));
  f.input.v_b = Eigen::Vector3d(0.9, -0.4, -0.6);
  f.input.gamma_ref = 0.6;
  f.input.w_p = SkewWeight(2000.0, 5000.0, 1000.0);
  f.input.w_delta_scale = 0.0;
  f.input.cold_start = true;
  return f;
}

bool MatricesClose(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y) {
  if (x.rows() != y.rows() || x.cols() != y.cols()) {
    return false;
  }
  double scale = 1.0;
  double diff = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    const double xi = x.data()[i];
    const double yi = y.data()[i];
    if (!std::isfinite(xi) || !std::isfinite(yi)) {
      if (xi != yi) {
        return false;
      }
      continue;
    }
    scale = std::max(scale, std::abs(xi));
    diff = std::max(diff, std::abs(xi - yi));
  }
  return diff <= 1e-9 * scale;
}

TEST(MpcSegmentCoreApproach, StructuredCatchAssemblyMatchesDense) {
  ReachFixture f = MakeReach7();
  f.params.w_perp = 10.0;  // the stop-path term stays compatible
  MpcSegmentCoreParams pd = f.params;
  pd.reference_assembly = true;
  MpcSegmentCore fast, dense;
  ASSERT_EQ(fast.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(dense.Init(*f.arm.model, f.arm.frame, pd, f.limits), MpcSegmentCoreReason::kNone);
  // One slack variable and seven rows more than the E1-F01 layout.
  const int n = f.arm.model->nv;
  ASSERT_EQ(fast.MainQp().H.rows(), n * (fast.NumBlocks() + fast.NumNodes()) + 1);
  ASSERT_EQ(fast.MainQp().C.rows(), 5 * n * fast.NumNodes() + 7);
  MpcSegmentCoreResult rf, rd;
  fast.ResizeResult(rf);
  dense.ResizeResult(rd);
  MpcSegmentCoreInput in = f.input;
  in.p_c = in.p_b;
  in.d_hat = Eigen::Vector3d(1.0, 2.0, -0.5).normalized();
  ReachReference(fast, f.arm.q_nominal, f.q_target, in);
  for (int cycle = 0; cycle < 2; ++cycle) {
    if (cycle == 1) {
      UseAsReference(rf, in);
      in.w_delta_scale = 0.6;  // the scaled-w_Δ Hessian on both paths
      in.cold_start = false;
    }
    ASSERT_TRUE(fast.Solve(in, rf)) << MpcSegmentCoreReasonName(rf.reason);
    ASSERT_TRUE(dense.Solve(in, rd)) << MpcSegmentCoreReasonName(rd.reason);
    const rtc::tsid::QPData& a = fast.MainQp();
    const rtc::tsid::QPData& b = dense.MainQp();
    EXPECT_TRUE(MatricesClose(a.H, b.H)) << "H cycle " << cycle;
    EXPECT_TRUE(MatricesClose(a.g, b.g)) << "g cycle " << cycle;
    EXPECT_TRUE(MatricesClose(a.A, b.A)) << "A cycle " << cycle;
    EXPECT_TRUE(MatricesClose(a.b, b.b)) << "b cycle " << cycle;
    EXPECT_TRUE(MatricesClose(a.C, b.C)) << "C cycle " << cycle;
    EXPECT_TRUE(MatricesClose(a.l, b.l)) << "l cycle " << cycle;
    EXPECT_TRUE(MatricesClose(a.u, b.u)) << "u cycle " << cycle;
    EXPECT_EQ((a.H - a.H.transpose()).cwiseAbs().maxCoeff(), 0.0) << "H symmetric, cycle " << cycle;
    EXPECT_LE((rf.u - rd.u).cwiseAbs().maxCoeff(), 1e-6 * (1.0 + rd.u.cwiseAbs().maxCoeff()));
  }
}

// ── 5. Behaviour ─────────────────────────────────────────────────────────────

// formulation §4 item 5: nothing to do ⇒ do nothing. A sanity check only — every
// residual is zero, so this cannot see a sign or a transpose (sections 2–4 do).
// It does run a_d == z exactly, i.e. the aligned-deadband branch, in a full solve.
TEST(MpcSegmentCoreApproach, ZeroSolutionWhenAlreadyAtTheCatch) {
  const ArmModel arm = Synthetic6R();
  const int n = arm.model->nv;
  MpcSegmentCoreParams p = GridParams(kSmallGrid);
  p.catch_terms = true;
  p.w_axis = 10.0;
  p.w_v_par = 5.0;
  p.w_v_perp = 5.0;
  p.rho_v = 1.0;
  p.v_rel_allow = 0.2;
  MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model);
  pinocchio::Data data(*arm.model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
  const Eigen::VectorXd g = pinocchio::rnea(*arm.model, data, arm.q_nominal, zero, zero);
  lim.tau_max = lim.tau_max.cwiseMax(3.0 * g.cwiseAbs() / p.eta_tau);
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = RestInput(arm.q_nominal);
  in.q_ref = arm.q_nominal.replicate(1, mpc.NumNodes() + 1);
  in.qd_ref.setZero(n, mpc.NumNodes() + 1);
  in.qdd_ref.setZero(n, mpc.NumNodes() + 1);
  in.reference_valid = true;
  const CatchPose here = CatchPoseAt(arm, data, arm.q_nominal, zero);
  in.p_b = here.p;
  in.a_d = here.z;
  in.v_b.setZero();
  in.w_p = SkewWeight(1000.0, 3000.0, 500.0);
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  const double z_tol = 10.0 * p.solver.eps_abs;
  EXPECT_LE(res.u.cwiseAbs().maxCoeff(), z_tol * p.u_scale);
  EXPECT_TRUE(res.catch_evaluated);
  EXPECT_LE(res.catch_pos_err.norm(), 1e-6);
  EXPECT_LE(res.catch_axis_err, 1e-6);
  EXPECT_LE(res.catch_v_rel.norm(), 1e-6);
  EXPECT_EQ(res.catch_gamma, 0.0) << "a ball at rest has no direction: γ is reported as 0";
  EXPECT_LE(std::abs(res.slack_v), z_tol);
}

// A reach on the synthetic arm: the reference is aimed at a posture whose hand
// is a few cm and a few degrees off the catch pose, so the catch terms have
// work to do within the trust region.
struct Reach6 {
  ArmModel arm;
  MpcSegmentCoreParams params;
  MpcSegmentCoreLimits limits;
  Eigen::VectorXd q_target;
  Eigen::VectorXd q_seed;
  CatchPose target;
};

Reach6 MakeReach6() {
  Reach6 f{Synthetic6R(), GridParams(Grid{6, 0.05, 6, 0.025, 1, {1, 2, 3}}), {}, {}, {}, {}};
  f.params.catch_terms = true;
  f.params.w_axis = 200.0;
  f.limits = LimitsFromModel(*f.arm.model);
  f.limits.tau_max *= 10.0;  // torque rows present, not binding
  Eigen::VectorXd d(6), off(6);
  d << 0.25, -0.20, 0.30, 0.20, -0.25, 0.20;
  off << 0.06, -0.05, 0.07, -0.06, 0.08, -0.07;
  f.q_target = f.arm.q_nominal + d;
  f.q_seed = f.q_target + off;
  pinocchio::Data data(*f.arm.model);
  f.target = CatchPoseAt(f.arm, data, f.q_target, Eigen::VectorXd::Zero(6));
  return f;
}

// ½ Σ_k (Δ_k/Δ_s) Σ_j (u_kj/u_scale)² + the catch terms on the NONLINEAR pose.
double NonlinearObjective(const Reach6& f, const MpcSegmentCore& mpc, const MpcSegmentCoreInput& in,
                          const MpcSegmentCoreResult& r, pinocchio::Data& data) {
  const int kc = mpc.CatchNode();
  double j = 0.0;
  for (int k = 0; k < mpc.NumNodes(); ++k) {
    const double w = (mpc.NodeTime(k + 1) - mpc.NodeTime(k)) / f.params.dt;
    j += 0.5 * w * (r.u.col(k) / f.params.u_scale).squaredNorm();
  }
  const CatchPose c = CatchPoseAt(f.arm, data, r.q.col(kc), r.qd.col(kc));
  const Eigen::Vector3d rp = c.p - in.p_b;
  j += 0.5 * rp.dot(in.w_p * rp);
  j += 0.5 * f.params.w_axis * rtc::math::se3::AxisAlignError(c.z, in.a_d).error.squaredNorm();
  return j;
}

// Real-time iteration: one QP per cycle, the next reference is this solution.
// A fixed "< 1 mm after K cycles" would measure the weight ratio, not the
// iteration; what is asserted is that K cycles reach the CONVERGED solution
// (50 cycles) and that the nonlinear objective goes down on the way.
TEST(MpcSegmentCoreApproach, RtiConvergesOnAReach) {
  const Reach6 f = MakeReach6();
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in;
  ReachReference(mpc, f.arm.q_nominal, f.q_seed, in);
  in.p_b = f.target.p;
  in.a_d = f.target.z;
  in.w_p = SkewWeight(2e4, 1e4, 3e4);
  in.w_delta_scale = 0.0;  // the pull toward a moving reference is not part of the objective
  in.cold_start = true;

  pinocchio::Data data(*f.arm.model);
  const int kc = mpc.CatchNode();
  const CatchPose seed = CatchPoseAt(f.arm, data, f.q_seed, Eigen::VectorXd::Zero(6));
  const double seed_pos = (seed.p - in.p_b).norm();
  const double seed_axis = rtc::math::se3::AxisAlignError(seed.z, in.a_d).error.norm();
  ASSERT_GT(seed_pos, 0.02) << "the reference already hits the target";
  ASSERT_GT(seed_axis, 0.05);

  double j_first = 0.0, j_eight = 0.0;
  MpcSegmentCoreResult eighth;
  for (int cycle = 1; cycle <= 50; ++cycle) {
    ASSERT_TRUE(mpc.Solve(in, res))
        << "cycle " << cycle << ": " << MpcSegmentCoreReasonName(res.reason);
    if (cycle == 1) {
      j_first = NonlinearObjective(f, mpc, in, res, data);
    }
    if (cycle == 8) {
      j_eight = NonlinearObjective(f, mpc, in, res, data);
      eighth = res;
    }
    UseAsReference(res, in);
    in.cold_start = false;
  }
  const double j_final = NonlinearObjective(f, mpc, in, res, data);
  RecordProperty("seed_pos_err_um", static_cast<int>(std::lround(1e6 * seed_pos)));
  RecordProperty("final_pos_err_um", static_cast<int>(std::lround(1e6 * res.catch_pos_err.norm())));
  RecordProperty("final_axis_err_urad", static_cast<int>(std::lround(1e6 * res.catch_axis_err)));
  std::printf(
      "[reach] seed %.1f mm / %.1f mrad -> converged %.3f mm / %.3f mrad; J %.4g -> %.4g "
      "-> %.4g\n",
      1e3 * seed_pos, 1e3 * seed_axis, 1e3 * res.catch_pos_err.norm(), 1e3 * res.catch_axis_err,
      j_first, j_eight, j_final);

  // Eight cycles are at the converged solution — in the objective and at the
  // catch node. (Not in every joint angle: the roll about the approach axis is
  // free and w_Δ is off here, so the iterates still drift along directions the
  // objective does not see.)
  EXPECT_NEAR(j_eight, j_final, 1e-3 * j_final);
  EXPECT_LE((eighth.catch_pos_err - res.catch_pos_err).norm(), 1e-4);
  EXPECT_NEAR(eighth.catch_axis_err, res.catch_axis_err, 1e-3);
  // …which is far closer to the catch pose than the reference was. How close
  // is the weights' trade against jerk, not a property of the iteration.
  EXPECT_LT(res.catch_pos_err.norm(), 0.1 * seed_pos);
  EXPECT_LT(res.catch_axis_err, 0.5 * seed_axis);
  // The objective goes down (the tolerance is the solver's, not a slope).
  EXPECT_LE(j_eight, j_first * (1.0 + 1e-6));
  EXPECT_LE(j_final, j_eight * (1.0 + 1e-6));
  // The reported catch residuals are the FK ones.
  const CatchPose at = CatchPoseAt(f.arm, data, res.q.col(kc), res.qd.col(kc));
  EXPECT_LE((res.catch_pos_err - (at.p - in.p_b)).norm(), 1e-12);
  EXPECT_NEAR(res.catch_axis_err, rtc::math::se3::AxisAlignError(at.z, in.a_d).error.norm(), 1e-9);
  // Limits at every node, rest at the end.
  for (int k = 1; k <= mpc.NumNodes(); ++k) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_GE(res.q(j, k), f.limits.q_min[j] + f.params.m_q - 1e-6);
      EXPECT_LE(res.q(j, k), f.limits.q_max[j] - f.params.m_q + 1e-6);
      EXPECT_LE(std::abs(res.qd(j, k)), f.params.eta_v * f.limits.qd_max[j] + 1e-6);
    }
  }
  EXPECT_LE(res.qd.col(mpc.NumNodes()).cwiseAbs().maxCoeff(), 1e-5);
  EXPECT_LE(res.qdd.col(mpc.NumNodes()).cwiseAbs().maxCoeff(), 1e-5);
  EXPECT_LE(res.slack_max, 1e-5);
}

// The velocity target is γ_ref·v̂_b (MD-53): the solution's γ follows it.
TEST(MpcSegmentCoreApproach, SolutionGammaFollowsGammaRef) {
  Reach6 f = MakeReach6();
  f.params.w_v_par = 2000.0;
  f.params.w_v_perp = 2000.0;
  pinocchio::Data data(*f.arm.model);
  double gamma[2] = {0.0, 0.0};
  int i = 0;
  for (const double gamma_ref : {1.0, 0.5}) {
    MpcSegmentCore mpc;
    ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult res;
    mpc.ResizeResult(res);
    MpcSegmentCoreInput in;
    ReachReference(mpc, f.arm.q_nominal, f.q_target, in);
    in.p_b = f.target.p;
    in.a_d = f.target.z;
    in.v_b = 0.5 * Eigen::Vector3d(0.6, 0.3, -0.74).normalized();
    in.gamma_ref = gamma_ref;
    in.w_p = 1e4 * Eigen::Matrix3d::Identity();
    in.w_delta_scale = 0.0;
    in.cold_start = true;
    for (int cycle = 0; cycle < 15; ++cycle) {
      ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
      UseAsReference(res, in);
      in.cold_start = false;
    }
    const int kc = mpc.CatchNode();
    const CatchPose at = CatchPoseAt(f.arm, data, res.q.col(kc), res.qd.col(kc));
    // The reported γ and relative velocity are the FK ones.
    EXPECT_NEAR(res.catch_gamma, in.v_b.dot(at.v) / in.v_b.squaredNorm(), 1e-9);
    EXPECT_LE((res.catch_v_rel - (in.v_b - at.v)).norm(), 1e-9);
    EXPECT_NEAR(res.catch_gamma, gamma_ref, 0.1) << "gamma_ref " << gamma_ref;
    gamma[i++] = res.catch_gamma;
    std::printf("[gamma] ref %.2f -> %.3f, pos err %.2f mm\n", gamma_ref, res.catch_gamma,
                1e3 * res.catch_pos_err.norm());
  }
  EXPECT_GT(gamma[0] - gamma[1], 0.3) << "γ did not follow γ_ref";
}

// s_v is the worst axis' excess over v_rel_allow, as a fraction of it, and zero
// when the hand can match the ball.
TEST(MpcSegmentCoreApproach, VelocitySlackIsTheExcessOverTheAllowance) {
  Reach6 f = MakeReach6();
  f.params.w_v_par = 50.0;
  f.params.w_v_perp = 50.0;
  f.params.rho_v = 5.0;
  f.params.v_rel_allow = 0.2;
  const Eigen::Vector3d dir = Eigen::Vector3d(0.6, 0.3, -0.74).normalized();
  for (const double speed : {0.1, 6.0}) {
    MpcSegmentCore mpc;
    ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult res;
    mpc.ResizeResult(res);
    MpcSegmentCoreInput in;
    ReachReference(mpc, f.arm.q_nominal, f.q_target, in);
    in.p_b = f.target.p;
    in.a_d = f.target.z;
    in.v_b = speed * dir;
    in.w_p = 1e4 * Eigen::Matrix3d::Identity();
    in.w_delta_scale = 0.0;
    in.cold_start = true;
    for (int cycle = 0; cycle < 25; ++cycle) {
      ASSERT_TRUE(mpc.Solve(in, res))
          << "speed " << speed << ": " << MpcSegmentCoreReasonName(res.reason);
      UseAsReference(res, in);
      in.cold_start = false;
    }
    // At convergence the reference IS the solution, so the linear model the
    // slack rows use and the FK value reported agree.
    const double excess =
        std::max(0.0, res.catch_v_rel.cwiseAbs().maxCoeff() / f.params.v_rel_allow - 1.0);
    std::printf("[slack_v] ball %.1f m/s: s_v %.4f, excess %.4f, |v_rel|inf %.3f m/s\n", speed,
                res.slack_v, excess, res.catch_v_rel.cwiseAbs().maxCoeff());
    if (speed < 1.0) {
      EXPECT_LE(res.slack_v, 1e-4) << "a matchable ball needs no slack";
      EXPECT_LE(excess, 1e-4);
    } else {
      ASSERT_GT(excess, 1.0) << "the fixture's ball is not out of reach";
      EXPECT_NEAR(res.slack_v, excess, 1e-2 * excess);
    }
  }
}

// w_delta_scale multiplies w_Δ; cold_start discards the warm start.
TEST(MpcSegmentCoreApproach, DeltaScaleAndColdStart) {
  const ReachFixture f = MakeReach7();
  MpcSegmentCoreParams no_delta = f.params;
  no_delta.w_delta = 0.0;
  MpcSegmentCore scaled, plain, full;
  ASSERT_EQ(scaled.Init(*f.arm.model, f.arm.frame, f.params, f.limits),
            MpcSegmentCoreReason::kNone);
  ASSERT_EQ(plain.Init(*f.arm.model, f.arm.frame, no_delta, f.limits), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(full.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult rs, rp, rf;
  scaled.ResizeResult(rs);
  plain.ResizeResult(rp);
  full.ResizeResult(rf);
  MpcSegmentCoreInput in = f.input;
  // Aimed OFF the target, so the pull toward the reference and the catch terms disagree.
  Eigen::VectorXd aim = f.q_target;
  aim[1] += 0.06;
  aim[3] -= 0.06;
  ReachReference(scaled, f.arm.q_nominal, aim, in);

  // Scale 0 is the w_Δ = 0 problem …
  in.w_delta_scale = 0.0;
  ASSERT_TRUE(scaled.Solve(in, rs)) << MpcSegmentCoreReasonName(rs.reason);
  ASSERT_TRUE(plain.Solve(in, rp)) << MpcSegmentCoreReasonName(rp.reason);
  EXPECT_TRUE(MatricesClose(scaled.MainQp().H, plain.MainQp().H));
  EXPECT_TRUE(MatricesClose(scaled.MainQp().g, plain.MainQp().g));
  EXPECT_LE((rs.q - rp.q).cwiseAbs().maxCoeff(), 1e-6);
  // … and scale 1 is a different one, that stays nearer the reference.
  in.w_delta_scale = 1.0;
  ASSERT_TRUE(full.Solve(in, rf)) << MpcSegmentCoreReasonName(rf.reason);
  ASSERT_GT((rf.q - rs.q).cwiseAbs().maxCoeff(), 1e-4) << "w_Δ did not change the answer";
  EXPECT_LT((rf.q - in.q_ref).norm(), (rs.q - in.q_ref).norm());
  // Back at scale 1 after a scaled solve, the Hessian is the scale-1 one again.
  in.w_delta_scale = 0.3;
  ASSERT_TRUE(scaled.Solve(in, rs));
  in.w_delta_scale = 1.0;
  ASSERT_TRUE(scaled.Solve(in, rs));
  EXPECT_TRUE(MatricesClose(scaled.MainQp().H, full.MainQp().H));

  // cold_start: after an unrelated problem, the solve is the one a fresh core gives.
  MpcSegmentCore fresh;
  ASSERT_EQ(fresh.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r_fresh, r_cold, r_warm;
  fresh.ResizeResult(r_fresh);
  full.ResizeResult(r_cold);
  full.ResizeResult(r_warm);
  MpcSegmentCoreInput other = f.input;
  Eigen::VectorXd elsewhere = f.arm.q_nominal;
  elsewhere[0] -= 0.25;
  elsewhere[5] += 0.3;
  ReachReference(full, f.arm.q_nominal, elsewhere, other);
  other.p_b += Eigen::Vector3d(-0.2, 0.15, 0.1);
  in.cold_start = true;
  ASSERT_TRUE(fresh.Solve(in, r_fresh));
  ASSERT_TRUE(full.Solve(other, r_warm));  // leaves the other problem's iterates
  ASSERT_TRUE(full.Solve(in, r_cold));
  EXPECT_EQ(r_cold.iterations, r_fresh.iterations);
  EXPECT_EQ((r_cold.q - r_fresh.q).cwiseAbs().maxCoeff(), 0.0) << "cold_start is not a fresh start";
  // The control: without it the solver starts from the other problem.
  ASSERT_TRUE(full.Solve(other, r_warm));
  in.cold_start = false;
  ASSERT_TRUE(full.Solve(in, r_warm));
  EXPECT_TRUE(r_warm.iterations != r_fresh.iterations ||
              (r_warm.q - r_fresh.q).cwiseAbs().maxCoeff() > 0.0)
      << "a warm start that is identical to a cold one: the fixture cannot tell them apart";
}

// ── 6. Allocation ────────────────────────────────────────────────────────────

struct AllocCounts {
  std::size_t op_new{0};
  std::size_t c_malloc{0};
};

AllocCounts GatedSolve(MpcSegmentCore& mpc, const MpcSegmentCoreInput& in,
                       MpcSegmentCoreResult& res, bool& ok) {
  AllocCounts c;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    ok = mpc.Solve(in, res);
    c.op_new = new_gate.count();
    c.c_malloc = malloc_gate.count();
  }
  return c;
}

TEST(MpcSegmentCoreApproach, TheAllocationGatesAreArmed) {
  {
    rtc::testing::ScopedAllocGate gate;
    std::vector<int> v(16);
    v[3] = 1;
    EXPECT_GT(gate.count(), 0U);
  }
  // Inside a shared library: pinocchio's explicitly instantiated Data constructor.
  {
    const ArmModel arm = RealArm7();
    rtc::testing::ScopedMallocGate gate;
    pinocchio::Data data(*arm.model);
    EXPECT_GT(gate.count(), 0U);
  }
}

// MD-22: the core's own path allocates nothing; ProxQP's allocations are
// recorded. The trust-region rejection runs the catch linearisation
// (pinocchio's kinematics derivatives, the frame Jacobian, rtc_math) and the
// WHOLE condensing — catch terms and the slack rows included — and stops only
// at the bounds, so its C-malloc count is the core's.
TEST(MpcSegmentCoreApproach, CatchPathAllocatesNothingOutsideTheQpSolver) {
  ReachFixture f = MakeReach7();
  f.params.w_perp = 10.0;
  MpcSegmentCore mpc;
  ASSERT_EQ(mpc.Init(*f.arm.model, f.arm.frame, f.params, f.limits), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult res;
  mpc.ResizeResult(res);
  MpcSegmentCoreInput in = f.input;
  in.p_c = in.p_b;
  in.d_hat = Eigen::Vector3d(0.3, -0.4, 0.2).normalized();
  ReachReference(mpc, f.arm.q_nominal, f.q_target, in);
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);
  UseAsReference(res, in);
  in.cold_start = false;
  in.w_delta_scale = 0.5;
  ASSERT_TRUE(mpc.Solve(in, res)) << MpcSegmentCoreReasonName(res.reason);  // warm-up
  UseAsReference(res, in);
  MpcSegmentCoreInput outside =
      in;  // the reference leaves the box by more than δ, AFTER the catch node
  for (int k = mpc.CatchNode() + 1; k <= mpc.NumNodes(); ++k) {
    outside.q_ref(2, k) = f.limits.q_max[2] + 3.0 * f.params.delta_tr;
  }

  bool ok = false;
  const AllocCounts full = GatedSolve(mpc, in, res, ok);
  EXPECT_TRUE(ok) << MpcSegmentCoreReasonName(res.reason);
  EXPECT_EQ(full.op_new, 0U) << "operator new inside Solve";
  RecordProperty("warm_solve_qp_solver_mallocs", static_cast<int>(full.c_malloc));
  std::printf("[alloc] warm Solve with catch terms: %zu C mallocs (ProxQP, known limitation)\n",
              full.c_malloc);

  const AllocCounts core = GatedSolve(mpc, outside, res, ok);
  EXPECT_FALSE(ok);
  ASSERT_EQ(res.reason, MpcSegmentCoreReason::kTrustRegionConflict);
  EXPECT_GT(res.linearize_us, 0.0) << "the rejection must come after linearisation";
  EXPECT_GT(res.condense_us, 0.0) << "the rejection must come after condensing";
  EXPECT_EQ(core.op_new, 0U);
  EXPECT_EQ(core.c_malloc, 0U) << "C-level allocation in the catch linearisation / condensing";

  // The weight helper is on the planner's path too.
  Eigen::Matrix3d w;
  bool built = false;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    built = rtc::catching::CatchPositionWeight(SkewWeight(1e-4, 4e-4, 9e-4), 1.0, 0.01, 1e4, w);
    EXPECT_EQ(new_gate.count() + malloc_gate.count(), 0U) << "CatchPositionWeight";
  }
  EXPECT_TRUE(built);
}

// ── 7. Fail-closed ───────────────────────────────────────────────────────────

struct Snapshot {
  Eigen::MatrixXd q, qd, qdd, u, slack, tau_ratio;
  Eigen::Vector3d pos, v_rel;
  double axis, gamma, slack_v;
  bool evaluated;

  explicit Snapshot(const MpcSegmentCoreResult& r)
      : q(r.q),
        qd(r.qd),
        qdd(r.qdd),
        u(r.u),
        slack(r.slack),
        tau_ratio(r.tau_ratio),
        pos(r.catch_pos_err),
        v_rel(r.catch_v_rel),
        axis(r.catch_axis_err),
        gamma(r.catch_gamma),
        slack_v(r.slack_v),
        evaluated(r.catch_evaluated) {}

  [[nodiscard]] bool Same(const MpcSegmentCoreResult& r) const {
    return q == r.q && qd == r.qd && qdd == r.qdd && u == r.u && slack == r.slack &&
           tau_ratio == r.tau_ratio && pos == r.catch_pos_err && v_rel == r.catch_v_rel &&
           axis == r.catch_axis_err && gamma == r.catch_gamma && slack_v == r.slack_v &&
           evaluated == r.catch_evaluated;
  }
};

class MpcSegmentCoreApproachFailClosed : public ::testing::Test {
 protected:
  void SetUp() override {
    f_ = MakeReach7();
    ASSERT_EQ(mpc_.Init(*f_.arm.model, f_.arm.frame, f_.params, f_.limits),
              MpcSegmentCoreReason::kNone);
    mpc_.ResizeResult(res_);
    good_ = f_.input;
    ReachReference(mpc_, f_.arm.q_nominal, f_.q_target, good_);
    ASSERT_TRUE(mpc_.Solve(good_, res_)) << MpcSegmentCoreReasonName(res_.reason);
    ASSERT_TRUE(res_.catch_evaluated);
  }

  void ExpectRejected(const MpcSegmentCoreInput& in, MpcSegmentCoreReason why, const char* what) {
    const Snapshot before(res_);
    EXPECT_FALSE(mpc_.Solve(in, res_)) << what;
    EXPECT_EQ(res_.reason, why) << what << ": got " << MpcSegmentCoreReasonName(res_.reason);
    EXPECT_FALSE(res_.valid) << what;
    EXPECT_TRUE(before.Same(res_)) << what << ": outputs changed on failure";
  }

  ReachFixture f_;
  MpcSegmentCore mpc_;
  MpcSegmentCoreResult res_;
  MpcSegmentCoreInput good_;
};

TEST_F(MpcSegmentCoreApproachFailClosed, RejectsNonFiniteCatchInputs) {
  MpcSegmentCoreInput in = good_;
  in.p_b[1] = kNan;
  ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "p_b NaN");
  in = good_;
  in.w_p(0, 2) = kInf;
  ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "w_p inf");
  in = good_;
  in.a_d[0] = kNan;
  ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "a_d NaN");
  in = good_;
  in.v_b[2] = kNan;
  ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "v_b NaN");
  in = good_;
  in.gamma_ref = kNan;
  ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "gamma_ref NaN");
  in = good_;
  in.w_delta_scale = kNan;
  ExpectRejected(in, MpcSegmentCoreReason::kNonFinite, "w_delta_scale NaN");
}

TEST_F(MpcSegmentCoreApproachFailClosed, RejectsOutOfRangeCatchInputs) {
  MpcSegmentCoreInput in = good_;
  in.a_d *= 1.5;
  ExpectRejected(in, MpcSegmentCoreReason::kDirectionNotUnit, "a_d not unit");
  for (const double g : {0.0, -0.2, 1.01}) {
    in = good_;
    in.gamma_ref = g;
    ExpectRejected(in, MpcSegmentCoreReason::kInputOutOfRange, "gamma_ref");
  }
  for (const double s : {-0.1, 1.5}) {
    in = good_;
    in.w_delta_scale = s;
    ExpectRejected(in, MpcSegmentCoreReason::kInputOutOfRange, "w_delta_scale");
  }
  in = good_;
  in.w_p(0, 1) += 5.0;  // not symmetric
  ExpectRejected(in, MpcSegmentCoreReason::kInputOutOfRange, "w_p asymmetric");
  in = good_;
  in.w_p = SkewWeight(2000.0, -50.0, 1000.0);  // symmetric, one negative eigenvalue
  ExpectRejected(in, MpcSegmentCoreReason::kInputOutOfRange, "w_p indefinite");
  // A rank-deficient PSD weight is a legitimate one (no pull along one axis).
  in = good_;
  in.w_p = SkewWeight(2000.0, 0.0, 1000.0);
  EXPECT_TRUE(mpc_.Solve(in, res_)) << MpcSegmentCoreReasonName(res_.reason);
}

TEST_F(MpcSegmentCoreApproachFailClosed, RejectsAMissingReferenceAndAFarAxis) {
  MpcSegmentCoreInput in = good_;
  in.reference_valid = false;
  ExpectRejected(in, MpcSegmentCoreReason::kReferenceRequired, "no reference");
  // The reference's catch-node axis is more than axis_theta_max (π/2) from a_d.
  pinocchio::Data data(*f_.arm.model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(f_.arm.model->nv);
  const Eigen::Vector3d z = CatchPoseAt(f_.arm, data, good_.q_ref.col(mpc_.CatchNode()), zero).z;
  in = good_;
  in.a_d = TiltedAxis(z, 2.0, Eigen::Vector3d(0.3, -0.5, 0.8));
  ExpectRejected(in, MpcSegmentCoreReason::kCatchAxisOutOfRange, "axis 2 rad off");
  in = good_;
  in.a_d = -z;
  ExpectRejected(in, MpcSegmentCoreReason::kCatchAxisOutOfRange, "axis antiparallel");
}

// A core WITHOUT catch terms never reads the catch inputs: the existing
// consumer (MpcSegmentPlanner) leaves them default-constructed, and garbage in them
// must not matter either.
TEST(MpcSegmentCoreApproach, CatchInputsAreIgnoredWhenCatchTermsAreOff) {
  const ArmModel arm = Synthetic6R();
  for (const MpcSegmentCoreParams& p : {MpcSegmentCoreParams{}, GridParams(kSmallGrid)}) {
    MpcSegmentCore mpc;
    ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)),
              MpcSegmentCoreReason::kNone);
    MpcSegmentCoreResult clean, dirty;
    mpc.ResizeResult(clean);
    mpc.ResizeResult(dirty);
    MpcSegmentCoreInput in = RestInput(arm.q_nominal);
    in.qd0 << 0.3, -0.2, 0.25, 0.1, -0.2, 0.3;
    ASSERT_TRUE(mpc.Solve(in, clean)) << MpcSegmentCoreReasonName(clean.reason);
    EXPECT_FALSE(clean.catch_evaluated);
    MpcSegmentCoreInput junk = in;
    junk.p_b.setConstant(kNan);
    junk.w_p.setConstant(kNan);
    junk.a_d.setZero();
    junk.v_b.setConstant(kInf);
    junk.gamma_ref = -3.0;
    ASSERT_TRUE(mpc.Solve(junk, dirty)) << MpcSegmentCoreReasonName(dirty.reason);
    EXPECT_EQ((clean.q - dirty.q).cwiseAbs().maxCoeff(), 0.0);
    // No pre-catch reference needed either: the pre-solve path still serves.
    EXPECT_TRUE(dirty.presolved);
  }
}

// ── 8. The position weight from a covariance ─────────────────────────────────

TEST(CatchPositionWeightTest, MatchesTheClosedFormAndClampsBothEnds) {
  Eigen::Matrix3d w;
  // Diagonal: w_i = κ / (σ_i² + σ_floor²).
  const Eigen::Matrix3d diag = Eigen::Vector3d(1e-4, 4e-4, 2.5e-3).asDiagonal();
  ASSERT_TRUE(rtc::catching::CatchPositionWeight(diag, 2.0, 0.01, 1e9, w));
  EXPECT_NEAR(w(0, 0), 2.0 / (1e-4 + 1e-4), 1e-6);
  EXPECT_NEAR(w(1, 1), 2.0 / (4e-4 + 1e-4), 1e-6);
  EXPECT_NEAR(w(2, 2), 2.0 / (2.5e-3 + 1e-4), 1e-6);
  EXPECT_LE(std::abs(w(0, 1)) + std::abs(w(0, 2)) + std::abs(w(1, 2)), 1e-9);

  // Rotated: κ (Σ + σ² I)⁻¹, exactly symmetric, and accepted by the core's own check.
  const Eigen::Matrix3d sigma = SkewWeight(1e-4, 4e-4, 2.5e-3);
  ASSERT_TRUE(rtc::catching::CatchPositionWeight(sigma, 2.0, 0.01, 1e9, w));
  const Eigen::Matrix3d expected = 2.0 * (sigma + 1e-4 * Eigen::Matrix3d::Identity()).inverse();
  EXPECT_LE((w - expected).cwiseAbs().maxCoeff(), 1e-9 * expected.cwiseAbs().maxCoeff());
  EXPECT_EQ((w - w.transpose()).cwiseAbs().maxCoeff(), 0.0);

  // The cap: no eigenvalue above w_max.
  ASSERT_TRUE(rtc::catching::CatchPositionWeight(sigma, 2.0, 0.01, 3000.0, w));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(w);
  EXPECT_LE(es.eigenvalues().maxCoeff(), 3000.0 * (1.0 + 1e-12));
  EXPECT_NEAR(es.eigenvalues().minCoeff(), 2.0 / (2.5e-3 + 1e-4), 1e-6);

  // A zero covariance: the floor alone bounds the weight.
  ASSERT_TRUE(rtc::catching::CatchPositionWeight(Eigen::Matrix3d::Zero(), 1.0, 0.01, 1e9, w));
  EXPECT_LE((w - 1e4 * Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-6);

  // A covariance that is NOT positive semidefinite (an estimator's rounding, or
  // worse): the negative eigenvalue counts as zero. Unclamped, λ = −σ_floor²
  // would be a division by zero and λ below it a NEGATIVE weight.
  for (const double lambda : {-1e-4, -5e-4}) {
    const Eigen::Matrix3d bad = SkewWeight(1e-4, lambda, 2.5e-3);
    ASSERT_TRUE(rtc::catching::CatchPositionWeight(bad, 1.0, 0.01, 1e9, w));
    ASSERT_TRUE(w.allFinite());
    es.compute(w);
    EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
    EXPECT_NEAR(es.eigenvalues().maxCoeff(), 1.0 / 1e-4, 1e-4) << "λ = " << lambda;
  }

  // An asymmetric input is symmetrised, deliberately (SelfAdjointEigenSolver
  // would silently read one triangle).
  Eigen::Matrix3d skewed = sigma;
  skewed(0, 1) += 2e-5;
  skewed(1, 0) -= 2e-5;
  Eigen::Matrix3d w_skewed;
  ASSERT_TRUE(rtc::catching::CatchPositionWeight(skewed, 2.0, 0.01, 1e9, w_skewed));
  ASSERT_TRUE(rtc::catching::CatchPositionWeight(sigma, 2.0, 0.01, 1e9, w));
  EXPECT_LE((w - w_skewed).cwiseAbs().maxCoeff(), 1e-9 * w.cwiseAbs().maxCoeff());
}

TEST(CatchPositionWeightTest, RejectsNonFiniteAndNonPositiveArguments) {
  const Eigen::Matrix3d sigma = SkewWeight(1e-4, 4e-4, 2.5e-3);
  const Eigen::Matrix3d sentinel = Eigen::Matrix3d::Constant(7.0);
  Eigen::Matrix3d w = sentinel;
  Eigen::Matrix3d nan_sigma = sigma;
  nan_sigma(1, 1) = kNan;
  EXPECT_FALSE(rtc::catching::CatchPositionWeight(nan_sigma, 1.0, 0.01, 1e4, w));
  for (const double bad : {0.0, -1.0, kNan, kInf}) {
    EXPECT_FALSE(rtc::catching::CatchPositionWeight(sigma, bad, 0.01, 1e4, w)) << "kappa " << bad;
    EXPECT_FALSE(rtc::catching::CatchPositionWeight(sigma, 1.0, bad, 1e4, w)) << "floor " << bad;
    EXPECT_FALSE(rtc::catching::CatchPositionWeight(sigma, 1.0, 0.01, bad, w)) << "w_max " << bad;
  }
  EXPECT_EQ(w, sentinel) << "the output is written only on success";
}

// ── 9. Timing: the measurement the grid decision reads (MD-51) ──────────
// Informational: nothing here asserts a time. The grid is chosen from this
// table by this rule (warm p99 ≤ 10 ms, cold p99 ≤ 12 ms, no failed
// solve; Release, 7 dof, 200 samples) and confirmed by the user, because the
// choice has costs outside this core (payload, sampler).
//
// One sample is one throw: the arm at rest in its nominal posture, a random
// target posture the velocity box can reach in the pre-catch time, a ball
// arriving along the approach axis. Every catch term and the torque rows are
// on. Four solves per sample:
//   cold        the first solve of a plan — the caller's reach reference, no
//               pull toward it (w_delta_scale 0), solver started from zero;
//   warm_same   the next message at the SAME grid point: reference = the last
//               solution, the ball target moved 1 cm, the solver warm;
//   adv_cold    the grid advanced by one pre-catch node. The grid is anchored
//               at t_c, so this is a DIFFERENT core (one node fewer): the
//               reference is the old nodes 1..N, x_0 the old node 1, but the
//               solver's iterates cannot follow — started from zero;
//   adv_stale   the same, started from whatever that core solved last (the
//               previous sample — another throw). About a quarter of these
//               warm runs fail and the core retries them from zero
//               (cold_retried), so the series shows what leaving cold_start
//               off costs.
// No solve may fail (MD-51) — that is asserted; the times are not.
// The full table needs RTC_MPC_SEGMENT_CORE_TIMING_FULL=1 (200 samples per grid); the
// default run is a smoke of the same code.

struct TimingGrid {
  const char* name;
  Grid grid;
};

const TimingGrid kTimingGrids[] = {
    {"A1", {12, 0.05, 14, 0.025, 1, {1, 1, 2, 2, 4, 4}}},
    {"B1", {12, 0.05, 7, 0.05, 1, {1, 1, 2, 3}}},
    {"A2", {12, 0.05, 14, 0.025, 2, {1, 1, 2, 2, 4, 4}}},
    {"B2", {12, 0.05, 7, 0.05, 2, {1, 1, 2, 3}}},
    // The grid MD-54 settled on: the pre-catch spacing doubled, so
    // B2's jerk hold (0.1 s) with half its pre-catch nodes.
    {"C1", {6, 0.1, 7, 0.05, 1, {1, 1, 2, 3}}},
};

struct Series {
  std::vector<double> total_us, solve_us, iters;
  int failures{0};
  int retried{0};
  std::string failure_log;

  void Add(const MpcSegmentCoreResult& r) {
    total_us.push_back(r.linearize_us + r.condense_us + r.solve_us);
    solve_us.push_back(r.solve_us);
    iters.push_back(r.iterations);
    retried += r.cold_retried ? 1 : 0;
  }

  void Fail(const MpcSegmentCoreResult& r) {
    ++failures;
    if (failures <= 4) {
      failure_log += std::string(" ") + MpcSegmentCoreReasonName(r.reason) + "/status" +
                     std::to_string(r.qp_status) + "/it" + std::to_string(r.iterations);
    }
  }
};

void ReportSeries(const std::string& key, const Series& s) {
  using rtc::testing::mpc_segment_core::Percentile;
  using rtc::testing::mpc_segment_core::RecordMicros;
  RecordMicros(key + ".total_us.p50", Percentile(s.total_us, 0.50));
  RecordMicros(key + ".total_us.p99", Percentile(s.total_us, 0.99));
  RecordMicros(key + ".total_us.max", Percentile(s.total_us, 1.0));
  ::testing::Test::RecordProperty(key + ".iterations.p50",
                                  static_cast<int>(Percentile(s.iters, 0.5)));
  ::testing::Test::RecordProperty(key + ".iterations.p99",
                                  static_cast<int>(Percentile(s.iters, 0.99)));
  ::testing::Test::RecordProperty(key + ".failures", s.failures);
  ::testing::Test::RecordProperty(key + ".cold_retried", s.retried);
  EXPECT_EQ(s.failures, 0) << key << ":" << s.failure_log;
  ::testing::Test::RecordProperty(key + ".samples", static_cast<int>(s.total_us.size()));
  RecordMicros(key + ".solve_us.p50", Percentile(s.solve_us, 0.50));
  std::printf(
      "[timing] %-22s n=%3zu fail=%2d retried=%3d | total p50 %7.0f p99 %7.0f max %7.0f us (QP "
      "p50 %7.0f) | iter p50 %3.0f p99 %3.0f%s%s\n",
      key.c_str(), s.total_us.size(), s.failures, s.retried, Percentile(s.total_us, 0.5),
      Percentile(s.total_us, 0.99), Percentile(s.total_us, 1.0), Percentile(s.solve_us, 0.5),
      Percentile(s.iters, 0.5), Percentile(s.iters, 0.99), s.failures > 0 ? " | failures:" : "",
      s.failure_log.c_str());
}

[[nodiscard]] bool OptimisedBuild() {
  const std::string bt = RTC_TEST_BUILD_TYPE;
  return bt == "Release" || bt == "RelWithDebInfo" || bt == "MinSizeRel";
}

[[nodiscard]] int TimingSamples() {
  const char* e = std::getenv("RTC_DECEL_MPC_TIMING_FULL");
  const bool full = e != nullptr && e[0] != '\0' && e[0] != '0';
  if (!OptimisedBuild()) {
    return 4;
  }
  return full ? 200 : 8;
}

// The shipped-like catch weights of the measurement. Values are placeholders
// for E1-F10's tuning; what matters here is that every term is assembled.
void TimingCatchParams(MpcSegmentCoreParams& p, bool with_slack) {
  p.catch_terms = true;
  p.w_axis = 100.0;
  p.w_v_par = 1.0;
  p.w_v_perp = 20.0;
  if (with_slack) {
    p.rho_v = 1.0;
    p.v_rel_allow = 0.5;
  }
}

// The throws of the measurement, from a fixed seed: the first-solve input of
// each (the reach reference to a random reachable posture, the ball there).
struct ThrowSampler {
  std::mt19937 rng{42};
  std::uniform_real_distribution<double> uni{-1.0, 1.0};
  std::uniform_real_distribution<double> frac{0.3, 1.0};

  MpcSegmentCoreInput Next(const ArmModel& arm, const MpcSegmentCoreLimits& lim,
                           const MpcSegmentCoreParams& p, const MpcSegmentCore& mpc,
                           pinocchio::Data& data, const Eigen::Matrix3d& w_p) {
    const int n = arm.model->nv;
    const double t_catch = mpc.NodeTime(mpc.CatchNode());
    // A target the velocity box reaches: the minimum-jerk peak 1.875·d/T stays
    // under 0.9·η_v·q̇_max (a faster reference cannot be followed inside the
    // trust region and is not a plan a planner should hand over).
    Eigen::VectorXd q_t = arm.q_nominal;
    for (int j = 0; j < n; ++j) {
      const double cap = 0.9 * p.eta_v * lim.qd_max[j] * t_catch / 1.875;
      const double d = (uni(rng) < 0.0 ? -1.0 : 1.0) * frac(rng) * cap;
      q_t[j] = std::clamp(arm.q_nominal[j] + d, lim.q_min[j] + 0.15, lim.q_max[j] - 0.15);
    }
    const CatchPose target = CatchPoseAt(arm, data, q_t, Eigen::VectorXd::Zero(n));
    MpcSegmentCoreInput in;
    ReachReference(mpc, arm.q_nominal, q_t, in);
    in.p_b = target.p + 0.01 * Eigen::Vector3d(uni(rng), uni(rng), uni(rng));
    in.a_d = TiltedAxis(target.z, 0.05, Eigen::Vector3d(uni(rng), uni(rng), uni(rng) + 2.0));
    in.v_b = -(4.5 + 1.5 * uni(rng)) * in.a_d;  // a_d = −v̂_b/‖v̂_b‖ (formulation §0.2)
    in.w_p = w_p;
    in.w_delta_scale = 0.0;
    in.cold_start = true;
    return in;
  }
};

// A change to the measured problem: its parameters, its inputs, or both.
struct TimingVariant {
  std::function<void(MpcSegmentCoreParams&)> params;
  std::function<void(MpcSegmentCoreInput&)> input;
};

void RunGridTiming(const ArmModel& arm, const MpcSegmentCoreLimits& lim, const TimingGrid& tg,
                   const std::string& key, int n_samples, bool with_slack,
                   const TimingVariant& variant = {}) {
  const int n = arm.model->nv;
  MpcSegmentCoreParams p = GridParams(tg.grid);
  TimingCatchParams(p, with_slack);
  Grid next_grid = tg.grid;
  next_grid.n_pre -= 1;
  MpcSegmentCoreParams p_next = GridParams(next_grid);
  TimingCatchParams(p_next, with_slack);
  if (variant.params) {
    variant.params(p);
    variant.params(p_next);
  }
  MpcSegmentCore mpc, adv_cold, adv_stale;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone) << key;
  ASSERT_EQ(adv_cold.Init(*arm.model, arm.frame, p_next, lim), MpcSegmentCoreReason::kNone) << key;
  ASSERT_EQ(adv_stale.Init(*arm.model, arm.frame, p_next, lim), MpcSegmentCoreReason::kNone) << key;
  MpcSegmentCoreResult res, res_same, res_adv;
  mpc.ResizeResult(res);
  mpc.ResizeResult(res_same);
  adv_cold.ResizeResult(res_adv);
  const int N = mpc.NumNodes();
  ::testing::Test::RecordProperty(key + ".n_vars", static_cast<int>(mpc.MainQp().H.rows()));
  ::testing::Test::RecordProperty(key + ".n_ineq", static_cast<int>(mpc.MainQp().C.rows()));

  Eigen::Matrix3d w_p;
  ASSERT_TRUE(
      rtc::catching::CatchPositionWeight(SkewWeight(1e-4, 4e-4, 2.5e-5), 1.0, 0.01, 1e4, w_p));
  ThrowSampler sampler;
  std::mt19937& rng = sampler.rng;
  std::uniform_real_distribution<double>& uni = sampler.uni;
  pinocchio::Data data(*arm.model);
  Series cold, warm_same, s_adv_cold, s_adv_stale;
  std::vector<double> pos_err_mm, overshoot;
  int slack_active = 0;
  for (int i = 0; i < n_samples; ++i) {
    MpcSegmentCoreInput in = sampler.Next(arm, lim, p, mpc, data, w_p);
    if (variant.input) {
      variant.input(in);
    }
    if (!mpc.Solve(in, res)) {
      cold.Fail(res);
      continue;
    }
    cold.Add(res);
    pos_err_mm.push_back(1e3 * res.catch_pos_err.norm());
    slack_active += res.slack_max > 1e-6 ? 1 : 0;
    // Velocity between nodes (the box is enforced AT the nodes): the worst
    // |q̇|/(η_v q̇_max) on a 10-point sweep of every interval.
    double over = 0.0;
    for (int k = 0; k < N; ++k) {
      const double dt = mpc.NodeTime(k + 1) - mpc.NodeTime(k);
      for (int m = 1; m < 10; ++m) {
        const double tau = dt * m / 10.0;
        const Eigen::VectorXd qd =
            res.qd.col(k) + tau * res.qdd.col(k) + 0.5 * tau * tau * res.u.col(k);
        for (int j = 0; j < n; ++j) {
          over = std::max(over, std::abs(qd[j]) / (p.eta_v * lim.qd_max[j]));
        }
      }
    }
    overshoot.push_back(over);

    // The next message.
    const Eigen::Vector3d moved =
        in.p_b + 0.01 * Eigen::Vector3d(uni(rng), uni(rng), uni(rng)).normalized();
    MpcSegmentCoreInput same = in;
    UseAsReference(res, same);
    same.p_b = moved;
    same.w_delta_scale = 1.0;
    same.cold_start = false;
    if (mpc.Solve(same, res_same)) {
      warm_same.Add(res_same);
    } else {
      warm_same.Fail(res_same);
    }

    MpcSegmentCoreInput adv = in;
    adv.q_ref = res.q.rightCols(N);
    adv.qd_ref = res.qd.rightCols(N);
    adv.qdd_ref = res.qdd.rightCols(N);
    adv.q0 = res.q.col(1);
    adv.qd0 = res.qd.col(1);
    adv.qdd0 = res.qdd.col(1);
    adv.p_b = moved;
    adv.w_delta_scale = 1.0;
    adv.cold_start = true;
    if (adv_cold.Solve(adv, res_adv)) {
      s_adv_cold.Add(res_adv);
    } else {
      s_adv_cold.Fail(res_adv);
    }
    adv.cold_start = false;
    if (adv_stale.Solve(adv, res_adv)) {
      s_adv_stale.Add(res_adv);
    } else {
      s_adv_stale.Fail(res_adv);
    }
  }
  ReportSeries(key + ".cold", cold);
  ReportSeries(key + ".warm_same", warm_same);
  ReportSeries(key + ".adv_cold", s_adv_cold);
  ReportSeries(key + ".adv_stale", s_adv_stale);
  using rtc::testing::mpc_segment_core::Percentile;
  ::testing::Test::RecordProperty(key + ".cold.pos_err_um.p50",
                                  static_cast<int>(std::lround(1e3 * Percentile(pos_err_mm, 0.5))));
  ::testing::Test::RecordProperty(
      key + ".cold.pos_err_um.p99",
      static_cast<int>(std::lround(1e3 * Percentile(pos_err_mm, 0.99))));
  ::testing::Test::RecordProperty(key + ".cold.torque_slack_active", slack_active);
  ::testing::Test::RecordProperty(key + ".cold.between_node_speed_permille",
                                  static_cast<int>(std::lround(1e3 * Percentile(overshoot, 1.0))));
  std::printf(
      "[timing] %-22s vars %ld rows %ld | first-solve pos err p50 %.1f p99 %.1f mm | "
      "torque slack active %d/%zu | max between-node |qd|/box %.3f\n",
      key.c_str(), static_cast<long>(mpc.MainQp().H.rows()),
      static_cast<long>(mpc.MainQp().C.rows()), Percentile(pos_err_mm, 0.5),
      Percentile(pos_err_mm, 0.99), slack_active, cold.total_us.size(), Percentile(overshoot, 1.0));
}

// A warm solve from ANOTHER problem's iterates: ProxQP calls a feasible QP
// infeasible on about a quarter of these (formulation §1.6; measured in E1-F07,
// #660). The core retries from zero, so a caller that left cold_start off on a
// new throw still gets the plan — the one a cold solve gives. A QP that really
// is infeasible fails both runs.
TEST(MpcSegmentCoreApproach, AStaleWarmStartIsRetriedCold) {
  const ArmModel arm = RealArm7();
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  MpcSegmentCoreParams p = GridParams(kTimingGrids[4].grid);
  TimingCatchParams(p, false);
  MpcSegmentCore stale, fresh;
  ASSERT_EQ(stale.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  ASSERT_EQ(fresh.Init(*arm.model, arm.frame, p, lim), MpcSegmentCoreReason::kNone);
  MpcSegmentCoreResult r_stale, r_fresh;
  stale.ResizeResult(r_stale);
  fresh.ResizeResult(r_fresh);
  Eigen::Matrix3d w_p;
  ASSERT_TRUE(
      rtc::catching::CatchPositionWeight(SkewWeight(1e-4, 4e-4, 2.5e-5), 1.0, 0.01, 1e4, w_p));
  ThrowSampler sampler;
  pinocchio::Data data(*arm.model);
  int retried = 0;
  double worst = 0.0;
  for (int i = 0; i < 24; ++i) {
    MpcSegmentCoreInput in = sampler.Next(arm, lim, p, stale, data, w_p);
    ASSERT_TRUE(fresh.Solve(in, r_fresh)) << i << " " << MpcSegmentCoreReasonName(r_fresh.reason);
    EXPECT_FALSE(r_fresh.cold_retried) << i << ": a cold solve has nothing to retry";
    in.cold_start = false;  // the solver still holds the previous throw's iterates
    ASSERT_TRUE(stale.Solve(in, r_stale)) << i << " " << MpcSegmentCoreReasonName(r_stale.reason);
    if (i == 0) {
      EXPECT_FALSE(r_stale.cold_retried) << "no iterates to be misled by yet";
    }
    retried += r_stale.cold_retried ? 1 : 0;
    worst = std::max(worst, (r_stale.q - r_fresh.q).cwiseAbs().maxCoeff());
  }
  ASSERT_GT(retried, 0) << "no stale warm start failed: the fixture does not reach the retry";
  RecordProperty("cold_retried_of_24", retried);
  EXPECT_LE(worst, 1e-4) << "the same QP has one solution, however the solver was started [rad]";

  // Really infeasible: at the upper edge of the box, moving outward at the
  // velocity limit — node 1 cannot stay inside whatever the jerk. The solver
  // is warm (the loop's last solve), so both runs happen and both fail.
  const MpcSegmentCoreResult before = r_stale;
  MpcSegmentCoreInput edge = RestInput(arm.q_nominal);
  edge.q0[3] = lim.q_max[3] - p.m_q - 1e-9;
  edge.qd0[3] = p.eta_v * lim.qd_max[3];
  edge.q_ref = edge.q0.replicate(1, stale.NumNodes() + 1);
  edge.qd_ref.setZero(arm.model->nv, stale.NumNodes() + 1);
  edge.qdd_ref.setZero(arm.model->nv, stale.NumNodes() + 1);
  edge.reference_valid = true;
  const CatchPose here = CatchPoseAt(arm, data, edge.q0, Eigen::VectorXd::Zero(arm.model->nv));
  edge.p_b = here.p;
  edge.a_d = here.z;
  edge.v_b = -5.0 * here.z;
  edge.w_p = w_p;
  EXPECT_FALSE(stale.Solve(edge, r_stale));
  EXPECT_EQ(r_stale.reason, MpcSegmentCoreReason::kQpFailed)
      << MpcSegmentCoreReasonName(r_stale.reason);
  EXPECT_TRUE(r_stale.cold_retried);
  EXPECT_EQ(r_stale.q, before.q) << "fail-closed: the trajectory is the last good one";
  // The same input on a cold solver fails once, without a retry.
  EXPECT_FALSE(stale.Solve(edge, r_stale));
  EXPECT_EQ(r_stale.reason, MpcSegmentCoreReason::kQpFailed)
      << MpcSegmentCoreReasonName(r_stale.reason);
  EXPECT_FALSE(r_stale.cold_retried) << "the failed run already reset the solver";
}

TEST(MpcSegmentCoreApproachTiming, GridTable7R) {
  const ArmModel arm = RealArm7();
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  const int n_samples = TimingSamples();
  RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
  for (const TimingGrid& tg : kTimingGrids) {
    RunGridTiming(arm, lim, tg, std::string("real_7dof.") + tg.name, n_samples, false);
  }
}

TEST(MpcSegmentCoreApproachTiming, GridTable6R) {
  const ArmModel arm = RealArm6();
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.1);
  const int n_samples = TimingSamples();
  RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
  for (const TimingGrid& tg : kTimingGrids) {
    RunGridTiming(arm, lim, tg, std::string("real_6dof.") + tg.name, n_samples, false);
  }
}

// The velocity slack is off by default (MD-52); what turning it on costs,
// and whether its penalty row provokes ProxQP's false-infeasible verdict, is
// E1-F10's input.
TEST(MpcSegmentCoreApproachTiming, VelocitySlackOn7R) {
  const ArmModel arm = RealArm7();
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
  RunGridTiming(arm, lim, kTimingGrids[3], "real_7dof.B2.slack_v", TimingSamples(), true);
}

// What the solve time depends on. Each variant changes ONE thing (two where
// named) in the smallest decision grid; none is a recommendation — they locate
// the cost for the decision recorded in E1-F07 (#660). Findings of the
// 2026-10-01 run are there; in short: the time is the QP's (linearisation and
// condensing are ~0.1 ms), it halves without the torque rows and again with
// half the pre-catch nodes, the solver tolerance and a pure variable rescaling
// change nothing (ProxQP's own equilibration already does that), and a
// heavier jerk weight halves the iterations by changing the problem.
TEST(MpcSegmentCoreApproachTiming, CostDrivers7R) {
  const ArmModel arm = RealArm7();
  const MpcSegmentCoreLimits lim = LimitsFromModel(*arm.model, 0.2);
  const int n_samples = TimingSamples();
  RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
  const TimingGrid& grid = kTimingGrids[3];
  const auto run = [&](const char* name, const TimingVariant& v) {
    RunGridTiming(arm, lim, grid, std::string("real_7dof.B2.") + name, n_samples, false, v);
  };
  run("eps_1e-4", {[](MpcSegmentCoreParams& p) {
                     p.solver.eps_abs = 1e-4;
                     p.reference_rest_tol = 1e-3;
                   },
                   {}});
  run("velocity_term_off", {[](MpcSegmentCoreParams& p) {
                              p.w_v_par = 0.0;
                              p.w_v_perp = 0.0;
                            },
                            {}});
  run("gamma_ref_0.2", {{}, [](MpcSegmentCoreInput& in) { in.gamma_ref = 0.2; }});
  run("no_trust_region", {[](MpcSegmentCoreParams& p) { p.delta_tr = kInf; }, {}});
  run("torque_rows_off", {[](MpcSegmentCoreParams& p) { p.rho_tau = 0.0; }, {}});
  // u_scale alone is NOT a preconditioner (mpc_segment_core.hpp): 1e2 weighs jerk
  // a hundred times heavier against every other term.
  run("jerk_weight_x100", {[](MpcSegmentCoreParams& p) { p.u_scale = 1e2; }, {}});
  run("torque_off+jerk_x100", {[](MpcSegmentCoreParams& p) {
                                 p.rho_tau = 0.0;
                                 p.u_scale = 1e2;
                               },
                               {}});
  // The SAME problem in another variable scale: (u/u_scale)²·R is unchanged
  // when R goes with u_scale².
  run("rescaled_1e2", {[](MpcSegmentCoreParams& p) {
                         p.jerk_weight = Eigen::VectorXd::Constant(7, 1e-2);
                         p.u_scale = 1e2;
                       },
                       {}});
  // Fewer nodes: a typical throw (the E0-F02 median entry is 0.44 s before the
  // catch, not the 0.6 s the table sizes for), and the chosen grid without its
  // torque rows.
  const TimingGrid typical{"B2_pre9", {9, 0.05, 7, 0.05, 2, {1, 1, 2, 3}}};
  RunGridTiming(arm, lim, typical, "real_7dof.B2_pre9", n_samples, false);
  RunGridTiming(arm, lim, kTimingGrids[4], "real_7dof.C1.torque_rows_off", n_samples, false,
                {[](MpcSegmentCoreParams& p) { p.rho_tau = 0.0; }, {}});
}

}  // namespace
