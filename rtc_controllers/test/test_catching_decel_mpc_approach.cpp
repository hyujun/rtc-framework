// E1-F07 (#660): the single-arm MPC core from APPROACH to the stop — the
// pre-catch grid and the catch terms added to DecelMpc (decel_mpc.hpp;
// formulation §1.6, plan MD-51 – MD-53). The stop-segment behaviour E1-F01
// pinned stays in test_catching_decel_mpc.cpp; this suite owns what is new,
// plus the regression that the new code leaves the old problem alone.
//
// The allocation gates: like the E1-F01 suite, this binary links THREE sensors
// (CMakeLists note); malloc_gate.hpp is the one that sees pinocchio's and
// ProxQP's own allocations.
#include "rtc_controllers/catching/decel_mpc.hpp"
#include "rtc_controllers/catching/jerk_segment.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/decel_mpc_fixture.hpp"
#include "rtc_controllers/testing/malloc_gate.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace {

using rtc::catching::DecelMpc;
using rtc::catching::DecelMpcInput;
using rtc::catching::DecelMpcLimits;
using rtc::catching::DecelMpcParams;
using rtc::catching::DecelMpcReason;
using rtc::catching::DecelMpcReasonName;
using rtc::catching::DecelMpcResult;
using rtc::testing::decel::ArmModel;
using rtc::testing::decel::LimitsFromModel;
using rtc::testing::decel::RealArm6;
using rtc::testing::decel::RealArm7;
using rtc::testing::decel::RestInput;
using rtc::testing::decel::Synthetic6R;

// ── Golden regression: the E1-F01 problem is untouched ───────────────────────
// The values below were captured from the build of `main` at edc0fa4e, BEFORE
// any E1-F07 change to the core, by running this test with
// RTC_DECEL_MPC_GOLDEN_PRINT=1. They pin what "the new parameters default to
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

void AddSolution(Probe& p, const DecelMpcResult& r) {
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
void ShiftUniform(const DecelMpcResult& r, double dt, double t_shift, DecelMpcInput& in) {
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
TEST(DecelMpcGolden, DefaultCoreOnThePresolvePath) {
  const ArmModel arm = Synthetic6R();
  const DecelMpcParams p;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model)), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.30, -0.25, 0.20, -0.15, 0.35, -0.30;
  in.qdd0 << 1.0, -0.5, 0.8, 0.0, -1.2, 0.4;
  ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
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
TEST(DecelMpcGolden, PerpendicularTermOnAWarmCycle) {
  const ArmModel arm = RealArm7();
  DecelMpcParams p;
  p.w_perp = 10.0;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, LimitsFromModel(*arm.model, 0.2)),
            DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 0.6, -0.4, 0.5, 0.7, -0.3, 0.4, 0.2;
  pinocchio::Data data(*arm.model);
  pinocchio::framesForwardKinematics(*arm.model, data, arm.q_nominal);
  in.p_c = data.oMf[arm.frame].translation() + Eigen::Vector3d(0.02, -0.01, 0.03);
  in.d_hat = Eigen::Vector3d(0.3, -0.4, 0.2).normalized();
  ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
  DecelMpcInput warm = in;
  ShiftUniform(res, p.dt, 0.02, warm);
  ASSERT_TRUE(mpc.Solve(warm, res)) << DecelMpcReasonName(res.reason);
  ASSERT_FALSE(res.presolved);
  Probe probe;
  AddQp(probe, mpc.MainQp());
  AddSolution(probe, res);
  ExpectGolden("perp_7r_warm", probe, kGolden_perp_7r_warm_tight, kGolden_perp_7r_warm_loose);
}

// The shipped stop horizon (MD-24) with a torque row that binds.
TEST(DecelMpcGolden, ShippedHorizonWithABindingTorqueRow) {
  const ArmModel arm = RealArm6();
  DecelMpcParams p;
  p.n_nodes = 14;
  p.dt = 0.025;
  p.n_blocks = 6;
  p.block_sizes = {1, 1, 2, 2, 4, 4};
  DecelMpcLimits lim = LimitsFromModel(*arm.model, 0.1);
  // Fixed numbers, not a search: the first joint's limit sits below what this
  // stop asks of it, so its rows bind and the slack is exercised.
  lim.tau_max *= 10.0;
  lim.tau_max[0] = 15.0;
  DecelMpc mpc;
  ASSERT_EQ(mpc.Init(*arm.model, arm.frame, p, lim), DecelMpcReason::kNone);
  DecelMpcResult res;
  mpc.ResizeResult(res);
  DecelMpcInput in = RestInput(arm.q_nominal);
  in.qd0 << 1.2, -0.6, 0.8, 0.5, 0.4, 0.4;
  ASSERT_TRUE(mpc.Solve(in, res)) << DecelMpcReasonName(res.reason);
  // The premise: a torque row is at (or past) its bound.
  ASSERT_GE(res.tau_ratio_max, p.eta_tau - 1e-3) << "the torque rows are not active";
  Probe probe;
  AddQp(probe, mpc.PresolveQp());
  AddQp(probe, mpc.MainQp());
  AddSolution(probe, res);
  ExpectGolden("torque_6r_shipped", probe, kGolden_torque_6r_shipped_tight,
               kGolden_torque_6r_shipped_loose);
}

}  // namespace
