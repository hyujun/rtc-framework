// The locomotion whole-body QP over a walking-like contact schedule
// (loco_gait.hpp): the support set changes four times a cycle while the
// kinematics stay consistent with it.
//
//   * the default solver settings' trace (a, λ, τ, status, iterations of every
//     tick of six cycles) is pinned by its digest, so a change that is meant to
//     leave the defaults alone is seen to;
//   * the warm start that resets the proximal step, with the released
//     contacts' forces pinned to zero, against the defaults as the control
//     group and a cold (equality-constrained) solve of the same QP as the
//     reference, with and without estimator-like noise: it converges on every
//     tick but a few, never twice in a row, settles a support change in a few
//     iterations, and its solution is the reference's optimum — no multiplier
//     on a row whose bound is slack, the same torque, the same objective. The
//     control group shows each of those failing, on the same ticks.

#include "loco_gait.hpp"
#include "loco_wbc_fixture.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

std::uint64_t Fnv1a(std::uint64_t h, const void* data, std::size_t n) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h ^= bytes[i];
    h *= 1099511628211ULL;
  }
  return h;
}

std::uint64_t Fnv1a(std::uint64_t h, const Eigen::VectorXd& v) {
  return Fnv1a(h, v.data(), static_cast<std::size_t>(v.size()) * sizeof(double));
}

// The digest of six cycles of the default settings (regularised inactive
// forces, the previous-result warm start), pinned on GCC / x86-64 in Release.
// Other targets (aarch64's fused multiply-add, a Debug or sanitizer build)
// give other bits for the same source, so there it is recorded, not compared.
// It is a number to compare against, not a claim that these torques are
// right: a change meant to alter the default trace replaces it in a commit of
// its own that says why.
constexpr std::uint64_t kDefaultTraceDigest = 0x206fed7e0217ef60ULL;

// Six cycles: enough for the default warm start to miss on some ticks, so the
// ticks after a miss are pinned too.
constexpr int kTraceCycles = 6;

TEST(LocomotionGaitTest, TheDefaultSettingsTraceIsUnchanged) {
  test::LocoRig rig;
  test::LocoGait gait(*rig.model);
  std::uint64_t h = 14695981039346656037ULL;
  int valid = 0;
  for (int t = 0; t < kTraceCycles * test::LocoGait::kCycle; ++t) {
    gait.Apply(t, rig);
    const CommandOutput& out = rig.Solve();
    valid += out.tau_valid ? 1 : 0;
    h = Fnv1a(h, out.a_opt);
    h = Fnv1a(h, out.lambda_opt);
    h = Fnv1a(h, out.tau);
    h = Fnv1a(h, &out.qp_status, sizeof(out.qp_status));
    h = Fnv1a(h, &out.qp_iterations, sizeof(out.qp_iterations));
  }
  char hex[17];
  std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(h));
  RecordProperty("default_trace_digest", hex);
  RecordProperty("default_trace_valid_ticks", std::to_string(valid));
#if defined(__x86_64__) && defined(NDEBUG) && !defined(__SANITIZE_ADDRESS__)
  EXPECT_EQ(h, kDefaultTraceDigest)
      << "the default trace changed: digest " << hex
      << ". If the change is meant to alter it, replace the constant in its own commit with "
         "the reason; otherwise the defaults are not preserved.";
#endif
}

// ── The reset-μ warm start with zero inactive forces ─────────────────────────

// The shipped locomotion QP with the reset-μ warm start and zero inactive forces.
YAML::Node TreatmentYaml() {
  YAML::Node n = YAML::Load(test::kLocoWbq);
  n["solver"]["warm_start"] = "previous_result_reset_mu";
  n["constraints"]["contact"]["inactive_lambda"] = "zero";
  return n;
}

const WQPFormulation& Wqp(test::LocoRig& rig) {
  return dynamic_cast<const WQPFormulation&>(rig.b.Controller().Formulation());
}

// max |z_i| over the inequality rows whose bound z_i points at (z > 0: upper,
// z < 0: lower) has slack > 1e-4 — a multiplier the optimum cannot have. Rows
// with an infinite bound count (their slack is infinite).
double WrongMultiplier(const QPData& qp, const Eigen::VectorXd& x, const Eigen::VectorXd& z) {
  const Eigen::VectorXd cx = qp.C * x;
  double out = 0.0;
  for (Eigen::Index i = 0; i < cx.size(); ++i) {
    const double slack = z(i) > 0.0 ? qp.u(i) - cx(i) : (z(i) < 0.0 ? cx(i) - qp.l(i) : 0.0);
    if (slack > 1e-4) {
      out = std::max(out, std::abs(z(i)));
    }
  }
  return out;
}

double Objective(const QPData& qp, const Eigen::VectorXd& x) {
  return 0.5 * x.dot(qp.H * x) + qp.g.dot(x);
}

// max |λ| over the slots not in contact, at the iterate x = [a; λ].
double InactiveForce(const test::LocoRig& rig, const Eigen::VectorXd& x) {
  double out = 0.0;
  int offset = rig.info.nv;
  for (std::size_t i = 0; i < rig.contacts.contacts.size(); ++i) {
    const int dim = rig.b.Contacts().contacts[i].contact_dim;
    if (!rig.contacts.contacts[i].active) {
      out = std::max(out, x.segment(offset, dim).lpNorm<Eigen::Infinity>());
    }
    offset += dim;
  }
  return out;
}

// τ = S (M a + h − Σ_active Jcᵀ λ) of the iterate x = [a; λ].
Eigen::VectorXd TorqueOf(const test::LocoRig& rig, const Eigen::VectorXd& x) {
  const int nv = rig.info.nv;
  Eigen::VectorXd f = rig.cache.M * x.head(nv) + rig.cache.h;
  int offset = nv;
  for (std::size_t i = 0; i < rig.contacts.contacts.size(); ++i) {
    const int dim = rig.b.Contacts().contacts[i].contact_dim;
    if (rig.contacts.contacts[i].active) {
      f.noalias() -=
          rig.cache.contact_frames[i].J.topRows(dim).transpose() * x.segment(offset, dim);
    }
    offset += dim;
  }
  return rig.info.S * f;
}

std::string Fmt(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.3g", v);
  return buf;
}

double Percentile(std::vector<double> v, double p) {
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(v.begin(), v.end());
  const auto i = static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size()))) - 1;
  return v[std::min(i, v.size() - 1)];
}

struct Group {
  int misses{0};
  int miss_pairs{0};  // two graded ticks in a row that did not converge
  int max_transition_iterations{0};
  double max_wrong_multiplier{0.0};
  double max_inactive_force{0.0};
  bool last_missed{false};

  void Count(const CommandOutput& out, bool transition) {
    const bool miss = !out.tau_valid;
    misses += miss ? 1 : 0;
    miss_pairs += (miss && last_missed) ? 1 : 0;
    last_missed = miss;
    if (transition) {
      max_transition_iterations = std::max(max_transition_iterations, out.qp_iterations);
    }
  }
};

void RecordGroup(const std::string& run, const char* name, const Group& g) {
  const std::string k = run + "_" + name + "_";
  ::testing::Test::RecordProperty(k + "misses", std::to_string(g.misses));
  ::testing::Test::RecordProperty(k + "miss_pairs", std::to_string(g.miss_pairs));
  ::testing::Test::RecordProperty(k + "transition_iterations_max",
                                  std::to_string(g.max_transition_iterations));
  ::testing::Test::RecordProperty(k + "wrong_multiplier_max", Fmt(g.max_wrong_multiplier));
  ::testing::Test::RecordProperty(k + "inactive_force_max", Fmt(g.max_inactive_force));
}

// One run: six cycles, the first one a warm-up (not graded) — 5500 graded
// ticks, 20 of them support changes.
void RunGaitGates(double noise_q, const std::string& run) {
  constexpr int kCycles = 6;
  constexpr double kEpsAbs = 1e-6;  // the rig's solver.eps_abs
  test::LocoRig treat(TreatmentYaml());
  test::LocoRig control;
  test::LocoGait gait_t(*treat.model, noise_q);
  test::LocoGait gait_c(*control.model, noise_q);
  ASSERT_EQ(Wqp(treat).SolverConfig().warm_start, WarmStartMode::kPreviousResultResetMu);
  ASSERT_EQ(Wqp(control).SolverConfig().warm_start, WarmStartMode::kPreviousResult);

  // The reference: the same QP solved from the equality-constrained start.
  const QPData& qp_t = Wqp(treat).LastQpForTesting();
  QPSolverWrapper reference;
  reference.Init(static_cast<int>(qp_t.H.rows()), static_cast<int>(qp_t.A.rows()),
                 static_cast<int>(qp_t.C.rows()), Wqp(treat).SolverConfig());

  Group t;
  Group c;
  int graded = 0;
  int transitions = 0;
  int compared = 0;
  double max_dtau = 0.0;
  double max_da = 0.0;
  double max_gap = -std::numeric_limits<double>::infinity();
  double max_pri_mismatch = 0.0;
  std::vector<double> tick_us;
  std::vector<double> tick_us_steady;
  for (int k = 0; k < kCycles * test::LocoGait::kCycle; ++k) {
    const bool transition = test::LocoGait::IsTransition(k);
    const auto t0 = std::chrono::steady_clock::now();
    gait_t.Apply(k, treat);
    const CommandOutput& out_t = treat.Solve();
    const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    gait_c.Apply(k, control);
    const CommandOutput& out_c = control.Solve();
    if (k < test::LocoGait::kCycle) {
      continue;
    }
    ++graded;
    transitions += transition ? 1 : 0;
    tick_us.push_back(us);
    if (!transition) {
      tick_us_steady.push_back(us);
    }
    t.Count(out_t, transition);
    c.Count(out_c, transition);

    if (out_c.tau_valid) {
      const QPData& qp = Wqp(control).LastQpForTesting();
      const QPSolverWrapper& s = Wqp(control).SolverForTesting();
      c.max_wrong_multiplier =
          std::max(c.max_wrong_multiplier, WrongMultiplier(qp, s.Primal(), s.InequalityDual()));
      c.max_inactive_force = std::max(c.max_inactive_force, InactiveForce(control, s.Primal()));
    }
    if (!out_t.tau_valid) {
      continue;
    }
    const QPSolverWrapper& s = Wqp(treat).SolverForTesting();
    const Eigen::VectorXd x = s.Primal();
    t.max_wrong_multiplier =
        std::max(t.max_wrong_multiplier, WrongMultiplier(qp_t, x, s.InequalityDual()));
    t.max_inactive_force = std::max(t.max_inactive_force, InactiveForce(treat, x));
    double pri = 0.0;
    {
      const Eigen::VectorXd e = qp_t.A * x - qp_t.b;
      pri = e.lpNorm<Eigen::Infinity>();
      const Eigen::VectorXd cx = qp_t.C * x;
      for (Eigen::Index i = 0; i < cx.size(); ++i) {
        pri = std::max({pri, cx(i) - qp_t.u(i), qp_t.l(i) - cx(i)});
      }
    }
    max_pri_mismatch = std::max(max_pri_mismatch, std::abs(out_t.qp_pri_res - pri));

    reference.ResetWarmStart();
    const SolveResult& r = reference.Solve(qp_t);
    if (!r.converged) {
      continue;
    }
    ++compared;
    const Eigen::VectorXd x_ref = reference.Primal();
    max_dtau = std::max(max_dtau, (out_t.tau - TorqueOf(treat, x_ref)).lpNorm<Eigen::Infinity>());
    max_da = std::max(max_da, (out_t.a_opt - x_ref.head(treat.info.nv)).lpNorm<Eigen::Infinity>());
    max_gap = std::max(max_gap, Objective(qp_t, x) - Objective(qp_t, x_ref));
  }

  ::testing::Test::RecordProperty(run + "_graded", std::to_string(graded));
  ::testing::Test::RecordProperty(run + "_compared", std::to_string(compared));
  ::testing::Test::RecordProperty(run + "_dtau_max", Fmt(max_dtau));
  ::testing::Test::RecordProperty(run + "_da_max", Fmt(max_da));
  ::testing::Test::RecordProperty(run + "_objective_gap_max", Fmt(max_gap));
  ::testing::Test::RecordProperty(run + "_pri_res_mismatch_max", Fmt(max_pri_mismatch));
  RecordGroup(run, "treatment", t);
  RecordGroup(run, "control", c);
  const double p50 = Percentile(tick_us, 0.5);
  const double p99 = Percentile(tick_us_steady, 0.99);
  const double max_us = Percentile(tick_us, 1.0);
  ::testing::Test::RecordProperty(run + "_tick_us_p50", Fmt(p50));
  ::testing::Test::RecordProperty(run + "_tick_us_steady_p99", Fmt(p99));
  ::testing::Test::RecordProperty(run + "_tick_us_max", Fmt(max_us));

  ASSERT_EQ(graded, 5 * test::LocoGait::kCycle);
  ASSERT_EQ(transitions, 20);
  // Convergence: at most 2 misses in 5500 ticks, never two in a row; the
  // control group misses at least 50 times and in a row at least once.
  EXPECT_LE(t.misses, 2);
  EXPECT_EQ(t.miss_pairs, 0);
  EXPECT_GE(c.misses, 50)
      << "the control no longer misses: this fixture cannot tell the modes apart";
  EXPECT_GE(c.miss_pairs, 1);
  // A support change settles within 20 inner iterations.
  EXPECT_LE(t.max_transition_iterations, 20);
  // Optimality: no multiplier on a slack row, the reference's torque,
  // acceleration and objective.
  EXPECT_LE(t.max_wrong_multiplier, 1e-5);
  EXPECT_GE(compared, graded - 2);
  EXPECT_LE(max_dtau, 1e-3);
  EXPECT_LE(max_da, 1e-4);
  EXPECT_LE(max_gap, 1e-3);
  // The released contacts carry no force in the QP itself; the control's
  // regularised ones drift.
  EXPECT_LE(t.max_inactive_force, 2.0 * kEpsAbs);
  EXPECT_GE(c.max_inactive_force, 1.0);
  // pri_res is the residual of the returned iterate, in the QP's units.
  EXPECT_LE(max_pri_mismatch, 1e-8);
  if (noise_q > 0.0) {
    EXPECT_GE(c.max_wrong_multiplier, 1e-2) << "the control's non-optimal SOLVED is gone";
  }

  // Time (tick = gait step with the cache update + Compute): judged on a
  // control PC only (RTC_RUN_TIMING_GATES=1); elsewhere recorded.
  const char* gate = std::getenv("RTC_RUN_TIMING_GATES");
  if (gate != nullptr && std::strcmp(gate, "1") == 0) {
    constexpr double kSteadyP99Us = 500.0;  // provisional
    EXPECT_LE(p99, kSteadyP99Us);
    EXPECT_LE(max_us, 2.0 * kSteadyP99Us);
  }
}

TEST(LocomotionGaitTest, TheResetMuWarmStartWithZeroInactiveForcesSolvesEveryTickOptimally) {
  RunGaitGates(0.0, "noise0");
}

TEST(LocomotionGaitTest, TheResetMuWarmStartStaysOptimalUnderStateNoise) {
  RunGaitGates(1e-3, "noise1e-3");
}

}  // namespace
}  // namespace rtc::tsid
