// Unit tests for rtc::grasp::PullForceEstimator (#167).
//
// Geometry used throughout (unless a test overrides it): common reference
// frame = world, object plane = xy (normal +z), two-contact opposing pinch:
//   contact 0 "thumb" below the object → contact normal (object→finger) = -z,
//   contact 1 "index" above the object → contact normal = +z.
// Tests feed finger-on-object forces directly with force_sign = +1 and
// R = I so expected sums are exact; dedicated tests cover sign inversion and
// FK rotation. Exact math asserts use force_raw (pre-filter); filter-path
// tests settle the Bessel LPF first.

#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using rtc::grasp::PullContactConfig;
using rtc::grasp::PullContactInput;
using rtc::grasp::PullEstimate;
using rtc::grasp::PullEstimatorParams;
using rtc::grasp::PullForceEstimator;

constexpr double kDt = 1.0 / 500.0;
const Eigen::Vector3d kPlaneNormal = Eigen::Vector3d::UnitZ();

PullContactConfig MakeConfig(const Eigen::Vector3d& normal_local, bool required = false,
                             double force_sign = 1.0) {
  PullContactConfig cfg;
  cfg.contact_normal_local = normal_local;
  cfg.required = required;
  cfg.force_sign = force_sign;
  return cfg;
}

PullContactInput MakeInput(const Eigen::Vector3d& force,
                           const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity()) {
  PullContactInput in;
  in.force = force;
  in.rotation = rotation;
  in.valid = true;
  return in;
}

/// Two-contact opposing pinch on the xy plane, finger-on-object convention.
std::vector<PullContactConfig> PinchConfigs() {
  return {MakeConfig(-Eigen::Vector3d::UnitZ()),  // thumb (below, pushes +z)
          MakeConfig(Eigen::Vector3d::UnitZ())};  // index (above, pushes -z)
}

PullEstimatorParams DefaultParams() {
  PullEstimatorParams params;
  params.sample_rate_hz = 500.0;
  params.filter_cutoff_hz = 5.0;
  return params;
}

/// Run `n` identical ticks (settles the LPF) and return the last estimate.
const PullEstimate& RunTicks(PullForceEstimator& est, const std::vector<PullContactInput>& inputs,
                             int n, const Eigen::Vector3d& normal = kPlaneNormal) {
  for (int i = 0; i < n - 1; ++i) {
    est.Update(inputs, normal, kDt);
  }
  return est.Update(inputs, normal, kDt);
}

// ── 1. FK rotation maps link-frame force to the reference frame ─────────────
TEST(PullForceEstimator, AxisRotationMapsForceToReference) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  const std::vector<PullContactConfig> configs = {MakeConfig(-Eigen::Vector3d::UnitZ())};
  est.Init(configs, params);

  // Rz(90°): link +x → reference +y; the contact normal (-z) is unchanged.
  const Eigen::Matrix3d rot =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // Link frame: grip +5 z (pushing up), shear -1 x.
  const std::vector<PullContactInput> inputs = {MakeInput(Eigen::Vector3d(-1.0, 0.0, 5.0), rot)};

  const PullEstimate& out = RunTicks(est, inputs, 1);
  ASSERT_TRUE(out.valid);
  // f_obj(ref) = (0, -1, 5); F̂ = -P∥ f_obj = (0, +1, 0).
  EXPECT_NEAR(out.force_raw.x(), 0.0, 1e-12);
  EXPECT_NEAR(out.force_raw.y(), 1.0, 1e-12);
  EXPECT_NEAR(out.force_raw.z(), 0.0, 1e-12);
}

// ── 2. Wire-contract sign inversion (env-on-fingertip → finger-on-object) ───
TEST(PullForceEstimator, SignInversionConvention) {
  PullForceEstimator est;
  // Default force_sign = -1: inputs follow the wire contract.
  const std::vector<PullContactConfig> configs = {
      MakeConfig(-Eigen::Vector3d::UnitZ(), false, -1.0),
      MakeConfig(Eigen::Vector3d::UnitZ(), false, -1.0)};
  est.Init(configs, DefaultParams());

  // Finger-on-object: thumb (-0.75, 0, 5), index (-0.75, 0, -5) — the object
  // is pulled +x, fingers resist with -x friction. Wire values are negated.
  const std::vector<PullContactInput> inputs = {MakeInput(Eigen::Vector3d(0.75, 0.0, -5.0)),
                                                MakeInput(Eigen::Vector3d(0.75, 0.0, 5.0))};

  const PullEstimate& out = RunTicks(est, inputs, 1);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.force_raw.x(), 1.5, 1e-12);
  EXPECT_NEAR(out.force_raw.y(), 0.0, 1e-12);
  EXPECT_NEAR(out.force_raw.z(), 0.0, 1e-12);
}

// ── 3. Opposing normal squeeze cancels (internal force ∈ null space) ────────
TEST(PullForceEstimator, OpposingSqueezeCancellation) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());

  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0)),    // thumb pushes up
      MakeInput(Eigen::Vector3d(0.0, 0.0, -5.0))};  // index pushes down

  const PullEstimate& out = RunTicks(est, inputs, 1);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(out.valid_contact_count, 2);
  EXPECT_NEAR(out.force_raw.norm(), 0.0, 1e-12);  // grip does not leak
}

// ── 4. Same-direction shear sums to the external pull ───────────────────────
TEST(PullForceEstimator, SameDirectionShearPull) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());

  // Object pulled +x with 1.5 N: fingers resist with Σ shear = -1.5 x.
  const std::vector<PullContactInput> inputs = {MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0)),
                                                MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0))};

  const PullEstimate& out = RunTicks(est, inputs, 1);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.force_raw.x(), 1.5, 1e-12);
  EXPECT_NEAR(out.force_raw.y(), 0.0, 1e-12);
  EXPECT_NEAR(out.force_raw.z(), 0.0, 1e-12);
}

// ── 5. Arbitrary plane normal: projection + 2D basis coordinates ────────────
TEST(PullForceEstimator, ArbitraryPlaneBasis) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  const Eigen::Vector3d n = Eigen::Vector3d(1.0, 1.0, 1.0).normalized();
  const std::vector<PullContactConfig> configs = {MakeConfig(-n)};
  est.Init(configs, params);

  // In-plane pull v ⊥ n; finger applies grip along +n plus resisting -v.
  const Eigen::Vector3d v = 2.0 * Eigen::Vector3d(1.0, -1.0, 0.0).normalized();
  const std::vector<PullContactInput> inputs = {MakeInput(5.0 * n - v)};

  const PullEstimate& out = RunTicks(est, inputs, 1000, n);
  ASSERT_TRUE(out.valid);
  // Raw: grip projected out exactly, pull recovered.
  EXPECT_NEAR((out.force_raw - v).norm(), 0.0, 1e-12);
  // Settled filter: 3D estimate ⊥ n and the 2D basis coords preserve norm.
  EXPECT_NEAR(out.force_filtered.dot(n), 0.0, 1e-9);
  EXPECT_NEAR((out.force_filtered - v).norm(), 0.0, 1e-6);
  EXPECT_NEAR(out.force_inplane.norm(), v.norm(), 1e-6);
}

// ── 6. Contact hysteresis on the normal force ────────────────────────────────
TEST(PullForceEstimator, ContactHysteresis) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  PullContactConfig cfg = MakeConfig(-Eigen::Vector3d::UnitZ());
  cfg.contact_on_threshold = 1.0;
  cfg.contact_off_threshold = 0.4;
  est.Init(std::vector<PullContactConfig>{cfg}, params);

  auto grip = [](double f_n) {
    return std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(0.0, 0.0, f_n))};
  };

  est.Update(grip(0.7), kPlaneNormal, kDt);  // between off/on, never engaged
  EXPECT_FALSE(est.contact_active(0));
  est.Update(grip(1.2), kPlaneNormal, kDt);  // crosses on-threshold
  EXPECT_TRUE(est.contact_active(0));
  est.Update(grip(0.7), kPlaneNormal, kDt);  // between: stays engaged
  EXPECT_TRUE(est.contact_active(0));
  est.Update(grip(0.3), kPlaneNormal, kDt);  // below off: disengages
  EXPECT_FALSE(est.contact_active(0));
  EXPECT_FALSE(est.estimate().valid);
}

// ── 7. Stale input and saturation both gate the contact out ─────────────────
TEST(PullForceEstimator, StaleAndSaturationGate) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());

  // Stale: valid=false on the index → only one active contact → invalid.
  std::vector<PullContactInput> inputs = {MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0)),
                                          MakeInput(Eigen::Vector3d(0.0, 0.0, -5.0))};
  inputs[1].valid = false;
  const PullEstimate& stale = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_EQ(stale.valid_contact_count, 1);
  EXPECT_FALSE(stale.valid);
  EXPECT_FALSE(stale.any_saturated);

  // Saturation: per-axis |raw| at the limit gates the contact + sets the flag.
  inputs[1].valid = true;
  inputs[1].force = Eigen::Vector3d(0.0, 0.0, -60.0);  // ≥ default 50 N
  const PullEstimate& sat = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_EQ(sat.valid_contact_count, 1);
  EXPECT_FALSE(sat.valid);
  EXPECT_TRUE(sat.any_saturated);
}

// ── 8. Friction utilization ratio and slip risk ──────────────────────────────
TEST(PullForceEstimator, SlipRatio) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  PullContactConfig cfg = MakeConfig(-Eigen::Vector3d::UnitZ());
  cfg.friction_coeff = 0.7;
  est.Init(std::vector<PullContactConfig>{cfg}, params);

  // f_n = 5, |f_t| = 1.75 → ρ = 1.75 / (0.7·5) = 0.5.
  const PullEstimate& low = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(1.75, 0.0, 5.0))}, kPlaneNormal, kDt);
  EXPECT_NEAR(low.max_friction_utilization, 0.5, 1e-6);
  EXPECT_FALSE(low.slip_risk);

  // |f_t| = 3.85 → ρ = 1.1 ≥ threshold (1.0).
  const PullEstimate& high = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(3.85, 0.0, 5.0))}, kPlaneNormal, kDt);
  EXPECT_NEAR(high.max_friction_utilization, 1.1, 1e-6);
  EXPECT_TRUE(high.slip_risk);
}

// ── 9. Required-role mask: thumb dropout invalidates despite enough contacts ─
TEST(PullForceEstimator, RequiredRoleGating) {
  PullForceEstimator est;
  // thumb required; index + middle above the object.
  const std::vector<PullContactConfig> configs = {
      MakeConfig(-Eigen::Vector3d::UnitZ(), /*required=*/true),
      MakeConfig(Eigen::Vector3d::UnitZ()), MakeConfig(Eigen::Vector3d::UnitZ())};
  est.Init(configs, DefaultParams());

  std::vector<PullContactInput> inputs = {MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0)),
                                          MakeInput(Eigen::Vector3d(0.0, 0.0, -2.5)),
                                          MakeInput(Eigen::Vector3d(0.0, 0.0, -2.5))};

  // Thumb sensor lost: 2 active contacts ≥ min(2) but the required role is
  // missing → must be invalid (index+middle-only false-valid guard).
  inputs[0].valid = false;
  const PullEstimate& dropped = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_EQ(dropped.valid_contact_count, 2);
  EXPECT_FALSE(dropped.valid);

  inputs[0].valid = true;
  const PullEstimate& full = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_EQ(full.valid_contact_count, 3);
  EXPECT_TRUE(full.valid);
}

// ── 10. Baseline subtraction removes constant in-plane residual ──────────────
TEST(PullForceEstimator, BaselineSubtraction) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());

  // Constant residual: both fingers biased -0.4 x (e.g. in-plane gravity).
  const std::vector<PullContactInput> rest = {MakeInput(Eigen::Vector3d(-0.4, 0.0, 5.0)),
                                              MakeInput(Eigen::Vector3d(-0.4, 0.0, -5.0))};
  const PullEstimate& before = est.Update(rest, kPlaneNormal, kDt);
  EXPECT_NEAR(before.force_raw.x(), 0.8, 1e-12);
  EXPECT_FALSE(before.baseline_applied);

  est.ArmBaseline();
  const PullEstimate& zeroed = est.Update(rest, kPlaneNormal, kDt);
  EXPECT_TRUE(zeroed.baseline_applied);
  EXPECT_NEAR(zeroed.force_raw.norm(), 0.0, 1e-12);

  // Additional external pull appears on top of the residual → only the pull.
  const std::vector<PullContactInput> pulled = {MakeInput(Eigen::Vector3d(-0.4 - 0.75, 0.0, 5.0)),
                                                MakeInput(Eigen::Vector3d(-0.4 - 0.75, 0.0, -5.0))};
  const PullEstimate& out = est.Update(pulled, kPlaneNormal, kDt);
  EXPECT_NEAR(out.force_raw.x(), 1.5, 1e-12);

  est.ClearBaseline();
  const PullEstimate& cleared = est.Update(rest, kPlaneNormal, kDt);
  EXPECT_FALSE(cleared.baseline_applied);
  EXPECT_NEAR(cleared.force_raw.x(), 0.8, 1e-12);
}

// ── 11. Bounded decay on contact loss (no hard zero) ─────────────────────────
TEST(PullForceEstimator, DecayOnContactLoss) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.decay_time_constant_s = 0.1;
  est.Init(PinchConfigs(), params);

  const std::vector<PullContactInput> pulling = {MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0)),
                                                 MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0))};
  const PullEstimate& settled = RunTicks(est, pulling, 1000);
  ASSERT_TRUE(settled.valid);
  const double settled_mag = settled.magnitude;
  EXPECT_NEAR(settled_mag, 1.5, 1e-6);

  // Total sensor loss: estimate decays exponentially, flagged invalid.
  std::vector<PullContactInput> lost = pulling;
  lost[0].valid = false;
  lost[1].valid = false;

  const PullEstimate& one = est.Update(lost, kPlaneNormal, kDt);
  EXPECT_FALSE(one.valid);
  const double expected_factor = std::exp(-kDt / params.decay_time_constant_s);
  EXPECT_NEAR(one.magnitude, settled_mag * expected_factor, 1e-9);
  EXPECT_GT(one.magnitude, 0.0);  // not a hard zero

  const PullEstimate& two = est.Update(lost, kPlaneNormal, kDt);
  EXPECT_NEAR(two.magnitude, settled_mag * expected_factor * expected_factor, 1e-9);
}

// ── 12. Directional scalar projection onto a known pull direction ───────────
TEST(PullForceEstimator, DirectionalProjection) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());
  est.SetPullDirection(Eigen::Vector3d(2.0, 0.0, 0.0));  // normalized internally

  const std::vector<PullContactInput> pulling = {MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0)),
                                                 MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0))};
  const PullEstimate& out = RunTicks(est, pulling, 1000);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.directional, 1.5, 1e-6);

  est.SetPullDirection(-Eigen::Vector3d::UnitX());
  const PullEstimate& negated = est.Update(pulling, kPlaneNormal, kDt);
  EXPECT_NEAR(negated.directional, -1.5, 1e-6);

  est.SetPullDirection(Eigen::Vector3d::Zero());  // degenerate → disabled
  const PullEstimate& disabled = est.Update(pulling, kPlaneNormal, kDt);
  EXPECT_EQ(disabled.directional, 0.0);
}

// ── Bonus: Init validation rejects ill-formed configs ────────────────────────
TEST(PullForceEstimator, InitRejectsInvalidConfig) {
  PullForceEstimator est;
  const PullEstimatorParams params = DefaultParams();

  EXPECT_THROW(est.Init({}, params), std::invalid_argument);

  PullContactConfig degenerate_normal = MakeConfig(Eigen::Vector3d::Zero());
  EXPECT_THROW(est.Init(std::vector<PullContactConfig>{degenerate_normal}, params),
               std::invalid_argument);

  PullContactConfig bad_hysteresis = MakeConfig(Eigen::Vector3d::UnitZ());
  bad_hysteresis.contact_on_threshold = 0.2;
  bad_hysteresis.contact_off_threshold = 0.5;  // on <= off
  EXPECT_THROW(est.Init(std::vector<PullContactConfig>{bad_hysteresis}, params),
               std::invalid_argument);

  PullEstimatorParams bad_min = params;
  bad_min.min_valid_contacts = 0;
  EXPECT_THROW(est.Init(PinchConfigs(), bad_min), std::invalid_argument);
}

// ── POD mirror for controller-owned SeqLock state (#167 P3) ──────────────────
TEST(PullForceEstimator, FillPullEstimateDataMirrorsAllFields) {
  PullEstimate in;
  in.force_filtered = Eigen::Vector3d(1.5, -2.5, 0.25);
  in.force_inplane = Eigen::Vector2d(1.5, -2.5);
  in.magnitude = 3.0;
  in.directional = 1.25;
  in.max_friction_utilization = 0.6;
  in.leakage_bound = 0.35;
  in.valid_contact_count = 3;
  in.valid = true;
  in.slip_risk = true;
  in.any_saturated = true;
  in.baseline_applied = true;

  rtc::grasp::PullEstimateData out;
  rtc::grasp::FillPullEstimateData(in, out);

  EXPECT_FLOAT_EQ(out.force[0], 1.5F);
  EXPECT_FLOAT_EQ(out.force[1], -2.5F);
  EXPECT_FLOAT_EQ(out.force[2], 0.25F);
  EXPECT_FLOAT_EQ(out.force_inplane[0], 1.5F);
  EXPECT_FLOAT_EQ(out.force_inplane[1], -2.5F);
  EXPECT_FLOAT_EQ(out.magnitude, 3.0F);
  EXPECT_FLOAT_EQ(out.directional, 1.25F);
  EXPECT_FLOAT_EQ(out.friction_utilization, 0.6F);
  EXPECT_FLOAT_EQ(out.leakage_bound, 0.35F);
  EXPECT_EQ(out.valid_contact_count, 3);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(out.slip_risk);
  EXPECT_TRUE(out.any_saturated);
  EXPECT_TRUE(out.baseline_applied);
}

}  // namespace
