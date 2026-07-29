// Unit tests for rtc::grasp::PullForceEstimator (#167).
//
// Geometry used throughout (unless a test overrides it): common reference
// frame = world, object plane = xy (normal +z), two-contact opposing pinch:
//   contact 0 "thumb" below the object → contact normal (object→finger) = -z,
//   contact 1 "index" above the object → contact normal = +z.
// The per-contact normal is a per-tick input (reference frame) — the caller
// supplies it from pinch geometry, so tests attach it to each PullContactInput
// (kThumbNormal / kOpposingNormal) rather than to the config. Tests feed
// finger-on-object forces directly with force_sign = +1 and R = I so expected
// sums are exact; dedicated tests cover sign inversion and FK rotation. Exact
// math asserts use force_raw (pre-filter); filter-path tests settle the Bessel
// LPF first.

#include "rtc_controllers/grasp/pull_force_estimator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace {

using rtc::grasp::PullContactConfig;
using rtc::grasp::PullContactInput;
using rtc::grasp::PullEstimate;
using rtc::grasp::PullEstimatorParams;
using rtc::grasp::PullForceEstimator;

constexpr double kDt = 1.0 / 500.0;
const Eigen::Vector3d kPlaneNormal = Eigen::Vector3d::UnitZ();
// Reference-frame contact normals for the opposing xy-plane pinch: thumb below
// the object (normal points down, from object into the thumb), the opposing
// index/middle above it (up). These are what the wiring derives from FK.
const Eigen::Vector3d kThumbNormal = -Eigen::Vector3d::UnitZ();
const Eigen::Vector3d kOpposingNormal = Eigen::Vector3d::UnitZ();

PullContactConfig MakeConfig(bool required = false, double force_sign = 1.0) {
  PullContactConfig cfg;
  cfg.required = required;
  cfg.force_sign = force_sign;
  return cfg;
}

PullContactInput MakeInput(const Eigen::Vector3d& force, const Eigen::Vector3d& contact_normal,
                           const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity()) {
  PullContactInput in;
  in.force = force;
  in.contact_normal = contact_normal;
  in.rotation = rotation;
  in.valid = true;
  return in;
}

/// Two-contact opposing pinch on the xy plane, finger-on-object convention.
std::vector<PullContactConfig> PinchConfigs() {
  return {MakeConfig(),   // thumb (below, pushes +z)
          MakeConfig()};  // index (above, pushes -z)
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
  const std::vector<PullContactConfig> configs = {MakeConfig()};
  est.Init(configs, params);

  // Rz(90°): link +x → reference +y; the reference-frame contact normal is -z.
  const Eigen::Matrix3d rot =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // Link frame: grip +5 z (pushing up), shear -1 x.
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(-1.0, 0.0, 5.0), kThumbNormal, rot)};

  const PullEstimate& out = RunTicks(est, inputs, 1);
  ASSERT_TRUE(out.valid);
  // f_obj(ref) = (0, -1, 5); F̂ = -P∥ f_obj = (0, +1, 0).
  EXPECT_NEAR(out.force_raw.x(), 0.0, 1e-12);
  EXPECT_NEAR(out.force_raw.y(), 1.0, 1e-12);
  EXPECT_NEAR(out.force_raw.z(), 0.0, 1e-12);
}

// ── 2. env-on-fingertip publisher → finger-on-object (force_sign = -1) ──────
TEST(PullForceEstimator, SignInversionConvention) {
  PullForceEstimator est;
  // The estimator assumes finger-on-object input (force_sign default +1);
  // force_sign = -1 is how a publisher following the FingertipSensor.msg wire
  // sign (env-on-fingertip) is wired in.
  const std::vector<PullContactConfig> configs = {MakeConfig(false, -1.0), MakeConfig(false, -1.0)};
  est.Init(configs, DefaultParams());

  // Finger-on-object: thumb (-0.75, 0, 5), index (-0.75, 0, -5) — the object
  // is pulled +x, fingers resist with -x friction. Wire values are negated.
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.75, 0.0, -5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(0.75, 0.0, 5.0), kOpposingNormal)};

  const PullEstimate& out = RunTicks(est, inputs, 1);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.force_raw.x(), 1.5, 1e-12);
  EXPECT_NEAR(out.force_raw.y(), 0.0, 1e-12);
  EXPECT_NEAR(out.force_raw.z(), 0.0, 1e-12);
}

// The default is the *input* convention, not a free parameter: the estimator
// sums finger-on-object forces, so an unset force_sign must pass the input
// through untouched. Flipping this default silently inverts every f_n gate
// (-n.f_obj), which reads as "grasped but no contact" — the #167 p1b failure.
TEST(PullForceEstimator, DefaultForceSignAssumesFingerOnObject) {
  EXPECT_DOUBLE_EQ(PullContactConfig{}.force_sign, 1.0);

  PullForceEstimator est;
  const std::vector<PullContactConfig> configs = {PullContactConfig{}, PullContactConfig{}};
  est.Init(configs, DefaultParams());

  // Same finger-on-object pinch as test 1, fed through the default config.
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0), kOpposingNormal)};

  const PullEstimate& out = RunTicks(est, inputs, 1);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(out.valid_contact_count, 2);
  EXPECT_NEAR(out.force_raw.x(), 1.5, 1e-12);
}

// ── 3. Opposing normal squeeze cancels (internal force ∈ null space) ────────
TEST(PullForceEstimator, OpposingSqueezeCancellation) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());

  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),       // thumb pushes up
      MakeInput(Eigen::Vector3d(0.0, 0.0, -5.0), kOpposingNormal)};  // index pushes down

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
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0), kOpposingNormal)};

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
  const std::vector<PullContactConfig> configs = {MakeConfig()};
  est.Init(configs, params);

  // In-plane pull v ⊥ n; finger applies grip along +n plus resisting -v.
  const Eigen::Vector3d v = 2.0 * Eigen::Vector3d(1.0, -1.0, 0.0).normalized();
  const std::vector<PullContactInput> inputs = {MakeInput(5.0 * n - v, -n)};

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
  PullContactConfig cfg = MakeConfig();
  cfg.contact_on_threshold = 1.0;
  cfg.contact_off_threshold = 0.4;
  est.Init(std::vector<PullContactConfig>{cfg}, params);

  auto grip = [](double f_n) {
    return std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(0.0, 0.0, f_n), kThumbNormal)};
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
  std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, -5.0), kOpposingNormal)};
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
  PullContactConfig cfg = MakeConfig();
  cfg.friction_coeff = 0.7;
  est.Init(std::vector<PullContactConfig>{cfg}, params);

  // f_n = 5, |f_t| = 1.75 → ρ = 1.75 / (0.7·5) = 0.5.
  const PullEstimate& low = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(1.75, 0.0, 5.0), kThumbNormal)},
      kPlaneNormal, kDt);
  EXPECT_NEAR(low.max_friction_utilization, 0.5, 1e-6);
  EXPECT_FALSE(low.slip_risk);

  // |f_t| = 3.85 → ρ = 1.1 ≥ threshold (1.0).
  const PullEstimate& high = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(3.85, 0.0, 5.0), kThumbNormal)},
      kPlaneNormal, kDt);
  EXPECT_NEAR(high.max_friction_utilization, 1.1, 1e-6);
  EXPECT_TRUE(high.slip_risk);
}

// ── 9. Required-role mask: thumb dropout invalidates despite enough contacts ─
TEST(PullForceEstimator, RequiredRoleGating) {
  PullForceEstimator est;
  // thumb required; index + middle above the object.
  const std::vector<PullContactConfig> configs = {MakeConfig(/*required=*/true), MakeConfig(),
                                                  MakeConfig()};
  est.Init(configs, DefaultParams());

  std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, -2.5), kOpposingNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, -2.5), kOpposingNormal)};

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
  const std::vector<PullContactInput> rest = {
      MakeInput(Eigen::Vector3d(-0.4, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(-0.4, 0.0, -5.0), kOpposingNormal)};
  const PullEstimate& before = est.Update(rest, kPlaneNormal, kDt);
  EXPECT_NEAR(before.force_raw.x(), 0.8, 1e-12);
  EXPECT_FALSE(before.baseline_applied);

  est.ArmBaseline();
  const PullEstimate& zeroed = est.Update(rest, kPlaneNormal, kDt);
  EXPECT_TRUE(zeroed.baseline_applied);
  EXPECT_NEAR(zeroed.force_raw.norm(), 0.0, 1e-12);

  // Additional external pull appears on top of the residual → only the pull.
  const std::vector<PullContactInput> pulled = {
      MakeInput(Eigen::Vector3d(-0.4 - 0.75, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(-0.4 - 0.75, 0.0, -5.0), kOpposingNormal)};
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

  const std::vector<PullContactInput> pulling = {
      MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0), kOpposingNormal)};
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

  const std::vector<PullContactInput> pulling = {
      MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0), kOpposingNormal)};
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

// ── 13. Per-tick contact normal drives the gate (not a body-fixed axis) ─────
TEST(PullForceEstimator, PerTickNormalDrivesGate) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  PullContactConfig cfg = MakeConfig();
  cfg.contact_on_threshold = 1.0;
  cfg.contact_off_threshold = 0.4;
  est.Init(std::vector<PullContactConfig>{cfg}, params);

  // Same +5 z force. With normal -z the compression f_n = 5 → contact engages.
  est.Update(std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal)},
             kPlaneNormal, kDt);
  EXPECT_TRUE(est.contact_active(0));

  // Rotate the per-tick normal to +x (⊥ the force): f_n = 0 → disengages. The
  // gate follows the supplied normal, proving it is not a body-fixed axis.
  est.Update(std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0),
                                                     Eigen::Vector3d::UnitX())},
             kPlaneNormal, kDt);
  EXPECT_FALSE(est.contact_active(0));
}

// ── 14. Degenerate per-tick contact normal skips the contact ────────────────
TEST(PullForceEstimator, DegenerateContactNormalSkipsContact) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());

  std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, -5.0), kOpposingNormal)};
  // Zero normal on the index (e.g. missing FK positions) → that contact is
  // unusable this tick → only one active contact → invalid.
  inputs[1].contact_normal = Eigen::Vector3d::Zero();
  const PullEstimate& out = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_EQ(out.valid_contact_count, 1);
  EXPECT_FALSE(out.valid);
}

// ── Bonus: Init validation rejects ill-formed configs ────────────────────────
TEST(PullForceEstimator, InitRejectsInvalidConfig) {
  PullForceEstimator est;
  const PullEstimatorParams params = DefaultParams();

  EXPECT_THROW(est.Init({}, params), std::invalid_argument);

  PullContactConfig bad_hysteresis = MakeConfig();
  bad_hysteresis.contact_on_threshold = 0.2;
  bad_hysteresis.contact_off_threshold = 0.5;  // on <= off
  EXPECT_THROW(est.Init(std::vector<PullContactConfig>{bad_hysteresis}, params),
               std::invalid_argument);

  PullContactConfig bad_saturation = MakeConfig();
  bad_saturation.force_saturation = 0.0;  // must be > 0
  EXPECT_THROW(est.Init(std::vector<PullContactConfig>{bad_saturation}, params),
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

// ── In-plane basis is named by inplane_x_reference ───────────────────────────
// Without a reference the 2-D pair is only norm-meaningful (see
// ArbitraryPlaneBasis); with one, [0] is the anti-gravity component and [1] the
// horizontal one, which is what makes a pull-gauge trace readable directly.
TEST(PullForceEstimator, InPlaneAxesFollowTheConfiguredReference) {
  // Pinch axis horizontal (+x) so the plane contains the vertical: e_x = +z
  // exactly, e_y = n x e_x = -y.
  const Eigen::Vector3d n = Eigen::Vector3d::UnitX();
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  params.inplane_x_reference = Eigen::Vector3d::UnitZ();
  const std::vector<PullContactConfig> configs = {MakeConfig()};

  {  // Purely vertical pull → all of it on axis [0].
    PullForceEstimator est;
    est.Init(configs, params);
    // grip +5 along the pinch axis, resisting -2 z ⇒ F̂ = +2 z.
    const std::vector<PullContactInput> inputs = {
        MakeInput(Eigen::Vector3d(5.0, 0.0, -2.0), -Eigen::Vector3d::UnitX())};
    const PullEstimate& out = RunTicks(est, inputs, 1000, n);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.force_inplane[0], 2.0, 1e-6);
    EXPECT_NEAR(out.force_inplane[1], 0.0, 1e-6);
  }
  {  // Purely horizontal pull → all of it on axis [1] (e_y = -y).
    PullForceEstimator est;
    est.Init(configs, params);
    const std::vector<PullContactInput> inputs = {
        MakeInput(Eigen::Vector3d(5.0, -2.0, 0.0), -Eigen::Vector3d::UnitX())};
    const PullEstimate& out = RunTicks(est, inputs, 1000, n);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.force_inplane[0], 0.0, 1e-6);
    EXPECT_NEAR(out.force_inplane[1], -2.0, 1e-6);
  }
}

TEST(PullForceEstimator, InPlaneAxesStayFiniteWhenPlaneIsEdgeOnToTheReference) {
  // Pinch axis parallel to the reference: the projection collapses, so the
  // basis must fall back rather than emit NaN or a zero axis.
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  params.inplane_x_reference = Eigen::Vector3d::UnitZ();
  const std::vector<PullContactConfig> configs = {MakeConfig()};
  est.Init(configs, params);

  const Eigen::Vector3d v(1.5, 0.0, 0.0);
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0) - v, kThumbNormal)};
  const PullEstimate& out = RunTicks(est, inputs, 1000, kPlaneNormal);
  ASSERT_TRUE(out.valid);
  EXPECT_TRUE(out.force_inplane.allFinite());
  EXPECT_NEAR(out.force_inplane.norm(), v.norm(), 1e-6);
}

// ═══════════════════════════════════════════════════════════════════════════
// #234 — self-description, recovery, validation, slip gate, touch thresholds
// ═══════════════════════════════════════════════════════════════════════════

// ── AC-1: the wire alone must reconstruct the in-plane pair ─────────────────
//
// force_inplane is expressed in a basis rebuilt from the observed pinch axis
// every tick, so two numbers on their own are not comparable across ticks and
// not invertible at all. With plane_normal + basis_x published, e_y = n x e_x
// and the 2-D pair maps back onto the 3-D in-plane force exactly.
Eigen::Vector3d ReconstructInPlane(const PullEstimate& est) {
  const Eigen::Vector3d e_y = est.plane_normal.cross(est.basis_x);
  return est.basis_x * est.force_inplane.x() + e_y * est.force_inplane.y();
}

TEST(PullForceEstimator, PublishedBasisReconstructsTheInPlaneForce) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  params.inplane_x_reference = Eigen::Vector3d::UnitZ();
  est.Init(std::vector<PullContactConfig>{MakeConfig()}, params);

  // Pinch axis horizontal, so the reference (+z) lies in the plane.
  const Eigen::Vector3d n = Eigen::Vector3d::UnitX();
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(5.0, -1.25, -2.0), -Eigen::Vector3d::UnitX())};
  const PullEstimate& out = RunTicks(est, inputs, 1000, n);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(out.basis_source, rtc::grasp::PullBasisSource::kReference);
  EXPECT_EQ(out.invalid_reason, rtc::grasp::PullInvalidReason::kNone);

  // The published normal is the unit normal actually used.
  EXPECT_NEAR(out.plane_normal.norm(), 1.0, 1e-12);
  EXPECT_NEAR(out.basis_x.norm(), 1.0, 1e-12);
  EXPECT_NEAR(out.basis_x.dot(out.plane_normal), 0.0, 1e-12);

  const Eigen::Vector3d rebuilt = ReconstructInPlane(out);
  EXPECT_NEAR((rebuilt - out.force_filtered).norm(), 0.0, 1e-9)
      << "rebuilt=" << rebuilt.transpose() << " filtered=" << out.force_filtered.transpose();
}

TEST(PullForceEstimator, BasisIsReconstructableThroughTheCarryFallback) {
  // The reference-projection branch collapses when the plane turns edge-on to
  // the reference; the tick then falls back to re-projecting the previous e_x,
  // which is *path-dependent* — the exact case a consumer cannot reproduce
  // from config alone, so basis_x must be on the wire for it.
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  params.inplane_x_reference = Eigen::Vector3d::UnitZ();
  est.Init(std::vector<PullContactConfig>{MakeConfig()}, params);

  // Contact gating is driven by contact_normal, which we hold fixed, so only
  // the plane normal moves between the two ticks.
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(1.0, -0.5, 5.0), kThumbNormal)};

  // Tick A: plane tilted 45 deg — e_x is neither the reference nor a world axis.
  const Eigen::Vector3d tilted = Eigen::Vector3d(0.0, 1.0, 1.0).normalized();
  const PullEstimate& first = est.Update(inputs, tilted, kDt);
  ASSERT_TRUE(first.valid);
  ASSERT_EQ(first.basis_source, rtc::grasp::PullBasisSource::kReference);

  // Tick B: plane normal now parallel to the reference → reference collapses,
  // but the carried e_x still has an in-plane component.
  const PullEstimate& second = est.Update(inputs, Eigen::Vector3d::UnitZ(), kDt);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(second.basis_source, rtc::grasp::PullBasisSource::kCarry);

  // The plane moved between the two ticks, so the filter output — a low-pass
  // over samples taken in *different* planes — carries a small component along
  // the current normal. The 2-D pair spans the plane only, so the reconstruction
  // recovers P∥(force_filtered), which is the whole of what force_inplane
  // claims to describe.
  const Eigen::Vector3d n = second.plane_normal;
  const Eigen::Vector3d in_plane = second.force_filtered - n * (n.dot(second.force_filtered));
  const Eigen::Vector3d rebuilt = ReconstructInPlane(second);
  EXPECT_NEAR((rebuilt - in_plane).norm(), 0.0, 1e-9)
      << "rebuilt=" << rebuilt.transpose() << " in_plane=" << in_plane.transpose();
}

TEST(PullForceEstimator, BasisSourceReportsTheWorldAxisSeed) {
  // No reference configured and no carry yet → world-axis seed. Reported so a
  // consumer does not read force_inplane[0] as "the vertical component".
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  params.inplane_x_reference = Eigen::Vector3d::Zero();
  est.Init(std::vector<PullContactConfig>{MakeConfig()}, params);

  const PullEstimate& out = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(1.0, 0.0, 5.0), kThumbNormal)},
      kPlaneNormal, kDt);
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(out.basis_source, rtc::grasp::PullBasisSource::kSeed);
  EXPECT_NEAR((ReconstructInPlane(out) - out.force_filtered).norm(), 0.0, 1e-9);
}

TEST(PullForceEstimator, InvalidTickPublishesNoPlaneOrBasis) {
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());
  const std::vector<PullContactInput> lost = {MakeInput(Eigen::Vector3d::Zero(), kThumbNormal),
                                              MakeInput(Eigen::Vector3d::Zero(), kOpposingNormal)};
  const PullEstimate& out = est.Update(lost, Eigen::Vector3d::Zero(), kDt);
  ASSERT_FALSE(out.valid);
  EXPECT_EQ(out.plane_normal.norm(), 0.0);
  EXPECT_EQ(out.basis_x.norm(), 0.0);
  EXPECT_EQ(out.basis_source, rtc::grasp::PullBasisSource::kNone);
}

TEST(PullForceEstimator, InvalidTickWithAUsablePlaneStillReportsTheNormal) {
  // plane_normal and basis_x are not the same gate: the normal describes what
  // the tick gated against and survives a contact-side failure, while the basis
  // only exists for ticks that produced an in-plane vector. Collapsing them
  // would make "the plane was fine, the thumb dropped" indistinguishable from
  // "there was no plane".
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 2;
  const std::vector<PullContactConfig> configs = {MakeConfig(/*required=*/true), MakeConfig()};
  est.Init(configs, params);

  std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, -5.0), kOpposingNormal)};
  inputs[0].valid = false;  // required role gone, plane normal untouched

  const PullEstimate& out = est.Update(inputs, kPlaneNormal, kDt);
  ASSERT_FALSE(out.valid);
  EXPECT_EQ(out.invalid_reason, rtc::grasp::PullInvalidReason::kRequiredContactMissing);
  EXPECT_NEAR(out.plane_normal.norm(), 1.0, 1e-12);
  EXPECT_EQ(out.basis_x.norm(), 0.0);
  EXPECT_EQ(out.basis_source, rtc::grasp::PullBasisSource::kNone);
}

// ── AC-1 / P-11: invalid_reason distinguishes the gates ─────────────────────
TEST(PullForceEstimator, InvalidReasonNamesTheFailedGate) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 2;
  const std::vector<PullContactConfig> configs = {MakeConfig(/*required=*/true), MakeConfig(),
                                                  MakeConfig()};
  est.Init(configs, params);

  std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, -2.5), kOpposingNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, -2.5), kOpposingNormal)};

  EXPECT_EQ(est.Update(inputs, kPlaneNormal, kDt).invalid_reason,
            rtc::grasp::PullInvalidReason::kNone);

  // Degenerate normal outranks everything: no plane, nothing else evaluated.
  EXPECT_EQ(est.Update(inputs, Eigen::Vector3d::Zero(), kDt).invalid_reason,
            rtc::grasp::PullInvalidReason::kDegenerateNormal);

  // Thumb (required) drops but two contacts remain — this is the case that
  // `valid=false` alone could not distinguish from "too few tips".
  inputs[0].valid = false;
  const PullEstimate& missing = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_EQ(missing.valid_contact_count, 2);
  EXPECT_EQ(missing.invalid_reason, rtc::grasp::PullInvalidReason::kRequiredContactMissing);

  // Thumb back, the other two gone → count below min, required role fine.
  inputs[0].valid = true;
  inputs[1].valid = false;
  inputs[2].valid = false;
  EXPECT_EQ(est.Update(inputs, kPlaneNormal, kDt).invalid_reason,
            rtc::grasp::PullInvalidReason::kInsufficientContacts);

  PullForceEstimator fresh;
  EXPECT_EQ(fresh.Update(inputs, kPlaneNormal, kDt).invalid_reason,
            rtc::grasp::PullInvalidReason::kNotInitialized);
}

// ── AC-1 / P-12 / P-14: the two contact sets are distinct and published ─────
TEST(PullForceEstimator, ContactAndTouchMasksMirrorThePerContactGates) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  est.Init(std::vector<PullContactConfig>{MakeConfig(), MakeConfig(), MakeConfig()}, params);

  // Contact 0 presses along the plane normal (in the sum), contact 1 presses
  // purely sideways (touching but f_n = 0, so not in the sum), contact 2 is
  // silent. The masks must therefore differ — which is precisely why
  // opposing_mask (touch-derived) is not the set that formed the estimate.
  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(5.0, 0.0, 0.0), kOpposingNormal),
      MakeInput(Eigen::Vector3d(0.0, 0.0, 0.0), kOpposingNormal)};
  const PullEstimate& out = est.Update(inputs, kPlaneNormal, kDt);

  EXPECT_EQ(out.contact_mask, 0b0000'0001U);
  EXPECT_EQ(out.touch_mask, 0b0000'0011U);
  EXPECT_EQ(out.valid_contact_count, 1);
  EXPECT_TRUE(est.contact_active(0));
  EXPECT_FALSE(est.contact_active(1));
  EXPECT_TRUE(est.touch_active(1));
}

// ── AC-2: no snapback on return from an invalid gap (P-4 / D3) ──────────────
//
// Four combinations: short/long gap x same/changed raw. Before the reseed the
// filter resumed from its pre-gap delay line and the first valid sample jumped
// straight back to the pre-gap force, undoing every decayed tick already
// published — the decay applied on the way down but not on the way up.
class PullRecovery : public ::testing::TestWithParam<std::tuple<int, double>> {};

TEST_P(PullRecovery, ReturnFromInvalidGapTracksTheCurrentForceNotThePreGapOne) {
  const auto [gap_ticks, return_pull] = GetParam();

  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.decay_time_constant_s = 0.1;
  est.Init(PinchConfigs(), params);

  auto pinch = [](double pull) {
    return std::vector<PullContactInput>{
        MakeInput(Eigen::Vector3d(-pull / 2.0, 0.0, 5.0), kThumbNormal),
        MakeInput(Eigen::Vector3d(-pull / 2.0, 0.0, -5.0), kOpposingNormal)};
  };

  // The input vector is named rather than passed as a temporary: RunTicks
  // returns a reference into `est`, not into `inputs`, but GCC's
  // -Wdangling-reference cannot tell which parameter a returned reference came
  // from and flags the temporary. Naming it removes the temporary instead of
  // suppressing the diagnostic, which keeps the warning live for a real case.
  const std::vector<PullContactInput> held = pinch(1.5);
  const PullEstimate& settled = RunTicks(est, held, 1000);
  ASSERT_TRUE(settled.valid);
  ASSERT_NEAR(settled.magnitude, 1.5, 1e-6);

  std::vector<PullContactInput> lost = pinch(1.5);
  lost[0].valid = false;
  lost[1].valid = false;
  double decayed = 0.0;
  for (int i = 0; i < gap_ticks; ++i) {
    decayed = est.Update(lost, kPlaneNormal, kDt).magnitude;
  }
  ASSERT_LT(decayed, 1.5);

  // First valid tick back. It must report the force that is actually there.
  const PullEstimate& back = est.Update(pinch(return_pull), kPlaneNormal, kDt);
  ASSERT_TRUE(back.valid);
  EXPECT_NEAR(back.force_raw.x(), return_pull, 1e-9);
  EXPECT_NEAR(back.magnitude, return_pull, 1e-6) << "gap=" << gap_ticks << " decayed=" << decayed;

  // And specifically: it is not the pre-gap value resurrected. (For the
  // same-raw case the two coincide by construction, so the check only bites
  // when the world moved — which is the failure that mattered.)
  if (return_pull != 1.5) {
    EXPECT_GT(std::abs(back.magnitude - 1.5), 1e-3);
  }
}

INSTANTIATE_TEST_SUITE_P(ShortAndLongGaps, PullRecovery,
                         ::testing::Combine(::testing::Values(2, 500),
                                            ::testing::Values(1.5, 0.25)));

TEST(PullForceEstimator, NonPositiveDtCollapsesTheEstimate) {
  // Policy pin (#234 AC-2): a clock that did not advance carries no
  // information about how far the estimate should have decayed, so an invalid
  // tick with dt <= 0 hard-zeroes instead of holding the last value — holding
  // would let a stalled caller republish a stale force forever.
  PullForceEstimator est;
  est.Init(PinchConfigs(), DefaultParams());
  const std::vector<PullContactInput> pulling = {
      MakeInput(Eigen::Vector3d(-0.75, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(-0.75, 0.0, -5.0), kOpposingNormal)};
  ASSERT_TRUE(RunTicks(est, pulling, 1000).valid);

  std::vector<PullContactInput> lost = pulling;
  lost[0].valid = false;
  lost[1].valid = false;
  const PullEstimate& zero_dt = est.Update(lost, kPlaneNormal, 0.0);
  EXPECT_FALSE(zero_dt.valid);
  EXPECT_EQ(zero_dt.magnitude, 0.0);
  EXPECT_EQ(zero_dt.force_inplane.norm(), 0.0);
}

// ── AC-4: slip_risk lives outside the valid gate (P-7) ──────────────────────
TEST(PullForceEstimator, SlipRiskSurvivesARequiredRoleDropout) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 2;
  // Thumb required; the opposing tip is the one sliding.
  const std::vector<PullContactConfig> configs = {MakeConfig(/*required=*/true), MakeConfig()};
  est.Init(configs, params);

  std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      // f_n = 5, |f_t| = 3.85 → rho = 1.1 >= threshold.
      MakeInput(Eigen::Vector3d(3.85, 0.0, -5.0), kOpposingNormal)};
  const PullEstimate& both = est.Update(inputs, kPlaneNormal, kDt);
  ASSERT_TRUE(both.valid);
  EXPECT_TRUE(both.slip_risk);

  // Thumb sensor drops. The pull vector is now invalid — but the other tip is
  // still measurably sliding, and reporting "no slip risk" there was the bug.
  inputs[0].valid = false;
  const PullEstimate& dropped = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_FALSE(dropped.valid);
  EXPECT_EQ(dropped.invalid_reason, rtc::grasp::PullInvalidReason::kRequiredContactMissing);
  EXPECT_EQ(dropped.valid_contact_count, 1);
  EXPECT_NEAR(dropped.max_friction_utilization, 1.1, 1e-6);
  EXPECT_TRUE(dropped.slip_risk);
}

TEST(PullForceEstimator, SlipRiskIsFalseWithNoActiveContact) {
  // The utilization max is vacuously zero with nothing active, so the flag
  // must stay false rather than inheriting the previous tick.
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  est.Init(std::vector<PullContactConfig>{MakeConfig()}, params);

  ASSERT_TRUE(est.Update(std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(3.85, 0.0, 5.0),
                                                                 kThumbNormal)},
                         kPlaneNormal, kDt)
                  .slip_risk);

  std::vector<PullContactInput> gone = {MakeInput(Eigen::Vector3d(3.85, 0.0, 5.0), kThumbNormal)};
  gone[0].valid = false;
  const PullEstimate& out = est.Update(gone, kPlaneNormal, kDt);
  EXPECT_EQ(out.valid_contact_count, 0);
  EXPECT_FALSE(out.slip_risk);
  EXPECT_EQ(out.max_friction_utilization, 0.0);
}

TEST(PullForceEstimator, SaturatedContactIsGatedOutOfTheSlipDiagnostic) {
  // A saturated sensor is gated out of the sum, so it contributes no
  // utilization either — its reading is a clipped lower bound, not a ratio.
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  PullContactConfig cfg = MakeConfig();
  cfg.force_saturation = 10.0;
  est.Init(std::vector<PullContactConfig>{cfg, cfg}, params);

  const std::vector<PullContactInput> inputs = {
      MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal),
      MakeInput(Eigen::Vector3d(10.0, 0.0, -5.0), kOpposingNormal)};  // clipped
  const PullEstimate& out = est.Update(inputs, kPlaneNormal, kDt);
  EXPECT_TRUE(out.any_saturated);
  EXPECT_EQ(out.contact_mask, 0b0000'0001U);
  EXPECT_FALSE(out.slip_risk);
  EXPECT_NEAR(out.max_friction_utilization, 0.0, 1e-9);
}

// ── AC-3: configure-time validation (P-6 + cross-review §3) ─────────────────
TEST(PullForceEstimator, InitRejectsMinValidContactsAboveTheContactCount) {
  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 5;  // 3-tip hand
  EXPECT_THROW(
      est.Init(std::vector<PullContactConfig>{MakeConfig(), MakeConfig(), MakeConfig()}, params),
      std::invalid_argument);

  params.min_valid_contacts = 3;  // exactly the count is fine
  EXPECT_NO_THROW(
      est.Init(std::vector<PullContactConfig>{MakeConfig(), MakeConfig(), MakeConfig()}, params));
}

TEST(PullForceEstimator, InitRejectsNonFiniteScalars) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  // Single-contact cases must lower min_valid_contacts, or the new count check
  // (#234 P-6) throws first and the test would pass without exercising the
  // finite check at all.
  PullEstimatorParams single = DefaultParams();
  single.min_valid_contacts = 1;

  {  // NaN passes `<= 0.0` silently — that is why these need explicit checks.
    PullForceEstimator est;
    PullEstimatorParams p = DefaultParams();
    p.decay_time_constant_s = nan;
    EXPECT_THROW(est.Init(PinchConfigs(), p), std::invalid_argument);
  }
  {
    PullForceEstimator est;
    PullEstimatorParams p = DefaultParams();
    p.slip_risk_threshold = inf;
    EXPECT_THROW(est.Init(PinchConfigs(), p), std::invalid_argument);
  }
  {
    PullForceEstimator est;
    PullEstimatorParams p = DefaultParams();
    p.alignment_error_rad = nan;
    EXPECT_THROW(est.Init(PinchConfigs(), p), std::invalid_argument);
  }
  {
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.friction_coeff = nan;
    EXPECT_THROW(est.Init(std::vector<PullContactConfig>{cfg}, single), std::invalid_argument);
  }
  {
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.force_saturation = nan;
    EXPECT_THROW(est.Init(std::vector<PullContactConfig>{cfg}, single), std::invalid_argument);
  }
  {
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.contact_on_threshold = nan;
    EXPECT_THROW(est.Init(std::vector<PullContactConfig>{cfg}, single), std::invalid_argument);
  }
}

TEST(PullForceEstimator, InitRejectsForceSignThatIsNotAConventionFlip) {
  // The header documents force_sign as a convention selector, not a gain: any
  // other value silently rescales the estimate while the units still read N.
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;  // single-contact configs below
  for (const double bad : {0.0, 2.0, -0.5, std::numeric_limits<double>::quiet_NaN()}) {
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.force_sign = bad;
    EXPECT_THROW(est.Init(std::vector<PullContactConfig>{cfg}, params), std::invalid_argument)
        << "force_sign=" << bad;
  }
  for (const double good : {1.0, -1.0}) {
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.force_sign = good;
    EXPECT_NO_THROW(est.Init(std::vector<PullContactConfig>{cfg}, params));
  }
}

// ── AC-7: optional touch thresholds (P-16) ──────────────────────────────────
TEST(PullForceEstimator, UnsetTouchThresholdsReproduceTheContactPairExactly) {
  // The containment contract `touch ⊇ contact` and the pinch-axis cold start
  // both depend on the touch gate reusing the contact thresholds when the
  // optional pair is omitted — so the default path must be bit-identical.
  PullContactConfig shared = MakeConfig();
  shared.contact_on_threshold = 1.0;
  shared.contact_off_threshold = 0.4;

  PullContactConfig explicit_same = shared;
  explicit_same.touch_on_threshold = 1.0;
  explicit_same.touch_off_threshold = 0.4;

  PullForceEstimator a;
  PullForceEstimator b;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  a.Init(std::vector<PullContactConfig>{shared}, params);
  b.Init(std::vector<PullContactConfig>{explicit_same}, params);

  // Sweep up through the band and back down so both edges of both gates fire.
  for (const double f : {0.0, 0.5, 0.9, 1.2, 3.0, 0.9, 0.5, 0.3, 0.0}) {
    const std::vector<PullContactInput> in = {
        MakeInput(Eigen::Vector3d(0.0, 0.0, f), kThumbNormal)};
    const PullEstimate& ra = a.Update(in, kPlaneNormal, kDt);
    const PullEstimate& rb = b.Update(in, kPlaneNormal, kDt);
    EXPECT_EQ(ra.contact_mask, rb.contact_mask) << "f=" << f;
    EXPECT_EQ(ra.touch_mask, rb.touch_mask) << "f=" << f;
    EXPECT_EQ(ra.valid, rb.valid) << "f=" << f;
    EXPECT_EQ(a.touch_active(0), b.touch_active(0)) << "f=" << f;
    EXPECT_DOUBLE_EQ(ra.magnitude, rb.magnitude) << "f=" << f;
  }
}

TEST(PullForceEstimator, SplitTouchThresholdDecouplesTheAxisGateFromTheSum) {
  // A tip with large shear reads as "touching" under the shared thresholds and
  // drags the pinch axis around while contributing nothing to the sum. Raising
  // only the touch pair keeps it out of the axis selection.
  PullContactConfig cfg = MakeConfig();
  cfg.contact_on_threshold = 1.0;
  cfg.contact_off_threshold = 0.4;
  cfg.touch_on_threshold = 4.0;
  cfg.touch_off_threshold = 2.0;

  PullForceEstimator est;
  PullEstimatorParams params = DefaultParams();
  params.min_valid_contacts = 1;
  est.Init(std::vector<PullContactConfig>{cfg}, params);

  // |f_obj| = 2 — above the contact gate (f_n = 2) but below touch ON (4).
  const PullEstimate& below = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(0.0, 0.0, 2.0), kThumbNormal)},
      kPlaneNormal, kDt);
  EXPECT_EQ(below.contact_mask, 0b0000'0001U);
  EXPECT_EQ(below.touch_mask, 0U) << "touch gate must use its own threshold";

  // |f_obj| = 5 — above touch ON.
  const PullEstimate& above = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(0.0, 0.0, 5.0), kThumbNormal)},
      kPlaneNormal, kDt);
  EXPECT_EQ(above.touch_mask, 0b0000'0001U);

  // Hysteresis: 3 is below ON (4) but above OFF (2) → stays latched.
  const PullEstimate& band = est.Update(
      std::vector<PullContactInput>{MakeInput(Eigen::Vector3d(0.0, 0.0, 3.0), kThumbNormal)},
      kPlaneNormal, kDt);
  EXPECT_EQ(band.touch_mask, 0b0000'0001U);
}

TEST(PullForceEstimator, InitRejectsAHalfSetTouchPair) {
  // min_valid_contacts lowered so the single-contact configs below fail on
  // the touch pair rather than on the contact-count check (#234 P-6).
  PullEstimatorParams single = DefaultParams();
  single.min_valid_contacts = 1;
  // Pairing a configured ON with an implicit zero OFF would latch the touch
  // gate on forever — a typo, not a policy.
  {
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.touch_on_threshold = 2.0;
    EXPECT_THROW(est.Init(std::vector<PullContactConfig>{cfg}, single), std::invalid_argument);
  }
  {
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.touch_off_threshold = 1.0;
    EXPECT_THROW(est.Init(std::vector<PullContactConfig>{cfg}, single), std::invalid_argument);
  }
  {  // on <= off is rejected the same way the contact pair is.
    PullForceEstimator est;
    PullContactConfig cfg = MakeConfig();
    cfg.touch_on_threshold = 1.0;
    cfg.touch_off_threshold = 2.0;
    EXPECT_THROW(est.Init(std::vector<PullContactConfig>{cfg}, single), std::invalid_argument);
  }
}

}  // namespace
