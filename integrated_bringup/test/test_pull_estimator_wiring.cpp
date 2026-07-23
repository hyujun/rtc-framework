// Unit tests for the shared pull-estimator wiring helper (#167 P3):
// tip-link → fingertip-slot resolve, degenerate-config gating, pinch-geometry
// plane normal, and baseline arming on the grasp_detected rising edge.
//
// Configs are built through ApplyPullEstimatorBlock (the production YAML
// parser) so key names cannot drift from demo_shared.yaml.

#include "integrated_bringup/support/pull_estimator_wiring.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using integrated_bringup::ConfigurePullEstimatorWiring;
using integrated_bringup::DemoSharedConfig;
using integrated_bringup::PullEstimatorWiring;
using integrated_bringup::UpdatePullEstimator;

constexpr double kRateHz = 500.0;
constexpr double kDt = 1.0 / kRateHz;

// tree-model tip_links order for ur5e_p1a (_base.yaml) — slot indices 0..3.
const std::vector<std::string> kTipLinks = {"thumb_tip_link", "index_tip_link", "middle_tip_link",
                                            "ring_tip_link"};

// Minimal production-shaped pull_estimator block. The per-contact gate normal
// is derived per tick from the (signed) plane normal: thumb gets -n, the
// opposing fingers +n, so an opposing squeeze cancels and f_n is positive for
// both. With plane_normal [0,0,1] this reproduces the old thumb-below (-z),
// index/middle-above (+z) convention.
DemoSharedConfig MakeSharedConfig(const std::string& extra_yaml = "") {
  const std::string yaml = R"(
demo_shared:
  pull_estimator:
    enabled: true
    min_valid_contacts: 2
    plane_normal_source: "fixed"
    plane_normal: [0.0, 0.0, 1.0]
    required_roles: ["thumb"]
    tips:
      tip_names: ["thumb", "index", "middle"]
      thumb:
        link: "thumb_tip_link"
        force_sign: 1.0
      index:
        link: "index_tip_link"
        force_sign: 1.0
      middle:
        link: "middle_tip_link"
        force_sign: 1.0
)" + extra_yaml;
  DemoSharedConfig cfg;
  integrated_bringup::ApplyDemoSharedConfig(YAML::Load(yaml)["demo_shared"], cfg);
  return cfg;
}

// Stage a 3-contact opposing pinch: forces are finger-on-object (force_sign
// +1 in MakeSharedConfig), squeeze along ±z cancels, `pull` rides in-plane.
void StagePinchInputs(PullEstimatorWiring& w, const Eigen::Vector3d& pull) {
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    auto& in = w.inputs[ki];
    in.valid = true;
    in.rotation = Eigen::Matrix3d::Identity();
    const double fz = (k == w.thumb_contact) ? 2.0 : -1.0;  // opposing squeeze
    in.force = Eigen::Vector3d(-pull.x() / 3.0, -pull.y() / 3.0, fz);
    w.positions[ki] = Eigen::Vector3d::Zero();
    w.position_valid[ki] = true;
  }
  // Pinch geometry positions: thumb at origin, index/middle above.
  if (w.thumb_contact >= 0) {
    int opposing = 0;
    for (int k = 0; k < w.num_contacts; ++k) {
      if (k == w.thumb_contact) {
        continue;
      }
      const double y = (opposing == 0) ? 0.01 : -0.01;
      w.positions[static_cast<std::size_t>(k)] = Eigen::Vector3d(0.0, y, 0.02);
      ++opposing;
    }
  }
}

const rtc::grasp::PullEstimate& SettleTicks(PullEstimatorWiring& w, bool grasp_detected,
                                            int ticks) {
  const rtc::grasp::PullEstimate* est = nullptr;
  for (int i = 0; i < ticks; ++i) {
    est = &UpdatePullEstimator(w, grasp_detected, kDt);
  }
  return *est;
}

// ── E-STOP tick (#234 P-1) ─────────────────────────────────────────────────

TEST(PullEstimatorWiring, EstopTickInvalidatesAndDecaysTheEstimate) {
  const DemoSharedConfig cfg = MakeSharedConfig();
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StagePinchInputs(w, Eigen::Vector3d(3.0, 0.0, 0.0));
  const rtc::grasp::PullEstimate& live = SettleTicks(w, /*grasp_detected=*/true, 200);
  ASSERT_TRUE(live.valid);
  const double live_magnitude = live.magnitude;
  ASSERT_GT(live_magnitude, 0.1);

  // Inputs stay staged from the last live tick: the E-STOP tick must ignore
  // them, not re-use them. This is the failure mode the fix exists for — a
  // frozen valid=1 estimate published under the E-STOP tick's stamp.
  rtc::grasp::PullEstimateData out{};
  integrated_bringup::StageEstopPullTick(w, kDt, out);

  EXPECT_FALSE(out.valid);
  EXPECT_FALSE(out.slip_risk);
  EXPECT_EQ(out.valid_contact_count, 0);
  EXPECT_EQ(w.opposing_mask, 0);
  // Bounded decay, not a hard zero: same policy as transient contact loss.
  EXPECT_LT(static_cast<double>(out.magnitude), live_magnitude);
  EXPECT_GT(static_cast<double>(out.magnitude), 0.0);

  // Held E-STOP keeps decaying toward zero.
  for (int i = 0; i < 500; ++i) {
    integrated_bringup::StageEstopPullTick(w, kDt, out);
  }
  EXPECT_FALSE(out.valid);
  EXPECT_LT(static_cast<double>(out.magnitude), 1e-3);
}

TEST(PullEstimatorWiring, EstopTickIsANoOpOnADisabledWiring) {
  PullEstimatorWiring w;  // never configured → disabled
  ASSERT_FALSE(w.enabled());
  rtc::grasp::PullEstimateData out{};
  out.magnitude = 7.0F;
  out.valid = true;
  integrated_bringup::StageEstopPullTick(w, kDt, out);
  // A disabled wiring publishes nothing of its own; the caller's field is left
  // exactly as it found it rather than being zeroed by a channel that is off.
  EXPECT_TRUE(out.valid);
  EXPECT_FLOAT_EQ(out.magnitude, 7.0F);
}

TEST(PullEstimatorWiring, EstopTickClearsTheBaselineArmingEdge) {
  // A grasp that was detected before the E-STOP must not arm a baseline
  // against the stopped tick's forces: the edge detector is reset, so the
  // rising edge is re-required after release.
  const DemoSharedConfig cfg = MakeSharedConfig(R"(
    use_baseline_subtraction: true
)");
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());
  ASSERT_TRUE(w.use_baseline);

  StagePinchInputs(w, Eigen::Vector3d(2.0, 0.0, 0.0));
  SettleTicks(w, /*grasp_detected=*/true, 10);
  ASSERT_TRUE(w.prev_grasp_detected);

  rtc::grasp::PullEstimateData out{};
  integrated_bringup::StageEstopPullTick(w, kDt, out);
  EXPECT_FALSE(w.prev_grasp_detected);
}

TEST(PullEstimatorWiring, ConfigureResolvesSlotsAndThumbRole) {
  const DemoSharedConfig cfg = MakeSharedConfig();
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);

  ASSERT_TRUE(w.enabled());
  EXPECT_EQ(w.num_contacts, 3);
  EXPECT_EQ(w.slot[0], 0);  // thumb → tip_links[0]
  EXPECT_EQ(w.slot[1], 1);
  EXPECT_EQ(w.slot[2], 2);
  EXPECT_EQ(w.thumb_contact, 0);
  EXPECT_FALSE(w.use_baseline);
  EXPECT_EQ(w.normal_source, integrated_bringup::PullPlaneNormalSource::kFixed);
}

TEST(PullEstimatorWiring, ConfigureThrowsOnUnknownTipLink) {
  const DemoSharedConfig cfg = MakeSharedConfig();
  PullEstimatorWiring w;
  const std::vector<std::string> wrong_links = {"thumb_tip_link", "index_tip_link",
                                                "pinky_tip_link"};
  EXPECT_THROW(ConfigurePullEstimatorWiring(cfg, kRateHz, wrong_links, w), std::runtime_error);
}

TEST(PullEstimatorWiring, ConfigureStaysDisabledWithoutLinksOrBlock) {
  const DemoSharedConfig cfg = MakeSharedConfig();
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, {}, w);  // no FK-backed slots
  EXPECT_FALSE(w.enabled());

  DemoSharedConfig no_block;  // pull_estimator block absent
  ConfigurePullEstimatorWiring(no_block, kRateHz, kTipLinks, w);
  EXPECT_FALSE(w.enabled());

  DemoSharedConfig disabled = MakeSharedConfig();
  disabled.pull_estimator_enabled = false;
  ConfigurePullEstimatorWiring(disabled, kRateHz, kTipLinks, w);
  EXPECT_FALSE(w.enabled());
}

TEST(PullEstimatorWiring, ConfigureThrowsWithoutThumbRole) {
  // The per-contact gate normal is derived from the thumb-opposition axis, so a
  // 'thumb' role is mandatory for BOTH plane_normal sources.
  for (const auto source : {integrated_bringup::PullPlaneNormalSource::kPinchGeometry,
                            integrated_bringup::PullPlaneNormalSource::kFixed}) {
    DemoSharedConfig cfg = MakeSharedConfig();
    cfg.pull_plane_normal_source = source;
    cfg.pull_tip_roles[0] = "palm";  // no "thumb" role anywhere
    PullEstimatorWiring w;
    EXPECT_THROW(ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w), std::runtime_error);
  }
}

TEST(PullEstimatorWiring, DerivesSignedPinchNormalPerContact) {
  const DemoSharedConfig cfg = MakeSharedConfig();  // fixed plane_normal [0,0,1]
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StagePinchInputs(w, Eigen::Vector3d(1.0, 0.0, 0.0));
  (void)UpdatePullEstimator(w, /*grasp_detected=*/true, kDt);

  // Thumb normal = -plane_normal; opposing fingers = +plane_normal. This is the
  // FK-derived gate normal, independent of the (identity here) fingertip
  // rotation — the fix that lets it track the grasp axis on hemispherical tips.
  EXPECT_NEAR((w.inputs[0].contact_normal - Eigen::Vector3d(0.0, 0.0, -1.0)).norm(), 0.0, 1e-9);
  EXPECT_NEAR((w.inputs[1].contact_normal - Eigen::Vector3d(0.0, 0.0, 1.0)).norm(), 0.0, 1e-9);
  EXPECT_NEAR((w.inputs[2].contact_normal - Eigen::Vector3d(0.0, 0.0, 1.0)).norm(), 0.0, 1e-9);
}

TEST(PullEstimatorWiring, FixedNormalInPlanePull) {
  const DemoSharedConfig cfg = MakeSharedConfig();
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StagePinchInputs(w, Eigen::Vector3d(1.5, 0.0, 0.0));
  const rtc::grasp::PullEstimate& est = SettleTicks(w, /*grasp_detected=*/true, 1000);
  ASSERT_TRUE(est.valid);
  EXPECT_NEAR(est.force_filtered.x(), 1.5, 1e-3);
  EXPECT_NEAR(est.force_filtered.y(), 0.0, 1e-6);
  EXPECT_NEAR(est.force_filtered.z(), 0.0, 1e-6);  // squeeze + P∥ cancel z
}

TEST(PullEstimatorWiring, PinchGeometryNormalMatchesFixed) {
  DemoSharedConfig cfg = MakeSharedConfig();
  cfg.pull_plane_normal_source = integrated_bringup::PullPlaneNormalSource::kPinchGeometry;
  cfg.pull_plane_normal = Eigen::Vector3d::UnitX();  // must be ignored
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  // StagePinchInputs geometry: index/middle midpoint (0,0,0.02) − thumb (0,0,0)
  // → normal +z, identical to the fixed-normal test.
  StagePinchInputs(w, Eigen::Vector3d(0.0, 2.0, 0.0));
  const rtc::grasp::PullEstimate& est = SettleTicks(w, true, 1000);
  ASSERT_TRUE(est.valid);
  EXPECT_NEAR(est.force_filtered.y(), 2.0, 1e-3);
  EXPECT_NEAR(est.force_filtered.z(), 0.0, 1e-6);
}

TEST(PullEstimatorWiring, PinchGeometryInvalidWithoutThumbPosition) {
  DemoSharedConfig cfg = MakeSharedConfig();
  cfg.pull_plane_normal_source = integrated_bringup::PullPlaneNormalSource::kPinchGeometry;
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StagePinchInputs(w, Eigen::Vector3d(1.0, 0.0, 0.0));
  w.position_valid[static_cast<std::size_t>(w.thumb_contact)] = false;  // thumb FK dropout
  const rtc::grasp::PullEstimate& est = SettleTicks(w, true, 10);
  EXPECT_FALSE(est.valid);  // degenerate normal → invalid tick + decay
}

TEST(PullEstimatorWiring, BaselineArmsOnGraspRisingEdge) {
  DemoSharedConfig cfg = MakeSharedConfig();
  cfg.pull_use_baseline_subtraction = true;
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  // Constant in-plane bias present before the grasp is established.
  StagePinchInputs(w, Eigen::Vector3d(1.0, 0.0, 0.0));
  (void)SettleTicks(w, /*grasp_detected=*/false, 500);

  // Rising edge arms the snapshot: the bias is captured and subtracted.
  const rtc::grasp::PullEstimate& zeroed = SettleTicks(w, /*grasp_detected=*/true, 1000);
  ASSERT_TRUE(zeroed.valid);
  EXPECT_TRUE(zeroed.baseline_applied);
  EXPECT_NEAR(zeroed.force_filtered.norm(), 0.0, 1e-3);

  // Extra pull on top of the captured bias is what remains.
  StagePinchInputs(w, Eigen::Vector3d(2.0, 0.0, 0.0));
  const rtc::grasp::PullEstimate& extra = SettleTicks(w, true, 1000);
  ASSERT_TRUE(extra.valid);
  EXPECT_NEAR(extra.force_filtered.x(), 1.0, 1e-3);
}

// ── Observed grasp shape ────────────────────────────────────────────────────
// The fixture above places index/middle symmetrically about the thumb, so a
// touch-selected axis and an all-tips centroid coincide — it cannot tell the
// two apart. These use four *asymmetric* tips, chosen so that no candidate
// opposing set shares an axis with another (in particular the three-tip
// centroid is not any single tip's position, which would let the four-finger
// case pass even with the centroid arithmetic wrong).

// Enough ticks for the touch hysteresis to latch and the axis to converge; the
// filtered estimate is not settled to steady state at this count.
constexpr int kShapeSettleTicks = 50;
// ~1 s at 500 Hz — long enough for the 5 Hz output filter to settle.
constexpr int kFilterSettleTicks = 500;

const Eigen::Vector3d kThumbPos(0.0, 0.0, 0.0);
const Eigen::Vector3d kIndexPos(0.0, 0.03, 0.02);
const Eigen::Vector3d kMiddlePos(0.0, 0.0, 0.025);
const Eigen::Vector3d kRingPos(0.0, -0.04, 0.015);

// Contact-index bitmask over {thumb=0, index=1, middle=2, ring=3}, matching
// PullEstimatorWiring::opposing_mask (which never sets the thumb's bit).
constexpr std::uint8_t kThumb = 1U << 0U;
constexpr std::uint8_t kIndex = 1U << 1U;
constexpr std::uint8_t kMiddle = 1U << 2U;
constexpr std::uint8_t kRing = 1U << 3U;

DemoSharedConfig MakeFourTipPinchConfig() {
  const std::string yaml = R"(
demo_shared:
  pull_estimator:
    enabled: true
    min_valid_contacts: 2
    plane_normal_source: "pinch_geometry"
    required_roles: ["thumb"]
    tips:
      tip_names: ["thumb", "index", "middle", "ring"]
      thumb:
        link: "thumb_tip_link"
        force_sign: 1.0
      index:
        link: "index_tip_link"
        force_sign: 1.0
      middle:
        link: "middle_tip_link"
        force_sign: 1.0
      ring:
        link: "ring_tip_link"
        force_sign: 1.0
)";
  DemoSharedConfig cfg;
  integrated_bringup::ApplyDemoSharedConfig(YAML::Load(yaml)["demo_shared"], cfg);
  return cfg;
}

// Un-normalized pinch axis (opposing centroid − thumb) for a touching set.
Eigen::Vector3d ExpectedAxis(std::uint8_t touching) {
  const std::array<Eigen::Vector3d, 4> pos = {kThumbPos, kIndexPos, kMiddlePos, kRingPos};
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int n = 0;
  for (int k = 1; k < 4; ++k) {
    if ((touching & (1U << static_cast<unsigned>(k))) != 0U) {
      sum += pos[static_cast<std::size_t>(k)];
      ++n;
    }
  }
  return sum / static_cast<double>(n) - kThumbPos;
}

// -P∥(total) — what the estimator must report pre-filter for a given plane.
Eigen::Vector3d ExpectedRaw(const Eigen::Vector3d& total, const Eigen::Vector3d& axis) {
  const Eigen::Vector3d n = axis.normalized();
  return -(total - n * n.dot(total));
}

// Stage the four tips with FK always valid, but a squeeze force only on the
// contacts named in `touching`. A non-touching tip keeps a valid *sensor*
// reading of zero — exactly the case that used to drag the axis: FK-valid but
// not gripping. The squeeze deliberately does not cancel (thumb +3, each
// opposing tip -1.5), so the residual is observable and the tests can tell
// which plane it was projected onto. `pull` is shared over the touching tips so
// their sum reproduces the external load exactly.
void StageFourTipInputs(PullEstimatorWiring& w, std::uint8_t touching,
                        const Eigen::Vector3d& pull = Eigen::Vector3d::Zero()) {
  const std::array<Eigen::Vector3d, 4> pos = {kThumbPos, kIndexPos, kMiddlePos, kRingPos};
  int n_touching = 0;
  for (int k = 0; k < 4; ++k) {
    if ((touching & (1U << static_cast<unsigned>(k))) != 0U) {
      ++n_touching;
    }
  }
  for (int k = 0; k < w.num_contacts; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    const bool touches = (touching & (1U << static_cast<unsigned>(k))) != 0U;
    auto& in = w.inputs[ki];
    in.valid = true;  // sensor is healthy either way; only the force differs
    in.rotation = Eigen::Matrix3d::Identity();
    const double fz = (k == w.thumb_contact) ? 3.0 : -1.5;
    const Eigen::Vector3d share =
        (n_touching > 0) ? Eigen::Vector3d(-pull / static_cast<double>(n_touching)) : pull;
    in.force = touches ? Eigen::Vector3d(share.x(), share.y(), fz) : Eigen::Vector3d::Zero();
    w.positions[ki] = pos[ki];
    w.position_valid[ki] = true;
  }
}

TEST(PullEstimatorWiring, PinchAxisFollowsTheTouchingTipsOnly) {
  struct Case {
    const char* name;
    std::uint8_t touching;
    std::uint8_t expected_mask;
    int expected_contacts;
  };

  const std::array<Case, 4> cases = {
      Case{"thumb+index", kThumb | kIndex, kIndex, 2},
      Case{"thumb+middle", kThumb | kMiddle, kMiddle, 2},
      Case{"thumb+index+middle", kThumb | kIndex | kMiddle, kIndex | kMiddle, 3},
      Case{"thumb+index+middle+ring", kThumb | kIndex | kMiddle | kRing, kIndex | kMiddle | kRing,
           4},
  };

  // The fixture must be able to tell the four axes apart, or these assertions
  // would also pass on a wrong opposing set.
  for (std::size_t a = 0; a < cases.size(); ++a) {
    for (std::size_t b = a + 1; b < cases.size(); ++b) {
      ASSERT_GT((ExpectedAxis(cases[a].touching).normalized() -
                 ExpectedAxis(cases[b].touching).normalized())
                    .norm(),
                0.1)
          << cases[a].name << " vs " << cases[b].name;
    }
  }

  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    PullEstimatorWiring w;
    ConfigurePullEstimatorWiring(MakeFourTipPinchConfig(), kRateHz, kTipLinks, w);
    ASSERT_TRUE(w.enabled());

    StageFourTipInputs(w, c.touching);
    const rtc::grasp::PullEstimate& est =
        SettleTicks(w, /*grasp_detected=*/true, kShapeSettleTicks);
    ASSERT_TRUE(est.valid);

    // The axis is observable through the per-contact gate normal: opposing tips
    // get +n̂, the thumb -n̂.
    const Eigen::Vector3d expected = ExpectedAxis(c.touching).normalized();
    EXPECT_EQ(w.opposing_mask, c.expected_mask);
    EXPECT_NEAR(
        (w.inputs[static_cast<std::size_t>(w.thumb_contact)].contact_normal + expected).norm(), 0.0,
        1e-9);
    for (int k = 1; k < 4; ++k) {
      if ((c.expected_mask & (1U << static_cast<unsigned>(k))) != 0U) {
        EXPECT_NEAR((w.inputs[static_cast<std::size_t>(k)].contact_normal - expected).norm(), 0.0,
                    1e-9)
            << "contact " << k;
      }
    }
    // Only the touching tips are summed, so the count matches the shape.
    EXPECT_EQ(est.valid_contact_count, c.expected_contacts);
  }
}

TEST(PullEstimatorWiring, IdleTipDoesNotSkewTheAxisOrTheEstimate) {
  // Regression: the axis used to be the centroid of every FK-valid non-thumb
  // tip, so a middle/ring finger merely hovering nearby tilted the pinch plane
  // — and with it the projection that produces the published force.
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(MakeFourTipPinchConfig(), kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StageFourTipInputs(w, kThumb | kIndex);
  const rtc::grasp::PullEstimate& est = SettleTicks(w, true, kShapeSettleTicks);
  ASSERT_TRUE(est.valid);

  const Eigen::Vector3d pinch_axis = ExpectedAxis(kIndex);
  const Eigen::Vector3d all_tips_axis = ExpectedAxis(kIndex | kMiddle | kRing);
  ASSERT_GT((all_tips_axis.normalized() - pinch_axis.normalized()).norm(), 0.1);
  EXPECT_NEAR((w.inputs[1].contact_normal - pinch_axis.normalized()).norm(), 0.0, 1e-9);

  // thumb +3 z, index -1.5 z (the idle tips contribute nothing).
  const Eigen::Vector3d total(0.0, 0.0, 1.5);
  const Eigen::Vector3d want = ExpectedRaw(total, pinch_axis);
  const Eigen::Vector3d skewed = ExpectedRaw(total, all_tips_axis);
  ASSERT_GT((want - skewed).norm(),
            0.1);  // the estimate can tell them apart too
  EXPECT_NEAR((est.force_raw - want).norm(), 0.0, 1e-9);
}

TEST(PullEstimatorWiring, NoAxisIsInventedBeforeTheFirstTouch) {
  // Selection is axis-independent (touch hysteresis on |f_obj|), so there is no
  // circular cold start to bootstrap around: the first tick simply has no
  // opposing set and reports invalid, and the second already has the real axis.
  // No sample is ever published against a guessed plane.
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(MakeFourTipPinchConfig(), kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StageFourTipInputs(w, kThumb | kIndex);
  const rtc::grasp::PullEstimate& first = UpdatePullEstimator(w, true, kDt);
  EXPECT_EQ(w.opposing_mask, 0);
  EXPECT_FALSE(first.valid);  // no axis ⇒ no estimate, rather than a provisional one

  const rtc::grasp::PullEstimate& second = UpdatePullEstimator(w, true, kDt);
  EXPECT_EQ(w.opposing_mask, kIndex);
  EXPECT_TRUE(second.valid);
  EXPECT_NEAR((w.inputs[1].contact_normal - ExpectedAxis(kIndex).normalized()).norm(), 0.0, 1e-9);
}

TEST(PullEstimatorWiring, ShapeChangeRetargetsTheAxis) {
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(MakeFourTipPinchConfig(), kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StageFourTipInputs(w, kThumb | kIndex);
  (void)SettleTicks(w, true, kShapeSettleTicks);
  EXPECT_EQ(w.opposing_mask, kIndex);

  // Regrip onto the middle finger: the index lets go, middle takes over.
  StageFourTipInputs(w, kThumb | kMiddle);
  const rtc::grasp::PullEstimate& est = SettleTicks(w, true, kShapeSettleTicks);
  ASSERT_TRUE(est.valid);
  EXPECT_EQ(w.opposing_mask, kMiddle);
  EXPECT_NEAR((w.inputs[2].contact_normal - ExpectedAxis(kMiddle).normalized()).norm(), 0.0, 1e-9);
}

TEST(PullEstimatorWiring, TipParkedInTheHysteresisBandDoesNotChatter) {
  // Regression: selecting the opposing set from contact_active made the axis a
  // function of the latch state and the latch a function of the axis. The ON
  // and OFF tests then saw two different axes, so a marginal tip could flap in
  // and out of the mask every tick and step the whole in-plane basis with it.
  // Touch selection is axis-independent, so a tip parked between the two
  // thresholds is decided once by its own hysteresis and then holds.
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(MakeFourTipPinchConfig(), kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  // index grips firmly; middle sits at |f| = 0.3 N — above contact_off (0.2)
  // but below contact_on (0.5), i.e. inside the band and never latched.
  StageFourTipInputs(w, kThumb | kIndex | kMiddle);
  w.inputs[2].force = Eigen::Vector3d(0.0, 0.0, -0.3);
  (void)SettleTicks(w, true, kShapeSettleTicks);
  EXPECT_EQ(w.opposing_mask, kIndex) << "a tip below the ON threshold must not join";

  const std::uint8_t settled_mask = w.opposing_mask;
  for (int i = 0; i < 200; ++i) {
    (void)UpdatePullEstimator(w, true, kDt);
    ASSERT_EQ(w.opposing_mask, settled_mask) << "mask changed on tick " << i;
  }
}

TEST(PullEstimatorWiring, BaselineArmsOnTheEdgeAndFollowsTheAxis) {
  // The baseline stores the *unprojected* reference wrench, so a regrip that
  // rotates the pinch plane carries the subtraction with it. A pre-projected
  // snapshot would leave a constant residual in the new plane, and re-arming on
  // the shape change instead would capture whatever load happened to be present
  // at that moment.
  DemoSharedConfig cfg = MakeFourTipPinchConfig();
  cfg.pull_use_baseline_subtraction = true;
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  // Tick 1 has no axis yet (nothing latched) so nothing is captured; tick 2 is
  // the first valid one and takes the snapshot.
  StageFourTipInputs(w, kThumb | kIndex);
  const rtc::grasp::PullEstimate& armed = SettleTicks(w, /*grasp_detected=*/true, 2);
  ASSERT_TRUE(armed.valid);
  EXPECT_TRUE(armed.baseline_applied);
  EXPECT_NEAR(armed.force_raw.norm(), 0.0, 1e-12);

  // Regrip to a plane ~34 deg away. The staged wrench sum is identical, so any
  // residual here is purely the baseline failing to follow the plane.
  StageFourTipInputs(w, kThumb | kMiddle);
  const rtc::grasp::PullEstimate& after = SettleTicks(w, true, kShapeSettleTicks);
  ASSERT_TRUE(after.valid);
  EXPECT_EQ(w.opposing_mask, kMiddle);
  EXPECT_NEAR(after.force_raw.norm(), 0.0, 1e-12);

  // The fixture discriminates: a snapshot captured pre-projected on the old
  // plane would leave this much residual on the new one.
  const Eigen::Vector3d total(0.0, 0.0, 1.5);
  const Eigen::Vector3d stale_residual =
      ExpectedRaw(total, ExpectedAxis(kMiddle)) - ExpectedRaw(total, ExpectedAxis(kIndex));
  ASSERT_GT(stale_residual.norm(), 0.1);
}

TEST(PullEstimatorWiring, PullIsRecoveredUnderAChangingContactSet) {
  // The gate normal is only half the story — the same axis drives the in-plane
  // projection that produces the published number.
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(MakeFourTipPinchConfig(), kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  // A pull along +x is in-plane for every candidate axis here (they all lie in
  // the y-z plane), so the same load must be recovered whatever the shape.
  const Eigen::Vector3d pull(1.0, 0.0, 0.0);

  StageFourTipInputs(w, kThumb | kIndex, pull);
  const rtc::grasp::PullEstimate& pinch = SettleTicks(w, true, kFilterSettleTicks);
  ASSERT_TRUE(pinch.valid);
  EXPECT_EQ(w.opposing_mask, kIndex);
  EXPECT_NEAR(pinch.force_filtered.x(), pull.x(), 1e-3);

  StageFourTipInputs(w, kThumb | kIndex | kMiddle | kRing, pull);
  const rtc::grasp::PullEstimate& four = SettleTicks(w, true, kFilterSettleTicks);
  ASSERT_TRUE(four.valid);
  EXPECT_EQ(w.opposing_mask, kIndex | kMiddle | kRing);
  EXPECT_NEAR(four.force_filtered.x(), pull.x(), 1e-3);
}

TEST(PullEstimatorWiring, ResetRtStateDropsTheGraspEdgeAndTheContactSet) {
  DemoSharedConfig cfg = MakeFourTipPinchConfig();
  cfg.pull_use_baseline_subtraction = true;
  PullEstimatorWiring w;
  ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w);
  ASSERT_TRUE(w.enabled());

  StageFourTipInputs(w, kThumb | kIndex);
  const rtc::grasp::PullEstimate& settled = SettleTicks(w, true, kShapeSettleTicks);
  ASSERT_TRUE(settled.valid);
  ASSERT_EQ(w.opposing_mask, kIndex);
  ASSERT_TRUE(w.prev_grasp_detected);
  ASSERT_TRUE(settled.baseline_applied);

  integrated_bringup::ResetPullEstimatorRtState(w);
  EXPECT_FALSE(w.prev_grasp_detected);
  EXPECT_EQ(w.opposing_mask, 0);
  EXPECT_FALSE(w.estimator->estimate().baseline_applied);

  // Re-activation: grasp_detected is high from the first tick again, and that
  // must read as a fresh rising edge rather than being swallowed by the state
  // left over from before the gap.
  StageFourTipInputs(w, kThumb | kIndex);
  const rtc::grasp::PullEstimate& first = UpdatePullEstimator(w, true, kDt);
  EXPECT_EQ(w.opposing_mask, 0) << "touch hysteresis must start from cold";
  EXPECT_FALSE(first.valid);
  const rtc::grasp::PullEstimate& second = UpdatePullEstimator(w, true, kDt);
  EXPECT_TRUE(second.valid);
  EXPECT_TRUE(second.baseline_applied) << "the edge re-armed after the reset";
}

TEST(PullEstimatorWiring, ConfigureThrowsWithoutAnOpposingTipUnderPinchGeometry) {
  // A thumb-only pinch_geometry config can never form a pair: the opposing set
  // is empty forever and every tick decays to invalid. Fail at configure.
  const std::string yaml = R"(
demo_shared:
  pull_estimator:
    enabled: true
    plane_normal_source: "pinch_geometry"
    tips:
      tip_names: ["thumb"]
      thumb:
        link: "thumb_tip_link"
)";
  DemoSharedConfig cfg;
  integrated_bringup::ApplyDemoSharedConfig(YAML::Load(yaml)["demo_shared"], cfg);
  PullEstimatorWiring w;
  EXPECT_THROW(ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w), std::runtime_error);
  // A throw must leave the wiring disabled — "enabled with zero contacts" would
  // drive Update() with an empty input span on every tick.
  EXPECT_FALSE(w.enabled());
}

TEST(PullEstimatorWiring, ConfigureAllowsASingleTipUnderAFixedNormal) {
  // The fixed normal never consults the opposing set, so the opposing-tip
  // requirement does not apply to it.
  const std::string yaml = R"(
demo_shared:
  pull_estimator:
    enabled: true
    min_valid_contacts: 1
    plane_normal_source: "fixed"
    plane_normal: [0.0, 0.0, 1.0]
    tips:
      tip_names: ["thumb"]
      thumb:
        link: "thumb_tip_link"
)";
  DemoSharedConfig cfg;
  integrated_bringup::ApplyDemoSharedConfig(YAML::Load(yaml)["demo_shared"], cfg);
  PullEstimatorWiring w;
  ASSERT_NO_THROW(ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w));
  EXPECT_TRUE(w.enabled());
  EXPECT_EQ(w.num_contacts, 1);
}

TEST(PullEstimatorWiring, ConfigureThrowsOnRepeatedRole) {
  // Roles key the per-tip config block and pick the thumb, so a repeat resolves
  // both entries to the same link and slot: the tip is double-counted in the
  // sum, and a repeated "thumb" leaves the pinch axis identically zero (the
  // last match wins as the thumb, its twin becomes the opposing tip).
  const std::string yaml = R"(
demo_shared:
  pull_estimator:
    enabled: true
    plane_normal_source: "pinch_geometry"
    tips:
      tip_names: ["thumb", "thumb"]
      thumb:
        link: "thumb_tip_link"
)";
  DemoSharedConfig cfg;
  integrated_bringup::ApplyDemoSharedConfig(YAML::Load(yaml)["demo_shared"], cfg);
  PullEstimatorWiring w;
  EXPECT_THROW(ConfigurePullEstimatorWiring(cfg, kRateHz, kTipLinks, w), std::runtime_error);
  EXPECT_FALSE(w.enabled());
}

}  // namespace
