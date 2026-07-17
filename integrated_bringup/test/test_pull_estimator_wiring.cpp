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

#include <cmath>
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

}  // namespace
