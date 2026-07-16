// ─────────────────────────────────────────────────────────────────────────────
// Unit tests for integrated_bringup::ApplyDemoSharedConfig and BuildGraspController
// (demo_shared_config.hpp / demo_shared_config.cpp)
// ─────────────────────────────────────────────────────────────────────────────
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

using integrated_bringup::ApplyDemoSharedConfig;
using integrated_bringup::BuildGraspController;
using integrated_bringup::DemoSharedConfig;
using integrated_bringup::VirtualTcpMode;

// ═══════════════════════════════════════════════════════════════════════════
// Defaults / no-op
// ═══════════════════════════════════════════════════════════════════════════

TEST(DemoSharedConfigTest, DefaultConstructedValues) {
  DemoSharedConfig cfg;
  EXPECT_EQ(cfg.vtcp.mode, VirtualTcpMode::kDisabled);
  EXPECT_FLOAT_EQ(cfg.grasp_contact_threshold, 0.5f);
  EXPECT_FLOAT_EQ(cfg.grasp_force_threshold, 1.0f);
  EXPECT_EQ(cfg.grasp_min_fingertips, 2);
  EXPECT_EQ(cfg.grasp_controller_type, "contact_stop");
  EXPECT_FALSE(cfg.has_force_pi_block);
}

TEST(DemoSharedConfigTest, NullNodeIsNoOp) {
  DemoSharedConfig cfg;
  cfg.grasp_contact_threshold = 0.75f;
  cfg.grasp_controller_type = "force_pi";

  YAML::Node empty;  // invalid / null
  ApplyDemoSharedConfig(empty, cfg);

  EXPECT_FLOAT_EQ(cfg.grasp_contact_threshold, 0.75f);
  EXPECT_EQ(cfg.grasp_controller_type, "force_pi");
}

TEST(DemoSharedConfigTest, PartialOverridePreservesOtherFields) {
  DemoSharedConfig cfg;
  cfg.grasp_contact_threshold = 0.9f;  // preserved
  cfg.grasp_force_threshold = 1.0f;
  cfg.grasp_min_fingertips = 2;

  YAML::Node node = YAML::Load("grasp_force_threshold: 3.5\n");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_FLOAT_EQ(cfg.grasp_force_threshold, 3.5f);
  EXPECT_FLOAT_EQ(cfg.grasp_contact_threshold, 0.9f);  // untouched
  EXPECT_EQ(cfg.grasp_min_fingertips, 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// Virtual TCP parsing
// ═══════════════════════════════════════════════════════════════════════════

TEST(DemoSharedConfigTest, VirtualTcpModeStrings) {
  struct Case {
    const char* str;
    VirtualTcpMode expected;
  };

  const Case cases[] = {
      {"centroid", VirtualTcpMode::kCentroid}, {"weighted", VirtualTcpMode::kWeighted},
      {"constant", VirtualTcpMode::kConstant}, {"disabled", VirtualTcpMode::kDisabled},
      {"bogus", VirtualTcpMode::kDisabled},  // unknown → disabled
  };
  for (const auto& c : cases) {
    DemoSharedConfig cfg;
    YAML::Node node;
    node["virtual_tcp_mode"] = c.str;
    ApplyDemoSharedConfig(node, cfg);
    EXPECT_EQ(cfg.vtcp.mode, c.expected) << "input=" << c.str;
  }
}

TEST(DemoSharedConfigTest, VirtualTcpOffsetAndOrientation) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
virtual_tcp_mode: constant
virtual_tcp_offset: [0.1, -0.2, 0.3]
virtual_tcp_orientation: [1.5708, 0.0, -1.5708]
)YAML");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_EQ(cfg.vtcp.mode, VirtualTcpMode::kConstant);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[0], 0.1);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[1], -0.2);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[2], 0.3);
  EXPECT_DOUBLE_EQ(cfg.vtcp.orientation[0], 1.5708);
  EXPECT_DOUBLE_EQ(cfg.vtcp.orientation[1], 0.0);
  EXPECT_DOUBLE_EQ(cfg.vtcp.orientation[2], -1.5708);
}

TEST(DemoSharedConfigTest, VirtualTcpOffsetShortSequenceLeavesExtrasUntouched) {
  DemoSharedConfig cfg;
  cfg.vtcp.offset = {{9.0, 9.0, 9.0}};  // sentinel
  YAML::Node node = YAML::Load("virtual_tcp_offset: [1.0, 2.0]\n");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[0], 1.0);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[1], 2.0);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[2], 9.0);  // preserved
}

// ═══════════════════════════════════════════════════════════════════════════
// Grasp threshold fields
// ═══════════════════════════════════════════════════════════════════════════

TEST(DemoSharedConfigTest, GraspThresholdsAndType) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
grasp_contact_threshold: 0.42
grasp_force_threshold: 2.5
grasp_min_fingertips: 3
grasp_controller_type: force_pi
)YAML");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_FLOAT_EQ(cfg.grasp_contact_threshold, 0.42f);
  EXPECT_FLOAT_EQ(cfg.grasp_force_threshold, 2.5f);
  EXPECT_EQ(cfg.grasp_min_fingertips, 3);
  EXPECT_EQ(cfg.grasp_controller_type, "force_pi");
}

// The whitelist { force_pi, contact_stop, none } is enforced at parse time so
// an unrecognized string never falls through to a controller branch. All three
// valid values must parse without throwing.
TEST(DemoSharedConfigTest, GraspControllerTypeAcceptsWhitelist) {
  for (const char* t : {"force_pi", "contact_stop", "none"}) {
    DemoSharedConfig cfg;
    YAML::Node node;
    node["grasp_controller_type"] = t;
    EXPECT_NO_THROW(ApplyDemoSharedConfig(node, cfg)) << "type=" << t;
    EXPECT_EQ(cfg.grasp_controller_type, t);
  }
}

// An unknown grasp_controller_type must be rejected (propagates to
// on_configure → CallbackReturn::FAILURE), not silently accepted.
TEST(DemoSharedConfigTest, GraspControllerTypeRejectsUnknown) {
  DemoSharedConfig cfg;
  YAML::Node node;
  node["grasp_controller_type"] = "bogus_mode";
  EXPECT_THROW(ApplyDemoSharedConfig(node, cfg), std::runtime_error);
}

// ═══════════════════════════════════════════════════════════════════════════
// force_pi_grasp block parsing
// ═══════════════════════════════════════════════════════════════════════════

TEST(DemoSharedConfigTest, ForcePiBlockParses) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
force_pi_grasp:
  Kp_base: 0.05
  Ki_base: 0.005
  f_contact_threshold: 0.4
  f_target: 3.0
  f_ramp_rate: 2.0
  ds_max: 0.08
  delta_s_max: 0.2
  integral_clamp: 0.15
  approach_speed: 0.3
  release_speed: 0.4
  lpf_cutoff_hz: 30.0
  fingers:
    thumb:
      q_open:  [0.0, 0.1, 0.2]
      q_close: [0.5, 1.0, 0.8]
    index:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.0, 1.1, 0.7]
    middle:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.0, 1.2, 0.9]
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_TRUE(cfg.has_force_pi_block);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.Kp_base, 0.05);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.Ki_base, 0.005);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.f_contact_threshold, 0.4);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.f_target, 3.0);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.f_ramp_rate, 2.0);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.ds_max, 0.08);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.delta_s_max, 0.2);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.integral_clamp, 0.15);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.approach_speed, 0.3);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.release_speed, 0.4);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.lpf_cutoff_hz, 30.0);

  // Thumb (index 0)
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_open[0], 0.0);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_open[1], 0.1);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_open[2], 0.2);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[0], 0.5);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[1], 1.0);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[2], 0.8);

  // Index (1), Middle (2)
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[1].q_close[1], 1.1);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[2].q_close[1], 1.2);
}

TEST(DemoSharedConfigTest, ForcePiBlockAbsentLeavesFlagFalse) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load("grasp_controller_type: force_pi\n");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_FALSE(cfg.has_force_pi_block);
  EXPECT_EQ(cfg.grasp_controller_type, "force_pi");
}

// units: "deg" → 내부 저장은 radians 로 변환되어야 한다.
TEST(DemoSharedConfigTest, ForcePiFingersDegreesConverted) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
force_pi_grasp:
  fingers:
    units: "deg"
    thumb:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [30.0, 60.0, 45.0]
    index:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.0, 90.0, 45.0]
    middle:
      q_open:  [10.0, 0.0, 0.0]
      q_close: [0.0, 60.0, 180.0]
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  constexpr double kD2R = std::numbers::pi_v<double> / 180.0;
  EXPECT_NEAR(cfg.force_pi_fingers[0].q_close[0], 30.0 * kD2R, 1e-12);
  EXPECT_NEAR(cfg.force_pi_fingers[0].q_close[1], 60.0 * kD2R, 1e-12);
  EXPECT_NEAR(cfg.force_pi_fingers[0].q_close[2], 45.0 * kD2R, 1e-12);
  EXPECT_NEAR(cfg.force_pi_fingers[1].q_close[1], 90.0 * kD2R, 1e-12);
  EXPECT_NEAR(cfg.force_pi_fingers[2].q_open[0], 10.0 * kD2R, 1e-12);
  EXPECT_NEAR(cfg.force_pi_fingers[2].q_close[2], std::numbers::pi_v<double>, 1e-12);
}

// units 미지정 → rad 로 해석 (하위 호환).
TEST(DemoSharedConfigTest, ForcePiFingersDefaultsToRadians) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
force_pi_grasp:
  fingers:
    thumb:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.524, 1.047, 0.785]
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[0], 0.524);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[1], 1.047);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[2], 0.785);
}

// units: "rad" 을 명시적으로 설정한 경우도 스케일 없이 통과.
TEST(DemoSharedConfigTest, ForcePiFingersExplicitRadians) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
force_pi_grasp:
  fingers:
    units: "rad"
    thumb:
      q_close: [0.5, 1.0, 0.7]
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[0], 0.5);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[1], 1.0);
  EXPECT_DOUBLE_EQ(cfg.force_pi_fingers[0].q_close[2], 0.7);
}

// ═══════════════════════════════════════════════════════════════════════════
// BuildGraspController gating
// ═══════════════════════════════════════════════════════════════════════════

TEST(BuildGraspControllerTest, ContactStopTypeResetsController) {
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "contact_stop";
  cfg.has_force_pi_block = true;  // ignored — type check wins

  auto ctrl = std::make_unique<rtc::grasp::GraspController>();
  BuildGraspController(cfg, 500.0, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}

TEST(BuildGraspControllerTest, ForcePiWithoutBlockIsSkipped) {
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = false;

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, 500.0, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}

TEST(BuildGraspControllerTest, ForcePiWithBlockBuildsControllerAtRequestedRate) {
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = true;
  cfg.force_pi_params.Kp_base = 0.04;
  cfg.force_pi_params.Ki_base = 0.004;
  cfg.force_pi_params.f_target = 2.5;
  cfg.force_pi_params.f_ramp_rate = 1.0;
  cfg.force_pi_params.lpf_cutoff_hz = 25.0;
  // ds_max must be > 0 so the internal FSM can step; leave default (0.05).

  // Provide plausible finger postures so Init() doesn't misbehave.
  for (auto& fc : cfg.force_pi_fingers) {
    fc.q_open = {0.0, 0.0, 0.0};
    fc.q_close = {0.5, 1.0, 0.7};
  }

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, 500.0, ctrl);
  ASSERT_NE(ctrl.get(), nullptr);

  // Init() seeds active_target_force_ from f_target and enters Idle.
  EXPECT_EQ(ctrl->phase(), rtc::grasp::GraspPhase::kIdle);
  EXPECT_DOUBLE_EQ(ctrl->target_force(), cfg.force_pi_params.f_target);

  // BuildGraspController should overwrite control_rate_hz regardless of YAML.
  // Pass a bogus stale rate; the builder must replace it with the argument.
  cfg.force_pi_params.control_rate_hz = 123.0;
  std::unique_ptr<rtc::grasp::GraspController> ctrl2;
  BuildGraspController(cfg, 250.0, ctrl2);
  ASSERT_NE(ctrl2.get(), nullptr);
  // CommandGrasp immediately updates active_target_force_ (phase transition
  // only occurs on the next Update() tick, so we don't assert on phase here).
  ctrl2->CommandGrasp(1.5);
  EXPECT_DOUBLE_EQ(ctrl2->target_force(), 1.5);
}

TEST(BuildGraspControllerTest, SwitchingBackToContactStopResetsExistingController) {
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = true;
  for (auto& fc : cfg.force_pi_fingers) {
    fc.q_open = {0.0, 0.0, 0.0};
    fc.q_close = {0.5, 1.0, 0.7};
  }

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, 500.0, ctrl);
  ASSERT_NE(ctrl.get(), nullptr);

  cfg.grasp_controller_type = "contact_stop";
  BuildGraspController(cfg, 500.0, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}

// ═══════════════════════════════════════════════════════════════════════════
// pull_estimator block parsing (#167)
// ═══════════════════════════════════════════════════════════════════════════

using integrated_bringup::BuildPullForceEstimator;
using integrated_bringup::PullPlaneNormalSource;

TEST(DemoSharedConfigTest, PullEstimatorDefaults) {
  DemoSharedConfig cfg;
  EXPECT_FALSE(cfg.has_pull_estimator_block);
  EXPECT_TRUE(cfg.pull_estimator_enabled);
  EXPECT_EQ(cfg.num_pull_contacts, 0);
  EXPECT_EQ(cfg.pull_plane_normal_source, PullPlaneNormalSource::kFixed);
  EXPECT_FALSE(cfg.pull_use_baseline_subtraction);
  EXPECT_FALSE(cfg.pull_has_direction);
}

TEST(DemoSharedConfigTest, PullEstimatorBlockAbsentLeavesFlagFalse) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load("grasp_controller_type: force_pi\n");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_FALSE(cfg.has_pull_estimator_block);
}

TEST(DemoSharedConfigTest, PullEstimatorBlockParses) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
pull_estimator:
  enabled: true
  filter_cutoff_hz: 8.0
  min_valid_contacts: 3
  decay_time_constant_s: 0.2
  slip_risk_threshold: 0.9
  alignment_error_rad: 0.05
  gravity_force: [0.0, 0.0, -1.5]
  plane_normal_source: "pinch_geometry"
  plane_normal: [0.0, 1.0, 0.0]
  use_baseline_subtraction: true
  pull_direction: [1.0, 0.0, 0.0]
  required_roles: ["thumb"]
  tips:
    tip_names: ["thumb", "index", "middle"]
    thumb:
      link: "thumb_tip_link"
      contact_normal_local: [0.0, 1.0, 0.0]
      friction_coeff: 0.6
      force_saturation: 30.0
      contact_on_threshold: 0.7
      contact_off_threshold: 0.3
      force_sign: 1.0
      force_bias: [0.1, -0.1, 0.2]
    index:
      link: "index_tip_link"
    middle:
      link: "middle_tip_link"
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_TRUE(cfg.has_pull_estimator_block);
  EXPECT_TRUE(cfg.pull_estimator_enabled);
  EXPECT_DOUBLE_EQ(cfg.pull_estimator_params.filter_cutoff_hz, 8.0);
  EXPECT_EQ(cfg.pull_estimator_params.min_valid_contacts, 3);
  EXPECT_DOUBLE_EQ(cfg.pull_estimator_params.decay_time_constant_s, 0.2);
  EXPECT_DOUBLE_EQ(cfg.pull_estimator_params.slip_risk_threshold, 0.9);
  EXPECT_DOUBLE_EQ(cfg.pull_estimator_params.alignment_error_rad, 0.05);
  EXPECT_DOUBLE_EQ(cfg.pull_estimator_params.gravity_force.z(), -1.5);
  EXPECT_EQ(cfg.pull_plane_normal_source, PullPlaneNormalSource::kPinchGeometry);
  EXPECT_DOUBLE_EQ(cfg.pull_plane_normal.y(), 1.0);
  EXPECT_TRUE(cfg.pull_use_baseline_subtraction);
  EXPECT_TRUE(cfg.pull_has_direction);
  EXPECT_DOUBLE_EQ(cfg.pull_direction.x(), 1.0);

  ASSERT_EQ(cfg.num_pull_contacts, 3);
  EXPECT_EQ(cfg.pull_tip_roles[0], "thumb");
  EXPECT_EQ(cfg.pull_tip_roles[1], "index");
  EXPECT_EQ(cfg.pull_tip_roles[2], "middle");
  EXPECT_EQ(cfg.pull_tip_links[0], "thumb_tip_link");
  EXPECT_EQ(cfg.pull_tip_links[1], "index_tip_link");
  EXPECT_EQ(cfg.pull_tip_links[2], "middle_tip_link");

  const auto& thumb = cfg.pull_contacts[0];
  EXPECT_DOUBLE_EQ(thumb.contact_normal_local.y(), 1.0);
  EXPECT_DOUBLE_EQ(thumb.friction_coeff, 0.6);
  EXPECT_DOUBLE_EQ(thumb.force_saturation, 30.0);
  EXPECT_DOUBLE_EQ(thumb.contact_on_threshold, 0.7);
  EXPECT_DOUBLE_EQ(thumb.contact_off_threshold, 0.3);
  EXPECT_DOUBLE_EQ(thumb.force_sign, 1.0);
  EXPECT_DOUBLE_EQ(thumb.force_bias.x(), 0.1);
  EXPECT_TRUE(thumb.required);

  // Tips not named in required_roles keep required=false; unset per-tip keys
  // keep their PullContactConfig defaults.
  EXPECT_FALSE(cfg.pull_contacts[1].required);
  EXPECT_FALSE(cfg.pull_contacts[2].required);
  EXPECT_DOUBLE_EQ(cfg.pull_contacts[1].force_sign, -1.0);
  EXPECT_DOUBLE_EQ(cfg.pull_contacts[1].friction_coeff, 0.7);
}

TEST(DemoSharedConfigTest, PullEstimatorForceCalibrationParsesRowMajor) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
pull_estimator:
  tips:
    tip_names: ["thumb"]
    thumb:
      force_calibration: [0.0, 1.0, 0.0,
                          -1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0]
)YAML");
  ApplyDemoSharedConfig(node, cfg);
  const auto& c = cfg.pull_contacts[0].force_calibration;
  EXPECT_DOUBLE_EQ(c(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(c(1, 0), -1.0);
  EXPECT_DOUBLE_EQ(c(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(c(0, 0), 0.0);
}

TEST(DemoSharedConfigTest, PullPlaneNormalSourceWhitelist) {
  for (const char* s : {"fixed", "pinch_geometry"}) {
    DemoSharedConfig cfg;
    YAML::Node node;
    node["pull_estimator"]["plane_normal_source"] = s;
    EXPECT_NO_THROW(ApplyDemoSharedConfig(node, cfg)) << "source=" << s;
  }
  for (const char* s : {"vision", "bogus"}) {
    DemoSharedConfig cfg;
    YAML::Node node;
    node["pull_estimator"]["plane_normal_source"] = s;
    EXPECT_THROW(ApplyDemoSharedConfig(node, cfg), std::runtime_error) << "source=" << s;
  }
}

// Typo in required_roles must be a hard error: silently dropping the
// thumb-mandatory gate would let index+middle count as a valid pinch.
TEST(DemoSharedConfigTest, PullEstimatorUnknownRequiredRoleThrows) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
pull_estimator:
  required_roles: ["thmub"]
  tips:
    tip_names: ["thumb", "index", "middle"]
)YAML");
  EXPECT_THROW(ApplyDemoSharedConfig(node, cfg), std::runtime_error);
}

// ═══════════════════════════════════════════════════════════════════════════
// BuildPullForceEstimator gating
// ═══════════════════════════════════════════════════════════════════════════

namespace {

YAML::Node MinimalPullEstimatorYaml() {
  return YAML::Load(R"YAML(
pull_estimator:
  tips:
    tip_names: ["thumb", "index", "middle"]
    thumb:
      contact_normal_local: [0.0, 0.0, 1.0]
    index:
      contact_normal_local: [0.0, 0.0, 1.0]
    middle:
      contact_normal_local: [0.0, 0.0, 1.0]
)YAML");
}

}  // namespace

TEST(BuildPullForceEstimatorTest, NoBlockResetsEstimator) {
  DemoSharedConfig cfg;
  auto est = std::make_unique<rtc::grasp::PullForceEstimator>();
  BuildPullForceEstimator(cfg, 500.0, est);
  EXPECT_EQ(est.get(), nullptr);
}

TEST(BuildPullForceEstimatorTest, DisabledBlockResetsEstimator) {
  DemoSharedConfig cfg;
  YAML::Node node = MinimalPullEstimatorYaml();
  node["pull_estimator"]["enabled"] = false;
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_TRUE(cfg.has_pull_estimator_block);
  EXPECT_FALSE(cfg.pull_estimator_enabled);

  auto est = std::make_unique<rtc::grasp::PullForceEstimator>();
  BuildPullForceEstimator(cfg, 500.0, est);
  EXPECT_EQ(est.get(), nullptr);
}

TEST(BuildPullForceEstimatorTest, EnabledBlockBuildsEstimator) {
  DemoSharedConfig cfg;
  ApplyDemoSharedConfig(MinimalPullEstimatorYaml(), cfg);

  std::unique_ptr<rtc::grasp::PullForceEstimator> est;
  BuildPullForceEstimator(cfg, 500.0, est);
  ASSERT_NE(est.get(), nullptr);
  EXPECT_EQ(est->num_contacts(), 3);
}

// Init() validation must propagate (on_configure → FAILURE), not be swallowed:
// hysteresis on <= off is invalid.
TEST(BuildPullForceEstimatorTest, InvalidHysteresisThrows) {
  DemoSharedConfig cfg;
  YAML::Node node = MinimalPullEstimatorYaml();
  node["pull_estimator"]["tips"]["thumb"]["contact_on_threshold"] = 0.1;
  node["pull_estimator"]["tips"]["thumb"]["contact_off_threshold"] = 0.5;
  ApplyDemoSharedConfig(node, cfg);

  std::unique_ptr<rtc::grasp::PullForceEstimator> est;
  EXPECT_THROW(BuildPullForceEstimator(cfg, 500.0, est), std::invalid_argument);
}
