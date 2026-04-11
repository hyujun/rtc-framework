// ─────────────────────────────────────────────────────────────────────────────
// Unit tests for ur5e_bringup::ApplyDemoSharedConfig and BuildGraspController
// (demo_shared_config.hpp / demo_shared_config.cpp)
// ─────────────────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>

#include "ur5e_bringup/controllers/demo_shared_config.hpp"
#include "ur5e_bringup/controllers/virtual_tcp.hpp"

#include "rtc_controllers/grasp/grasp_controller.hpp"
#include "rtc_controllers/grasp/grasp_types.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <numbers>
#include <string>

using ur5e_bringup::ApplyDemoSharedConfig;
using ur5e_bringup::BuildGraspController;
using ur5e_bringup::DemoSharedConfig;
using ur5e_bringup::VirtualTcpMode;

// ═══════════════════════════════════════════════════════════════════════════
// Defaults / no-op
// ═══════════════════════════════════════════════════════════════════════════

TEST(DemoSharedConfigTest, DefaultConstructedValues)
{
  DemoSharedConfig cfg;
  EXPECT_EQ(cfg.vtcp.mode, VirtualTcpMode::kDisabled);
  EXPECT_FLOAT_EQ(cfg.grasp_contact_threshold, 0.5f);
  EXPECT_FLOAT_EQ(cfg.grasp_force_threshold, 1.0f);
  EXPECT_EQ(cfg.grasp_min_fingertips, 2);
  EXPECT_EQ(cfg.grasp_controller_type, "contact_stop");
  EXPECT_FALSE(cfg.has_force_pi_block);
}

TEST(DemoSharedConfigTest, NullNodeIsNoOp)
{
  DemoSharedConfig cfg;
  cfg.grasp_contact_threshold = 0.75f;
  cfg.grasp_controller_type = "force_pi";

  YAML::Node empty;  // invalid / null
  ApplyDemoSharedConfig(empty, cfg);

  EXPECT_FLOAT_EQ(cfg.grasp_contact_threshold, 0.75f);
  EXPECT_EQ(cfg.grasp_controller_type, "force_pi");
}

TEST(DemoSharedConfigTest, PartialOverridePreservesOtherFields)
{
  DemoSharedConfig cfg;
  cfg.grasp_contact_threshold = 0.9f;     // preserved
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

TEST(DemoSharedConfigTest, VirtualTcpModeStrings)
{
  struct Case { const char* str; VirtualTcpMode expected; };
  const Case cases[] = {
    {"centroid", VirtualTcpMode::kCentroid},
    {"weighted", VirtualTcpMode::kWeighted},
    {"constant", VirtualTcpMode::kConstant},
    {"disabled", VirtualTcpMode::kDisabled},
    {"bogus",    VirtualTcpMode::kDisabled},  // unknown → disabled
  };
  for (const auto& c : cases) {
    DemoSharedConfig cfg;
    YAML::Node node;
    node["virtual_tcp_mode"] = c.str;
    ApplyDemoSharedConfig(node, cfg);
    EXPECT_EQ(cfg.vtcp.mode, c.expected) << "input=" << c.str;
  }
}

TEST(DemoSharedConfigTest, VirtualTcpOffsetAndOrientation)
{
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
virtual_tcp_mode: constant
virtual_tcp_offset: [0.1, -0.2, 0.3]
virtual_tcp_orientation: [1.5708, 0.0, -1.5708]
)YAML");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_EQ(cfg.vtcp.mode, VirtualTcpMode::kConstant);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[0],  0.1);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[1], -0.2);
  EXPECT_DOUBLE_EQ(cfg.vtcp.offset[2],  0.3);
  EXPECT_DOUBLE_EQ(cfg.vtcp.orientation[0],  1.5708);
  EXPECT_DOUBLE_EQ(cfg.vtcp.orientation[1],  0.0);
  EXPECT_DOUBLE_EQ(cfg.vtcp.orientation[2], -1.5708);
}

TEST(DemoSharedConfigTest, VirtualTcpOffsetShortSequenceLeavesExtrasUntouched)
{
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

TEST(DemoSharedConfigTest, GraspThresholdsAndType)
{
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

// ═══════════════════════════════════════════════════════════════════════════
// force_pi_grasp block parsing
// ═══════════════════════════════════════════════════════════════════════════

TEST(DemoSharedConfigTest, ForcePiBlockParses)
{
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

TEST(DemoSharedConfigTest, ForcePiBlockAbsentLeavesFlagFalse)
{
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load("grasp_controller_type: force_pi\n");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_FALSE(cfg.has_force_pi_block);
  EXPECT_EQ(cfg.grasp_controller_type, "force_pi");
}

// units: "deg" → 내부 저장은 radians 로 변환되어야 한다.
TEST(DemoSharedConfigTest, ForcePiFingersDegreesConverted)
{
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
  EXPECT_NEAR(cfg.force_pi_fingers[2].q_open[0],  10.0 * kD2R, 1e-12);
  EXPECT_NEAR(cfg.force_pi_fingers[2].q_close[2], std::numbers::pi_v<double>, 1e-12);
}

// units 미지정 → rad 로 해석 (하위 호환).
TEST(DemoSharedConfigTest, ForcePiFingersDefaultsToRadians)
{
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
TEST(DemoSharedConfigTest, ForcePiFingersExplicitRadians)
{
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

TEST(BuildGraspControllerTest, ContactStopTypeResetsController)
{
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "contact_stop";
  cfg.has_force_pi_block = true;  // ignored — type check wins

  auto ctrl = std::make_unique<rtc::grasp::GraspController>();
  BuildGraspController(cfg, 500.0, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}

TEST(BuildGraspControllerTest, ForcePiWithoutBlockIsSkipped)
{
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = false;

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, 500.0, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}

TEST(BuildGraspControllerTest, ForcePiWithBlockBuildsControllerAtRequestedRate)
{
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
    fc.q_open  = {0.0, 0.0, 0.0};
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

TEST(BuildGraspControllerTest, SwitchingBackToContactStopResetsExistingController)
{
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = true;
  for (auto& fc : cfg.force_pi_fingers) {
    fc.q_open  = {0.0, 0.0, 0.0};
    fc.q_close = {0.5, 1.0, 0.7};
  }

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, 500.0, ctrl);
  ASSERT_NE(ctrl.get(), nullptr);

  cfg.grasp_controller_type = "contact_stop";
  BuildGraspController(cfg, 500.0, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}
