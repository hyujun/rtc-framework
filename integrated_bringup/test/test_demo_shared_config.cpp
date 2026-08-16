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
using integrated_bringup::GraspCommandRejectReason;
using integrated_bringup::GraspHandMode;
using integrated_bringup::GraspHandModeName;
using integrated_bringup::GraspModeChangeRejectReason;
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

// String → enum → string must be the identity on the whole whitelist. The
// controllers keep the mode as a GraspHandMode enum and hand the name back out
// on two paths that a caller compares against the YAML they wrote: the
// read-only `grasp_controller_type` parameter the demo GUI reads to decide
// whether Grasp/Release can do anything, and the [grasp] throttled log line. A
// mapping that dropped a case (e.g. kNone printing "contact_stop") would make
// both lie about a config that loaded correctly, with nothing else to catch it.
TEST(DemoSharedConfigTest, GraspHandModeNameRoundTripsWhitelist) {
  for (const char* t : {"force_pi", "contact_stop", "none"}) {
    const auto mode = integrated_bringup::ParseGraspHandMode(t);
    EXPECT_STREQ(integrated_bringup::GraspHandModeName(mode), t);
  }
}

// The enum→name direction must also be total: every enumerator maps to a
// distinct, non-empty name. Iterating the enum (rather than the strings) is
// what catches a *newly added* mode that forgot its GraspHandModeName case and
// so falls through to another mode's label.
TEST(DemoSharedConfigTest, GraspHandModeNamesAreDistinct) {
  using integrated_bringup::GraspHandMode;
  using integrated_bringup::GraspHandModeName;
  const std::string contact_stop = GraspHandModeName(GraspHandMode::kContactStop);
  const std::string force_pi = GraspHandModeName(GraspHandMode::kForcePi);
  const std::string none = GraspHandModeName(GraspHandMode::kNone);

  EXPECT_FALSE(contact_stop.empty());
  EXPECT_NE(contact_stop, force_pi);
  EXPECT_NE(contact_stop, none);
  EXPECT_NE(force_pi, none);

  // And each name must survive the trip back, so the pair stays a bijection
  // over the whitelist rather than merely being distinct.
  EXPECT_EQ(integrated_bringup::ParseGraspHandMode(contact_stop), GraspHandMode::kContactStop);
  EXPECT_EQ(integrated_bringup::ParseGraspHandMode(force_pi), GraspHandMode::kForcePi);
  EXPECT_EQ(integrated_bringup::ParseGraspHandMode(none), GraspHandMode::kNone);
}

// ParseGraspHandMode is the enum-side gate; it must reject the same strings
// ApplyDemoSharedConfig rejects rather than defaulting to a mode.
TEST(DemoSharedConfigTest, ParseGraspHandModeRejectsUnknown) {
  EXPECT_THROW((void)integrated_bringup::ParseGraspHandMode("bogus_mode"), std::runtime_error);
  EXPECT_THROW((void)integrated_bringup::ParseGraspHandMode(""), std::runtime_error);
  // Case matters — the YAML whitelist is exact.
  EXPECT_THROW((void)integrated_bringup::ParseGraspHandMode("Force_PI"), std::runtime_error);
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
  // Same YAML key, new home: the filter it configures is the controller's
  // per-axis bank, not a GraspParams field.
  EXPECT_DOUBLE_EQ(cfg.force_pi_lpf_cutoff_hz, 30.0);

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

// Anomaly 축 세 키는 이 블록에서 따로 파싱된다. grip_decay_rate 는 원래 로더에
// 아예 없어서 YAML 로 설정할 수 없었다 (적어도 조용히 무시되고 default 가 돌았다).
TEST(DemoSharedConfigTest, ForcePiAnomalyParamsParse) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
force_pi_grasp:
  df_slip_threshold: 4.0
  f_slip_fraction: 0.6
  grip_tightening_rate: 0.9
  grip_decay_rate: 0.25
  f_max_multiplier: 1.5
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_DOUBLE_EQ(cfg.force_pi_params.df_slip_threshold, 4.0);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.f_slip_fraction, 0.6);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.grip_tightening_rate, 0.9);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.grip_decay_rate, 0.25);
  EXPECT_DOUBLE_EQ(cfg.force_pi_params.f_max_multiplier, 1.5);
}

// 제거된 키는 조용히 무시되지 않고 실패한다. 로더가 모르는 키를 넘기는 구조라
// 이 거부가 없으면 남아 있는 grip_tightening_ratio 가 사라지고 default rate 가
// 대신 도는 것을 아무도 모른다. 단위가 바뀌었으므로 (per-tick 비율 -> N/s)
// 자동 변환도 불가능하다.
TEST(DemoSharedConfigTest, ForcePiRejectsRemovedGripTighteningRatio) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load("force_pi_grasp:\n  grip_tightening_ratio: 0.15\n");
  // 메시지까지 고정한다 — 다른 이유로 throw 해도 EXPECT_THROW 는 green 이 된다.
  try {
    ApplyDemoSharedConfig(node, cfg);
    FAIL() << "removed key was accepted";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("grip_tightening_rate"), std::string::npos)
        << "메시지가 대체 키를 알려주지 않는다: " << e.what();
  }
}

// #432: 전이 판정의 thumb 슬롯은 finger_names 에서 해석된다. 기본 순서라
// 인덱스가 0 이더라도 "해석했다" 를 고정해야 한다 — 해석을 통째로 지워도
// GraspParams 의 default 0 이 같은 값을 주기 때문이다.
TEST(DemoSharedConfigTest, ForcePiThumbSlotResolvedFromFingerNames) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
force_pi_grasp:
  fingers:
    finger_names: ["index", "middle", "thumb", "ring"]
    index:
      q_open:  [0.0]
      q_close: [1.0]
    middle:
      q_open:  [0.0]
      q_close: [1.0]
    thumb:
      q_open:  [0.0]
      q_close: [1.0]
    ring:
      q_open:  [0.0]
      q_close: [1.0]
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_EQ(cfg.force_pi_params.thumb_finger_index, 2);
}

// "thumb" 이 없는 이름 목록은 구성 실패가 아니라 관례(슬롯 0)로 낙하한다 —
// thumb 을 그렇게 부르지 않는 손도 계속 구성되어야 하고, 전이는 슬롯 0 을
// 대향 손가락으로 삼아 계속 발화한다.
TEST(DemoSharedConfigTest, ForcePiThumbSlotFallsBackWhenNoThumbNamed) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
force_pi_grasp:
  fingers:
    finger_names: ["left_jaw", "right_jaw"]
    left_jaw:
      q_open:  [0.0]
      q_close: [1.0]
    right_jaw:
      q_open:  [0.0]
      q_close: [1.0]
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_EQ(cfg.force_pi_params.thumb_finger_index, 0);
  // num_grasp_fingers is deliberately NOT asserted here: it comes from
  // hand_finger_joint_map, which is parsed independently of finger_names
  // (grasp_tuning_guide.md 1.2). Tying the two in a test would pin a coupling
  // the loader does not have.
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

// SPEC CHANGE (B-3, was ContactStopTypeResetsController): the mode no longer
// gates construction, the block does. A force-pi block buys the *capability* to
// run the PI law; which law drives the hand each tick is Gains::grasp_hand_mode,
// and that is writable at runtime — so skipping construction under contact_stop
// would make the mode a one-way door (the switch cannot re-configure).
//
// Every mode, same answer. `none` is in the list on purpose: ur5e_p1b ships
// exactly that combination (a force_pi_grasp block with type "none"), so it is the
// shipped config this test speaks for, not a synthetic case.
TEST(BuildGraspControllerTest, ModeDoesNotGateTheBuildWhenTheBlockIsPresent) {
  for (const char* type : {"force_pi", "contact_stop", "none"}) {
    DemoSharedConfig cfg;
    cfg.grasp_controller_type = type;
    cfg.has_force_pi_block = true;
    for (auto& fc : cfg.force_pi_fingers) {
      fc.q_open = {0.0, 0.0, 0.0};
      fc.q_close = {0.5, 1.0, 0.7};
    }

    std::unique_ptr<rtc::grasp::GraspController> ctrl;
    BuildGraspController(cfg, ctrl);
    EXPECT_NE(ctrl.get(), nullptr) << "type " << type;
  }
}

TEST(BuildGraspControllerTest, ForcePiWithoutBlockIsSkipped) {
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = false;

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}

TEST(BuildGraspControllerTest, ForcePiWithBlockBuildsController) {
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = true;
  cfg.force_pi_params.Kp_base = 0.04;
  cfg.force_pi_params.Ki_base = 0.004;
  cfg.force_pi_params.f_target = 2.5;
  cfg.force_pi_params.f_ramp_rate = 1.0;
  cfg.force_pi_lpf_cutoff_hz = 25.0;
  // ds_max must be > 0 so the internal FSM can step; leave default (0.05).

  // Provide plausible finger postures so Init() doesn't misbehave.
  for (auto& fc : cfg.force_pi_fingers) {
    fc.q_open = {0.0, 0.0, 0.0};
    fc.q_close = {0.5, 1.0, 0.7};
  }

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, ctrl);
  ASSERT_NE(ctrl.get(), nullptr);

  // Init() seeds active_target_force_ from f_target and enters Idle.
  EXPECT_EQ(ctrl->phase(), rtc::grasp::GraspPhase::kIdle);
  EXPECT_DOUBLE_EQ(ctrl->target_force(), cfg.force_pi_params.f_target);

  // SPEC CHANGE: the "builder overwrites control_rate_hz" half of this test is
  // gone with the field. GraspController holds no filter, so it needs no sample
  // rate; the rate now reaches only the controller's own filter bank, where
  // GetDefaultDt() is its single source. Nothing is left unpinned — a wrong rate
  // there is a wrong cutoff, which TheTwoBanksHaveIndependentCutoffs measures.
  std::unique_ptr<rtc::grasp::GraspController> ctrl2;
  BuildGraspController(cfg, ctrl2);
  ASSERT_NE(ctrl2.get(), nullptr);
  // CommandGrasp immediately updates active_target_force_ (phase transition
  // only occurs on the next Update() tick, so we don't assert on phase here).
  ctrl2->CommandGrasp(1.5);
  EXPECT_DOUBLE_EQ(ctrl2->target_force(), 1.5);
}

// SPEC CHANGE (B-3, was SwitchingBackToContactStopResetsExistingController). The
// reset path is NOT deleted — it is re-aimed at the axis that still owns the
// decision. Both halves matter and they fail for different reasons: the first
// would catch a build that still destroys the controller when the mode moves off
// force_pi (the one-way door), the second a build that leaks a stale controller
// once the capability it was made from is gone.
TEST(BuildGraspControllerTest, TheBlockOwnsTheResetDecisionNotTheMode) {
  DemoSharedConfig cfg;
  cfg.grasp_controller_type = "force_pi";
  cfg.has_force_pi_block = true;
  for (auto& fc : cfg.force_pi_fingers) {
    fc.q_open = {0.0, 0.0, 0.0};
    fc.q_close = {0.5, 1.0, 0.7};
  }

  std::unique_ptr<rtc::grasp::GraspController> ctrl;
  BuildGraspController(cfg, ctrl);
  ASSERT_NE(ctrl.get(), nullptr);

  // Mode moves away from force_pi: the capability survives.
  cfg.grasp_controller_type = "contact_stop";
  BuildGraspController(cfg, ctrl);
  EXPECT_NE(ctrl.get(), nullptr);

  // The block goes away: the controller must go with it.
  cfg.has_force_pi_block = false;
  BuildGraspController(cfg, ctrl);
  EXPECT_EQ(ctrl.get(), nullptr);
}

// ═══════════════════════════════════════════════════════════════════════════
// grasp_command admission (B-3)
// ═══════════════════════════════════════════════════════════════════════════

// The service gate that used to be `!grasp_controller_`. Once the build stopped
// consulting the mode, nullness stopped meaning "the PI law is running", and an
// accepted command under contact_stop / none would answer "grasp started", step
// the FSM, and change nothing the hand can feel.
TEST(GraspCommandRejectReasonTest, OnlyForcePiWithAControllerIsAdmitted) {
  EXPECT_EQ(GraspCommandRejectReason(GraspHandMode::kForcePi, /*has_controller=*/true), nullptr);

  for (auto mode : {GraspHandMode::kContactStop, GraspHandMode::kNone}) {
    const char* why = GraspCommandRejectReason(mode, /*has_controller=*/true);
    ASSERT_NE(why, nullptr) << GraspHandModeName(mode) << " admitted a grasp command";
    // The two refusals must not read the same: "no block in your YAML" and "the
    // block is there but this mode does not run it" are different operator
    // problems with different fixes, and this message is all the operator gets.
    EXPECT_STRNE(why, GraspCommandRejectReason(GraspHandMode::kForcePi,
                                               /*has_controller=*/false));
  }
}

// Capability is checked before policy: with no controller the reason must name the
// missing block whatever the mode says, including force_pi.
TEST(GraspCommandRejectReasonTest, MissingControllerIsRefusedInEveryMode) {
  for (auto mode : {GraspHandMode::kForcePi, GraspHandMode::kContactStop, GraspHandMode::kNone}) {
    EXPECT_STREQ(GraspCommandRejectReason(mode, /*has_controller=*/false),
                 GraspCommandRejectReason(GraspHandMode::kForcePi, /*has_controller=*/false))
        << GraspHandModeName(mode);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Runtime grasp-mode change: the quiet gate (B-4b)
// ═══════════════════════════════════════════════════════════════════════════

namespace {
constexpr integrated_bringup::GraspModeSwitchState kQuiet{
    /*has_controller=*/true, /*contact_latched=*/false, /*grasp_phase_idle=*/true};
}  // namespace

// The policy: the mode may only move while the hand is quiet.
TEST(GraspModeChangeRejectReasonTest, QuietHandWithTheCapabilityIsAdmitted) {
  EXPECT_EQ(
      GraspModeChangeRejectReason(GraspHandMode::kContactStop, GraspHandMode::kForcePi, kQuiet),
      nullptr);
  EXPECT_EQ(
      GraspModeChangeRejectReason(GraspHandMode::kForcePi, GraspHandMode::kContactStop, kQuiet),
      nullptr);
  EXPECT_EQ(GraspModeChangeRejectReason(GraspHandMode::kForcePi, GraspHandMode::kNone, kQuiet),
            nullptr);
}

// A request for the mode already in force must be admitted even when the hand is
// as un-quiet as it gets. It changes nothing, so refusing it would make an
// idempotent set fail while an object is held — including the confirming re-send a
// GUI does against what it is already displaying.
TEST(GraspModeChangeRejectReasonTest, NoOpRequestIsNeverRefused) {
  const integrated_bringup::GraspModeSwitchState busy{
      /*has_controller=*/true, /*contact_latched=*/true, /*grasp_phase_idle=*/false};
  for (auto mode : {GraspHandMode::kContactStop, GraspHandMode::kForcePi, GraspHandMode::kNone}) {
    EXPECT_EQ(GraspModeChangeRejectReason(mode, mode, busy), nullptr) << GraspHandModeName(mode);
  }
  // Including with no controller at all — a config running "none" without a block
  // must still be able to re-assert "none".
  EXPECT_EQ(GraspModeChangeRejectReason(GraspHandMode::kNone, GraspHandMode::kNone,
                                        {/*has_controller=*/false, true, false}),
            nullptr);
}

// Each refusal has to be distinguishable: the three have different fixes (edit the
// YAML / open the hand / finish the grasp) and this string is all the operator gets.
TEST(GraspModeChangeRejectReasonTest, EveryRefusalNamesItsOwnCause) {
  const char* no_block = GraspModeChangeRejectReason(
      GraspHandMode::kContactStop, GraspHandMode::kForcePi,
      {/*has_controller=*/false, /*contact_latched=*/false, /*grasp_phase_idle=*/true});
  const char* latched = GraspModeChangeRejectReason(
      GraspHandMode::kContactStop, GraspHandMode::kForcePi,
      {/*has_controller=*/true, /*contact_latched=*/true, /*grasp_phase_idle=*/true});
  const char* grasping = GraspModeChangeRejectReason(
      GraspHandMode::kForcePi, GraspHandMode::kContactStop,
      {/*has_controller=*/true, /*contact_latched=*/false, /*grasp_phase_idle=*/false});

  ASSERT_NE(no_block, nullptr);
  ASSERT_NE(latched, nullptr);
  ASSERT_NE(grasping, nullptr);
  EXPECT_STRNE(no_block, latched);
  EXPECT_STRNE(no_block, grasping);
  EXPECT_STRNE(latched, grasping);
}

// Capability before policy, same order as the command gate: a force_pi request
// with no block reports the missing block even when the hand is also busy, because
// waiting will not fix that one.
TEST(GraspModeChangeRejectReasonTest, MissingCapabilityOutranksTheQuietChecks) {
  const char* both = GraspModeChangeRejectReason(
      GraspHandMode::kContactStop, GraspHandMode::kForcePi,
      {/*has_controller=*/false, /*contact_latched=*/true, /*grasp_phase_idle=*/false});
  const char* capability_only = GraspModeChangeRejectReason(
      GraspHandMode::kContactStop, GraspHandMode::kForcePi,
      {/*has_controller=*/false, /*contact_latched=*/false, /*grasp_phase_idle=*/true});
  ASSERT_NE(both, nullptr);
  EXPECT_STREQ(both, capability_only);
}

// Leaving force_pi is gated on the FSM being idle even with no contact latch, and
// entering a mode that needs no capability is still gated on the latch. Neither
// quiet term is redundant.
TEST(GraspModeChangeRejectReasonTest, BothQuietTermsGateIndependently) {
  // Latch only.
  EXPECT_NE(GraspModeChangeRejectReason(
                GraspHandMode::kContactStop, GraspHandMode::kNone,
                {/*has_controller=*/true, /*contact_latched=*/true, /*grasp_phase_idle=*/true}),
            nullptr);
  // Phase only.
  EXPECT_NE(GraspModeChangeRejectReason(
                GraspHandMode::kForcePi, GraspHandMode::kNone,
                {/*has_controller=*/true, /*contact_latched=*/false, /*grasp_phase_idle=*/false}),
            nullptr);
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
  EXPECT_DOUBLE_EQ(cfg.pull_contacts[1].force_sign, 1.0);  // finger-on-object default
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
      link: "thumb_tip_link"
    index:
      link: "index_tip_link"
    middle:
      link: "middle_tip_link"
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

// ═══════════════════════════════════════════════════════════════════════════
// Deployed force_pi tuning — the shipped YAML, not a copy of it
// ═══════════════════════════════════════════════════════════════════════════

// rtc_controllers' DeployedTuningReachesHoldWithinBudget asserts that the
// deployed grasp tuning reaches kHolding inside the behaviour tree's 10 s
// budget, but it runs against a hand-written mirror of this block, so on its
// own a YAML edit cannot fail it. This test closes that loop from the other
// side: it parses the shipped file and pins the values that mirror copies. If
// it fails after a retune, update GraspStiffnessEstimationTest's
// DeployedParams() to match and re-check the budget claim there.
namespace {
DemoSharedConfig LoadDeployedShared(const std::string& variant) {
  const std::string path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + variant + "/controllers/demo_shared.yaml";
  const YAML::Node root = YAML::LoadFile(path);
  DemoSharedConfig cfg;
  ApplyDemoSharedConfig(root["demo_shared"], cfg);
  return cfg;
}
}  // namespace

TEST(DeployedForcePiTuning, MatchesTheTuningRtcControllersAssertsAgainst) {
  for (const std::string variant : {"ur5e_p1a", "ur5e_p1b"}) {
    const DemoSharedConfig cfg = LoadDeployedShared(variant);
    ASSERT_TRUE(cfg.has_force_pi_block) << variant << ": force_pi_grasp block missing";
    const auto& gp = cfg.force_pi_params;

    // beta and K_est_max ship as a pair and neither may move alone. K_est_max
    // equals the seed K_contact_est starts from, so the estimate is pinned and
    // gain_scale is the constant 1/(1 + beta) = 0.769 — the behaviour these
    // robots ran while the estimator was inert. Releasing the estimate (400)
    // without retuning beta to 0.03 puts every grasp past the behaviour tree's
    // 10 s budget; retuning beta while pinned changes every grasp's gain by 26%
    // for no adaptation. #426 retired the release these two were waiting on
    // (grasp_tuning_guide.md 6.9), so this pair is now permanent rather than
    // pending — a diff that moves either value is a regression, not a rollout.
    EXPECT_DOUBLE_EQ(gp.beta, 0.3) << variant;
    EXPECT_DOUBLE_EQ(gp.K_est_max, 1.0) << variant;
    EXPECT_DOUBLE_EQ(gp.Kp_base, 0.02) << variant;
    EXPECT_DOUBLE_EQ(gp.Ki_base, 0.002) << variant;
    EXPECT_DOUBLE_EQ(gp.alpha_ema, 0.95) << variant;
    EXPECT_DOUBLE_EQ(gp.f_contact_threshold, 0.8) << variant;
    EXPECT_DOUBLE_EQ(gp.f_target, 2.0) << variant;
    EXPECT_DOUBLE_EQ(gp.f_ramp_rate, 2.0) << variant;
    EXPECT_DOUBLE_EQ(gp.ds_max, 0.05) << variant;
    EXPECT_DOUBLE_EQ(gp.delta_s_max, 0.15) << variant;
    EXPECT_DOUBLE_EQ(gp.integral_clamp, 0.1) << variant;
    EXPECT_DOUBLE_EQ(gp.approach_speed, 0.4) << variant;
    EXPECT_DOUBLE_EQ(gp.release_speed, 0.3) << variant;
    EXPECT_DOUBLE_EQ(gp.settle_epsilon, 0.5) << variant;
    EXPECT_DOUBLE_EQ(gp.settle_time, 0.3) << variant;
    EXPECT_DOUBLE_EQ(gp.contact_settle_time, 0.1) << variant;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// momentum_observer block + payload_estimator sub-block parsing (#135)
// ═══════════════════════════════════════════════════════════════════════════
//
// The YAML→struct hop for these two blocks had no test at all, which is worse
// here than the missing coverage suggests: every key is OPTIONAL and silently
// falls back to its default, so a misspelled key, a key wired to the wrong
// field, or a whole sub-block hung under the wrong parent all produce a
// perfectly valid config that simply does not do what it says. The estimator
// downstream cannot tell "the user asked for 0.02" from "nobody asked".
//
// Hence the shape below: every value differs from BOTH its own default and
// every other value in the block. Equal-valued fixtures are the failure mode
// this file is guarding — `num("min_sigma", pp.sigma0)` is one keystroke from
// the real line and passes any fixture whose two numbers happen to match.

TEST(DemoSharedConfigTest, MomentumObserverPayloadDefaults) {
  const integrated_bringup::MomentumObserverParams mp;
  EXPECT_FALSE(mp.has_block);
  EXPECT_TRUE(mp.enabled);
  EXPECT_TRUE(mp.gains.empty());

  const integrated_bringup::PayloadEstimatorParams& pp = mp.payload;
  EXPECT_FALSE(pp.has_block);
  EXPECT_TRUE(pp.enabled);
  // No default frame on purpose — guessing the last link would estimate about
  // the wrong point rather than refusing.
  EXPECT_TRUE(pp.frame.empty());
  EXPECT_DOUBLE_EQ(pp.sigma0, 1e-3);
  EXPECT_DOUBLE_EQ(pp.lambda_max, 0.05);
  EXPECT_DOUBLE_EQ(pp.min_sigma, 1e-3);
  EXPECT_DOUBLE_EQ(pp.max_fit_error, 5.0);
  EXPECT_DOUBLE_EQ(pp.min_gravity, 1e-3);
  EXPECT_DOUBLE_EQ(pp.max_arm_velocity, 1e-3);
  EXPECT_DOUBLE_EQ(pp.max_peripheral_velocity, 1e-4);
  EXPECT_DOUBLE_EQ(pp.settle_time_constants, 5.0);
}

TEST(DemoSharedConfigTest, MomentumObserverBlockAbsentLeavesFlagFalse) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load("grasp_controller_type: force_pi\n");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_FALSE(cfg.momentum_observer.has_block);
  EXPECT_FALSE(cfg.momentum_observer.payload.has_block);
}

// Layer 1b behaviour must survive verbatim: an observer block with no
// payload_estimator sub-block is "residual only", not "payload with defaults".
TEST(DemoSharedConfigTest, MomentumObserverWithoutPayloadSubBlockStaysResidualOnly) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  enabled: true
  gains: 25.0
)YAML");
  ApplyDemoSharedConfig(node, cfg);
  EXPECT_TRUE(cfg.momentum_observer.has_block);
  EXPECT_TRUE(cfg.momentum_observer.enabled);
  // A scalar gain broadcasts to every joint, so it arrives as a one-entry list.
  ASSERT_EQ(cfg.momentum_observer.gains.size(), 1U);
  EXPECT_DOUBLE_EQ(cfg.momentum_observer.gains[0], 25.0);
  EXPECT_FALSE(cfg.momentum_observer.payload.has_block);
}

TEST(DemoSharedConfigTest, MomentumObserverGainListParsesPerJoint) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  gains: [11.0, 12.0, 13.0]
)YAML");
  ApplyDemoSharedConfig(node, cfg);
  ASSERT_EQ(cfg.momentum_observer.gains.size(), 3U);
  EXPECT_DOUBLE_EQ(cfg.momentum_observer.gains[0], 11.0);
  EXPECT_DOUBLE_EQ(cfg.momentum_observer.gains[1], 12.0);
  EXPECT_DOUBLE_EQ(cfg.momentum_observer.gains[2], 13.0);
}

TEST(DemoSharedConfigTest, MomentumObserverPayloadSubBlockParsesEveryKey) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  enabled: true
  gains: 30.0
  payload_estimator:
    enabled: false
    frame: "wrist_3_link"
    sigma0: 0.011
    lambda_max: 0.022
    min_sigma: 0.033
    max_fit_error: 0.044
    min_gravity: 0.055
    max_arm_velocity: 0.066
    max_peripheral_velocity: 0.077
    settle_time_constants: 0.088
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  const integrated_bringup::PayloadEstimatorParams& pp = cfg.momentum_observer.payload;
  EXPECT_TRUE(pp.has_block);
  // `enabled: false` with the block present is the shipped configuration, and
  // it must NOT read back as the struct default (true).
  EXPECT_FALSE(pp.enabled);
  EXPECT_EQ(pp.frame, "wrist_3_link");
  EXPECT_DOUBLE_EQ(pp.sigma0, 0.011);
  EXPECT_DOUBLE_EQ(pp.lambda_max, 0.022);
  EXPECT_DOUBLE_EQ(pp.min_sigma, 0.033);
  EXPECT_DOUBLE_EQ(pp.max_fit_error, 0.044);
  EXPECT_DOUBLE_EQ(pp.min_gravity, 0.055);
  EXPECT_DOUBLE_EQ(pp.max_arm_velocity, 0.066);
  EXPECT_DOUBLE_EQ(pp.max_peripheral_velocity, 0.077);
  EXPECT_DOUBLE_EQ(pp.settle_time_constants, 0.088);
}

// #455 Layer 2B — the `inertial` sub-sub-block. Nested under payload_estimator
// because it reuses that block's frame and velocity gates: it is the same
// quasi-static lane with a different estimator on it.
TEST(DemoSharedConfigTest, MomentumObserverInertialSubBlockParsesEveryKey) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  enabled: true
  gains: 30.0
  payload_estimator:
    enabled: true
    frame: "wrist_3_link"
    inertial:
      enabled: true
      forgetting_factor: 0.995
      min_param_sigma: 0.011
      min_mass: 0.022
      max_com_offset: 0.033
      max_inertia_column: 0.044
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  const integrated_bringup::InertialEstimatorParams& ip = cfg.momentum_observer.payload.inertial;
  EXPECT_TRUE(ip.has_block);
  // The struct default is DISABLED (unlike Layer 2A's true), so an `enabled:
  // true` that read back as the default would prove nothing — this asserts the
  // key was actually consumed.
  EXPECT_TRUE(ip.enabled);
  EXPECT_DOUBLE_EQ(ip.forgetting_factor, 0.995);
  EXPECT_DOUBLE_EQ(ip.min_param_sigma, 0.011);
  EXPECT_DOUBLE_EQ(ip.min_mass, 0.022);
  EXPECT_DOUBLE_EQ(ip.max_com_offset, 0.033);
  EXPECT_DOUBLE_EQ(ip.max_inertia_column, 0.044);
}

// Layer 2A without the nested block is Layer 2A exactly as it shipped — the 2B
// lane must not arm itself by inheriting its struct defaults.
TEST(DemoSharedConfigTest, MomentumObserverInertialAbsentLeavesLayer2BDisarmed) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  enabled: true
  gains: 30.0
  payload_estimator:
    enabled: true
    frame: "wrist_3_link"
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  EXPECT_TRUE(cfg.momentum_observer.payload.has_block);
  EXPECT_FALSE(cfg.momentum_observer.payload.inertial.has_block);
  EXPECT_FALSE(cfg.momentum_observer.payload.inertial.enabled);
}

// A partial block keeps defaults rather than zeroing them — same contract the
// 2A sub-block carries, and the one that makes a half-written YAML safe.
TEST(DemoSharedConfigTest, MomentumObserverInertialPartialBlockKeepsDefaults) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  gains: 30.0
  payload_estimator:
    frame: "wrist_3_link"
    inertial:
      min_mass: 0.5
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  const integrated_bringup::InertialEstimatorParams& ip = cfg.momentum_observer.payload.inertial;
  EXPECT_TRUE(ip.has_block);
  EXPECT_DOUBLE_EQ(ip.min_mass, 0.5);
  EXPECT_DOUBLE_EQ(ip.forgetting_factor, 1.0);
  EXPECT_DOUBLE_EQ(ip.max_com_offset, 0.5);
  EXPECT_DOUBLE_EQ(ip.max_inertia_column, 1.0e-6);
}

// Presence of the sub-block is what arms Layer 2A; an empty map still counts,
// and every unset key keeps its default rather than becoming zero.
TEST(DemoSharedConfigTest, MomentumObserverPayloadPartialBlockKeepsDefaults) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  payload_estimator:
    frame: "tool0"
)YAML");
  ApplyDemoSharedConfig(node, cfg);

  const integrated_bringup::PayloadEstimatorParams& pp = cfg.momentum_observer.payload;
  EXPECT_TRUE(pp.has_block);
  EXPECT_EQ(pp.frame, "tool0");
  EXPECT_TRUE(pp.enabled);
  EXPECT_DOUBLE_EQ(pp.max_fit_error, 5.0);
  EXPECT_DOUBLE_EQ(pp.settle_time_constants, 5.0);
}

// A sub-block that is present but not a map (a scalar typo, or a key left
// dangling with no value) must leave the params untouched rather than throwing
// mid-parse or half-applying.
TEST(DemoSharedConfigTest, MomentumObserverPayloadNonMapSubBlockIsIgnored) {
  DemoSharedConfig cfg;
  YAML::Node node = YAML::Load(R"YAML(
momentum_observer:
  payload_estimator: "yes"
)YAML");
  EXPECT_NO_THROW(ApplyDemoSharedConfig(node, cfg));
  EXPECT_FALSE(cfg.momentum_observer.payload.has_block);
}
