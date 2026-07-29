// ── G2 schema layer contract (#236 S7c-1) ───────────────────────────────────
//
// These drive the ParseXxxParams free functions DIRECTLY — no controller, no
// URDF, no model. That is the point twice over: it is what a future binding
// does, and it is what lets these cases outlive the adapters (S7c-2).
//
// Why this file had to exist before the extraction could be called verified.
// The gain-length rule ("a sequence whose length != nv is a fail-fast config
// error, not a silently truncated value" — issue #172) was, on paper, covered by
// PControllerConfig.LoadConfigWrongSizeKpThrows. It is not. That case feeds a
// sequence SHORTER than nv, and a short sequence makes yaml-cpp's own
// `TypedBadConversion` fire when index [i] past the end is converted — and
// YAML::Exception derives from std::runtime_error, which is exactly what the
// EXPECT_THROW matches. So the assertion passes with the explicit length check
// DELETED. Measured, not reasoned: removing the `kd` check from
// ParseJointPdParams left all 505 rtc_controllers cases green.
//
// The case the old fixture could not see is the OVER-LONG sequence: every index
// in [0, nv) exists, nothing throws from yaml-cpp, and the extra entries are
// dropped in silence — the precise #172 defect the check was added to prevent.
// That is what the cases below pin, on both sides (over-long AND exact), so a
// deleted length check fails here rather than passing everywhere.
//
// S7c-2 extends the same reasoning to the three compliance schemas. Their
// length checks are spelled `n.size() != 6` rather than `!= nv`, but they fail
// identically: with the check deleted, a SHORT sequence still throws (yaml-cpp
// converts a past-the-end index) and a scalar still throws (index [0] of a
// scalar), so every pre-existing mis-shape case stayed green. Only the OVER-LONG
// sequence discriminates, and none of the adapter suites had one.
#include "rtc_controllers/params/cascaded_compliance_params.hpp"
#include "rtc_controllers/params/clik_params.hpp"
#include "rtc_controllers/params/joint_pd_params.hpp"
#include "rtc_controllers/params/osc_params.hpp"
#include "rtc_controllers/params/task_admittance_params.hpp"
#include "rtc_controllers/params/task_impedance_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

using rtc::CommandType;
using rtc::params::CascadedComplianceConfig;
using rtc::params::CascadedComplianceParams;
using rtc::params::ClikParams;
using rtc::params::JointPdParams;
using rtc::params::OscParams;
using rtc::params::ParseCascadedComplianceParams;
using rtc::params::ParseClikParams;
using rtc::params::ParseJointPdParams;
using rtc::params::ParseOscParams;
using rtc::params::ParseTaskAdmittanceParams;
using rtc::params::ParseTaskImpedanceParams;
using rtc::params::TaskAdmittanceConfig;
using rtc::params::TaskAdmittanceParams;
using rtc::params::TaskImpedanceConfig;
using rtc::params::TaskImpedanceFormulation;
using rtc::params::TaskImpedanceParams;
using rtc::params::TaskSelection;

constexpr int kNv = 6;

// An *undefined* node — the "no YAML config at all" case the controller manager
// hands a controller. This is NOT the same as a default-constructed `YAML::Node`,
// which is a DEFINED Null: `operator bool` is true for it, so it sails past every
// `if (!cfg)` early return and runs the full key-absent parse. Only this reaches
// the early-return branch, and the two paths floor different things.
YAML::Node UndefinedNode() {
  return YAML::Node(YAML::NodeType::Undefined);
}

// ── joint PD: the gain-length rule, from both sides ─────────────────────────

TEST(JointPdSchema, RejectsAnOverLongKpSequence) {
  // The sensor the pre-existing suite lacked. Seven entries for a 6-DOF model:
  // yaml-cpp is perfectly happy (indices 0..5 all exist), so ONLY the explicit
  // length check stands between this config and a silently dropped seventh gain.
  JointPdParams p;
  auto ct = CommandType::kTorque;
  const YAML::Node cfg = YAML::Load("kp: [1, 2, 3, 4, 5, 6, 7]");
  EXPECT_THROW(ParseJointPdParams(cfg, kNv, p, ct), std::runtime_error);
}

TEST(JointPdSchema, RejectsAnOverLongKdSequence) {
  JointPdParams p;
  auto ct = CommandType::kTorque;
  const YAML::Node cfg = YAML::Load("kd: [1, 2, 3, 4, 5, 6, 7]");
  EXPECT_THROW(ParseJointPdParams(cfg, kNv, p, ct), std::runtime_error);
}

TEST(JointPdSchema, RejectsAShortSequenceForTheRightReason) {
  // Kept for completeness, but note it is NOT the discriminating case: a short
  // sequence also throws via yaml-cpp when the length check is absent. The
  // over-long cases above are what make the check observable.
  JointPdParams p;
  auto ct = CommandType::kTorque;
  const YAML::Node cfg = YAML::Load("kp: [1, 2, 3]");
  EXPECT_THROW(ParseJointPdParams(cfg, kNv, p, ct), std::runtime_error);
}

TEST(JointPdSchema, AcceptsAndAppliesAnExactLengthSequence) {
  // Non-vacuity for the three above: a length rule that rejected everything
  // would satisfy them just as well.
  JointPdParams p;
  auto ct = CommandType::kTorque;
  const YAML::Node cfg = YAML::Load("kp: [1, 2, 3, 4, 5, 6]\nkd: [7, 8, 9, 10, 11, 12]");
  ASSERT_NO_THROW(ParseJointPdParams(cfg, kNv, p, ct));
  for (int i = 0; i < kNv; ++i) {
    EXPECT_DOUBLE_EQ(p.kp[static_cast<std::size_t>(i)], i + 1) << "kp[" << i << "]";
    EXPECT_DOUBLE_EQ(p.kd[static_cast<std::size_t>(i)], i + 7) << "kd[" << i << "]";
  }
}

TEST(JointPdSchema, DynamicsCompensationRequiresATorqueCommand) {
  // The cross-FIELD rule: g and C·v are N·m and cannot be added to a
  // position command. It is evaluated against the command type that will
  // actually be in force, so the key order inside the node must not matter.
  JointPdParams p;
  auto ct = CommandType::kTorque;
  EXPECT_THROW(
      ParseJointPdParams(YAML::Load("enable_gravity_compensation: true\ncommand_type: position"),
                         kNv, p, ct),
      std::runtime_error);

  JointPdParams q;
  auto ct2 = CommandType::kTorque;
  EXPECT_NO_THROW(ParseJointPdParams(
      YAML::Load("enable_gravity_compensation: true\ncommand_type: torque"), kNv, q, ct2));
  EXPECT_EQ(ct2, CommandType::kTorque);
}

// ── pre-migrated from test_controller_config.cpp (#298 S7c-2 3단계) ──────────
//
// The cases below were driven through JointPDController::LoadConfig / the CLIK
// and OSC equivalents. Every one of them asserts something the SCHEMA owns, so
// re-pointing them at the Parse free function is a change of driver, not of
// contract — and it is what lets them outlive the adapter that is about to be
// deleted. The halves that assert ADAPTER behaviour (set_gains() re-flooring,
// get_gains() round-tripping, OnDeviceConfigsSet, the trajectory split) are not
// migrated: they retire with their subject, with provenance in the delete
// commit.

TEST(JointPdSchema, ParsesEveryScalarAndFlagItIsGiven) {
  // From JointPDConfig.LoadConfigParsesGainsAndFlags. This is the non-vacuity
  // floor under every rejection case above: a parser that dropped the node
  // wholesale would satisfy all of them.
  JointPdParams p;
  auto ct = CommandType::kPosition;
  ASSERT_NO_THROW(ParseJointPdParams(YAML::Load(R"(
kp: [1, 2, 3, 4, 5, 6]
kd: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
enable_gravity_compensation: true
enable_coriolis_compensation: true
trajectory_speed: 2.5
command_type: torque
)"),
                                     kNv, p, ct));
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(p.kp[i], static_cast<double>(i + 1), 1e-12) << "kp[" << i << "]";
    EXPECT_NEAR(p.kd[i], 0.1 * static_cast<double>(i + 1), 1e-12) << "kd[" << i << "]";
  }
  EXPECT_TRUE(p.enable_gravity_compensation);
  EXPECT_TRUE(p.enable_coriolis_compensation);
  EXPECT_DOUBLE_EQ(p.trajectory_speed, 2.5);
  EXPECT_EQ(ct, CommandType::kTorque);
}

TEST(JointPdSchema, FloorsTheTrajectorySpeed) {
  // From JointPDConfig.LoadConfigClampsTrajectorySpeedFloor. The law divides by
  // this to get a segment duration, so a configured 0 is a division by zero, not
  // "as fast as possible".
  JointPdParams p;
  auto ct = CommandType::kTorque;
  ASSERT_NO_THROW(ParseJointPdParams(YAML::Load("trajectory_speed: 0.0"), kNv, p, ct));
  EXPECT_GE(p.trajectory_speed, 1e-6);
}

TEST(JointPdSchema, AnUndefinedNodeLeavesEveryFieldAlone) {
  // From JointPDConfig.LoadConfigNullNodeKeepsDefaults. Seeded with non-default
  // values so "kept the defaults" cannot pass by the parser writing defaults
  // back over them.
  JointPdParams p;
  p.kp[0] = 7.5;
  p.trajectory_speed = 3.0;
  auto ct = CommandType::kPosition;
  ASSERT_NO_THROW(ParseJointPdParams(UndefinedNode(), kNv, p, ct));
  EXPECT_DOUBLE_EQ(p.kp[0], 7.5);
  EXPECT_DOUBLE_EQ(p.trajectory_speed, 3.0);
  EXPECT_EQ(ct, CommandType::kPosition) << "a no-op parse must not rewrite the command type";
}

// ── OSC / CLIK: the guards that must survive the move ───────────────────────

TEST(OscSchema, RejectsANonTorqueCommandType) {
  OscParams p;
  auto ct = CommandType::kTorque;
  EXPECT_THROW(ParseOscParams(YAML::Load("command_type: position"), p, ct), std::runtime_error);
}

TEST(OscSchema, FloorsTheSingularityGuardsAndPostureGains) {
  // NUM-1 / NUM-2 / #277: a zero or negative value here would remove the guard
  // rather than widen it. Floored on the way in, not at the point of use only.
  OscParams p;
  auto ct = CommandType::kTorque;
  ASSERT_NO_THROW(ParseOscParams(
      YAML::Load("max_damping: 0.0\nsingularity_threshold: -1.0\nnull_kp: -5.0\nnull_kd: -2.0"), p,
      ct));
  EXPECT_GT(p.max_damping, 0.0) << "λ_max floor removed — a singular Λ⁻¹ would admit NaN torque";
  EXPECT_GT(p.singularity_threshold, 0.0) << "σ₀ floor removed — the §6.5 ramp is gone, not narrow";
  EXPECT_DOUBLE_EQ(p.null_kp, 0.0) << "a negative posture stiffness pushes AWAY from safe_position";
  EXPECT_DOUBLE_EQ(p.null_kd, 0.0) << "a negative posture damping injects energy";
}

TEST(OscSchema, FloorsThePostureGainsEvenWithNoNodeAtAll) {
  // NUM-6's loader half is "regardless of whether the key is present", and an
  // absent NODE is the widest case of that — the value then comes from the
  // constructor or a previous set_gains(), the two paths the floor exists for.
  OscParams p;
  p.null_kp = -3.0;
  p.null_kd = -1.0;
  auto ct = CommandType::kTorque;
  ASSERT_NO_THROW(ParseOscParams(YAML::Node(), p, ct));
  EXPECT_DOUBLE_EQ(p.null_kp, 0.0);
  EXPECT_DOUBLE_EQ(p.null_kd, 0.0);
}

TEST(OscSchema, ReportsTheRetiredDampingKeyInsteadOfSilentlyIgnoringIt) {
  // LoadConfig ignores unknown keys, so a deployed config carrying the retired
  // `damping` would keep parsing clean while its singularity behaviour changed
  // underneath it (#236 S2b). The parse layer is rclcpp-free, so it REPORTS.
  OscParams p;
  auto ct = CommandType::kTorque;
  rtc::params::OscRetiredKeys retired;
  ASSERT_NO_THROW(ParseOscParams(YAML::Load("damping: 0.1"), p, ct, &retired));
  EXPECT_TRUE(retired.damping);

  rtc::params::OscRetiredKeys none;
  ASSERT_NO_THROW(ParseOscParams(YAML::Load("max_damping: 0.1"), p, ct, &none));
  EXPECT_FALSE(none.damping) << "a config using the CURRENT key must not be reported as retired";

  // From OscConfig.RetiredDampingKeyIsIgnoredNotMapped: reporting is not the
  // whole rule — the retired key must not be quietly aliased onto the ramp
  // ceiling either. A constant λ and λ_max are different quantities.
  OscParams q;
  const double before_max = q.max_damping;
  const double before_sigma = q.singularity_threshold;
  auto ct2 = CommandType::kTorque;
  ASSERT_NO_THROW(ParseOscParams(YAML::Load("damping: 0.01"), q, ct2));
  EXPECT_DOUBLE_EQ(q.max_damping, before_max);
  EXPECT_DOUBLE_EQ(q.singularity_threshold, before_sigma);
}

TEST(OscSchema, ParsesEveryGainItIsGiven) {
  // From OscConfig.LoadConfigParsesAllGains — the non-vacuity floor under the
  // floor/reject cases above.
  OscParams p;
  auto ct = CommandType::kTorque;
  ASSERT_NO_THROW(ParseOscParams(YAML::Load(R"(
kp_pos: [3.0, 3.0, 3.0]
kd_pos: [0.2, 0.2, 0.2]
kp_rot: [1.0, 1.0, 1.0]
kd_rot: [0.1, 0.1, 0.1]
max_damping: 0.05
singularity_threshold: 0.03
enable_gravity_compensation: true
trajectory_speed: 0.15
trajectory_angular_speed: 0.7
max_traj_velocity: 0.6
max_traj_angular_velocity: 1.2
command_type: torque
)"),
                                 p, ct));
  EXPECT_NEAR(p.kp_pos[0], 3.0, 1e-12);
  EXPECT_NEAR(p.kd_pos[1], 0.2, 1e-12);
  EXPECT_NEAR(p.kp_rot[2], 1.0, 1e-12);
  EXPECT_NEAR(p.kd_rot[0], 0.1, 1e-12);
  EXPECT_NEAR(p.max_damping, 0.05, 1e-12);
  EXPECT_NEAR(p.singularity_threshold, 0.03, 1e-12);
  EXPECT_TRUE(p.enable_gravity_compensation);
  EXPECT_NEAR(p.trajectory_speed, 0.15, 1e-12);
  EXPECT_NEAR(p.trajectory_angular_speed, 0.7, 1e-12);
  EXPECT_NEAR(p.max_traj_velocity, 0.6, 1e-12);
  EXPECT_NEAR(p.max_traj_angular_velocity, 1.2, 1e-12);
  EXPECT_EQ(ct, CommandType::kTorque);
}

TEST(OscSchema, AnUndefinedNodeKeepsTheRestButStillFloorsThePostureGains) {
  // From OscConfig.LoadConfigNullNodeKeepsDefaults and the closing half of
  // OscConfig.PostureGainsAreFlooredAndTheFloorSurvivesSetGains.
  //
  // This is the branch FloorsThePostureGainsEvenWithNoNodeAtAll does NOT reach:
  // that one passes `YAML::Node()`, a DEFINED Null, which runs the full parse
  // with every key absent. An UNDEFINED node takes the `if (!cfg)` early return
  // instead, and a floor written only below that return leaves a negative gain
  // in the POD — reported by get_gains(), refused by the law.
  OscParams p;
  p.null_kp = -8.0;
  p.null_kd = -1.0;
  p.max_damping = 0.077;
  p.trajectory_speed = 0.33;
  auto ct = CommandType::kTorque;
  ASSERT_NO_THROW(ParseOscParams(UndefinedNode(), p, ct));
  EXPECT_DOUBLE_EQ(p.null_kp, 0.0) << "the `if (!cfg)` early return skipped the posture floor";
  EXPECT_DOUBLE_EQ(p.null_kd, 0.0);
  EXPECT_DOUBLE_EQ(p.max_damping, 0.077) << "an absent node must not reset the other fields";
  EXPECT_DOUBLE_EQ(p.trajectory_speed, 0.33);
}

TEST(ClikSchema, FloorsTheSingularityGuardsAndPostureGain) {
  ClikParams p;
  auto ct = CommandType::kPosition;
  ASSERT_NO_THROW(ParseClikParams(
      YAML::Load("max_damping: 0.0\nsingularity_threshold: 0.0\nnull_kp: -1.0"), p, ct));
  EXPECT_GT(p.max_damping, 0.0);
  EXPECT_GT(p.singularity_threshold, 0.0);
  EXPECT_DOUBLE_EQ(p.null_kp, 0.0);

  // From ClikConfig.LoadConfigFloorsBothEndsOfTheDlsRamp: negative is the same
  // failure with a sign, because AdaptiveDampingSquared's short-circuit is
  // `sigma0 <= 0` — zero alone does not discriminate a `> 0` guard from a `>= 0`
  // one.
  ASSERT_NO_THROW(ParseClikParams(YAML::Load("singularity_threshold: -1.0"), p, ct));
  EXPECT_GT(p.singularity_threshold, 0.0);

  // From ClikConfig.NegativePostureGainIsFlooredAndTheFloorSurvivesSetGains
  // (its schema half): a floor, not a rewrite. Without this a parser that
  // hard-zeroed null_kp would satisfy the assertion above.
  ASSERT_NO_THROW(ParseClikParams(YAML::Load("null_kp: 0.8"), p, ct));
  EXPECT_DOUBLE_EQ(p.null_kp, 0.8);
}

TEST(ClikSchema, ParsesEveryGainItIsGiven) {
  // From ClikConfig.LoadConfigParsesAllGains.
  ClikParams p;
  auto ct = CommandType::kTorque;
  ASSERT_NO_THROW(ParseClikParams(YAML::Load(R"(
kp_translation: [2.0, 3.0, 4.0]
kp_rotation: [1.1, 1.2, 1.3]
max_damping: 0.02
singularity_threshold: 0.03
null_kp: 0.8
enable_null_space: true
control_6dof: true
trajectory_speed: 0.2
trajectory_angular_speed: 0.6
max_traj_velocity: 0.8
max_traj_angular_velocity: 1.5
command_type: position
)"),
                                  p, ct));
  EXPECT_NEAR(p.kp_translation[0], 2.0, 1e-12);
  EXPECT_NEAR(p.kp_translation[2], 4.0, 1e-12);
  EXPECT_NEAR(p.kp_rotation[1], 1.2, 1e-12);
  EXPECT_NEAR(p.max_damping, 0.02, 1e-12);
  EXPECT_NEAR(p.singularity_threshold, 0.03, 1e-12);
  EXPECT_NEAR(p.null_kp, 0.8, 1e-12);
  EXPECT_TRUE(p.enable_null_space);
  EXPECT_TRUE(p.control_6dof);
  EXPECT_NEAR(p.trajectory_speed, 0.2, 1e-12);
  EXPECT_NEAR(p.trajectory_angular_speed, 0.6, 1e-12);
  EXPECT_NEAR(p.max_traj_velocity, 0.8, 1e-12);
  EXPECT_NEAR(p.max_traj_angular_velocity, 1.5, 1e-12);
  // Unlike OSC this law runs in either domain, so the key decides and nothing is
  // rejected.
  EXPECT_EQ(ct, CommandType::kPosition);
}

TEST(ClikSchema, AnUndefinedNodeKeepsTheRestButStillFloorsThePostureGain) {
  // From ClikConfig.LoadConfigNullNodeKeepsDefaults and the closing half of
  // ClikConfig.NegativePostureGainIsFlooredAndTheFloorSurvivesSetGains. See the
  // OSC twin for why an UNDEFINED node is a different branch from `YAML::Node()`.
  ClikParams p;
  p.null_kp = -0.8;
  p.max_damping = 0.077;
  p.singularity_threshold = 0.055;
  auto ct = CommandType::kPosition;
  ASSERT_NO_THROW(ParseClikParams(UndefinedNode(), p, ct));
  EXPECT_DOUBLE_EQ(p.null_kp, 0.0) << "the `if (!cfg)` early return skipped the posture floor";
  EXPECT_DOUBLE_EQ(p.max_damping, 0.077) << "an absent node must not reset the other fields";
  EXPECT_DOUBLE_EQ(p.singularity_threshold, 0.055);
}

TEST(ClikSchema, ReportsTheRetiredDampingKey) {
  ClikParams p;
  auto ct = CommandType::kPosition;
  rtc::params::ClikRetiredKeys retired;
  ASSERT_NO_THROW(ParseClikParams(YAML::Load("damping: 0.1"), p, ct, &retired));
  EXPECT_TRUE(retired.damping);

  // From ClikConfig.RetiredDampingKeyIsIgnoredNotMapped: reported, and not
  // aliased onto the ramp ceiling behind the report.
  ClikParams q;
  const double before_max = q.max_damping;
  const double before_sigma = q.singularity_threshold;
  auto ct2 = CommandType::kPosition;
  ASSERT_NO_THROW(ParseClikParams(YAML::Load("damping: 0.01"), q, ct2));
  EXPECT_DOUBLE_EQ(q.max_damping, before_max);
  EXPECT_DOUBLE_EQ(q.singularity_threshold, before_sigma);
}

// ── compliance 3종 (S7c-2) ──────────────────────────────────────────────────
//
// The message of whatever the parser throws, or "" if it accepts the node.
// Matching on the message matters for the same reason it does in the adapter
// suites: yaml-cpp's own conversion failure also derives from
// std::runtime_error, so a bare EXPECT_THROW passes even when the value never
// reached the check under test.
template <typename Fn>
std::string ParseError(Fn&& parse) {
  try {
    parse();
  } catch (const std::exception& e) {
    return e.what();
  }
  return {};
}

bool Mentions(const std::string& msg, const std::string& needle) {
  return msg.find(needle) != std::string::npos;
}

std::string ImpedanceError(const std::string& yaml, TaskSelection sel = TaskSelection::kFullSe3) {
  return ParseError([&] {
    TaskImpedanceParams p;
    TaskImpedanceConfig c;
    ParseTaskImpedanceParams(YAML::Load(yaml), p, sel, c);
  });
}

std::string AdmittanceError(const std::string& yaml) {
  return ParseError([&] {
    TaskAdmittanceParams p;
    TaskAdmittanceConfig c;
    ParseTaskAdmittanceParams(YAML::Load(yaml), p, c);
  });
}

std::string CascadeError(const std::string& yaml) {
  return ParseError([&] {
    CascadedComplianceParams p;
    CascadedComplianceConfig c;
    ParseCascadedComplianceParams(YAML::Load(yaml), p, c);
  });
}

// ── task impedance ─────────────────────────────────────────────────────────

TEST(TaskImpedanceSchema, RejectsAnOverLongDesiredInertiaSequence) {
  // The discriminating shape (see the file header): seven entries for a 6-D Λ_d.
  // Indices 0..5 all exist, so yaml-cpp throws nothing and the explicit length
  // check is the only thing between this file and a silently dropped entry.
  const auto msg = ImpedanceError("desired_inertia: [1, 2, 3, 4, 5, 6, 7]");
  EXPECT_TRUE(Mentions(msg, "desired_inertia")) << msg;
  EXPECT_TRUE(Mentions(msg, "6 entries")) << msg;
}

TEST(TaskImpedanceSchema, RejectsANonSpdDesiredInertia) {
  // NUM-2: Λ_d⁻¹ is formed every tick, so a zero or negative entry is a divide,
  // not a tuning mistake a clamp can absorb.
  EXPECT_TRUE(Mentions(ImpedanceError("desired_inertia: [1, 0, 1, 1, 1, 1]"), "must be > 0"));
  EXPECT_TRUE(Mentions(ImpedanceError("desired_inertia: [1, -1, 1, 1, 1, 1]"), "must be > 0"));
  // Non-vacuity: a well-shaped SPD Λ_d is accepted AND applied, and it is what
  // turns §5.2's "off unless asked for" default off.
  TaskImpedanceParams p;
  TaskImpedanceConfig c;
  ASSERT_NO_THROW(ParseTaskImpedanceParams(YAML::Load("desired_inertia: [1, 2, 3, 4, 5, 6]"), p,
                                           TaskSelection::kFullSe3, c));
  EXPECT_DOUBLE_EQ(p.inertia.desired_inertia[5], 6.0);
  EXPECT_FALSE(p.inertia.desired_inertia_natural);
}

TEST(TaskImpedanceSchema, TranslationOnlyRequiresPostureStiffness) {
  // §6.1: TRANSLATION_ONLY leaves orientation to the null space, so a zero
  // posture stiffness leaves rotation with no authority at all.
  EXPECT_TRUE(Mentions(ImpedanceError("nullspace_stiffness: 0.0", TaskSelection::kTranslationOnly),
                       "§6.1"));
  // And the floor runs FIRST, so a negative gain reaches the guard as 0 rather
  // than walking through a `kp == 0.0` comparison (#277).
  EXPECT_TRUE(Mentions(
      ImpedanceError("nullspace_stiffness: -30.0", TaskSelection::kTranslationOnly), "§6.1"));
  // FULL_SE3 is unaffected — orientation is a task DoF there.
  EXPECT_EQ(ImpedanceError("nullspace_stiffness: 0.0", TaskSelection::kFullSe3), "");
}

TEST(TaskImpedanceSchema, FloorsThePostureGainsBeforeTheDampingCorrection) {
  // The ORDER is the contract: §6.4's correction is `if (kp > 0.0)`, so an
  // unfloored negative K_pⁿ would skip the damping floor entirely.
  TaskImpedanceParams p;
  TaskImpedanceConfig c;
  ASSERT_NO_THROW(
      ParseTaskImpedanceParams(YAML::Load("nullspace_stiffness: -30.0\nnullspace_damping: -1.0"), p,
                               TaskSelection::kFullSe3, c));
  EXPECT_DOUBLE_EQ(p.nullspace_kp, 0.0);
  EXPECT_DOUBLE_EQ(p.nullspace_kd, 0.0);

  TaskImpedanceParams q;
  ASSERT_NO_THROW(
      ParseTaskImpedanceParams(YAML::Load("nullspace_stiffness: 25.0\nnullspace_damping: 0.1"), q,
                               TaskSelection::kFullSe3, c));
  EXPECT_DOUBLE_EQ(q.nullspace_kd, 2.0 * std::sqrt(25.0)) << "§6.4 floor K_dⁿ ≥ 2√K_pⁿ";
}

TEST(TaskImpedanceSchema, FormulationIsADeclaredChoiceAndAnUnknownOneIsRejected) {
  TaskImpedanceParams p;
  TaskImpedanceConfig c;
  ASSERT_NO_THROW(ParseTaskImpedanceParams(YAML::Load("formulation: inertia_shaping"), p,
                                           TaskSelection::kFullSe3, c));
  EXPECT_EQ(c.formulation, TaskImpedanceFormulation::kInertiaShaping);
  ASSERT_NO_THROW(ParseTaskImpedanceParams(YAML::Load("formulation: jacobian_transpose"), p,
                                           TaskSelection::kFullSe3, c));
  EXPECT_EQ(c.formulation, TaskImpedanceFormulation::kJacobianTranspose);
  EXPECT_TRUE(Mentions(ImpedanceError("formulation: shape_it_please"), "formulation must be"));
}

TEST(TaskImpedanceSchema, RejectsANonTorqueCommandType) {
  EXPECT_TRUE(Mentions(ImpedanceError("command_type: position"), "command_type"));
}

TEST(TaskImpedanceSchema, ARejectedParseLeavesTheNonGainConfigUntouched) {
  // The #172 contract at the parse layer: everything that can throw runs BEFORE
  // `config` is written, so a caller that keeps its live wiring in a
  // TaskImpedanceConfig can retry a bad file without half-applying it.
  TaskImpedanceParams p;
  TaskImpedanceConfig c;
  c.sensor_frame = "live_frame";
  c.wrench_enabled = true;
  EXPECT_THROW(ParseTaskImpedanceParams(
                   YAML::Load("external_wrench:\n  sensor_frame: new_frame\n  enabled: false\n"
                              "command_type: position\n"),
                   p, TaskSelection::kFullSe3, c),
               std::runtime_error);
  EXPECT_EQ(c.sensor_frame, "live_frame") << "a rejected parse rewrote the caller's frame";
  EXPECT_TRUE(c.wrench_enabled) << "a rejected parse rewrote the caller's wrench wiring";
}

TEST(TaskImpedanceSchema, AnUndefinedNodeFloorsTheGainsAndLeavesTheConfigAlone) {
  // NUM-6's loader half, and the reason the owner skips the frame lookup on this
  // path: none of the wiring keys were read, which is not the same as reading
  // them as their defaults.
  //
  // `YAML::NodeType::Undefined` and NOT a default-constructed `YAML::Node`: the
  // latter is a DEFINED Null (`IsDefined()` is true when `m_pNode` is null), so
  // it walks the FULL parse with every key absent and would pin the other branch
  // — same helper, same reason, as test_task_impedance_controller.cpp.
  TaskImpedanceParams p;
  p.nullspace_kp = -3.0;
  p.nullspace_kd = -1.0;
  TaskImpedanceConfig c;
  c.sensor_frame = "live_frame";
  ASSERT_NO_THROW(ParseTaskImpedanceParams(YAML::Node(YAML::NodeType::Undefined), p,
                                           TaskSelection::kFullSe3, c));
  EXPECT_DOUBLE_EQ(p.nullspace_kp, 0.0);
  EXPECT_DOUBLE_EQ(p.nullspace_kd, 0.0);
  EXPECT_EQ(c.sensor_frame, "live_frame");

  // The §6.1 guard runs on this path too, and the floor above it means a
  // negative posture stiffness reaches it as 0 rather than slipping past a
  // `kp == 0.0` comparison.
  TaskImpedanceParams q;
  q.nullspace_kp = -30.0;
  EXPECT_THROW(ParseTaskImpedanceParams(YAML::Node(YAML::NodeType::Undefined), q,
                                        TaskSelection::kTranslationOnly, c),
               std::runtime_error);
  EXPECT_DOUBLE_EQ(q.nullspace_kp, 0.0)
      << "the floor must be written back even when the guard then rejects the configuration — "
         "the owner commits these gains on the throwing path";
}

// ── task admittance ────────────────────────────────────────────────────────

TEST(TaskAdmittanceSchema, RejectsOverLongSequences) {
  // The discriminating shape on both widths this schema uses.
  EXPECT_TRUE(Mentions(AdmittanceError("desired_inertia: [1, 2, 3, 4, 5, 6, 7]"), "6-entry"));
  EXPECT_TRUE(Mentions(AdmittanceError("ik_kp_pos: [1, 2, 3, 4]"), "3-entry"));
  EXPECT_TRUE(
      Mentions(AdmittanceError("min_desired_inertia: [1, 2, 3]"), "min_desired_inertia must be"));
}

TEST(TaskAdmittanceSchema, RejectsAMisshapedIkGainInsteadOfIgnoringIt) {
  // `ik_kp_pos: 0.0` is the obvious way to write the §7.3 pure-feedforward
  // experiment; dropped silently it ran as a CLIK variant instead, with nothing
  // in the diagnostics to say so.
  EXPECT_TRUE(Mentions(AdmittanceError("ik_kp_pos: 0.0"), "ik_kp_pos"));
  // Non-vacuity: the well-shaped spelling is accepted and applied.
  TaskAdmittanceParams p;
  TaskAdmittanceConfig c;
  ASSERT_NO_THROW(ParseTaskAdmittanceParams(YAML::Load("ik_kp_pos: [0.0, 0.0, 0.0]"), p, c));
  EXPECT_DOUBLE_EQ(p.ik_kp_pos[0], 0.0);
}

TEST(TaskAdmittanceSchema, RejectsANonPositivePoseErrorLimit) {
  // Compared every tick against a CRITICAL fault, so 0 latches SAFE_STOP on the
  // first tick with no cause field pointing at the config (D6).
  EXPECT_TRUE(Mentions(AdmittanceError("pose_error_limit: 0.0"), "pose_error_limit"));
  EXPECT_TRUE(Mentions(AdmittanceError("pose_error_limit: -1.0"), "pose_error_limit"));
}

TEST(TaskAdmittanceSchema, RequiresAWrenchSourceAndAPositionCommand) {
  // §7.1: admittance takes force as its INPUT — without one it degenerates into
  // an expensive position hold, which is a configure error and not a fallback.
  EXPECT_TRUE(Mentions(AdmittanceError("external_wrench:\n  enabled: false\n"), "§7.1"));
  EXPECT_TRUE(Mentions(AdmittanceError("command_type: torque"), "command_type"));

  TaskAdmittanceParams p;
  TaskAdmittanceConfig c;
  ASSERT_NO_THROW(
      ParseTaskAdmittanceParams(YAML::Load("external_wrench:\n  sensor_frame: tool0\n"), p, c));
  EXPECT_EQ(c.sensor_frame, "tool0") << "the frame must leave the parse layer UNRESOLVED";
  EXPECT_EQ(c.command_type, CommandType::kPosition);
}

// ── cascaded compliance ────────────────────────────────────────────────────

TEST(CascadedComplianceSchema, RejectsOverLongSequences) {
  // The discriminating shape, on the nested sections where a dropped entry is
  // hardest to notice: `outer.stiffness` is the gain an operator tunes to make
  // the arm yield, and a silently truncated one leaves the default pushing back.
  EXPECT_TRUE(
      Mentions(CascadeError("outer:\n  stiffness: [1, 2, 3, 4, 5, 6, 7]\n"), "outer.stiffness"));
  EXPECT_TRUE(Mentions(CascadeError("inner:\n  kp_pos: [1, 2, 3, 4]\n"), "inner.kp_pos"));
  EXPECT_TRUE(Mentions(CascadeError("external_wrench:\n  deadband: [1, 1, 1, 1, 1, 1, 1]\n"),
                       "external_wrench.deadband"));
  // Non-vacuity: the exact widths are accepted AND applied.
  CascadedComplianceParams p;
  CascadedComplianceConfig c;
  ASSERT_NO_THROW(ParseCascadedComplianceParams(
      YAML::Load("inner:\n  kp_pos: [11, 22, 33]\nouter:\n  stiffness: [1, 2, 3, 4, 5, 6]\n"), p,
      c));
  EXPECT_DOUBLE_EQ(p.impedance.kp_pos[1], 22.0);
  EXPECT_DOUBLE_EQ(p.admittance.stiffness[5], 6.0);
}

TEST(CascadedComplianceSchema, RejectsASectionThatIsPresentButNotAMap) {
  // One level up from the leaf shape check: a whole section skipped leaves every
  // key under it at its default while the file says otherwise.
  EXPECT_TRUE(Mentions(CascadeError("outer: 3.0\n"), "outer must be a map"));
  EXPECT_TRUE(Mentions(CascadeError("inner: [1.0, 2.0]\n"), "inner must be a map"));
  EXPECT_TRUE(Mentions(CascadeError("external_wrench: 3.0\n"), "external_wrench must be a map"));
  // An ABSENT section stays legal — the defaults ARE the documented behaviour.
  EXPECT_EQ(CascadeError("nullspace_stiffness: 1.0\n"), "");
}

TEST(CascadedComplianceSchema, RejectsNonFiniteScalars) {
  // `std::max(0.0, NaN)` returns 0.0 — silently a third value — and an infinite
  // gain latches SAFE_STOP with `nan_inf` on the first tick with nothing
  // pointing at the config line.
  EXPECT_TRUE(Mentions(CascadeError("outer:\n  stiffness: [.nan, 0, 0, 0, 0, 0]\n"), "finite"));
  EXPECT_TRUE(Mentions(CascadeError("estop_damping: .nan\n"), "estop_damping"));
  // `.inf` and not an overflowing literal: yaml-cpp rejects the latter itself,
  // which would make this pass without the check ever running.
  EXPECT_TRUE(Mentions(CascadeError("outer:\n  damping: [.inf, 1, 1, 1, 1, 1]\n"), "finite"));
}

TEST(CascadedComplianceSchema, RejectsThresholdsThatWouldLatchOnTheFirstTick) {
  EXPECT_TRUE(Mentions(CascadeError("pose_error_limit: 0.0\n"), "pose_error_limit"));
  EXPECT_TRUE(Mentions(CascadeError("max_torque_rate: 0.0\n"), "max_torque_rate"));
  EXPECT_TRUE(
      Mentions(CascadeError("outer:\n  desired_inertia: [1, 0, 1, 1, 1, 1]\n"), "must be > 0"));
  EXPECT_TRUE(Mentions(CascadeError("inner:\n  kp_pos: [10, -1, 10]\n"), "must be >= 0"));
}

TEST(CascadedComplianceSchema, RequiresAnExternalWrenchSourceAndATorqueCommand) {
  EXPECT_TRUE(Mentions(CascadeError("external_wrench:\n  enabled: false\n"), "§7.6"));
  EXPECT_TRUE(Mentions(CascadeError("command_type: position\n"), "command_type"));

  CascadedComplianceParams p;
  CascadedComplianceConfig c;
  ASSERT_NO_THROW(ParseCascadedComplianceParams(
      YAML::Load("external_wrench:\n  sensor_frame: ft_sensor\n"), p, c));
  EXPECT_EQ(c.sensor_frame, "ft_sensor") << "the frame must leave the parse layer UNRESOLVED";
  EXPECT_EQ(c.command_type, CommandType::kTorque);
}

}  // namespace
