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
#include "rtc_controllers/params/clik_params.hpp"
#include "rtc_controllers/params/joint_pd_params.hpp"
#include "rtc_controllers/params/osc_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace {

using rtc::CommandType;
using rtc::params::ClikParams;
using rtc::params::JointPdParams;
using rtc::params::OscParams;
using rtc::params::ParseClikParams;
using rtc::params::ParseJointPdParams;
using rtc::params::ParseOscParams;

constexpr int kNv = 6;

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
}

TEST(ClikSchema, FloorsTheSingularityGuardsAndPostureGain) {
  ClikParams p;
  auto ct = CommandType::kPosition;
  ASSERT_NO_THROW(ParseClikParams(
      YAML::Load("max_damping: 0.0\nsingularity_threshold: 0.0\nnull_kp: -1.0"), p, ct));
  EXPECT_GT(p.max_damping, 0.0);
  EXPECT_GT(p.singularity_threshold, 0.0);
  EXPECT_DOUBLE_EQ(p.null_kp, 0.0);
}

TEST(ClikSchema, ReportsTheRetiredDampingKey) {
  ClikParams p;
  auto ct = CommandType::kPosition;
  rtc::params::ClikRetiredKeys retired;
  ASSERT_NO_THROW(ParseClikParams(YAML::Load("damping: 0.1"), p, ct, &retired));
  EXPECT_TRUE(retired.damping);
}

}  // namespace
