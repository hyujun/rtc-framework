// ── position_margin vs the CLIK position box (#473) ──────────────────────────
//
// `integration.position_margin` is applied to BOTH ends of every joint's raw
// limits, so a joint narrower than twice the margin comes out inverted
// (q_min > q_max). The refusal itself is #468's and is not what this file is
// about — CLIK is the sole position backbone, so absorbing an inverted box
// would ship a joint with no position bound at all.
//
// What IS asserted here is ATTRIBUTION. The box is assembled in the binding's
// LoadConfig, from a key the operator wrote and limits the model owns; the
// consumer that used to refuse it (`ClikReferenceGenerator::Init`) sees neither
// — only two post-margin numbers and an index. So the diagnosis has to happen
// where the inputs are, and this file pins that it does, and that the message
// carries all four things the operator needs: the joint's NAME, its raw range,
// the margin that was applied, and the largest margin that would fit.
//
// THE FIXTURE IS DERIVED FROM THE MODEL, not written down. The margin that
// inverts a joint depends on that joint's width, so the test reads the URDF the
// controller reads, finds the narrowest joint, and builds the two cases around
// it. A hardcoded margin would silently stop inverting anything the day a limit
// changes, and a hardcoded joint name would pin this test to one robot.
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <limits>
#include <memory>
#include <string>

namespace {

using integrated_bringup::DemoWbcController;

/// The narrowest joint of the model the controller loads, as (name, width).
/// Only finite spans count: a continuous joint's limits are ±inf and no margin
/// can invert it, so including them would make `narrowest` meaningless.
struct NarrowestJoint {
  std::string name;
  double width{std::numeric_limits<double>::infinity()};
};

NarrowestJoint FindNarrowestJoint() {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(RTC_UR5E_URDF_PATH, model);
  NarrowestJoint out;
  for (std::size_t j = 1; j < static_cast<std::size_t>(model.njoints); ++j) {
    const auto first = static_cast<Eigen::Index>(model.joints[j].idx_q());
    for (Eigen::Index i = first;
         i < first + static_cast<Eigen::Index>(model.joints[j].nq()) && i < model.nq; ++i) {
      const double w = model.upperPositionLimit[i] - model.lowerPositionLimit[i];
      if (std::isfinite(w) && w < out.width) {
        out.width = w;
        out.name = model.names[j];
      }
    }
  }
  return out;
}

/// The shipped config with one key replaced — the same idiom
/// test_wbc_layout_profile_gate uses, and for the same reason: LoadConfig
/// rejects a config missing any of its required sections long before it reaches
/// the box, so a hand-built node would carry a copy of the real schema and rot
/// against it. `tsid` is dropped because it names hand contact frames that only
/// exist in the combined arm+hand model.
YAML::Node ConfigWithMargin(double margin) {
  YAML::Node cfg = YAML::LoadFile(RTC_WBC_CONFIG_PATH)["demo_wbc_controller"];
  cfg.remove("tsid");
  cfg["integration"]["position_margin"] = margin;
  return cfg;
}

TEST(WbcPositionMarginBox, AnInvertingMarginIsRefusedByNameAndNotByIndex) {
  const NarrowestJoint narrow = FindNarrowestJoint();
  ASSERT_FALSE(narrow.name.empty()) << "no finitely-limited joint in the fixture URDF";

  // Just past the point where both ends cross. Derived, so it keeps inverting
  // exactly one joint — the narrowest — whatever that joint's width becomes.
  const double margin = 0.5 * narrow.width + 0.01;
  DemoWbcController ctrl(RTC_UR5E_URDF_PATH);

  try {
    ctrl.LoadConfig(ConfigWithMargin(margin));
    ADD_FAILURE() << "a margin of " << margin << " inverts joint '" << narrow.name << "' (width "
                  << narrow.width << ") and was accepted";
  } catch (const std::runtime_error& e) {
    const std::string msg = e.what();
    // The four things the operator cannot recover from the downstream message.
    EXPECT_NE(msg.find(narrow.name), std::string::npos)
        << "the message does not name the joint — the operator is left with an index: " << msg;
    EXPECT_NE(msg.find("position_margin"), std::string::npos)
        << "the message does not name the key that produced the box: " << msg;
    EXPECT_NE(msg.find(std::to_string(margin)), std::string::npos)
        << "the message does not carry the margin that was applied: " << msg;
    EXPECT_NE(msg.find(std::to_string(0.5 * narrow.width)), std::string::npos)
        << "the message does not say what margin would fit: " << msg;
    // Attribution: this is the binding's box, not the downstream consumer's.
    EXPECT_NE(msg.find("demo_wbc_controller"), std::string::npos) << msg;
    EXPECT_EQ(msg.find("ClikReferenceGenerator"), std::string::npos)
        << "the downstream gate answered first, so the message still cannot name the cause: "
        << msg;
  }
}

TEST(WbcPositionMarginBox, AMarginThatPinsAJointExactlyIsStillAccepted) {
  // q_min == q_max is a joint pinned to a point, which is a legitimate
  // configuration and NOT an inversion. Refusing it would turn a working
  // deployment into a bring-up failure, which is the direction that costs
  // availability — #468 pinned this equality and the new check must not
  // narrow it.
  const NarrowestJoint narrow = FindNarrowestJoint();
  ASSERT_FALSE(narrow.name.empty());

  DemoWbcController ctrl(RTC_UR5E_URDF_PATH);
  EXPECT_NO_THROW(ctrl.LoadConfig(ConfigWithMargin(0.5 * narrow.width)))
      << "a margin that pins joint '" << narrow.name << "' exactly was refused as an inversion";
}

TEST(WbcPositionMarginBox, TheShippedMarginLeavesEveryJointABox) {
  // Acceptance 3, stated as the property rather than as a number: whatever the
  // shipped margin is, it has to fit inside every joint of the shipped model.
  const YAML::Node cfg = YAML::LoadFile(RTC_WBC_CONFIG_PATH)["demo_wbc_controller"];
  const double shipped = cfg["integration"]["position_margin"].as<double>(0.02);
  const NarrowestJoint narrow = FindNarrowestJoint();

  EXPECT_LT(shipped, 0.5 * narrow.width)
      << "the SHIPPED margin " << shipped << " inverts joint '" << narrow.name << "' (width "
      << narrow.width << ") — this is a shipped-config regression, not a test failure";

  DemoWbcController ctrl(RTC_UR5E_URDF_PATH);
  EXPECT_NO_THROW(ctrl.LoadConfig(ConfigWithMargin(shipped)));
}

}  // namespace
