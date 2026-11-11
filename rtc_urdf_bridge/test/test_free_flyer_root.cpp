// IsFreeFlyerRoot: the root joint's TYPE decides, not its dimensions, and only
// joint 1 is looked at. The constants are the free-flyer's own nq / nv.
#include "rtc_urdf_bridge/free_flyer_root.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <pinocchio/multibody/joint/joint-spherical.hpp>
#include <pinocchio/multibody/joint/joint-translation.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

namespace rtc_urdf_bridge {
namespace {

// A one-joint model on `root`, with a unit body so the model is well formed.
template <typename Joint>
pinocchio::Model WithRoot(const Joint& root) {
  pinocchio::Model m;
  const auto id = m.addJoint(0, root, pinocchio::SE3::Identity(), "root");
  m.appendBodyToJoint(id, pinocchio::Inertia::Identity(), pinocchio::SE3::Identity());
  return m;
}

TEST(FreeFlyerRoot, AFreeFlyerRootFloatsAndTheConstantsAreItsSizes) {
  const pinocchio::Model m = WithRoot(pinocchio::JointModelFreeFlyer());
  EXPECT_TRUE(IsFreeFlyerRoot(m));
  EXPECT_EQ(m.joints[1].nq(), kFreeFlyerNq);
  EXPECT_EQ(m.joints[1].nv(), kFreeFlyerNv);
  EXPECT_EQ(m.nq, kFreeFlyerNq);
  EXPECT_EQ(m.nv, kFreeFlyerNv);
}

TEST(FreeFlyerRoot, AModelWithoutAJointDoesNotFloat) {
  const pinocchio::Model m;
  EXPECT_FALSE(IsFreeFlyerRoot(m));
}

TEST(FreeFlyerRoot, ARevoluteRootDoesNotFloat) {
  EXPECT_FALSE(IsFreeFlyerRoot(WithRoot(pinocchio::JointModelRX())));
}

// The case a dimension rule gets wrong: translation + spherical is nq 7 / nv 6
// like a free-flyer, but a composite — not the free-flyer's layout.
TEST(FreeFlyerRoot, ACompositeRootWithTheFreeFlyersDimensionsDoesNotFloat) {
  pinocchio::JointModelComposite root;
  root.addJoint(pinocchio::JointModelTranslation());
  root.addJoint(pinocchio::JointModelSpherical());
  const pinocchio::Model m = WithRoot(root);
  ASSERT_EQ(m.joints[1].nq(), kFreeFlyerNq);
  ASSERT_EQ(m.joints[1].nv(), kFreeFlyerNv);
  EXPECT_FALSE(IsFreeFlyerRoot(m));
}

// Only the root is looked at: a free-flyer deeper in the tree is a joint, not
// a floating base.
TEST(FreeFlyerRoot, AFreeFlyerBelowAFixedRootDoesNotFloat) {
  pinocchio::Model m = WithRoot(pinocchio::JointModelRX());
  const auto id =
      m.addJoint(1, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "inner");
  m.appendBodyToJoint(id, pinocchio::Inertia::Identity(), pinocchio::SE3::Identity());
  EXPECT_FALSE(IsFreeFlyerRoot(m));
}

}  // namespace
}  // namespace rtc_urdf_bridge
