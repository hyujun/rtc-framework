// ── iiwa7 + LEAP: a hand pose that tells joint orders apart, and its oracle ──
//
// For the tests that ask a controller where the fingertips are
// (test_hand_fk_wiring, and the wbc cases in test_demo_wbc_tsid_path — a wbc
// controller publishes poses only with TSID up, and that rig lives there).
//
// The oracle is forward kinematics on the FULL model, addressed by joint NAME.
// It shares no joint order, no reduced model and no frame id with the code
// under test.
#pragma once

#include "iiwa7_leap_test_fixture.hpp"

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

inline const std::vector<std::string> kLeapTips = {"thumb_tip_head", "index_tip_head",
                                                   "middle_tip_head", "ring_tip_head"};

// One value per hand joint, in the DEVICE's order (thumb first). No two are
// equal, so any permutation of the sixteen lands the tips somewhere else — with
// the hand at zero, or all joints at one value, a permuted read is the same
// read. All inside the joint limits.
inline constexpr std::array<double, kHandDof> kLeapQ = {0.60, 0.33, 0.21,  -0.30, 0.10, 0.52,
                                                        0.41, 0.28, -0.20, 0.80,  0.63, 0.47,
                                                        0.36, 1.10, 0.90,  0.70};

/// MakeIiwa7LeapState with the hand at kLeapQ.
inline rtc::ControllerState MakeIiwa7LeapStateWithHandPose() {
  rtc::ControllerState state = MakeIiwa7LeapState();
  for (std::size_t i = 0; i < kLeapQ.size(); ++i) {
    state.devices[1].positions[i] = kLeapQ[i];
  }
  return state;
}

/// Full-model FK with the named joints set and every other joint at neutral.
/// Returns the pose of `frame` expressed in `in_frame`.
inline pinocchio::SE3 FullModelPose(const pinocchio::Model& model,
                                    const std::map<std::string, double>& q_by_name,
                                    const std::string& in_frame, const std::string& frame) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  for (const auto& [name, value] : q_by_name) {
    EXPECT_TRUE(model.existJointName(name)) << name;
    q[model.joints[model.getJointId(name)].idx_q()] = value;
  }
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, q);
  EXPECT_TRUE(model.existFrame(in_frame)) << in_frame;
  EXPECT_TRUE(model.existFrame(frame)) << frame;
  return data.oMf[model.getFrameId(in_frame)].actInv(data.oMf[model.getFrameId(frame)]);
}

/// Where the full model puts `frame`, in the arm root, with the arm at kArmHome
/// and the hand at kLeapQ.
inline pinocchio::SE3 Iiwa7LeapOracle(const std::string& frame) {
  const auto devices = MakeIiwa7LeapDeviceConfigs();
  std::map<std::string, double> q;
  const auto& arm_names = devices.at("iiwa7").joint_state_names;
  for (std::size_t i = 0; i < arm_names.size(); ++i) {
    q[arm_names[i]] = kArmHome[i];
  }
  const auto& hand_names = devices.at("leap").joint_state_names;
  for (std::size_t i = 0; i < hand_names.size(); ++i) {
    q[hand_names[i]] = kLeapQ[i];
  }
  return FullModelPose(*SharedIiwa7LeapBuilder()->GetFullModel(), q, "link_0", frame);
}

inline pinocchio::SE3 ToSe3(const rtc::Pose& pose) {
  const Eigen::Quaterniond q(pose.quaternion[0], pose.quaternion[1], pose.quaternion[2],
                             pose.quaternion[3]);
  return pinocchio::SE3(q.toRotationMatrix(),
                        Eigen::Vector3d(pose.position[0], pose.position[1], pose.position[2]));
}

inline void ExpectSamePose(const pinocchio::SE3& got, const pinocchio::SE3& want, double tol,
                           const std::string& what) {
  EXPECT_LE((got.translation() - want.translation()).norm(), tol)
      << what << ": position off by " << (got.translation() - want.translation()).norm()
      << " m\n  got  " << got.translation().transpose() << "\n  want "
      << want.translation().transpose();
  EXPECT_LE((got.rotation() - want.rotation()).norm(), tol)
      << what << ": rotation off by " << (got.rotation() - want.rotation()).norm();
}

/// The hand pose and the rig are able to fail a fingertip assertion: the
/// device's order is not the hand model's, the hand root is not the arm tip,
/// and no two hand joints share a value.
inline void ExpectIiwa7LeapRigTellsTheCasesApart() {
  const auto builder = SharedIiwa7LeapBuilder();
  const pinocchio::Model& hand = *builder->GetTreeModel("leap");
  const auto names = MakeIiwa7LeapDeviceConfigs().at("leap").joint_state_names;
  ASSERT_EQ(hand.nq, kHandDof);
  EXPECT_EQ(builder->GetActuatedModel(), nullptr) << "the fixture's hand must stay serial";

  std::vector<int> idx;
  for (const auto& n : names) {
    ASSERT_TRUE(hand.existJointName(n)) << n;
    idx.push_back(static_cast<int>(hand.joints[hand.getJointId(n)].idx_q()));
  }
  EXPECT_FALSE(std::is_sorted(idx.begin(), idx.end()));

  const pinocchio::SE3 mount = Iiwa7LeapOracle("ee_link").actInv(Iiwa7LeapOracle("base"));
  EXPECT_GT(mount.translation().norm(), 1e-3);
  EXPECT_GT((mount.rotation() - Eigen::Matrix3d::Identity()).norm(), 1.0);

  std::vector<double> q(kLeapQ.begin(), kLeapQ.end());
  std::sort(q.begin(), q.end());
  EXPECT_EQ(std::adjacent_find(q.begin(), q.end()), q.end()) << "two hand joints share a value";
}

}  // namespace integrated_bringup::testfx
