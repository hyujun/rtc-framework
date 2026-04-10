// ─────────────────────────────────────────────────────────────────────────────
// Unit tests for ur5e_bringup::ComputeVirtualTcp (virtual_tcp.hpp)
// ─────────────────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>

#include "ur5e_bringup/controllers/virtual_tcp.hpp"

#include <pinocchio/math/rpy.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <span>

using ur5e_bringup::ComputeVirtualTcp;
using ur5e_bringup::FingertipVtcpInput;
using ur5e_bringup::VirtualTcpConfig;
using ur5e_bringup::VirtualTcpMode;

namespace {

constexpr double kEps = 1e-9;

// Build four fingertips arranged around the TCP origin so their centroid
// sits at the origin when all are active.
std::array<FingertipVtcpInput, 4> MakeBalancedFingertips()
{
  std::array<FingertipVtcpInput, 4> fts{};
  fts[0].position_in_tcp = Eigen::Vector3d( 0.01,  0.02,  0.03);
  fts[1].position_in_tcp = Eigen::Vector3d(-0.01, -0.02,  0.03);
  fts[2].position_in_tcp = Eigen::Vector3d( 0.01, -0.02, -0.03);
  fts[3].position_in_tcp = Eigen::Vector3d(-0.01,  0.02, -0.03);
  for (auto& ft : fts) ft.active = true;
  return fts;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Disabled mode
// ═══════════════════════════════════════════════════════════════════════════

TEST(VirtualTcpTest, DisabledModeReturnsInvalid)
{
  VirtualTcpConfig cfg;  // default = kDisabled
  ASSERT_EQ(cfg.mode, VirtualTcpMode::kDisabled);

  pinocchio::SE3 T_base_tcp = pinocchio::SE3::Identity();
  auto fts = MakeBalancedFingertips();
  auto result = ComputeVirtualTcp(cfg, T_base_tcp, fts);
  EXPECT_FALSE(result.valid);
  // T_tcp_vtcp stays at identity
  EXPECT_TRUE(result.T_tcp_vtcp.translation().isZero(kEps));
  EXPECT_TRUE(result.T_tcp_vtcp.rotation().isIdentity(kEps));
}

// ═══════════════════════════════════════════════════════════════════════════
// Centroid mode
// ═══════════════════════════════════════════════════════════════════════════

TEST(VirtualTcpTest, CentroidAllActiveAtIdentity)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kCentroid;

  auto fts = MakeBalancedFingertips();
  auto result = ComputeVirtualTcp(cfg, pinocchio::SE3::Identity(), fts);

  ASSERT_TRUE(result.valid);
  // Balanced set → centroid at origin
  EXPECT_TRUE(result.T_tcp_vtcp.translation().isZero(kEps));
  // Rotation is identity (no RPY set)
  EXPECT_TRUE(result.T_tcp_vtcp.rotation().isIdentity(kEps));
  // world_pose == T_base_tcp * T_tcp_vtcp == identity
  EXPECT_TRUE(result.world_pose.translation().isZero(kEps));
  EXPECT_TRUE(result.world_pose.rotation().isIdentity(kEps));
}

TEST(VirtualTcpTest, CentroidPartialActiveIsMeanOfActive)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kCentroid;

  std::array<FingertipVtcpInput, 4> fts{};
  fts[0].position_in_tcp = Eigen::Vector3d(1.0, 0.0, 0.0);
  fts[0].active = true;
  fts[1].position_in_tcp = Eigen::Vector3d(0.0, 1.0, 0.0);
  fts[1].active = true;
  // fts[2], fts[3] inactive → ignored
  fts[2].position_in_tcp = Eigen::Vector3d(100.0, 100.0, 100.0);
  fts[2].active = false;
  fts[3].active = false;

  auto result = ComputeVirtualTcp(cfg, pinocchio::SE3::Identity(), fts);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().x(), 0.5, kEps);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().y(), 0.5, kEps);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().z(), 0.0, kEps);
}

TEST(VirtualTcpTest, CentroidAllInactiveReturnsInvalid)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kCentroid;

  std::array<FingertipVtcpInput, 4> fts{};  // all inactive
  auto result = ComputeVirtualTcp(cfg, pinocchio::SE3::Identity(), fts);
  EXPECT_FALSE(result.valid);
}

// ═══════════════════════════════════════════════════════════════════════════
// Weighted mode
// ═══════════════════════════════════════════════════════════════════════════

TEST(VirtualTcpTest, WeightedZeroForceEqualsCentroid)
{
  // With f=0 every active finger weight is 1.0 → weighted mean == unweighted.
  VirtualTcpConfig cfg_w;
  cfg_w.mode = VirtualTcpMode::kWeighted;

  VirtualTcpConfig cfg_c;
  cfg_c.mode = VirtualTcpMode::kCentroid;

  auto fts = MakeBalancedFingertips();
  for (auto& ft : fts) ft.force_magnitude = 0.0;

  auto rw = ComputeVirtualTcp(cfg_w, pinocchio::SE3::Identity(), fts);
  auto rc = ComputeVirtualTcp(cfg_c, pinocchio::SE3::Identity(), fts);
  ASSERT_TRUE(rw.valid);
  ASSERT_TRUE(rc.valid);
  EXPECT_TRUE((rw.T_tcp_vtcp.translation() - rc.T_tcp_vtcp.translation()).isZero(kEps));
}

TEST(VirtualTcpTest, WeightedBiasedTowardHighForceFingertip)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kWeighted;

  std::array<FingertipVtcpInput, 2> fts{};
  fts[0].position_in_tcp = Eigen::Vector3d(1.0, 0.0, 0.0);
  fts[0].active = true;
  fts[0].force_magnitude = 0.0;
  fts[1].position_in_tcp = Eigen::Vector3d(0.0, 0.0, 0.0);
  fts[1].active = true;
  fts[1].force_magnitude = 9.0;  // weight = 10.0

  auto result = ComputeVirtualTcp(cfg, pinocchio::SE3::Identity(), fts);
  ASSERT_TRUE(result.valid);
  // weight0 = 1.0, weight1 = 10.0 → centroid_x = (1*1 + 10*0)/11 = 1/11
  EXPECT_NEAR(result.T_tcp_vtcp.translation().x(), 1.0 / 11.0, kEps);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().y(), 0.0, kEps);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().z(), 0.0, kEps);
}

TEST(VirtualTcpTest, WeightedAllInactiveReturnsInvalid)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kWeighted;

  std::array<FingertipVtcpInput, 4> fts{};  // all inactive → total_weight = 0
  auto result = ComputeVirtualTcp(cfg, pinocchio::SE3::Identity(), fts);
  EXPECT_FALSE(result.valid);
}

// ═══════════════════════════════════════════════════════════════════════════
// Constant mode
// ═══════════════════════════════════════════════════════════════════════════

TEST(VirtualTcpTest, ConstantModeAppliesOffsetIndependentOfFingertips)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kConstant;
  cfg.offset = {{0.1, -0.2, 0.3}};

  std::array<FingertipVtcpInput, 4> fts{};  // fingertips ignored in constant mode
  auto result = ComputeVirtualTcp(cfg, pinocchio::SE3::Identity(), fts);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().x(),  0.1, kEps);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().y(), -0.2, kEps);
  EXPECT_NEAR(result.T_tcp_vtcp.translation().z(),  0.3, kEps);
}

TEST(VirtualTcpTest, ConstantModeRpyOrientationApplied)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kConstant;
  cfg.offset = {{0.0, 0.0, 0.0}};
  // 90 deg rotation about Z
  cfg.orientation = {{0.0, 0.0, M_PI / 2.0}};

  std::array<FingertipVtcpInput, 4> fts{};
  auto result = ComputeVirtualTcp(cfg, pinocchio::SE3::Identity(), fts);
  ASSERT_TRUE(result.valid);

  const Eigen::Matrix3d R_expected =
      pinocchio::rpy::rpyToMatrix(0.0, 0.0, M_PI / 2.0);
  EXPECT_TRUE((result.T_tcp_vtcp.rotation() - R_expected).norm() < 1e-9);
}

// ═══════════════════════════════════════════════════════════════════════════
// World-pose composition
// ═══════════════════════════════════════════════════════════════════════════

TEST(VirtualTcpTest, WorldPoseEqualsBaseTimesTcpVtcp)
{
  VirtualTcpConfig cfg;
  cfg.mode = VirtualTcpMode::kConstant;
  cfg.offset = {{0.05, 0.0, 0.10}};

  // Non-trivial base→tcp transform
  pinocchio::SE3 T_base_tcp(pinocchio::SE3::Identity());
  T_base_tcp.translation() = Eigen::Vector3d(0.4, 0.2, 0.5);
  T_base_tcp.rotation() = pinocchio::rpy::rpyToMatrix(0.0, M_PI / 4.0, 0.0);

  std::array<FingertipVtcpInput, 4> fts{};
  auto result = ComputeVirtualTcp(cfg, T_base_tcp, fts);
  ASSERT_TRUE(result.valid);

  const pinocchio::SE3 expected = T_base_tcp.act(result.T_tcp_vtcp);
  EXPECT_TRUE((result.world_pose.translation() - expected.translation())
                  .isZero(1e-9));
  EXPECT_TRUE((result.world_pose.rotation() - expected.rotation()).norm() < 1e-9);

  // Spot-check offset is in the TCP frame (not base frame):
  // world = T_base_tcp.translation() + R_base_tcp * offset
  const Eigen::Vector3d offset_in_tcp(cfg.offset[0], cfg.offset[1], cfg.offset[2]);
  const Eigen::Vector3d expected_world =
      T_base_tcp.translation() + T_base_tcp.rotation() * offset_in_tcp;
  EXPECT_TRUE((result.world_pose.translation() - expected_world).isZero(1e-9));
}
