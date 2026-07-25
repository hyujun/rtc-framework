// ── test_closure_state_publisher — 노드 end-to-end (성공기준 4). ──────────────────
//    actuated JointState 주입 → 출력에 전체 model 관절 포함 + ‖φ(q_out)‖ < 1e-8 +
//    actuated(j_crank) 불변. crank_rocker 비특이 픽스처(neutral 근방 crank 유효 구간). ──
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "rtc_urdf_bridge/closure_state_publisher.hpp"
#include "rtc_urdf_bridge/loop_projection.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"
#include "test_urdf_path.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace rub = rtc_urdf_bridge;
using rtc::test::CrankRocker;
using rtc::test::TestUrdfPath;
using namespace std::chrono_literals;

namespace {

rclcpp::NodeOptions NodeParams() {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"urdf_path", TestUrdfPath("crank_rocker.urdf")},
      {"closure_path", TestUrdfPath("crank_rocker.closure.yaml")},
      {"input_topic", std::string("/test/actuated")},
      {"output_topic", std::string("/test/full")},
  });
  return opts;
}

}  // namespace

class ClosureStatePublisherTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// ── 성공기준 4: actuated 주입 → 전체 model 관절 출력 + loop 닫힘 + actuated 고정. ───
TEST_F(ClosureStatePublisherTest, ReconstructsLoopConsistentFullState) {
  const rub::ClosedChainModel cc = CrankRocker();

  auto node = std::make_shared<rub::ClosureStatePublisher>(NodeParams());
  auto helper = std::make_shared<rclcpp::Node>("test_helper");

  std::optional<sensor_msgs::msg::JointState> last_out;
  auto sub = helper->create_subscription<sensor_msgs::msg::JointState>(
      "/test/full", rclcpp::QoS(10),
      [&last_out](const sensor_msgs::msg::JointState& msg) { last_out = msg; });
  auto pub =
      helper->create_publisher<sensor_msgs::msg::JointState>("/test/actuated", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  // pub/sub matching 을 위해 잠시 spin.
  const auto deadline0 = std::chrono::steady_clock::now() + 1s;
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline0) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  const double kCrank = 0.15;  // 비특이 유효 구간 (P1 실측).
  sensor_msgs::msg::JointState in;
  in.name = {"j_crank"};
  in.position = {kCrank};
  pub->publish(in);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!last_out.has_value() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(last_out.has_value()) << "출력 JointState 미수신";

  // (a) 전체 model 관절(universe 제외 3개: j_crank/j_coupler/j_rocker) 포함.
  EXPECT_EQ(last_out->name.size(), static_cast<std::size_t>(cc.model.njoints - 1));
  for (const char* expected : {"j_crank", "j_coupler", "j_rocker"}) {
    EXPECT_NE(std::find(last_out->name.begin(), last_out->name.end(), std::string(expected)),
              last_out->name.end())
        << expected << " 누락";
  }
  ASSERT_EQ(last_out->position.size(), last_out->name.size());

  // (b) 출력을 full q 로 복원해 closure error ‖φ‖ < 1e-8 (loop 닫힘) 확인.
  Eigen::VectorXd q_out = pinocchio::neutral(cc.model);
  double crank_out = 0.0;
  for (std::size_t i = 0; i < last_out->name.size(); ++i) {
    const auto& jn = last_out->name[i];
    ASSERT_TRUE(cc.model.existJointName(jn)) << jn;
    const auto jid = cc.model.getJointId(jn);
    q_out[cc.model.idx_qs[jid]] = last_out->position[i];
    if (jn == "j_crank") {
      crank_out = last_out->position[i];
    }
  }
  EXPECT_TRUE(q_out.allFinite());

  pinocchio::Data data(cc.model);
  const auto errors = rub::ComputeClosureErrors(cc.model, data, cc.constraints, q_out);
  ASSERT_FALSE(errors.empty());
  for (const auto& e : errors) {
    EXPECT_LT(e.norm, 1e-8) << e.name;
  }

  // (c) actuated(j_crank) 는 입력값 그대로 고정.
  EXPECT_NEAR(crank_out, kCrank, 1e-9);
}

// ── 성공기준 5: 큰 actuated 점프를 한 메시지로 받아도 조립 분기를 유지한다 (#248). ────
//   cold start(초기 seed=q_ref) 나 빠른 동작·메시지 유실에서 실제로 발생하는 경로다.
//   뒤집힌 분기도 ‖φ‖≈0 이라 closure error 검사만으로는 잡히지 않으므로, 미세 continuation
//   기준해와 직접 대조한다.
TEST_F(ClosureStatePublisherTest, LargeActuatedJumpKeepsAssemblyBranch) {
  const rub::ClosedChainModel cc = CrankRocker();
  pinocchio::Data data(cc.model);
  const auto crank_q = cc.model.idx_qs[cc.model.getJointId("j_crank")];

  constexpr double kStart = 0.05;
  constexpr double kTarget = 1.2;  // Δ=1.15 rad — 기본 증분(0.05)의 23배 점프

  // 물리적으로 옳은 분기의 기준해: 0.01 rad 미세 continuation.
  Eigen::VectorXd q_truth = pinocchio::neutral(cc.model);
  q_truth[crank_q] = kStart;
  {
    const auto seed_res = rub::ProjectPassiveToConstraint(cc.model, data, cc.constraints, q_truth,
                                                          cc.actuated_joint_ids);
    ASSERT_TRUE(seed_res.converged);
    q_truth = seed_res.q;
    constexpr int kFineSteps = 115;
    for (int k = 1; k <= kFineSteps; ++k) {
      q_truth[crank_q] = kStart + (kTarget - kStart) * static_cast<double>(k) / kFineSteps;
      const auto res = rub::ProjectPassiveToConstraint(cc.model, data, cc.constraints, q_truth,
                                                       cc.actuated_joint_ids);
      ASSERT_TRUE(res.converged) << "기준해 continuation 실패 k=" << k;
      q_truth = res.q;
    }
  }

  auto node = std::make_shared<rub::ClosureStatePublisher>(NodeParams());
  auto helper = std::make_shared<rclcpp::Node>("test_helper_jump");

  std::optional<sensor_msgs::msg::JointState> last_out;
  auto sub = helper->create_subscription<sensor_msgs::msg::JointState>(
      "/test/full", rclcpp::QoS(10),
      [&last_out](const sensor_msgs::msg::JointState& msg) { last_out = msg; });
  auto pub =
      helper->create_publisher<sensor_msgs::msg::JointState>("/test/actuated", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  const auto deadline0 = std::chrono::steady_clock::now() + 1s;
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline0) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  // (1) 조립된 시작 형상으로 seed 확립 → (2) 한 메시지로 큰 점프.
  for (const double crank : {kStart, kTarget}) {
    last_out.reset();
    sensor_msgs::msg::JointState in;
    in.name = {"j_crank"};
    in.position = {crank};
    pub->publish(in);

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!last_out.has_value() && std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_TRUE(last_out.has_value()) << "출력 JointState 미수신 (crank=" << crank << ")";
  }

  Eigen::VectorXd q_out = pinocchio::neutral(cc.model);
  for (std::size_t i = 0; i < last_out->name.size(); ++i) {
    const auto jid = cc.model.getJointId(last_out->name[i]);
    q_out[cc.model.idx_qs[jid]] = last_out->position[i];
  }
  EXPECT_NEAR(q_out[crank_q], kTarget, 1e-9) << "actuated 는 입력값 그대로";
  EXPECT_LT((q_out - q_truth).cwiseAbs().maxCoeff(), 1e-6)
      << "단일 대점프 메시지에서 조립 분기를 벗어났다 (#248)";
}
