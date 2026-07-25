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
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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

// ── #250: residual floor 로봇에서 publisher 가 hold 없이 actuated 를 추종한다. ──────
//   P1B ring closure 의 ~20 nm URDF 좌표 불일치 → ‖φ‖ 바닥 2.8e-8 은 strict(1e-10) 에
//   도달 불가. 커밋 게이트가 strict 면 모든 메시지가 "직전 해 hold" 로 떨어져 손이
//   동결된다 — #124 는 이를 solver tolerance 완화(closure_tolerance: 1e-6)로 우회했고,
//   acceptance 분리(#250) 후에는 기본 옵션으로 추종해야 한다 (우회 파라미터 불필요).
//   floor 는 crank_rocker 의 c2 anchor 를 z 3e-8 로 어긋낸 임시 URDF 로 재현한다
//   (평면 기구 → 보상 불가, FlooredCrankRocker 와 동일 원리).
TEST_F(ClosureStatePublisherTest, TracksActuatedInputDespiteResidualFloor) {
  // 임시 floored URDF 생성 (c2_mount origin 만 z 3e-8).
  std::ifstream src(TestUrdfPath("crank_rocker.urdf"));
  ASSERT_TRUE(src.good());
  std::string urdf((std::istreambuf_iterator<char>(src)), std::istreambuf_iterator<char>());
  const std::string needle = "xyz=\"0.30 0 0\"";  // c2_mount origin (파일 내 유일)
  const auto pos = urdf.find(needle);
  ASSERT_NE(pos, std::string::npos) << "fixture 변경됨 — c2_mount origin 을 찾지 못했다";
  ASSERT_EQ(urdf.find(needle, pos + 1), std::string::npos) << "needle 이 유일하지 않다";
  urdf.replace(pos, needle.size(), "xyz=\"0.30 0 3e-08\"");
  const std::filesystem::path floored_path =
      std::filesystem::temp_directory_path() / "crank_rocker_floor_250.urdf";
  {
    std::ofstream out_file(floored_path);
    out_file << urdf;
  }

  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"urdf_path", floored_path.string()},
      {"closure_path", TestUrdfPath("crank_rocker.closure.yaml")},
      {"input_topic", std::string("/test/floor/actuated")},
      {"output_topic", std::string("/test/floor/full")},
  });
  auto node = std::make_shared<rub::ClosureStatePublisher>(opts);
  auto helper = std::make_shared<rclcpp::Node>("test_helper_floor");

  std::optional<sensor_msgs::msg::JointState> last_out;
  auto sub = helper->create_subscription<sensor_msgs::msg::JointState>(
      "/test/floor/full", rclcpp::QoS(10),
      [&last_out](const sensor_msgs::msg::JointState& msg) { last_out = msg; });
  auto pub = helper->create_publisher<sensor_msgs::msg::JointState>("/test/floor/actuated",
                                                                    rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);
  const auto deadline0 = std::chrono::steady_clock::now() + 1s;
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline0) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  constexpr double kCrank = 0.15;  // 비특이 유효 구간
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

  // floored 모델 기준으로 출력 재구성 → actuated 추종 + ‖φ‖ ≈ floor (≤ acceptance).
  const rub::ClosedChainModel cc_floor = rub::BuildClosedChainModelFromExtendedUrdf(
      floored_path.string(), TestUrdfPath("crank_rocker.closure.yaml"));
  Eigen::VectorXd q_out = pinocchio::neutral(cc_floor.model);
  double crank_out = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t i = 0; i < last_out->name.size(); ++i) {
    const auto& jn = last_out->name[i];
    ASSERT_TRUE(cc_floor.model.existJointName(jn)) << jn;
    q_out[cc_floor.model.idx_qs[cc_floor.model.getJointId(jn)]] = last_out->position[i];
    if (jn == "j_crank") {
      crank_out = last_out->position[i];
    }
  }
  // hold 됐다면 crank 는 q_ref(neutral 근방 ≈0) 에 남는다 — 추종이 핵심 판별.
  EXPECT_NEAR(crank_out, kCrank, 1e-9) << "floor 로봇에서 publisher 가 hold 됐다";
  EXPECT_TRUE(q_out.allFinite());

  pinocchio::Data data(cc_floor.model);
  const auto errors = rub::ComputeClosureErrors(cc_floor.model, data, cc_floor.constraints, q_out);
  ASSERT_FALSE(errors.empty());
  for (const auto& e : errors) {
    EXPECT_LT(e.norm, 1e-6) << e.name;  // acceptance 이내
    EXPECT_GT(e.norm, 1e-10) << e.name;  // floor 존재 (strict 도달 불가) — 픽스처 유효성
  }

  std::filesystem::remove(floored_path);
}
