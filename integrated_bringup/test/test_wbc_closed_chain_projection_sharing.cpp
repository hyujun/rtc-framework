// ── #175: WBC 가 closed-chain 사영을 한 번만 돌린다 (컨트롤러 레벨) ────────────
//
// 이 파일이 지키는 것은 두 가지이고, **둘 다 컨트롤러 레벨이어야만** 관측된다:
//
//  1. 사영 2→1. 두 소비자(축약 동역학 provider, fingertip FK)가 같은 사영을 읽으므로 pose 값만
//     보고는 1회인지 2회인지 알 수 없다 — wrapper 축 카운터가 유일한 증거다.
//  2. 입력 provenance. 통합 전 fingertip 은 자기 q_a 를 device 에서 per-slot 으로 모았고, 통합
//     후에는 provider 가 cache q 로 돌린 사영을 읽는다. cache 는 **블록 단위로 hold** 하므로
//     "사영은 이번 tick 에 돌았는데 입력은 직전 tick 값" 인 tick 이 존재한다. 이 축은 두 래퍼에
//     같은 q_a 를 먹이는 단위 테스트로는 원리적으로 관측할 수 없다 (그 테스트가 provenance 차이를
//     우회한다) — 그래서 device 폭·구멍을 조작할 수 있는 여기여야 한다.
//
// 하네스는 test_device_readability_gate.cpp 를 쓸 수 없다: 그 파일은 URDF 없이 컨트롤러를
// 세우므로(의도된 설계) closure 도 provider 도 fingertip FK 도 존재하지 않는다. 대신 shipped
// ur5e_p1b config + 실제 URDF/closure 로 bring-up 을 재생한다 (test_demo_wbc_tsid_path 와 동형).
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <cstdint>
#include <memory>
#include <string>

#ifndef RTC_WBC_CONFIG_DIR
#error "RTC_WBC_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::DemoWbcController;
using rtc::ControllerOutput;
using rtc::ControllerState;
namespace fx = integrated_bringup::testfx;

// shipped config 를 그대로 쓴다 — 인라인 YAML 사본을 두면 출하 config 가 바뀌어도 이 테스트는
// 옛 형상을 계속 통과시킨다. 그러면 "출하 로봇에서 사영이 1회" 라는 주장이 사본에 대한 주장이 된다.
YAML::Node ShippedWbcConfig(const char* robot) {
  const std::string path =
      std::string(RTC_WBC_CONFIG_DIR) + "/" + robot + "/controllers/demo_wbc_controller.yaml";
  YAML::Node root = YAML::LoadFile(path);
  return root["demo_wbc_controller"];
}

class ClosedChainProjectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctrl_ = std::make_unique<DemoWbcController>("");
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("wbc_closed_chain_projection_test");

    // 프로덕션 bring-up 순서 (rt_controller_node_params.cpp) + lifecycle 전이. on_configure 가
    // InitClik 을 돌려야 clik_tcp_frame_idx_ 가 서고, 그 인덱스가 fingertip 블록의 진입 게이트다.
    ctrl_->SetSystemModelConfig(fx::SharedUr5eP1bModelConfig());
    ctrl_->SetSharedModelBuilder(fx::SharedUr5eP1bBuilder());
    ctrl_->SetControlRate(1.0 / fx::kUr5eDt);
    const YAML::Node cfg = ShippedWbcConfig("ur5e_p1b");
    ASSERT_TRUE(cfg) << "shipped ur5e_p1b demo_wbc_controller.yaml 을 못 읽었다";
    ctrl_->LoadConfig(cfg);
    ctrl_->SetDeviceNameConfigs(fx::MakeUr5eP1bDeviceConfigs());
    configured_ = ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg) ==
                  DemoWbcController::CallbackReturn::SUCCESS;
    ASSERT_TRUE(configured_) << "on_configure 실패 — 이 파일의 모든 단언이 공허해진다";

    state_ = fx::MakeUr5eP1bState();
    // 첫 tick: cache seed + 사영 1회. 이후 fingertip pose 캐시가 채워진다.
    last_out_ = ctrl_->Compute(state_);
  }

  void TearDown() override {
    ctrl_.reset();
    node_.reset();
  }

  void Tick() {
    state_.iteration += 1;
    last_out_ = ctrl_->Compute(state_);
  }

  /// #248 walk-in: 사영 seed 는 closure reference 에서 출발하고 tick 당 actuated 증분이 클램프
  /// (0.05)되므로, 측정 형상까지 걸어오는 동안은 held 라 fingertip 이 유효해지지 않는다 (owning
  /// 모드도 같다). 유효해질 때까지 돌린다 — bound 를 두어 "영원히 안 나옴" 이 hang 이 아니라
  /// 실패로 보이게 한다.
  void WarmUpUntilFingertipValid() {
    constexpr int kMaxWarmupTicks = 600;
    for (int i = 0; i < kMaxWarmupTicks && !last_out_.task_link_pose_valid[0]; ++i) {
      Tick();
    }
    ASSERT_TRUE(last_out_.task_link_pose_valid[0])
        << "walk-in 이 " << kMaxWarmupTicks << " tick 안에 안 끝났다 — 이후 단언이 공허해진다";
  }

  /// 팔 관절을 **seed clamp 안쪽**으로 민다. 클램프를 넘기면 게이트와 무관하게 held 가 되어
  /// "provenance 때문에 hold" 와 "클램프 때문에 hold" 를 구분할 수 없다 — 그러면 provenance
  /// 테스트가 잘못된 이유로 green 이 된다.
  void NudgeArm() {
    constexpr double kStep = 0.005;  // ‖Δ‖ ≈ 0.012 < 0.05
    for (int i = 0; i < fx::kUr5eArmDof; ++i) {
      state_.devices[0].positions[static_cast<std::size_t>(i)] += kStep;
    }
  }

  void NudgeHand() {
    constexpr double kStep = 0.004;
    for (int i = 0; i < fx::kP1bHandDof; ++i) {
      state_.devices[1].positions[static_cast<std::size_t>(i)] += kStep;
    }
  }

  [[nodiscard]] static bool SamePosition(const ControllerOutput& a, const ControllerOutput& b) {
    return a.task_link_poses[0].position == b.task_link_poses[0].position;
  }

  std::unique_ptr<DemoWbcController> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  ControllerState state_{};
  ControllerOutput last_out_{};
  bool configured_{false};
};

// ── 사영 2→1 (구조 + 계측) ──────────────────────────────────────────────────
TEST_F(ClosedChainProjectionTest, FingertipFkSharesTheReducedDynamicsProjection) {
  ASSERT_TRUE(ctrl_->HandFkSharesProjectionForTesting())
      << "closed-chain 로봇인데 fingertip FK 가 자기 사영을 갖고 있다 — 2→1 이 배선되지 않았다";

  WarmUpUntilFingertipValid();

  const std::uint32_t seq0 = ctrl_->ReducedProjectionSeqForTesting();
  constexpr int kTicks = 20;
  for (int i = 0; i < kTicks; ++i) {
    Tick();
  }

  EXPECT_EQ(ctrl_->ReducedProjectionSeqForTesting() - seq0, static_cast<std::uint32_t>(kTicks))
      << "정상 tick 은 사영을 정확히 1회 돌려야 한다";
  EXPECT_EQ(ctrl_->HandFkProjectionCountForTesting(), 0U)
      << "fingertip FK 가 직접 사영하면 tick 당 2회로 돌아간 것이다";

  // 그리고 실제로 publish surface 에 fingertip 이 실린다 — 카운터만 맞고 pose 가 안 나오면
  // "사영을 안 한다" 는 사실이 최적화가 아니라 회귀다.
  const ControllerOutput& out = last_out_;
  bool any_valid = false;
  for (std::size_t f = 0; f < out.task_link_pose_valid.size(); ++f) {
    any_valid = any_valid || out.task_link_pose_valid[f];
  }
  EXPECT_TRUE(any_valid) << "fingertip TF 가 하나도 안 실린다 — 공유가 pose 를 죽였다";
}

// ── provenance: 폭 축 (arm narrow) ──────────────────────────────────────────
//   cache 는 arm 게이트가 닫히면 **어느 블록도** 갱신하지 않는다. 그래도 provider 는 사영을
//   돌리므로(입력은 직전 tick q) 실행 카운터는 증가한다 — fingertip 이 그것을 fresh 로 채택하면
//   안 된다.
TEST_F(ClosedChainProjectionTest, NarrowArmDoesNotRefreshFingertipPose) {
  WarmUpUntilFingertipValid();
  const ControllerOutput good = last_out_;

  // 게이트를 닫고 클램프 안쪽으로 민다.
  state_.devices[0].num_channels = fx::kUr5eArmDof - 1;
  NudgeArm();
  const std::uint32_t seq_before = ctrl_->ReducedProjectionSeqForTesting();
  Tick();
  const ControllerOutput narrow = last_out_;

  EXPECT_NE(ctrl_->ReducedProjectionSeqForTesting(), seq_before)
      << "전제: 좁은 tick 에도 사영 자체는 돈다 (그래서 실행 카운터만으로는 부족하다)";
  EXPECT_TRUE(SamePosition(narrow, good))
      << "cache 가 hold 한 블록으로 돌린 사영을 fingertip 이 fresh 로 채택했다";

  // Positive control: 같은 섭동을 게이트가 열린 상태로 주면 pose 는 **움직여야** 한다.
  // 이게 없으면 위 단언은 "애초에 안 움직이는 섭동" 으로도 통과한다.
  state_.devices[0].num_channels = fx::kUr5eArmDof;
  Tick();
  EXPECT_FALSE(SamePosition(last_out_, good))
      << "positive control 실패 — 섭동 자체가 관측 불가능해서 위 단언이 공허하다";
}

// ── provenance: 구멍 축 (#284) ──────────────────────────────────────────────
//   폭은 정상인데 이번 메시지가 안 쓴 슬롯이 있는 경우. num_channels 가 멀쩡하므로 폭만 좁히는
//   fixture 로는 원리적으로 재현되지 않는 축이다.
TEST_F(ClosedChainProjectionTest, HoledArmSlotDoesNotRefreshFingertipPose) {
  WarmUpUntilFingertipValid();
  const ControllerOutput good = last_out_;

  state_.devices[0].hole_mask = (static_cast<std::uint64_t>(1) << 2);  // 슬롯 2 미기입
  NudgeArm();
  Tick();
  EXPECT_TRUE(SamePosition(last_out_, good))
      << "구멍 난 슬롯이 섞인 사영을 fingertip 이 채택했다 (#284 축)";

  state_.devices[0].hole_mask = 0;
  Tick();
  EXPECT_FALSE(SamePosition(last_out_, good))
      << "positive control 실패 — 구멍만 메웠는데도 pose 가 안 움직인다";
}

// ── provenance: hand 축 (#291 all-or-nothing) ───────────────────────────────
//   arm 은 신선한데 hand 블록만 hold 되는 tick — cache 가 **시간이 섞인** q 를 사영에 넘긴다.
TEST_F(ClosedChainProjectionTest, NarrowHandDoesNotRefreshFingertipPose) {
  WarmUpUntilFingertipValid();
  const ControllerOutput good = last_out_;

  state_.devices[1].num_channels = fx::kP1bHandDof - 1;
  NudgeHand();
  Tick();
  EXPECT_TRUE(SamePosition(last_out_, good))
      << "hand 블록이 hold 된 tick 의 사영을 fingertip 이 fresh 로 채택했다";

  state_.devices[1].num_channels = fx::kP1bHandDof;
  Tick();
  EXPECT_FALSE(SamePosition(last_out_, good))
      << "positive control 실패 — hand 폭만 되돌렸는데 pose 가 안 움직인다";
}

// ── 복구: provenance 가 돌아오면 즉시 반영된다 (hold 가 latch 되지 않는다) ────
TEST_F(ClosedChainProjectionTest, FingertipRecoversAfterTheGateReopens) {
  WarmUpUntilFingertipValid();
  const ControllerOutput good = last_out_;

  state_.devices[0].valid = false;
  Tick();
  EXPECT_TRUE(SamePosition(last_out_, good)) << "unreported device tick 은 직전 pose 를 지킨다";

  state_.devices[0].valid = true;
  NudgeArm();
  Tick();

  ASSERT_TRUE(last_out_.task_link_pose_valid[0]) << "복구 후에도 fingertip 이 안 실린다";
  EXPECT_FALSE(SamePosition(last_out_, good))
      << "게이트가 다시 열렸는데 fingertip 이 옛 pose 에 latch 됐다";
}

// ── 유한 q + 비유한 v: 동역학은 hold, fingertip pose 는 살아 있어야 한다 ──────
//   통합의 위험 지점이다 — 이제 fingertip 이 provider 의 핸들을 읽으므로, UpdateDynamics 가
//   공용 status 에 세운 held 를 그대로 보면 유효한 pose 를 버린다 (운동학 스냅샷이 그것을 막는다).
TEST_F(ClosedChainProjectionTest, NonFiniteVelocityKeepsFingertipPoseAlive) {
  WarmUpUntilFingertipValid();
  const ControllerOutput good = last_out_;

  // 측정 속도만 오염시킨다 (위치는 유한하게 유지 — 운동학은 성립하는 tick).
  for (int i = 0; i < fx::kUr5eArmDof; ++i) {
    state_.devices[0].velocities[static_cast<std::size_t>(i)] =
        std::numeric_limits<double>::quiet_NaN();
  }
  NudgeArm();
  Tick();
  const ControllerOutput& nan_v = last_out_;

  ASSERT_TRUE(nan_v.task_link_pose_valid[0])
      << "비유한 속도가 fingertip pose 를 죽였다 — 운동학 status 분리가 무너졌다";
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_TRUE(std::isfinite(nan_v.task_link_poses[0].position[k])) << "NaN 이 pose 로 샜다";
  }
  EXPECT_FALSE(SamePosition(nan_v, good))
      << "위치는 움직였는데 pose 가 그대로면 dynamics 사유로 hold 된 것이다";
}

// ── configure-time fallback: serial 로봇은 공유하지 않는다 ────────────────────
TEST(ClosedChainProjectionSerialProfile, SerialRobotKeepsOwningPath) {
  auto ctrl = std::make_unique<DemoWbcController>("");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("wbc_serial_profile_test");
  ctrl->SetSystemModelConfig(fx::SharedIiwa7LeapModelConfig());
  ctrl->SetSharedModelBuilder(fx::SharedIiwa7LeapBuilder());
  ctrl->SetControlRate(1.0 / fx::kDt);
  const YAML::Node cfg = ShippedWbcConfig("iiwa7_leap");
  ASSERT_TRUE(cfg);
  ctrl->LoadConfig(cfg);
  ctrl->SetDeviceNameConfigs(fx::MakeIiwa7LeapDeviceConfigs());
  ASSERT_EQ(ctrl->on_configure(rclcpp_lifecycle::State{}, node, cfg),
            DemoWbcController::CallbackReturn::SUCCESS);

  EXPECT_FALSE(ctrl->HandFkSharesProjectionForTesting())
      << "구속 없는 로봇에는 빌릴 사영이 없다 — owning(비활성) 경로여야 한다";
  ControllerState state = fx::MakeIiwa7LeapState();
  for (int i = 0; i < 5; ++i) {
    state.iteration += 1;
    (void)ctrl->Compute(state);
  }
  EXPECT_EQ(ctrl->ReducedProjectionSeqForTesting(), 0U) << "serial 로봇은 사영하지 않는다";
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
