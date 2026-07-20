// ── test_activation_generation.cpp ───────────────────────────────────────────
// Base-level contract for the activation generation gate (issue #196 §3).
//
// rclcpp_lifecycle gates publishers only — a controller's target subscriptions
// stay alive while it is Inactive, so goals addressed to a deactivated
// controller keep landing on its pending-target queue. The queue cannot be
// flushed from the lifecycle thread (SpscQueue has exactly one consumer, the RT
// thread), so instead every activation bumps a generation counter, the off-RT
// push stamps it, and the RT drain discards entries whose stamp is stale.
//
// This file pins the base half of that contract:
//   - the counter starts at 0 and advances once per on_activate;
//   - IsCurrentGeneration accepts only the stamp of the current activation;
//   - ResetTargetInitialization() fires on every activation, which is what
//     forces each controller's first post-activation tick to re-seed its hold
//     target from the current device state.
//
// The per-controller half (that each concrete drain loop actually honours the
// stamp) lives in rtc_controllers/test/test_activation_generation_gate.cpp and
// integrated_bringup/test/test_activation_generation_gate.cpp.
// ─────────────────────────────────────────────────────────────────────────────
#include <rtc_controller_interface/rt_controller_interface.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace {

const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive");

// Minimal controller reproducing the marshal/drain idiom the seven production
// controllers share: SetDeviceTarget stamps the generation it observes, and a
// drain step keeps only the entries still belonging to the current activation.
class GenerationStubController : public rtc::RTControllerInterface {
 public:
  struct Entry {
    double value{0.0};
    std::uint32_t generation{0};
  };

  [[nodiscard]] rtc::ControllerOutput Compute(const rtc::ControllerState&) noexcept override {
    rtc::ControllerOutput out{};
    out.valid = true;
    return out;
  }

  void SetDeviceTarget(int, std::span<const double> target) noexcept override {
    pending_.push_back(Entry{target.empty() ? 0.0 : target[0], ActivationGeneration()});
  }

  [[nodiscard]] std::string_view Name() const noexcept override {
    return "GenerationStubController";
  }

  // Mirrors the RT-side drain predicate used by every concrete controller.
  [[nodiscard]] std::vector<double> DrainAccepted() {
    std::vector<double> accepted;
    for (const auto& entry : pending_) {
      if (IsCurrentGeneration(entry.generation)) {
        accepted.push_back(entry.value);
      }
    }
    pending_.clear();
    return accepted;
  }

  int reset_count{0};

 protected:
  void ResetTargetInitialization() noexcept override { ++reset_count; }

 private:
  std::vector<Entry> pending_;
};

}  // namespace

TEST(ActivationGenerationTest, StartsAtZeroBeforeAnyActivation) {
  GenerationStubController ctrl;
  EXPECT_EQ(ctrl.ActivationGeneration(), 0U);
  EXPECT_TRUE(ctrl.IsCurrentGeneration(0U));
}

TEST(ActivationGenerationTest, AdvancesOnceEveryActivation) {
  GenerationStubController ctrl;

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.ActivationGeneration(), 1U);

  ASSERT_EQ(ctrl.on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.ActivationGeneration(), 1U) << "deactivation must not consume a generation";

  ASSERT_EQ(ctrl.on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl.ActivationGeneration(), 2U);
}

TEST(ActivationGenerationTest, PreviousGenerationStampIsNoLongerCurrent) {
  GenerationStubController ctrl;
  (void)ctrl.on_activate(kInactive);
  const std::uint32_t stamp = ctrl.ActivationGeneration();

  EXPECT_TRUE(ctrl.IsCurrentGeneration(stamp));
  (void)ctrl.on_activate(kInactive);
  EXPECT_FALSE(ctrl.IsCurrentGeneration(stamp));
}

TEST(ActivationGenerationTest, ResetTargetInitializationRunsOnEveryActivation) {
  GenerationStubController ctrl;
  EXPECT_EQ(ctrl.reset_count, 0);

  (void)ctrl.on_activate(kInactive);
  EXPECT_EQ(ctrl.reset_count, 1);

  (void)ctrl.on_deactivate(kInactive);
  EXPECT_EQ(ctrl.reset_count, 1) << "only activation re-arms the self-init latch";

  (void)ctrl.on_activate(kInactive);
  EXPECT_EQ(ctrl.reset_count, 2);
}

// The behavioural statement of the acceptance criterion: a goal delivered while
// the controller is Inactive is not consumed after it is re-activated, while a
// goal delivered afterwards is.
TEST(ActivationGenerationTest, TargetQueuedWhileInactiveIsDroppedOnReactivation) {
  GenerationStubController ctrl;
  (void)ctrl.on_activate(kInactive);

  const std::vector<double> live{1.0};
  ctrl.SetDeviceTarget(0, live);
  EXPECT_EQ(ctrl.DrainAccepted().size(), 1U) << "targets sent while active must be honoured";

  (void)ctrl.on_deactivate(kInactive);
  const std::vector<double> stale{2.0};
  ctrl.SetDeviceTarget(0, stale);  // subscription is still alive while Inactive

  (void)ctrl.on_activate(kInactive);
  EXPECT_TRUE(ctrl.DrainAccepted().empty()) << "stale target survived the Inactive→Active cycle";

  const std::vector<double> fresh{3.0};
  ctrl.SetDeviceTarget(0, fresh);
  const auto accepted = ctrl.DrainAccepted();
  ASSERT_EQ(accepted.size(), 1U);
  EXPECT_DOUBLE_EQ(accepted[0], 3.0);
}

// Targets that predate the very first activation are covered by the same rule:
// generation 0 is invalidated the moment CM activates the controller.
TEST(ActivationGenerationTest, TargetQueuedBeforeFirstActivationIsDropped) {
  GenerationStubController ctrl;
  const std::vector<double> early{9.0};
  ctrl.SetDeviceTarget(0, early);

  (void)ctrl.on_activate(kInactive);
  EXPECT_TRUE(ctrl.DrainAccepted().empty());
}

// Controllers driven directly by unit tests never activate; both sides observe
// generation 0, so nothing is dropped and the legacy test path keeps working.
TEST(ActivationGenerationTest, NeverActivatedControllerAcceptsTargets) {
  GenerationStubController ctrl;
  const std::vector<double> target{4.0};
  ctrl.SetDeviceTarget(0, target);

  const auto accepted = ctrl.DrainAccepted();
  ASSERT_EQ(accepted.size(), 1U);
  EXPECT_DOUBLE_EQ(accepted[0], 4.0);
}
