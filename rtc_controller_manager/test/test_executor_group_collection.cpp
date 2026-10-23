// rclcpp executor contract RtControllerMain relies on (#935).
//
// Phase 3 hands the node's three callback groups to dedicated executors with
// add_callback_group(), then add_node()s the node — and each controller node —
// to nrt_callback_executor so the rest is served there. Whether that split
// survives a later configure rests on three rclcpp behaviours, pinned here
// against the rclcpp this workspace builds with:
//   - an executor that add_node()d a node also collects the groups created on
//     it LATER, unless another executor already claimed them;
//   - a node no executor was given is never served;
//   - remove_node() of a node the executor was never given throws.
// The first is why a post-Phase-3 configure would put everything it creates on
// nrt_callback; the last two are why its controller nodes would go dead and
// shutdown would abort. RtControllerNode refuses that configure instead.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>

namespace {

using namespace std::chrono_literals;

// The threads a callback has run on.
class Seen {
 public:
  void Mark() {
    const std::lock_guard<std::mutex> lock(mutex_);
    threads_.insert(std::this_thread::get_id());
  }

  std::set<std::thread::id> Threads() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return threads_;
  }

 private:
  mutable std::mutex mutex_;
  std::set<std::thread::id> threads_;
};

bool WaitFor(const std::function<bool()>& done, std::chrono::milliseconds limit = 2s) {
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return done();
}

class ExecutorGroupCollectionTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ExecutorGroupCollectionTest, AnAddedNodesLaterGroupsGoToItsExecutorUnlessAlreadyClaimed) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("group_collection_node");
  const auto claimed = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Phase 3's order: claim the dedicated group first, then add the node.
  rclcpp::executors::SingleThreadedExecutor dedicated;
  dedicated.add_callback_group(claimed, node->get_node_base_interface());
  rclcpp::executors::SingleThreadedExecutor general;
  general.add_node(node->get_node_base_interface());

  Seen on_claimed;
  Seen on_later;
  const auto claimed_timer = node->create_wall_timer(5ms, [&] { on_claimed.Mark(); }, claimed);
  // A group created after both executors took their share, as a configure
  // after Phase 3 creates them.
  const auto later = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto later_timer = node->create_wall_timer(5ms, [&] { on_later.Mark(); }, later);

  std::thread dedicated_thread([&] { dedicated.spin(); });
  std::thread general_thread([&] { general.spin(); });
  const auto dedicated_id = dedicated_thread.get_id();
  const auto general_id = general_thread.get_id();

  const bool ran =
      WaitFor([&] { return !on_claimed.Threads().empty() && !on_later.Threads().empty(); });
  dedicated.cancel();
  general.cancel();
  dedicated_thread.join();
  general_thread.join();

  ASSERT_TRUE(ran) << "a timer never fired";
  EXPECT_EQ(std::set<std::thread::id>{dedicated_id}, on_claimed.Threads());
  EXPECT_EQ(std::set<std::thread::id>{general_id}, on_later.Threads())
      << "a group created after add_node did not land on the add_node executor";
}

TEST_F(ExecutorGroupCollectionTest, ANodeNoExecutorWasGivenIsNeverServed) {
  auto served = std::make_shared<rclcpp_lifecycle::LifecycleNode>("group_collection_served");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(served->get_node_base_interface());

  // Created after the executor took its nodes, as a controller node a
  // post-Phase-3 configure creates.
  auto late = std::make_shared<rclcpp_lifecycle::LifecycleNode>("group_collection_late");
  std::atomic<int> served_ticks{0};
  std::atomic<int> late_ticks{0};
  const auto served_timer = served->create_wall_timer(5ms, [&] { served_ticks.fetch_add(1); });
  const auto late_timer = late->create_wall_timer(5ms, [&] { late_ticks.fetch_add(1); });

  std::thread spin([&] { executor.spin(); });
  const bool ran = WaitFor([&] { return served_ticks.load() >= 10; });
  executor.cancel();
  spin.join();

  ASSERT_TRUE(ran) << "the served node's timer never fired";
  EXPECT_EQ(0, late_ticks.load());
}

TEST_F(ExecutorGroupCollectionTest, RemovingANodeTheExecutorWasNeverGivenThrows) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("group_collection_never_added");
  rclcpp::executors::SingleThreadedExecutor executor;
  EXPECT_THROW(executor.remove_node(node->get_node_base_interface()), std::runtime_error);
}

}  // namespace
