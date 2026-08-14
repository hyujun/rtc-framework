// Aux-lane wiring tests for UdpHandNode (issue #345).
//
// The lane split moves the blocking file-I/O timers (timing CSV drain, stats
// JSON save) off the node's default executor thread — which also serves the
// joint-command subscription — onto a dedicated `hand_aux_io` executor thread.
// The split is load-bearing but *silent* when it breaks: if the aux callback
// group is auto-added to the executor that adds the node, everything still
// runs, on one thread again, and no existing test notices. These tests pin the
// properties that make the split real.
//
// main()'s two-executor wiring itself lives in udp_hand_node.cpp and cannot be
// linked here (it owns main()), so the tests exercise the same rclcpp mechanism
// the wiring depends on: a group the node does not surrender to add_node(), and
// a second executor that can therefore claim it.

#include "udp_hand_driver/udp_hand_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace {

class UdpHandNodeLaneTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  std::shared_ptr<UdpHandNode> MakeNode() { return std::make_shared<UdpHandNode>(); }
};

// The group must exist before any lifecycle transition: main() hands it to the
// aux executor at startup, long before configure. Creating it in on_configure
// would also break reconfiguration — the executor would keep holding the old,
// now-orphaned group and the aux timers would silently stop firing.
TEST_F(UdpHandNodeLaneTest, AuxGroupExistsAfterConstruction) {
  auto node = MakeNode();
  ASSERT_NE(node->GetAuxCallbackGroup(), nullptr);
}

// The load-bearing flag. If this is ever flipped to the rclcpp default (true),
// main()'s `main_executor.add_node()` claims the aux group, both lanes collapse
// onto one thread, and the CSV burst is back in front of the command
// subscription — with every other test still green.
TEST_F(UdpHandNodeLaneTest, AuxGroupIsNotAutomaticallyAddedWithNode) {
  auto node = MakeNode();
  EXPECT_FALSE(node->GetAuxCallbackGroup()->automatically_add_to_executor_with_node());
}

// Behavioural counterpart of the flag test: adding the node to an executor must
// leave the aux group unclaimed, so a second executor can take it. rclcpp
// throws if a group is added to two executors, so a no-throw here IS the
// assertion that the split is achievable.
TEST_F(UdpHandNodeLaneTest, AddNodeLeavesAuxGroupForASecondExecutor) {
  auto node = MakeNode();
  rclcpp::executors::SingleThreadedExecutor main_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  main_executor.add_node(node->get_node_base_interface());
  EXPECT_NO_THROW(aux_executor.add_callback_group(node->GetAuxCallbackGroup(),
                                                  node->get_node_base_interface()));

  aux_executor.remove_callback_group(node->GetAuxCallbackGroup());
  main_executor.remove_node(node->get_node_base_interface());
}

// Order independence: main() claims the aux group *before* add_node, so the
// reverse order must hold too. A future rclcpp that swept unassociated groups
// on add_node regardless of the flag would fail here first.
TEST_F(UdpHandNodeLaneTest, AuxGroupSurvivesBeingClaimedBeforeAddNode) {
  auto node = MakeNode();
  rclcpp::executors::SingleThreadedExecutor main_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  aux_executor.add_callback_group(node->GetAuxCallbackGroup(), node->get_node_base_interface());
  EXPECT_NO_THROW(main_executor.add_node(node->get_node_base_interface()));

  main_executor.remove_node(node->get_node_base_interface());
  aux_executor.remove_callback_group(node->GetAuxCallbackGroup());
}

// The aux slot default mirrors the shell SSoT get_os_cores() == 0. main() reads
// this before spinning, which is why it is declared in the constructor rather
// than on_configure.
TEST_F(UdpHandNodeLaneTest, AuxCpuSlotDefaultsToTheOsSlot) {
  auto node = MakeNode();
  EXPECT_EQ(node->AuxCpuSlot(), 0);
}

TEST_F(UdpHandNodeLaneTest, AuxCpuSlotHonoursParameterOverride) {
  auto node = MakeNode();
  node->set_parameter(rclcpp::Parameter("aux_cpu_slot", 3));
  EXPECT_EQ(node->AuxCpuSlot(), 3);
}

// A negative slot is the repo-wide "do not pin" sentinel (thread_config.hpp
// cpu_core == -1); ApplyThreadConfig skips the affinity step entirely for it.
TEST_F(UdpHandNodeLaneTest, AuxCpuSlotPassesThroughTheNoPinSentinel) {
  auto node = MakeNode();
  node->set_parameter(rclcpp::Parameter("aux_cpu_slot", -1));
  EXPECT_EQ(node->AuxCpuSlot(), udp_hand_driver::kNoPinSlot);
}

// ── use_cpu_affinity contract (issue #345) ──────────────────────────────────

TEST_F(UdpHandNodeLaneTest, CpuAffinityIsEnabledByDefault) {
  auto node = MakeNode();
  EXPECT_TRUE(node->UseCpuAffinity());
}

// The switch has to reach *both* lanes. Before this change the process pinned
// its main thread before the node existed, so the parameter could not be read
// at all and the pin happened regardless.
TEST_F(UdpHandNodeLaneTest, DisablingAffinitySuppressesTheAuxPin) {
  auto node = MakeNode();
  node->set_parameter(rclcpp::Parameter("aux_cpu_slot", 3));
  ASSERT_EQ(node->AuxCpuSlot(), 3);

  node->set_parameter(rclcpp::Parameter("use_cpu_affinity", false));
  EXPECT_EQ(node->AuxCpuSlot(), udp_hand_driver::kNoPinSlot);
}

// ── sil_mode → effective network endpoint ──────────────────────────────────
//
// The p1b hand yaml pins the socket to a real NIC (local_ip/local_interface) so
// hand traffic cannot leave via the same-subnet monitoring NIC. `sil_mode=
// firmware` moves the peer to loopback, which that pinning cannot reach:
// SO_BINDTODEVICE forces egress out the NIC, so connect()/send() both succeed
// while the datagram is dropped — the node would log "UDP socket opened" and
// then time out every recv with nothing naming the cause. The derivation has to
// drop the local binding together with the forced target_ip.

TEST(UdpHandEndpointDerivation, FirmwareSilDropsTheLocalBindingWithTheForcedTargetIp) {
  const auto endpoint =
      udp_hand_driver::DeriveHandEndpoint("firmware", {"192.168.0.100", "192.168.0.46", "enp86s0"});

  EXPECT_EQ(endpoint.target_ip, "127.0.0.1");
  EXPECT_TRUE(endpoint.local_ip.empty());
  EXPECT_TRUE(endpoint.local_interface.empty());
}

// `off` is the real-hardware path the local binding exists for: it must survive
// the derivation untouched, or the feature is a no-op.
TEST(UdpHandEndpointDerivation, RealHardwareKeepsTheConfiguredEndpoint) {
  const auto endpoint =
      udp_hand_driver::DeriveHandEndpoint("off", {"192.168.0.100", "192.168.0.46", "enp86s0"});

  EXPECT_EQ(endpoint.target_ip, "192.168.0.100");
  EXPECT_EQ(endpoint.local_ip, "192.168.0.46");
  EXPECT_EQ(endpoint.local_interface, "enp86s0");
}

// loopmodel opens no socket, so the endpoint is moot — pass it through rather
// than rewrite parameters the operator set.
TEST(UdpHandEndpointDerivation, LoopmodelSilKeepsTheConfiguredEndpoint) {
  const auto endpoint = udp_hand_driver::DeriveHandEndpoint(
      "loopmodel", {"192.168.0.100", "192.168.0.46", "enp86s0"});

  EXPECT_EQ(endpoint.target_ip, "192.168.0.100");
  EXPECT_EQ(endpoint.local_ip, "192.168.0.46");
  EXPECT_EQ(endpoint.local_interface, "enp86s0");
}

// An unbound endpoint (p1a and the generic driver yaml) is already loopback-safe
// in firmware SIL; only target_ip moves.
TEST(UdpHandEndpointDerivation, FirmwareSilOnAnUnboundEndpointOnlyMovesTargetIp) {
  const auto endpoint = udp_hand_driver::DeriveHandEndpoint("firmware", {"192.168.0.100", "", ""});

  EXPECT_EQ(endpoint.target_ip, "127.0.0.1");
  EXPECT_TRUE(endpoint.local_ip.empty());
  EXPECT_TRUE(endpoint.local_interface.empty());
}

TEST(UdpHandPinPolicy, ResolvePinSlotHonoursTheAffinitySwitch) {
  EXPECT_EQ(udp_hand_driver::ResolvePinSlot(7, true), 7);
  EXPECT_EQ(udp_hand_driver::ResolvePinSlot(0, true), 0);
  EXPECT_EQ(udp_hand_driver::ResolvePinSlot(7, false), udp_hand_driver::kNoPinSlot);
  EXPECT_EQ(udp_hand_driver::ResolvePinSlot(0, false), udp_hand_driver::kNoPinSlot);
}

// Already-unpinned stays unpinned — the switch can only remove pinning, never
// introduce it.
TEST(UdpHandPinPolicy, ResolvePinSlotKeepsAnAlreadyNegativeSlotNegative) {
  EXPECT_LT(udp_hand_driver::ResolvePinSlot(udp_hand_driver::kNoPinSlot, true), 0);
  EXPECT_LT(udp_hand_driver::ResolvePinSlot(udp_hand_driver::kNoPinSlot, false), 0);
}

// The sentinel must stay the value ApplyThreadConfig and pinning.py agree on:
// cpu_core < 0 means "skip affinity, keep policy", which is what makes
// use_cpu_affinity:=false leave the CommLoop at SCHED_FIFO 65.
TEST(UdpHandPinPolicy, NoPinSlotMatchesTheRepoWideSentinel) {
  EXPECT_EQ(udp_hand_driver::kNoPinSlot, -1);
}

// The thread config the aux lane is built from: CFS (never preempts the
// CommLoop) and a name that survives pthread_setname_np's 15-char truncation,
// which is how verify_rt_runtime.sh finds the thread in /proc/*/comm.
TEST_F(UdpHandNodeLaneTest, AuxThreadConfigIsNonRealtimeAndNameable) {
  EXPECT_EQ(udp_hand_driver::kHandAuxIoConfig.sched_policy, SCHED_OTHER);
  EXPECT_EQ(udp_hand_driver::kHandAuxIoConfig.sched_priority, 0);
  EXPECT_STREQ(udp_hand_driver::kHandAuxIoConfig.name, "hand_aux_io");
  EXPECT_LE(std::string(udp_hand_driver::kHandAuxIoConfig.name).size(), 15u);
  // Inherit by default; main() overwrites cpu_core from aux_cpu_slot.
  EXPECT_EQ(udp_hand_driver::kHandAuxIoConfig.cpu_core, -1);
}

// ── Pull-model arithmetic (issue #345) ──────────────────────────────────────
// The publish lane stopped being "one callback == one comm cycle" when it
// became a poll. These two pure functions carry that reinterpretation, so they
// are pinned directly rather than through a fully configured node.

TEST(UdpHandPublishMath, CyclesBetweenCountsTwoSequenceStepsPerCycle) {
  EXPECT_EQ(udp_hand_driver::CyclesBetween(0, 0), 0U);
  EXPECT_EQ(udp_hand_driver::CyclesBetween(0, 2), 1U);
  EXPECT_EQ(udp_hand_driver::CyclesBetween(0, 6), 3U);
  EXPECT_EQ(udp_hand_driver::CyclesBetween(100, 110), 5U);
}

// An odd sample means the writer is mid-Store. Counting tolerates it for free
// (integer division drops the low bit), so the load-bearing use is the
// *baseline*: PollAndPublish stores this result, and an odd baseline offsets
// every later difference by one step, losing cycles from then on. Pinned as its
// own step because a mutation that drops the mask is invisible in the counting.
TEST(UdpHandPublishMath, LastCompletedWriteDropsAnInFlightWrite) {
  EXPECT_EQ(udp_hand_driver::LastCompletedWrite(0), 0U);
  EXPECT_EQ(udp_hand_driver::LastCompletedWrite(1), 0U);
  EXPECT_EQ(udp_hand_driver::LastCompletedWrite(4), 4U);
  EXPECT_EQ(udp_hand_driver::LastCompletedWrite(5), 4U);
  EXPECT_EQ(udp_hand_driver::LastCompletedWrite(0xFFFFFFFFU), 0xFFFFFFFEU);
}

// The failure an odd baseline actually causes: a completed cycle reads as zero
// progress, so the publish is skipped and never recovered.
TEST(UdpHandPublishMath, OddBaselineWouldSwallowACompletedCycle) {
  // Correct: baseline masked to the last completed write.
  EXPECT_EQ(udp_hand_driver::CyclesBetween(udp_hand_driver::LastCompletedWrite(5), 6), 1U);
  // What an unmasked baseline would report for the very same tick.
  EXPECT_EQ(udp_hand_driver::CyclesBetween(5, 6), 0U);
}

// A 500 Hz loop wraps uint32 in ~99 days of continuous running; unsigned
// subtraction has to carry it, and a signed reading would go hugely negative.
TEST(UdpHandPublishMath, CyclesBetweenWrapsAroundUint32) {
  constexpr std::uint32_t kNearMax = 0xFFFFFFFEU;
  EXPECT_EQ(udp_hand_driver::CyclesBetween(kNearMax, 0), 1U);
  EXPECT_EQ(udp_hand_driver::CyclesBetween(kNearMax, 4), 3U);
}

TEST(UdpHandPublishMath, AdvanceDecimationFiresOnReachingTheThreshold) {
  int accum = 0;
  EXPECT_FALSE(udp_hand_driver::AdvanceDecimation(accum, 1, 5));
  EXPECT_FALSE(udp_hand_driver::AdvanceDecimation(accum, 3, 5));
  EXPECT_TRUE(udp_hand_driver::AdvanceDecimation(accum, 1, 5));
  EXPECT_EQ(accum, 0);
}

// The reason for `%=` over `= 0`: polls that observe several cycles must not
// throw the surplus away, or the decimated rate drifts below the configured
// one exactly when the consumer is running late.
TEST(UdpHandPublishMath, AdvanceDecimationCarriesTheRemainder) {
  int accum = 0;
  EXPECT_TRUE(udp_hand_driver::AdvanceDecimation(accum, 7, 5));
  EXPECT_EQ(accum, 2);
  EXPECT_TRUE(udp_hand_driver::AdvanceDecimation(accum, 3, 5));
  EXPECT_EQ(accum, 0);
}

// Over a long run the decimated rate must equal cycles/decimation regardless of
// how the cycles are chunked across polls — this is the property the old
// invocation counter had for free and the pull model has to earn.
TEST(UdpHandPublishMath, AdvanceDecimationHoldsRateUnderLumpyPolling) {
  constexpr int kDecimation = 5;
  const std::uint32_t chunks[] = {1, 1, 3, 1, 2, 1, 1, 4, 1, 1, 2, 2, 1, 1, 3, 1};
  int accum = 0;
  int fired = 0;
  std::uint32_t total = 0;
  for (const auto c : chunks) {
    total += c;
    if (udp_hand_driver::AdvanceDecimation(accum, c, kDecimation)) {
      ++fired;
    }
  }
  // 26 cycles / 5 == 5 completed intervals, with 1 cycle of remainder pending.
  EXPECT_EQ(total, 26U);
  EXPECT_EQ(fired, 5);
  EXPECT_EQ(accum, 1);
}

// comm_decimation > 1 lowers the effective state cadence, and link_decimation_
// is derived from it (on_configure). decimation == 1 must publish every cycle.
TEST(UdpHandPublishMath, AdvanceDecimationOfOnePublishesEveryCycle) {
  int accum = 0;
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(udp_hand_driver::AdvanceDecimation(accum, 1, 1));
  }
  EXPECT_EQ(accum, 0);
}

// on_configure clamps to >= 1, but a zero/negative decimation must degrade to
// "publish every cycle" rather than dividing by zero.
TEST(UdpHandPublishMath, AdvanceDecimationTreatsNonPositiveAsEveryCycle) {
  int accum = 0;
  EXPECT_TRUE(udp_hand_driver::AdvanceDecimation(accum, 1, 0));
  EXPECT_TRUE(udp_hand_driver::AdvanceDecimation(accum, 1, -3));
}

// The oversampling constant is the difference between "topic rate tracks
// loop_rate_hz" and "topic rate sits under it"; > 1 is the load-bearing part.
TEST(UdpHandPublishMath, PublishPollOversamplesTheProducer) {
  EXPECT_GT(udp_hand_driver::kPublishPollOversampling, 1.0);
}

}  // namespace
