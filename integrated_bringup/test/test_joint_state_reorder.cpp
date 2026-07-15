// Tests for the fixed-capacity JointStateReorder helper (issue #156).
//
// Pins two contracts:
//   1. Behavior parity with the former lazy std::vector reorder — named
//      (shuffled) / un-named / unknown-joint / empty-ref / over-capacity
//      inputs produce the same DeviceStateCache content, and num_channels is
//      clamped to kMaxDeviceChannels (the RT loop copy_n's num_channels
//      elements out of the fixed arrays).
//   2. Allocation-freedom — BuildJointStateReorder + WriteJointStateToCache
//      run inside the rt_callback lane (RT callback rule: mailbox-only), so
//      neither may touch the heap. Guarded by a TU-global operator new
//      counter sampled around the calls.

#include "integrated_bringup/backends/joint_state_reorder.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

// ── TU-global allocation counter ─────────────────────────────────────────────
// Replaces the global plain (non-aligned) new/delete pair. All overloads
// route through std::malloc/std::free so new/delete stay a matched pair
// (GCC -Wmismatched-new-delete fires on mixed default/custom pairs).
namespace {
std::atomic<std::size_t> g_alloc_count{0};
}  // namespace

void* operator new(std::size_t size) {
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(size))
    return p;
  throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
  g_alloc_count.fetch_add(1, std::memory_order_relaxed);
  if (void* p = std::malloc(size))
    return p;
  throw std::bad_alloc();
}

void operator delete(void* p) noexcept {
  std::free(p);
}

void operator delete[](void* p) noexcept {
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}

namespace {

constexpr auto kCap = static_cast<std::size_t>(rtc::kMaxDeviceChannels);

sensor_msgs::msg::JointState MakeMsg(const std::vector<std::string>& names,
                                     const std::vector<double>& pos,
                                     const std::vector<double>& vel = {},
                                     const std::vector<double>& eff = {}) {
  sensor_msgs::msg::JointState msg;
  msg.name = names;
  msg.position = pos;
  msg.velocity = vel;
  msg.effort = eff;
  return msg;
}

// 1. Named, shuffled: map[msg_idx] == device slot of that joint name.
TEST(BuildJointStateReorder, NamedShuffled_MapsToDeviceOrder) {
  rtc::JointStateReorder r;
  BuildJointStateReorder({"j2", "j0", "j1"}, {"j0", "j1", "j2"}, r);
  ASSERT_EQ(r.size, 3);
  EXPECT_EQ(r.map[0], 2);
  EXPECT_EQ(r.map[1], 0);
  EXPECT_EQ(r.map[2], 1);
}

// 2. Unknown joint name → -1 (slot skipped at write time).
TEST(BuildJointStateReorder, UnknownName_MarksMinusOne) {
  rtc::JointStateReorder r;
  BuildJointStateReorder({"j0", "ghost", "j2"}, {"j0", "j1", "j2"}, r);
  ASSERT_EQ(r.size, 3);
  EXPECT_EQ(r.map[0], 0);
  EXPECT_EQ(r.map[1], -1);
  EXPECT_EQ(r.map[2], 2);
}

// 3. Empty reference (joint_command_names unset) → size stays 0 (identity).
TEST(BuildJointStateReorder, EmptyRef_LeavesIdentity) {
  rtc::JointStateReorder r;
  BuildJointStateReorder({"j0", "j1"}, {}, r);
  EXPECT_EQ(r.size, 0);
}

// 4. More message names than capacity → clamped, no out-of-bounds.
TEST(BuildJointStateReorder, OverCapacity_ClampsToMaxDeviceChannels) {
  std::vector<std::string> names;
  std::vector<std::string> ref;
  for (std::size_t i = 0; i < kCap + 6; ++i)
    names.push_back("j" + std::to_string(i));
  for (std::size_t i = 0; i < kCap; ++i)
    ref.push_back("j" + std::to_string(i));

  rtc::JointStateReorder r;
  BuildJointStateReorder(names, ref, r);
  ASSERT_EQ(r.size, rtc::kMaxDeviceChannels);
  EXPECT_EQ(r.map[0], 0);
  EXPECT_EQ(r.map[kCap - 1], static_cast<int>(kCap - 1));
}

// 5. Reordered write: values land in device-slot order; unknown slot keeps
//    its previous cache value.
TEST(WriteJointStateToCache, WithReorder_PlacesByDeviceSlot) {
  rtc::JointStateReorder r;
  BuildJointStateReorder({"j2", "ghost", "j1"}, {"j0", "j1", "j2"}, r);

  const auto msg =
      MakeMsg({"j2", "ghost", "j1"}, {12.0, 99.0, 11.0}, {22.0, 99.0, 21.0}, {32.0, 99.0, 31.0});
  rtc::DeviceStateCache ds{};
  ds.positions[0] = -5.0;  // untouched slot sentinel (j0 absent from msg)

  WriteJointStateToCache(msg, r, ds);

  EXPECT_TRUE(ds.valid);
  EXPECT_EQ(ds.num_channels, 3);
  EXPECT_DOUBLE_EQ(ds.positions[0], -5.0);  // no source → keeps sentinel
  EXPECT_DOUBLE_EQ(ds.positions[1], 11.0);
  EXPECT_DOUBLE_EQ(ds.positions[2], 12.0);
  EXPECT_DOUBLE_EQ(ds.velocities[1], 21.0);
  EXPECT_DOUBLE_EQ(ds.velocities[2], 22.0);
  EXPECT_DOUBLE_EQ(ds.efforts[1], 31.0);
  EXPECT_DOUBLE_EQ(ds.efforts[2], 32.0);
}

// 6. Identity write (size == 0): direct copy, shorter velocity/effort lanes
//    only fill what the message carries.
TEST(WriteJointStateToCache, IdentityWhenNoReorder_PartialLanes) {
  const rtc::JointStateReorder r{};                       // size == 0
  const auto msg = MakeMsg({}, {1.0, 2.0, 3.0}, {10.0});  // no effort lane

  rtc::DeviceStateCache ds{};
  WriteJointStateToCache(msg, r, ds);

  EXPECT_TRUE(ds.valid);
  EXPECT_EQ(ds.num_channels, 3);
  EXPECT_DOUBLE_EQ(ds.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(ds.positions[2], 3.0);
  EXPECT_DOUBLE_EQ(ds.velocities[0], 10.0);
  EXPECT_DOUBLE_EQ(ds.velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(ds.efforts[0], 0.0);
}

// 7. Oversized message: num_channels clamps to kMaxDeviceChannels so the RT
//    loop's copy_n(cache.positions.data(), num_channels, ...) stays in
//    bounds.
TEST(WriteJointStateToCache, OverCapacity_ClampsNumChannels) {
  const rtc::JointStateReorder r{};
  std::vector<double> pos(kCap + 6, 0.0);
  pos[kCap - 1] = 7.0;
  const auto msg = MakeMsg({}, pos);

  rtc::DeviceStateCache ds{};
  WriteJointStateToCache(msg, r, ds);

  EXPECT_EQ(ds.num_channels, rtc::kMaxDeviceChannels);
  EXPECT_DOUBLE_EQ(ds.positions[kCap - 1], 7.0);
}

// 8. Allocation guard: build + write must not touch the heap (RT callback
//    rule). Counter sampled tightly around the calls — message construction
//    above is allowed to allocate, the helpers are not.
TEST(JointStateReorderRtGuard, BuildAndWrite_AllocationFree) {
  const std::vector<std::string> ref = {"j0", "j1", "j2", "j3", "j4", "j5"};
  const auto msg = MakeMsg({"j5", "j4", "j3", "j2", "j1", "j0"}, {5.0, 4.0, 3.0, 2.0, 1.0, 0.0},
                           {50.0, 40.0, 30.0, 20.0, 10.0, 0.0});
  rtc::JointStateReorder r;
  rtc::DeviceStateCache ds{};

  const std::size_t before = g_alloc_count.load(std::memory_order_relaxed);
  BuildJointStateReorder(msg.name, ref, r);
  WriteJointStateToCache(msg, r, ds);
  const std::size_t after = g_alloc_count.load(std::memory_order_relaxed);

  EXPECT_EQ(after - before, 0U) << "reorder helpers allocated on the RT lane";
  ASSERT_EQ(r.size, 6);
  // Reversed-name message: j0's value (0.0) lands in device slot 0, j5's
  // value (5.0) in slot 5 — an identity copy would give positions[0] == 5.0.
  EXPECT_DOUBLE_EQ(ds.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(ds.positions[5], 5.0);
}

}  // namespace
