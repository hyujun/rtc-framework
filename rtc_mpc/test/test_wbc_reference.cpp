/// @file test_wbc_reference.cpp
/// @brief `WbcReference` is the POD handed to a whole-body controller: it
///        must stay trivially copyable, default to "invalid", and survive the
///        byte copy a SeqLock performs with every field intact.

#include "rtc_mpc/types/wbc_reference.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <type_traits>

namespace rtc::mpc {
namespace {

TEST(WbcReference, TriviallyCopyableAndInvalidByDefault) {
  static_assert(std::is_trivially_copyable_v<WbcReference>);
  const WbcReference ref{};
  EXPECT_FALSE(ref.valid);
  EXPECT_FALSE(ref.beyond_horizon);
  EXPECT_EQ(ref.plan_id, 0u);
  EXPECT_EQ(ref.contact_mask, 0u);
  EXPECT_DOUBLE_EQ(ref.q_ref[0], 0.0);
  EXPECT_DOUBLE_EQ(ref.wrench_ref[kMaxContactVars - 1], 0.0);
}

TEST(WbcReference, MemcpyRoundTripKeepsEveryField) {
  WbcReference src{};
  src.nq = kMaxNq;
  src.nv = kMaxNv;
  src.n_contact_vars = kMaxContactVars;
  for (int i = 0; i < kMaxNq; ++i) {
    src.q_ref[static_cast<std::size_t>(i)] = 1.0 + i;
  }
  for (int i = 0; i < kMaxNv; ++i) {
    src.v_ref[static_cast<std::size_t>(i)] = -1.0 - i;
    src.a_ref[static_cast<std::size_t>(i)] = 0.5 * i;
    src.u_fb_acc[static_cast<std::size_t>(i)] = -0.5 * i;
  }
  for (int i = 0; i < kMaxContactVars; ++i) {
    src.wrench_ref[static_cast<std::size_t>(i)] = 10.0 * i;
  }
  src.contact_mask = 0b11u;
  src.observation_stamp_ns = 21;
  src.observation_sequence = 22;
  src.receive_stamp_ns = 23;
  src.plan_id = 24;
  src.model_epoch = 25;
  src.state_epoch = 0xFFFFFFF0u;
  src.observation_source_sequence = 0xFFFFFFFFFFFFFFF1ULL;
  src.schedule_id = 0x8000000000000002ULL;
  src.schedule_epoch = 27;
  src.progress = 0.25;
  src.beyond_horizon = true;
  src.valid = true;

  WbcReference dst{};
  std::memcpy(&dst, &src, sizeof(WbcReference));
  EXPECT_EQ(dst.nq, kMaxNq);
  EXPECT_EQ(dst.nv, kMaxNv);
  EXPECT_EQ(dst.n_contact_vars, kMaxContactVars);
  for (int i = 0; i < kMaxNq; ++i) {
    EXPECT_DOUBLE_EQ(dst.q_ref[static_cast<std::size_t>(i)], 1.0 + i);
  }
  for (int i = 0; i < kMaxNv; ++i) {
    EXPECT_DOUBLE_EQ(dst.v_ref[static_cast<std::size_t>(i)], -1.0 - i);
    EXPECT_DOUBLE_EQ(dst.a_ref[static_cast<std::size_t>(i)], 0.5 * i);
    EXPECT_DOUBLE_EQ(dst.u_fb_acc[static_cast<std::size_t>(i)], -0.5 * i);
  }
  for (int i = 0; i < kMaxContactVars; ++i) {
    EXPECT_DOUBLE_EQ(dst.wrench_ref[static_cast<std::size_t>(i)], 10.0 * i);
  }
  EXPECT_EQ(dst.contact_mask, 0b11u);
  EXPECT_EQ(dst.observation_stamp_ns, 21u);
  EXPECT_EQ(dst.observation_sequence, 22u);
  EXPECT_EQ(dst.receive_stamp_ns, 23u);
  EXPECT_EQ(dst.plan_id, 24u);
  EXPECT_EQ(dst.model_epoch, 25u);
  EXPECT_EQ(dst.state_epoch, 0xFFFFFFF0u);
  EXPECT_EQ(dst.observation_source_sequence, 0xFFFFFFFFFFFFFFF1ULL);
  EXPECT_EQ(dst.schedule_id, 0x8000000000000002ULL);
  EXPECT_EQ(dst.schedule_epoch, 27u);
  EXPECT_DOUBLE_EQ(dst.progress, 0.25);
  EXPECT_TRUE(dst.beyond_horizon);
  EXPECT_TRUE(dst.valid);
}

}  // namespace
}  // namespace rtc::mpc
