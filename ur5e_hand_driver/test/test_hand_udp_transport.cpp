// Unit tests for hand_udp_transport.hpp — UDP transport and HandCommStats.
//
// Tier 1.5: Tests construction, initial state, and HandCommStats defaults.
// Socket-level tests use loopback where safe, no real hand hardware needed.

#include <gtest/gtest.h>

#include "ur5e_hand_driver/hand_udp_transport.hpp"

#include <cstdint>
#include <string>

namespace rtc::test {

// ── HandCommStats defaults ─────────────────────────────────────────────────

TEST(HandCommStats, DefaultValues)
{
  HandCommStats stats{};
  EXPECT_EQ(stats.recv_ok, 0u);
  EXPECT_EQ(stats.recv_timeout, 0u);
  EXPECT_EQ(stats.recv_error, 0u);
  EXPECT_EQ(stats.cmd_mismatch, 0u);
  EXPECT_EQ(stats.mode_mismatch, 0u);
  EXPECT_EQ(stats.total_cycles, 0u);
  EXPECT_EQ(stats.event_skip_count, 0u);
}

// ── Construction ───────────────────────────────────────────────────────────

TEST(HandUdpTransport, Construction_NotOpen)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  EXPECT_FALSE(transport.is_open());
}

TEST(HandUdpTransport, RecvTimeoutMs_StoredCorrectly)
{
  HandUdpTransport transport("127.0.0.1", 55151, 0.4);
  EXPECT_DOUBLE_EQ(transport.recv_timeout_ms(), 0.4);
}

TEST(HandUdpTransport, CommStats_InitiallyZero)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  const auto& stats = transport.comm_stats();
  EXPECT_EQ(stats.recv_ok, 0u);
  EXPECT_EQ(stats.recv_timeout, 0u);
  EXPECT_EQ(stats.recv_error, 0u);
  EXPECT_EQ(stats.cmd_mismatch, 0u);
  EXPECT_EQ(stats.total_cycles, 0u);
}

TEST(HandUdpTransport, RecvErrorCount_InitiallyZero)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  EXPECT_EQ(transport.recv_error_count(), 0u);
}

// ── Open / Close lifecycle ─────────────────────────────────────────────────

TEST(HandUdpTransport, OpenClose_Loopback)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  ASSERT_TRUE(transport.Open());
  EXPECT_TRUE(transport.is_open());
  transport.Close();
  EXPECT_FALSE(transport.is_open());
}

TEST(HandUdpTransport, DoubleClose_Safe)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  ASSERT_TRUE(transport.Open());
  transport.Close();
  transport.Close();  // Should not crash
  EXPECT_FALSE(transport.is_open());
}

TEST(HandUdpTransport, Open_InvalidIP_Fails)
{
  HandUdpTransport transport("999.999.999.999", 55151, 10.0);
  EXPECT_FALSE(transport.Open());
  EXPECT_FALSE(transport.is_open());
}

TEST(HandUdpTransport, DestructorClosesSocket)
{
  {
    HandUdpTransport transport("127.0.0.1", 55151, 10.0);
    ASSERT_TRUE(transport.Open());
    // Destructor should close without crash
  }
}

// ── DrainStaleResponses on empty socket ────────────────────────────────────

TEST(HandUdpTransport, DrainStaleResponses_EmptySocket)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  ASSERT_TRUE(transport.Open());
  // Draining an empty socket should not block or crash
  transport.DrainStaleResponses();
  transport.Close();
}

// ── CommStats mutable access ───────────────────────────────────────────────

TEST(HandUdpTransport, CommStatsMut_Writable)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  transport.comm_stats_mut().total_cycles = 42;
  EXPECT_EQ(transport.comm_stats().total_cycles, 42u);
}

// ── WritePositionFireAndForget on open socket ──────────────────────────────

TEST(HandUdpTransport, WritePositionFireAndForget_NoRecv)
{
  HandUdpTransport transport("127.0.0.1", 55151, 10.0);
  ASSERT_TRUE(transport.Open());

  std::array<float, kNumHandMotors> cmd{};
  cmd[0] = 1.0f;
  // Fire-and-forget: should not block or crash (no receiver)
  transport.WritePositionFireAndForget(cmd);
  transport.Close();
}

}  // namespace rtc::test
