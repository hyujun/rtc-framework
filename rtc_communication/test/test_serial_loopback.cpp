// ── test_serial_loopback.cpp ────────────────────────────────────────────────
// Integration tests for rtc::SerialPort over a PTY loopback. Validates the
// line-level contract that Rs485Transport builds on: raw-mode line config,
// byte-transparent I/O, VMIN/VTIME receive timeout, and RAII fd cleanup.
//
// Covers:
//   - Lifecycle: default-constructed is closed; Open() of a bad path fails
//   - ConfigureLine() succeeds on a PTY (8N1 raw)
//   - Byte round-trip both directions (master <-> SerialPort)
//   - VTIME timeout returns 0 without blocking indefinitely
//   - RAII destructor closes the underlying fd
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_communication/rs485/serial_port.hpp"
#include "serial_test_support.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>

namespace {

using rtc::SerialParity;
using rtc::SerialPort;
using rtc::test::OpenPtyPair;
using rtc::test::PtyPair;

// ── SerialPort: lifecycle ───────────────────────────────────────────────────
TEST(SerialPortTest, DefaultConstructedIsNotOpen) {
  SerialPort port;
  EXPECT_FALSE(port.is_open());
  EXPECT_LT(port.fd(), 0);
}

TEST(SerialPortTest, OpenNonexistentDeviceFails) {
  SerialPort port;
  EXPECT_FALSE(port.Open("/dev/nonexistent-rtc-serial-999"));
  EXPECT_FALSE(port.is_open());
}

// ── SerialPort: line configuration on a PTY ─────────────────────────────────
TEST(SerialPortTest, ConfigureLineSucceedsOnPty) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  SerialPort port;
  ASSERT_TRUE(port.Open(pty.slave_path));
  EXPECT_TRUE(port.ConfigureLine(57600, 8, SerialParity::kNone, 1));

  ::close(pty.master_fd);
}

// ── SerialPort: RAII closes fd on destruction ───────────────────────────────
TEST(SerialPortTest, DestructorClosesFd) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  int captured_fd = -1;
  {
    SerialPort port;
    ASSERT_TRUE(port.Open(pty.slave_path));
    captured_fd = port.fd();
    ASSERT_GE(captured_fd, 0);
    EXPECT_GE(::fcntl(captured_fd, F_GETFD), 0);
  }
  errno = 0;
  EXPECT_EQ(::fcntl(captured_fd, F_GETFD), -1);
  EXPECT_EQ(errno, EBADF);

  ::close(pty.master_fd);
}

// ── SerialPort: byte round-trip master -> SerialPort ────────────────────────
TEST(SerialPortTest, ReadReceivesBytesFromMaster) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  SerialPort port;
  ASSERT_TRUE(port.Open(pty.slave_path));
  ASSERT_TRUE(port.ConfigureLine(115200, 8, SerialParity::kNone, 1));
  port.SetRecvTimeout(500);

  const std::array<uint8_t, 6> payload{0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34};
  ASSERT_EQ(::write(pty.master_fd, payload.data(), payload.size()),
            static_cast<ssize_t>(payload.size()));

  std::array<uint8_t, 32> buf{};
  ssize_t total = 0;
  while (total < static_cast<ssize_t>(payload.size())) {
    const ssize_t n = port.Read(
        std::span<uint8_t>(buf.data() + total, buf.size() - static_cast<std::size_t>(total)));
    ASSERT_GT(n, 0);
    total += n;
  }
  EXPECT_EQ(total, static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0, std::memcmp(buf.data(), payload.data(), payload.size()));

  ::close(pty.master_fd);
}

// ── SerialPort: byte round-trip SerialPort -> master ────────────────────────
TEST(SerialPortTest, WriteReachesMaster) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  SerialPort port;
  ASSERT_TRUE(port.Open(pty.slave_path));
  ASSERT_TRUE(port.ConfigureLine(115200, 8, SerialParity::kNone, 1));

  const std::array<uint8_t, 5> payload{0x01, 0x02, 0x03, 0x04, 0x05};
  ASSERT_EQ(port.Write(payload), static_cast<ssize_t>(payload.size()));
  port.Drain();

  std::array<uint8_t, 32> buf{};
  ssize_t total = 0;
  while (total < static_cast<ssize_t>(payload.size())) {
    const ssize_t n =
        ::read(pty.master_fd, buf.data() + total, buf.size() - static_cast<std::size_t>(total));
    ASSERT_GT(n, 0);
    total += n;
  }
  EXPECT_EQ(0, std::memcmp(buf.data(), payload.data(), payload.size()));

  ::close(pty.master_fd);
}

// ── SerialPort: VTIME timeout returns 0 without blocking forever ────────────
TEST(SerialPortTest, RecvTimeoutReturnsZero) {
  PtyPair pty = OpenPtyPair();
  ASSERT_TRUE(pty.ok);

  SerialPort port;
  ASSERT_TRUE(port.Open(pty.slave_path));
  ASSERT_TRUE(port.ConfigureLine(115200, 8, SerialParity::kNone, 1));
  port.SetRecvTimeout(100);  // ~1 decisecond

  std::array<uint8_t, 16> buf{};
  const auto t0 = std::chrono::steady_clock::now();
  const ssize_t n = port.Read(buf);
  const auto t1 = std::chrono::steady_clock::now();

  EXPECT_LE(n, 0);  // 0 on VTIME expiry with no data
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  EXPECT_GE(elapsed_ms, 50);
  EXPECT_LT(elapsed_ms, 2000);

  ::close(pty.master_fd);
}

}  // namespace
