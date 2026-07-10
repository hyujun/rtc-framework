// ── serial_test_support.hpp ─────────────────────────────────────────────────
// Shared pseudo-terminal (PTY) loopback helper for the RS485 serial tests.
//
// A PTY pair stands in for a real RS485 link: the master fd is the "device
// side" (the test writes bytes that SerialPort/Rs485Transport should receive,
// and reads back bytes they transmit), while the slave path is handed to
// SerialPort::Open() / Rs485TransportConfig::device. PTYs are always available
// (no root, no kernel module), so unlike the vcan-backed CAN tests these run
// unconditionally in CI.
//
// A PTY is not a real UART: TIOCSRS485 is unsupported (EnableRs485 returns
// false, which the transport tolerates) and the baud rate is ignored. Framing,
// timeout, and byte-transparency behaviour are all faithfully exercised.
// ─────────────────────────────────────────────────────────────────────────────
#ifndef RTC_COMMUNICATION_TEST_SERIAL_TEST_SUPPORT_HPP_
#define RTC_COMMUNICATION_TEST_SERIAL_TEST_SUPPORT_HPP_

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

namespace rtc::test {

struct PtyPair {
  int master_fd{-1};
  std::string slave_path;
  bool ok{false};
};

// Opens a PTY master/slave pair. Returns ok=false on failure (caller skips or
// fails the test). The master fd must be closed by the caller; the slave is
// owned by the SerialPort/Rs485Transport under test.
[[nodiscard]] inline PtyPair OpenPtyPair() {
  PtyPair pair;
  const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0)
    return pair;
  if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
    ::close(master);
    return pair;
  }
  const char* name = ::ptsname(master);
  if (name == nullptr) {
    ::close(master);
    return pair;
  }
  pair.master_fd = master;
  pair.slave_path = name;
  pair.ok = true;
  return pair;
}

}  // namespace rtc::test

#endif  // RTC_COMMUNICATION_TEST_SERIAL_TEST_SUPPORT_HPP_
