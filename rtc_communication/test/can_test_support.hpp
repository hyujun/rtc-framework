// ── can_test_support.hpp ────────────────────────────────────────────────────
// Shared vcan detection and skip helpers for the CAN/CANFD loopback tests.
//
// Local setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set up vcan0
// ─────────────────────────────────────────────────────────────────────────────
#ifndef RTC_COMMUNICATION_TEST_CAN_TEST_SUPPORT_HPP_
#define RTC_COMMUNICATION_TEST_CAN_TEST_SUPPORT_HPP_

#include <gtest/gtest.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace rtc::test {

inline constexpr char kVcanInterface[] = "vcan0";

[[nodiscard]] inline bool HasVcan() {
  return if_nametoindex(kVcanInterface) != 0;
}

// Returns the interface MTU, or -1 if the interface does not exist.
[[nodiscard]] inline int GetInterfaceMtu(const char* name) {
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0)
    return -1;
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
  const int rc = ::ioctl(fd, SIOCGIFMTU, &ifr);
  ::close(fd);
  return (rc < 0) ? -1 : ifr.ifr_mtu;
}

}  // namespace rtc::test

#define SKIP_IF_NO_VCAN()                                         \
  if (!rtc::test::HasVcan()) {                                    \
    GTEST_SKIP() << "vcan0 not available (sudo modprobe vcan && " \
                    "sudo ip link add dev vcan0 type vcan && "    \
                    "sudo ip link set up vcan0)";                 \
  }

#define SKIP_IF_NO_FD_VCAN()                                                                 \
  SKIP_IF_NO_VCAN();                                                                         \
  if (rtc::test::GetInterfaceMtu(rtc::test::kVcanInterface) < static_cast<int>(CANFD_MTU)) { \
    GTEST_SKIP() << "vcan0 is not CAN FD capable (MTU < " << CANFD_MTU                       \
                 << "); sudo ip link set vcan0 mtu 72";                                      \
  }

#endif  // RTC_COMMUNICATION_TEST_CAN_TEST_SUPPORT_HPP_
