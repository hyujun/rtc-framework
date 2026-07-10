#ifndef RTC_COMMUNICATION_CAN_CAN_SOCKET_HPP_
#define RTC_COMMUNICATION_CAN_CAN_SOCKET_HPP_

// RAII wrapper for a Linux SocketCAN raw socket (PF_CAN, SOCK_RAW, CAN_RAW).
//
// Designed for real-time use:
//   - No dynamic allocation after construction
//   - All I/O methods are noexcept
//   - Deterministic cleanup in destructor
//
// Unlike UDP (separate bind/connect sockets), a single CAN raw socket bound to
// an interface both sends and receives on that interface.

#include "rtc_communication/socket_options.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>

namespace rtc {

class CanSocket {
 public:
  CanSocket() noexcept = default;

  ~CanSocket() { Close(); }

  // Non-copyable, non-movable (owns raw fd).
  CanSocket(const CanSocket&) = delete;
  CanSocket& operator=(const CanSocket&) = delete;
  CanSocket(CanSocket&&) = delete;
  CanSocket& operator=(CanSocket&&) = delete;

  // -- Socket lifecycle ------------------------------------------------------

  // Creates the underlying CAN_RAW socket. Returns false on failure.
  [[nodiscard]] bool Open() noexcept {
    if (fd_ >= 0)
      return true;  // already open
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    return fd_ >= 0;
  }

  // Binds to a CAN interface by name (e.g. "can0", "vcan0").
  // The bound socket both sends and receives on this interface.
  [[nodiscard]] bool Bind(std::string_view interface_name) noexcept {
    if (fd_ < 0 && !Open())
      return false;

    // if_nametoindex needs a null-terminated string.
    char name_buf[IFNAMSIZ]{};
    const auto len = std::min(interface_name.size(), sizeof(name_buf) - 1);
    std::memcpy(name_buf, interface_name.data(), len);

    const unsigned int ifindex = if_nametoindex(name_buf);
    if (ifindex == 0) {
      Close();
      return false;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = static_cast<int>(ifindex);

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      Close();
      return false;
    }
    return true;
  }

  void Close() noexcept {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // -- Socket options --------------------------------------------------------

  // Restricts reception to frames matching (can_id & mask) == (rx_id & mask).
  [[nodiscard]] bool SetFilter(canid_t can_id, canid_t can_mask) noexcept {
    if (fd_ < 0)
      return false;
    can_filter filter{};
    filter.can_id = can_id;
    filter.can_mask = can_mask;
    return setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) == 0;
  }

  // Controls local echo of sent frames to other sockets (kernel default: on).
  [[nodiscard]] bool SetLoopback(bool enable) noexcept {
    if (fd_ < 0)
      return false;
    const int val = enable ? 1 : 0;
    return setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &val, sizeof(val)) == 0;
  }

  // Controls whether this socket receives its own sent frames (kernel default: off).
  [[nodiscard]] bool SetRecvOwnMessages(bool enable) noexcept {
    if (fd_ < 0)
      return false;
    const int val = enable ? 1 : 0;
    return setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &val, sizeof(val)) == 0;
  }

  // Enables reception of CAN FD frames (canfd_frame) in addition to classic frames.
  [[nodiscard]] bool SetFdFrames(bool enable) noexcept {
    if (fd_ < 0)
      return false;
    const int val = enable ? 1 : 0;
    return setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &val, sizeof(val)) == 0;
  }

  // Set receive buffer size (SO_RCVBUF).
  void SetRecvBufferSize(int bytes) noexcept { SetSocketRecvBufferSize(fd_, bytes); }

  // Set receive timeout (SO_RCVTIMEO).
  void SetRecvTimeout(int timeout_ms) noexcept { SetSocketRecvTimeout(fd_, timeout_ms); }

  // -- I/O (allocation-free, noexcept) ---------------------------------------

  // Sends one classic CAN frame. Returns bytes written, or -1 on error.
  [[nodiscard]] ssize_t SendFrame(const can_frame& frame) noexcept {
    const ssize_t n = ::write(fd_, &frame, sizeof(frame));
    return (n == static_cast<ssize_t>(sizeof(frame))) ? n : -1;
  }

  // Receives one classic CAN frame. Returns bytes read (CAN_MTU), or -1 on
  // error/timeout.
  [[nodiscard]] ssize_t RecvFrame(can_frame& frame) noexcept {
    return ::read(fd_, &frame, sizeof(frame));
  }

  // Sends one CAN FD frame (requires SetFdFrames(true) and an FD-capable
  // interface). Returns bytes written, or -1 on error.
  [[nodiscard]] ssize_t SendFdFrame(const canfd_frame& frame) noexcept {
    const ssize_t n = ::write(fd_, &frame, sizeof(frame));
    return (n == static_cast<ssize_t>(sizeof(frame))) ? n : -1;
  }

  // Receives one frame into a canfd_frame buffer. On an FD-enabled socket the
  // result is either CAN_MTU (classic frame) or CANFD_MTU (FD frame); the
  // caller distinguishes by the returned size. Returns -1 on error/timeout.
  [[nodiscard]] ssize_t RecvFdFrame(canfd_frame& frame) noexcept {
    return ::read(fd_, &frame, sizeof(frame));
  }

  // -- Accessors -------------------------------------------------------------

  [[nodiscard]] int fd() const noexcept { return fd_; }

  [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

 private:
  int fd_{-1};
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_CAN_CAN_SOCKET_HPP_
