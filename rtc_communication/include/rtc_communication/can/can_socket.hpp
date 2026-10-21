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
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
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

  // Set socket priority (SO_PRIORITY). Returns false when closed or on failure.
  [[nodiscard]] bool SetPriority(int priority) noexcept { return SetSocketPriority(fd_, priority); }

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

  // Non-blocking variants of SendFrame / SendFdFrame (MSG_DONTWAIT): when the
  // socket send buffer is exhausted — frames pending on a stalled controller —
  // they fail at once with errno = EAGAIN instead of sleeping in the kernel
  // until the default (infinite) SO_SNDTIMEO. Returns bytes written, or -1.
  [[nodiscard]] ssize_t SendFrameNonBlocking(const can_frame& frame) noexcept {
    const ssize_t n = ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
    return (n == static_cast<ssize_t>(sizeof(frame))) ? n : -1;
  }

  [[nodiscard]] ssize_t SendFdFrameNonBlocking(const canfd_frame& frame) noexcept {
    const ssize_t n = ::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
    return (n == static_cast<ssize_t>(sizeof(frame))) ? n : -1;
  }

  // Receives one frame into a canfd_frame buffer. On an FD-enabled socket the
  // result is either CAN_MTU (classic frame) or CANFD_MTU (FD frame); the
  // caller distinguishes by the returned size. Returns -1 on error/timeout.
  [[nodiscard]] ssize_t RecvFdFrame(canfd_frame& frame) noexcept {
    return ::read(fd_, &frame, sizeof(frame));
  }

  // Waits up to `timeout` (relative) for a frame, then reads it without
  // blocking. ppoll gives sub-millisecond deadlines, which SO_RCVTIMEO (ms)
  // cannot, and the read never sleeps past it: a wake with nothing queued
  // (POLLERR whose error was already consumed, another reader draining the
  // socket) returns 0 instead of blocking for SO_RCVTIMEO. Returns CAN_MTU /
  // CANFD_MTU on success, 0 when nothing arrived (timeout expired or a
  // spurious wake), -1 on error (errno set: EBADF when closed, EINTR when a
  // signal interrupted the wait, else the pending socket error — e.g.
  // ENETDOWN — that ppoll reported as POLLERR).
  [[nodiscard]] ssize_t RecvFdFrameWithTimeout(canfd_frame& frame,
                                               const timespec& timeout) noexcept {
    if (fd_ < 0) {
      errno = EBADF;  // ppoll ignores a negative fd and would report a timeout
      return -1;
    }
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int rc = ::ppoll(&pfd, 1, &timeout, nullptr);
    if (rc < 0)
      return -1;
    if (rc == 0)
      return 0;
    // POLLIN: one frame is queued. POLLERR / POLLHUP: recv() surfaces the
    // pending socket error as errno (sk_err is reported before EAGAIN).
    return RecvFdFrameNonBlocking(frame);
  }

  // Reads one queued frame without blocking (MSG_DONTWAIT), regardless of
  // SO_RCVTIMEO. Returns CAN_MTU / CANFD_MTU, 0 when nothing is queued, -1 on
  // error (errno set, EBADF when closed).
  [[nodiscard]] ssize_t RecvFdFrameNonBlocking(canfd_frame& frame) noexcept {
    if (fd_ < 0) {
      errno = EBADF;
      return -1;
    }
    const ssize_t n = ::recv(fd_, &frame, sizeof(frame), MSG_DONTWAIT);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      return 0;
    return n;
  }

  // -- Accessors -------------------------------------------------------------

  [[nodiscard]] int fd() const noexcept { return fd_; }

  [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

 private:
  int fd_{-1};
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_CAN_CAN_SOCKET_HPP_
