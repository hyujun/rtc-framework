#ifndef RTC_COMMUNICATION_SOCKET_OPTIONS_HPP_
#define RTC_COMMUNICATION_SOCKET_OPTIONS_HPP_

// Shared POSIX socket-option helpers used by the UDP and CAN RAII wrappers.
// Generic SOL_SOCKET plumbing only — no protocol-specific logic belongs here.

#include <sys/socket.h>
#include <sys/time.h>

namespace rtc {

// Set receive buffer size (SO_RCVBUF). No-op when fd is not open.
inline void SetSocketRecvBufferSize(int fd, int bytes) noexcept {
  if (fd >= 0) {
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bytes, sizeof(bytes));
  }
}

// Set receive timeout (SO_RCVTIMEO). No-op when fd is not open.
inline void SetSocketRecvTimeout(int fd, int timeout_ms) noexcept {
  if (fd < 0)
    return;

  struct timeval tv {};

  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

// Set socket priority (SO_PRIORITY): the kernel maps it to the tx qdisc band
// and the skb priority. Values above 6 need CAP_NET_ADMIN. Returns false when
// fd is not open or setsockopt fails (errno preserved).
[[nodiscard]] inline bool SetSocketPriority(int fd, int priority) noexcept {
  if (fd < 0)
    return false;
  return setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) == 0;
}

}  // namespace rtc

#endif  // RTC_COMMUNICATION_SOCKET_OPTIONS_HPP_
