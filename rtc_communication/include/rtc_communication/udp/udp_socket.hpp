#ifndef RTC_COMMUNICATION_UDP_UDP_SOCKET_HPP_
#define RTC_COMMUNICATION_UDP_UDP_SOCKET_HPP_

// RAII wrapper for a UDP socket (AF_INET, SOCK_DGRAM).
//
// Designed for real-time use:
//   - No dynamic allocation after construction
//   - All I/O methods are noexcept
//   - Deterministic cleanup in destructor
//
// Two usage modes:
//   1. Bind mode (receiver): Bind() -> Recv()
//   2. Connect mode (sender): Connect() -> Send()

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>

namespace rtc {

class UdpSocket {
 public:
  UdpSocket() noexcept = default;
  ~UdpSocket() { Close(); }

  // Non-copyable, non-movable (owns raw fd).
  UdpSocket(const UdpSocket&)            = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  UdpSocket(UdpSocket&&)                 = delete;
  UdpSocket& operator=(UdpSocket&&)      = delete;

  // -- Socket lifecycle ------------------------------------------------------

  // Creates the underlying SOCK_DGRAM socket. Returns false on failure.
  [[nodiscard]] bool Open() noexcept {
    if (fd_ >= 0) return true;  // already open
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    return fd_ >= 0;
  }

  // Binds to INADDR_ANY:port (receiver mode).
  [[nodiscard]] bool Bind(int port) noexcept {
    if (fd_ < 0 && !Open()) return false;

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      Close();
      return false;
    }
    return true;
  }

  // Resolves target_ip:port for subsequent Send() calls (sender mode).
  [[nodiscard]] bool Connect(std::string_view target_ip, int port) noexcept {
    if (fd_ < 0 && !Open()) return false;

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port   = htons(static_cast<uint16_t>(port));

    // inet_pton needs a null-terminated string.
    char ip_buf[INET_ADDRSTRLEN]{};
    const auto len = std::min(target_ip.size(), sizeof(ip_buf) - 1);
    std::memcpy(ip_buf, target_ip.data(), len);

    if (inet_pton(AF_INET, ip_buf, &target_addr_.sin_addr) <= 0) {
      Close();
      return false;
    }
    has_target_ = true;
    return true;
  }

  void Close() noexcept {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    has_target_ = false;
  }

  // -- Socket options --------------------------------------------------------

  // Set receive buffer size (SO_RCVBUF).
  void SetRecvBufferSize(int bytes) noexcept {
    if (fd_ >= 0) {
      setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &bytes, sizeof(bytes));
    }
  }

  // Set receive timeout (SO_RCVTIMEO).
  void SetRecvTimeout(int timeout_ms) noexcept {
    if (fd_ < 0) return;
    struct timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  // -- I/O (allocation-free, noexcept) ---------------------------------------

  // Receives up to buf.size() bytes. Returns bytes received, or -1 on error.
  [[nodiscard]] ssize_t Recv(std::span<char> buf) noexcept {
    return ::recv(fd_, buf.data(), buf.size(), 0);
  }

  // Sends data to the previously Connect()'ed target.
  [[nodiscard]] ssize_t Send(std::span<const uint8_t> data) noexcept {
    if (!has_target_) return -1;
    return ::sendto(fd_, data.data(), data.size(), 0,
                    reinterpret_cast<const sockaddr*>(&target_addr_),
                    sizeof(target_addr_));
  }

  // -- Accessors -------------------------------------------------------------

  [[nodiscard]] int fd() const noexcept { return fd_; }
  [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

 private:
  int         fd_{-1};
  sockaddr_in target_addr_{};
  bool        has_target_{false};
};

}  // namespace rtc

#endif  // RTC_COMMUNICATION_UDP_UDP_SOCKET_HPP_
