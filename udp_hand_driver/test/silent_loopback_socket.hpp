#pragma once

// Test-only: a loopback UDP socket that stands in for a hand that never
// answers. Bound to an ephemeral port on 127.0.0.1 (parallel test binaries
// cannot collide), non-blocking so Drain() returns whatever has arrived.
// Shared by the controller tests (silent device: every cycle times out) and the
// lifecycle E-Stop test (the one datagram that does arrive is the zero-write).
// The transport tests keep their own LoopbackDevice — that one answers.

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <vector>

namespace udp_hand_driver::test_support {

class SilentLoopbackSocket {
 public:
  SilentLoopbackSocket() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      return;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(0);
    if (::bind(fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
      Close();
      return;
    }
    socklen_t len = sizeof(addr);
    if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
      Close();
      return;
    }
    port_ = ntohs(addr.sin_port);
    ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL) | O_NONBLOCK);
  }

  ~SilentLoopbackSocket() { Close(); }

  SilentLoopbackSocket(const SilentLoopbackSocket&) = delete;
  SilentLoopbackSocket& operator=(const SilentLoopbackSocket&) = delete;

  [[nodiscard]] bool ok() const { return fd_ >= 0; }

  [[nodiscard]] int port() const { return port_; }

  // Every datagram currently queued, in arrival order, appended to `out`.
  void Drain(std::vector<std::vector<std::uint8_t>>& out) {
    std::array<std::uint8_t, 512> buf{};
    for (;;) {
      const ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
      if (n <= 0) {
        return;
      }
      out.emplace_back(buf.begin(), buf.begin() + n);
    }
  }

 private:
  void Close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_{-1};
  int port_{0};
};

}  // namespace udp_hand_driver::test_support
