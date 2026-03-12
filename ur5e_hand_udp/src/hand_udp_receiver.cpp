#include "ur5e_hand_udp/hand_udp_receiver.hpp"
#include "ur5e_hand_udp/hand_udp_codec.hpp"

#include "ur5e_rt_base/thread_utils.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>

namespace ur5e_rt_controller {

HandUdpReceiver::HandUdpReceiver(int port, const ThreadConfig& thread_cfg) noexcept
    : port_(port), thread_cfg_(thread_cfg) {}

HandUdpReceiver::~HandUdpReceiver() {
  Stop();
}

bool HandUdpReceiver::Start() {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(static_cast<uint16_t>(port_));
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Enlarge socket receive buffer to absorb bursts without packet loss.
  constexpr int kRecvBufSize = 256 * 1024;  // 256 KB
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF,
             &kRecvBufSize, sizeof(kRecvBufSize));

  // 100 ms timeout so ReceiveLoop can check stop_token between recvs.
  struct timeval tv{};
  tv.tv_sec  = 0;
  tv.tv_usec = 100'000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  running_.store(true, std::memory_order_release);
  receive_thread_ = std::jthread([this](std::stop_token st) {
    ReceiveLoop(std::move(st));
  });

  return true;
}

void HandUdpReceiver::Stop() noexcept {
  running_.store(false, std::memory_order_release);
  receive_thread_.request_stop();
  // Close the socket to unblock recv() immediately.
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  // jthread destructor joins automatically.
}

std::array<double, kNumHandJoints> HandUdpReceiver::GetLatestData() const {
  std::lock_guard lock(data_mutex_);
  return latest_state_.motor_positions;
}

HandState HandUdpReceiver::GetLatestState() const {
  std::lock_guard lock(data_mutex_);
  return latest_state_;
}

double HandUdpReceiver::GetUpdateRate() const noexcept {
  using Clock = std::chrono::steady_clock;
  static auto        last_time  = Clock::now();
  static std::size_t last_count = 0;

  const auto         now   = Clock::now();
  const double       elapsed =
      std::chrono::duration<double>(now - last_time).count();
  const std::size_t  count = packet_count_.load(std::memory_order_relaxed);

  if (elapsed < 0.01) {
    return 0.0;
  }

  const double rate = static_cast<double>(count - last_count) / elapsed;
  last_time  = now;
  last_count = count;
  return rate;
}

void HandUdpReceiver::ReceiveLoop(std::stop_token stop_token) {
  ApplyThreadConfig(thread_cfg_);

  const int fd = socket_fd_;  // local copy to avoid data race with Stop()
  std::array<char, hand_udp_codec::kRecvBytes + 64> buffer{};

  while (!stop_token.stop_requested()) {
    const ssize_t n = recv(fd, buffer.data(), buffer.size(), 0);
    if (n < 0) {
      continue;  // timeout (EAGAIN) or socket closed (EBADF) — recheck stop
    }
    if (static_cast<std::size_t>(n) < hand_udp_codec::kRecvBytes) {
      continue;  // short / malformed packet
    }

    // Decode into a local HandState — no lock held during memcpy.
    HandState state;
    if (hand_udp_codec::DecodeRecvPacket(
            std::span<const char>(buffer.data(), static_cast<std::size_t>(n)),
            state)) {
      {
        // Single lock to update the shared snapshot.
        std::lock_guard lock(data_mutex_);
        latest_state_ = state;
      }
      if (callback_) {
        callback_(state);
      }
      packet_count_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

}  // namespace ur5e_rt_controller
