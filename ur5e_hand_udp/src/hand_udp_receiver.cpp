#include "ur5e_hand_udp/hand_udp_receiver.hpp"

#include "ur5e_rt_base/thread_utils.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>

namespace ur5e_rt_controller {

namespace {
// Full UDP packet: 11 positions + 11 velocities + 11 currents + 44 sensors = 77 doubles
constexpr std::size_t kPacketDoubles = 77;
constexpr std::size_t kPacketBytes   = kPacketDoubles * sizeof(double);
}  // namespace

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
  return latest_data_;
}

double HandUdpReceiver::GetUpdateRate() const noexcept {
  using Clock = std::chrono::steady_clock;
  static auto   last_time  = Clock::now();
  static std::size_t last_count = 0;

  const auto   now   = Clock::now();
  const double elapsed =
      std::chrono::duration<double>(now - last_time).count();
  const std::size_t count = packet_count_.load(std::memory_order_relaxed);

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
  std::array<char, kPacketBytes + 64> buffer{};

  while (!stop_token.stop_requested()) {
    const ssize_t n = recv(fd, buffer.data(), buffer.size(), 0);
    if (n < 0) {
      continue;  // timeout (EAGAIN) or socket closed (EBADF) — recheck stop
    }
    if (static_cast<std::size_t>(n) < kPacketBytes) {
      continue;  // short / malformed packet
    }

    if (ParsePacket(std::span<const char>(buffer.data(),
                                          static_cast<std::size_t>(n)))) {
      if (callback_) {
        std::array<double, kNumHandJoints> snapshot;
        {
          std::lock_guard lock(data_mutex_);
          snapshot = latest_data_;
        }
        callback_(std::span<const double, kNumHandJoints>(snapshot));
      }
      packet_count_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

bool HandUdpReceiver::ParsePacket(std::span<const char> buffer) noexcept {
  if (buffer.size() < kPacketBytes) {
    return false;
  }

  // Layout: [0..10] motor_positions, [11..21] velocities,
  //         [22..32] currents,       [33..76] sensor_data
  std::array<double, kPacketDoubles> values{};
  std::memcpy(values.data(), buffer.data(), kPacketBytes);

  {
    std::lock_guard lock(data_mutex_);
    std::copy_n(values.begin(), kNumHandJoints, latest_data_.begin());
  }
  return true;
}

}  // namespace ur5e_rt_controller
