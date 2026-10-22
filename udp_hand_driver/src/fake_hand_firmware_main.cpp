// fake_hand_firmware_main.cpp -- full-SIL loopback firmware simulator (device side).
//
// protocol_version 1a / 1b: binds a UDP socket and answers the real
// udp_hand_node's request datagrams with protocol-correct responses (see
// fake_hand_firmware.hpp). Run alongside an unmodified udp_hand_node
// (use_fake_hand=false, target_ip=127.0.0.1) to exercise the whole transport
// path with no hardware. Launch both via `udp_hand.launch.py sil_mode:=firmware`.
//
// protocol_version 2: opens a CAN FD socket on `can_interface` (vcan0 by
// default) as the hand node `hand_side` names (D51) and answers CMD / REQ frames
// with the echo / RESP burst BuildCanFdResponse() builds — the device side of
// CanFdHandComm. No receive filter: the mirror itself sorts other traffic into
// its counters, as the real MCU would see the whole bus.
//
// This is a thin socket/param/log shell; all response logic lives in the
// header-only FakeHandFirmware class (unit-tested independently). The node reads
// parameters once at startup then runs its own recv loop, polling rclcpp::ok()
// via a 100 ms receive timeout — no executor spin is required.

#include "rtc_communication/can/canfd_transport.hpp"
#include "udp_hand_driver/fake_hand_firmware.hpp"
#include "udp_hand_driver/hand_canfd_packets.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <linux/can.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>

namespace {

// Read the FakeHandFirmware::Config from ROS parameters (declared with defaults).
// `hand_side` is declared for every protocol_version (it only matters on CAN FD);
// an unparsable value is a configuration error — `hand_side_ok` reports it.
// `proto` receives protocol_version (it selects the UDP / CAN FD shell in main).
udp_hand_driver::FakeHandFirmware::Config LoadConfig(rclcpp::Node& node, std::string& proto,
                                                     bool& hand_side_ok) {
  udp_hand_driver::FakeHandFirmware::Config cfg;
  proto = node.declare_parameter<std::string>("protocol_version", "1a");
  cfg.proto_1b = (proto == "1b");
  const std::string side = node.declare_parameter<std::string>("hand_side", "right");
  hand_side_ok = udp_hand_driver::canfd::ParseHandSide(side, cfg.hand_side);
  cfg.num_fingertips = static_cast<int>(
      node.declare_parameter<int64_t>("num_fingertips", udp_hand_driver::kDefaultNumFingertips));
  cfg.lpf_time_constant_s = node.declare_parameter<double>("lpf_time_constant_s", 0.1);
  cfg.effort_stiffness = node.declare_parameter<double>("effort_stiffness", 1.0);
  cfg.effort_damping = node.declare_parameter<double>("effort_damping", 0.1);
  cfg.step_rate_hz = node.declare_parameter<double>("step_rate_hz", 500.0);
  cfg.sensor_base_min = static_cast<int>(node.declare_parameter<int64_t>("sensor_base_min", 100));
  cfg.sensor_base_max = static_cast<int>(node.declare_parameter<int64_t>("sensor_base_max", 4000));
  cfg.sensor_noise_stddev = node.declare_parameter<double>("sensor_noise_stddev", 5.0);
  cfg.seed = static_cast<uint32_t>(node.declare_parameter<int64_t>("seed", 42));
  return cfg;
}

// proto2 shell: one CAN FD socket, no filter, 100 ms receive timeout polling
// rclcpp::ok(). Every received frame goes through BuildCanFdResponse(); the
// frames it yields are sent as 29-bit FD frames with BRS, in order.
int RunCanFd(rclcpp::Node& node, udp_hand_driver::FakeHandFirmware& firmware,
             const udp_hand_driver::FakeHandFirmware::Config& cfg,
             const std::string& can_interface) {
  rtc::CanFdTransportConfig tcfg{};
  tcfg.interface_name = can_interface;
  tcfg.extended_frame = true;
  tcfg.bitrate_switch = true;
  rtc::CanFdTransport transport(tcfg);
  if (!transport.Open()) {
    RCLCPP_ERROR(node.get_logger(), "CAN FD open(%s) failed: %s", can_interface.c_str(),
                 std::strerror(errno));
    return 1;
  }
  RCLCPP_INFO(node.get_logger(),
              "fake_hand_firmware on CAN FD %s (protocol_version=2, node=%u, num_fingertips=%d, "
              "step_rate=%.0f Hz)",
              can_interface.c_str(),
              static_cast<unsigned>(udp_hand_driver::canfd::NodeIdOf(cfg.hand_side)),
              cfg.num_fingertips, cfg.step_rate_hz);

  std::array<udp_hand_driver::FakeHandFirmware::CanFdFrame, udp_hand_driver::canfd::kNumRespFrames>
      out{};
  canfd_frame rx{};
  canfd_frame tx{};
  const timespec timeout{0, 100 * 1000 * 1000};  // 100 ms
  while (rclcpp::ok()) {
    const ssize_t n = transport.RecvFrameWithTimeout(rx, timeout);
    if (n < 0) {
      if (errno == EINTR)
        continue;  // signal (e.g. SIGINT) — re-check rclcpp::ok()
      RCLCPP_ERROR(node.get_logger(), "CAN FD recv failed: %s", std::strerror(errno));
      return 1;  // a dead interface is a failed peer, not a clean shutdown
    }
    if (n == 0)
      continue;
    const std::size_t count =
        firmware.BuildCanFdResponse(rx.can_id, rx.data, rx.len, out.data(), out.size());
    for (std::size_t i = 0; i < count; ++i) {
      tx = canfd_frame{};
      tx.can_id = out[i].can_id | CAN_EFF_FLAG;
      tx.len = static_cast<__u8>(out[i].len);
      tx.flags = CANFD_BRS;
      std::memcpy(tx.data, out[i].data.data(), out[i].len);
      if (!transport.SendFrame(tx))
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "CAN FD send failed: %s",
                             std::strerror(errno));
    }
  }
  RCLCPP_INFO(node.get_logger(), "fake_hand_firmware shutting down");
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fake_hand_firmware");

  const int port = static_cast<int>(node->declare_parameter<int64_t>("port", 55151));
  const std::string can_interface = node->declare_parameter<std::string>("can_interface", "vcan0");
  std::string proto;
  bool hand_side_ok = false;
  const udp_hand_driver::FakeHandFirmware::Config cfg = LoadConfig(*node, proto, hand_side_ok);
  if (!hand_side_ok) {
    RCLCPP_ERROR(node->get_logger(), "hand_side must be \"right\" or \"left\"");
    rclcpp::shutdown();
    return 1;
  }
  udp_hand_driver::FakeHandFirmware firmware(cfg);

  if (proto == "2") {
    const int rc = RunCanFd(*node, firmware, cfg, can_interface);
    rclcpp::shutdown();
    return rc;
  }

  // ── Open + bind the UDP socket ─────────────────────────────────────────────
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    RCLCPP_ERROR(node->get_logger(), "socket() failed: %s", std::strerror(errno));
    rclcpp::shutdown();
    return 1;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr =
      htonl(INADDR_ANY);  // reply source resolves to 127.0.0.1 for loopback peers
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    RCLCPP_ERROR(node->get_logger(), "bind(port=%d) failed: %s", port, std::strerror(errno));
    ::close(fd);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "fake_hand_firmware listening on UDP port %d (protocol_version=%s, "
              "num_fingertips=%d, step_rate=%.0f Hz)",
              port, cfg.proto_1b ? "1b" : "1a", cfg.num_fingertips, cfg.step_rate_hz);

  // ── Recv loop: ppoll timeout polls rclcpp::ok() so SIGINT stops cleanly ─────
  std::array<uint8_t, udp_hand_driver::packets::kMaxPacketSize> req{};
  std::array<uint8_t, udp_hand_driver::packets::kMaxPacketSize> resp{};
  pollfd pfd{fd, POLLIN, 0};
  const timespec timeout{0, 100 * 1000 * 1000};  // 100 ms

  while (rclcpp::ok()) {
    const int ready = ::ppoll(&pfd, 1, &timeout, nullptr);
    if (ready < 0) {
      if (errno == EINTR)
        continue;  // signal (e.g. SIGINT) — re-check rclcpp::ok()
      RCLCPP_ERROR(node->get_logger(), "ppoll() failed: %s", std::strerror(errno));
      break;
    }
    if (ready == 0 || (pfd.revents & POLLIN) == 0)
      continue;

    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    const ssize_t n =
        ::recvfrom(fd, req.data(), req.size(), 0, reinterpret_cast<sockaddr*>(&peer), &peer_len);
    if (n <= 0)
      continue;

    const std::size_t out_len =
        firmware.BuildResponse(req.data(), static_cast<std::size_t>(n), resp.data(), resp.size());
    if (out_len == 0)
      continue;  // unknown / malformed request — silently drop (real firmware would too)

    ::sendto(fd, resp.data(), out_len, 0, reinterpret_cast<const sockaddr*>(&peer), peer_len);
  }

  RCLCPP_INFO(node->get_logger(), "fake_hand_firmware shutting down");
  ::close(fd);
  rclcpp::shutdown();
  return 0;
}
