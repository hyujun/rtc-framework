#ifndef INTEGRATED_BRINGUP_BACKENDS_JOINT_STATE_REORDER_H_
#define INTEGRATED_BRINGUP_BACKENDS_JOINT_STATE_REORDER_H_

#include "rtc_base/types/types.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace rtc {

// ── JointStateReorder ────────────────────────────────────────────────────────
//
// Fixed-capacity msg-index → device-slot map for the named JointState lane of
// the integrated_bringup device backends. The map is built once from the
// first named message *inside the rt_callback lane* (FIFO executor), so both
// the one-shot build and the per-message copy must stay allocation-free,
// bounded loops (invariants.md §RT callback rule; issue #156 — this replaces
// the former lazy std::vector<int> build).
//
// size == 0 means "no reorder": WriteJointStateToCache falls back to an
// identity copy (un-named stream, or joint_command_names unset).
struct JointStateReorder {
  std::array<int, kMaxDeviceChannels> map{};  // msg idx -> device slot (-1 = unknown name)
  int size{0};
};

// One-shot build from the first named message. Bounded: scans at most
// kMaxDeviceChannels message names — a message wider than DeviceStateCache
// can never land in the fixed arrays anyway, so excess joints are dropped.
// std::string equality is compare-only; no heap traffic.
inline void BuildJointStateReorder(const std::vector<std::string>& msg_names,
                                   const std::vector<std::string>& ref_names,
                                   JointStateReorder& out) noexcept {
  out.size = 0;
  if (msg_names.empty() || ref_names.empty())
    return;
  const std::size_t n = std::min(msg_names.size(), static_cast<std::size_t>(kMaxDeviceChannels));
  for (std::size_t msg_i = 0; msg_i < n; ++msg_i) {
    out.map[msg_i] = -1;
    for (std::size_t ref_i = 0; ref_i < ref_names.size(); ++ref_i) {
      if (msg_names[msg_i] == ref_names[ref_i]) {
        out.map[msg_i] = static_cast<int>(ref_i);
        break;
      }
    }
  }
  out.size = static_cast<int>(n);
}

// Mailbox write: copy position/velocity/effort into the cache, reordered when
// a map exists, identity otherwise. num_channels is clamped to
// kMaxDeviceChannels — the RT loop copy_n's num_channels elements out of the
// fixed cache arrays (rt_controller_node_rt_loop.cpp), so an oversized wire
// value must never escape the cache.
inline void WriteJointStateToCache(const sensor_msgs::msg::JointState& msg,
                                   const JointStateReorder& reorder,
                                   DeviceStateCache& ds) noexcept {
  ds.num_channels =
      static_cast<int>(std::min(msg.position.size(), static_cast<std::size_t>(kMaxDeviceChannels)));

  if (reorder.size > 0) {
    const std::size_t n = std::min(msg.position.size(), static_cast<std::size_t>(reorder.size));
    for (std::size_t src = 0; src < n; ++src) {
      const int idx = reorder.map[src];
      if (idx < 0 || idx >= kMaxDeviceChannels)
        continue;
      const auto uidx = static_cast<std::size_t>(idx);
      ds.positions[uidx] = msg.position[src];
      if (src < msg.velocity.size())
        ds.velocities[uidx] = msg.velocity[src];
      if (src < msg.effort.size())
        ds.efforts[uidx] = msg.effort[src];
    }
  } else {
    const std::size_t n =
        std::min(msg.position.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < n; ++i)
      ds.positions[i] = msg.position[i];
    const std::size_t nv =
        std::min(msg.velocity.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < nv; ++i)
      ds.velocities[i] = msg.velocity[i];
    const std::size_t ne =
        std::min(msg.effort.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < ne; ++i)
      ds.efforts[i] = msg.effort[i];
  }
  ds.valid = true;
}

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_JOINT_STATE_REORDER_H_
