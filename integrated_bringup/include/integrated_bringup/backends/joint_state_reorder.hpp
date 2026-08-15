#ifndef INTEGRATED_BRINGUP_BACKENDS_JOINT_STATE_REORDER_H_
#define INTEGRATED_BRINGUP_BACKENDS_JOINT_STATE_REORDER_H_

#include "rtc_base/types/types.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
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
//
// ALSO PRODUCES ds.hole_mask, ds.velocity_hole_mask and ds.effort_hole_mask
// (issues #284, #446) — the per-slot companions to
// num_channels. The cache is PERSISTENT, so a slot this message does not reach
// keeps its previous value, and with an active reorder map that is a normal
// outcome rather than an error: BuildJointStateReorder maps only the names the
// reference list has, so a message carrying names the device does not declare
// leaves the unmatched device slots untouched while still counting toward the
// wire length. `num_channels >= model_dim` therefore passes on a device whose
// slot 0 was never written — the counterexample pinned in
// test_joint_state_reorder.cpp's WithReorder_PlacesByDeviceSlot.
//
// ASSIGNED, NEVER OR-ACCUMULATED. Freshness is a statement about THIS message,
// so the mask is rebuilt from scratch every call. Accumulating it across
// messages would let one lucky early message mark a slot fresh forever and
// reopen the gap this field exists to close.
inline void WriteJointStateToCache(const sensor_msgs::msg::JointState& msg,
                                   const JointStateReorder& reorder,
                                   DeviceStateCache& ds) noexcept {
  ds.num_channels =
      static_cast<int>(std::min(msg.position.size(), static_cast<std::size_t>(kMaxDeviceChannels)));

  if (reorder.size > 0) {
    const std::size_t n = std::min(msg.position.size(), static_cast<std::size_t>(reorder.size));
    uint64_t written = 0;
    uint64_t written_v = 0;
    uint64_t written_e = 0;
    for (std::size_t src = 0; src < n; ++src) {
      const int idx = reorder.map[src];
      if (idx < 0 || idx >= kMaxDeviceChannels)
        continue;
      const auto uidx = static_cast<std::size_t>(idx);
      const uint64_t bit = static_cast<uint64_t>(1) << uidx;
      ds.positions[uidx] = msg.position[src];
      written |= bit;
      // ONE ACCUMULATOR PER LANE, set inside the guard that writes the value
      // (#446). The three lanes diverge exactly here: `src` indexes the WIRE,
      // and a message whose `velocity` is shorter than its `position` — or
      // absent, which JointState allows and shipped drivers use — reaches this
      // slot with a fresh q and leaves velocities[uidx] holding the persistent
      // cache's previous value. Marking the slot written on the position lane
      // alone is what made that state indistinguishable from a fully reported
      // one for every reader of the mask.
      if (src < msg.velocity.size()) {
        ds.velocities[uidx] = msg.velocity[src];
        written_v |= bit;
      }
      if (src < msg.effort.size()) {
        ds.efforts[uidx] = msg.effort[src];
        written_e |= bit;
      }
    }
    ds.hole_mask = ~written;
    ds.velocity_hole_mask = ~written_v;
    ds.effort_hole_mask = ~written_e;
  } else {
    const std::size_t n =
        std::min(msg.position.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < n; ++i)
      ds.positions[i] = msg.position[i];
    // Identity write: the fresh slots are exactly the prefix [0, n), which is
    // also what num_channels reports — so on this path the mask never closes a
    // gate the width test would have left open. It is still computed rather
    // than left alone, because the cache is persistent and a message that
    // NARROWS must retire the slots the previous wider one filled.
    ds.hole_mask = ~SlotMaskBelow(static_cast<int>(n));
    // Each lane's fresh prefix is its OWN length (#446). On this path the three
    // are plainly independent — `nv` and `ne` are read from the message, not
    // derived from `n` — and `num_channels` reports only the first, so the
    // width test says nothing about the other two. A driver that omits a lane
    // lands here with nv == 0 and gets an all-ones mask, which is the honest
    // report: no slot of that lane was written, ever.
    const std::size_t nv =
        std::min(msg.velocity.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < nv; ++i)
      ds.velocities[i] = msg.velocity[i];
    ds.velocity_hole_mask = ~SlotMaskBelow(static_cast<int>(nv));
    const std::size_t ne =
        std::min(msg.effort.size(), static_cast<std::size_t>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < ne; ++i)
      ds.efforts[i] = msg.effort[i];
    ds.effort_hole_mask = ~SlotMaskBelow(static_cast<int>(ne));
  }
  ds.valid = true;
}

}  // namespace rtc

#endif  // INTEGRATED_BRINGUP_BACKENDS_JOINT_STATE_REORDER_H_
