#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace {
// Cycles between the periodic "cycles: N" DEBUG line. `int` so it can be passed
// to AdvanceDecimation, which shares the accumulate-and-carry arithmetic with
// the link decimation below.
constexpr int kCycleLogInterval = 500;
}  // namespace

void UdpHandNode::PreallocateMessages(HandUnit& unit) {
  unit.joint_js_msg.name = unit.joint_names;
  unit.joint_js_msg.position.resize(udp_hand_driver::kNumHandMotors);
  unit.joint_js_msg.velocity.resize(udp_hand_driver::kNumHandMotors);
  unit.joint_js_msg.effort.resize(udp_hand_driver::kNumHandMotors);

  unit.motor_js_msg.name = unit.motor_names;
  unit.motor_js_msg.position.resize(udp_hand_driver::kNumHandMotors);
  unit.motor_js_msg.velocity.resize(udp_hand_driver::kNumHandMotors);
  unit.motor_js_msg.effort.resize(udp_hand_driver::kNumHandMotors);

  unit.sensor_msg.fingertips.resize(static_cast<std::size_t>(num_fingertips_));
  for (int f = 0; f < num_fingertips_; ++f) {
    auto& fs = unit.sensor_msg.fingertips[static_cast<std::size_t>(f)];
    fs.name = (static_cast<std::size_t>(f) < unit.fingertip_names.size())
                  ? unit.fingertip_names[static_cast<std::size_t>(f)]
                  : "f" + std::to_string(f);
  }
}

void UdpHandNode::PollAndPublish() {
  for (auto& unit : units_) {
    HandUnit& u = *unit;
    if (!u.controller)
      continue;

    // Mask the low bit down to the last *completed* write: an odd sequence means
    // the CommLoop is mid-Store, and treating that as progress would both
    // mis-count cycles and desynchronise the even-valued baseline. Load() itself
    // retries past a torn read, so the snapshot below is at least as new as `seq`.
    const std::uint32_t seq = udp_hand_driver::LastCompletedWrite(u.controller->state_sequence());
    const std::uint32_t cycles = udp_hand_driver::CyclesBetween(u.last_published_seq, seq);
    if (cycles == 0)
      continue;  // no comm cycle of this hand completed since the last poll
    u.last_published_seq = seq;

    PublishState(u, u.controller->GetLatestState(), u.controller->GetLatestFTState(), cycles);
  }
}

void UdpHandNode::PublishState(HandUnit& u, const udp_hand_driver::UdpHandState& state,
                               const udp_hand_driver::FingertipFTState& ft_state,
                               std::uint32_t cycles) {
  // Stamped on the consumer thread. This is the publish time, not the sample
  // time — the CommLoop does not call now() (RT-1), and UdpHandState carries no
  // timestamp field, so adding one would change the POD and the SeqLock payload.
  const auto stamp = this->now();

  if (state.joint_valid) {
    u.joint_js_msg.header.stamp = stamp;
    for (int i = 0; i < udp_hand_driver::kNumHandMotors; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      // Add the per-joint offset: firmware frame → controller frame.
      // Position-only; velocity/effort are unaffected by a constant offset.
      u.joint_js_msg.position[iu] =
          static_cast<double>(state.joint_positions[iu] + u.joint_offset_rad[iu]);
      u.joint_js_msg.velocity[iu] = static_cast<double>(state.joint_velocities[iu]);
      u.joint_js_msg.effort[iu] = static_cast<double>(state.joint_currents[iu]);
    }
    u.joint_state_pub->publish(u.joint_js_msg);
  }

  if (state.motor_valid) {
    u.motor_js_msg.header.stamp = stamp;
    for (int i = 0; i < udp_hand_driver::kNumHandMotors; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      u.motor_js_msg.position[iu] = static_cast<double>(state.motor_positions[iu]);
      u.motor_js_msg.velocity[iu] = static_cast<double>(state.motor_velocities[iu]);
      u.motor_js_msg.effort[iu] = static_cast<double>(state.motor_currents[iu]);
    }
    u.motor_state_pub->publish(u.motor_js_msg);
  }

  if (state.num_fingertips > 0) {
    u.sensor_msg.header.stamp = stamp;

    const bool ft_valid = ft_enabled_ && ft_state.valid;

    for (int f = 0;
         f < state.num_fingertips && f < static_cast<int>(u.sensor_msg.fingertips.size()); ++f) {
      auto& fs = u.sensor_msg.fingertips[static_cast<std::size_t>(f)];

      if (sensor_uses_force_layout_) {
        // ── 1b: per-fingertip force vector [fx,fy,fz] measured by firmware ──
        // barometer/ToF stay zero (unused on 1b); Lx/Ly (u) and Temp are
        // decoded into state.sensor_force but held back this increment (current
        // firmware ships placeholders), so only the real force datum is
        // published. See proto-1b migration plan.
        const std::size_t force_base =
            static_cast<std::size_t>(f) * udp_hand_driver::kP1bValuesPerFingertip;
        for (int j = 0; j < 3; ++j) {
          const auto ju = static_cast<std::size_t>(j);
          fs.f[ju] = state.sensor_force[force_base + ju];
          fs.u[ju] = 0.0f;
        }
        // inference_enable is the consumers' "this fingertip's f[] is usable"
        // flag (CM's sensor lane and every controller's ft.valid gate on it),
        // NOT a report of whether the ML inferencer runs. On 1b the force is a
        // firmware measurement that needs no inferencer, so it is always
        // usable — tying this to ft_enabled_ made CM drop the whole lane,
        // because 1b intentionally leaves ft_inferencer.enabled=false (no
        // barometer input to infer from).
        fs.inference_enable = true;
        // D29: proto2's firmware contact flag; 1b never writes sensor_contact,
        // so it stays 0.0 there (no contact classifier on that path).
        fs.contact_flag = (state.sensor_contact[static_cast<std::size_t>(f)] != 0) ? 1.0f : 0.0f;
        continue;
      }

      const int sensor_base = f * udp_hand_driver::kSensorValuesPerFingertip;
      for (int b = 0; b < udp_hand_driver::kBarometerCount; ++b) {
        const auto bu = static_cast<std::size_t>(b);
        const auto si = static_cast<std::size_t>(sensor_base + b);
        fs.barometer[bu] = static_cast<float>(state.sensor_data[si]);
        fs.barometer_raw[bu] = static_cast<float>(state.sensor_data_raw[si]);
      }
      for (int t = 0; t < udp_hand_driver::kTofCount; ++t) {
        const auto tu = static_cast<std::size_t>(t);
        const auto si =
            static_cast<std::size_t>(sensor_base + udp_hand_driver::kBarometerCount + t);
        fs.tof[tu] = static_cast<float>(state.sensor_data[si]);
        fs.tof_raw[tu] = static_cast<float>(state.sensor_data_raw[si]);
      }

      if (ft_valid && f < ft_state.num_fingertips &&
          ft_state.per_fingertip_valid[static_cast<std::size_t>(f)]) {
        const int ft_base = f * udp_hand_driver::kFTValuesPerFingertip;
        fs.inference_enable = true;
        fs.contact_flag = ft_state.ft_data[static_cast<std::size_t>(ft_base)];
        for (int j = 0; j < 3; ++j) {
          const auto ju = static_cast<std::size_t>(j);
          fs.f[ju] = ft_state.ft_data[static_cast<std::size_t>(ft_base + 1 + j)];
          fs.u[ju] = ft_state.ft_data[static_cast<std::size_t>(ft_base + 4 + j)];
        }
      } else {
        fs.inference_enable = false;
        fs.contact_flag = 0.0f;
        fs.f = {};
        fs.u = {};
      }
    }
    u.sensor_state_pub->publish(u.sensor_msg);
    u.sensor_monitor_pub->publish(u.sensor_msg);
  }

  // Link status (decimated — not every cycle).
  // Published via standalone rclcpp::Publisher (not LifecyclePublisher),
  // so it works regardless of lifecycle state.
  // Accumulate comm cycles, not invocations (issue #345). Under the old push
  // model one callback was exactly one comm cycle, so ++ was the cycle count;
  // under polling an invocation can carry several. `%=` rather than `= 0` keeps
  // the remainder, so the intended link_status rate holds even when polls slip.
  // The publish itself stays capped at once per invocation — a poll that jumped
  // three cycles reports link state once, not three times.
  if (udp_hand_driver::AdvanceDecimation(u.link_cycle_counter, cycles, link_decimation_)) {
    // Strict per-channel link health (#1): mirrors the detector's E-STOP decision
    // exactly (same UdpHandController::LinkDown, same thresholds) so the
    // link_status Bool never disagrees with hand_link_down.
    const bool link_ok =
        !u.controller->LinkDown(static_cast<uint32_t>(link_fail_threshold_),
                                static_cast<uint32_t>(link_fail_threshold_sensor_));
    if (link_ok != u.prev_link_ok) {
      if (link_ok) {
        RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "%sHand link UP", u.log_tag.c_str());
      } else {
        RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                    "%sHand link DOWN (consecutive_recv_failures=%lu)", u.log_tag.c_str(),
                    static_cast<unsigned long>(u.controller->consecutive_recv_failures()));
      }
      u.prev_link_ok = link_ok;
    }
    u.link_msg.data = link_ok;
    u.link_status_pub->publish(u.link_msg);
  }

  // Same accumulate-and-carry reason as the link decimation above: `% 500` on an
  // invocation counter would fire at an unrelated rate once one invocation
  // stopped meaning one cycle. Same helper too — this used to hand-copy the
  // arithmetic because the counter was std::size_t and the helper takes int&.
  if (udp_hand_driver::AdvanceDecimation(u.publish_count, cycles, kCycleLogInterval)) {
    RCLCPP_DEBUG(::udp_hand_driver::logging::NodeLogger(), "cycles: %zu",
                 u.controller->cycle_count());
  }
}

void UdpHandNode::PublishCalibrationStatus() {
  static constexpr std::array<uint8_t, 1> kTrackedSensors = {
      udp_hand_driver::calibration::kSensorBarometer,
  };

  for (const auto& unit : units_) {
    if (!unit->controller || !unit->calib_status_pub)
      continue;
    for (const auto sensor_type : kTrackedSensors) {
      const auto snap = unit->controller->GetCalibrationStatus(sensor_type);
      rtc_msgs::msg::CalibrationStatus msg;
      msg.header.stamp = this->now();
      msg.sensor_type = snap.sensor_type;
      msg.state = snap.state;
      msg.progress_count = snap.progress_count;
      msg.target_count = snap.target_count;
      unit->calib_status_pub->publish(msg);
    }
  }
}
