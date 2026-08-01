#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace {
// Cycles between the periodic "cycles: N" DEBUG line.
constexpr std::size_t kCycleLogInterval = 500;
}  // namespace

void UdpHandNode::PreallocateMessages() {
  joint_js_msg_.name = joint_names_;
  joint_js_msg_.position.resize(udp_hand_driver::kNumHandMotors);
  joint_js_msg_.velocity.resize(udp_hand_driver::kNumHandMotors);
  joint_js_msg_.effort.resize(udp_hand_driver::kNumHandMotors);

  motor_js_msg_.name = motor_names_;
  motor_js_msg_.position.resize(udp_hand_driver::kNumHandMotors);
  motor_js_msg_.velocity.resize(udp_hand_driver::kNumHandMotors);
  motor_js_msg_.effort.resize(udp_hand_driver::kNumHandMotors);

  sensor_msg_.fingertips.resize(static_cast<std::size_t>(num_fingertips_));
  for (int f = 0; f < num_fingertips_; ++f) {
    auto& fs = sensor_msg_.fingertips[static_cast<std::size_t>(f)];
    fs.name = (static_cast<std::size_t>(f) < fingertip_names_.size())
                  ? fingertip_names_[static_cast<std::size_t>(f)]
                  : "f" + std::to_string(f);
  }
}

void UdpHandNode::PollAndPublish() {
  if (!controller_)
    return;

  // Mask the low bit down to the last *completed* write: an odd sequence means
  // the CommLoop is mid-Store, and treating that as progress would both
  // mis-count cycles and desynchronise the even-valued baseline. Load() itself
  // retries past a torn read, so the snapshot below is at least as new as `seq`.
  const std::uint32_t seq = udp_hand_driver::LastCompletedWrite(controller_->state_sequence());
  const std::uint32_t cycles = udp_hand_driver::CyclesBetween(last_published_seq_, seq);
  if (cycles == 0)
    return;  // no comm cycle completed since the last poll
  last_published_seq_ = seq;

  PublishState(controller_->GetLatestState(), controller_->GetLatestFTState(), cycles);
}

void UdpHandNode::PublishState(const udp_hand_driver::UdpHandState& state,
                               const udp_hand_driver::FingertipFTState& ft_state,
                               std::uint32_t cycles) {
  // Stamped on the consumer thread. This is the publish time, not the sample
  // time — the CommLoop does not call now() (RT-1), and UdpHandState carries no
  // timestamp field, so adding one would change the POD and the SeqLock payload.
  const auto stamp = this->now();

  if (state.joint_valid) {
    joint_js_msg_.header.stamp = stamp;
    for (int i = 0; i < udp_hand_driver::kNumHandMotors; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      // Add the per-joint offset: firmware frame → controller frame.
      // Position-only; velocity/effort are unaffected by a constant offset.
      joint_js_msg_.position[iu] =
          static_cast<double>(state.joint_positions[iu] + joint_offset_rad_[iu]);
      joint_js_msg_.velocity[iu] = static_cast<double>(state.joint_velocities[iu]);
      joint_js_msg_.effort[iu] = static_cast<double>(state.joint_currents[iu]);
    }
    joint_state_pub_->publish(joint_js_msg_);
  }

  if (state.motor_valid) {
    motor_js_msg_.header.stamp = stamp;
    for (int i = 0; i < udp_hand_driver::kNumHandMotors; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      motor_js_msg_.position[iu] = static_cast<double>(state.motor_positions[iu]);
      motor_js_msg_.velocity[iu] = static_cast<double>(state.motor_velocities[iu]);
      motor_js_msg_.effort[iu] = static_cast<double>(state.motor_currents[iu]);
    }
    motor_state_pub_->publish(motor_js_msg_);
  }

  if (state.num_fingertips > 0) {
    sensor_msg_.header.stamp = stamp;

    const bool ft_valid = ft_enabled_ && ft_state.valid;

    for (int f = 0; f < state.num_fingertips && f < static_cast<int>(sensor_msg_.fingertips.size());
         ++f) {
      auto& fs = sensor_msg_.fingertips[static_cast<std::size_t>(f)];

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
        fs.contact_flag = 0.0f;  // no contact classifier on the 1b path yet
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
    sensor_state_pub_->publish(sensor_msg_);
    sensor_monitor_pub_->publish(sensor_msg_);
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
  if (udp_hand_driver::AdvanceDecimation(link_cycle_counter_, cycles, link_decimation_)) {
    // Strict per-channel link health (#1): mirrors the detector's E-STOP decision
    // exactly (same UdpHandController::LinkDown, same thresholds) so the
    // link_status Bool never disagrees with hand_udp_link_down.
    const bool link_ok = !controller_->LinkDown(static_cast<uint32_t>(link_fail_threshold_),
                                                static_cast<uint32_t>(link_fail_threshold_sensor_));
    if (link_ok != prev_link_ok_) {
      if (link_ok) {
        RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "Hand UDP link UP");
      } else {
        RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(),
                    "Hand UDP link DOWN (consecutive_recv_failures=%lu)",
                    static_cast<unsigned long>(controller_->consecutive_recv_failures()));
      }
      prev_link_ok_ = link_ok;
    }
    link_msg_.data = link_ok;
    link_status_pub_->publish(link_msg_);
  }

  // Same accumulate-and-carry reason as the link decimation above: `% 500` on an
  // invocation counter would fire at an unrelated rate once one invocation
  // stopped meaning one cycle.
  publish_count_ += cycles;
  if (publish_count_ >= kCycleLogInterval) {
    publish_count_ %= kCycleLogInterval;
    RCLCPP_DEBUG(::udp_hand_driver::logging::NodeLogger(), "cycles: %zu",
                 controller_->cycle_count());
  }
}

void UdpHandNode::PublishCalibrationStatus() {
  if (!controller_ || !calib_status_pub_)
    return;

  static constexpr std::array<uint8_t, 1> kTrackedSensors = {
      udp_hand_driver::calibration::kSensorBarometer,
  };

  for (const auto sensor_type : kTrackedSensors) {
    const auto snap = controller_->GetCalibrationStatus(sensor_type);
    rtc_msgs::msg::CalibrationStatus msg;
    msg.header.stamp = this->now();
    msg.sensor_type = snap.sensor_type;
    msg.state = snap.state;
    msg.progress_count = snap.progress_count;
    msg.target_count = snap.target_count;
    calib_status_pub_->publish(msg);
  }
}
