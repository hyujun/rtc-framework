#include "udp_hand_driver/udp_hand_logging.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"

#include <array>
#include <string>

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

void UdpHandNode::PublishFromEventLoop(const udp_hand_driver::UdpHandState& state,
                                       const udp_hand_driver::FingertipFTState& ft_state) {
  const auto stamp = this->now();

  if (state.joint_valid) {
    joint_js_msg_.header.stamp = stamp;
    for (int i = 0; i < udp_hand_driver::kNumHandMotors; ++i) {
      const auto iu = static_cast<std::size_t>(i);
      joint_js_msg_.position[iu] = static_cast<double>(state.joint_positions[iu]);
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
    sensor_msg_.header.stamp = this->now();

    const bool ft_valid = ft_enabled_ && ft_state.valid;

    for (int f = 0; f < state.num_fingertips && f < static_cast<int>(sensor_msg_.fingertips.size());
         ++f) {
      auto& fs = sensor_msg_.fingertips[static_cast<std::size_t>(f)];

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
  ++link_cycle_counter_;
  if (link_cycle_counter_ >= link_decimation_) {
    link_cycle_counter_ = 0;

    const uint64_t failures = controller_->consecutive_recv_failures();
    const bool link_ok = (failures < link_fail_threshold_);
    if (link_ok != prev_link_ok_) {
      if (link_ok) {
        RCLCPP_INFO(::udp_hand_driver::logging::NodeLogger(), "Hand UDP link UP");
      } else {
        RCLCPP_WARN(::udp_hand_driver::logging::NodeLogger(), "Hand UDP link DOWN (failures=%lu)",
                    static_cast<unsigned long>(failures));
      }
      prev_link_ok_ = link_ok;
    }
    link_msg_.data = link_ok;
    link_status_pub_->publish(link_msg_);
  }

  if (++publish_count_ % 500 == 0) {
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
