#include "integrated_bringup/support/virtual_tcp_params.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>
#include <exception>
#include <string>
#include <vector>

namespace integrated_bringup {

namespace {

const char* ApplyTriple(const rclcpp::Parameter& p, const char* wrong_type,
                        const char* wrong_length, const char* non_finite,
                        std::array<double, 3>& out) noexcept {
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    return wrong_type;
  }
  const auto& v = p.as_double_array();
  if (v.size() != 3) {
    return wrong_length;
  }
  for (const double x : v) {
    if (!std::isfinite(x)) {
      return non_finite;
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out[i] = v[i];
  }
  return nullptr;
}

rclcpp::Parameter SeedParameter(const char* name, const VirtualTcpConfig& seed) {
  if (std::string_view(name) == kVirtualTcpModeParam) {
    return {name, std::string(VirtualTcpModeName(seed.mode))};
  }
  const auto& a = std::string_view(name) == kVirtualTcpOffsetParam ? seed.offset : seed.orientation;
  return {name, std::vector<double>(a.begin(), a.end())};
}

const char* Description(std::string_view name) noexcept {
  if (name == kVirtualTcpModeParam) {
    return "Virtual TCP mode: 'disabled' | 'centroid' | 'weighted' | 'constant'. "
           "Settable only while the controller is inactive.";
  }
  if (name == kVirtualTcpOffsetParam) {
    return "Virtual TCP offset [x, y, z] in the TCP frame [m], constant mode. "
           "Settable only while the controller is inactive.";
  }
  return "Virtual TCP orientation [roll, pitch, yaw] in the TCP frame [rad]. "
         "Settable only while the controller is inactive.";
}

}  // namespace

const char* VirtualTcpModeName(VirtualTcpMode mode) noexcept {
  for (const auto& [m, name] : kVirtualTcpModeNames) {
    if (m == mode) {
      return name.data();
    }
  }
  return kVirtualTcpModeNames[0].second.data();
}

std::optional<VirtualTcpMode> ParseVirtualTcpMode(std::string_view name) noexcept {
  for (const auto& [m, n] : kVirtualTcpModeNames) {
    if (n == name) {
      return m;
    }
  }
  return std::nullopt;
}

bool IsVirtualTcpParameter(std::string_view name) noexcept {
  return name == kVirtualTcpModeParam || name == kVirtualTcpOffsetParam ||
         name == kVirtualTcpOrientationParam;
}

const char* ApplyVirtualTcpParameter(const rclcpp::Parameter& p, bool has_hand,
                                     VirtualTcpConfig& vtcp) noexcept {
  const std::string_view name = p.get_name();
  if (name == kVirtualTcpModeParam) {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      return "virtual_tcp_mode must be a string";
    }
    const auto mode = ParseVirtualTcpMode(p.as_string());
    if (!mode) {
      return "virtual_tcp_mode must be one of 'disabled', 'centroid', 'weighted', 'constant'";
    }
    if (*mode != VirtualTcpMode::kDisabled && *mode != vtcp.mode && !has_hand) {
      return "virtual_tcp_mode other than 'disabled' needs a hand model, and this controller "
             "has none";
    }
    vtcp.mode = *mode;
    return nullptr;
  }
  if (name == kVirtualTcpOffsetParam) {
    return ApplyTriple(p, "virtual_tcp_offset must be a double array",
                       "virtual_tcp_offset requires 3 values", "virtual_tcp_offset must be finite",
                       vtcp.offset);
  }
  if (name == kVirtualTcpOrientationParam) {
    return ApplyTriple(p, "virtual_tcp_orientation must be a double array",
                       "virtual_tcp_orientation requires 3 values",
                       "virtual_tcp_orientation must be finite", vtcp.orientation);
  }
  return "not a virtual_tcp_* parameter";
}

const char* ApplyVirtualTcpSet(const rclcpp::Parameter& p, bool active, bool has_hand,
                               VirtualTcpConfig& vtcp) noexcept {
  if (active) {
    return kVtcpRefusedActiveReason;
  }
  return ApplyVirtualTcpParameter(p, has_hand, vtcp);
}

void DeclareVirtualTcpParameters(rclcpp_lifecycle::LifecycleNode& node, bool has_hand,
                                 const rclcpp::Logger& logger, VirtualTcpConfig& vtcp) noexcept {
  const VirtualTcpConfig seed = vtcp;
  try {
    for (const char* name :
         {kVirtualTcpModeParam, kVirtualTcpOffsetParam, kVirtualTcpOrientationParam}) {
      const rclcpp::Parameter seed_param = SeedParameter(name, seed);
      if (!node.has_parameter(name)) {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = Description(name);
        node.declare_parameter(name, seed_param.get_parameter_value(), d);
        continue;
      }
      const rclcpp::Parameter stored = node.get_parameter(name);
      if (stored.get_parameter_value() == seed_param.get_parameter_value()) {
        continue;
      }
      if (const char* why = ApplyVirtualTcpParameter(stored, has_hand, vtcp); why != nullptr) {
        RCLCPP_ERROR(logger,
                     "%s: the value stored while the controller was cleaned up is rejected at "
                     "configure (%s); keeping the demo_shared.yaml value",
                     name, why);
        const auto r = node.set_parameter(seed_param);
        if (!r.successful) {
          RCLCPP_ERROR(logger, "%s: could not reset the parameter to the YAML value (%s)", name,
                       r.reason.c_str());
        }
      }
    }
  } catch (const std::exception& e) {
    // No reset to the seed here: a key handled before the throw already agrees
    // with the server, and overwriting it would split the two.
    RCLCPP_ERROR(logger,
                 "virtual_tcp_* parameter declaration failed part way (%s); keys not reached keep "
                 "the demo_shared.yaml value",
                 e.what());
  } catch (...) {
    RCLCPP_ERROR(logger,
                 "virtual_tcp_* parameter declaration failed part way; keys not reached "
                 "keep the demo_shared.yaml value");
  }
}

}  // namespace integrated_bringup
