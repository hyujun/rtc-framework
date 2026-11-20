// locomotion_controller: a sensor_msgs/Joy as the velocity command
// (`velocity_command.type: joy`). Pure — no ROS, no clock, no allocation after
// configure; the subscription callback calls JoyToCommand and stores its result
// exactly as the Twist callback stores a Twist.
//
// The standard gamepad layout below is a COPY of rtc_teleop_input's
// config/standard_gamepad.yaml (the SDL GameController order that joy's
// game_controller_node publishes). The controller does not read that file at
// configure (no lookup into another package's install on the RT host's
// configure path); test_locomotion_joy_command compares the copy with the
// installed file, name by name.
//
//   velocity_command:
//     type: joy                 # twist (the default when absent) | joy
//     topic: <name>             # sensor_msgs/Joy
//     joy:                      # required with type joy, refused with twist
//       linear_x: <axis name>   # v_x  = clamp(axis, ±1) × command.v_max[0]
//       linear_y: <axis name>   # v_y  = clamp(axis, ±1) × command.v_max[1]
//       angular_z: <axis name>  # ω_z  = clamp(axis, ±1) × command.v_max[2]
//       height_rate: <axis name>  # optional: ḣ = clamp(axis, ±1) × command.hdot_max
//       deadman: <button name>  # the command is zero unless this button reads exactly 1
//
// A trigger (triggerleft / triggerright) rests at 0 and reads down to −1, so
// mapped to a command it moves one way only — a stick axis is the usual choice.
// Every axis named once; names from the layout (an axis where a button goes, or
// the reverse, is refused). A message is turned into all-zero axes when the
// deadman is not exactly 1 (released), and also — counted as a reject — when a
// mapped index is outside the message's arrays or a mapped axis is not finite.
// Unlike the Twist path's "a non-finite axis keeps its last value", a human
// input that cannot be read stops: holding the last value would keep the robot
// moving after the hand let go. header.stamp is not read (the receive time on
// the steady clock is the age, as for a Twist).
#ifndef INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_JOY_COMMAND_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_JOY_COMMAND_HPP_

#include "rtc_mpc/reference/command_filter.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace integrated_bringup::locomotion {

/// sensor_msgs/Joy axes / buttons, index = position (rtc_teleop_input
/// config/standard_gamepad.yaml). Stick pushed left or up reads positive.
inline constexpr std::array<std::string_view, 6> kStandardGamepadAxes{
    "leftx", "lefty", "rightx", "righty", "triggerleft", "triggerright"};
inline constexpr std::array<std::string_view, 21> kStandardGamepadButtons{"a",
                                                                          "b",
                                                                          "x",
                                                                          "y",
                                                                          "back",
                                                                          "guide",
                                                                          "start",
                                                                          "leftstick",
                                                                          "rightstick",
                                                                          "leftshoulder",
                                                                          "rightshoulder",
                                                                          "dpad_up",
                                                                          "dpad_down",
                                                                          "dpad_left",
                                                                          "dpad_right",
                                                                          "misc1",
                                                                          "paddle1",
                                                                          "paddle2",
                                                                          "paddle3",
                                                                          "paddle4",
                                                                          "touchpad"};

/// Layout indices of `velocity_command.joy`; −1 = unmapped (height_rate only).
struct JoyCommandMapping {
  int linear_x{-1};
  int linear_y{-1};
  int angular_z{-1};
  int height_rate{-1};
  int deadman{-1};
};

/// Full-deflection command per mapped axis: command.v_max and command.hdot_max.
struct JoyCommandScale {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  double height_rate{0.0};
};

/// One Joy message as a command.
struct JoyCommandResult {
  rtc::mpc::RawVelocityCommand twist{};  ///< all zero unless pressed and readable
  /// 1 the deadman read exactly 1, 0 it read anything else, −1 the message
  /// has no such button (rejected before it was read).
  std::int8_t deadman{-1};
  bool rejected{false};  ///< too short, or a mapped axis non-finite
};

/// The command one message asks for (rules in the file comment).
[[nodiscard]] JoyCommandResult JoyToCommand(const JoyCommandMapping& map,
                                            const JoyCommandScale& scale,
                                            std::span<const float> axes,
                                            std::span<const std::int32_t> buttons) noexcept;

/// `velocity_command.joy` → indices.
/// @throws std::invalid_argument naming `where.<key>`: not a map, an unknown or
///         missing key, a name not in the layout (or of the other kind), one
///         axis mapped twice.
[[nodiscard]] JoyCommandMapping ParseJoyCommandMapping(const YAML::Node& joy,
                                                       const std::string& where);

}  // namespace integrated_bringup::locomotion

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_JOY_COMMAND_HPP_
