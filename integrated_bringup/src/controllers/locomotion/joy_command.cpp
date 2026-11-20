// See joy_command.hpp.
#include "integrated_bringup/controllers/locomotion/joy_command.hpp"

#include "rtc_controllers/params/yaml_read.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace integrated_bringup::locomotion {

namespace {

[[noreturn]] void Refuse(const std::string& where, const std::string& what) {
  throw std::invalid_argument(where + ": " + what);
}

template <std::size_t N>
int IndexOf(const std::array<std::string_view, N>& names, const std::string& name) {
  const auto it = std::find(names.begin(), names.end(), name);
  return it == names.end() ? -1 : static_cast<int>(it - names.begin());
}

// The axis at `index`, clamped to [−1, 1] in float, then widened; false when
// the message has no such axis or it is not finite. An unmapped axis (−1)
// reads 0 and never fails.
bool ReadAxis(std::span<const float> axes, int index, double& out) noexcept {
  if (index < 0) {
    out = 0.0;
    return true;
  }
  if (static_cast<std::size_t>(index) >= axes.size()) {
    return false;
  }
  const float v = axes[static_cast<std::size_t>(index)];
  if (!std::isfinite(v)) {
    return false;
  }
  out = static_cast<double>(std::clamp(v, -1.0F, 1.0F));
  return true;
}

}  // namespace

JoyCommandResult JoyToCommand(const JoyCommandMapping& map, const JoyCommandScale& scale,
                              std::span<const float> axes,
                              std::span<const std::int32_t> buttons) noexcept {
  JoyCommandResult r{};
  const auto d = static_cast<std::size_t>(map.deadman);
  if (map.deadman < 0 || d >= buttons.size()) {
    r.rejected = true;
    return r;
  }
  r.deadman = buttons[d] == 1 ? 1 : 0;
  double x = 0.0;
  double y = 0.0;
  double wz = 0.0;
  double h = 0.0;
  if (!(ReadAxis(axes, map.linear_x, x) && ReadAxis(axes, map.linear_y, y) &&
        ReadAxis(axes, map.angular_z, wz) && ReadAxis(axes, map.height_rate, h))) {
    r.rejected = true;
    return r;
  }
  if (r.deadman != 1) {
    return r;
  }
  r.twist.linear_x = x * scale.linear_x;
  r.twist.linear_y = y * scale.linear_y;
  r.twist.angular_z = wz * scale.angular_z;
  r.twist.linear_z = h * scale.height_rate;
  return r;
}

JoyCommandMapping ParseJoyCommandMapping(const YAML::Node& joy, const std::string& where) {
  using rtc::params::FindMissingKey;
  using rtc::params::FindUnknownKey;
  using rtc::params::Spelling;
  if (!joy.IsMap()) {
    Refuse(where, "expected a map, got " + Spelling(joy));
  }
  if (const auto unknown =
          FindUnknownKey(joy, {"linear_x", "linear_y", "angular_z", "height_rate", "deadman"})) {
    Refuse(where, "unknown key '" + *unknown + "'");
  }
  if (const auto missing = FindMissingKey(joy, {"linear_x", "linear_y", "angular_z", "deadman"})) {
    Refuse(where, "required key '" + *missing + "' is missing");
  }
  // A layout name's index in `names`; an absent optional key is −1.
  const auto index = [&](const char* key, const auto& names, const char* kind) -> int {
    const YAML::Node n = joy[key];
    if (!n) {
      return -1;
    }
    const int i = n.IsScalar() ? IndexOf(names, n.Scalar()) : -1;
    if (i < 0) {
      Refuse(where + "." + key, "expected " + std::string(kind) +
                                    " of the standard gamepad layout, got " + Spelling(n));
    }
    return i;
  };
  JoyCommandMapping m{};
  m.linear_x = index("linear_x", kStandardGamepadAxes, "an axis");
  m.linear_y = index("linear_y", kStandardGamepadAxes, "an axis");
  m.angular_z = index("angular_z", kStandardGamepadAxes, "an axis");
  m.height_rate = index("height_rate", kStandardGamepadAxes, "an axis");
  m.deadman = index("deadman", kStandardGamepadButtons, "a button");
  unsigned used = 0;
  for (const int axis : {m.linear_x, m.linear_y, m.angular_z, m.height_rate}) {
    if (axis >= 0 && (used & (1U << axis)) != 0) {
      Refuse(where, "axis '" + std::string(kStandardGamepadAxes[static_cast<std::size_t>(axis)]) +
                        "' is mapped twice");
    }
    used |= axis >= 0 ? 1U << axis : 0U;
  }
  return m;
}

}  // namespace integrated_bringup::locomotion
