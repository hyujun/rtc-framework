// See joy_command.hpp.
#include "integrated_bringup/controllers/locomotion/joy_command.hpp"

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
  r.deadman = buttons[d] == 1;
  double x = 0.0;
  double y = 0.0;
  double wz = 0.0;
  double h = 0.0;
  if (!(ReadAxis(axes, map.linear_x, x) && ReadAxis(axes, map.linear_y, y) &&
        ReadAxis(axes, map.angular_z, wz) && ReadAxis(axes, map.height_rate, h))) {
    r.rejected = true;
    return r;
  }
  if (!r.deadman) {
    return r;
  }
  r.twist.linear_x = x * scale.linear_x;
  r.twist.linear_y = y * scale.linear_y;
  r.twist.angular_z = wz * scale.angular_z;
  r.twist.linear_z = h * scale.height_rate;
  return r;
}

JoyCommandMapping ParseJoyCommandMapping(const YAML::Node& joy, const std::string& where) {
  if (!joy.IsMap()) {
    Refuse(where, "expected a map");
  }
  static constexpr std::array<std::string_view, 5> kKeys{"linear_x", "linear_y", "angular_z",
                                                         "height_rate", "deadman"};
  for (const auto& kv : joy) {
    const auto key = kv.first.as<std::string>();
    if (std::find(kKeys.begin(), kKeys.end(), key) == kKeys.end()) {
      Refuse(where, "unknown key '" + key + "'");
    }
  }
  const auto name = [&](const char* key, bool required) -> std::string {
    const YAML::Node n = joy[key];
    if (!n) {
      if (required) {
        Refuse(where, std::string("required key '") + key + "' is missing");
      }
      return {};
    }
    if (!n.IsScalar()) {
      Refuse(where + "." + key, "expected a layout name");
    }
    return n.Scalar();
  };
  const auto axis = [&](const char* key, bool required) -> int {
    const std::string n = name(key, required);
    if (n.empty()) {
      if (required) {
        Refuse(where + "." + key, "expected an axis name, got an empty one");
      }
      return -1;
    }
    const int i = IndexOf(kStandardGamepadAxes, n);
    if (i < 0) {
      Refuse(where + "." + key, "'" + n + "' is not an axis of the standard gamepad layout");
    }
    return i;
  };
  JoyCommandMapping m{};
  m.linear_x = axis("linear_x", true);
  m.linear_y = axis("linear_y", true);
  m.angular_z = axis("angular_z", true);
  m.height_rate = axis("height_rate", false);
  const std::string deadman = name("deadman", true);
  m.deadman = IndexOf(kStandardGamepadButtons, deadman);
  if (m.deadman < 0) {
    Refuse(where + ".deadman", "'" + deadman + "' is not a button of the standard gamepad layout");
  }
  const std::array<int, 4> mapped{m.linear_x, m.linear_y, m.angular_z, m.height_rate};
  for (std::size_t i = 0; i < mapped.size(); ++i) {
    for (std::size_t j = i + 1; j < mapped.size(); ++j) {
      if (mapped[i] >= 0 && mapped[i] == mapped[j]) {
        Refuse(where, "axis '" +
                          std::string(kStandardGamepadAxes[static_cast<std::size_t>(mapped[i])]) +
                          "' is mapped twice");
      }
    }
  }
  return m;
}

}  // namespace integrated_bringup::locomotion
