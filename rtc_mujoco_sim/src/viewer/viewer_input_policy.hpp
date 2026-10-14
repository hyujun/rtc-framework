// ── viewer_input_policy.hpp ────────────────────────────────────────────────────
// Which viewer keys change the simulated world, for `viewer_input_enabled`.
//
// With viewer_input_enabled=false the viewer ignores every input that changes
// the world (pause / step / speed / reset / spawn / physics / solver keys and
// the Ctrl+drag perturbation), so an evaluation sim run with the viewer open
// cannot be perturbed by a stray key press. Camera, display and overlay keys
// keep working.
//
// Deliberately GLFW-free: GLFW is optional (MUJOCO_HAVE_GLFW is a PRIVATE
// define of mujoco_simulator_lib), and this set must be testable on a headless
// build too. The constants are raw GLFW key codes; viewer_callbacks.cpp
// static_asserts each one against its GLFW_KEY_* so a drift fails the build.
//
// The set must equal the world-changing cases of OnKey (viewer_callbacks.cpp):
// a key added there that changes the world belongs here as well.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_VIEWER_VIEWER_INPUT_POLICY_HPP_
#define RTC_MUJOCO_SIM_VIEWER_VIEWER_INPUT_POLICY_HPP_

namespace rtc::viewer {

// ── World-changing keys (raw GLFW key codes) ──────────────────────────────────
inline constexpr int kKeySpace = 32;         // pause / resume
inline constexpr int kKeyRight = 262;        // single physics step
inline constexpr int kKeyEqual = 61;         // max RTF x2
inline constexpr int kKeyKpAdd = 334;        // max RTF x2 (keypad)
inline constexpr int kKeyMinus = 45;         // max RTF /2
inline constexpr int kKeyKpSubtract = 333;   // max RTF /2 (keypad)
inline constexpr int kKeyR = 82;             // reset
inline constexpr int kKeyK = 75;             // projectile ball launch
inline constexpr int kKeyO = 79;             // object refresh
inline constexpr int kKeyG = 71;             // world gravity toggle
inline constexpr int kKeyN = 78;             // contact toggle
inline constexpr int kKeyI = 73;             // integrator cycle
inline constexpr int kKeyS = 83;             // solver cycle
inline constexpr int kKeyRightBracket = 93;  // solver iterations x2
inline constexpr int kKeyLeftBracket = 91;   // solver iterations /2

/// Printable name of a world-changing key, or nullptr if `key` does not change
/// the world (camera / display / overlay keys, modifiers, unknown codes).
[[nodiscard]] constexpr const char* WorldKeyName(int key) noexcept {
  switch (key) {
    case kKeySpace:
      return "Space";
    case kKeyRight:
      return "Right";
    case kKeyEqual:
      return "=";
    case kKeyKpAdd:
      return "KP+";
    case kKeyMinus:
      return "-";
    case kKeyKpSubtract:
      return "KP-";
    case kKeyR:
      return "R";
    case kKeyK:
      return "K";
    case kKeyO:
      return "O";
    case kKeyG:
      return "G";
    case kKeyN:
      return "N";
    case kKeyI:
      return "I";
    case kKeyS:
      return "S";
    case kKeyRightBracket:
      return "]";
    case kKeyLeftBracket:
      return "[";
    default:
      return nullptr;
  }
}

/// True if `key` changes the simulated world (and is therefore ignored when
/// viewer_input_enabled=false).
[[nodiscard]] constexpr bool IsWorldKey(int key) noexcept {
  return WorldKeyName(key) != nullptr;
}

}  // namespace rtc::viewer

#endif  // RTC_MUJOCO_SIM_VIEWER_VIEWER_INPUT_POLICY_HPP_
