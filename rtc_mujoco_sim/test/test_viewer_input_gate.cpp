// ── test_viewer_input_gate.cpp ────────────────────────────────────────────────
// viewer_input_enabled: which viewer keys count as world-changing, and the
// Config default. No MJCF load, no GLFW — the key set is GLFW-free on purpose
// (viewer_input_policy.hpp), so this runs on a headless build too. That each
// raw code equals its GLFW_KEY_* is pinned by static_asserts in
// viewer_callbacks.cpp, which only compiles when GLFW is present.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"
#include "viewer/viewer_input_policy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstring>

namespace rtc {
namespace {

// Raw GLFW key codes of the keys OnKey handles that change the world, written
// out as numbers here so a wrong constant in the header cannot also be the
// expectation.
constexpr auto kWorldKeys = std::to_array<int>({
    32,   // Space: pause / resume
    262,  // Right: single step
    61,   // =: RTF x2
    334,  // KP+: RTF x2
    45,   // -: RTF /2
    333,  // KP-: RTF /2
    82,   // R: reset
    75,   // K: projectile ball launch
    79,   // O: object refresh
    71,   // G: world gravity
    78,   // N: contacts
    73,   // I: integrator
    83,   // S: solver
    93,   // ]: solver iterations x2
    91,   // [: solver iterations /2
});

// Keys OnKey handles that only touch the camera, the display or an overlay,
// plus the modifiers OnKey tracks. None of these may be gated.
constexpr auto kViewOnlyKeys = std::to_array<int>({
    290,                         // F1: help
    258,                         // Tab: camera mode
    256,                         // Escape: home camera
    259,                         // Backspace: reset vis options
    67,  70,  84,  74,  66,      // C F T J B: vis flags / frames
    85,  69,  87,  76,  65, 88,  // U E W L A X: vis flags
    86,                          // V: geomgroup 0 alias
    48,  49,  50,  51,  52, 53,  // 0-5: geom groups
    294, 295, 296, 297,          // F5-F8: render flags
    292, 293, 298, 299,          // F3 F4 F9 F10: overlays
    80,                          // P: screenshot
    340, 341, 344, 345,          // Left/Right Shift, Left/Right Ctrl
    263, 264, 265,               // Left / Down / Up arrows: unbound
    342, 346,                    // Left / Right Alt: unbound
    257,                         // Enter: unbound
    -1,                          // GLFW_KEY_UNKNOWN
    0,                           // not a key code
    348,                         // GLFW_KEY_LAST (Menu): unbound
});

TEST(ViewerInputGate, EveryWorldChangingKeyIsGated) {
  for (const int key : kWorldKeys) {
    EXPECT_TRUE(viewer::IsWorldKey(key)) << "key " << key;
    const char* name = viewer::WorldKeyName(key);
    ASSERT_NE(name, nullptr) << "key " << key;
    EXPECT_GT(std::strlen(name), 0U) << "key " << key;
  }
}

TEST(ViewerInputGate, CameraDisplayAndModifierKeysAreNotGated) {
  for (const int key : kViewOnlyKeys) {
    EXPECT_FALSE(viewer::IsWorldKey(key)) << "key " << key;
    EXPECT_EQ(viewer::WorldKeyName(key), nullptr) << "key " << key;
  }
}

// The gated set is exactly kWorldKeys: sweep every GLFW key code (-1 .. 348)
// so a key added to the header without a matching expectation here — or a
// constant that silently aliases another key — shows up as a count mismatch.
TEST(ViewerInputGate, GatedSetIsExactlyTheWorldChangingKeys) {
  int gated = 0;
  for (int key = -1; key <= 348; ++key) {
    if (viewer::IsWorldKey(key)) {
      ++gated;
    }
  }
  EXPECT_EQ(gated, static_cast<int>(kWorldKeys.size()));
}

// The header constants are the codes the expectations above are written in.
TEST(ViewerInputGate, ConstantsAreTheRawGlfwCodes) {
  EXPECT_EQ(viewer::kKeySpace, 32);
  EXPECT_EQ(viewer::kKeyRight, 262);
  EXPECT_EQ(viewer::kKeyEqual, 61);
  EXPECT_EQ(viewer::kKeyKpAdd, 334);
  EXPECT_EQ(viewer::kKeyMinus, 45);
  EXPECT_EQ(viewer::kKeyKpSubtract, 333);
  EXPECT_EQ(viewer::kKeyR, 82);
  EXPECT_EQ(viewer::kKeyK, 75);
  EXPECT_EQ(viewer::kKeyO, 79);
  EXPECT_EQ(viewer::kKeyG, 71);
  EXPECT_EQ(viewer::kKeyN, 78);
  EXPECT_EQ(viewer::kKeyI, 73);
  EXPECT_EQ(viewer::kKeyS, 83);
  EXPECT_EQ(viewer::kKeyRightBracket, 93);
  EXPECT_EQ(viewer::kKeyLeftBracket, 91);
}

// The default keeps the viewer as interactive as it has always been; only an
// explicit `viewer_input_enabled: false` gates it.
TEST(ViewerInputGate, ConfigDefaultsToInputEnabled) {
  const MuJoCoSimulator::Config cfg;
  EXPECT_TRUE(cfg.viewer_input_enabled);
}

}  // namespace
}  // namespace rtc
