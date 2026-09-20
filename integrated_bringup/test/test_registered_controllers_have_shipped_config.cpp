// ── The registry ⟷ shipped-config contract ───────────────────────────────────
//
// One binary serves every robot and registration happens at static-init, so
// every registered controller is instantiated on every profile. A profile that
// ships no YAML for a controller therefore has to mean something, and there are
// exactly two defensible meanings:
//
//   * the controller runs on built-in defaults (issue #196 decision D2), or
//   * the controller is not used on this robot — ControllerEntry::config_required,
//     and the CM skips it.
//
// A controller that can do neither — one whose LoadConfig refuses an absent
// config — registered plain takes the WHOLE bring-up down on that robot,
// including the controllers whose configs are fine. That is not hypothetical:
// demo_inference_controller was registered plain while refusing an empty node,
// and ur5e_p1a and iiwa7_leap — two of three profiles — could not bring up at
// all, from the day it landed. This file is the gate for that class of defect.
//
// Nothing here is a hand-maintained list. Profiles are discovered from the
// config tree and controllers from the live registry, so a profile added
// tomorrow, or a controller added tomorrow, is covered without editing this
// file. What it does NOT check is whether a shipped config is adequate for its
// robot — see test_demo_wbc_config_lint.cpp for that question.
//
// ── Why there is no "does LoadConfig refuse an empty node?" probe ────────────
//
// The obvious stronger gate is to construct each controller and call
// LoadConfig({}), classifying the throwers. It was written, and it misclassified
// three of the five controllers, because with no URDF they throw out of arm-model
// construction rather than out of config validation. Giving the probe a real URDF
// would make the test build a pinocchio model per controller and pick a robot,
// which is a different (and robot-specific) test. The filesystem check below
// needs no probe: it fails on exactly the situation that breaks a bring-up.
//
// Note for anyone tempted to reinstate the probe with `if (!cfg)` reasoning:
// in yaml-cpp a DEFAULT-CONSTRUCTED node (what the CM passes when the file is
// absent) is truthy and Null, while a MISSING KEY node (file present, key
// misspelled) is falsy. `if (!cfg) return;` guards therefore fire on the typo
// case, not the absent-file case — the opposite of what they read like.

#include "rtc_controller_interface/controller_registry.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <set>
#include <string>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

// Profiles are discovered, never listed.
std::set<std::string> DiscoverProfiles() {
  std::set<std::string> profiles;
  for (const auto& dir : std::filesystem::directory_iterator(RTC_DEMO_SHARED_CONFIG_DIR)) {
    if (dir.is_directory() && std::filesystem::exists(dir.path() / "controllers")) {
      profiles.insert(dir.path().filename().string());
    }
  }
  return profiles;
}

bool ProfileShips(const std::string& profile, const rtc::ControllerEntry& entry) {
  const std::filesystem::path path = std::filesystem::path(RTC_DEMO_SHARED_CONFIG_DIR) / profile /
                                     "controllers" /
                                     (entry.config_subdir + entry.config_key + ".yaml");
  return std::filesystem::exists(path);
}

// ── Canaries ────────────────────────────────────────────────────────────────
//
// Every assertion below quantifies over the registry and over the discovered
// profiles. Either being empty satisfies all of them vacuously, and an empty
// registry is exactly what a --whole-archive regression on this target
// produces. These run first.

TEST(RegisteredControllerConfig, TheRegistryIsActuallyPopulated) {
  const auto& entries = rtc::ControllerRegistry::Instance().GetEntries();
  ASSERT_FALSE(entries.empty())
      << "controller registry is empty — the --whole-archive link flag on this test target "
         "regressed, and every assertion in this file would pass vacuously";

  bool found_inference = false;
  for (const auto& entry : entries) {
    if (entry.config_key == "demo_inference_controller") {
      found_inference = true;
    }
  }
  EXPECT_TRUE(found_inference) << "demo_inference_controller is not registered; either it was "
                                  "removed (update this canary) or its TU was stripped";
}

TEST(RegisteredControllerConfig, ProfilesAreDiscoverable) {
  const auto profiles = DiscoverProfiles();
  ASSERT_FALSE(profiles.empty()) << "no profile directories under " << RTC_DEMO_SHARED_CONFIG_DIR;
  for (const auto& profile : profiles) {
    EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(RTC_DEMO_SHARED_CONFIG_DIR) /
                                        profile / "controllers" / "demo_shared.yaml"))
        << profile << " has a controllers/ directory but no demo_shared.yaml — discovery is "
        << "picking up something that is not a robot profile";
  }
}

TEST(RegisteredControllerConfig, TheGateHasSomethingToRuleOut) {
  // If every profile shipped every controller's YAML, the contract below would
  // be satisfied without ever exercising config_required, and the day someone
  // dropped a YAML it would be the first real evaluation. Assert that the
  // asymmetry this gate exists for is actually present.
  const auto profiles = DiscoverProfiles();
  bool some_pair_unshipped = false;
  for (const auto& entry : rtc::ControllerRegistry::Instance().GetEntries()) {
    for (const auto& profile : profiles) {
      if (!ProfileShips(profile, entry)) {
        some_pair_unshipped = true;
      }
    }
  }
  EXPECT_TRUE(some_pair_unshipped)
      << "every profile ships every registered controller's YAML — this gate is currently "
         "vacuous; that is fine, but check the paths it builds still match the real layout";
}

// ── The contract ────────────────────────────────────────────────────────────

TEST(RegisteredControllerConfig, AnUnshippedControllerMustBeRegisteredAsRequiringConfig) {
  const auto profiles = DiscoverProfiles();
  for (const auto& entry : rtc::ControllerRegistry::Instance().GetEntries()) {
    if (entry.config_required) {
      continue;  // skipped where unshipped — that is the sanctioned meaning
    }
    for (const auto& profile : profiles) {
      EXPECT_TRUE(ProfileShips(profile, entry))
          << "profile '" << profile << "' ships no controllers/" << entry.config_subdir
          << entry.config_key << ".yaml, and '" << entry.config_key
          << "' is registered with RTC_REGISTER_CONTROLLER.\n"
             "That is only safe if the controller runs on built-in defaults (issue #196 D2). "
             "If it refuses an absent config — as a policy controller must — the CM fails its "
             "PreConfigure and the D1 checkpoint refuses EVERY controller on this robot.\n"
             "Fix by shipping the YAML, or by registering it with "
             "RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG so the CM skips it here.";
    }
  }
}

TEST(RegisteredControllerConfig, EveryRequiredConfigControllerIsShippedSomewhere) {
  // The other direction. A config_required controller that no profile
  // configures is now skipped on every robot — silently, since skipping is not
  // an error. Deleting the last profile's YAML must not be a no-op.
  const auto profiles = DiscoverProfiles();
  for (const auto& entry : rtc::ControllerRegistry::Instance().GetEntries()) {
    if (!entry.config_required) {
      continue;
    }
    bool shipped_anywhere = false;
    for (const auto& profile : profiles) {
      if (ProfileShips(profile, entry)) {
        shipped_anywhere = true;
      }
    }
    EXPECT_TRUE(shipped_anywhere)
        << "'" << entry.config_key
        << "' requires a config file and NO profile ships one, so it is skipped everywhere and "
           "cannot run on any robot";
  }
}

}  // namespace
