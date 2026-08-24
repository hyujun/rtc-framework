// ── Reading the SHIPPED controller configs, the way the controller reads them ─
//
// Extracted from test_compliance_task_equivalence.cpp when a second suite
// (#469 S3's admittance coupling) needed the same three files. The merge rule
// below is subtle enough that a second transcription of it would be a second
// place to get it wrong — and the failure it produces is a fixture that
// configures a controller production never builds, which reads as a passing
// test of the wrong thing.
//
// RTC_DEMO_SHARED_CONFIG_DIR is injected by CMake and points at
// integrated_bringup/config.
#pragma once

#include "integrated_bringup/support/demo_shared_config.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace integrated_bringup::testfx {

inline YAML::Node ShippedControllerNode(const std::string& profile, const std::string& key) {
  const std::string path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/controllers/" + key + ".yaml";
  const YAML::Node root = YAML::LoadFile(path);
  EXPECT_TRUE(root[key]) << path << ": missing top-level key '" << key << "'";
  return root[key];
}

// Give the controller the profile's shared block.
//
// In production the CM hands it over by declaring `config_variant` on the
// per-controller LifecycleNode, which LoadConfig reads to find
// config/<variant>/controllers/demo_shared.yaml. These fixtures build no node,
// so that lookup falls back to the legacy flat path, finds nothing, and the
// grasp controller, virtual TCP and pull-estimator lanes all stay at their
// built-in defaults — i.e. out of the comparison, which is where a copied
// controller is least observed. LoadConfig applies the controller node's OWN
// keys through the same ApplyDemoSharedConfig entry point (that is the
// documented per-controller override channel), so merging the shipped block in
// here delivers exactly what the node parameter would have. Existing keys win,
// since an override is precisely what they are.
inline void FillMissing(YAML::Node dst, const YAML::Node& src) {
  for (auto it = src.begin(); it != src.end(); ++it) {
    const auto key = it->first.as<std::string>();
    if (!dst[key]) {
      dst[key] = it->second;
    } else if (dst[key].IsMap() && it->second.IsMap()) {
      // Recurse rather than let the present key win whole. Production merges at
      // the STRUCT level — the shared file populates DemoSharedConfig and the
      // controller's own keys are applied on top of it — so a controller that
      // overrides one field of `pull_estimator` keeps every other field of the
      // shared block. Stopping at the top level here would instead delete them,
      // and the fixture would quietly configure a pull estimator that production
      // never builds.
      FillMissing(dst[key], it->second);
    }
  }
}

inline void MergeShippedShared(YAML::Node& cfg, const std::string& profile) {
  const std::string path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/controllers/demo_shared.yaml";
  const YAML::Node shared = YAML::LoadFile(path)["demo_shared"];
  // Throws rather than ASSERT_: an ASSERT_ here would return from THIS function
  // and leave the caller running an unmerged config, which is green for the
  // wrong reason.
  if (!shared || !shared.IsMap()) {
    throw std::runtime_error(path + ": missing 'demo_shared' map");
  }
  FillMissing(cfg, shared);
}

/// The profile's shared + controller config as the controller resolves it.
inline DemoSharedConfig ResolvedShared(const std::string& profile, const std::string& key) {
  YAML::Node cfg = ShippedControllerNode(profile, key);
  MergeShippedShared(cfg, profile);
  DemoSharedConfig out;
  ApplyDemoSharedConfig(cfg, out);
  return out;
}

}  // namespace integrated_bringup::testfx
