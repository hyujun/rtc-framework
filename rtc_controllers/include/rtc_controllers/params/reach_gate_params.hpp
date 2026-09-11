// ── Reach gate schema ────────────────────────────────────────────────────────
// The YAML half of inference/reach_gate.hpp: which bodies are the tips, which
// force lane each one reads, where each one is supposed to touch the object,
// and the five constants of the trained formula. POD + parser, yaml-cpp only,
// configure time only.
//
// ROBOT-AGNOSTIC (ARCH-1): links and force groups are opaque names that the
// binding resolves against its own model and sensor roster. Nothing here knows
// a hand.
//
// THE CONTACT POINTS MUST BE IN THE FRAME THE POLICY CALLS "THE OBJECT". A grasp
// record may quote them about a different origin (the one the grasp generator
// used), and the policy may have been trained against a re-centred asset; the
// two differ by a translation that no range check can see — every distance
// just comes out a few centimetres long and the gate never opens. Take them
// from what the policy itself was trained with.
#pragma once

#include <yaml-cpp/yaml.h>

#include <array>
#include <string>
#include <vector>

namespace rtc::params {

/// One tip of the gate. Order matters: `tips[0]` is the opposing digit that the
/// grasped test treats specially (see IsGraspedByForce).
struct ReachGateTip {
  std::string link;                     ///< body whose origin is the tip position
  std::string force_group;              ///< sensor group whose ‖F‖ this tip reads
  std::array<double, 3> contact_obj{};  ///< recorded contact point, object frame [m]
};

struct ReachGateParams {
  std::vector<ReachGateTip> tips;
  double tip_std{0.030};        ///< σ of the proximity ramp [m]
  double force_threshold{0.5};  ///< per-tip "pressing" threshold [N], strict >
  int min_fingers{2};           ///< non-opposing tips that must press
  int hold_on_steps{5};         ///< consecutive grasped POLICY steps to latch
  int hold_off_steps{100};      ///< consecutive non-grasped POLICY steps to release
};

/// Parse a `reach_gate:` map. Throws `std::invalid_argument` on: a missing or
/// non-map node; fewer than two tips; a tip without `link` / `force_group` or
/// with a `contact_obj` that is not three finite numbers; a repeated link or
/// force group; `tip_std` not > 0; `force_threshold` negative or non-finite;
/// `min_fingers` outside [1, tips − 1]; either step count < 1.
///
/// Every constant has a default equal to the trained value it came from, but a
/// present key that fails to parse is refused rather than defaulted — the
/// `as<T>(default)` idiom would read a typo as "the trained value".
ReachGateParams ParseReachGateParams(const YAML::Node& node);

}  // namespace rtc::params
