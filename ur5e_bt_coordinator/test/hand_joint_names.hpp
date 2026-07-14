#pragma once
/// Shared assm_v1 hand joint-name table for both test fixtures (inject + e2e).
///
/// Single in-package copy so a hand-joint rename cannot update one tier's
/// fixture and miss the other (the two fixture headers are never included
/// together, so the compiler would not flag divergence). Mirrors
/// udp_hand_driver::kDefaultHandMotorNames; kept package-local because the
/// tests should not grow a cross-package dependency for one constant.

#include <string>
#include <vector>

namespace rtc_bt::test {

/// assm_v1 10-DoF order (thumb:3 / index:3 / middle:3 / ring:1). Hand nodes
/// map finger→joint via these name prefixes (FingerJointIndices).
inline const std::vector<std::string>& AssmV1JointNames() {
  static const std::vector<std::string> kNames = {
      "thumb_cmc_aa", "thumb_cmc_fe",  "thumb_mcp_fe",  "index_mcp_aa",  "index_mcp_fe",
      "index_dip_fe", "middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe", "ring_mcp_fe"};
  return kNames;
}

}  // namespace rtc_bt::test
