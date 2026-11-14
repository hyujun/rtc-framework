// The locomotion whole-body QP over a walking-like contact schedule
// (loco_gait.hpp): the support set changes four times a cycle while the
// kinematics stay consistent with it.
//
//   * the default solver settings' trace (a, λ, τ, status, iterations of every
//     tick of six cycles) is pinned by its digest, so a change that is meant to
//     leave the defaults alone is seen to.

#include "loco_gait.hpp"
#include "loco_wbc_fixture.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>

namespace rtc::tsid {
namespace {

std::uint64_t Fnv1a(std::uint64_t h, const void* data, std::size_t n) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h ^= bytes[i];
    h *= 1099511628211ULL;
  }
  return h;
}

std::uint64_t Fnv1a(std::uint64_t h, const Eigen::VectorXd& v) {
  return Fnv1a(h, v.data(), static_cast<std::size_t>(v.size()) * sizeof(double));
}

// The digest of six cycles of the default settings (regularised inactive
// forces, the previous-result warm start), on GCC / x86-64 in Release when it
// was pinned — the repo sets no -march, -ffast-math or -ffp-contract. It is a number to compare
// against, not a claim that these torques are right: a change meant to alter the default trace
// replaces it in a commit of its own that says why.
constexpr std::uint64_t kDefaultTraceDigest = 0x206fed7e0217ef60ULL;

// Six cycles: enough for the default warm start to miss on some ticks, so the
// ticks after a miss are pinned too.
constexpr int kTraceCycles = 6;

TEST(LocomotionGaitTest, TheDefaultSettingsTraceIsUnchanged) {
  test::LocoRig rig;
  test::LocoGait gait(*rig.model);
  std::uint64_t h = 14695981039346656037ULL;
  int valid = 0;
  for (int t = 0; t < kTraceCycles * test::LocoGait::kCycle; ++t) {
    gait.Apply(t, rig);
    const CommandOutput& out = rig.Solve();
    valid += out.tau_valid ? 1 : 0;
    h = Fnv1a(h, out.a_opt);
    h = Fnv1a(h, out.lambda_opt);
    h = Fnv1a(h, out.tau);
    h = Fnv1a(h, &out.qp_status, sizeof(out.qp_status));
    h = Fnv1a(h, &out.qp_iterations, sizeof(out.qp_iterations));
  }
  char hex[17];
  std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(h));
  RecordProperty("default_trace_digest", hex);
  RecordProperty("default_trace_valid_ticks", std::to_string(valid));
  EXPECT_EQ(h, kDefaultTraceDigest)
      << "the default trace changed: digest " << hex
      << ". If the change is meant to alter it, replace the constant in its own commit with "
         "the reason; otherwise the defaults are not preserved.";
}

}  // namespace
}  // namespace rtc::tsid
