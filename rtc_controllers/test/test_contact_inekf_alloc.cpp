// ── Contact InEKF core: zero heap allocation on the RT path ──────────────────
//
// Two gates with different reach, both armed around the same calls:
//   ScopedAllocGate  — global operator new (any TU, including the library's);
//   ScopedMallocGate — the C allocator for the whole executable, which is what
//                      sees Eigen's own allocations inside the static library.
// Each gate gets a positive control made INSIDE the library, so a zero below
// is a measurement and not a sleeping gate.
#include "contact_inekf_testing.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/estimation/inertial_estimator.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rtc::estimation::testing {
namespace {

TEST(ContactInekfAlloc, PositiveControlsFireInsideTheLibrary) {
  {
    // Init()'s refusal builds its message with std::string inside the library.
    ContactInekf f;
    ContactInekf::Config bad = FixtureConfig();
    bad.gravity = -1.0;
    rtc::testing::ScopedAllocGate gate;
    EXPECT_THROW(f.Init(bad, 2), std::invalid_argument);
    EXPECT_GT(gate.count(), 0U);
  }
  {
    // InertialEstimator::Init() sizes dynamic Eigen buffers inside the library.
    InertialEstimator est;
    InertialEstimator::Config c;
    c.forgetting_factor = 0.99;
    c.min_param_sigma = 1e-3;
    c.min_mass = 1e-2;
    c.max_com_offset = 1.0;
    c.max_inertia_column = 1e-3;
    rtc::testing::ScopedMallocGate gate;
    est.Init(7, c);
    EXPECT_GT(gate.count(), 0U);
  }
}

TEST(ContactInekfAlloc, RtCallsAllocateNothing) {
  ContactInekf f;
  f.Init(FixtureConfig(), kInekfMaxContacts);
  const std::array<Vec3, 2> stance{Vec3(0.05, 0.1, -0.7), Vec3(0.05, -0.1, -0.7)};
  StationaryImuAccumulator acc;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  std::size_t news = 0;
  std::size_t mallocs = 0;
  double sink = 0.0;
  int accepted = 0;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    for (int k = 0; k < 200; ++k) {
      accepted += acc.Add(Vec3(1e-4, 0.0, -1e-4), Vec3(0.2, -0.1, 9.80)) ? 1 : 0;
    }
    accepted += acc.Add(Vec3(nan, 0, 0), Vec3::Zero()) ? 1 : 0;
    (void)f.Initialize(acc.MeanGyro(), acc.MeanAccel(), 0.4, 0.3, stance);
    for (int round = 0; round < 2; ++round) {
      for (int k = 0; k < 1000; ++k) {
        const double t = 2e-3 * k;
        (void)f.Propagate(Vec3(std::sin(t), 0.5 * std::cos(t), -0.3),
                          Vec3(0.3 * std::cos(2.0 * t), 0.1, 9.81), 2e-3);
        if (k == 500) {
          (void)f.SetContactNoiseScale(1, 3.0);
          (void)f.Propagate(Vec3(nan, 0, 0), Vec3::Zero(), 2e-3);  // refused
          (void)f.Propagate(Vec3::Zero(), Vec3::Zero(), 1.0);      // refused
        }
        sink += f.orientation().w() + f.position().x() + f.covariance()(0, 0) +
                f.angular_velocity().z() + f.contact_position(1).x();
        const BaseState b = ContactInekf::BaseFromImu(f.rotation(), f.position(), f.velocity(),
                                                      f.angular_velocity(), Mat3::Identity(),
                                                      Vec3(0.04, 0.0, -0.08));
        sink += b.v_WB.x();
      }
      f.Reset();
      (void)f.Initialize(acc.MeanGyro(), acc.MeanAccel(), 0.4, 0.3, stance);
    }
    news = new_gate.count();
    mallocs = malloc_gate.count();
  }
  EXPECT_TRUE(std::isfinite(sink));
  EXPECT_EQ(accepted, 200);
  EXPECT_TRUE(f.initialized());
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(mallocs, 0U);
}

TEST(ContactInekfAlloc, ContactOperationsAllocateNothing) {
  ContactInekf f;
  StartFilter(f, 1, 2);
  ASSERT_TRUE(f.initialized());
  // Everything the RT caller would own is built before the gates.
  std::vector<ContactKinematics> kin(4);
  std::vector<ContactUpdateReport> rep(4);
  Eigen::MatrixXd J = Eigen::MatrixXd::Constant(3, 6, 0.05);
  for (int i = 0; i < 4; ++i) {
    kin[static_cast<std::size_t>(i)] = Kin(FootPoint(i), true, J);
  }
  std::size_t news = 0;
  std::size_t mallocs = 0;
  int ok = 0;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    for (int k = 0; k < 1000; ++k) {
      const double t = 2e-3 * k;
      ok +=
          f.Propagate(Vec3(std::sin(t), 0.3, -0.2), Vec3(0.1, 0.2, 9.81), 2e-3) == InekfStatus::kOk
              ? 1
              : 0;
      if (k % 100 == 50) {
        const int j = 2 + (k / 100) % 2;
        if (f.contact_active(j)) {
          ok += f.RemoveContact(j) == InekfStatus::kOk ? 1 : 0;
        } else {
          ok += f.InsertContact(j, kin[static_cast<std::size_t>(j)]) == InekfStatus::kOk ? 1 : 0;
        }
      }
      for (int i = 0; i < 4; ++i) {
        const auto si = static_cast<std::size_t>(i);
        kin[si].use = f.contact_active(i);
        if (kin[si].use) {
          kin[si].position = f.rotation().transpose() * (f.contact_position(i) - f.position());
        }
      }
      ok += f.UpdateContacts(kin, rep) == InekfStatus::kOk ? 1 : 0;
      (void)f.UpdateContacts(std::span<const ContactKinematics>(kin.data(), 3), rep);  // refused
      if (k % 100 == 25) {  // the support-height pin: applied, inflated, gated, refused
        GroundUpdateReport g;
        ok += f.UpdateGroundHeight(0, 0.0, 2e-3, 10.83, false, g) == InekfStatus::kOk ? 1 : 0;
        ok += f.UpdateGroundHeight(1, 0.0, 2e-3, 10.83, true, g) == InekfStatus::kOk ? 1 : 0;
        ok += f.UpdateGroundHeight(1, 1.0, 2e-3, 10.83, false, g) == InekfStatus::kOk ? 1 : 0;
        (void)f.UpdateGroundHeight(0, 0.0, 0.0, 10.83, false, g);  // refused
      }
    }
    news = new_gate.count();
    mallocs = malloc_gate.count();
  }
  EXPECT_EQ(ok, 2040);
  EXPECT_EQ(news, 0U);
  EXPECT_EQ(mallocs, 0U);
}

}  // namespace
}  // namespace rtc::estimation::testing
