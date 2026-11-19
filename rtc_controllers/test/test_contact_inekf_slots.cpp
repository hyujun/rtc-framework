// ── Contact InEKF core: fixed-slot bookkeeping ───────────────────────────────
//
// Every operation (Propagate, InsertContact, RemoveContact, UpdateContacts)
// is replayed on the packed dynamic-size reference from the same input state
// and must agree; insertion / removal are checked bit-exactly where they
// promise to leave things untouched; refusals leave the state bit-identical;
// mixed sequences are deterministic.
#include "contact_inekf_packed_oracle.hpp"
#include "contact_inekf_testing.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

namespace rtc::estimation::testing {
namespace {

constexpr double kDt = 2e-3;

struct Truthish {
  std::mt19937_64 rng;
  std::normal_distribution<double> nd{0.0, 1.0};

  explicit Truthish(std::uint64_t seed) : rng(seed) {}

  Vec3 Randn() { return Vec3(nd(rng), nd(rng), nd(rng)); }
};

/// Body measurements near the filter's own prediction (σ 2 mm).
std::vector<ContactKinematics> NearPrediction(const ContactInekf& f, Truthish& t,
                                              const std::vector<bool>& use) {
  std::vector<ContactKinematics> k = PredictedKinematics(f);
  for (std::size_t i = 0; i < k.size(); ++i) {
    if (f.contact_active(static_cast<int>(i))) {
      k[i].position += t.Randn() * 2e-3;
      k[i].use = use[i];
    }
  }
  return k;
}

TEST(ContactInekfSlots, EveryOperationMatchesThePackedReference) {
  const ContactInekf::Config cfg = FixtureConfig();
  ContactInekf f;
  StartFilter(f, 1, 2);
  ASSERT_TRUE(f.initialized());
  Truthish t(kSeed);
  std::uniform_int_distribution<int> pick(0, 3);
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  std::array<double, 4> worst_x{};
  std::array<double, 4> worst_P{};
  std::array<int, 4> count{};
  const char* names[] = {"Propagate", "Insert", "Remove", "Update"};
  for (int op = 0; op < 2000; ++op) {
    const int kind = pick(t.rng);
    std::vector<int> act;
    std::vector<int> free;
    for (int i = 0; i < 4; ++i) {
      (f.contact_active(i) ? act : free).push_back(i);
    }
    PackedFilter o = PackedFilter::FromCore(f);
    if (kind == 0) {
      const std::vector<int> idx = ActiveIndex(f);
      const Vec3 w(3.0 * ud(t.rng), 3.0 * ud(t.rng), 3.0 * ud(t.rng));
      const Vec3 a = f.rotation().transpose() * Vec3(0, 0, kG0) + t.Randn();
      Mat3 R1;
      Vec3 v1;
      Vec3 p1;
      Access::MeanMap(f.rotation(), f.velocity(), f.position(), w - f.gyro_bias(),
                      a - f.accel_bias(), kDt, kG0, R1, v1, p1);
      ASSERT_EQ(f.Propagate(w, a, kDt), InekfStatus::kOk);
      o.Propagate(Gather(Access::Phi(f), idx), Gather(Access::Qd(f), idx), R1, v1, p1);
    } else if (kind == 1) {
      if (free.empty()) {
        continue;
      }
      const int j = free[static_cast<std::size_t>(op) % free.size()];
      Eigen::MatrixXd J(3, 6);
      for (Eigen::Index c = 0; c < 6; ++c) {
        J.col(c) = t.Randn() * 0.1;
      }
      const ContactKinematics k = Kin(FootPoint(j) + 0.05 * t.Randn(), true, J);
      ASSERT_EQ(f.InsertContact(j, k), InekfStatus::kOk);
      o.Insert(j, k.position, BodyNoise(k, cfg));
    } else if (kind == 2) {
      if (act.empty()) {
        continue;
      }
      const int j = act[static_cast<std::size_t>(op) % act.size()];
      ASSERT_EQ(f.RemoveContact(j), InekfStatus::kOk);
      o.Remove(j);
    } else {
      if (act.empty()) {
        continue;
      }
      std::vector<bool> use(4, false);
      for (int j : act) {
        use[static_cast<std::size_t>(j)] = ud(t.rng) > -0.5;
      }
      const std::vector<ContactKinematics> k = NearPrediction(f, t, use);
      std::vector<ContactUpdateReport> rep(4);
      ASSERT_EQ(f.UpdateContacts(k, rep), InekfStatus::kOk);
      std::vector<PackedMeasurement> m;
      for (int j = 0; j < 4; ++j) {
        const auto sj = static_cast<std::size_t>(j);
        if (rep[sj].result == ContactUpdateResult::kUsed) {
          m.push_back({j, k[sj].position, BodyNoise(k[sj], cfg)});
        }
      }
      if (!m.empty()) {
        o.Update(m);
      }
    }
    const auto [dx, dP] = o.Distance(f);
    const auto ki = static_cast<std::size_t>(kind);
    worst_x[ki] = std::max(worst_x[ki], dx);
    worst_P[ki] = std::max(worst_P[ki], dP);
    ++count[ki];
  }
  bool pass = true;
  for (std::size_t i = 0; i < 4; ++i) {
    std::printf("[measure] %-9s n=%4d: state rel %.3e  P rel %.3e (1e-12)\n", names[i], count[i],
                worst_x[i], worst_P[i]);
    pass = pass && worst_x[i] <= 1e-12 && worst_P[i] <= 1e-12 && count[i] > 100;
  }
  EXPECT_TRUE(pass);
}

// ── Bit-exact promises ───────────────────────────────────────────────────────

/// A state built only by operations, with slots 0 and 2 active.
void Prepared(ContactInekf& f) {
  StartFilter(f, 2, 3);
  ASSERT_EQ(f.RemoveContact(1), InekfStatus::kOk);
  Truthish t(5);
  for (int k = 0; k < 50; ++k) {
    ASSERT_EQ(f.Propagate(Vec3(0.3, -0.2, 0.5), Vec3(0.2, 0.1, kG0), kDt), InekfStatus::kOk);
    std::vector<ContactUpdateReport> rep(4);
    ASSERT_EQ(f.UpdateContacts(NearPrediction(f, t, {true, true, true, true}), rep),
              InekfStatus::kOk);
  }
}

bool OutsideSlotBitEqual(const Cov& a, const Cov& b, int slot) {
  const int o = ContactInekf::SlotOffset(slot);
  for (Eigen::Index c = 0; c < a.cols(); ++c) {
    for (Eigen::Index r = 0; r < a.rows(); ++r) {
      const bool in = (r >= o && r < o + 3) || (c >= o && c < o + 3);
      if (!in && std::memcmp(&a(r, c), &b(r, c), sizeof(double)) != 0) {
        return false;
      }
    }
  }
  return true;
}

TEST(ContactInekfSlots, InsertThenRemoveRestoresEveryBit) {
  ContactInekf f;
  Prepared(f);
  const Cov& P = f.covariance();
  ASSERT_TRUE(P == P.transpose());  // premise: P built by operations is bit symmetric
  const Snapshot before = Capture(f);
  ASSERT_EQ(f.InsertContact(1, Kin(FootPoint(1))), InekfStatus::kOk);
  ASSERT_EQ(f.RemoveContact(1), InekfStatus::kOk);
  EXPECT_TRUE(BitEqual(Capture(f), before));
}

TEST(ContactInekfSlots, InsertTouchesOnlyItsRowsAndColumns) {
  ContactInekf f;
  Prepared(f);
  const Cov P0 = f.covariance();
  const Mat3 R0 = f.rotation();
  const Vec3 v0 = f.velocity();
  const Vec3 p0 = f.position();
  const Vec3 d0 = f.contact_position(0);
  const Vec3 d2 = f.contact_position(2);
  const Vec3 bg0 = f.gyro_bias();
  const Vec3 ba0 = f.accel_bias();
  ASSERT_EQ(f.InsertContact(3, Kin(FootPoint(3))), InekfStatus::kOk);
  EXPECT_TRUE(OutsideSlotBitEqual(P0, f.covariance(), 3));
  EXPECT_TRUE(f.rotation() == R0 && f.velocity() == v0 && f.position() == p0);
  EXPECT_TRUE(f.contact_position(0) == d0 && f.contact_position(2) == d2);
  EXPECT_TRUE(f.gyro_bias() == bg0 && f.accel_bias() == ba0);
  EXPECT_TRUE(f.contact_active(3));
}

TEST(ContactInekfSlots, RemoveZeroesItsRowsAndKeepsTheRest) {
  ContactInekf f;
  Prepared(f);
  const Cov P0 = f.covariance();
  const Vec3 d0 = f.contact_position(0);
  ASSERT_EQ(f.SetContactNoiseScale(2, 3.0), InekfStatus::kOk);
  ASSERT_EQ(f.RemoveContact(2), InekfStatus::kOk);
  const Cov& P = f.covariance();
  const int o = ContactInekf::SlotOffset(2);
  bool plus_zero = true;
  for (Eigen::Index i = 0; i < P.rows(); ++i) {
    for (int k = 0; k < 3; ++k) {
      for (double x : {P(o + k, i), P(i, o + k)}) {
        plus_zero = plus_zero && x == 0.0 && !std::signbit(x);
      }
    }
  }
  EXPECT_TRUE(plus_zero);
  EXPECT_TRUE(OutsideSlotBitEqual(P0, P, 2));
  EXPECT_TRUE(f.contact_position(2) == Vec3::Zero());
  EXPECT_FALSE(std::signbit(f.contact_position(2).x()));
  EXPECT_FALSE(f.contact_active(2));
  EXPECT_EQ(f.contact_noise_scale(2), 1.0);
  EXPECT_TRUE(f.contact_position(0) == d0);
}

TEST(ContactInekfSlots, RefusalsLeaveTheStateUntouched) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  {
    ContactInekf f;
    f.Init(FixtureConfig(), 4);
    EXPECT_EQ(f.InsertContact(0, Kin(FootPoint(0))), InekfStatus::kNotInitialized);
  }
  ContactInekf f;
  Prepared(f);  // slots 0, 2 active
  const Snapshot before = Capture(f);

  struct Case {
    int slot;
    ContactKinematics k;
    InekfStatus want;
  };

  Eigen::MatrixXd Jbad = Eigen::MatrixXd::Zero(3, 2);
  Jbad(1, 1) = nan;
  const std::vector<Case> insert{
      {0, Kin(FootPoint(0)), InekfStatus::kSlotActive},
      {-1, Kin(FootPoint(0)), InekfStatus::kBadSlot},
      {4, Kin(FootPoint(0)), InekfStatus::kBadSlot},
      {1, Kin(Vec3(nan, 0, 0)), InekfStatus::kNonFiniteInput},
      {1, Kin(FootPoint(1), true, Jbad), InekfStatus::kNonFiniteInput},
      // σ_q² J Jᵀ overflows.
      {1, Kin(FootPoint(1), true, Eigen::MatrixXd::Constant(3, 2, 1e200)),
       InekfStatus::kNonFiniteResult},
  };
  for (const Case& c : insert) {
    EXPECT_EQ(f.InsertContact(c.slot, c.k), c.want) << c.slot;
    EXPECT_TRUE(BitEqual(Capture(f), before)) << c.slot;
  }
  EXPECT_EQ(f.RemoveContact(1), InekfStatus::kSlotInactive);
  EXPECT_EQ(f.RemoveContact(-1), InekfStatus::kBadSlot);
  EXPECT_EQ(f.RemoveContact(4), InekfStatus::kBadSlot);
  EXPECT_TRUE(BitEqual(Capture(f), before));

  // The anchor p + R r itself overflows (p near the double range).
  {
    ContactInekf g = f;
    std::array<Vec3, 4> d = ZeroContacts();
    std::array<bool, 4> on{};
    for (int i : {0, 2}) {
      on[static_cast<std::size_t>(i)] = true;
      d[static_cast<std::size_t>(i)] = f.contact_position(i);
    }
    Access::SetState(g, f.rotation(), f.velocity(), Vec3(1.5e308, 0, 0), d, on, f.gyro_bias(),
                     f.accel_bias(), f.covariance());
    const Snapshot gb = Capture(g);
    const Vec3 r = f.rotation().transpose() * Vec3(1.5e308, 0, 0);
    EXPECT_EQ(g.InsertContact(1, Kin(r)), InekfStatus::kNonFiniteResult);
    EXPECT_TRUE(BitEqual(Capture(g), gb));
  }
  // A non-PD covariance makes the inserted block fail its Cholesky test.
  Cov P = f.covariance();
  P(3, 3) = -1.0;
  Access::SetCovariance(f, P);
  const Snapshot bad = Capture(f);
  EXPECT_EQ(f.InsertContact(1, Kin(FootPoint(1))), InekfStatus::kNotPositiveDefinite);
  EXPECT_TRUE(BitEqual(Capture(f), bad));
}

// ── Determinism over mixed sequences ─────────────────────────────────────────

void MixedRun(ContactInekf& f, std::uint64_t seed, int ticks) {
  Truthish t(seed);
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int k = 0; k < ticks; ++k) {
    ASSERT_EQ(f.Propagate(Vec3(2.0 * ud(t.rng), ud(t.rng), ud(t.rng)),
                          f.rotation().transpose() * Vec3(0, 0, kG0) + t.Randn(), kDt),
              InekfStatus::kOk);
    if (k % 97 == 13) {
      const int j = (k / 97) % 4;
      if (f.contact_active(j)) {
        ASSERT_EQ(f.RemoveContact(j), InekfStatus::kOk);
      } else {
        ASSERT_EQ(f.InsertContact(j, Kin(FootPoint(j))), InekfStatus::kOk);
      }
    }
    std::vector<ContactUpdateReport> rep(4);
    ASSERT_EQ(f.UpdateContacts(NearPrediction(f, t, {true, true, true, true}), rep),
              InekfStatus::kOk);
  }
}

TEST(ContactInekfSlots, MixedSequencesAreDeterministic) {
  ContactInekf a;
  ContactInekf b;
  StartFilter(a, 3, 2);
  StartFilter(b, 3, 2);
  MixedRun(a, 21, 5000);
  MixedRun(b, 21, 5000);
  EXPECT_TRUE(BitEqual(Capture(a), Capture(b)));
  const Snapshot first = Capture(a);
  a.Reset();
  StartFilter(a, 3, 2);
  MixedRun(a, 21, 5000);
  EXPECT_TRUE(BitEqual(Capture(a), first));
}

}  // namespace
}  // namespace rtc::estimation::testing
