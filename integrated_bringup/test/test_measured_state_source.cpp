// MeasuredStateSource seam: the device layout map, the `estimator` section and
// the truth source's copy and verdict. No model — a synthetic joint order whose
// device rosters are permutations of it, so a copy that ignores the map shows.
#include "integrated_bringup/controllers/locomotion/measured_state_source.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using integrated_bringup::locomotion::BuildDeviceIndices;
using integrated_bringup::locomotion::DeviceIndices;
using integrated_bringup::locomotion::EstimatorConfig;
using integrated_bringup::locomotion::ExternalWrenchKind;
using integrated_bringup::locomotion::HandBit;
using integrated_bringup::locomotion::HandRoster;
using integrated_bringup::locomotion::kBaseBit;
using integrated_bringup::locomotion::kBodyJointsBit;
using integrated_bringup::locomotion::MeasuredRobotState;
using integrated_bringup::locomotion::ParseEstimatorConfig;
using integrated_bringup::locomotion::StateSourceKind;
using integrated_bringup::locomotion::StateSourceStatus;
using integrated_bringup::locomotion::TruthStateSource;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kNow = 10'000 * kMs;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Model order: body joints interleaved with the hand's, as a hand mounted
// mid-tree would put them.
const std::vector<std::string> kJointOrder = {"b0", "b1", "h0", "h1", "b2"};
const std::vector<std::string> kBodyRoster = {"b2", "b0", "b1"};  // a permutation
const std::vector<std::string> kHandRoster = {"h1", "h0"};
constexpr int kBodyDevice = 0;
constexpr int kHandDevice = 1;

DeviceIndices MakeIndices() {
  return BuildDeviceIndices(kJointOrder, kBodyDevice, kBodyRoster,
                            {HandRoster{kHandDevice, kHandRoster}}, "test");
}

TruthStateSource::Params MakeParams() {
  TruthStateSource::Params p;
  p.body_max_age_ns = 10 * kMs;
  p.hand_max_age_ns = 50 * kMs;
  p.num_contacts = 2;
  p.model_epoch = 7;
  return p;
}

// Body channel c carries 0.1·(c+1) rad and 1.0·(c+1) rad/s; hand channel c
// 2.0 + 0.1·c and 3.0 + c. Base pose p = (1, 2, 3), q = a unit quaternion.
rtc::ControllerState MakeState() {
  rtc::ControllerState s{};
  s.num_devices = 2;
  auto& body = s.devices[kBodyDevice];
  body.valid = true;
  body.num_channels = 3;
  for (std::size_t c = 0; c < 3; ++c) {
    body.positions[c] = 0.1 * static_cast<double>(c + 1);
    body.velocities[c] = 1.0 * static_cast<double>(c + 1);
  }
  body.joint_recv_steady_ns = kNow - 2 * kMs;
  body.joint_sequence = 41;
  body.base_pose = {1.0, 2.0, 3.0, 0.0, 0.0, std::sin(0.25), std::cos(0.25)};
  body.base_twist = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  body.base_recv_steady_ns = kNow - 1 * kMs;
  body.base_valid = true;
  auto& hand = s.devices[kHandDevice];
  hand.valid = true;
  hand.num_channels = 2;
  for (std::size_t c = 0; c < 2; ++c) {
    hand.positions[c] = 2.0 + 0.1 * static_cast<double>(c);
    hand.velocities[c] = 3.0 + static_cast<double>(c);
  }
  hand.joint_recv_steady_ns = kNow - 5 * kMs;
  return s;
}

// q / v slot of a model joint name.
std::size_t Joint(const std::string& name) {
  for (std::size_t a = 0; a < kJointOrder.size(); ++a) {
    if (kJointOrder[a] == name) {
      return a;
    }
  }
  ADD_FAILURE() << name;
  return 0;
}

// ── BuildDeviceIndices ──────────────────────────────────────────────────────

TEST(BuildDeviceIndices, MapsEveryChannelToItsModelJoint) {
  const auto idx = MakeIndices();
  EXPECT_EQ(5, idx.num_joints);
  EXPECT_EQ(kBodyDevice, idx.body.device);
  EXPECT_EQ(3, idx.body.num_channels);
  EXPECT_EQ(4, idx.body.joint[0]);  // b2
  EXPECT_EQ(0, idx.body.joint[1]);  // b0
  EXPECT_EQ(1, idx.body.joint[2]);  // b1
  ASSERT_EQ(1, idx.num_hands);
  EXPECT_EQ(kHandDevice, idx.hands[0].device);
  EXPECT_EQ(3, idx.hands[0].joint[0]);  // h1
  EXPECT_EQ(2, idx.hands[0].joint[1]);  // h0
}

TEST(BuildDeviceIndices, RefusesALayoutThatDoesNotCoverTheModelExactlyOnce) {
  const auto refused = [](const std::vector<std::string>& order, int body_dev,
                          const std::vector<std::string>& body,
                          const std::vector<HandRoster>& hands) {
    EXPECT_THROW(static_cast<void>(BuildDeviceIndices(order, body_dev, body, hands, "test")),
                 std::invalid_argument);
  };
  const HandRoster hand{kHandDevice, kHandRoster};
  refused(kJointOrder, kBodyDevice, {"b2", "b0", "bX"}, {hand});                 // not in the model
  refused(kJointOrder, kBodyDevice, {"b2", "b0", "b1", "h0"}, {hand});           // on two rosters
  refused(kJointOrder, kBodyDevice, {"b2", "b0", "b0"}, {hand});                 // twice on one
  refused(kJointOrder, kBodyDevice, {"b2", "b0"}, {hand});                       // b1 uncovered
  refused(kJointOrder, kBodyDevice, kBodyRoster, {});                            // hand uncovered
  refused(kJointOrder, kBodyDevice, kBodyRoster, {{kBodyDevice, kHandRoster}});  // slot twice
  refused(kJointOrder, -1, kBodyRoster, {hand});  // slot out of range
  refused(kJointOrder, rtc::ControllerState::kMaxDevices, kBodyRoster, {hand});
  refused(kJointOrder, kBodyDevice, {}, {hand});   // empty roster
  refused({"b0", "b0"}, kBodyDevice, {"b0"}, {});  // model twice
  refused({}, kBodyDevice, {"b0"}, {});            // empty model
  std::vector<std::string> wide(integrated_bringup::locomotion::kMaxStateJoints + 1, "j");
  for (std::size_t i = 0; i < wide.size(); ++i) {
    wide[i] += std::to_string(i);
  }
  refused(wide, kBodyDevice, wide, {});  // past capacity
  // The passing twin of the refusals above.
  EXPECT_NO_THROW(
      static_cast<void>(BuildDeviceIndices(kJointOrder, kBodyDevice, kBodyRoster, {hand}, "t")));
}

// ── ParseEstimatorConfig ───────────────────────────────────────────────────

TEST(ParseEstimatorConfig, ReadsTheTruthSection) {
  const EstimatorConfig cfg = ParseEstimatorConfig(
      YAML::Load("{source: truth, body_max_age_ms: 10.0, hand_max_age_ms: 50}"), "estimator");
  EXPECT_EQ("truth", cfg.source);
  EXPECT_DOUBLE_EQ(10.0, cfg.body_max_age_ms);
  EXPECT_DOUBLE_EQ(50.0, cfg.hand_max_age_ms);
}

TEST(ParseEstimatorConfig, RefusesEveryMalformedSection) {
  for (const char* bad : {
           "{source: imu, body_max_age_ms: 10, hand_max_age_ms: 50}",
           "{source: truth, body_max_age_ms: 10}",
           "{source: truth, body_max_age_ms: 10, hand_max_age_ms: 50, extra: 1}",
           "{source: truth, body_max_age_ms: 0, hand_max_age_ms: 50}",
           "{source: truth, body_max_age_ms: -1, hand_max_age_ms: 50}",
           "{source: truth, body_max_age_ms: 1001, hand_max_age_ms: 50}",
           "{source: truth, body_max_age_ms: .nan, hand_max_age_ms: 50}",
           "{source: truth, body_max_age_ms: ten, hand_max_age_ms: 50}",
           "[truth]",
       }) {
    EXPECT_THROW(static_cast<void>(ParseEstimatorConfig(YAML::Load(bad), "estimator")),
                 std::invalid_argument)
        << bad;
  }
  EXPECT_THROW(static_cast<void>(ParseEstimatorConfig(YAML::Node{}, "estimator")),
               std::invalid_argument);
}

// ── TruthStateSource ───────────────────────────────────────────────────────

TEST(TruthStateSource, RefusesBadParams) {
  auto p = MakeParams();
  p.body_max_age_ns = 0;
  EXPECT_THROW(TruthStateSource{p}, std::invalid_argument);
  p = MakeParams();
  p.hand_max_age_ns = -1;
  EXPECT_THROW(TruthStateSource{p}, std::invalid_argument);
  p = MakeParams();
  p.num_contacts = integrated_bringup::locomotion::kMaxStateContacts + 1;
  EXPECT_THROW(TruthStateSource{p}, std::invalid_argument);
}

TEST(TruthStateSource, CopiesTheFullStateThroughTheLayoutMap) {
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  const auto state = MakeState();
  MeasuredRobotState out{};

  ASSERT_TRUE(src.Update(state, idx, kNow, out));

  EXPECT_EQ(StateSourceStatus::kOk, out.status);
  EXPECT_EQ(StateSourceKind::kTruth, out.kind);
  EXPECT_EQ(12, out.nq);
  EXPECT_EQ(11, out.nv);
  const auto& base = state.devices[kBodyDevice].base_pose;
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(base[i], out.q[i]) << i;  // a unit quaternion passes unchanged
  }
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(state.devices[kBodyDevice].base_twist[i], out.v[i]) << i;
  }
  // Body roster {b2, b0, b1}, hand roster {h1, h0}: bit-exact copies.
  const auto& body = state.devices[kBodyDevice];
  const auto& hand = state.devices[kHandDevice];
  EXPECT_EQ(body.positions[0], out.q[7 + Joint("b2")]);
  EXPECT_EQ(body.positions[1], out.q[7 + Joint("b0")]);
  EXPECT_EQ(body.positions[2], out.q[7 + Joint("b1")]);
  EXPECT_EQ(body.velocities[0], out.v[6 + Joint("b2")]);
  EXPECT_EQ(body.velocities[2], out.v[6 + Joint("b1")]);
  EXPECT_EQ(hand.positions[0], out.q[7 + Joint("h1")]);
  EXPECT_EQ(hand.positions[1], out.q[7 + Joint("h0")]);
  EXPECT_EQ(hand.velocities[1], out.v[6 + Joint("h0")]);

  const uint32_t all = kBodyJointsBit | kBaseBit | HandBit(0);
  EXPECT_EQ(all, out.fresh_mask);
  EXPECT_EQ(all, out.valid_mask);
  EXPECT_EQ(0b11U, out.contact_mask);  // stance prior: both contacts closed
  for (const auto& w : out.ee_wrench) {
    EXPECT_EQ(ExternalWrenchKind::kUnknown, w.kind);
    for (double f : w.wrench) {
      EXPECT_EQ(0.0, f);
    }
  }
  EXPECT_EQ(7U, out.model_epoch);
  EXPECT_EQ(41U, out.sequence);
  EXPECT_EQ(body.joint_recv_steady_ns, out.stamp_steady_ns);  // the older of the two
}

TEST(TruthStateSource, NormalisesANearUnitQuaternion) {
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  auto state = MakeState();
  auto& p = state.devices[kBodyDevice].base_pose;
  for (std::size_t i = 3; i < 7; ++i) {
    p[i] *= 1.0 + 5e-4;  // inside the tolerance
  }
  MeasuredRobotState out{};
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  const double norm = std::sqrt(out.q[3] * out.q[3] + out.q[4] * out.q[4] + out.q[5] * out.q[5] +
                                out.q[6] * out.q[6]);
  EXPECT_NEAR(1.0, norm, 4 * std::numeric_limits<double>::epsilon());
}

// Each body / base fault, alone, with the status it must report.
TEST(TruthStateSource, ReportsTheFirstBodyOrBaseFault) {
  const auto idx = MakeIndices();

  struct Case {
    const char* what;
    void (*spoil)(rtc::ControllerState&);
    StateSourceStatus expected;
  };

  const Case cases[] = {
      {"device not reported", [](rtc::ControllerState& s) { s.devices[kBodyDevice].valid = false; },
       StateSourceStatus::kBodyUnreadable},
      {"too few channels", [](rtc::ControllerState& s) { s.devices[kBodyDevice].num_channels = 2; },
       StateSourceStatus::kBodyUnreadable},
      {"position hole", [](rtc::ControllerState& s) { s.devices[kBodyDevice].hole_mask = 0b100; },
       StateSourceStatus::kBodyUnreadable},
      {"velocity hole",
       [](rtc::ControllerState& s) { s.devices[kBodyDevice].velocity_hole_mask = 0b1; },
       StateSourceStatus::kBodyUnreadable},
      {"body group absent", [](rtc::ControllerState& s) { s.num_devices = 0; },
       StateSourceStatus::kBodyUnreadable},
      {"joints stale",
       [](rtc::ControllerState& s) {
         s.devices[kBodyDevice].joint_recv_steady_ns = kNow - 11 * kMs;
       },
       StateSourceStatus::kBodyStale},
      {"joints never received",
       [](rtc::ControllerState& s) { s.devices[kBodyDevice].joint_recv_steady_ns = 0; },
       StateSourceStatus::kBodyStale},
      {"base flag down", [](rtc::ControllerState& s) { s.devices[kBodyDevice].base_valid = false; },
       StateSourceStatus::kBaseMissing},
      {"base never received",
       [](rtc::ControllerState& s) { s.devices[kBodyDevice].base_recv_steady_ns = 0; },
       StateSourceStatus::kBaseMissing},
      {"base stale",
       [](rtc::ControllerState& s) {
         s.devices[kBodyDevice].base_recv_steady_ns = kNow - 11 * kMs;
       },
       StateSourceStatus::kBaseStale},
      {"zero quaternion",
       [](rtc::ControllerState& s) {
         auto& p = s.devices[kBodyDevice].base_pose;
         p[3] = p[4] = p[5] = p[6] = 0.0;
       },
       StateSourceStatus::kBaseInvalid},
      {"quaternion 1 % off unit",
       [](rtc::ControllerState& s) {
         auto& p = s.devices[kBodyDevice].base_pose;
         for (std::size_t i = 3; i < 7; ++i) {
           p[i] *= 1.01;
         }
       },
       StateSourceStatus::kBaseInvalid},
      {"NaN quaternion",
       [](rtc::ControllerState& s) { s.devices[kBodyDevice].base_pose[6] = kNaN; },
       StateSourceStatus::kBaseInvalid},
      {"NaN joint position",
       [](rtc::ControllerState& s) { s.devices[kBodyDevice].positions[1] = kNaN; },
       StateSourceStatus::kNonFinite},
      {"NaN base twist",
       [](rtc::ControllerState& s) { s.devices[kBodyDevice].base_twist[4] = kNaN; },
       StateSourceStatus::kNonFinite},
      {"NaN base position",
       [](rtc::ControllerState& s) { s.devices[kBodyDevice].base_pose[0] = kNaN; },
       StateSourceStatus::kNonFinite},
  };
  for (const Case& c : cases) {
    TruthStateSource src(MakeParams());
    auto state = MakeState();
    c.spoil(state);
    MeasuredRobotState out{};
    EXPECT_FALSE(src.Update(state, idx, kNow, out)) << c.what;
    EXPECT_FALSE(out.valid) << c.what;
    EXPECT_EQ(c.expected, out.status) << c.what << ": " << ToString(out.status);
  }
}

TEST(TruthStateSource, AFailedLaneReadsNeutralAndClaimsNoSample) {
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  auto state = MakeState();
  state.devices[kBodyDevice].positions[1] = kNaN;  // body joints non-finite
  state.devices[kBodyDevice].base_valid = false;   // base missing
  MeasuredRobotState out{};
  out.q.fill(5.0);  // whatever the caller's buffer held
  out.v.fill(5.0);

  EXPECT_FALSE(src.Update(state, idx, kNow, out));
  EXPECT_EQ(StateSourceStatus::kBaseMissing, out.status);  // precedes non-finite
  EXPECT_EQ(0U, out.valid_mask & (kBodyJointsBit | kBaseBit));
  EXPECT_EQ(0U, out.fresh_mask & (kBodyJointsBit | kBaseBit));
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(0.0, out.v[i]) << i;
  }
  EXPECT_EQ(0.0, out.q[0]);
  EXPECT_EQ(1.0, out.q[6]);  // identity, not the stale 5.0
  EXPECT_EQ(state.devices[kBodyDevice].joint_recv_steady_ns, out.stamp_steady_ns);
}

TEST(TruthStateSource, AReceiptStampedAfterNowIsFresh) {
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  auto state = MakeState();
  state.devices[kBodyDevice].joint_recv_steady_ns = kNow + 3000;
  state.devices[kBodyDevice].base_recv_steady_ns = kNow + 5000;
  state.devices[kHandDevice].joint_recv_steady_ns = kNow + 1000;
  MeasuredRobotState out{};
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  EXPECT_EQ(kBodyJointsBit | kBaseBit | HandBit(0), out.fresh_mask);
}

TEST(TruthStateSource, AStaleHandHoldsItsJointsAtRestWithoutFailingTheState) {
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  auto state = MakeState();
  MeasuredRobotState out{};
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  const double held = out.q[7 + Joint("h1")];

  // The hand goes quiet (past its 50 ms) and its last message moves on.
  state.devices[kHandDevice].joint_recv_steady_ns = kNow - 51 * kMs;
  state.devices[kHandDevice].positions[0] = 9.0;
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  EXPECT_EQ(StateSourceStatus::kOk, out.status);
  EXPECT_EQ(held, out.q[7 + Joint("h1")]);
  EXPECT_EQ(0.0, out.v[6 + Joint("h1")]);
  EXPECT_EQ(0.0, out.v[6 + Joint("h0")]);
  EXPECT_EQ(0U, out.fresh_mask & HandBit(0));
  EXPECT_NE(0U, out.valid_mask & HandBit(0));  // held, so still a real sample

  // A NaN sample is a stale hand too, never a failed state.
  state.devices[kHandDevice].joint_recv_steady_ns = kNow - 1 * kMs;
  state.devices[kHandDevice].velocities[1] = kNaN;
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  EXPECT_EQ(held, out.q[7 + Joint("h1")]);
  EXPECT_EQ(0U, out.fresh_mask & HandBit(0));
}

TEST(TruthStateSource, AHandThatNeverReportedIsAtZeroAndNotValid) {
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  auto state = MakeState();
  state.devices[kHandDevice].valid = false;
  MeasuredRobotState out{};
  out.q.fill(5.0);  // whatever the caller's buffer held
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  EXPECT_EQ(0.0, out.q[7 + Joint("h0")]);
  EXPECT_EQ(0.0, out.q[7 + Joint("h1")]);
  EXPECT_EQ(0U, out.valid_mask & HandBit(0));
}

TEST(TruthStateSource, ResetBumpsTheEpochAndForgetsTheHeldHand) {
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  auto state = MakeState();
  MeasuredRobotState out{};
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  const uint32_t epoch = out.state_epoch;

  src.Reset();
  state.devices[kHandDevice].valid = false;
  ASSERT_TRUE(src.Update(state, idx, kNow, out));
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(0.0, out.q[7 + Joint("h1")]);
  EXPECT_EQ(0U, out.valid_mask & HandBit(0));
}

TEST(TruthStateSource, NoContactsMeansAnEmptyPrior) {
  auto p = MakeParams();
  p.num_contacts = 0;
  TruthStateSource src(p);
  MeasuredRobotState out{};
  ASSERT_TRUE(src.Update(MakeState(), MakeIndices(), kNow, out));
  EXPECT_EQ(0U, out.contact_mask);
}

// ── Output digest: the truth source's whole verdict over a random run ────────
//
// The digest of every output field over 1000 ticks of random lanes (finite,
// NaN, stale, missing, an off-unit quaternion, stale and NaN hands, a Reset
// now and then). Recorded on the implementation before the source interface
// grew a second implementation; any later change to what truth writes moves it.

uint64_t Fnv(uint64_t h, const void* data, std::size_t n) {
  const auto* b = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h = (h ^ b[i]) * 1099511628211ULL;
  }
  return h;
}

template <typename T>
uint64_t FnvValue(uint64_t h, const T& v) {
  return Fnv(h, &v, sizeof(T));
}

double Uniform(std::mt19937_64& rng) {  // [0, 1), the same on every library
  return static_cast<double>(rng() >> 11U) * 0x1.0p-53;
}

uint64_t TruthDigest(int ticks) {
  std::mt19937_64 rng(20261119U);
  const auto idx = MakeIndices();
  TruthStateSource src(MakeParams());
  MeasuredRobotState out{};
  uint64_t h = 1469598103934665603ULL;
  for (int t = 0; t < ticks; ++t) {
    auto state = MakeState();
    auto& body = state.devices[kBodyDevice];
    auto& hand = state.devices[kHandDevice];
    for (std::size_t c = 0; c < 3; ++c) {
      body.positions[c] = 4.0 * Uniform(rng) - 2.0;
      body.velocities[c] = 10.0 * Uniform(rng) - 5.0;
    }
    for (std::size_t c = 0; c < 2; ++c) {
      hand.positions[c] = 2.0 * Uniform(rng) - 1.0;
      hand.velocities[c] = 2.0 * Uniform(rng) - 1.0;
    }
    for (std::size_t i = 0; i < 3; ++i) {
      body.base_pose[i] = 2.0 * Uniform(rng) - 1.0;
    }
    double qn = 0.0;
    for (std::size_t i = 3; i < 7; ++i) {
      body.base_pose[i] = 2.0 * Uniform(rng) - 1.0;
      qn += body.base_pose[i] * body.base_pose[i];
    }
    for (std::size_t i = 3; i < 7; ++i) {
      body.base_pose[i] /= std::sqrt(qn);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      body.base_twist[i] = 2.0 * Uniform(rng) - 1.0;
    }
    body.joint_recv_steady_ns = kNow - static_cast<int64_t>(Uniform(rng) * 12.0 * kMs);
    body.base_recv_steady_ns = kNow - static_cast<int64_t>(Uniform(rng) * 12.0 * kMs);
    hand.joint_recv_steady_ns = kNow - static_cast<int64_t>(Uniform(rng) * 60.0 * kMs);
    body.joint_sequence = rng();
    const uint64_t faults = rng();
    if ((faults & 0xFU) == 0) {
      body.positions[faults >> 8U & 1U] = kNaN;
    }
    if ((faults >> 4U & 0xFU) == 0) {
      body.base_twist[faults >> 9U & 3U] = kNaN;
    }
    if ((faults >> 12U & 0xFU) == 0) {
      body.base_pose[6] *= 1.01;  // off unit by more than the tolerance
    }
    if ((faults >> 16U & 0xFU) == 0) {
      body.base_valid = false;
    }
    if ((faults >> 20U & 0x1FU) == 0) {
      body.valid = false;
    }
    if ((faults >> 25U & 0x7U) == 0) {
      hand.velocities[0] = kNaN;
    }
    if ((faults >> 28U & 0x7U) == 0) {
      hand.valid = false;
    }
    if ((faults >> 31U & 0x3FU) == 0) {
      src.Reset();
    }
    const bool ok = src.Update(state, idx, kNow, out);
    h = FnvValue(h, ok);
    h = FnvValue(h, out.nq);
    h = FnvValue(h, out.nv);
    h = Fnv(h, out.q.data(), static_cast<std::size_t>(out.nq) * sizeof(double));
    h = Fnv(h, out.v.data(), static_cast<std::size_t>(out.nv) * sizeof(double));
    h = FnvValue(h, out.contact_mask);
    h = FnvValue(h, out.stamp_steady_ns);
    h = FnvValue(h, out.sequence);
    h = FnvValue(h, out.model_epoch);
    h = FnvValue(h, out.state_epoch);
    h = FnvValue(h, out.fresh_mask);
    h = FnvValue(h, out.valid_mask);
    h = FnvValue(h, out.kind);
    h = FnvValue(h, out.status);
    h = FnvValue(h, out.valid);
    for (const auto& w : out.ee_wrench) {
      h = FnvValue(h, w.kind);
      h = Fnv(h, w.wrench.data(), sizeof(double) * w.wrench.size());
      h = FnvValue(h, w.stamp_steady_ns);
    }
  }
  return h;
}

TEST(TruthStateSource, OutputDigestOverARandomRunIsUnchanged) {
  constexpr uint64_t kRecorded = 0xf2368b39a6ee6ffeULL;
  const uint64_t digest = TruthDigest(1000);
  EXPECT_EQ(kRecorded, digest) << std::hex << "digest 0x" << digest;
}

}  // namespace
