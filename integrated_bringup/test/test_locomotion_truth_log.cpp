// FillLocomotionTruthLog — the T-tier row from the body group's lanes:
// freshness, the raw base copy, the floor-on-foot sign flip, the moment only
// on a declared lane, the CoP and its threshold, and the lane fit.
#include "csv_log_fixture.hpp"
#include "integrated_bringup/controllers/locomotion/truth_log.hpp"
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"
#include "rtc_base/testing/isolated_session_dir.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

namespace loco = integrated_bringup::locomotion;
using integrated_bringup::LocomotionTruthLogPod;

constexpr std::int64_t kNow = 5'000'000'000;
constexpr std::int64_t kMaxAge = 10'000'000;  // 10 ms
constexpr int kStride = 7;
constexpr double kCopMin = 10.0;

bool SameBits(double a, double b) {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

loco::TruthLogInputs Inputs(bool moment = true) {
  loco::TruthLogInputs in;
  in.now_steady_ns = kNow;
  in.max_age_ns = kMaxAge;
  in.contacts = 2;
  in.stride = kStride;
  in.moment = moment;
  in.cop_min_fz = kCopMin;
  return in;
}

// A body lane with a fresh base and both contact groups fresh.
rtc::DeviceState Lanes() {
  rtc::DeviceState d{};
  // Not a unit quaternion (|q| − 1 ≈ 1e-4): the copy must stay raw.
  d.base_pose = {0.1, -0.2, 0.75, 0.01, 0.02, 0.03, 0.9997503};
  d.base_twist = {0.3, -0.1, 0.05, 0.01, -0.02, 0.4};
  d.base_valid = true;
  d.base_recv_steady_ns = kNow - 1'000'000;
  d.base_sequence = 42;
  d.num_inference_groups = 2;
  for (int c = 0; c < 2; ++c) {
    const auto u = static_cast<std::size_t>(c);
    d.inference_enable[u] = true;
    d.inference_recv_steady_ns[u] = kNow - 2'000'000;
    d.inference_sequence[u] = 100 + u;
  }
  return d;
}

// Group c's lane values (foot-on-floor, as the backend writes them).
void SetGroup(rtc::DeviceState& d, int c, const std::array<float, 6>& foot_on_floor) {
  for (int k = 0; k < 6; ++k) {
    d.inference_data[static_cast<std::size_t>(c * kStride + 1 + k)] =
        foot_on_floor[static_cast<std::size_t>(k)];
  }
}

TEST(LocomotionTruthLog, TheBaseIsTheRawLane) {
  const rtc::DeviceState d = Lanes();
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, Inputs(), pod);
  EXPECT_TRUE(pod.truth_mask & LocomotionTruthLogPod::kBaseBit);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(SameBits(pod.base_p[i], d.base_pose[i])) << i;
    EXPECT_TRUE(SameBits(pod.base_v[i], d.base_twist[i])) << i;
    EXPECT_TRUE(SameBits(pod.base_w[i], d.base_twist[3 + i])) << i;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(SameBits(pod.base_q_xyzw[i], d.base_pose[3 + i])) << i;
  }
  EXPECT_EQ(pod.base_seq, 42U);
  EXPECT_DOUBLE_EQ(pod.base_age_us, 1000.0);
}

TEST(LocomotionTruthLog, AStaleOrRefusedOrMissingBaseIsNaNWithItsBitOff) {
  struct Case {
    const char* label;
    void (*spoil)(rtc::DeviceState&);
    double age_us;
  };

  const Case cases[] = {
      {"older than the bound",
       [](rtc::DeviceState& d) { d.base_recv_steady_ns = kNow - kMaxAge - 1; },
       (kMaxAge + 1) * 1e-3},
      {"producer verdict false", [](rtc::DeviceState& d) { d.base_valid = false; }, 1000.0},
      {"never received", [](rtc::DeviceState& d) { d.base_recv_steady_ns = 0; }, -1.0},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.label);
    rtc::DeviceState d = Lanes();
    c.spoil(d);
    LocomotionTruthLogPod pod{};
    loco::FillLocomotionTruthLog(d, Inputs(), pod);
    EXPECT_FALSE(pod.truth_mask & LocomotionTruthLogPod::kBaseBit);
    EXPECT_TRUE(std::isnan(pod.base_p[0]) && std::isnan(pod.base_q_xyzw[3]) &&
                std::isnan(pod.base_v[2]) && std::isnan(pod.base_w[1]));
    EXPECT_DOUBLE_EQ(pod.base_age_us, c.age_us);
    EXPECT_EQ(pod.base_seq, 42U);
  }
}

// The lane is foot-on-floor; the columns are floor-on-foot — exactly negated.
TEST(LocomotionTruthLog, TheWrenchIsTheNegatedLane) {
  rtc::DeviceState d = Lanes();
  SetGroup(d, 0, {1.25F, -3.5F, -401.0F, 0.123456789F, -7.0F, 0.0625F});
  SetGroup(d, 1, {-2.0F, 0.5F, -55.0F, 1.0F, 2.0F, -3.0F});
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, Inputs(), pod);
  EXPECT_EQ(pod.truth_mask, 0b111U);
  for (std::size_t c = 0; c < 2; ++c) {
    for (std::size_t k = 0; k < 6; ++k) {
      const double lane = static_cast<double>(d.inference_data[c * kStride + 1 + k]);
      EXPECT_TRUE(SameBits(pod.grf[c * 6 + k], -lane)) << c << ' ' << k;
    }
    EXPECT_EQ(pod.contact_seq[c], 100 + c);
    EXPECT_DOUBLE_EQ(pod.contact_age_us[c], 2000.0);
  }
}

TEST(LocomotionTruthLog, WithoutTheMomentFlagTheMomentAndCopAreNaN) {
  rtc::DeviceState d = Lanes();
  SetGroup(d, 0, {0.0F, 0.0F, -400.0F, 12.5F, 25.0F, 0.0F});
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, Inputs(/*moment=*/false), pod);
  EXPECT_EQ(pod.truth_mask & 0b10U, 0b10U);
  EXPECT_DOUBLE_EQ(pod.grf[2], 400.0);
  for (std::size_t k = 3; k < 6; ++k) {
    EXPECT_TRUE(std::isnan(pod.grf[k])) << k;
  }
  EXPECT_TRUE(std::isnan(pod.cop[0]) && std::isnan(pod.cop[1]));
}

// A floor-on-foot wrench of a known CoP p = (x, y, 0) in the site frame:
// τ = p × f, so τ_x = y f_z and τ_y = −x f_z. Values exact in float, so the CoP
// is exact too.
TEST(LocomotionTruthLog, TheCopOfAKnownOffset) {
  constexpr double x = 0.0625;
  constexpr double y = -0.03125;
  constexpr double fz = 400.0;
  const double fx = 3.0;
  const double fy = -1.5;
  const double tx = y * fz;
  const double ty = -x * fz;
  const double tz = x * fy - y * fx;
  rtc::DeviceState d = Lanes();
  SetGroup(d, 1,
           {static_cast<float>(-fx), static_cast<float>(-fy), static_cast<float>(-fz),
            static_cast<float>(-tx), static_cast<float>(-ty), static_cast<float>(-tz)});
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, Inputs(), pod);
  EXPECT_LE(std::abs(pod.cop[2] - x), 1e-12);
  EXPECT_LE(std::abs(pod.cop[3] - y), 1e-12);
}

TEST(LocomotionTruthLog, TheCopNeedsANormalForceAboveTheThreshold) {
  for (const float fz : {static_cast<float>(kCopMin), 0.0F, 5.0F, -20.0F}) {
    SCOPED_TRACE(fz);
    rtc::DeviceState d = Lanes();
    SetGroup(d, 0, {0.0F, 0.0F, -fz, 1.0F, 1.0F, 0.0F});
    LocomotionTruthLogPod pod{};
    loco::FillLocomotionTruthLog(d, Inputs(), pod);
    EXPECT_TRUE(std::isnan(pod.cop[0]) && std::isnan(pod.cop[1]));
    EXPECT_FALSE(std::isnan(pod.grf[2]));  // the wrench is still logged
  }
  {  // no threshold set: no CoP at all rather than an unguarded ratio
    rtc::DeviceState d = Lanes();
    SetGroup(d, 0, {0.0F, 0.0F, -100.0F, 1.0F, 1.0F, 0.0F});
    loco::TruthLogInputs in = Inputs();
    in.cop_min_fz = 0.0;
    LocomotionTruthLogPod pod{};
    loco::FillLocomotionTruthLog(d, in, pod);
    EXPECT_TRUE(std::isnan(pod.cop[0]));
  }
  rtc::DeviceState d = Lanes();
  SetGroup(d, 0,
           {0.0F, 0.0F, -std::nextafter(static_cast<float>(kCopMin), 100.0F), 1.0F, 1.0F, 0.0F});
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, Inputs(), pod);
  EXPECT_FALSE(std::isnan(pod.cop[0]));
}

TEST(LocomotionTruthLog, AContactGroupThatIsNotFreshOrDoesNotFitIsNaN) {
  struct Case {
    const char* label;
    void (*spoil)(rtc::DeviceState&, loco::TruthLogInputs&);
    std::uint32_t mask;  // expected contact bits
  };

  const Case cases[] = {
      {"producer verdict false",
       [](rtc::DeviceState& d, loco::TruthLogInputs&) { d.inference_enable[0] = false; }, 0b100U},
      {"older than the bound",
       [](rtc::DeviceState& d, loco::TruthLogInputs&) {
         d.inference_recv_steady_ns[1] = kNow - kMaxAge - 1;
       },
       0b010U},
      {"never received",
       [](rtc::DeviceState& d, loco::TruthLogInputs&) { d.inference_recv_steady_ns[0] = 0; },
       0b100U},
      {"group count is not the contact count",
       [](rtc::DeviceState& d, loco::TruthLogInputs&) { d.num_inference_groups = 3; }, 0U},
      {"stride without room for the force",
       [](rtc::DeviceState&, loco::TruthLogInputs& in) { in.stride = 3; }, 0U},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.label);
    rtc::DeviceState d = Lanes();
    SetGroup(d, 0, {1.0F, 1.0F, -100.0F, 1.0F, 1.0F, 1.0F});
    SetGroup(d, 1, {1.0F, 1.0F, -100.0F, 1.0F, 1.0F, 1.0F});
    loco::TruthLogInputs in = Inputs();
    c.spoil(d, in);
    LocomotionTruthLogPod pod{};
    loco::FillLocomotionTruthLog(d, in, pod);
    EXPECT_EQ(pod.truth_mask & 0b110U, c.mask);
    for (std::size_t g = 0; g < 2; ++g) {
      const bool fresh = (c.mask >> (1 + g)) & 1U;
      EXPECT_EQ(std::isnan(pod.grf[g * 6 + 2]), !fresh) << g;
      EXPECT_EQ(std::isnan(pod.cop[g * 2]), !fresh) << g;
    }
  }
}

// No lanes at all — a robot: the mask is 0 (the controller pushes no row).
TEST(LocomotionTruthLog, NoTruthLanesIsAnEmptyMask) {
  rtc::DeviceState d{};
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, Inputs(), pod);
  EXPECT_EQ(pod.truth_mask, 0U);
  EXPECT_DOUBLE_EQ(pod.base_age_us, -1.0);
  EXPECT_DOUBLE_EQ(pod.contact_age_us[0], -1.0);
}

// ── the external wrench lane ───────────────────────────────────────────────

loco::ExternalWrenchSample Push(std::int64_t age_ns = 1'000'000) {
  loco::ExternalWrenchSample s;
  s.wrench = {40.0, -12.5, 3.25, 0.5, -0.25, 1.75};
  s.recv_steady_ns = kNow - age_ns;
  s.sequence = 3;
  return s;
}

loco::TruthLogInputs InputsWith(const loco::ExternalWrenchSample* sample) {
  loco::TruthLogInputs in = Inputs();
  in.external = sample;
  in.external_max_age_ns = 20'000'000;  // 20 ms
  return in;
}

// D2: a fresh report is the received wrench, bit for bit, under its own bit.
TEST(LocomotionTruthLog, AFreshExternalWrenchIsTheReceivedOne) {
  const rtc::DeviceState d = Lanes();
  const loco::ExternalWrenchSample s = Push();
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, InputsWith(&s), pod);
  EXPECT_NE(pod.truth_mask & LocomotionTruthLogPod::kExternalBit, 0U);
  for (std::size_t k = 0; k < 6; ++k) {
    EXPECT_TRUE(SameBits(pod.ext_wrench[k], s.wrench[k])) << k;
  }
  EXPECT_DOUBLE_EQ(pod.ext_age_us, 1000.0);
  // The other lanes are what they are without it.
  LocomotionTruthLogPod plain{};
  loco::FillLocomotionTruthLog(d, Inputs(), plain);
  EXPECT_EQ(pod.truth_mask & ~LocomotionTruthLogPod::kExternalBit, plain.truth_mask);
  EXPECT_EQ(LocomotionTruthLogPod::kExternalBit & 0x1FU, 0U) << "clear of the base and contacts";
}

// D2: a zero wrench is a report too — "nothing applied" is not "nothing known".
TEST(LocomotionTruthLog, AZeroExternalWrenchIsFreshAndZero) {
  loco::ExternalWrenchSample s = Push();
  s.wrench.fill(0.0);
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(rtc::DeviceState{}, InputsWith(&s), pod);
  EXPECT_EQ(pod.truth_mask, LocomotionTruthLogPod::kExternalBit);
  for (double x : pod.ext_wrench) {
    EXPECT_EQ(x, 0.0);
  }
}

// D2: no lane, no message yet, a stale or a non-finite report — NaN, bit off.
TEST(LocomotionTruthLog, AnExternalWrenchThatIsNotKnownIsNaNWithItsBitOff) {
  const rtc::DeviceState d = Lanes();
  const auto expect_off = [&d](const loco::ExternalWrenchSample* s, double age_us,
                               const char* what) {
    LocomotionTruthLogPod pod{};
    pod.ext_wrench.fill(7.0);  // a held value must not survive
    loco::FillLocomotionTruthLog(d, InputsWith(s), pod);
    EXPECT_EQ(pod.truth_mask & LocomotionTruthLogPod::kExternalBit, 0U) << what;
    for (double x : pod.ext_wrench) {
      EXPECT_TRUE(std::isnan(x)) << what;
    }
    EXPECT_DOUBLE_EQ(pod.ext_age_us, age_us) << what;
  };
  expect_off(nullptr, -1.0, "no lane");
  loco::ExternalWrenchSample none = Push();
  none.sequence = 0;
  expect_off(&none, -1.0, "no message yet");
  const loco::ExternalWrenchSample stale = Push(20'000'001);
  expect_off(&stale, 20000.001, "stale");
  loco::ExternalWrenchSample bad = Push();
  bad.wrench[4] = std::numeric_limits<double>::infinity();
  expect_off(&bad, 1000.0, "non-finite");
  // At the bound it is still fresh; stamped after `now` it is age 0.
  const loco::ExternalWrenchSample edge = Push(20'000'000);
  const loco::ExternalWrenchSample ahead = Push(-5'000);
  for (const auto* s : {&edge, &ahead}) {
    LocomotionTruthLogPod pod{};
    loco::FillLocomotionTruthLog(d, InputsWith(s), pod);
    EXPECT_NE(pod.truth_mask & LocomotionTruthLogPod::kExternalBit, 0U);
  }
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, InputsWith(&ahead), pod);
  EXPECT_DOUBLE_EQ(pod.ext_age_us, 0.0);
}

// D1 / D2: the block's columns exist only when asked for, after every other.
TEST(LocomotionTruthLog, TheExternalColumnsAreAddedOnlyOnRequest) {
  const std::vector<std::string> names = {"left", "right"};
  const auto header = [&names](bool external) {
    std::ostringstream os;
    integrated_bringup::WriteLocomotionTruthLogHeader(
        os, names, integrated_bringup::LocomotionTruthLogColumnsFor(names, external));
    return os.str();
  };
  const auto row = [&names](bool external) {
    LocomotionTruthLogPod pod{};
    pod.ext_wrench = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    pod.ext_age_us = 250.0;
    std::ostringstream os;
    integrated_bringup::WriteLocomotionTruthLogRow(
        os, pod, integrated_bringup::LocomotionTruthLogColumnsFor(names, external));
    return os.str();
  };
  const std::string plain = header(false);
  EXPECT_EQ(header(true), plain +
                              ",truth_ext_fx,truth_ext_fy,truth_ext_fz,truth_ext_tx,"
                              "truth_ext_ty,truth_ext_tz,truth_ext_age_us");
  EXPECT_EQ(plain.find("truth_ext"), std::string::npos);
  EXPECT_EQ(header(false), [&names] {
    std::ostringstream os;
    integrated_bringup::WriteLocomotionTruthLogHeader(
        os, names, integrated_bringup::LocomotionTruthLogColumnsFor(names));
    return os.str();
  }()) << "the default is the header without the block";
  EXPECT_EQ(row(true), row(false) + ",1,2,3,4,5,6,250");
}

// Contacts past the model's count are NaN and unclaimed.
TEST(LocomotionTruthLog, ContactsPastTheModelAreNaN) {
  const rtc::DeviceState d = Lanes();
  LocomotionTruthLogPod pod{};
  loco::FillLocomotionTruthLog(d, Inputs(), pod);
  for (std::size_t c = 2; c < LocomotionTruthLogPod::kMaxContacts; ++c) {
    EXPECT_TRUE(std::isnan(pod.grf[c * 6]));
    EXPECT_EQ(pod.contact_seq[c], 0U);
    EXPECT_DOUBLE_EQ(pod.contact_age_us[c], -1.0);
  }
}

}  // namespace

// ── the controller: the file, its rows, and what it does not change ─────────

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using integrated_bringup::LocomotionControllerTestAccess;
namespace fs = std::filesystem;

constexpr int kTicks = 2500;
constexpr int kDrainEvery = 256;  // the ring holds 512; the rig spins no drain timer

YAML::Node InekfConfig() {
  YAML::Node cfg = YAML::Clone(ShippedConfig());
  cfg["estimator"]["source"] = "inekf";
  return cfg;
}

std::size_t Lines(const fs::path& p) {
  std::ifstream in(p);
  std::size_t n = 0;
  for (std::string line; std::getline(in, line);) {
    n += line.empty() ? 0 : 1;
  }
  return n;
}

bool SameOutput(const rtc::ControllerOutput& a, const rtc::ControllerOutput& b) {
  if (a.num_devices != b.num_devices || a.valid != b.valid) {
    return false;
  }
  for (int d = 0; d < a.num_devices; ++d) {
    const auto& x = a.devices[static_cast<std::size_t>(d)];
    const auto& y = b.devices[static_cast<std::size_t>(d)];
    if (x.num_channels != y.num_channels || x.has_pd_gains != y.has_pd_gains ||
        x.command_type != y.command_type) {
      return false;
    }
    for (const auto field :
         {&rtc::DeviceOutput::commands, &rtc::DeviceOutput::goal_positions,
          &rtc::DeviceOutput::target_positions, &rtc::DeviceOutput::target_velocities,
          &rtc::DeviceOutput::trajectory_positions, &rtc::DeviceOutput::trajectory_velocities,
          &rtc::DeviceOutput::feedforward, &rtc::DeviceOutput::pd_kp, &rtc::DeviceOutput::pd_kd}) {
      for (int c = 0; c < x.num_channels; ++c) {
        const auto u = static_cast<std::size_t>(c);
        if (!SameBits((x.*field)[u], (y.*field)[u])) {
          return false;
        }
      }
    }
  }
  return true;
}

class LocomotionTruthLogRigTest : public LocomotionControllerTest {
 protected:
  void ConfigureShipped(const YAML::Node& cfg) {
    ConfigureWithDevices(cfg, ShippedLaneDevices(cfg));
  }

  void BringUpShipped(const YAML::Node& cfg = ShippedConfig()) {
    ConfigureShipped(cfg);
    Activate();
    MakeStanding();
  }

  fs::path Path(std::string_view instance) const {
    return LocomotionControllerTestAccess::LogPath(*ctrl_, instance);
  }

  // One tick with the joint and IMU lanes stamped now and the base lane left
  // as it is (never received, after LoseBase).
  rtc::ControllerOutput TickWithoutBase() {
    const int64_t now = NowNs();
    state_.iteration += 1;
    state_.devices[kBody].joint_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    StampImu(now);
    return ctrl_->Compute(state_);
  }

  void LoseBase() {
    state_.devices[kBody].base_valid = false;
    state_.devices[kBody].base_recv_steady_ns = 0;
  }

  // Both soles' lane groups fresh now, standing (foot-on-floor −fz).
  void StampContacts(int64_t now) {
    auto& body = state_.devices[kBody];
    body.num_inference_groups = 2;
    for (std::size_t c = 0; c < 2; ++c) {
      body.inference_enable[c] = true;
      body.inference_recv_steady_ns[c] = now;
      body.inference_sequence[c] += 1;
      const std::size_t at = c * 7;
      body.inference_data[at + 1] = 1.5F;
      body.inference_data[at + 2] = -0.75F;
      body.inference_data[at + 3] = -170.0F - static_cast<float>(c);
      body.inference_data[at + 4] = 0.25F + static_cast<float>(c);  // groups differ
      body.inference_data[at + 5] = -1.25F;
      body.inference_data[at + 6] = 0.125F;
    }
  }

  void Drain() {
    LocomotionControllerTestAccess::DrainLogs(*ctrl_);
    ASSERT_EQ(LocomotionControllerTestAccess::LogDrops(*ctrl_), 0U);
    ASSERT_EQ(LocomotionControllerTestAccess::LogUnwritten(*ctrl_), 0U);
  }

  void Cleanup() {
    Deactivate();
    ASSERT_EQ(ctrl_->on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }

  rtc::testing::ScopedSessionDir session_;
};

// T1: no truth lane on any tick — no file, while the R-tier file is there
// from configure on.
TEST_F(LocomotionTruthLogRigTest, NoTruthLaneMakesNoTruthFile) {
  ConfigureShipped(InekfConfig());
  const fs::path truth = Path(integrated_bringup::kLocomotionTruthLogInstance);
  const fs::path diag = Path(integrated_bringup::kLocomotionDiagLogInstance);
  ASSERT_FALSE(truth.empty()) << "the shipped logs register the T tier";
  EXPECT_TRUE(fs::exists(diag));
  EXPECT_FALSE(fs::exists(truth));
  Activate();
  MakeStanding();
  LoseBase();
  for (int k = 0; k < kTicks; ++k) {
    last_out_ = TickWithoutBase();
    ASSERT_EQ(ctrl_->LastTruthLog().truth_mask, 0U) << k;
    if ((k + 1) % kDrainEvery == 0) {
      Drain();
    }
  }
  Drain();
  EXPECT_EQ(Lines(diag), static_cast<std::size_t>(kTicks) + 1);  // the R tier still logs
  Cleanup();
  EXPECT_FALSE(fs::exists(truth));
}

// T2: a fresh base on every tick — one row per tick, no drop.
TEST_F(LocomotionTruthLogRigTest, AFreshBaseOnEveryTickIsOneRowPerTick) {
  BringUpShipped();
  for (int k = 0; k < kTicks; ++k) {
    last_out_ = Tick();
    ASSERT_EQ(ctrl_->LastTruthLog().truth_mask, LocomotionTruthLogPod::kBaseBit) << k;
    if ((k + 1) % kDrainEvery == 0) {
      Drain();
    }
  }
  Drain();
  const fs::path truth = Path(integrated_bringup::kLocomotionTruthLogInstance);
  Cleanup();
  EXPECT_EQ(Lines(truth), static_cast<std::size_t>(kTicks) + 1);
}

// T2: lanes that come and go — a row exactly on the ticks with a fresh one.
TEST_F(LocomotionTruthLogRigTest, MixedLanesWriteARowOnlyOnTicksWithOne) {
  ConfigureShipped(InekfConfig());
  Activate();
  MakeStanding();
  std::size_t with_truth = 0;
  for (int k = 0; k < kTicks; ++k) {
    const int64_t now = NowNs();
    const bool base = k % 3 == 0;
    const bool contacts = k % 5 == 0;
    auto& body = state_.devices[kBody];
    body.base_valid = true;
    body.base_recv_steady_ns = base ? now : now - 1'000'000'000;
    body.num_inference_groups = 0;
    if (contacts) {
      StampContacts(now);
    }
    last_out_ = TickWithoutBase();
    const std::uint32_t want =
        (base ? LocomotionTruthLogPod::kBaseBit : 0U) | (contacts ? 0b110U : 0U);
    ASSERT_EQ(ctrl_->LastTruthLog().truth_mask, want) << k;
    with_truth += want != 0 ? 1 : 0;
    if ((k + 1) % kDrainEvery == 0) {
      Drain();
    }
  }
  Drain();
  const fs::path truth = Path(integrated_bringup::kLocomotionTruthLogInstance);
  Cleanup();
  EXPECT_EQ(Lines(truth), with_truth + 1);
}

// T3: the T base is the raw lane; the R base is the state source's normalised
// copy; a stale base clears the bit.
TEST_F(LocomotionTruthLogRigTest, TheTruthBaseIsRawAndTheDiagBaseNormalised) {
  BringUpShipped();
  auto& p = state_.devices[kBody].base_pose;
  p[3] = 0.01;
  p[4] = 0.0;
  p[5] = 0.0;
  p[6] = 1.0001;  // |q| − 1 ≈ 1e-4, inside the source's tolerance
  last_out_ = Tick();
  const auto& t = ctrl_->LastTruthLog();
  const auto& r = ctrl_->LastDiag();
  const double n = std::sqrt(p[3] * p[3] + p[4] * p[4] + p[5] * p[5] + p[6] * p[6]);
  ASSERT_EQ(t.truth_mask & LocomotionTruthLogPod::kBaseBit, LocomotionTruthLogPod::kBaseBit);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(SameBits(t.base_q_xyzw[i], p[3 + i])) << i;
    EXPECT_TRUE(SameBits(r.base_q_xyzw[i], p[3 + i] / n)) << i;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(SameBits(t.base_p[i], p[i])) << i;
    EXPECT_TRUE(SameBits(r.base_p[i], p[i])) << i;
  }
  EXPECT_FALSE(SameBits(t.base_q_xyzw[3], r.base_q_xyzw[3]));

  // A base older than body_max_age: NaN, bit off.
  const int64_t now = NowNs();
  state_.iteration += 1;
  state_.devices[kBody].joint_recv_steady_ns = now;
  state_.devices[kHand].joint_recv_steady_ns = now;
  state_.devices[kBody].base_recv_steady_ns = now - 1'000'000'000;
  StampImu(now);
  last_out_ = ctrl_->Compute(state_);
  EXPECT_EQ(ctrl_->LastTruthLog().truth_mask & LocomotionTruthLogPod::kBaseBit, 0U);
  EXPECT_TRUE(std::isnan(ctrl_->LastTruthLog().base_p[0]));
}

// T3: under the estimator the R base is the estimate and the T base the truth.
TEST_F(LocomotionTruthLogRigTest, UnderTheEstimatorTheTruthBaseIsNotTheDiagBase) {
  ConfigureShipped(InekfConfig());
  Activate();
  MakeStanding();
  state_.devices[kBody].base_pose[0] = 5.0;  // a truth the estimate does not follow
  last_out_ = Tick();
  EXPECT_TRUE(SameBits(ctrl_->LastTruthLog().base_p[0], 5.0));
  EXPECT_LT(std::abs(ctrl_->LastDiag().base_p[0]), 0.1);
}

// T4: the contact block is the negated lane, with the moment of the shipped
// moment lane; the diag's bits name the same groups.
TEST_F(LocomotionTruthLogRigTest, TheTruthWrenchIsTheNegatedLane) {
  BringUpShipped();
  StampContacts(NowNs());
  last_out_ = Tick();
  const auto& t = ctrl_->LastTruthLog();
  const auto& r = ctrl_->LastDiag();
  const auto& lane = state_.devices[kBody].inference_data;
  ASSERT_EQ(t.truth_mask, 0b111U);
  for (std::size_t c = 0; c < 2; ++c) {
    EXPECT_NE(r.meas_contact_valid & (1U << c), 0U) << c;
    for (std::size_t k = 0; k < 6; ++k) {
      EXPECT_TRUE(SameBits(t.grf[c * 6 + k], -static_cast<double>(lane[c * 7 + 1 + k])))
          << c << ' ' << k;
    }
    EXPECT_FALSE(std::isnan(t.cop[c * 2]));
  }
}

// The file read back BY NAME against the tick's record: every column of the
// last row is the pod field its name says (9 significant digits in the file).
TEST_F(LocomotionTruthLogRigTest, TheFileHoldsTheRecordUnderItsColumnNames) {
  BringUpShipped();
  StampContacts(NowNs());
  for (int k = 0; k < 5; ++k) {
    last_out_ = Tick();
  }
  const LocomotionTruthLogPod t = ctrl_->LastTruthLog();
  Drain();
  const auto csv = fx::ReadCsv(Path(integrated_bringup::kLocomotionTruthLogInstance));
  ASSERT_EQ(csv.rows.size(), 5U);
  const std::size_t last = csv.rows.size() - 1;
  // The contact names, in column order, from the CoP columns.
  std::vector<std::string> contacts;
  for (const auto& name : csv.header) {
    const std::string pre = "truth_cop_";
    if (name.rfind(pre, 0) == 0 && name.size() > pre.size() + 2 &&
        name.compare(name.size() - 2, 2, "_x") == 0) {
      contacts.push_back(name.substr(pre.size(), name.size() - pre.size() - 2));
    }
  }
  ASSERT_EQ(contacts.size(), 2U);
  EXPECT_EQ(csv.header.size(), 18U + 10U * contacts.size());
  const auto expect = [&](const std::string& name, double want) {
    const double got = csv.At(last, name);
    if (std::isnan(want)) {
      EXPECT_TRUE(std::isnan(got)) << name;
    } else {
      EXPECT_NEAR(got, want, 1e-6 * std::max(1.0, std::abs(want))) << name;
    }
  };
  expect("tick", static_cast<double>(t.tick));
  expect("truth_mask", t.truth_mask);
  expect("truth_base_seq", static_cast<double>(t.base_seq));
  expect("truth_base_age_us", t.base_age_us);
  const char* xyz[] = {"x", "y", "z"};
  const char* q[] = {"qx", "qy", "qz", "qw"};
  for (std::size_t i = 0; i < 3; ++i) {
    expect(std::string("truth_base_p") + xyz[i], t.base_p[i]);
    expect(std::string("truth_base_v") + xyz[i], t.base_v[i]);
    expect(std::string("truth_base_w") + xyz[i], t.base_w[i]);
  }
  for (std::size_t i = 0; i < 4; ++i) {
    expect(std::string("truth_base_") + q[i], t.base_q_xyzw[i]);
  }
  const char* axes[] = {"fx", "fy", "fz", "tx", "ty", "tz"};
  for (std::size_t c = 0; c < contacts.size(); ++c) {
    const std::string& n = contacts[c];
    for (std::size_t k = 0; k < 6; ++k) {
      expect("truth_grf_" + n + "_" + axes[k], t.grf[c * 6 + k]);
    }
    expect("truth_cop_" + n + "_x", t.cop[c * 2]);
    expect("truth_cop_" + n + "_y", t.cop[c * 2 + 1]);
    expect("truth_age_" + n + "_us", t.contact_age_us[c]);
    expect("truth_seq_" + n, static_cast<double>(t.contact_seq[c]));
  }
  // The two groups' moments differ, so a column read from the other group shows.
  EXPECT_NE(t.grf[3], t.grf[9]);
}

// Without the body group's moment flag the moment and the CoP are NaN, the
// force is still there.
TEST_F(LocomotionTruthLogRigTest, WithoutTheMomentFlagTheTruthMomentIsNaN) {
  const YAML::Node cfg = ShippedConfig();
  auto devices = ShippedLaneDevices(cfg);
  devices.at("g1").sensor_layout->has_native_moment = false;
  ConfigureWithDevices(cfg, devices);
  Activate();
  MakeStanding();
  StampContacts(NowNs());
  last_out_ = Tick();
  const auto& t = ctrl_->LastTruthLog();
  ASSERT_EQ(t.truth_mask, 0b111U);
  for (std::size_t c = 0; c < 2; ++c) {
    EXPECT_FALSE(std::isnan(t.grf[c * 6 + 2])) << c;
    for (std::size_t k = 3; k < 6; ++k) {
      EXPECT_TRUE(std::isnan(t.grf[c * 6 + k])) << c << ' ' << k;
    }
    EXPECT_TRUE(std::isnan(t.cop[c * 2]) && std::isnan(t.cop[c * 2 + 1])) << c;
  }
}

// G4 (b): the T log changes no output — the same 2500 ticks, logged and not.
TEST_F(LocomotionTruthLogRigTest, TheTruthLogChangesNoOutput) {
  YAML::Node off = YAML::Clone(ShippedConfig());
  YAML::Node logs(YAML::NodeType::Sequence);
  for (const auto& e : off["logs"]) {
    if (e["msg_type"].as<std::string>() != integrated_bringup::kLocomotionTruthLogMsgType) {
      logs.push_back(e);
    }
  }
  off["logs"] = logs;
  // The second controller, without the T log, on a node of its own.
  auto node_off = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_locomotion_truth_off", "", rclcpp::NodeOptions{}.use_global_arguments(false));
  auto ctrl_off = MakeController();
  ASSERT_EQ(ctrl_off->PreConfigure(node_off, off), CallbackReturn::SUCCESS);
  ctrl_off->SetDeviceNameConfigs(ShippedLaneDevices(off));
  ASSERT_EQ(ctrl_off->on_configure(rclcpp_lifecycle::State{}, node_off, off),
            CallbackReturn::SUCCESS);
  EXPECT_TRUE(LocomotionControllerTestAccess::LogPath(
                  *ctrl_off, integrated_bringup::kLocomotionTruthLogInstance)
                  .empty());
  BringUpShipped();
  ASSERT_EQ(ctrl_off->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  for (int k = 0; k < kTicks; ++k) {
    const int64_t now = NowNs();
    state_.iteration += 1;
    state_.devices[kBody].joint_recv_steady_ns = now;
    state_.devices[kBody].base_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    StampImu(now);
    if (k % 2 == 0) {
      StampContacts(now);
    }
    const rtc::ControllerOutput on = ctrl_->Compute(state_);
    const rtc::ControllerOutput no = ctrl_off->Compute(state_);
    ASSERT_TRUE(SameOutput(on, no)) << "tick " << k;
    if ((k + 1) % kDrainEvery == 0) {
      Drain();
    }
  }
  ASSERT_EQ(ctrl_off->on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
}

// ── the external wrench lane on the controller ────────────────────────────

geometry_msgs::msg::WrenchStamped PushMessage(double fx) {
  geometry_msgs::msg::WrenchStamped m;
  m.wrench.force.x = fx;
  m.wrench.force.y = -12.5;
  m.wrench.force.z = 3.25;
  m.wrench.torque.x = 0.5;
  m.wrench.torque.y = -0.25;
  m.wrench.torque.z = 1.75;
  return m;
}

YAML::Node ExternalWrenchConfig(double max_age_ms = 50.0) {
  YAML::Node cfg = YAML::Clone(ShippedConfig());
  cfg["external_wrench"]["topic"] = "sim/external_wrench";
  cfg["external_wrench"]["max_age_ms"] = max_age_ms;
  return cfg;
}

// The section's shape: both keys, a topic that may be empty, a finite age > 0.
TEST(LocomotionExternalWrenchConfig, BadShapesAreRefused) {
  const auto parsed =
      integrated_bringup::ParseLocomotionControllerConfig(ExternalWrenchConfig(40.0));
  EXPECT_EQ(parsed.external_wrench_topic, "sim/external_wrench");
  EXPECT_DOUBLE_EQ(parsed.external_wrench_max_age_ms, 40.0);
  YAML::Node absent = YAML::Clone(ShippedConfig());
  absent.remove("external_wrench");
  EXPECT_TRUE(
      integrated_bringup::ParseLocomotionControllerConfig(absent).external_wrench_topic.empty());
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    YAML::Node cfg = ExternalWrenchConfig();
    spoil(cfg);
    EXPECT_THROW(static_cast<void>(integrated_bringup::ParseLocomotionControllerConfig(cfg)),
                 std::invalid_argument)
        << what;
  };
  refused([](YAML::Node& c) { c["external_wrench"].remove("max_age_ms"); }, "no age");
  refused([](YAML::Node& c) { c["external_wrench"].remove("topic"); }, "no topic");
  refused([](YAML::Node& c) { c["external_wrench"]["max_age_ms"] = 0.0; }, "zero age");
  refused([](YAML::Node& c) { c["external_wrench"]["max_age_ms"] = -5.0; }, "negative age");
  refused([](YAML::Node& c) { c["external_wrench"]["max_age_ms"] = ".inf"; }, "infinite age");
  refused([](YAML::Node& c) { c["external_wrench"]["max_age_ms"] = "soon"; }, "not a number");
  refused([](YAML::Node& c) { c["external_wrench"]["body"] = "pelvis"; }, "unknown key");
  refused([](YAML::Node& c) { c["external_wrench"]["topic"] = YAML::Node(YAML::NodeType::Map); },
          "a topic that is not a name");
}

// D1: the lane exists only when asked for — whatever topic the shipped
// configuration carries, the subscription follows it, and with the lane off
// (an empty topic, what the profile ships today) no subscription, no bit, and
// (TheFileHoldsTheRecordUnderItsColumnNames) the header it always had. Which of the two ships is a
// deployment choice (a push scenario turns it on in an overlay), not pinned here.
TEST_F(LocomotionTruthLogRigTest, TheShippedExternalWrenchLaneFollowsItsTopic) {
  ASSERT_TRUE(ShippedConfig()["external_wrench"]) << "an overlay can only replace a shipped key";
  const std::string topic = ShippedConfig()["external_wrench"]["topic"].as<std::string>();
  const bool lane_on = !topic.empty();
  if (lane_on) {
    EXPECT_EQ(topic.find(' '), std::string::npos) << "a topic name has no blanks: " << topic;
  }
  BringUpShipped();
  EXPECT_EQ(ctrl_->ExternalWrenchSubscribed(), lane_on);
  if (lane_on) {
    return;  // the lane-on behaviour is TheExternalWrenchReachesTheTruthLog's
  }
  // A callback that somehow ran would still not reach the log.
  LocomotionControllerTestAccess::OnExternalWrench(*ctrl_, PushMessage(40.0));
  for (int k = 0; k < 5; ++k) {
    last_out_ = Tick();
    EXPECT_EQ(ctrl_->LastTruthLog().truth_mask & LocomotionTruthLogPod::kExternalBit, 0U);
  }
  Drain();
  const auto csv = fx::ReadCsv(Path(integrated_bringup::kLocomotionTruthLogInstance));
  for (const auto& name : csv.header) {
    EXPECT_EQ(name.rfind("truth_ext", 0), std::string::npos) << name;
  }
}

// D2: with a topic, the tick after a message logs it; the next message
// replaces it; with no message for longer than the bound the bit goes off.
TEST_F(LocomotionTruthLogRigTest, TheExternalWrenchReachesTheTruthLog) {
  BringUpShipped(ExternalWrenchConfig(50.0));
  ASSERT_TRUE(ctrl_->ExternalWrenchSubscribed());
  last_out_ = Tick();
  EXPECT_EQ(ctrl_->LastTruthLog().truth_mask & LocomotionTruthLogPod::kExternalBit, 0U)
      << "nothing received yet";
  EXPECT_DOUBLE_EQ(ctrl_->LastTruthLog().ext_age_us, -1.0);
  for (const double fx : {40.0, 0.0, -17.5}) {
    const auto msg = PushMessage(fx);
    LocomotionControllerTestAccess::OnExternalWrench(*ctrl_, msg);
    last_out_ = Tick();
    const LocomotionTruthLogPod t = ctrl_->LastTruthLog();
    ASSERT_NE(t.truth_mask & LocomotionTruthLogPod::kExternalBit, 0U) << fx;
    const std::array<double, 6> want = {msg.wrench.force.x,  msg.wrench.force.y,
                                        msg.wrench.force.z,  msg.wrench.torque.x,
                                        msg.wrench.torque.y, msg.wrench.torque.z};
    for (std::size_t k = 0; k < 6; ++k) {
      EXPECT_TRUE(SameBits(t.ext_wrench[k], want[k])) << fx << ' ' << k;
    }
    EXPECT_GE(t.ext_age_us, 0.0);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  last_out_ = Tick();
  const LocomotionTruthLogPod stale = ctrl_->LastTruthLog();
  EXPECT_EQ(stale.truth_mask & LocomotionTruthLogPod::kExternalBit, 0U);
  EXPECT_TRUE(std::isnan(stale.ext_wrench[0]));
  EXPECT_GE(stale.ext_age_us, 50e3);
  // The file carries the block under its names.
  Drain();
  const auto csv = fx::ReadCsv(Path(integrated_bringup::kLocomotionTruthLogInstance));
  ASSERT_GE(csv.header.size(), 7U);
  EXPECT_EQ(csv.header[csv.header.size() - 7], "truth_ext_fx");
  EXPECT_EQ(csv.header.back(), "truth_ext_age_us");
}

// D1: the lane feeds the log and nothing else — the same ticks with pushes
// reported and with the lane off give the same output, bit for bit.
TEST_F(LocomotionTruthLogRigTest, TheExternalWrenchChangesNoOutput) {
  const YAML::Node on_cfg = ExternalWrenchConfig();
  auto node_off = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_locomotion_ext_off", "", rclcpp::NodeOptions{}.use_global_arguments(false));
  auto ctrl_off = MakeController();
  const YAML::Node off = ShippedConfig();
  ASSERT_EQ(ctrl_off->PreConfigure(node_off, off), CallbackReturn::SUCCESS);
  ctrl_off->SetDeviceNameConfigs(ShippedLaneDevices(off));
  ASSERT_EQ(ctrl_off->on_configure(rclcpp_lifecycle::State{}, node_off, off),
            CallbackReturn::SUCCESS);
  BringUpShipped(on_cfg);
  ASSERT_EQ(ctrl_off->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  for (int k = 0; k < kTicks; ++k) {
    if (k % 7 == 0) {
      LocomotionControllerTestAccess::OnExternalWrench(*ctrl_, PushMessage(0.5 * k));
    }
    const int64_t now = NowNs();
    state_.iteration += 1;
    state_.devices[kBody].joint_recv_steady_ns = now;
    state_.devices[kBody].base_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    StampImu(now);
    const rtc::ControllerOutput on = ctrl_->Compute(state_);
    const rtc::ControllerOutput no = ctrl_off->Compute(state_);
    ASSERT_TRUE(SameOutput(on, no)) << "tick " << k;
  }
  EXPECT_NE(ctrl_->LastTruthLog().truth_mask & LocomotionTruthLogPod::kExternalBit, 0U);
}

}  // namespace
