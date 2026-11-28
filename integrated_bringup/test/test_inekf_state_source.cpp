// The inekf state source on its own: the start, the cadence (hold, gap), the
// joint / IMU coherence, the reset policy, the IMU-frame kinematics against
// the full model, the output map against hand-written formulas, determinism,
// and the `estimator.inekf` section. The model is the installed g1_loco one
// (a controller configured with truth lends its models and device layout);
// ticks run on a synthetic clock so receipt gaps are exact.
#include "integrated_bringup/controllers/locomotion/inekf_state_source.hpp"
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/joint/joints.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>

namespace rtc::estimation {

struct ContactFsmTestAccess {
  static ContactInekf& Core(ContactInekfSequencer& s) { return s.core_; }
};

struct ContactInekfTestAccess {
  static void SetMean(ContactInekf& f, const Eigen::Matrix3d& R, const Eigen::Vector3d& v,
                      const Eigen::Vector3d& p, const Eigen::Vector3d& omega) {
    f.R_ = R;
    f.v_ = v;
    f.p_ = p;
    f.omega_ = omega;
  }
};

}  // namespace rtc::estimation

namespace integrated_bringup::locomotion {

struct InekfStateSourceTestAccess {
  static rtc::estimation::ContactInekfSequencer& Seq(InekfStateSource& s) { return s.seq_; }
};

}  // namespace integrated_bringup::locomotion

namespace {

namespace est = rtc::estimation;
namespace loco = integrated_bringup::locomotion;
using loco::EstimatorResetReason;
using loco::ImuFrameKinematics;
using loco::InekfSourceConfig;
using loco::InekfStateSource;
using loco::MeasuredRobotState;
using loco::StateSourceStatus;
using Access = loco::InekfStateSourceTestAccess;

constexpr int64_t kT = 2'000'000;  // the shipped control period [ns]
constexpr uint32_t kBoth = 0b11U;
const std::vector<std::string> kSoles = {"left_sole", "right_sole"};
constexpr const char* kImuFrame = "imu_in_pelvis";

using rtc::testing::BitsEqual;

template <typename A, typename B>
void ExpectBitsEqual(const A& a, const B& b, const char* what) {
  ASSERT_EQ(a.size(), b.size()) << what;
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    EXPECT_TRUE(BitsEqual(a.data()[i], b.data()[i]))
        << what << "[" << i << "] " << a.data()[i] << " vs " << b.data()[i];
  }
}

void ExpectClose(double a, double b, double rel, const std::string& what) {
  EXPECT_LE(std::abs(a - b), rel * std::max(1.0, std::abs(b))) << what << ": " << a << " vs " << b;
}

class InekfSourceTest : public loco_rig::LocomotionControllerTest {
 protected:
  void SetUp() override {
    Configure(loco_rig::ShippedConfig());
    MakeStanding();
    shipped_ = loco::ParseInekfSourceConfig(cfg_["estimator"]["inekf"], "inekf");
    reduced_ = std::make_shared<const pinocchio::Model>(ctrl_->ReducedModel());
    body_to_full_ = integrated_bringup::LocomotionControllerTestAccess::ReducedToFull(*ctrl_);
  }

  std::unique_ptr<InekfStateSource> MakeSource(const InekfSourceConfig& cfg,
                                               std::shared_ptr<const pinocchio::Model> model = {},
                                               const std::string& imu_frame = kImuFrame) {
    InekfStateSource::Params p;
    p.body_max_age_ns = 10 * 1'000'000;
    p.hand_max_age_ns = 50 * 1'000'000;
    p.model_epoch = 3;
    p.imu_site = 0;
    p.config = cfg;
    return std::make_unique<InekfStateSource>(
        p, std::make_unique<ImuFrameKinematics>(model ? model : reduced_, kSoles, imu_frame),
        body_to_full_);
  }

  // One tick: the IMU sequence advanced by k and stamped at the clock, the
  // joints `joint_after_ns` later; the source reads at the later of the two.
  bool Step(InekfStateSource& src, MeasuredRobotState& out, uint64_t k = 1,
            int64_t joint_after_ns = 0, uint32_t planned = kBoth) {
    now_ += kT;
    auto& body = state_.devices[loco_rig::kBody];
    body.imu_sequence[0] += k;
    body.imu_recv_steady_ns[0] = now_;
    body.joint_recv_steady_ns = now_ + joint_after_ns;
    state_.devices[loco_rig::kHand].joint_recv_steady_ns = now_;
    return src.Update(state_, ctrl_->Indices(), {planned},
                      now_ + std::max<int64_t>(0, joint_after_ns), out);
  }

  void SetImu(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel) {
    auto& s = state_.devices[loco_rig::kBody].imu[0];
    s = {gyro.x(), gyro.y(), gyro.z(), accel.x(), accel.y(), accel.z()};
  }

  Eigen::Vector3d Gyro() const {
    const auto& s = state_.devices[loco_rig::kBody].imu[0];
    return {s[0], s[1], s[2]};
  }

  Eigen::Vector3d Accel() const {
    const auto& s = state_.devices[loco_rig::kBody].imu[0];
    return {s[3], s[4], s[5]};
  }

  // The body channel of a model joint.
  int Channel(const std::string& joint) const {
    const auto full_order = loco::JointOrder(ctrl_->FullModel());
    const auto& idx = ctrl_->Indices();
    for (int c = 0; c < idx.body.num_channels; ++c) {
      if (full_order[static_cast<std::size_t>(idx.body.joint[static_cast<std::size_t>(c)])] ==
          joint) {
        return c;
      }
    }
    ADD_FAILURE() << joint;
    return 0;
  }

  // The body joints in the reduced model's order, from the state.
  std::vector<double> BodyQ(const MeasuredRobotState& out) const {
    std::vector<double> q;
    for (const int a : body_to_full_) {
      q.push_back(out.q[7 + static_cast<std::size_t>(a)]);
    }
    return q;
  }

  // An independent kinematics + sequencer: what the source should have done.
  struct Oracle {
    std::unique_ptr<ImuFrameKinematics> kin;
    est::ContactInekfSequencer seq;
    std::array<est::ContactKinematics, 2> k{};
    std::array<Eigen::Vector3d, 2> rdot{};

    void Kinematics(const std::vector<double>& q, const std::vector<double>& qd) {
      for (auto& x : k) {
        x.jacobian.resize(3, kin->num_joints());
      }
      kin->Compute(q, qd, k, rdot);
    }
  };

  Oracle MakeOracle(const InekfSourceConfig& cfg) {
    Oracle o;
    o.kin = std::make_unique<ImuFrameKinematics>(reduced_, kSoles, kImuFrame);
    o.seq.Init(cfg.core, cfg.fsm, 2);
    return o;
  }

  // ⌈d / T⌉ of a policy duration (the source's own rule).
  static int LimitTicks(double d) {
    return static_cast<int>(std::ceil(d / (static_cast<double>(kT) * 1e-9) * (1.0 - 1e-12)));
  }

  InekfSourceConfig shipped_;
  std::shared_ptr<const pinocchio::Model> reduced_;
  std::vector<int> body_to_full_;
  int64_t now_{1'000'000'000'000};
};

// ── start ───────────────────────────────────────────────────────────────────

TEST_F(InekfSourceTest, StartsOnTheFirstStillSampleWithTheStanceMeanAsOrigin) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out)) << loco::ToString(out.status);
  EXPECT_EQ(loco::StateSourceKind::kEstimated, out.kind);
  EXPECT_TRUE(out.estimator.initialized);
  EXPECT_EQ(-1, out.estimator.periods);
  EXPECT_NE(0U, out.fresh_mask & loco::kBaseBit);
  EXPECT_NE(0U, out.valid_mask & loco::kBaseBit);
  EXPECT_EQ(3U, out.model_epoch);
  // The still pelvis reads gravity on its z axis: the start is level, heading 0,
  // and the base sits above the mean of the soles (the gauge's origin).
  const pinocchio::Model& full = ctrl_->FullModel();
  pinocchio::Data data(full);
  Eigen::VectorXd q = pinocchio::neutral(full);
  for (int a = 0; a < out.nq - 7; ++a) {
    q(7 + a) = out.q[7 + static_cast<std::size_t>(a)];
  }
  pinocchio::framesForwardKinematics(full, data, q);
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& s : kSoles) {
    mean += 0.5 * data.oMf[full.getFrameId(s)].translation();
  }
  for (int i = 0; i < 3; ++i) {
    ExpectClose(out.q[static_cast<std::size_t>(i)], -mean(i), 1e-12, "p_WB");
  }
  EXPECT_EQ(1.0, out.q[6]);
  EXPECT_EQ(0.0, out.q[3]);
  EXPECT_EQ(0.0, out.q[4]);
  EXPECT_EQ(0.0, out.q[5]);
  EXPECT_EQ(loco::kYawUnobservable | loco::kPositionUnobservable, out.estimator.unobservable);
  // The next sample steps once.
  ASSERT_TRUE(Step(*src, out));
  EXPECT_EQ(1, out.estimator.periods);
}

TEST_F(InekfSourceTest, AWindowOfThreeStartsOnTheThirdStillSampleAndAShakeRestartsIt) {
  InekfSourceConfig cfg = shipped_;
  cfg.init.window_ticks = 3;
  auto src = MakeSource(cfg);
  MeasuredRobotState out{};
  for (int t = 1; t <= 2; ++t) {
    EXPECT_FALSE(Step(*src, out)) << t;
    EXPECT_EQ(StateSourceStatus::kNotInitialized, out.status);
    EXPECT_EQ(0U, out.valid_mask & loco::kBaseBit);
    EXPECT_NE(0U, out.valid_mask & loco::kBodyJointsBit);  // the joints are read all the same
  }
  EXPECT_TRUE(Step(*src, out));

  auto shaken = MakeSource(cfg);
  const Eigen::Vector3d still_accel = Accel();
  EXPECT_FALSE(Step(*shaken, out));
  SetImu(Eigen::Vector3d(0.05, 0.0, 0.0), still_accel);  // above the gyro tolerance
  EXPECT_FALSE(Step(*shaken, out));
  SetImu(Eigen::Vector3d::Zero(), still_accel);
  EXPECT_FALSE(Step(*shaken, out));
  EXPECT_FALSE(Step(*shaken, out));
  EXPECT_TRUE(Step(*shaken, out));  // three still samples after the shake
}

TEST_F(InekfSourceTest, MovingJointsOrNoPlannedContactKeepItFromStarting) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  state_.devices[loco_rig::kBody].velocities[3] = 0.06;  // above joint_speed_tolerance
  EXPECT_FALSE(Step(*src, out));
  EXPECT_EQ(StateSourceStatus::kNotInitialized, out.status);
  state_.devices[loco_rig::kBody].velocities[3] = 0.0;
  EXPECT_FALSE(Step(*src, out, 1, 0, /*planned=*/0U));
  EXPECT_TRUE(Step(*src, out));
}

TEST_F(InekfSourceTest, TheFirstStanceComesTheFsmDwellsAfterTheStart) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));  // the start: no step, every slot in swing
  // By the FSM table: swing → candidate after the enter dwell, candidate →
  // stance after max(confirm dwell, 2) more ticks (the first report of an
  // insertion is read two ticks after it).
  const auto ticks = [&](double d) {
    return std::max(1, static_cast<int>(std::ceil(d / shipped_.fsm.period * (1.0 - 1e-12))));
  };
  const int expected =
      ticks(shipped_.fsm.enter_dwell) + std::max(ticks(shipped_.fsm.confirm_dwell), 2);
  int steps = 0;
  while (out.estimator.estimated_mask != kBoth && steps < 100) {
    ASSERT_TRUE(Step(*src, out));
    ++steps;
  }
  EXPECT_EQ(expected, steps);
  RecordProperty("ticks_to_first_stance", steps);
}

// ── hold and gap ────────────────────────────────────────────────────────────

TEST_F(InekfSourceTest, ARepeatedSampleHoldsTheBaseBitForBitThenGoesStale) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  SetImu(Eigen::Vector3d(0.003, -0.002, 0.001), Accel() + Eigen::Vector3d(0.01, 0.0, 0.0));
  for (int t = 0; t < 5; ++t) {
    ASSERT_TRUE(Step(*src, out));
  }
  const MeasuredRobotState before = out;
  const int ch = 0;
  auto& body = state_.devices[loco_rig::kBody];
  const auto a = static_cast<std::size_t>(ctrl_->Indices().body.joint[ch]);
  for (int t = 1; t <= shipped_.policy.max_hold_ticks; ++t) {
    body.positions[ch] += 1e-4;  // the joints still follow the encoders
    ASSERT_TRUE(Step(*src, out, /*k=*/0)) << t;
    EXPECT_EQ(0, out.estimator.periods);
    EXPECT_EQ(0U, out.fresh_mask & loco::kBaseBit) << t;
    EXPECT_NE(0U, out.valid_mask & loco::kBaseBit) << t;
    for (std::size_t i = 0; i < 7; ++i) {
      EXPECT_TRUE(BitsEqual(before.q[i], out.q[i])) << t << " q" << i;
    }
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(before.v[i], out.v[i])) << t << " v" << i;
    }
    EXPECT_EQ(body.positions[ch], out.q[7 + a]);
  }
  EXPECT_FALSE(Step(*src, out, 0));
  EXPECT_EQ(StateSourceStatus::kImuStale, out.status);
  EXPECT_EQ(0U, out.valid_mask & loco::kBaseBit);
  // The samples stopped for longer than the state was held: the next one does
  // not follow the last in time — a reset, then a start on the next still sample.
  const uint32_t epoch = out.state_epoch;
  EXPECT_FALSE(Step(*src, out, 1));
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kGap), out.estimator.reset_reason);
}

TEST_F(InekfSourceTest, AHoldWithinTheBudgetThenANewSampleStepsOnWithoutAReport) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  for (int t = 0; t < 15; ++t) {
    ASSERT_TRUE(Step(*src, out));
  }
  ASSERT_TRUE(out.estimator.report_valid);
  const uint32_t epoch = out.state_epoch;
  for (int t = 1; t <= shipped_.policy.max_hold_ticks; ++t) {
    ASSERT_TRUE(Step(*src, out, 0));
    EXPECT_FALSE(out.estimator.report_valid) << "no update ran on a held tick " << t;
  }
  ASSERT_TRUE(Step(*src, out, 1));
  EXPECT_EQ(1, out.estimator.periods);
  EXPECT_EQ(epoch, out.state_epoch);
  EXPECT_TRUE(out.estimator.report_valid);
  // A missing IMU counts as quiet ticks too: past the budget, a reset on return.
  state_.devices[loco_rig::kBody].imu_valid[0] = false;
  for (int t = 0; t <= shipped_.policy.max_hold_ticks; ++t) {
    EXPECT_FALSE(Step(*src, out, 1));
  }
  state_.devices[loco_rig::kBody].imu_valid[0] = true;
  EXPECT_FALSE(Step(*src, out, 1));
  EXPECT_EQ(epoch + 1, out.state_epoch);
}

TEST_F(InekfSourceTest, MissedSamplesStepOnTheNominalPeriodAndAGapResets) {
  for (const uint64_t k : {2ULL, 5ULL, 10ULL}) {
    MakeStanding();
    auto src = MakeSource(shipped_);
    Oracle o = MakeOracle(shipped_);
    MeasuredRobotState out{};
    ASSERT_TRUE(Step(*src, out));
    const std::vector<double> qd(body_to_full_.size(), 0.0);
    o.Kinematics(BodyQ(out), qd);
    const std::array<Eigen::Vector3d, 2> stance = {o.k[0].position, o.k[1].position};
    ASSERT_EQ(est::InekfStatus::kOk,
              o.seq.Initialize(Gyro(), Accel(), 2e-3, shipped_.init.yaw, stance));
    const Eigen::Vector3d still_accel = Accel();
    for (int t = 0; t < 4; ++t) {
      const uint64_t periods = t == 3 ? k : 1;
      SetImu(Eigen::Vector3d(0.002 * t, -0.001, 0.0005),
             still_accel + Eigen::Vector3d(0, 0.01 * t, 0));
      ASSERT_TRUE(Step(*src, out, periods)) << k << " " << t;
      o.Kinematics(BodyQ(out), qd);
      est::ContactStepInput in;
      in.gyro = Gyro();
      in.accel = Accel();
      in.kinematics = o.k;
      in.joint_velocity = o.rdot;
      in.planned_mask = kBoth;
      in.periods = static_cast<int>(periods);
      ASSERT_EQ(est::ContactFsmStatus::kOk, o.seq.Step(in));
      const est::ContactInekf& a = src->Sequencer().core();
      const est::ContactInekf& b = o.seq.core();
      ExpectBitsEqual(a.rotation().reshaped(), b.rotation().reshaped(), "R");
      ExpectBitsEqual(a.velocity(), b.velocity(), "v");
      ExpectBitsEqual(a.position(), b.position(), "p");
      ExpectBitsEqual(a.covariance().reshaped(), b.covariance().reshaped(), "P");
    }
    EXPECT_EQ(static_cast<int>(k), out.estimator.periods);
  }

  MakeStanding();
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  ASSERT_TRUE(Step(*src, out));
  const uint32_t epoch = out.state_epoch;
  const uint32_t resets = out.estimator.reset_count;
  EXPECT_FALSE(Step(*src, out, 11));  // 22 ms: longer than one step may span
  EXPECT_EQ(StateSourceStatus::kNotInitialized, out.status);
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(resets + 1, out.estimator.reset_count);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kGap), out.estimator.reset_reason);
  EXPECT_FALSE(out.estimator.initialized);
  EXPECT_TRUE(Step(*src, out));  // and starts again on the next still sample
}

TEST_F(InekfSourceTest, ASequenceThatGoesBackIsAMissingSampleAndAReset) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out, 5));
  const uint32_t epoch = out.state_epoch;
  state_.devices[loco_rig::kBody].imu_sequence[0] -= 3;
  EXPECT_FALSE(Step(*src, out, 0));
  EXPECT_EQ(StateSourceStatus::kImuMissing, out.status);
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kGap), out.estimator.reset_reason);
  EXPECT_TRUE(Step(*src, out));
}

TEST_F(InekfSourceTest, BeforeTheStartASequenceThatGoesBackOnlyRestartsTheWindow) {
  InekfSourceConfig cfg = shipped_;
  cfg.init.window_ticks = 3;
  auto src = MakeSource(cfg);
  MeasuredRobotState out{};
  EXPECT_FALSE(Step(*src, out, 5));
  EXPECT_FALSE(Step(*src, out, 1));
  const uint32_t epoch = out.state_epoch;
  state_.devices[loco_rig::kBody].imu_sequence[0] -= 3;
  EXPECT_FALSE(Step(*src, out, 0));
  EXPECT_EQ(StateSourceStatus::kImuMissing, out.status);
  EXPECT_EQ(epoch, out.state_epoch) << "no filter ran: nothing to reset";
  EXPECT_FALSE(Step(*src, out, 1));
  EXPECT_FALSE(Step(*src, out, 1));
  EXPECT_TRUE(Step(*src, out, 1));  // three still samples after the regression
  // A missed sample breaks the window as well.
  auto missed = MakeSource(cfg);
  EXPECT_FALSE(Step(*missed, out, 1));
  EXPECT_FALSE(Step(*missed, out, 2));
  EXPECT_FALSE(Step(*missed, out, 1));
  EXPECT_TRUE(Step(*missed, out, 1));
}

TEST_F(InekfSourceTest, AMissingOrNonFiniteImuIsMissing) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  auto& body = state_.devices[loco_rig::kBody];
  body.imu_valid[0] = false;
  EXPECT_FALSE(Step(*src, out));
  EXPECT_EQ(StateSourceStatus::kImuMissing, out.status);
  body.imu_valid[0] = true;
  body.imu[0][4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Step(*src, out));
  EXPECT_EQ(StateSourceStatus::kImuMissing, out.status);
  body.num_imu_sites = 0;
  EXPECT_FALSE(Step(*src, out));
  EXPECT_EQ(StateSourceStatus::kImuMissing, out.status);
}

// ── joint / IMU coherence ───────────────────────────────────────────────────

TEST_F(InekfSourceTest, SkewBeyondTheBoundSkipsTheUpdateAndBeyondTheInvalidBoundFailsTheState) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  for (int t = 0; t < 15; ++t) {
    ASSERT_TRUE(Step(*src, out));
  }
  ASSERT_EQ(kBoth, out.estimator.estimated_mask) << "both soles in stance";
  const int64_t skew = shipped_.policy.max_joint_imu_skew_ticks * kT;
  const int64_t invalid = shipped_.policy.max_skew_invalid_ticks * kT;

  // No slot used: the update runs on nothing (the sequencer still calls it,
  // every slot not requested), the modes stay.
  const auto none_used = [&](const char* what) {
    for (int c = 0; c < 2; ++c) {
      EXPECT_FALSE(src->Sequencer().action(c).use) << what << " " << c;
      EXPECT_NE(static_cast<uint8_t>(est::ContactUpdateResult::kUsed), out.estimator.update[c])
          << what << " " << c;
    }
  };
  ASSERT_TRUE(Step(*src, out, 1, skew));  // on the bound: a normal update
  ASSERT_TRUE(out.estimator.report_valid);
  EXPECT_EQ(static_cast<uint8_t>(est::ContactUpdateResult::kUsed), out.estimator.update[0]);

  const auto modes = out.estimator.mode;
  ASSERT_TRUE(Step(*src, out, 1, skew + 1));
  none_used("skew + 1");
  EXPECT_EQ(1, out.estimator.periods);  // the IMU still propagated
  EXPECT_EQ(modes, out.estimator.mode);

  ASSERT_TRUE(Step(*src, out, 1, invalid));
  EXPECT_FALSE(Step(*src, out, 1, invalid + 1));
  EXPECT_EQ(StateSourceStatus::kImuSkew, out.status);
  ASSERT_TRUE(Step(*src, out, 1, -skew));  // either order of the receipts
  ASSERT_TRUE(Step(*src, out, 1, -(skew + 1)));
  none_used("-(skew + 1)");
}

// ── reset policy ────────────────────────────────────────────────────────────

TEST_F(InekfSourceTest, NoConfirmedContactForTheLimitResets) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  const int limit = LimitTicks(shipped_.policy.max_no_anchor_s);
  for (int t = 1; t < limit; ++t) {
    ASSERT_TRUE(Step(*src, out, 1, 0, /*planned=*/0U)) << t;
  }
  EXPECT_EQ(limit - 1, out.estimator.no_anchor_ticks);
  const uint32_t epoch = out.state_epoch;
  EXPECT_FALSE(Step(*src, out, 1, 0, 0U));
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kNoAnchor), out.estimator.reset_reason);
}

TEST_F(InekfSourceTest, UpdatesThatGateEveryContactForTheLimitReset) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  for (int t = 0; t < 15; ++t) {
    ASSERT_TRUE(Step(*src, out));
  }
  ASSERT_EQ(kBoth, out.estimator.estimated_mask);
  // Both legs swing forward in one sample, at no joint speed: the anchors are
  // far from where the kinematics now puts the soles, every update gates.
  auto& body = state_.devices[loco_rig::kBody];
  body.positions[Channel("left_hip_pitch_joint")] -= 0.5;
  body.positions[Channel("right_hip_pitch_joint")] -= 0.5;
  const int limit = LimitTicks(shipped_.policy.max_gated_s);
  int last_gated = 0;
  int ticks = 0;
  const uint32_t epoch = out.state_epoch;
  while (Step(*src, out) && ticks < 1000) {
    last_gated = out.estimator.gated_ticks;
    ++ticks;
  }
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kGated), out.estimator.reset_reason);
  EXPECT_EQ(limit - 1, last_gated);
  EXPECT_EQ(limit, ticks + 1) << "gated from the first tick after the swing";
}

TEST_F(InekfSourceTest, AnFsmAndFilterThatDisagreeReset) {
  auto src = MakeSource(shipped_);
  Oracle o = MakeOracle(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  ASSERT_TRUE(Step(*src, out, 1, 0, 0U));
  o.Kinematics(BodyQ(out), std::vector<double>(body_to_full_.size(), 0.0));
  // An anchor the FSM did not insert.
  ASSERT_EQ(est::InekfStatus::kOk,
            est::ContactFsmTestAccess::Core(Access::Seq(*src)).InsertContact(0, o.k[0]));
  const uint32_t epoch = out.state_epoch;
  EXPECT_FALSE(Step(*src, out, 1, 0, 0U));
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kStateMismatch), out.estimator.reset_reason);
}

TEST_F(InekfSourceTest, RefusedStepsInARowResetAndHoldTheBaseUntilThen) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  ASSERT_TRUE(Step(*src, out));
  const MeasuredRobotState before = out;
  const Eigen::Vector3d accel = Accel();
  SetImu(Eigen::Vector3d(400.0, 0.0, 0.0), accel);  // ‖ω‖·T past the core's rotation bound
  const int n = shipped_.policy.max_refused_ticks;
  for (int t = 1; t < n; ++t) {
    ASSERT_TRUE(Step(*src, out)) << t;
    EXPECT_EQ(static_cast<uint8_t>(est::ContactFsmStatus::kPropagateRefused),
              out.estimator.step_status);
    EXPECT_EQ(t, out.estimator.refused_ticks);
    EXPECT_EQ(t, out.estimator.periods) << "a refused sample is not consumed";
    EXPECT_EQ(0U, out.fresh_mask & loco::kBaseBit);
    for (std::size_t i = 0; i < 7; ++i) {
      EXPECT_TRUE(BitsEqual(before.q[i], out.q[i])) << t << " q" << i;
    }
  }
  const uint32_t epoch = out.state_epoch;
  EXPECT_FALSE(Step(*src, out));
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kRefused), out.estimator.reset_reason);
}

TEST_F(InekfSourceTest, TwoCausesOnOneTickRecordTheFirstOfThePriority) {
  auto src = MakeSource(shipped_);
  Oracle o = MakeOracle(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  const int limit = LimitTicks(shipped_.policy.max_no_anchor_s);
  for (int t = 1; t < limit; ++t) {
    ASSERT_TRUE(Step(*src, out, 1, 0, 0U));
  }
  ASSERT_EQ(limit - 1, out.estimator.no_anchor_ticks);
  o.Kinematics(BodyQ(out), std::vector<double>(body_to_full_.size(), 0.0));
  ASSERT_EQ(est::InekfStatus::kOk,
            est::ContactFsmTestAccess::Core(Access::Seq(*src)).InsertContact(1, o.k[1]));
  const uint32_t resets = out.estimator.reset_count;
  EXPECT_FALSE(Step(*src, out, 1, 0, 0U));  // a mismatch, and the no-anchor limit reached
  EXPECT_EQ(resets + 1, out.estimator.reset_count);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kStateMismatch), out.estimator.reset_reason);
}

TEST_F(InekfSourceTest, ResetIsRequestedBumpsTheEpochAndForgetsTheSequence) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  const uint32_t epoch = out.state_epoch;
  src->Reset();
  // A sequence far ahead of the last one read: the first sample after Reset()
  // is a start, not a gap.
  ASSERT_TRUE(Step(*src, out, 1000));
  EXPECT_EQ(epoch + 1, out.state_epoch);
  EXPECT_EQ(static_cast<uint8_t>(EstimatorResetReason::kRequested), out.estimator.reset_reason);
}

TEST_F(InekfSourceTest, StaleJointsKeepTheirKinematicsOutOfTheUpdate) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  for (int t = 0; t < 15; ++t) {
    ASSERT_TRUE(Step(*src, out));
  }
  ASSERT_TRUE(out.estimator.report_valid);
  // Joints and IMU received together (no skew), both older than the body age limit.
  now_ += kT;
  auto& body = state_.devices[loco_rig::kBody];
  body.imu_sequence[0] += 1;
  body.imu_recv_steady_ns[0] = now_;
  body.joint_recv_steady_ns = now_;
  EXPECT_FALSE(src->Update(state_, ctrl_->Indices(), {kBoth}, now_ + 11'000'000, out));
  EXPECT_EQ(StateSourceStatus::kBodyStale, out.status);
  for (int c = 0; c < 2; ++c) {
    EXPECT_FALSE(src->Sequencer().action(c).use) << c;
  }
}

TEST_F(InekfSourceTest, TheQuaternionKeepsTheHemisphereOfTheLastOutput) {
  auto src = MakeSource(shipped_);
  MeasuredRobotState out{};
  ASSERT_TRUE(Step(*src, out));
  auto& core = est::ContactFsmTestAccess::Core(Access::Seq(*src));
  const auto yaw = [](double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).matrix(); };
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  // −100° → −130° crosses the trace sign: Eigen's conversion flips q to −q there.
  rtc::estimation::ContactInekfTestAccess::SetMean(core, yaw(-100.0 * M_PI / 180.0), zero, zero,
                                                   zero);
  ASSERT_TRUE(Step(*src, out, 0));
  const Eigen::Vector4d q1(out.q[3], out.q[4], out.q[5], out.q[6]);
  rtc::estimation::ContactInekfTestAccess::SetMean(core, yaw(-130.0 * M_PI / 180.0), zero, zero,
                                                   zero);
  ASSERT_TRUE(Step(*src, out, 0));
  const Eigen::Vector4d q2(out.q[3], out.q[4], out.q[5], out.q[6]);
  EXPECT_GT(q1.dot(q2), 0.0) << q1.transpose() << " | " << q2.transpose();
}

// ── kinematics and output map oracles ───────────────────────────────────────

TEST_F(InekfSourceTest, ImuFrameKinematicsMatchTheFullModelAndItsFiniteDifference) {
  // The shipped IMU frame (on the root, not rotated) and one rotated on the root.
  pinocchio::Model full_rot = ctrl_->FullModel();
  pinocchio::Model reduced_rot = *reduced_;
  const pinocchio::SE3 X(
      Eigen::Quaterniond(Eigen::Vector4d(0.3, -0.5, 0.2, 0.78)).normalized().toRotationMatrix(),
      Eigen::Vector3d(0.05, -0.02, 0.1));
  full_rot.addFrame(pinocchio::Frame("imu_rot", 1, 0, X, pinocchio::OP_FRAME));
  reduced_rot.addFrame(pinocchio::Frame("imu_rot", 1, 0, X, pinocchio::OP_FRAME));

  struct Variant {
    const pinocchio::Model* full;
    std::shared_ptr<const pinocchio::Model> reduced;
    std::string frame;
  };

  const std::vector<Variant> variants = {
      {&ctrl_->FullModel(), reduced_, kImuFrame},
      {&full_rot, std::make_shared<const pinocchio::Model>(reduced_rot), "imu_rot"}};
  for (const Variant& variant : variants) {
    const pinocchio::Model& full = *variant.full;
    pinocchio::Data data(full);
    ImuFrameKinematics kin(variant.reduced, kSoles, variant.frame);
    const int nj = kin.num_joints();
    std::mt19937_64 rng(20261119U);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::array<est::ContactKinematics, 2> k{};
    std::array<Eigen::Vector3d, 2> rdot{};
    for (auto& x : k) {
      x.jacobian.resize(3, nj);
    }
    const pinocchio::FrameIndex imu = full.getFrameId(variant.frame);
    Eigen::MatrixXd J(6, full.nv);
    for (int sample = 0; sample < 50; ++sample) {
      Eigen::VectorXd q = pinocchio::randomConfiguration(full, -Eigen::VectorXd::Ones(full.nq),
                                                         Eigen::VectorXd::Ones(full.nq));
      for (int a = 0; a < full.nq - 7; ++a) {
        const double lo = std::max(-2.0, full.lowerPositionLimit(7 + a));
        const double hi = std::min(2.0, full.upperPositionLimit(7 + a));
        q(7 + a) = lo + (hi - lo) * 0.5 * (u(rng) + 1.0);
      }
      // Two base poses per sample: the result may not depend on the base.
      for (int pose = 0; pose < 2; ++pose) {
        const Eigen::Quaterniond rot =
            Eigen::Quaterniond(Eigen::Vector4d(u(rng), u(rng), u(rng), u(rng))).normalized();
        q.head<3>() = Eigen::Vector3d(u(rng), u(rng), u(rng));
        q.segment<4>(3) = rot.coeffs();
        std::vector<double> qb;
        std::vector<double> qd;
        for (const int a : body_to_full_) {
          qb.push_back(q(7 + a));
          qd.push_back(u(rng));
        }
        kin.Compute(qb, qd, k, rdot);
        pinocchio::computeJointJacobians(full, data, q);
        pinocchio::updateFramePlacements(full, data);
        const Eigen::Matrix3d R_WI = data.oMf[imu].rotation();
        const Eigen::Vector3d p_WI = data.oMf[imu].translation();
        for (std::size_t c = 0; c < 2; ++c) {
          const pinocchio::FrameIndex f = full.getFrameId(kSoles[c]);
          const Eigen::Vector3d r = R_WI.transpose() * (data.oMf[f].translation() - p_WI);
          J.setZero();
          pinocchio::getFrameJacobian(full, data, f, pinocchio::LOCAL_WORLD_ALIGNED, J);
          for (int i = 0; i < 3; ++i) {
            ExpectClose(k[c].position(i), r(i), 1e-12, "r");
          }
          Eigen::Vector3d rd = Eigen::Vector3d::Zero();
          for (int j = 0; j < nj; ++j) {
            const Eigen::Vector3d col =
                R_WI.transpose() * J.block<3, 1>(0, 6 + body_to_full_[static_cast<std::size_t>(j)]);
            rd += col * qd[static_cast<std::size_t>(j)];
            for (int i = 0; i < 3; ++i) {
              ExpectClose(k[c].jacobian(i, j), col(i), 1e-12, "J");
            }
          }
          for (int i = 0; i < 3; ++i) {
            ExpectClose(rdot[c](i), rd(i), 1e-12, "rdot");
          }
        }
      }
      // J against the central difference of r (h = 1e-6).
      std::vector<double> qb;
      for (const int a : body_to_full_) {
        qb.push_back(q(7 + a));
      }
      const std::vector<double> qd0(static_cast<std::size_t>(nj), 0.0);
      kin.Compute(qb, qd0, k, rdot);
      const auto jac = k;
      std::array<est::ContactKinematics, 2> kp{};
      std::array<est::ContactKinematics, 2> km{};
      for (auto& x : kp) {
        x.jacobian.resize(3, nj);
      }
      for (auto& x : km) {
        x.jacobian.resize(3, nj);
      }
      for (int j = 0; j < nj; ++j) {
        auto qp = qb;
        auto qm = qb;
        qp[static_cast<std::size_t>(j)] += 1e-6;
        qm[static_cast<std::size_t>(j)] -= 1e-6;
        kin.Compute(qp, qd0, kp, rdot);
        kin.Compute(qm, qd0, km, rdot);
        for (std::size_t c = 0; c < 2; ++c) {
          const Eigen::Vector3d fd = (kp[c].position - km[c].position) / 2e-6;
          for (int i = 0; i < 3; ++i) {
            EXPECT_LE(std::abs(fd(i) - jac[c].jacobian(i, j)), 1e-6) << sample << " " << j;
          }
        }
      }
    }
  }
}

TEST_F(InekfSourceTest, TheOutputIsTheBaseOfTheImuStateByTheHandWrittenMap) {
  std::mt19937_64 rng(20261119U);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int sample = 0; sample < 100; ++sample) {
    // An IMU frame on the root at a random pose: the extrinsic.
    pinocchio::Model m = *reduced_;
    const Eigen::Quaterniond qbi =
        Eigen::Quaterniond(Eigen::Vector4d(u(rng), u(rng), u(rng), u(rng))).normalized();
    const pinocchio::SE3 X(qbi.toRotationMatrix(), Eigen::Vector3d(u(rng), u(rng), u(rng)) * 0.2);
    m.addFrame(pinocchio::Frame("imu_oracle", 1, 0, X, pinocchio::OP_FRAME));
    auto model = std::make_shared<const pinocchio::Model>(m);
    MakeStanding();
    // A still sample in that frame: gravity's reaction rotated into it.
    SetImu(Eigen::Vector3d::Zero(),
           qbi.toRotationMatrix().transpose() * Eigen::Vector3d(0.0, 0.0, 9.81));
    auto src = MakeSource(shipped_, model, "imu_oracle");
    MeasuredRobotState out{};
    ASSERT_TRUE(Step(*src, out)) << sample << " " << loco::ToString(out.status);

    const Eigen::Matrix3d R_WI = Eigen::Quaterniond(Eigen::Vector4d(u(rng), u(rng), u(rng), u(rng)))
                                     .normalized()
                                     .toRotationMatrix();
    const Eigen::Vector3d p(u(rng), u(rng), u(rng));
    const Eigen::Vector3d v(u(rng), u(rng), u(rng));
    const Eigen::Vector3d w(u(rng), u(rng), u(rng));
    rtc::estimation::ContactInekfTestAccess::SetMean(
        est::ContactFsmTestAccess::Core(Access::Seq(*src)), R_WI, v, p, w);
    ASSERT_TRUE(Step(*src, out, /*k=*/0));  // a hold: the output of the state as set

    const Eigen::Matrix3d R_BI = X.rotation();
    const Eigen::Vector3d p_BI = X.translation();
    const Eigen::Matrix3d R_WB = R_WI * R_BI.transpose();
    const Eigen::Vector3d p_WB = p - R_WB * p_BI;
    const Eigen::Vector3d p_IB = -R_BI.transpose() * p_BI;
    const Eigen::Vector3d v_WB = v + R_WI * w.cross(p_IB);
    const Eigen::Vector3d v_B = R_WB.transpose() * v_WB;
    const Eigen::Vector3d w_B = R_BI * w;
    const Eigen::Quaterniond qo(out.q[6], out.q[3], out.q[4], out.q[5]);
    EXPECT_NEAR(1.0, qo.norm(), 1e-15);
    const Eigen::Matrix3d R_out = qo.toRotationMatrix();
    for (int i = 0; i < 3; ++i) {
      ExpectClose(out.q[static_cast<std::size_t>(i)], p_WB(i), 1e-12, "p_WB");
      ExpectClose(out.v[static_cast<std::size_t>(i)], v_B(i), 1e-12, "v_B");
      ExpectClose(out.v[3 + static_cast<std::size_t>(i)], w_B(i), 1e-12, "w_B");
      for (int j = 0; j < 3; ++j) {
        ExpectClose(R_out(i, j), R_WB(i, j), 1e-12, "R_WB");
      }
    }
    const auto& idx = ctrl_->Indices();
    const auto& body = state_.devices[loco_rig::kBody];
    for (int c = 0; c < idx.body.num_channels; ++c) {
      const auto a = static_cast<std::size_t>(idx.body.joint[static_cast<std::size_t>(c)]);
      EXPECT_TRUE(BitsEqual(body.positions[static_cast<std::size_t>(c)], out.q[7 + a]));
      EXPECT_TRUE(BitsEqual(body.velocities[static_cast<std::size_t>(c)], out.v[6 + a]));
    }
  }
  // σ per sample → the core's density, bit for bit.
  const double root_t = std::sqrt(shipped_.imu.sample_period_s);
  EXPECT_TRUE(BitsEqual(shipped_.imu.gyro_sigma * root_t, shipped_.core.gyro_noise_density));
  EXPECT_TRUE(BitsEqual(shipped_.imu.accel_sigma * root_t, shipped_.core.accel_noise_density));
  EXPECT_TRUE(BitsEqual(shipped_.imu.gyro_bias_walk, shipped_.core.gyro_bias_walk));
  EXPECT_TRUE(BitsEqual(shipped_.imu.accel_bias_walk, shipped_.core.accel_bias_walk));
  EXPECT_EQ(shipped_.imu.sample_period_s, shipped_.fsm.period);
}

// ── determinism ─────────────────────────────────────────────────────────────

uint64_t Fnv(uint64_t h, const void* data, std::size_t n) {
  const auto* b = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h = (h ^ b[i]) * 1099511628211ULL;
  }
  return h;
}

uint64_t Digest(const MeasuredRobotState& s) {
  uint64_t h = 1469598103934665603ULL;
  h = Fnv(h, s.q.data(), sizeof(double) * static_cast<std::size_t>(s.nq));
  h = Fnv(h, s.v.data(), sizeof(double) * static_cast<std::size_t>(s.nv));
  const auto& e = s.estimator;
  for (const auto* p : {&e.evidence, &e.speed, &e.m2}) {
    h = Fnv(h, p->data(), sizeof(double) * p->size());
  }
  h = Fnv(h, e.p_diag.data(), sizeof(double) * e.p_diag.size());
  h = Fnv(h, e.mode.data(), e.mode.size());
  h = Fnv(h, e.reason.data(), e.reason.size());
  h = Fnv(h, e.update.data(), e.update.size());
  for (const uint64_t x :
       {static_cast<uint64_t>(s.status), static_cast<uint64_t>(s.state_epoch),
        static_cast<uint64_t>(s.fresh_mask), static_cast<uint64_t>(s.valid_mask),
        static_cast<uint64_t>(e.reset_count), static_cast<uint64_t>(e.reset_reason),
        static_cast<uint64_t>(e.periods), static_cast<uint64_t>(e.estimated_mask),
        static_cast<uint64_t>(e.gated_ticks), static_cast<uint64_t>(e.no_anchor_ticks)}) {
    h = Fnv(h, &x, sizeof(x));
  }
  return h;
}

TEST_F(InekfSourceTest, TheSameInputsGiveTheSameOutputsBitForBit) {
  std::vector<int64_t> update_ns;
  update_ns.reserve(2000);
  const auto run = [&]() {
    MakeStanding();
    now_ = 1'000'000'000'000;
    auto src = MakeSource(shipped_);
    std::mt19937_64 rng(20261119U);
    std::normal_distribution<double> n(0.0, 1.0);
    const Eigen::Vector3d g = Accel();
    std::vector<uint64_t> digests;
    MeasuredRobotState out{};
    update_ns.clear();
    int valid = 0;
    for (int t = 0; t < 2000; ++t) {
      SetImu(Eigen::Vector3d(n(rng), n(rng), n(rng)) * 5e-4,
             g + Eigen::Vector3d(n(rng), n(rng), n(rng)) * 1e-2);
      auto& body = state_.devices[loco_rig::kBody];
      for (int c = 0; c < body.num_channels; ++c) {
        body.positions[static_cast<std::size_t>(c)] += 1e-5 * n(rng);
      }
      const uint64_t k = (t % 97 == 50) ? 0 : (t % 89 == 40 ? 3 : 1);
      const uint32_t planned = (t / 300) % 2 == 0 ? kBoth : 0b01U;
      const auto t0 = std::chrono::steady_clock::now();
      valid += Step(*src, out, k, 0, planned) ? 1 : 0;
      update_ns.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now() - t0)
                              .count());
      digests.push_back(Digest(out));
    }
    EXPECT_GT(valid, 1000);
    return digests;
  };
  const auto a = run();
  const auto b = run();
  // Recorded, not judged (the time budget is the control PCs'): Update() on this host.
  std::sort(update_ns.begin(), update_ns.end());
  RecordProperty("update_p50_ns", static_cast<int>(update_ns[update_ns.size() / 2]));
  RecordProperty("update_p99_ns", static_cast<int>(update_ns[update_ns.size() * 99 / 100]));
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t t = 0; t < a.size(); ++t) {
    ASSERT_EQ(a[t], b[t]) << t;
  }
}

// ── the recorded inputs ─────────────────────────────────────────────────────
//
// Both sources record the estimator's inputs through one reading
// (ReadImuSite + RecordEstimatorInputs), so a truth-driven run logs what an
// offline replay steps on: the same lanes give the same input fields bit for
// bit, and each one is the value its definition names.

TEST_F(InekfSourceTest, BothSourcesRecordTheSameInputsByTheirDefinition) {
  auto inekf = MakeSource(shipped_);
  loco::TruthStateSource::Params tp;
  tp.body_max_age_ns = 10 * 1'000'000;
  tp.hand_max_age_ns = 50 * 1'000'000;
  tp.num_contacts = 2;
  tp.model_epoch = 3;
  tp.imu_site = 0;
  loco::TruthStateSource truth(tp);
  auto& body = state_.devices[loco_rig::kBody];
  MeasuredRobotState a{};
  MeasuredRobotState b{};

  struct Want {
    bool present;
    bool valid;
    int64_t age_ns;
    int64_t skew_ns;
  };

  // A tick: the IMU sequence advanced by one and stamped at the clock, the
  // joints `joint_after_ns` later.
  const auto stamp = [&](int64_t joint_after_ns) {
    now_ += kT;
    body.imu_sequence[0] += 1;
    body.imu_recv_steady_ns[0] = now_;
    body.joint_recv_steady_ns = now_ + joint_after_ns;
    state_.devices[loco_rig::kHand].joint_recv_steady_ns = now_;
  };
  const auto check = [&](const char* what, const loco::DeviceIndices& idx, int64_t now,
                         const Want& w) {
    SCOPED_TRACE(what);
    static_cast<void>(inekf->Update(state_, idx, {0b111U}, now, a));
    static_cast<void>(truth.Update(state_, idx, {0b111U}, now, b));
    const auto& ea = a.estimator;
    const auto& eb = b.estimator;
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_TRUE(BitsEqual(ea.imu[i], eb.imu[i])) << "imu " << i;
      EXPECT_TRUE(BitsEqual(ea.imu[i], w.present ? body.imu[0][i] : 0.0)) << "imu " << i;
    }
    EXPECT_EQ(ea.imu_valid, eb.imu_valid);
    EXPECT_EQ(ea.imu_sequence, eb.imu_sequence);
    EXPECT_EQ(ea.imu_age_ns, eb.imu_age_ns);
    EXPECT_EQ(ea.skew_ns, eb.skew_ns);
    EXPECT_EQ(ea.planned_mask, eb.planned_mask);
    EXPECT_EQ(w.valid, ea.imu_valid);
    EXPECT_EQ(w.present ? body.imu_sequence[0] : 0U, ea.imu_sequence);
    EXPECT_EQ(w.age_ns, ea.imu_age_ns);
    EXPECT_EQ(w.skew_ns, ea.skew_ns);
    EXPECT_EQ(kBoth, ea.planned_mask) << "the prior, masked to the model's contacts";
    // Truth records the inputs and estimates nothing.
    EXPECT_FALSE(eb.initialized);
    EXPECT_EQ(-1, eb.periods);
    EXPECT_EQ(0U, eb.reset_count);
    EXPECT_EQ(0U, eb.estimated_mask);
    EXPECT_EQ(0U, eb.num_slots);
  };
  const auto& idx = ctrl_->Indices();
  constexpr int64_t kUs = 1'000;
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  stamp(300 * kUs);  // joints after the IMU: positive skew, read at the joints
  check("joints after", idx, now_ + 300 * kUs, {true, true, 300 * kUs, 300 * kUs});
  stamp(-400 * kUs);  // joints before the IMU, read at the IMU
  check("joints before", idx, now_, {true, true, 0, -400 * kUs});
  stamp(0);
  body.imu_recv_steady_ns[0] = now_ + kUs;  // landed after the clock read
  check("stamped after now", idx, now_, {true, true, 0, -kUs});
  body.imu_sequence[0] -= 3;  // inekf's own verdict (missing, a reset); the reading is valid
  check("sequence back", idx, now_, {true, true, 0, -kUs});
  stamp(0);
  body.imu_valid[0] = false;
  check("producer invalid", idx, now_ + 50 * kUs, {true, false, 50 * kUs, 0});
  body.imu_valid[0] = true;
  stamp(0);
  body.imu[0][1] = kNaN;
  check("gyro NaN", idx, now_, {true, false, 0, 0});
  body.imu[0][1] = 0.0;
  stamp(0);
  body.imu[0][5] = std::numeric_limits<double>::infinity();
  check("accel Inf", idx, now_, {true, false, 0, 0});
  body.imu[0][5] = 9.81;
  stamp(0);
  body.imu_recv_steady_ns[0] = 0;
  check("never received", idx, now_, {true, false, -1, 0});
  stamp(0);
  body.num_imu_sites = 0;
  check("no site", idx, now_, {false, false, -1, 0});
  body.num_imu_sites = 2;
  stamp(0);
  loco::DeviceIndices no_body = idx;
  no_body.body.device = -1;
  check("no body device", no_body, now_, {false, false, -1, 0});
}

// ── estimator.inekf ─────────────────────────────────────────────────────────

TEST_F(InekfSourceTest, TheSectionRefusesEveryMalformedValueAndPassesItsNeighbour) {
  const YAML::Node base = cfg_["estimator"]["inekf"];

  struct Case {
    const char* what;
    std::function<void(YAML::Node&)> spoil;
    std::function<void(YAML::Node&)> pass;  // the closest value that is accepted
  };

  const auto set = [](const char* sec, const char* key, const YAML::Node& v) {
    return [=](YAML::Node& n) {
      if (sec[0] == '\0') {
        n[key] = v;
      } else {
        n[sec][key] = v;
      }
    };
  };
  const auto drop = [](const char* sec, const char* key) {
    return [=](YAML::Node& n) { n[sec].remove(key); };
  };
  const auto none = [](YAML::Node&) {};
  const std::vector<Case> cases = {
      {"imu.gyro_sigma missing", drop("imu", "gyro_sigma"), none},
      {"core.kinematic_sigma missing", drop("core", "kinematic_sigma"), none},
      {"fsm.recover_dwell missing", drop("fsm", "recover_dwell"), none},
      {"init.timeout_s missing", drop("init", "timeout_s"), none},
      {"policy.max_gated_s missing", drop("policy", "max_gated_s"), none},
      {"imu_frame missing", [](YAML::Node& n) { n.remove("imu_frame"); }, none},
      {"unknown key", set("core", "extra", YAML::Node(1.0)), none},
      {"unknown section key", set("", "extra", YAML::Node(1.0)), none},
      {"imu.accel_sigma 0", set("imu", "accel_sigma", YAML::Node(0.0)),
       set("imu", "accel_sigma", YAML::Node(1e-9))},
      {"core.gravity negative", set("core", "gravity", YAML::Node(-9.81)),
       set("core", "gravity", YAML::Node(9.81))},
      {"core.contact_gate_chi2 nan", set("core", "contact_gate_chi2", YAML::Load(".nan")),
       set("core", "contact_gate_chi2", YAML::Node(16.27))},
      {"core.prior_yaw_sigma inf", set("core", "prior_yaw_sigma", YAML::Load(".inf")), none},
      {"core.stationary_gyro_tolerance text",
       set("core", "stationary_gyro_tolerance", YAML::Node("small")), none},
      {"fsm enter == slip", set("fsm", "enter_speed", YAML::Node(0.15)),
       set("fsm", "enter_speed", YAML::Node(0.149))},
      {"fsm nis_low > nis_high", set("fsm", "nis_low", YAML::Node(12.0)),
       set("fsm", "nis_low", YAML::Node(11.0))},
      {"fsm s_u < 1", set("fsm", "unknown_noise_scale", YAML::Node(0.99)),
       set("fsm", "unknown_noise_scale", YAML::Node(1.0))},
      {"fsm dwell beyond 1e6 periods", set("fsm", "nis_dwell", YAML::Node(3000.0)),
       set("fsm", "nis_dwell", YAML::Node(1.0))},
      {"imu.sample_period_s past the core's step range",
       set("imu", "sample_period_s", YAML::Node(0.05)),
       set("imu", "sample_period_s", YAML::Node(0.02))},
      {"init.window_ticks 0", set("init", "window_ticks", YAML::Node(0)),
       set("init", "window_ticks", YAML::Node(1))},
      {"init.window_ticks not an integer", set("init", "window_ticks", YAML::Node(1.5)), none},
      {"init.yaw nan", set("init", "yaw", YAML::Load(".nan")),
       set("init", "yaw", YAML::Node(-3.0))},
      {"init.yaw past 2 pi", set("init", "yaw", YAML::Node(6.3)),
       set("init", "yaw", YAML::Node(6.28))},
      {"init.joint_speed_tolerance 0", set("init", "joint_speed_tolerance", YAML::Node(0.0)), none},
      {"policy.max_hold_ticks negative", set("policy", "max_hold_ticks", YAML::Node(-1)),
       set("policy", "max_hold_ticks", YAML::Node(0))},
      {"policy invalid bound below the skew bound",
       set("policy", "max_skew_invalid_ticks", YAML::Node(0)),
       set("policy", "max_skew_invalid_ticks", YAML::Node(1))},
      {"policy.max_refused_ticks 0", set("policy", "max_refused_ticks", YAML::Node(0)),
       set("policy", "max_refused_ticks", YAML::Node(1))},
      {"policy.max_no_anchor_s 0", set("policy", "max_no_anchor_s", YAML::Node(0.0)), none},
      {"imu_frame empty", set("", "imu_frame", YAML::Node("")), none},
      {"ground missing", [](YAML::Node& n) { n.remove("ground"); }, none},
      {"ground.max_gated missing", drop("ground", "max_gated"), none},
      {"ground unknown key", set("ground", "height", YAML::Node(0.0)), none},
      {"ground.enabled not a bool", set("ground", "enabled", YAML::Node("maybe")),
       set("ground", "enabled", YAML::Node(false))},
      {"ground.sigma 0", set("ground", "sigma", YAML::Node(0.0)),
       set("ground", "sigma", YAML::Node(1e-9))},
      // Checked while the pin is off too: its square underflows.
      {"ground.sigma 1e-170", set("ground", "sigma", YAML::Node(1e-170)),
       set("ground", "sigma", YAML::Node(1e-150))},
      {"ground.gate_chi2 nan", set("ground", "gate_chi2", YAML::Load(".nan")),
       set("ground", "gate_chi2", YAML::Node(10.83))},
      {"ground.max_gated 0", set("ground", "max_gated", YAML::Node(0)),
       set("ground", "max_gated", YAML::Node(1))},
      {"ground.max_gated not an integer", set("ground", "max_gated", YAML::Node(1.5)), none},
      {"ground.settle_s 0", set("ground", "settle_s", YAML::Node(0.0)),
       set("ground", "settle_s", YAML::Node(0.002))},
  };
  ASSERT_GE(cases.size(), 20U);
  EXPECT_NO_THROW(static_cast<void>(loco::ParseInekfSourceConfig(base, "inekf")));
  for (const auto& c : cases) {
    YAML::Node bad = YAML::Clone(base);
    c.spoil(bad);
    EXPECT_THROW(static_cast<void>(loco::ParseInekfSourceConfig(bad, "inekf")),
                 std::invalid_argument)
        << c.what;
    YAML::Node good = YAML::Clone(base);
    c.pass(good);
    EXPECT_NO_THROW(static_cast<void>(loco::ParseInekfSourceConfig(good, "inekf"))) << c.what;
  }
  // The source key: inekf needs its section; a misspelt source is refused.
  YAML::Node est = YAML::Clone(cfg_["estimator"]);
  est["source"] = "inekf";
  EXPECT_EQ("inekf", loco::ParseEstimatorConfig(est, "estimator").source);
  est["source"] = "inkef";
  EXPECT_THROW(static_cast<void>(loco::ParseEstimatorConfig(est, "estimator")),
               std::invalid_argument);
  est["source"] = "inekf";
  est.remove("inekf");
  EXPECT_THROW(static_cast<void>(loco::ParseEstimatorConfig(est, "estimator")),
               std::invalid_argument);
  est["source"] = "truth";
  EXPECT_FALSE(loco::ParseEstimatorConfig(est, "estimator").inekf.has_value());
}

TEST_F(InekfSourceTest, TheKinematicsRefuseAJointOfMoreThanOneDof) {
  pinocchio::Model m;
  const auto root =
      m.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "root_joint");
  const auto ball =
      m.addJoint(root, pinocchio::JointModelSpherical(), pinocchio::SE3::Identity(), "ball");
  m.addFrame(pinocchio::Frame("imu", root, 0, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  m.addFrame(pinocchio::Frame("sole", ball, 0, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  auto model = std::make_shared<const pinocchio::Model>(m);
  EXPECT_THROW(ImuFrameKinematics(model, {"sole"}, "imu"), std::invalid_argument);
  pinocchio::Model r;
  const auto root2 =
      r.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "root_joint");
  const auto hinge =
      r.addJoint(root2, pinocchio::JointModelRX(), pinocchio::SE3::Identity(), "hinge");
  r.addFrame(pinocchio::Frame("imu", root2, 0, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  r.addFrame(pinocchio::Frame("sole", hinge, 0, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  EXPECT_NO_THROW(ImuFrameKinematics(std::make_shared<const pinocchio::Model>(r), {"sole"}, "imu"));
}

TEST_F(InekfSourceTest, TheKinematicsRefuseAFrameOffTheRootOrUnknown) {
  EXPECT_THROW(ImuFrameKinematics(reduced_, kSoles, "imu_in_torso"), std::invalid_argument);
  EXPECT_THROW(ImuFrameKinematics(reduced_, kSoles, "no_such_frame"), std::invalid_argument);
  EXPECT_THROW(ImuFrameKinematics(reduced_, {"left_sole", "no_such_sole"}, kImuFrame),
               std::invalid_argument);
  EXPECT_NO_THROW(ImuFrameKinematics(reduced_, kSoles, kImuFrame));
}

}  // namespace
