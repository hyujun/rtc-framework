// ── test_contact_wrench_known_load.cpp ───────────────────────────────────────
// #177 crit#4, simulator half: hang a KNOWN external wrench on a grasped free
// body and pin what the fingertip contact lane reports for it.
//
// WHAT THIS PROVES THAT test_contact_wrench.cpp DOES NOT. That file drives a
// pendulum tip into the ground and asserts `force[2] > 0.1` — one axis, one
// sign, no magnitude, and a link frame that happens to equal the world frame,
// so it cannot see a transposed or permuted link transform and it has no
// reference value to be right or wrong about. crit#4 asks for a positive
// control instead: a load whose true value the test chose, carried through the
// solver into the sensor lane, checked as a vector.
//
// THE IDENTITY UNDER TEST. With the object quasi-static, Newton on the object
// says its contact, applied and gravity forces sum to zero:
//
//     Sum_i c_i + W_f + m g = 0,      c_i = force pad i applies TO the object
//
// MuJoCo's netforce contact sensor reports c_i (the "geom1-on-environment"
// convention), and ReadContactWrenches negates it to the ROS env-on-link sign
// before expressing it in the pad's link frame. So the world-frame sum of what
// the lane publishes is
//
//     Sum_i R_i f_i^link = Sum_i (-c_i) = W_f + m g
//
// which is the assertion below. Every term on the right is chosen by the test.
// A dropped or doubled negation, a transposed link rotation, or a component
// swap moves the left side off it.
//
// WHY IT IS THE SUM AND NOT THE PER-PAD FORCES. How the solver splits the load
// between two pads is a contact-mechanics detail this lane does not own and
// must not be pinned to; the sum is what Newton fixes, and the sum is what the
// pull estimator consumes downstream (#167). See the integrated_bringup half,
// test_pull_estimator_known_wrench.cpp, for what that consumer does with it.
// ─────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

#ifndef PINCH_MJCF_PATH
#error "PINCH_MJCF_PATH must be defined by CMake"
#endif

namespace rtc {
namespace {

using namespace std::chrono_literals;

// Squeeze torque on the pad arm [N.m]. Negative swings the pad towards +x,
// into the object (the arm hangs below its hinge — see the fixture). At the
// 0.5 m arm this is roughly 30 N of grip, which leaves Coulomb capacity at the
// two contacts (mu = 1.5) more than an order of magnitude above the tangential
// demand. The exact value does not enter any assertion: the identity under
// test holds for every grip firm enough not to slip.
constexpr double kSqueezeTorqueNm = -15.0;

// The known load. All three components are non-zero and pairwise distinct in
// magnitude, so a swapped or duplicated axis cannot survive; +x drives the
// object INTO the static pad, which is the direction that pad's normal force
// can absorb (see the fixture's header comment).
constexpr std::array<double, 6> kAppliedWrench = {2.0, -5.0, 3.0, 0.0, 0.0, 0.0};

constexpr double kGravityZ = -9.81;

// Quasi-static gate. The identity above drops an `m a` term, so what has to be
// small is the object's ACCELERATION — and the direct evidence for that is a
// reading that has stopped moving, not an object that has stopped moving. Under
// load this fixture settles to a steady creep of a few mm/s through the soft
// contact (measured: 3.8e-3, decaying by 1e-5 over six seconds) while the
// reported wrench holds to four decimals, so gating on velocity alone would
// wait forever for a condition the physics never reaches. The velocity bound
// below is kept only as a coarse "the grasp has not slipped out" check.
constexpr double kMaxCreepQvel = 5.0e-2;
constexpr double kStableForceN = 5.0e-2;
constexpr auto kStableWindow = std::chrono::milliseconds(150);

MuJoCoSimulator::Config MakePinchConfig() {
  MuJoCoSimulator::Config cfg;
  cfg.model_path = PINCH_MJCF_PATH;
  cfg.enable_viewer = false;
  cfg.sync_timeout_ms = 1.0;
  cfg.max_rtf = 0.0;
  cfg.n_substeps = 1;
  cfg.viewer_refresh_rate = 60.0;
  cfg.use_yaml_servo_gains = false;

  JointGroupConfig grp;
  grp.name = "pinch";
  grp.command_joint_names = {"ja"};
  grp.state_joint_names = {"ja"};
  grp.command_topic = "/pinch/cmd";
  grp.state_topic = "/pinch/state";
  grp.sensor_topic = "/pinch/sensors";
  grp.sensor_names = {"auto"};
  grp.is_robot = true;
  grp.contact_wrench.enabled = true;
  grp.contact_wrench.topic_prefix = "/test/pinch_contact";
  grp.contact_wrench.sensor_name_suffixes = {"_contact"};
  grp.contact_wrench.reference_site_suffixes = {"_ft_site"};
  cfg.groups.push_back(grp);
  return cfg;
}

// One sim tick's worth of the quantities the identity needs, all read on the
// sim thread inside the contact-wrench callback. Reading mjData from the test
// thread while SimLoop runs is explicitly unsupported (see GetData()'s contract
// in mujoco_simulator.hpp); the callback fires between ReadContactWrenches and
// the next mj_step, so there it is both safe and consistent with the samples.
struct Snapshot {
  bool populated{false};
  bool all_found{false};
  std::array<double, 3> sum_world{0.0, 0.0, 0.0};   // Sum_i R_i f_i^link
  std::array<double, 3> pad_a_link{0.0, 0.0, 0.0};  // as published, link frame
  double max_object_qvel{0.0};
};

class PinchKnownLoad : public ::testing::Test {
 protected:
  void SetUp() override {
    sim_ = std::make_unique<MuJoCoSimulator>(MakePinchConfig());
    ASSERT_TRUE(sim_->Initialize());
    ASSERT_TRUE(sim_->HasContactWrenches(0));
    ASSERT_EQ(sim_->GetContactWrenchInfos(0).size(), 2U)
        << "fixture must expose exactly the two pad contact sensors";

    object_body_ = sim_->FindBodyId("object");
    ASSERT_GT(object_body_, 0) << "fixture body 'object' not found";
    object_mass_ = sim_->GetModel()->body_mass[object_body_];
    ASSERT_GT(object_mass_, 0.0);
    object_dofadr_ = sim_->GetModel()->body_dofadr[object_body_];
    ASSERT_GE(object_dofadr_, 0);

    sim_->SetContactWrenchCallback(
        0, [this](const std::vector<JointGroup::ContactWrenchInfo>& infos,
                  const std::vector<JointGroup::ContactWrenchSample>& samples) {
          const mjData* d = sim_->GetData();
          if (d == nullptr || infos.size() != samples.size()) {
            return;
          }
          Snapshot snap;
          snap.populated = true;
          snap.all_found = true;
          for (std::size_t i = 0; i < samples.size(); ++i) {
            if (!samples[i].found) {
              snap.all_found = false;
              continue;
            }
            // v_world = R_WB * v_body; ReadContactWrenches took the transpose
            // to get here, so this is exactly its inverse.
            const mjtNum* r = d->xmat + (9 * static_cast<std::ptrdiff_t>(infos[i].body_id));
            const auto& f = samples[i].force;
            snap.sum_world[0] += r[0] * f[0] + r[1] * f[1] + r[2] * f[2];
            snap.sum_world[1] += r[3] * f[0] + r[4] * f[1] + r[5] * f[2];
            snap.sum_world[2] += r[6] * f[0] + r[7] * f[1] + r[8] * f[2];
            if (infos[i].target_name == "pad_a_contact") {
              snap.pad_a_link = f;
            }
          }
          for (int k = 0; k < 6; ++k) {
            snap.max_object_qvel =
                std::max(snap.max_object_qvel, std::abs(d->qvel[object_dofadr_ + k]));
          }
          std::lock_guard lock(snap_mutex_);
          snap_ = snap;
        });
  }

  Snapshot Latest() {
    std::lock_guard lock(snap_mutex_);
    return snap_;
  }

  // Drive the squeeze and spin until the reported wrench has stopped moving
  // with both contacts live. Returns that snapshot; `populated == false` means
  // it timed out, which the caller reports rather than asserting on stale
  // values. Any disturbance — a contact dropping out, a velocity excursion, a
  // step in the reading — restarts the window, so a pass means the reading held
  // still across a whole one, not that it happened to be quiet at one instant.
  Snapshot SettleUnderSqueeze(std::chrono::milliseconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    Snapshot last{};
    Snapshot reference{};
    auto reference_at = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
      sim_->SetCommand(0, {kSqueezeTorqueNm});
      std::this_thread::sleep_for(1ms);
      last = Latest();
      const auto now = std::chrono::steady_clock::now();
      if (!last.populated || !last.all_found || last.max_object_qvel > kMaxCreepQvel) {
        reference.populated = false;
        continue;
      }
      if (!reference.populated) {
        reference = last;
        reference_at = now;
        continue;
      }
      if (now - reference_at < kStableWindow) {
        continue;
      }
      double drift = 0.0;
      for (std::size_t k = 0; k < 3; ++k) {
        drift = std::max(drift, std::abs(last.sum_world[k] - reference.sum_world[k]));
      }
      if (drift < kStableForceN) {
        return last;
      }
      reference = last;
      reference_at = now;
    }
    last.populated = false;
    return last;
  }

  std::unique_ptr<MuJoCoSimulator> sim_;
  std::mutex snap_mutex_;
  Snapshot snap_{};
  int object_body_{-1};
  int object_dofadr_{-1};
  double object_mass_{0.0};
};

// ── 1. The fixture is not accidentally world-aligned ────────────────────────
//
// The whole point of the rotated pad frames is that a link-frame vector and a
// world-frame vector differ. If a later edit drops the quaternions, the main
// test below would still pass while having stopped checking the transform —
// so the fixture's own precondition gets an assertion of its own.
TEST_F(PinchKnownLoad, PadLinkFramesAreNotTheWorldFrame) {
  // One step so mjData carries a forward-kinematics pass; the SimLoop is not
  // running here, which is the only state in which reading mjData from the
  // test thread is allowed.
  sim_->StepForTest();
  const mjModel* m = sim_->GetModel();
  for (const char* body : {"pad_a_link", "pad_b_link"}) {
    const int id = sim_->FindBodyId(body);
    ASSERT_GT(id, 0) << body;
    const mjtNum* r = sim_->GetData()->xmat + (9 * static_cast<std::ptrdiff_t>(id));
    double off_diagonal = 0.0;
    for (int k : {1, 2, 3, 5, 6, 7}) {
      off_diagonal = std::max(off_diagonal, std::abs(static_cast<double>(r[k])));
    }
    EXPECT_GT(off_diagonal, 0.1) << body
                                 << " link frame is (near) world-aligned — the link transform is "
                                 << "no longer under test";
  }
  EXPECT_GT(m->nbody, 0);
}

// ── 2. Grasp with no applied load reports the object's weight ───────────────
//
// The zero-wrench case of the same identity, and the one whose right-hand side
// needs no service call: what the lane reports must sum to m*g, i.e. the pads
// feel the object pressing DOWN on them. A flipped negation in
// ReadContactWrenches makes this +m*g and the grasp look like it is being
// lifted by the object.
TEST_F(PinchKnownLoad, GraspWithNoAppliedLoadReportsTheObjectWeight) {
  sim_->Start();
  const Snapshot s = SettleUnderSqueeze(6000ms);
  sim_->Stop();

  ASSERT_TRUE(s.populated) << "grasp never settled with both contacts live "
                           << "(max|qvel| = " << s.max_object_qvel << ")";

  const double expected_z = object_mass_ * kGravityZ;
  EXPECT_NEAR(s.sum_world[0], 0.0, 0.15);
  EXPECT_NEAR(s.sum_world[1], 0.0, 0.15);
  EXPECT_NEAR(s.sum_world[2], expected_z, 0.15)
      << "expected the pads to carry the object weight (" << expected_z << " N)";
  EXPECT_LT(s.sum_world[2], 0.0) << "sign flip: the lane reports the object lifting the pads";
}

// ── 3. Known external wrench → the sensor lane, as a vector ────────────────
//
// crit#4 proper. The load is chosen by the test, hung on the object through
// the same SetExternalWrenchAtPoint the /sim/set_external_wrench service calls,
// and must reappear as the world-frame sum of the contact lane plus the
// object's weight. Sign, frame and magnitude are all pinned by one comparison.
TEST_F(PinchKnownLoad, KnownExternalWrenchReappearsInTheContactLane) {
  sim_->Start();
  const Snapshot pre = SettleUnderSqueeze(6000ms);
  ASSERT_TRUE(pre.populated) << "grasp never settled before the load was applied";

  ASSERT_TRUE(sim_->SetExternalWrenchAtPoint(object_body_, {0.0, 0.0, 0.0}, kAppliedWrench));
  const Snapshot s = SettleUnderSqueeze(6000ms);
  sim_->Stop();

  ASSERT_TRUE(s.populated) << "grasp never re-settled under the applied load "
                           << "(max|qvel| = " << s.max_object_qvel << ")";

  const std::array<double, 3> expected = {kAppliedWrench[0], kAppliedWrench[1],
                                          kAppliedWrench[2] + object_mass_ * kGravityZ};
  // 2 % of the load's magnitude. Measured residual on this fixture is 0.15 %
  // (worst axis 0.008 N of 5.95 N), so this keeps better than tenfold margin
  // while staying orders of magnitude tighter than any sign or axis error,
  // which move the answer by O(|W|).
  const double tol = 0.02 * std::sqrt(expected[0] * expected[0] + expected[1] * expected[1] +
                                      expected[2] * expected[2]);
  EXPECT_NEAR(s.sum_world[0], expected[0], tol);
  EXPECT_NEAR(s.sum_world[1], expected[1], tol);
  EXPECT_NEAR(s.sum_world[2], expected[2], tol);

  // The load moved the reading by much more than the tolerance in every axis —
  // otherwise the two settles above could be the same measurement and the
  // service call a no-op.
  for (int k = 0; k < 3; ++k) {
    EXPECT_GT(std::abs(s.sum_world[static_cast<std::size_t>(k)] -
                       pre.sum_world[static_cast<std::size_t>(k)]),
              10.0 * tol)
        << "axis " << k << ": applying the wrench barely changed the lane";
  }

  // What the lane actually publishes is the LINK-frame vector; if it happened
  // to equal the world-frame one, the rotation under test would be identity.
  const double link_vs_world = std::abs(s.pad_a_link[0] - s.sum_world[0]) +
                               std::abs(s.pad_a_link[1] - s.sum_world[1]) +
                               std::abs(s.pad_a_link[2] - s.sum_world[2]);
  EXPECT_GT(link_vs_world, tol) << "published link-frame force coincides with the world sum";
}

}  // namespace
}  // namespace rtc
