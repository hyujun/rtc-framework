// ── Golden-vector harness for the joint-space PD core (#236 S1) ─────────────
// This is the first slice of the rtc_controllers pure-library refactor, and it
// establishes the pattern S2–S6 follow. The claim under test is not "the new
// core is correct" but the far stronger "the extraction changed NOTHING" — so
// every comparison is bitwise (rtc::testing::BitsEqual), never a tolerance.
//
// Two oracles, deliberately:
//
//   1. PreExtractionInlineForm — a literal, rename-only copy of
//      joint_pd_controller.cpp:258-284 at 0f8da61c. DURABLE: it outlives the
//      adapter, so it still pins the core after S7 deletes JointPDController.
//
//   2. The live JointPDController itself. TRANSIENT (dies with S7), and its job
//      is the one thing oracle 1 structurally cannot do: catch a CORRELATED
//      transcription error. Oracle 1 and the core were both transcribed from
//      the same source by the same hand; if that reading was wrong, they agree
//      and the existing JointPD tests — EXPECT_NEAR(…, 0.0, 1e-3) and
//      "at least one command is nonzero" — are far too loose to notice.
//
// Oracle 2 also pins something oracle 1 cannot: the TRAJECTORY WIRING. The core
// takes a reference sample, not a generator, so trajectory ownership stays in
// the integration layer (see joint_pd_law.hpp "Why there is no trajectory
// here"). JointPdShim below is a minimal prototype of that binding — it owns
// the JointSpaceTrajectory and replays seed/retarget exactly as the adapter
// does — and the cross-check proves the recipe reproduces the shipped
// controller bit for bit. S7's binding starts from this shim.
//
// Every bitwise suite carries a non-vacuity partner
// (BitwiseComparisonRejectsAReassociatedLaw) proving the comparison can
// actually reject an algebraically equivalent regrouping. Without it the
// equivalence test pins nothing.

#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/joint/joint_pd_law.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/trajectory/joint_space_trajectory.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <random>
#include <span>
#include <string>
#include <vector>

// The RT allocation gate (rtc::testing::ScopedAllocGate, shared with the S2a/S3a
// suites via rtc_controllers/testing/alloc_gate.hpp) counts `operator new`, so it
// sees a std::vector or a header-inline helper. Unlike the task-space cores, this
// one does NOT also arm rtc_base's Eigen tripwire — the joint-space law is
// Eigen-free, so an Eigen allocation sensor here really would be vacuous rather
// than merely redundant.

namespace {

using rtc::CommandType;
using rtc::kMaxDeviceChannels;
using rtc::kMaxRobotDOF;
using rtc::joint::ComputeJointPdCommand;
using rtc::joint::JointPdGainsView;
using rtc::joint::JointPdInputs;
using rtc::testing::BitsEqual;
using rtc::testing::MakeRng;

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 1 — literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

// One random tick's worth of inputs. Fixed-size arrays throughout, so the
// core's defensive span bound can never fire on a draw — which is itself part
// of the claim: if it ever did fire, the core would diverge from this oracle,
// which has no such bound (the pre-extraction code had none either).
struct CoreDraw {
  std::array<double, kMaxRobotDOF> kp{};
  std::array<double, kMaxRobotDOF> kd{};
  bool use_gravity{false};
  bool use_coriolis{false};
  std::array<double, kMaxDeviceChannels> q{};
  std::array<double, kMaxRobotDOF> ref_pos{};
  std::array<double, kMaxRobotDOF> ref_vel{};
  std::array<double, kMaxRobotDOF> gravity{};
  std::array<double, kMaxRobotDOF> coriolis{};
  std::array<double, kMaxRobotDOF> prev_error{};
  double dt{0.002};
  std::size_t nq{6};
  std::size_t nc0{6};
  CommandType cmd{CommandType::kTorque};
};

// EXACTLY the expression as it stood inline in JointPDController::Compute()
// before #236 S1 lifted it into joint_pd_law.hpp
// (joint_pd_controller.cpp:258-284 at 0f8da61c). Rename-only transcription:
//   traj_state.positions → ref_pos      traj_state.velocities → ref_vel
//   dev0.positions       → q            gains.kp/kd           → kp/kd
//   gravity_torques_     → gravity      coriolis_forces_      → coriolis
//   out0.commands        → commands     command_type_         → cmd
// Nothing else moved: same operation order, same association, same branch
// nesting, same unguarded divide by dt.
void PreExtractionInlineForm(const CoreDraw& d, std::span<double> prev_error,
                             std::span<double> commands) {
  const std::size_t nq = d.nq;
  const auto nc0 = static_cast<int>(d.nc0);
  const double dt = d.dt;

  for (std::size_t i = 0; i < nq; ++i) {
    const double e = d.ref_pos[i] - d.q[i];
    const double de = (e - prev_error[i]) / dt;

    commands[i] = d.kp[i] * e + d.kd[i] * de;
    if (d.cmd == CommandType::kTorque) {
      if (d.use_gravity) {
        commands[i] += d.gravity[i];
      }
      if (d.use_coriolis) {
        commands[i] += d.coriolis[i];
      }
    } else {
      commands[i] += d.ref_vel[i];
    }

    prev_error[i] = e;
  }
  for (std::size_t i = nq; i < static_cast<std::size_t>(nc0); ++i) {
    commands[i] = (d.cmd == CommandType::kTorque) ? 0.0 : d.q[i];
  }
}

// Drive the core with the same draw.
void RunCore(const CoreDraw& d, std::span<double> prev_error, std::span<double> commands) {
  ComputeJointPdCommand(
      JointPdGainsView{std::span<const double>(d.kp), std::span<const double>(d.kd), d.use_gravity,
                       d.use_coriolis},
      JointPdInputs{std::span<const double>(d.q), std::span<const double>(d.ref_pos),
                    std::span<const double>(d.ref_vel), std::span<const double>(d.gravity),
                    std::span<const double>(d.coriolis)},
      d.dt, d.nq, d.nc0, d.cmd, prev_error, commands);
}

// Draws sweep the branch space the law actually has: three command lanes, all
// four gravity/Coriolis flag combinations, the nq < nc0 tail, the degenerate
// nq == 0 / nc0 == 0 tick (the F5 "unreadable device" shape), and a dt small
// enough that the finite difference dominates the proportional term.
CoreDraw RandomDraw(std::mt19937& rng, int trial) {
  std::uniform_real_distribution<double> gain(0.5, 500.0);
  std::normal_distribution<double> sig(0.0, 0.5);
  std::uniform_real_distribution<double> big(-50.0, 50.0);

  CoreDraw d;
  for (std::size_t i = 0; i < kMaxRobotDOF; ++i) {
    d.kp[i] = gain(rng);
    d.kd[i] = gain(rng);
    d.ref_pos[i] = sig(rng);
    d.ref_vel[i] = sig(rng);
    d.gravity[i] = big(rng);
    d.coriolis[i] = big(rng);
    d.prev_error[i] = sig(rng);
  }
  for (std::size_t i = 0; i < kMaxDeviceChannels; ++i) {
    d.q[i] = sig(rng);
  }

  switch (trial % 3) {
    case 0:
      d.cmd = CommandType::kTorque;
      break;
    case 1:
      d.cmd = CommandType::kPosition;
      break;
    default:
      d.cmd = CommandType::kPdFeedforward;  // shares the non-torque lane
      break;
  }
  d.use_gravity = (trial / 3) % 2 == 0;
  d.use_coriolis = (trial / 6) % 2 == 0;

  // Bounds: nq ≤ kMaxRobotDOF is the model-DOF cap the adapter enforces at
  // construction; nc0 ≥ nq is the #172 shape that produces the tail.
  std::uniform_int_distribution<int> nq_pick(0, static_cast<int>(kMaxRobotDOF));
  std::uniform_int_distribution<int> tail_pick(0, 8);
  d.nq = static_cast<std::size_t>(nq_pick(rng));
  d.nc0 = std::min(d.nq + static_cast<std::size_t>(tail_pick(rng)),
                   static_cast<std::size_t>(kMaxDeviceChannels));
  if (trial % 17 == 0) {
    d.nq = 0;  // unreadable device — nothing model-backed this tick
    d.nc0 = (trial % 34 == 0) ? 0 : 5;
  }

  // dt spans three decades; the smallest makes (e − e_prev)/dt the dominant
  // term, which is where a reassociation shows up first.
  static constexpr std::array<double, 3> kDts{0.004, 0.001, 0.00025};
  d.dt = kDts[static_cast<std::size_t>(trial) % kDts.size()];
  return d;
}

// ═══════════════════════════════════════════════════════════════════════════
// Oracle 2 — the shim (prototype of the S7 integration-layer binding)
// ═══════════════════════════════════════════════════════════════════════════

// Owns the trajectory (STRUCTURE) and calls the core (ALGORITHM). Mirrors the
// adapter's self-init / retarget / sample-then-advance sequence exactly; any
// divergence here is a divergence the future binding would ship.
class JointPdShim {
 public:
  using Traj = rtc::trajectory::JointSpaceTrajectory<kMaxRobotDOF>;

  // joint_pd_controller.cpp:184-192 — first-tick self-init.
  void SeedHold(std::span<const double> q, std::size_t nq) {
    Traj::State hold_state;
    for (std::size_t i = 0; i < nq; ++i) {
      hold_state.positions[i] = q[i];
      hold_state.velocities[i] = 0.0;
      hold_state.accelerations[i] = 0.0;
    }
    traj_.initialize(hold_state, hold_state, 0.01);
    time_ = 0.0;
    prev_error_ = {};
  }

  // joint_pd_controller.cpp:222-243 — trajectory re-init on a new target.
  void Retarget(std::span<const double> q, std::span<const double> v, std::span<const double> goal,
                double speed, std::size_t nq) {
    Traj::State start_state;
    Traj::State goal_state;

    double max_dist = 0.0;
    for (std::size_t i = 0; i < nq; ++i) {
      start_state.positions[i] = q[i];
      start_state.velocities[i] = v[i];
      start_state.accelerations[i] = 0.0;

      goal_state.positions[i] = goal[i];
      goal_state.velocities[i] = 0.0;
      goal_state.accelerations[i] = 0.0;

      max_dist = std::max(max_dist, std::abs(goal[i] - q[i]));
    }
    const double duration = std::max(0.01, max_dist / speed);
    traj_.initialize(start_state, goal_state, duration);
    time_ = 0.0;
  }

  // joint_pd_controller.cpp:249-250 (sample, then advance) + the core call.
  void Step(const JointPdGainsView& k, std::span<const double> q, std::span<const double> gravity,
            std::span<const double> coriolis, double dt, std::size_t nq, std::size_t nc0,
            CommandType cmd, std::span<double> commands) {
    const auto traj_state = traj_.compute(time_);
    time_ += dt;
    ComputeJointPdCommand(
        k,
        JointPdInputs{q, std::span<const double>(traj_state.positions),
                      std::span<const double>(traj_state.velocities), gravity, coriolis},
        dt, nq, nc0, cmd, std::span<double>(prev_error_), commands);
  }

 private:
  Traj traj_;
  double time_{0.0};
  std::array<double, kMaxRobotDOF> prev_error_{};
};

// planar_3r.urdf has Y-axis joints, so g(q) ≢ 0. serial_6dof.urdf is all-Z and
// would make every gravity assertion here vacuous — the same fixture trap
// test_dof_dynamics.cpp was written to escape.
std::string Planar3r() {
  return rtc::test::TestUrdfPath("planar_3r.urdf");
}

constexpr int kPlanarNv = 3;

rtc::ControllerState MakeState(int nc0, double dt) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nc0;
  state.devices[0].valid = true;
  return state;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Bit-identity against the literal pre-extraction form
// ═══════════════════════════════════════════════════════════════════════════

TEST(JointPdLaw, MatchesThePreExtractionInlineFormBitwise) {
  auto rng = MakeRng();
  for (int trial = 0; trial < 600; ++trial) {
    const CoreDraw d = RandomDraw(rng, trial);

    std::array<double, kMaxDeviceChannels> got_cmd{};
    std::array<double, kMaxDeviceChannels> want_cmd{};
    std::array<double, kMaxRobotDOF> got_prev = d.prev_error;
    std::array<double, kMaxRobotDOF> want_prev = d.prev_error;

    RunCore(d, std::span<double>(got_prev), std::span<double>(got_cmd));
    PreExtractionInlineForm(d, std::span<double>(want_prev), std::span<double>(want_cmd));

    for (std::size_t i = 0; i < kMaxDeviceChannels; ++i) {
      EXPECT_TRUE(BitsEqual(got_cmd[i], want_cmd[i]))
          << "trial " << trial << " lane=" << static_cast<int>(d.cmd) << " nq=" << d.nq
          << " nc0=" << d.nc0 << " channel " << i << ": " << got_cmd[i] << " vs " << want_cmd[i];
    }
    // The error history is an OUTPUT of this law, not just an input — a core
    // that computed the right command while leaving e_prev stale would corrupt
    // every subsequent tick's derivative term.
    for (std::size_t i = 0; i < kMaxRobotDOF; ++i) {
      EXPECT_TRUE(BitsEqual(got_prev[i], want_prev[i]))
          << "trial " << trial << " prev_error channel " << i;
    }
  }
}

// Non-vacuity partner. `kp·e + kd·((e − e_prev)/dt)` and
// `kp·e + (kd/dt)·(e − e_prev)` are the same law algebraically and the obvious
// "hoist the division out of the loop" edit; if bit equality could not tell
// them apart, the test above would be pinning nothing.
TEST(JointPdLaw, BitwiseComparisonRejectsAReassociatedLaw) {
  auto rng = MakeRng();
  int differing_channels = 0;
  int compared = 0;

  for (int trial = 0; trial < 600; ++trial) {
    CoreDraw d = RandomDraw(rng, trial);
    d.cmd = CommandType::kTorque;
    d.use_gravity = false;
    d.use_coriolis = false;  // isolate the PD term

    std::array<double, kMaxDeviceChannels> got_cmd{};
    std::array<double, kMaxRobotDOF> got_prev = d.prev_error;
    RunCore(d, std::span<double>(got_prev), std::span<double>(got_cmd));

    for (std::size_t i = 0; i < d.nq; ++i) {
      const double e = d.ref_pos[i] - d.q[i];
      const double reassociated = d.kp[i] * e + (d.kd[i] / d.dt) * (e - d.prev_error[i]);
      ++compared;
      if (!BitsEqual(got_cmd[i], reassociated))
        ++differing_channels;
      EXPECT_NEAR(got_cmd[i], reassociated, 1e-6 * (1.0 + std::abs(got_cmd[i])))
          << "the mutant must stay algebraically equivalent, else it proves nothing";
    }
  }
  ASSERT_GT(compared, 0) << "no channels compared — the draw generator degenerated";
  EXPECT_GT(differing_channels, 0)
      << "bit comparison cannot distinguish a reassociated law — the equivalence test is vacuous";
}

// ═══════════════════════════════════════════════════════════════════════════
// Cross-check against the live adapter (transient — dies with S7)
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// Drive adapter and shim through the same tick sequence and compare device-0
// commands bit for bit. Returns the largest |command| seen so the caller can
// prove the comparison was not over a field of zeros.
void RunCrossCheck(rtc::JointPDController& ctrl, JointPdShim& shim,
                   const rtc::JointPDController::Gains& g, CommandType cmd, int nc0, double dt,
                   double clamp_limit, double* peak_out) {
  const auto nq = static_cast<std::size_t>(std::min(nc0, kPlanarNv));
  auto state = MakeState(nc0, dt);
  double peak = 0.0;

  const std::array<double, kPlanarNv> goal{0.18, -0.11, 0.07};
  constexpr int kRetargetTick = 3;
  constexpr int kTicks = 24;

  for (int tick = 0; tick < kTicks; ++tick) {
    // A little motion so the finite-difference term is exercised; both sides
    // read the identical ControllerState. Velocity is the position increment
    // over dt, so the state handed to the retarget is self-consistent — and the
    // increment is small enough to keep the adapter's output clamp inert, which
    // the per-channel assertion below verifies rather than assumes.
    constexpr double kStep = 0.0005;
    for (int j = 0; j < nc0; ++j) {
      const auto uj = static_cast<std::size_t>(j);
      state.devices[0].positions[uj] = kStep * static_cast<double>(tick) * (1.0 + 0.3 * j);
      state.devices[0].velocities[uj] = kStep * (1.0 + 0.3 * j) / dt;
    }

    if (tick == kRetargetTick) {
      ctrl.SetDeviceTarget(0, std::span<const double>(goal));
    }

    const auto out = ctrl.Compute(state);

    // The adapter self-inits on its first tick and retargets on the tick that
    // drains the queued target — the shim mirrors that ordering exactly.
    if (tick == 0) {
      shim.SeedHold(std::span<const double>(state.devices[0].positions), nq);
    }
    if (tick == kRetargetTick) {
      shim.Retarget(std::span<const double>(state.devices[0].positions),
                    std::span<const double>(state.devices[0].velocities),
                    std::span<const double>(goal), g.trajectory_speed, nq);
    }

    // gravity_torques() is the value UpdateDynamics cached during the Compute
    // above, so the shim is fed the adapter's own g(q). Coriolis has no
    // accessor and is therefore off here; its branch is covered by oracle 1 and
    // end-to-end by PlanarDynamics.JointPdCoriolisMatchesModelMagnitude.
    const auto gravity = ctrl.gravity_torques();
    std::array<double, kMaxRobotDOF> zero_coriolis{};

    std::array<double, kMaxDeviceChannels> shim_cmd{};
    shim.Step(JointPdGainsView{std::span<const double>(g.kp), std::span<const double>(g.kd),
                               g.enable_gravity_compensation, g.enable_coriolis_compensation},
              std::span<const double>(state.devices[0].positions), std::span<const double>(gravity),
              std::span<const double>(zero_coriolis), dt, nq, static_cast<std::size_t>(nc0), cmd,
              std::span<double>(shim_cmd));

    for (int j = 0; j < nc0; ++j) {
      const auto uj = static_cast<std::size_t>(j);
      const double adapter = out.devices[0].commands[uj];
      // The adapter clamps after the law; the shim does not (clamping is a
      // limits concern owned by the integration layer). Keeping the clamp
      // provably inert is what makes the bitwise comparison meaningful — assert
      // it rather than assume it.
      ASSERT_LT(std::abs(adapter), clamp_limit)
          << "clamp fired at tick " << tick << " ch " << j << " — comparison would be invalid";
      EXPECT_TRUE(BitsEqual(adapter, shim_cmd[uj]))
          << "tick " << tick << " channel " << j << ": adapter " << adapter << " vs shim "
          << shim_cmd[uj];
      peak = std::max(peak, std::abs(adapter));
    }
  }
  *peak_out = peak;
}

}  // namespace

// Torque lane, gravity compensation ON, with a tail (nc0 = 5 > nv = 3).
TEST(JointPdLaw, CoreDrivenShimMatchesTheAdapterBitwiseTorqueLane) {
  rtc::JointPDController ctrl(Planar3r());
  ctrl.OnDeviceConfigsSet();  // populates max_joint_torque_ = kDefaultMaxJointTorque

  rtc::JointPDController::Gains g;
  g.kp.fill(8.0);
  g.kd.fill(0.8);
  g.enable_gravity_compensation = true;
  g.enable_coriolis_compensation = false;
  g.trajectory_speed = 0.3;
  ctrl.set_gains(g);

  JointPdShim shim;
  double peak = 0.0;
  RunCrossCheck(ctrl, shim, g, CommandType::kTorque, /*nc0=*/5, /*dt=*/0.002,
                /*clamp_limit=*/rtc::kDefaultMaxJointTorque, &peak);
  EXPECT_GT(peak, 1e-3) << "every compared command was ~0 — the cross-check is vacuous";
  // g(q) ≢ 0 on planar_3r is what makes the gravity branch observable at all.
  const auto gravity = ctrl.gravity_torques();
  EXPECT_GT(std::abs(gravity[0]) + std::abs(gravity[1]) + std::abs(gravity[2]), 1e-6)
      << "fixture produced zero gravity — the gravity branch was not exercised";
}

// Position lane: reference-velocity feedforward instead of the dynamics terms.
TEST(JointPdLaw, CoreDrivenShimMatchesTheAdapterBitwisePositionLane) {
  rtc::JointPDController ctrl(Planar3r());
  YAML::Node cfg = YAML::Load("command_type: position\n");
  ctrl.LoadConfig(cfg);  // before enabling anything — the mixed-unit check bites otherwise
  ctrl.OnDeviceConfigsSet();

  rtc::JointPDController::Gains g;
  // The position lane clamps to kDefaultMaxJointVelocity = 2.0 rad/s, which the
  // quintic's own feedforward peak (≈1.875·dist/duration) can reach on its own;
  // the slow trajectory_speed stretches the duration to keep it well inside.
  g.kp.fill(2.0);
  g.kd.fill(0.05);
  g.enable_gravity_compensation = false;
  g.enable_coriolis_compensation = false;
  g.trajectory_speed = 0.2;
  ctrl.set_gains(g);

  JointPdShim shim;
  double peak = 0.0;
  RunCrossCheck(ctrl, shim, g, CommandType::kPosition, /*nc0=*/5, /*dt=*/0.002,
                /*clamp_limit=*/rtc::kDefaultMaxJointVelocity, &peak);
  EXPECT_GT(peak, 1e-3) << "every compared command was ~0 — the cross-check is vacuous";
}

// ═══════════════════════════════════════════════════════════════════════════
// Contracts the extraction newly makes explicit
// ═══════════════════════════════════════════════════════════════════════════

// The tail [nq, nc0) is a contract, not a fallback: a fresh zero in a position
// lane commands "go to the origin", which on a real arm is a fall, not a stop.
TEST(JointPdLaw, TailPolicyFollowsTheCommandLane) {
  CoreDraw d;
  d.kp.fill(10.0);
  d.kd.fill(0.0);
  d.nq = 2;
  d.nc0 = 5;
  for (std::size_t i = 0; i < kMaxDeviceChannels; ++i)
    d.q[i] = 0.31 + 0.1 * static_cast<double>(i);

  std::array<double, kMaxDeviceChannels> cmd{};
  std::array<double, kMaxRobotDOF> prev{};

  d.cmd = CommandType::kTorque;
  RunCore(d, std::span<double>(prev), std::span<double>(cmd));
  for (std::size_t i = d.nq; i < d.nc0; ++i)
    EXPECT_TRUE(BitsEqual(cmd[i], 0.0)) << "torque tail channel " << i << " must be passive";

  prev = {};
  d.cmd = CommandType::kPosition;
  RunCore(d, std::span<double>(prev), std::span<double>(cmd));
  for (std::size_t i = d.nq; i < d.nc0; ++i)
    EXPECT_TRUE(BitsEqual(cmd[i], d.q[i]))
        << "position tail channel " << i << " must hold, not return to the origin";
}

// NUM-2: the divide by dt travels with the law. dt ≤ 0 kills the derivative
// term ONLY — skipping the write instead would leave a fresh zero, i.e. the
// same "go to the origin" hazard the tail policy exists to avoid.
TEST(JointPdLaw, NonPositiveDtZeroesOnlyTheDerivativeTerm) {
  CoreDraw d;
  d.kp.fill(4.0);
  d.kd.fill(9.0);
  d.nq = 3;
  d.nc0 = 3;
  d.cmd = CommandType::kTorque;
  for (std::size_t i = 0; i < kMaxRobotDOF; ++i) {
    d.ref_pos[i] = 0.5;
    d.prev_error[i] = -2.0;  // a large stale error the derivative would amplify
  }

  for (const double dt : {0.0, -0.001}) {
    d.dt = dt;
    std::array<double, kMaxDeviceChannels> cmd{};
    std::array<double, kMaxRobotDOF> prev = d.prev_error;
    RunCore(d, std::span<double>(prev), std::span<double>(cmd));
    for (std::size_t i = 0; i < d.nq; ++i) {
      const double e = d.ref_pos[i] - d.q[i];
      EXPECT_TRUE(BitsEqual(cmd[i], d.kp[i] * e))
          << "dt=" << dt << " channel " << i << ": expected the proportional term alone";
      EXPECT_TRUE(BitsEqual(prev[i], e)) << "error history must still advance at dt=" << dt;
    }
  }
}

// A short input span degrades the affected channels to the tail policy instead
// of reading out of range. This bound is additive — on a correctly sized caller
// it never fires, which is why the bit-identity suites above still hold.
TEST(JointPdLaw, ShortDynamicsSpanDegradesToTheTailPolicy) {
  CoreDraw d;
  d.kp.fill(10.0);
  d.kd.fill(0.0);
  d.nq = 4;
  d.nc0 = 4;
  d.cmd = CommandType::kTorque;
  d.use_gravity = true;
  for (std::size_t i = 0; i < kMaxRobotDOF; ++i) {
    d.ref_pos[i] = 0.2;
    d.gravity[i] = 1.0;
  }

  std::array<double, kMaxDeviceChannels> cmd{};
  std::array<double, kMaxRobotDOF> prev{};
  std::array<double, 2> short_gravity{1.0, 1.0};

  ComputeJointPdCommand(
      JointPdGainsView{std::span<const double>(d.kp), std::span<const double>(d.kd), true, false},
      JointPdInputs{std::span<const double>(d.q), std::span<const double>(d.ref_pos),
                    std::span<const double>(d.ref_vel), std::span<const double>(short_gravity),
                    std::span<const double>(d.coriolis)},
      d.dt, d.nq, d.nc0, d.cmd, std::span<double>(prev), std::span<double>(cmd));

  for (std::size_t i = 0; i < 2; ++i)
    EXPECT_TRUE(BitsEqual(cmd[i], d.kp[i] * 0.2 + 1.0)) << "channel " << i << " should be PD + g";
  for (std::size_t i = 2; i < d.nc0; ++i)
    EXPECT_TRUE(BitsEqual(cmd[i], 0.0))
        << "channel " << i << " past the span must fall to the tail";
}

// RT-1: the law runs on the tick. It has no containers and no Eigen, so this
// gate is a structural regression guard — mutation-verified by inserting a
// runtime-sized allocation (a `new`+immediate-`delete` pair would be elided).
TEST(JointPdLaw, IsAllocationFree) {
  auto rng = MakeRng();
  std::array<double, kMaxDeviceChannels> cmd{};
  std::array<double, kMaxRobotDOF> prev{};

  // Warm up outside the gate so nothing lazily allocates inside it.
  const CoreDraw warm = RandomDraw(rng, 0);
  RunCore(warm, std::span<double>(prev), std::span<double>(cmd));

  std::vector<CoreDraw> draws;
  draws.reserve(64);
  for (int trial = 1; trial <= 64; ++trial)
    draws.push_back(RandomDraw(rng, trial));

  std::size_t new_calls = 0;
  {
    // RAII rather than a bare flag: an ASSERT_* added inside the region returns
    // from the test, and a bare disarm line would then never run — leaving
    // counting on for every later test in the binary, where nothing reads it.
    rtc::testing::ScopedAllocGate heap_gate;
    for (const auto& d : draws)
      RunCore(d, std::span<double>(prev), std::span<double>(cmd));
    new_calls = heap_gate.count();
  }

  EXPECT_EQ(new_calls, 0u) << "the PD law allocated on the RT path (RT-1)";
}
