// The MPC seams of the locomotion_controller tests: the controller's test
// access, a stub handler whose plans are the measured state at rest, and the
// installed configuration with the MPC on. Header-only, shared by the MPC tests
// and the allocation gate.
#ifndef INTEGRATED_BRINGUP_TEST_LOCOMOTION_MPC_STUB_HPP_
#define INTEGRATED_BRINGUP_TEST_LOCOMOTION_MPC_STUB_HPP_

#include "locomotion_controller_rig.hpp"
#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace integrated_bringup {

// The seams the tests need into the controller's MPC.
struct LocomotionControllerTestAccess {
  static bool Spawn(LocomotionController& c, std::unique_ptr<rtc::mpc::MPCHandlerBase> h) {
    return c.SpawnMpcThread(std::move(h));
  }

  static std::uint32_t StateEpoch(const LocomotionController& c) { return c.mpc_state_epoch_; }

  static const rtc::tsid::ContactState& Contacts(const LocomotionController& c) {
    return c.contacts_;
  }

  /// The controller's model epoch moves on; the source keeps reporting the old
  /// one — the next tick's state is "of another model" (kModelEpoch).
  static void BumpModelEpoch(LocomotionController& c) { ++c.model_epoch_; }

  /// The packed E-STOP word (bit 0 active, the rest the edge count).
  static std::uint32_t EstopWord(const LocomotionController& c) {
    return c.estop_state_.load(std::memory_order_acquire);
  }

  /// The body channel whose joint is the reduced model's first (the stub's
  /// joint_offset moves that joint of the plan).
  static int ChannelOfFirstReducedJoint(const LocomotionController& c) {
    const auto& body = c.Indices().body;
    for (int k = 0; k < body.num_channels; ++k) {
      if (body.joint[static_cast<std::size_t>(k)] == c.reduced_to_full_.front()) {
        return k;
      }
    }
    return -1;
  }

  /// Reduced (body) joint j's slot in the full joint order (what the inekf source is given).
  static const std::vector<int>& ReducedToFull(const LocomotionController& c) {
    return c.reduced_to_full_;
  }

  /// The dynamics model's state the tick handed the WBC and the MPC.
  static const rtc::tsid::ControlState& QpState(const LocomotionController& c) {
    return c.qp_state_;
  }

  /// Guard refusals before kPoseGuard latches (round(0.5 s · control rate)).
  static std::uint32_t GuardSettleTicks(const LocomotionController& c) {
    return c.guard_settle_ticks_;
  }
};

}  // namespace integrated_bringup

namespace loco_rig {

constexpr int kNodes = 32;
constexpr double kNodeDt = 0.035;
constexpr int kNq = 30;
constexpr int kNv = 29;
constexpr int kNu = 35;
constexpr int kContactVars = 12;

// A handler that solves nothing: each Solve copies the snapshot into a
// standing plan (every node the measured state, at rest, the weight split
// over the two soles) whose flags and faults the test sets.
class StandingStub final : public rtc::mpc::MPCHandlerBase {
 public:
  explicit StandingStub(double weight) : weight_(weight) {
    cfg_.publish_deadline_ms = 0.0;
    cfg_.publish_prim_tol = 0.05;
  }

  rtc::mpc::MPCInitError Init(const rtc::mpc::MPCSolverConfig&, const rtc::mpc::RobotModelHandler&,
                              const rtc::mpc::OCPLimits&,
                              const rtc::mpc::PhaseContext&) noexcept override {
    return rtc::mpc::MPCInitError::kNoError;
  }

  rtc::mpc::MPCSolveError Solve(const rtc::mpc::PhaseContext& ctx,
                                const rtc::mpc::MPCStateSnapshot& state,
                                rtc::mpc::MPCSolution& out) noexcept override {
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_us.load()));
    if (gate.load() >= 0) {  // wait for the test to allow this solve (bounded)
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      while (gate.load() == 0 && std::chrono::steady_clock::now() < end) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (gate.load() > 0) {
        gate.fetch_sub(1);
      }
    }
    {
      const std::lock_guard<std::mutex> lock(snapshot_mutex);
      snapshots[snapshot_count % kSnapshots] = state;
      ++snapshot_count;
    }
    if (ctx.horizon != nullptr) {  // what the phase manager built from the command
      horizon_command_sequence.store(ctx.horizon->command_sequence);
      horizon_base_vx.store(ctx.horizon->stages[0].v[0]);
    }
    if (fail.load()) {
      return rtc::mpc::MPCSolveError::kSolverException;
    }
    out.horizon_length = kNodes;
    out.dt_node = kNodeDt;
    out.nq = kNq;
    out.nv = kNv;
    out.nu = kNu;
    out.nx = kNq + kNv;
    out.ndx = 2 * kNv;
    out.n_contact_vars = kContactVars;
    out.gain_nodes = 0;
    rtc::mpc::MarkSolved(out, converged.load());
    out.initial_solve = initial.load();
    out.cold_start = false;
    out.solve_duration_ns = 1234;
    out.observation_stamp_ns = state.timestamp_ns;
    out.observation_sequence = state.sequence;
    out.observation_source_sequence = state.source_sequence;
    out.model_epoch = state.model_epoch;
    out.state_epoch = state.state_epoch;
    out.horizon_offset_ns = std::llround(ctx.horizon_offset * 1e9);
    const std::uint32_t m = mask.load();
    for (int k = 0; k <= kNodes; ++k) {
      auto& q = out.q_traj[static_cast<std::size_t>(k)];
      for (int i = 0; i < kNq; ++i) {
        q[static_cast<std::size_t>(i)] = state.q[static_cast<std::size_t>(i)];
      }
      q[7] += joint_offset.load();
      q[2] += base_dz.load();
      if (const double tilt = base_tilt.load(); tilt != 0.0) {  // about x, from upright
        q[3] = std::sin(0.5 * tilt);
        q[4] = q[5] = 0.0;
        q[6] = std::cos(0.5 * tilt);
      }
      if (zero_quaternion.load()) {
        q[3] = q[4] = q[5] = q[6] = 0.0;
      }
      out.v_traj[static_cast<std::size_t>(k)].fill(0.0);
      out.v_traj[static_cast<std::size_t>(k)][0] = base_vx.load();
      // The sampler differentiates q and v: a rate flipping each node is a
      // large joint acceleration.
      out.v_traj[static_cast<std::size_t>(k)][6] = (k % 2 == 0 ? 1.0 : -1.0) * joint_rate.load();
      out.node_mode[static_cast<std::size_t>(k)] = m;
      out.node_prim_inf[static_cast<std::size_t>(k)] = 0.0;
    }
    for (int k = 0; k < kNodes; ++k) {
      out.a_ff_traj[static_cast<std::size_t>(k)].fill(0.0);
      out.u_traj[static_cast<std::size_t>(k)].fill(0.0);
      auto& l = out.lambda_traj[static_cast<std::size_t>(k)];
      l.fill(0.0);
      const double fz = fz_scale.load() * weight_;
      const int support = __builtin_popcount(m);
      for (int c = 0; c < 2; ++c) {
        if ((m >> c) & 1U) {
          l[static_cast<std::size_t>(6 * c + 2)] = fz / support;
        }
      }
      l[2] -= pull.load();  // moved from the first contact to the second
      l[8] += pull.load();
    }
    return rtc::mpc::MPCSolveError::kNoError;
  }

  [[nodiscard]] bool Initialised() const noexcept override { return true; }

  [[nodiscard]] const rtc::mpc::MPCSolverConfig& solver_config() const noexcept override {
    return cfg_;
  }

  [[nodiscard]] bool InitialSolvePending() const noexcept override { return initial.load(); }

  [[nodiscard]] std::string_view ocp_type() const noexcept override { return "kinodynamic"; }

  [[nodiscard]] int horizon_length() const noexcept override { return kNodes; }

  [[nodiscard]] int nq() const noexcept override { return kNq; }

  [[nodiscard]] int nv() const noexcept override { return kNv; }

  [[nodiscard]] int nu() const noexcept override { return kNu; }

  [[nodiscard]] int n_contact_vars() const noexcept override { return kContactVars; }

  void SeedWarmStart(const rtc::mpc::MPCSolution&) noexcept override {}

  std::atomic<int> sleep_us{0};
  std::atomic<bool> converged{true};
  std::atomic<bool> initial{true};
  std::atomic<bool> fail{false};
  std::atomic<std::uint32_t> mask{0b11U};
  std::atomic<double> joint_offset{0.0};
  std::atomic<bool> zero_quaternion{false};
  std::atomic<double> fz_scale{1.0};
  std::atomic<double> base_tilt{0.0};   ///< [rad] the plan's base rolled from upright
  std::atomic<double> base_dz{0.0};     ///< [m] the plan's base raised
  std::atomic<double> base_vx{0.0};     ///< [m/s] the plan's base velocity
  std::atomic<double> pull{0.0};        ///< [N] normal force moved between the soles
  std::atomic<double> joint_rate{0.0};  ///< the first joint's rate, flipping sign each node [rad/s]
  /// −1: solve freely; n ≥ 0: n more solves, then each waits (3 s at most)
  /// until the test allows another — one plan at a time, at a known tick.
  /// The MPC thread is its only decrementer; a waiting solve counts as a
  /// deadline miss / overrun, so no test asserts those counters while closed.
  std::atomic<int> gate{-1};
  // Read back from the last Solve's horizon: the command the planner integrated.
  std::atomic<std::uint64_t> horizon_command_sequence{0};
  std::atomic<double> horizon_base_vx{0.0};  ///< node 0 base v_x [m/s]
  // The states the solves were given, the last kSnapshots (snapshot_count total).
  static constexpr std::size_t kSnapshots = 256;
  std::mutex snapshot_mutex;
  std::array<rtc::mpc::MPCStateSnapshot, kSnapshots> snapshots{};
  std::size_t snapshot_count{0};

 private:
  double weight_;
  rtc::mpc::MPCSolverConfig cfg_{};
};

// A stub on `ctrl`'s MPC thread (configured, not yet activated); null on a refusal.
inline StandingStub* SpawnStandingStub(LocomotionController& ctrl) {
  const double weight =
      pinocchio::computeTotalMass(ctrl.FullModel()) * ctrl.FullModel().gravity.linear().norm();
  auto stub = std::make_unique<StandingStub>(weight);
  StandingStub* const raw = stub.get();
  return integrated_bringup::LocomotionControllerTestAccess::Spawn(ctrl, std::move(stub)) ? raw
                                                                                          : nullptr;
}

inline YAML::Node MpcConfig() {
  YAML::Node cfg = YAML::Clone(fx::ShippedControllerNode(kProfile, kKey));
  cfg["mpc"]["enabled"] = true;
  return cfg;
}

}  // namespace loco_rig

#endif  // INTEGRATED_BRINGUP_TEST_LOCOMOTION_MPC_STUB_HPP_
