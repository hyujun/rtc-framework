// ── TaskAdmittanceController — the URDF-bound half of slice 3 (#236) ─────────
// Everything here needs a robot: the §11.4.1 P2/P3 sign contrast against the
// impedance law, the wrench→motion path end to end, the §10.6 staleness
// behaviour, the position-domain E-STOP hold, and the RT allocation gate.
// The model-free numerics (integration scheme, §7.5 guards, DLS algebra) are
// pinned in test_admittance_core.cpp.
//
// Zero-allocation is checked by GLOBAL operator-new interposition: Compute()
// lives in another TU, so a same-TU Eigen guard would observe nothing.
#include "rtc_controllers/direct/task_impedance_controller.hpp"
#include "rtc_controllers/indirect/task_admittance_controller.hpp"
#include "test_urdf_path.hpp"
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <new>
#include <span>
#include <string>
#include <vector>

// ── Allocation counter (global new/delete interposition) ────────────────────
namespace {
thread_local bool g_alloc_active = false;
thread_local std::size_t g_alloc_count = 0;
}  // namespace

void* operator new(std::size_t n) {
  if (g_alloc_active)
    ++g_alloc_count;
  void* p = std::malloc(n != 0 ? n : 1);
  if (p == nullptr)
    throw std::bad_alloc();
  return p;
}

void* operator new[](std::size_t n) {
  return ::operator new(n);
}

void operator delete(void* p) noexcept {
  std::free(p);
}

void operator delete[](void* p) noexcept {
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}

namespace {

using rtc::TaskAdmittanceController;
using rtc::TaskImpedanceController;
using rtc::compliance::ComplianceState;
using rtc::compliance::kWrenchDim;
using rtc::compliance::Wrench6;

constexpr double kDt = 0.002;
constexpr int kNj = 7;

// serial_7dof, NOT serial_6dof: the 6-DoF fixture's joints are all +Z coaxial
// (σ_min ≡ 0), so every σ_min fault would fire before anything under test ran.
std::string Urdf7() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

// A generic, well-conditioned posture — away from the σ_min shell so DEGRADED /
// SAFE_STOP cannot be reached for kinematic reasons in tests about something else.
const std::vector<double>& Posture() {
  static const std::vector<double> q{0.1, -0.5, 0.3, -1.2, 0.4, 0.6, 0.2};
  return q;
}

rtc::ControllerState MakeState(const std::vector<double>& q, const std::vector<double>& qd = {}) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = kDt;
  state.devices[0].num_channels = kNj;
  state.devices[0].valid = true;
  for (int i = 0; i < kNj; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    state.devices[0].positions[ui] = (ui < q.size()) ? q[ui] : 0.0;
    state.devices[0].velocities[ui] = (ui < qd.size()) ? qd[ui] : 0.0;
  }
  return state;
}

void Send(TaskAdmittanceController& c, const Wrench6& w) {
  c.SetExternalWrench(std::span<const double, kWrenchDim>(w.data(), kWrenchDim));
}

void Send(TaskImpedanceController& c, const Wrench6& w) {
  c.SetExternalWrench(std::span<const double, kWrenchDim>(w.data(), kWrenchDim));
}

// A transparent wrench source (no deadband, no filter, no bias average) so tests
// about frames / signs / staleness see the raw value. Each conditioning stage
// has its own test in test_compliance_core.cpp; mixing them in here would only
// make failures ambiguous. activation_ramp_time: 0 removes the §10.7 ramp, which
// would otherwise zero the wrench on exactly the ticks under test.
// `extra` are top-level keys and go BEFORE the block; `wrench_extra` are
// external_wrench members and go inside it (already indented by the caller). A
// second top-level `external_wrench:` mapping would silently replace this one
// rather than merge with it, which is how the first draft of these tests
// "passed" a sensor_frame that was never read.
std::string TransparentYaml(const std::string& extra = {}, const std::string& wrench_extra = {}) {
  return extra + R"(
activation_ramp_time: 0.0
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 0
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)" + wrench_extra;
}

// Perfect-tracking plant: the joint state becomes exactly what the controller
// commanded last tick. Without it the "arm" never moves, so the IK tracking
// error is pinned to the compliant deviation by construction and nothing about
// the §7.3 loop is actually exercised. Tests that deliberately model a STUCK
// arm (the §7.3 windup bound) do not call this.
void ApplyCommand(rtc::ControllerState& state, const rtc::ControllerOutput& out) {
  for (int i = 0; i < kNj; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double q_next = out.devices[0].commands[ui];
    state.devices[0].velocities[ui] = (q_next - state.devices[0].positions[ui]) / kDt;
    state.devices[0].positions[ui] = q_next;
  }
}

// Independent model handle for the reference kinematics a test needs (J, ĝ).
// Deliberately NOT the controller's own — a reference that shares the object
// under test cannot contradict it.
struct Reference {
  rtc_urdf_bridge::PinocchioModelBuilder builder;
  rtc_urdf_bridge::RtModelHandle handle;
  pinocchio::FrameIndex tip;

  static rtc_urdf_bridge::ModelConfig Config() {
    rtc_urdf_bridge::ModelConfig c;
    c.urdf_path = Urdf7();
    c.root_joint_type = "fixed";
    return c;
  }

  Reference() : builder(Config()), handle(builder.GetFullModel()) {
    tip = static_cast<pinocchio::FrameIndex>(builder.GetFullModel()->nframes - 1);
  }

  Eigen::MatrixXd Jacobian(const std::vector<double>& q) {
    handle.ComputeJacobians(std::span<const double>(q.data(), q.size()));
    Eigen::MatrixXd J(6, handle.nv());
    handle.GetFrameJacobian(tip, pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
  }

  Eigen::VectorXd Gravity(const std::vector<double>& q) {
    handle.ComputeGeneralizedGravity(std::span<const double>(q.data(), q.size()));
    return handle.GetGeneralizedGravity();
  }

  Eigen::Vector3d TcpPosition(const std::vector<double>& q) {
    handle.ComputeJacobians(std::span<const double>(q.data(), q.size()));  // runs FK
    return handle.GetFramePlacement(tip).translation();
  }
};

std::vector<double> JointsOf(const rtc::ControllerState& state) {
  std::vector<double> q(kNj);
  for (int i = 0; i < kNj; ++i)
    q[static_cast<std::size_t>(i)] = state.devices[0].positions[static_cast<std::size_t>(i)];
  return q;
}

// ── Basic surface ───────────────────────────────────────────────────────────

TEST(TaskAdmittanceController, EmitsPositionCommands) {
  TaskAdmittanceController c(Urdf7());
  EXPECT_EQ(c.GetCommandType(), rtc::CommandType::kPosition);
  EXPECT_EQ(c.Name(), "TaskAdmittanceController");
  EXPECT_TRUE(c.external_wrench_enabled()) << "§7.1 — axis A != NONE is mandatory here";
  EXPECT_EQ(c.task_dim(), 6);

  auto state = MakeState(Posture());
  const auto out = c.Compute(state);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);
  // Nothing has been pushed yet, so the first command is the measured pose.
  for (int i = 0; i < kNj; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                Posture()[static_cast<std::size_t>(i)], 1e-9);
}

// ── §11.4.1 P2 / P3 — the same push, opposite responses ─────────────────────

TEST(TaskAdmittanceController, AdmittanceFollowsThePushWhileImpedanceResistsIt) {
  // ONE fixture, ONE wrench, ONE projection direction. Both laws are read
  // through d = Jᵀ f̂: the impedance controller's task torque projects NEGATIVE
  // onto it (it pushes back, P2) and the admittance controller's commanded
  // joint velocity projects POSITIVE (it goes with the push, P3) — because
  // q̇ᵀ(Jᵀf̂) = (J q̇)ᵀf̂ is the commanded task motion along the force.
  //
  // A sign flip in either controller breaks this test, and neither can be
  // "fixed" by flipping the other: the two assertions have opposite senses.
  Reference ref;
  const Eigen::MatrixXd J = ref.Jacobian(Posture());
  const Eigen::VectorXd g = ref.Gravity(Posture());

  Wrench6 push{};
  push[0] = 30.0;  // +x in the sensor (= tip body) frame

  // The LWA force the controllers actually see — the sensor frame is the tip
  // body frame, so Ad^{-T} rotates it. Read it from the controller's own
  // diagnostics rather than re-deriving the transform here.
  TaskAdmittanceController adm(Urdf7());
  adm.LoadConfig(YAML::Load(TransparentYaml()));
  auto state = MakeState(Posture());
  Send(adm, push);
  (void)adm.Compute(state);  // seed tick: X_d ← measured, wrench arrives
  Send(adm, push);
  const auto out_adm = adm.Compute(state);
  const auto d_adm = adm.GetDiagnosticsForTesting();
  ASSERT_TRUE(d_adm.wrench_valid);

  Eigen::Matrix<double, 6, 1> f_lwa;
  for (int i = 0; i < 6; ++i)
    f_lwa(i) = d_adm.wrench_lwa[static_cast<std::size_t>(i)];
  ASSERT_GT(f_lwa.norm(), 1.0);
  const Eigen::VectorXd d = J.transpose() * (f_lwa / f_lwa.norm());

  Eigen::VectorXd qdot(kNj);
  for (int i = 0; i < kNj; ++i)
    qdot(i) = out_adm.devices[0].target_velocities[static_cast<std::size_t>(i)];
  EXPECT_GT(qdot.dot(d), 0.0) << "P3 violated: admittance is resisting the push, not following it";

  // §6.3 with a heavy Λ_d makes the impedance law's wrench term dominate at
  // e = 0, which is the P2 condition ("push the robot, watch it push back").
  TaskImpedanceController imp(Urdf7(), TaskImpedanceController::Gains{});
  imp.LoadConfig(YAML::Load(TransparentYaml(R"(formulation: inertia_shaping
desired_inertia: [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]
max_inertia_ratio: 100.0
)")));
  Send(imp, push);
  (void)imp.Compute(state);
  Send(imp, push);
  const auto out_imp = imp.Compute(state);
  ASSERT_TRUE(imp.GetDiagnosticsForTesting().control_valid);

  Eigen::VectorXd tau_task(kNj);
  for (int i = 0; i < kNj; ++i)
    tau_task(i) = out_imp.devices[0].commands[static_cast<std::size_t>(i)] - g(i);
  EXPECT_LT(tau_task.dot(d), 0.0)
      << "P2 violated: impedance is yielding to the push, not resisting";
}

// ── §7.2 → §7.3 end to end ──────────────────────────────────────────────────

TEST(TaskAdmittanceController, SustainedWrenchMovesTheCompliantFrameAndTheArmFollowsIt) {
  Reference ref;
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));
  auto state = MakeState(Posture());

  // Closed loop: the arm actually follows the command, so the §7.3 IK is on the
  // hook for the tracking error rather than it being fixed by construction.
  const Eigen::Vector3d tcp_start = ref.TcpPosition(Posture());
  for (int k = 0; k < 3000; ++k) {
    Send(c, Wrench6{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    ApplyCommand(state, c.Compute(state));
  }
  const auto d = c.GetDiagnosticsForTesting();
  ASSERT_TRUE(d.control_valid);
  EXPECT_EQ(d.state, static_cast<std::uint8_t>(ComplianceState::kRunningContact))
      << "25 N is well past the 5 N contact threshold";

  const Eigen::Vector3d dev(d.compliant_deviation[0], d.compliant_deviation[1],
                            d.compliant_deviation[2]);
  const Eigen::Vector3d f(d.wrench_lwa[0], d.wrench_lwa[1], d.wrench_lwa[2]);
  ASSERT_GT(dev.norm(), 1e-3) << "the compliant frame never moved";
  EXPECT_GT(dev.normalized().dot(f.normalized()), 0.99)
      << "the deviation is not aligned with the applied force";

  // The default stiffness (200 N/m) puts the steady state at f/K_p.
  EXPECT_NEAR(dev.norm(), f.norm() / 200.0, 5e-3);

  // The IK closed the loop: the tracking error settled and the REAL tip
  // travelled the compliant displacement, in the direction of the force.
  Eigen::Matrix<double, 6, 1> e;
  for (int i = 0; i < 6; ++i)
    e(i) = d.pose_error[static_cast<std::size_t>(i)];
  EXPECT_LT(e.norm(), 5e-3) << "the arm is not following the compliant frame";

  const Eigen::Vector3d travel = ref.TcpPosition(JointsOf(state)) - tcp_start;
  EXPECT_NEAR(travel.norm(), dev.norm(), 5e-3);
  EXPECT_GT(travel.normalized().dot(f.normalized()), 0.99);
}

// ── §10.6 staleness: fade → DEGRADED, never SAFE_STOP ───────────────────────

TEST(TaskAdmittanceController, StaleWrenchFadesOutAndDegradesWithoutStopping) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));
  auto state = MakeState(Posture());

  for (int k = 0; k < 500; ++k) {
    Send(c, Wrench6{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }
  ASSERT_GT(c.GetDiagnosticsForTesting().wrench_fade, 0.99);

  // Producer dies. Defaults: timeout 0.05 s, fadeout 0.1 s.
  for (int k = 0; k < 200; ++k)
    (void)c.Compute(state);
  const auto d = c.GetDiagnosticsForTesting();
  EXPECT_TRUE(d.wrench_stale);
  EXPECT_DOUBLE_EQ(d.wrench_fade, 0.0) << "§10.6 MUST — an expired wrench goes to ZERO, never held";
  EXPECT_EQ(d.state, static_cast<std::uint8_t>(ComplianceState::kDegraded));
  EXPECT_TRUE(d.control_valid) << "wrench loss must not stop the controller";
}

TEST(TaskAdmittanceController, StiffnessReturnsTheFrameAfterWrenchLossButZeroStiffnessHolds) {
  auto run = [](const std::string& stiffness) {
    TaskAdmittanceController c(Urdf7());
    c.LoadConfig(YAML::Load(TransparentYaml("stiffness: " + stiffness + "\n")));
    auto state = MakeState(Posture());
    for (int k = 0; k < 500; ++k) {
      Send(c, Wrench6{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
      (void)c.Compute(state);
    }
    const double pushed = Eigen::Vector3d(c.GetDiagnosticsForTesting().compliant_deviation[0],
                                          c.GetDiagnosticsForTesting().compliant_deviation[1],
                                          c.GetDiagnosticsForTesting().compliant_deviation[2])
                              .norm();
    for (int k = 0; k < 3000; ++k)
      (void)c.Compute(state);  // producer dead — fade drives f_ext to 0
    const auto d = c.GetDiagnosticsForTesting();
    const double after = Eigen::Vector3d(d.compliant_deviation[0], d.compliant_deviation[1],
                                         d.compliant_deviation[2])
                             .norm();
    return std::pair<double, double>{pushed, after};
  };

  const auto stiff = run("[200.0, 200.0, 200.0, 5.0, 5.0, 5.0]");
  ASSERT_GT(stiff.first, 1e-3);
  EXPECT_LT(stiff.second, 1e-5) << "K_p > 0 must pull the compliant frame back to X_d";

  // K_p = 0 is the hand-guiding mode: losing the sensor must leave the arm where
  // the operator put it, not send it home.
  const auto soft = run("[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]");
  ASSERT_GT(soft.first, 1e-3);
  EXPECT_NEAR(soft.second, soft.first, 1e-3);
}

// ── Wrench-path robustness (F3 / F6) ────────────────────────────────────────

TEST(TaskAdmittanceController, ANonFiniteWrenchSampleCannotLatchSafeStop) {
  // One garbage packet used to be fatal-for-the-process: NaN survives the
  // conditioner (every comparison against it is false), reaches the compliant
  // frame, raises ComplianceFaults::nan_inf and LATCHES SAFE_STOP — which
  // ClearEstop() explicitly does not clear and ~/reset_fault does not yet wire.
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));
  auto state = MakeState(Posture());

  const Wrench6 good{{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Wrench6 poisoned = good;
  poisoned[2] = std::numeric_limits<double>::quiet_NaN();

  for (int k = 0; k < 50; ++k) {
    Send(c, (k == 10) ? poisoned : good);
    (void)c.Compute(state);
    ASSERT_NE(c.GetDiagnosticsForTesting().state,
              static_cast<std::uint8_t>(ComplianceState::kSafeStop))
        << "a non-finite wrench sample latched SAFE_STOP at tick " << k;
  }
  const auto d = c.GetDiagnosticsForTesting();
  EXPECT_EQ(d.wrench_rejected, 1u) << "the drop must be counted, not silent";
  EXPECT_TRUE(d.wrench_valid) << "the good samples around it must still get through";
  EXPECT_TRUE(std::isfinite(d.wrench_lwa[2]));
}

TEST(TaskAdmittanceController, AReactivationDoesNotReviveTheWrenchFromBeforeIt) {
  // The F/T driver dies while the controller is stopped. Its last reading is
  // still sitting in the input slot; a reset that only re-dates the sample
  // resurrects that force as FRESH on the first tick back — the inverse of
  // §10.6's "an expired wrench goes to ZERO, never held".
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml("stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n")));
  auto state = MakeState(Posture());
  for (int k = 0; k < 100; ++k) {
    Send(c, Wrench6{{60.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }
  ASSERT_TRUE(c.GetDiagnosticsForTesting().wrench_valid);

  c.ResetTargetInitializationForTesting();  // == on_activate; producer stays dead
  for (int k = 0; k < 10; ++k)
    (void)c.Compute(state);

  const auto d = c.GetDiagnosticsForTesting();
  EXPECT_FALSE(d.wrench_valid) << "the pre-activation sample was revived at age 0";
  EXPECT_FALSE(d.wrench_stale) << "'no producer yet' is the activation transient, not a timeout";
  EXPECT_LT(Eigen::Vector3d(d.wrench_lwa[0], d.wrench_lwa[1], d.wrench_lwa[2]).norm(), 1e-12);
  // K_p = 0, so any force that got through would still be visible as drift.
  EXPECT_LT(
      Eigen::Vector3d(d.compliant_deviation[0], d.compliant_deviation[1], d.compliant_deviation[2])
          .norm(),
      1e-12)
      << "a revived wrench moved the compliant frame after the reactivation";
}

// ── §7.3 integration basis ──────────────────────────────────────────────────

TEST(TaskAdmittanceController, IntegrateFromMeasuredReanchorsEveryTick) {
  // The arm never moves (a stuck lower controller). With the measured base, the
  // command can only ever be one tick ahead of it, so no divergence accrues.
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml("integrate_from_measured: true\n")));
  auto state = MakeState(Posture());
  for (int k = 0; k < 2000; ++k) {
    Send(c, Wrench6{{40.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }
  const auto d = c.GetDiagnosticsForTesting();
  EXPECT_LT(d.command_divergence, 0.05);
  EXPECT_NE(d.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop));
}

TEST(TaskAdmittanceController, SelfIntegratingCommandFaultsWhenItWindsAwayFromTheArm) {
  // Same stuck arm, `false` base: the command integrates away from q_meas, which
  // is the windup §7.3 requires a bound on.
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(
      TransparentYaml("integrate_from_measured: false\ncommand_divergence_limit: 0.05\n")));
  auto state = MakeState(Posture());
  bool stopped = false;
  double peak = 0.0;
  for (int k = 0; k < 3000 && !stopped; ++k) {
    Send(c, Wrench6{{40.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
    const auto d = c.GetDiagnosticsForTesting();
    peak = std::max(peak, d.command_divergence);
    stopped = d.state == static_cast<std::uint8_t>(ComplianceState::kSafeStop);
  }
  EXPECT_TRUE(stopped) << "the command wound to " << peak << " rad without faulting";

  // SAFE_STOP is latched — only ResetFault() clears it (E-8).
  (void)c.Compute(state);
  EXPECT_EQ(c.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop));
  c.ResetFault();
  (void)c.Compute(state);
  EXPECT_NE(c.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop));
}

// ── E-STOP: a position HOLD, not a slew and not a follow ────────────────────

TEST(TaskAdmittanceController, EstopLatchesThePositionMeasuredOnTheFirstHeldTick) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));
  auto state = MakeState(Posture());
  for (int k = 0; k < 100; ++k) {
    Send(c, Wrench6{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }

  c.TriggerEstop();
  const auto held = c.Compute(state);
  std::array<double, kNj> latched{};
  for (int i = 0; i < kNj; ++i)
    latched[static_cast<std::size_t>(i)] = held.devices[0].commands[static_cast<std::size_t>(i)];
  for (int i = 0; i < kNj; ++i)
    EXPECT_NEAR(latched[static_cast<std::size_t>(i)], Posture()[static_cast<std::size_t>(i)], 1e-9);

  // The arm is now backdriven while stopped. A hold that re-emitted q_meas would
  // follow it — leaving nothing to resist the push, which is the whole point of
  // an E-STOP hold.
  auto pushed = Posture();
  for (auto& v : pushed)
    v += 0.2;
  for (int k = 0; k < 50; ++k) {
    auto moved = MakeState(pushed);
    const auto out = c.Compute(moved);
    for (int i = 0; i < kNj; ++i)
      ASSERT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                  latched[static_cast<std::size_t>(i)], 1e-9);
  }
  EXPECT_TRUE(c.IsEstopped());

  // Recovery re-seeds from the measured state: the compliant frame must restart
  // at X_d = where the arm actually IS, not where it was before the stop.
  c.ClearEstop();
  auto moved = MakeState(pushed);
  const auto out = c.Compute(moved);
  for (int i = 0; i < kNj; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                pushed[static_cast<std::size_t>(i)], 1e-9);
}

TEST(TaskAdmittanceController, ALatchedSafeStopHoldsInsteadOfFollowingTheArm) {
  // A latched SAFE_STOP forces a re-seed on every tick (the held tick invalidates
  // the target init). The hold latch must survive that: if it were cleared by
  // the re-seed it would re-latch at the freshly measured position each tick,
  // turning the hold into a follower — the same defect the global-E-STOP path is
  // guarded against, reachable by a completely different route.
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(
      TransparentYaml("integrate_from_measured: false\ncommand_divergence_limit: 0.05\n")));
  auto state = MakeState(Posture());
  for (int k = 0; k < 3000; ++k) {
    Send(c, Wrench6{{40.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
    if (c.GetDiagnosticsForTesting().state == static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      break;
  }
  ASSERT_EQ(c.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop));

  const auto first = c.Compute(state);
  std::array<double, kNj> latched{};
  for (int i = 0; i < kNj; ++i)
    latched[static_cast<std::size_t>(i)] = first.devices[0].commands[static_cast<std::size_t>(i)];

  auto pushed = Posture();
  for (auto& v : pushed)
    v += 0.15;
  for (int k = 0; k < 20; ++k) {
    auto moved = MakeState(pushed);
    const auto out = c.Compute(moved);
    for (int i = 0; i < kNj; ++i)
      ASSERT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                  latched[static_cast<std::size_t>(i)], 1e-9)
          << "the SAFE_STOP hold followed the backdriven arm at tick " << k;
  }
}

// ── E-STOP hold across the boundaries that move the arm (F1, E-8) ───────────
// The complement of the two tests above. They pin what the latch must SURVIVE
// (a per-tick re-seed); these pin what must RETIRE it. A latch that outlives a
// deactivate or an E-STOP clear describes a pose the arm has since left, and the
// first held tick after the boundary commands it in one unbounded step —
// ComputeEstop has no slew and no divergence bound to catch that.

TEST(TaskAdmittanceController, AReactivationRetiresAHoldLatchedBeforeIt) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(
      TransparentYaml("integrate_from_measured: false\ncommand_divergence_limit: 0.05\n")));
  auto state = MakeState(Posture());
  for (int k = 0; k < 3000; ++k) {
    Send(c, Wrench6{{40.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
    if (c.GetDiagnosticsForTesting().state == static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      break;
  }
  ASSERT_EQ(c.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop));
  const auto held = c.Compute(state);
  ASSERT_NEAR(held.devices[0].commands[0], Posture()[0], 1e-9);

  // Deactivate, another controller drives the arm elsewhere, reactivate. The
  // SAFE_STOP is still latched (only ResetFault clears it), so the very first
  // tick back is a HELD one — the exact path the stale latch reaches.
  auto moved = Posture();
  for (auto& v : moved)
    v += 0.2;
  c.ResetTargetInitializationForTesting();  // == the CM's on_activate hook

  auto after = MakeState(moved);
  const auto out = c.Compute(after);
  for (int i = 0; i < kNj; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                moved[static_cast<std::size_t>(i)], 1e-9)
        << "joint " << i << ": the reactivation re-commanded the pre-deactivate hold";
}

TEST(TaskAdmittanceController, AnEstopHoldDoesNotSurviveAClearAndRetrigger) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));
  auto state = MakeState(Posture());
  for (int k = 0; k < 20; ++k) {
    Send(c, Wrench6{{5.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }

  c.TriggerEstop();
  const auto held = c.Compute(state);
  ASSERT_NEAR(held.devices[0].commands[0], Posture()[0], 1e-9);

  // Stop cleared, arm jogged on the pendant, stop hit again — all before this
  // controller ran a single clean tick, which is the only other thing that
  // retires the latch.
  c.ClearEstop();
  auto jogged = Posture();
  for (auto& v : jogged)
    v += 0.25;
  c.TriggerEstop();

  auto after = MakeState(jogged);
  const auto out = c.Compute(after);
  for (int i = 0; i < kNj; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                jogged[static_cast<std::size_t>(i)], 1e-9)
        << "joint " << i << ": the hold latched before the E-STOP clear came back";
}

TEST(TaskAdmittanceController, TheHoldIsClampedIntoTheJointLimitsAtABoundedRate) {
  // An arm already outside its mechanical range when the stop fires must not have
  // that pose latched and re-commanded forever — but the correction is a MOTION,
  // so it is rate-bound like every other motion this controller emits.
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));

  rtc::DeviceJointLimits limits;
  limits.max_velocity.assign(kNj, 1.0);
  limits.position_lower.assign(kNj, -2.0);
  limits.position_upper.assign(kNj, 0.05);  // Posture()[0] = 0.1 sits 0.05 rad past it
  rtc::DeviceNameConfig cfg;
  cfg.device_name = "arm";
  cfg.joint_limits = limits;
  c.SetDeviceNameConfigs(std::map<std::string, rtc::DeviceNameConfig>{{"arm", cfg}});

  auto state = MakeState(Posture());
  c.TriggerEstop();
  const auto first = c.Compute(state);
  EXPECT_NEAR(first.devices[0].commands[0], Posture()[0] - 1.0 * kDt, 1e-12)
      << "the joint-limit correction left as an unbounded step, or never happened";
  EXPECT_NEAR(first.devices[0].commands[3], Posture()[3], 1e-12)
      << "joint 3 is inside the band — the hold must not move it at all";

  for (int k = 0; k < 200; ++k)
    (void)c.Compute(state);
  const auto settled = c.Compute(state);
  EXPECT_NEAR(settled.devices[0].commands[0], 0.05, 1e-9)
      << "the hold never slewed all the way into the joint limit";
  EXPECT_NEAR(settled.devices[0].commands[3], Posture()[3], 1e-12);
}

// ── RT-1: no heap allocation on the tick ────────────────────────────────────

TEST(TaskAdmittanceController, ComputeIsAllocationFree) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));
  auto state = MakeState(Posture());
  Send(c, Wrench6{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

  // The FIRST Compute is the expensive one — it seeds X_d, the posture target,
  // the compliant frame and the wrench pipeline. If any of that allocates, the
  // very first RT tick after activation is the one that blows the deadline.
  g_alloc_count = 0;
  g_alloc_active = true;
  (void)c.Compute(state);
  g_alloc_active = false;
  EXPECT_EQ(g_alloc_count, 0u) << "the seeding tick allocated";

  g_alloc_count = 0;
  g_alloc_active = true;
  for (int k = 0; k < 200; ++k) {
    Send(c, Wrench6{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }
  g_alloc_active = false;
  EXPECT_EQ(g_alloc_count, 0u) << "the steady-state tick allocated";

  // E-STOP and fault paths are RT ticks too.
  c.TriggerEstop();
  g_alloc_count = 0;
  g_alloc_active = true;
  (void)c.Compute(state);
  g_alloc_active = false;
  EXPECT_EQ(g_alloc_count, 0u) << "the E-STOP hold allocated";
}

// ── Configure-time contracts ────────────────────────────────────────────────

TEST(TaskAdmittanceController, ConfigureRejectsAWrenchlessOrTorqueConfiguration) {
  {
    TaskAdmittanceController c(Urdf7());
    // §7.1: admittance takes force as its INPUT — A=NONE is not a fallback here.
    EXPECT_THROW(c.LoadConfig(YAML::Load("external_wrench:\n  enabled: false\n")),
                 std::runtime_error);
  }
  {
    TaskAdmittanceController c(Urdf7());
    // D14: the output is a position command; there is no kVelocity and no torque.
    EXPECT_THROW(c.LoadConfig(YAML::Load(TransparentYaml("command_type: torque\n"))),
                 std::runtime_error);
  }
  {
    TaskAdmittanceController c(Urdf7());
    EXPECT_THROW(c.LoadConfig(YAML::Load(
                     TransparentYaml("desired_inertia: [0.0, 1.0, 1.0, 0.1, 0.1, 0.1]\n"))),
                 std::runtime_error);
  }
  {
    TaskAdmittanceController c(Urdf7());
    EXPECT_THROW(c.LoadConfig(YAML::Load(TransparentYaml({}, "  sensor_frame: not_a_frame\n"))),
                 std::runtime_error);
  }
}

TEST(TaskAdmittanceController, ARejectedConfigureLeavesTheLiveGainsUntouched) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml("stiffness: [111.0, 111.0, 111.0, 3.0, 3.0, 3.0]\n")));
  const auto before = c.get_gains();

  // A bad sensor_frame is resolved LAST, after every gain has been parsed — the
  // ordering exists so a rejected reconfigure cannot leave half the new gains in.
  EXPECT_THROW(
      c.LoadConfig(YAML::Load(TransparentYaml("stiffness: [999.0, 999.0, 999.0, 9.0, 9.0, 9.0]\n",
                                              "  sensor_frame: not_a_frame\n"))),
      std::runtime_error);
  EXPECT_DOUBLE_EQ(c.get_gains().admittance.stiffness[0], before.admittance.stiffness[0]);
}

TEST(TaskAdmittanceController, MinDesiredInertiaFloorsAConfiguredValue) {
  // §7.4: the floor is applied where Λ_d is USED, so a YAML that asks for a
  // 1 g desired inertia still behaves as the 2 kg minimum. Verified through the
  // controller's motion, not by reading the gain back — the gain is stored as
  // written on purpose, and only the floor at the point of use protects contact
  // stability.
  auto travel = [](const std::string& inertia) {
    TaskAdmittanceController c(Urdf7());
    c.LoadConfig(YAML::Load(TransparentYaml("desired_inertia: " + inertia +
                                            "\nstiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n")));
    auto state = MakeState(Posture());
    for (int k = 0; k < 20; ++k) {
      Send(c, Wrench6{{20.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
      (void)c.Compute(state);
    }
    const auto d = c.GetDiagnosticsForTesting();
    return Eigen::Vector3d(d.compliant_deviation[0], d.compliant_deviation[1],
                           d.compliant_deviation[2])
        .norm();
  };
  const double tiny = travel("[0.001, 0.001, 0.001, 1.0e-6, 1.0e-6, 1.0e-6]");
  const double floored = travel("[2.0, 2.0, 2.0, 0.05, 0.05, 0.05]");
  EXPECT_NEAR(tiny, floored, 1e-12);
}

TEST(TaskAdmittanceController, MaxDampingFloorSurvivesSetGains) {
  // NUM-1: the λ_max floor has to sit where the DLS solve USES it, not only in
  // LoadConfig. set_gains() writes the struct straight into the SeqLock, so a
  // configure-time-only floor is bypassable by every caller holding the handle —
  // the same hole 0a61aaf closed for the operational-space controller.
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml()));

  auto g = c.get_gains();
  g.singularity_threshold = 10.0;  // σ₀ above this arm's σ_min → the DLS ramp is always armed
  g.max_damping = 0.0;             // exactly what LoadConfig would have floored away
  c.set_gains(g);

  auto state = MakeState(Posture());
  Send(c, Wrench6{{20.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  (void)c.Compute(state);

  const auto d = c.GetDiagnosticsForTesting();
  ASSERT_LT(d.sigma_min, g.singularity_threshold) << "σ₀ must be armed for λ² to be observable";
  ASSERT_GT(d.sigma_min, g.singularity_critical) << "posture must stay out of SAFE_STOP";
  EXPECT_GT(d.lambda_sq, 0.0) << "λ_max = 0 reached the DLS solve — J Jᵀ is left unregularised";
}

TEST(TaskAdmittanceController, BiasCalibrationSuppressesTheWrenchUntilItCommits) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(R"(
activation_ramp_time: 0.0
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 10
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)"));
  auto state = MakeState(Posture());

  // A standing offset present at activation IS the bias by §3.2.1's definition.
  for (int k = 0; k < 9; ++k) {
    Send(c, Wrench6{{7.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
    const auto d = c.GetDiagnosticsForTesting();
    EXPECT_FALSE(d.bias_calibrated);
    EXPECT_LT(Eigen::Vector3d(d.compliant_deviation[0], d.compliant_deviation[1],
                              d.compliant_deviation[2])
                  .norm(),
              1e-12)
        << "an uncalibrated wrench reached the control law at sample " << k;
  }
  for (int k = 0; k < 3; ++k) {
    Send(c, Wrench6{{7.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }
  const auto d = c.GetDiagnosticsForTesting();
  EXPECT_TRUE(d.bias_calibrated);
  EXPECT_NEAR(c.GetWrenchBiasForTesting()[0], 7.0, 1e-9);
  // ...and with the bias committed the same standing 7 N now reads as zero.
  EXPECT_LT(Eigen::Vector3d(d.wrench_lwa[0], d.wrench_lwa[1], d.wrench_lwa[2]).norm(), 1e-9);
}

TEST(TaskAdmittanceController, ReactivationReseedsTheCompliantFrame) {
  TaskAdmittanceController c(Urdf7());
  c.LoadConfig(YAML::Load(TransparentYaml("stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n")));
  auto state = MakeState(Posture());
  for (int k = 0; k < 300; ++k) {
    Send(c, Wrench6{{25.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    (void)c.Compute(state);
  }
  ASSERT_GT(Eigen::Vector3d(c.GetDiagnosticsForTesting().compliant_deviation[0],
                            c.GetDiagnosticsForTesting().compliant_deviation[1],
                            c.GetDiagnosticsForTesting().compliant_deviation[2])
                .norm(),
            1e-3);

  // An activation must not inherit the deviation accrued before it — otherwise
  // the first tick after re-activation steps the arm by the whole offset.
  c.ResetTargetInitializationForTesting();
  (void)c.Compute(state);
  const auto d = c.GetDiagnosticsForTesting();
  EXPECT_LT(
      Eigen::Vector3d(d.compliant_deviation[0], d.compliant_deviation[1], d.compliant_deviation[2])
          .norm(),
      1e-12);
  double v_norm_sq = 0.0;
  for (const double v : d.compliant_velocity)
    v_norm_sq += v * v;
  EXPECT_LT(v_norm_sq, 1e-24);
}

}  // namespace
