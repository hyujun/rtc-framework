// ── Unit tests for TaskImpedanceController (spec §6.2, A=NONE) ───────────────
// Control-law + selection-matrix + configure + activation + E-STOP + zero-alloc
// tests that need a real robot model (the algebraic nullspace/DLS identities
// T4.5/T5.1 are covered model-free in test_compliance_core.cpp).
//
// Zero-allocation (T7.1) is checked by GLOBAL operator-new interposition, not
// Eigen's set_is_malloc_allowed: Compute() lives in the rtc_controllers library
// (a separate TU compiled without the Eigen malloc macro), so a same-TU Eigen
// guard would observe nothing. operator-new interposition catches allocations in
// the linked library too, and is NDEBUG-independent — the "malloc 인터포지션"
// path §11.7 T7.1 explicitly permits.
#include "rtc_controllers/direct/task_impedance_controller.hpp"
#include "test_urdf_path.hpp"
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <new>
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

using rtc::TaskImpedanceController;
using Sel = rtc::TaskImpedanceController::TaskSelection;

std::string Urdf6() {
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

std::string Urdf7() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

rtc::ControllerState MakeState(int nj, const std::vector<double>& q, const std::vector<double>& qd,
                               double dt = 0.002) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nj;
  state.devices[0].valid = true;
  for (int i = 0; i < nj; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] =
        (i < static_cast<int>(q.size())) ? q[static_cast<std::size_t>(i)] : 0.0;
    state.devices[0].velocities[static_cast<std::size_t>(i)] =
        (i < static_cast<int>(qd.size())) ? qd[static_cast<std::size_t>(i)] : 0.0;
  }
  return state;
}

// Independent ĝ(q) for the same URDF, for τ = g(q) comparison.
std::vector<double> GravityAt(const std::string& urdf, const std::vector<double>& q) {
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = urdf;
  cfg.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(cfg);
  rtc_urdf_bridge::RtModelHandle handle(builder.GetFullModel());
  handle.ComputeGeneralizedGravity(std::span<const double>(q.data(), q.size()));
  auto g = handle.GetGeneralizedGravity();
  return std::vector<double>(g.data(), g.data() + g.size());
}

// ── T4.2: f_ext=0, e=0, q̇=0 → τ = g(q) ─────────────────────────────────────
TEST(TaskImpedance, GravityOnlyAtRest) {
  const std::vector<double> q(6, 0.3);  // bent pose ⇒ nonzero gravity (non-vacuous)
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  // First Compute seeds X_d = X_meas ⇒ e = 0; q̇ = 0 ⇒ ė = 0; nv == m ⇒ no
  // nullspace ⇒ τ = ĝ(q).
  const auto out = ctrl.Compute(state);
  const auto grav = GravityAt(Urdf6(), q);
  ASSERT_EQ(out.command_type, rtc::CommandType::kTorque);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                grav[static_cast<std::size_t>(i)], 1e-6)
        << "joint " << i;
}

// ── T4.4: TRANSLATION_ONLY filters rotation, keeps translation ──────────────
TEST(TaskImpedance, TranslationOnlySelection) {
  const std::vector<double> q(7, 0.2);
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 50.0;         // required for TRANSLATION_ONLY (§6.1)
  gains.activation_ramp_time = 0.0;  // full gain immediately
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kTranslationOnly);
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  const auto seed = ctrl.Compute(state);  // seeds X_d = X_meas
  const double px = seed.actual_task_positions[0];
  const double py = seed.actual_task_positions[1];
  const double pz = seed.actual_task_positions[2];

  auto target = [&](double x, double y, double z, double roll, double pitch, double yaw) {
    const std::array<double, 6> t{x, y, z, roll, pitch, yaw};
    ctrl.SetDeviceTarget(0, std::span<const double>(t.data(), t.size()));
    return ctrl.Compute(state);
  };

  // Same position, two different orientations → identical torque (rotation is
  // not in the task under TRANSLATION_ONLY).
  const auto rotA = target(px, py, pz, 0.4, -0.2, 0.1);
  const auto rotB = target(px, py, pz, -0.3, 0.5, -0.6);
  for (int i = 0; i < 7; ++i)
    EXPECT_NEAR(rotA.devices[0].commands[static_cast<std::size_t>(i)],
                rotB.devices[0].commands[static_cast<std::size_t>(i)], 1e-9)
        << "orientation leaked into task torque, joint " << i;

  // A position offset DOES change the torque (translation is regulated).
  const auto shifted = target(px + 0.05, py, pz, 0.4, -0.2, 0.1);
  double diff = 0.0;
  for (int i = 0; i < 7; ++i)
    diff += std::abs(shifted.devices[0].commands[static_cast<std::size_t>(i)] -
                     rotA.devices[0].commands[static_cast<std::size_t>(i)]);
  EXPECT_GT(diff, 1e-3) << "position error produced no task torque";
}

// ── T6.2: TRANSLATION_ONLY + nullspace_stiffness == 0 → configure error ─────
TEST(TaskImpedance, ConfigureRejectsTranslationOnlyZeroNullspace) {
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kTranslationOnly);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Node()), std::runtime_error);

  // FULL_SE3 with zero nullspace is fine (orientation is a task DoF).
  TaskImpedanceController ctrl_full(Urdf6(), gains, Sel::kFullSe3);
  EXPECT_NO_THROW(ctrl_full.LoadConfig(YAML::Node()));

  // TRANSLATION_ONLY with nonzero nullspace stiffness is accepted.
  TaskImpedanceController::Gains ok = gains;
  ok.nullspace_kp = 30.0;
  TaskImpedanceController ctrl_ok(Urdf6(), ok, Sel::kTranslationOnly);
  EXPECT_NO_THROW(ctrl_ok.LoadConfig(YAML::Node()));
}

// ── Activation: re-seeds the desired pose to the measured state ─────────────
TEST(TaskImpedance, ReactivationHoldsMeasured) {
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto s1 = MakeState(6, std::vector<double>(6, 0.1), std::vector<double>(6, 0.0));
  (void)ctrl.Compute(s1);  // seed at q1

  // Move to a new measured pose and force re-activation: X_d must follow to the
  // new measured pose (e = 0 ⇒ τ = gravity at q2), not drive back to q1.
  auto s2 = MakeState(6, std::vector<double>(6, 0.4), std::vector<double>(6, 0.0));
  ctrl.ResetTargetInitializationForTesting();
  const auto out = ctrl.Compute(s2);
  const auto grav = GravityAt(Urdf6(), std::vector<double>(6, 0.4));
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                grav[static_cast<std::size_t>(i)], 1e-6)
        << "joint " << i << " did not hold the re-measured pose";
}

// ── E-STOP: gravity-compensated damped hold (E-8) ───────────────────────────
TEST(TaskImpedance, EstopGravityCompHold) {
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));  // q̇ = 0
  (void)ctrl.Compute(state);
  ctrl.TriggerEstop();
  const auto out = ctrl.Compute(state);
  // τ = ĝ(q) − D·0 = ĝ(q), clamped (within limits here).
  const auto grav = GravityAt(Urdf6(), q);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                grav[static_cast<std::size_t>(i)], 1e-6);
}

// ── T5.1 flavor: commands stay finite across random configurations ──────────
TEST(TaskImpedance, OutputAlwaysFinite) {
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 20.0;
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kFullSe3);
  for (double a : {-1.2, -0.4, 0.0, 0.7, 1.5}) {
    auto state = MakeState(7, std::vector<double>(7, a), std::vector<double>(7, 0.05));
    const auto out = ctrl.Compute(state);
    for (int i = 0; i < 7; ++i)
      EXPECT_TRUE(std::isfinite(out.devices[0].commands[static_cast<std::size_t>(i)]))
          << "a=" << a << " joint " << i;
  }
}

// ── T7.1: Compute() allocates nothing (from the very first call) ────────────
TEST(TaskImpedance, ComputeIsAllocationFree) {
  const std::vector<double> q(7, 0.15);
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 40.0;  // exercise the nullspace / DLS path too
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kFullSe3);
  auto state = MakeState(7, q, std::vector<double>(7, 0.02));
  // Measure the FIRST Compute (spec T7.1: from the first call).
  g_alloc_count = 0;
  g_alloc_active = true;
  const auto out = ctrl.Compute(state);
  g_alloc_active = false;
  EXPECT_EQ(g_alloc_count, 0u) << "Compute() touched the heap on the RT path";
  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
}

}  // namespace
