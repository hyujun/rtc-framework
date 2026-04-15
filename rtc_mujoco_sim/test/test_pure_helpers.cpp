// ── test_pure_helpers.cpp ─────────────────────────────────────────────────────
// Pure-logic unit tests for MuJoCoSimulator static helpers. No MJCF load.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace rtc {
namespace {

// ── SolverNameToEnum ──────────────────────────────────────────────────────────
TEST(SolverNameToEnum, KnownValues) {
  EXPECT_EQ(MuJoCoSimulator::SolverNameToEnum("PGS"),    mjSOL_PGS);
  EXPECT_EQ(MuJoCoSimulator::SolverNameToEnum("CG"),     mjSOL_CG);
  EXPECT_EQ(MuJoCoSimulator::SolverNameToEnum("Newton"), mjSOL_NEWTON);
}

TEST(SolverNameToEnum, UnknownFallsBackToNewton) {
  EXPECT_EQ(MuJoCoSimulator::SolverNameToEnum(""),        mjSOL_NEWTON);
  EXPECT_EQ(MuJoCoSimulator::SolverNameToEnum("newton"),  mjSOL_NEWTON);  // case-sensitive
  EXPECT_EQ(MuJoCoSimulator::SolverNameToEnum("garbage"), mjSOL_NEWTON);
}

// ── ConeNameToEnum ────────────────────────────────────────────────────────────
TEST(ConeNameToEnum, KnownValues) {
  EXPECT_EQ(MuJoCoSimulator::ConeNameToEnum("elliptic"),  mjCONE_ELLIPTIC);
  EXPECT_EQ(MuJoCoSimulator::ConeNameToEnum("pyramidal"), mjCONE_PYRAMIDAL);
}

TEST(ConeNameToEnum, UnknownFallsBackToPyramidal) {
  EXPECT_EQ(MuJoCoSimulator::ConeNameToEnum(""),         mjCONE_PYRAMIDAL);
  EXPECT_EQ(MuJoCoSimulator::ConeNameToEnum("Elliptic"), mjCONE_PYRAMIDAL);
}

// ── JacobianNameToEnum ────────────────────────────────────────────────────────
TEST(JacobianNameToEnum, KnownValues) {
  EXPECT_EQ(MuJoCoSimulator::JacobianNameToEnum("dense"),  mjJAC_DENSE);
  EXPECT_EQ(MuJoCoSimulator::JacobianNameToEnum("sparse"), mjJAC_SPARSE);
  EXPECT_EQ(MuJoCoSimulator::JacobianNameToEnum("auto"),   mjJAC_AUTO);
}

TEST(JacobianNameToEnum, UnknownFallsBackToAuto) {
  EXPECT_EQ(MuJoCoSimulator::JacobianNameToEnum(""),     mjJAC_AUTO);
  EXPECT_EQ(MuJoCoSimulator::JacobianNameToEnum("AUTO"), mjJAC_AUTO);
}

// ── IntegratorNameToEnum ──────────────────────────────────────────────────────
TEST(IntegratorNameToEnum, KnownValues) {
  EXPECT_EQ(MuJoCoSimulator::IntegratorNameToEnum("Euler"),        mjINT_EULER);
  EXPECT_EQ(MuJoCoSimulator::IntegratorNameToEnum("RK4"),          mjINT_RK4);
  EXPECT_EQ(MuJoCoSimulator::IntegratorNameToEnum("implicit"),     mjINT_IMPLICIT);
  EXPECT_EQ(MuJoCoSimulator::IntegratorNameToEnum("implicitfast"), mjINT_IMPLICITFAST);
}

TEST(IntegratorNameToEnum, UnknownFallsBackToEuler) {
  EXPECT_EQ(MuJoCoSimulator::IntegratorNameToEnum(""),      mjINT_EULER);
  EXPECT_EQ(MuJoCoSimulator::IntegratorNameToEnum("euler"), mjINT_EULER);
}

// ── ApplyFakeLpfStep ──────────────────────────────────────────────────────────
TEST(ApplyFakeLpfStep, AlphaZeroNoOp) {
  std::vector<double> state  = {0.0, 1.0, 2.0};
  std::vector<double> target = {5.0, 5.0, 5.0};
  MuJoCoSimulator::ApplyFakeLpfStep(state, target, 0.0);
  EXPECT_DOUBLE_EQ(state[0], 0.0);
  EXPECT_DOUBLE_EQ(state[1], 1.0);
  EXPECT_DOUBLE_EQ(state[2], 2.0);
}

TEST(ApplyFakeLpfStep, AlphaOneImmediateConverge) {
  std::vector<double> state  = {0.0, 1.0, 2.0};
  std::vector<double> target = {5.0, 5.0, 5.0};
  MuJoCoSimulator::ApplyFakeLpfStep(state, target, 1.0);
  EXPECT_DOUBLE_EQ(state[0], 5.0);
  EXPECT_DOUBLE_EQ(state[1], 5.0);
  EXPECT_DOUBLE_EQ(state[2], 5.0);
}

TEST(ApplyFakeLpfStep, AlphaHalfSingleStep) {
  std::vector<double> state  = {0.0};
  std::vector<double> target = {10.0};
  MuJoCoSimulator::ApplyFakeLpfStep(state, target, 0.5);
  EXPECT_DOUBLE_EQ(state[0], 5.0);
  MuJoCoSimulator::ApplyFakeLpfStep(state, target, 0.5);
  EXPECT_DOUBLE_EQ(state[0], 7.5);
}

TEST(ApplyFakeLpfStep, IterativeConvergence) {
  std::vector<double> state  = {0.0};
  const std::vector<double> target = {1.0};
  constexpr double alpha = 0.1;
  for (int i = 0; i < 200; ++i) {
    MuJoCoSimulator::ApplyFakeLpfStep(state, target, alpha);
  }
  EXPECT_NEAR(state[0], 1.0, 1e-6);
}

TEST(ApplyFakeLpfStep, EmptyInputs) {
  std::vector<double> empty_state;
  std::vector<double> target = {1.0};
  MuJoCoSimulator::ApplyFakeLpfStep(empty_state, target, 0.5);
  EXPECT_TRUE(empty_state.empty());

  std::vector<double> state = {1.0};
  std::vector<double> empty_target;
  MuJoCoSimulator::ApplyFakeLpfStep(state, empty_target, 0.5);
  EXPECT_DOUBLE_EQ(state[0], 1.0);  // unchanged
}

TEST(ApplyFakeLpfStep, SizeMismatchProcessesMin) {
  std::vector<double> state  = {0.0, 0.0, 0.0};
  std::vector<double> target = {1.0, 1.0};
  MuJoCoSimulator::ApplyFakeLpfStep(state, target, 1.0);
  EXPECT_DOUBLE_EQ(state[0], 1.0);
  EXPECT_DOUBLE_EQ(state[1], 1.0);
  EXPECT_DOUBLE_EQ(state[2], 0.0);  // not touched
}

TEST(ApplyFakeLpfStep, NaNTargetSkipped) {
  std::vector<double> state  = {0.0, 0.0};
  std::vector<double> target = {std::numeric_limits<double>::quiet_NaN(), 5.0};
  MuJoCoSimulator::ApplyFakeLpfStep(state, target, 1.0);
  EXPECT_DOUBLE_EQ(state[0], 0.0);  // skipped, state unchanged
  EXPECT_DOUBLE_EQ(state[1], 5.0);
  EXPECT_FALSE(std::isnan(state[0]));
}

}  // namespace
}  // namespace rtc
