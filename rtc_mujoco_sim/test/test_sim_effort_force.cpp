// ── test_sim_effort_force.cpp ────────────────────────────────────────────────
// Live-sim coverage for effort read-back and external-force injection. Drives
// the simulator synchronously via StepForTest() (SimLoop thread NOT running).
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rtc {
namespace {

class SimEffortForce : public ::testing::Test {
 protected:
  void SetUp() override {
    sim_ = std::make_unique<MuJoCoSimulator>(test::MakeMinimalConfig());
    ASSERT_TRUE(sim_->Initialize());
  }

  std::unique_ptr<MuJoCoSimulator> sim_;
};

// GetEfforts returns one entry per state joint, all finite after a real step
// (efforts = qfrc_actuator + qfrc_applied, computed in ReadState).
TEST_F(SimEffortForce, EffortsSizedAndFiniteAfterStep) {
  sim_->StepForTest();
  const auto efforts = sim_->GetEfforts(0);
  ASSERT_EQ(efforts.size(), static_cast<std::size_t>(sim_->NumStateJoints(0)));
  for (double e : efforts) {
    EXPECT_TRUE(std::isfinite(e));
  }
}

// SetExternalForce on a valid body stages a wrench that PreparePhysicsStep
// memcpy's into data_->xfrc_applied on the next tick.
TEST_F(SimEffortForce, ExternalForceWritesXfrcApplied) {
  const int body_id = 1;  // link1 (world body is index 0)
  ASSERT_LT(body_id, sim_->GetModel()->nbody);

  const std::array<double, 6> wrench = {0.0, 0.0, 7.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(sim_->SetExternalForce(body_id, wrench));
  sim_->StepForTest();

  const std::size_t off = static_cast<std::size_t>(body_id) * 6;
  const auto* xfrc = sim_->GetData()->xfrc_applied;
  EXPECT_DOUBLE_EQ(xfrc[off + 2], 7.0);  // Fz applied to link1
  EXPECT_DOUBLE_EQ(xfrc[off + 0], 0.0);
}

// ClearExternalForce drops the dirty flag so PreparePhysicsStep zeroes
// xfrc_applied on the following tick.
TEST_F(SimEffortForce, ClearExternalForceZerosXfrcApplied) {
  const int body_id = 1;
  const std::array<double, 6> wrench = {0.0, 0.0, 7.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(sim_->SetExternalForce(body_id, wrench));
  sim_->StepForTest();
  const std::size_t off = static_cast<std::size_t>(body_id) * 6;
  ASSERT_DOUBLE_EQ(sim_->GetData()->xfrc_applied[off + 2], 7.0);

  sim_->ClearExternalForce();
  sim_->StepForTest();
  EXPECT_DOUBLE_EQ(sim_->GetData()->xfrc_applied[off + 2], 0.0);
}


// ── Application-point contract (#135) ────────────────────────────────────────
//
// A staged wrench must act where the CALLER named it, not where MuJoCo would
// put it on its own. The two differ by the body's centre-of-mass offset, and
// the gap is a finite, smooth, wrong answer rather than a crash — no gate and
// no NaN check downstream can see it, so it has to be pinned here.

// Cross product, spelled out so the expectation is independent of the mju_*
// call the implementation uses to produce it.
std::array<double, 3> Cross(const double* a, const std::array<double, 3>& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// The fixture must actually break the two symmetries the contract turns on,
// otherwise every test below passes without testing anything. Asserted rather
// than assumed because both properties live in an XML file that no compiler
// checks and that a future edit could flatten while the suite stays green.
TEST_F(SimEffortForce, ProbeBodyBreaksTheContractSymmetries) {
  const int body = sim_->FindBodyId("probe_link");
  ASSERT_GT(body, 0) << "fixture lost probe_link";

  const auto* data = sim_->GetData();
  const double* xpos = &data->xpos[3 * body];
  const double* xipos = &data->xipos[3 * body];
  const double offset = std::sqrt(std::pow(xipos[0] - xpos[0], 2) + std::pow(xipos[1] - xpos[1], 2) +
                                  std::pow(xipos[2] - xpos[2], 2));
  EXPECT_GT(offset, 1e-2) << "centre of mass sits on the body frame origin — a test using this "
                             "body cannot tell the two application points apart";

  // Non-identity orientation, so a body-local point and a world point differ.
  const double* xmat = &data->xmat[9 * body];
  EXPECT_GT(std::abs(xmat[2]) + std::abs(xmat[6]), 1e-2)
      << "probe_link is axis-aligned — R * point cannot be told apart from point";
}

// THE contract: a pure force requested at a body-local point produces a staged
// wrench that is equivalent to exactly that — force unchanged, and a torque
// carrying precisely the moment of transferring the application point to the
// centre of mass MuJoCo will actually use.
TEST_F(SimEffortForce, ExternalWrenchActsAtRequestedPoint) {
  const int body = sim_->FindBodyId("probe_link");
  ASSERT_GT(body, 0);

  // Deliberately oblique: no component is zero, so a dropped or swapped axis
  // cannot survive, and the force is not parallel to the moment arm.
  const std::array<double, 3> point = {0.031, -0.017, 0.023};
  const std::array<double, 6> wrench = {3.0, -5.0, 7.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(sim_->SetExternalWrenchAtPoint(body, point, wrench));
  sim_->StepForTest();

  const auto* data = sim_->GetData();
  const double* xpos = &data->xpos[3 * body];
  const double* xipos = &data->xipos[3 * body];
  const double* xmat = &data->xmat[9 * body];

  // Requested application point, body-local -> world.
  std::array<double, 3> p_world{};
  for (std::size_t r = 0; r < 3; ++r) {
    p_world[r] = xpos[r];
    for (std::size_t c = 0; c < 3; ++c) {
      p_world[r] += xmat[3 * r + c] * point[c];
    }
  }
  const std::array<double, 3> arm = {p_world[0] - xipos[0], p_world[1] - xipos[1],
                                     p_world[2] - xipos[2]};

  const std::size_t off = static_cast<std::size_t>(body) * 6;
  const auto* xfrc = sim_->GetData()->xfrc_applied;

  // Force is transferred untouched.
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(xfrc[off + i], wrench[i]) << "force component " << i;
  }

  // Torque carries the transfer moment arm x f.
  const std::array<double, 3> expected = Cross(arm.data(), {wrench[0], wrench[1], wrench[2]});
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(xfrc[off + 3 + i], expected[i], 1e-12) << "torque component " << i;
  }

  // The moment is not a rounding artefact — if it were, this test would pass
  // against an implementation that stages the request verbatim.
  EXPECT_GT(std::abs(expected[0]) + std::abs(expected[1]) + std::abs(expected[2]), 1e-3);

  // Round trip: read the staged wrench back about the point that was asked for
  // and the requested torque (zero) must reappear. This is the assertion a
  // sign error in the transfer fails and the componentwise checks above could
  // in principle share.
  const std::array<double, 3> back = {xipos[0] - p_world[0], xipos[1] - p_world[1],
                                      xipos[2] - p_world[2]};
  const std::array<double, 3> moment_back =
      Cross(back.data(), {xfrc[off + 0], xfrc[off + 1], xfrc[off + 2]});
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(xfrc[off + 3 + i] + moment_back[i], wrench[3 + i], 1e-12)
        << "wrench about the requested point, component " << i;
  }
}

// SetExternalForce is the point-at-origin case of the same call, not a second
// contract. Keeping one path is the point: two would be free to drift, and a
// divergence between them is invisible from either side.
TEST_F(SimEffortForce, SetExternalForceIsTheOriginCaseOfSetExternalWrench) {
  const int body = sim_->FindBodyId("probe_link");
  ASSERT_GT(body, 0);
  const std::array<double, 6> wrench = {3.0, -5.0, 7.0, 0.5, -0.25, 0.125};

  ASSERT_TRUE(sim_->SetExternalForce(body, wrench));
  sim_->StepForTest();
  const std::size_t off = static_cast<std::size_t>(body) * 6;
  std::array<double, 6> via_force{};
  for (std::size_t i = 0; i < 6; ++i) {
    via_force[i] = sim_->GetData()->xfrc_applied[off + i];
  }

  ASSERT_TRUE(sim_->SetExternalWrenchAtPoint(body, {0.0, 0.0, 0.0}, wrench));
  sim_->StepForTest();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(sim_->GetData()->xfrc_applied[off + i], via_force[i]) << "component " << i;
  }
}

// A name the model does not carry must be REFUSED, never absorbed. The world
// body is refused for the same reason: a wrench there is swallowed by the fixed
// base, so accepting it would report success for a request that does nothing.
TEST_F(SimEffortForce, UnknownBodyNameIsRefusedNotIgnored) {
  EXPECT_EQ(sim_->FindBodyId("no_such_body"), -1);
  EXPECT_EQ(sim_->FindBodyId(""), -1);
  EXPECT_EQ(sim_->FindBodyId(nullptr), -1);
  EXPECT_EQ(sim_->FindBodyId("world"), -1) << "world body must not be a valid wrench target";

  EXPECT_GT(sim_->FindBodyId("link1"), 0);
  EXPECT_GT(sim_->FindBodyId("probe_link"), 0);

  const std::array<double, 6> wrench = {0.0, 0.0, 7.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(sim_->SetExternalForce(sim_->FindBodyId("no_such_body"), wrench));
}

// ── External-library guard ───────────────────────────────────────────────────
//
// The transfer above is correct only while MuJoCo keeps applying xfrc_applied
// at the body centre of mass. That is measured behaviour, not a documented
// promise we control, and if a MuJoCo upgrade moved it to the body frame origin
// the compensation would become the very error it exists to remove — silently,
// because every assertion above is written against our own arithmetic. This
// test owns that assumption, on its own mjModel/mjData so it never touches the
// simulator's.
TEST(SimExternalForceLibraryContract, MujocoAppliesXfrcAtBodyCentreOfMass) {
  char err[512] = {0};
  mjModel* m = mj_loadXML(MINIMAL_MJCF_PATH, nullptr, err, sizeof(err));
  ASSERT_NE(m, nullptr) << err;
  mjData* d = mj_makeData(m);
  ASSERT_NE(d, nullptr);

  // Off-zero pose so the body is rotated and the moment arm is oblique.
  mj_resetData(m, d);
  d->qpos[0] = 0.3;
  d->qpos[1] = 0.6;
  mju_zero(d->qvel, static_cast<int>(m->nv));
  mju_zero(d->ctrl, static_cast<int>(m->nu));

  const int body = mj_name2id(m, mjOBJ_BODY, "link2");
  ASSERT_GT(body, 0);

  const std::size_t nv = static_cast<std::size_t>(m->nv);
  mju_zero(d->xfrc_applied, static_cast<int>(m->nbody) * 6);
  mj_forward(m, d);
  const std::vector<double> base(d->qfrc_smooth, d->qfrc_smooth + nv);

  const std::array<double, 3> f = {3.0, -5.0, 7.0};
  const std::size_t off = static_cast<std::size_t>(body) * 6;
  for (std::size_t i = 0; i < 3; ++i) {
    d->xfrc_applied[off + i] = f[i];
  }
  mj_forward(m, d);
  const std::vector<double> with(d->qfrc_smooth, d->qfrc_smooth + nv);

  std::vector<double> delta(nv);
  for (std::size_t i = 0; i < nv; ++i) {
    delta[i] = with[i] - base[i];
  }

  // J(point)^T f for each candidate application point.
  auto jac_t_f = [&](const double* point) {
    std::vector<double> jacp(3 * nv), jacr(3 * nv), out(nv, 0.0);
    mj_jac(m, d, jacp.data(), jacr.data(), point, body);
    for (std::size_t c = 0; c < nv; ++c) {
      for (std::size_t r = 0; r < 3; ++r) {
        out[c] += jacp[r * nv + c] * f[r];
      }
    }
    return out;
  };
  const auto at_origin = jac_t_f(&d->xpos[3 * body]);
  const auto at_com = jac_t_f(&d->xipos[3 * body]);

  double err_origin = 0.0;
  double err_com = 0.0;
  for (std::size_t i = 0; i < nv; ++i) {
    err_origin = std::fmax(err_origin, std::abs(delta[i] - at_origin[i]));
    err_com = std::fmax(err_com, std::abs(delta[i] - at_com[i]));
  }

  EXPECT_LT(err_com, 1e-9) << "MuJoCo no longer applies xfrc_applied at the body centre of mass — "
                              "StageExternalWrenches() compensates for an offset that is now wrong";
  EXPECT_GT(err_origin, 1e-6) << "body frame origin and centre of mass are indistinguishable in "
                                 "this pose — the check above proves nothing";

  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace rtc
