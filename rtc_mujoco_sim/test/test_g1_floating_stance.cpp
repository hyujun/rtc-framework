// ── test_g1_floating_stance.cpp ──────────────────────────────────────────────
// The floating-base humanoid plant model (W0-F01, #975): what the derived MJCF
// robot_descriptions/robots/g1/mjcf/g1_29dof_floating.xml must contain, and
// that it stands still on its own under its position servos.
//
// Plain MuJoCo C API — no simulator node, no ROS. The model is the fixture and
// comes through the ament index (robot_descriptions is data-only, ARCH-5), so
// what is pinned here is the asset's contract, not this package's code:
//   model  — freejoint 1 · sole box 2 (the upstream 4 spheres per foot) ·
//            pelvis framepos/framequat/framelinvel/frameangvel · gyro +
//            accelerometer on both IMU sites · <position> 23 with the agreed
//            gains (wrists rigid) · keyframe "stand" · implicitfast · dt 0.002/3
//   stance — mj_resetDataKeyframe("stand"), then pure mj_step for 10 s: the
//            feet do not slip, the pelvis stays up and level, both feet carry
//            load, no joint sags away from its target, no actuator saturates,
//            and the run is bit-reproducible
// The thresholds are the Sprint Contract numbers (#975 v3) and do not move
// with the measurement; the pilot that sized them is recorded in the issue.
// ──────────────────────────────────────────────────────────────────────────────
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

constexpr const char* kModelRel = "/robots/g1/mjcf/g1_29dof_floating.xml";

// ── Sprint Contract v3 (#975) ────────────────────────────────────────────────
constexpr double kTimestep = 0.002 / 3.0;  // physics_timestep / n_substeps of the sim YAML
constexpr double kSimSeconds = 10.0;
constexpr double kSettleSeconds = 1.0;  // metrics are judged on t >= 1 s
constexpr double kFootSlipMaxM = 0.005;
constexpr double kBaseHeightDevMaxM = 0.02;
constexpr double kBaseTiltMaxRad = 0.05;
constexpr double kContactNormalMinN = 5.0;
constexpr double kJointErrMaxRad = 0.1;
constexpr double kSaturationFraction = 0.99;
constexpr double kDeterminismTol = 1e-9;
constexpr int kActuated = 23;
constexpr double kLegWaistKp = 200.0, kLegWaistKv = 5.0;
constexpr double kArmKp = 100.0, kArmKv = 2.0;

const std::array<const char*, 2> kAnkleRoll{"left_ankle_roll_link", "right_ankle_roll_link"};
const std::array<const char*, 2> kSole{"left_sole", "right_sole"};
const std::array<const char*, 6> kWristJoints{"left_wrist_roll_joint",   "left_wrist_pitch_joint",
                                              "left_wrist_yaw_joint",    "right_wrist_roll_joint",
                                              "right_wrist_pitch_joint", "right_wrist_yaw_joint"};

bool IsLegOrWaist(const std::string& joint) {
  for (const char* key : {"hip", "knee", "ankle", "waist"}) {
    if (joint.find(key) != std::string::npos)
      return true;
  }
  return false;
}

class G1FloatingModel : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string path =
        ament_index_cpp::get_package_share_directory("robot_descriptions") + kModelRel;
    std::array<char, 1000> err{};
    m_ = mj_loadXML(path.c_str(), nullptr, err.data(), static_cast<int>(err.size()));
    ASSERT_NE(m_, nullptr) << path << ": " << err.data();
    d_ = mj_makeData(m_);
    ASSERT_NE(d_, nullptr);
  }

  void TearDown() override {
    if (d_)
      mj_deleteData(d_);
    if (m_)
      mj_deleteModel(m_);
  }

  int Id(mjtObj type, const char* name) const { return mj_name2id(m_, type, name); }

  std::string Name(mjtObj type, int id) const {
    const char* n = mj_id2name(m_, type, id);
    return n ? n : "";
  }

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
};

// ═══════════════════════════════════════════════════════════════════════════
// Model contract
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(G1FloatingModel, OneFreeJointOnThePelvis) {
  int free_joints = 0, free_id = -1;
  for (int j = 0; j < m_->njnt; ++j) {
    if (m_->jnt_type[j] == mjJNT_FREE) {
      ++free_joints;
      free_id = j;
    }
  }
  ASSERT_EQ(free_joints, 1);
  EXPECT_EQ(Name(mjOBJ_JOINT, free_id), "floating_base_joint");
  EXPECT_EQ(m_->jnt_bodyid[free_id], Id(mjOBJ_BODY, "pelvis"));
  EXPECT_EQ(m_->nq, 7 + kActuated);
  EXPECT_EQ(m_->nv, 6 + kActuated);
}

TEST_F(G1FloatingModel, WristsAreRigid) {
  for (const char* j : kWristJoints) {
    EXPECT_EQ(Id(mjOBJ_JOINT, j), -1) << j << " must not exist (wrists are welded)";
  }
  EXPECT_EQ(m_->njnt, 1 + kActuated);
  // The wrist bodies (and their mass) are still there — only the joints went.
  for (const char* b : {"left_wrist_yaw_link", "right_wrist_yaw_link"}) {
    ASSERT_GE(Id(mjOBJ_BODY, b), 0) << b;
    EXPECT_GT(m_->body_mass[Id(mjOBJ_BODY, b)], 0.0) << b;
  }
}

TEST_F(G1FloatingModel, SoleBoxesAreTheOnlyFootCollision) {
  for (std::size_t k = 0; k < 2; ++k) {
    const int sole = Id(mjOBJ_GEOM, kSole[k]);
    const int foot = Id(mjOBJ_BODY, kAnkleRoll[k]);
    ASSERT_GE(sole, 0) << kSole[k];
    ASSERT_GE(foot, 0) << kAnkleRoll[k];
    EXPECT_EQ(m_->geom_type[sole], mjGEOM_BOX) << kSole[k];
    EXPECT_EQ(m_->geom_bodyid[sole], foot) << kSole[k] << " must hang under " << kAnkleRoll[k];
    EXPECT_NE(m_->geom_contype[sole], 0) << kSole[k];
    EXPECT_NE(m_->geom_conaffinity[sole], 0) << kSole[k];
    // The upstream model has 4 r=5 mm spheres per foot; the box replaces them,
    // it does not join them.
    int colliding = 0, spheres = 0;
    for (int g = 0; g < m_->ngeom; ++g) {
      if (m_->geom_bodyid[g] != foot)
        continue;
      if (m_->geom_type[g] == mjGEOM_SPHERE)
        ++spheres;
      if (m_->geom_contype[g] != 0 || m_->geom_conaffinity[g] != 0)
        ++colliding;
    }
    EXPECT_EQ(spheres, 0) << kAnkleRoll[k];
    EXPECT_EQ(colliding, 1) << kAnkleRoll[k] << ": the sole box is the foot's only collision geom";
  }
}

TEST_F(G1FloatingModel, PelvisGroundTruthSensors) {
  const int pelvis = Id(mjOBJ_BODY, "pelvis");
  ASSERT_GE(pelvis, 0);
  const std::array<std::pair<const char*, mjtSensor>, 4> expected{{
      {"pelvis_pos", mjSENS_FRAMEPOS},
      {"pelvis_quat", mjSENS_FRAMEQUAT},
      {"pelvis_linvel", mjSENS_FRAMELINVEL},
      {"pelvis_angvel", mjSENS_FRAMEANGVEL},
  }};
  for (const auto& [name, type] : expected) {
    const int s = Id(mjOBJ_SENSOR, name);
    ASSERT_GE(s, 0) << name;
    EXPECT_EQ(m_->sensor_type[s], type) << name;
    EXPECT_EQ(m_->sensor_objtype[s], mjOBJ_XBODY) << name;
    EXPECT_EQ(m_->sensor_objid[s], pelvis) << name;
  }
}

TEST_F(G1FloatingModel, ImuGyroAndAccelerometerOnBothSites) {
  const std::array<std::tuple<const char*, mjtSensor, const char*>, 4> expected{{
      {"imu-pelvis-angular-velocity", mjSENS_GYRO, "imu_in_pelvis"},
      {"imu-pelvis-linear-acceleration", mjSENS_ACCELEROMETER, "imu_in_pelvis"},
      {"imu-torso-angular-velocity", mjSENS_GYRO, "imu_in_torso"},
      {"imu-torso-linear-acceleration", mjSENS_ACCELEROMETER, "imu_in_torso"},
  }};
  for (const auto& [name, type, site] : expected) {
    const int s = Id(mjOBJ_SENSOR, name);
    ASSERT_GE(s, 0) << name;
    EXPECT_EQ(m_->sensor_type[s], type) << name;
    EXPECT_EQ(m_->sensor_objtype[s], mjOBJ_SITE) << name;
    EXPECT_EQ(m_->sensor_objid[s], Id(mjOBJ_SITE, site)) << name << " site " << site;
  }
  EXPECT_EQ(m_->nsensor, 8) << "4 ground-truth + 4 IMU, nothing else";
}

TEST_F(G1FloatingModel, PositionServosWithContractGains) {
  ASSERT_EQ(m_->nu, kActuated);
  int leg_waist = 0, arm = 0;
  for (int a = 0; a < m_->nu; ++a) {
    const std::string name = Name(mjOBJ_ACTUATOR, a);
    ASSERT_EQ(m_->actuator_trntype[a], mjTRN_JOINT) << name;
    const std::string joint = Name(mjOBJ_JOINT, m_->actuator_trnid[2 * a]);
    EXPECT_EQ(joint, name) << "actuator and joint share the name upstream";
    // <position kp kv> compiles to gain kp, bias (0, -kp, -kv).
    EXPECT_EQ(m_->actuator_gaintype[a], mjGAIN_FIXED) << name;
    EXPECT_EQ(m_->actuator_biastype[a], mjBIAS_AFFINE) << name;
    EXPECT_EQ(m_->actuator_dyntype[a], mjDYN_NONE) << name;
    const double kp = IsLegOrWaist(joint) ? kLegWaistKp : kArmKp;
    const double kv = IsLegOrWaist(joint) ? kLegWaistKv : kArmKv;
    (IsLegOrWaist(joint) ? leg_waist : arm) += 1;
    // The stance test reads its saturation limit from here: a joint without
    // actuatorfrcrange would make every step count as saturated.
    EXPECT_GT(m_->jnt_actfrcrange[2 * m_->actuator_trnid[2 * a] + 1], 0.0) << name;
    const mjtNum* gain = m_->actuator_gainprm + a * mjNGAIN;
    const mjtNum* bias = m_->actuator_biasprm + a * mjNBIAS;
    EXPECT_DOUBLE_EQ(gain[0], kp) << name;
    EXPECT_DOUBLE_EQ(bias[1], -kp) << name;
    EXPECT_DOUBLE_EQ(bias[2], -kv) << name;
  }
  EXPECT_EQ(leg_waist, 12 + 3);
  EXPECT_EQ(arm, 8);
}

TEST_F(G1FloatingModel, KeyframeIntegratorTimestep) {
  EXPECT_EQ(m_->nkey, 1);
  EXPECT_GE(Id(mjOBJ_KEY, "stand"), 0);
  EXPECT_EQ(m_->opt.integrator, mjINT_IMPLICITFAST);
  EXPECT_NEAR(m_->opt.timestep, kTimestep, 1e-12);
}

// ═══════════════════════════════════════════════════════════════════════════
// Stance
// ═══════════════════════════════════════════════════════════════════════════

struct StanceMetrics {
  std::array<double, 2> foot_slip_max{};  // sole centre xy, relative to t = 1 s
  double base_height_dev_max = 0.0;       // pelvis z, relative to t0
  double base_tilt_max = 0.0;             // pelvis z axis vs world z
  int steps_without_both_feet = 0;        // t >= 1 s steps with a foot under 5 N
  double joint_err_max = 0.0;             // actuated joint vs keyframe ctrl
  int saturated = 0;  // (actuator, step) pairs at >= 99 % of the limit, t >= 1 s
  bool finite = true;
  std::vector<mjtNum> final_qpos;
};

StanceMetrics RunStance(const mjModel* m, mjData* d) {
  StanceMetrics r;
  const int key = mj_name2id(m, mjOBJ_KEY, "stand");
  mj_resetDataKeyframe(m, d, key);
  mj_forward(m, d);

  const int pelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");
  const int floor = mj_name2id(m, mjOBJ_GEOM, "floor");
  std::array<int, 2> sole{};
  for (std::size_t k = 0; k < 2; ++k)
    sole[k] = mj_name2id(m, mjOBJ_GEOM, kSole[k]);

  std::vector<int> qadr(static_cast<std::size_t>(m->nu));
  std::vector<double> limit(static_cast<std::size_t>(m->nu));
  for (int a = 0; a < m->nu; ++a) {
    const int j = m->actuator_trnid[2 * a];
    qadr[static_cast<std::size_t>(a)] = m->jnt_qposadr[j];
    limit[static_cast<std::size_t>(a)] = m->jnt_actfrcrange[2 * j + 1];
  }
  const mjtNum* target = m->key_ctrl + key * m->nu;

  const double z0 = d->xpos[3 * pelvis + 2];
  const int steps = static_cast<int>(std::lround(kSimSeconds / m->opt.timestep));
  const int settle = static_cast<int>(std::lround(kSettleSeconds / m->opt.timestep));
  std::array<std::array<double, 2>, 2> sole_ref{};
  std::array<mjtNum, 6> f6{};

  for (int i = 0; i < steps; ++i) {
    mj_step(m, d);
    for (int q = 0; q < m->nq; ++q) {
      if (!std::isfinite(d->qpos[q])) {
        r.finite = false;
        r.final_qpos.assign(d->qpos, d->qpos + m->nq);
        return r;
      }
    }
    if (i < settle)
      continue;
    for (int a = 0; a < m->nu; ++a) {
      if (std::fabs(d->actuator_force[a]) >=
          kSaturationFraction * limit[static_cast<std::size_t>(a)]) {
        ++r.saturated;
      }
    }
    if (i == settle) {
      for (std::size_t k = 0; k < 2; ++k) {
        sole_ref[k] = {d->geom_xpos[3 * sole[k]], d->geom_xpos[3 * sole[k] + 1]};
      }
    }
    for (std::size_t k = 0; k < 2; ++k) {
      const double dx = d->geom_xpos[3 * sole[k]] - sole_ref[k][0];
      const double dy = d->geom_xpos[3 * sole[k] + 1] - sole_ref[k][1];
      r.foot_slip_max[k] = std::max(r.foot_slip_max[k], std::hypot(dx, dy));
    }
    r.base_height_dev_max =
        std::max(r.base_height_dev_max, std::fabs(d->xpos[3 * pelvis + 2] - z0));
    const double cos_tilt = std::clamp(d->xmat[9 * pelvis + 8], -1.0, 1.0);
    r.base_tilt_max = std::max(r.base_tilt_max, std::acos(cos_tilt));
    for (int a = 0; a < m->nu; ++a) {
      r.joint_err_max = std::max(r.joint_err_max,
                                 std::fabs(d->qpos[qadr[static_cast<std::size_t>(a)]] - target[a]));
    }
    std::array<double, 2> normal{0.0, 0.0};
    for (int c = 0; c < d->ncon; ++c) {
      const mjContact& con = d->contact[c];
      if (con.geom[0] != floor && con.geom[1] != floor)
        continue;
      const int other = con.geom[0] == floor ? con.geom[1] : con.geom[0];
      for (std::size_t k = 0; k < 2; ++k) {
        if (other != sole[k])
          continue;
        mj_contactForce(m, d, c, f6.data());
        normal[k] += f6[0];
      }
    }
    if (!(normal[0] > kContactNormalMinN && normal[1] > kContactNormalMinN)) {
      ++r.steps_without_both_feet;
    }
  }
  r.final_qpos.assign(d->qpos, d->qpos + m->nq);
  return r;
}

TEST_F(G1FloatingModel, StandsStillForTenSeconds) {
  const StanceMetrics r = RunStance(m_, d_);
  ASSERT_TRUE(r.finite) << "state diverged to NaN/inf";
  std::cout << "[  METRIC ] foot_slip_max L/R = " << r.foot_slip_max[0] << " / "
            << r.foot_slip_max[1] << " m, base_height_dev_max = " << r.base_height_dev_max
            << " m, base_tilt_max = " << r.base_tilt_max
            << " rad, steps_without_both_feet = " << r.steps_without_both_feet
            << ", joint_err_max = " << r.joint_err_max << " rad, saturated = " << r.saturated
            << "\n";
  EXPECT_LE(r.foot_slip_max[0], kFootSlipMaxM);
  EXPECT_LE(r.foot_slip_max[1], kFootSlipMaxM);
  EXPECT_LE(r.base_height_dev_max, kBaseHeightDevMaxM);
  EXPECT_LE(r.base_tilt_max, kBaseTiltMaxRad);
  EXPECT_EQ(r.steps_without_both_feet, 0);
  EXPECT_LE(r.joint_err_max, kJointErrMaxRad);
  EXPECT_EQ(r.saturated, 0);
}

TEST_F(G1FloatingModel, KeyframeResetIsBitExact) {
  const int key = Id(mjOBJ_KEY, "stand");
  ASSERT_GE(key, 0);
  const mjtNum* key_qpos = m_->key_qpos + key * m_->nq;
  const mjtNum* key_qvel = m_->key_qvel + key * m_->nv;
  for (int rep = 0; rep < 3; ++rep) {
    mj_resetDataKeyframe(m_, d_, key);
    EXPECT_EQ(std::memcmp(d_->qpos, key_qpos, sizeof(mjtNum) * static_cast<std::size_t>(m_->nq)), 0)
        << "reset " << rep;
    EXPECT_EQ(std::memcmp(d_->qvel, key_qvel, sizeof(mjtNum) * static_cast<std::size_t>(m_->nv)), 0)
        << "reset " << rep;
    for (int i = 0; i < 300; ++i)
      mj_step(m_, d_);  // leave the keyframe before the next reset
  }
}

TEST_F(G1FloatingModel, ThreeRunsAreDeterministic) {
  std::array<StanceMetrics, 3> runs;
  for (auto& r : runs)
    r = RunStance(m_, d_);
  for (std::size_t i = 1; i < runs.size(); ++i) {
    ASSERT_TRUE(runs[i].finite);
    for (std::size_t k = 0; k < 2; ++k) {
      EXPECT_NEAR(runs[i].foot_slip_max[k], runs[0].foot_slip_max[k], kDeterminismTol)
          << "run " << i;
    }
    EXPECT_NEAR(runs[i].base_height_dev_max, runs[0].base_height_dev_max, kDeterminismTol);
    EXPECT_NEAR(runs[i].base_tilt_max, runs[0].base_tilt_max, kDeterminismTol);
    EXPECT_NEAR(runs[i].joint_err_max, runs[0].joint_err_max, kDeterminismTol);
    EXPECT_EQ(runs[i].steps_without_both_feet, runs[0].steps_without_both_feet);
    EXPECT_EQ(runs[i].saturated, runs[0].saturated);
    EXPECT_EQ(std::memcmp(runs[i].final_qpos.data(), runs[0].final_qpos.data(),
                          sizeof(mjtNum) * runs[0].final_qpos.size()),
              0)
        << "run " << i << " final qpos differs bit-wise";
  }
}

}  // namespace
