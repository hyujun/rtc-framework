// ── test_g1_floating_stance.cpp ──────────────────────────────────────────────
// The floating-base humanoid plant model (W0-F01, #975): what the derived MJCF
// robot_descriptions/robots/g1/mjcf/g1_29dof_floating.xml must contain, and
// that it stands still on its own under its position servos.
//
// Plain MuJoCo C API — no simulator node, no ROS. The model is the fixture and
// comes through the ament index (robot_descriptions is data-only, ARCH-5), so
// what is pinned here is the asset's contract, not this package's code:
//   model  — freejoint 1 · sole box 2 (the upstream 4 spheres per foot) with a
//            contact sensor, a framepos and an FT reference site each (#977) ·
//            pelvis framepos/framequat/framelinvel/frameangvel · gyro +
//            accelerometer on both IMU sites · <position> 23 with the agreed
//            gains (wrists rigid) · keyframe "stand" · implicitfast · <option
//            timestep> 0.002 = the control period the simulator divides by its
//            n_substeps (3) — the stance below steps at that substep, 0.002/3
//   stance — mj_resetDataKeyframe("stand"), then pure mj_step for 10 s: the
//            feet do not slip, the pelvis stays up and level, both feet carry
//            load, no joint sags away from its target, no actuator saturates,
//            and the run is bit-reproducible
//   IMU    — the same stance at rest: both accelerometers read the specific
//            force R_siteᵀ·(0, 0, g₀) and both gyros ~0, within controls
//            measured once on this MJCF
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
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

constexpr const char* kModelRel = "/robots/g1/mjcf/g1_29dof_floating.xml";

// ── Sprint Contract v3 (#975) ────────────────────────────────────────────────
constexpr double kControlPeriod =
    0.002;  // <option timestep>: what the simulator reads as its control period
constexpr int kSimSubsteps = 3;  // n_substeps of the g1_loco sim YAML
constexpr double kTimestep =
    kControlPeriod / kSimSubsteps;  // the physics substep the stance runs at
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
    ASSERT_NEAR(m_->opt.timestep, kControlPeriod, 1e-12) << "asset contract: control period";
    // Step at the simulator's physics substep, not at the control period the
    // asset declares — RunStance is then the same integration the sim runs.
    m_->opt.timestep = kTimestep;
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
}

TEST_F(G1FloatingModel, SoleContactSensorsFramePosAndReferenceSites) {
  // The simulator's contact wrench lane needs: a mjSENS_CONTACT of 17 values
  // (data="found force torque dist pos normal tangent", num=1, netforce) named
  // <target>_contact, and a site <target>_ft_site to express the wrench in.
  constexpr int kContactSensorDim = 17;
  for (std::size_t k = 0; k < 2; ++k) {
    const std::string stem = std::string(kSole[k]);
    const int sole = Id(mjOBJ_GEOM, kSole[k]);
    const int foot = Id(mjOBJ_BODY, kAnkleRoll[k]);
    const int contact = Id(mjOBJ_SENSOR, (stem + "_contact").c_str());
    ASSERT_GE(contact, 0) << stem;
    EXPECT_EQ(m_->sensor_type[contact], mjSENS_CONTACT) << stem;
    EXPECT_EQ(m_->sensor_dim[contact], kContactSensorDim) << stem;
    EXPECT_EQ(m_->sensor_objtype[contact], mjOBJ_GEOM) << stem;
    EXPECT_EQ(m_->sensor_objid[contact], sole) << stem << " must watch its own sole geom";
    const int pos = Id(mjOBJ_SENSOR, (stem + "_pos").c_str());
    ASSERT_GE(pos, 0) << stem;
    EXPECT_EQ(m_->sensor_type[pos], mjSENS_FRAMEPOS) << stem;
    EXPECT_EQ(m_->sensor_objtype[pos], mjOBJ_GEOM) << stem;
    EXPECT_EQ(m_->sensor_objid[pos], sole) << stem;
    const int site = Id(mjOBJ_SITE, (stem + "_ft_site").c_str());
    ASSERT_GE(site, 0) << stem;
    EXPECT_EQ(m_->site_bodyid[site], foot) << stem << " site hangs under " << kAnkleRoll[k];
    // On the sole's bottom face: box centre z minus its half-height, in the foot frame.
    EXPECT_NEAR(m_->site_pos[3 * site + 0], m_->geom_pos[3 * sole + 0], 1e-12) << stem;
    EXPECT_NEAR(m_->site_pos[3 * site + 1], m_->geom_pos[3 * sole + 1], 1e-12) << stem;
    EXPECT_NEAR(m_->site_pos[3 * site + 2],
                m_->geom_pos[3 * sole + 2] - m_->geom_size[3 * sole + 2], 1e-12)
        << stem;
  }
  EXPECT_EQ(m_->nsensor, 12)
      << "4 ground-truth + 4 IMU + 2 sole contact + 2 sole framepos, nothing else";
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
  // SetUp already asserted <option timestep> == kControlPeriod and lowered the
  // in-memory step to the physics substep; this pins the substep the stance uses.
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

// ═══════════════════════════════════════════════════════════════════════════
// IMU frame under static gravity
// ═══════════════════════════════════════════════════════════════════════════
// The robot held on the "stand" keyframe by its own position servos (the
// stance above, noise-free: MuJoCo does not add the MJCF noise= values). Once
// it has settled, each IMU site is at rest, so its accelerometer reads the
// specific force R_siteᵀ·(0, 0, g₀) — up in the site frame, not down — and its
// gyro reads zero. What is pinned is the frame and sign convention a consumer
// relies on; the residual motion of the servoed stance is what the bounds
// leave room for.

constexpr double kFrameSettleSeconds = 1.0;
constexpr double kFrameWindowSeconds = 2.0;
// Controls, measured once on this MJCF (MuJoCo 3.7.0, every physics substep of
// the 2 s window, pelvis then torso) before the bounds were written, and not
// moved since: the residual sway of the servoed stance. A change to the asset,
// the servos or MuJoCo that moves the robot more at rest fails here first.
constexpr std::array<double, 2> kStaticAccelErrMax{0.0066830339849574764, 0.011104294788760453};
constexpr std::array<double, 2> kStaticGyroNormMax{0.010827915362533231, 0.012966093697911824};
constexpr double kStaticBoundTol = 1e-9;

struct ImuFrameMetrics {
  double accel_err_max = 0.0;  // max ‖a_site − R_siteᵀ·(0, 0, g₀)‖ [m/s²]
  double gyro_norm_max = 0.0;  // max ‖ω_site‖ [rad/s]
  double accel_z_min = 0.0;    // min site-z component of the specific force [m/s²]
  int samples = 0;
};

std::array<ImuFrameMetrics, 2> RunStaticImuFrame(const mjModel* m, mjData* d) {
  const std::array<const char*, 2> sites{"imu_in_pelvis", "imu_in_torso"};
  const std::array<const char*, 2> gyros{"imu-pelvis-angular-velocity",
                                         "imu-torso-angular-velocity"};
  const std::array<const char*, 2> accels{"imu-pelvis-linear-acceleration",
                                          "imu-torso-linear-acceleration"};
  std::array<int, 2> site_id{}, gyro_adr{}, accel_adr{};
  for (std::size_t k = 0; k < 2; ++k) {
    site_id[k] = mj_name2id(m, mjOBJ_SITE, sites[k]);
    gyro_adr[k] = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, gyros[k])];
    accel_adr[k] = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, accels[k])];
  }
  const double g0 = -m->opt.gravity[2];

  mj_resetDataKeyframe(m, d, mj_name2id(m, mjOBJ_KEY, "stand"));
  mj_forward(m, d);
  const int settle = static_cast<int>(std::lround(kFrameSettleSeconds / m->opt.timestep));
  const int window = static_cast<int>(std::lround(kFrameWindowSeconds / m->opt.timestep));
  for (int i = 0; i < settle; ++i)
    mj_step(m, d);

  std::array<ImuFrameMetrics, 2> r{};
  for (auto& s : r)
    s.accel_z_min = std::numeric_limits<double>::infinity();
  for (int i = 0; i < window; ++i) {
    // mj_step evaluates the sensors and site_xmat on the same pre-integration
    // state, so the pair read after it belongs together.
    mj_step(m, d);
    for (std::size_t k = 0; k < 2; ++k) {
      // site_xmat is R_world←site, row-major: (R_siteᵀ·(0, 0, g₀))_c = R[2][c]·g₀.
      const mjtNum* rot = d->site_xmat + 9 * site_id[k];
      const mjtNum* acc = d->sensordata + accel_adr[k];
      const mjtNum* gyr = d->sensordata + gyro_adr[k];
      double err2 = 0.0, w2 = 0.0;
      for (int c = 0; c < 3; ++c) {
        const double diff = acc[c] - rot[6 + c] * g0;
        err2 += diff * diff;
        w2 += gyr[c] * gyr[c];
      }
      r[k].accel_err_max = std::max(r[k].accel_err_max, std::sqrt(err2));
      r[k].gyro_norm_max = std::max(r[k].gyro_norm_max, std::sqrt(w2));
      r[k].accel_z_min = std::min(r[k].accel_z_min, static_cast<double>(acc[2]));
      ++r[k].samples;
    }
  }
  return r;
}

TEST_F(G1FloatingModel, ImuReadsStaticGravityInTheSiteFrame) {
  const auto r = RunStaticImuFrame(m_, d_);
  const std::array<const char*, 2> label{"pelvis", "torso"};
  for (std::size_t k = 0; k < 2; ++k) {
    std::printf(
        "[  METRIC ] imu %s: accel_err_max = %.17g m/s^2, gyro_norm_max = %.17g rad/s, "
        "accel_z_min = %.17g m/s^2, samples = %d\n",
        label[k], r[k].accel_err_max, r[k].gyro_norm_max, r[k].accel_z_min, r[k].samples);
    EXPECT_EQ(r[k].samples, static_cast<int>(std::lround(kFrameWindowSeconds / kTimestep)));
    EXPECT_LE(r[k].accel_err_max, kStaticAccelErrMax[k] + kStaticBoundTol)
        << label[k] << ": accelerometer is not R_siteᵀ·(0, 0, g₀) at rest";
    EXPECT_LE(r[k].gyro_norm_max, kStaticGyroNormMax[k] + kStaticBoundTol) << label[k];
    EXPECT_GT(r[k].accel_z_min, 0.0) << label[k] << ": specific force must point up the site z";
  }
}

}  // namespace
