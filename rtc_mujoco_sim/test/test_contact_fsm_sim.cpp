// ── test_contact_fsm_sim.cpp ─────────────────────────────────────────────────
// The contact FSM + InEKF sequencer (rtc_controllers, test-only dependency)
// against MuJoCo contact truth, in process: MuJoCoSimulator driven by
// StepForTest, the public floating-base humanoid MJCF standing on its own
// position servos (pd_feedforward with zero feedforward — no gravity
// compensation, so the stance is the servos' work), the shipped IMU export
// noise model, IMU = the pelvis IMU site.
//
//   IMU      gyro / accelerometer: the mean of the exact sensors over the
//            tick's substeps plus the export model's noise on the tick's last
//            draw. Read once per tick, the welded pelvis's impulsive specific
//            force (> 100 m/s² spikes) aliases: integrating it gave 0.8 m/s
//            of velocity error per crane cycle, the substep mean 0.007 m/s.
//   FK       r_i = R_Iᵀ(p_site − p_I) of each sole reference site, J_i =
//            R_Iᵀ ∂p_site/∂q over the joint columns (mj_jacSite), ṙ_i = J_i q̇
//   truth    each sole's contact sensor `found`, debounced: a change counts
//            once it holds 20 ms (10 control ticks)
//   start    the filter initialises at 1.0 s from a 0.5 s stationary window
//            with both soles as stance points — standing on the ground
//
//   S1 quiet      10 s: no transition after acquisition, precision = recall = 1
//   S2 push       ±100 N lateral, ±60 N fore-aft on the pelvis for 0.1 s
//                 (truth must show no transition): no stance exit, no slip /
//                 unknown entry
//   S3 crane      pelvis welded to a mocap body (wrapper XML written at run
//                 time around the public model), standing 4 mm below the
//                 keyframe height so the soles carry load, lifted until both
//                 leave the ground for ≤ 0.4 s (checked), set down, 1.5 s
//                 apart, at 0.1 and 0.2 m/s; planned = truth shifted −40 / 0 /
//                 +40 ms (6 runs): precision ≥ 0.999, recall ≥ 0.99, no false
//                 exit, touchdown delay ≤ 150 ms, liftoff delay ≤ 4 ms
//                 (shift ≤ 0) / ≤ 44 ms (+40). The false exits of the 0.2 m/s
//                 run with the plan 40 ms early are reported only: a declared
//                 limit (both soles anchor in the air while descending faster
//                 than v_slip, then cycle stance ↔ slip after touchdown)
//   determinism   one crane recording made twice → the same output stream
//   S4 friction   report only: μ 0.3 and a 100 N lateral push — slip spans in
//                 truth vs detections
//
// StepForTest runs one physics substep, so a control tick is n_substeps of
// them; the IMU export model then draws once per substep and its bias walks
// √3 faster than on the SimLoop (immaterial next to the initial bias).
// Thresholds are pre-registered and do not move with the measurement.
// ──────────────────────────────────────────────────────────────────────────────
#include "rtc_controllers/estimation/contact_fsm.hpp"
#include "sim_config_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace rtc {
namespace {

using estimation::ContactFsm;
using estimation::ContactFsmStatus;
using estimation::ContactInekf;
using estimation::ContactInekfSequencer;
using estimation::ContactKinematics;
using estimation::ContactMode;
using estimation::ContactStepInput;
using estimation::InekfStatus;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

constexpr const char* kModelRel = "/robots/g1/mjcf/g1_29dof_floating.xml";
constexpr const char* kMeshRel = "/robots/g1/unitree_g1_29dof/meshes";
constexpr double kPeriod = 0.002;
constexpr int kSubsteps = 3;
constexpr int kSlots = 2;
constexpr int kInitTick = 500;      // 1.0 s
constexpr int kWindowTicks = 250;   // 0.5 s stationary window
constexpr int kEvalTick = 750;      // 1.5 s: metrics from here
constexpr int kDebounceTicks = 10;  // 20 ms
constexpr int kGuardTicks = 30;     // ±60 ms around a truth transition
constexpr double kFalseExitSpeed = 0.02;
constexpr double kEps = 1e-12;

const std::array<const char*, kSlots> kSoleSite{"left_sole_ft_site", "right_sole_ft_site"};
const std::array<const char*, kSlots> kSoleGeom{"left_sole", "right_sole"};
const std::array<const char*, kSlots> kSoleContact{"left_sole_contact", "right_sole_contact"};
// The model's 23 actuated joints (the fixture group commands all of them).
const std::array<const char*, 23> kJoints{
    "left_hip_pitch_joint",      "left_hip_roll_joint",        "left_hip_yaw_joint",
    "left_knee_joint",           "left_ankle_pitch_joint",     "left_ankle_roll_joint",
    "right_hip_pitch_joint",     "right_hip_roll_joint",       "right_hip_yaw_joint",
    "right_knee_joint",          "right_ankle_pitch_joint",    "right_ankle_roll_joint",
    "waist_yaw_joint",           "waist_roll_joint",           "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    "left_elbow_joint",          "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",  "right_elbow_joint"};

std::string Share() {
  return ament_index_cpp::get_package_share_directory("robot_descriptions");
}

/// One control tick as the estimator sees it, plus the truth.
struct Row {
  Vec3 gyro;
  Vec3 accel;
  std::array<ContactKinematics, kSlots> kin;
  std::array<Vec3, kSlots> rdot;
  std::array<bool, kSlots> found{};
  std::array<double, kSlots> sole_speed{};  // horizontal, world [m/s]
};

/// Pelvis height relative to the keyframe's: `kCraneDown` while standing
/// (the soles carry load — the weld alone would hold the pelvis at the
/// keyframe height with the soles barely touching), `top` at the top.
constexpr double kCraneDown = -0.004;

struct Crane {
  double speed{0.1};  // [m/s]
  double top{0.012};  // [m]
};

struct Push {
  double t0{3.0};
  int axis{1};  // world force axis (0 x, 1 y)
  double force{100.0};
};

struct SimSpec {
  double seconds{10.0};
  bool crane{false};
  Crane c;
  bool push{false};
  Push p;
  double friction{0.0};  // 0 = the model's
};

/// The crane wrapper: the public model, absolute mesh directory (MuJoCo
/// resolves it against the top-level file), its integrator, and a mocap body
/// welded to the pelvis frame (relpose identity).
std::filesystem::path CraneDir() {
  return std::filesystem::temp_directory_path() /
         ("rtc_contact_fsm_crane_" + std::to_string(::getpid()));
}

std::string WriteCraneModel() {
  const std::filesystem::path dir = CraneDir();
  std::filesystem::create_directories(dir);
  const std::filesystem::path file = dir / "crane.xml";
  std::ofstream out(file);
  out << "<mujoco model=\"crane\">\n"
      << "  <include file=\"" << Share() << kModelRel << "\"/>\n"
      << "  <compiler meshdir=\"" << Share() << kMeshRel
      << "\"/>\n"
      // The simulator takes explicit <option> attributes from the top-level
      // file only, so the included model's integrator is restated here.
      << "  <option integrator=\"implicitfast\"/>\n"
      << "  <worldbody><body name=\"crane\" mocap=\"true\"/></worldbody>\n"
      << "  <equality><weld body1=\"crane\" body2=\"pelvis\" relpose=\"0 0 0 1 0 0 0\" "
         "solref=\"0.004 1\"/></equality>\n"
      << "</mujoco>\n";
  return file.string();
}

class Recorder {
 public:
  explicit Recorder(const SimSpec& spec) : spec_(spec) {
    const std::string path = spec.crane ? WriteCraneModel() : Share() + kModelRel;
    auto cfg = test::HeadlessConfig(path, 10.0);
    cfg.physics_timestep = kPeriod;
    cfg.n_substeps = kSubsteps;
    cfg.reset_keyframe = "stand";
    JointGroupConfig g = test::RobotGroup("body", {kJoints.begin(), kJoints.end()});
    g.sensor_names = {"imu-pelvis-angular-velocity", "imu-pelvis-linear-acceleration"};
    g.imu_noise.enabled = true;
    g.imu_noise.seed = 20261114;
    g.imu_noise.gyro = {5e-4, 1e-3, 1e-5};
    g.imu_noise.accel = {1e-2, 5e-2, 1e-4};
    cfg.groups.push_back(std::move(g));
    sim_ = std::make_unique<MuJoCoSimulator>(cfg);
  }

  ~Recorder() {
    if (spec_.crane) {
      std::error_code ec;
      std::filesystem::remove_all(CraneDir(), ec);
    }
  }

  Recorder(const Recorder&) = delete;
  Recorder& operator=(const Recorder&) = delete;

  /// Run the scenario; false (with a message) if the simulator is not usable.
  bool Record(std::vector<Row>& rows, std::string& why) {
    if (!sim_->Initialize()) {
      why = "Initialize failed";
      return false;
    }
    m_ = sim_->GetModel();
    d_ = sim_->GetData();
    imu_ = mj_name2id(m_, mjOBJ_SITE, "imu_in_pelvis");
    pelvis_ = mj_name2id(m_, mjOBJ_BODY, "pelvis");
    for (std::size_t i = 0; i < kSlots; ++i) {
      site_[i] = mj_name2id(m_, mjOBJ_SITE, kSoleSite[i]);
      geom_[i] = mj_name2id(m_, mjOBJ_GEOM, kSoleGeom[i]);
      const int s = mj_name2id(m_, mjOBJ_SENSOR, kSoleContact[i]);
      if (site_[i] < 0 || geom_[i] < 0 || s < 0) {
        why = "missing sole site / geom / contact sensor";
        return false;
      }
      contact_adr_[i] = m_->sensor_adr[s];
    }
    if (spec_.crane) {  // the restated integrator must stay the public model's
      std::array<char, 1000> err{};
      mjModel* pub = mj_loadXML((Share() + kModelRel).c_str(), nullptr, err.data(),
                                static_cast<int>(err.size()));
      const bool same = pub != nullptr && pub->opt.integrator == m_->opt.integrator;
      mj_deleteModel(pub);
      if (!same) {
        why = "crane wrapper integrator differs from the public model's";
        return false;
      }
    }
    if (imu_ < 0 || pelvis_ < 0 || (spec_.crane && m_->nmocap != 1)) {
      why = "missing IMU site / pelvis / crane";
      return false;
    }
    for (const char* j : kJoints) {
      const int id = mj_name2id(m_, mjOBJ_JOINT, j);
      if (id < 0) {
        why = std::string("missing joint ") + j;
        return false;
      }
      dofs_.push_back(m_->jnt_dofadr[id]);
    }
    if (spec_.friction > 0.0) {
      for (int gid = 0; gid < m_->ngeom; ++gid) {
        if (m_->geom_bodyid[gid] == 0 || gid == geom_[0] || gid == geom_[1]) {
          if (!sim_->SetGeomFrictionForTest(gid, spec_.friction)) {
            why = "friction seam refused";
            return false;
          }
        }
      }
    }
    const std::vector<double> q = sim_->GetQposForTest();
    const std::vector<double> key = sim_->GetKeyQposForTest("stand");
    std::vector<double> ctrl(kJoints.size());
    for (std::size_t j = 0; j < kJoints.size(); ++j) {
      const int id = mj_name2id(m_, mjOBJ_JOINT, kJoints[j]);
      ctrl[j] = key[static_cast<std::size_t>(m_->jnt_qposadr[id])];
    }
    sim_->SetControlMode(0, JointControlMode::kPdFeedforward);
    sim_->StageCommand(0, JointControlMode::kPdFeedforward, ctrl, {}, {}, {});
    const std::array<double, 3> base{q[0], q[1], q[2]};
    const std::array<double, 4> quat{q[3], q[4], q[5], q[6]};
    const int n = static_cast<int>(spec_.seconds / kPeriod);
    rows.resize(static_cast<std::size_t>(n));
    std::vector<mjtNum> jacp(static_cast<std::size_t>(3 * m_->nv));
    std::vector<mjtNum> jaci(static_cast<std::size_t>(3 * m_->nv));
    for (int k = 0; k < n; ++k) {
      const double t = k * kPeriod;
      if (spec_.crane) {
        std::array<double, 3> pos = base;
        pos[2] = base[2] + CraneHeight(t);
        if (!sim_->SetMocapPoseForTest(0, pos, quat)) {
          why = "mocap seam refused";
          return false;
        }
      }
      if (spec_.push) {
        std::array<double, 6> w{};
        if (t >= spec_.p.t0 && t < spec_.p.t0 + 0.1) {
          w[static_cast<std::size_t>(spec_.p.axis)] = spec_.p.force;
        }
        (void)sim_->SetExternalForce(pelvis_, w);
      }
      Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
      std::vector<double> raw;
      for (int s = 0; s < kSubsteps; ++s) {
        sim_->StageCommand(0, JointControlMode::kPdFeedforward, ctrl, {}, {}, {});
        sim_->StepForTest();
        raw = sim_->GetSensorDataForTest(0);
        for (int c = 0; c < 6; ++c) {
          mean[c] += raw[static_cast<std::size_t>(c)] / kSubsteps;
        }
      }
      Row& r = rows[static_cast<std::size_t>(k)];
      // Tick mean of the exact sensors + the export model's noise on the last
      // substep's draw (export − sensordata).
      const std::vector<double> ex = sim_->GetSensorExportForTest(0);
      for (int c = 0; c < 6; ++c) {
        mean[c] += ex[static_cast<std::size_t>(c)] - raw[static_cast<std::size_t>(c)];
      }
      r.gyro = mean.head<3>();
      r.accel = mean.tail<3>();
      const Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> Ri(d_->site_xmat +
                                                                              9 * imu_);
      const Mat3 R = Ri;
      const Vec3 pi(d_->site_xpos[3 * imu_], d_->site_xpos[3 * imu_ + 1],
                    d_->site_xpos[3 * imu_ + 2]);
      mj_jacSite(m_, d_, jaci.data(), nullptr, imu_);
      std::array<mjtNum, 6> vimu{};
      mj_objectVelocity(m_, d_, mjOBJ_SITE, imu_, vimu.data(), 0);
      for (int i = 0; i < kSlots; ++i) {
        const auto si = static_cast<std::size_t>(i);
        const int st = site_[si];
        const Vec3 ps(d_->site_xpos[3 * st], d_->site_xpos[3 * st + 1], d_->site_xpos[3 * st + 2]);
        r.kin[si].position = R.transpose() * (ps - pi);
        mj_jacSite(m_, d_, jacp.data(), nullptr, st);
        r.kin[si].jacobian.resize(3, static_cast<Eigen::Index>(dofs_.size()));
        for (std::size_t c = 0; c < dofs_.size(); ++c) {
          const int dof = dofs_[c];
          Vec3 col;
          for (int a = 0; a < 3; ++a) {
            col[a] = jacp[static_cast<std::size_t>(a * m_->nv + dof)] -
                     jaci[static_cast<std::size_t>(a * m_->nv + dof)];
          }
          r.kin[si].jacobian.col(static_cast<Eigen::Index>(c)) = R.transpose() * col;
        }
        // ṙ = d/dt Rᵀ(p_s − p_I) = Rᵀ((v_s − v_I) − ω × (p_s − p_I)), from the
        // site velocities of the same instant as the positions (after mj_step
        // positions and velocities are pre-integration, qvel is not).
        std::array<mjtNum, 6> vs{};
        mj_objectVelocity(m_, d_, mjOBJ_SITE, st, vs.data(), 0);
        const Vec3 dv = Vec3(vs[3], vs[4], vs[5]) - Vec3(vimu[3], vimu[4], vimu[5]);
        r.rdot[si] = R.transpose() * (dv - Vec3(vimu[0], vimu[1], vimu[2]).cross(ps - pi));
        r.found[si] = d_->sensordata[contact_adr_[si]] > 0.5;
        std::array<mjtNum, 6> vel{};
        mj_objectVelocity(m_, d_, mjOBJ_GEOM, geom_[si], vel.data(), 0);
        r.sole_speed[si] = std::hypot(vel[3], vel[4]);
      }
    }
    return true;
  }

 private:
  /// Pelvis height offset of the crane: down to 1.5 s, then per cycle up to
  /// `top` at `speed`, hover 50 ms, back down, stand 1.5 s.
  [[nodiscard]] double CraneHeight(double t) const {
    if (t < 1.5) {
      return kCraneDown;
    }
    const double travel = spec_.c.top - kCraneDown;
    const double tr = travel / spec_.c.speed;
    const double hover = 0.05;
    const double cycle = 2.0 * tr + hover + 1.5;
    const double x = std::fmod(t - 1.5, cycle);
    if (x < tr) {
      return kCraneDown + spec_.c.speed * x;
    }
    if (x < tr + hover) {
      return spec_.c.top;
    }
    if (x < 2.0 * tr + hover) {
      return spec_.c.top - spec_.c.speed * (x - tr - hover);
    }
    return kCraneDown;
  }

  SimSpec spec_;
  std::unique_ptr<MuJoCoSimulator> sim_;
  const mjModel* m_{nullptr};
  const mjData* d_{nullptr};
  int imu_{-1};
  int pelvis_{-1};
  std::array<int, kSlots> site_{};
  std::array<int, kSlots> geom_{};
  std::array<int, kSlots> contact_adr_{};
  std::vector<int> dofs_;
};

/// Debounced truth: a change of `found` counts once it holds kDebounceTicks.
std::vector<std::array<bool, kSlots>> Truth(const std::vector<Row>& rows) {
  const auto n = rows.size();
  std::vector<std::array<bool, kSlots>> gt(n);
  for (int i = 0; i < kSlots; ++i) {
    const auto si = static_cast<std::size_t>(i);
    bool state = rows.front().found[si];
    int run = 0;
    for (std::size_t k = 0; k < n; ++k) {
      const bool raw = rows[k].found[si];
      run = raw != state ? run + 1 : 0;
      if (run >= kDebounceTicks) {
        for (std::size_t j = k + 1 - kDebounceTicks; j <= k; ++j) {
          gt[j][si] = raw;
        }
        state = raw;
        run = 0;
      }
      gt[k][si] = state;
    }
  }
  return gt;
}

ContactInekf::Config SimCoreConfig() {
  ContactInekf::Config c;
  c.gravity = 9.81;
  c.gyro_noise_density = 5e-4 * std::sqrt(kPeriod);
  c.accel_noise_density = 1e-2 * std::sqrt(kPeriod);
  c.gyro_bias_walk = 1e-5;
  c.accel_bias_walk = 1e-4;
  c.contact_noise_density = 1e-3;
  c.joint_position_sigma = 2e-3;
  c.kinematic_sigma = 2e-3;
  c.contact_gate_chi2 = 16.27;
  c.prior_velocity_sigma = 1e-2;
  c.prior_position_sigma = 1e-2;
  c.prior_yaw_sigma = 1e-3;
  c.prior_gyro_bias_sigma = 1e-3;
  c.prior_accel_bias_sigma = 5e-2;
  c.stationary_accel_tolerance = 0.5;
  c.stationary_gyro_tolerance = 0.05;
  return c;
}

// The pre-registered thresholds — the same values as rtc_controllers'
// synthetic fixture (FsmFixture / CoreFixture there); both are frozen by the
// sprint contract, not tuned here.
ContactFsm::Config SimFsmConfig() {
  ContactFsm::Config c;
  c.period = kPeriod;
  c.enter_speed = 0.05;
  c.slip_speed = 0.15;
  c.nis_low = 6.25;
  c.nis_high = 11.34;
  c.enter_dwell = 10e-3;
  c.confirm_dwell = 10e-3;
  c.release_dwell = 4e-3;
  c.slip_dwell = 6e-3;
  c.nis_dwell = 6e-3;
  c.recover_dwell = 20e-3;
  c.evidence_time_constant = 20e-3;
  c.unknown_noise_scale = 100.0;
  return c;
}

struct Replay {
  std::vector<std::array<ContactMode, kSlots>> mode;
  std::vector<std::array<double, kSlots>> evidence;
  std::vector<std::array<bool, kSlots>> planned;
  int bad_status{0};
  int contract{0};  // anchor-holding mode ⇔ core slot active
};

/// Feed the recording through a fresh sequencer; planned = truth shifted by
/// `shift` ticks (positive: planned later), or all on when `all_planned`.
Replay Estimate(const std::vector<Row>& rows, const std::vector<std::array<bool, kSlots>>& gt,
                int shift, bool all_planned) {
  const int n = static_cast<int>(rows.size());
  Replay out;
  out.mode.resize(rows.size());
  out.evidence.resize(rows.size());
  out.planned.resize(rows.size());
  ContactInekfSequencer seq;
  seq.Init(SimCoreConfig(), SimFsmConfig(), kSlots);
  estimation::StationaryImuAccumulator acc;
  for (int k = 0; k < n; ++k) {
    const Row& r = rows[static_cast<std::size_t>(k)];
    std::uint32_t mask = 0;
    for (int i = 0; i < kSlots; ++i) {
      const int src = std::clamp(k - shift, 0, n - 1);
      const bool p = all_planned || gt[static_cast<std::size_t>(src)][static_cast<std::size_t>(i)];
      out.planned[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = p;
      mask |= p ? (1U << i) : 0U;
    }
    if (k < kInitTick) {
      if (k >= kInitTick - kWindowTicks) {
        (void)acc.Add(r.gyro, r.accel);
      }
      continue;
    }
    if (k == kInitTick) {
      const std::array<Vec3, kSlots> pts{r.kin[0].position, r.kin[1].position};
      if (seq.Initialize(acc.MeanGyro(), acc.MeanAccel(), kWindowTicks * kPeriod, 0.0, pts) !=
          InekfStatus::kOk) {
        ++out.bad_status;
        return out;
      }
      continue;
    }
    ContactStepInput in;
    in.gyro = r.gyro;
    in.accel = r.accel;
    in.kinematics = r.kin;
    in.joint_velocity = r.rdot;
    in.planned_mask = mask;
    if (seq.Step(in) != ContactFsmStatus::kOk) {
      ++out.bad_status;
    }
    for (int i = 0; i < kSlots; ++i) {
      const auto si = static_cast<std::size_t>(i);
      out.mode[static_cast<std::size_t>(k)][si] = seq.fsm().mode(i);
      out.evidence[static_cast<std::size_t>(k)][si] = seq.fsm().evidence(i);
      if (ContactFsm::HoldsAnchor(seq.fsm().mode(i)) != seq.core().contact_active(i)) {
        ++out.contract;
      }
    }
  }
  return out;
}

struct Metrics {
  long tp{0};
  long fp{0};
  long fn{0};
  int transitions{0};  // after the first stance, all slots
  int stance_exits{0};
  int slip_in{0};
  int unknown_in{0};
  int false_exit{0};
  int gt_transitions{0};
  std::vector<double> touchdown;
  std::vector<double> liftoff;
  double longest_no_contact{0.0};

  [[nodiscard]] double precision() const {
    return tp + fp > 0 ? static_cast<double>(tp) / static_cast<double>(tp + fp) : std::nan("");
  }

  [[nodiscard]] double recall() const {
    return tp + fn > 0 ? static_cast<double>(tp) / static_cast<double>(tp + fn) : std::nan("");
  }
};

double Max(const std::vector<double>& v) {
  return v.empty() ? std::nan("") : *std::max_element(v.begin(), v.end());
}

Metrics Measure(const std::vector<Row>& rows, const std::vector<std::array<bool, kSlots>>& gt,
                const Replay& rp, int shift) {
  Metrics m;
  const int n = static_cast<int>(rows.size());
  const int guard = kGuardTicks + std::abs(shift);
  auto stance = [&](int k, int i) {
    return rp.mode[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] ==
           ContactMode::kStance;
  };
  auto truth = [&](int k, int i) {
    return gt[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
  };
  auto planned = [&](int k, int i) {
    return rp.planned[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
  };
  for (int i = 0; i < kSlots; ++i) {
    std::vector<int> edges;
    for (int k = kInitTick + 1; k < n; ++k) {
      if (truth(k, i) != truth(k - 1, i)) {
        edges.push_back(k);
      }
    }
    m.gt_transitions += static_cast<int>(edges.size());
    for (std::size_t e = 0; e + 1 < edges.size(); ++e) {
      if (!truth(edges[e], i)) {
        m.longest_no_contact = std::max(m.longest_no_contact, (edges[e + 1] - edges[e]) * kPeriod);
      }
    }
    for (int k = kEvalTick; k < n; ++k) {
      bool near = false;
      for (int e : edges) {
        near = near || std::abs(k - e) <= guard;
      }
      if (!near) {
        m.tp += stance(k, i) && truth(k, i);
        m.fp += stance(k, i) && !truth(k, i);
        m.fn += !stance(k, i) && truth(k, i);
      }
      const ContactMode a = rp.mode[static_cast<std::size_t>(k - 1)][static_cast<std::size_t>(i)];
      const ContactMode b = rp.mode[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)];
      if (a != b) {
        ++m.transitions;
        m.slip_in += b == ContactMode::kSlip;
        m.unknown_in += b == ContactMode::kUnknown;
        if (a == ContactMode::kStance) {
          ++m.stance_exits;
          if (truth(k, i) && planned(k, i) &&
              rows[static_cast<std::size_t>(k)].sole_speed[static_cast<std::size_t>(i)] <
                  kFalseExitSpeed) {
            ++m.false_exit;
          }
        }
      }
    }
    for (int e : edges) {
      if (e < kEvalTick) {
        continue;
      }
      if (truth(e, i)) {  // touchdown: from max(truth, planned) on edge
        int ref = e;
        while (ref < n && !planned(ref, i)) {
          ++ref;
        }
        for (int k = ref; k < n; ++k) {
          if (stance(k, i)) {
            m.touchdown.push_back((k - ref) * kPeriod);
            break;
          }
        }
      } else {  // liftoff: from min(truth, planned) off edge
        int ref = e;
        while (ref - 1 > kInitTick && truth(ref - 1, i) && !planned(ref - 1, i)) {
          --ref;
        }
        for (int k = ref; k < n; ++k) {
          if (!stance(k, i)) {
            m.liftoff.push_back((k - ref) * kPeriod);
            break;
          }
        }
      }
    }
  }
  return m;
}

void Print(const char* name, int shift, const Metrics& m) {
  std::printf(
      "[metric] %-12s shift %+3d ms | precision %.5f recall %.5f | GT transitions %d, longest "
      "no-contact %.3f s | touchdown n=%zu max %.0f ms | liftoff n=%zu max %.0f ms | "
      "stance exits %d false exits %d slip in %d unknown in %d\n",
      name, 2 * shift, m.precision(), m.recall(), m.gt_transitions, m.longest_no_contact,
      m.touchdown.size(), 1e3 * Max(m.touchdown), m.liftoff.size(), 1e3 * Max(m.liftoff),
      m.stance_exits, m.false_exit, m.slip_in, m.unknown_in);
}

#define RECORD_OR_FAIL(spec, rows)             \
  std::vector<Row> rows;                       \
  {                                            \
    Recorder rec(spec);                        \
    std::string why;                           \
    ASSERT_TRUE(rec.Record(rows, why)) << why; \
  }

TEST(ContactFsmSim, QuietStance) {
  SimSpec spec;
  spec.seconds = 10.0;
  RECORD_OR_FAIL(spec, rows);
  const auto gt = Truth(rows);
  const Replay rp = Estimate(rows, gt, 0, true);
  EXPECT_EQ(rp.bad_status, 0);
  EXPECT_EQ(rp.contract, 0);
  const Metrics m = Measure(rows, gt, rp, 0);
  Print("S1 quiet", 0, m);
  ASSERT_EQ(m.gt_transitions, 0) << "fixture: the stance must not lift";
  EXPECT_EQ(m.transitions, 0);
  EXPECT_EQ(m.precision(), 1.0);
  EXPECT_EQ(m.recall(), 1.0);
}

TEST(ContactFsmSim, PushesWithoutLiftHoldStance) {
  const std::array<Push, 4> pushes{
      {{3.0, 1, 100.0}, {3.0, 1, -100.0}, {3.0, 0, 60.0}, {3.0, 0, -60.0}}};
  for (const Push& p : pushes) {
    SimSpec spec;
    spec.seconds = 6.0;
    spec.push = true;
    spec.p = p;
    RECORD_OR_FAIL(spec, rows);
    const auto gt = Truth(rows);
    const Replay rp = Estimate(rows, gt, 0, true);
    EXPECT_EQ(rp.bad_status, 0);
    EXPECT_EQ(rp.contract, 0);
    const Metrics m = Measure(rows, gt, rp, 0);
    Print(p.axis == 1 ? "S2 push y" : "S2 push x", 0, m);
    ASSERT_EQ(m.gt_transitions, 0) << "fixture: this push must not lift a foot";
    EXPECT_EQ(m.stance_exits, 0) << p.axis << " " << p.force;
    EXPECT_EQ(m.slip_in, 0);
    EXPECT_EQ(m.unknown_in, 0);
  }
}

TEST(ContactFsmSim, CraneTouchdownAndLiftoff) {
  const std::array<Crane, 2> cranes{{{0.1, 0.012}, {0.2, 0.025}}};
  for (const Crane& c : cranes) {
    SimSpec spec;
    spec.seconds = 20.0;
    spec.crane = true;
    spec.c = c;
    RECORD_OR_FAIL(spec, rows);
    const auto gt = Truth(rows);
    for (const int shift : {-20, 0, 20}) {
      const Replay rp = Estimate(rows, gt, shift, false);
      EXPECT_EQ(rp.bad_status, 0);
      EXPECT_EQ(rp.contract, 0);
      const Metrics m = Measure(rows, gt, rp, shift);
      Print(c.speed < 0.15 ? "S3 crane 0.1" : "S3 crane 0.2", shift, m);
      ASSERT_GE(m.gt_transitions, 8) << "fixture: the crane must lift the soles";
      ASSERT_LE(m.longest_no_contact, 0.4 + kEps) << "fixture: no-contact span";
      EXPECT_GE(m.precision(), 0.999);
      EXPECT_GE(m.recall(), 0.99);
      if (c.speed > SimFsmConfig().slip_speed && shift < 0) {
        std::printf("[metric] (report) false exits above: plan early, descent above v_slip\n");
      } else {
        EXPECT_EQ(m.false_exit, 0);
      }
      ASSERT_FALSE(m.touchdown.empty());
      ASSERT_FALSE(m.liftoff.empty());
      EXPECT_LE(Max(m.touchdown), 0.150 + kEps);
      EXPECT_LE(Max(m.liftoff), (shift <= 0 ? 0.004 : 0.044) + kEps);
    }
  }
}

TEST(ContactFsmSim, CraneRecordingReplaysBitForBit) {
  SimSpec spec;
  spec.seconds = 8.0;
  spec.crane = true;
  RECORD_OR_FAIL(spec, a);
  RECORD_OR_FAIL(spec, b);
  const Replay ra = Estimate(a, Truth(a), 0, false);
  const Replay rb = Estimate(b, Truth(b), 0, false);
  ASSERT_EQ(ra.mode.size(), rb.mode.size());
  int transitions = 0;
  for (std::size_t k = 0; k < ra.mode.size(); ++k) {
    ASSERT_EQ(ra.mode[k], rb.mode[k]) << k;
    ASSERT_EQ(0, std::memcmp(ra.evidence[k].data(), rb.evidence[k].data(), sizeof(double) * kSlots))
        << k;
    transitions += k > 0 && ra.mode[k] != ra.mode[k - 1];
  }
  EXPECT_GT(transitions, 4);
}

TEST(ContactFsmSim, LowFrictionSlipReportOnly) {
  SimSpec spec;
  spec.seconds = 6.0;
  spec.friction = 0.3;
  spec.push = true;
  spec.p = {3.0, 1, 100.0};
  RECORD_OR_FAIL(spec, rows);
  const auto gt = Truth(rows);
  const Replay rp = Estimate(rows, gt, 0, true);
  EXPECT_EQ(rp.bad_status, 0);
  EXPECT_EQ(rp.contract, 0);
  // Truth slip span: found and sole speed > 0.05 m/s for ≥ 20 ms.
  int spans = 0;
  int detected = 0;
  for (int i = 0; i < kSlots; ++i) {
    int run = 0;
    int start = 0;
    for (int k = kEvalTick; k < static_cast<int>(rows.size()); ++k) {
      const auto& r = rows[static_cast<std::size_t>(k)];
      const bool sl = gt[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] &&
                      r.sole_speed[static_cast<std::size_t>(i)] > 0.05;
      if (sl) {
        start = run == 0 ? k : start;
        ++run;
      }
      if ((!sl || k + 1 == static_cast<int>(rows.size())) && run > 0) {
        if (run >= kDebounceTicks) {
          ++spans;
          for (int j = start; j < k; ++j) {
            if (rp.mode[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] !=
                ContactMode::kStance) {
              ++detected;
              break;
            }
          }
        }
        run = 0;
      }
    }
  }
  const Metrics m = Measure(rows, gt, rp, 0);
  Print("S4 friction", 0, m);
  std::printf("[metric] S4 friction: truth slip spans %d, detected %d\n", spans, detected);
}

}  // namespace
}  // namespace rtc
