// Shared pieces of the catching closed-loop suites (S5.3 / S5.5).
//
// Extracted from test_catching_tracking.cpp when the CLIK sweep
// (test_catching_clik_sweep.cpp) needed the same model, the same independent
// oracle and the same profile with different reference gains. The profile is a
// FUNCTION of the gains rather than a second copy of the YAML: a sweep that
// pasted its own would drift from the suite it is supposed to extend, and the
// one thing both must agree on is what "the shipped law" means.

#pragma once

#include "catching_cloud_fixture.hpp"
#include "rtc_base/types/types.hpp"  // rtc::SteadyNowNs
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace integrated_bringup::testfx {

constexpr double kDt = 0.002;  // 500 Hz, the shipped control_rate
/// kDt on the steady axis: one tick of FakeSteadyClock.
constexpr std::int64_t kDtNs = 2'000'000;
static_assert(static_cast<double>(kDtNs) == kDt * 1e9, "kDtNs must be kDt in ns");

/// The clock a closed-loop suite hands DemoCatchingController::
/// SetClockForTesting: advanced exactly one control period per tick instead
/// of slept through, so the controller's clock and `state.dt` agree by
/// construction. A function pointer carries no state, hence one instant per
/// process; Restart() (per case) puts it on the real steady axis so a stamp
/// the test takes from rtc::SteadyNowNs stays comparable. The planner thread
/// does not read it — a suite that injects it keeps the planner disabled.
class FakeSteadyClock {
 public:
  static std::int64_t Now() noexcept { return now_ns_.load(std::memory_order_relaxed); }

  static void Step() noexcept { now_ns_.fetch_add(kDtNs, std::memory_order_relaxed); }

  static void Restart() noexcept { now_ns_.store(rtc::SteadyNowNs(), std::memory_order_relaxed); }

 private:
  static inline std::atomic<std::int64_t> now_ns_{0};
};

constexpr const char* kCatchFrame = "catch_frame";
/// The shipped offset (config/ur5e_p1b/_base.yaml urdf.extra_frames).
const Eigen::Vector3d kCatchXyz{0.015, 0.145, 0.052};

/// The fixture's model config plus the catch frame, which the shared fixture
/// does not carry (it predates D-10). Built here rather than added to the
/// shared fixture so the other suites keep the model they were written for.
rtc_urdf_bridge::ModelConfig MakeConfigWithCatchFrame() {
  rtc_urdf_bridge::ModelConfig cfg = integrated_bringup::testfx::MakeUr5eP1bModelConfig();
  rtc_urdf_bridge::ExtraFrameConfig frame;
  frame.name = kCatchFrame;
  frame.parent = "l_palm_link";
  frame.xyz = kCatchXyz;
  frame.provisional = false;
  cfg.extra_frames.push_back(frame);
  return cfg;
}

/// The builder of MakeConfigWithCatchFrame(), built once per process. The
/// xacro → Pinocchio parse (~90 ms) is nearly all of a case's bring-up, and a
/// builder has no mutators once built — production shares one across every
/// controller the same way (RTControllerInterface::SetSharedModelBuilder).
std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> SharedCatchFrameBuilder() {
  static const auto builder =
      std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(MakeConfigWithCatchFrame());
  return builder;
}

/// The INDEPENDENT oracle: a second model, driven at the joints the controller
/// commanded, answering "where is the catch frame really".
class CatchFrameOracle {
 public:
  explicit CatchFrameOracle(rtc_urdf_bridge::PinocchioModelBuilder& builder) {
    // The ACTUATED model, which is what CombinedModelCache selects for a
    // closed-chain hand — and therefore what the controller's commands are
    // expressed against.
    //
    // Independence here means "not reading the controller's own cache", not
    // "a different model": a reduced model places an inherited frame at the
    // world pose it had in the REFERENCE configuration the reduction was taken
    // at, so the full model and the actuated one disagree about the catch
    // frame by a few millimetres at the same arm angles. Judging the
    // controller against a model it is not driving would measure that
    // disagreement and call it a tracking error (measured: 2.7 mm).
    model_ = builder.GetActuatedModel();
    if (!model_) {
      model_ = builder.GetFullModel();
    }
    data_ = std::make_unique<pinocchio::Data>(*model_);
    frame_id_ = rtc::catching::ResolveCatchFrame(*model_, kCatchFrame);
    q_ = Eigen::VectorXd::Zero(model_->nq);
  }

  /// `arm` is in device (joint_state_names) order; the hand stays at zero,
  /// which is where this test's measured hand sits.
  /// The catch frame's linear velocity (world axes) at arm `q` moving at
  /// `qd`, both device order — J(q)·q̇ by velocity forward kinematics.
  Eigen::Vector3d LinearVelocityAt(const std::vector<std::string>& arm_names,
                                   const std::array<double, 64>& q,
                                   const std::array<double, 64>& qd, int arm_dof) {
    q_.setZero();
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->nv);
    for (int i = 0; i < arm_dof; ++i) {
      const auto jid = model_->getJointId(arm_names[static_cast<std::size_t>(i)]);
      q_[model_->joints[jid].idx_q()] = q[static_cast<std::size_t>(i)];
      v[model_->joints[jid].idx_v()] = qd[static_cast<std::size_t>(i)];
    }
    pinocchio::forwardKinematics(*model_, *data_, q_, v);
    return pinocchio::getFrameVelocity(*model_, *data_, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED)
        .linear();
  }

  pinocchio::SE3 PoseAt(const std::vector<std::string>& arm_names,
                        const std::array<double, 64>& arm_values, int arm_dof) {
    q_.setZero();
    for (int i = 0; i < arm_dof; ++i) {
      const auto jid = model_->getJointId(arm_names[static_cast<std::size_t>(i)]);
      const auto idx = model_->joints[jid].idx_q();
      q_[idx] = arm_values[static_cast<std::size_t>(i)];
    }
    pinocchio::forwardKinematics(*model_, *data_, q_);
    pinocchio::updateFramePlacement(*model_, *data_, frame_id_);
    return data_->oMf[frame_id_];
  }

 private:
  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex frame_id_{0};
  Eigen::VectorXd q_;
};

/// What a copy of the shipped D-16 box says about its own `provisional` flag.
enum class AccelBoxFlag { kCleared, kProvisional, kAbsent };

/// The SHIPPED ur5e_p1b derived acceleration box with only its `provisional`
/// flag rewritten, as an absolute path `robot.arm.accel_limits_path` can name.
///
/// The shipped file stays `provisional: true` until the real arm's box is
/// identified (#537 Q4), and this fixture is judged on the real-arm axis —
/// where that flag parks the controller. A copy rather than a second file in
/// the package: the values are read from the installed file on every run, so
/// they cannot drift from it, and no "cleared" box ships next to the real one.
/// One file per process and flag, removed when the process exits.
std::string AccelLimitsCopy(AccelBoxFlag flag = AccelBoxFlag::kCleared) {
  struct Copy {
    std::filesystem::path path;
    Copy(const Copy&) = delete;
    Copy& operator=(const Copy&) = delete;

    explicit Copy(AccelBoxFlag f) {
      const std::string shipped =
          ament_index_cpp::get_package_share_directory("integrated_bringup") +
          "/config/ur5e_p1b/derived_accel_limits.yaml";
      YAML::Node doc = YAML::LoadFile(shipped);
      YAML::Node group = doc["derived_accel_limits"]["ur5e"];
      if (!group || !group.IsMap()) {
        throw std::runtime_error(shipped + " has no derived_accel_limits.ur5e");
      }
      const char* tag = "cleared";
      switch (f) {
        case AccelBoxFlag::kCleared:
          group["provisional"] = false;
          break;
        case AccelBoxFlag::kProvisional:
          group["provisional"] = true;
          tag = "provisional";
          break;
        case AccelBoxFlag::kAbsent:
          group.remove("provisional");
          tag = "absent";
          break;
      }
      path = std::filesystem::temp_directory_path() /
             ("rtc_catching_accel_box_" + std::to_string(::getpid()) + "_" + tag + ".yaml");
      std::ofstream out(path);
      out << doc << '\n';
      if (!out) {
        throw std::runtime_error("could not write " + path.string());
      }
    }

    ~Copy() {
      std::error_code ec;
      std::filesystem::remove(path, ec);
    }
  };

  switch (flag) {
    case AccelBoxFlag::kProvisional: {
      static const Copy copy{AccelBoxFlag::kProvisional};
      return copy.path.string();
    }
    case AccelBoxFlag::kAbsent: {
      static const Copy copy{AccelBoxFlag::kAbsent};
      return copy.path.string();
    }
    case AccelBoxFlag::kCleared:
      break;
  }
  static const Copy copy{AccelBoxFlag::kCleared};
  return copy.path.string();
}

/// The reference generator's gains. Defaulted to what the S5.3 suite was
/// written against; the sweep passes the SHIPPED profile's values instead,
/// because "how well does the law track" is a question about the law that is
/// actually deployed, not about a faster one chosen to make a test converge.
struct ReferenceGains {
  double omega{20.0};
  double v_max{3.0};
  double a_max{30.0};
  double track_err_abort{0.5};
};

std::string TrackingYaml(const std::string& topic, const Eigen::Vector3d& p_c,
                         const Eigen::Vector3d& a_d, double gamma_f, double t_c_offset_s,
                         const ReferenceGains& gains = {}) {
  std::ostringstream os;
  os.precision(12);
  os << R"(
command_type: "position"
diagnostic:
  hand_step: false
  oracle_plan:
    enabled: true
    p_c: [)"
     << p_c.x() << ", " << p_c.y() << ", " << p_c.z() << R"(]
    a_d: [)"
     << a_d.x() << ", " << a_d.y() << ", " << a_d.z() << R"(]
    t_c_offset_s: )"
     << t_c_offset_s << R"(
    gamma_f: )"
     << gamma_f << R"(
catching:
  catch_frame: ")"
     << kCatchFrame << R"("
  io:
    traj_topic: ")"
     << topic << R"("
    expected_frame: "world"
    n_min: 7
    t_stale: 0.2
    future_tol: 0.01
    horizon_min: 0.3
    track:
      eval_offset: 0.05
  prediction:
    dt_expected: 0.05
  sim:
    io:
      future_tol: 0.2
  reference:
    # Cleared, like every other provisional flag in this fixture: the profile
    # below fixes all four values, and this suite is judged on the REAL-ARM
    # axis (its device configs declare no backend), where a provisional block
    # parks the controller before it can command anything.
    provisional: false
    omega: )"
     << gains.omega << R"(
    zeta: 1.0
    v_max: )"
     << gains.v_max << R"(
    a_max: )"
     << gains.a_max << R"(
  joint_cmd:
    K_p: 20.0
    K_a: 8.0
    K_n: 1.0
    w_task: 1.0
    w_a: 0.5
    w_arm: 0.01
    w_smooth: 0.001
    damping_sq: 0.0001
    qp:
      max_iter: 30
    lag:
      # Cleared like the other provisional flags here (real-arm axis, #537 Q5).
      provisional: false
      T_arm: 0.0
  supervisor:
    track_err_abort: )"
     << gains.track_err_abort << R"(
    n_qp: 3
    # Cleared like the other provisional flags here (real-arm axis); the two
    # deadlines themselves are left to the parser defaults (#537 S9b).
    deadline:
      provisional: false
    decel:
      a_dec: 10.0
  robot:
    arm:
      limit_margin: 0.05
      accel_limits_package: "integrated_bringup"
      # The shipped box with its provisional flag cleared (AccelLimitsCopy).
      accel_limits_path: ")"
     << AccelLimitsCopy() << R"("
      accel_limits_group: "ur5e"
    hand:
      provisional: false
      rho_eps: 0.02
      q_open:  [0,0,0,0,0,0,0,0,0,0]
      q_pre:   [0,0,0,0,0,0,0,0,0,0]
      q_close: [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5]
      caging_mask: [true,true,true,true,true,true,true,true,true,true]
      eta_close: 0.9
      T_close_e2e: 0.28
  core:
    ball:
      diameter: 0.067
      mass: 0.057
      restitution: 0.75
      provisional: false
  planner:
    gamma:
      eta_v: 0.9
    catchability:
      manipulability_min:
        arm_5row: 0.1
        provisional: false
    # The S7 supervisor needs both once the law is wired (a configuration
    # without them is parked). The wait pose is the fixture's home, where
    # every suite measures the arm at start: an aligned arm skips homing
    # (#537 S7 Q13), so the S5/S6 cases still arm on their first tick. The
    # freeze is the shipped p1b value, above T_close_e2e + T_arm + h.
    wait_pose: [)"
     << kUr5eHome[0] << ", " << kUr5eHome[1] << ", " << kUr5eHome[2] << ", " << kUr5eHome[3] << ", "
     << kUr5eHome[4] << ", " << kUr5eHome[5] << R"(]
    freeze: {T_freeze: 0.36}
topics:
  ur5e:
    subscribe:
      - topic: "ur5e/joint_goal"
        role: "target"
  p1b:
    subscribe:
      - topic: "p1b/joint_goal"
        role: "target"
)";
  return os.str();
}

}  // namespace integrated_bringup::testfx
