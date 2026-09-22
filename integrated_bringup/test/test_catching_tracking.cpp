// ── The catching tracking law, closed loop on the real model (S5.3) ─────────
//
// G4-H / G5-A / G5-B. What is under test is the CHAIN — sample → soft-catch
// reference → extended CLIK → arm command — against the shipped `ur5e_p1b`
// model and its real `catch_frame`, not against a stub.
//
// THE ORACLE IS INDEPENDENT. Every assertion about where the hand ended up is
// computed from a SECOND pinocchio model this file builds and drives at the
// controller's commanded joints. Reading the controller's own cache would
// prove that the controller agrees with itself, which is the one thing that
// cannot fail.
//
// THE LOOP IS CLOSED WITH A PERFECT SERVO: the measured state of tick n+1 is
// the command of tick n. That is the sim's own assumption (no actuation lag,
// 2026-09-20), and it is what makes a position-error assertion mean "the law
// converges" rather than "the plant is slow".
//
// REAL TIME IS REAL. The controller reads the steady clock per tick (plan §3
// forbids tick×dt), so the test SLEEPS one control period between ticks. That
// costs a fraction of a second and it is what keeps the reference's own time
// axis and the integration step consistent; a loop that spun as fast as it
// could would advance `dt` per tick while the clock stood still, and every
// number downstream would describe a system that does not exist.

#include "arm_lag_fixture.hpp"
#include "catching_cloud_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::kCatchingArmDeviceIdx;
using integrated_bringup::testfx::kP1bHandDof;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using rtc::ControllerOutput;
using rtc::ControllerState;

using namespace std::chrono_literals;

constexpr double kDt = 0.002;  // 500 Hz, the shipped control_rate
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

std::string TrackingYaml(const std::string& topic, const Eigen::Vector3d& p_c,
                         const Eigen::Vector3d& a_d, double gamma_f, double t_c_offset_s) {
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
    omega: 20.0
    zeta: 1.0
    v_max: 3.0
    a_max: 30.0
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
      T_arm: 0.0
  supervisor:
    track_err_abort: 0.5
    n_qp: 3
    decel:
      a_dec: 10.0
  robot:
    arm:
      limit_margin: 0.05
      accel_limits_package: "integrated_bringup"
      accel_limits_path: "config/ur5e_p1b/derived_accel_limits.yaml"
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

class CatchingTrackingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_tracking_test");
    builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(MakeConfigWithCatchFrame());
    oracle_ = std::make_unique<CatchFrameOracle>(*builder_);
    arm_names_ =
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names;
    topic_ = "/test_catching_tracking/prediction";

    // Where the catch frame starts, and where we will ask it to go.
    std::array<double, 64> home{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      home[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    start_pose_ = oracle_->PoseAt(arm_names_, home, kUr5eArmDof);
  }

  void TearDown() override {
    if (executor_) {
      executor_->remove_node(node_->get_node_base_interface());
    }
    pub_.reset();
    executor_.reset();
    node_.reset();
  }

  /// Bring the controller up with a plan whose catch point is `p_c` and whose
  /// approach axis is `a_d`, then arm it.
  /// `tweak` edits the parsed YAML before configure, so a case can vary ONE
  /// key without restating the profile — and so the thing it varies is visible
  /// at the call site rather than buried in a second copy of the config.
  void BringUp(const Eigen::Vector3d& p_c, const Eigen::Vector3d& a_d, double gamma_f = 0.0,
               double t_c_offset_s = 1.0, const std::function<void(YAML::Node&)>& tweak = nullptr) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
    const rclcpp_lifecycle::State prev;
    // on_configure runs LoadConfig itself (CM's 3-pass contract), so the YAML
    // goes in here rather than through a separate call — passing an empty node
    // would make the base re-parse nothing over the parsed config.
    YAML::Node yaml = YAML::Load(TrackingYaml(topic_, p_c, a_d, gamma_f, t_c_offset_s));
    if (tweak) {
      tweak(yaml);
    }
    ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    rclcpp::QoS qos{rclcpp::KeepLast(1)};
    qos.best_effort();
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    // The measured state starts at the home posture.
    state_ = MakeState(kUr5eHome);
  }

  static ControllerState MakeState(const std::array<double, kUr5eArmDof>& arm) {
    ControllerState state{};
    state.num_devices = 2;
    state.dt = kDt;
    auto& a = state.devices[0];
    a.num_channels = kUr5eArmDof;
    a.valid = true;
    for (int i = 0; i < kUr5eArmDof; ++i) {
      a.positions[static_cast<std::size_t>(i)] = arm[static_cast<std::size_t>(i)];
    }
    auto& h = state.devices[1];
    h.num_channels = kP1bHandDof;
    h.valid = true;
    return state;
  }

  void PublishPrediction(std::uint64_t sequence) {
    integrated_bringup::testing::CloudSpec spec;
    spec.n = 8;
    spec.sequence = sequence;
    spec.p0 = ball_p0_;
    spec.vel = ball_vel_;
    auto msg = integrated_bringup::testing::MakeCloud(spec);
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const std::int64_t wall =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - 5'000'000;
    msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
    pub_->publish(msg);
    for (int i = 0; i < 8; ++i) {
      executor_->spin_some(2ms);
    }
  }

  /// Run `ticks` closed-loop control periods, re-publishing a prediction about
  /// every 30 ms so the lane never goes stale, and sleeping one period each
  /// tick so the controller's clock and its `dt` agree.
  void RunClosedLoop(int ticks, bool publish = true) {
    for (int t = 0; t < ticks; ++t) {
      if (publish && t % 15 == 0) {
        PublishPrediction(static_cast<std::uint64_t>(t / 15) + 1);
      }
      const ControllerOutput out = ctrl_->Compute(state_);
      last_output_ = out;
      if (out.devices[0].num_channels >= kUr5eArmDof) {
        // The perfect servo: what was commanded is what is measured next tick.
        for (int i = 0; i < kUr5eArmDof; ++i) {
          const auto ui = static_cast<std::size_t>(i);
          commanded_[ui] = out.devices[0].commands[ui];
          state_.devices[0].positions[ui] = out.devices[0].commands[ui];
        }
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
    }
  }

  pinocchio::SE3 CatchPose() { return oracle_->PoseAt(arm_names_, commanded_, kUr5eArmDof); }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<CatchFrameOracle> oracle_;
  std::unique_ptr<DemoCatchingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::vector<std::string> arm_names_;
  std::string topic_;
  pinocchio::SE3 start_pose_{pinocchio::SE3::Identity()};
  ControllerState state_{};
  ControllerOutput last_output_{};
  std::array<double, 64> commanded_{};
  /// Where the published prediction puts the ball. Defaults to the decode
  /// suite's diagonal; the lead-compensation case moves it into the arm's
  /// workspace so the reference it produces is reachable.
  std::array<double, 3> ball_p0_{0.0, 0.0, 0.0};
  std::array<double, 3> ball_vel_{1.0, 2.0, 3.0};
};

// ── G5-A: a static target converges ─────────────────────────────────────────

TEST_F(CatchingTrackingTest, ConvergesToAStaticCatchPointAndAxis) {
  // γ_f = 0 makes the catch point the attractor for the whole run (L4 §5.3:
  // with γ ≡ 0 the ball drops out of the dynamics entirely), which is what
  // turns this into the static-target case G5-A asks for. The target is 6 cm
  // from where the hand starts and the axis is tilted 15°, so both task rows
  // have work to do.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.04, 0.03, 0.02);
  const Eigen::Vector3d z0 = start_pose_.rotation().col(2);
  const Eigen::Vector3d tilt_axis = z0.cross(Eigen::Vector3d::UnitZ()).normalized();
  const Eigen::Vector3d a_d = (Eigen::AngleAxisd(15.0 * M_PI / 180.0, tilt_axis) * z0).normalized();

  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, a_d));
  // Four ticks to cross IDLE → ARMED → TRACKING → APPROACH: the supervisor
  // takes ONE edge per tick, and the oracle plan is built on the tick the
  // supervisor is ready for one.
  RunClosedLoop(4);
  ASSERT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach)
      << "the oracle plan never took the controller into APPROACH";

  // 1.4 s. The AXIS task sets the pace: K_a = 8 rad/s is a 125 ms time
  // constant, and the position error is geometrically tied to what is left of
  // the axis error — the catch frame sits 0.145 m from the palm origin, so a
  // residual degree of tilt is about 2.5 mm of position. A shorter run
  // measures how far the law has got, not where it converges (at 0.6 s: 2.7 mm
  // and 0.83 deg, both still falling).
  RunClosedLoop(700);

  const pinocchio::SE3 pose = CatchPose();
  const double position_error = (pose.translation() - p_c).norm();
  const Eigen::Vector3d z = pose.rotation().col(2);
  const double axis_error_deg = std::acos(std::clamp(z.dot(a_d), -1.0, 1.0)) * 180.0 / M_PI;

  EXPECT_LT(position_error, 1e-3) << "position error " << position_error * 1e3 << " mm";
  EXPECT_LT(axis_error_deg, 0.5) << "axis error " << axis_error_deg << " deg";
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach)
      << "the run aborted; last reason = " << static_cast<int>(ctrl_->GetLastReason());
}

// ── G5-B: the boxes hold ────────────────────────────────────────────────────

TEST_F(CatchingTrackingTest, RespectsTheJointAndStepLimitsThroughout) {
  // A target far enough away that the solver wants more than the joints can
  // give — which is when a box is the only thing between the law and a command
  // the drive answers with a protective stop.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.45, -0.35, 0.30);
  const Eigen::Vector3d a_d = start_pose_.rotation().col(2);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, a_d));
  RunClosedLoop(4);  // IDLE → ARMED → TRACKING → APPROACH
  ASSERT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach);

  const auto configs = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
  const auto limits = configs.at("ur5e").joint_limits.value();
  std::array<double, kUr5eArmDof> previous = kUr5eHome;

  for (int t = 0; t < 200; ++t) {
    RunClosedLoop(1);
    if (last_output_.devices[0].num_channels < kUr5eArmDof) {
      continue;
    }
    for (int i = 0; i < kUr5eArmDof; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      const double q = last_output_.devices[0].commands[ui];
      // Position: inside the device box. The margin means the command should
      // not even reach it, but the box itself is the contract.
      EXPECT_GE(q, limits.position_lower[ui] - 1e-9) << "joint " << i << " below its lower limit";
      EXPECT_LE(q, limits.position_upper[ui] + 1e-9) << "joint " << i << " above its upper limit";
      // Velocity: the per-tick step is bounded by the joint's own limit. A
      // solver that ignored the box would show up here as one large step, not
      // as a drift.
      const double step = std::abs(q - previous[ui]) / kDt;
      EXPECT_LE(step, limits.max_velocity[ui] + 1e-6)
          << "joint " << i << " stepped at " << step << " rad/s on tick " << t;
      previous[ui] = q;
    }
  }
}

// ── The command is continuous across the start of tracking ──────────────────

TEST_F(CatchingTrackingTest, TheFirstCommandedTickDoesNotJump) {
  // The hand-off from "holding the activation pose" to "following the law" is
  // where a seeding mistake shows up, and it shows up as a step of whatever
  // the gap happened to be — on a position interface, straight into the drive.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.05, 0.0, 0.0);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, start_pose_.rotation().col(2)));

  RunClosedLoop(1);
  std::array<double, kUr5eArmDof> before{};
  for (int i = 0; i < kUr5eArmDof; ++i) {
    before[static_cast<std::size_t>(i)] =
        last_output_.devices[0].commands[static_cast<std::size_t>(i)];
  }
  RunClosedLoop(3);  // crosses into APPROACH and takes the first law ticks

  // By VALUE: binding a reference into a temporary map's member leaves it
  // dangling the moment the statement ends, and the limits then read as zero —
  // which turns this bound into "must not move at all" and the assertion into
  // one about the test's own lifetime bug.
  const auto configs = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
  const auto limits = configs.at("ur5e").joint_limits.value();
  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double jump = std::abs(last_output_.devices[0].commands[ui] - before[ui]);
    // Three ticks at the joint's own velocity limit is the most any honest
    // command could move in that time.
    EXPECT_LE(jump, 3.0 * limits.max_velocity[ui] * kDt + 1e-9)
        << "joint " << i << " jumped by " << jump << " rad when the law took over";
  }
}

// ── G5-C: the QP's share of the tick ────────────────────────────────────────

TEST_F(CatchingTrackingTest, QpSolveTimeStaysInsideItsBudget) {
  // The budget (plan §7.3, 2026-09-22, provisional): p99 <= 400 us and max
  // <= 1500 us against a 2000 us tick. Quantiles rather than a mean, because
  // what threatens a control loop is the tail — a mean of 200 us with a 3 ms
  // outlier is a missed tick, and the mean cannot see it.
  //
  // THIS MEASURES THIS MACHINE, so the numbers are recorded as properties as
  // well as asserted: a failure on a slower host should read as "measure your
  // host" rather than as a regression in the solver.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.10, 0.05, 0.05);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, start_pose_.rotation().col(2)));
  RunClosedLoop(4);
  ASSERT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach);

  std::vector<double> solve_us;
  solve_us.reserve(400);
  for (int t = 0; t < 400; ++t) {
    RunClosedLoop(1);
    const auto& solve = ctrl_->GetLastSolve();
    if (solve.reached_solve) {
      solve_us.push_back(solve.solve_time_us);
    }
  }
  ASSERT_GT(solve_us.size(), 300U) << "too few solves to say anything about a tail";

  std::sort(solve_us.begin(), solve_us.end());
  const auto at = [&solve_us](double q) {
    const auto idx = static_cast<std::size_t>(q * static_cast<double>(solve_us.size() - 1));
    return solve_us[idx];
  };
  const double p50 = at(0.50);
  const double p99 = at(0.99);
  const double worst = solve_us.back();
  RecordProperty("solve_p50_us", static_cast<int>(p50));
  RecordProperty("solve_p99_us", static_cast<int>(p99));
  RecordProperty("solve_max_us", static_cast<int>(worst));

  EXPECT_LT(p99, 400.0) << "p99 " << p99 << " us (p50 " << p50 << ")";
  EXPECT_LT(worst, 1500.0) << "max " << worst << " us";
}

// ── G7-H (d): the fault latch outlives an E-STOP clear ──────────────────────

TEST_F(CatchingTrackingTest, AQpFailureStreakLatchesAFaultThatClearEstopDoesNotRelease) {
  // The half of P-1 (d) S5.1 could not reach: it needed a fault SOURCE, and
  // the QP failure streak is the one the design names (L7 §4.2).
  //
  // The streak is provoked with a plan whose approach axis is not a unit
  // vector, which CLIK refuses before the solve. That is a real failure mode —
  // a plan is data, and a planner that emitted a degenerate axis would look
  // exactly like this — rather than an injected error with no counterpart.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.05, 0.0, 0.0);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, Eigen::Vector3d::UnitZ(), 0.0, 1.0, [](YAML::Node& yaml) {
    yaml["diagnostic"]["oracle_plan"]["a_d"] = YAML::Load("[0.0, 0.0, 0.0]");
  }));
  RunClosedLoop(4);
  ASSERT_TRUE(ctrl_->IsPlanActive()) << "the oracle plan was never built";

  // Each failure costs a cycle — APPROACH, ABORT_SAFE, RETREAT, ARMED,
  // TRACKING, a fresh plan — so three of them take a few dozen ticks.
  RunClosedLoop(60);
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kFault);

  // P-1 (d): the two paths are separate. A global clear must not launder a
  // controller fault.
  ctrl_->TriggerEstop();
  RunClosedLoop(1);
  ctrl_->ClearEstop();
  RunClosedLoop(2);
  EXPECT_TRUE(ctrl_->HasLatchedFault()) << "ClearEstop released the controller fault";
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kFault);

  // And the reset does release it, on the tick that services it — leaving the
  // controller DISARMED, because a fault reset is not an arming (P-1 (c)).
  ctrl_->ResetFault();
  RunClosedLoop(2);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kIdle);
  EXPECT_FALSE(ctrl_->IsArmRequested());
}

// ── G5-E: the lead compensation, measured on an injected delay ──────────────

TEST_F(CatchingTrackingTest, LeadCompensationReducesTheErrorUnderAnActuationDelay) {
  // What this gate was stuck on: with no actuation lag in the sim (decided
  // 2026-09-20), leading and not leading produce identical commands and the
  // "before/after" comparison measures nothing. The delay comes from a FIXTURE
  // that only this test can reach (plan §7.3, option ㄱ) — the runtime keeps
  // `T_arm: 0.0`, which is the truth in sim.
  //
  // The comparison is run TWICE against the same plant and the same ball, with
  // the compensation off and on. Asserting that the compensation HELPS, rather
  // than asserting an absolute error, is what keeps this a measurement of the
  // compensation instead of a measurement of this machine.
  constexpr int kDelayTicks = 25;  // 50 ms
  const double t_arm_s = kDelayTicks * kDt;

  // The ball flies through the workspace and is closest to the catch point at
  // the catch instant, so a reference that follows it late is a reference that
  // is in the wrong place — which is the error being measured.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.05, 0.02, 0.0);
  const Eigen::Vector3d a_d = start_pose_.rotation().col(2);

  const auto run = [&](bool lead_enabled) {
    ball_p0_ = {p_c.x() - 0.30, p_c.y(), p_c.z()};
    ball_vel_ = {0.75, 0.0, 0.0};  // 0.4 s of flight covers the 0.30 m
    BringUp(p_c, a_d, /*gamma_f=*/0.4, /*t_c_offset_s=*/0.4,
            [lead_enabled, t_arm_s](YAML::Node& yaml) {
              yaml["catching"]["joint_cmd"]["lag"]["T_arm"] = t_arm_s;
              yaml["catching"]["joint_cmd"]["lag"]["lead_enable"] = lead_enabled;
            });

    std::array<double, kUr5eArmDof> initial{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      initial[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    integrated_bringup::testing::ArmLagPlant<kUr5eArmDof> plant(kDelayTicks, initial);

    // Closed loop THROUGH the delay: what the controller measures this tick is
    // what it commanded `kDelayTicks` ticks ago.
    for (int t = 0; t < 260; ++t) {
      if (t % 15 == 0) {
        PublishPrediction(static_cast<std::uint64_t>(t / 15) + 1);
      }
      const ControllerOutput out = ctrl_->Compute(state_);
      std::array<double, kUr5eArmDof> cmd{};
      if (out.devices[0].num_channels >= kUr5eArmDof) {
        for (int i = 0; i < kUr5eArmDof; ++i) {
          cmd[static_cast<std::size_t>(i)] = out.devices[0].commands[static_cast<std::size_t>(i)];
        }
      } else {
        cmd = plant.Measured();
      }
      const auto& measured = plant.Step(cmd);
      for (int i = 0; i < kUr5eArmDof; ++i) {
        const auto ui = static_cast<std::size_t>(i);
        commanded_[ui] = cmd[ui];
        state_.devices[0].positions[ui] = measured[ui];
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
    }
    // Judged on the MEASURED configuration: the delay is the whole point, so
    // scoring the command would score a pose the arm has not reached.
    std::array<double, 64> measured_arm{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      measured_arm[static_cast<std::size_t>(i)] =
          state_.devices[0].positions[static_cast<std::size_t>(i)];
    }
    return (oracle_->PoseAt(arm_names_, measured_arm, kUr5eArmDof).translation() - p_c).norm();
  };

  const double without_lead = run(false);
  const double with_lead = run(true);
  RecordProperty("lead_off_error_mm", static_cast<int>(without_lead * 1e3));
  RecordProperty("lead_on_error_mm", static_cast<int>(with_lead * 1e3));

  EXPECT_LT(with_lead, without_lead)
      << "lead off " << without_lead * 1e3 << " mm, lead on " << with_lead * 1e3 << " mm";
}

// ── The abort ends, even after the lane goes quiet ──────────────────────────

TEST_F(CatchingTrackingTest, AnAbortCompletesAndReArmsWithNoVisionLeft) {
  // The state after every real abort: the ball has landed, vision stops
  // publishing, and the snapshot goes stale within `io.t_stale`. If the
  // supervisor asked vision about it, the stale answer would decide for
  // ABORT_SAFE — which holds, never drops the plan and never re-arms. The arm
  // is stopped and nothing says so.
  //
  // `n_qp` is raised so the failure aborts WITHOUT latching a fault: what is
  // under test is the ordinary abort path, not the escalation.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.05, 0.0, 0.0);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, Eigen::Vector3d::UnitZ(), 0.0, 1.0, [](YAML::Node& yaml) {
    yaml["diagnostic"]["oracle_plan"]["a_d"] = YAML::Load("[0.0, 0.0, 0.0]");
    yaml["catching"]["supervisor"]["n_qp"] = 1000;
  }));
  RunClosedLoop(4);
  ASSERT_TRUE(ctrl_->IsPlanActive());
  // Watch for the abort rather than asserting on one tick: with the arm
  // already at rest the stop completes immediately, so ABORT_SAFE can come and
  // go inside a couple of ticks. What this test needs to know is that it
  // HAPPENED — otherwise the assertions below would pass on a controller that
  // never aborted at all.
  bool saw_abort = false;
  for (int t = 0; t < 8 && !saw_abort; ++t) {
    RunClosedLoop(1);
    saw_abort = ctrl_->GetMode() == rtc::catching::Mode::kAbortSafe;
  }
  ASSERT_TRUE(saw_abort) << "precondition: the degenerate plan never aborted";

  // No more predictions: the lane goes stale (t_stale is 0.2 s here).
  RunClosedLoop(150, /*publish=*/false);
  EXPECT_NE(ctrl_->GetMode(), rtc::catching::Mode::kAbortSafe)
      << "the abort never completed with a stale vision lane";
  EXPECT_FALSE(ctrl_->IsPlanActive()) << "the aborted plan was never dropped";
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
