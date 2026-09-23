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
#include "catching_tracking_fixture.hpp"
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
#include <pinocchio/algorithm/rnea.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
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
// Moved to catching_tracking_fixture.hpp so the CLIK sweep shares ONE profile.
using integrated_bringup::testfx::CatchFrameOracle;
using integrated_bringup::testfx::kCatchFrame;
using integrated_bringup::testfx::kCatchXyz;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
using integrated_bringup::testfx::TrackingYaml;

using namespace std::chrono_literals;

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
  ///
  /// `t_c_offset_s` defaults to 5 s. These cases are about the APPROACH law,
  /// and from S7 the catch instant ENDS the approach — COMMITTED at t_c −
  /// T_freeze, DECEL at t_c — so the default puts it beyond every run below.
  /// (It was 1 s while nothing happened at t_c.)
  void BringUp(const Eigen::Vector3d& p_c, const Eigen::Vector3d& a_d, double gamma_f = 0.0,
               double t_c_offset_s = 5.0, const std::function<void(YAML::Node&)>& tweak = nullptr,
               const std::function<void(std::map<std::string, rtc::DeviceNameConfig>&)>&
                   device_tweak = nullptr) {
    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    // `device_tweak` edits the DEVICE limits, which the catching YAML cannot
    // reach: the position box comes from the device, and a case about that box
    // has to be able to put it somewhere the arm actually goes.
    auto devices = integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs();
    if (device_tweak) {
      device_tweak(devices);
    }
    ctrl_->SetDeviceNameConfigs(std::move(devices));
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

  void PublishPrediction(std::uint64_t sequence, std::uint64_t generation = 42) {
    integrated_bringup::testing::CloudSpec spec;
    spec.n = 8;
    spec.sequence = sequence;
    spec.generation = generation;
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
      state_.iteration += 1;
      state_.t_relative_s = static_cast<double>(state_.iteration) * kDt;
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

// ── Decision K (S6-C2): the dynamic form bounds the arm torque ──────────────

class CatchingTorqueTest : public CatchingTrackingTest {
 protected:
  static constexpr double kEta = 0.8;  // joint_cmd.eta_tau default (D-16)

  /// Worst |τ_i| / (η·τ_max,i) over the arm across 300 ticks toward the G5-B
  /// far target. The oracle is RNEA on the ACTUATED model (the one the
  /// controller drives, see CatchFrameOracle), fed the commanded arm path:
  /// τ = RNEA(q_{n−1}, v_{n−1}, (v_n − v_{n−1})/dt), v_n = (q_n − q_{n−1})/dt —
  /// what the perfect servo would have to deliver. The hand sits at zero.
  double WorstTorqueRatio(const std::function<void(YAML::Node&)>& tweak, int& clik_failures) {
    const auto model =
        builder_->GetActuatedModel() ? builder_->GetActuatedModel() : builder_->GetFullModel();
    EXPECT_TRUE(model);
    pinocchio::Data data(*model);
    const auto limits =
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_limits.value();
    std::array<Eigen::Index, kUr5eArmDof> vidx{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      const auto jid = model->getJointId(arm_names_[static_cast<std::size_t>(i)]);
      vidx[static_cast<std::size_t>(i)] = model->joints[jid].idx_v();
    }
    const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.45, -0.35, 0.30);
    const Eigen::Vector3d a_d = start_pose_.rotation().col(2);
    // t_c 5 s out, like BringUp's default: 300 ticks of APPROACH (see there).
    // And no REF_SATURATED abort: the far target saturates the reference for
    // most of the run BY DESIGN — that is the load this case measures — and
    // S7's streak rule (L7 §4.2) would end the approach it is measuring.
    BringUp(p_c, a_d, 0.0, 5.0, [&tweak](YAML::Node& y) {
      y["catching"]["supervisor"]["sat_ticks"] = 1000000;
      if (tweak) {
        tweak(y);
      }
    });
    if (::testing::Test::HasFatalFailure() || !model) {
      return 0.0;  // BringUp's ASSERT returned from BringUp only
    }
    RunClosedLoop(4);  // IDLE → ARMED → TRACKING → APPROACH
    EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach);

    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model->nq);
    Eigen::VectorXd q1 = q0;
    Eigen::VectorXd q2 = q0;
    const auto load = [&](Eigen::VectorXd& q) {
      for (int i = 0; i < kUr5eArmDof; ++i) {
        q[vidx[static_cast<std::size_t>(i)]] = commanded_[static_cast<std::size_t>(i)];
      }
    };
    // One tick at a time with our OWN publishing: RunClosedLoop(1) would
    // republish sequence 1 every tick, which the lane de-duplicates, and the
    // ball would go stale 0.2 s in (APPROACH → RETREAT holds the arm, a stop
    // no torque bound describes).
    std::uint64_t sequence = 2;  // RunClosedLoop(4) above used 1
    const auto tick = [&](int t) {
      if (t % 15 == 0) {
        PublishPrediction(sequence++);
      }
      RunClosedLoop(1, /*publish=*/false);
    };
    load(q1);
    tick(0);
    load(q2);
    double worst = 0.0;
    clik_failures = 0;
    for (int t = 1; t <= 300; ++t) {
      q0 = q1;
      q1 = q2;
      tick(t);
      load(q2);
      EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach) << "tick " << t;
      clik_failures += ctrl_->GetLastSolve().converged ? 0 : 1;
      const Eigen::VectorXd v_prev = (q1 - q0) / kDt;
      const Eigen::VectorXd v = (q2 - q1) / kDt;
      const Eigen::VectorXd a = (v - v_prev) / kDt;
      const Eigen::VectorXd tau = pinocchio::rnea(*model, data, q1, v_prev, a);
      for (int i = 0; i < kUr5eArmDof; ++i) {
        const auto ui = static_cast<std::size_t>(i);
        worst = std::max(worst, std::abs(tau[vidx[ui]]) / (kEta * limits.max_torque[ui]));
      }
    }
    return worst;
  }
};

TEST_F(CatchingTorqueTest, WithoutAnAccelerationConstraintTheFarTargetBreaksTheBound) {
  // The premise of the next case: with no BINDING acceleration constraint this
  // run demands more than η·τ_max, so a green dynamic run is the rows' doing.
  //
  // "No binding constraint" is the kinematic form with bounds no run can
  // reach, not a missing derived box: from S7 the supervisor's joint-space
  // stop and homing ramp with that box, so a profile without it cannot run a
  // trial and is parked (SupervisorValueMissing) — and the kinematic form's
  // task rows replace the per-joint box in the solve, which is the absence
  // this premise needs.
  int failures = 0;
  const double worst = WorstTorqueRatio(
      [](YAML::Node& y) {
        y["catching"]["joint_cmd"]["accel_constraint"] = "kinematic";
        // The validator's ceilings, far above what this run asks for.
        y["catching"]["joint_cmd"]["task_accel_max_linear"] = 500.0;
        y["catching"]["joint_cmd"]["task_accel_max_angular"] = 500.0;
      },
      failures);
  // Measured 1.10 (deterministic: the servo is perfect and the clock only
  // moves the reference's time axis). The case needs "breaks it", not "by how
  // much"; 1.05 keeps it clear of the dynamic case's ≤ 1.001.
  EXPECT_GT(worst, 1.05) << "worst " << worst;
}

TEST_F(CatchingTorqueTest, TheDynamicFormKeepsTheArmTorqueInsideItsBound) {
  int failures = 0;
  const double worst = WorstTorqueRatio(
      [](YAML::Node& y) { y["catching"]["joint_cmd"]["accel_constraint"] = "dynamic"; }, failures);
  EXPECT_EQ(failures, 0) << "the torque rows made the QP fail";
  EXPECT_LE(worst, 1.0 + 1e-3) << "the arm torque left its bound";
  EXPECT_GT(worst, 0.8) << "the bound should be active in this run";
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach)
      << "the run aborted; last reason = " << static_cast<int>(ctrl_->GetLastReason());
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
  // Each failure costs a cycle — APPROACH, ABORT_SAFE, RETREAT, ARMED,
  // TRACKING, a fresh plan. From S7 each cycle also needs a NEW BALL: after a
  // re-arm the supervisor refuses the track the last attempt was on (#537 S7
  // Q15, R-TRACK), so the same prediction would park it in ARMED and the
  // streak would never complete. The fixture throws a new ball (a new track
  // generation) for every attempt, which is what a thrower retrying does.
  bool planned = false;
  std::uint64_t generation = 42;
  std::uint64_t sequence = 1;
  bool was_aborting = false;
  for (int t = 0; t < 400 && !ctrl_->HasLatchedFault(); ++t) {
    if (t % 15 == 0) {
      PublishPrediction(sequence++, generation);
    }
    RunClosedLoop(1, /*publish=*/false);
    planned = planned || ctrl_->IsPlanActive();
    const bool aborting = ctrl_->GetMode() == rtc::catching::Mode::kAbortSafe;
    if (aborting && !was_aborting) {
      ++generation;  // the next attempt is at a new ball
    }
    was_aborting = aborting;
  }
  ASSERT_TRUE(planned) << "the oracle plan was never built";
  // The latch is raised in ABORT_SAFE; the escalation edge is the next tick's.
  RunClosedLoop(2, /*publish=*/false);
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

// ── Readiness lost while the arm is moving (code review 2026-09-23) ─────────

TEST_F(CatchingTrackingTest, DisarmingMidApproachRampsTheArmDownAndEndsInIdle) {
  // The operator lowers `catching.enable` during an approach. Before this was
  // fixed the supervisor answered kParamsTbd, the table had no row for it from
  // APPROACH, and so the mode did not move — which meant the driver stopped
  // calling the law while `WriteDeviceCommand` kept sending the carried
  // command. The arm's commanded velocity went to zero in ONE tick: the
  // one-tick infinite deceleration `decel_target.hpp` says this controller
  // must never emit. And the machine never returned to IDLE, so the disarm had
  // no visible effect at all.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.10, 0.08, 0.05);
  const Eigen::Vector3d a_d = start_pose_.rotation().col(2);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, a_d));
  RunClosedLoop(4);
  ASSERT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach);

  // Get the arm genuinely moving, then measure the speed it is moving at.
  RunClosedLoop(60);
  std::array<double, 64> before = commanded_;
  RunClosedLoop(1);
  double moving_step = 0.0;
  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    moving_step = std::max(moving_step, std::abs(commanded_[ui] - before[ui]));
  }
  ASSERT_GT(moving_step, 1e-5) << "the arm was not moving, so this case cannot show a freeze";

  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, false));
  executor_->spin_some(2ms);

  // The very next tick must route the readiness loss through the stop, not
  // drop the command where it stood.
  before = commanded_;
  RunClosedLoop(1);
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kAbortSafe)
      << "readiness lost in APPROACH did not reach the ramp";
  double first_stop_step = 0.0;
  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    first_stop_step = std::max(first_stop_step, std::abs(commanded_[ui] - before[ui]));
  }
  // A ramp bounded by qdd_max cannot shed a whole tick's motion in one tick,
  // so the first stopping step is still a large fraction of the moving one.
  // A freeze would make this exactly zero.
  EXPECT_GT(first_stop_step, 0.2 * moving_step)
      << "the command stopped in one tick (" << first_stop_step << " vs " << moving_step
      << ") — that is a velocity step, not a deceleration";

  // And the cycle terminates: ABORT_SAFE holds until the arm has stopped, then
  // RETREAT, and from RETREAT readiness-lost goes to IDLE. A disarm that does
  // not end in IDLE is one the operator cannot see.
  RunClosedLoop(400, /*publish=*/false);
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kIdle)
      << "the machine never returned to IDLE after the disarm";
}

TEST_F(CatchingTrackingTest, TheStopStaysInsideTheBoxTheSolverWasGiven) {
  // `JointSpaceDecelStep` documents its bounds as "the caller's box, already
  // narrowed by limit_margin". Handing it the raw device limits instead lets
  // the ramp integrate out past the value CLIK was kept away from, and the
  // backend's clamp of that is invisible to the solver — the unattributable
  // command/solution mismatch the margin exists to prevent.
  //
  // The difference is only OBSERVABLE where the two boxes differ, so this case
  // has to manufacture that: the shipped UR5e limits are +/-6.28 and a 0.05
  // margin is nowhere near the arm in this fixture, which is why a first
  // version of this test passed against the defect. Here `shoulder_pan` is
  // fenced 0.12 rad above home, so the margined bound sits 0.05 rad inside a
  // bound the ramp can actually reach, and the catch point pushes that joint
  // straight at it.
  constexpr double kMargin = 0.05;
  constexpr double kFence = 0.12;
  const double pan_home = kUr5eHome[0];
  ASSERT_NO_FATAL_FAILURE(BringUp(
      start_pose_.translation() + Eigen::Vector3d(0.02, 0.22, 0.0), start_pose_.rotation().col(2),
      /*gamma_f=*/0.0, /*t_c_offset_s=*/1.0,
      [kMargin](YAML::Node& y) {
        y["demo_catching_controller"]["catching"]["robot"]["arm"]["limit_margin"] = kMargin;
      },
      [pan_home, kFence](std::map<std::string, rtc::DeviceNameConfig>& devices) {
        auto& lim = devices["ur5e"].joint_limits;
        ASSERT_TRUE(lim.has_value()) << "precondition: the fixture arm declares joint limits";
        lim->position_lower[0] = pan_home - kFence;
        lim->position_upper[0] = pan_home + kFence;
      }));
  RunClosedLoop(4);
  ASSERT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach);

  const auto lo = ctrl_->GetArmPositionBoxLowerForTesting();
  const auto hi = ctrl_->GetArmPositionBoxUpperForTesting();
  ASSERT_EQ(static_cast<int>(lo.size()), kUr5eArmDof) << "the margined box was never built";
  ASSERT_NEAR(hi[0], pan_home + kFence - kMargin, 1e-12) << "precondition: the box is margined";

  // Drive the pan joint at the fence, then pull readiness out from under it so
  // the QP-independent ramp — not CLIK — is what finishes the motion.
  RunClosedLoop(120);
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, false));
  executor_->spin_some(2ms);
  RunClosedLoop(400, /*publish=*/false);

  for (int i = 0; i < kUr5eArmDof; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_GE(commanded_[ui], lo[ui] - 1e-9) << "joint " << i << " stopped below the solver's box";
    EXPECT_LE(commanded_[ui], hi[ui] + 1e-9) << "joint " << i << " stopped above the solver's box";
  }
}

TEST_F(CatchingTrackingTest, ANewBallReusingTheOldSequenceStillRetreats) {
  // TRACK_CHANGED had no test anywhere before this one, which is how a
  // sequence-only newness rule survived: A-S5-4 lets a new epoch restart its
  // numbering at any value, and the supervisor's epoch comparison lives inside
  // the `is_new` branch. So a new ball whose first snapshot happened to reuse
  // the last number was consumed — the controller tracked the NEW ball's
  // samples — while the supervisor was never told the track had changed.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.04, 0.03, 0.02);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, start_pose_.rotation().col(2)));
  RunClosedLoop(4);
  ASSERT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach);

  // A different ball, first snapshot, numbered exactly like the last one of
  // the previous track.
  const std::uint64_t reused = static_cast<std::uint64_t>(3 / 15) + 1;
  PublishPrediction(reused, /*generation=*/43);
  state_.iteration += 1;
  state_.t_relative_s = static_cast<double>(state_.iteration) * kDt;
  (void)ctrl_->Compute(state_);

  EXPECT_EQ(ctrl_->GetLastReason(), rtc::catching::Reason::kTrackChanged)
      << "a new track reusing the last sequence was read as a repeat";
  EXPECT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kRetreat)
      << "APPROACH did not drop the plan built for the previous ball";
}

TEST_F(CatchingTrackingTest, TheTrajectorySubscriptionStaysOnTheNodesDefaultGroup) {
  // The ingress's thread-safety is NOT enforced by a lock: `CatchingTrajInput`
  // has no synchronisation at all, and `ingress_diag_box_` is a single-writer
  // SeqLock. on_activate is a SECOND writer to both, and what makes that safe
  // today is only that the subscription callback and the lifecycle call land
  // on the same thread — the CM attaches every controller LifecycleNode's
  // DEFAULT callback group to one SingleThreadedExecutor, and runs
  // switch_controller (hence on_activate) as a callback on that same executor.
  //
  // Nothing asserted that before this test. Giving the subscription an
  // explicit callback group, or moving it to a reentrant one, would put a
  // genuine data race into a lane whose only symptom is a torn snapshot. This
  // locks the half that lives in this package: the sub is created with no
  // SubscriptionOptions, so rclcpp routes it to the default group.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.04, 0.03, 0.02);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, start_pose_.rotation().col(2)));

  bool found = false;
  node_->get_node_base_interface()->get_default_callback_group()->find_subscription_ptrs_if(
      [&](const rclcpp::SubscriptionBase::SharedPtr& sub) {
        if (std::string(sub->get_topic_name()).find(topic_) != std::string::npos) {
          found = true;
        }
        return false;
      });
  EXPECT_TRUE(found) << "the vision subscription is no longer on the node's default callback "
                        "group — on_activate's Reset()/Store() are then a real second writer";
}

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
  // Watch for the abort rather than asserting on one tick: with the arm
  // already at rest the stop completes immediately, so ABORT_SAFE can come and
  // go inside a couple of ticks. What this test needs to know is that it
  // HAPPENED — otherwise the assertions below would pass on a controller that
  // never aborted at all. Watched from the FIRST tick: the degenerate plan
  // fails on the first law tick, which is the fourth tick of the bring-up,
  // and a window that opened after it would miss the abort it is waiting for.
  bool saw_plan = false;
  bool saw_abort = false;
  for (int t = 0; t < 12 && !saw_abort; ++t) {
    RunClosedLoop(1);
    saw_plan = saw_plan || ctrl_->IsPlanActive();
    saw_abort = ctrl_->GetMode() == rtc::catching::Mode::kAbortSafe;
  }
  ASSERT_TRUE(saw_plan);
  ASSERT_TRUE(saw_abort) << "precondition: the degenerate plan never aborted";

  // No more predictions: the lane goes stale (t_stale is 0.2 s here).
  RunClosedLoop(150, /*publish=*/false);
  EXPECT_NE(ctrl_->GetMode(), rtc::catching::Mode::kAbortSafe)
      << "the abort never completed with a stale vision lane";
  EXPECT_FALSE(ctrl_->IsPlanActive()) << "the aborted plan was never dropped";
}

// ── PROC-7: the per-tick record is rebuilt, not carried forward (S5.4) ──────

TEST_F(CatchingTrackingTest, ATickThatDoesNotRunTheLawClearsTheBlocksItDidNotCompute) {
  // The one assertion that needs a real model: only a tick that actually
  // reaches the reference generator and the QP can FILL those blocks, so only
  // here can "cleared on the next tick" be told apart from "never written".
  //
  // The failure this pins is the one PROC-7 exists for. A record that carried
  // the last tracking tick's reference and solve time forward would make an
  // E-STOP look, in the file and on the operator's screen, like a tick that
  // solved normally — with a γ, a task reference and a solve time that were
  // computed for a command nobody sent.
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.04, 0.03, 0.02);
  const Eigen::Vector3d a_d = start_pose_.rotation().col(2);
  ASSERT_NO_FATAL_FAILURE(BringUp(p_c, a_d));
  RunClosedLoop(20);
  ASSERT_EQ(ctrl_->GetMode(), rtc::catching::Mode::kApproach);

  const auto tracking = ctrl_->GetLastTickRecord();
  ASSERT_TRUE(tracking.ref_valid) << "precondition: the law ran on this tick";
  ASSERT_TRUE(tracking.clik_ran);
  ASSERT_GT(tracking.clik_solve_us, 0.0);
  ASSERT_GT(Eigen::Vector3d(tracking.ref_x[0], tracking.ref_x[1], tracking.ref_x[2]).norm(), 0.0);
  ASSERT_TRUE(tracking.plan_valid);
  ASSERT_NEAR(tracking.plan_p_c[0], p_c.x(), 1e-9);
  ASSERT_GT(tracking.track_err_rad, 0.0);
  const std::uint64_t tracking_tick = tracking.tick;

  // A stop. The law does not run, and CM substitutes its own hold anyway —
  // so what the row says about this tick is the only record of it.
  ctrl_->TriggerEstop();
  RunClosedLoop(1, /*publish=*/false);

  const auto stopped = ctrl_->GetLastTickRecord();
  EXPECT_GT(stopped.tick, tracking_tick) << "the record is still the tracking tick's";
  EXPECT_TRUE(stopped.estop_active);
  EXPECT_FALSE(stopped.ref_valid);
  EXPECT_DOUBLE_EQ(stopped.ref_x[0], 0.0);
  EXPECT_DOUBLE_EQ(stopped.ref_gamma, 0.0);
  EXPECT_DOUBLE_EQ(stopped.ref_u_des[2], 0.0);
  EXPECT_FALSE(stopped.clik_ran);
  EXPECT_EQ(stopped.clik_status, -1) << "0 is ProxQP's SOLVED — a stopped tick must not claim it";
  EXPECT_DOUBLE_EQ(stopped.clik_solve_us, 0.0);
  EXPECT_DOUBLE_EQ(stopped.track_err_rad, 0.0);
  // And the plan is GONE, which is a different mechanism from the one above:
  // the stop's reset (P-1 (b)) invalidates it, so the record shows the
  // invalidation rather than a plan the controller is no longer following.
  // Asserting it here is what keeps the two apart — the reference and solve
  // blocks are empty because this tick did not compute them, the plan block is
  // empty because there is no longer a plan.
  EXPECT_FALSE(stopped.plan_valid) << "the stop did not invalidate the plan";
  EXPECT_DOUBLE_EQ(stopped.plan_p_c[0], 0.0);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
