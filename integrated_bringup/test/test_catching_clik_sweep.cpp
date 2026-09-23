// ── What the QP CLIK does across the ball envelope (S5.5 characterisation) ──
//
// The S5.3 suite answers "does the law converge on ONE catch". This one asks
// how its task-space POSITION and AXIS performance move as the ball changes,
// which is the question the gate maps (plan §11 S3.5b) leave open: they say
// which catch poses are reachable, not how well the law reaches them when the
// ball gives it less time and a different approach direction.
//
// WHERE THE BALL ENTERS THE LAW. Two places, and only two:
//   * `t_c` — the γ ramp spans [plan, t_c] (L4 §4.2), so a faster ball is a
//     shorter ramp. Here t_c = L/v with L the detection-to-catch distance, so
//     speed sets the time budget directly.
//   * the SAMPLE — with γ_f > 0 the reference blends toward the ball's own
//     position and velocity at the lead instant, so the trajectory's direction
//     and speed enter the target, not just the endpoint.
// The approach axis `a_d` leans the palm TOWARD −v̂ — opposing the incoming
// ball — but by a BOUNDED angle (`kMaxTiltDeg`). Taking −v̂ outright is what a
// first version of this sweep did, and it measured the wrong thing: it demands
// a 42–87° reorientation from the arm's start pose, which is a REACHABILITY
// question (S3.5a's catchability map, and S6's job to answer) and not a
// question about how well the QP tracks. The symptom was unmistakable — the
// end-to-end error FELL as the ball got faster (280 mm at 2 m/s, 114 mm at
// 4.85 m/s), because a faster ball simply gave the diverging axis task less
// time to drag the hand away. A planner picks a pose it can hold; this sweep
// gives the law the same courtesy and then asks how well it gets there.
//
// γ_f = 0 would drop the ball out of the dynamics entirely (L4 §5.3) and the
// sweep would measure the same step response 36 times.
//
// THE PROFILE IS THE SHIPPED ONE. `ur5e_p1b`'s reference gains (ω = 10, v_max
// 3.5, a_max 21) rather than the S5.3 suite's faster ω = 20 — the question is
// about the law that is deployed. ω = 10 is a ~0.4 s settling time, and the
// fast end of this sweep gives it less than that, which is a result and not a
// test failure. The assertions below are therefore about SOLVER HEALTH
// (convergence, boxes, budget), which must hold at every speed, and the errors
// are written to a CSV for the analysis to read.
//
// TWO ERRORS, KEPT APART. `clik_err` is ‖catch frame − the reference the
// solver was handed this tick‖ — the QP's OWN tracking. `catch_err` is
// ‖catch frame − p_c‖ at the catch instant — the whole chain. Reporting only
// the second would blame the solver for a reference that was never going to
// arrive in time, which is precisely the confusion this file exists to
// prevent.

#include "catching_cloud_fixture.hpp"
#include "catching_tracking_fixture.hpp"
#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::testfx::CatchFrameOracle;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kUr5eArmDof;
using integrated_bringup::testfx::kUr5eHome;
using integrated_bringup::testfx::MakeConfigWithCatchFrame;
using integrated_bringup::testfx::ReferenceGains;
using integrated_bringup::testfx::TrackingYaml;
using rtc::ControllerOutput;
using rtc::ControllerState;

using namespace std::chrono_literals;

/// The shipped `ur5e_p1b` reference block (controllers/demo_catching_controller.yaml).
constexpr ReferenceGains kShipped{/*omega=*/10.0, /*v_max=*/3.5, /*a_max=*/21.0,
                                  /*track_err_abort=*/0.3};

/// How far from the catch point the lane is assumed to pick the ball up. The
/// time budget is L/v, so this constant is what turns "speed" into "seconds".
/// 1.2 m is the shipped sim's own order of magnitude (spawn 0.49 m behind the
/// base, catch 0.57 m in front of it).
constexpr double kDetectRangeM = 1.2;

/// How far the palm is allowed to lean toward the incoming ball. A catch pose
/// is chosen by the planner from the reachable set, so the axis task here is
/// ball-derived in DIRECTION and bounded in magnitude — see the header note.
constexpr double kMaxTiltDeg = 25.0;

/// γ_f. L7 §4.5's worked example uses 0.25, and it is large enough that the
/// ball's velocity is visible in the reference without making the terminal
/// relative speed unphysical for a 57 g ball.
constexpr double kGammaF = 0.25;

/// The sweep axes. Speeds span the shipped sim's projectile (2.0 m/s) to the
/// upper end the S3.5b gate map measured (4.85 m/s). Elevations are the
/// DESCENT angles at the catch; azimuths rotate the approach around the base.
constexpr double kSpeeds[] = {2.0, 3.0, 4.0, 4.85};
constexpr double kElevationsDeg[] = {35.0, 50.0, 63.0};
constexpr double kAzimuthsDeg[] = {-20.0, 0.0, 20.0};

struct CaseResult {
  double speed{};
  double elevation_deg{};
  double azimuth_deg{};
  double t_c_s{};
  double axis_demand_deg{};   // the reorientation the case asks for
  double catch_err_m{};       // ‖catch frame − p_c‖ at the catch instant
  double axis_err_deg{};      // ∠(catch frame z, a_d) at the catch instant
  double clik_err_max_m{};    // worst ‖catch frame − reference‖ over the run
  double clik_err_final_m{};  // the same at the catch instant
  int ticks{};
  int non_converged{};
  int bound_conflict{};
  int ref_saturated{};
  double solve_us_max{};
  double solve_us_mean{};
  std::string end_mode;
};

std::int64_t SteadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

class ClikSweepTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("catching_clik_sweep");
    builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(MakeConfigWithCatchFrame());
    oracle_ = std::make_unique<CatchFrameOracle>(*builder_);
    arm_names_ =
        integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs().at("ur5e").joint_state_names;
    topic_ = "/test_catching_clik_sweep/prediction";
    std::array<double, 64> home{};
    for (int i = 0; i < kUr5eArmDof; ++i) {
      home[static_cast<std::size_t>(i)] = kUr5eHome[static_cast<std::size_t>(i)];
    }
    start_pose_ = oracle_->PoseAt(arm_names_, home, kUr5eArmDof);
  }

  void TearDown() override {
    if (executor_ && node_) {
      executor_->remove_node(node_->get_node_base_interface());
    }
    pub_.reset();
    executor_.reset();
    ctrl_.reset();
    node_.reset();
  }

  /// One (speed, elevation, azimuth) case, start to catch instant.
  CaseResult RunCase(double speed, double elevation_deg, double azimuth_deg) {
    const double el = elevation_deg * M_PI / 180.0;
    const double az = azimuth_deg * M_PI / 180.0;
    // A DESCENDING arrival: +x-ish, rotated by the azimuth, falling at `el`.
    const Eigen::Vector3d dir{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                              -std::sin(el)};
    const Eigen::Vector3d v_ball = speed * dir;
    // The catch point rotates with the azimuth: a ball arriving from the left
    // is caught to the left. The magnitude is a reachable ~8 cm from home.
    const Eigen::AngleAxisd yaw(az, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d p_c = start_pose_.translation() + yaw * Eigen::Vector3d(0.06, 0.05, 0.03);
    // Lean the start axis toward −v̂, capped at kMaxTiltDeg.
    const Eigen::Vector3d z0 = start_pose_.rotation().col(2).normalized();
    const Eigen::Vector3d want = (-dir).normalized();
    const double full = std::acos(std::clamp(z0.dot(want), -1.0, 1.0));
    const double tilt = std::min(full, kMaxTiltDeg * M_PI / 180.0);
    Eigen::Vector3d axis = z0.cross(want);
    const Eigen::Vector3d a_d =
        axis.norm() < 1e-9 ? z0 : (Eigen::AngleAxisd(tilt, axis.normalized()) * z0).normalized();
    const double t_c = kDetectRangeM / speed;

    ctrl_ = std::make_unique<DemoCatchingController>("");
    ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
    ctrl_->SetSharedModelBuilder(builder_);
    ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
    const rclcpp_lifecycle::State prev;
    YAML::Node yaml = YAML::Load(TrackingYaml(topic_, p_c, a_d, kGammaF, t_c, kShipped));
    // This sweep measures the SOLVER across the ball envelope, up to the catch
    // instant; the S7 supervisor's own gates are not what it is about, and
    // two of them would end cases before they measured anything:
    //  - admission (g) refuses a plan whose t_c is inside T_freeze, and the
    //    fast balls here are caught 0.25 s after detection — inside the
    //    fixture's 0.36 s. The hand's closure time is what sets that window
    //    (T_freeze >= T_close_e2e + T_arm + h), and the hand plays no part in
    //    a CLIK sweep, so both are shortened to fit the fastest ball;
    //  - REF_SATURATED would abort the approaches whose saturation this sweep
    //    RECORDS (r.ref_saturated) rather than obeys.
    yaml["catching"]["robot"]["hand"]["T_close_e2e"] = 0.1;
    yaml["catching"]["planner"]["freeze"]["T_freeze"] = 0.2;
    yaml["catching"]["supervisor"]["sat_ticks"] = 1000000;
    EXPECT_EQ(ctrl_->on_configure(prev, node_, yaml),
              DemoCatchingController::CallbackReturn::SUCCESS);
    EXPECT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
    node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    rclcpp::QoS qos{rclcpp::KeepLast(1)};
    qos.best_effort();
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    ControllerState state{};
    state.num_devices = 2;
    state.dt = kDt;
    state.devices[0].num_channels = kUr5eArmDof;
    state.devices[0].valid = true;
    for (int i = 0; i < kUr5eArmDof; ++i) {
      state.devices[0].positions[static_cast<std::size_t>(i)] =
          kUr5eHome[static_cast<std::size_t>(i)];
    }
    state.devices[1].num_channels = integrated_bringup::testfx::kP1bHandDof;
    state.devices[1].valid = true;

    CaseResult r{};
    r.speed = speed;
    r.elevation_deg = elevation_deg;
    r.azimuth_deg = azimuth_deg;
    r.t_c_s = t_c;
    r.axis_demand_deg = tilt * 180.0 / M_PI;

    // The catch INSTANT in this test's own clock. The plan is built on the
    // tick the supervisor first has a usable prediction, so the absolute t_c
    // is pinned there rather than at SetUp: pinning it earlier would measure
    // the bring-up ticks as if the ball had been flying through them.
    std::int64_t t_c_abs = 0;
    std::array<double, 64> commanded{};
    double solve_us_sum = 0.0;
    int solves = 0;
    const int max_ticks = static_cast<int>((t_c + 0.25) / kDt) + 16;

    for (int tick = 0; tick < max_ticks; ++tick) {
      if (tick % 10 == 0) {
        // The ball is a LINE THROUGH p_c AT t_c. Its origin is recomputed on
        // every publish from the current clock, because the cloud's samples
        // start at the message stamp — republishing a fixed origin would make
        // the ball jump backwards every 10 ticks and the measured error would
        // be an artefact of the fixture.
        const std::int64_t now = SteadyNs();
        const double to_catch = t_c_abs == 0 ? t_c : static_cast<double>(t_c_abs - now) * 1e-9;
        const Eigen::Vector3d p0 = p_c - v_ball * to_catch;
        integrated_bringup::testing::CloudSpec spec;
        spec.n = 16;  // 16 x 50 ms = 0.8 s, longer than the slowest t_c here
        spec.sequence = static_cast<std::uint64_t>(tick / 10) + 1;
        spec.p0 = {p0.x(), p0.y(), p0.z()};
        spec.vel = {v_ball.x(), v_ball.y(), v_ball.z()};
        auto msg = integrated_bringup::testing::MakeCloud(spec);
        const std::int64_t wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count() -
                                  5'000'000;
        msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
        msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
        pub_->publish(msg);
        for (int i = 0; i < 8; ++i) {
          executor_->spin_some(2ms);
        }
      }

      state.iteration += 1;
      state.t_relative_s = static_cast<double>(state.iteration) * kDt;
      const ControllerOutput out = ctrl_->Compute(state);
      if (out.devices[0].num_channels >= kUr5eArmDof) {
        for (int i = 0; i < kUr5eArmDof; ++i) {
          const auto ui = static_cast<std::size_t>(i);
          commanded[ui] = out.devices[0].commands[ui];
          state.devices[0].positions[ui] = out.devices[0].commands[ui];  // perfect servo
        }
      }

      const auto rec = ctrl_->GetLastTickRecord();
      if (t_c_abs == 0 && rec.plan_valid) {
        t_c_abs = SteadyNs() + static_cast<std::int64_t>(t_c * 1e9);
      }
      if (rec.clik_ran) {
        r.ticks += 1;
        solves += 1;
        solve_us_sum += rec.clik_solve_us;
        r.solve_us_max = std::max(r.solve_us_max, static_cast<double>(rec.clik_solve_us));
        if (!rec.clik_converged) {
          r.non_converged += 1;
        }
        if (rec.clik_bound_conflict) {
          r.bound_conflict += 1;
        }
      }
      if (rec.ref_saturated) {
        r.ref_saturated += 1;
      }
      if (rec.ref_valid) {
        const pinocchio::SE3 pose = oracle_->PoseAt(arm_names_, commanded, kUr5eArmDof);
        const Eigen::Vector3d ref{rec.ref_x[0], rec.ref_x[1], rec.ref_x[2]};
        const double e = (pose.translation() - ref).norm();
        r.clik_err_max_m = std::max(r.clik_err_max_m, e);
        r.clik_err_final_m = e;
        if (trace_ != nullptr) {
          const Eigen::Vector3d refd{rec.ref_xd[0], rec.ref_xd[1], rec.ref_xd[2]};
          *trace_ << state.t_relative_s << ',' << ref.x() << ',' << ref.y() << ',' << ref.z() << ','
                  << pose.translation().x() << ',' << pose.translation().y() << ','
                  << pose.translation().z() << ',' << refd.norm() << ',' << e << ','
                  << rec.ref_gamma << ',' << (rec.ref_saturated ? 1 : 0) << ',' << rec.track_err_rad
                  << '\n';
        }
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
      if (t_c_abs != 0 && SteadyNs() >= t_c_abs) {
        break;  // the catch instant: this is where the numbers are read
      }
    }

    const pinocchio::SE3 pose = oracle_->PoseAt(arm_names_, commanded, kUr5eArmDof);
    r.catch_err_m = (pose.translation() - p_c).norm();
    const Eigen::Vector3d z = pose.rotation().col(2);
    r.axis_err_deg = std::acos(std::clamp(z.dot(a_d), -1.0, 1.0)) * 180.0 / M_PI;
    r.solve_us_mean = solves > 0 ? solve_us_sum / solves : 0.0;
    r.end_mode = std::to_string(static_cast<int>(ctrl_->GetMode()));

    const rclcpp_lifecycle::State prev2;
    (void)ctrl_->on_deactivate(prev2);
    executor_->remove_node(node_->get_node_base_interface());
    pub_.reset();
    executor_.reset();
    ctrl_.reset();
    return r;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder_;
  std::unique_ptr<CatchFrameOracle> oracle_;
  std::unique_ptr<DemoCatchingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::vector<std::string> arm_names_;
  std::string topic_;
  pinocchio::SE3 start_pose_{pinocchio::SE3::Identity()};
  /// Optional per-tick dump for ONE case, so a surprising aggregate can be
  /// explained rather than guessed at.
  std::ofstream* trace_{nullptr};
};

TEST_F(ClikSweepTest, TheSolverStaysHealthyAcrossTheBallEnvelope) {
  std::vector<CaseResult> results;
  const char* trace_env = std::getenv("CATCHING_SWEEP_TRACE");
  std::ofstream trace;
  if (trace_env != nullptr) {
    trace.open(trace_env);
    trace.precision(9);
    trace << "t,ref_x,ref_y,ref_z,act_x,act_y,act_z,ref_speed,clik_err,gamma,saturated,"
             "track_err_rad\n";
    trace_ = &trace;
  }
  for (double v : kSpeeds) {
    for (double el : kElevationsDeg) {
      for (double az : kAzimuthsDeg) {
        results.push_back(RunCase(v, el, az));
        trace_ = nullptr;  // the first case only
      }
    }
  }
  ASSERT_EQ(results.size(),
            std::size(kSpeeds) * std::size(kElevationsDeg) * std::size(kAzimuthsDeg));

  // The CSV is the deliverable; the assertions below are the regression.
  const char* out_dir = std::getenv("CATCHING_SWEEP_DIR");
  const std::filesystem::path path =
      std::filesystem::path(out_dir != nullptr ? out_dir : ".") / "clik_sweep.csv";
  std::ofstream csv(path);
  csv.precision(9);
  csv << "speed_m_s,elevation_deg,azimuth_deg,t_c_s,axis_demand_deg,catch_err_m,axis_err_deg,clik_"
         "err_max_m,"
         "clik_err_final_m,ticks,non_converged,bound_conflict,ref_saturated,solve_us_mean,"
         "solve_us_max,end_mode\n";
  for (const CaseResult& r : results) {
    csv << r.speed << ',' << r.elevation_deg << ',' << r.azimuth_deg << ',' << r.t_c_s << ','
        << r.axis_demand_deg << ',' << r.catch_err_m << ',' << r.axis_err_deg << ','
        << r.clik_err_max_m << ',' << r.clik_err_final_m << ',' << r.ticks << ',' << r.non_converged
        << ',' << r.bound_conflict << ',' << r.ref_saturated << ',' << r.solve_us_mean << ','
        << r.solve_us_max << ',' << r.end_mode << '\n';
  }
  csv.close();
  std::cout << "clik sweep written to " << path << "\n";

  // SOLVER HEALTH holds at every point of the envelope. These are the claims
  // that must not depend on the ball: a QP that stops converging, or that
  // leaves its budget, is broken whatever the trajectory was. The ERRORS are
  // not asserted here — they are the measurement, and pinning them would turn
  // a characterisation into a tautology.
  for (const CaseResult& r : results) {
    SCOPED_TRACE("v=" + std::to_string(r.speed) + " el=" + std::to_string(r.elevation_deg) +
                 " az=" + std::to_string(r.azimuth_deg));
    EXPECT_GT(r.ticks, 0) << "the law never ran — this case measured nothing";
    EXPECT_EQ(r.non_converged, 0) << "the QP failed to converge";
    EXPECT_LT(r.solve_us_max, 1500.0) << "outside the G5-C max budget";
    EXPECT_TRUE(std::isfinite(r.catch_err_m) && std::isfinite(r.axis_err_deg))
        << "a non-finite pose reached the oracle";
  }
}

TEST_F(ClikSweepTest, HowLongTheShippedLawNeedsForOneCatchPose) {
  // The sweep above reports 10-14 cm at the catch instant and the trace says
  // why: the arm moves about as far as the reference does, but not in the same
  // DIRECTION, because the approach-axis row is pulling it. The catch frame
  // sits 0.145 m from the palm, so a 25 deg reorientation sweeps it ~6 cm all
  // by itself — the two task rows are geometrically coupled and the QP is
  // compromising between them, exactly as its weights ask it to.
  //
  // That leaves one number the sweep cannot give, because every case there
  // stops at t_c: how long the shipped law needs to finish. Without it the
  // sweep's errors read as "the law is bad" instead of "the ball is not giving
  // it enough time", and those call for opposite work — retuning versus
  // planning the catch earlier. So this case runs ONE representative pose to
  // convergence and reports the settling time.
  const double el = 35.0 * M_PI / 180.0;
  const Eigen::Vector3d dir{std::cos(el), 0.0, -std::sin(el)};
  const Eigen::Vector3d p_c = start_pose_.translation() + Eigen::Vector3d(0.06, 0.05, 0.03);
  const Eigen::Vector3d z0 = start_pose_.rotation().col(2).normalized();
  const Eigen::Vector3d want = (-dir).normalized();
  const Eigen::Vector3d axis = z0.cross(want).normalized();
  const Eigen::Vector3d a_d =
      (Eigen::AngleAxisd(kMaxTiltDeg * M_PI / 180.0, axis) * z0).normalized();

  ctrl_ = std::make_unique<DemoCatchingController>("");
  ctrl_->SetSystemModelConfig(MakeConfigWithCatchFrame());
  ctrl_->SetSharedModelBuilder(builder_);
  ctrl_->SetDeviceNameConfigs(integrated_bringup::testfx::MakeUr5eP1bDeviceConfigs());
  const rclcpp_lifecycle::State prev;
  // gamma_f = 0 here: this case is about the LAW's settling time, so the ball
  // is taken out of the dynamics (L4 section 5.3) and the catch point is a
  // static attractor. Leaving it in would measure the chase as well.
  // t_c beyond the 3 s run: from S7 the catch instant ends the approach
  // (DECEL at t_c), and this case measures how long the law needs to arrive.
  YAML::Node yaml = YAML::Load(TrackingYaml(topic_, p_c, a_d, /*gamma_f=*/0.0,
                                            /*t_c_offset_s=*/5.0, kShipped));
  ASSERT_EQ(ctrl_->on_configure(prev, node_, yaml),
            DemoCatchingController::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl_->on_activate(prev), DemoCatchingController::CallbackReturn::SUCCESS);
  node_->set_parameter(rclcpp::Parameter(integrated_bringup::kCatchingEnableParam, true));
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_->get_node_base_interface());
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.best_effort();
  pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

  ControllerState state{};
  state.num_devices = 2;
  state.dt = kDt;
  state.devices[0].num_channels = kUr5eArmDof;
  state.devices[0].valid = true;
  for (int i = 0; i < kUr5eArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] =
        kUr5eHome[static_cast<std::size_t>(i)];
  }
  state.devices[1].num_channels = integrated_bringup::testfx::kP1bHandDof;
  state.devices[1].valid = true;

  std::array<double, 64> commanded{};
  double t_pos_1mm = -1.0;
  double t_axis_1deg = -1.0;
  const int ticks = 1500;  // 3 s
  for (int tick = 0; tick < ticks; ++tick) {
    if (tick % 10 == 0) {
      integrated_bringup::testing::CloudSpec spec;
      spec.n = 16;
      spec.sequence = static_cast<std::uint64_t>(tick / 10) + 1;
      spec.p0 = {p_c.x(), p_c.y(), p_c.z()};
      spec.vel = {0.0, 0.0, 0.0};
      auto msg = integrated_bringup::testing::MakeCloud(spec);
      const std::int64_t wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count() -
                                5'000'000;
      msg.header.stamp.sec = static_cast<std::int32_t>(wall / 1'000'000'000LL);
      msg.header.stamp.nanosec = static_cast<std::uint32_t>(wall % 1'000'000'000LL);
      pub_->publish(msg);
      for (int i = 0; i < 8; ++i) {
        executor_->spin_some(2ms);
      }
    }
    state.iteration += 1;
    state.t_relative_s = static_cast<double>(state.iteration) * kDt;
    const ControllerOutput out = ctrl_->Compute(state);
    if (out.devices[0].num_channels >= kUr5eArmDof) {
      for (int i = 0; i < kUr5eArmDof; ++i) {
        const auto ui = static_cast<std::size_t>(i);
        commanded[ui] = out.devices[0].commands[ui];
        state.devices[0].positions[ui] = out.devices[0].commands[ui];
      }
    }
    const pinocchio::SE3 pose = oracle_->PoseAt(arm_names_, commanded, kUr5eArmDof);
    const double pos_err = (pose.translation() - p_c).norm();
    const double ax_err =
        std::acos(std::clamp(pose.rotation().col(2).dot(a_d), -1.0, 1.0)) * 180.0 / M_PI;
    if (t_pos_1mm < 0.0 && pos_err < 1e-3) {
      t_pos_1mm = state.t_relative_s;
    }
    if (t_axis_1deg < 0.0 && ax_err < 1.0) {
      t_axis_1deg = state.t_relative_s;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
  }

  const pinocchio::SE3 pose = oracle_->PoseAt(arm_names_, commanded, kUr5eArmDof);
  const double pos_err = (pose.translation() - p_c).norm();
  const double ax_err =
      std::acos(std::clamp(pose.rotation().col(2).dot(a_d), -1.0, 1.0)) * 180.0 / M_PI;
  std::cout << "settling on the shipped law (8.3 cm + " << kMaxTiltDeg << " deg):\n"
            << "  position < 1 mm at   : " << t_pos_1mm << " s\n"
            << "  axis     < 1 deg at  : " << t_axis_1deg << " s\n"
            << "  final position error : " << pos_err * 1000.0 << " mm\n"
            << "  final axis error     : " << ax_err << " deg\n";

  // It CONVERGES — that is the claim the sweep's errors have to be read
  // against. The thresholds are loose on purpose: this case exists to say
  // "given enough time the law arrives", not to pin a settling time that a
  // gain change would have to chase.
  EXPECT_LT(pos_err, 2e-3) << "the law did not converge in 3 s";
  EXPECT_LT(ax_err, 1.0) << "the axis did not converge in 3 s";
  EXPECT_GT(t_pos_1mm, 0.0) << "position never reached 1 mm";

  const rclcpp_lifecycle::State prev2;
  (void)ctrl_->on_deactivate(prev2);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
