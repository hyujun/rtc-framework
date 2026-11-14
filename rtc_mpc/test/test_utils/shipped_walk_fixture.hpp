#pragma once

/// @file shipped_walk_fixture.hpp
/// @brief The g1_loco profile's walk as shipped — copies of its gait, swing,
///        terrain and reference sections (this package reads no profile file)
///        — and a window of it on the public G1 from the stand pose. Shared by
///        the kinodynamic OCP, receding-horizon and phase-manager suites. Not
///        installed — test-local by design.

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/reference/command_filter.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"
#include "test_utils/loco_reference_fixture.hpp"
#include "test_utils/shipped_walk_yaml.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace rtc::mpc::test_utils {

/// A window of the shipped walk at @p t0, straight at @p v_x [m/s], the
/// shipped nominal pelvis height (0.7925 m): stance from `t0 − 4`, walk
/// requested at once, taken over before `t0` (policy off).
struct ShippedWalkWindow {
  rtc::mpc::GaitScheduler scheduler;
  rtc::mpc::StageReferenceBuilder builder;

  ShippedWalkWindow(const pinocchio::Model& model, const rtc::mpc::RobotModelHandler& handler,
                    const Eigen::VectorXd& stand, double t0, double v_x) {
    rtc::mpc::GaitTable table;
    const std::vector<std::string> contacts = {"left_foot", "right_foot"};
    EXPECT_EQ(rtc::mpc::LoadGaitTable(YAML::Load(kShippedGaitYaml), contacts, table),
              rtc::mpc::GaitTableError::kNoError);
    EXPECT_EQ(scheduler.Init(table, kKinoNodes * kKinoDt), rtc::mpc::GaitSchedulerError::kNoError);
    rtc::mpc::ReferenceConfig ref;
    EXPECT_EQ(rtc::mpc::LoadReferenceConfig(YAML::Load(kShippedReferenceYaml),
                                            ActuatedJointNames(model), ref),
              rtc::mpc::ReferenceError::kNoError);
    EXPECT_EQ(builder.Init(handler, ref, ShippedSwing(), rtc::mpc::FlatSurface(0.0), kKinoNodes + 1,
                           kKinoDt),
              rtc::mpc::ReferenceError::kNoError);
    scheduler.SetPolicyEnabled(false);
    rtc::mpc::FilteredCommand walk;
    walk.v_x = 0.5;
    walk.valid = true;
    EXPECT_EQ(scheduler.Advance(t0 - 4.0, walk, rtc::mpc::PlanarVelocity{}),
              rtc::mpc::GaitSchedulerError::kNoError);
    scheduler.RequestGait(2);
    for (const double t : {t0 - 3.9, t0 - 3.0, t0 - 2.0, t0 - 1.0, t0}) {
      EXPECT_EQ(scheduler.Advance(t, walk, rtc::mpc::PlanarVelocity{}),
                rtc::mpc::GaitSchedulerError::kNoError);
    }
    EXPECT_EQ(scheduler.current_gait(), 2U);
    EXPECT_FALSE(scheduler.switch_pending());
    cmd = RegisteredCommand();
    cmd.v_x = v_x;
    cmd.v_y = 0.0;
    cmd.omega_z = 0.0;
    cmd.base_height = 0.7925;
    EXPECT_EQ(scheduler.Advance(t0, cmd, rtc::mpc::PlanarVelocity{}),
              rtc::mpc::GaitSchedulerError::kNoError);
    EXPECT_EQ(builder.Build(scheduler, SnapshotAt(model, stand), cmd),
              rtc::mpc::ReferenceError::kNoError);
  }

  [[nodiscard]] const HorizonReference& horizon() const { return builder.active(); }

  rtc::mpc::FilteredCommand cmd;  ///< the command of the window (and of later ticks)
};

/// The gait table's contact names of the G1 soles fixture, in its contact order.
inline std::vector<std::string> ShippedContactNames() {
  return {"left_foot", "right_foot"};
}

/// The `LocomotionPhaseManager` section of the shipped profile: the OCP's
/// horizon and the gait, swing, terrain and reference sections above.
inline YAML::Node ShippedLocomotionYaml() {
  YAML::Node n;
  n["horizon_nodes"] = kKinoNodes;
  n["dt"] = kKinoDt;
  n["gait"] = YAML::Load(kShippedGaitYaml);
  n["swing"] = YAML::Load(kShippedSwingYaml);
  n["terrain"] = YAML::Load(kShippedTerrainYaml);
  n["reference"] = YAML::Load(kShippedReferenceYaml);
  return n;
}

}  // namespace rtc::mpc::test_utils
