// ── test_simulator_init.cpp ──────────────────────────────────────────────────
// MJCF-fixture-based tests for Initialize(), joint/sensor discovery, validation.
// ──────────────────────────────────────────────────────────────────────────────
#include "test_fixture.hpp"

#include <gtest/gtest.h>

namespace rtc {
namespace {

TEST(SimulatorInit, HappyPath) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.NumGroups(), 1u);
  EXPECT_EQ(sim.NumGroupJoints(0), 2);
  EXPECT_EQ(sim.NumStateJoints(0), 2);
  EXPECT_TRUE(sim.IsGroupRobot(0));
  EXPECT_EQ(sim.NumJoints(), 2);  // nq == 2 hinge joints
}

TEST(SimulatorInit, InvalidModelPath) {
  auto cfg = test::MakeMinimalConfig();
  cfg.model_path = "/nonexistent/path/to/model.xml";
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, UnknownCommandJoint) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].command_joint_names = {"j1", "does_not_exist"};
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, CommandJointsMustCoverAllXmlJoints) {
  auto cfg = test::MakeMinimalConfig();
  // XML has j1 + j2, YAML covers only j1 → should fail (bidirectional check)
  cfg.groups[0].command_joint_names = {"j1"};
  cfg.groups[0].state_joint_names   = {"j1"};
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, UnknownStateJoint) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].state_joint_names = {"j1", "ghost"};
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, EmptyCommandJoints) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].command_joint_names.clear();
  cfg.groups[0].joint_names.clear();
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, EmptyTopics) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].command_topic.clear();
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, NoGroupsFails) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups.clear();
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, DuplicateGroupNameAcrossRobotAndFake) {
  auto cfg = test::MakeMinimalConfig();
  JointGroupConfig fake;
  fake.name                = "arm";  // duplicate
  fake.command_joint_names = {"j1"};
  fake.command_topic       = "/fake/cmd";
  fake.state_topic         = "/fake/state";
  fake.is_robot            = false;
  cfg.groups.push_back(fake);
  MuJoCoSimulator sim(std::move(cfg));
  EXPECT_FALSE(sim.Initialize());
}

TEST(SimulatorInit, JointNamesExposed) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  const auto& names = sim.GetJointNames(0);
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "j1");
  EXPECT_EQ(names[1], "j2");
}

TEST(SimulatorInit, OutOfRangeGroupIndexIsSafe) {
  MuJoCoSimulator sim(test::MakeMinimalConfig());
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.NumGroupJoints(99), 0);
  EXPECT_FALSE(sim.IsGroupRobot(99));
  EXPECT_TRUE(sim.GetPositions(99).empty());
  EXPECT_TRUE(sim.GetJointNames(99).empty());
}

TEST(SimulatorInit, SensorDiscoveryAuto) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].sensor_names = {"auto"};
  cfg.groups[0].sensor_topic = "/arm/sensors";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_TRUE(sim.HasSensors(0));
  const auto& infos = sim.GetSensorInfos(0);
  EXPECT_EQ(infos.size(), 2u);  // jointpos + force
}

TEST(SimulatorInit, SensorDiscoveryExplicit) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].sensor_names = {"ee_force"};
  cfg.groups[0].sensor_topic = "/arm/sensors";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  const auto& infos = sim.GetSensorInfos(0);
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(infos[0].name, "ee_force");
  EXPECT_EQ(infos[0].dim,  3);  // force sensor = 3D
}

TEST(SimulatorInit, UnknownSensorNameSkipped) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].sensor_names = {"ghost_sensor", "ee_force"};
  cfg.groups[0].sensor_topic = "/arm/sensors";
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.GetSensorInfos(0).size(), 1u);  // ghost skipped
}

TEST(SimulatorInit, StateJointInheritsFromXmlWhenEmpty) {
  auto cfg = test::MakeMinimalConfig();
  cfg.groups[0].state_joint_names.clear();  // → should inherit XML all joints
  MuJoCoSimulator sim(std::move(cfg));
  ASSERT_TRUE(sim.Initialize());
  EXPECT_EQ(sim.NumStateJoints(0), 2);
}

}  // namespace
}  // namespace rtc
