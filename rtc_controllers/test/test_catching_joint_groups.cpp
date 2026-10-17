// E3-F02 (#641): `planner.groups` — the parser (names and joint names as
// written), the resolution against the arm device's joints, and the model-order
// map a segment planner reads.
#include "rtc_controllers/catching/joint_groups.hpp"
#include "rtc_controllers/catching/planner_params.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using rtc::catching::GroupsOfModel;
using rtc::catching::JointGroups;
using rtc::catching::JointGroupSpec;
using rtc::catching::kMaxPlanNv;
using rtc::catching::ParsePlannerParams;
using rtc::catching::ResolveJointGroups;

const std::vector<std::string> kDevice = {"j0", "j1", "j2", "j3", "j4", "j5", "j6"};

// The parser's message for `yaml`, or "" when it parses.
std::string ParseError(const std::string& yaml) {
  try {
    (void)ParsePlannerParams(YAML::Load(yaml));
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return {};
}

TEST(JointGroupsParse, AbsentIsNoGroup) {
  EXPECT_TRUE(ParsePlannerParams(YAML::Load("planner: {enabled: true}")).groups.empty());
  EXPECT_TRUE(rtc::catching::PlannerParams{}.groups.empty());
}

TEST(JointGroupsParse, ReadsNamesAndJointsAsWritten) {
  const auto p = ParsePlannerParams(
      YAML::Load("planner: {groups: [{name: trunk, joints: [j0, j1]}, {name: b_2, joints: [j6, "
                 "j2, j3, j4, j5]}]}"));
  ASSERT_EQ(p.groups.size(), 2U);
  EXPECT_EQ(p.groups[0].name, "trunk");
  EXPECT_EQ(p.groups[0].joints, (std::vector<std::string>{"j0", "j1"}));
  EXPECT_EQ(p.groups[1].name, "b_2");
  EXPECT_EQ(p.groups[1].joints, (std::vector<std::string>{"j6", "j2", "j3", "j4", "j5"}));
}

TEST(JointGroupsParse, RejectsAMalformedDeclarationByName) {
  struct Case {
    const char* yaml;
    const char* says;
  };

  for (const Case& c : {
           Case{"planner: {groups: 3}", "'planner.groups' must be a non-empty sequence"},
           Case{"planner: {groups: []}", "'planner.groups' must be a non-empty sequence"},
           Case{"planner: {groups: [{name: a, joints: [j0]}, {name: b, joints: [j1]}, {name: c, "
                "joints: [j2]}, {name: d, joints: [j3]}, {name: e, joints: [j4]}]}",
                "at most 4 (kMaxJointGroups)"},
           Case{"planner: {groups: [3]}", "'planner.groups[0]' must be a map"},
           Case{"planner: {groups: [{joints: [j0]}]}", "'planner.groups[0].name' must be a"},
           Case{"planner: {groups: [{name: 1a, joints: [j0]}]}",
                "'planner.groups[0].name' must be a"},
           Case{"planner: {groups: [{name: a.b, joints: [j0]}]}",
                "'planner.groups[0].name' must be a"},
           Case{"planner: {groups: [{name: a, joints: [j0]}, {name: a, joints: [j1]}]}",
                "'planner.groups[1].name' 'a' names a group already declared"},
           Case{"planner: {groups: [{name: a}]}", "'planner.groups[0].joints' must be a non-empty"},
           Case{"planner: {groups: [{name: a, joints: []}]}",
                "'planner.groups[0].joints' must be a non-empty"},
           Case{"planner: {groups: [{name: a, joints: [[j0]]}]}",
                "'planner.groups[0].joints[0]' must be a joint name"},
           Case{"planner: {groups: [{name: a, joints: [j0, j1]}, {name: b, joints: [j1]}]}",
                "'planner.groups[1].joints' lists 'j1', which is already in a group"},
           Case{"planner: {groups: [{name: a, joints: [j0, j0]}]}",
                "'planner.groups[0].joints' lists 'j0', which is already in a group"},
           Case{"planner: {groups: [{name: a, joints: [j0], weight: 1}]}",
                "'planner.groups[0].weight' is not a key of a group"},
       }) {
    SCOPED_TRACE(c.yaml);
    const std::string what = ParseError(c.yaml);
    EXPECT_NE(what.find(c.says), std::string::npos) << what;
  }
}

// ── Resolution against the device ────────────────────────────────────────────

TEST(JointGroupsResolve, NoGroupIsOneGroupOfEveryJoint) {
  JointGroups g;
  g.n_groups = 3;  // overwritten
  std::string why;
  ASSERT_TRUE(ResolveJointGroups({}, kDevice, g, &why)) << why;
  EXPECT_EQ(g.n_groups, 0);
  EXPECT_EQ(g.Count(), 1);
  for (int d = 0; d < static_cast<int>(kDevice.size()); ++d) {
    EXPECT_EQ(g.GroupOf(d), 0);
  }
}

TEST(JointGroupsResolve, MapsEveryDeviceJointToItsGroupWhateverTheOrderWritten) {
  const std::vector<JointGroupSpec> specs = {
      {"a", {"j5", "j0"}}, {"b", {"j1", "j6", "j3"}}, {"c", {"j4", "j2"}}};
  JointGroups g;
  std::string why;
  ASSERT_TRUE(ResolveJointGroups(specs, kDevice, g, &why)) << why;
  EXPECT_EQ(g.n_groups, 3);
  EXPECT_EQ(g.Count(), 3);
  const std::array<int, 7> expected = {0, 1, 2, 1, 2, 0, 1};
  for (int d = 0; d < 7; ++d) {
    EXPECT_EQ(g.GroupOf(d), expected[static_cast<std::size_t>(d)]) << "device joint " << d;
  }
}

TEST(JointGroupsResolve, RefusesAJointTheDeviceDoesNotHaveAndAJointInNoGroup) {
  JointGroups g;
  g.n_groups = 2;
  g.group_of_device[0] = 1;
  const JointGroups before = g;

  struct Case {
    std::vector<JointGroupSpec> specs;
    const char* says;
  };

  const std::vector<Case> cases = {
      // A misspelt joint, or one of another device.
      {{{"a", {"j0", "j1", "j2"}}, {"b", {"j3", "j4", "j5", "other_j6"}}},
       "'planner.groups[1].joints' names 'other_j6', which is not a joint of the arm device"},
      // The groups must partition the device.
      {{{"a", {"j0", "j1", "j2"}}, {"b", {"j3", "j4", "j5"}}},
       "'planner.groups' leaves joint 'j6' in no group"},
      // Disjoint (the parser refuses this too; the resolver does not rely on it).
      {{{"a", {"j0", "j1", "j2", "j3"}}, {"b", {"j3", "j4", "j5", "j6"}}},
       "'planner.groups' puts joint 'j3' in two groups"},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.says);
    std::string why;
    EXPECT_FALSE(ResolveJointGroups(c.specs, kDevice, g, &why));
    EXPECT_NE(why.find(c.says), std::string::npos) << why;
    EXPECT_EQ(g.n_groups, before.n_groups) << "written on failure";
    EXPECT_EQ(g.group_of_device, before.group_of_device) << "written on failure";
  }
  // The control: the same declaration with every joint in place resolves.
  std::string why;
  EXPECT_TRUE(ResolveJointGroups(
      std::vector<JointGroupSpec>{{"a", {"j0", "j1", "j2"}}, {"b", {"j3", "j4", "j5", "j6"}}},
      kDevice, g, &why))
      << why;
}

// ── The model-order map ───────────────────────────────────────────────────────

TEST(JointGroupsOfModel, ReadsTheGroupThroughDeviceOfModel) {
  const std::vector<JointGroupSpec> specs = {{"a", {"j0", "j1", "j2"}},
                                             {"b", {"j3", "j4", "j5", "j6"}}};
  JointGroups g;
  std::string why;
  ASSERT_TRUE(ResolveJointGroups(specs, kDevice, g, &why)) << why;
  // Reversed: model joint m is device joint 6 − m, so an identity map is wrong
  // on every joint but the middle one.
  const std::array<int, 7> reversed = {6, 5, 4, 3, 2, 1, 0};
  std::array<std::int8_t, kMaxPlanNv> of_model{};
  ASSERT_TRUE(GroupsOfModel(g, reversed, 7, of_model));
  const std::array<int, 7> expected = {1, 1, 1, 1, 0, 0, 0};
  for (int m = 0; m < 7; ++m) {
    EXPECT_EQ(of_model[static_cast<std::size_t>(m)], expected[static_cast<std::size_t>(m)])
        << "model joint " << m;
  }
  // A model of some of the device's joints (a chain) reads them the same way.
  const std::array<int, 3> chain = {5, 0, 3};
  ASSERT_TRUE(GroupsOfModel(g, chain, 3, of_model));
  EXPECT_EQ(of_model[0], 1);
  EXPECT_EQ(of_model[1], 0);
  EXPECT_EQ(of_model[2], 1);
  // No group: every model joint is in group 0.
  ASSERT_TRUE(GroupsOfModel(JointGroups{}, reversed, 7, of_model));
  for (int m = 0; m < 7; ++m) {
    EXPECT_EQ(of_model[static_cast<std::size_t>(m)], 0);
  }
  // Out of range.
  const std::array<int, 2> bad = {0, kMaxPlanNv};
  EXPECT_FALSE(GroupsOfModel(g, bad, 2, of_model));
  EXPECT_FALSE(GroupsOfModel(g, reversed, 8, of_model)) << "nv beyond the map";
  EXPECT_FALSE(GroupsOfModel(g, reversed, 0, of_model));
}

}  // namespace
