// `planner.groups` — the parser (names and joint names as
// written), the resolution against the arm device's joints, and the model-order
// map a segment planner reads.
#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/joint_groups.hpp"
#include "rtc_controllers/catching/planner_params.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdint>
#include <limits>
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
           Case{"planner: {groups: [{name: names, joints: [j0]}]}",
                "'planner.groups[0].name' 'names' is reserved"},
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

// ── planner.segment.mpc.groups.<name> ─────────────────────────────────────────

constexpr const char* kTwoGroups =
    "groups: [{name: a, joints: [j0, j1]}, {name: b, joints: [j2, j3, j4, j5, j6]}]";

TEST(JointGroupsMpcKeys, AGroupWithoutKeysTakesTheBlocksOwnPartition) {
  const auto p = ParsePlannerParams(YAML::Load(std::string("planner: {") + kTwoGroups + "}"));
  EXPECT_EQ(p.mpc_segment.n_groups, 2);
  for (int g = 0; g < 2; ++g) {
    std::array<int, rtc::catching::kMaxSegmentNodes> base{};
    int n = 0;
    rtc::catching::MpcSegmentGroupStopBase(p.mpc_segment, g, base, n);
    EXPECT_EQ(n, p.mpc_segment.n_blocks);
    EXPECT_EQ(base, p.mpc_segment.blocks);
    EXPECT_EQ(p.mpc_segment.groups[static_cast<std::size_t>(g)].pre_block, 1);
  }
  EXPECT_EQ(ParsePlannerParams(YAML::Load("planner: {}")).mpc_segment.n_groups, 0);
}

TEST(JointGroupsMpcKeys, ReadsEachGroupsBlocksAtItsIndex) {
  const auto p = ParsePlannerParams(YAML::Load(
      std::string("planner: {") + kTwoGroups +
      ", segment: {mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 1, 2, 3]}, replan: "
      "{k_max: 2}, groups: {b: {blocks: {pre: 3, stop: [2, 2, 3]}}}}}}"));
  const auto& d = p.mpc_segment;
  EXPECT_EQ(d.groups[0].pre_block, 1);
  EXPECT_EQ(d.groups[0].n_stop_blocks, 0);
  EXPECT_EQ(d.groups[1].pre_block, 3);
  ASSERT_EQ(d.groups[1].n_stop_blocks, 3);
  EXPECT_EQ(d.groups[1].stop_blocks[2], 3);
  std::array<int, rtc::catching::kMaxSegmentNodes> base{};
  int n = 0;
  rtc::catching::MpcSegmentGroupStopBase(d, 1, base, n);
  EXPECT_EQ(n, 3);
  EXPECT_EQ(base[0], 2);
  // A selection that does not open the mpc map reads none of it.
  rtc::catching::PlannerKeySelection none;
  none.segment_mpc = false;
  const auto q =
      ParsePlannerParams(YAML::Load(std::string("planner: {") + kTwoGroups +
                                    ", segment: {mpc: {groups: {zz: {blocks: {pre: 0}}}}}}"),
                         none);
  EXPECT_EQ(q.mpc_segment.groups[1].pre_block, 1);
}

TEST(JointGroupsMpcKeys, RejectsAGroupKeyByName) {
  struct Case {
    const char* mpc;
    const char* says;
  };

  const std::string head = std::string("planner: {") + kTwoGroups +
                           ", segment: {mpc: {horizon: {n_nodes: 7, dt_s: 0.05, blocks: [1, 1, "
                           "2, 3]}, replan: {k_max: 2}, approach: {n_pre_max: 6}, groups: ";
  for (const Case& c : {
           Case{"3", "'planner.segment.mpc.groups' must be a map"},
           Case{"{c: {}}", "'planner.segment.mpc.groups.c' names no group of 'planner.groups'"},
           Case{"{a: 1}", "'planner.segment.mpc.groups.a' must be a map"},
           Case{"{a: {w_acc: 1}}", "'planner.segment.mpc.groups.a.w_acc' is not a key"},
           Case{"{a: {w_vel: -1}}", "'planner.segment.mpc.groups.a.w_vel'"},
           Case{"{a: {w_vel: 1.0e5}}", "'planner.segment.mpc.groups.a.w_vel'"},
           Case{"{a: {w_rest: 1}}",
                "'planner.segment.mpc.groups.a.w_rest' > 0 holds the group near its rest "
                "posture, 'planner.wait_pose', which is not given"},
           Case{"{a: {blocks: {mid: 1}}}",
                "'planner.segment.mpc.groups.a.blocks.mid' is not a key"},
           Case{"{a: {blocks: {pre: 0}}}", "'planner.segment.mpc.groups.a.blocks.pre'"},
           Case{"{a: {blocks: {stop: [3, 4]}}}",
                "'planner.segment.mpc.groups.a.blocks.stop' must be a sequence of 3.."},
           Case{"{a: {blocks: {stop: [2, 2, 2]}}}",
                "'planner.segment.mpc.groups.a.blocks.stop' sums to 6 but horizon.n_nodes is 7"},
           Case{"{a: {blocks: {stop: [1, 1, 1, 1, 1, 1, 1]}}}", ""},
       }) {
    SCOPED_TRACE(c.mpc);
    const std::string what = ParseError(head + c.mpc + "}}}");
    if (*c.says == 0) {
      EXPECT_EQ(what, "") << "the control";
    } else {
      EXPECT_NE(what.find(c.says), std::string::npos) << what;
    }
  }
  // With the wait pose the rest weight parses.
  EXPECT_EQ(ParseError(std::string("planner: {") + kTwoGroups +
                       ", wait_pose: [0, 0, 0, 0, 0, 0, 0], segment: {mpc: {groups: {a: {w_rest: "
                       "1, w_vel: 0.5}}}}}"),
            "");
  // One group is the whole arm, which the cores block as without groups: its
  // own blocks would be ignored, so they are refused — its weights are not.
  const std::string one =
      "planner: {groups: [{name: all, joints: [j0, j1, j2]}], segment: {mpc: "
      "{groups: {all: ";
  for (const char* entry : {"{blocks: {pre: 2}}", "{blocks: {stop: [2, 2, 3]}}"}) {
    SCOPED_TRACE(entry);
    EXPECT_NE(ParseError(one + entry + "}}}}")
                  .find("'planner.segment.mpc.groups.all.blocks': with one group"),
              std::string::npos);
  }
  EXPECT_EQ(ParseError(one + "{w_vel: 0.5, blocks: {}}}}}}"), "");
  // Groups' keys need groups.
  EXPECT_NE(ParseError("planner: {segment: {mpc: {groups: {a: {}}}}}")
                .find("'planner.segment.mpc.groups.a' names no group of 'planner.groups'"),
            std::string::npos);
}

TEST(JointGroupsMpcKeys, TheAngularMomentumKeysDefaultOffParseAndAreCheckedByName) {
  const auto d0 = ParsePlannerParams(YAML::Load("planner: {}")).mpc_segment;
  EXPECT_EQ(d0.ang_w_pre, 0.0);
  EXPECT_EQ(d0.ang_w_post, 0.0);
  EXPECT_EQ(d0.ang_scale, 1.0);
  const auto d = ParsePlannerParams(
                     YAML::Load("planner: {segment: {mpc: {cost: {angular_momentum: {w_pre: 0.01, "
                                "w_post: 1, scale: 10}}}}}"))
                     .mpc_segment;
  EXPECT_EQ(d.ang_w_pre, 0.01);
  EXPECT_EQ(d.ang_w_post, 1.0);
  EXPECT_EQ(d.ang_scale, 10.0);

  struct Case {
    const char* map;
    const char* says;
  };

  for (const Case& c : {
           Case{"{w: 1}", "'planner.segment.mpc.cost.angular_momentum.w' is not a key"},
           Case{"{w_pre: -1}", "'planner.segment.mpc.cost.angular_momentum.w_pre'"},
           Case{"{w_post: 2.0e4}", "'planner.segment.mpc.cost.angular_momentum.w_post'"},
           Case{"{scale: 0}", "'planner.segment.mpc.cost.angular_momentum.scale'"},
           Case{"{scale: -10}", "'planner.segment.mpc.cost.angular_momentum.scale'"},
           Case{"3", "'planner.segment.mpc.cost.angular_momentum' must be a map"},
       }) {
    SCOPED_TRACE(c.map);
    const std::string what = ParseError(
        std::string("planner: {segment: {mpc: {cost: {angular_momentum: ") + c.map + "}}}}");
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

// ── The groups a plan leaves free ─────────────────────────────────────────────

TEST(JointGroupsLeftFree, NamesTheFirstGroupWithAJointOutsideThePlanAndNoRestWeight) {
  const std::vector<JointGroupSpec> specs = {
      {"a", {"j0", "j1", "j2"}}, {"b", {"j3", "j4"}}, {"c", {"j5", "j6"}}};
  JointGroups g;
  std::string why;
  ASSERT_TRUE(ResolveJointGroups(specs, kDevice, g, &why)) << why;
  using rtc::catching::FirstGroupLeftFree;
  using rtc::catching::JointSet;
  const JointSet plan = 0b0011111;  // j0..j4: the plan leaves j5, j6 (group c) out
  const std::array<double, 3> held = {0.0, 0.0, 1.0};
  const std::array<double, 3> c_free = {1.0, 1.0, 0.0};
  EXPECT_EQ(FirstGroupLeftFree(g, 7, plan, held), -1) << "c is held";
  EXPECT_EQ(FirstGroupLeftFree(g, 7, plan, c_free), 2);
  // A plan of every joint leaves nothing out, whatever the weights.
  EXPECT_EQ(FirstGroupLeftFree(g, 7, 0b1111111, c_free), -1);
  // A joint of b out too, and b without its weight: b is named first (device
  // order); with its weight, c is.
  const std::array<double, 3> b_c_free = {1.0, 0.0, 0.0};
  EXPECT_EQ(FirstGroupLeftFree(g, 7, 0b0001111, b_c_free), 1);
  EXPECT_EQ(FirstGroupLeftFree(g, 7, 0b0001111, c_free), 2);
  // No groups declared: one group without any rest weight.
  const std::array<double, 1> none = {5.0};
  EXPECT_EQ(FirstGroupLeftFree(JointGroups{}, 7, plan, none), 0);
  EXPECT_EQ(FirstGroupLeftFree(JointGroups{}, 7, 0b1111111, none), -1);
}

// The refusals a configure (and the offline batch) makes of a plan that leaves a group free: the
// plan holds trunk and one arm (group a, b), the other arm (c) is left out.
struct LeftFreeRig {
  std::vector<JointGroupSpec> specs = {
      {"a", {"j0", "j1", "j2"}}, {"b", {"j3", "j4"}}, {"c", {"j5", "j6"}}};
  JointGroups groups;
  rtc::catching::JointSet plan = 0b0011111;
};

LeftFreeRig MakeLeftFreeRig() {
  LeftFreeRig r;
  std::string why;
  EXPECT_TRUE(ResolveJointGroups(r.specs, kDevice, r.groups, &why)) << why;
  return r;
}

TEST(JointGroupsLeftFree, TheMpcRefusalNamesTheLeftOutGroupWhileTheAngularTermIsOn) {
  const LeftFreeRig r = MakeLeftFreeRig();
  rtc::catching::PlannerParams p;
  p.groups = r.specs;
  using rtc::catching::MpcSegmentLeftFreeRefusal;
  EXPECT_EQ(MpcSegmentLeftFreeRefusal(p, r.groups, 7, r.plan), "") << "the term is off";
  p.mpc_segment.ang_w_post = 1.0;
  const std::string why = MpcSegmentLeftFreeRefusal(p, r.groups, 7, r.plan);
  EXPECT_NE(why.find("group 'c' with no rest weight"), std::string::npos) << why;
  p.mpc_segment.groups[2].w_rest = 1.0;
  EXPECT_EQ(MpcSegmentLeftFreeRefusal(p, r.groups, 7, r.plan), "");
  EXPECT_NE(
      MpcSegmentLeftFreeRefusal(p, JointGroups{}, 7, r.plan).find("the arm (no planner.groups)"),
      std::string::npos)
      << "no groups: no rest weight exists";
  EXPECT_EQ(MpcSegmentLeftFreeRefusal(p, JointGroups{}, 7, 0b1111111), "") << "nothing left out";
}

TEST(JointGroupsLeftFree, TheDockingRefusalReadsEachRangesTermsAndAnyJointsTorqueCost) {
  const LeftFreeRig r = MakeLeftFreeRig();
  using rtc::catching::MpcDockingLeftFreeRefusal;
  rtc::catching::MpcDockingSegmentPlannerParams d;
  d.n_groups = 3;
  const auto refusal = [&] { return MpcDockingLeftFreeRefusal(d, r.specs, r.groups, 7, r.plan); };
  EXPECT_EQ(refusal(), "") << "no term reads the left-out arm";
  // A torque cost on the trunk alone reads the other arm's motion too (M, g):
  // the left-out group needs its pre-catch rest.
  d.groups[0].r_tau = 0.1;
  std::string why = refusal();
  EXPECT_NE(why.find("pre-catch stages"), std::string::npos) << why;
  EXPECT_NE(why.find("group 'c'"), std::string::npos) << why;
  d.groups[2].w_q_nom = 1.0;
  EXPECT_EQ(refusal(), "");
  // The core's uniform torque cost, the trunk's own switched off: the arm's
  // falls back on it.
  d.groups[0].r_tau = 0.0;
  d.groups[2].w_q_nom = std::numeric_limits<double>::quiet_NaN();
  d.core.r_tau = Eigen::VectorXd::Constant(7, 0.1);
  EXPECT_NE(refusal().find("pre-catch rest (w_q_nom)"), std::string::npos);
  d.core.w_q_nom = Eigen::VectorXd::Constant(7, 0.5);
  EXPECT_EQ(refusal(), "");
  // The angular term from the catch node on: the stop rest.
  d.core.w_ang_post = 1.0;
  why = refusal();
  EXPECT_NE(why.find("the angular-momentum term on the stop nodes"), std::string::npos) << why;
  EXPECT_NE(why.find("stop rest (w_rest_stop)"), std::string::npos) << why;
  d.groups[2].w_rest_stop = 2.0;
  EXPECT_EQ(refusal(), "");
  // Before the catch: the posture weight.
  d.core.w_q_nom.resize(0);
  d.core.r_tau.resize(0);
  d.core.w_ang_pre = 0.01;
  EXPECT_NE(refusal().find("pre-catch rest (w_q_nom)"), std::string::npos);
}

}  // namespace
