// ── The TSID QP row budget on the shipped demo_wbc configs ───────────────────
//
// The formulation sizes its QP rows from the MaxEqDim / MaxIneqDim of the
// constraints the controller registers. Before that, the budget was a fixed
// formula that did not count joint_limit's rows, and an active constraint that
// did not fit was silently left out of the QP: on iiwa7_leap with all four
// fingertip contacts active the torque limits were dropped (82 rows used, 75
// held). Here the real controller on each shipped config solves one tick with
// every contact and every configured constraint active, and the rows it used
// are compared with a count taken from the YAML alone (the model dimensions
// come from the fixtures) — not from the Max*Dim under test.
//
// ur5e_p1a has no model fixture (its hand model is not in this workspace); its
// TSID block is held to ur5e_p1b's by the config comparison at the bottom:
// same constraint types and the same contact shapes give the same rows.
//
// Also here, on the same shipped block: the joint_limit box integrates over the
// control period (#1019) — a YAML dt that disagrees with 1/control_rate fails
// LoadConfig, a missing one takes the period.
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using integrated_bringup::DemoWbcController;
namespace fx = integrated_bringup::testfx;

// QP rows of one TSID block with every contact active, counted from the YAML.
struct ExpectedRows {
  int eq{0};
  int ineq{0};
  std::vector<std::string> constraint_names;  // rtc_tsid Name() of each entry
};

int ContactDim(const YAML::Node& c) {
  return c["type"].as<std::string>("point") == "surface" ? 6 : 3;
}

int FrictionFaces(const YAML::Node& c) {
  if (c["friction_faces"]) {
    return c["friction_faces"].as<int>();
  }
  return c["n_faces"].as<int>(4);
}

// nv = n_actuated: every shipped demo_wbc model is fixed-base.
ExpectedRows CountRows(const YAML::Node& tsid, int nv) {
  ExpectedRows out;
  int contact_vars = 0;
  int friction_rows = 0;
  for (const auto& c : tsid["contacts"]) {
    contact_vars += ContactDim(c);
    friction_rows += FrictionFaces(c) + 1 + (ContactDim(c) == 6 ? 6 : 0);
  }
  const std::map<std::string, std::string> kName = {{"eom", "eom"},
                                                    {"contact_constraint", "contact_accel"},
                                                    {"joint_limit", "joint_limit"},
                                                    {"friction_cone", "friction_cone"},
                                                    {"torque_limit", "torque_limit"}};
  for (auto it = tsid["constraints"].begin(); it != tsid["constraints"].end(); ++it) {
    const auto type = it->second["type"].as<std::string>();
    const auto name = kName.find(type);
    if (name == kName.end()) {
      ADD_FAILURE() << "constraint type '" << type << "' has no row count here — add it";
      continue;
    }
    out.constraint_names.push_back(name->second);
    if (type == "contact_constraint") {
      out.eq += contact_vars;
    } else if (type == "joint_limit" || type == "torque_limit") {
      out.ineq += nv;
    } else if (type == "friction_cone") {
      out.ineq += friction_rows;
    }  // eom: no rows on a fixed base
  }
  return out;
}

struct Profile {
  const char* name;
  const rtc_urdf_bridge::ModelConfig& model;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder;
  std::map<std::string, rtc::DeviceNameConfig> devices;
  rtc::ControllerState state;
  double dt;
  int nv;
};

void ExpectNoRowsDropped(Profile p) {
  SCOPED_TRACE(p.name);
  const YAML::Node cfg = fx::ShippedControllerNode(p.name, "demo_wbc_controller");
  ASSERT_TRUE(cfg && cfg["tsid"]);
  const ExpectedRows expected = CountRows(cfg["tsid"], p.nv);
  ASSERT_GT(expected.ineq, 0);

  auto ctrl = std::make_unique<DemoWbcController>("");
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(std::string("qp_row_budget_") + p.name);
  ctrl->SetSystemModelConfig(p.model);
  ctrl->SetSharedModelBuilder(p.builder);
  ctrl->SetControlRate(1.0 / p.dt);
  ctrl->LoadConfig(cfg);
  ctrl->SetDeviceNameConfigs(p.devices);
  ASSERT_EQ(ctrl->on_configure(rclcpp_lifecycle::State{}, node, cfg),
            DemoWbcController::CallbackReturn::SUCCESS);
  (void)ctrl->Compute(p.state);  // one tick: cache and reference are current

  const auto* result = ctrl->SolveTsidAllContactsActiveForTesting(expected.constraint_names);
  ASSERT_NE(result, nullptr) << "no TSID stack, or a constraint name is not registered";
  EXPECT_EQ(result->rows_dropped, 0);
  EXPECT_NE(result->status, rtc::tsid::kRowBudgetExceeded);
  EXPECT_EQ(result->n_ineq_used, expected.ineq);
  EXPECT_EQ(result->n_eq_used, expected.eq);
  ::testing::Test::RecordProperty(std::string(p.name) + "_ineq_rows",
                                  std::to_string(result->n_ineq_used));
}

TEST(DemoWbcQpRowBudget, Iiwa7LeapHoldsEveryRowWithAllContactsActive) {
  ExpectNoRowsDropped({"iiwa7_leap", fx::SharedIiwa7LeapModelConfig(), fx::SharedIiwa7LeapBuilder(),
                       fx::MakeIiwa7LeapDeviceConfigs(), fx::MakeIiwa7LeapState(), fx::kDt,
                       fx::kArmDof + fx::kHandDof});
}

TEST(DemoWbcQpRowBudget, Ur5eP1bHoldsEveryRowWithAllContactsActive) {
  ExpectNoRowsDropped({"ur5e_p1b", fx::SharedUr5eP1bModelConfig(), fx::SharedUr5eP1bBuilder(),
                       fx::MakeUr5eP1bDeviceConfigs(), fx::MakeUr5eP1bState(), fx::kUr5eDt,
                       fx::kUr5eArmDof + fx::kP1bHandDof});
}

// ── The YAML dispatch fingerprint (WD-58) ────────────────────────────────────
//
// What the tsid.tasks / tsid.constraints dispatch builds on a shipped config,
// in YAML order — each entry's key, type, resolved Name, priority, weight,
// active flag and row counts — plus an FNV-1a digest of five ticks of the
// all-contacts solve's x_opt. Pinned before the dispatch moved into rtc_tsid
// (task_factory) and unchanged by the move: the move is a refactor.

std::uint64_t Fnv1a(std::uint64_t h, const Eigen::VectorXd& v) {
  const auto* bytes = reinterpret_cast<const unsigned char*>(v.data());
  for (std::size_t i = 0; i < static_cast<std::size_t>(v.size()) * sizeof(double); ++i) {
    h ^= bytes[i];
    h *= 1099511628211ULL;
  }
  return h;
}

std::string DispatchFingerprint(Profile p) {
  const YAML::Node cfg = fx::ShippedControllerNode(p.name, "demo_wbc_controller");
  auto ctrl = std::make_unique<DemoWbcController>("");
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(std::string("tsid_dispatch_") + p.name);
  ctrl->SetSystemModelConfig(p.model);
  ctrl->SetSharedModelBuilder(p.builder);
  ctrl->SetControlRate(1.0 / p.dt);
  ctrl->LoadConfig(cfg);
  ctrl->SetDeviceNameConfigs(p.devices);
  if (ctrl->on_configure(rclcpp_lifecycle::State{}, node, cfg) !=
      DemoWbcController::CallbackReturn::SUCCESS) {
    ADD_FAILURE() << "configure failed";
    return {};
  }
  (void)ctrl->Compute(p.state);
  auto& f = ctrl->TsidFormulationForTesting();
  const auto& contacts = ctrl->TsidContactStateForTesting();
  std::ostringstream out;
  out.precision(17);
  const YAML::Node tsid = cfg["tsid"];
  for (auto it = tsid["tasks"].begin(); it != tsid["tasks"].end(); ++it) {
    const std::string key = it->first.as<std::string>();
    const YAML::Node entry = it->second;
    const rtc::tsid::TaskBase* t = f.GetTask(key);
    out << "task " << key << " " << entry["type"].as<std::string>("") << " ";
    if (t == nullptr) {
      out << "null\n";
      continue;
    }
    out << t->Name() << " p" << t->Priority() << " w" << t->Weight() << " a" << t->IsActive()
        << " r" << t->ResidualDim() << "\n";
  }
  const ExpectedRows rows = CountRows(tsid, p.nv);
  for (const auto& name : rows.constraint_names) {
    const rtc::tsid::ConstraintBase* c = f.GetConstraint(name);
    out << "constraint " << name << " ";
    if (c == nullptr) {
      out << "null\n";
      continue;
    }
    out << "a" << c->IsActive() << " eq" << c->EqDim(contacts) << " in" << c->IneqDim(contacts)
        << "\n";
  }
  std::uint64_t digest = 14695981039346656037ULL;
  for (int tick = 0; tick < 5; ++tick) {
    (void)ctrl->Compute(p.state);
    const auto* result = ctrl->SolveTsidAllContactsActiveForTesting(rows.constraint_names);
    if (result == nullptr) {
      ADD_FAILURE() << "no TSID stack";
      return {};
    }
    digest = Fnv1a(digest, result->x_opt);
  }
  char hex[32];
  std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(digest));
  out << "digest " << hex << "\n";
  return out.str();
}

// Recorded on main before the move (2026-11-12, this workspace's build).
const char* const kIiwa7LeapFingerprint =
    "task posture posture posture p0 w1 a1 r23\n"
    "task se3_tcp se3 se3_tcp p0 w50 a0 r6\n"
    "task force force force p1 w0 a0 r0\n"
    "task contact_consistency contact_consistency contact_consistency p1 w0 a0 r0\n"
    "task object_wrench object_wrench object_wrench p1 w0 a0 r0\n"
    "task internal_force internal_force internal_force p1 w0 a0 r0\n"
    "task object_se3 object_se3 object_se3 p0 w0 a0 r0\n"
    "constraint eom a1 eq0 in0\n"
    "constraint joint_limit a1 eq0 in23\n"
    "constraint friction_cone a0 eq0 in0\n"
    "constraint torque_limit a1 eq0 in23\n"
    "digest fdf3aabec5c58e85\n";
const char* const kUr5eP1bFingerprint =
    "task posture posture posture p0 w1 a1 r16\n"
    "task se3_tcp se3 se3_tcp p0 w50 a1 r6\n"
    "task force force force p1 w0 a0 r0\n"
    "task contact_consistency contact_consistency contact_consistency p1 w0 a0 r0\n"
    "task object_wrench object_wrench object_wrench p1 w0 a0 r0\n"
    "task internal_force internal_force internal_force p1 w0 a0 r0\n"
    "task object_se3 object_se3 object_se3 p0 w0 a0 r0\n"
    "constraint eom a1 eq0 in0\n"
    "constraint joint_limit a1 eq0 in16\n"
    "constraint friction_cone a0 eq0 in0\n"
    "constraint torque_limit a1 eq0 in16\n"
    "digest 5f31a08fb80ff1bd\n";

TEST(DemoWbcTsidDispatch, TheShippedConfigsBuildTheSameStack) {
  for (Profile p : {Profile{"iiwa7_leap", fx::SharedIiwa7LeapModelConfig(),
                            fx::SharedIiwa7LeapBuilder(), fx::MakeIiwa7LeapDeviceConfigs(),
                            fx::MakeIiwa7LeapState(), fx::kDt, fx::kArmDof + fx::kHandDof},
                    Profile{"ur5e_p1b", fx::SharedUr5eP1bModelConfig(), fx::SharedUr5eP1bBuilder(),
                            fx::MakeUr5eP1bDeviceConfigs(), fx::MakeUr5eP1bState(), fx::kUr5eDt,
                            fx::kUr5eArmDof + fx::kP1bHandDof}}) {
    SCOPED_TRACE(p.name);
    const std::string fp = DispatchFingerprint(p);
    std::printf("dispatch fingerprint %s:\n%s", p.name, fp.c_str());
    ::testing::Test::RecordProperty(std::string(p.name) + "_dispatch", fp);
    EXPECT_EQ(fp,
              std::string(p.name) == "iiwa7_leap" ? kIiwa7LeapFingerprint : kUr5eP1bFingerprint);
  }
}

// Σ joint_state_names over the devices of a profile's _base.yaml.
int ProfileJointCount(const std::string& profile) {
  const YAML::Node root =
      YAML::LoadFile(std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/" + profile + "/_base.yaml");
  const YAML::Node devices = root["/**"]["ros__parameters"]["devices"];
  int n = 0;
  for (auto it = devices.begin(); it != devices.end(); ++it) {
    if (it->second["joint_state_names"]) {
      n += static_cast<int>(it->second["joint_state_names"].size());
    }
  }
  return n;
}

// ur5e_p1a: same rows as ur5e_p1b by construction (see the header comment).
TEST(DemoWbcQpRowBudget, Ur5eP1aTsidRowsMatchUr5eP1b) {
  const YAML::Node a = fx::ShippedControllerNode("ur5e_p1a", "demo_wbc_controller")["tsid"];
  const YAML::Node b = fx::ShippedControllerNode("ur5e_p1b", "demo_wbc_controller")["tsid"];
  ASSERT_TRUE(a && b);
  const int nv = fx::kUr5eArmDof + fx::kP1bHandDof;
  const ExpectedRows ra = CountRows(a, nv);
  const ExpectedRows rb = CountRows(b, nv);
  EXPECT_EQ(ra.constraint_names, rb.constraint_names);
  EXPECT_EQ(ra.eq, rb.eq);
  EXPECT_EQ(ra.ineq, rb.ineq);
  ASSERT_EQ(a["contacts"].size(), b["contacts"].size());
  // The nv the count assumes: the joints both profiles' devices declare.
  EXPECT_EQ(ProfileJointCount("ur5e_p1a"), nv);
  EXPECT_EQ(ProfileJointCount("ur5e_p1b"), nv);
}

// ── joint_limit dt against the control rate (#1019) ──────────────────────────
// LoadConfig builds the TSID stack (it needs the model, the builder and the
// rate, as on_configure's caller sets them first).
void LoadUr5eP1b(DemoWbcController& ctrl, const YAML::Node& cfg, double rate_hz) {
  ctrl.SetSystemModelConfig(fx::SharedUr5eP1bModelConfig());
  ctrl.SetSharedModelBuilder(fx::SharedUr5eP1bBuilder());
  ctrl.SetControlRate(rate_hz);
  ctrl.LoadConfig(cfg);
}

TEST(DemoWbcJointLimitDt, AYamlDtThatDisagreesWithTheControlRateFailsTheConfigure) {
  YAML::Node cfg = YAML::Clone(fx::ShippedControllerNode("ur5e_p1b", "demo_wbc_controller"));
  cfg["tsid"]["constraints"]["joint_limit"]["dt"] = fx::kUr5eDt;
  DemoWbcController at_rate("");
  ASSERT_NO_THROW(LoadUr5eP1b(at_rate, cfg, 1.0 / fx::kUr5eDt));
  EXPECT_DOUBLE_EQ(at_rate.TsidJointLimitDtForTesting(), fx::kUr5eDt);
  DemoWbcController twice_as_fast("");
  EXPECT_THROW(LoadUr5eP1b(twice_as_fast, cfg, 2.0 / fx::kUr5eDt), std::invalid_argument);
}

// The shipped configs give no dt; a blank `dt:` (null) means the same.
TEST(DemoWbcJointLimitDt, AMissingDtTakesTheControlPeriod) {
  YAML::Node cfg = YAML::Clone(fx::ShippedControllerNode("ur5e_p1b", "demo_wbc_controller"));
  ASSERT_FALSE(cfg["tsid"]["constraints"]["joint_limit"]["dt"]) << "premise: shipped without dt";
  DemoWbcController ctrl("");
  ASSERT_NO_THROW(LoadUr5eP1b(ctrl, cfg, 1000.0));
  EXPECT_DOUBLE_EQ(ctrl.TsidJointLimitDtForTesting(), 1e-3);

  cfg["tsid"]["constraints"]["joint_limit"]["dt"] = YAML::Node(YAML::NodeType::Null);
  DemoWbcController blank("");
  ASSERT_NO_THROW(LoadUr5eP1b(blank, cfg, 1000.0));
  EXPECT_DOUBLE_EQ(blank.TsidJointLimitDtForTesting(), 1e-3);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
