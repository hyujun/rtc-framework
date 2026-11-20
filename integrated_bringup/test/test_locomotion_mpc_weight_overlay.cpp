// locomotion_controller.mpc.kinodynamic weights through a sim overlay (g1_loco): the overlay
// files tools weight_overlay (rtc_tools) writes — fixtures in test/data/weight_overlay/, kept equal
// to the generator's output by rtc_tools test_weight_overlay.py — are loaded as a ROS params file,
// their parameters are written over the controller's YAML tree the way
// ApplyControllerParamOverrides does, and the tree goes through rtc_mpc's kinodynamic OCP config
// loader. Two claims:
//   - an overlay that writes EVERY weight leaf at its theta = 0 value (the shipped number, as a
//     float) yields a config equal, bit for bit, to the shipped YAML's, for every weight vector;
//   - an overlay with one group at theta != 0 yields shipped × 10^theta on exactly that group's
//     entries and the shipped value everywhere else.
//
// WHAT IS NOT COVERED. ApplyControllerParamOverrides itself (rtc_controller_manager
// src/rt_controller_node_params.cpp) sits in an anonymous namespace there and no header declares
// it, so this test cannot call it; ApplyOverlay() below repeats its pass 2 (list_parameters under
// `<controller>.`, dotted path split, scalar / double-array write). The ROS side is real — the
// params-file parse and typing (a YAML float is a double, a list of floats a double array), the
// parameter names — and so is the loader; a change to the real function's pass 2 or its refusal
// pass (CheckOverridePaths) is NOT seen here. No ROS node is spun up beyond a parameter holder.
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "shipped_catching_bringup_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
namespace loco = integrated_bringup::locomotion;

constexpr const char* kProfile = "g1_loco";
constexpr const char* kController = "locomotion_controller";   // the overlay's config key
constexpr const char* kNodeName = "integrated_rt_controller";  // the overlay's node
constexpr const char* kFixtureDir = RTC_WEIGHT_OVERLAY_FIXTURE_DIR;

YAML::Node InstalledLocomotionConfig() {
  const std::string main = ament_index_cpp::get_package_share_directory("integrated_bringup") +
                           "/config/" + kProfile + "/controllers/locomotion_controller.yaml";
  return rtc::LoadControllerConfig(main, kController);
}

bool BitEqual(const Eigen::Ref<const Eigen::VectorXd>& a,
              const Eigen::Ref<const Eigen::VectorXd>& b) {
  return a.size() == b.size() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

std::vector<std::string> SplitDots(const std::string& s) {
  std::vector<std::string> out;
  std::size_t start = 0;
  for (std::size_t pos = s.find('.'); pos != std::string::npos; pos = s.find('.', start)) {
    if (pos > start) {
      out.push_back(s.substr(start, pos - start));
    }
    start = pos + 1;
  }
  if (start < s.size()) {
    out.push_back(s.substr(start));
  }
  return out;
}

// The overlay file as the ROS parameters its node would receive, written into @p root (the
// controller's YAML tree) as ApplyControllerParamOverrides's pass 2 does: `.`-separated name
// below `<controller>.`, walked with Node::reset (a value copy through operator= would splice
// the child over the root), double / double-array leaves. Returns the number of leaves written;
// any other parameter type is a failure here (the weight overlay writes only those two).
std::size_t ApplyOverlay(YAML::Node root, const std::string& overlay_path) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  options.arguments({"--ros-args", "--params-file", overlay_path});
  options.automatically_declare_parameters_from_overrides(true);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  std::size_t written = 0;
  {
    rclcpp::Node node(kNodeName, options);

    const std::string prefix = std::string(kController) + ".";
    const auto listed = node.list_parameters({kController}, 20);
    for (const auto& pname : listed.names) {
      if (pname.rfind(prefix, 0) != 0) {
        continue;
      }
      const auto parts = SplitDots(pname.substr(prefix.size()));
      EXPECT_FALSE(parts.empty());
      YAML::Node parent(root);
      for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
        parent.reset(parent[parts[i]]);
      }
      const rclcpp::Parameter p = node.get_parameter(pname);
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        parent[parts.back()] = p.as_double();
      } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        YAML::Node arr(YAML::NodeType::Sequence);
        for (double v : p.as_double_array()) {
          arr.push_back(v);
        }
        parent[parts.back()] = arr;
      } else {
        ADD_FAILURE() << pname << " is not a double or a double array: the YAML scalar type of the "
                      << "overlay is wrong (an int where the controller reads a double)";
        continue;
      }
      ++written;
    }
  }
  context->shutdown("test done");
  return written;
}

class LocomotionMpcWeightOverlay : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = InstalledLocomotionConfig();
    ASSERT_TRUE(root_["mpc"]["kinodynamic"].IsMap());
    // The MPC model: the CM's floating model with every joint outside the body roster (the hand)
    // locked at its neutral value — as test_locomotion_mpc_config.cpp builds it.
    const auto full = fx::ShippedModelBuilder(kProfile)->GetFullModel();
    ASSERT_NE(full, nullptr);
    const YAML::Node roster = fx::ShippedSimParams(kProfile)["devices"]["g1"]["joint_state_names"];
    ASSERT_TRUE(roster.IsSequence());
    std::vector<std::string> body;
    for (const auto& j : roster) {
      body.push_back(j.as<std::string>());
    }
    std::vector<pinocchio::JointIndex> lock;
    for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(full->njoints); ++j) {
      if (std::find(body.begin(), body.end(), full->names[j]) == body.end()) {
        lock.push_back(j);
      }
    }
    model_ = pinocchio::buildReducedModel(*full, lock, pinocchio::neutral(*full));
    const loco::HumanoidModelConfig cfg = loco::LoadHumanoidModelConfig(root_["model"], kProfile);
    ASSERT_EQ(handler_.Init(model_, loco::ToRobotModelHandlerYaml(cfg)),
              rtc::mpc::RobotModelInitError::kNoError);
  }

  rtc::mpc::KinodynamicOCPConfig Load(const YAML::Node& tree) const {
    rtc::mpc::KinodynamicOCPConfig cfg;
    EXPECT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(tree["mpc"]["kinodynamic"], handler_, cfg),
              rtc::mpc::KinodynamicOCPConfigError::kNoError);
    return cfg;
  }

  YAML::Node root_;
  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
};

// Every weight vector of two loaded configs, bit for bit.
void ExpectSameWeights(const rtc::mpc::KinodynamicOCPConfig& got,
                       const rtc::mpc::KinodynamicOCPConfig& want) {
  EXPECT_TRUE(BitEqual(got.state_weights, want.state_weights));
  EXPECT_TRUE(BitEqual(got.terminal_weights, want.terminal_weights));
  EXPECT_TRUE(BitEqual(got.input_weights, want.input_weights));
  EXPECT_TRUE(BitEqual(got.contact_tracking_weights, want.contact_tracking_weights));
  EXPECT_EQ(got.terminal_scaling, want.terminal_scaling);
}

// Writing every weight leaf at its theta = 0 value changes nothing.
TEST_F(LocomotionMpcWeightOverlay, AnOverlayOfEveryKeyAtThetaZeroLoadsTheShippedConfigBitForBit) {
  const rtc::mpc::KinodynamicOCPConfig shipped = Load(root_);
  YAML::Node tree = YAML::Clone(root_);
  const std::size_t written =
      ApplyOverlay(tree, std::string(kFixtureDir) + "/all_keys_theta0.yaml");
  // state and terminal: 4 lists each (base position, orientation, linear and angular velocity)
  // and 2 × 23 joint leaves each; input: 2 lists + 23 joint leaves; contact_tracking: 6 lists
  EXPECT_EQ(written, 2U * (4U + 2U * 23U) + 2U + 23U + 6U);
  ExpectSameWeights(Load(tree), shipped);
}

struct ScaledGroup {
  const char* fixture;
  double theta;
  // key paths under mpc.kinodynamic: a list is scaled element-wise, a map value-wise
  std::vector<std::vector<std::string>> paths;
};

// One group at theta != 0: its entries are shipped × 10^theta, every other entry the shipped value.
// The expected tree scales the group's entries in the YAML with std::pow(10.0, theta); the
// generator computes 10.0 ** theta in Python, which calls the same libm pow but is not promised to
// agree to the last bit, so the scaled entries are compared to a relative 1e-15 (a few ulp) and
// the entries that are NOT scaled bit for bit.
TEST_F(LocomotionMpcWeightOverlay, OneGroupAtThetaScalesExactlyThatGroup) {
  const std::vector<ScaledGroup> cases = {
      {"joint_position_m0p5.yaml",
       -0.5,
       {{"state_weights", "joints"}, {"terminal_weights", "joints"}}},
      {"base_orientation_p0p5.yaml",
       0.5,
       {{"state_weights", "base_orientation"}, {"terminal_weights", "base_orientation"}}},
  };
  const rtc::mpc::KinodynamicOCPConfig shipped = Load(root_);
  for (const ScaledGroup& c : cases) {
    SCOPED_TRACE(c.fixture);
    YAML::Node got_tree = YAML::Clone(root_);
    EXPECT_GT(ApplyOverlay(got_tree, std::string(kFixtureDir) + "/" + c.fixture), 0U);
    const rtc::mpc::KinodynamicOCPConfig got = Load(got_tree);

    YAML::Node want_tree = YAML::Clone(root_);
    const double scale = std::pow(10.0, c.theta);
    for (const auto& path : c.paths) {
      YAML::Node n = want_tree["mpc"]["kinodynamic"];
      for (const auto& key : path) {
        n.reset(n[key]);
      }
      if (n.IsSequence()) {
        for (std::size_t i = 0; i < n.size(); ++i) {
          n[i] = n[i].as<double>() * scale;
        }
      } else {
        for (auto it = n.begin(); it != n.end(); ++it) {
          it->second = it->second.as<double>() * scale;
        }
      }
    }
    const rtc::mpc::KinodynamicOCPConfig want = Load(want_tree);

    // what the group does not own is the shipped value, bit for bit
    EXPECT_TRUE(BitEqual(got.input_weights, shipped.input_weights));
    EXPECT_TRUE(BitEqual(got.contact_tracking_weights, shipped.contact_tracking_weights));
    EXPECT_EQ(got.terminal_scaling, shipped.terminal_scaling);
    // what it owns moved, and equals shipped × 10^theta
    EXPECT_FALSE(BitEqual(got.state_weights, shipped.state_weights));
    EXPECT_FALSE(BitEqual(got.terminal_weights, shipped.terminal_weights));
    ASSERT_EQ(got.state_weights.size(), want.state_weights.size());
    for (Eigen::Index i = 0; i < got.state_weights.size(); ++i) {
      const auto k = static_cast<std::size_t>(i);
      const double w = want.state_weights[i];
      const double t = want.terminal_weights[i];
      if (w == shipped.state_weights[i]) {
        EXPECT_EQ(got.state_weights[i], w) << "state entry " << k << " is not in the group";
      } else {
        EXPECT_NEAR(got.state_weights[i], w, std::abs(w) * 1e-15) << "state entry " << k;
      }
      if (t == shipped.terminal_weights[i]) {
        EXPECT_EQ(got.terminal_weights[i], t) << "terminal entry " << k << " is not in the group";
      } else {
        EXPECT_NEAR(got.terminal_weights[i], t, std::abs(t) * 1e-15) << "terminal entry " << k;
      }
    }
  }
}

}  // namespace
