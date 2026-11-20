// estimator_replay — the inekf state source replayed offline from a run's logs
// (controllers/locomotion/estimator_replay.hpp).
//
//   ros2 run integrated_bringup estimator_replay --session <dir> --out <replay.csv>
//       [--profile g1_loco] [--config-dir <dir>] [--overlay <yaml>]... [--run <n>|all]
//       [--no-truth] [--joint-sigma <rad> [--joint-seed <n>]]
//
// <dir> holds locomotion_diag.csv and joint_ff.csv (a session's
// controllers/locomotion_controller/, or the session root above it);
// locomotion_truth.csv beside them, when present, adds the truth columns. A file
// that holds two sims (a later run appended to the same session folder) is
// replayed from its last one unless --run names another. The
// model and the estimator.inekf section are the profile's (installed share by
// default), each --overlay laid on the controller's section in order — a sim
// overlay file (integrated_rt_controller: ros__parameters: <key>: …) or the
// section itself — so an A/B runs one input through two configs. Writes the
// replay CSV (17 significant digits) and <out>.json (counts, extrinsic,
// contact names, Update() wall time — reported, not judged). --joint-sigma adds
// seeded white noise to the joint positions the source reads (the truth
// contact velocities keep the recorded ones; default 0 = the recorded values
// bit for bit), named in the sidecar — the sim records exact encoders.
#include "integrated_bringup/controllers/locomotion/estimator_replay.hpp"
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "integrated_bringup/controllers/locomotion/measured_state_source.hpp"
#include "integrated_bringup/support/profile_params.hpp"
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_urdf_bridge/free_flyer_root.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace loco = integrated_bringup::locomotion;

constexpr const char* kKey = "locomotion_controller";

struct Args {
  std::string session;
  std::string out;
  std::string profile{"g1_loco"};
  std::string config_dir;
  std::vector<std::string> overlays;
  bool truth{true};
  int run{loco::kLastRun};
  double joint_sigma{0.0};
  uint64_t joint_seed{1};
};

void Usage() {
  std::cerr
      << "usage: estimator_replay --session <dir> --out <replay.csv> [--profile <name>]\n"
         "                        [--config-dir <dir>] [--overlay <yaml>]... [--run <n>|all]\n"
         "                        [--no-truth] [--joint-sigma <rad> [--joint-seed <n>]]\n";
}

bool Parse(int argc, char** argv, Args& a) {
  bool seed_given = false;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    const auto value = [&](std::string& dst) {
      if (i + 1 >= argc) {
        return false;
      }
      dst = argv[++i];
      return true;
    };
    std::string v;
    if (k == "--session" && value(a.session)) {
    } else if (k == "--out" && value(a.out)) {
    } else if (k == "--profile" && value(a.profile)) {
    } else if (k == "--config-dir" && value(a.config_dir)) {
    } else if (k == "--overlay" && value(v)) {
      a.overlays.push_back(v);
    } else if (k == "--run" && value(v)) {
      if (v == "all") {
        a.run = loco::kAllRuns;
      } else {
        char* end = nullptr;
        const long n = std::strtol(v.c_str(), &end, 10);  // NOLINT(runtime/int)
        if (v.empty() || *end != '\0' || n < 0 || n > 1'000'000) {
          return false;
        }
        a.run = static_cast<int>(n);
      }
    } else if (k == "--no-truth") {
      a.truth = false;
    } else if (k == "--joint-sigma" && value(v)) {
      char* end = nullptr;
      a.joint_sigma = std::strtod(v.c_str(), &end);
      if (v.empty() || *end != '\0' || !std::isfinite(a.joint_sigma) || a.joint_sigma < 0.0) {
        return false;
      }
    } else if (k == "--joint-seed" && value(v)) {
      char* end = nullptr;
      errno = 0;
      const unsigned long long n = std::strtoull(v.c_str(), &end, 10);  // NOLINT(runtime/int)
      if (v.empty() || std::isdigit(static_cast<unsigned char>(v[0])) == 0 || *end != '\0' ||
          errno == ERANGE) {
        return false;
      }
      a.joint_seed = static_cast<uint64_t>(n);
      seed_given = true;
    } else {
      return false;
    }
  }
  // A seed without a sigma would name a noise that is not there.
  return !a.session.empty() && !a.out.empty() && !(seed_given && a.joint_sigma == 0.0);
}

std::string ShareFile(const std::string& package, const std::string& rel) {
  return ament_index_cpp::get_package_share_directory(package) + "/" + rel;
}

// The controller's section of an overlay file: a sim overlay's node block, a
// file keyed by the controller, or the section itself.
YAML::Node OverlaySection(const YAML::Node& file) {
  const YAML::Node node = file["integrated_rt_controller"]["ros__parameters"][kKey];
  if (node) {
    return node;
  }
  if (file[kKey]) {
    return file[kKey];
  }
  return file;
}

double Percentile(std::vector<double> x, double p) {
  if (x.empty()) {
    return 0.0;
  }
  std::sort(x.begin(), x.end());
  const auto at = static_cast<std::size_t>(p * static_cast<double>(x.size() - 1) + 0.5);
  return x[std::min(at, x.size() - 1)];
}

// A JSON string literal (quotes, backslashes and control characters escaped).
std::string Json(const std::string& s) {
  std::string out = "\"";
  for (const char ch : s) {
    if (ch == '"' || ch == '\\') {
      out += '\\';
      out += ch;
    } else if (static_cast<unsigned char>(ch) < 0x20) {
      std::array<char, 8> buf{};
      std::snprintf(buf.data(), buf.size(), "\\u%04x", static_cast<unsigned>(ch));
      out += buf.data();
    } else {
      out += ch;
    }
  }
  return out + '"';
}

void JsonStrings(std::ostream& os, const std::vector<std::string>& v) {
  os << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    os << (i == 0 ? "" : ", ") << Json(v[i]);
  }
  os << ']';
}

int Run(const Args& a) {
  // ── the logs ──────────────────────────────────────────────────────────────
  fs::path dir = a.session;
  if (!fs::exists(dir / "locomotion_diag.csv")) {
    dir = dir / "controllers" / kKey;
  }
  const fs::path diag = dir / "locomotion_diag.csv";
  const fs::path joint_ff = dir / "joint_ff.csv";
  const fs::path truth = dir / "locomotion_truth.csv";
  const bool with_truth = a.truth && fs::exists(truth);

  // ── the profile: model and estimator ──────────────────────────────────────
  const std::string config_dir =
      a.config_dir.empty() ? ShareFile("integrated_bringup", "config") : a.config_dir;
  const fs::path profile = fs::path(config_dir) / a.profile;
  std::vector<std::string> files;
  for (const char* f : {"_base.yaml", "sim.yaml"}) {
    if (fs::exists(profile / f)) {
      files.push_back((profile / f).string());
    }
  }
  const YAML::Node params = integrated_bringup::LoadProfileParams(files);
  const YAML::Node urdf = params["urdf"];
  if (!urdf || !urdf.IsMap()) {
    throw std::runtime_error(profile.string() + ": no urdf block");
  }
  const std::string pkg = urdf["package"].as<std::string>("robot_descriptions");
  const bool extended = urdf["extended"].as<bool>(false);
  const rtc_urdf_bridge::ModelConfig model_cfg = integrated_bringup::ModelConfigFromUrdfParams(
      urdf, ShareFile(pkg, urdf["path"].as<std::string>()),
      extended ? ShareFile(pkg, urdf["closure_path"].as<std::string>()) : "");
  rtc_urdf_bridge::PinocchioModelBuilder builder(model_cfg);
  auto full = builder.GetActuatedModel();
  if (!full) {
    full = builder.GetFullModel();
  }
  if (!full || !rtc_urdf_bridge::IsFreeFlyerRoot(*full)) {
    throw std::runtime_error("the profile's model has no free-flyer root");
  }

  YAML::Node ctrl = rtc::LoadControllerConfig(
      (profile / "controllers" / (std::string(kKey) + ".yaml")).string(), kKey);
  for (const auto& o : a.overlays) {
    integrated_bringup::MergeParamsInto(ctrl, OverlaySection(YAML::LoadFile(o)));
  }
  const loco::HumanoidModelConfig model = loco::LoadHumanoidModelConfig(ctrl["model"], "model");
  loco::ValidateAgainstModel(model, *full, "model");
  const loco::EstimatorConfig estimator =
      loco::ParseEstimatorConfig(ctrl["estimator"], "estimator");
  if (!estimator.inekf) {
    throw std::runtime_error("the controller config has no estimator.inekf section");
  }
  const auto body_tree = builder.GetTreeModel(model.body_group);
  if (!body_tree) {
    throw std::runtime_error("the model declares no tree model '" + model.body_group + "'");
  }
  const loco::BodyModel body = loco::BuildBodyModel(*full, *body_tree);

  loco::ReplaySetup setup;
  setup.params.body_max_age_ns = static_cast<int64_t>(estimator.body_max_age_ms * 1e6);
  setup.params.hand_max_age_ns = static_cast<int64_t>(estimator.hand_max_age_ms * 1e6);
  const auto& sites = model.imu.sites;
  setup.params.imu_site = static_cast<int>(
      std::find(sites.begin(), sites.end(), model.imu.primary_site) - sites.begin());
  setup.params.config = *estimator.inekf;
  setup.model = body.reduced;
  for (const auto& c : model.contacts) {
    setup.contact_names.push_back(c.name);
    setup.contact_frames.push_back(c.frame);
  }

  const loco::ReplayLog log =
      loco::LoadReplayLog(diag.string(), joint_ff.string(), with_truth ? truth.string() : "",
                          setup.contact_names, a.run);
  setup.roster = log.roster;
  setup.joint_noise_sigma = a.joint_sigma;
  setup.joint_noise_seed = a.joint_seed;
  loco::EstimatorReplay replay(setup);

  // ── replay ────────────────────────────────────────────────────────────────
  std::ofstream os(a.out);
  if (!os) {
    throw std::runtime_error(a.out + ": cannot write");
  }
  os << std::setprecision(17);
  loco::WriteReplayHeader(os, setup.contact_names);
  os << '\n';
  std::vector<double> update_ns;
  update_ns.reserve(log.ticks.size());
  std::size_t approx = 0;
  for (std::size_t i = 0; i < log.ticks.size(); ++i) {
    const loco::ReplayTruth* t = with_truth && log.truth[i].has_value() ? &*log.truth[i] : nullptr;
    const auto t0 = std::chrono::steady_clock::now();
    static_cast<void>(replay.Step(log.ticks[i], t));
    update_ns.push_back(static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0)
            .count()));
    approx += replay.last_joint_exact() ? 0 : 1;
    loco::WriteReplayRow(os, log.ticks[i], t, replay);
    os << '\n';
  }

  // ── the sidecar ───────────────────────────────────────────────────────────
  std::ofstream js(a.out + ".json");
  if (!js) {
    throw std::runtime_error(a.out + ".json: cannot write");
  }
  js << std::setprecision(17) << "{\n"
     << "  \"session\": " << Json(dir.string()) << ",\n"
     << "  \"profile\": " << Json(profile.string()) << ",\n"
     << "  \"overlays\": ";
  JsonStrings(js, a.overlays);
  js << ",\n  \"truth\": " << (with_truth ? "true" : "false") << ",\n"
     << "  \"joint_sigma\": " << a.joint_sigma << ",\n"
     << "  \"joint_seed\": " << a.joint_seed << ",\n"
     << "  \"rows\": " << log.ticks.size() << ",\n"
     << "  \"diag_rows\": " << log.diag_rows << ",\n"
     << "  \"unmatched_rows\": " << log.unmatched << ",\n"
     << "  \"gaps\": " << log.gaps << ",\n"
     << "  \"missing_ticks\": " << log.missing_ticks << ",\n"
     << "  \"runs\": " << log.runs << ",\n"
     << "  \"run\": " << log.run << ",\n"
     << "  \"approx_joint_ticks\": " << approx << ",\n"
     << "  \"sample_period_s\": " << setup.params.config.imu.sample_period_s << ",\n"
     << "  \"gravity\": " << setup.params.config.core.gravity << ",\n"
     << "  \"weight_n\": " << pinocchio::computeTotalMass(*full) * full->gravity.linear().norm()
     << ",\n"
     << "  \"contacts\": ";
  JsonStrings(js, setup.contact_names);
  js << ",\n  \"contact_frames\": ";
  JsonStrings(js, setup.contact_frames);
  js << ",\n  \"imu_frame\": " << Json(setup.params.config.imu_frame) << ",\n  \"R_BI\": [";
  for (int i = 0; i < 9; ++i) {
    js << (i == 0 ? "" : ", ") << replay.R_BI()(i / 3, i % 3);
  }
  js << "],\n  \"p_BI\": [" << replay.p_BI().x() << ", " << replay.p_BI().y() << ", "
     << replay.p_BI().z() << "],\n"
     << "  \"update_ns\": {\"p50\": " << Percentile(update_ns, 0.5)
     << ", \"p99\": " << Percentile(update_ns, 0.99) << ", \"max\": " << Percentile(update_ns, 1.0)
     << "}\n}\n";
  std::cout << a.out << ": " << log.ticks.size() << " ticks (" << log.unmatched
            << " diag rows without joint_ff, " << log.gaps << " gaps, " << approx
            << " approximate joint receipts)\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!Parse(argc, argv, a)) {
    Usage();
    return 2;
  }
  try {
    return Run(a);
  } catch (const std::exception& e) {
    std::cerr << "estimator_replay: " << e.what() << '\n';
    return 1;
  }
}
