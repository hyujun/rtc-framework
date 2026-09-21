// ── `catch_pose_ik_batch` — offline catch-pose judge for the S3.5a map ──────
//
// Reads a candidate list, runs `rtc::catching::CatchPoseIk::Solve` on each with
// the tuning `ParseCatchPoseIkParams` produces, and writes one CSV row per
// candidate. The python side (`rtc_tools.analysis.catchability_map`) owns the
// throw grid, the ballistic flight and the aggregation; the verdict is this
// binary's, so the map and the runtime planner share one judgement (S1.9).
//
// ARCH-7-exempt: an offline inspection tool, in the sense design-principles.md
// §"ARCH-7 의 범위" gives that category — it knows no robot (every model, frame
// and option comes from argv), owns no RT loop and no ROS node, and appears in
// no launch file or bringup chain. It is the development tool that generates a
// plan §11 map and, later, the S6.2 equivalence oracle; it is never part of a
// running system.
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/math/rpy.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;

constexpr std::string_view kUsage =
    R"(catch_pose_ik_batch — offline catch-pose judge (dynamic_catching S3.5a, plan §11)

  --model-config PATH   ModelConfig YAML (PinocchioModelBuilder::LoadModelConfig
                        schema, NOT the ros__parameters robot config). Must
                        declare the catch frame under `extra_frames`.
  --sub-model NAME      arm sub-model to judge in; omit to use the full model
  --catch-frame NAME    frame name (default: catch_frame)
  --params PATH         YAML holding the `catching:` tree (planner.ik.*,
                        planner.catchability.*). Omit to use in-code defaults.
                        Accepted shapes: a top-level `catching:` map; a shipped
                        controller config (`<controller>: {catching: ...}`); or
                        the tree itself (top-level `planner:`). Anything else
                        is an error, never a silent default.
  --print-options       print the options --params resolves to (and where in
                        the file the tree was found) and exit; needs no model
  --candidates PATH     candidate CSV: id[,seed_id],p_c_x,p_c_y,p_c_z,v_x,v_y,v_z
                        in MODEL WORLD coordinates. '-' reads stdin.
  --seeds PATH          seed CSV: seed_id,q0,... (one row per wait-pose candidate)
  --out PATH            output CSV (default: stdout)
  --dump-frame          print the catch frame placement relative to its parent
                        and exit — the S2.3b check through the production builder
  -h, --help            this text
)";

[[noreturn]] void Die(const std::string& msg) {
  std::cerr << "catch_pose_ik_batch: " << msg << '\n';
  std::exit(2);
}

struct Args {
  std::string model_config;
  std::string sub_model;
  std::string catch_frame{"catch_frame"};
  std::string params;
  std::string candidates;
  std::string seeds;
  std::string out;
  bool dump_frame{false};
  bool print_options{false};
};

[[nodiscard]] Args ParseArgs(int argc, char** argv) {
  Args a;
  const auto value = [&](int& i, std::string_view flag) {
    if (i + 1 >= argc) {
      Die(std::string(flag) + " needs a value");
    }
    return std::string(argv[++i]);
  };
  for (int i = 1; i < argc; ++i) {
    const std::string_view f = argv[i];
    if (f == "-h" || f == "--help") {
      std::cout << kUsage;
      std::exit(0);
    } else if (f == "--model-config") {
      a.model_config = value(i, f);
    } else if (f == "--sub-model") {
      a.sub_model = value(i, f);
    } else if (f == "--catch-frame") {
      a.catch_frame = value(i, f);
    } else if (f == "--params") {
      a.params = value(i, f);
    } else if (f == "--candidates") {
      a.candidates = value(i, f);
    } else if (f == "--seeds") {
      a.seeds = value(i, f);
    } else if (f == "--out") {
      a.out = value(i, f);
    } else if (f == "--dump-frame") {
      a.dump_frame = true;
    } else if (f == "--print-options") {
      a.print_options = true;
    } else {
      Die("unknown argument '" + std::string(f) + "' (try --help)");
    }
  }
  if (a.model_config.empty() && !a.print_options) {
    Die("--model-config is required unless --print-options (try --help)");
  }
  return a;
}

/// The options `--params` resolves to; the in-code defaults when it is absent.
/// Everything that decides WHICH tree is read lives in the library
/// (`ResolveCatchingTree`), so that it is a tested function and not a guess
/// made here. `tree_path` receives where the tree was found.
[[nodiscard]] rtc::catching::CatchPoseIkOptions LoadOptions(const std::string& params_path,
                                                            std::string& tree_path) {
  tree_path = "<none: in-code defaults>";
  if (params_path.empty()) {
    return {};
  }
  const rtc::catching::CatchingTree tree =
      rtc::catching::ResolveCatchingTree(YAML::LoadFile(params_path), params_path);
  tree_path = tree.path;
  rtc::catching::CatchPoseIkRetiredKeys retired{};
  const auto parsed = rtc::catching::ParseCatchPoseIkParams(tree.node, &retired);
  if (retired.lambda || retired.manip_min) {
    std::cerr << "catch_pose_ik_batch: note — the params file carries retired planner.ik keys"
              << '\n';
  }
  // An absent `planner` section is the parser's documented all-defaults case,
  // and legitimate (the S4.0 controller configs ship without one) — but it is
  // also what a wrongly-shaped file used to look like, so it is said out loud.
  const YAML::Node& node = tree.node;
  if (!node["planner"]) {
    std::cerr << "catch_pose_ik_batch: note — '" << params_path << "' (" << tree.path
              << ") has no `planner` section; every option is the in-code default\n";
  }
  // A TBD threshold leaves manipulability_min non-finite on purpose, which
  // Solve reports as kOptionsInvalid for every candidate. Say so once here
  // rather than letting the map come out uniformly empty.
  if (rtc::catching::ActiveManipulabilityMin(parsed).tbd) {
    std::cerr << "catch_pose_ik_batch: the active manipulability_min row is TBD — every "
                 "candidate will be rejected as options_invalid\n";
  }
  return parsed.options;
}

/// `GetReducedModel` throws `std::out_of_range` for an unknown name (it never
/// returns null); the translation here only makes the message name the flag.
[[nodiscard]] std::shared_ptr<const pinocchio::Model> PickModel(const rub::PinocchioModelBuilder& b,
                                                                const std::string& sub_model) {
  if (sub_model.empty()) {
    return b.GetFullModel();
  }
  try {
    return b.GetReducedModel(sub_model);
  } catch (const std::out_of_range&) {
    Die("--sub-model '" + sub_model + "' is not a sub-model of the model config");
  }
}

/// The S2.3b check, through the production builder: does the frame the model
/// actually carries sit where the config says it should, relative to the parent
/// the config names? Prints the residual and fails non-zero if it is not
/// numerically zero, so a mis-declared offset cannot pass quietly.
int DumpFrame(const pinocchio::Model& model, const rub::ModelConfig& cfg,
              const std::string& frame_name) {
  if (!model.existFrame(frame_name)) {
    Die("model has no frame '" + frame_name + "'");
  }
  const auto declared =
      std::find_if(cfg.extra_frames.begin(), cfg.extra_frames.end(),
                   [&](const rub::ExtraFrameConfig& e) { return e.name == frame_name; });
  if (declared == cfg.extra_frames.end()) {
    Die("'" + frame_name + "' is not declared under extra_frames in the model config");
  }
  if (!model.existFrame(declared->parent)) {
    Die("this model does not carry the declared parent frame '" + declared->parent +
        "' (a reduced model may have locked it away)");
  }
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, pinocchio::neutral(model));
  const pinocchio::SE3 rel =
      data.oMf[model.getFrameId(declared->parent)].actInv(data.oMf[model.getFrameId(frame_name)]);
  const pinocchio::SE3 expected(
      pinocchio::rpy::rpyToMatrix(declared->rpy.x(), declared->rpy.y(), declared->rpy.z()),
      declared->xyz);
  const double dp = (rel.translation() - expected.translation()).norm();
  const double dr = (rel.rotation() - expected.rotation()).norm();
  std::cout << "frame " << frame_name << "  parent " << declared->parent << "  nv " << model.nv
            << "  provisional " << (declared->provisional ? "true" : "false") << '\n';
  std::printf("declared_xyz %.17g %.17g %.17g\n", declared->xyz.x(), declared->xyz.y(),
              declared->xyz.z());
  std::printf("actual_xyz   %.17g %.17g %.17g\n", rel.translation().x(), rel.translation().y(),
              rel.translation().z());
  std::printf("residual_translation_m %.17g\nresidual_rotation_fro %.17g\n", dp, dr);
  for (int r = 0; r < 3; ++r) {
    std::printf("R_in_parent_row%d %.17g %.17g %.17g\n", r, rel.rotation()(r, 0),
                rel.rotation()(r, 1), rel.rotation()(r, 2));
  }
  constexpr double kTol = 1e-12;
  if (dp > kTol || dr > kTol) {
    std::cerr << "catch_pose_ik_batch: the built frame does not match the declared offset\n";
    return 1;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = ParseArgs(argc, argv);
    if (args.print_options) {
      std::string tree_path;
      const rtc::catching::CatchPoseIkOptions resolved = LoadOptions(args.params, tree_path);
      std::cout << "params_tree " << tree_path << '\n'
                << rtc::catching::FormatCatchPoseIkOptions(resolved);
      return 0;
    }
    const rub::ModelConfig cfg = rub::PinocchioModelBuilder::LoadModelConfig(args.model_config);
    const rub::PinocchioModelBuilder builder(cfg);
    const std::shared_ptr<const pinocchio::Model> model = PickModel(builder, args.sub_model);

    if (args.dump_frame) {
      return DumpFrame(*model, cfg, args.catch_frame);
    }
    if (args.candidates.empty() || args.seeds.empty()) {
      Die("--candidates and --seeds are required unless --dump-frame (try --help)");
    }

    // The handle must be built from the model alone: one carrying a device joint
    // order is refused by Solve as kJointOrderMismatch (note 7).
    rub::RtModelHandle handle(model);
    // Before any input is read: `GetFrameId` answers an unknown name with the
    // universe frame, and the run would then "succeed" with a map of nothing
    // but model_invalid rows.
    pinocchio::FrameIndex frame = 0;
    try {
      frame = rtc::catching::ResolveCatchFrame(handle.GetModel(), args.catch_frame);
    } catch (const std::invalid_argument& e) {
      Die(std::string("--catch-frame: ") + e.what() +
          (args.sub_model.empty() ? std::string(" [full model]")
                                  : " [--sub-model " + args.sub_model + "]"));
    }

    std::string tree_path;
    const rtc::catching::CatchPoseIkOptions opt = LoadOptions(args.params, tree_path);

    std::vector<rtc::catching::BatchCandidate> candidates;
    if (args.candidates == "-") {
      candidates = rtc::catching::ParseCandidateCsv(std::cin);
    } else {
      std::ifstream in(args.candidates);
      if (!in) {
        Die("cannot open --candidates '" + args.candidates + "'");
      }
      candidates = rtc::catching::ParseCandidateCsv(in);
    }
    std::ifstream seed_in(args.seeds);
    if (!seed_in) {
      Die("cannot open --seeds '" + args.seeds + "'");
    }
    const std::map<int, Eigen::VectorXd> seeds = rtc::catching::ParseSeedCsv(seed_in);

    const std::vector<rtc::catching::BatchRow> rows =
        rtc::catching::RunBatch(handle, frame, candidates, seeds, opt);

    std::ofstream file;
    if (!args.out.empty()) {
      file.open(args.out);
      if (!file) {
        Die("cannot open --out '" + args.out + "'");
      }
    }
    std::ostream& os = args.out.empty() ? std::cout : file;
    os << rtc::catching::BatchCsvHeader(handle.nv()) << '\n';
    for (const auto& row : rows) {
      os << rtc::catching::BatchCsvRow(row, handle.nv()) << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "catch_pose_ik_batch: " << e.what() << '\n';
    return 1;
  }
}
