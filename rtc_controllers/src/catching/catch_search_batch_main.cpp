// ── `catch_search_batch` — the planner's search, offline (L3 §4.1) ──────────
//
// Builds the catch search a configuration selects on the catch sub-model, runs
// `rtc::catching::CatchSearch::Plan` over the wakes of each throw, and writes
// one CSV row per wake. The python side (`rtc_tools.analysis.catch_search_map`)
// owns the throws, the flight that fills the prediction snapshots, the values
// a controller binds at configure time and the aggregation; the verdict is
// this binary's, so the map and the running planner share one search.
//
// ARCH-7-exempt: an offline inspection tool in the sense of
// design-principles.md §"ARCH-7 의 범위" — it knows no robot (the model, the
// catching tree, the binding and the wakes all arrive by argv), owns no RT
// loop and no ROS node, and appears in no launch file or bringup chain.
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_controllers/catching/catch_search_batch.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;

constexpr std::string_view kUsage =
    R"(catch_search_batch — the planner's catch search, offline (dynamic_catching L3 §4.1)

Every wake is the search's first look at an arm at rest on planner.wait_pose
that follows no plan; the covariance is zero and the clock does not advance.
The wakes of a throw run in order up to the first that returns a plan.

  --model-config PATH   ModelConfig YAML (PinocchioModelBuilder::LoadModelConfig
                        schema). Must declare the catch frame under `extra_frames`.
  --sub-model NAME      the catch sub-model (planner.sub_model of the profile)
  --catch-frame NAME    frame name (default: catch_frame)
  --params PATH         YAML holding the COMPOSED `catching:` tree — includes and
                        overlays already merged. Accepted shapes: a top-level
                        `catching:` map; `<controller>: {catching: ...}`; or the
                        tree itself (top-level `planner:`).
  --binding PATH        what a controller resolves at configure time and no key
                        of the tree gives: `search` (grid | nlp),
                        `device_of_model`, and that search's map (see
                        ParseSearchBatchBinding). Every key is required.
  --wakes PATH          wake CSV: throw_id,wake,now_ns,t_ns,p_{x,y,z},v_{x,y,z},
                        a_{x,y,z} — one row per predicted sample, MODEL WORLD
                        coordinates, instants in ns on one axis. '-' = stdin
  --out PATH            output CSV (default: stdout)
  -h, --help            this text
)";

[[noreturn]] void Die(const std::string& msg) {
  std::cerr << "catch_search_batch: " << msg << '\n';
  std::exit(2);
}

struct Args {
  std::string model_config;
  std::string sub_model;
  std::string catch_frame{"catch_frame"};
  std::string params;
  std::string binding;
  std::string wakes;
  std::string out;
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
    } else if (f == "--binding") {
      a.binding = value(i, f);
    } else if (f == "--wakes") {
      a.wakes = value(i, f);
    } else if (f == "--out") {
      a.out = value(i, f);
    } else {
      Die("unknown argument '" + std::string(f) + "' (try --help)");
    }
  }
  const auto require = [](std::string_view flag, const std::string& text) {
    if (text.empty()) {
      Die(std::string(flag) + " is required (try --help)");
    }
  };
  require("--model-config", a.model_config);
  require("--sub-model", a.sub_model);
  require("--params", a.params);
  require("--binding", a.binding);
  require("--wakes", a.wakes);
  return a;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = ParseArgs(argc, argv);
    const rub::ModelConfig cfg = rub::PinocchioModelBuilder::LoadModelConfig(args.model_config);
    const rub::PinocchioModelBuilder builder(cfg);
    std::shared_ptr<const pinocchio::Model> model;
    try {
      model = builder.GetReducedModel(args.sub_model);
    } catch (const std::out_of_range&) {
      Die("--sub-model '" + args.sub_model + "' is not a sub-model of the model config");
    }
    // Built from the model alone: the catch-pose IK refuses a handle that
    // carries a device joint order.
    rub::RtModelHandle handle(model);
    // Before any input is read: an unknown frame name would otherwise resolve
    // to the universe frame and the run would write a map of nothing.
    pinocchio::FrameIndex frame = 0;
    try {
      frame = rtc::catching::ResolveCatchFrame(handle.GetModel(), args.catch_frame);
    } catch (const std::invalid_argument& e) {
      Die(std::string("--catch-frame: ") + e.what() + " [--sub-model " + args.sub_model + "]");
    }

    const rtc::catching::CatchingTree tree =
        rtc::catching::ResolveCatchingTree(YAML::LoadFile(args.params), args.params);
    const rtc::catching::SearchBatchBinding binding =
        rtc::catching::ParseSearchBatchBinding(YAML::LoadFile(args.binding), model->nv);
    rtc::catching::SearchBatchSearch built =
        rtc::catching::MakeSearchBatchSearch(model, handle, frame, tree.node, binding);
    if (built.search == nullptr) {
      // A profile the search cannot run on is an answer, not a broken run —
      // but there is no map to write from it.
      std::cerr << "catch_search_batch: the search refused its configuration — " << built.error
                << '\n';
      return 3;
    }

    std::vector<rtc::catching::SearchBatchWake> wakes;
    if (args.wakes == "-") {
      wakes = rtc::catching::ParseSearchWakeCsv(std::cin);
    } else {
      std::ifstream in(args.wakes);
      if (!in) {
        Die("cannot open --wakes '" + args.wakes + "'");
      }
      wakes = rtc::catching::ParseSearchWakeCsv(in);
    }
    const std::vector<rtc::catching::SearchBatchRow> rows =
        rtc::catching::RunSearchBatch(*built.search, built.q_rest, wakes);

    std::ofstream file;
    if (!args.out.empty()) {
      file.open(args.out);
      if (!file) {
        Die("cannot open --out '" + args.out + "'");
      }
    }
    std::ostream& os = args.out.empty() ? std::cout : file;
    const int nv = model->nv;
    os << rtc::catching::SearchBatchCsvHeader(nv) << '\n';
    for (const auto& row : rows) {
      os << rtc::catching::SearchBatchCsvRow(row, nv) << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "catch_search_batch: " << e.what() << '\n';
    return 1;
  }
}
