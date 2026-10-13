// ── `catch_search_batch` — the planner's search, offline (L3 §4.1) ──────────
//
// Builds the catch search a configuration selects on the catch sub-model, runs
// `rtc::catching::CatchSearch::Plan` over the wakes of each throw, and writes
// one CSV row per wake. A binding that also carries a segment planner's values
// runs the whole planner wake instead (`rtc::catching::PlannerCycle::Run`: the
// search, the first segment, the pair). The python side
// (`rtc_tools.analysis.catch_search_map`)
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

With `segment` in the binding a wake is the whole planner wake — the search,
the first segment's solve and the cycle's decision to publish the pair — on a
clock that stands at the wake's instant. The wakes of a throw then run up to
the first whose pair is published, and each row also says what the cycle and
the segment planner did (outcome, segment_outcome, segment_core_reason, ...).

  --model-config PATH   ModelConfig YAML (PinocchioModelBuilder::LoadModelConfig
                        schema). Must declare the catch frame under `extra_frames`.
  --sub-model NAME      the catch sub-model (planner.sub_model of the profile):
                        a `sub_models` entry of the model config, else a
                        `tree_models` entry
  --search-sub-model NAME
                        the search's own model (planner.search.sub_model of the
                        profile), when it is a chain of --sub-model: a
                        `sub_models` entry. The search runs on it and — in a
                        whole wake — the segment planner on --sub-model; the
                        binding then gives `n_dev` and `segment.device_of_model`
  --catch-frame NAME    frame name (default: catch_frame)
  --params PATH         YAML holding the COMPOSED `catching:` tree — includes and
                        overlays already merged. Accepted shapes: a top-level
                        `catching:` map; `<controller>: {catching: ...}`; or the
                        tree itself (top-level `planner:`).
  --binding PATH        what a controller resolves at configure time and no key
                        of the tree gives: `search` (grid | nlp),
                        `device_of_model`, and that search's map (see
                        ParseSearchBatchBinding). Every key is required.
                        Optional: `segment` (`mode`: mpc | mpc_docking, and that
                        planner's map) — the whole wake is then run
  --wakes PATH          wake CSV: throw_id,wake,now_ns,t_ns,p_{x,y,z},v_{x,y,z},
                        a_{x,y,z} — one row per predicted sample, MODEL WORLD
                        coordinates, instants in ns on one axis. '-' = stdin
  --out PATH            output CSV (default: stdout); the last column, wall_us, is
                        the wall-clock time of the Plan call [us]
  --print-reach-bound   print the reach pre-filter's sphere (catch_reach_bound/1,
                        one JSON line: centre in the model world, radius, each
                        search's tolerance) and exit 0. Reads no
                        wakes and builds no search: --binding and --wakes are
                        not needed
  -h, --help            this text
)";

[[noreturn]] void Die(const std::string& msg) {
  std::cerr << "catch_search_batch: " << msg << '\n';
  std::exit(2);
}

struct Args {
  std::string model_config;
  std::string sub_model;
  std::string search_sub_model;  // empty: --sub-model for the search too
  std::string catch_frame{"catch_frame"};
  std::string params;
  std::string binding;
  std::string wakes;
  std::string out;
  bool print_reach_bound{false};
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
    } else if (f == "--search-sub-model") {
      a.search_sub_model = value(i, f);
    } else if (f == "--catch-frame") {
      a.catch_frame = value(i, f);
    } else if (f == "--params") {
      a.params = value(i, f);
    } else if (f == "--binding") {
      a.binding = value(i, f);
    } else if (f == "--wakes") {
      a.wakes = value(i, f);
    } else if (f == "--print-reach-bound") {
      a.print_reach_bound = true;
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
  if (!a.print_reach_bound) {
    require("--binding", a.binding);
    require("--wakes", a.wakes);
  }
  return a;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = ParseArgs(argc, argv);
    const rub::ModelConfig cfg = rub::PinocchioModelBuilder::LoadModelConfig(args.model_config);
    const rub::PinocchioModelBuilder builder(cfg);
    // --sub-model: a chain (`sub_models`) or, for a device that is a tree, a
    // `tree_models` entry — the order a controller's configure looks it up in.
    const auto declared = [](const auto& entries, const std::string& name) {
      for (const auto& e : entries) {
        if (e.name == name) {
          return true;
        }
      }
      return false;
    };
    std::shared_ptr<const pinocchio::Model> whole;
    if (declared(cfg.sub_models, args.sub_model)) {
      whole = builder.GetReducedModel(args.sub_model);
    } else if (declared(cfg.tree_models, args.sub_model)) {
      whole = builder.GetTreeModel(args.sub_model);
    } else {
      Die("--sub-model '" + args.sub_model + "' is not a sub-model of the model config");
    }
    // The search's model: its own chain, or --sub-model.
    const bool two_models =
        !args.search_sub_model.empty() && args.search_sub_model != args.sub_model;
    std::shared_ptr<const pinocchio::Model> model = whole;
    if (two_models) {
      if (!declared(cfg.sub_models, args.search_sub_model)) {
        Die("--search-sub-model '" + args.search_sub_model +
            "' is not a `sub_models` entry of the model config");
      }
      model = builder.GetReducedModel(args.search_sub_model);
    }
    const std::string& search_name = two_models ? args.search_sub_model : args.sub_model;
    // Built from the model alone: the catch-pose IK refuses a handle that
    // carries a device joint order.
    rub::RtModelHandle handle(model);
    // Before any input is read: an unknown frame name would otherwise resolve
    // to the universe frame and the run would write a map of nothing.
    pinocchio::FrameIndex frame = 0;
    rtc::catching::SegmentBatchModel segment_model;
    try {
      frame = rtc::catching::ResolveCatchFrame(handle.GetModel(), args.catch_frame);
      if (two_models) {
        segment_model.arm = whole;
        segment_model.catch_frame = rtc::catching::ResolveCatchFrame(*whole, args.catch_frame);
      }
    } catch (const std::invalid_argument& e) {
      Die(std::string("--catch-frame: ") + e.what() + " [--sub-model " + args.sub_model + "]");
    }

    const rtc::catching::CatchingTree tree =
        rtc::catching::ResolveCatchingTree(YAML::LoadFile(args.params), args.params);
    if (args.print_reach_bound) {
      // The sphere both searches build (`ComputeReachBound` at its default
      // iterations on the same model and frame) and the tolerance they test it with.
      const rtc::catching::ReachBound bound =
          rtc::catching::ComputeReachBound(handle.GetModel(), frame);
      std::cout << rtc::catching::ReachBoundJson(bound, args.catch_frame, search_name,
                                                 rtc::catching::SearchReachTolerances(tree.node))
                << '\n';
      return 0;
    }
    const rtc::catching::SearchBatchBinding binding =
        rtc::catching::ParseSearchBatchBinding(YAML::LoadFile(args.binding), model->nv, whole->nv);
    // A profile a function cannot run on is an answer, not a broken run — but
    // there is no map to write from it.
    const bool whole_wake = binding.segment.has_value();
    rtc::catching::SearchBatchSearch built;
    rtc::catching::PlanBatchBuild cycle;
    if (whole_wake) {
      cycle = rtc::catching::MakePlanBatch(model, handle, frame, tree.node, binding,
                                           two_models ? &segment_model : nullptr);
      if (cycle.batch == nullptr) {
        std::cerr << "catch_search_batch: the planner refused its configuration — " << cycle.error
                  << '\n';
        return 3;
      }
    } else {
      built = rtc::catching::MakeSearchBatchSearch(model, handle, frame, tree.node, binding);
      if (built.search == nullptr) {
        std::cerr << "catch_search_batch: the search refused its configuration — " << built.error
                  << '\n';
        return 3;
      }
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
        whole_wake ? cycle.batch->Run(wakes)
                   : rtc::catching::RunSearchBatch(*built.search, built.q_rest, wakes);

    std::ofstream file;
    if (!args.out.empty()) {
      file.open(args.out);
      if (!file) {
        Die("cannot open --out '" + args.out + "'");
      }
    }
    std::ostream& os = args.out.empty() ? std::cout : file;
    // q_star is written per DEVICE joint.
    const int nv = binding.n_dev > 0 ? binding.n_dev : model->nv;
    os << (whole_wake ? rtc::catching::PlanBatchCsvHeader(nv)
                      : rtc::catching::SearchBatchCsvHeader(nv))
       << '\n';
    for (const auto& row : rows) {
      os << (whole_wake ? rtc::catching::PlanBatchCsvRow(row, nv)
                        : rtc::catching::SearchBatchCsvRow(row, nv))
         << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "catch_search_batch: " << e.what() << '\n';
    return 1;
  }
}
