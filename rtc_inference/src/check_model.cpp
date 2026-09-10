// ── rtc_inference_check — what does this .onnx actually expose? ──────────────
//
// A bring-up cycle is a slow way to learn that a model's tensors are not what
// a config says they are. This prints the answer in a second.
//
// WHY IT IS NOT A SECOND VALIDATOR. The authoritative check runs at configure
// time inside `OnnxEngine::Init`. A standalone tool that re-implemented the
// comparison would be a second copy of the same rules and would drift from the
// first — the failure mode AP-DOC-1 names. So this calls `CompareModelIo`, the
// very same function, and renders it with the very same `IoReport::Format`.
// There is nothing here for the engine to disagree with.
//
// It deliberately does NOT read the controller YAML. That schema is parsed by
// `rtc_controllers` (which owns the feature/segment rules and the yaml-cpp
// dependency); teaching this tool to read it would put a second YAML parser in
// a package that has none. Declarations are passed inline instead, which keeps
// this a lookup tool rather than a config checker.
//
//   rtc_inference_check MODEL.onnx
//       Dump the model's inputs and outputs.
//
//   rtc_inference_check MODEL.onnx --input obs:1x34 --output action:1x6
//       Compare against those declarations and exit non-zero on any mismatch.
//       An empty name (":1x34") declares an unnamed tensor, i.e. asks for the
//       positional binding an unnamed config would get.
//
//   rtc_inference_check MODEL.onnx --input obs:1x34
//       Declare one side only: that side is judged, the other is dumped and
//       does not affect the exit status.

#include "rtc_inference/shape_report.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace {

constexpr int kExitUsage = 2;

void PrintUsage() {
  std::cerr << "usage: rtc_inference_check MODEL.onnx\n"
            << "                          [--input NAME:D1xD2x... ]...\n"
            << "                          [--output NAME:D1xD2x...]...\n\n"
            << "  With no --input/--output, dumps the model's tensors.\n"
            << "  With them, compares and exits 1 on any mismatch.\n"
            << "  Declaring one side only judges that side and dumps\n"
            << "  the other.\n"
            << "  An empty NAME (\":1x34\") declares an unnamed tensor\n"
            << "  (positional binding).\n";
}

/// Parse "name:1x34" into a TensorSpec. Returns false on anything malformed —
/// a silently-misparsed dimension would make this tool lie about the very
/// thing it exists to check.
bool ParseSpec(std::string_view arg, rtc::TensorSpec& out) {
  const auto colon = arg.rfind(':');
  if (colon == std::string_view::npos) {
    return false;
  }
  out.name = std::string(arg.substr(0, colon));
  out.shape.clear();
  const auto dims = arg.substr(colon + 1);
  if (dims.empty()) {
    return false;
  }
  std::size_t pos = 0;
  while (pos <= dims.size()) {
    const auto x = dims.find('x', pos);
    const auto token = dims.substr(pos, x == std::string_view::npos ? x : x - pos);
    if (token.empty()) {
      return false;
    }
    char* end = nullptr;
    const long v = std::strtol(std::string(token).c_str(), &end, 10);
    if (end == nullptr || *end != '\0') {
      return false;
    }
    out.shape.push_back(static_cast<std::int64_t>(v));
    if (x == std::string_view::npos) {
      break;
    }
    pos = x + 1;
  }
  return !out.shape.empty();
}

}  // namespace

#ifdef HAS_ONNXRUNTIME

namespace {

/// The model's own view of itself, read straight from the session.
void CollectModelIo(Ort::Session& session, std::vector<rtc::TensorSpec>& inputs,
                    std::vector<rtc::TensorSpec>& outputs) {
  Ort::AllocatorWithDefaultOptions alloc;
  for (std::size_t i = 0; i < session.GetInputCount(); ++i) {
    inputs.push_back({session.GetInputNameAllocated(i, alloc).get(),
                      session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape()});
  }
  for (std::size_t o = 0; o < session.GetOutputCount(); ++o) {
    outputs.push_back({session.GetOutputNameAllocated(o, alloc).get(),
                       session.GetOutputTypeInfo(o).GetTensorTypeAndShapeInfo().GetShape()});
  }
}

void Dump(std::string_view label, const std::vector<rtc::TensorSpec>& tensors) {
  std::cout << "  " << label << " (" << tensors.size() << ")\n";
  for (std::size_t i = 0; i < tensors.size(); ++i) {
    // Shapes go through the shared renderer, so a dynamic axis reads as "?"
    // here exactly as it does in a configure-time failure.
    std::cout << "    [" << i << "] " << tensors[i].name << "  "
              << rtc::detail::FormatShape(tensors[i].shape) << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    PrintUsage();
    return kExitUsage;
  }
  const std::string model_path = argv[1];
  std::vector<rtc::TensorSpec> declared_inputs;
  std::vector<rtc::TensorSpec> declared_outputs;

  for (int i = 2; i < argc; ++i) {
    const std::string_view flag = argv[i];
    if (flag != "--input" && flag != "--output") {
      std::cerr << "rtc_inference_check: unknown argument '" << flag << "'\n";
      PrintUsage();
      return kExitUsage;
    }
    if (++i >= argc) {
      std::cerr << "rtc_inference_check: " << flag << " needs a NAME:D1xD2x... value\n";
      return kExitUsage;
    }
    rtc::TensorSpec spec;
    if (!ParseSpec(argv[i], spec)) {
      std::cerr << "rtc_inference_check: cannot parse '" << argv[i]
                << "' (expected NAME:D1xD2x...)\n";
      return kExitUsage;
    }
    (flag == "--input" ? declared_inputs : declared_outputs).push_back(std::move(spec));
  }

  std::vector<rtc::TensorSpec> model_inputs;
  std::vector<rtc::TensorSpec> model_outputs;
  try {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "rtc_inference_check");
    Ort::SessionOptions opts;
    Ort::Session session(env, model_path.c_str(), opts);
    CollectModelIo(session, model_inputs, model_outputs);
  } catch (const std::exception& e) {
    std::cerr << "rtc_inference_check: cannot open '" << model_path << "': " << e.what() << "\n";
    return kExitUsage;
  }

  if (declared_inputs.empty() && declared_outputs.empty()) {
    std::cout << model_path << "\n";
    Dump("inputs ", model_inputs);
    Dump("outputs", model_outputs);
    return 0;
  }

  // The same call `OnnxEngine::Init` makes, rendered the same way. If this
  // passes, configure will too.
  const auto report =
      rtc::CompareModelIo(model_inputs, declared_inputs, model_outputs, declared_outputs);

  // A side nobody declared was not asked about, and must not be answered.
  // `--input` and `--output` are independent repeatable flags, so checking the
  // inputs first is the natural way to write a config incrementally — but an
  // undeclared side is an EMPTY declaration, which the comparison rules read as
  // "the config claims none of these tensors" and mark every model tensor
  // unclaimed. Judged through `Ok()`, that reported a mismatch for a model that
  // matched. The side is dumped instead, so the operator still sees it.
  const rtc::ReportSides sides = declared_inputs.empty()    ? rtc::ReportSides::kOutputsOnly
                                 : declared_outputs.empty() ? rtc::ReportSides::kInputsOnly
                                                            : rtc::ReportSides::kBoth;
  const bool ok = (sides == rtc::ReportSides::kOutputsOnly)  ? report.OutputsOk()
                  : (sides == rtc::ReportSides::kInputsOnly) ? report.InputsOk()
                                                             : report.Ok();

  std::cout << report.Format(model_path, sides);
  if (sides == rtc::ReportSides::kInputsOnly) {
    std::cout << "  (no --output given: outputs were not checked)\n";
    Dump("outputs", model_outputs);
  } else if (sides == rtc::ReportSides::kOutputsOnly) {
    std::cout << "  (no --input given: inputs were not checked)\n";
    Dump("inputs ", model_inputs);
  }
  return ok ? 0 : 1;
}

#else  // !HAS_ONNXRUNTIME

int main() {
  std::cerr << "rtc_inference_check: built without ONNX Runtime — there is no way to open a "
               "model. Install onnxruntime and rebuild rtc_inference.\n";
  return kExitUsage;
}

#endif  // HAS_ONNXRUNTIME
