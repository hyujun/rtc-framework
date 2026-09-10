#ifndef RTC_INFERENCE_SHAPE_REPORT_HPP_
#define RTC_INFERENCE_SHAPE_REPORT_HPP_

// ── Model I/O vs declared schema: comparison and diagnostic table ────────────
//
// WHY THIS IS A SEPARATE, ORT-FREE HEADER.
//
// The failure this reports on is the single most useful one an inference
// controller can produce: a retrained .onnx whose tensors no longer match the
// config that describes them. It has to be right, and "right" here means:
//
//   1. Report EVERY discrepancy, not the first. A retrain typically moves
//      several tensors at once, so throwing on the first one costs one whole
//      bring-up cycle per tensor to discover the rest.
//   2. Print the numbers. "output[0] shape mismatch vs config" — the message
//      this replaces — tells an operator that something is wrong and nothing
//      about what to edit.
//   3. Bind by NAME when the config supplies names, because shape alone cannot
//      tell two same-shaped tensors apart (see ModelConfig's banner).
//
// Keeping it ORT-free buys exhaustive coverage cheaply: every rule below is a
// pure function over two lists of shapes, so `test_shape_report.cpp` drives all
// of them without a model file. That is a complement to, not a substitute for,
// the end-to-end gate in `test_onnx_engine.cpp` which loads a real fixture
// through ORT — the two answer different questions (is the RULE right / is it
// WIRED right).
//
// NON-RT. Called from Init/on_configure, allocates freely, and its whole
// purpose is to build a string. Never call it from a tick.

#include "rtc_inference/inference_types.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rtc {

/// How one side (inputs or outputs) was paired up.
enum class MatchMode : std::uint8_t {
  kPositional,   ///< no declared name anywhere: entry i ↔ entry i
  kByName,       ///< every declared tensor named: paired by that name
  kMixedRefused  ///< some named, some not — the intended binding is ambiguous
};

/// What happened to one paired (or unpaired) tensor.
enum class TensorVerdict : std::uint8_t {
  kOk,              ///< both sides present and compatible
  kShapeMismatch,   ///< same rank, a static dimension disagrees
  kRankMismatch,    ///< different number of dimensions
  kBadDeclaredDim,  ///< a declared dimension is not static positive
  kNotDeclared,     ///< the model has this tensor, the config does not
  kNotInModel,      ///< the config declares this tensor, the model has none
  kDuplicateName    ///< the config declares the same name twice
};

/// One row of the report.
struct TensorComparison {
  std::string name;
  TensorVerdict verdict{TensorVerdict::kOk};
  std::vector<std::int64_t> model_shape;     ///< empty when kNotInModel
  std::vector<std::int64_t> declared_shape;  ///< empty when kNotDeclared
  /// Index of the first offending dimension for kShapeMismatch /
  /// kBadDeclaredDim; -1 when the verdict is not about one dimension.
  int first_bad_dim{-1};
};

/// The full comparison, inputs and outputs together.
struct IoReport {
  std::vector<TensorComparison> inputs;
  std::vector<TensorComparison> outputs;
  std::size_t model_inputs{0};
  std::size_t declared_inputs{0};
  std::size_t model_outputs{0};
  std::size_t declared_outputs{0};
  MatchMode input_match{MatchMode::kPositional};
  MatchMode output_match{MatchMode::kPositional};

  /// True when every row is kOk and neither side was refused for mixed naming.
  /// Arity disagreement always produces at least one kNotDeclared / kNotInModel
  /// row, so it is covered by this too.
  [[nodiscard]] bool Ok() const noexcept {
    if (input_match == MatchMode::kMixedRefused || output_match == MatchMode::kMixedRefused) {
      return false;
    }
    const auto passed = [](const TensorComparison& row) {
      return row.verdict == TensorVerdict::kOk;
    };
    return std::ranges::all_of(inputs, passed) && std::ranges::all_of(outputs, passed);
  }

  /// Render the aligned table. Safe to call on a passing report (useful for an
  /// informational dump); the caller decides whether it is an error.
  [[nodiscard]] std::string Format(std::string_view model_path) const;
};

/// Compare a model's I/O against the declared schema.
///
/// PAIRING is chosen per side from the DECLARED names:
///
///   - every declared tensor named  → by name. A declared tensor whose name
///     the model does not export is kNotInModel; a model tensor no declaration
///     claims is kNotDeclared. This is what detects a swap of two tensors with
///     identical shapes, which shape comparison provably cannot.
///   - no declared tensor named     → positional, entry i ↔ entry i. Preserved
///     for consumers that do not know their model's tensor names; it inherits
///     the blind spot above.
///   - some named, some not         → refused (kMixedRefused). Guessing here
///     would silently pick one of two different bindings.
///
/// Compatibility of one dimension follows the rule the buffers require:
///   - a DECLARED dim must be static positive; it sizes a real allocation, and
///     a -1 there would cast to SIZE_MAX in the element-count product
///   - a MODEL dim < 0 is dynamic (dynamic batch/sequence) and accepts any
///     declared size
///   - otherwise the two must be equal
[[nodiscard]] IoReport CompareModelIo(const std::vector<TensorSpec>& model_inputs,
                                      const std::vector<TensorSpec>& declared_inputs,
                                      const std::vector<TensorSpec>& model_outputs,
                                      const std::vector<TensorSpec>& declared_outputs);

// ── Implementation ──────────────────────────────────────────────────────────
// Header-only: this package is an INTERFACE library, so there is no .cpp to
// put it in.

namespace detail {

/// "[1, 34]", with a dynamic (negative) model dimension shown as "?" rather
/// than as its raw -1: the distinction between "the model accepts any size
/// here" and "a dimension is literally minus one" is the whole reason a
/// dynamic axis does not fail the comparison.
inline std::string FormatShape(const std::vector<std::int64_t>& shape) {
  if (shape.empty()) {
    return "--";
  }
  std::string out = "[";
  for (std::size_t i = 0; i < shape.size(); ++i) {
    if (i != 0) {
      out.append(", ");
    }
    if (shape[i] < 0) {
      out.append("?");
    } else {
      out.append(std::to_string(shape[i]));
    }
  }
  out.append("]");
  return out;
}

/// The human-readable half of a verdict, with the numbers folded in.
inline std::string FormatVerdict(const TensorComparison& row) {
  switch (row.verdict) {
    case TensorVerdict::kOk:
      return "OK";
    case TensorVerdict::kShapeMismatch: {
      const auto d = static_cast<std::size_t>(row.first_bad_dim);
      std::string out = "shape differs at dim ";
      out.append(std::to_string(row.first_bad_dim)).append(" (model ");
      out.append(std::to_string(row.model_shape[d])).append(" != config ");
      out.append(std::to_string(row.declared_shape[d])).append(")");
      return out;
    }
    case TensorVerdict::kRankMismatch: {
      std::string out = "rank differs (model ";
      out.append(std::to_string(row.model_shape.size())).append(" dims, config ");
      out.append(std::to_string(row.declared_shape.size())).append(" dims)");
      return out;
    }
    case TensorVerdict::kBadDeclaredDim: {
      const auto d = static_cast<std::size_t>(row.first_bad_dim);
      std::string out = "config dim ";
      out.append(std::to_string(row.first_bad_dim))
          .append(" must be static positive (got ")
          .append(std::to_string(row.declared_shape[d]))
          .append(")");
      return out;
    }
    case TensorVerdict::kNotDeclared:
      return "the model has it, the config does not declare it";
    case TensorVerdict::kNotInModel:
      return "declared in the config, absent from the model";
    case TensorVerdict::kDuplicateName:
      return "the config declares this name more than once";
  }
  return "";
}

/// Compare one paired tensor. `declared` is checked for static-positive dims
/// FIRST, because a bad declared dim makes the equality question meaningless
/// (and is the operator's error either way).
inline TensorComparison ComparePair(std::string name, const std::vector<std::int64_t>& model,
                                    const std::vector<std::int64_t>& declared) {
  TensorComparison row;
  row.name = std::move(name);
  row.model_shape = model;
  row.declared_shape = declared;

  for (std::size_t i = 0; i < declared.size(); ++i) {
    if (declared[i] <= 0) {
      row.verdict = TensorVerdict::kBadDeclaredDim;
      row.first_bad_dim = static_cast<int>(i);
      return row;
    }
  }
  if (model.size() != declared.size()) {
    row.verdict = TensorVerdict::kRankMismatch;
    return row;
  }
  for (std::size_t i = 0; i < declared.size(); ++i) {
    // A negative model dim is dynamic and accepts anything the config declares.
    if (model[i] >= 0 && model[i] != declared[i]) {
      row.verdict = TensorVerdict::kShapeMismatch;
      row.first_bad_dim = static_cast<int>(i);
      return row;
    }
  }
  row.verdict = TensorVerdict::kOk;
  return row;
}

/// Which pairing the declared side asks for. Judged on the DECLARED side alone
/// — the model always names its tensors, so asking it would answer nothing.
inline MatchMode ChooseMatchMode(const std::vector<TensorSpec>& declared) {
  const auto named = [](const TensorSpec& t) { return !t.name.empty(); };
  if (declared.empty() || std::ranges::all_of(declared, named)) {
    return MatchMode::kByName;
  }
  if (std::ranges::none_of(declared, named)) {
    return MatchMode::kPositional;
  }
  return MatchMode::kMixedRefused;
}

/// Positional walk over the longer of the two sides, so a tensor that exists on
/// only one of them still gets a row instead of being dropped.
inline std::vector<TensorComparison> CompareSidePositional(
    const std::vector<TensorSpec>& model, const std::vector<TensorSpec>& declared) {
  std::vector<TensorComparison> rows;
  const std::size_t n = std::max(model.size(), declared.size());
  rows.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const bool has_model = i < model.size();
    const bool has_declared = i < declared.size();
    // Prefer the model's name: it is the one an operator sees in Netron, and
    // an unnamed declaration has none to offer.
    std::string name = has_model ? model[i].name : declared[i].name;
    if (name.empty()) {
      name = "#" + std::to_string(i);
    }
    if (has_model && has_declared) {
      rows.push_back(ComparePair(std::move(name), model[i].shape, declared[i].shape));
      continue;
    }
    TensorComparison row;
    row.name = std::move(name);
    if (has_model) {
      row.model_shape = model[i].shape;
      row.verdict = TensorVerdict::kNotDeclared;
    } else {
      row.declared_shape = declared[i].shape;
      row.verdict = TensorVerdict::kNotInModel;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

/// Name-keyed pairing. Rows come out in DECLARED order first — that is the
/// order the consumer indexes its buffers by, so the table reads in the same
/// order as the config it is diagnosing — followed by any model tensor no
/// declaration claimed.
inline std::vector<TensorComparison> CompareSideByName(const std::vector<TensorSpec>& model,
                                                       const std::vector<TensorSpec>& declared) {
  std::vector<TensorComparison> rows;
  rows.reserve(std::max(model.size(), declared.size()));
  std::vector<bool> claimed(model.size(), false);

  for (std::size_t d = 0; d < declared.size(); ++d) {
    const auto& want = declared[d];
    // A repeated declared name would make two buffers fight over one tensor;
    // the second one is the error, so the first keeps its normal verdict.
    const bool repeated =
        std::any_of(declared.begin(), declared.begin() + static_cast<long>(d),
                    [&want](const TensorSpec& prev) { return prev.name == want.name; });
    if (repeated) {
      TensorComparison row;
      row.name = want.name;
      row.declared_shape = want.shape;
      row.verdict = TensorVerdict::kDuplicateName;
      rows.push_back(std::move(row));
      continue;
    }
    const auto it =
        std::ranges::find_if(model, [&want](const TensorSpec& m) { return m.name == want.name; });
    if (it == model.end()) {
      TensorComparison row;
      row.name = want.name;
      row.declared_shape = want.shape;
      row.verdict = TensorVerdict::kNotInModel;
      rows.push_back(std::move(row));
      continue;
    }
    claimed[static_cast<std::size_t>(std::distance(model.begin(), it))] = true;
    rows.push_back(ComparePair(want.name, it->shape, want.shape));
  }

  for (std::size_t m = 0; m < model.size(); ++m) {
    if (claimed[m]) {
      continue;
    }
    TensorComparison row;
    row.name = model[m].name;
    row.model_shape = model[m].shape;
    row.verdict = TensorVerdict::kNotDeclared;
    rows.push_back(std::move(row));
  }
  return rows;
}

/// Column widths measured over a whole side, so every row of that side lines
/// up. Bundled rather than passed as three `std::size_t` because three
/// adjacent same-typed parameters are silently swappable.
struct ColumnWidths {
  std::size_t name{0};
  std::size_t model{0};
  std::size_t declared{0};
};

/// One "  [i] name  model SHAPE  config SHAPE  verdict" line.
inline void AppendRow(std::string& out, std::size_t index, const TensorComparison& row,
                      const ColumnWidths& widths) {
  const auto pad = [&out](const std::string& s, std::size_t w) {
    out.append(s);
    out.append(w > s.size() ? w - s.size() : 0, ' ');
  };
  out.append("    [").append(std::to_string(index)).append("] ");
  pad(row.name, widths.name + 2);
  out.append("model ");
  pad(FormatShape(row.model_shape), widths.model + 2);
  out.append("config ");
  pad(FormatShape(row.declared_shape), widths.declared + 2);
  out.append(FormatVerdict(row)).append("\n");
}

/// How the side was paired, spelled out. Worth a column of its own: "paired by
/// name" is the difference between a report that can see a swap of two
/// identical shapes and one that provably cannot.
inline std::string_view DescribeMatch(MatchMode mode) {
  switch (mode) {
    case MatchMode::kByName:
      return "paired by name";
    case MatchMode::kPositional:
      return "paired positionally — two tensors of the same shape are indistinguishable";
    case MatchMode::kMixedRefused:
      return "REFUSED: some tensors are named in the config and some are not";
  }
  return "";
}

inline void AppendSide(std::string& out, std::string_view label,
                       const std::vector<TensorComparison>& rows, std::size_t model_count,
                       std::size_t declared_count, MatchMode mode) {
  out.append("  ").append(label).append("  (model ").append(std::to_string(model_count));
  out.append(" / config ").append(std::to_string(declared_count)).append(", ");
  out.append(DescribeMatch(mode)).append(")\n");
  ColumnWidths widths;
  for (const auto& row : rows) {
    widths.name = std::max(widths.name, row.name.size());
    widths.model = std::max(widths.model, FormatShape(row.model_shape).size());
    widths.declared = std::max(widths.declared, FormatShape(row.declared_shape).size());
  }
  for (std::size_t i = 0; i < rows.size(); ++i) {
    AppendRow(out, i, rows[i], widths);
  }
}

}  // namespace detail

inline std::string IoReport::Format(std::string_view model_path) const {
  std::string out = "rtc_inference: '";
  out.append(model_path);
  out.append("' I/O does not match the declared schema\n");
  detail::AppendSide(out, "inputs ", inputs, model_inputs, declared_inputs, input_match);
  detail::AppendSide(out, "outputs", outputs, model_outputs, declared_outputs, output_match);
  return out;
}

inline IoReport CompareModelIo(const std::vector<TensorSpec>& model_inputs,
                               const std::vector<TensorSpec>& declared_inputs,
                               const std::vector<TensorSpec>& model_outputs,
                               const std::vector<TensorSpec>& declared_outputs) {
  IoReport report;
  report.model_inputs = model_inputs.size();
  report.declared_inputs = declared_inputs.size();
  report.model_outputs = model_outputs.size();
  report.declared_outputs = declared_outputs.size();
  report.input_match = detail::ChooseMatchMode(declared_inputs);
  report.output_match = detail::ChooseMatchMode(declared_outputs);

  // A refused side still gets rows — walked positionally — so the operator can
  // see WHICH tensors are named and which are not, rather than a bare refusal.
  const auto compare = [](MatchMode mode, const std::vector<TensorSpec>& model,
                          const std::vector<TensorSpec>& declared) {
    return (mode == MatchMode::kByName) ? detail::CompareSideByName(model, declared)
                                        : detail::CompareSidePositional(model, declared);
  };
  report.inputs = compare(report.input_match, model_inputs, declared_inputs);
  report.outputs = compare(report.output_match, model_outputs, declared_outputs);
  return report;
}

}  // namespace rtc

#endif  // RTC_INFERENCE_SHAPE_REPORT_HPP_
