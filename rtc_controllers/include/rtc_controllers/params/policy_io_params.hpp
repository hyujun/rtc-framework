// ── Learned-policy I/O schema (G2 schema layer) ──────────────────────────────
// The YAML that describes how a controller's state is flattened into a policy's
// input tensors and how its output tensors are sliced back into device
// commands. Sibling of params/clik_params.hpp and friends: POD + parser,
// yaml-cpp only, called once at configure time and never from a tick.
//
// WHY THE SCHEMA IS THE POINT. Retraining a policy changes feature order,
// tensor width and normalisation constants far more often than it changes what
// the controller does with the result. Every one of those must be a YAML edit,
// so this parser — not C++ — is what has to refuse a mismatch. The failure it
// exists to prevent is silent: a feature list whose order drifted from training
// produces a perfectly finite, perfectly in-limits command stream that simply
// does the wrong thing.
//
// WHY TENSORS ARE GROUPS AND NOT A FLAT LIST (#511 C-1). A multi-input policy's
// tensors are filled by DIFFERENT producers — one carries the robot observation,
// the next (P5) carries the previous step's recurrent state — and each has its
// own element space, its own feature sum and its own normalisation constants. A
// flat `input_shapes` plus a `tensor:` tag on every feature has nowhere to put
// those per-tensor invariants, so the grouping is what lets the parser state
// them once per tensor instead of re-deriving them per feature.
//
// WHY TENSOR NAMES ARE MANDATORY (#511 D-1). `rtc::InferenceEngine` binds by
// name when every declared tensor has one and positionally when none do. The
// positional path is what `udp_hand_driver` ships with, and it is exactly how a
// re-export that swapped two same-shaped tensors goes undetected (#511 B-1).
// A policy schema always names, so that path is unreachable from here.
//
// ROBOT-AGNOSTIC (ARCH-1). Feature ids are robot facts ("ur5e.position" is six
// because a UR5e has six joints), so this layer never learns the table. The
// caller passes a resolver; unknown ids are refused with the id in the message.
#pragma once

#include "rtc_controllers/inference/policy_io.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace rtc::params {

/// One declared input tensor and everything that fills it.
///
/// `segments` is parallel to `features`: entry i is where feature i landed in
/// THIS tensor, assigned by prefix sum over the YAML order. That is the whole
/// mechanism by which reordering the YAML list reorders the tensor, so a test
/// that reorders the list and observes the packed buffer is testing the
/// contract and not the parser's bookkeeping.
///
/// `offset`/`scale` are this tensor's own affine lane. Per tensor rather than
/// per policy because normalisation constants come from the statistics of the
/// quantity a tensor carries, and two tensors carrying different quantities
/// share nothing but the model that consumes them.
struct InputTensorSpec {
  std::string name;                                    ///< must match the .onnx input name
  std::vector<std::int64_t> shape;                     ///< e.g. {1, 34}
  std::vector<std::string> features;                   ///< ids, YAML order
  std::vector<rtc::inference::InputSegment> segments;  ///< resolved, parallel to `features`
  std::vector<float> offset;                           ///< empty = identity
  std::vector<float> scale;                            ///< empty = identity

  /// `source: recurrent` — this tensor is filled by the PREVIOUS step's output,
  /// not by observation features. Mutually exclusive with `features`, and it
  /// carries no affine lane: normalising a policy's own internal representation
  /// with statistics gathered from robot observations is meaningless.
  bool recurrent{false};

  /// Element count of this tensor (product of `shape`).
  [[nodiscard]] std::size_t Numel() const noexcept;
};

/// One declared output tensor. What is READ out of it is described separately
/// by `PolicyIoParams::output_features`, because one tensor can carry several
/// commands and one command never spans two tensors.
struct OutputTensorSpec {
  std::string name;                 ///< must match the .onnx output name
  std::vector<std::int64_t> shape;  ///< e.g. {1, 6}

  /// `feeds: "<input name>"` — this tensor is the next step's value for that
  /// recurrent input. Empty for an ordinary command tensor. A tensor that feeds
  /// an input is NOT also a command: it carries the policy's internal state, and
  /// slicing joint targets out of it would be reading hidden units as radians
  /// (finite and in range, so nothing downstream would object).
  std::string feeds;

  /// Element count of this tensor (product of `shape`).
  [[nodiscard]] std::size_t Numel() const noexcept;
};

/// One command a binding reads out of an output tensor.
///
/// `device` and `role` are OPAQUE STRINGS here (ARCH-1): which device groups a
/// robot has, and which roles a controller can drive, are both facts the
/// binding owns. This layer only enforces that both are declared and that the
/// PAIR is unique — a rule that holds for any robot and any controller, and the
/// one that closes #511 B-2. The pre-#511 binding inferred the role from the
/// entry's name suffix in a loop with no `break`, so two entries ending in
/// `target_position` (the natural shape of a multi-output policy: one per
/// device) silently left the last one driving both.
///
/// There is deliberately no separate `name:` key. With `device` and `role`
/// mandatory a name could only restate them, and a label that can disagree with
/// the thing it labels is a second place for the config to be wrong.
/// Diagnostics quote the pair as `<device>/<role>`.
struct OutputCommandSpec {
  std::string device;  ///< device group this command drives
  std::string role;    ///< what the slice means to that device
  rtc::inference::OutputSlice slice;
};

/// Parsed and cross-checked policy I/O description.
struct PolicyIoParams {
  std::vector<InputTensorSpec> inputs;    ///< YAML order == engine binding order
  std::vector<OutputTensorSpec> outputs;  ///< YAML order == engine binding order

  std::vector<OutputCommandSpec> output_features;  ///< YAML order

  /// Recurrent links (`h_out` → next step's `h_in`), one per `feeds:`. Empty for
  /// a feed-forward policy, which is the common case and costs it nothing.
  ///
  /// WHO ACTS ON THIS. Not the engine — `udp_hand_driver` shares it and has no
  /// recurrence, and "when does the state reset" is a control decision a tensor
  /// runner has no standing to make. The binding owns the timing; this layer
  /// owns the declaration and its cross-checks.
  std::vector<rtc::inference::RecurrentLink> recurrent_links;

  /// RT ticks per policy evaluation. 1 = every tick. The controller holds the
  /// previous action on the ticks in between, so this is what turns a 500 Hz
  /// control loop into a 50 Hz policy without touching `control_rate`.
  int decimation{1};
};

/// Element count for a feature id, or <= 0 when the id is unknown.
///
/// Supplied by the CALLER rather than tabulated here: see the ARCH-1 note in
/// the file banner. A resolver that returns 0 for an id it does not recognise
/// is what turns a typo into a configure failure instead of a zero-width
/// feature that silently shifts every subsequent offset.
using FeatureSizeFn = std::function<int(std::string_view)>;

/// Parse the `inference:` I/O schema out of @p cfg.
///
/// Throws `std::invalid_argument` on any violation below — non-RT, called from
/// LoadConfig / on_configure, where a loud failure is the correct outcome:
///
///   - `inputs` / `outputs` missing or empty, an entry that is not a map, a
///     missing or empty `name`, a duplicate name within either side, or a
///     `shape` that is absent, empty, or carries a dimension that is not
///     strictly positive
///   - `features` empty, carrying an id the resolver rejects, or repeating an
///     id declared by ANY input tensor (#511 D-6 — the message names both
///     positions, because the same id in two tensors is a copy-paste artefact
///     far more often than it is deliberate double normalisation)
///   - the resolved feature counts of a tensor not summing to that tensor's
///     element count
///   - `offset` / `scale` present with a length that is neither 0 nor that
///     tensor's element count, or carrying a non-finite value
///   - `output_features` empty, missing `device` or `role`, repeating a
///     (`device`, `role`) pair, naming a tensor that was not declared, slicing
///     past that tensor's element count, declaring a non-positive count, or
///     overlapping another slice **on the same tensor**
///   - `decimation` < 1
///   - an input declaring both `features` and `source: recurrent`, or neither;
///     a `source:` that is not the literal `recurrent`; an affine lane on a
///     recurrent tensor; or a recurrent tensor that no output feeds (nothing
///     would ever write it, so the policy would read zeros forever)
///   - a `feeds:` naming an input that does not exist or is not recurrent,
///     whose element count differs, or that another output already feeds; and
///     an `output_features` entry slicing a tensor that feeds an input
///   - the pre-#511 flat keys (`input_shape`, `output_shapes`,
///     `input_features`), refused with the migration named rather than left to
///     surface as "inputs is missing"
///
/// Overlap is judged per tensor because two tensors legitimately start at
/// offset 0; it is two slices of the SAME tensor claiming an element that means
/// one of them is not reading what its name says.
PolicyIoParams ParsePolicyIoParams(const YAML::Node& cfg, const FeatureSizeFn& feature_size);

}  // namespace rtc::params
