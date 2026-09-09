// ── Learned-policy I/O schema (G2 schema layer) ──────────────────────────────
// The YAML that describes how a controller's state is flattened into a policy's
// input tensor and how its output heads are sliced back into device commands.
// Sibling of params/clik_params.hpp and friends: POD + parser, yaml-cpp only,
// called once at configure time and never from a tick.
//
// WHY THE SCHEMA IS THE POINT. Retraining a policy changes feature order,
// tensor width and normalisation constants far more often than it changes what
// the controller does with the result. Every one of those must be a YAML edit,
// so this parser — not C++ — is what has to refuse a mismatch. The failure it
// exists to prevent is silent: a feature list whose order drifted from training
// produces a perfectly finite, perfectly in-limits command stream that simply
// does the wrong thing.
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

/// Parsed and cross-checked policy I/O description.
///
/// `input_segments` is parallel to `input_features`: entry i is where feature i
/// landed in the flattened tensor, assigned by prefix sum over the YAML order.
/// That is the whole mechanism by which reordering the YAML list reorders the
/// tensor, so a test that reorders the list and observes the packed buffer is
/// testing the contract and not the parser's bookkeeping.
struct PolicyIoParams {
  std::vector<std::int64_t> input_shape;                 ///< e.g. {1, 34}
  std::vector<std::vector<std::int64_t>> output_shapes;  ///< one per head, e.g. {{1,6},{1,1}}

  std::vector<std::string> input_features;                   ///< ids, YAML order
  std::vector<rtc::inference::InputSegment> input_segments;  ///< resolved, parallel to above
  std::vector<float> input_offset;                           ///< empty = identity
  std::vector<float> input_scale;                            ///< empty = identity

  std::vector<std::string> output_names;                   ///< names, YAML order
  std::vector<rtc::inference::OutputSlice> output_slices;  ///< parallel to above

  /// RT ticks per policy evaluation. 1 = every tick. The controller holds the
  /// previous action on the ticks in between, so this is what turns a 500 Hz
  /// control loop into a 50 Hz policy without touching `control_rate`.
  int decimation{1};

  /// Total element count of the input tensor (product of `input_shape`).
  [[nodiscard]] std::size_t InputNumel() const noexcept;
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
///   - `input_shape` / `output_shapes` missing, empty, or carrying a dimension
///     that is not strictly positive
///   - `input_features` empty, carrying an id the resolver rejects, or carrying
///     the same id twice (a repeat is a copy-paste artefact; the sum check
///     below would otherwise have to be wrong in a second place to catch it)
///   - the resolved feature counts not summing to `InputNumel()`
///   - `input_offset` / `input_scale` present with a length that is neither 0
///     nor `InputNumel()`, or carrying a non-finite value
///   - `output_features` empty, naming a head that does not exist, slicing past
///     that head's element count, declaring a non-positive count, repeating a
///     name, or overlapping another slice **on the same head**
///   - `decimation` < 1
///
/// Overlap is judged per head because two heads legitimately start at offset 0;
/// it is two slices of the SAME head claiming an element that means one of them
/// is not reading what its name says.
PolicyIoParams ParsePolicyIoParams(const YAML::Node& cfg, const FeatureSizeFn& feature_size);

}  // namespace rtc::params
