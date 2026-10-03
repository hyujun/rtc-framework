// Controller config composition — one YAML file plus its `include:` fragments.
//
// A controller's config is `<config_key>.yaml` with the controller's tree under
// the top-level `<config_key>:` key. A file may also carry a top-level
// `include:` list naming sibling fragments; each fragment holds part of the same
// tree, again under `<config_key>:`, and the loader merges them into one node:
//
//   # demo_x_controller.yaml            # x/part_a.yaml
//   include:                            demo_x_controller:
//     - x/part_a.yaml                     gains:
//   demo_x_controller:                      kd: 2.0
//     gains:
//       kp: 1.0
//
// The CM applies ROS-parameter overrides to the COMPOSED node, so an override
// reaches a fragment's key exactly as it reaches a key in the main file.
//
// Every other reader of a controller config (tests of shipped profiles, offline
// tools) must go through this function — or its Python twin,
// `rtc_tools.utils.controller_config.load_controller_config` — rather than
// `YAML::LoadFile`. A reader that loads only the main file sees a tree with the
// fragments' keys silently missing.
#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace rtc {

/// A controller config that exists but cannot be composed: a missing or
/// unreadable fragment, a bad `include:` entry, or two files setting the same
/// key. The message names the files and the key path.
///
/// Deliberately NOT a `YAML::BadFile`. The CM reads `BadFile` as "this variant
/// ships no config for the controller" and runs on defaults (or skips a
/// `config_required` controller). A fragment that went missing is a broken
/// config, never an absent one.
class ControllerConfigIncludeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// Loads @p yaml_path and returns the node under @p config_key with every
/// `include:` fragment merged in.
///
/// Without a top-level `include:` key the result is `file[config_key]`,
/// unchanged — including the undefined node a misspelled key yields.
///
/// With one:
///  - the main file's top-level keys must be exactly `include` and
///    @p config_key, and @p config_key must hold a map;
///  - `include` is a list of paths relative to the main file's directory.
///    Absolute paths and `..` components are rejected;
///  - a fragment's only top-level key is @p config_key (a map). A fragment
///    cannot itself `include`;
///  - maps merge recursively. Scalars, sequences and nulls are leaves: a leaf
///    set by two files, or a key that is a map in one file and a leaf in
///    another, is an error;
///  - key order is the main file's, then each fragment's new keys in include
///    order.
///
/// @throws YAML::BadFile  @p yaml_path itself cannot be opened ("no config").
/// @throws ControllerConfigIncludeError  any composition failure above.
/// @throws YAML::Exception  @p yaml_path does not parse.
[[nodiscard]] YAML::Node LoadControllerConfig(const std::string& yaml_path,
                                              const std::string& config_key);

/// One line per leaf of @p node, in tree order: `<dotted.key.path>\t<value>`.
///
/// A scalar's value is its text, a null is `null`, and a sequence is one leaf
/// rendered as `[item, item]` (a map inside a sequence as `{key: value}`). This
/// is the form two readers are compared in:
/// `rtc_tools.utils.controller_config.controller_config_leaf_lines` produces
/// the same lines from the Python loader's tree, so a change here must be made
/// there too.
[[nodiscard]] std::vector<std::string> ControllerConfigLeafLines(const YAML::Node& node);

}  // namespace rtc
