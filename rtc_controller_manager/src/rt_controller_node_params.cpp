// ── Parameter declaration and loading
// ──────────────────────────────────────────
#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"
#include <rtc_base/logging/session_dir.hpp>
#include <rtc_urdf_bridge/closure_yaml_loader.hpp>
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/urdf_analyzer.hpp>
#include <rtc_urdf_bridge/xacro_processor.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>  // std::lround
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace urtc = rtc;

namespace {

// Resolve `<pkg_share>/<rel>` for a package-share-relative resource path.
std::string PackageSharePath(const std::string& pkg, const std::string& rel) {
  return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
}

// Split `str` on `.` into non-empty components.
std::vector<std::string> SplitDotPath(const std::string& str) {
  std::vector<std::string> out;
  std::string::size_type start = 0;
  for (auto pos = str.find('.'); pos != std::string::npos; pos = str.find('.', start)) {
    if (pos > start) {
      out.emplace_back(str.substr(start, pos - start));
    }
    start = pos + 1;
  }
  if (start < str.size()) {
    out.emplace_back(str.substr(start));
  }
  return out;
}

// Write @p param's typed value into @p node under @p key.
// Supports the scalar + string-array parameter kinds that controller YAMLs
// exercise; silently skips types this helper hasn't been extended to carry.
void SetYamlScalarFromParam(YAML::Node node, const std::string& key,
                            const rclcpp::Parameter& param) {
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      node[key] = param.as_bool();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      node[key] = param.as_int();
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      node[key] = param.as_double();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      node[key] = param.as_string();
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
      YAML::Node arr(YAML::NodeType::Sequence);
      for (bool v : param.as_bool_array()) {
        arr.push_back(v);
      }
      node[key] = arr;
      break;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      YAML::Node arr(YAML::NodeType::Sequence);
      for (int64_t v : param.as_integer_array()) {
        arr.push_back(v);
      }
      node[key] = arr;
      break;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      YAML::Node arr(YAML::NodeType::Sequence);
      for (double v : param.as_double_array()) {
        arr.push_back(v);
      }
      node[key] = arr;
      break;
    }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      YAML::Node arr(YAML::NodeType::Sequence);
      for (const auto& v : param.as_string_array()) {
        arr.push_back(v);
      }
      node[key] = arr;
      break;
    }
    default:
      // Unsupported: byte_array, not_set. Silently skip.
      break;
  }
}

// Walk ROS parameter names under `<config_key>.` and apply each as a nested
// write into @p ctrl_node, treating `.` as the YAML path separator.
//
// Example: param `demo_wbc_controller.mpc.enabled = true` applied against
// the node loaded from `demo_wbc_controller.yaml` pokes `mpc.enabled` to
// `true`, overriding whatever the YAML file contained.
//
// Rationale — the controller config is read via `YAML::LoadFile(...)` and
// does NOT participate in the ROS parameter tree, so launch-arg overrides
// (e.g. sim_ur5e_p1a.launch.py's `enable_mpc` / `mpc_engine` flags) would otherwise
// be silently discarded. This helper is the single bridge between the two.
void ApplyControllerParamOverrides(rclcpp_lifecycle::LifecycleNode& node, YAML::Node ctrl_node,
                                   const std::string& config_key) {
  // list_parameters depth counts `.`-separated segments from the root; we
  // set a generous cap to cover any reasonable nested YAML.
  constexpr uint64_t kMaxDepth = 20;
  const auto params = node.list_parameters({config_key}, kMaxDepth);
  const std::string prefix = config_key + ".";

  for (const auto& pname : params.names) {
    if (pname.rfind(prefix, 0) != 0) {
      continue;
    }
    const auto parts = SplitDotPath(pname.substr(prefix.size()));
    if (parts.empty()) {
      continue;
    }

    // Walk to the parent map. CRITICAL: `Node::operator=` in yaml-cpp is a
    // VALUE COPY that overwrites the left-hand node's data — using it here
    // (e.g. `parent = parent[x]`) would splice the child subtree over the
    // ctrl_node root and lose sibling keys (observed as the grand
    // `integration` section vanishing after applying a single
    // `demo_wbc_controller.mpc.*` override). `Node::reset(other)` is the
    // reference-rebind that leaves the underlying tree untouched.
    YAML::Node parent(ctrl_node);
    for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
      parent.reset(parent[parts[i]]);
    }

    try {
      SetYamlScalarFromParam(parent, parts.back(), node.get_parameter(pname));
    } catch (const std::exception& e) {
      RCLCPP_WARN(node.get_logger(), "[param-override] failed to apply '%s': %s", pname.c_str(),
                  e.what());
    }
  }
}

}  // namespace

// ── Initialisation helpers
// ────────────────────────────────────────────────────
void RtControllerNode::ResetControllerBringUpState() {
  controllers_.clear();
  controller_nodes_.clear();
  controller_types_.clear();
  controller_topic_configs_.clear();
  controller_name_to_idx_.clear();
  controller_slot_mappings_.clear();
  active_groups_.clear();
  group_slot_map_.clear();
  active_controller_idx_.store(0);

  // Device-config state built by LoadDeviceNameConfigs(). A configure that
  // fails returns FAILURE, which parks the node in UNCONFIGURED — on_cleanup
  // (the only other place these are cleared) is reachable from INACTIVE only,
  // so without this a retry inherits the refused attempt's device state. It
  // matters when the retry's group set SHRINKS: LoadDeviceNameConfigs()
  // re-assigns the two slot vectors wholesale but only inserts-or-overwrites
  // per group, so a group that is no longer claimed keeps its stale
  // DeviceNameConfig and still reaches every controller via
  // SetDeviceNameConfigs().
  device_name_configs_.clear();
  slot_to_group_name_.clear();
  slot_to_sensor_layout_.clear();

  // Deliberately NOT reset here: device_timeouts_. It is written only after
  // the last ResetControllerBringUpState() call site in
  // DeclareAndLoadParameters(), so no refused attempt can leave it dirty, and
  // a retry after a SUCCESSFUL configure must pass through on_cleanup (which
  // does clear it). Adding it would defend a state that cannot occur.
}

// ── Per-controller flat slot mappings (RT-safe, no map lookup) ──────────────
//
// Phase 4: capability per slot is unknown here (it depends on the backend
// impl, and is patched in by PropagateCapabilitiesIntoMappings after
// CreateDeviceBackends). Slots / num_groups are filled now; capabilities[]
// stays zero.
//
// Both guards below defend against a group that reaches here without a slot
// or past the fixed slot capacity. Neither is reachable through YAML today —
// the controller-interface parser refuses a group that declares neither
// `subscribe:` nor `publish:` (rt_controller_interface.cpp), so the
// topics-filter that builds group_slot_map_ never actually drops a group, and
// the group-count ceiling upstream therefore sees every group. But that is an
// invariant enforced in a *different package*, with nothing binding the two
// together: the moment the parser contract relaxes, a phantom group would
// alias device slot 0 (wrong actuator, wrong digital twin, duplicate watchdog
// entry) and the RT read loop would index past its fixed arrays. Same
// judgement `1ab99b8` applied to controller_name_to_idx_ — the container has
// to be safe on its own rather than rely on who reads it next.
bool RtControllerNode::BuildControllerSlotMappings() {
  controller_slot_mappings_.assign(controllers_.size(), ControllerSlotMapping{});
  for (std::size_t ci = 0; ci < controllers_.size(); ++ci) {
    auto& mapping = controller_slot_mappings_[ci];
    const auto& groups = controller_topic_configs_[ci].groups;

    // Counted before the loop: the RT read loop iterates `groups` directly,
    // so its bound is the group count as declared, not the count that
    // survived any filtering.
    if (groups.size() > static_cast<std::size_t>(ControllerSlotMapping::kMaxSlots)) {
      RCLCPP_FATAL(get_logger(),
                   "Controller '%s' declares %zu device groups, past the RT path's fixed capacity "
                   "of %d — refusing to configure",
                   controllers_[ci]->Name().data(), groups.size(),
                   ControllerSlotMapping::kMaxSlots);
      return false;
    }

    int gi = 0;
    for (const auto& [gname, ggroup] : groups) {
      const auto it = group_slot_map_.find(gname);
      if (it == group_slot_map_.end()) {
        // operator[] would have default-inserted 0 here, silently routing this
        // group's commands to whichever device owns slot 0.
        RCLCPP_FATAL(get_logger(),
                     "Controller '%s' group '%s' has no device slot — refusing to configure "
                     "rather than defaulting it to slot 0",
                     controllers_[ci]->Name().data(), gname.c_str());
        return false;
      }
      mapping.slots[static_cast<std::size_t>(gi)] = it->second;
      ++gi;
    }
    mapping.num_groups = gi;
  }
  return true;
}

bool RtControllerNode::DeclareAndLoadParameters() {
  // Helper: declare only if not already auto-declared from YAML overrides.
  // (NodeOptions::automatically_declare_parameters_from_overrides is enabled.)
  auto safe_declare = [this](const std::string& name, const rclcpp::ParameterValue& val) {
    if (!has_parameter(name)) {
      declare_parameter(name, val);
    }
  };

  // RT loop rate. Framework supports kMin..kMaxControlRateHz; 500 Hz is the
  // declare_parameter default only — robot bringups should set this in YAML
  // (e.g. 1000.0 / 2000.0 for higher-bandwidth devices, lower for ToF-rich
  // sensing loops).
  safe_declare("control_rate", rclcpp::ParameterValue(rtc::kDefaultControlRateHz));
  safe_declare("kp", rclcpp::ParameterValue(5.0));
  safe_declare("kd", rclcpp::ParameterValue(0.5));
  safe_declare("enable_logging", rclcpp::ParameterValue(true));
  safe_declare("log_dir", rclcpp::ParameterValue(std::string("")));
  safe_declare("max_log_sessions", rclcpp::ParameterValue(10));
  safe_declare("enable_timing_log", rclcpp::ParameterValue(true));
  safe_declare("enable_device_log", rclcpp::ParameterValue(true));
  safe_declare("enable_estop", rclcpp::ParameterValue(true));
  safe_declare("init_timeout_sec", rclcpp::ParameterValue(5.0));
  // Default empty: robot bringup yaml must specify which controller to start.
  // Code default carries no assumption about which controllers are registered
  // (rtc_* stays robot-agnostic; controllers are registered by robot bringup).
  safe_declare("initial_controller", rclcpp::ParameterValue(std::string("")));
  safe_declare("use_sim_time_sync", rclcpp::ParameterValue(false));
  safe_declare("sim_sync_timeout_sec", rclcpp::ParameterValue(5.0));

  // Robot config variant directory.  Path is composed as
  // <pkg_share>/config/<config_variant>/controllers/<config_key>.yaml.
  // Empty (default) → legacy flat layout (config/controllers/...).  Robot
  // bringups should set this in their system YAML so all variant-specific
  // configs live under one directory.  No trailing slash.
  safe_declare("config_variant", rclcpp::ParameterValue(std::string("")));

  // ── Device timeouts (per-device, indexed by group name) ──────────────────
  safe_declare("device_timeout_names", rclcpp::ParameterValue(std::vector<std::string>{}));
  safe_declare("device_timeout_values", rclcpp::ParameterValue(std::vector<double>{}));
  // Timeout applied to a configured device group that device_timeout_names
  // does not mention. Every device group is watched (issue #198 §1), so this
  // is what an unlisted group gets — deliberately generous, because a value
  // nobody chose should not be the reason a robot E-STOPs. Matches the
  // tightest value the shipped robot configs use, so an omission is caught by
  // the WARN it emits rather than by a surprise trip.
  safe_declare("device_timeout_default_ms", rclcpp::ParameterValue(1000.0));

  control_rate_ = get_parameter("control_rate").as_double();
  if (!(control_rate_ >= rtc::kMinControlRateHz && control_rate_ <= rtc::kMaxControlRateHz)) {
    RCLCPP_WARN(get_logger(),
                "control_rate=%.1f Hz is outside the design range [%.0f, %.0f]; "
                "proceeding but timing tests may not pass.",
                control_rate_, rtc::kMinControlRateHz, rtc::kMaxControlRateHz);
  }
  budget_us_ = 1.0e6 / control_rate_;

  // Rate-derived E-STOP threshold for rejected controller output. The floor
  // keeps the window meaningful at the low end of the supported rate range,
  // where kOutputRejectEstopSeconds alone would fire after a couple of ticks.
  invalid_output_estop_ticks_ = std::max<uint64_t>(
      kMaxConsecutiveOverruns,
      static_cast<uint64_t>(std::lround(kOutputRejectEstopSeconds * control_rate_)));

  // Rate-derived watchdog cadence, same reasoning as the threshold above:
  // CheckTimeouts() owes a fixed kWatchdogCheckHz, not a fixed tick count.
  watchdog_check_divisor_ = std::max<std::uint32_t>(
      1, static_cast<std::uint32_t>(std::lround(control_rate_ / kWatchdogCheckHz)));

  // init_timeout_ticks_ is rate-dependent — recomputed here so that the
  // wall-clock window stays at `init_timeout_sec` regardless of the
  // configured loop rate.
  const double init_timeout_sec = get_parameter("init_timeout_sec").as_double();
  init_timeout_ticks_ = static_cast<uint64_t>(init_timeout_sec * control_rate_);
  enable_logging_ = get_parameter("enable_logging").as_bool();
  enable_estop_ = get_parameter("enable_estop").as_bool();
  use_sim_time_sync_ = get_parameter("use_sim_time_sync").as_bool();
  sim_sync_timeout_sec_ = get_parameter("sim_sync_timeout_sec").as_double();

  // NOTE: Logging setup and device name configs are deferred until after
  // controller loading, because active_groups_ must be known first.

  // ── Resolve system URDF and model topology ─────────────────────────────────
  std::string urdf_path;
  // Package that owns the resolved URDF. Captured by whichever branch below
  // wins so the Extended-URDF closure resolution can resolve a urdf.closure_path
  // override against the same package share dir regardless of the source branch.
  std::string urdf_pkg;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> shared_builder;
  {
    // 1) Top-level urdf section (preferred)
    if (has_parameter("urdf.package") && has_parameter("urdf.path")) {
      try {
        const auto pkg = get_parameter("urdf.package").as_string();
        const auto rel = get_parameter("urdf.path").as_string();
        urdf_path = PackageSharePath(pkg, rel);
        system_model_config_.urdf_path = urdf_path;
        urdf_pkg = pkg;

        if (has_parameter("urdf.root_joint_type")) {
          system_model_config_.root_joint_type = get_parameter("urdf.root_joint_type").as_string();
        }

        ParseSubModels(system_model_config_);
        ParseTreeModels(system_model_config_);

        if (has_parameter("urdf.passive_joints")) {
          system_model_config_.passive_joints =
              get_parameter("urdf.passive_joints").as_string_array();
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to resolve system URDF config: %s", e.what());
      }
    }

    // 2) Fallback: scan devices for first URDF config (backward compatibility)
    if (urdf_path.empty()) {
      const auto params = list_parameters({"devices"}, 10);
      for (const auto& prefix : params.prefixes) {
        const std::string pkg_key = prefix + ".urdf.package";
        const std::string path_key = prefix + ".urdf.path";
        if (has_parameter(pkg_key) && has_parameter(path_key)) {
          try {
            const auto pkg = get_parameter(pkg_key).as_string();
            const auto rel = get_parameter(path_key).as_string();
            urdf_path = PackageSharePath(pkg, rel);
            system_model_config_.urdf_path = urdf_path;
            urdf_pkg = pkg;
            RCLCPP_INFO(get_logger(), "URDF path from devices config (fallback): %s",
                        urdf_path.c_str());
          } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to resolve URDF from devices config: %s", e.what());
          }
          break;
        }
      }
    }

    if (urdf_path.empty()) {
      RCLCPP_WARN(get_logger(), "No URDF configured — controllers may lack kinematics");
    }

    // ── Extended-URDF: resolve the closure sidecar (both branches) ───────────
    // urdf.extended=true declares a spanning-tree URDF whose loop-closure /
    // actuation info lives in a sibling <stem>.closure.yaml. Resolved here —
    // AFTER both the top-level and devices-fallback branches settle urdf_path —
    // so a devices-config bring-up honours urdf.extended identically (this block
    // used to be nested inside the top-level branch and was silently skipped on
    // the fallback path). The shared builder below then runs the closed-chain
    // pipeline (constraints + q_ref + actuated joints) on the full model.
    //
    // Isolated in its own try/catch so a bad urdf.extended type or an
    // unresolvable closure_path cannot abort the shared-builder construction.
    // A missing sidecar under extended:true is escalated to ERROR (not WARN):
    // the model silently loses loop constraints, so a closed-chain robot's
    // dynamics would be wrong — that must be loud.
    if (!urdf_path.empty()) {
      try {
        const bool extended =
            has_parameter("urdf.extended") && get_parameter("urdf.extended").as_bool();
        if (extended) {
          std::string closure_path;
          if (has_parameter("urdf.closure_path")) {
            // Explicit override, resolved against the URDF's package share dir.
            const auto rel_closure = get_parameter("urdf.closure_path").as_string();
            closure_path = PackageSharePath(urdf_pkg, rel_closure);
          } else {
            // <stem>.closure.yaml sibling — rtc_urdf_bridge owns the convention.
            closure_path = rtc_urdf_bridge::DeriveClosureSidecarPath(urdf_path);
          }

          if (std::filesystem::exists(closure_path)) {
            system_model_config_.closure_yaml_path = closure_path;
            RCLCPP_INFO(get_logger(), "Extended-URDF closure sidecar: %s", closure_path.c_str());
          } else {
            RCLCPP_ERROR(get_logger(),
                         "urdf.extended=true but closure sidecar not found (%s) — RT model "
                         "will LACK loop constraints (closed-chain dynamics will be wrong). "
                         "Fix the path/install the sidecar, or set urdf.extended=false.",
                         closure_path.c_str());
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to resolve Extended-URDF closure config (%s) — continuing "
                     "without loop constraints",
                     e.what());
      }
    }

    // ── Build the shared PinocchioModelBuilder once (both branches) ──────────
    // Built AFTER closure resolution so closure_yaml_path (when set) is bound
    // into the RT model. Shared with every registered controller — each would
    // otherwise re-run the same xacro → tinyxml2 → Pinocchio pipeline against an
    // identical ModelConfig (4× URDF parses on a 3-controller bring-up). Failure
    // is non-fatal: controllers fall back to constructing their own builder from
    // GetSystemModelConfig().
    if (!urdf_path.empty()) {
      try {
        shared_builder =
            std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(system_model_config_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(),
                    "Shared PinocchioModelBuilder build failed (%s) — "
                    "controllers will build their own",
                    e.what());
      }

      if (shared_builder) {
        const auto& analyzer = shared_builder->GetAnalyzer();
        RCLCPP_INFO(get_logger(),
                    "System URDF: %s (%zu sub_models, %zu tree_models, "
                    "%zu yaml-passive_joints; %zu <mimic> tags auto-locked by "
                    "PinocchioModelBuilder, %zu transmission-less passive)",
                    urdf_path.c_str(), system_model_config_.sub_models.size(),
                    system_model_config_.tree_models.size(),
                    system_model_config_.passive_joints.size(), analyzer.GetMimicJoints().size(),
                    analyzer.GetPassiveJoints().size());
      } else {
        RCLCPP_INFO(get_logger(),
                    "System URDF: %s (%zu sub_models, %zu tree_models, %zu "
                    "yaml-passive_joints)",
                    urdf_path.c_str(), system_model_config_.sub_models.size(),
                    system_model_config_.tree_models.size(),
                    system_model_config_.passive_joints.size());
      }
    }
  }

  // ── Instantiate and configure all registered controllers ─────────────────
  //
  // 3-pass bring-up so device-name configs are resolved BEFORE controllers'
  // on_configure runs:
  //   Pass 1: factory + per-controller LifecycleNode + PreConfigure
  //           (loads yaml → topic_config_; no RegisterLog yet)
  //   Pass 2: collect topic_config_ → active_groups_/group_slot_map_;
  //           LoadDeviceNameConfigs(); SetDeviceNameConfigs() per controller
  //           (triggers OnDeviceConfigsSet → joint_names captured)
  //   Pass 3: on_configure() per controller — RegisterLog now sees populated
  //           name spans, so CSV headers match data row widths.
  const auto& entries = urtc::ControllerRegistry::Instance().GetEntries();

  // Per-controller yaml node retained until Pass 3.
  std::vector<YAML::Node> controller_yamls;
  controller_yamls.reserve(entries.size());

  // Any bring-up error latches here and refuses the whole configure at the
  // next checkpoint (D1). Errors are collected rather than short-circuited so
  // one run reports every broken controller instead of only the first.
  bool bring_up_failed = false;

  // A previous configure attempt that failed leaves these populated; clearing
  // makes the bring-up idempotent so a retry cannot register a controller
  // twice. No-op on the first call.
  ResetControllerBringUpState();

  // ── Duplicate config_key scan (issue #196 Phase 5) ───────────────────────
  //
  // Two RTC_REGISTER_CONTROLLER macros claiming the same key is a packaging
  // mistake: CM looks controllers up by key, so the second registration
  // shadows the first and the operator silently runs code they did not
  // select. ControllerRegistry::Register() only warns — it cannot throw,
  // because static-init ordering across TUs makes a throwing registrar
  // fragile (see controller_registry.cpp).
  //
  // This runs *before* the Pass 1 loop rather than inside it: refusing after
  // the first factory() call would already have constructed a controller and
  // its LifecycleNode for a bring-up that can never be correct. Every
  // duplicate is reported in one run, then the whole configure is refused.
  {
    std::unordered_map<std::string, std::size_t> first_seen;
    first_seen.reserve(entries.size());
    bool duplicate_found = false;
    for (std::size_t i = 0; i < entries.size(); ++i) {
      const auto [it, inserted] = first_seen.emplace(entries[i].config_key, i);
      if (!inserted) {
        duplicate_found = true;
        RCLCPP_FATAL(get_logger(),
                     "Duplicate controller config_key '%s': registration #%zu "
                     "(package='%s') collides with #%zu (package='%s').",
                     entries[i].config_key.c_str(), i, entries[i].config_package.c_str(),
                     it->second, entries[it->second].config_package.c_str());
      }
    }
    if (duplicate_found) {
      RCLCPP_FATAL(get_logger(),
                   "Controller registry contains duplicate config_key(s) — refusing to "
                   "configure. Make each RTC_REGISTER_CONTROLLER key unique across all "
                   "linked TUs; no controller was instantiated.");
      return false;
    }
  }

  // ── Pass 1: instantiate, build LifecycleNode, PreConfigure ───────────────
  for (std::size_t i = 0; i < entries.size(); ++i) {
    const auto& entry = entries[i];
    auto ctrl = entry.factory(urdf_path);

    if (!system_model_config_.urdf_path.empty()) {
      ctrl->SetSystemModelConfig(system_model_config_);
    }
    if (shared_builder) {
      ctrl->SetSharedModelBuilder(shared_builder);
    }
    // Control rate is a property of THIS NODE, not of a bring-up pass, and it is
    // already parsed by the time we get here — so hand it over before anything
    // reads it. PreConfigure below calls the controller's LoadConfig, and several
    // controllers validate rate-dependent YAML there (filter cutoffs against
    // Nyquist). Setting the rate only in Pass 2 left those checks running against
    // GetDefaultDt()'s fallback, which rejects cutoffs that are perfectly legal at
    // the configured rate (kMaxControlRateHz is 5000) and fails the whole
    // configure. Pass 2 still re-sends it, harmlessly, next to the device configs.
    ctrl->SetControlRate(control_rate_);

    // Load YAML for this controller.
    //
    // Fail-closed with one deliberate exception (issue #196, decision D2): a
    // *missing* config file keeps the historical behaviour — empty node, so
    // PreConfigure / on_configure fall through to built-in defaults, which is
    // how controllers without a per-variant YAML are meant to run. A file that
    // exists but cannot be read (parse error, bad override, unknown package)
    // is a real config error and refuses the whole configure instead of
    // silently running a controller with defaults the operator never asked
    // for.
    //
    // config_variant gates the whole config tree (system YAML + controllers +
    // mujoco + shared).  When non-empty, path becomes
    // <pkg>/config/<variant>/controllers/<subdir><key>.yaml — `subdir` is the
    // registration-time config_subdir (e.g. "direct/" / "indirect/" for torque
    // vs. position groups within the same variant).  Empty variant → legacy
    // flat layout (no per-variant directory).  Hoisted above the try block so
    // the post-construction LifecycleNode declare_parameter call can re-use it.
    const std::string variant = get_parameter("config_variant").as_string();

    YAML::Node ctrl_node;
    try {
      const std::string pkg_dir =
          ament_index_cpp::get_package_share_directory(entry.config_package);
      std::string yaml_path;
      yaml_path.append(pkg_dir).append("/config/");
      if (!variant.empty()) {
        yaml_path.append(variant).append("/");
      }
      yaml_path.append("controllers/").append(entry.config_subdir);
      yaml_path.append(entry.config_key).append(".yaml");
      YAML::Node file_node = YAML::LoadFile(yaml_path);
      ctrl_node = file_node[entry.config_key];
      ApplyControllerParamOverrides(*this, ctrl_node, entry.config_key);
    } catch (const YAML::BadFile& e) {
      // No config file for this controller/variant — run on defaults.
      // BadFile must be caught before std::exception: YAML::Exception derives
      // from std::runtime_error, so the order below is what separates
      // "absent" from "present but broken".
      RCLCPP_INFO(get_logger(), "No config file for '%s' (pkg=%s) — using built-in defaults",
                  ctrl->Name().data(), entry.config_package.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Config load failed for '%s' (pkg=%s): %s", ctrl->Name().data(),
                   entry.config_package.c_str(), e.what());
      bring_up_failed = true;
    }

    // Create a dedicated LifecycleNode per controller.  Namespace is
    // "/<config_key>" so controller-owned topics declared with relative
    // paths in YAML resolve to /<config_key>/<topic> automatically.
    // main() attaches these to nrt_callback_executor.
    // Per-controller LifecycleNodes must NOT inherit the parent's CLI
    // remappings (e.g. `--ros-args -r __node:=robot_rt_controller`).  Without
    // use_global_arguments(false), the global remap rules rename every child
    // to the parent's node name and namespace, which clobbers the intended
    // /<config_key> namespacing and produces one
    // `rcl.logging_rosout: Publisher already registered` warning per child.
    const std::string ctrl_ns = "/" + entry.config_key;
    const auto ctrl_node_options = rclcpp::NodeOptions().use_global_arguments(false);
    auto ctrl_lc_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(entry.config_key, ctrl_ns,
                                                                          ctrl_node_options);

    // Propagate config_variant to the per-controller LifecycleNode so
    // controllers can pass it to LoadDemoSharedYamlFile() without re-declaring
    // it themselves.  Reads back the CM's current value.
    ctrl_lc_node->declare_parameter<std::string>("config_variant", variant);

    // PreConfigure: stores node_ + parses yaml so GetTopicConfig() is valid
    // before Pass 2 builds active_groups_. No RegisterLog or resource
    // allocation here.
    const auto pre_ret = ctrl->PreConfigure(ctrl_lc_node, ctrl_node);
    if (pre_ret != urtc::RTControllerInterface::CallbackReturn::SUCCESS) {
      // Was a warning that continued with defaults, which is how a controller
      // whose config never parsed still reached the active/switch candidate
      // set and RT Compute() (issue #196 §1).
      RCLCPP_ERROR(get_logger(), "Controller '%s' PreConfigure failed", ctrl->Name().data());
      bring_up_failed = true;
    }

    // ── Tier 2 of the identifier-collision guard (issue #196 Phase 5) ──────
    //
    // controller_name_to_idx_ is a SINGLE namespace holding both Name() and
    // config_key, so switch_controller / initial_controller resolve through
    // either. A collision in Name(), or across Name() and some other
    // controller's config_key, shadows exactly like a duplicate config_key
    // does — the operator silently runs a controller they did not select.
    //
    // Why this cannot join the Tier 1 scan above: ControllerEntry carries no
    // name, and Name() is a virtual on the instance, so it is unknowable
    // until factory() has run. Tier 2 therefore latches bring_up_failed and
    // lets the D1 checkpoint refuse. Controllers and their LifecycleNodes do
    // exist by then, but no publisher, device backend, log channel, service,
    // timer or RT thread has been created — those all live past D1.
    // emplace, not operator[]: on collision the FIRST claimant keeps the slot,
    // so the map is fail-safe on its own rather than relying on D1 being the
    // next reader. (An unconditional write here left the map holding exactly
    // the shadowing the FATAL below warns about — harmless only while no read
    // sits between here and D1, which nothing enforces. Tier 1 above is
    // structured the same way.) The loop still runs to completion so one run
    // reports every collision.
    const std::string ctrl_name(ctrl->Name());
    auto claim = [&](const std::string& id) {
      const auto [prev, inserted] = controller_name_to_idx_.emplace(id, static_cast<int>(i));
      if (inserted) {
        return;
      }
      bring_up_failed = true;
      RCLCPP_FATAL(get_logger(),
                   "Controller identifier '%s' is already claimed by registration #%d "
                   "(config_key='%s'); #%zu (config_key='%s', Name()='%s') would shadow "
                   "it in the switch_controller / initial_controller namespace.",
                   id.c_str(), prev->second,
                   entries[static_cast<std::size_t>(prev->second)].config_key.c_str(), i,
                   entry.config_key.c_str(), ctrl_name.c_str());
    };
    claim(ctrl_name);
    // Name() == config_key is legal (and common in tests). Claiming it twice
    // would make the entry collide with itself.
    if (entry.config_key != ctrl_name) {
      claim(entry.config_key);
    }

    controllers_.push_back(std::move(ctrl));
    controller_nodes_.push_back(std::move(ctrl_lc_node));
    controller_types_.push_back(entry.config_key);
    controller_yamls.push_back(ctrl_node);
  }

  // Checkpoint before Pass 2/3 (D1): refuse here rather than after
  // on_configure, so a doomed bring-up never creates publishers,
  // subscriptions, or log channels. CM's on_configure turns this into a
  // lifecycle FAILURE, so the RT thread and device backends never start.
  if (bring_up_failed) {
    RCLCPP_FATAL(get_logger(),
                 "Controller bring-up failed — refusing to configure. Fix the "
                 "controller config(s) reported above; no controller is registered.");
    ResetControllerBringUpState();
    return false;
  }

  // Cache per-controller topic configs and build active_groups_ +
  // group_slot_map_
  controller_topic_configs_.reserve(controllers_.size());
  for (const auto& ctrl : controllers_) {
    controller_topic_configs_.push_back(ctrl->GetTopicConfig());

    const auto& tc = controller_topic_configs_.back();
    std::string groups_info;
    for (const auto& [name, group] : tc.groups) {
      if (!group.subscribe.empty() || !group.publish.empty()) {
        groups_info += name + "(" + std::to_string(group.subscribe.size()) + "sub+" +
                       std::to_string(group.publish.size()) + "pub) ";
      }
    }
    RCLCPP_INFO(get_logger(), "Controller '%s': %s", ctrl->Name().data(), groups_info.c_str());
  }

  // ── Build active_groups_ (union of all controllers' groups) ──────────────
  {
    int slot_idx = 0;
    for (const auto& tc : controller_topic_configs_) {
      for (const auto& [name, group] : tc.groups) {
        if (!group.subscribe.empty() || !group.publish.empty()) {
          if (active_groups_.insert(name).second) {
            group_slot_map_[name] = slot_idx++;
          }
        }
      }
    }
    if (slot_idx > urtc::PublishSnapshot::kMaxGroups) {
      RCLCPP_FATAL(get_logger(), "Too many device groups (%d > %d)", slot_idx,
                   urtc::PublishSnapshot::kMaxGroups);
      rclcpp::shutdown();
      // Previously this returned from a void function and the caller carried
      // on building publishers/backends while the shutdown was still pending.
      // Reporting the failure stops configure at the caller too.
      ResetControllerBringUpState();
      return false;
    }
  }

  // ── Load device name configs (needs active_groups_) ─────────────────────
  if (!LoadDeviceNameConfigs()) {
    RCLCPP_FATAL(get_logger(),
                 "Device config exceeds the RT path's fixed capacity — refusing to configure.");
    ResetControllerBringUpState();
    return false;
  }

  if (!BuildControllerSlotMappings()) {
    ResetControllerBringUpState();
    return false;
  }

  // Pass device configs to all controllers. The control rate went out in Pass 1
  // (PreConfigure's LoadConfig already needs it); re-sending it is idempotent and
  // keeps this loop correct for any controller added to controllers_ later.
  for (auto& ctrl : controllers_) {
    ctrl->SetControlRate(control_rate_);
    ctrl->SetDeviceNameConfigs(device_name_configs_);
  }

  // ── Pass 3: drive on_configure now that device-name configs are resolved ─
  // RegisterLog calls inside controllers' on_configure can capture populated
  // joint_name / motor_name / sensor_name spans, so the CSV header writer
  // emits a row that matches the runtime data row width.
  {
    const rclcpp_lifecycle::State unconfigured_state(
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    for (std::size_t ci = 0; ci < controllers_.size(); ++ci) {
      const auto cfg_ret = controllers_[ci]->on_configure(unconfigured_state, controller_nodes_[ci],
                                                          controller_yamls[ci]);
      if (cfg_ret != urtc::RTControllerInterface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Controller '%s' on_configure failed",
                     controllers_[ci]->Name().data());
        bring_up_failed = true;
      }
    }
    if (bring_up_failed) {
      RCLCPP_FATAL(get_logger(),
                   "Controller on_configure failed — refusing to configure. "
                   "No controller is registered as an active/switch candidate.");
      ResetControllerBringUpState();
      return false;
    }
  }

  // ── Deferred logging setup ───────────────────────────────────────────────
  // Phase C: CM only owns cm_timing_log.csv (its own per-tick scheduling
  // timing). Controller data CSVs are produced by each controller's own
  // ControllerLogSet under <session>/controllers/<config_key>/. The legacy
  // CM-side DataLogger and `<session>/controller/` directory have been
  // removed.
  if (enable_logging_) {
    const std::string log_dir_param = get_parameter("log_dir").as_string();
    const int max_sessions = static_cast<int>(get_parameter("max_log_sessions").as_int());

    std::filesystem::path session_dir;
    if (!log_dir_param.empty()) {
      session_dir = std::filesystem::path(log_dir_param);
      std::filesystem::create_directories(session_dir);
      urtc::EnsureSessionSubdirs(session_dir);
    } else {
      session_dir = ResolveAndSetupSessionDir();
    }
    const auto logging_root = session_dir.parent_path();
    urtc::CleanupOldSessions(logging_root, max_sessions);

    const bool enable_timing = get_parameter("enable_timing_log").as_bool();
    if (enable_timing) {
      const auto timing_dir = urtc::TimingDir(session_dir);
      std::filesystem::create_directories(timing_dir);
      const auto timing_path = timing_dir / "cm_timing_log.csv";
      if (!cm_timing_logger_.Open(timing_path, &urtc::WriteRtTickTimingHeader,
                                  &urtc::WriteRtTickTimingRow)) {
        RCLCPP_WARN(get_logger(),
                    "Failed to open cm_timing_log.csv at %s — per-tick timing "
                    "CSV disabled",
                    timing_path.string().c_str());
      }
    }

    RCLCPP_INFO(get_logger(), "Session dir: %s (max_sessions=%d)", session_dir.string().c_str(),
                max_sessions);
  }

  // ── Device liveness entries: one per configured device group ─────────────
  //
  // The set is driven by group_slot_map_ (the groups that get a backend), not
  // by device_timeout_names. That list used to decide *which devices are
  // watched at all*, so omitting a name silently exempted that device from
  // both the watchdog and the startup gate (issue #198 §1). It now supplies
  // the timeout *value* for a group; a group it does not mention falls back to
  // device_timeout_default_ms and is still watched.
  {
    const auto timeout_names = get_parameter("device_timeout_names").as_string_array();
    const auto timeout_values = get_parameter("device_timeout_values").as_double_array();
    const double default_timeout_ms = get_parameter("device_timeout_default_ms").as_double();

    std::unordered_map<std::string, double> timeout_by_group;
    for (std::size_t i = 0; i < timeout_names.size(); ++i) {
      if (i >= timeout_values.size()) {
        RCLCPP_WARN(get_logger(),
                    "device_timeout_names['%s'] has no matching value — using %.0f ms",
                    timeout_names[i].c_str(), default_timeout_ms);
        continue;
      }
      if (!group_slot_map_.contains(timeout_names[i])) {
        RCLCPP_WARN(get_logger(), "Device timeout '%s' has no matching device group — ignored",
                    timeout_names[i].c_str());
        continue;
      }
      timeout_by_group[timeout_names[i]] = timeout_values[i];
    }

    for (const auto& [group_name, slot] : group_slot_map_) {
      // Phase 4: state lane is owned by DeviceBackend — resolved from
      // devices.<group>.backend.state_topic (DeviceNameConfig::backend). A
      // group with no backend block still gets an entry: its backend will fail
      // to come up, and the startup gate must refuse rather than quietly drive
      // a device that has no driver.
      std::string state_topic;
      if (auto it = device_name_configs_.find(group_name);
          it != device_name_configs_.end() && it->second.backend.has_value()) {
        state_topic = it->second.backend->state_topic;
      }

      const auto it = timeout_by_group.find(group_name);
      const double timeout_ms = it != timeout_by_group.end() ? it->second : default_timeout_ms;
      if (it == timeout_by_group.end()) {
        RCLCPP_WARN(get_logger(),
                    "Device group '%s' is not listed in device_timeout_names — watching it at the "
                    "default %.0f ms",
                    group_name.c_str(), default_timeout_ms);
      }

      DeviceTimeoutEntry entry;
      entry.group_name = group_name;
      entry.state_topic = state_topic;
      entry.slot = slot;
      entry.timeout = std::chrono::milliseconds(static_cast<int>(timeout_ms));
      device_timeouts_.push_back(std::move(entry));
      RCLCPP_INFO(get_logger(), "Device liveness: '%s' (slot %d) → watching '%s' (%dms)",
                  group_name.c_str(), slot,
                  state_topic.empty() ? "<no backend state topic>" : state_topic.c_str(),
                  static_cast<int>(timeout_ms));
    }
  }

  // No device groups at all (e.g. a controller that owns no devices) — there
  // is nothing to wait for, and FirstUnreadyDevice() opens the gate on the
  // first tick. This is the ONLY remaining path that skips the init wait; it
  // used to also fire whenever device_timeout_names was empty, which turned a
  // missing YAML line into "start immediately, watch nothing".
  if (device_timeouts_.empty()) {
    RCLCPP_INFO(get_logger(), "No device groups configured — no startup readiness gate");
  }

  // Resolve initial_controller parameter → controller index.
  // Empty or unknown name: fall back to the first registered controller
  // (idx 0). rtc_* stays robot-agnostic — it does not assume any specific
  // controller name is registered (ARCH-1).
  const std::string initial_ctrl = get_parameter("initial_controller").as_string();
  const auto it = controller_name_to_idx_.find(initial_ctrl);
  if (it != controller_name_to_idx_.end()) {
    active_controller_idx_.store(it->second);
  } else {
    const std::string fallback_name =
        controllers_.empty() ? std::string{"<none>"} : std::string(controllers_.front()->Name());
    RCLCPP_WARN(get_logger(),
                "initial_controller '%s' not found — defaulting to first "
                "registered controller '%s'",
                initial_ctrl.c_str(), fallback_name.c_str());
    active_controller_idx_.store(0);
  }

  // No controller registered means there is no valid active candidate, and
  // active_controller_idx_ = 0 would index empty vectors — CM's on_configure
  // publishes controller_types_[active] before it returns. Refuse instead of
  // reporting a configured node with nothing to run.
  if (controllers_.empty()) {
    RCLCPP_FATAL(get_logger(), "No controller registered — refusing to configure.");
    return false;
  }

  return true;
}
