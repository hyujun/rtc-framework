// ── UR5e demo controller registration ─────────────────────────────────────────
//
// Registers DemoJointController and DemoTaskController into the global
// ControllerRegistry.  The library is linked with --whole-archive so the
// linker preserves this translation unit without a Force function.

#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_inference/onnx/onnx_engine.hpp"

#include <memory>

RTC_REGISTER_CONTROLLER(demo_joint_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoJointController>(urdf))

RTC_REGISTER_CONTROLLER(demo_task_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoTaskController>(
                            urdf, integrated_bringup::DemoTaskController::Gains{}))

RTC_REGISTER_CONTROLLER(demo_wbc_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoWbcController>(urdf))

// #469 S2. The config_key here and DemoComplianceController::Name() land in the
// SAME lookup namespace as every other entry above, and a collision refuses the
// whole bring-up rather than warning — which is the guard that catches a copied
// controller whose identifiers were not fully renamed.
RTC_REGISTER_CONTROLLER(demo_compliance_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoComplianceController>(
                            urdf, integrated_bringup::DemoComplianceController::Gains{}))

// The engine is a constructor argument so tests can drive every success path
// with a deterministic fake — there is no policy file yet, and even once there
// is, a unit test should not need one. Production gets the real ONNX engine
// here; when ONNX Runtime was absent at build time this same name resolves to
// the stub, whose Init() is a silent no-op, which is why the controller gates
// on `is_initialized()` rather than trusting configure to have loaded anything.
//
// REQUIRING_CONFIG, unlike every other controller here: LoadConfig refuses an
// absent config node (inference/parameters.cpp) because a policy controller has
// no defensible defaults — no model path, no IO schema, no policy frame. That
// refusal is correct and stays, but as a plain registration it also meant any
// robot shipping no demo_inference_controller.yaml failed the whole bring-up,
// taking the other four controllers down with it (ur5e_p1a and iiwa7_leap both
// did). The flag makes "this robot ships no policy config" mean "this robot
// does not run the policy controller", which is what the GUI roster in
// demo_gui/discovery.py has always assumed.
RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG(
    demo_inference_controller, "", "integrated_bringup",
    std::make_unique<integrated_bringup::DemoInferenceController>(
        urdf, std::make_unique<rtc::OnnxEngine>()))

// dynamic_catching S4.0 — the catching controller's minimal skeleton (arm hold
// + unshaped hand step). REQUIRING_CONFIG for two reasons: the hand profile
// (L6 §6) has no defensible default, so LoadConfig refuses an absent config
// node, and ur5e_p1a is not a catching target and ships no YAML for it.
RTC_REGISTER_CONTROLLER_REQUIRING_CONFIG(
    demo_catching_controller, "", "integrated_bringup",
    std::make_unique<integrated_bringup::DemoCatchingController>(urdf))
