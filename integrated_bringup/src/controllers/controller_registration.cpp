// ── UR5e demo controller registration ─────────────────────────────────────────
//
// Registers DemoJointController and DemoTaskController into the global
// ControllerRegistry.  The library is linked with --whole-archive so the
// linker preserves this translation unit without a Force function.

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
RTC_REGISTER_CONTROLLER(demo_inference_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoInferenceController>(
                            urdf, std::make_unique<rtc::OnnxEngine>()))
