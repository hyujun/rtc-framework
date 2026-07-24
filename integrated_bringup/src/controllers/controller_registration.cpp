// ── UR5e demo controller registration ─────────────────────────────────────────
//
// Registers DemoJointController and DemoTaskController into the global
// ControllerRegistry.  The library is linked with --whole-archive so the
// linker preserves this translation unit without a Force function.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/demo_wbc_controller.hpp"
#include "rtc_controller_interface/controller_registry.hpp"
#include "rtc_controllers/direct/task_impedance_controller.hpp"

RTC_REGISTER_CONTROLLER(demo_joint_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoJointController>(urdf))

RTC_REGISTER_CONTROLLER(demo_task_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoTaskController>(
                            urdf, integrated_bringup::DemoTaskController::Gains{}))

RTC_REGISTER_CONTROLLER(demo_wbc_controller, "", "integrated_bringup",
                        std::make_unique<integrated_bringup::DemoWbcController>(urdf))

// Task-space impedance (§6.2 A=NONE, torque, MuJoCo-only). A robot-agnostic
// rtc_controllers controller registered directly (no integrated_bringup wrapper
// needed — it composes no hand/grasp/TF); FULL_SE3 by default, gains from
// config/<robot>/controllers/task_impedance_controller.yaml.
RTC_REGISTER_CONTROLLER(
    task_impedance_controller, "", "integrated_bringup",
    std::make_unique<rtc::TaskImpedanceController>(urdf, rtc::TaskImpedanceController::Gains{}))
