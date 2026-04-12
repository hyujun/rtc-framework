#pragma once

#include <memory>

#include "rtc_tsid/core/formulation_base.hpp"

namespace rtc::tsid {

// YAML config의 formulation_type에 따라 WQP 또는 HQP 인스턴스 생성
// "wqp" → WQPFormulation, "hqp" → HQPFormulation
std::unique_ptr<FormulationBase> create_formulation(
    const pinocchio::Model& model,
    const RobotModelInfo& robot_info,
    const ContactManagerConfig& contact_cfg,
    const YAML::Node& config);

}  // namespace rtc::tsid
