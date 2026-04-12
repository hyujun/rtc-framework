#include "rtc_tsid/formulation/formulation_factory.hpp"

#include <stdexcept>

#include "rtc_tsid/formulation/wqp_formulation.hpp"
#include "rtc_tsid/formulation/hqp_formulation.hpp"

namespace rtc::tsid {

std::unique_ptr<FormulationBase> create_formulation(
    const pinocchio::Model& model,
    const RobotModelInfo& robot_info,
    const ContactManagerConfig& contact_cfg,
    const YAML::Node& config) {
  std::string type = "wqp";
  if (config && config["formulation_type"]) {
    type = config["formulation_type"].as<std::string>();
  }

  std::unique_ptr<FormulationBase> formulation;

  if (type == "wqp") {
    auto wqp = std::make_unique<WQPFormulation>();
    wqp->init(model, robot_info, contact_cfg, config);
    formulation = std::move(wqp);
  } else if (type == "hqp") {
    auto hqp = std::make_unique<HQPFormulation>();
    hqp->init(model, robot_info, contact_cfg, config);
    formulation = std::move(hqp);
  } else {
    throw std::runtime_error(
        "Unknown formulation_type: '" + type + "'. Use 'wqp' or 'hqp'.");
  }

  return formulation;
}

}  // namespace rtc::tsid
