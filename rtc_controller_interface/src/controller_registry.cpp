#include "rtc_controller_interface/controller_registry.hpp"

namespace rtc
{

ControllerRegistry & ControllerRegistry::Instance() noexcept
{
  static ControllerRegistry instance;
  return instance;
}

void ControllerRegistry::Register(ControllerEntry entry)
{
  entries_.push_back(std::move(entry));
}

}  // namespace rtc
