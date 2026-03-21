#ifndef RTC_CONTROLLER_INTERFACE_CONTROLLER_TYPES_H_
#define RTC_CONTROLLER_INTERFACE_CONTROLLER_TYPES_H_

// Re-export controller-specific types from rtc_base.
// This header exists as a convenience for downstream packages that need
// the types used by RTControllerInterface (ControllerOutput, ControllerState,
// CommandType, TopicConfig, etc.) without pulling in the full interface header.

#include "rtc_base/types/types.hpp"

#endif  // RTC_CONTROLLER_INTERFACE_CONTROLLER_TYPES_H_
