#ifndef RTC_CONTROLLER_MANAGER_RT_CONTROLLER_MAIN_HPP_
#define RTC_CONTROLLER_MANAGER_RT_CONTROLLER_MAIN_HPP_

#include <string>

namespace rtc {

// Reusable entry-point logic shared by all robot-specific executables.
// Call ForceXxxRegistration() functions BEFORE calling this.
//
// `node_name` is required: rtc_controller_manager is robot-agnostic and does
// not own a runtime ROS node identity. The robot-specific bringup executable
// (e.g. ur5e_rt_controller) supplies a name that matches its own executable
// so that exec / node / pgrep / log identifiers stay aligned.
int RtControllerMain(int argc, char **argv, const std::string &node_name);

} // namespace rtc

#endif // RTC_CONTROLLER_MANAGER_RT_CONTROLLER_MAIN_HPP_
