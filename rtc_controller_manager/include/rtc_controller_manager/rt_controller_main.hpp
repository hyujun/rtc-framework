#ifndef RTC_CONTROLLER_MANAGER_RT_CONTROLLER_MAIN_HPP_
#define RTC_CONTROLLER_MANAGER_RT_CONTROLLER_MAIN_HPP_

namespace rtc
{

// Reusable entry-point logic shared by all robot-specific executables.
// Call ForceXxxRegistration() functions BEFORE calling this.
int RtControllerMain(int argc, char ** argv);

}  // namespace rtc

#endif  // RTC_CONTROLLER_MANAGER_RT_CONTROLLER_MAIN_HPP_
