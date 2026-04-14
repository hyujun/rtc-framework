// ── Core lifecycle: constructor, destructor, callback groups, timers ──────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rtc_base/logging/session_dir.hpp>

#include <sys/eventfd.h>
#include <unistd.h>  // close

using namespace std::chrono_literals;
namespace urtc = rtc;

// ── Controller registry ──────────────────────────────────────────────────────
//
// To add a new controller:
//   1. Implement RTControllerInterface in your package
//   2. Create a config/controllers/<subdir>/<key>.yaml
//   3. Use RTC_REGISTER_CONTROLLER() macro in a .cpp file
//   4. Link the final executable against your library
//
// See rtc_controllers/src/controller_registration.cpp for built-in examples.
// ─────────────────────────────────────────────────────────────────────────────

// ── Constructor / destructor ──────────────────────────────────────────────────
RtControllerNode::RtControllerNode()
: Node("rt_controller",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  logger_(nullptr)
{
  CreateCallbackGroups();
  DeclareAndLoadParameters();
  CreateSubscriptions();
  CreatePublishers();
  ExposeTopicParameters();
  CreateTimers();

  // eventfd for RT→publish thread wakeup (non-blocking to avoid RT stalls)
  publish_eventfd_ = eventfd(0, EFD_NONBLOCK);
  if (publish_eventfd_ < 0) {
    RCLCPP_WARN(get_logger(),
        "eventfd() failed: publish thread will use polling fallback");
  }

  // Publish initial active controller name (transient_local so late subscribers receive it)
  {
    std_msgs::msg::String ctrl_name_msg;
    ctrl_name_msg.data = std::string(
      controllers_[static_cast<std::size_t>(
          active_controller_idx_.load(std::memory_order_acquire))]->Name());
    active_ctrl_name_pub_->publish(ctrl_name_msg);
  }

  RCLCPP_INFO(get_logger(), "RtControllerNode ready — %.0f Hz, E-STOP: %s",
              control_rate_, enable_estop_ ? "ON" : "OFF");
  RCLCPP_INFO(get_logger(),
      "Threading: clock_nanosleep RT loop + SPSC publish offload + "
      "Sensor/Log/Aux executors");
}

RtControllerNode::~RtControllerNode()
{
  // Stop RT loop + publish thread (idempotent — safe if already stopped by main)
  StopRtLoop();
  StopPublishLoop();

  if (publish_eventfd_ >= 0) {
    close(publish_eventfd_);
    publish_eventfd_ = -1;
  }

  if (logger_) {
    logger_->DrainBuffer(log_buffer_);
    logger_->Flush();
  }
}

// ── Session directory helpers ─────────────────────────────────────────────────
std::filesystem::path RtControllerNode::ResolveAndSetupSessionDir()
{
  return urtc::ResolveSessionDir();
}

// ── CallbackGroup creation ────────────────────────────────────────────────────
void RtControllerNode::CreateCallbackGroups()
{
  cb_group_sensor_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_log_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_aux_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void RtControllerNode::CreateTimers()
{
  // Drain the SPSC log ring buffer from the log thread (Core 4).
  // File I/O stays entirely out of the 500 Hz RT thread.
  static constexpr auto kLogDrainPeriod = 10ms;
  drain_timer_ =
    create_wall_timer(kLogDrainPeriod, [this]() {DrainLog();}, cb_group_log_);
}
