// custom_controller.cpp - v4 (E-STOP Support)
#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include "ur5e_rt_controller/controllers/pd_controller.hpp"
#include "ur5e_rt_controller/data_logger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ur_msgs/msg/io_states.hpp>

#include <chrono>
#include <memory>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class CustomController : public rclcpp::Node {
public:
  CustomController()
    : Node("custom_controller"),
      controller_(std::make_unique<ur5e_controller::PDController>(5.0, 0.5)),
      logger_(std::make_unique<ur5e_controller::DataLogger>("/tmp/ur5e_control_log.csv"))
  {
    // Parameters (v4: E-STOP parameters)
    this->declare_parameter("control_rate", 500.0);
    this->declare_parameter("kp", 5.0);
    this->declare_parameter("kd", 0.5);
    this->declare_parameter("enable_logging", true);
    this->declare_parameter("robot_timeout_ms", 100.0);  // v4
    this->declare_parameter("hand_timeout_ms", 200.0);   // v4
    this->declare_parameter("enable_estop", true);       // v4
    
    control_rate_ = this->get_parameter("control_rate").as_double();
    enable_logging_ = this->get_parameter("enable_logging").as_bool();
    robot_timeout_ = std::chrono::milliseconds(
        static_cast<int>(this->get_parameter("robot_timeout_ms").as_double()));
    hand_timeout_ = std::chrono::milliseconds(
        static_cast<int>(this->get_parameter("hand_timeout_ms").as_double()));
    enable_estop_ = this->get_parameter("enable_estop").as_bool();
    
    // Subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&CustomController::joint_state_callback, this, std::placeholders::_1));
    
    target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_joint_positions", 10,
        std::bind(&CustomController::target_callback, this, std::placeholders::_1));
    
    hand_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", 10,
        std::bind(&CustomController::hand_state_callback, this, std::placeholders::_1));
    
    // Publishers
    command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands", 10);
    
    // v4: E-STOP status publisher
    estop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/system/estop_status", 10);
    
    // Control timer
    auto control_period = std::chrono::microseconds(
        static_cast<int>(1000000.0 / control_rate_));
    control_timer_ = this->create_wall_timer(
        control_period,
        std::bind(&CustomController::control_loop, this));
    
    // v4: Timeout monitor timer (50Hz)
    if (enable_estop_) {
      timeout_timer_ = this->create_wall_timer(
          20ms,
          std::bind(&CustomController::check_timeouts, this));
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Custom Controller initialized (v4 - E-STOP)");
    RCLCPP_INFO(this->get_logger(), 
                "Control rate: %.1f Hz", control_rate_);
    RCLCPP_INFO(this->get_logger(), 
                "E-STOP enabled: %s", enable_estop_ ? "true" : "false");
  }
  
  ~CustomController() {
    if (logger_) {
      logger_->flush();
    }
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_positions_.assign(msg->position.begin(), 
                               msg->position.begin() + 6);
      current_velocities_.assign(msg->velocity.begin(), 
                                 msg->velocity.begin() + 6);
      last_robot_update_ = this->now();
      state_received_ = true;
    }
  }
  
  void target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 6) {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target_positions_.assign(msg->data.begin(), 
                              msg->data.begin() + 6);
      target_received_ = true;
    }
  }
  
  void hand_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(hand_mutex_);
    last_hand_update_ = this->now();
    hand_data_received_ = true;
  }
  
  // v4: Timeout monitoring
  void check_timeouts() {
    if (!enable_estop_) return;
    
    auto now = this->now();
    bool robot_timeout = false;
    bool hand_timeout = false;
    
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_received_) {
        auto elapsed = now - last_robot_update_;
        robot_timeout = (elapsed > robot_timeout_);
      }
    }
    
    {
      std::lock_guard<std::mutex> lock(hand_mutex_);
      if (hand_data_received_) {
        auto elapsed = now - last_hand_update_;
        hand_timeout = (elapsed > hand_timeout_);
      }
    }
    
    // Trigger E-STOP on timeout
    if (robot_timeout && !controller_->is_estopped()) {
      RCLCPP_ERROR(this->get_logger(), "Robot data timeout - triggering E-STOP");
      controller_->trigger_estop();
      publish_estop_status(true);
    }
    
    if (hand_timeout) {
      controller_->set_hand_estop(true);
      if (!hand_estop_logged_) {
        RCLCPP_WARN(this->get_logger(), "Hand data timeout - hand E-STOP active");
        hand_estop_logged_ = true;
      }
    } else {
      if (hand_estop_logged_) {
        RCLCPP_INFO(this->get_logger(), "Hand data restored - hand E-STOP cleared");
        hand_estop_logged_ = false;
      }
      controller_->set_hand_estop(false);
    }
  }
  
  void control_loop() {
    if (!state_received_ || !target_received_) {
      return;
    }
    
    std::vector<double> current_pos, current_vel, target_pos;
    
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_pos = current_positions_;
      current_vel = current_velocities_;
    }
    
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target_pos = target_positions_;
    }
    
    // v4: Check E-STOP status
    if (controller_->is_estopped()) {
      // Send zero velocity commands
      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data.assign(6, 0.0);
      command_pub_->publish(msg);
      return;
    }
    
    // Compute control command
    double dt = 1.0 / control_rate_;
    auto commands = controller_->compute_command(
        current_pos, current_vel, target_pos, dt);
    
    // Publish command
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = commands;
    command_pub_->publish(msg);
    
    // Logging
    if (enable_logging_ && logger_) {
      double timestamp = this->now().seconds();
      logger_->log_control_data(timestamp, current_pos, target_pos, commands);
    }
    
    loop_count_++;
    
    if (loop_count_ % 500 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Control loop running: %zu iterations", loop_count_);
    }
  }
  
  // v4: Publish E-STOP status
  void publish_estop_status(bool estopped) {
    auto msg = std_msgs::msg::Bool();
    msg.data = estopped;
    estop_pub_->publish(msg);
  }

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hand_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;  // v4
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;  // v4
  
  // Controller and logger
  std::unique_ptr<ur5e_controller::PDController> controller_;
  std::unique_ptr<ur5e_controller::DataLogger> logger_;
  
  // State variables
  std::vector<double> current_positions_;
  std::vector<double> current_velocities_;
  std::vector<double> target_positions_;
  
  std::mutex state_mutex_;
  std::mutex target_mutex_;
  std::mutex hand_mutex_;
  
  bool state_received_{false};
  bool target_received_{false};
  bool hand_data_received_{false};
  
  // v4: Timeout tracking
  rclcpp::Time last_robot_update_;
  rclcpp::Time last_hand_update_;
  std::chrono::milliseconds robot_timeout_;
  std::chrono::milliseconds hand_timeout_;
  bool enable_estop_{true};
  bool hand_estop_logged_{false};
  
  // Parameters
  double control_rate_;
  bool enable_logging_;
  size_t loop_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
