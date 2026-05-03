#include "ur5e_bringup/controllers/demo_wbc_controller.hpp"

#include "ur5e_bringup/controllers/owned_topics.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace ur5e_bringup {

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────
RTControllerInterface::CallbackReturn DemoWbcController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    CreateOwnedTopics(*this, owned_topics_);
    mpc_timing_cb_group_ =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ── Phase C: register controller-owned CSV log channels ─────────────
    for (const auto& entry : parsed_log_entries_) {
      if (entry.instance.empty()) {
        RCLCPP_ERROR(logger_, "logs entry msg_type=%s missing required `instance:` field",
                     entry.msg_type.c_str());
        return CallbackReturn::FAILURE;
      }
      if (entry.msg_type == "rtc_msgs/DeviceStateLog") {
        // Q-MSG-3: ur5e_state vs hand_state instance names.
        const bool is_hand = (entry.instance == "hand_state");
        const std::vector<std::string> joint_names_copy =
            is_hand ? hand_joint_names_ : ur5e_joint_names_;
        const std::vector<std::string> motor_names_copy =
            is_hand ? hand_motor_names_ : std::vector<std::string>{};
        auto handle = log_set_.RegisterLog<ur5e::DeviceStateLogPod>(
            entry.instance,
            [joint_names_copy, motor_names_copy](std::ostream& os) {
              ur5e::WriteDeviceStateLogHeader(os, joint_names_copy, motor_names_copy);
            },
            [](std::ostream& os, const ur5e::DeviceStateLogPod& p) {
              ur5e::WriteDeviceStateLogRow(os, p);
            });
        if (!handle) {
          RCLCPP_WARN(logger_, "Failed to open device_state CSV for instance=%s",
                      entry.instance.c_str());
        } else if (entry.instance == "ur5e_state") {
          ur5e_state_log_handle_ = handle;
        } else if (is_hand) {
          hand_state_log_handle_ = handle;
        }
      } else if (entry.msg_type == "rtc_msgs/DeviceSensorLog") {
        const std::vector<std::string> sensor_names_copy = hand_sensor_names_;
        auto handle = log_set_.RegisterLog<ur5e::DeviceSensorLogPod>(
            entry.instance,
            [sensor_names_copy](std::ostream& os) {
              ur5e::WriteDeviceSensorLogHeader(os, sensor_names_copy);
            },
            [](std::ostream& os, const ur5e::DeviceSensorLogPod& p) {
              ur5e::WriteDeviceSensorLogRow(os, p);
            });
        if (!handle) {
          RCLCPP_WARN(logger_, "Failed to open device_sensor CSV for instance=%s",
                      entry.instance.c_str());
        } else if (entry.instance == "hand_sensor") {
          hand_sensor_log_handle_ = handle;
        }
      }
    }
    if (!log_set_.empty() && node) {
      log_drain_cb_group_ =
          node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node->create_wall_timer(
          std::chrono::milliseconds(100), [this]() { log_set_.DrainAll(); }, log_drain_cb_group_);
    }

    // Phase D: declare tunable gains as ROS 2 parameters seeded from
    // gains_lock_ (populated by LoadConfig from YAML), register the
    // set-parameters callback, and create the WBC FSM grasp_command srv.
    DeclareGainParameters();
    param_callback_handle_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
          return OnGainParametersSet(params);
        });

    // Force-PI grasp_command srv: WBC has no grasp_controller_ — the srv
    // handler updates grasp_cmd_ atomic + gains.grasp_target_force, which
    // the WBC FSM (UpdatePhase) reads to drive kApproach/kRelease.
    grasp_command_srv_ = node_->create_service<rtc_msgs::srv::GraspCommand>(
        "grasp_command", [this](const std::shared_ptr<rtc_msgs::srv::GraspCommand::Request> req,
                                std::shared_ptr<rtc_msgs::srv::GraspCommand::Response> resp) {
          if (estopped_.load(std::memory_order_acquire)) {
            resp->ok = false;
            resp->message = "E-STOP active";
            return;
          }
          using Req = rtc_msgs::srv::GraspCommand::Request;
          if (req->command == Req::GRASP) {
            if (!(req->target_force > 0.0)) {
              resp->ok = false;
              resp->message = "GRASP requires target_force > 0";
              return;
            }
            auto g = gains_lock_.Load();
            g.grasp_target_force = req->target_force;
            gains_lock_.Store(g);
            grasp_cmd_.store(static_cast<int>(Req::GRASP), std::memory_order_release);
            RCLCPP_INFO(logger_, "[grasp_command] GRASP target=%.2fN (WBC FSM kApproach)",
                        req->target_force);
            resp->ok = true;
            resp->message = "grasp started @ " + std::to_string(req->target_force) + " N";
          } else if (req->command == Req::RELEASE) {
            grasp_cmd_.store(static_cast<int>(Req::RELEASE), std::memory_order_release);
            RCLCPP_INFO(logger_, "[grasp_command] RELEASE (WBC FSM kRelease)");
            resp->ok = true;
            resp->message = "release accepted";
          } else {
            resp->ok = false;
            resp->message = "command must be GRASP or RELEASE";
          }
        });
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "DemoWbcController on_configure failed: %s", e.what());
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(logger_, "DemoWbcController on_configure failed: unknown");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}


RTControllerInterface::CallbackReturn DemoWbcController::on_activate(
    const rclcpp_lifecycle::State& prev, const rtc::ControllerState& device_snapshot) noexcept {
  ActivateOwnedTopics(prev, owned_topics_);

  // MPC tick-timing CSV + 1 Hz aux timer: one-shot setup per controller
  // lifetime (gated on mpc_timing_initialized_). Re-activation after a
  // deactivate must NOT re-Open the CSV (truncates accumulated rows) or
  // re-register the timer (executor churn).
  if (mpc_enabled_ && !mpc_timing_initialized_) {
    if (!mpc_timing_logger_.Open()) {
      RCLCPP_WARN(logger_, "MpcTimingLogger::Open() failed — MPC timing CSV disabled");
    } else {
      RCLCPP_INFO(logger_, "MPC tick-timing CSV: %s", mpc_timing_logger_.Path().c_str());
    }
    auto node = get_lifecycle_node();
    if (node && mpc_timing_cb_group_) {
      mpc_timing_timer_ = node->create_wall_timer(
          std::chrono::seconds(1), [this]() { LogMpcSolveTimingTick(); }, mpc_timing_cb_group_);
    }
    mpc_timing_initialized_ = true;
  }

  // Delegate to base for hold-init (InitializeHoldPosition lazy-spawns the
  // MPC thread on the first call). Snapshot is empty when CM activates the
  // initial controller before any RT tick — RT loop's auto-hold path then
  // takes over and triggers spawn on the first sensor read.
  const auto rc = RTControllerInterface::on_activate(prev, device_snapshot);

  // Resume the MPC solve loop if the thread has already been spawned (either
  // by the base on_activate above, or by a previous activation cycle). Calling
  // Resume on a freshly-spawned thread that was never paused is a no-op.
  if (mpc_thread_) {
    mpc_thread_->Resume();
  }

  return rc;
}


RTControllerInterface::CallbackReturn DemoWbcController::on_deactivate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // Pause the MPC solve loop so it stops burning CPU while this controller
  // is inactive. The timing logger / timer keep running — the MPCThread
  // TimingProducer drain naturally yields zero rows while paused, and the
  // aggregate stats are skipped via the mpc_manager_.Enabled() guard at
  // the top of LogMpcSolveTimingTick.
  if (mpc_thread_) {
    mpc_thread_->Pause();
  }
  DeactivateOwnedTopics(prev, owned_topics_);
  log_set_.DrainAll();  // flush in-flight log SPSC residue
  return CallbackReturn::SUCCESS;
}


RTControllerInterface::CallbackReturn DemoWbcController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  mpc_timing_timer_.reset();
  mpc_timing_cb_group_.reset();
  mpc_timing_initialized_ = false;
  mpc_timing_tick_ = 0;
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetOwnedTopics(owned_topics_);
  grasp_command_srv_.reset();
  param_callback_handle_.reset();
  return RTControllerInterface::on_cleanup(prev);
}

// ── Phase D: gain → ROS 2 parameter declaration & callback ────────────────
//
// Mirrors DemoTask/DemoJoint Phase D pattern but with WBC-specific keys:
// arm_*/hand_* trajectory, TSID weights, MPC runtime gates. mpc_enable and
// riccati_gain_scale are forwarded into mpc_manager_ at set time so they
// take effect immediately (no controller restart required).

}  // namespace ur5e_bringup
