#include "qpup_control/qpup_control_loop.hpp"

namespace qpup_control {

bool QPUPControlLoop::init() {
  ros::NodeHandle qpup_hw_nh(nh_, "combined_robot_hw");
  qpup_hw_ = std::make_unique<combined_robot_hw::CombinedRobotHW>();
  if (!qpup_hw_->init(nh_, qpup_hw_nh)) {
    ROS_FATAL_STREAM_NAMED(name_, "Failed to initialize combined_robot_hw");
    return false;
  }

  controller_manager_ = std::make_unique<controller_manager::ControllerManager>(
      qpup_hw_.get(), nh_);

  // Set last_control_loop_time_ for first update call
  last_control_loop_time_ = ros::Time::now();

  return true;
}

void QPUPControlLoop::update() {
  // Update current time using UNIX time
  ros::Time time_now = ros::Time::now();

  // Calculate control loop period using Monotonic Time
  current_control_loop_time_ = ros::Time::now();

  ros::Duration control_loop_period =
      current_control_loop_time_ - last_control_loop_time_;
  last_control_loop_time_ = current_control_loop_time_;

  qpup_hw_->read(time_now, control_loop_period);

  bool reset_controllers =
      (control_loop_period.toSec() > controller_watchdog_timeout_);
  ROS_WARN_STREAM_COND_NAMED(
      reset_controllers, name_,
      "Control loop period exceeded watchdog timeout by "
          << (control_loop_period.toSec() - controller_watchdog_timeout_)
          << " seconds. Resetting controllers.");
  controller_manager_->update(time_now, control_loop_period, reset_controllers);

   qpup_hw_->write(time_now, control_loop_period);
}

} // namespace qpup_control
