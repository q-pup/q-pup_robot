#include "odrive_state_controller/odrive_state_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <pluginlib/class_list_macros.hpp>

namespace odrive_state_controller {

bool OdriveStateController::init(qpup_hw::OdriveStateInterface* hw, ros::NodeHandle& root_nh,
                                 ros::NodeHandle& controller_nh) {
  // List of joints names(associated with odrives) to be published
  std::vector<std::string> joint_names;

  // If specified, publishes joints in order of param, otherwise uses odrive_state_interface order
  if (controller_nh.getParam("joints", joint_names)) {
    ROS_INFO_STREAM("Joints parameter specified, publishing specified joints in desired order.");
  } else {
    joint_names = hw->getNames();
  }

  num_odrive_axis_ = joint_names.size();
  for (unsigned i = 0; i < num_odrive_axis_; i++) ROS_DEBUG("Got joint %s", joint_names[i].c_str());

  if (!controller_nh.getParam("publish_rate", publish_rate_)) {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  // realtime publisher
  realtime_pub_.reset(
      new realtime_tools::RealtimePublisher<odrive_state_msgs::OdriveState>(root_nh, "odrive_states", 4));

  // get joints and allocate message
  for (unsigned i = 0; i < num_odrive_axis_; i++) {
    odrive_state_.push_back(hw->getHandle(joint_names[i]));
    realtime_pub_->msg_.name.push_back(joint_names[i]);
    realtime_pub_->msg_.axis_error.push_back(0);
    realtime_pub_->msg_.axis_state.push_back(0);
    realtime_pub_->msg_.motor_flags.push_back(0);
    realtime_pub_->msg_.encoder_flags.push_back(0);
    realtime_pub_->msg_.controller_flags.push_back(0);
    realtime_pub_->msg_.motor_error.push_back(0);
    realtime_pub_->msg_.encoder_error.push_back(0);
    realtime_pub_->msg_.sensorless_error.push_back(0);
    realtime_pub_->msg_.shadow_count.push_back(0);
    realtime_pub_->msg_.count_in_cpr.push_back(0);
    realtime_pub_->msg_.iq_setpoint.push_back(0.0);
    realtime_pub_->msg_.iq_measured.push_back(0.0);
    realtime_pub_->msg_.vbus_voltage.push_back(0.0);
  }

  // Clear errors service
  clear_errors_ = controller_nh.advertiseService("clear_errors", &OdriveStateController::clearErrorsCallback, this);

  return true;
}

void OdriveStateController::starting(const ros::Time& time) {
  last_publish_time_ = time;
}

void OdriveStateController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    if (realtime_pub_->trylock()) {
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

      realtime_pub_->msg_.header.stamp = time;
      for (unsigned i = 0; i < num_odrive_axis_; i++) {
        realtime_pub_->msg_.axis_error[i] = odrive_state_[i].getAxisError();
        realtime_pub_->msg_.axis_state[i] = odrive_state_[i].getAxisState();
        realtime_pub_->msg_.motor_flags[i] = odrive_state_[i].getMotorFlags();
        realtime_pub_->msg_.encoder_flags[i] = odrive_state_[i].getEncoderFlags();
        realtime_pub_->msg_.controller_flags[i] = odrive_state_[i].getControllerFlags();
        realtime_pub_->msg_.motor_error[i] = odrive_state_[i].getMotorError();
        realtime_pub_->msg_.encoder_error[i] = odrive_state_[i].getEncoderError();
        realtime_pub_->msg_.sensorless_error[i] = odrive_state_[i].getSensorlessError();
        realtime_pub_->msg_.shadow_count[i] = odrive_state_[i].getShadowCount();
        realtime_pub_->msg_.count_in_cpr[i] = odrive_state_[i].getCountInCPR();
        realtime_pub_->msg_.iq_setpoint[i] = odrive_state_[i].getIQSetpoint();
        realtime_pub_->msg_.iq_measured[i] = odrive_state_[i].getIQMeasured();
        realtime_pub_->msg_.vbus_voltage[i] = odrive_state_[i].getVbusVoltage();
      }
      realtime_pub_->unlockAndPublish();
    }
  }
}

void OdriveStateController::stopping(const ros::Time& /*time*/) {}

bool OdriveStateController::clearErrorsCallback(std_srvs::Empty::Request& /* request */,
                                                std_srvs::Empty::Response& /*response*/) {
  for (unsigned i = 0; i < num_odrive_axis_; i++) {
    odrive_state_[i].triggerClearErrors();
  }
  return 1U;
}

}  // namespace odrive_state_controller

PLUGINLIB_EXPORT_CLASS(odrive_state_controller::OdriveStateController, controller_interface::ControllerBase)