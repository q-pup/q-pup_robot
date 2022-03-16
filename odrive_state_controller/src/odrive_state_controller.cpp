#include "odrive_state_controller/odrive_state_controller.hpp"

#include "qpup_utils/qpup_params.hpp"

#include <algorithm>
#include <cstddef>
#include <pluginlib/class_list_macros.hpp>

namespace odrive_state_controller {

bool OdriveStateController::init(qpup_hw::OdriveStateInterface* hw, ros::NodeHandle& root_nh,
                                 ros::NodeHandle& controller_nh) {
  logger_ = qpup_utils::getLoggerName(controller_nh);

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
  (void)(do_not_clear_errors_flag_.test_and_set());  // Init so that it doesn't clear errors on startup
  clear_errors_server_ =
      controller_nh.advertiseService("clear_errors", &OdriveStateController::clearErrorsCallback, this);

  for (unsigned i = 0; i < num_odrive_axis_; i++) {
    std::string joint_name = odrive_state_[i].getName();

    odrive_axis_state_cmd_[joint_name].store(odrive_state_msgs::SetAxisState::Request::Type::AXIS_STATE_IDLE);
    odrive_control_mode_cmd_[joint_name].store(odrive_state_msgs::SetControlMode::Request::Type::CONTROL_MODE_POSITION);
    odrive_input_mode_cmd_[joint_name].store(odrive_state_msgs::SetInputMode::Request::Type::INPUT_MODE_PASSTHROUGH);
  }
  set_axis_state_server_ =
      controller_nh.advertiseService("set_axis_state", &OdriveStateController::setAxisStateCallback, this);
  set_control_mode_server_ =
      controller_nh.advertiseService("set_control_mode", &OdriveStateController::setControlModeCallback, this);
  set_input_mode_server_ =
      controller_nh.advertiseService("set_input_mode", &OdriveStateController::setInputModeCallback, this);

  return true;
}

void OdriveStateController::starting(const ros::Time& time) {
  last_publish_time_ = time;
}

void OdriveStateController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  for (unsigned i = 0; i < num_odrive_axis_; i++) {
    std::string joint_name = odrive_state_[i].getName();

    odrive_state_[i].setAxisState(odrive_axis_state_cmd_[joint_name].load());
    odrive_state_[i].setControlMode(odrive_control_mode_cmd_[joint_name].load());
    odrive_state_[i].setInputMode(odrive_input_mode_cmd_[joint_name].load());
  }

  // Only trigger clear_errors once per service call
  const bool clear_errors = !do_not_clear_errors_flag_.test_and_set();
  for (unsigned i = 0; i < num_odrive_axis_; i++) {
    odrive_state_[i].setClearErrors(clear_errors);
  }

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
  do_not_clear_errors_flag_.clear();
  return true;
}

bool OdriveStateController::setAxisStateCallback(odrive_state_msgs::SetAxisState::Request& request,
                                                 odrive_state_msgs::SetAxisState::Response& /* response */) {
  if (request.axis_state >= odrive_state_msgs::SetAxisState::Request::Type::AXIS_STATE_INVALID) {
    ROS_ERROR_STREAM_NAMED(logger_, "Invalid Axis State: " << static_cast<unsigned>(request.axis_state));
    return false;
  }

  odrive_axis_state_cmd_[request.joint_name].store(request.axis_state);
  return true;
}

bool OdriveStateController::setControlModeCallback(odrive_state_msgs::SetControlMode::Request& request,
                                                   odrive_state_msgs::SetControlMode::Response& /* response */) {
  if (request.control_mode >= odrive_state_msgs::SetControlMode::Request::Type::CONTROL_MODE_INVALID) {
    ROS_ERROR_STREAM_NAMED(logger_, "Invalid Control Mode: " << static_cast<unsigned>(request.control_mode));
    return false;
  }

  odrive_control_mode_cmd_[request.joint_name].store(request.control_mode);
  return true;
}

bool OdriveStateController::setInputModeCallback(odrive_state_msgs::SetInputMode::Request& request,
                                                 odrive_state_msgs::SetInputMode::Response& /* response */) {
  if (request.input_mode >= odrive_state_msgs::SetInputMode::Request::Type::INPUT_MODE_INVALID) {
    ROS_ERROR_STREAM_NAMED(logger_, "Invalid Input Mode: " << static_cast<unsigned>(request.input_mode));
    return false;
  }

  odrive_input_mode_cmd_[request.joint_name].store(request.input_mode);
  return true;
}

}  // namespace odrive_state_controller

PLUGINLIB_EXPORT_CLASS(odrive_state_controller::OdriveStateController, controller_interface::ControllerBase)
