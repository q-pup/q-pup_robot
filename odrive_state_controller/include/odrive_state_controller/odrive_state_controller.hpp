#pragma once

#include "controller_interface/controller.h"
#include "odrive_state_msgs/OdriveState.h"
#include "odrive_state_msgs/SetAxisState.h"
#include "odrive_state_msgs/SetControlMode.h"
#include "odrive_state_msgs/SetInputMode.h"
#include "qpup_hw/odrive_state_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"

namespace odrive_state_controller {
class OdriveStateController : public controller_interface::Controller<qpup_hw::OdriveStateInterface> {
 public:
  OdriveStateController() : publish_rate_(0.0) {}

  virtual bool init(qpup_hw::OdriveStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

 private:
  std::vector<qpup_hw::OdriveStateHandle> odrive_state_;
  std::map<std::string, std::atomic<uint16_t>> odrive_axis_state_cmd_;
  std::map<std::string, std::atomic<uint8_t>> odrive_control_mode_cmd_;
  std::map<std::string, std::atomic<uint16_t>> odrive_input_mode_cmd_;
  std::shared_ptr<realtime_tools::RealtimePublisher<odrive_state_msgs::OdriveState>> realtime_pub_;
  ros::ServiceServer clear_errors_server_;
  ros::ServiceServer set_axis_state_server_;
  ros::ServiceServer set_control_mode_server_;
  ros::ServiceServer set_input_mode_server_;

  std::atomic_flag do_not_clear_errors_flag_ = ATOMIC_FLAG_INIT;
  ros::Time last_publish_time_;
  double publish_rate_;
  unsigned int num_odrive_axis_;

  bool clearErrorsCallback(std_srvs::Empty::Request& /* request */, std_srvs::Empty::Response& /*response*/);
  bool setAxisStateCallback(odrive_state_msgs::SetAxisState::Request& request,
                            odrive_state_msgs::SetAxisState::Response& /* response */);
  bool setControlModeCallback(odrive_state_msgs::SetControlMode::Request& request,
                              odrive_state_msgs::SetControlMode::Response& /* response */);
  bool setInputModeCallback(odrive_state_msgs::SetInputMode::Request& request,
                            odrive_state_msgs::SetInputMode::Response& /* response */);
};
}  // namespace odrive_state_controller
