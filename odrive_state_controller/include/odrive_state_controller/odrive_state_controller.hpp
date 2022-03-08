#pragma once

#include "controller_interface/controller.h"
#include "odrive_state_msgs/OdriveState.h"
#include "qpup_hw/odrive_state_interface.hpp"
#include "realtime_tools/realtime_publisher.h"

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
  std::shared_ptr<realtime_tools::RealtimePublisher<odrive_state_msgs::OdriveState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
  unsigned int num_odrive_axis_;
};
}  // namespace odrive_state_controller