#pragma once

#include "combined_robot_hw/combined_robot_hw.h"
#include "controller_manager/controller_manager.h"
#include "qpup_hw/qpup_hw.hpp"
#include "ros/ros.h"

#include <memory>
#include <string>

namespace qpup_control {

class QPUPControlLoop {
public:
  QPUPControlLoop(std::string name,
                  const ros::NodeHandle &nh = ros::NodeHandle())
      : name_(std::move(name)), nh_(nh),
        private_nh_(ros::NodeHandle(nh, name_)) {}

  QPUPControlLoop(QPUPControlLoop &&) = delete;
  QPUPControlLoop(const QPUPControlLoop &) = delete;
  QPUPControlLoop &operator=(const QPUPControlLoop &&) = delete;
  QPUPControlLoop &operator=(const QPUPControlLoop &) = delete;
  virtual ~QPUPControlLoop() = default;

  virtual bool init();

  void update();

protected:
  static constexpr double DEFAULT_CONTROL_FREQUENCY{50.0};
  static constexpr double DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT{5.0};

  const std::string name_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::unique_ptr<combined_robot_hw::CombinedRobotHW> qpup_hw_;
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Time current_control_loop_time_;
  ros::Time last_control_loop_time_;

  double control_freq_{0.0};
  double controller_watchdog_timeout_{0.0};
};
} // namespace qpup_control
