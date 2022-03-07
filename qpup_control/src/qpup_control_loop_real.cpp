#include "qpup_control/qpup_control_loop_real.hpp"
#include "qpup_utils/qpup_params.hpp"

namespace qpup_control {
// static constexpr class members must have definitions outside of their class
// to compile. This can be removed in C++17
constexpr double QPUPControlLoop::DEFAULT_CONTROL_FREQUENCY;
constexpr double QPUPControlLoop::DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT;

QPUPControlLoopReal::QPUPControlLoopReal(const ros::NodeHandle &nh)
    : QPUPControlLoop("qpup_control_loop_real", nh) {
  control_freq_ = qpup_utils::getParam(private_nh_, name_, "control_frequency",
                                       DEFAULT_CONTROL_FREQUENCY);
  controller_watchdog_timeout_ =
      qpup_utils::getParam(private_nh_, name_, "controllers_watchdog_timeout",
                           DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT);
  if (!this->init()) {
    ROS_ERROR_NAMED(name_, "Failed to initialize RobotHW");
    throw std::runtime_error("Failed to initialize RobotHW");
  }
}

void QPUPControlLoopReal::runForeverBlocking() {
  ros::Rate rate(control_freq_);

  while (ros::ok()) {
    this->update();
    rate.sleep();
  }
}
} // namespace qpup_control
