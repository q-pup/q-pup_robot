#pragma once

#include "qpup_control_loop.hpp"
#include "qpup_hw/qpup_hw_real.hpp"

namespace qpup_control {
class QPUPControlLoopReal : public QPUPControlLoop {
public:
  explicit QPUPControlLoopReal(const ros::NodeHandle &nh);

  void runForeverBlocking();
};
} // namespace qpup_control
