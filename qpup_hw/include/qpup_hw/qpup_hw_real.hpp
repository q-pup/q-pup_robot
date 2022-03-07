#pragma once

#include "qpup_hw/qpup_hw.hpp"

namespace qpup_hw {

class QPUPHWReal : public QPUPHW {
public:
  explicit QPUPHWReal() : QPUPHW("QPUPHWReal"){};

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  void read(const ros::Time &time, const ros::Duration &period) override;
  void write(const ros::Time &time, const ros::Duration &period) override;
};

} // namespace qpup_hw
