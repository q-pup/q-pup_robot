#pragma once

#include "qpup_hw/navx/AHRS.h"
#include "qpup_hw/qpup_hw.hpp"
#include "qpup_utils/qpup_can.hpp"

#include <memory>

namespace qpup_hw {

class QPUPHWReal : public QPUPHW {
 public:
  explicit QPUPHWReal() : QPUPHW("QPUPHWReal"){};
  virtual ~QPUPHWReal();

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  void read(const ros::Time &time, const ros::Duration &period) override;
  void write(const ros::Time &time, const ros::Duration &period) override;
 private:
  std::unique_ptr<qpup_hw::navx::AHRS> imu_;
  std::unique_ptr<qpup_utils::QPUP_CAN> can_;
};

} // namespace qpup_hw
