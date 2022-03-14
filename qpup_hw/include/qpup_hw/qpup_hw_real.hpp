#pragma once

#include <math.h>

#include <memory>

#include "qpup_hw/navx/AHRS.h"
#include "qpup_hw/qpup_hw.hpp"
#include "qpup_utils/qpup_can.hpp"

namespace qpup_hw {

class QPUPHWReal : public QPUPHW {
 public:
  struct OdriveAxisParams{
    uint8_t can_id;
    double pos_gain;
    double vel_gain;
    double vel_integrator_gain;
  };

  explicit QPUPHWReal() : QPUPHW("QPUPHWReal"){};
  virtual ~QPUPHWReal();

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  void read(const ros::Time &time, const ros::Duration &period) override;
  void write(const ros::Time &time, const ros::Duration &period) override;

 private:
  bool loadOdriveConfigFromParamServer(ros::NodeHandle &robot_hw_nh);
  bool updatePIDGains();

  std::unique_ptr<qpup_hw::navx::AHRS> imu_;
  std::unique_ptr<qpup_utils::QPUP_CAN> can_;
  std::map<std::string, OdriveAxisParams> odrive_axis_params_;

  static constexpr double TORQUE_CONSTANT{0.6778688430786133};
  static constexpr double RADIANS_PER_ROTATION{2 * M_PI};
};

}  // namespace qpup_hw
