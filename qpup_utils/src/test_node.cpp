#include "qpup_utils/qpup_generated/qpup_can_generated.h"
#include "qpup_utils/qpup_can.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "odrive_can_test");
  ros::NodeHandle nh;

  qpup_utils::QPUP_CAN can(__BYTE_ORDER__, "can0");
  assert(can.configure());
  assert(can.activate());

  while (ros::ok()) {
    ODRIVE_WRITE_RTR_ALL(can, GET_MOTOR_ERROR);
    ODRIVE_WRITE_RTR_ALL(can, GET_ENCODER_ERROR);
    ODRIVE_WRITE_RTR_ALL(can, GET_SENSORLESS_ERROR);
    ODRIVE_WRITE_RTR_ALL(can, GET_ENCODER_COUNT);
    ODRIVE_WRITE_RTR_ALL(can, GET_IQ);
    ODRIVE_WRITE_RTR_ALL(can, GET_SENSORLESS_ESTIMATES);
    ODRIVE_WRITE_RTR_ALL(can, GET_VBUS_VOLTAGE);

    const auto motor_error_opt = can.getLatestValue(QPUP_AXIS_3_GET_MOTOR_ERROR_FRAME_ID);
    if (motor_error_opt.has_value()) {
      const uint32_t motor_error = std::get<qpup_odrive_get_motor_error_t>(motor_error_opt.value()).motor_error;
      ROS_INFO_STREAM("Axis 3 motor error: " << motor_error);
    }

    const auto vbus_opt = can.getLatestValue(QPUP_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID);
    if (vbus_opt.has_value()) {
      const double vbus_voltage = std::get<qpup_odrive_get_vbus_voltage_t>(vbus_opt.value()).vbus_voltage;
      ROS_INFO_STREAM("Axis 3 vbus voltage: " << vbus_voltage);
    }

    const auto heartbeat_opt = can.getLatestValue(QPUP_AXIS_3_HEARTBEAT_FRAME_ID);
    if (heartbeat_opt.has_value()) {
      const auto heartbeat_info = std::get<qpup_odrive_heartbeat_t>(heartbeat_opt.value());
      ROS_INFO_STREAM("Axis 3 axis_error: " << heartbeat_info.axis_error);
      ROS_INFO_STREAM("Axis 3 axis_state: " << heartbeat_info.axis_state);
      ROS_INFO_STREAM("Axis 3 controller_flags: " << heartbeat_info.controller_flags);
      ROS_INFO_STREAM("Axis 3 encoder_flags: " << heartbeat_info.encoder_flags);
      ROS_INFO_STREAM("Axis 3 motor_flags: " << heartbeat_info.motor_flags);
    }
  }

  assert(can.deactivate());
  assert(can.cleanup());
}
