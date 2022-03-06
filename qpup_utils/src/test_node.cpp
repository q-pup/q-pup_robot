#include "qpup_can_database.h"
#include "qpup_utils/qpup_can.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "odrive_can_test");
  ros::NodeHandle nh;

  qpup_utils::QPUP_CAN can(__BYTE_ORDER__, "can0");
  assert(can.configure());
  assert(can.activate());

  assert(can.writeRTRFrame(QPUP_CAN_DATABASE_AXIS_3_GET_MOTOR_ERROR_FRAME_ID,
                           sizeof(qpup_can_database_axis_3_get_motor_error_t)));
  assert(can.writeRTRFrame(QPUP_CAN_DATABASE_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID,
                           sizeof(qpup_can_database_axis_3_get_vbus_voltage_t)));
  assert(can.writeRTRFrame(QPUP_CAN_DATABASE_AXIS_3_HEARTBEAT_FRAME_ID, sizeof(qpup_can_database_axis_3_heartbeat_t)));

  auto motor_error = can.getLatestValue(QPUP_CAN_DATABASE_AXIS_3_GET_MOTOR_ERROR_FRAME_ID);
  auto vbus = can.getLatestValue(QPUP_CAN_DATABASE_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID);
  auto heartbeat_info = can.getLatestValue(QPUP_CAN_DATABASE_AXIS_3_HEARTBEAT_FRAME_ID);

  while (!motor_error.has_value() || !vbus.has_value() || !heartbeat_info.has_value()) {
    motor_error = can.getLatestValue(QPUP_CAN_DATABASE_AXIS_3_GET_MOTOR_ERROR_FRAME_ID);
    vbus = can.getLatestValue(QPUP_CAN_DATABASE_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID);
    heartbeat_info = can.getLatestValue(QPUP_CAN_DATABASE_AXIS_3_HEARTBEAT_FRAME_ID);
  }

  assert(motor_error != std::nullopt);
  assert(vbus != std::nullopt);
  assert(heartbeat_info != std::nullopt);

  auto motor_error_message = std::get<qpup_can_database_axis_3_get_motor_error_t>(motor_error.value());
  auto vbus_message = std::get<qpup_can_database_axis_3_get_vbus_voltage_t>(vbus.value());
  auto hearbeat_info_message = std::get<qpup_can_database_axis_3_heartbeat_t>(heartbeat_info.value());

  ROS_INFO_STREAM("Axis 3 vbus voltage: " << vbus_message.vbus_voltage);
  ROS_INFO_STREAM("Axis 3 state: " << hearbeat_info_message.axis_state);
  ROS_INFO_STREAM("Axis 3 motor error: " << motor_error_message.motor_error);

  //  assert(writeFrame(canid_t msg_id, uint8_t* data, uint8_t size);

  while (ros::ok()) {
    assert(can.writeRTRFrame(QPUP_CAN_DATABASE_AXIS_3_GET_MOTOR_ERROR_FRAME_ID,
                             sizeof(qpup_can_database_axis_3_get_motor_error_t)));
    assert(can.writeRTRFrame(QPUP_CAN_DATABASE_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID,
                             sizeof(qpup_can_database_axis_3_get_vbus_voltage_t)));
    assert(
        can.writeRTRFrame(QPUP_CAN_DATABASE_AXIS_3_HEARTBEAT_FRAME_ID, sizeof(qpup_can_database_axis_3_heartbeat_t)));

    motor_error_message = std::get<qpup_can_database_axis_3_get_motor_error_t>(motor_error.value());
    vbus_message = std::get<qpup_can_database_axis_3_get_vbus_voltage_t>(vbus.value());
    hearbeat_info_message = std::get<qpup_can_database_axis_3_heartbeat_t>(heartbeat_info.value());

    ROS_INFO_STREAM("Axis 3 vbus voltage: " << vbus_message.vbus_voltage);
    ROS_INFO_STREAM("Axis 3 state: " << hearbeat_info_message.axis_state);
    ROS_INFO_STREAM("Axis 3 motor error: " << motor_error_message.motor_error);
  };

  assert(can.deactivate());
  assert(can.cleanup());
}

// Encode signals
// Pack RTR frame
// Send Frames
// get latest RTR
// get some latest streamed
