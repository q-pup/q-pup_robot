#include "qpup_generated/qpup_can_generated.h"
#include "qpup_utils/qpup_can.hpp"

void readUpdateWrite(qpup_utils::QPUP_CAN& can) {
  // read

  // send rtr requests
  can.writeRTRFrame(QPUP_AXIS_3_GET_MOTOR_ERROR_FRAME_ID, sizeof(qpup_odrive_get_motor_error_t));

  // read periodic data
  // read rtr responses

  // update

  // write
  // write commands
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odrive_can_test");
  ros::NodeHandle nh;

  qpup_utils::QPUP_CAN can(__BYTE_ORDER__, "can0");
  assert(can.configure());
  assert(can.activate());

  while (ros::ok()) {
    assert(can.writeRTRFrame(QPUP_AXIS_3_GET_MOTOR_ERROR_FRAME_ID, sizeof(qpup_odrive_get_motor_error_t)));
    assert(can.writeRTRFrame(QPUP_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID, sizeof(qpup_odrive_get_vbus_voltage_t)));
    assert(can.writeRTRFrame(QPUP_AXIS_3_HEARTBEAT_FRAME_ID, sizeof(qpup_odrive_heartbeat_t)));

    double motor_error = qpup_odrive_get_motor_error_motor_error_decode(std::get<qpup_odrive_get_motor_error_t>(can.getLatestValue(QPUP_AXIS_3_GET_MOTOR_ERROR_FRAME_ID).value()).motor_error);
    double vbus_voltage = qpup_odrive_get_vbus_voltage_vbus_voltage_decode(std::get<qpup_odrive_get_vbus_voltage_t>(can.getLatestValue(QPUP_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID).value()).vbus_voltage);
//    auto heartbeat_info = std::get<qpup_odrive_heartbeat_t>(can.getLatestValue(QPUP_AXIS_3_HEARTBEAT_FRAME_ID).value());

//    heartbeat_info.axis_error;
//    heartbeat_info.axis_state;
//    heartbeat_info.controller_flags;
//    heartbeat_info.encoder_flags;
//    heartbeat_info.motor_flags;

    ROS_INFO_STREAM("Axis 3 motor error: " << motor_error);
    ROS_INFO_STREAM("Axis 3 vbus voltage: " << vbus_voltage);
//    ROS_INFO_STREAM("Axis 3 state: " << hearbeat_info_message.axis_state);

    //  assert(writeFrame(canid_t msg_id, uint8_t* data, uint8_t size);
  };

  assert(can.deactivate());
  assert(can.cleanup());
}

// Encode signals
// Pack RTR frame
// Send Frames
// get latest RTR
// get some latest streamed
