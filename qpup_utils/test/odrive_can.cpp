#include <gtest/gtest.h>

#include "qpup_utils/qpup_can.hpp"
#include "qpup_generated/qpup_can_generated.h"

TEST(TestSuite, testCase1) {
  qpup_utils::QPUP_CAN can(__BYTE_ORDER__, "can0");
  ASSERT_TRUE(can.configure());
  ASSERT_TRUE(can.activate());

  ASSERT_TRUE(can.writeRTRFrame(QPUP_AXIS_3_GET_MOTOR_ERROR_FRAME_ID, sizeof(qpup_odrive_get_motor_error_t)));
  ASSERT_TRUE(can.writeRTRFrame(QPUP_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID, sizeof(qpup_odrive_get_vbus_voltage_t)));

  auto motor_error = can.getLatestValue(QPUP_AXIS_3_GET_MOTOR_ERROR_FRAME_ID);
  auto vbus = can.getLatestValue(QPUP_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID);

  while (!motor_error.has_value() || !vbus.has_value()) {
    motor_error = can.getLatestValue(QPUP_AXIS_3_GET_MOTOR_ERROR_FRAME_ID);
    vbus = can.getLatestValue(QPUP_AXIS_3_GET_VBUS_VOLTAGE_FRAME_ID);
  }

  ASSERT_NE(motor_error, std::nullopt);
  ASSERT_NE(vbus, std::nullopt);

  auto motor_error_message = std::get<qpup_odrive_get_motor_error_t>(motor_error.value());
  auto vbus_message = std::get<qpup_odrive_get_vbus_voltage_t>(vbus.value());

  ROS_INFO_STREAM(vbus_message.vbus_voltage);
  ROS_INFO_STREAM(motor_error_message.motor_error);

  //  ASSERT_TRUE(writeFrame(canid_t msg_id, uint8_t* data, uint8_t size);

  ASSERT_TRUE(can.deactivate());
  ASSERT_TRUE(can.cleanup());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "odrive_can_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

// Encode signals
// Pack RTR frame
// Send Frames
// get latest RTR
// get some latest streamed
