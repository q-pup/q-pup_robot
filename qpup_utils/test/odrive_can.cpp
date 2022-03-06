#include <gtest/gtest.h>

#include "qpup_utils/qpup_can.hpp"
#include "qpup_can_database.h"

TEST(TestSuite, testCase1) {
  qpup_utils::QPUP_CAN can(__BYTE_ORDER__, "can0");
  ASSERT_TRUE(can.configure());
  ASSERT_TRUE(can.activate());

  ASSERT_TRUE(can.writeRTRFrame(QPUP_CAN_DATABASE_AXIS_0_GET_MOTOR_ERROR_FRAME_ID,
                                sizeof(qpup_can_database_axis_0_get_motor_error_t)));
  auto motor_error = can.getLatestValue(QPUP_CAN_DATABASE_AXIS_0_GET_MOTOR_ERROR_FRAME_ID);
  ASSERT_NE(motor_error, std::nullopt);

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
