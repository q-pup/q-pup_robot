#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <chrono>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <variant>
#include <vector>

#include "qpup_generated/qpup_can_generated.h"
#include "realtime_tools/realtime_buffer.h"
#include "ros/ros.h"

// clang-format off

// MSG_NAME must be upper case
#define CASE_ODRIVE_IDS(MSG_NAME)          \
  case QPUP_AXIS_1_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_2_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_3_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_4_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_5_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_6_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_7_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_8_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_9_##MSG_NAME##_FRAME_ID:  \
  case QPUP_AXIS_10_##MSG_NAME##_FRAME_ID: \
  case QPUP_AXIS_11_##MSG_NAME##_FRAME_ID: \
  case QPUP_AXIS_12_##MSG_NAME##_FRAME_ID
// clang-format on

// MSG_NAME must be lower case
#define UNPACK_ODRIVE_MSG_STRUCT(MSG_NAME, DATA_VARIANT, SRC_CAN_FRAME)                                             \
  do {                                                                                                              \
    if (SRC_CAN_FRAME.can_dlc != ODRIVE_RTR_DLC) {                                                                  \
      ROS_ERROR_STREAM_NAMED(logger_, "Size mismatch between canframe size and requested decoded size. dlc: "       \
                                          << static_cast<unsigned>(SRC_CAN_FRAME.can_dlc)                           \
                                          << " expected size: " << ODRIVE_RTR_DLC);                                 \
    }                                                                                                               \
    DATA_VARIANT.emplace<qpup_odrive_##MSG_NAME##_t>();                                                             \
    if (qpup_odrive_##MSG_NAME##_unpack(std::get_if<qpup_odrive_##MSG_NAME##_t>(&DATA_VARIANT), SRC_CAN_FRAME.data, \
                                        ODRIVE_RTR_DLC) < 0) {                                                      \
      ROS_ERROR_STREAM_NAMED(logger_,                                                                               \
                             "Failed to unpack message via " << STRINGIFY(qpup_odrive_##MSG_NAME##_unpack()));      \
    }                                                                                                               \
  } while (0)

#define STRINGIFY(s) _STRINGIFY(s)
#define _STRINGIFY(s) #s

#define ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, AXIS)                                                      \
  do {                                                                                                          \
    if (!QPUP_CAN_INSTANCE.writeODriveRTRFrame(QPUP_AXIS_##AXIS##_##COMMAND##_FRAME_ID)) {                      \
      ROS_ERROR_STREAM_NAMED(                                                                                   \
          QPUP_CAN_INSTANCE.getLogger(), "Failed to write ODrive RTR Frame: " << STRINGIFY(QPUP_AXIS_##AXIS##_##COMMAND##_FRAME_ID)); \
    }                                                                                                           \
  } while (0)

#define ODRIVE_WRITE_RTR_ALL(QPUP_CAN_INSTANCE, COMMAND) \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 1);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 2);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 3);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 4);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 5);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 6);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 7);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 8);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 9);       \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 10);      \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 11);      \
  ODRIVE_WRITE_RTR(QPUP_CAN_INSTANCE, COMMAND, 12);

namespace qpup_utils {

class QPUP_CAN {
 public:
  using received_CAN_data =
      std::variant<qpup_odrive_heartbeat_t, qpup_odrive_get_motor_error_t, qpup_odrive_get_encoder_error_t,
                   qpup_odrive_get_sensorless_error_t, qpup_odrive_get_encoder_estimates_t,
                   qpup_odrive_get_encoder_count_t, qpup_odrive_get_iq_t, qpup_odrive_get_sensorless_estimates_t,
                   qpup_odrive_get_vbus_voltage_t>;
  explicit QPUP_CAN() = delete;

  /**
   * QPUP_CAN
   * @param default_endianess the default endianness of other CAN
   * nodes. Used for automatic endianness conversion
   * @param can_interface_name name of socketcan interface
   */
  explicit QPUP_CAN(int default_endianess, std::string can_interface_name);
  virtual ~QPUP_CAN();

  bool configure();
  bool activate();
  bool deactivate();
  bool cleanup();

  std::optional<received_CAN_data> getLatestValue(canid_t msg_id);
  bool writeODriveRTRFrame(canid_t msg_id);
  bool writeRTRFrame(canid_t msg_id, uint8_t size);
  bool writeFrame(canid_t msg_id, uint8_t* data, uint8_t size);

  std::string getLogger();

 private:
  void readSocketTask();

  std::string logger_;
  int default_endianness_;
  std::string can_interface_name_;
  int socket_handle_{};

  enum class state { UNCONFIGURED, INACTIVE, ACTIVE, FINALIZED, ERROR, UNKNOWN };
  friend std::ostream& operator<<(std::ostream& out, state& state) {
    switch (state) {
      case state::UNCONFIGURED:
        out << "UNCONFIGURED";
        break;
      case state::INACTIVE:
        out << "INACTIVE";
        break;
      case state::ACTIVE:
        out << "ACTIVE";
        break;
      case state::FINALIZED:
        out << "FINALIZED";
        break;
      case state::ERROR:
        out << "ERROR";
        break;
      case state::UNKNOWN:
        out << "UNKNOWN";
        break;
    }
    return out;
  }
  state internal_state_{state::UNKNOWN};

  std::thread read_thread_;
  std::atomic_flag run_read_thread_ = ATOMIC_FLAG_INIT;

  std::map<canid_t, realtime_tools::RealtimeBuffer<received_CAN_data>> recv_map_;
  std::timed_mutex recv_map_mtx_;
  static constexpr std::chrono::milliseconds MUTEX_LOCK_TIMEOUT{1};

  // FIXME: Upstream odrive firmware seems to hardcode rtr dlc to 8, even when messages are smaller
  static constexpr uint8_t ODRIVE_RTR_DLC{8};
};

}  // namespace qpup_utils
