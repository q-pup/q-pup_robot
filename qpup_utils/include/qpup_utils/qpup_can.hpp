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

#include "qpup_can_database.h"
#include "realtime_tools/realtime_buffer.h"
#include "ros/ros.h"

// clang-format off
#define ODRIVE_RECEIVED_TYPES(NODE_ID)                           \
  qpup_can_database_axis_##NODE_ID##_heartbeat_t,                \
  qpup_can_database_axis_##NODE_ID##_get_motor_error_t,          \
  qpup_can_database_axis_##NODE_ID##_get_encoder_error_t,        \
  qpup_can_database_axis_##NODE_ID##_get_sensorless_error_t,     \
  qpup_can_database_axis_##NODE_ID##_get_encoder_estimates_t,    \
  qpup_can_database_axis_##NODE_ID##_get_encoder_count_t,        \
  qpup_can_database_axis_##NODE_ID##_get_iq_t,                   \
  qpup_can_database_axis_##NODE_ID##_get_sensorless_estimates_t, \
  qpup_can_database_axis_##NODE_ID##_get_vbus_voltage_t

// MSG_NAME must be upper case
#define CASE_ODRIVE_IDS(MSG_NAME)                       \
  case QPUP_CAN_DATABASE_AXIS_0_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_1_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_2_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_3_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_4_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_5_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_6_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_7_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_8_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_9_##MSG_NAME##_FRAME_ID:  \
  case QPUP_CAN_DATABASE_AXIS_10_##MSG_NAME##_FRAME_ID: \
  case QPUP_CAN_DATABASE_AXIS_11_##MSG_NAME##_FRAME_ID


// MSG_NAME must be lower case
#define UNPACK_MSG_STRUCT(MSG_NAME, DST_PTR, SRC_CAN_FRAME)                                                          \
  do {                                                                                                               \
    if (SRC_CAN_FRAME.can_dlc != sizeof(qpup_can_database_axis_0_##MSG_NAME##_t)) {                                  \
      ROS_ERROR_STREAM_NAMED(logger_, "Size mismatch between canframe size and requested decoded size");                       \
    }                                                                                                                \
    if (!qpup_can_database_axis_0_##MSG_NAME##_unpack(std::get_if<qpup_can_database_axis_0_##MSG_NAME##_t>(DST_PTR), \
                                                      SRC_CAN_FRAME.data, SRC_CAN_FRAME.can_dlc)) {                  \
      ROS_ERROR_NAMED(logger_, "Failed to unpack message via qpup_can_database_axis_0##MSG_NAME##_unpack()");        \
    }                                                                                                                \
  } while (0);

// clang-format on

namespace qpup_utils {

class QPUP_CAN {
  using received_CAN_data =
      std::variant<ODRIVE_RECEIVED_TYPES(0), ODRIVE_RECEIVED_TYPES(1), ODRIVE_RECEIVED_TYPES(2),
                   ODRIVE_RECEIVED_TYPES(3), ODRIVE_RECEIVED_TYPES(4), ODRIVE_RECEIVED_TYPES(5),
                   ODRIVE_RECEIVED_TYPES(6), ODRIVE_RECEIVED_TYPES(7), ODRIVE_RECEIVED_TYPES(8),
                   ODRIVE_RECEIVED_TYPES(9), ODRIVE_RECEIVED_TYPES(10), ODRIVE_RECEIVED_TYPES(11)>;

 public:
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
  bool writeRTRFrame(canid_t msg_id, uint8_t size);
  bool writeFrame(canid_t msg_id, uint8_t* data, uint8_t size);

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
};

}  // namespace qpup_utils
