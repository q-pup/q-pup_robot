#include "qpup_utils/qpup_can.hpp"

#include "qpup_utils/qpup_utils.hpp"

namespace qpup_utils {

QPUP_CAN::QPUP_CAN(int default_endianness, std::string can_interface_name)
    : logger_(qpup_utils::getLoggerName() + "/can"), can_interface_name_(std::move(can_interface_name)) {
  switch (default_endianness) {
    case __ORDER_LITTLE_ENDIAN__:
      break;
    case __ORDER_BIG_ENDIAN__:
      // FIXME
      throw std::invalid_argument("__ORDER_BIG_ENDIAN__ is not supported.");
    case __ORDER_PDP_ENDIAN__:
      throw std::invalid_argument("__ORDER_PDP_ENDIAN__ is not supported.");
    default:
      throw std::invalid_argument("Endianness(" + std::to_string(default_endianness) + ") is not supported.");
  }
  default_endianness_ = default_endianness;
  internal_state_ = state::UNCONFIGURED;
}

QPUP_CAN::~QPUP_CAN() {
  if (internal_state_ != state::FINALIZED) {
    ROS_ERROR_STREAM_NAMED(logger_, "destructor called from non-finalized state: " << internal_state_);
    // Do not prevent cleanup if in wrong state
  }

  if (read_thread_.joinable()) {
    // Usually read_thread should already be joined in from call to cleanup()
    ROS_WARN_STREAM_NAMED(logger_, "read_thread joined in destructor instead of in cleanup()");
    read_thread_.join();
  }
}

bool QPUP_CAN::configure() {
  if (internal_state_ != state::UNCONFIGURED) {
    ROS_ERROR_STREAM_NAMED(logger_, "configure() called from invalid state: " << internal_state_);
    internal_state_ = state::ERROR;
    return false;
  }

  // Create CAN socket
  socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_handle_ < 0) {
    ROS_ERROR_STREAM_NAMED(logger_, "Failed to create CAN socket: " << strerror(errno));
    internal_state_ = state::ERROR;
    return false;
  }

  // Find interface index from interface name
  struct ifreq ifr;
  strncpy(ifr.ifr_name, can_interface_name_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_handle_, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR_STREAM_NAMED(logger_, "Failed to find interface index from CAN interface name("
                                        << can_interface_name_ << "): " << strerror(errno));
    internal_state_ = state::ERROR;
    return false;
  }

  // TODO: filters
  //  // set software filters on can_id
  //  std::vector<struct can_filter> filters;
  //  filters.resize(ids.size());
  //  for (int i = 0; i < ids.size(); i++) {
  //    filters[i].can_id = (canid_t)ids[i];
  //    filters[i].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  //  }
  //  if (setsockopt(socket_handle_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) *
  //  filters.size()) < 0) {
  //    ROS_WARN_STREAM_NAMED(logger_, "Failed to set CAN filters: " << strerror(errno));
  //  }

  // set receive timeout to be very small, so it's not blocking
  struct timeval timeout {};
  timeout.tv_usec = 1;
  if (setsockopt(socket_handle_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    ROS_WARN_STREAM_NAMED(logger_, "Failed to create set CAN socket recv timeout: " << strerror(errno));
  }

  // bind socket to can addr
  struct sockaddr_can sock_addr;
  sock_addr.can_family = AF_CAN;
  sock_addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_handle_, reinterpret_cast<struct sockaddr*>(&sock_addr), sizeof(sock_addr)) < 0) {
    ROS_ERROR_STREAM_NAMED(
        logger_, "Failed to bind socket to CAN interface(" << can_interface_name_ << "): " << strerror(errno));
    internal_state_ = state::ERROR;
    return false;
  }

  // start read thread
  read_thread_ = std::thread(&QPUP_CAN::readSocketTask, this);
  run_read_thread_.test_and_set();

  internal_state_ = state::INACTIVE;
  return true;
}

bool QPUP_CAN::activate() {
  if (internal_state_ != state::INACTIVE) {
    ROS_ERROR_STREAM_NAMED(logger_, "activate() called from invalid state: " << internal_state_);
    internal_state_ = state::ERROR;
    return false;
  }
  // start allowing sending of CAN messages
  internal_state_ = state::ACTIVE;
  return true;
}

bool QPUP_CAN::deactivate() {
  if (internal_state_ != state::ACTIVE) {
    ROS_ERROR_STREAM_NAMED(logger_, "deactivate() called from invalid state: " << internal_state_);
    internal_state_ = state::ERROR;
    return false;
  }

  internal_state_ = state::INACTIVE;
  return true;
}

bool QPUP_CAN::cleanup() {
  if (internal_state_ != state::INACTIVE) {
    ROS_ERROR_STREAM_NAMED(logger_, "cleanup() called from invalid state: " << internal_state_);
    internal_state_ = state::ERROR;
    return false;
  }

  run_read_thread_.clear();

  bool success = true;
  if (read_thread_.joinable()) {
    read_thread_.join();
  } else {
    ROS_ERROR_STREAM_NAMED(logger_, "failed to join read_thread in cleanup()");
    success &= false;
  }

  {
    std::lock_guard<std::timed_mutex> lock_guard(recv_map_mtx_);
    recv_map_.clear();
  }

  if (close(socket_handle_) < 0) {
    ROS_ERROR_STREAM_NAMED(logger_, "failed to close socket cleanup(): " << strerror(errno));
    success &= false;
  }

  return success;
}

std::optional<QPUP_CAN::received_CAN_data> QPUP_CAN::getLatestValue(canid_t msg_id) {
  if (internal_state_ != state::INACTIVE && internal_state_ != state::ACTIVE) {
    ROS_ERROR_STREAM_NAMED(logger_, "getLatestValue() called from invalid state: " << internal_state_);
    internal_state_ = state::ERROR;
    return std::nullopt;
  }

  try {
    return std::optional<received_CAN_data>{*recv_map_.at(msg_id).readFromRT()};
  } catch (const std::out_of_range&) {
    ROS_WARN_STREAM_NAMED(logger_, "No received data for msg_id: " << std::showbase << std::hex << msg_id);
    return std::nullopt;
  }
}

bool QPUP_CAN::writeODriveRTRFrame(canid_t msg_id) {
  return QPUP_CAN::writeRTRFrame(msg_id, QPUP_CAN::ODRIVE_RTR_DLC);
}

bool QPUP_CAN::writeRTRFrame(canid_t msg_id, uint8_t size) {
  if (internal_state_ != state::ACTIVE) {
    ROS_ERROR_STREAM_NAMED(logger_, "writeRTRFrame() called from invalid state: " << internal_state_);
    internal_state_ = state::ERROR;
    return false;
  }
  return QPUP_CAN::writeFrame(msg_id | CAN_RTR_FLAG, nullptr, size);
}

bool QPUP_CAN::writeFrame(canid_t msg_id, uint8_t* data, uint8_t size) {
  if (internal_state_ != state::ACTIVE) {
    ROS_ERROR_STREAM_NAMED(logger_, "writeRTRFrame() called from invalid state: " << internal_state_);
    internal_state_ = state::ERROR;
    return false;
  }

  if (size > CAN_MAX_DLEN) {
    ROS_ERROR_STREAM_NAMED(logger_, "Tried to write data of invalid size(" << size << " bytes) to can id "
                                                                           << std::showbase << std::hex << msg_id);
    internal_state_ = state::ERROR;
    return false;
  }

  struct can_frame frame {};
  frame.can_id = msg_id;
  frame.can_dlc = size;
  if (data != nullptr) {
    std::memcpy(frame.data, data, size);
  }

  // TODO non-blocking send. silently blocks thread when no other can nodes present on the bus
  const int bytes_sent = send(socket_handle_, &frame, sizeof(struct can_frame), 0);
  if (bytes_sent != sizeof(struct can_frame)) {
    ROS_ERROR_STREAM_NAMED(logger_, "Failed to send all " << sizeof(struct can_frame) << " bytes of msg to "
                                                          << std::showbase << std::hex << frame.can_id << ". Only sent "
                                                          << std::dec << bytes_sent
                                                          << " bytes. Error: " << strerror(errno));
    internal_state_ = state::ERROR;
    return false;
  }

  return true;
}

std::string QPUP_CAN::getLogger(){
  return logger_;
}

void QPUP_CAN::readSocketTask() {
  struct can_frame frame {};
  int bytes_read;

  ROS_INFO_STREAM_NAMED(logger_, "CAN Read thread started!");

  while (run_read_thread_.test_and_set()) {
    do {  // Read Until CAN Buffer is emptied
      bytes_read = recv(socket_handle_, &frame, sizeof(struct can_frame), 0);
      if (bytes_read == sizeof(struct can_frame)) {
        received_CAN_data data;

        switch (frame.can_id & CAN_ERR_MASK) {
          CASE_ODRIVE_IDS(HEARTBEAT) : {
            UNPACK_ODRIVE_MSG_STRUCT(heartbeat, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_MOTOR_ERROR) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_motor_error, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_ENCODER_ERROR) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_encoder_error, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_SENSORLESS_ERROR) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_sensorless_error, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_ENCODER_ESTIMATES) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_encoder_estimates, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_ENCODER_COUNT) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_encoder_count, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_IQ) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_iq, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_SENSORLESS_ESTIMATES) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_sensorless_estimates, data, frame);
            break;
          }
          CASE_ODRIVE_IDS(GET_VBUS_VOLTAGE) : {
            UNPACK_ODRIVE_MSG_STRUCT(get_vbus_voltage, data, frame);
            break;
          }

          case 0x81109216:
            // FIXME: Battery UAVCAN frame not handled
            break;

          default:
            ROS_ERROR_STREAM_NAMED(
                logger_, "CAN wrapper encountered unhandled CAN ID: " << std::showbase << std::hex << frame.can_id);
        }

        if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
          ROS_WARN_NAMED(logger_, "CAN wrapper failed to lock mutex. CAN data lost!");
          break;
        }
        std::lock_guard<std::timed_mutex> lock_guard(recv_map_mtx_, std::adopt_lock);

        canid_t msg_id = frame.can_id & CAN_ERR_MASK;
        try {
          recv_map_.at(msg_id).writeFromNonRT(data);
        } catch (const std::out_of_range&) {
          // create map entry on first receipt
          recv_map_.emplace(std::make_pair(msg_id, realtime_tools::RealtimeBuffer(data)));
        }

      } else if (bytes_read < 0 && errno != EWOULDBLOCK) {
        ROS_WARN_STREAM_NAMED(logger_, "CAN wrapper failed to receive CAN message because: " << strerror(errno));
      }
    } while (bytes_read == sizeof(struct can_frame));

    // Allow other threads to run
    std::this_thread::yield();
  }
}

}  // namespace qpup_utils
