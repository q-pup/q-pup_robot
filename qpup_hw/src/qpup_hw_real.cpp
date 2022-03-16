#include "qpup_hw/qpup_hw_real.hpp"

#include <memory>
#include <unordered_set>

#include "pluginlib/class_list_macros.hpp"
#include "qpup_utils/qpup_params.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace qpup_hw {

// clang-format off
const std::unordered_set<std::string> DISCONNECTED_JOINT_LIST{
   "lf_hip_joint",
   "lf_upper_leg_joint",
   "lf_lower_leg_joint",
   "rf_hip_joint",
   "rf_upper_leg_joint",
   "rf_lower_leg_joint",
   "lh_hip_joint",
   "lh_upper_leg_joint",
   "lh_lower_leg_joint",
//    "rh_hip_joint",
//    "rh_upper_leg_joint",
//    "rh_lower_leg_joint",
};
// clang-format on

bool QPUPHWReal::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  if (!QPUPHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  if (!loadOdriveConfigFromParamServer(robot_hw_nh)) {
    return false;
  }

  imu_ = std::make_unique<qpup_hw::navx::AHRS>(
      qpup_utils::getParam<std::string>(root_nh, logger_, "imu_interface_name", "/dev/ttyACM0"));
  can_ = std::make_unique<qpup_utils::QPUP_CAN>(
      __BYTE_ORDER__, qpup_utils::getParam<std::string>(root_nh, logger_, "can_interface_name", "can0"));

  if (!(can_->configure() && can_->activate())) {
    return false;
  }

  return updatePIDGains();
}

QPUPHWReal::~QPUPHWReal() {
  can_->deactivate();
  can_->cleanup();
}

void QPUPHWReal::read(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  for (const auto &joint_name : joint_names_) {
    // Note: *_decode is not called on any of the below signals since none of them have any offset or scaling so it
    // doesn't do anything. They're all encoded as floats, or integers with no scaling nor offset.

    // The following RTR Requests are not used:
    //    QPUP_ODRIVE_GET_SENSORLESS_ESTIMATES_FRAME_ID

    // Send RTR Requests
    can_->writeODriveRTRFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                          QPUP_ODRIVE_GET_MOTOR_ERROR_FRAME_ID));
    can_->writeODriveRTRFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                          QPUP_ODRIVE_GET_ENCODER_ERROR_FRAME_ID));
    can_->writeODriveRTRFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                          QPUP_ODRIVE_GET_SENSORLESS_ERROR_FRAME_ID));
    can_->writeODriveRTRFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                          QPUP_ODRIVE_GET_ENCODER_COUNT_FRAME_ID));
    can_->writeODriveRTRFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                          QPUP_ODRIVE_GET_IQ_FRAME_ID));
    can_->writeODriveRTRFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                          QPUP_ODRIVE_GET_VBUS_VOLTAGE_FRAME_ID));

    // Read Streamed Data
    const auto heartbeat_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_HEARTBEAT_FRAME_ID));
    if (heartbeat_frame.has_value()) {
      const auto heartbeat = std::get<qpup_odrive_heartbeat_t>(heartbeat_frame.value());

      odrive_state_data_[joint_name].axis_error = heartbeat.axis_error;
      odrive_state_data_[joint_name].axis_state = heartbeat.axis_state;
      odrive_state_data_[joint_name].motor_flags = heartbeat.motor_flags;
      odrive_state_data_[joint_name].encoder_flags = heartbeat.encoder_flags;
      odrive_state_data_[joint_name].controller_flags = heartbeat.controller_flags;

      ROS_WARN_STREAM_COND_NAMED(heartbeat.axis_error != 0, logger_,
                                 joint_name << " axis_error: " << std::showbase << std::hex << heartbeat.axis_error);
    }

    const auto encoder_estimates_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_GET_ENCODER_ESTIMATES_FRAME_ID));
    if (encoder_estimates_frame.has_value()) {
      const auto encoder_estimates = std::get<qpup_odrive_get_encoder_estimates_t>(encoder_estimates_frame.value());

      actuator_joint_states_[joint_name].actuator_velocity = encoder_estimates.vel_estimate * RADIANS_PER_ROTATION;
      actuator_joint_states_[joint_name].actuator_position = encoder_estimates.pos_estimate * RADIANS_PER_ROTATION;
    }

    // Read RTR Responses
    const auto motor_errors_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_GET_MOTOR_ERROR_FRAME_ID));
    if (motor_errors_frame.has_value()) {
      const auto motor_errors = std::get<qpup_odrive_get_motor_error_t>(motor_errors_frame.value());

      odrive_state_data_[joint_name].motor_error = motor_errors.motor_error;

      ROS_WARN_STREAM_COND_NAMED(
          motor_errors.motor_error != 0, logger_,
          joint_name << " motor_errors: " << std::showbase << std::hex << motor_errors.motor_error);
    }

    const auto encoder_errors_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_GET_ENCODER_ERROR_FRAME_ID));
    if (encoder_errors_frame.has_value()) {
      const auto encoder_errors = std::get<qpup_odrive_get_encoder_error_t>(encoder_errors_frame.value());

      odrive_state_data_[joint_name].encoder_error = encoder_errors.encoder_error;

      ROS_WARN_STREAM_COND_NAMED(
          encoder_errors.encoder_error != 0, logger_,
          joint_name << " motor_errors: " << std::showbase << std::hex << encoder_errors.encoder_error);
    }

    const auto sensorless_errors_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_GET_SENSORLESS_ERROR_FRAME_ID));
    if (sensorless_errors_frame.has_value()) {
      const auto sensorless_errors = std::get<qpup_odrive_get_sensorless_error_t>(sensorless_errors_frame.value());

      odrive_state_data_[joint_name].sensorless_error = sensorless_errors.sensorless_error;

      ROS_WARN_STREAM_COND_NAMED(
          sensorless_errors.sensorless_error != 0, logger_,
          joint_name << " sensorless_errors: " << std::showbase << std::hex << sensorless_errors.sensorless_error);
    }

    const auto encoder_count_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_GET_ENCODER_COUNT_FRAME_ID));
    if (encoder_count_frame.has_value()) {
      const auto encoder_count = std::get<qpup_odrive_get_encoder_count_t>(encoder_count_frame.value());

      odrive_state_data_[joint_name].shadow_count = encoder_count.shadow_count;
      odrive_state_data_[joint_name].count_in_cpr = encoder_count.count_in_cpr;
    }

    const auto iq_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_GET_IQ_FRAME_ID));
    if (iq_frame.has_value()) {
      const auto iq = std::get<qpup_odrive_get_iq_t>(iq_frame.value());

      actuator_joint_states_[joint_name].actuator_effort = iq.iq_measured * TORQUE_CONSTANT;

      odrive_state_data_[joint_name].iq_setpoint = iq.iq_setpoint;
      odrive_state_data_[joint_name].iq_measured = iq.iq_measured;
    }

    const auto vbus_voltage_frame = can_->getLatestValue(qpup_utils::QPUP_CAN::getOdriveCANCommandId(
        odrive_axis_params_[joint_name].can_id, QPUP_ODRIVE_GET_VBUS_VOLTAGE_FRAME_ID));
    if (vbus_voltage_frame.has_value()) {
      const auto vbus_voltage = std::get<qpup_odrive_get_vbus_voltage_t>(vbus_voltage_frame.value());

      odrive_state_data_[joint_name].vbus_voltage = vbus_voltage.vbus_voltage;
    }

    if (DISCONNECTED_JOINT_LIST.find(joint_name) != DISCONNECTED_JOINT_LIST.end()) {
      actuator_joint_states_[joint_name].actuator_velocity = 0;
      actuator_joint_states_[joint_name].actuator_position = actuator_joint_commands_[joint_name].actuator_data;
    }
  }
  actuator_to_joint_state_interface_.propagate();

  // Read IMU state
  // TODO(mreynolds): Hardcoded name
  {
    tf2::Quaternion quat;
    quat.setRPY(imu_->GetRoll() / -180.0 * M_PI, imu_->GetPitch() / -180.0 * M_PI,
                imu_->GetFusedHeading() / 180.0 * M_PI /* + offset*/);
    imu_states_["imu"].orientation[0] = quat.x();
    imu_states_["imu"].orientation[1] = quat.y();
    imu_states_["imu"].orientation[2] = quat.z();
    imu_states_["imu"].orientation[3] = quat.w();

    imu_states_["imu"].angular_velocity[0] = imu_->GetRawGyroX() / 180.0 * M_PI;
    imu_states_["imu"].angular_velocity[1] = imu_->GetRawGyroY() / 180.0 * M_PI;
    imu_states_["imu"].angular_velocity[2] = imu_->GetRawGyroZ() / 180.0 * M_PI;

    imu_states_["imu"].linear_acceleration[0] = imu_->GetRawAccelX() * 9.81;
    imu_states_["imu"].linear_acceleration[1] = imu_->GetRawAccelY() * 9.81;
    imu_states_["imu"].linear_acceleration[2] = imu_->GetRawAccelZ() * 9.81;
  }
}

void QPUPHWReal::write(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  for (const auto &joint_name : joint_names_) {
    if (DISCONNECTED_JOINT_LIST.find(joint_name) != DISCONNECTED_JOINT_LIST.end()) {
      continue;
    }

    // Clear Errors Command
    if (odrive_state_data_[joint_name].clear_errors) {
      qpup_odrive_clear_errors_t clear_errors_message{};

      if (can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                       QPUP_ODRIVE_CLEAR_ERRORS_FRAME_ID),
                           &clear_errors_message, 0)) {
        ROS_INFO_STREAM_NAMED(logger_, "Cleared Odrive Errors on " << joint_name << "!");
      } else {
        ROS_ERROR_STREAM_NAMED(logger_, "Failed to write clear errors command to " << joint_name << ".");
      }
    }

    uint8_t outgoing_can_data_buffer[CAN_MAX_DLEN];

    // Controller Mode Command (Control Mode + Input Mode)
    if (odrive_state_data_[joint_name].control_mode_cmd != last_control_mode_ ||
        odrive_state_data_[joint_name].input_mode_cmd != last_input_mode_) {
      // Encode Signals
      qpup_odrive_set_controller_mode_t set_controller_mode_message{};
      set_controller_mode_message.control_mode = odrive_state_data_[joint_name].control_mode_cmd;
      set_controller_mode_message.input_mode = odrive_state_data_[joint_name].input_mode_cmd;

      // Pack Message
      const int message_size = qpup_odrive_set_controller_mode_pack(
          outgoing_can_data_buffer, &set_controller_mode_message, qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC);

      if (message_size < 0) {
        ROS_ERROR_STREAM_NAMED(logger_, "CAN Packing Error...");
      } else if (message_size != qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC) {
        ROS_ERROR_STREAM_NAMED(logger_, "Mis-match in packed size. Expected: "
                                            << qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC << " Got: " << message_size);
      } else {  // message_size == qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC

        if (can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                         QPUP_ODRIVE_SET_CONTROLLER_MODE_FRAME_ID),
                             outgoing_can_data_buffer, message_size)) {
          ROS_INFO_STREAM_NAMED(logger_, "Set Controller Mode to : ControlMode("
                                             << odrive_state_data_[joint_name].control_mode_cmd << ") InputMode("
                                             << odrive_state_data_[joint_name].input_mode_cmd << ") on " << joint_name
                                             << "!");
        } else {
          ROS_ERROR_STREAM_NAMED(logger_, "Failed to set Controller Mode to : ControlMode("
                                              << odrive_state_data_[joint_name].control_mode_cmd << ") InputMode("
                                              << odrive_state_data_[joint_name].input_mode_cmd << ") on " << joint_name
                                              << "!");
        }
      }
    }
    last_control_mode_ = odrive_state_data_[joint_name].control_mode_cmd;
    last_input_mode_ = odrive_state_data_[joint_name].input_mode_cmd;

    // Axis State Command
    if (odrive_state_data_[joint_name].axis_state_cmd != odrive_state_data_[joint_name].axis_state) {
      // Encode Signals
      qpup_odrive_set_axis_state_t set_axis_state_message{};
      set_axis_state_message.axis_requested_state = odrive_state_data_[joint_name].axis_state_cmd;

      // Pack Message
      const int message_size = qpup_odrive_set_axis_state_pack(outgoing_can_data_buffer, &set_axis_state_message,
                                                               qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC);

      if (message_size < 0) {
        ROS_ERROR_STREAM_NAMED(logger_, "CAN Packing Error...");
      } else if (message_size != qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC) {
        ROS_ERROR_STREAM_NAMED(logger_, "Mis-match in packed size. Expected: " << qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC
                                                                               << " Got: " << message_size);
      } else {  // message_size == qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC

        if (can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                         QPUP_ODRIVE_SET_AXIS_STATE_FRAME_ID),
                             outgoing_can_data_buffer, message_size)) {
          ROS_INFO_STREAM_NAMED(logger_, "Set Axis State to " << odrive_state_data_[joint_name].axis_state_cmd << " on "
                                                              << joint_name << "!");
        } else {
          ROS_ERROR_STREAM_NAMED(logger_, "Failed to set Axis State to "
                                              << odrive_state_data_[joint_name].axis_state_cmd << " on " << joint_name
                                              << "!");
        }
      }
    }

    bool successful_joint_write = false;
    
    // Movement Commands (Only send in closed loop mode)
    if (odrive_state_data_[joint_name].axis_state == odrive_state_msgs::SetAxisState::Request::Type::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      // TODO: range checks before encoding signals
      switch (actuator_joint_commands_[joint_name].type) {
        case QPUPHW::QPUPActuatorJointCommand::Type::POSITION: {
          joint_to_actuator_position_interface_.propagate();

          // Encode Signals
          qpup_odrive_set_input_pos_t set_input_pos_message{};
          set_input_pos_message.input_pos = qpup_odrive_set_input_pos_input_pos_encode(
              actuator_joint_commands_[joint_name].actuator_data / RADIANS_PER_ROTATION);
          set_input_pos_message.vel_ff = qpup_odrive_set_input_pos_vel_ff_encode(0 / RADIANS_PER_ROTATION);
          set_input_pos_message.torque_ff = qpup_odrive_set_input_pos_torque_ff_encode(0);

          // Pack Message
          const int message_size = qpup_odrive_set_input_pos_pack(outgoing_can_data_buffer, &set_input_pos_message,
                                                                  qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC);

          if (message_size < 0) {
            ROS_ERROR_STREAM_NAMED(logger_, "CAN Packing Error...");
          } else if (message_size != qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC) {
            ROS_ERROR_STREAM_NAMED(logger_, "Mis-match in packed size. Expected: " << qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC
                                                                                  << " Got: " << message_size);
          } else {  // message_size == qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC

            successful_joint_write =
                can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                            QPUP_ODRIVE_SET_INPUT_POS_FRAME_ID),
                                outgoing_can_data_buffer, message_size);
          }
          break;
        }

        case QPUPHW::QPUPActuatorJointCommand::Type::VELOCITY: {
          joint_to_actuator_velocity_interface_.propagate();

          // Encode Signals
          qpup_odrive_set_input_vel_t set_input_vel_message{};
          set_input_vel_message.input_vel = qpup_odrive_set_input_vel_input_vel_encode(
              actuator_joint_commands_[joint_name].actuator_data / RADIANS_PER_ROTATION);
          set_input_vel_message.input_torque_ff = qpup_odrive_set_input_vel_input_torque_ff_encode(0);

          // Pack Message
          const int message_size = qpup_odrive_set_input_vel_pack(outgoing_can_data_buffer, &set_input_vel_message,
                                                                  qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC);

          if (message_size < 0) {
            ROS_ERROR_STREAM_NAMED(logger_, "CAN Packing Error...");
          } else if (message_size != qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC) {
            ROS_ERROR_STREAM_NAMED(logger_, "Mis-match in packed size. Expected: " << qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC
                                                                                  << " Got: " << message_size);
          } else {  // message_size == qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC

            successful_joint_write =
                can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                            QPUP_ODRIVE_SET_INPUT_VEL_FRAME_ID),
                                outgoing_can_data_buffer, message_size);
          }

          break;
        }

        case QPUPHW::QPUPActuatorJointCommand::Type::EFFORT: {
          joint_to_actuator_velocity_interface_.propagate();

          // Encode Signals
          qpup_odrive_set_input_torque_t set_input_torque_message{};
          set_input_torque_message.input_torque =
              qpup_odrive_set_input_vel_input_vel_encode(actuator_joint_commands_[joint_name].actuator_data);

          // Pack Message
          const int message_size = qpup_odrive_set_input_torque_pack(outgoing_can_data_buffer, &set_input_torque_message,
                                                                    qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC);

          if (message_size < 0) {
            ROS_ERROR_STREAM_NAMED(logger_, "CAN Packing Error...");
          } else if (message_size != qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC) {
            ROS_ERROR_STREAM_NAMED(logger_, "Mis-match in packed size. Expected: "
                                                << qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC << " Got: " << message_size);
          } else {  // message_size == qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC

            successful_joint_write =
                can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                            QPUP_ODRIVE_SET_INPUT_TORQUE_FRAME_ID),
                                outgoing_can_data_buffer, message_size);
          }

          break;
        }

        case QPUPHW::QPUPActuatorJointCommand::Type::NONE: {
          ROS_DEBUG_STREAM_NAMED(logger_, joint_name << " has a " << actuator_joint_commands_[joint_name].type
                                                    << " command type. Sending Stop Command to Motors!");
          //      successful_joint_write = //todo: stop motors command?
          break;
        }

        default: {
          ROS_ERROR_STREAM_NAMED(logger_, joint_name << " has a joint command with index "
                                                    << static_cast<int>(actuator_joint_commands_[joint_name].type)
                                                    << ",which is an unknown command type. Sending "
                                                        "Stop Command to Roboteq Controller!");
          //      successful_joint_write = //todo: stop motors command?}
        }
      }
      if (!successful_joint_write &&
          actuator_joint_commands_[joint_name].type != QPUPHW::QPUPActuatorJointCommand::Type::NONE) {
        ROS_ERROR_STREAM_NAMED(logger_, "Failed to write " << actuator_joint_commands_[joint_name].type << " command to "
                                                          << joint_name << ".");
      }
    }
  }
}

bool QPUPHWReal::loadOdriveConfigFromParamServer(ros::NodeHandle &robot_hw_nh) {
  // Get joint list info
  XmlRpc::XmlRpcValue joints_list;
  bool param_fetched = robot_hw_nh.getParam("joints", joints_list);
  if (!param_fetched) {
    ROS_WARN_STREAM_NAMED(logger_, robot_hw_nh.getNamespace() << "/joints could not be loaded from parameter server.");
    return false;
  }
  ROS_DEBUG_STREAM_NAMED(logger_, robot_hw_nh.getNamespace() << "/joints loaded from parameter server.");
  ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // NOLINTNEXTLINE(modernize-loop-convert): iterator only valid for XmlRpcValue::TypeStruct
  for (int joint_index = 0; joint_index < joints_list.size(); joint_index++) {
    ROS_ASSERT(joints_list[joint_index].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(joints_list[joint_index].hasMember("name"));
    ROS_ASSERT(joints_list[joint_index]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string joint_name = joints_list[joint_index]["name"];

    ROS_ASSERT(joints_list[joint_index].hasMember("odrive_axis_can_node_id"));
    ROS_ASSERT(joints_list[joint_index]["odrive_axis_can_node_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    odrive_axis_params_[joint_name].can_id = static_cast<int>(joints_list[joint_index]["odrive_axis_can_node_id"]);

    ROS_ASSERT(joints_list[joint_index].hasMember("pos_gain"));
    ROS_ASSERT(joints_list[joint_index]["pos_gain"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    odrive_axis_params_[joint_name].pos_gain = static_cast<double>(joints_list[joint_index]["pos_gain"]);

    ROS_ASSERT(joints_list[joint_index].hasMember("vel_gain"));
    ROS_ASSERT(joints_list[joint_index]["vel_gain"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    odrive_axis_params_[joint_name].vel_gain = static_cast<double>(joints_list[joint_index]["vel_gain"]);

    ROS_ASSERT(joints_list[joint_index].hasMember("vel_integrator_gain"));
    ROS_ASSERT(joints_list[joint_index]["vel_integrator_gain"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    odrive_axis_params_[joint_name].vel_integrator_gain =
        static_cast<double>(joints_list[joint_index]["vel_integrator_gain"]);
  }
  return true;
}

bool QPUPHWReal::updatePIDGains() {
  bool successful_joint_write = true;

  uint8_t outgoing_can_data_buffer[CAN_MAX_DLEN];

  for (const auto &joint_name : joint_names_) {
    // Encode Signals
    qpup_odrive_set_pos_gain_t set_pos_gain_message{};
    set_pos_gain_message.pos_gain = qpup_odrive_set_pos_gain_pos_gain_encode(odrive_axis_params_[joint_name].pos_gain);

    // Pack Message
    int message_size = qpup_odrive_set_pos_gain_pack(outgoing_can_data_buffer, &set_pos_gain_message,
                                                     qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC);

    if (message_size < 0) {
      ROS_ERROR_STREAM_NAMED(logger_, "CAN Packing Error...");
    } else if (message_size != qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC) {
      ROS_ERROR_STREAM_NAMED(logger_, "Mis-match in packed size. Expected: " << qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC
                                                                             << " Got: " << message_size);
    } else {  // message_size == qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC

      successful_joint_write &=
          can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                       QPUP_ODRIVE_SET_POS_GAIN_FRAME_ID),
                           outgoing_can_data_buffer, message_size);
    }

    // Encode Signals
    qpup_odrive_set_vel_gains_t set_vel_gains_message{};
    set_vel_gains_message.vel_gain =
        qpup_odrive_set_vel_gains_vel_gain_encode(odrive_axis_params_[joint_name].vel_gain);
    set_vel_gains_message.vel_integrator_gain =
        qpup_odrive_set_vel_gains_vel_integrator_gain_encode(odrive_axis_params_[joint_name].vel_integrator_gain);

    // Pack Message
    message_size = qpup_odrive_set_vel_gains_pack(outgoing_can_data_buffer, &set_vel_gains_message,
                                                  qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC);

    if (message_size < 0) {
      ROS_ERROR_STREAM_NAMED(logger_, "CAN Packing Error...");
    } else if (message_size != qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC) {
      ROS_ERROR_STREAM_NAMED(logger_, "Mis-match in packed size. Expected: " << qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC
                                                                             << " Got: " << message_size);
    } else {  // message_size == qpup_utils::QPUP_CAN::ODRIVE_CMD_WITH_DATA_DLC

      successful_joint_write &=
          can_->writeFrame(qpup_utils::QPUP_CAN::getOdriveCANCommandId(odrive_axis_params_[joint_name].can_id,
                                                                       QPUP_ODRIVE_SET_VEL_GAINS_FRAME_ID),
                           outgoing_can_data_buffer, message_size);
    }
  }

  return successful_joint_write;
}

}  // namespace qpup_hw
PLUGINLIB_EXPORT_CLASS(qpup_hw::QPUPHWReal, hardware_interface::RobotHW)
