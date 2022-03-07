#include "qpup_hw/qpup_hw_real.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "pluginlib/class_list_macros.hpp"

#include <memory>


namespace qpup_hw {

bool QPUPHWReal::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  if (!QPUPHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // TODO(wmmc88): QPUP_CAN

  // TODO(mreynolds): Load interface from yaml
  imu_ = std::make_unique<qpup_hw::navx::AHRS>("/dev/ttyACM0");

  return true;
}

void QPUPHWReal::read(const ros::Time & /*time*/,
                      const ros::Duration & /*period*/) {

  for (const auto &joint_name : joint_names_) {

    // TODO: odrive can
    actuator_joint_states_[joint_name].actuator_velocity = 0;
    actuator_joint_states_[joint_name].actuator_position = 0;
    actuator_joint_states_[joint_name].actuator_effort = 0;

    //  TODO: add interface for motor/encoder/controller fault flags
  }
  actuator_to_joint_state_interface_.propagate();

  //  TODO: add interface for odrive level info like vbus voltage

  // Read IMU state
  // TODO(mreynolds): Hardcoded name
  {
    tf2::Quaternion quat;
    quat.setRPY(imu_->GetRoll() / -180.0 * M_PI,
                imu_->GetPitch() / -180.0 * M_PI,
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

void QPUPHWReal::write(const ros::Time & /*time*/,
                       const ros::Duration & /*period*/) {

  for (const auto &joint_name : joint_names_) {
    bool successful_joint_write = false;
    switch (actuator_joint_commands_[joint_name].type) {
    case QPUPHW::QPUPActuatorJointCommand::Type::POSITION:
      joint_to_actuator_position_interface_.propagate();
      //      successful_joint_write = smth smth
      //      actuator_joint_commands_[joint_name].actuator_data

      break;

    case QPUPHW::QPUPActuatorJointCommand::Type::VELOCITY:
      joint_to_actuator_velocity_interface_.propagate();
      //      successful_joint_write = smth smth
      //      actuator_joint_commands_[joint_name].actuator_data

      break;

    case QPUPHW::QPUPActuatorJointCommand::Type::EFFORT:
      joint_to_actuator_velocity_interface_.propagate();

      actuator_joint_commands_[joint_name].actuator_data =
          actuator_joint_commands_[joint_name].joint_data;

      //      successful_joint_write = smth smth
      //      actuator_joint_commands_[joint_name].actuator_data
      break;

    case QPUPHW::QPUPActuatorJointCommand::Type::NONE:
      ROS_DEBUG_STREAM_NAMED(
          name_, joint_name
                     << " has a " << actuator_joint_commands_[joint_name].type
                     << " command type. Sending Stop Command to Motors!");
      //      successful_joint_write = //todo: stop motors command?
      break;

    default:
      ROS_ERROR_STREAM_NAMED(
          name_, joint_name << " has a joint command with index "
                            << static_cast<int>(
                                   actuator_joint_commands_[joint_name].type)
                            << ",which is an unknown command type. Sending "
                               "Stop Command to Roboteq Controller!");
      //      successful_joint_write = //todo: stop motors command?
    }
    if (!successful_joint_write &&
        actuator_joint_commands_[joint_name].type !=
            QPUPHW::QPUPActuatorJointCommand::Type::NONE) {
      ROS_ERROR_STREAM_NAMED(
          name_, "Failed to write " << actuator_joint_commands_[joint_name].type
                                    << " command to " << joint_name << ".");
    }
  }
}

} // namespace qpup_hw
PLUGINLIB_EXPORT_CLASS(qpup_hw::QPUPHWReal, hardware_interface::RobotHW)
