#include "qpup_hw/qpup_hw_real.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace qpup_hw {

bool QPUPHWReal::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  if (!QPUPHW::init(root_nh, robot_hw_nh)) {
    return false;
  }
  // TODO(wmmc88): QPUP_CAN
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
