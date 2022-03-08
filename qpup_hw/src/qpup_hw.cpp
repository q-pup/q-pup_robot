#include "qpup_hw/qpup_hw.hpp"

namespace qpup_hw {

bool QPUPHW::init(ros::NodeHandle & /*root_nh*/, ros::NodeHandle &robot_hw_nh) {
  if (!loadJointInfoFromParameterServer(robot_hw_nh)) {
    return false;
  }

  for (const auto &joint_name : joint_names_) {
    registerStateInterfacesAndTransmissions(joint_name);
    registerCommandInterfacesAndTransmissions(joint_name);
  }

  // Register ImuSensorHandle
  // TODO(mreynolds): hardcoding
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu",
      "base_link",
      imu_states_["imu"].orientation,
      imu_states_["imu"].orientation_covariance,
      imu_states_["imu"].angular_velocity,
      imu_states_["imu"].angular_velocity_covariance,
      imu_states_["imu"].linear_acceleration,
      imu_states_["imu"].linear_acceleration_covariance));

  this->registerInterface(&joint_state_interface_);
  this->registerInterface(&joint_position_interface_);
  this->registerInterface(&joint_velocity_interface_);
  this->registerInterface(&joint_effort_interface_);
  this->registerInterface(&imu_sensor_interface_);

  return true;
}

void QPUPHW::read(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  ROS_ERROR_STREAM_NAMED(logger_, "read function called from " << logger_
                                                               << " class. read calls should only happen "
                                                                  "to the overloaded function.");
}

void QPUPHW::write(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  ROS_ERROR_STREAM_NAMED(logger_, "write function called from " << logger_
                                                                << " class. write calls should only happen "
                                                                   "to the overloaded function.");
}

void QPUPHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                      const std::list<hardware_interface::ControllerInfo> &stop_list) {
  // Reset commands for joints claimed by stopping controllers
  for (const auto &controller : stop_list) {
    for (const auto &hardware_interface_resource_list : controller.claimed_resources) {
      for (const auto &joint_name : hardware_interface_resource_list.resources) {
        actuator_joint_commands_[joint_name].type = QPUPActuatorJointCommand::Type::NONE;
        actuator_joint_commands_[joint_name].joint_data = 0.0;
      }
    }
  }

  // Set command type for joints claimed by starting controllers
  for (const auto &controller : start_list) {
    for (const auto &claimed : controller.claimed_resources) {
      for (const auto &joint_name : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          actuator_joint_commands_[joint_name].type = QPUPActuatorJointCommand::Type::POSITION;
          actuator_joint_commands_[joint_name].joint_data = actuator_joint_states_[joint_name].joint_position;
        } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
          actuator_joint_commands_[joint_name].type = QPUPActuatorJointCommand::Type::VELOCITY;
          actuator_joint_commands_[joint_name].joint_data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface") {
          actuator_joint_commands_[joint_name].type = QPUPActuatorJointCommand::Type::EFFORT;
          actuator_joint_commands_[joint_name].joint_data = 0.0;
        }
      }
    }
  }

  // TODO: doswitch to change control modes on odrive (to go to and from open
  // loop mode)
}

bool QPUPHW::loadJointInfoFromParameterServer(ros::NodeHandle &robot_hw_nh) {
  XmlRpc::XmlRpcValue joints_list;
  bool param_fetched = robot_hw_nh.getParam("joints", joints_list);
  if (!param_fetched) {
    ROS_WARN_STREAM_NAMED(logger_, robot_hw_nh.getNamespace() << "/joints could not be loaded from parameter server.");
    return false;
  }
  ROS_DEBUG_STREAM_NAMED(logger_, robot_hw_nh.getNamespace() << "/joints loaded from parameter server.");
  ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // NOLINTNEXTLINE(modernize-loop-convert): iterator only valid for
  // XmlRpcValue::TypeStruct
  for (int joint_index = 0; joint_index < joints_list.size(); joint_index++) {
    ROS_ASSERT(joints_list[joint_index].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(joints_list[joint_index].hasMember("name"));
    ROS_ASSERT(joints_list[joint_index]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string joint_name = joints_list[joint_index]["name"];
    joint_names_.push_back(joint_name);
    ROS_ASSERT(joints_list[joint_index].hasMember("transmission_reduction"));
    ROS_ASSERT(joints_list[joint_index]["transmission_reduction"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    joint_transmissions_.emplace(joint_name, joints_list[joint_index]["transmission_reduction"]);
  }
  return true;
}

void QPUPHW::registerStateInterfacesAndTransmissions(const std::string &joint_name) {
  // Register JointStateHandle to the JointStateInterface
  hardware_interface::JointStateHandle joint_state_handle(
      joint_name, &actuator_joint_states_[joint_name].joint_position,
      &actuator_joint_states_[joint_name].joint_velocity, &actuator_joint_states_[joint_name].joint_effort);
  joint_state_interface_.registerHandle(joint_state_handle);

  // Wrap Actuators States
  actuator_state_data_[joint_name].position.push_back(&actuator_joint_states_[joint_name].actuator_position);
  actuator_state_data_[joint_name].velocity.push_back(&actuator_joint_states_[joint_name].actuator_velocity);
  actuator_state_data_[joint_name].effort.push_back(&actuator_joint_states_[joint_name].actuator_effort);

  // Wrap Joint States
  joint_state_data_[joint_name].position.push_back(&actuator_joint_states_[joint_name].joint_position);
  joint_state_data_[joint_name].velocity.push_back(&actuator_joint_states_[joint_name].joint_velocity);
  joint_state_data_[joint_name].effort.push_back(&actuator_joint_states_[joint_name].joint_effort);

  // Register ActuatorToJointStateHandle with wrapped state data to a
  // ActuatorToJointStateInterface
  transmission_interface::ActuatorToJointStateHandle actuator_to_joint_state_handle(
      joint_name, &joint_transmissions_.find(joint_name)->second, actuator_state_data_[joint_name],
      joint_state_data_[joint_name]);
  actuator_to_joint_state_interface_.registerHandle(actuator_to_joint_state_handle);

  // TODO: register new state interface for motor voltage, current, etc.
}

void QPUPHW::registerCommandInterfacesAndTransmissions(const std::string &joint_name) {
  // Register JointHandle associated with a JointStateHandle to command
  // interfaces
  hardware_interface::JointHandle joint_command_handle(joint_state_interface_.getHandle(joint_name),
                                                       &actuator_joint_commands_[joint_name].joint_data);
  joint_position_interface_.registerHandle(joint_command_handle);
  joint_velocity_interface_.registerHandle(joint_command_handle);
  joint_effort_interface_.registerHandle(joint_command_handle);

  // Wrap Actuator Commands
  actuator_command_data_[joint_name].position.push_back(&actuator_joint_commands_[joint_name].actuator_data);
  actuator_command_data_[joint_name].velocity.push_back(&actuator_joint_commands_[joint_name].actuator_data);
  actuator_command_data_[joint_name].effort.push_back(&actuator_joint_commands_[joint_name].actuator_data);

  // Wrap Joint Commands
  joint_command_data_[joint_name].position.push_back(&actuator_joint_commands_[joint_name].joint_data);
  joint_command_data_[joint_name].velocity.push_back(&actuator_joint_commands_[joint_name].joint_data);
  joint_command_data_[joint_name].effort.push_back(&actuator_joint_commands_[joint_name].joint_data);

  // Register JointToActuatorPositionHandle with wrapped command data to a
  // JointToActuatorPositionInterface
  transmission_interface::JointToActuatorPositionHandle joint_to_actuator_position_handle(
      joint_name, &joint_transmissions_.find(joint_name)->second, actuator_command_data_[joint_name],
      joint_command_data_[joint_name]);
  joint_to_actuator_position_interface_.registerHandle(joint_to_actuator_position_handle);

  // Register JointToActuatorVelocityHandle with wrapped command data to a
  // JointToActuatorVelocityInterface
  transmission_interface::JointToActuatorVelocityHandle joint_to_actuator_velocity_handle(
      joint_name, &joint_transmissions_.find(joint_name)->second, actuator_command_data_[joint_name],
      joint_command_data_[joint_name]);
  joint_to_actuator_velocity_interface_.registerHandle(joint_to_actuator_velocity_handle);

  // Register JointToActuatorEffortHandle with wrapped command data to a
  // JointToActuatorVelocityInterface
  transmission_interface::JointToActuatorEffortHandle joint_to_actuator_effort_handle(
      joint_name, &joint_transmissions_.find(joint_name)->second, actuator_command_data_[joint_name],
      joint_command_data_[joint_name]);
  joint_to_actuator_effort_interface_.registerHandle(joint_to_actuator_effort_handle);
}

}  // namespace qpup_hw
