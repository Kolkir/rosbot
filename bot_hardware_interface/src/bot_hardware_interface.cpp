#include "bot_hardware_interface.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>

BotHardwareInterface::BotHardwareInterface(ros::NodeHandle& node_handle)
    : name_("hardware_interface"), node_handle_(node_handle) {
  // Parse the URDF for joint names & interfaces, then initialize them
  // Check if the URDF model needs to be loaded
  LoadURDF(node_handle_, "robot_description");

  ros::NodeHandle rpnh(node_handle_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
  error += !rosparam_shortcuts::get(name_, node_handle_,
                                    "mobile_base_controller/wheel_radius",
                                    wheel_radius_);
  error += !rosparam_shortcuts::get(
      name_, node_handle_, "mobile_base_controller/linear/x/max_velocity",
      max_velocity_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  wheel_diameter_ = 2.0 * wheel_radius_;
  // ros_control RobotHW needs velocity in rad/s but in the config its given in
  // m/s
  max_velocity_ = LinearToAngular(max_velocity_);

  // Initialize the hardware interface
  init(node_handle_, node_handle_);
}

bool BotHardwareInterface::init(ros::NodeHandle& /*root_nh*/,
                                ros::NodeHandle& /*robot_hw_nh*/) {
  ROS_INFO("Initializing rosbot Hardware Interface ...");

  num_joints_ = joint_names_.size();
  ROS_INFO("Number of joints: %d", (int)num_joints_);

  joint_positions_.resize(num_joints_);
  joint_velocities_.resize(num_joints_);
  joint_efforts_.resize(num_joints_);
  joint_velocity_commands_.resize(num_joints_);

  for (unsigned int i = 0; i < num_joints_; i++) {
    hardware_interface::JointStateHandle joint_state_handle(
        joint_names_[i], &joint_positions_[i], &joint_velocities_[i],
        &joint_efforts_[i]);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle,
                                                 &joint_velocity_commands_[i]);
    velocity_joint_interface_.registerHandle(joint_handle);

    joint_positions_[i] = 0.0;
    joint_velocities_[i] = 0.0;
    joint_efforts_[i] = 0.0;  // unused with diff_drive_controller

    joint_velocity_commands_[i] = 0.0;
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  ROS_INFO("... Done Initializing rosbot Hardware Interface");
  return true;
}

void BotHardwareInterface::read(const ros::Time& /*time*/
                                ,
                                const ros::Duration& period) {
  // Read from robot hw (motor encoders)
  // Fill joint_state_* members with read values
  double wheel_angles[2] = {0, 0};
  double wheel_angle_deltas[2] = {0, 0};
  for (std::size_t i = 0; i < num_joints_; ++i) {
    wheel_angles[i] = GetMotorAngle(i);
    wheel_angle_deltas[i] = wheel_angles[i] - joint_positions_[i];

    joint_positions_[i] += wheel_angle_deltas[i];
    joint_velocities_[i] = wheel_angle_deltas[i] / period.toSec();
    joint_efforts_[i] = 0.0;  // unused with diff_drive_controller
  }
}

void BotHardwareInterface::write(const ros::Time& /*time*/,
                                 const ros::Duration& period) {
  // Write to robot hw
  // joint velocity commands from ros_control's RobotHW are in rad/s
  for (std::size_t i = 0; i < num_joints_; ++i) {
    SetMotorVelocity(i, joint_velocities_[i], joint_velocity_commands_[i],
                     period);
  }
}

double BotHardwareInterface::GetWheelRadius() const {
  return wheel_radius_;
}

double BotHardwareInterface::GetMaxVelocity() const {
  return max_velocity_;
}

void BotHardwareInterface::LoadURDF(const ros::NodeHandle& nh,
                                    std::string param_name) {
  std::string urdf_string;
  urdf_model_ = std::make_unique<urdf::Model>();

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok()) {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name)) {
      ROS_INFO_STREAM_NAMED(
          name_, "Waiting for model URDF on the ROS param server at location: "
                     << nh.getNamespace() << search_param_name);
      nh.getParam(search_param_name, urdf_string);
    } else {
      ROS_INFO_STREAM_NAMED(
          name_, "Waiting for model URDF on the ROS param server at location: "
                     << nh.getNamespace() << param_name);
      nh.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }

  if (!urdf_model_->initString(urdf_string))
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
  else
    ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
}

double BotHardwareInterface::LinearToAngular(const double& distance) const {
  return distance / wheel_radius_;
}

double BotHardwareInterface::AngularToLinear(const double& angle) const {
  return angle * wheel_radius_;
}

LogHWInterface::LogHWInterface(ros::NodeHandle& node_handle)
    : BotHardwareInterface(node_handle) {}

double LogHWInterface::GetMotorAngle(size_t index) {
  ROS_INFO_STREAM("Get motor angle " << index);
  return 0;
}

void LogHWInterface::SetMotorVelocity(size_t index,
                                      double measured_value,
                                      double setpoint,
                                      const ros::Duration& dt) {
  ROS_INFO_STREAM("Set motor "
                  << index << " velocity: measured_value = " << measured_value
                  << " setpoint = " << setpoint << " duration = " << dt);
}
