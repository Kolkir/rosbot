#ifndef BOTHARDWAREINTERFACE_H
#define BOTHARDWAREINTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <urdf/model.h>

class BotHardwareInterface : public hardware_interface::RobotHW {
 public:
  BotHardwareInterface(ros::NodeHandle& node_handle);
  ~BotHardwareInterface() override = default;
  BotHardwareInterface(const BotHardwareInterface&) = delete;
  BotHardwareInterface& operator=(const BotHardwareInterface&) = delete;

  /* Initialising a custom robot is done by registering joint handles
   * (\ref hardware_interface::ResourceManager::registerHandle) to hardware
   * interfaces that group similar joints and registering those individual
   * hardware interfaces with the class that represents the custom robot
   * (derived from this hardware_interface::RobotHW)
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /* Read data from the robot hardware
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /* Write commands to the robot hardware.
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  double GetWheelsBase() const;

  double GetWheelRadius() const;

  double LinearToAngular(const double& distance) const;

  double AngularToLinear(const double& angle) const;

 private:
  void LoadURDF(const ros::NodeHandle& nh, std::string param_name);

  virtual double GetMotorAngle(size_t index) = 0;

  virtual void SetMotorVelocity(size_t index,
                                double measured_value,
                                double setpoint,
                                const ros::Duration& dt) = 0;

 private:
  std::string name_;
  ros::NodeHandle node_handle_;

  // hardware_interface::JointStateInterface gives read access to all joint
  // values without conflicting with other controllers.
  hardware_interface::JointStateInterface joint_state_interface_;
  // hardware_interface::VelocityJointInterface inherits from
  // hardware_interface::JointCommandInterface and is used for reading and
  // writing joint velocities. Because this interface reserves the joints for
  // write access, conflicts with other controllers writing to the same joints
  // might occure. To only read joint velocities, avoid conflicts using
  // hardware_interface::JointStateInterface.
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  std::vector<std::string> joint_names_;
  std::size_t num_joints_ = 0;
  std::unique_ptr<urdf::Model> urdf_model_;

  double wheels_base_ = 0;
  double wheel_radius_ = 0.;

  // Data member array to store the controller commands which are sent to the
  // robot's resources (joints, actuators)
  // The diff_drive_controller uses the
  // hardware_interface::VelocityJointInterface It provides semantic meaning to
  // the wheel joints describing that they require velocity commands.
  std::vector<double> joint_velocity_commands_;

  // Data member arrays to store the state of the robot's resources (joints,
  // sensors) These values are filled in the read() method and are registered to
  // the joint_state_interface_ of type hardware_interface::JointStateInterface.
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
};

class LogHWInterface : public BotHardwareInterface {
 public:
  LogHWInterface(ros::NodeHandle& node_handle);

  // BotHardwareInterface interface
 private:
  double GetMotorAngle(size_t index) override;

  void SetMotorVelocity(size_t index,
                        double measured_value,
                        double setpoint,
                        const ros::Duration& dt) override;
};

#endif  // BOTHARDWAREINTERFACE_H
