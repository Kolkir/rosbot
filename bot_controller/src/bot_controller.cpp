#include "bot_controller.h"

#include <ros/ros.h>

const double RPM_TO_RAD_PER_SECOND = 0.1047;

BotController::BotController(
    ros::NodeHandle& node_handler,
    std::unique_ptr<HardwareInterfaceBase> hardware_interface)
    : hardware_interface(std::move(hardware_interface)) {
  params = ReadBotParams(node_handler);
  rpm_denominator = RPM_TO_RAD_PER_SECOND * params.wheel_radius;
}

void BotController::VelocityCallback(const geometry_msgs::Twist& twist_msg) {
  // ROS_INFO_STREAM("VelocityCallback :\n" << twist_msg);

  CalculateLeftWheelRPM(twist_msg.linear.x, twist_msg.angular.z);
  CalculateRightWheelRPM(twist_msg.linear.x, twist_msg.angular.z);

  if (hardware_interface) {
    hardware_interface->SetLeftRPM(left_rpm);
    hardware_interface->SetRightRPM(right_rpm);
  } else {
    ROS_WARN_STREAM("Bot controller missing hardware interface");
  }
}

void BotController::CalculateLeftWheelRPM(double linear_velocity,
                                          double angular_velocity) {
  left_rpm = (linear_velocity - (angular_velocity * params.wheels_base) / 2.0) /
             rpm_denominator;
  left_rpm = ScaleRPM(left_rpm);
}

void BotController::CalculateRightWheelRPM(double linear_velocity,
                                           double angular_velocity) {
  right_rpm =
      (linear_velocity + (angular_velocity * params.wheels_base) / 2.0) /
      rpm_denominator;
  right_rpm = ScaleRPM(right_rpm);
}

double BotController::ScaleRPM(double rpm) {
  if (rpm > params.max_rpm)
    return params.max_rpm;
  if (rpm < -params.max_rpm)
    return 0.0;
  return rpm;
}
