#include "hardware_interface.h"
#include <ros/ros.h>

void HardwareInterfaceBase::SetLeftRPM(double rpm) {
  if (rpm != left_rpm) {
    SetLeftRPMImpl(rpm);
    left_rpm = rpm;
  }
}

void HardwareInterfaceBase::SetRightRPM(double rpm) {
  if (rpm != right_rpm) {
    SetRightRPMImpl(rpm);
    right_rpm = rpm;
  }
}

double HardwareInterfaceBase::GetRightRPM() const {
  return right_rpm;
}

double HardwareInterfaceBase::GetLeftRPM() const {
  return left_rpm;
}

void LoggingHardwareInterface::SetLeftRPMImpl(double rpm) {
  ROS_INFO_STREAM("Bot left rpm = " << rpm);
}

void LoggingHardwareInterface::SetRightRPMImpl(double rpm) {
  ROS_INFO_STREAM("Bot right rpm = " << rpm);
}

std::tuple<double, double> LoggingHardwareInterface::GetWheelsRevolutions() {
  auto current_time = ros::Time::now();
  auto dt = current_time - last_reporting_time;

  auto dt_num = dt.toSec() / 60.0;
  auto right_revolutions = dt_num * GetRightRPM();
  auto left_revolutions = dt_num * GetLeftRPM();

  last_reporting_time = current_time;

  return {right_revolutions, left_revolutions};
}
