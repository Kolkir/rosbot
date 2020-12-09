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

void LoggingHardwareInterface::SetLeftRPMImpl(double rpm) {
  ROS_INFO_STREAM("Bot left rpm = " << rpm);
}

void LoggingHardwareInterface::SetRightRPMImpl(double rpm) {
  ROS_INFO_STREAM("Bot right rpm = " << rpm);
}
