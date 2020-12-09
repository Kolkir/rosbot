#ifndef BOTCONTROLLER_H
#define BOTCONTROLLER_H

#include <geometry_msgs/Twist.h>
#include "bot_params.h"
#include "hardware_interface.h"

namespace ros {
class NodeHandle;
}

class BotController {
 public:
  BotController(ros::NodeHandle& node_handler,
                std::unique_ptr<HardwareInterfaceBase> hardware_interface);

  BotController(const BotController&) = delete;
  BotController& operator=(const BotController&) = delete;

  void VelocityCallback(const geometry_msgs::Twist& twist_msg);

 private:
  void CalculateLeftWheelRPM(double linear_velocity, double angular_velocity);

  void CalculateRightWheelRPM(double linear_velocity, double angular_velocity);

  double ScaleRPM(double rpm);

 private:
  BotParams params;
  double left_rpm = 0.0;
  double right_rpm = 0.0;
  double rpm_denominator = 0.0;
  std::unique_ptr<HardwareInterfaceBase> hardware_interface;
};

#endif  // BOTCONTROLLER_H
