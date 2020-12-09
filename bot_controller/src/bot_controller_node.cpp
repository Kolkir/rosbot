#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "bot_controller.h"
#include "hardware_interface.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bot_controller");
  ros::NodeHandle node_handler;

#if defined(__arm__)
#else
  auto hardware_interface = std::make_unique<LoggingHardwareInterface>();
#endif

  BotController bot_controller(node_handler, std::move(hardware_interface));
  // the subsrciber object should be alive during the node life cycle
  auto cmd_vel_subscriber = node_handler.subscribe(
      "/cmd_vel", 10, &BotController::VelocityCallback, &bot_controller);

  while (node_handler.ok()) {
    ros::spin();
  }

  return 0;
}
