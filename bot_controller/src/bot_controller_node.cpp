#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "bot_controller.h"
#include "hardware_interface.h"
#include "odometry_reporter.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bot_controller");
  ros::NodeHandle node_handler;

#if defined(__arm__)
#else
  auto hardware_interface = std::make_unique<LoggingHardwareInterface>();
#endif

  BotController bot_controller(node_handler, std::move(hardware_interface));
  OdometryReporter odometry_reporter(node_handler, bot_controller.GetParams());
  // the subsrciber object should be alive during the node life cycle
  auto cmd_vel_subscriber = node_handler.subscribe(
      "/cmd_vel", 10, &BotController::VelocityCallback, &bot_controller);

  ros::Rate rate(1.0);
  while (node_handler.ok()) {
    ros::spinOnce();

    auto revolutions = bot_controller.GetWheelsRevolutions();
    auto current_time = odometry_reporter.AddMeasurements(
        std::get<0>(revolutions), std::get<1>(revolutions));
    odometry_reporter.Report(current_time);
    rate.sleep();
  }

  return 0;
}
