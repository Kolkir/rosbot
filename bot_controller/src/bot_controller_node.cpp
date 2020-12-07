#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class BotController {
 public:
  void VelocityCallback(const geometry_msgs::Twist& twist_msg) {
    ROS_WARN_STREAM("Got x vel = " << twist_msg.linear.x);
    ROS_WARN_STREAM("Got ang vel = " << twist_msg.angular.z);
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bot_controller");
  ros::NodeHandle node_handler;

  BotController bot_controller;
  // the subsrciber object should be alive during node life cycle
  auto cmd_vel_subscriber = node_handler.subscribe(
      "/cmd_vel", 10, &BotController::VelocityCallback, &bot_controller);

  while (node_handler.ok()) {
    ros::spin();
  }

  return 0;
}
