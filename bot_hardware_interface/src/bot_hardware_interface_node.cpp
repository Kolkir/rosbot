#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include "bot_hardware_interface.h"

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "bot_hardware_interface");
  ros::NodeHandle node_handle;

  std::unique_ptr<BotHardwareInterface> bot_hw =
      std::make_unique<LogHWInterface>(node_handle);
  assert(bot_hw);

  controller_manager::ControllerManager cm(bot_hw.get());

  // Setup a separate thread that will be used to service ROS callbacks.
  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(50.0);  // 50 Hz rate

  while (ros::ok()) {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    prev_time = time;

    bot_hw->read(time, period);
    cm.update(time, period);
    bot_hw->write(time, period);

    rate.sleep();
  }
  return 0;
}
