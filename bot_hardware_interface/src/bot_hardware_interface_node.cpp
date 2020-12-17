#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include "bot_hardware_interface.h"
#include "byj_steppers_hw.h"

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "bot_hardware_interface");
  ros::NodeHandle node_handle;

  std::string hw_backend;
  std::size_t error =
      !rosparam_shortcuts::get("rosbot", node_handle, "hw_backend", hw_backend);
  rosparam_shortcuts::shutdownIfError("rosbot", error);

  std::unique_ptr<BotHardwareInterface> bot_hw;
  if (hw_backend == "byj") {
    bot_hw = std::make_unique<BYJSteppersHW>(node_handle);
  } else {
    bot_hw = std::make_unique<LogHWInterface>(node_handle);
  }
  assert(bot_hw);

  controller_manager::ControllerManager cm(bot_hw.get());

  const int num_threads = 3;  // 1 - for general callbacks, 2 - for wheel timers

  // Setup a separate thread that will be used to service ROS callbacks.
  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(num_threads);
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
