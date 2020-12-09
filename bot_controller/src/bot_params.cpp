#ifndef BOT_PARAMS_CPP
#define BOT_PARAMS_CPP

#include "bot_params.h"
#include <ros/ros.h>

BotParams ReadBotParams(ros::NodeHandle& node_handler) {
  BotParams params;
  // Read values
  if (!node_handler.param("/bot_controller/wheel_radius", params.wheel_radius,
                          0.07)) {
    ROS_ERROR_STREAM("Using default wheel radius param");
  } else {
    ROS_WARN_STREAM("Wheel radius param = " << params.wheel_radius);
  }

  if (!node_handler.param("/bot_controller/wheels_base", params.wheels_base,
                          0.10)) {
    ROS_ERROR_STREAM("Using default wheels base param");
  } else {
    ROS_WARN_STREAM("Wheels base param = " << params.wheels_base);
  }

  if (!node_handler.param("/bot_controller/max_rpm", params.max_rpm, 15.0)) {
    ROS_ERROR_STREAM("Using default max rpm param");
  } else {
    ROS_WARN_STREAM("Max RPM param = " << params.max_rpm);
  }

  // Check values
  if (params.wheel_radius < std::numeric_limits<double>::epsilon()) {
    ROS_ERROR_STREAM("Incorrect wheel radius value " << params.wheel_radius);
    exit(1);
  }

  if (params.max_rpm < std::numeric_limits<double>::epsilon()) {
    ROS_ERROR_STREAM("Incorrect max RPM value " << params.max_rpm);
    exit(1);
  }
  return params;
};

#endif  // BOT_PARAMS_CPP
