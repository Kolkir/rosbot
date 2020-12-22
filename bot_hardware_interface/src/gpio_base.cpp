#include "gpio_base.h"
#include <ros/ros.h>

GPIO_Log::GPIO_Log() {
  ROS_INFO_STREAM("GPIO initialized");
}

GPIO_Log::~GPIO_Log() {
  ROS_INFO_STREAM("GPIO closed");
}

void GPIO_Log::Output(const std::vector<size_t>& pins,
                      const std::vector<size_t>& values) {
  std::string msg;
  size_t len = pins.size();
  for (size_t i = 0; i < len; ++i) {
    msg += std::to_string(pins[i]);
    msg += " = ";
    msg += std::to_string(values[i]);
    msg += "; ";
  }
  ROS_INFO_STREAM("GPIO output " << msg);
}

void GPIO_Log::ConfigureOutputPints(const std::vector<size_t>& pins) {
  std::string msg;
  size_t len = pins.size();
  for (size_t i = 0; i < len; ++i) {
    msg += std::to_string(pins[i]);
    msg += "; ";
  }
  ROS_INFO_STREAM("GPIO configure output " << msg);
}
