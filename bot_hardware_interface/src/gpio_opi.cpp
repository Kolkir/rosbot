#include "gpio_opi.h"

#include <gpiod.h>
#include <ros/ros.h>
#include <cassert>

GPIO_OPI::GPIO_OPI() : chip_(nullptr, [](gpiod_chip*) {}) {
  auto* chip = gpiod_chip_open_by_name(gpio_chip_name_.data());
  if (!chip) {
    ROS_ERROR_STREAM("Failed to open GPIO chip : " << gpio_chip_name_);
    ros::shutdown();
  }
  chip_ = ChipPtr(chip, [](gpiod_chip* chip) { gpiod_chip_close(chip); });
}

GPIO_OPI::~GPIO_OPI() {}

void GPIO_OPI::Output(const std::vector<size_t>& pins,
                      const std::vector<size_t>& values) {
  assert(pins.size() == values.size());
  size_t len = pins.size();
  for (size_t i = 0; i < len; ++i) {
    auto pin_num = pins[i];
    auto ret = gpiod_line_set_value(pins_.at(pin_num).get(), values[i]);
    if (ret < 0) {
      ROS_ERROR_STREAM("Set line output failed\n");
      ros::shutdown();
    }
  }
}

void GPIO_OPI::ConfigureOutputPints(const std::vector<size_t>& pins) {
  pins_.clear();
  for (auto pin_num : pins) {
    auto* pin = gpiod_chip_get_line(chip_.get(), pin_num);
    if (!pin) {
      ROS_ERROR_STREAM("Failed to open GPIO pin  : " << pin_num);
      ros::shutdown();
    }
    pins_.emplace(
        pin_num, PinPtr(pin, [](gpiod_line* pin) { gpiod_line_release(pin); }));

    auto ret = gpiod_line_request_output(pin, "Consumer", 0);
    if (ret < 0) {
      ROS_ERROR_STREAM("Request GPIO pin " << pin_num << " as output failed\n");
      ros::shutdown();
    }
  }
}
