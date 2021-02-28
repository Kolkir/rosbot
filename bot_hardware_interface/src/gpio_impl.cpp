#include "gpio_impl.h"

#include <gpiod.h>
#include <ros/ros.h>
#include <cassert>

GPIOImpl::GPIOImpl(const std::string& chip_name)
    : gpio_chip_name_(chip_name)
    , chip_(nullptr, [](gpiod_chip*) {}) {
  auto* chip = gpiod_chip_open_by_name(gpio_chip_name_.data());
  if (!chip) {
    ROS_ERROR_STREAM("Failed to open GPIO chip : " << gpio_chip_name_);
    ros::shutdown();
  }
  chip_ = ChipPtr(chip, [](gpiod_chip* chip) { gpiod_chip_close(chip); });
}

GPIOImpl::~GPIOImpl() {}

void GPIOImpl::Output(const std::vector<size_t>& pins,
                      const std::vector<size_t>& values) {
  assert(pins.size() == values.size());
  if (pins_.empty()) {
    return;
  } 
  size_t len = pins.size();
  for (size_t i = 0; i < len; ++i) {
    try {
      auto pin_num = pins[i];
      auto ret = gpiod_line_set_value(pins_.at(pin_num).get(), values[i]);
      if (ret < 0) {
        ROS_ERROR_STREAM("Set pin" << pin_num << " output failed\n");
        ros::shutdown();
      }
    } catch(const std::exception& err) {
      ROS_ERROR_STREAM("Output pin " << pins[i] << " " << err.what());
      ros::shutdown();
    }
  }
}

void GPIOImpl::ConfigureOutputPints(const std::vector<size_t>& pins) {
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
    sleep(1);
    ret = gpiod_line_set_value(pin, 1);
    sleep(1);
    ret = gpiod_line_set_value(pin, 0);
    sleep(1);
    if (ret < 0) {
      ROS_ERROR_STREAM("Set line output failed " << pin_num);
      ros::shutdown();
    }
    ROS_INFO_STREAM("Pin set " << pin_num);
  }
}
