#include "gpio_opi.h"

#include <ros/ros.h>
#include <wiringPi.h>
#include <cassert>

GPIO_OPI::GPIO_OPI() {
  wiringPiSetupGpio();
}

GPIO_OPI::~GPIO_OPI() {}

void GPIO_OPI::Output(const std::vector<size_t>& pins,
                      const std::vector<size_t>& values) {
  assert(pins.size() == values.size());

  size_t len = pins.size();
  for (size_t i = 0; i < len; ++i) {
    digitalWrite(pins[i], values[i] >= 1 ? HIGH : LOW);
  }
}

void GPIO_OPI::ConfigureOutputPints(const std::vector<size_t>& pins) {
  for (auto pin : pins) {
    pinMode(pin, OUTPUT);
  }
}
