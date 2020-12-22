#ifndef GPIO_OPI_H
#define GPIO_OPI_H

#include <memory>
#include <string>
#include <unordered_map>
#include "gpio_base.h"

struct gpiod_chip;
struct gpiod_line;

class GPIO_OPI : public GPIOBase {
 public:
  GPIO_OPI();
  ~GPIO_OPI() override;
  GPIO_OPI(const GPIO_OPI&) = delete;
  GPIO_OPI& operator=(const GPIO_OPI&) = delete;

  // GPIOBase interface
  void Output(const std::vector<size_t>& pins,
              const std::vector<size_t>& values) override;
  void ConfigureOutputPints(const std::vector<size_t>& pins) override;

 private:
  std::string gpio_chip_name_ = "gpiochip1";
  using ChipPtr = std::unique_ptr<gpiod_chip, void (*)(gpiod_chip*)>;
  ChipPtr chip_;
  using PinPtr = std::unique_ptr<gpiod_line, void (*)(gpiod_line*)>;
  std::unordered_map<size_t, PinPtr> pins_;
};

#endif  // GPIO_OPI_H
