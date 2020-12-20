#ifndef GPIO_OPI_H
#define GPIO_OPI_H

#include "gpio_base.h"

class GPIO_OPI : public GPIOBase {
 public:
  GPIO_OPI();
  GPIO_OPI(const GPIO_OPI&) = delete;
  GPIO_OPI& operator=(const GPIO_OPI&) = delete;

  ~GPIO_OPI() override;

  // GPIOBase interface
  void Output(const std::vector<size_t>& pins,
              const std::vector<size_t>& values) override;
  void ConfigureOutputPints(const std::vector<size_t>& pins) override;
};

#endif  // GPIO_OPI_H
