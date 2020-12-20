#ifndef GPIOBASE_H
#define GPIOBASE_H

#include <vector>

class GPIOBase {
 public:
  virtual ~GPIOBase() {}
  virtual void Output(const std::vector<size_t>& pins,
                      const std::vector<size_t>& values) = 0;
  virtual void ConfigureOutputPints(const std::vector<size_t>& pins) = 0;
};

#endif  // GPIOBASE_H
