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

class GPIO_Log : public GPIOBase {
 public:
  GPIO_Log();
  ~GPIO_Log() override;

  // GPIOBase interface
  void Output(const std::vector<size_t>& pins,
              const std::vector<size_t>& values) override;
  void ConfigureOutputPints(const std::vector<size_t>& pins) override;
};

#endif  // GPIOBASE_H
