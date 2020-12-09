#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

class HardwareInterfaceBase {
 public:
  HardwareInterfaceBase() = default;
  virtual ~HardwareInterfaceBase() {}
  HardwareInterfaceBase(const HardwareInterfaceBase&) = delete;
  HardwareInterfaceBase& operator=(const HardwareInterfaceBase&) = delete;

  void SetLeftRPM(double rpm);
  void SetRightRPM(double rpm);

 private:
  virtual void SetLeftRPMImpl(double rpm) = 0;
  virtual void SetRightRPMImpl(double rpm) = 0;

 private:
  double left_rpm = 0.0;
  double right_rpm = 0.0;
};

class LoggingHardwareInterface : public HardwareInterfaceBase {
 public:
  using HardwareInterfaceBase::HardwareInterfaceBase;

 private:
  void SetLeftRPMImpl(double rpm) override;
  void SetRightRPMImpl(double rpm) override;
};

#endif  // HARDWAREINTERFACE_H
