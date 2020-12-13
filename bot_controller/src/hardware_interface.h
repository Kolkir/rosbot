#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

#include <ros/ros.h>
#include <tuple>

class HardwareInterfaceBase {
 public:
  HardwareInterfaceBase() = default;
  virtual ~HardwareInterfaceBase() {}
  HardwareInterfaceBase(const HardwareInterfaceBase&) = delete;
  HardwareInterfaceBase& operator=(const HardwareInterfaceBase&) = delete;

  void SetLeftRPM(double rpm);
  void SetRightRPM(double rpm);

  double GetRightRPM() const;
  double GetLeftRPM() const;

  virtual std::tuple<double, double> GetWheelsRevolutions() = 0;

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

  std::tuple<double, double> GetWheelsRevolutions() override;

 private:
  void SetLeftRPMImpl(double rpm) override;
  void SetRightRPMImpl(double rpm) override;

 private:
  ros::Time last_reporting_time;
};

#endif  // HARDWAREINTERFACE_H
