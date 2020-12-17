#ifndef BYJSTEPPERSHW_H
#define BYJSTEPPERSHW_H

#include <atomic>
#include "bot_hardware_interface.h"

// Differential drive Implementation with 28BYJ-48 stepper motors

class BYJStepper {
 public:
  BYJStepper(ros::NodeHandle& node_handle, std::vector<size_t> pins);
  BYJStepper(const BYJStepper&) = delete;
  BYJStepper& operator=(const BYJStepper&) = delete;

  void SetTimeout(ros::Duration timeout);

  double GetAngle();

 private:
  void HWUpdate(const ros::TimerEvent& event);

 private:
  ros::Timer timer_;
  ros::Duration timeout_{1.0};  // seconds
  size_t halfstep_ = 0;
  std::atomic<size_t> ticks_;
  std::vector<size_t> pins_;
};

class BYJSteppersHW : public BotHardwareInterface {
 public:
  BYJSteppersHW(ros::NodeHandle& node_handle);

  // BotHardwareInterface interface
 private:
  double GetMotorAngle(size_t index);
  void SetMotorVelocity(size_t index,
                        double measured_value,
                        double setpoint,
                        const ros::Duration& dt);

 private:
  void SetLinearVelocity(size_t index, double value);

 private:
  std::unique_ptr<BYJStepper> left_motor_;
  std::unique_ptr<BYJStepper> right_motor_;

  double rpm_multiplier_ = 0.0;  // the length of a wheel used to calculate RPM

  double const steps_per_rotation_ = 4076;  // in the half-step mode
};

#endif  // BYJSTEPPERSHW_H
