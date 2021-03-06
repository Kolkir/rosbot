#ifndef BYJSTEPPERSHW_H
#define BYJSTEPPERSHW_H

#include <atomic>
#include "bot_hardware_interface.h"
#include "gpio_base.h"

// Differential drive Implementation with 28BYJ-48 stepper motors

class BYJStepper {
 public:
  enum Direction { CW, CCW };

  BYJStepper(ros::NodeHandle& node_handle,
             std::shared_ptr<GPIOBase> gpio,
             std::vector<size_t> pins,
             Direction dir);
  BYJStepper(const BYJStepper&) = delete;
  BYJStepper& operator=(const BYJStepper&) = delete;

  void SetRPM(double rpm);

  void SetOriginalDirection();

  void SetOpositeDirection();

  double GetAngleAccumulated() const;

 private:
  void HWUpdate(const ros::TimerEvent& event);
  void Stop();
  static Direction GetOpositeDirection(Direction dir);

 private:
  std::shared_ptr<GPIOBase> gpio_;
  ros::Timer timer_;
  ros::Duration timeout_{1.0};  // seconds
  size_t halfstep_ = 0;
  std::vector<size_t> pins_;
  Direction direction_ = CW;
  Direction original_direction_ = CW;
  std::atomic<double> angle_accumulated_;
};

class BYJSteppersHW : public BotHardwareInterface {
 public:
  BYJSteppersHW(ros::NodeHandle& node_handle);

  // BotHardwareInterface interface
 private:
  double GetMotorAngle(size_t index) override;
  void SetMotorVelocity(size_t index,
                        double measured_value,
                        double setpoint,
                        const ros::Duration& dt) override;

 private:
  void SetLinearVelocity(size_t index, double value);

 private:
  std::shared_ptr<GPIOBase> gpio_;
  std::unique_ptr<BYJStepper> left_motor_;
  std::unique_ptr<BYJStepper> right_motor_;

  double rpm_multiplier_ = 0.0;  // the length of a wheel used to calculate RPM
};

#endif  // BYJSTEPPERSHW_H
