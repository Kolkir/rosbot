#ifndef BYJSTEPPERSHW_H
#define BYJSTEPPERSHW_H

#include "bot_hardware_interface.h"

// Differential drive Implementation with 28BYJ-48 stepper motors
class BYJSteppersHW : public BotHardwareInterface {
 public:
  BYJSteppersHW();

  // BotHardwareInterface interface
 private:
  double GetMotorAngle(size_t index);
  void SetMotorVelocity(size_t index,
                        double measured_value,
                        double setpoint,
                        const ros::Duration& dt);
};

#endif  // BYJSTEPPERSHW_H
