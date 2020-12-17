#include "byj_steppers_hw.h"

BYJSteppersHW::BYJSteppersHW() {}

double BYJSteppersHW::GetMotorAngle(size_t index) {}

void BYJSteppersHW::SetMotorVelocity(size_t index,
                                     double measured_value,
                                     double setpoint,
                                     const ros::Duration& dt) {}
