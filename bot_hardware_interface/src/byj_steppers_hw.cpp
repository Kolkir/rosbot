#include "byj_steppers_hw.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include "gpio_opi.h"

static std::vector<std::vector<size_t>> halfstep_seq = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}};

static size_t half_step_ticks_count = 4076;

static double angle_per_tick = 0.003067962;  // radians

BYJSteppersHW::BYJSteppersHW(ros::NodeHandle& node_handle)
    : BotHardwareInterface(node_handle) {
  rpm_multiplier_ = GetWheelRadius() * 2 * M_PI;

  size_t error = 0;
  std::vector<double> left_pins;
  error += !rosparam_shortcuts::get("rosbot", node_handle, "left_motor_pins",
                                    left_pins);
  std::vector<double> right_pins;
  error += !rosparam_shortcuts::get("rosbot", node_handle, "right_motor_pins",
                                    right_pins);
  rosparam_shortcuts::shutdownIfError("rosbot", error);

  std::vector<size_t> left_control_pins(left_pins.begin(), left_pins.end());
  std::vector<size_t> right_control_pins(right_pins.begin(), right_pins.end());

  gpio_ = std::make_shared<GPIO_OPI>();
  // gpio_ = std::make_shared<GPIO_Log>();
  left_motor_ = std::make_unique<BYJStepper>(node_handle, gpio_,
                                             left_control_pins, BYJStepper::CW);
  right_motor_ = std::make_unique<BYJStepper>(
      node_handle, gpio_, right_control_pins, BYJStepper::CCW);
}

double BYJSteppersHW::GetMotorAngle(size_t index) {
  switch (index) {
    case 0:
      return left_motor_->GetAngle();
      break;
    case 1:
      return right_motor_->GetAngle();
      break;
    default:
      ROS_ERROR_STREAM(
          "BYJSteppersHW::GetMotorAngle failed with incorrect motor index "
          "= "
          << index);
      ros::shutdown();
      exit(1);
  }
  return 0;
}

void BYJSteppersHW::SetMotorVelocity(size_t index,
                                     double /*measured_value*/,
                                     double setpoint,
                                     const ros::Duration& /*dt*/) {
  // joint velocity commands from ros_control's RobotHW are in rad/s
  auto linear_velocity = AngularToLinear(setpoint);  // meter/s

  //  ROS_INFO_STREAM("SetMotorVelocity index = " << index
  //                                              << " value = " << setpoint
  //                                              << " dur = " << dt.toSec());
  SetLinearVelocity(index, linear_velocity);
}

void BYJSteppersHW::SetLinearVelocity(size_t index, double value) {
  auto rpm = fabs(value / rpm_multiplier_) * 60.0;

  //  ROS_INFO_STREAM("Linear velocity index = "
  //                  << index << " value = " << value << " rpm = " << rpm
  //                  << " timeout = " << seconds_timeout.toSec());

  switch (index) {
    case 0:
      value >= 0.0 ? left_motor_->SetOriginalDirection()
                   : left_motor_->SetOpositeDirection();
      left_motor_->SetRPM(rpm);
      break;
    case 1:
      value >= 0.0 ? right_motor_->SetOriginalDirection()
                   : right_motor_->SetOpositeDirection();
      right_motor_->SetRPM(rpm);
      break;
    default:
      ROS_ERROR_STREAM(
          "BYJSteppersHW::SetLinearVelocity failed with incorrect motor index "
          "= "
          << index);
      ros::shutdown();
      exit(1);
  }
}

BYJStepper::BYJStepper(ros::NodeHandle& node_handle,
                       std::shared_ptr<GPIOBase> gpio,
                       std::vector<size_t> pins,
                       Direction dir)
    : gpio_(gpio), ticks_(0), pins_(pins), original_direction_(dir) {
  if (pins_.size() != 4) {
    ROS_ERROR_STREAM("Incorrect number of pins for a motor");
    ros::shutdown();
    exit(1);
  }
  gpio_->ConfigureOutputPints(pins_);
  timer_ =
      node_handle.createTimer(ros::Duration(timeout_), &BYJStepper::HWUpdate,
                              this, /*oneshot*/ false, /*autostart*/ false);
}

void BYJStepper::SetRPM(double rpm) {
  auto seconds_timeout = ros::Duration();
  if (rpm > 0) {
    seconds_timeout = ros::Duration(60.0 / (rpm * steps_per_rotation_));
  }
  timeout_ = seconds_timeout;
  const double one_millisec = 0.001;
  if (timeout_.toSec() < one_millisec) {
    timer_.stop();
    gpio_->Output(pins_, {0, 0, 0, 0});
  } else {
    timer_.setPeriod(timeout_);
    timer_.start();
  }
}

void BYJStepper::SetOriginalDirection() {
  direction_ = original_direction_;
}

void BYJStepper::SetOpositeDirection() {
  direction_ = GetOpositeDirection(original_direction_);
}

void BYJStepper::HWUpdate(const ros::TimerEvent& /*event*/) {
  gpio_->Output(pins_, halfstep_seq[halfstep_]);

  if (direction_ == Direction::CW) {
    halfstep_ += 1;
    if (halfstep_ >= halfstep_seq.size()) {
      halfstep_ = 0;
    }
  } else {
    if (halfstep_ == 0) {
      halfstep_ = halfstep_seq.size() - 1;
    } else {
      halfstep_ -= 1;
    }
  }

  ticks_ += 1;
  if (ticks_.load() >= half_step_ticks_count) {
    ticks_ = 0;
  }
}

BYJStepper::Direction BYJStepper::GetOpositeDirection(
    BYJStepper::Direction dir) {
  switch (dir) {
    case CW:
      return CCW;
    case CCW:
      return CW;
    default:
      ROS_ERROR_STREAM("Incorrect motor direction " << dir);
      ros::shutdown();
  }
  return CW;
}

double BYJStepper::GetAngle() const {
  auto ticks = ticks_.load();
  auto angle = ticks * angle_per_tick;

  // normalize
  angle = fmod(angle, 2.0 * M_PI);

  if (angle < 0)
    angle += 2.0 * M_PI;

  return angle;
}
