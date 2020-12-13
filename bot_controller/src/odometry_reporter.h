#ifndef ODOMETRYREPORTER_H
#define ODOMETRYREPORTER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "bot_params.h"

class OdometryReporter {
 public:
  OdometryReporter(ros::NodeHandle& node_handler, const BotParams& params);
  ros::Time AddMeasurements(double right_revolutions, double left_revolutions);
  void Report(ros::Time current_time);

 private:
  void ReportTransform(ros::Time current_time);
  void ReportOdometry();

 private:
  ros::Time last_time;
  double dist_per_revolution = 0;  // in meters
  double wheels_base = 0;          // in meters

  double x = 0;
  double y = 0;
  double th = 0;

  double angular_velocity = 0;
  double linear_velocity = 0;
  geometry_msgs::Quaternion odometry_quaternion;

  ros::Publisher odometry_publisher;
  tf::TransformBroadcaster odometry_broadcaster;
};

#endif  // ODOMETRYREPORTER_H
